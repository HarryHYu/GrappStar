"""Camera: a background thread that keeps the newest frame of one video source.

    cam = Camera("dart", lambda: V4L2Source("/dev/grappstar-dartcam", "YUYV", 720, 480, 30))
    cam.latest()            # newest Frame or None
    cam.jpeg()              # newest frame as JPEG bytes (for the MJPEG web stream)
    cam.buffer(10)          # frames from the last 10 s (ring buffer)
    cam.hold(t_from)        # keep every frame from t_from on until release_hold() (the flight)
    cam.start_recording("runs/flight.avi"); ...; cam.stop_recording()
    cam.status()            # dict for the UI
    cam.stop()

source_factory() must return a FrameSource (see INTERFACES.md). If it raises,
returns None, or the source stops giving frames, the thread closes it and calls
the factory again, waiting a little longer each time (backoff) so a missing
camera does not spin the CPU. When frames flow again the wait resets.

Ring buffer sizing (review R1-C #4): the ring keeps buffer_s seconds. Its RAM cap follows the time window: by
default (buffer_max_mb=None) it is buffer_s x the measured frame rate x the frame size x 1.25, at least
BUFFER_MAX_MB and at most RAM_HARD_MAX_MB. 720x480 BGR at 30 fps is 31 MB/s, so 15 s needs about 470 MB.
A hold (hold(t_from), set at fire time from 2 s before the launch) pins the flight frames: they are never dropped
by age or count. When the RAM cap is reached the oldest held frames are JPEG-compressed in memory (about 10x
smaller; buffer() decodes them back). A very long hold (an operator prompt with no timeout) thins the packed
frames AFTER the first HOLD_KEEP_HEAD_S of the hold, so the launch and flight frames are always kept. Read with
buffer(since=t) so frames before t are never decoded.
release_hold() ends it (a hold also ends by itself after hold_max_s).
"""
import collections
import threading
import time
import traceback
from typing import Callable, Optional

import cv2

from .deinterlace import deinterlace as _deinterlace, normalize_mode
from .frame import Frame
from .quality import frame_quality, BAD_BELOW
from .record import Recorder

STATS_WINDOW_S = 2.0      # fps and bad fraction are measured over this window
MAX_BUFFER_FRAMES = 4000  # hard cap so a fast camera cannot eat all the RAM
BUFFER_MAX_MB = 400.0     # minimum ring RAM cap in MB of image data (720x480 BGR is about 1 MB per frame)
RAM_HARD_MAX_MB = 1200.0  # automatic cap never goes above this (Orin Nano: 8 GB shared with the GPU)
HELD_MAX_MB = 120.0       # packed held frames above this are thinned (about 1200 to 1800 frames of 720x480)
HOLD_KEEP_HEAD_S = 8.0    # never thinned: from the hold start (2 s before launch) through launch, flight, landing
HOLD_MAX_S = 300.0        # a hold nobody released ends by itself after this
PACK_JPEG_QUALITY = 92


class _Packed:
    """A JPEG-compressed frame in the ring (held frames over the RAM cap). Decoded back by unpack()."""
    __slots__ = ("data", "t", "idx", "quality", "bad")

    def __init__(self, f):
        ok, buf = cv2.imencode(".jpg", f.img, [cv2.IMWRITE_JPEG_QUALITY, PACK_JPEG_QUALITY])
        if not ok:
            raise ValueError("jpeg encode failed")
        self.data = buf.tobytes()
        self.t, self.idx, self.quality, self.bad = f.t, f.idx, f.quality, f.bad

    @property
    def nbytes(self) -> int:
        return len(self.data)

    def unpack(self) -> Frame:
        import numpy as np
        img = cv2.imdecode(np.frombuffer(self.data, np.uint8), cv2.IMREAD_COLOR)
        return Frame(img=img, t=self.t, idx=self.idx, quality=self.quality, bad=self.bad)


def _unpack(f):
    return f.unpack() if isinstance(f, _Packed) else f


def _nbytes(f) -> int:
    if isinstance(f, _Packed):
        return f.nbytes
    img = getattr(f, "img", None)
    return int(getattr(img, "nbytes", 0) or 0)


class Camera:
    def __init__(self, name: str, source_factory: Callable, quality: bool = True,
                 deinterlace="none", buffer_s: float = 20.0, jpeg_quality: int = 80,
                 max_fps: Optional[float] = None, start: bool = True,
                 backoff_min: float = 0.5, backoff_max: float = 5.0,
                 stall_s: float = 3.0, none_limit: int = 100, buffer_max_mb: Optional[float] = None,
                 hold_max_s: float = HOLD_MAX_S):
        self.name = name
        self.source_factory = source_factory
        self.quality_enabled = bool(quality)
        self.deinterlace_mode = normalize_mode(deinterlace)
        self.buffer_s = float(buffer_s)
        # None = automatic (follows buffer_s and the frame rate); a number = fixed cap in MB
        self.buffer_auto = buffer_max_mb is None
        self.buffer_max_bytes = int(float(BUFFER_MAX_MB if buffer_max_mb is None else buffer_max_mb) * 1024 * 1024)
        self.hold_max_s = float(hold_max_s)
        self.jpeg_quality = int(jpeg_quality)
        self.max_fps = max_fps
        self.backoff_min = backoff_min
        self.backoff_max = backoff_max
        self.stall_s = stall_s          # no frame for this long -> reconnect
        self.none_limit = none_limit    # this many read() == None in a row -> reconnect

        self._lock = threading.Lock()
        self._new_frame = threading.Condition(self._lock)
        self._jpeg_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._src = None
        self._src_lock = threading.Lock()   # guards self._src swaps (worker vs. stop / reconnect)

        # State below is protected by self._lock.
        self._latest = None             # newest Frame
        self._latest_q = None           # its full quality dict
        self._prev_img = None           # previous image, for freeze detection
        self._ring = collections.deque()   # (receive time, Frame)
        self._ring_bytes = 0            # image bytes held by the ring
        self._hold_t = None             # frames with f.t >= this are pinned (None = no hold)
        self._hold_set_at = None        # receive time of the first frame after hold() (expiry clock)
        self._n_packed = 0              # packed (JPEG) entries at the front of the ring
        self._packed_bytes = 0
        self._dropped_held = 0          # held frames lost anyway (HELD_MAX_MB reached)
        self._stats = collections.deque()  # (receive time, bad) over STATS_WINDOW_S
        self._last_rx = 0.0             # time.time() of the newest frame
        self._next_due = 0.0            # max_fps schedule: the next frame is kept from about this time
        self._frames = 0
        self._connected = False
        self._source_info = None
        self._errors = 0
        self._last_error = None
        self._reconnects = 0
        self._recorder = None
        self._jpeg_cache = (None, None)  # (Frame, bytes): encoded lazily, once per frame
        if start:
            self.start()

    # ------------------------------------------------------------ lifecycle

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="cam-" + self.name, daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 3.0) -> None:
        """Signal the worker and wait for it. The worker closes its own source on exit, so a source is never
        closed under a read() still running in the thread (a stuck read just finishes later and then closes)."""
        self._stop.set()
        with self._lock:
            self._new_frame.notify_all()
        t = self._thread
        if t is not None:
            t.join(timeout)
        if t is None or not t.is_alive():
            self._close_source()        # no worker (never started, or finished): nothing else will close it
        self.stop_recording()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.stop()

    # ------------------------------------------------------------ worker thread

    def _error(self, msg: str) -> None:
        with self._lock:
            self._errors += 1
            self._last_error = msg

    def _close_source(self) -> None:
        with self._src_lock:
            src, self._src = self._src, None
        if src is not None:
            try:
                src.close()
            except Exception:
                pass
        with self._lock:
            self._connected = False

    def _run(self) -> None:
        try:
            self._loop()
        finally:
            self._close_source()        # the worker owns its source: close it here, not from stop()

    def _loop(self) -> None:
        backoff = self.backoff_min
        nones = 0
        opened_at = 0.0
        while not self._stop.is_set():
            # 1. Open the source if we do not have one.
            with self._src_lock:
                src = self._src
            if src is None:
                try:
                    src = self.source_factory()
                    if src is None:
                        raise RuntimeError("source factory returned None")
                except Exception as e:
                    self._error("open: %s" % e)
                    self._stop.wait(backoff)
                    backoff = min(self.backoff_max, backoff * 2)
                    continue
                with self._src_lock:
                    self._src = src
                if self._stop.is_set():
                    break
                nones = 0
                opened_at = time.time()
                with self._lock:
                    self._source_info = dict(getattr(src, "info", {}) or {})

            # 2. Read one frame.
            try:
                f = src.read()
            except Exception as e:
                self._error("read: %s" % e)
                self._reconnect()
                self._stop.wait(backoff)
                backoff = min(self.backoff_max, backoff * 2)
                continue

            if f is None:
                nones += 1
                last = max(self._last_rx, opened_at)
                if nones >= self.none_limit or time.time() - last > self.stall_s:
                    self._error("no frames from source, reconnecting")
                    self._reconnect()
                    self._stop.wait(backoff)
                    backoff = min(self.backoff_max, backoff * 2)
                else:
                    self._stop.wait(0.005)
                continue

            # 3. Got a frame: healthy again.
            nones = 0
            backoff = self.backoff_min
            try:
                self._handle(f)
            except Exception:
                self._error("process: " + traceback.format_exc(limit=2))

    def _reconnect(self) -> None:
        self._close_source()
        with self._lock:
            self._reconnects += 1

    def _handle(self, f: Frame, now: Optional[float] = None) -> None:
        now = time.time() if now is None else now
        if self.max_fps:
            # schedule, not a minimum gap: with max_fps equal to the source rate a few ms of jitter must not drop
            # frames. Keep a frame from 20 % of a period before it is due; the next is due one period later
            # (from now when we fell behind, so a stall does not cause a burst).
            period = 1.0 / float(self.max_fps)
            if now < self._next_due - 0.2 * period:
                return                  # throttled: drop this frame
            self._next_due = max(self._next_due + period, now)
        if self.deinterlace_mode != "none":
            f.img = _deinterlace(f.img, self.deinterlace_mode)
        q = None
        if self.quality_enabled:
            q = frame_quality(f.img, self._prev_img)
            f.quality = q["score"]
            f.bad = q["score"] < BAD_BELOW
        rec = None
        with self._lock:
            self._prev_img = f.img
            self._latest = f
            self._latest_q = q
            self._last_rx = now
            self._frames += 1
            self._connected = True
            self._ring.append((now, f))
            self._ring_bytes += _nbytes(f)
            self._stats.append((now, f.bad))
            while self._stats and now - self._stats[0][0] > STATS_WINDOW_S:
                self._stats.popleft()
            self._trim_locked(now, _nbytes(f))
            rec = self._recorder
            self._new_frame.notify_all()
        if rec is not None:
            try:
                rec.write(f)
            except Exception as e:
                self._error("record: %s" % e)

    def _cap_locked(self, frame_bytes: int) -> int:
        if not self.buffer_auto:
            return self.buffer_max_bytes
        fps = self._fps_locked() or 30.0
        need = int(self.buffer_s * min(fps, 120.0) * frame_bytes * 1.25)
        return max(self.buffer_max_bytes, min(need, int(RAM_HARD_MAX_MB * 1024 * 1024)))

    def _held_locked(self, f) -> bool:
        return self._hold_t is not None and f.t >= self._hold_t

    def _pop_locked(self) -> None:
        _, f = self._ring.popleft()
        n = _nbytes(f)
        self._ring_bytes -= n
        if self._n_packed:
            self._n_packed -= 1
            self._packed_bytes -= n

    def _trim_locked(self, now: float, frame_bytes: int) -> None:
        if self._hold_t is not None and self._hold_set_at is None:
            self._hold_set_at = now                     # measured on the frame clock from the first frame after hold()
        if self._hold_t is not None and now - self._hold_set_at > self.hold_max_s:
            self._hold_t = None                         # forgotten hold: end it
        cap = self._cap_locked(frame_bytes)
        ring = self._ring
        # 1. age, count and RAM limits for frames that are not held (the oldest come first)
        while len(ring) > 1 and not self._held_locked(ring[0][1]) and (
                now - ring[0][0] > self.buffer_s or len(ring) > MAX_BUFFER_FRAMES or self._ring_bytes > cap):
            self._pop_locked()
        if self._hold_t is None:
            if self._n_packed:      # no hold: packed frames are only kept while they are in the time window
                while self._n_packed and len(ring) > 1 and now - ring[0][0] > self.buffer_s:
                    self._pop_locked()
            return
        # 2. held frames over the RAM cap: compress the oldest ones in place
        while self._ring_bytes > cap and self._n_packed < len(ring) - 1:
            i = self._n_packed
            t_rx, f = ring[i]
            try:
                p = _Packed(f)
            except Exception:
                break
            ring[i] = (t_rx, p)
            self._ring_bytes += p.nbytes - _nbytes(f)
            self._packed_bytes += p.nbytes
            self._n_packed += 1
        # 3. last resort (a very long hold): drop the oldest packed frames AFTER the protected head (hold start to
        # hold start + HOLD_KEEP_HEAD_S: launch, flight and landing). The newest frames stay unpacked anyway.
        cut = self._hold_t + HOLD_KEEP_HEAD_S
        i = 0
        while self._packed_bytes > HELD_MAX_MB * 1024 * 1024 and i < self._n_packed:
            f = ring[i][1]
            if f.t < cut:
                i += 1
                continue
            del ring[i]
            n = _nbytes(f)
            self._ring_bytes -= n
            self._packed_bytes -= n
            self._n_packed -= 1
            self._dropped_held += 1

    # ------------------------------------------------------------ public API

    def hold(self, t_from: Optional[float] = None) -> None:
        """Pin every frame with f.t >= t_from (default: now) until release_hold(): the flight frames.

        Frames already in the ring from t_from on are kept too. Set it at fire time with t_fire - 2 s."""
        with self._lock:
            self._hold_t = time.time() if t_from is None else float(t_from)
            self._hold_set_at = None

    def release_hold(self) -> None:
        """End the hold: the ring goes back to its normal time window (trimmed on the next frame)."""
        with self._lock:
            self._hold_t = None

    def held(self) -> list:
        """The held frames (f.t >= the hold time), decoded, oldest first. [] without a hold."""
        with self._lock:
            if self._hold_t is None:
                return []
            items = [f for _, f in self._ring if f.t >= self._hold_t]
        return [_unpack(f) for f in items]

    def latest(self) -> Optional[Frame]:
        with self._lock:
            return self._latest

    def wait_frame(self, timeout: float = 2.0, after_idx: Optional[int] = None) -> Optional[Frame]:
        """Block until a frame arrives (newer than after_idx frames counted, if given)."""
        end = time.time() + timeout
        with self._lock:
            start_count = self._frames if after_idx is None else after_idx
            while self._frames <= start_count and not self._stop.is_set():
                left = end - time.time()
                if left <= 0:
                    return None
                self._new_frame.wait(left)
            return self._latest

    def jpeg(self, quality: Optional[int] = None) -> Optional[bytes]:
        """Latest frame as JPEG bytes. Encoded only when asked, and only once per frame,
        so several web viewers share one encode."""
        f = self.latest()
        if f is None:
            return None
        with self._jpeg_lock:
            cached_frame, data = self._jpeg_cache
            if cached_frame is f and quality is None:
                return data
            ok, buf = cv2.imencode(".jpg", f.img,
                                   [cv2.IMWRITE_JPEG_QUALITY, int(quality or self.jpeg_quality)])
            if not ok:
                return None
            data = buf.tobytes()
            if quality is None:
                self._jpeg_cache = (f, data)
            return data

    def buffer(self, seconds: Optional[float] = None, since: Optional[float] = None) -> list:
        """Frames received in the last `seconds` before the newest frame (all if None).
        Measured from the newest frame, so it still works after the link drops.
        since: only frames with f.t >= since (filtered BEFORE packed frames are decoded: use it after a hold)."""
        with self._lock:
            if not self._ring:
                return []
            if seconds is None:
                items = [f for _, f in self._ring]
            else:
                cutoff = self._ring[-1][0] - seconds
                items = [f for t, f in self._ring if t >= cutoff]
        if since is not None:
            items = [f for f in items if f.t >= since]
        return [_unpack(f) for f in items]       # packed (held) frames are decoded outside the lock

    def start_recording(self, path: str, fps: Optional[float] = None, **kw) -> str:
        """Start writing frames to path (AVI + sidecar). Returns the path."""
        self.stop_recording()
        rec = Recorder(path, fps=fps or self._measured_fps() or 30.0, **kw)
        with self._lock:
            self._recorder = rec
        return path

    def stop_recording(self) -> Optional[str]:
        """Stop recording; returns the path written (the JPEG folder in fallback mode)."""
        with self._lock:
            rec, self._recorder = self._recorder, None
        if rec is None:
            return None
        rec.close()
        return rec.path

    def _measured_fps(self) -> float:
        with self._lock:
            return self._fps_locked()

    def _fps_locked(self) -> float:
        if len(self._stats) < 2:
            return 0.0
        span = self._stats[-1][0] - self._stats[0][0]
        return (len(self._stats) - 1) / span if span > 0 else 0.0

    def status(self) -> dict:
        """JSON friendly snapshot for the UI."""
        now = time.time()
        with self._lock:
            f = self._latest
            stats = [b for t, b in self._stats if now - t <= STATS_WINDOW_S]
            fresh = now - self._last_rx < self.stall_s
            return {
                "name": self.name,
                "connected": bool(self._connected and fresh),
                "fps": round(self._fps_locked(), 1) if fresh else 0.0,
                "frames": self._frames,
                "quality": None if f is None or f.quality is None else round(f.quality, 3),
                "quality_detail": self._latest_q,
                "bad": None if f is None else bool(f.bad),
                "bad_frac": round(sum(stats) / len(stats), 3) if stats else None,
                "recording": self._recorder.requested_path if self._recorder else None,
                "resolution": None if f is None else [int(f.img.shape[1]), int(f.img.shape[0])],
                "age_s": None if f is None else round(now - self._last_rx, 3),
                "buffer_frames": len(self._ring),
                "buffer_mb": round(self._ring_bytes / (1024.0 * 1024.0), 1),
                "buffer_s": round(self._ring[-1][0] - self._ring[0][0], 2) if self._ring else 0.0,
                "hold": self._hold_t,
                "packed_frames": self._n_packed,
                "dropped_held": self._dropped_held,
                "errors": self._errors,
                "last_error": self._last_error,
                "reconnects": self._reconnects,
                "source": self._source_info,
                "deinterlace": self.deinterlace_mode,
            }
