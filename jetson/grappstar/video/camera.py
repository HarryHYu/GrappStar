"""Camera: a background thread that keeps the newest frame of one video source.

    cam = Camera("dart", lambda: V4L2Source("/dev/grappstar-dartcam", "YUYV", 720, 480, 30))
    cam.latest()            # newest Frame or None
    cam.jpeg()              # newest frame as JPEG bytes (for the MJPEG web stream)
    cam.buffer(10)          # frames from the last 10 s (ring buffer)
    cam.start_recording("runs/flight.avi"); ...; cam.stop_recording()
    cam.status()            # dict for the UI
    cam.stop()

source_factory() must return a FrameSource (see INTERFACES.md). If it raises,
returns None, or the source stops giving frames, the thread closes it and calls
the factory again, waiting a little longer each time (backoff) so a missing
camera does not spin the CPU. When frames flow again the wait resets.
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
BUFFER_MAX_MB = 400.0     # ring buffer cap in MB of image data (720x480 BGR is about 1 MB per frame)


def _nbytes(f) -> int:
    img = getattr(f, "img", None)
    return int(getattr(img, "nbytes", 0) or 0)


class Camera:
    def __init__(self, name: str, source_factory: Callable, quality: bool = True,
                 deinterlace="none", buffer_s: float = 20.0, jpeg_quality: int = 80,
                 max_fps: Optional[float] = None, start: bool = True,
                 backoff_min: float = 0.5, backoff_max: float = 5.0,
                 stall_s: float = 3.0, none_limit: int = 100, buffer_max_mb: float = BUFFER_MAX_MB):
        self.name = name
        self.source_factory = source_factory
        self.quality_enabled = bool(quality)
        self.deinterlace_mode = normalize_mode(deinterlace)
        self.buffer_s = float(buffer_s)
        self.buffer_max_bytes = int(float(buffer_max_mb) * 1024 * 1024)
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

    def _handle(self, f: Frame) -> None:
        now = time.time()
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
            while len(self._ring) > 1 and (now - self._ring[0][0] > self.buffer_s
                                           or len(self._ring) > MAX_BUFFER_FRAMES
                                           or self._ring_bytes > self.buffer_max_bytes):
                self._ring_bytes -= _nbytes(self._ring.popleft()[1])
            self._stats.append((now, f.bad))
            while self._stats and now - self._stats[0][0] > STATS_WINDOW_S:
                self._stats.popleft()
            rec = self._recorder
            self._new_frame.notify_all()
        if rec is not None:
            try:
                rec.write(f)
            except Exception as e:
                self._error("record: %s" % e)

    # ------------------------------------------------------------ public API

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

    def buffer(self, seconds: Optional[float] = None) -> list:
        """Frames received in the last `seconds` before the newest frame (all if None).
        Measured from the newest frame, so it still works after the link drops."""
        with self._lock:
            if not self._ring:
                return []
            if seconds is None:
                return [f for _, f in self._ring]
            cutoff = self._ring[-1][0] - seconds
            return [f for t, f in self._ring if t >= cutoff]

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
                "errors": self._errors,
                "last_error": self._last_error,
                "reconnects": self._reconnects,
                "source": self._source_info,
                "deinterlace": self.deinterlace_mode,
            }
