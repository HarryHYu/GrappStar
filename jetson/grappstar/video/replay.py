"""Play back a recording made by video.record.Recorder as a FrameSource.

    src = FileSource("flight.avi")      # or the "flight.avi_jpg" folder
    while (f := src.read()) is not None: ...   (Python 3.8+ walrus)

Timestamps, quality and bad flags come from the sidecar <path>.frames.jsonl.
Without a sidecar (any random video file) t is synthesised from the fps,
starting at t0 (default: the file's modification time, or 0 if unknown).
realtime=True sleeps between frames to reproduce the original pace.
"""
import glob
import json
import os
import time
from typing import Optional, Union

import cv2

from .frame import Frame, ensure_bgr
from .record import sidecar_path


def load_sidecar(path: str) -> dict:
    """Read <path>.frames.jsonl into {i: record}. Missing file -> {}."""
    out = {}
    for p in (sidecar_path(path), os.path.join(path, "frames.jsonl")):
        if os.path.isfile(p):
            with open(p) as fh:
                for line in fh:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        rec = json.loads(line)
                        out[int(rec["i"])] = rec
                    except (ValueError, KeyError):
                        continue      # tolerate a half written last line
            break
    return out


class FileSource:
    def __init__(self, path: str, realtime: bool = False, fps: Optional[float] = None,
                 t0: Union[str, float, None] = "mtime", name: Optional[str] = None):
        self.path = path
        self.name = name or os.path.basename(path.rstrip("/"))
        self.realtime = realtime
        self.side = load_sidecar(path)
        self._i = 0
        self._wall0 = None
        self._tfirst = None
        self.cap = None
        self.files = None
        if os.path.isdir(path):
            self.files = sorted(glob.glob(os.path.join(path, "*.jpg")) +
                                glob.glob(os.path.join(path, "*.jpeg")) +
                                glob.glob(os.path.join(path, "*.png")))
            self.frame_count = len(self.files)
            file_fps = None
        else:
            if not os.path.exists(path):
                raise FileNotFoundError(path)
            self.cap = cv2.VideoCapture(path)
            if not self.cap.isOpened():
                raise RuntimeError("cannot open video file %r" % path)
            self.frame_count = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
            file_fps = self.cap.get(cv2.CAP_PROP_FPS) or None
        self.fps = float(fps or file_fps or 30.0)
        if t0 == "mtime":
            try:
                t0 = os.path.getmtime(path)
            except OSError:
                t0 = 0.0
        self.t0 = float(t0 or 0.0)

    def __len__(self):
        return self.frame_count

    def _next_image(self):
        if self.files is not None:
            if self._i >= len(self.files):
                return None
            return cv2.imread(self.files[self._i], cv2.IMREAD_COLOR)
        if self.cap is None:
            return None
        ok, img = self.cap.read()
        return img if ok else None

    def read(self) -> Optional[Frame]:
        img = self._next_image()
        if img is None:
            return None
        i = self._i
        self._i += 1
        rec = self.side.get(i)
        if rec is not None:
            t = float(rec["t"])
            q = rec.get("q")
            bad = bool(rec.get("bad", False))
        else:
            t, q, bad = self.t0 + i / self.fps, None, False
        if self.realtime:
            now = time.time()
            if self._wall0 is None:
                self._wall0, self._tfirst = now, t
            else:
                wait = (t - self._tfirst) - (now - self._wall0)
                if wait > 0:
                    time.sleep(min(wait, 2.0))
        return Frame(img=ensure_bgr(img), t=t, idx=i, quality=q, bad=bad)

    def read_all(self) -> list:
        """Convenience: every remaining frame as a list."""
        out = []
        while True:
            f = self.read()
            if f is None:
                return out
            out.append(f)

    def close(self) -> None:
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        self.files = [] if self.files is not None else None
