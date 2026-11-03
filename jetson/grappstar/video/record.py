"""Record frames to an AVI plus a sidecar of exact timestamps.

    rec = Recorder("flight.avi", fps=30)
    rec.write(frame)          # thread safe
    rec.close()

Files:
    flight.avi                  MJPG video (cv2.VideoWriter)
    flight.avi.frames.jsonl     one line per frame: {"i", "t", "q", "bad"}
        i   position of the frame in the file (0, 1, 2, ...)
        t   time.time() at capture, so replay lines up with telemetry logs
        q   quality score or null, bad = skip for vision

If the VideoWriter cannot open (missing codec), we fall back to a folder of
JPEGs: flight.avi_jpg/000000.jpg ... and the sidecar becomes
flight.avi_jpg.frames.jsonl. rec.path always tells you what was really written.
"""
import json
import os
import threading
from typing import Optional

import cv2

from .frame import Frame


SIDECAR_FLUSH_EVERY = 30   # sidecar lines between flushes (1 s at 30 fps)


def sidecar_path(path: str) -> str:
    return path + ".frames.jsonl"


class Recorder:
    def __init__(self, path: str, fps: float = 30.0, fourcc: str = "MJPG",
                 jpeg_quality: int = 90, force_jpeg_folder: bool = False):
        self.requested_path = path
        self.path = path
        self.fps = float(fps) if fps and fps > 0 else 30.0
        self.fourcc = fourcc
        self.jpeg_quality = int(jpeg_quality)
        self.force_jpeg_folder = force_jpeg_folder
        self.mode = None            # "video" or "jpeg", decided on the first frame
        self.size = None            # (w, h) of the video
        self.count = 0
        self._writer = None
        self._side = None
        self._lock = threading.Lock()
        self._closed = False
        d = os.path.dirname(os.path.abspath(path))
        os.makedirs(d, exist_ok=True)

    def _open(self, frame: Frame) -> None:
        """Open the writer once we know the frame size."""
        h, w = frame.img.shape[:2]
        self.size = (w, h)
        if not self.force_jpeg_folder:
            wr = cv2.VideoWriter(self.path, cv2.VideoWriter_fourcc(*self.fourcc), self.fps, (w, h))
            if wr.isOpened():
                self._writer = wr
                self.mode = "video"
            else:
                wr.release()
        if self.mode is None:
            # Fallback: a folder of JPEGs, works with any OpenCV build.
            self.path = self.requested_path + "_jpg"
            os.makedirs(self.path, exist_ok=True)
            self.mode = "jpeg"
        self._side = open(sidecar_path(self.path), "w")

    def write(self, frame: Frame) -> None:
        with self._lock:
            if self._closed:
                return
            if self.mode is None:
                self._open(frame)
            img = frame.img
            if (img.shape[1], img.shape[0]) != self.size:
                # A video file needs one size; resize frames that differ.
                img = cv2.resize(img, self.size)
            if self.mode == "video":
                self._writer.write(img)
            else:
                name = os.path.join(self.path, "%06d.jpg" % self.count)
                cv2.imwrite(name, img, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            rec = {"i": self.count, "t": round(float(frame.t), 6),
                   "q": None if frame.quality is None else round(float(frame.quality), 3),
                   "bad": bool(frame.bad)}
            self._side.write(json.dumps(rec) + "\n")
            self.count += 1
            if self.count % SIDECAR_FLUSH_EVERY == 0:
                self._side.flush()      # a crash or power cut keeps all but the last few timestamps

    def close(self) -> Optional[str]:
        """Finish the files. Returns the path written (None if no frame came)."""
        with self._lock:
            if self._closed:
                return self.path if self.count else None
            self._closed = True
            if self._writer is not None:
                self._writer.release()
                self._writer = None
            if self._side is not None:
                self._side.close()
                self._side = None
            return self.path if self.count else None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
