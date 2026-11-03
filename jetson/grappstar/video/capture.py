"""Frame sources: live V4L2 cameras, GStreamer pipelines and in-memory lists.

Every source follows the duck-typed FrameSource contract in INTERFACES.md:
    .name, .read() -> Optional[Frame], .close()

Why the explicit FOURCC: on JetPack the OpenCV build and the system libjpeg do not
match, so letting the driver pick MJPEG by default can fail or give garbage. We set
the pixel format (YUYV or MJPG) first, then width, height and fps, because many UVC
drivers reset the size when the format changes.
"""
import os
import re
import time
from dataclasses import replace
from typing import Optional, Sequence, Union

import cv2
import numpy as np

from .frame import Frame, ensure_bgr


# ----------------------------------------------------------------- helpers

def fourcc_to_str(code) -> str:
    """Turn the float that cap.get(CAP_PROP_FOURCC) returns into e.g. 'MJPG'."""
    c = int(code)
    if c <= 0:
        return ""
    return "".join(chr((c >> (8 * i)) & 0xFF) for i in range(4)).strip("\x00")


def resolve_device(dev: Union[int, str]) -> Union[int, str]:
    """Accept 0, "0", "/dev/video0", a /dev/v4l/by-id/... link or a udev symlink
    like /dev/grappstar-dartcam, and return the integer index when we can find it.
    OpenCV's V4L2 backend is most reliable with an integer index.
    """
    if isinstance(dev, int):
        return dev
    s = str(dev).strip()
    if s.isdigit():
        return int(s)
    real = os.path.realpath(s)          # follows by-id and udev symlinks
    m = re.fullmatch(r"/dev/video(\d+)", real)
    if m:
        return int(m.group(1))
    return real                         # unusual path: let OpenCV try it as is


# ----------------------------------------------------------------- V4L2

class V4L2Source:
    """A USB (UVC) camera or capture dongle through OpenCV's V4L2 backend.

    Raises RuntimeError if the device cannot be opened, so Camera can retry.
    After opening, .info holds what the driver actually gave us.
    """

    def __init__(self, dev, fourcc: str = "YUYV", width: Optional[int] = None,
                 height: Optional[int] = None, fps: Optional[float] = None,
                 name: Optional[str] = None, buffersize: Optional[int] = 1):
        self.dev = dev
        self.name = name or str(dev)
        self.fourcc = (fourcc or "YUYV").upper()
        if self.fourcc == "MJPEG":
            self.fourcc = "MJPG"
        self._idx = 0
        target = resolve_device(dev)
        self.cap = cv2.VideoCapture(target, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError("cannot open V4L2 device %r (resolved to %r)" % (dev, target))
        # Order matters: FOURCC first, then size, then fps.
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        if width:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
        if height:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
        if fps:
            self.cap.set(cv2.CAP_PROP_FPS, float(fps))
        if buffersize:
            # Small driver queue = low latency (we always want the newest frame).
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, int(buffersize))
        # Read back what we really got: drivers silently pick the nearest mode.
        self.info = {
            "dev": str(dev),
            "resolved": str(target),
            "fourcc_requested": self.fourcc,
            "fourcc": fourcc_to_str(self.cap.get(cv2.CAP_PROP_FOURCC)),
            "width": int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            "height": int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            "fps": float(self.cap.get(cv2.CAP_PROP_FPS)),
        }

    def read(self) -> Optional[Frame]:
        if self.cap is None:
            return None
        ok, img = self.cap.read()
        t = time.time()
        if not ok or img is None or img.size == 0:
            return None
        f = Frame(img=ensure_bgr(img), t=t, idx=self._idx)
        self._idx += 1
        return f

    def close(self) -> None:
        if self.cap is not None:
            self.cap.release()
            self.cap = None


# ----------------------------------------------------------------- GStreamer

def has_gstreamer() -> bool:
    """True if this OpenCV build was compiled with GStreamer support."""
    try:
        info = cv2.getBuildInformation()
    except Exception:
        return False
    m = re.search(r"GStreamer:\s*(\S+)", info)
    return bool(m and m.group(1).upper().startswith("YES"))


_SINK = "videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false"


def _caps_tail(width, height, fps) -> str:
    parts = []
    if width:
        parts.append("width=%d" % int(width))
    if height:
        parts.append("height=%d" % int(height))
    if fps:
        parts.append("framerate=%d/1" % int(round(fps)))
    return ("," + ",".join(parts)) if parts else ""


def gst_pipeline_yuyv(dev, width=None, height=None, fps=None) -> str:
    """v4l2src pipeline for raw YUYV (called YUY2 in GStreamer)."""
    return ("v4l2src device=%s ! video/x-raw,format=YUY2%s ! %s"
            % (dev, _caps_tail(width, height, fps), _SINK))


def gst_pipeline_mjpeg(dev, width=None, height=None, fps=None, decoder="jpegdec") -> str:
    """v4l2src pipeline for MJPEG. decoder: "jpegdec" (CPU, always there) or
    "nvjpegdec" (Jetson hardware decoder, avoids the libjpeg mismatch)."""
    if decoder == "nvjpegdec":
        dec = "nvjpegdec ! video/x-raw,format=I420"
    elif decoder == "jpegdec":
        dec = "jpegdec"
    else:
        raise ValueError("decoder must be 'jpegdec' or 'nvjpegdec'")
    return ("v4l2src device=%s io-mode=2 ! image/jpeg%s ! jpegparse ! %s ! %s"
            % (dev, _caps_tail(width, height, fps), dec, _SINK))


class GstSource:
    """OpenCV capture from a GStreamer pipeline string ending in appsink."""

    def __init__(self, pipeline: str, name: str = "gst"):
        self.name = name
        self.pipeline = pipeline
        self._idx = 0
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            hint = "" if has_gstreamer() else " (this OpenCV build has no GStreamer)"
            raise RuntimeError("cannot open GStreamer pipeline%s: %s" % (hint, pipeline))
        self.info = {"pipeline": pipeline,
                     "width": int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                     "height": int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}

    def read(self) -> Optional[Frame]:
        if self.cap is None:
            return None
        ok, img = self.cap.read()
        t = time.time()
        if not ok or img is None:
            return None
        f = Frame(img=ensure_bgr(img), t=t, idx=self._idx)
        self._idx += 1
        return f

    def close(self) -> None:
        if self.cap is not None:
            self.cap.release()
            self.cap = None


# ----------------------------------------------------------------- in memory

class ListSource:
    """Plays a list of Frames (or plain images) from memory. For tests and the simulator.

    realtime=True sleeps so frames come out at the pace of their timestamps
    (plain images are paced at `fps`). loop=True starts again at the end.
    restamp=True replaces each frame's t with time.time() when it is read
    (useful when looping, so timestamps keep moving forward).
    Each read returns a new Frame object (same pixels), so callers may set
    .quality / .bad without touching the originals.
    """

    def __init__(self, frames: Sequence, realtime: bool = False, loop: bool = False,
                 fps: float = 30.0, restamp: bool = False, name: str = "list"):
        self.name = name
        self.realtime = realtime
        self.loop = loop
        self.restamp = restamp
        self.fps = float(fps)
        t0 = time.time()
        self.frames = []
        for i, f in enumerate(frames):
            if isinstance(f, Frame):
                self.frames.append(f)
            else:
                self.frames.append(Frame(img=ensure_bgr(np.asarray(f)), t=t0 + i / self.fps, idx=i))
        self._pos = 0
        self._count = 0
        self._closed = False
        self._wall0 = None      # wall clock time of the first frame of the current pass
        self._t0 = None         # frame timestamp of the first frame of the current pass

    def __len__(self):
        return len(self.frames)

    def read(self) -> Optional[Frame]:
        if self._closed or not self.frames:
            return None
        if self._pos >= len(self.frames):
            if not self.loop:
                return None
            self._pos = 0
            self._wall0 = None
        src = self.frames[self._pos]
        if self.realtime:
            now = time.time()
            if self._wall0 is None:
                self._wall0, self._t0 = now, src.t
            else:
                wait = (src.t - self._t0) - (now - self._wall0)
                if wait > 0:
                    time.sleep(min(wait, 1.0))
        self._pos += 1
        out = replace(src, idx=self._count if self.loop else src.idx)
        if self.restamp:
            out.t = time.time()
        self._count += 1
        return out

    def close(self) -> None:
        self._closed = True
