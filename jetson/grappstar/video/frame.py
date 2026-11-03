"""The Frame record passed between every video module.

See jetson/INTERFACES.md ("video"). A frame is one BGR image plus the wall clock
time it was captured, so video lines up with telemetry logs on replay.
"""
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class Frame:
    img: np.ndarray                   # BGR uint8, HxWx3
    t: float                          # time.time() at capture
    idx: int                          # frame counter from the source
    quality: Optional[float] = None   # 0..1 from video.quality (None = not evaluated)
    bad: bool = False                 # True = snow / dropout / sync loss: skip for vision

    @property
    def shape(self):
        """(height, width) of the image, handy for status displays."""
        return self.img.shape[:2]


def ensure_bgr(img: np.ndarray) -> np.ndarray:
    """Return a HxWx3 uint8 image (grey or BGRA input is converted)."""
    import cv2
    if img.ndim == 2:
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if img.ndim == 3 and img.shape[2] == 4:
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    if img.ndim == 3 and img.shape[2] == 1:
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img
