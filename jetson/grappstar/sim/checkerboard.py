"""Synthetic checkerboard pictures through any lens (tests of the dart camera calibration).

board_view(lens, rvec, tvec, pattern=(9, 6), square_m=0.025, rng=None) -> BGR uint8
random_views(lens, n, rng) -> list of pictures with the board at random distances, tilts and places, many near the
edges of the picture (as README section 7b asks Harry to record them).
"""
from __future__ import annotations

import math
from typing import List, Optional

import cv2
import numpy as np


def board_view(lens, rvec, tvec, pattern=(9, 6), square_m: float = 0.025, rng: Optional[np.random.Generator] = None,
               noise: float = 6.0, blur: float = 0.8, analog: bool = True) -> np.ndarray:
    """The printed board (pattern inner corners, white margin of one square) seen through lens.

    rvec, tvec: board frame to camera frame (OpenCV convention, metres). Background: a grey room."""
    w, h = lens.size
    uu, vv = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
    rays = lens.unproject(np.column_stack([uu.ravel(), vv.ravel()]))        # camera frame
    R, _ = cv2.Rodrigues(np.asarray(rvec, np.float64))
    t = np.asarray(tvec, np.float64).reshape(3)
    n = R[:, 2]                                                             # board normal in the camera frame
    denom = rays @ n
    with np.errstate(divide="ignore", invalid="ignore"):
        s = (t @ n) / denom
    P = rays * s[:, None] - t                                               # camera frame, relative to board origin
    B = P @ R                                                               # board frame coordinates
    cols, rows = pattern
    bx, by = B[:, 0] / square_m, B[:, 1] / square_m
    # squares: corner (0,0) is the first inner corner; the board has cols+1 by rows+1 squares
    inside = (bx > -1) & (bx < cols) & (by > -1) & (by < rows) & (s > 0)
    margin = (bx > -2) & (bx < cols + 1) & (by > -2) & (by < rows + 1) & (s > 0)
    chk = ((np.floor(bx) + np.floor(by)) % 2 == 0)
    val = np.full(len(rays), 110.0)
    val[margin] = 235.0
    val[inside & chk] = 25.0
    img = val.reshape(h, w).astype(np.float32)
    # supersampling stand-in: soften the hard edges a little like optics do
    img = cv2.GaussianBlur(img, (0, 0), max(0.3, blur))
    rng = rng or np.random.default_rng(0)
    if analog:
        img = img + rng.normal(0, noise, img.shape).astype(np.float32)
        img = cv2.GaussianBlur(img, (0, 1), sigmaX=0.6, sigmaY=0.01)
    out = np.clip(img, 0, 255).astype(np.uint8)
    return cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)


def random_views(lens, n: int, rng: np.random.Generator, pattern=(9, 6), square_m: float = 0.025) -> List[np.ndarray]:
    """n pictures of the board at 0.18 to 0.4 m, tilted up to 45 deg, its centre aimed anywhere up to 60 deg off
    the axis (so the corners of the fisheye picture are covered)."""
    out = []
    cols, rows = pattern
    centre = np.array([(cols - 1) * square_m / 2, (rows - 1) * square_m / 2, 0.0])
    for _ in range(n):
        dist = rng.uniform(0.18, 0.4)
        off = math.radians(rng.uniform(0, 60))
        az = rng.uniform(0, 2 * math.pi)
        d = np.array([math.sin(off) * math.cos(az), math.sin(off) * math.sin(az), math.cos(off)])
        c_cam = d * dist                                       # board centre in the camera frame
        ax = np.cross([0.0, 0.0, 1.0], d)
        Ra, _ = cv2.Rodrigues(ax / (np.linalg.norm(ax) + 1e-12) * off)    # board faces the camera ...
        tilt = rng.normal(0, 0.35, 3)                                        # ... give or take a tilt
        tilt[2] = rng.uniform(-math.pi, math.pi) * 0.2
        R = Ra @ cv2.Rodrigues(tilt)[0]
        t = c_cam - R @ centre
        out.append(board_view(lens, cv2.Rodrigues(R)[0], t, pattern, square_m, rng))
    return out
