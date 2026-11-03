"""Rover targeting: from a pixel in the rover camera to range and bearing on the ground.

The rover camera is a Razer Kiyo X (UVC, up to 1920x1080, about 82 degrees diagonal FOV).
Its mounting height and pitch are not known in advance; they come from calibration
(grappstar.vision.calibrate). The launcher has pitch (aim) only and no yaw, so the
operator turns the whole rover until the bearing is close to zero, then the ballistics
solver picks the launcher pitch for the range.

Conventions
-----------
- Image: u to the right, v down, pixels, at the calibration resolution (calib_size)
  unless an image_size is passed (intrinsics are then scaled).
- Camera axes: x right, y down, z forward (OpenCV).
- World (rover) frame: origin on the ground directly below the camera (the camera foot
  point), X to the right, Y forward, Z up. The ground is the flat plane Z = 0 and the
  camera sits at Z = height_m.
- pitch_deg: camera tilt DOWN from horizontal, positive = looking down (0 = horizon in
  the image centre, 90 = looking straight down).
- roll_deg: rotation about the optical axis, positive = the camera's right side is lower
  (the picture appears rotated anticlockwise).
- yaw_deg: camera heading relative to the rover (and launcher) axis, positive = camera
  looks to the right. Default 0; set it if the camera is not aligned with the launcher.
- range_m: horizontal distance from the camera foot point to the ground point.
- bearing_deg: angle from rover straight ahead to the point, positive = to the right.

Public API:
    CameraModel(fx, fy, cx, cy, height_m, pitch_deg, roll_deg=0, ...)
        .pixel_to_ground(u, v) -> (range_m, bearing_deg) or None above the horizon
        .ground_to_pixel(range_m, bearing_deg) -> (u, v) or None if behind the camera
        .world_to_pixel(x, y, z), .ground_point(u, v), .for_size(w, h)
        .range_uncertainty(u, v), .save(path), CameraModel.load(path)
    detect_orange(img) -> (u, v, area_px) or None   (u, v = bottom centre of the blob)
    turn_advice(bearing_deg) -> "Turn the rover 12 degrees right"
    target_info(cam, u, v, image_size=None) -> dict for the UI
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, asdict
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

from grappstar.video.frame import ensure_bgr


@dataclass
class CameraModel:
    fx: float
    fy: float
    cx: float
    cy: float
    height_m: float
    pitch_deg: float
    roll_deg: float = 0.0
    dist: Optional[List[float]] = None            # OpenCV distortion (k1, k2, p1, p2[, k3])
    calib_size: Optional[Tuple[int, int]] = None  # (width, height) the intrinsics refer to
    yaw_deg: float = 0.0
    uncalibrated: bool = False                    # True = default intrinsics, not measured
    notes: str = ""

    # ------------------------------------------------------------ geometry

    def K(self) -> np.ndarray:
        return np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]], np.float64)

    def rotation(self) -> np.ndarray:
        """R_wc: columns are the camera x, y, z axes expressed in the world frame."""
        th = math.radians(self.pitch_deg)
        rh = math.radians(self.roll_deg)
        ya = math.radians(self.yaw_deg)
        fwd = np.array([0.0, math.cos(th), -math.sin(th)])
        down = np.array([0.0, -math.sin(th), -math.cos(th)])
        right = np.array([1.0, 0.0, 0.0])
        xc = math.cos(rh) * right + math.sin(rh) * down
        yc = -math.sin(rh) * right + math.cos(rh) * down
        R = np.stack([xc, yc, fwd], axis=1)
        Rz = np.array([[math.cos(ya), math.sin(ya), 0], [-math.sin(ya), math.cos(ya), 0], [0, 0, 1]])
        return Rz @ R

    def for_size(self, width: int, height: int) -> "CameraModel":
        """Copy with intrinsics scaled to a stream of width x height pixels.

        Assumes the stream is the full sensor view resampled (same aspect, no crop). If the
        aspect differs, x and y are scaled independently (a warning-worthy situation: UVC
        cameras often crop instead of scale for other aspects; calibrate at the stream size).
        """
        if not self.calib_size:
            return self
        cw, ch = self.calib_size
        if (int(width), int(height)) == (int(cw), int(ch)):
            return self
        sx, sy = width / float(cw), height / float(ch)
        d = asdict(self)
        d.update(fx=self.fx * sx, fy=self.fy * sy, cx=(self.cx + 0.5) * sx - 0.5,
                 cy=(self.cy + 0.5) * sy - 0.5, calib_size=(int(width), int(height)))
        return CameraModel(**d)

    def _model_for(self, image_size) -> "CameraModel":
        return self.for_size(*image_size) if image_size else self

    def _normalised(self, u: float, v: float) -> Tuple[float, float]:
        if self.dist and any(abs(x) > 0 for x in self.dist):
            p = cv2.undistortPoints(np.array([[[u, v]]], np.float64), self.K(),
                                    np.array(self.dist, np.float64))
            return float(p[0, 0, 0]), float(p[0, 0, 1])
        return (u - self.cx) / self.fx, (v - self.cy) / self.fy

    def pixel_ray(self, u: float, v: float, image_size=None) -> np.ndarray:
        """Unit ray direction in the world frame for pixel (u, v)."""
        m = self._model_for(image_size)
        x, y = m._normalised(u, v)
        d = m.rotation() @ np.array([x, y, 1.0])
        return d / np.linalg.norm(d)

    def ground_point(self, u: float, v: float, image_size=None) -> Optional[Tuple[float, float]]:
        """(X right, Y forward) metres on the flat ground, or None if at/above the horizon."""
        d = self.pixel_ray(u, v, image_size)
        if d[2] >= -1e-6:
            return None
        t = self.height_m / -d[2]
        return float(t * d[0]), float(t * d[1])

    def pixel_to_ground(self, u: float, v: float, image_size=None) -> Optional[Tuple[float, float]]:
        """(range_m, bearing_deg) of pixel (u, v) on flat ground; None above the horizon.

        Flat ground assumption: a target on a slope or behind a rise will be misranged.
        """
        p = self.ground_point(u, v, image_size)
        if p is None:
            return None
        x, y = p
        return float(math.hypot(x, y)), float(math.degrees(math.atan2(x, y)))

    def world_to_pixel(self, x: float, y: float, z: float = 0.0, image_size=None) -> Optional[Tuple[float, float]]:
        """Project a world point (X right, Y forward, Z up from the ground) to a pixel."""
        m = self._model_for(image_size)
        pc = m.rotation().T @ (np.array([x, y, z], np.float64) - np.array([0, 0, m.height_m]))
        if pc[2] <= 1e-6:
            return None
        if m.dist and any(abs(k) > 0 for k in m.dist):
            p, _ = cv2.projectPoints(pc.reshape(1, 1, 3), np.zeros(3), np.zeros(3), m.K(),
                                     np.array(m.dist, np.float64))
            return float(p[0, 0, 0]), float(p[0, 0, 1])
        return float(m.fx * pc[0] / pc[2] + m.cx), float(m.fy * pc[1] / pc[2] + m.cy)

    def ground_to_pixel(self, range_m: float, bearing_deg: float, image_size=None) -> Optional[Tuple[float, float]]:
        """Inverse of pixel_to_ground: pixel of the ground point at range and bearing."""
        b = math.radians(bearing_deg)
        return self.world_to_pixel(range_m * math.sin(b), range_m * math.cos(b), 0.0, image_size)

    def horizon_v(self, u: Optional[float] = None) -> float:
        """Image row of the horizon (at column u, default centre), ignoring distortion."""
        u = self.cx if u is None else u
        # Solve for v where the ray is horizontal: (R @ [x, y, 1])_z = 0.
        R = self.rotation()
        x = (u - self.cx) / self.fx
        a, b, c = R[2, 0], R[2, 1], R[2, 2]
        if abs(b) < 1e-9:
            return float("nan")
        y = -(a * x + c) / b
        return float(y * self.fy + self.cy)

    def range_uncertainty(self, u: float, v: float, px: float = 1.0, pitch_err_deg: float = 1.0,
                          image_size=None) -> Optional[dict]:
        """Range spread from +-px pixel error in v and +-pitch_err_deg pitch error.

        Returns {range_m, px_lo, px_hi, pitch_lo, pitch_hi, lo, hi, plus_m, minus_m}; any
        value may be inf when the perturbed ray goes above the horizon. Far targets near
        the horizon are very sensitive to pitch: one degree can double the range.
        """
        base = self.pixel_to_ground(u, v, image_size)
        if base is None:
            return None
        r0 = base[0]

        def rng(model, uu, vv):
            p = model.pixel_to_ground(uu, vv, image_size)
            return float("inf") if p is None else p[0]

        r_px_a, r_px_b = rng(self, u, v - px), rng(self, u, v + px)
        d = asdict(self)
        d["pitch_deg"] = self.pitch_deg - pitch_err_deg
        r_p_a = rng(CameraModel(**d), u, v)
        d["pitch_deg"] = self.pitch_deg + pitch_err_deg
        r_p_b = rng(CameraModel(**d), u, v)
        vals = [r_px_a, r_px_b, r_p_a, r_p_b]
        hi, lo = max(vals), min(vals)
        return {"range_m": r0, "px_lo": min(r_px_a, r_px_b), "px_hi": max(r_px_a, r_px_b),
                "pitch_lo": min(r_p_a, r_p_b), "pitch_hi": max(r_p_a, r_p_b),
                "lo": lo, "hi": hi, "plus_m": hi - r0, "minus_m": r0 - lo}

    # ------------------------------------------------------------ persistence

    def to_dict(self) -> dict:
        d = asdict(self)
        d["calib_size"] = list(self.calib_size) if self.calib_size else None
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "CameraModel":
        keys = cls.__dataclass_fields__.keys()
        kw = {k: d[k] for k in keys if k in d}
        if kw.get("calib_size") is not None:
            kw["calib_size"] = tuple(int(x) for x in kw["calib_size"])
        if kw.get("dist") is not None:
            kw["dist"] = [float(x) for x in kw["dist"]]
        return cls(**kw)

    def save(self, path: str) -> None:
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=1)

    @classmethod
    def load(cls, path: str) -> "CameraModel":
        with open(path) as f:
            return cls.from_dict(json.load(f))


# ================================================================ orange marker

ORANGE_HUE = (5, 20)       # OpenCV hue units (0..179); safety orange is about 10 to 15
ORANGE_S_MIN = 140
ORANGE_V_MIN = 90


def orange_mask(img: np.ndarray, hue=ORANGE_HUE, s_min: int = ORANGE_S_MIN,
                v_min: int = ORANGE_V_MIN) -> np.ndarray:
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, (int(hue[0]), int(s_min), int(v_min)), (int(hue[1]), 255, 255))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    return m


def detect_orange(img: np.ndarray, min_area: int = 25, hue=ORANGE_HUE, s_min: int = ORANGE_S_MIN,
                  v_min: int = ORANGE_V_MIN) -> Optional[Tuple[float, float, int]]:
    """Find the largest safety orange blob. Returns (u, v, area_px) or None.

    (u, v) is the BOTTOM centre of the blob: the point where a standing marker (cone, pole,
    flag) touches the ground, which is what pixel_to_ground needs. u is the mean column of
    the blob's lowest rows, v is its lowest row (+0.5 px, the pixel's lower edge).
    """
    if img is None or img.size == 0:
        return None
    img = ensure_bgr(img)
    m = orange_mask(img, hue, s_min, v_min)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m, 8)
    if n <= 1:
        return None
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    area = int(stats[i, cv2.CC_STAT_AREA])
    if area < min_area:
        return None
    top, hgt = int(stats[i, cv2.CC_STAT_TOP]), int(stats[i, cv2.CC_STAT_HEIGHT])
    bottom = top + hgt - 1
    band = max(2, hgt // 10)
    ys, xs = np.nonzero(lab[max(top, bottom - band + 1):bottom + 1] == i)
    u = float(xs.mean()) if len(xs) else float(stats[i, cv2.CC_STAT_LEFT] + stats[i, cv2.CC_STAT_WIDTH] / 2)
    return u, float(bottom) + 0.5, area


# ================================================================ operator helpers

def turn_advice(bearing_deg: float, deadband_deg: float = 1.0) -> str:
    """Plain text: which way and how far to turn the rover to face the target."""
    if bearing_deg is None or not math.isfinite(bearing_deg):
        return "No target bearing"
    b = float(bearing_deg)
    if abs(b) < deadband_deg:
        return "Target is straight ahead"
    deg = int(round(abs(b)))
    return "Turn the rover %d degree%s %s" % (deg, "" if deg == 1 else "s", "right" if b > 0 else "left")


def target_info(cam: CameraModel, u: float, v: float, image_size=None) -> dict:
    """Everything the UI needs for a clicked or detected target pixel."""
    p = cam.pixel_to_ground(u, v, image_size)
    if p is None:
        return {"ok": False, "msg": "That point is at or above the horizon", "range_m": None,
                "bearing_deg": None}
    unc = cam.range_uncertainty(u, v, image_size=image_size) or {}

    def fin(x):
        return None if x is None or not math.isfinite(x) else round(float(x), 2)

    return {"ok": True, "range_m": round(p[0], 2), "bearing_deg": round(p[1], 1),
            "advice": turn_advice(p[1]), "range_lo_m": fin(unc.get("lo")),
            "range_hi_m": fin(unc.get("hi")), "uncalibrated": bool(cam.uncalibrated),
            "msg": turn_advice(p[1]) + ("" if not cam.uncalibrated else
                                        " (camera NOT calibrated: range is a rough guess)")}
