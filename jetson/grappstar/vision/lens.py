"""Dart camera lens model (Caddx Ant: wide fisheye, analog 720x480 after the capture dongle).

The vision code (horizon, flight phases, landing zone, aerial map) was written and tuned for a plain pinhole camera
with a 100 deg horizontal field of view and square pixels. The real dart camera is a fisheye, and an NTSC capture
of 720x480 shows a 4:3 picture, so its pixels are not square (each pixel is about 0.89 as wide as it is tall).
Instead of teaching every algorithm about the fisheye, every dart frame is first turned into that canonical
pinhole view (rectify()): one cv2.remap per frame, about 1 ms on the Jetson. The algorithms then see what they
were tuned on, and the only thing that depends on the real lens is this one measured model.

Lens model (OpenCV fisheye, the "equidistant plus polynomial" model):
    theta = angle of the ray from the optical axis
    theta_d = theta * (1 + k1 theta^2 + k2 theta^4 + k3 theta^6 + k4 theta^8)
    u = fx * theta_d * cos(phi) + cx,  v = fy * theta_d * sin(phi) + cy
With k1..k4 = 0 this is the plain equidistant fisheye (angle proportional to the distance from the centre).

Where the numbers come from (order of trust):
1. A calibration file written by `python3 -m grappstar.vision.calibrate dart ...` from a printed checkerboard
   (README section 7b). This measures fx, fy, cx, cy and k1..k4.
2. settings.json "dart_hfov_deg" (and "dart_aspect": "4:3" or "16:9"): an equidistant model with that horizontal
   field of view. Only a guess for the real lens.
3. DEFAULT_HFOV_DEG below. The Caddx Ant is sold as "165 deg" (diagonal, 1.8 mm lens). With an equidistant lens a
   165 deg diagonal is about 132 deg across a 4:3 picture; 135 deg is used. NOT MEASURED.

How much a wrong field of view costs was measured on the synthetic flights (tests/test_lens.py,
tests/test_realistic_flight.py): a model 15 deg too narrow or too wide moves the horizon pitch near the image centre
by under 1 deg and the range estimate by under 3 percent, but it bends straight lines near the image edges and the
aerial map loses links. Calibrate the lens once (10 minutes) and the problem goes away.

Pinhole lenses (lens="pinhole", for example a test camera or the simulator's old default) work too.
"""
from __future__ import annotations

import json
import math
import os
from typing import Optional, Sequence, Tuple

import numpy as np

DEFAULT_HFOV_DEG = 135.0     # Caddx Ant, equidistant guess from the 165 deg diagonal spec: MEASURE (calibrate dart)
DEFAULT_SIZE = (720, 480)    # NTSC capture
DEFAULT_ASPECT = "4:3"       # the picture shape the camera sends (Caddx Ant menu: 4:3 or 16:9)
CANON_HFOV_DEG = 100.0       # the pinhole view every algorithm was tuned on
CANON_SIZE = (720, 480)      # canonical view size (square pixels)


def _aspect_ratio(aspect) -> float:
    if isinstance(aspect, (int, float)):
        return float(aspect)
    a = str(aspect or DEFAULT_ASPECT).replace("/", ":")
    if ":" in a:
        x, y = a.split(":")
        return float(x) / float(y)
    return float(a)


class DartLens:
    """A camera model: model ("fisheye" or "pinhole"), K (3x3), D (k1..k4, fisheye only), size (w, h)."""

    def __init__(self, K, D=None, size: Tuple[int, int] = DEFAULT_SIZE, model: str = "fisheye",
                 source: str = "", rms: Optional[float] = None):
        self.K = np.asarray(K, dtype=np.float64).reshape(3, 3)
        self.D = np.zeros(4) if D is None else np.asarray(D, dtype=np.float64).reshape(-1)[:4]
        if len(self.D) < 4:
            self.D = np.concatenate([self.D, np.zeros(4 - len(self.D))])
        self.size = (int(size[0]), int(size[1]))
        if model not in ("fisheye", "pinhole"):
            raise ValueError("lens model must be 'fisheye' or 'pinhole'")
        self.model = model
        self.source = source
        self.rms = rms
        self._maps = {}

    # ------------------------------------------------------------ constructors
    @classmethod
    def equidistant(cls, hfov_deg: float = DEFAULT_HFOV_DEG, size: Tuple[int, int] = DEFAULT_SIZE,
                    aspect=DEFAULT_ASPECT, source: str = "") -> "DartLens":
        """Plain equidistant fisheye with this horizontal field of view across the picture. aspect is the shape of
        the picture the camera sends ("4:3" squeezed into 720x480 gives fy = 0.89 fx)."""
        w, h = int(size[0]), int(size[1])
        fx = (w / 2.0) / math.radians(hfov_deg / 2.0)
        fy = fx * (h / w) * _aspect_ratio(aspect)
        K = [[fx, 0, (w - 1) / 2.0], [0, fy, (h - 1) / 2.0], [0, 0, 1]]
        return cls(K, None, (w, h), "fisheye", source or f"equidistant {hfov_deg:g} deg ({aspect}), not measured")

    @classmethod
    def pinhole(cls, hfov_deg: float = CANON_HFOV_DEG, size: Tuple[int, int] = CANON_SIZE,
                aspect=None, source: str = "") -> "DartLens":
        w, h = int(size[0]), int(size[1])
        fx = (w / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        fy = fx if aspect is None else fx * (h / w) * _aspect_ratio(aspect)
        K = [[fx, 0, (w - 1) / 2.0], [0, fy, (h - 1) / 2.0], [0, 0, 1]]
        return cls(K, None, (w, h), "pinhole", source or f"pinhole {hfov_deg:g} deg")

    @classmethod
    def load(cls, path: str) -> "DartLens":
        with open(path) as f:
            d = json.load(f)
        return cls(d["K"], d.get("D"), tuple(d["size"]), d.get("model", "fisheye"),
                   source=f"calibration {os.path.basename(path)}", rms=d.get("rms"))

    @classmethod
    def from_settings(cls, calib_path: Optional[str] = None, hfov_deg: Optional[float] = None,
                      lens: Optional[str] = None, aspect=None,
                      size: Tuple[int, int] = DEFAULT_SIZE) -> "DartLens":
        """The dart lens from settings: a calibration file when it exists, else lens + hfov_deg, else the default."""
        if calib_path and os.path.exists(calib_path):
            return cls.load(calib_path)
        if (lens or "fisheye") == "pinhole":
            return cls.pinhole(hfov_deg or CANON_HFOV_DEG, size, aspect=aspect)
        return cls.equidistant(hfov_deg or DEFAULT_HFOV_DEG, size, aspect or DEFAULT_ASPECT)

    def to_dict(self) -> dict:
        return {"model": self.model, "K": self.K.tolist(), "D": self.D.tolist(), "size": list(self.size),
                "hfov_deg": round(self.hfov_deg(), 2), "vfov_deg": round(self.vfov_deg(), 2), "rms": self.rms,
                "source": self.source}

    def save(self, path: str) -> None:
        d = os.path.dirname(os.path.abspath(path))
        os.makedirs(d, exist_ok=True)
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=1)

    def scaled(self, size: Tuple[int, int]) -> "DartLens":
        """The same lens for another capture size (the K rows scale with the image)."""
        w, h = int(size[0]), int(size[1])
        if (w, h) == self.size:
            return self
        sx, sy = w / self.size[0], h / self.size[1]
        K = self.K.copy()
        K[0, 0] *= sx
        K[0, 2] = (K[0, 2] + 0.5) * sx - 0.5
        K[1, 1] *= sy
        K[1, 2] = (K[1, 2] + 0.5) * sy - 0.5
        return DartLens(K, self.D, (w, h), self.model, self.source, self.rms)

    # ------------------------------------------------------------ projection
    def _distort_theta(self, theta: np.ndarray) -> np.ndarray:
        t2 = theta * theta
        k1, k2, k3, k4 = self.D
        return theta * (1 + t2 * (k1 + t2 * (k2 + t2 * (k3 + t2 * k4))))

    def _undistort_theta(self, theta_d: np.ndarray) -> np.ndarray:
        theta = np.array(theta_d, dtype=np.float64)
        if not np.any(self.D):
            return theta
        for _ in range(20):      # Newton on theta_d(theta) = given
            t2 = theta * theta
            k1, k2, k3, k4 = self.D
            f = theta * (1 + t2 * (k1 + t2 * (k2 + t2 * (k3 + t2 * k4)))) - theta_d
            df = 1 + t2 * (3 * k1 + t2 * (5 * k2 + t2 * (7 * k3 + t2 * 9 * k4)))
            theta = theta - f / np.where(np.abs(df) < 1e-9, 1e-9, df)
        return theta

    def project(self, rays: np.ndarray) -> np.ndarray:
        """Camera-frame rays (N, 3), z forward, x right, y down -> pixels (N, 2). Rays behind give NaN for pinhole."""
        r = np.asarray(rays, dtype=np.float64).reshape(-1, 3)
        x, y, z = r[:, 0], r[:, 1], r[:, 2]
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        if self.model == "pinhole":
            with np.errstate(divide="ignore", invalid="ignore"):
                zz = np.where(z > 1e-9, z, np.nan)
                return np.column_stack([fx * x / zz + cx, fy * y / zz + cy])
        rho = np.hypot(x, y)
        theta = np.arctan2(rho, z)
        td = self._distort_theta(theta)
        with np.errstate(divide="ignore", invalid="ignore"):
            s = np.where(rho > 1e-12, td / rho, 0.0)
        return np.column_stack([fx * x * s + cx, fy * y * s + cy])

    def unproject(self, uv: np.ndarray) -> np.ndarray:
        """Pixels (N, 2) -> unit camera-frame rays (N, 3)."""
        p = np.asarray(uv, dtype=np.float64).reshape(-1, 2)
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        a = (p[:, 0] - cx) / fx
        b = (p[:, 1] - cy) / fy
        if self.model == "pinhole":
            r = np.column_stack([a, b, np.ones_like(a)])
        else:
            td = np.hypot(a, b)
            theta = self._undistort_theta(td)
            with np.errstate(divide="ignore", invalid="ignore"):
                s = np.where(td > 1e-12, np.sin(theta) / td, 1.0)
            r = np.column_stack([a * s, b * s, np.cos(theta)])
        return r / np.linalg.norm(r, axis=1, keepdims=True)

    def hfov_deg(self) -> float:
        w, h = self.size
        cy = self.K[1, 2]
        r = self.unproject(np.array([[-0.5, cy], [w - 0.5, cy]]))
        a0 = math.atan2(r[0, 0], r[0, 2])
        a1 = math.atan2(r[1, 0], r[1, 2])
        return math.degrees(a1 - a0)

    def vfov_deg(self) -> float:
        w, h = self.size
        cx = self.K[0, 2]
        r = self.unproject(np.array([[cx, -0.5], [cx, h - 0.5]]))
        return math.degrees(math.atan2(r[1, 1], r[1, 2]) - math.atan2(r[0, 1], r[0, 2]))

    # ------------------------------------------------------------ rectification
    def rectify_maps(self, out_size: Tuple[int, int] = CANON_SIZE, out_hfov_deg: float = CANON_HFOV_DEG):
        """cv2.remap maps from this lens to a square-pixel pinhole view of out_hfov_deg (cached)."""
        key = (tuple(out_size), float(out_hfov_deg))
        if key in self._maps:
            return self._maps[key]
        W, H = int(out_size[0]), int(out_size[1])
        f = (W / 2.0) / math.tan(math.radians(out_hfov_deg) / 2.0)
        uu, vv = np.meshgrid(np.arange(W, dtype=np.float64), np.arange(H, dtype=np.float64))
        rays = np.stack([(uu - (W - 1) / 2.0) / f, (vv - (H - 1) / 2.0) / f, np.ones_like(uu)], -1).reshape(-1, 3)
        p = self.project(rays).reshape(H, W, 2).astype(np.float32)
        p = np.nan_to_num(p, nan=-1e4)
        import cv2
        m1, m2 = cv2.convertMaps(p[..., 0], p[..., 1], cv2.CV_16SC2)
        self._maps[key] = (m1, m2)
        return m1, m2

    def is_canonical(self, out_size: Tuple[int, int] = CANON_SIZE, out_hfov_deg: float = CANON_HFOV_DEG) -> bool:
        """True when rectify() would not change anything (a square-pixel pinhole of the canonical FOV)."""
        if self.model != "pinhole":
            return False
        return abs(self.hfov_deg() - out_hfov_deg) < 0.5 and abs(self.K[0, 0] - self.K[1, 1]) < 1e-6 * self.K[0, 0]

    def rectify(self, img: np.ndarray, out_size: Optional[Tuple[int, int]] = None,
                out_hfov_deg: float = CANON_HFOV_DEG) -> np.ndarray:
        """The canonical pinhole view of one frame (black outside the lens' picture)."""
        import cv2
        h, w = img.shape[:2]
        lens = self.scaled((w, h))
        if out_size is None:
            out_size = (w, int(round(w * CANON_SIZE[1] / CANON_SIZE[0])))
        if lens.is_canonical(out_size, out_hfov_deg) and (w, h) == tuple(out_size):
            return img
        m1, m2 = lens.rectify_maps(out_size, out_hfov_deg)
        return cv2.remap(img, m1, m2, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)


def rectify_frames(frames: Sequence, lens: Optional[DartLens], out_hfov_deg: float = CANON_HFOV_DEG) -> list:
    """New Frame objects with rectified images (same t, idx, quality, bad). lens None: the frames unchanged."""
    if lens is None or not frames:
        return list(frames)
    from ..video.frame import Frame
    out = []
    for f in frames:
        img = getattr(f, "img", None)
        if img is None:
            out.append(f)
            continue
        if img.ndim == 2:
            import cv2
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        out.append(Frame(img=lens.rectify(img, out_hfov_deg=out_hfov_deg), t=f.t, idx=getattr(f, "idx", 0),
                         quality=getattr(f, "quality", None), bad=getattr(f, "bad", False)))
    return out
