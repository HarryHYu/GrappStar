"""Rover camera calibration helpers (Razer Kiyo X, or any UVC camera).

Two steps:

1. Intrinsics (focal length, principal point, lens distortion) from 10 to 20 photos of a
   printed checkerboard held at different angles and positions. Default board: 9x6 inner
   corners, 25 mm squares (OpenCV's standard pattern). Do this once per camera and
   resolution:

       python -m grappstar.vision.calibrate intrinsics --images 'calib/*.jpg' --out cam_intr.json

2. Mounting (height above ground, pitch down, roll) once the camera is fixed on the rover:

   a) Checkerboard lying FLAT on the ground in front of the rover (best). solvePnP gives the
      camera pose relative to the board plane, so height, pitch and roll come out directly:

       python -m grappstar.vision.calibrate ground --image board_on_ground.jpg \\
           --intrinsics cam_intr.json --square 0.025 --out rover_cam.json

   b) An orange marker standing on flat ground at a measured distance straight ahead of
      the camera, with the camera height measured with a tape. Solves the pitch:

       python -m grappstar.vision.calibrate marker --image marker.jpg --distance 5.0 \\
           --height 0.42 --intrinsics cam_intr.json --out rover_cam.json

If nothing is calibrated, default_camera() gives Kiyo X intrinsics estimated from the 82
degree diagonal FOV at 1280x720 (fx = fy, about 845 px). That model has
uncalibrated=True and a warning: ranges from it are rough.

    python -m grappstar.vision.calibrate default --height 0.4 --pitch 12 --out rover_cam.json

All functions return plain dicts or a CameraModel (grappstar.vision.target).
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import sys
from typing import Iterable, List, Optional, Sequence, Tuple, Union

import cv2
import numpy as np

from grappstar.vision.target import CameraModel, detect_orange

DEFAULT_PATTERN = (9, 6)      # inner corners (columns, rows)
DEFAULT_SQUARE_M = 0.025
KIYO_X_DIAG_FOV_DEG = 82.0
UNCALIBRATED_WARNING = ("Camera intrinsics are NOT calibrated: using a Kiyo X estimate from the "
                        "82 degree diagonal FOV. Ranges may be off by 10 percent or more.")


# ================================================================ defaults

def kiyo_default_intrinsics(width: int = 1280, height: int = 720,
                            diag_fov_deg: float = KIYO_X_DIAG_FOV_DEG) -> dict:
    """Pinhole intrinsics estimated from the diagonal FOV (square pixels, centred, no distortion)."""
    diag = math.hypot(width, height)
    f = (diag / 2.0) / math.tan(math.radians(diag_fov_deg) / 2.0)
    return {"fx": f, "fy": f, "cx": (width - 1) / 2.0, "cy": (height - 1) / 2.0, "dist": None,
            "calib_size": [int(width), int(height)], "uncalibrated": True,
            "warning": UNCALIBRATED_WARNING}


def default_camera(height_m: float = 0.4, pitch_deg: float = 10.0, roll_deg: float = 0.0,
                   width: int = 1280, height: int = 720) -> CameraModel:
    """Uncalibrated Kiyo X model; check .uncalibrated and show .notes to the operator."""
    return camera_from(kiyo_default_intrinsics(width, height),
                       {"height_m": height_m, "pitch_deg": pitch_deg, "roll_deg": roll_deg})


def camera_from(intr: Union[dict, CameraModel], mounting: dict) -> CameraModel:
    """Combine intrinsics (dict or CameraModel) and a mounting dict into a CameraModel."""
    if isinstance(intr, CameraModel):
        intr = intr.to_dict()
    unc = bool(intr.get("uncalibrated", False))
    return CameraModel(fx=float(intr["fx"]), fy=float(intr["fy"]), cx=float(intr["cx"]),
                       cy=float(intr["cy"]), height_m=float(mounting["height_m"]),
                       pitch_deg=float(mounting["pitch_deg"]),
                       roll_deg=float(mounting.get("roll_deg", 0.0)),
                       dist=None if intr.get("dist") is None else [float(x) for x in intr["dist"]],
                       calib_size=tuple(intr["calib_size"]) if intr.get("calib_size") else None,
                       yaw_deg=float(mounting.get("yaw_deg", 0.0)), uncalibrated=unc,
                       notes=UNCALIBRATED_WARNING if unc else "")


def intrinsics_for_size(intr: Union[dict, CameraModel], width: int, height: int) -> dict:
    """Intrinsics dict scaled to an image of width x height (no-op if it already matches)."""
    cam = camera_from(intr, {"height_m": 1.0, "pitch_deg": 0.0})
    out = dict(intr.to_dict() if isinstance(intr, CameraModel) else intr)
    s = cam.for_size(int(width), int(height))
    out.update(fx=s.fx, fy=s.fy, cx=s.cx, cy=s.cy,
               calib_size=list(s.calib_size) if s.calib_size else [int(width), int(height)])
    return out


def _K(intr: Union[dict, CameraModel]) -> Tuple[np.ndarray, np.ndarray]:
    if isinstance(intr, CameraModel):
        intr = intr.to_dict()
    K = np.array([[intr["fx"], 0, intr["cx"]], [0, intr["fy"], intr["cy"]], [0, 0, 1]], np.float64)
    d = np.zeros(5) if intr.get("dist") is None else np.array(intr["dist"], np.float64)
    return K, d


# ================================================================ intrinsics

def board_points(pattern=DEFAULT_PATTERN, square_m: float = DEFAULT_SQUARE_M) -> np.ndarray:
    """Object points of the inner corners in the board frame (metres, z = 0)."""
    cols, rows = pattern
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_m
    return objp


def find_corners(img: np.ndarray, pattern=DEFAULT_PATTERN) -> Optional[np.ndarray]:
    """Sub-pixel checkerboard inner corners (N x 1 x 2 float32) or None."""
    grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(grey, tuple(pattern), flags)
    if not ok:
        return None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4)
    return cv2.cornerSubPix(grey, corners, (5, 5), (-1, -1), crit)


def calibrate_intrinsics(images: Iterable[np.ndarray], pattern=DEFAULT_PATTERN,
                         square_m: float = DEFAULT_SQUARE_M, fix_k3: bool = True) -> dict:
    """Intrinsics from checkerboard images. Needs at least 3 detections (10+ recommended).

    Returns {fx, fy, cx, cy, dist, rms, calib_size, n_images, n_used, uncalibrated: False}.
    """
    objp = board_points(pattern, square_m)
    obj, img_pts, size, n = [], [], None, 0
    for img in images:
        n += 1
        if img is None:
            continue
        h, w = img.shape[:2]
        if size is None:
            size = (w, h)
        elif size != (w, h):
            raise ValueError("all calibration images must have the same size")
        c = find_corners(img, pattern)
        if c is not None:
            obj.append(objp)
            img_pts.append(c)
    if len(obj) < 3:
        raise ValueError("checkerboard found in only %d of %d images (need 3 or more)" % (len(obj), n))
    flags = cv2.CALIB_FIX_K3 if fix_k3 else 0
    rms, K, dist, _, _ = cv2.calibrateCamera(obj, img_pts, size, None, None, flags=flags)
    return {"fx": float(K[0, 0]), "fy": float(K[1, 1]), "cx": float(K[0, 2]), "cy": float(K[1, 2]),
            "dist": [float(x) for x in dist.ravel()], "rms": float(rms),
            "calib_size": [int(size[0]), int(size[1])], "n_images": n, "n_used": len(obj),
            "uncalibrated": False}


def save_json(d: dict, path: str) -> None:
    with open(path, "w") as f:
        json.dump(d, f, indent=1)


def load_json(path: str) -> dict:
    with open(path) as f:
        return json.load(f)


# ================================================================ mounting

def mounting_from_ground_board(img: np.ndarray, intr: Union[dict, CameraModel],
                               pattern=DEFAULT_PATTERN, square_m: float = DEFAULT_SQUARE_M,
                               board_thickness_m: float = 0.0) -> dict:
    """Height, pitch and roll from one image of a checkerboard lying flat on the ground.

    Returns {height_m, pitch_deg, roll_deg, board_range_m, reproj_px}. board_thickness_m is
    added to the height (the board surface sits above the ground by its thickness).
    Yaw is not returned: the board corner order is ambiguous under rotation, so align the
    launcher by aiming at a marker instead.
    """
    c = find_corners(img, pattern)
    if c is None:
        raise ValueError("checkerboard not found")
    K, d = _K(intrinsics_for_size(intr, img.shape[1], img.shape[0]))
    objp = board_points(pattern, square_m)
    ok, rvec, tvec = cv2.solvePnP(objp, c, K, d, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise ValueError("solvePnP failed")
    R, _ = cv2.Rodrigues(rvec)
    t = tvec.reshape(3)
    C = -R.T @ t                        # camera centre in the board frame
    up = np.array([0.0, 0.0, 1.0]) * (1.0 if C[2] >= 0 else -1.0)
    x_cam, z_cam = R[0, :], R[2, :]     # camera axes in the board frame
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -float(z_cam @ up)))))
    cp = math.cos(math.radians(pitch))
    roll = math.degrees(math.asin(max(-1.0, min(1.0, -float(x_cam @ up) / max(1e-6, cp)))))
    proj, _ = cv2.projectPoints(objp, rvec, tvec, K, d)
    err = float(np.sqrt(((proj.reshape(-1, 2) - c.reshape(-1, 2)) ** 2).sum(1).mean()))
    centre = objp.mean(0)
    horiz = (centre - C)[:2]
    return {"height_m": float(abs(C[2])) + float(board_thickness_m), "pitch_deg": pitch,
            "roll_deg": roll, "board_range_m": float(np.linalg.norm(horiz)), "reproj_px": err}


def pitch_from_marker(u: float, v: float, distance_m: float, height_m: float,
                      intr: Union[dict, CameraModel], roll_deg: float = 0.0) -> dict:
    """Solve the camera pitch so that pixel (u, v) lands at distance_m on flat ground.

    The marker should stand straight ahead of the camera (then the yaw estimate is also
    meaningful: the bearing the model reports for the marker, with sign flipped).
    distance_m is the horizontal distance from the point on the ground directly below the
    camera lens to the marker base. Returns {pitch_deg, yaw_deg, height_m, roll_deg}.
    """
    base = camera_from(intr, {"height_m": height_m, "pitch_deg": 0.0, "roll_deg": roll_deg})

    def rng_at(p):
        base.pitch_deg = p
        r = base.pixel_to_ground(u, v)
        return float("inf") if r is None else r[0]

    lo, hi = -30.0, 89.0         # range decreases as pitch increases
    if not (rng_at(hi) <= distance_m <= rng_at(lo)):
        if rng_at(lo) < distance_m:
            raise ValueError("marker pixel too low in the image for that distance and height")
        raise ValueError("cannot reach that distance with any pitch")
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if rng_at(mid) > distance_m:
            lo = mid
        else:
            hi = mid
    pitch = 0.5 * (lo + hi)
    base.pitch_deg = pitch
    b = base.pixel_to_ground(u, v)
    return {"pitch_deg": pitch, "yaw_deg": -b[1] if b else 0.0, "height_m": float(height_m),
            "roll_deg": float(roll_deg)}


def mounting_from_marker(img: np.ndarray, distance_m: float, height_m: float,
                         intr: Union[dict, CameraModel], roll_deg: float = 0.0) -> dict:
    """Detect the orange marker in img (bottom centre) and solve the pitch."""
    det = detect_orange(img)
    if det is None:
        raise ValueError("no orange marker found")
    u, v, area = det
    intr = intrinsics_for_size(intr, img.shape[1], img.shape[0])
    out = pitch_from_marker(u, v, distance_m, height_m, intr, roll_deg)
    out.update(marker_u=u, marker_v=v, marker_area_px=area)
    return out


# ================================================================ CLI

def _read_images(patterns: Sequence[str]) -> List[np.ndarray]:
    paths = []
    for p in patterns:
        paths.extend(sorted(glob.glob(p)) or [p])
    imgs = [cv2.imread(p) for p in paths]
    return [i for i in imgs if i is not None]


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m grappstar.vision.calibrate",
                                 description="Rover camera calibration helpers")
    sub = ap.add_subparsers(dest="cmd", required=True)
    a = sub.add_parser("intrinsics", help="checkerboard intrinsics from several images")
    a.add_argument("--images", nargs="+", required=True)
    a.add_argument("--cols", type=int, default=DEFAULT_PATTERN[0])
    a.add_argument("--rows", type=int, default=DEFAULT_PATTERN[1])
    a.add_argument("--square", type=float, default=DEFAULT_SQUARE_M, help="square size in metres")
    a.add_argument("--out", default="cam_intrinsics.json")
    g = sub.add_parser("ground", help="height/pitch/roll from a board lying on the ground")
    g.add_argument("--image", required=True)
    g.add_argument("--intrinsics", help="JSON from 'intrinsics' (default: Kiyo X estimate)")
    g.add_argument("--cols", type=int, default=DEFAULT_PATTERN[0])
    g.add_argument("--rows", type=int, default=DEFAULT_PATTERN[1])
    g.add_argument("--square", type=float, default=DEFAULT_SQUARE_M)
    g.add_argument("--thickness", type=float, default=0.0, help="board thickness in metres")
    g.add_argument("--out", default="rover_cam.json")
    m = sub.add_parser("marker", help="pitch from an orange marker at a known distance")
    m.add_argument("--image", required=True)
    m.add_argument("--distance", type=float, required=True)
    m.add_argument("--height", type=float, required=True)
    m.add_argument("--intrinsics")
    m.add_argument("--out", default="rover_cam.json")
    d = sub.add_parser("default", help="write an uncalibrated Kiyo X model")
    d.add_argument("--height", type=float, default=0.4)
    d.add_argument("--pitch", type=float, default=10.0)
    d.add_argument("--width", type=int, default=1280)
    d.add_argument("--img-height", type=int, default=720)
    d.add_argument("--out", default="rover_cam.json")
    args = ap.parse_args(argv)

    if args.cmd == "intrinsics":
        res = calibrate_intrinsics(_read_images(args.images), (args.cols, args.rows), args.square)
        save_json(res, args.out)
        print(json.dumps(res, indent=1))
        return 0
    if args.cmd == "default":
        cam = default_camera(args.height, args.pitch, 0.0, args.width, args.img_height)
        cam.save(args.out)
        print("WARNING: " + UNCALIBRATED_WARNING)
        print(json.dumps(cam.to_dict(), indent=1))
        return 0
    img = cv2.imread(args.image)
    if img is None:
        print("cannot read %s" % args.image, file=sys.stderr)
        return 2
    if args.intrinsics:
        intr = load_json(args.intrinsics)
    else:
        intr = kiyo_default_intrinsics(img.shape[1], img.shape[0])
        print("WARNING: " + UNCALIBRATED_WARNING)
    if args.cmd == "ground":
        mnt = mounting_from_ground_board(img, intr, (args.cols, args.rows), args.square, args.thickness)
    else:
        mnt = mounting_from_marker(img, args.distance, args.height, intr)
    cam = camera_from(intr, mnt)
    cam.save(args.out)
    print(json.dumps({"mounting": mnt, "camera": cam.to_dict()}, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
