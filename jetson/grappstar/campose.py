"""Rover camera pose from the base spec (base/spec/params.py FRAMEWORK), with fallbacks.

The Kiyo X is mounted ON the base frame, looking forward along the launch azimuth (bore-sighted to the launcher), so
its height and downward tilt are design numbers in params.py. Ranges for the ballistics are measured from the launcher
pivot, so the camera's forward offset from the pivot is added to the camera range.

Order of trust: a calibration file (measured) > params.py FRAMEWORK (design) > default guesses.
The params entry is searched flexibly because the cad-frame worker names it: any dict inside FRAMEWORK whose key
contains "cam" or "kiyo" and that has a height (z or height, mm) and a tilt (tilt_deg, pitch_deg or tilt, deg down).
Optional: x (mm, forward of the world origin) and pivot x from LAUNCH (pivot or pivot_x).
"""
from __future__ import annotations

import importlib.util
import os
from typing import Optional

HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS = os.path.normpath(os.path.join(HERE, "..", "..", "base", "spec", "params.py"))


def load_params(path: str = PARAMS):
    if not os.path.exists(path):
        return None
    spec = importlib.util.spec_from_file_location("grappstar_base_params", path)
    mod = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(mod)
    except Exception:
        return None
    return mod


def _num(d: dict, *keys) -> Optional[float]:
    for k in keys:
        v = d.get(k)
        if isinstance(v, (int, float)):
            return float(v)
        if isinstance(v, (tuple, list)) and len(v) == 3 and k in ("pos", "xyz", "origin"):
            return None
    return None


DEFAULT_DECK_HEIGHT_M = 0.15   # base deck top above the ground on the rover: MEASURE (settings.json deck_height_m)


def find_camera_pose(params, deck_height_m: float = DEFAULT_DECK_HEIGHT_M) -> Optional[dict]:
    """{height_m, pitch_deg, x_m, pivot_x_m, source} from params.FRAMEWORK, or None."""
    fw = getattr(params, "FRAMEWORK", None) if params is not None else None
    if not isinstance(fw, dict):
        return None
    cam = fw.get("camera")
    if isinstance(cam, dict) and isinstance(cam.get("pos"), (tuple, list)) and isinstance(cam.get("pitch_deg"), (int, float)):
        # base spec v1.0: pos (x, y, z) mm in the world frame whose origin is on the launcher pivot axis
        # (+X = launch azimuth, Z up); pitch_deg negative = looking down. deck_z = deck top z (mm).
        x, y, z = (float(v) for v in cam["pos"])
        deck_z = fw.get("deck_z")
        if isinstance(deck_z, (int, float)):
            h = deck_height_m + (z - float(deck_z)) / 1000.0
            note = f"deck {deck_height_m:.3f} m above ground (setting) + {(z - float(deck_z)):.0f} mm"
        else:
            h = deck_height_m + 0.265
            note = "deck_z missing: assumed 265 mm above the deck"
        return {"height_m": h, "pitch_deg": -float(cam["pitch_deg"]), "x_m": x / 1000.0, "pivot_x_m": 0.0,
                "y_m": y / 1000.0, "source": f"params.FRAMEWORK.camera ({note})"}
    cands = []

    def walk(d, depth=0):
        if not isinstance(d, dict) or depth > 3:
            return
        for k, v in d.items():
            if isinstance(v, dict):
                if any(w in str(k).lower() for w in ("cam", "kiyo")):
                    cands.append((k, v))
                walk(v, depth + 1)
    walk(fw)
    for k, v in cands:
        pos = v.get("pos") or v.get("xyz") or v.get("origin")
        z = _num(v, "z", "height", "z_mm", "height_mm")
        x = _num(v, "x", "x_mm")
        if isinstance(pos, (tuple, list)) and len(pos) == 3:
            x = float(pos[0]) if x is None else x
            z = float(pos[2]) if z is None else z
        tilt = _num(v, "tilt_deg", "pitch_deg", "tilt", "down_deg")
        if z is None or tilt is None:
            continue
        pivot = None
        launch = getattr(params, "LAUNCH", None)
        if isinstance(launch, dict):
            p = launch.get("pivot") or launch.get("pivot_xyz")
            if isinstance(p, (tuple, list)) and len(p) == 3:
                pivot = float(p[0])
            elif isinstance(launch.get("pivot_x"), (int, float)):
                pivot = float(launch["pivot_x"])
        return {"height_m": z / 1000.0, "pitch_deg": tilt, "x_m": (x or 0.0) / 1000.0,
                "pivot_x_m": (pivot / 1000.0) if pivot is not None else None, "source": f"params.FRAMEWORK[{k!r}]"}
    return None


def camera_model(calib_path: Optional[str], size=(1280, 720), params_path: str = PARAMS,
                 deck_height_m: float = DEFAULT_DECK_HEIGHT_M):
    """CameraModel plus a note on where its pose came from. Adds .range_offset_m (camera ahead of the pivot)."""
    from .vision.target import CameraModel
    from .sim.world import default_rover_model
    if calib_path and os.path.exists(calib_path):
        m = CameraModel.load(calib_path)
        m.notes = (m.notes or "") + " (calibration file)"
        src = "calibration file"
    else:
        m = default_rover_model(*size)
        src = "default guesses"
    pose = find_camera_pose(load_params(params_path), deck_height_m)
    offset = 0.0
    if pose:
        if src != "calibration file":
            m.height_m, m.pitch_deg = pose["height_m"], pose["pitch_deg"]
            src = pose["source"] + " with default intrinsics"
        if pose.get("pivot_x_m") is not None:
            offset = pose["x_m"] - pose["pivot_x_m"]
    m.range_offset_m = offset
    m.pose_source = src
    return m
