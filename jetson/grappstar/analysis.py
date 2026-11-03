"""Post-flight analysis: flight phases, landing zone verdict and aerial map from the dart video and flightlog.

Each step is guarded: a vision failure never blocks the mission; it gives an UNSURE verdict with the reason, and
the operator decides.
"""
from __future__ import annotations

import json
import os
import time
import traceback
from typing import Any, Dict, List, Optional

G = 9.81
# Resting poses the dart takes on FLAT ground, as nose elevation (deg, negative = nose down). Lying flat on its
# belly (it skids and topples) or nose buried at about -60 deg (kedge landing, the flight sim's -54..-66). The tilt
# cue is the distance of the measured rest elevation from the nearest of these. ASSUMPTION: confirm with real
# landings and edit this tuple (settings could override it later).
REST_POSES_DEG = (0.0, -60.0)


def rest_tilt(flightlog: Optional[dict], fres=None) -> Optional[dict]:
    """Roll-invariant tilt cue of the landed dart.

    Uses the still gravity vector in the body frame after landing: the accelerometer reads the specific force
    f_b = R^T (0, 0, g), so the nose elevation is asin(f_b.x / |f_b|), whatever the spin about the nose (roll).
    Falls back to the quaternion pitch of the resting samples (also roll-invariant: R20 = sin(pitch)).
    Returns {"nose_elev_deg", "tilt_deg", "expected_deg", "source", "n"} or None without resting data."""
    import math
    import numpy as np
    s = (flightlog or {}).get("samples") if isinstance(flightlog, dict) else None
    rows = [r for r in (s or []) if isinstance(r, (list, tuple)) and len(r) >= 8]
    if len(rows) < 5:
        return None
    a = np.array([r[:8] for r in rows], float)
    a = a[np.argsort(a[:, 0])]
    t = a[:, 0] / 1000.0
    t_rest = None
    if fres is not None and getattr(fres, "imu_times", None):
        t_rest = fres.imu_times.get("landed")
    if t_rest is None and isinstance(flightlog.get("t_land_ms"), (int, float)):
        t_rest = flightlog["t_land_ms"] / 1000.0 + 0.5
    sel = a[t >= t_rest] if t_rest is not None else a[-25:]
    if len(sel) < 3:
        sel = a[-10:]
    f = sel[:, 5:8]
    mag = np.linalg.norm(f, axis=1)
    still = np.abs(mag - G) < 0.08 * G
    if still.sum() >= 3:
        fm = np.median(f[still], axis=0)
        elev = math.degrees(math.asin(max(-1.0, min(1.0, fm[0] / (np.linalg.norm(fm) or 1.0)))))
        src = "accelerometer"
        n = int(still.sum())
    else:
        q = sel[:, 1:5]
        q = q / np.maximum(np.linalg.norm(q, axis=1, keepdims=True), 1e-9)
        r20 = 2 * (q[:, 1] * q[:, 3] - q[:, 0] * q[:, 2])
        elev = math.degrees(math.asin(float(np.clip(np.median(r20), -1, 1))))
        src = "quaternion"
        n = len(sel)
    exp = min(REST_POSES_DEG, key=lambda e: abs(elev - e))
    return {"nose_elev_deg": elev, "tilt_deg": abs(elev - exp), "expected_deg": exp, "source": src, "n": n}


def _save_png(path: str, img) -> Optional[str]:
    try:
        import cv2
        if img is None:
            return None
        cv2.imwrite(path, img)
        return path
    except Exception:
        return None


def run_post_flight(frames: List[Any], flightlog: Optional[dict], outdir: Optional[str], t_fire: Optional[float] = None,
                    terrain_model=None, do_mosaic: bool = True, log=None,
                    clock_offset_s: Optional[float] = None) -> Dict[str, Any]:
    """Returns {"flight": dict|None, "terrain": dict, "mosaic": dict|None, "files": {...}, "errors": [...]}."""
    out: Dict[str, Any] = {"flight": None, "terrain": None, "mosaic": None, "files": {}, "errors": []}
    if outdir:
        os.makedirs(outdir, exist_ok=True)

    def note(msg):
        out["errors"].append(msg)
        if log:
            log(msg)

    good = [f for f in frames if not getattr(f, "bad", False)]
    t0 = time.time()
    fres = None
    try:
        from .vision import flight
        if flightlog is not None and clock_offset_s is None:
            clock_offset_s = flightlog.get("clock_offset_s")
        fres = flight.analyze_flight(frames, flightlog, clock_offset_s=clock_offset_s)
        out["flight"] = fres.to_dict()
    except Exception as e:
        note(f"flight analysis failed: {type(e).__name__}: {e}")
        out["flight_trace"] = traceback.format_exc(limit=3)
    # landing frames: from the flight result, or simply the last good frames
    land_frames = []
    try:
        if fres is not None:
            from .vision import flight
            land_frames = flight.landing_frames(frames, fres, n=15)
    except Exception as e:
        note(f"landing frame pick failed: {e}")
    if not land_frames:
        land_frames = good[-15:]
    try:
        from .vision import terrain
        # Tilt cue from the IMU: never the Euler roll (spin about the nose, arbitrary for a dart), only the
        # roll-invariant nose elevation from the resting gravity vector, against the expected flat-ground pose.
        tilt = None
        try:
            tilt = rest_tilt(flightlog, fres)
        except Exception as e:
            note(f"rest tilt failed: {e}")
        out["rest_tilt"] = tilt
        if tilt is not None:
            rep = terrain.analyze_landing(land_frames, model=terrain_model, imu_roll_deg=tilt["tilt_deg"],
                                          imu_pitch_deg=0.0)
        else:
            rep = terrain.analyze_landing(land_frames, model=terrain_model)
        out["terrain"] = rep.to_dict()
        if outdir and getattr(rep, "image", None) is not None:
            p = _save_png(os.path.join(outdir, "landing.png"), rep.image)
            if p:
                out["files"]["landing.png"] = p
    except Exception as e:
        note(f"terrain analysis failed: {type(e).__name__}: {e}")
        out["terrain"] = {"cls": "unknown", "score": 0, "verdict": "UNSURE", "probs": {},
                          "reasons": [f"Landing zone analysis failed ({type(e).__name__}). Look at the video."]}
    if do_mosaic:
        try:
            from .vision import mosaic
            lt = fres.t_impact if fres is not None and getattr(fres, "t_impact", None) else None
            import inspect
            if "flight" in inspect.signature(mosaic.build_mosaic).parameters:
                m = mosaic.build_mosaic(frames, landing_t=lt, flight=fres)
            else:
                m = mosaic.build_mosaic(frames, landing_t=lt)
            out["mosaic"] = m.to_dict()
            if outdir and getattr(m, "image", None) is not None:
                p = _save_png(os.path.join(outdir, "mosaic.png"), m.image)
                if p:
                    out["files"]["mosaic.png"] = p
        except Exception as e:
            note(f"mosaic failed: {type(e).__name__}: {e}")
    out["n_frames"] = len(frames)
    out["n_good"] = len(good)
    out["elapsed_s"] = round(time.time() - t0, 2)
    if outdir:
        p = os.path.join(outdir, "flight.json")
        try:
            with open(p, "w") as f:
                from .jlog import _clean
                json.dump(_clean({k: v for k, v in out.items() if k != "files"}), f, indent=1)
            out["files"]["flight.json"] = p
        except Exception as e:
            note(f"could not write flight.json: {e}")
    return out
