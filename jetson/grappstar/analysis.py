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
        # landing frames whatever their bad flag (a black void or buried camera is marked bad by the link check,
        # and analyze_landing handles dark frames itself); never in-flight frames
        t_l = getattr(fres, "t_landed", None) if fres is not None else None
        land_frames = [f for f in frames if t_l is not None and f.t >= t_l][-15:] or frames[-15:]
    try:
        from .vision import terrain
        # No attitude hand-off: the dart rests nose down, so neither its pitch nor its roll about that long axis
        # says anything reliable about the ground slope (review: it turned good hard ground into NO GO).
        att = None
        rep = terrain.analyze_landing(land_frames, model=terrain_model, attitude=att)
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
