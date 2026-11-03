"""Landing terrain classifier evaluated on full simulated flights (not on standalone textures).

For each terrain class and seed a whole dart flight is rendered (sim.flight_video.render_flight, dropouts on),
then the landing verdict is computed exactly as analysis.run_post_flight does it:

    fres = flight.analyze_flight(frames, flightlog)        # once without and once with the truth flightlog
    land = flight.landing_frames(frames, fres, n=15)       # fallback: the last 15 good frames
    tilt = analysis.rest_tilt(flightlog, fres)           # roll-invariant rest tilt (None without a flightlog)
    rep = terrain.analyze_landing(land, imu_roll_deg=tilt and tilt["tilt_deg"], imu_pitch_deg=tilt and 0.0)

Usage (from jetson/):
    python tests/eval_terrain_flight.py                       # seeds 0..5, 720x480 and 480x320
    python tests/eval_terrain_flight.py --seeds 0 1 --sizes 480x320 --no-flightlog --jobs 4

Importable: evaluate(...) returns a dict with the per run rows and confusion matrices; the reduced version runs
in tests/test_terrain_flight.py.
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from grappstar.sim.flight_video import render_flight  # noqa: E402
from grappstar.vision import flight, terrain  # noqa: E402

CLASSES = terrain.CLASSES
PRED_NAMES = list(terrain.CLASSES) + ["unknown"]
SIZES = ((720, 480), (480, 320))


def _gamma(img, g):
    return np.clip(255.0 * (img / 255.0) ** g, 0, 255).astype(np.uint8)


# Robustness checks (--perturb): applied to every frame before the analysis, like a different camera would be.
PERTURB = {
    "none": lambda img, rng: img,
    "blur": lambda img, rng: cv2.GaussianBlur(img, (0, 0), 1.5),
    "noise": lambda img, rng: np.clip(img + rng.normal(0, 10, img.shape), 0, 255).astype(np.uint8),
    "dark": lambda img, rng: np.clip(img * 0.65, 0, 255).astype(np.uint8),
    "bright": lambda img, rng: _gamma(img, 0.7),
    "zoom": lambda img, rng: cv2.resize(img[img.shape[0] // 4:3 * img.shape[0] // 4,
                                            img.shape[1] // 4:3 * img.shape[1] // 4],
                                        (img.shape[1], img.shape[0]), interpolation=cv2.INTER_LINEAR),
    "lowres": lambda img, rng: cv2.resize(cv2.resize(img, (img.shape[1] // 2, img.shape[0] // 2),
                                                     interpolation=cv2.INTER_AREA),
                                          (img.shape[1], img.shape[0]), interpolation=cv2.INTER_LINEAR),
}


def perturb_frames(frames: Sequence, kind: str, seed: int = 0) -> list:
    if kind == "none":
        return list(frames)
    from grappstar.video.frame import Frame
    rng = np.random.default_rng(seed)
    fn = PERTURB[kind]
    return [Frame(fn(f.img, rng), f.t, f.idx, f.quality, f.bad) for f in frames]


def _tilt(flightlog, fres) -> Optional[float]:
    """Same IMU tilt hand-off as analysis.run_post_flight: the roll-invariant rest tilt, never the Euler roll."""
    from grappstar import analysis
    t = analysis.rest_tilt(flightlog, fres)
    return None if t is None else t["tilt_deg"]


def landing_report(frames: Sequence, flightlog: Optional[dict], model=None, make_image: bool = False,
                   use_attitude: bool = True):
    """analysis.run_post_flight's terrain step. Returns (TerrainReport, landing frames).

    use_attitude=False skips the IMU tilt hand-off (diagnostic: shows how much the tilt penalty changes the
    verdicts)."""
    fres = None
    try:
        fres = flight.analyze_flight(frames, flightlog)
    except Exception:
        fres = None
    land: list = []
    try:
        if fres is not None:
            land = flight.landing_frames(frames, fres, n=15)
    except Exception:
        land = []
    if not land:
        land = [f for f in frames if not getattr(f, "bad", False)][-15:]
    tilt = _tilt(flightlog, fres) if use_attitude else None
    if tilt is not None:
        rep = terrain.analyze_landing(land, model=model, imu_roll_deg=tilt, imu_pitch_deg=0.0, make_image=make_image)
    else:
        rep = terrain.analyze_landing(land, model=model, make_image=make_image)
    return rep, land


def run_case(cls: str, seed: int, size: Tuple[int, int], flightlog_modes: Iterable[bool] = (False, True),
             use_attitude: bool = True, perturb: str = "none") -> List[dict]:
    """Render one flight and analyse it once per flightlog mode. Returns one row per mode."""
    frames, truth = render_flight(seed=int(seed), landing_class=cls, size=tuple(size), dropouts=True)
    frames = perturb_frames(frames, perturb, seed)
    rows = []
    for use_log in flightlog_modes:
        t0 = time.perf_counter()
        rep, land = landing_report(frames, truth["flightlog"] if use_log else None, use_attitude=use_attitude)
        rows.append({"truth": cls, "seed": int(seed), "size": "%dx%d" % tuple(size), "flightlog": bool(use_log),
                     "pred": rep.cls, "verdict": rep.verdict, "score": int(rep.score),
                     "conf": float(rep.confidence), "n_land": len(land), "slope_deg": rep.slope_deg,
                     "probs": {k: round(float(v), 3) for k, v in rep.probs.items()},
                     "t_terrain_s": time.perf_counter() - t0})
    return rows


def _run_case_star(args):
    return run_case(*args)


def confusion(rows: Sequence[dict]) -> np.ndarray:
    cm = np.zeros((len(CLASSES), len(PRED_NAMES)), int)
    for r in rows:
        cm[CLASSES.index(r["truth"]), PRED_NAMES.index(r["pred"])] += 1
    return cm


def format_confusion(cm: np.ndarray, title: str = "") -> str:
    lines = []
    if title:
        lines.append(title)
    lines.append("truth \\ pred " + " ".join(n[:7].rjust(7) for n in PRED_NAMES))
    for i, c in enumerate(CLASSES):
        lines.append(c[:12].ljust(12) + " " + " ".join(str(x).rjust(7) for x in cm[i]))
    n = cm.sum()
    acc = cm.diagonal().sum() / n if n else 0.0
    lines.append("accuracy %.3f (%d/%d)" % (acc, cm.diagonal().sum(), n))
    return "\n".join(lines)


def accuracy(rows: Sequence[dict]) -> float:
    return float(np.mean([r["pred"] == r["truth"] for r in rows])) if rows else 0.0


def unsafe_go(rows: Sequence[dict]) -> List[dict]:
    """Runs with a GO verdict although the truth is sand_dust or water_void (must never happen)."""
    return [r for r in rows if r["truth"] in ("sand_dust", "water_void") and r["verdict"] == "GO"]


def evaluate(classes: Sequence[str] = CLASSES, seeds: Sequence[int] = range(6),
             sizes: Sequence[Tuple[int, int]] = SIZES, flightlog_modes: Sequence[bool] = (False, True),
             jobs: int = 1, verbose: bool = True, use_attitude: bool = True, perturb: str = "none") -> dict:
    tasks = [(c, s, tuple(sz), tuple(flightlog_modes), use_attitude, perturb) for sz in sizes for c in classes for s in seeds]
    t0 = time.time()
    if jobs > 1:
        import multiprocessing as mp
        with mp.get_context("fork").Pool(jobs) as pool:
            parts = pool.map(_run_case_star, tasks)
    else:
        parts = [run_case(*t) for t in tasks]
    rows = [r for p in parts for r in p]
    out = {"rows": rows, "by_size": {}, "elapsed_s": time.time() - t0}
    for sz in sizes:
        key = "%dx%d" % tuple(sz)
        rs = [r for r in rows if r["size"] == key]
        out["by_size"][key] = {"cm": confusion(rs), "acc": accuracy(rs), "unsafe_go": unsafe_go(rs)}
    if verbose:
        print(report_text(out))
    return out


def report_text(out: dict) -> str:
    lines = []
    for r in out["rows"]:
        slope = "-" if r["slope_deg"] is None else "%.0f" % r["slope_deg"]
        lines.append("%-11s seed %2d %-7s log=%d -> %-11s %-6s score %3d conf %.2f tilt %s (%d frames, pipeline %.0f ms)"
                     % (r["truth"], r["seed"], r["size"], r["flightlog"], r["pred"], r["verdict"], r["score"],
                        r["conf"], slope, r["n_land"], 1000 * r["t_terrain_s"]))
    for key, d in out["by_size"].items():
        lines.append("")
        lines.append(format_confusion(d["cm"], "confusion matrix %s (rows = truth, cols = predicted)" % key))
        verdicts: Dict[str, Dict[str, int]] = {}
        for r in out["rows"]:
            if r["size"] == key:
                verdicts.setdefault(r["truth"], {}).setdefault(r["verdict"], 0)
                verdicts[r["truth"]][r["verdict"]] += 1
        lines.append("verdicts: " + "; ".join("%s %s" % (c, verdicts.get(c, {})) for c in CLASSES))
        lines.append("unsafe GO (truth sand_dust or water_void): %d" % len(d["unsafe_go"]))
    lines.append("elapsed %.1f s" % out["elapsed_s"])
    return "\n".join(lines)


def _parse_size(s: str) -> Tuple[int, int]:
    w, h = s.lower().split("x")
    return int(w), int(h)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--seeds", type=int, nargs="*", default=list(range(6)))
    ap.add_argument("--classes", nargs="*", default=list(CLASSES))
    ap.add_argument("--sizes", type=_parse_size, nargs="*", default=list(SIZES))
    ap.add_argument("--no-flightlog", action="store_true", help="only run without the truth flightlog")
    ap.add_argument("--no-attitude", action="store_true",
                    help="diagnostic: do not pass the landed attitude (no tilt penalty)")
    ap.add_argument("--perturb", default="none", choices=sorted(PERTURB),
                    help="robustness check: alter every frame (blur, noise, exposure, zoom, resolution)")
    ap.add_argument("--jobs", type=int, default=max(1, min(4, os.cpu_count() or 1)))
    a = ap.parse_args(argv)
    modes = (False,) if a.no_flightlog else (False, True)
    evaluate(a.classes, a.seeds, a.sizes, modes, jobs=a.jobs, use_attitude=not a.no_attitude,
             perturb=a.perturb)
    return 0


if __name__ == "__main__":
    sys.exit(main())
