"""Whole post-flight analysis on REALISTIC synthetic dart flights (the numbers behind the thresholds).

Each case renders a flight with sim.flight_video.render_flight(profile="caddx"): the real launcher (about 6.9 m/s,
2.6 to 4.8 m, C3 table speed with a 3 percent spread), the Caddx Ant fisheye with an unknown true field of view
(125 to 150 deg), a 4:3 picture in 720x480 (non-square pixels), auto exposure lag, motion blur, sensor and FM link
noise, sparkles, line jitter, interlaced fields, YUYV capture, 60 ms capture delay, repeated frames, dropouts,
torn and snow frames, and daylight, overcast, low light or a weak link. Then analysis.run_post_flight runs exactly
as the mission calls it (lens model from settings: the default 135 deg equidistant, or the calibrated true lens).

Measured per case: video launch and impact timing errors, range error (video only with and without the commanded
pitch, and with the IMU flight log), terrain verdict, aerial map landing marker error.

Usage (from jetson/):
    python tests/eval_realistic.py --n 48 --jobs 4                # prints a summary and per case rows
    python tests/eval_realistic.py --n 24 --size 720x480 --lens true
Importable: run_case(), evaluate(), summarise(). tests/test_realistic_flight.py runs a reduced set and asserts the
accuracy the README quotes.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from typing import Dict, List, Optional, Sequence

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from grappstar.analysis import run_post_flight  # noqa: E402
from grappstar.sim.flight_video import render_flight  # noqa: E402
from grappstar.vision.lens import DartLens  # noqa: E402

CLASSES = ("bedrock", "hard_ground", "loose_soil", "sand_dust", "vegetation", "water_void")
LINKS = ("daylight", "daylight", "overcast", "lowlight", "weak_link")
# what the verdict should be for each true class: GO only on hard ground and loose soil; never GO on sand, rock
# or water; vegetation is UNSURE (operator decides) but GO or NO_GO are not counted as unsafe
EXPECT = {"hard_ground": "GO", "loose_soil": "GO", "vegetation": "UNSURE", "sand_dust": "NO_GO",
          "bedrock": "NO_GO", "water_void": "NO_GO"}
UNSAFE_GO = ("sand_dust", "bedrock", "water_void")


def case_params(seed: int) -> dict:
    """Reproducible random conditions for one case."""
    rng = np.random.default_rng([int(seed), 31337])
    return {
        "seed": int(seed),
        "cls": CLASSES[int(seed) % len(CLASSES)],
        "link": LINKS[int(rng.integers(len(LINKS)))],
        "lens_hfov": float(rng.uniform(125.0, 150.0)),
        "rest": "flat" if rng.random() < 0.8 else "buried",
    }


def run_case(seed: int, size=(360, 240), lens: str = "default", mosaic: bool = False, verbose: bool = False,
             **over) -> dict:
    p = case_params(seed)
    p.update(over)
    t0 = time.time()
    frames, truth = render_flight(seed=p["seed"], size=tuple(size), profile="caddx", landing_class=p["cls"],
                                  link=p["link"], lens_hfov_deg=p["lens_hfov"], rest=p["rest"],
                                  pitch_deg=p.get("pitch"), v0=p.get("v0"))
    t_render = time.time() - t0
    if lens == "true":
        L = truth["lens"]
        dl = DartLens(L["K"], L["D"], tuple(L["size"]), "fisheye")
    else:
        dl = DartLens.equidistant(size=tuple(size))
    lat = truth["video_latency_s"]
    tl, ti = truth["t_launch"] + lat, truth["t_impact"] + lat
    R = truth["range_pivot_m"]
    row = dict(p, pitch=truth["launch_pitch_deg"], v0=truth["v0"], range=R, t_flight=truth["t_impact"] - truth["t_launch"],
               render_s=round(t_render, 1))
    runs = {"vid_aim": (None, truth["launch_pitch_deg"]), "vid": (None, None), "imu": (truth["flightlog"], None)}
    for name, (fl, aim) in runs.items():
        res = run_post_flight(frames, fl, None, aim_pitch_deg=aim, dart_lens=dl,
                              clock_offset_s=truth["clock_offset_s"] if fl else None, video_latency_s=0.06,
                              do_mosaic=mosaic and name == "imu")
        f = res.get("flight") or {}
        rng_m = f.get("range_m")
        row[name + "_range"] = rng_m
        row[name + "_err"] = None if rng_m is None else rng_m / R - 1.0
        tr = res.get("terrain") or {}
        row[name + "_verdict"] = tr.get("verdict")
        row[name + "_cls"] = tr.get("cls")
        row[name + "_score"] = tr.get("score")
        if name == "vid":
            vt = f.get("video_times") or {}
            row["vid_launch_err"] = None if vt.get("launch") is None else vt["launch"] - tl
            row["vid_impact_err"] = None if vt.get("impact") is None else vt["impact"] - ti
        if name == "imu":
            row["imu_latency"] = f.get("video_latency_s")
            row["imu_impact_err"] = None if f.get("t_impact") is None else f["t_impact"] - ti
            m = res.get("mosaic") or {}
            if mosaic:
                row["mosaic_used"] = len(m.get("used") or [])
                row["mosaic_source"] = m.get("landing_source")
        if verbose:
            print(seed, name, f.get("notes"), res.get("errors"))
    return row


def _one(args):
    seed, size, lens, mosaic = args
    try:
        return run_case(seed, size, lens, mosaic)
    except Exception as e:     # a crash is a result too
        import traceback
        return {"seed": seed, "error": f"{type(e).__name__}: {e}", "trace": traceback.format_exc(limit=4)}


def evaluate(seeds: Sequence[int], size=(360, 240), lens: str = "default", jobs: int = 1,
             mosaic: bool = False) -> List[dict]:
    tasks = [(int(s), tuple(size), lens, mosaic) for s in seeds]
    if jobs > 1:
        import multiprocessing as mp
        with mp.get_context("fork").Pool(jobs) as pool:
            return pool.map(_one, tasks)
    return [_one(t) for t in tasks]


def _pct(xs, q):
    xs = [x for x in xs if x is not None]
    return float(np.percentile(xs, q)) if xs else float("nan")


def summarise(rows: List[dict]) -> dict:
    ok = [r for r in rows if "error" not in r]
    out = {"n": len(rows), "crashed": len(rows) - len(ok)}
    for k in ("vid_aim_err", "vid_err", "imu_err"):
        e = [abs(r[k]) for r in ok if r.get(k) is not None]
        out[k] = {"n": len(e), "median_abs": _pct(e, 50), "p90_abs": _pct(e, 90), "max_abs": max(e) if e else None,
                  "missing": sum(1 for r in ok if r.get(k) is None)}
    for k in ("vid_launch_err", "vid_impact_err", "imu_impact_err", "imu_latency"):
        e = [r[k] for r in ok if r.get(k) is not None]
        out[k] = {"p5": _pct(e, 5), "median": _pct(e, 50), "p95": _pct(e, 95), "n": len(e)}
    for k in ("vid", "imu"):
        v = [(r["cls"], r.get(k + "_verdict")) for r in ok]
        out[k + "_terrain"] = {
            "expected": sum(1 for c, x in v if x == EXPECT[c]) / max(1, len(v)),
            "unsafe_go": sum(1 for c, x in v if c in UNSAFE_GO and x == "GO"),
            "go_on_good": sum(1 for c, x in v if c in ("hard_ground", "loose_soil") and x == "GO"),
            "n_good": sum(1 for c, _ in v if c in ("hard_ground", "loose_soil")),
        }
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=24)
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--size", default="360x240")
    ap.add_argument("--lens", choices=("default", "true"), default="default")
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--mosaic", action="store_true")
    ap.add_argument("--json", help="write rows here")
    a = ap.parse_args(argv)
    size = tuple(int(x) for x in a.size.split("x"))
    t = time.time()
    rows = evaluate(range(a.start, a.start + a.n), size, a.lens, a.jobs, a.mosaic)
    for r in rows:
        if "error" in r:
            print("CRASH", r["seed"], r["error"], r.get("trace"))
            continue
        print("%3d %-11s %-9s %-6s p%4.1f v%4.2f R%4.2f | aim %+5.1f%% vid %+5.1f%% imu %+5.1f%% | L %+4.0f I %+4.0f ms | %s %s/%s" % (
            r["seed"], r["cls"], r["link"], r["rest"], r["pitch"], r["v0"], r["range"],
            100 * (r["vid_aim_err"] or float("nan")), 100 * (r["vid_err"] or float("nan")),
            100 * (r["imu_err"] or float("nan")), 1000 * (r.get("vid_launch_err") or float("nan")),
            1000 * (r.get("vid_impact_err") or float("nan")), r.get("imu_cls"), r.get("imu_verdict"),
            r.get("vid_verdict")))
    s = summarise(rows)
    print(json.dumps(s, indent=1))
    print("%.0f s" % (time.time() - t))
    if a.json:
        with open(a.json, "w") as f:
            json.dump({"rows": rows, "summary": s}, f, indent=1, default=float)


if __name__ == "__main__":
    main()
