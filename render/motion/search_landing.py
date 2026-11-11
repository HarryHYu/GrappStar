"""Search physically plausible launch inputs for a landing whose own bounce / tumble leaves the dart inverted or on a
flank, so the v34 firmware rights it with the righting leg, plants both drills and holds (loam, v34 tail tie).

    ../../kedge/.venv/bin/python search_landing.py [--n 160] [--seed 7] --node-path <node_modules dir>

Inputs (uniform): rail 33..35 deg, muzzle 6.7..6.9 m/s, tip-off pitch rate -1..1 rad/s and initial pitch offset
-5..5 deg (flight34's disturbance range: 0.4 rad/s cradle tip-off, 5 deg + 1 rad/s), yaw rate -1..1 rad/s and yaw offset
-5..5 deg, steady cross-wind -1.5..1.5 m/s (along world Y), flat loam (slope 0). Each run: launch on the base geometry,
Flight3D to 20 mm above the ground, then the righting sim from that state (sim_dump.mjs inits). Writes
search/search_results.json (inputs, flight end, sim outcome per run) and prints the runs that qualify."""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import base_motion as BM  # noqa: E402
import dart_motion as DM  # noqa: E402
import landing as LD  # noqa: E402

SIM_JOB = {"cfg": {"axle": "rigid", "tie": True, "legLen": 3},
           "D": {"hull": {"mass": 0.1905, "cgX": 83.3},
                 "pod": {"mass": 0.026, "pivotX": 132, "pivotY": 9.5, "stopDeg": 46.75, "lat": 31.15},
                 "leg": {"mass": 0.0055, "len": 113, "padW": 37, "pad": 3.5, "hingeX": 33, "hingeY": 14.5, "stopDeg": 178}},
           "LAYOUTS": {"tail": {"pivotX": 132}}, "cgUp": -2.6, "soil": "loam"}      # = v34_tail_tie_worm.json T_rigid_tie
CGX = 0.0833
LEG_MOVES = ("legFlip", "legRock", "podsThenFlip")


def reel_point(B):
    """the line leaves the base over the lip roller top once the rail is down (rail_low, D45): sim tow direction"""
    S = B.state(B.t0["anchor"])
    v = B.values(S)
    ur, wr, rp = B.roller(S, v["launch_carriage"])
    Wr = B.rail_W(v["launch_rail"])
    return BM.pts_world_m((Wr @ np.array([ur, 0.0, wr + rp, 1.0]))[:3])


def draw(rng):
    return dict(rail_deg=round(float(rng.uniform(33.0, 35.0)), 3), v0=round(float(rng.uniform(6.7, 6.9)), 3),
                q0=round(float(rng.uniform(-1.0, 1.0)), 3), dth=round(float(rng.uniform(-5.0, 5.0)), 2),
                r0=round(float(rng.uniform(-1.0, 1.0)), 3), dpsi=round(float(rng.uniform(-5.0, 5.0)), 2),
                wind_y=round(float(rng.uniform(-1.5, 1.5)), 3), slope_deg=0.0)


def run_sims(inits, node_path, tag, nproc=4):
    env = dict(os.environ)
    if node_path:
        env["NODE_PATH"] = node_path
    os.makedirs(os.path.join(HERE, "search"), exist_ok=True)
    procs, outs = [], []
    for k in range(nproc):
        chunk = inits[k::nproc]
        if not chunk:
            continue
        out = os.path.join(HERE, "search", f"_{tag}_{k}.json")
        job = dict(SIM_JOB, out=out, inits=chunk)
        procs.append(subprocess.Popen(["node", os.path.join(HERE, "sim_dump.mjs"), json.dumps(job)], env=env, cwd=HERE,
                                      stderr=subprocess.DEVNULL))
        outs.append(out)
    for p in procs:
        p.wait()
    res = []
    for o in outs:
        res += json.load(open(o))
        os.remove(o)
    return res


def qualifies(r, case_sum):
    mv = [m["move"] for m in r["moves"]]
    first_tilt = r["moves"][0]["tilt"] if r["moves"] else 0.0
    return (r["righted"] and r["anchored"] == 2 and r["held"] >= 55 and not r["blowup"] and any(m in LEG_MOVES for m in mv)
            and first_tilt > 60.0 and 4.4 < r["final_p"][0] < 5.6 and "align" not in [p["p"] for p in r["phases"]])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=160)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--node-path", default=os.environ.get("NODE_PATH", ""))
    ap.add_argument("--recheck", default="", help="results json: re-run its runs that used the leg, with snapped pitch")
    a = ap.parse_args()
    t0 = time.time()
    B = BM.BaseKin()
    S_cocked = B.state(B.t0["flight"] - 1e-6)
    geom = LD.LaunchGeom(B, S_cocked, CGX)
    cloud = DM.dart_cloud()
    reel = reel_point(B)
    rng = np.random.default_rng(a.seed)
    cases, inits = [], []
    pool = None
    if a.recheck:
        R0 = json.load(open(a.recheck))
        pool = [r["inputs"] for r in R0["runs"] if any(m in LEG_MOVES for m in r["moves"])]
        a.n = len(pool)
    for i in range(a.n):
        inp = dict(pool[i]) if pool else draw(rng)
        inp["rail_deg"] = round(B.snap_pitch(inp["rail_deg"])[0], 6)   # a pitch the aim train really reaches
        c = LD.Case(geom, inp, cloud)
        cases.append((inp, c.summary()))
        inits.append(dict(id=i, init=c.sim_init(CGX, reel)))
    print(f"{a.n} flights in {time.time() - t0:.0f} s", flush=True)
    res = run_sims(inits, a.node_path, f"s{a.seed}{'r' if a.recheck else ''}")
    res.sort(key=lambda r: r["id"])
    rows = []
    good = []
    for r in res:
        inp, cs = cases[r["id"]]
        mv = [m["move"] for m in r["moves"]]
        row = dict(id=r["id"], inputs=inp, flight=cs, righted=r["righted"], rightT=r["rightT"], moves=mv,
                   first_tilt=r["moves"][0]["tilt"] if r["moves"] else None, anchored=r["anchored"], held=r["held"],
                   aligned="align" in [p["p"] for p in r["phases"]], final_p=r["final_p"], t=r["t"], qualifies=qualifies(r, cs))
        rows.append(row)
        if row["qualifies"]:
            good.append(row)
    out = dict(what=__doc__.split("\n\n")[0], seed=a.seed, n=a.n, reel_world=np.round(reel, 4).tolist(), runs=rows,
               qualifying=[g["id"] for g in good])
    tag = f"recheck_{os.path.basename(a.recheck)[:-5]}" if a.recheck else f"s{a.seed}"
    path = os.path.join(HERE, "search", f"search_results_{tag}.json")
    json.dump(out, open(path, "w"), indent=1)
    cnt = {}
    for row in rows:
        k = "+".join(row["moves"]) or "none"
        cnt[k] = cnt.get(k, 0) + 1
    print("moves:", dict(sorted(cnt.items(), key=lambda kv: -kv[1])))
    for g in good:
        print(g["id"], g["inputs"], "| tilt", round(g["first_tilt"], 0), g["moves"], "rightT", round(g["rightT"], 2), "align", g["aligned"],
              "final", g["final_p"])
    print(f"{len(good)} qualify; {time.time() - t0:.0f} s; {path}")


if __name__ == "__main__":
    main()
