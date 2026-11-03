"""Write base/out/ballistics.json exactly per contract C3 (the Jetson solver interpolates it).

{"version": 1, "pitch_min": ..., "pitch_max": ..., "drag_n": [0.5, 0.75, 1.0],
 "table": [{"pitch_deg": ..., "v_muzzle": ..., "range_m": [r05, r075, r10], "t_flight_s": [...], "apex_m": [...]}]}

Meaning of the numbers (also written to base/out/ballistics_meta.json, which the Jetson may ignore):
- drag_n: slip-clutch setting, expressed as the line pull at the reel exit with the reel FULL (20 m on). The coupled
  reel model (inertia, clutch torque, guide eye, line weight and drag) is used, so the real pull on the dart differs.
- range_m: level ground, from the ground point under the rail pivot (main axis) to the dart's first ground contact
  (the pivot is fixed on the rover; the muzzle moves with pitch). Pivot height = deck height (ASSUMED 0.15 m) + 0.165 m.
- v_muzzle: from launcher.py at that pitch (gravity and friction along the rail change it slightly).
- t_flight_s: muzzle to ground contact. apex_m: highest point of the dart above the ground.

python ballistics.py [--step 2.5] [--out ../out/ballistics.json]
"""
from __future__ import annotations

import argparse
import json
import os

from common import load_params
from flight import simulate
from launcher import launch

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "out", "ballistics.json")
DRAG_N = [0.5, 0.75, 1.0]


def build(p=None, step: float = 2.5, drag_n=DRAG_N, launch_over: dict | None = None):
    p = p or load_params()
    lo, hi = float(p.flight["pitch_min"]), float(p.flight["pitch_max"])
    pitches = []
    x = lo
    while x <= hi + 1e-9:
        pitches.append(round(x, 3))
        x += step
    if pitches[-1] < hi:
        pitches.append(hi)
    table = []
    for th in pitches:
        v = launch(p, th, **(launch_over or {}))["v_muzzle"]
        rows = [simulate(p, v, th, brake_n=b, reel_model=True) for b in drag_n]
        if not all(r["landed"] for r in rows):
            raise RuntimeError(f"a shot at pitch {th} did not land within the time limit")
        table.append(dict(pitch_deg=th, v_muzzle=round(v, 3),
                          range_m=[round(r["range_m"], 3) for r in rows],
                          t_flight_s=[round(r["t_flight_s"], 3) for r in rows],
                          apex_m=[round(r["apex_m"], 3) for r in rows]))
    doc = dict(version=1, pitch_min=lo, pitch_max=hi, drag_n=list(drag_n), table=table)
    meta = dict(params_source=p.source, dart_mass_kg=p.dart.mass, dart_cda_m2=p.dart.cda,
                pivot_height_m=p.launch.get("pivot_h"), release_u_m=p.launch.get("release_u"), dart_w_m=p.launch.get("dart_w"),
                range_origin="ground point under the rail pivot (main axis)" if p.launch.get("pivot_h") else "muzzle ground point",
                stroke_m=launch_over.get("stroke", p.launch["stroke"]) if launch_over else p.launch["stroke"],
                notes=__doc__.split("Meaning of the numbers")[1].split("python ballistics.py")[0].strip())
    return doc, meta


def validate(doc: dict) -> list:
    """Contract C3 checks. Returns a list of problems (empty = OK)."""
    errs = []
    keys = {"version", "pitch_min", "pitch_max", "drag_n", "table"}
    if set(doc) != keys:
        errs.append(f"top-level keys {sorted(doc)} != {sorted(keys)}")
    if list(doc.get("drag_n", [])) != [0.5, 0.75, 1.0]:
        errs.append("drag_n must be [0.5, 0.75, 1.0] (contract C3)")
    if doc.get("version") != 1:
        errs.append("version must be 1")
    n = len(doc.get("drag_n", []))
    last = -1e9
    import math as _m
    if not doc.get("table"):
        errs.append("empty table")
    for row in doc.get("table", []):
        if set(row) != {"pitch_deg", "v_muzzle", "range_m", "t_flight_s", "apex_m"}:
            errs.append(f"row keys {sorted(row)}")
            continue
        vals = [row["pitch_deg"], row["v_muzzle"]] + list(row["range_m"]) + list(row["t_flight_s"]) + list(row["apex_m"])
        if any(not isinstance(v, (int, float)) or not _m.isfinite(v) for v in vals):
            errs.append(f"non-finite value at pitch {row['pitch_deg']}")
            continue
        if any(v <= 0 for v in row["range_m"] + row["t_flight_s"]):
            errs.append(f"non-positive range or time at pitch {row['pitch_deg']}")
        for k in ("range_m", "t_flight_s", "apex_m"):
            if len(row.get(k, [])) != n:
                errs.append(f"{k} length != drag_n length at pitch {row.get('pitch_deg')}")
        if row["pitch_deg"] <= last:
            errs.append("pitches must increase")
        last = row["pitch_deg"]
        if not (doc["pitch_min"] - 1e-9 <= row["pitch_deg"] <= doc["pitch_max"] + 1e-9):
            errs.append("pitch outside [pitch_min, pitch_max]")
    if doc.get("table") and all("pitch_deg" in r for r in doc["table"]):
        if abs(doc["table"][0]["pitch_deg"] - doc["pitch_min"]) > 1e-9 or abs(doc["table"][-1]["pitch_deg"] - doc["pitch_max"]) > 1e-9:
            errs.append("table must span pitch_min to pitch_max")
    return errs


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--step", type=float, default=2.5)
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args(argv)
    p = load_params()
    doc, meta = build(p, a.step)
    errs = validate(doc)
    if errs:
        raise SystemExit("contract C3 violated: " + "; ".join(errs))
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    json.dump(doc, open(a.out, "w"), indent=1)
    json.dump(meta, open(os.path.join(os.path.dirname(os.path.abspath(a.out)), "ballistics_meta.json"), "w"), indent=1)
    print(f"params: {p.source}")
    print("pitch  v0     range 0.5/0.75/1.0 N     t_flight      apex")
    for r in doc["table"]:
        print(f"{r['pitch_deg']:5.1f} {r['v_muzzle']:5.2f}  " + " ".join(f"{x:5.2f}" for x in r["range_m"]) + "  " +
              " ".join(f"{x:4.2f}" for x in r["t_flight_s"]) + "  " + " ".join(f"{x:4.2f}" for x in r["apex_m"]))
    print("wrote", os.path.abspath(a.out))
    return doc


if __name__ == "__main__":
    main()
