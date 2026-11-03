"""Range sensitivity: change one uncertain input at a time (low / high) and rerun launch + flight at one pitch.
Tells Harry which measurement matters most. Writes base/out/physics/sensitivity.json.

python sensitivity.py [--pitch 35] [--brake 0.75]
"""
from __future__ import annotations

import argparse
import copy
import json
import os

from common import load_params
from flight import simulate
from launch import launch

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "..", "out", "physics")


def setter(path):
    def f(p, v):
        obj = p
        parts = path.split(".")
        for k in parts[:-1]:
            obj = obj[k] if isinstance(obj, dict) else getattr(obj, k)
        if isinstance(obj, dict):
            obj[parts[-1]] = v
        else:
            setattr(obj, parts[-1], v)
    return f


CASES = [
    ("spring rate N/mm (each)", "spring.rate_n_mm", 0.110, 0.140),
    ("spring initial tension N", "spring.f_init", 5.0, 10.0),
    ("dart mass kg", "dart.mass", 0.265, 0.300),
    ("dart CdA m2", "dart.cda", 2.37e-3, 2.86e-3),
    ("rail friction mu", "launch.mu_rail", 0.15, 0.40),
    ("carriage mass kg", "launch.carriage_mass", 0.030, 0.080),
    ("reel brake N", "flight.brake_n", 0.5, 1.0),
    ("guide eye gain", "flight.eye_gain", 1.0, 1.4),
    ("line mass kg/m", "line.lam", 0.50e-3, 0.75e-3),
    ("muzzle height m", "launch.muzzle_height", 0.20, 0.50),
]


def run(p=None, pitch=35.0, brake=None):
    p = p or load_params()
    if brake is not None:
        p.flight["brake_n"] = brake

    def shot(q):
        v = launch(q, pitch, model="lumped")["v_muzzle"]
        return v, simulate(q, v, pitch, brake_n=q.flight["brake_n"], reel_model=True)["range_m"]

    v0, r0 = shot(p)
    rows = []
    for name, path, lo, hi in CASES:
        out = []
        for val in (lo, hi):
            q = copy.deepcopy(p)
            setter(path)(q, val)
            out.append(shot(q))
        rows.append(dict(name=name, low=lo, high=hi, v_low=out[0][0], v_high=out[1][0], range_low=out[0][1],
                         range_high=out[1][1], swing_m=abs(out[1][1] - out[0][1])))
    rows.sort(key=lambda r: -r["swing_m"])
    return dict(params_source=p.source, pitch_deg=pitch, brake_n=p.flight["brake_n"], v_base=v0, range_base=r0, rows=rows)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--pitch", type=float, default=35.0)
    ap.add_argument("--brake", type=float, default=None)
    a = ap.parse_args(argv)
    r = run(pitch=a.pitch, brake=a.brake)
    print(f"params: {r['params_source']}; base: v {r['v_base']:.2f} m/s, range {r['range_base']:.2f} m at {r['pitch_deg']} deg")
    print(f"{'input':28s} {'low':>9s} {'high':>9s}  range low / high   swing")
    for x in r["rows"]:
        print(f"{x['name']:28s} {x['low']:9.4g} {x['high']:9.4g}  {x['range_low']:5.2f} / {x['range_high']:5.2f}   {x['swing_m']:.2f} m")
    os.makedirs(OUT_DIR, exist_ok=True)
    json.dump(r, open(os.path.join(OUT_DIR, "sensitivity.json"), "w"), indent=1)
    return r


if __name__ == "__main__":
    main()
