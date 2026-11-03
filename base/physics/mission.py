"""One mission, time and battery budget: aim, cock + fire, flight, wrap, tow the rover to the dart, unwrap, reel the
dart home, seat. Wiper numbers from drivetrain.py at 11.1 V with sag; drum N20 and dart numbers ASSUMED (marked).

python mission.py [--target 4.0] [--tow-n 30] [--tow-m 3.0]
"""
from __future__ import annotations

import argparse
import json
import os

import drivetrain
from common import load_params

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "..", "out", "physics")

ASSUMED = dict(
    drum_shift_s=3.0,       # N20 drum move between adjacent positions (cad-drum to confirm)
    drum_a=0.15,            # N20 current at 9 V, A (buck from the 3S)
    wrap_s=18.0,            # retrieve.py: 240 deg of drum at ~20 rpm N20 through 9:1
    anchor_s=60.0,          # dart: kick + drills + foot (dart battery, not the base battery)
    release_s=20.0,         # dart release sequence
    idle_w=4.0,             # ESP32-S3 + AS5600 + BTS logic, base side, W (Jetson not counted)
    pack_wh=5.2 * 11.1,     # 3S 5200 mAh (spec BATTERY)
)


def run(p=None, tow_n=30.0, tow_m=3.0, reel_n=3.0):
    p = p or load_params()
    d = drivetrain.run(p, light_n=reel_n, reel_len=5.0, tow_len=tow_m)
    f = d["functions"]
    a = ASSUMED
    steps = []

    def add(name, t, wh, note=""):
        steps.append(dict(step=name, time_s=t, wh=wh, note=note))

    drum_wh = a["drum_a"] * 9.0 * a["drum_shift_s"] / 3600
    add("aim (worst case full 6 to 47 deg)", f["aim"]["time_s"], f["aim"]["energy_wh"])
    add("drum to LAUNCH", a["drum_shift_s"], drum_wh, "N20 ASSUMED")
    add("cock + fire + reset", f["cock"]["time_s"], f["cock"]["energy_wh"])
    add("flight", 1.0, 0.0)
    add("dart anchors", a["anchor_s"], 0.0, "dart battery")
    add("drum LAUNCH -> NEUTRAL -> WRAP (7 wraps)", 2 * a["drum_shift_s"] + a["wrap_s"], drum_wh * (2 + a["wrap_s"] / a["drum_shift_s"]))
    add("drum to REEL", a["drum_shift_s"], drum_wh)
    tow = drivetrain.cap_fn(p, tow_n, tow_m)
    ev = drivetrain.evaluate(p, "tow", tow, p.battery.v_nom)
    add(f"tow the rover {tow_m} m at {tow_n} N", ev["time_s"], ev["energy_wh"])
    add("dart release", a["release_s"], 0.0, "dart battery")
    add("unwrap + drum back to REEL (reel-in on the capstan or take-up)", a["wrap_s"] + a["drum_shift_s"], drum_wh * (1 + a["wrap_s"] / a["drum_shift_s"]))
    rl = drivetrain.cap_fn(p, reel_n, 5.0)
    er = drivetrain.evaluate(p, "reel", rl, p.battery.v_nom)
    add("reel the dart home 5 m", er["time_s"], er["energy_wh"])
    add("drum to LAUNCH (ready)", a["drum_shift_s"], drum_wh)
    t = sum(s["time_s"] for s in steps)
    wh = sum(s["wh"] for s in steps) + a["idle_w"] * t / 3600
    return dict(params_source=p.source, steps=steps, total_s=t, total_wh=wh, idle_wh=a["idle_w"] * t / 3600,
                missions_per_pack=a["pack_wh"] * 0.8 / wh, assumed=a)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--tow-n", type=float, default=30.0)
    ap.add_argument("--tow-m", type=float, default=3.0)
    a = ap.parse_args(argv)
    r = run(tow_n=a.tow_n, tow_m=a.tow_m)
    print(f"params: {r['params_source']}")
    for s in r["steps"]:
        print(f"  {s['step']:58s} {s['time_s']:6.1f} s  {s['wh'] * 1000:6.1f} mWh  {s['note']}")
    print(f"total {r['total_s']:.0f} s ({r['total_s'] / 60:.1f} min), {r['total_wh']:.2f} Wh incl. {r['idle_wh']:.2f} Wh idle; "
          f"about {r['missions_per_pack']:.0f} missions per 3S 5200 mAh pack (80 % usable, base side only)")
    os.makedirs(OUT_DIR, exist_ok=True)
    json.dump(r, open(os.path.join(OUT_DIR, "mission.json"), "w"), indent=1)
    return r


if __name__ == "__main__":
    main()
