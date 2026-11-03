#!/usr/bin/env python3
"""Print the C1 command order the Jetson sent in each recorded run and whether it matches BASE_SPEC 10.1 (D35).
  python3 systest/c1_order.py [systest/runs]"""
import glob
import json
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import harness  # noqa: E402

root = sys.argv[1] if len(sys.argv) > 1 else harness.RUNS
for d in sorted(glob.glob(os.path.join(root, "*", "jetson", "*", ""))):
    name = d.split(os.sep)[-4]
    sc = harness.SCENARIOS.get(name, harness.SCENARIOS["nominal"])   # c_c1_sequence reads the scenario's options
    summ = {}
    try:                                     # the shot pitch (D45 low aim) and where the dart seated, from the run
        with open(os.path.join(d, "..", "..", "summary.json")) as f:
            summ = json.load(f)
    except (OSError, ValueError):
        pass
    seat = next((c for c in summ.get("checks", []) if c.get("check", "").startswith("seated on the take-up")), {})
    r = types.SimpleNamespace(app=types.SimpleNamespace(run_dir=d), sc=sc, solution=summ.get("solution") or {},
                              seat_rec={"inReel": False, "step": "UNWRAP"} if "'step': 'UNWRAP'" in seat.get("detail", "") else {})
    ok, seq = harness.c_c1_sequence(r, {})
    print(f"{'OK  ' if ok else 'DIFF'} {name:24s} {seq}")
