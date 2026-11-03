#!/usr/bin/env python3
"""Print the C1 command order the Jetson sent in each recorded run and whether it matches BASE_SPEC 10.1 (D35).
  python3 systest/c1_order.py [systest/runs]"""
import glob
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import harness  # noqa: E402

root = sys.argv[1] if len(sys.argv) > 1 else harness.RUNS
for d in sorted(glob.glob(os.path.join(root, "*", "jetson", "*", ""))):
    r = types.SimpleNamespace(app=types.SimpleNamespace(run_dir=d))
    ok, seq = harness.c_c1_sequence(r, {})
    print(f"{'OK  ' if ok else 'DIFF'} {d.split(os.sep)[-4]:24s} {seq}")
