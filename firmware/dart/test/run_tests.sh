#!/bin/sh
# Builds and runs the PC tests of the dart logic, then checks every JSON file the tests wrote with Python.
set -e
cd "$(dirname "$0")"
g++ -std=c++17 -O1 -Wall -Wextra -I../kedge_dart -o test_logic test_logic.cpp
./test_logic
python3 - <<'PY'
import json, glob, sys
bad = 0
for p in sorted(glob.glob("out/*.json")):
    try:
        j = json.load(open(p))
        if "samples" in j:
            assert j["hz"] == 50 and len(j["samples"]) == j["n"], "n mismatch"
            assert all(len(s) == 8 for s in j["samples"]), "sample width"
            ts = [s[0] for s in j["samples"]]
            assert ts == sorted(ts), "time order"
        else:
            for k in ("phase", "fault", "seq", "armed", "landed", "current_mA", "rail_V", "roll", "pitch", "event", "t_launch_ms", "t_land_ms"):
                assert k in j, "missing " + k
        print("json ok:", p)
    except Exception as e:
        print("JSON FAIL:", p, e); bad += 1
sys.exit(1 if bad else 0)
PY

# Host simulator of the real sketch (about 40 s). Skip with: ./run_tests.sh --quick
if [ "$1" != "--quick" ] && command -v make >/dev/null; then
  if ( cd ../sim && make -s && python3 test_sim.py ) > out/sim.log 2>&1; then
    tail -4 out/sim.log
    echo "dart firmware: host logic checks ALL PASS, simulator 11 tests OK"
  else
    tail -30 out/sim.log; echo "SIMULATOR TESTS FAILED"; exit 1
  fi
fi
