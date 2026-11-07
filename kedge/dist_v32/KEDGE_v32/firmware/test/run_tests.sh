#!/bin/sh
# Builds and runs the PC tests of the dart logic, then checks every JSON file the tests wrote with Python.
set -e
cd "$(dirname "$0")"
# kedge/fw keeps a copy of the firmware for the old kedge pipeline (kedge/fw/README.md): it must match this one
if [ -d ../../../kedge/fw/kedge_dart ]; then
  for f in kedge_dart/kedge_dart.ino kedge_dart/kedge_logic.h test/test_logic.cpp; do
    if ! cmp -s ../$f ../../../kedge/fw/$f; then echo "kedge/fw/$f differs from firmware/dart/$f: run firmware/dart/tools/sync_kedge_fw.sh"; exit 1; fi
  done
  echo "kedge/fw copy matches firmware/dart"
fi
g++ -std=c++17 -O1 -Wall -Wextra -I../kedge_dart -o test_logic test_logic.cpp
./test_logic
g++ -std=c++17 -O2 -Wall -Wextra -I../kedge_dart -o fuzz_logic fuzz_logic.cpp
# randomized property test, 1000 episodes on each of two seeds (full run: ./fuzz_logic [seed]). The output goes to a
# file first so a failure is not hidden by the pipe to tail.
mkdir -p out
for seed in 20261103 1; do
  if ./fuzz_logic --quick $seed > out/fuzz_$seed.log 2>&1; then tail -18 out/fuzz_$seed.log; else tail -40 out/fuzz_$seed.log; echo "FUZZ FAILED (seed $seed, out/fuzz_$seed.log)"; exit 1; fi
done
# episodes that once failed in full runs, replayed alone (v4.1: seed 8 episode 4747, a refused press of the other
# direction restarted the 2 s hold cap; seeds 3, 5 and 1: test-side classification of documented limits)
for ep in "8 4748 4747" "3 741 740" "5 6168 6167" "1 1774 1773" "3 5452 5451" "8 2822 2821" "2 572 571" "1 5683 5682" "1 3730 3729"; do
  if ! ./fuzz_logic $ep > out/fuzz_replay.log 2>&1; then tail -30 out/fuzz_replay.log; echo "FUZZ REPLAY FAILED ($ep)"; exit 1; fi
done
echo "fuzz replays of past failures: PASS"
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

# Host simulator of the real sketch, the Jetson client and the web page (about 2.5 min). Skip with: ./run_tests.sh --quick
if [ "$1" != "--quick" ] && command -v make >/dev/null; then
  if ( cd ../sim && make -s && python3 test_sim.py && python3 test_jetson_client.py && { ! command -v node >/dev/null || node test_page.js; } ) > out/sim.log 2>&1; then
    grep -E "^Ran|^OK|passed, " out/sim.log
    echo "dart firmware: host logic checks ALL PASS, simulator HTTP tests OK, Jetson client tests OK, web page browser checks OK"
  else
    tail -30 out/sim.log; echo "SIMULATOR TESTS FAILED"; exit 1
  fi
fi
