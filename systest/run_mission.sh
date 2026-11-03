#!/bin/sh
# GrappStar end-to-end system test: builds the base shim (real base firmware logic on a pty) and the dart firmware
# simulator, then runs whole missions with the real Jetson code against both, nominal and with faults.
#
#   systest/run_mission.sh                 nominal mission only
#   systest/run_mission.sh all             every scenario (about 1 to 2 min each)
#   systest/run_mission.sh drum_timeout,power_wrap
#   systest/run_mission.sh --list
#   SYSTEST_SPEED=1 systest/run_mission.sh  base firmware in real time (default 2x; the dart sim is always real time)
#
# Needs: g++ (C++17), make, python3 with numpy, opencv (cv2) and pyserial. Results: systest/runs/<scenario>/.
set -e
cd "$(dirname "$0")/.."
python3 -c "import numpy, cv2, serial" 2>/dev/null || { echo "need python3 numpy, opencv (cv2), pyserial: pip install numpy opencv-python-headless pyserial"; exit 2; }
if [ "$1" = "--list" ]; then exec python3 systest/harness.py --list; fi
exec python3 systest/harness.py --scenario "${1:-nominal}"
