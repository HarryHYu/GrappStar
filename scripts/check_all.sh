#!/bin/bash
# Run every test suite in the repo and print one summary line per suite.
# Usage: scripts/check_all.sh            (from the repo root)
# Python env for CAD and physics: kedge/.venv (kedge/setup_mac.sh or: python3 -m venv kedge/.venv &&
#   kedge/.venv/bin/pip install -r kedge/requirements.txt pytest)
# Python env for the Jetson tests on a PC: any venv with numpy<2, opencv-python-headless, pyserial, pytest.
#   Set JPY=/path/to/python if it is not kedge/.venv.
cd "$(dirname "$0")/.."
ROOT=$(pwd)
PY=${PY:-$ROOT/kedge/.venv/bin/python}
JPY=${JPY:-$PY}
fail=0
run() {                      # run NAME DIR COMMAND...
    local name=$1 dir=$2; shift 2
    if [ ! -d "$dir" ]; then echo "SKIP  $name (no $dir)"; return; fi
    local log; log=$(mktemp)
    if (cd "$dir" && timeout 1800 "$@") > "$log" 2>&1; then
        echo "PASS  $name: $(grep -E -i 'passed|all pass|checks|clean' "$log" | tail -1)"
    else
        echo "FAIL  $name (log: $log)"; tail -15 "$log" | sed 's/^/      /'; fail=1
    fi
}
run "dart firmware (host)"   firmware/dart/test   bash run_tests.sh
[ -f firmware/base/test/run_tests.sh ] && run "base firmware (host)" firmware/base/test bash run_tests.sh
run "jetson software"        jetson               "$JPY" -m pytest -q
run "cad library and tools"  base                 "$PY" -m pytest -q tools/tests
ls base/physics/test_*.py base/physics/tests/*.py >/dev/null 2>&1 && run "physics models" base/physics "$PY" -m pytest -q
[ -f base/run_all.sh ] && [ "$1" = "--full" ] && run "base pipeline (assembly, audit, pack, sim, guide)" base bash run_all.sh
exit $fail
