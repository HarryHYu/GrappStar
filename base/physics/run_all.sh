#!/bin/sh
# Run every physics model on the current params and regenerate all outputs. About 3 to 5 minutes.
set -e
cd "$(dirname "$0")"
PY=${PY:-../../kedge/.venv/bin/python}
$PY -m pytest -q || echo "TESTS FAILED (models still run)"
$PY launch.py --sweep
$PY flight.py --sweep
$PY payout.py
$PY capstan.py
$PY drivetrain.py
$PY sensitivity.py
$PY attitude.py
$PY recoil.py
$PY retrieve.py
$PY ballistics.py
$PY summary.py
