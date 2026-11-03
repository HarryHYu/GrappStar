#!/bin/sh
# Run every physics model on the current params and regenerate all outputs (table, SUMMARY, PDF), then the tests.
# About 3 to 5 minutes. A failing step does not stop the run; the last line says PASS or FAIL.
cd "$(dirname "$0")" || exit 1
PY=${PY:-../../kedge/.venv/bin/python}
FAILED=""
for step in "launcher.py --sweep" "flight.py --sweep" payout.py capstan.py drivetrain.py sensitivity.py attitude.py \
            recoil.py retrieve.py mission.py ballistics.py summary.py plots.py; do
    echo "== $step"
    # shellcheck disable=SC2086
    $PY $step || FAILED="$FAILED $step;"
done
echo "== tests"
$PY -m pytest -q -p no:cacheprovider || FAILED="$FAILED tests;"
echo
if [ -z "$FAILED" ]; then
    echo "PASS: all models ran and all tests passed"
else
    echo "FAIL:$FAILED (see the output above)"
    exit 1
fi
