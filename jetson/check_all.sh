#!/usr/bin/env bash
# Everything, including the slow tests against the real base and dart firmware simulators.
set -euo pipefail
cd "$(dirname "$0")"
PY="${PYTHON:-python3}"
"$PY" -m pytest -q -m "not slow" "$@"
"$PY" -m pytest -q -m slow "$@"
