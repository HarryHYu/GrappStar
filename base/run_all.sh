#!/usr/bin/env bash
# Whole CAD chain: assembly export, audit, render, sim, steps, pack, STEP, BOM, guide.
#   bash run_all.sh [demo|base] [OUT]
# demo = only the demo subsystem (default OUT out/demo), base = every subsystem except demo (default OUT out/base).
# The audit must be CLEAN or the chain stops (FORCE=1 carries on). PY overrides the python
# (default ../kedge/.venv/bin/python when present, else python3). The print pack goes to OUT/pack.
# Tools that are not there yet (render, sim, steps) are skipped with a note.
set -u
cd "$(dirname "$0")"

MODE="${1:-demo}"
case "$MODE" in
  demo) ONLY=(--only demo) ;;
  base) ONLY=() ;;
  *) echo "usage: bash run_all.sh [demo|base] [OUT]"; exit 2 ;;
esac
OUT="${2:-out/$MODE}"
PACK="$OUT/pack"
if [ -z "${PY:-}" ]; then
  if [ -x ../kedge/.venv/bin/python ]; then PY=../kedge/.venv/bin/python; else PY=python3; fi
fi
mkdir -p "$OUT"
FAILED=()
T0=$(date +%s)

step() {   # step <label> <required 0|1> <cmd...>
  local label="$1" req="$2"; shift 2
  local t=$(date +%s)
  echo
  echo "=== $label"
  "$@"
  local rc=$?
  echo "--- $label: exit $rc, $(( $(date +%s) - t )) s"
  if [ $rc -ne 0 ]; then
    FAILED+=("$label (exit $rc)")
    if [ "$req" = 1 ]; then
      echo "STOP: $label failed."
      exit $rc
    fi
  fi
  return 0
}

optional() {   # optional <tool> <args...>: run tools/<tool>.py when it exists
  local tool="$1"; shift
  if [ -f "tools/$tool.py" ]; then
    step "$tool" 0 "$PY" -m "tools.$tool" "$@"
  else
    echo
    echo "=== $tool: tools/$tool.py not there yet, skipped"
  fi
}

echo "GrappStar CAD chain: mode $MODE, OUT $OUT, python $PY"
step "assembly export" 1 "$PY" -m tools.assembly "$OUT" "${ONLY[@]}"

echo
echo "=== audit"
"$PY" -m tools.audit "$OUT" "${ONLY[@]}"
ARC=$?
if [ $ARC -ne 0 ]; then
  if [ "${FORCE:-0}" = 1 ]; then
    echo "--- audit: NOT CLEAN (exit $ARC). FORCE=1, carrying on."
    FAILED+=("audit NOT CLEAN (forced on)")
  else
    echo
    echo "STOP: the audit is NOT CLEAN (see $OUT/audit.txt). Fix the model, or run with FORCE=1 to build the outputs anyway."
    exit 1
  fi
else
  echo "--- audit: CLEAN"
fi

optional render "$OUT" "${ONLY[@]}"
optional sim "$OUT" "${ONLY[@]}"
optional steps "$OUT" "${ONLY[@]}" --shots
step "pack" 0 "$PY" -m tools.pack "$OUT" "$PACK" "${ONLY[@]}"
step "step export" 0 "$PY" -m tools.step_export "$OUT" "${ONLY[@]}"
step "bom" 0 "$PY" -m tools.bom "$OUT" "${ONLY[@]}"
step "guide" 0 "$PY" -m tools.guide "$OUT" "${ONLY[@]}" --pack "$PACK"

echo
echo "=== done in $(( $(date +%s) - T0 )) s: $OUT"
for f in audit.txt audit_sheet.png sections.png sim.html assembly.html BOM.md guide.pdf step/assembly_rest.step pack/PRINT_SHEET.pdf; do
  [ -e "$OUT/$f" ] && echo "  $OUT/$f"
done
if [ ${#FAILED[@]} -gt 0 ]; then
  echo "Problems:"
  for f in "${FAILED[@]}"; do echo "  $f"; done
  exit 1
fi
exit 0
