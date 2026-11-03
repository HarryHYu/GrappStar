#!/usr/bin/env bash
# Whole CAD chain: assembly export, audit, render, sim, steps, pack, STEP, BOM, guide.
#   bash run_all.sh [demo|base] [OUT]
# demo = only the demo subsystem (default OUT out/demo), base = every subsystem except demo (default OUT out/base).
# The audit must be CLEAN or the chain stops (FORCE=1 carries on). PY overrides the python
# (default ../kedge/.venv/bin/python when present, else python3). JOBS sets the worker processes of the audit and
# the STEP export (default 2: memory). The print pack goes to OUT/pack.
# Every run first removes the downstream outputs of an earlier run from OUT (guide, BOM, pack, step, pages, shots,
# sheets), so a run that stops early never leaves stale files behind. OUT must be under this folder or /tmp.
# Tools that are not there yet (render, sim, steps) are skipped with a note.
# Works with the bash 3.2 of macOS (empty arrays under set -u are written ${A[@]+"${A[@]}"}).
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
JOBS="${JOBS:-2}"
if [ -z "${PY:-}" ]; then
  if [ -x ../kedge/.venv/bin/python ]; then PY=../kedge/.venv/bin/python; else PY=python3; fi
fi

# ---- clean the downstream outputs of an earlier run, only inside a safe OUT
if [ -z "$OUT" ]; then echo "STOP: OUT is empty."; exit 2; fi
mkdir -p "$OUT" || exit 2
HERE="$(pwd -P)"
OUTABS="$(cd "$OUT" && pwd -P)" || { echo "STOP: cannot enter OUT $OUT"; exit 2; }
TMPABS="$(cd /tmp && pwd -P)"
case "$OUTABS" in
  "$HERE"/*|"$TMPABS"/*|/tmp/*) ;;
  *) echo "STOP: OUT $OUT ($OUTABS) is not under $HERE or /tmp; refusing to clean it."; exit 2 ;;
esac
if [ "$OUTABS" = "/" ] || [ "$OUTABS" = "$HERE" ] || [ "$OUTABS" = "$TMPABS" ] || [ "$OUTABS" = "/tmp" ]; then
  echo "STOP: OUT $OUT ($OUTABS) is not a dedicated output folder; refusing to clean it."
  exit 2
fi
case "$OUTABS" in
  "$HERE"/tools|"$HERE"/tools/*|"$HERE"/cad|"$HERE"/cad/*|"$HERE"/physics|"$HERE"/physics/*|"$HERE"/spec|"$HERE"/spec/*)
    echo "STOP: OUT $OUT is inside the sources; refusing to clean it."; exit 2 ;;
esac
if [ -e "$OUTABS/.git" ]; then echo "STOP: OUT $OUT is a git checkout; refusing to clean it."; exit 2; fi
for f in guide.pdf BOM.csv BOM.md pack step sim.html sim_body.html assembly.html assembly_body.html shots \
         audit.txt audit.json audit.log audit_sheet.png sections.png views web; do
  rm -rf "${OUTABS:?}/$f"
done
rm -f "${OUTABS:?}"/sim*.html "${OUTABS:?}"/assembly*.html "${OUTABS:?}"/BOM.*

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

echo "GrappStar CAD chain: mode $MODE, OUT $OUT, python $PY, jobs $JOBS"
step "assembly export" 1 "$PY" -m tools.assembly "$OUT" ${ONLY[@]+"${ONLY[@]}"}

# The audit runs in the background while the outputs that do not need it are built (render, sim, steps, pack,
# STEP, BOM); the guide (which prints the audit result) waits for it. SERIAL=1 runs it first and stops on NOT CLEAN
# before anything else (FORCE=1 carries on). AUDIT_ARGS adds audit options (for example "--dt 0.2 --max-mm 1.0").
audit_wait() {
  wait "$APID"; ARC=$?
  echo "--- audit: exit $ARC, $(( $(date +%s) - AT0 )) s (log $OUT/audit.log)"
  tail -n 3 "$OUT/audit.log" | sed 's/^/    /'
  if [ $ARC -ne 0 ]; then
    if [ "${FORCE:-0}" = 1 ]; then
      FAILED+=("audit NOT CLEAN (forced on)")
    else
      echo
      echo "STOP: the audit is NOT CLEAN (see $OUT/audit.txt). Fix the model, or run with FORCE=1 to build the outputs anyway."
      exit 1
    fi
  fi
}
echo
echo "=== audit (jobs $JOBS ${AUDIT_ARGS:-})"
AT0=$(date +%s)
# shellcheck disable=SC2086
"$PY" -m tools.audit "$OUT" ${ONLY[@]+"${ONLY[@]}"} --jobs "$JOBS" ${AUDIT_ARGS:-} > "$OUT/audit.log" 2>&1 &
APID=$!
if [ "${SERIAL:-0}" = 1 ]; then audit_wait; fi

optional render "$OUT" ${ONLY[@]+"${ONLY[@]}"}
optional sim "$OUT" ${ONLY[@]+"${ONLY[@]}"}
optional steps "$OUT" ${ONLY[@]+"${ONLY[@]}"} --shots
step "pack" 0 "$PY" -m tools.pack "$OUT" "$PACK" ${ONLY[@]+"${ONLY[@]}"}
step "step export" 0 "$PY" -m tools.step_export "$OUT" ${ONLY[@]+"${ONLY[@]}"} --jobs "$JOBS"
step "bom" 0 "$PY" -m tools.bom "$OUT" ${ONLY[@]+"${ONLY[@]}"}
if [ "${SERIAL:-0}" != 1 ]; then echo; echo "=== waiting for the audit"; audit_wait; fi
step "guide" 0 "$PY" -m tools.guide "$OUT" ${ONLY[@]+"${ONLY[@]}"} --pack "$PACK"

echo
echo "=== done in $(( $(date +%s) - T0 )) s: $OUT"
for f in audit.txt audit_sheet.png sections.png sim.html assembly.html BOM.md guide.pdf step/assembly_rest.step pack/PRINT_SHEET.pdf; do
  [ -e "$OUT/$f" ] && echo "  $OUT/$f"
done
if [ ${#FAILED[@]} -gt 0 ]; then
  echo "Problems:"
  for f in ${FAILED[@]+"${FAILED[@]}"}; do echo "  $f"; done
  exit 1
fi
exit 0
