#!/bin/bash
# full pipeline: generate -> solve angles (regenerate once so the stops sit at the solved angles) -> sweep audit -> launcher fit
# -> clearance -> meshes -> renders -> pack -> extras.   ./run_all.sh out31 v31 [stepK stepD]
set -e
OUT=${1:-out31}; V=${2:-v31}; SK=${3:-1}; SD=${4:-1}
cd "$(dirname "$0")"
PY=${PY:-$( [ -x .venv/bin/python ] && echo .venv/bin/python || echo python3 )}
KV=$V timeout 1200 $PY kedge_v27.py $OUT > $OUT.gen.log 2>&1 || { tail -20 $OUT.gen.log; exit 1; }
grep -E "WARN|wt=False" $OUT.gen.log || true
if ! $PY solve_deg.py $OUT; then
  D=$($PY -c "import json;P=json.load(open('$OUT/parts.json'))['P'];import os;k=json.loads(os.environ.get('KOV','{}'));k.update(DEPLOY_DEG=P['DEPLOY_DEG'],KICK_DEG=P['KICK_DEG']);print(json.dumps(k))")
  echo "regenerating with $D"; KOV="$D" KV=$V timeout 1200 $PY kedge_v27.py $OUT > $OUT.gen.log 2>&1 || { tail -20 $OUT.gen.log; exit 1; }
  $PY solve_deg.py $OUT
fi
timeout 3600 $PY audit27.py $OUT $SK $SD > $OUT.audit.log 2>&1 || true
$PY -c "
import json;r=json.load(open('$OUT/audit.json'))
for k,v in r['collisions'].items(): print('COLLISION',k, v[:4])
for k in ['ground_below','auger_depth_mm','foot_housing_clear_mm','pod_clear_mm','belly_outside_r21','hard_stops_mm3','folded_bbox','printed_cm3','L']: print(k, r[k])"
tail -1 $OUT.audit.log
timeout 2400 $PY launcher_fit.py $OUT out30 > $OUT.fit.log 2>&1 || true; tail -1 $OUT.fit.log
timeout 1200 $PY clearance.py $OUT > $OUT.clear.log 2>&1; head -12 $OUT.clear.log
timeout 300 $PY meshes.py $OUT
timeout 900 node shots.js $OUT && $PY sheet.py $OUT
timeout 900 $PY pack.py $OUT pack_$V > $OUT.pack.log 2>&1; tail -16 $OUT.pack.log
$PY massprops.py $OUT > $OUT.mass.log 2>&1 || tail -5 $OUT.mass.log
$PY stepcheck.py pack_$V > $OUT.step.log 2>&1; tail -1 $OUT.step.log
# ---- reports and pack extras
timeout 900 $PY physics.py $OUT > $OUT.phys.log 2>&1 || tail -5 $OUT.phys.log
timeout 300 $PY coupon.py pack_$V
timeout 120 $PY wires.py $OUT pack_$V > /dev/null
timeout 1200 $PY wireroom.py $OUT pack_$V > $OUT.wireroom.log 2>&1 || true; tail -3 $OUT.wireroom.log
$PY wiring_dot.py pack_$V || true
timeout 600 $PY barrel.py $OUT pack_$V > /dev/null
timeout 300 node extra_shots.js $OUT || true
timeout 900 $PY flight_sim.py $OUT pack_$V > /dev/null 2>&1 || echo flight_sim failed
timeout 1200 $PY assembly_step.py $OUT pack_$V || true
$PY plate_view.py pack_$V || true
$PY runsheet.py $OUT pack_$V && $PY report.py $OUT pack_$V variants_final.json || true
