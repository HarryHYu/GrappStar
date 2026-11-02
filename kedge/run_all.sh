#!/bin/bash
# full pipeline: generate -> solve angles -> sweep audit -> clearance -> meshes -> renders -> pack
set -e
OUT=${1:-out27}; V=${2:-v27}
cd "$(dirname "$0")"
KV=$V timeout 1200 python3 kedge_v27.py $OUT > $OUT.gen.log 2>&1 || { tail -20 $OUT.gen.log; exit 1; }
grep -E "WARN|wt=False" $OUT.gen.log || true
python3 solve_deg.py $OUT
timeout 1800 python3 audit27.py $OUT 2 1 > $OUT.audit.log 2>&1 || true
python3 -c "
import json;r=json.load(open('$OUT/audit.json'))
for k,v in r['collisions'].items(): print('COLLISION',k, v[:4])
for k in ['ground_below','auger_depth_mm','foot_housing_clear_mm','pod_clear_mm','belly_outside_r21','folded_bbox','printed_cm3','L']: print(k, r[k])"
tail -1 $OUT.audit.log
timeout 1200 python3 clearance.py $OUT > $OUT.clear.log 2>&1; head -12 $OUT.clear.log
timeout 300 python3 meshes.py $OUT
timeout 900 node shots.js $OUT && python3 sheet.py $OUT
timeout 900 python3 pack.py $OUT pack_$V > $OUT.pack.log 2>&1; tail -13 $OUT.pack.log
# ---- reports and pack extras
timeout 900 python3 physics.py $OUT > $OUT.phys.log 2>&1 || tail -5 $OUT.phys.log
timeout 300 python3 coupon.py pack_$V
timeout 120 python3 wires.py $OUT pack_$V > /dev/null
python3 wiring_dot.py pack_$V
timeout 600 python3 barrel.py $OUT pack_$V > /dev/null
timeout 300 node extra_shots.js $OUT
timeout 900 python3 flight_sim.py $OUT pack_$V > /dev/null 2>&1 || echo flight_sim failed
