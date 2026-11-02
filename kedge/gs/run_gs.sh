#!/bin/bash
# ground station pipeline: geometry -> kinematic tables -> physics -> mission timeline -> collision audit -> print pack -> HD sim
set -e
cd "$(dirname "$0")"
PY=../.venv/bin/python
OUT=${1:-out}
mkdir -p $OUT
$PY dart_mesh.py ../out30 $OUT > /dev/null      # first: the generator shapes the retrieve funnel around the folded dart
$PY gs_gen.py $OUT
$PY mkdata.py $OUT
$PY physics_gs.py $OUT > $OUT/physics_gs.log
$PY timeline.py $OUT
$PY audit_gs.py $OUT 1 | tail -3
$PY pack_gs.py $OUT ../pack_gs | head -1
$PY wiring_gs.py ../pack_gs
$PY mksim.py $OUT ../pack_gs/GrappStar_mission_sim.html
$PY mkasm.py $OUT ../pack_gs/GrappStar_assembly_steps.html
