#!/bin/bash
# Build the delivery zips for one dart version: ./make_dist.sh v31  (after run_all.sh outN vN)
set -e
cd "$(dirname "$0")"
V=$1; OUT=out${V#v}; P=pack_$V; D=dist_$V/KEDGE_$V
FW=../firmware/dart
rm -rf dist_$V; mkdir -p $D/{print,step,firmware,launcher,docs,source}
cp $P/*.stl $P/*.3mf $D/print/; mv $D/print/barrel_keepout.stl $D/launcher/ 2>/dev/null || true
cp $P/*.step $D/step/; rm -f $D/step/*assembly*.step
# firmware v4 (firmware/dart in the repository: sketch, PC tests, wiring tools and the build docs)
if [ -d $FW ]; then
  mkdir -p $D/firmware/kedge_dart $D/firmware/test $D/firmware/wiring
  cp $FW/kedge_dart/*.ino $FW/kedge_dart/*.h $D/firmware/kedge_dart/
  cp $FW/test/*.cpp $FW/test/*.sh $D/firmware/test/ 2>/dev/null || true
  cp $FW/*.md $D/firmware/ 2>/dev/null || true
  cp $FW/wiring/* $D/firmware/wiring/ 2>/dev/null || true
else
  cp -r fw/kedge_dart fw/test $D/firmware/
fi
cp $P/barrel_profile.* $P/barrel.json $D/launcher/; cp $OUT/launcher_fit.json $D/launcher/ 2>/dev/null || true
cp $P/flight_sim.png $P/plates.png $D/docs/ 2>/dev/null || true
cp $P/KEDGE_${V}_run_sheet.pdf $P/KEDGE_${V}_engineering_report.pdf $P/wire_cut_list.csv $P/wiring.png $OUT/audit_sheet.png $OUT/sections.png \
   $OUT/audit.json $OUT/physics.json $OUT/mass_props.json $OUT/wireroom.json $P/print_report.json $P/step_check.json $D/docs/ 2>/dev/null || true
cp DART_PHYSICS31.md $D/docs/ 2>/dev/null || true
cp variants_final.json $D/docs/variants_all.json
cp ${GEN:-kedge_v32.py} $D/source/kedge_${V}.py
cp audit27.py solve_deg.py clearance.py meshstep.py meshes.py viewer.html shots.js extra_shots.js sheet.py pack.py coupon.py wires.py wiring_dot.py \
   barrel.py assembly_step.py physics.py flight_sim.py refresh_variants.py plate_view.py report.py runsheet.py variants.py run_all.sh make_dist.sh \
   launcher_fit.py wireroom.py massprops.py stepcheck.py review.js review33.js review34.js montage.py pubshots.js requirements.txt \
   wires34.py runsheet34.py report34.py mkpub.sh ovl.py qrender.py $D/source/ 2>/dev/null || true
cp dart_*.py $D/source/ 2>/dev/null || true
mkdir -p $D/source/vendor && cp vendor/three.min.js $D/source/vendor/ 2>/dev/null || true
cd dist_$V && zip -qr KEDGE_${V}_build_pack.zip KEDGE_$V && (cd KEDGE_$V/print && zip -q ../../KEDGE_${V}_STL.zip *.stl *.3mf) && (cd KEDGE_$V/step && zip -q ../../KEDGE_${V}_STEP.zip *.step)
(cd ../$P && ls KEDGE_*assembly_folded.step >/dev/null 2>&1 && zip -q -9 ../dist_$V/KEDGE_${V}_assembly_folded_STEP.zip KEDGE_*assembly_folded.step) || true
ls -la *.zip
