#!/bin/bash
set -e
V=$1; OUT=out${V#v}; P=pack_$V; D=dist_$V/KEDGE_$V
rm -rf dist_$V; mkdir -p $D/{print,step,firmware,launcher,docs,source}
cp $P/*.stl $P/*.3mf $D/print/; mv $D/print/barrel_keepout.stl $D/launcher/ 2>/dev/null || true
cp $P/*.step $D/step/; rm -f $D/step/*assembly*.step
cp -r fw/kedge_dart fw/test fw/jetson fw/reel $D/firmware/; rm -f $D/firmware/test/test_logic $D/firmware/test/t; rm -rf $D/firmware/jetson/__pycache__
cp $P/barrel_profile.* $P/barrel.json $D/launcher/; cp $P/flight_sim.png $P/plates.png $D/docs/ 2>/dev/null || true
cp $P/KEDGE_${V}_run_sheet.pdf $P/KEDGE_${V}_engineering_report.pdf $P/wire_cut_list.csv $P/wiring.png $OUT/audit_sheet.png $OUT/sections.png $OUT/audit.json $OUT/physics.json $P/print_report.json $D/docs/
cp variants_final.json $D/docs/variants_all.json
cp kedge_v27.py $D/source/kedge_${V}.py
cp audit27.py solve_deg.py clearance.py meshstep.py meshes.py viewer.html shots.js extra_shots.js sheet.py pack.py coupon.py wires.py wiring_dot.py barrel.py assembly_step.py physics.py flight_sim.py refresh_variants.py plate_view.py report.py runsheet.py variants.py run_all.sh make_dist.sh $D/source/
cd dist_$V && zip -qr KEDGE_${V}_build_pack.zip KEDGE_$V && (cd KEDGE_$V/print && zip -q ../../KEDGE_${V}_STL.zip *.stl *.3mf) && (cd KEDGE_$V/step && zip -q ../../KEDGE_${V}_STEP.zip *.step)
(cd ../$P && ls KEDGE_*assembly_folded.step >/dev/null 2>&1 && zip -q -9 ../dist_$V/KEDGE_${V}_assembly_folded_STEP.zip KEDGE_*assembly_folded.step) || true
ls -la *.zip
