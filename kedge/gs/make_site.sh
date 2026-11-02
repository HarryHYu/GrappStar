#!/bin/bash
# rebuild everything the online build pack serves (run after run_gs.sh)
set -e
cd "$(dirname "$0")/.."
source .venv/bin/activate
python gs/guide_gs.py gs/out pack_gs
(cd gs && python overview_pdf.py out ../pack_gs)
rm -f site/*.zip site/*.b64.txt
(cd pack_gs && zip -qr ../site/GrappStar_GS_print_pack.zip print plates plates.json plates.png print_report.json GrappStar_ground_station_build_guide.pdf wiring_gs.png)
zip -qr site/GrappStar_GS_source.zip gs/*.py gs/*.js gs/inspect.html gs/sim_template.html gs/assembly_template.html gs/run_gs.sh gs/make_site.sh gs/ARCHITECTURE.md meshstep.py out30/parts.json out30/physics.json out30/audit.json out30/*.npz vendor/package/build/three.min.js vendor/package/examples/js/controls/OrbitControls.js vendor/package/LICENSE requirements.txt setup_mac.sh
zip -qr site/GrappStar_firmware.zip fw -x "*/__pycache__/*" "fw/test/test_logic" "fw/test/test_gs"
cp dist_v30/KEDGE_v30_STL.zip site/
for z in site/*.zip; do base64 -i "$z" -o "$z.b64.txt"; done
rm -f site/*.zip
cp pack_gs/GrappStar_mission_sim.html site/sim.html
cp pack_gs/GrappStar_assembly_steps.html site/assembly.html
cp pack_gs/wiring_gs.png site/wiring_gs.png
cp pack_gs/GrappStar_ground_station_build_guide.pdf site/
cp pack_gs/GrappStar_system_overview.pdf site/
python - <<'PY'
from PIL import Image
import glob, os
for f in sorted(glob.glob('pack_gs/renders/*.png')):
    im = Image.open(f).convert('RGB'); im = im.crop((0, 40, im.width, im.height)); im.thumbnail((1600, 1000)); im.save('site/' + os.path.basename(f).replace('.png', '.jpg'), quality=82)
im = Image.open('pack_gs/plates.png').convert('RGB'); im.thumbnail((1800, 1200)); im.save('site/plates.jpg', quality=82)
PY
ls -la site | awk '{print $5, $9}'
