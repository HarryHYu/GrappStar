# GrappStar launcher: context handover (2 Nov 2026)

## Where it stands
Desk size (0.3 scale) spring dart launcher, one motor, three one way bearings, piston pitch. Current deliverable is the HAND TEST build v10: everything printed, no motor plate, printed bushings in place of the one way bearings, every pin snaps in with a printed C clip. Harry is printing and assembling it now. Sim artifact: claude.ai/artifact/HB9HhCbGiG9pXYUQk1KHHn (v14). Latest files in /mnt/user-data/outputs: grappstar_desk_v10_handtest_stl.zip, GrappStar_desk_v10_handtest_print_sheet.pdf, GrappStar_PISTON_SIMULATION.html, 07_frame.stl (latest frame alone).

## Harry's standing rules
Very brief replies, no em dashes. Everything thick and strong. No hand operation in the final design (hand test build is the exception, he asked for it). One motor only, no worm gear, no printed pawls. As few parts as possible. STLs only after he approves the sim. The sim must be an exact copy of the STLs. Do not break compatibility with parts he has already printed: when fixing one part, change only that part. He gets angry fast when things take long; keep rounds short.

## Mechanism (final design, gen_oa35 lineage)
- Rail body (rail beam 64 x 60 with V grooves, barrel tubes, bracket) pivots on the motor axle at (PX, PY) = (-460.7, -240.7) full scale, rail coords.
- Big gear (19T module 10, 14 teeth present, 48 thick = 14.4 mm print) is mounted ON the rail on its own pin at (CAMX, CAMY) = (-350, -130), meshing the rack on the carriage. Because it rides with the rail, cocking pulls the rail DOWN onto the piston, same direction as its weight, so one backstop holds everything. This was the key fix: earlier layouts with the gear on the frame had cocking lifting the nose and no one way bearing could hold it.
- Drive gear 12T on the axle, bearing A inside (forward turns drive it, backward freewheels), drives the big gear. Ratio: 1.583 motor turns per shot.
- Pinion 16T module 8 on the axle, bearing B inside (backward turns drive it, forward freewheels), drives the crank gear 32T module 8 on a pedestal in the thick minus leg. Crank stub sits in bearing C (backstop).
- Crank pin (radius 50) drives a connecting rod (44 x 24, 260 long) up to a lug pin on the rail bracket (192 from the pivot, straight above it). Backward motor turns raise pitch 6 to 47 degrees; past the top the crank rolls over and the launcher drops to 6. Working run psi 334 down to 174 (clockwise).
- Last tooth of the big gear cut to the pitch line (tapered toward the rail), tooth before it 30 percent taller, so the two let go together. Release at TH_DRIVE0 + 287.
- All rods 12 mm, bearings HF1216 (12 x 18 x 16). Bushings on the test build are the same size, swap later.
- Spring posts Ø44 on a 48 x 30 bar at the muzzle, carriage pegs Ø32.
- Snap fit: pins have a head one end, groove 6.5 wide x 3 deep (full scale) the other, C clip Ø54 with an opening 0.84 of the groove core. 6 clips, 3 bushings.

## Pieces in the hand test build (print one each unless noted)
01 big gear, 02 drive gear, 03 axle (one piece with D socket head and groove), 04 carriage, 05 dart, 06 rail body (rail + barrel + bracket + lug pin printed on), 07 frame (legs, base, pedestal, NO motor plate), 08 pinion, 09 crank gear (stub printed on), 10 conrod, 11 big gear pin, 12 clip x6, 13 bushing x3, 14 crank pin. 21 pieces.

## Fixes made since the first v10 zip (frame only, reprint 07_frame.stl)
1. Base plate extended 27 mm forward so the pedestal leg stands on it (was hanging off).
2. Motor plate hole opened so the axle head could pass. Then
3. Motor plate removed entirely for the hand test (axle head sits outside the plus leg, turned by hand).
The zip and sim now carry the motor-plate-free frame. When the motor goes back on, re-add the plate from gen_oa34 (box 160 x FH x 20 at ZMOT with a 71 radius boss) but with the hole 35.5 radius so the head passes, located by the 6 M3 holes.

## Code (all in /home/claude, manifold3d CSG, exported at 0.3)
- gen_oa35.py: current generator (gen_oa34 = same plus motor plate). Chain: gen_oa21 (self aim) > 22 (tilt frame) > 23 (piston + belt) > 24 (keyed crank) > 25 (motor on crank, herringbone) > 26 (bearings back, big gear on rail) > 27 (thicker) > 28 (spur, relieved last tooth, rail 54 deep) > 29 (two teeth release together, rail 44 wide) > 30 (12 mm rods, HF1216) > 31 (12 mm piston pins, rail 64 x 60) > 32 (m8 pitch gears 48 thick) > 33 (thick spring posts, CSG measured crank phase PH_C) > 34 (snap fit, bushings) > 35 (no motor plate).
- build_oa35.py: part placement list and roles (gear, drive, axle, pinion, crank, rod, rail, carriage, dart, fixed); PH_D drive gear phase found by CSG.
- collide_oa34.py (works against build_oa35 if the path is changed): full sweep, expect "bad 5" (crank vs frame a few mm3, known), mesh checks 0.
- build_small35.py: merges rail body, scales 0.3, writes stlsmall35, PRINT_SHEET, PDF and zip to /mnt/user-data/outputs, manifest_small35.json for the sim.
- /tmp/mk38.py: builds the sim page from sim24_head.html + sim24_tail.html and the manifest, writes sim_local.html, grappstar-piston-sim.html (artifact) and GrappStar_PISTON_SIMULATION.html (offline, three.js inlined). /tmp may be gone in a new container; the sim24 head/tail files live in /home/claude.
- probe.js and /tmp/shot.js: headless Playwright checks (errors, pitch readouts, screenshots to frames/).
- To change a part: edit gen_oa35.py, run it, run collide, run build_small35.py, run mk38, publish the artifact by the same file path, send the zip. Keep every other part's geometry untouched if he has printed it.

## Known limits / open items
- Backward 15 degree step lands at about 13 degrees (calibration only).
- Dropping from max pitch past the top is a drop, not a glide; he accepted it.
- Rail hardware: he once considered an MGN12H linear rail instead of the printed V rail; not pursued.
- Motor version print sheet (v9, bearings and rods bought) exists: grappstar_desk_v9_stl.zip, but it predates the snap fit and the frame fixes; regenerate from gen_oa34 with the plate hole fix before using it.
- Next expected: assembly feedback from the hand test (fits, clip tightness, mesh), then one way bearings and the motor plate back on.
