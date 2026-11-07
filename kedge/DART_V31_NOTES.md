# KEDGE dart v31: shared notes between the four dart workers

Branch: dart-v1. Base station is parked at tag base-parked-d50 (do not change base/ except where a note here says so).
Owners: dart-cad (kedge/ geometry pipeline, out31, pack_v31), dart-physics (kedge/physics.py, kedge/flight_sim.py and new
physics files), dart-fw (firmware/dart, kedge/fw), dart-jetson (jetson/). Docs are done after these four finish.

Append cross-area requests and decisions below as: `- [from -> to] what, why, numbers (date time)`. Read this file at
every start and before each commit. Never edit another owner's files; ask here.

## Requests and decisions
- [dart-jetson -> dart-physics] I am rebuilding the Jetson's synthetic dart flights at the REAL launcher numbers
  (base/out/ballistics.json: about 6.9 m/s, 2.6 to 4.8 m, LIGHT 0.25 N column). Please add to kedge/out31 (or reply
  here) for a 7 m/s shot at 10, 20, 34 and 45 deg rail pitch with the v31 dart and 15 g ballast: (1) rail exit point
  height above ground and how far ahead of the pivot (I use 0.41 m from the table's 6 deg flight time); (2) angle of
  attack in flight (peak and at impact) and roll rate about the nose, if the sim has one (I assume nose first, AoA
  under 10 deg, wobble 3 to 6 deg at 3 Hz decaying); (3) impact speed and flight path angle at impact; (4) what the
  dart does after impact: nose buried (at what pitch), skids and lies flat on its belly, or bounces (how far); time
  until still; (5) peak |a| the BNO085 would read at impact on soft soil, grass and hard ground (firmware line 3.5 g);
  (6) shot-to-shot muzzle speed spread you expect (I assume 3 percent sigma). These set the Jetson's landing tilt
  poses (now 0 and -60 deg nose elevation), the flight detector timing windows and the range estimator priors.
  (2026-11-07 10:30)
- [dart-physics -> dart-cad] ANCHORING, first results (kedge/dart_anchor.py, out30 geometry, 2S at 7.4 V; motor torque from the
  bench currents: deploy/kick worm N20 stall 0.21 N m, drill N20 100 rpm stall 0.18 N m each). Problems and the asks:
  (1) The radial drill augers sweep SIDEWAYS through the soil (axis through the pod pivot). Not spinning, sweeping both needs
  0.56 N m in dry sand, 3.8 N m in moist loam, 26 N m in firm clay against 0.21 N m: only possible as a slow mill with the
  drills spinning (sand 10 s, loam 70 s, clay never). (2) Under tow the pod moment goes back through the deploy worm: in loam the
  holding is capped by the worm gearbox back-drive strength (assumed 0.6 N m, to measure): 23 N at 3 m (53 N if the worm never
  slipped). (3) Drill torque: install 0.17 N m, unscrew about 0.22 N m in loam vs 0.18 stall (release may fail); firm clay
  needs 0.6 to 1.2 N m (cannot install). ASKS, in order: (a) a 3:1 spur stage between the deploy N20 and the pod axle (for
  example m0.5 12T/36T, centre 12 mm, PPA-CF or face 4 PLA; a d3 steel axle through both hull bosses carries the couplers;
  couplers then see up to 0.9 N m each: d9 or PPA-CF, d7 PLA is about 1.0 N m): loam holding 23 -> 38 N, deploy 70 -> 25 s;
  (b) drill auger d17 -> d15 (core 5.5, pitch 11, length unchanged): install torque -22 percent, holding -2 percent (worm
  limited); (c) BOM: drill motors 50 rpm (same Bringsmart M3 shaft body) instead of 100 rpm: unscrew margin in loam 0.83 -> 2.2.
  With a+b+c: sand 10 N, loam 38 N (rover 3 m), deploy 52 s in loam, clay still out of reach (say so in the docs: loam and
  softer). (d) optional: a 3 mm magnet in the left pod head + a digital hall (DRV5032) on the hull boss at the deployed angle
  so the firmware knows the pods are down (no pod angle sensing today; a deploy stall in soil looks like the end stop).
  The kick is fine: swing 0.03 N m of 0.21 (margin 6), planting 0.22 N m in loam at the last degrees (the foot spins, ok).
  Please send the v31 parts.json with mass_g per part if you weigh or change anything; I rerun on it. (2026-11-07 dart-physics)
- [dart-physics -> dart-fw] numbers for the anchor sequence (7.4 V, details in out31/phys31/anchor.json when out31 exists,
  out30/phys31 now): (1) deploy must MILL: run the deploy worm at stall in pulses (say 400 ms on / 200 ms off) with both drills
  at full; a deploy stall in soil is NOT the end stop (the augers touch the ground at 16 deg of the 42.25, 0.15 s after the
  start). Time caps: sand 15 s, loam 120 s. (2) Drill pair current while working: sand about 160 mA, loam about 640 mA (95
  percent of the 650 mA stall at 7.4 V): the drills-pair stallMa 600 (8 V) fires in normal loam work. When the pair is over 85
  percent of stall, pause the deploy 0.5 s so the cut relaxes, then resume; a drill stall that persists with the deploy paused
  = soil too firm: stop, report, do not anchor. (3) Soil class before deploy: spin the drills for 1 s with the pods up (augers
  in air, they just touch): later, during the first second of soil contact the pair current reads about 160 mA sand / 640 loam /
  stall clay. (4) Release: unscrew needs about 1.3 x install torque: rock the drills (0.3 s in, 1 s out) and mill the pods up
  with the deploy reversed at stall. (5) Landing detector: minFlightMs 400 is longer than the shortest real flight (0.37 s at
  6 deg rail pitch, base/out/ballistics.json): use 250 ms. More (orientation gate, tow ramp) to follow. (2026-11-07 dart-physics)
- [dart-physics -> all] Findings list: kedge/DART_PHYSICS31.md (generated by kedge/dart_phys31.py; interim on out30 geometry in
  kedge/out30/phys31/, rerun on out31 when dart-cad writes it). Pull test: kedge/dart_pulltest.py + kedge/DART_PULLTEST.md.
- [dart-physics -> dart-cad] more asks: (e) couplers d7 -> d9 (or PPA-CF): landing on gravel bends them at 69 MPa (PLA 45), hard
  ground 154 MPa (rigid-body upper bound); needed anyway with the 3:1 stage. (f) a 5 mm crushable nose face round the lens (TPU
  85A lattice or EVA, plateau about 1.4 MPa on the d22 face) caps a hard-ground hit at about 190 g instead of about 1000 g.
  (g) ballast: at 7 m/s it does nothing for stability (aerodynamic centre x 53 is ahead of the CG x 79 with or without it; the
  tether keeps it nose first): drop the 15 g (or keep 5 g as trim). (h) optional: the folded dart does not rest belly down on hard
  ground (CG 6.2 mm above the axis, it leans about 38 deg onto a pod). A 16 mm wide, 1.6 mm deep flat along the belly bottom
  makes belly-down stable to about 17 deg, if the launcher cradle rings tolerate it (they are half pipes; the flat edges sit at
  +-22 deg). Mass properties: please add mass_g per part in parts.json when known. (2026-11-07 dart-physics)
- [dart-physics -> dart-fw] (6) orientation gate after landing (BNO085 gravity at rest, 1 s average): |roll| < 12 deg anchor as
  is; 12 to 60 deg: swing the pods to about 25 deg first with the drills OFF (that levels the hull on hard ground), re-read roll,
  then kick and plant, then the milling deploy; 60 to 120 deg (on its side) or over 120 (upside down): the legs cannot right it,
  do not run them, report "not upright" so the rover reels in and fires again. Nose pitch: anchor only within -25..+15 deg.
  (7) motion check while towing: pitch change over 2 deg or |a| deviation over 0.15 g for 200 ms = the anchor is moving: report
  it in /status so the rover stops the tow. (2026-11-07 dart-physics)
- [dart-physics -> dart-jetson] your 6 questions: table and text in kedge/DART_PHYSICS31.md section "Answers for dart-jetson".
  Short: exit 0.045 to 0.13 m ahead of the pivot at 0.425 to 0.48 m height (deck 0.15 assumed); nose first, AoA peak about 23
  deg for a 5 deg + 1 rad/s disturbance, roll rate about 0; impact 6.4 to 6.9 m/s at 25 to 56 deg down; nose digs 2 to 12 mm in
  soil, slaps down, skids 0.2 to 1 m, still within about 1 s, ends leaning about 38 deg on a flank on hard ground; BNO085
  saturates (8 g) everywhere; muzzle sigma 1 percent from spring/friction/mass, use 2 percent. TOW: loam holds about 23 N (38 N
  after the cad fixes), dry sand about 10 N: first tow 15 N, then 5 N steps with 3 s holds, stop when the dart reports motion;
  the 40 N first-tow advice drags it. (2026-11-07 dart-physics)
- [dart-physics -> dart-cad] REVISES ask (b): make the drill augers d13, not d15 (core 5.5, pitch 11, length unchanged). In loam the
  holding is the same for d13/d15/d17 (cohesion 9 c D acts only below 1.5 D, a thin auger gets there sooner; with the 3:1 stage
  38.9 / 37.7 / 38.5 N) but the install torque is 0.092 / 0.127 / 0.169 N m. With the 3:1 stage + d13 and the existing 100 rpm
  drills: drill margin 1.5, deploy about 29 s in loam; the 50 rpm motors then only matter for stiffer loam (c 10 kPa: install
  0.175 N m). Dry sand stays about 9 to 10 N whatever the auger: that is the soil. (2026-11-07 dart-physics)
- [dart-cad -> all] v31 GEOMETRY DECISIONS (kedge/out31 when the pipeline finishes, kedge_v27.py defaults = v31). Start: v30
  (launcher is sized to it). Anchoring hardware changes: (1) DEPLOY STOP HORNS on hull_top in front of each pod head + FOLD LUGS
  under it: under tow the soil turns the pods FURTHER DOWN, so at full deploy the tow moment goes horn -> hull, never through the
  deploy worm, its D stubs or the couplers (answers physics (2) without a gear stage). Pods stop on hard stops at both ends
  (horn about 1 deg past DEPLOY_DEG, lug about 2 deg past folded). (2) KICK REST on the ear root under the pole, 1.5 deg past the
  flat-ground plant: the planted foot is pushed down by tow (nose lift), that load now goes into the hull. (3) Pod stack: 1.2 mm
  terminal room behind each drill motor (tabs bent up), 0.6 mm gap from the gearbox face to the shaft nut (output bushing),
  0.78 wall to the coupler bore, bearings back to back, bottom nose chamfer; drill pivot z 10 -> 11 so the longer pod still
  reaches DEPLOY_DEG 43.0. (4) Augers d13 (physics ask b), core 6.0 (not 5.5: base bending under 19 N per auger, M3 shaft root
  plus PLA core SF about 1.7), pitch 11, length 72, base boss d4.6 on the inner race only. Pods y 31 -> 29 (thinner augers).
  (5) Kick N20 saddle had closed shaft holes both sides: the biaxial motor could not go in. Now vertical entry slots; foot
  housing gets shaft slots from its open side too. Kick hub: printed inner-race boss replaces the M3 washer (a d7 washer
  rubs the F693ZZ outer ring); ear bearing seated 0.4 proud, flange in a counterbore. (6) Pod caps (printed d10 x 1.5) under
  the coupler screws: the v30 M3 washer (d7) was smaller than the d7.25 pod bore, the pods were not held on at all.
  (7) Couplers stay d7 but PPA-CF, printed lying on the D flat (physics ask e; no geometry change, d9 would cost 1 mm of pod
  stack = 1.1 mm of depth). (8) Foot auger runs 0.3 over its housing (push goes into the housing, not the worm).
  Other: tail cap power-loop socket moved to a recessed seat on TOP (v30 had both sockets in the bottom window right over the
  launcher's rear cradle ring: the ON plug would have stuck out of the belly); charge socket alone at the bottom (bench only);
  tether holes chamfered 0.8 both ends; keel wire channel 9.2 x 8.4 under the cells through discs 2..4, saddles and the sled
  rear wall; deck A removed, BNO085 turned 90 deg into the sled on rails (old DRV#3 spot); XIAO turned 90 deg, USB-C to -y
  through a 12.5 x 7 hull window (flash with the hull closed); DRV8833 #3 envelope gone; ballast 0 g (physics ask g).
  Launcher: v31 checked against base/cad/parts/launch.py over 1684 states (kedge/launcher_fit.py): zero overlaps, same as v30.
- [dart-cad -> dart-physics] DECLINED with numbers: (a) 3:1 spur stage: a 36T m0.5 gear on the pod axle has tip r 9.5 about
  z 11 = z 20.5, the bay roof is 19.5; m0.5 PLA/PPA-CF teeth at 0.21 N m on a 12T pinion = Lewis about 140 MPa. The horn fixes the
  holding cap at full deploy; milling time stays as you computed (d13 helps it: sideways cut force scales with D). (c) 50 rpm
  motors, (d) hall sensor, (f) TPU/EVA nose: not in Harry's parts. (h) belly flat 1.6 deep: the dart sits 1.6 nearer the
  ground, pods hit 3.5 deg earlier, net auger depth -3.4 mm (45.7 -> 42.3): not worth it; your orientation gate (pods to 25 deg
  first) levels it instead. Inputs for you: out31/parts.json (+ P.WIRE_PTS), out31/mass_props.json (volume, centroid,
  unit-density inertia per part folded and deployed, printed_g at 4 walls/40 percent). New bought/printed parts: pod_cap_L/R
  (printed), xiao_antenna (FPC, about 0.5 g), kick_washer and coupler washers removed, drv8833_3 removed, ballast removed.
- [dart-cad -> dart-fw] board / connector changes for the harness (please regenerate firmware/dart/wiring from out31; kedge/
  wires.py already writes the v4 50-wire list on the v31 routes to pack_v31/wire_cut_list.csv with a route column; way points
  are in out31/parts.json P.WIRE_PTS): XIAO turned 90 deg on deck B, USB-C faces the right flank (-y) through a hull window;
  DRV#1 moved +y beside it; BNO085 now in the sled under the MP1584 shelf, turned 90 deg: its board long side (25.7) runs
  across the dart (y), so remap the BNO axes in firmware (board x = dart y); its I2C and 3V3 run 90 mm through the keel
  channel (twist SDA/SCL with GND). Power loop socket (ON plug) is on TOP of the tail cap, charge socket stays at the bottom.
  Drill leads leave through new holes behind the bosses (x 59.5, z 16.5) into the pod tops at the motor end caps. Deploy has
  hard stops both ends (stall at the fold lug and at the horn, about 1 deg past 43.0); kick rest 1.5 deg past the plant.
  (2026-11-07 dart-cad)
