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
