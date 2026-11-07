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
