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
