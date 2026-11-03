# GrappStar base station spec

Status: v0.1 (3 Nov 2026, 10:45 Sydney). Sections 1 to 4 are decided and binding now. Numbers marked TBD arrive in
v1.0 (about 12:00) from the architecture panel plus the workers' NOTES files. `params.py` holds every shared number;
this file explains them. Workers never edit either file: write change requests in your REPORT.md or NOTES.md.

## 1. Concept (baseline, binding)

One 12 V wiper motor (self-locking worm) drives the whole base. A main shaft runs coaxial with the wiper output and is
also the rail pivot axis (as in Harry's v10 launcher). A sliding dog collar keyed to the main shaft couples it to
either the LAUNCH output gear or the REEL output gear (both turn free on the shaft), or to nothing (NEUTRAL).

An N20 selector drum (right-angle self-locking worm N20) moves the collar and sequences everything else. The order is
physical, along one drum angle: LAUNCH, NEUTRAL, REEL, then a long REEL dwell during which a segment gear drives the
wrap arm round the capstan (wraps); turning back unwraps first, then shifts. The N20 worm holds any drum angle unpowered.

- LAUNCH mode, wiper FORWARD: drive gear through sprag #1 turns the big gear on the rail, which draws the carriage
  rack back against the two springs, releases at the relieved last tooth (fire), and re-engages (reset). One big gear
  turn per shot.
- LAUNCH mode, wiper BACKWARD: sprag #1 freewheels; a pawl-free printed one-way element (swing-arm direction splitter)
  drives a printed self-locking worm stage, which turns the v10 crank-rocker: rail pitch oscillates between its limits.
  The worm holds the pitch while cocking.
- REEL mode, wiper FORWARD: the reel train turns the capstan (wind-in) and overdrives the storage reel through the
  slip clutch (take-up behind the capstan). Sprag #2 is the capstan backstop to the frame: the capstan never turns
  out, so the tow load holds in every drum state, and the slip clutch has a reaction during payout.
- Payout at launch: the line bypasses the capstan and pays straight off the storage reel, which spins free on 2 x 608
  against the slip clutch set light (0.5 to 1 N at the line) by the drum's brake track.
- Fault: wiper off, drum to NEUTRAL. Manual lever on the drum shaft end as the hand fallback.

## 2. World frame (binding)

Right-handed, millimetres. Origin: on the wiper output shaft axis, in the plane of the motor adapter plate face.
+X forward (launch azimuth, horizontal), +Y left looking forward, +Z up. The main shaft axis is the Y axis through
the origin unless v1.0 says otherwise (the wiper output shaft points along +Y or -Y: TBD by the layout).
Angles: rail pitch is measured up from +X. Drum angle 0 = LAUNCH. Wiper angle positive = FORWARD (the cock direction).

## 3. Ownership (binding): every part has exactly one owner

| Worker | Owns (geometry, kinematics, print orientation, assembly steps) |
|---|---|
| cad-drum | main shaft; wiper shaft coupler; dog collar; LAUNCH output gear and REEL output gear (with their dog faces and bushings or bearings on the main shaft); shift fork and its guide; selector drum barrel with all tracks (fork groove, brake track, segment gear and locking arc, code bumps); N20 mount and the N20-to-drum gears; wrap drive idler(s) up to the mesh with the wrap ring gear; drum position contacts; manual lever; AS5600 magnet holder on the main shaft end |
| cad-launch | everything driven from the LAUNCH output gear onward: launch input gear(s), sprag #1 hub and drive gear, big gear and its pin, rail (split for printing), cradle and barrel for the dart, carriage and rack, springs, spring anchors and any pulleys, front stop bumpers, dart pusher and tether exit at the tail, the aim train (swing-arm splitter, worm and wheel, crank, conrod, lug), rail pivot bushings or bearings on the main shaft, launcher contacts (dart seated, carriage home, rail low) |
| cad-reel | everything driven from the REEL output gear onward: reel input gear(s), capstan, capstan shaft, sprag #2 backstop hub (bolts to a frame block), storage reel and its 608s and axle, slip clutch with preload lever (the drum's brake track pushes this lever: reel owns the lever, drum owns the track), centrifugal shoes if used, wrap cup or arm with its ring gear and bearing, all line guides and eyes, the exit eye, reel contacts if any |
| cad-frame | base deck and rover mount, cheeks and bearing blocks for every shaft at the layout positions, wiper motor adapter plate (parametric), electronics bay (ESP32-S3 DevKitC-1, BTS7960, DRV8833, MP1584, optional HX711, fuse), AS5600 board bracket, guards and covers, cable routes, all joints between frame pieces (keys, M3 heat-set inserts) |
| physics | numbers only (stroke, ratios, clutch torques, wraps, ballistics table); no geometry |
| coordinator | params.py, this spec, the assembly timeline, integration |

Interface rules:
- A gear mesh across two owners: the owner of the driving side states the module, tooth count, face width and profile
  shift in its NOTES; the centre distance comes from params.py LAYOUT; the audit checks the mesh at 1 degree steps.
- Bearing seats in frame blocks: the shaft owner states the bearing (608, CSK20PP outer key, bushing) and the axial
  stack; the frame owner builds the seat from the shared library.
- Levers and cams across owners (brake lever, fork): the cam owner states the follower path; the lever owner states
  the lever geometry and preload; both use the same params.
- Contacts: the mechanism owner builds the printed contact (M3 screw head touching an M3 nut) in its parts; cad-frame
  routes the wires; fw-base owns the GPIO table.

## 4. Sensors and contacts (names binding, positions TBD)

| Name | Where | Closes when | Owner |
|---|---|---|---|
| `AS5600` | main shaft end, magnet on the shaft | (angle sensor) | drum (magnet), frame (board) |
| `SW_DRUM_A`, `SW_DRUM_B` (`SW_DRUM_C` if needed) | drum code bumps | code table in v1.0 | drum |
| `SW_SEATED` | cradle at the dart tail | dart seated against the pusher | launch |
| `SW_HOME` | rail, carriage front stop | carriage fully forward (uncocked) | launch |
| `SW_RAIL_LOW` | crank or rail | rail at minimum pitch | launch |
| `SW_WRAP_HOME` (optional) | wrap ring | wrap arm at home | reel |
| `HX711` (optional) | load cell in the exit eye mount | (tension) | reel (mount), frame (board) |
| BTS7960 IS | electronics bay | (wiper current) | frame, fw-base |

## 5. Layout, trains, drum timing, physics, mission timeline: TBD in v1.0

## 6. Decisions log

- 3 Nov 10:45: baseline concept above (agreed plan + coordinator calls on the drum mechanism, sprag #2 as backstop,
  swing-arm aim splitter, main shaft = rail pivot). Harry was away and answered no questions; assumptions below.

## 7. Assumptions Harry must confirm (goes into the final MEASURE_LIST.md)

- Wiper motor: shaft diameter, flats or spline, end thread, mount hole pattern, housing size, torque and speed.
- Springs: rate and initial tension (hang a 1 L bottle on one spring, measure length before and after).
- Hardware box: M8 bolts or 8 mm rod for 608 axles (printed PPA-CF axles otherwise), a spare MP1584 for the drum N20,
  the 100 kg load cell + HX711.
- Rover: deck size and mounting holes; whether the Jetson should drive it.
- Dart build: v30 assumed.
