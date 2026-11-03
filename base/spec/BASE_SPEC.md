# GrappStar base station spec

Status: v1.2 (3 Nov 2026, 12:30 Sydney). Everything here is binding, together with base/spec/DECISIONS.md (D1 onward)
and base/spec/params.py (the numbers). `params.py` holds every shared number;
this file explains them. Workers never edit either file: write change requests in your REPORT.md or NOTES.md.

## 1. Concept (baseline, binding)

One 12 V wiper motor (self-locking worm) drives the whole base. A main shaft runs coaxial with the wiper output and is
also the rail pivot axis (as in Harry's v10 launcher). A sliding dog collar keyed to the main shaft couples it to
either the LAUNCH output gear or the REEL output gear (both turn free on the shaft), or to nothing (NEUTRAL).

An N20 selector drum (right-angle self-locking worm N20) moves the collar and sequences everything else. The order is
physical, along one drum angle: LAUNCH, NEUTRAL, WRAP, REEL (D16). In the WRAP sector the collar stays in NEUTRAL
while a segment gear drives the wrap cup round the capstan (7 turns); REEL engages only with the wraps on. Turning
back unwraps first (collar in NEUTRAL, take-up running), then shifts. The N20 worm holds any drum angle unpowered.

- LAUNCH mode, wiper FORWARD: drive gear through sprag #1 turns the big gear on the rail, which draws the carriage
  rack back against the two springs, releases at the relieved last tooth (fire), and re-engages (reset). One big gear
  turn per shot.
- LAUNCH mode, wiper BACKWARD: sprag #1 freewheels; a pawl-free printed one-way element (swing-arm direction splitter)
  drives a printed self-locking worm stage, which turns the v10 crank-rocker: rail pitch oscillates between its limits.
  The worm holds the pitch while cocking.
- REEL mode, wiper FORWARD: the reel train turns the capstan (wind-in). Sprag #2 is the capstan backstop to the
  frame: the capstan never turns out, so the tow load holds in every drum state.
- TAKE-UP BUS (D15): every FORWARD wiper turn, in any mode, overdrives the storage reel through its slip clutch via a
  bus gear upstream of the dog and a forward-only swing idler: it winds the slack while cocking, tidies the line in
  NEUTRAL, takes up behind the capstan in REEL and during the unwrap. A drag brake parks the bus on backward turns.
- Payout at launch: the line bypasses the capstan and pays straight off the storage reel, which spins free on 2 x 608
  against the slip clutch set LIGHT (0.5 N at the line, D7) by the drum's brake track, reacting against the parked bus.
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

## 4. Sensors and contacts (binding; positions are owned by the mechanism owner, GPIOs by fw-base)

Printed contacts: an M3 screw head touching an M3 nut, ESP32 internal pull-up, closed = LOW, firmware debounce 20 ms.
The full list with suggested GPIOs is params.SWITCHES.

| Name | Owner | Closes when |
|---|---|---|
| `SW_DRUM_A`, `SW_DRUM_B`, `SW_DRUM_C` | cad-drum | Gray code of the drum zone (params.DRUM.codes); 000 = broken wire fault |
| `SW_DOG_L`, `SW_DOG_R` | cad-drum | collar engaged with the LAUNCH dog ring / the REEL gear (checked during every drum move) |
| `SW_N20` | cad-drum | N20 in mesh (open = hand lever latched out) |
| `SW_INDEX` | cad-drum | 6-bump index cam on the main shaft (D12 fallback when the AS5600 has no magnet) |
| `SW_HOME` | cad-launch | carriage on its front stop (uncocked) |
| `SW_GAP` | cad-launch | big gear in its toothless gap (rest) |
| `SW_RAIL_LOW` | cad-launch | rail at its minimum pitch (crank low dead point) |
| `SW_SEATED` | cad-launch | dart tail against the pusher |
| `SW_ARM` | cad-reel | wrap cup ring at home (one pulse per cup turn) |
| `AS5600` | cad-drum (magnet cap), cad-launch (bracket on the aim housing) | wiper angle, multi-turn (magnet: any small disc edge-on, D12) |
| `HX711` (optional) | cad-reel (mount), cad-frame (board) | line tension |
| BTS7960 IS | cad-frame, fw-base | wiper current |

## 5. Layout (binding; numbers in params.LAYOUT)

- Main axis = world Y, 165 above the deck top (deck_top_z -165). The wiper motor sits on the -y side of the adapter
  plate with its gearbox and can ABOVE the axis (WIPER.orient_deg 90), so the body stays clear of the wrap cup.
- Main axis stack (world y): coupler with the printed shear-pin torque limiter 0.5..46 (D26); take-up bus gear
  m1.5 50T 40..48 (D25); REEL output gear m2 50T 50.5..62.5 (dogs to 67.5); dog collar 72..98 at NEUTRAL (stroke 7
  each way); LAUNCH dog ring 100.5..111 bolted to the launch sleeve flange; launch sleeve 111..193 (rail cheeks,
  two-layer big gear mesh, drive gear 26T m2.5 on sprag #1 at 142.5..158.5, splitter sun 174..182) with its outboard
  bushing and thrust face in cad-launch's aim housing at 187..193 (D23); d12 spigot through the sleeve to about 200;
  magnet cap 195..203; AS5600 bracket on the aim housing at about 205. Owners may move a part inside its band and
  must report the final numbers.
- Rail mid plane (drive gear, big gear) at y 140; the rail body spans about y 95..185 and stays above w 35 near the
  pivot so it clears the collar and dog ring at every pitch.
- Capstan axis 72.1 from the main axis at 255 deg (x -18.66, z -69.64): REEL gear 50T meshes the capstan gear 22T
  (capstan 2.27 x wiper). cad-reel's frame R sits with its origin at (x -18.66, y 33.5, z -69.64), R +X = world +X,
  R +Y = world +Z, R +Z = world -Y: the capstan gear plane (R Z -29..-17) lands on world y 50.5..62.5.
- Selector drum axis at (x 60, z -112), y 1..142, parallel to the main axis: front-below, clear of the big gear sweep
  (lowest z about -44) and of the reel gear zone (x up to about +8). The fork reaches up to the collar groove (y 82..88).
  The wrap idler (segment gear pinion plus a 48T m1.5 output gear) sits between the drum and the capstan (89 mm apart)
  and its output gear runs in the cup gear plane (cad-reel R Z -43..-35 = world y 68.5..76.5).
- Aim pedestal: crank centre at about (x 58, y 195, z -100.5), about 55 outboard of the rail plane (cad-launch).
- Electronics bay on the deck in front of the drum (x 110..240, y -70..60). Kiyo X camera mount and RC832 holder on
  the frame (D18), camera pose in params.FRAMEWORK.camera.
- Keep-out zones (params.LAYOUT.keep_out): the rail sweep, the wiper body, the line path.

## 6. Cross-owner interfaces (binding)

| Interface | Driving side | Driven side | Numbers |
|---|---|---|---|
| LAUNCH dog ring to sleeve flange | cad-drum ring | cad-launch sleeve | flange face y 111, OD 64, 3 x M3 at r 28 + 3 lugs; the ring's joint follows the sleeve |
| REEL output gear to capstan gear | cad-drum m2 50T | cad-reel m2 22T | centre distance 72.1, plane y 50.5..62.5 |
| Take-up bus | cad-drum bus gear m1.5 50T on the outer coupler, plane y 40..48 | cad-reel c1 m1.5 46T on the capstan tube, swing idler 30T, reel gear 20T, clutch | centre 72.1, clutch disk 2.5 x wiper, no drag brake (D25) |
| Wrap train | cad-drum segment gear, pinion (4 turns), idler output 42T m1.5 in plane y 68.5..76.5 | cad-reel cup gear 24T m1.5 | centre 49.6; 7 cup turns across drum 62..302, ring at home at both ends; locking arc holds the cup outside the sector |
| Brake track to clutch pushrod | cad-drum track 2 | cad-reel pushrod (fail-safe LIGHT) | LIGHT 0.5 N / FIRM 5.0 N at the line, reel full, per drum zone (params.DRUM.zones) |
| Fork to collar | cad-drum | cad-drum | groove y 82..88 at NEUTRAL |
| Launch sleeve far end | cad-launch sleeve journal | cad-launch aim housing bushing (housing on the deck, frame drills 4 x M3) | y 187..193, thrust face (LAUNCH engage pushes outboard) (D23) |
| Rail pivot pulley and line channel | cad-reel pulleys | cad-launch bosses | D14 |
| Every bearing seat | shaft owner states bearing and stack | cad-frame builds the seat | shared library seats |

## 7. Drum zones (binding; params.DRUM = cad-drum drum_NOTES v4)

LAUNCH (-4..8, collar LAUNCH, clutch LIGHT, cup locked) / L_TO_N (8..32) / NEUTRAL (32..50, clutch FIRM) /
WRAP_PREP (50..62, clutch to LIGHT, cup lock releases at 60) / WRAP (62..302, collar NEUTRAL, clutch LIGHT, 7 cup
turns = 4 pinion turns x 42/24) / N_TO_R (302..340: cup locked, clutch LIGHT to FIRM 302..316, collar NEUTRAL until
318 then ramps to REEL) / REEL (340..352, collar REEL, clutch FIRM). Park targets (never on a code edge): LAUNCH 0,
NEUTRAL 41, WRAP_PREP 56, WRAPPED 309, REEL 347. End stops -4 and 352 (a 4 deg gap keeps the code ramps from
aliasing). Gray code (A B C): LAUNCH 100, L_TO_N 110, NEUTRAL 010, WRAP_PREP 011, WRAP 001, N_TO_R 101, REEL 111;
000 = broken common wire. SW_DOG_L / SW_DOG_R are checked during every move (the wrap window is never entered with
either closed). Unload jog before any shift out: 8 deg. A fault from REEL or N_TO_R goes to WRAPPED (309), never
NEUTRAL; a boot inside WRAP stops and reports; REEL jogs forward only.

## 8. Launcher and aim (binding; params.LAUNCH, params.AIM)

Coaxial launch sleeve with an integral key rib (D13). Drive gear 26T m2.5 on sprag #1, big gear 40T m2.5 (33 teeth,
v10 relieved last tooth) on the rail at 82.6 from the pivot: 1.538 wiper turns per shot. Springs 1:1, preload 35,
stroke 245, 280 at full cock, fallback hole set at preload 5, provision for a second pair (D5). Compliant stop at
least 15 mm under 300 N (D6). Aim: swing-arm splitter on backward turns, self-locking worm, crank-rocker 6 to 47 deg
(cad-launch may extend the minimum down to about -10 for retrieval), 10 wiper turns per crank cycle, aim only at
HOME. Retrieval funnel and lip roller at the muzzle; never fire without the tether (D11).

## 9. Reel (binding; params.REEL)

Capstan d 40 with sprag #2 as the backstop (wind-in only), capstan gear 22T (2.27 x wiper), 7 wraps laid by Harry's
v7 wrap cup driven through a hollow capstan tube. Storage reel (d 50 core, d 72 flanges, two-piece) on 2 x 608 with a shroud,
face slip clutch LIGHT 0.5 N / FIRM 5.0 N (line tension with the reel full, D24) set by the drum's brake track
through a fail-safe pushrod, driven by the take-up bus on every forward wiper turn (D15, D25; no drag brake). Line over 693ZZ pulleys (D14). Tow limit default 50 N.

## 10. Firmware behaviour (binding; details in firmware/base/PROTOCOL.md)

Sequence (D16, D17, D22): boot and rebuild state -> aim (backward, only at HOME) -> arm and fire (forward one shot; the
take-up winds the slack) -> flight -> LANDED (landing-zone verdict from the Jetson) -> unload jog, drum to NEUTRAL ->
WRAP at once while the line is slack (drum 62..302, wiper off) -> WRAPPED park (309) -> wait ANCHORED -> TIDY (forward by
time: the FIRM take-up pulls the slack through the free capstan) -> REEL fly-in (wiper creeping forward while the
drum enters REEL, then SW_DOG open) -> TOW (tow current limit) -> dart release -> reel in on the capstan until about
1.5 m remain -> unload jog -> drum back to the WRAPPED park 309 (collar to NEUTRAL) -> UNWRAP (drum
302..62 while the wiper turns forward at about 30 percent, take-up only) -> NEUTRAL final approach on the FIRM
take-up until SW_SEATED (fault DART_STUCK after 20 s) -> LAUNCH. A no-go landing retrieves the unanchored dart on
the take-up (or after a wrap, on the capstan).
De-cock: from a partial cock run backward until SW_HOME and SW_GAP close. Refusals: no fire unless LAUNCH, SW_HOME,
SW_GAP, SW_SEATED and a valid token; aim only at HOME; no drum move unless the wiper is stopped; no leaving LAUNCH
unless SW_HOME; wiper backward refused outside LAUNCH except the 30 deg unload jog. Current trips (D8): ignore 50 ms
after a start; 5 A for 30 ms in aim, reel and tow; 6.5 A while cocking; tow at free current plus the 50 N equivalent
averaged over 200 ms. Any fault: wiper off, drum to NEUTRAL, except when wraps are on, the spring is drawn, or the drum itself faulted:
then the drum HOLDS (D27).

## 11. Physics summary (owner: physics; base/physics/FINDINGS.md)

About 13 J stored, about 7.5 m/s, about 4.5 m at 35 deg with the 0.5 N brake (4 to 5 m band); a second spring pair
gives about 8 to 9 m. Cocking peak about 3 N m at the wiper. Tow 50 N at about 2.3 N m. The tether keeps the dart
nose first. The ballistics table for the Jetson is base/out/ballistics.json (contract C3).

## 12. Changing this spec

Owners move parts inside their bands freely and report the final numbers in their NOTES. Anything that moves an axis,
an interface or a zone edge goes to the coordinator first (REPORT.md or a message); the coordinator updates params.py,
this file and DECISIONS.md and tells everyone.

## 13. Decisions log

- See base/spec/DECISIONS.md (D1 to D19 and later). 3 Nov 10:45: baseline concept above (agreed plan + coordinator calls on the drum mechanism, sprag #2 as backstop,
  swing-arm aim splitter, main shaft = rail pivot). Harry was away and answered no questions; assumptions below.

## 14. Assumptions Harry must confirm (goes into the final MEASURE_LIST.md)

- Wiper motor: shaft diameter, flats or spline, end thread, mount hole pattern, housing size, torque and speed.
- Springs: rate and initial tension (hang a 1 L bottle on one spring, measure length before and after).
- Hardware box: M8 bolts or 8 mm rod for 608 axles (printed PPA-CF axles otherwise), a spare MP1584 for the drum N20,
  the 100 kg load cell + HX711.
- Rover: deck size and mounting holes; whether the Jetson should drive it.
- Dart build: v30 assumed.
