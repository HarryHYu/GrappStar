# Launcher + aim train: notes for the spec (worker cad-launch)

Status: Phase 0 (before BASE_SPEC). Numbers come from `launch_kin.py` (run it: `python launch_kin.py`). All are
proposals for `params.py` LAUNCH / AIM; the code reads them from P when the spec lands.

## 1. Concept (Harry's v10, sized to KEDGE v30)

- Rail pivots on the MAIN AXIS (coaxial with the wiper output, as in v10).
- Drive gear on the main axis, sprag #1 (CSK20PP) inside it: forward wiper cocks and fires, backward freewheels.
- Big gear rides ON the rail on its own pin, meshes the drive gear and the rack under the carriage. Teeth present over
  part of the circle; last tooth relieved to the pitch line, the one before 30 percent taller (v10 release).
  One big gear turn = cock + fire + reset. Rest ("HOME") = carriage on the front stop with tooth 1 just engaged, so a
  backward wiper turn (aim) pushes the carriage into its stop and sprag #1 slips: the spring is never decocked by aiming.
- Order in a shot: AIM first (backward turns, carriage at HOME, spring slack), then FIRE (forward turns: cock, release,
  reset to HOME in one run). Aiming with the spring cocked is not allowed (backward turns would let the spring unwind
  through sprag #1 under control; firmware rule: `aim` only in HOME).
- Carriage = long sled (its rack has to be one stroke long anyway), so the dart rides in a cradle ON the carriage and
  moves with it while cocking. Cradle is a half pipe under the belly, top and flanks open for the legs (KEDGE v30 has
  nothing below the hull midline outside the d42 cylinder: checked on the folded mesh, `launch_dart_v30.npz`).
- Aim: v10 crank-rocker scaled by 0.6 (pitch table identical to v10: 6.0 to 47.1 deg), rotated -90 deg about the pivot
  so the crank sits low and forward. Driven only by BACKWARD wiper turns: swing-arm direction splitter (pawl-free: an
  idler on a friction-dragged arm swings into mesh one way, out the other) -> 2:1 bevel -> self-locking worm (single
  start, m2, pd 24, lead angle 4.8 deg) -> worm wheel 20T = crank gear. The worm holds the pitch while cocking and at
  the shot. No third sprag.
- Conflict to note: v10 handover says "no worm gear, no printed pawls". The coordinator asked for a printed
  self-locking worm in the aim train (system plan section 3), so I follow that; it is pawl-free.

## 2. Springs, stroke, energy (direct 1:1, no pulley)

| item | value |
|---|---|
| springs | 2 x Century 16 x 165.1 x 1.4, k 0.125 N/mm, F0 7 N each (P.SPRING) |
| preload at HOME | 5 mm extension (15.3 N total holds the carriage on the stop) |
| full cock | extension = SPRING.max_ext = 250 mm |
| stroke | 245 mm |
| peak force | 76.5 N total (38.3 N each) |
| stored energy released | 11.2 J |
| spring length | 170 at HOME, 415 cocked (hook to hook) |

Why 1:1: energy is capped by the spring extension (11.2 J at 250 mm) whatever the ratio. A pulley only trades force
for stroke, and the moving spring mass grows with the ratio squared (each spring about 56 g, about 19 g effective at the
hook). 2:1 doubles the rack and carriage length (heavier carriage); 1:2 multiplies spring effective mass by 4. 1:1 gives
the most dart energy. Estimated moving mass: dart 279 g + carriage about 80 g + springs about 37 g effective, so about
70 percent of the energy goes to the dart before losses: roughly 7 m/s. Physics worker: please confirm with your model and
the real carriage mass from my CAD (I will put it in `launch_REPORT.md`). If the hang test allows more than 250 mm, only
`SPRING.max_ext` changes and everything follows.

## 3. Gears (all PPA-CF, face 12)

| gear | value |
|---|---|
| rack / big gear module | 2.5 |
| big gear | 40T full (pitch d 100), 33 teeth present, gap 7 teeth (63 deg) |
| drive gear (main axis) | 26T (pitch d 65, root d 58.75: holds the 47 OD sprag with a 5.9 wall), keyway 6 |
| centre distance | 82.6 (big gear pin is 82.6 from the main axis, on the rail) |
| wiper turns per shot | 40/26 = 1.538 (553.8 wiper deg) |
| torque: big gear / wiper and sprag #1 | 3.8 / 2.5 N m at full cock (sprag rated 50) |
| shot time | about 2.1 s at 45 rpm free speed, plus the stop |

During the shot (about 60 ms) the wiper keeps turning about 16 deg; the 63 deg gap covers it (release check is done on the
real 2D tooth geometry in the CAD, see REPORT).

## 4. Rail frame and envelope

Rail frame: origin on the main axis, u along the rail to the muzzle, w up, y left (same y as world). World:
x = u cos p - w sin p, z = u sin p + w cos p, p = pitch.

| item | rail coords (u, w) / y |
|---|---|
| main axis (pivot) | (0, 0) |
| big gear pin | (82.6, 0), y 0 plane |
| rack pitch line | w = 50 |
| dart axis | w = 85 (belly w 64), dart nose forward |
| carriage pusher face at HOME / cocked | u 75.6 / u -169.4 |
| dart tail at HOME, nose at HOME | u 75.6, u 227.6 (augers reach back to u 59 at |y| about 31, w above 86) |
| rail bars (carriage shoes) | u -185 to 115 |
| spring anchors (muzzle bar) | u about 240, y +-34, w about 60 |
| springs | y +-34, w about 60, under the dart flanks (the flank legs are all above w 85) |
| rail length | u -190 to 250 (about 440 mm, printed in 2 pieces, bolted M3) |
| aim lug on the rail | (115.2, 0) at y about +55 (outboard of the rail) |
| crank (worm wheel) centre | world (58.0, -100.5) relative to the pivot, y about +55 |
| crank radius / conrod | 30 / 156 |

Swept envelope: the rail rear end dips to about 150 below the main axis at 47 deg; the muzzle rises to about 260
above it. So the MAIN AXIS must be at least about 160 above the deck (or the deck cut away behind the axis). Front of the
envelope about 260 forward of the axis at 6 deg; rear about 190 behind. Width y -40 to +75 (aim stack on +y).

## 5. Interfaces I need from the spec

1. MAIN AXIS position and axis direction (LAYOUT["main"]): along world y.
2. Launch input: preferred = coaxial. The dog clutch's launch-side driven member continues as a "launch sleeve",
   20 mm OD with a 6 mm key for 14 mm (sprag #1 inner race) plus a keyed seat for the splitter sun gear (about 12 mm) and
   two plain journal sections (d 30, 10 long each) for the rail cheek bushings. Axial span needed on the main axis:
   y -20 to +40 (drive gear at y -8..+8, rail cheeks y +-9..+19, sun gear y +24..+36). Who owns the sleeve: I propose I
   do (`launch_sleeve`, PPA-CF), with the drum worker's dog face on its end, dog geometry from the drum worker. If the
   spec instead puts the launch drive on a gear (drum worker's "launch output gear"), I need its module, tooth count,
   face width and position: then my drive gear sits on its own launch shaft, and the rail still pivots on the main axis.
3. Rail rest: frame stop / pivot support. The rail cheeks ride on the launch sleeve journals (printed bushing, slow
   relative rotation). Frame supports the sleeve on both sides of the rail (bearing seats by the drum/frame worker).
4. Aim pedestal on the frame at world (58, -100.5) from the main axis, plane y about +55: worm wheel axle (8 mm, two 608
   bearings) and the worm shaft (vertical, two 608s). Splitter countershaft parallel to the main axis.
5. Frame mounting: the rail is carried only by the main axis and the conrod. Frame worker: a 608-pair pedestal block
   (I can supply its bolt pattern: 4 x M3 into heat-set inserts or self-tap).
6. Physics (w/physics): spring numbers above, carriage mass (to come), pitch 6 to 47 deg (tune AIM_K/AIM_ROT/rod if the
   range table wants another range).

## 6. Switches (printed contacts: M3 screw head on an M3 nut, internal pull-up)

| name | where (rail coords) | closes when |
|---|---|---|
| SW_SEATED | under the cradle at the pusher, u = pusher face + 3 | dart tail within 3 mm of the pusher (flex tongue pressed by the belly) |
| SW_HOME | rail, at the carriage front stop | carriage on the front stop (HOME) |
| SW_LOW | frame, under the rail near the rear | rail pitch below 7 deg (at the LOW dead point, 6.0) |

Positions get exact numbers in the CAD (REPORT). Firmware uses SW_HOME as the shot reference (AS5600 angle at the
closing edge = HOME), and SW_LOW to zero the aim crank (closing edge on the backward run near the low dead point).

## 7. Aim table for fw-base (backward wiper degrees from the LOW dead point; 10 wiper turns per full aim cycle)

Rising stroke: 0 to 1596 wiper deg (6.0 to 47.1 deg). Then the rail falls back to 6.0 over the next 2004 wiper deg
(the worm controls the fall: no drop). Forward wiper turns never move the aim (splitter disengaged, worm holds).
| crank deg from LOW | backward wiper deg | pitch deg |
|---|---|---|
| 0 | 0 | 6.01 |
| 10 | 100 | 6.24 |
| 20 | 200 | 6.93 |
| 30 | 300 | 8.12 |
| 40 | 400 | 9.83 |
| 50 | 500 | 12.10 |
| 60 | 600 | 14.93 |
| 70 | 700 | 18.32 |
| 80 | 800 | 22.22 |
| 90 | 900 | 26.50 |
| 100 | 1000 | 30.98 |
| 110 | 1100 | 35.39 |
| 120 | 1200 | 39.42 |
| 130 | 1300 | 42.76 |
| 140 | 1400 | 45.19 |
| 150 | 1500 | 46.62 |
| 160 | 1600 | 47.06 |
| 170 | 1700 | 46.59 |
| 180 | 1800 | 45.37 |
| 190 | 1900 | 43.52 |
| 200 | 2000 | 41.20 |
| 210 | 2100 | 38.54 |
| 220 | 2200 | 35.64 |
| 230 | 2300 | 32.59 |
| 240 | 2400 | 29.49 |
| 250 | 2500 | 26.39 |
| 260 | 2600 | 23.36 |
| 270 | 2700 | 20.46 |
| 280 | 2800 | 17.72 |
| 290 | 2900 | 15.19 |
| 300 | 3000 | 12.91 |
| 310 | 3100 | 10.91 |
| 320 | 3200 | 9.21 |
| 330 | 3300 | 7.85 |
| 340 | 3400 | 6.85 |
| 350 | 3500 | 6.23 |
| 360 | 3600 | 6.01 |
