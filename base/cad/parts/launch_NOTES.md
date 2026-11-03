# Launcher + aim train: notes (worker cad-launch)

Current state for BASE_SPEC v1.1 (D23: sleeve y 111..193, two-layer big gear, sleeve outboard bushing in the aim
housing). Every number below comes from the code: `launch_kin.py` (sizing), `launch.py` (geometry, kin, contacts),
`launch_aim.py` (aim train), `launch_line.py` (tether on the rail). Run `python cad/parts/launch_kin.py` for the
sizing summary.

## 1. How it works (Harry's v10, sized to KEDGE v30)

- The rail pivots on the main axis. The LAUNCH SLEEVE (coaxial on the main shaft's d12 spigot, driven by cad-drum's
  LAUNCH dog ring at its flange, y 111) carries sprag #1 inside the 26T drive gear.
- The BIG GEAR rides on the rail on its own pin (82.6 from the axis). Two layers on one hub:
  - sector layer (y 134..146, the rack plane): 27 of 40 teeth, Harry's release teeth (last one cut to the pitch line,
    the one before 30 percent taller);
  - full 40T layer (y 146..154) that the drive gear meshes all the way round. (One sector layer would leave the
    drive in the toothless gap for 117 deg of every turn: found in review round 1.)
- One big gear turn (553.8 forward wiper deg) = cock, release, reset to rest. Backward wiper turns freewheel in
  sprag #1; with the carriage on its stop the gear cannot follow, so aiming never decocks.
- Firmware rule: aim only at HOME (spring slack). Aim, then fire in one forward run.
- The carriage is a long sled (the rack must be one stroke long). The dart rides on it in a light V cradle (5 rings
  and 2 strips), tail on the pusher; the tether leaves through the pusher hole on the axis.
- Aim: v10 crank-rocker scaled 0.6, rotated -90 deg about the pivot (pitch table identical to v10, 6.0 to 47.1 deg),
  driven only by BACKWARD wiper turns: swing-arm splitter (pawl-free) -> 2:1 bevel -> self-locking worm (lead
  4.8 deg) -> worm wheel 20T = crank. 10 wiper turns per aim cycle. The worm holds the pitch while cocking and at the
  shot.

## 2. Springs, stroke, energy (direct 1:1)

| item | value |
|---|---|
| springs | 2 x Century 16 x 165.1 x 1.4, k 0.125 N/mm, F0 7 N each (P.SPRING) |
| stroke (carriage travel at release, 2D tooth sweep) | 240.9 mm (release travel 240.2, peak 240.9) |
| preload at HOME | 38.8 mm (P.LAUNCH.spring_preload, = max_ext 280 - stroke); fallback hole set 8.8 for 250 |
| spring extension at full cock | 279.7 mm (limit 280) |
| force at HOME / full cock | 23.7 N / 83.9 N total |
| energy released | 12.7 J |
| spring hook pin distance | 203.9 at HOME, 444.8 cocked |

1:1 is best (a pulley trades force for stroke and multiplies the moving spring mass by the ratio squared).

## 3. Gears and timing

| gear | value |
|---|---|
| rack / big gear | m 2.5; big gear 40T (pitch d 100): sector layer 27 teeth present (gap 13 teeth, 117 deg) + full layer 40T |
| drive gear | 26T m2.5, face 8 (full-layer plane), hub holds sprag #1 on an integral outer key rib |
| centre distance | 82.6 |
| wiper turns per shot | 1.538 (553.8 deg) |
| torque at full cock | big gear 4.1 N m, wiper / sprag #1 2.7 N m |

Tooth timing (`launch.release_table`, real 2D contact): at rest tooth 0 is 3 pitches (27 deg) before the mesh centre
so it is the first tooth to touch rack tooth 0 while the carriage sits on its stop (a turning gear cannot enter a
stationary rack at the mesh centre). Release at big gear -281.25 deg = 432.7 forward wiper deg from rest. The gear then
runs on through its gap to rest (-360). Audited with the gear lagging up to 12 deg behind during the shot.

## 4. Geometry (rail frame: origin on the main axis at y 140, u along the rail to the muzzle, w up, y = world y - 140)

World: x = u cos p - w sin p, z = u sin p + w cos p (p = pitch); world origin on the main axis (LAYOUT v1.0).

| item | value |
|---|---|
| rack pitch line / dart axis | w 50.1 / w 85 (dart rests 0.4 low on the cradle) |
| carriage rear face at HOME / cocked | u 70.6 / u -170.3 (pusher face, dart tail at HOME: u 78.6) |
| shoe track (rail walls y +-22..28) | u -178.3 to 190 |
| spring anchors (6 mm steel pins, double shear in post + strap) | u 276.7, y +-38, spring axis w 60 |
| rail | u -178.3 to 286.7, 3 printed pieces split at u -30 and 140 (2 x M3 x 16 per flange pair) |
| tether pulleys (693ZZ, printed groove r 6) | top (u -200.9, w 78.1), bottom (u -200.9, w 30), pivot (u 0, y -21, w -24) |
| sleeve (world y) | flange 111..119 (dog ring, 3 x M3 r 28, lug slots), cheek journals 123..131 and 159.5..167.5, sprag rib 142.5..157.5, sun rib 174..182, end 193 in the aim housing bushing |
| aim crank centre | world (58.0, about 202, -100.5) |

Swept envelope: rail rear dips to about z -150 at 47 deg (fork bottom about 22 above the deck); muzzle about +260 up.

## 5. Contacts (printed: M3 screw head on an M3 nut, internal pull-up; no wire rides on the carriage)

| name | where | closes when |
|---|---|---|
| SW_HOME | flex lever on the RIGHT rail lip at u 40..69, y -25 | a rigid 20 mm foot under the right spring arm sits on its screw: carriage within about +-10 mm of the stop. Overshoot into the stop just slides the foot (never a load path) |
| SW_SEATED | flex lever on the LEFT rail lip, mirrored | the dart tail pushes a pushrod in the pusher (y 15, w 86) back 1.5 mm; its wedge pushes a plunger whose foot presses the lever screw. Only meaningful at HOME |
| SW_GAP | generic contact on the right cheek ledge (u 61.6, w -21) | a 2 mm cam bump on the big gear web passes its tip: gear within about +-5 deg of rest (in its gap) |
| SW_RAIL_LOW | on the aim housing, cam on the crank | crank within the window around the LOW dead point (see launch_aim) |

Firmware: shot reference = SW_GAP closing edge at the end of the forward run (or AS5600 angle); SW_HOME confirms the
carriage really came home; SW_RAIL_LOW zeroes the aim crank on the backward run.

## 6. Carriage stop

The spring arms (w 72..80) hit stop units on brackets over the springs at u 78.6. Harry has no TPU (D26): a printed
PLA flexure stop (about 19 mm travel, peak under 300 N, 2.8 to 3.3 J) is being designed (`launch_stop.py`). TPU pads
remain an optional upgrade.

## 7. Aim table for fw-base

P.AIM.table holds the full cycle (backward wiper deg from the LOW dead point vs pitch). Rising stroke 0..1596 wiper deg
(6.0 to 47.1 deg), falling stroke 1596..3600 (back to 6.0, controlled by the worm, no drop).

## 8. Interfaces

- cad-drum: dog ring keys to the sleeve flange at y 111 (3 x M3 at r 28, nut traps outboard, 3 lug slots 6.4 x 3.2);
  its joint follows launch_sleeve. Spigot d12 runs through the 12.35 sleeve bore to the magnet cap at y 195..203.
- cad-frame: the aim housing stands on the deck (4 x M3, footprint in launch_aim); AS5600 bracket on the housing
  outer side at about y 205. Keep out of the rail sweep (LAYOUT keep_out).
- cad-reel: the tether arrives at the rear of the pivot pulley, `launch.line_entry(P, S)` (world); `launch.line_path(P, S)`
  continues it over the rail pulleys to the pusher and the dart tail.
- Timeline keys used by kin: aim_deg, cock, fired, dart_flight, dart_return, phase ("lower" runs the crank over the
  top and down the falling stroke).
