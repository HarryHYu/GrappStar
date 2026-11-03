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

## 2. Current numbers (generated: `PYTHONPATH=. python cad/parts/launch_numbers.py`)

| item | value |
|---|---|
| params version | 1.2.3 |
| springs | 2 x Century C-243 (5/8 x 6-1/2 in x 0.054 in wire), k 0.125 N/mm, F0 7.0 N each, max_ext 265.0 mm |
| stroke (travel at release / peak) | 240.2 / 240.9 mm (design value 240.9) |
| preload at HOME / extension at full cock | 23.8 / 264.7 mm |
| force at HOME / full cock (both springs) | 19.9 / 80.2 N |
| energy released | 12.06 J |
| hook pin distance HOME / cocked | 188.9 / 429.8 mm |
| big gear | m2.5 40T: sector layer 27 teeth (gap 117 deg) + full 40T layer |
| drive gear | 26T, centre distance 82.6 |
| wiper deg per shot / release at | 553.8 / 432.7 forward wiper deg from rest |
| torque at full cock: big gear / wiper | 4.01 / 2.61 N m |
| moving mass (carriage body + arms + seat parts) | 101.6 g (dart 279 g) |
| carriage rear face HOME / cocked (u) | 70.6 / -170.3 |
| rail span (u) / splits | -178.3 .. 273.5 / (-30.0, 140.0) |
| spring anchors (u, y, w) | 263.5, +-38 (provision +-56), 60 |
| tether pulleys (rail frame) | top (-200.9, 0.0, 78.1), bottom (-200.9, 0.0, 30.0), pivot (0.0, -21.0, -44.0) |
| aim | pitch 6.01 .. 47.06 deg, 10 wiper deg per crank deg, worm lead 4.76 deg |

1:1 springs (a pulley trades force for stroke and multiplies the moving spring mass by the ratio squared). Preload =
SPRING.max_ext - stroke (spec D30: max_ext 265, so 23.8 mm). The stroke is fixed by the 27-tooth sector.

## 3. Tooth timing

`launch.release_table` (real 2D tooth contact): at rest tooth 0 is 3 pitches (27 deg) before the mesh centre so it is
the first tooth to touch rack tooth 0 while the carriage sits on its stop (a turning gear cannot enter a stationary rack
at the mesh centre). Release at big gear -281.25 deg. The gear then runs on through its 117 deg gap to rest (-360).
Audited with the gear lagging up to 12 deg behind during the shot; carriage rebound positions off the stop are inside
those shot states.

## 4. Geometry (rail frame: origin on the main axis at y 140, u along the rail to the muzzle, w up, y = world y - 140)

World: x = u cos p - w sin p, z = u sin p + w cos p (p = pitch); world origin on the main axis (LAYOUT v1.0).

| item | value |
|---|---|
| rack pitch line / dart axis | w 50.1 / w 85 (dart rests 0.4 low on the cradle) |
| shoe track (rail walls y +-22..28) | u -178.3 to 190 |
| spring anchors | 6 mm steel pins in post + strap (double shear); second-pair holes at y +-56 (D5) |
| rail | 3 printed pieces split at u -30 and 140 (2 x M3 x 16 per flange pair) |
| tether pulleys | 693ZZ in printed pulleys (groove r 6); positions in section 2 |
| carriage | body (|y| <= 21.5, prints on its side) + 2 bolted spring arms (half lap, 2 x M3 each) |
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

## 6. Carriage stop (`launch_stop.py`, D26: no TPU)

Printed PLA flexure per side: 16 closed loops (31 tapered leaves, 2.93 to 1.2 mm thick, 14 tall) lying over the main
spring, in a bolted frame, preloaded 4.4 mm (54 N, more than the 20 N spring pull, so the carriage rests exactly at
u_stop). Both sides: 12.4 N/mm, 3.26 J at 19 mm, peak 290 N, leaf stress 34 MPa. Hard stop on the rail at 19 mm.
PLA returns most of the energy: the carriage rebounds up to about 100 mm before the springs and friction settle it; the
gear is in its gap then (audited). Firmware: wait 0.5 s after the release before reading SW_HOME. TPU pads would damp
better if Harry ever gets TPU.

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
