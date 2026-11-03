# sensors: design notes (worker cad-sensors, 3 Nov 2026)

Module `sensors.py` owns the AS5600 mount, SW_RAIL_LOW and SW_GAP. It has no joints of its own: AS5600 and
SW_RAIL_LOW parts are world fixed (the aim housing is world fixed), SW_GAP parts ride on cad-launch's `launch_rail`.

## Why this module exists

The builder dry run (review r2 J items 1, 5, 7) found no CAD for these. By base-v1 1d668d6 cad-launch had added
them, but:

- their leaf switch (`launch_switch.py`, leaf 1.2 thick, screw 5 mm above the root, travel 1.2) bends to about
  3 t d / (2 L^2) = 8.6 percent strain, ten times the 0.8 percent PLA limit from coupon 2. It would crack or set.
- the AS5600 board was held by 2 x M2.5 self tap screws (Harry has no M2.5) at two assumed mid holes; the real board
  has 4 corner holes.

So both switches are now `cad.lib.switch.contact_switch(30)` (the coupon 2 geometry: lever t 1.4, gap 0.93 mm,
strain 0.78 percent) and the AS5600 has a printed carrier. `sensors_launch_patch.diff` is the exact change cad-launch
applies to delete their old parts and cut the pilots below.

## 1. AS5600 (Adafruit PID 6357)

Board facts from Adafruit's EAGLE file (github adafruit/Adafruit-AS5600-Magnetic-Angle-Sensor-PCB): 25.4 x 17.78 x 1.6,
plated 2.5 mm holes at (2.54, 2.54), (22.86, 2.54), (2.54, 15.24), (22.86, 15.24), chip (SOIC-8) at the board centre,
JST SH 4 pin connectors at x 3.048 and 22.352 opening to the short edges, 6 pin header row (unpopulated) at y 2.54.

Stack along world y (board long side along x, chip facing the magnet, chip centre on the axis):

| y (mm) | what |
|---|---|
| 201.5 | magnet face (drum_magnet_cap end face, cap r 10) |
| 202.05 | JST connector faces (2.95 tall): 0.55 clear of the turning cap |
| 203.5 | chip face (SOIC 1.5 tall; 1.75 max) |
| 200.2 .. 203.2 | strip screw heads (at z +-14, radius 11.25: 1.25 outside the cap) |
| 203.2 .. 205.0 | clamp strip bands on the board edges (thick part to 206.3, 0.3 short of the plate) |
| 205.0 .. 206.6 | board |
| 206.6 .. 214.6 | carrier plate with 4 pegs d2.3 through the corner holes, 2 hex nut pockets in its back |

Chip to magnet: 2.0 mm nominal, 1.75 mm with the tallest SOIC. ASSUMPTION: the target was 1 to 2 mm, design 1.0. 1.0 is
not possible with this board: the JST connectors sit at radius 7.5 to 11.8, inside the cap radius 10, and stand 2.95
tall. To get 1.0, cad-drum would step the cap end face down to r 7 for the last 1.5 mm; then move `JST_CAP_CLR` and the
board 1 mm in. The AS5600 datasheet allows 0.5 to 3 mm with a 6 x 2.5 diametric magnet, so 2.0 is fine.

Fixing: no M2.5 and no heat staking needed. The 4 pegs locate the board (chip on the axis within the peg clearance,
0.1 mm); two printed strips clamp the long edges (they cover only 3.2 mm in from each edge, clear of every component).
Each strip is pulled by an M3x12 from its front face into an M3 nut in a hex pocket in the carrier back; the strip's
thick part stops 0.3 mm short of the plate so the band really clamps the board. The carrier column (x -16..-6) runs
down beside the aim housing spine and is screwed to its +x face with 2 x M3x16 (pilots d2.6 in the spine at y 209.1,
z -33 and -44, cut by cad-launch); below the plate the column is 10 mm thick round those holes. Cable: the +x
connector (the -x one is free too). Fit the board after the drum magnet cap. Print the carrier on its back (pegs up),
the strips on their front face.

## 2. SW_RAIL_LOW (GPIO 40)

Read by cad-launch's crank cam lobe (`launch_aim.swl_cam_joint`): dwell radius 41.5 at psi 0 (world +x from the
crank centre, x 58.04, z -100.54), +-4 deg dwell, ramps to r 39.5, plane y 193.8..199.4. The lever runs along world -y
(root outboard, tip in the cam plane), actuated towards +x. Its rest actuator face sits at r 41.5 - (pretravel + 0.3),
so at the dwell the cam pushes 0.3 mm past first contact (overtravel to the hard stop is 0.6). The jam nut and screw end
sit at y 200.7..207, outside the crank plane.

Bracket (PLA): backing plate behind the switch base (world x 114.5..119.5), a column down to cad-launch's housing foot
extension (x 80..130.9, y 193.8..201.8, top at deck + 8), a foot on it and a web (about 48 deg) under the cantilevered
plate. Fixing: 1 x M3x10 down into the pilot at (107, 197.8), and the housing's 4th deck screw (x 125.9; deck screws go
in from above into the deck's heat-set inserts) becomes an M3x20 through the bracket foot (6), the foot extension (8)
and into the insert. The column stops 3.4 mm short of that screw so the head and driver have room. cad-launch also cut
a pilot at x 116: it lies under the bracket column and is not used. The switch base screws to the plate with
4 x M3x8 self tap. Print foot down.

## 3. SW_GAP (GPIO 39)

Read by the big gear cam bump (launch.big_gear_solid: r 29.7 at 225 deg in the rail frame when the gear rests in its
toothless gap, 2 mm tall, +-5 deg flat top, 10 deg ramps, radial width 26.7..32.7) on the sector web's right face
(rail y_rel -6). The lever tip touches the bump 1 mm inside its centre radius (r 28.7); the lever runs from there to
its root at 213.75 deg (rail u-w plane, 11 deg off the radial): within about 29 deg of the radial the contact screw
and jam nut stay outside the circle the bump sweeps (tested over a full gear turn). Actuated towards -y. The jam nut
would stand into the gear web, so the lever tip carries a 4 mm printed riser (part of the lever, beyond the jam nut,
rigid): the actuator face is the riser top, at rest 0.46 mm off the web face; the screw end stays 0.5 mm under it.

One printed part: the cad.lib switch base with a 10 mm lever clamp pad (library default 14; the flexing beam, so the
coupon 2 strain and gap, is unchanged), its low +y side trimmed 1.7 mm past the wire channel (mounting wings not
needed), fused with an arm: a web in the base plane up to a tab on the right cheek's outer face (rail y_rel -17),
2 x M3x12 along +y into cheek pilots d2.6 at (u 48, w 2) and (u 64, w 2), heads in counterbores. No backing plate:
cad-drum's dog post sits at world y < 111.5 and the base bottom is at world y 113.0.

Neighbours (min gap over pitch 6 to 47, merged tips launch 763df11, drum 72a3a6e, reel 5df615a, frame 0862250):
reel_p2_post (pulley 2 arm) 3.1 mm, launch_sleeve 2.0, drum_dog_post 1.5, launch_gear_pin 3.6, drum_dogring_launch
3.6. The coordinator asked for 5 mm from the p2 post: not reachable here, the sleeve flange closes in from the other
side (212.5 deg gives 3.8 / 1.1). The audit enforces Clearance minimums of 2.5 / 1.5 / 1.0 mm on these three.
If more room is wanted, cad-reel lowers the pulley 2 arm top by 2 mm.

Closure windows (lever push vs angle, closed when the push reaches the pretravel 1.24 mm): SW_GAP closed for the gear
within about +-15 deg of rest; SW_RAIL_LOW closed within about +-9 deg of crank from the LOW dead point (pitch 6.0 to
6.2), touching up to about +-13 deg.

## 4. Spring guard

Not in this module: cad-frame builds the spring guards (frame_guards.py, joint following launch_rail, in base-v1).
The sensors audit runs with frame loaded, so the switches are checked against the guards.

## Assumptions (Harry: check)

- The AS5600 board is used with the STEMMA QT cable only (no header pins soldered).
- Air gap 2.0 mm (see 1).
- Contact gap set by turning: 0.93 mm, 0.5 mm per turn (the step text says back off 1.9 turns from touching).
- The STEMMA cable plugs into the +x connector; a JST SH plug needs about 6 mm beyond the board edge (kept free).
