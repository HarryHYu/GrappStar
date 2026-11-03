# Fit coupon

Print this set FIRST, before any real part. Each tile tells you one number in `base/cad/lib/fits.py`.
Print everything in PLA with your normal profile (0.4 nozzle, 0.2 layers, 4 walls), flat as exported, no
supports. Let the parts cool fully (10 minutes) before testing. The first character of every label is the
tile letter below; the rest is the value that tile was made with.

How to change a fit: open `base/cad/lib/fits.py`, change the one number named in the step, save, then rerun
`../kedge/.venv/bin/python -m tools.risk_coupon` from `base/` (on the Mac after `bash kedge/setup_mac.sh`;
it rewrites this fit coupon in base/pack/fit_coupon and the risk coupons in base/pack/coupons), then the
audit of your subsystem. Change one number at a time.
If none of the variants is right, pick the closest one, move the number one more step in that direction
and print only that tile again.

## Parts

| part | qty | print bbox mm | what |
|---|---|---|---|
| coupon_a1_608_rib | 1 | 34.0 x 40.0 x 9.6 | 608 crush-rib seat, rib interference 0.10 mm radial |
| coupon_a2_608_rib | 1 | 34.0 x 40.0 x 9.6 | 608 crush-rib seat, rib interference 0.15 mm radial |
| coupon_a3_608_rib | 1 | 34.0 x 40.0 x 9.6 | 608 crush-rib seat, rib interference 0.20 mm radial |
| coupon_b1_key_bore | 1 | 36.0 x 40.0 x 8.6 | 20 mm keyed bore, clearance 0.20 mm diametral |
| coupon_b2_key_bore | 1 | 36.0 x 40.0 x 8.6 | 20 mm keyed bore, clearance 0.30 mm diametral |
| coupon_b3_key_bore | 1 | 36.0 x 40.0 x 8.6 | 20 mm keyed bore, clearance 0.40 mm diametral |
| coupon_b_key_shaft | 1 | 22.6 x 19.9 x 16.0 | Keyed shaft stub d 20, 16 long, prints upright |
| coupon_c_m3_insert | 1 | 50.0 x 22.0 x 8.6 | M3 heat-set insert holes 3.8 / 4.0 / 4.2 |
| coupon_d_m3_nut | 1 | 52.0 x 22.0 x 6.6 | M3 nut traps AF 5.6 / 5.8 / 6.0 |
| coupon_e_m3_holes | 1 | 58.0 x 20.0 x 6.6 | M3 tap and clearance holes 2.5 / 2.6 / 2.7 / 3.2 / 3.4 |
| coupon_f_pin_host | 1 | 36.0 x 36.0 x 4.6 | Host plate for the d 6 pins: snap row on top, clip row below |
| coupon_f_snap_pin_1 | 1 | 9.0 x 9.0 x 9.2 | Snap pin d 6, fit 0.25 (1 notch) |
| coupon_f_clip_pin_1 | 1 | 9.0 x 9.0 x 8.4 | Clip pin d 6, fit 0.25 (1 notch) |
| coupon_f_snap_pin_2 | 1 | 9.0 x 9.0 x 9.2 | Snap pin d 6, fit 0.35 (2 notches) |
| coupon_f_clip_pin_2 | 1 | 9.0 x 9.0 x 8.4 | Clip pin d 6, fit 0.35 (2 notches) |
| coupon_f_c_clip | 2 | 8.4 x 7.8 x 1.5 | C clip for the d 6 clip pin |
| coupon_g_eye | 1 | 44.0 x 22.0 x 5.6 | Dyneema eye (v7) in 3 mm and 5 mm plates |
| coupon_h_gear_plate | 1 | 50.1 x 30.0 x 9.0 | Gear plate: pins d 5 at centre distance 27.10 |
| coupon_h_gear_z12 | 1 | 21.0 x 21.0 x 5.0 | Spur gear m1.5 z12, backlash 0.15 |
| coupon_h_gear_z24 | 1 | 39.0 x 39.0 x 5.0 | Spur gear m1.5 z24, backlash 0.15 |

## A. 608 bearing seat (labels A .10 / .15 / .20: rib interference, mm)

You need: three 608 bearings (or one, pressed in and out).
1. Press a 608 into each tile with your thumb or a vice. Use a flat block, keep it square.
2. Good: it goes in with a firm push, sits flat on the lip, and does not fall out when you turn the tile
   over and tap it. The outer race must not be pinched: spin the bearing, it should spin as freely as
   before.
3. Too tight (needs a hammer, tile cracks, bearing feels rough): pick a smaller value.
   Too loose (falls out, wobbles): pick a bigger value.
4. In fits.py set `BRG_RIB_INTERF` to the value printed on the best tile (now 0.15).

## B. 20 mm keyed bore (labels B .20 / .30 / .40: bore clearance, mm diametral)

1. Push the printed keyed shaft stub (coupon_b_key_shaft) into each tile, key in the keyway.
2. Good for a running fit: it slides in by hand with no force, turns by the key only, and has no visible
   wobble when you rock it.
3. Too tight: needs force or files: bigger value. Too loose: rocks or clicks: smaller value.
4. In fits.py set `SHAFT_IN_BORE` to the best value (now 0.3).
5. If the key binds but the round part is fine, raise `KEY_CLEAR` by 0.05 (now 0.2); if the
   key rattles while the bore is good, lower it by 0.05.

## C. M3 heat-set inserts (labels 3.8 / 4.0 / 4.2: hole diameter)

You need: three M3x4x5 heat-set inserts, soldering iron with an insert tip.
1. Set the iron to about 220 C. Press an insert into each hole, straight, until flush.
2. Good: it goes in with light pressure, a small ring of plastic rises round it, and an M3 screw tightened
   firmly does not spin the insert or pull it out.
3. Spins or pulls out: smaller hole. Needs a lot of force, or plastic bulges a lot: bigger hole.
4. In fits.py set `M3_INSERT_D` to the best hole (now 4.0).

## D. M3 nut traps (labels 5.6 / 5.8 / 6.0: across flats)

1. Drop an M3 nut into each hex pocket.
2. Good: it presses in with a finger, stays put when you turn the tile over, and does not turn when you
   drive a screw in from below.
3. Will not go in: bigger. Falls out: smaller (a nut that just drops in is fine if a screw holds it).
4. In fits.py set `M3_NUT_AF` to the best value (now 5.80).

## E. M3 holes (labels 2.5 / 2.6 / 2.7 / 3.2 / 3.4: hole diameter)

1. Drive an M3 screw into each of 2.5, 2.6, 2.7 (self tapping into plastic) and push one through 3.2
   and 3.4.
2. Tap hole good: the screw cuts its thread with moderate force and holds when you pull hard, the plastic
   does not split. Set `M3_TAP_D` to the best one (now 2.6).
3. Close hole good: the screw passes with a little drag, no play.
   Set `M3_CLOSE_D` to it (now 3.2).
4. Clearance hole good: the screw drops through freely.
   Set `M3_CLEAR_D` to it (now 3.4). If 3.2 is already loose, it can come down too.
5. These holes print vertical. Holes printed lying down come out smaller: that extra is
   `HOLE_HORIZ_EXTRA` (now 0.15), leave it unless a horizontal hole is tight.

## F. d 6 snap pin and clip pin (1 notch = fit .25, 2 notches = fit .35)

The fit is taken off the pin shank: shank d = 6 minus the fit.
The host plate has 6 mm holes: top row for the snap pins, bottom row for the clip pins (the labels 1 and 2
only say which pin to try there). Print two C clips.
1. Push each snap pin through a top hole until the barbs click past the far face.
2. Put each clip pin through a bottom hole and push a C clip onto the groove from the side.
3. Good: the pin turns freely in the hole with very little side play, the snap pin needs a firm push and
   does not fall out, the clip snaps on with a click and stays.
4. If the 1 notch pins (fit .25) are best for sliding, keep `HOLE_SLIDE` = 0.25. If they bind and
   the 2 notch pins (fit .35) are right,
   set `HOLE_SLIDE` to about .30 to .35.
   If the 2 notch pins rattle, `HOLE_CLEAR` (now 0.35) can come down by 0.05 to 0.1.
5. If a prong snaps off or the clip cracks, tell the CAD side (clips.py proportions), not fits.py.

## G. Dyneema eye (labels G 3 and 5: plate thickness, mm)

1. Thread 1 mm Dyneema through both eyes and saw it back and forth under load for a minute.
2. Good: the line runs smoothly, no fuzz on the line, no sharp edge you can feel with a fingernail at
   either face.
3. If the eye is rough, sand the cones; if the throat is too tight to thread, raise `line.EYE_THROAT_D`
   (now 4.4); this one is in line.py, not fits.py.

## H. Gear mesh m1.5 z12 / z24 (label H 1.5), centre distance 27.10 mm

1. Drop both gears onto the pins of the gear plate and turn the big gear by hand through several turns.
2. Good: smooth, no tight spot, a tiny bit of play you can just feel when you rock one gear while holding
   the other.
3. Tight or notchy: raise `GEAR_CENTER_EXTRA` (now 0.1) by 0.05 or `GEAR_BACKLASH`
   (now 0.15) by 0.05. Lots of play (clacks when reversing): lower one of them by 0.05.
4. If a gear is tight on its pin but the mesh is fine, raise `SHAFT_IN_BORE` (now 0.3);
   check tile B first, it tests the same number.

## Record

Write the numbers you chose and the date at the top of fits.py as a comment, for example
`# coupon 2026-11-10: BRG_RIB_INTERF 0.15, SHAFT_IN_BORE 0.30, ...`, then commit.
