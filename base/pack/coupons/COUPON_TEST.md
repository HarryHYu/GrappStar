# Day-one risk coupons: test sheet

Print these before the big parts. Each coupon is made from the same geometry functions and numbers as the real part, so a pass here is a pass there. Let parts cool 10 minutes before testing. The fit coupon (tolerance tiles A to H, base/pack/fit_coupon/COUPON.md) is printed alongside.

## Settings

PLA: 0.4 mm nozzle, 0.2 mm layers, 4 walls, 40 % gyroid infill unless noted, 5 top and 5 bottom layers, hole and XY compensation 0, no supports. PPA-CF: hardened steel nozzle (never brass), filament dried (80 C, 8 h) and fed from a dry box, 0.2 mm layers, 6 walls, 60 % gyroid infill, 6 top and 6 bottom layers, hole and XY compensation 0, no supports. The 3MF files carry geometry only: set the material and these settings in Bambu Studio.

## If a coupon fails

To change a number: on the Mac, after bash kedge/setup_mac.sh, run kedge/.venv/bin/python -m tools.risk_coupon from base/ (it rewrites base/pack/coupons and base/pack/fit_coupon). Easier: give the failing result to a Claude session and let it change the numbers. Change one number at a time and reprint only that coupon.

## Plates

| plate | material | objects |
|---|---|---|
| risk_plate_1_PLA.3mf | PLA | 19 |
| risk_plate_2_PPA-CF.3mf | PPA-CF | 26 |

## Tests

### 1. Swing-arm direction splitter

- Parts: rc1_plate, rc1_sleeve, rc1_sun, rc1_arm, rc1_idler, rc1_output, rc1_crank, rc1_stop
- Bought: O-ring 20 x 1.5 (arm hub groove), e-clip for the d8 arm pin, 2 x M3x16
- Do: O-ring into the arm hub groove. On the plate: sleeve on the centre pin, sun on its rib, arm on the sleeve above the sun with its pin down through the idler, e-clip on the pin, output on its pin, crank on top, stop finger on the column (2 x M3x16). Turn the crank 10 turns each way.
- Pass: Clockwise seen from above: the arm swings the idler into the output at once and the output turns, no skipped teeth. Anticlockwise: the arm swings out to the finger within about 20 deg of crank, the idler runs free and the output stays still. No jam at engagement.
- If it fails, change: launch_aim.py: FIT (pin and bore play), arm_solid O-ring groove (drag), A() alpha_d = alpha_e + 20 (swing out), _stop_post_xy; launch_gears.py center_distance extra / spur backlash (mesh).
- Made from: launch_aim.sun_solid, arm_solid, idler_solid (m1.5 z30/18/30, face 8, FIT 0.35), launch_gears.spur for the output, launch_aim.A: centre distances 36.10 / 36.10, output at (11.7, 68.4), engaged arm 96.2 deg, stop 116.2 deg (launch_aim._stop_post_xy), sleeve rib launch.RIB_W 6 / RIB_R 12.8

### 2. Printed contact switch

- Parts: rc2_switch_base, rc2_switch_lever
- Bought: M3x10 SHCS ISO 4762 + M3 jam nut, M3 nut, 2x M3x10 self tapping (pan or SHCS) through the lever root into the pad, at most M3x11.7, 2 wires, multimeter
- Do: Nut into the pillar trap (drop of glue), lever clamped with 2 M3 self tappers, contact screw head down through the lever, set the head 0.9 mm above the nut (0.5 mm per turn), lock the jam nut. Meter on beep between screw and nut. Press the lever tip 100 times.
- Pass: Measure the tip travel with calipers (depth rod from the lever tip down to the base top, note the reading at rest, press until it beeps, read again): beep within 0.9 to 1.5 mm of tip travel (design 1.24), silent when let go, tip stops on the hard stop without cracking, lever back flat, same after 100 presses.
- If it fails, change: cad/lib/switch.py contact_switch args travel, overtravel, t, b; STRAIN_LIMIT, NUT_PRESS (loose nut), SCREW_LENGTHS.
- Made from: cad.lib.switch.contact_switch(lever_len=30) with the library defaults (travel 0.8, overtravel 0.6, beam t 1.4): gap 0.93, pretravel at the tip 1.24, close force 1.10 N, strain 0.0078

### 3. Dog clutch and compliant fork

- Parts: rc3_base, rc3_reel_face, rc3_collar, rc3_fork, rc3_dog_post, rc3_rail
- Bought: M3 nut (yoke tab), 2 x M3x12 (SW_DOG screws in the post), 1 x M3 (post clamp), multimeter
- Do: Press the rail into the base. REEL face on the journal (dogs up), collar on the hex (dogs down), fork pads in the collar groove and its sleeve on the rail, dog post clamped on the rail below the fork, SW_DOG screws in the post set 0.5 mm short of the tab nut at full engagement. Push the fork 7 mm down with the dogs tip to tip, then turn the REEL face until they drop in. Turn the face hard both ways, then pull the fork back. Meter on beep between the lower screw and the tab nut.
- Pass: Yoke flexes when tip to tip (no jam, roofs slide off), dogs drop in within a quarter turn, under hand torque the dogs pull in, never walk out, a light pull on the fork releases them, the yoke springs back flat; the meter beeps only with the dogs fully in, never tip to tip.
- If it fails, change: _drum_kin.py DEFAULTS dog_backcut, dog_roof, dog_deg, dog_gap, fork_k; drum.py GEOM fork_t, yoke_r, groove, dog_touch, dogpost_s; fits.py SHAFT_IN_BORE (collar on the hex, face on the journal), HOLE_SLIDE (fork on the rail).
- Made from: drum.dog_teeth (dog_n 4, dog_h 5, dog_deg 36, backcut 4, roof 1.2, r 18..26), drum.fork, drum.dog_post (dog_touch 0.5), drum.rail, drum.hexprism, GEOM journal d30, groove (82.0, 88.0), stroke 7, neutral gap 2.5

### 4. Face slip clutch with the flexure spring

- Parts: rc4_hub_plate, rc4_friction_disc, rc4_pressure_plate, rc4_flexure, rc4_adjuster_plate
- Bought: 3 x M3x20 SHCS, 3 x M3 nut, string, coins in a small bag, kitchen scale
- Do: Stack on the key stub: hub plate (counterbores down), friction disc, pressure plate (pads up), flexure (beam down, tabs on the screws), adjuster plate (traps up); 3 screws from below, nuts on top. Run the nuts down until the flexure just touches (zero), then turn all 3 nuts the same, 1/4 turn (0.125 mm) at a time. Clamp the stub level in a vice, tie a string to a disc lug (r 28.5 mm), lay it over the rim and hang a bag of coins weighed on a kitchen scale. Add or take coins to find the hold and slip weights at each setting.
- Pass: At each setting the lower weight must not pull the disc round and the higher one must (smooth steady slip, no judder); same result 3 times; the flexure never bottoms before 1 turn. Bands: 1/4 turn: 35 g holds, 60 g slips (design 48 g, 0.013 N m); 1/2 turn: 70 g holds, 120 g slips (design 95 g, 0.027 N m); 3/4 turn: 105 g holds, 180 g slips (design 143 g, 0.040 N m); 1 turn: 140 g holds, 240 g slips (design 190 g, 0.053 N m).
- If it fails, change: cad/lib/clutch.py: slip_clutch args flex_t, flex_gap, flex_ro, r_in, r_out, n_grooves; MU_PLA_PLA, E_PLA, SIGMA_FLEX_PLA_SUSTAINED (creep: if the torque falls overnight).
- Made from: cad.lib.clutch.slip_clutch() defaults (reel does not use it: its clutch is the spool face plus the PETG rocker): shaft d20 key 6, friction r 18.5..27, mu 0.3, flexure t 0.8 gap 0.5, 0.106 N m per mm, travel 0.50; on the shared keyed stub of coupon 7

### 5. Bearing seats

- Parts: rc5_seats_608_693, rc5_csk20_seat
- Bought: 608ZZ, F693ZZ, 693ZZ, CSK20PP
- Do: Press each bearing in square with a flat block or a vice (tile left to right: 608, F693ZZ, 693ZZ); CSK20PP into the round boss with its outer keyway on the printed key.
- Pass: Firm press, seated flat on the lip, stays in when tapped upside down, spins as freely as before; the CSK20PP key slides in without force, it still freewheels one way and locks the other.
- If it fails, change: fits.py BRG_RIB_INTERF, BRG_RIB_BORE (all seats); bearings.py CSK20_KEY_RCLEAR, CSK20_OKEY_W (key), KEY_CLEAR in fits.py. If the 608 is off, print tile A of the fit coupon (base/pack/fit_coupon, COUPON.md).
- Made from: cad.lib.bearings.seat_608(bottom lip, lip_d 14) as reel, seat_f693(), seat_693(), seat_csk20(key_deg 90) as the reel capstan block (wall 4); fits BRG_RIB_INTERF 0.15, BRG_RIB_BORE 0.3, CSK20_KEY_RCLEAR 0.2

### 6. M3 holes

- Parts: rc6_m3_tile
- Bought: M3x4x5 heat-set insert, M3 nut, 2 x M3x10
- Do: Left to right: heat-set the insert (220 C), press the nut in, push a screw through, self-tap a screw in.
- Pass: Insert flush and does not spin when the screw is tightened; nut presses in and holds; screw drops through the clearance hole; the tap hole cuts a thread and holds a hard pull without splitting.
- If it fails, change: fits.py M3_INSERT_D, M3_NUT_AF, M3_CLEAR_D, M3_TAP_D (tiles C, D, E of the fit coupon in base/pack/fit_coupon bracket them).
- Made from: cad.lib.fasteners.m3_insert (d 4.0), m3_nut_trap (AF 5.8), m3_clear (d 3.4), m3_tap (d 2.6)

### 7. Key rib and spigot

- Parts: rc7_key_stub, rc7_key_bore, rc7_spigot, rc7_bushing
- Bought: CSK20PP from coupon 5 (inner keyway)
- Do: Slide the key stub into the keyed tile and into the CSK20PP inner race (the real interface). Push the spigot into the sleeve slice and turn it.
- Pass: Stub slides in by hand, key in the keyway, no visible rock. Play: clamp the tile, tape a ruler to the stub end, mark the ruler 60 mm from the axis, rock the stub both ways: the mark moves under 1 mm (about 1 deg). The spigot turns freely; rocked at its free end it moves under 0.5 mm.
- If it fails, change: fits.py KEY_CLEAR, SHAFT_IN_BORE; shafts.py KEY20_H, SHAFT_PRINT_UNDERSIZE; launch.py SPIGOT_BORE (now 12.35); drum.py GEOM spig_d.
- Made from: cad.lib.shafts.keyed_shaft(d 20, key_w 6, key_h 2.6) and keyed_bore (clear 0.3); drum.main_shaft (spig_d 12); launch.sleeve_solid (SPIGOT_BORE 12.35)

### 8. Gear meshes m1.25, m1.5, m2, m2.5

- Parts: rc8_m125_plate, rc8_m125_seg, rc8_m125_pinion, rc8_m15_plate, rc8_m15_z42, rc8_m15_z24, rc8_m2_plate, rc8_m2_z50, rc8_m2_z22, rc8_m25_plate, rc8_m25_big, rc8_m25_drive
- Bought: none
- Do: Each pair on its plate (label = module), flat faces down; turn or rock through every tooth.
- Pass: Smooth, no tight spot, a little play when rocked; m2.5: the tall tooth and the cut-down last tooth pass without catching, then the mesh lets go at the gap.
- If it fails, change: fits.py GEAR_BACKLASH, GEAR_CENTER_EXTRA (m1.25, m1.5, m2); launch_gears.py center_distance extra, spur backlash, sector_gear_2d tall_factor (m2.5); _drum_kin.py pin_x (m1.25 pinion).
- Made from: m1.25: gears.spur_gear z60 sector / z10 shift 0.4 tip_short 0.02 (drum.tip_short_for), cd 44.33 (drum.a_seg); m1.5: gears.spur_gear z42 sector / z24, cd 49.60; m2: gears.spur_gear z50 sector (drum GEOM out_m, out_z) / z22 (reel Z_CAPG), cd 72.10; m2.5: launch_gears.sector_gear_2d z40 (n_present 27, last 6 + gap) / launch_gears.spur z26, cd 82.60 (launch L); coupon face 6, fits GEAR_BACKLASH 0.15, GEAR_CENTER_EXTRA 0.1

### 9. Shear-pin torque limiter

- Parts: rc9_hub_lever, rc9_outer_lever, rc9_shear_pin x3
- Bought: string or a strap, luggage scale
- Do: Hub slice into the outer slice, pin through both holes (head outside). Clamp the hub lever, hang the luggage scale on the outer lever hole (r 60 mm) at right angles and pull slowly until the pin shears. Break all 3 pins. Torque = kg x 9.81 x 0.06 N m.
- Pass: Each pin shears cleanly between 5.9 and 10.2 kg on the scale (3.5 to 6 N m, drum_NOTES; design 4.9 N m = 8.3 kg), the 3 pins within 20 percent of each other, both slices undamaged (holes still round); a fresh pin goes in by hand.
- If it fails, change: drum.py GEOM shear_d (torque goes with d squared), hub_od (radius), shear_s; fits for the hole: drum.coupler shear hole (shear_d + 0.1) and shear_pin (shear_d - 0.04). Spec D8 / D26 limit 5 N m.
- Made from: drum.coupler_hub and drum.coupler (sliced s 3.5..16.5), drum.shear_pin: d4 PLA in single shear at r 13 (shear_s 13); 4.9 N m at 30 MPa

## Parts

| part | material | qty | print bbox mm | what |
|---|---|---|---|---|
| rc1_plate | PLA | 1 | 83.3 x 83.2 x 28.3 | Splitter base: centre pin d8, output pin d8, stop finger column (2 x M3 tap) |
| rc1_sleeve | PPA-CF | 1 | 26.0 x 26.0 x 24.5 | Sleeve stub d20 with the integral key rib (launch.RIB_W, RIB_R) under the sun and the crank |
| rc1_sun | PPA-CF | 1 | 47.8 x 48.0 x 8.0 | launch_aim.sun_solid: sun m1.5 z30 as built |
| rc1_arm | PPA-CF | 1 | 57.9 x 32.3 x 14.0 | launch_aim.arm_solid: swing arm as built (O-ring groove, d8 pin, clip groove) |
| rc1_idler | PPA-CF | 1 | 30.0 x 29.7 x 8.0 | launch_aim.idler_solid: idler m1.5 z18 as built |
| rc1_output | PPA-CF | 1 | 47.8 x 48.0 x 8.0 | Output m1.5 z30 (launch_gears.spur, face 8) on a d8 pin |
| rc1_crank | PPA-CF | 1 | 48.2 x 30.3 x 20.0 | Hand crank, keyed on the sleeve rib like the sun |
| rc1_stop | PLA | 1 | 47.1 x 10.0 x 6.4 | Stop finger: rounded end at the as-built stop post position |
| rc2_switch_base | PLA | 1 | 48.0 x 35.0 x 11.5 | contact_switch base: pad, nut pillar, hard stop, wire channel |
| rc2_switch_lever | PLA | 1 | 44.0 x 5.0 x 8.0 | contact_switch lever, printed on its side (Y up) |
| rc3_base | PLA | 1 | 40.0 x 91.5 x 45.0 | Base: d30 journal (REEL sleeve) and hex 22 AF (main shaft), rail hole (press) |
| rc3_reel_face | PPA-CF | 1 | 61.6 x 54.0 x 9.5 | REEL gear dog face (drum.dog_teeth, phase 45) with a handle |
| rc3_collar | PPA-CF | 1 | 60.0 x 60.0 x 18.0 | Collar lower half: drum.dog_teeth facing down, fork groove, hex bore |
| rc3_fork | PLA | 1 | 80.9 x 100.8 x 40.0 | drum.fork as built: compliant yoke, rail sleeve, follower pin, M3 nut in the tab |
| rc3_dog_post | PLA | 1 | 75.5 x 20.0 x 37.6 | drum.dog_post as built: SW_DOG_L / SW_DOG_R screw bosses, clamps on the rail |
| rc3_rail | PPA-CF | 1 | 77.3 x 10.0 x 9.0 | drum.rail profile d10 with the print flat, cut to 77 long |
| rc4_hub_plate | PLA | 1 | 54.0 x 54.0 x 6.0 | clutch.slip_clutch hub_plate (library defaults) |
| rc4_friction_disc | PLA | 1 | 57.0 x 55.0 x 3.0 | clutch.slip_clutch friction_disc (library defaults) |
| rc4_pressure_plate | PLA | 1 | 54.0 x 54.0 x 3.5 | clutch.slip_clutch pressure_plate (library defaults) |
| rc4_flexure | PLA | 1 | 50.0 x 50.0 x 1.7 | clutch.slip_clutch flexure (library defaults) |
| rc4_adjuster_plate | PLA | 1 | 54.0 x 54.0 x 4.0 | clutch.slip_clutch adjuster_plate (library defaults) |
| rc5_seats_608_693 | PLA | 1 | 60.0 x 28.0 x 9.6 | 608 (reel spool seat), F693ZZ and 693ZZ crush-rib seats |
| rc5_csk20_seat | PLA | 1 | 55.3 x 55.3 x 16.0 | CSK20PP seat with the printed key (reel capstan block, key_deg 90) |
| rc6_m3_tile | PLA | 1 | 54.0 x 18.0 x 8.6 | M3 heat-set hole, nut trap, clearance and tap holes (fits.py values) |
| rc7_key_stub | PPA-CF | 1 | 21.2 x 20.0 x 31.0 | shafts.keyed_shaft d20 key 6 (reel capstan tube numbers), 31 long, also the coupon 4 clutch shaft |
| rc7_key_bore | PPA-CF | 1 | 36.0 x 36.0 x 8.6 | shafts.keyed_bore d20 key 6, 8 long (keyed hubs on the sleeve rib are PPA-CF) |
| rc7_spigot | PPA-CF | 1 | 28.0 x 25.4 x 22.0 | drum.main_shaft slice: hex 8 long + d12 spigot 20 long, on a hex flat |
| rc7_bushing | PPA-CF | 1 | 32.0 x 32.0 x 16.0 | launch.sleeve_solid slice 16 long: spigot bore d12.35 |
| rc8_m125_plate | PPA-CF | 1 | 62.3 x 18.0 x 11.0 | Pins at 44.33 |
| rc8_m125_seg | PLA | 1 | 38.1 x 46.7 x 6.0 | drum segment gear m1.25 z60 sector |
| rc8_m125_pinion | PPA-CF | 1 | 16.0 x 15.3 x 6.0 | drum wrap pinion m1.25 z10 x0.4 tip_short 0.02 |
| rc8_m15_plate | PPA-CF | 1 | 67.6 x 18.0 x 11.0 | Pins at 49.60 |
| rc8_m15_z42 | PPA-CF | 1 | 41.0 x 29.6 x 6.0 | drum wrap idler output m1.5 z42 sector |
| rc8_m15_z24 | PPA-CF | 1 | 38.9 x 38.9 x 6.0 | reel cup ring gear m1.5 z24 |
| rc8_m2_plate | PPA-CF | 1 | 90.1 x 18.0 x 11.0 | Pins at 72.10 |
| rc8_m2_z50 | PPA-CF | 1 | 61.0 x 39.3 x 6.0 | drum REEL output m2 z50 sector |
| rc8_m2_z22 | PPA-CF | 1 | 48.0 x 47.7 x 6.0 | reel capstan gear m2 z22 |
| rc8_m25_plate | PPA-CF | 1 | 100.6 x 18.0 x 11.0 | Pins at 82.60 |
| rc8_m25_big | PPA-CF | 1 | 61.5 x 49.0 x 6.0 | launch big gear m2.5 z40 sector: teeth 21..26 (tall, relieved) |
| rc8_m25_drive | PPA-CF | 1 | 42.5 x 44.0 x 6.0 | launch drive gear m2.5 z26 sector |
| rc9_hub_lever | PPA-CF | 1 | 79.0 x 26.0 x 13.0 | drum.coupler_hub slice (shear pin hole) with a lever, string hole at r 60 |
| rc9_outer_lever | PPA-CF | 1 | 86.0 x 40.0 x 7.0 | drum.coupler slice (shear pin hole) with a lever, string hole at r 60 |
| rc9_shear_pin | PLA | 3 | 4.6 x 6.0 x 12.1 | drum.shear_pin as built (d4 PLA) |
