# sensors: REPORT (worker cad-sensors, 3 Nov 2026)

Branch `w/sensors`. Files: `sensors.py` (subsystem module), `test_sensors.py`, `sensors_NOTES.md` (design and
numbers), `sensors_launch_patch.diff` (the change cad-launch applied to their files), this report.

## Done

1. AS5600 mount (Adafruit PID 6357). Board modelled from Adafruit's EAGLE file: 4 corner holes (not the 2 mid holes
   assumed before), chip at the board centre, JST SH connectors on the short edges. Printed carrier on the aim housing
   spine (2 x M3x16), 4 printed pegs through the board holes, 2 printed clamp strips pulled down by M3x12 into M3 nuts in the carrier back.
   No M2.5 screws and no heat staking. Chip to magnet 2.0 mm (1.75 with the tallest SOIC), see open item 1.
2. SW_RAIL_LOW (GPIO 40): cad.lib contact switch (coupon 2 geometry) on a printed bracket standing on the aim housing
   foot extension, pressed by cad-launch's crank cam. Closed within about +-9 deg of crank from the LOW dead point
   (pitch 6.0 to 6.2).
3. SW_GAP (GPIO 39): cad.lib contact switch whose base and cheek arm are one printed part, screwed to the right rail
   cheek outer face; 4 mm riser on the lever tip so the jam nut stays under the gear web; pressed by the big gear cam
   bump; closed within about +-14 deg of the gear rest angle.
4. Spring guard: dropped from my scope (coordinator): cad-frame / cad-launch own it.
5. The old launch_switch leaf switches for SW_GAP and SW_RAIL_LOW (about 8.6 percent bending strain, 10x the PLA limit)
   and the M2.5 AS5600 box are deleted by cad-launch (patch applied in w/cad-launch, launch.py b72f249 and the aim
   commit); the housing and cheek pilots I need are cut by them.
6. Assembly steps: Sensors 1 (build both switches), 2 (SW_RAIL_LOW), 3 (SW_GAP), 4 (AS5600). Print orientations: all
   flat (switch bases on their bottom, levers on their side, carrier on its back, strips on their front, bracket foot
   down).

## Verified

(filled below with pasted test and audit output)

## Open items

1. Air gap 2.0 mm instead of the 1.0 design: the JST connectors (2.95 tall, radius 7.5 to 11.8) must clear the turning
   magnet cap (r 10). For 1.0, cad-drum steps the cap end face down to r 7 for 1.5 mm, then sensors.JST_CAP_CLR moves
   the board 1 mm closer. 2.0 is inside the AS5600 range (0.5 to 3 mm) for a 6 x 2.5 diametric magnet.
2. Harry: after building, set each contact gap by turning (touch, back off about 1.9 turns), then check the closure
   windows by hand as in steps Sensors 2 and 3.
3. cad-launch also cut a foot pilot at x 116 that is unused (under the bracket column): harmless, may be deleted.
