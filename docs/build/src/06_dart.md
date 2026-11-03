# 6. Dart build

The dart is built from three documents. This section says which to use for what, and in what order.

- **KEDGE v30 run sheet** (appendix at the end, also `kedge/pack_v30/KEDGE_v30_run_sheet.pdf`): printing, board
  prep, mechanical assembly, function test. Its own section numbers are used below.
- **`firmware/dart/WIRING.md`** (below): the new wiring. It replaces the run sheet's wiring (its sections 7 and 9 and
  its wiring picture).
- **`firmware/dart/BENCH_TEST.md`** (below): the bench checks for the new firmware.

## What changed from the run sheet

- The dart now uses two DRV8833 boards. DRV#3 goes to the base selector drum; its pocket on the keel stays empty.
- DRV#2 channel A drives BOTH drill motors (right drill leads still swapped), channel B drives the foot.
- No pin bridge on DRV#2.
- **Wire numbers changed.** The tape flags carry the NEW numbers from the new cut list. Wherever the run sheet names
  a wire number (sections 7.6, 8b, 8d, 8e, 8f and 10), look it up in the table "Run sheet wire numbers" below.
- Use the firmware in `firmware/dart/`, not the copy inside the v30 pack. Ignore the run sheet's `firmware/reel`
  sketch: `firmware/base/` replaces it. The Adafruit INA219 library is no longer needed (BNO08x still is).
- Run sheet section 10, "Put them in kedge_logic.h (kickContactMa, kickStallMa, deployStallMa)": those names are
  gone. Enter the measured currents as BENCH_TEST.md says.
- Foot direction: the foot auger is right-hand thread, so it screws in when it turns **clockwise seen from the motor
  end, looking toward the auger tip**. The run sheet's "clockwise, seen from the shaft end" in 7.6 means the same
  only if you stand behind the motor; use the motor-end wording.

## Dart build order (follow this; it merges the three documents)

1. Run sheet section 4: print and check the fit coupon.
2. Run sheet section 5: start plate A, flash the firmware from `firmware/dart/` (section 8.2 of this guide).
3. Run sheet section 6 steps 1 to 3: clip headers, set the buck. Set MP1584 #1 to 5.04 V and MP1584 #2 to 8.0 V
   from the 3S LiPo on XT60 lead #1 (run sheet 6.2), before either buck is soldered to anything. (This replaces
   WIRING.md step B2.) MP1584 #2 later becomes the base drum supply at 9.0 V.
4. Run sheet section 6 step 4: find the SLP pad on DRV#1 and DRV#2 (WIRING.md step B6). Meter on resistance: SLP to
   VM must read open.
5. Battery unplugged. Solder the bench harness with WIRING.md Path B, steps B1 and B3 to B10, on the bench, outside
   the hull, motor leads left free at their far ends. Leave the BMS end of the battery wires for step 9.
6. Battery unplugged. **Click the u.FL antenna onto the VTX** (never power the VTX without it). Bench power-up: the
   8.0 V MP1584 #2 on the LiPo feeds the INA219 VIN+ pad and main ground, exactly as run sheet 7.5. Check the
   readings there (about 4.7 V at the XIAO 5V pad, VTX LED on, `z` finds 0x40 and 0x4a).
7. Motor directions on the bench (BENCH_TEST.md, the steps that need only a loose motor; run sheet 7.6 with the
   new wire numbers). Mark each motor lead. Disconnect the bench supply.
8. Start plate B. Run sheet 8a: nose ballast.
9. Battery unplugged. Run sheet 8b: battery contacts and BMS, using the new wire numbers.
10. Run sheet 8c, 8d, 8e, 8f in order (pods, tray, close the hull, kick leg and foot). The run sheet closes the hull
    with the kick and drill leads threaded out through their holes, then solders those motors outside: that is
    intended. Antenna still on before every loop-plug-in.
11. BENCH_TEST.md for everything that needs the legs fitted, then run sheet section 10 (function test in damp soil).
    Steps that use the Jetson (`kedge_client.py`, the RC832) wait until sections 7 and 9 are done: from the Jetson
    use `python3 -m grappstar dart status`.

## Dart wiring (new pin map)

@include firmware/dart/WIRING.md

@image firmware/dart/wiring/wiring.png Dart wiring v4 (numbers match the new cut list). To zoom in, open firmware/dart/wiring/wiring.svg on a screen.

### Run sheet wire numbers

When the run sheet names a wire number, use the new number from this table.

@wiremap

### New dart wire cut list (replaces run sheet section 9)

@csv firmware/dart/wiring/wire_cut_list.csv

## Dart bench tests

@include firmware/dart/BENCH_TEST.md
