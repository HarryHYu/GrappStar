# 6. Dart build

The dart is built from three documents. This section says which to use for what, and in what order.

- **KEDGE v30 run sheet** (`kedge/pack_v30/KEDGE_v30_run_sheet.pdf`, a separate 10-page PDF from the dart project: print it
  alongside this guide): printing, board
  prep, mechanical assembly, function test. Its own section numbers are used below.
- **`firmware/dart/WIRING.md`** (below): the new wiring. It replaces the run sheet's wiring (its sections 7 and 9 and
  its wiring picture).
- **`firmware/dart/BENCH_TEST.md`** (below): the bench checks for the new firmware.

## What changed from the run sheet

- The dart now uses two DRV8833 boards. DRV#3 goes to the base selector drum; its pocket on the keel stays empty.
- DRV#2 channel A drives BOTH drill motors (right drill leads still swapped), channel B drives the foot.
- No pin bridge on DRV#2 (ignore the bridge in run sheet 7.3).
- To reverse a motor, swap its two thin input wires at the driver instead of the motor leads (run sheet 7.6 and
  10 swap motor leads by v29 number): drills = D8 / D9 at DRV#2 AIN1 / AIN2, foot = D6 / D7 at DRV#2 BIN1 / BIN2,
  deploy = D0 / D1 at DRV#1 AIN1 / AIN2, kick = D2 / D3 at DRV#1 BIN1 / BIN2. Exception: if only ONE drill turns
  the wrong way, swap that drill's motor leads.
- **Wire numbers changed.** The tape flags carry the NEW numbers from the new cut list. Wherever the run sheet names
  a wire number (sections 2 (parts table, "in wire 18"), 6.4, 6.5, 7.1, 7.6, 8b, 8d, 8e, 8f and 10), look it up in
  the table "Run sheet wire numbers" below. The two that matter most: run sheet 6.4 "3V3 wires (25 to 27) go only
  to the SLP pad" means NEW wires 23 and 24; run sheet 6.5 "wire 18 has a Schottky diode" means NEW wire 16.
- The old cut list, `wires.json` and `wiring.png` in `kedge/pack_v30/` are the 53-wire v29 files: replaced by
  `firmware/dart/wiring/` (wire_cut_list.csv, wiring.png).
- Use the firmware in `firmware/dart/`, not the copy inside the v30 pack. Ignore the run sheet's `firmware/reel`
  sketch: `firmware/base/` replaces it. The Adafruit INA219 library is no longer needed (BNO08x still is).
- Run sheet section 10, "Put them in kedge_logic.h (kickContactMa, kickStallMa, deployStallMa)": kickContactMa is
  still there, but kickStallMa and deployStallMa are gone (now `stallMa[]`). Enter the measured currents as
  BENCH_TEST.md says.
- Foot direction: the foot auger is right-hand thread, so it screws in when it turns **clockwise seen from the motor
  end, looking toward the auger tip**. The run sheet's "clockwise, seen from the shaft end" in 7.6 means the same
  only if you stand behind the motor; use the motor-end wording.

## Dart build order (follow this; it merges the three documents)

1. Run sheet section 4: print and check the fit coupon.
2. Run sheet section 5: start plate A, flash the firmware from `firmware/dart/` (section 8.2 of this guide).
3. Run sheet section 6 steps 1 to 3: clip headers, set the buck, cut the wires (cut and flag them from the NEW cut
   list below, not the run sheet's section 9). Set MP1584 #1 to 5.04 V and MP1584 #2 to 8.0 V
   from the 3S LiPo on XT60 lead #1 (run sheet 6.2), before either buck is soldered to anything. (This replaces
   WIRING.md step B2.) MP1584 #2 later becomes the base drum supply at 9.0 V.
   Then fit the one capacitor (470 to 1000 uF, 10 V or more) across MP1584 #1's output: long leg OUT+, striped leg
   OUT- (run sheet 7.1; this is the capacitor WIRING.md B2 fits).
4. Run sheet section 6 step 4: find the SLP pad on DRV#1 and DRV#2 (WIRING.md step B6). Meter on ohms (not beep: beep only catches a dead short): SLP to VM must read over 50 ohms (a Pololu pull-up of
   tens of kilohms is fine). Under 50 ohms means the pad is the motor supply: do not put 3V3 on it.
5. Battery unplugged. Solder the bench harness with WIRING.md Path B, steps B3 to B10 (B2 is replaced by step 3
   above), on the bench, outside the hull, motor leads left free at their far ends. B1 (cells and BMS) and every
   wire end that goes to the BMS (P+, P-) wait for step 9. Heat-shrink or tape every free wire end (wire 9 at its
   socket end, wire 13 at its BMS end): they are live during the bench power-up.
6. Battery unplugged. **Click the u.FL antenna onto the VTX** (never power the VTX without it). Bench power-up: the
   8.0 V MP1584 #2 on the LiPo feeds the INA219 VIN+ pad and main ground, exactly as run sheet 7.5. Check the
   readings there (about 4.7 V at the XIAO 5V pad, VTX LED on; on the KEDGE WiFi page key `z` finds 0x40 and 0x4a). USB unplugged whenever the
   bench supply is connected. BENCH_TEST.md section "Power: bench supply or battery" says how the bench supply
   stands in for the battery.
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
