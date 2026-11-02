# 6. Dart build

Build the dart with the KEDGE v30 run sheet (appendix at the end of this guide, also
`kedge/pack_v30/KEDGE_v30_run_sheet.pdf`). It is a complete, step by step, timed build day: fit coupon, plates,
board prep, harness, power-up tests, motor directions, mechanical assembly, function test.

**One change from the run sheet: the dart pin map.** The run sheet uses three DRV8833 boards. The dart now uses two,
so the third can drive the base selector drum:

- DRV#1: deploy and kick (unchanged).
- DRV#2 channel A: BOTH drill motors (0.7 A stall together, fine on one 1.2 A channel), right drill motor leads
  swapped as before so the pair counter-rotates.
- DRV#2 channel B: the foot N20.
- No DRV#3 in the dart: skip every run sheet wire that goes to DRV#3, and do not bridge AIN1 to BIN1 on DRV#2.

Where the run sheet wiring (its sections 7 and 9 and its wiring picture) differs from `firmware/dart/WIRING.md`
below, follow WIRING.md. Use the firmware in `firmware/dart/`, not the copy inside the v30 pack. Ignore the run
sheet's `firmware/reel` sketch: the base firmware in `firmware/base/` replaces it.

The rest of the run sheet stands: the fit coupon, both plates, the 5.04 V buck, the Schottky diode, the capacitor
polarity, the battery contacts, the assembly order and the function test.

## Dart wiring (new pin map)

@include firmware/dart/WIRING.md

@image firmware/dart/wiring/wiring.png Dart wiring v4 (numbers match the new cut list)

### New dart wire cut list (replaces run sheet section 9)

@csv firmware/dart/wiring/wire_cut_list.csv

## Dart bench tests

@include firmware/dart/BENCH_TEST.md optional

The run sheet sections 7.5, 7.6, 8d step 6 and 10 are the dart bench checks: power-up readings, motor directions,
battery power-up, then the function test in damp soil.
