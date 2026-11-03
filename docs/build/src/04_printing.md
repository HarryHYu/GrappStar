# 4. Printing

Printer: Bambu X2D (bed 256 x 256 x 260 mm). PLA for everything except loaded gears and keyed shafts, which are
PPA-CF (the print list says which).

Magnet for the bench: the AS5600 magnet cap is coupon 10 on the PLA risk plate
(`base/pack/coupons/rc10_magnet_cap.stl`; it fits the main shaft spigot, so it is for the base, not the bare
motor); glue the magnet in it. For the bench test on the clamped motor, tape the magnet centred on the shaft end and clamp the AS5600 board 1 to 2 mm from it. `status`
must show `"magnet":"OK"` (WEAK: move the board closer; STRONG: further; NONE: no magnet seen).

Print settings (the 3MF files carry geometry only: set material and settings in Bambu Studio for each plate):

| Plate | Material | Layer | Walls | Infill | Top / bottom | Hole compensation |
|---|---|---|---|---|---|---|
| Dart fit coupon, dart plates A and B | PLA | 0.16 mm | 4 | 40% gyroid | 5 / 5 | start at 0; adjust from the dart fit coupon (run sheet section 4) |
| Base fit coupon, risk plate 1 | PLA | 0.2 mm | 4 | 40% gyroid | 5 / 5 | 0 ("print as exported") |
| Risk plate 2 | PPA-CF | 0.2 mm | 6 | 60% gyroid | 6 / 6 | 0 |
| Base plates, PLA | PLA | 0.2 mm | 4 | 40% gyroid | 5 / 5 | 0, unless the coupons showed a fit off |
| Base plates, PPA-CF | PPA-CF | 0.2 mm | 6 | 60% gyroid | 6 / 6 | 0, unless the coupons showed a fit off |

No supports on any plate. The base print sheet (below) has the final word for each part.

Hole compensation is a process setting in Bambu Studio, "X-Y hole compensation" (Process, Quality tab; the name may
move between versions). It is saved with each project, so check it on every plate.

PPA-CF (risk plate 2 and the loaded base parts): hardened steel nozzle only (CF wears a brass nozzle quickly).
Dry the spool first (about 80 C for 8 hours in a filament dryer, or the printer's own drying if it has one; it
absorbs water and prints weak and stringy otherwise). Use the Bambu PPA-CF profile for temperatures. Print with the
room ventilated, and wear a dust mask when sanding or trimming CF parts.

Order: dart fit coupon first (20 minutes, checks your printer's hole sizes), then the base fit coupon
(`base/pack/fit_coupon/coupon_plate_1.3mf`, PLA) and the two base risk coupon plates (below, test stage 0), then the dart plates, then the base once the coupons pass and the wiper and spring numbers
are in (section 1, order of work).

## Day-one risk coupons (print before any big part)

Two plates from the base print pack with small test pieces of the riskiest features: swing splitter, printed
contact, dog clutch, slip clutch, bearing seats, heat-set inserts and nut traps, key rib, gear meshes, shear pin.
Print them, then work through the test sheet. Do not print the big base parts until every coupon passes.

Plates: `base/pack/coupons/risk_plate_1_PLA.3mf` (PLA, 23 pieces) and `risk_plate_2_PPA-CF.3mf` (PPA-CF, 26 pieces). Each
piece is also there on its own (`base/pack/coupons/rcN_*.stl`) for a reprint. A printable copy of the test sheet is
`base/pack/coupons/COUPON_TEST.pdf`. The coupons need a few bought parts (MEASURE_LIST C21): M3 screws and a jam
nut, heat-set inserts, one each of 608ZZ, F693ZZ and 693ZZ, a CSK20PP.
Take the CSK20PP and bearings back out of the coupons after testing: they go into the base.

If a coupon fails: write down which test and what you saw, and do not change the CAD yourself. The sheet names the
design numbers to change; give the result to a Claude Code session in the repo and ask it to change them and re-run
the coupons (in `base/`: `../kedge/.venv/bin/python -m tools.risk_coupon`, after `bash kedge/setup_mac.sh` once from the repo
root).
Then reprint only that piece from its `rcN_*.stl`.

@imageglob base/pack/coupons/*.png Risk coupon plate

@includeglob base/pack/coupons/COUPON_TEST.md first

### Base fit coupon

@imageglob base/pack/fit_coupon/*.png Base fit coupon plate

@include base/pack/fit_coupon/COUPON.md

## Dart prints

@printlist dart

## Base prints

The base print pack comes from the base pipeline (`base/out/base/pack`: PRINT_SHEET.pdf, the plates in
`plates/plate_<n>_<material>.3mf`, print_report.json). It holds every part already in print orientation,
the plates as 3MF, and the material for each part.

@printlist base

Before printing the base:

1. The wiper and spring numbers are in params.py and the pipeline was re-run (the audit said CLEAN).
2. Print one test part from the pack in PPA-CF and check a 608 drops into its seat with a firm thumb press, and an
   M3 heat-set insert sets straight. Keep hole compensation at 0 for base parts: base fits are
   tuned in the CAD through the coupons, not in the slicer.
3. PPA-CF: as in the PPA-CF paragraph and the settings table at the top of this section.
