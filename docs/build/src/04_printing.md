# 4. Printing

Printer: Bambu X2D (bed 256 x 256 x 260 mm). PLA for everything except loaded gears and keyed shafts, which are
PPA-CF (the print list says which).

Order: dart fit coupon first (20 minutes, checks your printer's hole sizes), then the dart plates, then the base
once the wiper and spring numbers are in (section 1, order of work).

## Dart prints

@printlist dart

## Base prints

The base print pack comes from the base pipeline (`base/pack`). It holds every part already in print orientation,
the plates as 3MF, and the material for each part.

@printlist base

Before printing the base:

1. The wiper and spring numbers are in params.py and the pipeline was re-run (the audit said CLEAN).
2. Print one test part from the pack in PPA-CF and check a 608 drops into its seat with a firm thumb press, and an
   M3 heat-set insert sets straight. Adjust hole compensation the same way as the dart coupon.
3. PPA-CF: dry the spool first (it absorbs water), hardened nozzle, settings from the Bambu PPA-CF profile.
