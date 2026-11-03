# 4. Printing

Printer: Bambu X2D (bed 256 x 256 x 260 mm). PLA for everything except loaded gears and keyed shafts, which are
PPA-CF (the print list says which).

Hole compensation (used by both coupons): in Bambu Studio it is a process setting, "X-Y hole compensation"
(Process, Quality tab; the name may move between versions). Set it before slicing each plate; it is saved with the
project, so set it on both dart plates and every base plate.

Order: dart fit coupon first (20 minutes, checks your printer's hole sizes), then the base day-one risk coupon
plate (below, test stage 0), then the dart plates, then the base once the coupons pass and the wiper and spring
numbers are in (section 1, order of work).

## Day-one risk coupons (print before any big part)

One plate from the base print pack with small test pieces of the riskiest features: swing splitter, printed contact,
dog clutch, slip clutch, bearing seats, heat-set inserts and nut traps, key rib, gear meshes. Print it, then work
through its test sheet. Do not print the big base parts until every coupon passes.

@imageglob base/**/coupon/coupon_plate_*.png Risk coupon plate

@includeglob base/**/coupon/COUPON.md first

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
