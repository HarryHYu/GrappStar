# First session back: what to do, in order

Everything else waits until these are done. The measuring takes an afternoon; the coupons wait for the 608s to arrive.

## 1. Read (15 minutes)

- Build guide sections 1 (what the system is) and 2 (safety).
- The rule for every wiring step: battery unplugged first.

## 2. Measure (1 to 2 hours; caliper, kitchen scale, water bottles, phone camera)

Take a photo of every caliper reading and put them in a new folder `originals/measure_2026-11/` (create it).
Write each number into `docs/build/MEASURE_RESULTS.md` (a fill-in sheet; print it).

1. Wiper motor (MEASURE_LIST A1 to A5, A8): output shaft diameter and flats, end thread and length, the mount hole
   pattern (press the motor face onto paper), the housing size, the boss and hub.
2. Launch springs (B1 to B3), safety glasses on while a loaded spring hangs: free length with a steel rule or
   tape (the spring is longer than a caliper), then hang about 1, 2 and 3 kg of water bottles (weigh each load:
   W1, W2, W3 in kg) from ONE spring and measure hook to hook each time (L1, L2, L3 in mm). Both springs.
   `python3 base/physics/calib.py spring L0 W1:L1 W2:L2 W3:L3` gives the rate and initial tension. Enter the
   weaker spring's numbers.
3. Small parts (H1, H3, H4): drum N20 shaft, CSK20PP keyway depths, BTS7960 heatsink height and hole spacing.
4. Count the parts box (group C): worm N20s (4 needed), the spare MP1584, 8 mm rod or M8 bolts, 693ZZ / F693ZZ left
   after the dart, resistors (2 x 10 k, 1 k, 680 R, 100 k, 22 k), a small magnet for the AS5600, spare XT60s, and
   the coupon extras (MEASURE_LIST C21).
5. Update the design with the numbers (build guide section 14, "After you measure"): the easy way is to open
   Claude Code in the repo, give it MEASURE_RESULTS.md and ask it to enter the numbers in params.py and re-run the
   base pipeline. By hand: the commands are in that section.

## 3. Buy

- 608 bearings, 10 (search "608ZZ bearing 8x22x7").
- Only if not in the box: a small neodymium disc magnet for the AS5600 (6 to 10 mm across, 2 to 3 mm thick, or
  a 6 x 2.5 mm diametric disc).

## 4. Print first (test pieces only)

1. The dart fit coupon (`kedge/pack_v30/fit_coupon.stl`, 20 minutes). Check it against run sheet section 4.
2. The base fit coupon (`base/pack/fit_coupon/coupon_plate_1.3mf`, PLA; checks in its COUPON.md; tile A uses three
   608s) and the two base risk coupon plates (`base/pack/coupons/risk_plate_1_PLA.3mf` PLA, `risk_plate_2_PPA-CF.3mf` PPA-CF: hardened
   nozzle, dry spool) and the test sheet `COUPON_TEST.pdf` (test stage 0). Every check must pass before the big
   base parts. The coupons need a few bought parts (C21) and one each of 608, F693ZZ, 693ZZ and a CSK20PP.
3. The dart plates A and B do not wait for the base coupon or the measurements: start them once the dart fit
   coupon passes.

## 5. Do NOT do yet

- Do not print the big base parts until the coupons pass AND the measurements are in params.py and the pipeline
  has been re-run (the motor adapter, saddle and couplers depend on them).
- Do not connect the LiPo to anything new. No wiring with a battery plugged in, ever.
- Do not fit the launch springs (they go on at test stage 5).
- Do not fire anything. When the time comes: never without the tether tied on.
- Do not plug the dart into the Mac with its power loop plug in.
