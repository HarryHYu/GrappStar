# Base station physics: how to use it

Every number for the launcher, the dart flight, the reel, the capstan and the wiper loads comes from the Python
files in this folder. They read the shared numbers from `base/spec/params.py`. When you measure something, put
the measured value in the spec and run everything again; the ballistics table for the Jetson and the summary PDF
update themselves.

## One-time setup

The scripts use the repo's Python environment `kedge/.venv` (numpy, matplotlib, reportlab, pytest). If it is
missing: `cd kedge && python3 -m venv .venv && .venv/bin/pip install -r requirements.txt`. Below, `python` means
`../../kedge/.venv/bin/python` when you are in `base/physics`.

## Run everything (about 3 to 5 minutes)

1. Open a terminal in the repo.
2. `cd base/physics`
3. `./run_all.sh`
4. Read `SUMMARY.md` (or `base/out/physics/physics_summary.pdf`) for the numbers. The Jetson reads
   `base/out/ballistics.json`.

## After the spring hang test

1. Hang 1 kg, 2 kg and 3 kg from ONE spring (water bottles in a bag are fine). Measure the length each time, from
   the outside of one hook to the outside of the other, in mm. Also measure it with nothing hanging. Then do the
   other spring the same way (MEASURE_LIST B2).
2. Run, with your numbers, once per spring: `python calib.py spring 165.1 1:187 2:265.5 3:344`
   (first the free length, then kg:mm for each weight).
3. It prints the rate (N/mm), the initial tension (N) and the force per spring at full cock (265 mm extension)
   against the 38.4 N catalogue rating.
4. Edit `base/spec/params.py` yourself: in `SPRING` set `k_n_per_mm` and `F0_n` (the mean of the two springs)
   and `measured=True`. If the STRONGER spring's force at full cock is over 38.4 N, fit the springs on hole set F
   (8.5 mm rearward, 256.5 mm at full cock; MEASURE_LIST B4). Go by that force, not by the verdict words.
5. `./run_all.sh`

## After measuring the wiper motor

The wiper is measured through the base firmware on the bench, as MEASURE_LIST A6 and A7 say (after CALIBRATION.md
(a) and (b)). Never put the meter in series at full power or in a stall: its 10 A range cannot take a start surge.

1. A6, free speed and free current: a forward bench jog runs at 40 percent power. Read `rpm` and `amps` in
   telemetry while it runs steady. `n0_rpm` = jog rpm x 2.5 (never the raw jog rpm); `free_a` = the `amps` reading.
2. A7 (optional): the firmware trips jogs at 2.5 to 3 A, so the stall current cannot be read and is not needed. The
   torque at the trip (300 mm bar, luggage scale) is only a lower bound for `stall_nm`.
3. Edit `WIPER` in `base/spec/params.py`: `n0_rpm`, `free_a` and `measured=True`; keep `stall_nm` and `stall_a`.
   Then `./run_all.sh`. If the summary shows "STALL" for a function, the wiper is too weak for it.

## After the first real shots (the best calibration of all)

1. On level ground, fire 2 or 3 shots at different pitches (for example 20 and 36 deg) with the LIGHT clutch.
2. Measure each range from the ground point under the rail pivot to where the dart first touched down.
3. `python calib.py range 20:0.5:3.6 36:0.5:4.2` (pitch:brake:range for each shot).
4. It prints a muzzle speed factor. Put it in `base/spec/params.py` as `LAUNCH` `v_scale=<factor>`, then
   `./run_all.sh`: the table the Jetson uses (and the summary) now carry it, every run. This corrects spring,
   friction and reel errors in one step.

## Other numbers worth measuring

- Dart mass on a kitchen scale with the ballast and tether knot (0.279 kg assumed).
- Payout brake (MEASURE_LIST G4): hanging weights, since a luggage scale cannot read 0.5 N. In LAUNCH, 40 g on the
  line must not pull line off and 50 g must (LIGHT 0.5 N); in NEUTRAL the FIRM clutch (5 N) lets line off at
  about 0.5 kg on the luggage scale.
- Rail friction: tilt the rail with the dart on it until it slides. mu = tan(angle).
- Rover mass, wheelbase and deck height (for recoil and launch height): edit `ROVER` in `recoil.py` and
  `deck_h` in `common.py`.

## What each file does

See `REPORT.md` (table "Done"). Problems found for the design are in `FINDINGS.md`.

## Tests

`python -m pytest -q` in this folder runs every check (about 1 min; run_all.sh runs them last). If the
staleness test fails, the spec changed something the table depends on: run `./run_all.sh`. They never write to `base/out` or `kedge/out30` (the conftest checks it).
