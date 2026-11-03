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
   the outside of one hook to the outside of the other, in mm. Also measure it with nothing hanging.
2. Run, with your numbers: `python calib.py spring 165.1 1:187 2:265.5 3:344`
   (first the free length, then kg:mm for each weight).
3. It prints the rate (N/mm), the initial tension (N) and the force at full cock with a verdict.
4. Edit `base/spec/params.py` yourself: in `SPRING` set `k_n_per_mm`, `F0_n` and `measured=True`.
   If the verdict says "too high", use the 250 mm anchor holes (`max_ext` 250).
5. `./run_all.sh`

## After measuring the wiper motor

1. Battery unplugged while you connect anything. Then free speed: count the output turns in 30 s at 12.0 V
   (rpm = turns x 2) and read the current.
2. Stall: a 100 mm arm on the output, a luggage scale at the end, power for 1 second only, read kg and amps.
3. `python calib.py wiper --v 12.0 --rpm 44 --i-free 1.4 --stall-kg 3.1 --arm-mm 100 --i-stall 17` (your numbers).
4. Edit `WIPER` in `base/spec/params.py` with the printed values (`n0_rpm`, `stall_nm`, `stall_a`, `free_a`) and set
   `measured=True`, then `./run_all.sh`. If the summary shows "STALL" for a function, the wiper is too weak for it.

## After the first real shots (the best calibration of all)

1. On level ground, fire 2 or 3 shots at different pitches (for example 20 and 36 deg) with the LIGHT clutch.
2. Measure each range from the ground point under the rail pivot to where the dart first touched down.
3. `python calib.py range 20:0.5:3.6 36:0.5:4.2` (pitch:brake:range for each shot).
4. It prints a muzzle speed factor. Put it in `base/spec/params.py` as `LAUNCH` `v_scale=<factor>`, then
   `./run_all.sh`: the table the Jetson uses (and the summary) now carry it, every run. This corrects spring,
   friction and reel errors in one step.

## Other numbers worth measuring

- Dart mass on a kitchen scale with the ballast and tether knot (0.279 kg assumed).
- Slip clutch: luggage scale on the line with the reel full. LIGHT should read 0.5 N, FIRM 5 N.
- Rail friction: tilt the rail with the dart on it until it slides. mu = tan(angle).
- Rover mass, wheelbase and deck height (for recoil and launch height): edit `ROVER` in `recoil.py` and
  `deck_h` in `common.py`.

## What each file does

See `REPORT.md` (table "Done"). Problems found for the design are in `FINDINGS.md`.

## Tests

`python -m pytest -q` in this folder runs every check (about 1 min; run_all.sh runs them last). If the
staleness test fails, the spec changed something the table depends on: run `./run_all.sh`. They never write to `base/out` or `kedge/out30` (the conftest checks it).
