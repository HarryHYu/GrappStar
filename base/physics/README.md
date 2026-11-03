# Base station physics: how to use it

Every number for the launcher, the dart flight, the reel, the capstan and the wiper loads comes from the Python
files in this folder. They read the shared numbers from `base/spec/params.py`. When you measure something, put
the measured value in the spec and run everything again; the ballistics table for the Jetson and the summary PDF
update themselves.

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
3. It prints the rate (N/mm) and the initial tension (N). Ask for them to go into `SPRING` in `base/spec/params.py`
   (`k_n_per_mm`, `F0_n`).
4. Check the force at full cock: initial tension + rate x extension. Keep it at or under about 40 N
   (Century rates this spring at 38.4 N). If it is over, use the 250 mm anchor holes.
5. `./run_all.sh`

## After measuring the wiper motor

1. Battery unplugged while you connect anything. Then free speed: count the output turns in 30 s at 12.0 V
   (rpm = turns x 2) and read the current.
2. Stall: a 100 mm arm on the output, a luggage scale at the end, power for 1 second only, read kg and amps.
3. `python calib.py wiper --v 12.0 --rpm 44 --i-free 1.4 --stall-kg 3.1 --arm-mm 100 --i-stall 17` (your numbers).
4. Ask for the printed values to go into `WIPER` in `base/spec/params.py`, then `./run_all.sh`.

## Other numbers worth measuring

- Dart mass on a kitchen scale with the ballast and tether knot (0.279 kg assumed).
- Slip clutch: luggage scale on the line with the reel full. LIGHT should read 0.5 N, FIRM 5 N.
- Rail friction: tilt the rail with the dart on it until it slides. mu = tan(angle).
- Rover mass, wheelbase and deck height (for recoil and launch height): edit `ROVER` in `recoil.py` and
  `deck_h` in `common.py`.

## What each file does

See `REPORT.md` (table "Done"). Problems found for the design are in `FINDINGS.md`.

## Tests

`python -m pytest -q` in this folder runs every check (about 25 s). They never write to `base/out` or `kedge/out30` (the conftest checks it).
