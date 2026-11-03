# frame_REPORT (worker cad-frame) - interim, updated through the day

Final version at 18:30 Sydney. Notes and data: `frame_NOTES.md`.

## Done so far
- Parametric frame from params (LAYOUT, FRAMEWORK, WIPER): deck (dovetail tiles over the bed size, own drill template
  for the rover), cheeks (dovetail split over the bed size), bearing blocks (608 / CSK20PP / printed bushing seats from
  cad.lib.bearings), cheek ties, pedestals for shafts not along Y, wiper motor adapter (slotted, separately printed,
  standoff hub) and saddle, electronics bay (ESP32-S3 cradle with USB slot, BTS7960 standoffs and vents, DRV8833,
  MP1584, HX711 pockets, fuse cradle, wire exits with tie anchors), AS5600 board bracket, cable-tie points on the feet.
- Assembly steps (whole-base order), load hand calcs, PC tests, quick-look render script.

## Requests to other workers / coordinator
- cad-infra / coordinator: `tools.assembly` loads every `cad/parts/*.py` without a leading underscore as a subsystem.
  `launch_kin.py` (cad-launch) and `drum_gears.py`, `test_drum_gears.py` (cad-drum) break `Assembly.load` (no
  parts/joints/kin). Rename them with a leading underscore (as frame does: `_frame_*.py`) or have the loader skip
  modules without `parts`.
- I merged `origin/w/cad-infra` and `origin/base-v1` into `w/cad-frame` to code against the library.

## Open items
- Conform to BASE_SPEC v1.0 / params LAYOUT when it lands; guards over gears and the spring path; full-assembly audit.
