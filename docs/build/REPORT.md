# docs worker report (docs/build/)

Status: in progress. Final version at 18:30 Sydney.

## Done

- `make_guide.py`: builds `GrappStar_Build_Guide.md` and `.pdf` from `docs/build/src/NN_*.md` plus the other workers'
  docs (included live, headings pushed down). Missing sources become placeholders naming the owner, so it always
  builds. Appends `base/guide/*.pdf` and the KEDGE v30 run sheet as PDF appendices (pypdf). Renders the system
  diagram with graphviz. Lints for em dashes. Re-run: `python3 docs/build/make_guide.py`
  (needs reportlab, pypdf, pillow; graphviz `dot` optional).
- `SYSTEM_ELECTRICAL.md` + `system_electrical.dot/.svg/.png`.
- `MEASURE_LIST.md`, `TEST_PLAN.md`, `README_draft.md`.

## Decisions made (owner away)

- Drum supply: MP1584 #2 (the dart build's spare 8.0 V bench supply) is re-set to 9.0 V after the dart bench test and
  becomes the drum N20 supply. It taps the BTS7960 B+ / B- screws so both motor branches sit behind the 20 A fuse.
- Rover power: XT60 Y harness made from spare XT60s (in the August order); lead #2 (20 A) to the wiper branch,
  lead #1 (10 A) to the Jetson barrel pigtail and the RC832.
- Dart build: the v30 run sheet stays the step-by-step dart build (appended to the guide); the guide overrides only
  its wiring with `firmware/dart/WIRING.md` (DRV#3 removed) and its reel sketch with `firmware/base`.

## Conflicts and requests

1. Worm N20 count: 4 needed now (dart kick, deploy, foot, base drum). kedge/CLAUDE.md says 2 in hand and "buy one
   more"; the August order lists 8 worm N20s. In MEASURE_LIST C3. Coordinator: confirm the "only purchase" claim.
2. pins.h needs resistors (3 x 10 k pull-downs, 1 k + 680 R IS divider, 100 k + 22 k battery divider). Not on any
   parts list; in MEASURE_LIST C7. fw-base: state the fallback if Harry has none.
3. params.py BOARDS.drv8833 says "Pololu 2130"; Harry's carriers have VIN/VCC pads (kedge/CLAUDE.md section 4),
   so they are probably not Pololu. Harmless for geometry if the size is right; flagged.
4. Physics (w/physics SUMMARY): range about 4 m at 35 degrees with the current springs. TEST_PLAN stage 8 uses a
   short target accordingly.
