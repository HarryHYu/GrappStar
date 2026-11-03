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
5. fw-base: SYSTEM_ELECTRICAL's first power-on sequence relies on the 10 k pull-downs on R_EN, L_EN and DRV8833 SLP
   (pins.h comments). Please keep them in WIRING.md with a picture of where each resistor goes.
6. fw-base: MEASURE_LIST A6/A7 use `amps` telemetry after the current-sense calibration (the meter's 10 A range cannot
   take the wiper start or stall). CALIBRATION.md should calibrate IS at low duty with the meter in series.

7. Spring preload: launch_NOTES (11:30) says 5 mm preload and 250 mm full cock; the coordinator's message and
   physics FINDINGS F10 say 35 mm and 280 mm. MEASURE_LIST B4 defers to the base assembly steps. cad-launch and the
   coordinator: make them agree.
8. DRV8833 #3 SLP: coordinator ruled unwired (11:15); firmware/base/WIRING.md at 11:33 still wires it to GPIO21. Sent
   to fw-base. My docs follow the coordinator, with a "wire SLP to 3V3" fallback for non-Pololu boards.
9. fw-base WIRING.md said both MP1584s are in the dart and to buy another; the dart uses one. Sent to fw-base.
10. Spec v1.0 section 4 lists 11 contacts (SW_DRUM_A/B/C, SW_DOG_L/R, SW_N20, SW_INDEX, SW_HOME, SW_GAP, SW_RAIL_LOW,
    SW_SEATED, SW_ARM); pins.h at 12:10 has 8 switch GPIOs and PROTOCOL.md still has the pre-v1.0 drum order
    (wrap from REEL). fw-base to update; the guide includes their files live, and TEST_PLAN points at PROTOCOL.md for
    the wrap/REEL command order.

## Review rounds

- Round 1 (electrical reviewer, fresh agent, saw the docs and pins.h/params.py): 12 findings, all fixed: inline 2 A
  and 1 A fuses on the thin taps, battery divider 100 k at the B+ screw, SLP-to-VM resistance check (some DRV8833
  boards pull EEP up to VM), fuses fitted only with the battery unplugged, polarity checks at the BTS7960 and
  MP1584, lead #2 as the motor-only emergency stop, Jetson shutdown before unplugging, second ground path warning,
  A6 free current from telemetry (not the meter), hanging weights for the 0.5 to 0.75 N payout drag, slow-motion
  frame counting formula, stall test arm at 90 degrees, spring 4 kg load limit, defined FORWARD in test stage 1.
- Round 2 (cold learner, fresh agent, read the whole preview guide with the worker branches' docs): 15 findings.
  Fixed in docs/build: a merged dart build order (run sheet + WIRING.md + BENCH_TEST), a generated run-sheet-to-new
  wire number table (the run sheet's 7.6/8b/8e/10 numbers now point at other wires), foot direction wording,
  order of work (Jetson on its wall supply first, A6/A7 at test stage 1), P5 left open until the buck is set,
  SLP pull-up procedure, release order (pods up before kick back), one payout brake number (0.5 to 0.75 N),
  spring 4 kg limit 160 mm, INA219 library dropped. Sent to fw-dart (their files): antenna before power, B2 order,
  BENCH_TEST step order and Jetson-only steps, B4/B5 overlap. "No jetson CLI" was stale (it is on w/jetson).
- Round 3 (fresh agent, cross-document consistency against spec v1.0 and DECISIONS): 13 groups. Fixed in docs/build:
  FORWARD defined physically, `require_seated false` for the no-dart dry cycles, estop exceptions (DRUM_HELD when
  cocked or wrapped), no plug pulled with the battery in (AS5600 fault test by moving the board; barrel plugged in
  with the battery out), magnet required until the firmware has the D12 index mode, jog duty in the stall test,
  tow limit 50 N default, CALIBRATION (f) moved to stage 4, `reel` then `stop`, contact reading on USB only,
  bench-built motors may stay connected for the rover power-on. Sent 11 items to fw-base (D16 order, 11 contacts,
  D9 SLP, D12, D8 trips, limit_n 20 vs 50, STEMMA plug rule, MP1584, aim range, 2 A fuse, FORWARD).
