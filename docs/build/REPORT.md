# docs worker report (docs/build/)

Status: in progress. Final version at 18:30 Sydney.

## Done

- `make_guide.py`: builds `GrappStar_Build_Guide.md` and `.pdf` from `docs/build/src/NN_*.md` plus the other workers'
  docs, included live (headings pushed down, relative images fixed). Directives: include, includeglob, image,
  imageglob, csv, printlist (base pack in any of csv/json/md form, from base/pack or base/out/base/pack; dart from
  kedge/pack_v30), wiremap (run sheet v29 wire numbers to the new dart numbers), assumed (params measured=False and
  None blocks), status, date. Missing sources become placeholders naming the owner, so it always builds. Appends
  `base/guide/*.pdf` and the KEDGE v30 run sheet as PDF appendices with bookmarks. Renders the system diagram with
  graphviz (wide pictures get their own rotated page). Lints for em dashes. Also writes the one-pagers:
  `Wiring_Cheat_Sheet_Base.pdf`, `Wiring_Cheat_Sheet_Dart.pdf`, `Commands_Cheat_Sheet.pdf`, and
  `MEASURE_RESULTS.md/.pdf` (fill-in sheet generated from MEASURE_LIST).
  Re-run: `python3 docs/build/make_guide.py` (reportlab, pypdf, pillow; graphviz `dot` optional).
- `SYSTEM_ELECTRICAL.md` + `system_electrical.dot/.svg/.png`: whole rover, gauges, fuses, first power-on sequence.
- `MEASURE_LIST.md` (Item | How | Changes | Reprint, groups A to H, read by the coordinator's site), `TEST_PLAN.md`
  (stages 0 coupons to 9 full mission), `FIRST_DAY.md`, `README_draft.md` (coordinator owns the root README).
- `test_make_guide.py`: 6 PC tests (em dash cleaning, heading demotion, placeholders, dart print list, full build has
  sections 1 to 15 and the key safety strings, base pack in csv/json/md forms plus the coupon sheet).

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
11. physics: base/physics/SUMMARY.md on base-v1 (12:10) is still the v0.1-stub run; asked physics to regenerate.
12. fw-base: firmware defaults disagree with the decisions: `limit_n` 20 (D3: 50 N), `trip_a` 17 / `cock_trip_a` 15
    (D8: 5 A and 6.5 A). MEASURE_LIST G10 tells Harry to set `limit_n` 50 until the defaults change.

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
- Round 4 (fresh agent, every number and name against params 1.0.1, BASE_SPEC v1.0, DECISIONS, FINDINGS, NOTES):
  17 findings, fixed: torque limiter is a shear pin (break-test, no preload), drum hand lever 14 AF, drum gear 12:1,
  CSK20PP outer keyway 1.5, wiper orientation decided (D20), no steel key (integral rib, D13), stop pad 300 N rule,
  tension per amp about 22 N/A, limit_n set to 50, 608 uses (reel 2, aim pedestal 4), magnet gap 1 to 2 mm, green
  park wire never wired, D9 exception to "WIRING.md wins", fault exceptions, dart SLP beep check, D17 final approach.
- Round 5 (learner-eyes, fresh agent following FIRST_DAY and the order of work): 16 findings. Fixed: one power-up
  order everywhere (USB in first, battery last), Mac clone step, exact pipeline commands and a "Claude Code does it"
  route, MEASURE_RESULTS.md/.pdf fill-in sheet generated from MEASURE_LIST, dart Path B step split around the BMS,
  bench-supply reading of BENCH_TEST steps 0 to 2, hole compensation location, spring measured with a rule and real
  weights, dart plates do not wait for base coupons, kickContactMa wording, part references. Sent to fw-dart: bench
  supply version of BENCH_TEST 0 to 2. fw-base items (SLP, MP1584, 2 A fuse) already sent; guide notes cover them.
- Round 6 (fresh safety reviewer): 14 findings. Fixed: one stop rule everywhere (estop, lead #2 XT60 as the physical
  kill, then battery), "cutting power freezes stored energy", an approach checklist and a failed-decock procedure,
  hand lever never under load, line-tension stance and G9 before any tow, spring hook margin (42 N vs 43.6 N) with
  stance and the preload fallback, unfused Y harness kept under 10 cm, 18350 charging on the B6 (Li-ion 2S, 1 A),
  dart pick-up rule, rover chocked and LiPo strapped, require_seated back on before shots, meter range wording.
  Sent: jetson README (COCK runs without a prompt; one stop rule), fw-dart (no cloth near augers).
- Round 7 (learner, sections 7 to 11 against the firmware headers and cli.py): every JSON command, set/get key, cal
  word and CLI subcommand quoted is accepted by the code. Fixed: order of work now bench-tests the electronics with
  the motor in a vice BEFORE base assembly, then calibrates on the assembled base; where to type commands (Mac
  Serial Monitor vs `grappstar base cmd` on the Jetson, stop the service first); Jetson wall-supply branch in the
  power-on sequence; flashing with the wiring left on; glossary terms. Sent to fw-base (cock_rest before mode
  changes, two command hosts, free current after (c), AS5600 fault test without hands near the shaft, magnet gap,
  fuse) and jetson (service holds the port).
- params 1.2.1 folded in: D30 springs C-243 rated 38.4 N, full cock 265 mm; D31 SW_DOG_L/R; D32 17 N/A; range
  4.8 m at 36 deg, 8.5 m with four springs.
