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

## Conflicts and requests (status at 12:45)

Resolved during the day (kept for the record):
- DRV8833 #3 SLP: D9 unwired; fw-base WIRING now matches (GPIO21 is SW_HOME). Boards are Pololu 2130 (order).
- Spring preload: D5 then D30 (C-243 rated 38.4 N, full cock 265 mm, fallback 250 mm). MEASURE_LIST B4 follows D30.
- Tension model and defaults: D32/D33, firmware now ships k_n_per_a 17 with offset -10.4 and limit_n 50.
- AS5600 magnet: firmware has the D12 SW_INDEX fallback; MEASURE_LIST C13 says magnet recommended, cam fallback.
- Physics SUMMARY regenerated on params 1.0.1 and later.
- Dart docs (fw-dart): antenna before power, bench supply section, no cloth near augers, safe-to-touch rule.
- Jetson README: COCK confirmation, one stop rule, tow prerequisite and refusal while k_n_per_a is factory.

Still open (owner):
1. Worm N20 count: 4 needed (dart kick, deploy, foot, base drum). In MEASURE_LIST C3 and D19 confirm list.
2. Resistors (2 x 10 k, 1 k, 680 R, 100 k, 22 k): not on any parts list; MEASURE_LIST C7.
3. fw-base: check PROTOCOL.md matches BASE_SPEC 10.1 (tidy, retrieve, wrap from LAUNCH/NEUTRAL, 7 wraps) and the
   learner-round items sent at 12:25 (cock_rest before mode changes, two command hosts, free current after (c),
   AS5600 fault test without hands near the shaft, magnet gap 1 to 2 mm, 2 A tap fuse, MP1584 "buy another").
4. physics: base/physics/README.md should document `calib.py range` and `ballistics.py --v-scale` (TEST_PLAN
   stage 6 step 5 points there).
5. cad-infra: base print pack, coupon sheet (base/pack/coupons/COUPON_TEST.md) and assembly steps (base/guide) not
   on base-v1 yet; the guide shows placeholders until they land and picks them up automatically.

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
- Round 8 (learner, printing and the day-one coupons): 12 findings. Fixed: one print settings table per plate,
  PPA-CF handling (hardened nozzle, drying, ventilation, dust) before risk plate 2, coupon failure route (report to
  a Claude session, reprint the single rcN piece), two plates named with paths, coupon bought extras (C21) in
  FIRST_DAY and parts, contact gap 0.9 mm matched to the coupon, spring free length with a rule, git install prompt,
  glossary (AF, SHCS, e-clip, jam nut, module, crush rib, backlash, self-tapper, hole compensation), Markdown image
  paths relative to docs/build. Sent to cad-infra: luggage-scale-too-coarse test, shear pin pass range, unmeasurable
  play tests, missing base fit coupon, settings in the sheet, Mac command.
- Round 9 (coordinator's review files r1_G and r2_J, items owned by docs): fixed r2_J #10 (stale pin map: diagram
  GPIOs now filled from pins.h at build, new test that every GPIO in the built guide is a pins.h pin), D38 (no
  4-spring promises), #18 (stage 4 uses rubber-band stand-in springs; cock/decock then fire), #16 (A6 n0 scaled
  x 2.5 from the 40 percent jog), #22 (per-contact gaps), #23, #24 (tools), #25/26 (coupon sheet included), #27
  (motor adapter M3 x 10, 10 of 12), #28 (swing-screw drum release); r1_G #6 (B4 = D30), #7 (stage 2 current band
  under the jog trips), #9 (firing via the new `grappstar base fire`, arm and fire inside the 3 s token life),
  #10 (estop holds), #14/#15 (SLP unwired, no 3V3 fallback). Sent to fw-base: k_n_per_a procedure (4 A vs 3 A jog
  trip), latch wording, WIRING 2 A fuse; fw-dart: B2 battery-unplugged wording.
- Round 10 (full cold read, steps 0 to 11): 19 findings. Fixed: clone then `git checkout base-v1` if main lacks the
  docs, Jetson step 4 after flashing, calibration part letters ((d) free current, (f) tension, (g) cock geometry),
  range numbers now generated from base/out/ballistics.json at build time (best 3.7 m at 38.5 deg today, not the
  older 5 m), safety energy about 7 J, A7 optional (jog trips far below stall), A6 after CALIBRATION (a) and (b) at
  step 7, dart SLP check in ohms (over 50 ohms), `z` over USB only, dart wires cut from the new list, a magnet for the
  bench test, base fit coupon in FIRST_DAY and order, hole compensation 0 for base parts, coupon command path from
  base/, C21 exactly as the sheets list, WIRING section numbers, flashing with wiring left on, step 11 starts at
  stage 4. `grappstar base fire` is on w/jetson (merge pending). Sent: cad-infra (coupon commands, magnet cap),
  fw-base (FORWARD on the bench, README section ref).
