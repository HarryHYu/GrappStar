# fw-base report (firmware/base)

Worker `fw-base`, 3 Nov 2026, branch `w/fw-base`. Firmware string `base-1.3.1`, params `1.3.8` (params_gen.h regenerated from
spec 1.3.8 at 3a32af7). Nothing here has run on hardware: every claim below is from the PC simulation and the arduino-cli
compile (the base-1.3.1 changes were checked with the PC tests only: no arduino-cli in the final cross-check session).

Status (final cross-check, 3 Nov, base-1.3.1): the firmware defaults now match the contacts as built and the PC sim
models those contacts from the CAD, not from the firmware's own constants (section "Final cross-check" below). All
PC tests and both fuzz runs pass.

## Done

- `grappstar_base/`: Arduino sketch for the ESP32-S3 DevKitC-1-N8. All logic is plain C++ in headers, tested on a PC.
  - `base_logic.h`: state machine, fault manager, drum mover, aim, cock, fire, decock, wrap, unwrap, tidy, retrieve,
    reel and tow, jog, cal, set/get, telemetry, events, power-loss recovery, D12 no-magnet index mode.
  - `config.h` (defaults; spec numbers from `params_gen.h`), `pins.h`, `drives.h` (BTS7960 soft start, reverse gap, IS
    scaled by duty, trips, stall; DRV8833 brake), `sensors.h` (debounced contacts with chatter detection and edge
    timing; AS5600 multi-turn tracker), `persist.h` (NVS records with CRC), `json_lite.h`.
  - `grappstar_base.ino`: USB CDC JSON lines, 5 ms tick task on core 1 with a 300 ms task watchdog, flash writes in
    loop() with a start gate (a motor starts only after the "moving" record is written), IS sampled across the PWM
    period, HX711 auto detect.
- `tools/gen_params.py`: `base/spec/params.py` to `grappstar_base/params_gen.h`. Drum parks, wraps, capstan, tow
  limit, current trips, tension model, drum speed and the 37-point aim table all come from params.
- `test/`: `sim.h` (physics sim: wiper DC motor with optional inertia and coast, IS output, AS5600 with dropouts and
  glitches, drum Gray code with edge offsets, dog window and tooth-on-tooth, launch train with release, take-up and
  capstan line model, HX711, battery, fault injection, dry-fire detector; SW_GAP, SW_RAIL_LOW and SW_SEATED as the CAD
  builds them), `test_base.cpp` (470 checks),
  `test_fuzz.cpp` (random commands and faults against invariants), `test_gen_params.py`, `run_tests.sh`.
- Docs: `PROTOCOL.md` (contract C1, BASE_SPEC 10.1 sequence, section 10 lists every change for the Jetson),
  `README.md` (exact Arduino IDE settings, core and library versions), `WIRING.md` (every step starts "Battery
  unplugged."), `CALIBRATION.md`, `BENCH_TEST.md`.

## Decisions made (Harry was away)

1. Mechanism: spec v1.2 / params 1.2.6 (drum as cad-drum built it: LAUNCH 0, NEUTRAL 41, WRAP 62..302 with 7 cup
   turns, WRAPPED park 314, REEL 347; D22 / D35 sequence).
2. Fault response: wiper off at once; drum to NEUTRAL, except it HOLDS (MODE `DRUM_HELD` with the reason) with the
   wraps on (params fault_rules, D27), with the spring possibly drawn, with a bad drum code or a drum fault.
3. Power cut mid-cock: the record holds the start angle, so boot assumes the spring is drawn (cock refused,
   `decock` with `force` if SW_HOME lies). Safer than trusting a contact.
4. A drum code change with the drum still near the wrap window makes the wrap count doubtful: only `unwrap` (with
   the take-up) until NEUTRAL.
5. Tow (D33 / D34 / D39 / D43): the limit is the dart-side tension, at most 55 N (D46); estimate 13.1 x (I - I_free) - 2.3;
   trip = (max_n + 2.3) / 13.1 + I_free + 1 A (at least 5 A); the tow starts at the minimum duty and speeds up only
   while the line is slack; it holds on the 200 ms average OR on the raw estimate held 30 ms at the limit; a re-sent
   tow at the limit stays in HOLD (stop + reel to wind again); constants cannot change while moving or holding.
6. Fly-in at 25 percent duty (asked about 20) so the 3 A trip reads real current (IS scaling needs duty 60 or more).
7. Final approach time limit: max(20 s, 10 s + 1.5 x line_m / slowest take-up speed at the duty), 30 s with line_m
   unknown, `max_s` 5..600. The coordinator asked 5..180; physics shows 20 m at duty 0.3 needs about 500 s.
8. Reel-in trip stays 5 A (D33) although physics suggested 6 A; one config line if Harry agrees.
9. `retrieve` (and `reel` until `seated`) runs only in the NEUTRAL sector (D39); anywhere else, the WRAPPED park
   included, it is refused with `WRAPS_ON`.
10. Backward runs in LAUNCH trip at 3 A (D40); after an aim or decock a slow forward gap run closes SW_GAP (big gear
    drag, cad-launch); SW_HOME ignored 0.5 s after the release (D45).
11. Index mode (D12): cock stops a bump early, a missed bump faults within 1.5 bump periods, every aim re-homes on
    SW_RAIL_LOW, aim tolerance at least 2 deg.

## Verified

(base-1.3.1, final cross-check, 3 Nov, from firmware/base/test: `./run_tests.sh`; summary lines)

```
== params generator
19 checks, 19 passed, 0 failed: ALL PASS
== firmware logic (simulation)
470 checks, 470 passed, 0 failed: ALL PASS

fuzz: 300 seeds (1..300), 300 steps each, 27.9 h simulated, 928 shots in 241 seeds
fuzz: no invariant violated
== fuzz with faults during motion, held jogs and the tow limit check

fuzz: 200 seeds (5001..5200), 400 steps each, 19.5 h simulated, 227 shots in 116 seeds
fuzz: no invariant violated
```

Compile of base-1.3.0 (`arduino-cli`, esp32 core 3.3.12, fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc, HX711 by Rob Tillaart
0.6.6; not repeated for base-1.3.1, whose changes are in the headers the PC tests compile):

```
Sketch uses 407049 bytes (31%) of program storage space. Maximum is 1310720 bytes.
Global variables use 35600 bytes (10%) of dynamic memory, leaving 292080 bytes for local variables. Maximum is 327680 bytes.
```

## Open items

- Known gap: SW_HOME stuck closed AND the saved angle lost AND the train about 300 deg into a cock: a recovery
  `cock` can still reach the release (its reset allowance of 157 deg overlaps). Every other combination is refused.
  The test starts this case at 270 deg.
- Tow on a very stiff line: the slow start keeps the simulated overshoot under about 10 percent (systest's 5.6 kN/m
  Dyneema plant: limit respected); measure on the bench (CALIBRATION f).
- `cal cock_rest` with `force:true` trusts the operator: on a drawn spring the next forward jog or cock can fire.
- Two contacts failing together (a drum code bit AND SW_ARM, or SW_HOME AND SW_GAP with the record lost) defeat the
  cross-checks; single contact failures are caught (fuzzed with faults injected during motion).
- The sim cannot produce a dog friction-locked under load, contact bounce bursts, VBAT ripple or AS5600 nonlinearity
  (review notes in this report's history); bench tests in BENCH_TEST.md cover them by hand.
- Spec text that disagrees with params / decisions (reported to the coordinator): BASE_SPEC 30 deg unload jog
  (params 1.3.3 now 6, firmware uses it), take-up "about 30 percent" (firmware 40, I4 cap), D31 309 / D32 -6 (superseded by 314 / -10.4).
- systest F31 (fixed after the final push, 19:45): in D12 index mode the estimated coast after each reset grew the
  launch phase about 11 deg at REST, so `mode NEUTRAL` was LAUNCH_BUSY after every shot. The coast estimate no longer
  moves the phase at REST; regression in the D12 index test.
- Still over at 18ab2b2 (systest): ref_tow_stiff 50.4 N and tow_stiff 46.2 N dart side vs the 46.0 bound on the
  stiff 3 m line; ref_nominal passes (38.6 N). Next step if Harry wants it: hold small and stiff-line limits earlier.
- Pending at the final push: systest re-runs of ref_tow_stiff (F21, 46.2 N vs 46.0 bound before the taper fix) and
  F30 (15 N check pull held at 20.5 N) on the taper fix 6dd2cf3. If either still overshoots, hold small limits earlier.
- Review round 8 (drum moves and boot; protocol and sketch glue) was started but its agents stopped on an API limit
  before reporting: those two areas were last reviewed in rounds 1 to 6 only.

## Review rounds (independent agents; every finding proved with a probe, fixed with a regression test)

- Docs cross-check (13 code findings: aim trip, cock after a shift, two clears, silent decock, mode ok without collar, cock and SW_SEATED, decock and SW_GAP, ...): 256a71c.
- Drum and mission safety (drum into WRAP after a cut, jog dog checks, fly-in current scaling, D34 tow trip): fe22a96.
- Protocol, persistence, glue (estop with a bad id, key slots, status length, cal ranges, flash wear): 103eb0d.
- Launch side (index-mode missed bump fired, stop dropped the drawn hint, decock to REST, index aim): b352cbc.
- Sim fidelity (AS5600 dropouts lost cock travel and fired; IS sampling; inertia added to the sim): 7b7c367, 629ce02.
- PROTOCOL.md line by line against the code (18 doc mismatches, UNWRAPPED without MODE): 3ff1a86, 3ed2044.
- Adversarial hunt (cock jog with a failing collar contact fired; tow overshoot on stiff lines and ratchet from HOLD; frozen load cell; constants changed mid-tow; lying drum codes unwinding without take-up; tip-to-tip centering into sprag #2): 4bb87cc, 8353486.
- Earlier review reports r1_D and r1_G, re-checked item by item:

| report | # | finding (short) | status | evidence | commit |
|---|---|---|---|---|---|
| r1_D | 1 | PROTOCOL old drum order, 4 wraps, NOT_IN_REEL, no slip | FIXED | PROTOCOL.md sections 2 and 5 (wrap to WRAPPED 314, 7 turns, mode REEL only from N_TO_R) | 8f41b6f, ee60e0c |
| r1_D | 2 | Tow law 8 N/A, fixed 5 A trip ends every tow in OVERCURRENT | FIXED | config.h tension model 17 / -10.4 from params; towTrip() max(6.5 A, need + 1 A); 200 ms average; default-constant tow probe holds at 50 N | 01f1fe0, fe22a96 |
| r1_D | 3 | SW_DOG_L/R not checked during drum moves | FIXED | tickDrumMove stages 0/1 and drum jogs check both | 01f1fe0, fe22a96 |
| r1_D | 4 | No WRAPPED park; tidy only in NEUTRAL | FIXED | targetDeg WRAPPED 314; wrap parks there; tidy also at the park below 318 | 01f1fe0 |
| r1_D | 5 | pins.h contradicts params.SWITCHES | FIXED | pins.h matches params SWITCHES; SLP not wired | 55dd476 |
| r1_D | 6 | Fault in REEL/N_TO_R holds instead of going to WRAPPED | NOT APPLICABLE | params 1.2.5 fault_rules: the drum HOLDS with the wraps on; firmware holds | 7b7c367 |
| r1_D | 7 | Same as #6 for estop | NOT APPLICABLE | as #6 | 7b7c367 |
| r1_D | 8 | aim inherits the previous trip | FIXED | homing leg and planAim set tripA 5 A; probe 5.00 A through S_AIM | 01f1fe0 |
| r1_D | 9 | Drum jog end-stop guard: untrusted estimate, wrong end stops | FIXED (see below) | guard at end stops -4/352 with 2 deg margin; refused when the angle is not trusted | 3ed2044 |
| r1_D | 10 | No SW_GAP check: fire/rest/decock | FIXED | fire/arm, decock end, reset end; cock from REST and cal cock_rest now also need SW_GAP | 55dd476, 256a71c, 3ed2044 |
| r1_D | 11 | config bit map vs pin table | FIXED | swBit = pins.h = WIRING = BENCH | 55dd476, 8f41b6f |
| r1_D | 12 | Tow limit not averaged 200 ms | FIXED | tensionAvgMs 200 | a453ed3 |
| r1_D | 13 | SW_GAP not checked by fire, arm, decock | FIXED | as #10 | 55dd476, 256a71c |
| r1_D | 14 | Wiper backward jog allowed outside LAUNCH | FIXED | cmdJog: BACKSTOP outside LAUNCH for backward | 3ed2044 |
| r1_D | 15 | Drum jog through WRAP unwraps without take-up | FIXED | WRAPS_ON in WRAP; running jog stops at the WRAP edge | fe22a96, b352cbc |
| r1_D | 16 | wrapsFull = 4 | FIXED | P_DRUM_WRAP_TURNS 7 | 55dd476, fe22a96 |
| r1_D | 17 | Current not scaled below duty 60 | FIXED | all helper duties >= 60 (fly-in 64); IS sampled across the PWM period | fe22a96, 629ce02 |
| r1_G | 1 | BENCH contact table wrong GPIOs, D key | FIXED | BENCH_TEST contact table = SW_TEL keys and pins.h | 8f41b6f |
| r1_G | 2 | BENCH step 2 drum walk cannot pass | FIXED | walk order valid for the Gray codes; one clear after the walk | 256a71c, 8f41b6f, fe22a96 |
| r1_G | 3 | Old single SW_DOG layout | FIXED | 12 contacts in swBit order | 8f41b6f |
| r1_G | 4 | Same plus PROTOCOL sw keys | FIXED | key table = SW_TEL | 8f41b6f |
| r1_G | 11 | Walk faults on row 1, reboot needed | FIXED | as #2 | 256a71c, fe22a96 |
| r1_G | 12 | CALIBRATION mode REEL from NEUTRAL | FIXED | wrap, then mode REEL | 8f41b6f, fe22a96 |
| r1_G | 13 | k_n_per_a by jog against the 3 A jog trip | FIXED | reel against a luggage scale (5 A / tow trips) | fe22a96 |
| r1_G | 14 | WIRING SLP on GPIO21 | FIXED | SLP unconnected, 3V3 fallback | 8f41b6f |
| r1_G | 15 | SLP contradictions | FIXED | as #14 | 8f41b6f |
| r1_G | 16 | No 2 A buck fuse | FIXED | WIRING resistor/fuse table and Part B | 8f41b6f, ee60e0c |
| r1_G | 17 | Same as #16 | FIXED | as #16 | 8f41b6f, ee60e0c |
| r1_G | 18 | aim_home_deg range note | FIXED | -7200..7200 | 8f41b6f |
| r1_G | 19 | README trips 17/15 A | FIXED | README trip list = config | ee60e0c, d11549f |
| r1_G | 20 | WIRING says buy an MP1584 | FIXED | MP1584 #2 is the dart spare | ee60e0c |

- Review round 2 D2 (26 findings at ba121b9), re-checked item by item:

| # | sev | finding (short) | status | evidence / commit |
|---|---|---|---|---|
| 1 | high | k_n_per_a calibration ignores the offset; tension_offset_n not saved | FIXED | two-pull fit in CALIBRATION; Tun ver 2 / 256a71c, 3ed2044 |
| 2 | high | PROTOCOL old drum sequence | FIXED | 8a0ea41, 3ed2044 |
| 3 | high | retrieve ignores max_s | FIXED | d11549f, 7b7c367 |
| 4 | high | same, take-up final approach | FIXED | d11549f |
| 5 | medium | PROTOCOL old trips / tension law | FIXED | 77f0ab9, D40/D42 docs in the 1.2.9 docs commit |
| 6 | medium | fault in REEL drives out to 314 | FIXED (hold, params fault_rules) | 7b7c367 |
| 7 | medium | same for N_TO_R | FIXED | 7b7c367 |
| 8 | medium | line_m after a cut mid-reel from the motion-start record | FIXED | c28396c + b2f37ee (also a cut mid-fire) |
| 9 | medium | AS5600 multi-turn count not lined up at boot | FIXED | c28396c (first read absolute) + b2f37ee (resync keeps alignment) |
| 10 | medium | tow retry from HOLD over the limit | FIXED | 4bb87cc |
| 11 | medium | drum code back after 000 trusted at the sector end | FIXED | b2f37ee (boot path, park check, in-sector re-reference) |
| 12 | medium | boot in N_TO_R guessed 321 | FIXED | c28396c + b2f37ee (re-reference down to the WRAP edge with take-up) |
| 13 | medium | retrieve in REEL / park | FIXED | 7418b0e, 3ff1a86 |
| 14 | medium | tow trip fixed 6.5 A | FIXED | fe22a96, 4bb87cc |
| 15 | medium | cut during wiper motion, decock no-op | FIXED | b352cbc, 3ff1a86 |
| 16 | medium | set trip_a mid-motion | FIXED | 4bb87cc |
| 17 | medium | drum jog skips dog check / enters WRAP | FIXED | fe22a96 |
| 18 | medium | IS scaling without the capacitor | FIXED | 629ce02 (16 paced reads) |
| 19 | low | require_seated / vbat_wired persist | FIXED | c28396c + b2f37ee |
| 20 | low | retrieve in REEL seats on the capstan | FIXED | 7418b0e |
| 21 | low | drum jog back from the park unwraps | FIXED | fe22a96, 4bb87cc |
| 22 | low | tension_offset_n not saved | FIXED | 256a71c |
| 23 | low | decock RESET without SW_GAP | FIXED | 256a71c |
| 24 | low | same | FIXED | 256a71c |
| 25 | low | 3 A trip blind at 20 % fly-in | FIXED | fe22a96 (fly-in 25 %), D41 rework c28396c |
| 26 | low | current unscaled below duty 60 | FIXED | b2f37ee (floored divisor) |

- Nuisance-fault review (checks likely to false-trip on real contacts): 89872c8.
- systest ref_nominal on its stiff Dyneema plant: D41 fly-in and the tow limit pass since 87c48b3; the two remaining failures are Jetson-side (line_m set, command order).
- Round 7, tow path (the taper past 70 % of max_n set the duty back to about 0.95 on a taut line, which explains the
  systest first-pull overshoot; fly-in limit blind when the drum code is not REEL): 6dd2cf3.
- Round 7, launch side (a fault or estop with SW_GAP open shifted the drum out of LAUNCH; mode / cock / decock that
  moved nothing kept the arm token; decock could not recover a stopped gap run): 6153e8f, docs 0f2fada.

## What Harry must measure (bench, then CALIBRATION.md)

1. Wiper direction: which way RPWM turns the shaft, and that FORWARD cocks (CALIBRATION g); AS5600 `enc_sign`.
2. Wiper current: IS scale `amps_per_volt` at low duty with a meter in series, `offset_a` with the motor off, free current
   `free_a` while reeling slack (`cal free`).
3. Tension: `k_n_per_a` and `tension_offset_n` with a luggage scale at the line end (dart side, D43/D46) at 3 loads
   (defaults 13.1 and -2.3 are physics estimates; tow cap 55 N). Hard prerequisite of the first tow. Then
   `tension_cal` reads true.
4. Drum speed (`drum_dps`, learned automatically between code edges; default 8.1 deg/s) and the end stops.
5. Aim: `aim_home_deg` (SW_RAIL_LOW position, default 3502 from the CAD contact) and `aim_offset` against a phone
   inclinometer on the rail (CALIBRATION e).
6. Launch train angles: SW_HOME opening (first tooth, about 4 deg), release (432.7), SW_GAP closing (`gap_close_deg`,
   about 531: CALIBRATION h reads `gap_deg` from the `COCKED` event); compare telemetry `cock` with what you see.
7. Spring rate and preload (hang test), battery sag under the cock peak, AS5600 magnet (status "magnet":"OK").
8. DRV8833 SLP: if the drum does nothing, battery unplugged, SLP to 3V3 (never VM).
9. Take-up line speed at duty 0.3 (physics 0.058 to 0.073 m/s) for the retrieve time limit.

## Requests to other workers

- Coordinator / spec: DECISIONS D31 still says faults go to WRAPPED (params says hold); BASE_SPEC 10.1 step 7 sends a
  no-go from LAUNCH straight to retrieve (needs `mode NEUTRAL` first); params SWITCHES.SW_ARM says "closed at home"
  but cad-reel says it opens at home (firmware follows cad-reel); physics suggested a 6 A reel-in trip (firmware
  keeps 5 A per D33).
- Jetson: follow PROTOCOL.md section 10 (80 items); test against w/fw-base, not the older base-1.2 snapshot.

## Final cross-check (3 Nov, base-1.3.1)

A read-only cross-check against the CAD found defaults that the PC tests could not see, because `test/sim.h` drew the
contacts from the firmware's own constants. Fixed, each with a regression test in `test_base.cpp`
("contacts as built", the timeline and the fire refusals):

| finding | fix |
|---|---|
| fw-1 / cad_support-2: `gapCloseDeg` 520, the CAD SW_GAP closes about 531: every reset ran into the first tooth (BAD_STATE, no RESET) | default 531; new saved `set` key `gap_close_deg` (480..553, BUSY while moving, `cal factory`); `COCKED` reports `gap_deg` (where SW_GAP opened on the draw) and CALIBRATION h turns it into `gap_close_deg`; bench record (Tun) version 3 |
| fw-2: the gap run allowed 150 deg, the CAD drag is 127 big-gear deg (195 wiper deg) | 215 deg; the tests drag 195.4 wiper deg |
| fw-3 / cad_support-6: `aimHomeDeg` 3365, the CAD SW_RAIL_LOW closes at about 3502 (aims 3 to 6 deg high) | default 3502 |
| cad_support-3: SW_SEATED is pressed only with the carriage at HOME, but arm and fire after `cock` needed it closed | latch: the reading taken as the cock starts counts while COCKED (cleared by a fire, a decock, any fault and a restart) |
| cad_support-9: the edge table 8/32/340 against the bumps as built 6/34/342 | `edgeDeg` from params DRUM `code_edges` through `params_gen.h` |
| fw-8: params_gen.h from 1.3.4, stale fallbacks, unchecked names | regenerated (1.3.8); fallbacks 13.1 / -2.3 / 55 N and `P_DRUM_UNLOAD_JOG_DEG`; `gen_params.py` checks every name the firmware reads (test fails on a missing one) |
| fw-4 to fw-7, fw-9, fw-11, docs-16 | PROTOCOL (tow trip 6.5 A, fault in REEL holds, D45 rail-low step, SW_ARM examples, versions, retrieve default as BASE_SPEC 10.1 step 17), this report's decision 9, README mission summary |

`test/sim.h` now models SW_GAP (closed within 22.9 wiper deg of rest), SW_RAIL_LOW (within 97.6 aim deg of the low dead
point) and SW_SEATED (closed only with the carriage home and the dart seated) from the CAD, and a test proves that the
old defaults fail against it.
