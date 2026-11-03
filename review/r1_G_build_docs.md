# Round 1 G: build documentation (learner following cold)

Reviewed at base-v1 commit 0bfd59a (tree at 12:20). Method: independent finder subagents, one per lens; every finding of
medium severity or above was checked by two skeptic subagents told to refute it; kept only if fewer than
two refuted it. Low findings are listed unverified. Severity is the finder's, lowered when both skeptics
agreed it is real but rated it lower.

Confirmed: 21. Refuted and dropped: 1.

| # | Sev | File:line | Owner | Finding |
|---|---|---|---|---|
| 1 | high | `firmware/base/BENCH_TEST.md:48` | docs | BENCH_TEST contact table lists the wrong GPIOs and a telemetry key that does not exist |
| 2 | high | `firmware/base/BENCH_TEST.md:59` | fw-base | BENCH_TEST step 2 drum-code walk cannot pass: first row change faults ('drum code changed with the drum still'), row 7 jumps sectors, and rows 7 to 9 give the wrong drum_pos/mode |
| 3 | high | `firmware/base/WIRING.md:27` | docs | Contact pin table in WIRING.md, BENCH_TEST.md, Build Guide and CALIBRATION.md uses the old SW_DOG layout (D27). Firmware swBit follows D31 (SW_DOG_L/SW_DOG_R), so every contact from GPIO14 on is wired to the wrong logical switch |
| 4 | high | `firmware/base/WIRING.md:27` | fw-base | Contact pin table in WIRING.md / BENCH_TEST.md / PROTOCOL telemetry keys is from the old firmware (one SW_DOG); the current firmware has SW_DOG_L + SW_DOG_R, so every launcher contact from GPIO39 on lands on the wrong input |
| 5 | medium | `base/spec/params.py:236` | docs | params.SWITCHES claims to match pins.h, but every GPIO differs, including the DRV8833 and HX711 pins swapped and SW_HOME on GPIO21, which the firmware drives as an output |
| 6 | medium | `docs/build/MEASURE_LIST.md:34` | docs | MEASURE_LIST B4 gives the superseded 280 mm full-cock extension (and 35 mm preload); D30/params now say 265 mm and 23.8 mm because 280 mm overloads the spring |
| 7 | medium | `docs/build/TEST_PLAN.md:41` | docs | Stage 1 pass band for wiper free current (1 to 3 A) overlaps the jog trips (2.5 A forward, 3 A backward in NEUTRAL) |
| 8 | medium | `docs/build/TEST_PLAN.md:64` | docs | Mode sequences in TEST_PLAN and CALIBRATION send `mode REEL` from NEUTRAL/LAUNCH, which the D16 firmware always refuses |
| 9 | medium | `docs/build/TEST_PLAN.md:67` | docs | Dry-fire stages 3 and 4 cannot be done as written: no dart means NOT_SEATED, and the CLI cannot send fire within the 3 s token life |
| 10 | medium | `docs/build/TEST_PLAN.md:70` | docs | Stage 3 estop test expects the drum to go to NEUTRAL after every move, but the firmware deliberately holds it during cock/fire and with wraps on |
| 11 | medium | `firmware/base/BENCH_TEST.md:58` | docs | BENCH_TEST step 2 drum-code walk faults on the very first row and can lock the base in a DRUM_SENSOR fault that only a reboot clears |
| 12 | medium | `firmware/base/CALIBRATION.md:96` | fw-base | Calibration and test plan send `mode REEL` from NEUTRAL, which the firmware always refuses (NOT_WRAPPED); REEL to NEUTRAL is refused too (WRAPS_ON) |
| 13 | medium | `firmware/base/CALIBRATION.md:163` | docs | CALIBRATION (e) k_n_per_a: jogging to 20 N needs about 4 A, but a jog trips OVERCURRENT at 3 A |
| 14 | medium | `firmware/base/WIRING.md:165` | docs | WIRING.md and the Build Guide wire DRV8833 SLP to GPIO21, against D9 and SYSTEM_ELECTRICAL. TEST_PLAN then adds SLP to 3V3, which shorts 3V3 into the GPIO21 output the firmware drives LOW |
| 15 | medium | `firmware/base/WIRING.md:165` | fw-base | DRV8833 SLP wiring contradicts itself across docs: WIRING says wire SLP to GPIO21, SYSTEM_ELECTRICAL/D9/pins.h say leave it unwired, TEST_PLAN says wire it to 3V3 |
| 16 | medium | `firmware/base/WIRING.md:168` | docs | WIRING.md wires the 22 AWG MP1584 input straight to the B+ screw behind only the 20 A fuse, without the 2 A inline fuse SYSTEM_ELECTRICAL requires |
| 17 | medium | `firmware/base/WIRING.md:168` | fw-base | WIRING.md wires the 22 AWG buck input straight off the B+ screw behind only the 20 A fuse, with no 2 A inline fuse; SYSTEM_ELECTRICAL requires the 2 A fuse |
| 18 | low | `firmware/base/CALIBRATION.md:151` | docs | CALIBRATION note warns that aim_home_deg can only be set from -720 to 720; the firmware accepts -7200..7200 |
| 19 | low | `firmware/base/README.md:144` | fw-base | Firmware README safety summary gives the wrong current trips (17 A / 15 A while cocking); the firmware trips at 5 A, 6.5 A while cocking, 3 A on jogs, hard 17 A |
| 20 | low | `firmware/base/WIRING.md:64` | fw-base | WIRING.md tells the learner to buy an MP1584, which breaks the parts rule and contradicts SYSTEM_ELECTRICAL/MEASURE_LIST (MP1584 #2 is freed by the v4 dart) |
| 21 | low | `firmware/dart/WIRING.md:97` | fw-dart | Dart WIRING step B2 connects the 3S LiPo to the MP1584 without starting from 'battery unplugged' or saying to make the connection before plugging in |

## 1. BENCH_TEST contact table lists the wrong GPIOs and a telemetry key that does not exist

- File: `firmware/base/BENCH_TEST.md:48`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: docs

Scenario: The firmware maps contacts with swBit = {0..8}. That puts SW_DOG_L on GPIO14 (key `l`), SW_DOG_R on GPIO39 (`r`), SW_SEATED on GPIO40 (`S`), SW_HOME on GPIO41 (`H`), SW_RAIL_LOW on GPIO42 (`L`) and SW_WRAP_HOME on GPIO10 (`W`). If the learner closes the contact on GPIO39 as the table says, they see `"r":1`, not `"S":1`. Closing GPIO14 gives `"l":1` (no `D` key exists). Every launcher contact after that is off by one pin. This step is the check meant to catch wiring errors, but it confirms the stale pinout that WIRING.md:116-120 shares. If they "fix" the wiring to match the doc, SW_SEATED ends up on the SW_DOG_R input: with the drum in LAUNCH, seating a dart then makes collarLaunch() false and launch actions are refused or fault (SWITCH_DISAGREE when both dog inputs are closed).

Evidence: BENCH_TEST.md:48-52 "| `SW_DOG` (GPIO14) | `"D":1` | ... | `SW_SEATED` (GPIO39) | `"S":1` | `SW_HOME` (GPIO40) ... `SW_RAIL_LOW` (GPIO41) ... `SW_WRAP_HOME` (GPIO42)"; config.h:14 `enum SwId { SW_DRUM_A, SW_DRUM_B, SW_DRUM_C, SW_DOG_L, SW_DOG_R, SW_SEATED, SW_HOME, SW_RAIL_LOW, SW_WRAP_HOME, SW_INDEX ...}`; config.h:17 `SW_TEL = {"a","b","c","l","r","S","H","L","W","I"}`; config.h:37 `swBit = {0,1,2,3,4,5,6,7,8,-1}`; pins.h:32-37 PIN_SW_D 14, E 39, F 40, G 41, H 42, I 10.

Fix: Replace the table rows with: SW_DOG_L GPIO14 -> "l":1; SW_DOG_R GPIO39 -> "r":1; SW_SEATED GPIO40 -> "S":1; SW_HOME GPIO41 -> "H":1; SW_RAIL_LOW GPIO42 -> "L":1; SW_WRAP_HOME GPIO10 -> "W":1. Add: closing l and r together raises SWITCH_DISAGREE after 0.2 s. Fix the same table in WIRING.md sections 5 and 6, and in the build guide copy.

> Skeptic (confirmed): I could not refute this. It is a real defect, and it affects more files than the claim names.

**What the code does**
- `config.h:14` lists the contacts in this order: SW_DRUM_A, B, C, SW_DOG_L, SW_DOG_R, SW_SEATED, SW_HOME, SW_RAIL_LOW, SW_WRAP_HOME, SW_INDEX.
- `config.h:17` gives their telemetry keys: a, b, c, l, r, S, H, L, W, I.
- `config.h:37` sets `swBit={0..8,-1}`. Nothing in the .ino overrides this.
- `grappstar_base.ino:28` maps bit i to `PIN_SW_A+i`. From `pins.h`, the pins are 11, 12, 13, 14, 39, 40, 41, 42, 10.
- Result: SW_DOG_L is GPIO14 (key `l`), SW_DOG_R is GPIO39 (`r`), SW_S
>
> Skeptic (confirmed): I could not refute this. The mismatch the claim describes is in the code, though the doc may not be the side that is wrong.

What the code does:
- grappstar_base.ino:28 and :88: hwSwitches() sets bit i when SW_PINS[i] reads LOW. SW_PINS is PIN_SW_A..J, which pins.h:29-38 puts on GPIO 11, 12, 13, 14, 39, 40, 41, 42, 10, 35.
- config.h:37: swBit = {0,1,2,3,4,5,6,7,8,-1}, in the SwId order of config.h:14 (DRUM_A, DRUM_B, DRUM_C, DOG_L, DOG_R, SEATED, HOME, RAIL_LOW, WRAP_HOME, INDEX).
- base_logic.h:1011 writes the key SW_TEL[i] (config.h:17) for each closed contact.
- No code changes swBit at ru
>

## 2. BENCH_TEST step 2 drum-code walk cannot pass: first row change faults ('drum code changed with the drum still'), row 7 jumps sectors, and rows 7 to 9 give the wrong drum_pos/mode

- File: `firmware/base/BENCH_TEST.md:59`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: The learner follows the drum code table by hand with the N20 stopped. Going from row 1 (b) to row 2 (a b) raises FAULT DRUM_SENSOR "drum code changed with the drum still" 300 ms later. This comes from base_logic.h:413 and was added in fw-base commit a6e2394, after the docs were written. The doc says a fault happens only when you skip a row, and that the fix is to go back to row 1 and clear, so the learner loops forever. Under the current secCode {LAUNCH 0x1, LN 0x3, NEUTRAL 0x2, PREP 0x6, WRAP 0x4, LOCK 0x5, REEL 0x7}, row 6 (b c = PREP) to row 7 (a b c = REEL) also jumps three sectors. Row 8 (a c) is LOCK, which reports drum_pos WRAPPED, not 'BETWEEN'. Row 9 (c) is WRAP, which reports BETWEEN / mode NEUTRAL, not 'WRAPPED / REEL'. Rows 3 and 7 report mode NONE, not LAUNCH/REEL, unless SW_DOG_L/SW_DOG_R are closed. My scratch run (/tmp/claude-0/review-scratch-G/bench.cpp) printed: row 2 FAULT "drum code changed with the drum still", row 3 mode=NONE, rows 7 and 8 stuck at sec=3. A corrected order with a clear after each row (bench2.cpp) passes cleanly.

Evidence: BENCH_TEST.md:61 "| 2 | `a` `b` | `BETWEEN` (LAUNCH side) | `NONE` |", :66 "| 7 | `a` `b` `c` | `REEL` | `REEL` |", :67 "| 8 | `a` `c` | `BETWEEN` (laying wraps) | `REEL` |", :68 "| 9 | `c` | `WRAPPED` | `REEL` |", :74 "A `FAULT` `DRUM_SENSOR` while you do this is expected if you skip a row". base_logic.h:413 "if (!d.running() && now - d.stopT > 300 && booted && quietSec != SEC_INVALID && state != S_JOG) { quietSec = sec; raise(F_DRUM_SENSOR, \"drum code changed with the drum still\"); return; }"; config.h:99 "uint8_t secCode[SEC_COUNT] = {0x1, 0x3, 0x2, 0x6, 0x4, 0x5, 0x7};"; base_logic.h:146 "case SEC_LOCK: return DP_WRAPPED;". Copied into GrappStar_Build_Guide.md:2100-2107.

Fix: Replace the table with the current travel order and the real readings: b NEUTRAL/NEUTRAL; a b BETWEEN/NONE; a LAUNCH/NONE (LAUNCH only with SW_DOG_L closed); back to a b, then b; b c BETWEEN/NEUTRAL; c BETWEEN/NEUTRAL; a c WRAPPED/NONE; a b c REEL/NONE (REEL only with SW_DOG_R closed); then walk back. Say plainly that every hand change of the code gives FAULT DRUM_SENSOR "drum code changed with the drum still" (the drum cannot move by itself), and that this is expected: after each row send {"id":2,"cmd":"clear"} and check drum_pos. Regenerate the build guide.

> Skeptic (confirmed): I could not refute this. I checked each part against the code on w/review.

1) Every hand change faults. In base_logic.h:411-414, watch() raises F_DRUM_SENSOR "drum code changed with the drum still" whenever sec != quietSec and all of these hold: the N20 is off for over 300 ms, booted, quietSec != SEC_INVALID, and state != S_JOG. Changing the code by hand on the bench meets every one. Row 1 to row 2 (b, 0x2 NEUTRAL, to a b, 0x3 LN) is a legal adjacent step: updateSector at :367 accepts it, so sec changes and the fault fires 300 ms later. The doc (BENCH_TEST.md:74-76) says DRUM_SENSOR happens o
>
> Skeptic (confirmed): I could not refute this. I checked it against the code in firmware/base/grappstar_base/ (the claim cites config.h and base_logic.h without that subfolder, but the lines match).
(1) The fault on the first row change is real. watch() in base_logic.h, around line 413, has: `if (sec != quietSec) { if (!d.running() && now - d.stopT > 300 && booted && quietSec != SEC_INVALID && state != S_JOG) { ... raise(F_DRUM_SENSOR, "drum code changed with the drum still") } }`. quietSec is set to sec on the first pass after boot. With the N20 stopped and the board idle, every hand change of the code is therefor
>

## 3. Contact pin table in WIRING.md, BENCH_TEST.md, Build Guide and CALIBRATION.md uses the old SW_DOG layout (D27). Firmware swBit follows D31 (SW_DOG_L/SW_DOG_R), so every contact from GPIO14 on is wired to the wrong logical switch

- File: `firmware/base/WIRING.md:27`
- Severity: high (claimed critical); 2 of 2 skeptics confirmed
- Owner: docs

Scenario: A learner wires the contacts exactly as WIRING.md section 1/5 says: SW_DOG on GPIO14, SW_SEATED on GPIO39, SW_HOME on GPIO40, SW_RAIL_LOW on GPIO41, SW_WRAP_HOME on GPIO42. In config.h, swBit={0..8,-1} maps PIN_SW_D..I (GPIO14,39,40,41,42,10) to SW_DOG_L, SW_DOG_R, SW_SEATED, SW_HOME, SW_RAIL_LOW, SW_WRAP_HOME, so the firmware reads every contact as a different one. Results: (1) The firmware's SW_HOME, the cock/fire/decock safety gate, is fed by the rail-low cam on GPIO41. It reads 'carriage home' whenever the rail is low, whatever the spring state, and 'drawn' once the rail lifts. (2) The fire interlock reads SW_SEATED from the carriage-home contact (GPIO40). That contact is open whenever the spring is cocked, so fire always returns NOT_SEATED and a normal run can never fire. (3) The tip-to-tip SW_DOG contact (normally open) sits where the firmware expects SW_DOG_L closed when engaged, so collarLaunch() is never true. (4) The seated dart closes 'SW_DOG_R' (GPIO39), so maybeReel()/BACKSTOP and 'SW_DOG_L and SW_DOG_R both closed' SWITCH_DISAGREE faults follow. (5) SW_WRAP_HOME (firmware GPIO10) is never wired. BENCH_TEST step 2 tells the learner to look for telemetry key "D", which the firmware never sends (SW_TEL = a b c l r S H L W I), so the bench check cannot catch the mix-up. CALIBRATION step 185 'Check H is in telemetry sw' with the carriage forward also fails, because the doc-wired home contact reports as S.

Evidence: WIRING.md:27-31 '| 14 | `PIN_SW_D` | contact `SW_DOG` ... | 39 | `PIN_SW_E` | contact `SW_SEATED` ... | 40 | `PIN_SW_F` | contact `SW_HOME` ... | 41 | `PIN_SW_G` | contact `SW_RAIL_LOW` ... | 42 | `PIN_SW_H` | contact `SW_WRAP_HOME`'; WIRING.md:37-39 '(bit 0..7 = `PIN_SW_A`..`PIN_SW_H`). Default: all eight in the order ... `SW_DOG`, `SW_SEATED`, `SW_HOME`...'; WIRING.md:116 '| `SW_DOG` | `PIN_SW_D` | 14 | `D` |'; BENCH_TEST.md:48-52 same table with "D":1; GrappStar_Build_Guide.md:904-908, 914-916, 993-997, 2087-2091 (copies); CALIBRATION.md:105 'if `SW_DOG` says the dog teeth sit tip to tip'. Firmware: config.h:14 'enum SwId { SW_DRUM_A, SW_DRUM_B, SW_DRUM_C, SW_DOG_L, SW_DOG_R, SW_SEATED, SW_HOME, SW_RAIL_LOW, SW_WRAP_HOME, SW_INDEX, SW_COUNT }'; config.h:17 SW_TEL {"a","b","c","l","r","S","H","L","W","I"}; config.h:37 'int8_t swBit[SW_COUNT] = {0, 1, 2, 3, 4, 5, 6, 7, 8, -1};'; pins.h:32-38 PIN_SW_D 14, E 39, F 40, G 41, H 42, I 10, J 35; base_logic.h:664 'if (c.requireSeated && !closed(SW_SEATED)) return ... "NOT_SEATED"'; DECISIONS.md:39 D31 restores SW_DOG_L and SW_DOG_R.

Fix: Replace the contact rows in WIRING.md sections 1 and 5 (and regenerate the Build Guide copies, BENCH_TEST step 2 table, CALIBRATION 105) with the firmware order: GPIO11 SW_DRUM_A key a; GPIO12 SW_DRUM_B b; GPIO13 SW_DRUM_C c; GPIO14 SW_DOG_L l (closes when the collar is engaged at least 5 mm into LAUNCH); GPIO39 SW_DOG_R r (engaged into REEL); GPIO40 SW_SEATED S; GPIO41 SW_HOME H; GPIO42 SW_RAIL_LOW L; GPIO10 SW_WRAP_HOME W; GPIO35 SW_INDEX I (only without the AS5600 magnet). Change line 37 to 'bit 0..9 = PIN_SW_A..PIN_SW_J' with the ten-name default order, and the BENCH_TEST row to 'SW_DOG_L (GPIO14) "l":1' plus 'SW_DOG_R (GPIO39) "r":1'.

> Skeptic (confirmed): Checked the code and the claim holds. The firmware is in firmware/base/grappstar_base/. config.h:14 defines SwId as {SW_DRUM_A, SW_DRUM_B, SW_DRUM_C, SW_DOG_L, SW_DOG_R, SW_SEATED, SW_HOME, SW_RAIL_LOW, SW_WRAP_HOME, SW_INDEX}. config.h:37 sets swBit = {0,1,2,3,4,5,6,7,8,-1}. grappstar_base.ino:28 orders SW_PINS as A..J = GPIO 11,12,13,14,39,40,41,42,10,35. So the firmware reads GPIO14 as SW_DOG_L, 39 as SW_DOG_R, 40 as SW_SEATED, 41 as SW_HOME, 42 as SW_RAIL_LOW and 10 as SW_WRAP_HOME.

WIRING.md:27-31 and :116 still give the D27 single-SW_DOG layout: 14=SW_DOG, 39=SEATED, 40=HOME, 41=RAIL_LO
>
> Skeptic (confirmed): I could not refute this. The firmware and the docs give different contact orders. In firmware/base/grappstar_base/config.h:14 the SwId order is SW_DRUM_A, B, C, SW_DOG_L, SW_DOG_R, SW_SEATED, SW_HOME, SW_RAIL_LOW, SW_WRAP_HOME, SW_INDEX. config.h:37 sets swBit = {0,1,2,3,4,5,6,7,8,-1}, and grappstar_base.ino maps bit i to SW_PINS[i] = PIN_SW_A..J (GPIO 11,12,13,14,39,40,41,42,10,35, from pins.h:29-38). So the firmware reads GPIO14 as SW_DOG_L, GPIO39 as SW_DOG_R, GPIO40 as SW_SEATED, GPIO41 as SW_HOME, GPIO42 as SW_RAIL_LOW and GPIO10 as SW_WRAP_HOME. WIRING.md:27-31 and :116 still give the ol
>

## 4. Contact pin table in WIRING.md / BENCH_TEST.md / PROTOCOL telemetry keys is from the old firmware (one SW_DOG); the current firmware has SW_DOG_L + SW_DOG_R, so every launcher contact from GPIO39 on lands on the wrong input

- File: `firmware/base/WIRING.md:27`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: A learner wires the contacts exactly as WIRING.md sections 1, 5 and 7 say: SW_DOG on GPIO14, SW_SEATED on GPIO39, SW_HOME on GPIO40, SW_RAIL_LOW on GPIO41, SW_WRAP_HOME on GPIO42. config.h swBit = {0,1,2,3,4,5,6,7,8,-1} over SwId {DRUM_A,DRUM_B,DRUM_C,DOG_L,DOG_R,SEATED,HOME,RAIL_LOW,WRAP_HOME,INDEX}, so the firmware reads GPIO14 as SW_DOG_L (key l), GPIO39 as SW_DOG_R (r), GPIO40 as SW_SEATED (S), GPIO41 as SW_HOME (H), GPIO42 as SW_RAIL_LOW (L), and SW_WRAP_HOME is expected on GPIO10 (PIN_SW_I), which no doc mentions. I proved this with a scratch harness (/tmp/claude-0/review-scratch-G/bench.cpp) that prints "SW_SEATED GPIO39 expect S -> firmware sees SW_DOG_R key 'r'", "SW_HOME GPIO40 expect H -> firmware sees SW_SEATED key 'S'" and so on. Results: BENCH_TEST step 2 contact checks show the wrong keys, and the docs have no fix for that. Because the carriage-home contact is read as SW_SEATED, the fire interlock sees a dart as seated when none is there. A seated dart (GPIO39) reads as SW_DOG_R, so mode LAUNCH with a dart on gives SWITCH_DISAGREE. Aim homing waits on GPIO42 (the wrap arm contact) instead of the rail-low cam. The docs also describe SW_DOG as a 'tip to tip' contact (closed when NOT engaged). The firmware's SW_DOG_L/R close when the collar IS engaged, and mode LAUNCH/REEL only reports engaged when they are closed, so a contact built to the doc's meaning never confirms LAUNCH. CALIBRATION.md (c) step 5 promises a fault 'dogs stayed tip to tip' that does not exist in base_logic.h.

Evidence: WIRING.md:27 "| 14 | `PIN_SW_D` | contact `SW_DOG` | dog collar landed tip to tip (fork spring deflected) |"; WIRING.md:28-31 "| 39 | `PIN_SW_E` | contact `SW_SEATED` ... | 40 | `PIN_SW_F` | contact `SW_HOME` ... | 41 ... `SW_RAIL_LOW` | 42 ... `SW_WRAP_HOME`"; WIRING.md:116 "| `SW_DOG` | `PIN_SW_D` | 14 | `D` |"; BENCH_TEST.md:48-52 "| `SW_DOG` (GPIO14) | `\"D\":1` | ... `SW_SEATED` (GPIO39) | `\"S\":1`"; CALIBRATION.md:105 "if `SW_DOG` says the dog teeth sit tip to tip". Code: config.h:14 "enum SwId { SW_DRUM_A, SW_DRUM_B, SW_DRUM_C, SW_DOG_L, SW_DOG_R, SW_SEATED, SW_HOME, SW_RAIL_LOW, SW_WRAP_HOME, SW_INDEX, SW_COUNT };", config.h:17 SW_TEL {"a","b","c","l","r","S","H","L","W","I"}, config.h:37 "int8_t swBit[SW_COUNT] = {0, 1, 2, 3, 4, 5, 6, 7, 8, -1};", pins.h:37 "#define PIN_SW_I 10"; base_logic.h:151 collarLaunch requires closed(SW_DOG_L). Spec v1.2 D31 restores SW_DOG_L/R. The same stale tables are copied into docs/build/GrappStar_Build_Guide.md:904, 915, 993, 1937, 2087-2091 and PROTOCOL.md:86.

Fix: Rewrite WIRING.md section 1 rows 14/39/40/41/42 (and add GPIO10) and the section 5 table to match swBit: GPIO14 SW_DOG_L (key l, closes when the collar is engaged in LAUNCH), GPIO39 SW_DOG_R (key r, closes when engaged in REEL), GPIO40 SW_SEATED (S), GPIO41 SW_HOME (H), GPIO42 SW_RAIL_LOW (L), GPIO10 SW_WRAP_HOME (W). Make the same change to the BENCH_TEST.md step 2 contact table, the PROTOCOL.md sw key list and the mode row, and the CALIBRATION.md (c) step 5 text (replace the SW_DOG wiggle text with the SW_DOG_L/R engage check and its real fault text). Also make base/spec/params.py SWITCHES gpio numbers agree with pins.h: the comment there says "these match it" but they list SW_DRUM_A=10, SW_HOME=21, DRV8833 ain1=15/ain2=16, HX711 dt=17/sck=18. Then re-run python3 docs/build/make_guide.py.

> Skeptic (confirmed): The defect is real. I read the code and spec myself.

Firmware (the code under firmware/base/grappstar_base/):
- config.h:14 defines the SwId order as {DRUM_A, DRUM_B, DRUM_C, DOG_L, DOG_R, SEATED, HOME, RAIL_LOW, WRAP_HOME, INDEX}.
- config.h:37 sets swBit = {0,1,2,3,4,5,6,7,8,-1}.
- pins.h:29-37 maps bits 0..8 to GPIO 11,12,13,14,39,40,41,42,10.
- The .ino builds SW_PINS from PIN_SW_A..J, and base_logic.h:129 reads c.swBit[id].
- So by default the firmware reads: GPIO14 as SW_DOG_L, GPIO39 as SW_DOG_R, GPIO40 as SW_SEATED, GPIO41 as SW_HOME, GPIO42 as SW_RAIL_LOW, and GPIO10 as SW_WRAP_HOME.
>
> Skeptic (confirmed): I checked the code, and the claim holds. The reviewer's paths are off by one folder: the files are firmware/base/grappstar_base/config.h and pins.h, not firmware/base/config.h. What the code says:
- config.h:14 defines SwId as {DRUM_A, DRUM_B, DRUM_C, DOG_L, DOG_R, SEATED, HOME, RAIL_LOW, WRAP_HOME, INDEX}.
- config.h:17 gives the telemetry keys {a,b,c,l,r,S,H,L,W,I}.
- config.h:37 sets swBit = {0,1,2,3,4,5,6,7,8,-1}.
- grappstar_base.ino:28 maps bits 0..9 to SW_PINS {A..J}, and pins.h maps those to GPIO 11,12,13,14,39,40,41,42,10,35.

So by default the firmware reads:
- GPIO14 as SW_DOG_L
- G
>

## 5. params.SWITCHES claims to match pins.h, but every GPIO differs, including the DRV8833 and HX711 pins swapped and SW_HOME on GPIO21, which the firmware drives as an output

- File: `base/spec/params.py:236`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: docs

Scenario: BASE_SPEC section 4 sends readers to params.SWITCHES for 'the full list with suggested GPIOs', and the block says 'these match it'. A learner, or cad-frame routing the harness, who uses it would put DRV8833 AIN1/AIN2 on GPIO15/16 (the firmware's HX711 SCK/DT) and the HX711 on GPIO17/18 (the firmware's drum PWM). The drum would never run, and the HX711 data line would be driven by PWM. Drum code contacts would land on GPIO10/11/12 instead of 11/12/13, which gives code 000 or wrong sectors and DRUM_SENSOR faults. SW_HOME on GPIO21 puts a contact to GND on the pin the sketch drives HIGH (hwDrumSleep(true)) whenever the drum runs. With the carriage home that shorts an output-high GPIO straight to GND. Contact names also disagree with the firmware (SW_GAP, SW_N20, SW_ARM do not exist in SwId; SW_WRAP_HOME is missing).

Evidence: params.py:236 '# fw-base owns the final table (firmware/base/grappstar_base/pins.h); these match it.'; params.py:237-239 SW_DRUM_A gpio=10, SW_DRUM_B gpio=11, SW_DRUM_C gpio=12; params.py:244 'SW_HOME=dict(... gpio=21)'; params.py:251-252 'DRV8833=dict(ain1=15, ain2=16, ...), HX711=dict(dt=17, sck=18, ...)'. Versus pins.h:22-24 'PIN_DRUM_IN1 17 / PIN_DRUM_IN2 18 / PIN_DRUM_SLP 21', pins.h:29-31 PIN_SW_A 11 / B 12 / C 13, pins.h:41-42 'PIN_HX_DT 16 / PIN_HX_SCK 15'; grappstar_base.ino:60 'digitalWrite(PIN_DRUM_SLP, awake ? HIGH : LOW)'; BASE_SPEC.md:64 'The full list with suggested GPIOs is params.SWITCHES.'

Fix: Regenerate params.SWITCHES from pins.h + config.h swBit: SW_DRUM_A 11, SW_DRUM_B 12, SW_DRUM_C 13, SW_DOG_L 14, SW_DOG_R 39, SW_SEATED 40, SW_HOME 41, SW_RAIL_LOW 42, SW_WRAP_HOME 10, SW_INDEX 35; DRV8833 ain1=17, ain2=18, slp 'NOT WIRED (D9)'; HX711 dt=16, sck=15. Remove or mark 'not wired in firmware' the SW_GAP / SW_N20 / SW_ARM entries (or add them to SwId). Add a test that compares params.SWITCHES with pins.h.

> Skeptic (confirmed): I couldn't refute this. I checked each part against the code. params.py:236 says the table matches pins.h. params.py:237-252 gives DRUM_A/B/C as 10/11/12, DOG_L/R as 13/14, SW_HOME as 21, DRV8833 ain1/ain2 as 15/16 and HX711 dt/sck as 17/18. pins.h has PIN_DRUM_IN1/IN2/SLP as 17/18/21, PIN_SW_A..J as 11, 12, 13, 14, 39, 40, 41, 42, 10, 35, and PIN_HX_DT/SCK as 16/15. The DRV8833 and HX711 pin pairs really are swapped. config.h:14 lists the SwId order as DRUM_A, DRUM_B, DRUM_C, DOG_L, DOG_R, SEATED, HOME, RAIL_LOW, WRAP_HOME, INDEX, and swBit at config.h:37 maps those to bits 0..9. So the firmw
>
> Skeptic (confirmed): I checked the claim against the code and it holds. params.py:236 says the SWITCHES table matches firmware/base/grappstar_base/pins.h, but it does not.

GPIO mismatches:
- **Drum code contacts:** params puts SW_DRUM_A/B/C on 10/11/12. Firmware swBit {0,1,2} maps them to PIN_SW_A/B/C = 11/12/13.
- **Dog contacts:** params puts SW_DOG_L/R on 13/14. Firmware has them on 14/39.
- **SW_HOME:** params puts it on 21. In firmware, GPIO21 is PIN_DRUM_SLP: it is set to OUTPUT and LOW in setup at .ino:132, and hwDrumSleep(true) drives it HIGH at .ino:60. Firmware puts SW_HOME on PIN_SW_G = 41.
- **SW_RAIL
>

## 6. MEASURE_LIST B4 gives the superseded 280 mm full-cock extension (and 35 mm preload); D30/params now say 265 mm and 23.8 mm because 280 mm overloads the spring

- File: `docs/build/MEASURE_LIST.md:34`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: docs

Scenario: The base assembly steps that B4 defers to are not in the repo (Build Guide section 5 says 'NOT IN THE REPO YET'). So the only number the learner has for choosing the preload hole is 35 mm at rest and 280 mm at full cock. D30 says 280 mm is about 42 N, 9 percent over the Century C-243's 38.4 N rated maximum safe load, and sets 265 mm full cock with 23.8 mm preload. If the learner sets the anchors for 280 mm, every cock overstresses the springs, which risks permanent set or a spring breaking under load.

Evidence: MEASURE_LIST.md:34 "The coordinator's design value is 35 mm stretch at rest and 280 mm at full cock"; params.py:51 "max_ext=265.0, # D30: force at full cock about 40 N (catalogue 38.4 N, low cycle count); was 280"; params.py:172 "spring_preload=23.8, # D30: extension at HOME = SPRING.max_ext 265 - stroke 241.2"; DECISIONS.md:38 D30 "280 mm would be about 42 N, 9 percent over the rating". Mirrored in GrappStar_Build_Guide.md:2568.

Fix: Change B4 to: 'Design value (D30): 23.8 mm stretch at rest and 265 mm at full cock (stroke 241.2 mm); fallback hole set 8.8 mm / 250 mm if the hang test shows a weaker spring. Never set it for 280 mm.' Also change the assumed rate to the params.py value. Regenerate the build guide.

> Skeptic (confirmed): I checked the claim and it holds. docs/build/MEASURE_LIST.md:34 (B4) says "35 mm stretch at rest and 280 mm at full cock", and the build guide repeats it word for word at docs/build/GrappStar_Build_Guide.md:2568. DECISIONS.md D30 (line 38) explicitly replaces the 280 mm from D5: full cock is now 265 mm, about 40 N. At 280 mm the spring would carry about 42 N, which is 9 percent over the C-243's rated 38.4 N. params.py matches D30: SPRING.max_ext=265.0 (line 51), max_ext_fallback=250.0, stroke=241.2, spring_preload=23.8 (line 172) and spring_preload_fallback=8.8. B4 defers to "the base assembly
>
> Skeptic (confirmed): I could not refute the stale number, but the harm scenario is overstated, so I rate this low, not medium.

The stale number is real:
- docs/build/MEASURE_LIST.md:34 (copied in GrappStar_Build_Guide.md:2568) still says "35 mm stretch at rest and 280 mm at full cock".
- The Build Guide launch table at line 2431 also says "Spring extension full / rest | 280 / 35 mm", with 13.1 J and 84 N.
- Against that: DECISIONS.md D30 replaces D5's 280 mm with 265 mm, and says 280 mm would be about 42 N, 9 percent over the C-243's 38.4 N rating. params.py:51 sets max_ext=265 and params.py:172 sets spring_prelo
>

## 7. Stage 1 pass band for wiper free current (1 to 3 A) overlaps the jog trips (2.5 A forward, 3 A backward in NEUTRAL)

- File: `docs/build/TEST_PLAN.md:41`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: docs

Scenario: On the bench the drum code reads NEUTRAL (jumper). A forward wiper jog in NEUTRAL counts as take-up: trip 2.5 A and duty capped at 40 %. A backward jog trips at 3.0 A. A motor with a free current of 2.6 A passes by the TEST_PLAN's own band, but every forward jog latches OVERCURRENT 'wiper over 2 A' after 30 ms. The learner thinks the motor or wiring is bad. BENCH_TEST step 4 also says a jog runs at 'about 60 % power', but a forward jog in NEUTRAL runs at 40 %, so the rpm the learner sees is lower than they are told to expect. Simulator check: forward jog trip=2.5, backward trip=3.0 (/tmp/claude-0/review-scratch-G/nojumper.cpp).

Evidence: TEST_PLAN.md:41-42 "wiper free current about 1 to 3 A (A6 in MEASURE_LIST.md), no fault"; BENCH_TEST.md:112-113 "a jog runs at about 60 % power"; config.h:62-63 `takeupTripA = 2.5f; jogTripA = 3.0f;`; config.h:121 `takeupMaxDuty = 102` (40 %); base_logic.h:768-769 `bool takeup = sec >= SEC_NEUTRAL && sec <= SEC_WRAP && dd == c.reelDir; w.c.tripA = takeup ? c.takeupTripA : ...`.

Fix: Change the pass band to 'under 2.0 A (the bench jog trips at 2.5 A forward)'. Add: 'if you get OVERCURRENT "wiper over 2 A" on a forward jog with nothing attached, measure the free current with the meter (CALIBRATION (b)) and report it: the firmware take-up trip must be raised before the base is built.' In BENCH_TEST step 4 say forward jogs on the bench run at 40 % power and backward jogs at 60 %.

> Skeptic (confirmed): I checked each part of the claim against the code and could not refute it.

1. **The bench drum code reads NEUTRAL.** GrappStar_Build_Guide.md tells the learner to keep the GPIO12-to-GND jumper on for the whole bench test, so drum_pos reads NEUTRAL.

2. **A forward wiper jog in NEUTRAL counts as take-up.** In base_logic.h:768-769 (cmdJog), `takeup = sec >= SEC_NEUTRAL && sec <= SEC_WRAP && dd == c.reelDir`. config.h:71 sets reelDir = +1, so dir 1 (the direction TEST_PLAN, MEASURE_LIST A6 and the bench steps all use) is take-up. That sets tripA = takeupTripA = 2.5 A (config.h:62) and caps duty 
>
> Skeptic (confirmed): I checked the code and the docs, and the claim holds. The mechanism is real, but I rate it low, not medium.

Code path (all read directly):
- base_logic.h:768 sets `takeup = sec in NEUTRAL..WRAP && dd == c.reelDir`.
- config.h:71 sets reelDir = +1, so `jog wiper dir 1` counts as take-up whenever the drum code reads NEUTRAL.
- On the bench the drum code does read NEUTRAL. The Build Guide (lines 1001, 1148, 2056) tells the learner to put a jumper from GPIO12 to GND. Without it the code is 000, i.e. SEC_INVALID, and cmdJog refuses with SHIFTING.
- base_logic.h:769 then sets tripA = takeupTripA = 
>

## 8. Mode sequences in TEST_PLAN and CALIBRATION send `mode REEL` from NEUTRAL/LAUNCH, which the D16 firmware always refuses

- File: `docs/build/TEST_PLAN.md:64`
- Severity: medium (claimed high); 2 of 2 skeptics confirmed
- Owner: docs

Scenario: With the current firmware, REEL is reached only through `wrap` (NEUTRAL -> through WRAP -> REEL), and `mode NEUTRAL` from REEL is refused WRAPS_ON. So Stage 3 step 1 (`mode` LAUNCH, NEUTRAL, REEL, NEUTRAL, LAUNCH) stops at REEL with NOT_WRAPPED. Stage 3 step 4 ('REEL mode: wrap, then unwrap') cannot start in REEL. Stage 6 step 1 (`mode` REEL, `wrap`) gets NOT_WRAPPED. The same happens in CALIBRATION (b) free current step 1, (c) step 4 (`mode REEL` after NEUTRAL) and (e) step 2 (`mode REEL` then `wrap`). The learner cannot finish calibration, so Stage 3 is blocked.

Evidence: TEST_PLAN.md:64 "`mode` LAUNCH, NEUTRAL, REEL, NEUTRAL, LAUNCH: 20 full rounds"; TEST_PLAN.md:116 "`mode` REEL, `wrap`, then `reel`"; CALIBRATION.md:72, 95-98, 161; base_logic.h:589-593 `if (t == DP_REEL) { ... if (sec < SEC_LOCK) return "NOT_WRAPPED"; } else { if (sec >= SEC_WRAP) return "WRAPS_ON"; }`; base_logic.h:686-697 cmdWrap: from NEUTRAL `startDrumMove(DP_REEL ...)`; base_logic.h:700-709 cmdUnwrap goes back to NEUTRAL.

Fix: Stage 3 step 1: `mode` LAUNCH, NEUTRAL, then `wrap` (ends in REEL, `WRAPPED` event), `unwrap` (ends in NEUTRAL, `UNWRAPPED`), `mode` LAUNCH; expect drum_pos LAUNCH / NEUTRAL / REEL / NEUTRAL / LAUNCH. Stage 3 step 4: from NEUTRAL, `wrap` then `unwrap`. Stage 6 step 1: `mode` NEUTRAL, `wrap`, then `reel`. CALIBRATION (b), (c) step 4 and (e) step 2: replace `mode REEL` with `mode NEUTRAL` then `wrap`, and go back with `unwrap`. Also update (c) step 5 to the new dog contacts (SW_DOG_L / SW_DOG_R, fault why "dogs did not engage (collar contact open)", up to 40 deg in LAUNCH and 120 deg in REEL).

> Skeptic (confirmed): The claim holds. In firmware/base/grappstar_base/config.h:23 the sectors are ordered SEC_LAUNCH < SEC_LN < SEC_NEUTRAL < SEC_PREP < SEC_WRAP < SEC_LOCK < SEC_REEL. In base_logic.h, cmdMode (lines 589-593) works like this: when the target is REEL it returns NOT_WRAPPED whenever sec < SEC_LOCK, which covers LAUNCH, NEUTRAL, PREP and WRAP. Otherwise it returns WRAPS_ON whenever sec >= SEC_WRAP. So `mode REEL` from NEUTRAL or LAUNCH is always refused. `mode NEUTRAL` or `mode LAUNCH` from REEL (sec 6 >= SEC_WRAP) is also always refused. The only ways in and out of REEL are cmdWrap (lines 686-697: f
>
> Skeptic (confirmed): I could not refute this. The claim holds against the current code. config.h:23 orders the sectors SEC_LAUNCH(0), LN, NEUTRAL(2), PREP, WRAP(4), LOCK(5), REEL(6). In base_logic.h, cmdMode (around lines 583-604) handles t==DP_REEL like this: it only succeeds if the drum is already in REEL with the collar engaged, and otherwise returns "NOT_WRAPPED" when sec < SEC_LOCK. So `mode REEL` sent from LAUNCH or NEUTRAL is always refused. For any target other than REEL, it returns "WRAPS_ON" when sec >= SEC_WRAP, so `mode NEUTRAL` from REEL is refused as well. The only way into REEL is cmdWrap, which nee
>

## 9. Dry-fire stages 3 and 4 cannot be done as written: no dart means NOT_SEATED, and the CLI cannot send fire within the 3 s token life

- File: `docs/build/TEST_PLAN.md:67`
- Severity: medium (claimed high); 2 of 2 skeptics confirmed
- Owner: docs

Scenario: Stage 3 step 3 (no springs, no dart) and Stage 4 (springs, no dart) say '`arm`, then `fire` with the token'. require_seated defaults to true, so `arm` with no dart is refused NOT_SEATED. Even with SW_SEATED closed, the documented method is one `python3 -m grappstar base cmd` process per command. Each process opens the port (up to 3 s wait_connected), prints the token and exits. The learner then has to type a second full command with the 6-digit token within 3000 ms. That gives TOKEN_EXPIRED, or NOT_ARMED if the new connection loses the token. The operator page has no arm/fire outside a full mission. So the first spring shots, the key safety gate before Stage 5, cannot be run. The learner is pushed to improvise, for example `set require_seated 0`, which is saved in flash and would carry over to dart shots.

Evidence: TEST_PLAN.md:67 "Cock and fire with no springs: `arm`, then `fire` with the token."; TEST_PLAN.md:84 "`arm` and `fire` once."; base_logic.h:663 `if (c.requireSeated && !closed(SW_SEATED)) return fitted(SW_SEATED) ? "NOT_SEATED" : "NO_SENSOR";`; config.h:146 `armMs = 3000`; cli.py:124-127 `base cmd` sends one command per process; cli.py:318 no arm/fire subcommand; config.h:150 requireSeated saved via tunNow().

Fix: In Stages 3 and 4 give one command that arms and fires in a single process, for example: `python3 -c "from grappstar.comms.base import BaseClient; c=BaseClient('/dev/grappstar-base').start(); c.wait_connected(3); print(c.fire(c.arm()))"` (run in ~/GrappStar/jetson). Tell the learner to load a printed dummy-sized block that closes SW_SEATED, or to send `{"cmd":"set","key":"require_seated","val":0}` for stages 3 and 4 only. Then add a Stage 5 precondition: `set require_seated 1` and check that `get require_seated` replies true before any dart is loaded.

> Skeptic (confirmed): I checked the claim against the code and it holds.

1. The NOT_SEATED refusal is real. In base_logic.h:655-666, fireCheck() returns NOT_SEATED when c.requireSeated is set and SW_SEATED is open (or NO_SENSOR if the contact is not fitted). cmdArm() at line 668 calls fireCheck() first. config.h:150 sets requireSeated = true by default, and it is persisted (persist.h:54, base_logic.h:111/119). cmdFire() also re-runs fireCheck(). So with no dart, `arm` is refused in Stage 3 step 3 and in Stage 4. Neither stage tells the learner to close SW_SEATED by hand, load a block, or change require_seated. The
>
> Skeptic (confirmed): I checked the claim against the code and could not refute it. TEST_PLAN.md:67 (Stage 3, no dart) and :84 (Stage 4, no dart) both tell the learner to `arm` and then `fire`. In firmware base_logic.h:664, fireCheck() returns NOT_SEATED when c.requireSeated is set and SW_SEATED is open. cmdArm calls fireCheck first, and config.h:150 sets requireSeated = true by default. The mock matches this (mock_base.py:1125) and so does the test (test_base.cpp:335). No part of TEST_PLAN, CALIBRATION or the build guide mentions require_seated or tells the learner to close SW_SEATED for these stages. So as writte
>

## 10. Stage 3 estop test expects the drum to go to NEUTRAL after every move, but the firmware deliberately holds it during cock/fire and with wraps on

- File: `docs/build/TEST_PLAN.md:70`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: docs

Scenario: The learner sends `estop` mid-wrap, mid-unwrap, in REEL, or mid-cock/fire. The firmware holds the drum and emits MODE ok:false err DRUM_HELD ("wraps on..." or "launch train not at rest..."). An estop during cock/fire also sets the shot phase to UNKNOWN. The doc marks this as a fail. It also gives no recovery, so the next steps fail: `mode` gets LAUNCH_BUSY, `arm` gets NOT_COCKED, `aim` gets PHASE_UNKNOWN.

Evidence: TEST_PLAN.md:70 "Send `estop` in the middle of each kind of move once. The wiper stops at once and the drum goes to NEUTRAL."; base_logic.h:244-246 safeToNeutral returns "wraps on..." / "launch train not at rest..."; base_logic.h:963 `if (was == S_COCK || was == S_FIRE || was == S_DECOCK) { phaseHint = cockPhase; phase = PH_UNKNOWN; }`; base_logic.h:970 `evHeld(hold)`.

Fix: Expected results per move: during `mode` or `aim`, the drum goes to NEUTRAL (MODE ok). During `wrap`, `unwrap` or `reel`, the drum holds (MODE ok:false, err DRUM_HELD, why 'wraps on'); recover with `unwrap`. During `cock` or `fire`, the drum holds (DRUM_HELD, 'launch train not at rest') and the phase becomes UNKNOWN; recover with `{"cmd":"cal","what":"decock"}` (carriage drawn) or `cock` (carriage home), then `aim` again.

> Skeptic (confirmed): I could not refute this. The claim holds up against the code.

The doc: TEST_PLAN.md:70 (Stage 3 step 5) says "Send `estop` in the middle of each kind of move once. ... the drum goes to NEUTRAL." Steps 1-4 just before it list the kinds of move: mode, aim, cock/fire, wrap/unwrap and reel. So "each kind" includes all of them.

What the firmware does in cmdStop (base_logic.h ~959-974):
1. It halts both motors.
2. If it was in S_COCK, S_FIRE or S_DECOCK, it sets phase = PH_UNKNOWN.
3. It calls safeToNeutral(F_NONE). If that returns a reason and the drum is not at NEUTRAL, it sends evHeld(hold), wh
>
> Skeptic (confirmed): I could not refute this claim. TEST_PLAN.md:70 (repeated in GrappStar_Build_Guide.md:2316) says that after an estop during each kind of move "the drum goes to NEUTRAL". The firmware does not do that.

What the code does:
- In cmdStop (base_logic.h:957-972), estop calls safeToNeutral(F_NONE). It only drives the drum to NEUTRAL when that returns nullptr. Otherwise it calls evHeld, which sends MODE ok:false err DRUM_HELD.
- The sector order in config.h:23 is LAUNCH, LN, NEUTRAL, PREP, WRAP, LOCK, REEL.
- So in the REEL sector, sec >= SEC_WRAP is true, and safeToNeutral returns "wraps on". This co
>

## 11. BENCH_TEST step 2 drum-code walk faults on the very first row and can lock the base in a DRUM_SENSOR fault that only a reboot clears

- File: `firmware/base/BENCH_TEST.md:58`
- Severity: medium (claimed high); 2 of 2 skeptics confirmed
- Owner: docs

Scenario: Learner follows step 2 with the GPIO12 jumper (b). Row 2 closes `a`. The drum N20 is off, so after 300 ms the firmware raises FAULT DRUM_SENSOR "drum code changed with the drum still" (base_logic.h:413). The doc says a fault happens only if you skip a row. Rows 6 to 9 also do not match the firmware Gray order: secCode is LAUNCH a, LN ab, NEUTRAL b, PREP bc, WRAP c, LOCK ac, REEL abc (config.h:99), and drumPos() reports LOCK as WRAPPED and WRAP as BETWEEN (base_logic.h:144-148). Going from row 6 (bc) to row 7 (abc) skips three sectors, so the base sets secBad and stays on PREP. When the learner walks back to `b` (sector 4 to 2, another jump), `clear` keeps answering STILL_PRESENT for good. I proved this in the simulator (/tmp/claude-0/review-scratch-G/bench_walk.cpp). Even a correct walk faults on row 2, and the first `clear` after it raises the fault again because quietSec is stale (walk2.cpp: two FAULT lines, the second clear works). The table's expected values are also wrong: row 3 `mode` LAUNCH is really NONE (SW_DOG_L is fitted and open), row 7 `mode` REEL is really NONE (SW_DOG_R open), row 8 ac shows WRAPPED (doc says BETWEEN), row 9 c shows BETWEEN (doc says WRAPPED).

Evidence: BENCH_TEST.md:65-68 "| 6 | `b` `c` | `BETWEEN` (REEL side) | `NONE` | / | 7 | `a` `b` `c` | `REEL` | `REEL` | / | 8 | `a` `c` | `BETWEEN` (laying wraps) | `REEL` | / | 9 | `c` | `WRAPPED` | `REEL` |"; BENCH_TEST.md:75 "A `FAULT` `DRUM_SENSOR` while you do this is expected if you skip a row"; config.h:99 `secCode[SEC_COUNT] = {0x1, 0x3, 0x2, 0x6, 0x4, 0x5, 0x7}`; base_logic.h:413 `raise(F_DRUM_SENSOR, "drum code changed with the drum still")`; base_logic.h:367 a jump of more than one sector sets secBad; base_logic.h:978 clear returns STILL_PRESENT while secBad.

Fix: Rewrite the step 2 walk in the firmware order: b NEUTRAL/NEUTRAL, ab BETWEEN/NONE, a LAUNCH/NONE (LAUNCH only with SW_DOG_L closed), ab, b, bc BETWEEN/NEUTRAL, c BETWEEN/NEUTRAL, ac WRAPPED/NONE, abc REEL/NONE (REEL only with SW_DOG_R closed), then back the same way to b. Tell the learner that every change made by hand raises DRUM_SENSOR "drum code changed with the drum still" (expected, nothing moves). At the end, with only b closed, send `clear` twice (the first clear raises it again), or press RESET on the DevKitC. Say that a jump (for example bc to abc) leaves `clear` answering STILL_PRESENT until a reset.

> Skeptic (confirmed): The core of the claim holds up, both in the code and when run.

1) A fault on the first row is certain. watch() at base_logic.h:412-415 raises F_DRUM_SENSOR "drum code changed with the drum still" whenever the sector changes while the N20 is off (more than 300 ms after d.stopT), once booted, with quietSec valid and state not S_JOG. quietSec becomes NEUTRAL after boot, so closing `a` (code 0x3 = SEC_LN) on row 2 raises it. The doc at :75 says a fault is expected only if you skip a row or open everything, so the doc is wrong.

2) The table does not match the firmware. config.h:99 orders the code
>
> Skeptic (confirmed): I could not refute this. The claim holds against the code.

1) Gray order: config.h:99 sets secCode = {a, ab, b, bc, c, ac, abc} for LAUNCH, LN, NEUTRAL, PREP, WRAP, LOCK, REEL. The table in BENCH_TEST.md:58-68 puts abc straight after bc (row 6 to 7). That is sector 3 to sector 6. In updateSector() (base_logic.h:367), abs(s-sec)>1 with booted set makes secBad true and keeps sec at PREP. Row 8 (ac, LOCK) is also a jump from PREP. Row 9 (c, WRAP) is next to PREP, so it is accepted. drumPos() (base_logic.h:144-148) maps LOCK to WRAPPED and WRAP to BETWEEN, so the table's row 8 BETWEEN and row 9 W
>

## 12. Calibration and test plan send `mode REEL` from NEUTRAL, which the firmware always refuses (NOT_WRAPPED); REEL to NEUTRAL is refused too (WRAPS_ON)

- File: `firmware/base/CALIBRATION.md:96`
- Severity: medium (claimed high); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: CALIBRATION (c) step 4 tells the learner to send mode NEUTRAL, REEL, NEUTRAL, LAUNCH and wait for each MODE ok. The REEL line comes back {"ok":false,"err":"NOT_WRAPPED"}. I checked this in a scratch run (modes.cpp). Under D16 the firmware only lets the drum enter REEL through `wrap`, and REEL to NEUTRAL needs `unwrap`. The same dead end appears in CALIBRATION (b) Free running current step 1 (mode REEL, then wait for MODE ok), in (e) step 2 (mode REEL, then wrap), in TEST_PLAN stage 3 step 1 (20 rounds of LAUNCH, NEUTRAL, REEL, NEUTRAL, LAUNCH), and in stage 6 step 1. The learner cannot finish calibration (b), (c) or (e), or test stages 3 and 6, as written.

Evidence: CALIBRATION.md:72 "1. Line slack. `{\"id\":1,\"cmd\":\"mode\",\"to\":\"REEL\"}` and wait for the `MODE` event with `\"ok\":true`."; CALIBRATION.md:96 "{\"id\":4,\"cmd\":\"mode\",\"to\":\"REEL\"}"; CALIBRATION.md:161; TEST_PLAN.md:64 "1. `mode` LAUNCH, NEUTRAL, REEL, NEUTRAL, LAUNCH: 20 full rounds."; TEST_PLAN.md:116 "`mode` REEL, `wrap`, then `reel`". base_logic.h:589-591 "if (t == DP_REEL) { // REEL engages only with the wraps on (D16): from LOCK, or use \"wrap\" ... if (sec < SEC_LOCK) return \"NOT_WRAPPED\";" and :593 "if (sec >= SEC_WRAP) return \"WRAPS_ON\"; // unwrap first". Mirrored in GrappStar_Build_Guide.md:1904, 1928, 1993.

Fix: In CALIBRATION (b), (c) and (e) and TEST_PLAN stages 3 and 6, replace `mode REEL` with `mode NEUTRAL` followed by `wrap` (wait for WRAPPED), and replace REEL to NEUTRAL with `unwrap`. For (c) step 4 the sequence becomes: mode NEUTRAL, wrap, unwrap, mode LAUNCH, mode NEUTRAL. TEST_PLAN stage 3 step 1 becomes: mode LAUNCH, mode NEUTRAL, wrap, unwrap, mode LAUNCH. Regenerate the build guide.

> Skeptic (confirmed): I could not refute this. The defect is real; only the severity is overstated.

Code (firmware/base/grappstar_base/base_logic.h, cmdMode at about lines 583-596): the sector order in config.h:23 is SEC_LAUNCH=0, SEC_LN=1, SEC_NEUTRAL=2, SEC_PREP=3, SEC_WRAP=4, SEC_LOCK=5, SEC_REEL=6.
- `mode REEL`: if the drum is not already in REEL with the collar engaged, any sec below SEC_LOCK returns "NOT_WRAPPED". From NEUTRAL (sec 2) that is always refused, before any drum move starts.
- `mode NEUTRAL` or `mode LAUNCH` from REEL (sec 6 >= SEC_WRAP=4) always returns "WRAPS_ON".
- The only routes into and ou
>
> Skeptic (confirmed): I checked this against the code and the claim holds. In firmware/base/grappstar_base/config.h:23 the sector order is SEC_LAUNCH, SEC_LN, SEC_NEUTRAL, SEC_PREP, SEC_WRAP, SEC_LOCK, SEC_REEL. In base_logic.h, cmdMode (lines 589-594) handles to=REEL like this: it returns ok at once only if the drum is already in SEC_REEL with the collar engaged. Otherwise it returns "NOT_WRAPPED" whenever sec < SEC_LOCK, which covers LAUNCH and NEUTRAL. In the other direction, any target other than REEL returns "WRAPS_ON" when sec >= SEC_WRAP. So REEL to NEUTRAL or REEL to LAUNCH through `mode` is always refused.
>

## 13. CALIBRATION (e) k_n_per_a: jogging to 20 N needs about 4 A, but a jog trips OVERCURRENT at 3 A

- File: `firmware/base/CALIBRATION.md:163`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: docs

Scenario: In REEL a wiper jog uses jogTripA = 3.0 A (6 samples, 30 ms). The doc's own example says the pull at 20 N reads 4.0 A (free 1.5 A + 20/8). The capstan pull therefore latches FAULT OVERCURRENT "wiper over 3 A" before the scale reaches 20 N, and k_n_per_a can never be measured as written. A `reel` would use tripA 5 A instead.

Evidence: CALIBRATION.md:163-166 "Jog in short steps: `{"id":3,"cmd":"jog","motor":"wiper","dir":1,"ms":300}`... When the scale reads about 20 N... Example: amps 4.0, free_a 1.5"; config.h:63 `float jogTripA = 3.0f;`; base_logic.h:769 jog in REEL `w.c.tripA = ... : c.jogTripA`; base_logic.h:735 reel uses `c.tripA` (5 A, config.h:60).

Fix: Replace the jog with a slow reel and stop: `{"id":3,"cmd":"reel","duty":0.3,"max_n":40,"until":"stop"}`. Watch the scale, read `amps` at about 20 N, then send `{"id":4,"cmd":"stop"}`. Say that a jog cannot pull more than about 3 A.

> Skeptic (confirmed): I checked the code and could not refute the claim. After `wrap` finishes (tickWrap / cmdWrap, base_logic.h:690 and 1169) the drum sector is SEC_REEL. In cmdJog (base_logic.h:745-775), a wiper jog at SEC_REEL with dd == reelDir passes the BACKSTOP check. It is not `takeup`, because takeup only covers NEUTRAL..WRAP. It is not the cock branch either, because p is DP_REEL and not DP_LAUNCH. So line 769 sets w.c.tripA = c.jogTripA, which is 3.0 A (config.h:63). A deadman refresh keeps the same trip. drives.h:55-56 latches overcurrent after 6 ticks in a row with ampsRaw > tripA, once the 50 ms inrus
>
> Skeptic (confirmed): I checked the code and the claim holds. After `wrap` the collar sits in SEC_REEL (base_logic.h:690/1169 emit WRAPPED only when sec==SEC_REEL). In cmdJog (base_logic.h:746-770), `takeup` is true only for sec NEUTRAL..WRAP, so in SEC_REEL a forward wiper jog sets w.c.tripA = c.jogTripA = 3.0 A (config.h:63). drives.h:55-56 trips on unfiltered ampsRaw > tripA for 6 ticks in a row (30 ms) once the 50 ms inrush window has passed. base_logic.h:398 then latches the overcurrent fault "wiper over 3 A". The firmware's own defaults are freeA = 1.5 A (params_gen.h:20, params.py:31) and kNperA = 8.0 (confi
>

## 14. WIRING.md and the Build Guide wire DRV8833 SLP to GPIO21, against D9 and SYSTEM_ELECTRICAL. TEST_PLAN then adds SLP to 3V3, which shorts 3V3 into the GPIO21 output the firmware drives LOW

- File: `firmware/base/WIRING.md:165`
- Severity: medium (claimed high); 2 of 2 skeptics confirmed
- Owner: docs

Scenario: The learner follows WIRING.md Part B step 10 and the Build Guide (lines 974, 1042): SLP goes to GPIO21. On the Pololu 2130, SLP is pulled up to VIN (the 9 V buck output) through tens of kilohms (dart WIRING.md:113). That puts a 9 V pull-up on a 3.3 V ESP32 pin. With the battery in and USB out, it back-feeds the unpowered ESP32 through the pin clamp, which D9 says the design avoids. Worse: if the drum does not turn at TEST_PLAN stage 1 for any reason, TEST_PLAN:45 tells the learner to add a wire from SLP to the ESP32 3V3 pin. With SLP already on GPIO21, setup() runs pinMode(PIN_DRUM_SLP, OUTPUT); digitalWrite(LOW) and hwDrumSleep(false) drives it LOW again whenever the drum sleeps. That shorts the 3V3 rail straight into the GPIO21 low-side driver: pin damage, or a sagging 3V3 that resets the board. SYSTEM_ELECTRICAL first-power step 3 also says 'Check no wire is on SLP', so a learner reading the Build Guide gets two opposite instructions.

Evidence: WIRING.md:22 '| 21 | `PIN_DRUM_SLP` | DRV8833 SLP (nSLEEP) | high = driver on. Optional 10 k from SLP to GND |'; WIRING.md:97 '- SLP to GPIO21.'; WIRING.md:165 '10. Battery unplugged. DRV8833: AIN1 GPIO17, AIN2 GPIO18, SLP GPIO21 (optional 10 k SLP to GND)'; BENCH_TEST.md:129 'SLP wire on GPIO21?'; GrappStar_Build_Guide.md:899, 974, 1042 (same). Contradicted by DECISIONS.md:17 'D9 | DRV8833 #3 SLP pin not wired (board pulls it high) ... avoids back-feeding the ESP32'; pins.h:24 'PIN_DRUM_SLP 21 // NOT WIRED (decision D9...)'; SYSTEM_ELECTRICAL.md:92-93 'Its SLP / nSLEEP / EEP pad is NOT wired'; TEST_PLAN.md:45 'add a wire from DRV8833 SLP to the ESP32 3V3 pin'; grappstar_base.ino:132 'pinMode(PIN_DRUM_SLP, OUTPUT); digitalWrite(PIN_DRUM_SLP, LOW);'.

Fix: In WIRING.md, change the GPIO21 row to 'not wired (D9): leave DRV8833 SLP unconnected; the Pololu board pulls it high'. Delete 'SLP to GPIO21 ... optional 10 k' (lines 97-98) and the 'SLP GPIO21 (optional 10 k SLP to GND)' part of step 10. Change BENCH_TEST.md:129 to 'SLP must have no wire on it'. Regenerate the Build Guide. In TEST_PLAN.md:45, add 'only if SLP has no other wire on it (never also to GPIO21)'.

> Skeptic (confirmed): The core contradiction is real, and I checked every citation. The decision record (DECISIONS.md:17, D9) and the pin file (pins.h:24, "NOT WIRED") both say DRV8833 SLP has no wire. SYSTEM_ELECTRICAL.md:70 and 92-93 (Build Guide 812 and 834-835) say the same and tell the learner to check that no wire is on SLP. But firmware/base/WIRING.md:22, :97 and :165, BENCH_TEST.md:129, and the Build Guide at 899, 974, 1042 and 2168 tell the learner to wire SLP to GPIO21. So the merged Build Guide gives opposite instructions, as the claim says.

The firmware drives the pin either way. It sets PIN_DRUM_SLP a
>
> Skeptic (confirmed): The conflict is real, and I checked every cited line. firmware/base/WIRING.md:22, :97 and :165, BENCH_TEST.md:129, and the generated Build Guide (899, 974, 1042, 2168) all wire DRV8833 SLP to GPIO21. Three sources say the opposite: DECISIONS.md D9 ('SLP pin not wired ... avoids back-feeding the ESP32'), pins.h:24 ('NOT WIRED (decision D9...)'), and SYSTEM_ELECTRICAL.md:70/92-93/124, which also appear in the Build Guide at 812/834-835/866 ('Check no wire is on SLP'). So the one Build Guide gives the learner two opposite instructions. The firmware does drive the pin: setup() at line 132 sets it 
>

## 15. DRV8833 SLP wiring contradicts itself across docs: WIRING says wire SLP to GPIO21, SYSTEM_ELECTRICAL/D9/pins.h say leave it unwired, TEST_PLAN says wire it to 3V3

- File: `firmware/base/WIRING.md:165`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: The learner does WIRING Part B step 10 and solders DRV8833 SLP to GPIO21 (plus the optional 10 k to GND). SYSTEM_ELECTRICAL first power-on step 3 then says no wire may be on SLP. TEST_PLAN stage 1 says to add a wire from SLP to 3V3. D9 says SLP is not wired because the Pololu 2130 pulls it high itself, which 'avoids back-feeding the ESP32'. Following WIRING ties the board's pull-up to an ESP32 GPIO that pins.h marks 'NOT WIRED', and params.py now gives GPIO21 to SW_HOME. A learner who follows TEST_PLAN later ends up with SLP on both GPIO21 and 3V3, so a GPIO driven low is shorted to 3V3.

Evidence: WIRING.md:22 "| 21 | `PIN_DRUM_SLP` | DRV8833 SLP (nSLEEP) | high = driver on. Optional 10 k from SLP to GND |"; WIRING.md:97 "- SLP to GPIO21."; WIRING.md:165 "10. Battery unplugged. DRV8833: AIN1 GPIO17, AIN2 GPIO18, SLP GPIO21 (optional 10 k SLP to GND)"; BENCH_TEST.md:129 "SLP wire on GPIO21?"; SYSTEM_ELECTRICAL.md:92-93 "Its SLP / nSLEEP / EEP pad is NOT wired ... Check no wire is on SLP"; TEST_PLAN.md:45 "add a wire from DRV8833 SLP to the ESP32 3V3 pin"; pins.h:24 "#define PIN_DRUM_SLP 21 // NOT WIRED (decision D9: the Pololu 2130 pulls SLP high itself)"; DECISIONS.md:17 D9.

Fix: Follow D9 everywhere. In WIRING.md line 22 write 'not wired (D9)'. Delete the SLP sentence at line 97 and change step 10 (line 165) to 'AIN1 GPIO17, AIN2 GPIO18, GND to ESP32 GND. Leave SLP unwired.' Change BENCH_TEST.md:129 from 'SLP wire on GPIO21?' to 'SLP pad not wired?'. Keep TEST_PLAN's 3V3 fallback only as 'if a non-Pololu board does not pull SLP high', and say SLP must then never also go to GPIO21.

> Skeptic (confirmed): I checked every citation and each one says what the claim says. firmware/base/WIRING.md tells the learner to wire SLP to GPIO21 in three places: the pin table at line 22, "SLP to GPIO21" at line 97, and Part B step 10 at line 165. firmware/base/BENCH_TEST.md:129 then troubleshoots "SLP wire on GPIO21?". The other sources all say SLP is not wired: DECISIONS.md D9 (line 17), params.py:251 (slp="NOT WIRED (D9)"), pins.h:24 ("NOT WIRED (decision D9...)"), and SYSTEM_ELECTRICAL.md lines 70, 92-93 and 124 ("Check no wire is on SLP"). docs/build/REPORT.md:42-43 even records that the coordinator ruled
>
> Skeptic (confirmed): I checked every quoted line and each one says what the claim says.

**Docs that say to wire SLP to GPIO21:**
- firmware/base/WIRING.md:22 (table row: SLP on GPIO21, optional 10 k to GND).
- firmware/base/WIRING.md:97 ("SLP to GPIO21").
- firmware/base/WIRING.md:165 (Part B step 10: "SLP GPIO21").
- firmware/base/BENCH_TEST.md:129 asks "SLP wire on GPIO21?".

**Docs that say to leave SLP unwired:**
- base/spec/DECISIONS.md D9: SLP not wired, the board pulls it high, avoids back-feeding the ESP32.
- base/spec/params.py:251: slp="NOT WIRED (D9)".
- pins.h:24: "NOT WIRED (decision D9...)".
- docs/
>

## 16. WIRING.md wires the 22 AWG MP1584 input straight to the B+ screw behind only the 20 A fuse, without the 2 A inline fuse SYSTEM_ELECTRICAL requires

- File: `firmware/base/WIRING.md:168`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: docs

Scenario: WIRING.md is the doc SYSTEM_ELECTRICAL says wins, and it is the step list a learner follows. Part B step 12 connects buck IN+ to BTS7960 B+ with no fuse of its own, and Part A step 2 sets the buck from the 20 A lead with no small fuse either. If the thin tap or the buck input shorts (pinched 22 AWG, solder bridge on the MP1584), only the 20 A fuse protects it. SYSTEM_ELECTRICAL itself explains that 22 AWG behind only the 20 A fuse burns before that fuse blows: a fire risk on the rover. The parts list (MEASURE_LIST C8) has the 2 A fuse holder for exactly this tap.

Evidence: WIRING.md:48 '3S LiPo + --> 20 A fuse (XT60 lead #2) --> BTS7960 B+ --+--> MP1584 IN+'; WIRING.md:63 'Take the buck input from the BTS7960 B+ and B- terminals, so it sits behind the fuse.'; WIRING.md:168 '12. Battery unplugged. Buck input: IN+ to BTS7960 B+, IN- to BTS7960 B-.' (Build Guide 925, 940, 1045). Versus SYSTEM_ELECTRICAL.md:43-44 'The thin taps (MP1584 input, RC832) each get their own small inline fuse (2 A and 1 A) right at the tap. A 22 AWG wire behind only the 20 A or 10 A fuse would burn before that fuse blew'; SYSTEM_ELECTRICAL.md:60 'P4 ... 2 A inline fuse in the + wire right at the screw'.

Fix: WIRING.md:48: draw 'B+ --> 2 A inline fuse --> MP1584 IN+'. Line 63: 'through a 2 A inline fuse fitted right at the B+ screw (MEASURE_LIST C8; without it use 18 AWG under 10 cm)'. Step 12: 'Battery unplugged. Buck input: IN+ through the 2 A inline fuse to BTS7960 B+ (fuse right at the screw), IN- to BTS7960 B-.' Add the 2 A fuse holder to the 'You need' list at line 136. Regenerate the Build Guide.

> Skeptic (confirmed): The defect is real. The cited lines say what the claim says. firmware/base/WIRING.md line 48 draws B+ going straight to MP1584 IN+. Line 63 says to take the buck input from the B+/B- terminals "so it sits behind the fuse", which means the 20 A fuse. Step 12 (line 168) says "IN+ to BTS7960 B+, IN- to BTS7960 B-". The "You need" list at line 136 names only the 20 A fuse lead. "fuse" appears nowhere else in WIRING.md, and none of those places mentions a 2 A fuse. docs/build/SYSTEM_ELECTRICAL.md requires that fuse in three places: line 27 (power tree), lines 43-44 (the reason: a thin 22 AWG tap be
>
> Skeptic (confirmed): I checked the files and the claim holds. One supporting point is overstated.

The cited lines say what the claim says:
- WIRING.md:48 draws B+ --+--> MP1584 IN+ with nothing in between.
- WIRING.md:63 says the buck input comes from the B+/B- terminals "so it sits behind the fuse". That means the 20 A fuse.
- Part B step 12 (line 168) is "IN+ to BTS7960 B+, IN- to BTS7960 B-".
- The 'You need' list (line 136) names only the 20 A fuse lead.
- A grep finds no 2 A or inline fuse anywhere in WIRING.md. It also gives no gauge for the buck tap.

The other documents ask for this fuse:
- SYSTEM_ELECTRI
>

## 17. WIRING.md wires the 22 AWG buck input straight off the B+ screw behind only the 20 A fuse, with no 2 A inline fuse; SYSTEM_ELECTRICAL requires the 2 A fuse

- File: `firmware/base/WIRING.md:168`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: The learner builds from WIRING.md Part B, the step-by-step list, and connects MP1584 IN+ directly to BTS7960 B+. If the thin buck lead or the buck shorts, only the 20 A fuse protects it. SYSTEM_ELECTRICAL itself says a 22 AWG tap behind only the 20 A fuse 'would burn before that fuse blew'. The two documents disagree, and WIRING.md is the one with the numbered build steps.

Evidence: WIRING.md:168 "12. Battery unplugged. Buck input: IN+ to BTS7960 B+, IN- to BTS7960 B-."; WIRING.md:47-50 power diagram (no fuse on the MP1584 tap); SYSTEM_ELECTRICAL.md:60 "| P4 | BTS7960 B+ / B- | MP1584 #2 IN+ / IN- | 22 AWG | under the same screws, 2 A inline fuse in the + wire right at the screw |"; SYSTEM_ELECTRICAL.md:43-44 "A 22 AWG wire behind only the 20 A or 10 A fuse would burn before that fuse blew if it shorted."

Fix: In WIRING.md step 12 write: 'Battery unplugged. Buck input: IN+ through the 2 A inline fuse (fuse right at the screw) to BTS7960 B+, IN- to BTS7960 B-. 22 AWG.' Add the 2 A fuse to the section 2 diagram and to the 'You need' list in section 7.

> Skeptic (confirmed): The claim checks out against the actual files. In firmware/base/WIRING.md, the section 2 power diagram (line 48) runs the 20 A fuse to BTS7960 B+ and then straight to MP1584 IN+, with no fuse on that branch. The prose at line 63 says only to take the buck input from B+/B- "so it sits behind the fuse", meaning the 20 A one. Part B step 12 (line 168) says "Buck input: IN+ to BTS7960 B+, IN- to BTS7960 B-" with no inline fuse. The Part A "You need" list (line 136) lists only the 20 A fuse lead. A grep for "fuse" in WIRING.md finds no 2 A fuse anywhere. docs/build/SYSTEM_ELECTRICAL.md line 60 (run
>
> Skeptic (confirmed): I could not refute this. The quotes are accurate and the two documents disagree on a power-safety point.

**What WIRING.md says.** firmware/base/WIRING.md line 168 is step 12 of Part B: "Buck input: IN+ to BTS7960 B+, IN- to BTS7960 B-." It names no fuse. The section 2 power diagram (lines 47-50) and the bullet at line 63 ("Take the buck input from the BTS7960 B+ and B- terminals, so it sits behind the fuse") only ever mean the 20 A fuse. I searched the whole file for "2 A": there is no match. Part A, which powers the buck to set it to 9.0 V, and the "You need" list (line 136) also have no 2 A
>

## 18. CALIBRATION note warns that aim_home_deg can only be set from -720 to 720; the firmware accepts -7200..7200

- File: `firmware/base/CALIBRATION.md:151`
- Severity: low (claimed low); unverified (low)
- Owner: docs

Scenario: A learner who needs aim_home_deg = 3340 reads the note, assumes `set` will refuse it, and stops to contact the firmware author instead of sending the command, which works.

Evidence: CALIBRATION.md:151-152 "at the time of writing the `set` range for `aim_home_deg` is -720 to 720, but the default in `config.h` is 3365. Until that is fixed in the firmware, `set` may refuse the value you need."; base_logic.h:844 `out[n++] = {"aim_home_deg", &c.aimHomeDeg, -7200, 7200};`

Fix: Delete the note (lines 151-152). Change the 'If the reply is OUT_OF_RANGE' sentence at line 144 to say the range is -7200 to 7200.


## 19. Firmware README safety summary gives the wrong current trips (17 A / 15 A while cocking); the firmware trips at 5 A, 6.5 A while cocking, 3 A on jogs, hard 17 A

- File: `firmware/base/README.md:144`
- Severity: low (claimed low); unverified (low)
- Owner: fw-base

Scenario: During the bench test or cocking, the learner sees FAULT OVERCURRENT "wiper over 7 A" or "over 3 A" on a jog. The README says the trip is 17 A (15 A cocking), so the learner thinks it is a false trip or a firmware bug and raises trip_a or cock_trip_a. Those keys can be set up to 19 A, which removes the first-line protection that D8/D26 rely on before the shear pin.

Evidence: README.md:144 "- Over-current trip 17 A (15 A while cocking)."; config.h "float tripA = 5.0f; // aim, reel, tow", "float cockTripA = 6.5f; // cock and fire (peak about 5.1 A)", "float jogTripA = 3.0f;", "float hardTripA = 17.0f; // never exceeded in any state"; base_logic.h:682 "w.c.tripA = c.cockTripA; // ... (D8: 6.5 A while cocking)".

Fix: Change README.md:144 to: 'Over-current trips (D8): 5 A for aim, reel and tow; 6.5 A while cocking and firing; 2.5 A for take-up only; 3 A for jogs; 17 A hard limit in every state. Do not raise trip_a or cock_trip_a to get past a trip: find what binds.' Regenerate the build guide.


## 20. WIRING.md tells the learner to buy an MP1584, which breaks the parts rule and contradicts SYSTEM_ELECTRICAL/MEASURE_LIST (MP1584 #2 is freed by the v4 dart)

- File: `firmware/base/WIRING.md:64`
- Severity: low (claimed low); unverified (low)
- Owner: fw-base

Scenario: The learner reads 'Both of yours are in the dart ... Buy another one' and either orders a part (Harry's rule: only parts in hand plus 608 bearings) or takes the 5.04 V buck out of the dart. The v4 dart WIRING sets only one buck (5.04 V). SYSTEM_ELECTRICAL and MEASURE_LIST C4 say MP1584 #2 is the spare from the dart build, to be reset to 9.0 V.

Evidence: WIRING.md:64-65 "**You may need a spare MP1584.** Both of yours are in the dart (one at 8.12 V, one at 5.04 V). Buy another one, or borrow one from the dart for the bench."; SYSTEM_ELECTRICAL.md:39 "MP1584 #2 is the spare from the dart build (it was the 8.0 V bench supply there): reset it to 9.0 V"; MEASURE_LIST.md:43 "C4 | MP1584 #2 (the spare from the dart build)".

Fix: Replace WIRING.md lines 64-65 with: 'Use MP1584 #2, the spare from the dart build (it was the 8 V bench supply there). Reset it to 9.0 V in Part A.' Regenerate the build guide.


## 21. Dart WIRING step B2 connects the 3S LiPo to the MP1584 without starting from 'battery unplugged' or saying to make the connection before plugging in

- File: `firmware/dart/WIRING.md:97`
- Severity: low (claimed low); unverified (low)
- Owner: fw-dart

Scenario: Path B says 'Every step starts with the battery unplugged' (line 92), but B2 is the only wiring step that does not. A learner could plug the LiPo into XT60 lead #1 first and then clip or solder the live leads onto the MP1584 IN pads, which is wiring with the battery live (the cause of the first XIAO's death). There is also no check that IN+ and IN- are not shorted. The base equivalent (WIRING.md Part A steps 1 to 3) does it in the safe order.

Evidence: firmware/dart/WIRING.md:97-99 '**Step B2. Set the buck before it is wired in** (as in the run sheet step 6.2). The MP1584 on its own: 3S LiPo on XT60 lead #1 to MP1584 IN+ / IN-, meter on DC volts across OUT+ and OUT-, turn the small brass screw until it reads **5.04 V**. Disconnect the LiPo.'; line 92 'Every step starts with the battery unplugged.'

Fix: Rewrite B2: 'Step B2. Battery unplugged. Set the buck before it is wired in. 1) Battery unplugged: solder short leads to MP1584 IN+ (red) and IN- (black), nothing on OUT; connect them to the fused side of XT60 lead #1; meter beep: IN+ to IN- must not beep. 2) Plug the battery in, meter on DC volts across OUT+ and OUT-, turn the brass screw to 5.04 V. 3) Unplug the battery. Then fit the capacitor across OUT...'


## Refuted (dropped)

- `docs/build/TEST_PLAN.md:25` TEST_PLAN stage 0/1 never fits the GPIO12 jumper, so the base boots faulted and every stage 1 jog is refused
