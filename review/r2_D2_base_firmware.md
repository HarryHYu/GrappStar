# Round 2 D2: base firmware re-review (fw-base)

Reviewed at base-v1 commit ba121b9. Method: independent finder subagents, one per lens; every finding of
medium severity or above was checked by two skeptic subagents told to refute it; kept only if fewer than
two refuted it. Low findings are listed unverified. Severity is the finder's, lowered when both skeptics
agreed it is real but rated it lower.

Confirmed: 26. Refuted and dropped: 0.

STATUS: base-v1 moved 250 commits (1285 lines in firmware/base) while this ran. Each finding was re-checked against
863d58d by one checker subagent each; see the Now column.

| # | Sev | File:line | Owner | Finding | Now (863d58d) |
|---|---|---|---|---|---|
| 1 | high | `firmware/base/CALIBRATION.md:210` | fw-base | CALIBRATION.md k_n_per_a procedure ignores the -10.4 N offset and tension_offset_n is not persisted: a calibrated base ends every default tow in a fault at about 75-80 N | fixed |
| 2 | high | `firmware/base/PROTOCOL.md:82` | fw-base | PROTOCOL.md (C1) still documents the old drum sequence; no tidy, retrieve, TIDIED or WRAPPED park (R1 #1 not fixed) | fixed |
| 3 | high | `firmware/base/grappstar_base/base_logic.h:986` | fw-base | retrieve ignores max_s: DART_STUCK always fires at a fixed 20 s, so a normal final approach from 2 m (or any NO_GO retrieval) faults | fixed |
| 4 | high | `firmware/base/grappstar_base/base_logic.h:1269` | fw-base | retrieve ignores max_s: the take-up final approach always faults DART_STUCK after a fixed 20 s, breaking the no-go retrieve | fixed |
| 5 | medium | `firmware/base/PROTOCOL.md:293` | fw-base | PROTOCOL.md (C1) still documents the old trips and tension law: tow at trip_a 5 A, aim without a trip, k_n_per_a 8 with no offset, SAFING to NEUTRAL (R1 #1 still open) | fixed |
| 6 | medium | `firmware/base/grappstar_base/base_logic.h:237` | fw-base | Fault or estop in REEL drives the drum out of REEL to 314 (no unload jog), contrary to params.DRUM.fault_rules and BASE_SPEC 10/10.1 (HOLD); afterwards the step-15 `reel` silently becomes take-up only | fixed |
| 7 | medium | `firmware/base/grappstar_base/base_logic.h:237` | fw-base | A fault in REEL or N_TO_R drives the drum to the WRAPPED park, but params fault_rules, BASE_SPEC 10.1 and PROTOCOL.md say the drum HOLDS (DRUM_HELD "wraps on") | fixed |
| 8 | medium | `firmware/base/grappstar_base/base_logic.h:320` | fw-base | line_m after a power cut comes from the motion-start record: the capstan reel-in pulls the dart into the base | still open |
| 9 | medium | `firmware/base/grappstar_base/base_logic.h:324` | fw-base | The AS5600 multi-turn count is never lined up with the raw angle, so the wiper position is never restored after a power cycle: a cocked or aimed base always boots with phase UNKNOWN and aim unknown | still open |
| 10 | medium | `firmware/base/grappstar_base/base_logic.h:369` | fw-base | Retrying a tow from HOLD restarts a pull already over the limit: the estimate is zeroed and the limit is blind for 350 ms at full duty | still open |
| 11 | medium | `firmware/base/grappstar_base/base_logic.h:412` | fw-base | Drum code returning after 000 (wire fixed, then clear) leaves drumDeg at the sector's low end marked trusted: unwrap faults DRUM_TIMEOUT mid-wrap, mode NEUTRAL parks 6 deg off | still open |
| 12 | medium | `firmware/base/grappstar_base/base_logic.h:755` | fw-base | Boot after a power cut in N_TO_R: drumDeg is guessed at 321 (collar REEL side), `wrap` answers WRAPPED without moving, and tidy and reel are refused for good | still open |
| 13 | medium | `firmware/base/grappstar_base/base_logic.h:791` | fw-base | retrieve (and reel until seated) is accepted in REEL and at the WRAPPED park: the dart is seated on the capstan/through the wraps | fixed |
| 14 | medium | `firmware/base/grappstar_base/base_logic.h:801` | fw-base | Tow trip is a fixed 6.5 A: any max_n above about 75 N can never reach LIMIT/HOLD and always ends in OVERCURRENT (D34 computed trip not implemented) | fixed |
| 15 | medium | `firmware/base/grappstar_base/base_logic.h:886` | fw-base | Power cut during any wiper motion loses phase REST, and decock then silently does nothing: aim is refused for good (no RESET path) | fixed |
| 16 | medium | `firmware/base/grappstar_base/base_logic.h:932` | fw-base | `set trip_a` while a motion runs overwrites that motion's own trip (take-up 2.5 A, jog/fly-in 3 A, tow 6.5 A) | fixed |
| 17 | medium | `firmware/base/grappstar_base/base_logic.h:1308` | fw-base | Drum jog skips the SW_DOG_L/R check and can run back from the WRAPPED park into the wrap window, unwrapping with the wiper stopped (R1 #15 still open) | fixed |
| 18 | medium | `firmware/base/grappstar_base/grappstar_base.ino:82` | fw-base | Wiper IS scaling assumes an averaged IS voltage, but the averaging capacitor is optional and hwWiperIs takes 8 back-to-back point samples: at partial duty the D8 trips depend on PWM phase | fixed |
| 19 | low | `firmware/base/grappstar_base/base_logic.h:111` | fw-base | require_seated=false (and vbat_wired=false) persist silently in NVS: fire without SW_SEATED keeps working after every reboot | still open |
| 20 | low | `firmware/base/grappstar_base/base_logic.h:791` | fw-base | retrieve (until seated) is accepted in REEL and seats the dart on the capstan, against 10.1 step 17 | fixed |
| 21 | low | `firmware/base/grappstar_base/base_logic.h:844` | fw-base | Drum jog backward from the WRAPPED park runs into the WRAP sector and unwraps with the wiper stopped (R1 #15 not fixed) | fixed |
| 22 | low | `firmware/base/grappstar_base/base_logic.h:906` | fw-base | New settable key tension_offset_n is not saved in NVS: a calibrated tension offset is silently lost at every reboot | fixed |
| 23 | low | `firmware/base/grappstar_base/base_logic.h:1331` | fw-base | decock ends on SW_HOME alone and reports RESET ok without SW_GAP closed (R1 #10 still open for decock) | fixed |
| 24 | low | `firmware/base/grappstar_base/base_logic.h:1331` | fw-base | decock ends as REST without SW_GAP; spec requires SW_HOME and SW_GAP (R1 #10 partly open) | fixed |
| 25 | low | `firmware/base/grappstar_base/drives.h:50` | fw-base | Current below duty 60 is not scaled: the 3 A jog trip is blind during the 20 % fly-in and centering jog (R1 #17 still open) | fixed |
| 26 | low | `firmware/base/grappstar_base/drives.h:50` | fw-base | (round 1 #17, still open) Wiper current is not scaled below duty 60: the 3 A jog/fly-in trip cannot be reached at flyDuty 51 | partly fixed |

## 1. CALIBRATION.md k_n_per_a procedure ignores the -10.4 N offset and tension_offset_n is not persisted: a calibrated base ends every default tow in a fault at about 75-80 N

- Now (863d58d): **fixed** at `firmware/base/CALIBRATION.md:241-272; firmware/base/grappstar_base/persist.h:53-62; firmware/base/grappstar_base/base_logic.h:111-126`. CALIBRATION.md now fits both k_n_per_a and tension_offset_n from two pulls (k = (N3-N1)/(I3-I1), offset = N1 - k(I1-free_a)), so the -10.4 offset is no longer left out. Tun now stores tensOffsetN (tunNow/restoreTun handle it), its version was bumped to 2 so old ver-1 records are rejected (validTun checks ver==2), and towTrip() now raises the trip limit to cover (maxN - offset)/k + free_a + 1.

- File: `firmware/base/CALIBRATION.md:210`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: The bench follows CALIBRATION.md (e): a 20 N pull, then k = 20 / (amps - free_a). The firmware law adds tension_offset_n = -10.4 on top. With the params law (17 x (I - If) - 10.4), the procedure gives k = 20 / (30.4/17) = 11.2. The estimate then reaches 50 N only at I = 6.9 A, which is above the 6.5 A tow trip. Every default tow ends in OVERCURRENT or STALL instead of LIMIT/HOLD. Harness (scratch cal2.cpp, set k_n_per_a 11.18 or 7.8, then reel until tension with max_n 50): k 11.18 gives FAULT STALL at a true peak of 81.2 N; k 7.80 gives FAULT OVERCURRENT 'wiper over 6.5 A' at a true peak of 82.8 N. Separately, `set tension_offset_n` is accepted but lost at reboot: after the reboot the harness reads -10.4 again (scratch cal.cpp, tunNow then restoreTun). Tun keeps ver 1, so a board that ran any `set` or `cal` on base-1.0 restores the old k_n_per_a 8 over the new default 17.

Evidence: CALIBRATION.md:210 '5. `k_n_per_a = 20 / (amps - free_a)`. Example: amps 2.6, free_a 1.5: 20 / 1.1 = 18.2.'
base_logic.h:368 'float tc = winding ? (w.amps - c.freeA) * c.kNperA + c.tensOffsetN : 0;'
persist.h:54-56: Tun has kNperA and limitN but no tensOffsetN; 'uint16_t ver = 1' is the same as base-1.0, whose config.h:129 had 'kNperA = 8.0f'.
base_logic.h:906 'out[n++] = {"tension_offset_n", &c.tensOffsetN, -50, 50};' is settable but missing from tunNow() (base_logic.h:111-114) and restoreTun() (119-121).

Fix: CALIBRATION.md step 5: `k_n_per_a = (20 - tension_offset_n) / (amps - free_a)` (that is (20 + 10.4) / (amps - free_a)), with a corrected example. Add tensOffsetN to Tun, tunNow() and restoreTun(), and bump Tun ver to 2 so stale ver-1 records (k 8) are rejected and the v1.2 defaults load.

> Skeptic (confirmed): I checked the code and the claim holds.

1. The estimate includes the offset. base_logic.h:368 computes tc = (w.amps - c.freeA) * c.kNperA + c.tensOffsetN. config.h:172-173 sets the defaults to kNperA 17 and tensOffsetN -10.4, which matches D33/D34 (line N = 17 x (I - I_free) - 10.4).

2. The calibration formula leaves the offset out. CALIBRATION.md:210 says k = 20 / (amps - free_a), and section (e) never mentions tension_offset_n (grep finds it nowhere in that file). On the bench the take-up drag is still in the drivetrain, so 20 N on the line comes at I - If = 30.4/17 = 1.79 A. The formula t
>
> Skeptic (confirmed): I could not refute this in the tree under review (w/review at 5b8c38c). Every cited line says what the claim says.

What I checked in the reviewed tree:
- firmware/base/CALIBRATION.md step 5 says `k_n_per_a = 20 / (amps - free_a)`, and the section never mentions tension_offset_n.
- The firmware estimate is `(amps - freeA) * kNperA + tensOffsetN`, at base_logic.h:368. The defaults are k 17.0 and offset -10.4 (config.h:172-173, params.py tow_tension_model, D33).
- If the real line follows the params law, a 20 N pull happens at I - If = 30.4/17 = 1.79 A. The procedure then gives k = 11.2. With th
>

## 2. PROTOCOL.md (C1) still documents the old drum sequence; no tidy, retrieve, TIDIED or WRAPPED park (R1 #1 not fixed)

- Now (863d58d): **fixed** at `firmware/base/PROTOCOL.md:88-92,248-255,454-475`. PROTOCOL.md has been rewritten. `wrap` now parks at 314 with the collar in NEUTRAL, and the doc says "no MODE event follows". It also documents the `tidy`→TIDIED, `retrieve`→SEATED and `decock` commands and adds the TIDIED event. The section 8 mission now runs wrap→WRAPPED, tidy, mode REEL, then reel until tension, so a Jetson built from the doc no longer waits for a MODE that never comes or gets refused NOT_WRAPPED.

- File: `firmware/base/PROTOCOL.md:82`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: A Jetson or mock built from PROTOCOL.md, as D35 requires ("fw-base owns PROTOCOL.md and documents exactly this"), runs the section 8 mission: `reel until slip` in NEUTRAL before the wrap (tidy before wrapping, against D22), then `wrap` and waits for `WRAPPED` and then `MODE` REEL. The firmware ends the wrap at 314 with only a WRAPPED event (tickWrap sends no MODE), so the Jetson waits for a MODE event that never comes. Its next command, `reel` until tension at 314, is refused NOT_WRAPPED (cmdReel treats LOCK below 318 as neutral and refuses tension). The tow never starts.

Evidence: PROTOCOL.md:82 "`wrap` ... then into REEL with the fly-in (wiper creeping forward at 20 % until `SW_DOG_R` closes)... `WRAPPED` event, then `MODE` (REEL)". PROTOCOL.md:222 "`WRAPPED` | wraps laid and REEL engaged (followed by `MODE`)". PROTOCOL.md:395-401 run the mission as reel slip, wrap -> WRAPPED, MODE REEL, then tow. A grep for `tidy`, `retrieve`, TIDIED or 314 in the 491-line file finds nothing (last change 8f41b6f, before the v1.2 firmware). Code: base_logic.h:761 `startDrumMove(DP_WRAPPED, false)` and base_logic.h:1260 tickWrap emits only WRAPPED. base_logic.h:979 and 986 add the undocumented tidy and retrieve commands.

Fix: Rewrite the PROTOCOL.md command table (wrap, unwrap, add tidy {ms,duty} -> TIDIED, retrieve {duty,max_s} -> SEATED, decock), the event table (WRAPPED = parked at 314, collar NEUTRAL, no MODE; add TIDIED), stop points (0/41/56/314/347) and the section 8 mission so they match BASE_SPEC 10.1 steps 1-18 exactly.

> Skeptic (confirmed): The claim holds. On w/review, firmware/base/PROTOCOL.md was last changed in 8f41b6f and still documents the old drum sequence.
- Line 82: `wrap` goes "into REEL with the fly-in ... `WRAPPED` event, then `MODE` (REEL)".
- Line 222: WRAPPED means "REEL engaged (followed by `MODE`)".
- Line 232 shows an example WRAPPED event followed by MODE REEL.
- Lines 395-396 (section 8 mission) run `reel until slip` in NEUTRAL before `wrap`, then expect "WRAPPED, MODE REEL".
- There is no `tidy`, `retrieve` or `decock` command, no TIDIED event and no 314 park anywhere in the file.

The firmware does somethin
>
> Skeptic (confirmed): The claim holds; I could not refute it. firmware/base/PROTOCOL.md (491 lines, last changed in 8f41b6f) still documents the old sequence:
- Line 82: `wrap` goes on into REEL with the fly-in, and sends `WRAPPED` then `MODE` (REEL).
- Line 222: `WRAPPED` means "REEL engaged (followed by `MODE`)".
- Line 232 shows the example WRAPPED then MODE REEL.
- Line 396, the section 8 mission, runs `reel until slip` before `wrap`, then expects WRAPPED and MODE REEL.
- There is no `tidy`, `retrieve`, `TIDIED`, `decock` command or 314 park anywhere in the file. The only TIDIED hits in the repo's markdown are 
>

## 3. retrieve ignores max_s: DART_STUCK always fires at a fixed 20 s, so a normal final approach from 2 m (or any NO_GO retrieval) faults

- Now (863d58d): **fixed** at `firmware/base/grappstar_base/base_logic.h:1086`. The retrieve handler now calls approachLimit(o, a) before cmdReel (line 1086), and plain reel calls it too (line 1073). approachLimit (lines 847-854) uses max_s (5 to 600 s) when it is given. Otherwise it works out a default from line_m: 10 s + 1.5 x line_m / take-up speed, at least 20 s and at most 600 s. tickReel (line 1375) checks approachMs and prints the real limit in the fault text, so the fixed 20 s DART_STUCK no longer happens.

- File: `firmware/base/grappstar_base/base_logic.h:986`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: Step 17 of BASE_SPEC 10.1: the Jetson sends {"cmd":"retrieve","duty":0.3,"max_s":120} at NEUTRAL with 2.0 m of line out. PC sim (scratch2.cpp): FAULT DART_STUCK "final approach: no SW_SEATED in 20 s" after 20.0 s with 0.40 m still out. With 5 m (a NO_GO retrieval) it faults after 20 s with 3.4 m out. The spec default (20 + line_m/0.08 s) would allow 45 s and 82 s. With 1.6 m the sim seats at 19.9 s, so any normal approach is on the edge of faulting.

Evidence: base_logic.h:986-989 `else if (!strcmp(cmd, "retrieve")) { if (!o.num("duty", &a)) a = 0.3; if (!o.num("max_n", &m)) m = c.limitN; err = cmdReel(a, m, "seated"); }` never reads max_s. base_logic.h:1269 `if (until == U_SEATED && now - opT > c.approachMaxMs) { raise(F_DART_STUCK, "final approach: no SW_SEATED in 20 s"); ...}` and config.h:154 `uint32_t approachMaxMs = 20000;`. BASE_SPEC 10.1 step 17: "`retrieve` {duty 0.3, max_s} | `SEATED` | ... (FAULT `DART_STUCK` after max_s: default 20 + line_m / 0.08 s, the take-up is slow)".

Fix: Add a member `uint32_t approachMs`. In the retrieve handler: `double ms; if (o.num("max_s", &ms) && ms > 0) approachMs = (uint32_t)(ms * 1000); else approachMs = 20000 + (lineKnown ? (uint32_t)(lineM / 0.08f * 1000) : 0);` before calling cmdReel. Set approachMs = c.approachMaxMs for plain `reel until seated`. In tickReel line 1269 use approachMs, and put the real limit into the why text.

> Skeptic (confirmed): I could not refute this. The defect is real.

What the code does:
- The retrieve handler (base_logic.h:986-989) reads only `duty` and `max_n`, then calls `cmdReel(a, m, "seated")`. It never reads `max_s`, and nothing in the firmware tree contains the string max_s.
- `cmdReel` starts the timer at line 802 (`opT = now`).
- In NEUTRAL (`takeupOnly`), `tickReel` line 1269 faults DART_STUCK once `now - opT > c.approachMaxMs`. That limit is a fixed 20000 ms (config.h:154). It does not depend on line length or anything the command sends.

What the spec says:
- BASE_SPEC 10.1 step 17 lists `max_s` as 
>
> Skeptic (confirmed): The core claim holds. In base_logic.h:986-989 the `retrieve` handler reads only `duty` and `max_n` and never looks at `max_s`. tickReel at line 1269 faults DART_STUCK once `c.approachMaxMs` passes, and config.h:154 fixes that at 20000 ms. Nothing computes the spec's default (20 + line_m/0.08 s). That breaks the C1 command table, BASE_SPEC 10.1 step 17, which DECISIONS D35 makes the single agreed sequence. So this is a real gap between the code and the contract.

Some points pull the severity down from high:
(1) The prose summary at BASE_SPEC.md:157 says "fault DART_STUCK after 20 s", so the sp
>

## 4. retrieve ignores max_s: the take-up final approach always faults DART_STUCK after a fixed 20 s, breaking the no-go retrieve

- Now (863d58d): **fixed** at `firmware/base/grappstar_base/base_logic.h:847-853,1086,1375`. The retrieve and reel handlers now call approachLimit(), which reads max_s (5 to 600, BAD_ARG outside that range). Without max_s it uses max(20, 10 + 1.5*line_m/v), or 30 s when line_m is unknown, capped at 600 s. tickReel checks `now - opT > approachMs`, and the fault text now reports the actual limit. test_base.cpp:888-892 tests this and PROTOCOL.md:91 documents it. One small leftover: the comment at line 843 still says "DART_STUCK after 20 s".

- File: `firmware/base/grappstar_base/base_logic.h:1269`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: No-go landing, line_m 4.5. Per BASE_SPEC 10.1 the Jetson skips to step 17 and sends retrieve {duty 0.3, max_s 80}. The spec default timeout is 20 + 4.5/0.08 = 76 s. The firmware ignores max_s and raises DART_STUCK at 20 s. Harness (scratch.cpp, NEUTRAL, lineOut 4.5, `retrieve` with max_s 80): FAULT DART_STUCK at t=25.6 s with 2.9 m of line still out. A normal mission (line_m about 1.5, about 18 s at 0.08 m/s) is also within margin of the fixed 20 s.

Evidence: base_logic.h:986-988 'else if (!strcmp(cmd, "retrieve")) { if (!o.num("duty", &a)) a = 0.3; if (!o.num("max_n", &m)) m = c.limitN; err = cmdReel(a, m, "seated");' (max_s never read)
base_logic.h:1269 'if (until == U_SEATED && now - opT > c.approachMaxMs) { raise(F_DART_STUCK, "final approach: no SW_SEATED in 20 s");'
config.h:154 'uint32_t approachMaxMs = 20000;'
BASE_SPEC 10.1 step 17: 'FAULT `DART_STUCK` after max_s: default 20 + line_m / 0.08 s, the take-up is slow'

Fix: In the retrieve handler, read max_s (default 20 + (lineKnown ? lineM : lineTotalM) / 0.08) and store it in a member approachMs (bounded, for example 5..600 s). tickReel then uses `now - opT > approachMs`, and the fault text no longer hard-codes 20 s. Document max_s in PROTOCOL.md.

> Skeptic (confirmed): I could not refute this. The defect is real.

1) Binding contract: BASE_SPEC 10.1 is labelled "binding, D35". Commit c21a2d4 changed step 17 to `retrieve` {duty 0.3, max_s}, with "FAULT DART_STUCK after max_s: default 20 + line_m / 0.08 s". Step 7 sends a NO_GO landing to 14, then 16 (skipped when never wrapped), then 17. So an unwrapped no-go dart is retrieved from the full landing distance on the take-up alone.

2) Code: base_logic.h:986-988 (the `retrieve` handler) reads only duty and max_n. It never reads max_s and never works out a line-based default. tickReel at base_logic.h:1269 faults 
>
> Skeptic (confirmed): I could not refute this claim. The code says what the reviewer says it does. In base_logic.h around lines 986-988, the `retrieve` handler reads only `duty` and `max_n`. It never looks at `max_s` and passes nothing about time to cmdReel. On the take-up-only path, tickReel at line 1269 raises F_DART_STUCK once now - opT is more than c.approachMaxMs. That value is fixed at 20000 ms (config.h:154), and nothing per command changes it. The other timeout, reelMaxMs, is 240 s and does not help here. The binding spec says otherwise. BASE_SPEC 10.1 is the binding C1 sequence (D35). Its step 17 is `retri
>

## 5. PROTOCOL.md (C1) still documents the old trips and tension law: tow at trip_a 5 A, aim without a trip, k_n_per_a 8 with no offset, SAFING to NEUTRAL (R1 #1 still open)

- Now (863d58d): **fixed** at `firmware/base/PROTOCOL.md:340-347`. PROTOCOL.md was rewritten for base-1.2.1 and now matches the code. The trip table and lines 103-109 give the tow trip as max(tow_trip_a 6.5 A, the computed limit), and aim/reel-in as trip_a 5 A, which is what base_logic.h:740/753/876 does. The tension estimate is 17 x (amps - free_a) + tension_offset_n (default -10.4), and tension_offset_n is in the key table at line 429. The fault response at lines 374-396 says the drum holds with the wraps on instead of going to NEUTRAL. The tidy, retrieve and decock rows are at lines 90-92.

- File: `firmware/base/PROTOCOL.md:293`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: Commit 01f1fe0 changed the firmware to the v1.2 rules (tow trip 6.5 A, aim 5 A, N = 17 x (I - If) - 10.4, faults in REEL go to 314) but did not touch PROTOCOL.md. A Jetson or mock built from C1 expects a tow to trip at 5 A and computes tension with k 8 and no offset (about twice the firmware's estimate per D32). It reads SAFING as 'drum to NEUTRAL' and does not know the tension_offset_n key or the tidy, retrieve and decock commands (D35). Its limit settings and fault handling then disagree with the base.

Evidence: PROTOCOL.md:92 'In REEL (capstan, wiper FORWARD, trip `trip_a`):'
PROTOCOL.md:94 'Tension = load cell when one is live, else the estimate `(amps - free_a) x k_n_per_a`'
PROTOCOL.md:293 '| reel, tow, `cal decock`, and the default `trip_a` | 5 A |'
PROTOCOL.md:298 '| aim | no trip of its own: it keeps the trip the previous motion left (2.5, 3 or 5 A). Known firmware issue, expected to become 5 A |'
PROTOCOL.md:366 '| `k_n_per_a` | 8 | ...'
PROTOCOL.md:157 '`SAFING` (fault: drum on its way to NEUTRAL)'
versus base_logic.h:801, 683/696, 368 and config.h:63/172-173

Fix: Rewrite the PROTOCOL.md reel section and the trip table to match the code and D33/D34: tow 6.5 A (or the computed trip), reel-in 5 A, aim 5 A of its own, estimate 17 x (amps - free_a) + tension_offset_n (default -10.4), with tension_offset_n in the key table. Fix the SAFING and fault-rule text, and add the tidy, retrieve {duty, max_s} and decock rows per BASE_SPEC 10.1.

> Skeptic (confirmed): I could not refute this. The claim is accurate. `git log -- firmware/base/PROTOCOL.md` shows the file was last changed in 8f41b6f, the base-1.0 contract. Commit 01f1fe0 (firmware to spec v1.2) changed base_logic.h, config.h, params_gen.h and the tests, but not PROTOCOL.md.

What PROTOCOL.md still says:
- line 3: "Firmware `base-1.0`"
- line 293: reel, tow and decock trip at 5 A
- line 298: aim has no trip of its own, "expected to become 5 A"
- line 94: estimate is (amps - free_a) x k_n_per_a, with no offset
- line 366: `k_n_per_a` default 8
- line 157: SAFING goes to NEUTRAL
- line 82: `wrap` 
>
> Skeptic (confirmed): The claim holds. Commit 01f1fe0 changed only base_logic.h, config.h, params_gen.h and the tests. PROTOCOL.md was last touched by 8f41b6f, which documents base-1.0, so every cited line is still there:
- PROTOCOL.md:92 says reel trips at trip_a.
- PROTOCOL.md:94 gives the estimate as (amps - free_a) x k_n_per_a.
- PROTOCOL.md:293 puts reel and tow at 5 A.
- PROTOCOL.md:298 says aim has no trip of its own.
- PROTOCOL.md:366 gives k_n_per_a a default of 8.
- PROTOCOL.md:157 describes SAFING as the drum going to NEUTRAL.

The code says otherwise:
- base_logic.h:801 picks the reel trip as `w.c.tripA
>

## 6. Fault or estop in REEL drives the drum out of REEL to 314 (no unload jog), contrary to params.DRUM.fault_rules and BASE_SPEC 10/10.1 (HOLD); afterwards the step-15 `reel` silently becomes take-up only

- Now (863d58d): **fixed** at `firmware/base/grappstar_base/base_logic.h:255`. safeToNeutral now returns "wraps on: the drum holds..." when sec >= SEC_WRAP, and the Sector enum orders WRAP < LOCK < REEL (config.h:24). So in REEL, raise() (line 231-237) and the estop path (line 1155-1159) no longer call startDrumMove: atPark(DP_WRAPPED) is false, so they emit MODE err DRUM_HELD and the collar stays in REEL.

- File: `firmware/base/grappstar_base/base_logic.h:237`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: A tow faults (for example OVERCURRENT). raise() sees sec == SEC_REEL, safeTarget() returns DP_WRAPPED and safeToNeutral() returns nullptr, so a safing move pulls the collar out of the loaded REEL dogs with the wiper off and no unload jog. params 1.2.6 fault_rules says 'the drum HOLDS where it is (D27, MODE err DRUM_HELD)', and BASE_SPEC 10.1 recovery says the drum HOLDS with the wraps on and the Jetson reads DRUM_HELD. Harness (scratch.cpp): after the tow fault the drum is at WRAPPED 314.0 and the event is 'MODE ok:true mode NEUTRAL'. After `clear`, the C1 step 15 `reel` {until stop} is accepted with ok:true but runs take-up only, because the park counts as neutral. line_m never counts down, so the Jetson's stop at about 1.5 m never comes (only the 240 s timeout ends it). The cmdStop(true) estop path does the same.

Evidence: base_logic.h:237 'int safeTarget() const { return (sec == SEC_REEL || sec == SEC_LOCK) ? DP_WRAPPED : DP_NEUTRAL; }'
base_logic.h:250 'if (sec == SEC_WRAP) return "inside the wrap window: the operator decides (unwrap)";   // REEL / N_TO_R go to WRAPPED'
base_logic.h:519 'if (engaged && c.centerDeg > 0 && enc.ok && !safing)' (safing skips the unload jog)
base_logic.h:1050-1052 (estop path)
params.py:150 'fault_rules=dict(from_REEL_or_N_TO_R="the drum HOLDS where it is (D27, MODE err DRUM_HELD), never NEUTRAL ...'

Fix: Follow params.DRUM.fault_rules and BASE_SPEC 10.1: in safeToNeutral, return "wraps on: the drum holds (D27)" when sec >= SEC_WRAP (REEL, N_TO_R, WRAP), so raise() and estop emit MODE err DRUM_HELD and leave the collar in REEL. Ask the coordinator to delete the conflicting sentence in BASE_SPEC 7 / D31 ('goes to WRAPPED').

> Skeptic (confirmed): I could not refute this. The code does what the claim says, and the newest spec text asks for something else.

Code (firmware/base/grappstar_base/base_logic.h):
- raise(), lines 226-228: hold = safeToNeutral(f); st = safeTarget(). If hold is nullptr and the drum is not already at st, it calls startDrumMove(st, true).
- safeTarget(), line 237: for SEC_REEL or SEC_LOCK (N_TO_R) it returns DP_WRAPPED (314).
- safeToNeutral(): returns a hold reason for SEC_WRAP. For REEL or N_TO_R it gives none unless one of the other checks fires (drum fault, invalid code, hand mode, wraps unknown, loaded). So a 
>
> Skeptic (confirmed): I could not refute this claim. The code does what the claim says, and the newest binding spec says the opposite.

What the code does (base_logic.h):
- safeTarget() (line 237) returns DP_WRAPPED when sec is SEC_REEL or SEC_LOCK.
- safeToNeutral() (lines 245-253) only returns a hold reason for SEC_WRAP, a drum fault, an invalid code, hand mode, unknown wraps, or a loaded launch train. REEL and N_TO_R return nullptr.
- So raise() (lines 226-228) starts a safing drum move to 314. startDrumMove's centering jog is skipped when safing (line 519, `!safing`), so the collar leaves the REEL dogs with no 
>

## 7. A fault in REEL or N_TO_R drives the drum to the WRAPPED park, but params fault_rules, BASE_SPEC 10.1 and PROTOCOL.md say the drum HOLDS (DRUM_HELD "wraps on")

- Now (863d58d): **fixed** at `firmware/base/grappstar_base/base_logic.h:255`. safeToNeutral now returns "wraps on: the drum holds..." whenever sec >= SEC_WRAP, which covers WRAP, LOCK and REEL (config.h:24 enum). raise() (line 231-237) and estop in cmdStop (line 1155-1159) both skip startDrumMove when hold is set and send MODE ok:false err DRUM_HELD, as params fault_rules and BASE_SPEC line 194 require. safeTarget still returns DP_WRAPPED for REEL/LOCK, but the code no longer reaches it while the wraps are on. It is kept on purpose as a guard (comment at line 240).

- File: `firmware/base/grappstar_base/base_logic.h:237`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: An OVERCURRENT (or any fault) during the tow in REEL: the firmware enters SAFING, drives the drum without a centering jog from 347 back to 314 and sends MODE ok:true drum_pos WRAPPED. PC sim (scratch.cpp case D): FAULT OVERCURRENT, then MODE {ok:true, mode:NEUTRAL, drum_pos:WRAPPED} at 314. Under the current contract the Jetson expects MODE ok:false err DRUM_HELD and offers unwrap or retry. Instead the collar has silently left REEL, so the "retry" offered by 10.1 (re-tow) is refused NOT_WRAPPED until a new `mode REEL` fly-in. estop (cmdStop line 1051) does the same.

Evidence: base_logic.h:237 `int safeTarget() const { return (sec == SEC_REEL || sec == SEC_LOCK) ? DP_WRAPPED : DP_NEUTRAL; }`. base_logic.h:227-228 raise() starts startDrumMove(st, true). safeToNeutral (245-254) only holds for SEC_WRAP. params.py:150 (v1.2.6) `from_REEL_or_N_TO_R="the drum HOLDS where it is (D27, MODE err DRUM_HELD), never NEUTRAL..."`. BASE_SPEC 10.1 Recovery (commit a388984): "A fault with the wraps on (REEL, N_TO_R, the WRAP sector or the park) HOLDS the drum where it is (D27)". PROTOCOL.md:336 documents why "wraps on: the line must be unwrapped by the operator | WRAP sector, lock or REEL", a string the code never emits. (BASE_SPEC 7 and D31 still say WRAPPED: the coordinator changed the rule after the firmware.)

Fix: In safeToNeutral, return "wraps on: the line must be unwrapped by the operator" when `sec >= SEC_WRAP` (in place of the SEC_WRAP-only line 250), so raise() and cmdStop(estop) send DRUM_HELD. Remove the WRAPPED target from safeTarget, or keep it only if the coordinator restores the D31 wording, and align BASE_SPEC 7.

> Skeptic (confirmed): I could not refute this. The code path is real. In base_logic.h:237, safeTarget() returns DP_WRAPPED for SEC_REEL and SEC_LOCK. safeToNeutral() (lines 245-254) returns a hold reason for SEC_WRAP only; its comment says "REEL / N_TO_R go to WRAPPED". So in raise() (lines 226-228), a fault in REEL starts startDrumMove(DP_WRAPPED, true) and enters S_SAFING, which ends with MODE ok:true. test_base.cpp:506 even asserts this for estop ("drum to the WRAPPED park, never NEUTRAL").

That contradicts the current contract text:
- params.py:150 fault_rules (the canonical params file, v1.2.6) now says the d
>
> Skeptic (confirmed): I checked the claim against the code and it holds. In base_logic.h:237, safeTarget() returns DP_WRAPPED when the drum sector is SEC_REEL or SEC_LOCK. safeToNeutral() (lines 245-254) only refuses to move for SEC_WRAP, so a fault in REEL or N_TO_R lets raise() (lines 226-228) start a safing move to the 314 park and send MODE ok:true drum_pos WRAPPED. cmdStop with estop (lines 1049-1053) does the same. The repo's own test confirms this is intended: test_base.cpp:506 checks "estop with wraps on: drum to the WRAPPED park, never NEUTRAL".

The newer and binding contract says something different:
- p
>

## 8. line_m after a power cut comes from the motion-start record: the capstan reel-in pulls the dart into the base

- Now (863d58d): **still open** at `firmware/base/grappstar_base/base_logic.h:331`. resolveBoot still does `if (s && s->lineM >= 0) { lineM = s->lineM; lineKnown = true; }` and never checks `s->moving & MV_WIPER`. The capstan reel start (line 877, and 1400) still calls markMoving(MV_WIPER), which saves the starting line_m (line 265). After a power cut mid-reel, boot therefore reports that stale value as known.

- File: `firmware/base/grappstar_base/base_logic.h:320`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: Step 15: capstan reel-in from line_m 8. Power is cut at a true 3.0 m. The flash record still holds line_m 8.0, written by markMoving at the reel start, with the moving bit set. At boot line_m = 8.0 and is reported as known. The Jetson resumes `reel until stop` and sends stop at line_m 1.5, per 10.1. PC sim (scratch3.cpp): fw line_m 1.50 when the true line out is 0.00. The dart is reeled all the way in on the capstan at duty 0.8, against 10.1 step 17 "Never seat on the capstan", until the 50 N tension limit stops it.

Evidence: base_logic.h:320 `if (s && s->lineM >= 0) { lineM = s->lineM; lineKnown = true; } else { lineKnown = false; }` ignores `s->moving & MV_WIPER`. base_logic.h:270 `markMoving(...)` -> persist() writes the snapshot only at motion start. base_logic.h:1151 the final values are written only once everything is still for 300 ms.

Fix: In resolveBoot: `if (s && s->lineM >= 0 && !(s->moving & MV_WIPER)) { lineM = s->lineM; lineKnown = true; } else lineKnown = false;`. The Jetson then has to `set line_m` again, as after a shot.

> Skeptic (confirmed): The claim checks out against the code. base_logic.h:320 restores lineM from the boot record and sets lineKnown = true with no check of s->moving. Two lines above, at :306, the wiper angle, aim and cock phase are all distrusted when MV_WIPER is set. lineM is integrated from wiper rotation during reel (:400). It is only persisted in three ways: by markMoving at motion start (:802 cmdReel, :270 to persist), through the dirty flag, or by the still-for-300 ms write (:1151). None of those runs mid-reel, so after a power cut during a capstan reel the record holds the start value (8.0) with MV_WIPER s
>
> Skeptic (confirmed): I could not refute this; the defect is real. In base_logic.h, line 400 lowers lineM continuously as the wiper turns forward during a capstan reel. The flash record is written only at motion start, through markMoving -> persist (lines 270 and 802), and again once everything has been still for 300 ms (line 1151). So after a power cut mid-reel, the record holds the line_m from the start of the reel with MV_WIPER set in moving. resolveBoot does not trust the wiper and drum fields when their moving bits are set (lines 306-307), but line 320 restores lineM and sets lineKnown=true without checking s-
>

## 9. The AS5600 multi-turn count is never lined up with the raw angle, so the wiper position is never restored after a power cycle: a cocked or aimed base always boots with phase UNKNOWN and aim unknown

- Now (863d58d): **still open** at `firmware/base/grappstar_base/base_logic.h:334`. The bug is still there. resolveBoot still sets wiperOk only when wrap180(enc.absDeg() - wrap360(s->wiperDeg)) is under 3 degrees (lines 334-337). Nothing lines the multi-turn count up with the raw angle when the record can't be used: the only setDeg call is that restore, and the first reading in sensors.h:82 sets lastRaw but leaves count at 0. So when the magnet's raw angle at first boot isn't near 0, every power cycle still boots with phase UNKNOWN and aim unknown. The persistent-glitch resync at sensors.h:93 also still drops the step without counting it, so the count can still drift out of line with the raw angle.

- File: `firmware/base/grappstar_base/base_logic.h:324`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: Real board, magnet at any raw angle other than about 0 at first boot (for example raw 1000 = 88 deg). The base aims, cocks and sits idle, so the record is written with moving=0 and wiperKnown=1. Then the power is cycled. resolveBoot computes diff = wrap180(absDeg - wrap360(wiperDeg)), which is about 88 deg, so it is not under 3, and wiperOk stays false. The base comes up with phase UNKNOWN and aimKnown false: aim is refused (PHASE_UNKNOWN), cock is refused (LOADED_UNKNOWN), and an operator decock plus re-home is needed. Nothing ever re-aligns the count, so this happens on every reboot and NVS wiper persistence never works. Proven in /tmp/claude-0/review-scratch-D2/test/scratch.cpp: a copy of test 3 of testPowerLoss with S.wiperDeg = 88 at build time gives 'phase 5 (UNKNOWN), aimKnown 0' after the power cycle. With offset 0 (all the PC tests) it gives COCKED and aim known. The sim starts the shaft at raw 0 (sim.h:174), which hides the bug.

Evidence: sensors.h:78-79 `if (!have || !ok) { // first reading or back after a loss: take it as is` / `lastRaw = raw & 4095; have = true; ok = true; ...` (count stays 0 whatever the raw angle). sensors.h:88 the persistent-glitch resync `lastRaw = raw & 4095; glitchRun = 0; return;` drops counts uncounted. base_logic.h:324-325 `float diff = wrap180(enc.absDeg() - wrap360(s->wiperDeg));` / `if (fabsf(diff) < 3.0f) { enc.setDeg(s->wiperDeg + diff); wiperOk = true; }`. The only other setDeg/count writes are the index-mode and enc_sign paths (base_logic.h:296, 302, 926) and the 4096*1000 renormalisation (1145-1146).

Fix: In resolveBoot, right after the wiperTrust block (after base_logic.h:326), add `if (enc.ok && !wiperOk && !indexMode) enc.setDeg(enc.absDeg());` so that wrap360(enc.deg()) == absDeg() from the first boot on. Checked in scratch: the full test_base suite still passes 311/311 and the 88-deg power-cycle case restores COCKED with aim known. Also make sensors.h:88 keep the alignment: on a persistent-glitch resync do `count += d * sign` before taking the new lastRaw, instead of dropping the step. Add a power-cycle test in which the sim's wiperDeg does not start at 0.

> Skeptic (confirmed): I could not refute this. The claim checks out against the unmodified code.

1. sensors.h:78-79 takes the first magnet reading (and the first one after a loss) without touching `count`. `count` starts at 0, so at boot enc.deg() is 0 even though absDeg() is the raw angle.
2. Grepping the firmware finds only these writes to `count`: setDeg in resolveBoot (base_logic.h:325, and only when diff < 3), the index-mode lines (296, 302), the enc_sign flip (926) and the renormalisation (1145-1146). None of them lines `count` up with the raw angle on a normal AS5600 boot, and homing does not do it either.

>
> Skeptic (confirmed): The claim holds. In sensors.h:78-79 the first good reading (and any reading after a loss) sets lastRaw = raw but leaves count at 0, so deg() is 0 while absDeg() equals the raw angle. Nothing in the firmware lines the two up afterwards. The only writes to count are the index-mode lines (base_logic.h:296, 302), the enc_sign negation (926), the 4096*1000 renormalisation (1145-1146) and setDeg, and setDeg is called only inside the wiperTrust block at base_logic.h:325. The saved record stores s.wiperDeg = enc.deg() (base_logic.h:259), which counts from that arbitrary start. After a power cycle, bas
>

## 10. Retrying a tow from HOLD restarts a pull already over the limit: the estimate is zeroed and the limit is blind for 350 ms at full duty

- Now (863d58d): **still open** at `firmware/base/grappstar_base/base_logic.h:858`. cmdReel (858-878) still accepts a reel from S_HOLD through idleCheck (681) with no over-limit check, and sets state = S_REEL. In readSensors (387), tCur is still zeroed when not winding unless the state is S_HOLD, so it reads 0 through the tensionIgnoreMs window. tickReel (1390) still skips the limit until now - w.startT > tensionIgnoreMs. BASE_SPEC.md:194 still requires that "a retry never restarts a pull that is already over the limit".

- File: `firmware/base/grappstar_base/base_logic.h:369`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: The tow is holding at LIMIT. The Jetson (or operator) sends reel {until tension, max_n 50} again; idleCheck accepts it from S_HOLD. state becomes S_REEL, so readSensors resets tCur to 0 during the 350 ms ignore window, and tickReel skips the limit check for 350 ms. The wiper ramps to full duty against a line already at or over the limit, and the estimate has to rebuild from 0 with no taper. Harness (scratch.cpp): stiff anchor K 400: hold at 54.5 N, retry peaks at 67.3 N. Rigid anchor K 2000: hold at 58.9 N, retry peaks at 91.3 N and ends in FAULT. BASE_SPEC 10.1: 'a retry never restarts a pull that is already over the limit'.

Evidence: base_logic.h:631 'if (state != S_IDLE && state != S_HOLD) return "BUSY";' (HOLD accepted)
base_logic.h:367-369 'bool winding = w.outDir == c.reelDir && sec == SEC_REEL && now - w.startT > c.tensionIgnoreMs; ... tCur = winding ? tCur + (tc - tCur) * 0.2f : (state == S_HOLD ? tCur : 0);'
base_logic.h:1284 'if (frac >= 1.0f && now - w.startT > c.tensionIgnoreMs) { w.halt(now, io.wiper); enterHold("tension limit"); return; }'

Fix: In cmdReel: if state == S_HOLD (or holdLimit) and un == U_TENSION and tensionN >= maxN * c.resumeFrac, do not start the wiper. Re-emit LIMIT holding:true (or refuse with OVER_LIMIT). Also keep tCur seeded through the ignore window after a hold (do not zero it when holdLimit is set), so the taper and the limit act from the first tick.

> Skeptic (confirmed): I could not refute this. The code does what the claim says.
- idleCheck (base_logic.h:629-635) lets S_HOLD through. Every other motion command adds `if (state == S_HOLD) return "HOLDING"` (cmdMode, cmdAim, cmdUnwrap and others at lines 650, 674, 701, 721, 754, 769, 823). cmdReel (784-805) has no such guard and no check of tension against maxN. It clears holdLimit, calls w.want(reelDir, reelDutyCmd) and sets state = S_REEL.
- w.want restarts the drive: drives.h:64 sets startT = now and outDuty = 0, and the duty then ramps up.
- readSensors (367-369): for 350 ms after startT (tensionIgnoreMs = 3
>
> Skeptic (confirmed): I read the code and the claim holds up. In base_logic.h, cmdReel (line 784) calls idleCheck(true), and idleCheck (line 631) lets S_HOLD through. Several other commands refuse HOLD with an explicit "if (state == S_HOLD) return HOLDING": cmdMode, cmdAim and cmdJog do. cmdReel has no such check, and it never compares the current tensionN with maxN or resumeFrac. It clears holdLimit, sets state = S_REEL and calls w.want(reelDir, reelDutyCmd) at the full commanded duty, which starts a new wiper run, so w.startT becomes now.

In readSensors (lines 367-369), winding is false until now - w.startT > te
>

## 11. Drum code returning after 000 (wire fixed, then clear) leaves drumDeg at the sector's low end marked trusted: unwrap faults DRUM_TIMEOUT mid-wrap, mode NEUTRAL parks 6 deg off

- Now (863d58d): **still open** at `firmware/base/grappstar_base/base_logic.h:438`. updateSector still does `if (was == SEC_INVALID || !booted) { drumKnown = true; return; }` without setting drumDeg, wraps or wrapsKnown, and degTrusted still defaults to true (line 530). So when the drum code comes back after a 000 boot, drumDeg stays at 0, gets clamped to the sector's low edge (line 464) and is treated as trusted. resolveBoot (lines 321-329) was the only part changed, so the scenario still happens; I found this by reading the code and did not run a test.

- File: `firmware/base/grappstar_base/base_logic.h:412`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: Boot with the common wire off (code 000): DRUM_SENSOR and DRUM_HELD, which is correct. The operator fixes the wire and sends `clear`. The sector returns while booted, so updateSector sets drumKnown but never sets drumDeg: drumDeg stays 0 and is clamped to the sector's low edge, with degTrusted still at its default true. PC sim (scratch.cpp case E): drum truly at 180 in WRAP, the firmware believes 62 (trusted); `unwrap` expects about 2.6 s, so after 5.4 s it raises DRUM_TIMEOUT "drum did not reach the target in time" at true 140, wraps half removed, drum HELD. Case F: at NEUTRAL (true 39) the firmware believes 32; `mode NEUTRAL` drives 1.1 s forward and parks at true 47.1 while reporting 41, close to the 50 code edge.

Evidence: base_logic.h:412 `if (was == SEC_INVALID || !booted) { drumKnown = true; return; }` sets no drumDeg and leaves degTrusted and wrapsKnown as they were. base_logic.h:493 `bool degTrusted = true;` is the default. base_logic.h:437 trackDrum `if (drumDeg < lo) drumDeg = lo;` clamps 0 to the sector's low edge. resolveBoot (line 319) only runs once, and with the sector invalid it skipped the angle rebuild.

Fix: In updateSector, when `was == SEC_INVALID && booted`, rebuild the drum state like resolveBoot does: `int k = detentClosed(); drumDeg = k >= 0 ? c.targetDeg[k] : <sector middle>; degTrusted = k >= 0; wraps = wrapsAt(sec, drumDeg); wrapsKnown = sec != SEC_WRAP; drumKnown = true;`. Initialise `degTrusted = false`.

> Skeptic (confirmed): I traced the path in firmware/base/grappstar_base/base_logic.h and it is real. I read the code but did not run the sim.

1. Boot with code 000. updateSector (line ~404) returns early on SEC_INVALID, so sec stays SEC_INVALID. resolveBoot (line 319) then takes the else branch: drumKnown=false, wrapsKnown=false, wraps=0. It leaves drumDeg (0, line 70) and degTrusted (true, line 493) untouched. The secBad timer then raises F_DRUM_SENSOR.
2. Fixing the wire does not trip "drum code changed with the drum still". quietSec was SEC_INVALID, and the check requires quietSec != SEC_INVALID, so it just cop
>
> Skeptic (confirmed): I could not refute this; the defect is real. Code: base_logic.h:412 `if (was == SEC_INVALID || !booted) { drumKnown = true; return; }` returns without touching drumDeg, degTrusted or wrapsKnown. drumDeg starts at 0 (line 70) and degTrusted at true (line 493). resolveBoot (lines 309-319) only rebuilds the angle when sec != SEC_INVALID at boot, and it runs once. After that, the only places that write drumDeg are a real adjacent edge (line 428), the trackDrum clamp (line 437, which pulls 0 up to secLo) and the end of a move (line 588). cmdClear (line 1061) only asks that the code be valid again. 
>

## 12. Boot after a power cut in N_TO_R: drumDeg is guessed at 321 (collar REEL side), `wrap` answers WRAPPED without moving, and tidy and reel are refused for good

- Now (863d58d): **still open** at `firmware/base/grappstar_base/base_logic.h:815`. resolveBoot (line 327) now sets degTrusted=false after a cut during a drum move, but it still guesses drumDeg at the sector middle (321, above collarNdeg 318). Nothing reads that flag: cmdWrap still answers WRAPPED at once for any SEC_LOCK/SEC_REEL with no atPark or degTrusted check and no re-reference move, and tidy (line 1078) and reel (line 864) still use the untrusted drumDeg < collarNdeg, so they still refuse NOT_IN_NEUTRAL / NOT_WRAPPED.

- File: `firmware/base/grappstar_base/base_logic.h:755`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: Power is cut during step 9 (`wrap`) after the drum entered N_TO_R (true 306, record has MV_DRUM set). At boot drumDeg = 321 (sector middle, untrusted), so modeName is NONE. The Jetson repeats `wrap`: ok plus an immediate `WRAPPED` {wraps:7}, but the drum does not move. Step 11 `tidy` is refused NOT_IN_NEUTRAL and `reel until slip` is refused NOT_WRAPPED. `mode NEUTRAL` is refused WRAPS_ON. PC sim (scratch.cpp case A): boot drumDeg(fw)=321.0 true=306.0 mode=NONE; wrap -> ok, WRAPPED; tidy -> NOT_IN_NEUTRAL. The base never reaches the 314 park, and the sequence stalls at the tidy unless the operator skips it.

Evidence: base_logic.h:316 `drumDeg = k >= 0 ? c.targetDeg[k] : 0.5f * (fmaxf(secLo(sec), c.targetDeg[0]) + fminf(secHi(sec), c.targetDeg[2])); degTrusted = k >= 0;` gives 0.5*(302+340) = 321 in LOCK. base_logic.h:755 `if (sec == SEC_LOCK || sec == SEC_REEL) { ... "WRAPPED" ... return nullptr; }` returns without checking atPark(DP_WRAPPED). base_logic.h:981 tidy needs `(sec == SEC_LOCK && drumDeg < c.collarNdeg)` with collarNdeg = 318.

Fix: In cmdWrap, answer at once only when `atPark(DP_WRAPPED) && degTrusted` (or when in REEL). Otherwise, in LOCK with !degTrusted, re-reference the drum: drive backward until the WRAP code (302 edge) appears. updateSector snaps drumDeg and sets degTrusted. Then drive forward to 314 with startDrumMove(DP_WRAPPED) and send WRAPPED at the end. Running back past the 302 edge by less than 1 deg does not change the wrap count (wrapsAt uses floor(f+0.15)).

> Skeptic (confirmed): I could not refute this. The cited lines say what the claim says.

1. Boot (base_logic.h:309-318). If the saved record has MV_DRUM set, drumTrust is false. In SEC_LOCK, detentClosed() returns -1 (line 141-142), so drumDeg is set to 0.5*(fmaxf(302, targetDeg[0]=0) + fminf(340, targetDeg[2]=347)) = 321 and degTrusted is false. config.h:123-128 has targetDeg = {0,41,347,314} in the order LAUNCH, NEUTRAL, REEL, WRAPPED, and config.h:121 puts the edges at 302 and 340. 321 is at or above collarNdeg (318, config.h:129), so modeName() is not NEUTRAL (line 158).

2. `wrap` (line 755). It returns WRAPPE
>
> Skeptic (confirmed): I checked the code and the claim holds. The one overstatement is "refused for good".

**Boot.** In resolveBoot (base_logic.h:309-318), a record with MV_DRUM set gives drumTrust=false. detentClosed() (line 141) returns -1 for SEC_LOCK, so drumDeg falls back to 0.5*(max(302,targetDeg[0]) + min(340,targetDeg[2])). That is 0.5*(302+340) = 321, with degTrusted=false. modeName() (line 158) says NEUTRAL in LOCK only when drumDeg < collarNdeg (318), so the mode is NONE.

**wrap.** cmdWrap (line 755) sends the WRAPPED event and returns success whenever sec is LOCK or REEL. It never checks atPark(DP_WRA
>

## 13. retrieve (and reel until seated) is accepted in REEL and at the WRAPPED park: the dart is seated on the capstan/through the wraps

- Now (863d58d): **fixed** at `firmware/base/grappstar_base/base_logic.h:868`. cmdReel now returns WRAPS_ON for until=seated whenever sec != SEC_NEUTRAL, so it is refused in REEL and at the WRAPPED park (SEC_LOCK). retrieve goes through the same check. In NEUTRAL with a dog contact closed, the call is still refused (NOT_WRAPPED via line 865). Tests at test_base.cpp:460 and :842 cover both cases, and PROTOCOL.md:91 documents the change.

- File: `firmware/base/grappstar_base/base_logic.h:791`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: After the capstan reel-in (step 15) the Jetson skips unwrap, or a no-go after a wrap, and sends `retrieve`. The firmware accepts it in REEL and winds the capstan at 5 A / limit_n 50 N until SW_SEATED. Harness (scratch.cpp, REEL, lineOut 1.5): reply ok:true, then SEATED with the dart seated by the capstan. At the WRAPPED park (LOCK below 318, counted as 'neutral') it seats through the 7 wraps on the take-up. BASE_SPEC 10.1 step 17: 'take-up final approach in NEUTRAL only ... Never seat on the capstan'. D16 notes that a seated dart pulled through the wraps can see about 157 N, which the current sense cannot see.

Evidence: base_logic.h:986 'else if (!strcmp(cmd, "retrieve")) {           // final approach until SW_SEATED (take-up in NEUTRAL, capstan in REEL)'
base_logic.h:790-791 'bool neutral = (sec == SEC_NEUTRAL || (sec == SEC_LOCK && drumDeg < c.collarNdeg)) && ...; if (!collarReel() && !neutral) return "NOT_WRAPPED";' (REEL passes, the park passes)

Fix: In the retrieve handler, and in cmdReel for un == U_SEATED, require sec == SEC_NEUTRAL with both dog contacts open (wraps 0). Otherwise refuse, with WRAPS_ON in REEL, N_TO_R or WRAP and NOT_IN_NEUTRAL elsewhere. Update PROTOCOL.md.

> Skeptic (confirmed): I could not refute the gating claim. The code does what the claim says. `retrieve` (base_logic.h:986-988) calls `cmdReel(a, m, "seated")`. At :790-791, `cmdReel` accepts the call when `collarReel()` is true, which is sector REEL with the R dog closed. It also accepts it when `neutral` is true, and `neutral` includes `SEC_LOCK && drumDeg < collarNdeg`, so the WRAPPED park at 314 passes while the cup is locked and the wraps are on. In REEL, `takeupOnly` is false, so the capstan winds with the normal trip current until SW_SEATED rises (:1267). The firmware comment at :986 states this design outri
>
> Skeptic (confirmed): The claim holds up against the code. In base_logic.h, `retrieve` (line 986) calls cmdReel(a, m, "seated"). The cmdReel gate at line 791 is `if (!collarReel() && !neutral) return "NOT_WRAPPED"`. collarReel() (line 152) is true in a confirmed REEL, so `retrieve` and `reel until seated` are both accepted there. With takeupOnly=false the capstan winds at c.tripA until rose(SW_SEATED) (line 1267). The `neutral` test also counts SEC_LOCK below collarNdeg as neutral, which includes the WRAPPED park with the wraps still on. Nothing else refuses this: there is no wraps check and no sec==SEC_NEUTRAL req
>

## 14. Tow trip is a fixed 6.5 A: any max_n above about 75 N can never reach LIMIT/HOLD and always ends in OVERCURRENT (D34 computed trip not implemented)

- Now (863d58d): **fixed** at `firmware/base/grappstar_base/base_logic.h:857`. The computed tow trip from D34 is now in the firmware. towTrip() = min(hardTripA 17 A, max(towTripA 6.5 A, (maxN - offset)/kNperA + freeA + 1)), and line 876 uses it for reel until tension, and line 1025 too when a constant changes. On top of that, D39 (base/spec/DECISIONS.md:47) caps max_n at 70 N, and cmdReel refuses anything higher with BAD_ARG (line 869, limitMaxN = 70), so the max_n 90 and 110 cases from the review are now rejected up front.

- File: `firmware/base/grappstar_base/base_logic.h:801`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: The Jetson asks for a tow with max_n 90 or 110 (params.REEL allows up to 110; cmdReel caps at limitMaxN 110). At the 6.5 A trip the estimate is only 17 x 5.0 - 10.4 = 74.6 N, so the raw-current trip always fires before the filtered estimate reaches max_n. Harness (scratch.cpp towTo): max_n 90 gives FAULT OVERCURRENT 'wiper over 6.5 A' at a true peak of 82.6 N; max_n 110 also OVERCURRENT; max_n 75 gives STALL; max_n 90 with the HX711 gives STALL at 81.4 N. None of them reaches HOLD, and the fault then safes the drum out of REEL.

Evidence: base_logic.h:801 'w.c.tripA = takeupOnly ? c.takeupTripA : until == U_TENSION ? c.towTripA : c.tripA;   // D8'
config.h:63 'float towTripA = 6.5f;'
base_logic.h:794 'if (mxn > c.limitMaxN) mxn = c.limitMaxN;' (110)
DECISIONS D34: 'Tow trip computed in firmware: I_trip = (tow_limit_n + 10.4) / 17.0 + I_free'

Fix: For until tension, set w.c.tripA = fminf(w.c.hardA - 0.5f, fmaxf(c.towTripA, (maxN - c.tensOffsetN) / c.kNperA + c.freeA + 1.0f)). Alternatively refuse (OUT_OF_RANGE) any max_n whose equivalent current reaches the hard trip. Document the rule in PROTOCOL.md.

> Skeptic (confirmed): I checked the code and the claim holds. In base_logic.h:801 a tow (until tension) sets w.c.tripA = c.towTripA, and that value is a fixed 6.5 A (config.h:63). Nothing else in the code computes or raises it. The only other writes to w.c.tripA are other motions and `set trip_a`, which writes c.tripA, not the tow trip. drives.h:55 trips on ampsRaw > tripA, held for 30 ms, once the 50 ms inrush blanking has passed. The tension estimate at base_logic.h:368 is (amps - freeA) * kNperA + tensOffsetN, with freeA 1.5, kNperA 17 and offset -10.4, so 6.5 A gives (6.5-1.5)*17 - 10.4 = 74.6 N. The estimate i
>
> Skeptic (confirmed): I could not refute this. The code says what the claim says. base_logic.h:801 sets the tow trip to the fixed c.towTripA, and config.h:63 sets that to 6.5 A. Nothing else recomputes it: the only other places that write w.c.tripA are aim, cock, jog, decock and the trip_a parameter. DECISIONS D34 asks for a trip worked out in firmware as I_trip = (tow_limit_n + 10.4)/17 + I_free. D8 also says the tow trip should be the free current plus the equivalent of the set limit. Neither is implemented.

cmdReel accepts any max_n above 0 and caps it at limitMaxN, which is 110 (base_logic.h:794, config.h:180)
>

## 15. Power cut during any wiper motion loses phase REST, and decock then silently does nothing: aim is refused for good (no RESET path)

- Now (863d58d): **fixed** at `firmware/base/grappstar_base/base_logic.h:970`. When phase is UNKNOWN with no drawn hint and both SW_HOME and SW_GAP are closed, cal decock now sets phase = PH_REST, clears cockPhase and phaseHint, marks dirty and emits RESET {ok:true, how:"decock"} (lines 970-974), so aim is no longer stuck at PHASE_UNKNOWN. resolveBoot still boots as UNKNOWN. If SW_GAP is not fitted, decock sends RESET ok:false and the phase stays UNKNOWN, but the spec treats SW_GAP as a required switch (BASE_SPEC.md:73,159), so that case falls outside the spec.

- File: `firmware/base/grappstar_base/base_logic.h:886`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: Power is cut during the tow or the capstan reel-in (collar in REEL, launch train decoupled, phase REST, SW_HOME and SW_GAP closed). At boot the phase is UNKNOWN. After unwrap, retrieve and mode LAUNCH, `aim` is refused PHASE_UNKNOWN. BASE_SPEC 10.1 recovery says `decock`, wait RESET, then aim. Here decock replies ok, sends no event and leaves the phase UNKNOWN, so the Jetson waits for RESET until it times out and aim stays refused. PC sim (scratch6.cpp): BOOT phase UNKNOWN; decock ok; RESET seen=0; aim again -> PHASE_UNKNOWN. The same happens after a `stop` in the first moments of `cock` (scratch4.cpp: decock ok, no RESET, aim PHASE_UNKNOWN, mode NEUTRAL LAUNCH_BUSY) and after a power cut during `aim`. The only ways out are `cock` (draws the spring) or `cal cock_rest`.

Evidence: base_logic.h:334-335 `} else { aimKnown = false; phase = PH_UNKNOWN;` runs whenever the record says the wiper moved, even though the switch check at base_logic.h:341 already proves REST (`phase == PH_REST && (!closed(SW_HOME) || ... !closed(SW_GAP))`). base_logic.h:886 `if (phase == PH_UNKNOWN && !(phaseHint > c.homeOpenDeg + c.homeWinDeg && phaseHint < c.releaseDeg + c.releaseWinDeg)) return nullptr;   // carriage home: "cock" finds the angle` returns ok with no event and no state change. BASE_SPEC 10.1 Recovery: "`decock` (same as `cal decock`), wait `RESET`, then `aim` again".

Fix: In cmdCal decock (line 886), when SW_HOME is closed, SW_GAP is closed (or not fitted) and the hint shows no draw: set `phase = PH_REST; cockPhase = 0; phaseHint = -1; dirty = true;` and emit `RESET` {ok:true, how:"decock"} instead of returning silently. In resolveBoot, when the record's phase was REST and the record shows the launch train was not driven (drum sector not LAUNCH, or SW_HOME and SW_GAP still closed), keep PH_REST rather than PH_UNKNOWN.

> Skeptic (confirmed): I read the code and the claim holds up, with one caveat.

(1) Losing REST at boot works as designed. Reel/tow sets MV_WIPER (base_logic.h:802, 1294), so after a power cut wiperTrust is false. resolveBoot then takes the else branch at 334-335 and sets phase UNKNOWN even when SW_HOME and SW_GAP are closed. PROTOCOL.md 430-434 documents this ("if the wiper was moving ... the shot phase UNKNOWN").

(2) The decock no-op is real. At line 886, with SW_HOME closed, phase UNKNOWN and no draw hint (phaseHint -1), it returns nullptr. It sends no event and changes nothing. aim then refuses at line 676 wit
>
> Skeptic (confirmed): The defect is real. I checked the code, not the comments.

1. The boot path is as claimed. Reel/tow sets MV_WIPER (base_logic.h:802, 1294). After a power cut, wiperTrust is false, so resolveBoot runs the else branch at :334-335 and sets phase = PH_UNKNOWN. The REST check at :341 can only downgrade the phase, never set it back to REST. So after the cut the phase is UNKNOWN even though SW_HOME and SW_GAP are closed and the hint shows no draw.

2. aim (cmdAim :676) refuses with PHASE_UNKNOWN.

3. In cmdCal decock, the case SW_HOME closed + PH_UNKNOWN + no draw in the hint returns nullptr at :886.
>

## 16. `set trip_a` while a motion runs overwrites that motion's own trip (take-up 2.5 A, jog/fly-in 3 A, tow 6.5 A)

- Now (863d58d): **fixed** at `firmware/base/grappstar_base/base_logic.h:1024`. cmdSet now copies c.tripA into w.c.tripA only during S_AIM, or during S_REEL when the reel is neither take-up-only nor U_TENSION (the motions that start on c.tripA). A tow gets towTrip() back, clamped to hardTripA (line 1025). Take-up (NEUTRAL reel), S_JOG and the fly-in/centering helpers in S_DRUM/S_WRAP/S_UNWRAP keep their own trip, so the failure no longer happens. The 3-19 A range on trip_a is unchanged, but it now only reaches motions that already run at c.tripA.

- File: `firmware/base/grappstar_base/base_logic.h:932`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: During a retrieve in NEUTRAL (take-up trip 2.5 A, I4), the host sends {set trip_a 5}, even with the unchanged default. The running take-up now trips at 5 A, double the take-up-only protection. Harness (scratch.cpp): w.c.tripA is 2.50 before the set and 5.00 after it. The same applies to a REEL fly-in or centering jog (3 A, raised to trip_a) and to a tow (6.5 A, lowered to 5 A), where the tow then faults OVERCURRENT at about 50 N instead of reaching LIMIT. Up to 19 A is accepted, which is above the 17 A hard trip.

Evidence: base_logic.h:932 'if (!strcmp(k, "trip_a") && state != S_COCK && state != S_FIRE) w.c.tripA = c.tripA;'
base_logic.h:801 (take-up 2.5 / tow 6.5), base_logic.h:531 'void wiper(int dir, int duty, float trip) { w.c.tripA = trip; ...' (fly-in/take-up 3 / 2.5 A)

Fix: Never touch w.c.tripA from cmdSet. Each motion start already sets its own trip (cmdAim, cmdReel, cmdJog, wiper(), cmdCal decock), so delete the line, or apply it only when !w.running() && state == S_IDLE. Optionally refuse `set trip_a` with BUSY while moving().

> Skeptic (confirmed): I could not refute this. The code does what the claim says. In base_logic.h, cmdSet (line 922) applies `if (!strcmp(k, "trip_a") && state != S_COCK && state != S_FIRE) w.c.tripA = c.tripA;` at line 932, and only S_COCK and S_FIRE are excluded. The `set` dispatcher at line 992 has no BUSY or moving() check, so the command is accepted in S_REEL, S_JOG, S_WRAP, S_UNWRAP, S_DRUM, S_HOLD and S_DECOCK.

Each motion sets its own trip only when it starts:
- cmdReel, line 801: take-up 2.5 A, tow 6.5 A, otherwise trip_a.
- cmdJog, line 837: take-up 2.5 A, cock 6.5 A, otherwise jog 3 A.
- wiper(), line 5
>
> Skeptic (confirmed): I could not refute this. The code says what the claim says. In base_logic.h, cmdSet (lines 922-937) has no BUSY or moving() guard for the generic keys. The only special handling at line 932 is `if (!strcmp(k, "trip_a") && state != S_COCK && state != S_FIRE) w.c.tripA = c.tripA;`, so any other state, including S_REEL, S_JOG, S_AIM and S_DECOCK, gets the live wiper trip overwritten.

Each motion sets its own trip only once, when it starts:
- cmdReel (line 801): takeupTripA in NEUTRAL, towTripA for U_TENSION, otherwise c.tripA.
- cmdJog (line 837): takeupTripA, cockTripA or jogTripA.
- wiper() (l
>

## 17. Drum jog skips the SW_DOG_L/R check and can run back from the WRAPPED park into the wrap window, unwrapping with the wiper stopped (R1 #15 still open)

- Now (863d58d): **fixed** at `firmware/base/grappstar_base/base_logic.h:1420`. cmdJog now applies the dog-contact rule (line 923) and refuses inside WRAP (line 921). tickJog checks SW_DOG_L/R on every drum tick and halts at the WRAP edge (lines 1419-1422). A scratch test of the reported scenario (/tmp/claude-0/review-scratch-stale/f8/t.cpp): from the WRAPPED park 314.9 with SW_DOG_R stuck closed, the second -1 jog raises fault 6 (SWITCH_DISAGREE) at 303.2 deg on entering WRAP, with 7.00 wraps still on, and the later jogs are refused with FAULTED.

- File: `firmware/base/grappstar_base/base_logic.h:1308`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: Drum at the WRAPPED park 314 with SW_DOG_R stuck closed (collar jammed in REEL). The operator sends drum jog dir -1 (1500 ms, repeated). cmdJog refuses only sec == SEC_REEL && dd < 0, and tickJog never checks the dog contacts. Harness (scratch.cpp): 6 jogs leave the drum in the WRAP sector at 249.8 deg with 5.48 wraps (1.5 wraps unwound, no take-up), state IDLE, no fault. BASE_SPEC 7: 'SW_DOG_L / SW_DOG_R are checked during every move (the wrap window is never entered with either closed)'. D16 requires the take-up whenever wraps come off.

Evidence: base_logic.h:844 'if (sec == SEC_REEL && dd < 0) return "WRAPS_ON";   // leaving REEL needs the centering jog: use unwrap' (LOCK is not covered)
base_logic.h:1308-1313: tickJog for the drum checks only the end stops, never closed(SW_DOG_L)/closed(SW_DOG_R)
base_logic.h:562 has the dog check, but only inside tickDrumMove.

Fix: In cmdJog for the drum, refuse dd < 0 when sec >= SEC_LOCK, and refuse any jog that would cross a WRAP sector edge (sec == SEC_WRAP, or the edge next to it): answer WRAPS_ON and point to unwrap/wrap. In tickJog for jogMotor 2, apply the line-562 rule each tick: halt and raise F_SWITCH_DISAGREE if (closed(SW_DOG_L) && sec > SEC_LN) || (closed(SW_DOG_R) && sec < SEC_LOCK).

> Skeptic (confirmed): I could not refute this one. I checked the code and reproduced the problem myself.

Code (firmware/base/grappstar_base/base_logic.h):
- In the drum branch of cmdJog, the only direction check is `if (sec == SEC_REEL && dd < 0) return "WRAPS_ON";`. A backward jog is therefore accepted in SEC_LOCK (N_TO_R, where the WRAPPED park 314 sits) and in SEC_WRAP. The launchExitCheck guard only applies at sec <= SEC_LN or when the launcher is known loaded.
- tickJog (around lines 1305-1313) checks only the two groove end stops for jogMotor 2. It never reads SW_DOG_L or SW_DOG_R.
- The collar-contact rule 
>
> Skeptic (confirmed): I couldn't refute this; the defect is real. I checked it by reading the code, not by running anything. The WRAPPED park at 314 deg sits in N_TO_R, which is SEC_LOCK (config.h:24 orders the sectors LAUNCH, LN, NEUTRAL, PREP, WRAP, LOCK, REEL; params.py puts WRAPPED at 314 inside N_TO_R).

For the drum, cmdJog (base_logic.h:840-846) refuses only these cases: unknown sector, hand mode, the launch exit check, and `sec == SEC_REEL && dd < 0` (WRAPS_ON). It never looks at SW_DOG_L or SW_DOG_R, and a backward jog from SEC_LOCK is allowed. tickJog (1297-1318) checks only the two end stops for jogMotor
>

## 18. Wiper IS scaling assumes an averaged IS voltage, but the averaging capacitor is optional and hwWiperIs takes 8 back-to-back point samples: at partial duty the D8 trips depend on PWM phase

- Now (863d58d): **fixed** at `firmware/base/grappstar_base/grappstar_base.ino:84-88`. hwWiperIs now takes 16 reads paced 53.125 us apart. Each read lands 3.125 us later in the 50 us period of the 20 kHz PWM (PWM_HZ 20000 in pins.h:47), so the 16 reads cover the whole period evenly and their mean is the true average with or without the capacitor. That is the alternative fix the review proposed. The capacitor is still optional rather than required: WIRING.md:59 and :109 call it "strongly recommended", and with the sampling fix that is acceptable.

- File: `firmware/base/grappstar_base/grappstar_base.ino:82`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: The board is built as WIRING.md allows, without the optional 1-2.2 uF on GPIO1 (the only filter left is the pin capacitance with 680 R, tau well under 1 us). The BTS7960 IS output is a 20 kHz pulse train (I during the on time, 0 otherwise). hwWiperIs takes 8 analogReadMilliVolts calls back to back, which span about 1 to 3 PWM periods, so each tick's reading is a phase-dependent fraction between 0 and I. WiperDrive then multiplies it by 255/duty. Example: aim at aimSlowDuty 110 (43 %) with a real 7 A jam against a 5 A trip. A tick reads about 16 A when the samples land in the on window and about 0 when they land in the off window. The trip needs 6 consecutive over-trip ticks (tripSamples), so with mostly-off ticks breaking the run the over-current trip is delayed by many 30 ms windows or never fires (about 0.43^6 = 0.6 % per window in the correlated case). The 17 A hard trip has the same problem. The reel tension estimate (17 N/A) gets the same bimodal noise. The PC sim models IS as the exact average (sim.h simIs: `S.amps / S.isAmpsPerVolt * (S.wDuty / 255.0)`), so no test can see this.

Evidence: grappstar_base.ino:82 `float hwWiperIs() { uint32_t mv = 0; for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(PIN_WIPER_IS); return mv / 8000.0f; }`. drives.h:48-50 `// current: the IS pin only reports while the high side conducts, so its average is I x duty. Scale back.` / `if (outDir && outDuty >= c.minScaleDuty) a *= 255.f / outDuty;`. drives.h:18 `tripSamples = 6`. WIRING.md:93 `**Optional:** a 1 uF to 2.2 uF capacitor from GPIO1 to GND smooths the PWM pulses on the IS signal. Only if you have one.`

Fix: Make the IS filter capacitor mandatory: in WIRING.md section 3 and Part B step 3, change it to '1 uF (to 2.2 uF) ceramic from GPIO1 to GND, required' (tau = 680 R x 1 uF = 0.68 ms, more than 10 PWM periods and well inside the 30 ms trip window), and add it to the parts list and the Part C ohm/visual checks. Alternatively, or as well, in hwWiperIs spread the samples evenly over a whole number of PWM periods, for example 20 reads paced 10 us apart over 200 us = 4 periods at 20 kHz.

> Skeptic (confirmed): The defect is real, but it matters less than claimed.

The code matches the claim. grappstar_base.ino:82 takes 8 analogReadMilliVolts calls back to back, with no pacing and no averaging over a known window. drives.h:48-50 scales the reading by 255/outDuty. That scaling is only right if the IS voltage is already averaged over the PWM period. The trip at drives.h:56-57 uses ampsRaw, the unsmoothed value, not the EMA'd amps, and needs 6 over-trip ticks in a row. WIRING.md:93 makes the 1-2.2 uF capacitor optional ("Only if you have one"). Without it, 1 k + 680 R plus pin capacitance gives a sub-mi
>
> Skeptic (confirmed): The defect is real, but its size is overstated.

What the code does:
- grappstar_base.ino:82 takes 8 back-to-back analogReadMilliVolts point samples. Nothing paces or windows them.
- pins.h:47 sets PWM_HZ to 20000 (50 us period). LEDC runs on its own clock, so the reads have no fixed relation to the PWM phase.
- The divider has a source impedance of 1k||680, about 405 R. Without the capacitor, the only filter is pin and S/H capacitance, a time constant in the nanoseconds. Each read therefore catches the IS pulse train at either I or 0, not its average.
- WIRING.md:93 and Part B step 3 really d
>

## 19. require_seated=false (and vbat_wired=false) persist silently in NVS: fire without SW_SEATED keeps working after every reboot

- Now (863d58d): **still open** at `firmware/base/grappstar_base/base_logic.h:112`. tunNow() still packs requireSeated and vbatWired into Tun.requireSeated (bit 0 and bit 2), and restoreTun() (line 122) loads both back at boot. The flags show up only through `get` (lines 1128-1129), not in status or telemetry. CALIBRATION.md:303 now says outright that the setting is saved, and PROTOCOL.md:443 does not mark it "NOT saved".

- File: `firmware/base/grappstar_base/base_logic.h:111`
- Severity: low (claimed medium); 1 of 2 skeptics confirmed
- Owner: fw-base

Scenario: On the bench, `set require_seated false` is sent to dry-run the shot cycle. It is written into the Tun record and restored at every boot: the harness (scratch cal.cpp) reads require_seated 0 after tunNow and restoreTun. In the field, `arm` and `fire` then pass without a dart on the pusher, contrary to the BASE_SPEC 10 refusal 'no fire unless ... SW_SEATED'. That risks a dry fire, or a shot with the dart and tether not seated (D11). Neither status nor telemetry reports the flag, so the Jetson cannot see it. vbat_wired false, which turns off BROWNOUT and NO_POWER, persists the same way through bit 2.

Evidence: base_logic.h:111 't.requireSeated = (c.requireSeated ? 1 : 0) | (c.vbatWired ? 0 : 2);'
base_logic.h:119 'c.requireSeated = t->requireSeated & 1; c.vbatWired = !(t->requireSeated & 2);'
base_logic.h:924 'if (!strcmp(k, "require_seated")) { c.requireSeated = v != 0; return nullptr; }' plus base_logic.h:992 '... err = cmdSet(s, a); ... if (!err) tunDirty = true;'
base_logic.h:729 'if (c.requireSeated && !closed(SW_SEATED)) return ...' is the only seated check.

Fix: Do not persist require_seated: always boot with requireSeated = true, and mark the key 'NOT saved' in PROTOCOL.md like host_timeout_s. If vbat_wired must persist, report both flags in status and in the HELLO/BOOT event so the Jetson can refuse a mission with an interlock off.

> Skeptic (refuted): The mechanics in the claim are accurate. tunNow (base_logic.h:111) packs requireSeated and vbatWired into Tun.requireSeated. restoreTun (base_logic.h:119) unpacks them at boot. Any successful `set` marks tunDirty (base_logic.h:992), and the record is saved once everything is still (base_logic.h:1152). What does not hold is the claim that this is silent, invisible or against the spec.

1. The persistence is documented and intended. firmware/base/PROTOCOL.md:360 says set/get keys are "Saved in flash once everything has stopped, kept over a power cycle and a new upload, except where the table say
>
> Skeptic (confirmed): I checked the code and the mechanics are as claimed. In base_logic.h, tunNow() (line 111) packs requireSeated into bit 0 of Tun.requireSeated and !vbatWired into bit 2, and restoreTun() (line 119) unpacks both at boot. cmdSet (lines 924-925) accepts both keys at any time. Line 992 sets tunDirty after a successful set, and line 1152 calls saveTun(tunNow()) once everything is still. fireCheck (line 729) is the only SW_SEATED gate. No status or telemetry field reports either flag; the only way to read them is `get` (lines 1023-1024). On the Jetson side, nothing outside mock_base reads require_sea
>

## 20. retrieve (until seated) is accepted in REEL and seats the dart on the capstan, against 10.1 step 17

- Now (863d58d): **fixed** at `firmware/base/grappstar_base/base_logic.h:868`. On the newest tree, cmdReel refuses an until-seated reel with WRAPS_ON unless the drum is in SEC_NEUTRAL (`if (un == U_SEATED && sec != SEC_NEUTRAL) return "WRAPS_ON";`). Both `retrieve` (line 1084) and `reel until seated` go through cmdReel, so a retrieve sent in REEL is now refused and the capstan never seats the dart.

- File: `firmware/base/grappstar_base/base_logic.h:791`
- Severity: low (claimed medium); 1 of 2 skeptics confirmed
- Owner: fw-base

Scenario: After step 15 the Jetson (or an operator) sends `retrieve` while still in REEL. The firmware accepts it (PC sim scratch.cpp case C: retrieve -> ok, state REEL, collar REEL) and winds the capstan with the 5 A trip until SW_SEATED. The tension can reach about 17 x (5 - 1.5) - 10 ≈ 49 N before anything stops it. The spec says the final approach is on the take-up in NEUTRAL only ("Never seat on the capstan").

Evidence: base_logic.h:986-988 retrieve -> `cmdReel(a, m, "seated")`. base_logic.h:791 `if (!collarReel() && !neutral) return "NOT_WRAPPED";` lets collarReel through for every `until`. The comment at base_logic.h:986 says "(take-up in NEUTRAL, capstan in REEL)". BASE_SPEC 10.1 step 17: "take-up final approach in NEUTRAL only ... Never seat on the capstan".

Fix: In the retrieve handler (and for `reel until seated`), refuse when not in NEUTRAL: `if (collarReel() || sec >= SEC_WRAP) err = "WRAPS_ON"; else err = cmdReel(...)`, or in cmdReel: `if (un == U_SEATED && !neutral) return "WRAPS_ON";`.

> Skeptic (confirmed): The code path is real. The retrieve handler (base_logic.h:986-988) calls cmdReel(a, m, "seated") without checking the mode. In cmdReel (base_logic.h:790-791), `neutral` is false in REEL, but `collarReel()` is true, so the `NOT_WRAPPED` guard does not fire. No later check rejects U_SEATED outside NEUTRAL; the only one is NO_SENSOR, at line 795. So the command runs with takeupOnly=false, tripA = c.tripA (5 A), and maxN = limitN, which defaults to 50 N (config.h:174-178). The tension check at line 1283-1284 stops it at about 50 N, and the 5 A trip gives 17 x (5 - free) - 10.4. That is about the 4
>
> Skeptic (refuted): The narrow fact is true. base_logic.h:986-988 sends `retrieve` to cmdReel(a, m, "seated"), and line 791 lets it through whenever collarReel() is true, so `retrieve` sent in REEL winds the capstan until SW_SEATED. The claim does not hold up as a medium defect, for four reasons.

(1) The spec and contract allow seating on the capstan. BASE_SPEC line 157-158 says "A no-go landing retrieves the unanchored dart on the take-up (or after a wrap, on the capstan)". The C1 contract (WORKPLAN:91) lists `reel ... until: seated` with no NEUTRAL-only restriction. PROTOCOL.md:84 and :99-100 describe REEL `un
>

## 21. Drum jog backward from the WRAPPED park runs into the WRAP sector and unwraps with the wiper stopped (R1 #15 not fixed)

- Now (863d58d): **fixed** at `firmware/base/grappstar_base/base_logic.h:1422`. This is fixed. tickJog now stops any drum jog on the tick where sec reads SEC_WRAP. updateSector runs earlier in the same tick, so a backward jog from the WRAPPED park halts at the 302 edge with a "wrap window: use wrap or unwrap" abort. cmdJog (line 921) also refuses any drum jog that starts in SEC_WRAP or while wrapsDoubt is set, returning WRAPS_ON. I checked this by reading the code; I did not run the sim. The drum may still coast a little past the edge before it stops.

- File: `firmware/base/grappstar_base/base_logic.h:844`
- Severity: low (claimed low); unverified (low)
- Owner: fw-base

Scenario: At the WRAPPED park 314 the operator sends `jog drum dir -1` repeatedly (each up to 1500 ms). Nothing stops at the 302 edge: the drum enters WRAP and the cup unwinds with no take-up. PC sim (scratch5.cpp): six jogs, all ok, take the drum to 249.8 deg, wraps 7 -> 5.48, 0.196 m of slack freed, state IDLE, no fault (D16: unwrap needs the take-up).

Evidence: base_logic.h:843-845 `if ((sec <= SEC_LN || loadedKnown()) && ...) ...; if (sec == SEC_REEL && dd < 0) return "WRAPS_ON"; d.want(dd * c.drumSign);` refuses backward only in REEL. tickJog (1308-1312) only stops at the groove ends.

Fix: In cmdJog drum: `if (sec >= SEC_WRAP && dd < 0) return "WRAPS_ON";` (use unwrap). In tickJog: stop a drum jog when sec changes into SEC_WRAP from SEC_LOCK or SEC_PREP.


## 22. New settable key tension_offset_n is not saved in NVS: a calibrated tension offset is silently lost at every reboot

- Now (863d58d): **fixed** at `firmware/base/grappstar_base/persist.h:57; firmware/base/grappstar_base/base_logic.h:116,125`. Tun ver 2 now stores tensOffsetN (it is saved in tunNow and restored in restoreTun), and PROTOCOL.md:429 lists the key. test_base.cpp:772-775 checks that tension_offset_n -3 survives a reboot, and all 384 checks pass.

- File: `firmware/base/grappstar_base/base_logic.h:906`
- Severity: low (claimed low); unverified (low)
- Owner: fw-base

Scenario: On the bench the operator fits the D32 tension law and sends {"cmd":"set","key":"tension_offset_n","val":-6}. The reply is ok and tunDirty triggers a Tun save, but Tun has no field for tensOffsetN. After the next power cycle the offset is back to -10.4. Every estimated tow tension is then 4.4 N low and the LIMIT stop comes about 4.4 N late. CALIBRATION.md:6-7 and PROTOCOL.md:360 promise that every set value is kept over a power cycle, and the PROTOCOL key table lists no exception for this key (it does not list the key at all).

Evidence: base_logic.h:906 `out[n++] = {"tension_offset_n", &c.tensOffsetN, -50, 50};` (added since 0bfd59a). persist.h:55-56 Tun fields `ampsPerVolt, offsetA, freeA, kNperA, hxNPerCount, hxOff, limitN, drumDps, aimOffset, aimTol, tripA, cockTripA, vbatLow, aimHomeDeg`, no tension offset. base_logic.h:110-121 tunNow/restoreTun do not touch tensOffsetN.

Fix: Add `float tensOffsetN = 0;` to Tun, bump Tun ver to 2 and the static_assert to 72 (validTun checks `ver == 2`), store it in tunNow (`t.tensOffsetN = c.tensOffsetN;`) and restore it in restoreTun (`c.tensOffsetN = t->tensOffsetN;`). Add the key to the PROTOCOL.md set/get table.


## 23. decock ends on SW_HOME alone and reports RESET ok without SW_GAP closed (R1 #10 still open for decock)

- Now (863d58d): **fixed** at `firmware/base/grappstar_base/base_logic.h:1442`. tickDecock now sets PH_REST and emits RESET ok only when SW_HOME has stayed closed for decockExtraDeg AND (!fitted(SW_GAP) || closed(SW_GAP)). If SW_GAP stays open it keeps running until the travel or time limit, then raises F_BAD_STATE "decock: SW_GAP never closed" (line 1448). The decock-by-travel fallback path (line 1432) still does not check SW_GAP, but that path exists for travel-only decock and was not part of this finding.

- File: `firmware/base/grappstar_base/base_logic.h:1331`
- Severity: low (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: Partial cock, stop, then `decock`. The big gear stops short of its gap (in the harness SW_GAP is held open). tickDecock declares phase REST and emits RESET ok:true once SW_HOME has stayed closed for 15 deg. The next C1 steps then fail with nothing to explain why: harness (scratch.cpp) gives RESET ok:true, phase REST, gap closed 0, then `mode NEUTRAL` gives LAUNCH_BUSY, and fire from REST would give NOT_COCKED. BASE_SPEC 10: 'De-cock: from a partial cock run backward until SW_HOME and SW_GAP close'.

Evidence: base_logic.h:1329-1335 'if (closed(SW_HOME)) { if (!homeSeen) { homeSeen = true; homeFrom = enc.deg(); } } ... if (homeSeen && (enc.deg() - homeFrom) * c.aimDir >= c.decockExtraDeg) { w.halt(now, io.wiper); phase = PH_REST; ... evBegin(o, "RESET"); o.boolean("ok", true); ...' (no SW_GAP test)

Fix: At the end of the run-on, require !fitted(SW_GAP) || closed(SW_GAP) before setting PH_REST. If SW_GAP is still open, keep running back up to a bounded extra travel (for example 40 deg). After that, raise F_BAD_STATE "decock ended with the big gear out of its gap" (phase UNKNOWN) instead of RESET ok.

> Skeptic (confirmed): The claim holds as a spec non-conformance and a gap in fault detection. It does not make anything unsafe.

What the code does: tickDecock (base_logic.h:1319-1338) ends the normal path once SW_HOME has stayed closed for decockExtraDeg (15 deg, config.h:115). It then sets PH_REST and sends RESET ok:true, and it never looks at SW_GAP. The decockByTravel path (1321-1325) has no SW_GAP test either. BASE_SPEC line 159 says "run backward until SW_HOME and SW_GAP close". The fire/reset path does check, at line 1250 ("reset ended with the big gear out of its gap" -> F_BAD_STATE), and the boot check at 
>
> Skeptic (confirmed): The cited code is accurate. tickDecock (base_logic.h:1328-1336) sets phase = PH_REST and emits RESET ok:true once SW_HOME has stayed closed for decockExtraDeg (15 deg). It never reads SW_GAP. The decock-by-travel branch (1322-1324) does not read it either. That contradicts the spec in two places. BASE_SPEC section 10 (line 159) says to run backward "until SW_HOME and SW_GAP close". PROTOCOL.md:433 says REST needs both switches closed. The fire reset path does check (line 1250: "reset ended with the big gear out of its gap"), so the R1 fix reached reset but not decock. The downstream behaviour 
>

## 24. decock ends as REST without SW_GAP; spec requires SW_HOME and SW_GAP (R1 #10 partly open)

- Now (863d58d): **fixed** at `firmware/base/grappstar_base/base_logic.h:1442`. The defect is fixed on the newest tree. In tickDecock, the normal path now ends as REST only when SW_HOME stays closed for the whole run-on and SW_GAP is closed (or not fitted). Otherwise it keeps running until the travel or time limit, then raises "decock: SW_GAP never closed" (line 1447). cal cock_rest now refuses with SWITCH_DISAGREE when SW_GAP is fitted and open, unless force is set (line 959). One gap remains, and only on the forced path: decock-by-travel still sets REST without checking SW_GAP. That path runs only when the operator passes force, which is the exception the fix allowed.

- File: `firmware/base/grappstar_base/base_logic.h:1331`
- Severity: low (claimed low); unverified (low)
- Owner: fw-base

Scenario: Decock from a partial cock where the big gear stops just past its gap (SW_GAP open) when SW_HOME closes. tickDecock still sends RESET ok and sets phase REST. Then `arm`/`fire` is refused NOT_COCKED (gapOk needs SW_GAP for REST), `mode NEUTRAL`/`wrap` is refused LAUNCH_BUSY (launchExitCheck), and on the next boot the REST record is downgraded to UNKNOWN (line 341). The base reports a clean reset while it is in a state that blocks every exit. cal cock_rest (line 875) also declares REST with SW_GAP open.

Evidence: base_logic.h:1331-1335 `if (homeSeen && (enc.deg() - homeFrom) * c.aimDir >= c.decockExtraDeg) { ... phase = PH_REST; ... "RESET" ...}` never checks SW_GAP. BASE_SPEC 10: "De-cock: from a partial cock run backward until SW_HOME and SW_GAP close." base_logic.h:643 `if (fitted(SW_GAP) && !closed(SW_GAP)) return "LAUNCH_BUSY";`.

Fix: In tickDecock, end only when `homeSeen && run-on done && (!fitted(SW_GAP) || closed(SW_GAP))`, and keep running back (bounded by the existing travel and time limit at line 1337) until SW_GAP closes. In cal cock_rest, refuse with SWITCH_DISAGREE when SW_GAP is fitted and open, unless force is set.


## 25. Current below duty 60 is not scaled: the 3 A jog trip is blind during the 20 % fly-in and centering jog (R1 #17 still open)

- Now (863d58d): **fixed** at `firmware/base/grappstar_base/config.h:167`. flyDuty is now 64, at or above minScaleDuty 60 (drives.h:17,50), so fly-in current gets scaled and the 3 A jogTripA sees the real current. The stage-5 centering jog runs at jogDuty 150 (base_logic.h:586). test_base.cpp:856 guards flyDuty >= minScaleDuty. Readings are still unscaled for the first ~70 ms of the ramp while outDuty is below 60, which is the intended design.

- File: `firmware/base/grappstar_base/drives.h:50`
- Severity: low (claimed low); unverified (low)
- Owner: fw-base

Scenario: REEL fly-in, or the stage-5 REEL centering jog, at flyDuty 51. The dogs land tip to tip and the motor draws 10 A. The IS average gives 10 x 51/255 = 2 A, and that is not scaled back because 51 < minScaleDuty 60. The firmware reads 2 A, below the 3 A jogTripA, so the D8 trip never fires. Only the 200 ms stall check (or a true 85 A for the 17 A hard trip) stops it.

Evidence: drives.h:17 'int minScaleDuty = 60;'
drives.h:50 'if (outDir && outDuty >= c.minScaleDuty) a *= 255.f / outDuty;'
config.h:149 'int flyDuty = 51;'
base_logic.h:522, 538, 612: wiper(c.reelDir, c.flyDuty, c.jogTripA)

Fix: Raise flyDuty to at least 64 (above minScaleDuty), or scale every sample with outDuty > 0 (a *= 255.f / fmaxf(outDuty, 25.f)). As a fallback, compare the unscaled reading with tripA x outDuty / 255 when outDuty < minScaleDuty.


## 26. (round 1 #17, still open) Wiper current is not scaled below duty 60: the 3 A jog/fly-in trip cannot be reached at flyDuty 51

- Now (863d58d): **partly fixed** at `firmware/base/grappstar_base/config.h:167 (flyDuty = 64); firmware/base/grappstar_base/drives.h:50 (scaling still gated on outDuty >= minScaleDuty)`. The main case is fixed. flyDuty is now 64, which is at least minScaleDuty 60, so fly-in, the REEL centering jog and the LAUNCH wiggle get scaled current and can reach the 3 A trip. A test at test_base.cpp:856 checks this. What remains: drives.h:50 still skips scaling below duty 60, so during the first ~70 ms of every start's ramp (0 to 60 at 255 per 300 ms) the reading is still I x duty/255. In that window the 'never ignored' hardA 17 A trip only fires on a far larger real current.

- File: `firmware/base/grappstar_base/drives.h:50`
- Severity: low (claimed low); unverified (low)
- Owner: fw-base

Scenario: Fly-in into REEL, the REEL centering jog (mvStage 5) and the LAUNCH wiggle run the wiper at flyDuty 51 with jogTripA 3 A. Below minScaleDuty 60 the averaged IS reading (I x 51/255 = 0.2 I) is used unscaled, so the 3 A trip needs a real 15 A. At 20 % duty the stall current is about 0.2 x 18 A = 3.6 A, so a real stall above the 3 A jog limit never trips on current and is caught only by the 200 ms stall check. The same blind spot covers every start's ramp below duty 60 (about 70 ms), including the 'never ignored' hard trip.

Evidence: drives.h:17 `int minScaleDuty = 60;`. drives.h:50 `if (outDir && outDuty >= c.minScaleDuty) a *= 255.f / outDuty;`. config.h:149 `int flyDuty = 51;`. config.h:65 `float jogTripA = 3.0f; // jogs, centering jog, fly-in`. base_logic.h:522, 538 and 612 call `wiper(..., c.flyDuty, c.jogTripA)`.

Fix: Always scale while driving: `if (outDir && outDuty > 0) a *= 255.f / fmaxf(outDuty, 20.f);` (clamp only against tiny duties), or raise flyDuty to at least minScaleDuty (60). This depends on the averaging capacitor from the previous finding.

