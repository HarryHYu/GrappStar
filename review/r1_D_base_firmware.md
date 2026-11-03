# Round 1 D: base firmware (fw-base)

Reviewed at base-v1 commit 0bfd59a (tree at 12:20). Method: independent finder subagents, one per lens; every finding of
medium severity or above was checked by two skeptic subagents told to refute it; kept only if fewer than
two refuted it. Low findings are listed unverified. Severity is the finder's, lowered when both skeptics
agreed it is real but rated it lower.

Confirmed: 17. Refuted and dropped: 1.

Re-checked 14:10 Sydney against base-v1 863d58d, one checker subagent per finding: see the Now column.

| # | Sev | File:line | Owner | Finding | Now (863d58d) |
|---|---|---|---|---|---|
| 1 | high | `firmware/base/PROTOCOL.md:54` | fw-base | PROTOCOL.md (contract C1) describes the old drum order: a Jetson built from it cannot complete a mission | fixed |
| 2 | high | `firmware/base/grappstar_base/base_logic.h:327` | fw-base | Default tow tension law contradicts D32, and the fixed 5 A reel trip ends every default tow in an OVERCURRENT fault instead of LIMIT/HOLD | fixed |
| 3 | high | `firmware/base/grappstar_base/base_logic.h:479` | fw-base | SW_DOG_L/SW_DOG_R are not checked during drum moves: the wrap window is entered, and the unwrap take-up runs, with the collar still engaged | fixed |
| 4 | high | `firmware/base/grappstar_base/base_logic.h:695` | fw-base | No WRAPPED park (309): wrap runs straight into the REEL fly-in, and tidy only works in NEUTRAL before the wrap (BASE_SPEC 10 / D22 order not implemented) | fixed |
| 5 | high | `firmware/base/grappstar_base/pins.h:24` | fw-base | pins.h contradicts the shared contract params.SWITCHES on nearly every GPIO: GPIO21 is a contact input in the spec but a driven output in firmware, DRV8833 and HX711 pins are swapped, and SW_GAP / SW_N20 / SW_ARM are missing | fixed |
| 6 | medium | `firmware/base/grappstar_base/base_logic.h:244` | fw-base | A fault in REEL or N_TO_R holds the drum in place instead of going to the WRAPPED park (309) as D31 / BASE_SPEC 7 require | moot |
| 7 | medium | `firmware/base/grappstar_base/base_logic.h:244` | fw-base | A fault or estop in REEL/N_TO_R holds the drum instead of going to WRAPPED 309 (D31 fault rule) | moot |
| 8 | medium | `firmware/base/grappstar_base/base_logic.h:619` | fw-base | aim never sets its own current trip: it runs with whatever trip the last motion left (2.5 A take-up, 3 A jog, or 6.5 A cocking) instead of D8's 5 A | fixed |
| 9 | medium | `firmware/base/grappstar_base/base_logic.h:1216` | fw-base | Drum jog end-stop guard uses an untrusted mid-sector estimate and wrong end stops: after boot it drives past -4 / 352 | fixed |
| 10 | medium | `firmware/base/grappstar_base/config.h:14` | fw-base | SW_GAP does not exist in the firmware: fire and rest are accepted, and decock ends, without the big gear in its gap | fixed |
| 11 | medium | `firmware/base/grappstar_base/config.h:37` | fw-base | Contact bit map in config.h does not match the WIRING.md / BENCH_TEST.md pin table: from GPIO39 up, every contact is read as a different switch | fixed |
| 12 | low | `firmware/base/grappstar_base/base_logic.h:328` | fw-base | Tow limit is not averaged over 200 ms: about 25 ms EMA plus a single-tick trigger | fixed |
| 13 | low | `firmware/base/grappstar_base/base_logic.h:662` | fw-base | SW_GAP is not implemented: fire, arm and decock never check the big gear is in its toothless gap | fixed |
| 14 | low | `firmware/base/grappstar_base/base_logic.h:767` | fw-base | Wiper backward jog is allowed in NEUTRAL/WRAP_PREP/WRAP (unlimited travel), but the spec allows backward outside LAUNCH only for the unload jog | fixed |
| 15 | low | `firmware/base/grappstar_base/base_logic.h:775` | fw-base | Drum jog backward through the WRAP sector unwraps with the wiper stopped, bypassing the unwrap take-up interlock (D16) | partly fixed |
| 16 | low | `firmware/base/grappstar_base/config.h:104` | fw-base | wrapsFull = 4, but the cup lays 7 wraps: telemetry and the WRAPPED event report the wrong count | fixed |
| 17 | low | `firmware/base/grappstar_base/drives.h:96` | fw-base | Wiper current is not scaled below duty 60, so the 3 A jog trip and the 17 A hard trip are blind at flyDuty 51 (fly-in, REEL centering jog, LAUNCH wiggle) | fixed |

## 1. PROTOCOL.md (contract C1) describes the old drum order: a Jetson built from it cannot complete a mission

- Now (863d58d): **fixed** at `firmware/base/PROTOCOL.md:84`. PROTOCOL.md has been rewritten to follow the D35 sequence. `mode REEL` now runs only from N_TO_R/WRAPPED, and `wrap` goes NEUTRAL to the WRAPPED park 314 with 7 SW_ARM pulses. `unwrap` ends at NEUTRAL 41, `tidy` and `until:"slip"` are documented, and NOT_IN_REEL along with the old 40/80/338 stop points is gone; the parks 0/41/56/314/347 match BASE_SPEC.md:126 and config.h:142.

- File: `firmware/base/PROTOCOL.md:54`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: The Jetson follows PROTOCOL.md: after landing it sends mode REEL, then wrap. The firmware answers mode REEL from NEUTRAL with NOT_WRAPPED (base_logic.h:591), so the mission stops at the first retrieval step. PROTOCOL also says unwrap returns 'to the REEL stop point', which is wrong: the code ends at NEUTRAL. It says there are 4 wraps and gives stop points 'LAUNCH 0, NEUTRAL 40, REEL 80, WRAPPED 338', but the code uses 1.5/41/347.5 and the spec uses 0/41/309/347. It lists the error NOT_IN_REEL, which the code never sends (it sends NOT_IN_NEUTRAL). It does not document `until:"slip"` (tidy).

Evidence: PROTOCOL.md:54 '`wrap` | | drum on from REEL to the WRAPPED stop point: the segment gear lays exactly 4 wraps ... | `NOT_IN_REEL`'. PROTOCOL.md:55 'unwrap ... drum back to the REEL stop point'. PROTOCOL.md:76 'stop points: LAUNCH 0, NEUTRAL 40, REEL 80, WRAPPED 338'. PROTOCOL.md:56 lists until only as 'stop | seated | tension'. Code: base_logic.h:591 `if (sec < SEC_LOCK) return "NOT_WRAPPED";` and base_logic.h:692 `if (sec < SEC_NEUTRAL) return "NOT_IN_NEUTRAL";`.

Fix: Rewrite PROTOCOL.md lines 49-56, 75-77 and 83 to match the BASE_SPEC 10 / D16 / D22 sequence once the firmware implements it. mode REEL only from WRAPPED. wrap: NEUTRAL -> WRAPPED (309), 7 wraps, error NOT_IN_NEUTRAL. reel until slip = tidy at WRAPPED. unwrap: REEL -> WRAPPED -> NEUTRAL with the take-up. Stop points 0/41/56/309/347.

> Skeptic (confirmed): I checked the claim against the code and it holds. PROTOCOL.md describes the old drum order and does not match the firmware. Below, "the code" means base_logic.h.

What PROTOCOL.md says (lines 49 and 54-56):
- `mode REEL` moves the drum to REEL.
- `wrap` runs from REEL to WRAPPED, lays 4 wraps, and is refused with NOT_IN_REEL.
- `unwrap` returns the drum to the REEL stop point.
- `reel` accepts `until` = stop | seated | tension only.

Line 76 gives the stop points as LAUNCH 0 / NEUTRAL 40 / REEL 80 / WRAPPED 338. Line 83 says wraps is 4 at WRAPPED.

What the code does:
- **cmdMode (line 591):*
>
> Skeptic (confirmed): I checked the code and the claim holds. firmware/base/PROTOCOL.md, the file C1 names as the final contract (WORKPLAN C1: "fw-base owns the final firmware/base/PROTOCOL.md; jetson implements against it"), still describes the old drum order. The firmware in firmware/base/grappstar_base/base_logic.h follows the D16/D22 order instead.

Mismatches I confirmed in the code:
- **mode REEL:** cmdMode (lines 589-591) only allows REEL when already in REEL or from LOCK. From NEUTRAL it returns `if (sec < SEC_LOCK) return "NOT_WRAPPED";`. PROTOCOL.md:51 lists no such refusal and implies REEL can be reached
>

## 2. Default tow tension law contradicts D32, and the fixed 5 A reel trip ends every default tow in an OVERCURRENT fault instead of LIMIT/HOLD

- Now (863d58d): **fixed** at `firmware/base/grappstar_base/base_logic.h:857`. The defaults now match the review's fix. kNperA is 17 (config.h:193), and there is a settable tension_offset_n (tensOffsetN = -10.4) that is saved in Tun and used in tc at line 385. On a tension reel the trip is now set by towTrip(), which takes max(towTripA 6.5 A, (maxN - offset)/kNperA + freeA + 1 A) and caps it at hardTripA 17 A (lines 857 and 876). It is recomputed whenever a constant changes (line 1025), so the LIMIT/HOLD acts before the over-current fault. With the defaults, a 50 N tow needs about 5.05 A against a 6.5 A trip. One small part is still open: some sim tow tests still override kNperA and freeA (test_base.cpp:420, 931, 977). New tests do cover the default constants (test_base.cpp:777, 787) and the 70 N trip scaling (test_base.cpp:816).

- File: `firmware/base/grappstar_base/base_logic.h:327`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: Fresh build with the default constants (k_n_per_a 8, no offset). Sequence: wrap, then reel until tension with an anchored dart (default limit_n 50). The current estimate only reaches 50 N at I - I_free = 6.25 A, which is I = 7.75 A. The reel trip w.c.tripA = 5 A fires first. Proven with the PC sim (scratch /tmp/claude-0/review-scratch-D/t1.cpp, case C): it ended in state FAULT with code OVERCURRENT "wiper over 5 A" and a true line tension of 60.7 N, with no LIMIT event. The drum then holds in REEL and the tow needs a 'clear'. Calibrating k_n_per_a does not fully fix it. Any limit_n above about 53 N (the 5 A equivalent by D32: 17*(5-1.5)-6) can never be reached because the 5 A trip always fires first, yet limit_n accepts up to 150 (limitMaxN). The repo's own tow tests pass only because they overwrite kNperA and freeA with sim-matched values (test_base.cpp:410, 705).

Evidence: base_logic.h:327 `float tc = winding ? (w.amps - c.freeA) * c.kNperA : 0;` (no offset term); config.h:129 `float kNperA = 8.0f;`; base_logic.h:733 `w.c.tripA = takeupOnly ? c.takeupTripA : c.tripA;` (fixed 5 A whatever limit_n is). D32: "line N = 17 x (I - I_free) - 6 ... the firmware's 8 N/A constant would let the tow reach about twice the limit". D8: "tow trip at measured free current plus the 50 N equivalent".

Fix: Set the default kNperA to 17 and add an offset key tension_off_n (default -6, settable, kept in Tun). Compute tc = kNperA*(amps - freeA) + offset, clamped at 0. In cmdReel (non take-up) set w.c.tripA = fmaxf(c.tripA, c.freeA + (maxN - offset)/c.kNperA + margin) with margin about 0.5 A, so the tension limit (LIMIT/HOLD) always acts before the over-current fault, as D8 requires. Remove the kNperA/freeA overrides from the tow tests so the defaults are tested.

> Skeptic (confirmed): I could not refute this. The defect is real.

What the code does:
- base_logic.h:327 estimates tension as tc = (w.amps - c.freeA) * c.kNperA. There is no offset term.
- The defaults are kNperA = 8 (config.h:129), freeA = 1.5 and limitN = 50.
- cmdReel (base_logic.h:733) always sets w.c.tripA = c.tripA = 5 A for a capstan reel or tow. It never looks at maxN, freeA or the tension law.
- At 5 A the estimate tops out at 8 * 3.5 = 28 N. Reaching the default 50 N limit would need 7.75 A.
- The over-current check in drives.h:55 uses the instantaneous ampsRaw over 6 ticks. The estimate uses filtered a
>
> Skeptic (confirmed): The claim holds up. I checked it against the code and also re-ran the claimant's sim.

1. The tension estimate ignores D32. base_logic.h:327 computes `tc = (w.amps - c.freeA) * c.kNperA`, with no offset term. The default kNperA is 8 (config.h:129) and freeA is 1.5 (config.h:47-52). D32 (DECISIONS.md:40, commit 0bfd59a, added after the firmware commit a6e2394) says line N = 17 x (I - I_free) - 6. It also names the 8 N/A constant as wrong. The firmware never adopted D32. CALIBRATION.md:166 still uses 8 as its example.

2. The reel trip always fires first. cmdReel sets `w.c.tripA = c.tripA` (5 A,
>

## 3. SW_DOG_L/SW_DOG_R are not checked during drum moves: the wrap window is entered, and the unwrap take-up runs, with the collar still engaged

- Now (863d58d): **fixed** at `firmware/base/grappstar_base/base_logic.h:599-602`. tickDrumMove now checks the dogs every stage-0/1 tick, before mvWiperRule runs. It fails with SWITCH_DISAGREE when DOG_L is closed and sec > SEC_LN, or when DOG_R is closed and sec < SEC_LOCK. Case 1: the unwrap fails on its first WRAP-sector tick, before the take-up can start. Case 2: a wrap from NEUTRAL with DOG_L closed fails on its first move tick. What remains is small: cmdWrap still accepts the command and fails one tick later instead of refusing it, and in case 1 the drum may coast slightly into the WRAP code before it stops, but the wiper is never driven.

- File: `firmware/base/grappstar_base/base_logic.h:479`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: Case 1: unwrap from REEL while the collar has not come out of the REEL dogs, so SW_DOG_R is still closed (centering jog timed out, or a stuck fork). The drum runs back through N_TO_R and into WRAP. mvWiperRule then drives the wiper forward at 30 percent with the capstan still coupled. That is the case D16 forbids: the capstan winds and pulls the seated dart through the remaining wraps (up to about 157 N, which the current sense cannot see). The fault only comes on arrival at NEUTRAL. In t1.cpp the wiper turned 811 deg forward inside WRAP with SW_DOG_R closed, then 'SWITCH_DISAGREE collar still engaged at NEUTRAL'. Case 2: wrap from NEUTRAL with SW_DOG_L closed is accepted. The drum enters the wrap window and, from 318, the fly-in turns the wiper FORWARD (the cock direction) with the collar in LAUNCH. In t1.cpp: 'wrap with SW_DOG_L closed: ok', 'entered WRAP window=1'.

Evidence: base_logic.h:479-482 `bool take = mvTakeup && (sec == SEC_WRAP || sec == SEC_PREP); bool fly = targetSec(mvTarget) == SEC_REEL && (sec == SEC_REEL || (sec == SEC_LOCK && drumDeg >= c.nrEngageDeg - 18)); if (take) wiper(c.reelDir, ...); else if (fly) wiper(c.reelDir, ...)`. Neither condition looks at the dogs. Stage 0 (lines 503-519) never reads SW_DOG_L/R. The only checks are at arrival (lines 531-535) and the both-closed check in watch (line 402). cmdWrap (lines 687-697) and cmdUnwrap (lines 700-711) do not check the dogs either. BASE_SPEC 7: 'SW_DOG_L / SW_DOG_R are checked during every move (the wrap window is never entered with either closed)'.

Fix: In tickDrumMove stages 0 and 1, run a check every tick. If (sec==SEC_PREP || sec==SEC_WRAP || (mvDir<0 && sec==SEC_LOCK && drumDeg < c.nrEngageDeg - 18) || (mvDir>0 && sec==SEC_NEUTRAL)) && (closed(SW_DOG_L) || closed(SW_DOG_R)), then call failDrum(safing, F_SWITCH_DISAGREE, "collar engaged at the wrap window") and return -1. In mvWiperRule, add `&& !closed(SW_DOG_L) && !closed(SW_DOG_R)` to `take`, and `&& !closed(SW_DOG_L)` to `fly`. In cmdWrap (and in cmdUnwrap when starting from PREP or NEUTRAL), refuse with SWITCH_DISAGREE if either dog is closed.

> Skeptic (confirmed): I could not refute this; the code does what the claim says. In base_logic.h, mvWiperRule (lines 478-483) picks take-up (`mvTakeup && sec in WRAP/PREP`) or fly-in (`target REEL && (REEL or LOCK past nrEngageDeg-18)`) without reading SW_DOG_L or SW_DOG_R. tickDrumMove stage 0 (503-519) and stage 1 (520-528) never read the dogs either. Inside a move the dogs are checked only at arrival (stage 4, lines 531-535), plus the 200 ms both-closed check in watch() (line 402), which misses a single closed contact. cmdWrap (687-697) and cmdUnwrap (700-711) check no dog either, unlike the jog path at lines 7
>
> Skeptic (confirmed): I could not refute this. The code matches the claim:
- `mvWiperRule` (base_logic.h:479-484) sets `take` from `mvTakeup` plus the WRAP/PREP sector only. It sets `fly` from the target sector plus REEL, or LOCK past `nrEngageDeg-18`. Neither reads SW_DOG_L or SW_DOG_R.
- In `tickDrumMove`, stage 0 (503-519) and stage 1 (520-528) never read the dogs. The only dog checks during a move are in stage 4, after arrival (531-535). That is where 'collar still engaged at NEUTRAL' is raised, after the drum has already crossed WRAP.
- `watch()` checks only that both dogs are closed (line 402) and that the co
>

## 4. No WRAPPED park (309): wrap runs straight into the REEL fly-in, and tidy only works in NEUTRAL before the wrap (BASE_SPEC 10 / D22 order not implemented)

- Now (863d58d): **fixed** at `firmware/base/grappstar_base/base_logic.h:822`. cmdWrap now runs startDrumMove(DP_WRAPPED, false) to stop at the WRAPPED park (314) with the collar still in NEUTRAL; the REEL fly-in only happens on a later "mode REEL". A dedicated "tidy" command (line 1076) accepts the WRAPPED park when drumDeg < collarNdeg and both dogs are open. The full-mission test (test_base.cpp:957-987: wrap, parked at WRAPPED, tidy, then REEL) passes, along with all 384 checks; "mode WRAPPED" is still BAD_ARG, but the spec does not require it.

- File: `firmware/base/grappstar_base/base_logic.h:695`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: Normal mission after LANDED: mode NEUTRAL, then wrap. The drum does not stop at 309. It goes on to REEL and the wiper creeps forward (fly-in), so the capstan is coupled before the dart has anchored. Next the Jetson sends tidy as {cmd:reel, until:slip}. The reply is BAD_ARG because a tidy is only accepted in NEUTRAL. Any request for WRAPPED gets BAD_ARG. The sequence WRAPPED park -> wait ANCHORED -> TIDY (FIRM take-up, capstan free) -> REEL fly-in cannot be run. The scratch test /tmp/claude-0/review-scratch-D/t1.cpp prints 'after wrap: state=IDLE sec=6 drum=347.0 collarReel=1', 'tidy (reel until slip): BAD_ARG' and 'mode WRAPPED: BAD_ARG'. The repo's own test (test_base.cpp:370-378) runs the tidy BEFORE the wrap, which is the reverse of D22.

Evidence: base_logic.h:695 `if (!startDrumMove(DP_REEL, false)) return "POSITION_UNKNOWN";` (cmdWrap's target is REEL). base_logic.h:140 `static int targetSec(int t) { return t == DP_LAUNCH ? SEC_LAUNCH : t == DP_NEUTRAL ? SEC_NEUTRAL : SEC_REEL; }` (there is no WRAPPED target). config.h:102 `float targetDeg[3] = {1.5f, 41, 347.5f};` (no 309). base_logic.h:722-725 `bool neutral = sec == SEC_NEUTRAL ...; if (!neutral && un == U_SLIP) return "BAD_ARG";`. BASE_SPEC 10: 'WRAP at once ... (drum 62..302, wiper off) -> WRAPPED park (309) -> wait ANCHORED -> TIDY ... -> REEL fly-in'. params.DRUM.targets WRAPPED=309.0.

Fix: Add a DP_WRAPPED target: targetDeg[DP_WRAPPED]=309, and targetSec(DP_WRAPPED)=SEC_LOCK. In cmdWrap call startDrumMove(DP_WRAPPED) and keep the wiper off (no fly-in on that leg). In stage 4, a move to SEC_LOCK must check that both dogs are open, as for NEUTRAL. Accept cmdReel until=slip/stop as take-up only when sec==SEC_LOCK with both dogs open (takeupOnly, takeupTripA, takeupMaxDuty). Keep the fly-in for 'mode REEL' from LOCK only. In cmdUnwrap from REEL, run unload jog -> DP_WRAPPED (collar to NEUTRAL, dogs verified open) -> DP_NEUTRAL with the take-up. Update test_base.cpp to the D22 order.

> Skeptic (confirmed): I could not refute this. The claim holds against the code and the binding spec.

Spec: BASE_SPEC section 10 and D22 give the order WRAP while the line is slack -> WRAPPED park at 309 (cup locked, collar NEUTRAL, clutch FIRM) -> wait ANCHORED -> TIDY (FIRM take-up, capstan turning freely) -> REEL fly-in. params.DRUM.targets has WRAPPED=309.0, and the scripted step list in params.py:274-276 has 'wrapped' (drum_deg 309) and then 'tidy'.

Code (base_logic.h):
- cmdWrap (line 695) calls startDrumMove(DP_REEL, false). There is no stop at 309.
- targetSec (line 140) maps every target other than LAUNC
>
> Skeptic (confirmed): I checked the code and the spec, and the claim holds. In cmdWrap (base_logic.h:695) the drum's target is DP_REEL, and targetSec (line 140) has only three targets: LAUNCH, NEUTRAL and REEL. There is no 309 park target. The repo's own test confirms the result: after a wrap the drum is at about 347.5 with collarReel() true (test_base.cpp:376). cmdReel (lines 722-725) accepts until=slip (the tidy) only when sec==SEC_NEUTRAL with both dogs open. With the collar in REEL it returns BAD_ARG. So once the wrap has run, no tidy with a free capstan is possible. BASE_SPEC section 10 is marked binding and g
>

## 5. pins.h contradicts the shared contract params.SWITCHES on nearly every GPIO: GPIO21 is a contact input in the spec but a driven output in firmware, DRV8833 and HX711 pins are swapped, and SW_GAP / SW_N20 / SW_ARM are missing

- Now (863d58d): **fixed** at `firmware/base/grappstar_base/pins.h:21-46`. This is fixed on the newest tree. pins.h now matches params.SWITCHES (base/spec/params.py:248): DRUM_A/B/C on 10/11/12, DOG_L/R on 13/14, N20 47, HOME 21 (input), GAP 39, RAIL_LOW 40, SEATED 41, ARM 42, INDEX 35, DRV8833 on 15/16 and HX711 on 17/18. PIN_DRUM_SLP and PIN_DRUM_FLT are gone, and SwId in config.h:15 now includes SW_N20, SW_GAP and SW_ARM. The fire guard (base_logic.h:693) and the de-cock path (base_logic.h:959 and 1441) both check SW_GAP.

- File: `firmware/base/grappstar_base/pins.h:24`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: The harness or cad-launch routes contacts by params.SWITCHES, which says it matches pins.h. (1) SW_HOME goes to GPIO21, which one side of the contact ties to GND. The firmware drives GPIO21 as an output (PIN_DRUM_SLP) and sets it HIGH every time the drum wakes (hwDrumSleep(true)). While the carriage is home the contact is closed, so a push-pull HIGH output is shorted straight to GND. That overloads the pad, and SW_HOME is never read at all. (2) DRV8833 AIN1/AIN2 go to GPIO15/16 and the HX711 to 17/18. The firmware PWMs 17/18 into the HX711 DT/SCK and bit-bangs the HX711 on the DRV8833 inputs, so the drum never moves or moves at random. (3) The drum code is wired A=10, B=11, C=12, but the firmware reads A=11, B=12, C=13. Every sector decodes wrong, so the drum is driven to the wrong stop or toward its hard stops. (4) BASE_SPEC section 4 needs SW_GAP for the fire and de-cock refusals ('no fire unless LAUNCH, SW_HOME, SW_GAP, SW_SEATED'). The firmware has no SW_GAP, SW_N20 or SW_ARM input, so that interlock cannot be enforced.

Evidence: params.py:244 `SW_HOME=dict(..., gpio=21)`, params.py:237-241 `SW_DRUM_A ... gpio=10`, `SW_DRUM_B gpio=11`, `SW_DRUM_C gpio=12`, `SW_DOG_L gpio=13`, `SW_DOG_R gpio=14`, params.py:245-248 `SW_GAP gpio=39`, `SW_RAIL_LOW gpio=40`, `SW_SEATED gpio=41`, `SW_ARM gpio=42`, params.py:251-252 `DRV8833=dict(ain1=15, ain2=16, ...)`, `HX711=dict(dt=17, sck=18, ...)`, params.py:236 'fw-base owns the final table ... these match it'. Against: pins.h:22-24 `#define PIN_DRUM_IN1 17`, `PIN_DRUM_IN2 18`, `PIN_DRUM_SLP 21`. pins.h:29-31 `PIN_SW_A 11 ... PIN_SW_C 13`. pins.h:41-42 `PIN_HX_DT 16`, `PIN_HX_SCK 15`. grappstar_base.ino:60 `digitalWrite(PIN_DRUM_SLP, awake ? HIGH : LOW);` and :132 `pinMode(PIN_DRUM_SLP, OUTPUT)`. config.h:15 SwId has no SW_GAP, SW_N20 or SW_ARM.

Fix: Bring pins.h, config.h SwId/swBit and WIRING.md onto params.SWITCHES (SW_DRUM_A..C = 10/11/12, SW_DOG_L/R = 13/14, SW_HOME 21, SW_GAP 39, SW_RAIL_LOW 40, SW_SEATED 41, SW_ARM 42, SW_N20 47, SW_INDEX 35, DRV8833 AIN1/AIN2 = 15/16, HX711 DT/SCK = 17/18). Delete PIN_DRUM_SLP: SLP is not wired per D9, so GPIO21 must never be an output. Move PIN_DRUM_FLT off 47 or drop it. Add SW_GAP, SW_N20 and SW_ARM to SwId and enforce SW_GAP in the fire and de-cock guards. If fw-base keeps its own table, then update params.SWITCHES in the same change, since the comment there claims the two match.

> Skeptic (confirmed): I checked the code and could not refute the claim. In params.py:234-254, SWITCHES says "fw-base owns the final table (pins.h); these match it". They do not match. params.py gives SW_DRUM_A/B/C=10/11/12, DOG_L/R=13/14, SW_HOME=21, SW_GAP=39, RAIL_LOW=40, SEATED=41, ARM=42, N20=47, DRV8833 ain1/ain2=15/16 and HX711 dt/sck=17/18. pins.h has DRUM_IN1/IN2=17/18, DRUM_SLP=21 (driven), DRUM_FLT=47, PIN_SW_A..H=11,12,13,14,39,40,41,42, SW_I=10, SW_J=35 and HX_DT/SCK=16/15. In config.h, swBit {0..8} maps SW_DRUM_A to bit 0 (GPIO11), B to GPIO12, C to GPIO13, DOG_L to 14, DOG_R to 39, SEATED to 40, HOME
>
> Skeptic (confirmed): I could not refute this. Every pin the claim cites checks out. params.py:234-252 SWITCHES says "fw-base owns the final table (pins.h); these match it", then gives DRUM_A/B/C=10/11/12, DOG_L/R=13/14, HOME=21, GAP=39, RAIL_LOW=40, SEATED=41, ARM=42, N20=47, DRV8833 ain1/ain2=15/16 and HX711 dt/sck=17/18. pins.h instead has DRUM_IN1/IN2=17/18, PIN_DRUM_SLP=21 (an output, written HIGH by hwDrumSleep at grappstar_base.ino:60 and :132), PIN_DRUM_FLT=47, switch pins A..D=11..14, E..H=39..42, I=10, J=35, and HX DT/SCK=16/15. So the comment "these match it" is false on almost every line. config.h SwId 
>

## 6. A fault in REEL or N_TO_R holds the drum in place instead of going to the WRAPPED park (309) as D31 / BASE_SPEC 7 require

- Now (863d58d): **moot** at `firmware/base/grappstar_base/base_logic.h:255`. The fault rule has changed: base/spec/params.py:150 (DRUM.fault_rules.from_REEL_or_N_TO_R) and base/spec/BASE_SPEC.md:195 now say a fault with the wraps on (REEL, N_TO_R, WRAP or the park) HOLDS the drum with MODE err DRUM_HELD and never goes to NEUTRAL; the old D31 "go to WRAPPED" rule no longer applies. The firmware does exactly this in safeToNeutral() at base_logic.h:255. DP_WRAPPED now exists (targetSec maps to SEC_LOCK, park 314), and safeTarget() at line 242 keeps the WRAPPED branch only as a guard in case the rule changes back.

- File: `firmware/base/grappstar_base/base_logic.h:244`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: A tow in REEL faults, for example the OVERCURRENT above or a STALL. raise() calls safeToNeutral(), which returns "wraps on" for any sec >= SEC_WRAP, so the drum stays in REEL with the collar engaged. Sim (t1.cpp case C): MODE ok:false DRUM_HELD "wraps on: the line must be unwrapped by the operator" with drum_pos REEL. The binding rule says a fault from REEL or N_TO_R goes to WRAPPED 309: collar to NEUTRAL, cup locked, never NEUTRAL. The firmware has no WRAPPED target at all: targetDeg has only 3 entries and targetSec() maps everything else to REEL.

Evidence: base_logic.h:244 `if (sec >= SEC_WRAP || wraps > 0) return "wraps on: the line must be unwrapped by the operator";` and :227 `if (!hold && detentClosed() != DP_NEUTRAL) { if (startDrumMove(DP_NEUTRAL, true)) ...`. config.h targetDeg = {1.5f, 41, 347.5f}. BASE_SPEC 7: "A fault from REEL or N_TO_R goes to WRAPPED (309), never NEUTRAL". params DRUM.fault_rules.from_REEL_or_N_TO_R.

Fix: Add a DP_WRAPPED target at 309 (targetDeg[3], targetSec -> SEC_LOCK, stage-4 check: both dog contacts open). In raise(), when sec is SEC_LOCK or SEC_REEL, the fault is not a drum fault, the drum code is valid and wrapsKnown is true, call startDrumMove(DP_WRAPPED, true) (safing: no wiper motion) instead of holding. Keep HOLD for a WRAP-sector fault and for drum faults.

> Skeptic (confirmed): I could not refute this. The code does what the claim says.
- base_logic.h:244: for any fault, safeToNeutral() returns "wraps on: the line must be unwrapped by the operator" whenever sec >= SEC_WRAP or wraps > 0. Faults in REEL or N_TO_R (SEC_LOCK) always meet that condition.
- raise() (around lines 213-232) has only two outcomes. It either starts startDrumMove(DP_NEUTRAL, true) or calls d.halt() and evHeld(hold). Nothing in it moves the drum to a WRAPPED park.
- No WRAPPED park target exists anywhere. config.h:102 has targetDeg[3] = {1.5, 41, 347.5} (LAUNCH, NEUTRAL, REEL). targetSec() at bas
>
> Skeptic (confirmed): The claim checks out against the code. In base_logic.h, raise() (around line 227) only ever moves the drum to DP_NEUTRAL, or else holds it. safeToNeutral() at line 244 returns "wraps on: ..." whenever sec >= SEC_WRAP or wraps > 0, so any non-drum fault in SEC_LOCK (N_TO_R) or SEC_REEL holds the drum where it is: collar engaged in REEL, then a DRUM_HELD MODE event. There is no WRAPPED drive target. targetSec() at line 140 maps everything other than LAUNCH and NEUTRAL to SEC_REEL, and the code reads targetDeg only at indices 0..2. DP_WRAPPED exists only as a reported position (line 147).

The sp
>

## 7. A fault or estop in REEL/N_TO_R holds the drum instead of going to WRAPPED 309 (D31 fault rule)

- Now (863d58d): **moot** at `/tmp/claude-0/review-wt/base-v1/base/spec/params.py:150 (code: firmware/base/grappstar_base/base_logic.h:256)`. The firmware still holds the drum in REEL/N_TO_R after a fault or estop. safeToNeutral returns "wraps on" at base_logic.h:256, and raise() and cmdStop() at lines 231 and 1155 never move the drum to WRAPPED. That matches the current spec, though. Spec commit a388984 rewrote params.py:150 fault_rules.from_REEL_or_N_TO_R to "the drum HOLDS where it is (D27, MODE err DRUM_HELD), never NEUTRAL", and BASE_SPEC.md:195 says the same, so the D31 "go to WRAPPED" rule no longer applies.

- File: `firmware/base/grappstar_base/base_logic.h:244`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: During a tow (drum in REEL, collar engaged), an OVERCURRENT or STALL trip or an estop. safeToNeutral returns 'wraps on', so raise() and cmdStop leave the drum in REEL with the collar engaged. There is no code path that ever moves the drum to the WRAPPED park, so the D31 rule 'from REEL or N_TO_R go to WRAPPED (309), never NEUTRAL' is never carried out.

Evidence: base_logic.h:244 `if (sec >= SEC_WRAP || wraps > 0) return "wraps on: the line must be unwrapped by the operator";`. base_logic.h:227 `if (!hold && detentClosed() != DP_NEUTRAL) { if (startDrumMove(DP_NEUTRAL, true)) ...`: NEUTRAL is the only safing target. DECISIONS D31 restores 'the fault rule "from REEL or N_TO_R go to WRAPPED, never NEUTRAL"'. params.DRUM.fault_rules.from_REEL_or_N_TO_R.

Fix: Once a DP_WRAPPED target exists (see the WRAPPED finding), add a branch in raise() and in cmdStop(estop) that runs before the hold. If (sec == SEC_REEL || sec == SEC_LOCK) && wrapsKnown && f is not F_DRUM_TIMEOUT or F_DRUM_SENSOR && !secBad, call startDrumMove(DP_WRAPPED, true) and set state = S_SAFING. Safing never turns the wiper, so the move must not jog. Keep holding in WRAP/PREP (the boot-in-WRAP rule).

> Skeptic (confirmed): I couldn't refute this. The claim is accurate.

The code (base_logic.h):
- raise() (lines ~212-233) has only one safing target. It moves the drum to DP_NEUTRAL when safeToNeutral() returns nullptr, and otherwise halts the drum and emits DRUM_HELD.
- safeToNeutral() line 244 returns "wraps on: ..." whenever sec >= SEC_WRAP or wraps > 0. In SEC_REEL and SEC_LOCK, wrapsAt() returns c.wrapsFull (line 162), so any fault from REEL or N_TO_R takes the hold branch. The drum stays put, and in REEL the collar stays engaged.
- cmdStop(estop) (lines ~957-973) does the same: either startDrumMove(DP_NEUTRAL
>
> Skeptic (confirmed): The claim holds up against the code. In raise() (base_logic.h:212-232) the only drum target is DP_NEUTRAL (line 227). safeToNeutral() (line 244) returns "wraps on..." whenever sec >= SEC_WRAP or wraps > 0. SEC_LOCK and SEC_REEL both come after SEC_WRAP in config.h:23, and wrapsAt() returns wrapsFull for both, so a fault in REEL or LOCK (the firmware's N_TO_R sector, which drumPos() reports as DP_WRAPPED at line 147) always ends in d.halt with state S_FAULT and a DRUM_HELD event. cmdStop(estop) at lines 966-971 does the same thing: it only tries DP_NEUTRAL and otherwise holds. A search for DP_W
>

## 8. aim never sets its own current trip: it runs with whatever trip the last motion left (2.5 A take-up, 3 A jog, or 6.5 A cocking) instead of D8's 5 A

- Now (863d58d): **fixed** at `firmware/base/grappstar_base/base_logic.h:740,753`. Both ways an aim can drive the motor now set `w.c.tripA = c.tripA` before `w.want`: the homing branch of cmdAim at line 740, and planAim at line 753, which handles the first leg, the leg after homing and each retry. In the only branch that skips it (planAim with opTravel=0), tickAim halts the motor straight away without driving it, so the 5 A trip from D8 always applies to an aim and a stale 2.5 A or 6.5 A trip can no longer carry over.

- File: `firmware/base/grappstar_base/base_logic.h:619`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: Normal mission order: final approach in NEUTRAL (reel until seated, which sets w.c.tripA = takeupTripA 2.5 A), then mode LAUNCH, then aim. The aim runs with a 2.5 A trip. Sim proof (t1.cpp case A): tripA during S_AIM was 2.50, and with an aim load of about 2.9 A (below D8's 5 A) the aim ended in an OVERCURRENT fault. Another path: cock, stop, cal cock_rest force:true, aim. The aim then runs with tripA 6.50 (case B). This breaks D8's 5 A aim protection against a jammed aim worm.

Evidence: base_logic.h:619 `homing = true; opFrom = enc.deg(); w.want(c.aimDir, c.aimDuty);` and :632 `w.want(c.aimDir, t < c.aimSlowDeg ? c.aimSlowDuty : c.aimDuty);`. No w.c.tripA assignment anywhere in cmdAim (608-626). Every other motion sets it, e.g. :733 (reel), :769 (jog), :823 (decock), :648 (cock). cmdStop (957) and raise (212) do not reset it.

Fix: In cmdAim, just before markMoving (line 624), add `w.c.tripA = c.tripA;`. Also reset `w.c.tripA = c.tripA;` in cmdStop and raise so that no state keeps a stale helper trip.

> Skeptic (confirmed): I could not refute this; the code confirms it. In base_logic.h, cmdAim (608-626) and planAim (627-634) only call w.want() and never set w.c.tripA. The aim tick doesn't either: grep finds no tripA write tied to S_AIM. The only places that put the trip back to c.tripA (5 A) are begin() (93), the end of cocking (1127), decock (823), reset (1152), reel in non-takeup mode (733) and a trip_a param write (862). The motions that set other trips are reel in NEUTRAL (takeupTripA 2.5 A, line 733), jog (jogTripA 3 A, or cockTripA 6.5 A in the cock direction, line 769), the drum-move wiper helpers (jogTrip
>
> Skeptic (confirmed): I could not refute this. cmdAim (base_logic.h:608-626), planAim (:627-634) and tickAim (:1076 on) only call w.want(c.aimDir, ...). None of them sets w.c.tripA. The only places that reset it to c.tripA (5 A) are begin (:93), decock (:823), the trip_a param write (:862), the end of cocking (:1127) and the fire reset (:1152). Reel (:733) sets 2.5 A take-up or 5 A. Jog (:769) sets 2.5, 3 or 6.5 A. The mode-move helper wiper() (:475, called at :466/467/481/482/492/550) sets jogTripA 3 A or takeupTripA 2.5 A. tickReel, tickJog, cmdStop and raise do not put the value back. So the normal order (reel u
>

## 9. Drum jog end-stop guard uses an untrusted mid-sector estimate and wrong end stops: after boot it drives past -4 / 352

- Now (863d58d): **fixed** at `firmware/base/grappstar_base/base_logic.h:922 (also :1416, config.h:137-142)`. A drum jog toward an end stop is now refused with POSITION_UNKNOWN when the angle is untrusted (sector LAUNCH with dd<0, or REEL with dd>0), see base_logic.h:922. The jog guard at :1416 now stops 2 deg before secLoDeg/secHiDeg, and config.h now sets those to the spec values -4/352, with targets {0,41,347,314} taken from params.DRUM.

- File: `firmware/base/grappstar_base/base_logic.h:1216`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: The drum is parked at REEL (347.5) or LAUNCH (1.5) and the record is untrusted (first boot, invalid record, or the moving bit set by a reset during a drum move). resolveBoot sets drumDeg to the sector middle: 343.75 in REEL, 4.75 in LAUNCH. A drum jog +1 in REEL runs until the estimate reaches 351.5, so the real drum reaches about 354 (end stop 352). A jog -1 in LAUNCH runs until the estimate reaches -2.5, so the real drum reaches about -4.6 (end stop -4). t2.cpp prints 'after jog: est=351.55 true=354.04' and 'est=-2.57 true=-4.60'. Even when the angle is trusted, the guard stops 0.5 deg short of 352 because the targets and end stops in config are not the spec values.

Evidence: base_logic.h:1216 `if ((jogDir < 0 && sec == SEC_LAUNCH && drumDeg <= c.targetDeg[DP_LAUNCH] - 4) || (jogDir > 0 && sec == SEC_REEL && drumDeg >= c.targetDeg[DP_REEL] + 4))`. base_logic.h:280 `drumDeg = 0.5f * (fmaxf(secLo(sec), c.targetDeg[0]) + fminf(secHi(sec), c.targetDeg[2])); degTrusted = false;`. config.h:101 `float secLoDeg = -5, secHiDeg = 355;`. config.h:102 `float targetDeg[3] = {1.5f, 41, 347.5f};`. BASE_SPEC 7: 'LAUNCH 0, NEUTRAL 41 ... REEL 347. End stops -4 and 352'.

Fix: Set secLoDeg=-4, secHiDeg=352 and targetDeg {0, 41, 347} (generate them from params.DRUM). In cmdJog for the drum, refuse POSITION_UNKNOWN when !degTrusted && ((sec==SEC_LAUNCH && dd<0) || (sec==SEC_REEL && dd>0)). Otherwise base the guard on the end stop with a margin, not on target+-4: stop at drumDeg <= secLoDeg + 2 and drumDeg >= secHiDeg - 2.

> Skeptic (confirmed): The core claim holds up when I trace the code. In resolveBoot (base_logic.h:280), an untrusted drum record sets drumDeg = 0.5*(max(secLo,targetDeg[0]) + min(secHi,targetDeg[2])), with degTrusted=false. secLo/secHi come from config secLoDeg=-5 / secHiDeg=355 and the edges {8,...,340}. That gives 4.75 in LAUNCH and 343.75 in REEL, while the drum actually sits near its park point (1.5 or 0 / 347.5 or 347). Only an edge crossing sets degTrusted back to true (line 379), and trackDrum just dead-reckons and clamps to the sector range [-5,8] or [340,355].

cmdJog for the drum (lines 770-775) allows dd
>
> Skeptic (confirmed): I checked the code and the claim holds. In resolveBoot (base_logic.h:277-280), when the record is untrusted (no record, an invalid one, or MV_DRUM set) the drum angle becomes the sector middle and degTrusted=false. With the config values (secLo/secHi -5/355, edges 8 and 340, targets 1.5/347.5) that gives 343.75 in REEL and 4.75 in LAUNCH.

Neither drum jog direction that matters is refused:
- In REEL, cmdJog (base_logic.h:746-790) allows dd>0. Only dd<0 returns WRAPS_ON.
- In LAUNCH, dd<0 is let through by the `!(sec==SEC_LAUNCH && dd<0)` exception.
- Nothing in cmdJog checks degTrusted.

trac
>

## 10. SW_GAP does not exist in the firmware: fire and rest are accepted, and decock ends, without the big gear in its gap

- Now (863d58d): **fixed** at `firmware/base/grappstar_base/base_logic.h:784`. SW_GAP is now a real input: SwId, swBit bit 7, and pins.h:36 on GPIO 39. fireCheck (base_logic.h:784-785), cock from REST (:769), cal cock_rest unless force (:959), the tickDecock end condition (:1441) and resolveBoot (:354/:358) all require SW_GAP closed. The tests that check this, at test_base.cpp:850-851 and :769-771, pass (384/384).

- File: `firmware/base/grappstar_base/config.h:14`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: The 6.5 A peak at the release browns out the board and it resets in the RESETTING phase (SW_HOME closed, big gear partway through its reset, before the gap). On boot the phase is UNKNOWN with the hint 440, so cock gives LOADED_UNKNOWN and decock gives SWITCH_DISAGREE. The operator's way out is 'cal cock_rest', which only checks SW_HOME and declares REST with cockPhase 0. fireCheck then accepts a shot from REST with the gear not in its gap. The draw finds SW_HOME opening about 60-95 deg late and raises SWITCH_DISAGREE mid-draw, which leaves the spring partly drawn and the phase unknown. The spec's refusal 'no fire unless ... SW_GAP' and the decock end 'until SW_HOME and SW_GAP close' cannot be implemented: there is no such input.

Evidence: config.h:14 `enum SwId { SW_DRUM_A, SW_DRUM_B, SW_DRUM_C, SW_DOG_L, SW_DOG_R, SW_SEATED, SW_HOME, SW_RAIL_LOW, SW_WRAP_HOME, SW_INDEX, SW_COUNT };` (no SW_GAP). base_logic.h:662 `if (!((phase == PH_COCKED && !closed(SW_HOME)) || (phase == PH_REST && closed(SW_HOME)))) return "NOT_COCKED";`. base_logic.h:1237-1240: decock declares REST on SW_HOME + 15 deg alone. base_logic.h:806-807: cal cock_rest checks SW_HOME only. BASE_SPEC 10: 'De-cock: from a partial cock run backward until SW_HOME and SW_GAP close. Refusals: no fire unless LAUNCH, SW_HOME, SW_GAP, SW_SEATED'. params.SWITCHES SW_GAP gpio 39.

Fix: Add SW_GAP to SwId, swBit, SW_TEL and pins.h (params gpio 39). Require closed(SW_GAP) in fireCheck's REST branch, in cmdCock from REST, in cal cock_rest (unless force), and in tickDecock's end condition (closed(SW_HOME) && closed(SW_GAP)). In resolveBoot, when SW_HOME and SW_GAP are both closed, set phase REST; with SW_HOME closed and SW_GAP open, keep UNKNOWN and let cock creep forward to the gap.

> Skeptic (confirmed): The main point holds up. SW_GAP is missing everywhere in the firmware. config.h:14 SwId has no SW_GAP, and neither do swBit, SW_NAME or SW_TEL. pins.h gives GPIO39 to PIN_SW_E, which swBit maps to SW_DOG_R, so params.SWITCHES SW_GAP gpio=39 clashes with it. A grep of firmware/ finds no "SW_GAP" at all. The spec makes the contact binding in two places. BASE_SPEC section 4 lists it in the table. Section 10 (lines 159-160) says decock runs "until SW_HOME and SW_GAP close" and "no fire unless LAUNCH, SW_HOME, SW_GAP, SW_SEATED". DECISIONS has no entry that drops SW_GAP. D27 removes SW_DOG_L/R but 
>
> Skeptic (confirmed): Not refuted. The core claim checks out. BASE_SPEC section 4 ("binding") lists SW_GAP (cad-launch, "big gear in its toothless gap (rest)"), and params.SWITCHES gives it gpio 39. Section 10 says de-cock runs "until SW_HOME and SW_GAP close" and that there is "no fire unless LAUNCH, SW_HOME, SW_GAP, SW_SEATED...".

The firmware has no SW_GAP anywhere. A grep of firmware/base finds no match. config.h:14 SwId has no entry for it, and pins.h uses GPIO39 for PIN_SW_E, which the default swBit maps to SW_SEATED, so it conflicts with params as well. The cited checks rely on SW_HOME alone:
- fireCheck (b
>

## 11. Contact bit map in config.h does not match the WIRING.md / BENCH_TEST.md pin table: from GPIO39 up, every contact is read as a different switch

- Now (863d58d): **fixed** at `firmware/base/grappstar_base/config.h:15`. The firmware, docs and params now give the same pin for every contact. config.h:15/38 maps SwId in order to PIN_SW_A..L, and pins.h:29-40 gives GPIO 10,11,12,13,14,47,21,39,40,41,42,35. WIRING.md:23-34 and :145-156, BENCH_TEST.md:66-72 and params.py SWITCHES:251-262 all list the same pins and telemetry keys (for example SW_HOME on 21 'H', SW_SEATED on 41 'S', SW_RAIL_LOW on 40 'L'), so wiring to the docs no longer swaps any contacts.

- File: `firmware/base/grappstar_base/config.h:37`
- Severity: medium (claimed high); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: The base is wired exactly as WIRING.md says: GPIO39 = SW_SEATED, GPIO40 = SW_HOME, GPIO41 = SW_RAIL_LOW, GPIO42 = SW_WRAP_HOME, and one SW_DOG on GPIO14. The firmware maps SwId to bits with swBit = {0..8,-1} in enum order SW_DRUM_A,B,C,SW_DOG_L,SW_DOG_R,SW_SEATED,SW_HOME,SW_RAIL_LOW,SW_WRAP_HOME. That gives bit4 (GPIO39) = SW_DOG_R, bit5 (GPIO40) = SW_SEATED, bit6 (GPIO41) = SW_HOME, bit7 (GPIO42) = SW_RAIL_LOW and bit8 (GPIO10) = SW_WRAP_HOME. Results: (a) The carriage-home contact on GPIO40 is read as SW_SEATED. Whenever the carriage is home, the requireSeated fire interlock passes with no dart on the rail. (b) The rail-low contact on GPIO41 is read as SW_HOME. Once the rail rises during aim, SW_HOME 'opens' and base_logic.h:427 drops phase REST to PH_UNKNOWN, so cock and fire are blocked. (c) Aim homing uses SW_RAIL_LOW, which is now the wrap-arm contact on GPIO42, so the aim phase is wrong. (d) A dart seated on GPIO39 reads as SW_DOG_R. BENCH_TEST step 2 also fails: pressing SW_HOME on GPIO40 shows "S":1, not "H":1.

Evidence: config.h:37 `int8_t swBit[SW_COUNT] = {0, 1, 2, 3, 4, 5, 6, 7, 8, -1};` with config.h:15 `enum SwId { SW_DRUM_A, SW_DRUM_B, SW_DRUM_C, SW_DOG_L, SW_DOG_R, SW_SEATED, SW_HOME, SW_RAIL_LOW, SW_WRAP_HOME, SW_INDEX, ...}`. pins.h:33-37 `PIN_SW_E 39 ... PIN_SW_I 10`. WIRING.md:27-31 `| 14 | PIN_SW_D | contact SW_DOG |`, `| 39 | PIN_SW_E | contact SW_SEATED |`, `| 40 | PIN_SW_F | contact SW_HOME |`, `| 41 | PIN_SW_G | contact SW_RAIL_LOW |`, `| 42 | PIN_SW_H | contact SW_WRAP_HOME |`. WIRING.md section 1 text: 'bit 0..7 = PIN_SW_A..PIN_SW_H ... all eight'. BENCH_TEST.md:48-52 has the same table.

Fix: Pick one table and make all three files agree. Either rewrite WIRING.md sections 1 and 5 and the BENCH_TEST.md step 2 table to the firmware order: GPIO14 SW_DOG_L, GPIO39 SW_DOG_R, GPIO40 SW_SEATED, GPIO41 SW_HOME, GPIO42 SW_RAIL_LOW, GPIO10 SW_WRAP_HOME, GPIO35 SW_INDEX, with keys l/r instead of D. Or set swBit to match WIRING.md. Preferably make both follow params.SWITCHES (see the next finding).

> Skeptic (confirmed): The claim holds up. In grappstar_base.ino:28, SW_PINS = {PIN_SW_A..PIN_SW_J} = GPIO {11,12,13,14,39,40,41,42,10,35}. Bit i is the contact on SW_PINS[i] (base_logic.h:44). config.h:37 sets swBit = {0..8,-1} in SwId order {DRUM_A,B,C,DOG_L,DOG_R,SEATED,HOME,RAIL_LOW,WRAP_HOME,INDEX}. So the firmware reads GPIO39 as SW_DOG_R, GPIO40 as SW_SEATED, GPIO41 as SW_HOME, GPIO42 as SW_RAIL_LOW and GPIO10 as SW_WRAP_HOME.

WIRING.md:27-31 and its section-1 text ("bit 0..7 ... all eight ... SW_DOG, SW_SEATED, SW_HOME, SW_RAIL_LOW, SW_WRAP_HOME on A to H") put SW_SEATED on 39, SW_HOME on 40, SW_RAIL_LOW on
>
> Skeptic (confirmed): I could not refute this. The mismatch is real. config.h:15 defines the enum order SW_DRUM_A, B, C, SW_DOG_L, SW_DOG_R, SW_SEATED, SW_HOME, SW_RAIL_LOW, SW_WRAP_HOME, SW_INDEX. config.h:37 maps them with swBit = {0,1,2,3,4,5,6,7,8,-1}. pins.h maps bits 0..9 to GPIO 11, 12, 13, 14, 39, 40, 41, 42, 10, 35. So in the firmware, GPIO39 is SW_DOG_R, GPIO40 is SW_SEATED, GPIO41 is SW_HOME, GPIO42 is SW_RAIL_LOW and GPIO10 is SW_WRAP_HOME.

firmware/base/WIRING.md section 1 (lines 27-31 and 37-39) still describes the old 8-contact layout: one SW_DOG on GPIO14, then SW_SEATED, SW_HOME, SW_RAIL_LOW and S
>

## 12. Tow limit is not averaged over 200 ms: about 25 ms EMA plus a single-tick trigger

- Now (863d58d): **fixed** at `firmware/base/grappstar_base/base_logic.h:386`. The tow tension filter now uses alpha = dt / c.tensionAvgMs, where tensionAvgMs = 200 (config.h:195). That gives a 200 ms time constant, so a 30-50 ms spike lifts tCur by only about 15-22% of its height. LIMIT still triggers on a single tick (line 1390), but only on the filtered value, so a short spike no longer ends the tow.

- File: `firmware/base/grappstar_base/base_logic.h:328`
- Severity: low (claimed low); unverified (low)
- Owner: fw-base

Scenario: During a tow the current-based tension tCur is filtered with alpha 0.2 per 5 ms tick, a time constant of about 22 ms, and tickReel enters LIMIT on the first tick where frac >= 1. A short current spike of about 30-50 ms (a dart hop, a gear tooth) above the limit equivalent ends the tow with LIMIT/HOLD (or IDLE for until stop/seated), where D8's 200 ms average would have ridden through it.

Evidence: base_logic.h:328 `tCur = winding ? tCur + (tc - tCur) * 0.2f : ...` (tick 5 ms); :1190 `if (frac >= 1.0f && now - w.startT > c.tensionIgnoreMs) { w.halt(now, io.wiper); enterHold("tension limit"); return; }`. D8: "tow trip at measured free current plus the 50 N equivalent, averaged over 200 ms".

Fix: Use a 200 ms average: `float a = dt / 200.f; tCur += (tc - tCur) * a;` (pass dt into the filter), or keep a 40-sample boxcar of tc and compare its mean with maxN.


## 13. SW_GAP is not implemented: fire, arm and decock never check the big gear is in its toothless gap

- Now (863d58d): **fixed** at `firmware/base/grappstar_base/base_logic.h:784`. SW_GAP now exists and is fitted by default (config.h:15, swBit bit 7, pins.h:36). fireCheck needs it closed at REST and open when COCKED (base_logic.h:784-785), so arm and fire check it. Cocking refuses REST with the gear out of its gap (769), decock finishes only on SW_HOME plus SW_GAP (972, 1441), and boot demotes REST if the gap is open (354, 358). One small gap is left: cmdAim (726) does not read SW_GAP itself. It relies on phase==REST, and that phase can still be forced.

- File: `firmware/base/grappstar_base/base_logic.h:662`
- Severity: low (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: BASE_SPEC 10: "no fire unless LAUNCH, SW_HOME, SW_GAP, SW_SEATED and a valid token" and "De-cock: ... run backward until SW_HOME and SW_GAP close". The firmware has no SW_GAP contact (it is missing from the SwId enum, swBit, pins and PROTOCOL). Example: phase REST is declared with 'cal cock_rest force:true' while the big gear actually sits in the reset arc just after a release. SW_HOME is closed in both cases, so arm/fire are accepted and the fire-from-rest cycle starts with the gear out of its gap. SW_GAP would have refused it. Decock likewise stops on SW_HOME alone (decockExtraDeg run-on) with the gear position unverified.

Evidence: config.h:14 `enum SwId { SW_DRUM_A, SW_DRUM_B, SW_DRUM_C, SW_DOG_L, SW_DOG_R, SW_SEATED, SW_HOME, SW_RAIL_LOW, SW_WRAP_HOME, SW_INDEX, SW_COUNT };` (no SW_GAP); base_logic.h:662 `if (!((phase == PH_COCKED && !closed(SW_HOME)) || (phase == PH_REST && closed(SW_HOME)))) return "NOT_COCKED";`; :1235 decock ends on closed(SW_HOME) only. grep for SW_GAP in firmware/base: no hits.

Fix: Add SW_GAP to SwId, swBit, SW_NAME/SW_TEL, pins.h and PROTOCOL/WIRING. In fireCheck, when phase is REST require closed(SW_GAP), else return NOT_AT_REST, or NO_SENSOR if SW_GAP is not fitted. When phase is COCKED require !closed(SW_GAP). In cmdCal decock and tickDecock, finish only when both SW_HOME and SW_GAP are closed. In cmdAim, refuse unless SW_GAP is closed (aim only at HOME).

> Skeptic (confirmed): The core claim holds. BASE_SPEC section 4 lists SW_GAP as a binding contact. Section 10 says "De-cock: ... run backward until SW_HOME and SW_GAP close" and "no fire unless LAUNCH, SW_HOME, SW_GAP, SW_SEATED and a valid token". params.py:245 defines SW_GAP with gpio 39. D27 in DECISIONS.md merges SW_DOG_L/R but never removes SW_GAP, and the workplan does not defer it. In the firmware, config.h has no SW_GAP in SwId and grep finds no GAP contact anywhere in firmware/base. GPIO 39 is just PIN_SW_E, assigned through swBit to another contact. fireCheck (base_logic.h:662) gates on phase plus SW_HOME
>
> Skeptic (confirmed): The literal claim is true, but it is much less serious than stated, and the fault lies mainly upstream of the firmware.

Confirmed:
- BASE_SPEC section 4 says its sensor table is binding, and it lists SW_GAP (owner cad-launch). Section 10 lines 159-160 say fire needs SW_GAP and de-cock runs until SW_HOME and SW_GAP close. params.py:245 gives SW_GAP GPIO 39.
- The firmware has no SW_GAP anywhere: not in the SwId enum in config.h, not in WIRING.md's 8-pin table (where GPIO 39 is used for SW_SEATED), and not in PROTOCOL. fireCheck (base_logic.h:662) gates only on SW_HOME plus the phase. tickDecoc
>

## 14. Wiper backward jog is allowed in NEUTRAL/WRAP_PREP/WRAP (unlimited travel), but the spec allows backward outside LAUNCH only for the unload jog

- Now (863d58d): **fixed** at `firmware/base/grappstar_base/base_logic.h:911`. cmdJog's wiper branch now has `if (p != DP_LAUNCH && dd != c.reelDir) return "BACKSTOP";`. drumPos() returns DP_NEUTRAL in NEUTRAL and DP_BETWEEN in WRAP_PREP/WRAP, so a backward wiper jog is refused everywhere outside LAUNCH.

- File: `firmware/base/grappstar_base/base_logic.h:767`
- Severity: low (claimed low); unverified (low)
- Owner: fw-base

Scenario: Drum in NEUTRAL or in the WRAP sector. {"cmd":"jog","motor":"wiper","dir":-1,"ms":1500} is accepted and repeatable. Sim (t3.cpp): ok, the wiper turned -210 deg in NEUTRAL; it is also accepted in the WRAP sector (t2.cpp). The BACKSTOP refusal only applies at sec >= SEC_REEL.

Evidence: base_logic.h:767 `if ((sec >= SEC_REEL || closed(SW_DOG_R)) && dd != c.reelDir) return "BACKSTOP";` with no refusal for dd != reelDir when sec is SEC_NEUTRAL..SEC_WRAP. BASE_SPEC 10: "wiper backward refused outside LAUNCH except the 30 deg unload jog".

Fix: In cmdJog (wiper branch) add `if (p != DP_LAUNCH && !closed(SW_DOG_L) && dd != c.reelDir) return "BACKSTOP";` so that only the internal centering/unload jog in startDrumMove/tickDrumMove turns the wiper backward outside LAUNCH.


## 15. Drum jog backward through the WRAP sector unwraps with the wiper stopped, bypassing the unwrap take-up interlock (D16)

- Now (863d58d): **partly fixed** at `firmware/base/grappstar_base/base_logic.h:920-921,1421`. Fixed: a drum jog started in SEC_WRAP (or while wrapsDoubt) is now refused with WRAPS_ON (line 921), and tickJog halts any drum jog the moment it enters the wrap window (line 1421). A backward jog can therefore no longer unwrap with the wiper stopped. Still open: a backward drum jog that starts in SEC_LOCK with SW_DOG_R closed is accepted, because line 920 refuses only sec == SEC_REEL. It ramps the collar R toward N with no centering jog until it reaches WRAP and halts. That is the shift-without-centering-jog part of the finding.

- File: `firmware/base/grappstar_base/base_logic.h:775`
- Severity: low (claimed low); unverified (low)
- Owner: fw-base

Scenario: Drum in WRAP (or N_TO_R/LOCK below REEL). Each {"cmd":"jog","motor":"drum","dir":-1,"ms":1500} is accepted and drives the cup back about 33 deg with no take-up, freeing line as slack. Sim (t2.cpp): one jog from 166 deg made 0.082 m of slack. Repeated jogs unwrap fully. The same branch also lets a backward drum jog leave LOCK with SW_DOG_R still closed (only sec == SEC_REEL is refused), which is a shift without the centering jog.

Evidence: base_logic.h:774-775 `if ((sec <= SEC_LN || loadedKnown()) && !(sec == SEC_LAUNCH && dd < 0)) {...}` / `if (sec == SEC_REEL && dd < 0) return "WRAPS_ON";`. There is no check for SEC_PREP..SEC_LOCK or closed(SW_DOG_R). D16: "Unwrap: collar back to NEUTRAL first, then the cup unwinds while the wiper turns forward (take-up only)".

Fix: Change line 775 to `if (dd < 0 && (sec >= SEC_PREP || closed(SW_DOG_R))) return "WRAPS_ON";` (use unwrap, which runs the take-up and centering jog). If a bench override is needed, gate it behind an explicit force flag and still refuse while SW_DOG_R is closed.


## 16. wrapsFull = 4, but the cup lays 7 wraps: telemetry and the WRAPPED event report the wrong count

- Now (863d58d): **fixed** at `firmware/base/grappstar_base/config.h:146`. wrapsFull is now set from P_DRUM_WRAP_TURNS, which params_gen.h:188 defines as 7 (gen_params.py emits it), and it falls back to 7 if that is missing. wrapsAt(), the WRAPPED event and the SW_ARM pulse check therefore all use 7, and PROTOCOL.md:88-89 now say 7 turns.

- File: `firmware/base/grappstar_base/config.h:104`
- Severity: low (claimed low); unverified (low)
- Owner: fw-base

Scenario: After a full wrap the WRAPPED event says wraps:4 and telemetry wraps tops out at 4. The spec has 7 cup turns over 62..302 (params.DRUM.wrap_turns 7, D16 'Wraps stay 7'). A Jetson that checks wraps==7 before the tidy/tow would never see a complete wrap. During WRAP the estimate is also scaled to 4 (wrapsAt).

Evidence: config.h:104 `int wrapsFull = 4;`. base_logic.h:164 `float f = (deg - c.wrapFromDeg) / (c.wrapToDeg - c.wrapFromDeg) * c.wrapsFull;`. params.DRUM: `wrap_turns=7`. BASE_SPEC 7: 'WRAP (62..302 ... 7 cup turns'.

Fix: Set wrapsFull = 7 (emit P_DRUM_WRAP_TURNS from gen_params.py and use it). Update PROTOCOL.md:83 to say 7.


## 17. Wiper current is not scaled below duty 60, so the 3 A jog trip and the 17 A hard trip are blind at flyDuty 51 (fly-in, REEL centering jog, LAUNCH wiggle)

- Now (863d58d): **fixed** at `firmware/base/grappstar_base/config.h:167`. flyDuty is now 64. That is above minScaleDuty 60 (drives.h:17,50), so the fly-in, REEL centering jog and LAUNCH wiggle (base_logic.h:560,576,662) now get the 255/duty correction and the 3 A and 17 A trips see the real current. The other helper duties are all at least 60 as well (nudge 90, decock 120, takeup 102, jog 150, reelMin 70); the only thing left undone is the suggested startup assert that would keep a future config change from dropping below minScaleDuty.

- File: `firmware/base/grappstar_base/drives.h:96`
- Severity: low (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: The IS pin only reports while the high side conducts, so averaged IS volts equal I x duty/255. Below minScaleDuty = 60 the code skips the 255/duty correction. Every helper motion at flyDuty = 51 (20 percent) therefore reads about 0.2 x the real winding current. Example: the collar jams during REEL fly-in or the REEL centering jog. At 20 percent duty the stalled wiper draws about 0.2 x 18 A = 3.6 A, but ampsRaw reads about 0.72 A, so jogTripA = 3.0 never fires. Even hardA = 17 A would need about 85 A real. Only the AS5600 stall detector is left. In D12 degraded mode (no magnet) rateDps is NaN, so the motor holds a stall until the motion timeout. This breaks the D8 per-state current trip for every flyDuty motion.

Evidence: drives.h:63 `int minScaleDuty = 60; // below this duty the IS reading is too short a pulse to scale reliably`. drives.h:95-96 `float a = isVolts * c.ampsPerVolt - c.offsetA; ... if (outDir && outDuty >= c.minScaleDuty) a *= 255.f / outDuty;`. drives.h:101 `bool over = (ampsRaw > c.tripA && ...) || ampsRaw > c.hardA;`. config.h:117 `int flyDuty = 51; // 20 percent`. base_logic.h:466 `wiper(c.reelDir, c.flyDuty, c.jogTripA)`, :482 `else if (fly) wiper(c.reelDir, c.flyDuty, c.jogTripA);`, :550 `wiper(c.aimDir, c.flyDuty, c.jogTripA)`.

Fix: Always scale once the ramp is over: `if (outDir && outDuty > 0) a *= 255.f / fmaxf(outDuty, 25.f);`. Rely on the 1-2.2 uF IS capacitor (make it mandatory in WIRING.md section 3) so short pulses average correctly. Or keep minScaleDuty but raise flyDuty to at least 60 and assert at startup that every configured helper duty (flyDuty, takeupDuty, nudgeDuty, decockDuty) is at least minScaleDuty.

> Skeptic (confirmed): The code behaviour is real; the line numbers are off. The scaling line is drives.h:50, not :96, because the file is only 109 lines. WiperDrive::tick applies `a *= 255.f / outDuty` only when outDuty >= minScaleDuty (60). The sim model (test/sim.h:174-176, IS = I x duty/255) and hwWiperIs (8 instantaneous ADC samples, no forced averaging) both give a reading of about I x duty/255. Every flyDuty = 51 helper uses jogTripA = 3.0 A, so ampsRaw comes out at about 0.2 x the real current. Sites: the REEL fly-in through mvWiperRule (base_logic.h:482), the REEL first leg of the centering jog (:466, mvSta
>
> Skeptic (confirmed): The mechanism is real, but most of the consequences claimed are overstated. drives.h lines 48-50 (not 96 as cited) skip the 255/duty correction when outDuty < minScaleDuty (60). flyDuty is 51 (config.h:117). It is used for the REEL window take-up (base_logic.h:466), REEL fly-in (:482) and the LAUNCH/REEL engage wiggle (:550), each with jogTripA = 3.0 A. hwWiperIs (grappstar_base.ino:83) averages 8 instantaneous ADC samples. The cap is optional (WIRING.md section 3). Either way the reading averages to roughly I x duty/255. So at duty 51 a real 3.6 A stall reads about 0.7 A and jogTripA never fi
>

## Refuted (dropped)

- `firmware/base/grappstar_base/base_logic.h:854` 'set require_seated false' removes the binding SW_SEATED fire interlock and keeps it off across reboots
