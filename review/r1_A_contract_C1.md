# Round 1 A: contract C1, base firmware vs Jetson

Reviewed at base-v1 commit d07a014. Method: independent finder subagents, one per lens; every finding of
medium severity or above was checked by two skeptic subagents told to refute it; kept only if fewer than
two refuted it. Low findings are listed unverified. Severity is the finder's, lowered when both skeptics
agreed it is real but rated it lower.

Confirmed: 15. Refuted and dropped: 0.

| # | Sev | File:line | Owner | Finding |
|---|---|---|---|---|
| 1 | high | `firmware/base/PROTOCOL.md:54` | fw-base | PROTOCOL.md still describes the pre-D16 commands: wrap/unwrap direction, refusal codes, reel in NEUTRAL, until:"slip", limit_n default and wrap count all differ from the firmware |
| 2 | high | `firmware/base/PROTOCOL.md:54` | fw-base | PROTOCOL.md (C1 'final') documents a drum order and wrap/unwrap/reel rules that base-1.0 does not implement |
| 3 | high | `firmware/base/grappstar_base/base_logic.h:685` | fw-base | Firmware cannot run the binding section 10 / D22 sequence: wrap goes straight to REEL with no WRAPPED 309 park, and the take-up tidy works only in NEUTRAL before the wrap |
| 4 | high | `jetson/grappstar/comms/mock_base.py:1159` | jetson | mock_base.py models a different drum than base-1.0, so mission tests pass on paths the real firmware never takes and the real paths are never tested |
| 5 | high | `jetson/grappstar/mission.py:924` | jetson | FINAL_APPROACH sends a command the firmware does not have, so every run seats the dart on the capstan and then unwraps from REEL with the dart seated (the D16 hazard) |
| 6 | medium | `firmware/base/PROTOCOL.md:76` | fw-base | PROTOCOL.md section 3 (and mock_base.py, built to it) describe a different drum model than the firmware for drum_pos, mode and drum_deg |
| 7 | medium | `jetson/grappstar/mission.py:282` | jetson | A fault mid-cock (or mid-fire from rest) leaves the spring drawn: the mission retries cock into LOADED_UNKNOWN forever, never offers a decock, and suppresses the 'still cocked' warning |
| 8 | medium | `jetson/grappstar/mission.py:492` | jetson | Mission sends `tidy` and `retrieve`, which no firmware command handles. The tidy is silently skipped and the final approach falls back to a capstan wrap/reel/unwrap cycle |
| 9 | medium | `jetson/grappstar/mission.py:492` | jetson | TIDY step is dead code against base-1.0 ('tidy' is UNKNOWN_CMD and base.reel rejects until='slip'), and the mission order ANCHOR->TIDY->WRAP contradicts the binding WRAP->ANCHOR->TIDY->REEL order |
| 10 | medium | `jetson/grappstar/mission.py:543` | jetson | is_wrapped() reads firmware drum_pos WRAPPED (a transient sector) or a wraps estimate of 4 as 'wrapped and engaged', so the mission skips the wrap and the reel is then refused |
| 11 | low | `firmware/base/grappstar_base/base_logic.h:817` | fw-base | `cal decock` with the carriage home and phase UNKNOWN replies ok but sends no RESET event, although PROTOCOL promises an immediate RESET |
| 12 | low | `firmware/base/grappstar_base/base_logic.h:1178` | fw-base | The firmware raises fault code DART_STUCK, which is not in the contract's fault list or in the Jetson FAULTS tuple |
| 13 | low | `firmware/base/grappstar_base/config.h:17` | fw-base | Telemetry sw keys differ: the firmware sends l, r and I; PROTOCOL.md, base.py and the mock use a single D |
| 14 | low | `jetson/grappstar/comms/mock_base.py:493` | jetson | Mock base answers NOT_ARMED for an expired token where the firmware answers TOKEN_EXPIRED, and a test locks the mock behaviour in |
| 15 | low | `jetson/grappstar/ui/static/app.js:22` | jetson | The operator page never shows 'Dart seated': it reads base.sw.seated, but telemetry uses sw.S |

## 1. PROTOCOL.md still describes the pre-D16 commands: wrap/unwrap direction, refusal codes, reel in NEUTRAL, until:"slip", limit_n default and wrap count all differ from the firmware

- File: `firmware/base/PROTOCOL.md:54`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: The Jetson implements against PROTOCOL.md (C1 'final'), and the mock base matches it. On the real firmware: (1) `wrap` from REEL is impossible: mode REEL without wraps -> NOT_WRAPPED. wrap actually runs NEUTRAL->REEL and refuses with NOT_IN_NEUTRAL (not NOT_IN_REEL) or LAUNCH_BUSY. So the mission's `NOT_IN_REEL -> mode REEL` recovery (mission.py:764-766) can never match: from LAUNCH, wrap raises NOT_IN_NEUTRAL. (2) `unwrap` ends in NEUTRAL, not at the REEL stop point. (3) `reel` is accepted in NEUTRAL (take-up only, at most 40 %), and there is an undocumented until:"slip". These are the only ways to do the tidy and the take-up final approach. (4) The doc lists `mode` REEL refusal WRAPS_ON, but the firmware answers NOT_WRAPPED. Wiper jog has an undocumented SHIFTING refusal. A non-integer numeric id gets an undocumented BAD_ID reply. (5) limit_n default is 50 in firmware (config.h:130) versus 20 in the doc, so a `reel` without max_n pulls 2.5x the documented default. Wraps at WRAPPED are 4 (config.h:104), and DECISIONS D16 says 7.

Evidence: PROTOCOL.md:54 `wrap | | drum on from REEL to the WRAPPED stop point ... | NOT_IN_REEL, HOLDING, common`; :55 `unwrap | | drum back to the REEL stop point`; :56 reel refusals `NOT_WRAPPED (reel needs the wraps on...)`, until `stop|seated|tension`; :49 mode refusals list `WRAPS_ON` but not NOT_WRAPPED; :137 `limit_n | 20`. Firmware base_logic.h:591 `if (sec < SEC_LOCK) return "NOT_WRAPPED";` (mode REEL), :686 comment `wrap: from NEUTRAL through the WRAP sector ... to REEL`, :692 `return "NOT_IN_NEUTRAL"`, :702 unwrap `drum back to NEUTRAL`, :718 `U_SLIP`, :722-723 reel in NEUTRAL, :759 `return "SHIFTING"`, :881 `BAD_ID`; config.h:130 `float limitN = 50.0f`

Fix: Update PROTOCOL.md to match base_logic.h (version bump, since the change is not additive). wrap: NEUTRAL -> REEL (WRAPPED event, then MODE), refusals NOT_IN_NEUTRAL, LAUNCH_BUSY, POSITION_UNKNOWN, HOLDING. unwrap: REEL -> NEUTRAL, refusals NOT_WRAPPED, POSITION_UNKNOWN, LAUNCH_BUSY, ANGLE_SENSOR. mode REEL: NOT_WRAPPED (use wrap). reel: accepted in NEUTRAL as take-up only (duty capped, until stop|seated|slip; tension refused NOT_WRAPPED); until:"slip" = tidy, ends with LIMIT holding:false; until:"seated" in NEUTRAL raises DART_STUCK after 20 s. Add SHIFTING to jog, BAD_ID to the id rules, DART_STUCK to the fault list, limit_n default 50, and the real wrap count. Tell the jetson owner the contract changed.

> Skeptic (confirmed): I checked every point against the actual files and could not refute the claim. The firmware sits under firmware/base/grappstar_base/, not firmware/base/ as the claim's paths say, but the line numbers match.

What PROTOCOL.md (headed "contract C1, final", base-1.0) says:
- :54 `wrap` goes from REEL to WRAPPED, refusal NOT_IN_REEL.
- :55 `unwrap` goes back to the REEL stop point.
- :56 `reel` is refused with NOT_WRAPPED unless the wraps are on; `until` is stop|seated|tension only.
- :49 `mode` refusals list WRAPS_ON only.
- :110 the UNWRAPPED event means "back at the REEL detent".
- :137 limit_n
>
> Skeptic (confirmed): I checked the claim against the code and it holds. PROTOCOL.md was marked "final C1" in ce68344. Its last update was 39badb1, which came before the D16 firmware commits 93bfb1c and a6e2394. Those commits changed the behaviour in ways that cannot be added on, but PROTOCOL.md was neither updated nor given a new version, even though its own line 4 says a version bump is required for changes like that.

What I confirmed in firmware/base/grappstar_base/base_logic.h:
- cmdWrap refuses with POSITION_UNKNOWN, NOT_IN_NEUTRAL (when sec < SEC_NEUTRAL) and LAUNCH_BUSY, and moves the drum to DP_REEL. The d
>

## 2. PROTOCOL.md (C1 'final') documents a drum order and wrap/unwrap/reel rules that base-1.0 does not implement

- File: `firmware/base/PROTOCOL.md:54`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: The Jetson side (mission.py, the mock, base.py docstrings) was written against PROTOCOL.md. A Jetson that follows section 8 sends mode REEL from NEUTRAL (the firmware refuses NOT_WRAPPED), expects wrap to go REEL -> WRAPPED and unwrap to return to REEL (the firmware goes NEUTRAL -> REEL and REEL -> NEUTRAL), expects reel only at WRAPPED, and does not know until:'slip', the NEUTRAL take-up final approach, the DART_STUCK fault or the NOT_IN_NEUTRAL refusal. Following the documented typical mission on the real base fails at step 7, and the documented final approach (reel until seated while wrapped, then unwrap) is the D16 hazard.

Evidence: PROTOCOL.md:54 `wrap | drum on from REEL to the WRAPPED stop point ... | NOT_IN_REEL`; :55 `unwrap | drum back to the REEL stop point`; :56 reel `NOT_WRAPPED (reel needs the wraps on)`, until only stop|seated|tension; :76 `stop points: LAUNCH 0, NEUTRAL 40, REEL 80, WRAPPED 338`; :207-213 typical mission `mode REEL` -> `wrap` ... `reel until seated` -> `unwrap`. Fault list in section 6 has no DART_STUCK. Firmware: base_logic.h:24-26 (F_DART_STUCK), 589-591, 692-694, 709, 718-724; config.h:100-102.

Fix: Bump the contract and rewrite PROTOCOL.md sections 2-4, 6 and 8 to match base_logic.h, or to match the D22 sequence once fixed: the drum order LAUNCH/NEUTRAL/PREP/WRAP/LOCK(WRAPPED park)/REEL, wrap from NEUTRAL, mode REEL only with wraps on, unwrap to NEUTRAL, reel take-up only in NEUTRAL with until 'slip' (tidy) and 'seated' (final approach, DART_STUCK after 20 s), NOT_IN_NEUTRAL, telemetry sw keys l/r/I, and a corrected typical mission.

> Skeptic (confirmed): The defect is real. I checked the code and could not refute it.

PROTOCOL.md says it is the "contract C1, final" for base-1.0. Its git history shows it was last changed in 39badb1, before the D16 rework in 93bfb1c and a6e2394. As a result it describes a drum order that base_logic.h no longer implements:
- **Stop points:** PROTOCOL.md:76 lists "LAUNCH 0, NEUTRAL 40, REEL 80, WRAPPED 338". config.h has targetDeg = {1.5, 41, 347.5}, seven code sectors, and wrapFromDeg/wrapToDeg = 62/302, so REEL sits after the wrap sectors.
- **mode REEL:** cmdMode (base_logic.h:589-591) refuses REEL with NOT_WRA
>
> Skeptic (confirmed): I checked this against the code and it holds up. firmware/base/PROTOCOL.md, which calls itself C1 "final" for base-1.0, does not match firmware/base/grappstar_base/base_logic.h:

1. **Drum order.** PROTOCOL.md:76 gives the stop points as LAUNCH 0, NEUTRAL 40, REEL 80, WRAPPED 338. config.h has the sector edges at {8,32,50,62,302,340}, the targets at {1.5, 41, 347.5} and the wrap zone at 62..302. So REEL sits after the wrap sectors, not before them.

2. **`mode REEL`.** The doc lists only `WRAPS_ON`, `POSITION_UNKNOWN` and similar refusals. In cmdMode (base_logic.h:589-591), `mode REEL` from NE
>

## 3. Firmware cannot run the binding section 10 / D22 sequence: wrap goes straight to REEL with no WRAPPED 309 park, and the take-up tidy works only in NEUTRAL before the wrap

- File: `firmware/base/grappstar_base/base_logic.h:685`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: BASE_SPEC section 10 (D22, D31): NEUTRAL -> WRAP while slack -> park at WRAPPED 309 (cup locked, collar NEUTRAL) -> wait ANCHORED -> TIDY on the FIRM take-up through the free capstan -> REEL fly-in. In base-1.0, cmdWrap calls startDrumMove(DP_REEL) and runs through LOCK into REEL with the fly-in in one move. The stop points are only LAUNCH/NEUTRAL/REEL (no 309), so the drum cannot stop at the park. cmdReel accepts take-up-only reels just in SEC_NEUTRAL, so a tidy at the park (SEC_LOCK) is refused with NOT_WRAPPED. The firmware itself implements the pre-D22 order 'tidy before wrapping' (and its full-mission test does tidy, then wrap). A wrap after the tidy has made the anchored side taut is the case D22 says cannot work (about 360 N), so the firmware leaves no correct way to run the sequence. The return path has the same gap: unwrap goes REEL -> NEUTRAL in one move instead of unload jog -> 309 park -> unwrap.

Evidence: base_logic.h:685 comment `// wrap: from NEUTRAL through the WRAP sector ... to REEL (fly-in)`; 694 `if (!startDrumMove(DP_REEL, false)) return "POSITION_UNKNOWN";`; base_logic.h:140 `targetSec(int t) { return t == DP_LAUNCH ? SEC_LAUNCH : t == DP_NEUTRAL ? SEC_NEUTRAL : SEC_REEL; }`; config.h:102 `float targetDeg[3] = {1.5f, 41, 347.5f};` (no 309 park); base_logic.h:721-723 `bool neutral = sec == SEC_NEUTRAL && ...; if (!collarReel() && !neutral) return "NOT_WRAPPED";`; base_logic.h:714 `the tidy before wrapping (until "slip")`; test_base.cpp:701-702 tidy, then wrap. Spec: BASE_SPEC.md:150-157, DECISIONS.md D22 and D31 ('park angles WRAP_PREP 56 and WRAPPED 309').

Fix: Add a WRAPPED park stop point (targetDeg 309, sector LOCK, collar NEUTRAL). `wrap` stops there and sends WRAPPED. Allow `reel until:"slip"` (take-up only, takeupMaxDuty, takeupTripA) at the park. Allow `mode REEL` from the park to do the fly-in, with a MODE event. Make `unwrap` from REEL first do the unload jog and go to the 309 park, then run 302..62 with the 30 % take-up. Update PROTOCOL.md and test_base.cpp to this order.

> Skeptic (confirmed): I could not refute this claim. Every piece of evidence it cites is in the code. In base_logic.h, targetSec() (line 140) maps every target other than LAUNCH and NEUTRAL to SEC_REEL. config.h:102 has only three stop points, {1.5, 41, 347.5}, so there is no 309 park. cmdWrap (685-697) only runs startDrumMove(DP_REEL), so the wrap goes straight from NEUTRAL through LOCK into REEL with the fly-in. cmdUnwrap (700-713) only runs startDrumMove(DP_NEUTRAL). It has no unload jog and no stop at a WRAPPED park.

Nothing else in the code lets the tidy happen at the park. With the drum in SEC_LOCK, cmdReel 
>
> Skeptic (confirmed): I checked the claim against the code and it holds. BASE_SPEC section 10 is marked binding and so is D22. Section 10 orders the steps as NEUTRAL -> WRAP while slack -> WRAPPED park (309) -> wait ANCHORED -> TIDY on the FIRM take-up through the free capstan -> REEL fly-in. On the return it goes unload jog -> 309 park -> UNWRAP 302..62 at 30 % take-up. D31 confirms the park angle WRAPPED 309, which lies inside the LOCK sector (edges 302..340 in config.h:100).

What the code does:
(1) cmdWrap (base_logic.h:686-697) calls startDrumMove(DP_REEL). targetSec() only maps to LAUNCH, NEUTRAL or REEL, and
>

## 4. mock_base.py models a different drum than base-1.0, so mission tests pass on paths the real firmware never takes and the real paths are never tested

- File: `jetson/grappstar/comms/mock_base.py:1159`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: Same command sequence, different results. (a) From NEUTRAL, wrap: the mock replies NOT_IN_REEL, so mission.wrap() falls back to mode REEL and wraps again. The firmware accepts wrap and ends in REEL with WRAPPED and MODE events. (b) mode REEL from NEUTRAL: the mock moves; the firmware refuses NOT_WRAPPED. (c) unwrap: the mock ends at REEL; the firmware ends at NEUTRAL. (d) reel: the mock allows it only at drum_pos WRAPPED and refuses it in NEUTRAL; the firmware allows it with the collar in REEL (drum_pos 'REEL') or as a take-up-only reel in NEUTRAL (until seated/stop/slip), and raises DART_STUCK there. (e) The mock has no until:'slip'. (f) From LAUNCH, firmware wrap replies NOT_IN_NEUTRAL, a code the mock never sends. (g) Stop points differ: mock 0/40/80/338, firmware 1.5/41/347.5, with drum_pos WRAPPED only in LOCK 302-340. As a result test_mission exercises wrap->NOT_IN_REEL->mode REEL, and the final-approach fallback wraps from REEL. The unwrap-with-dart-seated hazard and the dead tidy/retrieve commands never show up in tests.

Evidence: mock_base.py:1159-1160 `if self.code_bad or self.drum_deg < DRUM_DEG["REEL"] - DRUM_WIN_DEG: return "NOT_IN_REEL"`; 1179 `self._start_drum(DRUM_DEG["REEL"], "unwrap", ...)`; 1186-1187 `if self.drum_pos() != "WRAPPED": return "NOT_WRAPPED"`; 1192 `if u not in ("stop", "seated", "tension")`; mock_base.py:47 `DRUM_DEG = {"LAUNCH": 0.0, "NEUTRAL": 40.0, "REEL": 80.0, "WRAPPED": 338.0}`. Firmware: base_logic.h:589-591 (mode REEL refused NOT_WRAPPED below LOCK), 692 `NOT_IN_NEUTRAL`, 694 wrap -> DP_REEL, 709 unwrap -> DP_NEUTRAL, 721-724 reel rules. basesim run: `mode REEL` from NEUTRAL -> NOT_WRAPPED; `wrap` -> WRAPPED + MODE REEL/REEL; `unwrap` -> MODE NEUTRAL/NEUTRAL; `reel until seated` in NEUTRAL -> ok.

Fix: Rewrite the mock's drum model from base_logic.h: the sectors LAUNCH/LN/NEUTRAL/PREP/WRAP/LOCK/REEL with config.h edgeDeg/targetDeg; wrap from sec>=NEUTRAL to REEL; mode REEL only from LOCK; unwrap to NEUTRAL; reel with collar REEL or take-up only in NEUTRAL (slip, DART_STUCK at 20 s); NOT_IN_NEUTRAL. Or run test_mission against jetson/tools/basesim, as test_base_realsim.py already does for the client.

> Skeptic (confirmed): I could not refute this claim. The mismatch it describes is real and I reproduced it.

Mock (jetson/grappstar/comms/mock_base.py):
- Line 47: the stop points are LAUNCH 0 / NEUTRAL 40 / REEL 80 / WRAPPED 338. This is the old drum order, with REEL before the wrap sectors.
- Lines 1159-1160: wrap is refused with NOT_IN_REEL below REEL.
- Line 1179: unwrap ends at REEL.
- Lines 1186-1187: reel works only at drum_pos WRAPPED.
- Line 1192: there is no until:'slip'.

Firmware (firmware/base/grappstar_base/base_logic.h, which reports base-1.0):
- cmdMode: REEL is refused with NOT_WRAPPED when sec < S
>
> Skeptic (confirmed): I could not refute this. The divergence is real, and I checked it against the code rather than the docs.

What the mock does (jetson/grappstar/comms/mock_base.py):
- Stop points (line 47): DRUM_DEG = LAUNCH 0, NEUTRAL 40, REEL 80, WRAPPED 338, with the wrap sectors between REEL and WRAPPED.
- _cmd_wrap (1150-1163) replies NOT_IN_REEL below REEL and drives to WRAPPED.
- _cmd_unwrap (1165-1181) goes back to REEL.
- _cmd_reel (1183-1192) needs drum_pos WRAPPED, otherwise NOT_WRAPPED, and accepts until stop/seated/tension only (no 'slip').

What the firmware does (firmware/base/grappstar_base/base
>

## 5. FINAL_APPROACH sends a command the firmware does not have, so every run seats the dart on the capstan and then unwraps from REEL with the dart seated (the D16 hazard)

- File: `jetson/grappstar/mission.py:924`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: Normal run. RETRIEVE stops with about 1.5 m left, UNWRAP takes the drum back to NEUTRAL, then FINAL_APPROACH sends {"cmd":"retrieve"}. base-1.0 replies UNKNOWN_CMD (I checked this in basesim built from firmware/base). The fallback then wraps again (NEUTRAL to REEL, about 27 s, dragging the dart about 0.5 m to lay the 4 turns), reels until seated on the capstan, and calls unwrap() with the dart seated. Firmware unwrap from REEL always starts with the stage-5 window take-up: the wiper runs FORWARD on the engaged capstan for 60 deg (about 43 mm of line), for up to 3 s, at 20 % duty. That is below the 30 % duty where stall detection starts, and the trip is only jogTripA 3 A. So the capstan winds against a dart that is already seated in the cradle. D16 forbids exactly this: 'pull the seated dart through the remaining wraps (up to about 157 N, invisible to the current sense)'. The same seated-then-unwrap sequence also happens when retrieve() seats the dart on the capstan: line_m <= approach_m+0.3 (a short shot or reshoot) goes to until:'seated' at line 910, and SW_SEATED closing during the capstan reel is handled at lines 888-891. bring_home() then runs UNWRAP from REEL.

Evidence: mission.py:924 `self.base.cmd("retrieve")`; 929-935 `self.wrap()` ... `self.base.reel(..., until="seated")` ... `self.unwrap()`; mission.py:910 `self.base.reel(self.cfg.retrieve_duty, max_n=self.cfg.retrieve_limit_n, until="seated")`. Firmware base_logic.h:890-911 has no "retrieve" in the dispatch (basesim reply: {"id":2,"ok":false,"err":"UNKNOWN_CMD"}). base_logic.h:454 `if (sec == SEC_REEL) { mvStage = 5; wiper(c.reelDir, c.flyDuty, c.jogTripA); return true; }` and 463-466 (stage 5 runs until dogWindowDeg 60 or 3000 ms). config.h:113 `dogWindowDeg = 60`, config.h:117 `flyDuty = 51; // 20 percent`. The firmware's intended final approach is take-up only in NEUTRAL: base_logic.h:713-714 and 1177-1178 (DART_STUCK after 20 s), and test_base.cpp:716 `reel ... until seated` after the unwrap.

Fix: In final_approach(), drop the 'retrieve' command and the wrap fallback. Check drum_pos == 'NEUTRAL' and wraps == 0 (otherwise unwrap first), then call `self.base.reel(min(self.cfg.retrieve_duty, 0.4), max_n=self.cfg.retrieve_limit_n, until="seated")` and `wait_evt(("SEATED",), 25.0, m)`. DART_STUCK arrives as a FAULT event, which raises BaseError: add 'DART_STUCK' to the fault list in _do. In retrieve(), never use until='seated' on the capstan. With line unknown or <= approach_m+0.3, reel until:'stop' only while line_m > approach_m (or for a timed short pull), and leave the seating to FINAL_APPROACH. If SW_SEATED closes while wrapped, stop and ask the operator: no automatic unwrap with the dart seated.

> Skeptic (confirmed): I could not refute this. I checked each step against the code.

(1) The firmware has no retrieve command. The dispatch in firmware/base/grappstar_base/base_logic.h:890-915 handles ping, status, stop, estop, clear, mode, aim, arm, fire, cock, wrap, unwrap, reel, jog, cal, set and get; anything else gets err="UNKNOWN_CMD". jetson/docs/C1_NOTES.md:78 also says "No `tidy` or `retrieve` command in the code yet." The only "retrieve" commands in the repo are in the old kedge/ prototype, not firmware/base.

(2) So on every normal run, final_approach() (mission.py:917-935) falls into its UNKNOWN_CMD fa
>
> Skeptic (confirmed): I couldn't refute this. I checked each step against the code.
(1) The firmware has no "retrieve" command. The dispatch in firmware/base/grappstar_base/base_logic.h (around lines 890-911) has ping, status, stop, estop, clear, mode, aim, arm, fire, cock, wrap, unwrap, reel, jog, cal, set and get, and anything else gets err="UNKNOWN_CMD". No other "retrieve" appears in the firmware. In the Jetson code the string appears only in mission.py, at lines 22 and 24 (module docstring), 918 (method docstring) and 924 (the actual send).
(2) The normal run always takes the fallback. bring_home() (mission.py
>

## 6. PROTOCOL.md section 3 (and mock_base.py, built to it) describe a different drum model than the firmware for drum_pos, mode and drum_deg

- File: `firmware/base/PROTOCOL.md:76`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: PROTOCOL.md (labelled 'final') says: REEL is the stop point at 80 deg, before the wraps; WRAPPED is the resting stop point at 338 deg after `wrap`; mode stays REEL while wrapping. The firmware (D16) does this instead: LAUNCH 1.5, NEUTRAL 41, REEL 347.5 (wraps already on, wraps=4). drum_pos WRAPPED is only the transient SEC_LOCK. While the wraps are being laid (SEC_WRAP) mode is NEUTRAL, not REEL. mock_base.py implements the PROTOCOL model (WRAPPED 338 as the final pose, REEL 80 before the wraps), so the Jetson tests pass against telemetry the real base never sends. The is_wrapped bug above is one direct result: Jetson code written to this table treats WRAPPED as the resting wrapped state.

Evidence: PROTOCOL.md:75 `REEL (also while wrapping or wrapped: the collar stays in REEL)`; PROTOCOL.md:76 `stop points: LAUNCH 0, NEUTRAL 40, REEL 80, WRAPPED 338`; config.h:102 `float targetDeg[3] = {1.5f, 41, 347.5f};   // stop points (dwell centres): LAUNCH, NEUTRAL, REEL`; config.h:23 `enum Sector { SEC_LAUNCH, SEC_LN, SEC_NEUTRAL, SEC_PREP, SEC_WRAP, SEC_LOCK, SEC_REEL, ...`; base_logic.h:147 `case SEC_LOCK: return DP_WRAPPED;`; base_logic.h:158 NEUTRAL is reported for `sec == SEC_NEUTRAL || sec == SEC_PREP || sec == SEC_WRAP`; mock_base.py:47 `DRUM_DEG = {"LAUNCH": 0.0, "NEUTRAL": 40.0, "REEL": 80.0, "WRAPPED": 338.0}`.

Fix: Rewrite PROTOCOL.md section 3 to match the firmware: stop points LAUNCH 1.5, NEUTRAL 41, REEL 347.5; REEL means wraps on and collar engaged; WRAPPED means the collar is shifting into REEL (SEC_LOCK, transient, not a stop point); BETWEEN covers LN/PREP/WRAP; mode is NEUTRAL while laying wraps and NONE in LOCK. Then update mock_base.py (DRUM_DEG, DRUM_ORDER, drum_pos(), mode) to the same model.

> Skeptic (confirmed): I checked the code and the claim holds. The cited paths are under firmware/base/grappstar_base/.

What PROTOCOL.md says. It is headed "contract C1, final", and its section 3 says the following:
- mode is REEL while wrapping or wrapped.
- The stop points are LAUNCH 0, NEUTRAL 40, REEL 80 and WRAPPED 338.
- wraps is 0 up to REEL and 4 at WRAPPED.

What the firmware does (D16):
- config.h:23 orders the sectors LAUNCH, LN, NEUTRAL, PREP, WRAP, LOCK, REEL.
- config.h:102 sets targetDeg to {1.5, 41, 347.5}, so REEL is the last stop point, after the wraps.
- base_logic.h drumPos() maps SEC_LOCK to DP
>
> Skeptic (confirmed): I couldn't refute this. The claim holds up against the code. PROTOCOL.md section 3 (lines 75-76, 87) still describes the old drum model: stop points LAUNCH 0 / NEUTRAL 40 / REEL 80 / WRAPPED 338, mode stays REEL while wrapping, and wraps is 0 up to REEL and 4 at WRAPPED. The firmware follows D16/D27 instead:
- firmware/base/grappstar_base/config.h:23 has the sector order LAUNCH, LN, NEUTRAL, PREP, WRAP, LOCK, REEL.
- config.h:100-104 puts the stop points at {1.5, 41, 347.5}, lays the wraps between 62 and 302 deg, and sets wrapsFull=4.
- base_logic.h:147 maps only SEC_LOCK (340..~347) to DP_WRA
>

## 7. A fault mid-cock (or mid-fire from rest) leaves the spring drawn: the mission retries cock into LOADED_UNKNOWN forever, never offers a decock, and suppresses the 'still cocked' warning

- File: `jetson/grappstar/mission.py:282`
- Severity: medium (claimed high); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: COCK runs and the wiper trips (OVERCURRENT at cock_trip 6.5 A, BROWNOUT or STALL) halfway through the draw. The firmware sets the phase to UNKNOWN with the carriage drawn (SW_HOME open), and the drum HOLDS in LAUNCH. _do offers Retry/Skip/Abort. Retry clears the fault and sends cock, which the firmware refuses with LOADED_UNKNOWN every time. Skip leads to FIRE, where ensure_launch/cock is skipped because tel cock is null, and arm is refused with NOT_COCKED. Abort is left: the finally block warns only if shot['cocked'] is set, and that is set only after a COCKED event, so the operator gets ABORTED with no warning that the spring is drawn. The section 10 de-cock recovery (cal decock) is never offered. The same thing happens for a fault during a fire from REST: fire_sent is set, so the warning condition `not fire_sent` hides it too.

Evidence: mission.py:282 `if self.state in ("ABORTED", "FAILED") and self.shot.get("cocked") and not self.shot.get("fire_sent"):`; mission.py:471-473 `self.base.cmd("cock")` / `self.wait_evt("COCKED", ...)` / `self.shot["cocked"] = True`; mission.py:565 `if t.get("cock") is not None and t.get("cock") < 0.999:` (null when the phase is unknown); mission.py:318-335 (Retry just clears and repeats). Firmware: base_logic.h:218-220 (a fault during COCK/FIRE sets `phase = PH_UNKNOWN`), 245-246 (drum held while the launch train is not at rest), 644 `if (phase == PH_UNKNOWN && !closed(SW_HOME)) return "LOADED_UNKNOWN";`, 1003-1004 (tel cock null for UNKNOWN), 806-822 cal decock.

Fix: Base the decision on telemetry, not shot flags. In _do for COCK/FIRE errors (and in the finally block), read tel: if mode == 'LAUNCH' and SW_HOME is open (`not sw_closed(t, "H")`) or tel cock is null/>0, add a 'Decock' choice. That choice calls `self.base.decock()` and waits `wait_evt("RESET", 60, m, pred=lambda e: e.get("how") == "decock")`, then re-aims, since the aim becomes unknown. Treat a LOADED_UNKNOWN or NOT_COCKED refusal the same way. In finally, warn 'spring may be drawn: Decock' whenever that telemetry condition holds, whatever shot['cocked'] or fire_sent say.

> Skeptic (confirmed): The defect is real, and I could not refute its core. The claimed severity is a bit high because the page already offers a way to recover.

What the code confirms:
- **Fault handling in the firmware.** In base_logic.h `raise()` (around line 218), a fault while `state == S_COCK` or `S_FIRE` (or in `PH_COCKING` / `PH_FIRING`) sets `phase = PH_UNKNOWN`. `safeToNeutral()` then returns "launch train not at rest" whenever the train may be loaded, so the drum holds in LAUNCH.
- **Retry loops.** `cmdCock()` returns `LOADED_UNKNOWN` whenever `phase == PH_UNKNOWN && !closed(SW_HOME)`. `telBuild` gives `c
>
> Skeptic (confirmed): I read the code on w/review and the claim holds. I did not run a simulation.

Firmware (firmware/base/grappstar_base/base_logic.h):
- raise() (218-221): a fault while in S_COCK, S_FIRE or PH_COCKING/PH_FIRING sets phase = PH_UNKNOWN.
- safeToNeutral() keeps the drum held when the launch train is not at rest.
- cmdCock (644): with PH_UNKNOWN and SW_HOME open it returns LOADED_UNKNOWN. When SW_HOME reads home but phaseHint says the spring is drawn, it also returns LOADED_UNKNOWN.
- fireCheck (662): it accepts only COCKED with SW_HOME open, or REST with SW_HOME closed. Anything else gives NOT_COC
>

## 8. Mission sends `tidy` and `retrieve`, which no firmware command handles. The tidy is silently skipped and the final approach falls back to a capstan wrap/reel/unwrap cycle

- File: `jetson/grappstar/mission.py:492`
- Severity: medium (claimed high); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: Normal mission against the real firmware (base-1.0, D16 drum order). In step TIDY, mission.tidy() sends {"cmd":"tidy"}. The firmware dispatch has no such command and replies {"ok":false,"err":"UNKNOWN_CMD"}, so tidy() logs 'base has no tidy command: skipped' and returns. The slack is never wound onto the storage reel before WRAP, but D16/D22 assume the anchored side is taut after the tidy. In FINAL_APPROACH, {"cmd":"retrieve"} also gets UNKNOWN_CMD. The fallback then lays the wraps again (NEUTRAL->REEL, about 24 s), reels the last metre on the capstan until seated, and unwraps again (another 24 s). D16 says the final approach runs on the NEUTRAL take-up. The firmware implements both steps as `reel` in NEUTRAL: until:"slip" for the tidy (ends with a LIMIT event, why "tidy done (take-up slipping)") and until:"seated" for the take-up approach (DART_STUCK fault after 20 s). The Jetson can't use either: BaseClient.reel() raises ValueError for until="slip". I reproduced this with the real-firmware simulator (jetson/tools/basesim built to /tmp/claude-0/review-scratch-A/basesim): after mode NEUTRAL, `tidy` -> UNKNOWN_CMD, `retrieve` -> UNKNOWN_CMD, `reel` duty 0.5 until slip -> ok, then LIMIT "tidy done (take-up slipping)". The mock base never shows this because it has neither command and still models the pre-D16 drum (wrap REEL->WRAPPED, reel only when WRAPPED).

Evidence: mission.py:492 `self.base.cmd("tidy", timeout=2.0)` / :494 `if e.err == "UNKNOWN_CMD": ... skipped`; mission.py:924 `self.base.cmd("retrieve")`, fallback :929 `self.wrap()` + capstan reel until seated + unwrap; base.py:469 `if until not in ("seated", "tension", "stop"): raise ValueError`; firmware base_logic.h:870-916 dispatch has no tidy/retrieve (`else err = "UNKNOWN_CMD";`); base_logic.h:718 `... !strcmp(u, "slip") ? U_SLIP : -1;`, :722-725 reel accepted in NEUTRAL (takeup only), :1179-1182 slip end emits LIMIT why "tidy done (take-up slipping)"; mock_base.py:1186 `if self.drum_pos() != "WRAPPED": return "NOT_WRAPPED"`

Fix: In comms/base.py:469 accept "slip" (`until not in ("seated","tension","stop","slip")`). Rewrite mission.tidy() to call `self.base.reel(duty, until="slip")` with the drum in NEUTRAL and wait for LIMIT (holding false). Rewrite final_approach() to call `self.base.reel(duty<=0.4, max_n=..., until="seated")` in NEUTRAL after unwrap and wait for SEATED (treat FAULT DART_STUCK as failure). Drop the `tidy`/`retrieve` commands and the wrap/capstan fallback. Update mock_base to the D16 model (wrap NEUTRAL->REEL, unwrap back to NEUTRAL, reel in NEUTRAL with take-up only and until slip/seated) so the tests exercise the real contract.

> Skeptic (confirmed): I checked the claim against the code and it holds. The firmware dispatch in firmware/base/grappstar_base/base_logic.h (lines 885-915) handles ping, status, stop, estop, clear, mode, aim, arm, fire, cock, wrap, unwrap, reel, jog, cal, set and get. Anything else gets `else err = "UNKNOWN_CMD"`, so `tidy` and `retrieve` are both refused.

**What the mission does with those refusals (jetson/grappstar/mission.py)**
- `tidy()` (lines 488-499) catches UNKNOWN_CMD, logs "skipped" and returns. The TIDY step never runs against the real firmware.
- `final_approach()` (lines 917-941) catches UNKNOWN_CMD a
>
> Skeptic (confirmed): I checked the code and the claim holds. The firmware dispatch, `handleLine` in firmware/base/grappstar_base/base_logic.h around lines 888-915, handles only ping, status, stop, estop, clear, mode, aim, arm, fire, cock, wrap, unwrap, reel, jog, cal, set and get. Anything else gets `else err = "UNKNOWN_CMD"`, and the firmware has no "tidy" or "retrieve" command anywhere.

**Tidy.** jetson/grappstar/mission.py:488-499 sends `cmd("tidy")`. On UNKNOWN_CMD it logs "skipped" and returns, so the TIDY step (called at :370) does nothing against the real firmware.

**Final approach.** mission.py:917-940 s
>

## 9. TIDY step is dead code against base-1.0 ('tidy' is UNKNOWN_CMD and base.reel rejects until='slip'), and the mission order ANCHOR->TIDY->WRAP contradicts the binding WRAP->ANCHOR->TIDY->REEL order

- File: `jetson/grappstar/mission.py:492`
- Severity: medium (claimed high); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: Today, in every run, tidy() sends {"cmd":"tidy"}. The firmware replies UNKNOWN_CMD and the step logs 'skipped', so the slack is never tidied. The firmware's tidy is `reel until:"slip"` (take-up only, in NEUTRAL), but BaseClient.reel() raises ValueError for until='slip', so the mission cannot issue it either. The order is also wrong. _sequence runs ANCHOR, then TIDY, then WRAP. BASE_SPEC section 10 and D22 require WRAP at once while the line is slack, before anchoring. Once a working tidy is wired in at its current place, it makes the anchored dart side taut and only then starts the wrap. D22 says laying a turn then has to draw line through the blocked capstan payout (about 360 N at the ring), so the N20 drum stops in the WRAP sector. That gives DRUM_TIMEOUT with wraps partly on, the firmware holds the drum (DRUM_HELD), and the mission is lost.

Evidence: mission.py:363-371 `r = self._do("ANCHOR", self.anchor)` ... `self._do("TIDY", self.tidy)` then `self._do("WRAP", self.wrap)`; mission.py:492-496 `self.base.cmd("tidy", timeout=2.0)` / `if e.err == "UNKNOWN_CMD": ... return`; base.py:469 `if until not in ("seated", "tension", "stop"): raise ValueError`. Firmware: base_logic.h:718 `!strcmp(u, "slip") ? U_SLIP`, 1179-1182 (tidy ends with LIMIT why 'tidy done (take-up slipping)'), basesim reply to tidy: UNKNOWN_CMD. BASE_SPEC.md:150-154 '-> WRAP at once while the line is slack ... -> WRAPPED park (309) -> wait ANCHORED -> TIDY'; DECISIONS.md D22.

Fix: Reorder _sequence for a GO verdict: BASE_NEUTRAL -> WRAP (to the 309 park, once the firmware supports it, see the fw-base finding) -> ANCHOR -> TIDY -> mode REEL (fly-in) -> CHECK -> TOW. Implement tidy() as `self.base.cmd("reel", duty=0.2, max_n=self.cfg.check_n, until="slip")` and wait for a LIMIT event whose why starts with 'tidy'; an ABORTED 'timeout' is a failure. Add 'slip' to the allowed until values in BaseClient.reel(). Never run the tidy before the wrap.

> Skeptic (confirmed): The facts check out, but the claimed severity is too high. (1) Dead code is real. mission.py:492 sends cmd "tidy". The firmware command dispatcher falls through to `err = "UNKNOWN_CMD"` (base_logic.h:915), and it has no "tidy" verb; its tidy is `reel until:"slip"` (base_logic.h:718, 1179-1183; test_base.cpp:370-373). mock_base.py has no tidy or slip handler either, so it returns UNKNOWN_CMD too. tidy() logs "skipped" in every run. BaseClient.reel() (comms/base.py:469) rejects until='slip', so the mission cannot issue the real tidy either. The C1 contract in WORKPLAN line 91 also lists only sea
>
> Skeptic (confirmed): I checked the code and the factual parts of the claim hold.
(1) The firmware's command dispatch in base_logic.h, around lines 890-915, accepts ping, status, stop, estop, clear, mode, aim, arm, fire, cock, wrap, unwrap, reel, jog, cal, set and get. Anything else gets err="UNKNOWN_CMD". So when mission.py:492 sends {"cmd":"tidy"}, the step always takes the UNKNOWN_CMD branch at lines 494-496, logs 'skipped' and returns. In base-1.0 the TIDY step is dead code.
(2) The firmware's real tidy is reel until:"slip" (base_logic.h:1179-1183, test_base.cpp:370-373). BaseClient.reel() in comms/base.py:469 
>

## 10. is_wrapped() reads firmware drum_pos WRAPPED (a transient sector) or a wraps estimate of 4 as 'wrapped and engaged', so the mission skips the wrap and the reel is then refused

- File: `jetson/grappstar/mission.py:543`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: On the real firmware, drum_pos "WRAPPED" means SEC_LOCK, the sector where the collar is still ramping from NEUTRAL to REEL. It is not the resting wrapped state. Also, wrapsAt() already reports wraps=4 in SEC_LOCK, and in the last ~4% of SEC_WRAP. Failure path: during the WRAP step the drum stalls in SEC_LOCK (DRUM_TIMEOUT 'drum not turning'). Because it is a drum fault, safeToNeutral() makes the drum hold. The operator clears the fault and chooses Retry WRAP. mission.wrap() sees is_wrapped(tel)==True (drum_pos WRAPPED, wraps 4) and returns without sending wrap. CHECK/TOW then call ensure_reel(), which does nothing for the same reason, and send reel. The firmware refuses with NOT_WRAPPED because collarReel() is false in SEC_LOCK. Every Retry repeats this, so the mission cannot get past CHECK/TOW without manual jogging.

Evidence: mission.py:542-543 `def is_wrapped(t: dict) -> bool:\n        return t.get("drum_pos") == "WRAPPED" or (t.get("wraps") or 0) >= 3.5`; mission.py:758-759 `if self.is_wrapped(self.tel()):\n            return`; base_logic.h:147 `case SEC_LOCK: return DP_WRAPPED;`; base_logic.h:162 `if (s == SEC_LOCK || s == SEC_REEL) return c.wrapsFull;`; base_logic.h:723 `if (!collarReel() && !neutral) return "NOT_WRAPPED";`; base_logic.h:156-160 modeName() returns "REEL" only when collarReel(), and "NONE" in SEC_LOCK.

Fix: Require the collar to be engaged as well as the wraps: `return t.get("mode") == "REEL" and (t.get("wraps") or 0) >= 3.5`. On the firmware, mode is REEL only in SEC_REEL with collarReel(). In the mock/PROTOCOL model, mode stays REEL while wrapped, so the check holds for both. mission.wrap() then re-sends wrap from SEC_LOCK/SEC_WRAP, which cmdWrap accepts (sec >= SEC_NEUTRAL).

> Skeptic (confirmed): I could not refute this claim. Every step of the failure path is in the code.

1. `base_logic.h:147`: drumPos() returns DP_WRAPPED for SEC_LOCK. telBuild (around line 1000) sends it as drum_pos. `base_logic.h:162`: wrapsAt() returns wrapsFull for SEC_LOCK and for SEC_REEL. modeName() (156-160) returns "REEL" only when collarReel() is true, and that needs sec==SEC_REEL with the dogs confirmed. In SEC_LOCK it returns "NONE".

2. The drum can stop and hold there in the middle of a wrap. cmdWrap starts startDrumMove(DP_REEL). In tickDrumMove, stage 0 can raise F_DRUM_TIMEOUT while sec==SEC_LOCK ("
>
> Skeptic (confirmed): I could not refute this. I checked each step of the failure path in the code.

1. config.h:28 maps DP_WRAPPED to the string "WRAPPED". base_logic.h:147 maps SEC_LOCK to DP_WRAPPED. wrapsAt() (base_logic.h:162, wrapsFull=4 in config.h:104) returns 4 in both SEC_LOCK and SEC_REEL. So in SEC_LOCK, and in SEC_REEL with the collar open, telemetry reads drum_pos WRAPPED or wraps 4 even though collarReel() is false.

2. A drum fault can leave the drum in exactly those states. tickDrumMove has three faults that do it:
   - "drum not turning (no code edge)" (line 517) while in SEC_LOCK;
   - "did not r
>

## 11. `cal decock` with the carriage home and phase UNKNOWN replies ok but sends no RESET event, although PROTOCOL promises an immediate RESET

- File: `firmware/base/grappstar_base/base_logic.h:817`
- Severity: low (claimed low); unverified (low)
- Owner: fw-base

Scenario: After a power loss mid-cycle the phase is UNKNOWN with SW_HOME closed. The operator presses Decock (app.py:332 sends cal decock and says 'wait for the RESET event'). The firmware returns ok with no event and no motion, so no RESET ever arrives. Anything waiting for RESET {how: decock}, as PROTOCOL section 5 specifies ('RESET event ... immediately if the carriage is already home'), times out. Only the phase REST branch (line 816) emits RESET.

Evidence: base_logic.h:816 `if (phase == PH_REST) { ... evBegin(e, "RESET"); ... return nullptr; }`; :817 `if (phase == PH_UNKNOWN && !(phaseHint > ...)) return nullptr;   // carriage home: "cock" finds the angle`; PROTOCOL.md:130 `RESET event with how:"decock" (immediately if the carriage is already home)`; app.py:333 `"decocking: wait for the RESET event (carriage home)"`

Fix: Either emit RESET in the UNKNOWN-and-home branch too, with ok:true, how:"decock" and a field such as phase:"UNKNOWN" (do not set phase REST), or change PROTOCOL.md:130 to say that with the phase UNKNOWN and the carriage home the reply is ok and no event follows (run cock to re-zero). Either way, make app.py's message match.


## 12. The firmware raises fault code DART_STUCK, which is not in the contract's fault list or in the Jetson FAULTS tuple

- File: `firmware/base/grappstar_base/base_logic.h:1178`
- Severity: low (claimed low); unverified (low)
- Owner: fw-base

Scenario: `reel until seated` in NEUTRAL (take-up only) that gets no SW_SEATED within 20 s latches fault DART_STUCK. The FAULT event carries code DART_STUCK and telemetry shows fault "DART_STUCK". PROTOCOL.md section 6 calls its list of 8 codes complete, and base.py FAULTS / mock_base.py FAULTS have the same 8. Any Jetson code that maps or validates fault codes against the contract treats DART_STUCK as unknown, and the mock can never produce it.

Evidence: base_logic.h:24-25 enum and FAULT_NAME include `"DART_STUCK"`; base_logic.h:1178 `raise(F_DART_STUCK, "final approach: no SW_SEATED in 20 s");`; PROTOCOL.md:159-160 `Fault codes: STALL, OVERCURRENT, DRUM_TIMEOUT, DRUM_SENSOR, ANGLE_SENSOR, SWITCH_DISAGREE, BROWNOUT, BAD_STATE.`; base.py:45-46 FAULTS has no DART_STUCK.

Fix: Add DART_STUCK to PROTOCOL.md section 6, with cause 'reel until seated on the take-up (NEUTRAL) without SW_SEATED for 20 s'. Add it to FAULTS in base.py and mock_base.py.


## 13. Telemetry sw keys differ: the firmware sends l, r and I; PROTOCOL.md, base.py and the mock use a single D

- File: `firmware/base/grappstar_base/config.h:17`
- Severity: low (claimed low); unverified (low)
- Owner: fw-base

Scenario: With the collar engaged in LAUNCH, the real firmware sends sw {"a":1,"l":1,...}, and in REEL it sends "r":1. SW_INDEX would appear as "I". The contract defines only "D" (SW_DOG, dogs tip to tip), which the firmware never sends. On real hardware tel_contact(tel, "SW_DOG") is therefore always False. l, r and I are unknown keys to the Jetson. The assertion in jetson/tests/test_comms_base.py:70 (`set(t["sw"]) <= set(SW_KEYS)`) passes only because the tests run against the mock; real telemetry in LAUNCH fails it.

Evidence: config.h:17 `static const char* const SW_TEL[SW_COUNT] = {"a", "b", "c", "l", "r", "S", "H", "L", "W", "I"};`; PROTOCOL.md:86 ``D` `SW_DOG` (dogs tip to tip)`; base.py:54-55 `SW_KEYS = {..., "D": "SW_DOG", ...}`; mock_base.py:379 `out["D"] = 1`.

Fix: Document l = SW_DOG_L (collar engaged in LAUNCH), r = SW_DOG_R (collar engaged in REEL) and I = SW_INDEX in PROTOCOL.md section 3, and remove D. In base.py SW_KEYS and mock_base.py SW_KEYS, replace "D": "SW_DOG" with "l": "SW_DOG_L", "r": "SW_DOG_R" and "I": "SW_INDEX". Make the mock send l or r when its collar is engaged.


## 14. Mock base answers NOT_ARMED for an expired token where the firmware answers TOKEN_EXPIRED, and a test locks the mock behaviour in

- File: `jetson/grappstar/comms/mock_base.py:493`
- Severity: low (claimed low); unverified (low)
- Owner: jetson

Scenario: arm, wait more than 3 s, then fire(token). Firmware: the watch() tick sets tokenExpired=true, and cmdFire returns "TOKEN_EXPIRED" (the error PROTOCOL documents). Mock: _substep clears self.token on expiry, so _cmd_fire hits `if self.token is None: return "NOT_ARMED"` and its own TOKEN_EXPIRED branch can't be reached. test_comms_base.py:180 asserts NOT_ARMED. Jetson code or an operator message that keys on TOKEN_EXPIRED (for example a re-arm prompt) would pass against the mock and behave differently on the base.

Evidence: mock_base.py:493-494 `if self.token is not None and self.sim_t - self.token_t > TOKEN_S: self.token = None`; mock_base.py:1130-1134 `if self.token is None: return "NOT_ARMED"` before `TOKEN_EXPIRED`; firmware base_logic.h:410 `if (tokenLive && now - tokenT > c.armMs) { tokenLive = false; tokenExpired = true; }`, :675 `if (!tokenLive) return tokenExpired ? "TOKEN_EXPIRED" : "NOT_ARMED";`; tests/test_comms_base.py:180 `assert err_of(c.fire, tok) == "NOT_ARMED"           # expired`

Fix: In the mock, keep an expired flag like the firmware does: on expiry set `self.token=None; self.token_expired=True`. In _cmd_fire return "TOKEN_EXPIRED" when token is None and token_expired, and clear the flag on any fire and on arm. Change test_comms_base.py:180 to expect TOKEN_EXPIRED.


## 15. The operator page never shows 'Dart seated': it reads base.sw.seated, but telemetry uses sw.S

- File: `jetson/grappstar/ui/static/app.js:22`
- Severity: low (claimed low); unverified (low)
- Owner: jetson

Scenario: app.py:188 passes the raw telemetry to the page as status.base. fillTable then calls pick(base, "seated"), which looks for base.seated or base.sw.seated. The firmware (and the mock) send the seated contact as sw.S. So the 'Dart seated' row always shows '-', and the operator cannot see from the page whether the dart is in the cradle before a shot or after a retrieve.

Evidence: app.js:19 `// [key, label]; "seated" is read from base.sw.seated`; app.js:22 `["seated", "Dart seated"]`; app.js:55 `for (const sub of ["sw", ...]) { ... key in obj[sub] ...}`; base_logic.h telBuild writes `o.integer(SW_TEL[i], 1)`, where SW_TEL[SW_SEATED] is "S" (config.h:17); PROTOCOL.md:86 ``S` `SW_SEATED``.

Fix: Add seated in app.py status(): `"base": dict(tel, link=self.base.link(), seated=tel_seated(tel))`, using tel_seated from comms.base. Alternatively, in app.js read `!!(base.sw && base.sw.S)` for the seated row.

