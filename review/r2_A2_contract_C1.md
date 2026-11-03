# Round 2 A2: contract C1 against the D35 sequence (BASE_SPEC 10.1)

Reviewed at base-v1 commit ba121b9. Method: independent finder subagents, one per lens; every finding of
medium severity or above was checked by two skeptic subagents told to refute it; kept only if fewer than
two refuted it. Low findings are listed unverified. Severity is the finder's, lowered when both skeptics
agreed it is real but rated it lower.

Confirmed: 18. Refuted and dropped: 7.

STATUS: base-v1 moved on while this ran. Each finding was re-checked against
base-v1 472bb65 by one checker subagent each; see the Now column.

| # | Sev | File:line | Owner | Finding | Now (472bb65) |
|---|---|---|---|---|---|
| 1 | high | `firmware/base/PROTOCOL.md:82` | fw-base | PROTOCOL.md (C1) still describes the pre-D35 telemetry and events: wrap is followed by MODE REEL, WRAPPED is a transient ramp with mode NONE, and there is no tidy/retrieve/decock command and no TIDIED event (R1 #1/#2/#6 persist in new form) | fixed |
| 2 | high | `firmware/base/grappstar_base/base_logic.h:986` | fw-base | Firmware `retrieve` ignores max_s and faults DART_STUCK after a fixed 20 s, so a normal final approach from 1.5 m or more trips DART_STUCK | fixed |
| 3 | high | `jetson/grappstar/mission.py:388` | jetson | Mission tidies before wrapping (ANCHOR -> TIDY -> WRAP). With the firmware tidy now working, every GO run tightens the anchored line and only then lays the wraps (R1-A #9 persists and is worse) | fixed |
| 4 | high | `jetson/grappstar/mission.py:566` | jetson | Mission never sends `mode REEL` after `wrap` (D35 step 12), so the CHECK pull is refused NOT_WRAPPED at the 314 park and every GO run stops at CHECK | fixed |
| 5 | high | `jetson/grappstar/mission.py:566` | jetson | Mission reads the D35 WRAPPED park (drum_pos WRAPPED, mode NEUTRAL, wraps 7) as an engaged capstan, so it never sends `mode REEL` and every CHECK/TOW reel is refused (R1 #10 still open, new form) | partly fixed |
| 6 | medium | `firmware/base/PROTOCOL.md:82` | fw-base | PROTOCOL.md was not updated for D35: no tidy/retrieve/decock commands, no TIDIED event, wrap still documented as ending in REEL with MODE, and the typical mission is the old order (R1-A #1/#2 persist) | fixed |
| 7 | medium | `firmware/base/grappstar_base/base_logic.h:1269` | fw-base | `retrieve` ignores max_s and DART_STUCK fires after a fixed 20 s, not the D35 default of 20 + line_m/0.08 s | fixed |
| 8 | medium | `jetson/grappstar/comms/mock_base.py:47` | jetson | mock_base.py still sends the old drum telemetry (WRAPPED 338 with mode REEL accepting reel tension, 4 wraps, no tidy/retrieve/TIDIED/DART_STUCK), so mock mission tests pass on output the firmware never produces (R1 #4/#6 still open) | fixed |
| 9 | medium | `jetson/grappstar/comms/mock_base.py:1173` | jetson | mock_base.py still models the pre-D16 drum and has no tidy/retrieve/decock, so the mission tests never run the D35 sequence (R1-A #4 persists) | fixed |
| 10 | medium | `jetson/grappstar/mission.py:566` | jetson | is_wrapped() counts 4 of 7 wraps as wrapped, so after a D27 hold mid-wrap the Retry skips the wrap and CHECK is refused forever; no unwrap option is offered (R1 #10 persists) | still open |
| 11 | medium | `jetson/grappstar/mission.py:938` | jetson | Mission still seats on the capstan and sends `retrieve` without duty/max_s (R1-A #5 partly persists) | partly fixed |
| 12 | low | `base/spec/BASE_SPEC.md:195` | coordinator | BASE_SPEC 10.1 recovery says a fault in REEL/N_TO_R holds the drum with DRUM_HELD, but D31 and the firmware drive to the 314 park and send MODE ok | fixed |
| 13 | low | `firmware/base/grappstar_base/base_logic.h:986` | fw-base | Firmware `retrieve` is accepted in REEL and seats the dart on the capstan, which 10.1 step 17 forbids ('NEUTRAL only. Never seat on the capstan') | fixed |
| 14 | low | `jetson/grappstar/comms/base.py:54` | jetson | base.py and mock_base.py SW_KEYS still use D (SW_DOG) and W=SW_WRAP_HOME; the firmware sends l, r, n, G and I (R1 #13 still open on the Jetson side) | partly fixed |
| 15 | low | `jetson/grappstar/comms/base.py:469` | jetson | BaseClient.reel() rejects until='slip', which 10.1 step 11 keeps as the tidy alternative | partly fixed |
| 16 | low | `jetson/grappstar/comms/mock_base.py:507` | jetson | Mock answers NOT_ARMED for an expired token where the firmware answers TOKEN_EXPIRED (R1-A #14 persists) | fixed |
| 17 | low | `jetson/grappstar/ui/static/app.js:22` | jetson | Operator page 'Dart seated' row always shows '-': it reads base.seated / base.sw.seated, but telemetry sends sw.S (R1 #15 still open) | fixed |
| 18 | low | `systest/harness.py:626` | coordinator | systest c_tidy passes when TIDY runs before WRAP, so the system test cannot catch the D35 order violation | partly fixed |

## 1. PROTOCOL.md (C1) still describes the pre-D35 telemetry and events: wrap is followed by MODE REEL, WRAPPED is a transient ramp with mode NONE, and there is no tidy/retrieve/decock command and no TIDIED event (R1 #1/#2/#6 persist in new form)

- Now (472bb65): **fixed** at `firmware/base/PROTOCOL.md:88`. PROTOCOL.md now matches the firmware on all the points the finding raised. Line 88: `wrap` goes to the WRAPPED park 314 and sends `WRAPPED` with no `MODE`. Lines 187 and 189: mode is NEUTRAL below 318 and the park is in the WRAPPED sector. Lines 90-92: `tidy`, `retrieve` and `decock` are in the command list. Line 254 adds TIDIED, and lines 376-381 describe the fault response (the drum holds with the wraps on, which matches safeToNeutral in base_logic.h:255). The version line says base-1.2.1, and section 10 lists the base-1.2 changes.

- File: `firmware/base/PROTOCOL.md:82`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: The Jetson implements against PROTOCOL.md. The doc says a `wrap` ends with `WRAPPED` then `MODE` (REEL). base-1.2 instead parks at 314 and sends only `WRAPPED`. The doc also says drum_pos WRAPPED means 'collar ramping' and that mode is NONE in the lock sector, while the firmware sends mode NEUTRAL at the park. A client waiting for the documented MODE REEL after wrap times out. Code that treats WRAPPED as wraps-on-and-engaged (mission.is_wrapped, mock_base) misreads the park, as the finding above shows. The firmware has `tidy` (TIDIED event), `retrieve` (SEATED, DART_STUCK) and `decock`, but none appears in the command list or the event table. The SAFING row still says the drum goes to NEUTRAL, while a fault from REEL or N_TO_R goes to the WRAPPED park.

Evidence: PROTOCOL.md:82 `wrap ... then into REEL with the fly-in ... WRAPPED event, then MODE (REEL)`; :158 mode `NEUTRAL (code NEUTRAL, wrap prep or WRAP ...), else NONE (shifting, lock sector, ...)`; :160 `WRAPPED (lock sector 302 to 340: wraps on, collar ramping)`; :157 `SAFING (fault: drum on its way to NEUTRAL)`; :222 `WRAPPED | ok, wraps | wraps laid and REEL engaged (followed by MODE)`; :232 example `WRAPPED ... then MODE ... "mode":"REEL"`. The command table at :70-88 has no tidy/retrieve/decock, and the event table at :213-229 has no TIDIED. Firmware: base_logic.h:1260 (WRAPPED only), :158 (NEUTRAL below collarNdeg 318), :979-990 (tidy/retrieve/decock), :1271 `evBegin(o, "TIDIED")`, :237 `safeTarget() ... ? DP_WRAPPED : DP_NEUTRAL`.

Fix: Rewrite the affected rows to BASE_SPEC 10.1 / D35. wrap: from NEUTRAL or LAUNCH to the WRAPPED park 314 (cup locked, collar NEUTRAL), `WRAPPED {ok,wraps:7}`, no MODE; then `mode REEL` is the fly-in (MODE REEL). drum_pos WRAPPED: the lock sector, where the park 314 is the rest point. mode: NEUTRAL also in the lock sector below 318 deg. Add rows for `tidy {ms,duty}` (TIDIED {ok}; NOT_IN_NEUTRAL, POSITION_UNKNOWN, BAD_ARG), `retrieve {duty,max_n,max_s}` (SEATED; FAULT DART_STUCK) and `decock` (alias of cal decock, RESET). Add TIDIED to section 4. State the SAFING target: WRAPPED from REEL/N_TO_R, NEUTRAL otherwise. Bump the version line to base-1.2 and list the changes in section 10.

> Skeptic (confirmed): I could not refute the claim. Every PROTOCOL.md line it cites says what it quotes.
- Line 3 still says firmware `base-1.0`.
- Line 82: `wrap` goes "into REEL with the fly-in ... WRAPPED event, then MODE (REEL)".
- Line 157: SAFING means the drum is "on its way to NEUTRAL".
- Line 158: mode is NEUTRAL only in NEUTRAL, PREP or WRAP. It is NONE in the lock sector.
- Line 160: WRAPPED means "lock sector ... collar ramping".
- Line 222: WRAPPED is "wraps laid and REEL engaged (followed by MODE)".
- Line 232: the example sequence shows MODE REEL after WRAPPED.
- grep finds no `tidy`, `retrieve`, `TI
>
> Skeptic (confirmed): I could not refute this. Every citation checks out. firmware/base/PROTOCOL.md still says it covers `base-1.0` (line 3), but base_logic.h:22 defines GB_FW_VERSION "base-1.2". PROTOCOL.md:82 says wrap goes 'then into REEL with the fly-in ... WRAPPED event, then MODE (REEL)', and :222 and :232/:396 say the same. In the code, cmdWrap (base_logic.h:761) calls startDrumMove(DP_WRAPPED) with the comment 'park at WRAPPED (314): collar NEUTRAL, cup locked'. tickWrap (:1258-1260) then sends only the WRAPPED event and no evMode, unlike tickUnwrap, which does call evMode. So a client that waits for the do
>

## 2. Firmware `retrieve` ignores max_s and faults DART_STUCK after a fixed 20 s, so a normal final approach from 1.5 m or more trips DART_STUCK

- Now (472bb65): **fixed** at `firmware/base/grappstar_base/base_logic.h:1086`. `retrieve` now calls approachLimit() (lines 847-854). That function reads max_s (5 to 600 s), and when max_s is absent it sets a default from line_m and the take-up speed. The DART_STUCK check at line 1375 now compares against approachMs instead of a fixed 20000 ms. At duty 0.3 the default works out to about 49 s for 1.5 m and about 76 s for 2.5 m, both longer than the spec's 20 + line_m/0.08, so a normal approach no longer faults at 20 s.

- File: `firmware/base/grappstar_base/base_logic.h:986`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: 10.1 step 17: retrieve {duty 0.3, max_s}, FAULT DART_STUCK after max_s, default 20 + line_m/0.08 s (the take-up is slow). The firmware reads only duty and max_n and uses approachMaxMs = 20000 for every final approach. With the normal approach_m of 1.5 m the spec allows 38.75 s, but the firmware faults at 20 s. Reproduced in basesim: mode NEUTRAL, set line_m 2.5, retrieve {duty 0.3, max_s 60} -> ok, then {"evt":"FAULT","code":"DART_STUCK","why":"final approach: no SW_SEATED in 20 s"} about 20 s later, with the dart not seated. The mission then shows a fault instead of seating the dart.

Evidence: base_logic.h:986-989 `else if (!strcmp(cmd, "retrieve")) { if (!o.num("duty", &a)) a = 0.3; if (!o.num("max_n", &m)) m = c.limitN; err = cmdReel(a, m, "seated"); }` (no max_s); base_logic.h:1269 `if (until == U_SEATED && now - opT > c.approachMaxMs) { raise(F_DART_STUCK, "final approach: no SW_SEATED in 20 s"); return; }`; config.h:154 `uint32_t approachMaxMs = 20000;`. BASE_SPEC.md 10.1 step 17 `retrieve {duty 0.3, max_s} ... FAULT DART_STUCK after max_s: default 20 + line_m / 0.08 s`.

Fix: In the retrieve branch, read max_s: `double ms; if (!o.num("max_s", &ms)) ms = 20.0 + (lineKnown ? lineM : 0) / 0.08; if (!(ms > 0 && ms <= 600)) err = "BAD_ARG";`, then store approachUntilMs = (uint32_t)(ms*1000) in a member and test `now - opT > approachUntilMs` at line 1269. Use the same default for `reel until seated` in NEUTRAL. Change the why text to 'final approach: no SW_SEATED in max_s'.

> Skeptic (confirmed): The defect is real. The claim gets one detail wrong: a final approach from exactly 1.5 m does not fault.

What the code does:
- In firmware/base/grappstar_base/base_logic.h:986-989, `retrieve` reads only `duty` and `max_n`. It never reads `max_s`, and it does not compute the default of 20 + line_m/0.08 s.
- Line 1269 faults `DART_STUCK` once the elapsed time passes `c.approachMaxMs`.
- config.h:154 hard-codes `approachMaxMs` to 20000 ms.

What the spec says: BASE_SPEC.md 10.1 step 17 requires `retrieve {duty 0.3, max_s}`, with the fault after max_s and a default of 20 + line_m/0.08 s. firmware
>
> Skeptic (confirmed): The defect is real; I could not refute it. The `retrieve` branch at firmware/base/grappstar_base/base_logic.h:986-989 reads only `duty` and `max_n` and never reads `max_s`. tickReel (line 1269) faults DART_STUCK when `now - opT > c.approachMaxMs`, and approachMaxMs is fixed at 20000 in config.h:154. Nothing else changes it: grep finds no other writer and no `max_s` handling anywhere in the firmware. The authoritative spec, BASE_SPEC.md 10.1 step 17, says `retrieve {duty 0.3, max_s}` faults DART_STUCK after max_s, with default 20 + line_m/0.08 s. D35 makes 10.1 the one fixed C1 sequence. So the
>

## 3. Mission tidies before wrapping (ANCHOR -> TIDY -> WRAP). With the firmware tidy now working, every GO run tightens the anchored line and only then lays the wraps (R1-A #9 persists and is worse)

- Now (472bb65): **fixed** at `jetson/grappstar/mission.py:471-480`. _sequence now runs BASE_NEUTRAL -> WRAP (to the park while the line is slack) -> ANCHOR -> TIDY -> FLY_IN (mode REEL) -> CHECK -> TOW, so the tidy no longer pulls the line taut before the wraps go on. tidy() (line 695) waits for TIDIED only when the firmware has the command and falls back to reel-until-slip/LIMIT only on UNKNOWN_CMD; it has no explicit drum-at-park guard, but every path that reaches it has already run the wrap.

- File: `jetson/grappstar/mission.py:388`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: Base-1.2 now has `tidy`. _sequence runs BASE_NEUTRAL, ANCHOR, then TIDY: tidy() sends `tidy` in NEUTRAL, which the firmware accepts, and the FIRM take-up pulls the anchored line taut. Only then does WRAP send `wrap`. D22 and D35 require the wrap at once while the line is slack, with the tidy after anchoring at the park. Laying turns on a taut anchored line has to draw line through the blocked capstan payout (about 360 N, D22). The N20 drum stalls in the WRAP sector, giving DRUM_TIMEOUT with the wraps partly on, and the firmware holds the drum ('inside the wrap window'). The mission is lost. In round 1 the tidy was dead code (UNKNOWN_CMD). It now really runs in the wrong place.

Evidence: mission.py:376-389 `r = self._do("ANCHOR", self.anchor)` ... `break` then `self._do("TIDY", self.tidy)` then `self._do("WRAP", self.wrap)`; mission.py:510 `self.base.cmd("tidy", timeout=2.0)`. Firmware base_logic.h:979-985 accepts tidy in NEUTRAL. BASE_SPEC 10.1 steps 8-12: mode NEUTRAL, wrap (WRAPPED), dart /auto (ANCHORED), tidy (TIDIED), mode REEL.

Fix: Reorder _sequence to BASE_NEUTRAL -> WRAP (to the park) -> ANCHOR -> TIDY -> REEL_MODE (base_mode('REEL')) -> CHECK -> TOW. In tidy(), first check that the drum is at the park (drum_pos WRAPPED, mode NEUTRAL) and refuse to tidy from NEUTRAL before a wrap. Wait for TIDIED only, not TIDY or LIMIT.

> Skeptic (confirmed): I could not refute this. The claim is accurate.

**What the code does.** `_sequence` in jetson/grappstar/mission.py (around lines 365-389) runs the steps in this order: BASE_NEUTRAL, then ANCHOR, then `self._do("TIDY", self.tidy)`, then `self._do("WRAP", self.wrap)`, then CHECK and TOW.
- `tidy()` sends `cmd("tidy")` and only skips on UNKNOWN_CMD.
- In firmware/base/grappstar_base/base_logic.h:979-985, `tidy` is accepted when the drum is in the NEUTRAL sector with the dogs open. That is exactly the state `base_neutral()` leaves the drum in.
- The firmware then runs the take-up forward for tidy
>
> Skeptic (confirmed): I could not refute this. I read the code, firmware and spec on w/review and did not edit anything.

1. Mission order is real. In jetson/grappstar/mission.py, _sequence breaks out of the shot loop after ANCHOR, then runs `self._do("TIDY", self.tidy)` and only then `self._do("WRAP", self.wrap)` (around lines 376-389). jetson/tests/test_mission.py:10 asserts this same order (... "TIDY", "WRAP", "CHECK" ...), so the tests lock the defect in.

2. The firmware accepts tidy before the wrap. In firmware/base/grappstar_base/base_logic.h:979-985, `tidy` is allowed when sec == SEC_NEUTRAL, which is where
>

## 4. Mission never sends `mode REEL` after `wrap` (D35 step 12), so the CHECK pull is refused NOT_WRAPPED at the 314 park and every GO run stops at CHECK

- Now (472bb65): **fixed** at `jetson/grappstar/mission.py:480`. The GO sequence now runs TIDY and then FLY_IN (fly_in, line 572, sends mode REEL from the WRAPPED park) before CHECK, so the CHECK pull no longer starts at the 314 park. Also, ensure_reel (lines 775-782) now calls fly_in whenever mode is not REEL, and anchored_check, the tow loop and retrieve all call it, so the reshoot path (wrap, then bring_home, then retrieve) also flies in before reel until stop.

- File: `jetson/grappstar/mission.py:566`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: GO run against the base-1.2 firmware. WRAP: wrap parks the drum at 314 and sends only WRAPPED (drum_pos WRAPPED, mode NEUTRAL, wraps 7). mission.wrap() then sees is_wrapped() true. CHECK calls ensure_reel(), which does nothing for the same reason, then sends reel until tension. The firmware treats the park as take-up only (sec LOCK, drumDeg 314 < collarNdeg 318) and refuses with NOT_WRAPPED. Every Retry does the same thing, so there is no tow. Reproduced with basesim built from this tree: mode NEUTRAL -> wrap -> WRAPPED; reel {duty 0.5, max_n 20, until tension} -> {"ok":false,"err":"NOT_WRAPPED"}; tidy -> TIDIED; mode REEL -> MODE REEL; the reel is then accepted. The reshoot path (wrap, then bring_home) has the same gap: retrieve's reel until stop at the park runs the take-up only, so line_m never falls.

Evidence: mission.py:566 `return t.get("drum_pos") == "WRAPPED" or (t.get("wraps") or 0) >= 3.5`; mission.py:569-572 ensure_reel only calls self.wrap(); mission.py:782-793 wrap() waits WRAPPED and is_wrapped, never mode REEL; mission.py:812 `self.base.reel(..., until="tension")`. Firmware base_logic.h:790-792 `bool neutral = (sec == SEC_NEUTRAL || (sec == SEC_LOCK && drumDeg < c.collarNdeg)) ...; if (neutral && un == U_TENSION) return "NOT_WRAPPED";` and base_logic.h:761 wrap -> startDrumMove(DP_WRAPPED). BASE_SPEC 10.1 step 12 says `mode` REEL -> `MODE` ok (REEL).

Fix: Change is_wrapped to `t.get("mode") == "REEL" and (t.get("wraps") or 0) >= 3.5` (rename it engaged). Add a park check `t.get("drum_pos") == "WRAPPED" and (t.get("wraps") or 0) >= 3.5`. Make wrap() skip the command only when parked or engaged. Make ensure_reel() call wrap() when not parked, then (after the tidy) self.base_mode("REEL"), waiting for MODE with mode REEL. Drop the NOT_IN_REEL fallback in wrap(). In reshoot(), send mode REEL after wrap() before bring_home().

> Skeptic (confirmed): I could not refute this. The defect is real; I traced it in the code but did not run basesim myself.

1. Nothing in the mission sends `mode REEL` after the wrap. The only `base_mode("REEL")` call is the NOT_IN_REEL fallback inside `wrap()` (mission.py:789). That fallback only runs if the wrap command itself is refused, which the base-1.2 `cmdWrap` (base_logic.h:751-764) never does. That function drives the drum to the DP_WRAPPED park, with the collar NEUTRAL and the cup locked.
2. `_sequence` (mission.py:366-401) goes TIDY, WRAP, CHECK, TOW. It has no step that sends `mode REEL`, even though B
>
> Skeptic (confirmed): I tried to refute the claim and could not. It is real.

The code:
- jetson/grappstar/mission.py:366-401 (_sequence) runs TIDY, then WRAP, then CHECK. It never calls base_mode("REEL"). The only `mode REEL` in mission.py is the NOT_IN_REEL fallback in wrap() at line 789.
- The current firmware never sends NOT_IN_REEL from cmdWrap, so that fallback never runs.
- is_wrapped (line 566) returns true as soon as drum_pos is WRAPPED. That means wrap() and ensure_reel() both return without doing anything once the drum is at the park.

The firmware (firmware/base/grappstar_base/base_logic.h):
- cmdWrap (
>

## 5. Mission reads the D35 WRAPPED park (drum_pos WRAPPED, mode NEUTRAL, wraps 7) as an engaged capstan, so it never sends `mode REEL` and every CHECK/TOW reel is refused (R1 #10 still open, new form)

- Now (472bb65): **partly fixed** at `jetson/grappstar/mission.py:768-782`. The main failure is fixed. ensure_reel() (line 775) now calls fly_in() (`mode REEL`) whenever mode != REEL, after checking is_wrapped, and it runs before CHECK (1062), on every TOW round (1110) and before the capstan RETRIEVE (1192), so the WRAPPED park after a fault gets flown in again. What remains: is_wrapped (line 773) still returns True for `wraps >= 3.5` although P_DRUM_WRAP_TURNS is 7. After a fault held partway through the wrap sectors (state no longer WRAP, wraps about 4 to 6), wrap() (line 1030) returns without sending wrap, and ensure_reel then sends `mode REEL` away from the park.

- File: `jetson/grappstar/mission.py:566`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: Normal GO run against base-1.2. WRAP: the firmware parks at 314 and sends only WRAPPED. Telemetry then reads {mode:NEUTRAL, drum_pos:WRAPPED, wraps:7} (seen in basesim). is_wrapped() returns True, so CHECK's ensure_reel() never runs the D35 step 12 `mode REEL`. reel(until=tension) lands on cmdReel with neutral=true (LOCK, 314<318) and the firmware refuses NOT_WRAPPED. Retry hits the same refusal every time, and TOW does the same. I proved it with jetson/tests/test_mission_realsim.py against freshly built basesim: 'hung at CHECK: base refused reel: NOT_WRAPPED' x3. The same misread shows in two more places. (a) After a tow fault the firmware safes to the WRAPPED park (safeTarget), and Retry TOW is refused again. (b) On the reshoot path, RETRIEVE `reel until stop` at the park is accepted as take-up only: line_m never falls, so the step times out with 'line_m not falling'. Separately, the `wraps >= 3.5` threshold dates from the 4-wrap model. Wraps now run 0..7, and the estimate reaches 4 about halfway through the WRAP sector. After a fault held inside the wrap window (safeToNeutral holds there), Retry WRAP returns at once without sending wrap.

Evidence: mission.py:565-566 `def is_wrapped(t: dict) -> bool:\n        return t.get("drum_pos") == "WRAPPED" or (t.get("wraps") or 0) >= 3.5`; mission.py:570 `if wrapped and not self.is_wrapped(self.tel()):`; mission.py:812 `self.base.reel(self.cfg.reel_duty * 0.6, max_n=self.cfg.check_n, until="tension")`; base_logic.h:761 `if (!startDrumMove(DP_WRAPPED, false)) ...   // park at WRAPPED (314): collar NEUTRAL`; base_logic.h:1260 tickWrap emits only WRAPPED (no MODE); base_logic.h:158 modeName returns "NEUTRAL" for `sec == SEC_LOCK && drumDeg < c.collarNdeg`; base_logic.h:790-792 `bool neutral = (sec == SEC_NEUTRAL || (sec == SEC_LOCK && drumDeg < c.collarNdeg)) ... if (neutral && un == U_TENSION) return "NOT_WRAPPED";`. basesim telemetry after wrap: {'state':'IDLE','mode':'NEUTRAL','drum_deg':314,'drum_pos':'WRAPPED',...,'wraps':7}; then reel(until=tension) -> 'base refused reel: NOT_WRAPPED'; mode REEL -> MODE ok mode REEL drum_pos REEL.

Fix: Split the predicate. `def is_wrapped(t): return t.get("drum_pos") in ("WRAPPED", "REEL") and t.get("wraps") == 7` (or >= params wrap_turns). Add `def reel_engaged(t): return t.get("mode") == "REEL" and t.get("drum_pos") == "REEL"`. wrap() returns early only if is_wrapped. After WRAPPED it waits for drum_pos WRAPPED, not a MODE event. ensure_reel(): if not is_wrapped then wrap(); then if not reel_engaged then base_mode("REEL") (D35 step 12). Call ensure_reel() before CHECK, TOW and the capstan RETRIEVE, and again on every Retry, because a fault from REEL safes to the park.

> Skeptic (confirmed): I could not refute this; the code confirms it. In firmware/base/grappstar_base/base_logic.h, cmdWrap calls startDrumMove(DP_WRAPPED), which parks the drum at 314 with the collar in NEUTRAL. tickWrap then sends only a WRAPPED event, with no MODE event. modeName() reports NEUTRAL for SEC_LOCK when drumDeg < collarNdeg. cmdReel treats that same position as neutral and returns NOT_WRAPPED for until=="tension". That matches D35 in DECISIONS.md: wrap ends at the 314 park with no MODE REEL, and a separate `mode REEL` (the fly-in) has to come before the tow.

In jetson/grappstar/mission.py, line 566 m
>
> Skeptic (confirmed): I could not refute the claim. I checked it against the code and reproduced it.

Firmware (firmware/base/grappstar_base/base_logic.h):
- cmdWrap parks at DP_WRAPPED with collar NEUTRAL.
- tickWrap emits only the WRAPPED event, with no MODE.
- modeName returns "NEUTRAL" for SEC_LOCK with drumDeg < collarNdeg.
- cmdReel sets neutral=true for that same condition and returns NOT_WRAPPED when until=tension.

This matches DECISIONS.md D35: wrap ends at the WRAPPED park with no MODE REEL, and "mode REEL is the fly-in" is a separate later step.

Mission (jetson/grappstar/mission.py):
- is_wrapped (line
>

## 6. PROTOCOL.md was not updated for D35: no tidy/retrieve/decock commands, no TIDIED event, wrap still documented as ending in REEL with MODE, and the typical mission is the old order (R1-A #1/#2 persist)

- Now (472bb65): **fixed** at `firmware/base/PROTOCOL.md:88`. PROTOCOL.md has been rewritten for base-1.2.1 / D35. It now has `tidy`, `retrieve` and `decock` (lines 90-92) and the TIDIED event (line 254). `wrap` is now allowed from LAUNCH, ends in WRAPPED with no MODE, and `mode REEL` does the fly-in. Section 8 (lines 455+) now runs wrap, tidy, mode REEL, ..., unwrap, retrieve. The version matches GB_FW_VERSION, and jetson/grappstar/comms/mock_base.py now handles TIDIED.

- File: `firmware/base/PROTOCOL.md:82`
- Severity: medium (claimed high); 1 of 2 skeptics confirmed
- Owner: fw-base

Scenario: A Jetson or mock author working from the contract implements wrap as 'WRAPPED, then MODE (REEL)', waits for a MODE REEL that never comes, and never sends `mode REEL` (which is exactly what mission.py does). The contract does not list `tidy` {ms,duty}/TIDIED, `retrieve` {duty,max_s}/SEATED or the top-level `decock`, so the mock still answers UNKNOWN_CMD for all three. It says wrap from LAUNCH is refused NOT_IN_NEUTRAL, while the firmware (and step 8 of 10.1) allow it. The typical mission in section 8 still tidies with `reel until slip` before `wrap`, the order D22/D35 forbid. Header and section 10 still say fw base-1.0, but the firmware reports base-1.2. PROTOCOL.md was last changed in 8f41b6f, before both D35 (201d392) and the firmware change 01f1fe0.

Evidence: PROTOCOL.md:82 wrap `... then into REEL with the fly-in ... WRAPPED event, then MODE (REEL) ... NOT_IN_NEUTRAL (drum in LAUNCH or the LAUNCH to NEUTRAL ramp)`; :222 `WRAPPED | ok, wraps | wraps laid and REEL engaged (followed by MODE)`; :46 long-action list without tidy/retrieve/decock; :68-88 command table has no tidy, retrieve or decock rows; :213-229 event table has no TIDIED; :111 `until:"slip": the tidy before a wrap`; :395-396 typical mission tidy then wrap `-> WRAPPED, MODE REEL`; :3 `Firmware base-1.0`. Firmware base_logic.h:979-990 (tidy, retrieve, decock), :1271 TIDIED, :761 wrap parks at DP_WRAPPED with no MODE; basesim HELLO fw "base-1.2".

Fix: Rewrite PROTOCOL.md to BASE_SPEC 10.1 and bump the version to base-1.2. wrap: from LAUNCH/NEUTRAL to the WRAPPED park 314 (collar NEUTRAL, cup locked), event WRAPPED only (no MODE), already parked or in REEL gives an immediate WRAPPED; refusals POSITION_UNKNOWN, LAUNCH_BUSY, HAND_MODE, HOLDING. Add rows: `tidy` {ms default 3000, max 60000, duty default 0.3} take-up only at the park or NEUTRAL, TIDIED event, refusals BAD_ARG/NOT_IN_NEUTRAL/POSITION_UNKNOWN plus the common ones; `retrieve` {duty 0.3, max_s} NEUTRAL take-up until SW_SEATED, SEATED, FAULT DART_STUCK after max_s; `decock` = `cal decock`. mode REEL = fly-in from the park. Add TIDIED to section 4, fix WRAPPED's 'when' text and the section 4 example, and replace section 8 with the 18 steps of 10.1. Add a section 10 entry for these changes.

> Skeptic (refuted): The claim is accurate for the local checkout, but that checkout is out of date. It is not the current origin/base-v1, which is the build under review.

1. Local tree: firmware/base/PROTOCOL.md was last changed in 8f41b6f. Line 3 says base-1.0. Line 82 says wrap ends with "WRAPPED event, then MODE (REEL)". There are no rows for tidy, retrieve or decock and no TIDIED event. Line 111 says `until:"slip"` is "the tidy before a wrap". Lines 395-396 give the old mission order. Meanwhile the local base_logic.h (01f1fe0) has the tidy, retrieve and decock commands, has wrap park at DP_WRAPPED, and repor
>
> Skeptic (confirmed): The claim holds. I checked it against the code and could not refute it. firmware/base/PROTOCOL.md was last changed in 8f41b6f. That is before 01f1fe0, the firmware change to spec v1.2, which added tidy, retrieve and decock and the WRAPPED park 314. Line 3 of PROTOCOL.md still says `base-1.0`, but base_logic.h:22 defines GB_FW_VERSION "base-1.2". The long-action list at :46 and the command table at :68-88 have no tidy, retrieve or top-level decock, yet base_logic.h:979-990 implements all three. The event table has no TIDIED, but base_logic.h:1271 emits it. :82 and :222 say wrap ends in REEL wit
>

## 7. `retrieve` ignores max_s and DART_STUCK fires after a fixed 20 s, not the D35 default of 20 + line_m/0.08 s

- Now (472bb65): **fixed** at `firmware/base/grappstar_base/base_logic.h:847-855 (approachLimit), :1086 (retrieve), :1375 (DART_STUCK uses approachMs); jetson/grappstar/mission.py:1233-1236`. The retrieve handler now parses `max_s` (5 to 600 s, otherwise BAD_ARG) and stores it in `approachMs`, and tickReel's DART_STUCK check uses that value. With no `max_s`, the default is computed from the take-up speed model, max(20, 10 + 1.5*line_m/v), which gives about 49.5 s for 1.5 m at duty 0.3, longer than the spec's 38.75 s. With line_m unknown it is 30 s. The Jetson's final_approach also sends max_s = 20 + line_m/0.08 and waits max_s + 10 s.

- File: `firmware/base/grappstar_base/base_logic.h:1269`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: Normal return: the capstan reel-in stops with line_m about 1.5 m (D35 step 15; mission approach_m 1.5). After unwrap, `retrieve` runs the take-up in NEUTRAL. At the slow take-up speed of about 0.08 m/s that BASE_SPEC assumes, 1.5 m takes about 19 s before any slack or drag, and the spec allows 20 + 1.5/0.08 = 38.75 s. The firmware raises FAULT DART_STUCK at 20 s regardless, and a `max_s` sent by the Jetson is ignored. A dart still moving in gets a false DART_STUCK, and the mission shows a base FAULT on a normal run. Any remaining line above about 0 m makes the fixed limit shorter than the spec value.

Evidence: base_logic.h:986-988 `else if (!strcmp(cmd, "retrieve")) { if (!o.num("duty", &a)) a = 0.3; if (!o.num("max_n", &m)) m = c.limitN; err = cmdReel(a, m, "seated");` (no max_s); base_logic.h:1269 `if (until == U_SEATED && now - opT > c.approachMaxMs) { raise(F_DART_STUCK, "final approach: no SW_SEATED in 20 s");`; config.h:154 `uint32_t approachMaxMs = 20000;`; BASE_SPEC.md 10.1 step 17 `retrieve {duty 0.3, max_s} | SEATED | ... (FAULT DART_STUCK after max_s: default 20 + line_m / 0.08 s, the take-up is slow)`.

Fix: In the retrieve handler, parse `max_s`. Default it to `20 + (lineKnown ? lineM : 0) / 0.08`, and store it in a member such as `approachLimitMs = (uint32_t)(max_s * 1000)`. Use that member in tickReel's DART_STUCK check instead of c.approachMaxMs, and drop the fixed '20 s' from the why text. On the Jetson, final_approach should send `max_s = 20 + line_m/0.08` and keep final_timeout_s above it.

> Skeptic (confirmed): I could not refute this. The firmware breaks the binding command table. BASE_SPEC 10.1 is headed "binding, D35", and step 17 there says `retrieve {duty 0.3, max_s}` raises FAULT DART_STUCK "after max_s: default 20 + line_m / 0.08 s, the take-up is slow".

What the code does:
- The retrieve handler in base_logic.h:986-988 reads only duty and max_n, then calls cmdReel(a, m, "seated"). It never reads max_s.
- tickReel (base_logic.h:1269) uses the fixed c.approachMaxMs, which config.h:154 sets to 20000, and the fault text says "20 s".
- line_m does not drop during unwrap or the take-up-only phase.
>
> Skeptic (confirmed): I checked the claim against the code and could not refute it. The `retrieve` handler at base_logic.h:986-988 reads only `duty` and `max_n`. It never parses `max_s`, then calls cmdReel(a, m, "seated"). In tickReel, the NEUTRAL path at base_logic.h:1269 raises F_DART_STUCK once now - opT > c.approachMaxMs. config.h:154 sets that to 20000 ms. A grep shows approachMaxMs is used nowhere else, so neither `set` nor the command can change it. BASE_SPEC 10.1 step 17 is binding under D35 and says `retrieve {duty 0.3, max_s}`, with DART_STUCK after max_s and a default of 20 + line_m/0.08 s. Step 15 stops
>

## 8. mock_base.py still sends the old drum telemetry (WRAPPED 338 with mode REEL accepting reel tension, 4 wraps, no tidy/retrieve/TIDIED/DART_STUCK), so mock mission tests pass on output the firmware never produces (R1 #4/#6 still open)

- Now (472bb65): **fixed** at `jetson/grappstar/comms/mock_base.py:1272 (mode REEL only from N_TO_R), :1465-1467 (no tension refusal on take-up), :1114 (tidy/retrieve/decock commands), :938 (DART_STUCK)`. The mock now uses the params.DRUM zones (park at 314, REEL 347, 7 wraps, collar NEUTRAL below 318) and has tidy, retrieve, decock and the DART_STUCK fault. A scratch run (/tmp/claude-0/review-scratch-stale/m13.py) reached the park after wrap with drum 314, 7 wraps and mode NEUTRAL; `reel until tension` there was refused with NOT_WRAPPED and `tidy` was accepted.

- File: `jetson/grappstar/comms/mock_base.py:47`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: Mock mission test: after wrap the mock reports drum_pos WRAPPED with mode REEL and wraps 4. It accepts reel until tension there, so CHECK and TOW pass. `tidy` and `retrieve` reply UNKNOWN_CMD, so the mission skips TIDY and runs the capstan fallback in FINAL_APPROACH. The firmware reports mode NEUTRAL and wraps 7 at the park, refuses reel tension there, emits TIDIED, and can raise DART_STUCK. The realsim test hangs at CHECK, but the mock suite stays green, so it hides finding 1 and never exercises the TIDIED, retrieve or DART_STUCK handling.

Evidence: mock_base.py:47 `DRUM_DEG = {"LAUNCH": 0.0, "NEUTRAL": 40.0, "REEL": 80.0, "WRAPPED": 338.0}`; :50 `N_WRAPS = 4`; :231-232 `if self.drum_deg >= DRUM_DEG["REEL"] - DRUM_WIN_DEG: return "REEL"  # REEL, the wrap sectors and WRAPPED: the collar stays in REEL`; :1200 `if self.drum_pos() != "WRAPPED": return "NOT_WRAPPED"`; :98-99 FAULTS without DART_STUCK; :930 `return fn(m, extra) if fn else "UNKNOWN_CMD"` (no tidy/retrieve handler). Firmware: config.h targets {0,41,347,314}, wrapsFull 7, base_logic.h:158, :790-792, :979-990, :1269.

Fix: Re-model the mock on base_logic.h: zones and targets from params.DRUM (LAUNCH 0, NEUTRAL 41, WRAP 62..302, park 314, REEL 347), wraps 0..7. The modeName rules: NEUTRAL in the lock sector below 318, REEL only at REEL. wrap parks at 314 and emits WRAPPED only. mode REEL works only from the lock sector. reel: capstan only in REEL; in NEUTRAL or below 318 in the lock sector it is take-up only, with tension refused NOT_WRAPPED and line_m unchanged. Add tidy (TIDIED), retrieve (SEATED, DART_STUCK after max_s), decock, and DART_STUCK in FAULTS.

> Skeptic (confirmed): I could not refute this. The core claim is real, and one supporting detail is overstated.

What the mock does, in jetson/grappstar/comms/mock_base.py:
- Line 47 sets DRUM_DEG = {LAUNCH 0, NEUTRAL 40, REEL 80, WRAPPED 338}.
- Line 50 sets N_WRAPS = 4.
- Lines 231-232: the `mode` property returns REEL from REEL - 5 deg onward. That covers the wrap sectors and WRAPPED.
- _cmd_reel (around line 1200) accepts any `until`, including tension, as long as drum_pos() is WRAPPED.
- FAULTS (lines 98-99) has no DART_STUCK.
- The _commands() table (around lines 914-925) has no tidy, retrieve or decock, so t
>
> Skeptic (confirmed): I checked the code and the claim holds. In jetson/grappstar/comms/mock_base.py, line 47 sets DRUM_DEG to {LAUNCH 0, NEUTRAL 40, REEL 80, WRAPPED 338} and line 50 sets N_WRAPS = 4. The mode property at lines 230-232 returns "REEL" for every angle from REEL up to and including WRAPPED. _cmd_reel only accepts the command when drum_pos() == "WRAPPED" and allows until=tension there. The _commands() table has no tidy, retrieve or decock entries, so those commands return UNKNOWN_CMD (line 930). FAULTS has no DART_STUCK.

The firmware behaves differently:
- config.h:127 sets the park targets to {0, 41
>

## 9. mock_base.py still models the pre-D16 drum and has no tidy/retrieve/decock, so the mission tests never run the D35 sequence (R1-A #4 persists)

- Now (472bb65): **fixed** at `jetson/grappstar/comms/mock_base.py:1111-1115`. The mock now models the D35 drum: wrap parks at 314 (line 1423), mode REEL returns NOT_WRAPPED below N_TO_R (1277-1278), unwrap goes to NEUTRAL (1446), and take-up reel works at NEUTRAL or the park with tension refused (1463-1467). The dispatch at 1114-1115 now has tidy, retrieve and decock. Two stop points differ slightly from the suggested fix (LAUNCH 0 instead of 1.5, REEL 347 instead of 347.5), which doesn't matter here; I stopped the test_mission run before it finished.

- File: `jetson/grappstar/comms/mock_base.py:1173`
- Severity: medium (claimed high); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: Commands from 10.1 behave differently on the mock. Step 9 `wrap` from NEUTRAL gives NOT_IN_REEL on the mock (the firmware parks at 314). Step 11 `tidy`, step 17 `retrieve` and the `decock` recovery give UNKNOWN_CMD on the mock (the dispatch has none of them), and the firmware has all three. Step 12 `mode REEL` from NEUTRAL moves the drum on the mock (the firmware refuses with NOT_WRAPPED). Step 16 `unwrap` ends at REEL 80 on the mock (the firmware ends at NEUTRAL). Reel works only at WRAPPED 338 on the mock, so the mock accepts a tension reel at its 'WRAPPED' stop while the firmware refuses it at the 314 park. As a result test_mission passes with the NOT_WRAPPED-at-CHECK defect and the tidy-before-wrap order, and never sees TIDIED or the retrieve path.

Evidence: mock_base.py:47 `DRUM_DEG = {"LAUNCH": 0.0, "NEUTRAL": 40.0, "REEL": 80.0, "WRAPPED": 338.0}`; :1173-1174 `if self.code_bad or self.drum_deg < DRUM_DEG["REEL"] - DRUM_WIN_DEG: return "NOT_IN_REEL"`; :1192 unwrap -> DRUM_DEG["REEL"]; :1201-1202 `if self.drum_pos() != "WRAPPED": return "NOT_WRAPPED"`; :1204 until without slip; :916-924 command map has no tidy, retrieve or decock. Firmware base_logic.h:761, 979-990, 790-792.

Fix: Rebuild the mock's drum from config.h/base_logic.h: sectors LAUNCH/LN/NEUTRAL/PREP/WRAP/LOCK/REEL; stop points LAUNCH 1.5, NEUTRAL 41, WRAPPED park 314, REEL 347.5. wrap from LAUNCH/NEUTRAL to the park with WRAPPED only. mode REEL only from LOCK (NOT_WRAPPED below). unwrap to NEUTRAL with UNWRAPPED then MODE. reel take-up only at NEUTRAL or the park (tension -> NOT_WRAPPED, slip -> LIMIT tidy). Add `tidy` {ms, duty} -> TIDIED, `retrieve` {duty, max_s} NEUTRAL only -> SEATED or FAULT DART_STUCK, and `decock` as an alias of cal decock. Alternatively run test_mission against basesim (test_mission_realsim.py) as the gate.

> Skeptic (confirmed): I read the code and the claim holds. mock_base.py:47 still uses the old drum map, DRUM_DEG = {LAUNCH 0, NEUTRAL 40, REEL 80, WRAPPED 338}, with sectors L-N/N-R/WRAP and no PREP/LOCK. _cmd_wrap returns NOT_IN_REEL below REEL-5 (line 1173). _cmd_unwrap drives to DRUM_DEG["REEL"] (line 1192). _cmd_reel accepts only drum_pos()=="WRAPPED" (line 1201). The _commands() dispatch (lines 916-924) has no tidy, retrieve or decock, so handle() returns UNKNOWN_CMD for all three. The firmware has all three: base_logic.h:979 tidy (event TIDIED at 1271), :986 retrieve, :990 decock as an alias of cal decock. Th
>
> Skeptic (confirmed): The claim holds; I found nothing that refutes it. In jetson/grappstar/comms/mock_base.py, line 47 still has the old drum stops (LAUNCH 0, NEUTRAL 40, REEL 80, WRAPPED 338). The command map at lines 914-924 has no tidy, retrieve or decock, so handle() returns UNKNOWN_CMD for all three. _cmd_wrap (lines 1162-1176) returns NOT_IN_REEL when the drum is below REEL, which includes NEUTRAL. _cmd_unwrap ends at REEL (line 1192). _cmd_reel only accepts drum_pos()=="WRAPPED" at 338 and has no NEUTRAL take-up path. The firmware is different: firmware/base/grappstar_base/base_logic.h has tidy at line 979,
>

## 10. is_wrapped() counts 4 of 7 wraps as wrapped, so after a D27 hold mid-wrap the Retry skips the wrap and CHECK is refused forever; no unwrap option is offered (R1 #10 persists)

- Now (472bb65): **still open** at `jetson/grappstar/mission.py:768-773`. is_wrapped() still returns True when wraps >= 3.5 (firmware full count is P_DRUM_WRAP_TURNS 7). The new guard on state WRAP/UNWRAP does not help, because after a drum fault the state is FAULT/HOLD. So a hold mid-wrap with drum_pos BETWEEN and wraps 4..6 still makes wrap() (line 1029) return at once, and the firmware's reel/mode REEL then refuses with NOT_WRAPPED (base_logic.h:708, 833). The fault prompt (lines 396-401) only changes the message on DRUM_HELD and still offers Retry/Abort, with no Unwrap choice.

- File: `jetson/grappstar/mission.py:566`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: A drum fault (DRUM_TIMEOUT, BROWNOUT, SWITCH_DISAGREE) inside the WRAP sector after the 4th cup turn. The firmware holds the drum (safeToNeutral: 'inside the wrap window'), and telemetry shows drum_pos BETWEEN with wraps 4..6 and wrapsKnown true. The operator picks Retry -> wrap() returns at once because wraps >= 3.5. The WRAP step 'succeeds', then CHECK sends reel and the firmware answers NOT_WRAPPED (sector WRAP is neither REEL nor 'neutral'). A `mode REEL` would also be refused NOT_WRAPPED because sec < SEC_LOCK. Retry never recovers. Spec 10.1 says the Jetson 'offers unwrap or retry' on DRUM_HELD, but the mission offers only Retry/Abort.

Evidence: mission.py:566 `return t.get("drum_pos") == "WRAPPED" or (t.get("wraps") or 0) >= 3.5`; mission.py:780-782 `def wrap(self): if self.is_wrapped(self.tel()): return`. config.h:131-136 wrapsFull = 7; base_logic.h:164-165 wraps counted linearly 0..7 across the WRAP sector; base_logic.h:246 `if (sec == SEC_WRAP) return "inside the wrap window: the operator decides (unwrap)"`; base_logic.h:655 `if (sec < SEC_LOCK) return "NOT_WRAPPED";`.

Fix: `is_wrapped = lambda t: t.get("drum_pos") in ("WRAPPED", "REEL")`. Do not use the wraps count, or require wraps >= the firmware's full count (7, from params). When a DRUM_HELD MODE event is present, add an "Unwrap" choice to the prompt that calls self.unwrap() before Retry.

> Skeptic (confirmed): The defect is real. The line numbers in the claim are stale, and the first step that fails is not CHECK, but the substance holds.

What the code does:
- is_wrapped is at mission.py:768-773, not :566. It returns False while state is WRAP or UNWRAP. Otherwise it returns `drum_pos in ("WRAPPED","REEL") or wraps >= 3.5`.
- The firmware counts wraps linearly across the WRAP sector, from 0 to wrapsFull=7 (base_logic.h:164-168 wrapsAt). So wraps of 4, 5 or 6 can show while drum_pos is BETWEEN.
- A fault inside WRAP holds the drum. safeToNeutral returns "drum fault" for DRUM_TIMEOUT, or "wraps on..." 
>
> Skeptic (confirmed): I could not refute this claim. The line numbers in the claim are wrong, and the step where it fails is FLY_IN, not CHECK. The defect itself is real.

1. is_wrapped (mission.py:768-773) returns False only when state is WRAP or UNWRAP. Otherwise it returns True when drum_pos is WRAPPED or REEL, or when wraps >= 3.5. The firmware counts wraps with wrapsAt (base_logic.h:166-170): floor(f+0.15) on a 0..7 scale (wrapsFull = 7) inside SEC_WRAP. So a drum stopped part way, at f >= 3.85, reports wraps 4..6.

2. A fault inside the WRAP sector holds the drum where it is. safeToNeutral (base_logic.h:250-2
>

## 11. Mission still seats on the capstan and sends `retrieve` without duty/max_s (R1-A #5 partly persists)

- Now (472bb65): **partly fixed** at `jetson/grappstar/mission.py:1201-1204 (with bring_home at 564-570)`. Fixed: retrieve() now skips the capstan when the line length is unknown or short, and otherwise reels with until="stop", so it never asks to seat on the capstan. final_approach() (1222-1236) unwraps first, sends retrieve with duty, max_n and max_s=20+line/0.08, waits max_s+10, and no longer has the UNKNOWN_CMD capstan fallback; mock_base now implements retrieve. Still open: if SW_SEATED closes during the capstan reel-in, retrieve() only stops, logs a warning and returns, and bring_home() then unwraps with the dart seated without asking the operator, which is not what the fix asked for.

- File: `jetson/grappstar/mission.py:938`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: (a) RETRIEVE with line_m unknown, or line_m <= approach_m + 0.3 (a short shot or a reshoot), sends reel until seated on the capstan. bring_home() then runs unwrap from REEL with the dart seated, which 10.1 step 17 forbids. (b) final_approach() sends a bare {"cmd":"retrieve"}, so max_s never follows the 10.1 default (20 + line_m/0.08). It also keeps the UNKNOWN_CMD fallback that wraps again and seats on the capstan, and that fallback runs against the unchanged mock_base (which has no retrieve).

Evidence: mission.py:938 `self.base.reel(self.cfg.retrieve_duty, max_n=self.cfg.retrieve_limit_n, until="seated")`; mission.py:952 `self.base.cmd("retrieve")`; mission.py:955-963 fallback `self.wrap()` ... `self.base.reel(..., until="seated")` ... `self.unwrap()`. BASE_SPEC 10.1 steps 15-17.

Fix: In retrieve(), never use until='seated'. With line unknown, use until='stop' and stop when line_m <= approach_m, on SEATED, or after a timed pull, and leave the seating to FINAL_APPROACH. If SW_SEATED closes while wrapped, stop and ask the operator instead of unwrapping automatically. In final_approach(), first check drum_pos == 'NEUTRAL' and wraps == 0, then send `self.base.cmd("retrieve", duty=0.3, max_s=round(20 + (line_m or self.cfg.approach_m) / 0.08, 1))` and wait for SEATED with a timeout of max_s + 10. Remove the UNKNOWN_CMD capstan fallback.

> Skeptic (confirmed): I could not refute this. The cited code is there and behaves as described, though part (b) matters less than claimed.

(a) mission.py:909 takes the until='stop' capstan path only when line is known and line > approach_m + 0.3 (1.8 m with the default approach_m of 1.5). In every other case, line:938 runs `base.reel(retrieve_duty, ..., until="seated")` on the capstan. ensure_reel() wraps first, and reshoot() always runs WRAP before bring_home(). So the dart seats on the capstan, bring_home() then runs UNWRAP, and final_approach() returns early because seated_in_retrieve is set. BASE_SPEC 10.1 st
>
> Skeptic (confirmed): Part (a) holds up against the code. In jetson/grappstar/mission.py, retrieve() at lines 896-943 only takes the until="stop" path when line is known and line > approach_m + 0.3 (1.8 m by default). Otherwise it falls through to line 938, `self.base.reel(self.cfg.retrieve_duty, max_n=..., until="seated")`, after calling ensure_reel(), which wraps if the capstan is not already wrapped. So the dart is seated by the capstan at duty 0.6 with the wraps on. bring_home() (lines 403-407) then runs UNWRAP with the dart already seated, and final_approach returns early. BASE_SPEC 10.1 step 17 says "Never se
>

## 12. BASE_SPEC 10.1 recovery says a fault in REEL/N_TO_R holds the drum with DRUM_HELD, but D31 and the firmware drive to the 314 park and send MODE ok

- Now (472bb65): **fixed** at `base/spec/BASE_SPEC.md:193-195; firmware/base/grappstar_base/base_logic.h:255; firmware/base/test/test_base.cpp:465; jetson/grappstar/mission.py:396-401`. The decision went the other way from the suggested fix: D31 is now marked superseded ("faults with the wraps on HOLD the drum, params 1.2.5"), and the firmware was changed to match spec 10.1. On a fault in REEL it holds and sends MODE DRUM_HELD with why "wraps on: the drum holds" (test_base.cpp:465 checks that it stays in DP_REEL), and the mission no longer prints the fixed "went to NEUTRAL" text (it shows the held reason or the telemetry drum_pos). One thing left over: the Jetson mock (jetson/grappstar/comms/mock_base.py:18 and :533 _safe_target) still parks REEL/N_TO_R at WRAPPED, which no longer matches the firmware.

- File: `base/spec/BASE_SPEC.md:195`
- Severity: low (claimed low); unverified (low)
- Owner: coordinator

Scenario: A fault during the tow (seen in basesim: STALL in REEL). The firmware safes the drum from REEL 347 to the park 314 and sends {"evt":"MODE","ok":true,"mode":"NEUTRAL","drum_pos":"WRAPPED"}. 10.1 says the drum HOLDS where it is and the Jetson reads MODE DRUM_HELD. The mission, written to 10.1, finds no DRUM_HELD and tells the operator 'The drum went to NEUTRAL', which is wrong: it is at the park with 7 wraps on.

Evidence: BASE_SPEC.md:194-195 `A fault with the wraps on (REEL, N_TO_R, the WRAP sector or the park) HOLDS the drum where it is (D27): the Jetson reads the MODE event (DRUM_HELD ...)`; DECISIONS D31 'the fault rule "from REEL or N_TO_R go to WRAPPED, never NEUTRAL"'; base_logic.h:237 `int safeTarget() const { return (sec == SEC_REEL || sec == SEC_LOCK) ? DP_WRAPPED : DP_NEUTRAL; }`; mission.py:340-341 `"The drum went to NEUTRAL. "`.

Fix: Coordinator: make 10.1 match D31: 'from REEL or N_TO_R the drum safes to the WRAPPED park (MODE ok:true, drum_pos WRAPPED); in the WRAP sector or at the park it holds (MODE DRUM_HELD)'. Then the jetson owner reports the drum_pos from the MODE event in the mission message instead of the fixed 'went to NEUTRAL'.


## 13. Firmware `retrieve` is accepted in REEL and seats the dart on the capstan, which 10.1 step 17 forbids ('NEUTRAL only. Never seat on the capstan')

- Now (472bb65): **fixed** at `firmware/base/grappstar_base/base_logic.h:869`. cmdReel now refuses any until=seated request (which is what retrieve forwards at line 1086) with WRAPS_ON whenever sec != SEC_NEUTRAL. A retrieve in REEL is now rejected before the capstan starts. When sec is NEUTRAL but a dog switch is closed, collarReel() is false, so the request also fails, with NOT_WRAPPED. The error code differs from the suggested NOT_IN_NEUTRAL, but jetson/tests/test_comms_base.py:279 and mock_base.py document and check WRAPS_ON.

- File: `firmware/base/grappstar_base/base_logic.h:986`
- Severity: low (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: With the drum in REEL (after a tow or the step 15 reel-in, when the Jetson skips unwrap because of a mission retry or an operator click), retrieve is sent. The firmware forwards it to cmdReel(..., "seated"), which accepts it because collarReel() is true. The capstan then winds the dart into the cradle. Afterwards unwrap from REEL pulls the seated dart through the wraps, the D16 hazard. Reproduced in basesim: wrap, mode REEL, set line_m 1.5, retrieve -> ok, SEATED with mode REEL and wraps 7.

Evidence: base_logic.h:986 `else if (!strcmp(cmd, "retrieve")) {           // final approach until SW_SEATED (take-up in NEUTRAL, capstan in REEL)` then `err = cmdReel(a, m, "seated");`; base_logic.h:791 `if (!collarReel() && !neutral) return "NOT_WRAPPED";` (REEL passes). BASE_SPEC 10.1 step 17 `take-up final approach in NEUTRAL only ... Never seat on the capstan`.

Fix: In the retrieve branch, before cmdReel: `bool neutralTU = sec == SEC_NEUTRAL && !closed(SW_DOG_L) && !closed(SW_DOG_R) && !secBad; if (!neutralTU) err = sec == SEC_INVALID || secBad ? "POSITION_UNKNOWN" : "NOT_IN_NEUTRAL"; else err = cmdReel(...)`. Document NOT_IN_NEUTRAL as the retrieve refusal in PROTOCOL.md.

> Skeptic (confirmed): The literal claim holds. In base_logic.h:986-989 the `retrieve` branch goes straight to cmdReel(a, m, "seated") with no sector check. At line 791 cmdReel accepts the command when collarReel() is true, so `retrieve` in REEL starts the capstan (takeupOnly=false) and runs until SW_SEATED. BASE_SPEC 10.1 step 17 says `retrieve` is the "take-up final approach in NEUTRAL only ... Never seat on the capstan", and D35 says `retrieve` means take-up in NEUTRAL. The `tidy` branch right above does check the sector and returns NOT_IN_NEUTRAL; `retrieve` has no such check. So this is a real contract deviatio
>
> Skeptic (confirmed): The literal claim holds, but the scenario is overstated. base_logic.h:986-988 sends `retrieve` straight to cmdReel(a, m, "seated"), and cmdReel (line 791) accepts it whenever collarReel() is true. The comment even says "capstan in REEL". BASE_SPEC 10.1 is marked binding (D35), and its step 17 says `retrieve` is the take-up final approach "in NEUTRAL only ... Never seat on the capstan". So the firmware does not enforce the contract for this command. That deviation is real.

Why the severity should be low, not medium:
(1) The scenario given cannot happen through the shipped Jetson. In jetson/gra
>

## 14. base.py and mock_base.py SW_KEYS still use D (SW_DOG) and W=SW_WRAP_HOME; the firmware sends l, r, n, G and I (R1 #13 still open on the Jetson side)

- Now (472bb65): **partly fixed** at `jetson/grappstar/comms/base.py:54`. Fixed: mock_base.py:123 now has the full firmware key map (l, r, n, G, I, W=SW_ARM, no D), and the test imports that map. Still open: base.py:54-55 SW_KEYS still uses D=SW_DOG and W=SW_WRAP_HOME and has no l, r, n, G or I, so tel_contact('SW_DOG_L'/'SW_N20'/'SW_GAP'/'SW_INDEX') always returns False. The mission.py:55-61 sw_closed docstring still describes D and its long-name map has no n or G.

- File: `jetson/grappstar/comms/base.py:54`
- Severity: low (claimed low); unverified (low)
- Owner: jetson

Scenario: Real telemetry in LAUNCH is sw {a,l,n,H,G,S,W}, and in REEL it is {a,b,c,r,n,H,G,S,W} (basesim output). base.SW_KEYS has no l, r, n or G, so tel_contact('SW_DOG_L'/'SW_N20'/'SW_GAP') always returns False. test_comms_base.py:70 `set(t["sw"]) <= set(SW_KEYS)` passes only against the mock and fails on firmware telemetry. The mock never sends l, r, n or G, so hand mode (n open) and gap state cannot be tested.

Evidence: base.py:54-55 `SW_KEYS = {"a": ..., "D": "SW_DOG", "S": "SW_SEATED", "H": "SW_HOME", "L": "SW_RAIL_LOW", "W": "SW_WRAP_HOME"}`; mock_base.py:101-102 the same; mock_base.py:385 `out["D"] = 1`; config.h:17 `SW_TEL[SW_COUNT] = {"a", "b", "c", "l", "r", "n", "H", "G", "L", "S", "W", "I"}`; PROTOCOL.md:173-186.

Fix: In base.py and mock_base.py set `SW_KEYS = {"a":"SW_DRUM_A","b":"SW_DRUM_B","c":"SW_DRUM_C","l":"SW_DOG_L","r":"SW_DOG_R","n":"SW_N20","H":"SW_HOME","G":"SW_GAP","L":"SW_RAIL_LOW","S":"SW_SEATED","W":"SW_ARM","I":"SW_INDEX"}`. Make the mock send l in LAUNCH, r in REEL, n always and G at rest, and drop D. Update the mission.py sw_closed docstring and long-name map at :57.


## 15. BaseClient.reel() rejects until='slip', which 10.1 step 11 keeps as the tidy alternative

- Now (472bb65): **partly fixed** at `jetson/grappstar/comms/base.py:474-479`. The main failure is fixed. reel() now accepts until='slip' (line 478) and sends it to the firmware, so base.reel(0.3, until='slip') no longer raises ValueError. Still open: the docstring at lines 475-477 still says "needs drum_pos WRAPPED", "default limit_n 20" and "capped at 150", where PROTOCOL.md gives REEL or NEUTRAL/park take-up, limit_n 50 and a 70 cap (not the 110 the finding cites). There are still no tidy()/retrieve() helpers, and EVENTS/FAULTS (lines 43-46) still lack TIDIED and DART_STUCK.

- File: `jetson/grappstar/comms/base.py:469`
- Severity: low (claimed low); unverified (low)
- Owner: jetson

Scenario: The operator or mission code calls base.reel(0.3, until='slip') for the tidy alternative 10.1 allows. BaseClient raises ValueError before anything is sent, although the firmware accepts slip at NEUTRAL and at the park. The docstring also still says 'needs drum_pos WRAPPED' and gives limit_n 20 and a 150 cap (the firmware uses 50 and 110).

Evidence: base.py:469-470 `if until not in ("seated", "tension", "stop"): raise ValueError("until must be seated, tension or stop")`; base.py:466-468 docstring. base_logic.h:788 `!strcmp(u, "slip") ? U_SLIP`. BASE_SPEC 10.1 step 11 '`reel` until `slip` stays as an alternative'.

Fix: Accept 'slip' (`if until not in ("seated", "tension", "stop", "slip")`). Fix the docstring: REEL with the collar engaged, or take-up only at NEUTRAL or the park; default limit_n 50, cap 110. Add tidy(ms=3000, duty=0.3) and retrieve(duty=0.3, max_s=None) helpers, and add TIDIED to EVENTS and DART_STUCK to FAULTS.


## 16. Mock answers NOT_ARMED for an expired token where the firmware answers TOKEN_EXPIRED (R1-A #14 persists)

- Now (472bb65): **fixed** at `jetson/grappstar/comms/mock_base.py:1384`. The tick now sets token_expired=True when it drops the token (line 610), arm clears it (line 1147), and _cmd_fire returns TOKEN_EXPIRED when token is None and token_expired is set. This matches firmware base_logic.h:798 line for line. tests/test_comms_base.py:382-383 now expect TOKEN_EXPIRED on every fire after expiry until the next arm, which is also what the firmware does.

- File: `jetson/grappstar/comms/mock_base.py:507`
- Severity: low (claimed low); unverified (low)
- Owner: jetson

Scenario: arm, wait more than 3 s, then fire(token). The firmware sets tokenExpired in its tick and fire returns TOKEN_EXPIRED. The mock's tick sets self.token = None, so _cmd_fire returns NOT_ARMED, and its own TOKEN_EXPIRED branch is unreachable after a tick.

Evidence: mock_base.py:507-508 `if self.token is not None and self.sim_t - self.token_t > TOKEN_S: self.token = None`; mock_base.py:1144-1145 `if self.token is None: return "NOT_ARMED"`; firmware base_logic.h:695 `if (!tokenLive) return tokenExpired ? "TOKEN_EXPIRED" : "NOT_ARMED";`.

Fix: Add self.token_expired: set it True in the tick when the token expires, and in _cmd_fire return 'TOKEN_EXPIRED' when token is None and token_expired. Clear it on arm and on any fire attempt. Update the test that expects NOT_ARMED for an expired token.


## 17. Operator page 'Dart seated' row always shows '-': it reads base.seated / base.sw.seated, but telemetry sends sw.S (R1 #15 still open)

- Now (472bb65): **fixed** at `jetson/grappstar/app.py:188-189`. The status builder now does `"base": dict(tel, link=..., seated=bool((tel.get("sw") or {}).get("S")) if tel else None)`. That gives app.js a top-level base.seated, so pick(base, "seated") at app.js:481 (called through fillTable("t-base", st.base, BASE_FIELDS) at app.js:616) shows true or false, not '-', whenever telemetry is present.

- File: `jetson/grappstar/ui/static/app.js:22`
- Severity: low (claimed low); unverified (low)
- Owner: jetson

Scenario: The dart sits in the cradle, so firmware telemetry carries sw {..., S:1}. app.py passes the raw tel as status.base. pick(base, "seated") finds neither base.seated nor base.sw.seated, so the row shows '-' before the shot and after retrieve.

Evidence: app.js:19 `// [key, label]; "seated" is read from base.sw.seated`; app.js:22 `["seated", "Dart seated"]`; app.js:55 pick() looks in obj.sw[key]; app.py:188 `"base": dict(tel, link=self.base.link()),`; config.h:17 SW_TEL[SW_SEATED] = "S".

Fix: In app.py:188 use `"base": dict(tel, link=self.base.link(), seated=tel_seated(tel))` (import tel_seated from comms.base), or in app.js render that row from `!!(base.sw && base.sw.S)`.


## 18. systest c_tidy passes when TIDY runs before WRAP, so the system test cannot catch the D35 order violation

- Now (472bb65): **partly fixed** at `systest/harness.py:983 (C1_SEQ), systest/harness.py:1033 (c_c1_sequence), systest/harness.py:901 (c_tidy, unchanged), systest/harness.py:943 (c_seated)`. The order violation is now caught. nominal_checks() includes c_c1_sequence, whose regex requires `wrap > dart auto > tidy > mode REEL` (and c_d22_order is still there), so a mission that runs ANCHOR -> TIDY -> WRAP fails the systest. Two things remain. c_tidy still passes on "TIDY ran and was not skipped" and checks neither the order nor the TIDIED event. No retrieve check confirms that SEATED arrives in NEUTRAL with 0 wraps: c_seated only reads the plant's seated flag and switch S, and C1_SEQ makes `retrieve` optional, so seating on the capstan still passes.

- File: `systest/harness.py:626`
- Severity: low (claimed low); unverified (low)
- Owner: coordinator

Scenario: The mission runs ANCHOR -> TIDY -> WRAP (wrong under D35/D22). c_tidy checks only that a TIDY step ran and was not skipped, so the systest reports the tidy as passed. Nothing checks that WRAPPED came before TIDIED or that MODE REEL followed the tidy.

Evidence: harness.py:626-631 `def c_tidy(rig, res): ... ran = "TIDY" in res["steps"] ... return ran and not skipped, ...` (docstring 'TIDY after anchoring').

Fix: Check the order in c_tidy: index(WRAP step) < index(ANCHOR) < index(TIDY) < index of a MODE event with mode REEL, and the TIDIED event present. Add a c_retrieve check that SEATED arrives with tel mode NEUTRAL and wraps 0 (never on the capstan).


## Refuted (dropped)

- `jetson/grappstar/mission.py:799` unwrap() treats telemetry wraps null (unknown) as 0 and skips the unwrap when the drum is inside the WRAP sector
- `jetson/grappstar/mission.py:388` Mission never sends `mode REEL` after the wrap (D35 step 12), and its order is ANCHOR, TIDY, WRAP instead of WRAP, ANCHOR, TIDY. Every GO run hangs at CHECK on the real firmware (R1 #5/#9 persist)
- `jetson/grappstar/mission.py:329` A fault in TOW, CHECK or RETRIEVE sends the drum to the WRAPPED park, then Retry loops on NOT_WRAPPED with no 'Release the dart' choice, and the prompt wrongly says the drum went to NEUTRAL
- `jetson/grappstar/mission.py:482` Decock recovery dead-ends: Retry of COCK after a decock cocks with the aim unknown, then FIRE's re-aim is refused NOT_AT_REST. A fault mid-cock also suppresses the 'still COCKED' warning (R1 #7 partly persists)
- `jetson/grappstar/comms/mock_base.py:1174` mock_base still models the pre-D16 drum and keeps phase COCKING after a cock fault, so test_mission passes on paths the firmware refuses (R1 #4 persists, cock case new)
- `firmware/base/grappstar_base/base_logic.h:986` Firmware `retrieve` ignores max_s and raises DART_STUCK after a fixed 20 s, so the D35 no-go retrieve from full line out always faults
- `jetson/grappstar/mission.py:983` NO_GO reshoot path wraps the drum and then 'retrieves' at the WRAPPED park with reel until=stop, which the firmware runs as an unbounded take-up with line_m frozen (D35 step 7 says skip the wrap)
