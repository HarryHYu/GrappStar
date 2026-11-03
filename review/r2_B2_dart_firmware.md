# Round 2 B2: dart firmware re-review (fw-dart)

Reviewed at base-v1 commit ba121b9. Method: independent finder subagents, one per lens; every finding of
medium severity or above was checked by two skeptic subagents told to refute it; kept only if fewer than
two refuted it. Low findings are listed unverified. Severity is the finder's, lowered when both skeptics
agreed it is real but rated it lower.

Confirmed: 8. Refuted and dropped: 0.

Re-checked 14:10 Sydney against base-v1 863d58d, one checker subagent per finding: see the Now column.

| # | Sev | File:line | Owner | Finding | Now (863d58d) |
|---|---|---|---|---|---|
| 1 | medium | `firmware/dart/kedge_dart/kedge_logic.h:438` | fw-dart | Round 1 #2 is still open: a 'timeout' landing (after an impact, or with no impact at all) freezes the log so late that the launch, the flight and the impact have already been overwritten in the 12 s ring | fixed |
| 2 | medium | `jetson/grappstar/comms/dart.py:214` | jetson | Round 1 #4 is still open: Dart.flightlog gives up for good on the 409 'busy' refusal, so with auto-anchor-on-landing, or when the dart is still in FLIGHT, the shot's IMU log is never fetched | fixed |
| 3 | medium | `jetson/grappstar/comms/dart.py:214` | jetson | R1 #4 not fixed: Jetson drops the flight log for good on the dart's 409 'busy' /flightlog refusal (every shot with auto_anchor_on_landing, and whenever the dart is still in FLIGHT) | fixed |
| 4 | low | `firmware/dart/API.md:236` | fw-dart | API.md fault table still says a step timer ending during a failed current read raises SEQ_TIMEOUT, which the R1 #3 fix removed | fixed |
| 5 | low | `firmware/dart/kedge_dart/kedge_dart.ino:322` | fw-dart | New keep=1 key-release stop aborts a running /auto or /release sequence when a refused key is let go (or the page loses focus) | still open |
| 6 | low | `firmware/dart/kedge_dart/kedge_logic.h:410` | fw-dart | Round 1 #1 only partly fixed: reversing a motor back and forth still gets round the hold cap and stall detection, so a jammed motor can run with no limit | still open |
| 7 | low | `firmware/dart/test/fuzz_logic.cpp:27` | fw-dart | Fuzz test checks only JSON syntax for the flight log and a copy of phase() for phases, so the freeze window bug cannot fail it | still open |
| 8 | low | `jetson/grappstar/cli.py:179` | jetson | R1 #5 not fixed: CLI 'dart release' prints IDLE (success) when the release was aborted by /stop | fixed |

## 1. Round 1 #2 is still open: a 'timeout' landing (after an impact, or with no impact at all) freezes the log so late that the launch, the flight and the impact have already been overwritten in the 12 s ring

- Now (863d58d): **fixed** at `firmware/dart/kedge_dart/kedge_logic.h:478-489 (stop() path at :382)`. LAUNCH now schedules a freeze at freezeCap() = tLaunch-2000+(LOG_N-1)*20, and the LAND and stop() paths cap their freeze time to the same value, so the ring can no longer overwrite the launch. The new test_logic case for a timeout landing after an impact passes (log 2985..14965, tLaunch 5000), and all 224 checks pass.

- File: `firmware/dart/kedge_dart/kedge_logic.h:438`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-dart

Scenario: Reproduced at HEAD with /tmp/claude-0/review-scratch-B2/r2.cpp (5 ms ticks, armed log-only). (a) Launch at t=5000 (8 g for 40 ms), 1.5 s of free fall at 0.3 g, then a 6 g impact. After that the dart is nudged to 1.4 g for 40 ms every 800 ms (tether tug or grass), so the 1 s stillness never completes. The output is how=timeout, tLand=6545, frozen=1, oldest sample 8020, newest 20000, 'launch sample in log: NO'. (b) The same launch with no impact, swinging to 1.5 g for 100 ms every 700 ms, so the dart is never still. The output is tLand=20000, freeze at 23000, oldest sample 11020, launch not in the log. Both runs give the same result with base=0xFFFFE000 (across the millis() wrap), so wrap is not the cause. /flightlog then returns 600 samples of the dart lying on the ground, with no launch and no impact. This breaks C2 ('frozen from 2 s before launch detection until 3 s after landing'), and vision/flight.py finds no launch or impact.

Evidence: kedge_logic.h:104 `flightTimeoutMs = 15000`; :181 `static const int LOG_N = 600;` (12 s at 50 Hz); :123 `return land(now, "timeout");`; :138 `return land(tImpact, "timeout");`; :433 on LAUNCH only `log.window(flight.tLaunch - 2000);`, no freeze scheduled; :438 `uint32_t fr = flight.tLand + 3000; log.scheduleFreeze((int32_t)(fr - now) > 0 ? fr : now);`. This code is unchanged since 0bfd59a; the diff does not touch handleFlight or Flight.

Fix: Stop the ring from overwriting the start of the window. On LAUNCH (after line 433) add `log.scheduleFreeze(flight.tLaunch - 2000 + (LOG_N - 1) * 20);`. On LAND (line 438) use `uint32_t cap = flight.tLaunch - 2000 + (LOG_N - 1) * 20; uint32_t fr = flight.tLand + 3000; if ((int32_t)(fr - cap) > 0) fr = cap; log.scheduleFreeze((int32_t)(fr - now) > 0 ? fr : now);`. In stop() (line ~341) apply the same cap to `now + 3000`. Add a test_logic case for scenario (a) that requires `(int32_t)(K.log.at(0).t - K.flight.tLaunch) <= 0` after the timeout landing.

> Skeptic (confirmed): I could not refute this; the defect is real at HEAD. Checked in firmware/dart/kedge_dart/kedge_logic.h:
- Flight::update ends a flight with "timeout" only once now - tLaunch >= 15000. In IMPACTED it then returns land(tImpact, "timeout"). Flight::poll does the same. Any reading between stillHi (1.15 g) and impactG (3.5 g) resets stillOn, so the 1 s stillness never completes.
- handleFlight on LAUNCH only calls log.window(tLaunch - 2000), which is a read-side filter and schedules no freeze. On LAND it calls scheduleFreeze(max(tLand + 3000, now)). For a timeout landing, now is tLaunch + 15000, so
>
> Skeptic (confirmed): I could not refute this. The code does what the claim says.

The code: Flight::update and poll (kedge_logic.h:123, :138, :151) only declare a 'timeout' landing once now - tLaunch >= 15000. handleFlight on LAUNCH (:433) only calls log.window(tLaunch-2000) and schedules no freeze. On LAND (:438) the freeze goes at max(tLand+3000, now). After an impact, tLand = tImpact, which is far in the past, so the freeze happens at once at tLaunch+15000. With no impact, tLand = now, so the freeze happens at tLaunch+18000. FlightLog is a 600-slot ring at 50 Hz (12 s) and offer() keeps overwriting until frozen
>

## 2. Round 1 #4 is still open: Dart.flightlog gives up for good on the 409 'busy' refusal, so with auto-anchor-on-landing, or when the dart is still in FLIGHT, the shot's IMU log is never fetched

- Now (863d58d): **fixed** at `jetson/grappstar/comms/dart.py:202-215; jetson/grappstar/mission.py:906-917`. Dart.flightlog now retries a DartRefused whose reason starts with "busy" (the firmware sends "refused: busy (motors running, armed or in flight)" at firmware/dart/kedge_dart/kedge_dart.ino:275), backing off for up to 45 s. Before fetching, Mission.landing_zone also calls wait_phase for LANDED/ANCHORED/IDLE/FAULT/ARMED with a 40 s timeout, so a dart that is still in FLIGHT or running auto-anchor is waited out instead of dropping the log.

- File: `jetson/grappstar/comms/dart.py:214`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: Mission with auto_anchor_on_landing=True: on landing the dart starts /auto by itself (kedge_logic.h handleFlight -> startAuto), so seqRunning() stays true for up to 35 s. Mission.landing_zone calls self.dart.flightlog() right after the landing wait. The firmware answers 409 'refused: busy (motors running, armed or in flight)', Dart.flightlog re-raises DartRefused without retrying, and landing_zone logs 'no flightlog' and goes on with fl=None. Nothing fetches the log later. The same happens when the dart is still in FLIGHT, waiting for its 15 s timeout.

Evidence: dart.py:214-215 `except DartRefused:` / `raise`; mission.py:687 `fl = self.dart.flightlog()` with no wait_phase before it; mission.py:698-699 `except DartError as e: self.log.warn(f"no flightlog: {e}")`; kedge_dart.ino:257 `bool busy = K.running() || K.seqRunning() || K.armed || K.flight.flying();`

Fix: In Dart.flightlog, treat a DartRefused whose reason starts with 'busy' as retryable: sleep 1 s and try again until a deadline of about 40 s, re-raising only after that. In Mission.landing_zone, first call `self.dart.wait_phase(("LANDED","ANCHORED","IDLE","FAULT"), 40.0, cancel=self.cancel)` (catch its timeout and carry on), then fetch the log.

> Skeptic (confirmed): I checked this against the code and it holds up.

1. In jetson/grappstar/comms/dart.py:214-215, `flightlog` re-raises DartRefused at once, with no retry. dart.py:98-103 turns any HTTP 409 into DartRefused.

2. The firmware (firmware/dart/kedge_dart/kedge_dart.ino:257-258) answers /flightlog with 409 "refused: busy" whenever any of these is true: K.running(), K.seqRunning(), K.armed or K.flight.flying().

3. In kedge_logic.h:440-442, `handleFlight` runs startAuto on landing when armed and autoOnLanding are both set. That makes seqRunning() true until the anchor sequence ends, which can take up 
>
> Skeptic (confirmed): The defect is real. I checked the code and could not refute it.

1. The firmware refuses while busy. In firmware/dart/kedge_dart/kedge_dart.ino:257-258, handleFlightlog returns 409 "refused: busy" whenever K.running(), K.seqRunning(), K.armed or K.flight.flying() is true.

2. Auto-anchor keeps it busy. In kedge_logic.h handleFlight (around lines 436-442), a landing clears `armed` and then calls startAuto when armed && autoOnLanding. While that runs, seqRunning() stays true for up to autoMaxMs = 35000 ms.

3. Mission arms with that setting. Mission.arm (mission.py:523) calls dart.arm(True, auto
>

## 3. R1 #4 not fixed: Jetson drops the flight log for good on the dart's 409 'busy' /flightlog refusal (every shot with auto_anchor_on_landing, and whenever the dart is still in FLIGHT)

- Now (863d58d): **fixed** at `jetson/grappstar/mission.py:909`. Both parts of the fix are now in. landing_zone waits up to 40 s for the dart to reach LANDED, ANCHORED, IDLE, FAULT or ARMED before it calls flightlog(), and it disarms the dart if it is still ARMED. Dart.flightlog (comms/dart.py:202-215) now retries a 409 whose reason starts with "busy" for up to busy_wait_s=45 s, and the reason string is stripped of the "refused: " prefix at dart.py:104, so the check matches the firmware text.

- File: `jetson/grappstar/comms/dart.py:214`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: Case 1: MissionConfig.auto_anchor_on_landing=True, so Mission.arm sends /arm?on=1&auto=1 (mission.py:523). On landing the firmware starts the anchor sequence itself, so seqRunning() stays true for up to 35 s. Mission.landing_zone calls dart.flightlog() right after wait_landing + settle_s (mission.py:659, 687). The firmware answers 409 'refused: busy (motors running, armed or in flight)'. Dart.flightlog re-raises DartRefused at once, with no wait and no retry. landing_zone catches it as DartError and logs 'no flightlog', so there is no flightlog_shotN.json and no IMU data for analyse(). Nothing fetches the frozen log later. Case 2 (auto=0): line and video say landed, but the dart is still in FLIGHT (it rocks on the tether and is never still for 1 s, so it waits for its 15 s timeout). The fetch is refused the same way. Neither dart.py nor mission.py changed since 0bfd59a, and firmware handleFlightlog still refuses on the same conditions.

Evidence: jetson/grappstar/comms/dart.py:214-215 `except DartRefused:` / `raise`
firmware/dart/kedge_dart/kedge_dart.ino:257-258 `bool busy = K.running() || K.seqRunning() || K.armed || K.flight.flying(); ...` `if (busy) { web.send(409, "text/plain", "refused: busy (motors running, armed or in flight)\n"); return; }`
jetson/grappstar/mission.py:687 `fl = self.dart.flightlog()` (no wait on the dart phase first); mission.py:698-699 `except DartError as e:` `self.log.warn(f"no flightlog: {e}")`

Fix: In Mission.landing_zone, before calling flightlog(), wait for a phase in which the dart serves the log: `try: self.dart.wait_phase(("LANDED","ANCHORED","IDLE","FAULT"), 40.0, fail=(), cancel=self.cancel) except DartError: pass`. 40 s covers the 15 s flight timeout and the 35 s auto-anchor limit. Also, in Dart.flightlog, retry a DartRefused whose reason starts with 'busy': sleep 1 s and retry until a deadline (a `busy_wait_s=40` parameter), and re-raise only when the deadline passes. Replace the bare `except DartRefused: raise`.

> Skeptic (confirmed): I could not refute this. I checked the code and the defect is real.

- jetson/grappstar/comms/dart.py:214-215: Dart.flightlog re-raises DartRefused (the HTTP 409 subclass of DartError, lines 35 and 98-103) at once, with no wait and no retry.
- firmware/dart/kedge_dart/kedge_dart.ino:257-258: handleFlightlog answers 409 "busy" whenever K.running() || K.seqRunning() || K.armed || K.flight.flying().

Case 1 is concrete. auto_anchor_on_landing=True makes Mission.arm send auto=1 (mission.py:523). On landing, kedge_logic.h:440-442 sets go = armed && autoOnLanding and calls startAuto, so seqRunning()
>
> Skeptic (confirmed): I checked the claim against the code and it holds, mainly for Case 1.

Case 1 (auto_anchor_on_landing=True):
- Mission.arm sends arm(True, auto=cfg.auto_anchor_on_landing) (mission.py:523).
- In the firmware, landing runs startAuto(now) straight away whenever armed && autoOnLanding and the landing was not a fallback (kedge_logic.h:440-442). So seqRunning() goes true at once.
- The anchor sequence lasts many seconds. Its limits include kickMaxMs=15000, deployMaxMs=6000 and drillAfterMs=3000.
- wait_landing breaks as soon as the dart reports LANDED, ANCHORING or ANCHORED (mission.py:644, 648). I
>

## 4. API.md fault table still says a step timer ending during a failed current read raises SEQ_TIMEOUT, which the R1 #3 fix removed

- Now (863d58d): **fixed** at `firmware/dart/API.md:255`. The SEQ_TIMEOUT row now reads only "anchor or release sequence over 35 s", with no failed-current-read clause. KICK_TIMEOUT (line 254) still keeps its kick-timer clause, and line 157 agrees with both.

- File: `firmware/dart/API.md:236`
- Severity: low (claimed low); unverified (low)
- Owner: fw-dart

Scenario: The R1 #3 fix changed the !curOk branch of tick(). A sequence motor timer that ends on a tick with a failed INA219 read is now a normal step end (drive off; the sequence moves on at the next good read). Only the kick's limits in K_SWING/K_PLANT/R_KICK fault. API.md, the C2 owner doc that the Jetson team writes its fault handling against, still lists the old cause under SEQ_TIMEOUT, and line 141 of the same file says the opposite ('One failed current read at the moment a step ends does not fault'). A client or operator who sees SEQ_TIMEOUT will look for I2C read failures that no longer cause it, instead of the real cause: the 35 s sequence limit.

Evidence: firmware/dart/API.md:236 `| SEQ_TIMEOUT | anchor or release sequence over 35 s, or a sequence motor timer ran out while current reads were failing | all off |`
firmware/dart/API.md:141 `- One failed current read at the moment a step ends does not fault: the step ends and the next good read moves on.`
firmware/dart/kedge_dart/kedge_logic.h:475-478 `for (int m = 0; m < NMOT; m++) if (dir[m] && (int32_t)(now - until[m]) >= 0) {` / `if (m == KICK && (seq == K_SWING || seq == K_PLANT || seq == R_KICK)) { setFault(F_KICK_TIMEOUT, ...); return; }` / `drive(m, 0, 0, now);`

Fix: API.md:236: change the SEQ_TIMEOUT cause to `anchor or release sequence over 35 s` and delete the 'or a sequence motor timer ran out while current reads were failing' clause. Keep line 235 (KICK_TIMEOUT 'a kick timer ran out while current reads were failing'), which is still true for the kick.


## 5. New keep=1 key-release stop aborts a running /auto or /release sequence when a refused key is let go (or the page loses focus)

- Now (863d58d): **still open** at `firmware/dart/kedge_dart/kedge_dart.ino:344-345`. The keep=1 key-release branch still calls K.stop(t, false). In kedge_logic.h:367-387 that call still sets seq = ABORTED when a sequence is running and clears fault to F_NONE. The page's release() and lost() at kedge_dart.ino:252/256 still send that request even when the key was refused, and there is no keyRelease-style helper like linkLost (kedge_logic.h:468).

- File: `firmware/dart/kedge_dart/kedge_dart.ino:322`
- Severity: low (claimed medium); 1 of 2 skeptics confirmed
- Owner: fw-dart

Scenario: Auto-anchor starts on landing, or the Jetson sends /auto, so the dart is ANCHORING. On the dart web page the operator taps or holds a run key (for example '3'). The firmware refuses it with 'busy', so no motor command is accepted. On keyup, release() sends /key?c=%20&keep=1. lost() sends the same thing on blur, pagehide or visibilitychange, for example when the phone switches apps while a key is held. The handler calls K.stop(now, false), which sets seq = ABORTED and stops every motor. The anchor ends half done: kick part-way, foot or drills not planted, phase LANDED/IDLE with seq 'aborted'. Mission.anchor's wait_phase then fails. A refused key press has caused a sequence abort. Scratch proof /tmp/claude-0/review-scratch-B2/keep.cpp: 'before: phase=ANCHORING seq=kick swing kick=1 / key 3 -> busy / after keyup stop: phase=IDLE seq=aborted kick=0'. This goes against the intent in test_logic.cpp ('a key tapped in FLIGHT and its keyup stop (disarm=0) do not cancel auto on landing'): the same tap one phase later cancels the anchor. The same path also clears any latched fault (stop() sets fault = F_NONE) just because a finger was lifted.

Evidence: kedge_dart.ino:235 `function release(id){...if(!held.size)send(' ',1)}   // keep=1: stop the motors, stay armed`
kedge_dart.ino:322-323 `if ((c == " " || c == "s") && web.arg("keep") == "1") { xSemaphoreTake(mtx, portMAX_DELAY); K.stop(millis(), false); ...`
kedge_logic.h:337-339 `bool wasSeq = seqRunning(); stopAll(now); if (wasSeq) seq = ABORTED;`
kedge_logic.h:344 `fault = F_NONE;`
kedge_logic.h:395 `if (seqRunning()) return "busy";` (the key itself was refused)

Fix: A key release should stop only the manual hold-to-run motors, as linkLost does. Add to Kedge: `void keyRelease(uint32_t now) { if (!seqRunning() && running()) { stopAll(now); ev("keys released: motors stopped"); } }` (no seq change, no fault clear). In the keep=1 branch at kedge_dart.ino:323, call `K.keyRelease(millis())` instead of `K.stop(millis(), false)`. An explicit STOP (space, the STOP button, /stop) still aborts. Update API.md:98 to match, and add a test: startAuto, key('3') refused, keyRelease, then require seq still anchoring.

> Skeptic (refuted): The code does what the claim says. A refused run key that is released, or a page that loses focus while a key is held, sends /key?c=%20&keep=1. That calls K.stop(now,false), which aborts a running sequence and clears the fault (kedge_logic.h stop(): wasSeq -> seq=ABORTED, fault=F_NONE). But it is not a defect under the rules I was given to refute against:

1) It is documented. firmware/dart/API.md:98 says keep=1 is "like /stop?disarm=0: the web page sends this when the last held key is released or the page loses focus". API.md:116 says /stop?disarm=0 "stops the motors, aborts any sequence and 
>
> Skeptic (confirmed): The mechanism is real. In firmware/dart/kedge_dart/kedge_dart.ino the page's release() and lost() send /key?c=%20&keep=1 once the last held key is gone. The keep=1 branch (lines 322-323) calls K.stop(millis(), false). Kedge::stop (kedge_logic.h:336-347) records wasSeq, calls stopAll, sets seq = ABORTED and fault = F_NONE whatever disarm is. key() refuses run keys with "busy" while a sequence runs (line 395), but nothing in the page or the handler checks whether the held key was accepted. So: hold '3' during ANCHORING, get busy, let go, and the anchor is aborted. The reviewer's scratch program 
>

## 6. Round 1 #1 only partly fixed: reversing a motor back and forth still gets round the hold cap and stall detection, so a jammed motor can run with no limit

- Now (863d58d): **still open** at `firmware/dart/kedge_dart/kedge_logic.h:456`. Line 456 still reads `cont = repeat && lastKeyDir[m] == d`, so every reversal resets heldStart and motT0. The bothLock at line 455 only fires while the motor is still moving the other way. I rebuilt the original alt.cpp against the current header (in /tmp/claude-0/review-scratch-stale/s152/) and it gives the same numbers as before: deploy driven 25905 of 30000 ms with no refusals and no stall events, with or without the INA219, and 28185 ms at period 420 with ms=400.

- File: `firmware/dart/kedge_dart/kedge_logic.h:410`
- Severity: low (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-dart

Scenario: The pods are jammed, so the deploy motor draws stall current (185 mA) in both directions. The operator rocks them by pressing deploy down, then deploy up, and so on, each press lasting under about 450 ms. The page sends a stop on each keyup, and API.md:95 says this is how to reverse. A script sending '1' and '!' alternately every 400 ms with ms=350 does the same. The new bothLock only fires when the opposite key arrives while dir[m] == -d, that is, while the motor is still moving. Here the motor has always stopped first, either from the keyup stop or from its 350 ms deadman, so the lock never fires. cont is false on every flip, which resets heldStart[m], so the 8 s cap (2 s without the INA219) is never reached. drive() also resets motT0[m] on every flip, so the manual stall check needs 250 ms surge-ignore plus 200 ms stallHoldMs = 450 ms of continuous running, which a 350 ms run never gives. Scratch proof /tmp/claude-0/review-scratch-B2/alt.cpp, which drives the real kedge_logic.h over 30 s: 'period=400 ms=350: deploy driven 25905 of 30000 ms at stall current, refused=0, stall events=0'. The run with no INA219 gives the same 25905 ms, against a 2 s cap. With period 420 and ms=400 (the most the Jetson app sends) it gives 28185 ms. The N20 sits at stall or reversal current for as long as the alternation goes on.

Evidence: kedge_logic.h:410 `if (repeat && lastKeyDir[m] == -d && dir[m] == -d) { bothLock[m] = true; ...`  (needs the motor still moving in the other direction)
kedge_logic.h:411 `bool cont = repeat && lastKeyDir[m] == d; lastKeyDir[m] = d;`
kedge_logic.h:412 `if (!cont) heldStart[m] = now;`
kedge_logic.h:318 `if (d && dir[m] != d) motT0[m] = now;`
kedge_logic.h:497 `if (now - motT0[m] < tu.surgeIgnoreMs) settled = false;`
API.md:95 `To reverse, release (the page then sends a stop) and press the other.`

Fix: Base the hold cap on the motor, not on its direction. In key() change line 411 to `bool cont = repeat; lastKeyDir[m] = d;`. A reversal inside the repeat window then keeps heldStart[m], so all the reversals together use up one manualMaxMs / manualMaxNoInaMs budget. Once the cap is reached, line 414 sets stallLock for whichever direction is pressed, so both directions get refused until the keys have been released for a full window. Keep the bothLock line for the simultaneous case. Add a test_logic case: alternate '1' and '!' every 400 ms with ms=350 for 20 s and require the deploy motor's total on-time to be <= 8000 ms (<= 2000 ms with inaOk=false).

> Skeptic (confirmed): The claim holds up against the code. In kedge_logic.h key() (lines 409-420), bothLock only fires when `dir[m] == -d`, meaning the motor is still running the other way. If the motor has already stopped (from the keyup stop or from its own runMs/deadman expiry), lastKeyDir[m] != d makes cont false. heldStart[m] is then reset to now, so the 8000/2000 ms cap at line 413 never builds up. drive() at line 318 also resets motT0[m] whenever the direction changes, and the tick stall check (line 497: settled needs now-motT0 >= surgeIgnoreMs 250, then heldT for stallHoldMs 200) needs about 450 ms of conti
>
> Skeptic (confirmed): The claim checks out against the current code. kedge_logic.h:410 only sets bothLock when dir[m] == -d, which means the motor must still be running the other way. Line 411 makes cont false on any change of direction, so line 412 resets heldStart[m]. drive() at line 318 resets motT0[m] whenever the direction changes, and line 417 only puts the old value back when cont is true. The manual stall check in tick() needs about 450 ms of continuous running (surgeIgnoreMs 250 + stallHoldMs 200). A 350 ms deadman run, or a run ended by the keyup stop, never gets that far. API.md:95 (firmware/dart/API.md)
>

## 7. Fuzz test checks only JSON syntax for the flight log and a copy of phase() for phases, so the freeze window bug cannot fail it

- Now (863d58d): **still open** at `firmware/dart/test/fuzz_logic.cpp:516`. The only change to fuzz_logic.cpp since ba121b9 is the motor-limit slack and a disarm before /auto. checkLogJson (lines 516-541) still checks only JSON validity, n, field count and time order, with no check of the tLaunch-2000..tLand+3000 window. apiPhase (325-334) still copies phase() line for line, and imuAcc still has no impact-then-nudges or never-still tumbling trace.

- File: `firmware/dart/test/fuzz_logic.cpp:27`
- Severity: low (claimed low); unverified (low)
- Owner: fw-dart

Scenario: The fuzz run reports 'FUZZ PASS' with 0 violations on invariants 7 and 8 (`--quick`, 323896 flightlog/status checks) while Round 1 #2 is still in the code (finding above, reproduced by r2.cpp). Invariant 8 (checkLogJson, lines 515-541) only checks that the JSON is valid, that n matches the sample count, that each sample has 8 fields and that samples are in time order. It never checks that a frozen log of a launched flight covers tLaunch-2000 .. tLand+3000, or even contains tLaunch. Invariant 7 compares K.phase() with apiPhase() (lines 324-334), which repeats phase()'s code line for line (kedge_logic.h:248-257), so it can only fail on a typo. The IMU generator (imuAcc, lines 586-612) has no trace where post-impact nudges or a never-still swing run the flight into the 15 s timeout, so the failing case is never generated either.

Evidence: fuzz_logic.cpp:27 `//   8 every status() and LogJson output is valid JSON (own validator below)`; :536 `if (cnt != nn) why = "n does not match the sample count"; else if (!width) ... else if (!order) ...` (no window or launch check); :332 `if (K.landed && (K.seq == IDLE || K.seq == ABORTED) && !motor) return P_LANDED;` is the same as kedge_logic.h:254 `if (landed && (seq == IDLE || seq == ABORTED) && !running()) return P_LANDED;`

Fix: Add a log window invariant to checkLogJson. When `K.log.frozen && K.flight.launched && T.bno` and the IMU was fresh across the launch, require `(int32_t)(K.log.at(0).t - (K.flight.tLaunch - 2000)) <= 20`, and require the newest sample to be at or after `min(tLand + 3000, tLaunch - 2000 + 11980) - 40`. Add IMU kinds 'impact then 1.4 g nudges every 800 ms' and 'tumbling at 1.5 g, never still' to imuAcc so the timeout landing paths are exercised. For invariant 7, take the expected phase from the API table driven by the test's own event record (last command, landing seen) rather than from K's fields in the same order as phase().


## 8. R1 #5 not fixed: CLI 'dart release' prints IDLE (success) when the release was aborted by /stop

- Now (863d58d): **fixed** at `jetson/grappstar/cli.py:179`. The release command now calls wait_phase(("IDLE",), 90, want_seq="released"). Because want_seq is set, a seq of "aborted" raises DartError (comms/dart.py:303-304), and an IDLE with any other seq is not accepted, so it keeps polling until the 90 s timeout. cmd_dart catches the error and exits with code 2, so a release stopped by /stop no longer prints IDLE as if it had succeeded.

- File: `jetson/grappstar/cli.py:179`
- Severity: low (claimed low); unverified (low)
- Owner: jetson

Scenario: Bench: the dart has not landed since its last arm (landed=false). Run `python3 -m grappstar dart release`, then press STOP on the web page (or send /stop) mid-sequence. The firmware sets seq=aborted, and phase() returns IDLE because the LANDED branch needs landed=true. wait_phase checks `s['phase'] in phases` before it checks for 'aborted', so it returns at once and the CLI prints 'IDLE'. The operator reads that as released while the drills, foot or kick are part-way. Since the new phase() rule, any manual motor still running after a landing also shows IDLE, so a 'release' shows IDLE in that case too.

Evidence: jetson/grappstar/cli.py:178-179 `d.release()` / `print(d.wait_phase(("IDLE",), 90, log=print)["phase"])`
jetson/grappstar/comms/dart.py:273 `if s["phase"] in phases and (want_seq is None or s.get("seq") == want_seq): return s` (before the aborted check at 277-280)
firmware/dart/kedge_dart/kedge_logic.h:254-256 `if (landed && (seq == IDLE || seq == ABORTED) && !running()) return P_LANDED; if (armed) return P_ARMED; return P_IDLE;`
(Mission.release_dart at mission.py:887-888 correctly passes want_seq="released".)

Fix: cli.py:179: `print(d.wait_phase(("IDLE",), 90, log=print, want_seq="released")["phase"])`. With want_seq set, wait_phase raises on seq 'aborted' (dart.py:279-280) instead of reporting IDLE.

