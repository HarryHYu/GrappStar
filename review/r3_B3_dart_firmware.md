# Round 3 B3: dart firmware on the w/fw-dart tip

Reviewed at base-v1 commit 258001f. Method: independent finder subagents, one per lens; every finding of
medium severity or above was checked by two skeptic subagents told to refute it; kept only if fewer than
two refuted it. Low findings are listed unverified. Severity is the finder's, lowered when both skeptics
agreed it is real but rated it lower.

Confirmed: 7. Refuted and dropped: 0.

| # | Sev | File:line | Owner | Finding |
|---|---|---|---|---|
| 1 | medium | `firmware/dart/kedge_dart/kedge_logic.h:472` | fw-dart | R2-B2 #6 still open, and worse than reported: reversing a motor resets the manual hold cap and the stall timer, so a jammed motor can run at stall current with no time limit (with or without the INA219) |
| 2 | low | `firmware/dart/API.md:209` | fw-dart | API.md contradicts itself on arming from ANCHORED: one line says it is refused, the next says it starts a new shot |
| 3 | low | `firmware/dart/kedge_dart/kedge_dart.ino:275` | fw-dart | Firmware sends one 409 reason ('busy (motors running, armed or in flight)') for every /flightlog refusal, so the client's busy-retry also spins up to 45 s on an armed dart; the mock sends 'armed'/'in flight' instead |
| 4 | low | `firmware/dart/kedge_dart/kedge_dart.ino:345` | fw-dart | R2-B2 #5 still open: the web page's key-release stop (/key?c=%20&keep=1) aborts a running /auto or /release when a refused key is let go, and clears any latched fault |
| 5 | low | `firmware/dart/test/fuzz_logic.cpp:538` | fw-dart | Removing the landing-time freeze cap still passes test_logic and fuzz: no test covers a landing 7 to 10 s after launch, and the fuzz still has no window invariant |
| 6 | low | `jetson/grappstar/comms/dart.py:264` | jetson | Dart.hold() sends /stop?disarm=0 after a refused key, which aborts a running /auto or /release and clears a latched fault |
| 7 | low | `jetson/grappstar/comms/dart.py:267` | jetson | Dart.hold always sends /stop?disarm=0 in its finally block, even when every key was refused, so `grappstar dart key C --hold S` aborts a running anchor or release and clears a latched fault |

## 1. R2-B2 #6 still open, and worse than reported: reversing a motor resets the manual hold cap and the stall timer, so a jammed motor can run at stall current with no time limit (with or without the INA219)

- File: `firmware/dart/kedge_dart/kedge_logic.h:472`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-dart

Scenario: The deploy worm is jammed and draws stall current (185 mA) in both directions. Someone sends '1' and '!' alternately with ms=350, either an operator rocking the motor to free it or a script. My scratch test (/tmp/claude-0/review-scratch-B3/b3.cpp, linked against the real kedge_logic.h at 258001f) ran this for 30 s. With a press every 400 ms, deploy was driven for 25875 of 30000 ms. With a press every 300 ms the motor reverses without stopping in between and was driven for 30000 of 30000 ms. In every case there were 0 refusals and 0 stall events. With the INA219 dead (inaOk=false, nothing to detect a stall), the results are the same: 25875 and 30000 ms, against the documented 2 s cap. The bothLock at line 471 needs a same-direction repeat inside the window (dirRepeat), and strict alternation never produces one. So cont is false on every press and heldStart is reset each time. drive() resets motT0 on every direction change, so the manual stall check (surgeIgnoreMs 250 + stallHoldMs 200) never completes. The round-1 regression test (test_logic.cpp:476) only covers a 75 ms alternation, and the test at line 563 deliberately allows a reversal with no stop.

Evidence: kedge_logic.h:471 `if (repeat && dirRepeat && otherRecent && lastKeyDir[m] == -d && dir[m] == -d) { bothLock[m] = true; ...`
kedge_logic.h:472 `bool cont = repeat && lastKeyDir[m] == d; lastKeyDir[m] = d;`
kedge_logic.h:473 `if (!cont) heldStart[m] = now;`
kedge_logic.h:474-475 `uint32_t cap = inaOk ? tu.manualMaxMs : tu.manualMaxNoInaMs, used = now - heldStart[m]; if (used >= cap) { stallLock[m] = d; ...`
kedge_logic.h:358 `if (d && dir[m] != d) motT0[m] = now;`
kedge_logic.h:480 `if (cont) motT0[m] = keepT0;`
Scratch output: `A1: ina=ok period=300 ms=350 alternate 1/!: deploy driven 30000 of 30000 ms at stall current, refused=0` and `A3: ina=dead period=300 ... driven 30000 of 30000 ms ... refused=0`

Fix: Count the hold cap per motor, not per direction, but keep the surge-ignore restart on a reversal. Line 472 becomes `bool cont = repeat, sameDir = repeat && lastKeyDir[m] == d; lastKeyDir[m] = d;` and line 480 becomes `if (sameDir) motT0[m] = keepT0;`. Line 473 stays `if (!cont) heldStart[m] = now;`. Once the cap is reached, line 475 locks each direction as it is pressed, and both stay refused while presses keep arriving inside the window. Verified on a patched copy: the same runs now give 6900 and 7995 ms with the INA219 and 1725 and 1995 ms without it, and all 234 test_logic checks still pass. Add a test_logic case that alternates '1' and '!' every 300 ms and every 400 ms with ms=350 for 30 s, with inaOk true and false, and require the deploy on-time to stay at or under manualMaxMs or manualMaxNoInaMs plus one run.

> Skeptic (confirmed): I could not refute this. One thing to know first: the local w/review HEAD (b45752b) is older than origin/base-v1. Its key() has no dirRepeat/otherRecent terms, so its line numbers do not match the claim. The origin/base-v1 tip (which contains 258001f) does match the claim word for word: kedge_logic.h:470-480 has the otherRecent/dirRepeat bothLock condition, `cont = repeat && lastKeyDir[m] == d`, `if (!cont) heldStart[m] = now`, and `if (cont) motT0[m] = keepT0`. drive() at line 358 resets motT0 whenever the direction changes.

I extracted origin/base-v1's firmware/dart into /tmp/claude-0/revie
>
> Skeptic (confirmed): I could not refute this. Note that w/review HEAD (b45752b) has an older kedge_logic.h. The cited code, with dirRepeat and otherRecent at line 471, is in origin/base-v1 (0ab7a5a, which contains 258001f), so I checked against that version.

Reproduced:
- I extracted origin/base-v1 firmware/dart/kedge_dart into /tmp/claude-0/review-scratch-B3/skB and built the reviewer's b3.cpp against the real kedge_logic.h.
- Output: with the INA219 working, deploy was driven 25875 ms of 30000 at a 400 ms period and 30000 of 30000 at 300 ms. With the INA219 dead the figures were the same. There were 0 refusals 
>

## 2. API.md contradicts itself on arming from ANCHORED: one line says it is refused, the next says it starts a new shot

- File: `firmware/dart/API.md:209`
- Severity: low (claimed low); unverified (low)
- Owner: fw-dart

Scenario: A client author goes by line 209 and sends /arm while the dart is ANCHORED, for example a reshoot without a release. The firmware refuses with 409 `anchored: release first`, as line 204 and the code say. A client written to line 209 then fails to arm, so it has no flight log and no landing detection for that shot.

Evidence: API.md:204 `- Refused while ANCHORED (anchored: release first; the legs are out)...`
API.md:209 `- Arming from ANCHORED, aborted or released starts a new shot (seq back to idle).`
kedge_logic.h:425 `if (seq == DONE) return "anchored: release first";`

Fix: Change API.md:209 to: `- Arming after aborted or released starts a new shot (seq back to idle). From ANCHORED it is refused: run /release first.`


## 3. Firmware sends one 409 reason ('busy (motors running, armed or in flight)') for every /flightlog refusal, so the client's busy-retry also spins up to 45 s on an armed dart; the mock sends 'armed'/'in flight' instead

- File: `firmware/dart/kedge_dart/kedge_dart.ino:275`
- Severity: low (claimed low); unverified (low)
- Owner: fw-dart

Scenario: Dart.flightlog retries a 409 whose reason starts with 'busy' for up to busy_wait_s=45 s. Its docstring says that refusals for armed or in flight raise straight away, and mock_dart.py answers those cases with `refused: armed` / `refused: in flight`, so the Jetson unit tests pass. The real firmware answers all three cases with `refused: busy (motors running, armed or in flight)`. Example: after `/arm?on=1&auto=0`, an operator runs `python3 -m grappstar dart flightlog`. The CLI hangs silently for about 45 s and then prints the refusal. jetson/tests/test_dart_realsim.py:69 (`d.flightlog()  # refused while armed`) takes 45 s for the same reason. Against kedge_sim with busy_wait_s=6, an armed dart was retried for 4.1 s before the refusal came back. The client cannot tell motors running, which is worth waiting for, from armed, which never clears by itself.

Evidence: kedge_dart.ino:274-275 `bool busy = K.running() || K.seqRunning() || K.armed || K.flight.flying();` / `if (busy) { web.send(409, "text/plain", "refused: busy (motors running, armed or in flight)\n"); return; }`
dart.py:203-204 `409 "busy" (motors running...) is retried with backoff for up to busy_wait_s; other refusals (armed, in flight) raise.` and dart.py:212 `if not e.reason.startswith("busy") or time.time() + delay > t_end: raise`
mock_dart.py:315-318 `refuse("armed" if self.armed and ph != "FLIGHT" else "in flight")` ... `refuse("busy")`
API.md:20 `and for /flightlog busy (motors running, armed or in flight)`

Fix: In handleFlightlog, read the three conditions separately and send a distinct reason for each, matching the mock and the client docstring: `if (flying) refused: in flight`, `else if (armed) refused: armed`, `else if (running||seqRunning) refused: busy`. Update API.md lines 20 and 255 to list the three reasons, and change the sim tests that assert `reason.startswith("busy")` while armed (sim/test_jetson_client.py:89) to expect `armed`.


## 4. R2-B2 #5 still open: the web page's key-release stop (/key?c=%20&keep=1) aborts a running /auto or /release when a refused key is let go, and clears any latched fault

- File: `firmware/dart/kedge_dart/kedge_dart.ino:345`
- Severity: low (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-dart

Scenario: The dart is ANCHORING, after auto-anchor on landing or a Jetson /auto. On the dart web page the operator taps or holds a run key, say '3'. The firmware refuses it with 'busy'. On keyup, release() sends /key?c=%20&keep=1, and lost() sends the same request on blur, pagehide or a hidden tab. The handler calls K.stop(t,false). That sets seq=ABORTED and stops every motor, leaving the anchor half done. The scratch test (b3.cpp, real kedge_logic.h) shows `before: seq=kick swing phase=ANCHORING kick=1`, then `key 3 -> busy`, then `after keyup stop: seq=aborted phase=IDLE kick=0`. The same path clears latched faults (stop() sets fault=F_NONE), so the 'fault latched: send stop first' guard is defeated by the page itself. Test C held drills into a short 10 times: OVERCURRENT latched each time, the page's keyup cleared it each time, and the next press drove into the short again (fault at the end = none). The test at test_logic.cpp:454 only covers the FLIGHT case. Nothing in the code at 258001f changed since round 2.

Evidence: kedge_dart.ino:252 `function release(id){...if(!held.size)send(' ',1)}   // keep=1: stop the motors, stay armed`
kedge_dart.ino:256 `function lost(){if(held.size){held.clear();fetch('/key?c=%20&keep=1&ms=0&n='+(++N)+'&p='+P,{keepalive:true})...`
kedge_dart.ino:344-345 `if ((c == " " || c == "s") && web.arg("keep") == "1") { ... K.stop(t, false); ...`
kedge_logic.h:379 `bool wasSeq = seqRunning();`, :391 `if (wasSeq) seq = ABORTED;`, :396 `fault = F_NONE;`
kedge_logic.h:454 `if (seqRunning()) return "busy";` (the key itself was refused)

Fix: Add a release-only stop to Kedge, modelled on linkLost: `void keyRelease(uint32_t now) { if (!seqRunning() && running()) { stopAll(now); ev("keys released: motors stopped"); } }`. It does not touch seq, fault or the arm. At kedge_dart.ino:345, call `K.keyRelease(t)` instead of `K.stop(t, false)`. An explicit STOP (space, the STOP button, /stop, /stop?disarm=0) still aborts and clears faults. Update API.md lines 101 and 121 to match. Add tests: startAuto, then key('3') refused, then keyRelease, and require seq still anchoring; and a trip fault followed by keyRelease, requiring fault still OVERCURRENT.

> Skeptic (confirmed): I could not refute this. The mechanism is in the code exactly as described, but medium overstates it, so I rate it low.

What the code does:
- kedge_dart.ino:252 `release()` sends `send(' ',1)`, which is `/key?c=%20&keep=1`, as soon as `held` is empty. That covers a run key the firmware refused with "busy", because the page never checks whether the key was accepted.
- kedge_dart.ino:256 `lost()` sends the same request on blur, pagehide or a hidden tab, but only while a key is held.
- kedge_dart.ino:344-345: the keep=1 branch calls `K.stop(t, false)`.
- In kedge_logic.h (actual lines 368, 379 a
>
> Skeptic (confirmed): I could not refute it. The code does what the claim says, and the abort path is real.
- On the page, hold() adds the key to `held` whatever the firmware answers. release() at kedge_dart.ino:252 and lost() at :256 send /key?c=%20&keep=1 whenever the last held key goes, even if that key was refused with 'busy'.
- The keep=1 branch at kedge_dart.ino:344-345 calls K.stop(t,false).
- In kedge_logic.h, stop() records wasSeq (368), stops all motors (379), sets seq=ABORTED when a sequence was running (380), and sets fault=F_NONE no matter what `disarm` is (385).
- So during ANCHORING (after auto-ancho
>

## 5. Removing the landing-time freeze cap still passes test_logic and fuzz: no test covers a landing 7 to 10 s after launch, and the fuzz still has no window invariant

- File: `firmware/dart/test/fuzz_logic.cpp:538`
- Severity: low (claimed low); unverified (low)
- Owner: fw-dart

Scenario: The R1/R2 #1 firmware bug is fixed at w/fw-dart 258001f. kedge_logic.h:494 freezeCap() = tLaunch - 2000 + 599*20. :498 schedules it on LAUNCH. :503-505 cap the landing freeze. :393 caps it in stop(). A scratch copy of the unchanged tree passes a new case for a 7.5 s impact landing (log 12985..24965, tLaunch 15000). However, the tests do not guard every part of the fix. In a scratch copy (/tmp/claude-0/review-scratch-B3/d2) I deleted only the landing cap at kedge_logic.h:504 (`if ((int32_t)(fr - cap) > 0) fr = cap;`). test_logic still reported '234 checks, ALL PASS', and `fuzz_logic --quick` still reported 'FUZZ PASS', with 0 violations on invariant 8 over 322968 checks. With that regression, an impact landing 7.5 s after launch fails. Input: arm, 8 g push at t=15000 ms, 0.4 g flight, 8 g impact at 22500 ms, then still. The freeze moves from the cap (24980) to tLand+3000 = 25510. The frozen log then starts at 13525 instead of 12985, so 525 ms of the C2 '2 s before launch' window is lost, and nothing fails. The scratch case /tmp/claude-0/review-scratch-B3/d2/test/tb3.cpp prints 'B3 7.5 s impact landing: tLaunch 15000 tLand 22510 how impact, log 13525..25505' and FAIL. The existing window tests miss this because each of them lands after the cap or long before it. test_logic.cpp:340 lands at 10 s, after the cap. :474 is a 15 s timeout. :523 is a stop at 6.5 s, and stop() keeps its own copy of the cap. The fuzz checkLogJson still checks only JSON syntax, n, row width and time order, so R2 #7 is still open on the fuzz side.

Evidence: fuzz_logic.cpp:538 `if (cnt != nn) why = "n does not match the sample count"; else if (!width) ... else if (!order) why = "samples not in time order";` (no window check). fuzz_logic.cpp:27 `//   8 every status() and LogJson output is valid JSON (own validator below)`. kedge_logic.h:503-505 `uint32_t fr = flight.tLand + 3000, cap = freezeCap(); if ((int32_t)(fr - cap) > 0) fr = cap; if (!log.frozen) log.scheduleFreeze((int32_t)(fr - now) > 0 ? fr : now);`. test_logic.cpp:336 lands at t=25000 (10 s, after the cap), :471 is a timeout, :523 is a stop: none lands between tLaunch+7000 and tLaunch+9980.

Fix: (1) In test_logic.cpp after line 342, add: `{ fresh(); ACC = [](uint32_t t) { return t < 15000 ? 1.0f : t < 15030 ? 8.0f : t < 22500 ? 0.4f : t < 22515 ? 8.0f : 1.0f; }; K.arm(true, false, NOW); run(32000); CHECK(std::string(K.flight.how) == "impact" && K.log.frozen && (int32_t)(K.log.at(0).t - (K.flight.tLaunch - 1980)) <= 0, "impact landing 7.5 s after launch: freeze capped, 2 s before launch kept"); }`. (2) In fuzz_logic.cpp checkLogJson, before the `if (why) violate(8, ...)` at line 541, add a window invariant: `if (!why && K.log.frozen && K.flight.launched && K.log.count == LOG_N && (int32_t)(K.log.at(0).t - (K.flight.tLaunch - 2000)) > 40) why = "frozen log lost the 2 s before launch";`. Also update the invariant 8 comment at line 27 to describe it.


## 6. Dart.hold() sends /stop?disarm=0 after a refused key, which aborts a running /auto or /release and clears a latched fault

- File: `jetson/grappstar/comms/dart.py:264`
- Severity: low (claimed medium); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: The dart is ANCHORING, after a landing or after `dart auto`. An operator on the bench runs `python3 -m grappstar dart key 3 --hold 1` (cli.py:196 calls d.hold). The first /key gets a 409 `busy`. hold() re-raises it, and its finally block sends /stop?disarm=0. The firmware's K.stop(t,false) then sets seq = ABORTED and turns off every motor. The anchor is left half done, with the kick part-way out and the foot and pods not planted, because a command the dart refused anyway. The CLI then prints `dart refused /key: busy` followed by the advice to check the WiFi. The same path clears a latched fault: after a `fault latched: send stop first` refusal, the fault is gone without anyone sending a stop. I reproduced this against the real sketch in kedge_sim (/tmp/claude-0/review-scratch-B3/hold_abort.py). Output: `before: ANCHORING kick swing / hold refused: busy / after: IDLE aborted stop: sequence aborted`.

Evidence: dart.py:257-262 `for k in keys: self.key(k, ms)` / `except DartRefused as e: if e.reason.startswith("stalled"): break / raise`
dart.py:264-267 `finally: ... self.stop(disarm=False)`
kedge_dart.ino:348 `/stop` with disarm=0 -> `K.stop(t, false)`
kedge_logic.h:379 `bool wasSeq = seqRunning();` ... :392 `if (wasSeq) seq = ABORTED;` ... :396 `fault = F_NONE;`
kedge_logic.h:454 `if (seqRunning()) return "busy";` (the key itself was refused)

Fix: In Dart.hold(), track whether any key was accepted: set `sent = False` before the loop and `sent = True` after each `self.key(...)` returns without raising. In the finally block, send `self.stop(disarm=False)` only when `sent` is true. A hold whose first key was refused (`busy`, `in flight`, `fault latched...`) then changes nothing on the dart. A hold that ran motors still always ends with a stop. Add a test in jetson/tests/test_comms_dart.py or sim/test_jetson_client.py: start /auto, call hold('3', 0.5), expect DartRefused, and require phase still ANCHORING.

> Skeptic (confirmed): The mechanism is real. In jetson/grappstar/comms/dart.py:247-269, hold() re-raises any DartRefused whose reason does not start with "stalled", and its finally block always sends self.stop(disarm=False). In firmware/dart/kedge_dart/kedge_dart.ino:351, /stop?disarm=0 calls K.stop(t,false). kedge_logic.h stop() then runs stopAll(), sets seq=ABORTED if a sequence was running, and clears fault=F_NONE no matter what disarm is. Manual keys are refused at kedge_logic.h:441-443 ("busy" while seqRunning, "in flight", "fault latched: send stop first") before any motor moves. So a refused first key still 
>
> Skeptic (confirmed): I could not refute this. The code does what the claim says.

Client side: jetson/grappstar/comms/dart.py:246-270, Dart.hold(). Any DartRefused whose reason does not start with "stalled" is re-raised from inside the try. The finally block then always calls self.stop(disarm=False), with no check for whether any key was accepted. stop(disarm=False) sends GET /stop?disarm=0 (dart.py:169-172). The class docstring for DartRefused (dart.py:37) says a 409 means "the dart answered and refused (nothing changed)". So hold() turns a refusal that changed nothing into a state change.

Firmware side: firmwar
>

## 7. Dart.hold always sends /stop?disarm=0 in its finally block, even when every key was refused, so `grappstar dart key C --hold S` aborts a running anchor or release and clears a latched fault

- File: `jetson/grappstar/comms/dart.py:267`
- Severity: low (claimed low); unverified (low)
- Owner: jetson

Scenario: The dart is ANCHORING or RELEASING, or has a latched fault such as OVERCURRENT or LOW_RAIL. The operator runs `python3 -m grappstar dart key 3 --hold 2` (cli.py:196 -> Dart.hold). The first self.key() gets a 409 'busy' (or 'fault latched: send stop first'). That is not a 'stalled' refusal, so hold re-raises it, but the finally block still calls self.stop(disarm=False). The firmware handles /stop?disarm=0 with K.stop(t,false) (kedge_dart.ino:351), which sets seq=ABORTED when a sequence is running and always sets fault=F_NONE (kedge_logic.h:391, :396). A command the dart refused ends up aborting the anchor (motors stop part-way) or silently clearing the fault the dart asked the operator to acknowledge.

Evidence: dart.py:255-262 `while ...: try: for k in keys: self.key(k, ms)` / `except DartRefused as e: if e.reason.startswith("stalled"): break ... raise`
dart.py:263-267 `finally: # stop the motors but keep the landing detector armed ... self.stop(disarm=False)`
cli.py:196 `d.hold(a.c, a.hold)`
kedge_dart.ino:351 `if (web.hasArg("disarm") && web.arg("disarm") == "0") { ... K.stop(t, false); ...`
kedge_logic.h:391 `if (wasSeq) seq = ABORTED;` / :396 `fault = F_NONE;`

Fix: In Dart.hold, set `accepted = False` before the loop and `accepted = True` after each self.key() call that does not raise. In the finally block, send the stop only when a key was accepted: `if accepted: try: self.stop(disarm=False) except Exception: self.safe_stop()`. A hold that was refused from the first key then leaves the sequence and the fault alone. If the firmware fix above (keyRelease) is adopted, a keep-style release endpoint can be used instead. Add a mock_dart test: start auto, call hold('3', 0.5) and expect DartRefused, then require seq still anchoring.

