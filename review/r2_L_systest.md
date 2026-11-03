# Round 2 L: system test harness validity (w/systest)

Reviewed at base-v1 commit 2226d7d. Method: independent finder subagents, one per lens; every finding of
medium severity or above was checked by two skeptic subagents told to refute it; kept only if fewer than
two refuted it. Low findings are listed unverified. Severity is the finder's, lowered when both skeptics
agreed it is real but rated it lower.

Confirmed: 15. Refuted and dropped: 1.

| # | Sev | File:line | Owner | Finding |
|---|---|---|---|---|
| 1 | medium | `firmware/base/test/sim.h:23` | systest | Anchored line modelled as a 40 N/m spring, about 60x softer than the 1 mm Dyneema tether: tow-limit overshoot is hidden, so c_tow_limit cannot catch it |
| 2 | medium | `systest/harness.py:866` | systest | Aim check only reads the firmware's own AIMED pitch, never the real plant aim, so an aim bug passes |
| 3 | medium | `systest/harness.py:916` | systest | c_reeled has no lower bound and the C1 regex makes `retrieve` optional, so seating on the capstan passes (violates 10.1 step 17 / D39) |
| 4 | medium | `systest/harness.py:955` | systest | Nominal and ref_nominal never check for faults or retries: firmware faults hidden by the scripted auto-Retry still pass |
| 5 | low | `firmware/base/test/sim.h:70` | systest | A non-holding dart is modelled as a free line at a constant 1 N, so the check pull's 'dragging' and 'slipping' branches are never exercised |
| 6 | low | `systest/base_shim.cpp:110` | systest | Shim USB TX path holds ~20 KB, about 10x the sketch's 2048-byte CDC buffer, so it hides the telemetry and event loss a slow host causes on hardware |
| 7 | low | `systest/base_shim.cpp:128` | systest | Shim event ring holds 24 events; the sketch's ring holds only 23 |
| 8 | low | `systest/base_shim.cpp:216` | systest | land_short and c_tug/c_wrap_free cannot fail in the D22 order: tugging a free dart costs no force, and the total wrap draw is under the 1 m threshold |
| 9 | low | `systest/harness.py:602` | systest | Reference driver swallows every error while waiting for RESET after FIRED, including a FAULT |
| 10 | low | `systest/harness.py:648` | systest | 10.1 step 14 order (stop, then dart release) is never checked; the reference driver itself releases before stop |
| 11 | low | `systest/harness.py:887` | systest | `tidy ran` only checks that the mission entered the TIDY step; it never checks that the base took up the slack |
| 12 | low | `systest/harness.py:1173` | systest | dart_never_anchors can never pass: 'no TOW' checks the whole history, but after a successful reshoot the mission must tow |
| 13 | low | `systest/harness.py:1183` | systest | snag_retrieve's 'LIMIT seen in RETRIEVE' counts LIMIT over the whole run, so the tow's own LIMIT events satisfy it |
| 14 | low | `systest/harness.py:1187` | systest | no_angle_sensor ANDs two mutually exclusive outcomes ('DONE in SW_INDEX mode' and 'explicit refusal'), so it cannot pass |
| 15 | low | `systest/harness.py:1275` | systest | nogo_reshoot keeps the single-shot C1 regex, which cannot match the two-shot NO_GO path that 10.1 allows |

## 1. Anchored line modelled as a 40 N/m spring, about 60x softer than the 1 mm Dyneema tether: tow-limit overshoot is hidden, so c_tow_limit cannot catch it

- File: `firmware/base/test/sim.h:23`
- Severity: medium (claimed high); 2 of 2 skeptics confirmed
- Owner: systest

Scenario: In every tow scenario (nominal, ref_*, usb_unplug_tow, power_tow, brownout_tow) the harness sets only anchorTautAt (harness.py:370) and leaves anchorK at its default of 40 N/m. At 40 N/m, reaching 50 N takes 1.25 m of reel-in, which is 16 % strain on a 7.7 m line. The spec line is 'Dyneema braid 150 lb, d 1.0, break 667 N' (params.py:54). Dyneema stretches about 3.5 % at break, so it is roughly 2.5 kN/m or stiffer at 7.7 m, and stiffer still on shorter tows. I wrote a scratch rig (/tmp/claude-0/review-scratch-L/tow_k.cpp): the real base_logic.h on sim.h with the harness's matched calibration, `reel duty 1 max_n 50 until tension`. Measured peak and held plant tension (HOLD self-locks the overshoot): K=40: 50.0 N; K=1000: 50.6 N; K=3000: 62.9 N (duty 0.8: 57.6 N); K=10000: 101.6 N (duty 0.8: 88.0 N). From K=3000 up the hold is over the c_tow_limit bound of 57.5 N, and at K=10000 it passes the 84 N limiter. With the soft plant these scenarios pass, so the harness reports the tow limit as respected when on hardware it would not be.

Evidence: sim.h:23 `double anchorK = 40;             // N per m of capstan travel after taut`
sim.h:71 `double reeled = anchorTautAt - lineOut; return reeled > 0 ? reeled * anchorK : 0.5;`
harness.py:370 `self.base.set(anchorTautAt=round(max(0.0, d), 3))` (anchorK never set anywhere in harness.py)
harness.py:902-905 c_tow_limit `return mx <= lim * 1.15`
params.py:54 `LINE = dict(name="Dyneema braid 150 lb", d=1.0, length_m=20.0, break_n=667.0)`

Fix: Derive the anchor stiffness from the line. In Rig._world, when ANCHORED is set, also send `anchorK = EA / max(d, 0.5)` with EA about 20000 N (1 mm Dyneema braid, about 3.5 % at 667 N; make it a Scenario knob `line_ea_n`). Re-send it whenever tow_assist or RefMission.s_tow moves anchorTautAt. Add a `tow_stiff` scenario (EA 20 kN, short line about 3 m) and its ref_ variant, both keeping c_tow_limit. Leave the 40 N/m default only in the firmware unit tests.

> Skeptic (confirmed): The claim holds up. The cited line numbers in harness.py are stale: the file is systest/harness.py, the ANCHORED hook is at line 357 and c_tow_limit at 646. The substance is right, though. In firmware/base/test/sim.h, line 23 defaults anchorK to 40 N/m and tension() at line 71 returns (anchorTautAt - lineOut) * anchorK. systest/harness.py never sets anchorK. It only sets anchorTautAt, at lines 357, 536 and 842. So every tow and check-pull scenario runs on a plant that needs 1.25 m of reel-in to reach 50 N.

I checked the overshoot claim myself. I rebuilt the claimant's rig against the repo's o
>
> Skeptic (confirmed): The core of the claim holds, though some citations are off. The harness is systest/harness.py, not firmware/base/test/harness.py. The anchor set is at line 357 (`anchorTautAt = lineOut - 0.3`), and tow_assist at about line 536 moves it by 0.6 m. c_tow_limit is at line 646 (`mx <= lim * 1.15`); lines 902-905 are only the power_* scenario list that includes it. I checked every file: nothing in systest/harness.py sets anchorK, so every system-test tow runs on the default 40 N/m from sim.h:23. Firmware unit tests use 50 to 80 N/m and test_fuzz uses 20 to 100 N/m, so no test anywhere uses a stiff l
>

## 2. Aim check only reads the firmware's own AIMED pitch, never the real plant aim, so an aim bug passes

- File: `systest/harness.py:866`
- Severity: medium (claimed high); 2 of 2 skeptics confirmed
- Owner: systest

Scenario: Change base_logic.h pitchAt() (line 174) to add a constant +5 deg to the table value. The firmware then drives the plant until its own biased pitch() equals the target and emits AIMED with pitch_deg equal to the target. The real rail ends up about 5 deg off. c_aim still passes, because |AIMED.pitch_deg - solution| <= 1. Nothing else catches the error either: the harness renders the dart flight at the solution pitch (line 437) and sets the plant line-out from the true range (line 436), whatever the plant aim is. In the field the dart misses the target and the mission is lost, yet nominal and ref_nominal stay green.

Evidence: harness.py:860-867 `ev = [e for e in rig.events if e.get("evt") == "AIMED"] ... ok = e.get("ok") and ... abs(e.get("pitch_deg", 0) - sol) <= 1.0`. base_logic.h:1192 builds the event from its own estimate: `float p = pitch(); ... o.num("pitch_deg", p, 2)` with pitch() = pitchAt(aimPhase) (line 181). The shim exposes the true plant train phase `"aimPh"` (base_shim.cpp:274/279, sim.h aimPh), but harness.py never reads it. harness.py:436-437 `self.base.set(lineOutAfterShot=round(rng_true * 1.05 + 0.3, 2))`; `self.video.render(sol.pitch_deg ...)` (the outcome does not depend on the plant aim). Run ref_nominal: `aim reached: AIMED 22.58 target 22.62` (firmware value only).

Fix: In c_aim, take the plant snapshot at the end of AIM (rig.step_plant['COCK'] or a peakreset reading on the AIMED event) and convert plant aimPh to pitch with the sim's own aim geometry (pitch = table from params.LAUNCH/AIM applied to (aimPh - aimHomeDeg) mod aimCycleDeg, exported by the shim as a new "pitchTrue" field computed in sim.h). Require |pitchTrue - solution| <= 1.0. Also render the dart video and set lineOutAfterShot from pitchTrue, not from sol.pitch_deg and rng_true.

> Skeptic (confirmed): I couldn't refute the core claim, but I rate it lower than claimed. The cited line numbers are stale. c_aim is at systest/harness.py:604-611, not :866, and the render call is at :420-422. The content matches what the claim says, though. c_aim takes the last AIMED event and checks its ok flag and |pitch_deg - solution| <= 1.0. In base_logic.h, that pitch_deg is the firmware's own pitch(), which is pitchAt(aimPhase): the firmware's estimate of the phase run through the firmware's own table. Nowhere does harness.py read the plant's aimPh (grep finds no matches). So the check is circular: it compa
>
> Skeptic (confirmed): The core claim holds, though the line numbers are off. The aim check is at systest/harness.py:607-611, not 866. It compares only the firmware's self-reported AIMED.pitch_deg with the solution pitch (`abs(e.get("pitch_deg", 0) - sol) <= 1.0`). harness.py never reads the plant's "aimPh", which base_shim.cpp:250/255 exports. The dart video is rendered from sol.pitch_deg (harness.py:422 and 930), and lineOutAfterShot is set from the true range (line 421). So the mission outcome in the systest does not depend on where the plant actually aimed.

A constant bias added inside pitchAt (base_logic.h:174
>

## 3. c_reeled has no lower bound and the C1 regex makes `retrieve` optional, so seating on the capstan passes (violates 10.1 step 17 / D39)

- File: `systest/harness.py:916`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: systest

Scenario: Mutate mission.py retrieve() to reel with until="stop" and never send stop, or make base_logic's line_m estimate read about 1 m high. The capstan then pulls the dart all the way home with the wraps on (lineOut 0 at UNWRAP). c_reeled passes, since 0 <= approach_m + 0.3. c_seated passes. C1_SEQ also accepts a run with no `retrieve` token at all. This already happens in the unmutated ref_nominal run: the reel-in was meant to stop at 1.5 m but the plant had only 0.618 m out at UNWRAP. c_reeled reported ok, and no `retrieve` was sent (`... reel stop > unwrap > mode LAUNCH` was accepted).

Evidence: harness.py:912-917 `return p.get("lineOut") is not None and float(p["lineOut"]) <= rig.app.cfg.mission.approach_m + 0.3` (docstring: 'or all the way to seated'). harness.py:967-968 `... (reel stop > )?unwrap > (retrieve > )?mode LAUNCH$`. BASE_SPEC 10.1 step 15: 'stop at line_m about 1.5'; step 17: '`retrieve` ... take-up final approach in NEUTRAL only ... Never seat on the capstan'. Run /tmp/claude-0/review-scratch-L/ref_nominal.log: `reeled in to the approach: line out 1.397 m at RETRIEVE start, 0.618 m at UNWRAP` -> ok.

Fix: Make c_reeled two-sided: approach_m - 0.4 <= plant lineOut at UNWRAP <= approach_m + 0.3, and plant seated must be False at the UNWRAP snapshot. Add a plant check that seating happened in NEUTRAL: record the plant inReel and wraps when 'seated' first turns true, and require inReel == False and wraps < 0.05. Make `retrieve` mandatory in C1_SEQ (`unwrap > retrieve > mode LAUNCH`), and only allow it to be dropped when the plant was already seated by the unwrap take-up with the dog out of REEL.

> Skeptic (confirmed): The gap is real, but one part of the claim goes too far. Note first that local HEAD of w/review (8b32e37) has an older systest/harness.py with no C1 check at all. The cited code is in origin/base-v1 (1d668d6), at different line numbers: c_reeled is at harness.py:927-932 and C1_SEQ at 982-983. Neither line number in the claim matches either version.

What holds up:
(1) c_reeled only checks `lineOut <= approach_m + 0.3` on the plant snapshot taken at UNWRAP. Its docstring even says "or all the way to seated", so a dart seated on the capstan (lineOut 0) passes. That is the opposite of BASE_SPEC 1
>
> Skeptic (confirmed): Note first: the cited code is not on the checked-out w/review HEAD. There, systest/harness.py has c_reeled at line 656 and no C1 regex at all. The cited lines are on origin/base-v1, with c_reeled at about line 927 and C1_SEQ at about line 982, and that is the version I checked.

The core claim holds. c_reeled (base-v1) passes whenever the plant lineOut at the UNWRAP snapshot is at most approach_m + 0.3. It has no lower bound. Its docstring even says it accepts reeling 'all the way to seated' on the capstan, with the wraps on. That contradicts BASE_SPEC 10.1 step 17 ('Never seat on the capstan'
>

## 4. Nominal and ref_nominal never check for faults or retries: firmware faults hidden by the scripted auto-Retry still pass

- File: `systest/harness.py:955`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: systest

Scenario: Mutate base_logic.h so the 50 ms current-trip blanking after a start is 0 ms, or make tickWrap raise DRUM_TIMEOUT once per wrap. The first attempt FAULTs. The scripted operator answers 'Retry' (Scenario.answer, up to retry_max=3), or RefMission.recover() clears and retries. The mission still ends DONE with the plant seated, so every nominal check passes. The unmutated nominal run already shows a DRUM_TIMEOUT FAULT followed by an automatic Retry. That run failed only for other reasons; no check flagged the fault itself.

Evidence: harness.py:955-962 nominal_checks() has no check on FAULT events or failure prompts. c_no_fault_left (line 947) only looks at the final telemetry and is not in the list. harness.py:829-832 `if "Retry" in choices and self._retries < self.retry_max: ... return "Retry"`. RefMission._run (around lines 755-765) catches every step exception and calls recover() up to retry_max, logging it only into rig.prompts. Nominal run log: `evt FAULT DRUM_TIMEOUT ... ANSWER Retry`.

Fix: Add to nominal_checks(): ("no FAULT / ABORTED-by-fault events", lambda r,res: (not any(e.get('evt')=='FAULT' for e in r.events), [e for e in r.events if e.get('evt')=='FAULT'])) and ("no failure prompts / retries", lambda r,res: (not any('failed' in p['text'] or p['answer']=='Retry' for p in res['prompts']), ...)). Fault scenarios keep their own check lists, which already replace nominal_checks.

> Skeptic (confirmed): The claim holds against origin/base-v1's systest/harness.py. Line numbers are off by about 15: the claim says 955, but nominal_checks() is at line 970 there. Note that the w/review HEAD (8b32e37) carries an older 1004-line harness with no RefMission or ref_nominal, so I checked the origin/base-v1 copy (git show origin/base-v1:systest/harness.py).

What I confirmed:
(1) nominal_checks() (around line 970) has no check on FAULT events, failure prompts or retry counts. c_no_fault_left exists but is not in the list. c_launch only looks at whether a fault is still present in the final telemetry, and
>
> Skeptic (confirmed): I checked this against origin/base-v1:systest/harness.py, the branch under review. The w/review working tree has an older harness with no RefMission, and its line numbers do not match the claim. In base-v1 the cited code is there and does what the claim says.

What the code does:
- nominal_checks() (line 970) has no check on FAULT events, on prompts that say "failed", or on Retry answers. c_no_fault_left (line 954) exists but nothing uses it, and it only reads final_tel anyway.
- 'nominal' takes the default nominal_checks(). 'ref_nominal' (line 1122) takes nominal_checks() minus 'tidy ran', pl
>

## 5. A non-holding dart is modelled as a free line at a constant 1 N, so the check pull's 'dragging' and 'slipping' branches are never exercised

- File: `firmware/base/test/sim.h:70`
- Severity: low (claimed medium); 2 of 2 skeptics confirmed
- Owner: systest

Scenario: dart_never_anchors (drills jammed, 'the check pull must catch it') sets anchor_holds=False, so the world never sets anchorTautAt. The plant stays in slack retrieve: tension() returns 1.0 N whatever the reel does. The CHECK reel then winds the dart all the way home and the mission fails on SEATED ('came all the way home'). That is the only branch the scenario can reach. A real dart jammed against a stone or wedged in grass drags with friction, or holds briefly and then creeps out. mission.anchored_check treats reaching check_n (15 N) as 'the main evidence' that the anchor holds ('a dart dragging over the ground pulls only a few N'). A dart dragging at 15-20 N would reach LIMIT, show no decay with the worm self-locked, and go on to TOW. The plant cannot produce a drag force or a creeping anchor, so this real failure is hidden and the scenario passes.

Evidence: sim.h:70 `if (anchorTautAt < 0) return lineOut > seatAtLine + 0.02 ? 1.0 : 0;   // slack retrieve: light take-up`
harness.py:367 `if ph == "ANCHORED" and (self.sc.anchor_holds or getattr(self, "anchor_ok", False)):` (else nothing is set)
harness.py:1164-1166 dart_never_anchors `anchor_holds=False`
jetson/grappstar/mission.py:823 `# Reaching the check tension is itself the main evidence: a dart dragging over the ground pulls only a few N.`

Fix: Add a drag and creep model to the plant. Add a Sim field `dragN` (shim knob): in tension(), when anchorTautAt < 0 and the dart is not seated, return fmax(1.0, dragN) instead of a constant 1.0. Add `slipN` for an anchored dart: in step(), if reeled * anchorK > slipN, move anchorTautAt down so tension stays at slipN (the anchor creeps). Have dart_never_anchors set dragN (for example 6 N), and add variants dragN 20 (above check_n) and slipN 25 (an anchor that creeps during the hold). Expect the check to fail in each, with 'no TOW on the dragging dart'.

> Skeptic (confirmed): The facts in the claim are right. The line numbers are slightly off: the code lives in systest/harness.py at 355 and 827-837, not 367 and 1164.

- **Plant:** in firmware/base/test/sim.h, tension() returns a constant 1.0 N whenever anchorTautAt < 0. Neither the plant nor the harness has a drag or creep model. The only place a slip is injected is the firmware unit test test_base.cpp:443 (anchorTautAt -= 0.2), which never goes through mission.anchored_check.
- **System test:** dart_never_anchors sets anchor_holds=False, so the harness never sets anchorTautAt. The check pull therefore always ends 
>
> Skeptic (confirmed): The core claim holds up in the code, though some cited line numbers are off (the harness is systest/harness.py: the anchor hook is at :355 and dart_never_anchors at :827-837).

1. firmware/base/test/sim.h:69-72. tension() returns a constant 1.0 N whenever anchorTautAt < 0. Otherwise it is (anchorTautAt - lineOut) * anchorK with nothing that limits it. The plant has no drag-force term and no way for the anchor to creep, slip or move once anchorTautAt is set. step() never changes anchorTautAt.

2. systest/harness.py:355. anchorTautAt is set only when anchor_holds or anchor_ok is true. dart_never
>

## 6. Shim USB TX path holds ~20 KB, about 10x the sketch's 2048-byte CDC buffer, so it hides the telemetry and event loss a slow host causes on hardware

- File: `systest/base_shim.cpp:110`
- Severity: low (claimed medium); 2 of 2 skeptics confirmed
- Owner: systest

Scenario: While the wiper moves, telemetry runs at 20 Hz with lines of about 250 bytes, roughly 5 KB/s. Suppose the Jetson reader stalls for 1 to 3 s (vision or GC pause, or a busy thread). On hardware, Serial.setTxBufferSize(2048) fills after about 0.4 s. From then on, writeLine(outBuf,0) drops every telemetry line, writeLine(l,20) in drainEvents fails, and events back up into the 23-slot ring and then are dropped as the newest (FIRED/STALL/FAULT can be lost). Reply writes can also block loop() for up to 1 s, which delays prefs.putBytes and therefore saveDoneSeq, so a saveAck motor start waits. In the shim, writeLine() does a raw non-blocking write() to the pty master, which I measured holding 20592 bytes with the slave open in raw mode and nobody reading (scratch /tmp/claude-0/review-scratch-L/ptycap.c). It then retries up to 100 ms. So the same 1 to 3 s stall loses nothing: no telemetry drops, no event drops, no NVS delay. The harness never checks evDrops or telDrops either. A systest pass therefore says nothing about host-stall behaviour on hardware.

Evidence: grappstar_base.ino:136 `Serial.setTxBufferSize(2048);`; :187 `while (Serial.availableForWrite() < n + 1) { if (millis() - t0 >= waitMs) return false; delay(1); }`; :199 `if (!writeLine(l, 20)) break;`; :232 `if (due) writeLine(outBuf, 0);  // telemetry is dropped when there is no room`. base_shim.cpp:113-118 `// room check: a pty buffer holds a few kB ... ssize_t n = write(masterFd, l.data(), l.size()); ... for (int tries = 0; done < l.size() && tries < 200; tries++) { usleep(500); ...`; :380 `if (!writeLine(tb)) telDrops++;`. Measured pty capacity is 20592 bytes, and ioctl(slave,FIONREAD) correctly reports the unread bytes (scratch /tmp/claude-0/review-scratch-L/fion.c).

Fix: Model the sketch's 2048-byte TX buffer in writeLine. Use `int q=0; ioctl(slaveKeepFd, FIONREAD, &q); int room = 2048 - q; if ((int)l.size() > room) return false;` (needs <sys/ioctl.h>), check this before any write, and never write a partial line. Give each caller the sketch's wait budget, measured in sim time and without usleep: telemetry gets 0 (drop and count telDrops); drainEvents keeps the line queued, matching the 20 ms then break; a reply waits up to 1000 ms of sim time and is then dropped. While a reply wait is pending, also hold the NVS completion (nvsPending/saveDoneSeq) and further serial reads, since loop() is blocked then. Then add a harness scenario that stops reading the pty for 2 s mid-REEL and asserts on evDrops and telDrops.

> Skeptic (confirmed): The fidelity gap is real, but its impact is overstated. I'm rating it low, not medium.

Confirmed in the code:
- systest/base_shim.cpp:110-120 `writeLine()` has no room check against 2048 bytes. It does a raw non-blocking write() to the pty master (O_NONBLOCK, line 98), then finishes any partial write with up to 200 x usleep(500) of wall-clock retries.
- The sketch (firmware/base/grappstar_base/grappstar_base.ino:136, 185-190, 199, 232) uses `Serial.setTxBufferSize(2048)` and an availableForWrite gate with per-caller waits: telemetry 0 ms, events 20 ms, replies 1000 ms.
- So the shim's back-pr
>
> Skeptic (confirmed): The fidelity gap is real, but the claim overstates it. Code check: systest/base_shim.cpp:110-122 writeLine does a non-blocking write() to the pty master, then retries up to 200 x 500 us (about 100 ms) of wall-clock time. It has no 2048-byte room check, so the only limit on what it holds is the pty/n_tty buffering, which the reviewer measured at about 20 KB. The sketch (firmware/base/grappstar_base/grappstar_base.ino:136, 185-190, 199, 232) uses a 2048-byte TX buffer with whole-line availableForWrite checks: telemetry wait 0, events 20 ms then break, replies 1000 ms. The 1000 ms reply wait bloc
>

## 7. Shim event ring holds 24 events; the sketch's ring holds only 23

- File: `systest/base_shim.cpp:128`
- Severity: low (claimed low); unverified (low)
- Owner: systest

Scenario: The host is not draining (TX buffer full) and the base raises a burst of events. The sketch's circular buffer treats `next == ringTail` as full, so it holds at most RING-1 = 23 entries and drops the 24th event. The shim accepts 24 because it checks `evq.size() >= 24`. A scenario that just fits in systest (exactly 24 queued events) loses its newest event on hardware.

Evidence: grappstar_base.ino:37 `static const int RING = 24;` and :92-93 `int next = (ringHead + 1) % RING; if (next == ringTail) { ringDrops = ringDrops + 1; return; }` (capacity 23). base_shim.cpp:128 `if (evq.size() >= 24) { evDrops++; return; }      // the sketch's ring (RING 24): drop the newest`.

Fix: Change base_shim.cpp:128 to `if (evq.size() >= 23) { evDrops++; return; }      // the sketch's ring: RING 24 slots, one kept empty = 23 events`.


## 8. land_short and c_tug/c_wrap_free cannot fail in the D22 order: tugging a free dart costs no force, and the total wrap draw is under the 1 m threshold

- File: `systest/base_shim.cpp:216`
- Severity: low (claimed medium); 1 of 2 skeptics confirmed
- Owner: systest

Scenario: land_short claims the wrap must 'tug the free dart, never jam against an anchored one'. In the BASE_SPEC 10.1 order (ref_land_short, and nominal/land_short once F4 is fixed) the dart is never anchored during WRAP: ANCHORED arrives only after `dart auto`, and anchorTautAt stays -1. Every laid turn then takes the free branch. That branch subtracts the line with no force on the drum and never blocks it, so wrapBlocks stays 0 for any landing_slack. c_tug compares d0 - d with 1.0 m, but the dart can be moved at most 7 x 2*pi*0.0205 = 0.90 m. landing_slack=0.3 therefore passes exactly like 0.9 or 0.0. On hardware, the last 0.6 m (with 0.3 m slack) must drag the dart over the ground through the wrap cup on the drum N20. Physics already puts the wrap at 30-37 % of an assumed N20 torque, so a dragged dart can stall the drum (DRUM_TIMEOUT mid-wrap, drum holding with partial wraps). The plant cannot produce that.

Evidence: base_shim.cpp:212-216 `if (S.anchorTautAt >= 0) { ... if (need > avail + 1e-9) { S.drumDeg = drumBefore; wrapBlocks++; } ... } else { S.lineOut = fmax(0.0, S.lineOut - need); dartTug += need; }`
harness.py:1025-1029 c_tug `return d0 - d < 1.0, ...`
harness.py:1020-1022 c_wrap_free `return n == 0`
harness.py:1279-1280 `sc(Scenario("land_short", "... the wrap must tug the free dart, never jam against an anchored one", landing_slack=0.3, d22=True))`

Fix: Give the shim the dart's position and drag. Add knobs `dartAt` (sent by Rig._world as dart_d after the shot) and `dartDragN` (Scenario knob, for example 3 N). In plantExtra's free branch, once S.lineOut - need < dartAt, the wrap is dragging the dart. If dartDragN x cup ring radius is above the N20's usable torque margin (params), block the drum (S.drumDeg = drumBefore; wrapBlocks++); otherwise slow drumDpsTrue in proportion. Change c_tug to check the drag against the D22 design figure (landing slack + 0.9 m - slack) instead of a constant 1.0 that cannot be exceeded. Add a land_short variant with a high dartDragN that is expected to raise DRUM_TIMEOUT and recover.

> Skeptic (confirmed): Note: local w/review (2e68a51) is behind origin/base-v1 and has none of this code. The cited code is on origin/base-v1 (863d58d) at base_shim.cpp:207-217 and harness.py:1035-1044 and 1310-1311, with slightly different line numbers from the claim. I checked it there.

What holds up:
(1) In plantExtra's free branch (anchorTautAt < 0) the shim only subtracts line and counts dartTug. It never touches drumDeg or wrapBlocks, and it applies no load.
(2) landing_slack is used in exactly one place: harness.py:352, where dart_d0 = lineOut_at_shot - landing_slack. It never reaches the plant. anchorTautAt
>
> Skeptic (refuted): I checked the code on origin/base-v1. The local w/review HEAD is older and does not contain these lines. plantExtra (base_shim.cpp:207-221) and c_wrap_free/c_tug (harness.py:1035-1044) do say what the claim quotes. The conclusion drawn from them does not hold.

(1) The checks can fail. Their job is to catch the wrong order, not to model drag. The harness sets anchorTautAt only when the dart reports ANCHORED (harness.py:367-373). If the firmware, mission or driver anchors before `wrap` (the F4/pre-D22 order), the anchored branch runs. With landing_slack=0.3 the avail value (0.3 m) is less than 
>

## 9. Reference driver swallows every error while waiting for RESET after FIRED, including a FAULT

- File: `systest/harness.py:602`
- Severity: low (claimed low); unverified (low)
- Owner: systest

Scenario: 10.1 step 5 waits for `FIRED`, then `RESET`. Mutate base_logic so the launch train never emits RESET, or raises STALL during the post-shot reset. wait_event raises (a timeout or BaseError on FAULT), s_flight catches it with `except Exception: pass`, and the driver carries on to `set line_m` / `wrap`. The ref_* scenarios still pass, because recover() clears any latched fault on the next step.

Evidence: harness.py:598-603 `def s_flight(self): ... try: self.wait("RESET", 30, getattr(self, "fire_mark", 0)) except Exception: pass`. comms/base.py:510 'Raises BaseError for: a FAULT event ...'.

Fix: Let the exception propagate (remove the try/except), or catch only the case where RESET was already consumed before fire_mark. On a timeout or FAULT, raise RuntimeError('no RESET after FIRED') so the step fails and is counted.


## 10. 10.1 step 14 order (stop, then dart release) is never checked; the reference driver itself releases before stop

- File: `systest/harness.py:648`
- Severity: low (claimed low); unverified (low)
- Owner: systest

Scenario: c1_tokens drops every `stop` (line 985), so mission.py could send dart /release while the base is still in HOLD or REEL at the limit, or skip the stop entirely, and C1_SEQ still matches. RefMission.s_release, the 'binding sequence' driver, calls d.release() before b.cmd('stop'), which is the reverse of 10.1. Its own C1 check is built from ref.tokens, a list the harness writes itself, and those tokens do not include stop either. So the reverse order passes, as in ref_nominal (dart RELEASING logged together with ABORTED HOLD).

Evidence: harness.py:646-649 `if st.get("phase") != "RELEASING": self.d.release()` then `self.b.cmd("stop", timeout=2.0)`. harness.py:985 `if c in ("ping", "status", "get", "clear", "stop", "estop", None): continue`. BASE_SPEC 10.1 step 14: '`stop`, then dart `/release` | `ABORTED`, then dart phase RELEASING ... the worm holds the line while the dart lets go'.

Fix: In s_release send `self.b.cmd("stop")`, wait for ABORTED (or the state to leave REEL/HOLD), then call d.release(). Keep `stop` tokens in c1_tokens (collapse repeats) and change C1_SEQ to `(reel tension > )+stop > dart release > (reel stop > stop > )?unwrap > retrieve > mode LAUNCH`. For the ref driver, compare tokens parsed from base.log RX lines (the wire) rather than ref.tokens.


## 11. `tidy ran` only checks that the mission entered the TIDY step; it never checks that the base took up the slack

- File: `systest/harness.py:887`
- Severity: low (claimed medium); 1 of 2 skeptics confirmed
- Owner: systest

Scenario: Mutate base_logic.h line 983 to `tidyUntil = now;` (or reply ok and emit TIDIED at once without driving the take-up). TIDIED arrives, the mission logs 'line tidied', and c_tidy passes (TIDY in steps, no 'no tidy' message). The 0.9 m of dart-side slack stays on the line through the REEL fly-in. No check reads the plant `slack` after TIDY, so the 10.1 step 11 behaviour is untested. A mission.py tidy() that accepts LIMIT as success (mission.py:516) also passes.

Evidence: harness.py:882-887 `ran = "TIDY" in res["steps"]; skipped = any("no tidy" in m for m in msgs); return ran and not skipped`. The shim exposes plant truth `"slack"` / `"maxSlack"` (base_shim.cpp:275), which harness.py never checks. ref_nominal drops this check entirely (line 1107).

Fix: Record the plant snapshot when TIDIED arrives (or at the next step change, rig.step_plant['WRAP'] or ['REEL_FLYIN']) and require plant slack <= 0.1 m with anchorTautAt >= 0 and lineOut within 0.1 m of anchorTautAt. Keep this check in the ref_* scenarios as well.

> Skeptic (confirmed): The core claim holds, but several of its citations are wrong. systest/harness.py has 1004 lines. The check is c_tidy at lines 626-631, not 882-887, and there is no ref_nominal in the file (grep finds nothing). The substance is still right. c_tidy passes when "TIDY" is in res["steps"] and no mission log line contains "no tidy". It reads no plant state. A grep shows harness.py never reads the plant's "slack" or "maxSlack" fields, even though base_shim.cpp:251-256 exposes them. The only slack-related harness code is the tow-hook trigger (lines 514-537), which adds slack rather than checking it. N
>
> Skeptic (refuted): Part of the claim is true. c_tidy in systest/harness.py (lines 626-631, not 887) only checks that "TIDY" appears in res["steps"] and that no "no tidy" message was logged. The rest of the claim does not hold up.

1. The scenario depends on plant state that does not exist. In firmware/base/test/sim.h the plant variable `slack` only grows when the line unwraps (sim.h:153-155: w1 < w0). Nothing sets it on landing or anchoring, so it is 0 through LAND, WRAP, anchor and TIDY. Take-up only lowers `slack` toward 0 (sim.h:123). The only take-up path that changes lineOut needs anchorTautAt < 0 (sim.h:13
>

## 12. dart_never_anchors can never pass: 'no TOW' checks the whole history, but after a successful reshoot the mission must tow

- File: `systest/harness.py:1173`
- Severity: low (claimed low); unverified (low)
- Owner: systest

Scenario: Correct behaviour: the check pull catches the non-holding anchor, the operator chooses 'Release and reshoot', unjam_dart_retry sets anchor_ok=True, the second dart anchors, and the mission goes through CHECK and TOW to DONE. mission.history is kept across reshoots (mission.py:137/273), so 'TOW' is in res['steps'] and the check fails. If the reshoot never anchors, max_reshoots aborts the mission and c_done fails instead. Either way the scenario is red for a correct system and green for none, so it cannot tell a regression apart.

Evidence: harness.py:1173 `("no TOW on the dragging dart", lambda r, res: ("TOW" not in res["steps"], res["steps"]))` together with `("mission DONE (dart retrieved after the failed check)", c_done)` and policy 'Release and reshoot'. mission.py:970-974 reshoot() keeps self.history.

Fix: Check that no TOW step comes before the first reshoot: steps = res['steps']; first_fire2 = index of the second 'FIRE' (or len(steps) if none); ok = 'TOW' not in steps[:first_fire2] and r.app.mission.reshoots >= 1.


## 13. snag_retrieve's 'LIMIT seen in RETRIEVE' counts LIMIT over the whole run, so the tow's own LIMIT events satisfy it

- File: `systest/harness.py:1183`
- Severity: low (claimed low); unverified (low)
- Owner: systest

Scenario: Every mission tows with `reel until tension`, which ends in LIMIT (three LIMIT events in TOW in ref_nominal). If base_logic stops honouring max_n for `reel until stop` (the snag then ends in an OVERCURRENT FAULT, or the reel just stalls), the 'LIMIT seen in RETRIEVE' check still passes on the TOW LIMITs.

Evidence: harness.py:1183 `("LIMIT seen in RETRIEVE", c_evt("LIMIT"))`. c_evt (lines 845-849) is `k = res["events"].count(name); return k >= n` over all events, with no step filter. ref_nominal timeline lines 40-44: LIMIT x3 during TOW.

Fix: In _on_evt, store the current step with each event (dict(e2, _step=self.step())). Make the check `any(e.get('evt')=='LIMIT' and e.get('_step')=='RETRIEVE' for e in r.events)`, and also require that no FAULT occurred in RETRIEVE.


## 14. no_angle_sensor ANDs two mutually exclusive outcomes ('DONE in SW_INDEX mode' and 'explicit refusal'), so it cannot pass

- File: `systest/harness.py:1187`
- Severity: low (claimed low); unverified (low)
- Owner: systest

Scenario: The description says 'the mission must run or refuse clearly'. If fw-base implements D12, the mission ends DONE with no ANGLE_SENSOR prompt or error, and the 'refusal is explicit' check fails. If the firmware refuses with ANGLE_SENSOR, c_done fails. Neither correct behaviour can turn the scenario green.

Evidence: harness.py:1186-1189 `checks=[("mission DONE in SW_INDEX mode", c_done), ("refusal is explicit (ANGLE_SENSOR)", lambda r, res: (any("ANGLE_SENSOR" in (p["text"]) ...) or "ANGLE_SENSOR" in str(res["error"]), ...)), ...]`. collect() sets res['pass'] = all(c['ok'] ...) (line 494).

Fix: Replace the two checks with one: ok = c_done(r,res)[0] or (res['mission_state'] in ('ABORTED','FAILED') and ('ANGLE_SENSOR' in str(res['error']) or any('ANGLE_SENSOR' in p['text'] for p in res['prompts'][:2]))). Once DECISIONS records which branch is binding (implement D12 or require the magnet), keep only that branch.


## 15. nogo_reshoot keeps the single-shot C1 regex, which cannot match the two-shot NO_GO path that 10.1 allows

- File: `systest/harness.py:1275`
- Severity: low (claimed low); unverified (low)
- Owner: systest

Scenario: nogo_reshoot uses the default nominal_checks(), which include c_c1_sequence. A correct NO_GO run per 10.1 step 7 goes `aim > cock > arm > fire > set line_m > ... unwrap/retrieve > mode LAUNCH > aim > ... > mode LAUNCH`, two shots in one mission log. C1_SEQ is anchored ^...$ around exactly one shot, so the check always fails and the scenario is red even when the system is correct.

Evidence: harness.py:1275-1277 `sc(Scenario("nogo_reshoot", ..., extra_checks=[("two shots", ...)]))`, so checks default to nominal_checks() (Scenario.__init__ line 813). harness.py:967-968 C1_SEQ matches one aim..mode LAUNCH cycle. BASE_SPEC 10.1 step 7: 'NO_GO: skip 9 to 13, go to 14 (reel in on the capstan if wrapped, else 16)'.

Fix: For nogo_reshoot, split the tokens at each `mode LAUNCH` that is followed by `aim`. Require the last segment to match C1_SEQ and the earlier segments to match a NO_GO pattern: `^(mode LAUNCH > )?aim > (cock > )?arm > fire > set line_m > ((mode NEUTRAL > )?wrap > mode REEL > reel stop > stop > unwrap > )?retrieve > mode LAUNCH$`.


## Refuted (dropped)

- `systest/harness.py:892` c_wrapped accepts 3.5 turns although the build lays 7 (the same stale threshold FINDINGS F11 calls a bug)
