# Round 1 B: dart firmware (fw-dart)

Reviewed at base-v1 commit 0bfd59a. Method: independent finder subagents, one per lens; every finding of
medium severity or above was checked by two skeptic subagents told to refute it; kept only if fewer than
two refuted it. Low findings are listed unverified. Severity is the finder's, lowered when both skeptics
agreed it is real but rated it lower.

Confirmed: 5. Refuted and dropped: 0.

| # | Sev | File:line | Owner | Finding |
|---|---|---|---|---|
| 1 | medium | `firmware/dart/kedge_dart/kedge_logic.h:401` | fw-dart | Holding both directions of one motor bypasses the 8 s hold cap and stall detection, so the motor runs with no limit |
| 2 | medium | `firmware/dart/kedge_dart/kedge_logic.h:428` | fw-dart | A 'timeout' landing after a real impact freezes the flight log about 15 s after launch, so the launch, the flight and the impact are all lost from the 12 s ring |
| 3 | medium | `firmware/dart/kedge_dart/kedge_logic.h:463` | fw-dart | One failed INA219 read on the tick a normal phase timer ends aborts /auto or /release with SEQ_TIMEOUT |
| 4 | medium | `jetson/grappstar/comms/dart.py:214` | jetson | Jetson gives up on /flightlog for good on the dart's 409 'busy' refusal, which happens every shot with auto_anchor_on_landing=True and whenever the dart is still in FLIGHT |
| 5 | low | `jetson/grappstar/cli.py:179` | jetson | CLI 'dart release' reports success when the release was aborted by /stop (phase IDLE, seq aborted) |

## 1. Holding both directions of one motor bypasses the 8 s hold cap and stall detection, so the motor runs with no limit

- File: `firmware/dart/kedge_dart/kedge_logic.h:401`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-dart

Scenario: On the web page, two fingers hold the 'deploy down' and 'deploy up' buttons (they are separate pointer ids, so both stay in `held`). The same happens with Jetson Dart.hold(['1','!']) or anything else that sends '1' and '!' alternately. Each key flips the direction, so `cont` is false on every request: heldStart[m] is reset each time and the manualMaxMs (8 s) / manualMaxNoInaMs (2 s) cap is never reached. drive() also resets motT0[m] on every flip, so `settled` (now - motT0 >= 250 ms) is never true and the manual stall check never runs. A scratch run (/tmp/claude-0/review-scratch-B/t1.cpp) alternated '1' and '!' every 75 ms for 20 s: the deploy motor was driven for 20000 of 20000 ms, stallLock stayed 0, and no stall stop happened. If the pods are jammed, the N20 sits at stall or reversal current until the user lets go. Without the INA219 nothing limits it, because the trip also needs a current reading.

Evidence: kedge_logic.h:395 `bool repeat = now - lastCmd[m] < win; lastCmd[m] = now; lastRunMs[m] = runMs;`
kedge_logic.h:401 `bool cont = repeat && lastKeyDir[m] == d; lastKeyDir[m] = d;`
kedge_logic.h:402 `if (!cont) heldStart[m] = now;`
kedge_logic.h:311 `if (d && dir[m] != d) motT0[m] = now;`
kedge_logic.h:484 `if (now - motT0[m] < tu.surgeIgnoreMs) settled = false;`
kedge_logic.h:66 `uint32_t manualMaxMs = 8000; // a manual motor (held key or repeats) never runs longer`

Fix: Base the hold cap on the motor, not on its direction. In key() use `bool cont = repeat;` so a direction flip inside the repeat window does not reset heldStart[m]; lastKeyDir is then only needed for the motT0 rule. Also refuse a reversal while the motor is still being driven: `if (dir[m] && dir[m] != d) return "busy: release the other direction first";`, placed before drive(). Otherwise, keep motT0 when reversing inside the window. Either way the surge-ignore timer can no longer be restarted forever.

> Skeptic (confirmed): I could not refute this. I checked the code and ran a test, and the defect is real.

Code path (firmware/dart/kedge_dart/kedge_logic.h):
- key() at lines 395-406: `cont = repeat && lastKeyDir[m] == d`, and `heldStart[m] = now` whenever cont is false. So the hold cap (`used = now - heldStart[m]`) starts again on every direction flip.
- drive() at line 311 sets motT0[m] = now whenever the direction changes. The `if (cont) motT0[m] = keepT0` restore does not run, because cont is false.
- In tick(), the manual stall check (lines 484-487) needs `settled`, meaning now - motT0 >= surgeIgnoreMs (250 m
>
> Skeptic (confirmed): I could not refute this. I checked the code and reran the scratch test. In kedge_logic.h key() (lines 395-407), `cont = repeat && lastKeyDir[m] == d`, so a key in the opposite direction always gives cont=false. That sets heldStart[m] = now, so the cap check (`used = now - heldStart`) restarts on every flip. drive() at line 311 resets motT0[m] whenever the direction changes, and key() only puts keepT0 back when cont is true. So with flips every 75 ms, `settled` in the manual stall branch of tick() (line 484) never becomes true. The no-current check, which also keys off motT0, never runs either.
>

## 2. A 'timeout' landing after a real impact freezes the flight log about 15 s after launch, so the launch, the flight and the impact are all lost from the 12 s ring

- File: `firmware/dart/kedge_dart/kedge_logic.h:428`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-dart

Scenario: Armed log-only. Launch at t=5000 (40 ms over 8 g), 1.5 s of free fall, impact of 6 g at t=6540. The dart then lies on soil with something nudging it about every 0.8 s: a tether tug, swaying grass, or a branch. |a| briefly reaches 1.4 g, outside the 0.85..1.15 stillness band, but never goes over 3.5 g. The 1 s stillness timer keeps restarting, so Flight stays IMPACTED until flightTimeoutMs. At t=20005 it calls land(tImpact, "timeout"), which gives tLand=6555. handleFlight then computes fr=tLand+3000=9555. That time is already past, so the log freezes at once, at now=20005. A scratch test (/tmp/claude-0/review-scratch-B/t1.cpp) prints: tLaunch=5000 tLand=6555 how=timeout, frozen=1 count=600 oldest=8020 newest=20000, launch in log: NO. The actual flight was 1.5 s, yet /flightlog keeps only samples from t>=8020: 12 s of the dart lying on the ground, with no launch, no flight and no impact. This breaks C2 ('frozen from 2 s before launch ... until 3 s after landing') and API.md, which says samples are lost only 'in a flight longer than about 7 s'. vision/flight.py imu_events then finds no launch or impact runs. The FLYING timeout (no impact, never still) has the same problem: it freezes at tLaunch+18000.

Evidence: kedge_logic.h:138  `if ((int32_t)(now - tLaunch) >= (int32_t)flightTimeoutMs) return land(tImpact, "timeout");`
kedge_logic.h:148  `if (flying() && (int32_t)(now - tLaunch) >= (int32_t)flightTimeoutMs) return land(st == IMPACTED ? tImpact : now, "timeout");`
kedge_logic.h:428  `uint32_t fr = flight.tLand + 3000; log.scheduleFreeze((int32_t)(fr - now) > 0 ? fr : now);`
kedge_logic.h:181  `static const int LOG_N = 600;` (12 s at 50 Hz), while flightTimeoutMs = 15000 (line 104).
The freeze is only ever scheduled from the LAND event, and nothing freezes the ring before the oldest window sample (tLaunch-2000) is overwritten.

Fix: Make sure the ring never overwrites the start of the window. (1) On LAUNCH (handleFlight, line 423), also call `log.scheduleFreeze(flight.tLaunch - 2000 + (LOG_N - 1) * 20);`. That is the latest moment when the whole window from tLaunch-2000 still fits in the ring. (2) On LAND, schedule the earlier of the two times instead of tLand+3000: `uint32_t cap = flight.tLaunch - 2000 + (LOG_N - 1) * 20; uint32_t fr = flight.tLand + 3000; if ((int32_t)(fr - cap) > 0) fr = cap; log.scheduleFreeze((int32_t)(fr - now) > 0 ? fr : now);`. Keep the frozen log frozen through later LAND events, because FlightLog::offer already ignores samples once frozen. A normal flight (window under 12 s) is unchanged. A 'timeout' landing then still returns tLaunch-2000 .. tLaunch+9980, with the launch and the impact in it. Add a test_logic.cpp case like the scratch test: impact, nudges until the timeout, then require log.at(0).t <= tLaunch.

> Skeptic (confirmed): I could not refute this. I reproduced it independently with my own scratch program, /tmp/claude-0/review-scratch-B/skA/r.cpp. The program drives Kedge through onAccel, imuSample and tick: armed log-only, 8 g from t=5000 to 5040, 0.2 g until 6540, a 6 g impact, then 1.4 g blips of 40 ms every 800 ms. The output is: "landed (timeout)", tLaunch=5000, tLand=6550, frozen=1, count=600, oldest=8010, newest=19990. The launch and the impact are both missing from the frozen ring. (The t1.cpp in the scratch dir tests something else, motor timing, so it does not back the claim; my test does.)

Code check:
>
> Skeptic (confirmed): I could not refute this. The bug is real and I reproduced it myself.

Code (firmware/dart/kedge_dart/kedge_logic.h):
- In IMPACTED, any sample outside 0.85..1.15 g sets stillOn=false, so the 1 s stillness timer restarts (lines 139-142). A 1.4 g nudge does not count as a new impact because it is under impactG=3.5.
- After flightTimeoutMs=15000 from launch, Flight lands with "timeout" and tLand=tImpact (lines 138 and 148).
- handleFlight schedules the freeze at tLand+3000, or at `now` if that time has already passed (line 428).
- LAUNCH only calls log.window(tLaunch-2000) and never schedules a f
>

## 3. One failed INA219 read on the tick a normal phase timer ends aborts /auto or /release with SEQ_TIMEOUT

- File: `firmware/dart/kedge_dart/kedge_logic.h:463`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: fw-dart

Scenario: During /release, the drills-out lead timer (drillLeadMs = 3000) ends normally, but the INA219 read on that 5 ms tick returns NaN (one I2C error from motor noise; the sketch makes hwCurrent return NAN on any failed transfer). The !curOk branch treats every timer expiry in a sequence as a fault, so the release aborts with SEQ_TIMEOUT and every motor stops. A healthy read on that tick would have moved on to R_PODS. The same happens on the normal ends of footLeadMs (R_FOOT), footAfterMs (F_AFTER), deployMaxMs (D_DEPLOY) and drillAfterMs (D_AFTER, which aborts an otherwise finished anchor). A scratch run (/tmp/claude-0/review-scratch-B/t1.cpp, test 2) with one NaN at the drills-lead expiry gave `103000 motor time limit without a current reading` and `seq=aborted fault=SEQ_TIMEOUT`. The dart then sits half released, or half anchored, in FAULT, and the Jetson's wait_phase fails the mission step.

Evidence: kedge_logic.h:461-465:
`if (!curOk) {`
`  for (int m = 0; m < NMOT; m++) if (dir[m] && (int32_t)(now - until[m]) >= 0) {`
`    if (seqRunning()) { setFault(m == KICK ? F_KICK_TIMEOUT : F_SEQ_TIMEOUT, "motor time limit without a current reading", now); return; }`
`    drive(m, 0, 0, now);`
compare with the normal path, kedge_logic.h:472-476, where only the kick timer in K_SWING/K_PLANT/R_KICK is a fault and every other expiry is just `drive(m, 0, 0, now);` (the phase advances on `!dir[...]`).

Fix: In the !curOk branch, use the same expiry rules as lines 472-476. Fault only for `m == KICK && (seq == K_SWING || seq == K_PLANT || seq == R_KICK)` (F_KICK_TIMEOUT); otherwise call `drive(m, 0, 0, now)` and let the sequence switch advance on the next tick that has a good reading (R_DRILL/F_AFTER/D_AFTER/R_FOOT/D_DEPLOY all advance on `!dir[...]`). Keep the sequence-wide autoMaxMs/releaseMaxMs check, and keep the inaFailLimit rule (3 failed reads in a row = F_CURRENT_SENSOR) for real sensor loss.

> Skeptic (confirmed): The claim holds. In kedge_logic.h tick(), when the INA219 read fails (ma is NaN, which hwCurrent in kedge_dart.ino:83 returns on any failed I2C transfer), the !curOk branch at lines 461-466 raises a fault for any motor whose timer has expired while a sequence is running: F_KICK_TIMEOUT for the kick, F_SEQ_TIMEOUT for every other motor. It then returns. The normal path at lines 472-477 treats only a kick expiry in K_SWING/K_PLANT/R_KICK as a fault. Every other expiry just calls drive(m,0,0), and the sequence switch then moves on when it sees !dir[...]:
- R_DRILL to R_PODS when the drillLeadMs t
>
> Skeptic (confirmed): I could not refute this. The claim checks out against the code. In kedge_logic.h:461-466, when a tick has no current reading (curOk false), any motor whose `until` has passed during a running sequence faults with F_SEQ_TIMEOUT, or F_KICK_TIMEOUT for the kick, and the tick returns. The healthy path at lines 472-477 treats only the KICK timer in K_SWING/K_PLANT/R_KICK as a fault. Every other expiry there is just drive(m,0,0), and the switch then moves on when !dir[...] (F_AFTER, D_DEPLOY, D_AFTER, R_DRILL and the release phases).

hwCurrent in kedge_dart.ino returns NAN whenever an I2C transfer 
>

## 4. Jetson gives up on /flightlog for good on the dart's 409 'busy' refusal, which happens every shot with auto_anchor_on_landing=True and whenever the dart is still in FLIGHT

- File: `jetson/grappstar/comms/dart.py:214`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: Case 1: MissionConfig.auto_anchor_on_landing=True, so Mission.arm sends /arm?on=1&auto=1. On landing the dart starts /auto by itself (kedge_logic.h handleFlight -> startAuto) and stays ANCHORING for up to 35 s. Mission._sequence then runs LANDING_ZONE right after FLIGHT plus settle_s=2 s, and landing_zone calls dart.flightlog(). The firmware answers 409 'refused: busy (motors running, armed or in flight)' because K.seqRunning() is true. Dart.flightlog re-raises DartRefused at once, without retrying, and landing_zone catches it as DartError ('no flightlog'). So every shot in this mode gets no IMU log: no flightlog_shotN.json and no IMU launch/impact/v0 for analyse(). The frozen log still sits on the dart, but nothing fetches it later. Case 2 (default auto=0): wait_landing ends on line+video before the dart has decided its landing. Example: the dart rocks on the tether and never reads 1 s still after the impact, so it stays in FLIGHT until its 15 s timeout. The /flightlog 2 s later is refused 'in flight' and the log is lost the same way.

Evidence: firmware/dart/kedge_dart/kedge_dart.ino:256-257
  bool busy = K.running() || K.seqRunning() || K.armed || K.flight.flying(); ...
  if (busy) { web.send(409, "text/plain", "refused: busy (motors running, armed or in flight)\n"); return; }
jetson/grappstar/comms/dart.py:214-215
            except DartRefused:
                raise
jetson/grappstar/mission.py:664  fl = self.dart.flightlog()   (called right after wait_landing + settle_s, never waits on dart phase)
jetson/grappstar/mission.py:675-676  except DartError as e: self.log.warn(f"no flightlog: {e}")
mission.py:97 auto_anchor_on_landing: bool = False   # True: press A so the dart anchors by itself

Fix: In Mission.landing_zone, before fetching, wait for a phase in which the dart serves the log: s = self.dart.wait_phase(("LANDED","ANCHORED","IDLE","FAULT"), 40.0, fail=(), cancel=self.cancel). This covers FLIGHT (at most 15 s) and an auto anchor run (at most 35 s); on timeout, carry on without the log. Also in Dart.flightlog, treat a DartRefused whose reason starts with 'busy' as retryable: sleep 1 s and retry until a deadline of about 40 s, re-raising only at the end, instead of `except DartRefused: raise`.

> Skeptic (confirmed): I couldn't refute this. The code does what the claim says.

1. Firmware refusal. handleFlightlog (kedge_dart.ino:254-257) sends 409 "refused: busy" whenever K.running(), K.seqRunning(), K.armed or K.flight.flying() is true.

2. Client gives up immediately. Dart.flightlog (dart.py:214-215) re-raises DartRefused at once; only DartError and ValueError get the 3-try retry. Mission.landing_zone (mission.py:663-676) catches it as DartError, logs "no flightlog" and calls analyse(fl=None). Nothing fetches the log again for that shot:
- anchor() (mission.py:730-757) waits for ANCHORED but never calls f
>
> Skeptic (confirmed): I could not refute this claim. The code paths check out.

Firmware (kedge_dart.ino:256-257): handleFlightlog answers 409 "refused: busy" whenever any of K.running(), K.seqRunning(), K.armed or K.flight.flying() is true.

Case 1 is concrete and happens on every shot in that mode.
- mission.arm sends auto=cfg.auto_anchor_on_landing (mission.py:505).
- On landing, kedge_logic.h:430-432 does `go = armed && autoOnLanding; armed=false; if (go) startAuto(now)`. That starts an anchor sequence, so seqRunning() stays true until the sequence ends. The limit is autoMaxMs=35000.
- wait_landing (mission.py:
>

## 5. CLI 'dart release' reports success when the release was aborted by /stop (phase IDLE, seq aborted)

- File: `jetson/grappstar/cli.py:179`
- Severity: low (claimed low); unverified (low)
- Owner: jetson

Scenario: Bench use: the dart has not landed since the last arm (landed=false). Run `python3 -m grappstar dart release`, then press STOP on the web page (or send /stop) mid sequence. The firmware sets seq=ABORTED, and phase() returns IDLE because the LANDED branch needs landed=true. wait_phase checks `s['phase'] in phases` before its aborted check, so it returns at once and the CLI prints 'IDLE' as if the legs were stowed. The operator then believes the dart is released while drills, foot or kick are part-way.

Evidence: jetson/grappstar/cli.py:178-179
            d.release()
            print(d.wait_phase(("IDLE",), 90, log=print)["phase"])
jetson/grappstar/comms/dart.py:273  if s["phase"] in phases and (want_seq is None or s.get("seq") == want_seq): return s
kedge_logic.h:254-256  if (landed && (seq == IDLE || seq == ABORTED)) return P_LANDED; if (armed) return P_ARMED; return P_IDLE;
(mission.release_dart correctly passes want_seq="released"; the CLI does not)

Fix: cli.py:179: print(d.wait_phase(("IDLE",), 90, log=print, want_seq="released")["phase"]). With want_seq set, wait_phase raises on seq 'aborted', as in Mission.release_dart.

