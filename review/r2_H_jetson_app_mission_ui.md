# Round 2 H: Jetson app, mission control, operator UI, comms robustness (jetson)

Reviewed at base-v1 commit c21a2d4. Method: independent finder subagents, one per lens; every finding of
medium severity or above was checked by two skeptic subagents told to refute it; kept only if fewer than
two refuted it. Low findings are listed unverified. Severity is the finder's, lowered when both skeptics
agreed it is real but rated it lower.

Confirmed: 11. Refuted and dropped: 0.

| # | Sev | File:line | Owner | Finding |
|---|---|---|---|---|
| 1 | medium | `jetson/grappstar/app.py:357` | jetson | act_set passes any key straight to base `set` (require_seated, host_timeout_s, trip_a, aim_offset, line_m, enc_sign), with no allowlist and no mission-running guard |
| 2 | medium | `jetson/grappstar/comms/base.py:519` | jetson | wait_event ignores link loss and the base reset whose HELLO is always missed after USB re-enumeration; FIRE then waits the full 30 s and the 15 s dart-video buffer drops the flight |
| 3 | medium | `jetson/grappstar/mission.py:293` | jetson | No 'launcher still drawn' warning after STOP during COCK, or after a fire that never released |
| 4 | medium | `jetson/grappstar/mission.py:427` | jetson | After an abort with the dart out or anchored there is no way to resume: preflight refuses every recovery state and the UI has no wrap, release or reel controls |
| 5 | medium | `jetson/grappstar/mission.py:621` | jetson | Reshoot with a failed dart arm takes the previous shot's LANDED and frozen flight log as the new landing |
| 6 | medium | `jetson/grappstar/mission.py:854` | jetson | Dart comes home during TOW: only Retry is offered, which pulls the seated dart into the cradle again, and there is no release or bring-home path |
| 7 | medium | `jetson/grappstar/mission.py:886` | jetson | RETRIEVE reel (until "stop") keeps acting on stale telemetry, so the dart is pulled into the cradle at full duty until the 30 N LIMIT |
| 8 | medium | `jetson/grappstar/ui/server.py:398` | jetson | Unauthenticated, CSRF-able action API: any LAN client or any web page open in the operator's browser can answer the FIRE prompts and drive the base and dart |
| 9 | low | `jetson/grappstar/mission.py:141` | jetson | Prompt ids restart at p1 in every Mission and an answer is not tied to the prompt text, so a stale card can confirm a different FIRE prompt (another pitch and range) |
| 10 | low | `jetson/grappstar/mission.py:866` | jetson | 'Keep towing' permanently raises the shared tow round limit for all later missions |
| 11 | low | `jetson/grappstar/ui/static/app.js:242` | jetson | The keyboard STOP (Space) does nothing while focus is in any input, such as right after typing a range and pressing Enter |

## 1. act_set passes any key straight to base `set` (require_seated, host_timeout_s, trip_a, aim_offset, line_m, enc_sign), with no allowlist and no mission-running guard

- File: `jetson/grappstar/app.py:357`
- Severity: medium (claimed high); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: Only drag_n, tow_limit_n, check_n and retrieve_limit_n are checked. Any other key falls through to `self.base.set(key, val)`, and unlike jog, aim and mode this is allowed while a mission runs. Three examples. (1) During AIM-to-FIRE, POST set {key:'aim_offset', val:10}: the base takes the new offset, so the confirmed pitch is no longer what the rail actually points at. (2) POST set {key:'require_seated', val:0}: the firmware arm interlock 'dart seated' is disabled (base_logic.h:854), so arm/fire works with the dart unseated. (3) POST set {key:'host_timeout_s', val:0}: the host watchdog is off, so the base keeps running motion if the Jetson dies. The page itself only sends drag_n and tow_limit_n, so nothing in the UI needs this pass-through.

Evidence: app.py:345-357 `def act_set(self, key: str, val: Any) -> dict: ... if key in ("drag_n", "tow_limit_n", "check_n", "retrieve_limit_n"): ... return ...` then `self.base.set(key, val)` with no `_busy()` check; firmware base_logic.h:854 `if (!strcmp(k, "require_seated")) { c.requireSeated = v != 0; return nullptr; }`

Fix: Replace line 357 with `return {"ok": False, "msg": f"unknown setting {key}"}`, so only the four mission settings can be set from the page. Base tuning and calibration stay on the CLI `base` command. If base keys from the page must stay, add `if self._busy(): return {"ok": False, "msg": "mission running"}` and an explicit allowlist that excludes require_seated, vbat_wired, enc_sign, host_timeout_s, trip_a, cock_trip_a and aim_offset.

> Skeptic (confirmed): I read the code and could not refute the claim. jetson/grappstar/app.py:345-358: only drag_n, tow_limit_n, check_n and retrieve_limit_n are checked. Any other key reaches `self.base.set(key, val)` with no allowlist and no `_busy()` check. Every other motion or config action (neutral, mode, jog, decock, dart_key, aim) does refuse with "mission running". App.action (app.py:212) dispatches any `act_<name>` with the POSTed JSON args. ui/server.py binds 0.0.0.0:8080 and only checks a token when `ui_token` is set, so anyone who can reach the page can send set {key, val}.

The firmware does not stop 
>
> Skeptic (confirmed): The core claim holds. In jetson/grappstar/app.py:345-358, act_set only handles drag_n, tow_limit_n, check_n and retrieve_limit_n itself. Every other key goes to `self.base.set(key, val)`, with no allowlist and no `_busy()` check. jog, decock and dart_key all have that check. App.action (app.py:212) dispatches any `act_<name>` from the HTTP API. The UI token is optional and defaults to "" (app.py:46, cli.py:39), so by default anyone on the WiFi can POST. The firmware cmdSet (base_logic.h:922-935) takes require_seated, vbat_wired, aim_offset and trip_a at any time, even while moving. Only enc_si
>

## 2. wait_event ignores link loss and the base reset whose HELLO is always missed after USB re-enumeration; FIRE then waits the full 30 s and the 15 s dart-video buffer drops the flight

- File: `jetson/grappstar/comms/base.py:519`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: A USB drop or an ESP32 reset (for example a BROWNOUT at the release, when the wiper draws its peak current) happens just after `fire` is sent. The FIRED event is lost while the port is closed. On a reset, the HELLO the firmware sends 'once at start-up' is also always lost: BaseClient waits reconnect_s (1 s) plus re-enumeration before it reopens the port. So the HELLO -> BASE_RESET path (base.py:533) never fires. BaseClient.wait_event has no link check: it waits out the whole timeout through the disconnect and the reconnect. Proven with /tmp/claude-0/review-scratch-H/wait_across_drop.py: the link was pulled at 0.5 s and restored at 0.8 s (reconnects=1), and wait_event('FIRED', 4) still raised a plain BaseTimeout after 4.0 s. Mission.fire's `except (BaseTimeout, BaseDisconnected)` (mission.py:582) is meant to recover fast but runs only after fire_timeout_s = 30 s. The dart camera keeps buffer_s=15 (app.py:132/146), and landing_zone keeps frames from t_fire - pre_s onward (mission.py:659). By then the buffer holds only frames from about 17-32 s after the shot: the flight video for the flight analysis and aerial mosaic is gone, and the landing step starts 30 s late. The same blind wait applies to AIMED/COCKED/WRAPPED/MODE across a reset: up to 60 s of dead time per step.

Evidence: base.py:519-538 the wait loop only scans `self._events` and does `self._cv.wait(min(left, 0.5))`. It never looks at `self.connected` or a link generation. base.py:249-257 `_set_connected(False)` fails only `_pending` commands. base.py:284 `self._stop.wait(self.reconnect_s)` comes before the reopen. PROTOCOL.md section 4: HELLO 'once at start-up'. mission.py:580-587 `e = self.wait_evt("FIRED", self.cfg.fire_timeout_s, m)` / `except (BaseTimeout, BaseDisconnected):` but mission.wait_evt (mission.py:222-233) only ever gets BaseTimeout. mission.py:84 `fire_timeout_s: float = 30.0`. app.py:132 and 146 `buffer_s=15`. Scratch output: `wait_event raised BaseTimeout after 4.0 s: no FIRED event in 4.0 s; reconnects=1`.

Fix: In BaseClient, add `self.link_gen = 0` and increment it inside `_set_connected` when the link goes down (`if self.connected and not up: self.link_gen += 1`, under _cv, followed by notify_all as now). Make event_mark() return the event sequence and record the link_gen at that moment (or store both). In wait_event, raise `BaseDisconnected("link lost while waiting for ...")` when `self.link_gen` differs from the generation at the mark, or when `not self.connected`; do the same in wait_tel. In Mission.wait_evt, let BaseDisconnected propagate, but first wait up to about 3 s for the link to come back (`self.base.wait_connected(3)`). Then, in fire(), the existing handler checks dart_is_out on fresh telemetry: wait for one tel line with `tel_t` later than the reconnect before calling dart_is_out. After a reconnect, also treat the session as possibly reset (BASE_RESET semantics): send `status` and check its phase.

> Skeptic (confirmed): I could not refute this. The core defect is real, though two details of the scenario are overstated.

What I confirmed in the code:
- **The wait loop ignores the link.** In base.py:519-538, wait_event only scans self._events and calls _cv.wait(min(left, 0.5)). It never reads self.connected or any link generation. wait_tel (540-555) has the same gap.
- **A link drop fails only pending commands.** _set_connected(False) (249-257) fails entries in _pending and nothing else. _run (259-284) counts reconnects, then waits reconnect_s=1.0 before it reopens the port.
- **Lost lines stay lost.** pyserial
>
> Skeptic (confirmed): The core of the claim holds in the code, though some details are overstated.

What I confirmed:
- BaseClient.wait_event (jetson/grappstar/comms/base.py:504-538) only scans self._events and waits on _cv. It never checks self.connected or any link generation.
- _set_connected(False) (base.py:249-257) fails only _pending commands. wait_tel is just as blind.
- Mission.wait_evt (mission.py:~226-242) catches only BaseTimeout, polls in 0.25 s slices, and runs out the full timeout. So the `except (BaseTimeout, BaseDisconnected)` in fire() (mission.py ~605) cannot see a BaseDisconnected from the wait. 
>

## 3. No 'launcher still drawn' warning after STOP during COCK, or after a fire that never released

- File: `jetson/grappstar/mission.py:293`
- Severity: medium (claimed high); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: (a) The operator presses STOP while COCK is running. wait_evt raises Aborted before line 491 sets shot['cocked']. _safe_stop then sends `stop`, and PROTOCOL says a stop during cock makes the shot phase UNKNOWN with the carriage part drawn. (b) FIRE is sent but the base faults before the release (BAD_STATE 'no release' or 'fire cycle took too long') and the operator picks Abort. fire_sent is set, so the warning is suppressed while the spring is still fully drawn. (c) cock() returns early at lines 486-487 when telemetry already shows cock>=0.999, so shot['cocked'] is never set. In all three cases the mission ends ABORTED or FAILED and never tells the operator to press Decock, so they walk up to a launcher whose drawn spring is held only by the worm. Reproduced with the repo harness (scratch tests in /tmp/claude-0/review-scratch-H/): (a) ended with error 'operator STOP', base status phase UNKNOWN, cock null, H open; (c-fire) mb.fail_next['fire']='BAD_STATE' then Abort ended with error 'operator chose Abort', phase COCKED, cock 1. Neither error mentions Decock. tests/test_mission.py::test_abort_at_fire_confirmation only covers the one case that works.

Evidence: mission.py:293 `if self.state in ("ABORTED", "FAILED") and self.shot.get("cocked") and not self.shot.get("fire_sent"):`
mission.py:486-491 `if t.get("cock") is not None and t.get("cock") >= 0.999:\n    return\n...\nself.wait_evt("COCKED", ...)\nself.shot["cocked"] = True`
PROTOCOL.md: `stop` ... "A stop during cock or fire makes the shot phase `UNKNOWN`"; section 6: LANDED-in-LAUNCH fault with phase not REST: "the drawn spring is held only by the wiper worm ... Recover with `clear`, then `cal decock` or `fire`".

Fix: In _run's finally block, decide from the base itself, not from shot flags. Read base.status() (best effort) and fresh tel. Warn when the mission ended ABORTED or FAILED and mode is LAUNCH and (status phase not in ('REST',) or tel cock is None or tel cock > 0.01 or sw_closed(tel,'H') is False). Fall back to the flag test only when the base does not answer. Also set self.shot['cocked'] = True before sending `cock` (line 489) and on the early return at 487. Add tests for STOP mid-cock and for a fire BAD_STATE followed by Abort.

> Skeptic (confirmed): I could not refute this. The claim is real.

The code says what the claim says. mission.py:293 shows the "still COCKED" warning only when shot['cocked'] is set and shot['fire_sent'] is not. shot['cocked'] is set only at mission.py:491, after the COCKED event arrives. The early return at 486-487, used when the spring is already cocked, never sets it. fire() sets fire_sent at line 602 straight after base.fire() returns, before FIRED is confirmed.

Spec: BASE_SPEC.md:192 says a stop or fault during cocking leaves the phase UNKNOWN and needs a `decock`. So the base is in exactly the state the warn
>
> Skeptic (confirmed): The defect is real. I checked the code and reran the reviewer's scratch tests.

**Code.** The only end-of-mission warning is at jetson/grappstar/mission.py:293. It fires only when `shot['cocked']` is set and `shot['fire_sent']` is not.
- `shot['cocked']` is set only at line 491, after `wait_evt("COCKED")` returns. A STOP during COCK raises Aborted before that line.
- The early return at lines 486-487 (telemetry already shows cock >= 0.999) also skips setting it.
- `fire_sent` is set as soon as `base.fire()` is sent (around line 609). So a fire that faults before the release, followed by Abort,
>

## 4. After an abort with the dart out or anchored there is no way to resume: preflight refuses every recovery state and the UI has no wrap, release or reel controls

- File: `jetson/grappstar/mission.py:427`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: The operator picks Abort at any prompt after FIRE, for example 'Abort' on the tow_assist prompt with the dart ANCHORED 15 m away and the capstan wrapped and holding. The mission ends ABORTED. Start mission builds a new Mission, and preflight adds 'capstan still wrapped', 'dart not seated' and 'dart busy (ANCHORED...)' as problems, then raises StepFailed with only Retry or Abort. Nothing in Mission starts from RELEASE or RETRIEVE. The operator page (index.html) has no wrap, unwrap, reel, mode REEL or dart release buttons, only a 1.5 s wiper jog, which cannot reel because reel needs WRAPPED. The dart keys route can send 'r' only through raw key entry. In the field the dart and line cannot be recovered from the operator UI after any post-fire abort.

Evidence: mission.py:426-429 `if (t.get("wraps") or 0) > 0.05:\n    problems.append("capstan still wrapped (unwrap first)")\nif sw_closed(t, "S") is False:\n    problems.append("dart not seated on the launcher (seated contact open)")`; mission.py:435-436 `if ds["phase"] not in ("IDLE", "ARMED"):\n    problems.append(f"dart busy ...")`; mission.py:453-454 `if problems:\n    raise StepFailed(...)`; app.py:255-262 act_start always runs a full mission from PREFLIGHT.

Fix: Add a recovery entry, Mission.recover() / App.act_recover, exposed as a 'Recover dart' button. It runs: RELEASE if the dart phase is ANCHORING, ANCHORED or FAULT (with dart_anchored=True), then BASE_NEUTRAL, WRAP, bring_home(), then LAUNCH_READY, reusing _do so Retry and Abort still apply. Alternatively, when preflight finds the dart out, wrapped or anchored, offer 'Recover dart' as a StepFailed choice that runs that same sequence.

> Skeptic (confirmed): I checked this against the code and firmware. The main claim holds, but some details in the scenario are wrong.

What I confirmed:
- Mission._run (mission.py:281-284) handles Aborted with only _safe_stop. Nothing runs RELEASE, RETRIEVE or UNWRAP.
- Every new run of Mission starts at preflight (app.py act_start). Preflight raises StepFailed, with only Retry or Abort, in three cases: wraps > 0.05, the seated contact open, or the dart phase not IDLE/ARMED (mission.py:426-436, 453).
- App has no unwrap, wrap, reel or dart-release act_* (grep of app.py), and index.html has no such buttons.
- The fi
>
> Skeptic (confirmed): I checked the claim against the code and it holds.

1. **What happens on abort.** Mission._run (mission.py:276-301) only calls _safe_stop() when it catches Aborted. It never releases the dart, unwraps or retrieves.

2. **Preflight blocks every recovery state.** Mission.preflight (mission.py:426-454) refuses each of the states left behind:
   - wraps > 0.05 ("capstan still wrapped")
   - seated contact open
   - dart phase not IDLE or ARMED (for example ANCHORED)
   
   It then raises StepFailed, which offers only Retry or Abort. The full sequence leaves preflight only by passing it, so a new m
>

## 5. Reshoot with a failed dart arm takes the previous shot's LANDED and frozen flight log as the new landing

- File: `jetson/grappstar/mission.py:621`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: Shot 1 lands, the verdict is NO GO, and the operator picks 'Retrieve and reshoot'. The dart is never released or re-armed, so it stays in phase LANDED with the shot-1 flight log frozen (API.md: the log 'stays until the next arm'). On shot 2, dart.arm() raises DartError (a WiFi glitch) and line 534 only logs it, or DartRefused is answered with 'Continue without'. dart_phase_at_arm is never set, so stale is False. On the first poll after FIRE, wait_landing sees phase LANDED and adds 'dart', so the landing is declared about 0.1 s after the fire while the dart is still in the air. landing_zone then fetches /flightlog, which is the frozen shot-1 log, and analyses it as shot 2. tell_line_out sends the base a line_m worked out from the wrong flight, and RETRIEVE stops at the wrong point.

Evidence: mission.py:533-534 `except DartError as e:\n    self.log.warn(f"dart not answering at arm: ...")` (dart_phase_at_arm unset)
mission.py:621 `stale = self.shot.get("dart_phase_at_arm") in ("LANDED",)`
mission.py:643 `if ds["phase"] in ("LANDED", "ANCHORING", "ANCHORED") and (seen_flight or not stale):`

Fix: Make LANDED count only after FLIGHT unless the arm is confirmed. Set stale = self.shot.get('dart_phase_at_arm') != 'ARMED', so any arm that failed or was skipped needs seen_flight (or a dart reboot) before the dart phase counts. In landing_zone, drop a flight log whose t_launch_ms is null or earlier than shot['dart_t_ms_at_arm'], or any log when the arm was not confirmed.

> Skeptic (confirmed): I could not refute this. I traced the path in the code and it happens as claimed.

1) How the dart ends up in LANDED. arm() calls dart.arm(..., auto=cfg.auto_anchor_on_landing), and that is log-only (auto=0) by default. Per firmware/dart/API.md, after a log-only landing the dart shows phase LANDED. The flight log is frozen and "stays until the next arm".

2) Nothing clears it before shot 2. If the operator picks 'Retrieve and reshoot', landing_zone returns "reshoot". _sequence (mission.py:378) then calls reshoot(). reshoot() only runs release_dart when the phase is ANCHORING/ANCHORED or dart_a
>
> Skeptic (confirmed): I could not refute this. I traced every step in the code.

1) The dart really does stay LANDED through a reshoot. landing_zone returns "reshoot", and _sequence calls reshoot() (mission.py:970-990). That runs release_dart only when the phase is ANCHORING/ANCHORED or dart_anchored is set, so a LANDED dart is never released. bring_home, wrap and base_neutral send nothing to the dart. _do only calls dart.safe_stop when dart_motion_possible(), which needs ANCHORING/RELEASING or more than 200 mA. PREFLIGHT, which would object to a non-IDLE/ARMED dart, sits outside the reshoot loop. In the firmware (
>

## 6. Dart comes home during TOW: only Retry is offered, which pulls the seated dart into the cradle again, and there is no release or bring-home path

- File: `jetson/grappstar/mission.py:854`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: The anchor pulls out during a tow round and the line drags the dart back until the seated contact closes. tow() raises StepFailed with choices=['Retry'], so _do offers only Retry or Abort. Retry runs tow() again: ensure_reel, then `reel until tension` at tow_limit_n (40 N) against a dart sitting in the cradle with its legs and drills deployed (dart phase ANCHORED). It reaches LIMIT, sw S is closed, and it raises the same StepFailed. That is an endless loop where every Retry is a 40 N pull into the launcher. The only way out is Abort, which leaves the dart ANCHORED (never released), the capstan wrapped and the base in HOLD, all states the next mission's preflight refuses. CHECK has the matching case and offers 'Release and reshoot' (lines 820-823), but TOW does not, and _sequence has no handler for it after TOW.

Evidence: mission.py:850-854 `if e.get("evt") == "SEATED" or sw_closed(self.tel(), "S"):\n    self.stop_settle()\n    raise StepFailed("the dart came home during the tow: the anchor pulled out",\n                     choices=["Retry"])`
mission.py:405-407 `self._do("TOW", self.tow)\nself._do("RELEASE", self.release_dart)\nself.bring_home()` (no branch on the TOW return value)

Fix: Raise StepFailed(..., choices=['Release and bring home']) at line 853. In _sequence, read r = self._do('TOW', self.tow); if r == 'Release and bring home', run RELEASE (dart_anchored is still True), then bring_home() (RETRIEVE returns at once when seated), then LAUNCH_READY, and return. Do not offer a plain Retry when S is closed.

> Skeptic (confirmed): I checked the claim against the code and it holds. In mission.py:850-854, tow() raises StepFailed(..., choices=["Retry"]) when the dart comes home. _do (lines 332-334) swaps the default choices for list(extra) + ["Abort"], so the operator sees only Retry and Abort. Retry calls self._clear_fault() and then tow() again from the top: ensure_reel, then base.reel(reel_duty, max_n=tow_limit_n=40, until="tension"). PROTOCOL.md:84/99 says a reel with until "tension" does not stop on SW_SEATED. Only until "seated" does that and emits SEATED. So the firmware winds the capstan until it reaches LIMIT at u
>
> Skeptic (confirmed): I checked this against the code and the spec, and the defect is real.

1. Mission code. tow() at jetson/grappstar/mission.py:850-854 raises StepFailed with choices=["Retry"] when SEATED arrives or sw S is closed. _do (lines 332-334) replaces the default choices with the extra ones, so the prompt shows only ["Retry", "Abort"]. Choosing Retry runs tow() again from the top. ensure_reel() does nothing because the capstan is still wrapped, and then it sends `reel` with until="tension" and max_n=tow_limit_n (40 N).

2. Firmware. base_logic.h cmdReel (line ~799) only short-circuits on SW_SEATED when 
>

## 7. RETRIEVE reel (until "stop") keeps acting on stale telemetry, so the dart is pulled into the cradle at full duty until the 30 N LIMIT

- File: `jetson/grappstar/mission.py:886`
- Severity: medium (claimed high); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: During RETRIEVE the mission starts `reel until="stop"` (line 882). The real firmware stops that reel only on LIMIT (max_n = retrieve_limit_n, 30 N) or reelMaxMs (240 s). It does NOT stop on SW_SEATED: base_logic.h:1176 stops only when `until == U_SEATED`. So the Jetson is the only thing that stops the reel at approach_m, and it does so from `self.base.tel` without checking its age. If telemetry stops arriving (USB stall, cable bump, CDC re-enumeration while the base keeps running), the loop keeps reading the last line_m (above approach_m) and never sends stop. Proven with /tmp/claude-0/review-scratch-H/retrieve_stale.py (MockBase with seat_ends_any_reel=False to match the firmware, line 3 m, telemetry silenced right after the reel started). The base reeled 2.7 m -> 0 m, the dart hit the cradle, tension rose to 30.1 N and the base stopped with LIMIT. Meanwhile the Jetson still showed line 2.73 m with tel_age 3 s, and the mission ended with the misleading error 'snag while retrieving (30.1 N)'. If the link is fully lost instead, the operator's STOP (app.act_stop -> base.safe_stop) cannot reach the base either. host_timeout_s is never set (it stays at the firmware default 0), so nothing stops the motor short of LIMIT or the 240 s timeout.

Evidence: mission.py:882 `self.base.reel(self.cfg.retrieve_duty, max_n=self.cfg.retrieve_limit_n, until="stop")`; mission.py:886-893 `tt = self.tel()` / `cur = tt.get("line_m")` / `if cur is not None and cur <= self.cfg.approach_m: self.base.stop()`. There is no check of base.tel_t or base.connected. mission.py:219-220 `def tel(self): return self.base.tel or {}`. comms/base.py:331-332 tel and tel_t are kept as they were across disconnects. Firmware base_logic.h:1176 `if (until == U_SEATED && rose(SW_SEATED)) {...}` is the only seated stop; config.h:148 `float hostTimeoutS = 0;`. No `host_timeout_s` appears anywhere under jetson/grappstar. Scratch output: `t+2.7s base state=REEL line=0.00 tension=12.6 dart=seated ... jetson tel line=2.73 tel_age=2.76`, then `StepFailed('snag while retrieving (30.1 N)...')`.

Fix: In the retrieve loop (mission.py:884-909), check telemetry age on every pass: `if time.time() - self.base.tel_t > 0.5 or not self.base.connected: self.base.safe_stop(); raise StepFailed("base telemetry stale during retrieve: stopped")`. A Mission.fresh_tel() helper that raises on stale data, used by every tel-driven motion loop, does the same job. Also make the base stop by itself when the Jetson goes silent: in preflight, call `self.base.set("host_timeout_s", 1.5)` (restore it at the end of the mission if you want), and give BaseClient a keep-alive (e.g. send `ping` every 0.5 s from a small thread while connected). Then a lost link or a dead Jetson app stops the reel within 1.5 s.

> Skeptic (confirmed): I checked the claim against the code and it holds up, though the scenario and the fix both need adjusting.

What the code does:
- mission.py:910 (the claim says 882; the line numbers have shifted) starts `reel until="stop"` on the capstan.
- Firmware tickReel (base_logic.h:1267) stops on SW_SEATED only when `until == U_SEATED`. With U_STOP the capstan branch also skips the seat slow-down at line 1287. So the only stops left are the tension limit (frac>=1, then enterHold at 30 N) and reelMaxMs = 240 s (config.h:185).
- The Jetson loop (mission.py ~913-937) reads `self.base.tel` through `tel()` 
>
> Skeptic (confirmed): I could not refute it. The claim's line numbers are a little off (the code is at mission.py:909-935), but the code does what the claim says.

What I checked:
- **Jetson loop.** `retrieve()` sends `reel(retrieve_duty, max_n=retrieve_limit_n, until="stop")`. The loop then reads `self.tel()`, which is just `self.base.tel or {}`. It never checks `base.tel_t` or `base.connected`. The only age check on `tel_t` in mission.py is at line 361 (fault clear).
- **Firmware.** In `tickReel` (base_logic.h:1266-1290), the seated stop runs only when `until == U_SEATED && rose(SW_SEATED)`, and the slow-down nea
>

## 8. Unauthenticated, CSRF-able action API: any LAN client or any web page open in the operator's browser can answer the FIRE prompts and drive the base and dart

- File: `jetson/grappstar/ui/server.py:398`
- Severity: medium (claimed critical); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: The default `grappstar serve` binds to 0.0.0.0 and has no token (cli.py:280 `--host default="0.0.0.0"`, cli.py:292 token optional, app.py:46 `ui_token: str = ""`, and settings.example.json sets none). do_POST never checks Content-Type, Origin or Referer, so a cross-site `fetch(url,{method:'POST',mode:'no-cors',body:...})` or a `<form enctype=text/plain>` reaches app.action. Prompt ids are guessable (p1, p2, ...). Once the mission shows 'Second confirmation: FIRE NOW?', any device on the rover WiFi, or a page the operator has open in another tab, can POST /api/action/answer {prompt_id:'pN', choice:'FIRE NOW'} for N=1..30. The launcher fires and the UI confirm() dialogs never come into it. The same route reaches jog, decock, dart_key 'A' (arm), aim and set require_seated=0. I confirmed this with /tmp/claude-0/review-scratch-H/csrf.py: a text/plain body `{"choice":"=","prompt_id":"p5","choice":"FIRE NOW"}` sent with Origin http://evil.example was accepted, and the server called app.action('answer', {'choice':'FIRE NOW','prompt_id':'p5'}).

Evidence: server.py:398 `m = re.fullmatch(r"/api/action/([A-Za-z0-9_\-]{1,64})", path)` ... 408-410 `if raw.strip(): args = json.loads(raw.decode("utf-8"))` (no Content-Type or Origin check); 218-219 `token = self.server.token; if not token: return {}`; 430 `def make_server(app, host: str = "0.0.0.0", ...`; cli.py:280 `s.add_argument("--host", default="0.0.0.0")`; app.py:46 `ui_token: str = ""`

Fix: In do_POST, reject a request (415) unless Content-Type starts with application/json, and reject it (403) when an Origin or Referer header is present and its host:port differs from the Host header. This blocks simple cross-site requests, and a JSON content type forces a CORS preflight that the server never grants. In cmd_serve, when no --token is given, generate one with secrets.token_urlsafe(12) and print the http://<ip>:port/?token=... URL, so the page is never open without a token. Optionally make prompt ids unguessable (see the prompt-id finding).

> Skeptic (confirmed): I checked the code and the mechanism is real.

What the code does:
- cli.py:280 binds to 0.0.0.0 by default, cli.py:292 makes --token optional, and cli.py:39 passes `ui_token=a.token or ""`.
- server.py:217-219 `_check_token` returns {} (allow) when no token is set.
- do_POST (server.py:394-428) never checks Content-Type, Origin or Referer. It runs json.loads on any body and passes the result to app.action.
- app.action dispatches any `act_*` method: start, answer, jog, mode, decock, and so on.
- mission.answer (mission.py:172-179) only checks that the prompt id is the current one and that the
>
> Skeptic (confirmed): The claim checks out against the code. server.py do_POST (lines ~393-427) calls _check_token, and _check_token (215-219) returns {} at once when self.server.token is empty. The token is empty by default: cli.py:292 `--token` has no default, cli.py:39 passes `ui_token=a.token or ""`, and app.py:46 sets `ui_token: str = ""`. The server binds 0.0.0.0 by default (cli.py:280, make_server). do_POST never looks at Content-Type, Origin or Referer. It runs json.loads on any body, so a cross-site text/plain simple POST (a no-cors fetch, or a form with enctype=text/plain) gets through to app.action. Dupl
>

## 9. Prompt ids restart at p1 in every Mission and an answer is not tied to the prompt text, so a stale card can confirm a different FIRE prompt (another pitch and range)

- File: `jetson/grappstar/mission.py:141`
- Severity: low (claimed medium); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: Mission 1 reaches 'FIRE at pitch 30.0 deg toward 4.0 m?' as id p3. The operator aborts from the laptop, enters a new target of 8 m and starts mission 2, whose third prompt is again p3 but reads 'FIRE at pitch 40.0 deg toward 8.0 m?'. Meanwhile a phone tab was locked or backgrounded and still shows the old p3 card. On unlock the operator taps Fire. The confirm() dialog shows the OLD text and blocks JS, so polling cannot refresh the card while it is open. They press OK, the POST {prompt_id:'p3', choice:'Fire'} matches mission 2's open prompt, and mission.answer accepts it. The second confirmation 'FIRE NOW?' contains no pitch or range, so the operator never sees the 40 deg / 8 m solution they are firing at.

Evidence: mission.py:141 `self._pid = 0` (per Mission instance, and app.py act_start creates a new Mission per run); mission.py:192 `p = Prompt(f"p{self._pid}", text, list(choices), danger, self.step)`; mission.py:175 `if p is None or p.id != prompt_id or choice not in p.choices: return False` (text not checked); app.js:409 `confirm("Are you sure: " + choice + "?\n\n" + (p.text || ""))` uses the card's cached text; mission.py:571 `self.ask("Second confirmation: FIRE NOW?", ...)`

Fix: Make prompt ids unique across missions and unguessable, for example `p = Prompt(f"p{self._pid}-{secrets.token_hex(4)}", ...)` or a module-level itertools.count combined with the run start time. Also put the pitch and range into the second confirmation text: `self.ask(f"Second confirmation: FIRE NOW at {p} deg / {rng:.1f} m?", ...)`.

> Skeptic (confirmed): The mechanism is real. Mission._pid starts at 0 in every Mission (mission.py:141), and app.py act_start builds a new Mission for each run, so ids repeat as p1, p2 and so on. Mission.answer (mission.py:175) checks only the id and that the choice is allowed, never the text. The prompt order before FIRE is nearly fixed: optional preflight warnings (456), the target/solution confirm (473), an optional dart-arm refusal (529), then FIRE (592). So two missions will often give their FIRE prompt the same id. In app.js:404-412 the click handler uses the card's cached p (id and text) for both the confirm
>
> Skeptic (confirmed): I checked the code and the mechanism is real. mission.py:141 sets `self._pid = 0` on each Mission. app.py act_start (lines 255-262) calls _make_mission(), which builds a new Mission for every run. ask() (lines 188-192) creates ids `p{n}` that start again at p1. answer() (line 175) checks only the id and that the choice is allowed, never the text, and app.py act_answer passes the call to whichever mission is current. The prompts before the FIRE prompt are the same in every run: preflight warnings only when there are any (line 456), then the target/solution prompt (473), then the arm-refused pro
>

## 10. 'Keep towing' permanently raises the shared tow round limit for all later missions

- File: `jetson/grappstar/mission.py:866`
- Severity: low (claimed low); unverified (low)
- Owner: jetson

Scenario: App passes the same MissionConfig object (self.cfg.mission) to every Mission. Each time an operator answers 'Keep towing', tow_max_rounds goes up by 10 on that shared object. A later mission in the same app session starts with 40, 50 and so on rounds before it asks, so the safety prompt drifts away from its configured value.

Evidence: mission.py:865-867 `if c == "Keep towing":\n    self.cfg.tow_max_rounds += 10\n    return self.tow()`; app.py:252 `return Mission(..., cfg=self.cfg.mission, ...)`

Fix: Keep the extension local: give tow() a max_rounds argument (default self.cfg.tow_max_rounds) and call self.tow(max_rounds=10) on 'Keep towing' instead of changing cfg. Or have App pass dataclasses.replace(self.cfg.mission) to each Mission.


## 11. The keyboard STOP (Space) does nothing while focus is in any input, such as right after typing a range and pressing Enter

- File: `jetson/grappstar/ui/static/app.js:242`
- Severity: low (claimed low); unverified (low)
- Owner: jetson

Scenario: On a laptop the operator types a target range into 'Range m' and presses Enter. The keydown handler clicks btn-target-range, but focus stays in the input. If the launcher or a jog then needs stopping, Space (the STOP key advertised in index.html:20, title 'Keyboard: Space') only types into the number field and no stop is sent, because the global handler returns early whenever focus is in an INPUT.

Evidence: app.js:242 `if (typing(e)) return;` placed before line 243 `if (e.key === " ") { e.preventDefault(); if (!e.repeat) { stopHolds(); action("stop", {}); } return; }`; index.html:20 `title="Stop all motion. Hold 1 s for ESTOP. Keyboard: Space"`

Fix: Handle STOP before the typing() check, using Escape everywhere and Space outside inputs: `if (e.key === "Escape" || (e.key === " " && !typing(e))) { e.preventDefault(); if (!e.repeat) { stopHolds(); action("stop", {}); } return; } if (typing(e)) return;`. Update the button title to name Escape.

