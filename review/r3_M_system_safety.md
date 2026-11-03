# Round 3 M: whole-system safety (people, LiPo, stored spring energy)

Reviewed at base-v1 commit 863d58d. Method: independent finder subagents, one per lens; every finding of
medium severity or above was checked by two skeptic subagents told to refute it; kept only if fewer than
two refuted it. Low findings are listed unverified. Severity is the finder's, lowered when both skeptics
agreed it is real but rated it lower.

Confirmed: 8. Refuted and dropped: 1.

| # | Sev | File:line | Owner | Finding |
|---|---|---|---|---|
| 1 | high | `jetson/grappstar/ui/static/app.js:21` | jetson | A drawn carriage with phase UNKNOWN shows as 'Cock -' on the operator page, and the safety rule 'loaded whenever cock above 0' misses it |
| 2 | medium | `docs/build/SYSTEM_ELECTRICAL.md:100` | docs | First power-on checks battery polarity only after the BTS7960 and MP1584 are already connected |
| 3 | medium | `docs/build/src/02_safety.md:40` | docs | The documented fallback when decock fails ('fire it with the tethered dummy') is refused by the firmware, leaving no unload path |
| 4 | medium | `jetson/grappstar/app.py:306` | jetson | The ESTOP button's drum-to-NEUTRAL safing is cancelled about 1 ms later by the aborting mission's own 'stop' |
| 5 | medium | `jetson/grappstar/cli.py:321` | jetson | TEST_PLAN fire tool 'grappstar base fire' does not exist, so the only documented confirmed way to fire in stages 4-6 is missing |
| 6 | medium | `jetson/grappstar/mission.py:524` | jetson | A host_timeout_s of 1.5 s left in flash by a killed mission aborts every later page Decock and drum mode change after 1.5 s |
| 7 | medium | `jetson/grappstar/ui/server.py:462` | jetson | STOP, ESTOP and dart_stop are refused with 403 after the Jetson app restarts, so the open page cannot stop anything until it reloads |
| 8 | low | `firmware/base/WIRING.md:216` | docs | Bench build connects MP1584 #2 to DRV8833 VM without re-measuring the buck; the first output check is with the DRV8833 already on it |

## 1. A drawn carriage with phase UNKNOWN shows as 'Cock -' on the operator page, and the safety rule 'loaded whenever cock above 0' misses it

- File: `jetson/grappstar/ui/static/app.js:21`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: Cock the launcher and send decock. A fault, a stop or a power cut arrives while the spring is still drawn. I reproduced this in the firmware sim: cock, then decock, then a STALL fault after 600 ms, then clear. The result is phase UNKNOWN with the carriage at 392.6 deg, about 95 percent of the full draw. Telemetry is `"cock":null,"sw":{"a":1,"l":1,"n":1,"S":1}` (no H). The page's Base table shows Cock as "-", the same as "no data". It never shows SW_HOME, and it does not show the mission's DRAWN warning when no mission is running. Build Guide section 2 tells the operator to "Treat the base as loaded whenever `cock` in telemetry is above 0". null is not above 0, so an operator can walk up to a nearly fully drawn 280 N carriage. The same happens after power is lost mid-cock: the boot sets phase UNKNOWN (base_logic.h:346), so cock is null.

Evidence: firmware/base/grappstar_base/base_logic.h:1195-1197: `float ck = phase == PH_REST ? 0 : ... : NAN; if (isnan(ck)) nul("cock");` (UNKNOWN gives null).
jetson/grappstar/ui/static/app.js:20-22: BASE_FIELDS has ["cock","Cock"] and no SW_HOME field. app.js:46-47: `function fmt(v) { if (v === null || v === undefined || v === "") return "-";`
docs/build/src/02_safety.md:60-61: "Treat the base as loaded whenever `cock` in telemetry is above 0."
Scratch run (/tmp/claude-0/review-scratch-M/tb scratch): `phase=5 home_closed=0 cockPh=392.6` and `tel: {..."cock":null,..."sw":{"a":1,"l":1,"n":1,"S":1}...}`

Fix: In app.js, add a launcher row computed from telemetry. When `tel.cock == null && !(tel.sw||{}).H`, or `tel.cock > 0`, show it in red as "DRAWN / LOADED" (or "UNKNOWN - may be drawn"). Show "home" only when cock == 0 and sw.H == 1. Also add ["H", "Carriage home"] next to "seated" in App.status (app.py:189). In docs/build/src/02_safety.md:60-61 (and the generated Build Guide line 204), change the rule to "Treat the base as loaded unless `cock` is 0 AND `H` (SW_HOME) is closed; `cock` null means the angle is unknown and the carriage may be drawn."

> Skeptic (confirmed): I could not refute this. The code says what the claim says.

Firmware (base_logic.h:1195-1197): `cock` is set only for REST (0), COCKED/FIRING (1), COCKING and RESETTING. Phase UNKNOWN falls through to NAN, and the telemetry then sends null. The code really does reach UNKNOWN with the carriage drawn. Line 1150 sets it when a COCK, FIRE or DECOCK is stopped by a fault or stop. Lines 346-347 set it at boot when the saved phase was COCKING, FIRING or RESETTING. Lines 224-225 set it when a save is taken mid-motion. In each case the firmware remembers the drawn angle in `phaseHint`, but it never se
>
> Skeptic (confirmed): I could not refute this. I checked each part of the chain in the code:
- Firmware: base_logic.h telBuild sets ck = NAN for every phase except REST, COCKED, FIRING, COCKING and RESETTING, so PH_UNKNOWN emits "cock":null. The firmware does go to PH_UNKNOWN with the carriage drawn. Line 1150 does this when a COCK, FIRE or DECOCK is stopped or faulted mid-cycle (phaseHint keeps the angle). Lines 346-347 do it at boot after power is lost mid-cock or mid-reset, and lines 359 and 507 in other cases. The firmware knows the carriage is drawn: line 763 refuses with LOADED_UNKNOWN when phase is UNKNOWN a
>

## 2. First power-on checks battery polarity only after the BTS7960 and MP1584 are already connected

- File: `docs/build/SYSTEM_ELECTRICAL.md:100`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: docs

Scenario: The XT60 Y harness and lead #2 are home-made from spare XT60s, so red and black can be swapped at lead #2. SYSTEM_ELECTRICAL step 5 says to fit the 20 A and 2 A fuses and plug the battery in, then meter B+ to B-, and to 'unplug at once' if the reading is negative. Once the battery is in reversed, the BTS7960's two half-bridges conduct through their body diodes (B- to low-side diode to M to high-side diode to B+), which shorts the pack until the 20 A fuse blows. MP1584 #2 also takes -12.6 V through its 2 A fuse, which destroys the buck IC. The meter check comes after the damage. WIRING.md Part B step 13 and Part D step 23 (line 238) have the same gap: the battery goes onto the BTS7960 with no polarity check at all.

Evidence: SYSTEM_ELECTRICAL.md:100 `5. Battery unplugged. Fit the 20 A fuse and the 2 A inline fuse. Plug the battery in. Meter on DC volts:`
SYSTEM_ELECTRICAL.md:101 `- BTS7960 B+ (red probe) to B- (black probe): battery voltage, positive. Negative means reversed: unplug at once.`
Step 2 (line 88 onward) checks for shorts only, never polarity.

Fix: Insert a new step before step 5, and the same step before WIRING.md Part D step 23: 'Battery unplugged, 20 A and 2 A fuses OUT. Plug the battery in. Meter DC volts, red probe on the battery-side clip of the 20 A fuse holder, black probe on BTS7960 B-: must read battery voltage, positive. Negative: unplug and swap the lead #2 wires at the XT60. Unplug the battery.' Only after that, fit the fuses and continue with the current step 5.

> Skeptic (confirmed): I couldn't refute this. The gap is real, though one detail of the scenario is off.

What the code and docs say:
- docs/build/SYSTEM_ELECTRICAL.md section 4, steps 1-4: pull the fuses, beep-test for shorts (step 2, including the Y harness male XT60 pins), visual check, and cell voltages on the balance lead. None of these steps checks polarity of the home-made Y harness or the lead going to the BTS7960.
- Step 5 (line 100) fits the 20 A and 2 A fuses, plugs the battery in, and only then meters B+ to B- ("Negative means reversed: unplug at once"). At that point the BTS7960 (P2) and MP1584 #2 IN (
>
> Skeptic (confirmed): I checked the claim against the docs and it holds. I am rating it low rather than medium.

**What the docs say:**
- SYSTEM_ELECTRICAL.md section 4, step 5 fits the 20 A and 2 A fuses and plugs in the battery. Only then does it meter B+ to B- ("Negative means reversed: unplug at once").
- Step 2 is a beep (short) test, which cannot detect polarity. Step 4 measures the balance lead, which checks the pack but not the home-made Y harness or lead #2.
- WIRING.md has the same gap. Part B step 13 wires the fused XT60 lead to B+/B- with no polarity check. Part D step 23 plugs the battery in and only t
>

## 3. The documented fallback when decock fails ('fire it with the tethered dummy') is refused by the firmware, leaving no unload path

- File: `docs/build/src/02_safety.md:40`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: docs

Scenario: `cal decock` faults while the spring is still drawn: STALL, "decock: SW_HOME never closed", or a timeout. raise() then sets phase UNKNOWN (base_logic.h:223-225). The operator follows the guide and tries to fire the launcher with the tethered dummy. In the sim, after clear: `arm` returns NOT_COCKED, `cock` returns LOADED_UNKNOWN, and a forward wiper jog returns WOULD_FIRE. The documented recovery cannot be done. The operator is left with a drawn 12 J carriage and a guide that has run out. Its only remaining instruction is "Never hand-release", so they may improvise: cal cock_rest force, hand lever, or prying the carriage.

Evidence: docs/build/src/02_safety.md:40-41: "If the carriage is drawn and `cal decock` fails: treat the launcher as loaded, keep the rail pointed down range, and fire it with the tethered dummy into the clear zone." The same text is in docs/build/src/13_troubleshooting.md:21 and GrappStar_Build_Guide.md:184-185 and 3468.
base_logic.h:785: `if (!((phase == PH_COCKED && !closed(SW_HOME)) || (phase == PH_REST && closed(SW_HOME))) || !gapOk) return "NOT_COCKED";`
base_logic.h:763: `if (phase == PH_UNKNOWN && !closed(SW_HOME)) return "LOADED_UNKNOWN";`
base_logic.h:907: jog in cockDir with PH_UNKNOWN returns "WOULD_FIRE".
Scratch run: `arm: {"ok":false,"err":"NOT_COCKED"}`, `cock: {"ok":false,"err":"LOADED_UNKNOWN"}`, `jog fwd: {"ok":false,"err":"WOULD_FIRE"}`

Fix: Replace the fallback in 02_safety.md:40-41, 13_troubleshooting.md:21 and the regenerated guide with steps the firmware allows: "If `decock` faults: `clear` and send `decock` again (it resumes backward from wherever it stopped). If it faults again: leave it loaded, rail pointed down range, lead #2 out, nobody in front or beside the springs, and fix the cause (STALL: binding in the worm or rack, checked from the side with the battery out; SW_HOME never closed: switch or wiring) before powering it again. Never use `cal cock_rest force` or the hand lever on a drawn carriage, and never hand-release the carriage." Alternatively (fw-base), add an explicit forced-fire path that the docs can name, but nothing like that exists today.

> Skeptic (confirmed): I could not refute this claim. I checked it against firmware/base/grappstar_base/base_logic.h.

1. A fault during decock puts the launch train in PH_UNKNOWN. raise() (lines 223-225) sets phase = PH_UNKNOWN whenever state == S_DECOCK, and keeps the last angle in phaseHint. A STALL, a timeout or "SW_HOME never closed" can each leave the carriage drawn with SW_HOME open.

2. Nothing that could fire the launcher is allowed from that state:
- fireCheck() (line 785) only accepts PH_COCKED with SW_HOME open, or PH_REST with SW_HOME closed. Both `arm` and `fire` go through it, so they return NOT_COCKE
>
> Skeptic (confirmed): The claim holds up in the code. When a fault happens during S_DECOCK, raise() sets phase = PH_UNKNOWN (base_logic.h:223-225). After the fault is cleared, the carriage is still drawn (SW_HOME open):
- fireCheck (line 785) only allows a shot from PH_COCKED with SW_HOME open, or from PH_REST with SW_HOME closed. Arm/fire therefore return NOT_COCKED.
- cmdCock (line 763) returns LOADED_UNKNOWN when phase is UNKNOWN and SW_HOME is open, so the carriage can never reach PH_COCKED. The only assignment of PH_COCKED is at line 1322 in the cock tick, which needs cmdCock to succeed first.
- A wiper jog in
>

## 4. The ESTOP button's drum-to-NEUTRAL safing is cancelled about 1 ms later by the aborting mission's own 'stop'

- File: `jetson/grappstar/app.py:306`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: A mission is running and the operator presses the ESTOP button (app.js:125, or a POST to estop). act_estop calls mission.abort(), which sets cancel and wakes the mission thread, then sends base.estop(). The firmware starts the safing drum move to NEUTRAL (cmdStop(true) -> startDrumMove, state S_DRUM). The woken mission thread raises Aborted and Mission._run calls _safe_stop() -> base.safe_stop(), which sends a plain 'stop'. cmdStop(false) halts the drum mid-move. Reproduced with the sim app (mission at the AIM prompt, base IDLE in LAUNCH): tx order was estop at t, stop at t+1 ms; the only event was ABORTED MODE 'stop'; the drum stayed at LAUNCH. The page still reports 'ESTOP: wiper off, drum to NEUTRAL if safe'. On hardware the drum can be left part-way between parks (BETWEEN).

Evidence: app.py:302-310 `self.mission.abort("operator ESTOP")` then `self.base.estop(timeout=0.5)` with message `"ESTOP: wiper off, drum to NEUTRAL if safe"`; mission.py:307-311 `except Aborted ... self._safe_stop()`; mission.py:346-347 `def _safe_stop(self): ok_b = self.base.safe_stop()`; base.py:494-499 safe_stop sends `stop` first; base_logic.h:1145-1151 cmdStop(false) runs `d.halt(now, io.drum)` and sets state S_IDLE (no safing move).

Fix: Make the mission's abort path keep the ESTOP. Add an `estop` flag to Mission.abort(why, estop=False) and set it from act_estop. In Mission._safe_stop (and in the _do failure path) send `self.base.estop(timeout=0.5)` instead of `safe_stop()` when the flag is set. Alternatively, act_estop can call mission.abort(), join mission.thread for up to 1.5 s, and only then send base.estop(), so the estop is the last command the base receives.

> Skeptic (confirmed): I could not refute this claim. I read the code and did not run the simulator.

- **app.py:302-310:** act_estop calls mission.abort("operator ESTOP") and then base.estop(timeout=0.5). abort() (mission.py:176-184) only sets cancel, briefly takes fire_lock and notifies _pcv. It does not wait for the mission thread to finish its cleanup.
- **mission.py:206-236:** The woken mission thread raises Aborted. This happens in wait_prompt, check_cancel or sleep, and also in the dart wait_phase paths with cancel=. Mission._run catches it (mission.py:307-311) and calls _safe_stop(). That calls base.safe_sto
>
> Skeptic (confirmed): I could not refute this. The claim holds.

1. app.py act_estop calls `self.mission.abort("operator ESTOP")` first. abort() (mission.py:176-184) sets cancel, passes through fire_lock and calls notify_all on _pcv. It does not wait for the mission thread. act_estop then sends `base.estop(timeout=0.5)` straight away.

2. The mission thread wakes within 0.2 s at most. ask() polls `_pcv.wait(0.2)`, and wait_tel and sleep check cancel. When it wakes it raises Aborted, and _run's `except Aborted` calls `_safe_stop()`, which calls `base.safe_stop()`. In comms/base.py:494-502, safe_stop sends `stop` fir
>

## 5. TEST_PLAN fire tool 'grappstar base fire' does not exist, so the only documented confirmed way to fire in stages 4-6 is missing

- File: `jetson/grappstar/cli.py:321`
- Severity: medium (claimed high); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: Builder at stage 4 step 3 or stage 5 step 1 runs `python3 -m grappstar base fire` as TEST_PLAN says. argparse exits with: invalid choice: 'fire' (choose from 'status', 'ping', 'monitor', 'cmd', 'mode', 'aim', 'stop', 'estop', 'clear', 'wrap', 'unwrap', 'reel'). I reproduced this in the base-v1 worktree. So the promised pre-fire screen (mode, pitch, cock, seated, then type FIRE) does not exist. A 3 s token cannot be typed by hand, so the builder has to improvise: a shell one-liner or a script that sends `arm` and then `fire` with the token. That fires the first 12 J spring shots with no state display and no explicit confirmation.

Evidence: jetson/grappstar/cli.py:321-322: `s.add_argument("what", choices=["status", "ping", "monitor", "cmd", "mode", "aim", "stop", "estop", "clear",
 "wrap", "unwrap", "reel"])` (no "fire"; cmd_base lines 115-160 have no fire branch).
docs/build/TEST_PLAN.md:6-7: "How to fire: `python3 -m grappstar base fire` on the Jetson. It shows mode, pitch, cock and seated, asks you to type FIRE, then sends `arm` and `fire` with the token at once"; also lines 95 and 117. The same text is in GrappStar_Build_Guide.md:3141, 3230 and 3252. docs/build/REPORT.md:124: "`grappstar base fire` is on w/jetson (merge pending)".

Fix: Add "fire" to the base `what` choices in cli.py and a branch in cmd_base that does five things. (1) Read `status` and telemetry and print mode, drum_pos, pitch_deg, phase, cock, sw.H, sw.G and sw.S. (2) Refuse unless mode == LAUNCH, phase is COCKED or REST, and there is no fault. (3) Print "Tether tied on? Nobody in front of the rail? Type FIRE" and continue only if input() returns exactly "FIRE". (4) Run `tok = c.cmd("arm")["token"]` and then `c.cmd("fire", token=tok)` with no pause. (5) Wait for the FIRED and RESET events and print them. Alternatively, merge the pending w/jetson implementation into base-v1 before the guide ships.

> Skeptic (confirmed): Not refuted: on base-v1, `python3 -m grappstar base fire` does not exist. jetson/grappstar/cli.py:321-322 lists the base 'what' choices as status, ping, monitor, cmd, mode, aim, stop, estop, clear, wrap, unwrap and reel, with no 'fire'. cmd_base (lines 115-160) has no fire branch, so argparse rejects the command. docs/build/TEST_PLAN.md lines 6-7, 96 and 118 tell the builder to fire with this command at stages 4-5. The operator page's mission FIRE step (mission.py, two confirmations, then arm and fire) runs only inside the full mission (aim, cameras, dart), so it does not cover the bench stage
>
> Skeptic (confirmed): The defect is real. On origin/base-v1 (52c3b03), and on w/review, jetson/grappstar/cli.py:321-322 lists the base `what` choices without "fire", and cmd_base (lines 115-160) has no fire branch. I ran `python3 -m grappstar base fire` from jetson/ and got exactly the reported argparse error: "invalid choice: 'fire'". On base-v1, docs/build/TEST_PLAN.md lines 6-7, 96 and 118 tell the builder to fire with this command at stage 4 step 3 and stage 5 step 1. docs/build/REPORT.md admits the tool is "on w/jetson (merge pending)". origin/w/jetson (7af8bd0) does have the branch: its cli.py lines 124-145 s
>

## 6. A host_timeout_s of 1.5 s left in flash by a killed mission aborts every later page Decock and drum mode change after 1.5 s

- File: `jetson/grappstar/mission.py:524`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: A mission is running, so _keepalive_start has set host_timeout_s=1.5. The firmware saves host_timeout_s to flash (PROTOCOL.md:595; it is part of Tun, base_logic.h:116/125). The Jetson process then dies (OOM, SIGKILL, Jetson reboot) and systemd restarts it. _keepalive_stop never runs, so 1.5 s stays in flash. The App sends nothing to the base outside a mission: no periodic ping, and DartPoller polls only the dart. The operator then presses Decock, which the abort message asks for ('press Decock on the page before touching the launcher'), or a drum mode change (LAUNCH to NEUTRAL is 41 deg / 8.1 deg/s, about 5 s). The base host watchdog aborts it after 1.5 s with ABORTED HOST and the drum is left BETWEEN. Reproduced with the sim: after set host_timeout_s 1.5, act_mode gave [ABORTED MODE stop, ABORTED HOST 'host silent'] and the drum ended at drum_pos BETWEEN. Every later mission also reads 1.5 as _ka_prev and writes it back, so the state never clears. A drawn spring cannot be decocked from the page in one go.

Evidence: mission.py:524-525 `self._ka_prev = self.base.get("host_timeout_s").get("val")` / `self.base.set("host_timeout_s", self.cfg.host_timeout_s)`; mission.py:545 restores only from `_ka_prev`; base_logic.h:525 `if (c.hostTimeoutS > 0 && running && now - hostT > (uint32_t)(c.hostTimeoutS * 1000)) { cmdStop(false); evAborted("HOST", "host silent"); ...}`; base_logic.h:1092 a `set` marks tunDirty, which is written to flash; app.py:338-343 act_decock and app.py:326-330 act_mode send one command and nothing after it.

Fix: Have the App own the keepalive instead of the Mission. In App.start(), after base.start(), start a permanent daemon thread that calls base.ping(timeout=0.5) every 0.4 s while the link is up. On each (re)connect, set host_timeout_s to cfg.mission.host_timeout_s (1.5). Remove the per-mission set/restore in Mission._keepalive_start/_keepalive_stop. Page actions (decock, mode) are then fed and also covered by the watchdog. If the per-mission design is kept, App.start() must instead write host_timeout_s=0 on every connect, and _keepalive_stop must restore a fixed idle value (0), never the read-back _ka_prev.

> Skeptic (confirmed): I checked the claim against the code and it holds.
(1) mission.py:518-546. _keepalive_start reads host_timeout_s into _ka_prev, sets it to cfg.host_timeout_s (1.5) and starts a ping thread. The only thing that restores the value is _keepalive_stop (called from mission.py:318, in the run wrapper), and it writes back the read-back _ka_prev.
(2) The firmware persists the value. base_logic.h:1092 sets tunDirty after a successful set. base_logic.h:1257 writes Tun to flash once the base is still, and Tun includes hostTimeoutS (lines 116 and 125). PROTOCOL.md:595 says host_timeout_s is saved in flash
>
> Skeptic (confirmed): I could not refute this. I checked each step against the code.

1. Mission._keepalive_start (mission.py:524-525) reads the current host_timeout_s into _ka_prev and then sets it to cfg.host_timeout_s (1.5). Only _keepalive_stop (mission.py:540-547) puts the old value back, and it uses the read-back _ka_prev. No other code under jetson/grappstar writes host_timeout_s. That includes App.start, BaseClient reconnect and HELLO handling.

2. The firmware saves it. handleLine marks tunDirty on any accepted `set` (base_logic.h:1092). When the base is still, tunNow() includes hostTimeoutS and is written
>

## 7. STOP, ESTOP and dart_stop are refused with 403 after the Jetson app restarts, so the open page cannot stop anything until it reloads

- File: `jetson/grappstar/ui/server.py:462`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: The grappstar service crashes and systemd restarts it (grappstar.service: Restart=on-failure, RestartSec=3). The new process makes a new session_token. The operator's open page still sends the old token. Meanwhile the dart can still be running an autonomous anchor or release sequence (up to 35 s, kedge_logic.h:58), or a manual base action can be running (host watchdog off outside a mission). The operator presses STOP: _check_post returns 403 'page session expired' and app.act_stop never runs. app.js only toasts and reloads the page 1.5 s later, so the press is lost. The operator must notice and press again, at least about 2 s plus a page load later.

Evidence: server.py:159 `self.session_token = secrets.token_urlsafe(24)     # per process`; server.py:461-466 any POST with a token other than session_token gets `self._json(403, {"ok": False, "reload": True, ...})`; server.py:475-477 the action is never dispatched; app.js:93-96 `if (r.status === 403 && j.reload) { toast(...); setTimeout(() => location.reload(), 1500); return j; }` (the stop is not re-sent).

Fix: Exempt the stop-only actions from the session-token check, keeping the same-origin check. In do_POST, parse the action name before _check_post, and for name in ('stop','estop','dart_stop','abort') require only self._same_origin() (these actions can only remove energy, so CSRF is not a risk). Also, in app.js action(), when a 403 reload reply comes back for stop or estop, re-send the request once without the token before reloading.

> Skeptic (confirmed): I read the code and the claim holds. In jetson/grappstar/ui/server.py:159 UIServer.__init__ makes a new session_token for each process. do_POST (lines 468-475) calls _check_post before it even matches the action name. Any token other than session_token gets 403 {reload: true} at lines 461-466, and app.action (which would call act_stop, act_estop or act_dart_stop) never runs. No action is exempt, so stop, estop, dart_stop and abort are all refused. On the client, jetson/grappstar/ui/static/app.js:93-96 only shows a toast and reloads after 1.5 s; it never re-sends the stop. The setupStop pointer
>
> Skeptic (confirmed): I checked the code and the defect is real. I didn't run anything.

1. server.py:159 makes a new session_token every time the process starts. server.py:291-298 writes it into index.html only when the page is served.
2. app.js:34 reads TOKEN once, when the page loads. action() (app.js:85-100) sends it on every POST, including stop and estop.
3. do_POST (server.py:469-475) calls _check_post before it parses the action name. A wrong token gets 403 {reload:true} at server.py:461-466 and the connection closes. app.action() is never called. There is no exemption for stop, estop or dart_stop.
4. app.j
>

## 8. Bench build connects MP1584 #2 to DRV8833 VM without re-measuring the buck; the first output check is with the DRV8833 already on it

- File: `firmware/base/WIRING.md:216`
- Severity: low (claimed low); unverified (low)
- Owner: docs

Scenario: MP1584 #2 is set in Part A, then handled through the rest of Part B. It is the spare buck from the dart, previously set to other voltages, and has an open trimmer. Part B step 11 solders OUT+ to DRV8833 VM without a fresh unloaded reading. The first measurement is Part D step 23, taken with the battery in and the DRV8833 already connected. If the trimmer was turned, or Part A was skipped because the buck was 'already marked', VM can reach up to the 12.6 V input. That is above the DRV8833 absolute maximum of 11.8 V and its 10.8 V rating, and DRV8833 #3 is destroyed at power-on. SYSTEM_ELECTRICAL's rover path does this correctly (P5 taped until the buck reads 9.0 V), but the bench path in 07_electronics follows WIRING.md first.

Evidence: WIRING.md:216 `11. Battery unplugged. Buck output (already set to 9.0 V in Part A): OUT+ to DRV8833 VM, OUT- to DRV8833 GND.`
WIRING.md:239 `Measure the buck output again (9.0 V) ...` (after the battery is in with VM connected)

Fix: Change Part B step 11 to solder the VM wire to the DRV8833 only, leaving its free end taped. In Part D step 23, first measure buck OUT unloaded and require 8.8 to 9.2 V, re-setting it if not. Then unplug the battery, solder the VM wire to OUT+, and plug in again. This matches SYSTEM_ELECTRICAL steps 5 and 6.


## Refuted (dropped)

- `firmware/base/grappstar_base/base_logic.h:683` Base accepts cock/aim/reel at a flat resting battery; the resulting BROWNOUT leaves the spring part-drawn and both clear and decock are refused
