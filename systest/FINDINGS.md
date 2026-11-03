# System test findings

Found by the end-to-end harness (`systest/run_mission.sh`: the real base logic over a pty, the real dart firmware
sim over HTTP, the real Jetson App). Each entry: the exact failing scenario, the file and line, the owner.
Line numbers are against `origin/base-v1` at the time of the entry. Status: OPEN until the owner says fixed and the
harness re-runs green.

## Status (14:40 Sydney)

"fixed on branch" = green in the preview runs (`PREVIEW=1 systest/batch.sh`, base-v1 + w/fw-base + w/fw-dart +
w/jetson; commits in `systest/runs_preview.version` and the logs), or stated fixed by the owner with the commit and
re-tested where noted. Evidence in `systest/runs_preview/<scenario>/` and `systest/runs_fw87/`.

| # | owner | status | evidence |
|---|---|---|---|
| F1 | fw-base | open (not re-tested: the harness waits for BOOT) | |
| F2 | fw-base, jetson | FIXED in base-v1 | `tidy` -> TIDIED in every run |
| F3 | fw-base | partly (Jetson reads l r I) | PROTOCOL re-sync |
| F4 | jetson | fixed on branch | WRAP before ANCHOR in every mission run |
| F5 | jetson, docs | fixed on branch | `nominal_uncal` PASS: TOW refused, dart brought home |
| F6 | jetson | fixed on branch | prompt says "The drum HELD where it is (...)" |
| F7 | jetson | fixed on branch | `stall_cock`: Decock and re-aim, mission DONE |
| F8 | jetson | fixed on branch | `power_unwrap(_fasthello)`: BASE_RESET at once, DONE in about 120 s |
| F9 | fw-base | fw-base reports an index mode (`indexMode` in cmdAim); not re-tested: the plant has no index cam | |
| F10 | jetson | fixed on branch | `no_angle_sensor` PASS: preflight "AS5600 sees no magnet" |
| F11 | jetson | not reproduced since F4 (no partial wrap any more) | |
| F12 | jetson | fixed on branch | C1 order = 10.1 apart from F25 |
| F13 | fw-base | fixed on branch (fw-base) | retrieve at the park WRAPS_ON |
| F14 | fw-base, docs | fixed on branch | CALIBRATION.md two-point fit with offset (D42 default -3.0) |
| F15 | fw-base | fixed on branch | `ref_power_wrap`, `power_aim`, `ref_power_aim` PASS |
| F16 | fw-base | fixed on branch (fw-base: tension_offset_n saved; host_timeout_s deliberately not) | |
| F17 | fw-base | fixed on branch (fw-base 87c48b3) | |
| F18 | jetson | fixed on branch | no 60 s TOW waits |
| F19 | fw-base | fixed on branch | `ref_refusals`: every wrong command refused (fw-base 6381edd) |
| F20 | fw-base (D41) | fixed on branch | `ref_nominal` fly-in 14.6 N, no fault (c28396c) |
| F21 | fw-base | fixed on branch | `ref_tow_stiff` PASS (6381edd): tow limit held on the stiff line |
| F22 | fw-base / jetson | by design: host sends `stop` then `reel`; PROTOCOL + mission to align | |
| F23 | fw-base | fixed in firmware 783f3b2 (line_m - 0.88 m after the wrap): the Jetson must not subtract it too | re-test |
| F24 | jetson (low) | open | |
| F25 | jetson / spec (low) | open (stop-first is safer: suggest the spec adopts it) | |
| F26 | fw-base | fixed in 4231f46 (owner) | re-test `hand_lever` |
| F27 | fw-base (low) | fixed in 4231f46 (owner: clamp within aim_tol) | re-test |

## F1 (fw-base) `cal` (and `set`) during the first 60 ms are acknowledged ok, then silently undone by the boot rebuild

- Scenario: fresh base, the host sends `{"id":0,"cmd":"cal","what":"cock_rest"}` 10 ms after power-on.
  Reply `{"id":0,"ok":true}`. At 60 ms `BOOT` reports `"phase":"UNKNOWN"`, and `status` keeps saying `UNKNOWN`: the
  cock_rest was thrown away. The Jetson preflight then refuses ("launcher shot phase UNKNOWN").
  Log: `systest/runs/nominal/base.log` (first run, 12:15 Sydney).
- Where: `firmware/base/grappstar_base/base_logic.h:803-807` (`cal cock_rest` sets `phase = PH_REST` with no boot
  check) and `resolveBoot()` at `base_logic.h:268` which overwrites `phase` from the saved record on the first tick
  after 60 ms. `cal aim_home`, `cal aim`, `set line_m` have the same race.
- Real-world trigger: the Jetson opens the port as soon as the board enumerates; a script or the operator page
  sending setup lines right after a reset hits it. Also the ESP32 sends HELLO before BOOT, so "send after HELLO" is
  not enough.
- Fix suggestion: refuse `cal` (all but `drum`) and `set` with `BOOTING` while `bootPending`, like the actions
  (PROTOCOL.md section 2 lists `BOOTING` as a common refusal for actions only).
- Harness workaround: waits for `BOOT` before bench setup.

## F2 (fw-base + jetson) [FIXED in base-v1 d15129e: firmware has `tidy` / TIDIED, harness sees TIDY run] TIDY never runs: the Jetson sends `tidy`, the firmware only knows `reel until:"slip"`

- Scenario `nominal` (every run): mission step TIDY logs "base has no tidy command: skipped". The base replied
  `UNKNOWN_CMD` to `{"cmd":"tidy"}`. The firmware has a tidy, but as `reel` with `until:"slip"`
  (`firmware/base/grappstar_base/base_logic.h:718` parse, `:1179-1183` the slip detector, LIMIT event with
  `why:"tidy done (take-up slipping)"`), and `PROTOCOL.md` section 2 does not document `slip` at all.
  Jetson: `jetson/grappstar/mission.py:488-500` (`tidy()` sends `tidy`, waits for `TIDY`/`TIDIED`/`LIMIT`).
- Spec: BASE_SPEC section 10 and D22 make TIDY a required step (the FIRM take-up pulls the dart-side slack through the
  capstan before REEL fly-in). Skipping it leaves the landing slack on the dart side.
- Fix: one of the two owners adopts the other's form and PROTOCOL.md documents it (suggest: the Jetson sends
  `{"cmd":"reel","until":"slip"}` when `tidy` is UNKNOWN_CMD, and waits for LIMIT with `why` starting "tidy").

## F3 (fw-base) PROTOCOL.md no longer matches the firmware on base-v1

- `wrap`: PROTOCOL.md:54 says "drum on from REEL to the WRAPPED stop point", refusal `NOT_IN_REEL`; the firmware
  (`base_logic.h:687-697`) wraps from NEUTRAL or later and refuses `NOT_IN_NEUTRAL`; a wrap with the collar already
  in REEL answers WRAPPED at once (`:690`).
- `reel until`: firmware accepts `slip` (`base_logic.h:718`), PROTOCOL.md:56 lists only stop / seated / tension.
- Telemetry `sw` keys: firmware `config.h:17` sends `l` and `r` (SW_DOG_L / SW_DOG_R) and `I` (SW_INDEX); PROTOCOL.md
  section 3 lists `D` (SW_DOG). D27 says one SW_DOG. The Jetson (`mission.py` `sw_closed`) reads `D`.
- Seen in the harness: telemetry `{"sw":{"a":1,"l":1,"S":1,"H":1,"W":1}}` at LAUNCH.
- The Jetson is written against PROTOCOL.md, so every drift is a latent integration bug. Please re-sync PROTOCOL.md
  (or state the version the firmware implements) when the D22 / D27 rework lands.

## F4 (jetson) mission order is still pre-D22: ANCHOR before WRAP

- Scenario `nominal`: steps `... LANDING_ZONE, BASE_NEUTRAL, ANCHOR, TIDY, WRAP, CHECK, TOW ...`
  (`jetson/grappstar/mission.py:325-347` `_sequence`). D22 (spec v1.1): LANDED -> NEUTRAL -> WRAP -> WRAPPED ->
  anchor -> TIDY -> REEL fly-in -> TOW (laying turns against a taut dart side needs about 360 N at the ring).
- CONFIRMED with the D22 plant (13:40 run, base-v1 merged at d15129e). The harness plant now makes every laid turn
  draw one capstan circumference (d 40 + line: 0.129 m) from the dart side; an anchored dart side gives only its slack.
  Scenario `nominal`: dart lands with 0.9 m slack, ANCHOR, TIDY (3 s) pulls 0.19 m, WRAP lays 5.5 of 7 turns using the
  remaining 0.71 m, then the cup cannot turn: `FAULT DRUM_TIMEOUT "drum not turning (no code edge)"`, drum HELD
  at 250 deg (BETWEEN, wraps 5.48). This is the D22 jam, now reproduced end to end. Fix = the D22 order in `_sequence`.

## F5 (jetson + docs) the tow limit is only as good as CALIBRATION.md, and a tow fault has no way out

- Scenario `nominal_uncal` (factory `k_n_per_a` 8, `free_a` from params, i.e. CALIBRATION.md skipped): during TOW the
  plant line tension reached 61 N with `tow_limit_n` 40 N, then the base tripped OVERCURRENT (trip_a 5 A) holding
  the line (correct: wraps on, drum HELD). The Jetson offered Retry / Abort only; each Retry cleared the fault and
  restarted `reel until tension` into the same 61 N line: OVERCURRENT again within 50 ms, three times, then Abort
  with the dart anchored and the line at 61 N (log `systest/runs/nominal_uncal/`).
  Files: `jetson/grappstar/mission.py:300-322` (`_do`: a BaseError in TOW gives Retry / Abort; no "Release the dart"
  choice), `mission.py:808-830` (`tow`).
- Asks: (jetson) in TOW / CHECK, a base fault should offer "Release the dart" (stop, dart /release, then retrieve), and
  Retry should not restart a pull that is already above the limit; (jetson) preflight could warn when `get k_n_per_a`
  is still the factory 8.0 and there is no load cell; (fw-base / docs) BENCH_TEST or the first-mission checklist
  should make CALIBRATION.md of `free_a` and `k_n_per_a` a hard prerequisite of the first tow.
- With the calibrated bench (scenario `nominal`) the tow holds at 40.0 N exactly (plant truth 40.03 N).

## F6 (jetson) fault prompt says "went to NEUTRAL" when the base said DRUM_HELD

- Scenario `drum_timeout` (drum N20 jammed during `mode NEUTRAL` after the shot): the base raised DRUM_TIMEOUT and
  sent `{"evt":"MODE","ok":false,"mode":"LAUNCH","drum_pos":"LAUNCH","err":"DRUM_HELD","why":"drum fault"}`
  (correct per PROTOCOL.md section 6 / D27: a drum fault holds the drum). The operator prompt read "BASE_NEUTRAL
  failed: base refused wait: DRUM_TIMEOUT. The base is in FAULT and went to NEUTRAL."
- Where: `jetson/grappstar/mission.py:327` appends the NEUTRAL sentence for every fault code.
- Why it matters: the operator is told the collar is out of LAUNCH when it is still in LAUNCH (and in the cocked or
  wrapped cases the drum is deliberately held). Suggest: read the last MODE event; with `err:"DRUM_HELD"` say "the
  drum is HELD where it is (<why>)", and only say NEUTRAL when telemetry shows `drum_pos` NEUTRAL.
- Also: Retry right after DRUM_TIMEOUT re-sends the same `mode` at once (3 retries in 9 s in the log); the prompt
  could say what to check (N20 wires, drum free to turn, SW_N20 lever) before Retry.

## F7 (jetson) a fault while cocking traps the operator: Retry loops on LOADED_UNKNOWN, Decock is refused mid-mission

- Scenario `stall_cock`: extra 20 N m on the wiper 0.6 s into `cock`. Base: `FAULT OVERCURRENT` ("wiper over 6 A"),
  `MODE ok:false err:DRUM_HELD` (launch train not at rest: correct, D27). Shot phase becomes UNKNOWN with the carriage
  drawn (PROTOCOL.md: a stop during cock makes the phase UNKNOWN).
  Jetson: prompt "COCK failed ... went to NEUTRAL" (see F6), choices Retry / Skip / Abort. Retry: `clear` then `cock`
  -> `LOADED_UNKNOWN`, three times, the only way out is Abort (or Skip, which goes on to ARM with a half-drawn spring;
  arm then refuses NOT_COCKED, but Skip should not be offered here).
  The operator cannot press Decock while the mission runs: `jetson/grappstar/app.py:330` refuses with "mission running".
- Where: `jetson/grappstar/mission.py:300-332` (`_do`), `SKIPPABLE` contains COCK (`mission.py:48`), `app.py:328-333`.
- Suggest: when the base answers `LOADED_UNKNOWN` (or the phase is UNKNOWN with `SW_HOME` open), offer "Decock and
  re-aim" in the prompt (cal decock, wait RESET, then AIM again since decock turns the aim train), and do not offer
  Skip for COCK in that state.
- Harness: the scenario now models what the operator must do today (Abort, Clear, Decock, Start again) and checks
  that the second mission completes.

## F8 (jetson) a base reset during a long action is noticed only at the action timeout (60 s) when HELLO is missed

- Scenario `power_unwrap` (first batch, HELLO sent at power on): base killed 3 s into UNWRAP, restarted after 2 s.
  The Jetson's link dropped (`base link down: SerialException`) and came back, but HELLO and BOOT had been written
  before pyserial reopened the port, and `serial.Serial()` flushes the input buffer on open, so they were lost.
  `wait_event("UNWRAPPED")` then waited its whole `wrap_timeout_s` (60 s): prompt "UNWRAP failed: no UNWRAPPED
  event from the base in 60 s". Retry then recovered and the mission finished (DONE), 60 s late.
- Where: `jetson/grappstar/comms/base.py:505-538` (`wait_event` fails fast only on a HELLO event) and `:259-284`
  (`_run` knows the link went down and counts `reconnects`, but nothing waiting is told).
- Real hardware: the ESP32-S3 sends HELLO about 2.1 s after reset (setup waits for the host up to 1.5 s, the HX711
  probe up to 0.6 s), and the USB port re-enumerates in that time, so whether the Jetson reopens before HELLO is a
  race (reconnect_s, udev). Do not rely on HELLO.
- Suggest: treat a link drop as a session break: in `_run`'s `finally`, append a synthetic `{"evt":"HELLO","_lost":
  true}` (or bump a session counter that `wait_event` / `wait_tel` / `cmd` check) so every pending wait raises
  `BASE_RESET` at once; optionally confirm with `ping` (`t` smaller than before = reset).
- Same in `usb_unplug_tow` (USB out 3 s while the base held the tow at 40 N): "TOW failed: no ('LIMIT', 'SEATED')
  event from the base in 60 s", then Retry towed again and the mission finished. The line held at 40 N throughout
  (sprag #2 + worm), so it is safe, but the operator waits a minute with no explanation.
- The harness now models the sketch's 2.1 s setup delay by default (`power_unwrap`) and keeps the immediate-HELLO
  race as `power_unwrap_fasthello`.

## F9 (fw-base) D12 degraded mode (SW_INDEX index cam, no AS5600 magnet) is not implemented

- Scenario `no_angle_sensor` (AS5600 reads fail from power on): every wiper action is refused `ANGLE_SENSOR`
  (`aim` in the log; `status` says `magnet` `NONE`). The mission cannot get past AIM.
- D12 (`base/spec/DECISIONS.md:20`): "Fallback with no magnet: a 6-bump INDEX CAM on the main shaft read by SW_INDEX;
  firmware degraded mode counts turns and sixths with time interpolation and re-zeroes on SW_HOME, SW_RAIL_LOW ...".
  Firmware: `config.h:37` maps SW_INDEX to bit -1 (not read), and `base_logic.h` has no use of SW_INDEX at all
  (grep). PROTOCOL.md does not mention a degraded mode either.
- Either implement it (and the sim.h plant needs an index-cam contact so the host tests and this harness can run it),
  or record in the spec that D12's fallback is dropped and a magnet is mandatory (then MEASURE_LIST / BENCH_TEST:
  "fit the magnet before the first power-up").

## F10 (jetson) preflight passes with the angle sensor dead; refusals are reported as faults

- Same scenario: PREFLIGHT reports only the video warning and passes, although the base said `"as5600":false` in
  HELLO and `"magnet":"NONE"` in the `status` reply that preflight itself fetched (`jetson/grappstar/mission.py`
  `preflight()`, it never reads `magnet`). The operator finds out at AIM, after the target and the confirmations.
- The AIM refusal (`{"ok":false,"err":"ANGLE_SENSOR"}`, a refusal: nothing moved, no fault latched) is shown as
  "AIM failed: base refused aim: ANGLE_SENSOR. The base is in FAULT and went to NEUTRAL." (`mission.py:323-327`
  treats any err string that equals a fault code as a fault). Retry then repeats it 3 times without any new information.
- Suggest: preflight problem "angle sensor not reading (magnet NONE): fit the magnet / check the AS5600 wires" when
  `magnet` is `NONE` (warning for `WEAK` / `STRONG`); only add the FAULT sentence when telemetry `fault` is set.

## F11 (jetson) `is_wrapped` still uses the 4-wrap threshold: a half-laid 7-turn wrap counts as wrapped, Retry skips the wrap

- Scenario `nominal` after the F4 jam: the wrap stopped at 5.48 turns (telemetry `wraps` 5.48, `drum_pos` BETWEEN).
  The operator chose Retry: `wrap()` returned at once because `is_wrapped()` is true for `wraps >= 3.5`, the mission
  went on to CHECK, and `reel` was refused `NOT_WRAPPED` three times (Retry / Skip / Abort loop) until Abort.
- Where: `jetson/grappstar/mission.py:565-566` (`wraps >= 3.5`, from the 4-wrap firmware). The firmware now lays 7
  (`config.h:131 wrapsFull`, D16).
- Fix: wrapped only when `drum_pos == "WRAPPED"` (the drum code is absolute), or `wraps >= wrapsFull` read from the
  base; never from a partial estimate. Same test is used by `ensure_reel()` and `unwrap()`.

## F12 (jetson) the mission's C1 command order differs from BASE_SPEC 10.1 (D35) in four places

Checked by `systest/c1_order.py` (commands taken from the Jetson's own `mission.jsonl`, base_tx and dart_tx lines,
retries collapsed) against the 10.1 order. Batch 1 (base-v1 before d15129e), scenario `dog_tip` (completed DONE):

`aim > cock > arm > fire > set line_m > mode NEUTRAL > dart auto > tidy > wrap > reel tension > dart release > reel seated > unwrap > mode LAUNCH`

10.1 wants: `... set line_m > (mode NEUTRAL) > wrap > dart auto > tidy > mode REEL > reel tension ... > dart release > (reel stop) > unwrap > retrieve > mode LAUNCH`

1. `dart auto` before `wrap` (F4, D22).
2. No `mode REEL` fly-in after the tidy: the Jetson relies on `wrap` leaving the collar in REEL (old firmware).
   With the D35 park at WRAPPED 314 (collar NEUTRAL), `reel` would be refused or pull nothing.
3. `reel until seated` on the capstan (`mission.py` `retrieve()` falls to `until="seated"` whenever `line_m` is at or
   under approach_m + 0.3, which is the normal case after three tow rounds). 10.1 step 17: "Never seat on the capstan".
4. Final approach: when `retrieve` was UNKNOWN_CMD the fallback wrapped again and seated on the capstan
   (`power_unwrap`, `usb_unplug_tow`: `... unwrap > retrieve > wrap > reel seated > unwrap`). The firmware now has
   `retrieve` (base_logic.h:986), so drop the fallback.
The check is in every scenario (`C1 command order = BASE_SPEC 10.1 (D35)`); it goes green when the order matches.

## F13 (fw-base) `reel until tension` at the WRAPPED park is refused `NOT_WRAPPED` although the wraps are on

- Scenario `nominal_uncal`, batch 2 (base-v1 d15129e): `wrap` -> `WRAPPED {"wraps":7}` (park 314, collar NEUTRAL),
  then the Jetson's check pull `{"cmd":"reel","duty":0.48,"max_n":15.0,"until":"tension"}` -> `NOT_WRAPPED`, four times.
  The real cause is F12 item 2 (the Jetson does not send `mode REEL` first), but the code sends the operator the
  wrong way: telemetry says `drum_pos` WRAPPED and the prompt says "NOT_WRAPPED".
- Where: `firmware/base/grappstar_base/base_logic.h:791-792` (`!collarReel() && !neutral` and `neutral && un ==
  U_TENSION` both return `NOT_WRAPPED`).
- Suggest: `NOT_IN_REEL` (send `mode REEL`) when the wraps are on but the collar is in NEUTRAL; keep `NOT_WRAPPED`
  for no wraps. Document it in PROTOCOL.md.

## F14 (fw-base, CALIBRATION.md) following CALIBRATION.md leaves the -10.4 N `tension_offset_n`: the tow overshoots by 10 N

- Scenario `ref_nominal` (BASE_SPEC 10.1 driven directly, base-v1 d15129e), bench calibrated exactly as
  CALIBRATION.md (e) says (`free_a`, `k_n_per_a` matched to the motor): tow `reel until tension max_n 40` stopped with
  `LIMIT {"tension_n":40}` three rounds running while the plant line tension was 50.4 N (+26 %). Same at duty 0.8
  (`ref_tow_duty08`), so it is not duty related.
- Cause: the estimate is `(amps - free_a) * k_n_per_a + tension_offset_n` (`base_logic.h:368`), default offset
  -10.4 N (`config.h:173`, physics v1.2). CALIBRATION.md (e) computes `k_n_per_a = 20 / (amps - free_a)`, which is
  only right with a zero offset, and never mentions `tension_offset_n`. PROTOCOL.md's `set` table does not list it
  either. Whoever follows the doc tows 10.4 N above the limit (60 N at the default 50 N limit).
- Also in CALIBRATION.md (e) step 2: after `wrap` the drum now parks at WRAPPED with the collar in NEUTRAL (D35), so
  the step 3 jog only turns the take-up; a `mode REEL` is needed before the capstan pulls.
- Fix: either compute both from two scale readings (for example 10 N and 30 N: `k = 20 / (a30 - a10)`, `offset = 10 -
  k (a10 - free_a)`) and set both, or set `tension_offset_n` 0 in the procedure; add `mode REEL` after the wrap;
  list `tension_offset_n` in PROTOCOL.md section 5.
- Harness: the calibrated bench now sets `tension_offset_n` 0 as a correct two-point calibration would;
  `ref_doc_cal` keeps the doc-literal calibration and shows the overshoot.

## F15 (fw-base) power loss during the drum part of `wrap` (after the shot) leaves the phase UNKNOWN; `wrap` and `decock` are then both refused

- Scenario `ref_power_wrap` (BASE_SPEC 10.1, base-v1 d15129e): after FIRED and RESET, `wrap` from LAUNCH (D22: wrap
  straight from LAUNCH); the base is killed 1 s into the drum move (drum in L_TO_N, the wiper had finished its 3 deg
  unload jog and was stopped), restarted 2 s later.
  `BOOT {"record":true,"was_moving":true,"phase":"UNKNOWN","drum_known":true}`. Carriage home (`sw.H`).
  Then: `wrap` -> `LAUNCH_BUSY` (every retry), `cal decock` / `decock` -> `NOT_IN_LAUNCH` (drum BETWEEN).
- Where: `base_logic.h` `cmdWrap` -> `launchExitCheck()` (`:641-645`, `maybeLoaded()` true for UNKNOWN) while
  `cmdCal("decock")` needs `detentClosed() == DP_LAUNCH` (`:880`). The moving record is written with the wiper bit at
  the unload jog and only cleared when everything stops (`markMoving` / `persist`), so a cut during the drum-only part
  still says the wiper was moving, and resolveBoot makes the phase UNKNOWN although the AS5600 angle is valid and
  SW_HOME is closed.
- The only escape is `mode LAUNCH`, `cock` (creeps to SW_HOME open, re-zeroes and DRAWS THE SPRING again while the dart
  is out on its line), then `decock`, then `aim` again; or the bench-only `cal cock_rest`. The harness now tries that
  path and records whether it works.
- Suggest: clear the wiper moving bit when the wiper stops (persist with `movingBits & ~MV_WIPER` once `w` is idle),
  so a drum-only cut keeps the shot phase; and/or after a shot with SW_HOME and SW_GAP closed and a valid AS5600 angle,
  rebuild the phase as REST (the carriage cannot be drawn with SW_HOME closed unless SW_HOME lies, which
  `SWITCH_DISAGREE` covers). Also let `decock` run with the drum out of LAUNCH when the carriage is home (it would
  only need to report RESET).

## F16 (fw-base) `tension_offset_n` and `host_timeout_s` are not saved: lost at every power cycle

- Scenario `ref_power_wrap`: bench set `k_n_per_a`, `free_a` and `tension_offset_n 0` (two-point calibration), then a
  power loss and restart. After the restart the tow stopped at `LIMIT 40` with the plant line at 50.4 N: the offset was
  back at the -10.4 N default while `k_n_per_a` / `free_a` survived.
- Where: `firmware/base/grappstar_base/persist.h:53-58` (`struct Tun` has no field for `tensOffsetN` or
  `hostTimeoutS`), `base_logic.h:110-124` (`tunNow` / `restoreTun`). The `set` table (`base_logic.h` `keys()`) lists
  both, and PROTOCOL.md section 5 says "all saved in flash, kept over a power cycle".
- Consequence: every power cycle silently changes the tow limit by 10 N (F14) and switches the host watchdog off.
- Fix: add both to `Tun` (bump `ver` to 2 and accept a v1 record with defaults), add a host test that sets every key,
  power cycles and reads them back.

- F15 also for a cut during AIM (scenario `power_aim`, batch 2): aim runs the wiper BACKWARD with the carriage home
  (sprag #1 freewheels, the cock train cannot move), yet after the restart `BOOT "phase":"UNKNOWN"` and every `aim`
  is refused `PHASE_UNKNOWN`. PROTOCOL.md section 7 lists only "cocking, firing or resetting" as making the phase
  UNKNOWN. A backward run should keep the shot phase (REST), and only forget the aim.

## F17 (fw-base) `decock` with the carriage home and the phase UNKNOWN answers ok but does nothing and sends no RESET

- Scenario `power_aim` follow-up: phase UNKNOWN, `sw.H` closed. PROTOCOL.md section 5: "`RESET` event with
  `how:"decock"` (immediately if the carriage is already home)". Firmware `base_logic.h:886`:
  `if (phase == PH_UNKNOWN && !(phaseHint ...)) return nullptr;` replies `ok:true`, starts nothing, emits no event
  and leaves the phase UNKNOWN. A host that waits for RESET (the Jetson's Decock button, the D35 recovery "decock,
  wait RESET, then aim") hangs until its own timeout, and `aim` is still refused PHASE_UNKNOWN afterwards.
- Suggest: with the carriage home and no evidence of a drawn spring, `decock` sets the phase REST and sends
  `RESET {"how":"decock"}` (or refuses with a code that says "use cock to re-zero"), and PROTOCOL.md says which.

## F18 (jetson, on w/jetson 16a4c6c, not yet in base-v1) TOW waits 60 s per round: a stale telemetry `state` makes it think the base winds by itself

- Preview run (`systest/preview.sh`: base-v1 ba121b9 + w/fw-base 7b7c367 + w/fw-dart e47da79 + w/jetson 16a4c6c),
  scenario `nominal`, no load cell. Round 1: `reel ... until tension` -> `LIMIT 40 holding` (plant 40.2 N, good). The
  rover hook returns "slack". Round 2: the mission logs "base is already winding (slack taken up by itself)", sends
  nothing, and `wait_evt(LIMIT)` times out after 60 s ("TOW failed: no ('LIMIT', 'SEATED') event from the base in 60 s").
  Retry sends `reel` again and that round works; the same happens every round. The mission still ends DONE, 2 min late.
- Cause: `jetson/grappstar/mission.py:1013` (w/jetson) checks `self.tel().get("state") == "REEL"` right after the
  LIMIT event; the latest telemetry line was written just before the LIMIT and still says REEL (the HOLD telemetry
  arrives up to 50 ms later). Also, without a load cell (`tension_n` null) the firmware never winds again by itself
  (PROTOCOL.md: only "with a load cell"), so that branch can only be right when `tension_n` is a number.
- Fix: take the "already winding" branch only with a load cell and only on telemetry newer than the LIMIT event
  (`self.base.tel_t > e["_t"]`), or simply send `reel` and treat a `BUSY` reply as "already winding" (the code already
  handles BUSY).

## F19 (fw-base, new spec 1.2.7 / D39, expected to be in progress) `retrieve` at the WRAPPED park and `reel max_n 80` are accepted

- Scenario `ref_refusals` (preview: base-v1 ba121b9 + w/fw-base 6472524): wrong commands thrown into the 10.1
  sequence. Refused as they should be: `aim 85` OUT_OF_RANGE, `fire` without arm NOT_ARMED, `wrap` while cocked
  LAUNCH_BUSY, `reel until seated` with the wraps on WRAPS_ON. Accepted (D39 says refuse):
  - `retrieve {"duty":0.3}` at the WRAPPED park (drum 314, wraps on) -> `ok:true` (D39: retrieve only in NEUTRAL with
    the wraps off);
  - `reel {"duty":1,"max_n":80,"until":"tension"}` in REEL -> `ok:true` (D39: tow cap 70 N; the harness stopped it at
    once).
- The check `every wrong command refused` goes green when D39 is in the firmware.

## F20 (fw-base + spec, D35 order) with a stiff (real Dyneema) line the REEL fly-in after TIDY trips OVERCURRENT every time

- Found after R2-L review item 1: the plant line is now 1 mm Dyneema (EA 20 kN: about 5.6 kN/m at 3.6 m) instead
  of sim.h's 40 N/m spring. Scenario `ref_nominal` (BASE_SPEC 10.1 exactly, preview fw-base 6472524): the wrap takes
  the 0.9 m landing slack, the dart anchors, `tidy` -> TIDIED (dart side taut), `mode REEL` (fly-in) ->
  `FAULT OVERCURRENT "wiper over 3.0 A"` and `MODE ok:false DRUM_HELD` within 3 s (plant peak line tension during
  the fly-in 37.4 N, peak wiper 4.2 A); four retries, the same.
- Why: the fly-in creeps the wiper FORWARD until the collar drops into REEL (config.h:147, `jogTripA` 3.0 A at
  config.h:65). Once the dogs bite, every further wiper degree winds the capstan (1.667 capstan turns per wiper turn,
  r 20.5 mm: about 0.6 mm of line per wiper degree). On a taut Dyneema line 10 deg of creep is about 6 mm, about 33 N:
  far over the 3 A trip (about 14 N of line with the calibrated constants). With sim.h's soft spring the same 6 mm is
  0.24 N, which is why every earlier run passed.
- The order is binding (BASE_SPEC 10.1 steps 11-12): TIDY leaves the dart side taut at FIRM 5 N by design, so the
  fly-in always starts against a taut line in the field.
- Options for fw-base / coordinator: stop the fly-in creep the moment SW_DOG_R (or the collar position) says engaged
  and do not count wind-in current as a fault during the fly-in (treat it as the tow start: `LIMIT` at the tow cap);
  or fly in BEFORE the tidy and tidy through the engaged capstan (the take-up still runs in REEL); or leave a
  defined slack after the tidy (tidy by length). The harness check stays: fly-in on a taut 5.6 kN/m line.
- Note: the stiffness assumption (EA 20 kN) is mine from the line spec (667 N break, about 3.5 % at break); real
  braid has some bedding-in stretch, which makes it softer at first. Even at EA 5 kN (1.4 kN/m) 6 mm is about 8 N.

## F21 (fw-base) on a stiff line the tow overshoots the limit by about 25 % at duty 1

- Preview (fw-base c28396c, D41 in), scenario `ref_nominal` (BASE_SPEC 10.1, 1 mm Dyneema EA 20 kN, anchored at
  3.6 m: about 5.6 kN/m): after the fly-in (14.6 N), `reel duty 1 max_n 40 until tension` -> `LIMIT` after 245 ms with
  `tension_n` 48.9 (the firmware saw it was over), plant line 50.3 N held by the worm. Same pull in `ref_tow_duty08`
  at duty 0.8 for comparison in the batch log.
- Why: once taut the line tension rises at capstan speed x stiffness, several hundred N/s; the taper "over the last
  30 % of max_n" (PROTOCOL.md `reel`) never gets a sample in that window, and the 60 ms raw-estimate hold adds
  ~10 N. The worm then holds the overshoot (no back-off).
- Suggest: when a tow starts from HOLD or right after the fly-in (line known taut), start at low duty and raise it
  only while the estimate stays under about 50 % of max_n; or predict from the tension rate (stop when
  `tension + rate x 60 ms >= max_n`). Acceptance: `ref_nominal` / `ref_tow_stiff` plant peak under max_n + 15 %.

## F22 (fw-base: BY DESIGN, host must `stop` then `reel`; jetson / PROTOCOL to align) without a load cell, a held tow never pulls again: `reel` from HOLD answers "already at the limit" on a slack line

- Same run: after round 1 the rover drives 0.6 m toward the dart (plant line goes slack: 0.5 N). Round 2 and 3:
  `reel ... until tension` -> `LIMIT {"tension_n":48.9,"why":"already at the limit","holding":true}` within 5 ms,
  nothing turns. The held estimate (48.9 N from round 1) is kept although the line went slack, so the tow can never
  take up the slack the rover gives. PROTOCOL.md `reel`: "also accepted while holding (state HOLD): winds again".
- Suggest: without a load cell, a `reel` from HOLD (or after `stop`) re-measures: start winding at low duty and take
  the estimate from the new current after the start blanking, not the held value; refuse only with a load cell that
  shows the line still at the limit.

## F23 (fw-base: FIXED on w/fw-base 783f3b2, line_m drops 0.88 m after the wrap) `line_m` ignores the 0.9 m the wrap draws from the dart side: the reel-in leaves 0.6 m, not 1.5 m

- Same run: `set line_m 4.48` after the landing; the wrap (7 turns on d 40 + line) draws 0.9 m from the dart side
  (D22: the landing slack), plant line out 3.58 m after the wrap. The capstan reel-in stopped when telemetry
  `line_m` reached 1.5 (10.1 step 15): plant line out was 0.58 m. The dart then seated during the unwrap.
- Fix: the firmware subtracts `wrapsFull x pi x (d + line d)` from `line_m` when it lays the wraps (it knows the
  count), or the Jetson re-sends `set line_m` after WRAPPED. Without it, "stop at about 1.5 m" means 0.6 m.

## F24 (jetson, low) host watchdog margin: pings every 0.8 s against `host_timeout_s` 1.5

- Preview `nominal` (base at 2x sim speed, 3 parallel shards on 4 cores): the Jetson sets `host_timeout_s 1.5` and
  pings about every 0.8 s; one gap of 1.5 s of base time (0.75 s of wall time at 2x) during AIM made the base stop
  (`ABORTED {"what":"HOST","why":"host silent"}`). Retry recovered. At real speed the same needs a 1.5 s gap, so this
  is mostly my speed scaling, but the margin is only 0.7 s against a Python process that also runs video, vision and
  the landing analysis (GC or a busy GIL can take that).
- Suggest: ping from a dedicated thread at 0.3 s, or set the watchdog to 3 s; log the largest ping gap.
- Harness: the final batch runs the base at real time (`SYSTEST_SPEED=1`).

## F25 (jetson + spec, low) step 14 order: the mission stops the base before `/release`; 10.1 says release, then stop

- Preview `dog_tip`, `drum_timeout`, `nominal`: `... reel tension > stop > dart release > reel stop ...`
  (`mission.py` `release_dart`: `self.base.safe_stop()` then `self.dart.release()`). BASE_SPEC 10.1 step 14:
  "dart `/release`, then `stop`".
- Stop first is the safer order (the capstan never pulls against a releasing dart; the worm holds the line either way),
  so I suggest the spec adopts it. The C1 check accepts both orders meanwhile.

## F26 (fw-base) a drum turned by hand while the board is off gives `DRUM_SENSOR` at boot

- Preview (fw-base c28396c), scenario `power_launch_mode` set-up: board off, drum turned from LAUNCH to NEUTRAL
  (41 deg), board on. `BOOT {"record":true,"was_moving":false,"phase":"UNKNOWN"}` then
  `FAULT DRUM_SENSOR "drum code disagrees with the saved drum angle"`. The spec makes the drum lever the manual
  fallback (BASE_SPEC 1: "Manual lever on the drum shaft end as the hand fallback") and PROTOCOL.md section 7 says the
  sector always comes from the absolute drum code at boot. A clear is needed before anything moves. (The phase UNKNOWN in the same log is
  the harness cutting power 20 ms after `cal cock_rest`; fixed in the harness.) Re-confirmed on fw-base 6381edd.
- Suggest: at boot, a valid code in another sector than the record = the drum was moved by hand: take the code's
  sector (angle mid-sector), no fault; keep DRUM_SENSOR for invalid codes. New scenario `hand_lever` checks it.

## F27 (fw-base / spec, low) `aim 6` (the D45 rail-low step, as BASE_SPEC 10.1 writes it) is refused `OUT_OF_RANGE`

- Preview fw-base 6381edd, scenario `ref_nominal` (10.1 with step 7b): after the landing, `{"cmd":"aim","deg":6}`
  -> `{"ok":false,"err":"OUT_OF_RANGE"}` every time. `params_gen.h:280 P_AIM_PITCH_MIN 6.0f`, but `get pitch_range`
  is `max(pitchMin, table min)`; the aim table starts at 6.01 deg (`params_gen.h:284`), so `cmdAim` (`base_logic.h:812-813`,
  `deg >= lo - 1e-3`) refuses 6.00 by 0.009 deg. The tow then runs with the rail high (D45 says that loads the carriage stop against the spring).
- Fix: accept a target within `aim_tol` of the range ends (clamp to the end), or make 10.1 / the Jetson aim at
  `get pitch_range` min. The reference driver now falls back to `pitch_range` min and logs it.
- The Jetson mission (w/jetson 0195c38) already sends `aim 6.06` (`power_aim` base.log), so only the spec text and
  any other host that follows it literally are affected.
