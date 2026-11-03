# KEDGE dart HTTP API (contract C2, final, firmware kedge-dart-v4)

The dart is a WiFi access point. Join SSID `KEDGE`, password `grappstar`. The dart is at `http://192.168.4.1` (port 80).
Every call is a plain HTTP GET. The server handles one request at a time, so keep requests short, use a 1 s timeout,
and never poll `/status` faster than 5 Hz (held-key repeats every 150 ms are the exception).

## Conventions

| HTTP code | Meaning | Body |
|---|---|---|
| 200 | accepted | `text/plain` event lines (often empty), or JSON for `/status` and `/flightlog` |
| 409 | refused, nothing changed | `refused: <reason>` on the first line |
| 400 | missing parameter | text |
| 404 | unknown path | text |
| 503 | `/flightlog` could not get memory, try again | text |

Refusal reasons: `in flight`, `busy` (a sequence owns the motors), `already anchored, release first`,
`no current sensor` (INA219 missing or lost), `no IMU` (BNO085 missing or silent), `stalled: release the key first`,
`fault latched: send stop first`, `both directions held`, `sent before a stop`,
`battery low`, `unknown key`, and for `/flightlog` `busy (motors running, armed or in flight)`.

**One request at a time.** If a client opens a connection and never sends a request (a browser tab closing mid
request can do this), the ESP32 web server waits up to 5 s for it before serving anyone else. Motors are not affected
(the deadman and all safety run in their own task), but a `/stop` sent in that window is answered late. Use short
timeouts and retry `/stop`.

**Clients must treat 409 as an answer, not as a broken link.** Do not retry on 409. With Python urllib, catch
`urllib.error.HTTPError` with `e.code == 409` and read `e.read()`.

**Clock.** Every time in this API is the dart's `millis()` since boot. It restarts at 0 after any reboot (see `reset`
in `/status`). To line it up with the Jetson clock, read `t_ms` from `/status`.

## Endpoints

### GET /status

JSON, always 200. Example (written by the host test to `test/out/status_anchoring.json`):

```json
{"fw":"kedge-dart-v4","t_ms":5535,"phase":"ANCHORING","fault":null,"rail_V":7.60,"current_mA":24,"peak_mA":24,
 "speed":255,"motors":{"deploy":0,"kick":1,"drills":0,"foot":0},"seq":"kick swing","armed":false,
 "auto_on_landing":true,"landed":true,"roll":1.5,"pitch":-3.2,"accel_g":1.01,"peak_g":8.00,"ina_ok":true,"imu_ok":true,
 "t_launch_ms":3000,"t_land_ms":4010,"log_frozen":false,"reset":"poweron","event":"auto: kick swing"}
```

| Field | Type | Meaning |
|---|---|---|
| `fw` | string | firmware version |
| `t_ms` | int | dart clock |
| `phase` | string | `IDLE`, `ARMED`, `FLIGHT`, `LANDED`, `ANCHORING`, `ANCHORED`, `RELEASING`, `FAULT` (below) |
| `fault` | string or null | fault code (below), null when none |
| `rail_V` | number | motor rail volts from the INA219; `-1.00` when unknown |
| `current_mA` | number | total motor current, last good reading (stale when `ina_ok` is false; 9999 = INA219 range overflow, which trips) |
| `peak_mA` | number | highest current since the last `/auto`, `/release` or key `i` |
| `speed` | int | PWM duty 50..255 (keys `+` / `-`) |
| `motors` | object | `deploy`, `kick`, `drills`, `foot`: -1, 0 or +1 (+1 = out / in / down, -1 = back / up / unscrew) |
| `seq` | string | sequence step (below) |
| `armed` | bool | landing detector armed (cleared on landing, `/stop`, `/auto`, `/release`) |
| `auto_on_landing` | bool | when armed: start `/auto` by itself on landing |
| `landed` | bool | a landing was detected since the last arm |
| `roll`, `pitch` | number | degrees from the BNO085; stale when `imu_ok` is false |
| `accel_g` | number | last \|a\| in g (gravity included) |
| `peak_g` | number | highest \|a\| since the last arm (check of the impact level after a shot) |
| `ina_ok`, `imu_ok` | bool | INA219 reading (false after 3 failed reads, true again on the next good one; it is re-initialised at 10 Hz if it lost its setup), BNO085 sending data (false after 1 s of silence; its reports are re-enabled every 2 s) |
| `t_launch_ms`, `t_land_ms` | int or null | detected launch and landing times, null when none |
| `log_frozen` | bool | flight log holds a finished flight |
| `reset` | string | why the dart last booted: `poweron`, `software`, `brownout`, `panic`, `task_wdt`, `int_wdt`, `wdt`, `reset_pin`, `deepsleep`, `other` (USB upload) |
| `warnings` | array of strings | what went less than perfectly in the last `/auto` or `/release`, empty when clean: `foot_stalled`, `drills_stalled` (stone or full depth), `kick_plant_time_limit`, `pods_time_limit`, `kick_already_down`, `pods_already_at_stop`. `ANCHORED` with warnings may be a weak anchor |
| `event` | string | last event line |

`seq` values: `idle`, `kick swing`, `kick + foot planting`, `foot finishing`, `deploy + drills`, `drills finishing`,
`anchored`, `aborted`, `release: drills out`, `release: drills out + pods up`, `release: foot out`,
`release: kick back + foot out`, `released`. The anchor names are the same as v3. **Prefer `phase` over `seq` text.**

### GET /key?c=&lt;key&gt;&ms=&lt;run ms&gt;

Manual key, same keys as the web page and Serial Monitor.

- Run keys: `1` / `!` deploy pods down / up, `2` / `@` kick out / back, `3` / `#` drills in / out, `4` / `$` foot in / out.
  `ms` is how long the motor runs if no repeat arrives (deadman). Default 350, clamped to 50..500. To hold a key,
  repeat it every 150 ms with `ms=350`; when the repeats stop, the motor stops within 350 ms.
- A hold (one key repeated without a gap) is capped at 8 s in total, 2 s when the INA219 is missing. After the cap
  the key is refused until it has been released.
- Stall stop: one motor driven into an end stop stops after 0.25 s start-up + 0.2 s over its stall current. Repeats
  within max(last `ms`, 350) + 250 ms (at least 600 ms; 1.75 s for Serial keys) continue the same hold, so slow repeats
  do not restart the start-up time or the hold cap. Stall decisions use a filtered current (noise of about 15 mA does not matter).
- Two motors held at once: the worm of the pair is paused for 150 ms every 550 ms and the other is read alone; a
  stalled motor of the pair is stopped after two stall readings (about 1.1 s) while the other keeps running. Three or
  four at once: the limit is "one stalled while the others work normally".
- Stall currents are bench values at 8.0 V and scale with the rail voltage (0.6 to 1.15 x) and, for manual keys,
  with (`speed`/255) squared (the drivers run in slow decay: one input high, the other PWM inverted).
- A single motor drawing under 40 % of its free current for 0.5 s (open wire, or a short that the driver's own
  protection keeps chopping) is stopped with the event `<motor> draws no current: wire open or shorted`.
- One stalled drill while the other turns cannot be told from normal drilling (both share one channel and one current
  reading); only both stalled is detected.
- Both directions of one motor held at once (`1` and `!`): the motor stops and both are refused (`both directions held`)
  until neither has been pressed for a window. To reverse, release (the page then sends a stop) and press the other.
- At low `speed` the stall line never drops below 1.5 x the motor's free current, so normal running is not taken for a
  stall; below about speed 100 a stall may then only be stopped by the 8 s hold cap.
- Other keys: space or `s` stop (same as `/stop`; with `keep=1` it stops the motors but keeps the arm, like
  `/stop?disarm=0`: the web page sends this when the last held key is released or the page loses focus), `a` auto (same as `/auto`), `r` release (same as `/release`),
  `A` arm toggle (prefer `/arm`), `+` / `-` speed, `i` current, `b` attitude, `p` status line, `f` flight log summary,
  `z` I2C scan (does a full `/stop` first: motors off, disarm), `h` help. An OTA upload also does a full `/stop`.
- Optional `n` and `p`: a request counter and a page id (the web page uses them so a run key sent before that page's
  stop cannot arrive after it, answered 409 `sent before a stop`; `n` only counts together with `p`, and the last 4
  page ids are remembered).
- Refused (409) while a sequence runs (`busy`), during `FLIGHT`, while a fault is latched (`fault latched: send stop
  first`), and after a stall or the hold cap until the key has been released (`stalled: release the key first`).
  "Released" means no press for max(last `ms`, 350) + 250 ms. The opposite direction is never locked.
- A launch detected while a manual motor runs stops it (event `launch: motors stopped`).
- Sequences (`/auto`, `/release`) always run at full duty, whatever `speed` is.
- A manual run key moves the legs out of a known state, so `seq` `anchored` / `released` / `aborted` becomes `idle`.

### GET /stop

Stops every motor at once, aborts any sequence (`seq` becomes `aborted`), clears the fault, and **disarms**. A run
key that any client is still repeating (also in a deadman gap, or refused while a sequence ran) stays locked until
that key has been released, so a stop from the Jetson also holds against a phone still holding a button. The web
page's own key-release stop (`/key?c=%20&keep=1`) does not lock, so quick taps work. During
`FLIGHT` it also ends flight tracking (`landing` = `stop`, `t_land_ms` = now, log freezes 3 s later).
`/stop?disarm=0` stops the motors, aborts any sequence and clears the fault, but keeps the arm and flight tracking.
Always 200.

### GET /auto

Starts the anchor sequence: kick swing, kick + foot planting (stall on contact), foot finishing, pods down with drills
spinning, drills finishing, `anchored`.

- Refused in `FLIGHT` (decision below), while releasing (`busy`), when already anchored, and without the INA219.
- While `ARMED` (the launch was never seen) it disarms first (event `auto: disarmed first`) and starts.
- A repeated `/auto` while anchoring is accepted and does not restart it (safe to retry after a timeout).
- Accepted from `IDLE`, `ARMED`, `LANDED` and `FAULT` (it clears the fault).
- Timings: kick swing until contact (over 60 mA for 150 ms after 250 ms start-up; kick time limit 15.5 s, then
  `KICK_TIMEOUT`); kick + foot planting until the housing stalls (or 6 s); foot finishing 1.5 s; pods down until they
  stall (time limit 6 s, then it carries on, no fault); drills finishing 3 s. Whole sequence limit 35 s.
- A single motor that draws no current faults the sequence (`NO_CURRENT`). The anchor's pods step starts with the
  deploy worm alone for 0.3 s to check it draws current (open wire: `NO_CURRENT`). In the release the drills keep
  unscrewing from the start of the pods lift (the augers may still be in soil).
- Kick travel bookkeeping: the firmware adds up how long the kick has been driven out and back (at full duty) and
  assumes it was **stowed at power-on**. A kick that stalls near stowed at the start of `/auto` is a jam or wired
  backwards (`KICK_JAM`), not "already down"; a kick that stalls before it has come back half way in `/release` is
  `KICK_JAM`, not "stowed". Travel is not counted while the kick is stalled or planting. After any reset other than a
  power-on (brownout, crash, OTA) the position is unknown and these checks wait until the next completed release.
  `kickSwingMs` (4.3 s) is the bench swing time (BENCH_TEST step 5).
- Stall handling inside the sequence: while two motors run (kick + foot, pods + drills), the worm (kick or deploy)
  pauses 150 ms every 550 ms. It self-locks and holds position while the auger keeps turning and is read alone; the
  worm's current is (both) - (auger alone). A worm stall must also show as a rise of the total current over the
  phase's lowest reading, so an auger whose current changes during the pause cannot fake one. A stalled auger is
  stopped (anchor) or faults (release). An end-stop stall is only believed after a minimum travel time (pods 0.7 s,
  kick back 1.5 s). If the kick's housing stall is never seen, planting carries on after 6 s (`kick plant time limit`).
  A worm already at its end stop when such a step starts shows no rise; it is still stopped after 3 readings in a row
  over the line (3 of the last 4 readings, about 1.6 to 2.2 s). An auger current change of up to about 100 mA during
  the pause is tolerated. Known limit: if the pods jam in soil mid-travel and the drills' current rises during each
  pause by more than the margin, the jam is only ended by the 6 s pods limit. A retried
  `/auto` whose kick is already down skips the planting step (event `kick already down, foot planting`).
- One failed current read at the moment a step ends does not fault: the step ends and the next good read moves on.

### GET /release

Starts the release sequence, the anchor sequence backwards:

1. drills unscrew alone for 3 s (`release: drills out`)
2. pods lift while the drills keep unscrewing, until the pods stall on their stowed stop (after 6 s it carries on, event
   `release: pods time limit, foot out`)
3. foot unscrews alone for 1.5 s (`release: foot out`)
4. kick swings back while the foot keeps unscrewing for 3 s more, until the kick stalls on its stowed stop (15 s limit)
5. `seq` = `released`, phase `IDLE`, all motors off.

C2 listed the parts as "drills out, foot out, kick back, deploy up". The firmware runs them in the exact reverse order
of `/auto` (pods up before the kick comes back), because the kick goes down first and the pods last.

- Refused in `FLIGHT`, while anchoring (`busy`, send `/stop` first), and without the INA219.
- A repeated `/release` while releasing is accepted and does not restart it.
- Faults: drills or foot jammed (`DRILL_JAM`, `FOOT_JAM`), kick not stowed in 15 s (`KICK_TIMEOUT`), whole sequence over
  35 s (`SEQ_TIMEOUT`). All motors stop on any fault.

### GET /arm?on=1&auto=1

- `on=1` (default): arm the landing detector. `auto=1` (default) starts `/auto` by itself on landing; `auto=0` only
  records the flight log. Re-arming resets the flight detector, `landed`, and unfreezes the log.
- `on=0`: disarm. Always accepted, also in FLIGHT (then no auto on landing; the log still freezes after landing).
- Arming clears a latched fault. Key `A` toggles: when disarmed it arms with `auto=1`.
- Refused without the BNO085 (`no IMU`), with `auto=1` and no INA219, while a sequence runs, in `FLIGHT`, and with
  `auto=1` when the rail is under 6.6 V at rest, including 0 V with the plug out (`battery low`; a log-only arm is
  still allowed).
- An arm expires after 15 min without a launch (event `arm expired`).
- Idempotent: use this instead of the `A` toggle. Arm right before the shot, because `/stop` disarms.
- Arming from `ANCHORED`, `aborted` or `released` starts a new shot (`seq` back to `idle`).

### GET /flightlog

The BNO085 log, streamed as JSON (chunked, up to about 40 kB). The server is busy for about 0.3 s while it sends,
so it is refused (409 `busy (motors running, armed or in flight)`) while any motor runs, while armed and in FLIGHT.
Fetch it after landing (phase `LANDED` or `ANCHORED`).

```json
{"hz":50,"frozen":true,"phase":"ANCHORED","t_launch_ms":3000,"t_land_ms":4010,"landing":"impact","n":301,
 "fields":["t_ms","qw","qx","qy","qz","ax","ay","az"],
 "samples":[[1005,1.0000,0.0000,0.0000,0.0000,0.00,0.00,9.81], ...]}
```

| Field | Meaning |
|---|---|
| `hz` | nominal rate, 50. Samples sit on a 20 ms grid with up to 5 ms jitter; use each `t_ms` |
| `frozen` | true once the window after a flight is complete |
| `t_launch_ms`, `t_land_ms` | as in `/status`, null when none |
| `landing` | how the landing was decided: `impact`, `still`, `timeout`, `stop`; null when not landed |
| `n` | number of samples |
| `samples` | `[t_ms, qw, qx, qy, qz, ax, ay, az]`, oldest first |

- Quaternion: BNO085 game rotation vector, order w, x, y, z, 4 decimals. No magnetometer, so yaw is relative to the
  power-on heading.
- Acceleration: m/s², 2 decimals, gravity included, BNO085 chip axes. The BNO085 saturates near 8 g (78 m/s²), so the
  launch push and the impact read flat at about 78.
- Chip axes: how the BNO085 axes sit in the hull is not fixed by the firmware. BENCH_TEST.md step 7 finds them by
  standing the dart on its tail (nose up) and on its side and writes them down. Clients must use that mapping, not assume one.
- Window: when a flight was detected, the log runs from 2 s before launch until 3 s after landing, then freezes
  (`frozen: true`) and stays until the next arm. The ring holds 12 s (600 samples): if the landing is decided more
  than about 7 s after launch (a long flight, or a `timeout` landing), the log freezes 10 s after launch instead, so
  it always keeps 2 s before launch, the launch and the first 10 s; the end of a very long flight is then missing. Before any launch the reply
  is the last 12 s, live (`frozen: false`, `t_launch_ms: null`).
- `t_land_ms` is the last impact for `impact`, the start of stillness for `still`, for `timeout` the last impact if one
  was seen (else the time of the decision), and the time of the `/stop` for `stop`.

### GET /

The driving web page (hold-to-run buttons and keys, STOP, AUTO, RELEASE, arm, live status, flight log download).

## Phases

Worked out in this priority order (the first that applies):

| Phase | When |
|---|---|
| `FAULT` | a fault code is set (latched until `/stop`, `/auto`, `/release` or `/arm`) |
| `RELEASING` | release sequence running |
| `ANCHORING` | anchor sequence running |
| `FLIGHT` | launch detected, landing not yet decided |
| `ANCHORED` | anchor sequence finished (`seq` `anchored`) |
| `LANDED` | landed and no motor running (log-only arm, fallback landing, auto refused, or auto stopped by hand); a manual motor running after a landing shows `IDLE` |
| `ARMED` | landing detector armed, waiting for the launch |
| `IDLE` | none of the above (also after `/stop` aborts a sequence before any landing: `seq` `aborted`, no fault; after a landing it shows `LANDED`) |

Normal mission: `IDLE > ARMED > FLIGHT > ANCHORING > ANCHORED > RELEASING > IDLE` (tested). With auto on landing the
dart goes from `FLIGHT` straight to `ANCHORING`. A client waiting for `ANCHORED` must also give up on `FAULT` and on
`seq == "aborted"`.

## Faults

| Code | Cause | What the dart did |
|---|---|---|
| `OVERCURRENT` | motor current over 2000 mA for 10 ms while a motor runs (or the INA219 range overflowed) | all motors off |
| `KICK_TIMEOUT` | kick found no ground in 15.5 s (anchor), or no stow stop in 15 s (release), or a kick timer ran out while current reads were failing | all off |
| `SEQ_TIMEOUT` | anchor or release sequence over 35 s | all off |
| `NO_CURRENT` | a lone motor in a sequence drew under 40 % of its free current for 0.5 s: open wire or shorted output | all off |
| `DRILL_JAM`, `FOOT_JAM` | stalled while unscrewing in release | all off |
| `CURRENT_SENSOR` | 3 failed INA219 reads in a row during a sequence | all off; manual keys still work on timers |
| `LOW_RAIL` | rail under 5.6 V (including a dead 0 V rail) for 0.5 s while a motor runs | all off |
| `KICK_JAM` | kick stalled near stowed at the start of `/auto`, or before it came back half way in `/release` | all off |
| `BROWNOUT` | the dart rebooted from a brownout | booted with motors off |
| `WATCHDOG` | the dart rebooted from a crash or the 2 s safety-task watchdog | booted with motors off |

## Flight detection (decision record)

- Launch: \|a\| over 4 g continuously for 20 ms, **only while armed**, and not within 400 ms after free fall (\|a\|
  under 0.5 g for 40 ms or more). A dropped dart is in free fall before it hits, so a drop is never taken as a launch;
  rail vibration, tilts, the cocking jerk and the push's own ramp never read that low, so they do not matter. Launch
  time = start of those 20 ms.
  In practice the samples over 4 g must span at least 15 ms (one 5 ms sample period of slack): pushes of 24 ms or more
  are always seen (18 ms with steady 5 ms ticks; measured by the fuzz test with 4 to 7 ms tick jitter), knocks of 10 ms
  or less never, 15 to 23 ms sometimes. A launch alone never starts motors.
- Landing: once the dart has been in free fall (\|a\| under 0.6 g for 60 ms in total, or, for a tumbling dart that
  never reads that low, 400 ms after launch with \|a\| back under 4 g),
  an impact over 3.5 g at least 400 ms after launch (real flights last 0.6 s or more; this stops a dropped dart's bounce
  counting as a landing), then \|a\| between 0.85 and 1.15 g for 1 s. Each new impact restarts the stillness timer;
  landing time = the last impact. The free-fall rule stops a bump on the bench from counting as a flight.
- **Impact level 3.5 g, not the 6 g in C2.** The BNO085 clips at 8 g and low-pass filters its output, so a 2 ms
  impact can read only about 5.7 g; free flight reads under about 1.5 g even tumbling, so 3.5 g is a safe line.
  `peak_g` in `/status` shows what a real landing read; tune `impactG` from drop tests.
- **A reboot during the flight** (a short power break at impact: the JST plug is the only power path) loses the
  armed state and the log. `/status` then shows a small `t_ms` and `reset` `brownout` or `poweron` with phase `FAULT`
  or `IDLE`. The Jetson should treat that after a shot as "landed": send `/stop`, then `/auto`.
- Fallbacks: no impact seen but still for 2 s after free fall (soft landing in grass: `still`); no decision 15 s after
  launch (IMU silent, or a bump that was never a flight: `timeout`). **A fallback landing does not start the auto
  sequence** (event `no impact seen: auto not started, send /auto`); the phase is `LANDED` and the Jetson decides from
  the video.
- **`/auto` is refused during FLIGHT.** Running motors in the air would swing the legs out, spoil the flight and could
  foul the tether, and the landing detector starts the sequence itself on landing. If detection fails, the 15 s timeout
  ends FLIGHT, and `/stop` ends it at once.
- Manual keys and `/release` are refused during FLIGHT for the same reason.

## Changes from v3 (for client authors)

- New: `/release`, `/arm`, `/flightlog`, `/stop?disarm=0`, and the status fields `fw`, `phase`, `fault`,
  `auto_on_landing`, `accel_g`, `peak_g`, `ina_ok`, `imu_ok`, `t_launch_ms`, `t_land_ms`, `log_frozen`, `reset`.
- Refusals are now HTTP 409 (v3 silently ignored keys during auto).
- `/stop` disarms. `armed` means "landing detector armed"; the auto choice is `auto_on_landing`.
- `/key` `ms` is capped at 500 (was 5000); a hold is capped at 8 s. A held key that stalls stays off until released.
- The release used to be held keys from the Jetson (`#` + `!`, then `$` + `@`). Use `/release` and wait for
  `seq == "released"`; fail on `FAULT` or `aborted`.
