# KEDGE dart HTTP API (contract C2, final, firmware kedge-dart-v4)

The dart is a WiFi access point. Join SSID `KEDGE`, password `grappstar`. The dart is at `http://192.168.4.1` (port 80).
Every call is a plain HTTP GET. The server handles one request at a time, so keep requests short, use a 1 s timeout,
and never poll faster than 5 Hz.

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
`unknown key`.

**Clients must treat 409 as an answer, not as a broken link.** Do not retry on 409. With Python urllib, catch
`urllib.error.HTTPError` with `e.code == 409` and read `e.read()`.

**Clock.** Every time in this API is the dart's `millis()` since boot. It restarts at 0 after any reboot (see `reset`
in `/status`). To line it up with the Jetson clock, read `t_ms` from `/status`.

## Endpoints

### GET /status

JSON, always 200. Example (from the host test, `test/out/status_anchoring.json`):

```json
{"fw":"kedge-dart-v4","t_ms":4515,"phase":"ANCHORING","fault":null,"rail_V":7.60,"current_mA":25,"peak_mA":25,"speed":255,
 "motors":{"deploy":0,"kick":1,"drills":0,"foot":0},"seq":"kick swing","armed":false,"auto_on_landing":true,"landed":true,
 "roll":1.5,"pitch":-3.2,"accel_g":1.00,"ina_ok":true,"imu_ok":true,"t_launch_ms":3000,"t_land_ms":4010,"log_frozen":false,
 "reset":"poweron","event":"auto: kick swing"}
```

| Field | Type | Meaning |
|---|---|---|
| `fw` | string | firmware version |
| `t_ms` | int | dart clock |
| `phase` | string | `IDLE`, `ARMED`, `FLIGHT`, `LANDED`, `ANCHORING`, `ANCHORED`, `RELEASING`, `FAULT` (below) |
| `fault` | string or null | fault code (below), null when none |
| `rail_V` | number | motor rail volts from the INA219; `-1.00` when unknown |
| `current_mA` | number | total motor current; 0 when the INA219 is lost (check `ina_ok`) |
| `peak_mA` | number | highest current since the last `/auto`, `/release` or key `i` |
| `speed` | int | PWM duty 50..255 (keys `+` / `-`) |
| `motors` | object | `deploy`, `kick`, `drills`, `foot`: -1, 0 or +1 (+1 = out / in / down, -1 = back / up / unscrew) |
| `seq` | string | sequence step (below) |
| `armed` | bool | landing detector armed (cleared on landing, `/stop`, `/auto`, `/release`) |
| `auto_on_landing` | bool | when armed: start `/auto` by itself on landing |
| `landed` | bool | a landing was detected since the last arm |
| `roll`, `pitch` | number | degrees from the BNO085; stale when `imu_ok` is false |
| `accel_g` | number | last \|a\| in g (gravity included) |
| `ina_ok`, `imu_ok` | bool | INA219 reading, BNO085 sending data |
| `t_launch_ms`, `t_land_ms` | int or null | detected launch and landing times, null when none |
| `log_frozen` | bool | flight log holds a finished flight |
| `reset` | string | why the dart last booted: `poweron`, `software`, `brownout`, `panic`, `task_wdt`, `int_wdt`, `wdt`, `reset_pin`, `deepsleep`, `other` (USB upload) |
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
- Stall stop: a motor driven into an end stop stops after 0.25 s start-up + 0.2 s over its stall current. The stall
  currents scale with `speed`. With several motors held at once the limit is "one motor stalled, the others working
  normally", so one stalled motor is caught while the others run.
- Other keys: space or `s` stop (same as `/stop`), `a` auto (same as `/auto`), `r` release (same as `/release`),
  `A` arm toggle (prefer `/arm`), `+` / `-` speed, `i` current, `b` attitude, `p` status line, `f` flight log summary,
  `z` I2C scan (stops motors first), `h` help.
- Optional `n`: a request counter (the web page uses it so a run key that was sent before a stop cannot arrive after it).
- Refused (409) while a sequence runs (`busy`), during `FLIGHT`, and after a stall or the hold cap until the key has
  been released for 0.5 s (`stalled: release the key first`). The opposite direction is never locked.
- Sequences (`/auto`, `/release`) always run at full duty, whatever `speed` is.
- A manual run key moves the legs out of a known state, so `seq` `anchored` / `released` / `aborted` becomes `idle`.

### GET /stop

Stops every motor at once, aborts any sequence (`seq` becomes `aborted`), clears the fault, and **disarms**. During
`FLIGHT` it also ends flight tracking (`landing` = `stop`, `t_land_ms` = now, log freezes 3 s later).
`/stop?disarm=0` stops the motors but keeps the arm and flight tracking. Always 200.

### GET /auto

Starts the anchor sequence: kick swing, kick + foot planting (stall on contact), foot finishing, pods down with drills
spinning, drills finishing, `anchored`.

- Refused in `FLIGHT` (decision below), while releasing (`busy`), when already anchored, and without the INA219.
- A repeated `/auto` while anchoring is accepted and does not restart it (safe to retry after a timeout).
- Accepted from `IDLE`, `ARMED`, `LANDED` and `FAULT` (it clears the fault).
- Stall handling inside the sequence: the main motor is read alone by pausing the helper for 150 ms every 550 ms; the
  helper (foot while the kick plants, drills while the pods swing) is stopped if it stalls, and the sequence carries on.

### GET /release

Starts the release sequence, the anchor sequence backwards:

1. drills unscrew alone for 3 s (`release: drills out`)
2. pods lift while the drills keep unscrewing, until the pods stall on their stowed stop (6 s limit)
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
- `on=0`: disarm.
- Refused without the BNO085 (`no IMU`), with `auto=1` and no INA219, while a sequence runs, and in `FLIGHT`.
- Idempotent: use this instead of the `A` toggle. Arm right before the shot, because `/stop` disarms.
- Arming from `ANCHORED`, `aborted` or `released` starts a new shot (`seq` back to `idle`).

### GET /flightlog

The BNO085 log, streamed as JSON (chunked, up to about 40 kB). The server is busy for about 0.3 s while it sends,
so it is refused (409 `busy (motors running)`) while any motor runs.

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
  (`frozen: true`) and stays until the next arm. The ring holds 12 s (600 samples): in a flight longer than about
  7 s the earliest samples are gone and the first sample comes after `t_launch_ms - 2000`. Before any launch the reply
  is the last 12 s, live (`frozen: false`, `t_launch_ms: null`).
- `t_land_ms` is the last impact for `impact`, the start of stillness for `still`, and the time of the decision for
  `timeout` and `stop`.

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
| `LANDED` | landed and nothing running (log-only arm, fallback landing, auto refused, or auto stopped by hand) |
| `ARMED` | landing detector armed, waiting for the launch |
| `IDLE` | none of the above (also after `/stop` aborts a sequence: `seq` `aborted`, no fault) |

Normal mission: `IDLE > ARMED > FLIGHT > ANCHORING > ANCHORED > RELEASING > IDLE` (tested). With auto on landing the
dart goes from `FLIGHT` straight to `ANCHORING`. A client waiting for `ANCHORED` must also give up on `FAULT` and on
`seq == "aborted"`.

## Faults

| Code | Cause | What the dart did |
|---|---|---|
| `OVERCURRENT` | motor current over 2000 mA for 10 ms (or the INA219 range overflowed) | all motors off |
| `KICK_TIMEOUT` | kick found no ground in 15 s (anchor) or no stow stop in 15 s (release) | all off |
| `SEQ_TIMEOUT` | anchor or release sequence over 35 s | all off |
| `DRILL_JAM`, `FOOT_JAM` | stalled while unscrewing in release | all off |
| `CURRENT_SENSOR` | 3 failed INA219 reads in a row during a sequence | all off; manual keys still work on timers |
| `LOW_RAIL` | rail under 5.6 V for 0.5 s while a motor runs | all off |
| `BROWNOUT` | the dart rebooted from a brownout | booted with motors off |
| `WATCHDOG` | the dart rebooted from a crash or the 2 s safety-task watchdog | booted with motors off |

## Flight detection (decision record)

- Launch: \|a\| over 4 g continuously for 20 ms, **only while armed**. Launch time = start of those 20 ms.
  In practice the samples over 4 g must span at least 15 ms (one 5 ms sample period of slack), so a 20 ms push is seen
  and a 15 ms knock is not.
- Landing: once the dart has been in free fall (\|a\| under 0.6 g for 60 ms in total) and at least 150 ms after launch,
  an impact over 6 g, then \|a\| between 0.85 and 1.15 g for 1 s. Each new impact restarts the stillness timer;
  landing time = the last impact. The free-fall rule stops a bump on the bench from counting as a flight.
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
  `auto_on_landing`, `accel_g`, `ina_ok`, `imu_ok`, `t_launch_ms`, `t_land_ms`, `log_frozen`, `reset`.
- Refusals are now HTTP 409 (v3 silently ignored keys during auto).
- `/stop` disarms. `armed` means "landing detector armed"; the auto choice is `auto_on_landing`.
- `/key` `ms` is capped at 500 (was 5000); a hold is capped at 8 s. A held key that stalls stays off until released.
- The release used to be held keys from the Jetson (`#` + `!`, then `$` + `@`). Use `/release` and wait for
  `seq == "released"`; fail on `FAULT` or `aborted`.
