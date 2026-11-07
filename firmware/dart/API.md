# KEDGE dart HTTP API (contract C2, firmware kedge-dart-v4.1)

The dart is a WiFi access point. Join SSID `KEDGE`, password `grappstar`. The dart is at `http://192.168.4.1` (port 80).
Every call is a plain HTTP GET. The server handles one request at a time, so keep requests short, use a 1 s timeout,
and never poll `/status` faster than 5 Hz (held-key repeats every 150 ms are the exception).

## Conventions

| HTTP code | Meaning | Body |
|---|---|---|
| 200 | accepted | `text/plain` event lines (often empty), or JSON for `/status` and `/flightlog` |
| 409 | refused, nothing changed (one exception: `both directions held` also stops that motor) | `refused: <reason>` on the first line |
| 400 | missing parameter | text |
| 404 | unknown path | text |
| 503 | `/flightlog` could not get memory, try again | text |

Refusal reasons: `in flight`, `busy` (a sequence owns the motors), `already anchored, release first`,
`no current sensor` (INA219 missing or lost), `no IMU` (BNO085 missing or silent), `stalled: release the key first`, `hold limit: release the key first`,
`fault latched: send stop first`, `both directions held`, `sent before a stop`,
`battery low`, `anchored: release first`, `legs out: release first`, `unknown key`, and for `/flightlog` `busy (motors running)`, `armed (disarm first)`, `in flight`.
v4.1 adds, for `/auto`: `motors hot after a long milling: wait 60 s`, `not upright: on its side (force=1 tries anyway)`, `not upright: upside down (...)`,
`not upright: nose down (...)`, `not upright: nose up (...)`, and `battery low: charge it (release still works)`.

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
 "t_launch_ms":3000,"t_land_ms":4010,"log_frozen":false,"reset":"poweron","warnings":[],"video":true,"batt":"ok",
 "batt_V":7.60,"orient":"upright","tilt_deg":2,"roll_g":0,"pitch_g":0,"soil_mA":-1,"moving":false,"event":"auto: kick swing"}
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
| `armed` | bool | landing detector armed (cleared on landing, `/stop`, `/auto`, `/release`, `/arm?on=0`, or the 15 min arm expiry) |
| `auto_on_landing` | bool | when armed: start `/auto` by itself on landing |
| `landed` | bool | a landing was detected since the last arm |
| `roll`, `pitch` | number | degrees from the BNO085; stale when `imu_ok` is false |
| `accel_g` | number | last \|a\| in g (gravity included) |
| `peak_g` | number | highest \|a\| since the last arm (check of the impact level after a shot) |
| `ina_ok`, `imu_ok` | bool | INA219 reading (false after 3 failed reads, true again on the next good one; it is re-initialised at 10 Hz if it lost its setup), BNO085 sending data (false after 1 s of silence; its reports are re-enabled every 2 s) |
| `t_launch_ms`, `t_land_ms` | int or null | detected launch and landing times, null when none |
| `log_frozen` | bool | flight log holds a finished flight |
| `reset` | string | why the dart last booted: `poweron`, `software`, `brownout`, `panic`, `task_wdt`, `int_wdt`, `wdt`, `reset_pin`, `deepsleep`, `other` (USB upload) |
| `warnings` | array of strings | what went less than perfectly in the last `/auto` or `/release`, empty when clean: `foot_stalled`, `drills_stalled` (stone or full depth), `kick_plant_time_limit`, `pods_time_limit`, `kick_already_down`. `ANCHORED` with warnings may be a weak anchor |
| `video` | bool | v4.1: camera + VTX power wanted on (the optional switch on D10; without it the video is always on) |
| `batt` | string | v4.1: battery at rest (no motor for 1 s): `ok`, `low` (under 6.8 V: about 10 % left), `critical` (under 6.4 V: `/auto` refused, video switched off, `/release` and keys still work), `none` (rail under 3 V: plug out), `unknown` (no reading yet). 0.1 V hysteresis |
| `batt_V` | number | v4.1: the resting battery voltage behind `batt` (-1 before the first reading) |
| `orient` | string | v4.1: from the BNO085 accelerometer while still: `upright`, `leaning` (roll 12 to 60 deg), `side`, `inverted`, `nose_down`, `nose_up`, `unknown` (IMU silent or moving). Until the nose axis is set (`imuNoseAxis`, BENCH_TEST step 7) only `upright` / `side` / `inverted` from the tilt (70 / 120 deg) |
| `tilt_deg` | number | v4.1: angle between the dart's up axis and straight up (0 when `orient` is unknown) |
| `roll_g`, `pitch_g` | number | v4.1: body roll and nose pitch from gravity (0 until the nose axis is set) |
| `soil_mA` | number | v4.1: highest drill-pair current read alone while the augers were in soil in the last `/auto` (at 8.0 V; -1 = none). dart-physics: about 160 sand, 640 loam (7.4 V) |
| `moving` | bool | v4.1: while `ANCHORED`: the dart tilted over 2 deg from where it anchored, or \|a\| off 1 g by 0.15 g, for 200 ms (the anchor is moving under the tow: stop the tow). Event `anchor moving` on each start |
| `event` | string | last event line |

`seq` values: `idle`, `kick swing`, `kick + foot planting`, `foot finishing`, `deploy + drills`, `drills finishing`,
`anchored`, `aborted`, `release: drills out`, `release: drills out + pods up`, `release: foot out`,
`release: kick back + foot out`, `released`, and v4.1 `levelling: pods to 20 deg, drills off`. The anchor names are the same as v3. **Prefer `phase` over `seq` text.**

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
- Both directions of one motor held at once (`1` and `!` both being repeated): the motor stops and both are refused
  (`both directions held`) until neither has been pressed for a window. A single switch from one key to the other
  (release one, press the other, with or without a stop between) just reverses the motor. The 8 s hold cap counts
  per motor, so switching direction does not restart it.
- At low `speed` the stall line never drops below 1.5 x the motor's free current, so normal running is not taken for a
  stall; below about speed 70 a stall may then only be stopped by the 8 s hold cap.
- Other keys: space or `s` stop (same as `/stop`; with `keep=1` it only stops manual (key) motors: no
  sequence abort, no fault clear, no disarm, no lock; the web page sends this when the last held key is released or the
  page loses focus), `a` auto (same as `/auto`), `r` release (same as `/release`),
  `A` arm toggle (prefer `/arm`), `+` / `-` speed, `i` current, `b` attitude, `p` status line, `f` flight log summary,
  `z` I2C scan (does a full `/stop` first: motors off, disarm), `h` help. An OTA upload also does a full `/stop`.
- Optional `n` and `p`: a request counter and a page id (the web page uses them so a run key sent before that page's
  stop cannot arrive after it, answered 409 `sent before a stop`; `n` only counts together with `p`, and the last 4
  page ids are remembered).
- Refused (409) while a sequence runs (`busy`), during `FLIGHT`, while a fault is latched (`fault latched: send stop
  first`), and after a stall until the key has been released (`stalled: release the key first`), or after the 8 s hold cap
  (`hold limit: release the key first`; a client can tell it from an end stop).
  "Released" means no press for max(last `ms`, 350) + 250 ms. A stall lock never blocks the opposite direction
  (both directions held is the exception, see above).
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

### Client hold contract (agreed with the Jetson client, `Dart.hold()`)

How a client holds a run key, for example to bring the kick back (`@`) or push it out (`2`):

1. Send `/key?c=<key>&ms=350` every 150 ms for as long as the hold should last.
2. A refusal `stalled: release the key first` is the normal end: the motor hit its end stop and the dart already
   stopped it. Stop repeating; do not treat it as an error.
   A refusal `hold limit: release the key first` means the 8 s cap (per motor, over direction switches) ended the
   hold before any end stop: the move is not finished; report it, release, and hold again if needed.
3. Always finish with `/stop?disarm=0`: every motor off, the arm and flight tracking kept, no key locked.
4. Effect on the kick motor: it runs only while the repeats keep coming, stops within 350 ms of the last one, and
   stops by itself at a stall. Effect on the arm: none (a hold never disarms; only a plain `/stop` does).
5. The stall lock is per direction. After an `@` hold ends stalled, a `2` hold runs at once, and a fresh `@` hold
   after `2` presses also runs (the old `@` lock is dropped once `@` repeats have stopped).

### GET /auto (and /auto?force=1)

Starts the anchor sequence: kick swing, kick + foot planting (stall on contact), foot finishing, pods down with drills
spinning, drills finishing, `anchored`.

v4.1 (dart-physics findings, 7 Nov 2026):
- **Orientation gate.** `orient` `side`, `inverted`, `nose_down` or `nose_up`: refused (`not upright: ...`), nothing moves.
  The rover should reel in and fire again. `/auto?force=1` skips this check (the IMU axes are wrong, or the video shows
  it upright). `leaning` (roll 10 to 60 deg, the usual rest on hard ground: the dart leans about 38 deg on a pod): the
  pods first swing about 20 deg with the drills off (`seq` `levelling`, 0.65 s, then 0.6 s settle), which levels the
  hull; still not upright after that is fault `NOT_UPRIGHT`. Orientation unknown (IMU silent): no gate.
- **Battery critical** (`batt` `critical`): refused (`battery low: ...`), so the dart never anchors without the charge to
  let go. `/release` always runs.
- **Milling deploy.** The augers sweep sideways through soil, so the deploy worm stalls in soil long before its stop. In
  the pods step, a deploy stall before 85 % of the estimated pods travel (`podsTravelMs`; the estimate is the travel up
  to the stall reading, at most one 0.55 s cycle long) is soil: the deploy pauses 0.2 s while the drills keep cutting,
  then pushes again (event `pods in soil, milling`). No estimated travel gained for 5 s ends the step: taken as the stop
  when past 60 % (`pods down (no progress, past 60 %: the stop)`), else warning `pods_time_limit` (firm soil). The
  pods step is limited to 6 s until milling starts, 150 s with milling (loam takes about 90 s; whole sequence 180 s).
  After a pods step over 60 s the next `/auto` is refused for 60 s (`motors hot after a long milling: wait 60 s`). With the travel unknown
  (after a reset or hand moves with other motors) the first stall is the stop, as in v4.
- **Kick pulsing.** When planting ends on a kick stall, the kick pushes again in pulses (0.4 s on, 0.2 s off) while the
  foot finishes (in loam the kick stalls a few degrees short of its rest; dart-physics). Event `kick down, foot
  finishing (kick pulsing)`.
- **Drill relax.** When the drill pair, read alone in the worm's pause, is over 85 % of its stall line, the deploy pauses
  0.5 s so the cut relaxes (event `drills near stall, deploy paused 0.5 s`).
- Known limit: a soil stall past about 85 % of the estimated travel (less up to one cycle, so possibly from about 50 %)
  is taken as the stop (weak anchor; a pods-down hall sensor would fix it, see the notes).
- Known limit: pods already stuck in soil at the first readings of the step, with read noise near the margin, show no
  rise and may not read as stalled; the 5 s no-progress rule ends that (6 s when no milling started).

- Refused in `FLIGHT` (decision below), while releasing (`busy`), when already anchored, and without the INA219.
- While `ARMED` (the launch was never seen) it disarms first (event `auto: disarmed first`) and starts.
- A repeated `/auto` while anchoring is accepted and does not restart it (safe to retry after a timeout).
- Accepted from `IDLE`, `ARMED`, `LANDED` and `FAULT` (it clears the fault).
- Timings: kick swing until contact (over 60 mA for 150 ms after 250 ms start-up; kick time limit 15.5 s, then
  `KICK_TIMEOUT`); kick + foot planting until the housing stalls (or 6 s); foot finishing 1.5 s; pods down until they
  stall (time limit 6 s, 150 s once milling started, then it carries on, no fault); drills finishing 3 s. Whole sequence limit 180 s (v4.1).
- A single motor that draws no current faults the sequence (`NO_CURRENT`). The anchor's pods step starts with the
  deploy worm alone for 0.3 s to check it draws current (open wire: `NO_CURRENT`). In the release the drills keep
  unscrewing from the start of the pods lift (the augers may still be in soil).
- Kick travel bookkeeping: the firmware adds up how long the kick has been driven out and back (at full duty, scaled
  by rail voltage / 8.0 V) and
  assumes it was **stowed at power-on**. A kick that stalls near stowed at the start of `/auto` is a jam or wired
  backwards (`KICK_JAM`), not "already down"; in `/release` a kick that stalls before its real travel back covers
  60 % of the estimated travel out is `KICK_JAM`, not "stowed" (a jam in the last part of the return looks like the
  stowed stop). Travel is not counted while the kick is stalled (either way) or planting. After any reset other than a
  power-on (brownout, crash, OTA) the position is unknown and these checks wait until the next completed release.
  `kickSwingMs` (4.3 s) is the kick's swing in air at 8.0 V. The firmware measures it: a kick driven out by hand from
  stowed until it stalls logs `kick travel from stowed NNNN ms (at 8.0 V)` (BENCH_TEST step 5).
- Stall handling inside the sequence: while two motors run (kick + foot, pods + drills), the worm (kick or deploy)
  pauses 150 ms every 550 ms. It self-locks and holds position while the auger keeps turning and is read alone; the
  worm's current is (both) - (auger alone). A worm stall must also show as a rise of the total current over the
  phase's lowest reading, so an auger whose current changes during the pause cannot fake one. A stalled auger is
  stopped (anchor; it is not driven again in the next step) or faults (release). An end-stop stall is only believed after a minimum travel time (pods 0.7 s,
  kick back 1.5 s). If the kick's housing stall is never seen, planting carries on after 6 s (`kick plant time limit`).
  A worm already at its end stop when such a step starts shows no rise; it is still stopped
  over the line in 3 of the first 4 readings of the step (about 1.6 to 2.2 s). The rise asked for is half the way from
  the worm's own lowest reading in the step to its line (at least 25 mA); if the other motor draws nothing (open wire)
  no rise is needed. Known limit: an auger whose current drops during every pause by more than the worm's margin
  (about 80 mA for the kick while planting, about 100 mA for the pods) fakes an end stop from the start of the step;
  the step then ends early with no warning (weak anchor). BENCH_TEST step 9.6 measures this drop. Known limit: if the pods jam in soil mid-travel and the drills' current rises during each
  pause by more than the margin, the jam is only ended by the 6 s pods limit. A retried
  `/auto` whose kick is already planted (an earlier anchor got past the swing, or the kick was driven out by hand
  until it stalled) skips the planting step (event `kick already down, foot planting`).
- `NO_GROUND`: the kick stalls in the swing step with no contact phase while it is at 90 % or more of `kickSwingMs`
  (its swing in air) and has not been planted since the last release. That is its own end stop in air (dart on its
  back or side): fault, foot, pods and drills are not driven. A retry, or a swing stopped part way and retried, gives
  `NO_GROUND` again: send `/release`, set the dart right, then `/auto`. After a reset (travel unknown) only the travel
  of this swing counts. Geometry assumption, checked in BENCH_TEST step 5: on flat ground the kick's housing lands
  before 80 % of its swing in air (90 % is the line, 10 % margin for the travel estimate). Known limit: a kick driven out by hand in air until it stalls, then `/auto`, is
  taken as planted.
- Kick travel is also marked unknown (event `kick travel unknown`) when the kick is driven by hand together with any
  other motor, until the next completed release.
- In the swing step a stall with no contact phase is judged: planted since the last release: already down; at 90 % of
  `kickSwingMs` or more: `NO_GROUND`; near stowed, or after moving over 10 % of the swing in this step: `KICK_JAM` (a
  jam in air, or hard ground with no contact phase); otherwise already down. A "contact" at 90 % or more (a soft end
  stop in air) is also `NO_GROUND`. After `NO_GROUND` the kick's position is its end stop and it does not count as
  planted until it has come back under half way, so a retry or a `2` tap does not make the next `/auto` anchor in air.
- One failed current read at the moment a step ends does not fault: the step ends and the next good read moves on.

### GET /release

Starts the release sequence, the anchor sequence backwards. v4.1: drills that stall while unscrewing are **rocked** (in
0.3 s, then out again, up to 5 times, event `drills stuck, rocking them`) before `DRILL_JAM`; pods that stall on the
way up before 60 % of their way back are **milled** up (deploy paused 0.2 s while the drills unscrew, then up again,
event `pods held in soil, milling up`) and only give `PODS_JAM` after 5 s without progress.

1. drills unscrew alone for 3 s (`release: drills out`)
2. pods lift while the drills keep unscrewing for 0.6 s more, then the pods go on alone until they stall on their
   stowed stop (after 6 s it carries on, event `release: pods time limit (stowed or stalled), foot out`). Alone, the
   pods are read every 5 ms, so a stall before they came 60 % of their estimated way back is `PODS_JAM` (fault), not
   the stowed stop. A jam in the last 40 % is not caught (it looks like the stowed stop).
3. foot unscrews alone for 1.5 s (`release: foot out`)
4. kick swings back while the foot keeps unscrewing for 3 s more, until the kick stalls on its stowed stop (15 s limit)
5. `seq` = `released`, phase `IDLE`, all motors off.

C2 listed the parts as "drills out, foot out, kick back, deploy up". The firmware runs them in the exact reverse order
of `/auto` (pods up before the kick comes back), because the kick goes down first and the pods last.

- Refused in `FLIGHT`, while anchoring (`busy`, send `/stop` first), and without the INA219.
- A repeated `/release` while releasing is accepted and does not restart it.
- Faults: `DRILL_JAM`, `FOOT_JAM`, `KICK_JAM` (kick stalled before 60 % of the way back), `NO_CURRENT`, `KICK_TIMEOUT`
  (15 s), `SEQ_TIMEOUT` (180 s), plus `OVERCURRENT`, `LOW_RAIL`, `CURRENT_SENSOR`. All motors stop on any fault.

### GET /arm?on=1&auto=1

- `on=1` (default): arm the landing detector. `auto=1` (default) starts `/auto` by itself on landing; `auto=0` only
  records the flight log. Re-arming resets the flight detector, `landed`, and unfreezes the log.
- `on=0`: disarm. Always accepted, also in FLIGHT (then no auto on landing; the log still freezes after landing).
- Arming clears a latched fault. Key `A` toggles: when disarmed it arms with `auto=1`.
- Refused while `ANCHORED` (`anchored: release first`), and whenever the kick or the pods are known to be more than
  10 % out (`legs out: release first`, for example after an aborted anchor or a hand move), without the BNO085 (`no IMU`), with `auto=1` and no INA219, while a sequence runs, in `FLIGHT`, and with
  `auto=1` when the rail is under 6.6 V at rest, including 0 V with the plug out (`battery low`; a log-only arm is
  still allowed).
- An arm expires after 15 min without a launch (event `arm expired`).
- Idempotent: use this instead of the `A` toggle. Arm right before the shot, because `/stop` disarms.
- Arming after `aborted` or `released` starts a new shot (`seq` back to `idle`). From `ANCHORED` it is refused (above).

### GET /video?on=1 | on=0

v4.1. Switches the camera and VTX power (the optional MOSFET on D10, WIRING.md). Always 200. `video` in `/status` shows
the wanted state. The dart also switches the video on at boot and when armed, and off when the battery goes critical.
Without the switch fitted the call does nothing physical (the video stays on). 400 without `on`.

### GET /flightlog

The BNO085 log, streamed as JSON (chunked, up to about 40 kB). The server is busy for about 0.3 s while it sends,
so it is refused while any motor runs (409 `busy (motors running)`: retry, auto-anchor may be running), while armed
(`armed (disarm first)`) and in FLIGHT (`in flight`).
A client that stops reading is cut off after 8 s.
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
| `FAULT` | a fault code is set (latched until `/stop`, `/auto`, `/release` or `/arm`). A fault also disarms, so a landing can never clear it and start motors by itself |
| `RELEASING` | release sequence running |
| `ANCHORING` | anchor sequence running |
| `FLIGHT` | launch detected, landing not yet decided |
| `ANCHORED` | anchor sequence finished (`seq` `anchored`) |
| `LANDED` | landed and no motor running (log-only arm, fallback landing, auto refused, or auto stopped by hand), until a release completes or a key moves a leg; a manual motor running after a landing shows `IDLE` |
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
| `SEQ_TIMEOUT` | anchor or release sequence over 180 s (v4.1; was 35 s) | all off |
| `NO_CURRENT` | a lone motor in a sequence drew under 40 % of its free current for 0.5 s: open wire or shorted output | all off |
| `DRILL_JAM`, `FOOT_JAM` | stalled while unscrewing in release | all off |
| `CURRENT_SENSOR` | 3 failed INA219 reads in a row during a sequence | all off; manual keys still work on timers |
| `LOW_RAIL` | rail under 5.6 V (including a dead 0 V rail) for 0.5 s while a motor runs | all off |
| `KICK_JAM` | kick stalled near stowed at the start of `/auto`, or part way out with no contact (jam in air), or in `/release` before its travel back covered 60 % of its travel out | all off |
| `PODS_JAM` | in `/release` the pods stalled before they came 60 % of their estimated way back (jammed part way up) | all off |
| `NO_GROUND` | in `/auto` the kick swung (almost) its whole travel and stalled at its own end stop without touching ground (dart on its back or side) | all off; `/release` stows the kick |
| `NOT_UPRIGHT` | v4.1: still on its side, upside down or nose steep after the levelling step | all off; the pods are about 20 deg out: `/release` |
| `BROWNOUT` | the dart rebooted from a brownout | booted with motors off |
| `WATCHDOG` | the dart rebooted from a crash or the 2 s safety-task watchdog | booted with motors off |

## Flight detection (decision record)

- v4.1: the flight windows below are shorter than in v4 (push-over 400 -> 200 ms, first impact 400 -> 250 ms after
  launch): the real launcher flies 0.37 s at its 6 deg low aim, about 0.42 s from the start of the push to the impact.
- Launch: \|a\| over 4 g continuously for 20 ms, **only while armed**, and not within 400 ms after free fall (\|a\|
  under 0.5 g for 40 ms or more). A dropped dart is in free fall before it hits, so a drop is never taken as a launch;
  rail vibration, tilts, the cocking jerk and the push's own ramp never read that low, so they do not matter. Launch
  time = start of those 20 ms.
  In practice the samples over 4 g must span at least 15 ms (one 5 ms sample period of slack): pushes of 24 ms or more
  are always seen (18 ms with steady 5 ms ticks; measured by the fuzz test with 4 to 7 ms tick jitter), knocks of 10 ms
  or less never, 15 to 23 ms sometimes. A launch alone never starts motors.
- Landing: once the dart has been in free fall (\|a\| under 0.6 g for 60 ms in total, or, for a tumbling dart that
  never reads that low, 200 ms after launch with \|a\| back under 4 g),
  an impact over 3.5 g at least 250 ms after launch (the shortest real flight is about 0.42 s from the push; this keeps
  the push and the line snatch in the stroke from counting as a landing), then \|a\| between 0.85 and 1.15 g for 1 s. Each new impact restarts the stillness timer;
  landing time = the last impact. The free-fall rule stops a bump on the bench from counting as a flight.
- **Impact level 3.5 g, not the 6 g in C2.** The BNO085 clips at 8 g and low-pass filters its output, so a 2 ms
  impact can read only about 5.7 g; free flight reads under about 1.5 g even tumbling, so 3.5 g is a safe line.
  `peak_g` in `/status` shows what a real landing read; tune `impactG` from drop tests.
- Landing on its side, upside down or nose steep (`orient`): auto on landing does not start (event `landed on its
  side: auto not started`, and so on); the phase is `LANDED`.
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

## Changes from v4 to v4.1 (7 Nov 2026, for client authors)

- New status fields `video`, `batt`, `batt_V`, `orient`, `tilt_deg`, `roll_g`, `pitch_g`, `soil_mA`, `moving`; new
  `seq` `levelling: pods to 20 deg, drills off`; new fault `NOT_UPRIGHT`; new endpoint `/video?on=`; `/auto?force=1`.
- New `/auto` refusals `not upright: ...` and `battery low: ...` (409). Treat `not upright` like `NO_GROUND`: retrieve
  and fire again (or `force=1` if the video shows the dart upright).
- Anchoring can take up to 180 s (milling deploy up to 150 s, loam about 90 s): wait 200 s for `ANCHORED`.
- Release can rock the drills and mill the pods up (limit 180 s).
- New refusal `motors hot after a long milling: wait 60 s` (after a pods step over 60 s).
- Flight detection: push-over 200 ms, first impact 250 ms after launch (was 400 / 400).
- `fw` is `kedge-dart-v4.1`: compare with `startswith("kedge-dart-v4")`.

## Changes from v3 (for client authors)

- New: `/release`, `/arm`, `/flightlog`, `/stop?disarm=0`, and the status fields `fw`, `phase`, `fault`,
  `auto_on_landing`, `accel_g`, `peak_g`, `warnings`, `ina_ok`, `imu_ok`, `t_launch_ms`, `t_land_ms`, `log_frozen`, `reset`.
- Refusals are now HTTP 409 (v3 silently ignored keys during auto).
- `/stop` disarms. `armed` means "landing detector armed"; the auto choice is `auto_on_landing`.
- `/key` `ms` is capped at 500 (was 5000); a hold is capped at 8 s. A held key that stalls stays off until released.
- The release used to be held keys from the Jetson (`#` + `!`, then `$` + `@`). Use `/release` and wait for
  `seq == "released"`; fail on `FAULT` or `aborted`.
