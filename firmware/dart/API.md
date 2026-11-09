# KEDGE dart HTTP API (contract C2, firmware kedge-dart-v4.3, dart v34)

v4.3 in one paragraph (details in "Changes from v4.2 to v4.3" at the end): the v34 dart has no foot and no kick leg.
The kick channel drives the righting leg (`motors.kick`, keys `2` / `@`), `motors.foot` is always 0 and keys `4` / `$`
are refused (`no foot on v34`). `/auto` rights the dart first when it is not belly down (`seq` `righting: ...`, still
phase `ANCHORING`), then runs the pods down with the drills; it no longer refuses `not upright`. A dart it cannot right
in 6 moves ends in fault `NOT_UPRIGHT`.

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
v4.1 adds, for `/auto`: `motors hot after a long milling: wait 60 s` and `battery low: charge it (release still works)`
(the v4.1 / v4.2 `not upright: ...` refusals are gone in v4.3: the dart rights itself). v4.3 adds `no foot on v34` (keys `4` / `$`).

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
{"fw":"kedge-dart-v4.3","t_ms":5535,"phase":"ANCHORING","fault":null,"rail_V":7.60,"current_mA":24,"peak_mA":24,
 "speed":255,"motors":{"deploy":1,"kick":0,"drills":0,"foot":0},"seq":"deploy + drills","armed":false,
 "auto_on_landing":true,"landed":true,"roll":1.5,"pitch":-3.2,"accel_g":1.01,"peak_g":8.00,"ina_ok":true,"imu_ok":true,
 "t_launch_ms":3000,"t_land_ms":4010,"log_frozen":false,"reset":"poweron","warnings":[],"video":true,"batt":"ok",
 "batt_V":7.60,"orient":"upright","tilt_deg":0,"roll_g":0,"pitch_g":0,"soil_mA":-1,"moving":false,"event":"auto: upright (tilt 0 deg), deploy with drills spinning"}
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
| `motors` | object | `deploy`, `kick`, `drills`, `foot`: -1, 0 or +1 (+1 = out / in / down, -1 = back / up / unscrew). v4.3 (v34): `kick` is the righting leg (+1 = out over the nose, -1 = fold), `foot` is always 0 |
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
| `warnings` | array of strings | what went less than perfectly in the last `/auto` or `/release`, empty when clean: `drills_stalled` (stone or full depth), `pods_time_limit` (the pods step ended on a time limit: firm soil, or no stall seen at the stop), `depth_limited` (v4.2: the deploy guard stopped the pods part way, firm soil or a stone). `ANCHORED` with warnings may be a weak anchor. (`foot_stalled`, `kick_plant_time_limit`, `kick_already_down` belong to v4.2 and are never set on v34) |
| `video` | bool | v4.1: camera + VTX power wanted on (the optional switch on D10; without it the video is always on) |
| `batt` | string | v4.1: battery at rest (no motor for 1 s): `ok`, `low` (under 6.8 V: about 10 % left), `critical` (under 6.4 V: `/auto` refused, video switched off, `/release` and keys still work), `none` (rail under 3 V: plug out), `unknown` (no reading yet). 0.1 V hysteresis |
| `batt_V` | number | v4.1: the resting battery voltage behind `batt` (-1 before the first reading) |
| `orient` | string | v4.1: from the BNO085 accelerometer while still: `upright`, `leaning` (roll 10 to 60 deg), `side`, `inverted`, `nose_down`, `nose_up`, `unknown` (IMU silent or moving). Until the nose axis is set (`imuNoseAxis`, BENCH_TEST step 7) only `upright` / `side` / `inverted` from the tilt (70 / 120 deg). Report only: the v4.3 righting decides on `tilt_deg` |
| `tilt_deg` | number | v4.1: angle between the dart's up axis and straight up (0 when `orient` is unknown). v4.3: `/auto` rights the dart first when this is 15 or more |
| `roll_g`, `pitch_g` | number | v4.1: body roll and nose pitch from gravity (0 until the nose axis is set) |
| `soil_mA` | number | v4.1: highest drill-pair current read alone while the augers were in soil in the last `/auto` (at 8.0 V; -1 = none). dart-physics: about 160 sand, 640 loam (7.4 V) |
| `moving` | bool | v4.1: while `ANCHORED`: the dart tilted over 2 deg from where it anchored, or \|a\| off 1 g by 0.15 g, for 200 ms (the anchor is moving under the tow: stop the tow). Event `anchor moving` on each start |
| `event` | string | last event line |

`seq` values (v4.3): `idle`, `righting: leg out`, `righting: leg folding`, `righting: pods out, drills off`,
`righting: pods back up`, `righting: settling` (match on `righting`), `deploy + drills`, `drills finishing`, `anchored`,
`aborted`, `release: drills out`, `release: drills out + pods up`, `release: leg folding` (only when the leg was not
known folded), `released`. The v4.2 names `kick swing`, `kick + foot planting`, `foot finishing`, `release: foot out`,
`release: kick back + foot out` and `levelling: ...` are gone. **Prefer `phase` over `seq` text.**

### GET /key?c=&lt;key&gt;&ms=&lt;run ms&gt;

Manual key, same keys as the web page and Serial Monitor.

- Run keys: `1` / `!` pods down / up (the drills swing down / back), `2` / `@` righting leg out over the nose / fold
  (v4.3; the old kick channel), `3` / `#` drills in / out. `4` / `$` are refused `no foot on v34` (nothing changes).
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

v4.3 (dart v34): righting first when the dart is not belly down, then the anchor: pods down with the drills spinning,
drills finishing, `anchored`. The righting is the policy of the study sim (kedge/sim_righting simCore; the v34 check
righted loam 40/40, sand 20/20) on v34's hardware, where one deploy worm drives both pods:

1. If the leg is not known to be folded (moved by hand, or after a reset other than a power-on) it first goes back to
   its fold stop (`auto: leg to its fold stop first`, `seq` `righting: leg folding`). The pods never deploy with the leg out.
2. The tilt is read (BNO085 gravity while still; `tilt_deg`). Under 15 deg: belly down, no righting (`auto: upright
   (tilt N deg), deploy with drills spinning`). No attitude (BNO085 silent) or `/auto?force=1`: no righting, anchored as
   it lies (`auto: attitude unknown, ...`, `auto: forced, no righting`).
3. Otherwise one righting move (event `righting N: ...`), then 0.6 s still (up to 4 s for a fresh reading) and step 2
   again, where after a move a tilt under 20 deg counts as upright (the pods hold it; they level it further as they go
   down with the drills):
   - tilt over 110 deg (belly up): **leg flip**. The leg swings out over the nose and pushes the dart over end to end
     until the tilt falls under 60 deg (`righting: going over`), then 0.6 s more; or until it stalls for 1.5 s (its far
     stop, the bed in the nose top, reached in air once the dart is over: normal, not a fault; or a dart it cannot lift);
     at most 9 s. Then the leg folds to its stop (`righting: leg folded`; KICK_JAM if it stalls before 60 % of the way
     back, KICK_TIMEOUT after 15 s).
   - otherwise (on a flank, leaning on a pod): **pods roll**. Both pods swing down with the drills OFF until the tilt is
     under 10 deg (the low pod rolls it onto its belly) or they stall for 1 s, at most 10 s, and hold there. If that
     righted it, the anchor goes on from where they are; if not, they fold back to their stop first (`righting: pods
     back up first`). Every 4th flank move is a leg flip (the sim's list).
   - after 6 moves still not upright: fault `NOT_UPRIGHT` (`auto: not upright after 6 righting moves (tilt N deg)`), all
     motors off, leg and pods folded. The rover reels it in and fires again (or the operator forces it).
   Righting runs one motor at a time; the drills never turn while righting. The whole anchor is limited to 240 s.
4. The anchor itself is v4.2's: the pods go down with the drills spinning (milling in soil, deploy guard), drills
   finishing 3 s, `anchored`. The leg stays folded.

v4.1 and v4.2 notes that still hold for the anchor step:
- **Battery critical** (`batt` `critical`): refused (`battery low: ...`), so the dart never anchors without the charge to
  let go. `/release` always runs.
- **Milling deploy.** The augers sweep sideways through soil, so the deploy worm stalls in soil long before its stop. In
  the pods step, a deploy stall before 85 % of the estimated pods travel (`podsTravelMs`; the estimate is the travel up
  to the stall reading, at most one 0.55 s cycle long) is soil: the deploy pauses 0.2 s while the drills keep cutting,
  then pushes again (event `pods in soil, milling`). No estimated travel gained for 5 s ends the step: taken as the stop
  when past 60 % (`pods down (no progress, past 60 %: the stop)`), else warning `pods_time_limit` (firm soil). The
  pods step is limited to 6 s until milling starts, 150 s with milling (loam takes about 90 s). A pods step that never
  sees a stall (play or compliance at the stop) ends on these limits: `ANCHORED` with warning `pods_time_limit`, never a
  hang. After a pods step over 60 s the next `/auto` is refused for 60 s (`motors hot after a long milling: wait 60 s`).
  With the travel unknown (after a reset or hand moves with other motors) the first stall is the stop, as in v4.
- **Deploy guard (v4.2, dart-physics v32).** When the drill pair, read alone (in the worm's pause, two readings in a row,
  or for 0.2 s during a milling pause), is over 67 % of its real stall current (`drillPairStallMa`, 700 mA at 8.0 V,
  measured in BENCH_TEST step 4), the pods stop where they are, the drills stop, and the anchor ends `ANCHORED` with
  warning `depth_limited` (event `drills at 67 % of stall: depth limited, anchored`).
- Known limit: a soil stall past about 85 % of the estimated travel (less up to one cycle, so possibly from about 50 %)
  is taken as the stop (weak anchor; a pods-down hall sensor would fix it, see the notes).
- Known limit: pods already stuck in soil at the first readings of the step, with read noise near the margin, show no
  rise and may not read as stalled; the 5 s no-progress rule ends that (6 s when no milling started).

- Refused in `FLIGHT` (decision below), while releasing (`busy`), when already anchored, and without the INA219.
- While `ARMED` (the launch was never seen) it disarms first (event `auto: disarmed first`) and starts.
- A repeated `/auto` while anchoring is accepted and does not restart it (safe to retry after a timeout).
- Accepted from `IDLE`, `ARMED`, `LANDED` and `FAULT` (it clears the fault).
- Timings: righting moves as above (leg out at most 9 s, leg fold 15 s then `KICK_TIMEOUT`, pods roll at most 10 s,
  pods back 6 s); pods down until they stall (6 s, 150 s once milling started, then it carries on, no fault); drills
  finishing 3 s. Whole sequence limit 240 s (v4.3: room for 6 righting moves before the 150 s milling deploy).
- A single motor that draws no current faults the sequence (`NO_CURRENT`): the leg in a flip, the pods in a roll, and
  the deploy worm alone for 0.3 s at the start of the pods step. In the release the drills keep unscrewing from the
  start of the pods lift (the augers may still be in soil).
- Leg travel bookkeeping: the firmware adds up how long the leg has been driven out and back (at full duty, scaled by
  rail voltage / 8.0 V) and assumes it was **folded at power-on**. `kickSwingMs` (3.8 s) is its swing 0 to 178 deg at
  8.0 V; the firmware measures it: the leg driven out by hand from folded until it stalls logs `leg travel from folded
  NNNN ms (at 8.0 V)` (BENCH_TEST step 5). Within 2 % of the swing of the fold stop counts as folded. After any reset
  other than a power-on, or when the leg was driven by hand together with other motors, its position is unknown until
  it next folds to its stop (the next `/auto` does that first).
- Stall handling while two motors run (pods + drills): the deploy worm pauses 150 ms every 550 ms. It self-locks and
  holds position while the drills keep turning and are read alone; the worm's current is (both) - (drills alone). A
  worm stall must also show as a rise of the total current over the phase's lowest reading, so a drill current that
  changes during the pause cannot fake one. A stalled drill pair is stopped (anchor; not driven again in the next step)
  or faults (release). An end-stop stall is only believed after a minimum travel time (pods 0.7 s, leg fold in the
  release 1.5 s). Known limit: if the pods jam in soil mid-travel and the drills' current rises during each pause by
  more than the margin, the jam is only ended by the pods step limits.
- A landing knock or current spike while no motor runs is no fault: current checks run only while a motor is driven,
  and each start ignores 0.25 s of surge.
- One failed current read at the moment a step ends does not fault: the step ends and the next good read moves on.

### GET /release

Starts the release sequence: drills out, pods up, and (v4.3) the leg to its fold stop only if it is not known folded
(the anchor leaves it folded). v4.1: drills that stall while unscrewing are **rocked** (in
0.3 s, then out again, up to 5 times, event `drills stuck, rocking them`) before `DRILL_JAM`; pods that stall on the
way up before 60 % of their way back are **milled** up (deploy paused 0.2 s while the drills unscrew, then up again,
event `pods held in soil, milling up`) and only give `PODS_JAM` after 5 s without progress.

1. drills unscrew alone for 3 s (`release: drills out`)
2. pods lift while the drills keep unscrewing for 0.6 s more, then the pods go on alone until they stall on their
   stowed stop (event `release: pods up`; on the time limit `release: pods time limit (stowed or stalled)`). Alone, the
   pods are read every 5 ms, so a stall before they came 60 % of their estimated way back is milled up, then
   `PODS_JAM` (fault), not the stowed stop. A jam in the last 40 % is not caught (it looks like the stowed stop).
3. v4.3: only if the leg is not known folded: the leg folds until it stalls on its stop (`release: leg folding`, 15 s
   limit; `KICK_JAM` if it stalls before 60 % of its estimated way back).
4. `seq` = `released`, phase `IDLE`, all motors off.

- Refused in `FLIGHT`, while anchoring (`busy`, send `/stop` first), and without the INA219.
- A repeated `/release` while releasing is accepted and does not restart it.
- Faults: `DRILL_JAM`, `PODS_JAM`, `KICK_JAM` (leg stalled before 60 % of the way back), `NO_CURRENT`, `KICK_TIMEOUT`
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
| `KICK_TIMEOUT` | v4.3: the righting leg did not reach its fold stop in 15 s (anchor or release), or a leg timer ran out while current reads were failing | all off |
| `SEQ_TIMEOUT` | anchor sequence over 240 s (v4.3), release over 180 s | all off |
| `NO_CURRENT` | a lone motor in a sequence drew under 40 % of its free current for 0.5 s: open wire or shorted output | all off |
| `DRILL_JAM` | stalled while unscrewing in release (`FOOT_JAM`: v4.2 only, never on v34) | all off |
| `CURRENT_SENSOR` | 3 failed INA219 reads in a row during a sequence | all off; manual keys still work on timers |
| `LOW_RAIL` | rail under 5.6 V (including a dead 0 V rail) for 0.5 s while a motor runs | all off |
| `KICK_JAM` | v4.3: the righting leg, folding (after a flip, or in `/release`), stalled before its travel back covered 60 % of its travel out | all off |
| `PODS_JAM` | in `/release` the pods stalled before they came 60 % of their estimated way back (jammed part way up) | all off |
| `NO_GROUND` | v4.2 only (the kick's end stop in air); never raised on v34, where the leg's far stop in air is normal | |
| `NOT_UPRIGHT` | v4.3: still not upright (tilt 20 deg or more) after 6 righting moves | all off, leg and pods folded: reel in and fire again |
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
- v4.3: landing on its side or back starts auto on landing as usual; the auto rights the dart first. (v4.1 / v4.2 did
  not start it.)
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

## Changes from v4.2 to v4.3 (9 Nov 2026, dart v34, for client authors)

- `fw` is `kedge-dart-v4.3`. No foot, no kick leg: `motors.kick` is the righting leg, `motors.foot` is always 0, keys
  `4` / `$` are refused `no foot on v34`.
- `/auto` rights the dart first when `tilt_deg` is 15 or more (leg flip from its back, pods roll from a flank, up to 6
  moves; `seq` starts with `righting`, phase `ANCHORING`), then pods + drills as before. No more `not upright`
  refusals; auto on landing starts on its side or back too. `/auto?force=1` now means: no righting, anchor as it lies.
- New end: fault `NOT_UPRIGHT` after 6 moves (treat it like the old `not upright` refusal or `NO_GROUND`: reel in and
  fire again). `NO_GROUND`, `FOOT_JAM` and the warnings `foot_stalled`, `kick_plant_time_limit`, `kick_already_down`
  are never raised.
- Anchoring can take up to 240 s (righting, about 10 s a move, then the up to 150 s milling deploy): wait 260 s.
- The release no longer has foot or kick steps (drills out, pods up; the leg folds only if it was not folded).

## Changes from v4.1 to v4.2 (7 Nov 2026, for client authors)

- New warning `depth_limited` (deploy guard, see `/auto`). `fw` is `kedge-dart-v4.2`.
- `seq` levelling text now says 16 deg (v32): match on the word `levelling`.
- v32 drives each drill sleeve directly from its motor (no gear stage), so the drill channel is driven as before
  (`DRILL_SIGN` +1); `motors.drills` +1 means the sleeves screw in.

## Changes from v4 to v4.1 (7 Nov 2026, for client authors)

- New status fields `video`, `batt`, `batt_V`, `orient`, `tilt_deg`, `roll_g`, `pitch_g`, `soil_mA`, `moving`; new
  `seq` `levelling: pods to 16 deg, drills off`; new fault `NOT_UPRIGHT`; new endpoint `/video?on=`; `/auto?force=1`.
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
