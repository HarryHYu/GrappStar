# GrappStar base station protocol (contract C1)

Firmware `base-1.0` (`firmware/base/grappstar_base`), mechanism BASE_SPEC v1.0 (drum order LAUNCH / NEUTRAL / WRAP /
REEL, decision D16; sequence from panel lens B sections 7.1 to 7.4, decision D17). This file is the contract the Jetson
implements against. Changes are additive where possible (new fields, new events, new error codes); section 10 lists
everything that changed since the base-1.0 draft of this file.

Nothing here has run on real hardware. Every reply and event below comes from the code and the PC simulation
(`test/run_tests.sh`).

## 1. Link

- The ESP32-S3 DevKitC-1 native USB port (USB CDC, the port labelled USB). On the Jetson it shows up as `/dev/ttyACM*`.
  The baud rate is ignored.
- Newline-terminated JSON objects, UTF-8, no spaces added by the base. At most 256 bytes per line in both directions
  (the newline not counted). The base accepts `\n` or `\r` as the end of a line and ignores empty lines.
- Lines that do not start with `{` (ROM boot text after a reset) must be ignored by the Jetson. Resync on the next
  line that parses.
- Jetson to base: one command object per line. Base to Jetson: replies, telemetry and events, interleaved, each a
  whole line (the base never writes half a line).
- Output buffering: the reply to a command is written first (waits up to 1 s for USB buffer room), then any events.
  Events wait in a queue of 24 lines; when the queue is full the newest event is dropped. Telemetry is never queued:
  a telemetry line that does not fit the USB buffer is dropped. So read the port continuously.
- `HELLO` (section 4) marks a restart of the base. Anything pending on the old session failed.

## 2. Commands and replies

Every command line gets exactly one reply line, in order:

```
-> {"id":17,"cmd":"aim","deg":32.5}
<- {"id":17,"ok":true}
<- {"id":18,"ok":false,"err":"NOT_IN_LAUNCH"}
```

- `id`: echoed first. An integer (echoed as an integer, up to 2^53) or a string. Missing, `null` or a boolean gives
  `"id":null`. A number that is not an exact integer (`1.5`, `1e35`) is refused with
  `{"id":null,"ok":false,"err":"BAD_ID"}`. Strings are cut to 63 bytes; keep them under 24 bytes, because if the whole
  reply would pass 256 bytes it is rebuilt with only `id` (cut to 23), `ok`, `err` (and `token` for `arm`). If even
  that does not fit: `{"id":null,"ok":false,"err":"REPLY_OVERFLOW"}`. Use unique increasing integers.
- Field order in a reply: `id`, then any `val` / `min` / `max` from `get`, then `ok`, then `err` or the extra fields.
- A line that is not valid JSON: `{"id":null,"ok":false,"err":"BAD_JSON"}`. A line over 256 bytes:
  `{"id":null,"ok":false,"err":"TOO_LONG"}`. No `cmd`: `NO_CMD`. Unknown `cmd`: `UNKNOWN_CMD`.
- Keys are at most 15 characters, at most 12 keys per object are read (more are ignored), a repeated key takes the
  last value, unknown keys are ignored, nested objects or arrays as values are skipped.
- Long actions (`mode`, `aim`, `cock`, `fire`, `wrap`, `unwrap`, `reel`, `cal decock`, `estop`) reply `ok:true` as
  soon as they START. Completion is an event (section 4). `jog` ends silently when its time runs out (watch `state`).
  A refusal replies `ok:false` with `err` and nothing moves.
- "Already there" cases reply `ok:true` and send the completion event at once (listed per command).
- Before any motor starts, the base writes a "moving" record to flash (a few tens of ms): motion begins slightly after
  the reply.
- While an action runs, every other action is refused with `BUSY`. `ping`, `status`, `get`, `set`, `stop`, `estop`,
  `clear` and most `cal` actions always answer.

### Common refusals

Checked first by every action that moves something (in this order):

| err | meaning |
|---|---|
| `FAULTED` | a fault is latched (or the drum is still on its way to NEUTRAL after one): `clear` first |
| `BUSY` | another action runs |
| `BOOTING` | the first 60 ms after start-up |
| `NO_POWER` | battery under 6 V (unplugged, USB only). Only while `vbat_wired` is true |
| `ANGLE_SENSOR` | the AS5600 is not reading. Only for actions that turn the wiper (`aim`, `cock`, `arm`, `fire`, `reel`, `unwrap`, wiper `jog`, `cal decock`) |
| `HOLDING` | the reel is holding a tension (state `HOLD`): `stop` first. `reel` itself is allowed and winds again |

### Command list

| cmd | args | does | refusals (besides the common ones) |
|---|---|---|---|
| `ping` | | reply carries `fw` and `t` (ms since boot) | none |
| `status` | | reply carries `phase` (section 3), `aimed`, `aim_known`, `drum_known`, `wraps_known`, `magnet` (`OK`, `WEAK`, `STRONG`, `NONE`), `glitches` (AS5600 rejected samples, capped at 99999), `hx` (load cell live), `peak_a` (highest wiper current of the last run), `fw`, and while a fault is latched `why` (first 32 characters of the fault reason). These are flat fields of the reply, not a `tel` object | none |
| `stop` | | wiper off now, drum stops where it is (both worms hold). Aborts the running action with an `ABORTED` event. A stop during `cock`, `fire` or `cal decock` makes the shot phase `UNKNOWN`. A stop while faulted (also during `SAFING`) stops the drum and stays in `FAULT` | never refused |
| `estop` | | like `stop`, then (when not faulted) the drum goes to NEUTRAL if that is safe: state `MODE`, a `MODE` event at the end, immediately if already at NEUTRAL. If it is not safe the drum holds and a `MODE` event with `ok:false`, `err:"DRUM_HELD"` and `why` says why (section 6). The estop move never turns the wiper. A failed estop move raises a drum fault | never refused |
| `clear` | | clears a latched fault, back to `IDLE`. Not faulted: `ok`, no change | `BUSY` (still `SAFING`), `STILL_PRESENT` (the cause is still there: AS5600 not reading after `ANGLE_SENSOR`, drum code invalid after `DRUM_SENSOR`, battery under `vbat_low` after `BROWNOUT`) |
| `mode` | `to`: `LAUNCH` \| `NEUTRAL` \| `REEL` | moves the drum to that stop point (LAUNCH 0, NEUTRAL 41, REEL 347 drum deg) and verifies it (section 5). Leaving LAUNCH starts with the centering jog. `mode REEL` only engages REEL from the lock sector after a wrap (normally use `wrap`). Already there (sector, angle within 3 deg, and for LAUNCH the collar confirmed): `ok` and an immediate `MODE` event. Ends with a `MODE` event. LAUNCH to NEUTRAL about 2.5 s, NEUTRAL to LAUNCH about 2 s | `BAD_ARG`, `POSITION_UNKNOWN` (drum code invalid), `HAND_MODE` (`SW_N20` open: hand lever latched out), `NOT_WRAPPED` (`REEL` asked without the wraps on: use `wrap`), `WRAPS_ON` (`LAUNCH` or `NEUTRAL` asked from the WRAP sector, lock sector or REEL: use `unwrap`), `LAUNCH_BUSY` (leaving LAUNCH needs the launcher at rest, section 6) |
| `aim` | `deg`: rail pitch | turns the wiper BACKWARD until the pitch (from the aim table) is within `aim_tol` of `deg`. If the aim is unknown (always after a shift into LAUNCH, after a power cut, after `cal decock`) it first runs backward until `SW_RAIL_LOW` closes (home), then to the target: up to about 1.25 aim cycles (10 wiper turns per cycle), so allow 60 s. Up to 3 tries. `AIMED` event | `NOT_IN_LAUNCH` (collar not confirmed in LAUNCH, or the drum not stopped within 6 deg of the LAUNCH stop point), `PHASE_UNKNOWN`, `NOT_AT_REST` (aim only with the launcher at rest: backward drive with the spring drawn would let it unwind), `OUT_OF_RANGE` (see `get pitch_range`), `AIM_UNKNOWN` (aim unknown and no `SW_RAIL_LOW` fitted) |
| `cock` | | optional. Turns the wiper FORWARD from rest to the cocked angle (carriage drawn, short of the release tooth). `SW_HOME` must open at the first tooth. `COCKED` event. With the phase `UNKNOWN` and the carriage home it first creeps until `SW_HOME` opens (that re-zeroes the angle), then cocks. Already cocked (`SW_HOME` open): `ok` and an immediate `COCKED` event. Aim first: see the note under section 8 | `NOT_IN_LAUNCH`, `NO_SENSOR` (no `SW_HOME` fitted), `SWITCH_DISAGREE` (phase `COCKED` but `SW_HOME` closed, or `REST` but `SW_HOME` open), `LOADED_UNKNOWN` (carriage drawn with the angle unknown, or `SW_HOME` closed while the last known angle says drawn: `cal decock` first) |
| `arm` | | checks everything `fire` needs and replies `"token":<6-digit integer>,"valid_ms":3000`. A new `arm` replaces the old token. `mode`, `aim`, `cock`, `jog`, `cal decock`, `stop`, `estop` and any fault cancel it | `NOT_IN_LAUNCH`, `WRAPS_ON` (wraps not 0 or unknown), `NO_SENSOR` (no `SW_HOME`, or no `SW_SEATED` while `require_seated`), `NOT_COCKED` (neither `COCKED` with `SW_HOME` open and `SW_GAP` open, nor `REST` with `SW_HOME` closed and `SW_GAP` closed), `NOT_AIMED` (not aimed since the last shift, shot or aim-train move, or the pitch is more than `aim_tol` off the target), `NOT_SEATED` |
| `fire` | `token`: the integer from `arm` (a numeric string is accepted) | checks the token, then everything again, then runs the wiper FORWARD: from `COCKED` straight past the release tooth (about 1 s to `RESET`), from `REST` the whole cycle in one run (draw, release, reset: about 3 s). `FIRED` when `SW_HOME` closes (carriage released), `RESET` when the big gear is back in its gap with `SW_HOME` and `SW_GAP` closed. The token is single use; a wrong token cancels the arm | `BAD_ARG` (no `token`), `NOT_ARMED`, `TOKEN_EXPIRED` (over 3 s), `BAD_TOKEN`, then the common refusals and the `arm` refusals |
| `wrap` | | from NEUTRAL (or anywhere between NEUTRAL and REEL): drum forward through the WRAP sector, the cup lays 7 turns on the capstan with the collar in NEUTRAL and the wiper off, then into REEL with the fly-in (wiper creeping forward at 20 % until `SW_DOG_R` closes). `SW_ARM` must pulse 7 times. `WRAPPED` event, then `MODE` (REEL). About 15 s: use a timeout of 60 s or more. Already in REEL with the collar confirmed: `ok` and an immediate `WRAPPED` | `POSITION_UNKNOWN`, `NOT_IN_NEUTRAL` (drum in LAUNCH or the LAUNCH to NEUTRAL ramp), `HAND_MODE`, `LAUNCH_BUSY` (spring known drawn) |
| `unwrap` | | from REEL (or anywhere in the wrap sectors): centering jog (forward until the dog window is taken up, then 30 deg back), drum backward to NEUTRAL; while the drum is in the WRAP sector the wiper turns forward at 40 % (take-up only: the capstan is not driven, the take-up winds the freed line). `SW_ARM` must pulse 7 times. `UNWRAPPED` event, then `MODE` (NEUTRAL). About 16 s. Safe at any wrap count (also after a power cut mid-wrap). Already at NEUTRAL: `ok` and an immediate `UNWRAPPED` | `POSITION_UNKNOWN`, `NOT_WRAPPED` (drum in LAUNCH or the ramp), `ANGLE_SENSOR` (the take-up needs the wiper), `HAND_MODE`, `LAUNCH_BUSY` |
| `reel` | `duty` over 0 up to 1 (default 0.86; below 0.27 it runs at 0.27), `max_n` (default `limit_n`, capped at 110), `until`: `stop` (default) \| `seated` \| `tension` \| `slip` | REEL (collar confirmed, wraps on): capstan winds in (tow, retrieve), see below. NEUTRAL (sector NEUTRAL, both dog contacts open): take-up only, see below. Also accepted while holding (state `HOLD`): winds again | `BAD_ARG` (bad `until`, `duty` not in (0, 1], `max_n` not over 0, `slip` in REEL), `NOT_WRAPPED` (neither REEL nor NEUTRAL, or `tension` in NEUTRAL: no tension without the capstan), `NO_SENSOR` (`seated` without `SW_SEATED`) |
| `jog` | `motor`: `wiper` \| `drum`, `dir`: 1 \| -1, `ms` (default 300, capped at 1500) | runs one motor for `ms`. Deadman: the same jog (same motor and dir) sent again before it ends extends it. Wiper dir 1 = FORWARD, drum dir 1 = toward REEL. Ends silently (state back to `IDLE`), see below for the limits | see below |
| `cal` | `what`, see section 7 | calibration actions | per action (section 7), `BAD_ARG` for an unknown `what` |
| `set` | `key`, `val` (number or `true` / `false`) | sets a value (section 7) | `BAD_ARG` (no key or val), `BAD_KEY`, `OUT_OF_RANGE`, for `enc_sign`: `BAD_ARG` unless 1 or -1, `BUSY` while moving |
| `get` | `key` | reply carries `val` (or `min` and `max` for `pitch_range`) | `BAD_ARG` (no key), `BAD_KEY` |

#### `reel` in detail

In REEL (capstan, wiper FORWARD, trip `trip_a`):

- Tension = load cell when one is live, else the estimate `(amps - free_a) x k_n_per_a` (the first 350 ms of a run are
  ignored). The speed tapers over the last 30 % of `max_n`.
- Reaching `max_n` stops the wiper and sends `LIMIT`. With `until:"tension"` the base then HOLDS (state `HOLD`,
  `holding:true`, the worm and sprag #2 hold the line); with a load cell it winds again by itself when the tension
  falls under 60 % of `max_n`. With any other `until` it goes to `IDLE` (`holding:false`).
- `until:"seated"`: stops when `SW_SEATED` closes (`SEATED` event, `line_m` set to 0); slow (duty 0.43) over the last
  1 m of `line_m`. Already seated: `ok` and an immediate `SEATED`.
- `until:"stop"`: runs until `stop` (or the limit). The Jetson stops it at about 1.5 m from the line count or the
  camera.
- After 240 s any reel stops with `ABORTED` (`what:"REEL"`, `why:"timeout"`).
- An over-current or stall during a tow is a fault, never a hold.

In NEUTRAL (take-up only, decision D15 / lens B I4: wiper FORWARD at most 40 % duty, trip 2.5 A; the capstan is not
connected, so there is no tension limit and `line_m` does not change):

- `until:"seated"`: the final approach. Stops at `SW_SEATED` (`SEATED`). No `SW_SEATED` within 20 s: fault
  `DART_STUCK`.
- `until:"slip"`: the tidy before a wrap. Runs until the current has been steady (within 0.3 A) for 2 s, which means
  the take-up clutch slips on a straight line: `LIMIT` with `tension_n:null`, `why:"tidy done (take-up slipping)"`,
  `holding:false`. Not done within 15 s: `ABORTED` (`what:"REEL"`, `why:"timeout"`).
- `until:"stop"`: runs until `stop` (240 s limit).
- `until:"tension"` is refused (`NOT_WRAPPED`).

#### `jog` in detail

Wiper jog (about 60 % duty; forward in NEUTRAL, wrap prep or the WRAP sector is take-up only: at most 40 % duty,
trip 2.5 A; forward in LAUNCH uses the cock trip 6.5 A; every other jog trips at 3 A):

| err | when |
|---|---|
| `BUSY` | the drum is running |
| `SHIFTING` | the drum is in the LAUNCH to NEUTRAL ramp or the lock sector (dogs half engaged), or its code is invalid |
| `SWITCH_DISAGREE` | both dog contacts closed, or `SW_DOG_L` closed while the drum code says NEUTRAL or the wrap sectors |
| `NOT_IN_LAUNCH` | drum in the LAUNCH sector (or `SW_DOG_L` closed) but not at the stop point with the collar confirmed |
| `LAUNCH_BUSY` | the spring is known drawn but the drum code is not LAUNCH |
| `WOULD_FIRE` | forward in LAUNCH with the phase `COCKED`, `UNKNOWN`, `FIRING` or `RESETTING` |
| `NO_SENSOR` | forward in LAUNCH without `SW_HOME` fitted |
| `NOT_AT_REST` | backward in LAUNCH with the phase not `REST` (the drawn spring would unwind: use `cal decock`) |
| `BACKSTOP` | backward in REEL (or with `SW_DOG_R` closed): sprag #2 holds the capstan |

A forward jog in LAUNCH from rest draws the carriage (phase `COCKING` once `SW_HOME` opens) and stops itself short of
the release tooth with `ABORTED` (`what:"JOG"`, `why:"would pass the release tooth"`). A backward jog in LAUNCH turns
the aim train and clears `aimed`.

Drum jog (full speed, no centering jog, no checks of the target):

| err | when |
|---|---|
| `BUSY` | the wiper is running |
| `POSITION_UNKNOWN` | drum code invalid |
| `HAND_MODE` | `SW_N20` open |
| `LAUNCH_BUSY` | drum in LAUNCH or the ramp (or the spring known drawn) and the launcher not at rest; dir -1 inside LAUNCH is allowed |
| `WRAPS_ON` | dir -1 in REEL (leaving REEL needs the centering jog: use `unwrap`) |

A drum jog stops by itself 4 deg past the LAUNCH or REEL stop point toward the end stop (`ABORTED`, `what:"JOG"`,
`why:"drum end stop"`). Any jog clears `aimed` (drum) and cancels an arm token.

## 3. Telemetry

`{"tel":{...}}` at 10 Hz, 20 Hz while anything moves. Field order is fixed as below.

| field | type | meaning |
|---|---|---|
| `state` | string | `IDLE`, `MODE` (drum moving for `mode` or `estop`), `AIM`, `COCK`, `FIRE`, `WRAP`, `UNWRAP`, `REEL`, `HOLD`, `JOG`, `DECOCK`, `SAFING` (fault: drum on its way to NEUTRAL), `FAULT`. (`CAL` exists in the code but is never used) |
| `mode` | string | what the collar really does: `LAUNCH` (code LAUNCH, `SW_DOG_L` closed, `SW_DOG_R` open), `REEL` (code REEL, `SW_DOG_R` closed, `SW_DOG_L` open), `NEUTRAL` (code NEUTRAL, wrap prep or WRAP, both dog contacts open), else `NONE` (shifting, lock sector, code invalid, contacts disagree) |
| `drum_deg` | number \| null | drum angle estimate, deg (stop points LAUNCH 0, NEUTRAL 41, REEL 347; WRAP sector 62 to 302). Snapped at every code edge, dead reckoning in between, never outside the sector the code shows. Null when the code was never valid |
| `drum_pos` | string | from the drum code sector: `LAUNCH`, `NEUTRAL`, `REEL`, `WRAPPED` (lock sector 302 to 340: wraps on, collar ramping), `BETWEEN` (LAUNCH to NEUTRAL ramp, wrap prep, WRAP sector), `UNKNOWN` (code invalid) |
| `wiper_deg` | number \| null | wiper output shaft angle, multi-turn, FORWARD positive. Null until the AS5600 has read once |
| `rpm` | number \| null | wiper output rpm, signed; null while the AS5600 is not reading |
| `amps` | number | wiper current (A) |
| `pitch_deg` | number \| null | rail pitch from the aim table; null while the aim is unknown |
| `cock` | number \| null | 0 at rest, 1 cocked (and while firing), between while cocking or resetting; null when the phase is `UNKNOWN` |
| `wraps` | integer \| null | capstan wraps: 0 from LAUNCH to wrap prep, an estimate from the drum angle in the WRAP sector, 7 in the lock sector and REEL. Null when unknown (power cut inside the WRAP sector) |
| `line_m` | number \| null | line out estimate: counts down while the capstan winds. Null after a shot (until `set line_m` or `SEATED`) and after a start-up without a saved record |
| `tension_n` | number \| null | LOAD CELL tension only; null without a live load cell (the current estimate appears only in `LIMIT`) |
| `sw` | object | CLOSED contacts only, value 1 (absent = open). Keys below |
| `fault` | string \| null | latched fault code (section 6) |
| `vbat` | number \| null | battery volts. About 0 with the battery unplugged (not null). Null only when `vbat_wired` is false |

`sw` keys (one letter each, order as listed):

| key | contact | closed means |
|---|---|---|
| `a` `b` `c` | `SW_DRUM_A` `SW_DRUM_B` `SW_DRUM_C` | drum code bits A B C (table in section 5) |
| `l` | `SW_DOG_L` | collar engaged with the LAUNCH dog ring |
| `r` | `SW_DOG_R` | collar engaged with the REEL gear |
| `n` | `SW_N20` | drum N20 in mesh (open = hand lever latched out: hand mode) |
| `H` | `SW_HOME` | carriage on its front stop (uncocked) |
| `G` | `SW_GAP` | big gear in its toothless gap (rest) |
| `L` | `SW_RAIL_LOW` | rail at its low dead point (aim home) |
| `S` | `SW_SEATED` | dart tail on the pusher |
| `W` | `SW_ARM` | wrap cup ring at home (one pulse per cup turn) |
| `I` | `SW_INDEX` | index cam bump (only on a build without the AS5600 magnet, decision D12; not fitted by default) |

Shot phases (in `status`, `BOOT` and behind `cock`): `REST` (carriage home, big gear in its gap), `COCKING` (carriage
partly drawn, angle known; also at rest after a forward jog in LAUNCH), `COCKED`, `FIRING`, `RESETTING`, `UNKNOWN`.

Examples (from the simulation):

```
{"tel":{"state":"IDLE","mode":"LAUNCH","drum_deg":0,"drum_pos":"LAUNCH","wiper_deg":-4106.2,"rpm":0,"amps":0,"pitch_deg":34.98,"cock":0,"wraps":0,"line_m":null,"tension_n":null,"sw":{"a":1,"l":1,"n":1,"H":1,"G":1,"S":1,"W":1},"fault":null,"vbat":12.4}}
{"tel":{"state":"WRAP","mode":"NEUTRAL","drum_deg":181.3,"drum_pos":"BETWEEN","wiper_deg":-3972.6,"rpm":0,"amps":0,"pitch_deg":34.98,"cock":0,"wraps":3,"line_m":null,"tension_n":null,"sw":{"c":1,"n":1,"H":1,"G":1},"fault":null,"vbat":12.4}}
{"tel":{"state":"IDLE","mode":"REEL","drum_deg":347,"drum_pos":"REEL","wiper_deg":4455,"rpm":0,"amps":0,"pitch_deg":35,"cock":0,"wraps":7,"line_m":0,"sw":{"a":1,"b":1,"c":1,"r":1,"n":1,"H":1,"G":1,"W":1},"vbat":12.3,"lean":true}}
```

Line length forms:

- Normal: every field, unknown values as `null`, up to 1 decimal (`pitch_deg`, `cock`, `line_m` 2 decimals).
- Lean: when the normal line would pass 256 bytes, it is rebuilt with one decimal less and null fields LEFT OUT, and
  `"lean":true` is added at the end. This is common in REEL (many contacts closed at once). **An absent field means
  null.** The Jetson must accept both forms on every line.
- Truncated: if even the lean line does not fit (only with absurd values):
  `{"tel":{"state":...,"mode":...,"drum_pos":...,"fault":...,"trunc":true}}`.

## 4. Events

`{"evt":"NAME","t":<ms since boot>, ...}` (`HELLO` has no `t`). The Jetson must ignore events it does not know and
fields it does not know.

| evt | fields | when |
|---|---|---|
| `HELLO` | `fw`, `reset` (`POWERON`, `BROWNOUT`, `PANIC`, `WATCHDOG`, `SOFTWARE`, `USB`, `EXTERNAL`, `DEEPSLEEP`, `OTHER`), `hx711`, `as5600`, `pwm` (false: a PWM pin failed and the wiper bridge stays disabled: do not run) | once at start-up. Treat it as a new session |
| `BOOT` | `fw`, `record` (a saved state was found), `was_moving` (power was lost mid-motion), `drum_known`, `wraps_known`, `aim_known`, `phase`, `angle` (`AS5600`, `INDEX` or `NONE`: which angle source the base uses) | 60 ms after start-up, once the sensors settled (section 9) |
| `MODE` | `ok`, `mode`, `drum_pos`; with `ok:false` also `err` (`DRUM_HELD`, or the fault code of a failed safing move) and `why` (text) | a drum move ended (`mode`, `wrap` and `unwrap` after their own event, `estop`, safing); also right after `estop` or a fault when the drum is already at NEUTRAL; and as the reply event of `cal drum`. `err:"DRUM_HELD"`: the drum deliberately did not go to NEUTRAL |
| `AIMED` | `ok`, `pitch_deg` (null if unknown), `target_deg`; `err` (`AIM_MISS` after 3 tries, `OUT_OF_RANGE`) when `ok:false` | aim ended |
| `COCKED` | `ok`, `cock_deg` (wiper deg forward from rest) | cocked |
| `FIRED` | `pitch_deg`, `cock_deg` (wiper deg of the release) | the carriage was released (`SW_HOME` closed). `aimed` is cleared and `line_m` becomes null |
| `RESET` | `ok`; `how` (`decock` or `decock by travel`) after `cal decock` | the launcher is back at rest after a shot or a decock |
| `WRAPPED` | `ok`, `wraps` | wraps laid and REEL engaged (followed by `MODE`) |
| `UNWRAPPED` | `ok` | back at NEUTRAL with no wraps (followed by `MODE`) |
| `SEATED` | | `SW_SEATED` closed during `reel until seated` (`line_m` set to 0) |
| `LIMIT` | `tension_n` (null for the tidy), `max_n`, `why` (`tension limit` or `tidy done (take-up slipping)`), `holding` | the reel reached `max_n` (holding when `until:"tension"`), or the tidy finished |
| `STALL` | | the wiper stalled (sent right after the `FAULT` with code `STALL`) |
| `FAULT` | `code`, `why` (up to 47 characters) | a fault latched (section 6) |
| `ABORTED` | `what` (the state that was running: `MODE`, `AIM`, `COCK`, `FIRE`, `WRAP`, `UNWRAP`, `REEL`, `HOLD`, `JOG`, `DECOCK`; or `HOST`), `why` (`stop`, `estop`, `timeout`, `would pass the release tooth`, `drum end stop`, `host silent`) | an action ended without completing. Treat it as the failure of whatever you were waiting for (a `stop` you sent yourself also gives one). The host watchdog sends two: the stopped state with `why:"stop"`, then `what:"HOST"` |
| `OVERFLOW` | (no `t`) | an event did not fit 256 bytes (should never happen) |

Example sequence for `wrap` (simulation): reply `{"id":12,"ok":true}`, about 14 s later
`{"evt":"WRAPPED","t":47810,"ok":true,"wraps":7}` then `{"evt":"MODE","t":47810,"ok":true,"mode":"REEL","drum_pos":"REEL"}`.

## 5. Drum, collar and the shift sequence

Drum sectors in travel order (drum dir 1 = toward REEL), from params.DRUM and lens B table 4.1. Code bits A B C, 1 =
contact closed. Adjacent sectors differ by one bit (Gray code); 000 never occurs on a working drum.

| sector | drum deg | code A B C | `sw` | collar | `drum_pos` |
|---|---|---|---|---|---|
| LAUNCH | -5 to 8 (stop point 0) | 1 0 0 | `a` | LAUNCH | `LAUNCH` |
| ramp L to N | 8 to 32 | 1 1 0 | `a` `b` | shifting | `BETWEEN` |
| NEUTRAL | 32 to 50 (stop point 41) | 0 1 0 | `b` | NEUTRAL | `NEUTRAL` |
| wrap prep | 50 to 62 | 0 1 1 | `b` `c` | NEUTRAL, cup lock releases | `BETWEEN` |
| WRAP | 62 to 302 | 0 0 1 | `c` | NEUTRAL, cup lays 7 turns | `BETWEEN` |
| lock | 302 to 340 | 1 0 1 | `a` `c` | ramp N to R, cup locked | `WRAPPED` |
| REEL | 340 to 355 (stop point 347) | 1 1 1 | `a` `b` `c` | REEL (wraps on) | `REEL` |

End stops at -8 and 358 deg are never driven into.

How a drum move runs (for `mode`, `wrap`, `unwrap`, `estop` and safing):

1. Checks (section 2 and 6). The record "moving" goes to flash.
2. Centering jog, only when leaving an engaged collar (LAUNCH or REEL) and not when safing, and only while the AS5600
   reads: in LAUNCH the wiper turns 30 deg BACKWARD (inside the 60 deg dog window, so the collar touches no flank;
   never forward, that would cock). In REEL it first turns FORWARD at 20 % until the window is taken up (60 deg or
   3 s), then 30 deg back (sprag #2 holds the line meanwhile). Trip 3 A. The 30 deg leg is also capped in time (1.5 x
   its free-speed time), so a noisy angle cannot carry it past the window.
3. The N20 runs to the target sector's code edge, then on by time to the stop point. The drum speed (`drum_dps`,
   default 24 deg/s) is re-learned between every two code edges.
4. While unwrapping, the wiper turns forward at 40 % in the WRAP and wrap prep sectors (take-up, trip 2.5 A).
   Toward REEL the fly-in starts at 318 deg (end of the lock sector): wiper forward at 20 % (trip 3 A).
5. At the stop point the dog contacts must confirm the collar: REEL needs `SW_DOG_R` (fly-in continues for at most
   120 deg of wiper or 6 s); LAUNCH needs `SW_DOG_L` (if not yet, the wiper is turned BACKWARD at 20 % for at most
   40 deg, never forward; with the spring possibly drawn this is a fault instead); NEUTRAL needs both open after
   100 ms. After arriving in LAUNCH the aim is forgotten (`aim_known` false): the next `aim` re-homes.
6. `MODE` event (after `WRAPPED` / `UNWRAPPED` for those commands).

Timeouts: no code edge for 1.5 x the time the current sector should take plus 1.5 s, or the whole move over 1.5 x the
expected time plus 1.5 s: `DRUM_TIMEOUT`.

## 6. Faults and safety

### Interlocks (lens B 7.1, as the firmware applies them)

- The spring may be drawn ("loaded") whenever the phase is not `REST`, the angle says past the first tooth, or
  `SW_HOME` is open. Leaving LAUNCH (by `mode`, drum `jog`) needs the launcher not loaded and `SW_GAP` closed; this is
  checked whenever the drum code says LAUNCH or the ramp, the code is invalid, or `SW_DOG_L` is closed, so a failed
  code bit cannot open the way. With the phase known loaded (`COCKING`, `COCKED`, `FIRING`) every drum move is refused
  (`LAUNCH_BUSY`).
- `aim`, `cock`, `fire` and `cal decock` need the collar in LAUNCH all the time; the collar leaving LAUNCH during one
  is `BAD_STATE`.
- Aim only at rest. Fire needs LAUNCH, the right `SW_HOME` / `SW_GAP` state, `SW_SEATED` (while `require_seated`), a
  fresh aim and a valid token.
- Wiper backward is refused in REEL (`BACKSTOP`); forward in NEUTRAL and the wrap sectors is take-up only (40 %).
- The drum never moves while `SW_N20` is open (hand mode); the N20 latching out during a move is a fault.
- No mode change with wraps on; REEL only with the wraps on.

### Current and stall protection (decision D8)

| state | trip |
|---|---|
| reel, tow, `cal decock`, and the default `trip_a` | 5 A |
| cock, fire, forward jog in LAUNCH (`cock_trip_a`) | 6.5 A |
| take-up only (reel in NEUTRAL, take-up during unwrap, forward jog in NEUTRAL or the wrap sectors) | 2.5 A |
| jogs, centering jog, fly-in | 3 A |
| any state, never ignored | 17 A |
| aim | no trip of its own: it keeps the trip the previous motion left (2.5, 3 or 5 A). Known firmware issue, expected to become 5 A |

A trip needs 6 samples in a row (30 ms) over the limit, ignored for the first 50 ms after a start (inrush), except the
17 A limit. Stall: driving above 15 % duty, after the 300 ms soft start, shaft slower than 12 deg/s for 200 ms. Both
are always faults (also in a tow). A printed torque limiter in the coupler slips at about 5 N m.

### Fault codes

The first fault latches; later ones are ignored until `clear` (one exception: a second fault during `SAFING` stops the
drum, see below).

| code | causes (`why` texts) |
|---|---|
| `OVERCURRENT` | wiper current over the state trip for 30 ms (`wiper over N A`) |
| `STALL` | `wiper stalled` (also sends a `STALL` event) |
| `DRUM_TIMEOUT` | `drum not turning (no code edge)`, `drum did not reach the target in time`, `dogs did not engage (collar contact open)` |
| `DRUM_SENSOR` | `drum code 000 (wire off or bump missing)` or `drum code jumped over a sector` for 150 ms; `drum turned the wrong way (drum_sign?)` (N20 wires swapped); `target code came far too early`; `drum overshot the target sector`; `drum code changed with the drum still` (the worm cannot be back-driven: a contact is lying, or the drum was turned by hand while the base runs); `N20 latched out during a drum move`; `SW_ARM counted n of 7 cup turns`; `drum code disagrees with the saved drum angle` (at boot) |
| `ANGLE_SENSOR` | `AS5600 lost while the wiper runs` (4 bad reads, or no magnet); `AS5600 counts against the drive (enc_sign?)`; `AS5600 glitching (over 20 bad samples a second)`; in index mode `SW_INDEX bump overdue (index cam or contact)` |
| `SWITCH_DISAGREE` | `SW_DOG_L and SW_DOG_R both closed` (200 ms); `drum code contact chattering` during a drum move; `collar still engaged at NEUTRAL`; `SW_HOME opened at the wrong angle`; `SW_HOME still closed past the first tooth`; `SW_HOME chattering during decock` |
| `BROWNOUT` | `battery sagged under load`: under `vbat_low` for 300 ms while a motor runs |
| `BAD_STATE` | the mechanism did not do what the angle says: `collar left LAUNCH during a launch action`, `drum code away from LAUNCH but the spring is drawn`, `LAUNCH collar not confirmed with the spring possibly drawn`, `SW_RAIL_LOW never closed in an aim cycle`, `aim ran too far`, `carriage not drawn (SW_HOME still closed): rack or dog not engaged`, `carriage slipped home while cocking`, `SW_HOME closed at the end of cocking`, `SW_HOME never opened (stuck closed?)`, `release at the wrong angle` (after `FIRED`), `no release seen (SW_HOME did not close)`, `SW_HOME opened while resetting`, `reset ended with the carriage off its stop`, `reset ended with the big gear out of its gap`, `decock: SW_HOME never closed`, and the time limits (`cocking took too long`, `cock recovery took too long`, `fire cycle took too long`, `decock by travel took too long`) |
| `DART_STUCK` | `final approach: no SW_SEATED in 20 s` |

A fault during any launch-train motion makes the shot phase `UNKNOWN` (the last angle is kept as the phase hint,
section 9). Any fault cancels the arm token.

### Fault response

The wiper stops at once. Then the drum goes to NEUTRAL (state `SAFING`, then `FAULT`, `MODE` event with `ok:true`),
without any wiper motion, EXCEPT when that would be unsafe. Then the drum holds where it is (the worm holds it) and a
`MODE` event with `ok:false`, `err:"DRUM_HELD"` and `why` says why:

| `why` | meaning |
|---|---|
| `drum fault` | the fault is `DRUM_TIMEOUT` or `DRUM_SENSOR`: the drum may be jammed or its position wrong |
| `drum code invalid` | |
| `N20 latched out (hand mode)` | |
| `wraps unknown` | |
| `wraps on: the line must be unwrapped by the operator` | WRAP sector, lock or REEL: the capstan holds the line (sprag #2); recover with `clear`, then `unwrap` |
| `launch train not at rest: the wiper worm holds the spring (a shift under load could dry fire)` | there is no sear: the drawn spring is held only by the wiper worm through the collar. Recover with `clear`, then `cal decock` (or `fire` if it is safe to shoot) |

Already at NEUTRAL: an immediate `MODE` with `ok:true`. A second fault during `SAFING` stops the drum and sends
`MODE` with `err:"DRUM_HELD"` and that fault's `why`. A safing move that fails sends `MODE` with `ok:false`,
`err` = the drum fault code and its `why`, and stays in `FAULT`. `stop` during `SAFING` stops the drum (no `MODE`
event).

## 7. Calibration and settings

`cal` actions (`{"id":1,"cmd":"cal","what":"current"}`):

| what | args | does | refusals |
|---|---|---|---|
| `current` | | zero the wiper current reading (motor stopped) | `BUSY` (running) |
| `free` | | learn the free-running current from the run in progress (after 0.8 s of running) | `NOT_RUNNING` |
| `tare` | | zero the load cell | `NO_SENSOR` |
| `aim` | `deg`: pitch measured with a phone inclinometer | sets `aim_offset` so the reported pitch matches | `BAD_ARG` (no `deg`), `AIM_UNKNOWN` |
| `aim_home` | | declare the aim train at its home (where `SW_RAIL_LOW` closes on the backward run) | `BUSY` (moving) |
| `aim_forget` | | forget the aim: the next `aim` re-homes | `BUSY` |
| `cock_rest` | `force` (optional, true: `SW_HOME` known broken, checked by eye) | declare the launcher at rest (carriage forward). For a new build, or after checking by eye | `BUSY`, `SWITCH_DISAGREE` (`SW_HOME` open and no `force`) |
| `decock` | `force` (optional) | lets a drawn carriage down without firing: runs the wiper BACKWARD slowly (47 %, trip 5 A) so the spring unwinds under the worm, until `SW_HOME` has stayed closed for 15 deg more. Turns the aim train too: the aim becomes unknown. `RESET` event with `how:"decock"`. Carriage already home: phase `REST` gives an immediate `RESET`; phase `UNKNOWN` with no sign of a draw replies `ok` and does nothing (no event: use `cock` or `cal cock_rest`). `SW_HOME` closed while the last known angle says drawn: refused with `SWITCH_DISAGREE`, unless `force:true`: then it runs back by the travel that angle needs and calls it rest (`how:"decock by travel"`) | common (`ANGLE_SENSOR` included), `NOT_IN_LAUNCH` (drum code not LAUNCH), `NO_SENSOR` (no `SW_HOME`), `SWITCH_DISAGREE` |
| `drum` | | the drum code is absolute: this only reports, with a `MODE` event | `POSITION_UNKNOWN` |

`set` / `get` keys. Saved in flash once everything has stopped, kept over a power cycle and a new upload, except where
the table says otherwise:

| key | default | range | meaning |
|---|---|---|---|
| `limit_n` | 50 (params.REEL) | 1..110 | default `max_n` for `reel` |
| `k_n_per_a` | 8 | 0.1..200 | line N per amp above `free_a` (tension estimate without a load cell) |
| `free_a` | 1.5 (params.WIPER) | 0..10 | wiper free-running current |
| `amps_per_volt` | 12.5 | 1..200 | BTS7960 IS scale |
| `offset_a` | 0 | 0..5 | current zero (set by `cal current`) |
| `hx_n_per_count` | 0.0000476 | 0.0000001..1 | load cell scale |
| `drum_dps` | 24 | 5..720 | drum speed estimate (learned during moves; saved with the next `set` or `cal`) |
| `aim_tol` | 0.75 | 0.1..5 | AIMED tolerance, deg |
| `aim_offset` | 0 | -20..20 | pitch offset (set by `cal aim`) |
| `aim_home_deg` | 3365 | -7200..7200 | aim-train angle (backward wiper deg from the low dead point) where `SW_RAIL_LOW` closes |
| `line_m` | | 0..100 | line out. Tell the base after a shot (from the landing distance); `get` returns the number even while telemetry shows null. Kept in the position record |
| `trip_a` | 5 | 3..19 | wiper trip for reel, tow and decock |
| `cock_trip_a` | 6.5 | 3..19 | wiper trip for cock and fire |
| `vbat_low` | 9.9 | 6..12 | BROWNOUT threshold under load |
| `require_seated` | true | true / false | `arm` and `fire` need `SW_SEATED` |
| `vbat_wired` | true | true / false | the battery divider is fitted; false turns off `NO_POWER` and `BROWNOUT` and makes `vbat` null |
| `enc_sign` | 1 | 1 / -1 | AS5600 count direction (set while stopped) |
| `host_timeout_s` | 0 (off) | 0..600 | host watchdog: no line from the Jetson for this long while a motor runs = `stop` (two `ABORTED` events, the second `what:"HOST"`, `why:"host silent"`). Send `ping` to keep long runs alive. NOT saved: back to 0 after every start-up |
| `fw`, `params`, `pitch_range` | | get only | firmware version, params.py version (`1.0`), `min` / `max` pitch (6.01 / 47 with the built-in table) |

## 8. Typical mission (lens B 7.2, the params.TIMELINE scenario of the PC tests)

```
{"id":1,"cmd":"mode","to":"LAUNCH"}        -> MODE ok              (step 15/1: collar into LAUNCH, aim forgotten)
{"id":2,"cmd":"aim","deg":35}              -> AIMED ok             (step 2: homes on SW_RAIL_LOW first)
{"id":3,"cmd":"cock"}                      -> COCKED               (optional: fire from rest also works, about 2 s slower)
{"id":4,"cmd":"arm"}                       <- {"id":4,"ok":true,"token":482913,"valid_ms":3000}
{"id":5,"cmd":"fire","token":482913}       -> FIRED, RESET         (step 3)
   dart LANDED (dart link): {"id":6,"cmd":"set","key":"line_m","val":4.6}
{"id":7,"cmd":"mode","to":"NEUTRAL"}       -> MODE ok              (step 5: centering jog, drum to NEUTRAL)
   dart ANCHORED: {"id":8,"cmd":"reel","duty":0.2,"until":"slip"}  -> LIMIT "tidy done"   (step 6)
{"id":9,"cmd":"wrap"}                      -> WRAPPED, MODE REEL   (steps 7, 8: about 15 s, wait up to 60 s)
{"id":10,"cmd":"reel","duty":1,"max_n":50,"until":"tension"}       -> LIMIT holding        (step 9: tow)
{"id":11,"cmd":"stop"}                     -> ABORTED HOLD         (step 10: the dart releases)
{"id":12,"cmd":"reel","duty":0.8,"until":"stop"}   ... line_m about 1.5 ... {"id":13,"cmd":"stop"} -> ABORTED REEL
{"id":14,"cmd":"unwrap"}                   -> UNWRAPPED, MODE NEUTRAL   (steps 11 to 13: about 16 s)
{"id":15,"cmd":"reel","duty":0.3,"until":"seated"} -> SEATED       (step 14; FAULT DART_STUCK after 20 s)
{"id":16,"cmd":"mode","to":"LAUNCH"}       -> MODE ok              (step 15, then back to step 2)
```

Notes for the Jetson:

- After every shift into LAUNCH run `aim` before `cock` or `fire`. `arm` needs it anyway, and only an aim run tells
  the base where the collar sits in its dog window (a `cock` sent straight after a shift can fault with `BAD_STATE`
  "carriage not drawn" in the simulation).
- Wait for `RESET` after `FIRED` before the next command (the reset is part of `fire`).
- `line_m` is null after a shot. Set it from the landing distance, or the capstan line count and the seat-slow zone do
  not work.
- Treat `ABORTED` and `FAULT` as the end of whatever you were waiting for. After a `FAULT`, read the `MODE` event that
  follows (`ok:true` at NEUTRAL, or `DRUM_HELD` with the reason).
- Lens B step 4 (flight) and the LANDED / ANCHORED / release steps are dart and Jetson events; the base only sees the
  commands.

## 9. Power loss and the saved state

Both worm gears self-lock, so the mechanism stays where it stopped. The base keeps a position record in flash: at every
motion start it is written with "moving" set (a motor starts only once that write is done), and again with the final
positions about 300 ms after everything stopped. At the next start the `BOOT` event says what is known:

- Drum: the sector always comes from the drum code (absolute). The angle inside it comes from the record if the drum
  was not moving and the record agrees with the sector, else the stop point (LAUNCH, NEUTRAL, REEL) or the sector
  middle. A record that was not moving but disagrees with the code (drum turned by hand while off, or a bad contact)
  raises `DRUM_SENSOR` "drum code disagrees with the saved drum angle" (`clear` it once checked). `wraps` follows from
  the sector; inside the WRAP sector it is unknown (`wraps_known` false, `wraps` null) unless the angle was trusted.
  An interrupted move is repeated with `mode`, `wrap` or `unwrap`; `unwrap` is safe at any wrap count.
- Wiper: the AS5600 angle inside one turn is checked against the saved angle. If the wiper was moving, or the shaft
  was turned while off (over 3 deg), the aim becomes unknown (re-homed by the next `aim`) and the shot phase `UNKNOWN`.
- A launch train stopped mid-cycle (`COCKING`, `FIRING`, `RESETTING`) is never trusted: `UNKNOWN`.
- `SW_HOME` and `SW_GAP` must agree with the record: `REST` needs both closed, `COCKED` needs `SW_HOME` open; anything
  else gives `UNKNOWN`.
- `line_m` comes back from the record (null if it was unknown or there is no record).

The phase hint: whenever the phase becomes `UNKNOWN` (power cut, `stop` or a fault during a launch-train motion), the
last known cock angle is kept, also in flash. It is not reported, but it decides:

- `cock` is refused with `LOADED_UNKNOWN` when `SW_HOME` is open, or when `SW_HOME` is closed but the hint says the
  carriage was drawn (between the first tooth and the release): the switch may be lying and a creep could fire.
- `cal decock` with `SW_HOME` closed and such a hint is refused with `SWITCH_DISAGREE`; with `force:true` it runs back
  by the travel the hint needs (`RESET` `how:"decock by travel"`).

Recovery after `BOOT` (lens B 7.4):

| what the base reports | recovery |
|---|---|
| `mode` LAUNCH, `H` and `G` closed, phase `REST` | none |
| phase `UNKNOWN`, `H` open (carriage drawn) | `cal decock`, then `aim`, then `cock` / `fire` |
| phase `UNKNOWN`, `H` closed | `cock` (creeps until `SW_HOME` opens, then cocks), or `cal cock_rest` after checking by eye. `LOADED_UNKNOWN`: check by eye, then `cal decock` with `force:true` or `cal cock_rest` |
| `drum_pos` REEL, `r` closed | wrapped, line held: resume the tow or reel in, or `unwrap` |
| WRAP sector, `wraps` null | `unwrap` (take-up running), then `wrap` again if needed |
| `drum_pos` BETWEEN in the ramp, phase `UNKNOWN` | only `mode LAUNCH` is allowed (`LAUNCH_BUSY` otherwise) until the launcher is seen at rest |
| `drum_pos` WRAPPED (lock sector) | `wrap` (forward into REEL) or `unwrap` |
| `drum_pos` UNKNOWN | `FAULT` `DRUM_SENSOR`: check the code contacts; hand lever |

Without the AS5600 magnet (decision D12): if `SW_INDEX` is switched on in `config.h` (it is off by default) and the
AS5600 does not read at start-up, the base uses the 6-bump index cam instead (`BOOT` `angle:"INDEX"`, `status`
`magnet:"INDEX"`). The angle then moves in 60 deg steps with time interpolation in between and errs high: `cock` stops
one bump (60 deg) earlier, the `SW_HOME` and release angle windows are 60 deg wider, aim is within about 3 deg, and
a missing bump while driving is `ANGLE_SENSOR` "SW_INDEX bump overdue (index cam or contact)". Index mode is chosen
only at start-up.

## 10. Changes since the base-1.0 draft of this file

What the Jetson has to change (the firmware version string is still `base-1.0`):

1. Drum order is LAUNCH / NEUTRAL / WRAP / REEL (D16). `mode REEL` is refused without wraps (`NOT_WRAPPED`): use
   `wrap` from NEUTRAL. `wrap` now starts in NEUTRAL (`NOT_IN_NEUTRAL` replaces `NOT_IN_REEL`) and ends in REEL with a
   `WRAPPED` then a `MODE` event; `unwrap` starts in REEL and ends in NEUTRAL with `UNWRAPPED` then `MODE`. 7 wraps,
   not 4. Leaving REEL is only by `unwrap` (`mode` gives `WRAPS_ON`).
2. Stop points: NEUTRAL 41, REEL 347 (was 40 / 80 / 338). Drum speed default 24 deg/s: about 15 s for a wrap.
3. `reel` in NEUTRAL is allowed (take-up only): `until:"seated"` (final approach, new fault `DART_STUCK` after 20 s)
   and the new `until:"slip"` (tidy, `LIMIT` with `why:"tidy done (take-up slipping)"` and `tension_n:null`).
   `reel` in REEL needs the collar confirmed. `max_n` cap 110 (was 150); `limit_n` default 50 (was 20).
4. Telemetry `sw` keys: `D` (`SW_DOG`) is gone; new `l` (`SW_DOG_L`), `r` (`SW_DOG_R`), `n` (`SW_N20`), `G`
   (`SW_GAP`), `I` (`SW_INDEX`); `W` is now `SW_ARM` (wrap cup at home).
5. Telemetry `mode` is `NEUTRAL` during the wrap (the collar stays in NEUTRAL), `drum_pos` `WRAPPED` means the lock
   sector, and `vbat` reads about 0 (not null) with the battery unplugged.
6. New lean telemetry form: null fields left out and `"lean":true`. An absent field means null.
7. New refusals: `HAND_MODE`, `NOT_IN_NEUTRAL`, `SHIFTING`, `BAD_ID`, `REPLY_OVERFLOW`; `LAUNCH_BUSY` also when
   `SW_GAP` is open; `NOT_COCKED` also checks `SW_GAP`; `WRAPS_ON` for a drum jog back out of REEL.
8. New fault `DART_STUCK`; many new `why` texts (section 6); `SWITCH_DISAGREE` and `DRUM_SENSOR` have new causes.
9. Current trips per state (D8): 5 / 6.5 / 2.5 / 3 A, hard 17 A, 30 ms after a 50 ms inrush ignore; stall 12 deg/s
   for 200 ms (was 17 A / 15 A, 10 ms, 15 deg/s for 400 ms). `trip_a` default 5, `cock_trip_a` default 6.5.
10. Every shift out of LAUNCH or REEL starts with the centering jog; after a shift into LAUNCH the aim is unknown.
11. `cal decock` takes `force`; new `RESET` `how:"decock by travel"`; `cal cock_rest` takes `force`.
12. `BOOT` has a new field `angle`; `status` `magnet` may be `INDEX`.
13. New `ABORTED` `why` values `drum end stop`; `what` can be `HOLD`; the host watchdog sends two `ABORTED` events.
14. `host_timeout_s` is not saved in flash.
