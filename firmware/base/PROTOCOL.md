# GrappStar base station protocol (contract C1, final)

Firmware `base-1.0` (`firmware/base/grappstar_base`). This file is the contract the Jetson implements against. Changes
are additive only (new fields, new events, new error codes); nothing here is removed or renamed without a version bump.

## 1. Link

- The ESP32-S3 DevKitC-1 native USB port (USB CDC, the port labelled USB). On the Jetson it shows up as `/dev/ttyACM*`.
  The baud rate is ignored.
- Newline-terminated JSON objects, UTF-8, no spaces added by the base. At most 256 bytes per line in both directions
  (the newline not counted).
- Lines that do not start with `{` (ROM boot text after a reset) must be ignored by the Jetson.
- Jetson to base: one command object per line. Base to Jetson: replies, telemetry and events, interleaved, each a
  whole line (the base never writes half a line; when the USB buffer is full telemetry is dropped, events wait).

## 2. Commands and replies

Every command line gets exactly one reply line, in order:

```
-> {"id": 17, "cmd": "aim", "deg": 32.5}
<- {"id":17,"ok":true}
<- {"id":18,"ok":false,"err":"NOT_IN_LAUNCH"}
```

- `id`: echoed. A number (integers are echoed as integers) or a string (echoed, at most 63 characters). Missing or
  null gives `"id":null`. Use unique increasing integers.
- A line that is not valid JSON gets `{"id":null,"ok":false,"err":"BAD_JSON"}`; a line over 256 bytes gets
  `{"id":null,"ok":false,"err":"TOO_LONG"}`. No `cmd`: `NO_CMD`. Unknown `cmd`: `UNKNOWN_CMD`.
- Unknown extra keys in a command are ignored. Nested objects or arrays as values are skipped.
- Long actions (`mode`, `aim`, `cock`, `fire`, `wrap`, `unwrap`, `reel`, `jog`, `cal drum`, `cal decock`, `estop`) reply
  `ok:true` as soon as they START. Completion is an event (section 4). A refusal replies `ok:false` with `err` and
  nothing moves.
- While an action runs, every other action is refused with `BUSY`. `ping`, `status`, `get`, `set`, `stop`, `estop` and
  `clear` always answer.
- The reply to a command is always written before any event that command caused.

### Command list

| cmd | args | does | refusals (err) |
|---|---|---|---|
| `ping` | | reply carries `fw` and `t` (ms since boot) | |
| `status` | | reply carries `phase`, `aimed`, `aim_known`, `drum_known`, `wraps_known`, `magnet` (`OK`, `WEAK`, `STRONG`, `NONE`), `glitches` (AS5600 rejected samples), `hx` (load cell live), `peak_a`, `fw`, and `why` (fault reason, 32 chars) while faulted | |
| `stop` | | wiper off now, drum stops where it is (the worms hold both). Aborts any action (`ABORTED` event). A stop during cock or fire makes the shot phase `UNKNOWN` | never refused |
| `estop` | | like `stop`, then the drum goes to NEUTRAL when that is safe (state `MODE`, `MODE` event at the end). When it is not safe the drum holds and a `MODE` event with `ok:false`, `err:"DRUM_HELD"` and `why` says why (see 6) | never refused |
| `clear` | | clear a fault, back to `IDLE` (ok and no change when not faulted) | `STILL_PRESENT` (the cause is still there: AS5600 missing, impossible drum code, battery low), `BUSY` (still moving the drum to NEUTRAL) |
| `mode` | `to`: `LAUNCH` \| `NEUTRAL` \| `REEL` | moves the drum to that detent and verifies it (drum code, timing). Already there: ok and an immediate `MODE` event | `BAD_ARG`, `WRAPS_ON`, `POSITION_UNKNOWN` (run `cal drum`), `LAUNCH_BUSY` (leaving LAUNCH needs the shot phase `REST`), `SWITCH_DISAGREE`, `HOLDING`, common |
| `aim` | `deg`: rail pitch | turns the wiper BACKWARD until the pitch (from the aim table) is within `aim_tol` of `deg`. Homes on `SW_RAIL_LOW` first if the aim is unknown. `AIMED` event | `NOT_IN_LAUNCH`, `PHASE_UNKNOWN`, `NOT_AT_REST` (aim before cocking: backward drive with the spring drawn would let it unwind), `OUT_OF_RANGE` (see `get pitch_range`), `AIM_UNKNOWN` (no `SW_RAIL_LOW` fitted and not calibrated), common |
| `cock` | | turns the wiper FORWARD from rest to the cocked angle (carriage drawn, just before the release tooth). Checks `SW_HOME` opens at the first tooth. `COCKED` event. With the phase `UNKNOWN` and the carriage home it creeps until `SW_HOME` opens, which re-zeroes the angle. Already cocked: ok and an immediate `COCKED` event | `NOT_IN_LAUNCH`, `NO_SENSOR` (no `SW_HOME`), `LOADED_UNKNOWN` (carriage drawn with the angle unknown: `cal decock` first), `SWITCH_DISAGREE`, common |
| `arm` | | checks everything `fire` needs and returns `"token": <6-digit integer>, "valid_ms": 3000`. A new `arm` replaces the old token | `NOT_IN_LAUNCH`, `WRAPS_ON`, `NOT_COCKED`, `NOT_AIMED` (not aimed, or the pitch moved more than `aim_tol` from the aim target), `NOT_SEATED`, `NO_SENSOR` (no `SW_SEATED` while `require_seated`), common |
| `fire` | `token`: the integer from `arm` (a numeric string is accepted) | checks everything again, then runs the wiper FORWARD past the release tooth (`FIRED` when `SW_HOME` closes) and on to rest (`RESET`). The token is single use; a wrong token cancels the arm | `NOT_ARMED`, `TOKEN_EXPIRED` (over 3 s), `BAD_TOKEN`, then the `arm` refusals |
| `wrap` | | drum on from REEL into the wrap dwell until the target wraps are laid (index `SW_WRAP_HOME` checked when fitted). `WRAPPED` event | `NOT_IN_REEL`, `WRAPS_ON` (already wrapped or unknown), `HOLDING`, common |
| `unwrap` | | drum back to the REEL detent. `UNWRAPPED` event (immediately if already unwrapped at REEL) | `NOT_WRAPPED`, `POSITION_UNKNOWN`, `HOLDING`, common |
| `reel` | `duty` 0..1 (default 0.86), `max_n` (default `limit_n`, capped at 150), `until`: `stop` (default) \| `seated` \| `tension` | winds the capstan (wiper FORWARD). Speed tapers over the last 30 % of `max_n`. `until:"seated"`: stops at `SW_SEATED` (`SEATED` event), slow over the last 1 m of `line_m`. `until:"tension"`: stops at `max_n` and HOLDS (state `HOLD`, `LIMIT` event with `holding:true`; with a load cell it winds again if the line slackens below 60 %). Any `until`: reaching `max_n` stops with a `LIMIT` event. `stop` ends it | `NOT_WRAPPED` (reel needs the wraps on: the capstan carries the pull), `BAD_ARG`, `NO_SENSOR` (`seated` without `SW_SEATED`), common |
| `jog` | `motor`: `wiper` \| `drum`, `dir`: 1 \| -1, `ms` (default 300, capped at 1500) | runs one motor for `ms` (deadman: the same jog again before it ends extends it). Wiper dir 1 = FORWARD | `BAD_ARG`, `WOULD_FIRE` (forward with the launcher cocked or its phase unknown; a forward jog also stops itself before the release tooth), `NOT_AT_REST` (backward in LAUNCH with the carriage drawn), `BACKSTOP` (backward in REEL: sprag #2 holds the capstan), `BUSY` (the other motor runs), common |
| `cal` | `what`: see 5 | calibration actions | `BAD_ARG`, per action |
| `set` | `key`, `val` | sets a bench value, saved in flash | `BAD_KEY`, `OUT_OF_RANGE`, `BAD_ARG` |
| `get` | `key` | reply carries `val` (or `min`, `max` for `pitch_range`) | `BAD_KEY` |

Common refusals for actions: `FAULTED` (clear first), `BUSY`, `BOOTING` (first 60 ms), `NO_POWER` (battery
unplugged: under 6 V), `ANGLE_SENSOR` (AS5600 not reading; wiper actions only), `HOLDING` (the reel is holding a
tension: send `stop` first).

## 3. Telemetry

`{"tel":{...}}` at 10 Hz, 20 Hz while anything moves. Field order is fixed as below. Unknown values are `null`.

| field | type | meaning |
|---|---|---|
| `state` | string | `IDLE`, `MODE` (drum moving to a detent), `AIM`, `COCK`, `FIRE`, `WRAP`, `UNWRAP`, `REEL`, `HOLD`, `JOG`, `CAL` (drum home search), `DECOCK`, `SAFING` (fault: drum on its way to NEUTRAL), `FAULT` |
| `mode` | string | `LAUNCH`, `NEUTRAL`, `REEL` (also while wrapped), `NONE` (drum between detents or unknown) |
| `drum_deg` | number \| null | drum angle estimate, deg from the LAUNCH detent |
| `drum_pos` | string | `LAUNCH`, `NEUTRAL`, `REEL`, `WRAPPED`, `BETWEEN`, `UNKNOWN` |
| `wiper_deg` | number \| null | wiper output shaft angle, multi-turn, FORWARD positive |
| `rpm` | number \| null | wiper output rpm, signed |
| `amps` | number | wiper current (A) |
| `pitch_deg` | number \| null | rail pitch from the aim table (null while the aim is unknown) |
| `cock` | number \| null | 0 at rest, 1 cocked, between while cocking or resetting; null when the phase is unknown |
| `wraps` | integer \| null | capstan wraps |
| `line_m` | number \| null | line out estimate (null after a shot until `set line_m` or `SEATED`) |
| `tension_n` | number \| null | load cell tension; null without a load cell |
| `sw` | object | CLOSED contacts only, value 1 (absent = open): `a` `SW_DRUM_A`, `b` `SW_DRUM_B`, `c` `SW_DRUM_C`, `S` `SW_SEATED`, `H` `SW_HOME`, `L` `SW_RAIL_LOW`, `W` `SW_WRAP_HOME` |
| `fault` | string \| null | fault code (section 6) |
| `vbat` | number \| null | battery volts (null if the divider is not fitted) |

Example: `{"tel":{"state":"IDLE","mode":"LAUNCH","drum_deg":0,"drum_pos":"LAUNCH","wiper_deg":12.5,"rpm":0,"amps":0,"pitch_deg":35.02,"cock":0,"wraps":0,"line_m":0,"tension_n":null,"sw":{"a":1,"S":1,"H":1},"fault":null,"vbat":12.4}}`

If a line would exceed 256 bytes (only with absurd values) it is sent with fewer decimals, and failing that as
`{"tel":{"state":...,"mode":...,"drum_pos":...,"fault":...,"trunc":true}}`.

## 4. Events

`{"evt":"NAME","t":<ms since boot>, ...}`. The contract events plus four additive ones (`HELLO`, `BOOT`, `RESET`,
`ABORTED`). The Jetson must ignore events it does not know.

| evt | fields | when |
|---|---|---|
| `HELLO` | `fw`, `reset` (`POWERON`, `BROWNOUT`, `PANIC`, `WATCHDOG`, `SOFTWARE`, `USB`, `OTHER`), `hx711`, `as5600` (no `t`) | once at start-up |
| `BOOT` | `fw`, `record` (a saved state was found), `was_moving` (power was lost mid-motion), `drum_known`, `wraps_known`, `aim_known`, `phase` | 60 ms after start-up, once the sensors settled |
| `MODE` | `ok`, `mode`, `drum_pos`, `err` and `why` when `ok:false` | a drum move ended (also after `estop` or a fault). `err:"DRUM_HELD"`: the drum deliberately did not go to NEUTRAL |
| `AIMED` | `ok`, `pitch_deg`, `target_deg`, `err` (`AIM_MISS`, `OUT_OF_RANGE`) when `ok:false` | aim ended |
| `COCKED` | `ok`, `cock_deg` | cocked |
| `FIRED` | `pitch_deg`, `cock_deg` (wiper angle of the release) | the carriage was released (`SW_HOME` closed) |
| `RESET` | `ok`, `how` (`decock` after `cal decock`) | the launcher is back at rest after a shot or a decock |
| `WRAPPED` | `ok`, `wraps` | wraps laid |
| `UNWRAPPED` | `ok` | back at the REEL detent with no wraps |
| `SEATED` | | `SW_SEATED` closed during `reel until seated` (`line_m` is set to 0) |
| `LIMIT` | `tension_n`, `max_n`, `why`, `holding` | the reel reached `max_n` (holding when `until:"tension"`) |
| `STALL` | | the wiper stalled (sent with `FAULT` code `STALL`) |
| `FAULT` | `code`, `why` | a fault latched |
| `ABORTED` | `what` (the state that was running), `why` (`stop`, `estop`, `timeout`, `would pass the release tooth`) | an action ended without completing |

## 5. Calibration and settings

`cal` actions (`{"id":1,"cmd":"cal","what":"current"}`):

| what | args | does |
|---|---|---|
| `current` | | zero the wiper current reading (motor stopped) |
| `free` | | learn the free-running current (while a reel or jog has run over 0.8 s on a slack line) |
| `tare` | | zero the load cell (`NO_SENSOR` without one) |
| `aim` | `deg`: pitch measured with a phone inclinometer | sets `aim_offset` so the reported pitch matches |
| `aim_home` | | declare the aim train at its home (pitch minimum, where `SW_RAIL_LOW` closes) |
| `cock_rest` | | declare the launcher at rest (carriage forward, `SW_HOME` closed). For a new build |
| `decock` | | carriage drawn and its angle unknown (power lost mid-cock) or a cocked shot called off: runs the wiper BACKWARD slowly so the spring unwinds under the worm until `SW_HOME` closes. Turns the aim train too, so the aim becomes unknown. `RESET` event with `how:"decock"` |
| `drum` | | drum home search toward LAUNCH until a detent code is seen (after a power cut mid-move). Refused with wraps known on. The operator must check the line is slack first |

`set` / `get` keys (all saved in flash, kept over a power cycle):

| key | default | range | meaning |
|---|---|---|---|
| `limit_n` | 20 | 1..150 | default `max_n` for `reel` |
| `k_n_per_a` | 8 | 0.1..200 | line N per amp above free current (tension estimate without a load cell) |
| `free_a` | from params | 0..10 | wiper free-running current |
| `amps_per_volt` | 12.5 | 1..200 | BTS7960 IS scale |
| `offset_a` | 0 | 0..5 | current zero |
| `hx_n_per_count` | 1/21000 | | load cell scale |
| `drum_dps` | 60 | 5..720 | drum speed estimate (also learned at every detent-to-detent move) |
| `aim_tol` | 0.75 | 0.1..5 | AIMED tolerance, deg |
| `aim_offset` | 0 | -20..20 | pitch offset (set by `cal aim`) |
| `aim_home_deg` | 0 | -720..720 | aim-train angle where `SW_RAIL_LOW` closes |
| `line_m` | | 0..100 | line out (tell the base after a shot) |
| `trip_a` | 17 | 3..19 | wiper over-current trip |
| `cock_trip_a` | 15 | 3..19 | trip while cocking |
| `vbat_low` | 9.9 | 6..12 | BROWNOUT threshold under load |
| `require_seated` | true | true/false | fire needs `SW_SEATED` |
| `enc_sign` | 1 | 1 / -1 | AS5600 count direction (set while stopped) |
| `fw`, `params`, `pitch_range` | | get only | firmware version, params.py version, `min` / `max` pitch |

## 6. Faults and safety

Fault codes: `STALL`, `OVERCURRENT`, `DRUM_TIMEOUT`, `DRUM_SENSOR`, `ANGLE_SENSOR`, `SWITCH_DISAGREE`, `BROWNOUT`,
`BAD_STATE`. The first fault latches (later ones are ignored until `clear`).

On any fault the wiper stops at once and the drum goes to NEUTRAL (state `SAFING`, then `FAULT`, `MODE` event),
EXCEPT when that would be unsafe; then the drum holds where it is and a `MODE` event with `err:"DRUM_HELD"` says why:

- a drum fault (`DRUM_TIMEOUT`, `DRUM_SENSOR`) or an impossible drum code: the drum may be jammed or its position wrong;
- drum position or wraps unknown;
- wraps on: the capstan holds the line (sprag #2 backstop), and unwrapping under load is not safe;
- LAUNCH with the shot phase not `REST`: there is no sear, the drawn spring is held only by the wiper worm through the
  dog collar; shifting under that load could dry fire or jam the dog. Recover with `clear`, then `cal decock` or `fire`.

When each fault is raised:

| code | cause |
|---|---|
| `OVERCURRENT` | wiper current over `trip_a` (`cock_trip_a` while cocking) for 2 ticks (10 ms). During `reel until tension` it holds instead |
| `STALL` | driving above 30 % duty after the ramp but the shaft turned slower than 15 deg/s for 400 ms. During `reel until tension` it holds instead |
| `DRUM_TIMEOUT` | the drum did not leave its detent, or did not reach the target in expected time x 2 + 1.5 s (after one retry with a wiper nudge for dog teeth meeting tooth on tooth) |
| `DRUM_SENSOR` | a detent code outside the move appeared, the target code came far too early, the code was lost after arriving, or the wrap index was not seen |
| `ANGLE_SENSOR` | AS5600 lost (4 bad reads or no magnet) while the wiper runs |
| `SWITCH_DISAGREE` | an impossible drum code for 100 ms, a drum code contact chattering during a drum move, `SW_HOME` opening at the wrong angle |
| `BROWNOUT` | battery under `vbat_low` for 300 ms while a motor runs |
| `BAD_STATE` | the mechanism did not do what the angle says (no release, carriage not drawn, a home contact never seen, an action that took too long) |

## 7. Power loss and the saved state

The base saves its state to flash when everything has stopped, and marks it "moving" at every motion start. At the
next start (`BOOT` event):

- drum: known from the drum code if it shows a detent; else from the saved state if the drum was not moving; else
  `UNKNOWN` (`cal drum`).
- wiper: the AS5600 angle inside one turn is checked against the saved angle; if the wiper was moving or the shaft
  was turned by hand (over 3 deg), the aim becomes unknown (re-homed automatically by the next `aim`) and the shot
  phase becomes `UNKNOWN`.
- shot phase `UNKNOWN`: carriage home (`SW_HOME` closed): `cock` creeps until `SW_HOME` opens and continues. Carriage
  drawn: `cal decock`, then `cock`.
- a stop mid-cycle is never trusted: cocking, firing or resetting at power loss gives `UNKNOWN`.

## 8. Typical mission

```
{"id":1,"cmd":"mode","to":"LAUNCH"}        -> MODE ok
{"id":2,"cmd":"aim","deg":35}              -> AIMED ok
{"id":3,"cmd":"cock"}                      -> COCKED
{"id":4,"cmd":"arm"}                       <- {"id":4,"ok":true,"token":482913,"valid_ms":3000}
{"id":5,"cmd":"fire","token":482913}       -> FIRED, RESET
{"id":6,"cmd":"set","key":"line_m","val":8.2}
{"id":7,"cmd":"mode","to":"REEL"}          -> MODE ok
{"id":8,"cmd":"wrap"}                      -> WRAPPED
{"id":9,"cmd":"reel","duty":1,"max_n":40,"until":"tension"}   -> LIMIT holding (tow)
{"id":10,"cmd":"stop"}                     (dart releases)
{"id":11,"cmd":"reel","duty":0.8,"max_n":30,"until":"seated"} -> SEATED
{"id":12,"cmd":"unwrap"}                   -> UNWRAPPED
{"id":13,"cmd":"mode","to":"LAUNCH"}       -> MODE ok
```
