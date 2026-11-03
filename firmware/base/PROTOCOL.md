# GrappStar base station protocol (contract C1)

Firmware `base-1.2.1` (`firmware/base/grappstar_base`, the base-1.2 line), mechanism BASE_SPEC v1.2 (with the C1 sequence of section 10.1, D35) and `base/spec/params.py` 1.2.1 (as built into `params_gen.h`) (drum
order LAUNCH / NEUTRAL / WRAP / REEL with the WRAPPED park inside N_TO_R, decisions D16, D22 and D31; mission order
D22). This file is the contract the Jetson implements against. Changes are additive where possible (new fields, new
events, new error codes); section 10 lists everything the Jetson worker must change for base-1.2.

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
  Events wait in a queue of 24 lines; when the queue is full the newest event is dropped, and `status` field `ev_drops` counts it.
  Telemetry is never queued: a telemetry line that does not fit the USB buffer is dropped. So read the port
  continuously.
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
- Long actions (`mode`, `aim`, `cock`, `fire`, `wrap`, `unwrap`, `reel`, `tidy`, `retrieve`, `decock` and
  `cal decock`, `estop`) reply `ok:true` as soon as they START. Completion is an event (section 4). `jog` ends silently
  when its time runs out (watch `state`). A refusal replies `ok:false` with `err` and nothing moves.
- "Already there" cases reply `ok:true` and send the completion event at once (listed per command).
- Before any motor starts, the base writes a "moving" record to flash (a few tens of ms): motion begins slightly after
  the reply.
- While an action runs, every other action is refused with `BUSY`. `ping`, `status`, `get`, `set`, `stop`, `estop`,
  `clear` and most `cal` actions always answer (`set` and `cal` only after the first 60 ms: `BOOTING`).

### Common refusals

Checked first by every action that moves something (in this order):

| err | meaning |
|---|---|
| `FAULTED` | a fault is latched (or the drum is still on its way to its safe park after one): `clear` first |
| `BUSY` | another action runs |
| `BOOTING` | the first 60 ms after start-up (until `BOOT`). Also returned by `set` and by every `cal` except `cal drum` in that time |
| `NO_POWER` | battery under 6 V (unplugged, USB only). Only while `vbat_wired` is true |
| `ANGLE_SENSOR` | the AS5600 is not reading. Only for actions that turn the wiper (`aim`, `cock`, `arm`, `fire`, `reel`, `tidy`, `retrieve`, `unwrap`, wiper `jog`, `decock` / `cal decock`, and `mode` / `wrap` when they need the centering jog or the fly-in; see each command) |
| `HOLDING` | the reel is holding a tension (state `HOLD`): `stop` first. `reel` and `retrieve` are allowed and wind again |

### Command list

| cmd | args | does | refusals (besides the common ones) |
|---|---|---|---|
| `ping` | | reply carries `fw` and `t` (ms since boot) | none |
| `status` | | reply carries `phase` (section 3), `aimed`, `aim_known`, `drum_known`, `wraps_known`, `magnet` (`OK`, `WEAK`, `STRONG`, `NONE`, `INDEX`), `glitches` (AS5600 rejected samples, capped at 99999), `hx` (load cell live), `peak_a` (highest wiper current of the last run), `fw`, `ev_drops` (events dropped because the host did not read, capped at 99999), and while a fault is latched `why` (first 32 characters of the fault reason). These are flat fields of the reply, not a `tel` object | none |
| `stop` | | wiper off now, drum stops where it is (both worms hold). Aborts the running action with an `ABORTED` event. A stop during `cock`, `fire` or `decock` makes the shot phase `UNKNOWN`. A stop while faulted (also during `SAFING`) stops the drum and stays in `FAULT` | never refused |
| `estop` | | like `stop`, then (when not faulted) the drum goes to its safe park if that is safe (section 6: the WRAPPED park 314 from REEL or N_TO_R, else NEUTRAL 41): state `MODE`, a `MODE` event at the end, immediately if already there. If it is not safe the drum holds and a `MODE` event with `ok:false`, `err:"DRUM_HELD"` and `why` says why. The estop move never turns the wiper (no centering jog). A failed estop move raises a drum fault | never refused |
| `clear` | | clears a latched fault, back to `IDLE`. Not faulted: `ok`, no change | `BUSY` (still `SAFING`), `STILL_PRESENT` (the cause is still there: AS5600 not reading after `ANGLE_SENSOR`, drum code invalid after `DRUM_SENSOR`, battery under `vbat_low` after `BROWNOUT`) |
| `mode` | `to`: `LAUNCH` \| `NEUTRAL` \| `REEL` | moves the drum to that park (LAUNCH 0, NEUTRAL 41, REEL 347 drum deg) and verifies the collar (section 5). Leaving LAUNCH starts with the centering jog. `mode REEL` only runs from N_TO_R (`drum_pos` `WRAPPED`, normally the 314 park after `wrap`): the drum turns forward into REEL with the fly-in. Already there (the sector, the angle within 3 deg, and the collar confirmed: LAUNCH `SW_DOG_L` closed, REEL `SW_DOG_R` closed, NEUTRAL both open): `ok` and an immediate `MODE` event; otherwise the move runs and confirms the collar at the end. Ends with a `MODE` event. LAUNCH to NEUTRAL or back about 5 s, WRAPPED to REEL about 5 s | `BAD_ARG`, `POSITION_UNKNOWN` (drum code invalid), `HAND_MODE` (`SW_N20` open: hand lever latched out), `ANGLE_SENSOR` (AS5600 not reading and the move needs the wiper: `REEL` asked (fly-in), or leaving LAUNCH or REEL (centering jog)), `NOT_WRAPPED` (`REEL` asked below N_TO_R: use `wrap` first), `WRAPS_ON` (`LAUNCH` or `NEUTRAL` asked from WRAP, N_TO_R or REEL: use `unwrap`), `LAUNCH_BUSY` (leaving LAUNCH needs the launcher at rest, section 6), `BUSY` (wiper still running) |
| `aim` | `deg`: rail pitch | turns the wiper BACKWARD (trip 5 A) until the pitch (from the aim table) is within `aim_tol` of `deg`. If the aim is unknown (always after a shift into LAUNCH, after a power cut, after a decock) it first runs backward until `SW_RAIL_LOW` closes (home), then to the target: up to about 1.25 aim cycles (10 wiper turns per cycle), so allow 60 s. Up to 3 tries. `AIMED` event | `NOT_IN_LAUNCH` (collar not confirmed in LAUNCH, or the drum not stopped within 6 deg of the LAUNCH park), `PHASE_UNKNOWN`, `NOT_AT_REST` (aim only with the launcher at rest: backward drive with the spring drawn would let it unwind), `OUT_OF_RANGE` (see `get pitch_range`), `AIM_UNKNOWN` (aim unknown and no `SW_RAIL_LOW` fitted) |
| `cock` | | optional. Turns the wiper FORWARD (trip 6.5 A) from rest to the cocked angle (carriage drawn, short of the release tooth). `SW_HOME` must open at the first tooth. `COCKED` event. With the phase `UNKNOWN` and the carriage home it first creeps until `SW_HOME` opens (that re-zeroes the angle), then cocks. Already cocked (`SW_HOME` open): `ok` and an immediate `COCKED` event. Aim first: see the note under section 8 | `NOT_IN_LAUNCH`, `NO_SENSOR` (no `SW_HOME` fitted; or no `SW_SEATED` while `require_seated`), `SWITCH_DISAGREE` (phase `COCKED` but `SW_HOME` closed, or `REST` but `SW_HOME` open), `LOADED_UNKNOWN` (carriage drawn with the angle unknown, or `SW_HOME` closed while the last known angle says drawn: `decock` first), `NOT_SEATED` (`SW_SEATED` open while `require_seated`; checked after the phase checks, so an already cocked launcher still answers `COCKED`) |
| `arm` | | checks everything `fire` needs and replies `"token":<6-digit integer>,"valid_ms":3000`. A new `arm` replaces the old token. `mode`, `aim`, `cock`, `wrap`, `jog`, `decock`, `stop`, `estop` and any fault cancel it | `NOT_IN_LAUNCH`, `WRAPS_ON` (wraps not 0 or unknown), `NO_SENSOR` (no `SW_HOME`, or no `SW_SEATED` while `require_seated`), `NOT_COCKED` (neither `COCKED` with `SW_HOME` open and `SW_GAP` open, nor `REST` with `SW_HOME` closed and `SW_GAP` closed), `NOT_AIMED` (not aimed since the last shift, shot or aim-train move, or the pitch is more than `aim_tol` off the target), `NOT_SEATED` |
| `fire` | `token`: the integer from `arm` (a numeric string is accepted) | checks the token, then everything again, then runs the wiper FORWARD (trip 6.5 A up to the release, 5 A for the reset): from `COCKED` straight past the release tooth (about 1 s to `RESET`), from `REST` the whole cycle in one run (draw, release, reset: about 3 s). `FIRED` when `SW_HOME` closes (carriage released), `RESET` when the big gear is back in its gap with `SW_HOME` and `SW_GAP` closed. The token is single use; a wrong token cancels the arm | `BAD_ARG` (no `token`), `NOT_ARMED`, `TOKEN_EXPIRED` (over 3 s), `BAD_TOKEN`, then the common refusals and the `arm` refusals |
| `wrap` | | from LAUNCH (D22: straight after the shot; the same checks as leaving LAUNCH, then the centering jog), L_TO_N, NEUTRAL, WRAP_PREP or inside WRAP: drum forward through the WRAP window (62 to 302: the cup lays 7 turns on the capstan with the collar in NEUTRAL and the wiper off) to the WRAPPED park 314 in N_TO_R (cup locked, collar still NEUTRAL). `SW_ARM` must pulse 7 times across the window. `WRAPPED` event at the park; no `MODE` event follows. About 35 s from NEUTRAL, 40 s from LAUNCH: use a timeout of 90 s. Already in N_TO_R or REEL: `ok` and an immediate `WRAPPED` (nothing moves) | `POSITION_UNKNOWN`, `LAUNCH_BUSY` (launcher not at rest, or `SW_GAP` open, when leaving LAUNCH or L_TO_N; or the spring known drawn), `HAND_MODE`, `BUSY` (wiper running), `ANGLE_SENSOR` (from LAUNCH without the AS5600: the centering jog needs it) |
| `unwrap` | | from REEL, N_TO_R (the WRAPPED park included), WRAP or WRAP_PREP: from REEL first the centering jog (forward at 20 % until the 16 deg dog window is taken up or 3 s, then 8 deg back), then the drum backward to NEUTRAL 41. While the drum is in WRAP and WRAP_PREP the wiper turns forward at 40 % (take-up only, trip 2.5 A: the capstan is not driven, the take-up winds the freed line). After a full pass `SW_ARM` must have pulsed 7 times. `UNWRAPPED` event, then `MODE` (NEUTRAL). About 40 s from REEL. Safe at any wrap count (also after a power cut mid-wrap). Already at NEUTRAL: `ok` and an immediate `UNWRAPPED` | `POSITION_UNKNOWN`, `NOT_WRAPPED` (drum in LAUNCH or L_TO_N), `BUSY` (wiper running), `ANGLE_SENSOR` (the take-up needs the wiper), `HAND_MODE`, `LAUNCH_BUSY` (spring known drawn) |
| `tidy` | `ms` (default 3000 = config `tidyMs`, over 0 up to 60000), `duty` (default 0.3) | D22 tidy by time: take-up only, wiper FORWARD for `ms` (trip 2.5 A). Allowed at the WRAPPED park (N_TO_R below 318) or in NEUTRAL, both dog contacts open. `duty` over 0 up to 1; below 0.27 it runs at 0.27, above 0.40 it is capped at 0.40. `TIDIED` event at the end. Telemetry `state` is `REEL` meanwhile; `stop` ends it with `ABORTED` `what:"REEL"` | `BAD_ARG` (`ms` out of range, bad `duty`), `NOT_IN_NEUTRAL` (anywhere else, or a dog contact closed), `POSITION_UNKNOWN` (drum code invalid). These come before the common refusals |
| `retrieve` | `duty` (default 0.3), `max_n` (default `limit_n`) | final approach on the take-up: the same as `reel` with `until:"seated"`. In NEUTRAL (or at the WRAPPED park, below 318 deg) the take-up pulls the dart in (trip 2.5 A, fault `DART_STUCK` after 20 s without `SW_SEATED`). Never on the capstan (D35): in REEL it is refused. `SEATED` event. Already seated: `ok` and an immediate `SEATED` | as `reel`, so in REEL `WRAPS_ON` (`unwrap` first) |
| `decock` | `force` (optional) | the same as `cal decock` (section 7) | as `cal decock` |
| `reel` | `duty` over 0 up to 1 (default 0.86; below 0.27 it runs at 0.27), `max_n` (default `limit_n`, capped at 110), `until`: `stop` (default) \| `seated` \| `tension` \| `slip` | REEL (collar confirmed): the capstan winds in (tow, reel in), see below. NEUTRAL or the WRAPPED park (N_TO_R below 318), both dog contacts open: take-up only, see below. Also accepted while holding (state `HOLD`): winds again | `BAD_ARG` (bad `until`, `duty` not in (0, 1], `max_n` not over 0, `slip` in REEL), `NOT_WRAPPED` (neither REEL nor take-up position, or `tension` without the capstan), `WRAPS_ON` (`seated` in REEL: never seat the dart on the capstan, D35), `NO_SENSOR` (`seated` without `SW_SEATED`) |
| `jog` | `motor`: `wiper` \| `drum`, `dir`: 1 \| -1, `ms` (default 300, capped at 1500) | runs one motor for `ms`. Deadman: the same jog (same motor and dir) sent again before it ends extends it. Wiper dir 1 = FORWARD, drum dir 1 = toward REEL. Ends silently (state back to `IDLE`), see below for the limits | see below |
| `cal` | `what`, see section 7 | calibration actions | per action (section 7), `BAD_ARG` for an unknown `what` |
| `set` | `key`, `val` (number or `true` / `false`) | sets a value (section 7) | `BAD_ARG` (no key or val), `BAD_KEY`, `OUT_OF_RANGE`, for `enc_sign`: `BAD_ARG` unless 1 or -1, `BUSY` while moving |
| `get` | `key` | reply carries `val` (or `min` and `max` for `pitch_range`) | `BAD_ARG` (no key), `BAD_KEY` |

#### `reel` in detail

In REEL (capstan, wiper FORWARD):

- Trip: `tow_trip_a` (6.5 A) with `until:"tension"` (the tow: a 50 N tow draws about 5.1 A, so the tension estimate
  is the real limit), `trip_a` (5 A) with any other `until`.
- Tension = load cell when one is live, else the estimate
  `k_n_per_a x (amps - free_a) + tension_offset_n` (defaults 17.0 and -10.4: physics v1.2 with the clutch FIRM at 5 N;
  never below 0, averaged over about 200 ms (D33); the first 350 ms of a run are ignored). The speed tapers over the last 30 % of `max_n`.
- Reaching `max_n` stops the wiper and sends `LIMIT`. With `until:"tension"` the base then HOLDS (state `HOLD`,
  `holding:true`, the worm and sprag #2 hold the line); with a load cell it winds again by itself when the tension
  falls under 60 % of `max_n`. With any other `until` it goes to `IDLE` (`holding:false`).
- `until:"seated"` is refused in REEL with `WRAPS_ON` (D35: the dart is never seated on the capstan; reel in to
  about 1.5 m, `unwrap`, then `retrieve` on the take-up).
- `until:"stop"`: runs until `stop` (or the limit). The Jetson stops it at about 1.5 m from the line count or the
  camera.
- After 240 s any reel stops with `ABORTED` (`what:"REEL"`, `why:"timeout"`).
- An over-current or stall during a tow is a fault, never a hold.

In NEUTRAL or at the WRAPPED park (take-up only, decision D15 / lens B I4: wiper FORWARD at most 40 % duty, trip
2.5 A; the capstan is not driven, so there is no tension limit and `line_m` does not change):

- `until:"seated"` (or `retrieve`): the final approach. Stops at `SW_SEATED` (`SEATED` event, `line_m` set to 0).
  Already seated: `ok` and an immediate `SEATED`. No `SW_SEATED` within 20 s:
  fault `DART_STUCK`.
- `until:"slip"`: an older tidy, kept but no longer used by the mission (use `tidy`). Runs until the current has been
  steady (within 0.3 A) for 2 s: `LIMIT` with `tension_n:null`, `why:"tidy done (take-up slipping)"`,
  `holding:false`. Not done within 15 s: `ABORTED` (`what:"REEL"`, `why:"timeout"`).
- `until:"stop"`: runs until `stop` (240 s limit).
- `until:"tension"` is refused (`NOT_WRAPPED`).

#### `jog` in detail

Wiper jog (about 60 % duty, trip 3 A; forward in NEUTRAL, WRAP_PREP or WRAP is take-up only: at most 40 % duty, trip
2.5 A; forward in LAUNCH uses the cock trip 6.5 A):

| err | when |
|---|---|
| `BUSY` | the drum is running |
| `SHIFTING` | the drum is in L_TO_N or N_TO_R (dogs half engaged; this includes the WRAPPED park: use `tidy` there), or its code is invalid |
| `SWITCH_DISAGREE` | both dog contacts closed, or `SW_DOG_L` closed while the drum code says NEUTRAL, WRAP_PREP or WRAP |
| `NOT_IN_LAUNCH` | drum in the LAUNCH sector (or `SW_DOG_L` closed) but not at the park with the collar confirmed |
| `LAUNCH_BUSY` | the spring is known drawn but the drum code is not LAUNCH |
| `WOULD_FIRE` | forward in LAUNCH with the phase `COCKED`, `UNKNOWN`, `FIRING` or `RESETTING` |
| `NO_SENSOR` | forward in LAUNCH without `SW_HOME` fitted |
| `NOT_AT_REST` | backward in LAUNCH with the phase not `REST` (the drawn spring would unwind: use `decock`) |
| `BACKSTOP` | backward in REEL (or with `SW_DOG_R` closed): sprag #2 holds the capstan |

A forward jog in LAUNCH from rest draws the carriage (phase `COCKING` once `SW_HOME` opens) and stops itself short of
the release tooth with `ABORTED` (`what:"JOG"`, `why:"would pass the release tooth"`). A backward jog in LAUNCH turns
the aim train and clears `aimed`.

Drum jog (full speed, no centering jog, no checks of the target, no dog contact check):

| err | when |
|---|---|
| `BUSY` | the wiper is running |
| `POSITION_UNKNOWN` | drum code invalid |
| `HAND_MODE` | `SW_N20` open |
| `LAUNCH_BUSY` | drum in LAUNCH or L_TO_N (or the spring known drawn) and the launcher not at rest; dir -1 inside LAUNCH is allowed |
| `WRAPS_ON` | dir -1 in REEL (leaving REEL needs the centering jog: use `unwrap`) |

A drum jog stops by itself when the angle estimate reaches 4 deg past the LAUNCH park (-4 deg, which is the end stop
itself) or the REEL park (351 deg) (`ABORTED`, `what:"JOG"`, `why:"drum end stop"`). Any jog clears `aimed` (drum)
and cancels an arm token.

## 3. Telemetry

`{"tel":{...}}` at 10 Hz, 20 Hz while anything moves. Field order is fixed as below.

| field | type | meaning |
|---|---|---|
| `state` | string | `IDLE`, `MODE` (drum moving for `mode` or `estop`), `AIM`, `COCK`, `FIRE`, `WRAP`, `UNWRAP`, `REEL` (also `tidy` and `retrieve`), `HOLD`, `JOG`, `DECOCK`, `SAFING` (fault: drum on its way to its safe park), `FAULT`. (`CAL` exists in the code but is never used) |
| `mode` | string | what the collar really does: `LAUNCH` (code LAUNCH, `SW_DOG_L` closed, `SW_DOG_R` open), `REEL` (code REEL, `SW_DOG_R` closed, `SW_DOG_L` open), `NEUTRAL` (code NEUTRAL, WRAP_PREP, WRAP, or N_TO_R below 318 deg, both dog contacts open), else `NONE` (shifting, N_TO_R from 318, code invalid, contacts disagree) |
| `drum_deg` | number \| null | drum angle estimate, deg (parks LAUNCH 0, NEUTRAL 41, WRAPPED 314, REEL 347; WRAP window 62 to 302). Snapped at every code edge, dead reckoning in between, never outside the sector the code shows. Null when the code was never valid |
| `drum_pos` | string | from the drum code sector: `LAUNCH`, `NEUTRAL`, `REEL`, `WRAPPED` (the whole N_TO_R sector 302 to 340: wraps on, collar NEUTRAL below 318, then ramping; the park is 314), `BETWEEN` (L_TO_N, WRAP_PREP, WRAP), `UNKNOWN` (code invalid) |
| `wiper_deg` | number \| null | wiper output shaft angle, multi-turn, FORWARD positive. Null until the AS5600 has read once |
| `rpm` | number \| null | wiper output rpm, signed; null while the AS5600 is not reading |
| `amps` | number | wiper current (A) |
| `pitch_deg` | number \| null | rail pitch from the aim table; null while the aim is unknown |
| `cock` | number \| null | 0 at rest, 1 cocked (and while firing), between while cocking or resetting; null when the phase is `UNKNOWN` |
| `wraps` | integer \| null | capstan wraps: 0 from LAUNCH to WRAP_PREP, an estimate from the drum angle in WRAP, 7 in N_TO_R and REEL. Null when unknown (power cut inside WRAP) |
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

Examples (from the simulation: aimed in LAUNCH, halfway through a wrap, at the WRAPPED park, in REEL):

```
{"tel":{"state":"IDLE","mode":"LAUNCH","drum_deg":0,"drum_pos":"LAUNCH","wiper_deg":-4707.3,"rpm":-2.9,"amps":0,"pitch_deg":34.97,"cock":0,"wraps":0,"line_m":null,"tension_n":null,"sw":{"a":1,"l":1,"n":1,"H":1,"G":1,"S":1,"W":1},"fault":null,"vbat":12.4}}
{"tel":{"state":"WRAP","mode":"NEUTRAL","drum_deg":193.1,"drum_pos":"BETWEEN","wiper_deg":-4715.7,"rpm":0,"amps":0,"pitch_deg":35.34,"cock":0,"wraps":3,"line_m":null,"tension_n":null,"sw":{"c":1,"n":1,"H":1,"G":1},"fault":null,"vbat":12.4}}
{"tel":{"state":"IDLE","mode":"NEUTRAL","drum_deg":314,"drum_pos":"WRAPPED","wiper_deg":-4715.7,"rpm":0,"amps":0,"pitch_deg":35.34,"cock":0,"wraps":7,"line_m":null,"tension_n":null,"sw":{"a":1,"c":1,"n":1,"H":1,"G":1,"W":1},"fault":null,"vbat":12.4}}
{"tel":{"state":"IDLE","mode":"REEL","drum_deg":347,"drum_pos":"REEL","wiper_deg":-4333,"rpm":7,"amps":0,"pitch_deg":35.3,"cock":0,"wraps":7,"line_m":4.3,"sw":{"a":1,"b":1,"c":1,"r":1,"n":1,"H":1,"G":1,"W":1},"vbat":12.3,"lean":true}}
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
| `HELLO` | `fw` (`base-1.2.1`), `reset` (`POWERON`, `BROWNOUT`, `PANIC`, `WATCHDOG`, `SOFTWARE`, `USB`, `EXTERNAL`, `DEEPSLEEP`, `OTHER`), `hx711`, `as5600`, `pwm` (false: a PWM pin failed and the wiper bridge stays disabled: do not run) | once at start-up. Treat it as a new session |
| `BOOT` | `fw`, `record` (a saved state was found), `was_moving` (power was lost mid-motion), `drum_known`, `wraps_known`, `aim_known`, `phase`, `angle` (`AS5600`, `INDEX` or `NONE`: which angle source the base uses) | 60 ms after start-up, once the sensors settled (section 9) |
| `MODE` | `ok`, `mode`, `drum_pos`; with `ok:false` also `err` (`DRUM_HELD`, or the fault code of a failed drum move after a fault) and `why` (text) | a drum move ended (`mode`; `unwrap` after its `UNWRAPPED`; `estop`; safing after a fault); also right after `estop` or a fault when the drum is already at its safe park; and as the reply event of `cal drum`. NOT sent after `wrap`. `err:"DRUM_HELD"`: the drum deliberately did not move to its safe park |
| `AIMED` | `ok`, `pitch_deg` (null if unknown), `target_deg`; `err` (`AIM_MISS` after 3 tries, `OUT_OF_RANGE`) when `ok:false` | aim ended |
| `COCKED` | `ok`, `cock_deg` (wiper deg forward from rest) | cocked |
| `FIRED` | `pitch_deg`, `cock_deg` (wiper deg of the release) | the carriage was released (`SW_HOME` closed). `aimed` is cleared and `line_m` becomes null |
| `RESET` | `ok`; `how` (`decock` or `decock by travel`) after a decock; with `ok:false` also `why` | the launcher is back at rest after a shot or a decock. `ok:false`, `how:"decock"`, `why:"carriage home: nothing to decock, cock finds the angle"`: a decock with the carriage home and the phase `UNKNOWN` (nothing moved) |
| `WRAPPED` | `ok`, `wraps` | `wrap` reached the WRAPPED park (314, collar NEUTRAL), or at once when the drum is already in N_TO_R or REEL. No `MODE` follows |
| `TIDIED` | `ok` | `tidy` ran its time |
| `UNWRAPPED` | `ok` | back at NEUTRAL with no wraps (followed by `MODE`) |
| `SEATED` | | `SW_SEATED` closed during `reel until seated` or `retrieve` (`line_m` set to 0) |
| `LIMIT` | `tension_n` (null for the slip tidy), `max_n`, `why` (`tension limit` or `tidy done (take-up slipping)`), `holding` | the reel reached `max_n` (holding when `until:"tension"`), or the slip tidy finished |
| `STALL` | | the wiper stalled (sent right after the `FAULT` with code `STALL`) |
| `FAULT` | `code`, `why` (up to 47 characters) | a fault latched (section 6) |
| `ABORTED` | `what` (the state that was running: `MODE`, `AIM`, `COCK`, `FIRE`, `WRAP`, `UNWRAP`, `REEL` (also `tidy` and `retrieve`), `HOLD`, `JOG`, `DECOCK`; or `HOST`), `why` (`stop`, `estop`, `timeout`, `would pass the release tooth`, `drum end stop`, `host silent`) | an action ended without completing. Treat it as the failure of whatever you were waiting for (a `stop` you sent yourself also gives one). The host watchdog sends two: the stopped state with `why:"stop"`, then `what:"HOST"` |
| `OVERFLOW` | (no `t`) | an event did not fit 256 bytes (should never happen) |

Example sequences (simulation):

- `wrap` straight from LAUNCH after a shot: reply `{"id":7,"ok":true}`, about 42 s later
  `{"evt":"WRAPPED","t":81395,"ok":true,"wraps":7}` and nothing else.
- `mode REEL` from the WRAPPED park: about 4.3 s later `{"evt":"MODE","t":88695,"ok":true,"mode":"REEL","drum_pos":"REEL"}`.
- A fault in REEL: `{"evt":"FAULT",...,"code":"STALL","why":"wiper stalled"}`, `{"evt":"STALL",...}`, about 4.5 s
  later `{"evt":"MODE",...,"ok":true,"mode":"NEUTRAL","drum_pos":"WRAPPED"}`.

## 5. Drum, collar and the shift sequence

Drum sectors in travel order (drum dir 1 = toward REEL), from params.DRUM (BASE_SPEC v1.2 section 7). Code bits A B C,
1 = contact closed. Adjacent sectors differ by one bit (Gray code); 000 never occurs on a working drum.

| sector | drum deg | code A B C | `sw` | collar | `drum_pos` |
|---|---|---|---|---|---|
| LAUNCH | -4 to 8 (park 0) | 1 0 0 | `a` | LAUNCH | `LAUNCH` |
| L_TO_N | 8 to 32 | 1 1 0 | `a` `b` | shifting | `BETWEEN` |
| NEUTRAL | 32 to 50 (park 41) | 0 1 0 | `b` | NEUTRAL | `NEUTRAL` |
| WRAP_PREP | 50 to 62 | 0 1 1 | `b` `c` | NEUTRAL, cup lock releases at 60 | `BETWEEN` |
| WRAP | 62 to 302 | 0 0 1 | `c` | NEUTRAL, cup lays 7 turns | `BETWEEN` |
| N_TO_R | 302 to 340 (WRAPPED park 314) | 1 0 1 | `a` `c` | NEUTRAL until 318, then ramp to REEL; cup locked | `WRAPPED` |
| REEL | 340 to 352 (park 347) | 1 1 1 | `a` `b` `c` | REEL (wraps on) | `REEL` |

End stops at -4 and 352 deg. The drum turns at about 8.1 deg/s (`drum_dps`, re-learned between every two code
edges): NEUTRAL to the WRAPPED park takes about 34 s.

How a drum move runs (for `mode`, `wrap`, `unwrap`, `estop` and safing):

1. Checks (section 2 and 6). The record "moving" goes to flash.
2. Centering jog, only when leaving an engaged collar (LAUNCH or REEL), not for `estop` or safing (`mode`, `wrap` and
   `unwrap` refuse with `ANGLE_SENSOR` when it would be needed and the AS5600 is not reading): in LAUNCH the wiper turns 8 deg BACKWARD (half the 16 deg dog window, so the collar touches no flank;
   never forward, that would cock). In REEL it first turns FORWARD at 20 % until the 16 deg window is taken up (or
   3 s), then 8 deg back (sprag #2 holds the line meanwhile). Trip 3 A. The 8 deg leg is also capped in time (1.5 x
   its free-speed time), so a noisy angle cannot carry it past the window.
3. The N20 runs to the target sector's code edge, then on by time to the park.
4. During the run `SW_DOG_L` must stay open once the drum is past L_TO_N, and `SW_DOG_R` must stay open until N_TO_R
   (so the WRAP window is never entered with either closed); otherwise `SWITCH_DISAGREE` "collar contact disagrees
   with the drum code". While unwrapping, the wiper turns forward at 40 % in WRAP and WRAP_PREP (take-up, trip
   2.5 A). Toward REEL the fly-in starts at 318 deg: wiper forward at 20 % (trip 3 A).
5. At the park the dog contacts must confirm the collar: REEL needs `SW_DOG_R` (fly-in continues for at most 120 deg
   of wiper or 6 s); LAUNCH needs `SW_DOG_L` (if not yet, the wiper is turned BACKWARD at 20 % for at most 40 deg,
   never forward; with the spring possibly drawn this is a fault instead); NEUTRAL and the WRAPPED park need both
   open after 100 ms. After arriving in LAUNCH the aim is forgotten (`aim_known` false): the next `aim` re-homes.
6. The end event: `MODE` (`mode`, `estop`, safing), `WRAPPED` (`wrap`), `UNWRAPPED` then `MODE` (`unwrap`).

Checks against the clock: no code edge for 1.5 x the time the current sector should take plus 1.5 s (LAUNCH and REEL
count as 20 deg), or the whole move over 1.5 x the expected time plus 1.5 s: `DRUM_TIMEOUT`. The target code arriving
in under a quarter of the expected time to the target code's edge (only when that time is over 1.5 s, and only when
the start angle is trusted, that is a code edge was seen or a saved angle agreed at boot): `DRUM_SENSOR` "target code
came far too early".

## 6. Faults and safety

### Interlocks (lens B 7.1, as the firmware applies them)

- The spring may be drawn ("loaded") whenever the phase is not `REST`, the angle says past the first tooth, or
  `SW_HOME` is open. Leaving LAUNCH (by `mode`, `wrap` or a drum `jog`) needs the launcher not loaded and `SW_GAP`
  closed; this is checked whenever the drum code says LAUNCH or L_TO_N, the code is invalid (`mode`), or `SW_DOG_L` is
  closed, so a failed code bit cannot open the way. With the phase known loaded (`COCKING`, `COCKED`, `FIRING`) every
  drum move is refused (`LAUNCH_BUSY`).
- `aim`, `cock`, `fire` and `decock` need the collar in LAUNCH all the time; the collar leaving LAUNCH during one is
  `BAD_STATE`.
- Aim only at rest. Fire needs LAUNCH, the right `SW_HOME` / `SW_GAP` state, `SW_SEATED` (while `require_seated`), a
  fresh aim and a valid token.
- Wiper backward is refused in REEL (`BACKSTOP`); forward in NEUTRAL, WRAP_PREP, WRAP and at the WRAPPED park is
  take-up only (40 %).
- The drum never moves while `SW_N20` is open (hand mode); the N20 latching out during a move is a fault.
- `SW_DOG_L` and `SW_DOG_R` are checked during every drum move (section 5, step 4).
- No `mode LAUNCH` or `mode NEUTRAL` with wraps on (use `unwrap`); `mode REEL` only from N_TO_R (after `wrap`).

### Current and stall protection (decision D8)

| state | trip |
|---|---|
| aim, reel in (`reel` until `stop` in REEL), decock, the reset after a release (`trip_a`) | 5 A |
| cock, fire up to the release, forward jog in LAUNCH (`cock_trip_a`) | 6.5 A |
| tow (`reel` with `until:"tension"`, `tow_trip_a`). A 50 N tow draws about 5.1 A: the tension estimate is the real tow limit | 6.5 A |
| take-up only (`reel`, `tidy`, `retrieve` in NEUTRAL or at the WRAPPED park, take-up during `unwrap`, forward jog in NEUTRAL, WRAP_PREP or WRAP) | 2.5 A |
| jogs, centering jog, fly-in, the LAUNCH collar wiggle | 3 A |
| any state, never ignored | 17 A |

A trip needs 6 samples in a row (30 ms) over the limit. It is ignored for the first 50 ms after a start (inrush),
except the 17 A limit. Stall: driving above 15 % duty, after the 300 ms soft start, shaft slower than 12 deg/s for
200 ms. Both are always faults (also in a tow). A printed shear pin in the coupler breaks at about 5 N m (D26).

### Fault codes

The first fault latches; later ones are ignored until `clear` (one exception: a second fault during `SAFING` stops the
drum, see below).

| code | causes (`why` texts) |
|---|---|
| `OVERCURRENT` | wiper current over the state trip for 30 ms (`wiper over 6.5 A`: the trip in force, one decimal) |
| `STALL` | `wiper stalled` (also sends a `STALL` event) |
| `DRUM_TIMEOUT` | `drum not turning (no code edge)`, `drum did not reach the target in time`, `dogs did not engage (collar contact open)` |
| `DRUM_SENSOR` | `drum code 000 (wire off or bump missing)` or `drum code jumped over a sector` for 150 ms; `drum turned the wrong way (drum_sign?)` (N20 wires swapped); `target code came far too early`; `drum overshot the target sector`; `drum code changed with the drum still` (the worm cannot be back-driven: a contact is lying, or the drum was turned by hand while the base runs; one `clear` is enough once the drum is still); `N20 latched out during a drum move`; `SW_ARM counted n of 7 cup turns`; `drum code disagrees with the saved drum angle` (at boot) |
| `ANGLE_SENSOR` | `AS5600 lost while the wiper runs` (4 bad reads, or no magnet); `AS5600 counts against the drive (enc_sign?)`; `AS5600 glitching (over 20 bad samples a second)`; in index mode `SW_INDEX bump overdue (index cam or contact)` |
| `SWITCH_DISAGREE` | `SW_DOG_L and SW_DOG_R both closed` (200 ms, or at the end of a move); `collar contact disagrees with the drum code` (during a drum move); `drum code contact chattering` during a drum move; `collar still engaged at NEUTRAL` (also at the WRAPPED park); `SW_HOME opened at the wrong angle`; `SW_HOME still closed past the first tooth`; `SW_HOME chattering during decock` |
| `BROWNOUT` | `battery sagged under load`: under `vbat_low` for 300 ms while a motor runs |
| `BAD_STATE` | the mechanism did not do what the angle says: `collar left LAUNCH during a launch action`, `drum code away from LAUNCH but the spring is drawn`, `LAUNCH collar not confirmed with the spring possibly drawn`, `SW_RAIL_LOW never closed in an aim cycle`, `aim ran too far`, `carriage not drawn (SW_HOME still closed): rack or dog not engaged`, `carriage slipped home while cocking`, `SW_HOME closed at the end of cocking`, `SW_HOME never opened (stuck closed?)`, `release at the wrong angle` (after `FIRED`), `no release seen (SW_HOME did not close)`, `SW_HOME opened while resetting`, `reset ended with the carriage off its stop`, `reset ended with the big gear out of its gap`, `decock: SW_HOME never closed`, `decock: SW_GAP never closed`, and the time limits (`cocking took too long`, `cock recovery took too long`, `fire cycle took too long`, `decock by travel took too long`) |
| `DART_STUCK` | `final approach: no SW_SEATED in 20 s` |

A fault during any launch-train motion makes the shot phase `UNKNOWN` (the last angle is kept as the phase hint,
section 9). Any fault cancels the arm token.

### Fault response

The wiper stops at once. Then the drum goes to its safe park without any wiper motion (state `SAFING`, then `FAULT`,
`MODE` event with `ok:true`):

- from REEL or N_TO_R: the WRAPPED park 314 (collar NEUTRAL, wraps kept). Never NEUTRAL: that would run back through
  the wrap window and unwrap under tow;
- from LAUNCH, L_TO_N, NEUTRAL or WRAP_PREP: NEUTRAL 41.

`estop` uses the same targets (state `MODE` instead of `SAFING`). The drum holds where it is instead (the worm holds
it) when that would be unsafe, and a `MODE` event with `ok:false`, `err:"DRUM_HELD"` and `why` says why:

| `why` | meaning |
|---|---|
| `drum fault` | the fault is `DRUM_TIMEOUT` or `DRUM_SENSOR`: the drum may be jammed or its position wrong |
| `drum code invalid` | |
| `N20 latched out (hand mode)` | |
| `wraps unknown` | |
| `inside the wrap window: the operator decides (unwrap)` | the drum is in WRAP: recover with `clear`, then `unwrap` (or `wrap` to go on to the WRAPPED park) |
| `launch train not at rest: the wiper worm holds the spring (a shift under load could dry fire)` | there is no sear: the drawn spring is held only by the wiper worm through the collar. Recover with `clear`, then `decock` (or `fire` if it is safe to shoot) |

Already at the safe park: an immediate `MODE` with `ok:true`. A second fault during `SAFING` stops the drum and sends
`MODE` with `err:"DRUM_HELD"` and that fault's `why`. A safing move that fails sends `MODE` with `ok:false`, `err` =
the drum fault code and its `why`, and stays in `FAULT`. `stop` during `SAFING` stops the drum (no `MODE` event).

A boot inside WRAP: the base never moves by itself after a start-up. It stops and reports (`BOOT`, `wraps_known`
false unless the saved angle was trusted); the operator decides (section 9).

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
| `decock` (also the command `decock`) | `force` (optional) | lets a drawn carriage down without firing: runs the wiper BACKWARD slowly (47 %, trip 5 A) so the spring unwinds under the worm, until `SW_HOME` has stayed closed for 15 deg more and `SW_GAP` is closed too. Turns the aim train too: the aim becomes unknown. `RESET` event with `how:"decock"`. Carriage already home: phase `REST` gives an immediate `RESET`; phase `UNKNOWN` with no sign of a draw replies `ok`, moves nothing and sends `RESET` with `ok:false`, `how:"decock"`, `why:"carriage home: nothing to decock, cock finds the angle"` (then use `cock` or `cal cock_rest`). `SW_HOME` closed while the last known angle says drawn: refused with `SWITCH_DISAGREE`, unless `force:true`: then it runs back by the travel that angle needs and calls it rest (`how:"decock by travel"`) | common (`ANGLE_SENSOR` included), `NOT_IN_LAUNCH` (drum code not LAUNCH), `NO_SENSOR` (no `SW_HOME`), `SWITCH_DISAGREE` |
| `drum` | | the drum code is absolute: this only reports, with a `MODE` event | `POSITION_UNKNOWN` |

`set` / `get` keys. Saved in flash once everything has stopped, kept over a power cycle and a new upload, except where
the table says otherwise:

| key | default | range | meaning |
|---|---|---|---|
| `limit_n` | 50 (params.REEL) | 1..110 | default `max_n` for `reel` and `retrieve` (the tow limit) |
| `k_n_per_a` | 17.0 | 0.1..200 | line N per amp above `free_a` (tension estimate without a load cell) |
| `tension_offset_n` | -10.4 | -50..50 | line N added to the estimate: `k_n_per_a x (amps - free_a) + tension_offset_n` |
| `free_a` | 1.5 (params.WIPER) | 0..10 | wiper free-running current |
| `amps_per_volt` | 12.5 | 1..200 | BTS7960 IS scale |
| `offset_a` | 0 | 0..5 | current zero (set by `cal current`) |
| `hx_n_per_count` | 0.0000476 | 0.0000001..1 | load cell scale |
| `drum_dps` | 8.1 | 5..720 | drum speed estimate (learned during moves; saved with the next `set` or `cal`) |
| `aim_tol` | 0.75 | 0.1..5 | AIMED tolerance, deg |
| `aim_offset` | 0 | -20..20 | pitch offset (set by `cal aim`) |
| `aim_home_deg` | 3365 | -7200..7200 | aim-train angle (backward wiper deg from the low dead point) where `SW_RAIL_LOW` closes |
| `line_m` | | 0..100 | line out. Tell the base after a shot (from the landing distance); `get` returns the number even while telemetry shows null. Kept in the position record |
| `trip_a` | 5 | 3..19 | wiper trip for aim, reel in and decock |
| `cock_trip_a` | 6.5 | 3..19 | wiper trip for cock and fire |
| `tow_trip_a` | 6.5 | 3..19 | wiper trip for the tow (`reel` with `until:"tension"`) |
| `vbat_low` | 9.9 | 6..12 | BROWNOUT threshold under load |
| `require_seated` | true | true / false | `arm` and `fire` need `SW_SEATED` |
| `vbat_wired` | true | true / false | the battery divider is fitted; false turns off `NO_POWER` and `BROWNOUT` and makes `vbat` null |
| `enc_sign` | 1 | 1 / -1 | AS5600 count direction (set while stopped) |
| `host_timeout_s` | 0 (off) | 0..600 | host watchdog: no line from the Jetson for this long while a motor runs = `stop` (two `ABORTED` events, the second `what:"HOST"`, `why:"host silent"`). Send `ping` to keep long runs alive (a `wrap` or `unwrap` takes about 40 s) |
| `fw`, `params`, `pitch_range` | | get only | firmware version (`base-1.2.1`), params.py version (`1.2.1`), `min` / `max` pitch (6.01 / 47 with the built-in table) |

The take-up trip (2.5 A) and the jog trip (3 A) are fixed in `config.h`, not `set` keys.

The bench settings record changed in base-1.2 (version 2). A record written by base-1.0 is ignored, so after flashing
base-1.2 every bench value is back at its default: enter them again (CALIBRATION.md).

## 8. Typical mission (D22, the params.TIMELINE scenario of the PC tests)

```
{"id":1,"cmd":"mode","to":"LAUNCH"}        -> MODE ok              (collar into LAUNCH, aim forgotten)
{"id":2,"cmd":"aim","deg":35}              -> AIMED ok             (homes on SW_RAIL_LOW first)
{"id":3,"cmd":"cock"}                      -> COCKED               (optional: fire from rest also works, about 2 s slower)
{"id":4,"cmd":"arm"}                       <- {"id":4,"ok":true,"token":482913,"valid_ms":3000}
{"id":5,"cmd":"fire","token":482913}       -> FIRED, RESET
   flight; dart LANDED (landing-zone verdict go): {"id":6,"cmd":"set","key":"line_m","val":4.6}
{"id":7,"cmd":"wrap"}                      -> WRAPPED              (right after landing, straight from LAUNCH: centering jog,
                                                                    drum through the window to the 314 park; about 40 s,
                                                                    wait up to 90 s)
   wait for ANCHORED (dart link)
{"id":8,"cmd":"tidy"}                      -> TIDIED               (take-up by time, 3 s by default)
{"id":9,"cmd":"mode","to":"REEL"}          -> MODE ok REEL         (fly-in, about 5 s)
{"id":10,"cmd":"reel","duty":1,"max_n":50,"until":"tension"}       -> LIMIT holding        (tow)
{"id":11,"cmd":"stop"}                     -> ABORTED HOLD         (the dart releases)
{"id":12,"cmd":"reel","duty":0.8,"until":"stop"}   ... line_m about 1.5 ... {"id":13,"cmd":"stop"} -> ABORTED REEL
{"id":14,"cmd":"unwrap"}                   -> UNWRAPPED, MODE NEUTRAL   (back through the window with the take-up: about 40 s)
{"id":15,"cmd":"retrieve"}                 -> SEATED               (take-up; FAULT DART_STUCK after 20 s)
{"id":16,"cmd":"mode","to":"LAUNCH"}       -> MODE ok              (then back to aim)
```

Notes for the Jetson:

- `retrieve` in step 15 is the same as `{"cmd":"reel","duty":0.3,"until":"seated"}`.
- A no-go landing (BASE_SPEC 10.1 step 7): no tow. Not wrapped yet: `mode NEUTRAL`, then `retrieve` (take-up).
  Already wrapped: `mode REEL`, reel in on the capstan (`reel` until `stop`, `stop` at about 1.5 m), `unwrap`, then
  `retrieve`. (The firmware also accepts `retrieve` at the WRAPPED park, on the take-up, but the mission does not use it.)
- After every shift into LAUNCH run `aim` before `cock` or `fire`. `arm` needs it anyway, and only an aim run tells
  the base where the collar sits in its dog window (a `cock` sent straight after a shift can fault with `BAD_STATE`
  "carriage not drawn" in the simulation).
- Wait for `RESET` after `FIRED` before the next command (the reset is part of `fire`).
- `line_m` is null after a shot. Set it from the landing distance, or the capstan line count and the seat-slow zone do
  not work.
- Treat `ABORTED` and `FAULT` as the end of whatever you were waiting for. After a `FAULT`, read the `MODE` event that
  follows (`ok:true` at the safe park, or `DRUM_HELD` with the reason).
- Flight, LANDED, ANCHORED and the release are dart and Jetson events; the base only sees the commands.

## 9. Power loss and the saved state

Both worm gears self-lock, so the mechanism stays where it stopped. The base keeps a position record in flash: at every
motion start it is written with "moving" set (a motor starts only once that write is done), and again with the final
positions about 300 ms after everything stopped. At the next start the base never moves by itself; the `BOOT` event
says what is known:

- Drum: the sector always comes from the drum code (absolute). The angle inside it comes from the record if the drum
  was not moving and the record agrees with the sector, else the park (LAUNCH, NEUTRAL, REEL) or the sector middle
  (N_TO_R: 321, past the 318 collar point, so `tidy` is refused until a move). A record that was not moving but
  disagrees with the code (drum turned by hand while off, or a bad contact) raises `DRUM_SENSOR` "drum code disagrees
  with the saved drum angle" (`clear` it once checked). `wraps` follows from the sector; inside WRAP it is unknown
  (`wraps_known` false, `wraps` null) unless the angle was trusted. An interrupted move is repeated with `mode`,
  `wrap` or `unwrap`; `unwrap` is safe at any wrap count.
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
- `decock` with `SW_HOME` closed and such a hint is refused with `SWITCH_DISAGREE`; with `force:true` it runs back by
  the travel the hint needs (`RESET` `how:"decock by travel"`).

Recovery after `BOOT` (lens B 7.4):

| what the base reports | recovery |
|---|---|
| `mode` LAUNCH, `H` and `G` closed, phase `REST` | none |
| phase `UNKNOWN`, `H` open (carriage drawn) | `decock`, then `aim`, then `cock` / `fire` |
| phase `UNKNOWN`, `H` closed | `cock` (creeps until `SW_HOME` opens, then cocks), or `cal cock_rest` after checking by eye. `LOADED_UNKNOWN`: check by eye, then `decock` with `force:true` or `cal cock_rest` |
| `drum_pos` REEL, `r` closed | wrapped, line held: resume the tow or reel in, or `unwrap` |
| `drum_pos` WRAPPED (N_TO_R) | at the 314 park: `tidy`, `mode REEL`, `retrieve` or `unwrap`. Elsewhere in N_TO_R: `mode REEL` or `unwrap` |
| WRAP, `wraps` null | stopped and reported: `unwrap` (take-up running), or `wrap` on to the WRAPPED park |
| `drum_pos` BETWEEN in L_TO_N, phase `UNKNOWN` | only `mode LAUNCH` is allowed (`LAUNCH_BUSY` otherwise) until the launcher is seen at rest |
| `drum_pos` UNKNOWN | `FAULT` `DRUM_SENSOR`: check the code contacts; hand lever |

Known firmware issue (simulation): reversing inside the WRAP window (an `unwrap` stopped halfway, then `wrap`) can
miscount the cup turns and fault with `DRUM_SENSOR` "SW_ARM counted 8 of 7 cup turns" at the far end. After such a
stop prefer to finish the `unwrap`.

Without the AS5600 magnet (decision D12): if `SW_INDEX` is switched on in `config.h` (it is off by default) and the
AS5600 does not read at start-up, the base uses the 6-bump index cam instead (`BOOT` `angle:"INDEX"`, `status`
`magnet:"INDEX"`). The angle then moves in 60 deg steps with time interpolation in between and errs high: `cock` stops
one bump (60 deg) earlier, the `SW_HOME` and release angle windows are 60 deg wider, aim is within about 3 deg, and
a missing bump while driving is `ANGLE_SENSOR` "SW_INDEX bump overdue (index cam or contact)". Index mode is chosen
only at start-up.

## 10. Changes for the Jetson in base-1.2

What the Jetson worker must change (against the base-1.0 version of this file):

1. Version strings: `HELLO`, `BOOT`, `ping`, `status` and `get fw` say `base-1.2.1`; `get params` says `1.2.1`.
2. Mission order (D22): after `FIRED` and `RESET` and a go landing verdict, send `wrap` straight away, in LAUNCH (no
   `mode NEUTRAL` first; `wrap` does the centering jog and the launch-exit checks itself). Then wait for ANCHORED,
   send `tidy`, then `mode REEL`, then the tow. The old order (`mode NEUTRAL`, `reel until slip`, `wrap`) is gone.
3. `wrap` ends at the WRAPPED park 314, not in REEL: one `WRAPPED` event, and NO `MODE` event after it. Telemetry then
   shows `drum_pos` `WRAPPED`, `mode` `NEUTRAL`, `wraps` 7. `wrap` no longer returns `NOT_IN_NEUTRAL`; from LAUNCH it
   can return `LAUNCH_BUSY`. It takes about 35 to 40 s (drum 8.1 deg/s): wait up to 90 s.
4. `mode REEL` is now the normal step after `tidy` (fly-in from the WRAPPED park, `MODE` ok REEL). It is still refused
   with `NOT_WRAPPED` below N_TO_R.
5. New commands: `tidy` (`ms`, `duty`; event `TIDIED`; refusals `BAD_ARG`, `NOT_IN_NEUTRAL`, `POSITION_UNKNOWN`),
   `retrieve` (`duty`, `max_n`; event `SEATED`; fault `DART_STUCK`), `decock` (= `cal decock`; event `RESET`).
6. `reel`, `tidy` and `retrieve` at the WRAPPED park run the take-up only, as in NEUTRAL. A wiper `jog` there is
   refused with `SHIFTING`.
7. Faults and `estop` from REEL or N_TO_R now send the drum to the WRAPPED park (`MODE` `ok:true`, `drum_pos`
   `WRAPPED`), never to NEUTRAL, and no longer `DRUM_HELD` "wraps on". Inside WRAP the drum holds with the new `why`
   "inside the wrap window: the operator decides (unwrap)". `SAFING` / `FAULTED` mean "on the way to the safe park".
8. Timing: the drum turns at about 8.1 deg/s (was 24). `mode` LAUNCH to NEUTRAL about 5 s, WRAPPED to REEL about 5 s,
   `unwrap` from REEL about 40 s. Use a timeout of at least 90 s for `wrap` and `unwrap`, and send `ping` during them
   if `host_timeout_s` is on.
9. Drum geometry: sector names and angles of params.DRUM (section 5), parks LAUNCH 0, NEUTRAL 41, WRAPPED 314,
   REEL 347, end stops -4 / 352 (were -8 / 358). Centering jog 8 deg (was 30), dog window 16 deg (was 60).
10. New `SWITCH_DISAGREE` cause "collar contact disagrees with the drum code" (dog contacts checked during every drum
    move); "collar still engaged at NEUTRAL" also at the WRAPPED park. "target code came far too early" now only
    with a trusted start angle.
11. Current trips (D8): aim now has its own 5 A trip; the tow (`until:"tension"`) trips at 6.5 A (was 5 A), new
    `set` key `tow_trip_a`. The
    `OVERCURRENT` `why` has one decimal: `wiper over 6.5 A`, `wiper over 5.0 A`, `wiper over 2.5 A`.
12. Tension estimate: `k_n_per_a x (amps - free_a) + tension_offset_n`, defaults 17.0 and -10.4 (was 8 and no
    offset), so `LIMIT` `tension_n` values change. New `set` / `get` key `tension_offset_n` (-50..50).
13. Drum jog: the automatic end stop is 4 deg past the LAUNCH park (-4) or the REEL park (351).
14. `host_timeout_s`, `tension_offset_n` and `tow_trip_a` are saved in flash. The bench settings record is new
    (version 2): after flashing base-1.2 all bench values are back at their defaults until set again.
15. `status` has a new field `ev_drops` (events dropped because the USB queue was full).
16. `mode` replies ok at once only when the collar is confirmed (LAUNCH: `SW_DOG_L`, REEL: `SW_DOG_R`, NEUTRAL:
    neither); otherwise it runs the move. `mode`, `wrap` (from LAUNCH) and `unwrap` that need the centering jog or
    the fly-in are refused with `ANGLE_SENSOR` without the AS5600.
17. `cock` is refused with `NOT_SEATED` (or `NO_SENSOR`) while `require_seated` and `SW_SEATED` is open.
18. `decock` ends only with `SW_HOME` and `SW_GAP` closed (new fault text `decock: SW_GAP never closed`). With the
    carriage home and the phase `UNKNOWN` it now sends `RESET` with `ok:false` and a `why` (it moved nothing).
19. A drum turned by hand while the base runs (`drum code changed with the drum still`) needs only one `clear`.
20. `set` and `cal` (except `cal drum`) in the first 60 ms after a start-up reply `BOOTING`. Wait for `BOOT`.
21. The tension estimate is averaged over about 200 ms (D33), so `LIMIT` comes a little later than with base-1.0.
22. D35: `retrieve` and `reel` with `until:"seated"` are refused in REEL with `WRAPS_ON` (never seat the dart on the
    capstan): reel in to about 1.5 m, `unwrap`, then `retrieve` in NEUTRAL.
