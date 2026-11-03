# GrappStar base station protocol (contract C1)

Firmware `base-1.2.1` (`firmware/base/grappstar_base`, the base-1.2 line), mechanism BASE_SPEC v1.2 (with the C1 sequence of section 10.1, D35) and `base/spec/params.py` 1.2.7 (as built into `params_gen.h`: drum, trips, tension model and the aim table) (drum
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
  Events wait in a queue that holds 23 lines; when the queue is full the newest event is dropped, and `status` field `ev_drops` counts it.
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
  `{"id":null,"ok":false,"err":"BAD_ID"}` (a `stop` or `estop` with such an id is still carried out). Strings are cut to 63 bytes; keep them under 24 bytes, because if the whole
  reply would pass 256 bytes it is rebuilt with only `id` (cut to 23), `ok`, `err` (and `token` for `arm`). If even
  that does not fit: `{"id":null,"ok":false,"err":"REPLY_OVERFLOW"}`. A `status` reply that is too long drops the
  fault `why` first, then all its other fields (only `id` and `ok` left). Use unique increasing integers.
- Field order in a reply: `id`, then any `val` / `min` / `max` from `get`, then `ok`, then `err` or the extra fields.
- A line that is not valid JSON: `{"id":null,"ok":false,"err":"BAD_JSON"}`. A line over 256 bytes:
  `{"id":null,"ok":false,"err":"TOO_LONG"}`. No `cmd`: `NO_CMD`. Unknown `cmd`: `UNKNOWN_CMD`.
- Keys longer than 15 characters are cut to 15, at most 12 keys per object are read (more are ignored, but `id` and `cmd` always
  take a slot from an unknown key), a repeated key takes the
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

Checked by every action that moves something, in this order. Argument errors come BEFORE them: `BAD_ARG` of `mode`
(bad or missing `to`), `aim` (no `deg`), `jog` (bad `motor` or `dir`, `ms` not over 0), `fire` (no `token`, then the
token checks) and `max_s` of `reel` / `retrieve`; also `tidy`'s `ms`, `POSITION_UNKNOWN` and `NOT_IN_NEUTRAL`. Other
argument checks (`reel` `duty`, `until`, `max_n`) come after them.

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
| `estop` | | like `stop`, then (when not faulted) the drum goes to NEUTRAL 41 if that is safe (section 6; with the wraps on it always holds): state `MODE`, a `MODE` event at the end, immediately if already there. If it is not safe the drum holds and a `MODE` event with `ok:false`, `err:"DRUM_HELD"` and `why` says why (exactly at the WRAPPED park: `MODE` `ok:true`). The estop move never turns the wiper (no centering jog). A failed estop move raises a drum fault | never refused |
| `clear` | | clears a latched fault, back to `IDLE`. Not faulted: `ok`, no change | `BUSY` (still `SAFING`), `STILL_PRESENT` (the cause is still there: AS5600 not reading after `ANGLE_SENSOR`, drum code invalid after `DRUM_SENSOR`, battery under `vbat_low` after `BROWNOUT`) |
| `mode` | `to`: `LAUNCH` \| `NEUTRAL` \| `REEL` | moves the drum to that park (LAUNCH 0, NEUTRAL 41, REEL 347 drum deg) and verifies the collar (section 5). Leaving LAUNCH starts with the centering jog. `mode REEL` only runs from N_TO_R (`drum_pos` `WRAPPED`, normally the 314 park after `wrap`): the drum turns forward into REEL with the fly-in. Already there (LAUNCH or NEUTRAL: the sector, the angle within 3 deg, and the collar confirmed, LAUNCH `SW_DOG_L` closed, NEUTRAL both open; REEL: the REEL sector with `SW_DOG_R` closed, no angle check): `ok` and an immediate `MODE` event; otherwise the move runs and confirms the collar at the end. Ends with a `MODE` event. LAUNCH to NEUTRAL or back about 5 s, WRAPPED to REEL about 5 s | in this order: `BAD_ARG` (before the common refusals), then the common ones, `HOLDING`, `POSITION_UNKNOWN` (drum code invalid), `HAND_MODE` (`SW_N20` open: the N20 swung out of mesh), `ANGLE_SENSOR` (AS5600 not reading and the move needs the wiper: any `REEL` request (fly-in, so also `mode REEL` from NEUTRAL), or leaving LAUNCH or REEL (centering jog)), `WRAPS_ON` (wraps doubtful), `NOT_WRAPPED` (`REEL` asked below N_TO_R: use `wrap` first), `WRAPS_ON` (`LAUNCH` or `NEUTRAL` asked from WRAP, N_TO_R or REEL: use `unwrap`), `LAUNCH_BUSY` (leaving LAUNCH needs the launcher at rest, section 6), `BUSY` (wiper still running) |
| `aim` | `deg`: rail pitch | turns the wiper BACKWARD (trip 5 A) until the pitch (from the aim table) is within `aim_tol` of `deg`. If the aim is unknown (always after a shift into LAUNCH, after a power cut, after a decock) it first runs backward until `SW_RAIL_LOW` closes (home), then to the target: up to about 1.25 aim cycles (10 wiper turns per cycle), so allow 60 s. Up to 3 tries. `AIMED` event | `BAD_ARG` (no `deg`, before the common refusals), `HOLDING`, `NOT_IN_LAUNCH` (collar not confirmed in LAUNCH, or the drum not stopped within 6 deg of the LAUNCH park), `PHASE_UNKNOWN`, `NOT_AT_REST` (aim only with the launcher at rest: backward drive with the spring drawn would let it unwind), `OUT_OF_RANGE` (see `get pitch_range`), `AIM_UNKNOWN` (aim unknown and no `SW_RAIL_LOW` fitted). A target within a quarter of `aim_tol` of the table's lowest or highest pitch is moved a quarter of `aim_tol` inside, so the `pitch_range` ends are reachable. In index mode every `aim` re-homes on `SW_RAIL_LOW` first |
| `cock` | | optional. Turns the wiper FORWARD (trip 6.5 A) from rest to the cocked angle (carriage drawn, short of the release tooth). `SW_HOME` must open at the first tooth. `COCKED` event. With the phase `UNKNOWN` and the carriage home it first creeps until `SW_HOME` opens (that re-zeroes the angle), then cocks. Already cocked (`SW_HOME` open): `ok` and an immediate `COCKED` event. Aim first: see the note under section 8 | `NOT_IN_LAUNCH`, `NO_SENSOR` (no `SW_HOME` fitted; or no `SW_SEATED` while `require_seated`), `SWITCH_DISAGREE` (phase `COCKED` but `SW_HOME` closed, or `REST` but `SW_HOME` open, or `REST` with `SW_GAP` fitted and open), `LOADED_UNKNOWN` (carriage drawn with the angle unknown), `NOT_SEATED` (`SW_SEATED` open while `require_seated`; checked after those phase checks, so an already cocked launcher still answers `COCKED`), then `LOADED_UNKNOWN` (`SW_HOME` closed while the last known angle says drawn: `decock` first) |
| `arm` | | checks everything `fire` needs and replies `"token":<6-digit integer>,"valid_ms":3000`. A new `arm` replaces the old token. `mode`, `aim`, `cock`, `wrap`, `jog`, `decock`, `stop`, `estop` and any fault cancel it | `NOT_IN_LAUNCH`, `WRAPS_ON` (wraps not 0 or unknown), `NO_SENSOR` (no `SW_HOME`), `NOT_COCKED` (neither `COCKED` with `SW_HOME` open and `SW_GAP` open, nor `REST` with `SW_HOME` closed and `SW_GAP` closed), `NOT_AIMED` (not aimed since the last shift, shot or aim-train move, or the pitch is more than `aim_tol` off the target), then last `NOT_SEATED` or `NO_SENSOR` (the `SW_SEATED` check) |
| `fire` | `token`: the integer from `arm` (a numeric string is accepted) | checks the token, then everything again, then runs the wiper FORWARD (trip 6.5 A up to the release, 5 A for the reset): from `COCKED` straight past the release tooth (about 1 s to `RESET`), from `REST` the whole cycle in one run (draw, release, reset: about 3 s). `FIRED` when `SW_HOME` closes (carriage released), `RESET` when the big gear is back in its gap with `SW_HOME` and `SW_GAP` closed. The token is single use; a wrong token cancels the arm | `BAD_ARG` (no `token`), `NOT_ARMED`, `TOKEN_EXPIRED` (over 3 s), `BAD_TOKEN`, then the common refusals and the `arm` refusals |
| `wrap` | | from LAUNCH (D22: straight after the shot; the same checks as leaving LAUNCH, then the centering jog), L_TO_N, NEUTRAL, WRAP_PREP or inside WRAP: drum forward through the WRAP window (62 to 302: the cup lays 7 turns on the capstan with the collar in NEUTRAL and the wiper off) to the WRAPPED park 314 in N_TO_R (cup locked, collar still NEUTRAL). `SW_ARM` must pulse 7 times across the window. `WRAPPED` event at the park; no `MODE` event follows. About 35 s from NEUTRAL, 40 s from LAUNCH: use a timeout of 90 s. Already in N_TO_R or REEL: `ok` and an immediate `WRAPPED` (nothing moves) | in this order after the common refusals and `HOLDING`: `WRAPS_ON` (the wraps are doubtful after a lying drum code, see section 6: only `unwrap`), then the already-there `WRAPPED`, `POSITION_UNKNOWN`, `LAUNCH_BUSY` (launcher not at rest, or `SW_GAP` open, when leaving LAUNCH or L_TO_N), `HAND_MODE`, `LAUNCH_BUSY` (spring known drawn), `BUSY` (wiper running), `ANGLE_SENSOR` (from LAUNCH without the AS5600: the centering jog needs it) |
| `unwrap` | | from REEL, N_TO_R (the WRAPPED park included), WRAP or WRAP_PREP: from REEL first the centering jog (forward at 25 % until the 16 deg dog window is taken up or 3 s, then 8 deg back), then the drum backward to NEUTRAL 41. While the drum is in WRAP and WRAP_PREP the wiper turns forward at 40 % (take-up only, trip 2.5 A: the capstan is not driven, the take-up winds the freed line). After a full pass `SW_ARM` must have pulsed 7 times. `UNWRAPPED` event, then `MODE` (NEUTRAL). About 40 s from REEL. Safe at any wrap count (also after a power cut mid-wrap). With the wraps doubtful (see below) it runs the take-up from WRAP_PREP on too. Arriving at NEUTRAL clears the doubt. Already at NEUTRAL: `ok` and an immediate `UNWRAPPED` then `MODE` (also clears the doubt) | `POSITION_UNKNOWN`, `NOT_WRAPPED` (drum in LAUNCH or L_TO_N), `BUSY` (wiper running), `ANGLE_SENSOR` (the take-up needs the wiper), `HAND_MODE`, `LAUNCH_BUSY` (spring known drawn) |
| `tidy` | `ms` (default 3000 = config `tidyMs`, over 0 up to 60000), `duty` (default 0.3) | D22 tidy by time: take-up only, wiper FORWARD for `ms` (trip 2.5 A). Allowed at the WRAPPED park (N_TO_R below 318) or in NEUTRAL, both dog contacts open. `duty` over 0 up to 1; below 0.27 it runs at 0.27, above 0.40 it is capped at 0.40. `TIDIED` event at the end. Telemetry `state` is `REEL` meanwhile; `stop` ends it with `ABORTED` `what:"REEL"` | before the common refusals: `BAD_ARG` (`ms` out of range), `POSITION_UNKNOWN` (drum code invalid), `NOT_IN_NEUTRAL` (anywhere else, or a dog contact closed); after them: `BAD_ARG` (bad `duty`) |
| `retrieve` | `duty` (default 0.3), `max_n` (default `limit_n`), `max_s` (optional, 5 to 600) | final approach on the take-up: the same as `reel` with `until:"seated"`. Only in the NEUTRAL sector (D39): the take-up pulls the dart in (trip 2.5 A). No `SW_SEATED` within the time limit: fault `DART_STUCK`. The limit is `max_s`, else the longer of 20 s and 10 s + 1.5 x `line_m` / v when `line_m` is known, else 30 s, never over 600 s (see the `reel` detail). Never on the capstan (D35) and never with the wraps on: anywhere else, the WRAPPED park included, it is refused with `WRAPS_ON`. `SEATED` event. Already seated: `ok` and an immediate `SEATED` | as `reel`, so outside the NEUTRAL sector `WRAPS_ON` (`unwrap` first); `BAD_ARG` for `max_s` outside 5..600 |
| `decock` | `force` (optional) | the same as `cal decock` (section 7) | as `cal decock` |
| `reel` | `duty` over 0 up to 1 (default 0.86; below 0.27 it runs at 0.27), `max_n` (default `limit_n`, at most 70: `BAD_ARG` above, D39), `until`: `stop` (default) \| `seated` \| `tension` \| `slip`, `max_s` (optional, 5 to 600: time limit of `until:"seated"`) | REEL (collar confirmed): the capstan winds in (tow, reel in), see below. NEUTRAL or the WRAPPED park (N_TO_R below 318), both dog contacts open: take-up only, see below. Also accepted while holding (state `HOLD`): winds again | `BAD_ARG` (bad `until`, `duty` not in (0, 1], `max_n` not over 0, `slip` in REEL), `NOT_WRAPPED` (neither REEL nor take-up position, or `tension` without the capstan), `WRAPS_ON` (`seated` outside the NEUTRAL sector: never seat the dart on the capstan or with the wraps on, D35 / D39), `BAD_ARG` (`max_n` over 70), `NO_SENSOR` (`seated` without `SW_SEATED`), `BAD_ARG` also for `max_s` outside 5..600 (checked on every `reel`) |
| `jog` | `motor`: `wiper` \| `drum`, `dir`: 1 \| -1, `ms` (default 300, capped at 1500) | runs one motor for `ms`. Deadman: the same jog (same motor and dir) sent again before it ends extends it. Wiper dir 1 = FORWARD, drum dir 1 = toward REEL. Ends silently (state back to `IDLE`), see below for the limits | see below |
| `cal` | `what`, see section 7 | calibration actions | per action (section 7), `BAD_ARG` for an unknown `what` |
| `set` | `key`, `val` (number or `true` / `false`) | sets a value (section 7) | `BAD_ARG` (no key or val), `BAD_KEY`, `OUT_OF_RANGE`, for `enc_sign`: `BAD_ARG` unless 1 or -1 and `BUSY` while moving (every other key is accepted while moving) |
| `get` | `key` | reply carries `val` (numbers with up to 7 significant digits; `min` and `max` for `pitch_range`) | `BAD_ARG` (no key), `BAD_KEY` |

#### `reel` in detail

In REEL (capstan, wiper FORWARD):

- Trip with `until:"tension"` (the tow, D34): the larger of `tow_trip_a` (6.5 A) and the current the limit needs plus
  1 A, that is `(max_n - tension_offset_n) / k_n_per_a + free_a + 1`, never above the hard 17 A. With the defaults
  and `max_n` 50 that is 6.5 A (a 50 N tow draws about 5.1 A, so the tension estimate is the real limit); with
  `max_n` 70 (the maximum, D39) and `free_a` 1.5 it is (70 + 10.4) / 17 + 1.5 + 1 = 7.2 A. A `set` of any of these values during a tow moves the trip at once. Any other `until`:
  `trip_a` (5 A).
- Tension = load cell when one is live, else the estimate
  `k_n_per_a x (amps - free_a) + tension_offset_n` (defaults 17.0 and -10.4: physics v1.2 with the clutch FIRM at 5 N;
  never below 0, averaged over about 200 ms (D33); the first 350 ms of a run are ignored). The speed tapers over the last 30 % of `max_n`.
- Reaching `max_n` stops the wiper and sends `LIMIT`. With `until:"tension"` the base then HOLDS (state `HOLD`,
  `holding:true`, the worm and sprag #2 hold the line); with a load cell it winds again by itself when the tension
  falls under 60 % of `max_n`. With any other `until` it goes to `IDLE` (`holding:false`).
- `until:"seated"` is refused in REEL with `WRAPS_ON` (D35: the dart is never seated on the capstan; reel in to
  about 1.5 m, `unwrap`, then `retrieve` on the take-up in NEUTRAL).
- `until:"stop"`: runs until `stop` (or the limit). The Jetson stops it at about 1.5 m from the line count or the
  camera.
- After 240 s any reel stops with `ABORTED` (`what:"REEL"`, `why:"timeout"`).
- An over-current or stall during a tow is a fault, never a hold.

In NEUTRAL or at the WRAPPED park (take-up only, decision D15 / lens B I4: wiper FORWARD at most 40 % duty, trip
2.5 A; the capstan is not driven, so there is no tension limit and `line_m` does not change):

- `until:"seated"` (or `retrieve`), only in the NEUTRAL sector (at the WRAPPED park `WRAPS_ON`, D39): the final
  approach. Stops at `SW_SEATED` (`SEATED` event, `line_m` set to 0).
  Already seated: `ok` and an immediate `SEATED`. No `SW_SEATED` within the time limit: fault `DART_STUCK`
  "final approach: no SW_SEATED in N s". The limit is `max_s` when given (5 to 600 s); otherwise, with `line_m` known,
  the longer of 20 s and 10 s + 1.5 x `line_m` / v, where v is the slowest take-up line speed at the duty used
  (firmware model v = 0.29 x duty - 0.03 m/s, duty clamped to 0.27..0.40; physics: 0.058 m/s at duty 0.3); with
  `line_m` unknown 30 s; never over 600 s. A snagged dart cannot be seen in the current (physics), only by time. `line_m` is not tracked while only the take-up runs (only the capstan in REEL counts it), so
  it still holds the value from the last capstan reel-in or `set line_m`; it is unknown after a shot until the Jetson
  sends `set line_m`, and it is set to 0 on `SEATED`. Example: 5 m at duty 0.3 gives 10 + 1.5 x 5 / 0.057 = about 141 s (a 5 m take-up
  retrieval took about 62 s in the simulation). Send `set line_m` (or `max_s`) before a long take-up retrieval.
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
| `BAD_ARG` | bad `motor` or `dir`, `ms` not over 0 (before the common refusals) |
| `BUSY` | the drum is running |
| `SHIFTING` | the drum is in L_TO_N or N_TO_R (dogs half engaged; this includes the WRAPPED park: use `tidy` there), or its code is invalid |
| `SWITCH_DISAGREE` | both dog contacts closed |
| `NOT_IN_LAUNCH` | drum in the LAUNCH sector, or `SW_DOG_L` closed anywhere, but not at the LAUNCH park with the collar confirmed |
| `LAUNCH_BUSY` | the spring is known drawn but the drum code is not LAUNCH |
| `WOULD_FIRE` | forward in LAUNCH with the phase `COCKED`, `UNKNOWN`, `FIRING` or `RESETTING` |
| `NO_SENSOR` | forward in LAUNCH without `SW_HOME` fitted |
| `NOT_AT_REST` | backward in LAUNCH with the phase not `REST` (the drawn spring would unwind: use `decock`) |
| `BACKSTOP` | backward anywhere except at the LAUNCH park: in REEL (or with `SW_DOG_R` closed) sprag #2 holds the capstan; elsewhere backward fights the take-up bus one-way (BASE_SPEC 10) |

A forward jog in LAUNCH from rest draws the carriage (phase `COCKING` once `SW_HOME` opens) and stops itself short of
the release tooth with `ABORTED` (`what:"JOG"`, `why:"would pass the release tooth"`). A backward jog in LAUNCH turns
the aim train and clears `aimed`.

Drum jog (full speed, no centering jog, no checks of the target):

| err | when |
|---|---|
| `BAD_ARG` | as for the wiper jog |
| `BUSY` | the wiper is running |
| `POSITION_UNKNOWN` | drum code invalid, or a jog toward an end stop (dir -1 in LAUNCH, dir 1 in REEL) while the drum angle is not trusted (boot after a cut during a drum move) |
| `HAND_MODE` | `SW_N20` open |
| `LAUNCH_BUSY` | drum in LAUNCH or L_TO_N (or the spring known drawn) and the launcher not at rest; dir -1 inside LAUNCH is allowed |
| `WRAPS_ON` | dir -1 in REEL (leaving REEL needs the centering jog: use `unwrap`), any jog inside WRAP (only `wrap` and `unwrap` cross the window), or any drum jog while the wraps are doubtful |
| `SWITCH_DISAGREE` | a dog contact disagrees with the drum code (`SW_DOG_L` closed past L_TO_N, or `SW_DOG_R` closed before N_TO_R) |

A drum jog stops by itself when the angle estimate comes within 2 deg of an end stop (-2 deg in LAUNCH, 350 deg in
REEL) (`ABORTED`, `what:"JOG"`, `why:"drum end stop"`). It also stops when it enters WRAP
(`ABORTED`, `what:"JOG"`, `why:"wrap window: use wrap or unwrap"`). A dog contact that disagrees with the drum code
during the jog is `FAULT` `SWITCH_DISAGREE` "collar contact disagrees with the drum code". Any jog clears `aimed`
(drum) and cancels an arm token.

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
| `n` | `SW_N20` | drum N20 in mesh (open = the N20 swung out of mesh on its cradle for the hand lever: hand mode) |
| `H` | `SW_HOME` | carriage on its front stop (uncocked) |
| `G` | `SW_GAP` | big gear in its toothless gap (rest) |
| `L` | `SW_RAIL_LOW` | rail at its low dead point (aim home) |
| `S` | `SW_SEATED` | dart tail on the pusher |
| `W` | `SW_ARM` | wrap cup AWAY from home. The contact opens for about +-5 deg at home (cad-reel); the firmware counts a cup turn on each opening |
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
- Lean: when the normal line would pass 256 bytes, it is rebuilt with one decimal less on `drum_deg`, `wiper_deg`, `rpm`,
  `amps`, `tension_n`, `pitch_deg` and `line_m` (`cock` keeps 2, `vbat` 1), null fields LEFT OUT, and
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
| `AIMED` | `ok`, `pitch_deg` (null if unknown), `target_deg` (the target after the quarter-tolerance move at the table ends); `err` (`AIM_MISS` after 3 tries, `OUT_OF_RANGE`) when `ok:false` | aim ended |
| `COCKED` | `ok`, `cock_deg` (wiper deg forward from rest) | cocked |
| `FIRED` | `pitch_deg`, `cock_deg` (wiper deg of the release) | the carriage was released (`SW_HOME` closed). `aimed` is cleared and `line_m` becomes null |
| `RESET` | `ok`; `how` (`decock` or `decock by travel`) after a decock; sometimes `why` | the launcher is back at rest after a shot or a decock. `ok:true`, `how:"decock"`, `why:"SW_HOME and SW_GAP closed: at rest"`: a decock with the phase `UNKNOWN` and both contacts closed (nothing moved, phase now `REST`). `ok:false`, `how:"decock"`, `why:"carriage home: nothing to decock, cock finds the angle"`: a decock with the carriage home and the phase `UNKNOWN` (nothing moved) |
| `WRAPPED` | `ok`, `wraps` | `wrap` reached the WRAPPED park (314, collar NEUTRAL), or at once when the drum is already in N_TO_R or REEL. No `MODE` follows |
| `TIDIED` | `ok` | `tidy` ran its time |
| `UNWRAPPED` | `ok` | back at NEUTRAL with no wraps; always followed by `MODE`, also in the immediate case |
| `SEATED` | | `SW_SEATED` closed during `reel until seated` or `retrieve` (`line_m` set to 0) |
| `LIMIT` | `tension_n` (null for the slip tidy), `max_n`, `why` (`tension limit` or `tidy done (take-up slipping)`), `holding` | the reel reached `max_n` (holding when `until:"tension"`), or the slip tidy finished |
| `STALL` | | the wiper stalled (sent right after the `FAULT` with code `STALL`) |
| `FAULT` | `code`, `why` (the full text, the longest is 66 characters; the latched copy is cut to 47 and `status` `why` to 32) | a fault latched (section 6) |
| `ABORTED` | `what` (the state that was running: `MODE`, `AIM`, `COCK`, `FIRE`, `WRAP`, `UNWRAP`, `REEL` (also `tidy` and `retrieve`), `HOLD`, `JOG`, `DECOCK`; or `HOST`), `why` (`stop`, `estop`, `timeout`, `would pass the release tooth`, `drum end stop`, `wrap window: use wrap or unwrap`, `host silent`) | an action ended without completing. Treat it as the failure of whatever you were waiting for (a `stop` you sent yourself also gives one). The host watchdog sends two: the stopped state with `why:"stop"`, then `what:"HOST"` |
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
   never forward, that would cock). In REEL it first turns FORWARD at 25 % until the 16 deg window is taken up (or
   3 s), then 8 deg back (sprag #2 holds the line meanwhile). Trip 3 A. The 8 deg leg is also capped in time (1.5 x
   its free-speed time), so a noisy angle cannot carry it past the window.
3. The N20 runs to the target sector's code edge, then on by time to the park.
4. During the run `SW_DOG_L` must stay open once the drum is past L_TO_N, and `SW_DOG_R` must stay open until N_TO_R
   (so the WRAP window is never entered with either closed); otherwise `SWITCH_DISAGREE` "collar contact disagrees
   with the drum code". While unwrapping, the wiper turns forward at 40 % in WRAP and WRAP_PREP (take-up, trip
   2.5 A). Toward REEL the fly-in starts at 318 deg: wiper forward at 25 % (trip 3 A; 25 % so the trip sees the true current).
5. At the park the dog contacts must confirm the collar: REEL needs `SW_DOG_R` (fly-in continues for at most 120 deg
   of wiper or 6 s); LAUNCH needs `SW_DOG_L` (if not yet, the wiper is turned BACKWARD at 25 % for at most 40 deg,
   never forward; with the spring possibly drawn this is a fault instead); NEUTRAL and the WRAPPED park need both
   open after 100 ms. After arriving in LAUNCH the aim is forgotten (`aim_known` false): the next `aim` re-homes.
6. The end event: `MODE` (`mode`, `estop`, safing), `WRAPPED` (`wrap`), `UNWRAPPED` then `MODE` (`unwrap`).

Checks against the clock: no code edge for 1.5 x the time the current sector should take plus 1.5 s (LAUNCH and REEL
count as 20 deg), or the whole move over 1.5 x the expected time plus 1.5 s: `DRUM_TIMEOUT`. The target code arriving
in under a quarter of the expected time to the target code's edge (only when that time is over 1.5 s, and only when
the start angle is trusted, that is a code edge was seen or a saved angle agreed at boot): `DRUM_SENSOR` "target code
came far too early".

If the drum leaves the target sector before the park (an untrusted start angle, or `drum_dps` wrong), it stops, comes
back to the edge it just crossed and runs on from there. A second overshoot in the same move is `DRUM_SENSOR` "drum
overshot the target sector"; not back within the time of 10 deg plus 1.5 s is `DRUM_TIMEOUT` "drum did not come back
to the target sector". So a drum move started from an untrusted angle never runs on into the next sector.

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
- Wiper backward is refused everywhere except at the LAUNCH park (`BACKSTOP`; in REEL sprag #2, elsewhere the take-up
  bus one-way); forward in NEUTRAL, WRAP_PREP, WRAP and at the WRAPPED park is
  take-up only (40 %).
- The drum never moves while `SW_N20` is open (hand mode); the N20 latching out during a move is a fault.
- `SW_DOG_L` and `SW_DOG_R` are checked during every drum move (section 5, step 4).
- No `mode LAUNCH` or `mode NEUTRAL` with wraps on (use `unwrap`); `mode REEL` only from N_TO_R (after `wrap`).

### Current and stall protection (decision D8)

| state | trip |
|---|---|
| aim, reel in (`reel` until `stop` in REEL), decock, the reset after a release (`trip_a`) | 5 A |
| cock, fire up to the release, forward jog in LAUNCH (`cock_trip_a`) | 6.5 A |
| tow (`reel` with `until:"tension"`): the larger of `tow_trip_a` and `(max_n - tension_offset_n) / k_n_per_a + free_a + 1` (D34). A 50 N tow draws about 5.1 A: the tension estimate is the real tow limit | 6.5 A with the defaults, at most 17 A |
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
| `DRUM_TIMEOUT` | `drum not turning (no code edge)`, `drum did not reach the target in time`, `drum did not come back to the target sector`, `dogs did not engage (collar contact open)` |
| `DRUM_SENSOR` | `drum code 000 (wire off or bump missing)` or `drum code jumped over a sector` for 150 ms; `drum turned the wrong way (drum_sign?)` (N20 wires swapped); `target code came far too early`; `drum overshot the target sector` (a second time in one move); `drum code changed with the drum still` (the worm cannot be back-driven: a contact is lying, or the drum was turned by hand while the base runs; one `clear` is enough once the drum is still); `N20 latched out during a drum move`; `SW_ARM counted n of 7 cup turns` (checked only for a whole pass: entered at one end and left at the other in one `wrap` or `unwrap` move); `drum code disagrees with the saved drum angle` (at boot) |
| `ANGLE_SENSOR` | `AS5600 lost while the wiper runs` (4 bad reads, or no magnet); `AS5600 counts against the drive (enc_sign?)`; `AS5600 glitching (over 20 bad samples a second)`; `AS5600 angle jumped while turning (travel lost)` (a lasting jump taken without counting it, while the wiper turns or during aim, cock, fire or decock); in index mode `SW_INDEX bump overdue (index cam or contact)` |
| `SWITCH_DISAGREE` | `SW_DOG_L and SW_DOG_R both closed` (200 ms, or at the end of a move); `collar contact disagrees with the drum code` (during a drum move or a drum jog); `drum code contact chattering` during a drum move; `collar still engaged at NEUTRAL` (also at the WRAPPED park); `SW_HOME opened at the wrong angle`; `SW_HOME still closed past the first tooth`; `SW_HOME chattering during decock` |
| `BROWNOUT` | `battery sagged under load`: under `vbat_low` for 300 ms while a motor runs |
| `BAD_STATE` | the mechanism did not do what the angle says: `collar left LAUNCH during a launch action`, `drum code away from LAUNCH but the spring is drawn`, `LAUNCH collar not confirmed with the spring possibly drawn`, `SW_RAIL_LOW never closed in an aim cycle`, `aim ran too far`, `carriage not drawn (SW_HOME still closed): rack or dog not engaged`, `carriage slipped home while cocking`, `SW_HOME closed at the end of cocking`, `SW_HOME never opened (stuck closed?)`, `release at the wrong angle` (after `FIRED`), `no release seen (SW_HOME did not close)`, `SW_HOME opened while resetting`, `reset ended with the carriage off its stop`, `reset ended with the big gear out of its gap`, `decock: SW_HOME never closed`, `decock: SW_GAP never closed`, and the time limits (`cocking took too long`, `cock recovery took too long`, `fire cycle took too long`, `decock by travel took too long`) |
| `DART_STUCK` | `final approach: no SW_SEATED in N s` (N = the time limit in force, section 2 `reel`) |

A fault during any launch-train motion makes the shot phase `UNKNOWN` (the last angle is kept as the phase hint,
section 9). Any fault cancels the arm token.

### Fault response

The wiper stops at once. Then, from LAUNCH, L_TO_N, NEUTRAL or WRAP_PREP, the drum goes to NEUTRAL 41 without any
wiper motion (state `SAFING`, then `FAULT`, `MODE` event with `ok:true`). `estop` does the same (state `MODE` instead
of `SAFING`).

With the wraps on (WRAP, N_TO_R with the WRAPPED park, REEL) the drum never moves after a fault or an `estop`
(params fault rule since 1.2.5, D27): it holds where it is, the worm and sprag #2 hold the line, and the operator unwraps
when it is safe. Exactly at the WRAPPED park (within 4 deg of 314) the `MODE` event says `ok:true`; anywhere else it
is `DRUM_HELD`. The drum also holds when another reason makes a move unsafe. Then a `MODE` event with `ok:false`,
`err:"DRUM_HELD"` and `why` says why (checked in this order):

| `why` | meaning |
|---|---|
| `drum fault` | the fault is `DRUM_TIMEOUT` or `DRUM_SENSOR`: the drum may be jammed or its position wrong |
| `drum code invalid` | |
| `N20 latched out (hand mode)` | |
| `wraps unknown` | |
| `wraps on: the drum holds, the operator unwraps when safe` | the drum is in WRAP, N_TO_R or REEL: recover with `clear`, then (when the line is safe) `unwrap`, or resume the reel-in in REEL |
| `launch train not at rest: the wiper worm holds the spring (a shift under load could dry fire)` | there is no sear: the drawn spring is held only by the wiper worm through the collar. Recover with `clear`, then `decock` (or `fire` if it is safe to shoot) |

Already at NEUTRAL (or at the WRAPPED park): an immediate `MODE` with `ok:true`. A second fault during `SAFING` stops the drum and sends
`MODE` with `err:"DRUM_HELD"` and that fault's `why`. A safing move that fails sends `MODE` with `ok:false`, `err` =
the drum fault code and its `why`, and stays in `FAULT`. `stop` during `SAFING` stops the drum (no `MODE` event).

Doubtful wraps: a drum code change with the drum still (`DRUM_SENSOR` "drum code changed with the drum still") that
involves WRAP_PREP or anything beyond it marks the wraps doubtful (`wraps_known` false). After `clear`, `mode`, `wrap`
and a drum `jog` reply `WRAPS_ON`; only `unwrap` is accepted, and it runs the take-up all the way from WRAP_PREP.
`UNWRAPPED` at NEUTRAL clears the doubt.

A boot inside WRAP: the base never moves by itself after a start-up. It stops and reports (`BOOT`, `wraps_known`
false unless the saved angle was trusted); the operator decides (section 9).

## 7. Calibration and settings

`cal` actions (`{"id":1,"cmd":"cal","what":"current"}`):

| what | args | does | refusals |
|---|---|---|---|
| `current` | | zero the wiper current reading (motor stopped) | `BUSY` (running), `OUT_OF_RANGE` (the new offset would be outside 0..5 A) |
| `free` | | learn the free-running current from the run in progress (after 0.8 s of running) | `NOT_RUNNING`, `OUT_OF_RANGE` (over 10 A) |
| `tare` | | zero the load cell | `NO_SENSOR` |
| `aim` | `deg`: pitch measured with a phone inclinometer | sets `aim_offset` so the reported pitch matches | `BAD_ARG` (no `deg`), `AIM_UNKNOWN`, `OUT_OF_RANGE` (the offset would be outside -20..20) |
| `aim_home` | | declare the aim train at its home (where `SW_RAIL_LOW` closes on the backward run) | `BUSY` (moving) |
| `aim_forget` | | forget the aim: the next `aim` re-homes | `BUSY` |
| `cock_rest` | `force` (optional, true: `SW_HOME` known broken, checked by eye) | declare the launcher at rest (carriage forward). For a new build, or after checking by eye | `BUSY`, `SWITCH_DISAGREE` (`SW_HOME` open, or `SW_GAP` fitted and open, and no `force`; `force:true` overrides both) |
| `decock` (also the command `decock`) | `force` (optional) | lets a drawn carriage down without firing: runs the wiper BACKWARD slowly (47 %, trip 5 A) so the spring unwinds under the worm, until `SW_HOME` has stayed closed for 15 deg more and `SW_GAP` is closed too. Turns the aim train too: the aim becomes unknown. `RESET` event with `how:"decock"`. Carriage already home: phase `REST` gives an immediate `RESET`; phase `UNKNOWN` with no sign of a draw replies `ok` and moves nothing: with `SW_GAP` closed too the phase becomes `REST` and it sends `RESET` with `ok:true`, `how:"decock"`, `why:"SW_HOME and SW_GAP closed: at rest"`; with `SW_GAP` open it sends `RESET` with `ok:false`, `how:"decock"`, `why:"carriage home: nothing to decock, cock finds the angle"` (then use `cock` or `cal cock_rest`). `SW_HOME` closed while the last known angle says drawn: refused with `SWITCH_DISAGREE`, unless `force:true`: then it runs back by the travel that angle needs and calls it rest (`how:"decock by travel"`) | common (`ANGLE_SENSOR` included), `NOT_IN_LAUNCH` (drum code not LAUNCH), `NO_SENSOR` (no `SW_HOME`), `SWITCH_DISAGREE` |
| `drum` | | the drum code is absolute: this only reports, with a `MODE` event | `POSITION_UNKNOWN` |

`set` / `get` keys. Saved in flash once everything has stopped (only when a value really changed), kept over a power cycle and a new upload, except where
the table says otherwise:

| key | default | range | meaning |
|---|---|---|---|
| `limit_n` | 50 (params.REEL) | 1..70 (D39) | default `max_n` for `reel` and `retrieve` (the tow limit) |
| `k_n_per_a` | 17.0 | 0.1..200 | line N per amp above `free_a` (tension estimate without a load cell) |
| `tension_offset_n` | -10.4 | -50..50 | line N added to the estimate: `k_n_per_a x (amps - free_a) + tension_offset_n` |
| `free_a` | 1.5 (params.WIPER) | 0..10 | wiper free-running current |
| `amps_per_volt` | 12.5 | 1..200 | BTS7960 IS scale |
| `offset_a` | 0 | 0..5 | current zero (set by `cal current`) |
| `hx_n_per_count` | 0.0000476 | 0.0000001..1 | load cell scale |
| `drum_dps` | 8.1 | 5..720 | drum speed estimate (learned during moves; saved with the next `set` or `cal`) |
| `aim_tol` | 0.75 | 0.1..5 | AIMED tolerance, deg (raised to at least 2.0 at start-up in index mode) |
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
| `fw`, `params`, `pitch_range` | | get only | firmware version (`base-1.2.1`), params.py version (`1.2.7`, the version `params_gen.h` was generated from), `min` / `max` pitch (6.01 / 47 with the built-in table) |

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
{"id":11,"cmd":"stop"}                     -> ABORTED HOLD         (stop first: the worm holds the line; then the dart /release)
{"id":12,"cmd":"reel","duty":0.8,"until":"stop"}   ... line_m about 1.5 ... {"id":13,"cmd":"stop"} -> ABORTED REEL
{"id":14,"cmd":"unwrap"}                   -> UNWRAPPED, MODE NEUTRAL   (back through the window with the take-up: about 40 s)
{"id":15,"cmd":"retrieve"}                 -> SEATED               (take-up; FAULT DART_STUCK after the time limit)
{"id":16,"cmd":"mode","to":"LAUNCH"}       -> MODE ok              (then back to aim)
```

Notes for the Jetson:

- `retrieve` in step 15 is the same as `{"cmd":"reel","duty":0.3,"until":"seated"}`.
- A no-go landing (BASE_SPEC 10.1 step 7): no tow. Not wrapped yet: `mode NEUTRAL`, then `retrieve` (take-up).
  Already wrapped: `mode REEL`, reel in on the capstan (`reel` until `stop`, `stop` at about 1.5 m), `unwrap`, then
  `retrieve`. From LAUNCH a no-go needs `mode NEUTRAL` first: `retrieve` runs only in the NEUTRAL sector.
- After every shift into LAUNCH run `aim` before `cock` or `fire`. `arm` needs it anyway, and only an aim run tells
  the base where the collar sits in its dog window (a `cock` sent straight after a shift can fault with `BAD_STATE`
  "carriage not drawn" in the simulation).
- Wait for `RESET` after `FIRED` before the next command (the reset is part of `fire`).
- `line_m` is null after a shot. Set it from the landing distance, or the capstan line count and the seat-slow zone do
  not work.
- Treat `ABORTED` and `FAULT` as the end of whatever you were waiting for. After a `FAULT`, read the `MODE` event that
  follows (`ok:true` at NEUTRAL or the WRAPPED park, or `DRUM_HELD` with the reason: always with the wraps on).
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
  Exception (systest F15): if the wiper was turning with the launch train at rest (aim, centering jog, reel, take-up)
  and `SW_HOME` and `SW_GAP` are both closed at boot, the phase stays `REST`. A cut during `cock` or `fire` stays
  `UNKNOWN` with the drawn hint (below).
- A launch train stopped mid-cycle (`COCKING`, `FIRING`, `RESETTING`) is never trusted: `UNKNOWN`. The record was
  written as that motion started, so for a cut while cocking or firing the base assumes the worst: the phase hint
  becomes the cock end angle (carriage drawn). `cock` then replies `LOADED_UNKNOWN`.
- `SW_HOME` and `SW_GAP` must agree with the record: `REST` needs both closed, `COCKED` needs `SW_HOME` open; anything
  else gives `UNKNOWN`.
- `line_m` comes back from the record (null if it was unknown or there is no record).

The phase hint: whenever the phase becomes `UNKNOWN` (power cut, `stop` or a fault during a launch-train motion), the
last known cock angle is kept, also in flash (a `stop` while the phase was already `UNKNOWN` keeps the larger, more
drawn, of the old hint and the current angle). It is not reported, but it decides:

- `cock` is refused with `LOADED_UNKNOWN` when `SW_HOME` is open, or when `SW_HOME` is closed but the hint says the
  carriage was drawn (between the first tooth and the release): the switch may be lying and a creep could fire.
- `decock` with `SW_HOME` closed and such a hint is refused with `SWITCH_DISAGREE`; with `force:true` it runs back by
  the travel the hint needs (`RESET` `how:"decock by travel"`).

Recovery after `BOOT` (lens B 7.4):

| what the base reports | recovery |
|---|---|
| `mode` LAUNCH, `H` and `G` closed, phase `REST` | none |
| phase `UNKNOWN`, `H` open (carriage drawn) | `decock`, then `aim`, then `cock` / `fire` |
| phase `UNKNOWN`, `H` closed, power cut while cocking or firing (`cock` gives `LOADED_UNKNOWN`) | check by eye, then `decock` with `"force":true` (decock by travel, `RESET` `how:"decock by travel"`), then `aim` and `cock`; or `cal cock_rest` if the carriage is seen at rest |
| phase `UNKNOWN`, `H` closed, no sign of a draw | `cock` (creeps until `SW_HOME` opens, then cocks), or `cal cock_rest` after checking by eye |
| `drum_pos` REEL, `r` closed (also after a fault or `estop` in REEL: the drum held) | wrapped, line held: resume the tow or reel in, or `unwrap` |
| `drum_pos` WRAPPED (N_TO_R) | at the 314 park: `tidy`, `mode REEL` or `unwrap`. Elsewhere in N_TO_R: `mode REEL` or `unwrap` |
| WRAP, `wraps` null | stopped and reported: `unwrap` (take-up running), or `wrap` on to the WRAPPED park |
| `wraps_known` false after `DRUM_SENSOR` "drum code changed with the drum still" near the window | `clear`, then `unwrap` (the only drum command accepted) |
| `drum_pos` BETWEEN in L_TO_N, phase `UNKNOWN` | only `mode LAUNCH` is allowed (`LAUNCH_BUSY` otherwise) until the launcher is seen at rest |
| `drum_pos` UNKNOWN | `FAULT` `DRUM_SENSOR`: check the code contacts; hand lever |

The `SW_ARM` cup count is only checked for a whole pass. A `wrap` or `unwrap` that starts inside WRAP (after a stop or
a power cut) is not counted.

Without the AS5600 magnet (decision D12): if `SW_INDEX` is switched on in `config.h` (it is off by default) and the
AS5600 does not read at start-up, the base uses the 6-bump index cam instead (`BOOT` `angle:"INDEX"`, `status`
`magnet:"INDEX"`). The angle then moves in 60 deg steps with time interpolation in between and errs high: `cock` stops
one bump (60 deg) earlier, the `SW_HOME` and release angle windows are 60 deg wider, every `aim` re-homes on
`SW_RAIL_LOW` first and `aim_tol` is at least 2.0 deg. A missing bump while driving is `ANGLE_SENSOR` "SW_INDEX bump
overdue (index cam or contact)" within 1.5 measured bump periods (before the first bump of a run: (60 + 16) deg at
half the nominal speed, x 1.5, plus 0.3 s), so a cock that stops a bump early never reaches the release. Index mode
is chosen only at start-up.

AS5600 read errors: missed reads no longer lose travel (the step allowed grows with the time since the last good
read). A lasting jump that had to be taken without counting it is `ANGLE_SENSOR` "AS5600 angle jumped while turning
(travel lost)" while the wiper turns; when idle (for example a hand-turned shaft) the base forgets the aim and turns a
`COCKED` or `COCKING` phase into `UNKNOWN` with the drawn hint.

## 10. Changes for the Jetson in base-1.2

What the Jetson worker must change (against the base-1.0 version of this file):

1. Version strings: `HELLO`, `BOOT`, `ping`, `status` and `get fw` say `base-1.2.1`; `get params` says `1.2.7`.
2. Mission order (D22): after `FIRED` and `RESET` and a go landing verdict, send `wrap` straight away, in LAUNCH (no
   `mode NEUTRAL` first; `wrap` does the centering jog and the launch-exit checks itself). Then wait for ANCHORED,
   send `tidy`, then `mode REEL`, then the tow. The old order (`mode NEUTRAL`, `reel until slip`, `wrap`) is gone.
3. `wrap` ends at the WRAPPED park 314, not in REEL: one `WRAPPED` event, and NO `MODE` event after it. Telemetry then
   shows `drum_pos` `WRAPPED`, `mode` `NEUTRAL`, `wraps` 7. `wrap` no longer returns `NOT_IN_NEUTRAL`; from LAUNCH it
   can return `LAUNCH_BUSY`. It takes about 35 to 40 s (drum 8.1 deg/s): wait up to 90 s.
4. `mode REEL` is now the normal step after `tidy` (fly-in from the WRAPPED park, `MODE` ok REEL). It is still refused
   with `NOT_WRAPPED` below N_TO_R.
5. New commands: `tidy` (`ms`, `duty`; event `TIDIED`; refusals `BAD_ARG`, `NOT_IN_NEUTRAL`, `POSITION_UNKNOWN`),
   `retrieve` (`duty`, `max_n`, `max_s`; event `SEATED`; fault `DART_STUCK`), `decock` (= `cal decock`; event `RESET`).
6. `reel` and `tidy` at the WRAPPED park run the take-up only, as in NEUTRAL (`retrieve` is refused there, item 34). A wiper `jog` there is
   refused with `SHIFTING`.
7. Faults and `estop` with the wraps on (WRAP, N_TO_R, the WRAPPED park, REEL) HOLD the drum (params 1.2.5 on, D27):
   `MODE` `ok:false`, `err:"DRUM_HELD"`, `why:"wraps on: the drum holds, the operator unwraps when safe"` (exactly at
   the WRAPPED park `ok:true`). Nothing moves; the operator decides (`unwrap`, or resume the reel-in). Elsewhere the
   drum goes to NEUTRAL as before.
8. Timing: the drum turns at about 8.1 deg/s (was 24). `mode` LAUNCH to NEUTRAL about 5 s, WRAPPED to REEL about 5 s,
   `unwrap` from REEL about 40 s. Use a timeout of at least 90 s for `wrap` and `unwrap`, and send `ping` during them
   if `host_timeout_s` is on.
9. Drum geometry: sector names and angles of params.DRUM (section 5), parks LAUNCH 0, NEUTRAL 41, WRAPPED 314,
   REEL 347, end stops -4 / 352 (were -8 / 358). Centering jog 8 deg (was 30), dog window 16 deg (was 60).
10. New `SWITCH_DISAGREE` cause "collar contact disagrees with the drum code" (dog contacts checked during every drum
    move); "collar still engaged at NEUTRAL" also at the WRAPPED park. "target code came far too early" now only
    with a trusted start angle.
11. Current trips (D8): aim now has its own 5 A trip; the tow (`until:"tension"`) trips at 6.5 A (was 5 A), new
    `set` key `tow_trip_a`; since D34 the tow trip also rises with `max_n` (section 2, `reel`). The
    `OVERCURRENT` `why` has one decimal: `wiper over 6.5 A`, `wiper over 5.0 A`, `wiper over 2.5 A`.
12. Tension estimate: `k_n_per_a x (amps - free_a) + tension_offset_n`, defaults 17.0 and -10.4 (was 8 and no
    offset), so `LIMIT` `tension_n` values change. New `set` / `get` key `tension_offset_n` (-50..50).
13. Drum jog: the automatic end-stop guard stops 2 deg inside the end stops (-2 and 350); a jog toward an end stop
    with an untrusted drum angle is refused with `POSITION_UNKNOWN`.
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
23. Drum jog: refused with `WRAPS_ON` inside WRAP and with `SWITCH_DISAGREE` when a dog contact disagrees; a running
    drum jog stops on entering WRAP (`ABORTED` `why:"wrap window: use wrap or unwrap"`) and faults on a disagreeing
    dog contact.
24. A drum move that overshoots its target sector comes back to the edge once (new `DRUM_TIMEOUT` text "drum did not
    come back to the target sector").
25. After a power cut while cocking or firing, `cock` replies `LOADED_UNKNOWN`: recover with `decock` and
    `"force":true` once the carriage is checked (section 9).
26. Fly-in, LAUNCH wiggle and the REEL take-up of the centering jog run at 25 % (was 20 %).
27. `stop` / `estop` with a bad numeric id are still carried out (reply `BAD_ID`, `id` null). `cal current`, `cal free`
    and `cal aim` reply `OUT_OF_RANGE` beyond the `set` key ranges. `get` gives up to 7 significant digits.
28. `retrieve` and `reel` until `seated` take an optional `max_s` (5 to 600 s, `BAD_ARG` outside). The default time
    limit is the longer of 20 s and 10 s + 1.5 x `line_m` / v (v from the duty, about 0.058 m/s at 0.3; 30 s with
    `line_m` unknown, at most 600 s), and the `DART_STUCK` why says
    the seconds: "final approach: no SW_SEATED in N s". `line_m` does not count during take-up-only running.
29. Mission step 14 (BASE_SPEC 10.1): send `stop` first (`ABORTED`, the worm holds the line), then the dart release.
30. After a lying drum code near the wrap window the wraps are doubtful: `mode`, `wrap` and drum `jog` reply
    `WRAPS_ON` until an `unwrap` reaches NEUTRAL.
31. New `ANGLE_SENSOR` why "AS5600 angle jumped while turning (travel lost)". Index mode (D12): every `aim` re-homes,
    `aim_tol` at least 2.0 deg, a missed bump faults within 1.5 bump periods.
32. `decock` with the phase `UNKNOWN` and `SW_HOME` and `SW_GAP` both closed: phase `REST` and
    `RESET` `{"ok":true,"how":"decock","why":"SW_HOME and SW_GAP closed: at rest"}`. With `SW_GAP` open it is still
    `RESET` `ok:false` "carriage home: nothing to decock, cock finds the angle".
33. `aim` targets at the very ends of `pitch_range` are moved a quarter of `aim_tol` inside (reachable now).
34. D39: the tow limit is at most 70 N: `set limit_n` range 1..70 (`OUT_OF_RANGE` above), `reel` `max_n` over 70 is
    `BAD_ARG` (it used to be capped at 110). `retrieve` and `reel` until `seated` run only in the NEUTRAL sector;
    anywhere else, the WRAPPED park included, `WRAPS_ON`. From LAUNCH send `mode NEUTRAL` first.
35. `unwrap` always sends `UNWRAPPED` then `MODE`, also when the drum is already at NEUTRAL.
36. `SW_ARM` polarity (cad-reel): closed while the cup is away, open for about +-5 deg at home. Telemetry `W` present
    means the cup is away from home.
37. After a power cut while the wiper turned with the launch train at rest, boot keeps phase `REST` when `SW_HOME` and
    `SW_GAP` are closed (systest F15).
38. Wiper jog backward is refused with `BACKSTOP` everywhere except at the LAUNCH park (BASE_SPEC 10).
39. `cock` from `REST` and `cal cock_rest` reply `SWITCH_DISAGREE` when `SW_GAP` is fitted and open
    (`cal cock_rest` with `"force":true` still overrides).
40. Version: `get params` says `1.2.7`.
