# C1 as implemented by the base firmware

Source: branch `origin/w/fw-base`, commit `70a01f7`, firmware `base-1.0` (`GB_FW_VERSION`, base_logic.h:22).
There is no `firmware/base/PROTOCOL.md` on that branch yet; this document is read from the code. Line numbers are
`base_logic.h` (BL), `config.h` (CF), `grappstar_base.ino` (INO), `json_lite.h` (JL), `persist.h` (PS) on that commit.
Where the firmware differs from the draft contract C1 in `knowledge/WORKPLAN_2026-11-03.md`, the firmware is taken as
the truth; the differences are listed at the end.

The Jetson side follows this: `grappstar/comms/base.py` (BaseClient) and `grappstar/comms/mock_base.py` (MockBase,
same commands, checks, events, telemetry, states and phases).

## 1. Link and framing

- USB CDC on the DevKitC native port, baud ignored (INO:122). JSON objects, one per line, `\n` (or `\r`) ends a line.
- At most 256 bytes per line in both directions (JL:13). A longer incoming line is answered
  `{"id":null,"ok":false,"err":"TOO_LONG"}` (INO:183, INO:199, BL:714).
- Unparseable line: `{"id":null,"ok":false,"err":"BAD_JSON"}` (BL:715). The parser takes one flat object; nested
  values are skipped; at most 12 keys are kept (JL:14, JL:138), string values are truncated to 63 bytes, keys to 15
  (JL:19-21). A duplicate key: the last one wins (JL:137).
- Every command line gets exactly one reply; the reply is written first, then any events the command raised
  (INO:186-187). Long actions reply when accepted; completion is an event (BL:709).
- `id`: echoed. Missing, null or boolean: `null`; an integer number is printed as an integer; a string is echoed
  (BL:717-720). If the reply would exceed 256 bytes (very long string id) it is cut to id, ok, err (or token)
  (BL:758-765), or `{"id":null,"ok":false,"err":"REPLY_OVERFLOW"}`.
- No `cmd` string: `NO_CMD` (BL:722). Unknown command: `UNKNOWN_CMD` (BL:751).
- Events go through a 24-line ring; when the host does not read, the newest event is dropped silently (INO:35-36,
  INO:85-89). Telemetry is dropped (never queued) when the USB buffer is full (INO:208).

Jetson note: a reply with `id: null` belongs to the oldest command still waiting (the firmware answers in order).
BaseClient does that matching.

## 2. Replies

`{"id":17,"ok":true,...}` or `{"id":17,"ok":false,"err":"CODE"}` (BL:752-756). Extra fields on success:

| command | extra reply fields |
| --- | --- |
| `ping` | `fw` ("base-1.0"), `t` (ms since boot) (BL:755) |
| `status` | `phase`, `aimed`, `aim_known`, `drum_known`, `wraps_known`, `magnet` (NONE, WEAK, STRONG, OK), `glitches`, `hx` (load cell live), `peak_a`, `fw`, and `why` (fault reason, at most 32 chars) when faulted (BL:784-791). No telemetry object. |
| `arm` | `token` (six digit integer, 100000..999999), `valid_ms` (3000) (BL:544, BL:754, CF:121) |
| `get` | `val`, or `min` and `max` for `pitch_range` (BL:767-783) |

## 3. Commands

"Idle check" (BL:460-467), in this order: FAULT or SAFING gives `FAULTED`; any state other than IDLE or HOLD gives
`BUSY`; before the boot decision `BOOTING`; battery below 6 V (USB only) `NO_POWER` (CF:117); for commands that
turn the wiper with the AS5600 lost `ANGLE_SENSOR`.

| command | args (ranges) | refusals, in check order | what it does, completion |
| --- | --- | --- | --- |
| `ping` | | | reply only |
| `status` | | | reply with the status fields |
| `stop` | | never refused | wiper and drum off now (both worms self-lock). See section 7. |
| `estop` | | never refused | stop, then drum to NEUTRAL if safe. See section 7. |
| `clear` | | `BUSY` (SAFING), `STILL_PRESENT` | clears a fault (BL:808-815). ok (no-op) when not faulted. |
| `mode` | `to`: LAUNCH, NEUTRAL, REEL | `BAD_ARG`, idle check, `HOLDING`, `POSITION_UNKNOWN` (wraps unknown), `WRAPS_ON`, `SWITCH_DISAGREE`, `POSITION_UNKNOWN` (drum unknown), `LAUNCH_BUSY` (in LAUNCH, train not at rest), `BUSY` (wiper running) (BL:468-485) | state MODE (drum move); `MODE` event. Already there: ok and `MODE` at once. Clears `aimed` and the arm token. |
| `aim` | `deg` (number, inside `get pitch_range`, 15..60 with the placeholder table) | idle check (+AS5600), `HOLDING`, `NOT_IN_LAUNCH`, `PHASE_UNKNOWN`, `NOT_AT_REST`, `OUT_OF_RANGE`, `AIM_UNKNOWN` (aim unknown and no SW_RAIL_LOW) (BL:486-504) | state AIM; homes on SW_RAIL_LOW first if the aim is unknown; turns the aim train one way only (forward travel to the first crossing of the target); `AIMED` event. |
| `cock` | | idle check (+AS5600), `HOLDING`, `NOT_IN_LAUNCH`, `NO_SENSOR` (no SW_HOME), `SWITCH_DISAGREE` (phase vs SW_HOME), `LOADED_UNKNOWN` (phase unknown and carriage drawn: run `cal decock`) (BL:513-528) | state COCK; `COCKED` event. Already cocked: ok and `COCKED` at once. Phase UNKNOWN with the carriage home: slow creep until SW_HOME opens, then cocks normally. |
| `arm` | | idle check (+AS5600), `HOLDING`, `NOT_IN_LAUNCH`, `WRAPS_ON`, `NOT_COCKED`, `NOT_AIMED` (not aimed, or pitch off target by more than aim_tol 0.75 deg), `NOT_SEATED` (or `NO_SENSOR`) when require_seated (BL:531-547) | returns the token. |
| `fire` | `token` (number or numeric string) | `BAD_ARG`, `NOT_ARMED`, `TOKEN_EXPIRED`, `BAD_TOKEN` (and the token is burnt), then every `arm` check again (BL:548-557) | state FIRE, phase FIRING; `FIRED` at the release, `RESET` at rest. The token is single use. |
| `wrap` | | idle check, `HOLDING`, `NOT_IN_REEL` (drum not on the REEL detent), `WRAPS_ON`, `BUSY` (BL:558-570) | state WRAP; lays `wrapsTarget` = 4 wraps (CF:89); `WRAPPED` event. |
| `unwrap` | | idle check, `HOLDING`, `POSITION_UNKNOWN`, `NOT_WRAPPED` (drum below REEL), `BUSY` (BL:571-581) | state UNWRAP back to the REEL detent; `UNWRAPPED`. At REEL with no wraps: ok and `UNWRAPPED` at once. |
| `reel` | `duty` 0 < d <= 1 (default 220/255), `max_n` > 0 (default `limit_n` 20, capped at 150), `until`: stop (default), seated, tension | idle check (+AS5600; allowed from HOLD), `NOT_WRAPPED` (drum_pos must be WRAPPED), `BAD_ARG` (until, duty, max_n), `NO_SENSOR` (until seated without SW_SEATED) (BL:582-596) | state REEL. See section 8. until seated with the dart already seated: ok and `SEATED` at once. |
| `jog` | `motor`: wiper or drum, `dir`: 1 or -1, `ms` > 0 (default 300, capped at 1500) | `BAD_ARG`, idle check (+AS5600 for the wiper), `HOLDING`, wiper: `BUSY` (drum running), `WOULD_FIRE`, `NOT_AT_REST`, `BACKSTOP`; drum: `BUSY` (wiper running) (BL:605-630) | state JOG for ms. See section 9. |
| `cal` | `what` (below) | `BAD_ARG` (missing or unknown what) | see section 10 |
| `set` | `key`, `val` (number or bool) | `BAD_ARG`, `OUT_OF_RANGE`, `BAD_KEY`, `BUSY` (enc_sign while moving) (BL:693-706) | see section 10 |
| `get` | `key` | `BAD_ARG`, `BAD_KEY` | `val` |

`aim` needs the launch train at REST (`aimNeedsRest`, CF:125, BL:491): the backward drive that aims would let a
drawn spring unwind. So aim BEFORE cock. Cocking (forward drive) does not move the aim train, so `aimed` survives
`cock`.

## 4. Error codes (reply `err`)

| code | when |
| --- | --- |
| `TOO_LONG` | line over 256 bytes |
| `BAD_JSON` | not one flat JSON object |
| `NO_CMD` | no `cmd` string |
| `UNKNOWN_CMD` | unknown command |
| `REPLY_OVERFLOW` | the short fallback reply still did not fit |
| `BAD_ARG` | missing or invalid argument (mode `to`, aim `deg`, fire `token`, reel duty/max_n/until, jog motor/dir/ms, cal what or deg, set key/val, enc_sign not 1 or -1) |
| `BAD_KEY` | set/get of an unknown key |
| `OUT_OF_RANGE` | aim outside the pitch range (or no crossing in a cycle), set value outside its range |
| `FAULTED` | state FAULT or SAFING |
| `BUSY` | a motion is running (any state but IDLE and HOLD), or the other motor is running, or `clear` while SAFING, or `cal current`/`aim_home`/`cock_rest`/`set enc_sign` while moving |
| `BOOTING` | the first 60 ms after reset, before the boot decision (BL:864-867) |
| `NO_POWER` | battery below 6 V (USB only) |
| `ANGLE_SENSOR` | AS5600 lost, for aim, cock, arm, fire, reel, wiper jog, cal decock |
| `HOLDING` | in HOLD: mode, aim, cock, arm, fire, wrap, unwrap, jog |
| `POSITION_UNKNOWN` | drum or wraps unknown (after a power cut mid-move): run `cal drum` |
| `WRAPS_ON` | mode change, wrap, arm, fire or cal drum with wraps on |
| `SWITCH_DISAGREE` | drum code impossible, or SW_HOME disagrees with the launch phase (cock, cal cock_rest) |
| `LAUNCH_BUSY` | mode change while in LAUNCH with the launch train not at REST (cocked, mid cycle, unknown) |
| `NOT_IN_LAUNCH` | aim, cock, arm, fire, cal decock outside the LAUNCH detent |
| `PHASE_UNKNOWN` | aim with the launch phase unknown (first boot, power cut mid cycle, stop mid cock or fire) |
| `NOT_AT_REST` | aim, or a backward wiper jog in LAUNCH, with the train not at rest |
| `AIM_UNKNOWN` | aim with the aim unknown and no SW_RAIL_LOW; cal aim with the aim unknown |
| `NO_SENSOR` | the contact needed is not fitted (SW_HOME for cock/decock, SW_SEATED for arm or reel until seated, HX711 for cal tare) |
| `LOADED_UNKNOWN` | cock with the phase unknown and the carriage drawn |
| `NOT_COCKED` | arm or fire with phase not COCKED (or SW_HOME closed) |
| `NOT_AIMED` | arm or fire not aimed, or pitch off target by more than aim_tol |
| `NOT_SEATED` | arm or fire without SW_SEATED (re-checked at fire) |
| `NOT_ARMED` | fire without a live token (never armed, expired, used, burnt, or cleared by stop / a mode change / aim / cock / jog / fault) |
| `TOKEN_EXPIRED` | fire more than 3 s after arm, if the watchdog has not already cleared the token (it normally has: expect `NOT_ARMED`, BL:371) |
| `BAD_TOKEN` | wrong token (burns the arm) |
| `NOT_IN_REEL` | wrap outside the REEL detent |
| `NOT_WRAPPED` | reel without drum_pos WRAPPED; unwrap with the drum below REEL |
| `WOULD_FIRE` | forward wiper jog in LAUNCH with phase COCKED, UNKNOWN or FIRING |
| `BACKSTOP` | backward wiper jog in REEL or WRAPPED (sprag #2: the capstan never turns out) |
| `NOT_RUNNING` | cal free while the wiper is not running (or ran less than 0.8 s) |
| `STILL_PRESENT` | clear while the cause remains (ANGLE_SENSOR still lost, SWITCH_DISAGREE code still impossible, BROWNOUT battery still low) |

## 5. Events

Every event built by the logic carries `"t"` (ms since boot) right after `"evt"` (BL:191-192). An event that would
not fit 256 bytes is sent as `{"evt":"OVERFLOW"}` (BL:193).

| event | fields | when |
| --- | --- | --- |
| `HELLO` | `fw`, `reset` (POWERON, BROWNOUT, PANIC, WATCHDOG, SOFTWARE, USB, OTHER), `hx711`, `as5600` (no `t`) | the sketch, at every reset (INO:148-151) |
| `BOOT` | `fw`, `record` (NVS record valid), `was_moving`, `drum_known`, `wraps_known`, `aim_known`, `phase` | 60 ms after reset, the boot decision (BL:275-277) |
| `MODE` | `ok` true, `mode`, `drum_pos` | drum arrived (mode, estop, safing, cal drum) or already there (BL:194-197, BL:899-900, BL:1053) |
| `MODE` | `ok` false, `mode`, `drum_pos`, `err` = a reason string (not a code), e.g. "drum did not reach the detent in time" | the safing / estop drum move itself failed (BL:452-454) |
| `MODE` | `ok` false, `mode`, `err` "DRUM_HELD", `why` | a fault or estop could not send the drum to NEUTRAL (BL:214, BL:804) |
| `AIMED` | `ok`, `pitch_deg`, `target_deg`, `err` (OUT_OF_RANGE, AIM_MISS) when not ok | aim finished (BL:930-934); AIM_MISS after 3 tries outside aim_tol (BL:921) |
| `COCKED` | `ok` (always true in this firmware), `cock_deg` | cocked (BL:529, BL:954), or cock while already cocked |
| `FIRED` | `pitch_deg`, `cock_deg` (release angle) | SW_HOME closed at the release: the shot (BL:965-966). No `ok`. |
| `RESET` | `ok` true | launch train back at rest after a shot (BL:977) |
| `RESET` | `ok` true, `how` "decock" | cal decock finished (BL:1044) |
| `WRAPPED` | `ok` true, `wraps` | wrap finished (BL:991) |
| `UNWRAPPED` | `ok` true | unwrap finished, or already unwrapped (BL:574, BL:1004) |
| `SEATED` | (only `t`) | reel until seated saw SW_SEATED close; line_m set to 0 (BL:597, BL:1010) |
| `LIMIT` | `tension_n`, `max_n`, `why` ("tension limit", "stalled at the limit", "over current at the limit"), `holding` | reel tension limit: holding true = state HOLD (until tension), false = IDLE (BL:598-604) |
| `ABORTED` | `what` (state name: MODE, AIM, COCK, FIRE, WRAP, UNWRAP, REEL, HOLD, JOG, CAL, DECOCK), `why` (stop, estop, timeout, would pass the release tooth) | stop or estop of a running state, reel timeout 240 s, wiper jog stopped before the release tooth (BL:198-200, BL:797, BL:1017, BL:1028) |
| `FAULT` | `code`, `why` | a fault (BL:207) |
| `STALL` | (only `t`) | right after a FAULT with code STALL (BL:208) |
| `OVERFLOW` | none | an event did not fit |

Failures of AIM, COCK, FIRE, WRAP, UNWRAP and REEL motions come as a `FAULT` event (then the safing `MODE`), never as
an `ok: false` completion event, except AIMED (OUT_OF_RANGE, AIM_MISS) and the MODE cases above. `COCKED`, `WRAPPED`,
`UNWRAPPED` and `RESET` carry `ok` but it is always true in this firmware.

## 6. Telemetry

`{"tel":{...}}`, 10 Hz while still, 20 Hz while anything moves (`moving()`: a motor running or a state other than
IDLE, HOLD, FAULT) (BL:853-857, CF:123). Normal lines are about 200 bytes; over 256 the line is rebuilt with fewer
decimals, and if still too long sent as a short form with `"trunc": true` (BL:820-827). Numbers have trailing zeros
trimmed (12.50 prints 12.5, 3.00 prints 3) (JL:175-185).

| key | type | meaning |
| --- | --- | --- |
| `state` | string | STATE_NAME, section 7 |
| `mode` | string | LAUNCH, NEUTRAL, REEL (also while WRAPPED), NONE (between or unknown): derived from the drum, not the last command (BL:152-156) |
| `drum_deg` | number (1 dp) or null | drum estimate, deg from the LAUNCH detent; null when unknown |
| `drum_pos` | string | LAUNCH, NEUTRAL, REEL, WRAPPED, BETWEEN, UNKNOWN (BL:145-151) |
| `wiper_deg` | number (1 dp) or null | wiper multi-turn angle |
| `rpm` | number (1 dp) or null | wiper speed |
| `amps` | number (1 dp) | wiper current (BTS7960 IS) |
| `pitch_deg` | number (2 dp) or null | rail pitch from the aim table; null when the aim is unknown |
| `cock` | number 0..1 (2 dp) or null | REST 0, COCKING fraction, COCKED and FIRING 1, RESETTING falling to 0, UNKNOWN null |
| `wraps` | integer or null | capstan wraps laid; null when unknown |
| `line_m` | number (2 dp) or null | line out estimate; null when unknown, which is from FIRED until `set line_m` or SEATED (the payout is not measured) |
| `tension_n` | number (1 dp) or null | HX711 only: null without a live load cell, even though the firmware estimates the tension from the current for LIMIT |
| `sw` | object | closed contacts only, absent = open, one-letter keys (see the warning below) |
| `fault` | string or null | fault code |
| `vbat` | number (1 dp) or null | battery volts, null when the divider is not fitted |

Contact letters declared in config.h (CF:14, `SW_TEL`, "telemetry keys (PROTOCOL.md)"): `a` SW_DRUM_A, `b` SW_DRUM_B,
`c` SW_DRUM_C, `S` SW_SEATED, `H` SW_HOME, `L` SW_RAIL_LOW, `W` SW_WRAP_HOME. Drum codes: LAUNCH = a, NEUTRAL = a+b,
REEL = b (CF:29).

WARNING, firmware bug: `telBuild` does not use `SW_TEL`; it uses a stale local table
`{"L","N","R","W","S","C","A"}` indexed by the new contact enum (BL:844). As written, SW_SEATED is sent as `W` and
SW_HOME as `S`, so a reader of `S` would take "carriage home" for "dart seated". The fix is one line: use
`SW_TEL[i]` at BL:845. MockBase and BaseClient (`tel_seated`, `tel_contact`) use the `SW_TEL` letters.

## 7. States, phases, stop, estop, clear, faults

States (STATE_NAME, BL:28-30): `IDLE`, `MODE` (drum move for mode or estop), `AIM`, `COCK`, `FIRE`, `WRAP`, `UNWRAP`,
`REEL`, `HOLD`, `JOG`, `CAL` (cal drum home search), `SAFING` (fault drum move to NEUTRAL), `FAULT`, `DECOCK`.

Launch phases (PHASE_NAME, BL:32-33, in `status.phase` and `BOOT.phase`): `REST` (carriage forward, SW_HOME closed),
`COCKING`, `COCKED` (stopped at 560 deg, just short of the relieved tooth), `FIRING`, `RESETTING`, `UNKNOWN`.

HOLD (BL:598-604, BL:1019-1023): entered only by `reel until tension` at the limit (or a stall or over current at
the limit). The wiper is off; its worm holds the line. With a live load cell, HOLD winds again by itself (back to
REEL, no event) when the tension falls below 60 % of max_n (CF:107), then gives a new LIMIT. From HOLD:
`reel` is accepted (a new reel starts at once); `mode`, `aim`, `cock`, `arm`, `fire`, `wrap`, `unwrap`, `jog` are
refused `HOLDING`; `stop` or `estop` leaves HOLD (to IDLE, with `ABORTED {what: HOLD}`). `ping`, `status`, `get`,
`set`, `cal current/tare/aim/aim_home/cock_rest` work. So yes: send `stop` before any mode change or unwrap.

`stop` (BL:792-807): wiper and drum halt, the arm token dies. In a running state: `ABORTED {what, why: stop}`, state
IDLE; a stop during COCK or FIRE leaves the phase UNKNOWN (re-establish with `cock`, or `cal decock` if the carriage
is drawn). In FAULT or SAFING the state stays FAULT (a stop during SAFING halts the drum where it is). Never refused.

`estop` (same code): stop, then if not faulted and the drum may go to NEUTRAL safely, a drum move to NEUTRAL (state
MODE, then `MODE ok` and IDLE). If it may not (wraps on, launch train not at rest, drum unknown), the drum holds and
`MODE {ok: false, err: DRUM_HELD, why}` is sent. The ABORTED event says `why: estop`.

`clear` (BL:808-815): only in FAULT; refused `STILL_PRESENT` while the cause remains; clears the latches, state IDLE.
It does not move anything and does not change the phase or the drum.

Faults (BL:24-26, raised by `raise`, BL:202-216): the first fault wins. Wiper off, arm token dead, `FAULT {code,why}`
(plus `STALL`). Then, if `safeToNeutral` allows (BL:218-227), the drum goes to NEUTRAL in state SAFING and the state
becomes FAULT with `MODE ok NEUTRAL`. It holds (state FAULT at once, and `MODE {ok:false, err:DRUM_HELD, why}` unless
already at NEUTRAL) when: a drum fault (DRUM_TIMEOUT, DRUM_SENSOR), the drum code contacts disagree, the drum position
or the wraps are unknown, wraps are on ("the capstan holds the line"), or in LAUNCH with the train not at rest ("the
wiper worm holds the spring").

| fault | raised when |
| --- | --- |
| `STALL` | wiper driven hard (over 30 % duty, past the 300 ms ramp) with the shaft under 15 deg/s for 400 ms (drives.h:19-21, BL:361). During `reel until tension` this is a LIMIT instead (BL:378-381). |
| `OVERCURRENT` | two 5 ms samples over trip_a (17 A; cock_trip_a 15 A while cocking) (CF:52-53, BL:360). During `reel until tension` this is a LIMIT instead (BL:374-377). |
| `DRUM_TIMEOUT` | drum did not leave its detent or reach the target in time (expected x 2 + 1.5 s) after one back-off and wiper nudge retry; wrap, unwrap or cal drum too long (BL:446-451, BL:996, BL:1007, BL:1055) |
| `DRUM_SENSOR` | an unexpected detent code on the way, target code too early or lost after arriving, wrap index not seen (BL:415, BL:425, BL:434, BL:993-995, BL:1002, BL:1006) |
| `ANGLE_SENSOR` | AS5600 lost while the wiper runs or AIM, COCK, FIRE, REEL is active (BL:362) |
| `SWITCH_DISAGREE` | impossible drum code for over 100 ms, a drum code contact chattering during a drum move, SW_HOME opening at the wrong angle (BL:363-366, BL:945) |
| `BROWNOUT` | battery under vbat_low (9.9 V) for 300 ms while a motor runs (BL:368-369) |
| `BAD_STATE` | the mechanism did not do what the sensors expect: SW_RAIL_LOW never closed, aim ran too far, carriage not drawn or slipped, no release, release at the wrong angle, SW_HOME wrong while resetting, cock or fire too long, decock never saw SW_HOME (BL:909, BL:928, BL:939-957, BL:967-980, BL:1046) |

A fault during cocking leaves the phase COCKING and the drum held in LAUNCH; after `clear`, `cock` resumes from
where it stopped (or `cal decock` unloads). A fault during a reel with wraps on leaves the drum WRAPPED.

## 8. Reel: how it ends

`reel` needs drum_pos WRAPPED (BL:584). The duty tapers off above 70 % of max_n (BL:1014); with until seated and a
known line_m under 1 m the duty is cut to 110/255 (BL:1015). The tension is the load cell if live, else the current
estimate `(amps - free_a) * k_n_per_a` (BL:293-296), ignored for the first 350 ms (ramp inrush).

| until | ends with |
| --- | --- |
| `seated` | SW_SEATED closes: `SEATED`, IDLE, line_m 0. Tension limit: `LIMIT {holding:false}`, IDLE. |
| `tension` | tension limit (or stall or over current): `LIMIT {holding:true}`, HOLD. It does NOT stop at SW_SEATED. |
| `stop` | only `stop` (`ABORTED`), the tension limit (`LIMIT {holding:false}`, IDLE) or a fault. It does NOT stop at SW_SEATED. |

Any reel: 240 s without ending gives `ABORTED {what: REEL, why: timeout}`, IDLE (BL:1017).

## 9. Jog rules (BL:605-630, BL:1024-1036)

- Deadman: a repeat of the same motor and dir while jogging extends it (ms from now); it stops by itself at the end
  (no event). ms default 300, capped at 1500.
- Wiper, LAUNCH, forward (cock direction, +1): refused `WOULD_FIRE` when COCKED, UNKNOWN or FIRING. From REST it may
  draw the carriage (phase becomes COCKING) but stops before the release tooth with `ABORTED {what: JOG}`.
- Wiper, LAUNCH, backward (aim direction, -1): refused `NOT_AT_REST` unless REST; clears `aimed`.
- Wiper, REEL or WRAPPED, backward: refused `BACKSTOP` (sprag #2). Forward winds the capstan.
- Drum: any direction, clears `aimed`; refused `BUSY` while the wiper runs.

## 10. cal, set, get, persistence

`cal` (BL:631-672), `what`:
- `current`: zero the IS offset (`BUSY` while the wiper runs).
- `tare`: zero the load cell (`NO_SENSOR` without one).
- `free`: take the running current as the free current (`NOT_RUNNING` unless the wiper ran 0.8 s).
- `aim` + `deg`: the rail is really at deg now: sets aim_offset (`AIM_UNKNOWN`, `BAD_ARG`).
- `aim_home`: declare the aim train at home (`BUSY` while moving).
- `cock_rest`: declare the launch train at rest (`BUSY`, `SWITCH_DISAGREE` if SW_HOME is open). Needed once on a new
  build (phase UNKNOWN at first boot).
- `decock`: idle check, `NOT_IN_LAUNCH`, `NO_SENSOR`; carriage already home: ok, no event; otherwise state DECOCK,
  the wiper runs backward so the spring unwinds under the worm until SW_HOME closes, then `RESET {how: decock}`. The
  aim is forgotten (aim_known false): aim again.
- `drum`: idle check, `WRAPS_ON` (wraps known and on); at a detent: `MODE ok` at once; else home search toward LAUNCH
  until a detent code closes (state CAL), `MODE ok`. Allowed with the wraps unknown (it unwraps on the way).

`set key val` (BL:675-706): `limit_n` 1..150, `k_n_per_a` 0.1..200, `free_a` 0..10, `amps_per_volt` 1..200,
`offset_a` 0..5, `hx_n_per_count` 1e-7..1, `drum_dps` 5..720, `aim_tol` 0.1..5, `aim_offset` -20..20, `line_m`
0..100 (also makes line_m known), `trip_a` 3..19, `cock_trip_a` 3..19, `vbat_low` 6..12, `aim_home_deg` -720..720,
`require_seated` (bool or number), `enc_sign` (1 or -1). `get` takes the same keys plus `fw`, `params` (params.py
version), `pitch_range` (`min`, `max`).

Persistence (PS, BL:229-279, BL:887-891): a position record in NVS is written at every motion start with
`moving = 1`, and again with the final positions once everything has been still for 300 ms (no writes while idle).
After a power cut mid-motion the base trusts only what its sensors prove: drum from a detent code, else unknown
(`POSITION_UNKNOWN`: run `cal drum`); wraps unknown after a cut mid-wrap; aim and phase from the record only if the
AS5600 angle agrees within 3 deg; a mid-cycle phase (COCKING, FIRING, RESETTING) becomes UNKNOWN; SW_HOME must agree
with REST or COCKED. The `BOOT` event reports the result. Bench settings changed by `set` or `cal` are kept in a
second record (`Tun`, PS:51-61) written when the base is still.

## 11. Order of operations

A shot:

1. `mode LAUNCH`, wait `MODE ok` (refused `WRAPS_ON` with wraps on, `HOLDING` in HOLD).
2. `aim deg`, wait `AIMED ok` (needs phase REST; homes first if the aim is unknown).
3. `cock`, wait `COCKED` (about 3 s).
4. `arm`, take `token` (needs COCKED, aimed within 0.75 deg, SW_SEATED, no wraps, LAUNCH).
5. `fire token` within 3 s, wait `FIRED` (the shot), then `RESET` (train at rest; state FIRE until then).
6. After the shot: `aimed` is false and `line_m` is null.

Calling off a cocked shot: `cal decock`, wait `RESET {how: decock}`, then aim again. A cocked base cannot change mode
(`LAUNCH_BUSY`) and cannot aim (`NOT_AT_REST`).

Retrieve and tow: `mode REEL` (from REST only if in LAUNCH), `wrap` (WRAPPED, about 25 s), `reel ... until tension`
(LIMIT, HOLD), `stop` before release, `reel ... until seated` (SEATED), `unwrap` (UNWRAPPED), `mode NEUTRAL` or
`LAUNCH`.

## Differences from C1 draft

1. New command `cock`. `fire` no longer cocks: it only releases an already cocked train. `arm` needs COCKED, aimed and
   seated; aim must come before cock.
2. New `cal` actions: `decock`, `drum`, `cock_rest`, `aim_home`, `aim`, `current`, `tare`, `free`; a fixed `set`/`get`
   key list with ranges (section 10).
3. New events: `HELLO`, `BOOT`, `RESET` (after every shot and decock), `ABORTED`, `OVERFLOW`. Every logic event has a
   `t` field. `MODE`, `AIMED`, `COCKED`, `WRAPPED`, `UNWRAPPED`, `RESET` carry `ok`; `MODE` and `AIMED` can be
   `ok: false` with `err`.
4. `AIMED` has `pitch_deg` and `target_deg` (no `deg`). `FIRED` has `pitch_deg` and `cock_deg`. `LIMIT` has
   `tension_n`, `max_n`, `why`, `holding` (no `line_m`).
5. Telemetry `sw` is closed contacts only with one-letter keys (`S` seated, `H` carriage home, ...), not an object of
   named booleans. And the firmware currently sends the wrong letters (section 6 warning).
6. Telemetry `mode` is derived from the drum (`NONE` between detents), not the last commanded mode.
7. `drum_deg`, `wiper_deg`, `rpm`, `pitch_deg`, `cock`, `wraps`, `line_m` can be null (unknown). `wraps` is an
   integer. `line_m` is null from FIRED until `set line_m` or SEATED.
8. `tension_n` is null without a load cell, as in the draft, but the firmware still limits on a current based
   estimate and reports it in `LIMIT`.
9. `status` returns its own fields (phase, aimed, aim_known, ...) and no telemetry object.
10. `arm` returns an integer token plus `valid_ms`; an expired token normally gives `NOT_ARMED`.
11. `estop` and faults send the drum to NEUTRAL only when safe; otherwise the drum holds and `MODE {ok:false,
    err:DRUM_HELD, why}` explains. The draft said a fault always goes NEUTRAL.
12. Error codes are different: `FAULTED` (not FAULT), `WRAPS_ON` (not WRAPPED), `HOLDING` (not UNDER_LOAD),
    `OUT_OF_RANGE` (not RANGE), `BAD_ARG` (not BAD_ARGS), plus `LAUNCH_BUSY`, `PHASE_UNKNOWN`, `NOT_AT_REST`,
    `NOT_COCKED`, `NOT_AIMED`, `LOADED_UNKNOWN`, `POSITION_UNKNOWN`, `WOULD_FIRE`, `BACKSTOP`, `STILL_PRESENT`,
    `BOOTING`, `NO_POWER`, `TOO_LONG`, `BAD_JSON`, `NO_CMD`, `REPLY_OVERFLOW` (section 4).
13. States include `MODE` (drum moving), `CAL`, `SAFING`, `DECOCK`, `HOLD`; launch phases are new (section 7).
14. HOLD refuses everything that moves except `reel`; a stop is needed before mode, unwrap or jog. After a stop,
    unwrap is allowed even with tension on the line (no UNDER_LOAD check).
15. `reel`: duty must be above 0, max_n is capped at 150 N, reel needs drum_pos WRAPPED; `until tension` and `until
    stop` do not stop at SW_SEATED; a 240 s timeout ends any reel.
16. `jog`: ms default 300, capped at 1500, deadman refresh; refusals `WOULD_FIRE`, `NOT_AT_REST`, `BACKSTOP`.
17. Line limits: the firmware also caps its replies at 256 bytes and answers over-long input `TOO_LONG` with id null.
18. Firmware bug to fix before hardware: the telemetry `sw` letters (BL:844, use `SW_TEL`).

## What the Jetson mission must do differently

1. Shot order: LAUNCH_MODE, AIM, then COCK (new: `base.cock()`, wait `COCKED`), ARM, FIRE. Cock after the two FIRE
   confirmations (just before arm) so the spring is never left drawn while the operator decides; the cock takes
   about 3 s (give it its own timeout, 30 s).
2. If anything fails between COCKED and FIRED (arm refused, operator abort, timeout), run `base.decock()` and wait
   `RESET` before leaving LAUNCH, then aim again. A cocked base refuses mode changes (`LAUNCH_BUSY`) and aim
   (`NOT_AT_REST`).
3. Always aim after any mode change into LAUNCH, after a decock and after every shot: `aimed` is cleared by them.
   Check `base.status()["aimed"]` instead of comparing `pitch_deg` (ensure_launch). Aim only with phase REST.
4. Read `pitch_deg` from `AIMED` (no `deg` field).
5. After `FIRED`, the base stays in state FIRE until `RESET`; wait for IDLE (base_mode already waits) before REEL.
6. Contacts: dart seated is `tel["sw"].get("S")` (use `tel_seated(tel)` from comms.base), not `sw["seated"]`; the
   firmware must fix its letter table first (BL:844).
7. Landing detection cannot use `line_m` (null after FIRED). Use the dart and the video; after landing,
   `base.set("line_m", estimate)` (for example the ballistic line_out_m) so the line estimate, the seat slow down and
   the creep check work.
8. The anchored check measures line creep with `line_m`: set line_m first (step 7) or use `wiper_deg` deltas
   (capstan line = delta wiper deg / 360 x pi x 0.030 m).
9. With the real firmware a reel `until tension` does not end at the seat: a dart dragged home gives `LIMIT` (HOLD)
   with `sw.S` closed. After every LIMIT in CHECK and TOW, test `tel_seated` and treat it as "the dart came home".
   (MockBase ends any reel at the seat by default, `seat_ends_any_reel`; set it False to test the firmware way.)
10. `stop` before `mode`, `unwrap` or `jog` when the base may be in HOLD (`HOLDING` otherwise). Never unwrap with
    tension on the line: the firmware no longer checks.
11. Error handling: treat `FAULTED` like a fault (the old code was FAULT). After a fault the drum is NOT always in
    NEUTRAL: with wraps on or the launch train loaded it holds (MODE DRUM_HELD); word the operator message from that
    event. After `clear`, a COCKING phase resumes with `cock` or unloads with `decock`.
12. A `stop` (safe_stop) during COCK or FIRE leaves the phase UNKNOWN: the next cock answers `LOADED_UNKNOWN`
    (carriage drawn): run `decock`, aim, cock. Handle `PHASE_UNKNOWN` and `LOADED_UNKNOWN` that way.
13. `wait_event` now raises on `ABORTED` and on `ok: false` completion events (MODE DRUM_HELD, AIMED AIM_MISS):
    catch BaseError there.
14. Timeouts: `wrap` and `unwrap` take about 25 s (1470 drum deg at 60 deg/s): use a wrap timeout of 60 s, not
    mode_timeout_s 15. A reel ends by itself after 240 s (ABORTED).
15. `LIMIT` has no `line_m`: take it from telemetry. `status()` has no `tel`: read `base.tel`.
16. Preflight: also check `status()["phase"]` is REST (UNKNOWN: decock or cock recovery; COCKED: decock),
    `drum_pos` not UNKNOWN (cal drum), `wraps == 0`, no fault, `sw.S` closed.
