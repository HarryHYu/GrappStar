# C1 notes for the Jetson side

The contract is `firmware/base/PROTOCOL.md` on branch `w/fw-base` (fw `base-1.0`). Read it for the commands,
replies, error codes, telemetry, events, calibration keys, faults and the typical mission. This file only holds what
the Jetson side needs on top: how `comms/base.py` (BaseClient) and `comms/mock_base.py` (MockBase) implement it, the
differences from the draft C1 in `knowledge/WORKPLAN_2026-11-03.md`, and what the mission must do. Checked against
PROTOCOL.md at `93bfb1c` (2026-11-03).

## BaseClient (comms/base.py)

- One reader thread, reconnects by itself, replies matched by `id`. A reply with `id: null` (`BAD_JSON`, `TOO_LONG`)
  goes to the oldest command still waiting (the base answers in order).
- 256 bytes is a legal line (the newline not counted); `cmd()` raises ValueError only above 256.
- `HELLO` = the base restarted: every command still waiting fails with `BaseError(err="BASE_RESET")`, and a
  `wait_event` in progress raises `BaseError(err="BASE_RESET")`.
- `wait_event(names, ...)` raises BaseError for a `FAULT` (err = fault code), an `ABORTED` (err `ABORTED`, the event
  in `.reply`), and an awaited event with `ok: false` (err = its code: `DRUM_HELD`, `AIM_MISS`, `OUT_OF_RANGE`, or the
  fault code of a failed safing move). Switches: `fail_on_fault`, `fail_on_abort`, `fail_on_not_ok`.
- Helpers: `cock()`, `arm()` (returns the int token), `fire(token)`, `decock()` (`cal decock`), `cal_drum()`,
  `cal_cock_rest()`, `cal_aim_home()`, `cal_aim_forget()`, `cal_aim(deg)`, `tare()`, `reel(duty, max_n, until)`
  (duty must be above 0), `jog(motor, dir, ms=300)`.
- Constants `STATES`, `PHASES`, `EVENTS`, `FAULTS`, `ERRORS`, `SW_KEYS`; `tel_seated(tel)`, `tel_carriage_home(tel)`,
  `tel_contact(tel, "SW_DOG")` read the one-letter `sw` object.

## MockBase (comms/mock_base.py)

- Same commands, refusal codes and their check order, events and fields (all with `t`), telemetry keys and order,
  states (`IDLE MODE AIM COCK FIRE WRAP UNWRAP REEL HOLD JOG DECOCK SAFING FAULT`) and phases as PROTOCOL.md. Every
  line it sends is at most 256 bytes (`tx_max` records the longest).
- Drum: stop points LAUNCH 0, NEUTRAL 40, REEL 80, WRAPPED 338 at 10.8 deg/s (about 3.7 s per detent step, 24 s for
  the wraps); drum_pos BETWEEN and mode REEL in the wrap sectors; mode NONE between detents. The layout is the
  `DRUM_DEG` / `DRUM_ORDER` table and commands are a dispatch table (`_commands()`), so the WIP drum order
  (LAUNCH / NEUTRAL / WRAP / REEL) and new commands (tidy, retrieve) are a table change plus handlers.
- `HELLO` (with `pwm`) at start, `BOOT` 60 ms later. Telemetry 10 Hz, 20 Hz while moving, in real time; the mechanics
  run on `time_scale`.
- Test hooks (unchanged names): `fail_next[cmd] = code` (mode, aim, cock, fire, wrap, unwrap, reel, safing),
  `ignore`, `silent`, `inject_fault(code, present=False)` (present makes `clear` answer `STILL_PRESENT`),
  `fault_present`, `time_scale`, `received`, `world` (World: dart seated / flying / free / anchored, `slack_m`,
  `flight`, `on_fire`), `connected_pair`, `serve_pty`. New: `fresh=True` (new build: phase UNKNOWN, aim unknown),
  `code_bad` (invalid drum code), `dog_tip` (SW_DOG closed), `seat_ends_any_reel`.
- A fault during `fire` before the release leaves the phase COCKED (or REST if it had not started drawing), so
  `clear` then `fire` or `cal decock` recovers, as PROTOCOL.md section 6 says.
- Simplified: no dog wiggle (the 3 deg unload is applied to `wiper_deg`), the aim table is the firmware placeholder
  (pitch 15..60 over a 360 deg cycle, home at phase 0, `aim_home_deg` is stored but not used), the line speed is a
  placeholder (0.35 m/s at full duty), the tension model is the World model (anchored: slack then a steep rise).
- Deliberate difference: `seat_ends_any_reel` (default True, kept for the mission tests) ends every reel at the seat
  with `SEATED`. PROTOCOL.md stops at SW_SEATED only for `until: seated`; with `tension` or `stop` the real base winds
  on to `max_n` (LIMIT). Set it False to test the real behaviour.

## Differences from the C1 draft

1. `cock` is a new, optional command; `arm` accepts phase REST (carriage home) or COCKED, and `fire` from REST runs
   the whole cycle. `arm` needs aimed (within aim_tol), the dart seated, no wraps. Aim must happen at REST, so aim
   before cock.
2. New events `HELLO`, `BOOT`, `RESET` (after every shot and decock), `ABORTED`; every event has `t`; `MODE`, `AIMED`,
   `COCKED`, `WRAPPED`, `UNWRAPPED`, `RESET` carry `ok`. `AIMED` has `pitch_deg`, `target_deg` (no `deg`); `LIMIT` has
   `tension_n`, `max_n`, `why`, `holding` (no `line_m`).
3. Telemetry `sw` holds closed contacts only, one-letter keys `a b c D S H L W` (seated is `S`, not `seated`).
   `mode` is derived from the drum (`NONE` between detents). `wraps` is an integer or null; `pitch_deg`, `cock`,
   `line_m`, `drum_deg` can be null; `line_m` is null after a shot until `set line_m` or `SEATED`.
4. `status` returns flat fields (`phase`, `aimed`, `aim_known`, ...), no telemetry object. The token is an integer.
5. Faults and `estop` go to NEUTRAL only when safe; otherwise `MODE {ok:false, err:DRUM_HELD, why}` (wraps on, launch
   train not at rest, drum fault or invalid code).
6. Error codes: `FAULTED` (not FAULT), `WRAPS_ON` (not WRAPPED), `HOLDING` (not UNDER_LOAD), `OUT_OF_RANGE` (not
   RANGE), `BAD_ARG` (not BAD_ARGS), plus `LAUNCH_BUSY`, `PHASE_UNKNOWN`, `NOT_AT_REST`, `NOT_COCKED`, `NOT_AIMED`,
   `LOADED_UNKNOWN`, `POSITION_UNKNOWN`, `WOULD_FIRE`, `BACKSTOP`, `STILL_PRESENT`, `BOOTING`, `NO_POWER`, `TOO_LONG`,
   `BAD_JSON`, `NO_CMD`.
7. HOLD refuses every action except `reel` (send `stop` first). After a stop, unwrap is not refused under tension.
8. `reel` defaults duty 0.86 (floor 0.27), max_n = limit_n (20), capped at 150; it needs drum_pos WRAPPED. `wrap` and
   `unwrap` take about 24 s; mode steps about 4 s.
9. New settings `host_timeout_s` (host watchdog, `ABORTED {what: HOST}`), `vbat_wired`, `require_seated`; `cal
   aim_forget`; `cal drum` only reports (the drum code is absolute).

## Seen on the branch, not implemented

- Commit `70a01f7` (code read first): `telBuild` sent the contact letters from a stale table (seated as `W`, carriage
  home as `S`). Fixed at `93bfb1c` (uses `SW_TEL`).
- WIP commit `93bfb1c` (not in PROTOCOL.md yet): drum order LAUNCH / NEUTRAL / WRAP / REEL, `SW_TEL` = `a b c DL DR S
  H L W I` (two dog contacts and an extra one, not `D`), `reel until: slip`, a `DART_STUCK` fault (no SW_SEATED within
  20 s of final approach). No `tidy` or `retrieve` command in the code yet.

## What the mission must do

1. Shot: LAUNCH_MODE, AIM, then (optional) `cock()` and wait `COCKED`, ARM, FIRE. Cock after the two FIRE
   confirmations so the spring is never left drawn while the operator decides (or skip cock: fire from REST works,
   about 2 s slower). After `FIRED` wait for `RESET` (state FIRE until then) before any mode change.
2. If anything fails between COCKED and FIRED, `decock()` and wait `RESET`, then aim again: a cocked base refuses
   mode changes (`LAUNCH_BUSY`) and aim (`NOT_AT_REST`).
3. Aim after every mode change into LAUNCH, decock and shot (`aimed` is cleared). Check `status()["aimed"]`, not the
   telemetry pitch.
4. Read `pitch_deg` from `AIMED`.
5. Seated is `tel_seated(tel)` (`sw.S`), not `sw["seated"]`.
6. Landing cannot use `line_m` (null after the shot): use the dart and the video, then `set line_m` to the estimate
   (for example the ballistic line_out_m) so the line estimate, the seat slow down and the creep check work.
7. The anchored check measures creep with `line_m`: set it first (step 6).
8. With the real base, `reel until tension` does not end at the seat: after every LIMIT in CHECK and TOW, test
   `tel_seated` and treat it as "the dart came home".
9. `stop` before `mode`, `unwrap` or `jog` when in HOLD. Never unwrap with tension on the line.
10. Treat `FAULTED` like a fault; after a fault the drum is not always in NEUTRAL (MODE DRUM_HELD says why). A stop
    during cock or fire leaves the phase UNKNOWN: `cock` then answers `LOADED_UNKNOWN` (decock, aim, cock) or creeps
    if the carriage is home.
11. `wait_event` raises on `ABORTED`, on `ok:false` and on `HELLO` (`BASE_RESET`): handle BaseError there.
12. Timeouts: wrap and unwrap about 24 s (use 60 s, not mode_timeout_s 15); mode steps about 4 s each (LAUNCH to REEL
    7.4 s). For long reels either keep `host_timeout_s` at 0 or send `ping` periodically.
13. `LIMIT` has no `line_m` (read telemetry); `status()` has no `tel` (read `base.tel`).
14. Preflight: `status()["phase"]` REST (UNKNOWN: decock or cock recovery; COCKED: decock), drum_pos not UNKNOWN,
    `wraps == 0`, no fault, `sw.S` closed.
15. The reply comes BEFORE the events a command causes (MockBase does the same). After a `stop` that aborts a running
    action, the `ABORTED` event arrives just after `stop()` returns: wait for it (or take the next `event_mark()` only
    after it), else the next `wait_event` raises ABORTED for the old action.
