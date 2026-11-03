# C1 notes for the Jetson side

The contract is `firmware/base/PROTOCOL.md` (fw `base-1.3.1`, params 1.3.8; section 10 lists every change the
Jetson must follow) and the one C1 sequence of `base/spec/BASE_SPEC.md` section 10.1 (D35, D45). Read those for the
commands, replies, error codes, telemetry, events, calibration keys and faults. This file only holds what the Jetson
side needs on top: how `comms/base.py` (BaseClient) and `comms/mock_base.py` (MockBase) implement it, and what the
mission must do. Rewritten for base-1.3.1 (the earlier base-1.0 notes are in the git history).

## BaseClient (comms/base.py)

- One reader thread, reconnects by itself, replies matched by `id`. A reply with `id: null` (`BAD_JSON`, `TOO_LONG`)
  goes to the oldest command still waiting (the base answers in order).
- 256 bytes is a legal line (the newline not counted); `cmd()` raises ValueError only above 256.
- `HELLO` = the base restarted: every command still waiting fails with `BaseError(err="BASE_RESET")`, and a
  `wait_event` in progress raises `BaseError(err="BASE_RESET")`.
- `wait_event(names, ...)` raises BaseError for a `FAULT` (err = fault code), an `ABORTED` (err `ABORTED`, the event
  in `.reply`), and an awaited event with `ok: false` (err = its code: `DRUM_HELD`, `AIM_MISS`, `OUT_OF_RANGE`, or
  `NOT_OK` with the event in `.reply`, for example `RESET` `ok:false` after a decock that moved nothing). Switches:
  `fail_on_fault`, `fail_on_abort`, `fail_on_not_ok`.
- Helpers: `cock()`, `arm()` (returns the int token), `fire(token)`, `decock()`, `cal_drum()`, `cal_cock_rest()`,
  `cal_aim_home()`, `cal_aim_forget()`, `cal_aim(deg)`, `tare()`, `reel(duty, max_n, until)` (max_n at most 55 N at
  the dart, D46), `jog(motor, dir, ms=300)`, `cal(what)`, `set(key, val)`, `get(key)`.

## MockBase (comms/mock_base.py)

- Same commands, refusal codes and their check order, events and fields (all with `t`), telemetry keys and order,
  states and phases as `base_logic.h`; tests/test_comms_base.py runs the same probes against the real firmware logic
  (jetson/tools/basesim) where they must agree. Every line it sends is at most 256 bytes (`tx_max`).
- Drum (params.DRUM, code edges as built 6, 34, 50, 62, 302, 342): parks LAUNCH 0, NEUTRAL 41, WRAPPED 314 (in
  N_TO_R, collar NEUTRAL below 318), REEL 347, at about 8.1 deg/s (LAUNCH to NEUTRAL about 5.5 s; wrap about 37 s from
  NEUTRAL, 42 s from LAUNCH; unwrap about 41 s from REEL). `wrap` ends at the park with `WRAPPED` only; `mode REEL`
  flies in from the park; `unwrap` ends with `UNWRAPPED` then `MODE` (also at once at NEUTRAL).
- Contacts as cad-sensors and cad-launch build them: `SW_RAIL_LOW` closed within 97.6 aim deg of the low dead point
  (`aim_home_deg` 3502), `SW_GAP` within 22.9 wiper deg of rest (`gap_close_deg` 531; a value well below the CAD edge
  faults the reset like the firmware), `SW_SEATED` only with the carriage home (a dart seated when the cock started
  counts for `arm` and `fire` while `COCKED`).
- `HELLO` (with `pwm`) at start, `BOOT` 60 ms later. Telemetry 10 Hz, 20 Hz while moving, in real time; the
  mechanics run on `time_scale`.
- Test hooks: `fail_next[cmd] = code`, `ignore`, `silent`, `inject_fault(code, present=False)`, `fault_present`,
  `time_scale`, `received`, `world` (World: dart seated / flying / free / anchored, `slack_m`, `flight`, `on_fire`),
  `fresh=True` (new build: phase UNKNOWN, aim unknown), `code_bad`, `seat_ends_any_reel` (default False, as the
  firmware), `connected_pair`, `serve_pty`.
- Simplified: no sprag drag (so no gap run), no dog wiggle, the tension is the World model (anchored: slack then a
  steep rise).

## What the mission must do (BASE_SPEC 10.1)

1. Shot: LAUNCH_MODE, AIM, (optional) COCK, ARM, FIRE. Aim first: aim needs the carriage home. After `FIRED` wait for
   `RESET` before anything else. After every shift into LAUNCH, decock and shot the aim is redone.
2. After the landing: `set line_m`, then `aim` 6 deg when the pitch is above 15 deg (D45, step 7b), then `wrap`.
3. A stopped or faulted launch cycle leaves the phase `UNKNOWN`: `decock`, wait `RESET`, aim again (after the shot:
   the low pitch). `RESET` `ok:false` "SW_GAP open ..." means look at the carriage, then `cal cock_rest` with force.
4. `SW_SEATED` (`sw.S`) reads the dart only with the carriage home (`sw.H`): open while cocked, seated or not
   (mission.dart_seated returns None then). The firmware checks it at `cock` and keeps that reading while `COCKED`.
5. With the wraps on or not known off (drum_pos WRAPPED or REEL, or code bit `c` closed, or `wraps` null) only
   `unwrap` brings the drum back to NEUTRAL: `mode`, a drum jog and `retrieve` are refused `WRAPS_ON` there.
6. Never seat the dart on the capstan: reel in to about 1.5 m, `unwrap`, then `retrieve` in the NEUTRAL sector. The
   `retrieve` time limit `max_s` is the firmware default sent explicitly: the longer of 20 s and 10 s + 1.5 x line_m /
   v, v = 0.29 x duty - 0.03 m/s, 30 s with line_m unknown, at most 600 s.
7. Timeouts: `wrap` and `unwrap` at least 90 s; mode steps about 5 s each (allow 60 s). Keep the host watchdog fed
   (`ping` every 0.4 s with `host_timeout_s` 2.5 during a mission).
8. Treat `FAULTED` like a fault; with the wraps on a fault holds the drum (`MODE` `DRUM_HELD` says why).
9. The reply comes BEFORE the events a command causes. After a `stop` that aborts a running action, wait for its
   `ABORTED` event before the next `event_mark()`.
10. HOLD refuses every action except `reel`: `stop` first (and wait for `ABORTED`) before a new tow round (F22).
11. The dart (contract C2) serves one request at a time: never poll `/status` faster than 5 Hz in total. Waits read
    through `Dart.status_cached()` so they share the App's 5 Hz poller.
