# fw-base report (firmware/base)

Worker `fw-base`, 3 Nov 2026. Branch `w/fw-base`. DRAFT: updated through the day, final at 18:30.

## Done

- `grappstar_base/`: Arduino sketch for the ESP32-S3 DevKitC-1-N8. All logic in plain C++ headers, tested on a PC:
  - `pins.h` (every pin), `config.h` (defaults; spec numbers via `params_gen.h`), `drives.h` (BTS7960: 20 kHz PWM,
    300 ms soft start, 200 ms reverse gap, IS current scaled by duty, 2-sample over-current trip, stall detection;
    DRV8833: slow decay, brake, sleep when idle), `sensors.h` (debounced printed contacts with chatter detection,
    edge position capture; AS5600 multi-turn unwrap, glitch rejection, magnet health, rate), `persist.h` (NVS record
    with CRC and the "moving" flags, bench settings record), `json_lite.h` (flat JSON parser and a bounded builder),
    `base_logic.h` (state machine, fault manager, drum mover, aim, cock, fire, decock, wrap, unwrap, reel and tow,
    jog, cal, set/get, telemetry, events, power-loss recovery), `grappstar_base.ino` (USB CDC JSON lines, 5 ms tick
    task on core 1 with a task watchdog, NVS writes in loop() with a start gate, HX711 auto detect).
- `PROTOCOL.md`: final contract C1.
- `README.md`, `WIRING.md`, `CALIBRATION.md`, `BENCH_TEST.md`: learner docs.
- `tools/gen_params.py`: `base/spec/params.py` to `grappstar_base/params_gen.h` (P_* macros, derived aim table).
- `test/`: `sim.h` (wiper DC motor model with self-locking worm, AS5600, IS output, drum with Gray code bumps and edge
  jitter, dog tip-to-tip, launch train with spring load and release tooth, aim crank, capstan and line, HX711,
  battery, fault injection, dry-fire detector, delayed flash acks), `test_base.cpp` (mission and fault tests),
  `test_gen_params.py`, `run_tests.sh`.

## Decisions made (Harry was away)

1. Mechanism followed: BASE_SPEC v0.1 plus the cad-drum and cad-launch NOTES (drum Gray code sectors and stop plan,
   SW_DOG, 553.8 wiper deg per shot, aim table of 3600 wiper deg, aim before cock).
2. Fault response: wiper off at once; drum to NEUTRAL, EXCEPT it holds (MODE `err:"DRUM_HELD"` with the reason) when
   the drum code is invalid or a drum fault caused it, when wraps are on (sprag #2 holds the line), or in LAUNCH with
   the spring not at rest (no sear: a shift under load could dry fire). Recovery: `clear`, then `cal decock` or `fire`.
3. `fire` works from COCKED and from REST (whole cycle in one run, as the launch notes describe). `cock` is optional.
4. A motor only starts after the "moving" record is in flash (an NVS write stalls both cores for tens of ms).
5. Telemetry `sw` lists closed contacts only with one-letter keys, so every line fits the 256-byte limit.
6. Additive protocol items: events HELLO, BOOT, RESET, ABORTED; `cal` sub-commands; settings keys; optional host
   watchdog (`host_timeout_s`, off by default).

## Verified

(test output pasted at the end of the day)

## Open items

## What Harry must measure

## Requests to other workers
