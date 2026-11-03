# fw-dart REPORT (3 Nov 2026)

Worker `fw-dart`: KEDGE dart firmware v4 and the dart electronics docs. Everything is in `firmware/dart/`.
Status: DRAFT, updated through the day; the final version is pushed at 18:30 with fresh test output.

## Done

| Task | Where |
|---|---|
| 1. Firmware copied from `kedge/fw/` and moved on to v4 | `kedge_dart/kedge_dart.ino`, `kedge_dart/kedge_logic.h` |
| 2. Pin map with DRV8833 #3 freed: DRV#1 A deploy, DRV#1 B kick, DRV#2 A both drills (parallel, right drill swapped), DRV#2 B foot | `kedge_logic.h` `PINS[]`, compile-time checked against the XIAO Dn macros |
| 3. Contract C2: `/release`, `phase`, `/flightlog` (50 Hz, 12 s ring, frozen 2 s before launch to 3 s after landing) | `API.md` (final C2) |
| 4. Safety review of the whole firmware (6 independent review rounds, all confirmed findings fixed) | below |
| 5. PC tests: logic, simulator of the real sketch over HTTP, web page in headless Chromium | `test/`, `sim/` |
| 6. Docs: API, README (Arduino IDE), WIRING (v29 conversion and fresh build, learner steps), wiring diagram SVG/PNG, cut list, change list | `API.md`, `README.md`, `WIRING.md`, `wiring/` |
| 7. Bench test plan | `BENCH_TEST.md` |

## Decisions (made without Harry, recorded here)

1. **`/auto` is refused during FLIGHT** (409 `in flight`), and so are manual keys and `/release`. Motors in the air would
   swing the legs out and could foul the tether; the landing detector starts the sequence itself. FLIGHT always ends:
   landing, the 15 s flight timeout, or `/stop`.
2. **Release order is the exact reverse of `/auto`**: drills out alone 3 s, pods up with the drills still unscrewing,
   foot out alone 1.5 s, kick back with the foot still unscrewing. C2's text listed "drills, foot, kick, deploy".
3. **Auto-anchor on landing starts only after an impact landing** (impact over 6 g at least 400 ms after launch, after
   free fall or 400 ms with the push over, then still 1 s). Soft "still" and "timeout" landings stop at `LANDED` and the
   Jetson sends `/auto` (the jetson worker's mission arms log-only and sends `/auto` itself).
4. **Launch is only looked for while armed** (`/arm`); `/stop` disarms (`/stop?disarm=0` keeps the arm).
5. **Refusals are HTTP 409** with `refused: <reason>`; `/auto` and `/release` are idempotent while running.
6. **INA219 missing: auto and release refused** (no stall detection), manual keys on timers only (2 s hold cap).
   **BNO085 missing: arming refused.**
7. **INA219 read directly over I2C** (not the Adafruit library) so a failed read is seen; re-calibrated at 10 Hz if it
   browned out. Only the Adafruit BNO08x library is needed.
8. **Sequences always run at full duty**; thresholds scale with rail voltage (bench values at 8.0 V) and with duty for
   manual keys.
9. **Two motors at once**: the self-locking worm pauses 150 ms every 550 ms and the auger is read alone (pausing the
   worm, not the auger, so a stopped screw never fakes a stall); a worm stall in a sequence must also raise the total.
10. **Brownout or watchdog reset boots in `FAULT`** with motors off, until `/stop`, `/auto`, `/release` or `/arm`. Run keys are refused while a fault
    is latched.
11. **Geometry unchanged**: v30 stays the default build; the DRV#3 pocket in the sled stays empty.

## Verified

PC tests (`firmware/dart/test/run_tests.sh`, which also runs the simulator and the browser test): output pasted in the
final version below. Compile: `arduino-cli compile -b esp32:esp32:XIAO_ESP32S3` exit 0 with zero warnings from our
files on ESP32 core 3.3.2 (30 % flash, 20 % RAM) and 2.0.17.

## Review rounds and what they found

1. Safety (embedded), adversarial tests, Jetson compatibility: unbounded held key, multi-motor stall miss, helper motor
   stall unchecked, bench bump starting auto, PWM duty vs thresholds, NaN in JSON, BNO085 reports dropped by the library
   call, flight log blocking STOP, page reload dropping keys. All fixed.
2. Code review, docs review (learner view), docs worker: slow key repeats bypassing cap and stall, voltage-dependent
   thresholds, NaN reads zeroing detectors, tumbling dart never passing the free-fall gate, meter checks that would fail
   through motor windings, USB and battery order, antenna before power, buck set off the 3S lead. All fixed.
3. Host simulator of the real sketch: IMU timestamp ahead of the tick (flapping `imu_ok`, would have ended a real flight
   as a timeout), held key restarting after a fault. Fixed.
4. Adversarial tests with noise, jitter, battery 6.2 to 8.4 V, soils: pair check fooled by current coupling, slow
   repeats, noise at low PWM, dropped armed dart starting auto, keys in FAULT, motor running into the flight. Fixed.
5. Web page in headless Chromium: STOP button not clearing held keys, pagehide STOP cancelled by the browser. Fixed.
6. Hardware integration (datasheets): in progress.

## Requests to other workers

- **jetson**: (already adopted in a561376) treat 409 as an answer; use `/release`, `/arm`; accept `seq` `released`;
  fail on `FAULT` or `aborted`. Use `firmware/dart/sim/` (the real sketch on Linux) for integration tests.
- **coordinator**: C2 release order recorded as the reverse of `/auto` (decision 2). `scripts/check_all.sh` already runs
  `firmware/dart/test/run_tests.sh`, which now includes the simulator and the browser test (about 1.5 min).
- **docs**: run sheet v29 step 7.3 (DRV#2 bridge) and 7.6 (motor direction table) are superseded by `WIRING.md` and
  `BENCH_TEST.md`.

## Open items / what Harry must measure

- Stall and free currents of all four motor channels at 2S (BENCH_TEST steps 3 to 6), then `Tuning` (step 10).
- Drill current change while the pods pause in soil (watch for an early "pods down" in BENCH_TEST step 9).
- BNO085 axes in the hull (BENCH_TEST step 7) for the Jetson flight analysis.
- Kick swing time (BENCH_TEST step 5) against `kickMaxMs` (15 s) and the 8 s manual hold cap.
- Which DRV8833 boards he has (Pololu or generic): the SLP pad check in WIRING step B6 covers both.
- Nothing has run on real hardware yet.
