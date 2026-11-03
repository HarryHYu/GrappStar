# fw-dart REPORT (3 Nov 2026)

Worker `fw-dart`: KEDGE dart firmware v4 and the dart electronics docs. Everything is in `firmware/dart/`.
Firmware `kedge-dart-v4`. Nothing has run on real hardware yet: all verification is on the PC (logic tests, fuzz test,
the real sketch in a host simulator, the web page in headless Chromium, the Jetson's own client) plus compiles.

## Done

| Task | Where |
|---|---|
| 1. Firmware copied from `kedge/fw/` and moved on to v4 | `kedge_dart/kedge_dart.ino`, `kedge_dart/kedge_logic.h` |
| 2. Pin map with DRV8833 #3 freed: DRV#1 A deploy, DRV#1 B kick, DRV#2 A both drills (parallel, right drill leads swapped), DRV#2 B foot | `kedge_logic.h` `PINS[]`, compile-time checked against the XIAO Dn macros |
| 3. Contract C2: `/release`, `phase`, `/flightlog` (50 Hz, 12 s ring, frozen 2 s before launch to 3 s after landing) | `API.md` (final C2) |
| 4. Safety review (10 independent review rounds, a coordinator review, a failure-mode walk; confirmed findings fixed) | below, `FAILURE_MODES.md` |
| 5. PC tests: logic (with a motor plant model), a property fuzz test, the real sketch over HTTP, the web page in Chromium, the Jetson client end to end | `test/`, `sim/` |
| 6. Docs: API, README (Arduino IDE), WIRING (v29 conversion and fresh build, learner steps each starting "Battery unplugged"), wiring diagram SVG/PNG, cut list, change list | `API.md`, `README.md`, `WIRING.md`, `wiring/` |
| 7. Bench test plan | `BENCH_TEST.md` |

## Decisions (made without Harry, recorded here)

1. **`/auto` is refused during FLIGHT** (409 `in flight`), and so are manual keys and `/release`. Motors in the air would
   swing the legs out and could foul the tether; the landing detector starts the sequence itself. FLIGHT always ends:
   landing, the 15 s flight timeout, or `/stop`.
2. **`/auto` while armed (not flying) disarms and starts.** Refusing it deadlocked the Jetson when a launch was missed.
   Risk noted by review round 10: a retried `/auto` before a shot swings the kick on the rail. The Jetson only sends
   `/auto` after it has seen a landing.
3. **Release order is the exact reverse of `/auto`**: drills out alone 3 s, then pods up with the drills still
   unscrewing, foot out alone 1.5 s, kick back with the foot still unscrewing.
4. **Auto-anchor on landing starts only after an impact landing** (impact over 3.5 g at least 400 ms after launch, after
   free fall or 400 ms with the push over, then still 1 s; coordinator decision D28). Soft `still` and `timeout`
   landings stop at `LANDED` and the Jetson sends `/auto`.
5. **Launch is only looked for while armed**; `/stop` disarms, `/stop?disarm=0` keeps the arm. A disarming stop locks
   every key pressed in the last 600 ms until released; a keep-arm stop does not lock.
6. **Refusals are HTTP 409** `refused: <reason>`; `/auto` and `/release` are idempotent while running.
7. **INA219 missing: auto and release refused**, manual keys on timers only (2 s hold cap). **BNO085 missing: arming refused.**
8. **INA219 read directly over I2C** so a failed read is seen; re-calibrated at 10 Hz if it browned out. Only the
   Adafruit BNO08x library is needed.
9. **Sequences run at full duty**; thresholds scale with rail voltage (bench values at 8.0 V) and with duty squared for
   manual keys (slow decay drive).
10. **Two motors at once**: the self-locking worm pauses 150 ms every 550 ms and the auger is read alone. A worm stall in
    a sequence must also raise the total (rise rule: half the way from the worm's own lowest reading to its line, at
    least 25 mA, measured against the lowest of the last 4 readings), or be over the line in 3 of the first 4 readings
    of a step (worm already at its stop).
11. **Brownout or watchdog reset boots in `FAULT`** with motors off, until `/stop`, `/auto`, `/release` or `/arm`.
12. **Kick travel bookkeeping**: the kick's travel is counted from power-on (power on with the legs stowed), scaled by
    rail voltage. It turns "stall near stowed" into `KICK_JAM`, a stall with no contact at 90 % of the swing in air
    (not planted since the last release) into `NO_GROUND` (rounds 10 and 11, also on retries), and a release that
    stalls before 60 % of the way back into `KICK_JAM`. Assumes the housing lands on flat ground before 80 % of the
    swing in air (BENCH_TEST step 5 checks it; the firmware itself measures and logs the swing).
13. **Hold contract with the Jetson** (agreed with jetson, in `API.md`): repeat `/key` every 150 ms with `ms=350`,
    `stalled` is the normal end, always finish with `/stop?disarm=0`. The stall lock is per direction.
14. **`/arm` is refused while `ANCHORED`** (legs out: release first).
15. **Geometry unchanged**: v30 stays the default build; the DRV#3 pocket in the sled stays empty.

## Verified

TEST_OUTPUT_PLACEHOLDER

## Review rounds and what they found (all confirmed findings fixed unless listed as open)

1. Safety, adversarial tests, Jetson compatibility: unbounded held key, multi-motor stall miss, helper motor stall
   unchecked, bench bump starting auto, PWM duty vs thresholds, NaN in JSON, BNO085 reports dropped, flight log
   blocking STOP, page reload dropping keys.
2. Code and learner docs: slow key repeats bypassing cap and stall, voltage-dependent thresholds, NaN reads zeroing
   detectors, tumbling dart never passing the free-fall gate, meter checks through motor windings, USB and battery
   order, antenna before power, buck set off the 3S lead.
3. Host simulator of the real sketch: IMU timestamp ahead of the tick (would have ended a real flight as a timeout),
   held key restarting after a fault.
4. Noise, jitter, battery 6.2 to 8.4 V, soils: pair check fooled by current coupling, noise at low PWM, dropped armed
   dart starting auto, keys in FAULT, motor running into the flight.
5. Web page in Chromium: STOP not clearing held keys, pagehide STOP cancelled by the browser.
6. to 9. Hardware integration (datasheets), time wrap (signed compares everywhere), I2C bus recovery, web-loop
   watchdog, launch rules rejecting real shots (replaced by the free-fall rejection), kick jam misread (travel
   bookkeeping), lock bugs (`/auto` deadlock, keep-arm stop locking the page, wrong direction locked), fault while
   armed now disarms, stalled augers not re-driven.
10. Fresh review: a kick swinging through air ended in a false `ANCHORED` (now `NO_GROUND`); the no-rise rule could end
    planting early (now first 4 readings only); a quick key reversal latched "both directions held" (now reverses);
    a stalled `/flightlog` client could reboot the dart (watchdog 10 s to 30 s); arming while anchored; doc mismatches.
11. Fresh review of round 10: a retry after `NO_GROUND` (or a part swing retried) still anchored in air; flat ground
    could be called `NO_GROUND` (travel now voltage-scaled, geometry assumption documented and bench-checked); a falling
    auger current could hide a worm stall (rise now against the last 4 readings); alternating keys drove a jammed stop
    without limit (hold cap per motor). The fuzz test got a no-ground case and invariant 11.
12. Two fresh reviewers (logic; sketch, HTTP, page, docs): `NO_GROUND` hung on a stopwatch number with a 5 % margin
    (now the firmware measures and logs the swing, line at 90 %, ground rule 80 %); page STOP could fail silently (now
    retried, "STOP NOT CONFIRMED" shown); Cmd/Ctrl/Alt key handling; a swapped INA219 read negative and was trusted
    (now a sensor fault); IMU silent not in `/status` event; failed OTA restart gap; bench steps reordered with
    "battery unplugged" before every repositioning, bench leads removed before the cells go in.
- Coordinator R1-B review, the C2 cross-check with the jetson client, the clean-clone check (all README steps pass
  from a fresh clone; doc gaps fixed), the coordinator's `test_06_hold_and_stop` failure (firmware stall lock, fixed).

## Coordinator reviews: triage

| Review item | Status |
|---|---|
| r2_B2 #1 timeout landing freezes the log too late | fixed earlier (R1-B freeze cap at tLaunch - 2 s + 12 s, on launch, landing and stop; test "timeout landing") |
| r2_B2 #2, #3, #8 (dart.py flightlog 409, CLI release) | jetson's files; dart.py now retries `/flightlog` on 409 |
| r2_B2 #4 API says SEQ_TIMEOUT on a failed read at a step end | fixed earlier (API.md fault table) |
| r2_B2 #5 releasing a refused key aborts a running sequence | fixed: key-release stop only stops manual motors (`keyRelease`), tests in test_logic and test_page |
| r2_B2 #6 reversing gets round the hold cap | fixed in b9cf242: hold cap per motor |
| r2_B2 #7 fuzz checks only flight log syntax | fixed: invariant 8 also requires a frozen flight log to hold the launch |
| r3_N #5 WIRING A2 wire [15] vs [16] | fixed; every wire number re-checked against both lists, missing v29 numbers added, A7 checks DRV#2 GND and VM |
| docs worker learner round (7 items) | fixed: hull closed for assembled steps, struct and array order named, test-model warning, one capacitor location, B10 open circuit, full paths |

## Requests to other workers

- **jetson**: retry `/flightlog` on 409; `release_dart` gets `in flight` in FLIGHT; read `warnings`; check `imu_ok`
  before arming; new fault `NO_GROUND` (do not retry `/auto` blindly); `/arm` refused while `ANCHORED`. Hold contract
  agreed (no dart.py change needed).
- **coordinator**: `scripts/check_all.sh` runs `firmware/dart/test/run_tests.sh` (about 2 min with the simulator).
- **docs**: run sheet v29 step 7.3 and 7.6 are superseded by `WIRING.md` and `BENCH_TEST.md`.

## Open items / what Harry must measure

- Stall and free currents of all four channels at 2S (BENCH_TEST steps 3 to 6), then `Tuning` (step 10), then the PC tests.
- Kick swing time (step 5) into `kickSwingMs` (4300 now): `KICK_JAM` and `NO_GROUND` depend on it.
- Foot current drop while the kick pauses in soil (step 9.6) and drill current change while the pods pause: over about
  80 mA (kick) or 100 mA (pods) fakes an end stop from the start of the step (weak anchor, no warning).
- BNO085 axes in the hull (step 7); `impactG` (3.5 g) against the `peak_g` of real shots; SLP pad resistance (WIRING B6);
  the BMS cutoff against `lowRailV` 5.6 V.
- Known limits (documented in `API.md` / `FAILURE_MODES.md`): one dead drill, or a drill or the foot wired backwards,
  cannot be detected; a kick jam in the last 40 % of the return is not caught; below about speed 70 only the 8 s cap
  stops a manual stall; in the full fuzz run (8000 episodes) 6 chaos cases of the pods at a stop or jammed, with the
  drills' current rising during each pause, run just over 3.0 s, bounded by the 6 s pods limit; a power-on with the legs out gives a false `KICK_JAM` on `/auto` (release first).
