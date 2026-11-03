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
    a sequence must also raise the total (rise rule, round 10 version: half the way from the worm's own lowest reading
    to its line, at least 25 mA), or be over the line in 3 of the first 4 readings of a step (worm already at its stop).
11. **Brownout or watchdog reset boots in `FAULT`** with motors off, until `/stop`, `/auto`, `/release` or `/arm`.
12. **Kick travel bookkeeping**: the kick's travel is counted from power-on (power on with the legs stowed). It turns
    "stall near stowed" into `KICK_JAM`, a stall after a whole swing in air into `NO_GROUND` (new, round 10), and a
    release that stalls before 60 % of the way back into `KICK_JAM`.
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
- Coordinator R1-B review, the C2 cross-check with the jetson client, the clean-clone check (all README steps pass
  from a fresh clone; doc gaps fixed), the coordinator's `test_06_hold_and_stop` failure (firmware stall lock, fixed).

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
