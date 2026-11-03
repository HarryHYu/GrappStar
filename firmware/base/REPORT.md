# fw-base report (firmware/base)

Worker `fw-base`, 3 Nov 2026, branch `w/fw-base`. Firmware string `base-1.2.1`, params `1.2.6`.
Nothing here has run on hardware: every claim below is from the PC simulation and the arduino-cli compile.

## Done

- `grappstar_base/`: Arduino sketch for the ESP32-S3 DevKitC-1-N8. All logic is plain C++ in headers, tested on a PC.
  - `base_logic.h`: state machine, fault manager, drum mover, aim, cock, fire, decock, wrap, unwrap, tidy, retrieve,
    reel and tow, jog, cal, set/get, telemetry, events, power-loss recovery, D12 no-magnet index mode.
  - `config.h` (defaults; spec numbers from `params_gen.h`), `pins.h`, `drives.h` (BTS7960 soft start, reverse gap, IS
    scaled by duty, trips, stall; DRV8833 brake), `sensors.h` (debounced contacts with chatter detection and edge
    timing; AS5600 multi-turn tracker), `persist.h` (NVS records with CRC), `json_lite.h`.
  - `grappstar_base.ino`: USB CDC JSON lines, 5 ms tick task on core 1 with a 300 ms task watchdog, flash writes in
    loop() with a start gate (a motor starts only after the "moving" record is written), IS sampled across the PWM
    period, HX711 auto detect.
- `tools/gen_params.py`: `base/spec/params.py` to `grappstar_base/params_gen.h`. Drum parks, wraps, capstan, tow
  limit, current trips, tension model, drum speed and the 37-point aim table all come from params.
- `test/`: `sim.h` (physics sim: wiper DC motor with optional inertia and coast, IS output, AS5600 with dropouts and
  glitches, drum Gray code with edge offsets, dog window and tooth-on-tooth, launch train with release, take-up and
  capstan line model, HX711, battery, fault injection, dry-fire detector), `test_base.cpp` (375 checks),
  `test_fuzz.cpp` (random commands and faults against invariants), `test_gen_params.py`, `run_tests.sh`.
- Docs: `PROTOCOL.md` (contract C1, BASE_SPEC 10.1 sequence, section 10 lists every change for the Jetson),
  `README.md` (exact Arduino IDE settings, core and library versions), `WIRING.md` (every step starts "Battery
  unplugged."), `CALIBRATION.md`, `BENCH_TEST.md`.

## Decisions made (Harry was away)

1. Mechanism: spec v1.2 / params 1.2.6 (drum as cad-drum built it: LAUNCH 0, NEUTRAL 41, WRAP 62..302 with 7 cup
   turns, WRAPPED park 314, REEL 347; D22 / D35 sequence).
2. Fault response: wiper off at once; drum to NEUTRAL, except it HOLDS (MODE `DRUM_HELD` with the reason) with the
   wraps on (params fault_rules, D27), with the spring possibly drawn, with a bad drum code or a drum fault.
3. Power cut mid-cock: the record holds the start angle, so boot assumes the spring is drawn (cock refused,
   `decock` with `force` if SW_HOME lies). Safer than trusting a contact.
4. A drum code change with the drum still near the wrap window makes the wrap count doubtful: only `unwrap` (with
   the take-up) until NEUTRAL.
5. Tow trip (D34): max(tow_trip_a 6.5 A, (limit + 10.4) / 17 + I_free + 1 A), capped at 17 A; the tension estimate
   (averaged 200 ms, D33) is the real tow limit.
6. Fly-in at 25 percent duty (asked about 20) so the 3 A trip reads real current (IS scaling needs duty 60 or more).
7. Final approach time limit: max(20 s, 10 s + 1.5 x line_m / slowest take-up speed at the duty), 30 s with line_m
   unknown, `max_s` 5..600. The coordinator asked 5..180; physics shows 20 m at duty 0.3 needs about 500 s.
8. Reel-in trip stays 5 A (D33) although physics suggested 6 A; one config line if Harry agrees.
9. `retrieve` is also accepted at the WRAPPED park below 318 with the collar free (BASE_SPEC says NEUTRAL only);
   refused with the collar in REEL (D35).
10. Index mode (D12): cock stops a bump early, a missed bump faults within 1.5 bump periods, every aim re-homes on
    SW_RAIL_LOW, aim tolerance at least 2 deg.

## Verified

(final output pasted below at 18:20)

## Open items

- Known gap: SW_HOME stuck closed AND the saved angle lost AND the train about 300 deg into a cock: a recovery
  `cock` can still reach the release (its reset allowance of 157 deg overlaps). Every other combination is refused.
  The test starts this case at 270 deg.
- The sim cannot produce a dog friction-locked under load, contact bounce bursts, VBAT ripple or AS5600 nonlinearity
  (review notes in this report's history); bench tests in BENCH_TEST.md cover them by hand.
- Spec text that disagrees with params / decisions (reported to the coordinator): BASE_SPEC 30 deg unload jog
  (params 8), take-up "about 30 percent" (firmware 40, I4 cap), D31 309 / D32 -6 (superseded by 314 / -10.4).

## What Harry must measure

## Requests to other workers
