# jetson worker report (3 Nov 2026)

Everything that runs on the rover's Jetson Orin Nano Super, in `jetson/` (Python package `grappstar`, about 17,000
lines of Python with tests, plus the `basesim` C++ harness). Run as `python3 -m grappstar <command>`. Python 3.10,
numpy 1.26 (JetPack pins numpy<2), OpenCV, pyserial; nothing else is required (onnxruntime optional).

## Done

| Item | Where | State |
|---|---|---|
| C1 base client (reconnect, id-matched replies, events, telemetry, HELLO = restart, ABORTED and ok:false raise) | `grappstar/comms/base.py` | done, tested against the real firmware logic |
| Simulated base faithful to PROTOCOL.md (states, events, sw letters, HOLD, DRUM_HELD, host watchdog), pipe or pty | `grappstar/comms/mock_base.py` | done |
| **basesim**: the real `base_logic.h` + `test/sim.h` behind C1 on stdin/stdout or a pty, scenario side channel (`#sim anchor 3.5`) | `tools/basesim/` | done; built on demand by the tests |
| C2 dart client v4 (409 refusals, `/arm?auto=0`, clock sync from `t_ms`, flightlog busy retry, IMU axis remap, reboot-at-impact) | `grappstar/comms/dart.py` | done, tested against fw-dart's real firmware simulator |
| Simulated dart v4 | `grappstar/comms/mock_dart.py` | done |
| Video: V4L2 with explicit FOURCC (set before size), GStreamer pipelines (nvjpegdec), discovery by-id with role guess, analog quality (snow, dropout, tearing, freeze), deinterlace, recording with per-frame timestamps, replay, camera thread with reconnect and a byte-capped ring buffer | `grappstar/video/` | done |
| Flight analysis: phases from motion energy fused with the IMU (saturation-aware, glitch-proof, 3.5 g impact), attitude, horizon, trajectory and range | `grappstar/vision/flight.py` | done |
| Landing zone: scale-tolerant classical features + LDA prototypes, ONNX/TensorRT model hook, dark-landing rule, score 0 to 100 with reasons, report image | `grappstar/vision/terrain.py` | done (72/72 on held-out simulated flights) |
| Aerial map: ORB + RANSAC homography chain, quality gating, top-down rectification, landing marker | `grappstar/vision/mosaic.py` | done |
| Rover targeting: click or orange marker, flat-ground range and bearing, turn advice, calibration (checkerboard, ground board, marker), camera pose from `params.FRAMEWORK.camera` | `grappstar/vision/target.py`, `calibrate.py`, `grappstar/campose.py` | done |
| Ballistics C3: loads `base/out/ballistics.json`, interpolates pitch and drag, prefers the higher pitch (D29), pivot reference | `grappstar/ballistics.py` | done |
| Mission state machine in the D22 order with operator prompts, fault handling at every step, JSONL log, recording, replay | `grappstar/mission.py`, `app.py`, `replay.py` | done |
| Operator page (stdlib http.server, no CDN): both live videos, telemetry, docked prompts with landing report and map tabs, click-to-target, settings, manual jog and dart keys, STOP/ESTOP, log download | `grappstar/ui/` | done, driven end to end in Chromium at phone and laptop size |
| Whole-system simulator for the page (`serve --sim`) | `grappstar/sim/world.py` | done |
| Setup: install.sh, requirements.txt, systemd unit, udev rules, check_env.py, hotspot.sh, settings.json, README bring-up for a learner | `setup/`, `README.md` | done |

Mission steps (BASE_SPEC 10.1, D35): PREFLIGHT, LAUNCH_MODE, AIM, COCK, ARM, FIRE, FLIGHT, LANDING_ZONE, RAIL_LOW
(D45), BASE_NEUTRAL, WRAP, ANCHOR, TIDY, FLY_IN, CHECK, TOW, RELEASE, RETRIEVE, UNWRAP, FINAL_APPROACH, LAUNCH_READY. No-go and failed anchors retrieve and reshoot.
Confirmations: base goes live, aim, cock, fire (twice), landing verdict, tow assist.

## Verified

Last default run (`python -m pytest tests/ -q -o addopts='-m "not slow"'`, commit ce29e1a, Python 3.10, numpy 1.26):

```
397 passed, 31 deselected in 538.57s (0:08:58)
```

The slow suite (`-m slow`, both real firmwares: basesim with the real base_logic.h and fw-dart's kedge_sim) last
ran before the afternoon changes: 28 passed and 1 harness race (since fixed). It was not re-run after the weekly
limit stopped the day.

## Review rounds (fresh agents, all findings fixed, regression tests kept)

1. comms + mission: 11 findings (dart left running on an anchor timeout, ESTOP delayed by the dart link, fire retry after
   a real shot, STOP racing fire, bad JSON, ...). `tests/test_review_round1.py`.
2. video + vision: 12 findings (IMU saturation, glitch samples, launch time convention, throttle, replay, memory).
   `tests/test_review2_findings.py`.
3. mission vs firmware and D22: 11 findings (stop cancelling the firmware's safing move, final approach re-wrapping,
   reshoot limit leaving the dart out, load-cell rewind BUSY, decock warning). `tests/test_review3_findings.py`.
4. Browser test of the page: field-name mismatches, modal hiding the landing report, layout at 360 px. Fixed.
5. Coordinator relays (systest F2 to F6, R1-B, docs safety review): fixed.
6. Afternoon relays and self-review (`tests/test_review5_findings.py`, `tests/test_dart_v4_round11.py`):
   - BASE_NEUTRAL waited out the whole reset timeout after RAIL_LOW (30 s on hardware; found by profiling the tests).
   - LANDING_ZONE waited up to 40 s for a dart still in FLIGHT after the landing was decided: now a /stop.
   - O#13 final approach max_s from the firmware's own approachLimit formula (about 0.057 m/s at duty 0.3).
   - F22 stop before a new reel after HOLD; F24 watchdog 2.5 s with a fixed 0.4 s ping schedule.
   - F28 uncalibrated tow: the base `tension_cal` flag (base-1.3.0) when present, else k_n_per_a against the
     params.py default (read from base/spec/params.py) and the earlier firmware defaults.
   - D46 tow cap 55 N (page, app, CLI). fw-dart round 11: /arm refused while ANCHORED releases first; NO_GROUND
     offers only release and reshoot; hold() raises on "hold limit" and never stops a sequence it did not start.
   - Serial port opened exclusively (a second serve or base monitor gets "port busy").
   - Mock base follows base-1.3.0 (drum holds with the wraps on, SW_ARM polarity, k 13.1 / -2.3, cap 55).

## Open items

- Real footage: terrain and quality thresholds are tuned on synthetic frames only. Record real Caddx Ant clips and
  real landings, then re-learn (`python -m grappstar.vision.terrain --learn --write`).
- The fisheye of the Caddx Ant is not modelled (pinhole in the sim); mosaics and horizon pitch are approximate.
- Rover drive interface unknown: the tow asks the operator to drive. `grappstar/rover.py` is the hook.
- fw-base firmware is ahead of PROTOCOL.md (drum order, sw letters, tidy, defaults): the Jetson copes with both and
  tries `tidy`/`retrieve` with fallbacks. Align when PROTOCOL.md is updated.
- Ballistics range is from the pivot ground point; the muzzle offset is inside the physics table. Camera range offset
  uses `FRAMEWORK.camera` x (150 mm).

- Not done when the day stopped (all from relays or review round 6; none blocks a bench test):
  - fw-dart 41e0d19 items: hold() should treat a "stalled" on the very first press as a lock (retry once, then raise);
    wait_phase should use a time limit (about 10 s) for WiFi gaps instead of 15 counted misses; preflight prompt
    below 6.6 V dart rail; reuse DartPoller's status instead of polling on top; wait for log_frozen before reading
    /flightlog; new PODS_JAM fault text.
  - The firmware answers the 8 s manual hold cap with "stalled" too, so hold() cannot tell it from an end stop;
    a distinct refusal was requested from fw-dart.
  - Mock base: "wraps unknown" and hand-mode holds, `max_s` validation on `reel`, PARAMS_VERSION; the mock models
    params 1.3.2 (cap 55, k 13.1) and the `tension_cal` flag ahead of the merged firmware.
  - wait_launcher_rest counts cock null (phase unknown) as at rest when IDLE; the firmware still refuses an unsafe
    shift (LOADED_UNKNOWN), but the mission should treat null as not at rest.
  - jetson-vision: pass `dart_hfov_deg` (100) and `dart_lens` ("pinhole", "fisheye" on the real rover) settings into
    run_post_flight from mission.py and app.act_analyse.

## What Harry must measure or set up

1. **Deck height above the ground** on the rover (sets the camera height): `settings.json` `"deck_height"`.
2. **IMU axes** of the dart (BENCH_TEST.md step 7): `"imu_axes"`, for example `"y,-x,z"`.
3. **Base current constants** `free_a` and `k_n_per_a` / `tension_offset_n` (firmware/base/CALIBRATION.md (d) and
   (f)) and `limit_n`: the mission refuses TOW without a load cell until the base says `tension_cal` true.
   **Tow limit**: the dart's hand-pull holding force (TEST_PLAN stage 8 step 2) minus 10 N, never above 50 N
   (firmware cap 55 N): set it on the base (`limit_n`) and in `settings.json` `tow_limit`.
4. **Rover camera calibration** (printed 9x6 checkerboard, 25 mm squares): `python3 -m grappstar calibrate
   intrinsics --images ...`, then `calibrate mount` with the board on the ground.
5. **USB IDs** of the Kiyo X and the capture dongle (`lsusb`), then edit `setup/99-grappstar.rules`.
6. **RC832 channel** matching the VTX; check the dart view on the page.
7. Network: join KEDGE from the Jetson (README section 6), and choose how the phone reaches the page.
