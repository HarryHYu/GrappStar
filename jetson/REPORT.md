# jetson worker report

## Dart v31 round (7 Nov 2026, dart-jetson)

Goal: the dart side works first time with the real dart (real launcher about 7 m/s for 2.6 to 4.8 m, Caddx Ant
fisheye over an analog link, firmware API v4.1).

| Item | What changed | Where |
|---|---|---|
| Realistic synthetic flights | sim profile `caddx`: real launcher (C3 speed, 2 to 3 percent scatter, exit and drag fitted to base/out/ballistics.json within 0.8 percent), dart-physics' angle of attack, skid and lean, Caddx fisheye (4:3 in 720x480), auto exposure lag, motion blur, sensor and FM noise, sharpening, vignetting, chroma smear, sparkles, line jitter, interlacing, YUYV, 60 ms capture delay, repeated frames, daylight / overcast / low light / weak link | sim/flight_video.py, sim/analog.py |
| Dart lens | measured (`calibrate dart`, fisheye checkerboard) or default 135 deg; every frame rectified to the 100 deg pinhole view the vision was tuned on | vision/lens.py, vision/calibrate.py, sim/checkerboard.py |
| Flight analysis at the real speed | model from the pivot (exit 0.42 m, 0.10 m ahead), IMU pitch as the speed track, capture delay from settings, 50 Hz push integral as a lower bound only, rail and end noise floors, glitch and repeated frames, levelling-out ends the flight, relative sky brightness, horizon side filter, timing windows and priors from the realistic eval | vision/flight.py |
| Link check | torn-row test against the row neighbourhood plus black side strips (it dropped half the realistic rail frames) | video/quality.py, vision/flight.py |
| Landing verdict | ground ROI below a levelled horizon (dart lying or leaning 42 deg), classifier re-learned with 216 realistic flights | vision/terrain.py |
| Aerial map | landing marker from the flight path and the dart attitude (angle of attack) | vision/mosaic.py |
| Dart client and mission | /status spaced 0.2 s over every caller, WiFi gaps by time (10 s), first-press stall lock retried, log_frozen waited for, flight log fetched through a WiFi gap, v4.1: not upright (reshoot or force), battery low / critical, tow stopped on `moving`, fault codes in words, preflight INA219 and battery | comms/dart.py, mission.py, comms/mock_dart.py |
| Re-learning on real footage | `learn extract / latency / flight / terrain`; site files calib/flight_tuning.json (table_scale, video timing) and calib/terrain_prototypes.json read at start | learn.py, ballistics.py |
| Checked, already right | landing tilt is roll-invariant (rest_tilt); aim uses the 0.25 N column and the high branch over 6..47 deg; camera y offset (0.19 m right of the launch line) in campose; the ring buffer hold keeps the launch frames | tests/test_realistic_flight.py, test_campose.py, test_video_camera.py |

Accuracy on realistic synthetic flights (README section 13): range with the flight log median 1.2 percent (worst
4.2 to 5.5), video only median 4.4 percent; map marker median 0.07 m; landing verdict as expected 90 percent, no GO on
sand, rock or water, GO on 22 of 24 good landings (2 UNSURE).

Still open: `REST_POSES_DEG` keeps -60 (old pose; dart-physics says the dart lies or leans, and the firmware refuses
/auto nose down anyway); the video-only range has rare large misses when the landing is not seen (an IMU log
normally exists); every number above is synthetic until Harry re-learns on real footage (README section 12).

## Earlier rounds (3 Nov 2026)

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

(Dart v31 additions first.)
0a. **Dart camera lens**: `calibrate dart` with the printed checkerboard (README 7b) -> calib/dartcam.json.
0b. **Dart video capture delay**: 3 low shots, then `learn latency` (README 12) -> settings `dart_video_latency_s`.
0c. **Range against the tape**: 5 or more tape-measured shots, then `learn flight shots.csv` -> table_scale.
0d. **Landing ground pictures**: `learn extract`, sort by hand, `learn terrain labelled` (README 12).

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
