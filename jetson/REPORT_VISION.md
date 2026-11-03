# jetson-vision report (3 Nov 2026)

Worker `jetson-vision`, branch `w/jetson-vision`. Owns jetson/grappstar/vision/*, analysis.py, campose.py,
ballistics.py, video/* and their tests. Task list: review R1-C (review/r1_C_jetson_vision.md on origin/w/review).

## Done

| R1-C | What changed | Where |
|---|---|---|
| 1 (high) | Landing tilt is roll-invariant. The Euler roll (spin about the nose) is never used. `analysis.rest_tilt()` takes the still gravity vector in the body frame after landing (accelerometer, else the quaternion pitch), gets the nose elevation asin(f_x / abs(f)), and compares it with the nearest expected flat-ground pose (`REST_POSES_DEG = (0, -60)`: lying flat or nose buried). That tilt feeds the terrain slope penalty. | analysis.py, tests/eval_terrain_flight.py |
| 2 | The solver prefers the HIGHER pitch when two solutions exist (D29, `PREFER_HIGH`) and uses the whole pitch_min..pitch_max window (6..47). Default drag is the LIGHT clutch, 0.5 N (`LIGHT_DRAG_N`). `low=True` gives the low branch; `lob` keeps the jetson semantics (None = default, True = high, False = low). Range is from the ground under the rail pivot. | ballistics.py |
| 3 | Dart video ring: the RAM cap follows buffer_s (buffer_s x measured fps x frame size x 1.25, 400..1200 MB), so 15 s of 720x480 at 30 fps fits (was 13.5 s). New `Camera.hold(t_from)` / `release_hold()` / `held()`: held frames are never dropped by age or count; over the cap the oldest are JPEG-packed in memory (about 10x smaller) and `buffer()` decodes them. A forgotten hold ends after 300 s. The jetson worker wired hold/release into mission.py (fire lock, landing_zone, finally). | video/camera.py |
| 4 | Camera lateral offset kept: `find_camera_pose` returns y_m and rail_y_m (LAUNCH.rail_y, else LAYOUT.rail_plane_y). `camera_model` sets `.lateral_offset_m` (-0.19 m with v1 params: the camera is 0.19 m RIGHT of the launch line). New `campose.pivot_target()` and `target_info_pivot()` give range and bearing from the pivot and the launch line. app.py uses them (jetson applied it). | campose.py |
| 5 | Video-only range estimator rebuilt (`vision/flight.fuse_speed`). Muzzle speed is a 1-D search that fuses: (a) the horizon pitch track in flight (the dart flies nose first, so its pitch follows the flight path angle; the slope of tan(gamma) gives the horizontal speed), (b) the flight time with an asymmetric video timing window (impact seen 0 to 0.11 s late), (c) the last frame that still showed sky (lower bound on the flight time), (d) the C3 table muzzle speed as a prior (20 percent), (e) a search over a late-seen launch (sky views). The range is then scaled to the C3 table at that pitch. Pitch comes from the IMU, else the commanded rail pitch (`aim_pitch_deg`), else the rail horizon, else the flight time. `run_post_flight` takes `aim_pitch_deg`, `drag_n` and `ballistics` ("auto" loads base/out/ballistics.json; a placeholder table is not used). | vision/flight.py, analysis.py |
| 6 | Low findings: line payout is now asserted by value (and with margin 1.0); the pitch window clamp is tested with rows outside pitch_min..pitch_max. | tests/test_ballistics.py |
| 7 | mission.py drag 0.5 N: jetson worker's file, done there. | |

Why the old video-only estimate was 24 to 79 percent long: at low pitch the 0.5 m launch height sets most of
the flight time (at 10 deg only 23 ms per m/s), so a 0.1 s late video impact turned into several m/s of
speed. The horizon track does not depend on the impact time, so it fixes that.

## Verified

Venv: Python 3.13 with numpy<2, opencv-python-headless, pyserial, pytest (the code itself keeps to 3.10 syntax).

My files (vision, video, analysis, campose, ballistics tests), at 95b6dec plus the report:

    $ python -m pytest -c pytest.ini --rootdir . tests/test_flight.py tests/test_sim_flight.py tests/test_terrain.py
      tests/test_terrain_flight.py tests/test_target.py tests/test_mosaic.py tests/test_campose.py tests/test_ballistics.py
      tests/test_textures.py tests/test_video_*.py
    ======================= 205 passed in 139.65s (0:02:19) ========================

Whole jetson suite at bc19732 (round 3 fixes, base-v1 merged at 03:46 UTC):

    $ python -m pytest -c pytest.ini --rootdir . -rf tests
    FAILED tests/test_comms_base.py::test_fault_in_reel_parks_at_wrapped - BaseTimeout
    FAILED tests/test_comms_base.py::test_estop_parks_by_fault_rules - BaseError: base refused MODE: DRUM_HELD
    FAILED tests/test_comms_base.py::test_d35_sequence_mock_matches_basesim - 'base-1.3' == 'base-1.2'
    3 failed, 392 passed, 29 deselected in 1110.30s (0:18:30)

All three are the mock base against the newer base firmware (jetson comms / fw-base), not my files.

Earlier whole-suite run (95b6dec):

    $ python -m pytest -c pytest.ini --rootdir . -rf tests
    FAILED tests/test_comms_base.py::test_d35_sequence_mock_matches_basesim - AssertionError: assert 'base-1.2.1' == 'base-1.2'
    1 failed, 377 passed, 29 deselected in 544.86s (0:09:04)

The one failure is the mock base's firmware version string against the real base firmware: jetson comms / fw-base,
not my files.

Simulator accuracy of the new video-only range (sim speeds 6, 7, 8 m/s; seeds 2, 5, 7 not used for tuning):
with the commanded pitch the error is -5 to +10 percent at 8, 20 and 40 deg; without it -12 to +13 percent.
The old estimator was +18 to +79 percent at these speeds.

Independent review (fresh subagent, code only): 4 confirmed findings, all fixed in 1676e45. (1) A long hold could
decode GBs and dropped the launch frames first. Now buffer(since=) is used and only frames after the first 8 s of
the hold are thinned. (2) The speed model started the flight half a stroke low. Now it starts at the rail exit,
like the range model. (3) The aim pitch was not passed from the mission: the jetson worker wired it. (4) A range
exactly on a table row gave a false "two solutions" note. Then coordinator R2-C2 (80b6441): the speed is searched
only in 0.6 to 1.4 x the table v_muzzle, the default speed comes from the table, and an expired hold warns. The
placeholder table now uses the real 6..47 deg window (95b6dec). With the high branch, the mission tests' 60 deg
placeholder asked the base for 59.6 deg, which the base refuses.

Review round 2 (fresh subagent on mosaic, target, terrain, calibrate, video/*, analysis; 448571a):
- HIGH, fixed: a void (14 black landed frames flagged bad by the link check) plus one lit rim frame reached terrain
  as a single clean frame and got GO, because landing_frames() drops bad frames before terrain's own dark check.
  run_post_flight now runs terrain's dark check on the whole landed window first. The regression test
  (test_void_with_one_lit_frame_is_not_go) fails on the old code and passes now.
- Fixed: grey or HxWx1 images crashed analyze_landing (now converted to BGR); the analysis-failed fallback said
  score 0 with UNSURE (now 45, which is UNSURE); the mosaic degenerate test passed even with a crashing mosaic; the
  capture test could not tell read-back from requested values (now a stubborn fake driver).
- Also in this round: with no known pitch, the video-only path fits pitch and speed together to the horizon track,
  and a rail view that is all sky bounds the pitch from below (half the vertical field of view).

Review round 3 (fresh subagent on flight.py phases, IMU, clock; 6fc97c8), all fixed with tests:
- HIGH: the clock offset averaged the launch and impact matches; the video impact runs late, so the IMU-timed range
  came out 8 to 18 percent short at 6 deg and 6 m/s. Now the offset comes from the launch match alone; the
  impact match is used only without a video launch, corrected by 35 ms.
- HIGH: a NaN, null, string or short row in the flightlog, or a string t_launch_ms, made analyze_flight raise
  and lose the whole flight result. Rows are now sanitised (finite numbers only, sorted, duplicate times dropped),
  and a garbage log falls back to video only.
- MEDIUM-HIGH: a log that starts after the launch took the impact spike for the launch (range 0.37 m against 8.2 m).
  Now t_launch_ms is trusted, with no rail attitude or speed from the IMU, and a note is added.
- MEDIUM: duplicate timestamps gave dt = 0 (v0 10x too small); dropped samples biased the push integral. Now dt
  comes from positive steps and the integral uses the per-sample time steps.
- LOW: the launch shift found by the speed fit is now recorded as launch_fit_t (phases keep the observed launch).

Review round 4 (fresh subagent on the newest code; 668c7d7, f6e3fce), all fixed with tests:
- MEDIUM: the round 2 dark check used the LAST 45 landed frames, so a link dying 2.7 s after a good landing turned
  GO into NO_GO. It now uses the FIRST frames after landing (a void at landing is still caught).
- LOW: a flightlog without t_launch_ms that starts in free fall took the impact for the launch. A launch now
  needs about 1 g at rest just before the push.
- LOW: a non-numeric or NaN clock_offset_s in the flightlog lost the flight result; it is now ignored.
- LOW: Ballistics.solve crashed when no table row lies inside the pitch window; now unreachable with a note.
- Speed model tracks are cached: an analysis without a known pitch takes 0.7 s on this PC (0.5 s with the
  commanded pitch).

Terrain verdicts with the roll-invariant tilt (eval_terrain_flight, seeds 0 to 2, 480x320, with and without the
flightlog): accuracy 1.000 (36/36), unsafe GO (sand or water judged GO): 0.

## Open items

- Without a commanded pitch, video-only shots above about 40 deg read 5 to 11 percent short in the sim (one case at
  46 deg: 18 percent). The launch is hard to see against plain sky and the pitch fit lands low. With the commanded
  pitch (now passed by the mission) they are within 0 to 6 percent.

- mission.py now passes aim_pitch_deg, drag_n and buffer(since=), and calls hold / release_hold (jetson worker, on
  w/jetson with fallbacks). Check them after the coordinator merges both branches.
- sim/flight_video.py (jetson's) still draws 9..11 m/s and 25..45 deg by default. The --sim world passes the aimed
  pitch and the table speed; the real-speed tests pass both explicitly.
- test_comms_base version string (above): jetson / fw-base.
- Running pytest from the repo root collects firmware/base/test/test_gen_params.py, which calls sys.exit at import
  and stops the run. Run from jetson/ with `-c pytest.ini --rootdir .` (fw-base could guard that file with
  `if __name__ == "__main__"`).
- The real Caddx Ant is a fisheye (about 150 deg). The horizon pitch uses a pinhole model with hfov 100 deg. Once
  the camera is calibrated, pass its effective hfov to analyze_flight.
- `REST_POSES_DEG` (0 and -60 deg) is an assumption: check it on real landings and edit the tuple.

## Assumptions

- Muzzle speed spread around the C3 table: 20 percent (prior sigma). Horizon noise in flight 3 deg, with
  correlated samples counted 1 in 2. Video impact late by 0 to 0.11 s, early by at most 0.03 s. These numbers
  come from the flight simulator, not from real video.
- Tests compare against a C3 table made from the sim physics at the REAL table's muzzle speed (read from
  base/out/ballistics.json, never hard-coded), because the sim uses its own drag model. The true speeds tested
  are 6, nominal and 8 m/s.
- The reviewer said a target straight ahead of the camera reads "2.7 deg left of the launch line". The geometry
  (camera at y = -50 mm, rail at y = 140 mm, +Y left) puts it about 2.6 deg to the RIGHT. That is why uncorrected
  shots land left. The tests assert right.
- The RAM caps (1200 MB automatic maximum, 400 MB of packed held frames) assume the 8 GB Orin Nano with nothing
  else large in memory.
