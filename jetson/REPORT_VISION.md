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

PASTE_TESTS

## Open items

- The jetson worker still has to pass `aim_pitch_deg` and `drag_n` from mission.py to run_post_flight (exact line
  sent). Without it, the pitch comes from the rail horizon or the flight time; the sim results stay within 15
  percent, but the commanded pitch is the better prior.
- sim/flight_video.py (jetson's) still draws 9..11 m/s and 25..45 deg by default. The new real-speed tests pass
  pitch and speed explicitly. Suggested: draw from the C3 table.
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
