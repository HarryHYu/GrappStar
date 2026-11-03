# Round 1 C: Jetson vision, video, ballistics (jetson)

Reviewed at base-v1 commit 0bfd59a (tree at 12:20). Method: independent finder subagents, one per lens; every finding of
medium severity or above was checked by two skeptic subagents told to refute it; kept only if fewer than
two refuted it. Low findings are listed unverified. Severity is the finder's, lowered when both skeptics
agreed it is real but rated it lower.

Confirmed: 8. Refuted and dropped: 0.

| # | Sev | File:line | Owner | Finding |
|---|---|---|---|---|
| 1 | high | `jetson/grappstar/analysis.py:71` | jetson | Landing slope uses the dart's Euler roll (spin about its own nose), so the anchor GO/NO_GO verdict depends on an arbitrary angle, not on ground tilt |
| 2 | medium | `jetson/grappstar/ballistics.py:130` | jetson | Solver uses the low arc by default, but D29 / params AIM.solver require the higher pitch when two solutions exist |
| 3 | medium | `jetson/grappstar/mission.py:71` | jetson | Default payout drag 0.75 N reads the wrong C3 column: the real launch clutch is LIGHT 0.5 N, so shots land 0.2 to 0.4 m long |
| 4 | medium | `jetson/grappstar/video/camera.py:229` | jetson | Dart video ring drops the launch and flight frames before the post-flight analysis reads them (400 MB cap holds about 13.5 s; landing wait can take longer) |
| 5 | medium | `jetson/tests/test_campose.py:122` | jetson | Camera lateral offset is dropped and the test does not check it: params camera y=-50 mm vs rail y about 140 mm gives a constant 0.19 m side miss |
| 6 | medium | `jetson/tests/test_flight.py:47` | jetson | Flight tests only cover a 9-11 m/s, 25-45 deg sim launcher with a 30% range tolerance. Video-only range at the real launcher's speed is 24-79% too long and nothing catches it |
| 7 | low | `jetson/tests/test_ballistics.py:24` | jetson | Line payout check `line_out_m >= r` passes when sag and margin are removed |
| 8 | low | `jetson/tests/test_ballistics.py:52` | jetson | Ballistics tests never use a C3 table whose rows extend past pitch_min/pitch_max, so the pitch-window clamp is untested and removing it still passes |

## 1. Landing slope uses the dart's Euler roll (spin about its own nose), so the anchor GO/NO_GO verdict depends on an arbitrary angle, not on ground tilt

- File: `jetson/grappstar/analysis.py:71`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: I took a normal flight with a C2 flightlog. The dart comes to rest nose-down at about pitch -60 deg on flat hard ground. Its roll (ZYX Euler, R = Rz(yaw) Ry(-pitch) Rx(roll)) is how far it turned about its own nose axis, and that angle builds up over the flight (the sim also adds roll_rate * t, flight_video.py:442). analysis.py passes (t, roll, 0.0, yaw) to terrain.analyze_landing. slope_from_attitude sees |pitch| <= 45, so it returns slope_from_imu(roll, 0) = |roll|. That value replaces the texture slope and gets the IMU tilt penalty. I proved it with /tmp/claude-0/review-scratch-C/roll_slope.py, using the same hard_ground image, the same pitch of -60 deg and the same flat ground: spin 0 -> slope 0, score 90 GO; spin 30 -> slope 30, score 65 GO; spin 90 -> slope 90, score 0 NO_GO; spin 150 -> slope 150 (more than 90 deg is physically impossible), score 0 NO_GO. With no attitude the result is slope 3.0 (texture), score 90 GO. So a good anchor is called NO_GO, leading to a pointless retrieve and relaunch, whenever the dart happens to rest with a large roll. A real lean is never measured either, because the texture slope is skipped whenever an attitude is passed.

Evidence: analysis.py:68-71
    rest = [a for a in fres.attitude if a[0] >= fres.t_landed]
    if rest:
        t_, roll_, pitch_, yaw_ = rest[-1]
        att = (t_, roll_, 0.0, yaw_)
terrain.py:740-742
    if abs(pitch) > 45.0:
        return max(0.0, 90.0 - abs(pitch))
    return slope_from_imu(roll, pitch)
terrain.py:945-948
    tilt = slope_from_attitude(attitude)
    if tilt is not None:
        imu_roll_deg, imu_pitch_deg = tilt, 0.0
terrain.py:1064-1072: slope_src = "imu", pen = ... 15 + (slope - 25) * 2.0
flight.py:41 convention: roll about the nose (spin about body x), which for a nose-down dart is rotation about a near-vertical axis and tilts nothing (terrain.py:726 says this itself).

Fix: In analysis.py, stop passing roll as a tilt. The minimal correct change is to leave att = None (drop lines 66-71) so the slope comes from slope_from_texture. If a tilt cue from the IMU is wanted, compute it from the gravity direction, which does not depend on roll: with the resting pitch p, tilt from straight down = 90 - |p|. Compare it with the expected nose-first landing pitch, e.g. att = (t_, 0.0, pitch_, yaw_), so slope_from_attitude takes its |pitch| > 45 branch, and subtract the nominal landing pitch before applying the penalty. Never feed the Euler roll of a nose-down body into slope_from_imu.

> Skeptic (confirmed): I could not refute this; the defect is real. The code does what the claim says. In analysis.py:66-71 the last resting attitude sample is passed as (t, roll, 0.0, yaw). Because pitch is forced to 0, slope_from_attitude (vision/terrain.py:740-742) always takes the |pitch| <= 45 branch and returns slope_from_imu(roll, 0) = |roll|. analyze_landing (terrain.py:945-948) puts that value into imu_roll_deg. When imu values are present, the slope block (terrain.py ~1062-1072) skips slope_from_texture and applies the stronger IMU tilt penalty. By the flight.py convention (R = Rz(yaw) Ry(-pitch) Rx(roll),
>
> Skeptic (confirmed): I could not refute it. The defect is real and I reproduced it.

How the code runs:
- analysis.py:66-71 takes the last resting attitude sample and passes (t, roll, 0.0, yaw). The pitch is replaced with 0 on purpose. The comment there says roll is the "side tilt, which does follow the ground", but the module's own convention says otherwise. vision/flight.py:40-41 defines roll as rotation about the nose (body x), with R = Rz(yaw) Ry(-pitch) Rx(roll), and quat_to_rpy at flight.py:145-147 computes it that way. For a dart resting nose-down, that is spin about its own long axis. It says nothing about
>

## 2. Solver uses the low arc by default, but D29 / params AIM.solver require the higher pitch when two solutions exist

- File: `jetson/grappstar/ballistics.py:130`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: Target 4.85 m at drag 0.5 N, with the real base/out/ballistics.json. The range rises to 4.95 m at about 38.5 deg and then falls, so there are two crossings. Mission.aim (mission.py:447) and app._update_solution (app.py:392) call solve(rng, drag_n) without lob. The solver returns pitch 33.07 deg (t_flight 0.865 s). D29 says to prefer the higher pitch, which is 45.26 deg here. Checked by running: solve(4.85,0.5).pitch_deg=33.07 and solve(4.85,0.5,lob=True).pitch_deg=45.26. Every target between about 4.79 and 4.95 m (drag 0.5) is fired on the branch the spec rules out. The unit test test_low_branch_default_lob_optional locks in the wrong choice.

Evidence: ballistics.py:129-130 `cands.sort(key=lambda c: c[0])` / `p, t, apex, v = cands[-1] if lob else cands[0]`; module docstring ballistics.py:11 'Use the rising branch: the lower pitch'. DECISIONS.md D29: 'the solver uses the whole 6..47 deg table and prefers the higher pitch when two solutions exist'. params.py AIM.solver: 'prefer the higher pitch when two solutions exist'. mission.py:447 `sol = self.bal.solve(rng, self.cfg.drag_n)` (no lob).

Fix: Make the high branch the default: `p, t, apex, v = cands[0] if low else cands[-1]`. Rename the flag to `low: bool = False` (or default lob=True) and update the docstring, the 'lob (high branch)' note and test_low_branch_default_lob_optional so the default returns the higher pitch.

> Skeptic (confirmed): I could not refute this. The code does contradict the binding spec.

- **Spec:** DECISIONS.md D29 (spec v1.1) says "the solver uses the whole 6..47 deg table and prefers the higher pitch when two solutions exist". params.py:197 AIM.solver says the same. No later decision (up to D32) supersedes it.
- **Code:** ballistics.py:129-130 sorts the candidates by pitch and returns cands[0], the lowest pitch, unless lob=True. The module docstring (lines 11-12) states the opposite policy: "Use the rising branch: the lower pitch".
- **Callers:** none of the mission or app callers passes lob, so they alway
>
> Skeptic (confirmed): The claim holds up. The code does what the reviewer says: ballistics.py:129-130 sorts the candidates by pitch and takes cands[0] (the lowest pitch) unless lob=True, and the module docstring (line 11) says to use the lower pitch. The spec says the opposite. DECISIONS.md D29 says the solver "prefers the higher pitch when two solutions exist", and params.py:197 AIM.solver says "prefer the higher pitch when two solutions exist". The workplan's C3 section gives only the table format, nothing that overrides D29, and no later decision supersedes it. The production callers never pass lob: mission.py:4
>

## 3. Default payout drag 0.75 N reads the wrong C3 column: the real launch clutch is LIGHT 0.5 N, so shots land 0.2 to 0.4 m long

- File: `jetson/grappstar/mission.py:71`
- Severity: medium (claimed high); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: Normal run with the shipped defaults. Operator taps a target 4.4 m out. app._update_solution (app.py:392) and Mission.aim (mission.py:447) call bal.solve(4.4, cfg.drag_n=0.75), which returns pitch 32.23 deg. The launch clutch is set LIGHT, which is 0.5 N at the line with the reel full (D24, BASE_SPEC 11: 'about 4.5 m at 35 deg with the 0.5 N brake'). ballistics_meta.json defines drag_n as exactly that clutch setting at the reel exit with the reel full. Checked by running against base/out/ballistics.json: at_pitch(32.23, 0.5) gives 4.82 m, so the dart lands 0.42 m long. Other cases: target 4.0 m gives 23.29 deg and lands at 4.29 m; target 3.5 m gives 16.98 deg and lands at 3.69 m. Even at the reel-core worst case (+23 %, about 0.62 N) it is 4.62 m against 4.40 m.

Evidence: mission.py:71 `drag_n: float = 0.75            # payout brake setting (C3 column)`; cli.py:297 `s.add_argument("--drag", type=float, default=0.75, ...)`, also cli.py:350; settings.example.json:11 `"drag": 0.75,`; ballistics.py:104 `def solve(self, range_m: float, drag_n: float = 0.75, ...`. DECISIONS D24: 'Clutch FIRM 5.0 N, LIGHT 0.5 N, both as line tension with the storage reel FULL'. ballistics_meta.json notes: 'drag_n: slip-clutch setting, expressed as the line pull at the reel exit with the reel FULL'. D16: 'LAUNCH LIGHT (0.5 N)'.

Fix: Set the default to 0.5 N (the LIGHT clutch, D24) in MissionConfig.drag_n (mission.py:71), cli.py:297 and :350, settings.example.json:11, and as Ballistics.solve/at_pitch's default drag_n (ballistics.py:86, 104). Better still, read it from params.DRUM/clutch LIGHT_line_n so it follows the spec.

> Skeptic (confirmed): I couldn't refute this. The claim is real, and I reproduced its numbers.

**What the spec and data say the setting should be**
- D24 and params.py `clutch.LIGHT_line_n = 0.5` (line pull with the reel full) set LIGHT at 0.5 N.
- D16 and `DRUM.zones` put the clutch in LIGHT during LAUNCH.
- BASE_SPEC section 11 says "about 4.5 m at 35 deg with the 0.5 N brake".
- base/out/ballistics_meta.json defines the drag_n column as exactly this clutch setting: the line pull at the reel exit with the reel full.
- base/out/physics/summary.json tells the builder to "set 0.4 to 0.5 N (FINDINGS F5)".
- Contract
>
> Skeptic (confirmed): I could not refute this. The defect is real.

1. The spec puts LAUNCH at the LIGHT clutch, 0.5 N:
   - DECISIONS D16: LAUNCH LIGHT (0.5 N).
   - DECISIONS D24: LIGHT 0.5 N as line tension with the reel FULL.
   - BASE_SPEC line 30: clutch set LIGHT (0.5 N at the line).
   - params.py line 132: the LAUNCH zone is clutch="LIGHT".
   - params.py line 208: clutch LIGHT_line_n=0.5.
   Nothing in the spec gives 0.75 N.

2. The table uses the same 0.5 N value. base/out/ballistics_meta.json says drag_n is the slip-clutch setting, as line pull at the reel exit with the reel FULL. It also says the coupl
>

## 4. Dart video ring drops the launch and flight frames before the post-flight analysis reads them (400 MB cap holds about 13.5 s; landing wait can take longer)

- File: `jetson/grappstar/video/camera.py:229`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: Real dongle at 720x480 BGR and 30 fps is 1,036,800 B per frame. PAL at 720x576 and 25 fps is the same data rate. With BUFFER_MAX_MB=400 the ring keeps at most 404 frames, about 13.5 s, whatever buffer_s=15 says. landing_zone() only reads frames from that ring: c.buffer() filtered to t >= t_fire - pre_s (2 s). It reads them after wait_landing() and settle_s. Case 1 (no operator wait): the dart Wi-Fi is down after impact and the tether keeps creeping (wind, dart rolling), so only 'video' is detected. The loop then runs to land_timeout_s=10 s, then settles 2 s. That is 2 + 10 + 2 = 14 s or more from t_fire - pre_s, so the oldest frames have already been evicted. Case 2 (operator prompt): nothing is detected, so the mission calls ask('Landing not detected ... Is the dart down?'), which has no timeout, after the 10 s wait. Once the operator takes more than about 1.5 s to answer, every frame up to and past the launch is gone. analyze_flight then finds no launch ('no launch found'), with no trajectory, range or video clock offset. The mosaic uses a wrong flight window, and the operator's GO/NO-GO verdict is based on partial data. The recorded AVI has these frames, but the analysis never uses it.

Evidence: video/camera.py:31 `BUFFER_MAX_MB = 400.0`; camera.py:227-230 `while len(self._ring) > 1 and (now - self._ring[0][0] > self.buffer_s ... or self._ring_bytes > self.buffer_max_bytes): self._ring_bytes -= _nbytes(self._ring.popleft()[1])`; app.py:146 `deinterlace=c.dart_deinterlace, buffer_s=15)` (no buffer_max_mb, so the 400 MB default caps it at about 13.5 s); mission.py:90 `land_timeout_s: float = 10.0`; mission.py:632 `self.ask("Landing not detected (no dart signal, line still moving or no data). Is the dart down?", ...)` (ask() takes timeout=None and blocks with no limit); mission.py:636 `self.sleep(self.cfg.settle_s)`; mission.py:659 `frames = [f for f in c.buffer() if f.t >= t_fire - self.cfg.pre_s]` (runs only after all of the above). 400*1024*1024/(720*480*3)/30 = 13.48 s.

Fix: Take the flight frames before any unbounded wait. In Mission.wait_landing, store `self.shot['frames'] = [f for f in self.cams['dart'].buffer() if f.t >= t_fire - self.cfg.pre_s]` as soon as the loop exits, before the ask() at mission.py:632 and before settle. Then in landing_zone, append the frames received since then (c.buffer() filtered to t > last snapshot t) instead of re-reading the whole ring. Also make the cap match buffer_s for the dart camera: in app.py:145-146, pass buffer_max_mb=600 (15 s x 30 fps x 1.04 MB = about 470 MB, plus margin), or compute it from buffer_s and the frame size. When the frames before t_fire - pre_s are missing, fall back to reading the shot AVI with FileSource.

> Skeptic (confirmed): I read the code and could not refute this. Each cited line says what the claim says.

The ring cap: camera.py:227-230 drops old frames when they are older than buffer_s, when there are more than 4000 of them, or when they exceed buffer_max_bytes (400 MB by default, line 31). app.py:145-146 sets the dart camera to the default dart_size (720,480) and dart_fps 30 with buffer_s=15 and passes no buffer_max_mb. In YUYV mode the frames are delivered as BGR, so each frame is 720*480*3 = 1,036,800 B and the cap holds about 404 frames, or 13.48 s.

When the frames are read: landing_zone() (mission.py:65
>
> Skeptic (confirmed): I checked the claim against the code and it holds. The scenario needs degraded conditions, though.

1. **The ring cap.** In jetson/grappstar/video/camera.py:44-50 the ring cap defaults to 400 MB. app.py:145-146 builds the dart Camera with buffer_s=15 and passes no buffer_max_mb. The dart source is 720x480 YUYV (app.py:36-38). The GStreamer sink converts it to BGR (capture.py:123), so each frame is 1,036,800 bytes. Deinterlacing keeps that size. The eviction loop at camera.py:227-230 drops the oldest frames once ring bytes exceed the cap. That leaves about 404 frames, about 13.5 s at 30 fps, so
>

## 5. Camera lateral offset is dropped and the test does not check it: params camera y=-50 mm vs rail y about 140 mm gives a constant 0.19 m side miss

- File: `jetson/tests/test_campose.py:122`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: base/spec/params.py places the Kiyo X at pos (150, -50, 100) mm, with +Y to the left and the rail plane at y=140 (rail_plane_y), so the camera sits about 190 mm right of the launch line. find_camera_pose returns y_m, but nothing reads it: camera_model applies only the x offset (range_offset_m), and app._set_target adds only that range offset. The operator turns the rover until the camera bearing reads 0, which puts the target on the camera's line, 0.19 m right of the parallel launch line, so every shot lands about 0.19 m left of the target. test_base_spec_v1_camera_entry feeds pos y=-50.0 but asserts only height, pitch and range_offset. Any handling of y (none, wrong sign, wrong units) passes. Mutation: change `"y_m": y / 1000.0` to `"y_m": 0.0` or delete it; the test still passes.

Evidence: tests/test_campose.py:122-127 `pos=(150.0, -50.0, 100.0) ...` then asserts only `m.height_m`, `m.pitch_deg == 10.0`, `m.range_offset_m - 0.15`; grappstar/campose.py:65 `"y_m": y / 1000.0` (never read: grep finds no other use); grappstar/app.py:380-384 only `info["range_m"] = round(info["range_m"] + off, 2)`; base/spec/params.py:228 `camera=dict(model="Razer Kiyo X", pos=(150.0, -50.0, 100.0), ...)` and :113 `rail_plane_y=140.0`

Fix: In campose.camera_model, store m.lateral_offset_m = pose['y_m'] - rail_y_m, with rail_y_m taken from params LAUNCH/FRAMEWORK rail_plane_y (in metres, +left). In app._set_target, convert the target's camera-frame ground point (cam_model.ground_point) to pivot coordinates (X += lateral offset with the sign flipped for the +right convention, Y += range_offset_m). Then compute range_m = hypot and bearing_deg = atan2 relative to the launch line. Extend test_base_spec_v1_camera_entry to assert lateral_offset_m == pytest.approx(-0.19) when rail_plane_y=140 is in params. Add an app/target test asserting that a target straight ahead of the camera at 4 m reports a bearing of about atan2(0.19, 4) = 2.7 deg left of the launch line.

> Skeptic (confirmed): I checked each part of the claim against the code and it holds. One citation is wrong: the test is at jetson/tests/test_campose.py:38-45, not line 122 (the file has 45 lines).

1. The geometry is real. In base/spec/params.py, FRAME sets +Y to the left. FRAMEWORK.camera.pos is (150, -50, 100) mm. LAUNCH/FRAMEWORK rail_plane_y is 140.0, and the rail body spans y 95 to 185. The dart leaves in the rail plane at y≈140 while the camera sits at y=-50, so the camera is about 190 mm to the right of the launch line, looking along +X (look="+X", pitch only).

2. The y value is never used. campose.find_ca
>
> Skeptic (confirmed): I could not refute this. The line numbers in the claim are wrong, but what it says about the code is accurate.

- **The test.** The test is at jetson/tests/test_campose.py:38-45, not :122. It writes pos=(150.0, -50.0, 100.0) and checks only height_m, pitch_deg and range_offset_m, so nothing tests y.
- **y is never used.** campose.find_camera_pose (campose.py:64-65) returns "y_m": y/1000.0. Searching the whole jetson tree finds no other reference to y_m or to a lateral offset. camera_model sets only height_m, pitch_deg and range_offset_m (from x_m - pivot_x_m), and app._set_target (app.py ~375-
>

## 6. Flight tests only cover a 9-11 m/s, 25-45 deg sim launcher with a 30% range tolerance. Video-only range at the real launcher's speed is 24-79% too long and nothing catches it

- File: `jetson/tests/test_flight.py:47`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: Real launcher (base/out/ballistics.json): v0 about 7.6-7.7 m/s, pitch 6-47 deg, range 2.3-4.95 m, flight 0.34-1.1 s. I ran render_flight(seed, size=(360,240), pitch_deg=10, v0=7.7) and analyze_flight(frames) with no flightlog (script /tmp/claude-0/review-scratch-C/short_flight.py). True range 3.81 m; video-only estimates were 4.71 / 6.83 / 4.91 / 6.00 / 5.72 m for seeds 0/1/4/7/9 (seed 1: v0 solved as 11.5 m/s against 7.7 true). At pitch 20 / 7.66 m/s, true range 4.89 m, the estimates were 5.29-6.52 m. With a flightlog the error drops to about 5%. The cause is that a 50-90 ms impact-time error on a 0.5 s flight inflates v0 in the flight-time bisection. test_phases never runs in this regime: the sim draws pitch uniform(25,45) and speed uniform(9,11), and test_sim_flight asserts 0.9 < flight < 1.8 s and 5 < range < 12 m. It also allows 30% error. So a mutation such as adding +0.06 s to the video impact time in _video_phases would still pass. In a mission without the dart log, mission.tell_line_out sends the base line_m = range*1.05+0.3, which can be about 7.5 m instead of about 4.3 m.

Evidence: tests/test_flight.py:15 `RANGE_TOL = 0.30     # fraction of the true range`; :27-30 seeds only, no pitch_deg/v0 override; :47 `assert abs(r.range_m - truth["range_m"]) < RANGE_TOL * truth["range_m"]`. grappstar/sim/flight_video.py:588 `pitch = float(rng.uniform(25, 45))`, :590 `speed = float(rng.uniform(9.0, 11.0))`. tests/test_sim_flight.py:32-34 `assert 0.9 < truth["t_impact"] - truth["t_launch"] < 1.8` / `assert 5.0 < truth["range_m"] < 12.0`. grappstar/vision/flight.py:688 `v0 = _bisect(lambda v: tf(v, pitch), 2.0, 30.0, T)`. grappstar/mission.py:714-715 `est = fl["range_m"] * 1.05 + 0.3`.

Fix: Add test_phases cases that pass render_flight(..., pitch_deg in (8, 20, 40), v0=7.65), i.e. the real C3 envelope, with a video-only range tolerance of 15% or less. Make the sim defaults draw pitch and v0 from the C3 table range (pitch 6-47, v0 7.6-7.7). Fix the video-only path so the test passes: when there is no IMU, clamp the solved v0 to the C3 table's v_muzzle range (Ballistics.load().rows) and use the table's range at the measured pitch instead of the flight-time-bisected v0. Also have mission.tell_line_out skip the flight range when FlightResult.source == 'video' and use the ballistics line_out instead.

> Skeptic (confirmed): I could not refute it, and I reproduced it. I ran the reviewer's script against the current code (/tmp/claude-0/review-scratch-C/short_flight.py, which calls render_flight(seed, (360,240), pitch_deg, v0) and then analyze_flight with and without the flightlog).

Video-only results:
- Pitch 10 deg, v0 7.7 m/s, true range 3.81 m: estimates 4.91 / 6.00 / 5.72 m for seeds 4/7/9, so 29-57% too long.
- Pitch 20 deg, true range 4.89 m: estimates 5.29-6.28 m.
- Pitch 35 deg, seed 7: 6.77 m against 5.75 m, so 18% too long.
- With the IMU log the error stays within about 5%.

The cause is as described. V
>
> Skeptic (confirmed): I could not refute this. Every cited fact checks out, and I re-ran the scenario.

Cited code, all as stated:
- jetson/tests/test_flight.py:15 sets RANGE_TOL = 0.30.
- test_phases at test_flight.py:27-47 only varies seed, so it never passes pitch_deg or v0.
- flight_video.py:588/590 draws pitch from uniform(25,45) and speed from uniform(9,11) when no override is given.
- vision/flight.py:688 bisects v0 from the flight time T when there is no IMU v0.
- mission.py:714-715 takes fl["range_m"]*1.05+0.3 whenever 0.5 < range < 30. It does not check FlightResult.source.

The video-only path really hap
>

## 7. Line payout check `line_out_m >= r` passes when sag and margin are removed

- File: `jetson/tests/test_ballistics.py:24`
- Severity: low (claimed low); unverified (low)
- Owner: jetson

Scenario: Mutation: replace `line = (math.hypot(range_m, 0.0) + sag) * line_margin` with `line = range_m + 0.01`. This drops the 10% margin and the sag term, and the whole test_ballistics.py still passes (verified). The fallback line_m that mission.tell_line_out sends the base when there is no flight range, and the 'line out about X m' operator prompt, would then be about 10% short and untested.

Evidence: tests/test_ballistics.py:24 `assert s.t_flight_s > 0 and s.line_out_m >= r`; grappstar/ballistics.py:131-132 `sag = 8.0 * (apex * 0.25) ** 2 / (3.0 * max(range_m, 0.1))` / `line = (math.hypot(range_m, 0.0) + sag) * line_margin`

Fix: Assert the value: `assert s.line_out_m == pytest.approx(round((r + 8*(s.apex_m*0.25)**2/(3*r)) * 1.10, 2), abs=0.02)`. Also add a solve(..., line_margin=1.0) case asserting a line_out about 10% lower.


## 8. Ballistics tests never use a C3 table whose rows extend past pitch_min/pitch_max, so the pitch-window clamp is untested and removing it still passes

- File: `jetson/tests/test_ballistics.py:52`
- Severity: low (claimed medium); 1 of 2 skeptics confirmed
- Owner: jetson

Scenario: Mutation: in Ballistics.solve, replace `ok_idx = [i for i, p in enumerate(ps) if self.pitch_min - 1e-9 <= p <= self.pitch_max + 1e-9]` with `ok_idx = list(range(len(ps)))`. All tests in test_ballistics.py and test_campose.py still pass (ran in /tmp/claude-0/review-scratch-C/j). Every table the tests use has pitch_min/pitch_max equal to the first and last rows: the placeholder is 15..60 with rows range(15,61,5), and the test_c3 table is 20..40 with rows 20 and 40. With the mutated code, a C3 table with rows 10..45 and pitch_min=20, pitch_max=35 makes solve(8.6) return reachable=True at pitch_deg=41.0, outside the allowed rail window. C3 carries pitch_min/pitch_max separately from the rows, so the base physics model may legitimately emit rows beyond the mechanical window.

Evidence: grappstar/ballistics.py:113 `ok_idx = [i for i, p in enumerate(ps) if self.pitch_min - 1e-9 <= p <= self.pitch_max + 1e-9]`; tests/test_ballistics.py:9-10 fixture `Ballistics(make_placeholder(path=None), ...)` (ballistics.py: `"pitch_min": 15.0, "pitch_max": 60.0` with `for pitch in range(15, 61, 5)`); tests/test_ballistics.py:52-54 `{"version": 1, "pitch_min": 20, "pitch_max": 40, ... "pitch_deg": 20 ... "pitch_deg": 40`

Fix: Add a test with a table whose rows run 10..45 and pitch_min=20, pitch_max=35. Assert that solve() for a range reachable only at pitch 41 returns reachable=False, that every reachable solution has pitch_min <= pitch_deg <= pitch_max, and that range_min_m/range_max_m come only from rows inside the window. Also add a test that loads the shipped base/out/ballistics.json (REPO_TABLE) and solves 3.0 m and 4.5 m at drag 0.75. Today test_load_default only asserts `bl.rows and bl.pitch_min < bl.pitch_max`.

> Skeptic (refuted): The narrow fact is true: every table the tests use (the placeholder with pitch 15..60, the test_c3 table with 20..40, and mission_harness, which also uses the placeholder) has pitch_min/pitch_max equal to its first and last rows. So the ok_idx filter at jetson/grappstar/ballistics.py:113 is never exercised, and a mutation that removes it would survive. But the claim's severity depends on its stated premise that "the base physics model may legitimately emit rows beyond the mechanical window", and that premise is false.

The only C3 producer is base/physics/ballistics.py. It builds rows from lo/
>
> Skeptic (confirmed): I could not refute this. The test gap is real. grappstar/ballistics.py solve() (around lines 110-113) is the only code that limits solutions to the pitch window: it builds `ok_idx` by filtering rows to pitch_min..pitch_max. None of the tests can tell that filter apart from `list(range(len(ps)))`:
(a) The placeholder fixture has pitch_min=15 and pitch_max=60, and its rows run range(15,61,5), so every row is inside the window.
(b) test_c3_file_format uses pitch_min=20 and pitch_max=40, with rows only at 20 and 40.
(c) test_load_default loads the shipped base/out/ballistics.json. That window is 6
>
