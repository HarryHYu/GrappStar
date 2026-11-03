# Round 2 C2: Jetson vision and ballistics re-review (w/jetson-vision 568939e, w/jetson 79e30ed)

Reviewed at base-v1 commit 568939e. Method: independent finder subagents, one per lens; every finding of
medium severity or above was checked by two skeptic subagents told to refute it; kept only if fewer than
two refuted it. Low findings are listed unverified. Severity is the finder's, lowered when both skeptics
agreed it is real but rated it lower.

Confirmed: 4. Refuted and dropped: 1.

Re-checked 14:10 Sydney against base-v1 863d58d, one checker subagent per finding: see the Now column.

| # | Sev | File:line | Owner | Finding | Now (863d58d) |
|---|---|---|---|---|---|
| 1 | high | `jetson/grappstar/cli.py:301` | jetson | R1-C #3 is still broken in a real run: `serve --drag` defaults to 0.75 and settings.example.json says 0.75, so every served mission solves on the 0.75 N column, not the LIGHT 0.5 N clutch | partly fixed |
| 2 | medium | `jetson/grappstar/mission.py:877` | jetson | tell_line_out sends the video-only flight range to the base as line_m, so the capstan reel-in can pull the dart onto the capstan | still open |
| 3 | medium | `jetson/grappstar/video/camera.py:335` | jetson-vision | A long operator prompt still loses the launch frames: once the hold is over HELD_MAX_MB it drops the OLDEST packed frames (the launch first), and HOLD_MAX_S ends the hold without warning (R1-C #4 only partly fixed) | still open |
| 4 | medium | `jetson/grappstar/vision/flight.py:689` | jetson-vision | Video-only flight reconstruction still gives a range 30 to 80 percent too long (round 1 #6 not fixed): DEFAULT_V0 is 10 m/s and v0 is bisected from a noisy flight time | fixed |

## 1. R1-C #3 is still broken in a real run: `serve --drag` defaults to 0.75 and settings.example.json says 0.75, so every served mission solves on the 0.75 N column, not the LIGHT 0.5 N clutch

- Now (863d58d): **partly fixed** at `jetson/grappstar/cli.py:301`. Ballistics.at_pitch and solve now default to LIGHT_DRAG_N = 0.5 (ballistics.py:31, 93, 111), and MissionConfig still uses 0.5. The serve path is unchanged, though: `serve --drag` still defaults to 0.75 (cli.py:301), cmd_serve still always sets cfg.mission.drag_n = a.drag (cli.py:46), `solve --drag` still defaults to 0.75 (cli.py:354), and settings.example.json:11 still has "drag": 0.75. So every served mission still solves on the 0.75 N column, as the finding describes.

- File: `jetson/grappstar/cli.py:301`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: Normal start: `python -m grappstar serve` (or the boot service, using settings.json copied from settings.example.json as the README says). cmd_serve sets cfg.mission.drag_n = a.drag = 0.75, which overrides the MissionConfig default of 0.5. I proved this by running main(['serve','--sim']) with App stubbed: mission drag_n = 0.75, while MissionConfig().drag_n = 0.5. With the real base/out/ballistics.json the launch clutch is LIGHT 0.5 N (D24, params REEL clutch LIGHT_line_n=0.5). Effects: (a) every target from 4.40 to 4.85 m is rejected as 'too far: reachable 2.6 to 4.4 m at drag 0.75 N'. At 0.5 N those targets are reachable: solve(4.4,0.5) gives 24.9 deg, and solve(4.75,0.5) gives 45.17 deg on the high branch. The longest band, where the D29 high branch matters, cannot be used at all. (b) Shorter targets land long: solve(3.0,0.75) gives 9.85 deg, and at_pitch(9.85,0.5) is 3.11 m. 3.5 m gives 15.66 deg and lands at 3.68 m. 4.0 m gives 22.96 deg and lands at 4.27 m, which is 0.27 m long. The solve subcommand (cli.py:354) also defaults to 0.75.

Evidence: Seen on the w/jetson branch.
- jetson/grappstar/cli.py:46 `cfg.mission.drag_n = a.drag`
- cli.py:301 `s.add_argument("--drag", type=float, default=0.75, help="payout brake setting, N (C3 column)")`
- cli.py:354 `s.add_argument("--drag", type=float, default=0.75)`
- jetson/settings.example.json:11 `"drag": 0.75,`
- jetson/grappstar/mission.py:75 `drag_n: float = 0.5  # ... LIGHT 0.5 N (coordinator)`, which cmd_serve overrides.
- ballistics.py:88 and :106 still default `drag_n: float = 0.75`.
- params.py:216 `clutch=dict(LIGHT_line_n=0.5, ...)`
- Real table drag 0.75 column max is 4.384 m at 36 deg, while the 0.5 column reaches 4.847 m at 38.5 deg.
The w/jetson-vision branch has the same cli.py:297/350 and settings.example.json:11.

Fix: Set the default to 0.5 at cli.py:301 (serve --drag) and cli.py:354 (solve --drag), and set settings.example.json:11 to "drag": 0.5. Also change the drag_n defaults of Ballistics.at_pitch and solve (ballistics.py:88, 106) to 0.5; w/jetson-vision already does this as LIGHT_DRAG_N, so take its version when merging. A cleaner option is to use default=None for serve --drag and only override cfg.mission.drag_n when it is given. Add a test that main(['serve','--sim']) with App stubbed gives cfg.mission.drag_n == 0.5.

> Skeptic (confirmed): I checked this against origin/w/jetson and the claim holds.

- **The default path forces 0.75 N.** In jetson/grappstar/cli.py, cmd_serve (line 46) always runs `cfg.mission.drag_n = a.drag`. The serve parser (line 301) sets `--drag` to default=0.75, so every serve run replaces MissionConfig.drag_n = 0.5 (mission.py:75) with 0.75.
- **The settings file does not fix it.** The settings.json path (set_defaults from jetson/settings.json) only changes the default. settings.example.json:11 also says `"drag": 0.75`, so it keeps 0.75 too.
- **solve has the same default.** The solve subcommand (line 354)
>
> Skeptic (confirmed): I checked this on origin/w/jetson and the defect is real.

What the code does:
- cli.py:301 sets the serve `--drag` default to 0.75.
- cmd_serve at cli.py:46 always assigns `cfg.mission.drag_n = a.drag`. That overrides MissionConfig.drag_n = 0.5 (mission.py:75).
- settings.example.json:11 has `"drag": 0.75`. main() at cli.py:365-369 loads settings.json with set_defaults, so the boot service following the README copy step also gets 0.75.
- The mission solves with `self.bal.solve(rng, self.cfg.drag_n)` (mission.py:563), so every served mission uses the 0.75 column.
- The solve subcommand at cli.
>

## 2. tell_line_out sends the video-only flight range to the base as line_m, so the capstan reel-in can pull the dart onto the capstan

- Now (863d58d): **still open** at `jetson/grappstar/mission.py:974`. tell_line_out (mission.py:968-987) has not changed. It still uses any flight range_m between 0.5 and 30 m, with no check on fl["source"] and no cap against sol["line_out_m"], so a range from video alone still wins over the ballistic line-out and goes to the base as line_m. vision/flight.py did add fixes for the video-only range (a commanded-pitch prior and calibration to the C3 table), but mission.py:945 calls self.analyse without aim_pitch_deg, so the video-only path still estimates pitch from the horizon or the flight time, and nothing stops an overestimated range from making the capstan over-reel.

- File: `jetson/grappstar/mission.py:877`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: Worktree /tmp/claude-0/review-wt/jetson. The dart log is unavailable after landing, so run_post_flight gets flightlog=None and FlightResult.source == 'video'. Take a 40 deg shot with true range about 4.8 to 5.9 m: the flight analysis reports about 8.5 m (previous finding). tell_line_out sets line_m = 8.5*1.05+0.3 = 9.2 m against a real 5.0 to 6.0 m. In retrieve() (BASE_SPEC 10.1 step 15) the capstan reels until line_m <= approach_m 1.5 m, which is 7.7 m of reeling against the 3.5 to 4.5 m actually needed. The dart is pulled all the way into the cradle on the capstan with wraps on. BASE_SPEC step 17 says 'Never seat on the capstan'. The code only notices this afterwards, through sw S at mission.py:1102-1105 ('dart reached the cradle on the capstan (line_m estimate too long)'). The ballistic solution's line_out_m, which is accurate, is never used, because the flight range always wins.

Evidence: mission.py:877-879 `if isinstance(fl.get("range_m"), (int, float)) and 0.5 < fl["range_m"] < 30:` / `est = fl["range_m"] * 1.05 + 0.3` / `src = "flight analysis"`. There is no check of fl["source"].
mission.py:880-881 `elif sol.get("line_out_m"): est, src = sol["line_out_m"], "ballistics"`.
mission.py:1087-1089: retrieve uses line_m or line_est_m.
mission.py:1104 `self.log.warn("dart reached the cradle on the capstan (line_m estimate too long)")`.
BASE_SPEC.md:187-189 (step 15: reel and stop at line_m about 1.5; step 17: 'Never seat on the capstan').

Fix: In tell_line_out, use the flight range only when it comes from the IMU: `if fl.get("source") in ("imu", "video+imu") and isinstance(fl.get("range_m"), (int, float)) and 0.5 < fl["range_m"] < 30:`. Otherwise use sol["line_out_m"]. Also, when both exist, cap est at sol["line_out_m"] * 1.3, so that a bad vision range can never send the capstan far past the real line out.

> Skeptic (confirmed): I could not refute this. I checked it in the worktree at /tmp/claude-0/review-wt/jetson/jetson/grappstar/mission.py.

1. tell_line_out (lines 871-890) uses fl["range_m"] whenever 0.5 < range < 30. It never looks at fl["source"]. sol["line_out_m"] is only an elif fallback, and nothing compares the two values.

2. The video-only range really does run long. I ran the earlier reviewer's script, /tmp/claude-0/review-scratch-C2/vid_range.py, against the worktree code with PYTHONPATH set to the worktree; it renders 40 deg sim flights and calls analyze_flight(frames, None). With true ranges of 5.02 an
>
> Skeptic (confirmed): I tried to refute this and could not. The defect is real.

1. The code matches the claim. In the worktree (/tmp/claude-0/review-wt/jetson/jetson/grappstar/mission.py:871-890), tell_line_out takes fl["range_m"] whenever it falls between 0.5 and 30. It never checks fl["source"]. The ballistic sol["line_out_m"] is only an elif fallback.

2. The video-only path happens and gives the wrong range. analysis.run_post_flight calls flight.analyze_flight(frames, flightlog). With flightlog=None, or with no clock offset or no IMU launch, use_imu is False and source is set to "video" (vision/flight.py:639-6
>

## 3. A long operator prompt still loses the launch frames: once the hold is over HELD_MAX_MB it drops the OLDEST packed frames (the launch first), and HOLD_MAX_S ends the hold without warning (R1-C #4 only partly fixed)

- Now (863d58d): **still open** at `jetson/grappstar/video/camera.py:335`. The hold still has no end time: _held_locked is `f.t >= self._hold_t`. Step 3 still calls _pop_locked, which removes ring[0], the oldest packed frame, so the launch is the first frame lost once the store passes HELD_MAX_MB. Lines 309-310 still end the hold silently after hold_max_s. Meanwhile mission.py:874 still waits on the 'Is the dart down?' ask with no timeout, and the docstring claim at lines 20-22 is unchanged.

- File: `jetson/grappstar/video/camera.py:335`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: jetson-vision

Scenario: Real 720x480 BGR analog capture at 30 fps with normal composite noise. q92 JPEG is about 100 KB per frame. hold(t_fire-2) is set at fire time. Nothing detects the landing, so after land_timeout_s=10 the mission calls ask('Landing not detected ... Is the dart down?') (mission.py:787), which has no timeout. The hold also pins every frame AFTER the landing, so the packed store grows at 30 frames/s with no end. It reaches HELD_MAX_MB=400 after roughly 2.5 min, and from then on step 3 pops ring[0], the oldest packed frame, which is the launch. I reproduced it with /tmp/claude-0/review-scratch-C2/longhold.py, which drives Camera._handle with 720x480 noisy frames, buffer_s=15 and the automatic cap. 74 s and 134 s after the fire: held[0].t - t_fire = -2.00 and dropped_held=0. 194 s after the fire: held[0].t - t_fire = +49.8 s and dropped_held=1554. Every frame from the launch to 50 s after it is gone. analyze_flight then finds no launch (no trajectory, range or clock offset), which is the R1-C #4 failure again. Separately, at 300 s (HOLD_MAX_S) the hold just expires (lines 309-310) and the next trim drops all held frames by age. The docstring claim at lines 21-22 ('an operator prompt with no timeout cannot lose the launch frames') is false. The new test only covers a 30 s prompt (test_hold_keeps_launch_frames_through_a_long_landing_wait) and never reaches HELD_MAX_MB. So a mutation that pops the oldest packed frame (the current code) and one that pops the newest both pass.

Evidence: camera.py:42 `HELD_MAX_MB = 400.0       # compressed held frames above this are dropped (about 3 min of 720x480 at 30 fps)`; camera.py:43 `HOLD_MAX_S = 300.0`; camera.py:309-310 `if self._hold_t is not None and now - self._hold_set_at > self.hold_max_s: self._hold_t = None`; camera.py:334-337 `# 3. last resort: drop the oldest packed frames` / `while self._packed_bytes > HELD_MAX_MB * 1024 * 1024 and self._n_packed and len(ring) > 1: self._pop_locked(); self._dropped_held += 1` (_pop_locked pops ring[0], line 299); camera.py:296 `_held_locked` returns `f.t >= self._hold_t` with no upper bound. Scratch output: `t since fire 194 s held_first-t_fire=49.80 dropped_held=1554 packed=4168`.

Fix: Bound the hold to the flight. hold(t_from, span_s=HOLD_SPAN_S) with HOLD_SPAN_S = 30.0 stores self._hold_end = t_from + span_s, and _held_locked returns `self._hold_t <= f.t < self._hold_end`. Frames after the span follow the normal buffer_s window, so the landing frames stay in the live ring, and the packed store stays at about 30 s x 30 fps x 100 KB = 90 MB. Remove the forced expiry at lines 309-310 for a span-bounded hold, or make it log or raise a status error instead of silently dropping. If step 3 ever runs, drop the NEWEST packed frame, which is ring[self._n_packed-1] (`del ring[i]`, then adjust _ring_bytes, _packed_bytes and _n_packed), never ring[0]. Add a test that feeds 200 s after hold() and asserts held()[0].t == t_fire-2 and dropped_held == 0.

> Skeptic (confirmed): The claim holds, with one correction about where the code lives. The w/review / base-v1 camera.py has no hold at all. The cited code is on origin/w/jetson-vision (commit 2163415, "flight hold packs frames instead of dropping them (R1-C 4)"). origin/base-v1 mission.py:815-816 already calls c.hold(time.time() - pre_s) at fire time, guarded by hasattr, and only calls release_hold() at :946, after analyse() in landing_zone. So once the branches merge, the path is live.

Checked against the jetson-vision camera.py:
(1) _held_locked is `f.t >= self._hold_t` with no upper bound, so every frame after 
>
> Skeptic (confirmed): I checked this against the code that actually runs. The hold and pack logic lives in origin/w/jetson-vision jetson/grappstar/video/camera.py, which I diffed and found identical to the reviewer's worktree copy. The caller is origin/base-v1 jetson/grappstar/mission.py. The current w/review working tree contains neither, but the merged pipeline does.

The code path is real:
- fire() calls c.hold(time.time() - pre_s) inside fire_lock.
- When no landing source is found, wait_landing calls ask('Landing not detected ...'). That ask has timeout=None, so it blocks until the operator answers (mission.py
>

## 4. Video-only flight reconstruction still gives a range 30 to 80 percent too long (round 1 #6 not fixed): DEFAULT_V0 is 10 m/s and v0 is bisected from a noisy flight time

- Now (863d58d): **fixed** at `jetson/grappstar/vision/flight.py:810-856`. The no-IMU path no longer bisects v0 over 2 to 30 m/s. It now fuses the speed from the flight time, the sky track and the C3 table prior, then scales the range to the C3 table. analysis.py:111 passes the ballistics table and the aim pitch. I re-ran the reviewer's short.py on HEAD with a bare analyze_flight(frames) and no table: errors are now at most about 15% (for example 4.34 m against a true 3.77 m), down from 30 to 80%. The test_flight real_speed tests, which use the table and a 15% tolerance, pass. One thing is left: DEFAULT_V0 is still 10.0 at line 62, but it is now only a last-resort fallback.

- File: `jetson/grappstar/vision/flight.py:689`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: jetson-vision

Scenario: Worktree /tmp/claude-0/review-wt/jetson-vision, flight.py unchanged since 0bfd59a. Script /tmp/claude-0/review-scratch-C2/short.py runs render_flight(seed, (360,240), pitch_deg, v0=7.65), which is the real launcher speed, then analyze_flight(frames) with no flightlog. pitch 10: true 3.77 m, estimates 4.91 to 6.83 m (v0 solved 9.2 to 11.5 m/s). pitch 20: true 4.88 m, estimates 6.10 to 6.52 m. pitch 40: true 5.86 m, estimates 7.93 to 8.97 m. At pitch 40 no rail horizon pitch is found, so v0 falls back to DEFAULT_V0 = 10 and the pitch is solved from the flight time. Re-running with analyze_flight(frames, v0_default=7.65) gives 5.84 and 5.83 m against 5.86 m (short2.py). With the flightlog the error stays within about 5 percent. So any shot where the dart log is missing (dart Wi-Fi down after landing) reports a range 1.2 to 3 m too long, and the jetson mission passes it to the base as line_m (see the next finding).

Evidence: flight.py:62 `DEFAULT_V0 = 10.0`, while params.py:189 LAUNCH.flight says 'physics v1.2: 12.1 J, 7.41 m/s, 4.8 m at 0.5 N' and C3 v_muzzle is about 7.6 m/s.
flight.py:688-689 `if pitch is not None and v0 is None:` / `v0 = _bisect(lambda v: tf(v, pitch), 2.0, 30.0, T)` lets v0 range over 2 to 30 m/s from a video impact time with tens of ms of error.
flight.py:699-700 `elif pitch is None:` / `v0 = v0 or v0_default`.
There is no diff in jetson/grappstar/vision between 0bfd59a and HEAD on either worktree.

Fix: In flight.py set DEFAULT_V0 to the launcher's muzzle speed (7.4, or read it from Ballistics.load() / params LAUNCH.flight). On the no-IMU path (imu is None or no imu v0 and not saturated), stop bisecting v0 over 2..30. Use v0_default, or clamp the bisected v0 to the C3 v_muzzle band (about 7.0 to 8.0 m/s). Better: take res.range_m from the C3 table at the measured pitch and drag (Ballistics.at_pitch), because the drag-free ballistic() also ignores the 0.5 N tether drag. Add test_flight cases with render_flight(pitch_deg in (10, 20, 40), v0=7.65) and no flightlog, with a range tolerance of 15 percent or less.

> Skeptic (confirmed): I reproduced the claim. In /tmp/claude-0/review-wt/jetson-vision, flight.py:62 has DEFAULT_V0 = 10.0. With no flightlog, imu is None and res.v0 is None, which leaves two paths in the trajectory block:
- If a rail horizon pitch is found, v0 is bisected over 2 to 30 m/s from the video flight time (lines 688-689), with no clamp to the launcher's speed.
- If no pitch is found, v0 = v0_default = 10 (line 700) and the pitch is solved from the flight time.

The range then comes from the drag-free ballistic(). I ran /tmp/claude-0/review-scratch-C2/short.py from a worktree jetson dir. It renders frames
>
> Skeptic (confirmed): I reproduced the defect and could not refute it. In worktree /tmp/claude-0/review-wt/jetson-vision (HEAD 568939e), running /tmp/claude-0/review-scratch-C2/short.py from jetson/ gives these video-only results (no flightlog):
- pitch 10: estimates 4.91 to 6.83 m against 3.77 m true (v0 solved 9.2 to 11.5)
- pitch 20: estimates 6.10 to 6.52 m against 4.88 m true
- pitch 40: estimates 7.93 to 8.97 m against 5.86 m true (v0 stays at 10.0, the default)
With the flightlog the error is within about 7 percent (worst case 4.02 m against 3.77 m at pitch 10).

The code says what the claim says:
- flight.p
>

## Refuted (dropped)

- `jetson/grappstar/mission.py:809` landing_zone decodes the whole held span into raw BGR at once: after a long landing prompt, c.buffer() builds about 4 GB of frames on the 8 GB Orin Nano
