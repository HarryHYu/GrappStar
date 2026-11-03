# Round 2 F2: physics re-review (w/physics e347790)

Reviewed at base-v1 commit e347790. Method: independent finder subagents, one per lens; every finding of
medium severity or above was checked by two skeptic subagents told to refute it; kept only if fewer than
two refuted it. Low findings are listed unverified. Severity is the finder's, lowered when both skeptics
agreed it is real but rated it lower.

Confirmed: 6. Refuted and dropped: 4.

| # | Sev | File:line | Owner | Finding |
|---|---|---|---|---|
| 1 | medium | `base/physics/capstan.py:322` | physics | Tow limit is only checked at the 50 N default: any allowed setting above about 75 N reaches the shear-pin torque together with the 6.5 A tow trip, and nothing flags it (r2_K #7, not fixed) |
| 2 | medium | `base/physics/drivetrain.py:58` | physics | Aim gear check still models a gear that was never built (m2 face 10, splitter only); the built m1.5 face 8 splitter/bevel and the m2 z20 worm wheel fail at the 5 A aim trip (r2_K #5, physics part, not fixed) |
| 3 | medium | `base/physics/recoil.py:109` | physics | D5 second spring pair is still reported as an 8 to 9 m option with no cock-torque check; with 4 springs every cock needs 5.83 N m, above the shear pin and the 6.5 A cock trip (r2_K #6, not fixed) |
| 4 | low | `base/physics/capstan.py:47` | physics | Physics hard-trip model uses 6.0 A for both reel-in and tow; the spec trips are reel_in 5.0 A and tow 6.5 A, so the trip line force and the trip gear and shaft checks in drivetrain.json/capstan.json disagree with the firmware |
| 5 | low | `base/physics/test_phys_spec.py:74` | physics | Section 11 launch numbers are still not pinned by any test (R1 #9 not fixed) and no longer match the model: 13 J / 7.5 m/s / 4.5 m at 35 deg in the spec vs 12.1 J / 7.04 (6.49 after the snatch) / 3.72 m |
| 6 | low | `base/physics/test_phys_spec.py:81` | physics | test_committed_ballistics_not_stale checks one cell (36 deg, middle drag) and never runs validate() on the committed C3 file: corrupted or out-of-order rows still pass |

## 1. Tow limit is only checked at the 50 N default: any allowed setting above about 75 N reaches the shear-pin torque together with the 6.5 A tow trip, and nothing flags it (r2_K #7, not fixed)

- File: `base/physics/capstan.py:322`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: physics

Scenario: The firmware tow limit is adjustable from 2 to 110 N (params REEL.tow_limit_n comment) and Harry sets 100 N for clay. By hand, at 100 N with FIRM take-up: F_eff = 100 - 5 + 15.35 = 110.35 N, so T_wiper = 0.02 x 110.35 / (0.44 x 0.85) = 5.90 N m. The model gives 4.57 N m at 75 N, 5.05 at 84 N and 6.44 at 110 N. The 6.5 A tow trip is 4.55 N m and the PLA shear pin is about 5 N m (D26), or about 4.57 N m from its geometry. So the pin shears at the same moment the trip fires, before the line reaches the limit. The coupler spins free while sprag #2 holds the load on the line, the tow stops short, and the pin has to be replaced by hand. capstan.run evaluates slips_at_tow only for the tow input (50 N), and drivetrain.run uses only p.capstan['tow_limit_n'].

Evidence: capstan.py:322 `tow_frac=t_tow / lim, slips_at_tow=t_tow > LIMITER_FRAC_MAX * lim)` (t_tow at the single 50 N input); params.py:224 `tow_limit_n=50.0,  # firmware default, adjustable 2..110`; params.py:226 `current_trips_a=dict(... tow=6.5 ...)`. My run: capstan.wiper_torque_for(F, load_params()) gives 50 N 3.228, 75 N 4.565, 84 N 5.046, 100 N 5.902, 110 N 6.437 N m; kt x (6.5 - 1.5) = 4.545 N m.

Fix: In capstan.run, add f_tow_max = line_from_wiper(min(LIMITER_FRAC_MAX x limiter_nm(p), kt x (I_trip_tow - i_free)), p). That is about 60 N with 0.8 x 5 N m. Return it in the limiter dict and append a warning whenever the allowed tow range maximum (110 N) is above f_tow_max. In FINDINGS, state the cap (about 60 N), so the coordinator and firmware can clamp REEL.tow_limit_n and the UI, or cad-drum can enlarge the pin.

> Skeptic (confirmed): I could not refute this. The gap is real, but the claim overstates what happens.

What I confirmed:
- In base/physics/capstan.py run() (lines 314-321), t_tow = wiper_torque_for(tow) is computed once, at the single `tow` input. That input defaults to c["tow_limit_n"] = 50 N. tow_frac and slips_at_tow (t_tow > LIMITER_FRAC_MAX*lim, where LIMITER_FRAC_MAX = 0.8) are judged only at that point.
- Neither warnings() nor fw_trip() looks at the top of the adjustable range.
- The range is real and reachable. params.py REEL.tow_limit_n is commented "adjustable 2..110". Firmware config.h sets limitMaxN =
>
> Skeptic (confirmed): I checked the code and the claim holds in substance, though the pin-shear part is less certain than stated.

1) The model only checks the default. capstan.run (capstan.py:277-321) sets tow = c["tow_limit_n"] (50 N) and works out tow_frac and slips_at_tow at that one value. drivetrain.run (line 239) also uses only p.capstan["tow_limit_n"]. Nothing evaluates the top of the allowed range.

2) The firmware really does allow 2..110 N. config.h:180 has limitMaxN = 110, and base_logic.h:794 and :904 clamp to that and nothing lower. The tow-mode trip is towTripA = 6.5 A (config.h:63), applied when unt
>

## 2. Aim gear check still models a gear that was never built (m2 face 10, splitter only); the built m1.5 face 8 splitter/bevel and the m2 z20 worm wheel fail at the 5 A aim trip (r2_K #5, physics part, not fixed)

- File: `base/physics/drivetrain.py:58`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: physics

Scenario: Drum in LAUNCH, wiper turning backward to aim, and the rail or crank jams. Wiper torque rises to the 5 A aim trip: kt 0.909 x (5 - 1.5) = 3.18 N m. The physics run reports aim gear_sf_trip 1.37 (pass) because it checks mesh(3.18, 0.5, m2, z30, face 10). For the geometry actually built, printed_gears.mesh gives sf 0.76 for the splitter (m1.5 z30 face 8, 1:1) and 0.62 for the bevel (m1.5 z30->z15 face 8). Worm wheel by hand: worm torque = 3.18/2 x 0.95 = 1.51 N m; axial F = 1.51 / (0.012 x (0.084 + 0.266)/(1 - 0.266 x 0.084)) = 352 N; Lewis on m2 z20 face 12 gives 352/(12 x 2 x 0.322) = 45.5 MPa against 25 MPa allowed, so sf 0.55. The teeth strip before the trip fires, and drivetrain.json tells the team the aim train is safe.

Evidence: drivetrain.py:58 `gear=dict(module=2.0, face=10.0, z_driver=30),   # splitter / bevel stage (m 2 needed: m 1.5 fails at the 5 A trip)`; base/cad/parts/launch_aim.py:46-52 `M_S = 1.5`, `FACE_S = 8.0`, `M_B, Z_B1, Z_B2, FACE_B = 1.5, 30, 15, 8.0`, `WHEEL_FACE = 12.0`. My run: drivetrain.run(load_params()) aim gear_sf_trip 1.367; gears.mesh(3.18,1.0,1.5,30,8).sf = 0.76; gears.mesh(3.18,0.5,1.5,30,8).sf = 0.62. drivetrain.aim (line 93 on) has no Lewis check on the worm wheel or the z18 idler.

Fix: In drivetrain.AIM_DEFAULTS, set the stages to the built geometry: splitter dict(module=1.5, face=8.0, z_driver=30, idler z18) and bevel dict(module=1.5, face=8.0, z_driver=30, ratio 0.5). In evaluate()/aim(), add a Lewis check on the worm wheel (m2, z20, face 12) loaded by the worm thrust F = (T_wiper x eta_bevel / bevel) / (r_w (tan lambda + mu_e)/(1 - mu_e tan lambda)) at the peak, at the 5 A trip and at the limiter, and flag any sf < 1. That way drivetrain.json reports the aim train failing at the trip until cad-launch adds an aim-branch limiter of about 1 N m or upsizes the gears.

> Skeptic (confirmed): I could not refute this. I checked each part against the code and reproduced the numbers.
(1) Line 58 of base/physics/drivetrain.py says what the claim quotes. AIM_DEFAULTS["gear"] = dict(module=2.0, face=10.0, z_driver=30), and the comment there reads "m 2 needed: m 1.5 fails at the 5 A trip". evaluate() (lines 178-186) runs Lewis only on that one stage, with gear_ratio 1/bevel = 0.5.
(2) base/cad/parts/launch_aim.py:46-52 builds something else: splitter M_S = 1.5 with z30/18/30 and FACE_S = 8, bevel 1.5 z30 to z15 with face 8, and the worm wheel m2 z20 with WHEEL_FACE = 12. The params file a
>
> Skeptic (confirmed): I could not refute this claim. It holds up against the code.
- The physics model and the CAD disagree. base/physics/drivetrain.py:58 has AIM_DEFAULTS gear=dict(module=2.0, face=10.0, z_driver=30), and evaluate() runs mesh_tr = gears.mesh(t_trip, gear_ratio=0.5, m2, z30, face 10) with that geometry. Its own comment says "m 1.5 fails at the 5 A trip". The geometry actually built in base/cad/parts/launch_aim.py:46-52 is M_S=1.5 and FACE_S=8 for the splitter (sun z30, idler z18, output z30), and M_B, Z_B1, Z_B2, FACE_B = 1.5, 30, 15, 8.0 for the bevel. The worm wheel is m2 z20 with WHEEL_FACE=12.

>

## 3. D5 second spring pair is still reported as an 8 to 9 m option with no cock-torque check; with 4 springs every cock needs 5.83 N m, above the shear pin and the 6.5 A cock trip (r2_K #6, not fixed)

- File: `base/physics/recoil.py:109`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: physics

Scenario: Harry fits the provisioned second pair of C-243 springs (params LAUNCH second_pair_provision=True) because physics advertises 4 springs: 26.2 J, 10.4 m/s, 7.7/6.9/6.2 m. By hand: carriage force at 265 mm = 4 x (7 + 0.125 x 265) = 160.5 N, plus 1.0 N rail friction = 161.5 N. Times big gear r 0.05 gives 8.07 N m. Divided by (40/26 x 0.9) that is 5.83 N m at the wiper. That is above the 6.5 A cock trip (0.909 x 5 = 4.55 N m) and above the 5 N m shear pin (D26). Every cock trips near the end of the stroke or shears the pin, so the launcher cannot fire. recoil.spring_count_case never calls drivetrain.cock, and no physics output flags it.

Evidence: recoil.py:109-116 spring_count_case only runs launch()/simulate() for n springs, with no cock torque; recoil.py:142 `out["springs"] = [spring_count_case(p, n, a.pitch) for n in (2, 4)]`; FINDINGS.md:11 `4 springs (provision): 9.8 m/s`, FINDINGS.md:155 `4 springs (second pair): 26.2 J, 10.4 m/s ... range 8.7 / 7.9 / 7.2 m`; params.py:182 `second_pair_provision=True, # D5`. My run: drivetrain.cock with n_springs=4 gives t_peak 5.832 N m, f_peak 161.5 N, i_peak 7.9 A, flag 'peak torque 5.83 N m is 1.17 of the 5 N m limiter'.

Fix: In recoil.spring_count_case, compute c = drivetrain.cock(q) and ev = drivetrain.evaluate(q, 'cock', c, q.battery.v_nom). Add t_cock_peak, i_cock_peak and feasible = c['t_peak'] <= min(LIMITER_FRAC_MAX x limiter, kt x (TRIP_COCK_A - i_free)) to the returned dict, and print them. When feasible is False, change FINDINGS lines 11, 17 and 155 to say the 4-spring option cannot cock with the current 26:40 drive and the 5 N m pin. It needs about a 16T drive gear or a stronger pin plus a raised cock trip, so the coordinator can drop or condition the D5 provision.

> Skeptic (confirmed): I could not refute this claim. I checked the code and ran it.

What the code does:
- recoil.py:109-116 `spring_count_case` only calls `launch()` and `simulate()` for n springs. It never calls `drivetrain.cock` or `evaluate`.
- recoil.py:142 runs it for n = 2 and n = 4, then prints energy, speed and range only.
- FINDINGS.md:11, 13 and 146 and DECISIONS D5 present 4 springs as an 8 to 9 m option with no condition attached.
- params.py:182 sets `second_pair_provision=True`.

What I ran: from base/physics I called `drivetrain.cock` on a deepcopy of the params with n_springs changed, then `drivetr
>
> Skeptic (confirmed): I could not refute this claim. I checked the code and ran it on a scratch copy, which has since been deleted; the working tree is unchanged.

- **The code does what the claim says.** recoil.py:109-116 (spring_count_case) sets q.n_springs = n and runs only launch() and simulate(). It never calls drivetrain.cock or evaluate. recoil.py:142 reports the cases for 2 and 4 springs.
- **The numbers reproduce.** drivetrain.cock reads p.n_springs at drivetrain.py:137. With n_springs=4 it gives f_peak 161.5 N and t_peak 5.832 N m. evaluate(...,'cock',...) gives i_peak 7.91 A and frac_limiter 1.17, and fl
>

## 4. Physics hard-trip model uses 6.0 A for both reel-in and tow; the spec trips are reel_in 5.0 A and tow 6.5 A, so the trip line force and the trip gear and shaft checks in drivetrain.json/capstan.json disagree with the firmware

- File: `base/physics/capstan.py:47`
- Severity: low (claimed low); unverified (low)
- Owner: physics

Scenario: Firmware uses params REEL.current_trips_a (tow 6.5 A, reel_in 5.0 A, D33). Physics never reads that block and hard-codes TRIP_REEL_A = 6.0 for both. capstan.fw_trip reports hard_line_n 67.3 N at 4.09 N m. At the real 6.5 A tow trip the wiper torque is 4.55 N m: by hand, line = 17.0 x 5.0 - 10.4 = 74.6 N (the D32 model). That is 7 N more line pull, and 0.45 N m closer to the shear pin, than physics publishes. drivetrain.evaluate also computes the tow and reel gear_sf_trip and shaft_torque_at_trip at 6.0 A. The tow trip is therefore under-reported, and the reel trip check is done at a current the firmware never allows.

Evidence: capstan.py:47 `TRIP_REEL_A = 6.0        # hard trip in REEL (reel-in and tow)`; drivetrain.py:184 `trip_a = {"cock": capstan.TRIP_COCK_A, "reel": capstan.TRIP_REEL_A, "tow": capstan.TRIP_REEL_A}.get(...)`; capstan.py:214 `t_hard = p.wiper.kt * (TRIP_REEL_A - p.wiper.i_free)`; params.py:226 `current_trips_a=dict(aim=5.0, reel_in=5.0, cock=6.5, tow=6.5, ...)`; spec_bridge.py has no mapping for REEL.current_trips_a. Run output fw_trip: hard_a 6.0, hard_line_n 67.28.

Fix: In spec_bridge.apply, read REEL.current_trips_a into p.trips (aim, reel_in, cock, tow). In drivetrain.evaluate, use trip_a = p.trips['tow'] for 'tow' and p.trips['reel_in'] for 'reel'. In capstan.fw_trip, use p.trips['tow'] for t_hard and hard_line_n, falling back to the module constants only when the spec has no such block. Regenerate drivetrain.json and capstan.json.


## 5. Section 11 launch numbers are still not pinned by any test (R1 #9 not fixed) and no longer match the model: 13 J / 7.5 m/s / 4.5 m at 35 deg in the spec vs 12.1 J / 7.04 (6.49 after the snatch) / 3.72 m

- File: `base/physics/test_phys_spec.py:74`
- Severity: low (claimed medium); 1 of 2 skeptics confirmed
- Owner: physics

Scenario: BASE_SPEC section 11 (owner physics) says 'about 13 J stored, about 7.5 m/s, about 4.5 m at 35 deg with the 0.5 N brake (4 to 5 m band)'. Spec v1.2.5 through the model gives energy_stored_j 12.08, v_muzzle 7.04 at 35 deg (effective 6.49 after the snatch), and range 3.72 m at 35 deg and 0.5 N, the same as base/out/ballistics.json. Anyone who plans targets from section 11 or params LAUNCH.flight (4.8 m, 7.41 m/s) expects 4 to 5 m, but the shot lands at about 3.7 m. The only launch check is test_headline_launch_band, 6.5 < v < 7.8 on raw launch(). There is no assertion on energy or range. Mutation it misses: carriage_mass 0.06 -> 0.12 kg (the CAD carriage F12 warns about) gives v 6.5 to 6.6 m/s and about 0.4 m less range, and the test still passes. Same for a 10 percent spring rate error.

Evidence: BASE_SPEC.md:199 `About 13 J stored, about 7.5 m/s, about 4.5 m at 35 deg with the 0.5 N brake (4 to 5 m band)`; test_phys_spec.py:72-74 `r = launch(p, 36.0)` / `assert not r["over_safe"]` / `assert 6.5 < r["v_muzzle"] < 7.8`; run with load_params(): energy_stored_j 12.084, v_muzzle 7.041, effective v 6.493, simulate(...,35,brake 0.5,reel_model) range 3.716; launch(p,36,carriage_mass=0.13) v 6.446.

Fix: Add test_section11_numbers(p) with energy_stored_j == approx(12.1, rel=0.02), effective_muzzle(p,35)['v'] == approx(6.49, rel=0.02), simulate(p, that v, 35, brake_n=0.5, reel_model=True)['range_m'] == approx(3.72, rel=0.03), drivetrain.cock(p)['t_peak'] == approx(2.93, rel=0.03) and the tow value above. Use a single constants table that BASE_SPEC section 11 quotes. Replace the 6.5..7.8 band with these pins. Update BASE_SPEC section 11 (and send the coordinator the LAUNCH.flight note / AIM solver 'min range about 2.9 m') to 12.1 J, 6.5 m/s at the muzzle after the snatch, about 3.7 m at 35 deg / 0.5 N, min about 2.3 m at 6 deg.

> Skeptic (confirmed): The local w/review HEAD (8b32e37) is not origin/base-v1 (7d18cdd), and the cited file base/physics/test_phys_spec.py is not on w/review. I extracted origin/base-v1 with git archive into /tmp/claude-0/review-scratch-F2/A and ran it there. The working tree is unchanged.

Part of the claim holds on base-v1, which uses params VERSION 1.2.7 (the claim says 1.2.5):
(1) BASE_SPEC section 11 (line 199) still says "about 13 J, about 7.5 m/s". The model gives energy_stored_j 12.08 and launch(p,35) v_muzzle 6.95. FINDINGS.md and the params flight note already say 12.1 J, so section 11 is stale prose.
(2)
>
> Skeptic (refuted): The claim was checked against the wrong code. The review branch (HEAD 8b32e37) has no base/physics/test_phys_spec.py, so I extracted origin/base-v1 (7d18cdd, params VERSION 1.2.7) to /tmp/claude-0/review-scratch-F2/b1 and ran the model there.

1. The claimed mismatch is gone on base-v1. Spec v1.2.7 (commit 59ea66c, D36) moved the line guide behind the rail, so there is no longer a snatch. Results:
   - effective_muzzle(p,35) returns v=6.947, snatch=False, slack_m=0.0 (the claim says 6.49 after the snatch).
   - simulate(p, 6.947, 35, brake_n=0.5, reel_model=True) gives range_m = 4.18 m (the cl
>

## 6. test_committed_ballistics_not_stale checks one cell (36 deg, middle drag) and never runs validate() on the committed C3 file: corrupted or out-of-order rows still pass

- File: `base/physics/test_phys_spec.py:81`
- Severity: low (claimed medium); 2 of 2 skeptics confirmed
- Owner: physics

Scenario: Contract C3 base/out/ballistics.json is what the Jetson interpolates. In a scratch copy of base/{physics,spec,out} I edited the committed table: every pitch except 36 had range_m multiplied by 1.25, at 36 deg range_m[0] was x1.25 and range_m[2] x0.8, every t_flight_s was doubled, and row 3 had its pitch set to 50 (out of order and past pitch_max). pytest test_phys_spec.py test_phys_ballistics.py: 11 passed. ballistics.validate() on the same file reports ['pitch outside [pitch_min, pitch_max]', 'pitches must increase']. A model change that only affects other pitches or drag settings (for example a low-pitch rail-friction term) also leaves the file stale without failing the test. R1 #8 is only partly fixed.

Evidence: test_phys_spec.py:81 `row = min(doc["table"], key=lambda r: abs(r["pitch_deg"] - 36.0))`; :86-87 only `rng = simulate(..., brake_n=doc["drag_n"][1], ...)` and `assert row["range_m"][1] == pytest.approx(rng, rel=0.01)`; validate() is not called on the committed doc anywhere in the tests (test_phys_ballistics.py validates only freshly built docs).

Fix: In test_committed_ballistics_not_stale add `assert validate(doc) == []` and `assert doc["pitch_min"] == p.flight["pitch_min"] and doc["pitch_max"] == p.flight["pitch_max"]`. Then compare every drag column at three pitches (pitch_min, 36, pitch_max): for each row and each b in doc['drag_n'], assert range_m[i] == approx(simulate(p, v, pitch, brake_n=b, reel_model=True)['range_m'], rel=0.01), and do the same for t_flight_s and apex_m.

> Skeptic (confirmed): Note: test_phys_spec.py is not in the w/review checkout. It exists only on origin/base-v1, so I read it there with git show. The cited lines match what the claim says. test_committed_ballistics_not_stale does three things: it checks that meta.params_hash equals params_hash(p), it picks the one row nearest 36 deg, and it compares that row's v_muzzle and range_m[1] (middle drag) against a fresh simulate(). Nothing else in the repo runs validate() on the committed base/out/ballistics.json. test_phys_ballistics.py only validates docs it builds itself. The Jetson tests do not check the base table. 
>
> Skeptic (confirmed): Note first: base/physics/test_phys_spec.py is not on the w/review branch, which is not actually a copy of origin/base-v1. I reviewed origin/base-v1 by extracting it with git archive into /tmp/claude-0/review-scratch-F2/{b,c}. The repo working tree is unchanged.

The cited lines match the code. In origin/base-v1 test_phys_spec.py:77-87, test_committed_ballistics_not_stale does three things: it compares meta.params_hash with params_hash(p), checks v_muzzle on the row nearest 36 deg, and checks range_m[1] (middle drag) on that same row against simulate(). Nothing calls ballistics.validate() on th
>

## Refuted (dropped)

- `base/physics/calib.py:45` calib.py fits v_scale against the taut-line launch() speed, but ballistics.py applies it to the post-snatch effective_muzzle() speed, so a calibrated table comes out about 0.44 m short
- `base/physics/flight.py:73` Reel spin-up is counted twice at low pitch: the snatch spins the reel to the full dart speed, then simulate() resets it to v*cos(eye angle)
- `base/physics/calib.py:45` calib.fit_v_scale fits against launch() (taut-line speed) but ballistics.build applies v_scale to effective_muzzle() (after the snatch), so a perfect calibration shortens the C3 table by about 13 percent; test_fit_v_scale_recovers_factor cannot see this
- `base/physics/test_phys_spec.py:48` test_tow_50n_absolute window 2.3..3.8 N m accepts a 25 percent tow torque error and both the model (3.23) and the stale section 11 claim (2.3)
