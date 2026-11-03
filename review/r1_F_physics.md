# Round 1 F: base physics models (physics)

Reviewed at base-v1 commit 66f963a. Method: independent finder subagents, one per lens; every finding of
medium severity or above was checked by two skeptic subagents told to refute it; kept only if fewer than
two refuted it. Low findings are listed unverified. Severity is the finder's, lowered when both skeptics
agreed it is real but rated it lower.

Confirmed: 9. Refuted and dropped: 0.

| # | Sev | File:line | Owner | Finding |
|---|---|---|---|---|
| 1 | high | `base/out/ballistics.json:1` | physics | Contract C3 ballistics.json is stale: built from params v0.1-stub (280 mm full cock, 245 mm stroke); ranges about 0.37 m too long at the best pitch |
| 2 | high | `base/physics/spec_bridge.py:38` | physics | Capstan ratio inverted: spec capstan_ratio (capstan turns per wiper turn, 2.27) is loaded into a field the code uses as wiper turns per capstan turn; tow torque, tow current trip, reel speed, drum/key/gear checks off by 5.2x |
| 3 | high | `base/physics/test_phys_capstan.py:69` | physics | Tow tests run only on Params() defaults, so they miss the inverted capstan ratio from the spec bridge: tow torque comes out 0.53 N m instead of about 2.3 N m |
| 4 | medium | `base/physics/flight.py:43` | physics | range_m is measured from a fixed muzzle point 0.10 m ahead of the eye, not from the rail-pivot ground point that D29 requires; muzzle height ignores pitch |
| 5 | medium | `base/physics/spec_bridge.py:19` | physics | Spring preload from spec v1.2 is never read: every launcher/cock model runs at 276.2 mm full cock instead of 265 mm (over the D30 cap) |
| 6 | medium | `base/physics/spec_bridge.py:19` | physics | spec_bridge drops the v1.2 spring preload, carriage mass and the whole storage-reel geometry, so the C3 table is about 0.4 m (8 percent) too long |
| 7 | medium | `base/physics/spec_bridge.py:19` | physics | No test loads the v1.2 spec, so the unmapped spring_preload key goes unnoticed: preload stays at 35 mm and the spring is drawn to 276 mm, past the 265 mm cap |
| 8 | medium | `base/physics/test_phys_ballistics.py:5` | physics | Committed ballistics.json (contract C3) is stale, built from params v0.1-stub, and no test compares it with the current model |
| 9 | low | `base/physics/test_phys_launch.py:45` | physics | No test pins the section 11 launch numbers (13 J, 7.5 m/s, 4.5 m at 35 deg): the launch tests only check the model against itself |

## 1. Contract C3 ballistics.json is stale: built from params v0.1-stub (280 mm full cock, 245 mm stroke); ranges about 0.37 m too long at the best pitch

- File: `base/out/ballistics.json:1`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: physics

Scenario: The Jetson interpolates base/out/ballistics.json to pick the pitch for a target distance. The file was generated with params v0.1-stub (ballistics_meta.json: params_source 'v0.1-stub (still defaults for LAUNCH, AIM, REEL, LAYOUT)', stroke_m 0.245), i.e. 35 mm preload + 245 mm = 280 mm full cock. Spec v1.2/D30 is 265 mm full cock (23.8 + 241.2). Rebuilt with the v1.2 geometry (preload 23.8 mm) using the repo's own launcher+flight models: 36 deg v 7.292 m/s, range 4.559/4.121/3.732 m vs the file's 7.609 m/s, 4.926/4.454/4.035 m; 6 deg 2.342 vs 2.470 m; 47 deg 4.423 vs 4.788 m. Aiming at a 4.9 m target the Jetson picks about 36 deg and the dart lands about 0.37 m short (about 8 percent across the table), wrong landing-zone verdicts. Simply rerunning ballistics.py today does not fix it because of the preload mapping defect (it would produce 276 mm, v 7.52 m/s at 35 deg).

Evidence: base/out/ballistics_meta.json: "params_source": "base/spec/params.py v0.1-stub (still defaults for LAUNCH, AIM, REEL, LAYOUT)", "stroke_m": 0.245
ballistics.json table row pitch 36.0: "v_muzzle": 7.609, "range_m": [4.926, 4.454, 4.035]
ballistics.py:42  v = launch(p, th, **(launch_over or {}))["v_muzzle"]
Scratch run: launch(p,36,preload_ext=0.0238) -> v 7.292, simulate -> [4.559, 4.121, 3.732]

Fix: After fixing the spring_preload mapping in spec_bridge.py, regenerate with `cd base/physics && python ballistics.py` so ballistics.json and ballistics_meta.json carry params_source 'base/spec/params.py v1.2' and stroke_m 0.2412; add a test that fails when ballistics_meta.json params_source differs from the current params.VERSION. Update BASE_SPEC section 11 / FINDINGS headline numbers (13 J, 7.5 m/s) to the regenerated values (about 12.1 J, 7.3 m/s, about 4.6 m at 0.5 N).

> Skeptic (confirmed): I could not refute it; the claim holds. base/out/ballistics_meta.json says params_source "v0.1-stub (still defaults for LAUNCH, AIM, REEL, LAYOUT)" and stroke_m 0.245. The outputs were last touched in commit a9a2fc1, before spec v1.2 (0bfd59a). v1.2 (D30) sets stroke 241.2 mm and spring_preload 23.8 mm, which gives 265 mm full cock (params.py lines 51 and 171-172). I reran the repo's own models (ballistics.load_params, launcher.launch, flight.simulate with reel_model=True) and got the reviewer's numbers exactly. With preload 0.0238: 6 deg v 7.414, range 2.342 m; 36 deg v 7.292, ranges [4.559, 
>
> Skeptic (confirmed): The claim holds up. base/out/ballistics_meta.json records params_source "v0.1-stub (still defaults for LAUNCH, AIM, REEL, LAYOUT)" and stroke_m 0.245. git log shows ballistics.json was last committed at 00:20 on 2026-11-03 (6a0f273). Spec v1.2 (0bfd59a, D30: 265 mm full cock, stroke 241.2, spring_preload 23.8) landed at 01:15, so the file was never regenerated. Contract C3 says the Jetson interpolates this file. jetson/grappstar/ballistics.py has no calibration or scale factor, so any error in the table goes straight into the pitch it picks. I ran the repo's own models from base/physics (in me
>

## 2. Capstan ratio inverted: spec capstan_ratio (capstan turns per wiper turn, 2.27) is loaded into a field the code uses as wiper turns per capstan turn; tow torque, tow current trip, reel speed, drum/key/gear checks off by 5.2x

- File: `base/physics/spec_bridge.py:38`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: physics

Scenario: params.REEL.capstan_ratio = 50/22 = 2.27 is 'capstan turns per main shaft turn' (REEL gear 50T drives capstan 22T, so the capstan torque is 0.44 x wiper torque and line speed 45 rpm x 2.27 x pi x 0.04 = 0.21 m/s, matching REEL.line_speed_mps.free). spec_bridge maps it unchanged into p.capstan['ratio'], which capstan.py/drivetrain.py treat as 'wiper turns per drum turn' (default 0.6 from the old 40T->24T train): wiper_torque_for = r*F/(k*eta), drum_shaft_torque = t*k*eta, rpm_drum = rpm/k. capstan.run(load_params()) therefore reports: tow 50 N needs 0.53 N m and 2.08 A (correct with k = 0.44: 2.75 N m, 4.52 A); tow current trip = free + 0.58 A (correct +2.98 A); light reel-in 0.0375 m/s, 533 s for 20 m (correct 0.19 m/s); limiter slip puts 482 N on the line and 9.66 N m on the drum, with false 'key fails' / 'shaft over Kt 2' / 'drum pressure' warnings (correct 93 N, 1.87 N m, all pass); '5 A hard trip still allows 307 N' (correct 59 N). A firmware tow trip derived from this run (0.58 A over free, about 88 N per A) would cut a normal 50 N tow at about 9 N, and the line-per-amp contradicts D32 (17 N/A). fw_trip/drivetrain also run the Lewis check on a 40T driver / 91T driven mesh with 2.27x torque instead of the real 50T/22T mesh.

Evidence: params.py:206  capstan_ratio=50.0 / 22.0,                # capstan turns per main shaft turn
spec_bridge.py:38  "capstan_ratio": ("capstan", "ratio", 1.0),
common.py:331  d=0.040, ratio=0.6, ... # ratio = wiper turns per capstan turn (REEL gear 40 T -> capstan 24 T)
capstan.py:120  return r * (max(f_line - th, 0.0) + takeup_drive_force(p, overspeed, clutch_r)) / (k * eta)
capstan.py:40  CAP_GEAR = dict(module=2.0, face=12.0, z_driver=40)
capstan.py:197  m = gears.mesh(t_hard, p.capstan["ratio"], g["module"], g["z_driver"], g["face"], PPA_CF, 0.0)
drivetrain.py:160  ... wiper_turns=drum_turns * c["ratio"], ... gear_ratio=c["ratio"], train_k=r / (c["ratio"] * c["eta"]))
Run: capstan.run(load_params()) -> inputs ratio 2.2727, at_tow t_wiper 0.532, current_limit delta_over_free 0.576; with p.capstan['ratio']=22/50 -> t_wiper 2.749, delta 2.976, limiter line 92.9 N

Fix: In spec_bridge.py map the inverse: handle capstan_ratio explicitly, e.g. `if isinstance(REEL.get('capstan_ratio'), (int,float)): p.capstan['ratio'] = 1.0 / REEL['capstan_ratio']` (wiper turns per capstan turn = 22/50 = 0.44) and remove the 1:1 LAUNCH-style entry. Also map REEL.capstan.d, and set capstan.CAP_GEAR to z_driver=50 (REEL output gear m2 50T, face 12) so mesh(t, 0.44, 2.0, 50, 12) checks the real 50T/22T pair. Add a test: with the spec loaded, line speed at free run is about 0.21 m/s and the 50 N tow needs about 2.7 N m.

> Skeptic (confirmed): I could not refute this. The defect is real and I reproduced it.

1. **The spec convention.** base/spec/params.py:206 sets `capstan_ratio=50.0/22.0` and comments it as "capstan turns per main shaft turn". DECISIONS D20 and BASE_SPEC lines 94 and 111 agree: the REEL output gear (m2, 50T) drives the capstan gear (m2, 22T), so the capstan turns 2.27x faster than the wiper. The spec's own numbers fit only that reading. With 45 rpm x 2.27 x pi x 0.04 you get 0.21 m/s, which is `line_speed_mps.free` and the speed D20 quotes. D3 states the same sense for the old train ("capstan 1.667 x wiper" for 40T
>
> Skeptic (confirmed): I could not refute this claim. The defect is real and I reproduced it.

1. **The spec value means the opposite of what the code expects.** params.py:206 sets REEL.capstan_ratio = 50/22 = 2.27, commented as "capstan turns per main shaft turn". BASE_SPEC.md:94/143 and DECISIONS D20 agree: the REEL gear 50T drives the capstan 22T, so the capstan turns 2.27 times per wiper turn and line speed is 0.21 m/s.

2. **The bridge copies it across unchanged.** spec_bridge.py:38 maps it 1:1 into p.capstan['ratio'], and _overlay at line 28 of apply() applies it. Running load_params() gives p.capstan['ratio']
>

## 3. Tow tests run only on Params() defaults, so they miss the inverted capstan ratio from the spec bridge: tow torque comes out 0.53 N m instead of about 2.3 N m

- File: `base/physics/test_phys_capstan.py:69`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: physics

Scenario: In params.py v1.2, REEL.capstan_ratio = 50/22 = 2.27 is defined as capstan turns per main-shaft turn. In the physics code, capstan['ratio'] means the opposite: wiper turns per capstan turn (common.py:331, default 0.6). spec_bridge copies the number across without inverting it. With load_params(), capstan.wiper_torque_for(50, p) = 0.532 N m (BASE_SPEC section 11 says about 2.3 N m) and tow_point(50) v_line = 0.036 m/s (REEL.line_speed_mps at_50n says 0.15). The tow current threshold, the drivetrain tow fractions and the shaft-at-stall numbers are all built on that torque. Every test builds Params() with the 0.6 default, so the suite passes. I copied test_tow_roundtrip_and_numbers and switched it to load_params(): it fails at line 81 with 'assert 0.05 < 0.0360071286842502'. Mutation that goes uncaught: any change to how spec_bridge maps capstan_ratio, including this inversion bug. The test also compares the model with its own formula (line 76: t == r*(50-th+1.2*tk)/(k*eta)), so no absolute 2.3 N m figure is checked anywhere.

Evidence: spec_bridge.py:38 "capstan_ratio": ("capstan", "ratio", 1.0),
params.py:206 capstan_ratio=50.0 / 22.0,  # capstan turns per main shaft turn
common.py:331 ratio=0.6, ... # ratio = wiper turns per capstan turn
test_phys_capstan.py:70-71 p = Params() ...
test_phys_capstan.py:81 assert 0.05 < pt["v_line"] < 0.2  (passes only because p is Params(); with load_params() v_line = 0.036)

Fix: In spec_bridge.apply, after the REEL overlay, set p.capstan['ratio'] = 1.0 / REEL['capstan_ratio'] (0.44) and remove the direct 1.0-scale mapping. Add a test that uses common.load_params() and asserts capstan.wiper_torque_for(50, p) == pytest.approx(2.3, abs=0.5) and 0.10 < tow_point(50, p, 11.1)['v_line'] < 0.2. With ratio 0.44 the model gives about 2.75 N m. Either tighten the take-up terms or update the 2.3 figure in FINDINGS and BASE_SPEC section 11 to match.

> Skeptic (confirmed): I checked the claim against the code and it holds. spec_bridge.py:38 maps REEL.capstan_ratio onto capstan['ratio'] at scale 1.0, and nothing later in apply() inverts it. Line 38 is the only ratio handling for the capstan in that file. params.py:206 sets capstan_ratio=50/22 and labels it "capstan turns per main shaft turn". BASE_SPEC also says "capstan 2.27 x wiper". common.py:331 defines ratio=0.6 as "wiper turns per capstan turn". capstan.wiper_torque_for divides by k*eta, so it uses the wiper-per-capstan meaning. The two definitions are inverses of each other.

I ran it from base/physics. Wi
>
> Skeptic (confirmed): I checked this by reading the code and running it, and I could not refute it.
- base/physics/spec_bridge.py:38 copies REEL.capstan_ratio into p.capstan['ratio'] at scale 1.0, and apply() changes nothing after that.
- base/spec/params.py:206 defines capstan_ratio = 50/22 as "capstan turns per main shaft turn".
- DECISIONS D15 says the main shaft "always turns with the wiper", and D20 replaces 40T/24T with a 50T REEL gear driving a 22T capstan. D20 also says the 2.27 ratio "gives 0.21 m/s line speed". So the capstan turns faster than the wiper.
- The physics model uses ratio the other way round:
>

## 4. range_m is measured from a fixed muzzle point 0.10 m ahead of the eye, not from the rail-pivot ground point that D29 requires; muzzle height ignores pitch

- File: `base/physics/flight.py:43`
- Severity: medium (claimed high); 2 of 2 skeptics confirmed
- Owner: physics

Scenario: D29 says range_m is measured from the ground point under the rail pivot, and the Jetson adds only the camera-to-pivot offset. flight.simulate puts the muzzle at a constant (x 0.10, z 0.35) for every pitch and reports range = x_land - muzzle_x. In the real geometry the muzzle sits at pivot + L_rail*(cos th, sin th). So every row is short by L_rail*cos(th) (D29 itself quotes about 0.15 m short), and that error changes with pitch. The muzzle height is also wrong by L_rail*(sin th - sin th_ref): about 0.2 m between 6 and 47 deg for a rail of about 0.3 m. The fixed geometry also puts the eye-to-muzzle line at 45 deg, so at 6 deg the reel starts at only cos(39 deg)=0.78 of line speed even though launcher.py already spun it up to full speed. My scratch run shows peak dart line pull of 1.56 N against 0.62 N when the line is aligned with the rail, which double counts the reel spin-up at low pitch. The ballistics docstring still says 'from the muzzle's ground point', which contradicts D29.

Evidence: flight.py:43 `muzzle = np.array([0.10, p.launch["muzzle_height"]])` (no pitch dependence). flight.py:184 `range_m=float(nodes[-1, 0] - muzzle[0])`. flight.py:52-53 v_pay0 = vd0 @ u_start (eye at (0, 0.25)). ballistics.py:9 '- range_m: level ground, from the muzzle's ground point'. DECISIONS.md:37 D29 'range_m is measured from the ground point under the rail pivot ... shots would land 0.15 m short'

Fix: Give flight.simulate pivot-based geometry. Take the pivot height and the pivot-to-muzzle length from params (LAYOUT/LAUNCH) and set muzzle = (L*cos th, z_pivot + L*sin th) with the pivot at x=0. Put the line exit at the rail pivot pulley so u_start lies along the rail (D14). Return range_m = x_land - 0.0 (pivot). In ballistics.build(), pass muzzle_xz for each pitch, fix the docstring and meta notes, then regenerate the table.

> Skeptic (confirmed): I could not refute the core claim. Checked:

(1) base/physics/flight.py:41-45 puts the eye at (0, reel_height=0.25). When muzzle_xz is None, the muzzle is the constant (0.10, launch.muzzle_height=0.35), with no pitch term. Line 184 reports range_m = nodes[-1,0] - muzzle[0], so range is measured from this fixed point, which has no defined relation to the rail pivot.

(2) The override exists (muzzle_xz) but nothing uses it. base/physics/ballistics.py:44 calls simulate(p, v, th, brake_n=b, reel_model=True) with no muzzle_xz, so every C3 row uses the fixed geometry. Its docstring (line 9) still sa
>
> Skeptic (confirmed): I could not refute the core claim. The code says what the reviewer cites:

- **Fixed muzzle point.** In base/physics/flight.py:41-45 the eye sits at (0, reel_height=0.25). When `muzzle_xz` is None, the muzzle is a constant (0.10, muzzle_height=0.35) with no dependence on pitch. Line 184 reports `range_m = x_land - muzzle[0]`.
- **The table uses that default.** `ballistics.build()` (ballistics.py:43) calls `simulate(p, v, th, brake_n=b, reel_model=True)` without `muzzle_xz`, so every row of the C3 table uses that geometry. The docstrings in ballistics.py:9 and flight.py:9 still say "from the mu
>

## 5. Spring preload from spec v1.2 is never read: every launcher/cock model runs at 276.2 mm full cock instead of 265 mm (over the D30 cap)

- File: `base/physics/spec_bridge.py:19`
- Severity: medium (claimed high); 2 of 2 skeptics confirmed
- Owner: physics

Scenario: load_params() with params.py v1.2: LAUNCH has spring_preload=23.8 and stroke=241.2, but LAUNCH_MAP only knows the key 'preload_ext', so p.launch['preload_ext'] stays at the common.py default 0.035 m. launcher.launch() then computes e_full = 0.035 + 0.2412 = 0.2762 m. Running `python launcher.py` prints ext_full_mm 276.200, over_safe True, energy 12.76 J, v 7.52 m/s, carriage force 83.05 N. With the spec preload (23.8 mm, 265 mm full cock) the same model gives 12.08 J, 7.30 m/s, 80.25 N. Every downstream result (ballistics, recoil, cock torque in drivetrain.cock, stop peak 422 N vs 401 N) is computed for a spring state that D30 forbids (9 percent over the Century 38.4 N rating), and the shot is overestimated by about 0.22 m/s, about 0.4 m of range at 36 deg.

Evidence: spec_bridge.py:19  "preload_ext": ("launch", "preload_ext", 1e-3),   (no entry for "spring_preload")
params.py:172  spring_preload=23.8,        # D30: extension at HOME = SPRING.max_ext 265 - stroke 241.2
common.py:307  preload_ext=0.035,     # m spring extension at HOME (spec D5: 35 mm, 280 mm at full cock ...)
launcher.py:54-55  e0 = L["preload_ext"]; e_full = e0 + S / r
Run: load_params().launch -> {'stroke': 0.2412, ..., 'preload_ext': 0.035, 'ext_cap': 0.265}; launcher.py -> ext_full_mm 276.200, over_safe True

Fix: Add to LAUNCH_MAP in spec_bridge.py: "spring_preload": ("launch", "preload_ext", 1e-3), (and optionally "carriage_mass_target_kg": ("launch", "carriage_mass", 1.0)). Then update common.py:307 default/comment to 0.0238 and add a test asserting launch(p)['ext_full_mm'] == SPRING.max_ext (265) and over_safe is False with the spec loaded.

> Skeptic (confirmed): I could not refute this. It holds up both in the code and when run.

**The key is never mapped.** In base/physics/spec_bridge.py, LAUNCH_MAP at lines 15-25 has an entry for "preload_ext" (line 19) and none for "spring_preload". The function `_overlay` only copies keys that are in the map and silently skips the rest. base/spec/params.py v1.2 LAUNCH sets `spring_preload=23.8` and `spring_ext_full=265.0` (lines 172-173), so neither value reaches the physics. `apply()` copies SPRING.max_ext only into `p.launch['ext_cap']` (lines 84-86). Its own comment says max_ext "only caps", so nothing derives 
>
> Skeptic (confirmed): The defect is real, and I reproduced it. I could not refute it.

1. In base/physics/spec_bridge.py, LAUNCH_MAP (lines 15-25) has an entry for the key "preload_ext" but none for "spring_preload". `_overlay` (lines 57-64) skips any key that is not in the map.
2. base/spec/params.py v1.2 LAUNCH sets `spring_preload=23.8` (line 172) and `spring_ext_full=265.0`. It has no `preload_ext` key.
3. Nothing else in `apply()` sets the preload. It sets `ext_cap` from SPRING.max_ext, and nothing more. `common.load_params()` just calls `spec_bridge.apply`. So `p.launch['preload_ext']` stays at the common.py:
>

## 6. spec_bridge drops the v1.2 spring preload, carriage mass and the whole storage-reel geometry, so the C3 table is about 0.4 m (8 percent) too long

- File: `base/physics/spec_bridge.py:19`
- Severity: medium (claimed high); 2 of 2 skeptics confirmed
- Owner: physics

Scenario: params.py v1.2 gives LAUNCH.spring_preload=23.8 (D30), LAUNCH.carriage_mass_target_kg=0.060, and REEL.storage={core_d 50, flange_d 72, width 22} (D24/D25: r_full 30.7 mm). LAUNCH_MAP only knows 'preload_ext'/'carriage_mass_kg', and _overlay reads only top-level scalars, so the nested REEL.storage dict is skipped. load_params() today returns preload_ext=0.035, carriage_mass=0.08, and reel core 0.040 / flange 0.064 / width 0.020 (r_full 27.4 mm). The model then simulates a 276.2 mm extension against the 265 mm cap: launch(p,36) returns over_safe=True and energy 12.76 J instead of about 12.08 J. ballistics.build() never checks over_safe. Hand check at 36 deg, 0.5 N: committed table 4.926 m (v0 7.609). With preload 23.8 and the spec reel (r_full 30.73 mm, I_full 3.14e-5 kg m2), v0 is 7.215 and range 4.545 m, so the Jetson overshoots by 0.38 m. At 47 deg it is 4.788 vs 4.446 m. Every row is too long by about 3 to 8 percent. meta.stroke_m reads 0.245 while the spec says 241.2, so the committed json was also built from the v0.1 stub.

Evidence: spec_bridge.py:19 "preload_ext": ("launch", "preload_ext", 1e-3), but params.py:172 has spring_preload=23.8. spec_bridge.py:20 "carriage_mass_kg" vs params.py:178 carriage_mass_target_kg=0.060. spec_bridge.py:31-35 REEL_MAP core_d/flange_d/width at top level vs params.py:211 storage=dict(core_d=50.0, flange_d=72.0, width=22.0, ...). spec_bridge.py:61 `if k in mapping and isinstance(v, (int, float))` ignores nested dicts. Runtime: load_params().launch -> preload_ext 0.035, carriage_mass 0.08. reel core_d 0.04. launch(p,36) -> ext_full_mm 276.2, ext_cap_mm 265.0, over_safe True. ballistics_meta.json: "params_source": "base/spec/params.py v0.1-stub ...", "stroke_m": 0.245

Fix: In spec_bridge.py add LAUNCH_MAP entries "spring_preload": ("launch","preload_ext",1e-3) and "carriage_mass_target_kg": ("launch","carriage_mass",1.0). Overlay REEL["storage"] with REEL_MAP (core_d, flange_d, width), e.g. _overlay(p, mod.REEL.get("storage"), REEL_MAP, used), and map REEL.clutch.LIGHT_line_n to flight.brake_n. In ballistics.build(), raise when launch(...)["over_safe"] is true. Then regenerate base/out/ballistics.json and ballistics_meta.json from params v1.2.

> Skeptic (confirmed): The mechanism is real; the size the claim gives is too large. Checked against the code:
- spec_bridge.py LAUNCH_MAP has only 'preload_ext' and 'carriage_mass_kg'/'carriage_mass'. params.py v1.2 uses 'spring_preload' (23.8, D30) and 'carriage_mass_target_kg' (0.060), so neither is picked up.
- REEL_MAP expects core_d, flange_d and width at the top level of REEL. In v1.2 they sit inside REEL['storage'], and _overlay skips dicts, so they are never read.
- At runtime, load_params() reports source 'v1.2' (the stroke 0.2412 and ext_cap 0.265 are picked up) but still returns preload_ext 0.035, carria
>
> Skeptic (confirmed): The mechanism is real, but the claim overstates how big the error is.

What I confirmed in the code:
- base/physics/spec_bridge.py LAUNCH_MAP (lines 15-25) only recognises 'preload_ext', 'carriage_mass_kg' and 'carriage_mass'. params.py v1.2 LAUNCH instead uses 'spring_preload'=23.8 (D30) and 'carriage_mass_target_kg'=0.060, so neither value is applied.
- REEL_MAP expects core_d, flange_d and width at the top level of REEL. In params.py they are nested in REEL['storage']. _overlay (line 61) only reads int and float values, so the nested dict is skipped.
- REEL.clutch.LIGHT_line_n is not mapped
>

## 7. No test loads the v1.2 spec, so the unmapped spring_preload key goes unnoticed: preload stays at 35 mm and the spring is drawn to 276 mm, past the 265 mm cap

- File: `base/physics/spec_bridge.py:19`
- Severity: medium (claimed high); 2 of 2 skeptics confirmed
- Owner: physics

Scenario: params.py LAUNCH gives spring_preload=23.8 (D30: 265 - 241.2). LAUNCH_MAP only knows the key 'preload_ext', so preload_ext keeps the old 0.035 default. With load_params(), launch(p,35) reports ext_full_mm 276.2 > ext_cap_mm 265, over_safe=True, 12.76 J, 7.52 m/s and 4.80 m at 35 deg with 0.5 N. With the binding 23.8 mm preload the same model gives 12.08 J, 7.29 m/s, 4.56 m at 36 deg and a cocking peak of 2.94 N m. So the section 11 headline numbers and the cocking-current basis come from a spring overextended past D30. No test calls load_params(); grep finds no load_params or spec_bridge in any test_phys_*.py. test_flags_over_safe_extension uses Params() with explicit strokes and never checks the spec configuration.

Evidence: spec_bridge.py:19 "preload_ext": ("launch", "preload_ext", 1e-3),  (no 'spring_preload' entry)
params.py:172 spring_preload=23.8,  # D30: extension at HOME = SPRING.max_ext 265 - stroke 241.2
load_params().launch -> {'stroke': 0.2412, 'preload_ext': 0.035, ..., 'ext_cap': 0.265}; launch(p,35) -> 'ext_full_mm': 276.2, 'over_safe': True

Fix: Add "spring_preload": ("launch", "preload_ext", 1e-3) to LAUNCH_MAP. Add a test_phys_spec.py that uses load_params() and asserts p.launch['preload_ext'] == pytest.approx(0.0238), that launch(p,35)['over_safe'] is False, and that ext_full_mm <= 265 + 1e-6.

> Skeptic (confirmed): I checked this by running the code, and the claim holds. In base/physics/spec_bridge.py, LAUNCH_MAP maps only the key 'preload_ext' (line 19). base/spec/params.py v1.2 names the key 'spring_preload=23.8' (line 172, D30), and _overlay silently skips keys it does not know. So `load_params().launch` gives preload_ext=0.035, stroke=0.2412 and ext_cap=0.265 (output confirmed). Calling launcher.launch(p, 35) returns ext_full_mm=276.2 and over_safe=True, with a muzzle speed of 7.52 m/s.

No test_phys_*.py file calls load_params or imports spec_bridge, so nothing catches the mismatch. Every module tha
>
> Skeptic (confirmed): I checked the claim and it holds. In base/physics/spec_bridge.py, LAUNCH_MAP maps only "preload_ext" (line 19). base/spec/params.py LAUNCH uses the key spring_preload=23.8 (line 172). _overlay skips keys that are not in the map, so preload_ext keeps the Params default of 0.035 set at common.py:307. Nothing else in the repo reads spring_preload apart from params.py itself.

I ran the code: load_params().launch gives preload_ext 0.035 with stroke 0.2412 and ext_cap 0.265, and launch(p,35) returns ext_full_mm 276.2 with over_safe True. That breaks D30 (full cock at 265 mm; 280 mm was rejected bec
>

## 8. Committed ballistics.json (contract C3) is stale, built from params v0.1-stub, and no test compares it with the current model

- File: `base/physics/test_phys_ballistics.py:5`
- Severity: medium (claimed high); 2 of 2 skeptics confirmed
- Owner: physics

Scenario: ballistics_meta.json says params_source 'v0.1-stub (still defaults for LAUNCH, AIM, REEL, LAYOUT)' and stroke 0.245. The 36 deg row in base/out/ballistics.json reads v_muzzle 7.609 and range 4.926 m at 0.5 N. The current build(load_params()) gives 7.517 m/s and 4.82 m, and with the binding 23.8 mm preload 7.29 m/s and 4.56 m. The Jetson solver interpolates this table, so it under-estimates range at a given pitch by about 0.37 m (8 percent) and the dart lands roughly that far beyond the chosen target on a normal shot. test_contract_c3_shape builds a fresh doc with launch_over stroke=0.24 and pitch 30..40. test_validate_* use hand-made dicts. No test opens base/out/ballistics.json or checks that it matches the code and params.

Evidence: ballistics_meta.json:2 "params_source": "base/spec/params.py v0.1-stub (still defaults for LAUNCH, AIM, REEL, LAYOUT)"
ballistics.json row: {'pitch_deg': 36.0, 'v_muzzle': 7.609, 'range_m': [4.926, 4.454, 4.035], ...}
test_phys_ballistics.py:8 doc, meta = build(p, step=5.0, launch_over=dict(stroke=0.24))

Fix: Fix the spec_bridge issues above, then regenerate the table with python ballistics.py. Add a test that loads ../out/ballistics.json, asserts validate(doc) == [], checks the meta params_source starts with f"base/spec/params.py v{params.VERSION}" with no 'still defaults', and for 2 or 3 table pitches asserts v_muzzle and range_m match build(load_params()) within rel=0.01.

> Skeptic (confirmed): The defect is real, but the claim gets the direction of the error wrong and overstates how big it is.

What I confirmed:
1. base/out/ballistics_meta.json says params_source "base/spec/params.py v0.1-stub (still defaults for LAUNCH, AIM, REEL, LAYOUT)" and stroke_m 0.245. The current spec is params.py VERSION "1.2", and load_params() gives source "base/spec/params.py v1.2" with stroke 0.2412. The newest commit touching base/out/ is a9a2fc1 (physics: recoil). The spec has since moved to v1.1 (f75f2cc) and v1.2 (0bfd59a, which changes the springs and the 265 mm full cock, D30).
2. The committed 3
>
> Skeptic (confirmed): I checked this and the claim holds. The committed table is stale and no test catches it.

1. base/out/ballistics_meta.json says params_source "v0.1-stub (still defaults for LAUNCH, AIM, REEL, LAYOUT)" with stroke 0.245.
2. The 36 deg row in base/out/ballistics.json is v_muzzle 7.609 m/s and range_m [4.926, 4.454, 4.035].
3. With the current params, load_params() reports "base/spec/params.py v1.2" with stroke 0.2412. launch(p, 36) gives 7.517 m/s and simulate(..., brake_n=0.5, reel_model=True) gives 4.818 m. That matches the claimed 7.517 m/s and 4.82 m.
4. git log shows ballistics.json was las
>

## 9. No test pins the section 11 launch numbers (13 J, 7.5 m/s, 4.5 m at 35 deg): the launch tests only check the model against itself

- File: `base/physics/test_phys_launch.py:45`
- Severity: low (claimed medium); 2 of 2 skeptics confirmed
- Owner: physics

Scenario: test_lumped_and_chain_agree compares two of the model's own integrators (rel=0.03). test_energy_conservation_no_losses checks KE against the model's own stored energy with carriage mass 0. test_more_pitch_less_speed only checks ordering. No test asserts the absolute energy, muzzle speed or 35 deg range. Mutation I ran: in common.py:308, carriage_mass 0.080 -> 0.200. All 58 tests still pass, while load_params() at 35 deg drops from 7.52 m/s and 4.80 m to 6.53 m/s and 3.72 m, outside the 4 to 5 m band in BASE_SPEC section 11. The cocking peak check (test_phys_drivetrain.py test_cock_hand) also re-derives the model's own formula and never checks the 3 N m figure.

Evidence: test_phys_launch.py:49 assert a["v_muzzle"] == pytest.approx(b["v_muzzle"], rel=0.03)
common.py:308 carriage_mass=0.080,
mutation run: carriage_mass=0.200 -> '58 passed'; load_params() launch(p,35) v 6.53, range 3.72 m

Fix: Add a spec-anchored regression test using load_params() at 35 deg with 0.5 N: assert 11.5 <= energy_stored_j <= 13.5, 7.0 <= v_muzzle <= 8.0, 4.0 <= flight.simulate(p, v, 35, brake_n=0.5, reel_model=True)['range_m'] <= 5.0, and drivetrain.cock(p)['t_peak'] == pytest.approx(3.0, abs=0.4). These are the section 11 bands, so a change in mass, drag or spring that moves the published numbers fails the suite.

> Skeptic (confirmed): I checked the claim against the code and it holds. base/physics/test_phys_launch.py has no absolute assertion on stored energy, muzzle speed or range. test_lumped_and_chain_agree (line 49) only compares the model's two integrators with each other. test_energy_conservation_no_losses compares KE with the model's own stored energy, using carriage mass 0 and a near-massless spring. The pulley and pitch tests only check ratios and ordering. I searched the rest of the repo: test_phys_flight, test_phys_ballistics (which checks C3 shape and ordering only), base/tools/tests, base/cad/parts/test_launch.
>
> Skeptic (confirmed): I couldn't refute this. I checked it and reproduced the mutation.

(1) Every test in base/physics/test_phys_launch.py compares the model with itself:
- line 49 checks lumped against chain at rel=0.03.
- The energy test sets carriage mass to 0 and spring density to 1, then compares KE with the model's own stored energy.
- The pitch test only checks ordering.
- The other tests check pulley-ratio invariance or the over_safe flag.

(2) Nothing else pins the numbers either:
- test_phys_ballistics.py only checks the C3 shape and that range falls as drag rises.
- test_phys_flight.py runs at 9.0 and 8
>
