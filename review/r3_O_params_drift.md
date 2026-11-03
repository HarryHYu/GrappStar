# Round 3 O: params.py drift (modules that ignore or contradict the shared numbers)

Reviewed at base-v1 commit 472bb65. Method: independent finder subagents, one per lens; every finding of
medium severity or above was checked by two skeptic subagents told to refute it; kept only if fewer than
two refuted it. Low findings are listed unverified. Severity is the finder's, lowered when both skeptics
agreed it is real but rated it lower.

Confirmed: 13. Refuted and dropped: 0.

| # | Sev | File:line | Owner | Finding |
|---|---|---|---|---|
| 1 | high | `firmware/base/grappstar_base/params_gen.h:324` | fw-base | Committed params_gen.h is stale (v1.2.7): firmware builds with the old 5 A aim trip instead of the D40 3 A trip |
| 2 | high | `firmware/base/grappstar_base/params_gen.h:324` | fw-base | Firmware params_gen.h is stale at params 1.2.7: aim/backward-launch trip still 5.0 A, not the D40 3.0 A |
| 3 | medium | `base/cad/parts/drum.py:782` | cad-drum | cad-drum SW_INDEX block and lever sit inside the frame wiper-adapter hub (4830 mm3 solid overlap); the index switch cannot be fitted |
| 4 | medium | `base/cad/parts/reel.py:847` | cad-reel | reel.brake_mm parses the free-text DRUM zone 'clutch' strings; the pushrod stroke disagrees with cad-drum's brake track by up to 3.9 mm (FIRM at 8 deg, about 1/3 FIRM at the WRAPPED park) |
| 5 | medium | `base/out/ballistics_meta.json:3` | physics | Committed base/out/ballistics.json (contract C3) was built from params 1.2.7 (stroke 241.2); test_phys_spec fails on 1.2.8 |
| 6 | medium | `base/physics/drivetrain.py:64` | physics | drivetrain.py checks the cock drive gear with face 12 mm; params as built is face 7, which hides a Lewis failure at limiter stall |
| 7 | medium | `firmware/base/grappstar_base/config.h:76` | fw-base | One trip value (tripA) serves both aim (3 A) and capstan reel-in (5 A): params reel_in is never read |
| 8 | low | `base/cad/parts/_frame_kit.py:82` | cad-frame | cad-frame silently moves the camera to y -96 while params and jetson campose use y -50: the bearing correction is 46 mm off |
| 9 | low | `base/spec/BASE_SPEC.md:134` | cad-launch | BASE_SPEC section 8 launcher numbers contradict params.LAUNCH and D26/D30 |
| 10 | low | `base/spec/BASE_SPEC.md:162` | fw-base | BASE_SPEC section 10 says aim trips at 5 A, against params and D40 (3 A) |
| 11 | low | `base/spec/params.py:192` | physics | params.LAUNCH.flight still holds physics v1.2 range and speed (4.8 m, 7.41 m/s); the sim dart arc lands about 0.6 m beyond the physics range |
| 12 | low | `base/tools/render.py:279` | cad-infra | tools/render.ground_z looks up LAYOUT keys 'ground_z' and 'deck_z' that params does not have, so the viewer ignores deck_top_z |
| 13 | low | `jetson/grappstar/mission.py:1233` | jetson | Jetson final approach overrides the firmware time limit with a 0.08 m/s take-up speed; physics and firmware give 0.058 m/s at duty 0.3 |

## 1. Committed params_gen.h is stale (v1.2.7): firmware builds with the old 5 A aim trip instead of the D40 3 A trip

- File: `firmware/base/grappstar_base/params_gen.h:324`
- Severity: high (claimed critical); 2 of 2 skeptics confirmed
- Owner: fw-base

Scenario: Build the firmware as committed. config.h includes params_gen.h, which was generated from params v1.2.7, so P_REEL_CURRENT_TRIPS_A_AIM = 5.0 and Cfg::tripA = 5 A. Every backward run in LAUNCH (aim at base_logic.h:740/753, decock at :983) trips only at 5 A. D40 says the aim splitter gears have a Lewis safety factor of 0.62 at 5 A (they fail) and 1.44 at 3 A. So a jammed or overloaded aim crank strips the aim gears before the trip fires. I re-ran gen_params.py on params v1.2.8 into scratch and diffed it against the committed header: VERSION 1.2.7 vs 1.2.8, TRIPS_A_AIM 5.0 vs 3.0, BACKWARD_LAUNCH missing, LAUNCH_DRIVE_GEAR_FACE 16 vs 7, DRIVE_GEAR_Y 142.5..158.5 vs 147.5..154.5, STROKE 241.2 vs 240.9. Nothing in run_tests.sh / test_gen_params.py checks that the header is fresh.

Evidence: params_gen.h:5 `#define P_VERSION "1.2.7"`; params_gen.h:324 `#define P_REEL_CURRENT_TRIPS_A_AIM 5.0f`; params.py:13 `VERSION = "1.2.8"`; params.py:230 `current_trips_a=dict(aim=3.0, backward_launch=3.0, reel_in=5.0, ...)`; config.h:76 `float tripA = P_REEL_CURRENT_TRIPS_A_AIM;`; DECISIONS.md:48 D40 `every backward wiper run in LAUNCH (aim, decock, backward jog) trips at 3 A ... Lewis safety factor 0.62 in PPA-CF at the old 5 A trip`.

Fix: Run `python3 firmware/base/tools/gen_params.py` and commit the regenerated params_gen.h (P_VERSION 1.2.8, AIM trip 3.0). Add a freshness check to test/run_tests.sh: generate into a temp file and `diff` it against grappstar_base/params_gen.h, failing on any difference. Also change the config.h:62 fallback `#define P_REEL_CURRENT_TRIPS_A_AIM 5.0f` to 3.0f. Boards that already saved a ver-2 tuning record with trip_a = 5 still need a `set trip_a 3` + save (persist.h restores tripA at base_logic.h:124).

> Skeptic (confirmed): I could not refute this. The defect is real on origin/base-v1 (472bb65), the branch under review. One caveat: the local w/review HEAD (e7e3a65) has not picked up spec commit 4dc43fc. There, params.py still says VERSION 1.2.7 with aim=5.0, which matches the header. On origin/base-v1 itself:
- base/spec/params.py:13 is VERSION = "1.2.8".
- params.py:230 is current_trips_a=dict(aim=3.0, backward_launch=3.0, reel_in=5.0, ...), with the D40 note.
- The committed firmware/base/grappstar_base/params_gen.h still has P_VERSION "1.2.7" (line 5) and P_REEL_CURRENT_TRIPS_A_AIM 5.0f (line 324).
- config.h:
>
> Skeptic (confirmed): The claim holds against origin/base-v1, which is the branch under review. One caveat first: the local w/review checkout is not actually a copy of origin/base-v1. Its params.py is still 1.2.7 (aim=5.0) and it has no D40, so locally the header and params agree. On origin/base-v1, git show confirms every piece of the claim:
- params.py:13 VERSION = "1.2.8".
- params.py:230 current_trips_a has aim=3.0, backward_launch=3.0, reel_in=5.0.
- DECISIONS.md:48 D40 requires every backward wiper run in LAUNCH to trip at 3 A (Lewis safety factor 0.62 at 5 A).
- The committed firmware/base/grappstar_base/par
>

## 2. Firmware params_gen.h is stale at params 1.2.7: aim/backward-launch trip still 5.0 A, not the D40 3.0 A

- File: `firmware/base/grappstar_base/params_gen.h:324`
- Severity: high (claimed critical); 1 of 2 skeptics confirmed
- Owner: fw-base

Scenario: The rail jams or hits its stop during `aim` or `decock`, or during a backward jog in LAUNCH. config.h sets c.tripA from P_REEL_CURRENT_TRIPS_A_AIM, which is 5.0 A, and base_logic.h:740/753/983 run the aim and decock on that trip. The wiper keeps driving up to 5 A. D40 records that the m1.5 z30 face 8 splitter sun has a Lewis SF of 0.62 at 5 A, so the aim gears can strip before the trip fires. params.py 1.2.8 has aim=3.0 and backward_launch=3.0, but the header was never regenerated (it also still has stroke 241.2).

Evidence: params_gen.h:2 `// params.py VERSION: 1.2.7`; params_gen.h:324 `#define P_REEL_CURRENT_TRIPS_A_AIM 5.0f`; params_gen.h:251 `#define P_LAUNCH_STROKE 241.2f`; config.h:62 fallback `#define P_REEL_CURRENT_TRIPS_A_AIM 5.0f`; config.h:76 `float tripA = P_REEL_CURRENT_TRIPS_A_AIM;`. params.py:13 `VERSION = "1.2.8"`, params.py:230 `current_trips_a=dict(aim=3.0, backward_launch=3.0, ...)`, params.py:181 `stroke=240.9`.

Fix: Run `python firmware/base/tools/gen_params.py` and commit the regenerated params_gen.h (VERSION 1.2.8, AIM 3.0f). Change the config.h:62 fallback to 3.0f. Add a build or test check that P_VERSION equals params.VERSION. Since tripA is also used for reel-in (base_logic.h:876), give backward runs in LAUNCH their own constant from P_REEL_CURRENT_TRIPS_A_BACKWARD_LAUNCH so reel-in can stay at 5 A.

> Skeptic (confirmed): Checked against origin/base-v1 (472bb65), the branch under review. The local w/review HEAD (e7e3a65) has an older params.py (1.2.7, aim=5.0), so locally the header matches. On base-v1 itself the claim holds:
- base/spec/params.py has VERSION "1.2.8" and current_trips_a aim=3.0, backward_launch=3.0.
- DECISIONS.md D40 says every backward wiper run in LAUNCH trips at 3 A (Lewis SF 0.62 at 5 A, 1.44 at 3 A).
- firmware/base/grappstar_base/params_gen.h is still marked 1.2.7 and has P_REEL_CURRENT_TRIPS_A_AIM 5.0f (line 324), P_LAUNCH_STROKE 241.2f, and no BACKWARD_LAUNCH define at all.
- config.h:
>
> Skeptic (refuted): The claim's premise is false for this branch (w/review, at e7e3a65). base/spec/params.py:13 is `VERSION = "1.2.7"`, not 1.2.8. params.py:181 is `stroke=241.2`, not 240.9. params.py:230 is `current_trips_a=dict(aim=5.0, reel_in=5.0, cock=6.5, tow=6.5, blank_ms=50, hold_ms=30, note="D8/D33")`, so there is no aim=3.0 and no backward_launch key. The generated firmware/base/grappstar_base/params_gen.h (P_VERSION "1.2.7", P_LAUNCH_STROKE 241.2f, P_REEL_CURRENT_TRIPS_A_AIM 5.0f) matches params.py exactly, so the header is not stale. A repo-wide grep finds no "1.2.8", no "backward_launch", and no deci
>

## 3. cad-drum SW_INDEX block and lever sit inside the frame wiper-adapter hub (4830 mm3 solid overlap); the index switch cannot be fitted

- File: `base/cad/parts/drum.py:782`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-drum

Scenario: Ran Assembly.load(P, names=['drum','frame']) with S = {wiper_deg 0, drum_deg 0, mode LAUNCH}. drum_index_block (world bbox x 33.2..47.2, y 0.5..9, z -48..12) overlaps frame_motor_adapter (bbox x -52.5..52.5, y 0..24) by 4830 mm3, and drum_index_lever overlaps it by 251 mm3. No Allow covers the pair. cad-drum read DRUM.sw_index_mount ('2 x M3 in the wiper adapter plate front face at (x 43.1, z +6) and (x 43.1, z -36)') as if that face were at world y 0. cad-frame builds the adapter as wings at y 0..10 plus a hub of r 49 running y 0..24 into the cheek, so its face on the drum side is at y about 24 and the cheek is beyond it. The frame also never drills the two M3 holes (it has no 43.1 / sw_index reference). The index cam (idx_s 1..6, r up to 22.5) is enclosed by the adapter bore (r 33.5). The D12 no-magnet fallback encoder (SW_INDEX, GPIO 35) therefore cannot be built or mounted as modelled.

Evidence: drum.py:31 `idx_s=(1.0, 6.0), idx_r0=20.0, idx_bump=2.5, ...`; drum.py:779-783 `s0, s1 = 0.5, i1 + 3.0 ... for y in (-6.0, 36.0):  # params.DRUM.sw_index_mount: (x 43.1, z +6) and (x 43.1, z -36)`; _frame_motor.py:52 `HUB_R = 49.0`; _frame_motor.py:135-136 `tz = PLATE_T + standoff ... hub = prim.cyl(HUB_R, tz + SPIGOT_H, z=0.0)` (PLATE_T 10 + adapter_standoff 12 = y 0..24); params.py:161 sw_index_mount gives no y. Measured overlap: drum_index_block 4830.3 mm3, drum_index_lever 250.8 mm3 against frame_motor_adapter.

Fix: Give DRUM.sw_index_mount a world y on the drum side of the motor plate (cheek inner face at about y 32) and move the cam there. In drum.py set idx_s to about (33.5, 38.5), clear of the bus gear at 40, and set index_block s0 to the plate face. Have cad-frame drill the two M3 holes (or inserts) at the agreed (x, z) in that plate face. Then check that drum_index_block / drum_index_lever against frame_* parts show zero overlap.

> Skeptic (confirmed): Reproduced with tools/assembly.py Assembly.load(names=['drum','frame']). In both LAUNCH (wiper 0) and REEL (wiper 90), frame_motor_adapter has bbox x -52.5..52.5, y 0..24. drum_index_block (x 33.2..47.2, y 0.5..9, z -48..12) intersects it by 4830.3 mm3, and drum_index_lever by 250.8 mm3 (259.7 in REEL). _frame_motor.py:135-136 builds a solid hub of r 49 from y 0 to 24 (PLATE_T 10 + standoff 12 + spigot 2), bored only to r 33.5, so the block and lever sit inside the hub ring. drum.py:778-783 starts the block at s 0.5 and drills its M3 holes at (43.1, z +6/-36), treating the mount face as y 0. p
>
> Skeptic (confirmed): I tried to refute this and couldn't. I ran Assembly.load(P, ["drum","frame"]) from a scratch script at /tmp/claude-0/review-scratch-O/ov.py with S = {drum_deg 0, wiper_deg 0, mode LAUNCH}. drum_index_block overlaps frame_motor_adapter by 4830.3 mm3 and drum_index_lever overlaps it by 250.8 mm3. These are exactly the claimed figures. The block's bbox is x 33.2..47.2, y 0.5..9, z -48..12. Nothing in the repo grants an Allow for these pairs: the drum.py Allows at lines 1085-1098 and the frame.py Allows at 912-926 don't cover them, and drum_index appears in no other .py/.json file.

The code says 
>

## 4. reel.brake_mm parses the free-text DRUM zone 'clutch' strings; the pushrod stroke disagrees with cad-drum's brake track by up to 3.9 mm (FIRM at 8 deg, about 1/3 FIRM at the WRAPPED park)

- File: `base/cad/parts/reel.py:847`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-reel

Scenario: reel_push = brake_mm(P, S) is the reel pushrod joint value used by the assembly, sim and timeline. It only recognises clutch strings that equal 'FIRM' or 'LIGHT' or start with 'LIGHT to'. Every other params string falls through to the descending branch BRAKE_STROKE*(1-f) across the whole zone. Measured reel vs drum (K.brake_lift) in mm: drum 8.5 deg 3.92 vs 0.0; 33 deg 3.78 vs 1.0; 41 (NEUTRAL park, tidy) 2.0 vs 4.0; 312 deg 1.05 vs 4.0; 314 (WRAPPED park) 1.26 vs 4.0; 330 deg 2.95 vs 4.0. Results: just out of LAUNCH the model shows the storage clutch FIRM, where the spec says LIGHT. At NEUTRAL and at the WRAPPED park, where the tidy step needs FIRM, it shows it half or one-third applied. The pushrod also does not follow the drum track it rides on, so it floats or interpenetrates in the posed assembly.

Evidence: reel.py:847-854 `c = z.get("clutch", "LIGHT")
 if c == "FIRM": return BRAKE_STROKE
 if c == "LIGHT": return 0.0
 ... return BRAKE_STROKE * (f if c.startswith("LIGHT to") else 1.0 - f)`; params.py:133 `clutch="LIGHT (clutch ramps to FIRM over 30..40, after the collar shift)"`, :134 `clutch="FIRM from 40"`, :137 `clutch="LIGHT to FIRM 302..312"` (ramp read over the whole 302..340 zone); _drum_kin.py brake_ramps ((30,40,0,1),(50,62,1,0),(302,312,0,1)).

Fix: Stop parsing the zone text. In brake_mm use cad-drum's numeric profile: `from cad.parts import _drum_kin as DK; return BRAKE_STROKE * DK.brake_frac(dd, DK.cfg(P))`. Keep the zone and mode fallback only when _drum_kin cannot be imported. Alternatively, have the coordinator add numeric brake_ramps to params DRUM and have both modules read them.

> Skeptic (confirmed): The claim holds. In base/cad/parts/reel.py:839-856, brake_mm only treats a clutch string as FIRM or LIGHT when it matches exactly. Any other string falls into `BRAKE_STROKE * (f if c.startswith("LIGHT to") else 1.0 - f)`, with f taken across the whole zone. The params.py DRUM zones use free text that does not match:
- L_TO_N: 'LIGHT (clutch ramps to FIRM over 30..40, ...)' goes down the descending branch, so the rod reads almost FIRM at 8 deg.
- NEUTRAL: 'FIRM from 40' also descends, so the rod reads half applied at the 41 park.
- N_TO_R: 'LIGHT to FIRM 302..312' is ramped over all of 302..340
>
> Skeptic (confirmed): I could not refute this. I read the code, checked the params text and reproduced the numbers.

How the code goes wrong:
- brake_mm (base/cad/parts/reel.py:839-856) decides the stroke from the zone's free-text clutch string.
- L_TO_N has the clutch string "LIGHT (clutch ramps to FIRM over 30..40, ...)". That is not "LIGHT" and does not start with "LIGHT to", so the code takes the descending branch BRAKE_STROKE*(1-f) over the whole zone from 8 to 32 deg.
- NEUTRAL has "FIRM from 40". That is not exactly "FIRM", so it also descends, from 4 to 0 over 32 to 50 deg.
- N_TO_R has "LIGHT to FIRM 302..
>

## 5. Committed base/out/ballistics.json (contract C3) was built from params 1.2.7 (stroke 241.2); test_phys_spec fails on 1.2.8

- File: `base/out/ballistics_meta.json:3`
- Severity: medium (claimed medium); 1 of 2 skeptics confirmed
- Owner: physics

Scenario: On a normal checkout, `pytest base/physics` fails test_committed_ballistics_not_stale: the stored hash is cd6884897df6abd6 and the current hash is f011b9cab2d9b8c8. The params changes since that build were stroke 241.2 to 240.9 and the trips dict with aim 3.0. The Jetson reads its C3 table from inputs that no longer match the spec.

Evidence: ballistics_meta.json:2 `"params_source": "base/spec/params.py v1.2.7"`, line 3 `"params_hash": "cd6884897df6abd6"`, line 10 `"stroke_m": 0.2412`. params.py:181 `stroke=240.9`. pytest output: `AssertionError: ballistics.json was built from other physics inputs ... assert 'cd6884897df6abd6' == 'f011b9cab2d9b8c8'`. base/out/physics/summary.json source is also v1.2.7.

Fix: Run `cd base/physics && ./run_all.sh` (or at least `python ballistics.py`) against params 1.2.8 and commit base/out/ballistics.json, ballistics_meta.json and base/out/physics/*. Because current_trips_a is in the hash, also make the hash cover only the inputs ballistics actually uses, so a firmware-trip change does not mark C3 stale.

> Skeptic (confirmed): The defect is real on the actual review target, origin/base-v1 (472bb65). The local w/review branch is not an exact copy: it differs from origin/base-v1 in 71 files and still has params v1.2.7 / stroke=241.2, so the test passes there. On origin/base-v1, though, base/spec/params.py has VERSION="1.2.8", line 181 stroke=240.9, and line 230 current_trips_a aim=3.0. That includes commit 4dc43fc, 'Spec v1.2.8 ... stroke 240.9'. Meanwhile base/out/ballistics_meta.json still says 'params.py v1.2.7', hash cd6884897df6abd6, stroke_m 0.2412, and base/out/physics/summary.json source is v1.2.7. I exported 
>
> Skeptic (refuted): I could not reproduce the claim on either tree.

1. The checked-out branch (w/review, HEAD e7e3a65): params.py:13 has VERSION = "1.2.7" and params.py:181 has stroke=241.2, not 240.9. The aim trip is still 5.0 (line 230). These match ballistics_meta.json, which says v1.2.7, stroke 0.2412 and hash cd6884897df6abd6. Running `pytest test_phys_spec.py` in base/physics gave 8 passed, so test_committed_ballistics_not_stale does not fail here.

2. The current origin/base-v1 (efc14a5): params is at v1.2.9 (stroke=240.9, aim=3.0), and commit 21a50fd ("physics: full run on spec 1.2.9") rebuilt the commit
>

## 6. drivetrain.py checks the cock drive gear with face 12 mm; params as built is face 7, which hides a Lewis failure at limiter stall

- File: `base/physics/drivetrain.py:64`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: physics

Scenario: Cock function at the 5 N m limiter stall (a jammed carriage or sear). With face 12, drivetrain reports gear_sf_stall 1.69 (pass) and gear_sf_trip 1.86. With the as-built face 7 from params, I measured gear_sf_stall 0.98 (fails: 'gear fails; relies on the trip') and gear_sf_trip 1.08 (almost no margin). The physics report says the 26T drive gear survives a limiter stall, and it does not.

Evidence: drivetrain.py:64 `gear=dict(module=2.5, face=12.0, z_driver=26),`. params.py:176 `drive_gear=dict(module=2.5, teeth=26, face=7.0, ... note="... face 7 as built ...")`. spec_bridge.py has no mapping for LAUNCH.drive_gear. Scratch run: face 12 gives sf_stall 1.687, sf_trip 1.855; face 7 gives sf_stall 0.984, sf_trip 1.082.

Fix: In spec_bridge.apply, read LAUNCH.drive_gear (module, teeth, face) and LAUNCH.big_gear.teeth into p.launch. In drivetrain.cock, build gear=dict(module=dg['module'], face=dg['face'], z_driver=dg['teeth']) and ratio=big_gear.teeth/drive_gear.teeth from them, keeping COCK_DEFAULTS only as a fallback. Then rerun run_all.sh and report the cock stall verdict.

> Skeptic (confirmed): The claim holds against origin/base-v1, which is the tree under review. One caveat: the w/review working tree is not an exact copy of it. Its params.py has drive_gear face=16.0, while origin/base-v1 params.py:176 has face=7.0 with the note "face 7 as built (rail-relative 7.5..14.5, cad-launch 13:30)". I exported origin/base-v1 with git archive into /tmp/claude-0/review-scratch-O/A and checked there.

- **Hardcoded face:** drivetrain.py:64 hardcodes COCK_DEFAULTS gear=dict(module=2.5, face=12.0, z_driver=26). cock() passes c["gear"] to evaluate(), and evaluate() uses g["face"] for every Lewis c
>
> Skeptic (confirmed): The defect is real on the review target, origin/base-v1. One caveat about the local checkout: the w/review working tree is not an exact copy of base-v1. Its params.py is still v1.2.7, where LAUNCH.drive_gear has face=16.0, y=(142.5,158.5). On that tree alone the claim would look false. origin/base-v1 is at v1.2.8 (commit 4dc43fc, "launcher as built ... drive face 7"), and its base/spec/params.py:176 does say drive_gear face=7.0, y=(147.5,154.5), "face 7 as built".

On origin/base-v1, base/physics/drivetrain.py:64 still hard-codes COCK_DEFAULTS gear=dict(module=2.5, face=12.0, z_driver=26). Not
>

## 7. One trip value (tripA) serves both aim (3 A) and capstan reel-in (5 A): params reel_in is never read

- File: `firmware/base/grappstar_base/config.h:76`
- Severity: medium (claimed medium); 1 of 2 skeptics confirmed
- Owner: fw-base

Scenario: Once the header is regenerated (finding above), Cfg::tripA = REEL.current_trips_a.aim = 3 A. cmdReel uses the same c.tripA for capstan reel-in (until stop/seated, not take-up, not tension) at base_logic.h:876. Under the firmware's own model (line N = 17 x (I - 1.5) - 10.4), 3 A is only about 15 N at the line. The Jetson RETRIEVE sends max_n = 30 N (retrieve_limit_n), so a 15..30 N snag or drag ends in FAULT OVERCURRENT ('wiper over 3.0 A') instead of the LIMIT snag event the mission handles (mission.py retrieve expects LIMIT), and the reel-in aborts with a BaseError. params gives reel_in = 5.0 A for exactly this run, but no firmware name maps to it. EXPECTED in gen_params.py also leaves out P_REEL_CURRENT_TRIPS_A_REEL_IN.

Evidence: config.h:76 `float tripA = P_REEL_CURRENT_TRIPS_A_AIM;     // aim, reel-in`; base_logic.h:876 `w.c.tripA = takeupOnly ? c.takeupTripA : until == U_TENSION ? towTrip() : c.tripA;`; params.py:230 `aim=3.0, backward_launch=3.0, reel_in=5.0`; gen_params.py:37 EXPECTED lists only AIM, COCK, TOW trips.

Fix: Add `float reelTripA = P_REEL_CURRENT_TRIPS_A_REEL_IN;` (with an #ifndef default of 5.0f) to Cfg, use it in base_logic.h:876 in place of c.tripA for the capstan reel-in branch (and in the trip_a live update at :1024, key it as reel_trip_a), keep tripA for the aim and decock runs only. Add "P_REEL_CURRENT_TRIPS_A_REEL_IN" to EXPECTED in gen_params.py.

> Skeptic (confirmed): I couldn't refute this. One thing to know first: the local checkout (HEAD e7e3a65, merge-base 4d049da) is NOT the same as origin/base-v1 (472bb65). In the local base/spec/params.py:230 the line reads aim=5.0, reel_in=5.0. Checked there alone, the claim looks false: the quoted 'aim=3.0, backward_launch=3.0' isn't present, and aim and reel_in hold the same value. On origin/base-v1, which is the branch under review, the claim checks out.

What I found on origin/base-v1:
(1) params.py:230 has current_trips_a=dict(aim=3.0, backward_launch=3.0, reel_in=5.0, ...). DECISIONS D40 says every backward wi
>
> Skeptic (refuted): The claim rests on a params value that is not in the repo. base/spec/params.py:230 actually reads `current_trips_a=dict(aim=5.0, reel_in=5.0, cock=6.5, tow=6.5, blank_ms=50, hold_ms=30, note="D8/D33")`. There is no `aim=3.0` and no `backward_launch` key anywhere in the tree (I ran git grep over HEAD and searched base/ and knowledge/).

The checked-in params_gen.h:324-325 also defines both P_REEL_CURRENT_TRIPS_A_AIM and P_REEL_CURRENT_TRIPS_A_REEL_IN as 5.0f. The config.h default for P_REEL_CURRENT_TRIPS_A_AIM is 5.0f too. So once the header is regenerated, Cfg::tripA is 5 A, not 3 A. The capst
>

## 8. cad-frame silently moves the camera to y -96 while params and jetson campose use y -50: the bearing correction is 46 mm off

- File: `base/cad/parts/_frame_kit.py:82`
- Severity: low (claimed low); unverified (low)
- Owner: cad-frame

Scenario: FRAMEWORK.camera.pos = (150, -50, 100) lies inside FRAMEWORK.ebay (x 110..240, y -70..60) widened by 25 mm. _frame_kit.build therefore rewrites pos[1] = -70 - 26 = -96 and only adds a soft warning. Running frame.build(P) gives info camera_pos (150.0, -96.0, 100.0) and 'camera column moved to y -96 (off the bay)'. The printed mount and the Kiyo envelope sit at y -96. jetson/grappstar/campose.py still reads params y -50, so lateral_offset_m = -0.050 - 0.140 = -0.19 m against a real -0.236 m. pivot_target() then shifts every target 46 mm the wrong way, about 0.55 deg of bearing at 4.8 m, and the dart lands off target.

Evidence: _frame_kit.py:80-84 `pos = list(cam.get("pos", (150.0, -50.0, 100.0)))
 eb = fwr.get("ebay")
 if ... eb["y"][0] - 25 < pos[1] < eb["y"][1] + 25:
     pos[1] = eb["y"][0] - 26.0`; params.py:241-242 `ebay=dict(x=(110.0, 240.0), y=(-70.0, 60.0)...) camera=dict(... pos=(150.0, -50.0, 100.0)`; jetson/grappstar/campose.py:144 `lateral = pose["y_m"] - pose["rail_y_m"]` using params y.

Fix: Make params and CAD agree. Either the coordinator sets FRAMEWORK.camera.pos to (150, -96, 100), or another pose outside the bay, so jetson reads the as-built value, or cad-frame turns the relocation into a hard warning or error instead of silently overriding a shared number.


## 9. BASE_SPEC section 8 launcher numbers contradict params.LAUNCH and D26/D30

- File: `base/spec/BASE_SPEC.md:134`
- Severity: low (claimed low); unverified (low)
- Owner: cad-launch

Scenario: Someone sizing the spring anchors, rail length or stop from the binding section 8 prose would build preload 35 + stroke 245 = 280 mm at full cock. D30 says that is 9 percent over the C-243 rating (38.4 N). They would also cut a 33-tooth sector instead of 27 and make the stop 15 mm instead of the 19 mm that D26 needs for 300 N.

Evidence: BASE_SPEC.md:134-137 `big gear 40T m2.5 (33 teeth, ...) ... Springs 1:1, preload 35, stroke 245, 280 at full cock, fallback hole set at preload 5 ... Compliant stop at least 15 mm under 300 N (D6)`. params.py:177 `present=27`, :181 `stroke=240.9`, :182 `spring_preload=23.8`, :183 `spring_ext_full=265.0`, :184 `spring_preload_fallback=8.8`, :187 `stop=dict(travel_min=19.0, ...)`.

Fix: Change section 8 to: `big gear 40T m2.5 (27-tooth sector layer + full 40T layer) ... Springs 1:1, preload 23.8, stroke 240.9, 265 at full cock (D30), fallback hole set at preload 8.8 (250 mm); compliant stop at least 19 mm under 300 N (D26)`, or point to the params.LAUNCH keys instead of copying the numbers.


## 10. BASE_SPEC section 10 says aim trips at 5 A, against params and D40 (3 A)

- File: `base/spec/BASE_SPEC.md:162`
- Severity: low (claimed medium); 1 of 2 skeptics confirmed
- Owner: fw-base

Scenario: The binding firmware-behaviour prose tells fw-base to trip at 5 A for 30 ms in aim, reel and tow. params.REEL.current_trips_a has aim and backward_launch at 3.0 (D40) and tow at 6.5 (D33), and section 11 of the same spec says 3 A. Anyone implementing from section 10, as config.h did, sets an aim trip that D40 says strips the aim gears (SF 0.62).

Evidence: BASE_SPEC.md:162 `after a start; 5 A for 30 ms in aim, reel and tow; 6.5 A while cocking;`. BASE_SPEC.md:202-203 `every backward run in LAUNCH trips at 3 A (D40)`. params.py:230 `current_trips_a=dict(aim=3.0, backward_launch=3.0, reel_in=5.0, cock=6.5, tow=6.5, ...)`.

Fix: Change line 162 to: `3 A for 30 ms in every backward run in LAUNCH (aim, decock, backward jog, D40); 5 A in reel-in; 6.5 A while cocking and as the tow hard trip (D33)`, and point to params.REEL.current_trips_a for the values.

> Skeptic (confirmed): Note first: the local branch w/review is NOT a copy of origin/base-v1. On w/review, params.py:230 has aim=5.0, there is no D40, and config.h uses 5 A for aim. So the claim's evidence does not exist on the checked-out branch. On origin/base-v1, which is the review target, the claim holds. `git show origin/base-v1:base/spec/BASE_SPEC.md` line 162 reads "after a start; 5 A for 30 ms in aim, reel and tow; 6.5 A while cocking". Lines 204-205 of the same file say "Every backward run in LAUNCH trips at 3 A (D40)". params.py:231 has current_trips_a=dict(aim=3.0, backward_launch=3.0, reel_in=5.0, cock=
>
> Skeptic (refuted): I checked the claim against branch w/review (origin/base-v1), and its evidence is not in the repo:
- There is no D40. DECISIONS.md stops at D39, and grepping the .md, .py and .h files for "D40" returns nothing.
- params.py:230 says `current_trips_a=dict(aim=5.0, reel_in=5.0, cock=6.5, tow=6.5, ...)`. It has no aim=3.0 and no backward_launch key.
- BASE_SPEC.md has no "every backward run in LAUNCH trips at 3 A (D40)" text near lines 202-203. Section 11 never mentions 3 A, and no line in the spec contains "3 A".
- D8 (DECISIONS.md:16) and D33 (DECISIONS.md:41) both set aim at 5 A.
- config.h set
>

## 11. params.LAUNCH.flight still holds physics v1.2 range and speed (4.8 m, 7.41 m/s); the sim dart arc lands about 0.6 m beyond the physics range

- File: `base/spec/params.py:192`
- Severity: low (claimed low); unverified (low)
- Owner: physics

Scenario: launch.py flies the HD-sim dart along LAUNCH.flight, giving a 4.8 m range and 1.1 m apex. Physics v1.2.8 (BASE_SPEC section 11, ballistics.json) gives 4.19 / 3.75 / 3.36 m at 36 deg and 6.95 m/s. The sim shows the dart landing beyond the predicted range at every brake setting, and the TIMELINE flight line_out_m of 5.2 m inherits the stale range.

Evidence: params.py:192 `flight=dict(range_m=4.8, apex_m=1.1, pitch_deg=36.0, t_flight_s=1.1, note="for the sim; physics v1.2: 12.1 J, 7.41 m/s, 4.8 m at 0.5 N")`. BASE_SPEC.md:199-200 `12.1 J stored, 6.95 m/s at the muzzle, range ... 4.19 / 3.75 / 3.36 m at 36 deg`. launch.py:1066-1068 reads LAUNCH.flight range_m/apex_m. ballistics.json at 36 deg: range_m [4.187, 3.748, 3.364], apex_m [1.205, 1.162, 1.123], t_flight_s 0.819 mid.

Fix: Set LAUNCH.flight to range_m=3.75, apex_m=1.16, t_flight_s=0.82 (the 0.75 N mid-drag row at 36 deg, physics v1.2.8) and change the note to say so. Set TIMELINE flight line_out_m to about 4.0 to match.


## 12. tools/render.ground_z looks up LAYOUT keys 'ground_z' and 'deck_z' that params does not have, so the viewer ignores deck_top_z

- File: `base/tools/render.py:279`
- Severity: low (claimed low); unverified (low)
- Owner: cad-infra

Scenario: params defines LAYOUT['deck_top_z'] = -165 and FRAMEWORK['deck_z'], but no LAYOUT 'ground_z' or 'deck_z'. ground_z(P) therefore returns None and the web viewer puts the ground at the bounding-box minimum z of the assembly instead of the deck top at z -165, which also moves the ground in states where a part sweeps lower. The fallback silently wins over the shared number.

Evidence: render.py:278-282 `L = getattr(P, "LAYOUT", None)
 if isinstance(L, dict):
     for k in ("ground_z", "deck_z"):
         if isinstance(L.get(k), (int, float)):`; params.py LAYOUT has only `deck_top_z=-165.0`.

Fix: Add 'deck_top_z' to the lookup: `for k in ("ground_z", "deck_top_z", "deck_z"):`.


## 13. Jetson final approach overrides the firmware time limit with a 0.08 m/s take-up speed; physics and firmware give 0.058 m/s at duty 0.3

- File: `jetson/grappstar/mission.py:1233`
- Severity: low (claimed medium); 1 of 2 skeptics confirmed
- Owner: jetson

Scenario: Reshoot after a NO_GO landing verdict (mission.py:469, before the wrap). RETRIEVE is skipped because the line is not wrapped, so FINAL_APPROACH pulls the whole line on the take-up at takeup_duty 0.3. Take a 4.1 m shot: line_est_m = 4.1 x 1.05 + 0.3 = 4.6 m, and the Jetson sends max_s = 20 + 4.6/0.08 = 77.5 s. Physics retrieve.json gives v_line 0.0583 m/s at duty 0.3 on grass_high, so the pull needs about 79 s, plus the firmware's 300 ms ramp. The firmware raises FAULT DART_STUCK at 77.5 s (base_logic.h:1375) and the reshoot stops for a Retry. Without max_s, the firmware would have used 10 + 1.5 x 4.6 / 0.057 = 131 s. With line_m unknown (after a restart), the Jetson falls back to approach_m 1.5 (max_s 38.75 s), which fails for any line over about 2.2 m. The firmware's own default for that case is 30 s via approachMaxMs, but the Jetson sends an explicit value either way.

Evidence: mission.py:1233 `max_s = round(20.0 + float(line) / 0.08, 1)`; mission.py:1232 `line = self.shot.get("line_est_m") or self.cfg.approach_m`; base/out/physics/retrieve.json takeup rows: duty 0.3 grass_high `"v_line": 0.0583`; config.h:175 `float takeupMpsPerDuty = 0.29f, takeupMpsOff = 0.03f;` (0.057 m/s at 0.3); base_logic.h:849 `if (o.num("max_s", &ms)) { ... approachMs = (uint32_t)(ms * 1000); return true; }`.

Fix: In final_approach, either leave out max_s when line_m is known (so the firmware's approachLimit, 10 + 1.5 x line / v(duty), applies) or compute it the same way: `v = max(0.02, 0.29 * duty - 0.03); max_s = min(600, max(20, 10 + 1.5 * line / v))`, and set _wait_seated's timeout to max_s + 10. When line is unknown, do not substitute approach_m. Send no max_s, or use the firmware's 600 s cap with operator STOP.

> Skeptic (refuted): The cited code is accurate. mission.py:1232-1235 sends max_s = 20 + line/0.08, and an explicit max_s replaces the firmware's own approachLimit (base_logic.h:849). But the failure the claim describes does not happen.

1) Wrong speed for the scenario. The 0.0583 m/s figure is from the retrieve.json row "empty (line out, r core)", r = 25 mm. That row means all 20 m of line is out. After a 4.1 m shot, about 15.4 m is still on the storage reel, so the pack radius is about 29.5 mm. The row "full (20 m in)", r = 30.7 mm, gives 0.0706 m/s at duty 0.3 on grass_high. I interpolated v over the pack radiu
>
> Skeptic (confirmed): The defect is real, but the reviewer's worked example is wrong and the impact is smaller than claimed.

What checks out:
- mission.py:1231-1236 sends max_s = 20 + line/0.08 every time.
- base_logic.h:849 lets that value replace the firmware's own limit. That limit (base_logic.h:851-852) is max(20, 10 + 1.5*line/v), with v = 0.29*duty - 0.03 = 0.057 m/s at duty 0.3.
- The path is reachable. A NO_GO reshoot (mission.py:469) and _recover_sequence call bring_home without a wrap, so FINAL_APPROACH pulls the whole line on the take-up.

Why the 4.1 m example is wrong: it uses the 0.0583 m/s row, whic
>
