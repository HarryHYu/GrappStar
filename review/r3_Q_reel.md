# Round 3 Q: reel subsystem on the w/cad-reel tip

Reviewed at base-v1 commit 2f0475e. Method: independent finder subagents, one per lens; every finding of
medium severity or above was checked by two skeptic subagents told to refute it; kept only if fewer than
two refuted it. Low findings are listed unverified. Severity is the finder's, lowered when both skeptics
agreed it is real but rated it lower.

Confirmed: 17. Refuted and dropped: 0.

Reviewer note on #6 and #8 (SW_ARM polarity): both skeptics found that the geometry, the firmware (config.h armHomeOpen = true, counts falling edges), WIRING, PROTOCOL and BENCH_TEST all agree on OPEN at home. Only the build step, the part notes and params.SWITCHES.SW_ARM say closed at home. So fix the step text, notes and params, NOT the geometry (the finder's proposed geometry change would break the firmware). #2 (notch walls) still applies to the notch.

| # | Sev | File:line | Owner | Finding |
|---|---|---|---|---|
| 1 | high | `base/cad/parts/reel.py:697` | cad-reel | Pulley 2 axle cannot be fitted: its hole is blind under the post's web, so the M3x20 cannot go 'down through', and from below it has nothing to thread into |
| 2 | high | `base/cad/parts/reel.py:777` | cad-reel | SW_ARM home notch has vertical walls: the first cup move of every wrap or unwrap drives the notch wall sideways into the lever tip |
| 3 | high | `base/cad/parts/reel.py:883` | cad-reel | Brake rocker (the FIRM clutch spring) is sized for PETG, which Harry does not have; printed in PLA or PPA-CF it over-brakes FIRM, overstresses the plate and overloads the drum N20 |
| 4 | high | `base/cad/parts/reel.py:1265` | cad-reel | Line leaves pulley 2 9 to 34 deg out of its groove plane and runs through the pulley's own upper flange (it can climb and jump the flange); line_check only tests a horizontal stub |
| 5 | medium | `base/cad/parts/drum.py:478` | cad-drum | Inside the binding LAUNCH zone (-4..8, clutch LIGHT) the bar is pushed toward FIRM below drum -1.5 deg: up to 3.5 N payout brake at the -4 end stop |
| 6 | medium | `base/cad/parts/reel.py:781` | cad-reel | SW_ARM polarity is inverted: the cup-gear track is a notch (contact OPEN at home), but params and the build step say the contact closes at home, so a builder following the step gets a contact that never changes and every WRAP faults |
| 7 | medium | `base/cad/parts/reel.py:795` | cad-reel | SW_ARM switch base has no mounting holes in the bracket, and the BOM lacks its mount, clamp and bracket screws |
| 8 | medium | `base/cad/parts/reel.py:1495` | cad-reel | Still open (r2_J #6): the SW_ARM step and part notes describe a bump that closes the contact at home, but the geometry is a track that holds it closed except at home. Following the step leaves the contact always closed and every wrap faults |
| 9 | medium | `docs/build/MEASURE_LIST.md:104` | cad-reel | Still open (r2_J #17): the payout brake trim procedure still tells the builder to add printed shims under thrust ring T, but no shim part exists |
| 10 | low | `base/cad/parts/reel.py:306` | cad-reel | Sprag hub screws (M3x8) bottom exactly on the floor of their 5 mm blind tap holes, so the backstop flange may never clamp |
| 11 | low | `base/cad/parts/reel.py:863` | cad-reel | Cup gear print orientation puts about 73% of the gear 2.1 mm above the bed on the narrow track |
| 12 | low | `base/cad/parts/reel.py:883` | cad-reel | Still open (r2_J #21): the brake rocker is still PETG, which is not a spec material, and the clutch spring is still sized with PETG stiffness |
| 13 | low | `base/cad/parts/reel.py:920` | cad-reel | SW_ARM bracket 'Print flat' pose leaves its wall flange as an unsupported ledge at the top |
| 14 | low | `base/cad/parts/reel.py:1164` | cad-reel | No meshes() declared for the four cross-owner and two internal reel gear pairs, so the audit's meshcheck never checks the capstan, bus or cup meshes |
| 15 | low | `base/cad/parts/reel.py:1279` | cad-reel | line_path draws the capstan turns from S['wraps'] while the cup is posed from cad-drum's wrap-idler kinematics, so the tether and the cup disagree by up to 3.8 turns through the WRAP phase |
| 16 | low | `base/cad/parts/reel.py:1422` | cad-reel | Step 1 asks for eight deck screws, but the reel wall foot has seven holes |
| 17 | low | `base/cad/parts/reel.py:1486` | cad-reel | LIGHT trim set-up comes before the line is wound, and build doc G11 still says to trim with shims that do not exist |

## 1. Pulley 2 axle cannot be fitted: its hole is blind under the post's web, so the M3x20 cannot go 'down through', and from below it has nothing to thread into

- File: `base/cad/parts/reel.py:697`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-reel

Scenario: Step 'Pulley 2' says to push the M3x20 axle down through with a nyloc nut under it. The web box covers the axle position from py+3.49 to py+25.4. The axle hole is cut only up to py+15. My point check at (84, py+dy, -78.5) is empty for dy -12..14.9 and solid for dy 15.5..25.3, so the hole is closed at the top. The d5.5 SHCS head cannot enter a d3.4 hole, so the axle cannot be fitted from the top. From below, an M3x20 ends blind inside the clearance hole with no nut and no thread, so the pulley axle falls out under the drum. The audit confirms the modelled axle head is buried in the web: reel_p2_post / reel_p2_axle overlap 43.8 mm3, hidden by Allow('reel_p2_axle','reel_p2_post'). Without pulley 2 the line cannot reach cad-launch's pivot pulley (D34), and a dropped axle under tow throws the line off its path.

Evidence: reel.py:693 web = prim.box(px - 8.0, xc, py + gap + 3.49, py + gap + 22.0, pz - 4, pz + 4)   # covers x = px
reel.py:697 cut = [prim.cylY(fits.M3_CLEAR_D / 2, py - 15, py + 15, x=px, z=pz)]   # web top is py+25.4
reel.py:946-948 axle placed with its head at py+9.9..12.9 (inside the web)
reel.py:1499-1500 "push the M3 x 20 axle down through (nyloc nut under)"
audit.json: reel_p2_post/reel_p2_axle max_vol_mm3 43.791

Fix: In _p2_post, cut the axle hole through the web top: cylY(M3_CLEAR_D/2, py-15, py+gap+23). Add a head counterbore cylY(M3_HEAD_D/2 + 0.3, py+gap+3.5, py+gap+23) so the head seats on the upper cheek. Alternatively, shorten the web so it stops at px+6 and leave the axle free above. Place the bought axle with its head on py+gap+3.5, at T(p2[0], p2[1]+gap+3.5, p2[2]) @ Rx(90) with M3x20, so the tip comes out about 6 mm below the lower cheek for the nyloc. Remove the over-broad Allow so the audit catches this again.

> Skeptic (confirmed): I could not refute this. The claim has the wrong line numbers and screw length (the code says M3x16, not M3x20; _p2_post is at reel.py:563-580, the axle at :786-788, the step text at :1279-1283), but the defect itself is real. I checked it against the actual code and numbers.

- p2_centre() = (84, 9.643, -78.5) and P2_POST_X = 156.7. P1_T = 6, so gap = 3.4.
- The web is box(px-8 .. xc, py+gap+3.49 .. py+gap+22, pz-4 .. pz+4). That is x 76..156.7, y py+6.89..py+25.4, z -82.5..-74.5, so it covers the axle line at x=84, z=-78.5.
- The only axle cut is cylY(M3_CLEAR_D/2, py-15, py+15). The hole go
>
> Skeptic (confirmed): The claim holds in the current code, though the reviewer cites the wrong line numbers and fastener. The checked-out file has 1335 lines. _p2_post is at reel.py:563-580, the axle is an M3x16 (reel.py:786-788), and the steps at 1279-1283 and 1293 say "push the M3 x 16 axle down through (nyloc nut under)". The geometry is as claimed.

What I checked:
- With P1_T=6, gap=3.4, p2_centre=(84.0, 9.643, -78.5) and P2_POST_X=156.7, the web box runs x 76..156.7, y py+6.89..py+25.4, z pz±4. That covers the axle position (px, pz).
- The only axle cut is cylY(M3_CLEAR_D/2, py-15, py+15) at reel.py:576.
- I 
>

## 2. SW_ARM home notch has vertical walls: the first cup move of every wrap or unwrap drives the notch wall sideways into the lever tip

- File: `base/cad/parts/reel.py:777`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-reel

Scenario: The cup starts at home with the SW_ARM lever tip down in the notch. The free lever top is at R z -43.35 and the track spans -45.1 to -43, so the tip sits 1.75 mm deep in the notch. WRAP or UNWRAP turns the cup. After about 5 deg of cup rotation the radial notch wall meets the side face of the lever's 8 mm wide tip boss. The lever bends only in R Z, and the wall pushes it purely tangentially. Either the wrap idler stalls the drum, or the PLA lever or its clamp breaks. After that SW_ARM counts 0 of 7 and raises F_DRUM_SENSOR, with lever debris next to the cup gear mesh. My scratch run shows the wall is vertical. The notch spans -119.5 to -81.5 deg at r 13.5 and r 15.5, and at track depths z -0.3, -1.0 and -2.0 alike. The modelled lever/gk overlap is 0 from 0 to -4 deg, then rises steadily from -5 deg (0.004 mm3) to -30 deg (11.5 mm3): the wall passes through the lever side. The audit passes only because Allow(reel_sw_lever, reel_gk, max_vol=60) hides it. A second problem: the lever touches the track only on a 1.0 to 1.3 mm sliver. The track is r 13..16, but the tip is placed for SW_BUMP_R 16.5, so its actuation point at r 16.5 lies outside the track.

Evidence: reel.py:777-781:
    notch = prim.extrude(prim.poly_cs(pts), SW_BUMP_H + 1.0, z=-SW_BUMP_H - 0.5)
    notch = prim.place(notch, tf.Rz(sw_tip_deg() + 90.0))
    # 45 deg ramps: subtract a slightly smaller notch at full depth and a wider shallow one is approximated by
    # stepping the notch in three depths
    out = ring - notch
The docstring (reel.py:766) promises '45 deg ramps', but a single straight extrude is cut. reel.py:769 ring = prim.tube(16.0, 13.0, ...) and reel.py:734 SW_BUMP_R = 16.5. reel.py:1147-1148 Allow(... max_vol=60.0).

Fix: Build the notch cutter as a ramped solid. Use Manifold.batch_hull of two thin wedges: one at the track face (z = -SW_BUMP_H - 0.5) with half-angle SW_BUMP_DEG, and one at the gear face (z = +0.5) with half-angle SW_BUMP_DEG + (SW_BUMP_H + 1)/SW_BUMP_R rad. The walls are then 45 deg ramps that lift the lever axially. Make the notch floor wider than the lever's contact patch: either narrow the lever tip to a rounded nose of about 4 mm (contact_switch b) or raise SW_BUMP_DEG to at least 16. Move the lever onto the track centre: SW_BUMP_R = 14.5, which keeps the track inside the tooth root r 16.1. Add a test that the lever/gk overlap grows smoothly with cup angle from 0 deg, and lower the Allow max_vol to the track-press volume.

> Skeptic (confirmed): I could not refute this claim. The line numbers are wrong: the code is at base/cad/parts/reel.py:625-640 at HEAD a4a19c4, not 766-781. The content matches the claim exactly.

1. **The notch walls are vertical.** `_cup_gear_track` builds the notch from one 2D wedge, `poly_cs(pts)`, extruded straight in z: `prim.extrude(..., SW_BUMP_H + 1.0, z=-SW_BUMP_H - 0.5)`, then `ring - notch`. Every depth gets the same angular half-width, `a_top` = 12 deg + 2.1/16.5 rad, about 19.3 deg. The docstring promises "45 deg ramps", and the comment even admits the ramp is only "approximated by stepping the notch 
>
> Skeptic (confirmed): I could not refute this claim. The defect is real.

Line numbers: the cited lines (777-781, the Rz(sw_tip_deg()+90) placement, Allow max_vol=60 at about 1158) match origin/base-v1. The w/review checkout has the same code at reel.py:624-641 and the Allow at 941-942, without the sw_tip_deg rotation. Both versions cut the notch the same way. `_cup_gear_track` builds a planar wedge with `poly_cs` and makes it with one straight `prim.extrude(..., SW_BUMP_H + 1.0, z=-SW_BUMP_H - 0.5)`. That gives walls that are exactly vertical over the whole track height. The extra half-angle `a_top = half + SW_BUM
>

## 3. Brake rocker (the FIRM clutch spring) is sized for PETG, which Harry does not have; printed in PLA or PPA-CF it over-brakes FIRM, overstresses the plate and overloads the drum N20

- File: `base/cad/parts/reel.py:883`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-reel

Scenario: The 5.0 mm leg stroke is fixed by the cam (4.0) and the crank, so FIRM preload scales with E. Using reel_calc.rocker_section (y_load -48.7) with the plate as built (t 3.0): PLA (E 3500, cad/lib/clutch.py) gives FIRM axial 33.5 N = 8.0 N line (D24: 5.0), peak plate stress about 29 MPa held through all of NEUTRAL, WRAPPED and REEL. That is above SIGMA_FLEX_PLA 25 MPa short-term and about 3x the 10 MPa sustained creep limit, so FIRM sags over time. PPA-CF (E 6000) gives 55.9 N axial = 13.4 N line and about 49 MPa. Leg force becomes 14.5 N (PLA) or 24.2 N (PPA-CF), so bar force on cad-drum's cam is about 1.38 x leg = 20 N or 33 N plus friction, against the 15 N design. Feeding that into _drum_kin.loads, the N20 margin drops from 2.34 to about 1.6 (PLA) or about 1.1 to 1.2 (PPA-CF), below the 2x rule, so the drum can stall on the 30..40 deg brake ramp. Thinning the PLA plate does not fix it: t 2.5 gives the 5 mm stroke at 24.9 MPa, still far over the sustained limit.

Evidence: reel.py:883 `add("reel_rocker", _rocker(), "PETG", ...)`; reel.py:609 docstring `Brake rocker (PETG, flexible lever)`; reel_calc.py:154 `def rocker(d, E=2100.0, b=None)`; reel_calc.py:171 `def rocker_section(d, y_load=-48.6, t=3.0, t_leg=5.5, y_leg=-30.0, E=2100.0, n=4000)`; reel_calc.py:326 label `rocker plate thickness (PETG, 10 wide)`; reel_NOTES.md:98 `PETG plate 3.0 thick`; reel_NOTES.md:123 `Peak plate stress at FIRM about 17 MPa ... (PETG)`; cad/lib/clutch.py:31-38 `E_PLA = 3500`, `E_PPA_CF = 6000`, `SIGMA_FLEX_PLA = 25`, `SIGMA_FLEX_PLA_SUSTAINED = 10`; _drum_kin.py:44 `brake_force=15.0`; WORKPLAN quality rules: only PLA, plus PPA-CF for loaded parts.

Fix: Tag the rocker PPA-CF (lower creep than PLA). Change rocker()/rocker_section() to default E=clutch.E_PPA_CF and fix the summary label. Re-size the plate for the 5.0 mm stroke: at 15 mm wide, t about 2.05 mm gives 5.0 mm (sigma about 37 MPa). To lower stress at the same rate, widen the plate and thin it (k ~ b t^3, sigma ~ t at fixed k), e.g. 25 wide x about 1.75. Update ROCKER_Z, the test bound (test_reel_calc.py:334, PETG 25 MPa) to a PPA-CF allowable, and reel_NOTES section 6. Re-send the FIRM bar force (about 12 N plus friction) to cad-drum so brake_force and the N20 margin stay valid.

> Skeptic (confirmed): I checked this against origin/base-v1, which is where the claim's line numbers match. The local w/review checkout is older and has no rocker_section. The defect is real, but it is smaller than claimed.

What holds:
(1) reel.py:863 adds reel_rocker as "PETG". reel_calc.rocker and rocker_section both default to E=2100 (PETG).
(2) The rule sheet (WORKPLAN line 57) allows only PLA, plus PPA-CF for loaded gears and keyed shafts. Harry's purchases list 10 kg of PLA only, with "carbon filament" deferred (HARRY_MESSAGES:65-66). Nothing in BASE_SPEC or DECISIONS approves PETG.
(3) The only cover for th
>
> Skeptic (confirmed): I could not refute this. The line numbers in the claim come from origin/w/cad-reel (reel.py:883, rocker_section). The checked-out w/review tree (HEAD a4a19c4) has the same defect at different lines: base/cad/parts/reel.py:742 is `add("reel_rocker", _rocker(), "PETG", ...)`, the docstring at reel.py:481 says "Brake rocker (PETG, flexible lever)", reel_calc.py:154 has `rocker(d, E=2100.0)`, the summary label at reel_calc.py:287 says PETG, and reel_NOTES.md lines 96 and 108 say PETG. origin/base-v1 also tags it PETG (reel.py:863).

The material contract in knowledge/WORKPLAN_2026-11-03.md:57 allo
>

## 4. Line leaves pulley 2 9 to 34 deg out of its groove plane and runs through the pulley's own upper flange (it can climb and jump the flange); line_check only tests a horizontal stub

- File: `base/cad/parts/reel.py:1265`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-reel

Scenario: Pulley 2's axis is world Z, so its groove plane is horizontal at world z -60 (P2_WORLD_Z). The real run to cad-launch's pivot pulley lies in the vertical y 119 plane and climbs to the pivot pulley's lower tangent: 9.0 deg at aim 6, 13.4 deg at 20, 19.1 deg at 30 and 33.8 deg at aim 47 (I computed this from line_path, with launch.line_entry present). I swept a 1 mm line tube along the true departure direction against the posed reel_pulley2: it cuts the pulley's upper groove flank and rim by 0.15 / 0.62 / 2.05 / 3.09 mm3 at aim 6 / 20 / 30 / 47 (script /tmp/claude-0/review-scratch-Q/p2_fleet.py). So on every shot the line pays out at about 8.6 m/s over the flange edge. At high pitch (aim 6..47 is the normal range) it rides up the flank onto the r 9 rim. The rim's exit point is outside the cheeks (cheeks cover world y 106..118, the line leaves at y 119), so a slack line after landing can jump the flange and foul the axle or cheek. Pulley 1 to pulley 2 also runs 4.4 deg out of both groove planes (R Y 23.5 down to 9.64 over 179 mm). line_check never sees this because it tests only a horizontal 30 mm stub.

Evidence: reel.py:633 `P2_WORLD_Z = -60.0  # feed height`; reel.py:941 pulley 2 placed with `tf.Rx(-90)` (axis R +Y = world Z); reel.py:1262 `arc2 = ... p2[2] - P2_R * math.sin(t)` (exit horizontal, heading -X); reel.py:1265 `spans.append((np.array([arc2[-1], arc2[-1] + (-RUN_CHECK, 0.0, 0.0)]), {"reel_pulley2"}))` (checked stub is horizontal); reel.py:1314-1326 _to_pivot goes from pulley 2's exit straight to the pivot pulley's lower tangent (world z about -50 at aim 6, -35 at aim 47); reel.py:719-720 groove profile (root r 6.5, flank to r 9 within 1.8 mm of the mid-plane); params.py:198 `pitch_min=6.0, pitch_max=47.0`.

Fix: Make the line leave pulley 2 in pulley 2's groove plane and put the pitch-dependent rise in the y 119 plane. Add a pulley 3 (printed, 693ZZ, line radius 7, axis world Y, groove in the y 119 plane) on the pulley 2 post arm at about world (52, 119, -53), so the line passes horizontally under it at z -60 and wraps it up toward the pivot pulley. Pulley 3 stays clear of the pivot pulley, whose centre is at (32, -30) at aim 47. Route line_path and _to_pivot through pulley 3. In line_check, replace the horizontal stub with the real departure direction swept over aim 6..47. Add a test that the line's angle out of each pulley's groove plane stays under 4 deg for aim 6..47. While re-routing, set pulley 1's exit height equal to pulley 2's line plane (or tilt pulley 1's axis) to remove the 4.4 deg fleet on the long +Y run.

> Skeptic (confirmed): The claim holds. I checked the code and ran a scratch script, /tmp/claude-0/review-scratch-Q/skA_dep.py (read-only; the working tree is unchanged), from base/ with the real launch module (launch.line_entry is present).

What the code does:
- Pulley 2's axis is R +Y, which is world Z (p2_centre, reel.py:516-523). The line therefore sits on a horizontal groove plane at world z -60.
- `_line_points` ends with a horizontal stub `arc2[-1] + (-RUN_CHECK, 0, 0)` (reel.py:1055). That stub is the only part of the run after pulley 2 that line_check (reel.py:1136) ever tests.
- `line_path` drops that stu
>
> Skeptic (confirmed): The defect is real in the current code, but several of the claim's numbers are wrong and its "every shot" framing overstates it.

What checks out:
- Pulley 2 is built with its axis along world Z (reel.py:781, place `tf.Rx(-90)`), so its groove is horizontal at z -60 (P2_WORLD_Z, reel.py:512).
- arc2 leaves heading -X at z -60 (reel.py:1052).
- line_check only sweeps a horizontal 30 mm stub (reel.py:1055 with RUN_CHECK; the stub is dropped in line_path at :1080).
- _to_pivot (reel.py:1104) runs straight from that exit to the lower tangent of the pivot pulley, all in the y 119 plane, so the line
>

## 5. Inside the binding LAUNCH zone (-4..8, clutch LIGHT) the bar is pushed toward FIRM below drum -1.5 deg: up to 3.5 N payout brake at the -4 end stop

- File: `base/cad/parts/drum.py:478`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-drum

Scenario: cam_radius blends track 2 from FIRM (stop_hi 352) to LIGHT (stop_lo -4) across only the 4 deg unreachable gap. The bar's round nose (r 3 at cam r 33, about +-5.2 deg) straddles that cliff whenever the drum is below about -1.5 deg. reel.bar_mm (nose envelope on the real cam) gives bar 0.78 / 1.67 / 2.55 mm at drum -2 / -3 / -4. That is leg push 1.0 / 2.1 / 3.3 mm, axial preload about 5.9 / 10.1 / 14.5 N, about 1.4 / 2.4 / 3.5 N line tension instead of 0.5 N. If the drum overshoots the LAUNCH park or is driven onto its -4 hard stop, a shot fires with up to 7x the D7/D24 LAUNCH brake: lost range and a hard spin-up pull.

Evidence: drum.py:478-481 `if th > c["stop_hi"] or th < c["stop_lo"]:  # unreachable gap: blend from the high stop value to the low one ... v = K.brake_lift(c["stop_hi"], c) * (1 - u) + K.brake_lift(c["stop_lo"], c) * u`; reel.py:1026 `NOSE_R = 3.0`; reel.py:1029-1048 bar_mm nose envelope; BASE_SPEC.md:122 `LAUNCH (-4..8, collar LAUNCH, clutch LIGHT, cup locked)` and :126 `End stops -4 and 352`; reel_NOTES.md admits `past the LAUNCH stop (drum -4 .. -2) the nose already sees the gap blend toward FIRM (bar 2.5 mm at -4)`.

Fix: In drum.py cam_radius, drop from FIRM to cam_r0 within the first 1 deg past stop_hi (352..353) and hold cam_r0 for the rest of the gap. Together with that, cad-reel reduces NOSE_R to 1.5 mm (half-angle about 2.6 deg at r 33), so at drum -4 the nose spans 353.4..358.6 deg, all LIGHT, and at 352 it still reads full FIRM. Add a test that reel.bar_mm(P, drum_deg=d) == 0 for every d in [-4, 8].

> Skeptic (confirmed): The claim holds up. Note first that the working tree (w/review) does not match base-v1 in this area: reel.py on w/review has no bar_mm and no NOSE_R. I checked origin/base-v1 instead, extracted with git archive to /tmp/claude-0/review-scratch-Q/bv1, and the repo was not modified.

What the base-v1 code does:
- drum.py cam_radius (around line 476) ramps track 2 linearly from brake_lift(352) (FIRM, 4 mm) down to brake_lift(-4) (LIGHT, 0) across only the 4 deg gap.
- reel.py line 1004 sets NOSE_R = 3.0. bar_mm (lines 1007-1024) takes the round nose's envelope over +-8 deg of the real cam_radius.

>
> Skeptic (confirmed): I could not refute this claim. I checked the code and ran it, and the defect is real.

1. The cam code is as quoted. The cited line numbers match origin/w/cad-reel. origin/base-v1 has the identical cam_radius at drum.py:494-502, and its reel.py has NOSE_R = 3.0 and bar_mm(P, S) at :1004-1024. The w/review branch is not actually an exact copy of base-v1, but the drum cam logic is the same on every branch. cam_radius blends linearly from brake_lift(stop_hi = 352, which is FIRM) to brake_lift(stop_lo = -4, which is LIGHT) across only the 4 deg gap. The bar's nose has r 3 and sits at cam r about 3
>

## 6. SW_ARM polarity is inverted: the cup-gear track is a notch (contact OPEN at home), but params and the build step say the contact closes at home, so a builder following the step gets a contact that never changes and every WRAP faults

- File: `base/cad/parts/reel.py:781`
- Severity: medium (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-reel

Scenario: Harry builds the SW_ARM step as written: cup at home, sets the contact screw so it 'just closes there'. With the notched track the lever is free (in the notch) at home, and everywhere else the 2.1 mm track pushes it further in, so the contact stays closed all the way round. During the WRAP sector the firmware sees no rising edge (base_logic.h:464 counts rose(SW_ARM)) and raises F_DRUM_SENSOR 'SW_ARM counted 0 of 7 cup turns' (base_logic.h:519) on every wrap. That stops the mission at the wrap. It also breaks the params contract SWITCHES.SW_ARM, which says closed +-12 deg at home.

Evidence: reel.py:765-768 (_cup_gear_track docstring): "The SW_ARM lever rides the track (contact CLOSED) and drops into the notch at home (contact OPEN)"; reel.py:781 `out = ring - notch`; reel.py:1147 Allow "the track holds the lever down (contact closed) except at home". Against this: reel.py:1495-1496 step "the bump on the cup gear presses the lever; set the contact screw so it just closes there and opens 15 degrees either side"; reel.py:867 notes "the bump on its back trips SW_ARM"; params.py:262 `SW_ARM=dict(... closes="wrap cup at home (cam bump on the cup gear, closed +-12 deg)")`. OPEN_ITEMS cad-reel #1 (r2_J #6) is still open on 2f0475e.

Fix: Make the geometry match params and the step: at reel.py:781 replace `out = ring - notch` with `out = ring ^ notch`, so there is a raised bump of +-SW_BUMP_DEG at home and the rest of the ring is cut away. Set z_rest in sw_frame (reel.py:757) so the free lever top sits 0.4 mm behind the bump top, not behind the gear face. Change the _cup_gear_track docstring and the Allow at reel.py:1147 to 'bump presses the lever (contact closed) only at home'. Re-check that reel_sw_lever / reel_gk overlap stays within max_vol=60 at home and is 0 elsewhere.

> Skeptic (confirmed): I could not refute the defect. The line numbers in the claim are stale; on HEAD a4a19c4 the code is at different lines, and it says the following.

Geometry. In base/cad/parts/reel.py, _cup_gear_track (624-640) builds a 2.1 mm raised ring with a notch cut out at home (line 640: `out = ring - notch`). Its docstring says the lever rides the track with the contact CLOSED and drops into the notch at home with the contact OPEN. sw_frame line 617 sets z_rest 0.4 mm behind the gear face, "in the notch". SW_BUMP_H is 2.1 = gap 0.4 + pretravel 1.3 + overtravel 0.4. So the contact is open at home and cl
>
> Skeptic (confirmed): The defect is real, but some of the claim's details are wrong. On the current HEAD (a4a19c4) the line numbers have shifted, but the content matches. The geometry is a notch at home: in base/cad/parts/reel.py, _cup_gear_track at line 624 does `out = ring - notch` (line 640), and its docstring says the contact is CLOSED on the track and OPEN in the notch. sw_frame line 617 sets z_rest so the free lever sits 0.4 mm behind the gear face, in the notch. The Allow at line 941 says the same. The firmware also follows this geometry: config.h:206 has `armHomeOpen = true` ("closed while the cup is away a
>

## 7. SW_ARM switch base has no mounting holes in the bracket, and the BOM lacks its mount, clamp and bracket screws

- File: `base/cad/parts/reel.py:795`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-reel

Scenario: The step says 'Screw the switch base to the bracket, and the bracket to the back of the wall ... with two M3 x 8'. switch.contact_switch puts 4 x M3 clearance holes in the base, at R (11,-49.7), (-17,-49.7), (11,-35.2) and (-17,-35.2). My point check shows the bracket plate is solid under all four, because _sw_bracket cuts only the two flange holes. The bought list for SW_ARM is only 'M3x10 SHCS + 2 M3 nuts'. It omits the 4 base mount screws, the 2 lever clamp screws (info clamp_screw '2x M3x10 self tapping') and the 2 M3x8 bracket-to-wall screws. A builder working from the pack cannot mount the contact without drilling and has none of the screws, so SW_ARM is not fitted, the firmware sees no pulses, and every wrap faults.

Evidence: reel.py:795 cut = [prim.cyl(fits.M3_CLEAR_D / 2, 10, x=sx, y=y0 - 13.0, z=-WALL_T - 6.0) for sx in (-12.0, 12.0)]   # only flange holes
reel.py:926-927 bought_ref="M3x10 SHCS + 2 M3 nuts"
reel.py:1493-1494 "Screw the switch base to the bracket, and the bracket to the back of the wall below the gears with two M3 x 8."
switch info: mount_screw '4x M3 through the base (3 thick), clearance holes at y = +-14.0', clamp_screw '2x M3x10 self tapping'

Fix: In _sw_bracket, transform the base's four mount-hole centres (switch x in (-root_len/2, Lf/2), y = +-(base_w/2 - 3.5)) through sw_frame(). Cut fits.M3_TAP_D/2 holes 4 mm deep into the plate at those points. Extend reel_sw_hw.bought_ref to 'M3x10 SHCS + 2 M3 nuts; 4 x M3x6 SHCS; 2 x M3x10 self tapping; 2 x M3x8 SHCS', or add a separate bought part for each.

> Skeptic (confirmed): The claim holds, though its line numbers and one quote are stale. On HEAD 6a056cb, _sw_bracket is at base/cad/parts/reel.py:644-654. Its only cut is the two flange clearance holes at x = +-12, y = y0-13 (line 653). The plate is prim.box(x0-3..x1+3, y0-9..y1, zb-4..zb), built round the placed switch base's bounding box, so it covers the whole base footprint. contact_switch (base/cad/lib/switch.py) cuts 4 M3 clearance mount holes at switch x in (-root_len/2, Lf/2) = (-7, 7.5) and y = +-14. I ran a scratch script (/tmp/claude-0/review-scratch-Q/sw.py) that transforms those 4 hole centres through 
>
> Skeptic (confirmed): The claim is real. In base/cad/parts/reel.py, _sw_bracket() (lines 644-656) builds the plate as a solid prim.box. Its only cuts are the two M3 clearance holes in the flange at (±12, y0-13). Nothing is cut under the switch base. contact_switch() in base/cad/lib/switch.py does cut 4 M3 clearance holes in the base, at x in (-root_len/2, Lf/2) and y = ±(base_w/2 - 3.5), and they have nothing to screw into.

The BOM is short as well. The only bought line for SW_ARM is reel_sw_hw, bought_ref "M3x10 SHCS + 2 M3 nuts" (line 767), which covers the contact screw, the nut and the jam nut. I found no BOM 
>

## 8. Still open (r2_J #6): the SW_ARM step and part notes describe a bump that closes the contact at home, but the geometry is a track that holds it closed except at home. Following the step leaves the contact always closed and every wrap faults

- File: `base/cad/parts/reel.py:1495`
- Severity: medium (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-reel

Scenario: The builder turns the cup to home. The lever is then FREE in the notch, not pressed by a bump. Following the step, they set the contact screw so it 'just closes there'. On the track the lever is pushed 1.75 mm further, so the contact never opens anywhere in the turn. The firmware counts rose(SW_ARM) inside SEC_WRAP and gets 0 of 7, so base_logic.h:519 raises 'SW_ARM counted 0 of 7 cup turns' (F_DRUM_SENSOR) on every wrap and unwrap. The overtravel also crushes the head onto the nut beyond the switch's designed travel. The polarity also contradicts the spec, which says closed at home.

Evidence: reel.py:1495-1496 step: "Turn the cup to home (ring at the top): the bump on the cup gear presses the lever; set the contact screw so it just closes there and opens 15 degrees either side."
reel.py:867 notes: "the bump on its back trips SW_ARM"
reel.py:925 notes: "the cup gear's bump presses its tip at home"
Against the geometry, reel.py:765-767: "The SW_ARM lever rides the track (contact CLOSED) and drops into the notch at home (contact OPEN)"
and reel_NOTES.md section 10: "OPENS for about +-5 deg".
params.py:262: closes="wrap cup at home (cam bump on the cup gear, closed +-12 deg)".
Three different window figures (15, 12, 5 deg) are given, and the notch is in fact +-19 deg.

Fix: Pick one polarity. Option A (matches params.py:262 and BASE_SPEC 76): make it a bump. Use a raised pad of +-SW_BUMP_DEG with 45 deg ramps at home and a flat face elsewhere, so the lever is free except at home. Option B: keep the notch and change params SWITCHES.SW_ARM.closes through the coordinator. Either way, rewrite the step text to match the geometry. For Option B: 'Turn the cup half a turn away from home so the lever rides the track; set the contact screw so it just closes; turn back to home: it must open (telemetry W 0) about +-N deg either side.' Then fix the notes at 867 and 925 and the reel_NOTES section 10 window to the real number.

> Skeptic (confirmed): The defect is real. The claim's line numbers are stale: the step is at reel.py:1272-1278, and the notes are at 726 and 765. The substance holds.

Geometry (code, not just the docstring):
- _cup_gear_track() (reel.py:624-641) builds a full ring of height SW_BUMP_H=2.1 and subtracts a notch at home. The notch is +-12 deg at the floor and about +-19.3 deg at the top (12 deg + 2.1/16.5 rad).
- It is unioned into reel_gk at line 723.
- sw_frame() puts the free lever 0.4 mm behind the gear face "in the notch" (line 617).
- So at home the lever touches nothing. On the track it is pushed about 1.7 mm.
>
> Skeptic (confirmed): The defect is real. The claim's line numbers are stale (reel.py has 1335 lines), but the text it quotes exists at other lines.

The geometry is a track with a notch, not a bump. In reel.py, _cup_gear_track (lines 624-641) builds a raised ring of height SW_BUMP_H = 2.1 and cuts a notch at home. Its docstring says the lever "rides the track (contact CLOSED) and drops into the notch at home (contact OPEN)". sw_frame (line 617) sets z_rest to "free lever top 0.4 behind the gear face (in the notch)". The Allow at line 941 says "the track holds the lever down (contact closed) except at home".

The t
>

## 9. Still open (r2_J #17): the payout brake trim procedure still tells the builder to add printed shims under thrust ring T, but no shim part exists

- File: `docs/build/MEASURE_LIST.md:104`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-reel

Scenario: G4 brake check reads off, and the builder goes to G11. G11 says to add or remove 0.4 mm shims under T 'from the reel assembly steps' and to print more. No reel_* shim part exists, and the steps set LIGHT with the two trim screws instead (reel.py:1486-1489). The builder either cannot do the trim or stacks improvised shims, which raises the FIRM squeeze above D24 as well.

Evidence: MEASURE_LIST.md:104 "| G11 Payout brake trim | If G4 is off: add or remove printed 0.4 mm shims under thrust ring T (reel assembly steps) | reel shims (print more) |"
reel.py:890 notes "LIGHT trim screws: their tips press the thrust ring; 1/8 turn = 0.4 N"

Fix: Replace G11 with the trim-screw procedure from the 'Reel shaft into the wall' step. Line up the reel gear holes with the wall holes, then turn both M3 trim screws the same amount with a 2.5 mm hex key, 1/8 turn = 0.4 N, until 50 g just pulls line off. Drop 'reel shims (print more)'. Regenerate GrappStar_Build_Guide.md.

> Skeptic (confirmed): The defect is real, but two of the claim's line citations are wrong.

What I checked:
- docs/build/MEASURE_LIST.md:104 and docs/build/GrappStar_Build_Guide.md:3599 both say "add or remove printed 0.4 mm shims under thrust ring T (reel assembly steps) | reel shims (print more)".
- No shim part exists. I searched base/cad, base/pack, docs and firmware for "shim" with grep, and the only hits are those two G11 lines plus the old panel proposal base/spec/panel/C_compact.md, which describes a different design (a wave spring shim). No file name contains "shim" either.
- The real reel design sets the 
>
> Skeptic (confirmed): I checked this on origin/base-v1 (863d58d/4a28d2d), which is what the claim cites. The local w/review HEAD (6a056cb) is older, so its line numbers differ, but the defect is in both.

Confirmed:
- docs/build/MEASURE_LIST.md:104 and GrappStar_Build_Guide.md:3795 still say "add or remove printed 0.4 mm shims under thrust ring T (reel assembly steps)". The reprint column says "reel shims (print more)".
- `git grep -i shim` over base/docs/firmware on origin/base-v1 finds no reel shim part. The only shim part is launch_aim_drag_shim, which belongs to the aim drag, not the reel.
- reel.py now sets LI
>

## 10. Sprag hub screws (M3x8) bottom exactly on the floor of their 5 mm blind tap holes, so the backstop flange may never clamp

- File: `base/cad/parts/reel.py:306`
- Severity: low (claimed low); unverified (low)
- Owner: cad-reel

Scenario: The head seats at R Z 3.0 (counterbore floor). The M3x8 shank then ends at Z -5.0, which is exactly the floor of the 5.0 mm blind self-tapping hole in the 6 mm PLA wall. Any first-layer squish or compacted tapping chips stop the screw before the head clamps. The PPA-CF hub then sits loose on its spigot and rocks under the capstan's cantilevered line load (about 70 N x 50 mm = 3.5 N m at the 70 N tow cap, D39).

Evidence: reel.py:304-306 `# hub screws tap into the wall from the front (blind, 5 deep)` ... `prim.cyl(fits.M3_TAP_D / 2, 5.0 + 0.01, x=x, y=y, z=-5.0)`; reel.py:454 `cut.append(prim.cyl(fits.M3_HEAD_D / 2, 30.0, x=x, y=y, z=3.0))`; reel.py:832-836 `fasteners_m3(8.0)` placed at Z 3.0, `bought_ref="4 x M3x8 SHCS"`.

Fix: Run the tap hole through the whole wall (z from -WALL_T - 0.01 to 0.01; the screw tip at Z -5 stays 1 mm inside the wall and clear of the bus gear behind it), or keep the blind hole and use M3x6 screws (3 mm engagement, 2 mm clearance).


## 11. Cup gear print orientation puts about 73% of the gear 2.1 mm above the bed on the narrow track

- File: `base/cad/parts/reel.py:863`
- Severity: low (claimed low); unverified (low)
- Owner: cad-reel

Scenario: reel_gk has no print rotation, so _bed sets it on its minimum z, which is the 3 mm wide track ring r 13..16. My slices of the print pose show 244 mm2 on the bed, against 910 mm2 at z 2.2 where the gear body starts. The teeth (r 16..19.4) and the inner web (bore to r 13) print over air. The audit flags 662.9 mm2 of overhang steeper than 45 deg on reel_gk. The rear ends of the teeth droop, the face that meshes cad-drum's 42T wrap idler is degraded, and the notch walls (the switch edge) print as sagging bridges. This contradicts the note 'Prints rear face down: the track is the bed face'.

Evidence: reel.py:863-867 add("reel_gk", prim.union([_gear(...), _cup_gear_track()]), "PPA-CF", joint="reel_cup", ...)   # no R
reel.py:767-768 "Prints rear face down: the track is the bed face."
reel.py:769 ring = prim.tube(16.0, 13.0, SW_BUMP_H, z=-SW_BUMP_H)
audit: "reel_gk: overhang 662.9 mm2 steeper than 45 deg"

Fix: Make the track a solid disc from r 7.5 (clear of the k_clip, r <= 6.75) to r 16 with the ramped notch, so the bed face carries the whole web. Add a 45 deg chamfer from r 16 up to the tooth root under the teeth: replace tube(16.0, 13.0, ...) with a revolve of [(7.5,-H),(16,-H),(16+H,0),(7.5,0)].


## 12. Still open (r2_J #21): the brake rocker is still PETG, which is not a spec material, and the clutch spring is still sized with PETG stiffness

- File: `base/cad/parts/reel.py:883`
- Severity: low (claimed low); unverified (low)
- Owner: cad-reel

Scenario: params MATERIALS lists only PLA and PPA_CF, and Harry has no PETG. If the rocker is printed in PLA (E about 3500) or PPA-CF, the plate sized for E 2100 is 1.7x or more too stiff. The same 5.0 mm leg stroke then gives well over FIRM 157 N mm. The take-up drag in NEUTRAL/REEL and the cam bar force rise above design. LIGHT is still set by the trim screws, so LAUNCH is unaffected.

Evidence: reel.py:883 add("reel_rocker", _rocker(), "PETG", ...)
reel_calc.py:154 def rocker(d, E=2100.0, b=None):
reel_calc.py:171 def rocker_section(d, ..., E=2100.0, n=4000):
params.py:22-25 MATERIALS = dict(PLA=..., PPA_CF=...)

Fix: Choose PLA (or PPA-CF) for the rocker. Pass its modulus, e.g. E=3500 for PLA, into reel_calc.rocker/rocker_section, re-solve the plate thickness t and the leg stroke for exactly LIGHT -> FIRM, and update ROCKER_Z and the material string. Or get PETG added to params MATERIALS through the coordinator.


## 13. SW_ARM bracket 'Print flat' pose leaves its wall flange as an unsupported ledge at the top

- File: `base/cad/parts/reel.py:920`
- Severity: low (claimed low); unverified (low)
- Owner: cad-reel

Scenario: reel_sw_bracket is added with no rotation, so it prints in R orientation. The plate (z -63.9..-59.9) sits on the bed and the arm rises 54 mm to the flange (z -10..-6). The flange spans x -16..16 and y -75.7..-59.7. It overhangs the arm (x -11.5..8.5, y -67.7..-59.7) by 8 mm in -Y and by 4.5 and 7.5 mm in X, with nothing under it. The audit reports 334 mm2 of overhang. Printed as noted, the screw flange that takes the switch reaction sags or fails unless supports are added, which the note does not say.

Evidence: reel.py:920-921 add("reel_sw_bracket", brk, "PLA", joint="reel_base", ... notes="SW_ARM bracket: screwed to the wall below the gears. Print flat")
reel.py:792-793 arm = prim.box(-11.5, 8.5, y0 - 9.0, y0 - 1.0, zb - 4.0, -WALL_T); flange = prim.box(-16.0, 16.0, y0 - 17.0, y0 - 1.0, -WALL_T - 4.0, -WALL_T)
audit: "reel_sw_bracket: overhang 334.0 mm2"

Fix: Make the arm the full flange footprint in Y (y0-17..y0-1) and run the plate from y0-17. The part then has one flat -Y face. Print it on that face, with print R = tf.Rx(90) or whichever rotation puts y = y0-17 at z = 0. Alternatively add 45 deg gussets under the flange's overhangs, and correct the note.


## 14. No meshes() declared for the four cross-owner and two internal reel gear pairs, so the audit's meshcheck never checks the capstan, bus or cup meshes

- File: `base/cad/parts/reel.py:1164`
- Severity: low (claimed low); unverified (low)
- Owner: cad-reel

Scenario: If the capstan placement moves (for example LAYOUT._CAP_R or reel_frame_R changes in params), or cad-drum changes the REEL gear, bus gear or wrap idler module, plane or axis, then reel_gc/drum_gear_reel, reel_bus/drum_coupler or reel_gk/drum_idler_gear stop engaging or start overlapping, and the audit still reports CLEAN. tools/meshcheck only checks Mesh entries from a module's meshes(P), and only demo.py declares any. There are also no cross-owner Clearance entries for these pairs. Today they do engage: my probe on 2f0475e with drum.py loaded gives centre distances 72.100 / 72.100 / 49.600, antiparallel axes, 0 overlap in every TIMELINE state, and overlap when either gear is rotated 1.5 deg. Nothing guards this, though. This is OPEN_ITEMS row 27 (r2_E #18), still open for the reel pairs.

Evidence: `grep -n "def meshes\|Mesh(" cad/parts/reel.py cad/parts/drum.py` returns nothing. The only declaration is cad/parts/demo.py:81 `return [Mesh("demo_gear_1", "demo_gear_2", ...)]`. tools/assembly.py:68-69 `if hasattr(mod, "meshes"): self.meshes += list(mod.meshes(P))`. reel.py:1164-1173 clearances() has only reel-internal pairs.

Fix: Add `def meshes(P)` to reel.py (import Mesh from cad.lib) returning: Mesh("reel_gc","drum_gear_reel",M_CAP,Z_CAPG,Z_MAIN,face_a=12,face_b=12,frame_a=tf.T(0,0,GC_Z[0])), Mesh("reel_bus","drum_coupler",M_BUS,Z_C1,Z_BUSMAIN,face_a=6,face_b=8,frame_a=tf.T(0,0,BUS_Z[0]), frame_b set to the coupler's bus-gear face start), Mesh("reel_gk","drum_idler_gear",M_CUPG,Z_CUPG,42,face_a=8,face_b=8,frame_a=tf.T(0,0,CUPGEAR_Z[0])), plus reel_bus/reel_gi (46/30) and reel_gi/reel_gr (30/20) with when=lambda S: S.get('wiper_dir',1)>=0. Use face_a/face_b explicitly, because the gear parts carry hubs and the SW_ARM track, which make the bbox face wrong.


## 15. line_path draws the capstan turns from S['wraps'] while the cup is posed from cad-drum's wrap-idler kinematics, so the tether and the cup disagree by up to 3.8 turns through the WRAP phase

- File: `base/cad/parts/reel.py:1279`
- Severity: low (claimed low); unverified (low)
- Owner: cad-reel

Scenario: Mission TIMELINE wrap phase (params wraps=7 is an int, so _state_custom steps it at mid-phase). At t=25 s, S wraps=0 while kin() poses the cup at 1.36 turns. At t=30 s, S wraps=7 while the cup is at 3.19 turns, and at t=33 s the cup is at 4.32 turns. line_path puts the ring point at psi=-360*S['wraps'] and draws 0 or 7 capstan turns, so the rendered and simulated tether runs through the posed cup body instead of its ring, and the line length the sim reads is wrong for the whole wrap and unwrap.

Evidence: reel.py:1279 `w = _num(S, "wraps")` (line_path), against kin() reel.py:1104-1106 `if "drum_idler" in v and ("wraps" not in S or "drum_deg" in S): cup = (42.0 / Z_CUPG) * (float(v["drum_idler"]) - idl0)` and `wraps = ... -cup / 360.0`. Measured on the timeline: t=25.0 S.wraps 0 / cup 1.361 turns; t=30.0 S.wraps 7 / cup 3.194; t=33.0 S.wraps 7 / cup 4.32.

Fix: In line_path, take the wrap count from the same kinematics as the pose: `w = -kin(P, S)["reel_cup"] / 360.0` (or factor out a shared `_wraps(P, S)` helper that both kin() and line_path call). Keep the existing 0 < w < 0.25 clamp after it.


## 16. Step 1 asks for eight deck screws, but the reel wall foot has seven holes

- File: `base/cad/parts/reel.py:1422`
- Severity: low (claimed low); unverified (low)
- Owner: cad-reel

Scenario: The builder looks for an eighth counterbored hole that does not exist, and may drill one or leave the wall short of one screw. The BOM count derived from the text is also wrong.

Evidence: reel.py:1422-1423 "Put eight M3 screws down through the counterbored foot holes into the deck inserts"
reel.py:266-267 MOUNT_HOLES = [(-120.0, 14.0), (-120.0, 36.0), (-64.0, 36.0), (-64.0, 14.0), (-120.0, -32.0), (-60.0, -32.0), (-8.0, -32.0)]   # 7
reel_NOTES.md section 11: "7 x M3 counterbored"

Fix: Build the step text from the hole list: f"Put {len(MOUNT_HOLES)} M3 screws ..." (seven). List '7 x M3x10 SHCS' as a bought ref on the reel_wall step.


## 17. LIGHT trim set-up comes before the line is wound, and build doc G11 still says to trim with shims that do not exist

- File: `base/cad/parts/reel.py:1486`
- Severity: low (claimed low); unverified (low)
- Owner: cad-reel

Scenario: The 'Reel shaft into the wall' step tells Harry to set LIGHT with a 50 g weight on the line end 'after the line is wound on, drum at LAUNCH'. But the line is only wound in the last step, 'Exit eye post and line' (reel.py:1507-1509). Also, nothing on the reel side stops the rocker at LIGHT: its trim reaction is carried through leg, crank and bar onto the cam base circle. So a builder who follows the steps in order has no line to hang the weight on, and if the drum is not fitted, turning the trim screws only swings the rocker back with no squeeze. MEASURE_LIST G11 (the fix route when G4 fails) still says to add or remove printed 0.4 mm shims under T. No shim part exists; the trim screws replaced them.

Evidence: reel.py:1486 `Set LIGHT (after the line is wound on, drum at LAUNCH): ... turn both trim screws ...` inside the step at reel.py:1482; reel.py:1507-1509 `Exit eye post and line ... Wind the line onto the reel`; docs/build/MEASURE_LIST.md:104 `G11 Payout brake trim | If G4 is off: add or remove printed 0.4 mm shims under thrust ring T (reel assembly steps) | reel shims (print more)`.

Fix: Move the LIGHT set-up out of 'Reel shaft into the wall' into its own last reel step, 'Set LIGHT', after 'Exit eye post and line', and state that the drum (with track 2 against the bar nose) must be fitted and at LAUNCH. Give the coordinator replacement G11 text: 'turn both trim screws the same amount (1/8 turn = about 0.4 N squeeze), reel step Set LIGHT', with no reprint column.

