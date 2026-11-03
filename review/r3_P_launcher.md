# Round 3 P: launcher subsystem on the w/cad-launch tip

Reviewed at base-v1 commit 4ac70ba. Method: independent finder subagents, one per lens; every finding of
medium severity or above was checked by two skeptic subagents told to refute it; kept only if fewer than
two refuted it. Low findings are listed unverified. Severity is the finder's, lowered when both skeptics
agreed it is real but rated it lower.

Confirmed: 16. Refuted and dropped: 1.

| # | Sev | File:line | Owner | Finding |
|---|---|---|---|---|
| 1 | high | `base/cad/parts/launch.py:513` | cad-launch | Carriage yoke spring pegs: hollow PLA d6 cantilevers loaded across the layers, and the yoke prints standing on the four peg tips |
| 2 | high | `base/cad/parts/launch.py:578` | cad-launch | Drive gear key rib for sprag #1 is 2.8 deep, but the CSK20PP outer keyway is only 1.5 deep, so the sprag cannot be fitted (open item, still failing) |
| 3 | high | `base/cad/parts/launch.py:724` | cad-launch | SW_HOME and SW_SEATED lever contacts cannot open: the leaf is printed in its pressed position and the screw fixes the head-to-tip distance |
| 4 | medium | `base/cad/parts/launch.py:513` | cad-launch | Carriage spring pegs are hollow d6 PLA cantilevers printed upright with no hook locator: they bend across the layer lines at full cock |
| 5 | medium | `base/cad/parts/launch.py:620` | cad-launch | Sleeve key rib is line-to-line with sprag #1's inner keyway (6.0 in 6.0, crest exactly on the keyway floor) |
| 6 | medium | `base/cad/parts/launch.py:872` | cad-launch | Drive gear still prints on its thin collar, with the sprag hub and the whole tooth rim in the air (open item r2_E #13) |
| 7 | medium | `base/cad/parts/launch.py:1309` | cad-launch | Mid spring guards fitted in Launcher 2b block the hex-key path to the stop-frame rear-foot screws fitted later in Launcher 6 |
| 8 | medium | `base/cad/parts/launch_aim.py:53` | cad-launch | Aim train is still weaker than the 5 N m shear pin: bevel SF 0.44, so the only protection is the delayed 3 A trip (open item r2_K #5) |
| 9 | medium | `base/cad/parts/launch_aim.py:294` | cad-launch | Bevel z30 tooth ring prints over air: the backing disc is r18 but the teeth reach r26.3, 2.65 mm above the bed |
| 10 | medium | `base/cad/parts/launch_stop.py:180` | cad-launch | Stop-unit draw screws (M3x20) run 0.8 mm through the anchor into the last flexure leaf when the anchor is drawn home |
| 11 | low | `base/cad/parts/launch.py:583` | cad-launch | The drive gear's sprag-seating lip reaches r 13.4, so it bears on the CSK20 inner race face and rubs it in freewheel |
| 12 | low | `base/cad/parts/launch.py:629` | cad-launch | No inboard axial stop on the sleeve and dog-ring unit: it can slide 9.5 mm toward the drum out of the D23 bushing (open item, still failing) |
| 13 | low | `base/cad/parts/launch.py:1321` | cad-launch | Launcher 4 calls for M3 x 10 at the D37 flange, but the screw reaches the floor of cad-drum's blind tapped hole before its head seats |
| 14 | low | `base/cad/parts/launch.py:1348` | cad-launch | Launcher 6 gives the stop preload as 4.4 mm, but the anchor is printed 6.56 mm short |
| 15 | low | `base/cad/parts/launch.py:1381` | cad-launch | Aim 1 tells the builder to mount the AS5600 and SW_RAIL_LOW on housing features that were removed |
| 16 | low | `base/cad/parts/launch_aim.py:592` | cad-launch | Aim worm part prints standing on a d8 shaft stub, with its lower end face and every lower thread flank in the air |

## 1. Carriage yoke spring pegs: hollow PLA d6 cantilevers loaded across the layers, and the yoke prints standing on the four peg tips

- File: `base/cad/parts/launch.py:513`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: At full cock each spring pulls 40.1 N (F0 7 + 0.125 x 264.7; launch_kin.spring_force gives 80.2 N for both). The yoke's print_tf=best_bed(yk) puts w up. The pegs point down (w 51..74.2), so the part stands on the four d9 keeper flanges: bed contact 225.8 mm2, and the arm undersides are a 900 mm2 ledge 24 mm above the bed (scratch /tmp/claude-0/review-scratch-P/orient.py and slice2.py). That orientation also makes every peg bend ACROSS its layers. At line 526 each peg is now bored d3, so Z = pi(6^4-3^4)/(32x6) = 19.9 mm3. With the hook at w 60 the lever to the arm bottom (a0 73.2) is 13.2 mm: M = 529 N mm, sigma = 26.6 MPa. The hook ring is sized for the d8 anchor pin (inner r 4.05, hook_solid) on a d6 peg, so it can slide down to the flange: lever about 21.5 mm, 43 MPa. Both are above PLA interlayer strength (the file uses 30 MPa as the in-layer limit). The 'optional d3 steel pin' named in the comment is not a part, has no bought_ref, and no step mentions it. The result is a peg breaking at layer lines on the first cocks, which releases one spring at 40 N inside the guard and leaves the carriage pulled by one side (mission loss, hardware damage). This is open item r2_E #12, still open and made worse by the bore.

Evidence: launch.py:513-514 `s.append(cyl_w(3.0, n["w_spring"] - 9, a0 + 1, up, sg * yy))` / `cyl_w(4.5, n["w_spring"] - 11, n["w_spring"] - 9, ...)  # keeper flange`; launch.py:524-526 `# spring pegs hollow (d3 blind from the flange end): lighter, and an optional / d3 steel pin pressed in carries the hook load across the layers` `y = y - cyl_w(1.5, n["w_spring"] - 12.0, a0 + 4.0, up, sg * yy)`; launch.py:851-852 `Part("launch_carriage_yoke", yk, "PLA", ... print_tf=best_bed(yk)`; measured: yoke bed 225.8 mm2 (4 flange discs at z 0), overhang 899.6 mm2 at z 24.

Fix: Make the d3 steel pin mandatory: add a bought part per peg (4 x 'd3 x 24 steel dowel / cut 3 mm rod', bought_ref) pressed into the existing d3 bore, run the bore the full peg length, and list the pins in the spring step. Add a groove at w 60 that locates the hook so the lever stays 13 mm. Choose a yoke print_tf that does not stand on the peg tips, or build the pegs as the steel pins alone. Re-check the stress on the steel pin: 529 N mm on d3 gives Z 2.65 mm3, 200 MPa, so use a hardened dowel or d4.

> Skeptic (confirmed): The claim cites code from the wrong branch, but the defect is still real on the branch under review.

What the claim gets wrong: the cited lines (513/526, the 'spring pegs hollow' d3 blind bore and the 'optional d3 steel pin' comment, print_tf at 851) exist only on origin/w/cad-launch (launch.py:512-526, 851). They are not on the reviewed branch w/review (HEAD a4a19c4, clean tree), and not on origin/base-v1 either. On w/review the pegs are solid. They are built in carriage_yoke_solid at base/cad/parts/launch.py:412-413: d6 peg from w_spring-9 = 51 to a0+1 = 74.2, plus a d9 keeper flange at w 4
>
> Skeptic (confirmed): Several of the claim's details are wrong for the reviewed branch, but the core defect is real.

What is wrong: w/review matches origin/base-v1, and on it launch.py has no "spring pegs hollow" comment, no d3 blind bore and no "optional d3 steel pin". That text exists only on origin/w/cad-launch (I grepped every branch). The cited lines 513/524-526/851 are also from that branch; the real ones are 411-413 and 739-742. hook_solid's ring inner radius is 3.0+0.05 = 3.05, sized for a d6 peg, not a d8 one. Only the two pegs at y +-38 carry springs; the +-56 pegs are D5 provision pegs.

What is right: 
>

## 2. Drive gear key rib for sprag #1 is 2.8 deep, but the CSK20PP outer keyway is only 1.5 deep, so the sprag cannot be fitted (open item, still failing)

- File: `base/cad/parts/launch.py:578`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: Builder does Launcher 4 and slides the CSK20PP into the drive gear. The integral rib's crest is at r = 23.5 - 2.8 = 20.7. The bearing's outer keyway floor is at r 22.0 (bearings.py CSK20_OKEY_D = 1.5), or 22.2 with the library's CSK20_KEY_RCLEAR. So the rib runs 1.3 mm into the steel outer race and the sprag will not go in. With no sprag fitted there is no cocking load path. I checked this by intersecting drive_gear_solid(P) with cad.lib.bearings.bearing_csk20() (keyway lined up on +X, z -7..7): 105.56 mm3 of solid overlap, which is 1.3 x 5.8 x 14. The assembly audit cannot see it because launch_sprag is a plain tube and Allow('launch_sprag','launch_drive_gear') covers the pair.

Evidence: launch.py:578  g = g + prim.box(sp["od"] / 2 - 2.8, sp["od"] / 2 + 0.1, -sp["key"] / 2 + 0.1, sp["key"] / 2 - 0.1, -sp["w"] / 2, sp["w"] / 2)
cad/lib/bearings.py:35  CSK20_OKEY_W, CSK20_OKEY_D = 6.0, 1.5     # outer keyway width, depth from the OD
cad/lib/bearings.py:39  CSK20_KEY_RCLEAR = 0.2
scratch: (drive_gear_solid ^ bearing_csk20(0,0).translate((0,0,7))).volume() = 105.56

Fix: Size the rib from the library constants: `from cad.lib import bearings as BR` and make the box run radially from `sp["od"]/2 - (BR.CSK20_OKEY_D - BR.CSK20_KEY_RCLEAR)` (r 22.2) to `sp["od"]/2 + 0.1`, keeping width 5.8. Add a test that `(drive_gear_solid(P) ^ bearing_csk20().translate((0,0,7))).volume() < 1e-3`.

> Skeptic (confirmed): The defect is real, but the line number in the claim is wrong. The cited statement is at base/cad/parts/launch.py:474 in drive_gear_solid, not at line 578, and the text matches the claim exactly. The rib box runs radially from sp["od"]/2 - 2.8 = 23.5 - 2.8 = 20.7 to 23.6, and it is 5.8 wide over the full 14 mm sprag width. P.SPRAG in spec/params.py gives od=47 and key=6, so nothing overrides these values.

The 2.8 depth is the CSK20PP inner keyway depth (bearings.py CSK20_IKEY_D = 2.8). It is also the depth used for the sleeve rib (launch.py:516, RIB_R 12.8, commented "sprag keyway depth 2.8")
>
> Skeptic (confirmed): I could not refute this. The defect is real; only the line number in the claim is wrong. The code is at base/cad/parts/launch.py:474 (in drive_gear_solid), not :578, but the text is exactly as quoted: `g = g + prim.box(sp["od"]/2 - 2.8, sp["od"]/2 + 0.1, -sp["key"]/2 + 0.1, sp["key"]/2 - 0.1, -sp["w"]/2, sp["w"]/2)`. That puts the rib crest at r 20.7. The 2.8 is the inner keyway depth, CSK20_IKEY_D. The repo's own model of the bought part has a much shallower outer keyway: base/cad/lib/bearings.py:35 sets CSK20_OKEY_D = 1.5, which puts the floor at r 22.0. bearing_csk20() cuts the outer keyway
>

## 3. SW_HOME and SW_SEATED lever contacts cannot open: the leaf is printed in its pressed position and the screw fixes the head-to-tip distance

- File: `base/cad/parts/launch.py:724`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: Both levers are built as in Launcher 7. The modelled pressed geometry is also the printed geometry: the leaf bottom is at w_head1 = 65.6, the head bottom (screw fully in) at 62.6, and the nominal nut top at 62.6. The screw (6 mm shank plus 3 mm head) is rigid, so with the foot on the tip the head bottom always sits at w_tip - 9 = 62.6. The step's screw adjustment only moves the leaf, never the head relative to the tip. The nut top ends up at 62.4 + t_wire (pocket floor 60.0, nut 2.4, wire under the nut). (a) If t_wire >= 0.2, the head touches the nut at home. With the carriage away, the relaxed leaf holds the head at 62.6 - b, where b >= 0 is the back-off, so the head is still on the nut and the switch stays CLOSED. If t_wire > 0.2, the rigid foot also jams on the screw. (b) If t_wire < 0.2, the switch never closes. So no setting gives 'closes at home, opens 10 mm back'. Firmware base_logic.h:527 then raises F_SWITCH_DISAGREE ('SW_HOME still closed past the first tooth') on every cock, or keeps the phase UNKNOWN at rest (lines 361 and 805), so the launcher cannot shoot. SW_SEATED has the same geometry and reads seated with no dart, which defeats the dry-fire interlock (launch_NOTES.md:100: 12 J into the stop).

Evidence: launch.py:724-729 g["w_tip"] = n["arm_w0"] - 1.6 ... g["w_head1"] = g["w_tip"] - 6.0 ... g["w_head0"] = g["w_head1"] - SW.HEAD_H; g["w_nut0"] = g["w_head0"] - SW.NUT_T; g["leaf_w0"] = g["w_head1"]. launch.py:739 leaf = B(..., st["leaf_w0"], st["leaf_w1"]) (straight leaf at the pressed height, no pre-bend). launch.py:312 nut pocket from w_nut0 - 0.2. launch.py:1352-1356 'drop an M3 nut into the slot ... with a stripped wire end under it ... set the screw so the head just touches the nut, then back it off a quarter turn and check it closes with the carriage home and opens 10 mm back'. Scratch check /tmp/claude-0/review-scratch-P/sw.py: w_tip 71.6, w_head1 65.6, w_head0 62.6, w_nut0 60.2, leaf_w0 65.6, pocket floor 60.0 for both sides.

Fix: Print the relaxed leaf TRAVEL (about 1.0 mm) higher than the pressed state, so the leaf root and leaf sit at leaf_w0 + 1.0, and make the screw adjustment control the free gap. One way is to thread the screw down through the leaf with the head on top and the TIP touching the nut, with a jam nut, so turning it sets the tip-to-nut gap (about 0.4 mm free). The other is a compliant foot (a printed flex tongue on the yoke) that gives about 0.5 mm of overtravel. Then rewrite the Launcher 7 text to match: free gap 0.4 mm, set with the carriage 10 mm back, check closed at home. Apply the same change to the SW_SEATED lever and the plunger.

> Skeptic (confirmed): I could not refute this. The cited line numbers are wrong (the code is at seat_geom, launch.py:602-620; seat_lever_solid at 623-632; the assembly text at 1188-1192, not 724/1352), but the geometry is exactly as claimed. I ran seat_geom from the repo. Both sides give w_tip 71.6, w_head1 = leaf_w0 65.6, w_head0 62.6, w_nut0 60.2 and leaf_w1 66.8. The nut pocket floor is w_nut0 - 0.2 = 60.0 (line 307). lip_w1 is 64.125, so the leaf is a straight cantilever printed at the modelled pressed height. It has no pre-bend, and no shim or root offset lifts it.

The screw is an M3x6 SHCS with its head belo
>
> Skeptic (confirmed): The defect is real. On w/review, seat_geom is at launch.py:606-621, not 724. It sets w_tip = arm_w0 - 1.6 = 71.6, w_head1 = w_tip - 6 = 65.6, w_head0 = 62.6, w_nut0 = 60.2, and leaf_w0 = w_head1. seat_lever_solid (624-632) prints a straight leaf from leaf_w0 to leaf_w1. Its root block sits on lip_w1 at a fixed height, with no pre-bend, shim or root height adjustment. The nut pocket floor is w_nut0 - 0.2 = 60.0 (line 307), so the nut top sits at 62.4 + t_wire.

Why no setting works: the screw goes up through the tapped leaf with the head underneath. Call s >= 0 the gap from the head top down to
>

## 4. Carriage spring pegs are hollow d6 PLA cantilevers printed upright with no hook locator: they bend across the layer lines at full cock

- File: `base/cad/parts/launch.py:513`
- Severity: medium (claimed high); 1 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: The yoke prints w-up: best_bed returns identity (scratch yoke.py: local w maps to print z), so each peg axis is the print z axis. At full cock (D30: 265 mm extension, about 7 + 0.125 x 265 = 40 N per spring), the hook pulls the peg along u. The peg is a tube (d6 outside, d3 bore from line 524-525, Z = 19.9 mm3). The hook can rest anywhere between the keeper flange (w 51) and the arm root (a0 = 73.2). At w 60 the lever is 13.2 mm, giving 26.6 MPa. With the hook on the flange the lever is about 21.5 mm, giving about 43 MPa. Both stresses are tension across the layer lines, at or above PLA interlayer strength. The peg snaps while the carriage is held cocked, and the spring and its hook are released inside the guards. The 'optional d3 steel pin' is not in the Launcher 1 hardware list or in any step. OPEN_ITEMS r2_E #12 is still open.

Evidence: launch.py:513 s.append(cyl_w(3.0, n["w_spring"] - 9, a0 + 1, up, sg * yy)); launch.py:514 keeper flange cyl_w(4.5, w_spring - 11, w_spring - 9) (only one flange, no groove at w 60); launch.py:524-526 '# spring pegs hollow (d3 blind from the flange end): lighter, and an optional d3 steel pin pressed in carries the hook load across the layers' y = y - cyl_w(1.5, ...); launch.py:852 print_tf=best_bed(yk); launch.py:1301 hardware list: '2 x 8 mm steel rod pins ... for the spring anchors, 4 x 693ZZ ...', with no peg pins.

Fix: Make the pin mandatory and size it for the load. Bore the peg to d4.1 and press in a d4 steel rod (or an M4 x 30 screw): Z 6.3 mm3, about 95 MPa at a 15 mm lever. Add a second printed collar (r 4.5) at w_spring + 3, so the hook stays between w 57 and 63 instead of sliding down to the flange. List '4 x d4 x 30 steel pin' in the Launcher 1 hardware (2 used, 2 for the D5 pegs), and add 'press the steel pin into each yoke peg' to Launcher 5.

> Skeptic (refuted): The cited code is not in the tree under review. In the working tree (w/review HEAD a4a19c4), carriage_yoke_solid is at base/cad/parts/launch.py:393-430. The pegs are lines 412-413: a d6 cylinder from w_spring-9 to a0+1 with one r4.5 keeper flange. Nothing bores them. The comment 'spring pegs hollow (d3 blind from the flange end) ... optional d3 steel pin' and the cyl_w(1.5, ...) subtraction appear nowhere in HEAD or origin/base-v1. `git log -S` shows they exist only on origin/w/cad-launch (commit e30c6ad, 'carriage lightening checkpoint'), which is a different branch. The line numbers in the c
>
> Skeptic (confirmed): Several of the claim's details are wrong for this branch (w/review at a4a19c4, and also origin/base-v1), but the main defect is real.

What is wrong in the claim:
- launch.py is 1304 lines long. Line 513 is not the peg code.
- The pegs are not hollow. carriage_yoke_solid (launch.py:401-428) has no d3 blind bore, no 'optional d3 steel pin' comment, and no cyl_w(1.5, ...) cut. `git grep "hollow\|optional d3"` finds nothing in launch.py, on this branch or on origin/base-v1.
- The quoted hardware text ('2 x 8 mm steel rod pins ... 4 x 693ZZ') is not there either. The actual line is 1141: '2 x 6 mm
>

## 5. Sleeve key rib is line-to-line with sprag #1's inner keyway (6.0 in 6.0, crest exactly on the keyway floor)

- File: `base/cad/parts/launch.py:620`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: Launcher 4: push the sleeve through the sprag's steel inner keyway. The 12 mm sun rib goes through first, then the 15 mm sprag rib settles in the keyway. The printed PPA-CF rib is exactly 6.0 wide with its crest at r 12.8, which is the nominal 6 x 2.8 keyway floor, so there is zero clearance on width and on depth. Any print growth (typically +0.1 to 0.2 mm) stops the sleeve, and the builder ends up filing or forcing the rib that carries the cocking torque. The repo's own fit contract asks for 0.2 per side (fits.KEY_CLEAR) and 0.2 radial (CSK20_KEY_RCLEAR). The splitter sun on the same rib already follows it with a 6.4 x 13.2 keyway. Scratch check: sleeve_solid ^ bearing_csk20 placed on the rib = 0.044 mm3, i.e. touching.

Evidence: launch.py:620  RIB_W, RIB_R = 6.0, 12.8      # integral key rib on the sleeve (D13): 6 wide, top at r 12.8 (sprag keyway depth 2.8)
launch.py:634  s = s + prim.box(9.0, RIB_R, -RIB_W / 2, RIB_W / 2, zg - 7.5, zg + 7.5)
cad/lib/fits.py:16  KEY_CLEAR = 0.20         # key in keyway, each side
cad/lib/bearings.py:34  CSK20_IKEY_W, CSK20_IKEY_D = 6.0, 2.8
launch_aim.py:185  keyway for the sleeve's integral rib: prim.box(9.0, 13.2, -3.2, 3.2, ...)

Fix: Set RIB_W = CSK20_IKEY_W - 2 * fits.KEY_CLEAR (5.6) and RIB_R = 10 + CSK20_IKEY_D - CSK20_KEY_RCLEAR (12.6). The cheek slots (6.6 wide, to r 13.3), the washer and collar bores (r 13.4) and the sun keyway (6.4 / 13.2) all still clear it, and the D13 wall over the spigot is unchanged.

> Skeptic (confirmed): I could not refute this. The cited line number is wrong, but the code says what the claim says. On w/review (a4a19c4) the line is base/cad/parts/launch.py:516, not 620: `RIB_W, RIB_R = 6.0, 12.8`. Lines 530-531 build both rib lengths as `prim.box(9.0, RIB_R, -RIB_W/2, RIB_W/2, ...)`, so each rib has flat sides at y = +-3.0 and a flat crest at x = 12.8.

The sprag's inner keyway in base/cad/lib/bearings.py:34 and :173 is `prim.box(0, CSK20_D/2 + CSK20_IKEY_D, -CSK20_IKEY_W/2, CSK20_IKEY_W/2, ...)`. That puts it at y = +-3.0 with its floor at x = 10 + 2.8 = 12.8. The rib therefore meets the keyw
>
> Skeptic (confirmed): The defect is real. The line number is off: in the w/review tree the definition is at base/cad/parts/launch.py:516, not 620, but the content matches the claim exactly. `RIB_W, RIB_R = 6.0, 12.8`, and sleeve_solid adds `prim.box(9.0, RIB_R, -RIB_W/2, RIB_W/2, zg-7.5, zg+7.5)` under sprag #1 (line 530). The modelled sprag keyway in cad/lib/bearings.py:173 is `box(0, CSK20_D/2 + CSK20_IKEY_D, -CSK20_IKEY_W/2, ...)`, which is 6.0 wide with its floor at r 12.8. So the rib is nominally line-to-line on both width and depth. No other code path adds clearance: the cheek slot (line 287) and the sun keyw
>

## 6. Drive gear still prints on its thin collar, with the sprag hub and the whole tooth rim in the air (open item r2_E #13)

- File: `base/cad/parts/launch.py:872`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: drive_gear_solid has a symmetric 16 mm hub on a 7 mm face, plus an integral collar tube (r 16 / 13.4) that runs down to local z -19.8. With print_tf=bed(dg) unrotated, only that collar touches the bed: 240.1 mm2 under a 70 x 70 x 27.8 PPA-CF gear. The hub underside (r 16..27.5, which includes the 0.8 mm sprag seating lip) is a 1571 mm2 ledge at z about 12. The tooth rim underside (r 27.5..35) is an 878 mm2 ledge at z about 16. The 26T teeth that carry the cocking torque therefore start on a sagging layer 16 mm above the bed, and the sprag seat lip is printed over air.

Evidence: launch.py:575 `hub = prim.cyl(sp["od"] / 2 + 4.0, sp["w"] + 2.0, z=-(sp["w"] + 2.0) / 2)`; launch.py:582 `g = g + prim.tube(16.0, 13.4, -(sp["w"] + 2.0) / 2 - za + 0.01, z=za)`; launch.py:871-872 `Part("launch_drive_gear", ... print_tf=bed(dg)`; measured: bed 240.1 mm2, OVH 1571.3 mm2 at z~12 (r 16..27.5), 878.4 mm2 at z~16 (r 27.5..35).

Fix: Move the integral collar out into a separate printed washer, the same way launch_drive_washer is already done for the other side. Make the hub flush with one tooth face (hub z from -fw/2 to -fw/2 + 16) and keep the sprag lip on that face. Then set print_tf=bed(dg, R) so that flush face, with the teeth, hub and lip, sits on the bed.

> Skeptic (confirmed): The defect is real. The line numbers are wrong but the code they point at is correct. In the working tree the code is at launch.py:471 (hub), :478 (collar tube), :479 (lip) and :760-761 (Part with print_tf=bed(dg)), not 575/582/872. drive_gear_solid builds the 26T face (fw = 7, from y_full 7.5..14.5) and a symmetric hub cyl(27.5, 16) centred on z=0. It also adds tube(16, 13.4) running down to za = (y_chA[1]+0.2) - ym = -19.8, and a 0.8 mm lip (r 23.55..13.4) at z=-8. bed() in launch.py:189 is called with R=None, so the part is only translated and stays unrotated.

I ran a scratch script (/tmp/
>
> Skeptic (confirmed): The defect is real. The claim's line numbers are wrong: in this checkout the hub is at launch.py:471, the collar at :476, the lip at :477 and the Part/print_tf at :759-760, not 575/582/872. The quoted code matches word for word, though.

The geometry, using params from launch.py:119-122:
- y_full is (7.5, 14.5), so the face width fw is 7, not 16.
- The hub is cyl(27.5, 16) centred on z=0 (z -8..8).
- ym = 11 and za = (-9+0.2) - 11 = -19.8, so the collar tube (r 16/13.4) runs from z=-19.8 up to the hub underside.
- The lip (r 23.6/13.4, 0.8 thick) sits at the hub's underside, z -8..-7.2.
- prin
>

## 7. Mid spring guards fitted in Launcher 2b block the hex-key path to the stop-frame rear-foot screws fitted later in Launcher 6

- File: `base/cad/parts/launch.py:1309`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: Launcher 2b screws guard segment b (|y| 66..68, w 37..69 for u > 78) to the rail before the stop units. Launcher 6 then has to drive 2 x M3x12 along y through the rear foot (u_stop + 6 at w 44 and u_stop + 12 at w 38.5) into the rail wall. The stop frame provides a d7 hex-key access hole through its leg (|y| 71..81) for exactly this path, but guard b has no hole there: a key ray from the screw head outwards hits the guard plate on both sides. The builder cannot drive the screws as the steps are ordered and has to remove the mid guards again.

Evidence: launch.py:1309-1313 'Launcher 2b ... Screw the spring guard plates ... three per side: rear, middle (under the stop units), front'. launch.py:1344-1347 'Launcher 6 ... 2 x M3 x 12 through the rear foot into the rail wall'. launch_stop.py:191 REAR_BOLTS = ((6.0, 44.0), (12.0, 38.5)); launch_stop.py:213 hex key access cyl_y(3.5, sg*71, sg*81, ...). launch_guard.py:92 seg b plate _bx(u0, u1, Y_IN, Y_OUT, W_BOT, W_TOP_STOP); launch_guard.py:106 access holes only at its own bracket u. Scratch /tmp/claude-0/review-scratch-P/guard.py: intersection volume of ray and guard 9.71 mm3 for both bolts on both sides; ray and frame 0.

Fix: Either cut d6.4 hex-key holes in guard segment b at (u_stop + 6, w 44) and (u_stop + 12, w 38.5) on both sides (add them to segment_solid next to the bracket access holes), or move guard segment b out of Launcher 2b into Launcher 6, after the stop frames are bolted ('then fit the middle guard plates').

> Skeptic (confirmed): I could not refute this. It holds on the target tree (origin/base-v1). The cited line numbers match origin/w/cad-launch, and the local w/review launch.py has no Launcher 2b or launch_guard import at all, but the geometry is the same on base-v1.

Order of steps (base-v1 launch.py ~1251 and ~1290): "Launcher 2b: splice straps and spring guards" screws all three guard segments per side, middle one included ("under the stop units"). "Launcher 6: carriage stops" comes later. In it the stop spring goes on first, then the frame with "2 x M3 x 12 through the rear foot into the rail wall".

Geometry (l
>
> Skeptic (confirmed): I could not refute this. One caveat about where the code lives: the local w/review checkout does not contain this code. Its launch.py is 1304 lines long, has no 'Launcher 2b' step and never imports launch_guard. The cited text is in origin/w/cad-launch at launch.py:1309-1313 and in origin/base-v1 at about line 1251. launch_guard.py and launch_stop.py are byte-identical in both of those branches. I checked against those versions, saved under /tmp/claude-0/review-scratch-P/B/.

Order of steps: 'Launcher 2b' fits all three guard segments per side ('rear, middle (under the stop units), front ... F
>

## 8. Aim train is still weaker than the 5 N m shear pin: bevel SF 0.44, so the only protection is the delayed 3 A trip (open item r2_K #5)

- File: `base/cad/parts/launch_aim.py:53`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: At the D40 3 A trip the sleeve torque is 0.909 x (3 - 1.5) = 1.36 N m. The numbers at that torque are: sun 9.4 MPa, idler 10.9 MPa, bevel 24.6 MPa (SF 1.02 at 25 MPa), worm wheel 12.1 MPa. All are acceptable. The only mechanical limiter is the drum coupler shear pin at about 5 N m (D8/D26). At 5 N m the stresses scale x 3.67: bevel 90 MPa (SF 0.44 at 40), worm wheel 44 MPa (0.90), idler 40 MPa (1.0). Scenario: the aim train is jammed when an aim or backward jog starts (rail blocked, or a crank-pin clip off), and the motor current exceeds 3 A at once. The trip has params blank_ms 50 plus hold_ms 30, so the wiper (stall 15 N m) drives into the pin limit for up to 80 ms. The z30/z15 bevel, or the wheel, strips before the PLA pin shears, and the rail is then no longer held by the worm. The LEWIS block still sizes everything at the old 5 A trip and itself concludes that the bevel is the weak link.

Evidence: launch_aim.py:53 `M_B, Z_B1, Z_B2, FACE_B = 1.5, 30, 15, 8.0`; launch_aim.py:86 `# LEWIS (root bending at the 5 A aim trip)` ... lines 101-102 `57.4 MPa (SF 0.70 / 0.44). The bevel pair stays the weak link at the trip.`; params.py REEL `torque_limiter_nm=5.0` and `current_trips_a=dict(aim=3.0, backward_launch=3.0, ... blank_ms=50, hold_ms=30`.

Fix: Give the aim branch its own mechanical limiter below the bevel's limit: a printed PLA shear pin (or a friction slip ring) between the output countershaft and the bevel30 hub, sized to let go at 1.8 N m. That is above the 1.36 N m trip torque and below the bevel's 2.2 N m at 40 MPa. Add spares to the BOM. Update the LEWIS block to the 3 A trip and the new limiter.

> Skeptic (confirmed): The defect is real on w/review, but much of the evidence the claim cites does not exist on this tree, so it should be re-anchored before anyone acts on it.

Where the evidence is wrong:
- launch_aim.py has no LEWIS block. Lines 86-102 are the A()/_A() cache and geometry code. The bevel line `M_B, Z_B1, Z_B2, FACE_B = 1.5, 30, 15, 8.0` is around line 50, not 53.
- params.py:230 is `current_trips_a=dict(aim=5.0, reel_in=5.0, cock=6.5, tow=6.5, blank_ms=50, hold_ms=30)`. There is no aim=3.0, no backward_launch key and no decision D40.
- The claim says everything passes at the trip. On this tree t
>
> Skeptic (confirmed): I checked this against origin/base-v1. The local w/review checkout is stale: its params still have aim=5.0 and there is no D40, so a review of the local tree alone would get this wrong.

What the code says on origin/base-v1:
- launch_aim.py:53 is `M_B, Z_B1, Z_B2, FACE_B = 1.5, 30, 15, 8.0`, the same bevel pair as before.
- The LEWIS block still assumes the 5 A trip, which gives 3.18 N m at the sleeve. It finds the bevel at 57.4 MPa (SF 0.70 at 40 MPa, 0.44 at 25 MPa) and calls it "the weak link at the trip".
- params.py:234 has current_trips_a aim=3.0, backward_launch=3.0, blank_ms=50, hold_m
>

## 9. Bevel z30 tooth ring prints over air: the backing disc is r18 but the teeth reach r26.3, 2.65 mm above the bed

- File: `base/cad/parts/launch_aim.py:294`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: bevel1_solid adds a backing disc of r 18 and height back = y_b - (Y_CRANK[1] + 0.7) = 202.95 - 200.3 = 2.65 below the gear's back face. print_tf=bed(b1) puts that disc on the bed, so the whole tooth ring from r 18.0 to 26.3 (590 mm2) starts as a flat ledge 2.65 mm above the bed. The large end of each tooth, where the Lewis check puts the load (bevel factor 1.47), prints on a sagging first layer. This bevel is already the weakest gear in the aim train: at the 3 A trip, Wt = 60.6 N, sigma = 24.6 MPa, SF 1.63 at 40 MPa and 1.02 at the conservative 25 MPa. Any loss of root section at the large end therefore drops it below 1.

Evidence: launch_aim.py:293-294 `back = n["y_b"] - (Y_CRANK[1] + 0.7)` / `s = g1 + prim.cyl(18.0, back + 0.01, z=-back) + prim.cyl(8.0, 210.0 - n["y_b"], z=0)`; launch_aim.py:589-590 `Part("launch_aim_bevel30", ... print_tf=bed(b1)`; measured: OVH z~3 area 590.0 mm2 at r 18.0..26.3, bed 965.8 mm2 (the disc only).

Fix: Remove the r18 disc from bevel1_solid so the gear's flat back face (r up to 26.3) prints on the bed. Make the 2.65 mm spacer a separate PLA part, launch_aim_bevel_spacer = prim.tube(18.0, SHAFT_R + FIT/2, back), placed at y 200.3 with its own print_tf, and add it to the Aim step.

> Skeptic (confirmed): The defect is real. The cited line numbers are wrong (on this branch the code is at launch_aim.py:195-202 and 464, not 293-294 and 589-590), but the code and geometry match the claim.

- bevel1_solid builds g1 from G.bevel_pair. Its docstring (launch_gears.py:368) says the gear has a flat back face at local z=0. The function then adds prim.cyl(18.0, back+0.01, z=-back), where back = y_b - (Y_CRANK[1]+0.7) = 202.95 - 200.3 = 2.65. It also adds an r8 hub towards +z.
- print_tf=bed(b1) applies no rotation. It only shifts the part so its lowest point is at z=0, so the r18 disc bottom is the only t
>
> Skeptic (confirmed): The geometry claim holds. The cited line numbers are off (the code is at launch_aim.py:195-202 and the Part is at about line 464, not 294 and 589), but the code says what the claim says. bevel1_solid adds prim.cyl(18.0, back+0.01, z=-back) under G.bevel_pair's gear 1. launch_gears.bevel_pair (line 358) builds that gear by extruding the projected tooth profile up from a flat back face at z=0. back = y_b - (Y_CRANK[1]+0.7) = 202.95 - 200.3 = 2.65. print_tf = bed(b1) applies no rotation, so the disc bottom (z = -back) goes on the bed. I built the part with the repo's own load_params and LA.parts_
>

## 10. Stop-unit draw screws (M3x20) run 0.8 mm through the anchor into the last flexure leaf when the anchor is drawn home

- File: `base/cad/parts/launch_stop.py:180`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: Launcher 6: the head of each M3x20 draw screw bears on the front face of the frame front wall (u 248.15; the clearance hole has no counterbore), so the tip ends at u 228.15. When the anchor is drawn home onto the wall (anchor front face at u_front + 6.56 = 240.15), its rear face at y_mid +- 6 is at 222.41 + 6.56 = 228.97, with a 0.6 mm printed gap behind it to the last leaf. The tip therefore self-taps the last 3 mm through solid PLA past the 9 mm pilot, breaks out of the anchor rear face, crosses the 0.6 mm gap and presses about 0.2 mm into the last leaf near its centre block. Either the screws stall before the anchor seats, or the steel tips pre-bend the last leaf and act as point loads on it at every stop impact (148 N per side), which cracks the leaf and makes the stop softer.

Evidence: launch_stop.py:180 s = s - LA.cyl_u(1.3, lay["u_front"] - 9.0, lay["u_front"] + 1.0, ...) (9 mm pilot). launch_stop.py:218 f = f - LA.cyl_u(1.7, uF - 1.0, uF + FRAME_T + 1.0, ...) (clearance only, FRAME_T 8). launch_stop.py:278 'M3x20 draw screws through the frame front wall'. launch.py:1348 'the 2 x M3 x 20 draw screws ... until it sits on the wall'. Scratch /tmp/claude-0/review-scratch-P/stop1.py: u_front 233.59, last gap at y_mid+-6 from 221.81 to 222.41, anchor thickness there 11.18 mm.

Fix: Specify M3x16 draw screws. Seated, that is 8 mm of thread in the 9 mm pilot with the tip 0.8 mm short of the pilot floor, and 1.4 mm of engagement at the start, when the stack is unloaded. Alternatively, keep the M3x20 and add a 4 mm deep d6.5 counterbore... no: add a printed 4 mm spacer washer under each head. Change launch_stop.py:278 and launch.py:1348 to match.

> Skeptic (confirmed): I couldn't refute this. The interference is real, though the cited line numbers are off. The pilot is at launch_stop.py:174-178, not 180, and the assembly text is at launch.py:1176, not 1348.

What the code does:
- spring_solid cuts a d2.6 pilot from u_front-9 to u_front+1, so the pilot is 9 mm deep in the anchor.
- frame_solid makes the front wall run from uF to uF+8 (FRAME_T 8), with only a d3.4 clearance hole and no counterbore or spacer. So the head of each screw sits on the wall's front face at uF+8.
- The part notes and the Launcher 6 assembly text both call for M3x20 draw screws, pulled
>
> Skeptic (confirmed): The claim holds; I recomputed it from the current code. My absolute numbers differ from the claimant's (I get u_front 235.79, not 233.59), but the conclusion is the same.

**Geometry (/tmp/claude-0/review-scratch-P/stopB.py, run from base/, read-only):**
- From stop_calc: delta0 = 4.36 mm, so the wall rear face is at uF = u_front + 4.36 = 240.15. The wall is FRAME_T = 8 thick, so its front face is at 248.15.
- launch_stop.py:214 cuts only a plain d3.4 clearance hole through the wall: no counterbore, no washer or spacer.
- So the head of an M3x20 (launch_stop.py:275 and launch.py:1176 both say 
>

## 11. The drive gear's sprag-seating lip reaches r 13.4, so it bears on the CSK20 inner race face and rubs it in freewheel

- File: `base/cad/parts/launch.py:583`
- Severity: low (claimed low); unverified (low)
- Owner: cad-launch

Scenario: Sprag #1 floats axially by about 1.3 mm (between the lip face at local z -7.2 and the washer at z 8.1) and rests on the lip. The lip is an annulus from r 13.4 to 23.6. The CSK20 inner race runs to r 15 (CSK20_INNER_OD 30), so the lip touches both races. Every backward wiper turn spins the inner race on the sleeve while the drive gear stands still (the aim cycle is 3600 wiper deg). The inner race face then rubs the printed lip, which wears it and puts a drag torque on the drive gear and big gear in the backward direction while they should stay parked in the gap.

Evidence: launch.py:583  g = g + prim.tube(sp["od"] / 2 + 0.1, 13.4, 0.8, z=-(sp["w"] + 2.0) / 2)      # lip: ties the collar on, seats the sprag
cad/lib/bearings.py:32  CSK20_INNER_OD = 30.0

Fix: Make the lip touch only the outer race: inner radius at least CSK20_OUTER_ID/2 - 1 = 18.0, e.g. prim.tube(sp["od"]/2 + 0.1, 18.0, 0.8, ...). Keep the collar tie by joining the r 16 / 13.4 collar to the hub below z -8.


## 12. No inboard axial stop on the sleeve and dog-ring unit: it can slide 9.5 mm toward the drum out of the D23 bushing (open item, still failing)

- File: `base/cad/parts/launch.py:629`
- Severity: low (claimed low); unverified (low)
- Owner: cad-launch

Scenario: The sleeve is now bolted to the dog ring (D37), and the sleeve end butts the housing lip at y 193 (the outboard stop). Going inboard, the only thing that stops the unit is the dog collar, and only while it is engaged in LAUNCH. In NEUTRAL (collar y 72..98) the gap to the ring at y 100.5 is 2.5 mm; in REEL (collar 65..91) it is 9.5 mm. The sleeve is a plain d20 that slips through the plain cheek bores, the sprag inner race and the housing bushing (y 186..193), and nothing grips it axially. Example: rover on a side slope, left side high, while reeling. The unit slides 9.5 mm toward -y. The sleeve end (now y 183.5) leaves the D23 bushing completely, the sun rib (y 170..182 -> 160.5..172.5) keeps only 2.5 mm under the 8 mm sun, and the sprag rib keeps only about 5 mm under the sprag.

Evidence: launch.py:629  z_end = n["Y0"] + 53.0 - n["y_flange"]   (sleeve is a plain prim.cyl(10.0, z_end) + flange, no groove or shoulder)
launch_aim.py:462  h = h - prim.cylY(10.0 + FIT / 2, Y_W1[0] - 1, Y_W1[1], 0.0, az)   # sleeve bushing, thrust face y 193 (outboard only)
params LAYOUT stack: dog collar y (72.0, 98.0) at NEUTRAL, stroke 7 each way; dog ring y (100.5, 111.0)

Fix: Add an inboard thrust face. Cut a C-clip groove into the sleeve just inboard of the housing bushing (world y 185.0..186.0, core r 9.0, so local z 74..75), add a printed clip part (like clip_solid) that bears on the bushing's inboard face at y 186, and fit it in the Aim step after the housing goes on. Add a test that the sleeve's axial play is 0.5 mm or less.


## 13. Launcher 4 calls for M3 x 10 at the D37 flange, but the screw reaches the floor of cad-drum's blind tapped hole before its head seats

- File: `base/cad/parts/launch.py:1321`
- Severity: low (claimed low); unverified (low)
- Owner: cad-launch

Scenario: The counterbore floor is at local z 5.01, i.e. world y 116.01. An M3 x 10 tip then reaches y 106.01. I probed the posed ring along the screw axis: the tapped hole runs from y 111 down to 106.1, with a 0.6 mm floor at y 105.5..106.1. The screw bottoms about 0.1 mm before the head seats, so the flange-to-ring joint cannot be clamped (it can rattle axially), or the screw punches into the 0.6 mm floor under the dog face. Screw length tolerance (±0.29) makes it worse.

Evidence: launch.py:643  s = s - prim.cyl(3.0, 3.0, x=x, y=y, z=8.0 - 3.0 + 0.01)   (counterbore floor at flange z 5.01)
launch.py:1321  "... lugs in the ring slots, 3 x M3 x 10 from the flange side into the tapped ring"
drum.py:414  cuts.append(cyl(1.25, s0 + 0.6, s1 + 1, x, y, seg=24))     # M3 tapped from the sleeve side (blind, 4.9 deep)
scratch hole_probe.py: ring material on the screw axis only at y 105.5..106.1

Fix: Change the Launcher 4 text (and the Launcher 1 hardware list) to 3 x M3 x 8: the tip ends at y 108.0, which gives 3 mm of thread in the ring and 1.9 mm to spare before the floor.


## 14. Launcher 6 gives the stop preload as 4.4 mm, but the anchor is printed 6.56 mm short

- File: `base/cad/parts/launch.py:1348`
- Severity: low (claimed low); unverified (low)
- Owner: cad-launch

Scenario: The builder draws the anchor until 4.4 mm of travel has been taken up and stops, believing the stop is preloaded. The anchor is still 2.16 mm off the wall, so the preload is about 21 N per side instead of 32 N, and the stack no longer goes solid at the designed TRAVEL, because the gaps are sized from the 6.56 mm preload. stop_calc gives preload_mm 6.56, and launch_NOTES.md:94 also says 6.56.

Evidence: launch.py:1348-1349 'until it sits on the wall (4.4 mm of preload)'. launch_stop.py:92 delta0 = round(F0_TARGET / k_side, 2) evaluates to 6.56 (scratch run: preload_mm 6.56, preload_force_n 64.0). launch_stop.py:139 u_front = U_FW - c["preload_mm"].

Fix: Generate the number from stop_calc in the step text: '... until it sits on the wall (%.1f mm of preload)' % ST.stop_calc()['preload_mm'], which gives 6.6 mm.


## 15. Aim 1 tells the builder to mount the AS5600 and SW_RAIL_LOW on housing features that were removed

- File: `base/cad/parts/launch.py:1381`
- Severity: low (claimed low); unverified (low)
- Owner: cad-launch

Scenario: The builder follows Aim 1 and looks for 'the housing's outer bracket' for the AS5600 and a place on the housing to screw SW_RAIL_LOW. launch_aim.py removed the AS5600 arms and plate and the SW_RAIL_LOW pedestal; both now come from cad-sensors (carrier on the spine, bracket on the foot extension), as Launcher 7 itself says. The two steps contradict each other, and the bracket Aim 1 names does not exist.

Evidence: launch.py:1381-1383 'Mount the AS5600 board on the housing's outer bracket, chip facing inwards. Build SW_RAIL_LOW like SW_GAP and screw it to the housing where the crank cam presses it.' vs launch.py:1357-1358 'SW_GAP, SW_RAIL_LOW and the AS5600 carrier are fitted in the Sensors steps (cad-sensors)' and launch_aim.py:451 '(SW_RAIL_LOW pedestal tower removed: cad-sensors' bracket stands on the foot extension below)', launch_aim.py:454 '(AS5600 arms and plate removed: cad-sensors carrier screws to the spine +x face)'.

Fix: Delete those two sentences from Aim 1 and replace them with 'The AS5600 carrier and SW_RAIL_LOW bracket are fitted in Sensors 2 and 4 (cad-sensors).'


## 16. Aim worm part prints standing on a d8 shaft stub, with its lower end face and every lower thread flank in the air

- File: `base/cad/parts/launch_aim.py:592`
- Severity: low (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: launch_aim_worm (worm m2 pd24, d8 shaft and bevel z15 in one PPA-CF part) gets print_tf=bed(wm) unrotated, so the axis is vertical. Measured in print orientation: 28 x 28 x 54.5 mm on 50.1 mm2 of bed contact (the d8 shaft end under the z15 small end), a height-to-base ratio of about 7 in PPA-CF. The worm body starts at z 15 above a 7 mm gap that holds only the d8 shaft, so its lower end face (r 4..14, 435 mm2) is an unsupported ledge. Each mm of thread from z 16 to 47 adds about 74 mm2 of lower flank steeper than 45 deg; the 20 deg flank sits 70 deg from vertical. Total overhang is 2779 mm2. This is the self-locking worm that holds the rail and carries the full wheel thrust (151 N at the 3 A trip). A worm with drooped or support-scarred load flanks, or one that detaches mid-print, gives a binding or loose mesh and no reliable hold.

Evidence: launch_aim.py:307 `s = w + prim.cyl(SHAFT_R, z1 - z0, z=z0)` then the bevel z15 is added at z0 (lines 308-309); launch_aim.py:591-592 `Part("launch_aim_worm", wm, "PPA-CF", ... print_tf=bed(wm)`; measured: bed 50.1 mm2, bbox 28.0 x 28.0 x 54.5, OVH at z~15 435.5 mm2 r 4..14 and 73.8 mm2 per mm at r 9.5..14 for z 16..47.

Fix: Split the part. Use a plain d8 shaft (Harry's 8 mm steel rod, as the anchor pins already do) with two printed parts pinned to it by M3 cross pins or D-flats: (a) the worm, printed standing on its flat end face, with a 45 deg chamfer on the first thread turn; (b) the bevel z15, printed with its back face on the bed. Give each its own print_tf, and add the rod and pins as bought parts.

> Skeptic (confirmed): I checked this against origin/base-v1, where the cited line numbers match: worm_solid is at launch_aim.py:299-311 and the Part with print_tf=bed(wm) is at :591-592. The checked-out w/review tree has the same code at different line numbers. I exported base-v1 to /tmp/claude-0/review-scratch-P/A and measured the part in print orientation with the audit's own overhang test (45 deg). Results: bbox 28 x 28 x 54.46; true bed contact 50.08 mm2, which is just the d8 shaft end. The z15 bevel's small end face sits 0.2 mm above the bed (128 mm2 flagged as overhang at z=0.2). Slices give 50.1 mm2 of shaft
>
> Skeptic (confirmed): The geometry claim holds. The line numbers do not: on this branch worm_solid is at launch_aim.py:205-216 and the Part is at about line 466, not 307/592. The content is the same, though. worm_solid joins the worm (WORM_L 32), a d8 shaft cylinder from z0 to z1 and the bevel z15 into one solid. The Part gets print_tf=bed(wm) with no rotation, so the axis stands vertical.

I ran tools.audit.print_check on every launch_aim part (scratch script /tmp/claude-0/review-scratch-P/w.py). For launch_aim_worm it gives:
- print bbox 28.0 x 28.0 x 54.46 mm
- bed contact 50.1 mm2
- overhang 2778.6 mm2, which r
>

## Refuted (dropped)

- `base/cad/parts/launch.py:1231` launch_* audit states leave the bolted LAUNCH dog ring at wiper 0 while the sleeve turns, so all 79 states put the D37 lugs inside the ring
