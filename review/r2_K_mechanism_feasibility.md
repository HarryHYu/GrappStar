# Round 2 K: mechanism feasibility, load path torque and strength

Reviewed at base-v1 commit c631495. Method: independent finder subagents, one per lens; every finding of
medium severity or above was checked by two skeptic subagents told to refute it; kept only if fewer than
two refuted it. Low findings are listed unverified. Severity is the finder's, lowered when both skeptics
agreed it is real but rated it lower.

Confirmed: 8. Refuted and dropped: 0.

Re-checked 14:10 Sydney against base-v1 863d58d, one checker subagent per finding: see the Now column.

| # | Sev | File:line | Owner | Finding | Now (863d58d) |
|---|---|---|---|---|---|
| 1 | high | `base/cad/parts/launch.py:329` | cad-launch | Sprag #1 outer race has no key: the drive gear bore is a plain 47.1 slip fit with an empty slot, so the cocking torque has no load path | partly fixed |
| 2 | high | `base/cad/parts/launch.py:395` | cad-launch | Spring anchor pins: printed d6 PPA-CF, printed upright, with the hook 12 mm above the post. Bending runs across the layer lines at about 23 MPa at full cock | fixed |
| 3 | high | `base/cad/parts/reel_NOTES.md:100` | cad-reel | Brake track 2 drives nothing: no follower or bell crank links the drum's radial cam to the reel push rod, so the clutch is always LIGHT and FIRM 5 N is never reached | still open |
| 4 | medium | `base/cad/parts/_drum_kin.py:312` | cad-drum | Fork racking model is first-order and ignores the yoke-side offset; with a long arm on a 40 mm PLA sleeve the drum torque is underestimated and the N20 stalls at mu >= 0.35 | partly fixed |
| 5 | medium | `base/cad/parts/launch_aim.py:46` | cad-launch | Every gear in the aim train is weaker than the 5 A trip and the shear pin, so a blocked rail strips the aim gears before anything protects them | still open |
| 6 | medium | `base/spec/params.py:179` | physics | The provision for a second spring pair (D5) cannot work: with 4 springs every cock needs 5.83 N m, above both the shear pin and the 6.5 A cock trip | partly fixed |
| 7 | medium | `base/spec/params.py:221` | physics | Tow limits above about 75 N are accepted, but the shear pin breaks at the same torque as the 6.5 A tow trip (4.55 to 4.57 N m), so a high tow setting shears the pin | partly fixed |
| 8 | low | `base/cad/parts/launch_aim.py:264` | cad-launch | D23 thrust face missing: the launch sleeve has no axial stop in the aim housing, so it can drift up to 9.5 mm inboard and pull out of its bushing and both key ribs | partly fixed |

## 1. Sprag #1 outer race has no key: the drive gear bore is a plain 47.1 slip fit with an empty slot, so the cocking torque has no load path

- Now (863d58d): **partly fixed** at `base/cad/parts/launch.py:474`. drive_gear_solid no longer leaves the slot empty: it now adds an integral outer key rib, so the load path exists. But the rib crest sits at r = od/2 - 2.8 = 20.7, using the inner keyway depth of 2.8, while the CSK20PP outer keyway is only 1.5 deep (bearings.py CSK20_OKEY_D, floor at r 22.0). The rib therefore cuts 1.3 mm into the outer race and the bought sprag cannot be inserted. The interference is hidden because sprag_solid is a plain tube and Allow("launch_sprag","launch_drive_gear") at line 1059 suppresses the clash. The bore is also still a plain 47.1 with no crush ribs; seat_csk20, as the original fix proposed, would give the correct crest at r 22.2 and the ribs.

- File: `base/cad/parts/launch.py:329`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: When cocking, the sleeve rib locks the sprag inner race. The outer race then has to drive the 26T gear at 2.6 N m (the full-cock peak, launch_kin torque_wiper), and up to 5 N m at the shear-pin limiter. drive_gear_solid cuts a plain 47.1 bore over the 47.0 OD, which is clearance with no crush ribs. It also cuts a 6.2 x 4 slot that is left empty. parts() has no key, and the CSK20PP has keyways (6 x 1.5 outer, bearings.py:35), not keys. The outer race spins in the gear bore, so the gear does not turn and the spring is never cocked. If friction carries part of a cock and then slips, the spring back-drives the gear uncontrolled and the carriage slams home. The reel side of the same bearing uses the library's integral printed key rib (seat_csk20), which shows the intended solution.

Evidence: launch.py:329 `g = g - prim.cyl(sp["od"] / 2 + 0.05, 40, z=-20)`; launch.py:330 `g = g - prim.box(sp["od"] / 2 - 1.0, sp["od"] / 2 + 3.0, -sp["key"] / 2 - 0.1, sp["key"] / 2 + 0.1, -20, 20)` (a slot cut outward, not a rib); launch.py:336 sprag_solid is a plain tube; launch.py:596 Allow text claims 'sprag outer race keyed in the drive gear (press)'; no key Part in parts() (lines 422-475); bearings.py:35 `CSK20_OKEY_W, CSK20_OKEY_D = 6.0, 1.5`; reel.py:301 uses `bearings.seat_csk20(key_deg=90.0, ...)`.

Fix: Replace the bore and slot cuts in drive_gear_solid with the library seat: `g = g - prim.place(bearings.seat_csk20(depth=sp['w'], bottom='through', key_deg=0.0), ...)`. That gives crush ribs plus an integral 5.6 wide rib with its crest at r 22.2 over the full 14 mm sprag width. Rib bearing at the 5 N m limiter is 213 N / (1.3 x 14) = 11.7 MPa, SF 2.1 on PPA-CF. The gear is printed flat, so the rib is extruded along Z and its load stays in the layer plane. Alternatively, add a bought 6 x 4.5 x 14 steel key to parts() and the BOM.

> Skeptic (confirmed): I could not refute this claim. The code says what the claim says:
- In drive_gear_solid (base/cad/parts/launch.py:329-330), the 47.0 OD sprag sits in a plain bore of radius od/2+0.05. That bore is 47.1, which is 0.1 mm diametral clearance, with no crush ribs. The other cut is a 6.2-wide slot from r 22.5 to r 26.5. It removes material outward and adds no rib.
- sprag_solid is a plain tube.
- bearings.py:35 models the CSK20PP outer race with a 6 x 1.5 keyway, which is a groove, not a key. bearing_csk20 also cuts it as a groove.
- The gear slot and the race keyway therefore make an empty channel 
>
> Skeptic (confirmed): I checked the claim against the code and could not refute it.

1. The bore is a clearance fit. launch.py:329 cuts the drive gear bore at radius od/2 + 0.05, which is a 47.1 diameter over the 47.0 sprag OD. There are no crush ribs and no interference. The Allow text (line 596) and assembly step 6 (line 678) both say "press", but the geometry does not give a press fit.

2. Neither part has a key. Line 330 cuts a 6.2-wide slot outward into the gear hub. The CSK20PP is modelled with an outer keyway (bearings.py:35, CSK20_OKEY 6 x 1.5; bearing_csk20 cuts it). So there are two facing recesses and no
>

## 2. Spring anchor pins: printed d6 PPA-CF, printed upright, with the hook 12 mm above the post. Bending runs across the layer lines at about 23 MPa at full cock

- Now (863d58d): **fixed** at `base/cad/parts/launch.py:258-267,565-569,776-780`. The anchor pin is now listed as a bought steel part ("6 mm steel dowel or M6 bolt shank, 45 long"), so it is no longer a printed PPA-CF part. It also has no cantilever any more. The post now rises to wsp-5 and a top strap at wsp+5..+11 sits over the hook, so the pin runs through the post, the hook and the strap and carries the load in double shear with only 5 mm of span either side of the hook.

- File: `base/cad/parts/launch.py:395`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: Each Century spring pulls 40.1 N on its anchor pin at full cock (launch_kin spring_force(preload 24.1 + stroke 240.9) = 80.25 N for two springs). The front post that holds the pin only rises to w 48 (wsp - 12). The hook loop sits at w_spring = 60, so the d6 pin is a 12 mm cantilever above the post top. M = 40.1 x 12 = 481 N mm, and sigma = 32M/(pi d^3) = 22.7 MPa at the post-top section. The pin is printed upright (bed(..., Rx(180)): pin axis on the print Z), so this bending stress is interlayer tension at a layer line flush with the post top. The project's own allowable for PPA-CF root bending is 25 MPa (physics/common.py:118), and that figure is not reduced for the across-layer case, so the safety factor is 1.1. The 10 N preload also holds a static 5.7 MPa (creep) all the time. If a hook rides up toward the pin head at w 66, the lever is 18 mm and the stress is about 34 MPa. Every cock is a full load cycle. A pin that snaps at full cock releases one spring (about 6 J) and its hook toward the carriage, while the other spring yanks the carriage sideways. That damages hardware and puts the people nearby at risk.

Evidence: launch.py:395-399 `def anchor_pin_solid(P): """Printed PPA-CF pin d6 (vertical)...""" h = n["w_spring"] + 6.0 - (n["w_floor0"] - 1.0); return prim.cyl(3.0, h) + prim.cyl(5.0, 2.5, z=h)`; launch.py:210-211 the post `B(ua - 8, ua + 8, ..., n["w_floor0"], wsp - 12)` (top w 48); launch.py:233 pin hole `cyl_w(3.1, n["w_floor0"] - 1, wsp, ua, sg * ys)` runs to w 60 with no material above 48; launch.py:454 `print_tf=bed(anchor_pin_solid(P), tf.Rx(180))` (upright); spring hook placed at `tf.T(n["u_anchor"], y, n["w_spring"])` (w 60); common.py:118 `PPA_CF = Printed("PPA-CF", ..., 90e6, 25e6, 12e6)`.

Fix: Make the anchor pin a bought 6 mm steel dowel or an M6 bolt. The comment at line 231 already allows 'steel 6 mm', so mark the part bought in parts(). Also raise the anchor post to w 56 (change `wsp - 12` to `wsp - 4`), so the hook sits within 4 mm of the support (M = 160 N mm, 7.6 MPa even if printed). If the pin stays printed, print it lying with a 0.4 mm flat, the way drum.shear_pin does, so the bending runs along the layers.

> Skeptic (confirmed): I checked every cited line against the code and the claim holds.

1. **The posts stop 12 mm below the hook.** In launch.py:210-211 each front post runs from w_floor0 = 30 up to wsp - 12 = 48 (w_spring = 60 at line 108). It spans y 30..46, so it holds the pin at y = ±38. The 10 mm lower block (line 208) and the tie (line 212) both sit lower. The rail walls and lips stop at u_track1 = 190, behind u_anchor ≈ 246.7. Nothing supports the pin between w 48 and w 60.

2. **The hole adds no support.** The hole at line 233 runs from w 29 to w 60 with radius 3.1, but there is no material above w 48. It i
>
> Skeptic (confirmed): I tried to refute this and could not. Every number in the claim matches the code.

1) Load. params.SPRING has F0_n=7.0, k=0.125 and max_ext=265 (D30), so each spring pulls 7 + 0.125*265 = 40.1 N at full cock. The comment in params says "about 40 N".

2) Geometry (launch.py). n["w_floor0"]=30 and n["w_spring"]=60 (lines 99 and 108). The anchor post at lines 210-211 is B(ua-8, ua+8, y 30..46, w 30..wsp-12), so its top is at w 48. The pin hole at line 233 (cyl_w(3.1, 29, 60, ua, ±38)) sits inside that post's footprint (y 38 lies within 30..46). The only other solid near ua is the line 208 block, 
>

## 3. Brake track 2 drives nothing: no follower or bell crank links the drum's radial cam to the reel push rod, so the clutch is always LIGHT and FIRM 5 N is never reached

- Now (863d58d): **still open** at `base/cad/parts/reel.py:839`. On the new tree there is still no follower, bell crank or link part between the drum's track-2 cam (drum.py phi_brake=180) and the reel push rod, whose rear face is still at R(-88,-33,-24) (reel_NOTES.md:111). The reel_push joint is set from the drum zone table by brake_mm() rather than through any contact, and drum_REPORT.md:56 still lists "brake follower at drum L angle 180" as an open item for cad-reel. _drum_kin brake_force=15 is still an assumed value, and nothing has replaced the missing linkage.

- File: `base/cad/parts/reel_NOTES.md:100`
- Severity: high (claimed critical); 2 of 2 skeptics confirmed
- Owner: cad-reel

Scenario: In every run the drum turns to NEUTRAL 41, WRAPPED 314 or REEL 347, where track 2 is meant to lift 4 mm for FIRM. The cam is at the drum: its follower contact is at L angle 180, about world (30, y 18..29, z -112), and it pushes radially toward -x. The reel push rod's rear face is at R(-88, -33, -24), which is world (-106.7, y 57.5, z -102.6), and the rod is pushed along world -y. These points are about 137 mm apart in x and 30-40 mm apart in y, and the directions are perpendicular. No part in drum.py, reel.py or the frame modules spans the gap (grep for 'bell'/'follower' finds only reel_NOTES' promise to add one later). The fail-safe collar keeps the rod at LIGHT (0.43-0.6 N line) in every zone. Effects: (1) the tidy and the final approach on the FIRM take-up stall, so the firmware raises DART_STUCK. (2) In REEL the 7-wrap tail is 0.43 N instead of 5 N, so the wraps hold about 0.43 x 81 = 35 N at mu 0.10, below the 50 N default tow limit. The line slips through the wraps under tow and the retrieval fails. The drum torque budget also counts a 15 N load through a 1:1 linkage that does not exist.

Evidence: reel_NOTES.md:99-100 "BRAKE INPUT: push rod d 5.6, rear end face at R (-85, -35, -24), pushed along +Z by 4.0 mm for FIRM ... cad-drum's track is radial (lift 0 / 4 mm): once LAYOUT places the drum against my wall I add a 1:1 bell crank"; reel.py:79 "X_REEL = -88.0", reel.py:90 "PUSH_Y = -33.0", reel.py:459 "rod = prim.cyl(PUSH_D / 2, z_front + 24.0, x=X_REEL, y=PUSH_Y, z=-24.0)"; drum.py:53 "phi_brake=180.0"; drum_NOTES.md:94 "brake cam: base r 30, lift 0 (LIGHT) or 4 (FIRM). Follower contact at L angle 180 (straight back), 15 N at FIRM"; _drum_kin.py:44 "brake_force=15.0 ... cad-reel to confirm"; params REEL.clutch FIRM_line_n=5.0, tow_limit_n=50.0; reel_NOTES.md:114 wraps hold 208 N only "With the FIRM tail".

Fix: Add the brake follower linkage as a real part with joints and clearances. Use a bell crank on a frame-fixed pivot with a d6 roller follower on track 2 at drum phi 180, s 18..29, which turns the radial 4 mm lift into a 4 mm world -y push on the rod end at world (-106.7, 57.5, -102.6). This likely needs an intermediate link, given the 137 mm x offset. An alternative is to move track 2 and the follower so the cam sits next to the rod. Size the crank ratio so FIRM gives the 13 N rod push (D24). Then recompute _drum_kin.loads() brake_force for the real lever ratio and friction, and add an assembly test that drum 41/314/347 moves reel_push to 4.0 mm through actual contact.

> Skeptic (confirmed): I could not refute the claim. The linkage really is missing.

1) Linkage: reel_NOTES.md:99-101 says plainly that the bell crank and the d6 follower on track 2 have not been built yet ("once LAYOUT places the drum ... I add a 1:1 bell crank"). Searching base/cad for bell, follower and crank finds no brake linkage in reel.py, drum.py, frame.py or the _frame_*.py files. The only follower in drum.py is the fork's groove-1 pin. The reel_push joint (reel.py:603) is a slide whose position comes from brake_mm() (reel.py:632-649). That function reads the drum zone from the params table, so the assembly
>
> Skeptic (confirmed): I tried to refute this and couldn't. Every piece of the claim checks out.
(1) The quoted text is at reel_NOTES.md:99-101. It says the 1:1 bell crank with a d6 follower on track 2 will be added "once LAYOUT places the drum against my wall", so the notes themselves call it unbuilt.
(2) No such part exists anywhere. I grepped base/cad for bell, follower, crank, brake, rocker and reel_push. drum.py builds the brake cam profile (lines 456-476) and the code-switch levers, nothing else. reel.py adds the reel_rocker, which is the clutch spring between the rod and the thrust ring, and the reel_pushrod 
>

## 4. Fork racking model is first-order and ignores the yoke-side offset; with a long arm on a 40 mm PLA sleeve the drum torque is underestimated and the N20 stalls at mu >= 0.35

- Now (863d58d): **partly fixed** at `base/cad/parts/_drum_kin.py:324`. loads() now uses the exact equilibrium racking formula, (1+2mu*63.2/L)/(1-2mu*36.5/L), and the sleeve is 66 mm (drum.py:53). The N20 no longer stalls: at mu_rail 0.4 the worst torque is 1.22 N m against a 2.16 N m stall. What remains: the code assumes a greased rail at mu_rail 0.25. At mu_rail 0.4 the margin drops to 1.78 (1.43 with groove mu 0.4), which breaks the >=2 rule, and nothing tests mu 0.4. Groove 1 was not flattened and its pressure angle is still 43.7 deg.

- File: `base/cad/parts/_drum_kin.py:312`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-drum

Scenario: The drum moves L_TO_N (8..32) or N_TO_R (318..340) and the dogs land tip to tip, so the compliant yoke loads the fork. The fork sleeve (40 long, drum.py:589) rides the d10 rail at world (25, -58). The groove pushes the follower pin about 36.5 mm from the rail on one side. The collar reacts at the yoke pads about 63.2 mm from the rail on the opposite side, so both moments tip the sleeve the same way. Drawer-effect equilibrium is P(1 - 2 mu a1/L) = R(1 + 2 mu a2/L) + F0. The code instead uses P = (R + F0)(1 + 2 mu 45/L), which is only a first-order term. With the code's own inputs (R 6.75 N + 3 N on the axis, a 45 mm, mu 0.3) the exact force is 9.75/(1-0.675) = 30 N, not 16.3 N. A scratch run (/tmp/claude-0/review-scratch-K/rack2.py) used the real tip-to-tip yoke deflection profile, a1 36.5 and a2 63.2, and the groove's own 43.7 deg pressure angle (above the 30-35 deg guideline). Peak drum torque: 1.11 N m at mu 0.3 (margin 1.96 < 2 rule), 1.62 N m at mu 0.35 (margin 1.34), 2.53 N m at mu 0.4, which is more than the 2.16 N m drum stall. The sleeve self-locks outright at mu 0.55. A printed PLA sleeve on the rail at mu about 0.4 stalls the N20 mid-shift, leading to DRUM_TIMEOUT with the collar between dogs. The _drum_test assertion margin >= 2 passes only because the formula is wrong.

Evidence: _drum_kin.py:303 "def loads(c=None, r_groove=30.0, r_cam=32.0, rack_arm=45.0, sleeve_len=40.0, mu=0.3, residual_nm=0.2):"; _drum_kin.py:312 "rack = 1.0 + 2.0 * mu * rack_arm / sleeve_len"; _drum_kin.py:313 "f_fork = (c["fork_k"] * c["fork_free"] + 3.0) * rack"; drum.py:589 "sleeve = cyl(g["rail_d"] / 2 + 4.0, sb, sb + 40.0, rx, ry)"; drum.py:47 "rail_xy=(25.0, 58.0)" (collar axis at 0,0; pads at 3 and 9 o'clock, drum.py:570-571); loads() output groove_pressure_deg 43.7, cap_stall 2.16; _drum_test.py:121 "assert L["margin"] >= 2.0".

Fix: In loads(), replace the multiplier with the equilibrium form: f_fork = (R*(1 + 2*mu*a_yoke/L) + F0) / (1 - 2*mu*a_pin/L), with a_pin and a_yoke computed in drum.py from rail_xy, the pin point and the collar axis. Assert 2*mu*a_pin/L < 0.6 at mu 0.4. To restore the margin at mu 0.4, lengthen the fork sleeve to at least 70 mm, or move the rail onto the pin-to-collar line midway so the arms shrink. Also flatten groove 1 (widen tr_ln and tr_nr from 24/22 deg to about 34 deg, keeping the peak pressure angle at 35 deg or less), then rerun the margin test with mu 0.4.

> Skeptic (confirmed): I checked the claim against the code and could not refute it.

1. The cited code says what the claim says. In _drum_kin.py:303-313, loads() takes the yoke force plus 3 N of rail friction, (R + F0) = 6.75 + 3, and multiplies it by rack = 1 + 2*mu*45/40 = 1.675 at mu 0.3, giving f_fork = 16.33 N. That is a first-order drawer-effect term on one arm. It has no 1/(1 - 2*mu*a/L) feedback, so the racking friction is never allowed to load itself, and it ignores the moment from the yoke reaction.

2. The geometry from drum.G() matches the claim. drum_xy = (60, 112), phi_pin = -118.2 deg, rail_xy = (25,
>
> Skeptic (confirmed): I tried to refute this and couldn't. The defect is real.

1) The cited lines say what the claim says. _drum_kin.py:312-313 multiplies the whole fork force (fork_k*fork_free + 3 N) by 1 + 2*mu*45/40. That form is only exact when the drive acts on the rail axis and only the resisting force is offset. Here the driving force acts off-axis at the follower pin, so its own moment loads the sleeve ends and the friction term goes into the denominator: P(1 - 2*mu*a1/L) = ...

2) I checked the geometry myself in L coordinates (y down). Drum is at (60, 112) and phi_pin = atan2(-112, -60) = -118 deg. The g
>

## 5. Every gear in the aim train is weaker than the 5 A trip and the shear pin, so a blocked rail strips the aim gears before anything protects them

- Now (863d58d): **still open** at `base/cad/parts/launch_aim.py:47`. The aim gears are the same as before: splitter m1.5 face 8 at lines 47-49, bevel m1.5 z30/z15 face 8 at line 50, and the worm wheel with face 12 at line 53. The aim branch still has no torque limiter of its own; the diff since ba121b9 adds only the SW_RAIL_LOW cam/switch, the AS5600 bracket and the deck_z offset. DECISIONS D8/D26/D33 still set a 5 A aim trip and the roughly 5 N m coupler shear pin, so a jammed rail still strips the aim gears first. Also, physics/drivetrain.py aim() still has no Lewis check on the worm wheel (AIM_DEFAULTS, line 47).

- File: `base/cad/parts/launch_aim.py:46`
- Severity: medium (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: The drum is in LAUNCH and the wiper turns backward to aim. The rail meets an obstacle in its sweep, or the crank, conrod or lug jams. The wiper torque then rises toward the 5 A aim trip (kt 0.909 x (5 - 1.5) = 3.18 N m) and, if the trip misses, toward the shear pin's real break torque (4.57 N m at the 30 MPa the coupler code assumes). I checked the built geometry with base/physics/printed_gears.lewis against PPA-CF's 25 MPa allowable (scratch script /tmp/claude-0/review-scratch-K/torque.py). At the 3.18 N m trip the safety factors are: sun m1.5 z30 face 8 = 0.76, idler z18 = 0.66, bevel z15 face 8 = 0.77. The worm wheel m2 z20 face 12 carries 352 N of worm thrust (lead 4.8 deg, mu 0.25), which is 45.5 MPa (sf 0.55). At the shear pin torque the wheel carries 505 N, 65 MPa (sf 0.38), which is near the ultimate strength of printed PPA-CF. So the aim gears strip before the trip or the pin acts: the wheel is also the crank part, and the rail can no longer be aimed or held. The physics check passes only because it models a gear that was never built (m2, face 10, splitter only) and it never checks the worm wheel.

Evidence: launch_aim.py:46-52: `M_S = 1.5  # splitter module` / `Z_SUN, Z_IDL, Z_OUT = 30, 18, 30` / `FACE_S = 8.0` / `M_B, Z_B1, Z_B2, FACE_B = 1.5, 30, 15, 8.0` / `WHEEL_FACE = 12.0`; launch_kin.py:35,39 `worm_wheel_z=20`, `m_worm=2.0`. physics/drivetrain.py:58: `gear=dict(module=2.0, face=10.0, z_driver=30),   # splitter / bevel stage (m 2 needed: m 1.5 fails at the 5 A trip)`. Even for that larger gear the drivetrain run prints `aim (at limiter (5 N m)): gear fails (sf 0.87); relies on the trip`. drum.py:319: the shear pin is `about 5 N m`.

Fix: cad-launch: add a torque limiter in the aim branch itself, for example a printed shear key or pin between the splitter sun and the sleeve rib (launch_aim.sun_solid) set to about 1.0 N m. That is 3x the 0.32 N m aim peak, and at 1.0 N m the worm wheel sees about 110 N (sf about 1.7). Alternatively, upsize the stages until every aim gear has sf >= 1 at 4.6 N m, which needs about 2.5x the module x face on the wheel. physics: make drivetrain.aim read the real aim geometry (m1.5 z30/z18 face 8, bevel z15 face 8, wheel m2 z20 face 12). Add a Lewis check on the worm wheel driven by the worm thrust F = T_worm / (r_w (tan(lambda) + mu_e) / (1 - mu_e tan(lambda))) at the trip torque and at the limiter torque.

> Skeptic (confirmed): I checked the code and the defect is real against the project's own design rule. The "strips before anything protects it" part is overstated.

Confirmed:
(1) The built geometry is as the claim says. launch_aim.py:46-52 sets the splitter at m1.5 z30/z18/z30 face 8, the bevel at m1.5 z30/z15 face 8 and WHEEL_FACE 12. launch_kin.py sets m_worm 2.0, worm_wheel_z 20 and worm_pd 24.
(2) The sun is keyed on the launch sleeve and turns 1:1 with the wiper (sleeve_deg equals wiper_deg in the params storyboard). On backward turns sprag #1 freewheels, so the aim branch carries the full wiper torque.
(3) p
>
> Skeptic (confirmed): I could not refute the core of this claim. The headline overstates it.

What I checked:
1. The built geometry matches the evidence. launch_aim.py:46-52 has splitter m1.5 z30/z18/z30 face 8 and bevel m1.5 z30/z15 face 8. WHEEL_FACE is 12, and the worm wheel is m2 z20 (launch_kin).
2. physics/drivetrain.py:58 models the aim gear as module 2.0, face 10, z_driver 30. Its own comment says "m 2 needed: m 1.5 fails at the 5 A trip". It has no Lewis check on the worm wheel or the idler.
3. Running drivetrain.py reports "gear sf at the hard trip: aim 1.37". That passes only because of the m2/face-10 ge
>

## 6. The provision for a second spring pair (D5) cannot work: with 4 springs every cock needs 5.83 N m, above both the shear pin and the 6.5 A cock trip

- Now (863d58d): **partly fixed** at `base/spec/BASE_SPEC.md:199`. Decision D38 (base/spec/DECISIONS.md:46) drops 4-spring support in v1, and recoil.py:144, FINDINGS.md:15 and the build docs now mark 4 springs as not supported. The spec still has leftovers: BASE_SPEC.md:199-200 says "a second spring pair gives about 8 to 9 m", which breaks D38's "documents must not promise the 4-spring range". params.py:185 also still labels second_pair_provision as "# D5" with no D38 note.`

- File: `base/spec/params.py:179`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: physics

Scenario: Following D5, Harry fits the provisioned second pair of C-243 springs to get 8 to 9 m of range. I ran drivetrain.cock with n_springs = 4: peak cock torque at the wiper is 5.83 N m (carriage 161 N). That is above the shear pin's 4.57 N m (geometry at 30 MPa), above the nominal 5 N m limiter and above the 6.5 A cock trip (4.55 N m). Every cock trips or shears the pin near the end of the stroke, so the launcher cannot fire. Physics (recoil.py) reports the 4-spring energy and range without this torque check.

Evidence: params.py:179 `second_pair_provision=True, # D5`; DECISIONS.md:13 (D5) `Provision (anchor holes and clearance, not fitted) for a second pair of the same springs: about 8 to 9 m range`; physics run with p.n_springs=4: `cock t_peak 5.83 N m, f_peak 161.5 N`; 2-spring run: `cock 2.93 N m ... 58.7 % of limiter`; drum.py:319 shear pin `about 5 N m at 30 MPa` (geometry gives 4.57 N m).

Fix: physics: in recoil.py's spring-count comparison, compute drivetrain.cock(n_springs = 4) and flag it whenever t_peak > LIMITER_FRAC_MAX x limiter or > the cock trip torque. Then either drop the second-pair provision from D5 and params, or pair it with a cock ratio change that keeps the cock peak under 0.8 x 4.57 N m. For example, a 20T drive gear gives 5.83 x 20/26 = 4.5 N m, which is still too high, so in practice the ratio would have to be about 40/16. Alternatively, pair it with a stronger pin plus a raised cock trip.

> Skeptic (confirmed): I could not refute this claim; it holds up. I re-ran drivetrain.cock with the current params (COCK_DEFAULTS: drum_r 0.05, ratio 40/26, eta 0.9). With 2 springs the peak is t_peak 2.93 N m at f_peak 81.2 N. With 4 springs it is t_peak 5.83 N m at f_peak 161.5 N, which matches the claim exactly.

The result does not depend on the assumed efficiency. With eta = 1 the 4-spring peak is 161.5 x 0.05 / 1.538 = 5.25 N m, still above the limits below.

The limits it has to stay under:
- **Cock current trip:** FINDINGS.md:113 says the hard trip while cocking is 6.5 A, about 4.5 N m.
- **Mechanical limit
>
> Skeptic (confirmed): I could not refute this. The claim holds. I ran base/physics drivetrain.cock with the default params. With 2 springs it gives t_peak 2.93 N m and f_peak 81 N. With p.n_springs=4 it gives t_peak 5.83 N m and f_peak 161.5 N. The cock trip torque is kt*(TRIP_COCK_A - i_free) = 0.909*(6.5-1.5) = 4.55 N m. The shear-pin limiter is set at 5.0 N m (params torque_limiter_nm, D8/D26). With LIMITER_FRAC_MAX 0.8 the usable limit is 4.0 N m. So a 4-spring cock exceeds the 6.5 A trip, the nominal shear pin and the 0.8 limiter fraction. Near the end of the stroke the firmware trips or the pin shears, so the
>

## 7. Tow limits above about 75 N are accepted, but the shear pin breaks at the same torque as the 6.5 A tow trip (4.55 to 4.57 N m), so a high tow setting shears the pin

- Now (863d58d): **partly fixed** at `base/spec/params.py:228 (tow_limit_max_n=70.0, D39); base/spec/params.py:162 (torque_limiter_nm=5.0); base/physics/capstan.py:284,323`. D39 now caps the tow limit at 70 N in params, firmware (base_logic.h:869) and the UI (max 70). wiper_torque_for(70) is 4.30 N m, below the pin's ~4.57 N m geometric break (that torque is reached near 76 N), so a 100 N setting can no longer shear the pin. What remains: the cap is 70 N, not the ~59 N that 0.8 x 4.57 allows. Physics still models the pin as 5.0 N m and still checks only the 50 N default. Run at tow=70, it sets slips_at_tow=True (tow_frac 0.86), but nothing runs that check against tow_limit_max_n, and a 70 N tow uses about 94% of the pin's real strength.

- File: `base/spec/params.py:221`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: physics

Scenario: Harry sets the tow limit to 100 N for clay. The UI accepts 1..100 N and the firmware caps at 110. Through the real capstan train (capstan.wiper_torque_for, take-up included) the wiper torque is 4.56 N m at 75 N, 5.05 N m at 84 N and 5.90 N m at 100 N. The pin's real break torque from its coupler geometry is about 4.57 N m: d4 minus the 0.4 mm print flat leaves 11.7 mm2, x 30 MPa x r 13 mm. The 6.5 A tow trip (D33) is 0.909 x 5 = 4.55 N m. So the pin shears at the same moment the trip fires, before the line reaches the limit. The coupler then spins free while sprag #2 locks the load on the line. The tow stops with the rover still short of the anchor, and the pin has to be replaced by hand. Physics checks only the 50 N default (slips_at_tow uses `tow`), and it models the pin as 5.0 N m, not the 4.57 N m its own geometry gives.

Evidence: params.py:221 `tow_limit_n=50.0,  # firmware default, adjustable 2..110`; params.py:223 `current_trips_a=dict(aim=5.0, reel_in=5.0, cock=6.5, tow=6.5, ...)`; firmware/base/PROTOCOL.md:365 `| limit_n | 50 (params.REEL) | 1..110 |`; jetson/grappstar/app.py:348 `not (1 <= v <= 100)`. drum.py:29 `hub_od=26.0, ... shear_s=13.0, shear_d=4.0`; drum.py:324 `p = p - prim.box(-20, -g["shear_d"] / 2 + 0.4, ...)  # 0.4 mm flat`; drum.py:319 `PLA pin d4 in single shear at r 13 (about 5 N m at 30 MPa)`. capstan.py:321 `tow_frac=t_tow / lim, slips_at_tow=t_tow > LIMITER_FRAC_MAX * lim` evaluates only the default tow.

Fix: physics: model the limiter at the torque its geometry gives (about 4.57 N m), and add a check over the whole allowed tow range: f_max = line_from_wiper(LIMITER_FRAC_MAX x limiter) is about 60 N. Flag any allowed tow limit above f_max. Coordinator, firmware and Jetson: cap REEL.tow_limit_n, the firmware max_n/limit_n and the UI at 60 N. If clay towing (up to 112 N) is a requirement, cad-drum must enlarge the pin instead, for example d5 (about 7.1 N m), and move the tow trip below it.

> Skeptic (confirmed): I checked the code and the physics model, and the claim holds.

1) Allowed tow range. app.py:348 accepts 1..100 N. PROTOCOL.md:84 and :365 cap max_n and limit_n at 110 N. params.py:221 says the default can be adjusted from 2 to 110 N.

2) Tow trip. The tow trip is fixed at 6.5 A: config.h:63 `towTripA = 6.5f`, used at base_logic.h:801 when until == U_TENSION. D34 says the firmware computes I_trip = (limit + 10.4)/17 + I_free, but the firmware does not do that. With kt 0.909 and I_free 1.5, 6.5 A corresponds to 4.55 N m at the wiper.

3) Line force against wiper torque. I ran the repo's own phy
>
> Skeptic (confirmed): I checked the claim against the code and it holds. The one overstatement is that the pin breaks exactly when the trip fires; in fact the trip and the pin's break torque sit within a few percent of each other.

1) A high tow limit gets through every layer. jetson app.py:349 accepts 1..100 N. firmware cmdReel (base_logic.h ~795) caps max_n only at c.limitMaxN, 110 N per PROTOCOL.md.

2) The wiper torque values are right. Running base/physics/capstan.wiper_torque_for with the repo params gives 3.23 N m at 50 N, 4.56 at 75 N, 5.05 at 84 N and 5.90 at 100 N.

3) The tow current trip is 6.5 A in the
>

## 8. D23 thrust face missing: the launch sleeve has no axial stop in the aim housing, so it can drift up to 9.5 mm inboard and pull out of its bushing and both key ribs

- Now (863d58d): **partly fixed** at `base/cad/parts/launch_aim.py:322,351 (housing boss to y 194.5, d20.35 bore stops at y 193, d12.5 beyond it) and base/cad/parts/launch.py:526 (sleeve z_end = Y0+53-y_flange, end at y 193, no groove, clip or shoulder)`. The outboard thrust is fixed. The housing now has a lip at y 193: past that point the bore narrows to d12.5, and the sleeve, which now ends at y 193, butts against it. test_launch_aim.py:200 checks this, so the LAUNCH push no longer loads the swing arm. The inboard stop is still missing: the sleeve is a plain d20 with no C-clip groove or shoulder, nothing is fixed to it axially, and the flange still sits 4 mm clear of cheek A (119 vs 123). The sleeve can still drift inboard until the dog ring meets the collar, so it leaves the 187..193 bushing after 6 mm and the sun rib (174..182) walks out of the sun.

- File: `base/cad/parts/launch_aim.py:264`
- Severity: low (claimed medium); 1 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: The contract (BASE_SPEC line 116, D23) puts the sleeve's outboard bushing WITH a thrust face at y 187..193. In the code, the housing bore is a plain d20.35 straight through the wall (no lip). The sleeve is a plain d20 cylinder to y 192 (no shoulder, groove or clip). Nothing else fixes the sleeve axially: the flange is 4 mm clear of the rail cheek (119 vs 123), the cheek bores carry a slot so the rib passes, and the sun and sprag can slide on their ribs. Inboard, the only stop is the dog ring hitting the collar: 2.5 mm in NEUTRAL, and 9.5 mm with the collar in REEL (collar LAUNCH-side tips at 91 vs ring tips at 100.5). Vibration or rover roll while towing in REEL can walk the sleeve inboard. At 5 mm the sleeve end (192) leaves the 187..193 bushing. At 8 mm the sun rib (174..182) is out of the splitter sun (174..182). At 9.5 mm the sprag #1 rib engagement is down from 14 to 5 mm. The next `mode LAUNCH` then has to push the sleeve back through the free-turning sun. If the rib end lands on the sun face, the collar cannot reach full engagement (SW_DOG_L never closes) and the LAUNCH shift faults. Outboard, the LAUNCH tip-to-tip push (fork up to 6.75 N plus friction, drum_kin) is taken only by accident, after 0.5 mm, by the rib end face (y 182) rubbing on the friction-dragged swing arm hub (182.5..186.5), which bears on the housing wall at 187. That loads the arm that the pawl-free aim splitter needs to swing freely.

Evidence: launch_aim.py:264 `h = h - prim.cylY(10.0 + FIT / 2, Y_W1[0] - 1, Y_W1[1] + 1, 0.0, az)` (through bore, no lip); launch.py:355-356 `z_end = n["Y0"] + 52.0 - n["y_flange"]` / `s = prim.cyl(10.0, z_end) + prim.cyl(32.0, 8.0)` (plain d20 to y 192, only a flange at 111..119); launch_aim.py:59 `Y_ARM0, Y_ARM1 = 182.5, 186.5`; drum.py:34 `cap_s=(195.0, 201.5)`; drum_NOTES.md:74 'Thrust: two-way axial location at the frame outboard seat'; BASE_SPEC.md:116 'y 187..193, thrust face (LAUNCH engage pushes outboard) (D23)'.

Fix: Give the sleeve two-way axial location at the housing wall. (1) Extend the sleeve to y 194.6 (z_end = Y0 + 54.6 - y_flange). Cut a C-clip groove for d20 at y 193.3..194.4 and fit a clip that bears on the housing outer face y 193. This stops inboard drift and stays clear of the magnet cap at 195. (2) Add a full 360 deg thrust shoulder on the sleeve, r 13 at y 186.5..187, bearing on the housing inner face y 187, for the outboard engage push. Move the swing arm 0.5 mm inboard (Y_ARM0, Y_ARM1 = 182.0, 186.0, sun to 173.5..181.5) so the arm is no longer the thrust path. Keep the housing bore at d20.35 as the journal.

> Skeptic (confirmed): The core claim is real, but its headline scenario is overstated.

Confirmed in code:
- launch_aim.py:264 cuts a plain d20.35 bore through the housing wall at y 186..194, with no lip.
- launch.py:355-356 makes the sleeve a plain d20 cylinder to y 192 (Y0 140 + 52) with only the flange at 111..119. It has no shoulder, groove or clip.
- So the 'thrust face' that BASE_SPEC line 116 and D23 require at y 187..193 does not exist. That is a real contract miss.
- On the outboard side the claim holds. The swing arm bore is d20.35 (launch_aim.py:144) and the rib top is at r 12.8, so the rib end face at y
>
> Skeptic (refuted): The claim's main scenario cannot happen. It treats the rail cheek bores as no axial stop because they "carry a slot so the rib passes". In the code that slot is a narrow keyway at one angle only. In launch.py:229 it is ±3.3 mm wide and points down in the rail frame. The sprag rib is 6 wide with its top at r 12.8 (launch.py:347, 360), so it can only pass when it lines up with the slot within about ±1.4 deg. Even then it must line up against the rail pitch, because the rail and the sleeve turn on separate joints (launch.py:490-491). The rest of the time the rib is trapped between the cheeks. The
>
