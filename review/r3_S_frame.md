# Round 3 S: frame subsystem on the w/cad-frame tip

Reviewed at base-v1 commit f1720b1. Method: independent finder subagents, one per lens; every finding of
medium severity or above was checked by two skeptic subagents told to refute it; kept only if fewer than
two refuted it. Low findings are listed unverified. Severity is the finder's, lowered when both skeptics
agreed it is real but rated it lower.

Confirmed: 9. Refuted and dropped: 0.

| # | Sev | File:line | Owner | Finding |
|---|---|---|---|---|
| 1 | high | `base/cad/parts/frame.py:716` | cad-frame | The frame sets the wiper motor 10 mm further back than params and the drum coupler expect, so the coupler gets only 3.5 mm of the shaft flats and 1.2 mm of the M8 thread |
| 2 | high | `base/cad/parts/frame.py:854` | cad-frame | Deck tiles 1 and 2 are 255.5 mm in print pose: frame.checks() fails, and the tiles do not fit the 256 bed with margin |
| 3 | medium | `base/cad/parts/_frame_motor.py:283` | cad-frame | Puck adapter bore and lip are fixed constants, not taken from params.WIPER, so a measured motor collides with the puck after a params-only reprint |
| 4 | medium | `base/cad/parts/_frame_others.py:44` | cad-frame | load_others swallows any one subsystem's build error and drops every other subsystem, so the deck gets no inserts under the reel wall (sprag #2 backstop), drum brackets or posts, with no warning |
| 5 | medium | `base/cad/parts/frame.py:286` | cad-frame | The motor reaction path is the weakest link and the calc does not cover it: the bridge-to-plate connector is in interlayer torsion, and the leg foot has only a 14 mm screw span about the torque axis |
| 6 | medium | `base/cad/parts/frame.py:1053` | cad-frame | No step sets the heat-set inserts in the PPA-CF motor adapter hub, yet 'Motor onto the cheek' screws into them |
| 7 | medium | `base/cad/parts/frame.py:1081` | cad-frame | The motor-leg step says M3 x 16 for the bridge screws, which bottom out before clamping and push the bridge inserts out; it also says 2 deck screws where the leg has 4 |
| 8 | low | `base/cad/parts/_frame_ebay.py:146` | cad-frame | Fuse-holder tie slot is cut mostly inside the bay outer wall, leaving a 0.3 mm skin (a hole once sliced) |
| 9 | low | `base/cad/parts/_frame_ebay.py:151` | cad-frame | Bay sensor and motor wire exits face the solid motor leg 5 mm away (sensor window 100% blocked) |

## 1. The frame sets the wiper motor 10 mm further back than params and the drum coupler expect, so the coupler gets only 3.5 mm of the shaft flats and 1.2 mm of the M8 thread

- File: `base/cad/parts/frame.py:716`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-frame

Scenario: At the w/cad-frame tip (f1720b1), I ran frame.build(params) and then frame.parts(). frame_wiper_shaft spans world y -18..21.99. That puts the shaft flats at y -4..10 and the M8 thread at y 10..22. frame_wiper_motor tops out at y -6, which is the output boss top. cad-drum's coupler hub (drum.py GEOM) was built to the published model: boss top at y 4, flats to y 20, thread to y 32. Its D-bore runs s 6.5..19.8 and its M8 nut pocket is at s 20.8..27.3. So the D-bore overlaps the real flats only over y 6.5..10, which is 3.5 mm. From y 10 to 19.8 the D-bore sits around the d8 thread and drives nothing. The nut catches only 1.2 mm of thread (y 20.8..22), so it cannot clamp the hub. In a normal run the hub is not held axially, and wiper torque (up to the about 5 N m shear pin, 15 N m stall) goes through 3.5 mm of PPA-CF flats. The hub works loose or strips its flats, and the main shaft stops being driven. The audit stays CLEAN because a shaft that is too short leaves a gap rather than a collision.

Evidence: frame.py:716 `y_boss = (C["y_out"][ms]) if C["v1"] else (yo + ms * MOT.PLATE_T)   # v1: boss face on the plate rear (y -10)`. cfg gives y_in[-1]=0 and y_out[-1]=-10 (I printed this). _frame_motor.py:293: puck 'body z 0..PLATE_T (front face = world y 0)', so the motor bosses sit at y -10. params.py:16 'y = 0 at the wiper motor adapter plate face (the wiper boss face)' and params.py:33 'motor frame: origin on the output shaft axis in the boss face plane'. drum.py:29-31 `wiper_face_s=4.0, # wiper output boss top (frame_wiper_motor reaches y 4.0) ... flats to WIPER.shaft_len (y 20), then the M8 thread to y 32` and `hub_s=(4.5, 27.5), nut_s=20.8`. drum.py:279 `f1 = L - 0.2  # flats end at WIPER.shaft_len from the motor frame origin (y 0)`. frame_INTEGRATION.md:79-82 still tells cad-drum 'boss ... (y 0..4), the shaft d10 with flats 8 across from y 6 to 20, then the M8 thread y 20..32'. Measured bounds: frame_wiper_shaft (-5,-18,-5)..(5,21.99,5), frame_wiper_motor y max -6.0.

Fix: Use one boss-plane y for both owners. With the puck at y -10..0 the bosses cannot sit at y 0, so cad-frame has to publish the real offset. Add the boss plane (y = y_out[ms] = -10) to the contract through the coordinator, for example params.WIPER boss_plane_y=-10.0, and change the params.py:16/:33 wording. Correct frame_NOTES.md section 3/6 and frame_INTEGRATION.md:79-82 to: output boss top y -6, flats y -4..10, M8 thread y 10..22. Then cad-drum derives its hub from that value: wiper_face_s = boss_plane_y + hub_h (-6), D-bore flats from boss_plane_y + hub_h + 2.5 to boss_plane_y + shaft_len - 0.2 (9.8), nut_s about 11, and the hub starting at y -5.5 inside the puck bore (hub r 13 < BORE_R 15). If the drum stack must keep starting at y 0.5, the other route is for cad-frame to move the motor 10 mm forward, which needs a puck where the bosses bear at y 0.

> Skeptic (confirmed): I could not refute this. I ran frame.parts(params) from an archive of origin/w/cad-frame (f1720b1) under /tmp/claude-0/review-scratch-S/wf. I got frame_wiper_motor up to y -6.0, frame_motor_adapter at y -16..0 and frame_wiper_shaft at y -18..21.99, which are the claimed numbers.

Code path at the tip:
- frame.py:716 sets y_boss = C["y_out"][ms] in v1, which is -10. zdir is (0, +1, 0) because ms = -1.
- _frame_motor.motor_shaft puts the hub top at z hub_h = 4 and runs the flats from z 6 to shaft_len 20. The M8 thread runs from z 20 to 32.
- In world y that gives a boss top at -6, flats at y -4.
>
> Skeptic (confirmed): I could not refute the claim. I checked it at the w/cad-frame tip f1720b1 by exporting the tree to /tmp/claude-0/review-scratch-S/fb and running frame.parts(params). The repo working tree was not touched.

1. The measured bounds match the claim exactly: frame_wiper_motor reaches y max -6.0, frame_motor_adapter (the puck) spans y -16..0, and frame_wiper_shaft spans y -18..21.99.

2. At f1720b1, frame.py:716 sets `y_boss = C["y_out"][ms]` when C["v1"] is true. On that branch the cfg code sets y_in[-1] = 0 (a comment says 'motor plate front face IS the spec's y = 0'), so y_out[-1] = y_in - t = -1
>

## 2. Deck tiles 1 and 2 are 255.5 mm in print pose: frame.checks() fails, and the tiles do not fit the 256 bed with margin

- File: `base/cad/parts/frame.py:854`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-frame

Scenario: Default params on w/cad-frame f1720b1. deck_rect is x -160..269.5, so lx = 429.5 and nx = 2. seam_c puts the x seam at c = 21.75: hi - c = 247.75 passes the 248 limit. But split_plate/dovetail_seam_cs (_frame_util.py:94, depth=8.0) adds dovetail tails that run 8 mm past the seam into the '-' tile, so frame_deck_1 measures x 13.9..269.4 = 255.5 mm (frame_deck_2 is 255.5 x 211.8). I ran frame.checks(P) and it returns ['frame_deck_1: 255 x 154 x 8 does not fit the bed', 'frame_deck_2: 255 x 212 x 8 does not fit the bed']. That leaves 0.25 mm per side on the X2D's 256 bed, under the repo's own 2 mm margin. The deck, the base's foundation, cannot be sliced or printed reliably, and the subsystem check is never clean.

Evidence: frame.py:848 `nx, ny = int(math.ceil(lx / 248.0)), int(math.ceil(ly / 248.0))`; frame.py:854 `if c - lo > 248.0 or hi - c > 248.0 * (n - 1):`; _frame_util.py:94 `def dovetail_seam_cs(axis, c, lo, hi, n, depth=8.0, ...)` (the tails stick from the '+' side into the '-' side); frame.py:1019 `if sx > bed[0] - 2 or sy > bed[1] - 2 or sz > bed[2]:`. Measured: seams [{'axis':'x','c':21.75},{'axis':'y','c':74.0}], frame_deck_1 bbox x 13.9..269.4.

Fix: Count the tail depth in the tile budget. In frame.py:848 and :854 replace 248.0 with (bed[0] - 2 - 8.0), i.e. 246 - 8 = 238.0 for the tail depth: `nx = ceil(lx / 238.0)` and `if c - lo > 238.0 or hi - c > 238.0 * (n - 1): continue` (same for y). For this deck that limits c to [31.5, 80.0], which is feasible with 2 tiles. Add a test that every deck tile passes the frame.py:1019 check.

> Skeptic (confirmed): I could not refute this. I reproduced it by extracting f1720b1:base into /tmp/claude-0/review-scratch-S/base (the repo working tree was not touched). f1720b1 is an ancestor of origin/base-v1, and frame.py and _frame_util.py are the same in both, so base-v1 has the defect too.

What the code does:
- frame.py:848 picks the tile count with ceil(lx/248), and frame.py:854 accepts any seam with c-lo <= 248 and hi-c <= 248*(n-1).
- _frame_util.dovetail_seam_cs (depth=8.0) builds the '+' region from base = box(c, ..., c+big) plus tail polygons that reach c - depth. So the '+' tile spans [c-8, hi] in t
>
> Skeptic (confirmed): The defect is real, and I reproduced it. I extracted f1720b1 (origin/w/cad-frame) with git archive into /tmp/claude-0/review-scratch-S/f1720, so the repo was not touched, and ran frame.build/checks with the default params:
- deck_rect is (-160, 269.5, -138, 220).
- The seams are x c=21.75 and y c=74.0, as claimed.
- frame_deck_1 and frame_deck_2 measure 255.45 mm in X in print pose.
- frame.checks(P) returns exactly ['frame_deck_1: 255 x 154 x 8 does not fit the bed', 'frame_deck_2: 255 x 212 x 8 does not fit the bed'].

Why it happens: in seam_c (around frame.py:854 on f1720b1, line 784 on w/
>

## 3. Puck adapter bore and lip are fixed constants, not taken from params.WIPER, so a measured motor collides with the puck after a params-only reprint

- File: `base/cad/parts/_frame_motor.py:283`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-frame

Scenario: The design promise is that Harry edits only params.WIPER and reprints frame_motor_adapter. In the v1 puck that is not true. (1) Set WIPER hub_d=32, a common size for a wiper output bushing boss. The puck bore stays r 15, puck_checks returns [], and motor_body ^ puck overlaps by 389.6 mm3, so the motor cannot seat on the puck. (2) Set the third mount hole at (75, 0), 3 mm off the assumed spot and inside the ±4 mm slot the puck is meant to absorb. The d16 boss (r up to 83) runs into the rear lip (inner r 82, z -6..0): 33.1 mm3 overlap, and puck_checks again returns []. Both are only caught later by the assembly audit's 1 mm3 allowance, and both need a code change instead of a params change.

Evidence: _frame_motor.py:283 `BORE_R = 15.0                 # output boss r 12 + 3` (constant, never derived from w['hub_d']). _frame_motor.py:297 `body = prim.cyl(PUCK_R, t, z=0.0) + prim.tube(LIP_R, PUCK_R - 6.0, LIP_T, z=-LIP_T)` (lip inner r 82 is fixed). _frame_motor.py:321 puck_checks tests only the head slot against BORE_R, and nothing tests hub_d or boss_d. Scratch run /tmp/claude-0/review-scratch-S/s2.py: default overlap 0.0; mount hole (75,0): overlap 33.1, checks []; hub_d=32: overlap 389.6, checks [].

Fix: In puck(w), compute bore_r = max(15.0, w['hub_d']/2 + 1.5) and use it in place of BORE_R. Compute the lip inner radius as max(PUCK_R - 6, max over holes_rot(w) of hypot(hx,hy) + w['slot'] + w['boss_d']/2 + 1.0), and clamp it so at least 4 mm of lip remains under the RIM_R inserts. In puck_checks, add two failures: w['hub_d']/2 + 1.0 > bore_r, and r + slot + boss_d/2 + 1 > lip inner radius for any hole.

> Skeptic (confirmed): The claim does not hold against the local w/review checkout, which lags origin. There, base/cad/parts/_frame_motor.py has only 239 lines and no puck code. It does hold against origin/base-v1, the branch w/review is meant to copy, where commit 58e0b43 added the v1.x puck. I pulled that tree with git archive into /tmp/claude-0/review-scratch-S/skA, without touching the working tree. In that tree:

- `BORE_R = 15.0` is a module constant.
- puck() cuts `prim.cyl(BORE_R, ...)` and builds the lip as `prim.tube(LIP_R, PUCK_R - 6.0, LIP_T, z=-LIP_T)`, so the inner radius is fixed at 82 and the lip spa
>
> Skeptic (confirmed): The claim checks out against origin/base-v1, the branch under review. Note that the local w/review HEAD (8d038b5) is behind origin/base-v1 (4a28d2d). Its _frame_motor.py has only 239 lines and no puck code. The puck came in with commit 58e0b43, which is in origin/base-v1. I exported origin/base-v1:base into /tmp/claude-0/review-scratch-S/skB/repo with git archive, so the repo was not touched, and checked it there.

What the code does in origin/base-v1 base/cad/parts/_frame_motor.py:
- BORE_R = 15.0 and the lip inner radius PUCK_R - 6 = 82 are fixed constants. puck() never reads w['hub_d'] or w
>

## 4. load_others swallows any one subsystem's build error and drops every other subsystem, so the deck gets no inserts under the reel wall (sprag #2 backstop), drum brackets or posts, with no warning

- File: `base/cad/parts/_frame_others.py:44`
- Severity: medium (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-frame

Scenario: On the w/cad-frame tip (f1720b1), launch.parts() raises NameError (RIB_T, then _truss_windows, launch.py:385/415). Assembly(P, mods) raises, and the bare except at :45 sets asm = None for ALL subsystems. frame.build(P) then returns others_holes = [] (I measured 0) and warnings = []. The deck carries only its own 19 holes. It is missing the 7 inserts under reel_wall (the wall that holds the sprag #2 hub and reacts the ~70 N capstan-side tow: 60 N x 1.15 at 95 mm above the deck, about 6.7 N m), plus drum_bracket_a/b/idler/idler2, drum_switch_block, reel_exit_post and reel_p2_post: 25 inserts in all. I measured this by rebuilding with launch skipped. The motor plate is also left uncarved. Decks printed from this build cannot anchor the tow backstop. Nothing fails: the frame tests and the calc pass.

Evidence: _frame_others.py:43-46 "if mods:\n            asm = Assembly(P, mods)\n    except Exception:\n        asm = None". frame.py:445-446 "opolys, oholes, oown = OTH.deck_contacts(P, C[\"deck_z\"])\n    info[\"others_foot\"], info[\"others_holes\"] = opolys, oholes". frame_INTEGRATION.md: "launch (DOES NOT BUILD: NameError: name 'RIB_T' is not defined (launch.py:385))".

Fix: Build the others one module at a time. For each module, try Assembly(P, {n: m}) (or collect parts per module) and keep the ones that build. Record every failure (module name and exception) in a list that frame.build copies into info['warnings']. Make frame.checks fail when any non-frame subsystem fails to load or contributes no deck contacts. Do the same for the import failure at :39-40.

> Skeptic (confirmed): The code does what the claim says. base/cad/parts/_frame_others.py:31-46 on HEAD (w/review = base-v1) builds a single Assembly(P, mods) from every non-frame subsystem, and the whole thing sits inside a bare `except Exception: asm = None`. Assembly.__init__ (tools/assembly.py:43-75) calls mod.parts(P) for each module in turn, so if any one module raises, every subsystem is lost, not just that one. When asm is None, deck_contacts returns ([], [], []) (line 75-76). frame.build copies that into info["others_foot"/"others_holes"] (frame.py:407) and adds no warning. Nothing in frame.checks, which tu
>
> Skeptic (confirmed): The defect is real. On f1720b1 I reproduced it from an archived copy (/tmp/claude-0/review-scratch-S/f17): drum, reel and sensors each build, but launch.parts raises NameError 'RIB_T'. As a result, _frame_others.load_others(P) returns None, because the bare except at :45 wraps the whole Assembly(P, mods) call. deck_contacts then returns ([], [], []) and frame.build sets others_holes and others_foot to empty lists. No warning is added anywhere. frame.checks only turns info['warnings'] into failures (frame.py:1034), and nothing checks that the others loaded or that others_holes is non-empty. _fr
>

## 5. The motor reaction path is the weakest link and the calc does not cover it: the bridge-to-plate connector is in interlayer torsion, and the leg foot has only a 14 mm screw span about the torque axis

- File: `base/cad/parts/frame.py:286`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-frame

Scenario: All of the wiper reaction torque about Y passes from the plate (y -10..0) to the bridge (y -30..-18) through the connector 'conn' alone. I probed it: the section at y -14 is x 99..111 by z 143..187, 12 x 44 = 528 mm2. The plate prints with Rx(-90), so layers are planes of constant y, and the torque twists this section within a layer interface. Torsion: tau = T/(0.277 x 44 x 12^2) = T/1755 mm3. That is 1.65 MPa at the 2.9 N m cock, 2.85 MPa at the 5 N m limiter (SF 1.4 against the code's own PLA_ACROSS = 4 MPa) and 8.5 MPa at the 15 N m stall (SF 0.47). The motor's 12.8 N weight at about 93 mm from the leg adds about 1.2 N m statically and about 3.6 N m at 3 g rover bumps, giving about 5 MPa at the limiter plus bumps, above 4 MPa. The leg prints upright, so its root bending is across layers too: the calc compares 4.0 MPa at stall with the in-layer 10 MPa, but against the 4 MPa across-layer allowable it sits exactly at the limit. The leg foot is only 30 mm in x with screws 14 mm apart. _frame_calc itself prints 'motor leg foot screw pull 1071 N vs 600 OVER' at stall, and frame_REPORT.md does not mention it. The calc's adapter rows check the old 10-insert hub ring at R43 (MOT.ring_used, RING_R) rather than the built puck with 8 x M3 at R92.5. So the table validates a part that is not built and misses the weak connector entirely.

Evidence: frame.py:284-287 "bridge = prim.box(lx0, lx1, ly1, min(lip_back, yr) - 2.0, bz0, bz1)" / "xc0 = mx + MOT.LIP_R + 2.0" / "conn = prim.box(xc0, mx + sp[\"plate_x\"][1], ly1, yr + 0.01, bz0, bz1)". frame.py:664 "M = tf.Rx(-90) if s < 0 ...  # print flat". frame.py:310-313 the foot is box(lx0, lx1, fy0, fy1, ...) with feet (lx0 + 8, ...), (lx1 - 8, ...). _frame_calc.py:63 "rows.append((case, \"motor leg bending at the root (in-layer, MPa)\", sig, PLA_IN, ...". _frame_calc.py:47 "used = MOT.ring_used(w)". Run output: "FAULT stall 15 N m  motor leg foot screw pull (span 14 mm) 1071.4 600.0 OVER".

Fix: Lengthen the connector so it carries the torque in-layer. Raise bridge_z to (-40, 40) and widen conn to x 97..111 over the full bridge height, which gives tau = 15000/(0.29 x 80 x 14^2) = 3.3 MPa at stall. Better still, add a second connector on the other side of the window (x <= -LIP_R - 2) down to its own leg or foot, so the plate is held at two points 200 mm apart. Extend the leg foot in -x by 25 mm with 2 more M3 and a gusset along x, so the screw span about Y is at least 40 mm. Fix _frame_calc: use PLA_ACROSS for the leg root and add a conn torsion row. Replace the ring rows with the puck's 8 x M3 at RIM_R (MOT.RIM_N, MOT.RIM_R). Report the stall case in frame_REPORT.md.

> Skeptic (confirmed): Note first: the local w/review checkout is NOT the same as origin/base-v1. Its frame.py:286 is older code with no conn, lip_back or LIP_R. The cited code does exist in origin/base-v1 and origin/w/cad-frame, frame.py:264-345, and I reviewed that version (git archive into /tmp/claude-0/review-scratch-S/tree).

What the code confirms:
(1) Plate geometry. The plate is y -10..0, x mx-104..mx+111, z mz-100..mz+120. The main axis is 165 above the deck, so the plate bottom sits about 65 mm above the deck. In v1 the motor-side plate gets no foot flange: cheek_world[ms] = v1_motor["plate"] and the loop 
>
> Skeptic (confirmed): I could not refute this. One correction first: the claim is about origin/base-v1, not the checked-out w/review. w/review is not a copy of base-v1: 34 files differ under base/cad, and the w/review frame.py has no `conn`, `LIP_R` or puck code. Reading origin/base-v1 (git show, plus a git archive copy at /tmp/claude-0/review-scratch-S/bv1), most of the claim holds.

1) The connector is the only load path. In motor_support_v1 the bridge spans y -30 to -18 (ly1 to lip_back-2). Between y -18 and -10, only `conn` (x mx+99 to mx+111, z ±22, so 12 x 44 mm) joins the bridge to the plate. In v1 the motor
>

## 6. No step sets the heat-set inserts in the PPA-CF motor adapter hub, yet 'Motor onto the cheek' screws into them

- File: `base/cad/parts/frame.py:1053`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-frame

Scenario: A builder follows the generated steps. 'Heat-set inserts' (line 1049) tells them to use an iron at about 220 C (PLA), and its parts list is deck + cheeks + ties + frame_ebay, so frame_motor_adapter is left out. The bench step 'Wiper motor on its adapter' never mentions inserts. The next step, 'Motor onto the cheek' (lines 1082-1083), says 'M3 x 10 through the plate into every insert in the adapter hub', but those 4 to 6 ring holes are still empty 4.0 mm bores (_frame_motor.py:151). By then the motor is bolted to the adapter, which makes the inserts awkward to fit. The adapter is PPA-CF (frame.py:753), which needs an iron near 300 C, so the step's 220 C instruction would not seat the inserts even if the builder added the part. The motor cannot be fixed to the cheek as written.

Evidence: frame.py:1050 `text="Soldering iron at about 220 C (PLA) with an insert tip...`; frame.py:1053 `parts=deck + has("frame_cheek") + has("frame_tie") + ["frame_ebay"]),`; frame.py:1082-1083 `M3 x 10 through the plate into every " "insert in the adapter hub`; _frame_motor.py:151 `cut.append(prim.place(U.insert_hole(0, 0, 0.0), tf.T(x, y, tz + SPIGOT_H)))   # inserts in the hub face`; frame.py:753 frame_motor_adapter material "PPA-CF".

Fix: Add frame_motor_adapter to the 'Heat-set inserts' parts list. Extend its text: 'adapter hub (PPA-CF): iron at about 300 C, %d inserts in the hub face' (using len(MOT.ring_used(...))). Alternatively, put the same instruction at the start of 'Wiper motor on its adapter (bench)', before the M6 bolts go in.

> Skeptic (confirmed): The defect is real, but the claim's line numbers are wrong for this checkout (HEAD 8d038b5). In base/cad/parts/frame.py, the 'Heat-set inserts' step is at lines 982-986. Its parts list is `deck + has("frame_cheek") + has("frame_tie") + ["frame_ebay"]` and it gives one temperature, 220 C for PLA. Its text covers the deck, cheeks, bay, standoffs and ties, and never mentions the adapter. 'Wiper motor on its adapter (bench)' (1005-1011) covers only the M6 bolts. 'Motor onto the cheek' (1012-1020) says 'M3 x 10 through the plate into every insert in the adapter hub (%d of the 12 holes ...)'. _frame
>
> Skeptic (confirmed): I couldn't refute this. The claim's line numbers are about 70 lines off in the current tree (8d038b5), but the code does what it says. In base/cad/parts/frame.py, the 'Heat-set inserts' step (lines 982-986) says to use an iron at about 220 C (PLA). Its text lists deck, cheeks, bay, standoffs and ties, and its parts are deck + has("frame_cheek") + has("frame_tie") + ["frame_ebay"]. The adapter hub is in neither. 'Wiper motor on its adapter (bench)' (lines 1006-1011) covers only the M6 bolts and centring, with no inserts. 'Motor onto the cheek' (lines 1012-1020) then says 'M3 x 10 through the pl
>

## 7. The motor-leg step says M3 x 16 for the bridge screws, which bottom out before clamping and push the bridge inserts out; it also says 2 deck screws where the leg has 4

- File: `base/cad/parts/frame.py:1081`
- Severity: medium (claimed high); 1 of 2 skeptics confirmed
- Owner: cad-frame

Scenario: The builder follows the 'Motor onto the cheek' step. The leg's head counterbore ends at y -38 (I probed the built solid: head pocket y -55..-38, clamp length 8 mm, y -38..-30). The bridge insert hole runs from y -30 to -24, and solid PLA follows from -24 to -18. An M3 x 16 seated on y -38 would reach y -22, so its tip hits the hole floor 2 mm early. Either the head stays 2 mm proud and the leg-to-bridge joint is never clamped, or further tightening jacks the brass insert back out of the PLA. This joint is the motor end's only path to the deck: the motor plate (z 65..285) has no deck foot in the v1 path. At the 5 N m limiter each of these two screws carries about 208 N of shear (5000 N mm / 24 mm screw spacing), now through a loose joint. The same step also says '2 x M3 x 10 down into the deck', but the leg foot has 4 holes. With only 2 screws (e.g. both at x 86), the foot tips about its far edge with the screws 8 mm from it: about 5000/(2 x 8) = 312 N per screw at 5 N m and about 940 N at the 15 N m stall, which is at or above the ~1000 N insert pull-out.

Evidence: frame.py:1081-1082 "(stand the motor leg on the deck first: 2 x M3 x 10 down into the deck, " "2 x M3 x 16 along y through the leg into the plate bridge)". The part note contradicts it, frame.py:342-343: "4 x M3 x 10 into the deck, 2 x M3 x 12 along y into the bridge". Geometry, frame.py:307-308: leg - cylY(HEAD_D/2, ly0 - 1, ly1 - 8.0, ...) gives an 8 mm clamp length, and cuts.append(yfeat(U.insert_hole(0, 0, 0.0), xc, z, ly1, 1)) gives a 6 mm deep hole (_frame_util INS_L = 6.0). Leg feet, frame.py:313: 4 points (86,-69),(100,-69),(86,-16),(100,-16).

Fix: Change the step text at frame.py:1081-1082 to '4 x M3 x 10 down into the deck, 2 x M3 x 12 along y through the leg into the plate bridge'. An M3 x 12 tip ends at y -26, 4 mm into the 5 mm insert. Build both strings from the same constants as the part note so they cannot drift again.

> Skeptic (confirmed): I checked this against origin/base-v1, which is the code the claim's line numbers match. The local w/review checkout is an older tree: there the step text is at frame.py:1012-1015 and the leg has only 2 feet. In origin/base-v1:
- frame.py:311 cuts the head counterbore with cylY(HEAD_D/2, ly0-1, ly1-8.0). With leg_y = (-55, -30) the pocket ends at y -38, so the clamp length is 8 mm (y -38..-30).
- frame.py:312 adds yfeat(U.insert_hole(...), xc, z, ly1, 1). INS_L = 6.0, so the insert hole runs y -30..-24.
- The bridge (frame.py:284) is solid from y -30 to -18. Behind the hole floor is solid PLA,
>
> Skeptic (refuted): The claim describes code that is not on this branch (w/review, HEAD 8d038b5). On this branch, frame.py:1081 is not the motor-leg step at all. The 'Motor onto the cheek' step is at frame.py:1012-1015.

How each part of the claim compares with this branch:

1. "Part note says 4 x M3 x 10 into the deck, 2 x M3 x 12 into the bridge" is wrong. The part note at frame.py:303-304 says "2 x M3 x 10 into the deck, 2 x M3 x 16 along y into the bridge", which matches the step text word for word. There is nothing between them to drift.

2. "Leg has 4 deck holes" is wrong. frame.py:281 sets feet = [(bx0+7, 
>

## 8. Fuse-holder tie slot is cut mostly inside the bay outer wall, leaving a 0.3 mm skin (a hole once sliced)

- File: `base/cad/parts/_frame_ebay.py:146`
- Severity: low (claimed low); unverified (low)
- Owner: cad-frame

Scenario: PLACE fuse y = 6.0, so _tie_pair gets ya = fy - 4.5 = 1.5. The slot box spans y 0.3..2.7 and z -1..6, but the wall is y 0..2.4. I sectioned bay_box at x 73..74: only the outer 0.3 mm of wall survives over z 0..6. That is below one 0.4 mm extrusion line, so the slicer drops it and leaves a 4.4 x 6 mm hole through the outer wall. Only 0.3 mm of the slot opens into the bay floor, so the fuse tie has to be forced sideways through a recess in the wall. The bay ends up with an unintended opening and a fuse hold-down that barely works.

Evidence: _frame_ebay.py:68 `fuse=(46.0, 6.0, 0.0),`; :146 `cuts += _tie_pair(fx + fl / 2, fy - 4.5, fy + fw_ + 4.5)`; :85 `c = [prim.box(xc - 2.2, xc + 2.2, yy - 1.2, yy + 1.2, -1, FLOOR + 3) for yy in (ya, yb)]`; :48 `WALL, FLOOR = 2.4, 3.0`. Measured wall and floor volume in x 73..74, y 0..5, z 0..8: 10.74 mm3 (the intact section is about 27 mm3).

Fix: Keep the near slot inside the bay: `cuts += _tie_pair(fx + fl / 2, max(fy - 4.5, WALL + 1.4), fy + fw_ + 4.5)`. That puts the slot at y 2.6..5.0, clear of the wall and of the fuse body (y 6+), with the cradle walls at x 54..58 and 89..93 not in the way.


## 9. Bay sensor and motor wire exits face the solid motor leg 5 mm away (sensor window 100% blocked)

- File: `base/cad/parts/_frame_ebay.py:151`
- Severity: low (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-frame

Scenario: Default params. place_bay uses the params region (x 110..240, yaw 90), so M_bay puts the bay's y = W wall (the exit wall) at world x = 113. EXIT_SENS (local x 14..30) maps to world y -53..-37 and EXIT_MOTOR (local x 33..45) to world y -34..-22, both at deck + 5..15 mm. frame_motor_leg is a solid 30 x 25 column at x 78..108, y -55..-30 at every height, with gussets out to y -69..-16 at the bottom. I intersected the leg with a box x 106..108 in front of each window: the sensor window is 100% solid (320/320 mm3) and the motor window is partly blocked. The sensor bundle (switches, N20, AS5600, load cell) and the 14 AWG wiper pair therefore come out into a 5 mm slot against a flat wall and must bend 90 degrees inside 5 mm. A 14 AWG pair cannot do that without kinking, and the tie anchors just inside the windows then strain the wires against the window edge. The region branch of place_bay (lines 307-317) checks neither the exits nor the USB zone against footprints, which is how this got through. NOTES:76 claims the USB clearance is 'checked', but that code path does not run for the region branch.

Evidence: _frame_ebay.py:58-59 `EXIT_MOTOR = (33.0, 45.0)` / `EXIT_SENS = (14.0, 30.0)`; :150-151 cut both windows in the y = W wall; :307-317 region branch returns `tf.T(...) @ M0` with no footprint, exit or usb_zone test; params.py:245 `ebay=dict(x=(110.0, 240.0), ...)`. Measured: M_bay row x = -y_local + 237; frame_motor_leg slice at z -100: x 78..108, y -55..-30; frame_ebay world x 113..237.

Fix: Give the exits room: set params FRAMEWORK.ebay x=(135.0, 265.0), which leaves 30 mm between the leg face (x 108) and the exit wall (x 138); the deck already reaches x 269.4. In place_bay's region branch, also run the same tests as the search branch: a 25 mm deep zone in front of each exit window and the 45 mm USB zone must not intersect _footprints(parts) (plus info['keepout']), and append a warning (which reaches frame.checks) when one does.

> Skeptic (confirmed): The geometry is real. I checked it by building the full assembly from a scratch copy (/tmp/claude-0/review-scratch-S/baseA, default params). frame_motor_leg has a world bbox of x 76..108, y -77..-30, running from the deck to z 22. frame_ebay has a world bbox of x 113..237, y -67..57, so its y = W exit wall is at world x 113 and the gap to the leg face is 5 mm. The claimed intersection test reproduces exactly. A 2 mm box at x 106..108 in front of the sensor window (world y -53..-37, deck + 5..15) is 320/320 mm3 solid leg. The same box in front of the motor window (y -34..-22) is 64/192 mm3 soli
>
> Skeptic (confirmed): The geometry is real. I ran frame.build(load_params()) from a scratch script (/tmp/claude-0/review-scratch-S/bay_exit.py, run read-only and the tree is unchanged) and got these results:
- M_bay in world = rows [0 -1 0 237; 1 0 0 -67; 0 0 1 -165], so the bay's y = W exit wall sits at world x = 113.
- The frame_motor_leg bbox is x 76..108, y -77..-30.
- In front of the sensor window (local x 14..30, FLOOR+2..+12): a zone 0..5 mm out is clear, but 5..10 mm out is 100% solid leg (800/800 mm3), and a 25 mm zone is 3200/4000 mm3 leg.
- In front of the motor window (local x 33..45): 160/960 mm3 block
>
