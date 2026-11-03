# Round 2 J: builder dry run (Harry building from the repo alone)

Reviewed at base-v1 commit c631495. Method: independent finder subagents, one per lens; every finding of
medium severity or above was checked by two skeptic subagents told to refute it; kept only if fewer than
two refuted it. Low findings are listed unverified. Severity is the finder's, lowered when both skeptics
agreed it is real but rated it lower.

Confirmed: 28. Refuted and dropped: 0.

Re-checked 14:10 Sydney against base-v1 863d58d, one checker subagent per finding: see the Now column.

| # | Sev | File:line | Owner | Finding | Now (863d58d) |
|---|---|---|---|---|---|
| 1 | high | `base/cad/parts/launch.py:121` | cad-launch | SW_RAIL_LOW contact has no part: the assembly step installs three parts that do not exist, and the pack has no switch body for it | fixed |
| 2 | high | `base/cad/parts/launch.py:233` | cad-launch | Spring fallback anchor hole set and spring guard referenced for safety do not exist in the launcher CAD | partly fixed |
| 3 | high | `base/cad/parts/launch.py:467` | cad-launch | Carriage stop is a rigid TPU block, not the D26 PLA flexure; Harry has no TPU, and the step's fallback (solid PLA block) gives almost no travel | fixed |
| 4 | high | `base/cad/parts/launch.py:684` | cad-launch | Assembly steps fit the launch springs and hand-time the big gear with springs on, against the guide's 'springs off until stage 5' | still open |
| 5 | high | `base/cad/parts/launch.py:697` | cad-launch | SW_GAP and SW_RAIL_LOW contacts and the AS5600 bracket have no CAD part and no build step, but the firmware treats them as fitted by default | fixed |
| 6 | high | `base/cad/parts/reel.py:878` | cad-reel | SW_ARM (wrap cup at home) has no part and no build step, so every wrap faults | partly fixed |
| 7 | high | `base/spec/params.py:111` | cad-launch | AS5600 board has no mount anywhere in the model (no bracket, no pegs, no BOM line), but the guide says to heat-stake it onto printed pegs | partly fixed |
| 8 | high | `base/tools/steps.py:31` | cad-infra | plan_steps silently drops 10 assembly steps, among them buck setting, motor-to-cheek and big-gear timing | fixed |
| 9 | high | `base/tools/steps.py:52` | cad-infra | Generated assembly order fits the drum before the deck, wiper motor and motor adapter exist | still open |
| 10 | high | `docs/build/GrappStar_Build_Guide.md:1073` | docs | Committed Build Guide, its PDF and the base wiring cheat sheet carry an old pin map that does not match pins.h (drum, HX711, contacts, SLP) | fixed |
| 11 | high | `jetson/grappstar/comms/base.py:105` | jetson | Jetson serial open drops DTR while RTS is still asserted: the ESP32-S3 USB-Serial-JTAG resets on every port open | still open |
| 12 | medium | `base/cad/parts/drum.py:1147` | cad-drum | Drum B fits on drum A's plain hex in 6 orientations and the step gives no index, so code bands and fork groove can end up out of phase with the brake cam, wrap segment and lock | still open |
| 13 | medium | `base/cad/parts/frame.py:846` | cad-frame | BOM fastener totals are about 10 screws, and the frame steps alone need 37+ heat-set inserts, 49 M3x25 + nylocs + washers and M6 motor bolts | still open |
| 14 | medium | `base/cad/parts/launch.py:674` | cad-launch | Launcher steps need clips and an O-ring that are neither printed parts nor bought items, and Harry does not have them | partly fixed |
| 15 | medium | `base/cad/parts/launch.py:712` | cad-launch | Aim 3 cannot be done in the generated order: the launch sleeve has to slide axially over the fixed main-shaft spigot and into the bolted aim-housing bushing | partly fixed |
| 16 | medium | `docs/build/MEASURE_LIST.md:20` | docs | MEASURE_LIST A6 records wiper free speed from a 40 % duty take-up jog as WIPER.n0_rpm | fixed |
| 17 | medium | `docs/build/MEASURE_LIST.md:103` | cad-reel | Payout brake / clutch trim refers to shims in the 'reel assembly steps' that neither the steps nor the part list contain | partly fixed |
| 18 | medium | `docs/build/TEST_PLAN.md:83` | docs | Test stage 4 'cock and fire with no springs' cannot pass: with no springs the carriage never returns, so the firmware faults every cycle | fixed |
| 19 | low | `base/cad/parts/launch.py:460` | cad-launch | BOM lists 12 phantom bought items (spring segments and carriage-end hooks with no bought_ref) | still open |
| 20 | low | `base/cad/parts/launch.py:648` | cad-launch | Launcher 1 print list gives conrod and lug as PLA, but the model and pack print them in PPA-CF | still open |
| 21 | low | `base/cad/parts/reel.py:569` | cad-reel | Brake rocker is PETG, which Harry does not have; the clutch spring thickness is computed for PETG stiffness, so a PLA or PPA-CF print overloads the payout brake | still open |
| 22 | low | `docs/build/MEASURE_LIST.md:104` | docs | Contact gap numbers disagree: G12 says 1.1 mm for every contact, the steps say 0.25, 1.5 and 1.8 mm | fixed |
| 23 | low | `docs/build/SYSTEM_ELECTRICAL.md:126` | docs | SYSTEM_ELECTRICAL points to a 'flash from the Jetson' procedure that does not exist | fixed |
| 24 | low | `docs/build/src/03_parts.md:57` | docs | Tool list misses the drill and 3.4 mm bit, the heat-set insert tip and the 13 mm socket that the steps require | fixed |
| 25 | low | `docs/build/src/04_printing.md:22` | docs | Guide never picks up the risk coupon test sheet: its include glob names a folder and file that do not exist, so the day-one coupon section stays 'NOT IN THE REPO YET' | fixed |
| 26 | low | `docs/build/src/04_printing.md:22` | docs | Guide's risk-coupon section stays 'NOT IN THE REPO YET' even after rebuilding, because make_guide looks for the wrong file names | fixed |
| 27 | low | `docs/build/src/05_base_assembly.md:19` | docs | Guide's motor-adapter screw rule (6 x M3 x 12) disagrees with the frame step (M3 x 10, 10 of 12 holes) | fixed |
| 28 | low | `firmware/base/CALIBRATION.md:101` | docs | Drum manual release is described three different ways (thumb screw, swing screw, hand latch), and no latch exists | partly fixed |

## 1. SW_RAIL_LOW contact has no part: the assembly step installs three parts that do not exist, and the pack has no switch body for it

- Now (863d58d): **fixed** at `base/cad/parts/launch_aim.py:488`. launch_aim.parts_aim now creates launch_sw_rail_low, launch_sw_rail_low_screw and launch_sw_rail_low_nut, placed on a pedestal of the aim housing (lines 332-338, pilot holes cut at 359-360). launch.parts(params) now returns all three, so the 'Aim 1' step's part list (launch.py:1222) matches real parts.

- File: `base/cad/parts/launch.py:121`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: Harry reaches step 'Aim 1', which says 'Build SW_RAIL_LOW like SW_HOME and screw it to the housing where the crank bump presses it'. The print pack only holds launch_sw_home and launch_sw_seated, there is no rail-low body, and the aim housing has no pilot holes for one. With nothing on PIN_SW_I (GPIO40) the aim axis cannot home: BASE_SPEC 10 line 174 says 'aim ... homes on SW_RAIL_LOW first'. Every aim command then faults or never homes. Proof: in the exported asm.json the step lists ['launch_sw_rail_low','launch_sw_rail_low_screw','launch_sw_rail_low_nut'], but none of the three is in asm.parts, and steps.plan_steps prints 'unknown part' warnings for them.

Evidence: launch.py:121-136 switch_frames() returns only out["home"] and out["seated"]; launch.py:698-705 step text "Build SW_RAIL_LOW like SW_HOME ..." parts=[..., "launch_sw_rail_low", "launch_sw_rail_low_screw", "launch_sw_rail_low_nut"]; launch.py:649 says "the three switch bodies (flat)"; firmware/base/grappstar_base/pins.h:37 `#define PIN_SW_I 40 // SW_RAIL_LOW`; BASE_SPEC.md:74 assigns SW_RAIL_LOW to cad-launch.

Fix: In switch_frames(), add out["rail_low"] with a frame on launch_aim_housing at the crank bump (joint of the housing, screw 6), so that parts() makes launch_sw_rail_low, launch_sw_rail_low_screw (M3x6 SHCS) and launch_sw_rail_low_nut (M3 nut), and cut the SW.mount_holes pilots into aim_housing. Re-run the audit and the pack.

> Skeptic (confirmed): I could not refute this; the claim is accurate. switch_frames() in base/cad/parts/launch.py (lines 120-139) returns only "home" and "seated". parts() at lines 470-477 builds the switch bodies, screws and nuts only from that dict. I ran L.parts(params) and got just launch_sw_home, launch_sw_home_screw, launch_sw_home_nut, launch_sw_seated, launch_sw_seated_screw and launch_sw_seated_nut. launch_aim.py makes no switch either: it has no 'sw' or 'rail_low' hits. A repo-wide grep for rail_low/RAIL_LOW under base/cad finds only the step text and part list at launch.py:698-705, which names launch_sw_
>
> Skeptic (confirmed): I could not refute this; the defect is real. In base/cad/parts/launch.py:120-138, switch_frames() returns only "home" and "seated". parts() at launch.py:470-477 is the only place that builds launch_sw_* bodies, screws and nuts, and it loops over switch_frames(). I ran launch.parts(params) and got only launch_sw_home{,_nut,_screw} and launch_sw_seated{,_nut,_screw}, with no launch_sw_rail_low*. A grep of the whole repo finds the string 'sw_rail_low' only in launch.py, and only in the step at line 705. Nothing in _aim() or the aim housing models a rail-low switch: there are no SW.mount_holes pil
>

## 2. Spring fallback anchor hole set and spring guard referenced for safety do not exist in the launcher CAD

- Now (863d58d): **partly fixed** at `base/cad/parts/launch.py:288-291`. The spring guard is now built: frame.py:903 adds the frame_spring_guard_* C-channels from _frame_rguard.py. The fallback anchor hole set is still missing. The anchor-pin loop cuts holes only at u_anchor, one at y_spring and one at the Y_SPRING2 provision, so there is no rearward hole for spring_preload_fallback (params.py:184). B4 in base/site/index.html and BASE_SPEC.md:136 still tell the builder to use that hole set.

- File: `base/cad/parts/launch.py:233`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: Harry does B2. If the springs come out at the estimated rate or stiffer, full cock is over the 38.4 N C-243 rating. B4 and safety section 2 say 'use the fallback hole set' (250 mm full cock). The rail has one anchor hole per side, so there is nothing to switch to, and the hooks are overloaded on every shot. The 'printed spring guard' (D33, params spring_guard=True) is not produced either. A hook that fails at full draw is unguarded.

Evidence: launch.py:231-233: `# anchor pins ...: vertical holes through the posts` / `for sg in (1, -1): solid = solid - cyl_w(3.1, n["w_floor0"] - 1, wsp, ua, sg * ys)` (one hole per side at u_anchor). params.py:178 `spring_preload_fallback=8.8` and params.py:180 `spring_guard=True` are not read anywhere under base/cad or base/tools (grep finds nothing). MEASURE_LIST.md:34 'A fallback anchor hole set gives 250 mm at full cock ... use the fallback hole set'. Guide line 184-185 'fit the printed spring guard if the pack has one ... use the fallback hole set (MEASURE_LIST B4)'.

Fix: In launch.py add a second anchor hole per side at u_anchor - (spring_preload - spring_preload_fallback) = 15 mm rearward of the main one, labelled 'F' on the post. Add a printed guard part over both springs (params spring_guard). Name both in the stage-5 spring-fitting step.

> Skeptic (confirmed): I checked the claim against the code and could not refute it.

1. One anchor hole per side. The only anchor-hole cut in rail_solid is at base/cad/parts/launch.py:231-233: `for sg in (1,-1): solid = solid - cyl_w(3.1, n["w_floor0"]-1, wsp, ua, sg*ys)`. That is one vertical hole per side at u_anchor. u_anchor comes from L_home = spring_free + preload, using the single preload (launch.py:110-111). The parts list (launch.py:453-458) places exactly one launch_anchor_pin_{l,r} at u_anchor.

2. The fallback is never read. `spring_preload_fallback` (params.py:181) and `max_ext_fallback` (params.py:52)
>
> Skeptic (confirmed): I could not refute this. The code bears it out.

**Fallback anchor holes:**
- In base/cad/parts/launch.py:231-233, rail_solid cuts exactly one 6.2 mm vertical anchor hole per side, at ua = n["u_anchor"].
- u_anchor (line 111) comes from n["preload"], which launch_kin.py:61 sets as max(5, max_ext - stroke). That uses only SPRING.max_ext (265). Nothing anywhere computes a second hole for max_ext_fallback (250) or spring_preload_fallback (8.8).
- grep finds spring_preload_fallback, max_ext_fallback and spring_guard only in base/spec/params.py. Nothing under base/cad or base/tools reads them (pyca
>

## 3. Carriage stop is a rigid TPU block, not the D26 PLA flexure; Harry has no TPU, and the step's fallback (solid PLA block) gives almost no travel

- Now (863d58d): **fixed** at `base/cad/parts/launch_stop.py:265`. launch_stop_pad_r is gone. launch.py:792 now adds ST.stop_parts, which gives two PLA flexure springs and frames (launch_stop_spring_l/r and launch_stop_frame_l/r, TRAVEL 19.0), and Launcher 1 (launch.py:1136) now says the stop springs print at 100% infill. test_launch_stop.py:42-43 checks the 19 mm travel and an energy of at least 3 J. The only TPU traces left are an unused comment on PAD_L (launch.py:583) and a 15 mm default for stop_pad_travel in physics/common.py:316, which spec_bridge overrides.

- File: `base/cad/parts/launch.py:467`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: D26 sets the carriage stop as a printed PLA flexure with about 19 mm travel (2.8 to 3.3 J per shot needs about 19 mm for a 300 N peak), because there is no TPU in hand. The model instead ships launch_stop_pad_r, an 18 x 5 x 14 mm box in TPU, and the pack would put it on a TPU plate. Launcher 1 tells Harry to print it in PLA at 15 percent gyroid 'and replace it when it cracks'. Harry follows that: a PLA block compresses well under 1 mm, so the full carriage energy stops in that distance. The peak load is many times 300 N, on the carriage stop tab, the rail mid piece pocket and the rack, on every dry or real shot. Only one pad is modelled, yet the carriage comments speak of 'rail stop pads'.

Evidence: launch.py:467-469 Part("launch_stop_pad_r", B(0, PAD_L, -5.0, 0, 0, 14), "TPU", ... notes="Carriage stop pad"); launch.py:404 PAD_L = 18.0 # (physics F3: 15 to 20 mm of TPU or foam); launch.py:649-650 "TPU 95A: the carriage stop pad (if you have no TPU, print it in PLA at 15 percent gyroid and replace it when it cracks)"; DECISIONS.md:34 D26 "Carriage stop = printed PLA flexure with about 19 mm travel (no TPU in hand; TPU pads are an optional upgrade)".

Fix: Replace launch_stop_pad_r with the D26 PLA flexure (a curved or serpentine beam sized for about 19 mm of travel at about 300 N peak, in the pocket at launch.py:239-240), material "PLA". Fix the Launcher 1 and Launcher 3 text to match, with TPU named only as an optional upgrade. Add a physics or audit check that the stop travel is 19 mm or more.

> Skeptic (confirmed): I checked the claim against the code and it holds. launch.py:467-469 models launch_stop_pad_r as a solid box, B(0, 18, -5, 0, 0, 14), with material "TPU". PAD_L = 18 (line 404) and its comment cites "TPU or foam". The pocket at lines 239-240 is sized for that block. Launcher 1 (lines 649-650) says to print it in TPU 95A, or "if you have no TPU, print it in PLA at 15 percent gyroid and replace it when it cracks". Launcher 3 presses that same pad in. The spec says otherwise. DECISIONS.md D26 calls for a printed PLA flexure with about 19 mm of travel, says there is no TPU in hand, and makes TPU p
>
> Skeptic (confirmed): I could not refute this. On branch w/review the code says exactly what the claim says. launch.py:467-469 defines one part, launch_stop_pad_r, a solid box B(0, 18, -5, 0, 0, 14) with material "TPU". launch.py:404 says PAD_L = 18 and cites "15 to 20 mm of TPU or foam". The only pocket is at launch.py:240 (on -y only). The Launcher 1 text at launch.py:649-650 says to print the pad in TPU 95A, or in PLA at 15 percent gyroid if there is no TPU, "and replace it when it cracks". Launcher 3 says to press "the stop pad" into its pocket.

The spec says otherwise in two places, and nothing later override
>

## 4. Assembly steps fit the launch springs and hand-time the big gear with springs on, against the guide's 'springs off until stage 5'

- Now (863d58d): **still open** at `base/cad/parts/launch.py:1179`. The step generator still includes "Launcher 7: springs" at launch.py:1179, which hooks both springs on with the anchor pins. "Launcher 9" at launch.py:1199-1202 still says to turn the drive gear by hand until "at full cock the cut last tooth lets go". Both contradict GrappStar_Build_Guide.md:691 and FIRST_DAY.md:51, which say the springs stay off until test stage 5. TEST_PLAN.md stage 4 now uses rubber-band stand-ins (lines 91-92), but stage 5 at line 115 still only says "Fit both springs", with no procedure for battery out, HOME check or anchor-hole choice.

- File: `base/cad/parts/launch.py:684`
- Severity: high (claimed critical); 2 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: The generated step 19 'Launcher 7: springs' hooks both springs on during base assembly. Launcher 8 then has the builder turn the drive gear by hand until the carriage is drawn ('at full cock the cut last tooth lets go'). That is a hand-cocked dry fire of about 2 x 40 N on hooks near their rating. Then TEST_PLAN stage 4 ('NO springs') runs arm/fire on the bench with the springs actually on: full-energy dry shots, no dart, before the 30 m clear-area rule starts at stage 5. Meanwhile TEST_PLAN stage 5 just says 'Fit both springs' with no procedure: no carriage-at-HOME check, no battery-out step, no B4 hole choice.

Evidence: launch.py:684-690 'Launcher 7: springs' text: 'Hook the front end of each spring over its anchor pin and the back end over the carriage peg'. launch.py:691-695 'turn the drive gear FORWARD by hand ... at full cock the cut last tooth lets go'. Guide line 350: '- Leave the springs off until test stage 5 (section 12).' FIRST_DAY.md:48: '- Do not fit the launch springs (they go on at test stage 5).' TEST_PLAN.md:73 '## Stage 4. Base assembled, dry cycles, NO springs, no line'. TEST_PLAN.md:103: 'Fit both springs. Start at the lowest pitch'.

Fix: Take the springs out of the assembly sequence. Rewrite Launcher 7 as 'Spring anchor pins only (springs go on at test stage 5)'. Do Launcher 8 timing with no springs: push the carriage onto its pad by hand and mesh the first tooth after the gap with the rearmost rack tooth, without turning on to full cock. Move the spring-hooking text into TEST_PLAN stage 5 as a procedure: battery out, lead #2 out, telemetry `cock` 0 and `H` closed, choose the anchor hole from B4, hook front then rear, glasses on.

> Skeptic (confirmed): I could not refute this one. The cited lines say what the claim says, and they reach the builder. base/cad/parts/launch.py:684-695 define step "Launcher 7: springs": push the anchor pins in, then hook the front of each spring over its pin and the back over the carriage peg, with preload. Step "Launcher 8" then says to turn the drive gear FORWARD by hand, that turning on must pull the carriage back, and that "at full cock the cut last tooth lets go". With the springs already hooked, that is a hand-cocked release at full spring energy.

These steps are not dead text. base/tools/assembly.py:70-72
>
> Skeptic (confirmed): I could not refute this one. The quoted text is real and it reaches the builder. In base/cad/parts/launch.py:684-695, steps() includes 'Launcher 7: springs' ('Hook the front end of each spring over its anchor pin and the back end over the carriage peg'). Its parts list holds the spring segments and hooks. Next comes 'Launcher 8: time the big gear', which tells the builder to turn the drive gear FORWARD by hand with the carriage on its pad and says 'Turning on must pull the carriage back smoothly; at full cock the cut last tooth lets go.' base/tools/assembly.py:70-72 collects every subsystem's 
>

## 5. SW_GAP and SW_RAIL_LOW contacts and the AS5600 bracket have no CAD part and no build step, but the firmware treats them as fitted by default

- Now (863d58d): **fixed** at `base/cad/parts/launch.py:793 (SW_GAP part loop), launch.py:449 (cam bump on big gear), launch_aim.py:233 and 239 (SW_RAIL_LOW cam lobe on crank), launch_aim.py:488-495 (launch_sw_rail_low, screw, nut, launch_aim_as5600), launch_aim.py:340 (AS5600 bracket on housing), launch.py:1193 and 1215 (build steps)`. The newest tree builds launch_sw_gap with its screw and nut, adds a cam bump to the big gear, and adds launch_sw_rail_low with a screw, a nut and a cam lobe on the crank. It also has an AS5600 board and a bracket on the aim housing (plus a frame AS5600 bracket in _frame_ebay.py). The build steps now cover each of these, so the firmware's default of treating every switch as fitted matches the hardware.

- File: `base/cad/parts/launch.py:697`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: A builder who follows the repo builds no SW_GAP contact (no cam arc on the big gear, no switch). SW_RAIL_LOW appears only as text: its three part names do not exist, and the crank has no bump. There is no AS5600 bracket, though D23 places it on the aim housing. The default firmware config marks all switches fitted, so SW_GAP reads open: launchExitCheck refuses every drum move out of LAUNCH (LAUNCH_BUSY) and fire is refused. The first `aim` homes forever and faults 'SW_RAIL_LOW never closed in an aim cycle'. The AS5600 has nowhere to mount, and C13 says the wiper will not start without a magnet reading. The base cannot complete a mission.

Evidence: My plan_steps run warned: "step 'Aim 1: aim housing, worm, wheel, crank': unknown part launch_sw_rail_low" (also _screw, _nut). `grep -i 'SW_GAP|bump|5600' base/cad/parts/launch*.py` finds only the Aim 1 text at launch.py:698-699 ('screw it to the housing where the crank bump presses it'). params.py:252 SW_GAP owner cad-launch 'cam arc on the big gear, switch on the right cheek'. params.py:111 'AS5600 board bracket on the aim housing outer side ... owner cad-launch'. config.h:37 `int8_t swBit[SW_COUNT] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, -1};` (all fitted). base_logic.h:643 `if (fitted(SW_GAP) && !closed(SW_GAP)) return "LAUNCH_BUSY";`. base_logic.h:1169 'SW_RAIL_LOW never closed in an aim cycle'.

Fix: Add parts and steps: launch_sw_gap (switch body, screw, nut on the right cheek) and a cam arc on the big gear; launch_sw_rail_low with its screw and nut, plus a bump on the aim crank; and an AS5600 bracket on the aim-housing outer face (pegs for heat-staking, chip 1 mm from the magnet face at y about 204). Give each a build step with its gap setting. Until they exist, document in WIRING.md that the matching swBit must be set to -1.

> Skeptic (confirmed): I checked each part of the claim against the code and could not refute any of it.

(1) SW_GAP: launch.py switch_frames() (lines 121-135) defines only two contacts, "home" and "seated". The parts loop at 470-477 makes launch_sw_{home,seated} plus a screw and nut for each. A grep of base/cad for sw_gap or "gap" finds no cam arc on the big gear and no right-cheek switch. Gear "gap" appears only in tooth geometry and kinematics. No build step mentions SW_GAP. params.py:252 still assigns it to cad-launch.

(2) SW_RAIL_LOW: the step "Aim 1" (launch.py:697-705) lists launch_sw_rail_low, launch_sw_rai
>
> Skeptic (confirmed): I checked this on w/review (a74060e) and the claim holds.
(1) SW_GAP: launch.py switch_frames() (around lines 124-135) only defines 'home' and 'seated'. The parts loop at launch.py:470-477 therefore builds only launch_sw_home and launch_sw_seated, plus their screws and nuts. No step in launch.py mentions SW_GAP. big_gear_solid has no cam arc: grepping launch.py for 'cam' finds nothing, and 'gap' only finds text about the toothless gap.
(2) SW_RAIL_LOW: Aim 1 (launch.py:697-705) lists launch_sw_rail_low, launch_sw_rail_low_screw and launch_sw_rail_low_nut, but nothing creates those parts. The c
>

## 6. SW_ARM (wrap cup at home) has no part and no build step, so every wrap faults

- Now (863d58d): **partly fixed** at `base/cad/parts/reel.py:1272`. The main defect is fixed. reel.py now builds the SW_ARM parts: a bracket, a switch base, a lever and the screw and nuts, all at step 14 (lines 757-767). The cup gear reel_gk now has a raised track with a notch at home (_cup_gear_track, line 624), and there is a new build step called "SW_ARM contact". Two problems remain. First, the step text at line 1272 and the part notes at 726 and 765 describe the old polarity: the bump presses the lever at home, and the builder is told to set the screw so it "just closes there and opens 15 degrees either side". The geometry (track docstring at line 625), reel_NOTES section 10 and the firmware (config.h armHomeOpen=true) all say the reverse: the contact is closed on the track and open in the notch at home for about ±5°. Second, the step still gives no wiring (nut to GND, screw to GPIO42).

- File: `base/cad/parts/reel.py:878`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-reel

Scenario: The reel steps build no SW_ARM contact and no cam bump on the cup gear. The firmware ships SW_ARM as fitted, so during `wrap` it counts 0 cup pulses and raises DRUM_SENSOR 'SW_ARM counted 0 of 7 cup turns'. CALIBRATION (c) step 9 tells the builder to check a contact that was never built.

Evidence: params.py:255 `SW_ARM=dict(owner="cad-reel", closes="wrap cup at home (cam bump on the cup gear, closed +-12 deg)", gpio=42)`. The reel steps at reel.py:878-959 contain no SW_ARM. The part list has no reel switch part (reel_* parts are wall, sprag, c, bus, swing_arm, gi, gc, drum, k, cup, gk, spool, brg, s, t, pushrod, rocker, gr, shroud, exit_post). config.h:37 SW_ARM bit 10 fitted. base_logic.h:485 `"SW_ARM counted %d of %d cup turns"`.

Fix: Add a printed SW_ARM contact (body, M3 screw, M3 nut) on the reel wall and a cam bump on the cup gear, with a build step ('Cup gear' or a new 'SW_ARM contact' step) that gives the gap and the wiring (nut to GND, screw to GPIO42). Alternatively, set swBit[SW_ARM] = -1 by default and document that.

> Skeptic (confirmed): I checked the code and the claim holds.

CAD side:
- Nothing under base/cad mentions SW_ARM.
- reel.py parts() has no contact part. reel_gk (lines 550-553) is a plain _gear() with no cam bump.
- The 'Cup gear' build step (reel.py:918-921) only slides the gear on and fits the clip. No reel step mentions a contact.
- reel_NOTES.md section 10 says outright: 'SW_WRAP_HOME (spec, optional): not fitted. The drum's locking arc puts the cup exactly home...'. So cad-reel chose not to build it, even though the contract lists it as owned by cad-reel (params.py:258 SW_ARM owner cad-reel, gpio 42; BASE_SPE
>
> Skeptic (confirmed): I could not refute this claim. I checked each piece of it against the code.
(1) CAD: nothing under base/cad mentions SW_ARM (grep found zero hits). The cup gear, reel_gk at reel.py:550, is built by a plain _gear(M_CUPG, Z_CUPG, ...) call with no cam bump. The reel build steps (Screw the reel wall ... Exit eye post and line) have no contact step, and no reel part is a switch body. reel_NOTES.md section 10 says outright that the cup-home contact (SW_WRAP_HOME) is 'not fitted', because the drum code is said to identify home versus 7 turns.
(2) Spec and contract: BASE_SPEC section 4 is marked bind
>

## 7. AS5600 board has no mount anywhere in the model (no bracket, no pegs, no BOM line), but the guide says to heat-stake it onto printed pegs

- Now (863d58d): **partly fixed** at `docs/build/src/05_base_assembly.md:24 (and base/cad/parts/frame.py:984); mount now at base/cad/parts/launch_aim.py:340-343,495`. The main defect is fixed. cad-launch now builds an AS5600 bracket into launch_aim_housing (two arms and a plate with M2.5 self-tap holes, launch_aim.py:340-343 and :365-366), adds the bought part launch_aim_as5600 (launch_aim.py:495, chip 1.1 mm past the magnet face), and the "Aim 1" step mounts it (launch.py:1215-1222). Two text problems remain. The build guide still says "heat-stake the board onto its printed pegs" (05_base_assembly.md:24, GrappStar_Build_Guide.md:697), but the new bracket has no pegs and uses 2 x M2.5 x 6 screws. The frame heat-set step text (frame.py:984) still lists a "sensor bracket" that frame no longer builds.

- File: `base/spec/params.py:111`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: params assigns the AS5600 bracket to cad-launch ('AS5600 board bracket on the aim housing', owner cad-launch). frame.py:122 therefore turns off frame_as5600_bracket and frame_pcb_as5600. launch.py and launch_aim.py hold no AS5600 geometry (grep for 5600 finds nothing). The exported asm.json has no AS5600 part, and BOM.md lists no AS5600. When Harry reaches guide section 5 ('AS5600: heat-stake the board onto its printed pegs'), no part has pegs. The heat-set step also names a 'sensor bracket' that does not exist. Without a mount, the wiper angle sensor cannot sit 1 to 2 mm from the magnet, so the firmware has no shaft angle (D12 fallback only).

Evidence: params.py:111 dict(y=(204.0, 210.0), part="AS5600 board bracket on the aim housing outer side ...", owner="cad-launch"); frame.py:122 c["as5600_frame"] = not any("AS5600 board" in e.get("part","") and e.get("owner") != "cad-frame" ...); frame.py:931 `if "frame_as5600_bracket" in names:`; GrappStar_Build_Guide.md:355 "AS5600: heat-stake the board onto its printed pegs"; frame.py:846-848 "cheeks (block rings, ties, motor ring, sensor bracket)".

Fix: cad-launch: add a launch_as5600_bracket (PLA) on the aim housing outer side at y 204..210. Give it 4 pegs for the Adafruit board holes (heat-staked), with the chip on the main axis 1 to 2 mm past the magnet cap face (y 203). Add a bought part launch_pcb_as5600 with bought_ref "Adafruit AS5600 STEMMA QT (PID 6357)" and a step after 'Main shaft: LAUNCH dog ring and magnet cap'. cad-frame: drop 'sensor bracket' from the heat-set step text.

> Skeptic (confirmed): I could not refute this. I checked the code and ran the part builders from base/ with the real params.

1. params.py:111 gives the 'AS5600 board bracket on the aim housing outer side' at y 204..210 to cad-launch.
2. frame.py:122 sets C['as5600_frame'] to False when an AS5600 stack entry has an owner other than cad-frame. Running F.cfg(P) printed as5600_frame False, v1 True. _frame_ebay.py:352 only calls as5600_parts(), which makes frame_as5600_bracket and frame_pcb_as5600, when that flag is True. So frame.parts(P) returns no AS5600 parts, and the 'AS5600 board' assembly step at frame.py:931 is
>
> Skeptic (confirmed): I checked this against the code and could not refute it.

(1) In params.py, the stack entry at y 204..210 reads "AS5600 board bracket on the aim housing outer side" and its owner is cad-launch.

(2) frame.py:122 sets c["as5600_frame"] to False when any stack entry containing "AS5600 board" has an owner other than cad-frame. _frame_ebay.py:352 then skips as5600_parts(), and that function is the only code that makes frame_pcb_as5600 and frame_as5600_bracket (the one with 4 heat-staked pegs). Because of this, frame.py:931 never adds the "AS5600 board" assembly step.

(3) I ran frame.build(P) with
>

## 8. plan_steps silently drops 10 assembly steps, among them buck setting, motor-to-cheek and big-gear timing

- Now (863d58d): **fixed** at `base/tools/steps.py:129-153`. plan_steps now keeps every authored step. A part that was already fitted goes to `handle`, and a step left with no parts becomes a text step. The real-base test (base/tools/tests/test_steps_plan.py:146) asserts len(res) == len(asm.steps) + auto steps, and that assertion passes; the buck, motor-to-cheek, big-gear timing and wiring steps all appear in the output. The test still fails, but for a different reason: ordering warnings. A text step with no parts and no handle is always ready, so 'Launcher 9: time the big gear' is moved ahead of 'Launcher 3: big gear'. That is an ordering defect, not a dropped step.

- File: `base/tools/steps.py:31`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-infra

Scenario: A step whose parts all appeared in an earlier step, or a step with parts=[], disappears completely, text and all. Ten steps are lost from the generated assembly page and guide: frame 'Drill the rover', 'Deck on the rover', 'Motor-side cheek', 'Motor onto the cheek' (the only instruction that bolts the motor and adapter to the cheek), 'Shafts and trains', 'Set the buck BEFORE wiring the DRV8833' ('the DRV8833 dies above 10.8 V'), 'Wiring and lid'; and launch 'Launcher 1: print the parts', 'Launcher 8: time the big gear' (the only HOME timing of the big gear to the rack), 'Launcher 9: wiring'. A builder who follows the generated steps never mounts the motor to the cheek and never times the big gear. With the gear meshed out of time, the first cock either does not draw the carriage or releases at the wrong angle, and the firmware faults ('release at the wrong angle').

Evidence: steps.py:31-32 `elif n in seen: warn.append(... already in an earlier step)` (the part is skipped). steps.py:64 `return [s for s in steps if s["parts"]], warn`. My run printed WARN for frame_deck_*, frame_cheek_r, frame_motor_adapter, frame_ebay, frame_pcb_mp1584/drv8833 and frame_ebay_lid, and my diff against asm.steps listed the 10 titles above as DROPPED. launch.py:691-696 'Launcher 8: time the big gear' has parts=[].

Fix: Keep every authored step. Return `[s for s in steps if s["parts"] or s["text"]]`. When a step names a part that is already fitted, show it as context (not new) instead of removing the step: store it in a `context` list for the renderer. Add a test that the number of authored steps equals the number of non-auto steps in the output.

> Skeptic (confirmed): I couldn't refute this. I ran it and the steps really are dropped. In base/tools/steps.py:33-36, a part that already appeared in an earlier step is left out of the current step's parts. Line 64 then returns only `[s for s in steps if s["parts"]]`, so any authored step that ends up with no parts is removed, text included. Steps written with parts=[] are dropped the same way.

I ran plan_steps on the full Assembly (script at /tmp/claude-0/review-scratch-J/ps.py). Of 45 authored steps, 35 non-auto steps come out. The 10 dropped titles are exactly the ones claimed: Drill the rover, Deck on the rov
>
> Skeptic (confirmed): I could not refute this. The defect is real and I reproduced it on the full assembly. In base/tools/steps.py:24-64, plan_steps skips any part already in `seen` (lines 33-34) and finally returns `[s for s in steps if s["parts"]]` (line 64). A step with parts=[], or a step whose parts were all claimed earlier, is removed along with its title and text. The auto-step pass does not refill them. The only rescue is a Part.step=k pointing at that step, and only reel parts have Part.step set. guide.guide_steps (base/tools/guide.py:69-80) calls the same plan_steps, so the PDF guide loses the same steps.
>

## 9. Generated assembly order fits the drum before the deck, wiper motor and motor adapter exist

- Now (863d58d): **still open** at `base/tools/steps.py:233-264`. I ran plan_steps on the real Assembly.load() in the current tree, which now uses a contact and stage planner. It still puts the drum steps first: step 0 is 'Main shaft: coupler and limiter' and step 2 is 'Drum: bracket A...'. The reel step 'Screw the reel wall to the deck' comes at step 11, while 'Wiper motor on its adapter' (19), 'Heat-set inserts' (20) and 'Deck on the rover' (22) only come later, and the planner raises its own 'moved ahead of' warnings about this. Nothing gives the frame base steps a stage or an order ahead of the other subsystems.

- File: `base/tools/steps.py:52`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-infra

Scenario: Running plan_steps on the real assembly (I ran it: python -c 'Assembly.load(); plan_steps(a)') gives, in order: step 1 'Push the coupler hub onto the wiper output shaft', step 2 'Screw the block to the front face of the motor adapter plate', step 3 'Bolt bracket A to the deck (2 x M3 x 12)', and so on to step 10. Only after that come step 11 'Heat-set inserts' (deck and cheek) and step 12 'Wiper motor on its adapter'. A learner following the assembly page or the guide PDF has to screw into deck inserts that are not set yet, onto a deck that is not laid, and fit a coupler to a motor that is not mounted. It also breaks the guide's own rule 'before any shaft goes in'.

Evidence: assembly.py:39 `return sorted(m.name for m in pkgutil.iter_modules(...))` gives modules in the order drum, frame, launch, reel. assembly.py:71-72 then appends each module's steps in that order. frame.py:835 says its steps are 'in the whole-base order (other subsystems slot in where named)', with frame.py:884 `dict(title="Shafts and trains (other subsystems)", ... parts=[])` as the slot. steps.py:64 `return [s for s in steps if s["parts"]], warn` drops that slot, and nothing in plan_steps ever moves other subsystems' steps into it. guide.py:75 builds the guide PDF from the same plan_steps.

Fix: In plan_steps, put the steps together in base order instead of module order: frame steps up to and including the 'Shafts and trains' placeholder, then drum, launch and reel steps, then the rest of the frame steps. Keep the placeholder as a text-only step. Simplest way: give every step dict an explicit `order` key (frame 10..50, drum 60.., launch 100.., reel 200.., frame second-side 300..) and sort by it. Add a test that 'Bolt bracket A to the deck' comes after 'Heat-set inserts' and that 'Main shaft: coupler' comes after 'Wiper motor on its adapter'.

> Skeptic (confirmed): I ran plan_steps on Assembly.load() myself and the defect is real. Module order is ['drum','frame','launch','reel'], because subsystem_names() sorts alphabetically (assembly.py:39) and Assembly.__init__ adds each module's steps in that order. Nothing in plan_steps (steps.py:24-64) or guide_steps (guide.py:68-80) puts the steps back into base order. The only special ordering in tools is assembly.py:175, and that is for something else. The output starts with the 10 drum steps. Step 1 is 'Main shaft: coupler and limiter' (coupler hub onto the wiper shaft), step 3 is 'Drum: bracket A, axle, rail, 
>
> Skeptic (confirmed): I ran plan_steps on Assembly.load(load_params()). asm.modules comes out in the order ['drum', 'frame', 'launch', 'reel'] (subsystem_names() is sorted alphabetically, and Assembly.__init__ adds each module's steps in that order). Here is the order I got.

- Steps 1-10 are all drum steps. They start with 'Main shaft: coupler and limiter' (its text says "Push the coupler hub onto the wiper output shaft") and 'Drum: bracket A, axle, rail, drum A'.
- Step 11 is frame 'Heat-set inserts' and step 12 is 'Wiper motor on its adapter (bench)'.
- After those come launch, then reel, then the auto-generated
>

## 10. Committed Build Guide, its PDF and the base wiring cheat sheet carry an old pin map that does not match pins.h (drum, HX711, contacts, SLP)

- Now (863d58d): **fixed** at `docs/build/GrappStar_Build_Guide.md:1417-1442`. The committed guide, its PDF and Wiring_Cheat_Sheet_Base.pdf now carry the pins.h map: drum 15/16, HX711 17/18, SW_HOME 21, SW_SEATED 41, SW_N20 47 and no SLP wire. The bench chapter now says GPIO11 for NEUTRAL and GPIO47 for SW_N20 (lines 1569-1570, 2913-2916, 3063), and a regeneration differs only in prose, with no GPIO changes. test_make_guide.py now has test_every_gpio_in_the_guide_matches_pins_h, but it checks build_markdown() output, not the committed snapshot, so the full anti-staleness guard suggested in the fix is still missing.

- File: `docs/build/GrappStar_Build_Guide.md:1073`
- Severity: high (claimed critical); 2 of 2 skeptics confirmed
- Owner: docs

Scenario: Harry wires the base from the Build Guide (or the printed Wiring_Cheat_Sheet_Base.pdf), as docs/build/src/07_electronics.md tells him to. He puts DRV8833 AIN1/AIN2 on GPIO17/18, but the firmware drives GPIO15/16, so the drum never turns. He puts HX711 DT/SCK on GPIO16/15, and those are the pins the firmware drives as drum PWM: hwDrum holds both at 255 (HIGH) for brake at boot, so the ESP32 output fights the HX711 DOUT output (pin contention, possible damage). He puts SLP on GPIO21, which is the SW_HOME input. Every contact lands one or more pins off: guide SW_DRUM_A=11 (fw SW_B), SW_HOME=40 (fw SW_RAIL_LOW), SW_SEATED=39 (fw SW_GAP), SW_DOG=14 (fw SW_DOG_R), with no SW_N20 on 47. The firmware then reads the carriage-home, seated and dog contacts from the wrong mechanisms, and those contacts gate cock, fire and drum shifts. The bench jumper GPIO12 reads as SW_DRUM_C, so the code says WRAP, not NEUTRAL, and there is no GPIO47 jumper, so every drum move gives HAND_MODE. The guide's own bench chapter (lines 2302, 2326, 2341, 2414) is just as stale, so the bench test 'passes' on the wrong wiring. Regenerating the guide in a scratch copy changes about 955 lines, 48 of them with GPIO numbers.

Evidence: GrappStar_Build_Guide.md:1073 '| 17 | `PIN_DRUM_IN1` | DRV8833 AIN1 |'; :1075 '| 21 | `PIN_DRUM_SLP` | DRV8833 SLP (nSLEEP) |'; :1148 'AIN1 GPIO17, AIN2 GPIO18'; :1184 'DT to GPIO16, SCK to GPIO15'; :1218 'DRV8833: AIN1 GPIO17, AIN2 GPIO18, SLP GPIO21'; :1215 and :2302 'jumper from GPIO12 (`SW_DRUM_B`) to GND'. pins.h:24-25 '#define PIN_DRUM_IN1 15 / PIN_DRUM_IN2 16'; pins.h:29-35 'PIN_SW_A 10 ... PIN_SW_G 21 // SW_HOME'; pins.h:44-45 'PIN_HX_DT 17 / PIN_HX_SCK 18'; pins.h:23 'SLP is NOT wired (decision D9)'. Wiring_Cheat_Sheet_Base.pdf lists '17 PIN_DRUM_IN1', '21 PIN_DRUM_SLP', '16 PIN_HX_DT', yet its footer says 'SLP is not wired (decision D9)'. firmware/base/WIRING.md:13-36 (current) agrees with pins.h.

Fix: Run `python3 docs/build/make_guide.py` and commit the regenerated GrappStar_Build_Guide.md/.pdf, Wiring_Cheat_Sheet_Base.pdf and Commands_Cheat_Sheet.pdf. Add a test to docs/build/test_make_guide.py that fails when the committed GrappStar_Build_Guide.md differs from build_markdown() output, or at least when its section 7.2 pin table differs from the table in firmware/base/WIRING.md, so a stale snapshot cannot ship again.

> Skeptic (confirmed): I could not refute this. The claim holds, apart from one wrong path: pins.h is at firmware/base/grappstar_base/pins.h, not firmware/base/pins.h.

1. The committed guide is stale. docs/build/GrappStar_Build_Guide.md:1073-1090 lists DRUM_IN1/IN2=17/18, PIN_DRUM_SLP=21, SW_A..H=11,12,13,14,39,40,41,42 (SW_DOG, SW_SEATED, SW_HOME, SW_WRAP_HOME) and HX DT/SCK=16/15. Lines 1148, 1150, 1184, 1214, 1218, 2302, 2326, 2341 and 2414 repeat the old numbers.
2. pins.h says something else: DRUM_IN1/IN2=15/16, SLP not wired (D9), SW_A..L=10,11,12,13,14,47,21,39,40,41,42,35, HX=17/18. firmware/base/WIRING.md 
>
> Skeptic (confirmed): I could not refute this claim. One correction first: pins.h is at firmware/base/grappstar_base/pins.h, not firmware/base/pins.h. It defines:
- DRUM_IN1/IN2 = 15/16, with SLP not wired (D9)
- contacts A..L = 10, 11, 12, 13, 14, 47, 21, 39, 40, 41, 42, 35 (SW_DRUM_A/B/C, DOG_L, DOG_R, N20, HOME, GAP, RAIL_LOW, SEATED, ARM, INDEX)
- HX_DT/SCK = 17/18

firmware/base/WIRING.md matches pins.h.

The committed docs/build/GrappStar_Build_Guide.md says "Built ... from commit b218750". Its section 7.2 table (lines 1073-1088) and several other lines still carry the old map:
- DRUM_IN1/IN2 = 17/18 and DRUM
>

## 11. Jetson serial open drops DTR while RTS is still asserted: the ESP32-S3 USB-Serial-JTAG resets on every port open

- Now (863d58d): **still open** at `jetson/grappstar/comms/base.py:105-115`. The fix was only half applied. open() now sets rts=False and dtr=False on an unopened Serial and then calls ser.open(). But on an unopened port pyserial only stores those values. Inside open() (serialposix.py:322-338) it calls os.open, which raises both DTR and RTS on Linux cdc-acm. It then applies _update_dtr_state() before _update_rts_state(), so the lines still pass through RTS=1/DTR=0, the reset state. The RTS-first clear after open that the fix asked for is missing, and the docstring wrongly says the order is RTS before DTR.

- File: `jetson/grappstar/comms/base.py:105`
- Severity: high (claimed high); 2 of 2 skeptics confirmed
- Owner: jetson

Scenario: On Linux, opening /dev/grappstar-base (cdc-acm plus pyserial defaults) asserts DTR=1, RTS=1. SerialTransport.open() then sets dtr=False first, giving RTS=1, DTR=0. Per the ESP32-S3 TRM USB-Serial-JTAG CDC table, that state resets the chip (it is the state esptool uses for its USB-JTAG hard reset). It then sets rts=False, so the chip boots normally. Every `python3 -m grappstar base ping/status/cmd` and every BaseClient reconnect therefore reboots the base. If the reset re-enumerates the USB device, the open handle dies, BaseClient reconnects after reconnect_s=1.0 and resets the board again: a reset loop, and the base never answers. Even without the loop, `base cmd '{"cmd":"arm"}'` followed by a second `base cmd` for fire reboots the base in between, and a reconnect mid-mission reboots it while it is reeling or towing. The code comment's premise that the native port does not reset on open is wrong for this toggle order. Not tested on hardware, because the old ground station used the UART port.

Evidence: base.py:101 'self.ser = serial.Serial(self.port, self.baud, timeout=0.05, write_timeout=1.0)'; :102-103 '# The DevKitC native USB port does not reset on open, but a UART bridge port does (DTR/RTS): keep both lines low'; :105 'self.ser.dtr = False'; :106 'self.ser.rts = False'. README/udev: the base is on the native port 303a:1001 (setup/99-grappstar.rules:26).

Fix: Clear RTS before DTR so the lines never pass through RTS=1/DTR=0. After open: `self.ser.rts = False` then `self.ser.dtr = False` (lines 1,1 -> 0,1 'set download flag', no reset -> 0,0 'clear flag'). Better, build the port unopened, set rts/dtr, then open, and keep the explicit RTS-first clear after open because pyserial updates DTR before RTS. Correct the comment. Bench-check it with `base ping` twice in a row: no HELLO line and reset count unchanged.

> Skeptic (confirmed): I could not refute this. The code says what the claim says. jetson/grappstar/comms/base.py SerialTransport.open() builds serial.Serial(port, ...) already open, then sets `self.ser.dtr = False` and only after that `self.ser.rts = False` (lines 101-107).

How the lines move on Linux:
- On open, the cdc-acm driver raises DTR and RTS together in one SET_CONTROL_LINE_STATE request, giving DTR=1, RTS=1.
- pyserial's open() then re-asserts both, because the defaults are True.
- `ser.dtr = False` is a TIOCMBIC ioctl. cdc-acm sends a new control-line state at once with RTS still set, so the device sees
>
> Skeptic (confirmed): I could not refute this one. Every link in the chain checks out:
- jetson/grappstar/comms/base.py:101-106 opens the port with pyserial defaults, then sets `ser.dtr = False` and only after that `ser.rts = False`.
- The base firmware really is on the hardware USB-Serial-JTAG peripheral, not TinyUSB. The .ino header says "USB Mode: Hardware CDC and JTAG". The udev rule at jetson/setup/99-grappstar.rules maps 303a:1001 (Espressif USB JTAG/serial) to /dev/grappstar-base and repeats the "Hardware CDC and JTAG" requirement. That makes the silicon DTR/RTS reset logic apply.
- On Linux, cdc-acm raises 
>

## 12. Drum B fits on drum A's plain hex in 6 orientations and the step gives no index, so code bands and fork groove can end up out of phase with the brake cam, wrap segment and lock

- Now (863d58d): **still open** at `base/cad/parts/drum.py:498 and base/cad/parts/drum.py:521 (assembly step text at drum.py:1176-1179)`. The drum A spigot is still a plain regular hexprism(spig_af) and the drum B socket is a plain hexprism(spig_af + HOLE_SLIDE). There is no key, flat, rib or pin, so drum B still fits on in 6 orientations. The "Drum: drum B, hand lever, bracket B" step still says only "hex socket over the drum A spigot" and gives no L mark or alignment instruction.

- File: `base/cad/parts/drum.py:1147`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-drum

Scenario: Drum A carries the N20 gear, the wrap segment gear, the locking arc and the brake cam. Drum B carries the fork groove and the code bands A/B/C. They join on a regular hex spigot and socket. Five of the six ways drum B can go on are wrong by 60, 120 or 180 degrees. For example, the code reads LAUNCH (001) while the groove holds the collar in another zone and the wrap segment or brake cam sits elsewhere. The firmware then trusts the code: DOG timeouts, the cup turning while the line is on the launch path, or the clutch FIRM during payout. Calibration only says 'look' at the collar.

Evidence: drum.py:206-208 `def hexprism(af, s0, s1): """Hex with flats at +-y (vertices on +-x)."""` (a regular hex, no key). drum.py:491 `parts.append(hexprism(g["spig_af"], ...))` (drum A spigot). drum.py:508 `d = d - hexprism(g["spig_af"] + fits.HOLE_SLIDE, ...)` (drum B socket). drum.py:1147 step text: 'slide drum B onto the axle (hex socket over the drum A spigot), swing the fork down so its pin drops into the groove' (no orientation given).

Fix: Make the joint fit one way only: add a key (for example a 2 mm flat or rib on one hex face of the spigot with a matching notch in the socket) or an offset pin. Emboss an 'L' on both drums at the LAUNCH angle. Change the step to 'line up the L on drum B with the L on drum A (both at the top with drum A at LAUNCH), then slide it on'.

> Skeptic (confirmed): I checked the claim against the code and it holds. Line references are to base/cad/parts/drum.py.

The joint has no key:
- `hexprism`/`hex_cs` (lines 200-208) is a plain regular hexagon.
- Drum A's spigot (line 491) is `hexprism(spig_af)` with nothing added to it.
- Drum B's socket (line 514) is `hexprism(spig_af + HOLE_SLIDE)` with nothing added to it.
- No rib, flat, offset pin or marking geometry exists anywhere in either part. The "mark L" in the step at line 1137 is not modelled as geometry, and drum B has no matching mark at all.
- The assembly step at line 1147 gives no orientation.
So 
>
> Skeptic (confirmed): The core defect is real. drum.py:204-206 hexprism is a plain regular hex with no key. drum_a adds it as the spigot (line 491) and drum_b cuts the socket from the same hexprism(spig_af + HOLE_SLIDE) (line 508). Nothing else fixes the angle between the two parts: there is no flat, pin or rib, and no grep hit for a mark or emboss on drum B. So drum B goes on in any of 6 positions, 60 degrees apart. The step text at drum.py:1145-1149 gives no orientation. The only angle reference is "mark L at the top" for drum A in the idler step (line 1137). The groove runs over stop_lo..stop_hi (-4..352, about 
>

## 13. BOM fastener totals are about 10 screws, and the frame steps alone need 37+ heat-set inserts, 49 M3x25 + nylocs + washers and M6 motor bolts

- Now (863d58d): **still open** at `base/cad/parts/frame.py:840`. On HEAD, tools/bom.py still counts only what parts list in bought_ref, and no printed frame part lists any hardware. The deck Part at frame.py:840-841 mentions inserts and rover holes only in its notes. The only bought_ref values in frame.py are the wiper motor (681) and its shaft (893), so the BOM still leaves out the heat-set inserts, the M3x25 bolts with nylocs and washers, the M3x10/x12/x8 screws and the M6 motor bolts that the assembly steps call for (lines 983-1038). Some launch parts (launch_line, launch_muzzle, launch_guard) now list their screws, which only fixes a small part of the problem outside frame.py.

- File: `base/cad/parts/frame.py:846`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-frame

Scenario: bom.py totals only bought_ref entries. No frame part declares its hardware, so the generated BOM.md 'Fasteners' section has 10 items in total (2 M3 nuts, 1 M3x12, 1 M3x25+nut, 1 M3x6, 4 M3x8, 1 M8 bolt) and no inserts at all. The frame steps call for 'every M3 x 4 x 5 insert' (deck 37 plus cheek, bay and clamp posts), 'M3 x 25 SHCS ... washer and nyloc' through each of 49 perimeter holes, M3x10 for the cheek foot and adapter (10), the bay (4 x M3x10, 4 x M3x6 standoffs, 2 x M3x8 clamp, 4 x M3x6 lid) and 'the M6 bolts' for the wiper motor. M6 bolts are outside Harry's inventory (guide line 248 lists only M3 screws, nuts and inserts). Harry checks his box against the BOM, starts, and runs out at the deck-to-rover step or the motor adapter.

Evidence: frame.py:846-848 "Press every M3 x 4 x 5 insert square into its 4.0 mm hole until flush: deck (37), cheeks ..."; frame.py:856 "M3 x 25 SHCS down through every perimeter hole, washer and nyloc under the rover plate"; frame.py:870 "Put the M6 bolts through the adapter slots"; generated BOM.md Fasteners table has 6 rows totalling 10 pieces; GrappStar_Build_Guide.md:248 "| M3 screws, M3 nuts, M3 brass heat-set inserts | | Frame joints and printed contacts |" (no count, no M6).

Fix: Put the hardware into bought_ref on each frame part in the bom.py syntax. frame_deck_k: "<n> x M3x4x5 heat-set insert; <n> x M3x25 SHCS; <n> x M3 nyloc; <n> x M3 washer" from the counted holes. frame_cheek_r: inserts + "<n> x M3x10 SHCS". frame_motor_adapter: "10 x M3x10 SHCS; 4 x M6x<len> bolt (length from MEASURE A)". frame_ebay, lid, clamp and kit parts: their screws. Do the same in drum.py (M3x45 + nyloc, M3x12, x16, x8, x10, x6, M3 nuts, M8 nut) and launch.py (8 x M3x16 + nuts for the rail flanges, 2 x M3x10 per switch, 4 x M3 for the aim housing, 3 x M3x10 + nuts for the dog ring). Add M6 bolts to the guide's parts table as a check-the-box item.

> Skeptic (confirmed): I checked the claim against the code and it holds up.

1. base/tools/bom.py build() counts bought items only from each part's bought_ref. Its docstring says printed parts should list their own fasteners there (example: "4 x M3x10 SHCS ISO 4762; 4 x M3 heat-set insert"). Nothing else adds hardware to the BOM.

2. No frame part lists any fasteners in bought_ref:
   - frame.py uses bought_ref only for the wiper motor (line 597) and the wiper shaft (line 764).
   - _frame_kit.py and _frame_ebay.py use it only for the RC832, the Kiyo and the PCBs.
   - drum.py uses it only for the N20 motor.
   - l
>
> Skeptic (confirmed): I could not refute the core claim. I generated the BOM (python -m tools.bom into /tmp/claude-0/review-scratch-J/out). Its Fasteners table has exactly 6 rows totalling 10 pieces: 2 M3 nuts, 1 M3x12, 1 M3x25+nut, 1 M3x6, 4 M3x8 and 1 M8x25+nut. It lists no heat-set inserts, no M3x25 rover-mount screws, nylocs or washers, no M3x10, and no M6 bolts. The mechanism is as stated: bom.py build() counts only parse_ref(bought_ref). The bom.py docstring shows that printed parts are meant to declare their fasteners there ("4 x M3x10 SHCS; 4 x M3 heat-set insert"). No frame, drum or launch printed part doe
>

## 14. Launcher steps need clips and an O-ring that are neither printed parts nor bought items, and Harry does not have them

- Now (863d58d): **partly fixed** at `base/cad/parts/launch_aim.py:457-473 (and launch.py:1224-1235)`. Launcher 5 is fixed: the big gear pin now has a printed launch_gear_clip, and the step lists it (launch.py:755, 1154). The rest is still open. Aim 2 still says "Put the O-ring in the swing arm hub groove" and "fit the idler on the arm pin with its clip". The conrod is still described as "e-clips", and Aim 3 says "Join the conrod". No printed clip or bought_ref exists for the idler clip, the two conrod clips or the 20x1.5 O-ring. The Build Guide still lists the O-ring and e-clip as "Bought" (GrappStar_Build_Guide.md:362).

- File: `base/cad/parts/launch.py:674`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: Launcher 5 says 'Snap the C clip on the groove' of the big gear pin. Aim 2 says 'Put the O-ring in the swing arm hub groove ... fit the idler on the arm pin with its e-clip'. Aim 3 says 'Join the conrod ... with C clips'. That is 4 clips and a 20x1.5 O-ring. No launch clip is printed (reel has printed reel_c_clip etc., launch has none), and none is a bought part. BOM.md and the pack therefore hold none of them, and neither do Harry's inventory (SYSTEM_PLAN 5: the only purchase is 608s) or the guide's parts table. Harry cannot retain the big gear pin, idler or conrod. Without the O-ring the swing arm has no drag, so the aim splitter does not swing into mesh.

Evidence: launch.py:440 notes="Snap pin d10 for the big gear, C clip"; launch.py:674 "Snap the C clip on the groove"; launch.py:707-708 "Put the O-ring in the swing arm hub groove ... fit the idler on the arm pin with its e-clip"; launch.py:715 "with C clips"; launch_aim.py:350 "O-ring 20x1.5 in the hub drags it; printed idler pin, e-clip groove"; launch_aim.py:364 "Conrod 156 between pin centres, e-clips"; BOM bought list has no clip or O-ring line.

Fix: Add printed PPA-CF clip parts sized to each groove (launch_gear_pin_clip, launch_aim_idler_clip, launch_aim_conrod_clip_1/2), as reel.py does for reel_c_clip, and put them in the steps. Add a bought_ref "O-ring 20x1.5 NBR" on launch_aim_arm, with a printed alternative (a TPU-free PLA friction spring) since Harry has no O-ring. Otherwise list the O-ring under 'Possible extra purchases' in docs/build/src/03_parts.md.

> Skeptic (confirmed): I checked the claim against the code and it holds. launch.py:440 (launch_gear_pin, notes "C clip"), :674 (Launcher 5, "Snap the C clip on the groove"), :707-708 (Aim 2, "O-ring in the swing arm hub groove ... idler ... with its e-clip") and :715 (Aim 3, "conrod ... with C clips") say what the claim quotes. The steps' parts lists name only launch_big_gear/launch_gear_pin, launch_aim_sun/arm/idler and launch_aim_lug/conrod. No clip or O-ring Part exists.

I grepped base/cad/parts/launch*.py. No clips.c_clip part is made there and no bought_ref mentions a clip or O-ring. The only bought refs in l
>
> Skeptic (confirmed): I couldn't refute this. The code backs up every part of the claim.

1. **No clip parts in the launcher.** `gear_pin_solid` (`launch.py:339-343`) cuts a clip groove into the big gear pin. `launch_aim.py:147` cuts one into the idler pin on the swing arm. But I listed every `Part`/`add` name in `launch*.py` and none of them is a clip. `reel.py`, by contrast, defines `reel_c_clip`, `reel_cup_clip`, `reel_k_clip` and `reel_s_clip` with `clips.c_clip()` and adds them to its steps and Allows. The launcher does neither.

2. **The steps ask for parts that don't exist.** Launcher 5 says "Snap the C clip
>

## 15. Aim 3 cannot be done in the generated order: the launch sleeve has to slide axially over the fixed main-shaft spigot and into the bolted aim-housing bushing

- Now (863d58d): **partly fixed** at `base/cad/parts/launch.py:1229-1236 (vs base/cad/parts/drum.py:1199-1202)`. Fixed: Aim 3 now slides the sleeve along the axis onto the d12 spigot from the outboard side. The aim housing goes on after the sleeve (Aim 1 now says "goes onto the base later"), and the magnet cap is fitted last, so the impossible sideways lowering is gone. Still open: Aim 3 still treats the LAUNCH dog ring as already on the drum ("until its flange meets the drum's LAUNCH dog ring") and says to fit the 3 x M3 "from the drum side into the flange nut traps". The drum step says the ring is fitted to the sleeve flange first, before the sleeve goes on the spigot, with the screws "from the sleeve side into the ring". So the two steps still contradict each other, and the drum-side screw direction (blocked by the 2.5 mm gap to the collar) has not been removed.

- File: `base/cad/parts/launch.py:712`
- Severity: medium (claimed high); 2 of 2 skeptics confirmed
- Owner: cad-launch

Scenario: By Aim 3 the main shaft is fixed (drum step 9: hex in the coupler, M3 x 45 cross screw, collar in the fork) and the aim housing is screwed to the deck (Aim 1). The d12 spigot runs the whole length of the sleeve bore (y 96..200) and on through the aim housing to the magnet cap. 'Lower the rail assembly so the launch sleeve's outboard end enters the aim housing bushing' would need the sleeve to come down sideways over a shaft that passes through it, which is impossible. The builder has to take the main shaft or the aim housing back off. The same step also contradicts the drum step on the LAUNCH dog ring: drum says fit the ring to the sleeve beforehand with screws 'from the sleeve side into the ring'; launch says fit it at this point with screws 'from the drum side, nuts in the sleeve flange traps'. Once installed there is a 2.5 mm gap between the collar (y 98) and the ring (y 100.5), so no screw can go in from the drum side.

Evidence: launch.py:347 `SPIGOT_BORE = 12.35  # cad-drum d12 spigot runs in this bore the whole sleeve length`. params.py:109 `dict(y=(96.0, 200.0), part="main shaft d12 spigot through the launch sleeve bore (12.35)"`. params.py:107 aim-housing bushing at y 187..193. launch.py:713-715 'Lower the rail assembly so the launch sleeve's outboard end enters the aim housing bushing and the flange meets the drum's LAUNCH dog ring (3 x M3 from the drum side, nuts in the sleeve flange traps)'. drum.py:1174-1175 'Fit the LAUNCH dog ring on the launch sleeve flange (... 3 x M3 x 10 from the sleeve side into the ring) before the launch sleeve goes on the spigot'.

Fix: Re-sequence. On the bench: dog ring onto the sleeve flange (3 x M3 from the ring side into the flange nut traps). Then rail, sleeve, drive gear and splitter as in Launcher 2-6 and Aim 2. Then thread the main shaft spigot (REEL sleeve, REEL gear and collar already on it) through the sleeve bore, so main shaft and rail become one unit. Lower that unit with the collar groove into the fork pads, push the hex into the coupler and fit the cross screw. Only then slide and screw the aim housing on over the sleeve end (Aim 1 moves after this), and fit the magnet cap last. Remove the drum-side screw direction from Aim 3.

> Skeptic (confirmed): The claim holds up against the code.

1. Order. base/tools/assembly.py:39 sorts the subsystem modules alphabetically (demo, drum, frame, launch, reel), and their steps are appended in that order. So all drum steps come before all launch steps.

2. Drum "Main shaft: stack" (drum.py, around line 1168) fixes the main shaft in place: the hex goes into the coupler, the M3 x 45 cross screw and nyloc go in, the collar sits in the fork pads, and the REEL sleeve is screwed to the coupler. params.py stack has the d12 spigot running from y 96 to y 200.

3. Launch "Aim 1" (launch.py:697) screws the aim ho
>
> Skeptic (confirmed): I couldn't refute this. The claim holds up against the code.

Build order: base/tools/assembly.py:39 sorts the subsystem modules alphabetically (demo, drum, frame, launch, reel), and Assembly.__init__ collects steps per module in that order. tools/steps.py plan_steps keeps that order. So every drum step comes before any launch step. That includes "Main shaft: stack" (drum.py:1166-1172): the hex is pushed into the coupler, the M3 x 45 cross screw and nyloc go in, and the sleeve flange is screwed to the coupler. The shaft is therefore fixed at its -y end before the launcher steps start.

Launch 
>

## 16. MEASURE_LIST A6 records wiper free speed from a 40 % duty take-up jog as WIPER.n0_rpm

- Now (863d58d): **fixed** at `docs/build/MEASURE_LIST.md:20`. A6 now says the forward jog runs at 102/255 duty and tells the builder to record n0_rpm = measured rpm x 2.5, never the raw jog rpm. The Build Guide copy (GrappStar_Build_Guide.md:3513) has the same wording. The A7 calib.py --rpm line the finding referred to is gone, and A7 is now an optional stall-torque check.

- File: `docs/build/MEASURE_LIST.md:20`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: docs

Scenario: A6 runs on the bench (drum code NEUTRAL from the GPIO11 jumper) with {"cmd":"jog","motor":"wiper","dir":1}. Forward in NEUTRAL is take-up only, so the firmware caps the duty at takeupMaxDuty=102/255 (40 %). The shaft runs at about 40 % of its 12 V free speed (about 18 rpm, not about 45). A6 tells Harry to write that rpm into WIPER.n0_rpm and pass it to calib.py --rpm. params.py then carries about 0.4 x the real n0, which goes through gen_params into firmware n0Rpm (index-mode rate, centering-jog cap) and into physics/ballistics (cock time, reel speed). A7 corrects its reading for the reduced duty, but A6 does not.

Evidence: MEASURE_LIST.md:20 'Send {"cmd":"jog","motor":"wiper","dir":1,"ms":1500} ... Read rpm ... | `WIPER.n0_rpm`, `WIPER.free_a`'; base_logic.h:836-837 'bool takeup = sec >= SEC_NEUTRAL && sec <= SEC_WRAP && dd == c.reelDir; ... w.want(dd, takeup && c.jogDuty > c.takeupMaxDuty ? c.takeupMaxDuty : c.jogDuty)'; config.h:153 'int takeupMaxDuty = 102;   // 40 percent'; BENCH_TEST.md:120-121 'a forward wiper jog only counts as take-up, so it runs at 40 % power'.

Fix: In A6, either use dir -1 (jogDuty 150/255) and record n0_rpm = measured rpm x 255/150, or keep dir 1 and record measured rpm x 255/102. Say plainly that free speed scales with duty and that the raw jog rpm must not go into n0_rpm. Make the A7 calib.py line use the corrected rpm.

> Skeptic (confirmed): I could not refute this. The defect is real.

1. The A6 command lands at 40 % duty. MEASURE_LIST.md:20 has Harry send {"cmd":"jog","motor":"wiper","dir":1}. On the bench, BENCH_TEST.md step 0/4 tells him to keep a jumper from GPIO11 to GND, so the drum code reads NEUTRAL. In base_logic.h:836-837, `takeup` is true when sec is NEUTRAL..WRAP and dd == c.reelDir. reelDir is +1 (config.h:73). The jog therefore runs at min(jogDuty 150, takeupMaxDuty 102) = 102/255. BENCH_TEST.md:120 says the same: "runs at 40 % power". The only other route is a backward jog, which runs at jogDuty 150/255, so neither
>
> Skeptic (confirmed): I checked the code and the claim holds. MEASURE_LIST.md:20 (A6) sends {"cmd":"jog","motor":"wiper","dir":1} and tells Harry to record the raw telemetry rpm as WIPER.n0_rpm. It says nothing about duty or scaling. On the bench, BENCH_TEST.md:23-24 has him jumper GPIO11 to GND so the drum code reads NEUTRAL. In cmdJog (firmware/base/grappstar_base/base_logic.h:836-837), a dir=+1 jog in SEC_NEUTRAL..SEC_WRAP equals reelDir (+1, config.h:73), so takeup is true and the duty is capped at takeupMaxDuty=102/255 (config.h:153). The deadman refresh path (line ~821) only pushes jogUntil forward and keeps 
>

## 17. Payout brake / clutch trim refers to shims in the 'reel assembly steps' that neither the steps nor the part list contain

- Now (863d58d): **partly fixed** at `docs/build/MEASURE_LIST.md:104`. The design has changed: LIGHT preload is now set by two M3 trim screws in the brake rocker (reel.py:749 says "1/8 turn = 0.4 N", and the 'Brake rocker and push rod' step gives a starting point of tips 0.5 mm proud). But G11 at MEASURE_LIST.md:104 (and GrappStar_Build_Guide.md:3597) still says "add or remove printed 0.4 mm shims under thrust ring T" and lists "reel shims (print more)". No shim part exists, and no step tells the builder to turn the trim screws when G4 fails.

- File: `docs/build/MEASURE_LIST.md:103`
- Severity: medium (claimed medium); 2 of 2 skeptics confirmed
- Owner: cad-reel

Scenario: G4 fails, for example 40 g pulls line off (brake under 0.5 N, so loops at launch) or 50 g does not (over 1 N, so lost range). G11 tells Harry to 'add or remove printed 0.4 mm shims under thrust ring T (reel assembly steps)'. The reel steps never mention shims or a preload setting, only 'it should turn with a light, even drag', and there is no shim part to print. The builder has no way to set the 0.5 N LIGHT and 5 N FIRM values.

Evidence: MEASURE_LIST.md:103 'G11 Payout brake trim | If G4 is off: add or remove printed 0.4 mm shims under thrust ring T (reel assembly steps)'. reel.py:938-940 'Reel shaft into the wall' step: 'Spin the reel by hand: it should turn with a light, even drag.' No shim anywhere in reel.py (grep 'shim' finds only reel_NOTES.md:98). params.py:213 `clutch=dict(LIGHT_line_n=0.5, FIRM_line_n=5.0, ...)`.

Fix: Add a reel_shim part (0.4 mm ring for the reel shaft, print 4) and put a number in the 'Storage reel' step: 'start with 1 shim behind thrust ring T; in LAUNCH 40 g must not pull line off and 50 g must; add a shim to raise the drag, remove one to lower it'. Point G11 to that step by name.

> Skeptic (confirmed): I checked this and the defect is real. I edited nothing.

- **G11 text.** docs/build/MEASURE_LIST.md:103 and docs/build/GrappStar_Build_Guide.md:2915 both say: "add or remove printed 0.4 mm shims under thrust ring T (reel assembly steps)". The Changes column says "reel shims (print more)".
- **No shim anywhere else.** I searched every tracked text file for "shim". It appears only in reel_NOTES.md:98 ("0.4 mm printed shims under T trim it"), the old C_compact.md panel notes (which talk about a wave spring shim from an earlier design), generated html/js, and the G11 line itself. reel.py has no s
>
> Skeptic (confirmed): The claim holds. MEASURE_LIST.md:103 and GrappStar_Build_Guide.md:2915 both say "add or remove printed 0.4 mm shims under thrust ring T (reel assembly steps)" and give the reprint as "reel shims (print more)". But no shim part exists. Searching for "shim" in base/cad/parts/reel.py and the build docs finds nothing apart from these G11 rows and one design note, reel_NOTES.md:98 ("0.4 mm printed shims under T trim it"). That note records the designer's intent; it was never turned into a part or a step. Nothing in reel.py's add() part list is a shim. The "Storage reel" step (reel.py ~921-928) only
>

## 18. Test stage 4 'cock and fire with no springs' cannot pass: with no springs the carriage never returns, so the firmware faults every cycle

- Now (863d58d): **fixed** at `docs/build/TEST_PLAN.md:91`. Stage 4 item 3 now notes that with no springs the firmware would fault on every cycle. It has the builder hook rubber bands (or inner-tube loops) from the spring anchor pins to the carriage hooks as stand-in springs, which pull the carriage home so SW_HOME closes. It also adds cock/decock cycles before the 10 fire cycles and says to take the bands off afterwards.

- File: `docs/build/TEST_PLAN.md:83`
- Severity: medium (claimed high); 2 of 2 skeptics confirmed
- Owner: docs

Scenario: Stage 4, springs off, require_seated false: `arm`, then `fire`. The big gear draws the carriage back and the cut tooth lets go. With no springs nothing pulls the carriage forward, and at a positive pitch gravity slides it further back. SW_HOME never closes, so the firmware raises F_BAD_STATE 'no release seen (SW_HOME did not close)' and stops with the carriage drawn back and the big gear out of time with the rack. The stage asks for 10 such cycles and they cannot complete. If the builder keeps retrying, the big gear's first tooth meets the rack at the wrong place.

Evidence: TEST_PLAN.md:83-84: '3. Cock and fire with no springs: `arm`, then `fire` with the token. The carriage travels back, releases, and the big gear re-engages: 10 cycles.' base_logic.h:1233 `if (rose(SW_HOME)) { // the relieved tooth let the rack go: carriage home = shot`. base_logic.h:1244: `raise(F_BAD_STATE, "no release seen (SW_HOME did not close)")`. base_logic.h:1250 also faults if 'reset ended with the carriage off its stop'.

Fix: Replace stage 4 item 3 with a cycle that works without springs: `arm` (partial cock), then `cal decock`, 10 cycles, checking that SW_HOME and SW_GAP close at the end. Alternatively, after each `fire` expect the 'no release seen' fault: battery and lead #2 out, slide the carriage home by hand, `clear`. Move the full fire cycle to stage 5.

> Skeptic (confirmed): I checked the claim against the code, and it holds. In firmware/base/grappstar_base/base_logic.h, tickFire with phase PH_FIRING and homeSeen set only moves on to PH_RESETTING when rose(SW_HOME) happens, meaning the carriage is back on its front stop. Otherwise, once cockPhase passes releaseDeg + releaseWinDeg, it raises F_BAD_STATE "no release seen (SW_HOME did not close)". The firmware has no springs-off or dry-fire mode: no flag lets it skip that check, and the only stage 4 relaxation is require_seated=false. BASE_SPEC.md:18-19 says the springs are what return the carriage: the big gear draw
>
> Skeptic (confirmed): I could not refute this. I checked each step against the code and spec.

1. TEST_PLAN.md:73-84 asks for the cycle in Stage 4, which runs with "Springs off". Item 3 expects "The carriage travels back, releases, and the big gear re-engages: 10 cycles". The pass line says "all cycles complete with no fault".

2. BASE_SPEC.md:18-20 says the big gear "draws the carriage rack back against the two springs, releases at the relieved last tooth (fire), and re-engages (reset)". The two springs are the only thing that sends the carriage forward. Nothing in base/spec or docs describes any other return: no 
>

## 19. BOM lists 12 phantom bought items (spring segments and carriage-end hooks with no bought_ref)

- Now (863d58d): **still open** at `base/cad/parts/launch.py:784-791`. On HEAD, launch_spring_{l,r}_hook_c and the 5 coil segments per side (N_SEG=5) are still "bought" parts with no bought_ref. base/tools/bom.py:109-112 still adds a "<name> (bought, no bought_ref)" line for each one, so the BOM still shows 12 phantom 'Other' items.

- File: `base/cad/parts/launch.py:460`
- Severity: low (claimed low); unverified (low)
- Owner: cad-launch

Scenario: The spring coil segments and hook_c pieces are bought parts with no bought_ref. bom.build therefore adds 12 'Other' lines such as 'launch_spring_l_0 (bought, no bought_ref)', each with count 1, next to the real '2 x Century ... extension spring'. Harry checking the BOM thinks 12 more items are needed and cannot find what they are.

Evidence: launch.py:460-467 Part(f"launch_spring_{side}_hook_c", ..., "bought", ... notes="Spring hook on the carriage peg") and Part(f"launch_spring_{side}_{i}", ..., "bought", ...) with no bought_ref; BOM.md: "Parts that are not printed and have no bought_ref: launch_spring_l_hook_c, launch_spring_l_0, ...".

Fix: Give those parts bought_ref="(part of the Century spring)" as frame.py:764 does for the wiper shaft, or flag them as visual-only and skip them in bom.build.


## 20. Launcher 1 print list gives conrod and lug as PLA, but the model and pack print them in PPA-CF

- Now (863d58d): **still open** at `base/cad/parts/launch.py:1136`. The Launcher 1 text no longer has an explicit "PLA: ... conrod, lug" list. It now says "All PLA unless named" and then lists the PPA-CF parts, and that list leaves out the conrod and the lug. So the step still sends both to PLA. launch_aim.py:472-474 makes launch_aim_conrod and launch_aim_lug in PPA-CF.

- File: `base/cad/parts/launch.py:648`
- Severity: low (claimed low); unverified (low)
- Owner: cad-launch

Scenario: Launcher 1 text tells Harry 'PLA: ... conrod, lug, aim housing and cap'. The pack and BOM give launch_aim_conrod and launch_aim_lug as PPA-CF (they carry the rail lift load on d8 pins). If Harry follows the step text, as he would when reprinting a single part, he prints the loaded conrod and lug in PLA.

Evidence: launch.py:648-649 "PLA: rail rear, mid and front pieces ..., the three switch bodies (flat), conrod, lug, aim housing and cap"; BOM.md printed table: launch_aim_conrod PPA-CF, launch_aim_lug PPA-CF (from launch_aim.py:364 and :367).

Fix: Move 'conrod, lug' from the PLA list to the PPA-CF list in the Launcher 1 text. Better: generate that list from each part's material so the step and the pack cannot disagree.


## 21. Brake rocker is PETG, which Harry does not have; the clutch spring thickness is computed for PETG stiffness, so a PLA or PPA-CF print overloads the payout brake

- Now (863d58d): **still open** at `base/cad/parts/reel.py:742`. The rocker is still added as "PETG" (reel.py:742), and reel_calc.rocker still defaults to E=2100.0 (reel_calc.py:154), with the report line labelled "(PETG, 10 wide)" at reel_calc.py:287. The spec has no PETG source or fallback, so a PLA or PPA-CF print still over-stiffens the LAUNCH brake.

- File: `base/cad/parts/reel.py:569`
- Severity: low (claimed medium); 1 of 2 skeptics confirmed
- Owner: cad-reel

Scenario: The pack puts reel_rocker on a PETG plate. Harry has PLA and PPA-CF only, the guide never mentions PETG (section 4: 'PLA for everything except loaded gears and keyed shafts, which are PPA-CF'), and there is no fallback. reel_calc.rocker sizes the flexure thickness with E=2100 MPa. Printed in PLA (E about 3500) the same pre-deflection gives about 1.7 x the force, so the LAUNCH brake goes from 0.5 N to about 0.8 N. In PPA-CF (E about 6000+) it rises to about 1.5 N or more, above the 1 N the flight sim says starts costing range, toward the 3 N that kills it.

Evidence: reel.py:569-570 add("reel_rocker", _rocker(), "PETG", ... notes="Brake rocker: flexible lever = clutch spring"); reel_calc.py:154 `def rocker(d, E=2100.0, b=10.0):`; reel_calc.py:279 "rocker plate thickness (PETG, 10 wide)"; GrappStar_Build_Guide.md:266 PLA/PPA-CF only; SYSTEM_PLAN_2026-11-03.md:44-45 brake 0.5 to 1 N, above 1 N costs range, 3 N kills it.

Fix: Make reel_rocker PLA, and size its thickness with reel_calc.rocker(d, E=3500) (PLA) so the LAUNCH preload stays 0.5 N. Update the reel_calc report line and the BOM material. Add a bench check of the launch-mode drag (spring scale on the line, under 1 N) to the reel step.

> Skeptic (confirmed): The core of the claim holds, but several of its details are wrong or overstated.

Confirmed in the code:
- reel.py:569 makes reel_rocker in PETG.
- The geometry (ROCKER_Z = (0.5, 3.5), so a 3.0 mm plate; w = 5, so 10 mm wide) matches reel_calc.rocker(), which uses E = 2100. Running `python3 -m cad.parts.reel_calc` prints "rocker plate thickness (PETG, 10 wide) 3.0 mm", a peak stress of 18.0 MPa at REEL, and 0.8 mm of input pre-deflection at LAUNCH.
- WORKPLAN line 57 says "only parts Harry has ... PLA; PPA-CF for loaded gears and keyed shafts". HARRY_MESSAGES line 66 says other filaments are d
>
> Skeptic (refuted): The parts of the claim that carry its weight are wrong. (1) The guide does mention PETG and does give a fallback. GrappStar_Build_Guide.md:2870, in the measure list and repeated at docs/build/MEASURE_LIST.md:58, says "C19 PETG filament (brake rocker) | Look in the parts box. Needed for: reel | If not: PLA works for the first tests". So "never mentions PETG" and "there is no fallback" are false. Section 4's general rule does not override a specific listed item. (2) The proposed bench check is already in the guide. Line 2615 says "Set the payout brake with hanging weights (G4): 40 g does not pul
>

## 22. Contact gap numbers disagree: G12 says 1.1 mm for every contact, the steps say 0.25, 1.5 and 1.8 mm

- Now (863d58d): **fixed** at `docs/build/MEASURE_LIST.md:105`. G12 now points to the gap each assembly step gives: SW_HOME/SW_SEATED 0.25 mm, SW_INDEX about 1.5 mm, drum code about 1.8 mm, then a check in telemetry `sw`. The generic 1.1 mm and 3 mm bump figures are no longer in the build docs, and the compiled guide copy at GrappStar_Build_Guide.md:3598 has the same new text.

- File: `docs/build/MEASURE_LIST.md:104`
- Severity: low (claimed low); unverified (low)
- Owner: docs

Scenario: The guide rule 'Contacts: set each gap by turning the screw (MEASURE_LIST G12)' sends the builder to G12, which says 1.1 mm and 'its 3 mm bump'. If he sets SW_HOME to 1.1 mm (its step says half a turn back, 0.25 mm), the switch may not close at home. The firmware then reads the carriage as drawn (maybeLoaded) and refuses drum moves. The drum code switches, set to 1.1 mm instead of 1.8 mm, can stay closed on the low part of the band and give a wrong Gray code.

Evidence: MEASURE_LIST.md:104 'the M3 screw head sits about 1.1 mm from the nut and must close halfway up its 3 mm bump'. drum.py:1126 index contact 'about 1.5 mm on a low part' (idx_bump 2.5, drum.py:31). drum.py:1163 code switches 'gap about 1.8 mm with the nub low'. launch.py:662-663 SW_HOME 'until the head just touches the nut, then back it off half a turn (0.25 mm gap)'. Guide line 357 '- Contacts: set each gap by turning the screw (MEASURE_LIST G12).'

Fix: Rewrite G12 to: 'Set each contact to the gap its assembly step gives (SW_HOME/SW_SEATED 0.25 mm; SW_INDEX 1.5 mm; drum code 1.8 mm), then check in telemetry `sw` that it opens and closes at the right place'. Drop the generic 1.1 mm and 3 mm figures.


## 23. SYSTEM_ELECTRICAL points to a 'flash from the Jetson' procedure that does not exist

- Now (863d58d): **fixed** at `docs/build/SYSTEM_ELECTRICAL.md:125`. The rule now says "Flash from the Mac; battery out is enough, the wiring can stay on the board (firmware/base/README.md)". The pointer to a "flash from the Jetson" procedure that did not exist is gone. The README's Mac IDE and arduino-cli steps match this rule.

- File: `docs/build/SYSTEM_ELECTRICAL.md:126`
- Severity: low (claimed low); unverified (low)
- Owner: docs

Scenario: With the base installed on the rover, Harry follows the rule and looks in firmware/base/README.md for how to flash from the Jetson. The README only covers Arduino IDE on the Mac (sections 'Arduino IDE setup (Mac)', steps 1-9). There is no arduino-cli or esptool procedure in firmware/base/README.md or jetson/README.md, so he has to pull the ESP32 from the harness to re-flash after a params change (MEASURE_LIST.md:132 says 're-flash the base ESP32').

Evidence: SYSTEM_ELECTRICAL.md:125-126 'Flash from the Mac with the ESP32 out of the rover harness, or flash from the Jetson (firmware/base/README.md).'; grep for arduino-cli/esptool in firmware/base/README.md and jetson/README.md finds nothing.

Fix: Either delete 'or flash from the Jetson (firmware/base/README.md)', or add a 'Flash from the Jetson' section to firmware/base/README.md. That section needs: arduino-cli install of esp32:esp32@3.3.12 and 'HX711' by Rob Tillaart, `arduino-cli compile --upload -p /dev/grappstar-base --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc,USBMode=hwcdc,FlashSize=8M firmware/base/grappstar_base`, and `sudo systemctl stop grappstar` first.


## 24. Tool list misses the drill and 3.4 mm bit, the heat-set insert tip and the 13 mm socket that the steps require

- Now (863d58d): **fixed** at `docs/build/src/03_parts.md:57`. I checked the Tools list on the newest tree. It now includes a drill with 3 mm and 3.4 mm bits and a deburring tool for the rover deck holes, an M3 heat-set insert tip for the soldering iron, and a 13 mm socket with a 1/4 inch driver for the coupler M8 nut.

- File: `docs/build/src/03_parts.md:57`
- Severity: low (claimed low); unverified (low)
- Owner: docs

Scenario: Step 'Drill the rover' needs a drill to make 49 x 3.4 mm holes in the rover top plate. 'Heat-set inserts' calls for 'an insert tip'. 'Main shaft: coupler and limiter' tightens the M8 nut 'with a 13 mm socket (1/4 inch drive)'. None of these is in the guide's Tools list, so Harry finds out mid-build that he cannot mount the deck or set the coupler nut.

Evidence: 03_parts.md:57 (guide line 260) "Soldering iron and solder, flux, multimeter, wire strippers, flush cutters, small flat screwdriver ... hex keys ..." (no drill, bit, insert tip or socket); frame.py:852 "drill 3.4 mm. Deburr."; frame.py:846 "with an insert tip"; drum step text "tighten it with a 13 mm socket (1/4 inch drive)".

Fix: Add to the Tools list: "drill with 3 mm and 3.4 mm bits and a deburring tool (rover holes), M3 heat-set insert tip for the soldering iron, 13 mm socket and 1/4 in driver (coupler M8 nut)".


## 25. Guide never picks up the risk coupon test sheet: its include glob names a folder and file that do not exist, so the day-one coupon section stays 'NOT IN THE REPO YET'

- Now (863d58d): **fixed** at `docs/build/src/04_printing.md:54`. On the newest tree the guide uses `@includeglob base/pack/coupons/COUPON_TEST.md first` and `@imageglob base/pack/coupons/*.png` (lines 52-54), and make_guide.py:51 lists base/pack/coupons/COUPON_TEST.md. Both files exist, so the risk-coupon test sheet is now included in the guide instead of the "not in the repo" placeholder.

- File: `docs/build/src/04_printing.md:22`
- Severity: low (claimed low); unverified (low)
- Owner: docs

Scenario: The coupon pack and its test sheet are in the repo at base/pack/coupons/COUPON_TEST.md (plus rc1..rc9 STLs). make_guide includes `base/**/coupon/COUPON.md` (singular folder, different file name), which matches nothing. The built guide (line 286) therefore tells Harry the coupons are not in the repo, at the exact step FIRST_DAY says must pass before any big base part, and the coupon checks are missing from the PDF he prints.

Evidence: 04_printing.md:22 `@includeglob base/**/coupon/COUPON.md first`; make_guide.py:46 ("base/**/coupon/COUPON.md", ...); GrappStar_Build_Guide.md:286 "NOT IN THE REPO YET: `base/**/coupon/COUPON.md`"; actual file base/pack/coupons/COUPON_TEST.md.

Fix: Change the glob in 04_printing.md:20 and :22 and in make_guide.py:46 to base/**/coupons/COUPON_TEST.md (and coupons/*.png), then re-run make_guide.py.


## 26. Guide's risk-coupon section stays 'NOT IN THE REPO YET' even after rebuilding, because make_guide looks for the wrong file names

- Now (863d58d): **fixed** at `docs/build/src/04_printing.md:52`. The guide source now uses `@imageglob base/pack/coupons/*.png` and `@includeglob base/pack/coupons/COUPON_TEST.md` (lines 52 and 54). make_guide.py SOURCES at line 51 lists base/pack/coupons/COUPON_TEST.md. The rebuilt GrappStar_Build_Guide.md has the risk plate images at lines 330-332 and pulls in COUPON_TEST.md at line 336, with no placeholder in that section.

- File: `docs/build/src/04_printing.md:22`
- Severity: low (claimed low); unverified (low)
- Owner: docs

Scenario: FIRST_DAY step 4.2 and TEST_PLAN stage 0 send Harry to the guide's section 4 for the coupon plate and its test sheet. The coupon pack exists (base/pack/coupons/risk_plate_1.3mf, risk_plate_2.3mf, COUPON_TEST.md, risk_plate_*.png), but rebuilding the guide still prints the placeholder because the glob names a 'coupon' folder and a 'COUPON.md' file. Gate 0 has no instructions in the guide.

Evidence: src/04_printing.md:20 `@imageglob base/**/coupon/coupon_plate_*.png Risk coupon plate` and :22 `@includeglob base/**/coupon/COUPON.md first`. make_guide.py:46 `("base/**/coupon/COUPON.md", "cad-infra (risk coupons)")`. The actual files are base/pack/coupons/COUPON_TEST.md and base/pack/coupons/risk_plate_1.png / risk_plate_2.png. Guide line 284 '> NOT IN THE REPO YET: `base/**/coupon/COUPON.md`'.

Fix: Change the globs to `base/pack/coupons/risk_plate_*.png` and `base/pack/coupons/COUPON_TEST.md` (in src/04_printing.md and make_guide.py SOURCES), then re-run make_guide.py.


## 27. Guide's motor-adapter screw rule (6 x M3 x 12) disagrees with the frame step (M3 x 10, 10 of 12 holes)

- Now (863d58d): **fixed** at `docs/build/src/05_base_assembly.md:19-21`. The guide rule now says "fit M3 x 10 through the cheek plate into every insert in the adapter hub (10 of the 12 holes line up; the frame step 'Motor onto the cheek' lists them)". That matches the frame.py step at base/cad/parts/frame.py:1012, and the old "6 x M3 x 12" text is gone.

- File: `docs/build/src/05_base_assembly.md:19`
- Severity: low (claimed low); unverified (low)
- Owner: docs

Scenario: The frame step that carries the correct numbers ('Motor onto the cheek') is dropped from the generated steps, so the guide's general rule is all the builder sees. He fits 6 screws instead of 10, and M3 x 12 instead of the M3 x 10 the frame was designed around, into 5 mm heat-set inserts. The longer screws can bottom out or push the inserts out of the adapter hub, and the 4 missing screws weaken the joint that carries about 15 N m of wiper torque.

Evidence: src/05_base_assembly.md:18-19 (guide line 351-352): 'Motor adapter: bolt the wiper motor to the adapter on the bench first (M6 bolts from the cheek side), then fit the 6 x M3 x 12 from inside the cheek box'. frame.py frame step 'Motor onto the cheek' renders as: 'From the frame side of the plate, M3 x 10 through the plate into every insert in the adapter hub (10 of the 12 holes line up with inserts; the others stay empty).'

Fix: Change the guide rule to 'then fit M3 x 10 through the cheek plate into every insert in the adapter hub (10 of the 12 holes; the frame assembly step lists them), before any shaft goes in', or have it quote the frame step text.


## 28. Drum manual release is described three different ways (thumb screw, swing screw, hand latch), and no latch exists

- Now (863d58d): **partly fixed** at `docs/build/GrappStar_Build_Guide.md:2662`. Both sources now use the swing-screw wording: firmware/base/CALIBRATION.md:111-113 and docs/build/src/05_base_assembly.md:27-28. The generated guide was not rebuilt after the CALIBRATION.md change, so docs/build/GrappStar_Build_Guide.md:2662 (and presumably the PDF built from it) still says "flip the hand latch ... latch the N20 back in", even though line 700 of the same file uses the swing-screw wording. Rerunning make_guide.py should fix it.

- File: `firmware/base/CALIBRATION.md:101`
- Severity: low (claimed low); unverified (low)
- Owner: docs

Scenario: At CALIBRATION (c) step 3 Harry is told to 'flip the hand latch' to take the N20 out of mesh. The guide says loosen a 'thumb screw and slide the N20 out'. The drum step says loosen the M3 swing screw and swing the cradle out. The CAD has a pivot screw and a slotted swing screw, with no latch and no thumb screw. A learner looks for a part that does not exist, or forces the drum by the lever with the N20 still meshed, which back-drives the worm and can strip the 0.8-module pinion.

Evidence: CALIBRATION.md:101 'Battery still unplugged: flip the hand latch (`n` disappears from `sw`)'. src/05_base_assembly.md:25 'Manual drum fallback: loosen the N20 cradle thumb screw and slide the N20 out of mesh'. drum.py:1155-1156 'pivot screw M3 x 12 in the round hole, swing screw M3 x 12 in the slot' and drum.py:1177-1178 'loosen the swing screw, swing the N20 out, turn the lever, swing back and tighten'.

Fix: Use the drum step's wording everywhere: 'loosen the cradle swing screw (M3, in the slot), swing the N20 out of mesh until `n` disappears from `sw`, turn the hand lever, swing back and tighten'. Change CALIBRATION.md:101 and guide src/05_base_assembly.md:25 to match.

