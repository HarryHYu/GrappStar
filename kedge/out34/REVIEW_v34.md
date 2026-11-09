# KEDGE dart v34: independent review before printing (9 Nov 2026)

Scope: kedge_v34.py, out34 (parts.json, audit.json, launcher_fit.json, mass_props.json, review/, stage shots, sections),
pack_v34 (run sheet PDF, wire list), sim_righting/v34_check, firmware/dart v4.2. Nothing in the model was changed.
Rules checked: simple and working, a little motor load is fine, every cavity reachable, sealed body, only parts in hand.

Verdict: the motion geometry is clean (my own insertion sweeps agree with the audit), but six things stop the build,
the launch or the anchor as drawn. Each one has a fix that uses parts Harry already has.

---------------------------------------------------------------------------------------------------

## 1. Blockers

| # | What | Evidence | Fix or workaround (parts in hand) |
|---|---|---|---|
| B1 | **The putty pockets are sealed voids.** The run sheet says "press 14.6 g of putty flush into the two pockets". Both pockets are closed cavities inside the belly rails (0 to 0.8 mm skin on the inboard face), so the putty cannot go in. | hull_bottom.npz splits into 3 shells: the body and two 809.3 mm3 voids (2 x 809 = 1.62 cm3 = ballast_belly). The audit's single-body check counts B-rep solids, so it misses inner shells. Sections at x 90 and 113. | Model: let the pocket break through the rail's inner face (inner radius 17.2 to 16.3). On the print: cut the inner skin open with a knife from x 62 to 116 on each rail, then press the putty in. Without putty the folded CG moves to z +0.59 (above the axis). |
| B2 | **The 3-pin charge header does not fit its seat.** The right flank seat is the 2-pin seat mirrored: 7.9 mm opening, 5.2 mm pin pocket. A B3B-XH-A is 9.9 mm wide and its pin row needs about 6.5 mm with solder. The generator writes `wide 9.9` and then clips the header to 7.4 (`min(hz, 3.7)`), so the model fits but the part will not. | kedge_v34.py lines 1014-1018 (header clip) and 710-713 (seat cuts). The v31 fit coupon has a 10.2 mm 3-pin slot, so the coupon would pass while v34 fails. | Model: right seat opening 10.2 and pin pocket 7.0 (z), as the v31 slot. By hand: cut the two short end walls off the 3-pin shroud with flush cutters; that leaves 7.4 mm and the plug still mates on the long walls. Or skip the in-dart charge port and charge the cells in the XTAR MC2. |
| B3 | **The 8 radial screws barely hold, and the rear four do not hold at all.** The rear disc is 2.0 mm thick and the Ø2.5 pilot is wider than the disc, so at the four rear screws there is no disc material to cut a thread in. Disc 1 is 3.0 mm thick: an M3 thread leaves 0.25 mm walls fore and aft. The two lower disc-1 screws sit half in the corridor wire windows. | My section at z 3 through x 122.0 shows no disc round the rear pilots. Per-screw overlap of the modelled Ø2.9 shank with the tray (a full 7 mm bite in solid material is about 11.8 mm3): disc 1 upper 8.8, disc 1 lower 4.0, rear upper 5.1, rear lower 2.2 (slivers above and below the pilot). The audit reports only the 40.1 mm3 total. | Model: 6 mm thick bosses at the eight screw stations. By hand: epoxy the screws into their pilots, and run a bead of 5-min epoxy along the tail seam (x 120 to 152) when closing. Today the rear 30 mm of the clamshell and the tail face are unfastened. The tray's rear is then held only by the deploy N20's shaft, so landing loads go through that motor. |
| B4 | **The D sockets give the pods and the leg large free play.** The Ø3.4 socket (flat 0.5) on the Ø3.0 N20 stub (flat 0.5), at 0.2 radial play, turns ±11° before the flats bear with the parts concentric, and up to ±27° using the float in the bores. Each outer D fit (pod coupler in the arm head, leg coupler in the cheek) adds ±4° concentric, up to ±8°. That is 30° to 70° in total from one flank to the other. The sim assumed 3°. | Shapely check of the as-modelled profiles (coupler sections from the meshes: socket x ±1.70, flat at 1.20). Launcher run with the pods drooped (79 launch states): 5° droop gives 240 mm3 into the stop springs; 10° gives 1913 mm3; 15 to 20° hits the rails, launch springs and switches. | Model, simplest: press-fit sockets (stub socket Ø3.05 with its flat at 1.03; arm-head D bore 7.05; cheek D bore 6.05), checked on a coupon first. By hand: with each pod hard on its fold stop and the worm driven to stall in the fold direction, put a drop of CA or epoxy in each socket and D fit. Without this, (a) a pod that hangs 5° off its fold stop hits the launcher, and with two separate couplers only one pod can be on its stop; (b) under tow a pod swings back through all its play before the worm catches it, which levers the drill out of the soil. |
| B5 | **The bridle has no path through the launcher, and no ring or bridle step exists.** The bridle legs cross the launcher's carriage yoke, seat pushrod, rail rear and carriage. Base is parked and the slots listed in DART_V31_NOTES were never made. The ring is in no list, no model and no run-sheet step. | My run of the two legs (Ø1 line, eyes to a ring 48 mm behind the tail) against the base launch states: carriage yoke 5.9 mm3 at home, seat pushrod 1.0, rail rear 15.8 at the shot, carriage 3.2 as the dart leaves. launcher_fit.py never models the legs; it also skips the old line model. | Base: the 3 mm slots in the seat pushrod lug and the tether tube (notes, 9 Nov) before any shot. Ring and knots: section 3. |
| B6 | **The firmware auto sequence cannot anchor v34.** v4.2 still runs kick, then foot, then pods. The leg swings 178° in air and stalls at its bed in the nose; K_SWING reads that as an end stop with no ground (F_NO_GROUND). If it got past that, F_AFTER and R_FOOT drive the unwired foot alone, which trips F_NO_CURRENT after 500 ms. There is no righting move, and auto refuses to start when the dart is not upright. | kedge_logic.h K_SWING / F_AFTER / R_FOOT and the lone-motor no-current check (lines 850-905, 1011-1018). | Build day: drive by hand (1/! pods, 2/@ leg, 3/# drills); manual keys are unaffected. dart-fw needs a v34 sequence: no foot, leg used only for righting, pods plus drills. |

---------------------------------------------------------------------------------------------------

## 2. Parts, consumables and tools versus what Harry has

Sources: CLAUDE.md section 3 ("s3", all bought, including the third worm N20), the v33 run sheet's bought list (already in hand
for v33), docs/build/src/03_parts.md and its tools list.

| Item | v34 needs | In hand? | Note or substitute |
|---|---|---|---|
| XIAO ESP32S3, INA219, BNO085, MP1584 (5.04 V), Schottky, Caddx Ant, Zeus VTX, u.FL antenna | 1 each | yes (s3) | |
| DRV8833 | 2 | yes (3) | the third goes to the base |
| OTOTEC 2S BMS stick | 1 | bought (s3), not calipered | measure it before deck 2 goes on (5 mm under deck 2) |
| HIBATT 18350 | 2 | yes | |
| Worm N20 | 2 (leg, deploy) | yes (3) | foot N20 is the spare |
| Drill gearmotor | 2 | yes (3) | |
| M3 nut | 2 (auger hex pockets) | yes (s3) | |
| M3 x 8 countersunk | 8 radial | yes (v33 list had 9) | rear four have nothing to bite (B3) |
| M3 x 8 screw | 2 (pod caps) | yes (s3 "2 coupler screws") | 6.5 mm into a 7 mm pilot: fits |
| M3 grub screw | 0 | 1 (v33 foot) | the righting_leg note says "M3 grub screw in each cheek top"; there is no hole and no run-sheet step. Stale note; not needed, but see B4 |
| F693ZZ, washers | 0 | yes | spares |
| Battery spring | 1 | yes (2 from the AA holders) | the disc recess is Ø10 round; a square AA contact may need trimming |
| Contact plate | 1 | yes (2 plates) | |
| **BM contact "0.3 nickel strip with a tab"** | 1 | **not on any list** | use the second AA-holder flat plate; it has a solder tab. The front spring takes the extra thickness; the tab must pass the 4.0 x 1.5 slot in deck 1 |
| JST-XH 2-pin socket + shorting plug | 1 | yes | its wire loop sticks out where the left bridle leg runs (section 3) |
| JST-XH 3-pin socket | 1 | yes | does not fit its seat (B2) |
| Capacitor 470 to 1000 uF | 1 | yes | modelled Ø6.3 x 11 at y 15.3; a Ø8 to Ø10 can hits the wall there: check the can size |
| Ballast putty | 14.6 g at 9 g/cm3 (11.3 g if lead putty at 7, 17.8 g if tungsten at 11) | yes (15 g, s3) | cannot go in (B1) |
| Tether line | about 0.25 m for the bridle | yes: 1 mm Dyneema, 20 m (base list) | |
| **Bridle ring** | 1 | **not defined anywhere** | soft ring from the same line, or a printed PPA-CF ring (section 3) |
| Wire 22 / 24 / 26 / 28 AWG, 26 AWG silicone for the drill leads | per cut list | yes (v33 list: silicone 22, 24, 26, 28) | two 26 AWG silicone leads (OD 1.3 to 1.5) in a Ø3 hole is tight |
| Heat shrink, threadlocker | | yes (tools list) | |
| CA | skin plugs; also the couplers (B4) | probably (v33 list: "threadlocker (or CA)") | otherwise 5-min epoxy, which he has; plugs are then not removable |
| **Silicone or hot glue** (drill-lead holes) | dab x2 | **not on any list** | 5-min epoxy at the hole; the leads flex in the outside loop, not in the hole |
| Double-sided tape (DRV boards) | | foam tape (tools, v33 list) | |
| **Hacksaw or rotary cut-off, and a file** (cut 4.2 mm off each leg N20 stub) | | **not on the tools list** | flush cutters will not cut a 3 mm steel shaft. Clamp the gearbox and keep it cool while cutting |

---------------------------------------------------------------------------------------------------

## 3. Tether: rope path, ring, knots, rubbing, eye strength

**Path in the model.** Rover line, then a ring on the axis about 48 mm behind the tail (z 2.7), then two legs of about 78.6 mm.
Each leg ends at an eye lug: a Ø3.2 hole along y, 0.7 mm chamfers, in the outboard 5 mm of each arm head (y 29.15 to 34.15).
The eyes sit at (128.1, ±31.65, 2.7) folded and (134.3, ±31.65, 2.0) deployed. Nothing on the dart defines the ring, its
material, the leg lengths or the knots.

**Clearance, from straight segments checked against every mesh (line Ø1):**
- Folded (launcher, flight): the straight leg passes through the outboard-aft lower corner of its pivot block (x 137.5 to
  140.5) and 0.1 mm under the arm head's flat bottom. The real leg wraps that corner with about 2 mm of deflection. The
  corner is the block's r 1.2 side edge, not sharp, and line tension in flight is low (0.25 to 0.4 N). Acceptable.
- The leg then passes 2.5 mm outside the left socket opening and 2.3 mm off the tail edge (r 2.5 fillet).
  **The ON-loop shorting plug's wire loop sticks out of that opening, right under the left leg.** A snag pulls the ON plug,
  which powers the dart off. Keep the loop tight and tape it flat.
- During deploy (0 to 46.75°) the eyes move 6.2 mm aft. At 15° the leg still touches the block corner (−0.5 mm); at 30°
  it is clear by 0.2 mm; deployed (tow) it is clear by 1.1 mm and 2.5 mm off the tail edge.
- Righting: the leg sweeps the top centreline (|y| up to 18.5 at the pad), and the bridle runs low on the flanks. They never
  cross. An off-axis tow wraps one leg round the r 2.5 tail edge: rounded PLA, fine.
- Launcher: blocked (B5).
- The line must exit the y-axis hole and turn 60 to 70° over the 0.7 mm chamfer. That is a tight bend for 1 mm Dyneema
  (rated about 667 N); with a knot it gives roughly 40 to 50 % of rating, still about 5x at 55 N.

**Buildable recipe from parts in hand (my suggestion):**
- One 200 mm piece of the 1 mm Dyneema, its ends tied through the two eyes.
- Knot: a figure-8 loop or a palomar, knot on the outboard face, tails trimmed to 5 mm and locked with CA or epoxy. Keep
  the tails clear of the auger hub base, which spins 1.6 mm forward of the lug.
- Ring: a soft ring, i.e. a small figure-8 loop tied in the end of the rover line with the bridle threaded through it, so
  it slides and shares the pull as the sim assumes. Or a printed PPA-CF ring, OD 12, 3 mm round section, which fits the
  launcher's Ø14 pusher hole.
- Do not use an M3 washer: its stamped edges cut Dyneema.
- A fixed apex knot also works, but then an off-axis pull loads one leg with the full 55 N.

**Eye strength at 55 N.** Ligament 1.6 mm from the hole to the lug edge, 5 mm thick (3.6 mm at the chamfers), PLA. The arm
prints standing on its stop face, so part of the tow crosses layer lines (the pull is at 0.69 of the layer normal in the
arm frame).
- Shear-out about 4 MPa; ligament bending 10 to 18 MPa (upper bound).
- Margin about 2 with one leg taking the full 55 N; about 3.5 with both sharing (30.5 N per leg at 25.6°).
- Adequate, not generous. Print the arms with 100 % infill; at 5 mm with 4 walls the lug is nearly solid anyway.
- The coupler (Ø7 PPA-CF) carries the same 55 N into the block at about 10 MPa bending: fine.

---------------------------------------------------------------------------------------------------

## 4. Other findings, ranked

1. **Cells cannot slide in from the side (run sheet 5.5).** The four saddle rings wrap 270° with a 14.9 mm gap at the
   bottom (z −16), and both discs block axial entry. Each 18.66 mm cell must be snapped up from below, spreading each 1.2 x
   2 mm PLA ring by 1.9 mm a side (about 1 to 1.5 % strain). Expect cracks. Fix: open the rings to 19.2 mm at the bottom
   (they only locate; the belly carries the cells), or print the rings at 0.8 mm.
2. **The hull top cannot be lowered straight down (run sheet 6.4).** The XIAO's USB-C receptacle face (y −20.3) sits
   1.1 mm into the wall. My sweep of hull_top lowered over the populated tray collides with the XIAO from 5 mm above down to
   seated (9.1 mm3); nothing else collides, stubs included. Lower the top 1.5 mm to the left (+y) and slide it right as it
   seats. The tray drops into the hull bottom cleanly.
3. **Sealing claim not met.**
   - There is no seam lip. The hull_top note and the run-sheet print table describe "a seam lip hanging 1.5 below the
     split", but split_hull cuts exactly at z 0: the seam is a butt joint, backed by deck 1 only from x 26.7 to 142.
   - Open paths: the two pod coupler bores (0.2 mm annulus into the bay), each leg pocket through its Ø6.3 coupler bore
     into the knuckle and the bay, the tongue slots (0.15 mm), both JST openings, the USB-C window, and the unfastened
     rear seam (B3).
   - Result: dust-resistant at best. Tape over the USB and charge openings in the field.
4. **Run-sheet gaps.**
   - No bridle or ring step.
   - No direction mapping: key 1 must swing the drills down and key 2 must lift the leg off the tail over the nose; if
     not, swap that motor's two leads at DRV#1.
   - No "drive the pods up to stall before loading on the launcher", which takes up the B4 play toward fold.
   - The stale seam-lip print note, the stale grub-screw note, and "lower straight down" (item 2).
5. **CG claim (with putty) is not robust.** Folded z −0.15 with putty, +0.59 without; deployed −4.38 / −3.91; leg out
   −0.25 / +0.48. Both values sit inside the uncertainty of the catalogue bought masses (open item: weigh all bought parts),
   so "rests belly down" is neutral at best. Flight is not checked on v34:
   - x CG is 87.3 (57 % from the nose);
   - the 15 g that section 3 calls "needed for stable flight" is now in the belly;
   - the line pulls at the eyes (x 128) instead of the tail (152 and beyond), so its pitch arm about the CG drops from
     65 to 93 mm to 41 mm;
   - physics.py and flight_sim do not run on out34.
6. **Righting leg print.** Its crown (an r 21 arc) is the bed face, so real contact is a strip about 5 mm wide along
   120 mm (the 251 mm2 in print_report counts faces near the bed). Add a brim.
7. **Leg at the far rest.** The pad sits 80 mm ahead of the nose, 14 to 25 mm above the camera axis: it is in the top of
   the picture whenever the leg is out (and v4.2 auto would leave it out).
8. **Drill leads.** The 25 mm outside loop drapes over the front top of each pivot block. The direct distance from the
   hull hole to the neck exit is 19.3 mm folded and 19.2 mm deployed, so the length works, but the leads are exposed when
   the dart rolls on a pod.

---------------------------------------------------------------------------------------------------

## 5. The 0.55 N m worm peak in loam

**Which joint, which phase.**
- It is a **drill pod's deploy worm**: the right pod, seed 1017. The righting leg's worm carries 0 in every v34 run.
- It happens in the **tow-hold test after anchoring, at the 55 N step** (t 42.7 s), with the pod pushed 1.56° back off its
  46.75° stop (the 0.3° relax gap plus 1.26° of gear compliance).
- The sim records peaks only in the tow phase.

**Why.** Moment breakdown about that pod's hinge at the peak (my instrumented copy of simCore, unchanged physics):

| Term | N m | Direction |
|---|---|---|
| Soil holding the drill tip: 23.1 N forward, 9.5 N down, tip 40 mm deep, about 70 mm below the pivot | −0.79 | toward fold |
| Rope at the sim's eye (15 mm along the pod axis, about 10.9 mm drop deployed) | +0.23 | toward the stop |
| Pod weight | +0.01 | |
| **Worm** | **+0.55** | holds the rest |

- With the drills raked forward and the pivot above the soil, the soil always pushes the pod toward fold. The deploy stop
  only resists further deploy, so the worm takes the difference.
- The sim gives each pod **its own worm** (stall 0.21 N m). In that run the left pod sat on its stop carrying nothing, so
  the 0.55 is one pod's share. v34 has one worm on one axle.
- Higher than the study's 0.24 (which I reproduced exactly with the same runner): v34's pivots are 8 mm further forward and
  its stop is 46.75 instead of 50. Putting those back gives 0.41. The lighter dart accounts for the rest.

**Variants (40 random landings each, loam unless stated):**

| Case | Per-pod worm, separate worms (sim's metric): max / median | One worm on a stiff axle (sum of both pods, quasi-static): max / median |
|---|---|---|
| v34 as checked (sim eye), seeds 1000+ / 2000+ | 0.551 / 0.328; 0.603 / 0.334 | 0.150 / 0.127; 0.149 / 0.132 |
| Real v34 lug (3.9 fwd, 6.8 down), seeds 1000+ / 2000+ | 0.512 / 0.226; 0.597 / 0.246 | 0.390 / 0.133; 0.163 / 0.136 |
| Lug 3.9 fwd, 10 down (stop springs moved 3.5 mm) | 0.460 / 0.185; 0.402 / 0.193 | 0.158 / 0.132; 0.162 / 0.133 |
| Lug 20 mm below the pivot (cannot be built, sensitivity only) | 0.430 / 0.100 | 0.781 / 0.145 |
| Real lug, stop 40 / stop 55 | 0.481 / 0.213; 0.503 / 0.198 | 0.361 / 0.140; 0.322 / 0.118 |
| Real lug, sand (20 runs) | 0.274 / 0.143 | 0.147 / 0.107 |

Real lug over 80 runs: the per-pod value exceeds the 0.21 stall in 48 runs; one worm on a stiff axle exceeds it in 1 (0.39).

**The eye cannot fix it.** To balance one pod at 27.5 N per leg, the eye would need about 21 mm of drop when deployed. The
arm head allows about 8 mm under the launcher's stop springs, and the spinning hub starts 8.7 mm along the axis. The stop
angle barely matters.

**Simplest fix (geometry only): make the axle stiff so both pods load the one worm as a pair.**
- Change: press-fit the couplers. Stub sockets Ø3.4 to Ø3.05 (flat at 1.03), arm-head D bores 7.25 to 7.05, cheek D bores
  6.2 to 6.05. Coupon first.
- Effect on load: the pod sitting on its stop then cancels the loaded pod's fold-ward moment through the shaft. The single
  deploy worm sees a median 0.13 N m and at most 0.16 in 79 of 80 runs (one 0.39), i.e. under the 0.21 stall: "a little
  load".
- The same change fixes B4: no droop into the launcher, and the drill does not swing back through 30° or more before the
  worm holds.
- Optional second step, once base moves the launcher stop springs 3.5 mm down: EYE_DROP 6.8 to 10 cuts the single-pod case
  by another 10 to 30 %.
- Caveat: the stiff-axle numbers are quasi-static, taken from the sim's own states. A sim with one shared worm would
  confirm them.

---------------------------------------------------------------------------------------------------

## 6. Checks that passed

- Hands: left pod right-hand, right pod left-hand (from the meshes; pitch 12.0). They counter-rotate on one channel with
  the right leads swapped, DRILL_SIGN +1 as v33.
- Channels against the firmware: deploy DRV#1 A (D0/D1), leg on the old kick channel DRV#1 B (D2/D3), drills DRV#2 A
  (D8/D9) with the right motor swapped on the same pads, foot channel DRV#2 B (D6/D7) unwired.
- Insertion: tray into the hull bottom clean; hull top over the leg and deploy stubs clean (USB-C aside); the leg's cheeks
  drop into open pockets; couplers and plugs go in from outside.
- Collision sweep, hard stops, launcher fit of the folded body, wire room: as the audit says (only the moving pods' play is
  not modelled, B4).
- Fasteners: pod-cap M3 x 8 gives 6.5 mm into a 7 mm pilot.
- Supports: the leg N20 stubs ride in their couplers in the hull bores; the deploy stubs in the pod couplers in the pivot
  block bores (bore clearance 0.15 to 0.2 is under the 0.2 socket play, so the motors carry no radial load).
- Eye lugs: clear of the launcher stop springs by 0.6 mm folded, if the pods are exactly folded.

## 7. How the numbers were made

- Meshes: shells per part (trimesh split); sections; segment against mesh distance and inside tests; manifold overlaps.
- Insertion: hull_top lowered from +30 mm, and the populated tray lowered into the bottom, in 0.25 mm steps.
- Bridle and drooped pods: run against the base launch states (79) with base/tools/assembly.py, as launcher_fit does.
- Socket play: shapely rotation search on the as-modelled D profiles.
- Sim: sim_righting/v34_check/run34.mjs with a patch that records, during the tow, the per-joint peak, its time and tension,
  the soil / rope / gravity moments about each pod hinge, and max(0, −(net L + net R)) for one shared worm. The v34
  overrides are as in v34_righting_check.json, plus `D.tether.eyeY` for the real lug.
