# Drum subsystem notes (worker cad-drum): main shaft, dog clutch, N20 selector drum

Status: v5, 3 Nov 2026 12:30 Sydney. Conforms to params.py v1.2 (and D23, D24, D25 from v1.1). Deviations are in
section 0.

Every number lives in code:
- `_drum_kin.py` DEFAULTS: angles, groove and brake profiles, Gray code, stop plan, timing, loads. It has no CAD
  imports.
- `drum.py` GEOM: geometry. G() reads params.LAYOUT.axes (main, drum, capstan origins), deck_top_z and
  DRUM.reel_output, and derives the fork pin direction and the idler position from them.
- Firmware table: `drum_table.json` (`python cad/parts/_drum_kin.py --json`).

All coordinates: world mm. s = world y (distance from the adapter plate face). Local frame L (joint drum_base):
x = world x, y = world DOWN.

## 0. Deviations from params v1.0 (all forced; please fold into params)

| params | mine | why |
|---|---|---|
| DRUM.end_stops (-8, 358) | groove ends at -4 and 352 | (-8, 358) spans 366 deg: impossible on a single-turn drum with an absolute code. A 4 deg unreachable gap is needed so the code ramps never alias |
| targets WRAP_START 62, WRAP_END 302 | park angles 56 and 309 | 62 and 302 sit exactly on code edges, so a stop there reads either code. 56 is mid WRAP_PREP; 309 is in N_TO_R with the cup locked and the collar still NEUTRAL (the collar ramp starts at 318). The wrap window itself is exactly 62..302 |
| REEL cup gear driven by a 48T idler output | 42T m1.5 | 7 cup turns need 4 pinion turns x 42/24. 48/24 gives an even number (8 with 4 turns); 3.5 pinion turns would break the lock phase. Centre to the cup gear: 49.6 |
| REEL zone 340..355 | REEL dwell 340..352 (target 347) | end stop |
| WRAPPED park 309, clutch ramp 302..316 | park 314, clutch ramp 302..312 (FIRM from 312) | the tidy at the park needs FIRM. A 302..309 ramp at FIRM 5 N (pushrod about 15 N) needs 1.2 N m at the drum, under the 2x N20 margin. 314 is still in N_TO_R and the collar moves only from 318. A 30 percent rate error lands at 310.4..317.6 (clutch 0.92..1.0) |
| L_TO_N clutch ramp 8..32 | clutch ramp 30..40 | after the collar shift, so the two loads do not add (margin 2.3); FIRM by the NEUTRAL park 41 |

Everything else follows params: stack bands, drum axis (60, z -112), reel_output m2 50T face 12 at y 50.5..62.5,
bus gear m1.5 50T at y 40..48 (D25), zones and Gray codes (A B C), the contacts SW_DRUM_A/B/C, SW_DOG_L/R, SW_N20 and
SW_INDEX, magnet face y 178.5, 5 N m limiter.

## 1. What it does

- The wiper turns the coupler hub. A printed shear pin (about 5 N m, D8) drives the outer coupler, which carries:
  - the take-up bus gear 50T m1.5 (D15/D25, after the limiter), meshing cad-reel c1 46T at 72.1;
  - the 6-bump SW_INDEX cam (D12);
  - the hex socket for the main shaft.
- REEL output gear m2 50T (D20): free on a sleeve keyed to the hex. The sleeve is screwed to the coupler.
- LAUNCH dog ring: keyed into the cad-launch launch sleeve flange.
- DOG COLLAR: on the shaft hex, 4 back-cut roofed dogs each face.
- SELECTOR DRUM: two printed parts on a fixed d10 axle, driven by a self-locking right-angle N20 through 10:120.
  - groove 1 moves the collar through a long compliant fork;
  - track 2 sets the cad-reel slip clutch lever;
  - a segment gear with a locking arc turns the wrap idler only in the wrap window;
  - three Gray code bands are read by SW_DRUM_A/B/C.
- SW_DOG_L and SW_DOG_R close only when the collar is fully engaged (screw heads on a fixed post, nut in the yoke
  tab).
- SW_N20: N20 swung into mesh.
- Hand fallback: swing the N20 cradle out on its pivot screw, then turn the hand lever on the drum.

## 2. Main axis stack (s = world y)

| s | part | notes |
|---|---|---|
| 0.5..21 | `drum_coupler_hub` PPA-CF OD 26 | wiper bore from params.WIPER (d10, flats 8, M8). M8 nut in a round pocket from s 14.5 (13 mm socket) |
| 0.5..48 | `drum_coupler` PPA-CF OD 40 | runs on the hub; `drum_shear_pin` d4 radial at s 11.5, single shear at r 13 (about 5 N m at 30 MPa). Hex socket 22 AF s 30..48; M3 x 45 cross screw at s 35; 2 tapped M3 holes in the end face for the sleeve |
| 3..9 | SW_INDEX cam on the coupler | 6 bumps, base r 20, nub lift 2.5, 24 deg wide at 50 percent |
| 40..48 | bus gear on the coupler | 50T m1.5, face 8, tip r 39 |
| 30..106 | `drum_shaft` hex 22 AF | |
| 106..198.5 | spigot d12 | runs through the launch sleeve bore (12.35) and the aim housing bushing (D23); 1 mm flat from s 194.5 for the cap screw |
| 48..62 | `drum_sleeve_reel` | flange OD 44 s 48..50 (2 x M3 x 10 into the coupler), journal OD 30 s 50..62 |
| 50.5..69.5 | `drum_gear_reel` m2 50T face 12 (s 50.5..62.5), hub to 64.5, dogs 64.5..69.5 | free on the sleeve; inner lip on the sleeve end. Lewis 10 MPa at 5 N m |
| 72..98 | `drum_collar` at NEUTRAL | dogs 72..77 and 93..98, groove 82..88 (root r 26). It moves -7 to REEL (dog tips at 65) and +7 to LAUNCH (tips at 105) |
| 100.5..111 | `drum_dogring_launch` | dogs 100.5..105.5, body 105.5..111 (OD 62). 3 slots in the back face; 3 x M3 tapped at r 28.5 from the sleeve side |
| 195..201.5 | `drum_magnet_cap` | magnet pocket d6.2 x 2.6, face at s 201.5 (params magnet_face_y); radial M3 x 6 onto the spigot flat. AS5600 bracket: cad-launch |

### 2b. Launch sleeve interface (cad-launch)

- The flange face is at s 111.
- Lugs: 3 lugs 6.0 wide x 3.0 tall at r 15..21, at 60, 180 and 300 deg (dog 0 + 45 deg is 0). They go into my
  6.4 x 3.2 slots.
- Screws: 3 x M3 clearance holes at r 28.5 (0 / 120 / 240 deg). The heads sit on the sleeve side and thread into my
  ring.
- Spigot bushing d12.35.
- Thrust: two-way axial location at the frame outboard seat.

## 3. Dog clutch

- 4 dogs, r 18..26, height 5, root width 36 deg. Engaged play +-8 deg.
- Both flanks back-cut 4 deg: this resists walking out. Tips roofed 1.2 mm.
- Neutral gap 2.5, stroke 7, engagement 4.5, 0.5 tip to root.
- Tip to tip: the yoke (2.8 thick flat PLA, curved out-of-plane spring, about 1.5 N/mm) bends up to 4.5 mm and the
  drum still arrives. SW_DOG_x stays open.
- Contacts: the yoke tab carries an M3 nut. SW_DOG_R and SW_DOG_L screw heads on the fixed post touch it when the
  collar is within 0.5 mm of full engagement. The yoke compliance gives about 0.8 N of contact force at full
  engagement.

## 4. Drum (axis at world x 60, z -112; s 1..142)

`drum_drum_a` (PLA, prints standing on the gear face; 45 deg cones under every outward step):

| s | feature |
|---|---|
| 8..16 | drum gear 120T m0.8 x -0.3 (tip r 48.6). N20 pinion 10T m0.8 x +0.3 (tip shortened 0.08 m), centre 52.1 at drum angle -40 (L: front, up): world x 99.9, z -78.5 |
| 18..29 | brake cam: base r 30, lift 0 (LIGHT) or 4 (FIRM). Follower contact at L angle 180 (straight back), 15 N at FIRM |
| 52..60 | segment gear m1.25 from 60T, 39 teeth over the 240 deg window |
| 60.3..66 | locking arc ring r 38 |
| 29..77 | neck r 10, with the cones |
| 77..83 | hex spigot 16 AF into drum B |

`drum_drum_b` (PLA, prints standing on the barrel face):

| s | feature |
|---|---|
| 77.3..106 | groove 1 barrel r 30, groove 6.4 x 4 |
| 108..114 / 115..121 / 122..128 | code bands A / B / C (base r 22, nub lift 3, round-nub envelope). Levers at L angle 0 (front) |
| 128.7..135.7 | hex boss 14 AF for the hand lever |

Other parts:
- Fork:
  - pads in the collar groove at s 82.3..87.7;
  - yoke plane s 82.3..85.1;
  - sleeve 40 long on the rail;
  - follower pin d6 at s 91.8, entering the barrel from the main-axis side (L angle -118.2).
- `drum_rail` d10 at L (25, 58), i.e. world (25, z -58), s 1..142.
- `drum_dog_post`: clamped on the rail at s 63..71, bar to s 100.6. REEL boss to s 72.8; LAUNCH boss from s 94.6.
- Wrap idler: shaft d10 s 40..86 with the pinion (10T m1.25 x +0.4, tip shortened 0.02 m) at s 52..60 and the lock
  cam at s 60..66. Output gear `drum_idler_gear` 42T m1.5 at s 68.5..76.5. Axis at world (16.24, z -104.88).
  Journals: s 40.5..46.5 in `drum_bracket_idler2` and s 78..85 in `drum_bracket_idler`.
- Brackets A (s 1..7.7, with the N20 ear) and B (s 136..142) hold the axle and the rail.
- N20 cradle on the bracket A ear: pivot screw tapped in the ear at world (109.9, z -87.5), swing screw in a slot
  at (89.9, -87.5), SW_N20 nut in the ear at (109.9, -66.5).
- `drum_switch_block` s 105..128, front face 35.9 in front of the drum axis.
- `drum_index_block`: front of the coupler, face x 33.1, screwed to the adapter plate front face.

## 5. Drum angle table

| zone (params) | drum deg | collar | clutch | cup | code A B C |
|---|---|---|---|---|---|
| LAUNCH | -4..8 (target 0) | LAUNCH | LIGHT | home, locked | 100 |
| L_TO_N | 8..32 | ramp 8..32 | LIGHT (ramps to FIRM over 30..40) | locked | 110 (code from 6) |
| NEUTRAL | 32..50 (target 41) | NEUTRAL | FIRM | locked | 010 (from 34) |
| WRAP_PREP | 50..62 (park 56) | NEUTRAL | FIRM to LIGHT | lock frees at 62 | 011 |
| WRAP | 62..302 | NEUTRAL | LIGHT | 7 turns, linear | 001 |
| N_TO_R | 302..340 (park WRAPPED 314) | NEUTRAL to 318, ramp 318..340 | LIGHT to FIRM 302..312 | locked at 7 | 101 |
| REEL | 340..352 (target 347) | REEL | FIRM | locked | 111 (from 342) |

- Bit arcs (closed inside):
  - A: [-8, 34) and [302, 362)
  - B: [6, 62) and [342, 354)
  - C: [50, 354)
- Code 000 never occurs: fault DRUM_SENSOR. At both groove ends every band is fully up or fully down.

## 6. Stop plan and timing (`drum_table.json` has the same)

| target | dir | edge | code change | extra |
|---|---|---|---|---|
| LAUNCH 0 | - | 6 | 110 to 100 | 6 deg |
| NEUTRAL 41 | + | 34 | 110 to 010 | 7 |
| NEUTRAL 41 | - | 50 | 011 to 010 | 9 |
| WRAP_START 56 | + | 50 | 010 to 011 | 6 |
| WRAP_START 56 | - | 62 | 001 to 011 | 6 |
| WRAPPED 314 | + | 302 | 001 to 101 | 12 |
| WRAPPED 314 | - | pass down to edge 302 (101 to 001), then + 12 | | |
| REEL 347 | + | 342 | 101 to 111 | 5 |

Rate and timing:
- 8.1 deg/s nominal. A 30 percent rate error stays in the right zone (tested).
- Nominal times: LAUNCH to NEUTRAL 5.1 s, wrap window 29.6 s, WRAPPED to REEL 4.1 s, full travel 44 s.
- Timeout = 2 x nominal + 2 s, then DRUM_TIMEOUT.

## 7. Firmware rules (from both review rounds)

1. Wiper stopped before every drum move, except the fly-in into REEL (creep forward across 318..340).
2. Check SW_DOG_L / SW_DOG_R during and after every move:
   - LAUNCH needs DOG_L closed.
   - REEL needs DOG_R closed.
   - NEUTRAL, WRAP_PREP, WRAP and WRAPPED need both open.
   - Never enter the wrap window (62) with either closed.
3. Dog closed where it must be open, or open where it must be closed:
   - In LAUNCH or REEL: creep the wiper (REEL: forward only, sprag #2 blocks backward; LAUNCH: +-8 deg) until it
     closes.
   - Leaving LAUNCH or REEL under load: unload jog 8 deg (half the dog play) against the last load, then move.
   - Still disagreeing after 3 tries: SWITCH_DISAGREE.
4. Fault from REEL or N_TO_R: go to WRAPPED (314), never back through the wrap window. The collar is NEUTRAL there
   and the wraps and the locked cup stay on (the tow load is held by sprag #2). Fault from LAUNCH or L_TO_N: go to
   NEUTRAL (41).
5. Boot:
   - LAUNCH, L_TO_N, NEUTRAL, WRAP_PREP: go to NEUTRAL.
   - WRAP: stop, report; the operator chooses unwrap (-, wiper creeping forward for take-up) or finish (+).
   - N_TO_R, REEL: go to WRAPPED.
6. SW_N20 open means the hand fallback is active: no drum moves.
7. SW_INDEX: 6 pulses per wiper turn (degraded mode without the AS5600 magnet, D12).
8. Positive drum = N20 forward (DRV8833 #3 AIN1 high) = toward REEL = wrap direction (cup turns positive about
   world +Y, cad-reel's WRAP sense).

## 8. Contacts (printed: M3 screw head touching an M3 nut, pull-up, closed = LOW)

| name | GPIO (params) | where | closes |
|---|---|---|---|
| SW_DRUM_A / B / C | 10 / 11 / 12 | levers on bands s 108..114 / 115..121 / 122..128, drum front | nub on a bump, at 80 percent of the lift |
| SW_DOG_L | 13 | dog post LAUNCH boss | collar within 0.5 mm of full LAUNCH engagement |
| SW_DOG_R | 14 | dog post REEL boss | collar within 0.5 mm of full REEL engagement |
| SW_INDEX | 21 | index lever on the coupler cam | 6 times per wiper turn |
| SW_N20 | 47 | cradle screw on the bracket A ear nut | N20 swung into mesh |

Levers: PLA beam 1.6 thick, 30 long, round nub r 1.5, M3 x 6 in a 4 mm boss 8 mm below the nub. Contact gap at rest
1.76 (codes) and 1.47 (index).

## 9. Loads and strength

- N20 stall estimate 0.22 N m at 9 V (0.20 used). Drum stall capacity 2.16 N m at 12:1.
- Peak drum load 0.93 N m (at drum 32), margin 2.3 (params rule: at least 2). Peaks per load:

| load | peak N m | basis |
|---|---|---|
| fork shift | 0.86 | block force 6.75 N plus 3 N friction, x 1.68 racking (40 mm sleeve); groove pressure angle 44 deg, efficiency from the pressure angle with mu 0.3 |
| clutch ramps | 0.78 | 15 N pushrod at FIRM 5 N (D24, cad-reel to confirm), cam r 30 to 34, constant-slope ramps with blends |
| wrap | 0.78 | 0.06 N m at the cup x 10.5 / 0.81 |

  The ramps are placed so the loads do not add.
- At the 5 N m limiter:
  - hex torsion 2.5 MPa;
  - REEL gear Lewis 10 MPa (31 MPa even at 15 N m);
  - dogs 1.4 MPa.
- The shear pin limit could be 3.5 to 6 N m (printed PLA in shear). Bench-test one.

## 10. Wrap train (cad-reel)

- Window 62..302 = 40 pitches of the 60T m1.25 segment = 4 pinion turns.
- Lock: locked outside the window at whole turns (home and 7 cup turns). Home lock play about +-1.25 deg at the
  pinion; mesh backlash +-1 deg. At the window edges the pinion can run up to 10 deg ahead (under half a pitch,
  no jam; measured).
- Output gear 42T m1.5, face 8, in the cup gear plane s 68.5..76.5. Axis at world (16.24, z -104.88). Cup gear 24T
  m1.5 on the capstan axis, centre 49.6.
- Joint `drum_idler`: ring angle = -42/24 x idler angle.
- Cup hold in tow: the locking arc holds the pinion directly.

## 11. Frame interfaces (cad-frame)

| part | s | deck fixing |
|---|---|---|
| bracket A | 1..7.7 | 2 x M3 at world x 32 and 88 |
| bracket B | 136..142 | 2 x M3 at world x 32 and 88 |
| idler post | 78..85 | 2 x M3 at world x -8.8 and 23.2 |
| idler post 2 | 40.5..46.5 | 2 x M3 at world x -8.8 and 23.2 |
| switch block | 105..128 | 2 x M3 at world x 115 |

- All stand on the deck top (z -165).
- Index block: 2 x M3 into the adapter plate front face at world (43.1, z +6) and (43.1, z -36).
- Launch sleeve outboard seat: two-way thrust.
- AS5600 chip 1 to 2 mm beyond s 178.5 on the main axis.

## 12. Open items and measurements for Harry

- Wiper output shaft (the coupler hub is parametric).
- N20 output shaft diameter and flat (modelled d3 with a 2.5 flat).
- N20 stall torque (bench).
- Shear pin break torque (bench).
- Confirm a spare worm N20 for the drum (D19).
- AS5600 magnet (D12: optional, SW_INDEX fallback).

## 13. Printing (print_tf in drum.py; the pack tool writes the STLs in this orientation)

| part | material | on the bed | supports |
|---|---|---|---|
| drum_coupler | PPA-CF | bus gear face down | none (cam band 2.5 mm step only) |
| drum_coupler_hub | PPA-CF | motor-side face down | none |
| drum_shear_pin (x6) | PLA | lying on its 0.4 mm flat (shear across the layers) | none |
| drum_shaft | PPA-CF | lying on a hex flat | tree supports under the d12 spigot (it hangs 5 mm) |
| drum_sleeve_reel | PPA-CF | flange down | none |
| drum_gear_reel | PPA-CF | plain gear face down, dogs up | none (small lip) |
| drum_collar | PPA-CF | one dog face down | the body bridges 25 mm between the lower dogs (fine on the X2D) |
| drum_dogring_launch | PPA-CF | back face (slots) down, dogs up | none |
| drum_drum_a | PLA | N20 gear face down | none: 45 deg cones under every step |
| drum_drum_b | PLA | barrel face down | the 4 mm groove ceiling prints as short bridges |
| drum_idler | PPA-CF | pinion end down | none |
| drum_idler_gear | PPA-CF | flat | none |
| drum_n20_pinion | PPA-CF | flat | none |
| drum_fork | PLA | pad plane down (the thin yoke prints flat: it is the spring) | none |
| drum_dog_post, brackets, posts, blocks, cradle, strap, levers, hand lever, magnet cap | PLA | as modelled, largest flat face down | none |
| drum_axle, drum_rail | PPA-CF | lying on their 1 mm flat | none |

Infill: 40 percent gyroid default; 100 percent for the shear pins (their strength sets the 5 N m limit), the shaft,
the collar and the dog ring.
