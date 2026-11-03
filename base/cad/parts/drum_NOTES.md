# Drum subsystem notes (worker cad-drum): main shaft, dog clutch, N20 selector drum

Status: v6, 3 Nov 2026 13:05 Sydney. Conforms to params.py v1.2.6. Deviations are in section 0.

Every number lives in code:
- `_drum_kin.py` DEFAULTS: angles, groove and clutch profiles, Gray code, stop plan, timing, loads. It has no CAD
  imports.
- `drum.py` GEOM: geometry. G() reads params.LAYOUT.axes (main, drum, capstan origins), deck_top_z and
  DRUM.reel_output, and derives the fork pin direction and the idler position from them.
- Firmware table: `drum_table.json` (`python cad/parts/_drum_kin.py --json`). All codes in it are A B C strings.

Coordinates: world mm. s = world y (distance from the adapter plate face). Local frame L (joint drum_base): x = world
x, y = world DOWN.

State keys (params 1.2.3):
- wiper_deg drives the main shaft, coupler, bus gear and collar;
- sleeve_deg drives the LAUNCH dog ring (with cad-launch's sleeve);
- reel_deg drives the REEL gear;
- drum_deg drives the drum.

A state without wiper_deg takes cad-launch's sleeve angle.

## 0. Deviations from the params v1.0 table, all now in params v1.2.3

- Single-turn travel: end stops -4 / 352. The original -8 / 358 spanned 366 deg.
- Park angles off the code edges:
  - 56 (WRAP_PREP);
  - 314 (WRAPPED, inside N_TO_R: clutch FIRM from 312, collar NEUTRAL until 318).
- Idler output 42T (7 cup turns = 4 pinion turns x 42/24).
- Clutch ramps 30..40 and 302..312, which keep the 2x N20 margin at FIRM 5 N.
- One small difference: the cup lock frees at 62 (the window start), not 60. The lock layer is cut exactly where the
  pinion turns.

## 1. What it does

- The wiper turns the coupler hub. A printed shear pin (about 5 N m, D8/D26) drives the outer coupler, which
  carries:
  - the SW_INDEX cam (D12);
  - the take-up bus gear 50T m1.5 (D25, meshing cad-reel c1 46T at 72.1);
  - the REEL gear journal;
  - the hex socket for the main shaft.
- REEL output gear m2 50T (D20): free on the coupler journal, held by a retaining washer.
- LAUNCH dog ring: keyed to the cad-launch launch sleeve flange.
- DOG COLLAR: on the shaft hex.
- SELECTOR DRUM: drum A and drum B on a fixed d10 axle, self-locking right-angle N20 through 10:120. It carries:
  - groove 1 (long compliant fork);
  - track 2 (cad-reel clutch pushrod);
  - a segment gear with a locking arc (wrap idler);
  - three Gray code bands (SW_DRUM_A/B/C).
- SW_DOG_L / SW_DOG_R close only when the collar is fully engaged. SW_N20 closes when the N20 is swung into mesh.
- Hand fallback: loosen the swing screw, swing the N20 cradle out, turn the hand lever.

## 2. Main axis stack (s = world y)

| s | part | notes |
|---|---|---|
| 0.5..21 | `drum_coupler_hub` PPA-CF OD 26 | wiper bore from params.WIPER (d10, flats 8, M8 thread s 12..20). M8 nut at s 13.5..20 in a round pocket (13 mm socket) |
| 0.5..62 | `drum_coupler` PPA-CF | OD 40, 45 deg cone from s 24.7 up to the bus gear root; prints standing on its motor face |
| | (coupler) shear pin | `drum_shear_pin` d4, radial at s 9, single shear at r 13 (about 4.9 N m) |
| | (coupler) socket | hex 22 AF from s 30 |
| | (coupler) shaft retention | M3 x 20 tapped radially at s 34 into a dimple on a hex flat |
| 1..6 | SW_INDEX cam on the coupler | 6 bumps, base r 20, nub lift 2.5 |
| 40..48 | bus gear | 50T m1.5, face 8, tip r 39 |
| 48..50.3 | thrust step r 20 | the REEL gear bears on it |
| 50..62 | REEL journal OD 30 | |
| 62.05..63.5 | `drum_reel_washer` | r 12.9..17.4, 2 x M3 x 6 at r 13.2 into the journal end |
| 50.5..69.5 | `drum_gear_reel` m2 50T face 12, dogs 64.5..69.5 | bore steps to r 17.7 past the journal |
| 30..106 | `drum_shaft` hex 22 AF | |
| 106..198.5 | d12 spigot | through the launch sleeve bore (12.35) and the aim housing bushing (D23); flat from s 194.5 |
| 72..98 | `drum_collar` at NEUTRAL | dogs 72..77 and 93..98, groove 82..88. REEL engaged: tips at 65. LAUNCH engaged: tips at 105 |
| 100.5..111 | `drum_dogring_launch` | dogs 100.5..105.5, back face at 111 with 3 key slots, 3 x M3 tapped at r 28.5 |
| 195..201.5 | `drum_magnet_cap` | magnet pocket d6.2 x 2.6, face at 201.5; M3 x 6 radial onto the spigot flat. AS5600 bracket: cad-launch |

### 2b. Launch sleeve interface (cad-launch: please match)

- Sleeve flange face at s 111.
- Lugs on the SLEEVE: 3 lugs 6.0 wide x 3.0 tall, r 15..21, at 60 / 180 / 300 deg. They key into my slots, which are
  6.4 x 3.2. Note: launch.py on base-v1 has slots on the sleeve too, so there is no key; one side must have lugs.
  I keep the slots because the ring prints on that face.
- Screws: 3 x M3 clearance holes in the sleeve flange at r 28.5, 0 / 120 / 240 deg. The heads go on the sleeve side
  and thread into my ring. My ring has no room for counterbores between its dogs.
- The sleeve's two-way axial location is in cad-launch's aim housing (D23).

## 3. Dog clutch

- 4 dogs, r 18..26, height 5, root 36 deg. Engaged play +-8 deg.
- Both flanks back-cut 4 deg (resists walking out). Tips roofed 1.2 mm.
- Neutral gap 2.5, stroke 7, engagement 4.5.
- Tip to tip: the yoke (2.4 thick flat PLA, about 0.95 N/mm out of plane) bends up to 4.5 mm, the drum still
  arrives, and SW_DOG_x stays open.
- SW_DOG_L / SW_DOG_R: screw heads on the fixed dog post touch the nut in the yoke tab 0.5 mm before full engagement
  (about 0.5 N contact force through the yoke).

## 4. Drum (axis world x 60, z -112; s 1..142)

`drum_drum_a` (PLA, prints standing on the gear face; 45 deg cones under the cam and the segment gear):

| s | feature |
|---|---|
| 8..16 | drum gear 120T m0.8 x -0.3. N20 pinion 10T m0.8 x +0.3 at L angle -40, world (99.9, z -78.5) |
| 18..29 | clutch cam: base r 30, lift 0 / 4 mm, follower at L angle 180 (straight back), 15 N at FIRM |
| 52..60 | segment gear m1.25 from 60T, 39 teeth over 240 deg |
| 59.95..66 | locking arc ring r 38, sitting on the segment gear |
| 29..77 | neck r 10 |
| 77..83 | hex spigot 16 AF into drum B |

`drum_drum_b` (PLA, prints standing on the barrel face):

| s | feature |
|---|---|
| 77.3..106 | groove 1 barrel r 30, groove 6.4 x 4. The groove roof prints as a 4 mm ledge: clean it with a file if it droops |
| 108..114 / 115..121 / 122..128 | code bands A / B / C, base r 22, nub lift 3, round-nub envelope |
| 128.7..135.7 | hex boss 14 AF for the hand lever |

Fork:
- pads in the collar groove (s 82.3..87.7), yoke 82.3..84.7;
- sleeve on the rail s 32.3..98.3 at NEUTRAL (66 long, grease it);
- follower pin d6 at s 91.8, entering the barrel from the main-axis side.

Rail and dog post:
- `drum_rail` d10 at world (25, z -58), s 1..142.
- `drum_dog_post` clamps the rail at s 106.5..111.5, clear of the launch rail cheeks (y 123..131). Its bar runs back
  to s 69.8; REEL boss to s 72.8, LAUNCH boss from s 94.2. Clamp: slide fit on the rail, M3 x 8 tapped onto the
  rail flat. The two boss screws go in before the post goes on (no driver room later).

Wrap idler:
- `drum_idler` shaft: d10 journals in two posts (s 40.5..46.5 and 78..85), d12 between them (the shoulders locate it
  axially).
- Pinion 10T m1.25 x +0.4 at s 52..59.7; lock cam at s 60.3..66.
- `drum_idler_gear` 42T m1.5 on the D flat at s 68.5..76.5, with a thrust hub to post 1.
- Axis at world (16.24, z -104.88).

N20 cradle (on the bracket A ear):
- pivot M3 at world (109.9, z -87.5);
- swing M3 in a +-15 deg arc slot;
- SW_N20: nut in the cradle bottom face, screw tip tapped in the ear.

Contact blocks:
- `drum_switch_block`: s 105..128.
- `drum_index_block`: in front of the coupler, screwed to the adapter plate.

## 5. Drum angle table

| zone | drum deg | collar | clutch | cup | code A B C |
|---|---|---|---|---|---|
| LAUNCH | -4..8 (target 0) | LAUNCH | LIGHT | home, locked | 100 |
| L_TO_N | 8..32 | ramp 8..32 | LIGHT (ramp to FIRM over 30..40) | locked | 110 |
| NEUTRAL | 32..50 (target 41) | NEUTRAL | FIRM from 40 | locked | 010 |
| WRAP_PREP | 50..62 (park 56) | NEUTRAL | FIRM to LIGHT 50..62 | locked to 62 | 011 |
| WRAP | 62..302 | NEUTRAL | LIGHT | 7 turns | 001 |
| N_TO_R | 302..340 (park WRAPPED 314) | NEUTRAL to 318, ramp 318..340 | LIGHT to FIRM 302..312 | locked at 7 | 101 |
| REEL | 340..352 (target 347) | REEL | FIRM | locked | 111 |

- Code edges at 6, 34, 50, 62, 302 and 342 are the physical contact closing points: the bump ramps put the
  80 percent lift, where the contact closes, on the edge.
- Bit arcs (closed inside):
  - A: [-8, 34) and [302, 362)
  - B: [6, 62) and [342, 354)
  - C: [50, 354)
- Code 000 never occurs: fault DRUM_SENSOR.

## 6. Stop plan and timing

| target | dir | edge | code change (A B C) | extra |
|---|---|---|---|---|
| LAUNCH 0 | - | 6 | 110 to 100 | 6 |
| NEUTRAL 41 | + | 34 | 110 to 010 | 7 |
| NEUTRAL 41 | - | 50 | 011 to 010 | 9 |
| WRAP_START 56 | + | 50 | 010 to 011 | 6 |
| WRAP_START 56 | - | 62 | 001 to 011 | 6 |
| WRAPPED 314 | + | 302 | 001 to 101 | 12 |
| WRAPPED 314 | - | 342 | 111 to 101 | 28 (30 percent error lands 305.6..322.4: the collar has moved under 0.7 mm at 322) |
| REEL 347 | + | 342 | 101 to 111 | 5 |

Timing:
- 8.1 deg/s nominal. LAUNCH to NEUTRAL 5.1 s, wrap window 29.6 s, WRAPPED to REEL 4.1 s, full travel 44 s.
- Timeout = 2 x nominal + 2 s.

## 7. Firmware rules (drum_table.json "rules" has the same)

1. Wiper stopped before drum moves, with two exceptions:
   - the fly-in into REEL: creep the wiper forward over 318..340;
   - the unwrap: WRAPPED back to NEUTRAL with the wiper creeping forward for the bus take-up.
2. Check SW_DOG_L / SW_DOG_R during and after every move:
   - LAUNCH needs DOG_L closed;
   - REEL needs DOG_R closed;
   - NEUTRAL, WRAP_PREP, WRAP and WRAPPED need both open;
   - never enter the wrap window with either closed.
3. Dog not closed:
   - in LAUNCH: jog the wiper +-8 deg;
   - in REEL: creep forward only (sprag #2 blocks backward).

   Leaving LAUNCH or REEL under load: unload jog 8 deg first. SWITCH_DISAGREE after 3 tries.
4. Faults (D27, params 1.2.6):
   - In LAUNCH or L_TO_N: HOLD the drum unless SW_HOME is closed (spring not drawn), then NEUTRAL.
   - With the wraps on (WRAP, N_TO_R, REEL): HOLD where it is (DRUM_HELD), never NEUTRAL; the operator unwraps when
     safe.
   - REEL to WRAPPED (on operator command): by time from edge 342, never back into the wrap window.
5. Boot:
   - LAUNCH or L_TO_N: hold until SW_HOME, then NEUTRAL;
   - NEUTRAL or WRAP_PREP: go to NEUTRAL;
   - WRAP, N_TO_R or REEL: hold and report.
6. SW_N20 open means the hand fallback is active: no drum moves.
7. SW_INDEX: 6 pulses per wiper turn (degraded mode without the AS5600 magnet).
8. Positive drum = N20 forward (DRV8833 #3 AIN1 high) = wrap direction.

## 8. Contacts (printed: M3 screw head or tip touching an M3 nut, pull-up, closed = LOW)

| name | GPIO (params) | where | closes |
|---|---|---|---|
| SW_DRUM_A / B / C | 10 / 11 / 12 | levers on the bands, drum front | at 80 percent of the bump lift |
| SW_DOG_L | 13 | dog post LAUNCH boss (M3 x 8) | collar within 0.5 mm of full LAUNCH engagement |
| SW_DOG_R | 14 | dog post REEL boss (M3 x 8) | collar within 0.5 mm of full REEL engagement |
| SW_INDEX | 35 | index lever on the coupler cam | 6 times per wiper turn |
| SW_N20 | 47 | screw tip in the bracket A ear, nut in the cradle | N20 swung into mesh |

- Levers: PLA beam 1.6 thick, 36 long, round nub r 1.5, M3 x 6 in a 4 mm boss 8 mm below the nub. Peak strain
  0.66 percent (cad.lib.switch model, limit 0.8). Contact gap at rest 1.87 (codes) and 1.56 (index).
- The feet screw along x into the block face.

## 9. Loads and strength

- N20 stall estimate 0.22 N m at 9 V (0.20 used). Drum stall capacity 2.16 N m at 12:1.
- Peak drum load 0.92 N m, margin 2.34 (params rule: at least 2). Peaks per load:

| load | peak N m | basis |
|---|---|---|
| fork | 0.86 | yoke block force 4.3 N plus 3 N friction, x 2.04 racking: (1 + 2 mu 63.2/66) / (1 - 2 mu 36.5/66), mu 0.25 greased rail; groove 44 deg |
| clutch ramps | 0.78 | 15 N pushrod at FIRM 5 N (cad-reel: 10.8 N at 4 mm, design 15) |
| wrap | 0.78 | 0.06 N m at the cup x 10.5 / 0.81 |

  The loads never coincide.
- At the 5 N m limiter:
  - hex torsion 2.5 MPa;
  - REEL gear Lewis 10 MPa;
  - dogs 1.4 MPa.
- The 110 N tow maximum gives about 5.1 N m at the main shaft, at the limiter. The 50 N default gives 2.3 N m.

## 10. Wrap train (cad-reel)

- Window 62..302 = 40 pitches of the 60T m1.25 segment = 4 pinion turns.
- Lock: locked at home and at 7 cup turns. Lock play +-1.5 deg at the pinion; mesh backlash +-1 deg. Clean handover
  (one side always held).
- Output gear 42T m1.5, face 8, s 68.5..76.5, axis world (16.24, z -104.88). Cup gear 24T at 49.6.
- Ring angle = -42/24 x the drum_idler joint angle.

## 11. Frame interfaces (cad-frame)

| part | s | deck fixing |
|---|---|---|
| bracket A | 1..7.7 | 2 x M3 at world x 32 and 88 |
| bracket B | 136..142 | 2 x M3 at world x 32 and 88 |
| idler post 1 | 78..85 | 2 x M3 at world x -8.8 and 33.2 |
| idler post 2 | 40.5..46.5 | 2 x M3 at world x -8.8 and 33.2 |
| switch block | 105..128 | 2 x M3 at world x 115 |

- All stand on the deck top (z -165).
- Index block: 2 x M3 into the adapter plate front face at world (43.1, z +6) and (43.1, z -36).
- Clear the coupler cone (r up to 35 at s 24.7..40) and the bus gear (r 39, s 40..48).

## 12. Open items and measurements for Harry

- Wiper output shaft (the coupler hub is parametric).
- N20 output shaft diameter and flat (modelled d3, 2.5 flat).
- N20 stall torque (bench).
- Shear pin break torque (cad-infra coupon: about 8.3 kg on a luggage scale at r 60).
- Spare worm N20 for the drum (D19).
- AS5600 magnet (optional, SW_INDEX fallback).

## 13. Printing (print_tf in drum.py; the pack tool writes the STLs this way)

| part | material | on the bed | supports |
|---|---|---|---|
| drum_coupler | PPA-CF | motor face down | none (45 deg cone under the bus gear) |
| drum_coupler_hub | PPA-CF | motor face down | none |
| drum_shear_pin (x6) | PLA, 100 percent | lying on its 0.4 mm flat | none |
| drum_shaft | PPA-CF, 100 percent | lying on a hex flat | tree supports under the spigot |
| drum_gear_reel | PPA-CF | plain face down | none |
| drum_reel_washer | PPA-CF | flat | none |
| drum_collar | PPA-CF, 100 percent | one dog face down | the body bridges 25 mm between the lower dogs; groove roof 4 mm ledge |
| drum_dogring_launch | PPA-CF, 100 percent | slotted back face down | none |
| drum_drum_a | PLA | gear face down | none |
| drum_drum_b | PLA | barrel face down | groove roof ledge 4 mm |
| drum_idler | PPA-CF | pinion end down | none |
| drum_idler_gear, drum_n20_pinion | PPA-CF | flat | none |
| drum_fork | PLA | pad plane down (the yoke is the spring) | the far arm starts 2 mm up: let the slicer support it |
| drum_axle, drum_rail | PPA-CF | on their 1 mm flat | none |
| everything else | PLA | largest flat face down | none |
