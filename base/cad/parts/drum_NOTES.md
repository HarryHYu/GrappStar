# Drum subsystem notes (worker cad-drum): main shaft, dog clutch, N20 selector drum

Status: v3, 3 Nov 2026 11:45 Sydney. It follows BASE_SPEC v0.1 plus these coordinator decisions:
- coaxial launch sleeve with the LAUNCH dog ring, d12 spigot, main axis 165 above the deck;
- D16: drum order LAUNCH / NEUTRAL / WRAP / REEL;
- D15: take-up bus gear;
- D8: 5 N m torque limiter in the coupler;
- REEL output m2 40T.

Every number lives in code:
- `_drum_kin.py` DEFAULTS: timing, code table, loads. It has no CAD imports, so firmware and physics can use it.
- `drum.py` GEOM: geometry.

Both read overrides from `params.DRUM`: timing keys at the top level, geometry under `DRUM["geom"]`. The main axis
origin comes from `LAYOUT["main"]["origin"]`. Units: mm, deg, N, N m.

## 1. What it does

- One 12 V wiper motor turns the MAIN SHAFT through a coupler with a shear pin torque limiter (about 5 N m).
- The outer coupler carries the take-up BUS GEAR, which always turns with the wiper.
- REEL output gear (m2 40T): turns free on a keyed sleeve.
- LAUNCH dog ring: bolts to the cad-launch launch sleeve, which is coaxial.
- DOG COLLAR: keyed to the shaft hex, it couples the shaft to one of them or to neither.
- SELECTOR DRUM: two printed parts turning on a fixed d10 axle. A self-locking right-angle N20 drives it through a
  10:120 spur pair, and it holds any angle unpowered. It carries:
  - groove 1: moves the collar through a COMPLIANT FORK;
  - track 2: a cam for the cad-reel slip clutch lever;
  - a segment gear with a locking arc: turns the WRAP IDLER, and through it the cad-reel cup ring, only inside the
    wrap window;
  - three Gray code bands read by printed contacts SW_DRUM_A/B/C.
- SW_DOG on the fork (REQUIRED): reports a collar that did not reach its position (dogs tip to tip, or held engaged
  under load).
- Hand fallback: slide the N20 out of mesh, then turn the hand lever on the drum.
- Order (D16): the cup turns only with the collar in NEUTRAL; REEL engages only after the last wrap and the re-lock.
  Unwrapping runs in NEUTRAL with the wiper forward: the bus gear takes the line up and the capstan stays free.
  Fault means NEUTRAL.

## 2. Frames

- Local frame L, joint `drum_base`: z along the main axis, `s` = mm from the wiper boss face (= world origin plane).
  x = world +X. y = world DOWN. Default: main axis at world z 165, s = 0 at world y 0, axis along world +Y.
- Drum axis: parallel, 72 below the main axis (93 above the deck).
- Angles around the drum axis are in the L xy plane: 0 = +X forward, -90 = up toward the main shaft, 90 = down,
  180 = back.

## 3. Main axis stack (s from the wiper boss face)

| s | part | notes |
|---|---|---|
| 0.5..21 | `drum_coupler_hub` PPA-CF, OD 26 | wiper bore parametric from `WIPER` (d 10, flats 8, length 20, M8 thread). Round pocket d 19.2 from s 14.5 for the M8 nut and a 13 mm socket |
| 0.5..36 | `drum_coupler` PPA-CF, OD 40 | runs on the hub. Driven only through `drum_shear_pin` (PLA d4, radial, s 11.5, single shear at r 13: about 5 N m). Hex 22 AF socket s 22..36; M3 cross screw s 29 |
| 1..9 | bus gear on `drum_coupler` | D15: 40T m1.5, face 8, pitch r 30, tip r 31.5. After the limiter |
| 22..96 | `drum_shaft` hex 22 AF PPA-CF | |
| 96..160 | `drum_shaft` d12 spigot | inside the launch sleeve bushing; M3 cross hole at s 154 for the magnet cap |
| 36..54.5 | `drum_sleeve_reel` | hex bore, journal OD 30 (s 40..54.5), flange OD 44 (s 36..40) on the coupler face |
| 40.5..59.5 | `drum_gear_reel` m2 40T, face 14 at 40.5..54.5, dogs 54.5..59.5 | free on the sleeve. An inner lip (r 13..17.5) sits on the sleeve end, so the gear is held both ways |
| 62..88 | `drum_collar` at NEUTRAL | dogs both faces, groove 72..78 (root r 26). It moves -7 to REEL and +7 to LAUNCH |
| 90.5..101 | `drum_dogring_launch` | dogs 90.5..95.5; prints flat on its back face |
| 156..164 | `drum_magnet_cap` | 6 x 2.5 diametric magnet in the end face. AS5600 chip about 2 mm beyond (frame bracket) |

### 3b. LAUNCH dog ring to launch sleeve (cad-launch)

- The sleeve flange faces the motor at s 101.0. It is OD 64 and at least 8 thick.
- Lugs: 3 lugs 6.0 wide x 3.0 tall at r 15..21, at 60, 180 and 300 deg (dog 0 + 45 deg is 0). They go into my
  slots, which are 6.4 x 3.2.
- Screws: 3 x M3 holes at r 28, 0 / 120 / 240 deg, with nut traps on the outboard side. My ring has head
  counterbores.
- Bore at least 12.6, plus a d12.35 spigot bushing.
- Thrust both ways: the frame outboard seat must locate the sleeve axially in both directions. Engaging LAUNCH pushes
  it outboard; disengaging drags it toward the motor by about 8 N.

## 4. Dog clutch (both directions drive)

- 4 dogs per face, r 18..26, height 5, root width 36 deg. Engaged play about +-8 deg.
- Both flanks back-cut 4 deg. This resists walking out under load; it does not pull the dogs in, because
  tan 4 deg is less than the friction.
- Tips roofed (1.2 mm).
- Neutral tip gap 2.5; stroke 7; engagement 4.5; 0.5 tip to root.
- The groove commands 0.25 mm overtravel in every dwell, so the yoke holds the collar home with about 0.4 N.
- Tip to tip: the yoke (2.8 thick flat PLA, curved out-of-plane spring, about 1.6 N/mm) takes the drum's move up to
  its hard stops at 5.5 mm. SW_DOG closes at 3 mm deflection.
- Firmware:
  - After EVERY drum arrival (LAUNCH, NEUTRAL, REEL), check SW_DOG.
  - If it is closed after arriving in LAUNCH or REEL, jog the wiper +-15 deg (up to 60 deg, 30 percent duty) until it
    opens.
  - If it is closed after arriving in NEUTRAL, the collar is held engaged by train torque. Jog the wiper about 14 deg
    (half the dog play, AS5600) against the last load direction and recheck. Raise SWITCH_DISAGREE after 3 tries.
  - Before leaving LAUNCH or REEL under load, do the unload jog (about 14 deg) first.

## 5. Drum (two printed parts on the fixed axle `drum_axle` d10, s 7..131, pressed into brackets A and B)

`drum_drum_a` (PLA, prints gear face down, every step goes inward):

| s | feature |
|---|---|
| 16..24 | drum gear 120T m0.8 x -0.3 (tip r 48.6). N20 pinion 10T m0.8 x +0.3, centre 52.1 from the drum axis at angle +30 (front, below) |
| 25..33 | segment gear m1.25 cut from 60T (tip r 38.75): 39 teeth over the 240 deg window |
| 33.3..39 | locking arc layer: ring r 38, cut where the pinion's lock cam (r 11) sweeps |
| 41..52 | track 2 brake cam: base r 22, lift 0 (LIGHT) or 4 mm (FIRM). Follower contact at angle 200 (back, a little up) |
| 52..66 | neck r 10 (the idler output gear passes over it at 1.3 mm) |
| 66..72 | hex spigot 18 AF into drum B |

`drum_drum_b` (PLA, prints barrel face down):

| s | feature |
|---|---|
| 66..96 | groove 1 barrel r 30, groove 6.4 x 4. Follower pin d6 at angle -90 (top), pin plane s 81.8 at NEUTRAL |
| 97..103 / 104..110 / 111..117 | code bands A / B / C: base r 22, nub centre lift 3 mm. The band is the inner envelope of a round nub (r 1.5), so the lever follows exactly. Ramp 6 deg, 50 percent point at the edge |
| 117..123.7 | hex boss 14 AF for `drum_hand_lever` (28 mm arm with a finger hole) |

Other parts:
- `drum_rail`: d10, at L (45, 38), s 7..131.
- `drum_fork` plus `drum_fork_stop`: the fork prints flat on the pad plane s 72.3.
- Wrap idler:
  - `drum_idler`: pinion 10T m1.25 x +0.4 at s 25..33, lock cam at s 33..39, shaft d10 to s 70 with a D flat at
    s 52..62.
  - `drum_idler_gear`: 42T m1.5, face 8, at s 54..62, keyed on the flat.
  - Axis at L (-44.33, 72). Journal s 62..70 in `drum_bracket_idler` (s 63..69).
- `drum_n20` in `drum_n20_cradle` (M3 thumb screw slot, slide 4 mm in +x to unmesh) with `drum_n20_strap` (2 x M3).
- Code switches: `drum_switch_block` (s 94..117) with 3 x `drum_sw_lever_*`.

## 6. Drum angle table (firmware, audit, sim)

| drum deg | name | collar | brake track | cup turns | code C B A |
|---|---|---|---|---|---|
| -2 | hard stop | LAUNCH | LIGHT | 0 | 001 |
| 0 | LAUNCH (dwell -2..8) | LAUNCH | LIGHT | 0 | 001 (to 6) |
| 8..32 | shift L to N | moving | ramps to FIRM | 0 | 011 (6..34) |
| 41 | NEUTRAL (dwell 32..328) | NEUTRAL | FIRM (tidy) to 50, ramps to LIGHT 50..62 | 0 | 010 (34..56) |
| 62..302 | WRAP window | NEUTRAL | LIGHT | 0 to 7, linear | 110 (56..302) |
| 316 | WRAPPED | NEUTRAL | FIRM from 310 (ramp 302..310) | 7, locked | 111 (302..328) |
| 328..346 | shift N to R | moving | FIRM | 7 | 101 (328..346) |
| 348.5 | REEL (dwell 346..351) | REEL | FIRM | 7 | 100 |
| 351 | hard stop | REEL | FIRM | 7 | 100 |

- Bit arcs (closed inside):
  - A (b0): [-6, 34) and [302, 346)
  - B (b1): [6, 328)
  - C (b2): [56, 355]
- Code 000 never occurs in travel: fault DRUM_SENSOR.
- Gray code: adjacent sectors differ by one bit, so a slow contact can only show the neighbour.
- At both hard stops every band is fully up or fully down (tested).

## 7. Stop plan (run to the edge, then the extra angle by time)

| target | dir | watch edge | code change | extra |
|---|---|---|---|---|
| LAUNCH | -1 | 6 | 011 to 001 | 6 deg |
| NEUTRAL | +1 | 34 | 011 to 010 | 7 deg |
| NEUTRAL | -1 | 56 | 110 to 010 | 15 deg |
| WRAPPED | +1 | 302 | 110 to 111 | 14 deg |
| WRAPPED | -1 | 328 | 101 to 111 | 12 deg |
| REEL | +1 | 346 | 101 to 100 | 2.5 deg (or run on to the groove end at 351 at low duty) |

Rate and timing:
- Nominal 8.1 deg/s: N20 about 20 rpm at 9 V with 20 percent droop, 12:1.
- Calibrate the rate every move from the time between known edges (6 to 34, 56 to 302, or 302 to 328).
- A 30 percent rate error still lands in the right sector (tested).
- Nominal times: LAUNCH to NEUTRAL 5.1 s, NEUTRAL to WRAPPED 34 s, WRAPPED to REEL 4 s, full travel 44 s.
- Timeout = 2 x nominal + 2 s. Then raise DRUM_TIMEOUT, stop, and try NEUTRAL once.

Rules:
- Wiper stopped before any drum move, except the fly-in into REEL, where the firmware creeps the wiper forward while
  the drum crosses 328..346 so the dogs drop in.
- Boot: read the code. Transition codes: move toward NEUTRAL (LN: +). WRAPPING, WRAPPED, NR, REEL: unwrap means
  going back to NEUTRAL (-) with the wiper creeping forward (bus take-up).
- Positive drum = N20 forward (DRV8833 #3 IN1 high) = wrap direction. The cup wrap sense matches cad-reel
  (positive about world +Y).

## 8. Contacts (printed: M3 screw head touching an M3 nut, ESP32 pull-up, closed = LOW = bit 1)

| name | where | closes when |
|---|---|---|
| SW_DRUM_A (b0) | lever 0, band s 97..103, drum angle 0 (+X) | nub on a bump (lift over 1.5 of 3 mm) |
| SW_DRUM_B (b1) | lever 1, band s 104..110 | same |
| SW_DRUM_C (b2) | lever 2, band s 111..117 | same |
| SW_DOG (required) | fork: nut in the yoke tab, a screw in each prong (wired together) | yoke deflected over 3 mm either way |

- Levers: PLA flexure 30 long, round nub r 1.5. An M3 screw goes through the beam 8 mm below the nub; its head sits
  1.1 mm from the block nut at rest.
- Wiring: nuts to GND, screws to GPIO (fw-base assigns pins). Debounce 20 ms.

## 9. Torque and strength

- N20 stall: estimate 0.22 N m at 9 V (0.20 used). Basis: bench 0.18 A stall and 0.025 A free at 8 V; assumed
  15,000 rpm motor, 556:1, efficiency 0.30. Bench-test before printing (50 mm lever, kitchen scale).
- Drum capacity at 12:1 with 0.9 mesh efficiency: stall 2.16 N m, working (35 percent) 0.76 N m.

| drum load | peak N m | basis |
|---|---|---|
| wrap | 0.78 | cup 0.06 N m at LIGHT brake x 10.5 gear-up / 0.81 |
| fork shift | 0.59 | 11.8 N fork, groove slope 35 mm/rad, efficiency 0.7 |
| brake ramp | 0.64 | 10 N follower (cad-reel design force) |

  The loads do not overlap. Peak 0.78 is 36 percent of stall.
- cad-reel quoted 0.15 N m at the cup with 5 N of line. With the brake LIGHT during the wrap (D16) the line is about
  0.5 N; if the cup needs more than 0.07 N m, go to fewer wraps or a 10:150 drum gear.
- Shaft and gears at the 5 N m limiter:
  - hex torsion 2.5 MPa;
  - REEL gear Lewis 11.5 MPa (34 MPa even at 15 N m stall);
  - dog bearing 1.4 MPa;
  - bus gear Lewis 14 MPa at 2 N m.
- Shear pin: 30 MPa assumed for printed PLA in shear, so the limit could be 3.5 to 6 N m. Harry: break one on the
  bench with a lever if possible.

## 10. Wrap train (to cad-reel)

- Segment gear: 40 pitches of 60T m1.25 = 240 deg of drum = 4 pinion turns (drum 62..302).
- Lock: locked below 62 and above 302 at whole turns, so the cup is at home or exactly 7 turns.
- Idler output gear 42T m1.5, face 8, at s 54..62, axis L (-44.33, 72).
- Joint `drum_idler`: its angle is the pinion angle (it decreases as the drum turns forward). Ring = -42/24 x idler.
- Cup ring 24T m1.5: centre 49.6 from the idler axis, any angle round it; 7 turns per wrap stroke.
- Cup hold torque in tow: the locking arc holds the pinion directly (cup 1 N m = pinion 0.57 N m).
- Brake follower: angle 200, s 41..52, lift 0 or 4 mm, 10 N.

## 11. Frame interfaces (to cad-frame)

| part | s | deck fixing |
|---|---|---|
| bracket A | 7..15.7 | 2 x M3, x +-26 from the drum axis |
| bracket B | 124..131 | 2 x M3, x +-26 from the drum axis |
| idler bracket | 63..69 | 2 x M3, x -65 and -23 |
| N20 cradle | about 15..45, at x about 30..60 | M3 thumb screw in a 4 mm slot |
| switch block | 94..117 | 2 x M3, front face x 31.6 from the drum axis |

- All stand on the deck (L y 165 = world z 0).
- Wiper adapter plate at s <= 0.
- Launch sleeve outboard bearing: two-way thrust.
- AS5600 board faces the magnet cap end at s about 166, on the main axis.

## 12. Open items and measurements for Harry

- Wiper output shaft: the coupler hub is fully parametric (`WIPER`).
- N20 output shaft diameter and flat (modelled d3, D flat).
- N20 stall torque (bench test).
- Shear pin break torque (bench test, 3 spare pins).
- Spare worm N20: kedge/CLAUDE.md lists one more to buy for the dart. Confirm a third worm N20 exists for the drum.
- AS5600 magnet size (6 x 2.5 assumed).
