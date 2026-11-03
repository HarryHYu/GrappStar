# Drum subsystem notes (worker cad-drum): main shaft, dog clutch, N20 selector drum

Status: v2, 3 Nov 2026 11:15 Sydney. It follows BASE_SPEC v0.1 and the coordinator's main-axis decision (coaxial
launch sleeve, LAUNCH dog ring, d12 spigot, main axis 165 above the deck).

Every number lives in code:
- `_drum_kin.py` DEFAULTS: timing, code table, loads. It has no CAD imports, so firmware and physics can use it too.
- `drum.py` GEOM: geometry.

Both read overrides from `params.DRUM`: timing keys at the top level, geometry under `DRUM["geom"]`. The main axis
origin comes from `LAYOUT["main"]["origin"]`.

Units: mm, deg, N, N m.

## 1. What it does

- One 12 V wiper motor turns the MAIN SHAFT, which is coaxial with the wiper output.
- REEL output gear (motor side): m3 24T. It turns free on a keyed sleeve.
- LAUNCH dog ring (outboard): it bolts onto the cad-launch launch sleeve, which is coaxial and holds the drive gear
  with sprag #1 and the rail pivot journals.
- DOG COLLAR: keyed to the main shaft hex. It slides between the REEL gear and the LAUNCH dog ring.
- SELECTOR DRUM: one self-locking right-angle N20 turns it through a 10:90 spur pair. It holds any angle unpowered.
  It carries:
  - groove 1: moves the collar through a COMPLIANT FORK;
  - track 2: a cam for the cad-reel slip clutch preload lever;
  - a segment gear with a locking arc: turns the WRAP IDLER, and so the cad-reel wrap ring, only past REEL;
  - three Gray code bands read by printed contacts SW_DRUM_A/B/C.
- Proposed extra contact SW_DOG on the fork: dogs landed tip to tip.
- Hand fallback: slide the N20 cradle out of mesh, then turn the hand lever on the drum's outboard hex.
- The order is physical: there are no wraps outside REEL, and the collar cannot leave REEL until the drum has turned
  back through the whole unwrap. Fault means NEUTRAL.

## 2. Frames

- Local frame L, joint `drum_base`: z along the main axis, `s` = mm from the wiper boss face (= the adapter plate
  face, world origin). x = world +X. y = world DOWN. Default: main axis at world z 165, s = 0 at world y 0, main axis
  along world +Y. If LAYOUT flips the motor to point along -Y, only `drum_base` changes.
- Drum axis: parallel, 72 below the main axis (93 above the deck).
- Angles around the drum axis below are in the L xy plane (0 = +X forward, -90 = up toward the main shaft, 180 = back).
- Envelope: x -90..+80, s 6..168 (main shaft to s 160 + magnet cap), from the main axis (+31) down to the deck (-165).

## 3. Main axis stack (s from the wiper boss face)

| s | part | notes |
|---|---|---|
| 0.5..36 | `drum_coupler` PPA-CF, OD 40 | wiper bore parametric from `WIPER` (d 10, flats 8, length 20, M8 end thread). M8 nut in a 13.4 AF pocket clamps it on the shaft (fit the nut through the socket). Hex 22 AF socket s 22..36. M3 cross screw at s 29 |
| 22..96 | `drum_shaft` hex 22 AF, PPA-CF | 15 N m stall: 7.5 MPa torsion |
| 96..160 | `drum_shaft` d12 spigot | runs in a d 12.35 bushing inside the launch sleeve. Spigot end: M3 cross hole at s 154 for the magnet cap |
| 36..55 | `drum_sleeve_reel` PPA-CF | hex bore, journal OD 30 s 40..55, flange OD 44 s 36..40 thrusts on the coupler |
| 40.5..59.5 | `drum_gear_reel` m3 24T, face 14 at 40.5..54.5, dogs 54.5..59.5 | free on the sleeve journal (bore 30.3). Lewis 29 MPa at 15 N m stall |
| 62..88 | `drum_collar` at NEUTRAL | dogs 62..67 and 83..88, body OD 60, fork groove 72..78 to r 26. It moves -7 toward REEL (s 55..81) and +7 toward LAUNCH (69..95) |
| 90.5..104 | `drum_dogring_launch` | dogs 90.5..95.5, body 95.5..101 (OD 62, ID 27), 3 lugs 101..104 |
| 101.. | cad-launch launch sleeve | see 3b |
| 156..164 | `drum_magnet_cap` | 6 x 2.5 diametric magnet in the end face (s 161.4..164). AS5600 chip about 2 mm beyond, frame bracket |

### 3b. LAUNCH dog ring to launch sleeve interface (proposed to cad-launch)

- The sleeve end flange faces the motor at s = 101.0. It is OD 64 and at least 8 thick.
- Screws: 3 x M3 clearance holes at r 28 (0, 120, 240 deg, measured from dog 0 + 45 deg), with M3 nut traps on the
  outboard side. The screws go in from my side; the ring has counterbores d 6 x 3.2 outside its dog ring.
- Lugs: 3 slots, 6.4 wide x 3.2 deep axially, spanning r 15..21, at 60, 180 and 300 deg. My lugs are 6.0 x 3.0.
- Bore: at least 12.6 at the flange, with a d 12.35 bushing for the spigot further in.
- Thrust: engaging LAUNCH pushes the ring and sleeve outboard, so the frame outboard seat needs a thrust face.
  Engaging REEL closes through my REEL sleeve into the coupler and the M3 cross screw.

## 4. Dog clutch (both directions drive: aim and cock on LAUNCH, wind on REEL)

- 4 dogs per face, ring r 18..26, height 5, root width 30 deg.
- Both flanks back-cut 4 deg (dovetail): torque pulls the dogs together in either direction, so they cannot walk
  out under load.
- Tips roofed (1.2 mm chamfers on both flanks).
- Neutral tip gap 2.5 each side; stroke 7; engagement 4.5 with 0.5 tip-to-root clearance. Engaged play about
  +-14 deg (big entry window, about 30 percent fall straight in).
- Dog bearing stress at 15 N m: 190 N per dog, 3.8 MPa.
- To shift OUT under a held train torque, the firmware first unloads: wiper jog 3 deg against the load, then move the
  drum.
- Tip to tip: the drum finishes its move anyway, because the thin fork yoke (2.8 thick, flat printed PLA,
  out-of-plane curved spring) takes it at about 1.6 N/mm up to 5.5 mm. Then SW_DOG closes. Firmware jogs the wiper
  +15 / -15 deg (up to 60 deg total, 30 percent duty) until SW_DOG opens.

## 5. Drum (part `drum_drum`, PLA, one piece, prints standing on journal A with a brim)

| s | feature |
|---|---|
| 6..16 | journal A d16 in `drum_bracket_a` (s 7..14) |
| 16..24 | drum gear 90T m1 x -0.3 (tip r 45.7). N20 pinion 10T m1 x +0.3 at 50.1 from the drum axis, angle +30 (front, below) |
| 25..33 | segment gear m1.25 cut from 64T (tip r 41.25): 39 teeth present over the 225 deg wrap window |
| 33..39 | locking arc layer: ring r 38 cut to r 35.5 where the wrap pinion's lock cam (r 11) sweeps |
| 39..52 | neck d14 (the idler output gear passes over it at 1.3 mm) |
| 52..62 | track 2 brake cam: base r 22, lift 0 / 2 / 4 mm (light / mid / firm). Follower contact at angle 200 (back, a little below) |
| 62..90 | groove 1 barrel r 30, groove 6.4 wide x 4 deep. Follower pin d6 at angle -90 (top), fork plane s 75 at NEUTRAL |
| 91..111 | code bands (base r 22, bump r 25, 2 deg ramps): A (b0) 91..97, B (b1) 98..104, C (b2) 105..111. Contacts at angle 0 (+X) |
| 116..125 | journal B d16 in `drum_bracket_b` (s 117..124) |
| 125..133 | hex boss 12 AF for `drum_hand_lever` (30 mm arm with a knob; short so it clears the launch drive gear) |

Other drum subsystem parts:
- Shift rail `drum_rail`: d10 PPA-CF with a 1 mm print flat, at L (45, 38) (45 forward, 38 below the main axis),
  s 7..124. It is pressed into brackets A and B.
- `drum_fork`: carrier with a rail sleeve, an arm to the hub under the collar, and a follower pin.
- `drum_idler`: one PPA-CF part, axis at L (-46.83, 72) (behind the drum, level with it).
  - Pinion 10T m1.25 x +0.4 at s 25..33.
  - Lock cam r 11 at s 33..39.
  - Output gear 48T m1.5 at s 40..48.
  - Journal d10 s 48..56 in `drum_bracket_idler` (s 49..55).
- `drum_n20` sits in `drum_n20_cradle` (M3 thumb screw slot). `drum_switch_block` carries 3 x `drum_sw_lever_*`.

## 6. Drum angle table (firmware and audit)

| drum deg | name | collar | brake track | wraps | code C B A |
|---|---|---|---|---|---|
| -10 | hard stop (groove end) | LAUNCH | 0 | 0 | 001 |
| 0 | LAUNCH (dwell -10..6) | LAUNCH engaged | light (0) | 0 | 001 |
| 6..34 | shift L to N | moving | 0 | 0 | 011 from 4 |
| 40 | NEUTRAL (dwell 34..46) | centred | 0 | 0 | 010 (36..44) |
| 46..74 | shift N to R | moving | ramps 0 to mid over 46..98 | 0 | 110 (44..76) |
| 87 | REEL (dwell 74..346) | REEL engaged | about 0.45 of full | 0 | 111 (76..98) |
| 100..325 | wrap window | REEL | mid (0.5) | 0 to 4, linear | 101 (98..327) |
| 326..337 | brake ramp mid to firm | REEL | ramps to 1 | 4 | 100 from 327 |
| 338 | WRAPPED (dwell 327..346) | REEL | firm (1.0) | 4 | 100 |
| 346 | hard stop (groove end) | REEL | 1 | 4 | 100 |

- Code 000 never occurs in travel (wire off or bump missing): fault DRUM_SENSOR.
- The codes are Gray: adjacent sectors differ by one bit, so a slow contact can only show the neighbour.
- Bit arcs (contact closed inside):
  - A (b0): [-10, 36) and [76, 327)
  - B (b1): [4, 98)
  - C (b2): [44, 346]
- Code (C B A) to sector: 001 LAUNCH, 011 LN, 010 NEUTRAL, 110 NR, 111 REEL, 101 WRAPPING, 100 WRAPPED, 000 INVALID.
- Edge tolerance about 1.5 deg.

Why the brake is only "mid" while wrapping:
- Wrapping pulls about 0.5 m of line off the storage reel against its slip clutch, and the wrap train gears the ring
  torque up 6.4x onto the drum.
- At firm preload the wrap would need about 1.5 N m at the drum, which is over the N20 budget, so firm comes after the
  last wrap (326..337).
- cad-reel: the lever sees lift 0 (light), 2 mm (mid) and 4 mm (firm). Tell me the forces and the levels you want;
  the follower angle (200) and the base radius are parameters.

## 7. Stop plan (firmware: run until the edge, then run the extra angle by time)

| target | dir | watch edge | code change | extra |
|---|---|---|---|---|
| LAUNCH | -1 | 4 | 011 to 001 | 4 deg |
| NEUTRAL | +1 | 36 | 011 to 010 | 4 deg |
| NEUTRAL | -1 | 44 | 110 to 010 | 4 deg |
| REEL | +1 | 76 | 110 to 111 | 11 deg |
| REEL (unwrap) | -1 | 98 | 101 to 111 | 11 deg |
| WRAPPED | +1 | 327 | 101 to 100 | 11 deg |

Rate and timing:
- Calibrate the rate every move from the time between two known edges: 4 to 36 or 44 to 76 (32 deg), or 98 to 327
  (229 deg).
- Nominal 10.8 deg/s: N20 27 rpm at 12 V, about 20 rpm at 9 V, 20 percent load droop, 9:1.
- Every dwell is at least 12 deg wide, so a 30 percent rate error still stops inside it.
- Nominal times: LAUNCH to NEUTRAL 3.7 s, NEUTRAL to REEL 4.4 s, REEL to WRAPPED 23 s, full travel 33 s.
- Timeout = 2 x nominal + 2 s. Then raise DRUM_TIMEOUT, stop, and try NEUTRAL once.
- Never drive into the hard stops on purpose (the groove ends are the end stops).

Rules:
- Unknown position at boot: read the code. If it is a transition code, move toward NEUTRAL (LN: +, NR: -).
  If WRAPPING or WRAPPED, unwrap to REEL first, then go to NEUTRAL.
- Swap rule: wiper stopped (rpm 0) before any drum move.
- After moving into LAUNCH or REEL, check SW_DOG (jog the wiper if it is closed).
- Positive drum = N20 forward (DRV8833 #3 IN1 high).

## 8. Contacts (printed: M3 screw head touching an M3 nut, ESP32 pull-up, closed = LOW = bit 1)

| name (spec) | where | closes when |
|---|---|---|
| SW_DRUM_A (bit b0) | lever 0 on band A, s 94, drum angle position 0 (+X) | bump under the nub |
| SW_DRUM_B (bit b1) | lever 1, s 101 | bump under the nub |
| SW_DRUM_C (bit b2) | lever 2, s 108 | bump under the nub |
| SW_DOG (proposed, optional) | fork carrier, both sides of the yoke tab, wired in parallel | yoke deflected over 3 mm (dogs tip to tip, or a jammed shift) |

- Each lever is a PLA flexure 30 long, rooted at the switch block foot.
- Bump lift 3 mm at the nub. The M3 screw head sits 1.1 mm from the nut, so it closes at about half the bump height.
- Every nut wire goes to GND and every screw wire to its GPIO (fw-base assigns pins). Debounce 20 ms.

## 9. N20 torque check

- N20 stall estimate: 0.22 N m at 9 V.
  - Bench data: 0.18 A stall, 0.025 A free at 8 V.
  - Assumed: 15,000 rpm motor, 556:1, worm train efficiency 0.30.
- Drum capacity at 9:1 with 0.9 mesh efficiency: stall 1.6 N m, working (35 percent) 0.57 N m.
- Drum peak loads (N m):

| load | peak | basis |
|---|---|---|
| fork shift | 0.41 | 12.6 N fork at full block, groove slope 22.5 mm/rad, groove efficiency 0.7 |
| disengage | 0.37 | under 0.5 N m of train torque |
| brake ramps | 0.26 (first), 0.58 (second, alone) | second ramp is 11 deg |
| wrap | 0.60 | 1 N line at 36 mm plus 0.04 N m friction, geared up 6.4x |
| peak combined | 0.60 | 37 percent of the stall estimate |

- If the bench shows a weaker N20, go to a 10:100 drum gear (it changes only the drum and the cradle position).
- Supply: DRV8833 #3 behind an MP1584 buck at 9 V (DRV8833 VM max is 10.8 V).

## 10. Wrap train (to cad-reel)

- The segment gear turns the wrap pinion exactly 4 turns across drum 100..325.
- The lock cam holds the pinion at home below 100 and at 4 turns above 325 (lock play about +-3.5 deg at the pinion).
- Idler output gear: m1.5, 48T (pitch dia 72, tip r 37.5), face 8, at s 40..48. Axis at L (-46.83, 72), that is
  46.83 behind the drum axis and 72 below the main axis, parallel to the main axis.
- Joint `drum_idler`: its angle is the wrap pinion angle (it decreases as the drum turns forward). Gear the ring
  joint off it: ring = -48/ring_z x idler.
- Wraps = 4 x 48 / ring_z. A 48T m1.5 ring gives 4 wraps, centre 72.1 from the idler axis, at any angle round it
  that suits the capstan (s 40..48 plane).
- Ring torque budget: 0.08 N m at mid brake.
- cad-reel: tell me ring_z and where the ring centre must be. I can move the idler (`phi_idler`) and the output gear
  axially.

## 11. Frame interfaces (to cad-frame)

- Drum brackets A (s 7..14) and B (s 117..124), idler bracket (s 49..55), N20 cradle (s 22..37), switch block
  (s 88..114):
  - all stand on the deck (L y 165 = world z 0);
  - 2 x M3 per foot (A and B: x +-26 from the drum axis).
  - cad-frame: give me the deck insert pattern you prefer and I align the feet.
- Wiper adapter plate at s <= 0: the coupler starts at s 0.5.
- Launch sleeve outboard bearing and thrust face: frame. My spigot ends at s 160 (magnet cap to 164). The AS5600
  board faces the cap end at s 166, on the main axis.

## 12. Open items and measurements for Harry

- Wiper output shaft not measured: the coupler is fully parametric (`WIPER` shaft_d, shaft_flat, shaft_len).
- N20 output shaft is not in the caliper data. It is modelled d3 with a D flat; please measure the diameter and flat.
- N20 stall torque is an estimate: a bench stall test with a 50 mm lever and a kitchen scale would confirm it.
- AS5600 magnet: assumed 6 x 2.5 mm diametric (check what came with the board).
