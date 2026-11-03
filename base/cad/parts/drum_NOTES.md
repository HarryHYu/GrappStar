# Drum subsystem notes (worker cad-drum): main shaft, dog clutch, N20 selector drum

Status: phase 0 proposal (3 Nov 2026, before BASE_SPEC.md). Every number here lives in `drum_kin.py` (DEFAULTS:
timing, code, loads) and `drum.py` (GEOM: geometry). Both read overrides from `params.DRUM` once the spec sets it.
Units mm, deg, N, N m.

## 1. What it does

- One 12 V wiper motor turns the MAIN SHAFT (coaxial with the wiper output).
- Two output gears turn free on the main shaft: LAUNCH gear (outboard) and REEL gear (motor side).
- A DOG COLLAR, keyed to the main shaft (hex), slides between them. Its dogs lock it to one gear or neither.
- A SELECTOR DRUM, turned by one self-locking right-angle N20 through a 10:90 spur pair, moves the collar through a
  COMPLIANT FORK (groove 1), sets the storage reel slip clutch preload (track 2, cad-reel lever), and turns the WRAP
  ARM through a segment gear with a locking arc (only past REEL). The drum holds any angle unpowered (worm).
- Three printed contacts read a Gray code off the drum (absolute sector). A fourth contact on the fork reports dogs
  landed tip to tip. A hand lever on the drum end is the fallback (N20 cradle slides out of mesh first).
- Order is physical: no wraps outside REEL; the collar cannot leave REEL until the drum has turned back through the
  whole unwrap. Fault means drive the drum to NEUTRAL.

## 2. Frames and envelope (proposal: the coordinator sets LAYOUT)

- Main shaft frame `drum_main`: origin on the wiper output axis at the wiper gearbox boss face, axis along world +Y
  (motor on the -Y side, shaft points +Y). Coordinate `s` = distance along the shaft from the boss face.
- Drum axis: parallel to the main shaft, 72.0 below it by default (`drum_offset` = (0, -72) in the shaft frame XZ
  plane; the whole drum group can be rotated about the main shaft by `drum_phi` if the frame wants it behind).
- Envelope (default, drum below): x -95..+60, z (relative to main shaft axis) -124..+40, s -1..150.
  Main shaft axis height above the deck = 124 with the default brackets (frame: say if this must be lower; the drum
  can move behind the shaft instead, `drum_phi` = -180 puts it at the same height, 72 behind).

## 3. Main shaft stack (s, mm from the wiper boss face)

| s | part | notes |
|---|---|---|
| 0.5 .. 36 | coupler (PPA-CF), OD 40 | bore = wiper shaft (d 10, flats 8 across, length 20, M8 end thread, nut 13 AF x 6.5 in a pocket, all parametric `WIPER`); hex 22 AF socket s 22..36 for the main shaft; M3 cross screw through socket and shaft |
| 22 .. 116 | main shaft hex 22 AF (PPA-CF) | torsion at 15 N m stall: 7.5 MPa |
| 116 .. 146 | main shaft round d 20 | outboard bearing (frame) and coaxial rail pivot boss (launch/frame) |
| 36 .. 55 | REEL sleeve (PPA-CF), hex bore, journal OD 30, flange OD 44 at 36..40 | keyed, turns with the shaft; flange thrusts on the coupler face |
| 40.5 .. 54.5 | REEL output gear, dogs to 59.5 | turns free on the sleeve |
| 62 .. 88 | dog collar at NEUTRAL (dogs 62..67 and 83..88, fork groove 72..78, OD 60, groove root r 26) | moves -7 to REEL (s 55..81), +7 to LAUNCH (69..95) |
| 90.5 .. 109.5 | LAUNCH output gear, dogs 90.5..95.5 | turns free on the sleeve |
| 95 .. 114 | LAUNCH sleeve, flange 110..114 | |
| 114 .. 120 | shaft stop ring (M3 cross screw) | takes LAUNCH engage thrust |

Output gears (recommended, spec decides): module 3, 24 teeth, pitch dia 72, tip dia 78, face 14, PPA-CF, 20 deg.
Lewis bending at 15 N m stall: 30 MPa (PPA-CF fine). Both gears turn either direction; the launcher splits
directions with its own sprag and ratchet downstream. Train gears that mesh them: same module, axes parallel to Y.

## 4. Dog clutch

- 4 dogs per face, 30 deg wide at the root (60 deg gaps), height 5, ring r 18..26.
- Neutral tip gap 2.5 each side, stroke 7, engagement 4.5, 0.5 tip to root clearance when engaged.
- Drive faces back-cut 4 deg: torque pulls the dogs together, they cannot walk out under load. To shift OUT under a
  held train torque the firmware first unloads (jog the wiper 3 deg against the load) and then moves the drum.
- Tips roofed (1.2 mm chamfers): a tip to tip landing slides off or is held by the fork spring.
- Dog stress at 15 N m: 190 N per dog, 3.8 MPa bearing.
- Tip to tip: the fork yoke (semicircle, 2.8 thick, flat printed PLA) is a curved out-of-plane spring, about 1.6 N/mm,
  with hard stops at 5.5 mm. The drum finishes its move anyway; SW_DOG closes. Firmware jogs the wiper (+15 deg,
  -15 deg, up to 60 deg total, 30 percent duty) until SW_DOG opens (dogs dropped in).

## 5. Drum (drum frame d = s, same direction as the main shaft)

| s | feature |
|---|---|
| 1 .. 9 | wrap idler output gear (on the idler, not the drum) |
| 10 .. 17 | drum journal A (d 16) in bracket A; idler journal in bracket A |
| 18 .. 26 | segment gear: m 1.25, cut from 64 teeth, 40 teeth present (225 deg), r_p 40 |
| 26 .. 32 | locking arc layer (convex lock ring r 38, cut back where the pinion turns) |
| 33 .. 38 | idler tail bearing lug (bracket A) |
| 42 .. 56 | track 2 brake cam: base r 24, full lift 4 (r 28) |
| 60 .. 90 | groove 1 barrel, r 30, groove 6.2 wide x 4 deep, follower pin d 6 at s 75 (drum top) |
| 92 .. 112 | code bands b0 92..98, b1 99..105, b2 106..112: base r 24, bumps r 27 |
| 118 .. 124 | drum journal B (d 16) in bracket B |
| 126 .. 134 | drum gear 90 T m 1 (x -0.3), meshes N20 pinion 10 T m 1 (x +0.3), centre distance 50.1 |
| 134 .. 142 | hand lever hex boss 12 AF, pointer marks L N R W |

Angles around the drum axis (0 = +X, 90 = up toward the main shaft): fork follower pin 90; shift rail (d 10 PPA-CF,
clamped in brackets A and B) at (x +30, z +34) from the drum axis; wrap idler at 180, 46.83 from the drum axis;
brake follower contact (cad-reel lever) at 180, s 42..56; code switch block at 0 (+X), s 90..114; N20 at -30.

## 6. Drum angle table (firmware and audit)

| drum deg | name | collar (mm) | brake track | wraps | code b2 b1 b0 |
|---|---|---|---|---|---|
| -10 | hard stop (groove end) | -7 | 0 | 0 | 001 |
| 0 | LAUNCH (dwell -10..6) | -7 (LAUNCH engaged) | light (0) | 0 | 001 |
| 6..34 | shift L to N | harmonic | 0 | 0 | 011 from 4 |
| 40 | NEUTRAL (dwell 34..46) | 0 | 0 | 0 | 010 (36..44) |
| 46..74 | shift N to R | harmonic | ramps 0 to mid over 46..98 | 0 | 110 (44..76) |
| 80 | REEL (dwell 74..346) | +7 (REEL engaged) | about 0.36 | 0 | 111 (76..98) |
| 100..325 | wrap sector | +7 | mid (0.5) | 0 to 4, linear | 101 (98..327) |
| 326..337 | brake ramp mid to firm | +7 | ramps to 1 | 4 | 100 from 327 |
| 338 | WRAPPED (dwell 327..346) | +7 | firm (1.0) | 4 | 100 |
| 346 | hard stop (groove end) | +7 | 1 | 4 | 100 |

Code 000 never occurs in travel (wire off or bump missing): fault DRUM_SENSOR. Codes are Gray: adjacent sectors
differ by one bit, so a slow contact never shows a wrong sector, only the neighbour.

Bit arcs (closed inside): b0 [-10, 36) and [76, 327); b1 [4, 98); b2 [44, 346]. Code to sector: 001 LAUNCH,
011 LN, 010 NEUTRAL, 110 NR, 111 REEL, 101 WRAPPING, 100 WRAPPED, 000 INVALID. Contact tolerance about 1.5 deg.

Why the brake is only "mid" while wrapping: wrapping pulls about 0.5 m of line off the storage reel against its slip
clutch, and the wrap train gears the ring torque up 6.4x onto the drum. At firm preload the wrap would need about
1.5 N m at the drum (over the N20 budget). Firm comes after the last wrap (326..337). cad-reel: the lever sees lift
0 (light), 2 mm (mid), 4 mm (firm); tell me the forces and the levels you want.

## 7. Stop plan (firmware: run until the edge, then run the extra angle by time)

| target | dir | watch edge | code change | extra |
|---|---|---|---|---|
| LAUNCH | -1 | 4 | 011 to 001 | 4 deg |
| NEUTRAL | +1 | 36 | 011 to 010 | 4 deg |
| NEUTRAL | -1 | 44 | 110 to 010 | 4 deg |
| REEL | +1 | 76 | 110 to 111 | 4 deg |
| REEL (unwrap) | -1 | 98 | 101 to 111 | 18 deg |
| WRAPPED | +1 | 327 | 101 to 100 | 11 deg |

Rate: calibrate every move from the time between two known edges (4 to 36 or 44 to 76 = 32 deg; 98 to 327 = 229 deg).
Nominal 10.8 deg/s (N20 27 rpm at 12 V, about 20 rpm at 9 V, 20 percent load droop, 9:1). Every dwell is at least
12 deg wide, so a 30 percent rate error still stops inside the dwell. Nominal move times: LAUNCH to NEUTRAL 3.7 s,
NEUTRAL to REEL 3.7 s, REEL to WRAPPED 24 s, full travel 33 s. Timeout = 2 x nominal + 2 s, then DRUM_TIMEOUT, stop,
then try NEUTRAL once. Never drive into the hard stops on purpose (the groove ends are the end stops).
Unknown position at boot: read the code; if BETWEEN, move toward NEUTRAL using the code direction table above.
Swap rule: wiper stopped (rpm 0) before any drum move; after the move into LAUNCH or REEL, check SW_DOG.

## 8. Switches (printed contacts: M3 screw head touching an M3 nut, ESP32 pull-up, closed = LOW = bit 1)

| name | where | closes when |
|---|---|---|
| SW_D0 | code block, band b0 (s 95, drum angle position 0 = +X) | bump under lever |
| SW_D1 | code block, band b1 (s 102) | bump under lever |
| SW_D2 | code block, band b2 (s 109) | bump under lever |
| SW_DOG | fork carrier, either side of the yoke tab | yoke deflected over 3 mm (dogs tip to tip, or a jammed shift) |

Common: every nut wire to GND, every screw wire to its GPIO (fw-base assigns pins). Debounce 20 ms.

## 9. N20 torque check

- N20 stall estimate 0.22 N m at 9 V (bench: 0.18 A stall, 0.025 A free at 8 V; 15,000 rpm motor, 556:1, worm
  train efficiency 0.30). Drum capacity at 9:1 and 0.9 mesh: stall 1.6 N m, working (35 percent) 0.57 N m.
- Drum loads (peak, N m): fork shift 0.41 (12.6 N fork at full block, groove slope 22.5 mm/rad, groove efficiency
  0.7), disengage under 0.5 N m train torque 0.37, brake ramps 0.26 (first) and 0.58 (second, 11 deg, alone),
  wrap 0.60 (1 N line at 36 mm plus 0.04 N m friction, geared up 6.4x). Peak combined 0.60 N m: 37 percent of the
  stall estimate. If the bench shows a weaker N20, go to a 10:100 drum gear (one part).
- The N20 runs from DRV8833 #3 behind an MP1584 buck at 9 V (DRV8833 VM max 10.8 V).

## 10. Wrap train (to cad-reel)

- Segment gear on the drum (m 1.25, 40 of 64 teeth, starts at drum 100) drives the wrap pinion (10 T m 1.25, x +0.4)
  exactly 4 turns across the sector; the pinion is locked by the locking arc everywhere else (home at drum < 100,
  locked again at 4 turns past 325).
- The pinion shaft carries the IDLER OUTPUT GEAR: m 1.5, 48 T (pitch dia 72), at s 1..9, axis at drum angle 180,
  46.83 from the drum axis (that is x -46.83, z -72 from the main shaft).
- Wraps = 4 x idl_z / ring_z. With a 48 T m 1.5 ring gear: 4 wraps. Ring centre at 72.1 from the idler axis.
  cad-reel: give me ring_z and where the ring centre must be; I move the idler (angle and output gear axial position
  are parameters). Ring torque budget: 0.08 N m at mid brake.

## 11. Frame mounting (to cad-frame)

- Bracket A (s 10..17) and bracket B (s 118..124): feet on the deck plane (drum axis - 52), 2 x M3 each, holes at
  x +-30 from the drum axis. Switch block: 2 x M3 on the deck at x +48..+60, s 95 and 109.
- Wiper motor adapter plate (frame) at s = 0; the coupler needs 1 mm axial clearance from it.
- Main shaft outboard bearing (frame): bore 20.35 at s 126..140, thrust face at s 120.5.

## 12. Open items

- Wiper output shaft not measured: coupler fully parametric (`WIPER` shaft_d, shaft_flat, shaft_len, thread).
- N20 output shaft not in the caliper data: modelled d 3 with a D flat 2.5 (`n20_shaft_d`, `n20_shaft_flat`).
  Harry: measure the worm N20 output shaft diameter and flat.
- N20 stall torque is an estimate: a bench stall test with a lever and a kitchen scale would confirm it.
