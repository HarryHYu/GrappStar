# Reel system: interfaces and numbers (worker cad-reel)

Status: Phase 0 (before the spec). Every number here is a recommendation computed in `reel_calc.py`
(`python3 base/cad/parts/reel_calc.py` prints the table). Once `base/spec/params.py` defines `REEL`, the spec wins and
this file is updated to match. Questions or changes: write them in your REPORT.md, I read every branch.

## 1. What it is (Harry's design, kept)

- Storage reel + capstan + wrap cup, Harry's v7 line path: storage reel -> guide eye -> over the capstan -> groove on the
  wrap cup -> ring on the cup -> exit eye -> dart. Capstan dia 40 x 40 with the v7 base flange, v7 cup (ring 0.5 in front
  of the face and 1 mm outside the wall, U groove 1.2 x 1.0, 30 deg helix, 58 deg wrap), exit eye 31.5 past the ring,
  guide eye 1 mm behind the base flange plane and 2 mm above the cup top, eyes d 4.4 coned to 6.4 both faces.
- LAUNCH: cup at home (ring at the top), 0 wraps. The line runs guide -> straight over the cup -> ring -> exit eye and
  never touches the capstan. The storage reel spins on 2 x 608 and pays out against the slip clutch (light setting).
- REEL: wiper -> drum dog clutch -> main reel output gear (cad-drum) -> capstan gear. The capstan pulls; the storage reel
  is driven through an idler and the slip clutch (firm setting) and only takes up behind the capstan.
- WRAP: the drum's wrap output turns the cup gear; the cup orbits the capstan and lays the turns. Reverse unwraps.

## 2. Reel local frame R (all positions below)

- Origin: capstan axis on the FRONT face of the reel wall. +Z along the capstan axis toward the cup, +X forward (launch
  direction), +Y up. Right-handed, so +Z = world -Y: the cup faces the right side of the base looking forward. The frame
  worker places R in the world (one rigid transform, rotation about world Z only, or none).
- Capstan axis height above the deck top: `H_AXIS = 45` (default; reel flange r 32 + 13 clearance; the gears hang lower,
  see 5).

## 3. Envelope (R frame, mm)

| item | X | Y | Z |
|---|---|---|---|
| whole unit incl. gears | -120 .. +78 | -45 (deck) .. +35 | -45 .. +106 |
| reel wall (vertical plate, mine) | -120 .. +28 | -45 .. +32 | -16 .. 0 |
| capstan block (sprag housing) | -30 .. +30 | -45 .. +30 | 0 .. 18.5 |
| storage reel (flange dia 64) | -117 .. -53 | -32 .. +32 | 6 .. 30 |
| capstan drum (flange back face Z 19) | -23 .. +23 | -23 .. +23 | 19 .. 62 |
| wrap cup (rim Z 55, face Z 69, ring Z 69.5) | -28 .. +28 | -28 .. +28 | 55 .. 75 |
| guide eye centre | -45 | +26 | 18 |
| exit eye centre | +70 | +23.5 | 101 |
| gear zone behind the wall | -105 .. +27 | -45 .. +27 | -45 .. -16 |

Keep-out for other subsystems: the line sweep in front of the cup face (Z 69 .. 101, within r 30 of the capstan axis and
the cone to the exit eye) and the line from the exit eye to the dart tail (cad-launch: tell me where the tail sits in
LAUNCH and at the end of the stroke; I check the line against every part at every pitch).

## 4. Shafts, bearings, seats

| shaft | what | dia | supports | notes |
|---|---|---|---|---|
| C capstan tube | PPA-CF, carries capstan drum + capstan gear | 20.0 OD, 10.3 bore | CSK20PP (sprag #2) in the capstan block, Z 3 .. 17; PLA bushing in the wall, Z -16 .. 0 | integral key rib 5.8 x 2.6 into the sprag inner keyway; collar at Z 18.5 stops it against the sprag inner race |
| K cup shaft | PPA-CF, inside C, cup at the front, cup gear at the back | 10.0 | slides in C (0.3 clearance) | D flat at both ends (hidden), Harry's 4.5 neck + 5.6 head + C-clip in front of the cup |
| S reel shaft | PPA-CF (or 8 mm steel rod if Harry has one) | 8.0 at the 608s, 10.0 in the wall | PLA bushing in the wall, Z -16 .. 0 | reel on 2 x 608 on S; clutch disk keyed (D) and sliding on S; reel gear behind the wall |
| idler pin | PPA-CF stub in the wall rear face | 8.0 | | idler turns on it (0.3 clearance) |

- Bought: 1 x CSK20PP (sprag #2), 2 x 608 (storage reel). Nothing else. Printed: everything else.
- Sprag #2 = capstan BACKSTOP to the frame (coordinator preference, kept): outer ring keyed to the capstan block by an
  integral rib, inner ring keyed to C. The capstan only turns in the wind-in sense. Why it is the right assignment:
  it holds the tow load in every drum state (a fault to NEUTRAL under load is safe); it is the reaction for the slip
  clutch during payout (the clutch disk is geared to the capstan); it lets the firmware stop the wiper under load
  without relying on the worm. It also doubles as the front radial bearing of C (CSK = 6204-size ball bearing).
  The wind-in sense is fixed below; the payout sense never needs the capstan to turn.
- Sense: capstan wind-in = +Z rotation (counter-clockwise looking from the cup toward the wall). Cup WRAP = -Z
  rotation, cup UNWRAP = +Z. Storage reel take-up = +Z (line arrives on the reel top heading -X), payout = -Z.

## 5. Gears (meshes with cad-drum)

All gears behind the wall, spur, 20 deg, PPA-CF, centre distances + `fits.GEAR_CENTER_EXTRA` (0.10).

| gear | on | module | teeth | face | plane Z | centre (R frame X, Y) |
|---|---|---|---|---|---|---|
| main reel output (cad-drum) | main shaft | 2.0 | 40 (recommended) | 12 | -29 .. -17 | 64.1 from the capstan axis, any angle that clears my idler (below or forward is fine) |
| capstan gear Gc | C | 2.0 | 24 | 12 | -29 .. -17 | (0, 0) |
| idler Gi | idler pin | 2.0 | 24 | 12 | -29 .. -17 | (-47.1, -9.7) |
| reel gear Gr | S | 2.0 | 15 | 12 | -29 .. -17 | (-85, 0) |
| cup gear Gk | K | 1.5 | 30 | 8 | -43 .. -35 | (0, 0) |
| wrap pinion (cad-drum) | drum wrap output | 1.5 | yours | 8 | -43 .. -35 | 0.75 x (30 + z) + 0.1 from the capstan axis |

- main 40 : capstan 24 = capstan 1.667 x main. Wiper free 45 rpm -> capstan 75 rpm -> 0.16 m/s line. At wiper stall the
  line sees 373 N, below the 667 N break, so even a jammed firmware cannot snap the line through the capstan.
- Take-up: capstan 24 -> idler 24 -> reel 15: reel gear 1.6 x capstan. At the empty core that is 1.56 x the capstan
  line speed, so the clutch always slips and always takes up.
- If you need a different main gear, keep module 2, plane Z -29 .. -17 and tell me the tooth count; only the capstan
  speed changes.

## 6. Slip clutch and the brake lever (interface with the drum brake track)

- Face clutch on the reel's rear flange: annulus r 9 .. 17 on the reel against a disk D keyed (sliding) on S. A printed
  flexure spring at the front end of S presses the reel back onto D through the front 608. A non-rotating thrust ring T
  behind D is pushed forward by my brake rocker: pushing T forward squeezes the spring = firmer clutch.
- Settings (line tension at the mid pack radius, empty..full pack in brackets):
  - LAUNCH and NEUTRAL: 0.75 N (0.90 .. 0.64 N), clutch torque 0.018 N m, axial preload 4.6 N.
  - REEL and the whole wrap zone: 3.0 N (3.6 .. 2.6 N), 0.072 N m, preload 18.6 N.
  - Spring rate 3.5 N/mm, 1.3 mm precompressed at LAUNCH. An M3 screw on the spring seat trims both settings together
    (one turn = 0.5 mm = 1.7 N preload = about 0.07 N at the line in LAUNCH).
- BRAKE INPUT (what cad-drum drives): my pushrod, d 5, slides along +Z through the wall. Rear end = a flat contact face
  d 8 at R (-85, -38, -20) in LAUNCH. Push it +Z by 4.0 mm for REEL (track rise 4.0 mm, any slope up to 30 deg).
  Force at the pushrod: 5 N at LAUNCH (preload) rising to 21 N at REEL, plus the rocker friction: design your follower
  for 25 N. Fail safe: if the track lets go, the spring pushes the rod back to LAUNCH (light brake, safe for launch).
  If your track lever is somewhere else, tell me the point and direction and I move my pushrod to meet it.
- Brake track profile wanted vs drum position: LAUNCH 0, NEUTRAL 0, ramp, REEL 4.0, held at 4.0 through the whole
  wrap zone (tow happens at WRAPPED).

## 7. Wrap cup drive (interface with the drum wrap output)

- Cup turns: WRAP = 7 cup turns in the -Z sense from home; UNWRAP = 7 turns +Z back to home. Home = ring at the top
  (+Y), where the launch path is straight. WRAPPED must be an exact whole number of turns so the ring is back at the
  top (that is what makes the tow torque on the cup small, below).
- Cup torque you must supply: laying wraps 0.15 N m (line drawn through the ring and groove at up to 5 N).
- Cup torque you must HOLD during tow (the line drags the cup): Harry's v7 geometry nearly cancels it. Computed
  -0.11 .. -0.50 N m at 100 N tow (sign -: the WRAP sense), so design the hold for +/- 1.0 N m at the cup gear (both
  senses, margin for friction scatter), 3 N m short peak at wiper stall. If the drum train cannot hold 1 N m at the cup,
  say so and I add a cup detent.
- The cup gear plane Z -43 .. -35 is behind my capstan gear; your pinion shaft must clear Gc (tip r 26) and the main
  reel output gear.

## 8. Line, wraps, tensions (for physics and fw-base)

- Wraps 7 (Euler-Eytelwein, Dyneema on PLA): gain e^(mu 2 pi n) = 81 (mu 0.10), 733 (0.15), 60,000 (0.25). With the REEL
  clutch behind (2.6 N at a full reel) 7 wraps hold 208 N at mu 0.10 (2 x a 100 N tow limit) and 1,878 N at mu 0.15.
  6 wraps fail the margin at mu 0.10 (111 N), so 7.
- Wrapping lays 0.90 m of line (taken from the dart side, which must be slack: see the sequence).
- Storage reel: core dia 40, flange dia 64, 20 wide. 20 m of 1 mm line packs to dia 56.5 (3.75 mm freeboard).
- Launch payout (v 11 m/s, 0.30 m stroke): reel reaches 3,700 rpm (608 fine), line pull in the stroke 5.1 N (inertia
  1.7e-5 kg m2 estimate, refined from the CAD later), 1.3 J of spin. Least brake against overrun in flight: 0.11 N,
  so 0.75 N never overruns while the dart flies.
- At landing the dart stops dead and the spinning reel throws slack: 0.91 m at 0.75 N. Fix: a SLACK-LINE SHOE (printed
  flexure arm, no spring to buy): the line runs over an eye on the arm between the reel and the guide; any tension above
  about 0.3 N holds the shoe off the reel flange; when the line goes slack the flexure drops the shoe on the flange rim
  (about 6 N) and the reel stops within 0.11 m. It also stops the reel if anything else ever leaves the line slack.
- Tension vs wiper current (no load cell; linear motor model, eta 0.85, calibrate with a luggage scale):
  T = (I - 1.5) / 16.5 x 15 x 0.85 / 1.667 / 0.0205 N, i.e. about 22.6 N per amp above 1.5 A free.

| line tension | main torque | wiper amps | line speed |
|---|---|---|---|
| 5 N | 0.20 N m | 1.7 A | 0.159 m/s |
| 30 N | 1.21 N m | 2.8 A | 0.148 m/s |
| 50 N | 2.01 N m | 3.7 A | 0.139 m/s |
| 100 N | 4.02 N m | 5.9 A | 0.118 m/s |

## 9. Mission sequence for the reel (firmware, sim and audit)

1. LAUNCH: drum LAUNCH, cup home, clutch light. Fire. Reel pays out, slack-line shoe stops it at landing.
2. Dart LANDED (dart reports it; SW_SLACK closing is the base-side cue): drum LAUNCH -> NEUTRAL -> REEL (clutch firm).
   Do NOT reel yet: the line must stay slack on the dart side.
3. WRAP at once, before the dart anchors: drum through the wrap zone, cup 7 turns -Z (0.90 m drawn from the slack on
   the dart side; if the dart side runs out of slack the unanchored dart is tugged under 1 m toward the base, harmless).
   Drum stops at WRAPPED, ring at the top. SW_CUP_HOME closes once per turn: count 7.
4. Dart ANCHORING -> ANCHORED. The capstan is backstopped, so the dart cannot pull line out.
5. TOW: wiper forward, `reel` with max_n = the tow limit (physics worker; 50 to 100 N, below the anchor holding:
   dry sand 7 N, moist loam 45 N, firm clay 112 N). The capstan takes in the dart-side slack first, then winches.
6. Dart RELEASING -> released: `reel` with a low max_n (30 N) drags the dart home on the capstan until SEATED
   (cad-launch seat switch, or the current rise).
7. UNWRAP, interleaved so the seated dart is never pulled hard and never more than 64 mm of loose line: 14 steps of
   (cup +0.5 turn, then capstan wind-in 0.8 of the released 64 mm = 143 deg of capstan = 86 deg of main shaft). The
   storage reel takes up behind. Then capstan 1 more turn with 0 wraps: the clutch pulls the last slack in at 3 N.
8. Drum REEL -> NEUTRAL -> LAUNCH (clutch light, cup home). Ready.

## 10. Switches I need (printed contacts, M3 screw head on an M3 nut, internal pull-up)

| name | what closes it | where (R frame) | use |
|---|---|---|---|
| SW_CUP_HOME | a bump on the cup gear rim pushes a printed flex lever, closed for +/- 12 deg about home (ring top) | behind the wall at about (0, +30, -39) | wrap count, home check before LAUNCH |
| SW_SLACK | the slack-line shoe arm drops (line slack) | on the reel wall near (-60, +34, 10) | landing cue, slack detect during reel |

## 11. Frame mounting (cad-frame)

- My reel wall has a foot flange on the deck: 6 x M3 clearance holes (3.4) at R (-112, -45, -8), (-60, -45, -8),
  (+20, -45, -8), (-112, -45, +24), (-60, -45, +24) and (+22, -45, +40). Deck top = R Y -45. Use heat-set inserts or
  nuts in the deck. Final positions move with the spec; I keep the hole pattern in `reel.py` as `MOUNT_HOLES`.
- The exit eye post (X +70, Z 101) carries the full tow load over a PLA edge (Harry's note): it is PPA-CF and stands on
  its own foot with 2 x M3 at R (+62, -45, 101) and (+78, -45, 101).

## 12. Open items for others

- cad-drum: confirm main gear tooth count and position, wrap pinion teeth and position, brake track lever point.
- cad-launch: dart tail position in LAUNCH and at the end of the stroke; seat switch; the line from my exit eye to it.
- physics: allowed tow limit, launch speed (my overrun and spin-up numbers use 11 m/s).
- Harry to measure: CSK20PP keyway depths (inner assumed 2.8, outer 2.0), whether he has an 8 mm steel rod for S.
