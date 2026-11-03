# Reel system: interfaces and numbers (worker cad-reel)

Status: v2, 11:35 Sydney (adapted to BASE_SPEC v0.1, cad-drum NOTES and physics FINDINGS: m3 capstan gear
meshing the drum's m3 24T REEL output gear, 8 wraps from the drum's 4 idler turns, light brake 0.5 N, three brake
levels). Phase 0 numbers that changed are marked NEW. Every number here is a recommendation computed in `reel_calc.py`
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

## 5. Gears (meshes with cad-drum) NEW

All spur, 20 deg, PPA-CF, axes parallel to the main shaft. Centre distances from `gears.center_distance` (+0.10).

| gear | on | module | teeth | shift | face | plane (R frame Z) |
|---|---|---|---|---|---|---|
| REEL output (cad-drum) | main shaft | 3 | 24 | 0 | 14 | = capstan gear plane |
| capstan gear Gc | C | 3 | 12 | +0.3 | 14 | -43 .. -29 |
| take-up gear Gt | C | 2 | 24 | 0 | 10 | -27 .. -17 |
| take-up idler Gi | pin in my wall | 2 | 24 | 0 | 10 | -27 .. -17 |
| reel gear Gr | S | 2 | 15 | 0 | 10 | -27 .. -17 |
| cup ring gear Gk | K | 1.5 | 24 | 0 | 8 | -53 .. -45 |
| wrap idler output (cad-drum) | drum idler | 1.5 | 48 | 0 | 8 | = cup gear plane |

- Main 24T : capstan 12T = capstan 2.0 x main (physics F7 asked 2.5 to 3; 12T is the smallest m3 gear round the
  20 mm tube). Wiper free 45 rpm -> capstan 90 rpm -> 0.19 m/s. At wiper stall the line sees 311 N, under the 667 N
  break: a jammed firmware cannot snap the line through the capstan. 100 N tow = 4.8 N m at the main shaft, 6.8 A.
- Take-up: capstan 24 -> idler 24 -> reel 15: clutch disk 1.6 x capstan speed, 1.56 x the capstan line speed at the
  empty core, so the clutch always slips and always takes up.
- Cup ring 24T on the drum's 48T wrap idler: 4 idler turns = 8 cup turns = 8 WRAPS exactly (a whole number, so the
  ring ends at the top). Centre distance idler-ring 54.1. Capstan axis therefore sits 54.9 (m3 pair with the +0.3
  shift, see params when set) from the main shaft AND 54.1 from the wrap idler axis.
- R frame Z maps to world -Y (R +Z = cup end = world -Y). The gear plane order above (take-up next to the wall,
  capstan gear, then cup gear furthest back) keeps the take-up gear away from the dog collar (it sits at smaller s,
  beside the coupler r 20); the cup gear clears the collar (r 30) by 5 mm at 54.9. If LAYOUT needs another order, I
  move the planes (parameters `GT_Z`, `GC_Z`, `GK_Z`).
- WRAP sense: cup turns -Z in R = positive about world +Y (right hand). The ring gear turns the same sense as the
  drum for an even train (drum -> pinion -> idler shaft -> ring: two external meshes), so the drum must turn
  positive about world +Y while wrapping (drum angle increasing). cad-drum: confirm, else one more idler.

## 6. Slip clutch and the brake lever (interface with the drum brake track) NEW

- Face clutch: friction annulus r 13 .. 19 on the storage reel's FRONT face against the clutch disk D (the head of
  the reel shaft S, keyed to the reel gear). The reel is squeezed forward onto D by a thrust ring T on the rear 608
  inner race. T is pushed by my BRAKE ROCKER, a flexible PETG lever (3 x 10 section): pivot at the top, T contact 37
  below it, the push point 35 below that. With the clutch stack rigid, the rocker bends: it IS the clutch spring.
- Levels (line tension at mid pack, axial preload, push force at the rod):

| drum level | lift | line N | preload | push |
|---|---|---|---|---|
| LAUNCH, NEUTRAL (light) | 0 | 0.50 (0.60 empty .. 0.43 full) | 2.5 N | 1.3 N |
| wrapping (mid) | 2.0 mm | 1.75 | 8.8 N | 4.5 N |
| REEL, WRAPPED, tow (firm) | 4.0 mm | 3.0 (3.6 .. 2.6) | 15.1 N | 7.8 N |

- Light is set mechanically, not by the drum: the push rod has a collar that seats in a counterbore of my wall (the
  rocker's preload pushes it back there). If the track or the lever ever lets go, the brake stays LIGHT (safe to
  launch). Printed 0.4 mm shims under T trim the light level.
- BRAKE INPUT: cad-drum's track is radial (follower at drum angle 180, s 42..56, lift 0/2/4 mm outward). I provide the
  follower: a bell crank on my wall with a d 6 follower pin on the track and its other arm on my push rod, 1:1.
  Design force on the track 10 N (7.8 N + friction). The bell crank pivot point is set once LAYOUT places the
  reel against the drum; until then the push rod rear end at R (-85, -35, -24) is the contact.

## 7. Wrap cup drive (interface with the drum wrap output)

- Cup turns: WRAP = 8 cup turns in the -Z sense from home; UNWRAP = 8 turns +Z back to home. NEW Home = ring at the top
  (+Y), where the launch path is straight. WRAPPED must be an exact whole number of turns so the ring is back at the
  top (that is what makes the tow torque on the cup small, below).
- Cup torque you must supply: laying wraps 0.15 N m (line drawn through the ring and groove at up to 5 N).
- Cup torque you must HOLD during tow (the line drags the cup): Harry's v7 geometry nearly cancels it. Computed
  -0.11 .. -0.50 N m at 100 N tow (sign -: the WRAP sense), so design the hold for +/- 1.0 N m at the cup gear (both
  senses, margin for friction scatter), 3 N m short peak at wiper stall. Your locking arc holds the pinion at
  WRAPPED: with the 24T ring on the 48T idler that is 2 N m design (6 N m peak) at the idler shaft.
- The cup gear plane Z -43 .. -35 is behind my capstan gear; your pinion shaft must clear Gc (tip r 26) and the main
  reel output gear.

## 8. Line, wraps, tensions (for physics and fw-base)

- Wraps 8 NEW (Euler-Eytelwein, Dyneema on PLA): gain e^(mu 2 pi n) = 81 (mu 0.10), 733 (0.15), 60,000 (0.25). With the REEL
  clutch behind (2.6 N at a full reel) 7 wraps hold 208 N at mu 0.10 (2 x a 100 N tow limit) and 1,878 N at mu 0.15.
  6 wraps fail the margin at mu 0.10 (111 N), 7 is the least; 8 comes free from the drum train (390 N at mu 0.10).
- Wrapping lays 1.03 m of line (taken from the dart side, which must be slack: see the sequence).
- Storage reel: core dia 40, flange dia 64, 20 wide. 20 m of 1 mm line packs to dia 56.5 (3.75 mm freeboard).
- Launch payout (physics F1: 8.6 m/s, range 4 to 5 m): reel reaches 2,900 rpm (608 fine), line pull in the stroke
  3.1 N. Least brake against overrun in flight: 0.11 N, so 0.5 N never overruns while the dart flies.
- At landing the dart stops and the spinning reel throws slack: physics F5 says 0.24 m (realistic landing); my
  worst case (dead stop from 8 m/s at 0.5 N) is 1.4 m. Fix: a SLACK-LINE SHOE (printed
  flexure arm, no spring to buy): the line runs over an eye on the arm between the reel and the guide; any tension above
  about 0.3 N holds the shoe off the reel flange; when the line goes slack the flexure drops the shoe on the flange rim
  (about 6 N) and the reel stops within 0.11 m. It also stops the reel if anything else ever leaves the line slack.
- Tension vs wiper current (no load cell; linear motor model, eta 0.85, calibrate with a luggage scale):
  T = (I - 1.5) / 16.5 x 15 x 0.85 / 2.0 / 0.0205 N, i.e. about 18.8 N per amp above 1.5 A free. NEW

| line tension | main torque | wiper amps | line speed |
|---|---|---|---|
| 5 N | 0.24 N m | 1.8 A | 0.190 m/s |
| 30 N | 1.45 N m | 3.1 A | 0.175 m/s |
| 50 N | 2.41 N m | 4.2 A | 0.162 m/s |
| 100 N | 4.82 N m | 6.8 A | 0.131 m/s |

## 9. Mission sequence for the reel (firmware, sim and audit)

1. LAUNCH: drum LAUNCH, cup home, clutch light. Fire. Reel pays out, slack-line shoe stops it at landing.
2. Dart LANDED (dart reports it; SW_SLACK closing is the base-side cue): drum LAUNCH -> NEUTRAL -> REEL (clutch firm).
   Do NOT reel yet: the line must stay slack on the dart side.
3. WRAP at once, before the dart anchors: drum through the wrap zone, cup 8 turns -Z (1.03 m drawn from the slack on
   the dart side; if the dart side runs out of slack the unanchored dart is tugged under 1 m toward the base, harmless).
   Drum stops at WRAPPED, ring at the top. SW_WRAP_HOME closes once per cup turn: count 8.
4. Dart ANCHORING -> ANCHORED. The capstan is backstopped, so the dart cannot pull line out.
5. TOW: wiper forward, `reel` with max_n = the tow limit (physics worker; 50 to 100 N, below the anchor holding:
   dry sand 7 N, moist loam 45 N, firm clay 112 N). The capstan takes in the dart-side slack first, then winches.
6. Dart RELEASING -> released: `reel` with a low max_n (30 N) drags the dart home on the capstan until SEATED
   (cad-launch seat switch, or the current rise).
7. UNWRAP, interleaved so the seated dart is never pulled hard and never more than 64 mm of loose line: 16 steps of
   (cup +0.5 turn, then capstan wind-in 0.8 of the released 64 mm = 143 deg of capstan = 72 deg of main shaft). The
   storage reel takes up behind. Then capstan 1 more turn with 0 wraps: the clutch pulls the last slack in at 3 N.
8. Drum REEL -> NEUTRAL -> LAUNCH (clutch light, cup home). Ready.

## 10. Switches I need (printed contacts, M3 screw head on an M3 nut, internal pull-up)

| name | what closes it | where (R frame) | use |
|---|---|---|---|
| SW_WRAP_HOME (spec name) | a bump on the cup gear rim pushes a printed flex lever, closed for +/- 12 deg about home (ring top) | behind the wall at about (0, +30, -39) | wrap count, home check before LAUNCH |
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
