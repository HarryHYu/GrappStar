# Reel system: interfaces and numbers (worker cad-reel)

Status: v5, 13:15 Sydney, follows params v1.2 and DECISIONS D14, D15, D20, D22, D24, D25. Numbers come from `reel.py`
(geometry) and `reel_calc.py` (physics; `python3 base/cad/parts/reel_calc.py` prints the table). Requests: write
them in your REPORT.md or message the coordinator.

## 1. What it is (Harry's v7 reel, kept)

- Line path (v7): storage reel -> guide eye -> over the capstan -> groove on the wrap cup -> ring on the cup -> exit eye
  -> pulley 1 (693ZZ) -> along world +Y -> pulley 2 (693ZZ, own post) -> in the y 119 plane round the bottom of
  cad-launch's rail pivot pulley (D14, D34) -> along the rail to the dart. Capstan dia 40 x 40 with the v7
  base flange; v7 cup (ring 0.5 in front of the face and 1 mm outside the wall, U groove 1.2 x 1.0, 30 deg helix,
  58 deg wrap, groove cut last); exit eye 31.5 past the ring; guide eye 1 mm behind the drum flange plane and 2 mm
  above the cup top. Eyes are plain d 4.4 holes coned to 6.4 (their axes are tilted to bisect the line's bend).
- LAUNCH: cup home (ring at the top), 0 wraps: the line runs guide -> straight over the cup -> ring -> exit eye and
  never touches the capstan. The storage reel spins on 2 x 608 against the slip clutch (LIGHT).
- REEL: wiper -> dog -> REEL output gear (cad-drum, m2 50T) -> capstan gear m2 22T: capstan 2.27 x wiper (D20).
- TAKE-UP BUS (D15, D25): cad-drum's bus gear m1.5 50T (y 40..48, upstream of the dog, always turning) -> my bus gear
  c1 m1.5 46T (free on the capstan tube, centre 72.1) -> forward-only SWING IDLER m1.5 30T (arm rides c1's hub) ->
  reel gear m1.5 20T on the reel shaft S -> face slip clutch -> storage reel. Clutch disk = 50 / 20 = 2.5 x wiper
  (1.1 x the capstan in REEL, 1.34 x its line speed at the empty core).
- WRAP: cad-drum's wrap idler (42T m1.5, 4 pinion turns) turns my cup gear (m1.5 24T on the cup shaft K): the cup
  orbits the capstan and lays exactly 7 wraps, ring back at the top.
- Sprag #2 (CSK20PP) = capstan BACKSTOP in my PPA-CF sprag hub: the capstan only turns in the wind-in sense.

## 2. Reel frame R and placement

- params.LAYOUT.reel_frame_R: origin world (-18.661, 33.5, -69.643) = capstan axis on the front face of my reel wall;
  R +X = world +X, R +Y = world +Z, R +Z = world -Y. Deck top = R Y -95.36.
- R = Harry's v7 world frame shifted by (0, -44, +46): every v7 line rule carries over unchanged.
- Senses about R +Z: capstan wind-in +, storage reel take-up +, bus gear under a FORWARD wiper +, cup WRAP -,
  cup UNWRAP +.

## 3. Envelope (R frame, rest state, from the model)

| item | X | Y | Z | world y |
|---|---|---|---|---|
| whole reel unit | -136.5 .. +93 | -95.4 .. +43 | -46.5 .. +124 | 80 .. -90.5 |
| reel wall (PLA, 6 mm, foot and exit pad) | -128 .. +90 | -95.4 .. +43 | -40 .. +124 | |
| sprag hub (PPA-CF) | -33.5 .. +33.5 | -33.5 .. +33.5 | -6 .. +25 | |
| storage reel (flange d 72) | -124 .. -52 | -36 .. +36 | 12 .. 37.5 | |
| shroud | -131 .. -57 | -43.5 .. +30 | 0 .. 42 | |
| capstan drum | -23 .. +23 | -23 .. +23 | 26 .. 69 | |
| wrap cup (rim Z 62, face 76, ring 76.5) | -24 .. +24 | -24 .. +28 | 62 .. 80 | |
| guide eye centre | -45 | +26 | 25 | 8.5 |
| exit eye centre | +70 | +23.5 | 108 | -74.5 |
| pulley 1 centre (axis R Y) | +84 | +23.5 | 101 | -67.5 |
| pulley 2 centre (axis world Z), own post | +84 | +9.6 | -78.5 | 112 (world (65.3, 112, -60)) |
| pulley 2 post | +140 .. +173 | -95.4 .. +34.6 | -93 .. -64 | 97 .. 126 |
| gears behind the wall | -105 .. +36 | -36 .. +36 | -46.5 .. -6 | 40 .. 80 |

- My checks() test every reel part against the main-axis stack (coupler r 27 y 0.5..46, bus r 41, REEL gear r 54,
  collar r 32, dog ring r 33, shaft r 15) and the drum (r 44): clear.
- Keep out: the dart line sweep in front of the cup face (Z 76 .. 108, inside the cone from the ring circle to the
  exit eye) and the line run beyond pulley 1 (world x 72.3, z -46.1, from y -67.5 toward +Y).

## 4. Shafts and bearings

| shaft | material | dia | supports |
|---|---|---|---|
| C capstan tube | PPA-CF | 20 OD, 10.3 bore | CSK20PP in the sprag hub (Z 11 .. 25) + the hub bushing (Z -6 .. 11); key rib in the sprag and drum; collar (flat face on the sprag inner race, 45 deg cone into the drum mouth); D flat out the rear end for the capstan gear |
| K cup shaft | PPA-CF | 10 | inside C; D flats run out at the cup face (hidden under the clip) and under the cup gear; v7 neck + C-clip (dia 14) |
| S reel shaft | PPA-CF (8 mm steel rod also fits) | 8 | wall bushing + boss (9.5 long); 2 x 608 in the reel; head = clutch disk D r 31 |
| swing arm | PPA-CF | split C ring on the bus gear hub r 13 | stop pins on the wall rear face at the engaged and parked ends of its 12 deg swing |

Bought: CSK20PP, 2 x 608, 693ZZ (pulley 1), M3: 8 deck screws, 4 x M3x8 hub, 3 x M3x8 shroud, 2 x M3x8 trim,
M3x25 + nut rocker pivot, M3x16 + nyloc pulley axle, 4 x M3 exit post.

## 5. Gears (spur 20 deg PPA-CF, axes parallel to the main shaft)

| gear | on | m | teeth | face | plane R Z (world y) | meshes |
|---|---|---|---|---|---|---|
| bus gear c1 | free on C | 1.5 | 46 | 6 | -12.5 .. -6.5 (40..46) | cad-drum bus 50T (72.1), swing idler |
| swing idler | arm pin | 1.5 | 30 | 6 | -12.5 .. -6.5 | c1 always; reel gear on FORWARD turns |
| reel gear | S | 1.5 | 20 | 6 | -12.5 .. -6.5 | swing idler |
| capstan gear | C (D flat) | 2 | 22 | 12 | -29 .. -17 (50.5..62.5) | cad-drum REEL output 50T (72.1) |
| cup gear | K (D flat) | 1.5 | 24 | 8 | -43 .. -35 (68.5..76.5) | cad-drum wrap idler 42T |

- Phase convention: tooth 0 on +X of each gear's own frame at joint angle 0. Cross-owner meshes: my kin reads
  cad-drum's drum.kin and sets c1 = (50/46) drum_main + 3.78 deg, capstan = (50/22) drum_out_reel + 7.91 deg,
  cup = (42/24) (drum_idler - its value at drum 0), cup gear teeth +14.92 deg in the cup frame (zero overlap by
  numeric search). Without drum.py it falls back to S["wiper_deg"], S["reel_deg"], S["wraps"].
- Stall: at the 15 N m wiper stall the line sees 274 N (eta 0.85), under the 667 N break; the D26 shear pin (5 N m)
  caps it at about 100 N.
- Swing idler: forward turns drag the arm + into mesh against its stop (tooth forces are self-energising into the
  stop). Backward turns swing it 12 deg out (tips clear by about 7 mm). Payout back-drives the reel gear with the same
  tooth-force sense as take-up, so the idler stays IN during payout and the LIGHT clutch reacts on the self-locking
  wiper worm (no bus drag brake, D25). Gravity alone would keep the arm in (the idler sits above the capstan axis), so
  the arm ring is a split C bored 0.2 under the hub: its grip (target 15 to 30 N mm) swings it. Bench check: turn the
  wiper backward by hand, the arm must drop to its parked stop; if not, sand the ring bore.

## 6. Slip clutch and brake input (D24)

- Face clutch: annulus r 20 .. 30 (r_eff 25) on the storage reel's front flange disc against the clutch disk D (head
  of S). The thrust ring T presses the rear 608 inner race, so the reel onto D. T is pushed by two M3 TRIM SCREWS in
  my BRAKE ROCKER (PETG plate 3 x 15, pivot M3 at the top, screws 37 below the pivot, push point 33 below that). The
  clutch stack is rigid, so the rocker bends: it is the FIRM spring. LIGHT is set by the trim screws (hex key through
  two holes in the wall from behind): 1/8 turn is about 0.4 N of axial preload.
- Levels (line tension with the reel full, D24):

| level | rod stroke | line N (full .. empty) | torque | axial | push force |
|---|---|---|---|---|---|
| LIGHT (LAUNCH, WRAP) | 0 (rod on its collar seat) | 0.50 .. 0.63 | 15.7 N mm | 2.1 N | about 1 N (rocker preload) |
| FIRM (NEUTRAL, WRAPPED, REEL) | +4.0 mm | 5.0 .. 6.3 | 157 N mm | 20.9 N | 11.1 N (design 15 N) |

- Fail-safe: if the track or follower lets go, the rocker pushes the rod back onto its collar seat: LIGHT.
- Rocker bend at FIRM: 4 mm at the push point; the reel sits 8.5 mm in front of the rocker. Peak stress 17.5 MPa
  (PETG); creep would lower FIRM first, which the firmware sees as a softer tidy, never as a dry LAUNCH brake because
  LIGHT is set by the screws.
- BRAKE INPUT (cad-drum's track 2): push rod d 5.6, rear end face at R (-88, -33, -24), push along R +Z 4.0 mm.

## 7. Wrap cup

- 7 cup turns -Z from home = 7 wraps; the drum's locking arc holds the cup at home and at WRAPPED.
- Tow holding torque on the cup (independent review model: tension rises through the groove in active tow, lever 17.8
  .. 20.5): reel_calc gives -0.26 .. +0.54 N m at 100 N; the reviewer's wider model -0.92 .. +0.95. Design +-1 N m
  at the cup gear, both senses (50 N default tow limit: half that).

## 8. Line, wraps, tensions

- 7 wraps: e^(mu 2 pi 7) = 81 (mu 0.10), 733 (0.15). With the FIRM tail (5.0 N at a full reel) they hold 407 N at mu
  0.10 and 3,666 N at mu 0.15: 4x a 100 N tow even at mu 0.10. Measure mu on the printed capstan (hanging weight over
  one wrap) when built.
- Wrapping lays 0.90 m of line from the slack dart side (D22).
- Storage reel: core d 50, flange d 72, 22 wide, two-piece (flat front flange disc on three lugs). 20 m packs to d 62.8
  (4.6 mm freeboard).
- Launch payout: reel about 2,600 rpm, line pull in the stroke about 3 N, least brake against overrun in flight 0.09 N.
  Landing overrun: physics F5 about 0.24 m; worst case (dead stop at 8 m/s, 0.5 N) 1.15 m, which the shroud keeps on
  the reel.
- Tension from current (D32, physics): line N = 17 x (I - I_free) - 6. Zero against the REEL-mode current with a slack
  line; calibrate with a luggage scale.

## 9. Reel sequence (D22)

1. LAUNCH (LIGHT, cup home). Fire: payout against LIGHT.
2. LANDED -> NEUTRAL -> WRAP at once (LIGHT), dart side still slack: 7 turns, the line comes from the dart side.
3. WRAPPED (cup locked, FIRM). Dart anchors.
4. TIDY: wiper forward in NEUTRAL; the FIRM take-up pulls the dart-side slack through the capstan, which turns freely
   in its wind-in sense.
5. REEL fly-in, TOW with max_n (50 N default).
6. Release, reel-in on the capstan, unload.
7. UNWRAP in NEUTRAL with the wiper forward: freed line is pulled through the remaining turns in the capstan's free
   sense. Physics review: needs FIRM, not LIGHT (LIGHT tail 10 N mm is about the overrun drag of sprag + gears, and
   loops could form at the cup). params DRUM zones make the WRAP sector LIGHT: please make it FIRM while unwrapping
   (or the whole sector FIRM; D22 wraps from a slack dart side, so the wrap does not care).
8. Approach on the bus (FIRM) until SW_SEATED; LAUNCH (LIGHT).

## 10. Contacts

- SW_ARM (GPIO 42, mine): printed contact (cad.lib.switch, strain 0.79 percent) on a bracket behind the wall. Its
  lever rides a raised track on the cup gear's rear face: CLOSED (LOW) everywhere except home, where it drops into a
  notch and OPENS for about +-5 deg. Firmware: home = SW_ARM high, one open pulse per cup turn (7 per wrap).

## 11. Frame mounting

- Reel wall foot: under the storage reel (stops at world x -76, left of cad-frame's cheek r) plus a strip behind the
  wall (stops at R X 3, left of cad-drum's idler bracket). 7 x M3 counterbored into deck inserts at R (X, Z)
  (-120, 14), (-120, 36), (-64, 36), (-64, 14), (-120, -32), (-60, -32), (-8, -32). The guide post hangs from the
  wall 8 mm above the deck (the cheek's foot flange is under it).
- Exit post: own foot on the deck, 4 x M3 at R (58, 99), (58, 117), (82, 99), (82, 117).
- Pulley 2 post: own foot on the deck at world x 138, y 112, 4 x M3 (+-11.5 in x, +-9.5 in y about the column).
- cad-frame: the motor adapter plate must clear my sprag hub and drum: a notch r 36 about world (x -18.66, z -69.64).

## 12. Open items

- cad-frame: notch the motor adapter plate round my capstan (section 11).
- cad-drum: drum.kin's drum_main / drum_out_reel should follow S["wiper_deg"] / S["reel_deg"] (params 1.2.3).
- coordinator: WRAP sector FIRM for the unwrap (section 9).
- Harry measures: CSK20PP keyway depths (inner 2.8, outer 1.5 assumed), mu of Dyneema on the printed capstan.
