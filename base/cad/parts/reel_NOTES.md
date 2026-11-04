# Reel system: interfaces and numbers (worker cad-reel)

## D49 rail reel (params 1.4.0): supersedes the frame-mounted reel below

The reel now rides cad-launch's rail (study base/spec/proposals/D49_rail_reel_layout.md). reel.py builds:
- the reel HEAD on the rail rear flange (u -181.2): front plate, inboard spine (y -22..-16), P1 (capstan tube, u-shaft C,
  G_b axle, clutch lever post, spool shaft leg), P1b (tube rear bushing), P2 (cup shaft and W journals, SW_ARM), mid plate
  (spool shaft inboard bushing), spool shroud;
- Harry's v7 drum (unchanged) and cup (ring hole AXIAL at r 26, lug in front of the face) on the capstan axis
  (y 140, w 110.6), the free spool (d40/d62 x 20, two 608s) below the line at (u -224, w 57.6);
- the clutch: the spool's outboard face rubs the head disk of the spool shaft; the clutch lever (pivot parallel to the
  rail, upper arm under cad-drum's brake cam) presses a thrust ring on the inboard 608 through its FIRM screw; the lower
  arm is a PPA-CF leaf (4.4 thick): LIGHT 0.25 N fixed (LIGHT stop screw), FIRM 5 N (reel_calc.lever_leaf);
- the BRIDGE inboard of the main-axis stack (plate y 30..35, box arm to rail cheek A): countershaft C (sprag #2, C1 49T,
  bevel B1 2:1), u-shaft C to G_a 26T / G_b 32T / G_c 23T (capstan 2.307 x wiper); c1' 46T, swing idler 30T, T1 20T,
  bevels B2/B3 and u-shaft T to the spool shaft (2.5 x wiper). The plate outline avoids the frame-fixed SW_INDEX block.
Bevels are modelled as the large-end spur profile extruded to the cone (approximate; their mesh overlap is allowed).
Line: spool pack top -> eyelet straight on the dart axis (line_path / line_spans / line_exit); cad-launch continues from
line_exit. Checks: line_check, deck_check (47 deg deck plane), tether_check (whole assembly).
Open: the cup holds the groove exit torque alone (about 1.4 N m at 50 N, reel_calc.cup_torque) through the drum lock arc;
spool flange freeboard 2.75 mm at 20 m (bench-wind first); u-shaft C wind-up (steel rod fallback).


Status: v5, 13:15 Sydney, follows params v1.2 and DECISIONS D14, D15, D20, D22, D24, D25. Numbers come from `reel.py`
(geometry) and `reel_calc.py` (physics; `python3 base/cad/parts/reel_calc.py` prints the table). Requests: write
them in your REPORT.md or message the coordinator.

## 1. What it is (Harry's v7 reel, kept)

- Line path (v7): storage reel -> guide eye -> over the capstan -> groove on the wrap cup -> ring on the cup -> pulley 1
  (693ZZ; D43: replaces the exit eye, its groove line tangent to the ring-to-eye line at the old eye centre) ->
  along world +Y -> pulley 2 (693ZZ, on a swivel fork on its own post) -> in the y 119 plane round the bottom of
  cad-launch's rail pivot pulley (D14, D34) -> along the rail to the dart. Capstan dia 40 x 40 with the v7
  base flange; v7 cup (ring 0.5 in front of the face and 1 mm outside the wall, U groove 1.2 x 1.0, 30 deg helix,
  58 deg wrap, groove cut last); exit eye 31.5 past the ring; guide eye 1 mm behind the drum flange plane and 2 mm
  above the cup top. Eyes are plain d 4.4 holes coned to 6.4 (their axes are tilted to bisect the line's bend).
- LAUNCH: cup home (ring at the top), 0 wraps: the line runs guide -> straight over the cup -> ring -> pulley 1 and
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
- R is a pure translation of Harry's v7 world frame: every v7 line rule carries over unchanged.
- Senses about R +Z: capstan wind-in +, storage reel take-up +, bus gear under a FORWARD wiper +, cup WRAP -,
  cup UNWRAP +.

## 3. Envelope (R frame, rest state, from the model)

| item | X | Y | Z | world y |
|---|---|---|---|---|
| whole reel unit (without pulley 2's post) | -141 .. +90 | -95.4 .. +43 | -64 .. +123 | 97.4 .. -89.7 |
| reel wall (PLA, 6 mm, foot, guides, crank boss) | -134 .. +30 | -95.4 .. +43 | -40 .. +42 | |
| sprag hub (PPA-CF) | -33.5 .. +33.5 | -33.5 .. +33.5 | -6 .. +25 | |
| storage reel (flange d 80, D44) | -134 .. -54 | -40 .. +40 | 12 .. 36 | |
| shroud | -141 .. -60 | -47 .. +33 | 0 .. 41 | |
| capstan drum | -23 .. +23 | -23 .. +23 | 26 .. 69 | |
| wrap cup (rim Z 62, face 76, ring 76.5) | -24 .. +24 | -24 .. +28 | 62 .. 80 | |
| guide eye centre | -45 | +26 | 25 | 8.5 |
| old exit eye centre = pulley 1 tangent point (D43) | +70 | +23.5 | 107.2 | -73.7 |
| pulley 1 centre (axis R Y) | +72.9 | +23.5 | 100.8 | -67.3 (world x 54.3, z -46.1) |
| pulley 2 centre (axis world Z at swivel 0), on a swivel fork | +72.9 | +15.6 | -78.5 | 112 (world (54.3, 112, -54)) |
| pulley 2 post (foot 32 x 29, column at world x 138, y 91.9) | +70.9 .. +172.7 | -95.4 .. +29.6 | -72.8 .. -43.8 | 77.4 .. 106.4 |
| gears and SW_ARM behind the wall | -104.5 .. +36 | -75.7 .. +41 | -64 .. -6 | 39.5 .. 97.5 |
| brake bar, crank, guide caps, rocker | -113 .. +48.5 | -60 .. +40 | -3.5 .. +25 | 8.5 .. 37 |

- My checks() test every reel part against the main-axis stack (coupler r 27 y 0.5..46, bus r 41, REEL gear r 54,
  collar r 32, dog ring r 33, shaft r 15) and the drum (r 44): clear.
- Given the assembly (tools.audit passes it), checks() also runs tether_check: every span of the posed line
  (line_spans, then cad-launch's line_path spans) at every TIMELINE phase (start, end, at least every 1 s) and at 0
  and 7 wraps, as the d 1.0 tube, against every posed part of every subsystem except the parts that span is meant to
  touch (guide eye in the wall, capstan, cup ring, pulleys 1 and 2, exit post, storage reel, the pivot pulley, the
  rail pulleys, carriage, dart and cad-launch's own line model). It names each (span, part) that blocks the line.
- The line follows the cup as kin poses it (cad-drum's wrap idler), not S["wraps"]: mid-wrap the two are up to half
  a turn apart, and the line has to meet the posed parts.
- Keep out: the dart line sweep in front of the cup face (Z 76 .. 108, inside the cone from the ring circle to the
  exit eye) and the line run beyond pulley 1 (world x 72.3, z -46.1, from y -67.5 toward +Y).

## 4. Shafts and bearings

| shaft | material | dia | supports |
|---|---|---|---|
| C capstan tube | PPA-CF | 20 OD, 10.3 bore | CSK20PP in the sprag hub (Z 11 .. 25) + the hub bushing (Z -6 .. 11); key rib in the sprag and drum; collar (flat face on the sprag inner race, 45 deg cone into the drum mouth); D flat out the rear end for the capstan gear |
| K cup shaft | PPA-CF | 10 | inside C; D flats run out at the cup face (hidden under the clip) and under the cup gear; v7 neck + C-clip (dia 14) |
| S reel shaft | PPA-CF (8 mm steel rod also fits) | 8 | wall bushing + boss (12.7 long); 2 x 608 in the reel; head = clutch disk D r 31 |
| swing arm | PPA-CF | split C ring on the bus gear hub r 13 | stop pins on the wall rear face at the engaged and parked ends of its 12 deg swing |

Bought: CSK20PP, 2 x 608, 2 x 693ZZ (pulleys 1 and 2), M3: 7 deck screws, 4 x M3x8 hub, 3 x M3x8 shroud, 2 x M3x8 trim,
M3x25 + nut rocker pivot, pulley 1 axle M3x20 (tapped into its post), pulley 2 axle M3x20 + nyloc, 4 x M3 pulley 1
post, 4 x M3 pulley 2 post, 2 x M3x8 SW bracket, SW_ARM switch: M3x10 contact screw + 2 nuts, 2 x M3x10 self tap
(lever clamp), 4 x M3x8 self tap (base to bracket, into pilots through the plate), brake: M3x20 crank screw, M3x10 +
nut FIRM screw, 4 x M3x16 guide caps.

## 4b. Pulley 2 swivel (review r3_Q, D43)

- Over the aim range the line leaves pulley 2 toward cad-launch's pivot pulley 5 to 53 deg out of a fixed groove
  plane. Pulley 2 therefore sits in a swivel fork that turns about the incoming run from pulley 1 (axis along R -Z
  through the tangent point, R (79.9, 15.6)). The fork's tube (d 12, line bore d 6, coned mouth) turns in the post
  block's bore and is held by a C-clip above the block. The line tension pushes the fork shoulder up into the block.
- kin `reel_swivel` = swivel_deg(P, S): -3.8 deg at aim 6, -5.7 at 15, -18.3 at 36, -29.9 at 47 (the rail's top).
  The line stays within 2 deg of the groove plane (test). The pivot pulley centre is 6.5 up the rail NORMAL from
  launch.line_entry (its bottom), at the pitch launch.kin poses (cross-check cad_support-10: it was taken along the
  rail, 9.2 mm off, and at S aim_deg, which is up to 15 deg off while the rail lowers).
- The run from pulley 1 is 2.5 deg off the swivel axis (pulley 1 is 7.9 mm higher), 1.3 mm off the axis at the
  tube's top: inside the d 6 bore (1.2 mm clear).
- Pulley 2 at world z -54 (block 5.5 mm above the drum): the run to the pivot pulley clears cad-drum's d10 shift rail
  (x 25, z -58) by 0.88 mm at the 6 deg tow pitch (D45), 1.3 at 10, 2.1 at 15 (it was 0.13 at z -56). The line
  still leaves pulley 2 in the y 119 plane.

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
  cup = (42/24) (drum_idler - its value at drum 0), cup gear teeth +8.67 deg in the cup frame (zero overlap by
  numeric search). Without drum.py it falls back to S["wiper_deg"], S["reel_deg"], S["wraps"].
- Stall: at the 15 N m wiper stall the line sees 274 N (eta 0.85), under the 667 N break; the D26 shear pin (5 N m)
  caps it at about 100 N.
- Swing idler: forward turns drag the arm + into mesh against its stop (tooth forces are self-energising into the
  stop). Backward turns swing it 12 deg out (tips clear by about 2.2 mm). Payout back-drives the reel gear with the same
  tooth-force sense as take-up, so the idler stays IN during payout and the LIGHT clutch reacts on the self-locking
  wiper worm (no bus drag brake, D25). Gravity alone would keep the arm in (the idler sits above the capstan axis), so
  the arm ring is a split C bored 0.2 under the hub: its grip (target 15 to 30 N mm) swings it. Bench check: turn the
  wiper backward by hand, the arm must drop to its parked stop; if not, sand the ring bore.

## 6. Slip clutch and brake input (D24)

- Face clutch: annulus r 20 .. 30 (r_eff 25) on the storage reel's front flange disc against the clutch disk D (head
  of S). The thrust ring T presses the rear 608 inner race, so the reel onto D. T is pushed by two M3 TRIM SCREWS in
  my BRAKE ROCKER: PPA-CF plate 3.0 thick, printed flat, E about 5 GPa (Harry has no PETG); 15 wide, ring r 14.5
  round S; M3 pivot at the top, R Y 37; screws at Y 0; a stiff leg 10 x 5.5 runs down to Y -51.7, past the shroud
  through a window, and rests on a 0.45 mm pad on the wall at LIGHT. The clutch stack is rigid, so the plate bends:
  it is the FIRM spring. LIGHT is set by the trim screws (hex key through the reel gear and two holes in the wall from
  behind): 1/8 turn is about 0.4 N of axial preload.
- Brake input (review 2 and 3 rebuild; cad-drum's track 2 unchanged): the cam (base r 30, lift 4, follower at L angle
  180) pushes my BRAKE BAR (PPA-CF, 6 x 6) along R -X. Its round nose (r 3, R Z 9.2 .. 15.2 = world y 18.3 .. 24.3)
  sits 0.2 mm off the cam base circle at LIGHT, at R (48.66 - 0.2, -42.36). The bar runs in front of the wall,
  doglegs down at R X -30 .. -10 to R Y -55 (under the shroud and under the guide post, which ends at R Y -51 with a
  45 deg underside), in two U guides (wall jaw + printed cap, 2 x M3 x 16 each). A lug on its left end (up to R Z 25)
  turns a BELL CRANK (PPA-CF, axis R Y, on a d 7 boss at R (-107.6, Z 2), printed washer + M3 x 20 tapped 18 deep):
  arm A (tip circle 20 above the pivot) on the lug, arm B (14.1 along +X, in a 4 mm recess in the wall front) under
  the FIRM SCREW, an M3 x 10 through the rocker leg at R (-93.5, -48.7) with a lock nut on the leg's front.
  Bar 3.8 mm -> crank 11.0 deg -> arm B rises 2.75 mm; less the lost motion g at the screw (nominal 0.64) the leg
  moves 2.11 mm.
- Spring sizing with the real section (reel_calc.rocker_section, unit-load integration, E 5 GPa): 2.11 mm at the
  FIRM screw gives exactly LIGHT -> FIRM. FIRM scales with the real E of Harry's PPA-CF (4 .. 9 GPa printed): turning
  the FIRM screw sets g from 0 (E 4 GPa) to 1.6 (E 9 GPa). Set it with the luggage scale (reel_REPORT.md section 5, and the last build step).
- Levels (line tension with the reel full, D24):

| level | bar slide | line N (full .. empty) | torque | axial | leg force | bar force (cam) |
|---|---|---|---|---|---|---|
| LIGHT (LAUNCH, WRAP) | 0 (0.2 gap at the nose) | 0.50 .. 0.63 | 15.7 N mm | 2.1 N | 0.9 N (on the wall pad) | 0 |
| FIRM (NEUTRAL, WRAPPED, REEL) | 3.8 mm | 5.0 .. 6.3 | 157 N mm | 20.9 N | 9.0 N | 6.5 N + friction, about 9 N (design 15) |

- The bar follows the nose's envelope on the real cam (kin bar_mm): on the ramps the round nose touches higher up the
  ramp than the follower line, so the bar leads the lift by up to 1.5 mm there. Note for cad-drum: past the LAUNCH
  stop (drum -4 .. -2) the nose already sees the gap blend toward FIRM (bar 2.5 mm at -4).
- Fail-safe: if the track or follower lets go, the rocker spring pushes the leg, crank and bar back: LIGHT.
- Peak plate stress at FIRM about 17 MPa beside the S hole (PPA-CF, about 15 percent of its strength); creep would lower FIRM first, which the firmware
  sees as a softer tidy, never as a dry LAUNCH brake because LIGHT is set by the screws.

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
- Storage reel: core d 50, flange d 80 (D44: a mounded 20 m pack reaches r 35 .. 36; reel axis moved to R X -94), 22 wide, two-piece (flat front flange disc on three lugs). 20 m packs to d 62.8
  (8.6 mm freeboard on the d 80 flange).
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
  notch (+-12 deg at the gear face, SW_BUMP_DEG, 45 deg ramps). With the lever tip's width it OPENS from about -6 to
  +8 deg of cup turn (lever push 1.75 mm on the track, 0.04 at home, pretravel 1.28). Firmware: home = SW_ARM high,
  one open pulse per cup turn (7 per wrap). Setting (step "SW_ARM contact"): cup at home, lever free in the notch,
  contact screw head onto the nut and back 1.9 turns (gap 0.95 mm); check closed 15 deg either way, open at home.
- The switch base screws to the bracket with 4 x M3x8 self tap into d2.6 pilots through the 4 mm plate (the tips
  stand 1 mm out of its back; nothing lies within 5 mm there).

## 11. Frame mounting

- Reel wall foot: under the storage reel (stops at world x -76, left of cad-frame's cheek r) plus a strip behind the
  wall (stops at R X 3, left of cad-drum's idler bracket). 7 x M3 counterbored into deck inserts at R (X, Z)
  (-120, 14), (-120, 36), (-64, 36), (-64, 14), (-120, -32), (-60, -32), (-8, -32). The guide post hangs from the
  wall 8 mm above the deck (the cheek's foot flange is under it).
- Pulley 1 post: own foot on the deck, 4 x M3 at R (X, Z) (53, 89.2), (53, 125.2), (87, 89.2), (87, 125.2).
- Pulley 2 post: own foot on the deck (32 x 29, gussets fused to the column), column at world x 138, y 91.9, 4 x M3
  at +-11.5 in x and +-9.5 in y about it: world (126.5, 82.4), (126.5, 101.4), (149.5, 82.4), (149.5, 101.4), 2.8 mm
  or more of solid round each (they broke out of the old 17.7 mm foot, so the deck had no inserts there).
- deck_holes(P) lists all 15 reel deck screws in world (x, y) (wall 7, pulley 1 post 4, pulley 2 post 4);
  _frame_others.deck_contacts reads it, so cad-frame cuts an insert under each whatever its slice finds.
- cad-frame: the motor adapter plate must clear my sprag hub and drum: a notch r 30.5 about world (x -18.66, z -69.64) for y 0..24 (or its lower edge raised to z -38). Since cad-frame's 194 mm plate (y -16..0): a through hole r 30.5 about the same centre, plus a notch at world x 56..67, z -60..-43 for the line run from pulley 1 down to pulley 2 (it crosses the plate at world x 61.3, z about -49).

## 12. Open items

- cad-frame: notch the motor adapter plate round my capstan (section 11).
- cad-frame (cross-check cad_support-1): the tether check finds the motor adapter plate across the line twice: the
  guide eye -> cup ring run (0 wraps: load to approach) near world (-51, -5.5, -44), and the pulley 1 -> pulley 2
  run (every phase) near world (61.3, -5, -49). Carve both from reel.line_path(P, dict(wraps=0.0)) and
  dict(wraps=7.0) (pulley 2 is now at world z -54, so the second run sits about 0.7 mm higher at the plate).
- cad-launch: launch.line_path draws the pusher -> dart tail span straight to the landed dart. In rail_low (t 11 to
  12, rail 36 to 47 deg) that straight line dips 38 to 49 deg below the rail axis and the tether check finds it in
  launch_rail_front (near world (103, 140, 106)) and launch_big_gear (near (59, 140, 107)). The real line runs
  forward over the lip roller (launch_muzzle_roller) first: route the span over MZ.roller_top when the dart lies
  below the line ahead of the muzzle, and name the roller in that span's allowed parts.
- cad-drum: drum.kin's drum_main / drum_out_reel should follow S["wiper_deg"] / S["reel_deg"] (params 1.2.3).
- coordinator: WRAP sector FIRM for the unwrap (section 9).
- Harry measures: CSK20PP keyway depths (inner 2.8, outer 1.5 assumed), mu of Dyneema on the printed capstan.
