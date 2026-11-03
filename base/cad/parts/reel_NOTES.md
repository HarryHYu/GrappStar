# Reel system: interfaces and numbers (worker cad-reel)

Status: v3, 11:55 Sydney, follows DECISIONS D3, D4, D7, D15, D16 (base/spec/DECISIONS.md). Numbers come from
`reel.py` (geometry) and `reel_calc.py` (physics; `python3 base/cad/parts/reel_calc.py` prints the table). When
params.py v1.0 sets LAYOUT and REEL, the spec wins and this file follows. Requests: write them in your REPORT.md.

## 1. What it is (Harry's v7 reel, kept)

- Line path (v7): storage reel -> guide eye -> over the capstan -> groove on the wrap cup -> ring on the cup -> exit
  eye -> launcher pulleys (D14) -> dart. Capstan dia 40 x 40 with the v7 base flange; v7 cup (ring 0.5 in front of
  the face, 1 mm outside the wall, U groove 1.2 x 1.0, 30 deg helix, 58 deg wrap, groove cut last); exit eye 31.5
  past the ring; guide eye 1 mm behind the drum flange plane, 2 mm above the cup top. Eyes: plain d 4.4 coned to 6.4.
- LAUNCH: cup home (ring at the top), 0 wraps. The line goes guide -> straight over the cup -> ring -> exit eye and
  never touches the capstan. The storage reel spins on 2 x 608 against the slip clutch (LIGHT).
- REEL: wiper -> dog -> REEL output gear (cad-drum, m2 40T) -> capstan gear (m2 24T). Capstan = 1.667 x wiper.
- TAKE-UP BUS (D15): wiper -> cad-drum's BUS gear (m2 40T, upstream of the dog, always turning) -> my bus gear c1
  (m2 24T, free on the capstan tube) -> c2 (m2 24T, keyed to c1 by a hex) -> forward-only SWING IDLER (m2 24T on an
  arm pivoted on the capstan axis) -> reel gear (m2 16T) on the reel shaft S -> face slip clutch -> storage reel.
  Clutch disk = 40/24 x 24/16 = 2.5 x wiper (1.5 x the capstan in REEL, 1.46 x its line speed at the empty core).
  Cocking: 245 mm of line per wiper turn needed, the bus takes up 314 .. 440 mm per wiper turn.
- WRAP: cad-drum's wrap idler turns my cup ring gear (m1.5 24T on the cup shaft K); the cup orbits the capstan and
  lays 7 wraps.
- Sprag #2 (CSK20PP) = capstan BACKSTOP in the reel wall (D/spec): the capstan only turns in the wind-in sense.

## 2. Reel frame R

- Origin on the capstan axis at the FRONT face of my reel wall. +Z along the capstan axis toward the cup, +X forward
  (launch direction), +Y up. R +Z = world -Y (joint `reel_base`, rotation Rx(90), placed by P.LAYOUT["reel"] =
  dict(origin, rot_z_deg) when the spec sets it; default origin world (-40, 140, 45)).
- R = Harry's v7 world frame shifted by (0, -44, +42): every v7 rule carries over unchanged.
- Senses about R +Z: capstan wind-in +, storage reel take-up +, bus pair under a FORWARD wiper +, cup WRAP -,
  cup UNWRAP +. Wiper FORWARD turns the main shaft - about R +Z (= + about world +Y).

## 3. Envelope (R frame, rest state, from the model)

| item | X | Y | Z |
|---|---|---|---|
| whole reel unit | -122 .. +87 | -45 (deck top) .. +44 | -78.5 .. +116 |
| reel wall + foot + exit pad (one PLA part) | -122 .. +87 | -45 .. +44 | -21 .. +116 |
| storage reel (flange d 64) | -117 .. -53 | -32 .. +32 | 6 .. 35.3 |
| capstan drum | -23 .. +23 | -23 .. +23 | 22 .. 65 |
| wrap cup (rim Z 58, face 72, ring 72.5) | -24 .. +24 | -24 .. +28 | 58 .. 76 |
| guide eye centre (axis tilted 25 deg toward +Z) | -45 | +26 | 21 |
| exit eye centre (axis bisects the dart sweep and +X) | +70 | +23.5 | 104 |
| gears behind the wall | -105 .. +26 | -26 .. +38 | -78.5 .. -16 |

Keep out: the dart line sweep in front of the cup face (Z 72 .. 104, cone from the ring circle r 24.5 to the exit eye)
and the line beyond the exit eye along +X (cad-launch: your first pulley should sit on the +X line from the exit eye,
R (70 + d, 23.5, 104)).

## 4. Shafts and bearings

| shaft | material | dia | supports |
|---|---|---|---|
| C capstan tube | PPA-CF | 20 OD, 10.3 bore | CSK20PP in the wall's capstan block (Z 7 .. 21) + wall bushing/boss (Z -20.6 .. 0); integral key rib (sprag inner keyway and drum); collar on the sprag inner race; D flat for the capstan gear only |
| K cup shaft | PPA-CF | 10 | slides inside C; D flats under the cup's rear half and the cup gear; v7 neck + head + C-clip in front of the cup |
| S reel shaft | PPA-CF (8 mm steel rod also fits) | 8 | wall bushing; 2 x 608 inside the storage reel; head = clutch disk D (r 20) |
| swing arm | PPA-CF | ring on the wall boss r 14 | stop pins on the wall rear face at both ends of its 8 deg swing |

Bought: CSK20PP x 1 (sprag #2), 608 x 2, M3 x 25 + nut (rocker pivot), 6 + 2 M3 screws into the deck.

## 5. Gears (all spur 20 deg PPA-CF, m2 unless stated, axes parallel to the main shaft)

| gear | on | teeth | face | plane R Z | meshes |
|---|---|---|---|---|---|
| reel gear Gr | S | 16 | 10 | -34.5 .. -24.5 | swing idler |
| swing idler Gi | arm pin | 24 | 10 | -34.5 .. -24.5 | c2 always, Gr on FORWARD turns |
| take-up gear c2 | hex on c1 hub | 24 | 10 | -34.5 .. -24.5 | swing idler |
| bus gear c1 | free on C | 24 | 12 | -47.5 .. -35.5 | cad-drum BUS gear 40T, centre 64.1 |
| capstan gear Gc | C (D flat) | 24 | 12 | -61 .. -49 | cad-drum REEL output 40T, centre 64.1 |
| cup ring gear Gk | K (D flat) | m1.5 24 | 8 | -75 .. -67 | cad-drum wrap idler |

- The main shaft therefore sits 64.1 from the capstan axis; the bus plane is the motor side (smaller s) of the REEL
  plane, as D15 needs. cad-drum: BUS gear plane = my c1 plane, REEL gear plane = my Gc plane, wrap idler plane = my
  Gk plane (or tell me yours and I move my planes; they are named constants).
- Phase convention: every gear has tooth 0 on +X of its own frame at joint angle 0.
- Stall: at 15 N m wiper stall the line sees 373 N, under the 667 N break.
- Swing idler (pawl-free one-way): forward turns drag the arm + into mesh against its stop; backward turns swing it 8
  deg out. Payout back-drives the reel gear with the same tooth-force sense as take-up, so the idler stays IN during
  payout and the LIGHT clutch reacts through the bus onto the self-locking wiper worm (no separate bus brake needed).
  After aiming (backward) the cocking turns (forward) re-engage it before every shot. A tight idler pin fit gives the
  drag that swings the arm.

## 6. Slip clutch and brake input (interface with the drum brake track)

- Face clutch: annulus r 13 .. 19 on the storage reel's front face against the clutch disk D (head of S). The thrust
  ring T presses the rear 608 inner race and so the reel onto D. T is pushed by my BRAKE ROCKER (PETG, 3 x 10 plate,
  pivot M3 at the top, T 37 below the pivot, push point 35 below that). With the stack rigid, the rocker bends: it
  is the clutch spring.
- Levels (D7, D16): LIGHT in LAUNCH and in the WRAP sector, FIRM in NEUTRAL and REEL.

| level | rod push | line tension (mid pack; empty .. full) | axial preload | push force |
|---|---|---|---|---|
| LIGHT | 0 (rod on its stop) | 0.50 N (0.60 .. 0.43) | 2.5 N | 1.3 N |
| FIRM | +4.0 mm | 3.0 N (3.6 .. 2.6) | 15.1 N | 7.8 N |

- LIGHT is mechanical: the push rod's collar seats in a counterbore in the wall front (the rocker pushes it back). If
  the track or follower ever lets go, the brake stays LIGHT. 0.4 mm printed shims under T trim it.
- BRAKE INPUT: push rod d 5.6, rear end face at R (-85, -35, -24), pushed along +Z by 4.0 mm for FIRM, design force
  10 N. cad-drum's track is radial (lift 0 / 4 mm): once LAYOUT places the drum against my wall I add a 1:1 bell crank
  on my wall with a d 6 follower on track 2.

## 7. Wrap cup (interface with the drum wrap idler)

- 7 cup turns -Z from home = 7 wraps (D4); a whole number so the ring ends at the top. On my 24T ring the drum's
  wrap idler must turn exactly 7 x 24 / z_idler turns.
- Laying torque at the cup: about 0.03 N m per N of line drawn through the ring (r 24.5).
- Tow holding torque (v7 geometry nearly cancels it): -0.11 .. -0.50 N m at 100 N; design +-1 N m at the ring
  gear, held by the drum's locking arc at REEL/WRAPPED.

## 8. Line, wraps, tensions (physics, fw-base)

- 7 wraps on PLA (Euler-Eytelwein): e^(mu 2 pi 7) = 81 (mu 0.10), 733 (0.15).
  With the FIRM tail (2.6 N at a full reel) they hold 208 N at mu 0.10 and 1,878 N at mu 0.15.
- Wrapping lays 0.90 m of line.
- Storage reel: core d 40, flange d 64, rear flange a 45 deg cone (prints without support). 20 m packs inside the
  flanges with about 3.5 mm to spare.
- Launch payout (physics F1: 7.5 to 8.6 m/s): reel about 2,900 rpm, line pull in the stroke about 3 N, least brake
  against overrun in flight 0.11 N, so 0.5 N never overruns while the dart flies. Landing overrun: physics F5 0.24 m.
- Tension from wiper current (no load cell; linear model, eta 0.85; calibrate with a luggage scale):
  T = (I - I_free) / (I_stall - I_free) x 15 N m x 0.85 / 1.667 / 0.0205 m = about 22.6 N per amp above free.

| line tension | main torque | wiper amps | line speed |
|---|---|---|---|
| 5 N | 0.20 N m | 1.7 A | 0.159 m/s |
| 30 N | 1.21 N m | 2.8 A | 0.148 m/s |
| 50 N | 2.01 N m | 3.7 A | 0.139 m/s |
| 100 N | 4.02 N m | 5.9 A | 0.118 m/s |

## 9. Reel sequence (firmware; see my note to the coordinator on D16)

1. LAUNCH: brake LIGHT, cup home. Fire: the reel pays out against LIGHT.
2. Dart LANDED: WRAP at once, BEFORE the tidy and before anchoring, while the dart-side line lies slack: drum through
   the wrap sector, 7 cup turns. Physics: the turns can only take line from the dart side, because tail-to-ring flow
   is the capstan's payout sense, which sprag #2 blocks (with a taut dart side the ring would need about 0.5 N x
   e^(mu theta): 21 N at 4 turns, 360 N at 7).
3. Dart ANCHORING -> ANCHORED. Then the tidy (NEUTRAL, FIRM, wiper forward on the bus) takes the slack.
4. REEL (fly-in with the wiper creeping forward). TOW: `reel` with max_n = tow limit (D3: 50 N default).
5. Dart released: `reel` with low max_n drags it home on the capstan. Final approach per D17.
6. UNWRAP (D16): collar to NEUTRAL, cup unwinds 7 turns while the wiper turns forward: the bus takes up, and the line
   freed at the ring is pulled through the remaining turns in the capstan's free (wind-in) sense. Brake FIRM.
7. NEUTRAL -> LAUNCH (LIGHT, cup home).

## 10. Contacts

- SW_WRAP_HOME (spec, optional): not fitted. The drum's locking arc puts the cup exactly home or exactly 7 turns on,
  and the drum code reads which. Add it later if bench tests show the cup slipping.

## 11. Frame mounting (cad-frame)

- One printed reel wall: foot on the deck (deck top = R Y -45), 6 x M3 counterbored down through the foot at R (X, Z)
  (-116, 10), (-116, 36), (-62, 36), (-34, 10), (-34, 36), (24, 36), plus the exit post's 2 x M3 through the pad at
  (59, 104) and (81, 104). Heat-set inserts in the deck. The tow load stays inside the reel wall (exit eye -> pad ->
  wall -> sprag), so the deck only sees the screws.
- If cad-frame prefers to own the bearing block (spec section 3), my wall can be split at the foot; tell me.

## 12. Open items

- cad-drum: plane confirmation, wrap idler turns for 7 wraps, track 2 follower point.
- cad-launch: the first line pulley position after my exit eye.
- Harry to measure: CSK20PP keyway depths (inner 2.8, outer 1.5 assumed), whether he has 8 mm rod for S.
