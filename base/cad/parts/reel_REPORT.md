# cad-reel report (worker cad-reel, 3 Nov 2026)

Status: DRAFT, updated through the day; final version at 18:30 Sydney.

## 1. What is done

- `reel.py`: the whole reel subsystem on the cad-infra API (parts, joints, kin, allowed, clearances, grounded,
  audit_states, checks, steps, line_path), placed by params.LAYOUT.reel_frame_R and reading REEL, DRUM and LAYOUT
  from params (v1.2).
- `reel_v7.py`: Harry's v7 capstan drum and wrap cup ported exactly to manifold3d (ring, block, U groove cut last,
  funnels); only the bores changed (drum keyed on the capstan tube, cup keyed on the cup shaft with the D hidden).
- `reel_calc.py`: wraps (Euler-Eytelwein), storage pack, spin-up and overrun, clutch preloads and the rocker spring,
  gear train speeds and torques, current to tension, cup holding torque, unwrap schedule.
- `reel_NOTES.md`: every interface number for the other workers (v5, brake linkage in section 6).
- Brake input linkage (review 2 rebuild): cad-drum's track 2 cam pushes my brake bar, the bar turns a bell crank, the
  crank lifts the rocker's stiff leg. kin follows the round nose's envelope on cad-drum's real cam profile.
- Rocker spring sized with its real section (`reel_calc.rocker_section`).
- Tests: `test_reel_calc.py` and `test_reel.py` (counts in section 3).

## 2. Decisions I made (Harry was away)

- Kept Harry's v7 reel exactly where the line touches it (ring, groove, guide 1 mm behind the flange, exit eye 31.5
  past the ring, plain round eyes). The two eye axes are tilted to bisect the line's bend (still plain round holes):
  along X the launch line left the guide 50 deg off axis and grazed the post.
- Sprag #2 is the capstan backstop in a separate PPA-CF hub (prints upright, so the 47 mm press seat is round).
- Wrap order: wrap right after landing, while the dart-side line is slack (adopted as D22). With a taut dart side the
  turns would have to slip on the backstopped capstan (about 360 N at the ring at mu 0.15).
- Take-up bus (D15) built as one bus gear on the capstan tube that also drives a pawl-free swing idler riding its own
  hub (adopted as D25); no bus drag brake (payout keeps the idler engaged; the reaction goes to the self-locking wiper).
- Clutch: friction ring r 20..30 on a separate flat flange disc; the PPA-CF brake rocker is the FIRM spring; LIGHT is set
  by two M3 trim screws (tolerance-free), and the brake fails to LIGHT if the drum track lets go.
- Storage reel flange 72 (spec said 66: 1.6 mm freeboard for 20 m), two-piece so it prints without support (D25).
- No slack-line shoe: physics F5 puts landing overrun at 0.24 m and the shroud keeps loops on the reel.
- Brake linkage (review 2: the old push rod from behind the wall could not reach cad-drum's cam, which sits in front
  of my wall 137 mm away): a sliding bar in front of the wall, under the shroud and under a shortened guide post, plus
  a bell crank left of the reel pushing a stiff leg on the rocker from behind (arm B in a 3 mm recess in the wall).
  Chosen over a wedge because a sliding 45 deg wedge needs about 18 N at the cam (cad-drum designed for 15 N); the
  crank needs 5 N plus pin friction.
- Rocker in PPA-CF, not PETG (Harry has no PETG; PLA would creep at 17 MPa held for minutes). The plate stays
  3.0 mm (17 MPa at FIRM); the crank ratio (arm B 10.8 / arm A 20) sets the 2.1 mm leg stroke that gives exactly
  LIGHT to FIRM at E 5 GPa. Review 2 showed the old 4 mm at Y -33 would have given FIRM far too high.
- Wall top cut r 37.5 round the main axis (cad-drum's coupler grew to r 35 in my wall plane).
- SW_ARM lever tip moved 3 mm toward -X (home notch rotated with it) so its nuts clear cad-drum's wrap idler by 3.5 mm.

## 3. Verified (pasted at the final update)

## 4. Open items

- cad-frame (not mine to change):
  - frame_motor_adapter cuts through my sprag hub, sprag and capstan drum. Needs a notch r 30.5 about world
    (x -18.66, z -69.64) for y 0..24, or its lower edge raised to z -38.
  - frame_guard_front crosses my pulley 2 post arm. Needs a slot at world x 90..106, y 104..120, z -70..-30.
- D46 (coordinator): the cup ring stays a plain eye; a steel ring liner is left open for Harry. A roller in the ring
  would rebuild Harry's v7 cup, so I did not do it.
- D43 storage guide eye: kept as a plain eye. It carries only the tail, at most 5 N (FIRM). The pack position sweeps
  22 mm across a 45 mm run to the guide, which would throw a line off a pulley. Plain eye with coned mouths.
- Mid-wrap the dart line reaches pulley 1 steeply while the ring is turned away from home. It may brush the pulley rim
  and the lower cheek edge; this is slack line (D22). Under tow the cup is at home and the line is in the groove.
- Printing (needs supports or care, noted rather than redesigned):
  - sprag hub seat lip and the ring under it (paint supports inside the seat);
  - the pulley 1 and pulley 2 post cheeks and the pulley 2 arm (supports);
  - SW bracket flange (supports, or print on the flange's wall face).
- 693 inner-race bosses: 0.7 mm wall round the M3 hole; print slowly, or drop them and use two thin shims per pulley.
- Past the LAUNCH stop (drum -4 .. -2) the bar nose already sees cad-drum's gap blend toward FIRM (2.5 mm at -4).
  Harmless unless the drum parks on its stop.
- Rocker creep: 17 MPa held through NEUTRAL, WRAPPED and REEL. Recheck FIRM with the luggage scale after the first
  week (section 5).

## 5. What Harry must measure

1. Capstan sprag CSK20PP keyway depths (inner and outer) against the seat and key rib dimensions in reel_NOTES.md.
2. mu of the Dyneema on the printed capstan (7 wraps hold 407 N at mu 0.10 with the 5 N tail). Hang a weight over a
   printed drum offcut: the wrap count is set from it.
3. LIGHT drag: drum at LAUNCH, reel full. A 50 g weight on the line end must just pull line off slowly (0.5 N). Set
   it with the two trim screws: park the swing arm, line up the reel gear's holes, use a 2.5 mm hex key from behind.
4. FIRM drag: drum in REEL. The luggage scale on the line end must read 5.0 N at slip. Set it with the FIRM screw in
   the rocker leg (one turn is about 1 N), then lock its nut. This also measures the real E of Harry's PPA-CF: the
   screw range covers 4 to 9 GPa.
5. Bar travel: drum at LAUNCH, the nose must sit about 0.2 mm off the cam (feeler). At REEL the crank turns about 11 deg.
6. Storage pack: after winding 20 m, the pack top must stay at least 4 mm under the d 80 flange rim.
