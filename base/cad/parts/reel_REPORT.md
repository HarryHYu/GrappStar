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

## 5. What Harry must measure
