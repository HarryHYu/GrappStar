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
- `reel_NOTES.md`: every interface number for the other workers (v4).
- Tests: `test_reel_calc.py` (41) and `test_reel.py` (15).

## 2. Decisions I made (Harry was away)

- Kept Harry's v7 reel exactly where the line touches it (ring, groove, guide 1 mm behind the flange, exit eye 31.5
  past the ring, plain round eyes). The two eye axes are tilted to bisect the line's bend (still plain round holes):
  along X the launch line left the guide 50 deg off axis and grazed the post.
- Sprag #2 is the capstan backstop in a separate PPA-CF hub (prints upright, so the 47 mm press seat is round).
- Wrap order: wrap right after landing, while the dart-side line is slack (adopted as D22). With a taut dart side the
  turns would have to slip on the backstopped capstan (about 360 N at the ring at mu 0.15).
- Take-up bus (D15) built as one bus gear on the capstan tube that also drives a pawl-free swing idler riding its own
  hub (adopted as D25); no bus drag brake (payout keeps the idler engaged; the reaction goes to the self-locking wiper).
- Clutch: friction ring r 20..30 on a separate flat flange disc; the PETG brake rocker is the FIRM spring; LIGHT is set
  by two M3 trim screws (tolerance-free), and the brake fails to LIGHT if the drum track lets go.
- Storage reel flange 72 (spec said 66: 1.6 mm freeboard for 20 m), two-piece so it prints without support (D25).
- No slack-line shoe: physics F5 puts landing overrun at 0.24 m and the shroud keeps loops on the reel.

## 3. Verified (pasted at the final update)

## 4. Open items

## 5. What Harry must measure
