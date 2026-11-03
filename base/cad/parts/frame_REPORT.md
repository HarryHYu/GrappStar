# frame_REPORT (worker cad-frame) - interim, updated through the day

Final version at 18:30 Sydney. Data and interfaces: `frame_NOTES.md`. Whole-base audit: `frame_INTEGRATION.md`.

## Scope as built (BASE_SPEC v1.2.x)

- Motor plate (`frame_cheek_r`): the only plate after D23. Carries the fixed motor interface (67 mm coupler bore, a
  2 mm counterbore that keys the adapter hub, 12 x M3 clearance holes on R 43). Its outline starts as a rectangle from
  the deck to above the main axis and is CARVED round every other subsystem's swept volume in its y slab (one convex
  hull per part per 6 timeline states, 4 mm margin), keeping the piece that holds the axis; feet go wherever the
  carved outline still stands on the deck.
- Wiper adapter (`frame_motor_adapter`, PPA-CF): separately printed, every motor number from `params.WIPER`; slotted
  M6 holes (heads from the plate side, bench-assembled); hub ring keyed in the plate counterbore; inserts only at the
  ring positions clear of the motor's head slots (at least 4, no gap over 150 deg, checked).
- Deck: covers the params target footprint plus every foot of every subsystem (detected from the rest-state slice
  0.5 mm above the deck), an M3 insert under every round 2.9 to 4.1 mm hole found in another subsystem's foot, seams
  kept 14 mm off every deck hole, 1:1 drill template PDF + CSV for the rover.
- Electronics bay 124 x 124 in the params region (x 110..240, y -70..60), USB to +X (outward), exits to -X.
- Rover kit (D18): Kiyo X edge-plate mount (its own clip, or a 1/4-20 screw), RC832 holder (antenna up), cable anchors.
- Steps, load calcs, PC tests, render script, integration script.

## Integration auditor role

`cad/parts/_frame_tests/integration.py` runs `tools.audit` over every subsystem module and writes
`frame_INTEGRATION.md` (origin check, collisions with owners and worst states, clearances, floating, static fails).

## Requests to other workers / coordinator
- Every subsystem: put feet on the deck top (world z = LAYOUT.deck_top_z) with M3 clearance holes (3.4); the frame adds
  the inserts automatically. Optional `deck_holes(P)` for explicit holes.
- `tools.assembly` loads every `cad/parts/*.py` without a leading underscore as a subsystem: helper modules such as
  `launch_gears.py`, `launch_kin.py`, `reel_calc.py`, `test_*.py` make `Assembly.load()` (no `--only`) fail.
- cad-drum: the coupler is r 32 over y 0.5..36 (frame bore 67 sized for it). Tell me if it grows.

## Open items
- Guards over gears and the spring path (after the others' geometry settles), CG/stability with the full assembly.
