## Frame-origin check (run before the audit, 12:15 Sydney, base-v1 6421270)

**BLOCKER: three subsystems use a different world origin from BASE_SPEC section 2 / params v1.1.** The spec puts the
origin ON the main axis (main axis z = 0, deck top z = LAYOUT.deck_top_z = -165). In the rest state:

| subsystem | evidence (world, rest state) | offset from the spec |
|---|---|---|
| cad-drum | joint `drum_base` origin (0, 0, 165); `drum_shaft` z 154..176; brackets stand on z = 0 | +165 in z |
| cad-launch | joints `launch_rail`, `launch_sleeve`, `launch_drive` origin (0, 140, 165); `launch_aim_housing` stands on z = 0 | +165 in z |
| cad-reel | joint `reel_base` origin (-40, 140, 45); `reel_wall` x -162..47, y 24..161, stands on z = 0 | +165 in z, and not at LAYOUT.reel_frame_R (capstan should be at x -18.66, z -69.64, R origin y 33.5) |
| cad-frame | wiper axis z = 0, deck top z = -165, motor boss face y = 0 | spec |

Fix (each owner): subtract 165 from every world z (joint origins whose parent is None), i.e. read
`P.LAYOUT["deck_top_z"]` and put the deck there; cad-reel also places frame R from `LAYOUT["reel_frame_R"]`. Until then
the frame cannot see your feet on the deck (no deck inserts for them) and the audit below compares parts 165 mm apart.
The frame deck currently covers the params target footprint x -160..255, y -95..220 (4 dovetailed tiles).

