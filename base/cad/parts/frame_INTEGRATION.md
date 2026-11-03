# Whole-base integration audit (cad-frame)

Run 2026-11-03 14:46 Sydney on `7cce086` (base-v1 merged into w/cad-frame), `tools.audit --dt 0.5 --jobs 2 --no-refine`, 226 parts, 885 states, 139 s. Verdict: **NOT CLEAN**.

Subsystems: drum, frame, launch, reel, sensors

## Frame-origin check

Spec: main axis z = 0, deck top z = -165.0. Lowest part bottom per subsystem (rest state):

| subsystem | lowest part z | root joints (parent None) | verdict |
|---|---|---|---|
| drum | -165.0 | drum_base (0.0, 0.0, 0.0) | ok |
| frame | -173.0 | frame_wiper (0.0, 0.0, 0.0) | ok |
| launch | -165.0 | launch_rail (0.0, 140.0, 0.0); launch_sleeve (0.0, 140.0, 0.0); launch_drive (0.0, 140.0, 0.0) | ok |
| reel | -165.0 | reel_base (-18.7, 33.5, -69.6) | ok |
| sensors | -157.0 |  | ok |

## Collisions (1)

| part A (owner) | part B (owner) | worst overlap mm3 | states hit | worst state: phase, t | note |
|---|---|---|---|---|---|
| drum_coupler_hub (drum) | frame_wiper_shaft (frame) | 21.885 | 149 | aim_sweep, None |  |

## Failed clearances (0)
None.

## Floating or ungrounded parts (0)
None.

## Static fails (1)
- sensors_sw_rail_low_bracket (sensors): not watertight as STL (self-touching edges after vertex merge)

## Subsystem check failures (0)
None.

## Auditor's notes on the cross-owner pairs (cad-frame, hand analysis)

- **Frame motor end follows BASE_SPEC section 2 (since 14:05):** the motor plate occupies y -10..0 (its front face is
  the spec's y = 0), the puck adapter fills a window r 88 in it (y -10..0) with a rear lip to y -16 out to r 97, and
  the wiper boss face is at y -10. Nothing of the frame is at y > 0 near the main axis any more, except the gear hood
  (z 44..58, r over 44, carved round every sweep). The puck carries 2 M3 inserts on its front face for cad-drum's
  SW_INDEX block at params DRUM.sw_index_mount (world x 43.1, z +6 and z -36).
- **Puck vs the reel cup and capstan (y below 0):** the puck and the plate are carved round every other
  subsystem's swept volume, so the reel unit can pass behind the plate below the axis; the frame keeps at least 4 rim
  screws with both plate and lip material (checked).
- **Wiper shaft and the coupler (cad-drum):** the WIPER model (params, not measured) has the output boss r 12 standing
  4 mm proud of the boss face (y -10..-6), the shaft d10 with flats 8 across from y -4 to y +10 and the M8 thread
  y +10..+22. The coupler hub (y 0.5..) must take the flats and leave room for the M8 nut.
- **sensors subsystem vs the wiper envelope:** if SW_GAP parts sit inside `frame_wiper_motor` (an ENVELOPE of an
  unmeasured motor: gearbox x +-40, z -38..84, y -64..-18 world, can r 29 along +z from z 84 to 179, centre y -36),
  move them out of it or put them on the puck front face (y 0..).
- The frame motor plate, bridge, leg, puck, hood and front guard are carved in 3D round every other subsystem's swept
  volume (one convex hull per part per 6 timeline states, grown 4 mm); if a subsystem moves, the next frame build
  follows it. A subsystem that does not build is left out of the carve and of this audit (listed at the top).

Method: exact manifold intersection of every part pair over the mission timeline plus each subsystem's audit_states; pairs that never change relative pose are checked once. Allowed contacts are not listed.
