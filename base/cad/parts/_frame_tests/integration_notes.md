## Auditor's notes on the cross-owner pairs (cad-frame, hand analysis)

- **Frame motor end follows BASE_SPEC section 2 (since 14:05):** the motor plate occupies y -10..0 (its front face is
  the spec's y = 0), the puck adapter fills a window r 88 in it (y -10..0) with a rear lip to y -16 out to r 97, and
  the wiper boss face is at y -10. Nothing of the frame is at y > 0 near the main axis any more, except the gear hood
  (z 44..58, r over 44, carved round every sweep). The puck carries 2 M3 inserts on its front face for cad-drum's
  SW_INDEX block at params DRUM.sw_index_mount (world x 43.1, z +6 and z -36).
- **Puck vs the reel cup and capstan (y below 0):** the puck and the plate are carved round every other
  subsystem's swept volume, so the reel unit can pass behind the plate below the axis; the frame keeps at least 4 rim
  screws with both plate and lip material (checked).
- **Wiper angle:** `frame_wiper` (the motor's output stub) now takes cad-drum's `drum_main` value in every state (they
  are the same shaft); the old drum_coupler_hub x frame_wiper_shaft overlap came from the two joints disagreeing in
  the aim-sweep audit states.
- **Coupler hub flats (cad-drum, check):** the hub's flat bore assumes the flats run to y 20.5 from a boss face at
  y 0; with the boss face at y -10 (adapter plate face at y 0, BASE_SPEC section 2) the real flats end at y +10 and the
  last 10 mm of the hub's flat bore sits on the M8 thread. Geometry is clear; torque goes through 10 to 12 mm of flat
  engagement instead of 18. Shift the hub's flats 10 mm towards the motor if that is too short.
- **Wiper shaft and the coupler (cad-drum):** the WIPER model (params, not measured) has the output boss r 12 standing
  4 mm proud of the boss face (y -10..-6), the shaft d10 with flats 8 across from y -4 to y +10 and the M8 thread
  y +10..+22. The coupler hub (y 0.5..) must take the flats and leave room for the M8 nut.
- **sensors subsystem vs the wiper envelope:** if SW_GAP parts sit inside `frame_wiper_motor` (an ENVELOPE of an
  unmeasured motor: gearbox x +-40, z -38..84, y -64..-18 world, can r 29 along +z from z 84 to 179, centre y -36),
  move them out of it or put them on the puck front face (y 0..).
- The frame motor plate, bridge, leg, puck, hood and front guard are carved in 3D round every other subsystem's swept
  volume (one convex hull per part per 6 timeline states, grown 4 mm); if a subsystem moves, the next frame build
  follows it. A subsystem that does not build is left out of the carve and of this audit (listed at the top).
