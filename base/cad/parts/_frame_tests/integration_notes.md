## Auditor's notes on the cross-owner pairs (cad-frame, hand analysis)

- **Frame motor end follows BASE_SPEC section 2 (since 14:05):** the motor plate occupies y -10..0 (its front face is
  the spec's y = 0), the puck adapter fills a window r 88 in it (y -10..0) with a rear lip to y -16 out to r 97, and
  the wiper boss face is at y -10 (params WIPER boss_face_y, D48). Nothing of the frame is at y > 0 near the main axis
  any more, except the gear hood (z 44..58, r over 44, carved round every sweep). cad-drum's SW_INDEX block is on its
  idler post 2 strut (params DRUM.sw_index_mount), so the puck has no inserts for it.
- **Tether through the puck (cad_support-1):** cad-reel's line crosses the puck twice (guide eye to cup ring at
  wraps 0, pulley 1 to pulley 2 in every state). The frame cuts an r 3 channel round both runs (reel.line_path at
  wraps 0 and full wraps), run out to the puck's edge: the first into the capstan opening, the second as a rim notch.
  tools/tests/test_wiper_hub_and_tether.py checks the line against every frame part over the timeline.
- **Puck vs the reel cup and capstan (y below 0):** the puck and the plate are carved round every other
  subsystem's swept volume, so the reel unit can pass behind the plate below the axis; the frame keeps at least 4 rim
  screws with both plate and lip material (checked).
- **Wiper angle:** `frame_wiper` (the motor's output stub) now takes cad-drum's `drum_main` value in every state (they
  are the same shaft); the old drum_coupler_hub x frame_wiper_shaft overlap came from the two joints disagreeing in
  the aim-sweep audit states.
- **Coupler hub flats (cad-drum, resolved by D48):** params WIPER boss_face_y (-10) is the one datum: the frame
  places the motor from it and cad-drum derives the hub from it and the WIPER shaft numbers. The WIPER model (params,
  not measured) has the output boss r 12 standing 4 mm proud of the boss face (y -10..-6), the shaft d10 with flats 8
  across from y -4 to y +10 and the M8 thread y +10..+22. The hub reaches into the puck bore from y -5: D bore on
  13.5 mm of the flats, M8 nut at y 13..19.5 on the thread (tools/tests/test_wiper_hub_and_tether.py poses both).
- **sensors subsystem vs the wiper envelope:** if SW_GAP parts sit inside `frame_wiper_motor` (an ENVELOPE of an
  unmeasured motor: gearbox x +-40, z -38..84, y -64..-18 world, can r 29 along +z from z 84 to 179, centre y -36),
  move them out of it or put them on the puck front face (y 0..).
- The frame motor plate, bridge, leg, puck, hood and front guard are carved in 3D round every other subsystem's swept
  volume (one convex hull per part per 6 timeline states, grown 4 mm); if a subsystem moves, the next frame build
  follows it. A subsystem that does not build is left out of the carve and of this audit (listed at the top).
