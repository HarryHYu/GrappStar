# Whole-base integration audit (cad-frame)

Run 2026-11-03 14:06 Sydney on `0f2ee6e` (base-v1 merged into w/cad-frame), `tools.audit --dt 0.5 --jobs 2 --no-refine`, 224 parts, 897 states, 8 s. Verdict: **NOT CLEAN**.

Subsystems: drum, frame, launch, reel, sensors

## Frame-origin check

Spec: main axis z = 0, deck top z = -165.0. Lowest part bottom per subsystem (rest state):

| subsystem | lowest part z | root joints (parent None) | verdict |
|---|---|---|---|
| (all) | load failed: NameError: name 'RIB_T' is not defined | | |

## Collisions (45)

| part A (owner) | part B (owner) | worst overlap mm3 | states hit | worst state: phase, t | note |
|---|---|---|---|---|---|
| drum_index_block (drum) | frame_motor_adapter (frame) | 4830.288 | 897 | load, 0.0 |  |
| launch_aim_housing (launch) | sensors_sw_rail_low_bracket (sensors) | 3988.73 | 897 | load, 0.0 |  |
| drum_dog_post (drum) | sensors_sw_gap_bracket (sensors) | 1429.398 | 897 | load, 0.0 |  |
| drum_coupler_hub (drum) | frame_wiper_motor (frame) | 1238.537 | 897 | aim, 2.5 |  |
| launch_aim_housing (launch) | sensors_as5600_carrier (sensors) | 1207.447 | 897 | load, 0.0 |  |
| frame_motor_adapter (frame) | reel_sprag_hub (reel) | 1096.572 | 897 | load, 0.0 |  |
| launch_aim_housing (launch) | sensors_as5600 (sensors) | 586.065 | 897 | load, 0.0 |  |
| reel_p2_post (reel) | sensors_sw_gap_bracket (sensors) | 397.881 | 652 | load, 0.0 |  |
| launch_rail_mid (launch) | sensors_sw_gap_bracket (sensors) | 358.248 | 897 | load, 0.0 |  |
| launch_sw_rail_low (launch) | sensors_sw_rail_low (sensors) | 328.899 | 897 | load, 0.0 |  |
| launch_sw_gap (launch) | sensors_sw_gap (sensors) | 305.698 | 897 | load, 0.0 |  |
| launch_sw_rail_low (launch) | sensors_sw_rail_low_bracket (sensors) | 297.75 | 897 | load, 0.0 |  |
| launch_rail_mid (launch) | sensors_sw_gap (sensors) | 285.47 | 897 | load, 0.0 |  |
| drum_index_lever (drum) | frame_motor_adapter (frame) | 259.673 | 897 | cock, 8.5 |  |
| reel_p2_post (reel) | sensors_sw_gap (sensors) | 254.427 | 652 | aim_fall, None |  |
| drum_coupler (drum) | reel_wall (reel) | 248.199 | 897 | drum_index, 0.0 |  |
| launch_rail_mid (launch) | sensors_sw_gap_lever (sensors) | 224.71 | 897 | load, 0.0 |  |
| launch_aim_housing (launch) | sensors_sw_rail_low (sensors) | 203.52 | 897 | load, 0.0 |  |
| launch_aim_as5600 (launch) | sensors_as5600 (sensors) | 179.941 | 897 | load, 0.0 |  |
| launch_sw_gap (launch) | sensors_sw_gap_bracket (sensors) | 164.097 | 897 | load, 0.0 |  |
| launch_sw_gap (launch) | sensors_sw_gap_lever (sensors) | 146.761 | 897 | load, 0.0 |  |
| drum_gear_reel (drum) | reel_gc (reel) | 144.265 | 3 | reel_swing_out, 0.0 |  |
| launch_aim_as5600 (launch) | sensors_as5600_strip_bot (sensors) | 102.238 | 897 | load, 0.0 |  |
| launch_aim_as5600 (launch) | sensors_as5600_strip_top (sensors) | 102.238 | 897 | load, 0.0 |  |
| drum_shaft (drum) | frame_wiper_shaft (frame) | 99.656 | 897 | aim_sw_low, None |  |
| drum_coupler_hub (drum) | frame_wiper_shaft (frame) | 81.52 | 897 | aim_sweep, None |  |
| launch_sw_gap_screw (launch) | sensors_sw_gap_lever (sensors) | 80.334 | 897 | load, 0.0 |  |
| drum_coupler (drum) | reel_bus (reel) | 71.559 | 3 | reel_swing_out, 0.0 |  |
| launch_aim_housing (launch) | sensors_as5600_strip_bot (sensors) | 63.444 | 897 | load, 0.0 |  |
| launch_aim_housing (launch) | sensors_as5600_strip_top (sensors) | 63.444 | 897 | load, 0.0 |  |
| launch_sw_gap (launch) | sensors_sw_gap_screw (sensors) | 45.087 | 897 | load, 0.0 |  |
| launch_sw_rail_low_screw (launch) | sensors_sw_rail_low (sensors) | 41.364 | 897 | load, 0.0 |  |
| frame_motor_adapter (frame) | reel_sprag (reel) | 39.591 | 897 | load, 0.0 |  |
| launch_sw_rail_low_nut (launch) | sensors_sw_rail_low (sensors) | 36.061 | 897 | load, 0.0 |  |
| launch_carriage (launch) | launch_big_gear (launch) | 27.414 | 15 | sensors_gap_bump, None |  |
| launch_aim_as5600 (launch) | sensors_as5600_carrier (sensors) | 26.288 | 897 | load, 0.0 |  |
| launch_sw_rail_low_screw (launch) | sensors_sw_rail_low_lever (sensors) | 24.194 | 897 | load, 0.0 |  |
| frame_motor_adapter (frame) | reel_drum (reel) | 21.7 | 897 | drum_wiper, 0.0 |  |
| launch_rail_mid (launch) | sensors_sw_gap_jam (sensors) | 20.249 | 897 | load, 0.0 |  |
| launch_rail_mid (launch) | sensors_sw_gap_screw (sensors) | 20.129 | 897 | load, 0.0 |  |
| launch_sw_gap (launch) | sensors_sw_gap_nut (sensors) | 17.188 | 897 | load, 0.0 |  |
| launch_sw_gap_nut (launch) | sensors_sw_gap (sensors) | 12.392 | 897 | load, 0.0 |  |
| launch_sw_gap (launch) | sensors_sw_gap_jam (sensors) | 7.859 | 897 | load, 0.0 |  |
| launch_rail_mid (launch) | sensors_sw_gap_nut (sensors) | 7.514 | 897 | load, 0.0 |  |
| launch_sw_rail_low (launch) | sensors_sw_rail_low_nut (sensors) | 3.6 | 897 | load, 0.0 |  |

## Failed clearances (0)
None.

## Floating or ungrounded parts (0)
None.

## Static fails (2)
- frame_spring_guard_r_2 (frame): not watertight as STL (self-touching edges after vertex merge)
- frame_spring_guard_l_3 (frame): not watertight as STL (self-touching edges after vertex merge)

## Subsystem check failures (0)
None.

## Auditor's notes on the cross-owner pairs (cad-frame, hand analysis)

- **Adapter zone (frame) vs drum_index_block, drum_index_lever (drum), reel_sprag_hub, reel_sprag (reel).** The wiper
  adapter needs the ring r 33.5 to 49 round the main axis for y 0..24.5 (hub ring with the M3 inserts, keyed into the
  motor plate counterbore at y 22..24) plus its wings (motor bosses, y 0..10, out to r about 75 with
  `WIPER.orient_deg` 90: wings point up). Carving the adapter round these parts would remove 4 of its ring inserts
  and a motor boss slot. Request: cad-drum moves the SW_INDEX block and lever outside r 51 or beyond y 25; cad-reel
  keeps the sprag hub outside r 51 of the main axis for y below 25 (or moves the hub to the +y side of the reel wall).
- **drum_coupler_hub vs frame_wiper_motor and frame_wiper_shaft; drum_shaft vs frame_wiper_shaft.** The wiper model
  (params.WIPER, not measured) has the output boss r 12 standing 4 mm proud of the boss face (y 0..4), the shaft d10
  with flats 8 across from y 6 to 20, then the M8 thread y 20..32. The coupler hub starts at y 0.5 and its bore does
  not take this shaft (about 100 mm3 each). Request to cad-drum: hub bore = WIPER shaft (D-flat 10 / 8 plus fits), a
  12.5 x 4.5 recess for the boss, and room for the M8 nut on the thread (or start the hub at y 4.5).
- The frame motor plate, bridge and leg are carved in 3D round every other subsystem's swept volume (hulls per 6
  timeline states, 3 mm grown); if a subsystem moves, the next frame build follows it.

Method: exact manifold intersection of every part pair over the mission timeline plus each subsystem's audit_states; pairs that never change relative pose are checked once. Allowed contacts are not listed.
