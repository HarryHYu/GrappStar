# Whole-base integration audit (cad-frame)

Run 2026-11-03 13:28 Sydney on `234aff2` (base-v1 merged into w/cad-frame), `tools.audit --dt 0.5 --jobs 2 --no-refine`, 190 parts, 855 states, 119 s. Verdict: **NOT CLEAN**.

Subsystems: drum, frame, launch, reel

## Frame-origin check

Spec: main axis z = 0, deck top z = -165.0. Lowest part bottom per subsystem (rest state):

| subsystem | lowest part z | root joints (parent None) | verdict |
|---|---|---|---|
| drum | -165.0 | drum_base (0.0, 0.0, 0.0) | ok |
| frame | -173.0 | frame_rail_guard (0.0, 140.0, 0.0); frame_wiper (0.0, 0.0, 0.0) | parts below the deck |
| launch | -165.0 | launch_rail (0.0, 140.0, 0.0); launch_sleeve (0.0, 140.0, 0.0); launch_drive (0.0, 140.0, 0.0) | ok |
| reel | -165.0 | reel_base (-18.7, 33.5, -69.6) | ok |

## Collisions (20)

| part A (owner) | part B (owner) | worst overlap mm3 | states hit | worst state: phase, t | note |
|---|---|---|---|---|---|
| drum_index_block (drum) | frame_motor_adapter (frame) | 4830.288 | 855 | load, 0.0 |  |
| drum_coupler_hub (drum) | frame_wiper_motor (frame) | 1238.537 | 855 | aim, 2.5 |  |
| frame_motor_adapter (frame) | reel_sprag_hub (reel) | 1096.572 | 855 | load, 0.0 |  |
| frame_spring_guard_l_1 (frame) | launch_carriage_yoke (launch) | 837.094 | 28 | launch_shot, None |  |
| frame_spring_guard_l_2 (frame) | launch_carriage_yoke (launch) | 837.094 | 829 | launch_shot, None |  |
| frame_spring_guard_r_1 (frame) | launch_carriage_yoke (launch) | 837.094 | 28 | launch_shot, None |  |
| frame_spring_guard_r_2 (frame) | launch_carriage_yoke (launch) | 837.094 | 829 | launch_shot, None |  |
| launch_rail_front (launch) | launch_dart (launch) | 395.367 | 2 | approach, 103.5 |  |
| launch_carriage (launch) | launch_dart (launch) | 268.616 | 1 | approach, 103.5 |  |
| drum_index_lever (drum) | frame_motor_adapter (frame) | 259.673 | 855 | cock, 8.5 |  |
| drum_coupler (drum) | reel_wall (reel) | 248.199 | 855 | drum_index, 0.0 |  |
| drum_rail (drum) | launch_rail_mid (launch) | 194.735 | 73 | aim_fall, None |  |
| drum_gear_reel (drum) | reel_gc (reel) | 144.265 | 3 | reel_swing_out, 0.0 |  |
| drum_shaft (drum) | frame_wiper_shaft (frame) | 99.656 | 855 | aim_sw_low, None |  |
| drum_coupler_hub (drum) | frame_wiper_shaft (frame) | 81.52 | 855 | aim_sweep, None |  |
| drum_coupler (drum) | reel_bus (reel) | 71.559 | 3 | reel_swing_out, 0.0 |  |
| drum_rail (drum) | launch_pulley_piv (launch) | 42.717 | 55 | aim_fall, None |  |
| frame_motor_adapter (frame) | reel_sprag (reel) | 39.591 | 855 | load, 0.0 |  |
| frame_motor_adapter (frame) | reel_drum (reel) | 21.7 | 855 | drum_wiper, 0.0 |  |
| drum_rail (drum) | launch_line_c (launch) | 2.991 | 50 | aim, 5.0 |  |

## Failed clearances (0)
None.

## Floating or ungrounded parts (0)
None.

## Static fails (0)
None.

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
