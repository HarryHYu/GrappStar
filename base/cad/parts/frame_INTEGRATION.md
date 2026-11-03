# Whole-base integration audit (cad-frame)

Run 2026-11-03 14:26 Sydney on `f212c50` (base-v1 merged into w/cad-frame), `tools.audit --dt 0.5 --jobs 2 --no-refine`, 127 parts, 756 states, 601 s. Verdict: **NOT CLEAN**.

Subsystems: drum, frame, launch (DOES NOT BUILD: NameError: name 'RIB_T' is not defined (launch.py:385)), reel, sensors

**Left out of this audit because they do not build:** launch: DOES NOT BUILD: NameError: name 'RIB_T' is not defined (launch.py:385)

## Frame-origin check

Spec: main axis z = 0, deck top z = -165.0. Lowest part bottom per subsystem (rest state):

| subsystem | lowest part z | root joints (parent None) | verdict |
|---|---|---|---|
| drum | -165.0 | drum_base (0.0, 0.0, 0.0) | ok |
| frame | -173.0 | frame_wiper (0.0, 0.0, 0.0) | ok |
| reel | -165.0 | reel_base (-18.7, 33.5, -69.6) | ok |
| sensors | -157.0 |  | ok |

## Assembly errors
- joint drum_out_launch: gear refers to unknown joint launch_sleeve
- part sensors_sw_gap: unknown joint launch_rail
- part sensors_sw_gap_lever: unknown joint launch_rail
- part sensors_sw_gap_screw: unknown joint launch_rail
- part sensors_sw_gap_jam: unknown joint launch_rail
- part sensors_sw_gap_nut: unknown joint launch_rail

## Collisions (7)

| part A (owner) | part B (owner) | worst overlap mm3 | states hit | worst state: phase, t | note |
|---|---|---|---|---|---|
| frame_motor_adapter (frame) | reel_drum (reel) | 10012.096 | 756 | drum_wiper, 0.0 |  |
| frame_motor_adapter (frame) | reel_c (reel) | 2513.88 | 756 | drum_wiper, 0.0 |  |
| frame_motor_adapter (frame) | reel_k (reel) | 783.488 | 756 | drum_sweep, 0.0 |  |
| frame_wiper_motor (frame) | sensors_sw_gap (sensors) | 555.102 | 756 | load, 0.0 |  |
| frame_motor_adapter (frame) | sensors_sw_gap_lever (sensors) | 147.869 | 756 | load, 0.0 |  |
| frame_motor_adapter (frame) | sensors_sw_gap_jam (sensors) | 37.224 | 756 | load, 0.0 |  |
| frame_motor_adapter (frame) | sensors_sw_gap_screw (sensors) | 24.653 | 756 | load, 0.0 |  |

## Failed clearances (0)
None.

## Floating or ungrounded parts (16)
- drum_dogring_launch (drum): touches nothing in the rest state
- sensors_as5600 (sensors): not connected to a grounded part
- sensors_as5600_carrier (sensors): not connected to a grounded part
- sensors_as5600_strip_top (sensors): not connected to a grounded part
- sensors_as5600_nut_top (sensors): not connected to a grounded part
- sensors_as5600_screw_top (sensors): not connected to a grounded part
- sensors_as5600_strip_bot (sensors): not connected to a grounded part
- sensors_as5600_nut_bot (sensors): not connected to a grounded part
- sensors_as5600_screw_bot (sensors): not connected to a grounded part
- sensors_as5600_col_screws (sensors): not connected to a grounded part
- sensors_sw_rail_low (sensors): not connected to a grounded part
- sensors_sw_rail_low_lever (sensors): not connected to a grounded part
- sensors_sw_rail_low_screw (sensors): not connected to a grounded part
- sensors_sw_rail_low_jam (sensors): not connected to a grounded part
- sensors_sw_rail_low_nut (sensors): not connected to a grounded part
- sensors_sw_rail_low_bracket (sensors): not connected to a grounded part

## Static fails (3)
- drum_fork (drum): no real flat face on the bed (94.2 mm2, need 150)
- frame_cheek_r (frame): not watertight as STL (self-touching edges after vertex merge)
- frame_guard_front (frame): not watertight as STL (self-touching edges after vertex merge)

## Subsystem check failures (2)
- frame: frame_cheek_r: not watertight as an STL after vertex merge
- frame: frame_guard_front: not watertight as an STL after vertex merge

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
