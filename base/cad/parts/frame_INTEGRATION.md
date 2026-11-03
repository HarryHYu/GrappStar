# Whole-base integration audit (cad-frame)

Run 2026-11-03 01:54 Sydney on `e85d3dc` (base-v1 merged into w/cad-frame), `tools.audit --dt 0.5 --jobs 2 --no-refine`, 133 parts, 831 states, 145 s. Verdict: **NOT CLEAN**.

Subsystems: drum, frame, launch, reel

## Frame-origin check

Spec: main axis z = 0, deck top z = -165.0. Lowest part bottom per subsystem (rest state):

| subsystem | lowest part z | root joints (parent None) | verdict |
|---|---|---|---|
| drum | -165.0 | drum_base (0.0, 0.0, 0.0) | ok |
| frame | -173.0 | frame_wiper (0.0, 0.0, 0.0) | parts below the deck |
| launch | -135.4 | launch_rail (0.0, 140.0, 0.0); launch_sleeve (0.0, 140.0, 0.0); launch_drive (0.0, 140.0, 0.0) | ok |
| reel | -165.0 | reel_base (-18.7, 33.5, -69.6) | ok |

## Collisions (17)

| part A (owner) | part B (owner) | worst overlap mm3 | states hit | worst state: phase, t | note |
|---|---|---|---|---|---|
| drum_index_block (drum) | frame_motor_adapter (frame) | 6256.192 | 831 | load, 0.0 |  |
| launch_aim_housing (launch) | launch_aim_cap (launch) | 2380.36 | 831 | load, 0.0 |  |
| drum_bracket_idler2 (drum) | reel_wall (reel) | 1547.718 | 831 | load, 0.0 |  |
| drum_coupler_hub (drum) | frame_wiper_motor (frame) | 1238.537 | 831 | aim, 2.5 |  |
| drum_magnet_cap (drum) | launch_aim_housing (launch) | 662.037 | 831 | aim, 3.0 |  |
| frame_motor_adapter (frame) | reel_sprag_hub (reel) | 654.61 | 831 | load, 0.0 |  |
| launch_aim_conrod (launch) | launch_aim_housing (launch) | 570.682 | 286 | aim_sweep, None |  |
| drum_index_lever (drum) | frame_motor_adapter (frame) | 485.255 | 831 | aim, 5.0 |  |
| drum_shaft (drum) | launch_aim_housing (launch) | 254.05 | 831 | aim_fall, None |  |
| drum_gear_reel (drum) | reel_gc (reel) | 225.223 | 828 | drum_wiper, 0.0 |  |
| launch_rail_mid (launch) | launch_sleeve (launch) | 208.746 | 831 | aim_split_out, None |  |
| drum_shaft (drum) | frame_wiper_shaft (frame) | 99.656 | 831 | drum_index, 0.0 |  |
| drum_coupler_hub (drum) | frame_wiper_shaft (frame) | 97.197 | 831 | aim_sweep, None |  |
| drum_idler_gear (drum) | reel_gk (reel) | 82.343 | 798 | wrap, 29.5 |  |
| drum_coupler (drum) | reel_bus (reel) | 77.975 | 823 | aim_split_out, None |  |
| frame_motor_adapter (frame) | reel_sprag (reel) | 17.449 | 831 | load, 0.0 |  |
| frame_motor_leg (frame) | frame_cheek_r (frame) | 13.751 | 831 | load, 0.0 |  |

## Failed clearances (2)
- launch_aim_conrod / launch_aim_housing: gap 0.0 < 1.0 (conrod sweeps past the housing inner wall)
- reel_wall / reel_spool: gap 0.3 < 0.3 (reel spins in front of the wall)

## Floating or ungrounded parts (1)
- reel_shroud (reel): touches nothing in the rest state

## Static fails (6)
- launch_rail_rear (launch): no real flat face on the bed (108.5 mm2, need 150)
- launch_carriage (launch): no real flat face on the bed (108.0 mm2, need 150)
- launch_aim_lug (launch): no real flat face on the bed (50.1 mm2, need 87)
- launch_aim_housing (launch): 6 shells (printed parts must be one body); no real flat face on the bed (0.0 mm2, need 150)
- reel_shroud (reel): not watertight as STL (self-touching edges after vertex merge)
- reel_exit_post (reel): not watertight as STL (self-touching edges after vertex merge)

## Subsystem check failures (1)
- frame: layout: deck seam x = 51.2 only 11.9 mm from a deck hole

Method: exact manifold intersection of every part pair over the mission timeline plus each subsystem's audit_states; pairs that never change relative pose are checked once. Allowed contacts are not listed.
