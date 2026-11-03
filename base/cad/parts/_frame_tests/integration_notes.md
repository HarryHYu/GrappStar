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
