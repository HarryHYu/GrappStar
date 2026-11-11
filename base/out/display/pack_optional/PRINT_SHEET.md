# Print sheet

Generated 2026-11-11 00:10. Bed 256 x 256 x 260 mm (Bambu X2D). Masses and times are rough estimates: walls and infill per material from the settings below, volume at a typical print rate plus 2.14286 s per 0.2 mm layer, the layer time counted once per plate.

## Parts

| part | material | qty | print bbox mm | volume cm3 | mass g (each) | time h (each, alone) | overhang mm2 | supports |
|---|---|---|---|---|---|---|---|---|
| opt_reel_bracket | PLA | 1 | 193.5 x 40.0 x 50.0 | 64.3 | 31.3 | 0.62 | 21 | no |
| opt_drum_dummy | PLA | 1 | 96.0 x 96.0 x 128.0 | 44.7 | 55.5 | 1.21 | 0 | no |
| opt_drum_saddle | PLA | 1 | 44.0 x 82.0 x 32.4 | 37.5 | 16.7 | 0.35 | 0 | no |
| opt_reel_spool | PLA | 1 | 62.0 x 62.0 x 38.0 | 15.1 | 18.8 | 0.39 | 2814 | yes, tree supports from the plate (2814 mm2 over 45 deg) |

- **opt_reel_bracket**: Rotated: the part's -x axis points up. OPTIONAL reel bracket (glue the spool to its face)
- **opt_drum_dummy**: As modelled, bottom face on the bed. OPTIONAL selector drum look-alike (hollow), gear end down; sits on the drum saddle
- **opt_drum_saddle**: As modelled, bottom face on the bed. OPTIONAL saddle for the drum look-alike
- **opt_reel_spool**: Rotated: the part's +y axis points up. OPTIONAL storage reel look-alike (hollow spool)

## Plates

| plate | material | objects | mass g | time h |
|---|---|---|---|---|
| plate_1_PLA.3mf | PLA | 4: opt_drum_dummy, opt_drum_saddle, opt_reel_bracket, opt_reel_spool | 122 | 2.2 |

## Totals on the plates

- PLA: 4 objects on 1 plates, 122 g, about 41.0 m of 1.75 mm filament, 2.2 h

## Recommended settings

### PLA
- 0.4 mm nozzle, 0.2 mm layers
- 4 walls, 5 top and 5 bottom layers
- 40 % gyroid infill
- Nozzle 220 C, bed 55 to 60 C, textured PEI plate
- Door open or top vented

