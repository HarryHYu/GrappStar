# 5. Base assembly

The assembly steps come from the base pipeline: every step is generated from the same model that passed the
collision audit, with a picture per step. They are in `base/out/base/guide.pdf` (attached as an appendix at the end of this guide).

@exists base/out/base/guide.pdf

@include base/guide/ASSEMBLY.md optional
@include base/out/assembly_steps.md optional
@include base/guide/README.md optional

General rules for the base:

- Heat-set inserts: soldering iron at about 220 C with the insert tip, press straight in, let it cool before loading.
- Bearings: press in square with a flat block, never hammer on the inner race.
- Keyed shafts: check the key slides in the sprag (CSK20PP) keyway by hand before final assembly.
- Turn every train by hand (battery unplugged) after each step: it must turn without binding before the next step.
- Leave the springs off until test stage 5 (section 12).
- Motor adapter: bolt the wiper motor to the adapter on the bench first (M6 bolts from the cheek side), then from the
  frame side of the plate fit M3 x 10 through the cheek plate into every insert in the adapter hub (10 of the 12
  holes line up; the frame step "Motor onto the cheek" lists them), before any shaft goes in.
- Bearing blocks slide on over bearings that are already on their shaft.
- Drum coupler: drop the M8 nut through the socket, then the M3 cross screw.
- AS5600: heat-stake the board onto its printed pegs (soldering iron tip, gently); glue the magnet in its cap.
- Wires: leave a 150 mm loop at the rail pivot for SW_SEATED and SW_HOME. Never route a wire across a moving part.
- Contacts: set each gap by turning the screw (MEASURE_LIST G12).
- Manual drum fallback: loosen the cradle swing screw (M3, in the slot), swing the N20 out of mesh until `n`
  disappears from `sw`, turn the drum by the hand lever on its 14 mm hex boss, then swing the N20 back and tighten. Battery unplugged. Never use the hand lever while the carriage is drawn (`cock` above 0 or `SW_HOME` open) or while the line is under tension: shifting under load can dry fire the launcher or drop the line. De-cock and slacken the line first.
- Aim only with the carriage at HOME (springs slack). Fire low-pitch shots with the rover braked or chocked.
