# 5. Base assembly

The assembly steps come from the base pipeline: every step is generated from the same model that passed the
collision audit, with a picture per step. They are in `base/out/base/guide.pdf` (attached as an appendix at the end of this guide).

@exists base/out/base/guide.pdf

@include base/guide/ASSEMBLY.md optional
@include base/out/assembly_steps.md optional
@include base/guide/README.md optional

General rules for the base:

- Heat-set inserts: soldering iron with the insert tip, about 220 C in PLA and about 280 C in the PPA-CF motor
  puck; press straight in, let it cool before loading.
- Bearings: press in square with a flat block, never hammer on the inner race.
- Keyed shafts: check the key slides in the sprag (CSK20PP) keyway by hand before final assembly.
- Turn every train by hand (battery unplugged) after each step: it must turn without binding before the next step.
- Leave the springs off until test stage 5 (section 12).
- Motor puck: bolt the wiper motor to the puck adapter on the bench first (M6 bolts through the puck slots from the
  front, step "Wiper motor on the puck"), then feed the puck into the motor plate window from behind and fit
  M3 x 12 from the front through the plate into the lip inserts (step "Puck into the plate" says which rim holes),
  before any shaft goes in.
- Drum coupler: coupler hub onto the wiper shaft (flats in line), the M8 nut on the thread, then the outer coupler
  over the hub and a printed shear pin through both (step "Main shaft: coupler and limiter").
- AS5600: push the board onto the 4 carrier pegs and clamp it with the two printed strips (M3 x 12 into nuts in the
  carrier back); no heat staking. Glue the magnet in its cap.
- Wires: leave a 150 mm loop at the rail pivot for SW_SEATED and SW_HOME. Never route a wire across a moving part.
- Contacts: set each gap by turning the screw (MEASURE_LIST G12).
- Manual drum fallback: loosen the cradle swing screw (M3, in the slot), swing the N20 out of mesh until `n`
  disappears from `sw`, turn the drum by hand by its gear rim, then swing the N20 back and tighten. Battery unplugged. Never turn the drum by hand while the carriage is drawn (`cock` above 0 or `SW_HOME` open) or while the line is under tension: shifting under load can dry fire the launcher or drop the line. De-cock and slacken the line first.
- Aim only with the carriage at HOME (springs slack). Fire low-pitch shots with the rover braked or chocked.
