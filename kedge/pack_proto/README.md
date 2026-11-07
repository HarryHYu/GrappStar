# Prototype drill bits (test only)

Each bit screws onto the drill gearmotor's M3 threaded shaft, which runs up the middle as a steel spine. They are tapered
to a sharp point and use an anchor flight: the face toward the motor takes the pull-out load and is 45 deg, so it prints
without supports.

| File | Flight diameter | Pitch | Use |
|---|---|---|---|
| bit_T16_RH.stl | 16 | 12 | main candidate |
| bit_T16_LH.stl | 16 | 12 | its mirror (left-hand), for the other pod |
| bit_T12_RH.stl | 12 | 9 | slim: least torque |
| bit_T20_RH.stl | 20 | 15 | wide: most holding |
| drill_grip.stl | | | T handle that holds the gearmotor, so you can drill by hand |

Print: PLA, 0.16 mm layers, 100 % infill (or 6 walls), bit standing on its base with the tip up, no supports. The grip
prints standing on its T-bar, also without supports.

Fit:
1. Press an M3 nut into the hex pocket in the bit's base.
2. Spin one more M3 nut onto the motor shaft, all the way down to the motor face.
3. Screw the bit onto the shaft until it is fully on (the shaft cuts its own thread in the 2.6 mm bore).
4. Turn the shaft nut back up against the bit's base and tighten it (a drop of threadlocker helps). The bit is now locked
   in both directions.

Test (battery unplugged while you wire; bench supply at 8 V through the INA219 if you have it):
1. Push the gearmotor into the grip, then run the leads out of the slot.
2. Right-hand bits drill in when they turn clockwise, seen from the motor looking at the tip. If one goes the wrong way,
   swap its two leads.
3. Drill into a pot of damp garden soil and then dry sand. Note how long each bit takes to go in fully, and the current
   (or whether it stalls).
4. Hook a luggage scale onto the grip and pull straight up slowly. Note the peak force. Then drive the motor in reverse
   and note whether it backs out or stalls.
5. Send the numbers back: they set the final drill size.
