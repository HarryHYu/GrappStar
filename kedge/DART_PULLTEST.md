# KEDGE soil pull test and bench torque tests

Script: `kedge/dart_pulltest.py`. Predictions it compares against: `kedge/out31/phys31/anchor.json` (from
`dart_anchor.py`). Measured motor torques go to `kedge/bench31.json`, which the physics reads instead of its
assumptions; rerun `python dart_phys31.py` after any bench test.

## What you need

- Luggage scale (0 to 50 kg) or a load cell with an HX711 on any USB serial board (prints one force per line).
- Tape measure, a tent peg, a 1 m loop of the same Dyneema line, gloves.
- The dart charged, WiFi on (the script reads its pitch and roll to see the moment it starts to move).
- Optional: the rover, to pull from the real line height (0.41 m) and distance.

## A. Bench tests first (10 minutes, no soil)

Hold the motor in a vice by its body (not the gearbox). Fit a lever to the output: a strip of wood or a printed arm
with a hole at a known length L from the shaft centre (50 mm is good). Hook the luggage scale in the hole and pull
at right angles to the lever.

| Test | Power | Do | Command |
|---|---|---|---|
| B1 worm stall (kick or deploy N20) | 2S pack, full duty | Pull until the motor stops turning. Read the scale. 3 tries | `python3 dart_pulltest.py torque --what worm_stall` |
| B2 worm holding strength | OFF | Pull the lever backwards against the self-locking worm, slowly, until it slips, clicks or the shaft twists. Stop at 2 kg on a 50 mm lever (1 N m) if nothing gives | `... --what worm_backdrive` |
| B3 drill motor stall | 2S, full duty | as B1 | `... --what drill_stall` |
| B4 foot motor stall | 2S, full duty | as B1 | `... --what foot_stall` |

B2 matters most: under tow the pods push back through the deploy worm, and in loam the holding is limited by it
(assumed 0.6 N m). Use a spare worm motor for B2 if you have one; it may be damaged at the limit.

## B. Soil pull test (about 10 minutes per pull)

1. Pick flat ground. Note the soil: sand, loam or clay, and the squeeze test (dry crumbles, moist holds, wet smears).
2. Lay the dart belly down, tail toward where you will pull from. Run the anchor sequence (`a`) and note the time it took and
   whether it said anchored, any stall warnings, and the peak drill current.
3. Push a peg into the ground right behind the tail, touching it. Movement is measured from this peg.
4. Tie the loop to the tail line. Pull from a point at the real line height and distance if you can (the rover pusher is
   0.41 m up; 3 m away gives a 7 deg line). By hand: crouch so the line runs at about that angle.
5. Run `python3 dart_pulltest.py pull --soil "moist loam" --rover 3 --manual` (or `--serial /dev/ttyUSB0` with the HX711).
6. At each step (5 N, 10 N, 15 N ...): bring the force up slowly (2 to 3 s), hold it 10 s, press Enter, type the tape
   reading. The script stops when the tail moved more than 15 mm or the dart pitched more than 5 deg.
7. Stop at 50 N (the reel tow cap is 55 N at the dart).
8. Release: run the release sequence, time it, note if the drills stalled while unscrewing.
9. Three pulls per soil, each in fresh ground at least 30 cm from the last holes.

## C. What to look at

- The plot `pull_<soil>_<time>.png`: measured force against movement, with the predicted design (dashed), upper bound
  (dotted) and without-foot (dash-dot) lines.
- The holding to report is the force at 5 mm of tail movement (in the summary JSON). The design value should sit at or
  below it; if the measured value is under the design value, the soil numbers in `dart_model.py SOIL` are too high for that
  ground: lower them (cohesion first) and rerun.
- If the dart pitches nose up before it slides, the foot is too weak (or the kick worm slipped). If it rotates about the
  drills with the pods folding back, the deploy worm slipped (compare with B2).
- Tow advice for the rover: first tow at no more than the measured 5 mm force divided by 1.5, raised in 5 N steps.

## D. HX711 sketch (any ESP32 or Arduino with the HX711 library by Bogde)

```
#include "HX711.h"
HX711 s;
const float CAL = 0.0;      // counts per newton: hang a known mass, set CAL = reading / (kg x 9.81)
void setup() { Serial.begin(115200); s.begin(4, 5); s.tare(); }    // DOUT pin 4, SCK pin 5
void loop() { if (CAL != 0) Serial.println(s.get_units(1) / CAL, 2); else Serial.println(s.read()); delay(50); }
```
