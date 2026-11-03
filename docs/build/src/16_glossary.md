# 16. Glossary

| Word | Meaning here |
|---|---|
| 3S, 2S | Battery cells in series: 3S LiPo = 3 cells, 9.9 to 12.6 V in use; 2S 18350 = 2 cells, 6.0 to 8.4 V |
| AS5600 | Magnetic angle sensor: reads a magnet on the end of the main shaft to know the wiper angle |
| AWG | Wire size: smaller number = thicker wire. 14 AWG for the wiper, 22 to 28 AWG for signals |
| BASE_SPEC, params.py | The base design description and the file with every shared number (`base/spec/`) |
| BTS7960 | The big motor driver board that runs the wiper motor in both directions |
| Buck (MP1584) | Small board that turns a higher voltage into a lower one, set with its brass screw |
| Capstan | A drum the line wraps round several times; friction from the wraps lets it pull hard |
| Cock / de-cock | Draw the launch springs back / let them back down slowly without firing |
| CSK20PP, sprag | One-way bearing: turns freely one way, locks the other |
| D9, D12, D27 ... | Numbered design decisions in `base/spec/DECISIONS.md`; they override older notes |
| Deadman | A move that stops by itself unless the same command is sent again in time (the `jog` command) |
| Dog clutch, collar | A sliding ring with teeth that locks the main shaft to one gear or another |
| Drum (selector drum) | The N20-driven barrel with grooves that moves the dog collar and lays the wraps |
| DRV8833 | Small motor driver board for the N20 motors |
| estop | Emergency stop command: wiper off, drum to a safe position |
| Fly-in | Entering REEL with the wiper creeping forward so the dog teeth find their slots |
| Fuse | Thin link that melts and cuts the power if the current is far too high |
| Gray code | The three drum contacts read as a 3-digit code that changes one digit at a time |
| Heat-set insert | Brass M3 thread pressed into a printed hole with a hot soldering iron |
| IS | The BTS7960's current-sense output, read by the ESP32 to know the wiper current |
| JSON line | One command or reply as text in curly brackets, for example `{"cmd":"ping"}` |
| N20 | Small 12 mm gearmotor. "Worm" N20s are right-angle and hold their position unpowered |
| PPA-CF | Strong carbon-filled nylon filament, for loaded gears and keyed shafts |
| Printed contact | A switch made from an M3 screw head touching an M3 nut |
| pty | A pretend serial port on the Jetson, used by the fake base for tests |
| Pull-down / pull-up | A resistor that holds a pin low (or high) when nothing else drives it |
| Shear pin | A printed pin that breaks on purpose to protect the gears from the wiper's full torque |
| Slip clutch | Friction clutch that slips above a set force: the payout brake and the take-up |
| STEMMA QT | Adafruit's small 4-pin plug and cable (JST SH) for I2C sensors such as the AS5600 |
| Telemetry | The status line the base sends 10 times a second (`tel`) |
| Tether | The Dyneema line between the base and the dart |
| VTX | The dart's video transmitter (5.8 GHz). Never power it without its antenna |
| XT60 | The yellow battery plug used on the LiPo and its leads |
