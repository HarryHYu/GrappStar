# GrappStar base station firmware

Firmware `base-1.0` for the ESP32-S3 DevKitC-1-N8 in the base station.

Nothing in this folder has run on real hardware yet. The logic passes the PC tests (a simulation). Go slowly the first
time and follow BENCH_TEST.md before the wiper motor goes into the base.

## What it does

One wiper motor (BTS7960 driver, AS5600 angle sensor) drives everything: launch, aim and reel. A small N20 motor
(DRV8833 driver) turns the selector drum, which slides the dog collar between LAUNCH, NEUTRAL and REEL, and then lays
the wraps on the capstan. Printed contact switches tell the firmware where things are.

The Jetson sends commands over USB as JSON lines. The base answers every command, sends telemetry 10 times a second
(20 while something moves) and sends an event when an action ends. Full details: PROTOCOL.md.

| Wiper direction | LAUNCH mode | REEL mode |
|---|---|---|
| FORWARD (`dir` 1, RPWM) | cocks, fires and resets the launcher | winds the capstan |
| BACKWARD (`dir` -1, LPWM) | aims (turns the rail pitch) | refused (`BACKSTOP`): sprag #2 holds the capstan |

In NEUTRAL the wiper turns nothing.

## Files

| File | What |
|---|---|
| `grappstar_base/grappstar_base.ino` | the sketch: hardware glue (pins, USB, AS5600, HX711, flash). Open this one in the Arduino IDE |
| `grappstar_base/pins.h` | every GPIO pin |
| `grappstar_base/config.h` | defaults, contact list, which contact is on which pin |
| `grappstar_base/base_logic.h` | all the logic: commands, states, faults, telemetry |
| `grappstar_base/drives.h` | wiper and drum motor drive (soft start, reverse pause, current) |
| `grappstar_base/sensors.h` | contact debounce, AS5600 angle tracking |
| `grappstar_base/persist.h` | what is saved in flash |
| `grappstar_base/json_lite.h` | small JSON reader and writer |
| `grappstar_base/params_gen.h` | GENERATED from `base/spec/params.py`. Do not edit |
| `tools/gen_params.py` | makes `params_gen.h` |
| `test/run_tests.sh` | builds and runs the PC tests |
| `PROTOCOL.md` | every command, reply, event, setting and fault code |
| `WIRING.md` | pin table, power, step by step build |
| `BENCH_TEST.md` | first checks on the bench, before the motor goes into the base |
| `CALIBRATION.md` | current, drum, aim and tension calibration |

## Arduino IDE setup (Mac)

1. Install the Arduino IDE 2 from arduino.cc if you do not have it.
2. Arduino IDE > Settings. In "Additional boards manager URLs" add
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json` (skip if it is already there).
3. Tools > Board > Boards Manager. Search `esp32`. Install **esp32 by Espressif Systems**, version **3.3.12**.
   The sketch was compiled with 3.3.12. Version 2.0.17 should also work (the sketch handles both) but is untested.
4. Tools > Manage Libraries. Search `HX711`. Install **HX711 by Rob Tillaart**. You need it even without a load cell
   (the sketch includes it and finds out at boot whether one is fitted).
5. File > Open: `firmware/base/grappstar_base/grappstar_base.ino`. All the `.h` files open as tabs next to it.
6. Tools menu, set:
   - Board: **ESP32S3 Dev Module**
   - USB CDC On Boot: **Enabled** (the sketch refuses to compile otherwise, on purpose)
   - USB Mode: **Hardware CDC and JTAG**
   - Flash Size: 8MB (the N8 board has 8 MB)
   - Leave the rest at the defaults.
7. Plug a USB cable into the DevKitC port labelled **USB** (the native USB port). This is the same port the Jetson
   uses. The other port is labelled UART: do not use it for this firmware. Note: the old ground station used the UART
   port. This one does not.
8. Tools > Port: pick the one called something like `/dev/cu.usbmodem...`.
9. Click Upload (the arrow). If the upload cannot connect: hold the BOOT button, tap RESET, release BOOT, then Upload
   again. After an upload done this way, tap RESET once.

## First contact

1. Battery unplugged. USB only. If the drum code contacts are not wired yet, put a jumper from GPIO12 to GND first
   (it makes the drum code read NEUTRAL; WIRING.md section 5). Without it you get a harmless `FAULT` `DRUM_SENSOR`
   "drum code 000" a moment after start-up.
2. Tools > Serial Monitor. At the bottom right set the line ending to **Newline**. The baud rate does not matter (USB).
3. Tap RESET on the board. You should see a `HELLO` line, then a `BOOT` line, then telemetry lines:

   ```
   {"evt":"HELLO","fw":"base-1.0","reset":"POWERON","hx711":false,"as5600":true}
   {"evt":"BOOT","t":61,"fw":"base-1.0","record":false,"was_moving":false,"drum_known":true,"wraps_known":true,"aim_known":false,"phase":"UNKNOWN"}
   {"tel":{"state":"IDLE","mode":"NEUTRAL","drum_deg":...,"drum_pos":"NEUTRAL",...}}
   ```

   The board waits only 1.5 s for the Serial Monitor at start-up, so you may miss `HELLO`. That is fine.
4. Type this in the box at the top and press Enter:

   ```
   {"id":1,"cmd":"ping"}
   ```

   You should get `{"id":1,"ok":true,"fw":"base-1.0","t":...}` somewhere between the telemetry lines.
5. Telemetry scrolls fast. Untick "Autoscroll" to read a line, tick it again to follow.
6. Also try `{"id":2,"cmd":"status"}`. It shows `"magnet"`: `OK`, `WEAK`, `STRONG` or `NONE`.

Every command is one line of JSON with a `"cmd"`. The `"id"` is any number you like; the reply carries the same id.
A few examples (all in PROTOCOL.md):

```
{"id":3,"cmd":"jog","motor":"wiper","dir":1,"ms":500}
{"id":4,"cmd":"stop"}
{"id":5,"cmd":"get","key":"amps_per_volt"}
{"id":6,"cmd":"set","key":"limit_n","val":20}
{"id":7,"cmd":"clear"}
```

A refused command replies with `"ok":false` and an `"err"` code, and nothing moves.

## PC tests

The logic is plain C++ and runs on a computer against a simulated base. On the Mac you need the command line tools
once: `xcode-select --install` (gives `g++` and `python3`). Then, from the repository folder:

```
sh firmware/base/test/run_tests.sh
```

It tests the params generator, then builds `test_base` and runs it. The last line must say `ALL PASS`.

## How spec numbers get into the firmware

The shared numbers live in `base/spec/params.py`. After that file changes, run:

```
python3 firmware/base/tools/gen_params.py
```

It rewrites `grappstar_base/params_gen.h`. Then upload again. Never edit `params_gen.h` by hand. If a value is
missing in `params.py`, the firmware uses its own default from `config.h`, so the build never breaks.

Bench values (current scale, free current, N per amp, aim offset and so on) are not in `params.py`. You set them over
USB with `set` or learn them with `cal` (see CALIBRATION.md). They are saved in flash and kept over a power cycle and
over a new upload.

## Safety summary

- Any fault: the wiper stops at once. Then the drum goes to NEUTRAL when that is safe, so the wiper turns nothing.
- The drum does NOT go to NEUTRAL, and holds where it is, when:
  - the launch spring is drawn (LAUNCH with the shot phase not at rest): only the wiper worm holds the spring, and a
    shift under that load could dry fire;
  - wraps are on the capstan: the capstan holds the line;
  - the drum itself faulted, or its position or the wraps are unknown.

  It then sends a `MODE` event with `"err":"DRUM_HELD"` and a `"why"` that says which.
- `stop`: wiper off now, drum stops. `estop`: the same, then the drum goes to NEUTRAL when safe.
- Soft start: the wiper ramps up over 300 ms (inrush once blew a 10 A fuse). It always stops for 200 ms before it
  changes direction.
- Over-current trip 17 A (15 A while cocking). Stall: shaft slower than 15 deg/s for 400 ms while driving hard.
- AS5600 lost while the wiper runs: fault `ANGLE_SENSOR`. Without the AS5600 the wiper does not start.
- Battery below 9.9 V for 300 ms while a motor runs: fault `BROWNOUT`.
- Fire needs a fresh 6-digit token from `arm`, valid 3 s, used once.
- Both worm gears self-lock, so the mechanism stays where it stopped when the power goes. The position is saved in
  flash, and after a power cut mid-motion the base only trusts what its sensors can prove.
- After a fault, fix the cause, then send `{"id":1,"cmd":"clear"}`.

And the rule that matters most: **battery unplugged whenever a wire moves.** Plug the battery in last.
