# GrappStar base station firmware

Firmware `base-1.2.1` (the base-1.2 line) for the ESP32-S3 DevKitC-1-N8 in the base station. It follows BASE_SPEC v1.2,
`base/spec/params.py` 1.2.1 and the mission order of decision D22.

Nothing in this folder has run on real hardware yet. The logic passes the PC tests (a simulation). Go slowly the first
time and follow BENCH_TEST.md before the wiper motor goes into the base.

## What it does

One wiper motor (BTS7960 driver, AS5600 angle sensor) drives everything: launch, aim, take-up and reel. A small N20
motor (DRV8833 driver) turns the selector drum. The drum slides the dog collar and runs the wrap cup. Its order is
LAUNCH, NEUTRAL, WRAP, then the WRAPPED park, then REEL:

- LAUNCH (drum 0 deg): the collar drives the launcher.
- NEUTRAL (41 deg): the collar drives nothing. Forward wiper turns only drive the take-up (it winds slack onto the
  storage reel through a slip clutch).
- WRAP window (62 to 302 deg): the collar stays in NEUTRAL while the cup lays 7 turns of line on the capstan.
- WRAPPED park (314 deg): wraps on, cup locked, collar still in NEUTRAL (it only starts to move at 318). The take-up
  works here too.
- REEL (347 deg): the collar drives the capstan. REEL is only reached with the wraps on.

The drum is slow (about 8 deg per second): a wrap or an unwrap takes about 40 s.

Printed contacts (12 of them) tell the firmware where everything is.

| Wiper direction | LAUNCH | NEUTRAL, WRAP and the WRAPPED park | REEL |
|---|---|---|---|
| FORWARD (`dir` 1, RPWM) | cocks, fires and resets the launcher | take-up only, at most 40 % power | winds the capstan |
| BACKWARD (`dir` -1, LPWM) | aims (turns the rail pitch) | refused (`BACKSTOP`): it would fight the take-up bus one-way | refused (`BACKSTOP`): sprag #2 holds the capstan |

The Jetson sends commands over USB as JSON lines. The base answers every command, sends telemetry 10 times a second
(20 while something moves) and sends an event when an action ends. Full details: PROTOCOL.md.

A mission in short (D22): `mode LAUNCH`, `aim`, (`cock`), `arm`, `fire`, flight, `wrap` right after landing (straight
from LAUNCH, ends at the WRAPPED park), wait for the dart to anchor, `tidy` (take-up for a few seconds), `mode REEL`
(fly-in), tow (`reel` until tension), release, reel in, `unwrap` (back to NEUTRAL with the take-up running), final
approach (`retrieve`, until the dart is seated), `mode LAUNCH`.

## Files

| File | What |
|---|---|
| `grappstar_base/grappstar_base.ino` | the sketch: hardware glue (pins, USB, AS5600, HX711, flash). Open this one in the Arduino IDE |
| `grappstar_base/pins.h` | every GPIO pin |
| `grappstar_base/config.h` | defaults, contact list, drum sectors and parks, current trips |
| `grappstar_base/base_logic.h` | all the logic: commands, states, faults, telemetry |
| `grappstar_base/drives.h` | wiper and drum motor drive (soft start, reverse pause, current trip, stall) |
| `grappstar_base/sensors.h` | contact debounce, AS5600 angle tracking |
| `grappstar_base/persist.h` | what is saved in flash |
| `grappstar_base/json_lite.h` | small JSON reader and writer |
| `grappstar_base/params_gen.h` | GENERATED from `base/spec/params.py`. Do not edit |
| `tools/gen_params.py` | makes `params_gen.h` |
| `test/run_tests.sh` | builds and runs the PC tests |
| `PROTOCOL.md` | every command, reply, event, setting and fault code |
| `WIRING.md` | pin table, power, step by step build |
| `BENCH_TEST.md` | first checks on the bench, before the motor goes into the base |
| `CALIBRATION.md` | direction, current, drum, aim and tension calibration |

## Arduino IDE setup (Mac)

1. Install the Arduino IDE 2 from arduino.cc if you do not have it.
2. Arduino IDE > Settings. In "Additional boards manager URLs" add
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json` (skip if it is already there).
3. Tools > Board > Boards Manager. Search `esp32`. Install **esp32 by Espressif Systems**, version **3.3.12**.
   The sketch compiles with this version (checked with arduino-cli 1.5.1). 2.0.17 should also work (the sketch
   handles both) but was not compiled.
4. Tools > Manage Libraries. Search `HX711`. Install **HX711 by Rob Tillaart**, version **0.6.6** (not the one by
   bogde). You need it even without a load cell (the sketch includes it and finds out at boot whether one is fitted).
5. File > Open: `firmware/base/grappstar_base/grappstar_base.ino`. All the `.h` files open as tabs next to it.
6. In the Tools menu set exactly these:
   - Board "ESP32S3 Dev Module"
   - "USB CDC On Boot: Enabled" (the sketch refuses to compile otherwise, on purpose)
   - "USB Mode: Hardware CDC and JTAG"
   - Flash Size: 8MB (the N8 board has 8 MB)
   - Leave the rest at the defaults.
7. Plug a USB cable into the DevKitC port labelled **USB** (the native USB port). This is the same port the Jetson
   uses. The other port is labelled UART: do not use it for this firmware. Note: the old ground station used the UART
   port. This one does not.
8. Tools > Port: pick the one called something like `/dev/cu.usbmodem...`.
9. Click Upload (the arrow). If the upload cannot connect: hold the BOOT button, tap RESET, release BOOT, then Upload
   again. After an upload done this way, tap RESET once.

Coming from base-1.0: the saved bench settings (current scale, free current, N per amp and so on) are stored in a new
format in base-1.2. The old ones are ignored after the upload, so every bench value is back at its default. Enter
them again (CALIBRATION.md).

Command line instead of the IDE (same settings): `arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc
firmware/base/grappstar_base` from the repository folder. It should report about 398 kB of program (30 %) and about
35 kB of global variables (10 %). A compile only shows the code builds; it says nothing about the hardware.

## First contact

1. Battery unplugged. USB only. If the contacts are not wired yet, put two jumper wires on the board first:
   GPIO11 to GND (makes the drum code read NEUTRAL) and GPIO47 to GND (says the drum N20 is in mesh). WIRING.md
   section 6 explains why. Without the GPIO11 jumper you get a harmless `FAULT` `DRUM_SENSOR` "drum code 000" a moment
   after start-up.
2. Tools > Serial Monitor. At the bottom right set the line ending to **Newline**. The baud rate does not matter (USB).
3. Tap RESET on the board. You should see a `HELLO` line, then a `BOOT` line, then telemetry lines:

   ```
   {"evt":"HELLO","fw":"base-1.2.1","reset":"POWERON","hx711":false,"as5600":true,"pwm":true}
   {"evt":"BOOT","t":60,"fw":"base-1.2.1","record":false,"was_moving":false,"drum_known":true,"wraps_known":true,"aim_known":false,"phase":"UNKNOWN","angle":"AS5600"}
   {"tel":{"state":"IDLE","mode":"NEUTRAL","drum_deg":41,"drum_pos":"NEUTRAL",...,"sw":{"b":1,"n":1},"fault":null,"vbat":0}}
   ```

   The board waits only 1.5 s for the Serial Monitor at start-up, so you may miss `HELLO`. That is fine.
   With USB only, `vbat` reads about 0: that is normal. Motor commands are refused with `NO_POWER` until the battery
   is in.
4. Type this in the box at the top and press Enter:

   ```
   {"id":1,"cmd":"ping"}
   ```

   You should get `{"id":1,"ok":true,"fw":"base-1.2.1","t":...}` somewhere between the telemetry lines.
5. Telemetry scrolls fast. Untick "Autoscroll" to read a line, tick it again to follow.
6. Also try `{"id":2,"cmd":"status"}`. It shows `"magnet"`: `OK`, `WEAK`, `STRONG` or `NONE`.
7. Telemetry `sw` lists the closed contacts with one letter each: `a` `b` `c` (drum code), `l` (`SW_DOG_L`), `r`
   (`SW_DOG_R`), `n` (`SW_N20`), `H` (`SW_HOME`), `G` (`SW_GAP`), `L` (`SW_RAIL_LOW`), `S` (`SW_SEATED`), `W`
   (`SW_ARM`), `I` (`SW_INDEX`, only on a no-magnet build).

In the first 60 ms after a start-up motor commands, `set` and `cal` are refused with `BOOTING`: send again.

On the Jetson the same commands go through the GrappStar program instead of the Serial Monitor. Stop the service
first (`sudo systemctl stop grappstar`), then in one terminal `python3 -m grappstar base monitor` (shows telemetry and
events) and in a second terminal `python3 -m grappstar base cmd '{"id":1,"cmd":"ping"}'`.

Every command is one line of JSON with a `"cmd"`. The `"id"` is any whole number you like; the reply carries the same
id (a number like 1.5 gives `BAD_ID`). A few examples (all in PROTOCOL.md):

```
{"id":3,"cmd":"jog","motor":"wiper","dir":1,"ms":500}
{"id":4,"cmd":"stop"}
{"id":5,"cmd":"get","key":"amps_per_volt"}
{"id":6,"cmd":"set","key":"limit_n","val":50}
{"id":7,"cmd":"clear"}
```

A refused command replies with `"ok":false` and an `"err"` code, and nothing moves.

## PC tests

The logic is plain C++ and runs on a computer against a simulated base. On the Mac you need the command line tools
once: `xcode-select --install` (gives `g++` and `python3`). Then, from the repository folder:

```
sh firmware/base/test/run_tests.sh
```

It tests the params generator, then builds `test_base` (the mission and fault tests) and runs it, then the fuzz test
(random commands and faults, checking that nothing unsafe ever happens). Look for `ALL PASS` and
`fuzz: no invariant violated`.

## How spec numbers get into the firmware

The shared numbers live in `base/spec/params.py`. After that file changes, run:

```
python3 firmware/base/tools/gen_params.py
```

It rewrites `grappstar_base/params_gen.h`. Then upload again. Never edit `params_gen.h` by hand. If a value is
missing in `params.py`, the firmware uses its own default from `config.h`, so the build never breaks.

Bench values (current scale, free current, N per amp, tension offset, aim offset and so on) are not in `params.py`.
You set them over USB with `set` or learn them with `cal` (see CALIBRATION.md). They are saved in flash and kept over
a power cycle and over a new upload of the same firmware version.

## Safety summary

- Any fault: the wiper stops at once. Then, if the wraps are off and nothing below applies, the drum goes to NEUTRAL,
  so the wiper turns nothing.
- The drum does NOT move, and holds where it is, when:
  - the wraps are on (wrap window, WRAPPED park, REEL): the worm and sprag #2 hold the line, and the operator
    unwraps when it is safe (`unwrap`). This applies to `estop` too;
  - the launch spring may be drawn: there is no sear, only the wiper worm holds the spring, and a shift under that
    load could dry fire. Recover with `decock`;
  - the drum itself faulted, its position or the wraps are unknown, or the N20 is swung out of mesh (hand mode). After a
    drum code that changed with the drum still near the wrap window, only `unwrap` is accepted until NEUTRAL.

  It then sends a `MODE` event with `"err":"DRUM_HELD"` and a `"why"` that says which (exactly at the WRAPPED park
  it says `"ok":true` instead).
- After a power cut the base never moves by itself. Inside the wrap window it stops and reports. A cut while cocking or
  firing is treated as a drawn spring: `cock` is refused (`LOADED_UNKNOWN`) until a `decock` (PROTOCOL.md section 9).
- `stop`: wiper off now, drum stops. `estop`: the same, then the drum goes to NEUTRAL when that is safe (never with
  the wraps on).
- Before every shift out of LAUNCH or REEL the wiper makes a small centering jog (8 deg) so the collar is not loaded.
  The collar contacts `SW_DOG_L` and `SW_DOG_R` are checked during every drum move.
- Soft start: the wiper ramps up over 300 ms (inrush once blew a 10 A fuse). It always stops for 200 ms before it
  changes direction.
- Current trips per job (each for 30 ms, ignored for 50 ms after a start): aim and reel in 5 A, cock and fire 6.5 A,
  tow 6.5 A or more for a high `max_n` (the tension estimate is the real tow limit), take-up 2.5 A, jogs, centering and fly-in 3 A; 17 A at any
  time. Stall: the shaft slower than 12 deg/s for 200 ms while driving.
- AS5600 lost while the wiper runs, or its angle jumps so travel is lost: fault `ANGLE_SENSOR`. If the AS5600 is not reading, wiper actions and the drum
  moves that need the centering jog or the fly-in are refused (`ANGLE_SENSOR`). Exception: a build with no magnet can
  switch on the 6-bump index cam `SW_INDEX` in `config.h` (decision D12, off by default); then the base runs on that
  coarser angle (PROTOCOL.md section 9).
- Battery below 9.9 V for 300 ms while a motor runs: fault `BROWNOUT`.
- Fire needs a fresh 6-digit token from `arm`, valid 3 s, used once, plus the dart seated. `cock` also needs the dart
  seated.
- Both worm gears self-lock, so the mechanism stays where it stopped when the power goes. The position is saved in
  flash, and after a power cut mid-motion the base only trusts what its sensors can prove.
- After a fault, fix the cause, then send `{"id":1,"cmd":"clear"}`.

And the rule that matters most: **battery unplugged whenever a wire or plug moves.** Plug the battery in last, and
never pull a plug while the battery is in.
