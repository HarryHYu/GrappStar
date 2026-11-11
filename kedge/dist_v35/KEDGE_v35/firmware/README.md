# KEDGE dart firmware v4.3 (dart v34)

Firmware for the KEDGE dart: Seeed XIAO ESP32S3, two DRV8833 drivers, INA219 current sensor, BNO085 IMU. After
landing it rights the dart if it is not belly down, anchors it, releases it again, records the flight, and takes
commands over WiFi.

v4.3 (9 Nov 2026) is for the v34 dart: three motor channels. DRV#1 A = pods (deploy), DRV#1 B = the righting leg (the
old kick channel and N20), DRV#2 A = both drills. No foot and no kick leg: DRV#2 B and D6 / D7 are not wired, keys
`4` / `$` are refused. `/auto`: righting (belly up: the leg flips it end over end; on a flank: both pods roll it with
the drills off; the policy of the study sim, kedge/sim_righting), then pods down with the drills spinning. Details:
API.md "/auto" and "Changes from v4.2 to v4.3".

| File | What |
|---|---|
| `kedge_dart/kedge_dart.ino` | the sketch (hardware: pins, PWM, I2C, WiFi, web page, HTTP API) |
| `kedge_dart/kedge_logic.h` | all control logic (sequences, safety, flight detection, flight log), plain C++ |
| `test/test_logic.cpp`, `test/run_tests.sh` | PC tests of the logic against a motor model and synthetic IMU traces |
| `API.md` | the HTTP API for the Jetson (contract C2) |
| `WIRING.md`, `wiring/` | wire cut list, change list from v29, diagram |
| `BENCH_TEST.md` | how to check every motor and sensor before closing the hull |
| `tools/sync_kedge_fw.sh` | copies this firmware into `kedge/fw/` (that folder is a synced copy; run_tests.sh checks it) |
| `tools/wires_v4.py` | regenerates `wiring/` (run from the repo root; needs python3 with numpy, graphviz `dot`, and `kedge/out30`. Another graphviz version gives harmless image differences) |
| `sim/` | host simulator: the real sketch compiled on Linux with stub Arduino headers, serving the real HTTP API (`make && python3 test_sim.py`) |
| `REPORT.md` | build-day report |

## Arduino IDE setup (once)

1. Install Arduino IDE 2 (arduino.cc).
2. File > Preferences > Additional boards manager URLs, add:
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
3. Tools > Board > Boards Manager: search `esp32`, install **esp32 by Espressif Systems** (3.3.x tested; 2.0.17 also
   compiles).
4. Sketch > Include Library > Manage Libraries: install **Adafruit BNO08x** (say yes to its dependencies, Adafruit
   BusIO). The INA219 is read directly, so the Adafruit INA219 library is no longer needed (having it installed does no
   harm).
5. Open `firmware/dart/kedge_dart/kedge_dart.ino` (the folder name must stay `kedge_dart`; `kedge_logic.h` sits next to
   it and opens as a second tab).

## Tested versions (`arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3 --warnings all`)

v4.3, 9 Nov 2026: esp32 core 3.3.12 (XIAO_ESP32S3, USB CDC On Boot Enabled) compiles with no warnings from the sketch:
1,025,367 B flash (30 %), 70,852 B RAM (21 %). v4.1, 7 Nov 2026: esp32 core 3.3.12 and 2.0.17 compile with no warnings
from the sketch (Adafruit BNO08x 1.2.7, arduino-cli 1.5.2-rc.1). The table below is v4 (3 Nov).

| esp32 core | Result | Flash | RAM |
|---|---|---|---|
| 3.3.12 (latest) | OK, no warnings from the sketch | 1,014,379 B (30 %) | 70,500 B (21 %) |
| 3.3.2 | OK, no warnings from the sketch | 1,018,875 B (30 %) | 68,512 B (20 %) |
| 2.0.17 | OK, no warnings from the sketch | 805,405 B (24 %) | 66,968 B (20 %) |

Libraries: Adafruit BNO08x 1.2.7, plus Adafruit BusIO 1.17.4 and Adafruit Unified Sensor 1.1.15 (installed with it).
arduino-cli 1.5.2-rc.1.
Note for arduino-cli users: on this board the menu value for "USB CDC On Boot: Enabled" is `CDCOnBoot=default` and
`CDCOnBoot=cdc` means Disabled (the sketch then refuses to compile, on purpose).

## Board settings

- Tools > Board > esp32 > **XIAO_ESP32S3**.
- Tools > **USB CDC On Boot: Enabled** (the default for this board). If it is Disabled, the sketch will not compile.
- Leave the rest at the defaults.

## Upload over USB

1. **Battery unplugged** (power loop plug out). Never have the battery plug and USB in together while you work.
2. Plug the XIAO into the Mac with a USB-C data cable.
3. Tools > Port: pick the one that appears (Mac `/dev/cu.usbmodem...`, Linux `/dev/ttyACM0`, Windows `COM3` or similar).
4. Click Upload (arrow). Wait for "Hard resetting via RTS pin" / "Done uploading".
   If the port does not appear or the upload fails: hold the XIAO's **B** (boot) button, tap **R** (reset), release
   B, then upload again. After such an upload, tap R once.
5. Tools > Serial Monitor, 115200 baud, "No line ending". You should see `KEDGE v4.2 ready` and the key list.
   Type `z` and press Enter: expect `I2C device at 0x40` (INA219) and `0x4a` (BNO085; some boards answer at `0x4b`, also fine).

## Upload over WiFi (after the first USB upload)

USB unplugged, power loop plug in. Join WiFi `KEDGE` (password `grappstar`). Tools > Port > network port `kedge`. Upload. Motors stop, any sequence is
aborted and the dart disarms when the upload starts.

## Driving it

- Web page: join `KEDGE`, open `http://192.168.4.1`. Hold a button or key to run, release to stop.
- Serial Monitor: type a key and press Enter, each press runs 1.5 s.
- v4.1 page buttons `video on` / `video off` (optional switch, WIRING.md); third status line: battery, orientation,
  video.
- Keys: `1`/`!` pods down/up, `2`/`@` righting leg out (over the nose)/back (fold), `3`/`#` drills in/out (`4`/`$`
  refused: no foot on v34), space or `s` STOP,
  `a` auto-anchor, `r` release, `A` arm/disarm (auto-anchor on landing), `+`/`-` speed, `i` current, `b` attitude,
  `p` status, `f` flight log summary, `z` I2C scan, `h` help.
- Jetson: see `API.md`.

## Safety built in

- All motors are off from the first lines of `setup()`. If the chip resets, the DRV8833 inputs fall low and the
  motors coast.
- Every manual run has a deadman (350 ms on WiFi, 1.5 s per Serial key) and a total hold cap (8 s, 2 s without the
  INA219). A motor driven into an end stop stops on stall.
- WiFi client gone: manual motors stop at once. A running anchor or release sequence carries on (it is autonomous and
  has its own limits).
- Over-current trip 2000 mA, low rail (under 5.6 V while driving), sequence time limits, jam detection in release.
- All safety runs in its own task every 5 ms with a 2 s watchdog; a stuck web client or Serial port cannot block it.
- After a brownout or watchdog reset the dart boots with motors off and reports `FAULT` until `/stop`, `/auto`, `/release` or `/arm`.
- INA219 missing: manual keys only (on timers), auto and release refused. BNO085 missing: no arming, no flight log.
- A stuck I2C bus is recovered (9 clocks and a Wire restart). If the web loop hangs for 30 s while nothing moves, the
  dart is not armed or in flight, and no OTA upload runs, it restarts itself.
- v4.1: `/auto` refuses when the battery is critical (6.4 V at rest; `/release` always runs). In soil the pods are
  milled down in pulses (up to 150 s; loam about 90 s). Released drills that stick are rocked before `DRILL_JAM`.
- v4.3: a dart on its side or back is righted first (one motor at a time, drills off, 6 moves at most, then fault
  `NOT_UPRIGHT`); the leg's far stop in air is normal; the pods never deploy with the leg out.
- v4.2: the deploy guard stops the pods when the drill pair reaches 67 % of its stall (`depth_limited`): the main
  safeguard that the sleeves can still be backed out in firmer loam. v32 drives the sleeves directly (`DRILL_SIGN` +1).
- **Power on with the leg and the pods folded** (drive them to stall toward fold with `@` and `!` before loading the
  dart). The firmware counts their travel from power-on to tell a jam from the fold stop.

## Tuning

The current thresholds in `kedge_logic.h` (`struct Tuning`) are from the v29 bench numbers at 8 V. Measure the real
ones with BENCH_TEST.md steps 3 to 5 and write them in at step 10. Then run the PC tests.

## PC tests

```
cd firmware/dart/test && ./run_tests.sh
```

Needs g++ and python3 (Mac: run `xcode-select --install` once for g++). Prints every check and `ALL PASS`, runs the
fuzz test, then checks the JSON files the tests wrote (into `test/out/`).

If `make` is found it also runs the whole simulator suite below (about 2 minutes). That part needs Linux, make,
node 18 or newer and Playwright with Chromium (`npm i playwright && npx playwright install chromium`). Without node the
browser check is skipped; node without Playwright makes the run fail. `./run_tests.sh --quick` runs only the logic
and fuzz checks (seconds, any OS).

## Simulator (Linux, optional)

Needs Linux, g++, make, pthread, python3, and for the page check node 18+ with Playwright and Chromium
(`npm i playwright && npx playwright install chromium`).

```
cd firmware/dart/sim && make && python3 test_sim.py      # 15 HTTP tests against the real sketch
python3 test_jetson_client.py                            # 10 tests: the Jetson's own dart client against the real sketch (C2)
node test_page.js                                        # 12 checks of the web page in headless Chromium (one logged 409 is expected)
./kedge_sim --port 8790                                  # then open http://localhost:8790 (the real web page)
```

`/sim/launch`, `/sim/state` and `/sim/set?k=v` drive the simulated hardware (see `sim/sim_main.cpp`).

## Compile check without the IDE (optional)

Run from the repo root:

```
arduino-cli core install esp32:esp32 --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli lib install "Adafruit BNO08x"
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3 --warnings all firmware/dart/kedge_dart
```
