# KEDGE dart firmware v4

Firmware for the KEDGE dart: Seeed XIAO ESP32S3, two DRV8833 drivers (four motor channels), INA219 current sensor,
BNO085 IMU. It anchors the dart after landing, releases it again, records the flight, and takes commands over WiFi.

| File | What |
|---|---|
| `kedge_dart/kedge_dart.ino` | the sketch (hardware: pins, PWM, I2C, WiFi, web page, HTTP API) |
| `kedge_dart/kedge_logic.h` | all control logic (sequences, safety, flight detection, flight log), plain C++ |
| `test/test_logic.cpp`, `test/run_tests.sh` | PC tests of the logic against a motor model and synthetic IMU traces |
| `API.md` | the HTTP API for the Jetson (contract C2) |
| `WIRING.md`, `wiring/` | wire cut list, change list from v29, diagram |
| `BENCH_TEST.md` | how to check every motor and sensor before closing the hull |
| `tools/wires_v4.py` | regenerates `wiring/` |
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

## Board settings

- Tools > Board > esp32 > **XIAO_ESP32S3**.
- Tools > **USB CDC On Boot: Enabled** (the default for this board). If it is Disabled, the sketch will not compile.
- Leave the rest at the defaults.

## Upload over USB

1. **Battery unplugged** (power loop plug out). Never have the battery plug and USB in together while you work.
2. Plug the XIAO into the Mac with a USB-C data cable.
3. Tools > Port: pick the one that appears (`/dev/cu.usbmodem...`).
4. Click Upload (arrow). Wait for "Hard resetting via RTS pin" / "Done uploading".
   If the port does not appear or the upload fails: hold the XIAO's **B** (boot) button, tap **R** (reset), release
   B, then upload again. After such an upload, tap R once.
5. Tools > Serial Monitor, 115200 baud, "No line ending". You should see `KEDGE v4 ready` and the key list.
   Type `z` and press Enter: expect `I2C device at 0x40` (INA219) and `0x4a` (BNO085).

## Upload over WiFi (after the first USB upload)

USB unplugged, power loop plug in. Join WiFi `KEDGE` (password `grappstar`). Tools > Port > network port `kedge`. Upload. Motors stop and any sequence is
aborted when the upload starts.

## Driving it

- Web page: join `KEDGE`, open `http://192.168.4.1`. Hold a button or key to run, release to stop.
- Serial Monitor: type a key and press Enter, each press runs 1.5 s.
- Keys: `1`/`!` deploy down/up, `2`/`@` kick out/back, `3`/`#` drills in/out, `4`/`$` foot in/out, space or `s` STOP,
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
- After a brownout or watchdog reset the dart boots with motors off and reports `FAULT` until `/stop`.
- INA219 missing: manual keys only (on timers), auto and release refused. BNO085 missing: no arming, no flight log.

## Tuning

The current thresholds in `kedge_logic.h` (`struct Tuning`) are from the v29 bench numbers at 8 V. Measure the real
ones with BENCH_TEST.md steps 3 to 6 and write them in at step 9. Then run the PC tests.

## PC tests

```
cd firmware/dart/test && ./run_tests.sh
```

Needs g++ and python3 (Mac: run `xcode-select --install` once for g++). Prints every check and `ALL PASS`, then checks the JSON files the tests wrote.

## Compile check without the IDE (optional)

```
arduino-cli core install esp32:esp32 --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli lib install "Adafruit BNO08x"
arduino-cli compile -b esp32:esp32:XIAO_ESP32S3 firmware/dart/kedge_dart
```
