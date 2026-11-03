# 11. Bench tests

## 11.1 Base

This runs on the bench before the base is assembled (order of work step 7): wiper motor clamped in a vice, drum
N20 loose. Where to type the commands: on the bench (ESP32 on the Mac's USB) in the Arduino Serial Monitor. Once the ESP32 is
on the Jetson: `python3 -m grappstar base console` in `~/GrappStar/jetson`. It shows telemetry twice a second and
every event, and sends what you type: a JSON line, or a word with key=value pairs (`reel duty=0.3 until=stop`,
`set key=limit_n val=40`, `mode to=REEL`), or a bare `stop` / `estop` / `status`; `quit` ends. Only one program may
hold the serial port: stop the operator page and the grappstar service first (`sudo systemctl stop grappstar`).
Never move the USB plug with the battery in.


@include firmware/base/BENCH_TEST.md

## 11.2 Dart

The dart bench tests are in section 6 (`firmware/dart/BENCH_TEST.md` and the dart build order, steps 6, 7 and 11).
Foot check: key 4 turns the foot auger clockwise seen from the motor end, looking toward the tip.
