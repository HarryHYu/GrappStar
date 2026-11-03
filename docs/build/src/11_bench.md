# 11. Bench tests

## 11.1 Base

This runs on the bench before the base is assembled (order of work step 7): wiper motor clamped in a vice, drum
N20 loose. Where to type the commands: on the bench (ESP32 on the Mac's USB) in the Arduino Serial Monitor. Once the ESP32 is
on the Jetson, on the Jetson in `~/GrappStar/jetson`: `python3 -m grappstar base cmd '<the same JSON>'`, with
`python3 -m grappstar base monitor` in a second terminal for telemetry. If you installed the grappstar service, stop
it first (`sudo systemctl stop grappstar`): it holds the serial port. Never move the USB plug with the battery in.


@include firmware/base/BENCH_TEST.md

## 11.2 Dart

The dart bench tests are in section 6 (`firmware/dart/BENCH_TEST.md` and the dart build order, steps 6, 7 and 11).
Foot check: key 4 turns the foot auger clockwise seen from the motor end, looking toward the tip.
