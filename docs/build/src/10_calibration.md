# 10. Calibration (base)

Where to type the commands: on the bench (ESP32 on the Mac's USB) in the Arduino Serial Monitor. Once the ESP32 is
on the Jetson, on the Jetson in `~/GrappStar/jetson`: `python3 -m grappstar base cmd '<the same JSON>'`, with
`python3 -m grappstar base monitor` in a second terminal for telemetry. If you installed the grappstar service, stop
it first (`sudo systemctl stop grappstar`): it holds the serial port. Never move the USB plug with the battery in.

Do it after the base is assembled and wired, springs off, before test stage 4. The bench test (section 11.1) comes
first, on the bench (order of work step 7). Part (g), the cock geometry check,
needs the springs: do it at test stage 5. The values are stored on the ESP32.

@include firmware/base/CALIBRATION.md
