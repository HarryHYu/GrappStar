# 10. Calibration (base)

Where to type the commands: on the bench (ESP32 on the Mac's USB) in the Arduino Serial Monitor. Once the ESP32 is
on the Jetson: `python3 -m grappstar base console` in `~/GrappStar/jetson`. It shows telemetry twice a second and
every event, and sends what you type: a JSON line, or a word with key=value pairs (`reel duty=0.3 until=stop`,
`set key=limit_n val=40`, `mode to=REEL`), or a bare `stop` / `estop` / `status`; `quit` ends. Only one program may
hold the serial port: stop the operator page and the grappstar service first (`sudo systemctl stop grappstar`).
Never move the USB plug with the battery in. On the rover, where CALIBRATION.md says "battery unplugged" only so
you can turn the drum by hand or touch a contact while reading `sw`, use the motors-safe state: lead #2's XT60 out
(motors dead, ESP32 and Jetson still powered).

Do it after the base is assembled and wired, springs off, before test stage 4. The bench test (section 11.1) comes
first, on the bench (order of work step 7). Part (g), the cock geometry check,
needs the springs: do it at test stage 5. The values are stored on the ESP32.

@include firmware/base/CALIBRATION.md
