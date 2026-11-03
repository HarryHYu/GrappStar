# 13. Troubleshooting

First move for any problem: send `estop` (or pull the battery plug), then read the `fault` field in telemetry.

## Base

| Symptom | Likely cause | Fix |
|---|---|---|
| No reply from the base on the Jetson | Charge-only USB cable; cable in the UART port; wrong serial device | Data cable in the port marked USB. On the Jetson, `ls -l /dev/grappstar-base` must show a link to ttyACM0 (jetson/README.md section 4) |
| ESP32 resets when the wiper starts | Ground or USB power dip | Check the ESP32 GND wire to the BTS7960 GND; short USB cable; report BROWNOUT faults |
| Wiper does nothing, no fault | R_EN / L_EN not wired, 20 A fuse blown | Battery unplugged, check the fuse and the two enable wires |
| Wiper turns one way only | RPWM or LPWM loose | Re-seat the Dupont jumper |
| 20 A fuse blows | Stall into a hard stop, short on the wiper leads | Find the bind by turning by hand; never fit a bigger fuse |
| Fault STALL or OVERCURRENT | Something binds, springs stronger than planned | Battery unplugged, turn the train by hand, find the tight spot; redo the spring measurement |
| Fault DRUM_TIMEOUT | Drum N20 not turning: no 9 V, board asleep, gear slipping | Meter on DRV8833 VM (9.0 V); check AIN1 / AIN2 / GND; check the board make (MEASURE_LIST D3); check the N20 gear set screw |
| Fault DRUM_SENSOR or SWITCH_DISAGREE | A drum contact does not close or stays closed | Battery unplugged, ESP32 on USB only (contacts read on USB power): watch `sw` while turning the drum by the hand lever; bend or re-screw the contact |
| Fault ANGLE_SENSOR | AS5600 cable out, magnet too far | Re-seat the STEMMA cable; magnet 1 to 3 mm from the chip, centred |
| `pitch_deg` wrong after a restart | Pitch reference lost | Run the pitch part of the calibration again (section 10) |
| Drum stuck between positions with the power off | Expected: the worm holds it | Use the hand lever on the drum shaft end, battery unplugged |
| Base cocked and you need to unload without firing | | Send `{"cmd":"cal","what":"decock"}`: the wiper runs backward slowly and the springs unwind under the worm until `SW_HOME` closes. Never fire without the tether, and never hand-release the carriage |

## Rover and video

| Symptom | Likely cause | Fix |
|---|---|---|
| Jetson turns off under load | Battery low, or voltage dip | Charge the LiPo; check the barrel plug is tight; check `vbat` |
| No dart video | VTX and RC832 on different channels; RC832 not powered | Set the RC832 to the VTX channel (scan button); check its LED and the RCA lead |
| Dart video is snow | Range, antenna off, bad channel | Antenna on both ends; try another channel |
| Kiyo X shows nothing or errors | JPEG library mismatch on JetPack | The software forces YUYV or MJPEG (jetson/README.md); unplug and replug |

## Dart

| Symptom | Likely cause | Fix |
|---|---|---|
| Serial Monitor blank | USB CDC On Boot disabled | Tools > USB CDC On Boot: Enabled, re-upload |
| No KEDGE WiFi | XIAO not powered or crashed | Loop plug in? 5 V at the XIAO? Press reset |
| A motor turns the wrong way | Leads reversed | Loop plug out, swap that motor's pair (run sheet section 10) |
| Current trip during anchoring | Stall on a stone, or the trip set too low | Re-measure stall currents (section 14 F3) and update the tuning |
| INA219 or BNO085 not found | I2C wiring | `z` key scans; SDA and SCL must not be swapped |
