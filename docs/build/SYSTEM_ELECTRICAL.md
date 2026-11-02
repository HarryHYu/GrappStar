# GrappStar system electrical (rover, base and dart)

Diagram: `system_electrical.svg` / `system_electrical.png` (source `system_electrical.dot`). The ESP32 pin
table is owned by `firmware/base/WIRING.md` and `firmware/base/grappstar_base/pins.h`; this page shows how the
whole rover is powered and connected. If this page and WIRING.md ever disagree on a pin, WIRING.md wins.

## 1. What is on the rover

| Block | Part | Powered from |
|---|---|---|
| Battery | Zeee 3S LiPo 5200 mAh, XT60 (9.9 V empty in use, 11.1 V nominal, 12.6 V full) | |
| Wiper branch | XT60 lead #2 with the 20 A fuse, BTS7960 module, 12 V wiper motor (2 wires, park switch taped off) | LiPo |
| Drum branch | MP1584 #2 buck set to 9.0 V, DRV8833 #3 (the one freed from the dart), selector drum N20 worm | BTS7960 B+ / B- terminals, after the 20 A fuse |
| Base brain | ESP32-S3 DevKitC-1-N8 | Jetson USB (the port marked USB, not UART) |
| Sensors on the ESP32 | Adafruit AS5600 (STEMMA QT, I2C 0x36), printed contacts (M3 screw head on an M3 nut), optional HX711 + 100 kg load cell, battery divider | ESP32 3V3 |
| Jetson | Orin Nano Super dev kit | XT60 lead #1 (10 A fuse) to a 5.5 x 2.5 mm barrel pigtail |
| Video in | Lockmaner RC832 5.8 GHz receiver, AV (yellow RCA) out to the DIGITNOW USB capture, which plugs into the Jetson | XT60 lead #1 branch (12 V class input) |
| Rover camera | Razer Kiyo X | Jetson USB |
| Dart | KEDGE: XIAO ESP32S3, 2 x DRV8833, INA219, BNO085, Caddx Ant, Zeus VTX | its own 2S 18350 pack. No wire to the rover: the tether is Dyneema and carries no power |

## 2. Power tree

```
3S LiPo (XT60)
  -> XT60 Y harness (one male in, two female out; made from your spare XT60s, 14 AWG)
       -> lead #2, 20 A fuse, 14 AWG -> BTS7960 B+ / B-  -> wiper motor (BTS7960 M+ / M-, 14 AWG)
                                   \-> 22 AWG from the B+ / B- screws -> MP1584 #2 IN (9.0 V out)
                                          -> DRV8833 #3 VM / GND -> drum N20 (AOUT1 / AOUT2)
       -> lead #1, 10 A fuse, 18 AWG -> barrel pigtail -> Jetson DC jack
                                   \-> 22 AWG -> RC832 power in
Jetson USB -> ESP32-S3 DevKitC (5 V from USB; the board makes its own 3V3)
                ESP32 3V3 -> BTS7960 VCC, DRV8833 SLP (via GPIO), AS5600, HX711
Jetson USB -> DIGITNOW capture <- RC832 AV out
Jetson USB -> Kiyo X
```

Why it is laid out this way:
- The DRV8833 motor supply (VM) is 10.8 V maximum. The LiPo is 12.6 V full, so the drum N20 runs from MP1584 #2
  set to 9.0 V. MP1584 #2 is the spare from the dart build (it was the 8.0 V bench supply there): reset it to 9.0 V
  after the dart bench test.
- The buck taps the BTS7960 screw terminals, so both motor branches sit behind the 20 A fuse and the Jetson branch
  stays separate. A wiper stall that blows the 20 A fuse does not reboot the Jetson.
- The ESP32 takes power only from USB. It never connects to the LiPo directly. Its ground meets battery minus
  through the BTS7960 and DRV8833 GND wires (needed so the logic signals have a reference).

## 3. Wires and connectors

| Run | From | To | Gauge | Connector | Note |
|---|---|---|---|---|---|
| P1 | LiPo | Y harness | 14 AWG | XT60 | Make the Y from spare XT60s; solder, heat shrink each joint |
| P2 | Y harness | BTS7960 B+ / B- | 14 AWG | XT60 lead #2 (20 A fuse), screw terminals at the BTS7960 | Red to B+, black to B-. 10 A blew on wiper inrush before: keep 20 A |
| P3 | BTS7960 M+ / M- | wiper motor | 14 AWG | screw terminals; spade or solder at the motor | Red wire (+) and black (-) of the rebuilt 2-wire motor |
| P4 | BTS7960 B+ / B- | MP1584 #2 IN+ / IN- | 22 AWG | under the same screws | Set 9.0 V with nothing on the output first |
| P5 | MP1584 #2 OUT+ / OUT- | DRV8833 #3 VM / GND | 22 AWG | solder | Check 9.0 V on the meter before you connect |
| P6 | DRV8833 #3 AOUT1 / AOUT2 | drum N20 | 26 AWG | solder or JST | Swap the pair if the drum turns the wrong way (see calibration) |
| P7 | Y harness | Jetson DC jack | 18 AWG | XT60 lead #1 (10 A fuse), 5.5 x 2.5 mm barrel plug (same as the plug on the Jetson supply), centre positive | Check polarity with the meter before the first plug-in |
| P8 | lead #1 (after the fuse) | RC832 power in | 22 AWG | the RC832 power lead | Read the RC832 label for its input range first (see MEASURE_LIST) |
| S1 | Jetson USB-A | ESP32-S3 USB port (native, marked USB) | | USB-C or micro-USB cable to suit the board | Data cable, not charge-only |
| S2 | Jetson USB-A | DIGITNOW capture | | its own USB plug | |
| S3 | RC832 AV out | DIGITNOW video in | | yellow RCA | Video only; audio not used |
| S4 | Jetson USB-A | Kiyo X | | its own USB plug | |
| L1 | ESP32 pins | BTS7960 RPWM, LPWM, R_EN, L_EN, IS, VCC, GND | 26 to 28 AWG | Dupont jumpers | Pins in firmware/base/WIRING.md |
| L2 | ESP32 pins | DRV8833 #3 AIN1, AIN2, SLP, GND (nFAULT if present) | 26 to 28 AWG | Dupont | |
| L3 | ESP32 SDA / SCL / 3V3 / GND | AS5600 | | STEMMA QT to Dupont cable | Magnet 1 to 3 mm from the chip |
| L4 | ESP32 pins | each printed contact | 26 AWG twisted pair | Dupont at the ESP32, M3 ring or solder at the contact | One wire to the pin, the other to GND |
| L5 | ESP32 pins | HX711 DT, SCK, VCC, GND (optional) | 26 AWG | Dupont | Load cell 4 wires into HX711 E+, E-, A+, A- |
| L6 | BTS7960 B+ | ESP32 battery pin through the 100 k / 22 k divider | 26 AWG | solder at the divider | 12.6 V reads 2.27 V at the pin |

Gauges assume runs under 0.5 m. 14 AWG carries the 18 A wiper stall; 18 AWG is fine for the Jetson (about 2.5 A at
10 V); 22 AWG is fine for the drum branch (N20 stall about 0.2 A) and the RC832 (about 0.2 A).

## 4. First power-on sequence (do it in this order, once, after any rewiring)

Battery unplugged for steps 1 to 4. Have the multimeter ready.

1. Battery unplugged. Pull both fuses. Meter on beep: touch red and black of every branch (P2 at the BTS7960, P4,
   P5, P7 at the barrel plug, P8). Nothing may beep. A beep is a short: find it before going on.
2. Battery unplugged. Meter on beep: barrel plug centre pin to the red wire of lead #1 must beep (centre positive).
3. Battery unplugged. DRV8833 #3 VM wire disconnected from MP1584 #2 OUT. ESP32 USB cable unplugged.
4. Battery unplugged. Meter on DC volts on the LiPo balance lead: each cell 3.7 to 4.2 V.
5. Plug the battery in. Fit the 20 A fuse only. Meter on MP1584 #2 OUT: turn the brass screw until it reads 9.0 V.
   The wiper must not move (the BTS7960 enables are held low by their pull-downs). If it moves: unplug the battery
   and check R_EN / L_EN.
6. Unplug the battery. Solder or plug the DRV8833 VM wire (P5) to MP1584 #2 OUT.
7. Plug the battery in. Fit the 10 A fuse. The Jetson boots (green LED). The RC832 screen or LED lights.
8. Plug the ESP32 USB cable into the Jetson. The ESP32 power LED lights. Nothing moves (DRV8833 SLP has a pull-down
   and stays off until the firmware enables it).
9. On the Jetson, in `~/GrappStar/jetson`: `python3 -m grappstar base ping` (expect `"ok": true`), then
   `python3 -m grappstar base monitor`. Check `vbat` in telemetry matches the meter within 0.3 V.

Unplug order (every time): stop the motors (`estop`), pull the battery, then anything else.

## 5. Rules

- Battery unplugged before you touch, add or move any wire. This killed the first XIAO.
- Battery unplugged before you plug the ESP32 into the Mac. Flash from the Mac with the ESP32 out of the rover
  harness, or flash from the Jetson (firmware/base/README.md).
- Stop using the LiPo at 3.3 V per cell under load (9.9 V); the firmware reports `vbat` and the Jetson warns.
- Never connect the DRV8833 VM to the LiPo directly (10.8 V maximum; the LiPo is 12.6 V full).
- Never put 3V3 on a DRV8833 pad called VIN, VCC or VM: on these carriers that is the motor supply. 3V3 only reaches
  the SLP / nSLEEP / EEP pad, and here it comes from an ESP32 GPIO.
- Dart: power loop plug out whenever a wire moves or USB is plugged in (the dart run sheet rule).
- The VTX on the dart must have its antenna on before it is powered.
