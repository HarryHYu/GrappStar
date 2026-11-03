# Base station wiring

Board: ESP32-S3 DevKitC-1-N8. Pins come from `grappstar_base/pins.h`. Nothing here has been wired and tested on real
hardware yet: check every step with the multimeter.

**The rule: battery unplugged whenever a wire or plug moves. Plug the battery in last, after the checks. Never pull a
plug while the battery is in.**

## 1. Pin table

| GPIO | Name in pins.h | Connects to | Note |
|---|---|---|---|
| 4 | `PIN_RPWM` | BTS7960 RPWM | wiper FORWARD (dir 1), PWM 20 kHz |
| 5 | `PIN_LPWM` | BTS7960 LPWM | wiper BACKWARD (dir -1), PWM 20 kHz |
| 6 | `PIN_REN` | BTS7960 R_EN | plus 10 k from R_EN to GND |
| 7 | `PIN_LEN` | BTS7960 L_EN | plus 10 k from L_EN to GND |
| 1 | `PIN_WIPER_IS` | BTS7960 R_IS + L_IS through 1 k | plus 680 R from GPIO1 to GND (the IS divider) |
| 2 | `PIN_VBAT` | battery divider middle | 100 k from 3S + (fused side) to GPIO2, 22 k from GPIO2 to GND |
| 8 | `PIN_SDA` | AS5600 SDA | STEMMA QT blue wire |
| 9 | `PIN_SCL` | AS5600 SCL | STEMMA QT yellow wire |
| 15 | `PIN_DRUM_IN1` | DRV8833 AIN1 | drum N20 |
| 16 | `PIN_DRUM_IN2` | DRV8833 AIN2 | drum N20 |
| 10 | `PIN_SW_A` | contact `SW_DRUM_A` | drum code bit A |
| 11 | `PIN_SW_B` | contact `SW_DRUM_B` | drum code bit B |
| 12 | `PIN_SW_C` | contact `SW_DRUM_C` | drum code bit C |
| 13 | `PIN_SW_D` | contact `SW_DOG_L` | collar engaged in LAUNCH |
| 14 | `PIN_SW_E` | contact `SW_DOG_R` | collar engaged in REEL |
| 47 | `PIN_SW_F` | contact `SW_N20` | drum N20 in mesh (open = hand lever latched out) |
| 21 | `PIN_SW_G` | contact `SW_HOME` | carriage on its front stop (uncocked) |
| 39 | `PIN_SW_H` | contact `SW_GAP` | big gear in its toothless gap (rest) |
| 40 | `PIN_SW_I` | contact `SW_RAIL_LOW` | rail at its lowest pitch (aim home) |
| 41 | `PIN_SW_J` | contact `SW_SEATED` | dart tail on the pusher |
| 42 | `PIN_SW_K` | contact `SW_ARM` | wrap cup ring at home (one pulse per cup turn) |
| 35 | `PIN_SW_L` | contact `SW_INDEX` | only on a build without the AS5600 magnet (decision D12). Leave open otherwise |
| 17 | `PIN_HX_DT` | HX711 DT (DOUT) | optional load cell |
| 18 | `PIN_HX_SCK` | HX711 SCK | optional load cell |
| 3V3 | | AS5600 VIN, BTS7960 VCC, HX711 VCC | logic supply |
| GND | | common ground (see section 2) | |

The DRV8833 SLP pin is **not wired** (decision D9: the Pololu board pulls it high by itself). Its FLT pin is not used
either.

Which contact goes on which pin comes from `swBit` in `config.h`. Default: the order above, `SW_DRUM_A` on `PIN_SW_A`
through `SW_ARM` on `PIN_SW_K`. `SW_INDEX` is switched off in `config.h` unless the build has no magnet.

Pins the firmware avoids on purpose: 0, 3, 45, 46 (boot strapping), 19 and 20 (the USB port), 26 to 32 (flash),
43 and 44 (UART), 38 and 48 (the RGB LED). `params.py` suggested GPIO38 for `SW_HOME`; that is the RGB LED on newer
boards, so `SW_HOME` is on GPIO21 instead.

## 2. Power

```
3S LiPo + --> 20 A fuse (XT60 lead #2) --> BTS7960 B+ --+--> 1 to 2 A fuse --> MP1584 IN+   (buck, set to 9.0 V first)
                                                        +--> 100 k --> GPIO2 --> 22 k --> GND
3S LiPo - -----------------------------------------> BTS7960 B- --> MP1584 IN-

MP1584 OUT+ (9.0 V) --> DRV8833 VM (Pololu labels it VIN)
MP1584 OUT-         --> DRV8833 GND

BTS7960 M+ / M-  --> wiper motor red / black (never the green park switch wire)
DRV8833 AOUT1 / AOUT2 --> drum N20 motor
ESP32 power: USB from the Jetson (from the Mac on the bench)
```

- **Wiper:** 3S LiPo, then the 20 A fuse, then BTS7960 B+ and B-. Use thick wire (14 AWG) on B+, B-, M+ and M-.
  Never wire the wiper's green park switch wire.
- **Drum N20:** through the MP1584 buck set to **9.0 V**. The DRV8833 VM maximum is 10.8 V and a full 3S is 12.6 V,
  so **never connect the 3S straight to the DRV8833.** Set the buck to 9.0 V BEFORE its output touches the DRV8833
  (Part A below). Take the buck input from the BTS7960 B+ and B- terminals, so it sits behind the main fuse.
- **Small fuse (recommended):** a 1 to 2 A fuse in the buck IN+ lead. A short in the buck or the DRV8833 then blows
  this little fuse instead of burning thin wires on the way to the 20 A fuse.
- **You may need a spare MP1584.** Both of yours are in the dart (one at 8.12 V, one at 5.04 V). Buy another one,
  or borrow one from the dart for the bench. 8 to 9 V works for the N20; 9.0 V is the target.
- **ESP32:** powered over USB from the Jetson (on the bench: from the Mac). Never connect the battery to the ESP32
  5V or 3V3 pins.
- **Common ground:** ESP32 GND, BTS7960 GND (logic header), DRV8833 GND and MP1584 OUT- must all be joined. Without a
  common ground the control signals mean nothing.
- **Battery divider:** 100 k from the fused 3S + to GPIO2, 22 k from GPIO2 to GND. 12.6 V reads 2.27 V on the pin.
  It feeds `vbat` in telemetry and the battery checks (`NO_POWER` under 6 V, `BROWNOUT` under 9.9 V while a motor
  runs). With the battery unplugged `vbat` reads about 0. Optional: a 100 nF capacitor from GPIO2 to GND steadies the
  reading. If you leave the divider out, send `{"id":1,"cmd":"set","key":"vbat_wired","val":false}` (then `vbat` is
  `null` and there is no battery check).

## 3. BTS7960 details

- **10 k pull-downs on R_EN and L_EN.** While the ESP32 resets or is being flashed its pins float, and a floating
  enable pin can let the motor twitch. With 10 k to GND the BTS7960 stays off until the firmware drives R_EN and L_EN
  high. Solder each resistor between the EN pin and a GND pin on the BTS7960 header.
- **VCC** to ESP32 3V3, **GND** to ESP32 GND.
- **IS divider:** tie R_IS and L_IS together. From there a 1 k resistor to GPIO1. From GPIO1 a 680 R resistor to GND.
  The BTS7960 IS pin gives a small current: motor amps divided by 8500. At 20 A that is 2.35 mA, and 2.35 mA through
  the 680 R gives about 1.6 V at GPIO1, well under 3.3 V. So `amps_per_volt` = 8500 / 680 = 12.5 (the default).
- **Optional:** a 1 uF to 2.2 uF capacitor from GPIO1 to GND smooths the PWM pulses on the IS signal. Only if you
  have one. Ceramic is best; with an electrolytic the striped leg goes to GND.
- **Check for an onboard IS resistor first.** With everything unpowered, tie R_IS and L_IS together and put the meter
  (ohms) between that point and GND. Open (no reading) means no onboard resistor: the default scale 12.5 is right.
  If it reads a resistance Rb, write it down. The scale changes:

  `Rx = 1680 in parallel with Rb`, then `amps_per_volt = 8500 x 1680 / (Rx x 680)`.

  No onboard resistor gives 12.5. Two onboard 1 k gives about 54.5. Set it with
  `{"id":1,"cmd":"set","key":"amps_per_volt","val":54.5}` (your number), then calibrate it (CALIBRATION.md).

## 4. DRV8833 details

- VM (Pololu: VIN) from the buck OUT+ at 9.0 V. GND to the buck OUT- and to the ESP32 GND.
- AIN1 to GPIO15, AIN2 to GPIO16, AOUT1 and AOUT2 to the drum N20. Only channel A is used. Leave BIN1, BIN2, BOUT1,
  BOUT2 open.
- **SLP: leave it unconnected** (decision D9). The board pulls it high itself, so the driver is always awake. The
  DRV8833 chip has pull-downs on its AIN inputs, so the motor stays off while the ESP32 boots.
- FLT: leave it unconnected.
- Never put 3V3 on a pad that is the motor supply. Meter check (unpowered): if a pad beeps to VM, it is the motor supply.
- If the drum turns the wrong way, swap the two N20 wires at AOUT1 and AOUT2 (battery unplugged). CALIBRATION.md
  step (c) says how to tell.

## 5. Contact switches

Each printed contact is an M3 screw head touching an M3 nut.

- One side of each contact to its GPIO, the other side to GND.
- The ESP32 uses its internal pull-up. Closed = LOW = shown as closed in telemetry.
- No resistors needed. All the GND sides may share one GND wire.

| Contact | Pin | GPIO | Telemetry `sw` key |
|---|---|---|---|
| `SW_DRUM_A` | `PIN_SW_A` | 10 | `a` |
| `SW_DRUM_B` | `PIN_SW_B` | 11 | `b` |
| `SW_DRUM_C` | `PIN_SW_C` | 12 | `c` |
| `SW_DOG_L` | `PIN_SW_D` | 13 | `l` |
| `SW_DOG_R` | `PIN_SW_E` | 14 | `r` |
| `SW_N20` | `PIN_SW_F` | 47 | `n` |
| `SW_HOME` | `PIN_SW_G` | 21 | `H` |
| `SW_GAP` | `PIN_SW_H` | 39 | `G` |
| `SW_RAIL_LOW` | `PIN_SW_I` | 40 | `L` |
| `SW_SEATED` | `PIN_SW_J` | 41 | `S` |
| `SW_ARM` | `PIN_SW_K` | 42 | `W` |
| `SW_INDEX` | `PIN_SW_L` | 35 | `I` (only shows when switched on in `config.h`) |

The three drum code contacts read the drum position as a code (BENCH_TEST.md step 2 has the table). All three open
(code 000) never happens on a working drum: the firmware treats it as a wire off and faults (`DRUM_SENSOR`).

On the bench, before the drum is built, use two jumper wires:

- GPIO11 to GND: code "b only" = NEUTRAL.
- GPIO47 to GND: `SW_N20` closed. Without it the firmware thinks the hand lever is latched out and refuses every drum
  move (`HAND_MODE`).

## 6. AS5600 and HX711

- **AS5600** (Adafruit STEMMA QT): use a STEMMA QT cable. Red to 3V3, black to GND, blue (SDA) to GPIO8, yellow (SCL)
  to GPIO9. DIR pad to GND (fixed count direction).
- **Magnet:** the Adafruit board comes without one. Any small neodymium disc edge-on (6 to 10 mm) or a 6 x 2.5 mm
  diametric disc in the magnet cap on the main shaft end, **1 to 3 mm from the chip, centred** on it.
- **HX711** (optional): VCC to 3V3, GND to GND, DT to GPIO17, SCK to GPIO18. Load cell wires to E+, E-, A+, A- as
  printed on the HX711 board. The firmware finds it at boot (`"hx711":true` in the `HELLO` line).

## 7. Build order, step by step

You need: multimeter, the 20 A fuse lead (XT60 lead #2), a 1 to 2 A inline fuse for the buck, 2 x 10 k, 1 k, 680 R,
100 k, 22 k, optionally a 1 uF to 2.2 uF and a 100 nF capacitor, an insulated small screwdriver for the buck.

### Part A: set the buck (on its own, before the build)

The buck must be set to 9.0 V before it ever touches the DRV8833. This is done with the buck on its own: nothing else
is wired to it.

1. Battery unplugged. Solder short input leads to the MP1584: red to IN+ (with the 1 to 2 A fuse in it), black to
   IN-. Nothing on the output.
2. Battery unplugged. Connect the input leads to the fused side of the XT60 lead (red to +, black to -). Check with
   the meter (beep mode) that IN+ and IN- are not shorted.
3. Battery unplugged. Clip the meter probes onto OUT+ and OUT- (meter on DC volts), so you do not have to hold them.
4. Plug the battery in. Turn the little screw on the buck with the insulated screwdriver until the meter reads
   **9.0 V**. Turn slowly: many turns may be needed. Keep the screwdriver and probes off the other pads.
5. Battery unplugged: unplug it now, before anything else. Then disconnect the buck input leads from the XT60 lead.
   Mark the buck "9.0 V".

### Part B: build

1. Battery unplugged. Solder the two 10 k pull-downs: R_EN to GND and L_EN to GND on the BTS7960 header.
2. Battery unplugged. Check the BTS7960 for an onboard IS resistor (section 3). Write the number down.
3. Battery unplugged. Wire the IS divider: R_IS and L_IS tied, 1 k to GPIO1, 680 R from GPIO1 to GND. Optional
   capacitor from GPIO1 to GND.
4. Battery unplugged. BTS7960 logic: RPWM GPIO4, LPWM GPIO5, R_EN GPIO6, L_EN GPIO7, VCC 3V3, GND to ESP32 GND.
5. Battery unplugged. Battery divider: 100 k from the BTS7960 B+ terminal to GPIO2, 22 k from GPIO2 to GND.
   Optional 100 nF from GPIO2 to GND.
6. Battery unplugged. AS5600: STEMMA QT cable to 3V3, GND, GPIO8 (SDA), GPIO9 (SCL). DIR to GND.
7. Battery unplugged. Contacts: one side to the GPIO from section 5, other side to GND. Contacts not built yet: put
   the two bench jumpers on (GPIO11 to GND, GPIO47 to GND, section 5).
8. Battery unplugged. HX711 if you have it: 3V3, GND, GPIO17 (DT), GPIO18 (SCK).
9. Battery unplugged. USB check with no power wiring yet: plug USB into the port labelled USB, open the Serial
   Monitor and do "First contact" in README.md. `status` must show `"magnet":"OK"`. Close each contact and watch
   telemetry `sw` (BENCH_TEST.md step 2). Unplug USB.
10. Battery unplugged. DRV8833: AIN1 GPIO15, AIN2 GPIO16, GND to ESP32 GND. SLP and FLT stay unconnected.
    AOUT1 and AOUT2 to the drum N20.
11. Battery unplugged. Buck output (already set to 9.0 V in Part A): OUT+ to DRV8833 VM, OUT- to DRV8833 GND.
12. Battery unplugged. Buck input: IN+ (through its 1 to 2 A fuse) to BTS7960 B+, IN- to BTS7960 B-.
13. Battery unplugged. Power: fused XT60 lead + to BTS7960 B+, - to B-. Use 14 AWG. The 20 A fuse in the + lead.
14. Battery unplugged. Wiper motor to M+ and M- (red to M+, black to M-). On the bench the motor is clamped and has
    nothing attached (BENCH_TEST.md).
15. Battery unplugged. Ground: check ESP32 GND, BTS7960 GND, DRV8833 GND and buck OUT- are all joined.

### Part C: checks with the multimeter (still unplugged)

16. Battery unplugged. Beep mode: **B+ to B- must NOT beep.** A very short chirp is fine (capacitors on the boards
    charging from the meter). A steady beep is a short: stop and find it.
17. Battery unplugged. Beep mode: buck OUT+ to OUT- must not beep. DRV8833 VM to GND must not beep.
18. Battery unplugged. Beep mode: ESP32 GND to BTS7960 GND, DRV8833 GND and buck OUT- must beep (common ground).
19. Battery unplugged. Beep mode: ESP32 3V3 to GND must not beep (a short chirp is fine). 3V3 must not beep to B+ or
    to DRV8833 VM.
20. Battery unplugged. Ohms: GPIO2 to GND about 22 k (a bit less is fine). R_EN to GND and L_EN to GND about 10 k.
21. Battery unplugged. Check every motor and power screw terminal is tight and no bare strand touches its neighbour.

### Part D: first power

22. Battery unplugged. Plug USB in first. The Serial Monitor shows telemetry with `"state":"IDLE"`.
23. Plug the battery in, last. Nothing should move.
24. Measure the buck output again: 9.0 V. Measure the battery at the XT60 and compare with `vbat` in telemetry.
25. If anything is hot, smells or a motor moves on its own: unplug the battery at once.
26. Go on with BENCH_TEST.md.
