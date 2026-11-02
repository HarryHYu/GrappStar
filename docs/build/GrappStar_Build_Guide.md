# GrappStar build guide

One guide from parts to first mission: print, wire, flash, calibrate, test. Every hands-on step is in order. Every
wiring step starts with the battery unplugged.


Built 2026-11-03 10:59 Sydney time from commit `4403cab`.


How to use it: read section 1 and 2 first, then work through the "Order of work" list in section 1. Where a section
says "From `some/file.md`", that text comes straight from the repo, so it is always the latest version. Re-build the
guide after any change: `python3 docs/build/make_guide.py`.


# 1. What the system is

GrappStar is a rover that can pull itself out of trouble. A launcher on the rover fires a small dart (KEDGE) on a
Dyneema tether. The dart lands, screws itself into the soil, and the rover winches against it. Then the dart lets go
and is reeled home.

Three parts:

- **Rover.** Jetson Orin Nano Super on the rover's 3S LiPo. It runs the mission, the operator web page, and the
  vision (rover camera Razer Kiyo X, and the dart's video through an RC832 receiver and a DIGITNOW USB capture).
- **Base station** (on the rover). ONE 12 V wiper motor drives everything. A sliding dog collar on the main shaft
  couples the wiper to the LAUNCH train, to nothing (NEUTRAL), or to the REEL train. A small N20 motor turns a
  selector drum (like a motorcycle gear drum) that moves the collar and also lays and removes the capstan wraps.
  - LAUNCH, wiper forward: draws the spring carriage back, fires at the end of the stroke, resets.
  - LAUNCH, wiper backward: tilts the rail up and down to aim. A self-locking worm holds the angle.
  - REEL, wiper forward: the capstan winds in; the storage reel takes up the slack through a slip clutch.
  - Payout at launch: the line skips the capstan and pulls off the free-spinning storage reel against a light
    brake (0.5 to 1 N).
  - Any fault: wiper off, drum to NEUTRAL. There is a hand lever on the drum shaft end.
  - Brain: an ESP32-S3 DevKitC on USB from the Jetson, with an AS5600 angle sensor on the main shaft and printed
    contact switches (an M3 screw head touching an M3 nut).
- **Dart (KEDGE v30).** 42 mm x 152 mm, its own XIAO ESP32S3, 2S 18350 pack, WiFi access point. After landing a kick
  leg swings over the nose and plants a foot auger, then two drill pods swing down and screw in. A nose camera
  sends analog video at 5.8 GHz.

How they talk: the Jetson talks to the base over USB serial (JSON lines, `firmware/base/PROTOCOL.md`) and to the dart
over WiFi (HTTP at 192.168.4.1, `firmware/dart/API.md`). Video is analog 5.8 GHz into the USB capture.

A mission: pick a target on the rover camera view, the Jetson looks up the pitch in the ballistics table, the base
aims, cocks and fires on your confirmation, the dart video is analysed for the landing zone, the dart anchors, the
base wraps the capstan and tows, the dart releases, the base reels it home.

## Order of work

1. Measure the wiper motor and the springs (section 14, groups A and B). Put the numbers in `base/spec/params.py`
   and re-run the base pipeline. The base parts that touch the motor depend on them.
2. Confirm the parts box (section 14, group C). The only purchase is 608 bearings.
3. Print the dart fit coupon, then the dart plates (section 4). The dart does not depend on the measurements.
4. Flash the dart and build it (sections 6 and 8, and the dart run sheet in the appendix).
5. Print the base (section 4) and assemble it (section 5).
6. Wire the base and the rover (section 7). Flash the base ESP32 (section 8). Set up the Jetson (section 9).
7. Calibrate (section 10) and bench test (section 11).
8. Run the staged tests, bench to field (section 12).

# 2. Safety

Read this before any hands-on step.

## Electrical

- **Battery unplugged before you touch, add or move any wire.** On the dart that means the power loop plug out. The
  first XIAO died from wiring with the battery live.
- Battery unplugged before you plug any board into the Mac by USB.
- Check for shorts with the meter on beep before the first power-on of anything you wired: red to black must not beep.
- The 3S LiPo can deliver over 100 A into a short. Keep the 20 A and 10 A fuses in their leads. Never bypass them.
- LiPo care: charge on the iMax B6 in balance mode, on a non-flammable surface, never unattended. Stop using the
  pack at 3.3 V per cell under load (9.9 V). A puffed pack goes in a LiPo bag and to battery recycling.
- 18350 cells: never let both cells touch metal at the same time. Charge out of the dart in the XTAR, or through the
  tail balance socket with the loop plug out.
- Never connect the DRV8833 motor supply (VM) to the LiPo directly: 10.8 V maximum. It runs from the 9.0 V buck.
- The dart VTX must have its antenna on before it is powered.

## Mechanical

- The wiper motor is strong (about 15 N m) and self-locking. Hands out of the gears, drum and rail while the battery
  is plugged in. Pull the battery plug to stop everything.
- The launch springs store energy. Cocked carriage = loaded. Never put a hand in front of the rail or the carriage.
  To unload a cocked base without firing, see troubleshooting.
- From the first spring test on: safety glasses, nobody in front of the rail, the area clear for 30 m.
- Augers and drills are sharp and turn with force. Hold the dart by the hull, never near a pod or the foot.
- Soldering: iron on its stand, fan on for fumes, 3 s at most per joint near PLA. Heat gun on low, no lighter near PLA.

# 3. Parts

Everything below is already in hand except the 608 bearings. Section 14 group C lists the items to confirm by
looking in the box.

## The only purchase: 608 bearings

- Search: "608ZZ bearing 8x22x7" (or "608-2RS", sealed). Buy 8 to 10. Skateboard bearings are 608s and fine.
- Specs: 8 mm bore, 22 mm outside, 7 mm wide, rated well over 10,000 rpm.
- Uses: storage reel (2), capstan or wrap arm (2), rollers and spares. Axles: M8 bolt or 8 mm rod (check the box).

## Base station and rover

| Part | Qty | Note |
|---|---|---|
| 12 V wiper motor, self-locking worm | 1 | Rebuilt to 2 wires, park linkage removed. Measure it first (section 14 A) |
| BTS7960 motor driver module | 1 | Drives the wiper |
| ESP32-S3 DevKitC-1-N8 | 1 | Base brain, USB to the Jetson |
| Adafruit AS5600 (STEMMA QT) + diametric magnet | 1 | Main shaft angle |
| DRV8833 carrier | 1 | DRV8833 #3, freed from the dart by the new dart pin map |
| MP1584 buck | 1 | MP1584 #2 from the dart build, set to 9.0 V |
| N20 right-angle worm motor | 1 | Selector drum (count your N20s: section 14 C3) |
| CSK20PP one-way bearings (20 x 47 x 14) | 2 | #1 spring cock and fire, #2 capstan backstop |
| Century 16 x 165.1 x 1.4 mm extension springs | 2 | Launch springs (Bunnings 0487961). Measure them (section 14 B) |
| 608 bearings | 8 to 10 | The purchase |
| Dyneema braid 150 lb, 1 mm | 20 m | Tether |
| 100 kg load cell + HX711 | 1 | Optional line tension |
| 3S LiPo 5200 mAh (Zeee), iMax B6 charger | 1 | Rover battery |
| XT60 lead #1 (10 A fuse), lead #2 (20 A fuse), spare XT60s, fuse holder | | Lead #2 feeds the wiper; build the Y harness from spares |
| Barrel plug lead 5.5 x 2.5 mm | 1 | Jetson from the LiPo |
| Jetson Orin Nano Super | 1 | |
| Lockmaner RC832 receiver, DIGITNOW USB capture, RCA lead | 1 each | Dart video in |
| Razer Kiyo X | 1 | Rover camera |
| Resistors 10 k x 3, 1 k, 680 R, 100 k, 22 k | | Pull-downs and dividers (firmware/base/WIRING.md) |
| M3 screws, M3 nuts, M3 brass heat-set inserts | | Frame joints and printed contacts |
| Dupont jumpers, 14 / 18 / 22 / 26 AWG wire, heat shrink | | |

## Dart (KEDGE v30)

The full dart list with quantities is in the dart run sheet, section 2 (appendix). In short: XIAO ESP32S3, 2 x
DRV8833 (the third goes to the base), INA219, BNO085, MP1584 #1 (5.04 V), OTOTEC 2S BMS, 2 x 18350 cells, 3 worm
N20s (kick, deploy, foot), 2 drill gearmotors, 5 x F693ZZ, Caddx Ant, Zeus VTX and antenna, JST-XH sockets,
Schottky diode, capacitor, M3 hardware, 15 g nose ballast.

## Tools

Soldering iron and solder, flux, multimeter, wire strippers, flush cutters, small flat screwdriver (MP1584 trimmer),
heat shrink and heat gun, caliper, hex keys, tweezers, kitchen scale, luggage scale, tape measure, safety glasses,
5-minute epoxy, threadlocker, zip ties, foam tape.

# 4. Printing

Printer: Bambu X2D (bed 256 x 256 x 260 mm). PLA for everything except loaded gears and keyed shafts, which are
PPA-CF (the print list says which).

Order: dart fit coupon first (20 minutes, checks your printer's hole sizes), then the dart plates, then the base
once the wiper and spring numbers are in (section 1, order of work).

## Dart prints

Dart plates (in `kedge/pack_v30/`, also inside `kedge/dist_v30/KEDGE_v30_build_pack.zip` under `print/`):

- `kedge/pack_v30/plate_A_hull.3mf`
- `kedge/pack_v30/plate_B_legs_tray.3mf`
- `kedge/pack_v30/fit_coupon.stl` (print this first, 20 minutes)

| Dart part | Print pose | Size on bed mm | Volume cm3 |
|---|---|---|---|
| hull_top | split face down (on the bed), saddle up | 121 x 50 x 35 | 17.41 |
| hull_bottom | split face down, belly up | 121 x 42 x 21 | 11.5 |
| nose | tip down on the bed (flat Ø22), spigot up | 42 x 42 x 27 | 8.25 |
| tail_cap | end face down, spigot up (no supports) | 42 x 42 x 15 | 7.13 |
| tray | keel down on the bed; discs stand up, decks bridge 26 mm between discs | 109 x 39 x 36 | 15.98 |
| kick_leg | on its side (pole axis horizontal, hub face down) | 119 x 15 x 15 | 8.7 |
| drill_leg_L | upright, motor pocket open to the top (as installed folded) | 46 x 15 x 13 | 3.37 |
| drill_leg_R | upright, motor pocket open to the top (as installed folded) | 46 x 15 x 13 | 3.37 |
| drill_leg_L_auger | base down, nut slot sideways | 17 x 17 x 73 | 5.5 |
| drill_leg_R_auger | base down, nut slot sideways | 17 x 17 x 73 | 5.5 |
| foot_auger | base down | 17 x 17 x 40 | 3.06 |
| coupler_L | standing on its socket end | 7 x 6 x 36 | 1.24 |
| coupler_R | standing on its socket end | 7 x 6 x 27 | 0.9 |

Settings: PLA, 0.4 nozzle, 0.16 mm layers, 4 walls, 40% gyroid infill, 5 top and bottom layers, supports off, brim on for the augers and couplers. Hole compensation from the fit coupon (run sheet section 4).


## Base prints

The base print pack comes from the base pipeline (`base/pack`). It holds every part already in print orientation,
the plates as 3MF, and the material for each part.


> NOT IN THE REPO YET: `base/pack` (written by cad-infra (print pack)). Re-run `python3 docs/build/make_guide.py` after it lands and this section fills in.


Before printing the base:

1. The wiper and spring numbers are in params.py and the pipeline was re-run (the audit said CLEAN).
2. Print one test part from the pack in PPA-CF and check a 608 drops into its seat with a firm thumb press, and an
   M3 heat-set insert sets straight. Adjust hole compensation the same way as the dart coupon.
3. PPA-CF: dry the spool first (it absorbs water), hardened nozzle, settings from the Bambu PPA-CF profile.

# 5. Base assembly

The assembly steps come from the base pipeline: every step is generated from the same model that passed the
collision audit, with a picture per step. They are in `base/guide` (PDF, also attached as an appendix) and
`base/out`.




> NOT IN THE REPO YET: `base/guide/README.md` (written by cad-infra (assembly steps)). Re-run `python3 docs/build/make_guide.py` after it lands and this section fills in.


General rules for the base:

- Heat-set inserts: soldering iron at about 220 C with the insert tip, press straight in, let it cool before loading.
- Bearings: press in square with a flat block, never hammer on the inner race.
- Keyed shafts: check the key slides in the sprag (CSK20PP) keyway by hand before final assembly.
- Turn every train by hand (battery unplugged) after each step: it must turn without binding before the next step.
- Leave the springs off until test stage 4 (section 12).

# 6. Dart build

Build the dart with the KEDGE v30 run sheet (appendix at the end of this guide, also
`kedge/pack_v30/KEDGE_v30_run_sheet.pdf`). It is a complete, step by step, timed build day: fit coupon, plates,
board prep, harness, power-up tests, motor directions, mechanical assembly, function test.

**One change from the run sheet: the dart pin map.** The run sheet uses three DRV8833 boards. The dart now uses two,
so the third can drive the base selector drum:

- DRV#1: deploy and kick (unchanged).
- DRV#2 channel A: BOTH drill motors (0.7 A stall together, fine on one 1.2 A channel), right drill motor leads
  swapped as before so the pair counter-rotates.
- DRV#2 channel B: the foot N20.
- No DRV#3 in the dart: skip every run sheet wire that goes to DRV#3, and do not bridge AIN1 to BIN1 on DRV#2.

Where the run sheet wiring (its sections 7 and 9 and its wiring picture) differs from `firmware/dart/WIRING.md`
below, follow WIRING.md. Use the firmware in `firmware/dart/`, not the copy inside the v30 pack. Ignore the run
sheet's `firmware/reel` sketch: the base firmware in `firmware/base/` replaces it.

The rest of the run sheet stands: the fit coupon, both plates, the 5.04 V buck, the Schottky diode, the capacitor
polarity, the battery contacts, the assembly order and the function test.

## Dart wiring (new pin map)


> NOT IN THE REPO YET: `firmware/dart/WIRING.md` (written by fw-dart). Re-run `python3 docs/build/make_guide.py` after it lands and this section fills in.



> NOT IN THE REPO YET: `firmware/dart/wiring/wiring.png` (written by another worker). Re-run `python3 docs/build/make_guide.py` after it lands and this section fills in.


### New dart wire cut list (replaces run sheet section 9)


> NOT IN THE REPO YET: `firmware/dart/wiring/wire_cut_list.csv` (written by another worker). Re-run `python3 docs/build/make_guide.py` after it lands and this section fills in.


## Dart bench tests



The run sheet sections 7.5, 7.6, 8d step 6 and 10 are the dart bench checks: power-up readings, motor directions,
battery power-up, then the function test in damp soil.

# 7. Electronics and wiring (base and rover)

Battery unplugged for every step in this section. Build in this order: the power harness (Y lead, fuses), the base
electronics bay, the sensors and contacts, the Jetson and video. Then do the first power-on sequence (7.1 section 4)
before anything is connected to a motor.

## 7.1 The whole system


*From `docs/build/SYSTEM_ELECTRICAL.md`:*

### GrappStar system electrical (rover, base and dart)

Diagram: `system_electrical.svg` / `system_electrical.png` (source `system_electrical.dot`). The ESP32 pin
table is owned by `firmware/base/WIRING.md` and `firmware/base/grappstar_base/pins.h`; this page shows how the
whole rover is powered and connected. If this page and WIRING.md ever disagree on a pin, WIRING.md wins.

#### 1. What is on the rover

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

#### 2. Power tree

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

#### 3. Wires and connectors

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

#### 4. First power-on sequence (do it in this order, once, after any rewiring)

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

#### 5. Rules

- Battery unplugged before you touch, add or move any wire. This killed the first XIAO.
- Battery unplugged before you plug the ESP32 into the Mac. Flash from the Mac with the ESP32 out of the rover
  harness, or flash from the Jetson (firmware/base/README.md).
- Stop using the LiPo at 3.3 V per cell under load (9.9 V); the firmware reports `vbat` and the Jetson warns.
- Never connect the DRV8833 VM to the LiPo directly (10.8 V maximum; the LiPo is 12.6 V full).
- Never put 3V3 on a DRV8833 pad called VIN, VCC or VM: on these carriers that is the motor supply. 3V3 only reaches
  the SLP / nSLEEP / EEP pad, and here it comes from an ESP32 GPIO.
- Dart: power loop plug out whenever a wire moves or USB is plugged in (the dart run sheet rule).
- The VTX on the dart must have its antenna on before it is powered.


![Whole system electrical diagram (bold = battery power, dashed = signal, dotted = radio)](docs/build/system_electrical.png)

## 7.2 Base ESP32 wiring (pin by pin)


> NOT IN THE REPO YET: `firmware/base/WIRING.md` (written by fw-base). Re-run `python3 docs/build/make_guide.py` after it lands and this section fills in.


# 8. Flashing both ESP32s

Rules for both boards: battery unplugged (dart: power loop plug out) before the USB cable goes in. Use a data USB
cable. Flash the dart XIAO before it goes into the hull (run sheet section 5).

## 8.1 Base: ESP32-S3 DevKitC-1-N8


> NOT IN THE REPO YET: `firmware/base/README.md` (written by fw-base). Re-run `python3 docs/build/make_guide.py` after it lands and this section fills in.


## 8.2 Dart: XIAO ESP32S3

Arduino IDE: board XIAO_ESP32S3, Tools > USB CDC On Boot: Enabled (the sketch refuses to compile otherwise).
Libraries: Adafruit INA219, Adafruit BNO08x. Open `firmware/dart/kedge_dart/kedge_dart.ino`. After the first USB
upload you can upload over WiFi (port "kedge").


> NOT IN THE REPO YET: `firmware/dart/README.md` (written by fw-dart). Re-run `python3 docs/build/make_guide.py` after it lands and this section fills in.


# 9. Jetson setup

The Jetson already runs JetPack 6.2 (Ubuntu 22.04, Python 3.10) from the SD card with WiFi and SSH. Keep numpy
below 2 and do not pip install opencv-python (both broke things in August).


> NOT IN THE REPO YET: `jetson/README.md` (written by jetson). Re-run `python3 docs/build/make_guide.py` after it lands and this section fills in.


# 10. Calibration (base)

Do it after the base is assembled and wired, springs off, before test stage 3. The values are stored on the ESP32.


> NOT IN THE REPO YET: `firmware/base/CALIBRATION.md` (written by fw-base). Re-run `python3 docs/build/make_guide.py` after it lands and this section fills in.


# 11. Bench tests

## 11.1 Base


> NOT IN THE REPO YET: `firmware/base/BENCH_TEST.md` (written by fw-base). Re-run `python3 docs/build/make_guide.py` after it lands and this section fills in.


## 11.2 Dart



The dart bench checks are in the run sheet (appendix): section 7.5 (power-up readings), 7.6 (motor directions),
8d step 6 (battery power-up) and section 10 (function test and calibration in damp soil). With the new pin map the
foot is on DRV#2 channel B: key 4 must still turn the foot shaft clockwise seen from the shaft end.

# 12. First tests, bench to field


*From `docs/build/TEST_PLAN.md`:*

## First tests: bench to field

Do the stages in order. Do not skip ahead when a stage fails: fix it, then repeat the stage. Write the date and the
result of each stage in a notebook (or `docs/build/test_log.md`).

How to send base commands: the operator page on the Jetson (jetson/README.md) has buttons for most of these. From a
terminal on the Jetson, in `~/GrappStar/jetson`, send any command (firmware/base/PROTOCOL.md) with
`python3 -m grappstar base cmd '{"cmd": "jog", "motor": "wiper", "dir": 1, "ms": 300}'` and watch telemetry with
`python3 -m grappstar base monitor` in a second terminal. `'{"cmd": "estop"}'` stops everything and is always safe
to send.

Safety for every stage:
- Battery unplugged before you touch any wire.
- Hands out of the gears, the drum and the rail whenever the battery is plugged in.
- From stage 5 on: nobody in front of the rail, safety glasses on, the launch area clear for 30 m.
- Keep a hand near the battery plug. Pulling it is the real emergency stop.

### Stage 0. Power-on (no motors attached)

Follow SYSTEM_ELECTRICAL.md section 4, the first power-on sequence. Both motor leads disconnected from the drivers.

Pass: no beeps on the short check, MP1584 #2 reads 9.0 V, the Jetson boots, `status` replies, `vbat` within 0.3 V
of the meter, `fault` is null.
If it fails: a beep means a short (check the Y harness and the BTS7960 screws first). No reply: check the USB
cable is a data cable and in the ESP32 port marked USB; see troubleshooting in the build guide.

### Stage 1. Each motor alone, out of the base

Battery unplugged. Connect the wiper motor to BTS7960 M+ / M- and clamp the motor to the bench. Connect the drum
N20 to DRV8833 #3 AOUT1 / AOUT2, loose on the bench. Battery in.

1. `jog` wiper dir 1 for 300 ms, then dir -1. Mark which way the shaft turns for dir 1.
2. `jog` drum dir 1 for 500 ms, then dir -1. Mark the shaft direction.
3. Watch `amps` during each wiper jog.

Pass: each motor turns both ways, stops when the jog ends, wiper free current about 1 to 3 A (A6 in
MEASURE_LIST.md), no fault.
If it fails: no movement on the wiper: check R_EN / L_EN wiring and the 20 A fuse. Wiper turns only one way: one of
RPWM / LPWM is not connected. Drum does nothing: check SLP wiring and 9.0 V on VM. Wrong direction against
CALIBRATION.md: swap that motor's two leads (battery unplugged).

### Stage 2. Sensors alone

1. Turn the main shaft (or the magnet) by hand: `wiper_deg` changes smoothly, no jumps, and goes up when the shaft
   turns the FORWARD way.
2. Touch each printed contact closed with a finger: its entry in `sw` changes and changes back when released.
3. Load cell (if fitted): press it, `tension_n` rises.

Pass: every sensor responds and returns. If it fails: AS5600 not found: check SDA / SCL are not swapped and the
STEMMA cable is pushed in; angle jumps: magnet too far (keep 1 to 3 mm) or off centre. Contact never changes:
check the GND wire and that the screw head really touches the nut.

### Stage 3. Base assembled, dry cycles, NO springs, no line

Springs off, no tether, no dart. Run CALIBRATION.md first (zeros and drum positions).

1. `mode` LAUNCH, NEUTRAL, REEL, NEUTRAL, LAUNCH: 20 full rounds. Each change ends with the right `drum_pos`.
2. `aim` 15, 30, 45, then the minimum and maximum pitch: 5 rounds. Rail reaches each angle within 1 degree
   (`pitch_deg`) and holds with the motor off.
3. Cock and fire with no springs: `arm`, then `fire` with the token. The carriage travels back, releases, and the
   big gear re-engages: 10 cycles.
4. REEL mode: `wrap`, then `unwrap`: 10 cycles. Then `reel` at duty 0.3 for 3 s with no line.
5. Send `estop` in the middle of each kind of move once. The wiper stops at once and the drum goes to NEUTRAL.
6. Fault test: unplug the AS5600 cable while idle (battery can stay in for this one, nothing moves).
   The base reports `ANGLE_SENSOR` and refuses to move. Plug back in, `clear`.

Pass: all cycles complete with no fault, no clicking or skipping teeth, no part warm to the touch, the drum never
stops BETWEEN positions.
If it fails: a STALL fault means something binds: find it by turning by hand with the battery unplugged. Skipped
teeth: check the mesh spacing and the frame screws. Drum stops BETWEEN: re-run the drum calibration; check the drum
contacts close cleanly.

### Stage 4. Springs on, no dart

Fit both springs. Start at the lowest pitch, rail pointing at a clear area.

1. `aim` to the lowest pitch. `arm` and `fire` once. Look at every part of the launch train after the shot.
2. Five more shots. Check the stop bumpers, the carriage, the rack teeth and the big gear's last tooth after each.
3. Watch `amps` while cocking: note the peak.

Pass: the carriage stops on the bumpers, nothing cracks or loosens over 6 shots, cocking current stays under the
stall value, the rail holds its pitch while cocking.
If it fails: cracks at the bumpers: stop and report (no more dry shots). Rail drops while cocking: the aim worm is
slipping, check its mesh. STALL while cocking: the springs are stronger than planned (redo B2) or something binds.

### Stage 5. Dummy dart, no tether

Make the dummy: the plate A hull top and bottom, nose and tail cap (spare prints or the dart shells before the
electronics go in), taped shut with M8 nuts or sand inside to 280 g. Paint the nose bright.

1. `aim` 20 degrees. Load the dummy, check `SW_SEATED` is closed. Fire.
2. Film it side-on with the phone in slow motion, tape measure on the ground in view (muzzle speed, G5).
3. Measure the range. Repeat at 30 and 45 degrees (G6).

Pass: clean exit (no rubbing marks on the dummy), it flies nose first, range within 30 percent of the physics
model's no-tether range (`base/physics/SUMMARY.md`, "Same shot, no tether": 5.2 m at 35 degrees before the springs
were measured).
If it fails: tumbles: check the nose ballast is in the dummy too (it must match the real dart). Short range:
measure the spring (B2) and the muzzle speed, then re-run the physics. Rubbing marks: check the cradle and the
barrel against the dart.

### Stage 6. Tether on

Tie the line through the dummy's tail cap holes. Line routed as in the base assembly steps (it bypasses the
capstan for launch). Storage reel brake set: pull the line by hand with the luggage scale, 50 to 75 g (G4).

1. Fire at 30 degrees.
2. Check the line paid out with no loops or overrun on the reel.
3. `mode` REEL, `wrap`, then `reel` until `SW_SEATED` or the dummy is back near the base.
4. `unwrap`, `mode` LAUNCH. Check the line runs on the launch path again.

Pass: payout is clean, the range is at least 65 percent of stage 5 at the same pitch (the physics model expects
about 72 percent: 3.8 m against 5.2 m at 35 degrees), the wrap lays on the capstan, reel-in
works, unwrap returns the line to the launch path. Repeat 5 times.
If it fails: backlash loops on the reel: raise the payout drag a little (do not exceed 1 N). Big range loss: the
drag is too high or the line catches; watch the slow-motion video. Wrap misses the capstan: check the wrap arm
position at the end of `wrap`.

### Stage 7. The anchor in soil

The real dart, built and tested to the dart run sheet section 10. Use real soil (moist loam is the target).

1. Place the dart on the soil by hand, tether tied, line slack. Send `/auto` from the operator page (or key `a`).
2. When the dart reports ANCHORED: pull the tether by hand with the luggage scale, slowly, up to 50 N or until
   it moves (G7).
3. Re-anchor. Base in REEL with wraps on: `reel` with `until` tension at a limit 10 N under the hand result.
4. Send `/release`: drills out, foot out, kick back, deploy up. Then reel the dart home.

Pass: the dart anchors on its own, holds at least 20 N in loam, the base stops at its tension limit, the release
sequence frees the dart, it reels home.
If it fails: augers back out: swap that motor's leads (dart run sheet section 10). Pulls out at low force: try
firmer soil, and report the force (it sets the reel limit). Base overshoots the limit: redo the current
calibration (G1).

### Stage 8. Full mission

From the operator page on the Jetson (jetson/README.md): preflight, pick a target on the Kiyo view, aim, confirm
the fire prompt, flight video from the dart, landing verdict, anchor, tow, release, reel home.

1. First run: a short target well inside the range the operator page offers (the physics model gives about 4 m at
   35 degrees with the current springs), soft soil, open ground.
2. Watch every prompt: the software stops and asks before each dangerous step.

Pass: the whole sequence runs with only the expected operator confirmations, the dart lands within 2 m of the target,
anchors, the rover is pulled toward the dart, the dart releases and is reeled home.
If it fails: note the mission step and the fault code in the log (`mission.jsonl` on the operator page), then
repeat the stage where that step was first tested.


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
| Fault DRUM_TIMEOUT | Drum N20 not turning: no 9 V, SLP not high, gear slipping | Meter on DRV8833 VM (9.0 V); check the SLP wire; check the N20 gear set screw |
| Fault DRUM_SENSOR or SWITCH_DISAGREE | A drum contact does not close or stays closed | Watch `sw` while turning the drum by the hand lever (battery unplugged for the lever, then plug in to read); bend or re-screw the contact |
| Fault ANGLE_SENSOR | AS5600 cable out, magnet too far | Re-seat the STEMMA cable; magnet 1 to 3 mm from the chip, centred |
| `pitch_deg` wrong after a restart | Pitch reference lost | Run the pitch part of the calibration again (section 10) |
| Drum stuck between positions with the power off | Expected: the worm holds it | Use the hand lever on the drum shaft end, battery unplugged |
| Base cocked and you need to unload without firing | | Clear the area and fire it at the lowest pitch with no dart. If the base cannot fire, battery unplugged, and follow the unload step in the base assembly steps; never hand-release the carriage |

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

# 14. Measure list


*From `docs/build/MEASURE_LIST.md`:*

## What to measure or confirm

Everything nobody could know from here. Do group A and B first: the base parts that depend on them should not
be printed until those numbers are in `base/spec/params.py` and the pipeline has been re-run (see "After you
measure" at the end). Tools: caliper, multimeter, kitchen scale, luggage scale, phone camera, tape measure.

Photo rule: for every caliper reading, take a photo with the caliper on the part and the reading visible. Put the
photos in `originals/measure_2026-11/` so the numbers can be checked later.

### A. Wiper motor (params.py `WIPER`, set `measured=True` when done)

| # | What | How | Changes | Reprint after |
|---|---|---|---|---|
| A1 | Output shaft diameter | Caliper across the round part of the shaft, 3 places, take the smallest | `WIPER.shaft_d` | Wiper shaft coupler (drum parts) |
| A2 | Shaft flats or spline | Flats: caliper across the flats (`shaft_flat`). Spline: count the teeth, caliper the outside and root diameter, photo end-on | `WIPER.shaft_flat` (note a spline in REPORT) | Wiper shaft coupler |
| A3 | Shaft end thread and length | Thread a nut you know (M6, M8) onto it. Caliper the shaft length from the housing face to the tip | `WIPER.shaft_thread`, `WIPER.shaft_len` | Wiper shaft coupler |
| A4 | Mount hole pattern | Lay the motor face down on paper, press so the holes mark it (or trace with a pencil). Then caliper each hole centre from the shaft centre along two square directions and each hole diameter. Photo of the paper with a ruler | `WIPER.mount_holes` as (x, y, dia) from the shaft axis | Motor adapter plate (frame parts) |
| A5 | Housing size | Caliper the gearbox box (length, width, height) and the motor can (diameter, length), and how far the shaft sits from each edge | `WIPER.body_box` | Frame cheeks and guard if they clash (the audit says) |
| A6 | Free speed and free current at 12 V | Motor clamped, nothing on the shaft, LiPo through lead #2 (20 A fuse). Meter on the 10 A range in series with the red wire. Tape flag on the shaft: count turns for 30 s, rpm = turns x 2. Note the amps | `WIPER.n0_rpm`, `WIPER.free_a` | Nothing; re-run physics |
| A7 | Stall current and stall torque | Do NOT use the meter: stall is about 18 A, over the meter's 10 A range. After base calibration (firmware/base/CALIBRATION.md), use the `amps` telemetry. Torque: clamp a 300 mm bar on the shaft, hook the luggage scale at the end, jog the motor for at most 2 s at full duty and read the peak. Torque N m = kg x 9.81 x 0.30 | `WIPER.stall_a`, `WIPER.stall_nm` | Nothing; re-run physics and the ballistics table |

### B. Launch springs (params.py `SPRING`, set `measured=True` when done)

The two springs are extension springs with initial tension: they need a small force before they stretch at all, so
one weight is not enough. Use three loads. Every range number scales with the energy the springs store, so this is the most important measurement.

| # | What | How | Changes | Reprint after |
|---|---|---|---|---|
| B1 | Free length L0 | Caliper the spring hook to hook (inside of the hooks), no load | `SPRING.free_len` | Nothing |
| B2 | Rate k and initial tension F0 | Use water bottles in a shopping bag, weighed on the kitchen scale: about 1, 2 and 3 kg (W1, W2, W3 in kg). Hang each from ONE spring and measure hook to hook (L1, L2, L3). Plot or compute: k (N/mm) = (W3 - W1) x 9.81 / (L3 - L1). F0 (N) = W1 x 9.81 - k x (L1 - L0). The three points should lie on a line; if L1 equals L0 (the spring did not move), drop W1 and add a 4 kg load. Do both springs. Never stretch a spring longer than L0 + 250 mm | `SPRING.k_n_per_mm`, `SPRING.F0_n` | Nothing unless the stroke changes (physics says); re-run physics and the ballistics table |
| B3 | Wire and outside diameter | Caliper | `SPRING.wire`, `SPRING.od` | Spring anchors only if OD differs by more than 0.5 mm |

### C. Parts box: confirm you have these (no new purchases)

| # | Item | Needed for | If you do not have it |
|---|---|---|---|
| C1 | 608 bearings (8 to 10) | storage reel, capstan or wrap arm, cam roller | The only purchase: search "608ZZ bearing 8x22x7" |
| C2 | M8 bolts or 8 mm steel rod, and their lengths | axles through the 608s | Printed PPA-CF axles (tell the coordinator, it is a params switch) |
| C3 | Worm N20 count: you need 4 (dart kick, deploy, foot, and the base selector drum) | dart and base | The kedge notes said 2 in hand and one to buy; the August order listed 8 worm N20s. Count them |
| C4 | MP1584 #2 (the spare from the dart build) | drum N20 supply at 9.0 V | No other way to run the DRV8833 from the 3S: tell the coordinator |
| C5 | DRV8833 #3 (freed from the dart by the new pin map) | drum N20 | |
| C6 | 100 kg load cell + HX711 (August order) | optional line tension reading | Leave it out: the firmware uses wiper current instead |
| C7 | Resistors: 3 x 10 k, 1 x 1 k, 1 x 680 R, 1 x 100 k, 1 x 22 k | BTS7960 enable pull-downs, DRV8833 SLP pull-down, IS divider, battery divider (firmware/base/WIRING.md) | Any value within 20 percent works for the pull-downs; tell fw-base if the dividers differ |
| C8 | Spare XT60 plugs and a fuse holder, 14 AWG wire | XT60 Y harness | |
| C9 | Barrel plug 5.5 x 2.5 mm with lead | Jetson from the LiPo | Run the Jetson from its own supply on the bench; the rover needs one |
| C10 | Yellow RCA lead | RC832 AV out to the DIGITNOW | |
| C11 | USB data cable for the DevKitC, STEMMA QT to Dupont cable | base ESP32, AS5600 | |
| C12 | M3 screws (lengths from the base print list), M3 nuts, M3 heat-set inserts | printed contacts, frame joints | |

### D. Electrical labels to read

| # | What | How | Changes |
|---|---|---|---|
| D1 | RC832 input voltage | Read the label or the listing. Most take 7 to 24 V. If it says 12 V only, it still runs from the 3S (12.6 V full) but report it | SYSTEM_ELECTRICAL.md run P8 |
| D2 | Jetson DC input range | Label on the Jetson's own supply and the NVIDIA dev kit page. The 3S gives 9.9 to 12.6 V in use | SYSTEM_ELECTRICAL.md run P7 |
| D3 | DRV8833 carrier pad names | Which pad is SLP / nSLEEP / EEP on your boards. Meter on beep: that pad must NOT beep to VM / VIN | firmware/base/WIRING.md |
| D4 | DevKitC board version | Printed on the back (v1.0 or v1.1). The RGB LED pin differs; the firmware avoids both | Nothing |
| D5 | Video standard | When the dart video shows on the Jetson, note the size (720 x 480 = NTSC, 720 x 576 = PAL) | jetson config |

### E. Rover

| # | What | How | Changes | Reprint after |
|---|---|---|---|---|
| E1 | Deck size and mounting holes | Tape measure the free deck area; caliper the hole positions and diameters from one corner | params.py `FRAMEWORK` (rover mount) | Base deck / rover mount |
| E2 | Drive train and whether the Jetson drives it | Tell the coordinator what drives the wheels and what controls it | jetson rover module | Nothing |
| E3 | Kiyo X height and tilt on the rover | Tape measure lens to ground; phone level app on the camera for tilt | jetson camera model (target calibration) | Nothing |
| E4 | Base launch height | Tape measure from the ground to the main shaft axis when the base is mounted | physics (ballistics table) | Nothing |

### F. Dart (from the KEDGE notes, still open)

| # | What | How | Changes |
|---|---|---|---|
| F1 | Dart v30 is the one being built | Confirm | print pack choice |
| F2 | Bought part weights and the finished dart weight | Kitchen scale | `DART.mass_kg`, dart physics |
| F3 | Stall current of each dart motor at 2S | The dart `i` key while holding the shaft with pliers for 1 s | firmware/dart tuning values (see firmware/dart/README.md) |
| F4 | OTOTEC BMS size and F693ZZ flange | Caliper (pocket is 40.8 x 8 x 2.1 plus 0.5 each way) | dart geometry if it does not fit |
| F5 | Nose ballast | Weigh it: 15 g | flight stability |

### G. Bench and field numbers (measured during the TEST_PLAN stages)

| # | What | How | Changes |
|---|---|---|---|
| G1 | BTS7960 current sense scale | firmware/base/CALIBRATION.md (meter in series at low duty, compare with `amps`) | firmware `set` key (stored on the ESP32) |
| G2 | Battery voltage reading | Meter on the LiPo vs `vbat` telemetry | firmware `set` key |
| G3 | AS5600 zero and drum positions | firmware/base/CALIBRATION.md | stored on the ESP32 |
| G4 | Storage reel payout drag | Pull the line off the reel slowly with the luggage scale in LAUNCH mode: set 0.5 to 0.75 N (50 to 75 g). Above 1 N costs range, 3 N ruins the flight | slip clutch preload (reel parts) |
| G5 | Muzzle speed | Phone slow-motion video side-on with a tape measure in view: speed = distance moved / time between frames | physics model, ballistics table |
| G6 | Range at three pitches | Tape measure from the base to the dart at 3 pitches, no tether brake changes | ballistics table check (jetson uses it) |
| G7 | Soil holding force | Dart anchored, luggage scale on the tether, pull slowly up to 50 N, note where it moves | reel tension limit |
| G8 | Rail friction | Before the rail is mounted: lay the rail part on a flat board, dart in the cradle, lift one end of the board slowly until the dart slides; phone level app on the board. Friction = tan(angle) | physics (launch efficiency) |

### After you measure

1. Write the numbers into `base/spec/params.py` (set `measured=True` in `WIPER` and `SPRING`) or send them to the
   coordinator. Put photos in `originals/measure_2026-11/`.
2. Re-run the base pipeline (assembly, audit must say CLEAN, physics, ballistics table, print pack) and
   `firmware/base/tools/gen_params.py`, then re-flash the base ESP32.
3. Reprint only the parts named in the "Reprint after" column. If you have not printed them yet, print after step 2.


## Numbers still assumed in params.py right now


params.py version: `0.1-stub`.

| params.py name | State | Who fixes it |
|---|---|---|
| WIPER | assumed, not measured yet | MEASURE_LIST |
| SPRING | assumed, not measured yet | MEASURE_LIST |
| LAYOUT | not decided yet (None) | coordinator |
| DRUM | not decided yet (None) | coordinator |
| LAUNCH | not decided yet (None) | coordinator |
| AIM | not decided yet (None) | coordinator |
| REEL | not decided yet (None) | coordinator |
| FRAMEWORK | not decided yet (None) | coordinator |
| SWITCHES | not decided yet (None) | coordinator |
| TIMELINE | not decided yet (None) | coordinator |


# 15. Sources and appendices

This guide is built by `docs/build/make_guide.py` from the files below. A "NOT YET" row is a section that still
shows a placeholder; re-run the script after it lands.


| Source | Written by | In the repo |
|---|---|---|
| `base/spec/BASE_SPEC.md` | coordinator | yes |
| `base/pack` | cad-infra (print pack) | NOT YET |
| `base/out` | cad-infra (assembly pipeline) | NOT YET |
| `base/guide` | cad-infra (assembly steps) | NOT YET |
| `kedge/pack_v30/KEDGE_v30_run_sheet.pdf` | dart v30 pack (done) | yes |
| `firmware/base/WIRING.md` | fw-base | NOT YET |
| `firmware/base/README.md` | fw-base | NOT YET |
| `firmware/base/CALIBRATION.md` | fw-base | NOT YET |
| `firmware/base/BENCH_TEST.md` | fw-base | NOT YET |
| `firmware/base/PROTOCOL.md` | fw-base | NOT YET |
| `firmware/dart/WIRING.md` | fw-dart | NOT YET |
| `firmware/dart/README.md` | fw-dart | NOT YET |
| `firmware/dart/API.md` | fw-dart | NOT YET |
| `firmware/dart/BENCH_TEST.md` | fw-dart | NOT YET |
| `jetson/README.md` | jetson | NOT YET |


Appendices attached after this page (when present): the base assembly steps PDF from `base/guide`, then the KEDGE
v30 dart run sheet. In the run sheet, follow section 6 of this guide for the new dart pin map.

Other documents in the repo: `kedge/pack_v30/KEDGE_v30_engineering_report.pdf` (dart physics and loads),
`base/spec/BASE_SPEC.md` (base design), `firmware/base/PROTOCOL.md` (base serial protocol), `firmware/dart/API.md`
(dart HTTP API), `jetson/INTERFACES.md` (Jetson software).
