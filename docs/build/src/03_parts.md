# 3. Parts

Everything below is already in hand except the 608 bearings (and possibly the AS5600 magnet). Section 14 group C lists the items to confirm by
looking in the box.

## The only purchase: 608 bearings

- Search: "608ZZ bearing 8x22x7" (or "608-2RS", sealed). Buy 10. Skateboard bearings are 608s and fine.
- Specs: 8 mm bore, 22 mm outside, 7 mm wide, rated well over 10,000 rpm.
- Uses: the base CAD uses 5 (storage reel and aim pedestal); the coupons borrow some for testing; the rest are spares. Axles: M8 bolt or 8 mm rod (check the box).

## Possible extra purchases

- **AS5600 magnet** (optional): the Adafruit board does not include one. The magnet cap takes any small neodymium
  disc (6 to 10 mm across, 2 to 3 mm thick) mounted edge-on, or a 6 x 2.5 mm diametric disc. Look in the box first
  (old speakers, fridge magnets with a neodymium disc). Without a magnet the firmware has a less precise fallback (decision D12):
  a printed 6-bump index cam read by the contact SW_INDEX on GPIO35, switched on in firmware/base config.h.
- Not a purchase: a second pair of launch springs. The CAD has holes for them, but the drive train is not rated
  for four springs (decision D38). v1 uses the two springs you have.

## Base station and rover

| Part | Qty | Note |
|---|---|---|
| 12 V wiper motor, self-locking worm | 1 | Rebuilt to 2 wires, park linkage removed. Measure it first (section 14 A) |
| BTS7960 motor driver module | 1 | Drives the wiper |
| ESP32-S3 DevKitC-1-N8 | 1 | Base brain, USB to the Jetson |
| Adafruit AS5600 (STEMMA QT) | 1 | Main shaft angle. Needs a small magnet (see above) |
| DRV8833 carrier | 1 | DRV8833 #3, freed from the dart by the new dart pin map |
| MP1584 buck | 1 | MP1584 #2 from the dart build, set to 9.0 V |
| N20 right-angle worm motor | 1 | Selector drum (count your N20s: section 14 C3) |
| 693ZZ / F693ZZ bearings | as many as are left after the dart | Line pulleys (the dart uses 5 F693ZZ; count what is left, section 14 C20) |
| CSK20PP one-way bearings (20 x 47 x 14) | 2 | #1 spring cock and fire, #2 capstan backstop |
| Century 16 x 165.1 x 1.4 mm extension springs | 2 | Launch springs (Bunnings 0487961). Measure them (section 14 B) |
| 608 bearings | 10 | The purchase |
| Dyneema braid 150 lb, 1 mm | 20 m | Tether |
| 100 kg load cell + HX711 | 1 | Optional line tension |
| 3S LiPo 5200 mAh (Zeee), iMax B6 charger | 1 | Rover battery |
| XT60 lead #1 (10 A fuse), lead #2 (20 A fuse), spare XT60s, fuse holder | | Lead #2 feeds the wiper; build the Y harness from spares |
| Barrel plug lead 5.5 x 2.5 mm | 1 | Jetson from the LiPo |
| Jetson Orin Nano Super | 1 | |
| Lockmaner RC832 receiver, DIGITNOW USB capture, RCA lead | 1 each | Dart video in |
| Razer Kiyo X | 1 | Rover camera |
| Resistors 10 k x 2, 1 k, 680 R, 100 k, 22 k | | Pull-downs and dividers (firmware/base/WIRING.md) |
| M3 screws, M3 nuts, M3 brass heat-set inserts | | Frame joints and printed contacts |
| Dupont jumpers, 14 / 18 / 22 / 26 AWG wire, heat shrink | | |

## Dart (KEDGE v30)

The full dart list with quantities is in the dart run sheet, section 2 (`kedge/pack_v30/KEDGE_v30_run_sheet.pdf`). In short: XIAO ESP32S3, 2 x
DRV8833 (the third goes to the base), INA219, BNO085, MP1584 #1 (5.04 V), OTOTEC 2S BMS, 2 x 18350 cells, 3 worm
N20s (kick, deploy, foot), 2 drill gearmotors, 5 x F693ZZ, Caddx Ant, Zeus VTX and antenna, JST-XH sockets,
Schottky diode, capacitor, M3 hardware, 15 g nose ballast.

## Tools

Drill with 3 mm and 3.4 mm bits and a deburring tool (rover deck holes), an M3 heat-set insert tip for the
soldering iron, a 13 mm socket with a 1/4 inch driver (coupler M8 nut). Soldering iron and solder, flux, multimeter, wire strippers, flush cutters, small flat screwdriver (MP1584 trimmer),
heat shrink and heat gun, caliper, hex keys, tweezers, kitchen scale, luggage scale, tape measure, safety glasses,
5-minute epoxy, threadlocker, zip ties, foam tape.

## Base bill of materials (from the model)

@include base/out/base/BOM.md
