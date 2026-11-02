# Frame subsystem notes (worker cad-frame)

Status: phase 0 (3 Nov 2026, 11:00 Sydney). Code: `frame.py` (API module), helpers `_frame_util.py`,
`_frame_motor.py`, `_frame_ebay.py` (leading underscore so `tools.assembly` does not load them as subsystems).
`python -m tools.audit OUT --only frame` says CLEAN on the placeholder layout (37 parts, 296 states).

## 1. What the frame builds (all from params; placeholders until BASE_SPEC.md)

| Part | What | Material |
|---|---|---|
| `frame_deck` (or `frame_deck_1..n`) | flat plate, 8 thick, z -8..0. Split into dovetailed tiles only when over 248 mm. M3 heat-set inserts for every frame foot. Perimeter M3 holes for the rover (the deck is its own drill template) | PLA |
| `frame_cheek_r`, `frame_cheek_l` | vertical plates, 10 thick, inner faces at y = +-cheek_gap/2 (124 default), outline = hull of the bearing blocks, outboard foot flange with gussets, M3 down into the deck | PLA |
| `frame_blk_<shaft>_<l/r>` | bearing block on the cheek outer face, spigot keyed into the cheek hole, 4 x M3 into cheek inserts. Seat types: `608` (crush ribs), `sprag` (CSK20PP OD 47), `bush` (printed bushing for 20 mm printed shafts), or dict(od, w) | PLA, PPA-CF for main / launch / capstan |
| `frame_tie_<n>` | cheek spacers, M3 through each cheek into an insert at each end. Auto-placed in free cheek area (or `FRAMEWORK["ties"]`) | PLA |
| `frame_wiper_motor`, `frame_wiper_shaft` | bought envelope + rotating output stub (joint `frame_wiper`, value = `wiper_deg`) | bought |
| `frame_motor_adapter` | separately printed adapter (see section 3) | PPA-CF |
| `frame_motor_saddle` | strap saddle under the can end, 4 x M3 into the deck | PLA |
| `frame_ebay`, `frame_ebay_lid`, `frame_ebay_esp_clamp` | electronics bay (section 4) | PLA |
| `frame_pcb_*`, `frame_fuse_20a` | board envelopes | bought |
| `frame_as5600_bracket`, `frame_as5600_magnet_cap`, `frame_as5600_magnet`, `frame_pcb_as5600` | angle sensor on the main shaft end (section 5) | PLA / bought |

Fasteners: M3 only (M3 x 4 x 5 heat-set inserts, holes 4.0 x 6 deep), plus M6 for the wiper bosses (motor bolts).
Rover: M3 x 25 SHCS + nyloc through the deck perimeter holes.

## 2. Board data (looked up 3 Nov 2026)

| Board | Size (mm) | Mounting | Source |
|---|---|---|---|
| ESP32-S3-DevKitC-1 (v1.0, v1.1) | PCB 25.40 x 62.87 (drawing label 62.74), module antenna 6.3 past the top edge (69.2 overall). 2 x 22 pins, 2.54 pitch, rows 22.86 apart, first pin 7.96 from the USB edge | NO mounting holes: held in a printed cradle (pins down for DuPont jumpers) with a hold-down over the module | Espressif user guide v1.1 and DXF (dl.espressif.com/dl/schematics/esp_idf/DXF_ESP32-S3-DevKitC-1_V1.1_20220429.dxf) |
| ESP32 USB | USB edge: UART port centre x 6.0, native USB ("USB", J4) centre x 19.4. Official boards are Micro-USB; clones are USB-C. Native USB alone powers the board (Espressif) | one wall slot clears both ports and either plug | same |
| BTS7960 IBT-2 | PCB 49.66 x 49.40, 43 tall overall incl. heatsink (heatsink under the PCB, height NOT published: 22 assumed) | 4 x M3 on a 40 x 40 square: standoffs with inserts | Handsontec BTS7960 datasheet drawing |
| BTS7960 pins | screw terminal B+, B-, M+, M- (one edge); logic 2 x 4: RPWM, LPWM, R_EN, L_EN, R_IS, L_IS, VCC, GND | | same |
| Adafruit AS5600 (PID 6357) | 25.4 x 17.78, chip at the board centre (12.7, 8.89) | 4 x 2.5 holes at 2.54 from each edge (20.32 x 12.70): printed pegs, heat-staked | Adafruit Eagle files (github.com/adafruit/Adafruit-AS5600-Magnetic-Angle-Sensor-PCB) |
| AS5600 magnet | **NOT included with the Adafruit board** (the add-on magnet PID 9 is a 12 x 4.8 axial disc, wrong type). Need a **6 x 2.5 mm DIAMETRIC NdFeB disc**, air gap 0.5 to 3 mm (design: 1.0) | glued in the cap pocket | adafruit.com/product/6357, AS5600 datasheet |
| Pololu DRV8833 (2130) | 20.32 x 12.70, no holes | headers off, wires soldered, floor pocket with solder relief | pololu.com/product/2130 |
| MP1584EN buck | 22.26 x 16.96 x 3.98 (Harry's calipers), no holes, IN one end, OUT the other | floor pocket | kedge/CLAUDE.md section 2 |
| HX711 (optional) | generic 34 x 21 x 3 (seller); SparkFun 13879 30.48 x 22.86 | floor pocket fits both | SparkFun v11 Eagle files |
| 20 A fuse | inline ATO holder envelope 55 x 26 x 16 (typical). 14 AWG holder: Littelfuse FHAC0003ZXJB (FHAC0001ZXJ has 16 AWG leads) | cradle with a tie slot | littelfuse.com |
| XT60 | body 15.7 x 8.1; panel cutout 16.5 x 9 | wall cutout | seller data |

## 3. Wiper motor adapter (Harry edits only params.WIPER, reprints one part)

Fixed frame interface (never changes): on the motor-side cheek (`FRAMEWORK["motor_side"]`, default -1 = right) a
44 mm hole on the main axis and 6 x M3 inserts on R 29. The adapter hub stands the plate off the cheek by
(thickest block flange on that cheek + 1.5) so the plate clears neighbouring blocks and its M6 nut channels stay
reachable after mounting.

Motor frame: origin on the output shaft axis in the plane of the boss faces, +Z along the shaft (into the frame),
+X along the gearbox towards the can. Keys read from `params.WIPER` (any non-None value overrides the default):

| key | default (typical car wiper, NOT measured) | meaning |
|---|---|---|
| `mount_holes` | [(-24, 38, 6.5), (-24, -38, 6.5), (72, 0, 6.5)] | (x, y, dia) of each boss hole |
| `boss_d`, `boss_h` | 16, 8 | boss diameter, height above the gearbox face |
| `body_box` | (-38, 84, -40, 40, 56) | gearbox x0, x1, y0, y1, depth behind the gearbox face |
| `can_d`, `can_len`, `can_x0` | 58, 95, 84 | motor can |
| `hub_d`, `hub_h` | 24, 4 | output bushing boss round the shaft and its height past the boss faces |
| `shaft_d`, `shaft_flat`, `shaft_len`, `thread_d`, `thread_len` | 10, 8, 20, M8, 12 | output shaft |
| `bolt_d`, `bolt_nut_af`, `bolt_nut_h` | 6, 10, 5 | boss bolts and nuts |
| `slot` | 4 | every hole may sit +-4 mm from nominal (radial slots) |
| `rot_deg` | 0 | turns the motor about its shaft (0: can points backwards) |

Checks (`frame.checks`): slots clear the ring screws, plate fits the bed. Plate: 10 thick PPA-CF, outline = hull of
the ring and the slots. Saddle: under the can end, height computed from the motor numbers, cable tie or hose clamp.

## 4. Electronics bay

Box 182 x 92 x 58 (+3 lid), floor tabs 12 out on both long sides (footprint 182 x 116), 4 x M3 into the deck.
Placement is automatic (smallest deck, then nearest the main shaft) unless `FRAMEWORK["ebay"] = dict(x, y, yaw)`.

| Item | Where in the bay | Notes |
|---|---|---|
| ESP32-S3 DevKitC-1 | cradle, PCB underside 27 above the floor (DuPont housings on the downward pins), USB edge at the y = 0 wall | wall slot 29 x 15 for both USB ports: native USB cable to the Jetson stays plugged and reachable |
| BTS7960 | 4 standoffs (inserts), PCB underside 24 above the floor, terminal edge towards the x = L wall | 6 vent slots each long wall beside the heatsink, lid grille above |
| MP1584 (9.0 V out for DRV8833 VM) | floor pocket | set 9.0 V with a meter BEFORE connecting the DRV8833 |
| DRV8833 | floor pocket | headers off |
| HX711 (optional) | floor pocket | |
| 20 A fuse holder | cradle + tie | battery + goes through the fuse first |
| Wire exits | x = L wall: 14 AWG wiper pair window 14 x 8 and the XT60 cutout; x = 0 wall: sensor window 16 x 10 (switches, N20, AS5600 STEMMA QT, load cell) | a cable-tie anchor inside next to each exit (strain relief) |

Power path (to agree with fw-base WIRING.md): rover 3S XT60 -> 20 A fuse -> BTS7960 B+ and MP1584 IN+ (tap at the
BTS terminal, as the old station did) ; MP1584 OUT 9 V -> DRV8833 VIN; ESP32 from USB only; grounds common.
BTS7960 logic VCC (5 V) from the ESP32 5V pin (USB powered). fw-base owns the final pin and wire table.

## 5. AS5600

Magnet cap pressed on the main shaft end on the non-motor side (shaft end assumed 8 mm proud of the main block
face: **main shaft owner, please confirm or give me the shaft end**), M3 grub into an insert, magnet pocket 6.15 x
2.5. Board 1.0 mm from the magnet face, chip on the axis, bracket with 4 heat-stake pegs, 2 x M3 into the cheek.
STEMMA QT cable (200 mm) to the bay sensor window.

## 6. What I need from the other subsystems

- Coordinator / BASE_SPEC: `LAYOUT` (every shaft: origin, axis, and please add `d`, `brg` ('608' | 'sprag' | 'bush' |
  dict(od, w)), `cheeks` ((-1, 1), (1,), ...)). `FRAMEWORK`: cheek_gap, motor_side, rover deck limits, keep-out
  volumes (rail sweep, dart path, line path) so ties and the bay stay out of them.
- Shafts not along Y (cad-launch aim pedestal: vertical worm shaft + 8 mm worm wheel axle on two 608s at world
  (58, -100.5) plane y about +55): I will build pedestal blocks on the deck from `LAYOUT` entries with any axis.
  Send the bolt pattern or let me own the pedestal.
- Everyone: the axial position of each bearing on its shaft (I seat bearings inside the cheek thickness; blocks
  slide on over bearings already on the shaft). Tell me if a bearing must sit elsewhere.
- Keep-out: no other shaft block within about 40 mm (radius) of the main axis on the motor-side cheek (adapter hub).
- fw-base: board placement above; connectors: XT60 in, screw terminals on the BTS7960, DuPont on the ESP32 pins.

## 7. What Harry must measure (motor, boards)

1. Wiper motor: the three boss hole positions relative to the output shaft centre (x, y), hole diameter, boss
   diameter and height, gearbox box, can diameter and length, shaft diameter / flats / length to the thread.
   Put them in `params.WIPER`, run the pipeline, reprint `frame_motor_adapter` (and `frame_motor_saddle`).
2. BTS7960: heatsink height under the PCB and the 4 hole positions.
3. Which ESP32-S3 DevKitC-1 he has (Micro-USB official or USB-C clone): the slot fits both.
4. Buy or find a 6 x 2.5 mm diametric magnet (the Adafruit board has none).
