# Frame subsystem notes (worker cad-frame)

Status: phase 0, updated for BASE_SPEC v0.1 (3 Nov 2026, 11:05 Sydney). World frame per the spec: origin on the
wiper axis in the motor boss face plane (y = 0), so the deck top is at z = FRAMEWORK["deck_z"] (default -60) and
the motor-side cheek outer face is PLATE_T + adapter_standoff (10 + 12) from y = 0. The AS5600 magnet holder belongs to
cad-drum; the frame only builds the board bracket (magnet face y from DRUM["magnet_face_y"] when it exists). Code: `frame.py` (API module), helpers `_frame_util.py`,
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
| `frame_as5600_bracket`, `frame_pcb_as5600` | angle sensor board on the main shaft end (section 5) | PLA / bought |
| `frame_ped_<shaft>[_n]` | pedestals for shafts not along Y: pillow plates (horizontal axes) or a C bracket (vertical axes), 608 seats | PLA |

Fasteners: M3 only (M3 x 4 x 5 heat-set inserts, holes 4.0 x 6 deep), plus M6 for the wiper bosses (motor bolts).
Screw lengths: bearing blocks M3 x 8 (counterbore set so every block takes the same screw), cheek feet, saddle,
pedestals, bay tabs M3 x 10 (heads on the 6 mm flange), cheek ties, band lugs, motor ring M3 x 12 (through the
10 mm cheek), ESP32 clamp M3 x 8, BTS7960 M3 x 6, lid M3 x 6, AS5600 bracket M3 x 10.
Rover: M3 x 25 SHCS + nyloc through the deck perimeter holes.

## 2. Board data (looked up 3 Nov 2026)

| Board | Size (mm) | Mounting | Source |
|---|---|---|---|
| ESP32-S3-DevKitC-1 (v1.0, v1.1) | PCB 25.40 x 62.87 (drawing label 62.74), module antenna 6.3 past the top edge (69.2 overall). 2 x 22 pins, 2.54 pitch, rows 22.86 apart, first pin 7.96 from the USB edge | NO mounting holes: held in a printed cradle (pins down for DuPont jumpers) with a hold-down over the module | Espressif user guide v1.1 and DXF (dl.espressif.com/dl/schematics/esp_idf/DXF_ESP32-S3-DevKitC-1_V1.1_20220429.dxf) |
| ESP32 USB | USB edge: UART port centre x 6.0, native USB ("USB", J4) centre x 19.4. Official boards are Micro-USB; clones are USB-C. Native USB alone powers the board (Espressif) | opening only for the native port (12.5 x 9.5, either plug); the PCB edge beside it bears on the wall so a cable pull cannot drag the board out; the UART port stays inside (lid off to use it) | same |
| BTS7960 IBT-2 | PCB 49.66 x 49.40, 43 tall overall incl. heatsink (heatsink under the PCB, height NOT published: 22 assumed) | 4 x M3 on a 40 x 40 square: standoffs with inserts | Handsontec BTS7960 datasheet drawing |
| BTS7960 pins | screw terminal B+, B-, M+, M- (one edge); logic 2 x 4: RPWM, LPWM, R_EN, L_EN, R_IS, L_IS, VCC, GND | | same |
| Adafruit AS5600 (PID 6357) | 25.4 x 17.78, chip at the board centre (12.7, 8.89) | 4 x 2.5 holes at 2.54 from each edge (20.32 x 12.70): printed pegs, heat-staked | Adafruit Eagle files (github.com/adafruit/Adafruit-AS5600-Magnetic-Angle-Sensor-PCB) |
| AS5600 magnet | **NOT included with the Adafruit board** (the add-on magnet PID 9 is a 12 x 4.8 axial disc, wrong type). Need a **6 x 2.5 mm DIAMETRIC NdFeB disc**, air gap 0.5 to 3 mm (design: 1.0) | glued in the cap pocket | adafruit.com/product/6357, AS5600 datasheet |
| Pololu DRV8833 (2130) | 20.32 x 12.70, no holes | headers off, wires soldered, floor pocket with solder relief | pololu.com/product/2130 |
| MP1584EN buck | 22.26 x 16.96 x 3.98 (Harry's calipers), no holes, IN one end, OUT the other | floor pocket | kedge/CLAUDE.md section 2 |
| HX711 (optional) | generic 34 x 21 x 3 (seller); SparkFun 13879 30.48 x 22.86 | floor pocket 34.6 x 23.6 fits both, cable-tie hold-down | SparkFun v11 Eagle files |
| 20 A fuse | inline ATO holder envelope 55 x 26 x 16 (typical). 14 AWG holder: Littelfuse FHAC0003ZXJB (FHAC0001ZXJ has 16 AWG leads) | cradle with a tie slot | littelfuse.com |
| XT60 | body 15.7 x 8.1; panel cutout 16.5 x 9 | wall cutout | seller data |

## 3. Wiper motor adapter (Harry edits only params.WIPER, reprints one part)

Fixed frame interface (never changes): on the motor-side cheek (`FRAMEWORK["motor_side"]`, default -1 = right) a
44 mm hole on the main axis and 6 x M3 clearance holes on R 29, counterbored on the cheek INNER face. The adapter
hub (PLATE_T + adapter_standoff = 22 mm tall) carries 6 inserts in its face and stands the wings off the cheek over
the block flanges. Assembly: motor to adapter on the bench (M6 bolts from the cheek side of the wings, heads in the
slot counterbores, into threaded bosses or nuts behind the boss ears); then 6 x M3 x 12 from inside the cheek box into
the hub, BEFORE any shaft goes in. Nothing is screwed behind the gearbox.

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
| `bolt_d`, `bolt_head_d`, `bolt_nut_af`, `bolt_nut_h` | 6, 10, 10, 5 | boss bolts (head in the adapter slot) and nuts |
| `slot` | 4 | every hole may sit +-4 mm from nominal (radial slots) |
| `rot_deg` | 0 | turns the motor about its shaft (0: can points backwards); only the slots turn, the ring stays |

Checks (`frame.checks`): slots clear the ring inserts and the hub, plate fits the bed. Plate: 10 thick PPA-CF, outline = hull of
the ring and the slots. Saddle: under the can end, height computed from the motor numbers, cable tie or hose clamp.

## 4. Electronics bay

Box 182 x 92 x 58 (+4 lid), floor tabs 12 out on both long sides (footprint 182 x 116), 4 x M3 into the deck.
Placement is automatic unless `FRAMEWORK["ebay"] = dict(x, y, yaw)`: clear of every footprint, 45 mm free in front
of the native USB opening, then smallest deck plus shortest motor and sensor leads.

| Item | Where in the bay | Notes |
|---|---|---|
| ESP32-S3 DevKitC-1 | cradle, PCB underside 27 above the floor (DuPont housings on the downward pins), USB edge at the y = 0 wall, clamp pressing the module end | native USB opening facing free space (45 mm checked) |
| BTS7960 | 4 standoffs (inserts), PCB underside 24 above the floor, terminal edge towards the x = L wall | 6 vent slots each long wall beside the heatsink, lid grille above |
| MP1584 (9.0 V out for DRV8833 VM) | floor pocket + cable tie | set 9.0 V with a meter BEFORE connecting the DRV8833 |
| DRV8833 | floor pocket + cable tie | headers off |
| HX711 (optional) | floor pocket + cable tie | |
| 20 A fuse | WIRING.md puts it in XT60 lead #2 (outside the bay); the cradle holds that inline holder right at the battery entry so the fused lead stays short | battery + goes through the fuse first |
| perfboard 55 x 31 | 2 bosses on the y = 0 wall, 6 mm off it | IS divider (1 k / 680 R), battery divider (100 k / 22 k), 2 x 10 k pull-downs, 3V3 and GND buses for the switches |
| Wire exits | x = L wall: 14 AWG wiper pair window 14 x 8 and the XT60 cutout; x = 0 wall: sensor window 16 x 10 (switches, N20, AS5600 STEMMA QT, load cell) | a cable-tie anchor inside next to each exit (strain relief) |

Power path (to agree with fw-base WIRING.md): rover 3S XT60 -> 20 A fuse -> BTS7960 B+ and MP1584 IN+ (tap at the
BTS terminal, as the old station did) ; MP1584 OUT 9 V -> DRV8833 VIN; ESP32 from USB only; grounds common.
BTS7960 logic VCC from the ESP32 3V3 (WIRING.md, fw-base owns the pin and wire table; 3V3 VCC keeps the IBT-2 input
buffer thresholds below the ESP32 output high).

## 5. AS5600

cad-drum owns the magnet holder on the main shaft end (non-motor side). Until `DRUM["magnet_face_y"]` exists the
frame assumes the magnet face 21 mm beyond the main block face. Board 1.0 mm from the magnet face, chip on the axis,
bracket with 4 heat-stake pegs, 2 x M3 into the cheek. Cable: the run to the bay sensor window is longer than the
200 mm STEMMA QT cables; use a 400 mm or longer JST SH 4-pin cable, or solder 4 wires (VIN, GND, SDA, SCL) to the
header pads (keep I2C under about 1 m).

## 5b. Cable routes

Every bundle runs along a cheek foot flange (cable-tie points between the foot screws: two slots and a tunnel
under the foot) to the bay. Bay exits: 14 AWG wiper pair and XT60 battery lead on the x = L wall, everything else
(AS5600 STEMMA QT, drum N20 pair, contact switches, load cell) through the sensor window on the x = 0 wall. Never
route a wire across a moving part; wires to parts on the rail (SW_SEATED, SW_HOME) leave the frame at the main
axis (the rail pivot: least travel) with a loop of 150 mm, as the old station did.

## 5c. Other subsystems' feet on the deck (automatic)

The frame loads the other subsystem modules (any `cad/parts/<name>.py` with parts/joints/kin), poses them in the rest
state and slices every part that reaches the deck top 0.5 mm above it: the deck grows to cover every foot, and every
round 2.9 to 4.1 mm hole in a foot gets an M3 heat-set insert under it. Nothing to send me: put M3 clearance holes
(3.4) in your feet, sitting on the deck top (world z = deck_z). Optional: `deck_holes(P) -> [(x, y), ...]` (world) in
your module adds holes explicitly. The same load gives the swept volumes (one hull per part per 6 timeline states,
grown 4 mm): the guard band is cut open there and the electronics bay stays out of anything that sweeps lower than
80 mm above the deck.

## 6. What I need from the other subsystems

- Coordinator / BASE_SPEC: `LAYOUT` (every shaft: origin, axis, and please add `d`, `brg` ('608' | 'sprag' | 'bush' |
  dict(od, w)), `cheeks` ((-1, 1), (1,), ...)). `FRAMEWORK`: cheek_gap, motor_side, rover deck limits, keep-out
  volumes (rail sweep, dart path, line path) so ties and the bay stay out of them.
- Shafts not along Y (cad-launch aim pedestal: vertical worm shaft + 8 mm worm wheel axle on two 608s at world
  (58, -100.5) plane y about +55): I will build pedestal blocks on the deck from `LAYOUT` entries with any axis.
  Send the bolt pattern or let me own the pedestal.
- Everyone: the axial position of each bearing on its shaft (I seat bearings inside the cheek thickness; blocks
  slide on over bearings already on the shaft). Tell me if a bearing must sit elsewhere.
- Deck height: coordinator decision 11:00 (from cad-launch notes): main axis 165 above the deck, so the frame default
  is `deck_z = -165`. Put it in `FRAMEWORK["deck_z"]` in params.
- cad-drum: magnet face y (`DRUM["magnet_face_y"]`) and the main shaft end; the wiper coupler length (it lives in the
  44 mm cheek hole and the adapter hub).
- cad-reel: sprag #2 backstop hub "bolts to a frame block": give me its bolt pattern (or I give a block face with
  4 x M3 inserts on R 30 round the capstan axis) and the reaction torque (tow 1.2 N m at the limit, 14 N m at stall).
- Keep-out: no other shaft block within about 40 mm (radius) of the main axis on the motor-side cheek (adapter hub).
- fw-base: board placement above; connectors: XT60 in, screw terminals on the BTS7960, DuPont on the ESP32 pins.

## 7. What Harry must measure (motor, boards)

1. Wiper motor: the three boss hole positions relative to the output shaft centre (x, y), hole diameter, boss
   diameter and height, gearbox box, can diameter and length, shaft diameter / flats / length to the thread.
   Put them in `params.WIPER`, run the pipeline, reprint `frame_motor_adapter` (and `frame_motor_saddle`).
2. BTS7960: heatsink height under the PCB and the 4 hole positions.
3. Which ESP32-S3 DevKitC-1 he has (Micro-USB official or USB-C clone): the slot fits both.
4. Buy or find a 6 x 2.5 mm diametric magnet (the Adafruit board has none).
