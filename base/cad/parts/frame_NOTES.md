# Frame subsystem notes (worker cad-frame)

Status: BASE_SPEC v1.2.x (3 Nov 2026, afternoon). World frame per the spec: origin on the main axis in the wiper boss
face plane (y = 0), deck top z = -165 (`LAYOUT.deck_top_z`). After D23 the frame carries the motor end of the main
shaft only (the launch sleeve's outboard bushing and the AS5600 bracket are cad-launch's, on its aim housing).

Code: `frame.py` (API module), helpers `_frame_util.py`, `_frame_motor.py`, `_frame_ebay.py`, `_frame_kit.py`,
`_frame_guard.py`, `_frame_others.py`, `_frame_ped.py`, `_frame_calc.py` (leading underscore: not loaded as
subsystems), `frame_guards.py` (a second subsystem module: guards that ride on cad-launch's rail), tests and tools in
`_frame_tests/` (pytest suite, render, drill template, integration audit).

## 1. Parts

| Part | What | Material |
|---|---|---|
| `frame_deck_1..4` | 8 mm deck, dovetailed tiles (each under 248), covers the params target footprint and every foot of every subsystem; M3 insert under every frame foot hole AND every round M3 hole found in another subsystem's foot; perimeter M3 rover holes (the deck is its own drill template; 1:1 PDF too) | PLA, 20 % infill |
| `frame_cheek_r` | motor plate (y 22..32) with the coupler bore, keyed counterbore, 12-hole ring, and a bridge towards -y at x 76..108; carved in 3D round every other subsystem's swept volume | PLA |
| `frame_motor_leg` | column from the deck to the bridge in the free corner between the reel cup and the bay (x 76..108, y -55..-30), foot to -y, 2 x M3 into the deck, 2 x M3 along y into the bridge | PLA |
| `frame_motor_adapter` | wiper adapter, separately printed (section 3) | PPA-CF |
| `frame_wiper_motor`, `frame_wiper_shaft` | bought envelope and the rotating output stub (joint `frame_wiper` = `wiper_deg`) | bought |
| `frame_ebay`, `frame_ebay_lid`, `frame_ebay_esp_clamp`, `frame_pcb_*`, `frame_fuse_20a`, `frame_perfboard` | electronics bay (section 4) | PLA / bought |
| `frame_camera_mount`, `frame_kiyo_x` | Kiyo X column with a 16 mm edge plate for the camera's own clip (or a 1/4-20 screw) | PLA / bought |
| `frame_rc832_holder`, `frame_rc832` | RC832 stands antenna-end up in a slot, leads out low | PLA / bought |
| `frame_cable_anchor_1..2` | cable-tie anchors on the deck (camera USB, receiver AV lead) | PLA |
| `frame_spring_guard_l/r_1..3` (frame_guards.py) | C-channel over each launch spring, on joint `launch_rail`, carved round the rail and the aim parts | PLA |

Fasteners: M3 only (M3 x 4 x 5 heat-set inserts, holes 4.0 x 6 deep), plus M6 for the wiper bosses.

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

## 3. Wiper motor adapter (Harry edits only params.WIPER, reprints the puck)

BASE_SPEC section 2: world y = 0 is the adapter plate FRONT face, the motor sits behind it. The frame's motor plate
(`frame_cheek_r`) occupies y -10..0 with a window r 88. The adapter is a PUCK (`frame_motor_adapter`, PPA-CF) that
fills the window (y -10..0), with a rear lip out to r 97 (y -16..-10) that keys it, held by 8 x M3 x 12 from the
plate front face at r 92.5 into lip inserts (the frame uses the ones with both plate and lip material, at least 4,
checked). The motor bosses bear on the puck's rear face (y -10 = params WIPER boss_face_y, D48: the one datum the
frame places the motor from and cad-drum places its coupler hub from), the M6 bolts go in from the front (heads in the
slot counterbores) on the bench. cad-drum's SW_INDEX block sits on the strut of its idler post 2 (params
DRUM.sw_index_mount), so the puck front face has no inserts for it (the frame adds them only if sw_index_mount gives
(x, z) points). Puck and plate are carved round the reel cup and capstan behind the plate. Two tether channels (r 3,
cad_support-1, from cad-reel's line_path at wraps 0 and full wraps) are cut out to the puck's edge: the guide eye to
cup ring run (wraps 0) opens into the capstan opening, so the line swings into it when the wrap starts and can be laid
in from there; the pulley 1 to pulley 2 run is a notch at the rim, closed by the plate once fitted (the line is pushed
through from behind, like pulley 2's swivel tube).

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
| `orient_deg` (params) / `rot_deg` | 90 | turns the motor about its shaft: 90 = gearbox and can above the axis (D20); only the slots turn, the ring stays |
| `boss_face_y` (params, D48) | -10 | world y of the boss face; must be the motor plate's rear face (checked). cad-drum's coupler hub reads it with the shaft numbers |

Checks (`frame.checks`): slots inside the puck and clear of the bore and of any SW_INDEX inserts, at least 4 usable
rim screws, boss face on the plate's rear face (D48), every printed part watertight as an STL. The can stay
(`frame_can_stay`) is sized from the can numbers too: reprint it with the puck.

## 4. Electronics bay

Box 124 x 124 x 58 (+4 lid) in the params region (x 110..240, y -70..60), 4 x M3 through floor pads into the deck.
Yaw 90: the native-USB wall faces world +x (outward, 45 mm clear checked), the motor and sensor exits face world -x.

| Item | Where in the bay | Notes |
|---|---|---|
| ESP32-S3 DevKitC-1 | cradle, PCB underside 27 above the floor (DuPont housings on the downward pins), USB edge at the y = 0 wall, clamp pressing the module end | native USB opening facing free space (45 mm checked) |
| BTS7960 | 4 standoffs (inserts), PCB underside 24 above the floor, terminal edge towards the x = L wall | 6 vent slots each long wall beside the heatsink, lid grille above |
| MP1584 (9.0 V out for DRV8833 VM) | floor pocket + cable tie | set 9.0 V with a meter BEFORE connecting the DRV8833 |
| DRV8833 | floor pocket + cable tie | headers off |
| HX711 (optional) | floor pocket + cable tie | |
| 20 A fuse | WIRING.md puts it in XT60 lead #2 (outside the bay); the cradle holds that inline holder right at the battery entry so the fused lead stays short | battery + goes through the fuse first |
| perfboard 55 x 31 | 2 bosses on the y = 0 wall, 6 mm off it | IS divider (1 k / 680 R), battery divider (100 k / 22 k), 2 x 10 k pull-downs, 3V3 and GND buses for the switches |
| Wire exits | y = W wall (world -x): 14 AWG wiper pair window 12 x 8 and sensor window 16 x 10 (switches, N20, AS5600, load cell); x = L wall: battery lead | a cable-tie anchor inside next to each exit (strain relief) |

Power path (to agree with fw-base WIRING.md): rover 3S XT60 -> 20 A fuse -> BTS7960 B+ and MP1584 IN+ (tap at the
BTS terminal, as the old station did) ; MP1584 OUT 9 V -> DRV8833 VIN; ESP32 from USB only; grounds common.
BTS7960 logic VCC from the ESP32 3V3 (WIRING.md, fw-base owns the pin and wire table; 3V3 VCC keeps the IBT-2 input
buffer thresholds below the ESP32 output high).

## 5. Cable routes (bundle by bundle; pins and gauges: firmware/base/WIRING.md)

Battery unplugged for every step. Bay orientation: native-USB wall to world +x, motor and sensor windows on the wall
facing world -x (towards the leg and the drum), battery lead through the wall facing world +y.

| Bundle | From | Route | To | Strain relief |
|---|---|---|---|---|
| Battery (14 AWG, XT60 lead #2 with the inline 20 A fuse) | rover 3S | along the deck to the bay +y wall, fuse holder in its cradle just inside | BTS7960 B+ / B-; MP1584 IN from the BTS terminals | tie anchor inside the battery exit |
| Wiper pair (14 AWG) | BTS7960 M+ / M- | motor window (world x 113, y -17..-5), across 5 mm to the leg, up the leg column and over the bridge to the plate rear face, then along the can stay to the motor terminals | wiper motor (2-wire rebuild; park wire taped off) | tie anchor inside the window; cable ties round the leg column (32 x 25) and the stay post |
| Sensors (twisted, 26 AWG) | ESP32 / perfboard | sensor window (world y 21..37) to the drum switch block and the drum N20 (drum axis x 60, z -112) along the deck; the AS5600 and the aim contacts continue along the deck to cad-launch's aim housing (y about 205); rail contacts (SW_SEATED, SW_HOME) leave at the main axis with a 150 mm loop | contacts, N20 (DRV8833 outputs), AS5600 (I2C, keep the run under about 1 m), HX711 | tie anchor inside the window; ties on the drum bracket feet |
| Jetson USB | ESP32 native USB | out of the +x wall, 45 mm free in front | Jetson | tie to the deck edge |
| Camera, receiver | Kiyo X, RC832 | down the camera column / out of the RC832 notch to the deck cable anchors | Jetson USB, DIGITNOW capture | `frame_cable_anchor_1..2` |

Never route a wire across a moving part: the rail sweep (y 95..185), the drum, the reel cup and the wrap ring all move.

## 5b. Other subsystems' feet on the deck (automatic)

The frame loads every other subsystem module with parts/joints/kin, poses them in the rest state and slices every
part that reaches the deck top 0.5 mm above it: the deck grows to cover every foot, and every round 2.9 to 4.1 mm hole
in a foot gets an M3 heat-set insert under it. Put M3 clearance holes (3.4) in your feet on the deck top; optional
`deck_holes(P)` in your module adds holes explicitly. The same load gives the swept volumes (one hull per part per 6
timeline states, grown 4 mm) that the motor plate, bridge, leg and the bay keep out of.

## 6. What I need from the other subsystems (current)

- Everyone: feet on the deck top (z = LAYOUT.deck_top_z) with M3 clearance holes; inserts come automatically.
- cad-drum: keep the SW_INDEX block and lever, and anything else, outside r 51 of the main axis for y below 25 (the
  adapter hub ring). Coupler hub: done (D48, final cross-check): it reaches into the puck bore (r 15) from y -5 and
  takes the flats (y -4..10) and the M8 thread (y 10..22) of the WIPER shaft, placed from params WIPER boss_face_y.
- cad-reel: the sprag #2 hub clear of r 51 of the main axis for y below 25.
- cad-launch: M3 insert holes in the rail wall outer faces at the spring guard fixing points
  (`frame_guards.fix_points(P)`, 2 per guard piece per side, horizontal along y).
- fw-base: board placement above; connectors: battery lead with the inline 20 A fuse, screw terminals on the BTS7960,
  DuPont on the ESP32 pins, perfboard for the dividers and buses.

## 7. What Harry must measure (motor, boards)

1. Wiper motor: the three boss hole positions relative to the output shaft centre (x, y), hole diameter, boss
   diameter and height, gearbox box, can diameter and length, shaft diameter / flats / length to the thread.
   Put them in `params.WIPER`, run the pipeline, reprint `frame_motor_adapter` only.
2. BTS7960: heatsink height under the PCB and the 4 hole positions.
3. Which ESP32-S3 DevKitC-1 he has (Micro-USB official or USB-C clone): the opening fits both plugs.
4. Rover: its top plate and where the base sits; drill the deck pattern (PDF template) there.
