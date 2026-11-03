# What to measure or confirm

Everything nobody could know from here. Do group A and B first: the base parts that depend on them should not
be printed until those numbers are in `base/spec/params.py` and the pipeline has been re-run (see "After you
measure" at the end). Tools: caliper, multimeter, kitchen scale, luggage scale, phone camera, tape measure.

Photo rule: for every caliper reading, take a photo with the caliper on the part and the reading visible. Put the
photos in `originals/measure_2026-11/` so the numbers can be checked later.

## A. Wiper motor (params.py `WIPER`, set `measured=True` when done)

| # | What | How | Changes | Reprint after |
|---|---|---|---|---|
| A1 | Output shaft diameter | Caliper across the round part of the shaft, 3 places, take the smallest | `WIPER.shaft_d` | Wiper shaft coupler (drum parts) |
| A2 | Shaft flats or spline | Flats: caliper across the flats (`shaft_flat`). Spline: count the teeth, caliper the outside and root diameter, photo end-on | `WIPER.shaft_flat` (note a spline in REPORT) | Wiper shaft coupler |
| A3 | Shaft end thread and length | Thread a nut you know (M6, M8) onto it. Caliper the shaft length from the housing face to the tip | `WIPER.shaft_thread`, `WIPER.shaft_len` | Wiper shaft coupler |
| A4 | Mount hole pattern | Lay the motor face down on paper, press so the holes mark it (or trace with a pencil). Then caliper each hole centre from the shaft centre along two square directions and each hole diameter. Photo of the paper with a ruler | `WIPER.mount_holes` as (x, y, dia) from the shaft axis | Motor adapter plate (frame parts) |
| A5 | Housing size | Caliper the gearbox box (length, width, height) and the motor can (diameter, length), and how far the shaft sits from each edge | `WIPER.body_box` | Frame cheeks and guard if they clash (the audit says) |
| A6 | Free speed and free current | After the first power-on and the current-sense calibration (G1, CALIBRATION.md). Motor clamped, nothing on the shaft. Send a 35 s `jog`. Tape flag on the shaft: count turns for 30 s, rpm = turns x 2. Read `amps` and `vbat` from telemetry. (Not with the meter in series: the start current can pass 10 A and blow the meter fuse) | `WIPER.n0_rpm`, `WIPER.free_a` | Nothing; re-run physics |
| A7 | Stall current and stall torque | Do NOT use the meter: stall is about 18 A, over the meter's 10 A range. After base calibration (firmware/base/CALIBRATION.md), use the `amps` telemetry. Torque: clamp a 300 mm bar on the shaft, hook the luggage scale at the end, scale pulling at 90 degrees to the bar, scale anchored to the bench. Jog for at most 2 s at full duty and read the peak (the firmware stall trip may cut it early: note if it does). Torque N m = kg x 9.81 x 0.30. `python3 base/physics/calib.py wiper --v <vbat> --rpm <A6> --i-free <A6> --stall-kg <kg> --arm-mm 300 --i-stall <amps>` prints the whole WIPER block | `WIPER.stall_a`, `WIPER.stall_nm` | Nothing; re-run physics and the ballistics table |

## B. Launch springs (params.py `SPRING`, set `measured=True` when done)

The two springs are extension springs with initial tension: they need a small force before they stretch at all, so
one weight is not enough. Use three loads. Every range number scales with the energy the springs store, so this is the most important measurement.

| # | What | How | Changes | Reprint after |
|---|---|---|---|---|
| B1 | Free length L0 | Caliper the spring hook to hook (inside of the hooks), no load | `SPRING.free_len` | Nothing |
| B2 | Rate k and initial tension F0 | Use water bottles in a shopping bag, weighed on the kitchen scale: about 1, 2 and 3 kg (W1, W2, W3 in kg). Hang each from ONE spring and measure hook to hook (L1, L2, L3). Plot or compute: k (N/mm) = (W3 - W1) x 9.81 / (L3 - L1). F0 (N) = W1 x 9.81 - k x (L1 - L0). The three points should lie on a line; if L1 equals L0 (the spring did not move), drop W1 and add a 4 kg load, but only if L3 - L0 is under 160 mm. Do both springs. Never stretch a spring longer than L0 + 250 mm. Or let the computer fit it: `python3 base/physics/calib.py spring L0 W1:L1 W2:L2 W3:L3` (kg:mm pairs) prints the rate and initial tension | `SPRING.k_n_per_mm`, `SPRING.F0_n` | Nothing unless the stroke changes (physics says); re-run physics and the ballistics table |
| B3 | Wire and outside diameter | Caliper | `SPRING.wire`, `SPRING.od` | Spring anchors only if OD differs by more than 0.5 mm |

## C. Parts box: confirm you have these (no new purchases)

| # | Item | Needed for | If you do not have it |
|---|---|---|---|
| C1 | 608 bearings (8 to 10) | storage reel, capstan or wrap arm, cam roller | The only purchase: search "608ZZ bearing 8x22x7" |
| C2 | M8 bolts or 8 mm steel rod, and their lengths | axles through the 608s | Printed PPA-CF axles (tell the coordinator, it is a params switch) |
| C3 | Worm N20 count: you need 4 (dart kick, deploy, foot, and the base selector drum) | dart and base | The kedge notes said 2 in hand and one to buy; the August order listed 8 worm N20s. Count them |
| C4 | MP1584 #2 (the spare from the dart build) | drum N20 supply at 9.0 V | No other way to run the DRV8833 from the 3S: tell the coordinator |
| C5 | DRV8833 #3 (freed from the dart by the new pin map) | drum N20 | |
| C6 | 100 kg load cell + HX711 (August order) | optional line tension reading | Leave it out: the firmware uses wiper current instead |
| C7 | Resistors: 2 x 10 k, 1 x 1 k, 1 x 680 R, 1 x 100 k, 1 x 22 k | BTS7960 enable pull-downs, IS divider, battery divider (firmware/base/WIRING.md) | Any value within 20 percent works for the pull-downs; tell fw-base if the dividers differ |
| C8 | Spare XT60 plugs, 14 AWG wire, two small inline fuse holders with a 2 A and a 1 A fuse | XT60 Y harness; fuses for the MP1584 and RC832 taps | Without the small fuses: use 18 AWG for those taps and keep them under 10 cm |
| C9 | Barrel plug 5.5 x 2.5 mm with lead | Jetson from the LiPo | Run the Jetson from its own supply on the bench; the rover needs one |
| C10 | Yellow RCA lead | RC832 AV out to the DIGITNOW | |
| C11 | USB data cable for the DevKitC, STEMMA QT to Dupont cable | base ESP32, AS5600 | |
| C12 | M3 screws (lengths from the base print list), M3 nuts, M3 heat-set inserts | printed contacts, frame joints | |

## D. Electrical labels to read

| # | What | How | Changes |
|---|---|---|---|
| D1 | RC832 input voltage | Read the label or the listing. Most take 7 to 24 V. If it says 12 V only, it still runs from the 3S (12.6 V full) but report it | SYSTEM_ELECTRICAL.md run P8 |
| D2 | Jetson DC input range | Label on the Jetson's own supply and the NVIDIA dev kit page. The 3S gives 9.9 to 12.6 V in use and sags during a wiper stall. If the minimum is above 10 V, charge earlier (3.7 V per cell at rest) | SYSTEM_ELECTRICAL.md run P7 |
| D3 | DRV8833 carrier make | Is DRV8833 #3 a Pololu board (marked pololu, pads VIN, GND, BIN, AIN, nSLEEP) or a generic red board (pads VCC, EEP, ULT)? The base leaves SLP unwired, which relies on the board pulling it high (Pololu does) | firmware/base/WIRING.md, TEST_PLAN stage 1 |
| D4 | DevKitC board version | Printed on the back (v1.0 or v1.1). The RGB LED pin differs; the firmware avoids both | Nothing |
| D5 | Video standard | When the dart video shows on the Jetson, note the size (720 x 480 = NTSC, 720 x 576 = PAL) | jetson config |

## E. Rover

| # | What | How | Changes | Reprint after |
|---|---|---|---|---|
| E1 | Deck size and mounting holes | Tape measure the free deck area; caliper the hole positions and diameters from one corner | params.py `FRAMEWORK` (rover mount) | Base deck / rover mount |
| E2 | Drive train and whether the Jetson drives it | Tell the coordinator what drives the wheels and what controls it | jetson rover module | Nothing |
| E3 | Kiyo X height and tilt on the rover | Tape measure lens to ground; phone level app on the camera for tilt | jetson camera model (target calibration) | Nothing |
| E4 | Base launch height | Tape measure from the ground to the main shaft axis when the base is mounted | physics (ballistics table) | Nothing |

## F. Dart (from the KEDGE notes, still open)

| # | What | How | Changes |
|---|---|---|---|
| F1 | Dart v30 is the one being built | Confirm | print pack choice |
| F2 | Bought part weights and the finished dart weight | Kitchen scale | `DART.mass_kg`, dart physics |
| F3 | Stall current of each dart motor at 2S | The dart `i` key while holding the shaft with pliers for 1 s | firmware/dart tuning values (see firmware/dart/README.md) |
| F4 | OTOTEC BMS size and F693ZZ flange | Caliper (pocket is 40.8 x 8 x 2.1 plus 0.5 each way) | dart geometry if it does not fit |
| F5 | Nose ballast | Weigh it: 15 g | flight stability |

## G. Bench and field numbers (measured during the TEST_PLAN stages)

| # | What | How | Changes |
|---|---|---|---|
| G1 | BTS7960 current sense scale | firmware/base/CALIBRATION.md (meter in series at low duty, compare with `amps`) | firmware `set` key (stored on the ESP32) |
| G2 | Battery voltage reading | Meter on the LiPo vs `vbat` telemetry | firmware `set` key |
| G3 | AS5600 zero and drum positions | firmware/base/CALIBRATION.md | stored on the ESP32 |
| G4 | Storage reel payout drag | A luggage scale cannot read this low: use hanging weights in LAUNCH mode. Tie 50 g (coins in a bag, weighed) to the line end over the edge of the bench: it must NOT pull line off. 75 g must pull line off slowly. That is 0.5 to 0.75 N. Above 1 N costs range, 3 N ruins the flight | slip clutch preload (reel parts) |
| G5 | Muzzle speed | Phone slow-motion video side-on, tape measure laid in the flight line (same distance from the phone as the dart). Count the frames the nose takes to cross 0.5 m; speed m/s = 0.5 x slow-motion fps (240 or 120) / frames | physics model, ballistics table |
| G6 | Range at three pitches | Tape measure from the base to the dart at 3 pitches, no tether brake changes | ballistics table check (jetson uses it) |
| G7 | Soil holding force | Dart anchored, luggage scale on the tether, pull slowly up to 50 N, note where it moves | reel tension limit |
| G8 | Rail friction | Before the rail is mounted: lay the rail part on a flat board, dart in the cradle, lift one end of the board slowly until the dart slides; phone level app on the board. Friction = tan(angle) | physics (launch efficiency) |

## After you measure

1. Write the numbers into `base/spec/params.py` (set `measured=True` in `WIPER` and `SPRING`) or send them to the
   coordinator. Put photos in `originals/measure_2026-11/`.
2. Re-run the base pipeline (assembly, audit must say CLEAN, physics, ballistics table, print pack) and
   `firmware/base/tools/gen_params.py`, then re-flash the base ESP32.
3. Reprint only the parts named in the "Reprint after" column. If you have not printed them yet, print after step 2.
