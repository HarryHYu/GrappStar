# What to measure or confirm

Everything nobody could know from here. Before printing the base, do A1 to A5, A8, group B, and H1, H3, H4 (all
caliper and hang tests, no electronics needed): the base parts that depend on them should not be printed until
those numbers are in `base/spec/params.py` and the pipeline has been re-run (see "After you
measure" at the end). Tools: caliper, multimeter, kitchen scale, luggage scale, phone camera, tape measure.

Photo rule: for every caliper reading, take a photo with the caliper on the part and the reading visible. Put the
photos in `originals/measure_2026-11/` so the numbers can be checked later.

## A. Wiper motor (params.py `WIPER`, set `measured=True` when done)

| Item | How | Changes | Reprint |
|---|---|---|---|
| A1 Output shaft diameter | Caliper across the round part of the shaft, 3 places, take the smallest | `WIPER.shaft_d` | Wiper shaft coupler (drum parts) |
| A2 Shaft flats or spline | Flats: caliper across the flats (`shaft_flat`). Spline: count the teeth, caliper the outside and root diameter, photo end-on | `WIPER.shaft_flat` (note a spline in REPORT) | Wiper shaft coupler |
| A3 Shaft end thread and length | Thread a nut you know (M6, M8) onto it. Caliper the shaft length from the housing face to the tip | `WIPER.shaft_thread`, `WIPER.shaft_len` | Wiper shaft coupler |
| A4 Mount hole pattern | Lay the motor face down on paper, press so the holes mark it (or trace with a pencil). Then caliper each hole centre from the shaft centre along two square directions and each hole diameter. Photo of the paper with a ruler | `WIPER.mount_holes` as (x, y, dia) from the shaft axis | `frame_motor_adapter` and `frame_motor_saddle` |
| A5 Housing size | Caliper the gearbox box (length, width, height) and the motor can (diameter, length), and how far the shaft sits from each edge | `WIPER.body_box` | Frame cheeks and guard if they clash (the audit says) |
| A6 Free speed and free current | After the first power-on and the current-sense calibration (G1, CALIBRATION.md (b)). Motor clamped, nothing on the shaft, AS5600 magnet on the shaft. Send `{"cmd":"jog","motor":"wiper","dir":1,"ms":1500}` and paste it again every second for about 10 s (each one extends the jog; it stops 1.5 s after the last). Read `rpm`, `amps` and `vbat` in telemetry while it runs steady. Check rpm by eye: tape flag on the shaft, count turns in 10 s, rpm = turns x 6. (Do not use the meter in series for this long run. CALIBRATION.md (b) puts it in series only for single 1.5 s jogs at jog power, which the 10 A range takes; never at full power or in a stall) | `WIPER.n0_rpm`, `WIPER.free_a` | Nothing; re-run physics |
| A7 Stall current and stall torque | Do NOT use the meter: stall is about 18 A, over the meter's 10 A range. After base calibration (firmware/base/CALIBRATION.md), use the `amps` telemetry. Torque: clamp a 300 mm bar on the shaft, hook the luggage scale at the end, scale pulling at 90 degrees to the bar, scale anchored to the bench. Jog (1.5 s at most, repeated) and read the peak; the firmware stall trip may cut it early: note if it does. A jog runs at reduced power (firmware/base/BENCH_TEST.md step 4 gives the duty); stall torque scales with the drive, so full stall torque is about the reading divided by that duty. Torque N m = kg x 9.81 x 0.30. `python3 base/physics/calib.py wiper --v <vbat> --rpm <A6> --i-free <A6> --stall-kg <kg> --arm-mm 300 --i-stall <amps>` prints the whole WIPER block | `WIPER.stall_a`, `WIPER.stall_nm` | Nothing; re-run physics and the ballistics table |
| A8 Mounting boss and hub details | Caliper the boss round the output shaft (diameter `boss_d`, height `boss_h`), the hub (`hub_d`, `hub_h`), the threaded length (`thread_len`), where the motor can starts from the shaft (`can_x0`), and the bolt head and nut sizes the motor takes (`bolt_*`, M6 assumed: head 10, nut 10 AF x 5). The motor sits gearbox above the main axis (decision D20, `WIPER.orient_deg` 90): check the can clears the frame in that position | `WIPER.boss_d`, `boss_h`, `hub_d`, `hub_h`, `thread_len`, `can_x0`, `bolt_*` | `frame_motor_adapter`, `frame_motor_saddle` |

## B. Launch springs (params.py `SPRING`, set `measured=True` when done)

The two springs are extension springs with initial tension: they need a small force before they stretch at all, so
one weight is not enough. Use three loads. Every range number scales with the energy the springs store, so this is the most important measurement.

| Item | How | Changes | Reprint |
|---|---|---|---|
| B1 Free length L0 | Caliper the spring hook to hook (inside of the hooks), no load | `SPRING.free_len` | Nothing |
| B2 Rate k and initial tension F0 | Use water bottles in a shopping bag, weighed on the kitchen scale: about 1, 2 and 3 kg (W1, W2, W3 in kg). Hang each from ONE spring and measure hook to hook (L1, L2, L3). Plot or compute: k (N/mm) = (W3 - W1) x 9.81 / (L3 - L1). F0 (N) = W1 x 9.81 - k x (L1 - L0). The three points should lie on a line; if L1 equals L0 (the spring did not move), drop W1 and add a 4 kg load, but only if L3 - L0 is under 160 mm. Do both springs. Never stretch a spring longer than L0 + 250 mm. Or let the computer fit it: `python3 base/physics/calib.py spring L0 W1:L1 W2:L2 W3:L3` (kg:mm pairs) prints the rate and initial tension | `SPRING.k_n_per_mm`, `SPRING.F0_n` | Nothing unless the stroke changes (physics says); re-run physics and the ballistics table |
| B3 Wire and outside diameter | Caliper | `SPRING.wire`, `SPRING.od` | Spring anchors only if OD differs by more than 0.5 mm |
| B4 Which preload hole | Decision D5: 35 mm stretch at rest (HOME), 245 mm stroke, so 280 mm at full cock. A second anchor hole set gives 5 mm preload (250 mm at full cock) for a weaker spring. After B2, re-run the physics: it says which hole set to use | assembly choice (spring anchor holes) | Nothing |

## C. Parts box: confirm you have these (no new purchases)

| Item | How | Changes | Reprint |
|---|---|---|---|
| C1 608 bearings (buy 10) | Look in the parts box. Needed for: storage reel (2), aim pedestal worm wheel and worm shaft (4), 4 spares | If not: The only purchase: search "608ZZ bearing 8x22x7" |  |
| C2 M8 bolts or 8 mm steel rod, and their lengths | Look in the parts box. Needed for: axles through the 608s | If not: Printed PPA-CF axles (tell the coordinator, it is a params switch) |  |
| C3 Worm N20 count: you need 4 (dart kick, deploy, foot, and the base selector drum) | Look in the parts box. Needed for: dart and base | If not: The kedge notes said 2 in hand and one to buy; the August order listed 8 worm N20s. Count them |  |
| C4 MP1584 #2 (the spare from the dart build; confirm it, decision D19) | Look in the parts box. Needed for: drum N20 supply at 9.0 V | If not: No other way to run the DRV8833 from the 3S: tell the coordinator |  |
| C5 DRV8833 #3 (freed from the dart by the new pin map) | Look in the parts box. Needed for: drum N20 |  |  |
| C6 100 kg load cell + HX711 (August order) | Look in the parts box. Needed for: optional line tension reading | If not: Leave it out: the firmware uses wiper current instead |  |
| C7 Resistors: 2 x 10 k, 1 x 1 k, 1 x 680 R, 1 x 100 k, 1 x 22 k | Look in the parts box. Needed for: BTS7960 enable pull-downs, IS divider, battery divider (firmware/base/WIRING.md) | If not: Any value within 20 percent works for the pull-downs; tell fw-base if the dividers differ |  |
| C8 Spare XT60 plugs, 14 AWG wire, two small inline fuse holders with a 2 A and a 1 A fuse | Look in the parts box. Needed for: XT60 Y harness; fuses for the MP1584 and RC832 taps | If not: Without the small fuses: use 18 AWG for those taps and keep them under 10 cm |  |
| C9 Barrel plug 5.5 x 2.5 mm with lead | Look in the parts box. Needed for: Jetson from the LiPo | If not: Run the Jetson from its own supply on the bench; the rover needs one |  |
| C10 Yellow RCA lead | Look in the parts box. Needed for: RC832 AV out to the DIGITNOW |  |  |
| C11 USB data cable for the DevKitC, STEMMA QT to Dupont cable | Look in the parts box. Needed for: base ESP32, AS5600 |  |  |
| C12 M3 screws (lengths from the base print list), M3 nuts, M3 heat-set inserts | Look in the parts box. Needed for: printed contacts, frame joints |  |  |
| C13 AS5600 magnet: any small neodymium disc (6 to 10 mm, 2 to 3 mm thick, mounted edge-on) or a 6 x 2.5 mm diametric disc. The Adafruit board does not include one | Look in the parts box. Needed for: main shaft angle | If not: Required with the firmware as it is today (the wiper will not start without the magnet). Decision D12 plans a no-magnet mode on an index cam (SW_INDEX); until firmware/base/README.md says it exists, get a magnet |  |
| C14 Optional: a second pair of the launch springs (Bunnings I/N 0487961) | Look in the parts box. Needed for: range: about 4 to 5 m with two springs, 8 to 9 m with four | If not: Two springs work; the base has the provision built in |  |
| C15 8 mm steel rod (or M8 bolts) for: reel shaft S, aim worm wheel axle, storage reel axle | Look in the parts box. Needed for: reel and aim | If not: Printed PPA-CF shafts (the reel worker's fallback); tell the coordinator |  |
| C16 Hardware the CAD assumes: M3 x 4 x 5 heat-set inserts; M3 screws x 6, 8, 10, 12 mm (frame) and M3 x 25 cap screws + nylocs (rover mount); M6 bolts and nuts for the wiper (head 10, nut 10 AF); one M8 nut (drum coupler); 6 mm pins (spring anchors); a C-clip (reel cup head); cable ties, a hose clamp, glue (magnet) | Look in the parts box. Needed for: frame, drum, launch, reel | If not: Exact counts in the base print pack / assembly steps |  |
| C17 JST SH 4-pin (STEMMA QT) cable 400 mm or longer for the AS5600 | Look in the parts box. Needed for: the 200 mm one is too short | If not: Extend with 4 soldered wires and heat shrink |  |
| C18 Carriage stop | Nothing to find: it is a printed PLA flexure with about 19 mm travel (decision D26). TPU pads are an optional upgrade if you get TPU | none | |
| C19 PETG filament (brake rocker) | Look in the parts box. Needed for: reel | If not: PLA works for the first tests |  |
| C20 How many 693ZZ / F693ZZ bearings are left after the dart (the dart uses 5 F693ZZ) | Look in the parts box. Needed for: line pulleys on the base | If not: Tell the coordinator the count |  |

## D. Electrical labels to read

| Item | How | Changes | Reprint |
|---|---|---|---|
| D1 RC832 input voltage | Read the label or the listing. Most take 7 to 24 V. If it says 12 V only, it still runs from the 3S (12.6 V full) but report it | SYSTEM_ELECTRICAL.md run P8 |  |
| D2 Jetson DC input range | Label on the Jetson's own supply and the NVIDIA dev kit page. The 3S gives 9.9 to 12.6 V in use and sags during a wiper stall. If the minimum is above 10 V, charge earlier (3.7 V per cell at rest) | SYSTEM_ELECTRICAL.md run P7 |  |
| D3 DRV8833 carrier make | The order shows Pololu 2130 (x3). Confirm the board is marked pololu (pads VIN, GND, BIN, AIN, SLP). Pololu holds SLP high on the board, which is why the base leaves SLP unwired (D9) | TEST_PLAN stage 2 fallback if it is not | |
| D4 DevKitC board version | Printed on the back (v1.0 or v1.1). The RGB LED pin differs; the firmware avoids both | Nothing |  |
| D5 Video standard | When the dart video shows on the Jetson, note the size (720 x 480 = NTSC, 720 x 576 = PAL) | jetson config |  |

## E. Rover

| Item | How | Changes | Reprint |
|---|---|---|---|
| E1 Deck size and mounting holes | Tape measure the free deck area; caliper the hole positions and diameters from one corner | params.py `FRAMEWORK` (rover mount) | Base deck / rover mount |
| E2 Drive train and whether the Jetson drives it | Tell the coordinator what drives the wheels and what controls it | jetson rover module | Nothing |
| E3 Kiyo X height and tilt on the rover | Tape measure lens to ground; phone level app on the camera for tilt | jetson camera model (target calibration) | Nothing |
| E5 Rover mass, wheelbase, centre of mass | Bathroom scale for the mass; tape for wheelbase and track; centre of mass: balance the rover on a broom handle, front to back | physics `recoil.py` ROVER block (recoil check) | Nothing |
| E4 Base launch height | Tape measure from the ground to the main shaft axis when the base is mounted | physics (ballistics table) | Nothing |

## F. Dart (from the KEDGE notes, still open)

| Item | How | Changes | Reprint |
|---|---|---|---|
| F1 Dart v30 is the one being built | Confirm | print pack choice |  |
| F2 Bought part weights and the finished dart weight | Kitchen scale | `DART.mass_kg`, dart physics |  |
| F3 Stall current of each dart motor at 2S | The dart `i` key while holding the shaft with pliers for 1 s | firmware/dart tuning values (see firmware/dart/README.md) |  |
| F4 OTOTEC BMS size and F693ZZ flange | Caliper (pocket is 40.8 x 8 x 2.1 plus 0.5 each way) | dart geometry if it does not fit |  |
| F5 Nose ballast | Weigh it: 15 g | flight stability |  |

## G. Bench and field numbers (measured during the TEST_PLAN stages)

| Item | How | Changes | Reprint |
|---|---|---|---|
| G1 BTS7960 current sense scale | firmware/base/CALIBRATION.md (meter in series at low duty, compare with `amps`) | firmware `set` key (stored on the ESP32) |  |
| G2 Battery voltage reading | Meter on the LiPo vs `vbat` telemetry | firmware `set` key |  |
| G3 AS5600 zero and drum positions | firmware/base/CALIBRATION.md | stored on the ESP32 |  |
| G4 Storage reel payout drag | A luggage scale cannot read this low: use hanging weights in LAUNCH mode. Tie 40 g (coins in a bag, weighed) to the line end over the edge of the bench: it must NOT pull line off. 50 g must pull line off slowly. That sets the brake at 0.5 N (the design value). Above 1 N costs range, 3 N ruins the flight. Then in NEUTRAL the clutch is FIRM, 5 N (decision D24): with the luggage scale, line pulls off at about 0.5 kg | slip clutch preload (reel parts) |  |
| G5 Muzzle speed | Phone slow-motion video side-on, tape measure laid in the flight line (same distance from the phone as the dart). Count the frames the nose takes to cross 0.5 m; speed m/s = 0.5 x slow-motion fps (240 or 120) / frames | physics model, ballistics table |  |
| G6 Range at three pitches | Tape measure from the base to the dart at 3 pitches, tether on, brake unchanged | ballistics table check (jetson uses it) |  |
| G9 Line tension per amp | Line hooked to the luggage scale anchored to a post, base in REEL, `reel` slowly: note scale newtons and `amps` at 3 loads. The physics model gives line = 17.0 x (amps - free amps) - 6.2 N (the 6.2 N is the take-up drag). The firmware ships `k_n_per_a` 8, about half, so the line would reach about twice the set limit before it trips: this calibration matters | firmware `k_n_per_a` (CALIBRATION.md (e)) |  |
| G10 Tow limit (choose) | 50 N is the design default (decision D3) but the firmware ships `limit_n` 20: set it with `{"cmd":"set","key":"limit_n","val":50}`. Raise it only if G7 shows the anchor holds well above it (keep it below the holding force) | firmware `limit_n` |  |
| G11 Payout brake trim | If G4 is off: add or remove printed 0.4 mm shims under thrust ring T (reel assembly steps) | reel shims (print more) |  |
| G12 Contact gaps | Each printed contact: the M3 screw head sits about 1.1 mm from the nut and must close halfway up its 3 mm bump. Turn the screw to set it, watch `sw` in telemetry | nothing |  |
| G13 AS5600 air gap | Magnet face to chip: 1.0 mm design, 1 to 2 mm is the target (params), 0.5 to 3 mm still reads. Feeler: a stack of paper | nothing |  |
| G14 Wiper coupler shear pin | The coupler's torque limiter is a printed PLA pin (d 4) that SHEARS at about 5 N m (3.5 to 6 N m expected). Break one spare pin on the bench: pin in the coupler, coupler held in a vice by the hub, 300 mm bar on the output side, luggage scale at 90 degrees to the bar. Note the peak: N m = kg x 9.81 x 0.30. Six spare pins are in the print pack (decision D26) | pin diameter if it breaks well outside 3.5 to 6 N m (tell the coordinator) | spare shear pins |
| G7 Soil holding force | Dart anchored, luggage scale on the tether, pull slowly up to 50 N, note where it moves | reel tension limit |  |
| G8 Rail friction | Before the rail is mounted: lay the rail part on a flat board, dart in the cradle, lift one end of the board slowly until the dart slides; phone level app on the board. Friction = tan(angle) | physics (launch efficiency) |  |

## H. Other parts the CAD assumed

| Item | How | Changes | Reprint |
|---|---|---|---|
| H1 Drum N20 output shaft | Caliper the diameter (3 mm assumed) and the D flat depth | drum N20 pinion (drum_NOTES) | `drum` N20 pinion |
| H2 Drum N20 stall torque at 9 V | 50 mm lever on the shaft, pressing on the kitchen scale; jog the drum for at most 1 s. Torque N m = kg x 9.81 x 0.05. Estimate 0.22 N m (the drum gear is 12:1 for 2.16 N m at the drum) | drum gear ratio if it is much weaker (tell the coordinator) | `drum_drum`, `drum_n20_cradle` if the ratio changes |
| H3 CSK20PP keyway depths | Caliper the inner keyway (2.8 assumed) and outer keyway (1.5 assumed) depths | capstan tube key rib, capstan block rib (reel_NOTES) | capstan tube and block |
| H4 BTS7960 module | Height of the heatsink under the board (22 assumed) and the 4 mounting hole spacing (40 x 40 assumed) | `frame_ebay` standoffs | electronics bay |
| H5 DevKitC USB socket | Micro-USB (official board) or USB-C (clone). The slot fits both: just confirm, and have the right cable | nothing | nothing |

## After you measure

1. Fill in `docs/build/MEASURE_RESULTS.md` and put the photos in `originals/measure_2026-11/` (create the folder).
2. Easy way: open Claude Code in the repo and ask it to enter MEASURE_RESULTS.md into `base/spec/params.py`
   (`measured=True` in `WIPER` and `SPRING`), re-run the base pipeline and regenerate the firmware numbers.
3. By hand, in Terminal on the Mac:

   ```
   cd ~/GrappStar
   bash kedge/setup_mac.sh                      # once: builds kedge/.venv (Python and the CAD libraries)
   # edit base/spec/params.py: the WIPER and SPRING blocks, measured=True
   bash base/run_all.sh base                    # assembly, audit (must end CLEAN), print pack in base/out/base/pack
   kedge/.venv/bin/python firmware/base/tools/gen_params.py   # firmware numbers, then re-flash the base ESP32
   python3 docs/build/make_guide.py             # rebuild this guide
   ```

   Fields params.py does not have yet (wiper boss and hub, N20 shaft, keyway depths, BTS7960 heatsink) go to the
   CAD owners through MEASURE_RESULTS.md; the Claude session maps them.
4. Reprint only the parts named in the "Reprint" column. If you have not printed them yet, print after step 3.
