# KEDGE dart bench test (firmware v4.3, dart v34)

Goal: every motor turns the right way and stops on its own, both sensors work, the current thresholds are real
numbers, the dart rights itself from its back and its flanks, and one full anchor + release runs on soil. About 1 hour.

v34 has three motor channels: pods (DRV#1 A), the RIGHTING LEG (DRV#1 B, the old kick channel and kick N20) and the
drills (DRV#2 A). There is no foot and no kick leg: DRV#2 B is not wired, and keys `4` / `$` answer `no foot on v34`.

When to run each part:

| Steps | When |
|---|---|
| 0 to 2, and the direction + free current parts of 3 to 6 | loose motors on the bench (bench harness, before routing into the hull) |
| stall at the end stops in 3 to 6, then 7 to 10 | assembled: hull closed, legs and pods fitted (run sheet 8e and 8f done) |
| 11, and 12 step 4 (camera picture) | after the Jetson, RC832 and capture card are set up (rover section) |

You need: charged 18350 cells (XTAR MC2), multimeter, phone or laptop on WiFi `KEDGE`, a stopwatch, a tray or bucket
of damp soil at least 10 cm deep, a folded towel, and this sheet to write numbers on.

Directions used below (firmware convention, +1 in `/status` `motors`):

| Key | Motor | Must do |
|---|---|---|
| `1` | pods (deploy) | the pods swing the drills **DOWN** (from lying forward along the flanks to pointing down) |
| `2` | righting leg | the leg **lifts off the tail** (its pad leaves the tail top) and swings up **over the nose** |
| `3` | drills | augers turn to **screw in**: left clockwise, right anticlockwise, seen from the pivot end, looking toward the tip (the motor drives the auger directly, so both turn the same way) |
| `4` / `$` | (none) | refused: `no foot on v34`, nothing turns |

If a motor turns the wrong way: **battery unplugged**, swap that motor's two leads at its driver (pods: DRV#1 AOUT1 /
AOUT2, leg: DRV#1 BOUT1 / BOUT2, drills: see WIRING.md "Drill direction check"). Swapping its two thin input wires
(DRV#1 AIN1 / AIN2 or BIN1 / BIN2) does the same. Then check again.

**Before loading the dart into the launcher, and before every power-on with the battery:** drive the pods with `!`
until they stall at their fold stop, and the leg with `@` until it stalls in its groove (STOP after each). The firmware
counts the pods' and the leg's travel from where they are at power-on and takes that as folded.

## Power: bench supply or battery

The loose-motor steps (0 to 2, and the direction and free-current parts of 3 to 6) usually run before the cells and the
power loop socket are fitted. Then power comes from the **bench supply**: the spare MP1584 (#2) set to **8.0 V**, fed
by the 3S LiPo on XT60 lead #1 (10 A fuse), as in run sheet step 7.5.

- Connect it with the LiPo unplugged: bench MP1584 OUT+ to the dart's **INA219 VIN+** pad, bench MP1584 OUT- to the
  dart's **main ground** (the dart MP1584 IN- pad). Use small clip leads, or two temporary wires soldered to those pads.
  Check the polarity twice: red to VIN+, black to ground.
- "Insert the power loop plug" below then means: plug the LiPo into the bench MP1584. "Pull the plug" means: unplug
  the LiPo. "The battery voltage" means: the bench supply voltage, metered at INA219 VIN+ to ground (about 8.0 V).
- **USB out** while the bench supply is connected, exactly as with the battery.
- The assembled steps (end-stop stalls in 3 to 6, then 7 to 10) use the real cells and the power loop plug.
- **Before the cells go in: LiPo unplugged, remove the bench leads** (unclip, or desolder the two temporary wires from
  INA219 VIN+ and the MP1584 IN- pad). Nothing of the bench supply may stay inside the hull.

## Step 0. Set up

1. **Battery unplugged.** Wiring done and meter-checked (WIRING.md, v34 section). Firmware flashed (README.md).
2. Loose motors on the bench, or the assembled dart (hull closed, run sheet done) with the leg and the pods free to
   swing, folded. Nothing near the augers. Fingers clear.
3. The u.FL antenna must be clicked onto the VTX before any power (a VTX without its antenna can burn out). Unplug USB.
   Then insert the power loop plug. Nothing moves, nothing gets warm in 30 s. If anything heats: pull the plug.
4. Join `KEDGE`, open `http://192.168.4.1`. The status line must show `IDLE`, `INA219 ok`, `BNO085 ok`, and on the
   third line `battery ok` (on the bench supply at 8.0 V too). `battery low` or `critical`: charge the cells first.

## Step 1. Power and current sensor

1. Meter the supply: the battery at the power loop socket, or the bench supply at INA219 VIN+ to ground. Compare with
   `rail` on the page: within 0.15 V (the INA219 reads after its shunt, so `rail` can be a few tens of mV lower). Write
   both down.
2. With nothing running, `current` reads 0 to 5 mA (the INA219 sees the motor rail only).
   When a motor runs (step 3) the current must read **positive**. A negative current means INA219 VIN+ and VIN- are
   swapped: the firmware then treats the sensor as failed (`INA219 MISSING`). Battery unplugged and swap them.
3. Press `i` on the page: it prints rail, current, and the peak since the last `i`.
4. Battery state (firmware v4.1): the third status line shows `battery ok 7.xx V`. That number is the resting voltage
   with no motor running; it should match the meter within 0.15 V.

Pass: INA219 ok, rail agrees with the meter. If `INA219 MISSING`: battery unplugged, check D4/D5 and 3V3/GND to the
INA219. Then, with the power off (plug out or LiPo unplugged), plug in USB: Serial Monitor `z` must list 0x40.

## Step 2. BNO085 and attitude

1. Press `b`: roll, pitch and |a| about 1.00 g with the dart still.
2. Status `IDLE`, no key pressed, hands off the legs. Tilt the dart nose up: roll or pitch changes by about 90 (which
   one depends on how the chip sits; step 7 maps the axes). Roll it on its side: the other one changes by about 90.
3. Orientation: dart flat on its belly, the third status line says `upright` and `tilt` under 15.
   Lay it on its side: `side` (tilt about 90). On its back: `inverted`. If belly down reads `inverted`, the BNO085 board
   sits upside down in the hull: set `imuUpAxis = -3` in step 10. If belly down reads `side`, the board stands on edge:
   find the up axis in step 7 and set `imuUpAxis` to it. This matters (v4.3): `/auto` rights the dart first whenever the
   tilt is 15 deg or more, so a wrong axis makes it try to right a dart that already lies on its belly.
4. If `BNO085 MISSING`: battery unplugged, check wires. Then, plug still out, plug in USB: Serial `z` must list 0x4a (or 0x4b).
   If the Serial Monitor stays blank: right port, 115200 baud, tap R on the XIAO.

If a motor in steps 3 to 6 does not stop by itself at the end within 8 s (the hold cap), the default stall threshold
is above its real stall current. So press `i` once BEFORE each hold (it resets the peak), then hold, release, and
press `i` again: the peak it prints is the stall current.

Also check once, battery unplugged, meter on ohms: each DRV8833 SLP pad to VM. Under 50 ohms means the pad is tied to
the motor supply on that board: do not connect 3V3 to it (WIRING step B6).

## Step 3. Pods (DRV#1 A)

1. Hold `pods down` (key `1`) for 1 s. The pods swing the drills **down**. Wrong way: fix as above.
2. Hold it until the pods reach the end. They stop by themselves (event `stalled: deploy at ... mA, stopped`).
   Write down that mA: **deploy stall = ____ mA**. While moving freely, `current` reads **deploy free = ____ mA**.
3. Hold `pods up` (`!`) until they fold and stop by themselves.
4. **Pods travel time.** Speed 255. Pods stowed. Hold `1` and time with the stopwatch from the start until the pods hit
   the down stop: **pods travel = ____ ms** (default 1400). This goes into `podsTravelMs` (step 10). Firmware v4.1 uses
   it to tell a stall in soil (keep milling) from the down stop, so measure it carefully: repeat three times, take the
   middle value. Bring the pods back up with `!`.

## Step 4. Drills (DRV#2 A, both motors)

1. Pods down first (step 3). Hold `drills in` (`3`) for 2 s. Check the SLEEVE directions in the table (left clockwise,
   right anticlockwise, from the pivot end to the tip). Wrong: WIRING.md "Drill direction check".
2. **Drill pair free = ____ mA** (expect 60 to 70). Press a printed stick or a
   pencil end lightly against one auger flight (never a cloth or fingers: cloth winds in and pulls fingers with it):
   current rises.
   Do not stall both for long (two sticks, never hands). Stalling both reads **drill pair stall = ____ mA** (expect about 700 at 8 V).
   Write it into BOTH `stallMa[DRILLS]` (85 % of it) and `drillPairStallMa` (the number itself) in step 10: the v4.2
   deploy guard stops the pods when the drills read 67 % of it, so the augers never go deeper than they can be backed out.
3. Hold `drills out` (`#`): they reverse.

## Step 5. Righting leg (DRV#1 B, the old kick channel)

1. Hold `leg out` (`2`) for 1 s: the leg **lifts off the tail** and swings up **over the nose**. Wrong way: battery
   unplugged, swap the leg motor's two leads at DRV#1 BOUT1 / BOUT2. Check again.
2. Assembled (leg fitted). **Battery unplugged.** Dart belly down, leg folded in its groove, pods folded. Plug in
   (power on with them folded: the firmware counts the leg's travel from here). Speed 255. Hold `leg out` (`2`) until
   the leg stops by itself on its bed in the nose top (0 to 178 deg; a hold is capped at 8 s: if it is not there yet,
   release and hold again). While it swings, read `current` on the status line: **leg free = ____ mA**. When it stops,
   the event line says `leg travel from folded NNNN ms (at 8.0 V)`: **leg swing = ____ ms** (expect about 3800).
   This goes into `kickSwingMs` (step 10). The firmware measures it, so no stopwatch is needed.
3. Hold `leg back` (`@`) until it lies in its groove and stops by itself: **leg stall = ____ mA** (the
   `stalled: leg at ...` event). Repeat steps 2 and 3 twice more and use the middle value of the three swings.
   Reaching the nose-top bed in air is normal (it is where the leg ends when the dart is already upright); the
   righting folds it back from there.

## Step 6. No foot (v34)

1. Press `4`: the page log says `refused: no foot on v34` and nothing turns. Same for `$`.

## Step 7. BNO085 axes (for the Jetson flight analysis)

This step needs power for the IMU while you turn the dart by hand: status `IDLE` (not armed), nobody else on the
page or the Jetson, no key pressed, hands off the legs and augers.

1. Lay the dart flat, belly down. Open `http://192.168.4.1/flightlog` and look at the last sample: `ax, ay, az`.
   The axis reading about +9.8 points **up**.
2. Stand the dart on its tail, nose up: the axis reading about +9.8 points to the **nose**.
3. Lay it on its right side: the axis reading +9.8 points **left**.
4. Write: **nose = ___ (e.g. +x), left = ___, up = ___**. The Jetson flight analysis needs this mapping.
5. In the v34 hull the BNO085 lies flat on deck 1, parts up (up = +z), with its silkscreen arrow to the nose
   (dart-cad4). If that arrow is Y, the nose is +y (`imuNoseAxis = 2`); if it is X, the nose is +x (`imuNoseAxis = 1`).
   The check in this step decides, not the arrow.
6. The firmware needs two of them too (step 10): `imuUpAxis` = the up axis, `imuNoseAxis` = the nose axis, coded
   1 = x, 2 = y, 3 = z, with a minus sign for a minus axis (up = -z means `imuUpAxis = -3`). With `imuNoseAxis` set the
   firmware works out roll and pitch: after the upload, lay the dart flat: status `upright`, `roll_g` and `pitch_g` near
   0. Tilt the nose up by about 20 deg (a book under the nose): `nose_up`, `pitch_g` about +20. If it reads -20 the nose
   axis sign is wrong.

## Step 8. Flight log (safe, log only)

1. Open `http://192.168.4.1/arm?on=1&auto=0` (arm, log only: no motors will run). Status shows `ARMED`.
2. Hold the dart over the towel on the floor. Jerk it sharply upwards and let go about 40 cm above the towel.
3. Status should go `FLIGHT` then `LANDED` (or stay `ARMED` if the jerk was under 4 g: jerk harder).
4. Wait until the status shows `LANDED`, then 4 s more (if the towel landing ends as `timeout` that is about 19 s after the jerk, and the log is already frozen by then; `/flightlog` is refused while armed or in flight). Open `http://192.168.4.1/flightlog`: `frozen: true`, `t_launch_ms` and `t_land_ms` set, a few hundred
   samples. Save it. The towel drop usually reads `landing: "timeout"` or `"still"` (soft), which is correct.
5. Open `http://192.168.4.1/stop` to disarm.

**Safe to touch, after any arm or auto test:** press STOP, check the status is not `ARMED`, `FLIGHT`, `ANCHORING` or
`RELEASING` (`IDLE`, `LANDED` or `ANCHORED` is fine), pull the power loop plug (or unplug the bench LiPo), and only then pick the dart up or touch a leg. An
armed dart can start the auto sequence by itself after a knock.

## Step 8b. Optional video switch (only if you fitted it, WIRING.md "Optional: video power switch")

1. RC832 on, picture visible. Press `video off` on the page: the picture goes within 1 s. Press `video on`: it comes back.
2. Press `arm / disarm` with the video off: arming switches it back on. Press it again to disarm.

## Step 9. Anchor and release on soil (assembled, hull closed)

1. **Battery unplugged.** Leg and pods folded. Lay the dart on the damp soil, belly down, nose clear of the tray wall. Plug in.
2. Page: press `AUTO anchor`. Watch the status: `auto: upright (tilt N deg)` (no righting), deploy + drills, drills
   finishing, `ANCHORED`. The leg stays folded. In firm soil you will see `pods in soil, milling` and the pods go down
   in pulses (this can take up to 150 s, loam about 90 s); write down how long the pods step took and the `soil_mA` value from the status. Write the time of each event from the event lines on the web page
   (not the Serial Monitor: no USB while the battery is in). Note the working currents for `loadMa`.
3. Pull the tether gently by hand: the dart should hold.
4. Press `RELEASE`: drills out, pods up, `released`. The pods end folded, the leg was never moved. Before lifting the
   dart out of the soil: STOP, status `IDLE`, plug out (the "safe to touch" rule in step 8).
5. If it stops with `FAULT`: write down the fault and the last event, press STOP, and fix the threshold (step 10).
6. Righting from its back (v4.3). STOP, **battery unplugged**. Leg and pods folded (`@`, `!` to their stops). Lay the
   dart on its back on the soil, nothing within reach of the leg (it sweeps about 115 mm over the nose) or the augers.
   Plug in. Press `AUTO anchor`: `righting 1: leg flip`, the leg pushes the dart end over end over its nose (`going
   over`), folds back into its groove (`leg folded`); if it lands on a flank, `righting 2: pods roll it, drills off`:
   the pods swing down with the drills OFF and roll it onto its belly; then `upright after N righting moves`, deploy +
   drills, `ANCHORED`. Write down the moves and the time. Then `RELEASE`, STOP, battery unplugged.
7. Righting from each flank. Same as 6 with the dart on its left flank, then its right flank (leaning on a pod):
   one `pods roll it` move, then the anchor. If it ends `FAULT` `NOT_UPRIGHT` (`not upright after 6 righting moves`),
   write down the tilt in the event and what the dart did; `RELEASE` folds the pods.

## Step 10. Write the real thresholds

In `firmware/dart/kedge_dart/kedge_logic.h`, `struct Tuning`, set (all at full speed 255, battery fully charged).
The four-motor arrays are in the order {deploy, leg (the `KICK` channel), drills, foot (not on v34, leave it)}:
`stallMa[0]` is the pods, `stallMa[1]` the leg, `stallMa[2]` the drills. `impactG` is the one exception: it is in
`struct Flight`, not `Tuning`.

| Setting | Rule | Default |
|---|---|---|
| `stallMa[DEPLOY]` | 80 % of pods stall | 150 |
| `stallMa[KICK]` | 80 % of leg stall (step 5.3) | 150 |
| `stallMa[DRILLS]` | 85 % of drill pair stall | 600 |
| `freeMa[...]` | measured free currents (steps 3 to 5) | 30, 30, 70, (45) |
| `loadMa[...]` | typical working current in soil (step 9) | 40, 70, 420, 130 |
| `podsTravelMs` | pods travel stowed to down at full speed (step 3.4, the middle of three timings; v4.1 milling depends on it) | 1400 |
| `kickSwingMs` | leg swing from step 5 (the firmware's own `leg travel` number), in ms | 3800 |
| `impactG` | under the `peak_g` of a real landing (step 8 and the first shots), over 2 g | 3.5 |
| `kickMaxMs` | leg fold time limit: 2 x leg swing time, at least 8000 | 15000 |
| `rightedMaxDeg`, `rightTries` | righting (from the study sim): leave unless step 9.6 / 9.7 shows otherwise | 15, 6 |
| `drillPairStallMa` | the drill pair stall itself (step 4.2); the deploy guard is 67 % of it (`drillGuardFrac`) | 700 |
| `imuUpAxis`, `imuNoseAxis` | from step 7 (1 = x, 2 = y, 3 = z, minus for a minus axis) | 3, 0 (0 = nose not known: tilt check only) |
| `battLowV`, `battCritV` | leave unless the BMS cuts out earlier than 6.4 V at rest | 6.8, 6.4 |

The PC tests use a motor model with the v29 stall currents (pods and leg 185 mA, drills 700: `Plant` in
`test/test_logic.cpp`, and the `kStall`, `dStall`, `drStall` ranges in `test/fuzz_logic.cpp`). If a stall you
measured is higher, raise the model's number to your measured stall first, or `run_tests.sh` fails (for example
`stallMa[2]` over 700 fails "drill stall threshold below it"). The drill pair stall must stay under 1200 mA (one
DRV8833 channel).

Re-run `test/run_tests.sh`, upload, repeat steps 3 and 5 and step 9 once to see the stops still work.

## Step 11. Jetson link (after the Jetson is set up)

From the Jetson on `KEDGE`: `python3 -m grappstar dart status` (the jetson package).
Phase `IDLE`, fields present. Or simply `curl http://192.168.4.1/status`.

## Step 12. Re-check the wiring inside the closed hull

The run sheet (8e) closes the hull before the legs go on. This step is a re-check after all the tests.
1. **Battery unplugged.**
2. Check the routing (pack_v34 routes): drill wires with a loop at each flank hole for the pod swing; the leg pair
   clear of the leg cradle; nothing crossing a hinge; nothing pinched by the hull halves. DRV#2 B stays empty.
3. Insert the plug and repeat steps 3 and 5 (one swing each way) to make sure no wire catches. Then fold the pods
   (`!`) and the leg (`@`) to their stops.
4. Camera (after the RC832 is set up): switch on the RC832 and check the picture.

Bring back: the numbers from steps 1 to 5, 7 and 9. They go into `Tuning` and the REPORT.
