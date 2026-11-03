# KEDGE dart bench test

Goal: every motor turns the right way and stops on its own, both sensors work, the current thresholds are real
numbers, and one full anchor + release runs on soil. About 1 hour.

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
| `1` | deploy | pods swing **down** |
| `2` | kick | leg swings **out**, over the nose |
| `3` | drills | augers turn to **screw in**: left clockwise, right anticlockwise, seen from the motor end, looking toward the tip |
| `4` | foot | foot auger turns to **screw in**: clockwise seen from the motor end, looking toward the tip (right-hand foot auger) |

If a motor turns the wrong way: **battery unplugged**, swap that motor's two thin input wires at its driver
(deploy: DRV#1 AIN1/AIN2, kick: DRV#1 BIN1/BIN2, drills: DRV#2 AIN1/AIN2, foot: DRV#2 BIN1/BIN2). Then check again.

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

1. **Battery unplugged.** Wiring done and meter-checked (WIRING.md step A7 or B10). Firmware flashed (README.md).
2. Loose motors on the bench, or the assembled dart (hull closed, run sheet 8e and 8f done) with the legs free to
   swing. Nothing near the augers. Fingers clear.
3. The u.FL antenna must be clicked onto the VTX before any power (a VTX without its antenna can burn out). Unplug USB.
   Then insert the power loop plug. Nothing moves, nothing gets warm in 30 s. If anything heats: pull the plug.
4. Join `KEDGE`, open `http://192.168.4.1`. The status line must show `IDLE`, `INA219 ok`, `BNO085 ok`.

## Step 1. Power and current sensor

1. Meter the supply: the battery at the power loop socket, or the bench supply at INA219 VIN+ to ground. Compare with
   `rail` on the page: within 0.15 V (the INA219 reads after its shunt, so `rail` can be a few tens of mV lower). Write
   both down.
2. With nothing running, `current` reads 0 to 5 mA (the INA219 sees the motor rail only).
   When a motor runs (step 3) the current must read **positive**. A negative current means INA219 VIN+ and VIN- are
   swapped: the firmware then treats the sensor as failed (`INA219 MISSING`). Battery unplugged and swap them.
3. Press `i` on the page: it prints rail, current, and the peak since the last `i`.

Pass: INA219 ok, rail agrees with the meter. If `INA219 MISSING`: battery unplugged, check D4/D5 and 3V3/GND to the
INA219. Then, with the power off (plug out or LiPo unplugged), plug in USB: Serial Monitor `z` must list 0x40.

## Step 2. BNO085 and attitude

1. Press `b`: roll, pitch and |a| about 1.00 g with the dart still.
2. Status `IDLE`, no key pressed, hands off the legs. Tilt the dart nose up: roll or pitch changes by about 90 (which
   one depends on how the chip sits; step 7 maps the axes). Roll it on its side: the other one changes by about 90.
3. If `BNO085 MISSING`: battery unplugged, check wires. Then, plug still out, plug in USB: Serial `z` must list 0x4a (or 0x4b).
   If the Serial Monitor stays blank: right port, 115200 baud, tap R on the XIAO.

If a motor in steps 3 to 6 does not stop by itself at the end within 8 s (the hold cap), the default stall threshold
is above its real stall current. So press `i` once BEFORE each hold (it resets the peak), then hold, release, and
press `i` again: the peak it prints is the stall current.

Also check once, battery unplugged, meter on ohms: each DRV8833 SLP pad to VM. Under 50 ohms means the pad is tied to
the motor supply on that board: do not connect 3V3 to it (WIRING step B6).

## Step 3. Deploy (DRV#1 A)

1. Hold `deploy down` (key `1`) for 1 s. Pods swing **down**. Wrong way: fix as above.
2. Hold it until the pods reach the end. They stop by themselves (event `stalled: deploy at ... mA, stopped`).
   Write down that mA: **deploy stall = ____ mA**. While moving freely, `current` reads **deploy free = ____ mA**.
3. Hold `deploy up` (`!`) until they stow and stop by themselves.

## Step 4. Drills (DRV#2 A, both motors)

1. Pods down first (step 3). Hold `drills in` (`3`) for 2 s. Check the directions in the table (left clockwise,
   right anticlockwise, from motor end to tip). If both turn the same way: WIRING.md "Drill direction check".
2. **Drill pair free = ____ mA** (expect 60 to 70). Press a printed stick or a
   pencil end lightly against one auger flight (never a cloth or fingers: cloth winds in and pulls fingers with it):
   current rises.
   Do not stall both for long (two sticks, never hands). Stalling both reads **drill pair stall = ____ mA** (expect about 700 at 8 V).
3. Hold `drills out` (`#`): they reverse.

## Step 5. Kick (DRV#1 B)

1. Hold `kick out` (`2`): the leg swings **out over the nose**. Wrong way: fix as above.
2. Assembled (kick leg fitted). **Battery unplugged.** Lay the dart on its back with the kick free to swing in air,
   legs stowed. Plug in (power on with the legs stowed: the firmware counts the kick's travel from here). Speed 255.
   Hold `kick out` (`2`) until the leg stops by itself at its outer end stop (a hold is capped at 8 s: if it is not
   there yet, release and hold again). While it swings, read `current` on the status line: **kick free = ____ mA**.
   When it stops, the event line says `kick travel from stowed NNNN ms (at 8.0 V)`: **kick swing in air = ____ ms**.
   This number goes into `kickSwingMs` (step 10). The firmware measures it, so no stopwatch is needed. It must be well
   under 15000 (the auto limit).
3. Hold `kick back` (`@`) until it stows and stops by itself: **kick stall = ____ mA** (the `stalled: kick at ...`
   event). Repeat steps 2 and 3 twice more and use the middle value of the three swings.
4. STOP, then **battery unplugged**. Turn the dart belly down on a flat board, legs stowed. Plug in. Hold `2` until
   the housing rests on the board and the kick stops by itself: the event gives **kick to ground = ____ ms**. It must
   be under 0.8 x the swing in air, or `/auto` can call flat ground `NO_GROUND` (tell whoever owns the firmware).
   Bring it back with `@`, STOP, battery unplugged.

## Step 6. Foot (DRV#2 B)

1. Hold `foot in` (`4`): the foot auger turns the way that screws in. Wrong way: fix as above.
2. **Foot free = ____ mA** (expect about 40). Brake it by holding the coupler with
   pliers, or press a printed stick against the auger (never a cloth or fingers): **foot stall = ____ mA** (expect
   500 to 600).
3. Hold `foot out` (`$`): it reverses.

## Step 7. BNO085 axes (for the Jetson flight analysis)

This step needs power for the IMU while you turn the dart by hand: status `IDLE` (not armed), nobody else on the
page or the Jetson, no key pressed, hands off the legs and augers.

1. Lay the dart flat, belly down. Open `http://192.168.4.1/flightlog` and look at the last sample: `ax, ay, az`.
   The axis reading about +9.8 points **up**.
2. Stand the dart on its tail, nose up: the axis reading about +9.8 points to the **nose**.
3. Lay it on its right side (kick side up): the axis reading +9.8 points **left**.
4. Write: **nose = ___ (e.g. +x), left = ___, up = ___**. The Jetson flight analysis needs this mapping.

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

## Step 9. Anchor and release on soil (assembled, hull closed)

1. **Battery unplugged.** Legs stowed. Lay the dart on the damp soil, belly down, nose clear of the tray wall. Plug in.
2. Page: press `AUTO anchor`. Watch the status: kick swing, foot touches, kick + foot planting, foot finishing,
   deploy + drills, drills finishing, `ANCHORED`. Write the time of each event from the event lines on the web page
   (not the Serial Monitor: no USB while the battery is in). Note the working currents for `loadMa`.
3. Pull the tether gently by hand: the dart should hold.
4. Press `RELEASE`: drills out, pods up, foot out, kick back, `released`. The legs end stowed. Before lifting the dart
   out of the soil: STOP, status `IDLE`, plug out (the "safe to touch" rule in step 8).
5. If it stops with `FAULT`: write down the fault and the last event, press STOP, and fix the threshold (step 10).
6. Foot current drop (for the kick stall check). Press `AUTO anchor` again and wait for `ANCHORED`. Hands clear. Hold
   `foot in` (`4`) alone for 1 s and read `current` on the status line while holding (not `i`: its peak includes the
   start-up surge): **foot alone in soil = ____ mA**. Then hold `kick out` (`2`) and `4` together for 1 s:
   **kick + foot = ____ mA**. If (foot alone) is more than 80 mA below (kick + foot minus the kick's
   working current), the kick's end stop can be faked while planting: tell whoever owns the firmware. STOP after each.
7. No ground check. Press `RELEASE` and wait for `released` (legs stowed). STOP, **battery unplugged**. Lay the dart on
   its back on the bench, nothing within reach of the kick or the augers. Plug in. Press `AUTO anchor`: the kick swings
   through air to its end stop and the dart reports `FAULT` `NO_GROUND`; foot, pods and drills must not turn.
   Then press `RELEASE` to bring the kick back. **Release spins the drills (3 s), swings the pods and turns the foot
   too, with the dart on its back: hands and cloth well clear.** Then STOP, battery unplugged.

## Step 10. Write the real thresholds

In `firmware/dart/kedge_dart/kedge_logic.h`, `struct Tuning`, set (all at full speed 255, battery fully charged).
The four-motor arrays are in the order {deploy, kick, drills, foot}: `stallMa[0]` is deploy, `stallMa[1]` kick,
`stallMa[2]` drills, `stallMa[3]` foot. `impactG` is the one exception: it is in `struct Flight`, not `Tuning`.

| Setting | Rule | Default |
|---|---|---|
| `stallMa[DEPLOY]` | 80 % of deploy stall | 150 |
| `stallMa[KICK]` | 80 % of kick stall | 150 |
| `stallMa[DRILLS]` | 85 % of drill pair stall | 600 |
| `stallMa[FOOT]` | 80 % of foot stall | 450 |
| `kickContactMa` | about 2 x kick free, below kick stall / 2 | 60 |
| `freeMa[...]` | measured free currents (steps 3 to 6) | 30, 30, 70, 45 |
| `loadMa[...]` | typical working current in soil (step 9) | 40, 70, 420, 130 |
| `podsTravelMs` | pods travel stowed to down at full speed (step 3: hold `1` from stowed until it stalls, time it, or use the default) | 1400 |
| `kickSwingMs` | kick swing in air from step 5 (the firmware's own `kick travel` number), in ms | 4300 |
| `impactG` | under the `peak_g` of a real landing (step 8 and the first shots), over 2 g | 3.5 |
| `kickMaxMs` | 2 x kick swing time, at least 8000 | 15000 |

The PC tests use a motor model with the v29 stall currents (deploy and kick 185 mA, drills 700, foot 550: `Plant` in
`test/test_logic.cpp`, and the `kStall`, `dStall`, `drStall`, `fStall` ranges in `test/fuzz_logic.cpp`). If a stall you
measured is higher, raise the model's number to your measured stall first, or `run_tests.sh` fails (for example
`stallMa[2]` over 700 fails "drill stall threshold below it"). The drill pair stall must stay under 1200 mA (one
DRV8833 channel).

Re-run `test/run_tests.sh`, upload, repeat steps 3, 5 and 6 and step 9 once to see the stops still work.

## Step 11. Jetson link (after the Jetson is set up)

From the Jetson on `KEDGE`: `python3 -m grappstar dart status` (the jetson package).
Phase `IDLE`, fields present. Or simply `curl http://192.168.4.1/status`.

## Step 12. Re-check the wiring inside the closed hull

The run sheet (8e) closes the hull before the legs go on. This step is a re-check after all the tests.
1. **Battery unplugged.**
2. Check the routing: drill wires with a 30 mm loop at each flank hole for the pod swing; the foot pair with a 60 mm
   loop round the kick hub; nothing crossing a hinge; nothing pinched by the hull halves. The DRV#3 pocket stays empty.
3. Insert the plug and repeat steps 3, 5 and 6 (one swing each way) to make sure no wire catches.
4. Camera (after the RC832 is set up): switch on the RC832 and check the picture.

Bring back: the numbers from steps 1 to 7 and 9. They go into `Tuning` and the REPORT.
