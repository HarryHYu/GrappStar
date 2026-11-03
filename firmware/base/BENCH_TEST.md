# Base station bench test

No-load checks BEFORE the wiper motor goes into the base, with firmware `base-1.2.1`. Nothing here has run on real
hardware yet, so this is the first time the firmware meets the parts. Go one step at a time.

Each step says what to type, what you should see, and what to do if not. Keep `{"id":9,"cmd":"stop"}` ready to paste.

Two ways to send the commands:

- On the bench: the Arduino Serial Monitor (line ending "Newline"), as in README.md "First contact".
- On the Jetson: stop the service first (`sudo systemctl stop grappstar`). In one terminal run
  `python3 -m grappstar base monitor` (shows telemetry and events). In a second terminal send each command with
  `python3 -m grappstar base cmd '<json>'`, for example `python3 -m grappstar base cmd '{"id":1,"cmd":"ping"}'`.

Two rules for the whole test:

- **Battery unplugged** before you touch any wire, jumper or plug. Never pull a plug (not even the small sensor
  cable) while the battery is in.
- Never hold a turning shaft, and never put a hand near a spinning shaft.

## Set-up

1. Battery unplugged. Clamp the wiper motor firmly in a vice or to the bench. **Nothing attached to its shaft.** Put a
   strip of tape on the shaft as a flag so you can see it turn.
2. Battery unplugged. Fix the AS5600 board facing the magnet on the wiper shaft end, 1 to 2 mm away, centred (a
   temporary holder or tape is fine).
3. Battery unplugged. The drum N20 is OUT of the drum. Tape or clamp it down so it can spin freely. Tape flag on its
   shaft.
4. Battery unplugged. Contacts on their wires, loose on the bench. Put two jumper wires on:
   - GPIO11 (`SW_DRUM_B`) to GND. That makes the drum code read NEUTRAL. Without it the three drum contacts are all
     open (code 000), which the firmware treats as a wire off: `FAULT` `DRUM_SENSOR` "drum code 000 (wire off or bump
     missing)" right after start-up.
   - GPIO47 (`SW_N20`) to GND. That says the drum N20 is in mesh. Without it every drum move is refused with
     `HAND_MODE`.

   Leave both jumpers on for the whole bench test unless a step says otherwise.
5. Battery unplugged. Wired as in WIRING.md, and the Part C multimeter checks done. The 1 uF capacitor from GPIO1 to
   GND is strongly recommended (WIRING.md section 4).

## Step 1: USB only

Battery unplugged. Plug USB into the port labelled USB. Open the Serial Monitor (or the two Jetson terminals).

- Type: `{"id":1,"cmd":"ping"}`
- You should see: `{"id":1,"ok":true,"fw":"base-1.2.1","t":...}`, and telemetry lines with `"state":"IDLE"`,
  `"mode":"NEUTRAL"`, `"sw":{"b":1,"n":1}` and `vbat` about 0 (no battery).
- If not: no output at all means wrong port or "USB CDC On Boot: Enabled" not set (README.md). Telemetry but no reply
  means the line ending is not "Newline". `BAD_JSON` means a typo (straight quotes only, no smart quotes). `BOOTING`
  means you sent it in the first 60 ms after a start-up: send again.

- Type: `{"id":2,"cmd":"status"}`
- You should see: `"magnet":"OK"`.
- If not: `NONE`: check the STEMMA QT cable (3V3, GND, GPIO8, GPIO9) and that the magnet is there. `WEAK`: closer.
  `STRONG`: further away.

## Step 2: contacts, one jumper at a time

USB only, battery unplugged. Telemetry `sw` lists only the CLOSED contacts. With the two jumpers on you already see
`"b":1` and `"n":1`.

Use one spare jumper wire. For each row: battery still unplugged, put the jumper from the GPIO to GND (or press the
screw head onto its nut), watch `sw`, then take it off again before the next row. Only one contact at a time.

| Close | GPIO | You should see in `sw` |
|---|---|---|
| `SW_DOG_L` | 13 | `"l":1` |
| `SW_DOG_R` | 14 | `"r":1` |
| `SW_HOME` | 21 | `"H":1` |
| `SW_GAP` | 39 | `"G":1` |
| `SW_RAIL_LOW` | 40 | `"L":1` |
| `SW_SEATED` | 41 | `"S":1` |
| `SW_ARM` | 42 | `"W":1` (in the base `W` means the cup is AWAY from home: the contact opens at home) |

Then take the GPIO47 jumper off: `"n":1` must disappear. Put it back.

(`SW_INDEX` on GPIO35, key `I`, is only used on a build without the magnet and does not show in `sw` by default.)

Never close `SW_DOG_L` and `SW_DOG_R` together: that is a fault (`SWITCH_DISAGREE`), because the collar cannot be in
LAUNCH and REEL at once.

### Drum code contacts

The drum reads its position as a 3-bit Gray code, one contact per bit. Neighbouring sectors differ by one contact
only. Walk through the rows in order, changing ONE jumper per row (GPIO10 = `a`, GPIO11 = `b`, GPIO12 = `c`), and
check `drum_pos` in telemetry.

The real drum cannot turn by itself (its worm locks it), so the first change you make by hand raises
`FAULT` `DRUM_SENSOR` "drum code changed with the drum still". That is expected and nothing moves. Carry on: telemetry
`drum_pos` keeps following the code while the base sits in `FAULT`.

| Row | Closed | `drum_pos` | sector (params.DRUM) |
|---|---|---|---|
| 1 | `b` | `NEUTRAL` | NEUTRAL |
| 2 | `a` `b` | `BETWEEN` | L_TO_N |
| 3 | `a` | `LAUNCH` | LAUNCH |
| 4 | `a` `b` | `BETWEEN` | L_TO_N |
| 5 | `b` | `NEUTRAL` | NEUTRAL |
| 6 | `b` `c` | `BETWEEN` | WRAP_PREP |
| 7 | `c` | `BETWEEN` | WRAP (the wrap window) |
| 8 | `a` `c` | `WRAPPED` | N_TO_R (the WRAPPED park is in here) |
| 9 | `a` `b` `c` | `REEL` | REEL |

Then walk back the same way (rows 8, 7, 6, 5) and finish with only `b` closed.

The `mode` field also needs the collar contacts: at row 3 it says `NONE` until you also close `SW_DOG_L` (GPIO13):
then `LAUNCH`. At row 9 it says `NONE` until you close `SW_DOG_R` (GPIO14): then `REEL`. At rows 6, 7 and 8 it says
`NEUTRAL` (the collar stays in NEUTRAL there). Take any extra jumper off again before the next row.

At the end, only `b` (and the `n` jumper) closed, send `{"id":3,"cmd":"clear"}`. You should see `"ok":true` and
`"state":"IDLE"`. One `clear` is enough. `STILL_PRESENT` means the code is not valid (000, or you changed two contacts
at once): check the jumpers and send `clear` again.

- If a contact never shows: check its wire goes to the right GPIO and the other side to GND.
- If it shows when open: the two sides touch, or the GPIO wire touches GND somewhere.
- If it flickers on and off while held closed: the screw and nut do not touch firmly. Clean or tighten them.

## Step 3: battery in, current zero

Battery unplugged. USB plugged in. Then plug the battery in, last. Nothing should move.

- Look: telemetry `vbat` about the same as your meter reads at the XT60 (within about 0.3 V).
- If `vbat` stays about 0: the 100 k / 22 k divider is not connected to GPIO2.
- Type: `{"id":4,"cmd":"cal","what":"current"}` (motor stopped). You should see `"ok":true` and `amps` 0.0.

## Step 4: jog the wiper, and its direction

With the drum code at NEUTRAL a forward wiper jog only counts as take-up, so it runs at 40 % power (trip 2.5 A). A
backward jog runs at about 60 % (trip 3 A).

- Type: `{"id":5,"cmd":"jog","motor":"wiper","dir":1,"ms":500}`
- You should see: the shaft turns for half a second, starting gently (300 ms soft start). Telemetry `state` `JOG`,
  then `IDLE`. `amps` shows a value while it runs (not calibrated yet, so only roughly right).
- Write down which way the shaft turned for dir 1: CW or CCW, looking at the shaft end.

What FORWARD means: dir 1 is FORWARD, and FORWARD is the direction that cocks. On the built base, a short forward jog
in LAUNCH moves the carriage BACK (toward the stretched springs); if it does not, battery unplugged, swap M+ and M-
(CALIBRATION.md section g checks it). On the bench there is no carriage yet, so work it out on the parts: hold
sprag #1 in its drive gear the way it will sit in the base, looking from the wiper end, and turn it by hand. One way
it grips, the other way it spins free. The way it grips is FORWARD. Write it down as CW or CCW too.

- If dir 1 turned the other way than FORWARD: battery unplugged, swap the motor wires at M+ and M-, battery in, and
  jog again.
- Now check the angle sensor agrees: with dir 1, `wiper_deg` must go UP. If it went DOWN, wait until it has stopped,
  then `{"id":6,"cmd":"set","key":"enc_sign","val":-1}` and jog again.
- If it does not move but the reply was `"ok":true`: battery, fuse, B+/B- or the R_EN / L_EN wires. A longer jog that
  does not move ends in `FAULT` `STALL`, because the shaft did not turn.
- If the reply is `"err":"ANGLE_SENSOR"`: the AS5600 does not see the magnet. The wiper never starts without it (on
  this build: the no-magnet index fallback of README.md is off by default).
- If the reply is `"err":"NO_POWER"`: the firmware does not see the battery (`vbat` under 6 V). Check the divider.

Now the other way:

- Type: `{"id":7,"cmd":"jog","motor":"wiper","dir":-1,"ms":500}`
- You should see: the shaft turns the other way. `wiper_deg` goes DOWN.

Longer run, current and speed:

- Type: `{"id":8,"cmd":"jog","motor":"wiper","dir":-1,"ms":1500}`, then `{"id":9,"cmd":"status"}` after it stops.
- You should see: `rpm` in telemetry steady while it runs. It is lower than the 45 rpm spec because a jog runs at
  part power. `peak_a` in `status` shows the highest current of that run. A free motor should draw a few amps at
  most. (A jog trips at 3 A, or 2.5 A forward here: if you get `FAULT` `OVERCURRENT` "wiper over 3.0 A" on the bare
  motor, write down `peak_a`, `clear` and tell the firmware author.)
- If the current is very high with nothing attached, or the motor gets hot: `stop`, battery unplugged, check the
  wiring.

Reverse pause:

- The firmware always stops the wiper for 200 ms before it changes direction. That is too short to time by hand
  (the PC tests check it). What you can check: start `{"id":10,"cmd":"jog","motor":"wiper","dir":1,"ms":1500}` and,
  while it runs, send `{"id":11,"cmd":"jog","motor":"wiper","dir":-1,"ms":500}`.
- You should see: the second one refused with `"err":"BUSY"`. The motor keeps going until the first jog ends.
  You never get a direct reversal.

## Step 5: jog the drum N20

- Type: `{"id":12,"cmd":"jog","motor":"drum","dir":1,"ms":300}`, then the same with `"dir":-1`.
- You should see: the N20 turns one way, then the other. Telemetry `state` `JOG` then `IDLE`. (The drum code does not
  change: it comes from the jumper.)
- If the reply is `"err":"HAND_MODE"`: the GPIO47 jumper is missing.
- If it does not turn: buck output 9.0 V on DRV8833 VM? AIN1 on GPIO15, AIN2 on GPIO16? Common ground? SLP stays
  unconnected: that is correct (decision D9). If the drum does nothing: battery unplugged, wire DRV8833 SLP to the
  ESP32 3V3 pin (never to VM).
- Which way is right is checked later with the drum fitted (CALIBRATION.md section c).

## Step 6: fault test, AS5600 (wiper stopped)

This test is done with the wiper STOPPED. Never move the sensor, or a hand, near a spinning shaft. (Losing the sensor
while the wiper runs is checked by the PC tests: the wiper stops at once with `FAULT` `ANGLE_SENSOR`.)

1. Battery unplugged. Move the AS5600 board well away from the magnet (do not pull its plug with the battery in).
2. Plug the battery in. `status` shows `"magnet":"NONE"`.
3. Type: `{"id":13,"cmd":"jog","motor":"wiper","dir":1,"ms":500}`
4. You should see: `"ok":false,"err":"ANGLE_SENSOR"`. Nothing moves.
5. Battery unplugged. Put the board back. Plug the battery in. `status` shows `"magnet":"OK"`, and the same jog works
   again.
6. If the motor moved in step 3: battery unplugged at once, and do not go on. Report it.

## Step 7: fault test, stall (optional: skip if unsure)

**Never hold the shaft with your hands or fingers.** A wiper motor is strong enough to hurt you. Only do this with a
proper stop. Skip it if you do not have one: the PC tests check stall detection.

1. Battery unplugged. Take the multimeter out of the motor wire if it is still there (a stall is too much for its
   10 A range).
2. Battery unplugged. Fix a solid arm to the shaft (for example a wooden or printed lever on the shaft flat with the
   M8 nut). Clamp a padded block to the bench so the arm hits it after about a quarter turn of dir -1.
3. Plug the battery in. Stand clear. Type: `{"id":16,"cmd":"jog","motor":"wiper","dir":-1,"ms":1500}`
4. You should see: the arm hits the block, and within a fraction of a second the motor stops with
   `{"evt":"FAULT",...,"code":"OVERCURRENT","why":"wiper over 3.0 A"}` (the jog trip is only 3 A) or
   `"code":"STALL"` followed by a `{"evt":"STALL",...}` line. Both are a correct stop.
5. `{"id":17,"cmd":"status"}` shows `peak_a`. Write it down.
6. `{"id":18,"cmd":"clear"}`. Battery unplugged before you remove the arm.

## Step 8: stop and estop

- Type: `{"id":19,"cmd":"jog","motor":"wiper","dir":-1,"ms":1500}`, then at once `{"id":20,"cmd":"stop"}`.
- You should see: the motor stops at once. `{"evt":"ABORTED",...,"what":"JOG","why":"stop"}`.
- Same again with `{"id":21,"cmd":"estop"}`.
- You should see: the motor stops at once. `ABORTED` with `"why":"estop"`, then a `MODE` line with `"ok":true`: the
  drum code says NEUTRAL already, so the drum does not move. In the base, estop drives the drum to NEUTRAL when
  that is safe; with the wraps on (or another reason) the drum holds and a `MODE` line with `"err":"DRUM_HELD"` and
  a `"why"` comes.
- If the motor keeps running: unplug the battery. Report it.

## Step 9: brownout (optional)

This needs the battery below 12 V (check `vbat`). It raises the low-battery level for a moment so a normal battery
looks low.

1. Type: `{"id":22,"cmd":"set","key":"vbat_low","val":12}`
2. Type: `{"id":23,"cmd":"jog","motor":"wiper","dir":-1,"ms":1500}`
3. You should see: after about 0.3 s the motor stops with `FAULT` `BROWNOUT` "battery sagged under load".
4. **Set it back first** (it is saved in flash): `{"id":24,"cmd":"set","key":"vbat_low","val":9.9}`. Check with
   `{"id":25,"cmd":"get","key":"vbat_low"}`.
5. Then `{"id":26,"cmd":"clear"}`. (Clearing before you set it back gives `STILL_PRESENT`.)

## Done

Battery unplugged. Write down: which way dir 1 turns (CW or CCW from the shaft end), which way sprag #1 locks,
`enc_sign`, the `peak_a` values, anything odd. Then build the base and go on with CALIBRATION.md.
