# Base station bench test

No-load checks BEFORE the wiper motor goes into the base. Nothing here has run on real hardware yet, so this is the
first time the firmware meets the parts. Go one step at a time.

Each step says what to type, what you should see, and what to do if not. Type commands into the Serial Monitor
(line ending "Newline"). Keep `{"id":9,"cmd":"stop"}` ready to paste.

## Set-up

1. Battery unplugged. Clamp the wiper motor firmly in a vice or to the bench. **Nothing attached to its shaft.** Put a
   strip of tape on the shaft as a flag so you can see it turn.
2. Battery unplugged. Fix the AS5600 board facing the magnet on the wiper shaft end, 1 to 3 mm away, centred (a
   temporary holder or tape is fine).
3. Battery unplugged. The drum N20 is OUT of the drum. Tape or clamp it down so it can spin freely. Tape flag on its
   shaft.
4. Battery unplugged. Contacts on their wires, loose on the bench. Put a jumper wire from GPIO12 (`SW_DRUM_B`) to GND.
   That makes the drum code read NEUTRAL. Without it the three drum contacts are all open (code 000), which the
   firmware treats as a wire off: it faults with `DRUM_SENSOR` "drum code 000 (wire off or bump missing)" right after
   start-up. Leave the jumper on for the whole bench test unless a step says otherwise.
5. Battery unplugged. Wired as in WIRING.md, and the Part C multimeter checks done.

## Step 1: USB only

Battery unplugged. Plug USB into the port labelled USB. Open the Serial Monitor.

- Type: `{"id":1,"cmd":"ping"}`
- You should see: `{"id":1,"ok":true,"fw":"base-1.0","t":...}`, and telemetry lines with `"state":"IDLE"` and
  `"vbat":null` (no battery).
- If not: no output at all means wrong port or USB CDC On Boot not Enabled (README.md). Telemetry but no reply means
  the line ending is not "Newline". `BAD_JSON` means a typo (straight quotes only, no smart quotes).

- Type: `{"id":2,"cmd":"status"}`
- You should see: `"magnet":"OK"`.
- If not: `NONE`: check the STEMMA QT cable (3V3, GND, GPIO8, GPIO9) and that the magnet is there. `WEAK`: closer.
  `STRONG`: further away.

## Step 2: contacts, one by one

Battery unplugged (USB only is enough). Telemetry `sw` lists only the CLOSED contacts. With the NEUTRAL jumper on
GPIO12 you already see `"b":1` and `drum_pos` `NEUTRAL`.

Launcher and reel contacts: close each by hand (press the screw head onto its nut, or touch its two wires together),
watch `sw`, then open it again.

| Close | You should see in `sw` |
|---|---|
| `SW_DOG` (GPIO14) | `"D":1` |
| `SW_SEATED` (GPIO39) | `"S":1` |
| `SW_HOME` (GPIO40) | `"H":1` |
| `SW_RAIL_LOW` (GPIO41) | `"L":1` |
| `SW_WRAP_HOME` (GPIO42) | `"W":1` |

Drum code contacts: the drum reads its position as a 3-bit Gray code, one contact per bit (`base/cad/parts/drum_NOTES.md`).
Neighbouring positions differ by one contact only. Walk through them in this order, changing ONE contact per row, and
check `drum_pos` and `mode` in telemetry. The jumper on GPIO12 is `b`: unplug it from GND where a row says `b` open.

| Row | Closed | `drum_pos` | `mode` |
|---|---|---|---|
| 1 | `b` (the jumper) | `NEUTRAL` | `NEUTRAL` |
| 2 | `a` `b` | `BETWEEN` (LAUNCH side) | `NONE` |
| 3 | `a` | `LAUNCH` | `LAUNCH` |
| 4 | `a` `b` | `BETWEEN` | `NONE` |
| 5 | `b` | `NEUTRAL` | `NEUTRAL` |
| 6 | `b` `c` | `BETWEEN` (REEL side) | `NONE` |
| 7 | `a` `b` `c` | `REEL` | `REEL` |
| 8 | `a` `c` | `BETWEEN` (laying wraps) | `REEL` |
| 9 | `c` | `WRAPPED` | `REEL` |

Then walk back the same way (rows 8, 7, 6, 5) and finish with only `b` closed.

- If a contact never shows: check its wire goes to the right GPIO and the other side to GND.
- If it shows when open: the two sides touch, or the GPIO wire touches GND somewhere.
- If it flickers on and off while held closed: the screw and nut do not touch firmly. Clean or tighten them.
- A `FAULT` `DRUM_SENSOR` while you do this is expected if you skip a row or open everything for more than 0.15 s
  ("drum code 000" or "drum code jumped over a sector"). Nothing moves. Go back to row 1 (only `b`) and send
  `{"id":2,"cmd":"clear"}`. `STILL_PRESENT` means the code is still not valid.

Finish with only `b` closed (NEUTRAL) for the motor steps below. In NEUTRAL the wiper turns nothing, and no wiper jog
is refused because of the launcher.

## Step 3: battery in, current zero

Battery unplugged. USB plugged in. Then plug the battery in, last. Nothing should move.

- Look: telemetry `vbat` about the same as your meter reads at the XT60 (within about 0.3 V).
- If `vbat` stays `null`: the 100 k / 22 k divider is not connected to GPIO2.
- Type: `{"id":3,"cmd":"cal","what":"current"}` (motor stopped). You should see `"ok":true` and `amps` 0.0.

## Step 4: jog the wiper

- Type: `{"id":4,"cmd":"jog","motor":"wiper","dir":1,"ms":500}`
- You should see: the shaft turns for half a second, starting gently (300 ms soft start). Telemetry `state` `JOG`,
  then `IDLE`. `wiper_deg` goes UP. `amps` shows a value while it runs (not calibrated yet, so only roughly right).
- Write down which way the shaft turned for dir 1 (looking at the shaft end). Dir 1 is FORWARD, the direction that
  cocks the launcher. Compare with the build guide.
- If `wiper_deg` went DOWN: `{"id":5,"cmd":"set","key":"enc_sign","val":-1}` and jog again.
- If the shaft turned the wrong way for FORWARD: battery unplugged, swap the motor wires at M+ and M-, battery in, and
  jog again. (Then check `wiper_deg` again.)
- If it does not move but the reply was `"ok":true`: battery, fuse, B+/B- or the R_EN / L_EN wires. A longer jog
  that does not move ends in `FAULT` `STALL`, because the shaft did not turn.
- If the reply is `"err":"ANGLE_SENSOR"`: the AS5600 does not see the magnet. The wiper never starts without it.

Now the other way:

- Type: `{"id":6,"cmd":"jog","motor":"wiper","dir":-1,"ms":500}`
- You should see: the shaft turns the other way. `wiper_deg` goes DOWN.

Longer run, current and speed:

- Type: `{"id":7,"cmd":"jog","motor":"wiper","dir":1,"ms":1500}`, then `{"id":8,"cmd":"status"}` after it stops.
- You should see: `rpm` in telemetry steady while it runs. It is lower than the 45 rpm spec because a jog runs at
  about 60 % power. `peak_a` in `status` shows the highest current of that run. A free motor should draw a few amps
  at most.
- If the current is very high with nothing attached, or the motor gets hot: stop, battery unplugged, check the wiring.

Reverse pause:

- The firmware always stops the wiper for 200 ms before it changes direction. That is too short to time by hand
  (the PC tests check it). What you can check: start `{"id":9,"cmd":"jog","motor":"wiper","dir":1,"ms":1500}` and,
  while it runs, send `{"id":10,"cmd":"jog","motor":"wiper","dir":-1,"ms":500}`.
- You should see: the second one refused with `"err":"BUSY"`. The motor keeps going until the first jog ends.
  You never get a direct reversal.

## Step 5: jog the drum N20

- Type: `{"id":11,"cmd":"jog","motor":"drum","dir":1,"ms":300}`, then the same with `"dir":-1`.
- You should see: the N20 turns one way, then the other. Telemetry `state` `JOG` then `IDLE`.
- If it does not turn: buck output 9.0 V on DRV8833 VM? SLP wire on GPIO21? AIN1 GPIO17, AIN2 GPIO18? Common ground?
- Which way is right is checked later with the drum fitted (CALIBRATION.md step c).

## Step 6: fault test, AS5600

First without the sensor:

- Move the AS5600 board well away from the magnet (or pull the magnet off). `status` shows `"magnet":"NONE"`.
- Type: `{"id":12,"cmd":"jog","motor":"wiper","dir":1,"ms":500}`
- You should see: `"ok":false,"err":"ANGLE_SENSOR"`. Nothing moves.

Then lose it while running:

- Put the board back. `status` shows `"magnet":"OK"`.
- Type: `{"id":13,"cmd":"jog","motor":"wiper","dir":1,"ms":1500}` and, while it runs, slide the AS5600 board away
  from the magnet. (You may instead pull the small STEMMA QT plug. It is the only plug you may pull with the battery
  in: it is a 3.3 V sensor cable. Keep your hands away from the motor and its wires.)
- You should see: the motor stops at once. A line `{"evt":"FAULT",...,"code":"ANGLE_SENSOR","why":"AS5600 lost while the wiper runs"}`,
  telemetry `"state":"FAULT"`, `"fault":"ANGLE_SENSOR"`. The drum code already says NEUTRAL, so the drum does
  not move.
- Type: `{"id":14,"cmd":"clear"}` while the sensor is still away.
- You should see: `"err":"STILL_PRESENT"`.
- Put the sensor back, `clear` again. You should see `"ok":true` and `"state":"IDLE"`.
- If the motor did not stop: send `stop`, unplug the battery, and do not go on. Report it.

## Step 7: fault test, stall (optional: skip if unsure)

**Never hold the shaft with your hands or fingers.** A wiper motor is strong enough to hurt you. Only do this with a
proper stop. Skip it if you do not have one: the PC tests check stall detection.

1. Battery unplugged. Take the multimeter out of the motor wire if it is still there (a stall is too much for its
   10 A range).
2. Battery unplugged. Fix a solid arm to the shaft (for example a wooden or printed lever on the shaft flat with the
   M8 nut). Clamp a padded block to the bench so the arm hits it after about a quarter turn of dir 1.
3. Plug the battery in. Stand clear. Type: `{"id":15,"cmd":"jog","motor":"wiper","dir":1,"ms":1500}`
4. You should see: the arm hits the block, and within about half a second the motor stops with
   `{"evt":"FAULT",...,"code":"STALL","why":"wiper stalled"}` and a `{"evt":"STALL",...}` line. (`OVERCURRENT` instead
   is also a correct stop.)
5. `{"id":16,"cmd":"status"}` shows `peak_a`. Write it down.
6. `{"id":17,"cmd":"clear"}`. Battery unplugged before you remove the arm.

## Step 8: stop and estop

- Type: `{"id":18,"cmd":"jog","motor":"wiper","dir":1,"ms":1500}`, then at once `{"id":19,"cmd":"stop"}`.
- You should see: the motor stops at once. `{"evt":"ABORTED",...,"what":"JOG","why":"stop"}`.
- Same again with `{"id":20,"cmd":"estop"}`.
- You should see: the motor stops at once. `ABORTED` with `"why":"estop"`. The drum code says NEUTRAL already, so the
  drum does not move. In the base, estop drives the drum to NEUTRAL when that is safe, and otherwise sends a `MODE`
  line with `"err":"DRUM_HELD"` and a `"why"`.
- If the motor keeps running: unplug the battery. Report it.

## Step 9: brownout (optional)

This needs the battery below 12 V (check `vbat`). It raises the low-battery level for a moment so a normal battery
looks low.

1. Type: `{"id":21,"cmd":"set","key":"vbat_low","val":12}`
2. Type: `{"id":22,"cmd":"jog","motor":"wiper","dir":1,"ms":1500}`
3. You should see: after about 0.3 s the motor stops with `FAULT` `BROWNOUT` "battery sagged under load".
4. **Set it back first** (it is saved in flash): `{"id":23,"cmd":"set","key":"vbat_low","val":9.9}`. Check with
   `{"id":24,"cmd":"get","key":"vbat_low"}`.
5. Then `{"id":25,"cmd":"clear"}`. (Clearing before you set it back gives `STILL_PRESENT`.)

## Done

Battery unplugged. Write down: which way dir 1 turns, `enc_sign`, `peak_a` values, anything odd. Then build the base
and go on with CALIBRATION.md.
