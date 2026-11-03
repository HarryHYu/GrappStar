# Base station calibration

Do BENCH_TEST.md first. Steps (a) and (b) can be done on the bench. The rest needs the base assembled.

- Every command below goes into the Serial Monitor (line ending "Newline"). Any whole `id` number works.
- Every value you `set` or learn with `cal` is **saved in flash automatically** (once everything has stopped) and is
  kept over a power cycle and over a new upload. Read a value back with `get`, for example
  `{"id":1,"cmd":"get","key":"amps_per_volt"}`.
- If something faults, read the `FAULT` line (`code` and `why`), fix the cause, then send `{"id":1,"cmd":"clear"}`.
- `stop` is always there: `{"id":1,"cmd":"stop"}`. Keep it pasted in the box, ready to send.
- **Battery unplugged** before you touch any wire, plug, contact or the meter leads. Never pull a plug while the
  battery is in.

Nothing here has been tried on the real base yet. Expect to adjust.

## New build: tell the firmware the launcher is at rest

On a new build (or after a fresh upload with no saved state) the firmware does not know where the launcher is: the
shot phase is `UNKNOWN`. Then `aim` is refused (`PHASE_UNKNOWN`) and leaving LAUNCH is refused (`LAUNCH_BUSY`),
because only the wiper worm holds the spring and the firmware will not shift while the spring might be drawn.

Look at the launcher: carriage fully forward on its stop (`H` in telemetry `sw`) and the big gear in its gap (`G`).
Then send `{"id":1,"cmd":"cal","what":"cock_rest"}`. `SWITCH_DISAGREE` means `SW_HOME` is open: the carriage is not
home, or the contact is not touching.

## (a) AS5600 magnet and direction

1. Battery unplugged. USB only. Magnet in its cap on the shaft end, 1 to 3 mm from the chip, centred.
2. Send `{"id":1,"cmd":"status"}`. You want `"magnet":"OK"`.
   - `NONE`: no magnet seen or no I2C. Check the STEMMA QT cable and the magnet.
   - `WEAK`: move the magnet closer. `STRONG`: move it further away.
3. The wiper output cannot be turned by hand (its worm locks it). So for this check turn the magnet instead: before
   it is glued, hold it over the chip at 1 to 3 mm and turn it slowly (or turn the AS5600 board round the magnet).
   Watch `wiper_deg` in telemetry: it must change smoothly, about 360 per turn. Then fix the magnet in its cap.
4. Direction (needs the battery; done once in BENCH_TEST.md step 4, check it again in the base). Battery unplugged:
   check nobody's hands are near the shaft. Plug the battery in. Drum in NEUTRAL. Send:

   ```
   {"id":2,"cmd":"jog","motor":"wiper","dir":-1,"ms":500}
   ```

   Dir -1 is BACKWARD. `wiper_deg` must go DOWN. If it goes UP, wait until it has stopped, then send:

   ```
   {"id":3,"cmd":"set","key":"enc_sign","val":-1}
   ```

   Jog again: now it must go down. (`BUSY` means it was still moving: send again.) The firmware also catches a wrong
   sign by itself while driving hard: `FAULT` `ANGLE_SENSOR` "AS5600 counts against the drive (enc_sign?)".
5. In `status`, `glitches` counts rejected AS5600 readings. A few are fine. If it climbs fast while the motor runs,
   route the AS5600 cable away from the motor wires.

## (b) Wiper current

### Current zero

1. Battery plugged in, wiper stopped (`"amps"` should be near 0 anyway).
2. Send `{"id":1,"cmd":"cal","what":"current"}`. `amps` now reads 0.0 when stopped.

### Current scale (needs a multimeter with a 10 A range)

If WIRING.md section 3 found an onboard IS resistor, set the computed `amps_per_volt` first.

1. Battery unplugged. Put the meter in series with the motor + wire (BTS7960 M+ to the meter 10 A jack, meter COM to
   the motor red wire). In the MOTOR wire, not the battery wire. Meter on the 10 A DC range.
2. Plug the battery in. Drum in NEUTRAL (on the bench: the GPIO11 jumper). Jog backward for 1.5 s:

   ```
   {"id":2,"cmd":"jog","motor":"wiper","dir":-1,"ms":1500}
   ```

   Sending the same line again before it ends keeps it running (copy the line, then paste and Enter a few times).
3. Read the meter once it is steady, and `amps` in telemetry at the same moment.
4. `new value = old value x meter amps / shown amps`. The old value is 12.5 unless you changed it. Example: meter
   1.8 A, shown 1.5 A: 12.5 x 1.8 / 1.5 = 15.0. Send `{"id":3,"cmd":"set","key":"amps_per_volt","val":15.0}`.
5. Jog again and check they now agree. Then battery unplugged, take the meter out, reconnect the motor wire directly.
   The 10 A range must never see a stall.

### Free running current

The tension estimate uses the current above free running in REEL. Learn it with the wraps on and the line slack
(after (c)).

1. Line slack, drum in REEL (after a `wrap`, see (c)).
2. Send `{"id":2,"cmd":"jog","motor":"wiper","dir":1,"ms":1500}`, then within the 1.5 s, after at least 0.8 s, send
   `{"id":3,"cmd":"cal","what":"free"}`. `NOT_RUNNING` means you were too early or too late: try again.
3. Easier way: jog, read `amps` in telemetry while it runs steady, then
   `{"id":4,"cmd":"set","key":"free_a","val":1.6}` (your number).

## (c) Drum direction, modes, wrap and unwrap

The drum position comes from three code contacts (a Gray code, BENCH_TEST.md step 2). The code is absolute, so there
is no home search: the base always knows which sector the drum is in, even after a power cut.

1. Battery unplugged. The drum N20 is in place and in mesh (`n` in telemetry `sw`), the three drum code contacts are
   wired (the bench jumpers are OFF), and the line is slack.
2. USB plugged in, battery still unplugged. `{"id":1,"cmd":"cal","what":"drum"}` replies `"ok":true` and sends a
   `MODE` event with the `drum_pos` the code shows. `POSITION_UNKNOWN` means the code is invalid (000, or a jump):
   check the code contacts (BENCH_TEST.md step 2).
3. Get the drum to NEUTRAL (`drum_pos` `NEUTRAL`, only `b` of the code closed) by hand, because the motor direction
   is not checked yet. Battery still unplugged: flip the hand latch (`n` disappears from `sw`), turn the drum with the
   manual lever until only `b` is closed, latch the N20 back in (`n` closed). Turning the drum while the board runs
   gives `FAULT` `DRUM_SENSOR` "drum code changed with the drum still": expected, send `{"id":2,"cmd":"clear"}` (twice if the fault comes back at once).
4. Direction check. Plug the battery in. Drum at NEUTRAL. Send a short drum jog toward REEL:

   ```
   {"id":3,"cmd":"jog","motor":"drum","dir":1,"ms":800}
   ```

   The drum must move into the next sector toward REEL: `sw` shows `b` and `c` closed (code 0 1 1, the wrap prep
   sector), `drum_pos` `BETWEEN`. Then go back with `{"id":4,"cmd":"jog","motor":"drum","dir":-1,"ms":800}`: only
   `b` again.
5. If instead `sw` shows `a` and `b` (code 1 1 0, the ramp toward LAUNCH), the drum turned the wrong way: battery
   unplugged, swap the two N20 wires at AOUT1 and AOUT2. Bring the drum back to NEUTRAL with the hand lever as in step 3,
   plug the battery in and repeat step 4.
6. Now try the modes and wait for each `MODE` event with `"ok":true`:

   ```
   {"id":6,"cmd":"mode","to":"LAUNCH"}
   {"id":7,"cmd":"mode","to":"NEUTRAL"}
   ```

   After each one, telemetry `drum_pos` and `mode` must name that position, and the dog collar must sit where it
   should (look). In LAUNCH `l` must be closed; in NEUTRAL `l` and `r` open.
7. Wrap and unwrap. Line threaded as in the build, the dart off the rail, a little line pulled out and lying slack.
   Stand clear of the line.

   ```
   {"id":8,"cmd":"wrap"}
   ```

   The drum turns through the WRAP sector (the cup lays 7 turns on the capstan, `wraps` counts up in telemetry), then
   into REEL. Wait for `WRAPPED` and then `MODE` with `"mode":"REEL"` (about 15 s). `r` must be closed. Then:

   ```
   {"id":9,"cmd":"unwrap"}
   ```

   Wait for `UNWRAPPED` and `MODE` with `"mode":"NEUTRAL"` (about 16 s). `wraps` back to 0.

   `mode REEL` is refused (`NOT_WRAPPED`) unless the wraps are on: always use `wrap`. And with the wraps on, `mode` is
   refused (`WRAPS_ON`): always use `unwrap`.
8. The wiper turns a little during these moves. Keep clear:
   - leaving LAUNCH it first turns 30 deg BACKWARD (the centering jog) so the dogs carry no load;
   - leaving REEL (start of `unwrap`) it first turns forward a little, then 30 deg back;
   - entering REEL it creeps forward slowly until `r` closes (the fly-in);
   - entering LAUNCH, if `l` does not close, it turns BACKWARD slowly (up to 40 deg) until the dogs drop in;
   - during `unwrap` it turns forward at 40 % while the drum is in the WRAP sector (the take-up winds the freed line).

   If the dogs never drop in: `FAULT` `DRUM_TIMEOUT` "dogs did not engage (collar contact open)". Check the `l` / `r`
   contact closes when the collar is engaged.
9. If you get `SW_ARM counted n of 7 cup turns` (`DRUM_SENSOR`): the `SW_ARM` contact (`W`) must close once per cup
   turn. Check it with the drum still by watching `W` while the cup is at home.
10. The drum speed (`drum_dps`) is learned by itself between code edges. See it with
    `{"id":10,"cmd":"get","key":"drum_dps"}`. Nothing to do.
11. If you get `DRUM_TIMEOUT` or `DRUM_SENSOR`: read the `why`. Battery unplugged, check the code contacts close and
    open cleanly as the drum turns (turn it with the hand lever, `n` open), then `clear` and try again.

## (d) Aim

You need a phone inclinometer app (any "level" app that shows degrees). Base assembled, no dart, nobody in front of
the rail.

1. Battery unplugged. Fix the phone flat on the rail (tape or a rubber band), along the rail.
2. Plug the battery in. `{"id":1,"cmd":"mode","to":"LAUNCH"}`.
3. The launcher must be at rest: carriage fully forward, `H` and `G` in telemetry `sw`. On a new build, declare it
   (top of this file).
4. See the allowed range: `{"id":3,"cmd":"get","key":"pitch_range"}` (it replies `min` and `max`, about 6 and 47).
5. Aim at a middle pitch: `{"id":4,"cmd":"aim","deg":30}`. After every shift into LAUNCH the aim is unknown, so it
   first turns backward until `SW_RAIL_LOW` closes (home), then goes to 30. That can take up to about half a minute:
   one aim cycle is 10 wiper turns. Wait for the `AIMED` event with `"ok":true`.
6. Read the phone. Say it shows 33.4. Send `{"id":5,"cmd":"cal","what":"aim","deg":33.4}`. This sets `aim_offset` so
   the reported pitch matches the phone.
7. Check at 3 pitches: aim at a low, middle and high pitch inside the range (for example 15, 30, 42). At each, compare
   the phone with the `pitch_deg` in the `AIMED` event. Within about 1 deg is good.

### If the error is different at different pitches

Then the offset alone cannot fix it. Most likely `SW_RAIL_LOW` closes at a different point of the aim train than the
firmware assumes (`aim_home_deg`, the aim-train angle where `SW_RAIL_LOW` closes on the backward run).

1. Find the size of the error. First clear the offset: `{"id":1,"cmd":"set","key":"aim_offset","val":0}`. Jog
   backward in short steps:
   `{"id":1,"cmd":"jog","motor":"wiper","dir":-1,"ms":1500}`. Pick a pitch in the middle of the range where the rail
   moves quickly, for example 30. Write down `wiper_deg` when the PHONE reads 30, and `wiper_deg` when telemetry
   `pitch_deg` reads 30 (on the same up or down stroke).
2. D = the difference between the two `wiper_deg` numbers (as a positive number).
   - Telemetry reached 30 AFTER the phone did: the firmware is behind. New `aim_home_deg` = old + D.
   - Telemetry reached 30 BEFORE the phone did: new `aim_home_deg` = old minus D.

   Get the old value with `{"id":2,"cmd":"get","key":"aim_home_deg"}` (default 3365), then set the new one, for
   example `{"id":3,"cmd":"set","key":"aim_home_deg","val":3340}`. The allowed range is -7200 to 7200.
3. Changing `aim_home_deg` does not re-home by itself. Send `{"id":4,"cmd":"cal","what":"aim_forget"}`: the next
   `aim` homes again on `SW_RAIL_LOW`.
4. Redo steps 5 to 7 above.

## (e) Tension

### `k_n_per_a` (line newtons per amp, used without a load cell)

You need a luggage scale. Do (b) free current first.

1. Battery unplugged. Tie the line end to the luggage scale. Hook the scale to something solid. Line slack.
2. Plug the battery in. Get the drum to NEUTRAL (`{"id":1,"cmd":"mode","to":"NEUTRAL"}`), then
   `{"id":2,"cmd":"wrap"}` and wait for `WRAPPED`. The capstan only pulls with the wraps on.
3. Stand to the side of the line, never in line with it. Jog in short steps:
   `{"id":3,"cmd":"jog","motor":"wiper","dir":1,"ms":300}`. Watch the scale.
4. When the scale reads about 20 N (2 kg), read `amps` in telemetry while it pulls, then let the jog end. (A jog trips
   at 3 A: if you get `FAULT` `OVERCURRENT`, `clear` and use a lower pull, for example 15 N.)
5. `k_n_per_a = 20 / (amps - free_a)`. Example: amps 2.6, free_a 1.5: 20 / 1.1 = 18.2.
6. To release: unhook the scale by hand (20 N is safe to hold). The capstan never turns back (sprag #2).
7. Do it 3 times. Use the average: `{"id":4,"cmd":"set","key":"k_n_per_a","val":18.2}` (your number).
8. Then `{"id":5,"cmd":"unwrap"}`.

### HX711 load cell (only if fitted)

1. Check `HELLO` said `"hx711":true`, or `status` shows `"hx":true`.
2. Line slack, motor stopped: `{"id":1,"cmd":"cal","what":"tare"}`. `tension_n` reads about 0.
3. Hang a known weight on the line, for example 1 kg = 9.81 N. Read `tension_n`.
4. `new = old x 9.81 / shown`. Get the old value with `{"id":2,"cmd":"get","key":"hx_n_per_count"}` (default
   0.0000476). Set it: `{"id":3,"cmd":"set","key":"hx_n_per_count","val":0.0000512}` (your number).
5. If `tension_n` goes negative when you pull: battery unplugged, swap the A+ and A- wires on the HX711.

## (f) Wiper direction in the base and cock geometry check

This checks that FORWARD really cocks, that `SW_HOME` opens where the firmware expects, and that cocking stops before
the release tooth. The springs are hooked, so treat it as loaded: no dart, nobody and no hands in front of the
carriage or the rail.

1. Battery unplugged. Springs hooked, no dart. Carriage fully forward.
2. Plug the battery in. `{"id":1,"cmd":"mode","to":"LAUNCH"}`. Check `H` and `G` are in telemetry `sw`.
3. On a new build: `{"id":2,"cmd":"cal","what":"cock_rest"}`.
4. Aim first: `{"id":3,"cmd":"aim","deg":30}` and wait for `AIMED`. (After a shift into LAUNCH only an aim run tells
   the firmware where the dogs sit. Cocking straight after a shift can fault with `BAD_STATE` "carriage not drawn".)
5. `{"id":4,"cmd":"cock"}`. Watch the carriage and telemetry:
   - The carriage must move BACK (toward the springs' stretched side). If it does not move and you get `BAD_STATE`
     "carriage not drawn": the wiper turns the wrong way for FORWARD. Battery unplugged, swap M+ and M-, then redo
     (a) step 4 (`enc_sign`) and this step.
   - `H` must disappear almost at once: the first tooth takes the rack a few degrees after rest. If it opens too far
     from there you get `FAULT` `SWITCH_DISAGREE` "SW_HOME opened at the wrong angle": the contact sits in the wrong
     place, or the number in `config.h` needs changing (tell the firmware author).
   - `cock` rises to 1 and a `COCKED` event comes. `H` stays open.
6. Look: the carriage is drawn back and the last tooth still holds it, with clear margin before the relieved tooth. If
   it stops too early or too late, write down how many degrees and tell the firmware author (`cockEndDeg` in
   `config.h`; it is not a `set` key).
7. Let it down without firing: `{"id":5,"cmd":"cal","what":"decock"}`. The wiper runs BACKWARD slowly and the spring
   unwinds under the worm until `H` closes. A `RESET` event with `"how":"decock"` comes. This also turns the aim train,
   so the aim is unknown afterwards (the next `aim` homes again). Check `G` is closed again.
