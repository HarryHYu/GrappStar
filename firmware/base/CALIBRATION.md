# Base station calibration

Do BENCH_TEST.md first. Steps (a) and the first part of (b) can be done on the bench. The rest needs the base
assembled.

- Every command below goes into the Serial Monitor (line ending "Newline"). Any `id` number works.
- Every value you `set` or learn with `cal` is **saved in flash automatically** (once everything has stopped) and is
  kept over a power cycle and over a new upload. Read a value back with `get`, for example
  `{"id":1,"cmd":"get","key":"amps_per_volt"}`.
- If something faults, read the `FAULT` line (`code` and `why`), fix the cause, then send `{"id":1,"cmd":"clear"}`.
- `stop` is always there: `{"id":1,"cmd":"stop"}`. Keep it pasted in the box, ready to send.

Nothing here has been tried on the real base yet. Expect to adjust.

## (a) AS5600 magnet and direction

1. Battery unplugged. USB only. Magnet on the shaft end, 1 to 3 mm from the chip, centred.
2. Send `{"id":1,"cmd":"status"}`. You want `"magnet":"OK"`.
   - `NONE`: no magnet seen or no I2C. Check the STEMMA QT cable and the magnet.
   - `WEAK`: move the magnet closer. `STRONG`: move it further away.
3. The wiper output cannot be turned by hand (its worm locks it). So for this check turn the magnet instead: before
   it is glued, hold it over the chip at 1 to 3 mm and turn it slowly (or turn the AS5600 board round the magnet).
   Watch `wiper_deg` in telemetry: it must change smoothly, about 360 per turn. Then fix the magnet on the shaft.
4. Direction (needs the battery). Battery unplugged: check the motor is clamped and nothing is attached to its shaft.
   Then plug the battery in. Drum in NEUTRAL (or out of the base, as in the bench test). Send:

   ```
   {"id":2,"cmd":"jog","motor":"wiper","dir":1,"ms":500}
   ```

   Dir 1 is FORWARD. `wiper_deg` must go UP. If it goes DOWN, wait until it has stopped, then send:

   ```
   {"id":3,"cmd":"set","key":"enc_sign","val":-1}
   ```

   Jog dir 1 again: now it must go up. (`BUSY` means it was still moving: send again.)
5. In `status`, `glitches` counts rejected AS5600 readings. A few are fine. If it climbs fast while the motor runs,
   route the AS5600 cable away from the motor wires.

## (b) Wiper current

### Current zero

1. Battery plugged in, wiper stopped (`"amps"` should be near 0 anyway).
2. Send `{"id":1,"cmd":"cal","what":"current"}`. `amps` now reads 0.0 when stopped.

### Current scale (needs a multimeter with a 10 A range)

If WIRING.md section 3 found an onboard IS resistor, set the computed `amps_per_volt` first.

1. Battery unplugged. Put the meter in series with the motor + wire (BTS7960 M+ to meter 10 A jack, meter COM to the
   motor red wire). In the MOTOR wire, not the battery wire. Meter on the 10 A DC range.
2. Plug the battery in. Jog for 1.5 s:

   ```
   {"id":2,"cmd":"jog","motor":"wiper","dir":1,"ms":1500}
   ```

   Sending the same line again before it ends keeps it running (copy the line, then paste and Enter a few times).
3. Read the meter once it is steady, and `amps` in telemetry at the same moment.
4. `new value = old value x meter amps / shown amps`. The old value is 12.5 unless you changed it. Example: meter
   1.8 A, shown 1.5 A: 12.5 x 1.8 / 1.5 = 15.0. Send `{"id":3,"cmd":"set","key":"amps_per_volt","val":15.0}`.
5. Jog again and check they now agree. Then battery unplugged, take the meter out, reconnect the motor wire directly.
   The 10 A range must never see a stall.

### Free running current

The tension estimate uses the current above free running. Learn it in REEL mode with the line slack (the base
assembled, after (c)). On the bench you can try it with the bare motor, but redo it in the base.

1. Line slack. `{"id":1,"cmd":"mode","to":"REEL"}` and wait for the `MODE` event with `"ok":true`.
2. Send `{"id":2,"cmd":"jog","motor":"wiper","dir":1,"ms":1500}`, then within the 1.5 s, after at least 0.8 s, send
   `{"id":3,"cmd":"cal","what":"free"}`. `NOT_RUNNING` means you were too early or too late: try again.
3. Easier way: jog, read `amps` in telemetry while it runs steady, then
   `{"id":4,"cmd":"set","key":"free_a","val":1.6}` (your number).

## (c) Drum position and modes

The drum position comes from three code contacts (a Gray code, BENCH_TEST.md step 2). The code is absolute, so there
is no home search: the base always knows which sector the drum is in, even after a power cut.

1. Battery unplugged. The drum N20 is in place, the three drum code contacts are wired (the bench jumper is OFF), and
   the line is slack.
2. USB plugged in, battery still unplugged. `{"id":1,"cmd":"cal","what":"drum"}` replies `"ok":true` and sends a `MODE`
   event with the `drum_pos` the code shows. `POSITION_UNKNOWN` means the code is invalid (000, or a jump): check the
   code contacts (BENCH_TEST.md step 2).
3. Plug the battery in. Direction check: `{"id":2,"cmd":"jog","motor":"drum","dir":1,"ms":300}`. Drum dir 1 must turn
   the drum from LAUNCH toward REEL. If it turns the other way: battery unplugged, swap the two N20 wires at AOUT1 and
   AOUT2. (A wrong direction also shows up during a mode change as `FAULT` `DRUM_SENSOR` "drum turned the wrong way".)
4. Now try each mode and wait for each `MODE` event with `"ok":true`. The drum is slow: a change can take several
   seconds.

   ```
   {"id":3,"cmd":"mode","to":"NEUTRAL"}
   {"id":4,"cmd":"mode","to":"REEL"}
   {"id":5,"cmd":"mode","to":"NEUTRAL"}
   {"id":6,"cmd":"mode","to":"LAUNCH"}
   ```

   After each one, telemetry `drum_pos` and `mode` must name that position, and the dog collar must sit where it
   should (look).
5. The wiper may turn a little during a mode change. Keep clear:
   - leaving LAUNCH it first turns a few degrees BACKWARD to take the load off the dogs;
   - after a shift into LAUNCH or REEL, if `SW_DOG` says the dog teeth sit tip to tip, it wiggles the wiper slowly
     (up to about 60 deg) until they drop in. If they never do: `FAULT` `DRUM_TIMEOUT` "dogs stayed tip to tip".
6. The drum speed (`drum_dps`) is learned by itself between code edges. See it with
   `{"id":7,"cmd":"get","key":"drum_dps"}`. Nothing to do.
7. If you get `DRUM_TIMEOUT` or `DRUM_SENSOR`: read the `why`. Battery unplugged, check the code contacts close and
   open cleanly as the drum turns, then `clear` and try again.

## (d) Aim

You need a phone inclinometer app (any "level" app that shows degrees). Base assembled, no dart, nobody in front of
the rail.

1. Battery unplugged. Fix the phone flat on the rail (tape or a rubber band), along the rail.
2. Plug the battery in. `{"id":1,"cmd":"mode","to":"LAUNCH"}`.
3. The launcher must be at rest: carriage fully forward, `H` in telemetry `sw`. On a new build, declare it:
   `{"id":2,"cmd":"cal","what":"cock_rest"}`.
4. See the allowed range: `{"id":3,"cmd":"get","key":"pitch_range"}` (it replies `min` and `max`).
5. Aim at a middle pitch: `{"id":4,"cmd":"aim","deg":30}`. If the aim is unknown it first turns until `SW_RAIL_LOW`
   closes (home), then goes to 30. That can take a while: one aim cycle is several wiper turns. Wait for the `AIMED` event with `"ok":true`.
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

   Get the old value with `{"id":2,"cmd":"get","key":"aim_home_deg"}`, then set the new one, for example
   `{"id":3,"cmd":"set","key":"aim_home_deg","val":3340}`. If the reply is `OUT_OF_RANGE`, tell the firmware author
   (see the note below).
3. Changing `aim_home_deg` does not re-home by itself. Keep jogging backward until `L` appears in telemetry `sw` (use
   short steps near the end and stop as soon as it appears). This can take up to one full aim cycle (several wiper
   turns). Then `{"id":4,"cmd":"cal","what":"aim_home"}`.
4. Redo steps 5 to 7 above.

Note: at the time of writing the `set` range for `aim_home_deg` is -720 to 720, but the default in `config.h` is
3365. Until that is fixed in the firmware, `set` may refuse the value you need.

## (e) Tension

### `k_n_per_a` (line newtons per amp, used without a load cell)

You need a luggage scale. Do (b) free current first.

1. Battery unplugged. Tie the line end to the luggage scale. Hook the scale to something solid. Line slack.
2. Plug the battery in. `{"id":1,"cmd":"mode","to":"REEL"}`, then `{"id":2,"cmd":"wrap"}` and wait for `WRAPPED`.
   The capstan only pulls with the wraps on.
3. Stand to the side of the line, never in line with it. Jog in short steps:
   `{"id":3,"cmd":"jog","motor":"wiper","dir":1,"ms":300}`. Watch the scale.
4. When the scale reads about 20 N (2 kg), read `amps` in telemetry while it pulls, then let the jog end.
5. `k_n_per_a = 20 / (amps - free_a)`. Example: amps 4.0, free_a 1.5: 20 / 2.5 = 8.0.
6. To release: unhook the scale by hand (20 N is safe to hold). The capstan never turns back (sprag #2).
7. Do it 3 times. Use the average: `{"id":4,"cmd":"set","key":"k_n_per_a","val":8.0}`.

### HX711 load cell (only if fitted)

1. Check `HELLO` said `"hx711":true`, or `status` shows `"hx":true`.
2. Line slack, motor stopped: `{"id":1,"cmd":"cal","what":"tare"}`. `tension_n` reads about 0.
3. Hang a known weight on the line, for example 1 kg = 9.81 N. Read `tension_n`.
4. `new = old x 9.81 / shown`. Get the old value with `{"id":2,"cmd":"get","key":"hx_n_per_count"}` (default
   0.0000476). Set it: `{"id":3,"cmd":"set","key":"hx_n_per_count","val":0.0000512}` (your number).
5. If `tension_n` goes negative when you pull: battery unplugged, swap the A+ and A- wires on the HX711.

## (f) Cock geometry check

This checks `SW_HOME` opens where the firmware expects and that cocking stops before the release tooth. The springs
are hooked, so treat it as loaded: no dart, nobody and no hands in front of the carriage or the rail.

1. Battery unplugged. Springs hooked, no dart. Carriage fully forward.
2. Plug the battery in. `{"id":1,"cmd":"mode","to":"LAUNCH"}`. Check `H` is in telemetry `sw`.
3. On a new build: `{"id":2,"cmd":"cal","what":"cock_rest"}`.
4. `{"id":3,"cmd":"cock"}`. Watch telemetry:
   - `H` must disappear almost at once: the first tooth is already engaged at rest, so the carriage leaves its stop
     right away (`homeOpenDeg` in `config.h`, a few degrees of wiper). If it opens too far from there (more than
     `homeWinDeg`), you get `FAULT` `SWITCH_DISAGREE` "SW_HOME opened at the wrong angle": the contact sits in the
     wrong place, or the number in `config.h` needs changing (tell the firmware author).
   - `cock` rises to 1 and a `COCKED` event comes. `H` stays open.
5. Look: the carriage is drawn back and the last tooth still holds it, with clear margin before the relieved tooth. If
   it stops too early or too late, write down how many degrees and tell the firmware author (`cockEndDeg` in
   `config.h`; it is not a `set` key).
6. Let it down without firing: `{"id":4,"cmd":"cal","what":"decock"}`. The wiper runs BACKWARD slowly and the spring
   unwinds under the worm until `H` closes. A `RESET` event with `"how":"decock"` comes. This also turns the aim train,
   so the aim is unknown afterwards (the next `aim` homes again).
