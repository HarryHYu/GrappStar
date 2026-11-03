# Base station calibration

Firmware `base-1.3.0`. Do BENCH_TEST.md first. Sections (a) and (b) can be done on the bench. The rest needs the base
assembled. Do them in order: (d) needs REEL from (c), and (f) needs (b) and (d).

- Two ways to send the commands below:
  - on the bench: the Arduino Serial Monitor (line ending "Newline"), as in README.md;
  - on the Jetson: stop the service first (`sudo systemctl stop grappstar`), run `python3 -m grappstar base monitor`
    in one terminal (telemetry and events), and send each command from a second terminal with
    `python3 -m grappstar base cmd '<json>'`, for example `python3 -m grappstar base cmd '{"id":1,"cmd":"status"}'`.
- Any whole `id` number works.
- Every value you `set` or learn with `cal` is **saved in flash automatically** (once everything has stopped) and is
  kept over a power cycle and over a new upload of base-1.2. Read a value back with `get`, for example
  `{"id":1,"cmd":"get","key":"amps_per_volt"}`.
- **Coming from base-1.0:** the old saved bench values are ignored by base-1.2 (new record format). Everything is back
  at its default after the upload: redo or re-enter every value below.
- In the first 60 ms after a start-up commands are refused with `BOOTING`: just send again.
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
Then send `{"id":1,"cmd":"cal","what":"cock_rest"}`. Use `"force":true` only when you have SEEN the carriage on
its stop and a contact is known broken: the firmware then trusts you completely, and a wrong force on a drawn spring
can FIRE on the next forward jog or `cock`. `SWITCH_DISAGREE` means `SW_HOME` or `SW_GAP` is open: the carriage is not
home or the big gear is not in its gap, or a contact is not touching.

## (a) AS5600 magnet and direction

1. Battery unplugged. USB only. Magnet in its cap on the shaft end, 1 to 2 mm from the chip, centred. Any small
   neodymium disc mounted edge-on works (decision D12), or a 6 x 2.5 mm diametric disc.
2. Send `{"id":1,"cmd":"status"}`. You want `"magnet":"OK"`.
   - `NONE`: no magnet seen or no I2C. Check the STEMMA QT cable and the magnet.
   - `WEAK`: move the magnet closer. `STRONG`: move it further away.
3. The wiper output cannot be turned by hand (its worm locks it). So for this check turn the magnet instead: before
   it is glued, hold it over the chip at 1 to 2 mm and turn it slowly (or turn the AS5600 board round the magnet).
   Watch `wiper_deg` in telemetry: it must change smoothly, about 360 per turn. Then fix the magnet in its cap.
4. Direction (needs the battery; done once in BENCH_TEST.md step 4, check it again in the base). Battery unplugged:
   check nobody's hands are near the shaft. Plug the battery in. Drum in NEUTRAL. Send a FORWARD jog (a backward jog is only allowed at
   the LAUNCH park, `BACKSTOP` elsewhere):

   ```
   {"id":2,"cmd":"jog","motor":"wiper","dir":1,"ms":500}
   ```

   Dir 1 is FORWARD. `wiper_deg` must go UP. If it goes DOWN, wait until it has stopped, then send:

   ```
   {"id":3,"cmd":"set","key":"enc_sign","val":-1}
   ```

   Jog again: now it must go up. (`BUSY` means it was still moving: send again.) The firmware also catches a wrong
   sign by itself while driving hard: `FAULT` `ANGLE_SENSOR` "AS5600 counts against the drive (enc_sign?)".
5. In `status`, `glitches` counts rejected AS5600 readings. A few are fine. If it climbs fast while the motor runs,
   route the AS5600 cable away from the motor wires.

## (b) Wiper current

### Current zero

1. Battery plugged in, wiper stopped (`"amps"` should be near 0 anyway).
2. Send `{"id":1,"cmd":"cal","what":"current"}`. `amps` now reads 0.0 when stopped. `OUT_OF_RANGE` means the zero
   would be over 5 A: something is wrong with the IS wiring (WIRING.md section 4).

### Current scale (multimeter with a 10 A range, low duty only)

Fit the 1 uF capacitor from GPIO1 to GND first if you have it (WIRING.md section 4, strongly recommended; the sketch
averages 16 reads across the PWM period, so the reading is right without it too). If WIRING.md section 4 found an
onboard IS resistor, set the computed `amps_per_volt` first.

The meter's 10 A range cannot take a start surge or a stall. So this is done only at low duty, with nothing loading
the shaft: a FORWARD jog with the drum in NEUTRAL runs the take-up at 40 % power with the 300 ms soft start (on the
bench: the bare motor with the GPIO11 jumper on). Never in LAUNCH or REEL with the meter in.

1. Battery unplugged. Put the meter in series with the motor + wire (BTS7960 M+ to the meter 10 A jack, meter COM to
   the motor red wire). In the MOTOR wire, not the battery wire. Meter on the 10 A DC range.
2. Plug the battery in. Drum in NEUTRAL. Jog forward for 1.5 s:

   ```
   {"id":2,"cmd":"jog","motor":"wiper","dir":1,"ms":1500}
   ```

   Sending the same line again before it ends keeps it running (copy the line, then paste and Enter a few times).
3. Read the meter once it is steady, and `amps` in telemetry at the same moment.
4. `new value = old value x meter amps / shown amps`. The old value is 12.5 unless you changed it. Example: meter
   1.8 A, shown 1.5 A: 12.5 x 1.8 / 1.5 = 15.0. Send `{"id":3,"cmd":"set","key":"amps_per_volt","val":15.0}`.
5. Jog again and check they now agree. Then battery unplugged, take the meter out, reconnect the motor wire directly.
6. From now on every higher current (full-duty free current, stall, `peak_a` in `status`) comes from `amps` in
   telemetry, never from the meter.

## (c) Drum direction, modes, wrap and unwrap

The drum position comes from three code contacts (a Gray code, BENCH_TEST.md step 2). The code is absolute, so there
is no home search: the base always knows which sector the drum is in, even after a power cut. The drum turns slowly,
about 8 deg per second.

1. Battery unplugged. The drum N20 is in place and in mesh (`n` in telemetry `sw`), the three drum code contacts are
   wired (the bench jumpers are OFF), and the line is slack.
2. On a new build send `{"id":1,"cmd":"cal","what":"cock_rest"}` first (carriage forward, `H` and `G` closed: see
   "New build" above). Without it every drum move out of LAUNCH or L_TO_N is refused with `LAUNCH_BUSY`.
3. USB plugged in, battery still unplugged. `{"id":2,"cmd":"cal","what":"drum"}` replies `"ok":true` and sends a
   `MODE` event with the `drum_pos` the code shows. `POSITION_UNKNOWN` means the code is invalid (000, or a jump):
   check the code contacts (BENCH_TEST.md step 2).
4. Get the drum to NEUTRAL (`drum_pos` `NEUTRAL`, only `b` of the code closed) by hand, because the motor direction
   is not checked yet. Battery still unplugged: loosen the cradle swing screw (M3, in the slot), swing the N20 out of mesh
   until `n` disappears from `sw`, turn the hand lever until only `b` is closed, swing the N20 back and tighten the
   screw (`n` closed). Turning the drum while the board runs
   gives `FAULT` `DRUM_SENSOR` "drum code changed with the drum still": expected, send `{"id":3,"cmd":"clear"}` once
   the drum is still (one `clear` is enough).
5. Direction check. Plug the battery in. Drum at NEUTRAL. Send a drum jog toward REEL (the longest jog, 1.5 s, is
   about 12 deg):

   ```
   {"id":4,"cmd":"jog","motor":"drum","dir":1,"ms":1500}
   ```

   The drum must move into the next sector toward REEL: `sw` shows `b` and `c` closed (code 0 1 1, WRAP_PREP),
   `drum_pos` `BETWEEN`. Then go back with `{"id":5,"cmd":"jog","motor":"drum","dir":-1,"ms":1500}`: only `b` again.
6. If instead `sw` shows `a` and `b` (code 1 1 0, L_TO_N, toward LAUNCH), the drum turned the wrong way: battery
   unplugged, swap the two N20 wires at AOUT1 and AOUT2. Bring the drum back to NEUTRAL with the hand lever as in
   step 4, plug the battery in and repeat step 5.
7. Now try the modes and wait for each `MODE` event with `"ok":true` (about 5 s each):

   ```
   {"id":6,"cmd":"mode","to":"LAUNCH"}
   {"id":7,"cmd":"mode","to":"NEUTRAL"}
   ```

   After each one, telemetry `drum_pos` and `mode` must name that position, and the dog collar must sit where it
   should (look). In LAUNCH `l` must be closed; in NEUTRAL `l` and `r` open.
8. Wrap, REEL and unwrap. Line threaded as in the build, the dart off the rail, a little line pulled out and lying
   slack. Stand clear of the line.

   ```
   {"id":8,"cmd":"wrap"}
   ```

   The drum turns through the WRAP window (the cup lays 7 turns on the capstan, `wraps` counts up in telemetry) and
   stops at the WRAPPED park (314 deg). Wait for the `WRAPPED` event (about 35 s; no `MODE` event comes). Telemetry:
   `drum_pos` `WRAPPED`, `mode` `NEUTRAL`, `wraps` 7, `l` and `r` open. Then:

   ```
   {"id":9,"cmd":"mode","to":"REEL"}
   ```

   Wait for `MODE` with `"mode":"REEL"` (about 5 s). `r` must be closed. Then:

   ```
   {"id":10,"cmd":"unwrap"}
   ```

   Wait for `UNWRAPPED` and `MODE` with `"mode":"NEUTRAL"` (about 40 s). `wraps` back to 0.

   `mode REEL` is refused (`NOT_WRAPPED`) unless the drum is past the wrap window: always `wrap` first. And with the
   wraps on, `mode LAUNCH` or `mode NEUTRAL` is refused (`WRAPS_ON`): always use `unwrap`. A drum `jog` is refused
   inside the wrap window (`WRAPS_ON`) and a running drum jog stops when it enters it: only `wrap` and `unwrap` cross
   it.
9. The wiper turns a little during these moves. Keep clear:
   - leaving LAUNCH it first turns 8 deg BACKWARD (the centering jog) so the dogs carry no load;
   - leaving REEL (start of `unwrap`) it first turns forward a little (up to 16 deg), then 8 deg back;
   - entering REEL it creeps forward slowly from drum 318 deg until the dogs bite (`r` closes, or the current rises
     0.8 A); if `r` is still open at the park it winds in 5 deg at a time, stopping at an estimated 20 N (the fly-in);
   - entering LAUNCH, if `l` does not close, it turns BACKWARD slowly (up to 40 deg) until the dogs drop in;
   - during `unwrap` it turns forward at 40 % while the drum is in WRAP and WRAP_PREP (the take-up winds the freed
     line).

   These moves need the AS5600: without it `mode`, `wrap` and `unwrap` are refused with `ANGLE_SENSOR`. If the dogs
   never drop in: `FAULT` `DRUM_TIMEOUT` "dogs did not engage (collar contact open)" (or "... line tension reached the
   fly-in limit" when the line is taut). Check the `l` / `r` contact
   closes when the collar is engaged. `SWITCH_DISAGREE` "collar contact disagrees with the drum code" means `l` or
   `r` was closed where the collar cannot be engaged (both are checked during every drum move).
10. If you get `SW_ARM counted n of 7 cup turns` (`DRUM_SENSOR`): the `SW_ARM` contact (`W`) is closed while the cup
    is away from home and must OPEN for about +-5 deg at home (cad-reel); the firmware counts one cup turn per
    opening. Check it with the drum still: `W` must disappear from `sw` while the cup is at home.
11. The drum speed (`drum_dps`, default 8.1) is learned by itself between code edges. See it with
    `{"id":11,"cmd":"get","key":"drum_dps"}`. Nothing to do.
12. If you get `DRUM_TIMEOUT` or `DRUM_SENSOR`: read the `why`. Battery unplugged, check the code contacts close and
    open cleanly as the drum turns (N20 swung out of mesh, `n` open, turn the hand lever), then `clear` and try again.

## (d) Free running current

The tension estimate uses the current above free running in REEL. Learn it with the wraps on and the line slack.

1. Line slack. Drum in REEL: `{"id":1,"cmd":"wrap"}`, wait for `WRAPPED`, then `{"id":2,"cmd":"mode","to":"REEL"}`
   and wait for `MODE` REEL.
2. Send `{"id":3,"cmd":"jog","motor":"wiper","dir":1,"ms":1500}`, then within the 1.5 s, after at least 0.8 s, send
   `{"id":4,"cmd":"cal","what":"free"}`. `NOT_RUNNING` means you were too early or too late: try again.
3. Easier way: jog, read `amps` in telemetry while it runs steady, then
   `{"id":5,"cmd":"set","key":"free_a","val":1.6}` (your number).

## (e) Aim

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
   At the end the wiper may turn FORWARD slowly for a moment: the backward aim run can drag the big gear back out of
   its gap (`G` opens), and the gap run closes it again before `AIMED`. `BAD_STATE` "SW_GAP did not close in the forward
   gap run" means check the `SW_GAP` contact.
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
   example `{"id":3,"cmd":"set","key":"aim_home_deg","val":3340}`. The allowed range is -7200 to +7200.
3. Changing `aim_home_deg` does not re-home by itself. Send `{"id":4,"cmd":"cal","what":"aim_forget"}`: the next
   `aim` homes again on `SW_RAIL_LOW`.
4. Redo steps 5 to 7 above.

## (f) Tension (must be done before the first tow)

**Do not tow before `free_a` (section d), `k_n_per_a` and `tension_offset_n` are calibrated.** Without a load cell
the firmware stops a tow on its tension ESTIMATE, not on a measured force. With the defaults on an uncalibrated base
the PC system test reached 61 N against a 40 N limit. The current trip (5 to 6.7 A, set from `max_n`) is only a backstop.

### `k_n_per_a` and `tension_offset_n` (used without a load cell)

The estimate is: line N = `k_n_per_a` x (`amps` - `free_a`) + `tension_offset_n`. Defaults 14.8 and -2.6 (D43). Since D43 these constants give the DART-side line tension (the tow limit is
the tension at the dart); the luggage scale at the line end measures exactly that side, so fit them straight from the
scale. Pulls against a luggage scale at three loads give both numbers (this matches MEASURE_LIST G9).
Do not use jogs for this: a jog trips at 3 A, about 15 N.

You need a luggage scale and a solid post. Do (b) and (d) first and do not change `free_a` afterwards. If a load cell
is fitted and live (`status` `"hx":true`), the firmware uses it instead: skip this part.

1. Battery unplugged. Tie the line end to the luggage scale. Hook the scale to the post. Line slack.
2. Plug the battery in. Drum in REEL with the wraps on: `{"id":1,"cmd":"wrap"}`, wait for `WRAPPED` (the wrap only
   parks at the WRAPPED park), then `{"id":2,"cmd":"mode","to":"REEL"}` and wait for `MODE` REEL. The capstan only
   pulls in REEL.
3. Stand to the side of the line, never in line with it. Wind in slowly:

   ```
   {"id":3,"cmd":"reel","duty":0.3,"until":"stop"}
   ```

   (reel-in trip 5 A). Watch the scale. When it shows about 10 N, read `amps` in telemetry at that moment (the
   monitor log shows the last lines), then send `{"id":4,"cmd":"stop"}`. The worm and sprag #2 hold the line. Write
   down the scale reading N1 and the current I1.
4. Send the same `reel` line again and stop at about 20 N (N2, I2), then again at about 30 N (N3, I3). Keep every load
   under 40 N: the reel-in trips at 5 A (about 49 N with the default constants) and stops at `limit_n` anyway.
5. Fit the numbers (use the lowest and the highest load):
   - new `k_n_per_a` = (N3 - N1) / (I3 - I1)
   - new `tension_offset_n` = N1 - new `k_n_per_a` x (I1 - `free_a`)

   Check the middle load: N2 should be close to new `k_n_per_a` x (I2 - `free_a`) + new `tension_offset_n`. If it is
   off by more than 3 N, repeat the pulls.

   Example with `free_a` 1.5: N1 10 at I1 2.35, N3 30 at I3 3.7. k = 20 / 1.35 = 14.8. Offset = 10 - 14.8 x 0.85 = -2.6.
6. Set them: `{"id":5,"cmd":"set","key":"k_n_per_a","val":14.8}` and
   `{"id":6,"cmd":"set","key":"tension_offset_n","val":-2.6}` (your numbers; the offset range is -50 to 50). Both are
   saved in flash.
7. To release the line: unhook the scale from the post by hand, slowly (30 N is about 3 kg: hold it firmly). The
   capstan never turns back (sprag #2).
8. Optional check: `{"id":7,"cmd":"reel","duty":0.3,"max_n":20,"until":"tension"}` against the scale: it should hold
   (`LIMIT` `"holding":true`) with the scale at about 20 N. `stop`, release as in step 7.
9. `{"id":8,"cmd":"unwrap"}` when done.

A `FAULT` `OVERCURRENT` "wiper over 5.0 A" during a pull means the load went too high or the line jammed: `clear`
and use lower loads. In the tow itself the trip is the current `max_n` needs plus 1 A, at least 5 A (D34; the
old `tow_trip_a` setting is no longer used). These constants cannot be set while a motor runs or the reel holds
(`BUSY`): `stop` first. On a very stiff line the tow now starts slow; residual overshoot in the simulation was under
10 percent of a 40 N limit at 3000 N/m.

### HX711 load cell (only if fitted)

1. Check `HELLO` said `"hx711":true`, or `status` shows `"hx":true`.
2. Line slack, motor stopped: `{"id":1,"cmd":"cal","what":"tare"}`. `tension_n` reads about 0.
3. Hang a known weight on the line, for example 1 kg = 9.81 N. Read `tension_n`.
4. `new = old x 9.81 / shown`. Get the old value with `{"id":2,"cmd":"get","key":"hx_n_per_count"}` (default
   0.0000476). Set it: `{"id":3,"cmd":"set","key":"hx_n_per_count","val":0.0000512}` (your number).
5. If `tension_n` goes negative when you pull: battery unplugged, swap the A+ and A- wires on the HX711.

## (g) Wiper direction in the base and cock geometry check

This checks that FORWARD really cocks, that `SW_HOME` opens where the firmware expects, and that cocking stops before
the release tooth. The springs are hooked, so treat it as loaded: no dart, nobody and no hands in front of the
carriage or the rail.

1. Battery unplugged. Springs hooked, no dart. Carriage fully forward.
2. Plug the battery in. `{"id":1,"cmd":"mode","to":"LAUNCH"}`. Check `H` and `G` are in telemetry `sw`.
3. On a new build: `{"id":2,"cmd":"cal","what":"cock_rest"}`.
4. `cock` needs the dart seated. For this check without a dart send
   `{"id":3,"cmd":"set","key":"require_seated","val":false}`. **Set it back to true at the end** (it also comes back on by itself at the next start-up; it is saved in
   flash).
5. Aim first: `{"id":4,"cmd":"aim","deg":30}` and wait for `AIMED`. (After a shift into LAUNCH only an aim run tells
   the firmware where the dogs sit. Cocking straight after a shift can fault with `BAD_STATE` "carriage not drawn".)
6. `{"id":5,"cmd":"cock"}`. Watch the carriage and telemetry:
   - The carriage must move BACK (toward the springs' stretched side). If it does not move and you get `BAD_STATE`
     "carriage not drawn": the wiper turns the wrong way for FORWARD. Battery unplugged, swap M+ and M-, then redo
     (a) step 4 (`enc_sign`) and this step.
   - `H` must disappear almost at once: the first tooth takes the rack a few degrees after rest. If it opens too far
     from there you get `FAULT` `SWITCH_DISAGREE` "SW_HOME opened at the wrong angle": the contact sits in the wrong
     place, the dog window is much wider than 16 deg (measure it), or the number in `config.h` needs changing (tell
     the firmware author).
   - `cock` rises to 1 and a `COCKED` event comes. `H` stays open.
7. Look: the carriage is drawn back and the last tooth still holds it, with clear margin before the relieved tooth. If
   it stops too early or too late, write down how many degrees and tell the firmware author (`cockEndDeg` in
   `config.h`; it is not a `set` key).
8. Let it down without firing: `{"id":6,"cmd":"decock"}` (the same as `cal decock`). The wiper runs BACKWARD slowly
   and the spring unwinds under the worm until `H` and `G` are both closed. A `RESET` event with `"how":"decock"`
   comes. This also turns the aim train, so the aim is unknown afterwards (the next `aim` homes again). The decock trips at
   3 A (D40). `FAULT` `BAD_STATE` "decock: SW_GAP never closed" means `H` closed but `G` did not: check the `SW_GAP`
   contact.

   If a decock fails: `clear`, `{"id":8,"cmd":"mode","to":"LAUNCH"}`, then `decock` again (backward at 3 A until `H`
   and `G` close). `SWITCH_DISAGREE` while the carriage is visibly drawn: `{"id":9,"cmd":"decock","force":true}`
   (decock by travel). If the decock faults again there is no powered way out (a backward jog is `NOT_AT_REST`, and
   `fire` needs `COCKED`, the dart seated, an aim and a token): battery unplugged, unload the springs by hand as the
   build guide says, then `{"id":10,"cmd":"cal","what":"cock_rest"}`, then `aim`.
9. `{"id":7,"cmd":"set","key":"require_seated","val":true}`.
