# First tests: bench to field

Do the stages in order. Do not skip ahead when a stage fails: fix it, then repeat the stage. Write the date and the
result of each stage in a notebook (or `docs/build/test_log.md`).

How to send base commands: the operator page on the Jetson (jetson/README.md) has buttons for most of these. From a
terminal on the Jetson, in `~/GrappStar/jetson`, send any command (firmware/base/PROTOCOL.md) with
`python3 -m grappstar base cmd '{"cmd": "jog", "motor": "wiper", "dir": 1, "ms": 300}'` and watch telemetry with
`python3 -m grappstar base monitor` in a second terminal. `'{"cmd": "estop"}'` stops everything and is always safe
to send.

Safety for every stage:
- Battery unplugged before you touch any wire.
- Hands out of the gears, the drum and the rail whenever the battery is plugged in.
- From stage 5 on: nobody in front of the rail, safety glasses on, the launch area clear for 30 m.
- Never fire a dart (dummy or real) without the tether tied on.
- Emergency stop: `estop`. If that does not stop it, pull lead #2's XT60 (the wiper branch): both motors die and the
  Jetson stays up. Keep a hand near it.

## Stage 0. Risk coupons (before any big print)

Print the day-one risk coupon plate from the base print pack first (section 4 of the build guide). It tests, in a
few hours of printing, the things most likely to go wrong: the swing splitter, a printed contact, the dog clutch,
the slip clutch, the bearing seats, heat-set inserts and nut traps, the key rib and the gear meshes. Work through
its coupon test sheet (below, or in `base/pack`).

Pass: every check on the coupon sheet passes.
If it fails: change the printer setting the sheet names (hole compensation, flow, material) and print that coupon
again. Do not start the big base parts until the coupons pass.

## Stage 1. Power-on (no motors attached)

Follow SYSTEM_ELECTRICAL.md section 4, the first power-on sequence, steps 1 to 9. Both motors stay disconnected
from their drivers until step 10.

Pass: no steady beeps on the short check, MP1584 #2 reads 9.0 V, the Jetson boots, `ping` replies, `vbat` within 0.3 V
of the meter, `fault` is null.
If it fails: a beep means a short (check the Y harness and the BTS7960 screws first). No reply: check the USB
cable is a data cable and in the ESP32 port marked USB; see troubleshooting in the build guide.

## Stage 2. Each motor alone, out of the base

Battery unplugged. Connect the wiper motor to BTS7960 M+ / M- and clamp the motor to the bench. Connect the drum
N20 to DRV8833 #3 AOUT1 / AOUT2, loose on the bench. Battery in.

1. `jog` wiper dir 1 for 300 ms, then dir -1. Mark with a pen arrow on the motor which way the shaft turns for
   dir 1. That is FORWARD: on the built base it is the direction that draws the carriage back (cocks). Before the
   base exists it is only a label; once assembled, a short forward jog in LAUNCH must move the carriage BACK. If it
   moves forward, swap the wiper leads at M+ / M- (battery unplugged).
2. `jog` drum dir 1 for 500 ms, then dir -1. Mark the shaft direction.
3. Do the current-sense calibration now (MEASURE_LIST G1, CALIBRATION.md), then watch `amps` during a 1.5 s wiper
   jog (jogs are capped at 1.5 s; sending the same jog again extends it).

Pass: each motor turns both ways, stops when the jog ends, wiper free current about 1 to 3 A (A6 in
MEASURE_LIST.md), no fault.
If it fails: no movement on the wiper: check R_EN / L_EN wiring and the 20 A fuse. Wiper turns only one way: one of
RPWM / LPWM is not connected. Drum does nothing: check 9.0 V on VM and the AIN1 / AIN2 / GND jumpers. If it still does nothing, check which board
it is (MEASURE_LIST D3): a board that does not pull SLP high by itself cannot be used as wired (decision D9); write
it in the test log. Wrong direction against CALIBRATION.md: swap that motor's two leads (battery unplugged).

## Stage 3. Sensors alone

1. Battery unplugged, ESP32 on USB only. Before the magnet holder is on the motor, turn the magnet by hand over the
   chip (the wiper worm locks its own shaft, so on the built base use short `jog`s instead): `wiper_deg` changes
   smoothly, no jumps, and goes up for the FORWARD way (the arrow you drew in stage 2).
2. Touch each printed contact closed with a finger: its entry in `sw` changes and changes back when released.
3. Load cell (if fitted): press it, `tension_n` rises.

Pass: every sensor responds and returns. If it fails: AS5600 not found: check SDA / SCL are not swapped and the
STEMMA cable is pushed in; angle jumps: magnet too far (keep 1 to 2 mm) or off centre. Contact never changes:
check the GND wire and that the screw head really touches the nut.

## Stage 4. Base assembled, dry cycles, NO springs, no line

Springs off, no tether, no dart. Run CALIBRATION.md first (zeros and drum positions; part (f) waits for stage 5).
With no dart, `arm` refuses (`NOT_SEATED`): for this stage and stage 5 send
`{"cmd":"set","key":"require_seated","val":false}`, and set it back to `true` before stage 6.

1. `mode` LAUNCH, NEUTRAL, LAUNCH: 20 rounds, then every other mode the firmware offers, in the order PROTOCOL.md
   gives. Each change ends with the right `drum_pos`.
2. `aim` 15, 30, 45, then the minimum and maximum pitch: 5 rounds. Rail reaches each angle within 1 degree
   (`pitch_deg`) and holds with the motor off.
3. Cock and fire with no springs: `arm`, then `fire` with the token. The carriage travels back, releases, and the
   big gear re-engages: 10 cycles.
4. Wrap and unwrap, with a spare line through the capstan path and slack: `wrap`, get into REEL, then `unwrap` and
   back to NEUTRAL, using the order in PROTOCOL.md (the `wrap`, `unwrap` and `mode` rows; the firmware tells you
   with `NOT_IN_REEL` or `WRAPS_ON` if the order is wrong): 10 cycles. Once wrapped and in REEL, send `reel` with
   duty 0.3, wait 3 s, send `stop`.
5. Send `estop` in the middle of each kind of move once. The wiper stops at once and the drum goes to NEUTRAL,
   except when the spring is drawn, the wraps are on or the drum itself faulted: then the drum holds (`DRUM_HELD`),
   which is correct (decision D27).
6. Fault test: with nothing moving, slide the AS5600 board away from its magnet (do not unplug anything). The base
   reports the magnet missing and refuses to move. Put it back, `clear`.

Pass: all cycles complete with no fault, no clicking or skipping teeth, no part warm to the touch, the drum never
stops BETWEEN positions.
If it fails: a STALL fault means something binds: find it by turning by hand with the battery unplugged. Skipped
teeth: check the mesh spacing and the frame screws. Drum stops BETWEEN: re-run the drum calibration; check the drum
contacts close cleanly.

## Stage 5. Springs on, no dart

Fit both springs. Start at the lowest pitch, rail pointing at a clear area.

1. `aim` to the lowest pitch. `arm` and `fire` once. Look at every part of the launch train after the shot.
2. Five more shots. Check the stop bumpers, the carriage, the rack teeth and the big gear's last tooth after each.
3. Watch `amps` while cocking: note the peak (the physics expects about 5 A; decision D8 trips at 6.5 A while
   cocking).

Pass: the carriage stops on the bumpers, nothing cracks or loosens over 6 shots, cocking current stays under the
stall value, the rail holds its pitch while cocking.
If it fails: cracks at the bumpers: stop and report (no more dry shots). Rail drops while cocking: the aim worm is
slipping, check its mesh. STALL while cocking: the springs are stronger than planned (redo B2) or something binds.

## Stage 6. Dummy dart WITH the tether, first shots

Never fire a dart (dummy or real) without the tether tied on. The tether is what keeps it flying nose first; a shot
with no line can tumble or land tail first.

Make the dummy: the plate A hull top and bottom, nose and tail cap (spare prints or the dart shells before the
electronics go in), taped shut with M8 nuts or sand inside to 279 g including the 15 g nose ballast (decision D10). Paint the
nose bright. Tie the line through the tail cap holes. Route the line as in the base assembly steps (for launch it
bypasses the capstan). Set the payout brake with hanging weights (G4): 40 g does not pull line off, 50 g does.

1. `aim` 20 degrees. Load the dummy, check `SW_SEATED` is closed. Fire.
2. Film it side-on with the phone in slow motion, tape measure on the ground in view (muzzle speed, G5).
3. Measure the range. Repeat at 30 and 36 degrees (G6).
4. Check the line paid out with no loops or overrun on the storage reel.

Pass: clean exit (no rubbing marks on the dummy), it flies nose first, payout is clean, the range is within 30
percent of the ballistics table at the same pitch (best about 5.2 m at 36 degrees with the two springs and the 0.5 N brake;
`base/physics/SUMMARY.md` has the current number).
If it fails: short range: measure the springs (B2) and the muzzle speed, then re-run the physics. Rubbing marks:
check the cradle and the barrel against the dart. Loops on the reel: raise the brake a little (never over 1 N).
Big range loss with clean payout: the line catches somewhere; watch the slow-motion video.

## Stage 7. Reel cycle with the dummy

1. After a stage 6 shot, while the line is still slack: wrap (decision D22: wrap right after landing), get into
   REEL, reel in, then bring the dummy home, all in the order PROTOCOL.md gives (spec D17: reel until about 1.5 m
   remain, unwrap, then the final approach in NEUTRAL until `SW_SEATED`).
2. `unwrap`, then `mode` NEUTRAL and `mode` LAUNCH. Check the line runs on the launch path again (not round the capstan).
3. Fire again at 30 degrees.

Pass: the wrap lays on the capstan, reel-in works, unwrap returns the line to the launch path, and the next shot
pays out cleanly. Repeat 5 times.
If it fails: wrap misses the capstan: check the wrap arm position at the end of `wrap`. Line still round the
capstan after `unwrap`: do not fire; unwrap again and check the drum position.

## Stage 8. The anchor in soil

The real dart, built and tested to the dart run sheet section 10. Use real soil (moist loam is the target).

1. Place the dart on the soil by hand, tether tied, line slack. Wrap the capstan now, before the dart anchors
   (decision D22). Then send `/auto` from the operator page (or key `a`).
2. When the dart reports ANCHORED: pull the tether by hand with the luggage scale, slowly, up to 50 N or until
   it moves (G7).
3. Re-anchor (wrap first, as in step 1). Base in REEL with wraps on: `reel` with `until` tension at a limit 10 N under the hand result.
4. Send release (operator page, or `python3 -m grappstar dart release`): drills out, pods up, foot out, kick back.
   Then reel the dart home.

Pass: the dart anchors on its own, holds at least 20 N in loam, the base stops at its tension limit, the release
sequence frees the dart, it reels home.
If it fails: augers back out: swap that motor's leads (dart run sheet section 10). Pulls out at low force: try
firmer soil, and report the force (it sets the reel limit). Base overshoots the limit: redo the current
calibration (G1).

## Stage 9. Full mission

From the operator page on the Jetson (jetson/README.md): preflight, pick a target on the Kiyo view, aim, confirm
the fire prompt, flight video from the dart, landing verdict, anchor, tow, release, reel home.

1. First run: a short target well inside the range the operator page offers (3 to 4 m: the minimum is about 2.9 m and the best
   about 5 m with the two springs), soft soil, open ground.
2. Watch every prompt: the software stops and asks before each dangerous step.

Pass: the whole sequence runs with only the expected operator confirmations, the dart lands within 2 m of the target,
anchors, the rover is pulled toward the dart, the dart releases and is reeled home.
If it fails: note the mission step and the fault code in the log (`mission.jsonl` on the operator page), then
repeat the stage where that step was first tested.
