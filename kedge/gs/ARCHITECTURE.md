# GrappStar ground station (launcher + reel) on ONE wiper motor. Spec v4 (3 Nov 2026, final CAD gs/out/gs_parts.json, after the second sear design review)

Prototype scale = sized to the real parts: KEDGE v30 dart (folded 168.7 long, belly Ø42, pods y ±40, kick leg up to z 77
above the dart axis, 0.28 kg with ballast), 1 mm Dyneema 20 m, one 12 V car wiper motor, BTS7960, ESP32-S3 DevKitC-1.

## 1. Principle (underactuation)
One motor, one main shaft (Ø12 steel, axis = Y, horizontal, across the rover). Direction selects the job:
- FORWARD (F, motor "+"): reel only. Reel rotor is geared DIRECTLY to the main shaft (both directions).
  The launcher drive pinion sits on one way bearing A (2 x HF1216) and freewheels; backstop C (2 x HF1216, one in each cheek boss on the program gear stub) holds the launcher.
- BACKWARD (B, motor "-"): launcher program. Bearing A locks, the program gear turns. The reel rotor also turns backward,
  which RETRACTS its pickup pin (friction drag ring), so the line is free. That is exactly the state needed for launch.
- Motor stopped: the wiper worm is self locking, everything holds.

## 2. Launcher (elevating rail, program gear)
- Rail pivots on the main shaft. Rail coords: u along the rail (forward), w up (perpendicular), origin = main shaft axis.
  Dart axis is at w = +95 (DART_W). Elevation theta: 4 deg (LOW: load, cock, tow, drive) to 35 deg (FIRE).
- Program gear (PG, spur m3, 30 tooth pitch, 18 teeth present) on a stub shaft fixed in the rail at (u 66, w 0). Driven by
  the drive pinion (m3 z14) on the main shaft through bearing A. 2.143 main-shaft (wiper output) turns per PG turn
  (30/14). The AS5600 reads this same main shaft.
- PG meshes a rack bar under the carriage (rack pitch line w +45). Rack runs back from the carriage.
- Carriage slides in the rail under the cradle; its pusher stem comes up through a slot in the cradle and pushes the
  dart's tail cap ring (tether exits through the pusher centre hole).
- Springs: 2 extension springs, one each side, cords from the carriage over two pulleys at the muzzle (F693ZZ pairs)
  and back under the rail to anchors near the pivot. Stroke 208.2 mm (exact tooth polygons). 42 N at rest -> 192 N at
  full draw (total, 0.72 N/mm), 24.4 J stored. Cocking force includes shoe friction x1.21. Peak PG torque 11.6 N m; motor
  sees PG torque / (30/14) / 0.92: peak main-shaft torque 5.9 N m while cocking. Muzzle 10.7 m/s, range 8.4 m (leaves at 33 deg, 0.8 N end-pull peel), about 10 m of
  line out (physics_gs.json).
- Sear on the rail catches the carriage at full draw (clevis on an ISO 7379 5 x 25 shoulder bolt, arc catch face, 8-10 N
  spring, up-stop and down-stop at -24.5 deg). Trip CORD from a 3 mm dowel in the sear outer plate (out through an arc slot,
  so the cord runs outside the cheek in the plane y -50) to an M4 eye bolt on the trip post (height nut = trip adjust,
  preloaded die spring under the shelf gives if the cord is overdriven): it goes taut at phi 284.8 (rail 25.7 deg) and pulls the sear down as the rail rises; the
  hook lets go at about phi 294.5 (rail 33.0 deg): the shot goes near the top, every time, no extra actuator.
- SW_COCKED is a KW11 wired COM + NO, mounted outside the left cheek. The sear arm presses its lever while the hook sits
  up on its stop: CLOSED while the hook is up, OPEN while it is pushed down (mesh-solved phases from the CAD).
  Cocking: closed at the start (hook up, carriage forward), OPENS at phi 215.2 (keel pushes the hook down), CLOSES at
  241.2 (keel passed, hook up behind it). Firing: OPENS at phi 284.8 (cord taut) and stays open until the rail drops. So "closed" does not by itself mean cocked: the firmware also needs phi in the cocked dwell, and saves phi to
  flash at every stop so it survives a power cycle.
- PG also carries a lift cam (disc beside the gear). It bears on a roller fixed to the frame. Cam profile is generated as
  the envelope of the roller so theta(phi) follows the program exactly.

PG program, phi = PG angle (deg), motor runs B (theta(phi) at 0.5 deg steps: gs/out/theta_table.json):
| phi | what happens | theta |
|---|---|---|
| 340 (= -20) | first tooth meets the rack back end (carriage at rest) | 4 |
| 340 -> 246.5 | teeth drive the rack, carriage drawn back 208.2 mm. SW_COCKED opens at 215.2, closes again at 241.2 | 4 (cam base circle) |
| 246.5 -> 252.5 | last tooth lets go, carriage runs 1.5 mm onto the hook (18 mm keel, printed on its side). STATE "COCKED": cocking stops at 249.5 (dwell) | 4 |
| 252.5 -> 302.5 | cam lifts the rail (blended constant speed: the rail is a planet carrier, see physics) | 4 -> 35 |
| 284.5 -> 294.5 | trip cord taut, pulls the sear down (SW_COCKED opens at 284.8); it lets go at about 294.5: FIRE. Rack slides over the toothless arc | 25.3 -> 33.0 |
| 304.5 -> 332.5 | cam down flank, rail drops (overruns on bearing A), lands on rubber bumpers | 35 -> 4 |
| about 333 | fire run stops as SW_LOW closes (phi re-zeroed at the debounced edge), before the first tooth meets the rack again at 340 | 4 |
Cocking runs from about 333 through 360 to 249.5, at full duty to phi 230 then slow (duty 160, about 1/3 speed) so the hook
drops in and the stop is precise. 30% duty stalls at the full-draw load with the assumed wiper, so 130 is the bench
starting value. Cocking is refused while phi is in the lift/drop zone 252.5 to 331, and whenever SW_LOW is open (rail up).

Safety rules (firmware):
- jog b refused for phi 241..338 (cocked or lifting), when the rail is up (SW_LOW open), and when phi is unknown but
  SW_COCKED last closed after an opening with the rail down (probably cocked). Checked every 5 ms while a jog runs: a
  jog b stops at 241, at the 241.2 closing (re-zero), or if SW_LOW opens. Command "finish" runs B from 241..331 until the rail lands,
  only when asked: it fires if cocked.
- F (tow, retrieve, jog f) refused while the carriage is partly drawn, phi 340 through 0 to 241 (held only by backstop
  C), unless command "force" (one F command, 10 s). The firmware includes 340..360 because the first tooth meets the
  rack at 340, so the carriage is already drawn there.
- Cocking: trip 12 A and stall 300 ms. Everything else: 18 A and 600 ms.
- Cocking with phi unknown on a launcher that is already cocked: SW_LOW opens as the rail starts to lift (phi about
  256), the firmware stops at once (no shot) and phi becomes known. Then "finish" if wanted.
- Saved phi is checked against the switches at power up and before cock, finish and jog b (rail up or down, hook up in the
  dwell). If they disagree, phi becomes unknown.
- Rail up with phi unknown (for example a power cut during a fire run): cock, finish and jog b are refused. Command
  "lower" (only in that state, only when asked) runs B slowly (duty 160) with the cocking limits (12 A, 300 ms) until
  SW_LOW closes. The rail closes SW_LOW when it drops (the PG overruns to 332.5 and the falling rail turns round the pinion,
  so phi is re-zeroed to 333 at the debounced SW_LOW edge and the run stops there with phi known (see review rounds 10-11 below). From the lift
  side it passes the trip cord: it FIRES if the launcher was cocked.

## 3. Reel (spincast, stationary spool, on the rail)
- Axis parallel to Y, centred at the dart axis height (w 95), just behind the carriage at full draw, on the -Y side.
  Spool + rotor occupy y -66..-14. Exit eye on the reel axis at about y -11 (the line then angles about 10 deg in to the dart tail at y 0), so the line turns 90 deg in
  the eye and runs straight forward through the pusher to the dart tail.
- Spool is fixed to the rail (never turns). Rotor (cup with skirt over the spool front, like the v7 wrap cup) is keyed to the
  reel stub shaft. Stub shaft gear (z39) meshes a gear on the main shaft (z58): rotor 58/39 = 1.487 x main-shaft speed.
- Pickup pin in the rotor rim: a friction drag disc (O-ring on the stationary hub) with a cam slot moves the pin OUT when
  the rotor turns F (wind) and IN when it turns B. Pin in = smooth rim = line peels off the end of the stationary spool
  (end pull: nothing spins at launch, no overrun, about 0.8 N).
- Capacity: 20 m of 1 mm Dyneema (core Ø36, flange Ø58, width 26: 21 turns a layer, 8 layers, outer Ø51).
  Line per main-shaft F turn: 0.17 m empty spool to 0.23 m full (firmware tracks the fill).

## 4. Mission cycle (controller)
1. READY: PG at COCKED (phi known, 241..252.5), SW_COCKED (hook up) and SW_LOW closed, theta 4. FIRE also needs
   SW_SEATED (no dry fires).
2. FIRE: run B from 249.5 until SW_LOW closes after the drop, then stop with phi re-zeroed to 333 (AS5600 count on the main shaft; it cannot see the rail, and bearing A overruns in the drop). Shot evidence, any one of:
   SW_COCKED open between phi 278 and 300 (trip cord pulled the sear, opens at 284.8), SW_LOW opens then closes again (rail went up
   and came down), or as a fallback phi past the trip angle with the lift current gone. The shot itself does not load the
   PG, so there is no current signature of the shot. If SW_COCKED never opened between 278 and 300: FAULT "sear did not
   trip". If SW_LOW is not closed by 339.5: FAULT. Line pays out passively: line out is estimated by time (10.7 m/s,
   capped 12 m, CAD line out 9.6 m) or set by the Jetson.
3. Dart lands and anchors (dart firmware, WiFi).
4. TOW: run F (pin out, rotor winds). Tension limit from the HX711 load cell if fitted, else BTS7960 current (IS pin)
   -> line N: about 8.5 N per amp above free running in the CAD model (worm and pin friction vary, so calibrate; the
   HX711 is strongly recommended). Taper speed over the last 30%. Stop and hold at the limit (worm self-locks).
5. RELEASE: dart un-anchors (drills out, pods up, kick back).
6. COCK: run B from phi about 333 through 360 to COCKED 249.5. SW_COCKED opens at 215.2 and closes again at 241.2, before the
   carriage has settled: the firmware re-zeroes phi to 241.2 on that closing edge (only if it opened first in this run, or
   the keel was already on the hook at the start) and runs on to 249.5. FAULT if it has not re-closed by 252.5, or if it
   opens more than 20 deg away from 215.2 (phi was wrong).
   phi is re-zeroed only by SW_COCKED: after the drop the PG runs ahead of the pinion (overrun on bearing A), so the
   shaft count lags the real PG until the pinion catches up; it is correct again once the teeth re-engage.
7. RETRIEVE: run F, dart dragged tail first up the muzzle scoop into the cradle until SW_SEATED closes -> stop. SW_SEATED
   sits 4 mm in front of the seated tail, so the winch stops with the tail 0 to 4 mm from the pusher and no load on it.
   Slows over the last 1.5 m of the line estimate. The fallback is a current spike (4 A for 150 ms), which in the plant
   model puts about 80 N on the tail. Back to READY. No hand operation.

## 5. Rover-side electronics
ESP32-S3 DevKitC-1, BTS7960 (RPWM GPIO4, LPWM GPIO5, R_EN GPIO6, L_EN GPIO7, R_IS+L_IS -> GPIO1 via divider), 12 V
(3S LiPo, 20 A fuse). AS5600 on the main shaft end (I2C SDA GPIO8 SCL GPIO9, magnet on the shaft end). Switches to GND
with pull ups: SW_LOW (rail at 4 deg) GPIO10, SW_COCKED (KW11 outside the left cheek, COM + NO, pressed by the sear arm: closed = hook up) GPIO11,
SW_SEATED (dart tail within about 4 mm of the pusher) GPIO12. Optional HX711 load cell under the exit eye (DOUT GPIO13, SCK GPIO14). WiFi AP "GRAPPSTAR-GS"
on its own subnet, 192.168.5.1 (the dart AP "KEDGE" stays 192.168.4.1); the Jetson joins both, one WiFi interface each.
Firmware: fw/gs/ (README for wiring, keys, API, tuning).
Direction convention: motor + (RPWM) = F = reel wind. Motor - (LPWM) = B = launcher program.

## 6. Bought parts (ground station)
Wiper motor 12 V (in hand), BTS7960 (in hand), ESP32-S3 DevKitC-1 (in hand), AS5600 + magnet (in hand), HX711 + load
cell (in hand, optional), 12 mm hardened rod (h6, 400 mm), 2 steel 12 mm flange hubs, 608ZZ cam roller, 18 x 24 machined clutch sleeves, 8 mm steel rod (100 mm), HF1216 one way bearing x4 (2 for bearing A, 2 for backstop C), 6001-2RS
bearing (12x28x8) x6, F693ZZ x4 (in hand), 2 extension springs (spec in the BOM), Dyneema 1 mm 20 m (in hand) + 2 mm
cord 1 m for the springs, M2, M3 and M4 screws (full list in the build guide), M4 heat-set inserts, ISO 7379 5 x 25 M4 shoulder bolt (sear), O-ring for the drag disc, rubber bumpers x2, microswitches x3 (SW_COCKED a KW11 used on COM + NO), 10 k resistors x2 (BTS7960 R_EN / L_EN pull-downs).

## Planet carrier (review round 6)
The program gear rides on the rail round the drive pinion on the pivot axis, so main shaft angle alpha = theta - phi * 30/14.
While the cam lifts the rail, one shaft turn moves the PG by 1 / (30/14 - dtheta/dphi): motor torque grows by
(30/14) / (30/14 - dtheta/dphi). The lift is a blended constant-speed motion over 50 deg of PG (peak slope 0.83), so the factor
stays at or under 1.63; fire is at 35 deg. physics_gs.py, timeline.py, the firmware phi tracking (only while SW_LOW is open)
and the PC test plant all carry this term. When the rail drops (an overrun) the pinion ends 14.2 deg of PG behind: the
firmware subtracts that at the SW_LOW closing edge.

## Launch hardening (review round 8)
Carriage end stop: two 40 x 7 x 6.5 mm 90A urethane pads, rear 28 mm in pockets in solid blocks joined to the side wall
and lip (carriage KE about 3.1 J; the spring KE stays in the springs). Rail check strap: 2 mm Dyneema from a pin on the
right cheek (rail u 150, w 20) to a sprung eye post at world (130, +50, 46), taut at rail 36 deg (the cam tops out at 35):
it takes the recoil kick (about 0.29 N m s). Main springs: screw-in or swivel ends, foam surge dampers, flanged grooved
pulleys. Pusher hole 14 mm; the bridle knot sits inside the tail cap.

## Review round 9
Bearing A is a roller clutch: after the free-fall drop it relocks at once (no dead band), so the firmware stops B at the
SW_LOW closing edge and re-zeroes phi to 333 (the next cock re-zeroes at the sear switch); "finish" and "lower" end the
same way. Each B run first writes "phi unknown" to flash, so a power cut mid-run never restores a stale cocked phi. Fire
eases to duty 170 from phi 285 to 302.5 so the rail does not leave the cam over the top blend. Check strap at 37.5 deg with
a long die spring (about 1.5 J), keel and stem slots run 16 mm past rest, stop pads 40 x 14 x 6.5 with the spring cord
through them, bumpers 16 x 15 for the free-fall drop, flange hubs are standard 12 mm rigid flange couplings (32 x 5, 24 PCD).

## Review rounds 10-11 (firmware drop handling)
FIRING after the top: duty drops to topDuty 170 (285..302.5) then dropDuty 120 from 302.5, so the free fall (about 0.2 s)
has time margin before the rack at 340. The drop edge (SW_LOW closing, accepted only near the top) re-zeroes phi to 333 at
the debounced edge and ends the run. A dropMaxMs timer (300 ms, measure the real fall) from phi 304.5 faults if SW_LOW
never closes; SW_LOW never opening by 267.5 also faults. If the rail lands while the motor is stopped (a stop inside the
fall) phi is re-zeroed too. finish started with the rail up tracks the drop; finish after the drop is refused as done.
