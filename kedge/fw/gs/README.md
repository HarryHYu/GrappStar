# GrappStar ground station firmware v1

One wiper motor runs both the launcher and the reel (see `gs/ARCHITECTURE.md`).
Motor forward (F, RPWM) = reel winds. Motor backward (B, LPWM) = launcher program gear turns.

Files
- `grappstar_gs/grappstar_gs.ino`: hardware glue (pins, WiFi, web page, Serial, OTA).
- `grappstar_gs/gs_logic.h`: all the logic and the Tuning numbers. Tested on a PC by `fw/test/test_gs.cpp`.
- `fw/jetson/gs_client.py`, `mock_gs.py`, `mission.py`, `test_gs_client.py`: Jetson side.

Board: ESP32-S3 DevKitC-1, Arduino IDE board "ESP32S3 Dev Module". Libraries: HX711 (Rob Tillaart). Upload through
the USB port marked UART (or COM). Serial Monitor 115200, "No line ending".

## RULE

Unplug the LiPo (pull the XT60) every time a wire moves. Plug the battery in last, after you have checked the wiring.

## Wiring

| From | To | Note |
|---|---|---|
| 3S LiPo + | 20 A fuse (XT60 lead #2) then BTS7960 B+ | 10 A blew on inrush |
| 3S LiPo - | BTS7960 B- | |
| Wiper motor wire 1 | BTS7960 M+ | if `f` turns the launcher instead of the reel, swap M+ and M- |
| Wiper motor wire 2 | BTS7960 M- | |
| BTS7960 RPWM | GPIO4 | F, reel wind |
| BTS7960 LPWM | GPIO5 | B, launcher |
| BTS7960 R_EN | GPIO6 | driven high at boot. Add a 10 k resistor from R_EN to GND (see below) |
| BTS7960 L_EN | GPIO7 | driven high at boot. Add a 10 k resistor from L_EN to GND (see below) |
| BTS7960 VCC | ESP32 3V3 | logic supply |
| BTS7960 GND | ESP32 GND | common ground |
| BTS7960 R_IS + L_IS | tie together, then 1 k resistor to GPIO1 | current sense |
| GPIO1 | 680 R resistor to GND | the divider. Keeps GPIO1 under 3.3 V |
| AS5600 VCC / GND | 3V3 / GND | |
| AS5600 SDA / SCL | GPIO8 / GPIO9 | |
| AS5600 DIR | GND | fixed count direction |
| AS5600 magnet | end of the main shaft, 1 to 3 mm from the chip, centred | diametric magnet |
| SW_LOW microswitch | COM to GND, NO to GPIO10 | closed when the rail is down at 4 deg |
| SW_COCKED (KW11 outside the left cheek) | COM to GND, NO to GPIO11 | the sear arm presses the lever while the hook sits UP on its stop: closed = hook up, open = hook pushed down |
| SW_SEATED microswitch | COM to GND, NO to GPIO12 | closes when the dart tail is within 4 mm of the pusher |
| HX711 VCC / GND (optional) | 3V3 / GND | found automatically at boot |
| HX711 DT / SCK | GPIO13 / GPIO14 | load cell under the exit eye |
| ESP32 power | USB while on the bench | |

The two 10 k pull-downs on R_EN and L_EN: while the ESP32 resets or is being flashed its pins float, and a floating
enable pin can let the motor twitch. With 10 k to GND the BTS7960 stays off until the firmware drives R_EN / L_EN high.
Solder each resistor between the EN pin and a GND pin on the BTS7960 header.

Before the 1 k / 680 R divider: with everything unpowered, put the meter (ohms) between the R_IS pin and GND on the
BTS7960 board. If it reads a resistance (some boards already have one), write it down and tell Claude: `isAmpsPerVolt` changes.

## Keys (Serial Monitor, or the web page)

| Key | Does |
|---|---|
| `f` / `b` | jog reel wind / jog launcher. Serial: 1.5 s per press. Web: hold to run, release to stop |
| space or `s` | STOP everything (also aborts a cycle) |
| `x` | clear a FAULT |
| `c` | cock (launcher runs B; the sear switch opens then closes again behind the keel; slows from 230, stops at phi 249.5) |
| `F` `F` (capital, twice within 3 s) | FIRE (only from READY with the dart seated, switches checked again) |
| `t` | tow: wind until the line tension reaches the limit, then hold |
| `r` | retrieve: wind until the seated switch closes (tail within 4 mm of the pusher) |
| `y` `y` (twice within 3 s) | full cycle: fire, wait for `n`, tow, wait for `m`, cock, retrieve |
| `n` / `m` | anchored flag / released flag (for the cycle) |
| `l20` | set the tension limit to 20 N (2 to 150) |
| `+` / `-` | jog speed |
| `z` | zero. Motor stopped: zeros the current reading and tares the HX711 (refused while towing or in a cycle: the line may be loaded). While jogging `f` on a slack line: learns the free running current |
| `E` `E` (capital, twice within 3 s) | finish: run B from phi 241..331 until the rail lands (phi re-zeroed to 333). It FIRES if cocked (with or without a dart). Only when you mean it |
| `L` `L` (capital, twice within 3 s) | lower: only when the rail is up AND phi is unknown (for example after a power cut during a fire run). Runs B slowly (duty 160, 12 A / 300 ms limits as cocking) until the rail-down switch closes, re-zeroes phi there (288.5, the drop) and runs on to 338 like a fire run. It FIRES if the launcher was cocked. Only when you mean it |
| `O` (capital) | force: allow the next tow / retrieve / jog f while the carriage is partly drawn (10 s) |
| `i` / `p` | status line / status JSON |
| `h` | help |

Jog `b` is refused from phi 241 to 338 (cocked or lifting: it would fire), when phi is unknown but the launcher is
probably cocked, and whenever the rail is up (SW_LOW open). This is checked every 5 ms while it runs, not only at the key
press: a jog `b` stops by itself at phi 241, when the sear closes behind the keel, or if the rail starts to lift.
Use `F` `F` (fire) or `E` `E` (finish).
On the Serial Monitor, FIRE, finish, lower and cycle need the same capital key twice within 3 s, so one stray byte never fires.
Cock is refused while the rail is up. At power up (and before cock, finish, jog `b`) the saved phi is checked against
the switches (rail up or down, sear hook up in the dwell); if they disagree phi becomes unknown.
Rail up with phi unknown: cock, finish and jog `b` are all refused. Use `L` `L` (lower).
Tow, retrieve and jog `f` are refused while the carriage is partly drawn (phi 340 through 0 to 241): only the backstop
holds it then. Cock first, or send `O` (force) if you know why.
phi is saved to flash every time the motor stops, so after a power cycle the ground station knows where it is. If it
comes up with phi unknown and you press `c`, it stops by itself if the rail starts to lift (it was already cocked).

WiFi: join `GRAPPSTAR-GS`, password `grappstar`, open http://192.168.5.1 . Same keys, plus buttons. FIRE and finish ask first.

## API (for the Jetson)

- `GET /status`: JSON with state, cycle, phi, phi_known, theta_est, turns, line_m, current_A, tension_N, tension_src,
  limit_N, holding, switches {low, cocked, seated}, enc_ok, anchored, released, shot, fault, event.
- `GET /cmd?c=cock|fire|tow|retrieve|stop|cycle|limit|anchored|released|clear|line|zero|finish|lower|force&v=<number>`.
  `tow` v = distance in m (0 = until the limit). `cycle` v = tow distance. `limit` v = N. `line` v = line out in m.
  Answers 200 with the event text, or 409 if refused (busy, FAULT, not READY, dart not seated, carriage partly drawn).
- `GET /key?c=f|b&ms=350`: jog with deadman (ms capped at 1500). `GET /stop`.
- States: idle, cocking, ready, firing, payout, tow (holding true when stopped at the limit), retrieve, jog, lower, fault.
- OTA: Arduino IDE > Port > network port `grappstar-gs`.

Network: the ground station is its own access point on its own subnet, 192.168.5.1. The dart stays on `KEDGE`,
192.168.4.1. The Jetson joins both, one WiFi interface each (a USB WiFi stick for the second). See `mission.py`.

## Safety built in

- Over current: 12 A while cocking, 18 A otherwise. Stops at once, FAULT.
- Stall: driving above 30% PWM but the shaft has not moved for 300 ms while cocking, 600 ms otherwise: stops, FAULT
  (in tow it just holds instead).
- Fire: FAULT `sear did not trip` if SW_COCKED never opens between phi 278 and 300.
- Rail up with phi unknown: only `lower` may move the launcher (slow, cocking limits).
- AS5600 missing: the motor never starts. Lost while driving: FAULT.
- Soft start: PWM ramps from 0 to full over 300 ms.
- Direction change: always 200 ms stopped first.
- Deadman: web jog stops 350 ms after the last repeat, Serial jog after 1.5 s.
- The worm gear holds everything when the motor stops.

## Program angles (final CAD, phi = program gear angle, motor B)

| phi | what happens |
|---|---|
| 340 (-20) | first tooth meets the rack |
| 340 -> 246.5 | carriage drawn back 208.2 mm (42 N -> 192 N). Sear switch OPENS at 215.2 (keel pushes the hook down) and CLOSES at 241.2 (keel passed, hook up: phi re-zeroed here) |
| 246.5 -> 252.5 | last tooth lets go, carriage runs 1.5 mm onto the hook (18 mm keel, printed on its side). Cocking slows from 230 and stops at 249.5 |
| 252.5 -> 302.5 | cam lifts the rail 4 -> 35 deg |
| 284.5 -> 294.5 | trip cord (sear arm to an anchor on the base) goes taut at rail 25.7 deg and pulls the hook down (sear switch opens, stays open until the rail drops). Shot at about 294.5 (rail 33.0) |
| 304.5 -> 332.5 | rail drops onto the bumpers (PG overruns). Fire run stops as the rail lands (phi re-zeroed to 333) |

Cocking runs from about 333 through 360 to 249.5. Cocking is refused while phi is in the lift/drop zone 252.5 to 331.

## First power up, step by step

1. Wiper motor NOT connected to the mechanism yet (or springs unhooked). LiPo unplugged.
2. Plug in USB only. Serial Monitor. You should see `no HX711` or `HX711 found`, and the help line. Type `i`.
   It should show `idle`, switches, and no `AS5600 MISSING`. If AS5600 is missing: check SDA/SCL and the magnet.
3. Press each switch by hand and type `i` again: `L` and `S` go 1 when pressed. `C` shows 1 with the sear hook up
   (the sear arm presses the KW11 lever) and goes 0 when you push the hook down.
4. Unplug USB. Plug the LiPo in (fuse in the lead). Plug USB back in.
5. Type `z` (motor stopped: zeros the current offset).
6. Type `f` once. The motor runs 1.5 s. Type `i` right after: `turns` must have gone UP. If it went down, change
   `encSign` to -1 in the Tuning block of `gs_logic.h` and upload again.
7. Check the direction: `f` must turn the reel rotor. If it turns the launcher program gear, unplug the LiPo and swap M+ and M-.

## Tuning

IS scale: unpowered, tie R_IS and L_IS together and meter that point to GND (Rb, open = none). Rx = 1680 in parallel with Rb,
isAmpsPerVolt = 8500 x 1680 / (Rx x 680): 12.5 with no onboard resistor, about 54.5 with two onboard 1 k. Then calibrate
against a 10 A meter.
 (replace the placeholder numbers in `struct Tuning` in `gs_logic.h`, then upload)

Values learned with `z` are only kept until reset: copy them into the Tuning block.

1. Current reading (`isAmpsPerVolt`, placeholder 12.5). Put the multimeter (10 A range) in series with the motor + wire.
   Hold `f` on the web page with the reel free. Compare the meter with `current_A` on the page.
   New value = 12.5 x meter amps / shown amps.
2. Free running current (`iFreeA`, placeholder 1.5). Line slack. Hold `f`, after 1 s type `z` on the Serial Monitor
   (or press the zero button while holding f). It prints `learned iFreeA`.
3. Line newtons per amp (`kNperA`, placeholder 13.6 from the CAD model). Tie the line end to a luggage scale, hook the scale to something
   solid. On the web page press `-` four times (slow jog). Hold `f` and watch the scale. When it reads 20 N, read
   `current_A` on the page, then release. kNperA = 20 / (current_A - iFreeA). Do it 3 times, use the average.
4. HX711 (only if fitted, `hxNPerCount`). Line slack, motor stopped, type `z` (tare). Hang a 1 kg weight on the line
   (9.81 N). Read `tension_N`. New value = old value x 9.81 / shown N.
5. Cocking. With the springs hooked, type `c`. It should stop with the carriage resting on the sear, event
   `cocked: READY`, after `sear hook down` and `sear hook up behind the keel`. If it stops before the carriage is back
   on the hook, raise `phiCocked` (249.5) by 1. Never above 252 (the lift starts at 252.5). If the slow part (from 230)
   stalls, raise `cockSlowDuty` (130) by 10.
6. Firing. Place the dart (seated switch closed), clear the area in front, type `F`. Event shows `shot: ...` then
   `fired: PAYOUT`. If you get `FAULT: shot not seen`, send Claude the Serial log.
7. Tow limit. Default 20 N. KEDGE holding: dry sand about 7 N, moist loam 45 N, firm clay 110 N. Start at 20.
