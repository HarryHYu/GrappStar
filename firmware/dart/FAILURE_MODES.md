# KEDGE dart: failure modes and effects (electronics + firmware v4)

What can break, what the firmware notices, what it does, and what the Jetson or Harry should do about it.
Checked against firmware `kedge-dart-v4` at commit 17beeb3 (3 Nov 2026). Rows M5, M8, M10, M11, M20, I8, X1, P2, C4,
J3, O4 and O6 were updated for the afternoon fixes (see the status table at the end). If `kedge_logic.h` changes,
recheck the rows that name a threshold.

## How to read this

- Every time and threshold comes from `kedge_dart/kedge_logic.h` (`struct Tuning`, `struct Flight`) or `kedge_dart.ino`.
- Current thresholds are bench values at 8.0 V. They scale with the rail voltage (0.6 to 1.15 x) and, for manual keys,
  with (speed/255) squared. The mA numbers below are for a 7.6 V rail at full speed.
- "Lone motor" means it is the only motor running. The no-current check (open wire) only works on a lone motor.
- "Pair check": when two motors share the INA219, the worm (kick or deploy) pauses 150 ms every 550 ms and the other
  motor is read alone. One stall reading takes about 0.55 s, so two readings take about 1.1 s. A worm that is already at
  a stop when its step starts (so the current never rises) is believed after 3 of the last 4 readings, about 1.6 to 2.2 s.
- Check column:
  - **VERIFIED** means I followed the code path, and where it says so, also ran it:
    - `test_logic` = a check in `test/test_logic.cpp` (185 checks, all pass).
    - `sim` = the host simulator running the real sketch (`sim/kedge_sim`, fault knobs in `/sim/set`).
    - `scratch` = the `test_logic.cpp` plant model with a fault added (motor direction flipped between the firmware and
      the plant, a motor that draws 0 mA, a drill pair at half current, a custom accelerometer trace), run against the
      real `kedge_logic.h`. These programs are not kept in the repo.
    - `code` = code reading only.
  - **ASSUMED** means the row depends on how a part behaves (DRV8833, BMS, MP1584, ESP32 radio) and was not tested. The
    firmware's side of it may still be checked; the cell says so.

Fault codes latch: phase `FAULT` until `/stop`, `/auto`, `/release` or `/arm`. While a fault is latched, run keys get
409 `fault latched: send stop first`.

## 1. Motors

Normal anchor: kick swings alone, then kick + foot, then foot alone, then pods + drills, then drills alone.
Normal release: drills alone, pods + drills, foot alone, kick + foot (foot 3 s), then kick alone.
So the kick, foot and drills each run alone at some step. The deploy motor runs alone only for the first 0.3 s of the
anchor's pods step (the open-wire check), never alone in the release.

| # | Component or event | Failure mode | Firmware detects (threshold, time) | Firmware does | Jetson / Harry sees and does | Residual risk | Check |
|---|---|---|---|---|---|---|---|
| M1 | Any motor, manual key | Open wire (or short chopped by the driver) | Lone motor under 40 % of its free current (deploy and kick 11 mA, drills 27 mA, foot 17 mA) for 0.5 s, after 0.25 s start-up: about 0.75 s | That motor off, key locked until released, event `<motor> draws no current: wire open or shorted`. No fault code | Repeats get 409 `stalled: release the key first`. Plug out, check the motor wires with the meter | With two keys held at once there is no open-wire check | VERIFIED: code + test_logic (open drill wire, manual) |
| M2 | Kick, /auto | Open wire | Kick swing is a lone-motor step: no-current rule, about 0.75 s | `NO_CURRENT`, all off, `seq` aborted | Phase `FAULT`, event `kick draws no current...`. Fix the wire, `/auto` again | None worth noting | VERIFIED: code + test_logic (round 4) |
| M3 | Foot, /auto | Open wire | Not during planting (pair). At foot finishing (lone): 0.5 s into that step | `NO_CURRENT`, all off | Phase `FAULT`. Kick is already planted. Fix, then `/auto` (it sees the kick is down and plants the foot alone) | Foot spun nothing for up to 6 s first | VERIFIED: code + scratch |
| M4 | Drills (both), /auto | Open wire on the shared pair wires | Not while the pods swing (pair). At drills finishing (lone): 0.5 s into that step | `NO_CURRENT`, all off | Phase `FAULT`. Pods are already down with no drills in | Pods went down without cutting | VERIFIED: code + scratch |
| M5 | Deploy, /auto | Open wire, dead motor, stripped gear | Deploy-alone lead (first 0.3 s of the pods step): under 40 % of free current | `NO_CURRENT`, all off, event `deploy draws no current: wire open or shorted` | Phase `FAULT` | A stripped gear (current but no motion) is still not seen | VERIFIED: code + test |
| M6 | Deploy, /release | Open wire, dead motor | Nothing (same reason) | Event `pods time limit (stowed or stalled)`, sequence goes on, ends `released` | `seq` `released`, phase `IDLE`, pods still down. Jetson reels in a dart with pods out | Pods drag or snag on recovery | VERIFIED: code + scratch. See GAPS |
| M7 | Kick, /release | Open wire | Not while the foot runs (first 3 s of the kick step). Then lone: no-current rule | `NO_CURRENT`, all off | Phase `FAULT` | None worth noting | VERIFIED: code + scratch |
| M8 | Any motor | Short across its two outputs | ASSUMED: the DRV8833 protection chops it, so the INA219 average reads almost nothing: seen as an open wire (M1 to M7). If the INA219 catches over 2000 mA for 10 ms: `OVERCURRENT` | As M1 to M7, or `OVERCURRENT` with all off | As M1 to M7 | Deploy short seen in the anchor only (deploy-alone lead) | ASSUMED (driver behaviour) |
| M9 | Any motor | One lead shorted to ground or to the battery + | ASSUMED: one direction works, the other drives into the short and the driver chops it (as M8) | Same as M8, in one direction only | A motor that works one way and "draws no current" the other way | The anchor direction may work and only the release fails | ASSUMED |
| M10 | Kick, /auto | Jammed (or blocked by a stone) before it reaches the ground | First contact current already over the kick stall line, with the travel bookkeeping trusted and under 25 % of the swing | `KICK_JAM`, all off (after a non-power-on reset, travel unknown: still taken as `kick already down`, with a warning) | Phase `FAULT`, fault `KICK_JAM` | Jam near the ground (over 25 %) is taken as down | VERIFIED: code + test |
| M11 | Kick, /release | Jams half way back with stall current | Pair rule while the foot runs, or plain stall once the foot stops; travel back under 60 % of the travel out | `KICK_JAM`, all off | Phase `FAULT`, fault `KICK_JAM` | After a non-power-on reset (travel unknown) it is still taken as stowed | VERIFIED: code + test |
| M10b | Kick, /auto | Dart on its back or side: the kick swings through air | Kick stalls with no contact phase at 95 % or more of its swing in air, not planted since the last release (also on a retry or a part swing retried) | `NO_GROUND`, all off; foot, pods and drills not driven | Phase `FAULT`, fault `NO_GROUND`; `/release` stows the kick | Ground beyond 95 % of the air swing is also called `NO_GROUND`; a kick driven out by hand in air until it stalls is taken as planted | VERIFIED: code + test (round 10) |
| M12 | Kick, /release | Jams half way back but only slips (free current) | No stall. Kick timer 15 s | `KICK_TIMEOUT`, all off | Phase `FAULT`. Retry `/release` or drive `@` by hand | None worth noting | VERIFIED: code + scratch |
| M13 | Deploy, /auto | Jammed at the start (pods cannot leave the stow stop) | Worm at a stop from the start: 3 of 4 readings over the line, about 1.6 to 2.2 s | Taken as pods down: `pods down, drills finishing`, `anchored` | Phase `ANCHORED` with the pods up | As M5 | VERIFIED: code + sim (`deploy_jam=1`: anchored, deploy at 0) |
| M14 | Deploy, /release | Jammed down | Same rule | Taken as pods up, sequence ends `released` | `released` with the pods down | As M6 | VERIFIED: code + sim (`deploy_jam=1` from pods down: released, deploy at 1400) |
| M15 | Drills, /auto | Jammed (stone) | While pods swing: pair check, 2 readings (about 1.1 s). Drills finishing: lone stall over 570 mA pair line (600 at 8 V) for 0.2 s after 0.25 s | Drills off, event `drills stalled (stone?), pods carry on` or `drills stalled (full depth or stone)`; ends `anchored` | Phase `ANCHORED`; the event is the only clue the drills are shallow | Weak anchor reported as good | VERIFIED: code + test_logic + sim (`drill_jam=1`: anchored) |
| M16 | Drills, /release | Jammed | Lone stall 0.45 s (drills out step) or 2 pair readings (pods step) | `DRILL_JAM`, all off | Phase `FAULT`. See O5 for retries | Dart stuck in the ground | VERIFIED: code + test_logic + sim test_05 |
| M17 | Foot, /auto | Jammed (stone) | Pair check while the kick plants, lone stall at foot finishing | Foot off, event `foot stalled (stone?)...`, ends `anchored` | Phase `ANCHORED`; event is the only clue | Weak anchor reported as good | VERIFIED: code + test_logic + sim (`foot_jam=1`: anchored) |
| M18 | Foot, /release | Jammed | Lone stall (foot out step) or pair check (kick step) | `FOOT_JAM`, all off | Phase `FAULT` | Dart stuck | VERIFIED: code + test_logic |
| M19 | Any motor, manual | Driven into its end stop | Lone: over its stall line for 0.2 s after 0.25 s start-up. Two at once: pair check. 8 s hold cap in any case | Motor off, locked until the key is released | 409 `stalled: release the key first`. Release the key | Below about speed 70 only the 8 s cap stops a stall | VERIFIED: code + test_logic |
| M20 | Kick | Wired backwards | /auto: kick drives into the stow stop, stall at once with travel near 0 | `KICK_JAM`, all off (if travel is trusted) | Phase `FAULT`, fault `KICK_JAM` on a fresh shot | BENCH_TEST step 5 catches it before any shot | VERIFIED: code + test |
| M21 | Deploy | Wired backwards | Pods drive into the stow stop, stall at once, 3 of 4 rule | `pods down`, `anchored`, pods up | `ANCHORED` | As M5 | VERIFIED: code + scratch |
| M22 | Drills | Both wired backwards (D8/D9 swapped) | Unscrewing reads about 200 mA, under the 570 mA stall line | Nothing | `ANCHORED`, augers never went in | As M5. BENCH_TEST step 4 catches it | VERIFIED: code + scratch |
| M23 | Drills | Right drill not swapped (both turn the same way) | Cannot be seen in one shared current | Nothing | `ANCHORED`; one auger backs out, dart twists | Weak anchor. Only the BENCH_TEST step 4 look catches it | ASSUMED (no code path exists to see it) |
| M24 | Foot | Wired backwards | Unscrewing current is under the stall line | Nothing | `ANCHORED`, foot backs out | Nose lifts. BENCH_TEST step 6 catches it | VERIFIED: code + scratch |
| M25 | Drills | One of the two drill motors dead (open lead, broken gearbox) | Free pair current drops from about 70 to 35 mA, still over the 27 mA no-current line. In soil one auger digs normally | Nothing | `ANCHORED` with one drill | Anchor holds about half; may pull out | VERIFIED: code + scratch (pair at half current: manual and /auto both normal). See GAPS |
| M26 | Drills | One drill stalled, the other still turning | Not separable from normal digging (API.md says so) | Nothing until both stall | As M15 | As M25 | VERIFIED: code |

## 2. DRV8833 drivers

| # | Component or event | Failure mode | Firmware detects (threshold, time) | Firmware does | Jetson / Harry sees and does | Residual risk | Check |
|---|---|---|---|---|---|---|---|
| D1 | DRV#1 or DRV#2 | SLP pad not connected | Pololu carrier: SLP has its own pull-up, works normally. Generic board without pull-up: chip asleep, both its motors dead. Firmware sees each motor as an open wire: DRV#1 (deploy, kick) at the kick swing in 0.75 s; DRV#2 (drills, foot) at foot finishing | `NO_CURRENT`, all off (manual: `draws no current`) | Phase `FAULT` early in the first `/auto`. Check wire 23 / 24 | None if the first test is on the bench | ASSUMED (board pull-ups); firmware side VERIFIED by the open-wire runs |
| D2 | DRV#1 or DRV#2 | SLP pad is tied to VM on the board and 3V3 is soldered to it | Nothing: battery voltage (up to 8.4 V) goes into the XIAO 3V3 rail | Nothing (XIAO, INA219, BNO085 likely destroyed) | Dead dart. WIRING.md step B6 (ohm check, under 50 ohm = do not connect) prevents it | Destroyed parts | ASSUMED |
| D3 | DRV8833 | Over-temperature shutdown (both channels of that chip off until it cools) | No nFAULT wire, so only through the current: lone motor reads nothing (`NO_CURRENT`); in a pair step the worm shows no stall and ends on its time limit | As M1 to M5 | Phase `FAULT` `NO_CURRENT`, or a sequence that "finishes" with motors that stopped | A deploy or kick that silently stopped mid-step | ASSUMED (driver behaviour). See GAPS |
| D4 | DRV8833 | Over-current protection on a channel | As M8 | As M8 | As M8 | As M8 | ASSUMED |
| D5 | DRV#2 | VM or GND wire to DRV#2 broken (wire 12 or 15) | DRV#1 works, rail reading is fine. Drills and foot read as open wires (M3, M4) | `NO_CURRENT` at foot finishing | Phase `FAULT` | None worth noting | ASSUMED (premise); firmware side VERIFIED by the open-wire runs |

## 3. INA219 current sensor

| # | Component or event | Failure mode | Firmware detects (threshold, time) | Firmware does | Jetson / Harry sees and does | Residual risk | Check |
|---|---|---|---|---|---|---|---|
| I1 | INA219 | Absent at boot | Setup writes fail | `ina_ok` false, `rail_V` -1. `/auto`, `/release`, `/arm` with auto refused (`no current sensor`); log-only arm allowed. Manual keys work with a 2 s hold cap and the deadman. No stall, trip, low-rail or open-wire checks. Retried at 10 Hz: when it answers it is set up again, event `INA219 back` | Web page `INA219 MISSING`. Check wires 25, 27, 29, 31 | Manual motors run blind for up to 2 s | VERIFIED: code + sim (`--no-ina`, then `ina_dead=0`: back and calibrated; held key stopped after 2 s) |
| I2 | INA219 | Stops answering during a manual run | 3 failed reads in a row (15 ms) | Event `INA219 lost: manual keys only, timers still apply`; hold cap drops to 2 s | `ina_ok` false, `rail_V` -1 | As I1 | VERIFIED: code + sim (`ina_dead=1` mid hold) |
| I3 | INA219 | Stops answering during a sequence | 3 failed reads (15 ms) | `CURRENT_SENSOR`, all off | Phase `FAULT`. `/auto` and `/release` refused until it is back | Sequence stopped half way | VERIFIED: code + test_logic |
| I4 | INA219 | Loses its calibration (a 3V3 dip resets it: reads 0 mA, rail still right) | Calibration register read at 10 Hz | Re-initialised within 100 ms, event `INA219 lost its setup: re-initialised`. The 0 mA readings before that are too short to trip the 0.5 s no-current rule | Nothing to do | If it kept reading 0 mA, a lone motor would get `NO_CURRENT` | VERIFIED: code + sim (`ina_brownout=1` in kick swing: cal back, anchored) + scratch (stuck at 0 mA: `NO_CURRENT`) |
| I5 | INA219 | Occasional I2C errors | 1 or 2 failed reads are tolerated. A tick with no reading runs the timers only | Nothing visible; stall still caught with 1 read in 3 failing | Nothing | None worth noting | VERIFIED: code + test_logic |
| I6 | INA219 | Reading stuck at a wrong value | Not directly. Stuck low (about 50 mA): kick never sees contact, kick timer 15.5 s. Stuck over 2000 mA: trip | `KICK_TIMEOUT` or `OVERCURRENT` | Phase `FAULT` | Stuck values in between could fake a contact or a stall | VERIFIED: code + scratch (stuck 50 mA: KICK_TIMEOUT); premise ASSUMED |
| I7 | INA219 | Range overflow (over 3.2 A through the shunt) | Overflow bit read as 9999 mA; over 2000 mA for 10 ms (3 readings) | `OVERCURRENT`, all off | Phase `FAULT` | One or two overflow readings are ignored | VERIFIED: code + scratch |
| I8 | Motor rail | Shunt cracked or wire 9 / 11 open: motors have no power | Rail reads about 0 V while a motor runs | `LOW_RAIL` after 0.5 s, all off | `rail_V` near 0, phase `FAULT` | none | VERIFIED: code + test |

## 4. BNO085 IMU

| # | Component or event | Failure mode | Firmware detects (threshold, time) | Firmware does | Jetson / Harry sees and does | Residual risk | Check |
|---|---|---|---|---|---|---|---|
| B1 | BNO085 | Absent at boot | `begin` at 0x4A and 0x4B fails | `imu_ok` false; `/arm` refused (`no IMU`); manual, `/auto`, `/release` work; no flight log. Never retried until a reboot | Web page `BNO085 MISSING`. Check wires 26, 28, 30, 32, reboot | Fixing a loose wire needs a power cycle | VERIFIED: code + sim (`--no-bno`) |
| B2 | BNO085 | Goes silent, not armed | No report for 1 s | `imu_ok` false, event `BNO085 silent for 1 s`; reports re-enabled every 2 s; `BNO085 data back` when they return | `imu_ok` false | None while idle | VERIFIED: code + sim (`imu_dead=1`) |
| B3 | BNO085 | Goes silent while armed, before the shot | `imu_ok` false after 1 s, but the arm stays | Stays `ARMED`; a launch cannot be seen, so no auto on landing | Phase `ARMED` and `imu_ok` false. Jetson must not fire, or must plan to send `/auto` itself | Shot with no anchoring and no log | VERIFIED: code + sim (`imu_dead=1` then a launch trace: still ARMED). See GAPS |
| B4 | BNO085 | Goes silent in flight | Flight timeout 15 s after launch | `LANDED` (`timeout`), auto not started, event `no impact seen: auto not started, send /auto` | Phase `LANDED`. Jetson checks video, sends `/auto` | 15 s wait | VERIFIED: code + test_logic |
| B5 | BNO085 | Resets itself | `wasReset()` on the next tick | Reports re-enabled at once; flight detection carries on; yaw reference starts again | Small gap in the log, yaw jumps | A reset during the impact could miss it: then `still` fallback, no auto | VERIFIED: code + sim (`bno_reset=1` armed and mid flight: landed and anchoring); impact case ASSUMED |
| B6 | BNO085 | Saturates near 8 g and low-pass filters short spikes | Launch line 4 g, impact line 3.5 g (both under the clip) | Push and impact read flat at about 8 g; still detected. A push shorter than about 15 to 24 ms over 4 g may be missed (F1) | `peak_g` shows the real level after a shot | See F1 | VERIFIED: code + test_logic (5 g impact, 15 / 20 ms pushes) |
| B7 | BNO085 | Bad sample (NaN, infinity) | Checked on every report | Sample dropped | Nothing | None | VERIFIED: code + test_logic |

## 5. I2C bus

| # | Component or event | Failure mode | Firmware detects (threshold, time) | Firmware does | Jetson / Harry sees and does | Residual risk | Check |
|---|---|---|---|---|---|---|---|
| X1 | I2C bus (D4 / D5) | Stuck (a device holds SDA low after a glitch, or a wire shorts to GND) | Every transfer fails, each after up to the 20 ms timeout. INA219 lost after 3 ticks, BNO085 silent after 1 s | As I2 / I3 and B2 to B4; after 200 failed reads (1 s) with the IMU silent: 9 SCL clocks + Wire restart, at most every 3 s, event `I2C bus stuck: recovered` | `ina_ok` and `imu_ok` false until it recovers | A wire shorted to GND cannot be recovered: pull the plug | VERIFIED: code |

## 6. Power

| # | Component or event | Failure mode | Firmware detects (threshold, time) | Firmware does | Jetson / Harry sees and does | Residual risk | Check |
|---|---|---|---|---|---|---|---|
| P1 | Battery | Low while motors run | Rail under 5.6 V for 0.5 s while a motor runs (needs a good INA219 reading) | `LOW_RAIL`, all off | Phase `FAULT`. Charge the cells | None worth noting | VERIFIED: code + test_logic + sim (`rail_v=5.0`) |
| P2 | Battery | Low at idle, or when arming | `/arm` with auto: rail under 6.6 V at rest | Refused, `battery low`; a log-only arm is still allowed | Jetson sees the 409 | none | VERIFIED: code + test |
| P3 | Cells | Imbalance (one cell much lower) | Nothing: only the pack voltage is measured | Nothing | Nothing until the BMS cuts off (P5) | Sudden total power loss with a pack voltage that looked fine | ASSUMED. See GAPS |
| P4 | BMS | Over-current cut-off (short before the shunt, for example at the buck input or a pinched main wire) | The 2000 mA trip only sees motor current after the shunt | Nothing (everything loses power) | WiFi `KEDGE` and video vanish | Dead dart until reset | ASSUMED |
| P5 | BMS | Under-voltage cut-off, or any total power loss | Nothing before it happens | Everything off; motors coast, worms hold position. When power returns: reset `poweron`, phase `IDLE`, no fault | AP and video gone. After power returns, a small `t_ms` shows the reboot | Lost state (see C5) | ASSUMED (power); boot result VERIFIED: code + sim |
| P6 | Power loop plug | Bounces at launch | ASSUMED: XIAO resets (brownout or power-on). Firmware only knows the reset reason | Boots with motors off; arm, flight state and log are lost; phase `FAULT` `BROWNOUT` (or `IDLE` after `poweron`) | After the shot: `reset` is `brownout` or `poweron` with a small `t_ms`. Treat as landed: send `/auto` once video shows the dart down (`/stop` first is harmless) | No auto, no flight log | VERIFIED: code + sim (`--reset brownout` / boot) ; plug behaviour ASSUMED. See GAPS |
| P7 | Power loop plug | Bounces at impact or while anchoring | Same as P6 | Sequence lost where it was; legs hold (worms). A retried `/auto` with the kick down skips planting (`kick already down, foot planting`) | Reboot signature, then `/auto` | Drills and pods redo their steps | VERIFIED: code + test_logic (auto retried with the kick down); plug behaviour ASSUMED |
| P8 | Power loop plug | Short drop the XIAO rides through (with the 470 uF capacitor) but the motor rail dips | Rail under 5.6 V only counts after 0.5 s; current readings dip | Usually nothing. A dip during a pair reading can delay or, rarely, fake a stall reading | Nothing | Rare wrong step end | ASSUMED |
| P9 | MP1584 buck | Output dies | Nothing (XIAO, INA219, BNO085 lose power; DRV SLP comes from 3V3, so both drivers sleep) | Everything stops, motors coast | AP and video gone, battery still charged | Dead dart | ASSUMED |
| P10 | MP1584 buck | Output too high (adjust screw moved, or the regulator fails shorted) | Nothing | XIAO, VTX and camera may be destroyed (they are 5 V parts) | Dead dart. Set 5.04 V before wiring (WIRING.md B2); dab of nail varnish on the screw | Destroyed parts | ASSUMED |
| P11 | MP1584 buck | In dropout on a nearly flat pack, 5 V sags | ESP32 brownout detector | Reset, `BROWNOUT` fault on boot | Phase `FAULT` `BROWNOUT`. `LOW_RAIL` normally trips first while motors run | Reboot during a sequence (P7) | ASSUMED |
| P12 | Schottky diode | Fitted backwards or open | Nothing | XIAO runs only from USB | Dart dead on battery | Found at the power-on check | ASSUMED |

## 7. XIAO ESP32S3

| # | Component or event | Failure mode | Firmware detects (threshold, time) | Firmware does | Jetson / Harry sees and does | Residual risk | Check |
|---|---|---|---|---|---|---|---|
| C1 | XIAO | Brownout reset | Reset reason `ESP_RST_BROWNOUT` at boot | Motor pins driven low first, then `BROWNOUT` latched, event `boot after BROWNOUT reset, motors off` | Phase `FAULT`, `reset` `brownout`. `/stop` (or `/auto`, `/release`, `/arm`) clears it | See C5 | VERIFIED: code + sim (`--reset brownout`) + test_logic |
| C2 | XIAO | Crash (panic) | Reset reason panic | `WATCHDOG` latched (a crash and a watchdog share this code) | Phase `FAULT`, `reset` `panic` tells them apart | See C5 | VERIFIED: code + sim (`--reset panic`) |
| C3 | XIAO | Safety task stuck (no tick for 2 s) | Task watchdog, 2 s, panics | Reboot; motors coast during the reboot (DRV8833 inputs pull down); D6 / D7 twitch, so the foot may twitch | Phase `FAULT` `WATCHDOG`, `reset` `task_wdt` | Up to 2 s of uncontrolled motors before the reset | VERIFIED: code + sim (`--reset task_wdt` boot); real panic ASSUMED |
| C4 | XIAO | Web / Serial loop hangs (loop task stuck) | Safety task watches a loop heartbeat | After 30 s, if nothing moves, not armed or in flight and no OTA: restart (reset `software`); meanwhile the deadman stops manual motors and sequences finish | No HTTP answers for up to 30 s | A hang while armed or in flight waits for the landing | VERIFIED: code |
| C5 | XIAO | Any reboot (C1 to C3, P5 to P7) | Reset reason only | Arm, sequence step, flight state and flight log are lost (RAM only) | `t_ms` restarts at 0; check `reset` after every shot | Jetson that does not watch `reset` thinks the dart is still `ARMED` or anchoring | VERIFIED: code. See GAPS |

## 8. WiFi and web server

| # | Component or event | Failure mode | Firmware detects (threshold, time) | Firmware does | Jetson / Harry sees and does | Residual risk | Check |
|---|---|---|---|---|---|---|---|
| W1 | WiFi client | Leaves mid hold | Station-left event (may come late on real hardware); the key deadman (350 ms, max 500 ms) does not wait for it | Manual motors stop at once on the event, or at the deadman | Event `WiFi client lost, motors stopped` | Any station leaving stops another client's hold too (harmless) | VERIFIED: code + sim (`/sim/disconnect`) + test_logic; event timing ASSUMED |
| W2 | WiFi client | Leaves mid sequence | Same event | Sequence carries on to `anchored`, `released` or a fault | Reconnect, read `/status` | None | VERIFIED: code + sim + test_logic |
| W3 | WiFi client | Leaves mid flight | Same event | Nothing changes; landing still starts `/auto` | Reconnect, read `/status` | None | VERIFIED: code + sim |
| W4 | Web server | A client opens a connection and sends nothing (half-open) | Nothing | ESP32 web server waits up to 5 s for that request; other requests (including `/stop`) wait. Deadman and safety run in their own task and stop motors on time | `/stop` answered late. Use 1 s timeouts and retry `/stop` | Late `/stop` for up to 5 s (a sequence keeps running for that time) | VERIFIED: code + sim (stub waits 2 s: `/stop` answered after 2.0 s, held kick still stopped at its 500 ms deadman) |
| W5 | Web server | `/flightlog` on a weak link blocks the server | Refused while any motor runs, armed or in flight | 409 `busy (...)` | Fetch it after landing | Server slow while idle only | VERIFIED: code |

## 9. Jetson and operator mistakes

| # | Component or event | Failure mode | Firmware detects (threshold, time) | Firmware does | Jetson / Harry sees and does | Residual risk | Check |
|---|---|---|---|---|---|---|---|
| J1 | Jetson | Conflicting commands: `/release` while anchoring, `/auto` while releasing, run keys or `/arm` during a sequence | State check | 409 `busy`; nothing changes | Treat 409 as an answer; send `/stop` first if really meant | None | VERIFIED: code + test_logic |
| J2 | Jetson | Repeats `/auto` or `/release` after a timeout | Same sequence already running | Accepted, not restarted | Safe to retry | None | VERIFIED: code + test_logic |
| J3 | Two clients | One sends `/stop` while the other keeps repeating a run key | The stop locks every key pressed in the last 600 ms (running, in a deadman gap, or refused during a sequence) | Motor stays stopped; refused (`stalled: release the key first`) until that key is released | Both see the motor stop | none | VERIFIED: code + test |
| J4 | Jetson | Sends `/auto` while `ARMED` (dart on the launcher) | Not refused (only refused in `FLIGHT`) | Disarms and starts the kick swing on the launcher | Legs swing out on the launcher | Fouled launcher, damaged legs | VERIFIED: code + sim. See GAPS |
| J5 | Jetson | Sends `/arm?on=0` or `/stop` in flight | Accepted | `/arm?on=0`: no auto on landing. `/stop`: flight ends (`landing` `stop`) | Must send `/auto` after landing | Missed auto | VERIFIED: code + test_logic |
| J6 | Two clients | Both use the `A` toggle | Toggle | Second toggle undoes the first | Use `/arm?on=1` (idempotent) | Unarmed shot | VERIFIED: code |
| O1 | Harry | Presses keys during flight | `FLIGHT` phase | 409 `in flight`; the page's key-up stop keeps the arm (`disarm=0`), so auto on landing still runs | Nothing to do | None | VERIFIED: code + test_logic |
| O2 | Harry | Holds both directions of one motor | Opposite key while the motor runs the other way | Motor stops; both keys refused (`both directions held`) until neither is pressed for a window | Release both, press one | None | VERIFIED: code + test_logic |
| O3 | Harry | Arms on the bench, then handles the dart | Launch needs over 4 g for about 20 ms. Normal knocks never count | A hard knock: `FLIGHT` for about 2.4 s, then `LANDED` (`still`), disarmed, no auto. Keys refused while in `FLIGHT` | `/stop` ends it at once. Re-arm before the shot | None worth noting | VERIFIED: code + test_logic + scratch |
| O4 | Harry | Drops an armed dart | Free fall (under 0.5 g for 40 ms) before the hit | Not taken as a launch: no auto | Status stays `ARMED` | none; still disarm before carrying | VERIFIED: code + test |
| O5 | Harry | Retries `/release` after `DRILL_JAM` or `FOOT_JAM` | Each `/release` clears the fault and starts again at step 1 (drills out 3 s) | Still jammed: same fault again after about 0.45 s each try | Phase `FAULT` again. After two tries: dig it out or work it free with `#` and `$` by hand | Short stalls only; motors are not hurt | VERIFIED: code + scratch |
| O6 | Harry | Forgets the dart is armed | Arm age | Disarms after 15 min without a launch, event `arm expired` | Status `IDLE` | none | VERIFIED: code + test |

## 10. Flight events

| # | Component or event | Failure mode | Firmware detects (threshold, time) | Firmware does | Jetson / Harry sees and does | Residual risk | Check |
|---|---|---|---|---|---|---|---|
| F1 | Launch | Not detected (push too short or soft after the sensor filter, not armed, IMU silent) | Launch: over 4 g, samples spanning at least 15 ms; pushes of 24 ms or more always seen | Stays `ARMED` (or `IDLE`); no auto on landing | After the shot: still `ARMED`, `t_launch_ms` null. Jetson waits for the video, then `/auto` (accepted from `ARMED`) | Jetson has to act | VERIFIED: code + test_logic + sim (IMU dead) |
| F2 | Landing | Not detected | Soft landing: still (0.85 to 1.15 g) for 2 s after free fall: `still`. Nothing at all: 15 s after launch: `timeout` | `LANDED`, auto **not** started, event `no impact seen: auto not started, send /auto` | Jetson checks video, sends `/auto`. In `FLIGHT` `/auto` is refused; `/stop` ends `FLIGHT` first | Up to 15 s delay | VERIFIED: code + test_logic |
| F3 | Launch | False launch while armed on the launcher (knock over 4 g for about 25 ms) | Taken as a launch | About 2.4 s later `LANDED` (`still`), disarmed. The real shot is then not tracked and auto does not run | Phase `LANDED` before the shot. Jetson must check phase `ARMED` right before firing | Unanchored shot | VERIFIED: code + scratch. See GAPS |
| F4 | Landing | Dart snags on the tether or in a bush, hits, then hangs still | Impact over 3.5 g, then 1 s still | Starts `/auto` in mid air | Jetson sees `ANCHORING`; send `/stop` if the video shows it hanging | Legs swing in the air | ASSUMED (needs a real trace) |
| F5 | Landing | Lands upside down or on its side | No attitude check before auto | Auto runs anyway | Jetson decides from video and `roll` / `pitch` | Wasted anchor attempt | VERIFIED: code |
| F6 | Flight | Reboot in flight (P6) | Reset reason | Phase `FAULT` `BROWNOUT` / `WATCHDOG` or `IDLE`; no arm, no log | Small `t_ms` after the shot: treat as landed, then `/auto` | No auto, no log | VERIFIED: code + sim (boots) |

## GAPS

Failure modes where the firmware does nothing useful, or the docs and the code disagree. Each with a suggested fix.

1. **The deploy motor is never checked for an open wire, short or wrong direction** (M5, M6, M8, M13, M14, M21). It
   always runs with the drills, so the no-current rule never sees it, and a jammed or backwards deploy is taken as
   "pods down" or "pods up". The anchor ends `anchored` with the pods up; the release ends `released` with the pods
   down. Fix: start the pods step with 300 ms of deploy alone (no-current check, and "stalled from the very first
   reading" = fault `DEPLOY_JAM` unless this is a known retry). Report the step result in `/status`.
2. **A jammed or backwards kick is taken as "kick already down"** (M10, M20), and a kick that jams on the way back is
   taken as stowed (M11). Fix: keep a running total of kick travel time (out minus back) in RAM; believe "already down"
   only when the total says the kick is out, and believe "stowed" only after the back travel reaches about 80 % of the
   out travel. Otherwise fault `KICK_JAM`.
3. **`ANCHORED` does not say how good the anchor is** (M15, M17, M5). Drills or foot stopped by a stone, kick plant
   time limit and pods time limit all still end `anchored`; only `event` (one line, overwritten) shows it. Fix: add a
   `warnings` list to `/status` (for example `drills_stalled_early`, `pods_time_limit`, `kick_plant_limit`) that stays
   until the next `/auto` or `/arm`.
4. **One dead drill motor is invisible** (M25), and a backwards drill or foot is too (M22 to M24). Fix: a pre-shot
   self-test key or `/selftest`: each motor alone for 0.5 s each way in the air, compare with `freeMa` (drill pair under
   about 50 mA = one drill dead). Run it before every arm. Directions still need the BENCH_TEST look.
5. **STOP from one client does not stick against another client's held key** (J3). Fix: after `/stop` or key space,
   lock every run key for all clients until no run key has arrived for the hold window (as `stallLock`, for every
   motor), or until 1 s has passed.
6. **`/auto` is accepted while `ARMED`** (J4): a Jetson bug swings the legs out on the launcher. Fix: refuse `/auto`
   in `ARMED` with 409 `armed: disarm first`, or require `?force=1`.
7. **Dropping an armed dart can start the auto sequence** (O4), and **a false launch disarms before the real shot**
   (F3). Fix: only accept a launch after at least 300 ms near 1 g (0.85 to 1.15 g, sitting on the launcher) right
   before the push; a free fall before the push is not a launch. After a `still` landing that never saw free fall,
   go back to armed instead of disarming. Add an arm time limit (for example 120 s) so a forgotten arm expires (O6).
8. **Arming is allowed with a flat battery** (P2), and there is no warning at idle. Fix: refuse `/arm` and `/auto`
   when the resting rail is under about 7.0 V (2 x 3.5 V), and add `battery_low` to `/status`.
9. **The BNO085 can go silent while armed and the arm stays** (B3). Fix: when `imu_ok` goes false while armed, log
   `BNO085 silent while armed: launch will not be seen` (the phase can stay `ARMED`), and make the Jetson client
   refuse to fire while `imu_ok` is false. Also retry `begin_I2C` every few seconds
   while idle when the BNO085 was missing at boot (B1).
10. **No driver fault input** (D3, D4): over-temperature and over-current show only as missing current. Fix: wire the
    nFAULT pads of both DRV8833 boards together (open drain) to D10 with the internal pull-up, read it every tick, new
    fault `DRIVER`.
11. **No cell balance or cell voltage sense** (P3): the BMS can cut power with no warning. Fix: a divider (2 x 100 k)
    from BMS BM to a spare ADC pin. D10 is the only free pin, so if D10 goes to nFAULT (gap 10), add an I2C ADC
    (ADS1115 at 0x48) on the existing bus instead.
12. **No I2C bus recovery** (X1). Fix: after about 20 failed transfers in a row while no sequence runs, clock SCL 9
    times, send a STOP, `Wire.end()` and `Wire.begin()` again, then re-run the INA219 and BNO085 setup.
13. **Every reboot loses arm, step and flight state** (C5, P6, P7, F6). Fix: keep phase, armed, auto flag, `seq` and
    `t_launch_ms` in `RTC_NOINIT_ATTR` memory, which survives watchdog, panic and most brownout resets; report
    `resumed_from` in `/status`. And make the 470 to 1000 uF capacitor on the XIAO 5 V pin required, not "optional"
    (WIRING.md), and tape the power loop plug.
14. **Nothing watches the web loop** (C4). Fix: the safety task checks a counter that `loop()` bumps; if it has not
    moved for 10 s and no sequence runs, stop the motors and `ESP.restart()`.
15. **A dead motor rail (0 V) is not called a rail fault** (I8): the low-rail rule ignores readings under 0.5 V. Fix:
    with a good INA219 read, rail under 0.5 V while a motor runs = `LOW_RAIL` (or a new `NO_RAIL`).
16. **Docs disagree with the code:**
    - API.md fault table, `SEQ_TIMEOUT`: "or a sequence motor timer ran out while current reads were failing". The
      code does not do that: a step timer that ends on a failed read ends the step normally (API.md itself says so
      under `/auto`). Only the 35 s whole-sequence limit gives `SEQ_TIMEOUT`. Fix the table.
    - API.md fault table, `NO_CURRENT` "a lone motor in a sequence": true, but it should say the deploy motor is never
      a lone motor, so it is never covered (gap 1).
    - API.md `/key`: "A launch detected while a manual motor runs stops it" is only true while armed (a launch is only
      looked for while armed). Add "while armed".
    - `kedge/CLAUDE.md` sections 4 and 6 still describe v3: three DRV8833 boards (DRV#3 VM, SLP x3), DRV#2 inputs
      bridged, trip 2500 mA, helper paused every 400 ms. The code and WIRING.md are v4: two boards, trip 2000 mA,
      pause 150 ms every 550 ms. Point those sections at WIRING.md and API.md.
    - WIRING.md calls the 470 uF XIAO capacitor "optional"; API.md relies on the Jetson spotting reboots after impact
      because there is no other power path. See gap 13.


## Status of the gaps (fixed on 3 Nov, afternoon)

| Gap | Status |
|---|---|
| 1. Deploy never checked | FIXED: the anchor's pods step starts with the deploy worm alone for 0.3 s (open wire faults `NO_CURRENT`). The "already at the stop" shortcut was removed after review round 8 (non-spinning augers in soil could fake it). A backwards deploy is still not detectable by current. |
| 2. Kick jam misread | FIXED: kick travel bookkeeping; `KICK_JAM` instead of "already down" or "stowed". |
| 3. `ANCHORED` hides a weak anchor | FIXED: `warnings` list in `/status`. |
| 4. One dead drill, backwards drills or foot | OPEN: not detectable from the shared current; BENCH_TEST checks directions. A pre-shot self-test is a possible later addition. |
| 5. STOP across clients | FIXED: a stop locks run keys that any client is still repeating until released. |
| 6. `/auto` accepted while `ARMED` | CHANGED after review round 8: `/auto` while armed disarms first and starts (a refusal left the Jetson stuck when the launch was missed). The Jetson only sends `/auto` after its own landing decision. |
| 7. Launch fooled by a drop; arm never expires | FIXED: a push within 400 ms after free fall is not a launch (the first version, 300 ms of stillness, rejected real shots on a vibrating rail and was replaced); arm expires after 15 min. |
| 8. Arming on a flat battery | FIXED: refused under 6.6 V at rest. |
| 9. IMU silent while armed; BNO085 not retried | PARTLY: reports are re-enabled every 2 s; the Jetson should not fire while `imu_ok` is false (asked). A full re-begin while idle is not done (it blocks the bus for up to a second). |
| 10. No DRV8833 nFAULT input | OPEN (wiring): optional wire from both nFAULT pads to D10. |
| 11. No cell-voltage sense | OPEN (hardware). |
| 12. No I2C bus recovery | FIXED. |
| 13. A reboot loses the arm and log | PARTLY: the Jetson rule in API.md, and the 470 uF hold-up capacitor (WIRING, recommended). |
| 14. Nothing watches the web loop | FIXED: restart after 30 s hung with nothing moving (not during OTA; 30 s, not 10 s, so a stalled /flightlog client cannot cause it). |
| 15. A dead 0 V rail not flagged | FIXED: counts as `LOW_RAIL`. |
| 16. Docs disagreements | FIXED in API.md (SEQ_TIMEOUT text, NO_CURRENT for deploy); a launch is only looked for while armed, so "a launch stops a manual motor" applies while armed only. kedge/CLAUDE.md section 4 is marked superseded by the coordinator. |
