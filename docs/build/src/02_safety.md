# 2. Safety

Read this before any hands-on step.

## Electrical

- **Battery unplugged before you touch, add or move any wire.** On the dart that means the power loop plug out. The
  first XIAO died from wiring with the battery live.
- Order for every power-up: USB cables in first, battery last. Power-down: battery out first. The base ESP32 may stay on
  the Mac's USB while the battery goes in (firmware/base/WIRING.md Part D does this), but never plug the USB in
  while the battery is in. The dart: never USB and the power loop plug in at the same time.
- Check for shorts with the meter on beep before the first power-on of anything you wired: red to black must not beep.
- The 3S LiPo can deliver over 100 A into a short. Keep the 20 A and 10 A fuses in their leads. Never bypass them.
- LiPo care: charge on the iMax B6 in balance mode at 1C (5.2 A) or less, on a non-flammable surface (tile,
  concrete, a LiPo bag), never unattended and never overnight. Recharge when the pack reads 3.6 V per cell at rest
  (10.8 V); never run it below 3.3 V per cell under load (9.9 V).
- LiPo storage: if it will sit for more than a few days, use the B6 "storage" mode (3.8 V per cell), keep it in a
  LiPo bag or metal box, away from heat (never in a hot car). A puffed, dented or hot pack: stop, put it outside on
  concrete, then to battery recycling. Never puncture or short one.
- 18350 cells: never let both cells touch metal at the same time. Charge out of the dart in the XTAR MC2, or through
  the tail balance socket with the loop plug out: on the B6 choose Li-ion (LiIo) 2S balance, 1.0 A or less (not the
  LiPo 3S program). Check each cell's direction against the marks in the cell bay and meter the pack (7.2 to 8.4 V)
  before the loop plug goes in. Look the cells over after a hard landing; a dented cell is scrapped.
- Setting a buck on the live LiPo (dart step 3, base WIRING Part A): use clip leads or insulate every free lead end
  first, so nothing loose can touch.
- Never connect the DRV8833 motor supply (VM) to the LiPo directly: 10.8 V maximum. It runs from the 9.0 V buck.
- The dart VTX must have its antenna on before it is powered.

## Mechanical

- The wiper motor is strong (about 15 N m) and self-locking. Hands out of the gears, drum and rail while the battery
  is plugged in.
- **Stopping (the same everywhere):** 1. send `estop` (the operator page STOP button sends a plain `stop`: wiper off, drum stays). 2. If anything
  still moves, pull lead #2's XT60: both motors lose power, the Jetson stays up. Keep that plug within arm's reach
  whenever the base is powered; it is the physical kill switch. 3. Only then the main battery plug.
- **Cutting power freezes stored energy, it does not release it.** Both worms self-lock, so a drawn spring and a
  line under tension stay loaded with the power off. The dart has its own battery.
- **Before you walk up to the base or the dart** (to measure a range, fetch the dart, or touch anything): `estop`;
  lead #2 out; telemetry shows `cock` 0 and `H` (SW_HOME) closed; the line is slack (no tension); the dart reads
  IDLE on the operator page, or its loop plug is out. If the carriage is drawn and `decock` fails: (1) `clear` the fault and send `{"cmd":"mode","to":"LAUNCH"}` if the drum
  is not in LAUNCH, then `decock` again; (2) if it replies `SWITCH_DISAGREE` and you can SEE the carriage is drawn back,
  `{"cmd":"decock","force":true}` (it runs back by the travel the saved angle needs; it trusts you); (3) if the firmware cannot run the wiper at all, unload by hand:
  battery out (the worm holds the spring where it is), safety glasses on, spring guard on, stand to the side, and
  unhook ONE spring at a time from its anchor pin with a hook tool (a bent coat-hanger wire or a long screwdriver
  through the hook), never with fingers in the hook. Then the carriage is slack and safe. Afterwards, with the
  carriage home: `{"cmd":"cal","what":"cock_rest"}`, then `aim` again before any shot.
- Never use the hand lever while the carriage is drawn (unless `cock` is 0 AND `SW_HOME` is closed) or while the line is under tension: shifting under load can dry fire the launcher or drop the line. De-cock and slacken the line first.
- The launch springs store energy. Cocked carriage = loaded. Never put a hand in front of the rail or the carriage.
  To unload a cocked base without firing, see troubleshooting. The spring hooks have little margin at full draw
  (about 40 N each at full cock against the 38.4 N rating of the Century C-243, decision D30, until the
  springs are measured): during shots stand beside the base,
  not behind or in front of the springs, and fit the printed spring guard if the pack has one. After B2, if the
  force at full cock is over the rating, use the fallback hole set (MEASURE_LIST B4).
- **Line under tension:** the capstan never turns back (one-way bearing), so a towing line cannot be let off at the
  base. During any pull (hand test or tow) stand beside or behind the rover, never in line with the dart, glasses
  on. To slacken it, release the dart (`release`) or drive the rover toward the dart. Calibrate the current-to-
  tension number (MEASURE_LIST G9) before any tow: with uncalibrated values a simulated tow reached 61 N
  against a 40 N limit.
- Chock or brake the rover during every shot and every tow. Strap the LiPo down under a cover on the rover.
- **Picking up a landed dart:** send dart STOP, check it reads IDLE (an armed dart can start anchoring when moved),
  then pull its loop plug before you touch it.
- The dart is 279 g leaving the rail at about 7 m/s: about 7 J, like a hard-thrown cricket ball, with sharp augers.
  From the first spring test on: safety glasses on, nobody in front of the rail or within 2 m to either side, and
  the area in front clear for 30 m (four times the longest expected range, for a misfire at a wrong pitch).
- Before every shot: look along the rail, call "firing" out loud, keep hands off the rail and carriage.
- A cocked carriage stays loaded until it fires or is de-cocked (`decock`). Treat the base as LOADED unless
  telemetry shows `cock` 0 AND `H` (SW_HOME) closed in `sw`. Unknown (`cock` null) counts as loaded.
- **Never fire a dart, dummy or real, without the tether tied on.** The tether keeps it nose first; without it the
  dart can tumble or come down tail first.
- Augers and drills are sharp and turn with force. Hold the dart by the hull, never near a pod or the foot.
- Soldering: iron on its stand, fan on for fumes, 3 s at most per joint near PLA. Heat gun on low, no lighter near PLA.
