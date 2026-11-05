# 1. What the system is

GrappStar is a rover that can pull itself out of trouble. A launcher on the rover fires a small dart (KEDGE) on a
Dyneema tether. The dart lands, screws itself into the soil, and the rover winches against it. Then the dart lets go
and is reeled home.

Three parts:

- **Rover.** Jetson Orin Nano Super on the rover's 3S LiPo. It runs the mission, the operator web page, and the
  vision (rover camera Razer Kiyo X, and the dart's video through an RC832 receiver and a DIGITNOW USB capture).
- **Base station** (on the rover). ONE 12 V wiper motor drives everything. A sliding dog collar on the main shaft
  couples the wiper to the LAUNCH train, to nothing (NEUTRAL), or to the REEL train. A small N20 motor turns a
  selector drum (like a motorcycle gear drum) through four zones in a fixed order: LAUNCH, NEUTRAL, WRAP, REEL.
  - LAUNCH, wiper forward: draws the spring carriage back (241 mm), fires at the end of the stroke, resets.
  - LAUNCH, wiper backward: tilts the rail between 6 and 47 degrees to aim. A self-locking worm holds the angle.
  - WRAP zone: the collar stays in NEUTRAL while the drum turns a wrap cup 7 times round the capstan.
  - REEL, wiper forward: the capstan (REEL gear 50T to capstan gear 22T, 2.27 times wiper speed) winds in and tows. A one-way bearing stops it ever
    turning out. Tow limit 50 N by default.
  - Take-up: every forward wiper turn also winds the storage reel through a slip clutch, so slack never builds up.
  - Payout at launch: the line skips the capstan and pulls off the free-spinning storage reel against a light
    payout brake (0.5 N; over 1 N costs range).
  - Range: a few metres with the two springs (6 to 47 degrees of pitch). Two springs only in v1 (decision D38).
  - Protection: current trips in the firmware and a printed torque limiter (about 5 N m) in the wiper coupler. Any
    fault: wiper off, drum to NEUTRAL (except when the wraps are on, the spring is drawn or the
    drum itself faulted: then the drum holds, decision D27). A hand lever on the
    drum is the fallback.
  - Brain: an ESP32-S3 DevKitC on USB from the Jetson, with an AS5600 angle sensor on the main shaft and printed
    contact switches (an M3 screw head touching an M3 nut).
- **Dart (KEDGE v30).** 42 mm x 152 mm, its own XIAO ESP32S3, 2S 18350 pack, WiFi access point. After landing a kick
  leg swings over the nose and plants a foot auger, then two drill pods swing down and screw in. A nose camera
  sends analog video at 5.8 GHz.

How they talk: the Jetson talks to the base over USB serial (JSON lines, `firmware/base/PROTOCOL.md`) and to the dart
over WiFi (HTTP at 192.168.4.1, `firmware/dart/API.md`). Video is analog 5.8 GHz into the USB capture.

A mission (the fixed sequence in BASE_SPEC section 10.1): pick a target on the rover camera view; the Jetson looks
up the pitch; the base aims, cocks and fires on your confirmation; the dart video is checked for the landing zone;
the base lowers the rail to 6 degrees, then wraps the capstan at once while the line is still slack (the drum parks at WRAPPED); the dart anchors; the
base tidies the slack, shifts into REEL and tows; the base stops, then the dart releases; the base reels in to about 1.5 m, unwraps, and
brings the dart home on the take-up until it seats; back to LAUNCH.

@ballistics

## Order of work

0. Get the repo onto the Mac (Terminal; if macOS asks to install the command line developer tools for `git`,
   say Install and run the command again): `cd ~ && GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/HarryHYu/GrappStar.git`
   (skips the 145 MB photo PDF). If `~/GrappStar` then has no `docs/build` folder, run `cd ~/GrappStar && git checkout
   base-v1`. `git pull` updates it later.
1. Measure the wiper motor shaft, mount and housing (section 14, A1 to A5 and A8), the springs (group B), and the
   parts in H1, H3, H4. Put the
   numbers in `base/spec/params.py` and re-run the base pipeline and the physics (section 14, "After you measure").
   The base parts that touch the motor depend on
   them. A6 (speed, current) needs the base electronics: do it at step 7. A7 is optional. They change no
   printed part.
2. Confirm the parts box (section 14, group C). The only purchase is 608 bearings (and an AS5600 magnet if none
   is in the box).
3. Print the dart fit coupon, the base fit coupon and the two base risk coupon plates, and test them (section 4, test stage 0). The dart
   plates (section 4) do not wait for the base coupons or the measurements: start them any time after the dart
   fit coupon passes.
4. Flash the dart and build it (sections 6 and 8, and the dart run sheet `kedge/pack_v30/KEDGE_v30_run_sheet.pdf`: print it separately).
5. Set up the Jetson on its own wall supply (section 9, steps 0 to 3 only; its step 4 comes at the end of step 7).
6. Flash the base ESP32 from the Mac (section 8.1). Battery out is enough: any wiring already on the board can stay.
7. Wire the base electronics on the bench, wiper motor clamped in a vice and the drum N20 loose (section 7.2,
   firmware/base/WIRING.md Parts A to D), then bench test them (section 11.1), then calibration parts (a) and (b) (section 10). Measure A6 here. At the end:
   battery out, Jetson still on its wall supply, plug the ESP32 into the Jetson and do Jetson section 9 step 4
   (it creates `/dev/grappstar-base`, which every later `base` command uses).
8. Print the base (section 4, after the coupons pass and the measurements are in) and assemble it with the motor
   and drum N20 (section 5).
9. Put the base on the rover and wire the rover (section 7.1), then do the first power-on of everything together
   (7.1, its part 4). Then finish the Jetson setup (section 9, steps 5 to 7: cameras, dart WiFi, rover camera calibration).
10. Calibrate (section 10). Commands go to the base from the Jetson from now on (see section 10).
11. Run the staged tests from stage 4 on (section 12; stages 0 to 3 were done in steps 3, 7 and 9).
