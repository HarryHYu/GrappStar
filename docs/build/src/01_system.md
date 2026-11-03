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
  - LAUNCH, wiper forward: draws the spring carriage back (245 mm), fires at the end of the stroke, resets.
  - LAUNCH, wiper backward: tilts the rail between 6 and 47 degrees to aim. A self-locking worm holds the angle.
  - WRAP zone: the collar stays in NEUTRAL while the drum turns a wrap cup 7 times round the capstan.
  - REEL, wiper forward: the capstan (REEL gear 50T to capstan gear 22T, 2.27 times wiper speed) winds in and tows. A one-way bearing stops it ever
    turning out. Tow limit 50 N by default.
  - Take-up: every forward wiper turn also winds the storage reel through a slip clutch, so slack never builds up.
  - Payout at launch: the line skips the capstan and pulls off the free-spinning storage reel against a light
    payout brake (0.5 N; over 1 N costs range).
  - Range: about 3 to 5 m with the two springs (6 to 47 degrees of pitch; minimum about 2.9 m), about 8 to 9 m
    with a second pair.
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

A mission: pick a target on the rover camera view, the Jetson looks up the pitch in the ballistics table, the base
aims, cocks and fires on your confirmation, the dart video is analysed for the landing zone, the dart anchors, the
base wraps the capstan right after landing (while the line is still slack, decision D22), the
dart anchors, the base tidies the slack, engages REEL and tows, the dart releases, the base reels it home onto
the cradle, unwraps and returns to LAUNCH.

## Order of work

0. Get the repo onto the Mac (Terminal): `cd ~ && GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/HarryHYu/GrappStar.git`
   (skips the 145 MB photo PDF). `cd ~/GrappStar && git pull` updates it later.
1. Measure the wiper motor shaft, mount and housing (section 14, A1 to A5 and A8), the springs (group B), and the
   parts in H1, H3, H4. Put the
   numbers in `base/spec/params.py` and re-run the base pipeline. The base parts that touch the motor depend on
   them. A6 and A7 (speed, current, torque) need the base electronics: do them at test stage 2. They change no
   printed part.
2. Confirm the parts box (section 14, group C). The only purchase is 608 bearings (and an AS5600 magnet if none
   is in the box).
3. Print the dart fit coupon and the base risk coupon plate, and test them (section 4, test stage 0). The dart
   plates (section 4) do not wait for the base coupons or the measurements: start them any time after the dart
   fit coupon passes.
4. Flash the dart and build it (sections 6 and 8, and the dart run sheet in the appendix).
5. Print the base (section 4) and assemble it (section 5).
6. Set up the Jetson on its own wall supply (section 9). Flash the base ESP32 from the Mac, with nothing else
   connected to it (section 8.1).
7. Wire the base and the rover (section 7) and do the first power-on sequence (7.1, its part 4). While the Jetson
   is still on its wall supply, leave the barrel plug out in step 7.
8. Calibrate (section 10) and bench test (section 11).
9. Run the staged tests, bench to field (section 12).
