# 1. What the system is

GrappStar is a rover that can pull itself out of trouble. A launcher on the rover fires a small dart (KEDGE) on a
Dyneema tether. The dart lands, screws itself into the soil, and the rover winches against it. Then the dart lets go
and is reeled home.

Three parts:

- **Rover.** Jetson Orin Nano Super on the rover's 3S LiPo. It runs the mission, the operator web page, and the
  vision (rover camera Razer Kiyo X, and the dart's video through an RC832 receiver and a DIGITNOW USB capture).
- **Base station** (on the rover). ONE 12 V wiper motor drives everything. A sliding dog collar on the main shaft
  couples the wiper to the LAUNCH train, to nothing (NEUTRAL), or to the REEL train. A small N20 motor turns a
  selector drum (like a motorcycle gear drum) that moves the collar and also lays and removes the capstan wraps.
  - LAUNCH, wiper forward: draws the spring carriage back, fires at the end of the stroke, resets.
  - LAUNCH, wiper backward: tilts the rail up and down to aim. A self-locking worm holds the angle.
  - REEL, wiper forward: the capstan winds in; the storage reel takes up the slack through a slip clutch.
  - Payout at launch: the line skips the capstan and pulls off the free-spinning storage reel against a light
    brake (0.5 to 1 N).
  - Any fault: wiper off, drum to NEUTRAL. There is a hand lever on the drum shaft end.
  - Brain: an ESP32-S3 DevKitC on USB from the Jetson, with an AS5600 angle sensor on the main shaft and printed
    contact switches (an M3 screw head touching an M3 nut).
- **Dart (KEDGE v30).** 42 mm x 152 mm, its own XIAO ESP32S3, 2S 18350 pack, WiFi access point. After landing a kick
  leg swings over the nose and plants a foot auger, then two drill pods swing down and screw in. A nose camera
  sends analog video at 5.8 GHz.

How they talk: the Jetson talks to the base over USB serial (JSON lines, `firmware/base/PROTOCOL.md`) and to the dart
over WiFi (HTTP at 192.168.4.1, `firmware/dart/API.md`). Video is analog 5.8 GHz into the USB capture.

A mission: pick a target on the rover camera view, the Jetson looks up the pitch in the ballistics table, the base
aims, cocks and fires on your confirmation, the dart video is analysed for the landing zone, the dart anchors, the
base wraps the capstan and tows, the dart releases, the base reels it home.

## Order of work

1. Measure the wiper motor and the springs (section 14, groups A and B). Put the numbers in `base/spec/params.py`
   and re-run the base pipeline. The base parts that touch the motor depend on them.
2. Confirm the parts box (section 14, group C). The only purchase is 608 bearings.
3. Print the dart fit coupon, then the dart plates (section 4). The dart does not depend on the measurements.
4. Flash the dart and build it (sections 6 and 8, and the dart run sheet in the appendix).
5. Print the base (section 4) and assemble it (section 5).
6. Wire the base and the rover (section 7). Flash the base ESP32 (section 8). Set up the Jetson (section 9).
7. Calibrate (section 10) and bench test (section 11).
8. Run the staged tests, bench to field (section 12).
