# GrappStar system plan (agreed in discussion, 3 Nov 2026, not built yet)

This supersedes the overnight "ground station" design in `kedge/gs/` (3 Nov 2026, site
https://claude.ai/artifact/Uof3qNiqzPXed57HCM79jK). That design is kept for reference only. It went wrong in five ways:
it replaced Harry's self-wrapping capstan with a spincast reel, fixed the fire angle instead of the up/down aim,
had no system swap, specified parts Harry would have to buy, and was built as one thread.

Harry's own sources to build from: `originals/handover_launcher_v10.md`, `originals/reel-handover.md`,
`originals/grappstar_reel_full_v7*`, `originals/grappstar_desk_v10_handtest_stl.zip`, the HTML sims in `originals/`,
and every message and paste in `knowledge/HARRY_MESSAGES_AND_PASTES.md`.

## 1. Whole system

- Rover with a Jetson Orin Nano Super (WiFi, powered from the rover 3S LiPo through its barrel pigtail).
- Ground station on the rover: ONE 12 V wiper motor (self-locking worm) drives TWO systems, and the base swaps which
  system the motor is in:
  - System 1, launch: two directions. One direction oscillates the rail up and down to AIM. The other direction cocks
    the spring launcher, fires and resets.
  - System 2, reel: storage reel plus capstan. One direction reels in on the capstan. Payout is passive.
- Dart (KEDGE): flies on the tether, nose camera (Caddx Ant) -> 5.8 GHz VTX -> RC832 on the rover -> DIGITNOW capture ->
  Jetson, which analyses flight and landing zone. After landing the dart anchors itself (kick leg plants a foot auger
  ahead of the nose, two counter-rotating drill pods screw in), all on its own XIAO ESP32S3 and N20 motors.
- Then the capstan winches the rover against the anchored dart, then the dart releases and is reeled home.

## 2. Reel = storage reel + capstan

- Launch: the line BYPASSES the capstan and pays straight off the storage reel.
- Reel in: the line is wrapped round the capstan. The capstan carries the pull. Capstan friction grows with load, so
  the storage reel behind only needs light take-up tension.
- The capstan must wrap before reel-in and UNWRAP before the next launch.

## 3. Decisions so far

- Underactuation stays the concept, but one extra N20 (Harry has spares) is allowed. Get the most out of it.
- Extra N20 drives a SELECTOR DRUM (like a motorcycle shift drum), positions in a fixed line:
  - LAUNCH: dog clutch couples the wiper to the launch/aim train; storage reel brake set light for payout.
  - NEUTRAL: wiper coupled to nothing. The safe state on any fault.
  - REEL: dog clutch couples the wiper to the capstan; storage reel slip clutch set firmer for take-up.
  - Past REEL: continued N20 rotation orbits a WRAP ARM round the capstan to lay the wraps; reverse unwraps.
  - The order is physical: the capstan cannot be wrapped in launch mode and the mode cannot swap with wraps on.
  - The N20 worm self-locks: the drum holds any position unpowered.
- Capstan: Harry's multi-wrap capstan, wrapped and unwrapped by the drum's wrap arm. Backup idea: a TPU-lined wedge
  (V) groove sheave that grips without wraps and freewheels on the sprag during payout (needs a Dyneema grip check).
- Payout: storage reel spins free on 608 bearings and pays against a printed friction SLIP CLUTCH that is both the brake
  (about 0.5 to 1 N at the line; the dart flight sim says above about 1 N costs range and 3 N kills it) and the take-up
  drive during reel-in. Optional printed centrifugal shoes (baitcaster style) against overrun. No motor pre-spin: the
  dart spins the light reel up (11 m/s on a 40 mm core is about 5,000 rpm, fine for 608s).
- Sprag (one-way) bearings: Harry has 2 x CSK20PP (20 x 47 x 14, 50 N m, 6,000 rpm, internal and external keyways).
  #1 = spring cock and fire. #2 = capstan drive. Aim and storage take-up are light: printed ratchets or friction.
  Shafts 20 mm with a 6 mm key; printed PPA-CF keyed shafts carry about 12 N m (about 8 MPa torsion).
- Aim must hold while cocking: printed self-locking lead screw or worm in the aim train, driven through a printed ratchet.
- Swap reliability rules: every train torque-limited (slip clutch or current trip); a mode sensor tells the firmware the
  real drum position; fault -> NEUTRAL; manual lever on the drum as a hand fallback; launch and reel on separate trains.

## 4. Electronics (no new boards)

- Base: ESP32-S3 DevKitC-1-N8 (have) runs everything real-time. BTS7960 (have) drives the wiper. AS5600 (have) on the wiper
  shaft. The N20 selector drum runs on DRV8833 #3, freed from the dart (see below).
- Jetson talks to the base over USB serial and to the dart over WiFi. Video stays analog 5.8 GHz.
- Dart driver change to free DRV#3: DRV#1 = deploy + kick (unchanged); DRV#2 channel A = BOTH drills (0.7 A stall total,
  fine on one 1.2 A channel, right motor leads swapped as before); DRV#2 channel B = foot. Small firmware pin-map change.
- Switches: no microswitches in hand. Use PRINTED contact switches (printed flex lever, M3 screw head touching an M3 nut,
  two wires into an ESP32 pin with the internal pull-up, debounced in firmware). Microswitches are the one upgrade worth
  buying later.
- Jetson power: already from the battery. Nothing to buy.

## 5. The only purchase: 608 bearings

- Search: "608ZZ bearing 8x22x7" (or "608-2RS", sealed). Buy 8 to 10, any decent ABEC-5 or better (skateboard bearings
  are 608s and fine). Specs: 8 mm bore, 22 mm OD, 7 mm wide, rated well over 10,000 rpm.
- Uses: storage reel (2), capstan or wrap arm (2), cam roller and spares.
- Axles: an M8 bolt or 8 mm steel rod through each; check what is in the hardware box first.
- Plus the N20 for the dart foot that was already on the list.

## 6. Still open

- Harry to confirm the one-page spec built from this plan before anything is built.
- Exact swap trigger events (drum is N20-driven, so the firmware commands it; events like "carriage hit the end stop" and
  "dart seated" are inputs, not mechanical throws).
- Grip check if the V-groove backup is ever used.
- Build plan once approved: parallel sessions for capstan + wrap arm, storage reel + slip clutch, selector drum + clutch,
  launch + aim (from Harry's v10), frame + HD sim, base firmware, dart firmware pin change, Jetson software. Every part
  through the exact-mesh collision audit, physics and independent reviews; one model, one site.
