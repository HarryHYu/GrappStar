# GrappStar

Ground-Retaining Anchor, Projectile-Placed, Self-Tensioning Auger Rig.

A rover launches a dart-shaped anchor (KEDGE, codename Taproot) trailing a Dyneema braid tether. The dart films its landing zone from a nose camera, self-rights on landing, deploys three legs that screw-drill into the soil, and the rover winches itself along the taut line.

Backyard test rig and portfolio piece.

## Subsystems

| Dir | What | Status |
| --- | --- | --- |
| `reel/` | Winch stack on the rover | Current focus. Plan before printing more parts. |
| `dart/` | KEDGE anchor dart | Design done (v20 STL family). Built in parallel with the reel. |
| `launcher/` | Spring launcher | External design, frozen. Electrical interface only lives here. |
| `vision/` | Jetson vision and ballistics | Pipeline established. |
| `sim/` | Geometry solver and anchor physics | Active. Sim fixes feed STL revisions directly. |
| `cad/` `docs/` | STLs, drawings, notes | As they accumulate. |

## Conventions

- Terse and technically precise.
- No em dashes anywhere: code comments, docs, commit messages.
- Geometry and physics errors get fixed with full rebuilds, not patches.
- Simulation and physical design run in parallel.
- Print doctrine: burner PLA until design freeze, then PAHT-CF and PVA.
- This README is the source of truth. Update it when a decision changes.

## Hardware (Sep 2026)

Nothing physically built yet. Only the Jetson has been exercised.

**Compute**

- Jetson Orin Nano Super dev kit: main controller, rover vision and ballistics. JetPack 6.2, MAXN Super mode, 128 GB SD boot, jtop installed. Hostname `GrappStar`, LAN 192.168.0.249.
- XIAO ESP32-S3 Sense: dart brain, onboard camera, WiFi, u.FL antenna.
- ESP32-S3 DevKitC-1-N8: winch real-time MCU, USB to the Jetson.

**Motors**

- 12 V universal wiper motor: capstan drive. Wires red/black/green; green is the park switch, tape it off.
- Biax worm N20, 12 V 27 RPM, x4: dart deploy through-axle (1 used).
- Self-locking N20, 17 to 68 RPM, x4: dart kick sector gear (1 used).
- Bringsmart M3-shaft 100 RPM 12 V, x6: dart drills (3 used).
- No windlass motor yet. A Bringsmart on a D25 windlass gives about 0.26 m/s line speed with the 2x take-up, roughly 45 s to snub 12 m of slack. Open decision.

**Drivers and power**

- DRV8833 dual H-bridge x3: dart motors.
- BTS7960: winch driver.
- Dart: 2x 18350 button-top cells, 2S BMS, MP1584 5 V buck. Target cell spec 18 mm dia, ~35 mm long, 4.5 A continuous, unprotected. Confirm what arrived.
- Rover: 3S 5200 mAh 50C XT60 LiPo, B6AC charger, XT60 leads, inline fuses, barrel-to-XT60 pigtail for the Jetson.

**Sensors**

- BNO085 IMU and INA219 current sensor (dart), AS5600 magnetic encoder (reel payout count), 100 kg load cell + HX711 (tether tension).

**Mechanical and print**

- Bambu X2D, PLA only for now.
- F693ZZ flanged bearings, 693ZZ bearings, M3x4x5 heat-set inserts, DIN7991 M3 screws, shaft collars, JST-XH, Dupont, SMA cable + adapters.
- Tether: 150 lb Dyneema braid. Still to confirm: 300 m spool, 80 kg+ swivels, crimp sleeves, cheap spinning reel for the staging spool, rod-guide eyelets.

**Deferred**

- NVMe SSD, final filament and dryer box (after design freeze), launcher electronics, Arducam IMX477 (Kiyo X footage decides).

## Dart (KEDGE)

- Envelope D30 x 250 mm, ~245 g. Launched tail-first, tether pre-fed through the eye.
- v20 STL family: dogleg kick_v11 + slotted-yoke body_v19, 14-step collision sweep passed.
- 55% scale proof-of-concept print set generated (snap-rivet pins). Not printed yet.
- Five motors: 1 deploy (biax worm N20 as through-axle, legs keyed on D-flat stubs, flanged bearings in hull bosses take radial load, hard stops take anchor force), 1 kick (sector gear on a self-locking N20), 3 drills (Bringsmart, steel spine).
- Geometry laws, do not violate:
  - 21 mm dogleg in the kick leg. drop = pivot height - hull radius + pole radius (verified numerically).
  - Any dogleg in the kick leg requires a channel cut through the bridge arm (slotted yoke). Hard constraint.
  - Pin lengths must span the full joint stack: 10.6 mm drill joints, 14.0 mm clevis.
- Sim: Python geometry solver plus anchor physics (support-polygon gravity, parallel-axis inertia for roll, righting basin thresholds, hull bedding during clamp, tip-gated pin-depth measurement).

## Reel (winch)

Stack, upstream to downstream:

staging spool (storage, end-feed launch payout) -> tangential-hole windlass with its own small motor (slack snub, passive eyelet otherwise) -> capstan drum on the wiper worm through a one-way sprag bearing (carries the tow load) -> pinch brake + AS5600 pulley + load cell -> exit eyelet.

Mission phases: staged, launch (all passive, capstan freewheels on the sprag), range gate (AS5600 count trips the pinch brake), anchor (dart sets, line goes slack), snub (windlass takes up both strands at 2x), tow (sprag locks, worm hauls, windlass pays its wraps back out).

Numbers: 400 N tow target, 670 N line limit, ~13 N.m drum peak.

Status: reel v1 STL set (capstan drums, windlass hand rig, count pulley) rejected. Plan the reel before printing more parts. Windlass motor choice open.

## Launcher

External design, frozen. Die-spring launcher, ~12 m for v1, spring cartridge swappable for range. Rail sized for the D30 x 250 envelope. Release electrically triggered, cocking manual for v1. This repo covers the electrical interface only; electronics deferred.

## Vision / code

- Jetson: rover camera (Kiyo X webcam for now), landing-zone and ballistics, vision pipeline established.
- Dart: XIAO streams nose-camera images over WiFi; control link planned over ESP-NOW.
- Winch MCU: ESP32-S3 DevKitC real-time loop (brake, AS5600 count, load cell, BTS7960), USB to the Jetson.
- Dart bench bring-up order: XIAO headers + antenna, IDE + camera stream, motor spin tests, sensors, full power integration once cells arrive.

## Open items

- Windlass motor: buy a fast one or accept a slow snub.
- Dart cells: confirm what arrived.
- Tether hardware: confirm bought.
- Dart bench bring-up: not started.
