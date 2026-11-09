# KEDGE dart v34 physics: shared deploy worm, flight with the bridle, CG robustness (dart-physics, 9 Nov 2026)

Geometry and masses: out34 as committed on 9 Nov (249.4 g, folded CG x 87.32, z -0.15; pitch / yaw inertia 3.38e-4 / 3.79e-4 kg m2
from out34/mass_props.json). The press-fit coupler, rope-groove and screw-boss changes in progress move masses by a gram or two:
nothing below is that close to a limit.

Files:
- `sim_righting/v34_check/run34_shared.mjs`, `summarize_shared.py`, `v34_shared_worm.json`, `v34_shared_worm.md` (part 1)
- `flight34.py` -> `phys34/flight34.json` (part 2); `mass34.py` -> `phys34/cg34.json` (part 3)
- `dart_model.py` now reads parts.json `joint` / `sgn` (v31 to v33 unchanged). physics.py itself was not run: its landing and anchor
  modules still model the foot and the kick leg, which v34 does not have.

## 1. One deploy worm on a stiff axle

The study sim (index.html simCore, physics unchanged) with both pods on one worm, the real eye lug and a 78 mm sliding-loop bridle;
the firmware moves both pods together (one deploy channel). Loam 80 random landings, sand 20. Tow worm torque (the review's
0.55 N m metric) as max / median / runs over the 0.21 N m stall:

| Axle | Righted loam, sand | Held 55 N (loam) | Tow worm, loam | Tow worm, sand |
|---|---|---|---|---|
| Separate worms (as the sim, reference) | 80/80, 20/20 | 80 | 0.567 / 0.294 / 62 | 0.214 / 0.140 / 3 |
| Rigid axle, one joint | 80/80, 20/20 | 80 | 0.340 / 0.142 / 1 | 0.227 / 0.112 / 1 |
| Press fit, 25 N m/rad a side, no play | 80/80, 20/20 | 80 | 0.162 / 0.130 / 0 | 0.124 / 0.107 / 0 |
| Same with 0.5 deg play per pod | 80/80, 20/20 | 80 | 0.270 / 0.158 / 15 | 0.133 / 0.112 / 0 |
| Same with 1.5 deg play per pod | 77/80, 19/20 | 77 of 77 | 0.419 / 0.247 / 49 | 0.199 / 0.136 / 0 |

In sand every case slides at about 18 N (the sand's grip, as the published check). The peak is in the tow hold (rising tension
after the worm relaxes at the stops), as the review found. Verdict: a stiff axle with no
play keeps the one worm under its stall (median 0.13 to 0.14 N m), confirming the review's estimate; 1.5 deg of play per pod puts it
back near the separate-worm numbers, and even 0.5 deg lets 15 of 80 runs over: the couplers need no measurable play. Details, the play threshold and the other phases: `sim_righting/v34_check/v34_shared_worm.md`.

## 2. Flight with the bridle

Model: `flight34.py`, a planar rigid-body sim (pitch plane with gravity, yaw plane without) using flight_sim.py's strip aero unchanged
on the folded out34 geometry. flight_sim.py itself cannot take the bridle (one tow point on the axis) and its own mass table misses
v34 parts (237.7 g), so the new script takes mass, CG and inertia from mass_props.json. Check: on out32 it reproduces flight_sim.json's
v32c no-tether pitch case to the digit. Launch 6.8 m/s, rails 6 to 47 deg, 0.25 N payout drag along the trailing line, disturbance
at the muzzle either the cradle tip-off (0.4 rad/s) or flight_sim's 5 deg + 1 rad/s.

Where the bridle pulls: in pitch the loop sits on a circle round the eye axis, so the line always acts through the eye axis
(x 128.1, z 2.7, 2.85 mm above the CG): 40.8 mm behind the CG, whatever the leg length or a fixed knot. In yaw a sliding loop pulls
through the ellipse's centre of curvature, 14 mm in front of the eyes: 26.7 mm. The straight legs also run 1 mm into the arm heads
and the pivot blocks' aft corners (x 137 to 140.5), so the taut legs really leave the body at those corners: 52 mm (pitch) and 40 mm
(yaw). Both geometries are run; the truth is between them.

Airframe alone: aerodynamic centre x 47.0 (pitch) / 43.5 (yaw) at 10 deg, CG x 87.3: -0.96 / -1.04 calibre (v32c -0.81 / -0.85).
Without the line it tumbles at every rail, as v32c did. "Neutral speed" below = the speed above which the airframe's destabilising
moment beats the line's restoring moment; the flight is at 6.5 to 6.9 m/s.

| Tow | Pitch arm, neutral speed | Yaw arm, neutral speed | Tip-off: AoA max, sideslip max | 5 deg + 1 rad/s: AoA, sideslip |
|---|---|---|---|---|
| v34 bridle, legs from the lugs | 40.8 mm, 8.3 m/s | 26.7 mm, 6.7 m/s | 39, 13 deg | 47, 31 deg |
| v34 bridle, legs from the block corners | 52.2 mm, 9.4 m/s | 40.4 mm, 8.3 m/s | 32, 7 deg | 36, 18 deg |
| v34 bridle, fixed apex knot (lugs) | 40.8 mm, 8.3 m/s | 112 mm, 13.8 m/s | 39, 3 deg | 47, 9 deg |
| v34 with a tail eye at x 152 (for scale) | 64.7 mm, 10.5 m/s | 64.7 mm, 10.5 m/s | 24, 4 deg | 28, 12 deg |
| v32c, boom eye x 179.5 | 92.4 mm, 11.3 m/s | 92.4 mm, 11.3 m/s | 27, 4 deg | 32, 12 deg |

Verdict: it still flies nose first (AoA under 45 deg for the whole flight at every rail with the tip-off) and pitch is statically
stable, but on about half of v32c's restoring moment, so it turns into the flight path slowly. Yaw is about neutral with the legs
straight from the lugs (a sideslip picked up at the muzzle stays to impact) and stable if the legs ride on the block corners. A fixed
apex knot instead of the sliding loop makes yaw firmly stable (it does nothing for pitch), at the price the review gives: an off-axis
tow then loads one leg with the full 55 N.

Attitude at impact (sliding loop, legs from the lugs, tip-off nose up / nose down; body pitch: nose below the horizon is negative;
AoA: positive when the nose is below the flight path):

| Rail deg | Path deg down | Body pitch deg | AoA deg | Sideslip deg | Impact m/s | Range m |
|---|---|---|---|---|---|---|
| 6 | 24 | +12 / 0 | -36 / -24 | 8 | 6.9 | 2.5 |
| 15 | 29 | +9 / 0 | -38 / -28 | 11 | 6.7 | 3.3 |
| 20 | 32 | +1 / -6 | -33 / -27 | 12 | 6.6 | 3.7 |
| 25 | 36 | -14 / -15 | -23 / -21 | 13 | 6.6 | 4.1 |
| 30 | 41 | -32 / -28 | -9 / -12 | 13 | 6.6 | 4.4 |
| 34 | 44 | -48 / -41 | +3 / -3 | 13 | 6.5 | 4.5 |
| 40 | 50 | -72 / -62 | +21 / +11 | 12 | 6.5 | 4.5 |
| 47 | 57 | -95 / -84 | +38 / +27 | 11 | 6.5 | 4.4 |

So what it lands on: up to about 20 deg rail it arrives level or nose up to 13 deg high, i.e. flat on the belly with the tail and the
folded pods touching first; at 30 to 34 deg nose first along the path (the righting sim's landing assumption); at 40 to 47 deg nose
first and steep (62 to 95 deg nose down, the nose ahead of the path), then it falls onto its back. With the legs on the block corners
the same trend is milder (AoA -30 to +32, level landings only below about 12 deg). v32c lands nose first at every rail (body 16 to
73 deg nose down, AoA -8 to +26). Roll: nothing rolls it in flight (launch belly down, line symmetric), so it lands belly down unless
the cradle spins it. Righting copes with every one of these; the flat landings at shallow rails put more of the impact into the pods
and pivots than the sim's nose-first landings.

## 3. CG robustness (putty, catalogue masses)

`mass34.py`: bought parts x1.1 / x0.9 all together, and the adversarial splits (parts above the CG x1.1 and below x0.9, and the
reverse; the same fore and aft). Printed parts, putty and the wiring / glue estimates not scaled.

| Case | Mass g | CG x mm | CG z mm |
|---|---|---|---|
| Putty, catalogue | 249.4 | 87.31 | -0.15 |
| No putty, catalogue | 234.8 | 87.21 | +0.59 |
| Putty, bought +-10 % (all cases) | 237.5 to 261.3 | 86.02 to 88.59 | -0.49 to +0.19 |
| No putty, bought +-10 % (all cases) | 222.9 to 246.7 | 85.83 to 88.57 | +0.23 to +0.95 |

The putty moves the CG 0.74 mm up and 0.1 mm forward; the bought-mass spread moves it 1.4 mm in z and 2.7 mm in x, on a 21 mm hull.

Does anything in 1 or 2 change sign?
- Part 1, no. Rigid axle without putty: 80/80 and 20/20 righted, tow worm 0.175 / 0.141 / 0 over the stall; without putty and with
  the highest CG (z +0.95): 80/80 and 20/20, 0.207 / 0.142 / 0. With 1.5 deg play the medians stay at 0.27 to 0.28 (still over).
- Part 2, pitch no: neutral speed 8.1 to 8.6 m/s in every case (9.2 to 9.7 with the legs on the corners), above the flight speed.
  Yaw with the legs straight from the lugs is the one sign that depends on the masses: neutral speed 6.5 m/s (CG aft) to 7.0 m/s
  (CG forward) against 6.8 m/s, so whether a sideslip grows or decays slowly is set by the catalogue masses (CG x +-1.3 mm), not by
  the putty (0.1 mm). With the legs on the corners it is 8.0 to 8.5 m/s, stable in every case. With or without putty the AoA and
  sideslip peaks move by under 1 deg.

Plain answer: the putty does not matter for v34. Righting works from any attitude (100 of 100 with and without it), the one-worm load
is the same, and flight is set by the line, not by 0.74 mm of CG height. It can be left out, which also removes blocker B1 (sealed
pockets). If ballast is wanted at all, the nose is where it helps: the same 14.6 g at x 15 moves the CG 4.3 mm forward and raises the
neutral speeds to 9.2 m/s pitch and 7.6 m/s yaw (10.3 and 9.1 with the legs on the corners).
