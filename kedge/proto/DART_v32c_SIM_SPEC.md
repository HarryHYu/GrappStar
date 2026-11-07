# KEDGE dart v32c: preliminary numbers for simulation

Preliminary. Bought-part masses are catalogue values (weigh them on the day); printed masses assume 4 walls (1.7 mm)
and 40 % infill. Source: kedge/out32 (physics.json, mass_props.json, parts.json), 7 Nov 2026. The drills are about to
change (tapered bits), so treat drill numbers as provisional.

## Frame
Origin at the nose tip on the hull axis. +x runs to the tail, +y left, +z up. Lying flat, the belly is at z -21.

## Size
| Item | Value |
|---|---|
| Hull | cylinder, diameter 42 mm, 1.5 mm wall |
| Body length, nose tip to tail cap | 151.97 mm |
| Overall length incl. tail boom | 179.5 mm |
| Nose | frustum, 20 mm long, from a 22 mm tip to the full 42 mm |
| Folded width (drill pods) | y -43.8 to +43.8 mm |
| Folded height | z -21 to +101.8 mm (the folded foot auger is the top point) |
| Tail boom | 28 mm long, 9 x 9 mm square, rounded; tether eye at x about 179 |
| Hull frontal area (hull only) | 1385 mm2 (diameter 42) |

## Mass
| Item | g |
|---|---|
| Total | 286.8 |
| Printed, PLA (13 parts) | 130.6 |
| Printed, PPA-CF (couplers x2, tail boom, boom anchor) | 8.3 (densities within 3 % of PLA) |
| Bought (motors, cells, boards, wiring, screws) | 147.9 |
| Heaviest items | hull top 23.3, cells 2 x 22.0, tray 18.5, hull bottom 14.1, drill sleeves 2 x 14.1, wiring 12.0 |
| Motors | 3 x worm N20 10.5 each, 2 x drill gearmotor 11.0 each |
| Ballast | 0 |

## Centre of mass and inertia (folded)
| Item | Value |
|---|---|
| CG | x 87.0, y -0.4, z +8.2 mm (z is above the hull axis) |
| Inertia about the CG, kg mm2 | Ixx 157.6, Iyy 501.2, Izz 533.4, Ixy -5.7, Ixz -19.0, Iyz 2.6 |

## Aerodynamics (strip model, 7 m/s)
| Item | Value |
|---|---|
| Aerodynamic centre | x 51 to 53 mm (ahead of the CG) |
| Static margin | about -0.8 calibre: unstable on its own |
| Stability in flight | comes from the trailing tether, about 0.33 to 0.38 N at the tail eye |

## Launch (real launcher model)
| Item | Value |
|---|---|
| Spring energy | 12.1 J, from 2 springs |
| Stroke | 241 mm in 59 ms |
| Muzzle speed | 6.8 to 6.9 m/s |
| Peak acceleration | 17.7 g |
| Peak push on the tail face | 60 N |
| Launch angle | 6 to 47 deg (34 deg typical) |
| Range at 34 deg | about 4.7 m |
| Tether | 1 mm Dyneema; 0.25 N payout drag, plus about 4.7 N spool spin-up during the stroke |

## Impact (34 deg launch)
| Ground | Speed into the ground | Peak at CG | Pulse |
|---|---|---|---|
| Soft soil | 4.6 m/s | about 120 g | 2.1 ms |
| Gravel | 4.6 m/s | about 450 g | 0.5 ms |

## Anchoring hardware
| Item | Value |
|---|---|
| Drills (x2) | hollow sleeve augers around the motors: flights diameter 22 mm on an 18.7 mm core, pitch 15 mm, 100 mm long, opposite hands. They deploy 45 deg from pivots at x 56, z 12, reaching 44 to 49 mm into the ground |
| Foot | auger diameter 13 x 68 mm on the kick leg. The leg plants at 200.75 deg; the foot goes 63 to 68 mm into the ground |
| Drill motors | 100 rpm at 12 V; at 8 V both together draw about 700 mA at stall |
| Worm N20s (kick, deploy, foot) | about 0.21 N m stall, self-locking |
| Holding, dry sand | about 14 N |
| Holding, moist loam | about 73 N |
| Holding, firm clay | the drills cannot go in |
| Deploy time | about 9 s in sand, about 53 s in loam (the drills mill their way in) |

## Power
| Item | Value |
|---|---|
| Battery | 2S 18350 (6.0 to 8.4 V), 5.18 Wh |
| Draw with video on | about 2.5 W, so about 1.85 h |
| Worst motor current | 1.2 A |
