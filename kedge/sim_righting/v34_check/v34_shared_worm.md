# v34: one deploy worm for both pods (dart-physics, 9 Nov 2026)

Question: the sim gave each pod its own worm, so the review's 0.55 N m was one pod's share. With ONE deploy N20 (stall 0.21 N m) on one
press-fitted axle, what does the worm carry?

Model (`run34_shared.mjs`, a subclass of the study sim's `Sim`; simCore extracted unchanged, same seeds as v34_righting_check):
- Rigid axle: both pods are one body on one joint, one worm with the sim's model (3 deg backlash, 25 N m/rad gear spring, stall
  0.21 N m), the two pods' stops act together.
- Press fit with compliance: each pod on its own joint, coupled to a massless axle by a torsion spring (25 N m/rad a side, about a
  Ø7 PPA-CF coupler plus the Ø3 stub) with 0, 0.5, 1.0 or 1.5 deg of residual play per pod (total, so +-0.75 at 1.5), the axle on the
  one worm. 40 N m/rad was also run; it is the stiffest the sim's 1 ms explicit springs allow and gives numerical spikes at impact in
  about 15 % of runs, so 25 is the main case.
- Real eye lug (3.9 forward of and 6.8 below the pivot folded; (134.3, +-31.65, 2.0) anchored), legs 78 mm to a sliding loop (equal
  leg tension, 30.1 N a leg at 55 N), drill axis |y| 31.15, v34 masses as v34_righting_check (hull 191 g, CG x 83.4, 3.3 below the axis).
- Firmware with one deploy channel: "roll on the low pod" moves both pods; the plant drives the axle until both tips touch or it
  stalls (no one-pod levelling); after drilling to the stops the one worm backs off 0.3 deg, as the sim does.
- Worm torque logged every 1 ms in every phase. Loam 80 random landings (seeds 1000-1039, 2000-2039), sand 20 (1000-1019), rover 3 m.

Results (`v34_shared_worm.json`; tow worm = peak in the tow hold, max / median / runs over 0.21 N m):

| Axle | Righted loam, sand | Held 55 N, loam | Tow worm, loam | Tow worm, sand |
|---|---|---|---|---|
| Separate worms (the sim as published, reference) | 80/80, 20/20 | 80 | 0.567 / 0.294 / 62 | 0.214 / 0.140 / 3 |
| Rigid axle, one joint | 80/80, 20/20 | 80 | 0.340 / 0.142 / 1 | 0.227 / 0.112 / 1 |
| Press fit 25 N m/rad, no play | 80/80, 20/20 | 80 | 0.162 / 0.130 / 0 | 0.124 / 0.107 / 0 |
| Press fit, 0.5 deg play per pod | 80/80, 20/20 | 80 | 0.270 / 0.158 / 15 | 0.133 / 0.112 / 0 |
| Press fit, 1.0 deg play per pod | 80/80, 20/20 | 80 | 0.484 / 0.253 / 59 | 0.311 / 0.146 / 3 |
| Press fit, 1.5 deg play per pod | 77/80, 19/20 | 77 of 77 | 0.419 / 0.247 / 49 | 0.199 / 0.136 / 0 |
| 40 N m/rad, no play (sensitivity) | 80/80, 20/20 | 79 | 0.156 / 0.133 / 0 | 0.126 / 0.108 / 0 |
| 40 N m/rad, 1.5 deg play (sensitivity) | 80/80, 20/20 | 77 | 0.439 / 0.251 / 50 | 0.201 / 0.141 / 0 |

Sand: every anchored run in every case slides, nearly all at 17 to 18.6 N (one at 22 N, one at 50 N): the sand's grip on the drills,
as in the published check (0 of 20 held 55 N).
The worm is not what limits it.

Which phase. The holding peaks above 0.21 are all in the tow hold, while the tension rises to 55 N after the worm has backed off at
the stops (the rigid case's one run over: 0.34 N m at 30 N, seed 1005). The other phases are the same for every axle model:
- impact (flight and landing phases): 0.5 to 3.4 N m spikes on the held worm in nearly every run (median 0.8 to 1.1), the folded
  pods slapping the ground; the separate-worm sim shows the same,
- righting pushes: median 0.23 to 0.32, max 0.43 to 0.78 (the worm driven to stall, then the dart dropping back on a held pod),
- drilling to the stops: 0.19 to 0.21 (the stall itself),
- the leg worm: under 0.07 in the tow, up to 0.74 in righting, 1.9 at impact.

Why play undoes the pairing. After drilling, the worm has pushed the axle through the play, so both pods sit on their deploy stops
with the axle 0.75 deg ahead of them. When the soil pushes one pod toward fold, that pod drags the axle back onto the worm (0.3 deg)
while the other pod stays on its stop, held there by the bridle. The other pod only joins once the axle has crossed the whole play
back, i.e. after the worm's gear has wound up by (play - 0.3 deg) x 25 N m/rad: about 0.52 N m at 1.5 deg, 0.09 N m at 0.5 deg.

Verdicts:
- One worm on a stiff axle with no play: under the stall in 79 of 80 loam runs (median 0.13 to 0.14 N m), 20 of 20 in sand. The
  review's quasi-static estimate (median 0.13, one 0.39) is confirmed. Realistic press-fit compliance does no harm (max 0.16).
- 1.5 deg of residual play per pod: back near the separate-worm load (median 0.25, 64 % of runs over the stall). The press fits must
  have no play: 0.5 deg per pod already lets 15 of 80 loam runs over (median 0.16), 1.0 deg 59 of 80. Glue them (the B4 fix) and check
  on the coupon that a pod, with the axle held, does not move more than about 0.3 deg (the worm's own back-off).
- Righting and the 55 N hold are unaffected by the axle (100 % righted, 80 of 80 held in loam) except: with play, 4 of 100 runs end
  "stuck" at 16 deg tilt (the pod hold sags back past the firmware's 15 deg righted threshold after the push stops at 10 deg), and
  the sim's drive target (stop + 3 deg) is used up by backlash and compliance before the current reaches 0.9 x stall, so the drill
  phase ends on its 30 s timeout. Both are firmware thresholds for dart-fw, not physics limits.
- Putty / CG (part 3 of the brief, `../../DART_PHYSICS34.md`): rigid axle without putty 0.175 / 0.141 / 0, and with the highest CG
  (no putty, bought parts above the CG +10 %, below -10 %) 0.207 / 0.142 / 0, all righted; with 1.5 deg play the medians stay 0.27
  to 0.28. Nothing changes sign.

Run: from a folder with @dimforge/rapier3d-compat@0.14.0, `node run34_shared.mjs "$(cat X.job)"` with the jobs stored in
`v34_shared_worm.json` ("jobs"), then `python3 summarize_shared.py v34_shared_worm.json <outputs>`. 100 runs take about 4 minutes on
one core.
