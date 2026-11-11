# KEDGE dart v35: running handover

Read this first if you are a fresh worker. Generator: `kedge_v35.py` (copy of the final kedge_v34.py, pods turned round). Outputs:
out35 (out35/READY = CAD final, the Blender pipeline waits on it), pack_v35, dist_v35, pub35, review shots in out35/review/
(01-22 review34.js, v01-v12 = node review34.js out35 views_v35.json). Never touch kedge/reports/ (another agent) or base/ (parked). Commit only own
files, `git push origin dart-v1` (on rejection `git pull --rebase origin dart-v1`).

## The brief (coordinator, 11 Nov)
- The drill tips must rake BACK toward the tether, like a tent peg. Pods hinge forward (about x 50) and fold BACKWARD along the
  flanks; deployed the tips go down and aft, into the soil near the tail under the line.
- A printed deploy stop carries the tow load: the tow must close the pod onto that stop, never open it.
- Keep unless forced: arms, augers, couplers (press fit + CA), drill motors, nose putty, tail tie, righting leg, battery, ESP,
  firmware pin map. New: hull halves and tray. Mass near 248 g, recheck the CG for flight.
- Rules: only parts Harry has, no over-design, assembly must make physical sense, look at renders from every angle, kill by PID,
  disk tight (clean temporary outputs).

## Decisions
- Pivot x 50 (P.DRILL_X), z 9.5 (P.DRILL_PIV_Z, kept: the launcher's stop springs pass at z -1.1 outboard of |y| 25), |y| 31.15.
  parts.json pods "sgn" +1. Deploy solved 46.75 deg as v34 (the motor end in the hub 0.5 above flat ground).
- Pods reused: mirror_x() is a no-op (P.POD_DIR 1), so the v33/v34 arm and auger are built pointing +x as v33 did. The print STLs
  are the v34 ones by rotation (checked to 0.000 mm): v35 drill_leg_L = v34 drill_leg_R etc. So the LEFT pod has the left-hand auger
  (want_right = side < 0) and the swapped drill pair.
- Pivot blocks: v33 circles unmirrored, stops carved by the arm swept clockwise in x-z (the deploy stop forward of and below the head).
- Interior re-laid: leg cradle | disc 1 | deploy cradle (X_DCR 43.6..56.4) with DRV#1/#2 beside it | disc 1b (X_D1B 56.6..58.6, the
  B- spring on its rear face, top window |y| 13.2 above z 7) | cells 62..136 | rear disc | sockets, capacitor on the tail key.
- Deploy worm box under the righting leg's bridge: P.LEG_CH channel in the bridge underside (leg frame x 9..24.5, 2.4 under, ±7) and
  a box cavity / hump in the hull top. The leg is a reprint for this.
- Drill leads: tight holes at x DRILL_X + 10.5 (60.5), z 16.5, |y| 11.5.
- USB-C: the right pod covers the window folded and deployed (USB-C plug box checked 0 to 46.75 deg): USB before the pods are fitted,
  OTA after (run sheet 4, 6.4, 7). Window kept (needed for the build-time flash and check).

## Status (11 Nov)
- out35: audit CLEAN at 1 deg, launcher fit FITS (1684 states), ALL STEP VALID, wire room OK. Mass 249.6 g, CG folded x 75.57 z +0.39.
- Righting / tow sim: sim_righting/v35_check (run35_shared.mjs, summarize35.py, v35_righting_tow.json, the two .job files; run from a
  folder with @dimforge/rapier3d-compat@0.14.0). 100 % righted (random, named poses, fixed paths). Tow: worm 0, stops 4.24 N m both.
  Holding: loam moves at 45 to 49.5 N (v34 held 55), sand 17.2. Cause: anchors near the tail, nose pitches up about the tail edge.
- Flight: flight35.py -> phys35/flight35.json. Nose first at every rail, better than v34 (CG 7.6 mm further forward).
- Docs: pack_v35/KEDGE_v35_run_sheet.pdf (runsheet35.py), KEDGE_v35_engineering_report.pdf (report35.py), wire list (wires34.py,
  v35 routes), dist_v35 (all_STL by material, reprint_vs_v34 zip), pub35 + artifact Sqt8tF67veTYfxQjUFTZ5f (v35).

## Open
- Holding in loam is below v34 in the study sim (isotropic 25 N soil cap a drill); the drills are loaded about 95 % sideways now.
  A soil pull test decides; nothing changed in the CAD for it.
- Firmware: no change needed if the deploy N20 leads are swapped at DRV#1 AOUT1/AOUT2 (else PINS[DEPLOY].sign -1).
