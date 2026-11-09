# KEDGE dart v34: running handover (dart-cad4)

Read this first if you are a fresh worker. Generator: `kedge_v34.py` (copy of v33, remodelled). Outputs: out34, pack_v34,
dist_v34, pub34, review shots in out34/review/. Never touch kedge/reports/ (another agent). Commit only own files,
`git push origin dart-v1` (on rejection `git pull --rebase origin dart-v1`).

## Harry's rules for v34 (from the brief + two coordinator updates)
- Do NOT over-design: works and simple. NO latch (plain hard stop; a little worm load is OK, the worm self-locks).
  Reduce the worm load only with simple geometry (low bridle eye puts the tow moment into the deploy stop).
- Own pivot pins in bushings where simple (couplers running in hull bores, D-socket on the motor stub with play).
- Assembly must make physical sense: every cavity reachable (clamshell opens fully, tray carries everything), show
  exploded + assembly-stage shots. Sealed body (lips on seams, tight bores, sockets fill their seats).
- Looks amazing: flush, continuous, consistent fillets.

## Decisions (v34 layout)
- Pods at the rear: pivot x 132 (study 140: moved 8 mm forward so the leg pad fits behind the deploy stop, in front
  of the tail face the launcher pusher needs), z 10, drill axis |y| 31.15, fold FORWARD, deploy 50 deg to a carved hard
  stop (v33 pivot-block method, mirrored). One deploy N20 on the axle (motor pointing DOWN: low CG), v33 couplers.
- Drills: v33 tapered gimlets unchanged (mirrored to point forward; hands kept: left pod right-hand).
- Bridle eye: a lug on each arm head's outboard half hanging below the pivot (14 mm), so the tow pushes the pod into
  its deploy stop (moment about like the study's eye, worm unloaded).
- Tail boom, tail cap, boom anchor, foot, foot N20 removed. Hull halves have integral tail end walls (flat pusher face);
  the two JST sockets move to the tail end face lobes (|y| 11.5-17.3, outside the pusher's |y| < 11).
- Righting leg: hinge on the top centreline x 33, z 15; leg N20 box coaxial at y 0 in a knuckle just under the skin
  (motor pointing down), fork root (two cheeks in side pockets) on two couplers (Ø6 PPA-CF, D-socket on the stubs
  trimmed to 5 mm past the box), couplers run in hull bores; blade flush in a top groove; pad = flush arch strap on the
  tail top (x 143.5-149.3). Far stop 178 deg = the leg lying in the groove forward of the hinge (flute in the nose top).
- Low CG: 2 cells IN LINE on the keel (z -10), boards on decks over them, both N20 motors pointing down.

## Status
- [x] kedge_v34.py geometry builds in about 60 s (work/v34a = scratch build; `KOV='{"DEPLOY_DEG":46.75}'` = solved angle)
- [x] tools updated for signs: parts.json moving roots carry "sgn" (-1 for both groups) and "joint" ("leg" / "deploy");
      audit27.py, solve_deg.py, clearance.py, massprops.py, viewer.html read them. Leg part = "righting_leg" (P.KICK_DEG = P.LEG_DEG alias).
      audit belly rule (v34): below z -5 nothing between the hull and r 26 (the launcher cradle rings are r 21.4-23.4 up to z -5).
- [x] coarse audit (10/5 deg) CLEAN apart from fixed items (rerun at 1 deg in the pipeline)
- [ ] visual polish (forward trough at the hinge, flank socket openings, coupler ends, blade edges)
- [ ] launcher fit (eye lugs, bridle path), pack.py / runsheet.py / report.py / wires.py / wireroom.py for the v34 parts
- [ ] full pipeline `GEN=kedge_v34.py ./run_all.sh out34 v34 1 1`, pub34, make_dist v34, CLAUDE.md section 5, DART_V31_NOTES

## Key numbers (work/v34a, 9 Nov)
- mass 250.6 g incl. 14.6 g belly putty (Harry's ballast putty in two belly-rail pockets), CG folded x 87.5, z -0.12;
  anchored z -4.5; leg out z -0.25. Printed 81.6 cm3.
- pod pivot x 132, z 9.5 (study 10; 9.5 + the putty puts the CG on the axis), deploy solved 46.75 deg, drills 43.5 mm deep.
- leg: hinge x 33, z 14.5, 178 deg, pad centre 113 mm from the hinge (study 128: the pad must stay in front of the tail face for
  the launcher pusher), pad about 37 wide (study 56: limited by the folded pods' hubs).
- hard stops: deploy +2.5 deg 5.2 mm3, fold -3 deg 5.6, leg far rest +2 deg 90, leg fold -2 deg 440.

## Open problems
- leg shorter and pad narrower than the study (launcher pusher + folded pods); dart-physics to rerun the righting sim on out34.
- eye lugs hang to z -7.4 at |y| 29-34 (outside the cradle rings); bridle path past the carriage needs base changes (notes).
