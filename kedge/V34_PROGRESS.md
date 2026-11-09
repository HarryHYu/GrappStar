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
- [ ] kedge_v34.py geometry
- [ ] tools updated for signs (parts.json "sgn" per moving root; leg = "righting_leg")
- [ ] audit CLEAN 1 deg, launcher fit, STEP, wire room, mass props
- [ ] review shots, viewer pub34, pack, dist, docs

## Key numbers
(fill in)

## Open problems
(fill in)
