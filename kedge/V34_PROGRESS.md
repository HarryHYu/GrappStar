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
  the two JST sockets move to recessed seats in the tail flanks under the pad (plug in sideways; the end face stays flat for the pusher and its seat pushrod).
- Righting leg: hinge on the top centreline x 33, z 15; leg N20 box coaxial at y 0 in a knuckle just under the skin
  (motor pointing down), fork root (two cheeks in side pockets) on two couplers (Ø6 PPA-CF, D-socket on the stubs
  trimmed to 5 mm past the box), couplers run in hull bores; blade flush in a top groove; pad = flush arch strap on the
  tail top (x 143.5-149.3). Far stop 178 deg = the leg lying in the groove forward of the hinge (flute in the nose top).
- Low CG: 2 cells IN LINE on the keel (z -10), boards on decks over them, both N20 motors pointing down.

## Status (9 Nov, late)
- [x] geometry, tools (sgn/joint), coarse + 1 deg audit CLEAN, launcher fit FITS (1684 states), ALL STEP VALID, wire room OK
- [x] polish: flush skin plugs, swept forward bed, tapered blade flaring into the pad, rounded socket seats, eye lugs above the
      launcher stop springs, tray on a flat print base
- [x] v34 tools: wires34.py (45 wires), wireroom corridors, runsheet34.py, report34.py, extra_shots stages, mkpub.sh, ovl.py, qrender.py
- [x] CLAUDE.md section 5 v34 block (results line to fill), DART_V31_NOTES entries for dart-physics, dart-fw, base
- [x] final pipeline run after the taper: CLEAN, FITS, ALL STEP VALID, WIRE ROOM OK; done: ./mkpub.sh out34 pub34; node pubshots.js pub34 out34;
      GEN=kedge_v34.py ./make_dist.sh v34; fill CLAUDE.md results; commit out34, pack_v34 (assembly STEP via LFS), dist_v34, pub34
- physics.py / flight_sim (dart-physics files) fail on out34 (they hard-code kick_leg / foot_auger): asked dart-physics in the notes.

## Key numbers (work/v34a, 9 Nov)
- FINAL out34: mass 249.4 g incl. 14.6 g belly putty, CG folded x 87.3, z -0.15; anchored z -4.38; leg out z -0.25. Printed 81.0 cm3.
- pod pivot x 132, z 9.5 (study 10; 9.5 + the putty puts the CG on the axis), deploy solved 46.75 deg, drills 43.5 mm deep.
- leg: hinge x 33, z 14.5, 178 deg, pad centre 113 mm from the hinge (study 128: the pad must stay in front of the tail face for
  the launcher pusher), pad about 37 wide (study 56: limited by the folded pods' hubs).
- hard stops: deploy +2.5 deg 5.2 mm3, fold -3 deg 5.6, leg far rest +2 deg 90, leg fold -2 deg 440.

## Righting check (study sim, v34 geometry): loam 40/40 righted, median 8 s, held 55 N; sand 20/20; worm peak 0.55 N m in loam.

## Open problems
- leg shorter and pad narrower than the study (launcher pusher + folded pods); dart-physics to rerun the righting sim on out34.
- eye lugs hang to z -7.4 at |y| 29-34 (outside the cradle rings); bridle path past the carriage needs base changes (notes).
- worm peak 0.55 N m in loam in the sim (which joint/phase unknown); the real eye lug gives about half the study eye's moment.
- physics.py / flight_sim.py / dart_model.py (dart-physics) need the sgn/joint convention and no foot.

## dart-cad5 (9 Nov, review fixes; kedge/out34/REVIEW_v34.md)
- B1 putty: belly pockets and belly rails REMOVED; the putty is in the NOSE (physics: helps flight, righting works anyway): 14.6 g round and
  under the camera tube up to a 45 deg collar at x 13.2, pressed in from the back before the nose goes on. Part "ballast" (mass_g).
  New permanent audit check: every printed part's npz mesh is ONE shell (trimesh split == 1).
- B2 right seat: 3-pin B3B-XH-A at true size (9.9), opening 10.2, pin pocket 7.0; opening corners r 0.4.
- B3 screws: 6 mm bosses at all 8 stations (disc-1 station x 39.8 to clear the XIAO), pilot Ø2.5 1.5 deeper than the tip with 1.5+ walls;
  corridor windows stop at z -6 under the lower bosses; seat pocket no longer 0.5 too deep; M3 x 8 modelled at its true 8 mm. Audit table
  screw_bite_mm3: 10.1 each (full 7 mm bite 11.9; the countersunk head takes the rest).
- Tail seam: a lip across the split cannot print (both halves print split face down), so deck 1 continues as a TAIL KEY into slots in
  both end walls; the front is keyed by the nose spigot + lock rib. Stale "seam lip" print note removed.
- B4 + worm load: press-fit couplers (stub socket 3.05 / flat 1.03, arm-head D bore 7.05, cheek D bore 6.05), CA, set on the fold stops
  with the worm at stall toward fold. Coupon v34 row + fit_coupon_pegs.stl.
- B5 bridle: tail grooves (65 deg rim, end wall to |y| 4.2 z 3.2), loop on the axis about 26 mm behind the tail face; left ON seat 1.2 lower
  (stowed leg 3.3 mm over the plug + 10 mm loop). launcher_fit models the rope over every base state; rope_checks (stowed in grooves,
  plug clearance, anchored legs 1.07 mm clear). Towed, the left leg runs 5 mm off the skin over the ON socket: short flat loop (run sheet).
- Cells: U saddles open 19.2 at the bottom (legs to the print plane). USB-C: notch to the split + tray tab; audit insertion sweeps
  (tray into bottom, top onto tray, 0.25 mm steps) report zero overlap. Capacitor Ø8 x 12 on the tail key. Grub-screw note gone;
  righting leg brim 5 mm. Run sheet: putty, press fits, direction check, pods to stall before loading, hull top straight down, bridle.
- FINAL out34 (dart-cad5): audit CLEAN at 1 deg (new checks: one shell per part, insertion sweeps zero, screw bite 10.1 each), launcher_fit
  FITS with the rope (rope checks ok: plug 3.26 mm, towed 0.9 mm), ALL STEP VALID, wire room OK (0.27). Mass 248.0 g incl. 14.6 g nose putty,
  CG folded x 83.14, z +0.44; anchored z -3.81; leg out z +0.34. Printed 79.95 cm3. Renders out34/review/c01-c24 (views_cad5.json).
- Open: towed, the left leg runs about 5 mm off the skin over the ON socket (short flat shorting loop, taped: run sheet 7.1); bite 10.1 of
  11.9 mm3 (M3 x 8 countersunk through the 1.5 wall: the head takes the rest; an M3 x 10 would give the full 7 mm).

