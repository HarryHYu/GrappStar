---
name: grappstar-ground-station
description: "Where the one-wiper-motor launcher+reel design (built overnight 3 Nov 2026) lives, its online build pack, and the decisions Harry delegated"
metadata:
  node_type: memory
  type: project
  originSessionId: 9166a6d4-a3cd-40b0-b325-a012ce2aa581
  modified: 2026-11-02T14:25:49.809Z
---

Ground station (launcher + spincast reel on one wiper motor) was designed overnight 2-3 Nov 2026 while Harry slept ("make the call", work till 7 am). Code in `~/Downloads/kedge/gs/` (`run_gs.sh out` = whole pipeline, `make_site.sh` = site files), firmware in `fw/gs/`, print pack in `pack_gs/`. Online build pack artifact: https://claude.ai/artifact/Uof3qNiqzPXed57HCM79jK (publish from `site/index.html` with root `site/`; zips ship as `.zip.b64.txt` and are saved through the downloads capability).

Decisions made for him (he said all to scale, no more testing, ASSEF checkpoint about 13 Nov 2026): prototype scale sized to KEDGE v30; motor forward = reel, backward = launcher program gear; end-pull stationary spool replaces the v7 capstan/wrap cup; fixed 40 deg fire (rail lifts 4 to 40 by cam; a trip cord from a dowel on the sear to a sprung eye bolt fires it at rail 33 on a 35 deg lift); dart v30 (Ø17 x 72 augers) packaged; wiper motor dims unknown so a slotted adapter plate and crank-arm coupler.

State 7 am 3 Nov: fourteen review rounds done overnight, site v27, audit CLEAN, 105 firmware checks pass. Open: retrieve onto the cradle, wiper interface (measure first), drop time for dropMaxMs. Eleven review rounds.

**Why:** he wanted everything designed with only the big wiper motor (underactuation) and nothing left to hand operation.
**How to apply:** keep regenerating through `run_gs.sh` (audit must say CLEAN) and republish the same artifact; measured wiper torque/speed and spring rate go into `gs/physics_gs.py`. Related: [[kedge-dart]]
