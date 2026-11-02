# GrappStar

A rover fires an anchor dart (KEDGE) on a Dyneema tether. The dart films its flight and landing from a nose camera,
screws itself into the ground, and the rover winches itself out against it. One wiper motor runs the whole base
(launch, aim and reel), swapping between systems.

## Start here

1. `knowledge/SYSTEM_PLAN_2026-11-03.md`: the current agreed system plan, decisions, parts and electronics. Read first.
2. `kedge/CLAUDE.md`: the full engineering handover (every caliper measurement, dart geometry v29/v30, electrical,
   firmware, files, reviews). Section 10 covers the 3 Nov overnight ground-station design, now superseded by item 1.
3. `knowledge/HARRY_MESSAGES_AND_PASTES.md`: every message and paste from the 2-3 Nov sessions, including the S-KAS
   brainstorm and the August log.
4. `originals/`: everything Harry supplied: launcher v10 and reel handovers, reel v7, mini reel v9, desk launcher v8/v9/v10
   STLs, piston, reel and payout HTML sims, the caliper PDF with photos (`measurements.pdf`, Git LFS), the cloud handover
   zip (`KEDGE_handover.zip`, Git LFS), v26 dart, POC print, Lean Canvas, first print files.

## Layout

| Path | What |
|---|---|
| `knowledge/` | system plan, Harry's messages, Claude memory notes, screenshots Harry sent |
| `kedge/` | the working project: dart generator (`kedge_v27.py`), pipelines, firmware (`fw/`), ground station v1 (`gs/`) |
| `kedge/out29`, `kedge/out30` | dart v29 and v30 model data (STL, STEP, npz, audit, physics) |
| `kedge/pack_v29`, `kedge/pack_v30`, `kedge/dist_v29`, `kedge/dist_v30` | dart print packs, PDFs, zips (assembly STEPs via Git LFS) |
| `kedge/pack_gs`, `kedge/site` | ground station v1 print pack, HD mission sim, assembly steps, guide PDF (superseded design) |
| `kedge/fw` | dart firmware (`kedge_dart`), ground station firmware (`gs/grappstar_gs`), PC tests, Jetson scripts |
| `originals/` | Harry's source files, untouched |
| `docs/README_sep2026.md` | the September project README (hardware inventory, earlier reel concept) |

## Not in the repo

- `kedge/.venv` and `node_modules`: rebuild with `kedge/setup_mac.sh` and `npm install`.
- `kedge/work/meas/*.png`: 600 MB of page renders of `originals/measurements.pdf`, regenerate from the PDF.

## Rules

- No em dashes in any deliverable. Plain black Arial PDFs, no name on documents.
- Every geometry change goes through the exact-mesh collision audit before it is shown.
- Only parts Harry already has, plus 608 bearings. Everything else printed (Bambu X2D, PLA; PPA-CF for loaded gears).

Online: ground station v1 site https://claude.ai/artifact/Uof3qNiqzPXed57HCM79jK, dart viewer
https://claude.ai/artifact/Sqt8tF67veTYfxQjUFTZ5f
