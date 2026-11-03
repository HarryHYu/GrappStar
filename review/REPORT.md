# Review worker report (in progress)

Worker `review`: fresh-eyes adversarial review rounds of other workers' code. Owns review/ only; never edits
other directories. Method per topic: one finder subagent per lens reads the code itself and reports only concrete
defects (scenario, evidence, exact fix); every medium-or-higher finding goes to two skeptic subagents told to refute
it; a finding is kept unless both refute it. Low findings are listed unverified. Tools: review/tools/to_md.py and
emit.sh turn a workflow result into a topic file; review/tools/table.py prints the table below.

Round 1 started 12:20 Sydney on base-v1 0bfd59a. Round 2 re-reviews run on the newest code: base-v1 for
firmware and contract, the worker branch tips (read-only worktrees) where workers are ahead of base-v1.

## Topics so far

| Topic | File | Reviewed at | Confirmed | Refuted |
|---|---|---|---|---|
| Round 1 A: contract C1, base firmware vs Jetson | r1_A_contract_C1.md | 0bfd59a | 15 (5 high, 5 medium, 5 low) | 0 |
| Round 1 B: dart firmware (fw-dart) | r1_B_dart_firmware.md | 0bfd59a | 5 (4 medium, 1 low) | 0 |
| Round 1 C: Jetson vision, video, ballistics (jetson) | r1_C_jetson_vision.md | 0bfd59a | 8 (1 high, 5 medium, 2 low) | 0 |
| Round 1 D: base firmware (fw-base) | r1_D_base_firmware.md | 0bfd59a | 17 (5 high, 6 medium, 6 low) | 1 |
| Round 1 F: base physics models (physics) | r1_F_physics.md | 0bfd59a | 9 (3 high, 5 medium, 1 low) | 0 |
| Round 1 G: build documentation (learner following cold) | r1_G_build_docs.md | 0bfd59a | 21 (4 high, 13 medium, 4 low) | 1 |
| Round 2 A2: contract C1 against the D35 sequence (BASE_SPEC 10.1) | r2_A2_contract_C1.md | ba121b9 | 18 (5 high, 6 medium, 7 low) | 7 |
| Round 2 B2: dart firmware re-review (fw-dart) | r2_B2_dart_firmware.md | ba121b9 | 8 (3 medium, 5 low) | 0 |
| Round 2 C2: Jetson vision and ballistics re-review (w/jetson-vision 568939e, w/jetson 79e30ed) | r2_C2_jetson_vision.md | 568939e | 4 (1 high, 3 medium) | 1 |
| Round 2 D2: base firmware re-review (fw-base) | r2_D2_base_firmware.md | ba121b9 | 26 (4 high, 14 medium, 8 low) | 0 |
| Round 2 E: base CAD parts and assembly | r2_E_base_cad.md | 0bfd59a | 23 (2 critical, 8 high, 8 medium, 5 low) | 2 |
| Round 2 F2: physics re-review (w/physics e347790) | r2_F2_physics.md | e347790 | 6 (3 medium, 3 low) | 4 |
| Round 2 H: Jetson app, mission control, operator UI, comms robustness (jetson) | r2_H_jetson_app_mission_ui.md | 0bfd59a | 11 (8 medium, 3 low) | 0 |
| Round 2 I: CAD library and tools pipeline (cad-infra) | r2_I_cad_lib_tools.md | 0bfd59a | 7 (1 high, 2 medium, 4 low) | 1 |
| Round 2 J: builder dry run (Harry building from the repo alone) | r2_J_builder_dry_run.md | c631495 | 28 (11 high, 7 medium, 10 low) | 0 |
| Round 2 K: mechanism feasibility, load path torque and strength | r2_K_mechanism_feasibility.md | c631495 | 8 (3 high, 4 medium, 1 low) | 0 |
| Round 2 L: system test harness validity (w/systest) | r2_L_systest.md | 2226d7d | 15 (4 medium, 11 low) | 1 |
| Round 3 M: whole-system safety (people, LiPo, stored spring energy) | r3_M_system_safety.md | 863d58d | 8 (1 high, 6 medium, 1 low) | 1 |
| Round 3 N: Jetson setup, first run and dart wiring (learner) | r3_N_jetson_setup_dart_wiring.md | 863d58d | 7 (2 high, 2 medium, 3 low) | 0 |
| Round 3 O: params.py drift (modules that ignore or contradict the shared numbers) | r3_O_params_drift.md | 472bb65 | 13 (2 high, 5 medium, 6 low) | 0 |

Fix status: at 14:10 one checker subagent per confirmed finding re-read the newest base-v1 (863d58d; A2 at
472bb65): 115 fixed, 4 moot, 28 partly fixed, 23 still open of the 170 earlier findings. review/OPEN_ITEMS.md
lists every open and partly fixed item by owner (python3 review/tools/open_items.py regenerates it).

Note: findings inside one topic can share a root cause (finders work independently); each is listed so owners can
tick them off. Review subagents sometimes regenerate repo outputs (base/out) while proving a defect; I restore
those files before each commit so w/review only ever changes review/.
