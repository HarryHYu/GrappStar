# Review worker report (in progress)

Worker `review`: fresh-eyes adversarial review rounds of other workers' code. Owns review/ only; never edits
other directories. Method per topic: one finder subagent per lens reads the code itself and reports only concrete
defects (scenario, evidence, exact fix); every medium-or-higher finding goes to two skeptic subagents told to refute
it; a finding is kept unless both refute it. Tooling: review/tools/to_md.py turns a workflow result into a topic file.

## Round 1 (12:20 Sydney, base-v1 0bfd59a)

| Topic | File | Confirmed | Refuted |
|---|---|---|---|
| A Contract C1 base <-> Jetson | r1_A_contract_C1.md | 15 (5 high, 5 med, 5 low) | 0 |
| B Dart firmware | r1_B_dart_firmware.md | 5 (4 med, 1 low) | 0 |
| C Jetson vision/video/ballistics | r1_C_jetson_vision.md | 8 (1 high, 5 med, 2 low) | 0 |
| D Base firmware | r1_D_base_firmware.md | 17 (5 high, 6 med, 6 low) | 1 |
| F Physics models (early round 2 item) | r1_F_physics.md | 9 (3 high, 5 med, 1 low) | 0 |

Running: E base CAD and assembly, G build docs, H Jetson app/mission/UI, I CAD lib and tools pipeline.

Note: findings inside one topic can share a root cause (finders work independently); the summary table in each
file lists them separately so owners can tick each off.
