# kedge/fw: old firmware folder (read this first)

The dart firmware lives in **`firmware/dart/`** (repo root). Flash, test and edit it there. Start with
`firmware/dart/README.md`, wire with `firmware/dart/WIRING.md`, test with `firmware/dart/BENCH_TEST.md`.

| Folder here | What it is |
|---|---|
| `kedge_dart/` | an exact copy of `firmware/dart/kedge_dart/` (current firmware, made by `firmware/dart/tools/sync_kedge_fw.sh`). Do not edit it here: `firmware/dart/test/run_tests.sh` fails while the two differ |
| `test/test_logic.cpp`, `test/last_run.txt` | copy of `firmware/dart/test/test_logic.cpp` and its last output (same script) |
| `test/test_gs*`, `gs/`, `reel/` | the old ground station (v7 to v27 era). Superseded by `firmware/base/` and `base/`. Kept for history |
| `jetson/` | the old Jetson scripts for firmware v3 (`kedge_client.py`, `mission.py`, mocks, `pulltest.py`). Superseded by `jetson/grappstar/` (the dart client is `jetson/grappstar/comms/dart.py`). `kedge_client.py` still works against v4 (checked by `firmware/dart/sim/test_sim.py` test 10) |

Decision (dart-fw, 7 Nov 2026): keep a synced copy instead of deleting the folder, because `kedge/make_dist.sh`,
`kedge/setup_mac.sh` and `kedge/report.py` read these paths.
