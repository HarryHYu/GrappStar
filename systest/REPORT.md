# systest: end-to-end system test (worker `systest`, 3 Nov 2026)

Status: DRAFT (updated through the day; final at 18:20 Sydney).

## What it is

`systest/run_mission.sh [scenario|all|--list]` runs a whole GrappStar mission with the REAL code of all three parts
talking to each other over their real interfaces, no hardware:

| Part | What runs | Link |
|---|---|---|
| base | `systest/build/base_shim`: `firmware/base/grappstar_base/base_logic.h` (the header the sketch compiles) + the plant model `firmware/base/test/sim.h`, wired like `grappstar_base.ino` (5 ms tick, reply before events, 24-line event ring, telemetry dropped when the port is full, NVS writes 20 ms with saveAck) | C1 JSON lines over a pseudo-terminal (pyserial opens it like `/dev/ttyACM0`) |
| dart | `firmware/dart/sim/kedge_sim`: the unchanged `kedge_dart.ino` on stub Arduino headers, plant and IMU traces | C2 HTTP on 127.0.0.1 |
| Jetson | `jetson/grappstar` App: `comms/base.py`, `comms/dart.py`, `mission.py`, target vision on a synthetic rover view (orange cone), landing analysis on a synthetic dart flight video (`jetson/grappstar/sim/flight_video.py`) | |

The harness (`systest/harness.py`) is the outside world and the operator: when the base plant releases the carriage
the dart sim plays a launch and impact trace and the dart video plays the rendered flight; ANCHORED makes the line
taut on the base plant; the release frees it; the rover hook drives toward the dart during the tow; a scripted
operator answers every prompt. Faults go in through the shim control socket (drum jam, tooth on tooth, stall load,
AS5600 missing, brownout, snag, USB unplug / glitch, raw garbage on the serial line), the dart `/sim/set` knobs, or
by SIGKILL and restart of the base process (power loss: the plant and what reached NVS survive).

Every check uses plant truth (line tension, wraps, seated, collar engaged, dry fires) as well as events and telemetry.

Relation to `jetson/tools/basesim` + `jetson/tests/test_mission_realsim.py` (jetson worker): those test the mission
with the real base logic and mostly the mock dart, with fake cameras and a fake analysis, inside pytest. This harness is
standalone, runs the whole App (vision, analysis, recording) with the real dart firmware sim, and adds power loss
with NVS persistence, USB unplug / replug, raw serial injection and per-scenario fault triggers, which basesim does
not have. Nothing under jetson/ or firmware/ was changed.

## How to run

```
systest/run_mission.sh --list            all scenarios
systest/run_mission.sh                   nominal mission (about 2 min)
systest/run_mission.sh all               everything (about 1.5 h)
systest/run_mission.sh power_wrap,ref_power_wrap
systest/preview.sh [scenario|all]        same, on base-v1 + w/fw-base + w/fw-dart + w/jetson (before a merge)
python3 systest/c1_order.py              the C1 command order each recorded run used, against BASE_SPEC 10.1
```
Each run leaves `systest/runs/<scenario>/`: `timeline.jsonl` (steps, prompts and answers, events, dart phases, plant
snapshots, injections, checks), `summary.json`, `base.log` (every serial line both ways with sim time), `dart.log`,
`jetson/<run>/mission.jsonl` (the App's own log) and recorded videos.

## Scenarios

Two drivers: `mission` (the real `jetson/grappstar/mission.py` through the App, with a scripted operator) and `ref`
(BASE_SPEC 10.1 step by step with the Jetson's own `BaseClient` / `Dart` clients, plus operator-style recovery). The
`ref_*` scenarios test the firmware and dart against the binding sequence even while the mission lags behind it.

Plant: sim.h plus the D22 additions in the shim: every laid turn draws a capstan circumference from the dart side
(a free dart is tugged, an anchored one jams the cup), and tidy pulls an anchored line taut. Strict for `nominal`,
`land_short` and all `ref_*`; lenient for the mission fault scenarios until F4/F12 land (`SYSTEST_D22_ALL=1` for strict).

Checks per scenario (plant truth where possible): mission DONE, aim reached, fired, landed, anchored, tidy ran,
wrapped (plant wraps), REEL engaged and pulling, tow limit respected (peak plant tension per 5 ms tick, limit +15 %),
dart released, reeled in, unwrapped, seated (plant and SW_SEATED), back to LAUNCH, no dry fire (the plant counts the
collar leaving LAUNCH with the spring drawn), wraps laid without jamming, free dart tugged under 1 m, C1 command order
= BASE_SPEC 10.1, every injected fault actually fired, plus scenario specific ones.

## Results

(pasted from the last run below)

## Findings

See `systest/FINDINGS.md` (F1 onward), routed by the coordinator.
