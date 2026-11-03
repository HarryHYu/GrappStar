# Contract requests from the jetson worker

For the coordinator to relay. None of these block the Jetson software: it already copes with both answers.

## C1 (base serial), to fw-base

1. **SEATED ends every reel.** Whatever `until` says (`tension`, `stop`), the base should stop reeling and send
   `SEATED` when the seated switch closes. The Jetson's anchored check uses `until: tension`; if the anchor fails, the
   dart comes all the way home and must not be dragged into the cradle at full duty. (The mock base does this.)
2. **Line length.** C1 caps lines at 256 bytes. The full telemetry object with readable switch names is about 320
   bytes. fw-base already caps replies; please document the exact telemetry keys and the `sw` key names in
   PROTOCOL.md so the Jetson page shows the right ones. The Jetson accepts longer lines anyway.
3. **PROTOCOL.md.** Not on w/fw-base yet when checked (11:05). The Jetson team derived the protocol from the source
   (`jetson/docs/C1_AS_IMPLEMENTED.md`). Please confirm or correct it when PROTOCOL.md lands.

## C2 (dart HTTP), to fw-dart

Final API v4 (`firmware/dart/API.md`) covers everything the Jetson needed (`/arm?auto=0` for a log-only arm, 409
refusals, `t_ms` for clock sync, `/release` ending in `seq: released`). Implemented on the Jetson side.

4. **IMU chip axes.** The flight log is in BNO085 chip axes and the mapping is found on the bench (BENCH_TEST.md
   step 7). The Jetson takes it as a setting (`MissionConfig.imu_axes`, for example `"y,-x,z"`). Please make
   BENCH_TEST.md say which string to write down, in that form (body x = nose, y = left, z = up when level).
5. **Accelerometer saturation** at about 78 m/s^2 is fine for the Jetson (it uses over 4 g for launch and 3.5 g for impact, as the firmware does since D28).

## C3 (ballistics), to physics

6. The real table (spec v1.1: v_muzzle about 7.6 to 7.7 m/s, pitch 6 to 47 deg, best about 4.9 / 4.45 / 4.0 m near
   36 to 38 deg for 0.5 / 0.75 / 1.0 N, minimum about 2.3 to 2.5 m) replaced the placeholder automatically. The Jetson
   prefers the higher pitch when two solutions exist (D29) and adds the camera's 150 mm offset ahead of the pivot. If the physics model knows
   the line payout per shot, a `line_m` array per row would replace the Jetson's estimate (straight distance plus sag,
   times 1.1).

## C1 (base), new from systest F28

8. **Tension calibration flag.** Please add `"tension_cal": true|false` to `status` (true once `set k_n_per_a` or
   `cal` tension has been done on this unit, saved with the bench record). Today the Jetson can only compare
   `k_n_per_a` against the known defaults (8, 17, 14.8, 13.1), and that list breaks each time the physics default
   moves (F28). The Jetson already reads the flag when present (`mission.k_is_factory`).

## Unknowns that need Harry

7. **Rover drive interface** (motor controller, protocol). The mission asks the operator to drive by hand during the
   tow. `grappstar/rover.py` has the hook class for automation.
