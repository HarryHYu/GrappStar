# basesim

Firmware-in-the-loop simulator of the GrappStar base station. It compiles the REAL base firmware logic
(`firmware/base/grappstar_base/base_logic.h`) together with the firmware's own physics model (`firmware/base/test/sim.h`:
wiper, drum, aim, cock, capstan, line, dart, fault injection), ticks both every 5 ms in real time (or faster) and
speaks the C1 JSON-lines protocol exactly like the USB port: one JSON object per line, replies, telemetry, events.

## Build

```
make                                  # FW defaults to ../../../firmware/base, else /tmp/claude-0/basesim/firmware/base
make FW=/path/to/firmware/base        # any checkout or `git archive origin/w/fw-base firmware/base` extract
```

Needs g++ (gnu++17). `make` rebuilds when the firmware headers change.

## Run

```
./basesim [--speed X] [--pty] [--line-out M] [--anchor-taut-at M] [--seat-at M] [--load-cell]
```

| option | default | meaning |
|---|---|---|
| `--speed X` | 1 | simulated time runs X times faster than the wall clock (timeouts in the firmware scale too) |
| `--pty` | off | serve the port on a pseudo terminal; its path is printed as the first line on stderr (pyserial opens it) |
| `--line-out M` | 0 | line out at start (0 with `--seat-at 0`: the dart sits in the cradle, `SW_SEATED` closed) |
| `--anchor-taut-at M` | -1 | anchored tow: the line goes taut at M m out; -1 = a free dart |
| `--seat-at M` | 0 | the dart seats when the line out reaches M |
| `--load-cell` | off | HX711 fitted (`tension_n` in telemetry, `status.hx` true) |

At start it prints `HELLO` (like the sketch), the firmware prints `BOOT` 60 ms later. Like the firmware PC tests the
new build is declared at rest once (`cal cock_rest`) and the tension estimate constants (`k_n_per_a`, `free_a`) are set
to match the simulated motor, as a calibrated bench would be. Everything else is the firmware default.

## Scenario commands

Lines starting with `#sim ` are handled by the harness and never reach the firmware (on stdin, or on the pty):

| line | does |
|---|---|
| `#sim anchor M` | the dart is anchored: the line goes taut at M m out |
| `#sim free` | the dart lets go (free retrieve, it seats at the seat point) |
| `#sim seat M` | the dart seats when the line out reaches M |
| `#sim set NAME VAL` | set a sim field (below); `true`/`false` accepted for flags |
| `#sim fault NAME VAL` | the same as `set`, reads better for faults: `#sim fault drumJam 1`, `#sim fault encMissing 0` |
| `#sim dump` | writes one `{"sim":{...}}` line with the truth (line out, tension, shots, drum and wiper angles) |

Fields: `lineOut`, `anchorTautAt`, `anchorK` (N per m once taut), `seatAtLine`, `lineOutAfterShot` (8.0), `slack`,
faults `drumJam`, `drumJamAbove`, `toothOnTooth`, `toothNeedWiper`, `extraLoadNm` (stall), `searBroken` (no release),
`encMissing` (AS5600 gone), `encGlitchEvery`, `magnetWeak`, `encSign`, `swStuckClosed` / `swStuckOpen` / `swChatter`
(physical switch bit masks), `vbatOverride` (brownout), `vbat0`, `vbatWired`, `hxFitted`, and the model constants
`drumDpsTrue`, `aimLoadNm`, `cockMaxNm`, `reelFreeNm`. Unknown commands are reported on stderr.

## Python

`jetson/tests/basesim_transport.py`: `SubprocessTransport` (a BaseClient transport) and
`start_basesim(**scenario) -> (proc, BaseClient)`; `client.transport.sim("anchor 6.5")` sends scenario lines.
`jetson/tests/test_base_realsim.py` runs a whole mission against it.
