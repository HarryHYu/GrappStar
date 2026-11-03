"""Run the base firmware-in-the-loop simulator (jetson/tools/basesim) as a BaseClient transport.

basesim runs the REAL base firmware logic plus the firmware's physics model and speaks C1 JSON lines on its
stdin/stdout. SubprocessTransport has the same open/readline/write/close interface as grappstar.comms.base
SerialTransport and PipeEnd, so BaseClient talks to it like to the USB port:

    proc, base = start_basesim(speed=5, anchor_taut_at=7.5)
    base.cmd("mode", to="LAUNCH"); base.wait_event("MODE", timeout=10)
    base.transport.sim("free")          # scenario control: a "#sim free" line, never seen by the firmware
    base.close()                        # stops the simulator too

Binary: $GRAPPSTAR_BASESIM, else jetson/tools/basesim/basesim (built with make when missing, see find_basesim).
"""
from __future__ import annotations

import os
import select
import subprocess
import threading
import time
from typing import Optional, Sequence, Tuple

from grappstar.comms.base import BaseClient, BaseDisconnected

HERE = os.path.dirname(os.path.abspath(__file__))
TOOL_DIR = os.path.normpath(os.path.join(HERE, "..", "tools", "basesim"))
DEFAULT_BIN = os.path.join(TOOL_DIR, "basesim")


def find_basesim(build: bool = True) -> Optional[str]:
    """Path of the basesim binary, or None. Builds it with make (quietly) when it is missing and build is True."""
    env = os.environ.get("GRAPPSTAR_BASESIM")
    if env:
        return env if os.access(env, os.X_OK) else None
    if build:
        try:   # make is a no-op when the binary is up to date; rebuilds it when the firmware headers changed
            subprocess.run(["make", "-s", "-C", TOOL_DIR], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                           timeout=180, check=False)
        except Exception:
            pass
    return DEFAULT_BIN if os.access(DEFAULT_BIN, os.X_OK) else None


def scenario_args(speed: float = 1.0, line_out: Optional[float] = None, anchor_taut_at: Optional[float] = None,
                  seat_at: Optional[float] = None, load_cell: bool = False, pty: bool = False) -> list:
    a = ["--speed", str(speed)]
    if line_out is not None:
        a += ["--line-out", str(line_out)]
    if anchor_taut_at is not None:
        a += ["--anchor-taut-at", str(anchor_taut_at)]
    if seat_at is not None:
        a += ["--seat-at", str(seat_at)]
    if load_cell:
        a.append("--load-cell")
    if pty:
        a.append("--pty")
    return a


class SubprocessTransport:
    """Starts basesim on open() and reads / writes its stdout / stdin. close() stops it.

    BaseClient reopens a transport after a link error: a dead simulator is then started afresh (like a board reset).
    """

    def __init__(self, binary: Optional[str] = None, args: Sequence[str] = (), stderr_path: Optional[str] = None,
                 **scenario):
        self.binary = binary or find_basesim()
        if not self.binary:
            raise FileNotFoundError("basesim binary not found (make -C jetson/tools/basesim)")
        self.args = list(args) + scenario_args(**scenario)
        self.stderr_path = stderr_path
        self.proc: Optional[subprocess.Popen] = None
        self._buf = b""
        self._wlock = threading.Lock()

    # ---- the transport interface (grappstar.comms.base)
    def open(self) -> None:
        if self.proc is not None and self.proc.poll() is None:
            return                                   # already running (start_basesim opens it before BaseClient)
        err = open(self.stderr_path, "ab") if self.stderr_path else subprocess.DEVNULL
        try:
            self.proc = subprocess.Popen([self.binary] + self.args, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                         stderr=err, bufsize=0)
        finally:
            if self.stderr_path:
                err.close()
        self._buf = b""

    def readline(self, timeout: float = 0.2) -> Optional[bytes]:
        """One line without the newline, or None when nothing complete arrived in time."""
        t_end = time.monotonic() + timeout
        while True:
            i = self._buf.find(b"\n")
            if i >= 0:
                line, self._buf = self._buf[:i], self._buf[i + 1:]
                return line.rstrip(b"\r")
            p = self.proc
            if p is None or p.stdout is None:
                raise BaseDisconnected("basesim not running")
            left = t_end - time.monotonic()
            if left <= 0:
                return None
            r, _, _ = select.select([p.stdout], [], [], left)
            if not r:
                return None
            chunk = os.read(p.stdout.fileno(), 65536)
            if not chunk:
                raise BaseDisconnected(f"basesim exited ({p.poll()})")
            self._buf += chunk

    def write(self, data: bytes) -> None:
        p = self.proc
        if p is None or p.stdin is None or p.poll() is not None:
            raise BaseDisconnected("basesim not running")
        try:
            with self._wlock:
                p.stdin.write(data)
                p.stdin.flush()
        except (BrokenPipeError, OSError, ValueError) as e:
            raise BaseDisconnected(f"basesim write failed: {e}") from e

    def close(self) -> None:
        p, self.proc = self.proc, None
        if p is None:
            return
        try:
            if p.stdin:
                p.stdin.close()
        except Exception:
            pass
        try:
            p.terminate()
            p.wait(3)
        except Exception:
            p.kill()
        try:
            if p.stdout:
                p.stdout.close()
        except Exception:
            pass

    # ---- scenario control (README.md): "#sim anchor 6.5", "#sim free", "#sim fault drumJam 1", "#sim set lineOut 7"
    def sim(self, command: str) -> None:
        self.write(b"#sim " + command.strip().encode() + b"\n")


def start_basesim(binary: Optional[str] = None, timeout: float = 1.0, wait: float = 10.0,
                  stderr_path: Optional[str] = None, **scenario) -> Tuple[subprocess.Popen, BaseClient]:
    """Start basesim (scenario: speed, line_out, anchor_taut_at, seat_at, load_cell) and a BaseClient on it.

    Returns (proc, client) once the first telemetry arrived. client.transport.sim("...") sends scenario commands;
    client.close() stops the simulator.
    """
    tr = SubprocessTransport(binary, stderr_path=stderr_path, **scenario)
    tr.open()
    client = BaseClient(tr, timeout=timeout, reconnect_s=0.2).start()
    try:
        if not client.wait_connected(wait):
            raise TimeoutError("basesim link did not come up")
        client.wait_tel(lambda t: True, wait, what="first telemetry from basesim")
    except Exception:
        client.close()
        raise
    return tr.proc, client
