"""The Jetson dart client against the REAL dart firmware (fw-dart's host simulator, firmware/dart/sim/kedge_sim).

Skipped when the simulator binary is not available. Build it with: cd firmware/dart/sim && make
(or set GRAPPSTAR_DART_SIM to the kedge_sim path). Runs in real time: about 40 s.
"""
import os
import socket
import subprocess
import time
import urllib.request

import pytest

from grappstar.comms.dart import Dart, DartError, DartRefused, flightlog_arrays

HERE = os.path.dirname(os.path.abspath(__file__))
CANDIDATES = [os.environ.get("GRAPPSTAR_DART_SIM", ""),
              os.path.normpath(os.path.join(HERE, "..", "..", "firmware", "dart", "sim", "kedge_sim")),
              "/tmp/claude-0/dartsim/firmware/dart/sim/kedge_sim"]
SIM = next((p for p in CANDIDATES if p and os.path.exists(p)), None)
pytestmark = [pytest.mark.slow, pytest.mark.skipif(SIM is None, reason="dart firmware simulator not built")]


def _free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture(scope="module")
def sim():
    port = _free_port()
    p = subprocess.Popen([SIM, "--port", str(port)], stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL)
    d = Dart(f"127.0.0.1:{port}", timeout=1.0)
    t0 = time.time()
    while True:
        try:
            d.status()
            break
        except DartError:
            if time.time() - t0 > 10:
                p.kill()
                pytest.skip("simulator did not start")
            time.sleep(0.2)
    yield d, port
    p.terminate()
    p.wait(5)


def _sim(port, path):
    return urllib.request.urlopen(f"http://127.0.0.1:{port}{path}", timeout=2).read()


def test_status_fields_and_clock(sim):
    d, port = sim
    s = d.status()
    assert s["fw"].startswith("kedge-dart") and s["phase"] == "IDLE"
    for _ in range(5):
        d.status()
    assert d.clock_offset() is not None


def test_full_cycle_log_only_arm(sim):
    d, port = sim
    s = d.arm(True, auto=False)
    assert s["armed"] and not s["auto_on_landing"] and s["phase"] == "ARMED"
    with pytest.raises(DartRefused):
        d.flightlog()                     # refused while armed (v4)
    _sim(port, "/sim/launch?push_ms=40&flight_ms=900&impact=1")
    s = d.wait_phase(("LANDED",), 10, poll=0.2)
    assert s["t_launch_ms"] is not None and s["t_land_ms"] is not None
    fl = d.flightlog()
    a = flightlog_arrays(fl)
    assert fl["landing"] in ("impact", "still") and len(a["t"]) > 50
    assert max(a["amag"]) > 4 * 9.81
    d.auto()
    d.auto()                              # a repeat is accepted and does not restart
    s = d.wait_phase(("ANCHORED",), 40, poll=0.2)
    assert s["seq"] == "anchored"
    with pytest.raises(DartRefused) as ei:
        d.auto()
    assert "anchored" in ei.value.reason
    d.release()
    s = d.wait_phase(("IDLE",), 45, poll=0.2, want_seq="released")
    assert s["seq"] == "released"


def test_stop_during_anchor_is_aborted_not_fault(sim):
    d, port = sim
    d.auto()
    time.sleep(0.5)
    d.stop()
    s = d.status()
    assert s["seq"] == "aborted" and s["phase"] != "FAULT"
    with pytest.raises(DartError):
        d.wait_phase(("ANCHORED",), 2, poll=0.2)


def test_key_refused_in_flight(sim):
    d, port = sim
    d.arm(True, auto=False)
    _sim(port, "/sim/launch?push_ms=40&flight_ms=1500&impact=1")
    t0 = time.time()
    while d.status()["phase"] != "FLIGHT" and time.time() - t0 < 2:
        time.sleep(0.05)
    with pytest.raises(DartRefused) as ei:
        d.key("2", 350)
    assert "flight" in ei.value.reason
    d.wait_phase(("LANDED",), 10, poll=0.2)
