import threading
import time

import pytest

from grappstar.comms.dart import Dart, DartError, derive_phase, flightlog_arrays
from grappstar.comms.mock_dart import MockDart, synth_flightlog


@pytest.fixture
def md():
    m = MockDart(time_scale=5.0).start()
    yield m
    m.stop()


def test_status_has_phase(md):
    d = Dart(md.host)
    s = d.status()
    assert s["phase"] == "IDLE" and "rail_V" in s
    assert d.link()["connected"]


def test_derive_phase_old_firmware():
    assert derive_phase({"seq": "idle", "armed": True, "landed": False}) == "ARMED"
    assert derive_phase({"seq": "idle", "armed": True, "landed": True}) == "LANDED"
    assert derive_phase({"seq": "deploy + drills"}) == "ANCHORING"
    assert derive_phase({"seq": "anchored"}) == "ANCHORED"
    assert derive_phase({"seq": "aborted"}) == "FAULT"
    assert derive_phase({"seq": "x", "phase": "FLIGHT"}) == "FLIGHT"


def test_arm_land_auto_anchor(md):
    d = Dart(md.host)
    assert d.arm_landing()
    assert d.arm_landing()          # second call must not toggle it off
    assert d.status()["phase"] == "ARMED"
    md.launch()
    assert d.status()["phase"] == "FLIGHT"
    md.land()
    s = d.wait_phase("ANCHORED", 5, poll=0.05)
    assert s["seq"] == "anchored"
    fl = d.flightlog()
    a = flightlog_arrays(fl)
    assert fl["hz"] == 50 and len(a["t"]) > 100
    assert max(a["amag"]) > 6 * 9.81


def test_release(md):
    d = Dart(md.host)
    md.land(); d.auto(); d.wait_phase("ANCHORED", 5, poll=0.05)
    d.release()
    s = d.wait_phase("IDLE", 5, poll=0.05, fail=("FAULT",))
    assert s["event"] == "released"


def test_anchor_failure_raises(md):
    d = Dart(md.host)
    md.fail_anchor = True
    md.land(); d.auto()
    with pytest.raises(DartError):
        d.wait_phase("ANCHORED", 5, poll=0.05)


def test_hold_deadman_repeats_and_stops(md):
    d = Dart(md.host)
    d.hold(["#", "!"], 0.5)
    ks = [k for _, k, _ in md.keys]
    assert ks.count("#") >= 3 and ks.count("!") >= 3
    assert ks[-1] == " "             # always ends with stop


def test_hold_cancel(md):
    d = Dart(md.host)
    ev = threading.Event()
    threading.Timer(0.2, ev.set).start()
    t0 = time.time()
    d.hold("2", 5.0, cancel=ev)
    assert time.time() - t0 < 1.0


def test_unreachable():
    d = Dart("127.0.0.1:1", timeout=0.2, retries=0)
    with pytest.raises(DartError):
        d.status()
    assert not d.safe_stop()
    assert not d.link()["connected"]


def test_wait_phase_tolerates_short_outage(md):
    d = Dart(md.host, timeout=0.3, retries=0)
    md.down = True
    threading.Timer(0.5, lambda: setattr(md, "down", False)).start()
    md.land(); md._start_seq("anchor")
    s = d.wait_phase("ANCHORED", 5, poll=0.1)
    assert s["phase"] == "ANCHORED"


def test_flightlog_missing(md):
    d = Dart(md.host)
    with pytest.raises(DartError):
        d.flightlog()


def test_synth_flightlog_shape():
    fl = synth_flightlog()
    a = flightlog_arrays(fl)
    # on the rail about 1 g, launch spike over 4 g, free fall under 0.3 g, impact over 6 g
    g = 9.81
    t = a["t"]; m = a["amag"]
    tl, td = fl["t_launch_ms"] / 1000, fl["t_land_ms"] / 1000
    assert all(abs(x - g) < 0.5 for ti, x in zip(t, m) if ti < tl - 0.1)
    assert max(x for ti, x in zip(t, m) if tl <= ti < tl + 0.1) > 4 * g
    assert all(x < 0.3 * g for ti, x in zip(t, m) if tl + 0.1 < ti < td - 0.05)
    assert max(x for ti, x in zip(t, m) if td <= ti < td + 0.1) > 6 * g
