import threading
import time

import numpy as np
import pytest

from grappstar.comms.dart import Dart, DartError, DartRefused, derive_phase, flightlog_arrays, remap_flightlog
from grappstar.comms.mock_dart import MockDart, synth_flightlog


@pytest.fixture
def md():
    m = MockDart(time_scale=5.0).start()
    yield m
    m.stop()


def test_status_v4_fields(md):
    d = Dart(md.host)
    s = d.status()
    assert s["phase"] == "IDLE" and "rail_V" in s
    for k in ("fw", "t_ms", "fault", "auto_on_landing", "ina_ok", "imu_ok", "t_launch_ms", "log_frozen", "reset"):
        assert k in s
    assert d.link()["connected"]


def test_derive_phase_old_firmware():
    assert derive_phase({"seq": "idle", "armed": True, "landed": False}) == "ARMED"
    assert derive_phase({"seq": "idle", "armed": True, "landed": True}) == "LANDED"
    assert derive_phase({"seq": "deploy + drills"}) == "ANCHORING"
    assert derive_phase({"seq": "anchored"}) == "ANCHORED"
    assert derive_phase({"seq": "released"}) == "IDLE"
    assert derive_phase({"seq": "aborted"}) == "FAULT"
    assert derive_phase({"seq": "x", "phase": "FLIGHT"}) == "FLIGHT"


def test_arm_log_only_then_land_then_auto(md):
    d = Dart(md.host)
    s = d.arm(True, auto=False)
    assert s["armed"] and s["phase"] == "ARMED" and not s["auto_on_landing"]
    md.launch()
    assert d.status()["phase"] == "FLIGHT"
    with pytest.raises(DartRefused) as ei:
        d.auto()                              # refused in flight
    assert ei.value.reason == "in flight"
    with pytest.raises(DartRefused):
        d.key("2", 350)
    md.land()
    assert d.status()["phase"] == "LANDED"   # log-only: it waits for the mission
    d.auto()
    s = d.wait_phase("ANCHORED", 5, poll=0.05)
    assert s["seq"] == "anchored"
    with pytest.raises(DartRefused) as ei:
        d.auto()
    assert "already anchored" in ei.value.reason
    fl = d.flightlog()
    a = flightlog_arrays(fl)
    assert fl["hz"] == 50 and fl["frozen"] and len(a["t"]) > 100
    assert max(a["amag"]) > 6 * 9.81


def test_auto_on_landing(md):
    d = Dart(md.host)
    d.arm(True, auto=True)
    md.launch(); md.land()
    assert d.status()["phase"] == "ANCHORING"
    d.wait_phase("ANCHORED", 5, poll=0.05)


def test_launch_ignored_when_not_armed(md):
    d = Dart(md.host)
    md.launch()
    assert d.status()["phase"] == "IDLE"
    assert d.flightlog()["samples"] == []


def test_release_ends_released_idle(md):
    d = Dart(md.host)
    d.auto(); d.wait_phase("ANCHORED", 5, poll=0.05)
    d.release()
    s = d.wait_phase("IDLE", 5, poll=0.05, want_seq="released")
    assert s["seq"] == "released"


def test_release_refused_while_anchoring(md):
    d = Dart(md.host)
    d.auto()
    with pytest.raises(DartRefused) as ei:
        d.release()
    assert ei.value.reason == "busy"


def test_stop_aborts_and_wait_fails(md):
    d = Dart(md.host)
    d.auto()
    threading.Timer(0.1, d.stop).start()
    with pytest.raises(DartError) as ei:
        d.wait_phase("ANCHORED", 5, poll=0.05)
    assert "aborted" in str(ei.value)
    s = d.status()
    assert s["phase"] == "IDLE" and s["seq"] == "aborted" and not s["armed"]


def test_anchor_fault(md):
    d = Dart(md.host)
    md.fail_anchor = True
    d.auto()
    with pytest.raises(DartError) as ei:
        d.wait_phase("ANCHORED", 5, poll=0.05)
    assert "NO_CURRENT" in str(ei.value)          # fw v4.3: the pods step's open-wire check (no kick on v34)


def test_hold_deadman_repeats_and_stops(md):
    d = Dart(md.host)
    d.hold(["#", "!"], 0.5)
    ks = [k for _, k, _ in md.keys]
    assert ks.count("#") >= 3 and ks.count("!") >= 3
    assert ks[-1] == " "


def test_hold_refused_does_not_stop_a_running_sequence(md):
    """review B3: a hold refused at its first key (an /auto running) must not send /stop, which would abort it."""
    d = Dart(md.host)
    d.auto()
    with pytest.raises(DartRefused):
        d.hold("2", 0.5)
    assert not any(path == "/stop" for _, path, _ in md.requests)
    assert d.status()["phase"] == "ANCHORING"
    d.wait_phase("ANCHORED", 10, poll=0.05)


def test_hold_limit_raises_but_stalled_is_a_normal_end(md):
    """fw-dart f5487cd: "hold limit: release the key first" (8 s manual cap) means the move is unfinished: raise.
    "stalled" stays the normal end at an end stop. Both still end with /stop?disarm=0."""
    for reason, raises in (("hold limit: release the key first", True), ("stalled: release the key first", False)):
        d = Dart(md.host)
        n = {"k": 0}
        real_key = d.key
        def key(c, ms, reason=reason):
            n["k"] += 1
            if n["k"] > 2:
                raise DartRefused("/key", reason)
            return real_key(c, ms)
        d.key = key
        before = sum(1 for _, p, _ in md.requests if p == "/stop")
        if raises:
            with pytest.raises(DartRefused):
                d.hold("2", 3.0)
        else:
            d.hold("2", 3.0)
        assert sum(1 for _, p, _ in md.requests if p == "/stop") == before + 1


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
    d.auto()
    md.down = True
    threading.Timer(0.5, lambda: setattr(md, "down", False)).start()
    s = d.wait_phase("ANCHORED", 5, poll=0.1)
    assert s["phase"] == "ANCHORED"


def test_clock_offset(md):
    d = Dart(md.host)
    assert d.clock_offset() is None
    for _ in range(5):
        d.status()
    off = d.clock_offset()
    # this mock's millis run 5x fast and /status is spaced 0.2 s (5 Hz rule), so its clock drifts 0.8 s per poll
    # against ours: only check an offset exists; test_clock_offset_real_time checks the accuracy
    assert off is not None and abs(off - md.t_boot) < 10.0


def test_clock_offset_real_time():
    m = MockDart(time_scale=1.0).start()
    try:
        d = Dart(m.host)
        for _ in range(8):
            d.status()
        assert abs(d.clock_offset() - m.t_boot) < 0.02
    finally:
        m.stop()


def test_synth_flightlog_shape():
    fl = synth_flightlog()
    a = flightlog_arrays(fl)
    g = 9.81
    t = a["t"]; m = a["amag"]
    tl, td = fl["t_launch_ms"] / 1000, fl["t_land_ms"] / 1000
    assert all(abs(x - g) < 0.5 for ti, x in zip(t, m) if ti < tl - 0.1)
    assert max(x for ti, x in zip(t, m) if tl <= ti < tl + 0.1) > 4 * g
    assert all(x < 0.3 * g for ti, x in zip(t, m) if tl + 0.1 < ti < td - 0.05)
    assert max(x for ti, x in zip(t, m) if td <= ti < td + 0.1) > 6 * g


def _rot(q):
    qw, qx, qy, qz = q
    return np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                     [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                     [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])


def test_remap_flightlog_identity_and_rotation():
    fl = synth_flightlog()
    assert remap_flightlog(fl, "x,y,z") is fl
    # chip mounted with chip x = body -y, chip y = body x (rotated 90 deg about z): body = (chip y, -chip x, z)
    M = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], float)
    chip = dict(fl)
    rows = []
    for r in fl["samples"]:
        Rb = _rot(r[1:5])
        Rc = Rb @ M                       # world from chip
        a_chip = M.T @ np.array(r[5:8])
        # quaternion of Rc
        w = np.sqrt(max(0, 1 + np.trace(Rc))) / 2
        x = (Rc[2, 1] - Rc[1, 2]) / (4 * w); y = (Rc[0, 2] - Rc[2, 0]) / (4 * w); z = (Rc[1, 0] - Rc[0, 1]) / (4 * w)
        rows.append([r[0], w, x, y, z] + list(a_chip))
    chip["samples"] = rows
    back = remap_flightlog(chip, "y,-x,z")
    for r0, r1 in zip(fl["samples"][::25], back["samples"][::25]):
        assert np.allclose(_rot(r0[1:5]), _rot(r1[1:5]), atol=1e-3)
        assert np.allclose(r0[5:8], r1[5:8], atol=0.02)
    with pytest.raises(ValueError):
        remap_flightlog(chip, "y,x,z")    # mirrored: not a rotation


def test_flightlog_retries_while_busy(md):
    d = Dart(md.host)
    d.arm(True, auto=True)
    md.launch(); md.land()                 # auto-anchor starts at once: /flightlog answers 409 busy
    with pytest.raises(DartRefused):
        d.flightlog(busy_wait_s=0)
    fl = d.flightlog(busy_wait_s=10)       # waits out the anchor sequence (3.4 s / scale 5)
    assert fl["samples"]


def test_cli_dart_release_after_stop_is_not_success(md, capsys):
    from grappstar import cli
    d = Dart(md.host)
    d.auto(); d.wait_phase("ANCHORED", 5, poll=0.05)
    threading.Timer(0.1, d.stop).start()
    rc = cli.main(["dart", "release", "--host", md.host])
    assert rc != 0 or "released" not in capsys.readouterr().out
