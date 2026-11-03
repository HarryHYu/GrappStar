import json
import os
import time

import pytest

from grappstar.comms.base import BaseClient, BaseDisconnected, BaseError, BaseTimeout, pipe_pair
from grappstar.comms.mock_base import MockBase, connected_pair, serve_pty, World


@pytest.fixture
def pair():
    mb, c = connected_pair(time_scale=10.0)
    yield mb, c
    c.close()
    mb.stop()


def test_ping_and_telemetry(pair):
    mb, c = pair
    assert c.ping()["ok"]
    t = c.wait_tel(lambda t: True, 2.0)
    for k in ("state", "mode", "drum_deg", "drum_pos", "wiper_deg", "rpm", "amps", "pitch_deg", "cock", "wraps",
              "line_m", "tension_n", "sw", "fault", "vbat"):
        assert k in t, k
    assert t["mode"] == "NEUTRAL" and t["drum_pos"] == "NEUTRAL"


def test_line_length_limit_and_ids(pair):
    mb, c = pair
    ids = set()
    for _ in range(5):
        c.ping()
    for m in mb.received:
        assert len(json.dumps(m)) < 256
        ids.add(m["id"])
    assert len(ids) == len(mb.received)
    with pytest.raises(ValueError):
        c.cmd("set", key="x" * 300, val=1)


def test_launch_sequence(pair):
    mb, c = pair
    m = c.event_mark()
    c.mode("LAUNCH")
    c.wait_event("MODE", 5, since=m)
    c.wait_tel(lambda t: t["drum_pos"] == "LAUNCH" and t["state"] == "IDLE", 3)
    m = c.event_mark()
    c.aim(30)
    e = c.wait_event("AIMED", 10, since=m)
    assert abs(e["deg"] - 30) < 0.01
    tok = c.arm()
    m = c.event_mark()
    c.fire(tok)
    c.wait_event("COCKED", 5, since=m)
    e = c.wait_event("FIRED", 5, since=m)
    assert e["pitch_deg"] == 30
    c.wait_tel(lambda t: t["line_m"] > 7.0, 5)


def test_refusals(pair):
    mb, c = pair
    with pytest.raises(BaseError) as ei:
        c.aim(30)
    assert ei.value.err == "NOT_IN_LAUNCH"
    with pytest.raises(BaseError) as ei:
        c.wrap()
    assert ei.value.err == "NOT_IN_REEL"
    with pytest.raises(BaseError) as ei:
        c.fire("123456")
    assert ei.value.err == "NOT_IN_LAUNCH"
    r = c.cmd("bogus", check=False)
    assert r["err"] == "UNKNOWN_CMD"


def test_bad_token_and_expiry(pair):
    mb, c = pair
    m = c.event_mark()
    c.mode("LAUNCH"); c.wait_event("MODE", 5, since=m)
    c.arm()
    with pytest.raises(BaseError) as ei:
        c.fire("nope")
    assert ei.value.err == "BAD_TOKEN"
    tok = c.arm()
    time.sleep(0.5)  # 5 s at time_scale 10
    with pytest.raises(BaseError) as ei:
        c.fire(tok)
    assert ei.value.err == "TOKEN_EXPIRED"


def test_wrap_rules_and_reel(pair):
    mb, c = pair
    m = c.event_mark()
    c.mode("REEL"); c.wait_event("MODE", 5, since=m)
    with pytest.raises(BaseError) as ei:
        c.reel(0.5, until="stop")
    assert ei.value.err == "NOT_WRAPPED"
    m = c.event_mark()
    c.wrap(); c.wait_event("WRAPPED", 5, since=m)
    with pytest.raises(BaseError) as ei:
        c.mode("LAUNCH")
    assert ei.value.err == "WRAPPED"
    # tow against an anchored dart
    mb.world.dart = "anchored"; mb.line_m = 6.0
    m = c.event_mark()
    c.reel(1.0, max_n=30, until="tension")
    e = c.wait_event("LIMIT", 20, since=m)
    assert e["tension_n"] >= 30
    t = c.wait_tel(lambda t: t["state"] == "HOLD", 2)
    assert t["tension_n"] >= 30
    with pytest.raises(BaseError) as ei:
        c.unwrap()
    assert ei.value.err == "UNDER_LOAD"
    # released dart: retrieve until seated
    mb.world.dart = "free"
    m = c.event_mark()
    c.reel(1.0, max_n=30, until="seated")
    c.wait_event("SEATED", 30, since=m)
    m = c.event_mark()
    c.unwrap(); c.wait_event("UNWRAPPED", 5, since=m)
    m = c.event_mark()
    c.mode("NEUTRAL"); c.wait_event("MODE", 5, since=m)


def test_fault_goes_neutral_and_clear(pair):
    mb, c = pair
    m = c.event_mark()
    c.mode("LAUNCH"); c.wait_event("MODE", 5, since=m)
    m = c.event_mark()
    mb.inject_fault("STALL")
    with pytest.raises(BaseError) as ei:
        c.wait_event("AIMED", 3, since=m)
    assert ei.value.err == "STALL"
    t = c.wait_tel(lambda t: t["drum_pos"] == "NEUTRAL", 5, fail_on_fault=False)
    assert t["fault"] == "STALL" and t["mode"] == "NEUTRAL"
    with pytest.raises(BaseError) as ei:
        c.mode("LAUNCH")
    assert ei.value.err == "FAULT"
    c.clear()
    c.wait_tel(lambda t: t["fault"] is None, 2, fail_on_fault=False)
    c.mode("LAUNCH")


def test_fail_next_drum_timeout(pair):
    mb, c = pair
    mb.fail_next["mode"] = "DRUM_TIMEOUT"
    m = c.event_mark()
    c.mode("REEL")
    with pytest.raises(BaseError) as ei:
        c.wait_event("MODE", 5, since=m)
    assert ei.value.err == "DRUM_TIMEOUT"
    assert c.wait_tel(lambda t: True, 1)["drum_pos"] == "BETWEEN"


def test_timeout_when_no_reply(pair):
    mb, c = pair
    mb.ignore.add("ping")
    with pytest.raises(BaseTimeout):
        c.ping(timeout=0.3)
    mb.ignore.clear()
    assert c.ping()["ok"]


def test_disconnect_and_reconnect():
    a, b = pipe_pair()
    mb = MockBase(a, time_scale=10).start()
    c = BaseClient(b, reconnect_s=0.05).start()
    try:
        assert c.wait_connected(2)
        assert c.ping()["ok"]
        b.broken = True           # cable pulled
        t0 = time.time()
        while c.connected and time.time() - t0 < 2:
            time.sleep(0.01)
        assert not c.connected
        with pytest.raises(BaseDisconnected):
            c.ping()
        b.broken = False          # plugged back in
        assert c.wait_connected(2)
        assert c.ping()["ok"]
        assert c.reconnects >= 1
    finally:
        c.close(); mb.stop()


def test_garbage_lines_ignored(pair):
    mb, c = pair
    mb.tr.write(b"ESP-ROM:esp32s3-20210327\n{not json\n[1,2]\n")
    time.sleep(0.1)
    assert c.ping()["ok"]
    assert c.errors >= 1


def test_callbacks(pair):
    mb, c = pair
    got_t, got_e = [], []
    c.add_tel_callback(got_t.append)
    c.add_evt_callback(got_e.append)
    m = c.event_mark()
    c.mode("LAUNCH"); c.wait_event("MODE", 5, since=m)
    assert got_t and any(e["evt"] == "MODE" for e in got_e)


def test_telemetry_rate_doubles_while_moving():
    mb, c = connected_pair(time_scale=1.0)
    try:
        n = []
        c.add_tel_callback(lambda t: n.append(time.time()))
        time.sleep(1.0)
        idle = len(n)
        n.clear()
        c.mode("LAUNCH")   # drum moves 90 deg at 120 deg/s: 0.75 s
        time.sleep(0.6)
        moving = len(n) / 0.6
        assert 7 <= idle <= 13
        assert moving >= 14
    finally:
        c.close(); mb.stop()


@pytest.mark.skipif(not hasattr(os, "openpty"), reason="no pty")
def test_pty_with_real_serial_transport():
    pytest.importorskip("serial")
    mb = serve_pty(time_scale=10)
    c = BaseClient(mb.pty_path, reconnect_s=0.1).start()
    try:
        assert c.wait_connected(3)
        assert c.ping(timeout=2)["ok"]
        assert c.wait_tel(lambda t: True, 2)["mode"] == "NEUTRAL"
    finally:
        c.close(); mb.stop()
