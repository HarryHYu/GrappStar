"""BaseClient against MockBase, both following firmware/base/PROTOCOL.md (fw base-1.0); notes in docs/C1_NOTES.md."""
import json
import os
import threading
import time

import pytest

from grappstar.comms.base import (BaseClient, BaseDisconnected, BaseError, BaseTimeout, pipe_pair, tel_seated,
                                  tel_carriage_home, SW_KEYS)
from grappstar.comms.mock_base import MockBase, connected_pair, serve_pty, World

TEL_KEYS = ("state", "mode", "drum_deg", "drum_pos", "wiper_deg", "rpm", "amps", "pitch_deg", "cock", "wraps",
            "line_m", "tension_n", "sw", "fault", "vbat")


@pytest.fixture
def pair():
    mb, c = connected_pair(time_scale=10.0)
    c.wait_tel(lambda t: True, 2)
    yield mb, c
    c.close()
    mb.stop()
    assert mb.tx_max <= 256, f"mock sent a {mb.tx_max} byte line"


def err_of(fn, *a, **kw):
    with pytest.raises(BaseError) as ei:
        fn(*a, **kw)
    return ei.value.err


def to_mode(c, to, timeout=10):
    m = c.event_mark()
    c.mode(to)
    e = c.wait_event("MODE", timeout, since=m)
    assert e["ok"] is True and e["mode"] == to and e["drum_pos"] == to
    c.wait_tel(lambda t: t["drum_pos"] == to and t["state"] == "IDLE" and t["fault"] is None, 3, fail_on_fault=False)
    return e


def aim(c, deg, timeout=10):
    m = c.event_mark()
    c.aim(deg)
    return c.wait_event("AIMED", timeout, since=m)


def cock(c, timeout=10):
    m = c.event_mark()
    c.cock()
    return c.wait_event("COCKED", timeout, since=m)


def wrap(c, timeout=10):
    m = c.event_mark()
    c.wrap()
    return c.wait_event("WRAPPED", timeout, since=m)


# ---------------------------------------------------------------- basics
def test_ping_status_and_telemetry(pair):
    mb, c = pair
    r = c.ping()
    assert r["ok"] and r["fw"] == "base-1.0" and isinstance(r["t"], int)
    t = c.wait_tel(lambda t: True, 2)
    assert set(TEL_KEYS) <= set(t)
    assert t["mode"] == "NEUTRAL" and t["drum_pos"] == "NEUTRAL" and t["state"] == "IDLE"
    assert t["fault"] is None and isinstance(t["wraps"], int) and t["cock"] == 0
    # sw: one-letter keys, closed contacts only: NEUTRAL drum code b, dart seated S, carriage home H
    assert set(t["sw"]) <= set(SW_KEYS) and all(v == 1 for v in t["sw"].values())
    assert t["sw"].get("b") == 1 and "a" not in t["sw"] and tel_seated(t) and tel_carriage_home(t)
    s = c.status()
    for k in ("phase", "aimed", "aim_known", "drum_known", "wraps_known", "magnet", "glitches", "hx", "peak_a", "fw"):
        assert k in s, k
    assert s["phase"] == "REST" and s["aimed"] is False and "tel" not in s


def test_line_length_limit_ids_and_parse_errors(pair):
    mb, c = pair
    for _ in range(5):
        c.ping()
    ids = [m["id"] for m in mb.received]
    assert len(set(ids)) == len(ids)
    with pytest.raises(ValueError):
        c.cmd("set", key="x" * 300, val=1)
    assert mb.handle_line(b"garbage") == '{"id":null,"ok":false,"err":"BAD_JSON"}'
    long = json.dumps({"id": 1, "cmd": "ping", "x": "a" * 260}).encode()
    assert json.loads(mb.handle_line(long))["err"] == "TOO_LONG"
    assert json.loads(mb.handle_line(b'{"id":2}'))["err"] == "NO_CMD"
    assert json.loads(mb.handle_line(b'{"id":"abc","cmd":"ping"}'))["id"] == "abc"
    # worst case telemetry still fits 256 bytes
    with mb.lock:
        mb.drum_deg, mb.wiper_deg, mb.rpm, mb.amps = -12345.67, -4096000.0, -400.0, 19.9
        mb.line_est, mb.tension, mb.fault = 99.99, 1234.5, "SWITCH_DISAGREE"
        assert len(mb.telemetry_line()) <= 256
        mb.fault = None


def test_id_null_reply_goes_to_the_waiting_command(pair):
    mb, c = pair
    mb.ignore.add("ping")
    threading.Timer(0.15, lambda: c.transport.write(b"not json\n")).start()
    r = c.ping(timeout=2, check=False)
    assert r["id"] is None and r["err"] == "BAD_JSON"


# ---------------------------------------------------------------- the shot
def test_shot_order_aim_cock_arm_fire(pair):
    mb, c = pair
    to_mode(c, "LAUNCH")
    assert err_of(c.arm) == "NOT_AIMED"
    e = aim(c, 30)
    assert e["ok"] and abs(e["pitch_deg"] - 30) < 0.01 and e["target_deg"] == 30
    assert abs(c.wait_tel(lambda t: t["state"] == "IDLE", 2)["pitch_deg"] - 30) < 0.01
    e = cock(c)
    assert e["ok"] and e["cock_deg"] == 560
    t = c.wait_tel(lambda t: t["state"] == "IDLE", 2)
    assert t["cock"] == 1 and not tel_carriage_home(t)
    assert c.status()["phase"] == "COCKED" and c.status()["aimed"] is True
    # cocked: the worm holds the spring
    assert err_of(c.aim, 40) == "NOT_AT_REST"
    assert err_of(c.mode, "NEUTRAL") == "LAUNCH_BUSY"
    assert err_of(c.jog, "wiper", 1, 300) == "WOULD_FIRE"
    assert err_of(c.jog, "wiper", -1, 300) == "NOT_AT_REST"
    m = c.event_mark()
    assert cock(c)["ok"]                      # cock again: COCKED at once
    tok = c.arm()
    assert isinstance(tok, int) and 100000 <= tok <= 999999
    m = c.event_mark()
    c.fire(tok)
    e = c.wait_event("FIRED", 5, since=m)
    assert e["pitch_deg"] == 30 and e["cock_deg"] == 580 and mb.t_fire is not None
    e = c.wait_event("RESET", 5, since=m)
    assert e["ok"] is True
    t = c.wait_tel(lambda t: t["state"] == "IDLE", 2)
    assert t["cock"] == 0 and t["line_m"] is None        # the payout is not measured
    s = c.status()
    assert s["phase"] == "REST" and s["aimed"] is False
    assert err_of(c.fire, tok) == "NOT_ARMED"            # single use
    assert err_of(c.arm) == "NOT_AIMED"                  # aim again after a shot


def test_fire_from_rest_runs_the_whole_cycle(pair):
    mb, c = pair
    to_mode(c, "LAUNCH")
    aim(c, 40)
    tok = c.arm()                                        # cock is optional
    m = c.event_mark()
    c.fire(tok)
    c.wait_tel(lambda t: t["state"] == "FIRE", 2)
    e = c.wait_event("FIRED", 10, since=m)
    assert e["pitch_deg"] == 40
    c.wait_event("RESET", 10, since=m)
    assert c.status()["phase"] == "REST"


def test_arm_refusals_and_tokens(pair):
    mb, c = pair
    to_mode(c, "LAUNCH")
    cock(c)
    assert err_of(c.arm) == "NOT_AIMED"       # cocked without aiming (aim needs the train at rest)
    m = c.event_mark()
    c.decock()
    assert c.wait_event("RESET", 10, since=m)["how"] == "decock"
    assert c.status()["aim_known"] is False   # decock turns the aim train
    assert aim(c, 35)["ok"]                   # aim homes on SW_RAIL_LOW first
    cock(c)
    with mb.lock:
        mb.world.dart, mb.line_m = "free", 3.0
    assert err_of(c.arm) == "NOT_SEATED"
    with mb.lock:
        mb.world.dart, mb.line_m = "seated", 0.0
    c.arm()
    assert err_of(c.fire, 1) == "BAD_TOKEN"
    tok = c.arm()
    assert err_of(c.fire, tok + 1) == "BAD_TOKEN"
    assert err_of(c.fire, tok) == "NOT_ARMED"          # the wrong token burnt the arm
    tok = c.arm()
    time.sleep(0.45)                                    # 4.5 s at time_scale 10
    assert err_of(c.fire, tok) == "NOT_ARMED"           # expired


def test_refusals_outside_the_right_mode(pair):
    mb, c = pair
    assert err_of(c.aim, 30) == "NOT_IN_LAUNCH"
    assert err_of(c.cock) == "NOT_IN_LAUNCH"
    assert err_of(c.arm) == "NOT_IN_LAUNCH"
    assert err_of(c.fire, 123456) == "NOT_ARMED"
    assert err_of(c.wrap) == "NOT_IN_REEL"
    assert err_of(c.reel, 0.5) == "NOT_WRAPPED"
    assert err_of(c.unwrap) == "NOT_WRAPPED"
    assert c.cmd("bogus", check=False)["err"] == "UNKNOWN_CMD"
    assert c.cmd("mode", to="SIDEWAYS", check=False)["err"] == "BAD_ARG"
    assert c.cmd("mode", check=False)["err"] == "BAD_ARG"
    to_mode(c, "LAUNCH")
    assert err_of(c.aim, 5) == "OUT_OF_RANGE"
    assert err_of(c.aim, 80) == "OUT_OF_RANGE"
    m = c.event_mark()
    c.mode("LAUNCH")                                    # already there: ok and MODE at once
    assert c.wait_event("MODE", 1, since=m)["ok"]
    m = c.event_mark()
    c.mode("REEL")
    assert err_of(c.mode, "LAUNCH") == "BUSY"
    assert err_of(c.aim, 30) == "BUSY"
    c.wait_event("MODE", 10, since=m)


def test_fresh_build_needs_cock_rest(tmp_path):
    mb, c = connected_pair(time_scale=10.0, fresh=True)
    try:
        c.wait_tel(lambda t: True, 2)
        to_mode(c, "LAUNCH")
        assert c.wait_tel(lambda t: True, 1)["pitch_deg"] is None
        assert err_of(c.aim, 30) == "PHASE_UNKNOWN"
        c.cal_cock_rest()
        assert aim(c, 30)["ok"]
    finally:
        c.close(); mb.stop()


# ---------------------------------------------------------------- wrap, reel, hold
def test_wrap_rules_tow_hold_and_retrieve(pair):
    mb, c = pair
    to_mode(c, "REEL")
    assert err_of(c.reel, 0.5) == "NOT_WRAPPED"
    m = c.event_mark()
    c.wrap()
    assert err_of(c.mode, "LAUNCH") == "BUSY"
    c.wait_tel(lambda t: t["state"] == "WRAP" and t["drum_pos"] == "BETWEEN" and t["mode"] == "REEL", 3)
    e = c.wait_event("WRAPPED", 10, since=m)
    assert e["ok"] and e["wraps"] == 4
    t = c.wait_tel(lambda t: t["state"] == "IDLE", 2)
    assert t["drum_pos"] == "WRAPPED" and t["mode"] == "REEL" and t["wraps"] == 4
    assert err_of(c.mode, "LAUNCH") == "WRAPS_ON"
    m = c.event_mark()
    c.wrap()                                            # already wrapped: WRAPPED at once
    assert c.wait_event("WRAPPED", 1, since=m)["wraps"] == 4
    assert c.cmd("reel", duty=1.5, check=False)["err"] == "BAD_ARG"
    assert c.cmd("reel", duty=0.5, until="forever", check=False)["err"] == "BAD_ARG"
    assert c.cmd("reel", duty=0.5, max_n=0, check=False)["err"] == "BAD_ARG"
    assert err_of(c.jog, "wiper", -1, 300) == "BACKSTOP"
    # tow against an anchored dart: LIMIT, HOLD
    with mb.lock:
        mb.world.dart, mb.line_m, mb.world.slack_m = "anchored", 6.0, 0.3
    c.set("line_m", 6.0)
    m = c.event_mark()
    c.reel(1.0, max_n=30, until="tension")
    e = c.wait_event("LIMIT", 20, since=m)
    assert e["tension_n"] >= 30 and e["max_n"] == 30 and e["holding"] is True and e["why"] == "tension limit"
    t = c.wait_tel(lambda t: t["state"] == "HOLD", 2)
    assert t["tension_n"] >= 30 and t["rpm"] == 0 and t["line_m"] < 6.0
    assert err_of(c.unwrap) == "HOLDING"
    assert err_of(c.mode, "NEUTRAL") == "HOLDING"
    m = c.event_mark()
    c.stop()
    e = c.wait_event("ABORTED", 2, since=m)
    assert e["what"] == "HOLD" and e["why"] == "stop"
    # released dart: retrieve until seated
    with mb.lock:
        mb.world.dart = "free"
    m = c.event_mark()
    c.reel(1.0, max_n=30, until="seated")
    c.wait_event("SEATED", 40, since=m)
    t = c.wait_tel(lambda t: t["state"] == "IDLE", 2)
    assert t["line_m"] == 0 and tel_seated(t)
    m = c.event_mark()
    c.unwrap()
    assert c.wait_event("UNWRAPPED", 10, since=m)["ok"]
    m = c.event_mark()
    c.unwrap()                                          # at REEL with no wraps: UNWRAPPED at once
    c.wait_event("UNWRAPPED", 1, since=m)
    to_mode(c, "NEUTRAL")


def test_reel_until_stop_limit_does_not_hold_and_hold_resumes_on_slack(pair):
    mb, c = pair
    to_mode(c, "REEL")
    wrap(c)
    with mb.lock:
        mb.world.dart, mb.line_m, mb.world.slack_m = "anchored", 5.0, 0.0
    m = c.event_mark()
    c.reel(1.0, max_n=20, until="stop")
    e = c.wait_event("LIMIT", 10, since=m)
    assert e["holding"] is False
    c.wait_tel(lambda t: t["state"] == "IDLE", 2)
    # until tension holds; slack in the line (load cell) winds again, then holds again
    m = c.event_mark()
    c.reel(1.0, max_n=20, until="tension")
    c.wait_event("LIMIT", 10, since=m)
    c.wait_tel(lambda t: t["state"] == "HOLD", 2)
    m = c.event_mark()
    with mb.lock:
        mb.world.slack_m = 0.3                          # the rover drove toward the dart
    c.wait_tel(lambda t: t["state"] == "REEL", 3)
    assert c.wait_event("LIMIT", 10, since=m)["holding"] is True


def test_stop_during_reel_raises_aborted_in_wait(pair):
    mb, c = pair
    to_mode(c, "REEL")
    wrap(c)
    with mb.lock:
        mb.world.dart, mb.line_m = "free", 20.0
    m = c.event_mark()
    c.reel(0.5, until="seated")
    time.sleep(0.1)
    c.stop()
    with pytest.raises(BaseError) as ei:
        c.wait_event("SEATED", 3, since=m)
    assert ei.value.err == "ABORTED" and ei.value.reply["what"] == "REEL"


def test_seat_ends_reel_only_until_seated_when_firmware_exact(tmp_path):
    mb, c = connected_pair(time_scale=10.0, seat_ends_any_reel=False)
    try:
        c.wait_tel(lambda t: True, 2)
        to_mode(c, "REEL")
        wrap(c)
        with mb.lock:
            mb.world.dart, mb.line_m = "free", 1.0
        m = c.event_mark()
        c.reel(1.0, max_n=15, until="tension")
        e = c.wait_event(("LIMIT", "SEATED"), 20, since=m)
        assert e["evt"] == "LIMIT" and tel_seated(c.tel)  # wound on against the cradle
    finally:
        c.close(); mb.stop()


def test_seat_ends_any_reel_by_default(pair):
    mb, c = pair
    to_mode(c, "REEL")
    wrap(c)
    with mb.lock:
        mb.world.dart, mb.line_m = "free", 1.0
    m = c.event_mark()
    c.reel(1.0, max_n=15, until="tension")
    assert c.wait_event(("LIMIT", "SEATED"), 20, since=m)["evt"] == "SEATED"


# ---------------------------------------------------------------- faults, stop, estop
def test_fault_safes_to_neutral_and_clear(pair):
    mb, c = pair
    to_mode(c, "LAUNCH")
    m = c.event_mark()
    mb.inject_fault("STALL")
    with pytest.raises(BaseError) as ei:
        c.wait_event("AIMED", 3, since=m)
    assert ei.value.err == "STALL" and ei.value.reply["why"]
    c.wait_event("STALL", 1, since=m, fail_on_fault=False)
    e = c.wait_event("MODE", 5, since=m, fail_on_fault=False)
    assert e["ok"] and e["mode"] == "NEUTRAL"
    t = c.wait_tel(lambda t: t["state"] == "FAULT" and t["drum_pos"] == "NEUTRAL", 5, fail_on_fault=False)
    assert t["fault"] == "STALL"
    assert err_of(c.mode, "LAUNCH") == "FAULTED"
    c.clear()
    c.wait_tel(lambda t: t["fault"] is None and t["state"] == "IDLE", 2, fail_on_fault=False)
    to_mode(c, "LAUNCH")
    mb.inject_fault("ANGLE_SENSOR", present=True)
    c.wait_tel(lambda t: t["state"] == "FAULT", 5, fail_on_fault=False)
    assert err_of(c.clear) == "STILL_PRESENT"
    mb.fault_present = False
    c.clear()


def test_fault_while_cocking_holds_the_drum_in_launch(pair):
    mb, c = pair
    to_mode(c, "LAUNCH")
    aim(c, 30)
    mb.fail_next["cock"] = "STALL"
    m = c.event_mark()
    c.cock()
    with pytest.raises(BaseError) as ei:
        c.wait_event("COCKED", 10, since=m)
    assert ei.value.err == "STALL"
    with pytest.raises(BaseError) as ei:
        c.wait_event("MODE", 2, since=m, fail_on_fault=False)
    assert ei.value.err == "DRUM_HELD" and "launch train" in ei.value.reply["why"]
    t = c.wait_tel(lambda t: t["state"] == "FAULT", 2, fail_on_fault=False)
    assert t["drum_pos"] == "LAUNCH" and 0 < t["cock"] < 1
    c.clear()
    assert cock(c)["ok"]                                # cocking resumes from where it stopped


def test_fail_next_drum_timeout(pair):
    mb, c = pair
    mb.fail_next["mode"] = "DRUM_TIMEOUT"
    m = c.event_mark()
    c.mode("REEL")
    with pytest.raises(BaseError) as ei:
        c.wait_event("MODE", 5, since=m)
    assert ei.value.err == "DRUM_TIMEOUT"
    t = c.wait_tel(lambda t: t["state"] == "FAULT", 2, fail_on_fault=False)
    assert t["drum_pos"] == "BETWEEN" and t["mode"] == "NONE"
    c.clear()
    to_mode(c, "REEL")                                  # can continue from between


def test_stop_mid_cock_then_decock(pair):
    mb, c = pair
    to_mode(c, "LAUNCH")
    m = c.event_mark()
    c.cock()
    c.wait_tel(lambda t: (t["cock"] or 0) > 0.3, 5)
    c.stop()
    e = c.wait_event("ABORTED", 2, since=m)
    assert e["what"] == "COCK" and e["why"] == "stop"
    assert c.status()["phase"] == "UNKNOWN"
    assert err_of(c.cock) == "LOADED_UNKNOWN"
    assert err_of(c.mode, "NEUTRAL") == "LAUNCH_BUSY"
    assert err_of(c.aim, 30) == "PHASE_UNKNOWN"
    assert err_of(c.arm) == "NOT_COCKED"
    m = c.event_mark()
    c.decock()
    c.wait_event("RESET", 10, since=m)
    assert c.status()["phase"] == "REST"
    to_mode(c, "NEUTRAL")


def test_estop(pair):
    mb, c = pair
    to_mode(c, "REEL")
    m = c.event_mark()
    c.estop()
    e = c.wait_event("MODE", 5, since=m)
    assert e["mode"] == "NEUTRAL" and e["ok"]
    to_mode(c, "REEL")
    wrap(c)
    m = c.event_mark()
    c.estop()
    with pytest.raises(BaseError) as ei:
        c.wait_event("MODE", 2, since=m)
    assert ei.value.err == "DRUM_HELD"
    assert c.wait_tel(lambda t: True, 1)["drum_pos"] == "WRAPPED"


def test_jog_and_settings(pair):
    mb, c = pair
    assert c.cmd("jog", motor="arm", dir=1, check=False)["err"] == "BAD_ARG"
    assert c.cmd("jog", motor="drum", dir=2, check=False)["err"] == "BAD_ARG"
    c.jog("drum", 1, 1000)
    t = c.wait_tel(lambda t: t["state"] == "IDLE" and t["drum_pos"] == "BETWEEN", 3)
    assert t["mode"] == "NONE" and abs(t["drum_deg"] - 50.8) < 1.5
    c.set("limit_n", 30)
    assert c.get("limit_n")["val"] == 30
    assert err_of(c.set, "limit_n", 900) == "OUT_OF_RANGE"
    assert err_of(c.set, "nope", 1) == "BAD_KEY"
    r = c.get("pitch_range")
    assert r["min"] == 15 and r["max"] == 60
    assert c.get("fw")["val"] == "base-1.0"
    c.set("line_m", 7.5)
    assert c.get("line_m")["val"] == 7.5
    c.set("require_seated", False)
    assert c.get("require_seated")["val"] is False
    c.cal_aim_forget()
    assert c.status()["aim_known"] is False
    m = c.event_mark()
    c.cal_drum()                                         # the drum code is absolute: only reports
    assert c.wait_event("MODE", 1, since=m)["drum_pos"] == "BETWEEN"


def test_invalid_drum_code(pair):
    mb, c = pair
    with mb.lock:
        mb.code_bad = True
    t = c.wait_tel(lambda t: t["drum_pos"] == "UNKNOWN", 2)
    assert t["mode"] == "NONE"
    assert err_of(c.mode, "LAUNCH") == "POSITION_UNKNOWN"
    assert err_of(c.cal_drum) == "POSITION_UNKNOWN"
    mb.code_bad = False


def test_256_byte_line_is_legal(pair):
    mb, c = pair
    base = len(json.dumps({"id": 1000000, "cmd": "ping", "x": ""}, separators=(",", ":")))
    n = 256 - base
    r = c.cmd("ping", x="a" * (n - (len(str(c._next_id)) - 7)))
    assert r["ok"]
    assert max(len(json.dumps(m, separators=(",", ":"))) for m in mb.received) == 256
    with pytest.raises(ValueError):
        c.cmd("ping", x="a" * 300)


def test_hello_fails_pending_commands_and_waits(pair):
    mb, c = pair
    mb.ignore.add("ping")
    threading.Timer(0.15, lambda: mb._send({"evt": "HELLO", "fw": "base-1.0", "reset": "WATCHDOG"})).start()
    with pytest.raises(BaseError) as ei:
        c.ping(timeout=2)
    assert ei.value.err == "BASE_RESET"
    mb.ignore.clear()
    m = c.event_mark()
    threading.Timer(0.1, lambda: mb._send({"evt": "HELLO", "fw": "base-1.0", "reset": "BROWNOUT"})).start()
    with pytest.raises(BaseError) as ei:
        c.wait_event("MODE", 2, since=m)
    assert ei.value.err == "BASE_RESET"


def test_host_watchdog_stops_a_reel(pair):
    mb, c = pair
    to_mode(c, "REEL")
    wrap(c)
    with mb.lock:
        mb.world.dart, mb.line_m = "free", 30.0
    c.set("host_timeout_s", 2)
    m = c.event_mark()
    c.reel(0.5)
    e = c.wait_event("ABORTED", 3, since=m, fail_on_abort=False)
    assert e["what"] == "HOST" and e["why"] == "host silent"


def test_reel_defaults_and_from_hold(pair):
    mb, c = pair
    to_mode(c, "REEL")
    wrap(c)
    with mb.lock:
        mb.world.dart, mb.line_m, mb.world.slack_m = "anchored", 5.0, 0.0
    m = c.event_mark()
    c.cmd("reel", until="tension")                       # defaults: duty 0.86, max_n limit_n 20
    e = c.wait_event("LIMIT", 10, since=m)
    assert e["max_n"] == 20 and e["holding"]
    c.wait_tel(lambda t: t["state"] == "HOLD", 2)
    m = c.event_mark()
    c.reel(1.0, max_n=30, until="tension")               # allowed from HOLD: winds again
    assert c.wait_event("LIMIT", 10, since=m)["max_n"] == 30


# ---------------------------------------------------------------- link
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
    to_mode(c, "LAUNCH")
    assert got_t and any(e["evt"] == "MODE" and isinstance(e["t"], int) for e in got_e)


def test_telemetry_rate_doubles_while_moving():
    mb, c = connected_pair(time_scale=1.0)
    try:
        n = []
        c.add_tel_callback(lambda t: n.append(time.time()))
        time.sleep(1.0)
        idle = len(n)
        n.clear()
        c.mode("LAUNCH")   # drum moves 40 deg at 10.8 deg/s: 3.7 s
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
