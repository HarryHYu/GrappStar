"""BaseClient against MockBase. The mock follows the real firmware (base_logic.h, fw base-1.2) and the one C1
sequence of BASE_SPEC 10.1 (D35); notes in docs/C1_NOTES.md.

The last test runs the same scripted 10.1 sequence against basesim (the real firmware logic, jetson/tools/basesim) and
against the mock and compares the event names and the final telemetry; it is skipped when basesim cannot be built.
"""
import json
import os
import threading
import time

import pytest

from grappstar.comms.base import BaseClient, BaseDisconnected, BaseError, BaseTimeout, pipe_pair, tel_seated, \
    tel_carriage_home
from grappstar.comms.mock_base import MockBase, connected_pair, serve_pty, World, SW_KEYS, DRUM_DEG, FW

SCALE = 20.0
TEL_KEYS = ("state", "mode", "drum_deg", "drum_pos", "wiper_deg", "rpm", "amps", "pitch_deg", "cock", "wraps",
            "line_m", "tension_n", "sw", "fault", "vbat")


def W(sim_s, scale=SCALE):
    """Wall seconds to wait for sim_s simulated seconds, with slack."""
    return sim_s / scale * 1.5 + 1.0


@pytest.fixture
def pair():
    mb, c = connected_pair(time_scale=SCALE)
    c.wait_tel(lambda t: True, 2)
    yield mb, c
    c.close()
    mb.stop()
    assert mb.tx_max <= 256, f"mock sent a {mb.tx_max} byte line"


def err_of(fn, *a, **kw):
    with pytest.raises(BaseError) as ei:
        fn(*a, **kw)
    return ei.value.err


def run(c, name, evt, sim_s, **args):
    """Send an action, expect ok, wait for its completion event."""
    m = c.event_mark()
    c.cmd(name, **args)
    return c.wait_event(evt, W(sim_s), since=m)


def idle(c, sim_s=2):
    return c.wait_tel(lambda t: t["state"] == "IDLE", W(sim_s), what="IDLE", fail_on_fault=False)


def to_mode(c, to, sim_s=60):
    e = run(c, "mode", "MODE", sim_s, to=to)
    assert e["ok"] is True and e["mode"] == to and e["drum_pos"] == to
    c.wait_tel(lambda t: t["drum_pos"] == to and t["state"] == "IDLE", 3, fail_on_fault=False)
    return e


def names_since(c, mark):
    return [e["evt"] for e in c.events(mark)]


def wrapped_park(c):
    """mode NEUTRAL (if needed), wrap: the drum parks at WRAPPED 314 with the collar NEUTRAL."""
    e = run(c, "wrap", "WRAPPED", 60)
    assert e["ok"] and e["wraps"] == 7
    t = idle(c)
    assert t["drum_pos"] == "WRAPPED" and t["mode"] == "NEUTRAL" and t["drum_deg"] == 314
    return t


# ---------------------------------------------------------------- basics
def test_ping_status_and_telemetry(pair):
    mb, c = pair
    r = c.ping()
    assert r["ok"] and r["fw"] == "base-1.3.0" and isinstance(r["t"], int)
    assert any(e["evt"] == "HELLO" and e["fw"] == FW for e in c.events())
    boot = next(e for e in c.events() if e["evt"] == "BOOT")
    assert boot["fw"] == "base-1.3.0" and boot["angle"] == "AS5600" and boot["phase"] == "REST"
    t = c.wait_tel(lambda t: True, 2)
    assert tuple(t) == TEL_KEYS
    assert t["mode"] == "NEUTRAL" and t["drum_pos"] == "NEUTRAL" and t["drum_deg"] == 41 and t["state"] == "IDLE"
    assert t["fault"] is None and t["wraps"] == 0 and t["cock"] == 0
    # sw: the firmware's one-letter keys in its order, closed contacts only: NEUTRAL code b, N20 n, H, G, S
    # (W, SW_ARM, cad-reel: closed only while the wrap cup is away from home inside the WRAP window)
    assert list(t["sw"]) == [k for k in SW_KEYS if k in t["sw"]] and all(v == 1 for v in t["sw"].values())
    assert {"b", "n", "H", "G", "S"} <= set(t["sw"]) and not {"a", "c", "l", "r", "D", "W"} & set(t["sw"])
    assert tel_seated(t) and tel_carriage_home(t)
    s = c.status()
    assert list(s)[2:] == ["phase", "aimed", "aim_known", "drum_known", "wraps_known", "magnet", "glitches", "hx",
                           "peak_a", "fw", "ev_drops"]
    assert s["phase"] == "REST" and s["aimed"] is False and s["ev_drops"] == 0 and s["fw"] == "base-1.3.0"


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
    assert mb.handle_line(b'{"id":1.5,"cmd":"ping"}') == '{"id":null,"ok":false,"err":"BAD_ID"}'
    assert json.loads(mb.handle_line(b'{"id":3,"cmd":"bogus"}'))["err"] == "UNKNOWN_CMD"
    # get: val before ok (firmware field order)
    assert list(json.loads(mb.handle_line(b'{"id":4,"cmd":"get","key":"limit_n"}'))) == ["id", "val", "ok"]
    # worst case telemetry still fits 256 bytes (lean form: null fields left out)
    with mb.lock:
        mb.drum_deg, mb.wiper_deg, mb.rpm, mb.amps = 345.67, -4096000.0, -400.0, 19.9
        mb.line_est, mb.tension, mb.fault = 99.99, 1234.5, "SWITCH_DISAGREE"
        line = mb.telemetry_line()
        tel = json.loads(line)["tel"]
        assert len(line) <= 256 and (tel.get("lean") or tel.get("trunc"))
        mb.fault, mb.drum_deg = None, DRUM_DEG["NEUTRAL"]


def test_lean_telemetry_leaves_nulls_out(pair):
    mb, c = pair
    with mb.lock:
        lean = mb.telemetry(lean=True)
        mb.line_known = False
        lean2 = mb.telemetry(lean=True)
        mb.line_known = True
    assert lean["lean"] is True and list(lean)[-1] == "lean"
    assert "line_m" in lean and "line_m" not in lean2


def test_id_null_reply_goes_to_the_waiting_command(pair):
    mb, c = pair
    mb.ignore.add("ping")
    threading.Timer(0.15, lambda: c.transport.write(b"not json\n")).start()
    r = c.ping(timeout=2, check=False)
    assert r["id"] is None and r["err"] == "BAD_JSON"


# ---------------------------------------------------------------- the D35 sequence (BASE_SPEC 10.1)
def test_d35_sequence_step_by_step(pair):
    mb, c = pair
    # 1 mode LAUNCH: MODE ok; the aim is forgotten after the shift
    to_mode(c, "LAUNCH")
    assert c.status()["aim_known"] is False
    t = idle(c)
    assert t["pitch_deg"] is None and t["sw"].get("l") == 1 and t["sw"].get("a") == 1
    # 2 aim: homes on SW_RAIL_LOW first, AIMED
    e = run(c, "aim", "AIMED", 60, deg=35)
    assert e["ok"] and abs(e["pitch_deg"] - 35) <= 0.75 and e["target_deg"] == 35
    # 3 cock: COCKED
    e = run(c, "cock", "COCKED", 10)
    assert e["ok"] and e["cock_deg"] == 415
    t = idle(c)
    assert t["cock"] == 1 and "H" not in t["sw"] and "G" not in t["sw"]
    # 4 arm: token valid 3 s
    r = c.cmd("arm")
    assert 100000 <= r["token"] <= 999999 and r["valid_ms"] == 3000
    # 5 fire: FIRED then RESET
    m = c.event_mark()
    c.fire(r["token"])
    f = c.wait_event("FIRED", W(10), since=m)
    assert abs(f["pitch_deg"] - 35) <= 0.75 and f["cock_deg"] == 432.7
    assert c.wait_event("RESET", W(10), since=m)["ok"] is True
    t = c.wait_tel(lambda t: t["state"] == "IDLE" and t["cock"] == 0, 2)
    assert t["line_m"] is None
    c.wait_tel(lambda t: "S" not in t["sw"], 2)
    # 6 dart LANDED: set line_m
    c.wait_tel(lambda t: True, 1)
    time.sleep(W(2))                                     # the default flight lands at 8 m
    c.set("line_m", 8.0)
    assert c.get("line_m")["val"] == 8
    # 8 mode NEUTRAL
    to_mode(c, "NEUTRAL")
    # 9 wrap: WRAPPED only (no MODE REEL), the drum parks at WRAPPED 314 with the collar NEUTRAL
    m = c.event_mark()
    t = wrapped_park(c)
    assert names_since(c, m) == ["WRAPPED"] and t["wraps"] == 7 and set(t["sw"]) >= {"a", "c"}
    # 10 dart ANCHORED (0.5 m slack)
    with mb.lock:
        mb.world.dart, mb.world.slack_m = "anchored", 0.5
    # 11 tidy {ms, duty}: TIDIED, take-up only (the capstan is free: line_m does not change)
    m = c.event_mark()
    c.cmd("tidy", ms=3000, duty=0.3)
    c.wait_tel(lambda t: t["state"] == "REEL", 2)
    assert c.wait_event("TIDIED", W(5), since=m)["ok"] is True
    t = idle(c)
    assert t["line_m"] == 8 and t["drum_pos"] == "WRAPPED" and 0.15 < mb.world.slack_m < 0.3
    # 12 mode REEL: the fly-in from the park
    to_mode(c, "REEL")
    t = idle(c)
    assert t["drum_deg"] == 347 and t["sw"].get("r") == 1 and set(t["sw"]) >= {"a", "b", "c"}
    # 13 reel {duty 1, max_n, until tension}: LIMIT holding
    e = run(c, "reel", "LIMIT", 60, duty=1, max_n=40, until="tension")
    assert e["holding"] is True and e["max_n"] == 40 and e["tension_n"] >= 40 and e["why"] == "tension limit"
    t = c.wait_tel(lambda t: t["state"] == "HOLD", 2)
    assert t["rpm"] == 0 and t["line_m"] < 8
    # 14 stop: ABORTED (then dart /release)
    e = run(c, "stop", "ABORTED", 2)
    assert e["what"] == "HOLD" and e["why"] == "stop"
    with mb.lock:
        mb.world.dart = "free"
    # 15 reel until stop, stop at about 1.5 m: ABORTED
    m = c.event_mark()
    c.reel(1.0, until="stop")
    c.wait_tel(lambda t: t["line_m"] is not None and t["line_m"] < 1.5, W(60))
    c.stop()
    e = c.wait_event("ABORTED", 2, since=m, fail_on_abort=False)
    assert e["what"] == "REEL" and e["why"] == "stop"
    # 16 unwrap: UNWRAPPED, then MODE NEUTRAL
    m = c.event_mark()
    c.unwrap()
    c.wait_tel(lambda t: t["state"] == "UNWRAP", 2)
    c.wait_event("MODE", W(60), since=m)
    assert names_since(c, m) == ["UNWRAPPED", "MODE"]
    mode_ev = c.events(m)[-1]
    assert mode_ev["ok"] and mode_ev["mode"] == "NEUTRAL" and mode_ev["drum_pos"] == "NEUTRAL"
    t = idle(c)
    assert t["wraps"] == 0 and t["drum_deg"] == 41
    # 17 retrieve {duty 0.3, max_s}: SEATED (the take-up pulled the freed line during the unwrap already)
    e = run(c, "retrieve", "SEATED", 60, duty=0.3, max_s=60)
    t = c.wait_tel(lambda t: t["state"] == "IDLE" and t["line_m"] == 0, 2)
    assert tel_seated(t)
    # 18 mode LAUNCH
    to_mode(c, "LAUNCH")
    assert c.status()["phase"] == "REST"


def test_retrieve_take_up_speed_and_dart_stuck(pair):
    mb, c = pair
    with mb.lock:                                        # a free dart 1.8 m out (a NO_GO landing close by)
        mb.world.dart, mb.line_m = "free", 1.8
    c.set("line_m", 1.8)
    t0 = mb.sim_t
    e = run(c, "retrieve", "SEATED", 60)                 # defaults: duty 0.3, max_s from approachLimit()
    v = 1.8 / (mb.sim_t - t0)
    assert 0.07 < v < 0.1, v                            # the take-up is slow: about 0.09 m/s at duty 0.3
    # stuck: the take-up slips on a heavy dart: DART_STUCK after max_s, the drum stays at NEUTRAL (its park)
    with mb.lock:
        mb.world.dart, mb.line_m, mb.world.drag_free_n = "free", 3.0, 9.0
    m = c.event_mark()
    c.cmd("retrieve", max_s=5)
    with pytest.raises(BaseError) as ei:
        c.wait_event("SEATED", W(10), since=m)
    assert ei.value.err == "DART_STUCK" and "5 s" in ei.value.reply["why"]
    mode = c.wait_event("MODE", 2, since=m, fail_on_fault=False)
    assert mode["ok"] and mode["drum_pos"] == "NEUTRAL"
    c.clear()
    # default max_s follows line_m like base-1.2.1 approachLimit(): 10 + 1.5 x 3 / (0.29 x 0.3 - 0.03) = 88.9 s
    with mb.lock:
        mb.line_est = 3.0
        mb.handle({"cmd": "retrieve"})
        assert mb.op["max_s"] == pytest.approx(10 + 4.5 / 0.057, abs=0.01)
        mb._cmd_stop(False)
    assert err_of(c.cmd, "retrieve", max_s=4) == "BAD_ARG"      # firmware range 5..600
    assert err_of(c.cmd, "retrieve", max_s=601) == "BAD_ARG"


def test_retrieve_and_tidy_refusals(pair):
    mb, c = pair
    assert c.cmd("tidy", ms=0, check=False)["err"] == "BAD_ARG"
    assert c.cmd("tidy", ms=70000, check=False)["err"] == "BAD_ARG"
    assert c.cmd("retrieve", max_s=-1, check=False)["err"] == "BAD_ARG"
    assert c.cmd("reel", until="tension", check=False)["err"] == "NOT_WRAPPED"   # no tension without the capstan
    to_mode(c, "LAUNCH")
    assert c.cmd("tidy", check=False)["err"] == "NOT_IN_NEUTRAL"
    assert c.cmd("retrieve", check=False)["err"] == "NOT_WRAPPED"
    assert c.cmd("reel", check=False)["err"] == "NOT_WRAPPED"
    to_mode(c, "NEUTRAL")
    wrapped_park(c)
    # at the WRAPPED park the collar is NEUTRAL: take-up commands work (reel until slip: LIMIT tidy done)
    with mb.lock:
        mb.world.dart, mb.line_m, mb.world.slack_m = "anchored", 6.0, 0.1
    e = run(c, "reel", "LIMIT", 20, until="slip")
    assert e["tension_n"] is None and e["holding"] is False and e["why"] == "tidy done (take-up slipping)"
    to_mode(c, "REEL")
    assert c.cmd("tidy", check=False)["err"] == "NOT_IN_NEUTRAL"
    assert c.cmd("retrieve", check=False)["err"] == "WRAPS_ON"     # D35: never seat on the capstan
    assert c.cmd("reel", until="seated", check=False)["err"] == "WRAPS_ON"
    assert c.cmd("reel", until="slip", check=False)["err"] == "BAD_ARG"
    assert c.cmd("reel", until="forever", check=False)["err"] == "BAD_ARG"
    assert c.cmd("reel", duty=1.5, check=False)["err"] == "BAD_ARG"
    assert c.cmd("reel", max_n=0, check=False)["err"] == "BAD_ARG"


# ---------------------------------------------------------------- refusals per drum position
def test_refusals_at_neutral_and_launch(pair):
    mb, c = pair
    assert err_of(c.aim, 30) == "NOT_IN_LAUNCH"
    assert err_of(c.cock) == "NOT_IN_LAUNCH"
    assert err_of(c.arm) == "NOT_IN_LAUNCH"
    assert err_of(c.decock) == "NOT_IN_LAUNCH"
    assert err_of(c.fire, 123456) == "NOT_ARMED"
    assert err_of(c.mode, "REEL") == "NOT_WRAPPED"       # REEL only with the wraps on
    m = c.event_mark()
    c.unwrap()                                           # at NEUTRAL: UNWRAPPED at once, no MODE
    assert c.wait_event("UNWRAPPED", 1, since=m)["ok"]
    assert c.cmd("mode", to="SIDEWAYS", check=False)["err"] == "BAD_ARG"
    assert c.cmd("mode", check=False)["err"] == "BAD_ARG"
    m = c.event_mark()
    c.mode("NEUTRAL")                                    # already there: MODE at once
    assert c.wait_event("MODE", 1, since=m)["ok"]
    to_mode(c, "LAUNCH")
    assert err_of(c.unwrap) == "NOT_WRAPPED"
    assert err_of(c.mode, "REEL") == "NOT_WRAPPED"
    assert err_of(c.aim, 5) == "OUT_OF_RANGE"
    assert err_of(c.aim, 48) == "OUT_OF_RANGE"
    r = c.get("pitch_range")
    assert r["min"] == 6.01 and r["max"] == 47
    m = c.event_mark()
    c.mode("NEUTRAL")
    assert err_of(c.mode, "LAUNCH") == "BUSY"
    assert err_of(c.aim, 30) == "BUSY"
    c.wait_event("MODE", W(10), since=m)


def test_refusals_at_the_wrapped_park_and_in_reel(pair):
    mb, c = pair
    m = c.event_mark()
    c.wrap()
    c.wait_tel(lambda t: t["state"] == "WRAP" and t["drum_pos"] == "BETWEEN" and t["mode"] == "NEUTRAL", 3)
    assert err_of(c.mode, "LAUNCH") == "BUSY"
    c.wait_event("WRAPPED", W(60), since=m)
    for to in ("LAUNCH", "NEUTRAL"):
        assert err_of(c.mode, to) == "WRAPS_ON"
    assert err_of(c.aim, 30) == "NOT_IN_LAUNCH"
    assert err_of(c.decock) == "NOT_IN_LAUNCH"
    assert err_of(c.jog, "wiper", 1, 300) == "SHIFTING"
    m = c.event_mark()
    c.wrap()                                             # already wrapped: WRAPPED at once
    assert c.wait_event("WRAPPED", 1, since=m)["wraps"] == 7
    to_mode(c, "REEL")
    m = c.event_mark()
    c.wrap()                                             # in REEL too
    assert c.wait_event("WRAPPED", 1, since=m)["wraps"] == 7
    assert err_of(c.jog, "wiper", -1, 300) == "BACKSTOP"
    assert err_of(c.jog, "drum", -1, 300) == "WRAPS_ON"
    m = c.event_mark()
    c.mode("REEL")                                       # already there
    assert c.wait_event("MODE", 1, since=m)["mode"] == "REEL"


def test_wrap_straight_from_launch_and_launch_busy(pair):
    mb, c = pair
    to_mode(c, "LAUNCH")
    run(c, "cock", "COCKED", 10)
    assert err_of(c.wrap) == "LAUNCH_BUSY"
    assert err_of(c.mode, "NEUTRAL") == "LAUNCH_BUSY"
    assert err_of(c.jog, "wiper", 1, 300) == "WOULD_FIRE"
    assert err_of(c.jog, "wiper", -1, 300) == "NOT_AT_REST"
    assert err_of(c.aim, 30) == "NOT_AT_REST"
    m = c.event_mark()
    c.cock()                                             # cocked: COCKED at once
    assert c.wait_event("COCKED", 1, since=m)["ok"]
    e = run(c, "decock", "RESET", 20)                    # a cocked shot called off
    assert e["ok"] and e["how"] == "decock"
    assert c.status()["phase"] == "REST" and c.status()["aim_known"] is False
    m = c.event_mark()
    wrapped_park(c)                                      # D22: wrap starts from LAUNCH too
    assert names_since(c, m) == ["WRAPPED"]


def test_cock_needs_the_dart_seated_and_arm_refusals(pair):
    mb, c = pair
    to_mode(c, "LAUNCH")
    with mb.lock:
        mb.world.dart, mb.line_m = "free", 3.0
    assert err_of(c.cock) == "NOT_SEATED"
    assert err_of(c.arm) == "NOT_AIMED"
    run(c, "aim", "AIMED", 60, deg=30)
    assert err_of(c.arm) == "NOT_SEATED"
    with mb.lock:
        mb.world.dart, mb.line_m = "seated", 0.0
    c.arm()
    assert err_of(c.fire, 1) == "BAD_TOKEN"
    tok = c.arm()
    assert err_of(c.fire, tok + 1) == "BAD_TOKEN"
    assert err_of(c.fire, tok) == "NOT_ARMED"            # the wrong token burnt the arm
    tok = c.arm()
    time.sleep(W(4.5) - 1.0 + 0.1)
    assert err_of(c.fire, tok) == "TOKEN_EXPIRED"
    assert err_of(c.fire, tok) == "TOKEN_EXPIRED"        # the firmware keeps saying so until the next arm


def test_fire_from_rest_runs_the_whole_cycle(pair):
    mb, c = pair
    to_mode(c, "LAUNCH")
    run(c, "aim", "AIMED", 60, deg=40)
    tok = c.arm()                                        # cock is optional
    m = c.event_mark()
    c.fire(tok)
    c.wait_tel(lambda t: t["state"] == "FIRE", 2)
    assert abs(c.wait_event("FIRED", W(10), since=m)["pitch_deg"] - 40) < 0.75
    c.wait_event("RESET", W(10), since=m)
    s = c.status()
    assert s["phase"] == "REST" and s["aimed"] is False
    assert err_of(c.arm) == "NOT_AIMED"                  # aim again after a shot


def test_decock_variants(pair):
    mb, c = pair
    to_mode(c, "LAUNCH")
    e = run(c, "decock", "RESET", 2)                     # at rest: nothing to unwind, RESET ok at once
    assert e["ok"] is True and e["how"] == "decock"
    with mb.lock:                                        # phase UNKNOWN with the carriage home (a stop after the release)
        mb.phase, mb.phase_hint = "UNKNOWN", 500.0
    m = c.event_mark()
    c.cmd("cal", what="decock")
    e = c.wait_event("RESET", 1, since=m, fail_on_not_ok=False)
    assert e["ok"] is False and e["how"] == "decock" and "nothing to decock" in e["why"]
    with mb.lock:
        mb.phase_hint = 200.0                            # SW_HOME says home but the angle says drawn: lying switch
    assert err_of(c.decock) == "SWITCH_DISAGREE"
    e = run(c, "decock", "RESET", 2, force=True)
    assert e["how"] == "decock by travel" and c.status()["phase"] == "REST"


def test_stop_mid_cock_then_decock(pair):
    mb, c = pair
    to_mode(c, "LAUNCH")
    m = c.event_mark()
    c.cock()
    c.wait_tel(lambda t: (t["cock"] or 0) > 0.3, 5)
    c.stop()
    e = c.wait_event("ABORTED", 2, since=m, fail_on_abort=False)
    assert e["what"] == "COCK" and e["why"] == "stop"
    assert c.status()["phase"] == "UNKNOWN"
    assert err_of(c.cock) == "LOADED_UNKNOWN"
    assert err_of(c.mode, "NEUTRAL") == "LAUNCH_BUSY"
    assert err_of(c.aim, 30) == "PHASE_UNKNOWN"
    assert err_of(c.arm) == "NOT_COCKED"
    e = run(c, "cal", "RESET", 20, what="decock")
    assert e["ok"] and e["how"] == "decock" and c.status()["phase"] == "REST"
    to_mode(c, "NEUTRAL")


def test_fresh_build_needs_cock_rest():
    mb, c = connected_pair(time_scale=SCALE, fresh=True)
    try:
        c.wait_tel(lambda t: True, 2)
        to_mode(c, "LAUNCH")
        assert c.wait_tel(lambda t: True, 1)["pitch_deg"] is None
        assert err_of(c.aim, 30) == "PHASE_UNKNOWN"
        c.cal_cock_rest()
        assert run(c, "aim", "AIMED", 60, deg=30)["ok"]
    finally:
        c.close(); mb.stop()


# ---------------------------------------------------------------- tow, hold, seat
def test_reel_until_stop_limit_does_not_hold_and_hold_resumes_on_slack(pair):
    mb, c = pair
    wrapped_park(c)
    to_mode(c, "REEL")
    with mb.lock:
        mb.world.dart, mb.line_m, mb.world.slack_m = "anchored", 5.0, 0.0
    e = run(c, "reel", "LIMIT", 20, duty=1.0, max_n=20, until="stop")
    assert e["holding"] is False
    idle(c)
    e = run(c, "reel", "LIMIT", 20, duty=1.0, max_n=20, until="tension")
    c.wait_tel(lambda t: t["state"] == "HOLD", 2)
    assert err_of(c.unwrap) == "HOLDING" and err_of(c.mode, "NEUTRAL") == "HOLDING"
    m = c.event_mark()
    with mb.lock:
        mb.world.slack_m = 0.3                           # the rover drove toward the dart: the load cell sees slack
    c.wait_tel(lambda t: t["state"] == "REEL", W(4))
    assert c.wait_event("LIMIT", W(20), since=m)["holding"] is True
    m = c.event_mark()
    c.reel(1.0, max_n=30, until="tension")               # allowed from HOLD: winds again
    assert c.wait_event("LIMIT", W(20), since=m)["max_n"] == 30
    with mb.lock:
        assert mb.op["trip"] == 6.5                      # D34 tow trip: never below tow_trip_a


def test_seat_ends_reel_only_until_seated_by_default(pair):
    mb, c = pair
    assert mb.seat_ends_any_reel is False
    with mb.lock:
        mb.world.dart, mb.line_m = "free", 0.6
    m = c.event_mark()
    c.reel(0.3, until="stop")                            # take-up, until stop: winds on against the cradle
    c.wait_tel(lambda t: tel_seated(t), W(20))
    time.sleep(W(1))
    assert "SEATED" not in names_since(c, m) and c.tel["state"] == "REEL"
    c.stop()


def test_seat_ends_any_reel_when_switched_on():
    mb, c = connected_pair(time_scale=SCALE, seat_ends_any_reel=True)
    try:
        c.wait_tel(lambda t: True, 2)
        with mb.lock:
            mb.world.dart, mb.line_m = "free", 0.6
        e = run(c, "reel", ("SEATED", "LIMIT"), 20, duty=0.3, until="stop")
        assert e["evt"] == "SEATED"
    finally:
        c.close(); mb.stop()


def test_stop_during_retrieve_raises_aborted_in_wait(pair):
    mb, c = pair
    with mb.lock:
        mb.world.dart, mb.line_m = "free", 5.0
    m = c.event_mark()
    c.cmd("retrieve")
    time.sleep(0.1)
    c.stop()
    with pytest.raises(BaseError) as ei:
        c.wait_event("SEATED", 3, since=m)
    assert ei.value.err == "ABORTED" and ei.value.reply["what"] == "REEL"


# ---------------------------------------------------------------- faults, stop, estop (fault_rules)
def test_fault_in_reel_holds_the_drum(pair):
    """base-1.2.1 safeToNeutral (D27): with the wraps on (REEL) a fault holds the drum (MODE ok false, DRUM_HELD)."""
    mb, c = pair
    wrapped_park(c)
    to_mode(c, "REEL")
    with mb.lock:
        mb.world.dart, mb.line_m = "free", 20.0
    mb.fail_next["reel"] = "STALL"
    m = c.event_mark()
    c.reel(1.0, until="stop")
    with pytest.raises(BaseError) as ei:
        c.wait_event("LIMIT", W(10), since=m)
    assert ei.value.err == "STALL"
    with pytest.raises(BaseError) as ei:
        c.wait_event("MODE", 2, since=m, fail_on_fault=False)
    assert ei.value.err == "DRUM_HELD" and "wraps on" in ei.value.reply["why"]
    assert names_since(c, m) == ["FAULT", "STALL", "MODE"]
    t = c.wait_tel(lambda t: t["state"] == "FAULT", 2, fail_on_fault=False)
    assert t["drum_pos"] == "REEL" and t["fault"] == "STALL"
    assert c.status()["why"] == "wiper stalled"
    assert err_of(c.mode, "REEL") == "FAULTED"
    c.clear()
    idle(c)


def test_estop_follows_the_fault_rules(pair):
    mb, c = pair
    m = c.event_mark()
    c.estop()                                            # at NEUTRAL: MODE ok at once
    assert c.wait_event("MODE", 1, since=m)["drum_pos"] == "NEUTRAL"
    wrapped_park(c)
    to_mode(c, "REEL")
    m = c.event_mark()
    c.reel(0.5, until="stop")
    c.estop()
    with pytest.raises(BaseError) as ei:                 # wraps on (REEL): the drum holds (D27)
        c.wait_event("MODE", W(15), since=m, fail_on_abort=False)
    assert ei.value.err == "DRUM_HELD" and "wraps on" in ei.value.reply["why"]
    assert names_since(c, m) == ["ABORTED", "MODE"] and c.events(m)[0]["why"] == "estop"
    c.clear()
    idle(c)
    # estop inside the wrap window: the drum holds (DRUM_HELD)
    m = c.event_mark()
    c.unwrap()
    c.wait_tel(lambda t: t["drum_pos"] == "BETWEEN", W(15))
    c.estop()
    with pytest.raises(BaseError) as ei:
        c.wait_event("MODE", 2, since=m, fail_on_abort=False)
    assert ei.value.err == "DRUM_HELD" and "wraps on" in ei.value.reply["why"]
    run(c, "unwrap", "UNWRAPPED", 60)                    # the operator decides: unwrap


def test_fault_while_cocking_holds_the_drum_in_launch(pair):
    mb, c = pair
    to_mode(c, "LAUNCH")
    mb.fail_next["cock"] = "STALL"
    m = c.event_mark()
    c.cock()
    with pytest.raises(BaseError) as ei:
        c.wait_event("COCKED", W(10), since=m)
    assert ei.value.err == "STALL"
    with pytest.raises(BaseError) as ei:
        c.wait_event("MODE", 2, since=m, fail_on_fault=False)
    assert ei.value.err == "DRUM_HELD" and "launch train" in ei.value.reply["why"]
    t = c.wait_tel(lambda t: t["state"] == "FAULT", 2, fail_on_fault=False)
    assert t["drum_pos"] == "LAUNCH" and t["cock"] is None   # the phase is UNKNOWN after a fault mid-cock
    c.clear()
    e = run(c, "decock", "RESET", 20)                    # recovery: decock, then aim again
    assert e["ok"] and c.status()["aim_known"] is False


def test_fault_in_launch_at_rest_safes_to_neutral_and_clear(pair):
    mb, c = pair
    to_mode(c, "LAUNCH")
    m = c.event_mark()
    mb.inject_fault("ANGLE_SENSOR", present=True)
    e = c.wait_event("MODE", W(10), since=m, fail_on_fault=False)
    assert e["ok"] and e["mode"] == "NEUTRAL"
    t = c.wait_tel(lambda t: t["state"] == "FAULT" and t["drum_pos"] == "NEUTRAL", 5, fail_on_fault=False)
    assert t["fault"] == "ANGLE_SENSOR"
    assert err_of(c.clear) == "STILL_PRESENT"
    mb.fault_present = False
    c.clear()
    c.wait_tel(lambda t: t["fault"] is None and t["state"] == "IDLE", 2, fail_on_fault=False)


def test_fail_next_drum_timeout_holds(pair):
    mb, c = pair
    mb.fail_next["mode"] = "DRUM_TIMEOUT"
    m = c.event_mark()
    c.mode("LAUNCH")
    with pytest.raises(BaseError) as ei:
        c.wait_event("MODE", W(10), since=m)
    assert ei.value.err == "DRUM_TIMEOUT"
    with pytest.raises(BaseError) as ei:
        c.wait_event("MODE", 2, since=m, fail_on_fault=False)
    assert ei.value.err == "DRUM_HELD" and ei.value.reply["why"] == "drum fault"
    t = c.wait_tel(lambda t: t["state"] == "FAULT", 2, fail_on_fault=False)
    assert t["drum_pos"] == "BETWEEN" and t["mode"] == "NONE"
    c.clear()
    to_mode(c, "LAUNCH")                                 # can continue from between


# ---------------------------------------------------------------- settings, jog, link
def test_jog_and_settings(pair):
    mb, c = pair
    assert c.cmd("jog", motor="arm", dir=1, check=False)["err"] == "BAD_ARG"
    assert c.cmd("jog", motor="drum", dir=2, check=False)["err"] == "BAD_ARG"
    c.jog("drum", 1, 1500)
    t = c.wait_tel(lambda t: t["state"] == "IDLE" and t["drum_pos"] == "BETWEEN", 3)
    assert t["mode"] == "NEUTRAL" and abs(t["drum_deg"] - 53.2) < 1.5   # wrap prep: collar still NEUTRAL
    for k, v in (("limit_n", 30), ("tension_offset_n", -10.4), ("tow_trip_a", 7.5), ("k_n_per_a", 17)):
        c.set(k, v)
        assert c.get(k)["val"] == v
    assert err_of(c.set, "limit_n", 900) == "OUT_OF_RANGE"
    assert err_of(c.set, "tension_offset_n", 60) == "OUT_OF_RANGE"
    assert err_of(c.set, "nope", 1) == "BAD_KEY"
    assert c.get("fw")["val"] == "base-1.3.0"
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
    assert t["mode"] == "NONE" and t["drum_deg"] is None
    assert err_of(c.mode, "LAUNCH") == "POSITION_UNKNOWN"
    assert err_of(c.cal_drum) == "POSITION_UNKNOWN"
    assert c.cmd("tidy", check=False)["err"] == "POSITION_UNKNOWN"
    mb.code_bad = False


def test_256_byte_line_is_legal(pair):
    mb, c = pair
    base = len(json.dumps({"id": c._next_id, "cmd": "ping", "x": ""}, separators=(",", ":")))
    r = c.cmd("ping", x="a" * (256 - base))
    assert r["ok"]
    assert max(len(json.dumps(m, separators=(",", ":"))) for m in mb.received) == 256
    with pytest.raises(ValueError):
        c.cmd("ping", x="a" * 300)


def test_hello_fails_pending_commands_and_waits(pair):
    mb, c = pair
    mb.ignore.add("ping")
    threading.Timer(0.15, lambda: mb._send({"evt": "HELLO", "fw": FW, "reset": "WATCHDOG"})).start()
    with pytest.raises(BaseError) as ei:
        c.ping(timeout=2)
    assert ei.value.err == "BASE_RESET"
    mb.ignore.clear()
    m = c.event_mark()
    threading.Timer(0.1, lambda: mb._send({"evt": "HELLO", "fw": FW, "reset": "BROWNOUT"})).start()
    with pytest.raises(BaseError) as ei:
        c.wait_event("MODE", 2, since=m)
    assert ei.value.err == "BASE_RESET"


def test_host_watchdog_stops_a_retrieve(pair):
    mb, c = pair
    with mb.lock:
        mb.world.dart, mb.line_m = "free", 30.0
    c.set("host_timeout_s", 2)
    m = c.event_mark()
    c.cmd("retrieve")
    e = c.wait_event("ABORTED", W(4), since=m, fail_on_abort=False, pred=lambda e: e["what"] == "HOST")
    assert e["why"] == "host silent"
    assert [(x["what"], x["why"]) for x in c.events(m)] == [("REEL", "stop"), ("HOST", "host silent")]


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
        idle_n = len(n)
        n.clear()
        c.mode("LAUNCH")   # drum moves 41 deg at 8.1 deg/s: 5 s
        time.sleep(0.6)
        moving = len(n) / 0.6
        assert 7 <= idle_n <= 13
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


def test_reply_is_written_before_the_events_it_caused(pair):
    mb, c = pair
    seen = []
    c.on_line = lambda d, obj: seen.append(obj) if d == "rx" and "tel" not in obj else None
    m = c.event_mark()
    r = c.mode("NEUTRAL")                               # already there: ok and MODE at once
    c.wait_event("MODE", 1, since=m)
    i_rep = next(i for i, o in enumerate(seen) if o.get("id") == r["id"])
    i_evt = next(i for i, o in enumerate(seen) if o.get("evt") == "MODE")
    assert i_rep < i_evt


# ---------------------------------------------------------------- mock against the real firmware (basesim)
CMP_SPEED = 40.0
COMPARE = ("state", "mode", "drum_pos", "drum_deg", "wraps", "cock", "pitch_deg", "line_m", "fault")


def _d35_script(c, scale, scenario):
    """BASE_SPEC 10.1 steps 1 to 18 (GO landing, tow, capstan reel-in, unwrap, retrieve). Returns (events, tel)."""
    w = lambda s: s / scale * 1.5 + 2.0
    mark = c.event_mark()

    def act(name, evt, sim_s, **a):
        m = c.event_mark()
        c.cmd(name, **a)
        return c.wait_event(evt, w(sim_s), since=m, fail_on_abort=evt != "ABORTED")

    act("mode", "MODE", 60, to="LAUNCH")                         # 1
    act("aim", "AIMED", 60, deg=35)                              # 2
    act("cock", "COCKED", 20)                                    # 3
    tok = c.cmd("arm")["token"]                                  # 4
    act("fire", "RESET", 20, token=tok)                          # 5
    c.wait_tel(lambda t: t["state"] == "IDLE", w(2))
    time.sleep(w(2) - 2.0)
    c.cmd("set", key="line_m", val=8.0)                          # 6
    act("mode", "MODE", 60, to="NEUTRAL")                        # 8
    act("wrap", "WRAPPED", 80)                                   # 9
    c.wait_tel(lambda t: t["state"] == "IDLE", w(2))
    scenario("anchor")                                           # 10
    act("tidy", "TIDIED", 10, ms=3000, duty=0.3)                 # 11
    act("mode", "MODE", 60, to="REEL")                           # 12
    act("reel", "LIMIT", 60, duty=1, max_n=20, until="tension")  # 13
    act("stop", "ABORTED", 2)                                    # 14
    scenario("free")
    c.cmd("reel", until="stop")                                  # 15
    c.wait_tel(lambda t: t["line_m"] is not None and t["line_m"] < 1.5, w(80))
    act("stop", "ABORTED", 2)
    act("unwrap", "MODE", 80)                                    # 16
    act("retrieve", "SEATED", 60, duty=0.3, max_s=60)            # 17
    act("mode", "MODE", 60, to="LAUNCH")                         # 18
    tel = c.wait_tel(lambda t: t["state"] == "IDLE", w(2))
    time.sleep(0.3)
    tel = c.tel
    return [e["evt"] for e in c.events(mark)], tel


def test_d35_sequence_mock_matches_basesim():
    from basesim_transport import find_basesim, start_basesim
    binary = find_basesim(build=True)
    if binary is None:
        pytest.skip("basesim not built and cannot be built (make -C jetson/tools/basesim)")
    proc, real = start_basesim(binary, speed=CMP_SPEED, line_out=0.0, seat_at=0.0, load_cell=True)
    try:
        def sim_real(what):
            real.transport.sim("anchor 7.5" if what == "anchor" else "free")
        ev_real, tel_real = _d35_script(real, CMP_SPEED, sim_real)
        # compare major.minor only: the real firmware moves on in patch releases (base-1.2.1 ...)
        mm = lambda v: ".".join(str(v).split(".")[:2])
        assert mm(real.cmd("ping")["fw"]) == mm(FW)
    finally:
        real.close()
    mb, c = connected_pair(time_scale=CMP_SPEED, drum_at="LAUNCH")
    try:
        c.wait_tel(lambda t: True, 2)

        def sim_mock(what):
            with mb.lock:
                if what == "anchor":
                    mb.world.dart, mb.world.slack_m = "anchored", max(0.0, mb.line_m - 7.5)
                else:
                    mb.world.dart = "free"
        ev_mock, tel_mock = _d35_script(c, CMP_SPEED, sim_mock)
    finally:
        c.close(); mb.stop()
    assert ev_mock == ev_real
    assert ev_real == ["MODE", "AIMED", "COCKED", "FIRED", "RESET", "MODE", "WRAPPED", "TIDIED", "MODE", "LIMIT",
                       "ABORTED", "ABORTED", "UNWRAPPED", "MODE", "SEATED", "MODE"]
    assert {k: tel_mock.get(k) for k in COMPARE} == {k: tel_real.get(k) for k in COMPARE}
    assert set(tel_mock["sw"]) - {"L"} == set(tel_real["sw"]) - {"L"}
    assert list(tel_mock) == list(tel_real)

