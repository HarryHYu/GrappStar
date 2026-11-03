"""BaseClient against the REAL base firmware (firmware/base base_logic.h, fw base-1.2, + its physics model test/sim.h)
via basesim, following the one C1 sequence of BASE_SPEC 10.1 (D35).

Skipped when jetson/tools/basesim/basesim is missing and cannot be built (make -C jetson/tools/basesim, which needs the
firmware sources, see jetson/tools/basesim/README.md). Marked slow (python3 -m pytest -m slow, check_all.sh). The
simulator runs at SPEED times real time; firmware timings below are in simulated seconds and converted with T().

The tests run in file order on one simulator (one mission, load cell fitted). They use only BaseClient.cmd /
wait_event / wait_tel. Drum: LAUNCH / L_TO_N / NEUTRAL / WRAP_PREP / WRAP / N_TO_R (WRAPPED park 314, collar NEUTRAL) /
REEL. "wrap" parks at WRAPPED (WRAPPED only), "mode REEL" flies the collar in, "unwrap" goes back to NEUTRAL
(UNWRAPPED then MODE), "retrieve" seats the dart on the take-up in NEUTRAL.
"""
import os
import subprocess
import time

import pytest

from grappstar.comms.base import BaseClient, BaseError

from basesim_transport import find_basesim, scenario_args, start_basesim

SPEED = float(os.environ.get("GRAPPSTAR_BASESIM_SPEED", "8"))
BIN = find_basesim(build=True)
pytestmark = [pytest.mark.slow,
              pytest.mark.skipif(BIN is None, reason="basesim not built and cannot be built (no firmware sources?)")]

TEL_KEYS = ["state", "mode", "drum_deg", "drum_pos", "wiper_deg", "rpm", "amps", "pitch_deg", "cock", "wraps",
            "line_m", "tension_n", "sw", "fault", "vbat"]


def T(sim_s):
    """Wall seconds for sim_s simulated seconds, with slack for a loaded machine."""
    return sim_s / SPEED * 1.5 + 2.0


@pytest.fixture(scope="module")
def base():
    proc, c = start_basesim(BIN, speed=SPEED, line_out=0.0, seat_at=0.0, load_cell=True)   # dart in the cradle
    yield c
    c.close()
    assert proc.poll() is not None


def run(c, name, evt, sim_s, **args):
    """Send an action, expect ok, wait for its completion event (FAULT / ABORTED / ok:false raise BaseError)."""
    mark = c.event_mark()
    c.cmd(name, **args)
    return c.wait_event(evt, timeout=T(sim_s), since=mark)


def refused(c, name, **args):
    r = c.cmd(name, check=False, **args)
    assert r["ok"] is False, r
    return r["err"]


def idle(c, sim_s=2, fail_on_fault=True, pred=lambda t: True):
    return c.wait_tel(lambda t: t["state"] == "IDLE" and pred(t), T(sim_s), what="IDLE", fail_on_fault=fail_on_fault)


# ------------------------------------------------------------------------------------------------ link
def test_ping_status(base):
    r = base.cmd("ping")
    assert r["ok"] and r["fw"].startswith("base-1.") and isinstance(r["t"], int)
    s = base.cmd("status")
    assert s["phase"] == "REST" and s["drum_known"] and s["magnet"] == "OK" and s["hx"] is True
    assert any(e["evt"] == "HELLO" for e in base.events()) and any(e["evt"] == "BOOT" for e in base.events())


def test_telemetry_shape_and_rate(base):
    t = base.wait_tel(lambda t: True, 2)
    assert list(t.keys()) == TEL_KEYS
    assert t["state"] == "IDLE" and t["mode"] == "LAUNCH" and t["drum_pos"] == "LAUNCH" and t["fault"] is None
    assert t["cock"] == 0 and t["wraps"] == 0 and t["tension_n"] is not None and t["vbat"] > 12
    assert {"a", "l", "n", "H", "G", "S", "W"} <= set(t["sw"])
    n = [0]
    base.add_tel_callback(lambda _t: n.__setitem__(0, n[0] + 1))
    t0 = time.monotonic()
    time.sleep(2.0)
    hz = n[0] / ((time.monotonic() - t0) * SPEED)          # per simulated second
    assert 7 <= hz <= 12, hz                              # 10 Hz idle


# ------------------------------------------------------------------------------------------------ launch (10.1 steps 1 to 6)
def test_mode_launch_already_there(base):
    e = run(base, "mode", "MODE", 1, to="LAUNCH")
    assert e["ok"] and e["mode"] == "LAUNCH" and e["drum_pos"] == "LAUNCH"


def test_aim(base):
    e = run(base, "aim", "AIMED", 40, deg=35)            # homes on SW_RAIL_LOW first (aim unknown at boot)
    assert e["ok"] and abs(e["pitch_deg"] - 35) <= 0.75 and e["target_deg"] == 35
    t = idle(base)
    assert abs(t["pitch_deg"] - 35) <= 0.75


def test_cock(base):
    e = run(base, "cock", "COCKED", 10)
    assert e["ok"] and 405 < e["cock_deg"] < 425
    assert base.cmd("status")["phase"] == "COCKED"
    t = idle(base, pred=lambda t: t["cock"] == 1)
    assert "H" not in t["sw"] and "G" not in t["sw"]
    assert refused(base, "mode", to="NEUTRAL") == "LAUNCH_BUSY"
    assert refused(base, "wrap") == "LAUNCH_BUSY"
    assert refused(base, "aim", deg=30) == "NOT_AT_REST"


def test_arm_bad_token_then_fire(base):
    tok = base.cmd("arm")["token"]
    assert isinstance(tok, int) and 100000 <= tok <= 999999
    assert refused(base, "fire", token=tok + 1 if tok < 999999 else tok - 1) == "BAD_TOKEN"
    assert refused(base, "fire", token=tok) == "NOT_ARMED"   # a wrong token burns the arm
    r = base.cmd("arm")
    assert r["valid_ms"] == 3000
    mark = base.event_mark()
    base.cmd("fire", token=r["token"])
    f = base.wait_event("FIRED", T(10), since=mark)
    assert abs(f["pitch_deg"] - 35) < 1 and 420 < f["cock_deg"] < 445
    e = base.wait_event("RESET", T(10), since=mark)
    assert e["ok"]
    t = idle(base, pred=lambda t: t["cock"] == 0)
    assert t["line_m"] is None                             # line unknown after a shot


def test_set_line(base):
    base.cmd("set", key="line_m", val=8.0)
    assert base.cmd("get", key="line_m")["val"] == pytest.approx(8.0)
    base.wait_tel(lambda t: t["line_m"] == 8.0, 2, what="line_m 8")
    assert base.cmd("set", key="tension_offset_n", val=-10.4)["ok"]


# ------------------------------------------------------------------------------------------------ reel side (steps 8 to 18)
def test_neutral_refusals(base):
    e = run(base, "mode", "MODE", 15, to="NEUTRAL")
    assert e["ok"] and e["mode"] == "NEUTRAL" and e["drum_pos"] == "NEUTRAL"
    assert refused(base, "aim", deg=30) == "NOT_IN_LAUNCH"
    assert refused(base, "decock") == "NOT_IN_LAUNCH"
    assert refused(base, "mode", to="REEL") == "NOT_WRAPPED"   # REEL only with the wraps on
    assert refused(base, "reel", until="tension") == "NOT_WRAPPED"   # no tension without the capstan


def test_wrap_parks_at_wrapped(base):
    mark = base.event_mark()
    e = run(base, "wrap", "WRAPPED", 60)
    assert e["ok"] and e["wraps"] == 7
    t = idle(base, pred=lambda t: t["drum_pos"] == "WRAPPED")
    assert t["mode"] == "NEUTRAL" and t["drum_deg"] == 314 and t["wraps"] == 7 and {"a", "c"} <= set(t["sw"])
    time.sleep(T(0.5) - 2.0)
    assert [x["evt"] for x in base.events(mark)] == ["WRAPPED"]   # no MODE REEL here
    for to in ("LAUNCH", "NEUTRAL"):
        assert refused(base, "mode", to=to) == "WRAPS_ON"


def test_tidy(base):
    base.transport.sim("anchor 7.5")                       # the dart is anchored: line taut at 7.5 m
    e = run(base, "tidy", "TIDIED", 10, ms=3000, duty=0.3)
    assert e["ok"] is True
    t = idle(base)
    assert t["drum_pos"] == "WRAPPED" and t["line_m"] == 8   # take-up only: the capstan count does not change


def test_mode_reel_fly_in(base):
    e = run(base, "mode", "MODE", 30, to="REEL")
    assert e["ok"] and e["mode"] == "REEL" and e["drum_pos"] == "REEL"
    t = idle(base, pred=lambda t: t["drum_pos"] == "REEL")
    assert t["drum_deg"] == 347 and t["sw"].get("r") == 1
    assert refused(base, "tidy") == "NOT_IN_NEUTRAL"
    assert refused(base, "reel", until="slip") == "BAD_ARG"


def test_tow_to_limit_then_stop(base):
    e = run(base, "reel", "LIMIT", 60, duty=1, max_n=20, until="tension")
    assert e["holding"] is True and e["max_n"] == 20 and e["tension_n"] >= 20
    base.wait_tel(lambda t: t["state"] == "HOLD", 2, what="HOLD")
    assert refused(base, "unwrap") == "HOLDING"
    mark = base.event_mark()
    base.cmd("stop")
    a = base.wait_event("ABORTED", 2, since=mark)
    assert a["what"] == "HOLD" and a["why"] == "stop"
    idle(base)


def test_reel_until_stop(base):
    base.transport.sim("free")                             # the dart let go
    mark = base.event_mark()
    base.cmd("reel", until="stop")
    base.wait_tel(lambda t: t["line_m"] is not None and t["line_m"] < 1.5, T(60), what="line_m under 1.5")
    base.cmd("stop")
    a = base.wait_event("ABORTED", 2, since=mark, fail_on_abort=False)
    assert a["what"] == "REEL" and a["why"] == "stop"
    idle(base)


def test_unwrap(base):
    mark = base.event_mark()
    base.cmd("unwrap")
    base.wait_event("MODE", T(60), since=mark)
    assert [x["evt"] for x in base.events(mark)] == ["UNWRAPPED", "MODE"]
    m = base.events(mark)[-1]
    assert m["ok"] and m["mode"] == "NEUTRAL" and m["drum_pos"] == "NEUTRAL"
    t = idle(base, pred=lambda t: t["drum_pos"] == "NEUTRAL")
    assert t["wraps"] == 0 and t["drum_deg"] == 41


def test_retrieve_and_back_to_launch(base):
    e = run(base, "retrieve", "SEATED", 60, duty=0.3)
    assert e["evt"] == "SEATED"
    t = idle(base, pred=lambda t: t["line_m"] == 0)
    assert t["sw"].get("S") == 1
    e = run(base, "mode", "MODE", 20, to="LAUNCH")
    assert e["ok"] and e["mode"] == "LAUNCH"
    assert base.cmd("status")["aim_known"] is False        # forgotten after the shift into LAUNCH


# ------------------------------------------------------------------------------------------------ safety
def test_estop_goes_to_neutral(base):
    mark = base.event_mark()
    base.cmd("estop")
    e = base.wait_event("MODE", T(15), since=mark)
    assert e["ok"] and e["drum_pos"] == "NEUTRAL"
    idle(base)
    run(base, "mode", "MODE", 15, to="LAUNCH")


def test_injected_fault_safing_and_clear(base):
    mark = base.event_mark()
    base.cmd("aim", deg=30)                                # aim unknown again after re-entering LAUNCH: homing run
    base.wait_tel(lambda t: t["state"] == "AIM" and t["rpm"] not in (None, 0), T(2), what="aim running")
    base.transport.sim("fault encMissing 1")              # the AS5600 drops out while the wiper runs
    f = base.wait_event("FAULT", T(2), since=mark)
    assert f["code"] == "ANGLE_SENSOR"
    base.wait_tel(lambda t: t["state"] == "SAFING", T(2), what="SAFING", fail_on_fault=False)
    m = base.wait_event("MODE", T(15), since=mark, fail_on_fault=False)
    assert m["ok"] and m["drum_pos"] == "NEUTRAL"         # launch train at rest: safed to NEUTRAL
    t = base.wait_tel(lambda t: t["state"] == "FAULT", 2, what="FAULT", fail_on_fault=False)
    assert t["fault"] == "ANGLE_SENSOR"
    assert refused(base, "mode", to="LAUNCH") == "FAULTED"
    assert refused(base, "clear") == "STILL_PRESENT"
    base.transport.sim("fault encMissing 0")
    time.sleep(T(0.2))
    base.cmd("clear")
    t = idle(base, fail_on_fault=False)                    # the telemetry before the clear still shows the fault
    assert t["fault"] is None and t["mode"] == "NEUTRAL"


def test_drum_jam_fault_holds(base):
    run(base, "mode", "MODE", 15, to="LAUNCH")
    base.transport.sim("fault drumJam 1")                 # the selector drum N20 does not turn
    mark = base.event_mark()
    base.cmd("mode", to="NEUTRAL")
    f = base.wait_event("FAULT", T(15), since=mark)
    assert f["code"] == "DRUM_TIMEOUT"
    m = base.wait_event("MODE", 2, since=mark, fail_on_fault=False, fail_on_not_ok=False)
    assert m["ok"] is False and m["err"] == "DRUM_HELD" and m["drum_pos"] == "LAUNCH"   # a drum fault never moves it
    base.transport.sim("fault drumJam 0")
    base.cmd("clear")
    t = idle(base, fail_on_fault=False)
    assert t["fault"] is None


# ------------------------------------------------------------------------------------------------ pty mode
def test_pty_mode_with_load_cell():
    pytest.importorskip("serial")
    p = subprocess.Popen([BIN] + scenario_args(speed=SPEED, load_cell=True, pty=True), stdin=subprocess.PIPE,
                         stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    try:
        path = p.stderr.readline().decode().strip()
        assert path.startswith("/dev/")
        c = BaseClient(path, timeout=1.0).start()
        try:
            assert c.wait_connected(5)
            assert c.cmd("ping")["fw"].startswith("base-1.")
            assert c.cmd("status")["hx"] is True
            t = c.wait_tel(lambda t: t["tension_n"] is not None, 3, what="load cell tension")
            assert list(t.keys()) == TEL_KEYS
        finally:
            c.close()
    finally:
        p.terminate()
        p.wait(5)
