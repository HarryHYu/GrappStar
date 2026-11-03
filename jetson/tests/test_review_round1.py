"""Independent review: tests that demonstrate real bugs. Each test asserts the SAFE/CORRECT behaviour, so a failure
is the evidence of the bug described in its docstring."""
import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from grappstar.app import App
from grappstar.comms.base import BaseClient, pipe_pair
from grappstar.comms.dart import Dart, DartError
from grappstar.comms.mock_base import MockBase, World, DRUM_DEG, WRAP_DEG, N_WRAPS
from grappstar.jlog import JLog
from grappstar.mission import Mission, MissionConfig, StepFailed


# ---------------------------------------------------------------- fakes
class FakeDart:
    """Dart stuck in ANCHORING with motors on; wait_phase times out."""

    def __init__(self, phase="ANCHORING", release_raises=False):
        self.last = {"phase": phase, "current_mA": 420}
        self.stops = 0
        self.release_raises = release_raises
        self.on_line = None

    def status(self):
        return dict(self.last)

    def auto(self):
        return "auto"

    def wait_phase(self, phases, timeout, **kw):
        raise DartError("dart did not reach ANCHORED in 60 s")

    def release(self):
        if self.release_raises:
            raise DartError("dart unreachable at http://192.168.4.1/release: timeout")
        return "release"

    def stop(self):
        self.stops += 1
        return "stopped"

    def safe_stop(self):
        self.stops += 1
        return True


class FakeBase:
    tel = {}
    tel_t = 0.0

    def __init__(self):
        self.stops = 0

    def safe_stop(self):
        self.stops += 1
        return True

    def clear(self):
        return {"ok": True}


def _mission(base, dart, tmp_path, **kw):
    return Mission(base, dart, {}, None, JLog(str(tmp_path / "m.jsonl")), cfg=MissionConfig(**kw),
                   analyse=lambda *a, **k: {}, run_dir=None)


def _mock_pair(scale=8.0):
    a, b = pipe_pair()
    mb = MockBase(a, time_scale=scale, world=World()).start()
    c = BaseClient(b, reconnect_s=0.05).start()
    c.wait_connected(2)
    c.wait_tel(lambda t: True, 2)
    return mb, c


# ---------------------------------------------------------------- 1. dart motors left running on anchor timeout
def test_anchor_timeout_stops_dart_before_prompt(tmp_path):
    """mission.py:274-280 + 583: anchor() turns the wait_phase DartError into StepFailed, so _do never calls
    dart.safe_stop(); the dart keeps drilling (ANCHORING, 420 mA) while the Retry prompt waits for the operator."""
    dart = FakeDart()
    m = _mission(FakeBase(), dart, tmp_path)
    seen = {}

    def ask(text, choices, danger=False, timeout=None):
        seen["dart_stops_at_prompt"] = dart.stops
        raise RuntimeError("stop test here")

    m.ask = ask
    with pytest.raises(RuntimeError):
        m._do("ANCHOR", m.anchor)
    assert seen["dart_stops_at_prompt"] >= 1, "dart still ANCHORING with motors on while the operator is asked"


# ---------------------------------------------------------------- 2. release_dart outside _do
def test_release_and_reshoot_release_failure_is_retryable(tmp_path):
    """mission.py:324-326: after a failed anchor, "Release and reshoot" calls release_dart() outside _do, so a DartError
    or StepFailed from the release ends the whole mission as FAILED instead of offering Retry."""
    dart = FakeDart(phase="FAULT", release_raises=True)
    base = FakeBase()
    m = _mission(base, dart, tmp_path)
    for n in ("preflight", "aim", "arm", "fire", "wait_landing"):
        setattr(m, n, lambda: None)
    m.base_mode = lambda to: None
    m.landing_zone = lambda: "anchor"

    def anchor():
        raise StepFailed("dart did not anchor", choices=["Retry", "Release and reshoot"])
    m.anchor = anchor
    asked = []

    def ask(text, choices, danger=False, timeout=None):
        asked.append((text, choices))
        if "Release and reshoot" in choices:
            return "Release and reshoot"
        from grappstar.mission import Aborted
        raise Aborted("test done")
    m.ask = ask
    m.start()
    m.thread.join(5)
    assert m.state != "FAILED", f"mission crashed with no Retry offered: {m.error}"


# ---------------------------------------------------------------- 3. non-JSON /status escapes DartError
class _BadJson(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        body = b'{"phase":"ANCH'      # truncated body (WiFi drop, firmware glitch)
        self.send_response(200)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def test_dart_bad_json_raises_darterror():
    """dart.py:101: json.loads is not guarded, so a malformed body raises JSONDecodeError (ValueError). Every caller
    (wait_phase misses, wait_landing, anchor, release_dart, Mission._do) only catches DartError, so one bad reply
    ends the mission as FAILED (for example mid-flight or mid-anchoring)."""
    srv = ThreadingHTTPServer(("127.0.0.1", 0), _BadJson)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        d = Dart(f"127.0.0.1:{srv.server_address[1]}", timeout=0.5)
        with pytest.raises(DartError):
            d.status()
    finally:
        srv.shutdown(); srv.server_close()


# ---------------------------------------------------------------- 4. ESTOP delayed by an unreachable dart
def test_estop_reaches_base_fast_when_dart_unreachable():
    """app.py:338: act_estop calls dart.safe_stop() BEFORE base.estop(). Dart.stop uses retries=2 with a 1 s timeout,
    so with the dart out of WiFi range the base ESTOP is delayed by about 3 s."""
    blackhole = socket.socket()
    blackhole.bind(("127.0.0.1", 0))
    blackhole.listen(16)                       # accepts at kernel level, never answers
    mb, c = _mock_pair()
    try:
        app = App.__new__(App)
        app.mission = None
        app.base = c
        app.dart = Dart(f"127.0.0.1:{blackhole.getsockname()[1]}", timeout=1.0)
        t0 = time.time()
        t_estop = {}
        orig = c.estop

        def estop(**kw):
            t_estop["t"] = time.time()
            return orig(**kw)
        c.estop = estop
        app.act_estop()
        assert t_estop["t"] - t0 < 0.5, f"base ESTOP sent {t_estop['t'] - t0:.1f} s after the button"
    finally:
        c.close(); mb.stop(); blackhole.close()


# ---------------------------------------------------------------- 6. STOP is not atomic with the fire sequence
def test_stop_after_arm_prevents_fire(tmp_path):
    """mission.py:460-463 + app.py:326-333: act_stop sets cancel and sends stop, but the mission thread that already
    passed check_cancel() still sends fire. The base starts cocking after the operator pressed STOP (stopped again
    only later by _safe_stop)."""
    mb, c = _mock_pair()
    with mb.lock:
        mb.mode = "LAUNCH"
        mb.drum_deg = DRUM_DEG["LAUNCH"]
    c.wait_tel(lambda t: t.get("drum_pos") == "LAUNCH", 2)
    m = _mission(c, FakeDart(phase="IDLE"), tmp_path)
    m.ask = lambda text, choices, **k: choices[0]
    orig_arm = c.arm

    def arm(**kw):
        tok = orig_arm(**kw)
        # operator presses STOP right now (app.act_stop)
        m.cancel.set()
        c.safe_stop()
        return tok
    c.arm = arm
    try:
        m.fire()
    except Exception:
        pass
    try:
        names = [r["cmd"] for r in mb.received]
        i_stop = len(names) - 1 - names[::-1].index("stop")
        assert "fire" not in names[i_stop:], f"fire sent after STOP: {names}"
    finally:
        c.close(); mb.stop()


# ---------------------------------------------------------------- 7. lost FIRED event: Retry can never succeed
def test_fire_retry_after_lost_fired_event_recovers(tmp_path):
    """mission.py:454-466 + 295-297: if the base fired but the FIRED event is missed (link glitch, brownout reset),
    FIRE fails and Retry re-runs fire(): arm now fails NOT_SEATED forever (the dart is already out on the line).
    The only exit is Abort, with the dart deployed and no RETRIEVE."""
    mb, c = _mock_pair()
    with mb.lock:
        mb.mode = "LAUNCH"
        mb.drum_deg = DRUM_DEG["LAUNCH"]
    orig_evt = mb.evt
    mb.evt = lambda name, **kw: None if name == "FIRED" else orig_evt(name, **kw)
    c.wait_tel(lambda t: t.get("drum_pos") == "LAUNCH", 2)
    m = _mission(c, FakeDart(phase="IDLE"), tmp_path, fire_timeout_s=2.0)
    prompts = []

    def ask(text, choices, danger=False, timeout=None):
        prompts.append(text)
        if "Retry" in choices:
            if sum("FIRE failed" in p for p in prompts) >= 2:
                from grappstar.mission import Aborted
                raise Aborted("stuck")
            return "Retry"
        return choices[0]
    m.ask = ask
    try:
        from grappstar.mission import Aborted
        try:
            m._do("FIRE", m.fire)
        except Aborted:
            pass
        fails = [p for p in prompts if "FIRE failed" in p]
        assert not any("NOT_SEATED" in p for p in fails), f"retry loop after a real shot: {fails}"
    finally:
        c.close(); mb.stop()


# ---------------------------------------------------------------- 8. CLI reel sends a non-C1 field and never stops
def test_cli_base_reel_is_bounded(monkeypatch):
    """cli.py:131: `base reel` sends {"until": "stop", "ms": 1000}. C1 reel has no ms field, so a C1 firmware reels
    until a stop command, and cmd_base then closes the port without sending stop."""
    from grappstar import cli
    sent = []

    class C:
        tel = None

        def cmd(self, name, **kw):
            sent.append((name, kw))
            return {"ok": True}

        def close(self):
            pass
    monkeypatch.setattr(cli, "_base_client", lambda a: C())
    cli.main(["base", "reel", "0.5"])
    names = [n for n, _ in sent]
    reel_kw = sent[0][1]
    assert "ms" not in reel_kw or "stop" in names, f"unbounded reel per C1: {sent}"


# ---------------------------------------------------------------- 8. auto-anchor + reshoot reels against an anchored dart
def test_auto_anchor_reshoot_releases_first(tmp_path):
    """mission.py:319-322 + 691-699: with auto_anchor_on_landing the dart anchors itself on landing. A NO GO verdict
    (or "Retrieve and reshoot") calls reshoot(), which never releases the dart: RETRIEVE reels against the anchor up to
    retrieve_limit_n and reports a "snag"."""
    from mission_harness import Rig
    r = Rig(tmp_path, verdict="NO_GO", cfg=MissionConfig(auto_anchor_on_landing=True))
    r.policy["snag"] = "Abort"
    r.policy["Dart is home"] = "Abort"
    try:
        r.run(90)
        texts = [t for t, _ in r.answers]
        paths = [p for _, p, _ in r.md.requests]
        assert not any("snag" in t for t in texts) and "/release" in paths, \
            f"retrieved against an anchored dart: {texts[-2:]}"
    finally:
        r.close()
