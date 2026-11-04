"""Final cross-check (3 Nov, base-1.3.1): regressions for the Jetson findings jetson-1 to jetson-9, fw-9, fw-10 and
the firmware changes the mock must follow (SW_SEATED read at HOME with the cock latch, gap_close_deg, the CAD switch
windows, the drum code edges as built)."""
import threading
import time

import pytest

from grappstar.comms.base import BaseError, BaseTimeout
from grappstar.comms.dart import Dart
from grappstar.comms.mock_base import (AIM_HOME_DEG, FW, GAP_CLOSE_DEG, PARAMS_VERSION, ZONES, connected_pair)
from grappstar.jlog import JLog
from grappstar.mission import Mission, MissionConfig, approach_max_s, dart_seated

SCALE = 20.0


def W(sim_s):
    return sim_s / SCALE * 1.5 + 1.0


def err_of(fn, *a, **kw):
    with pytest.raises(BaseError) as ei:
        fn(*a, **kw)
    return ei.value.err


def run(c, name, evt, sim_s, **args):
    m = c.event_mark()
    c.cmd(name, **args)
    return c.wait_event(evt, W(sim_s), since=m)


@pytest.fixture
def pair():
    mb, c = connected_pair(time_scale=SCALE, drum_at="LAUNCH")
    c.wait_tel(lambda t: True, 2)
    yield mb, c
    c.close()
    mb.stop()


# ------------------------------------------------------------------ the mock follows base-1.3.1
def test_mock_version_edges_and_switch_defaults(pair):
    mb, c = pair
    assert FW == "base-1.3.1" and c.get("fw")["val"] == FW
    assert c.get("params")["val"] == PARAMS_VERSION and PARAMS_VERSION.startswith("1.4.")
    assert [z[2] for z in ZONES[:-1]] == [6.0, 34.0, 50.0, 62.0, 302.0, 342.0]      # params DRUM code_edges
    assert c.get("aim_home_deg")["val"] == AIM_HOME_DEG == 3502.0
    assert c.get("gap_close_deg")["val"] == GAP_CLOSE_DEG == 531.0
    assert err_of(c.set, "gap_close_deg", 470) == "OUT_OF_RANGE"
    c.set("gap_close_deg", 529.5)
    assert c.get("gap_close_deg")["val"] == 529.5
    c.cal("factory")
    assert c.get("gap_close_deg")["val"] == 531.0


def test_mock_refusals_follow_the_firmware(pair):
    """jetson-4: the drum and wiper jogs and the seat refusals of base_logic.h."""
    mb, c = pair
    run(c, "mode", "MODE", 60, to="NEUTRAL")
    assert err_of(c.jog, "wiper", -1, 200) == "BACKSTOP"           # backward only in LAUNCH
    run(c, "wrap", "WRAPPED", 90)
    c.wait_tel(lambda t: t["state"] == "IDLE" and t["drum_pos"] == "WRAPPED", 2)
    assert c.cmd("retrieve", check=False)["err"] == "WRAPS_ON"    # D39: never at the park
    assert c.cmd("reel", until="seated", check=False)["err"] == "WRAPS_ON"
    assert err_of(c.jog, "drum", -1, 200) == "WRAPS_ON"            # down from N_TO_R only by unwrap
    assert err_of(c.jog, "drum", 1, 200) == "SHIFTING"             # into REEL only by mode REEL
    with mb.lock:
        mb.drum_deg = 200.0                                         # stopped inside the wrap window
    c.wait_tel(lambda t: t["drum_pos"] == "BETWEEN", 2)
    assert err_of(c.jog, "drum", 1, 200) == "WRAPS_ON" and err_of(c.jog, "drum", -1, 200) == "WRAPS_ON"
    assert err_of(c.mode, "NEUTRAL") == "WRAPS_ON"
    m = c.event_mark()
    c.unwrap()
    c.wait_event("UNWRAPPED", W(60), since=m)
    m = c.event_mark()
    c.unwrap()                                                      # already at NEUTRAL: UNWRAPPED then MODE
    assert c.wait_event("UNWRAPPED", 1, since=m)["ok"] and c.wait_event("MODE", 1, since=m)["ok"]


def test_mock_aim_clamp_and_decock_at_rest(pair):
    """jetson-5: aim 6 clamped inside the table end, decock with the phase unknown at rest (the tow trip floor trip_a
    is checked in test_comms_base)."""
    mb, c = pair
    e = run(c, "aim", "AIMED", 60, deg=6.0)
    assert e["ok"] and abs(e["target_deg"] - (6.01 + 0.75 / 4)) < 0.02
    with mb.lock:
        mb.phase, mb.phase_hint = "UNKNOWN", 500.0                  # carriage home, gear in its gap
    m = c.event_mark()
    c.decock()
    e = c.wait_event("RESET", 1, since=m)
    assert e["ok"] is True and "SW_GAP closed" in e["why"] and c.status()["phase"] == "REST"
    with mb.lock:
        mb.phase, mb.phase_hint, mb.cock_deg = "UNKNOWN", 480.0, 480.0   # stopped in the reset: SW_GAP open
    m = c.event_mark()
    c.decock()
    e = c.wait_event("RESET", 1, since=m, fail_on_not_ok=False)
    assert e["ok"] is False and e["why"].startswith("SW_GAP open")


def test_mock_seated_contact_at_home_and_the_cock_latch(pair):
    """cad_support-3: SW_SEATED reads the dart only with the carriage home; the cock's reading counts while COCKED."""
    mb, c = pair
    run(c, "aim", "AIMED", 60, deg=30)
    t = c.wait_tel(lambda t: True, 1)
    assert t["sw"].get("S") == 1 and dart_seated(t) is True
    e = run(c, "cock", "COCKED", 20)
    assert abs(e["gap_deg"] - 22.9) < 0.5                            # 553.8 minus it: the CAD SW_GAP closing angle
    t = c.wait_tel(lambda t: t["state"] == "IDLE" and t["cock"] == 1, 2)
    assert "S" not in t["sw"] and "H" not in t["sw"] and dart_seated(t) is None
    assert c.arm()                                                   # the dart seated at the cock still counts
    mb.inject_fault("BAD_STATE")                                     # a fault clears the latch
    time.sleep(0.2)
    c.clear()
    assert c.status()["phase"] == "COCKED" and err_of(c.arm) == "NOT_SEATED"


def test_mock_reset_faults_on_a_gap_close_deg_below_the_cad_edge(pair):
    """fw-1: a gap_close_deg well below where SW_GAP really closes runs the reset into the first tooth."""
    mb, c = pair
    c.set("gap_close_deg", 520)
    run(c, "aim", "AIMED", 60, deg=30)
    m = c.event_mark()
    c.fire(c.arm())
    with pytest.raises(BaseError) as ei:
        c.wait_event("RESET", W(20), since=m)
    assert ei.value.err == "BAD_STATE" and "off its stop" in ei.value.reply.get("why", "")


def test_mock_refusals_match_basesim():
    """The same refusal probes against the real firmware logic (basesim) and the mock (jetson-4, jetson-5)."""
    from basesim_transport import find_basesim, start_basesim
    binary = find_basesim(build=True)
    if binary is None:
        pytest.skip("basesim not built and cannot be built (make -C jetson/tools/basesim)")
    speed = 20.0

    def ans(c, name, **a):
        r = c.cmd(name, check=False, **a)
        out = "ok" if r.get("ok") else r.get("err")
        time.sleep(0.25)
        if (c.tel or {}).get("state") not in ("IDLE", None):
            c.cmd("stop", check=False)
            time.sleep(0.4)
        return out

    def go(c, name, evt, sim_s, **a):
        m = c.event_mark()
        c.cmd(name, **a)
        return c.wait_event(evt, sim_s / speed * 2 + 3, since=m)

    def probe(c):
        res = {}
        go(c, "mode", "MODE", 30, to="NEUTRAL")
        time.sleep(0.3)
        res["NEUTRAL wiper jog -1"] = ans(c, "jog", motor="wiper", dir=-1, ms=200)
        go(c, "wrap", "WRAPPED", 90)
        time.sleep(0.5)
        res["park retrieve"] = ans(c, "retrieve", duty=0.3)
        res["park reel until seated"] = ans(c, "reel", duty=0.3, until="seated")
        res["park drum jog -1"] = ans(c, "jog", motor="drum", dir=-1, ms=200)
        res["park drum jog +1"] = ans(c, "jog", motor="drum", dir=1, ms=200)
        res["park wiper jog -1"] = ans(c, "jog", motor="wiper", dir=-1, ms=200)
        go(c, "unwrap", "UNWRAPPED", 90)
        time.sleep(0.5)
        m = c.event_mark()
        c.cmd("unwrap")
        time.sleep(0.5)
        res["NEUTRAL unwrap events"] = [e.get("evt") for e in c.events(m)]
        go(c, "mode", "MODE", 30, to="LAUNCH")
        time.sleep(0.3)
        e = go(c, "aim", "AIMED", 70, deg=6.0)
        res["LAUNCH aim 6.0"] = (e.get("ok"), e.get("target_deg"))
        res["get gap_close_deg"] = c.cmd("get", key="gap_close_deg", check=False).get("val")
        res["get aim_home_deg"] = c.cmd("get", key="aim_home_deg", check=False).get("val")
        res["get fw"] = c.cmd("get", key="fw", check=False).get("val")
        res["get params"] = c.cmd("get", key="params", check=False).get("val")
        return res

    proc, real = start_basesim(binary, speed=speed, line_out=0.0, seat_at=0.0)
    try:
        real.wait_tel(lambda t: True, 3)
        r_real = probe(real)
    finally:
        real.close()
    mb, c = connected_pair(time_scale=speed, drum_at="LAUNCH")
    try:
        c.wait_tel(lambda t: True, 3)
        r_mock = probe(c)
    finally:
        c.close()
        mb.stop()
    assert r_mock == r_real
    assert r_real["park retrieve"] == "WRAPS_ON" and r_real["park drum jog +1"] == "SHIFTING"
    assert r_real["NEUTRAL unwrap events"] == ["UNWRAPPED", "MODE"]


# ------------------------------------------------------------------ mission (jetson-1, -2, -3, fw-9)
class FakeBase:
    """Answers like base_logic.h with the drum stopped inside the wrap window and the wrap count unknown (a power cut
    during an unwrap or a wrap): mode and drum jogs WRAPS_ON, retrieve NOT_WRAPPED there, unwrap works."""

    def __init__(self, seated, line_m=None):
        self.sent = []
        self.evts = []
        self.lock = threading.Lock()
        self.session = 0
        sw = {"c": 1, "n": 1, "H": 1, "G": 1}
        if seated:
            sw["S"] = 1
        self.tel = {"mode": "NEUTRAL", "drum_pos": "BETWEEN", "state": "IDLE", "wraps": None, "sw": sw,
                    "line_m": line_m, "fault": None, "cock": 0, "pitch_deg": None}
        self.tel_t = time.time()
        self._run = True
        threading.Thread(target=self._tick, daemon=True).start()

    def _tick(self):
        while self._run:
            time.sleep(0.05)
            self.tel_t = time.time()

    def _evt(self, **e):
        with self.lock:
            self.evts.append(e)

    def event_mark(self):
        return len(self.evts)

    def events(self, since=0):
        return self.evts[since:]

    def wait_event(self, names, timeout, since=None, pred=None, session=None, fail_on_fault=True):
        names = (names,) if isinstance(names, str) else tuple(names)
        t_end = time.time() + timeout
        while time.time() < t_end:
            for e in self.evts[since or 0:]:
                if e.get("evt") in names and (pred is None or pred(e)):
                    if e.get("ok") is False:
                        raise BaseError("evt", e.get("err", "?"), e)
                    return e
            time.sleep(0.02)
        raise BaseTimeout("no " + str(names))

    def wait_tel(self, pred, timeout, what="", fail_on_fault=True, session=None):
        t_end = time.time() + timeout
        while time.time() < t_end:
            if pred(self.tel):
                return self.tel
            time.sleep(0.02)
        raise BaseTimeout(what)

    def cmd(self, name, timeout=None, check=True, **a):
        self.sent.append(name)
        t = self.tel
        in_wrap = t["drum_pos"] == "BETWEEN"
        if name in ("ping", "clear", "set"):
            return {"ok": True}
        if name == "get":
            return {"ok": True, "val": 0}
        if name == "status":
            return {"ok": True, "phase": "REST", "wraps_known": t["wraps"] is not None, "hx": False}
        if name == "stop":
            self._evt(evt="ABORTED")
            return {"ok": True}
        if name == "mode":
            if in_wrap and a["to"] in ("LAUNCH", "NEUTRAL"):
                raise BaseError("mode", "WRAPS_ON")
            t.update(mode=a["to"], drum_pos=a["to"])
            self._evt(evt="MODE", ok=True, mode=a["to"])
            return {"ok": True}
        if name == "unwrap":
            t.update(drum_pos="NEUTRAL", wraps=0, mode="NEUTRAL")
            t["sw"].pop("c", None)
            t["sw"]["b"] = 1
            self._evt(evt="UNWRAPPED", ok=True)
            self._evt(evt="MODE", ok=True, mode="NEUTRAL")
            return {"ok": True}
        if name == "wrap":
            t.update(drum_pos="WRAPPED", wraps=7)
            t["sw"]["a"] = 1
            self._evt(evt="WRAPPED", ok=True, wraps=7)
            return {"ok": True}
        if name in ("retrieve", "reel"):
            if in_wrap:
                raise BaseError(name, "NOT_WRAPPED")
            t["sw"]["S"] = 1
            t["line_m"] = 0
            self._evt(evt="SEATED")
            return {"ok": True}
        if name == "jog":
            raise BaseError("jog", "WRAPS_ON")
        raise BaseError(name, "UNKNOWN_CMD")

    def mode(self, to, **k): return self.cmd("mode", to=to, **k)
    def ping(self, **k): return self.cmd("ping", **k)
    def status(self, **k): return self.cmd("status", **k)
    def stop(self, **k): return self.cmd("stop", **k)
    def clear(self, **k): return self.cmd("clear", **k)
    def wrap(self, **k): return self.cmd("wrap", **k)
    def unwrap(self, **k): return self.cmd("unwrap", **k)
    def get(self, key, **k): return self.cmd("get", key=key, **k)
    def set(self, key, val, **k): return self.cmd("set", key=key, val=val, **k)
    def reel(self, duty, max_n=None, until="stop", **k): return self.cmd("reel", duty=duty, max_n=max_n, until=until, **k)

    def safe_stop(self):
        self.cmd("stop")
        return True


class FakeDart:
    last = {"phase": "IDLE"}

    def status(self):
        return {"phase": "IDLE"}

    def safe_stop(self, **k):
        return True


@pytest.mark.parametrize("seated,line_m", [(True, None), (False, None), (False, 1.2)])
def test_recover_from_a_drum_stopped_inside_the_wrap_window(tmp_path, seated, line_m):
    """jetson-1: a power cut during an unwrap leaves the drum inside WRAP with wraps null. Recover dart must unwrap
    (mode, drum jogs and retrieve are refused there) and end DONE."""
    b = FakeBase(seated, line_m)
    m = Mission(b, FakeDart(), {}, None, JLog(str(tmp_path / "m.jsonl")), cfg=MissionConfig(host_timeout_s=0),
                analyse=lambda *a, **k: {})
    m.start(recover=True)
    t_end = time.time() + 20
    while m.running() and time.time() < t_end:
        p = m.prompt
        if p is not None and p.answer is None:
            m.answer(p.id, "Base may move" if "Base may move" in p.choices else "Abort")
        time.sleep(0.02)
    m.thread.join(5)
    b._run = False
    assert "unwrap" in b.sent and m.state == "DONE", f"{m.state}: {m.error} / {b.sent}"
    assert b.tel["drum_pos"] == "LAUNCH"


def test_wraps_maybe_on():
    f = Mission.wraps_maybe_on
    assert f({"drum_pos": "BETWEEN", "wraps": None, "sw": {"c": 1}})
    assert f({"drum_pos": "BETWEEN", "wraps": 0, "sw": {"c": 1}})       # WRAP sector with a 0 estimate
    assert not f({"drum_pos": "BETWEEN", "wraps": 0, "sw": {"a": 1, "b": 1}})   # L_TO_N
    assert not f({"drum_pos": "NEUTRAL", "wraps": None, "sw": {"b": 1}})
    assert f({"drum_pos": "WRAPPED", "wraps": 7, "sw": {"a": 1, "c": 1}})


def test_rail_low_offers_decock_and_reaims_low(tmp_path):
    """jetson-2: a launch phase left UNKNOWN by a fault in the reset: RAIL_LOW offers Decock and re-aim, and the re-aim
    goes to the D45 low pitch, never back up to the shot pitch."""
    mb, c = connected_pair(time_scale=SCALE, drum_at="LAUNCH")
    try:
        c.wait_tel(lambda t: True, 2)
        run(c, "aim", "AIMED", 60, deg=35)
        with mb.lock:
            mb.phase, mb.phase_hint = "UNKNOWN", 480.0     # fault mid-reset, the gear back in its gap, carriage home
        cfg = MissionConfig(host_timeout_s=0, aim_timeout_s=10, reset_timeout_s=2)
        m = Mission(c, FakeDart(), {}, None, JLog(str(tmp_path / "m.jsonl")), cfg=cfg, analyse=lambda *a, **k: {})
        m.shot = {"pitch_deg": 35.0}
        m._sequence = lambda: m._do("RAIL_LOW", m.rail_low)
        m.start()
        seen = []
        t_end = time.time() + 30
        while m.running() and time.time() < t_end:
            p = m.prompt
            if p is not None and p.answer is None and p.id not in [s[0] for s in seen]:
                seen.append((p.id, list(p.choices)))
                m.answer(p.id, "Decock and re-aim" if "Decock and re-aim" in p.choices else "Abort")
            time.sleep(0.02)
        m.thread.join(5)
        aims = [r["deg"] for r in mb.received if r.get("cmd") == "aim"]
        assert any("Decock and re-aim" in ch for _, ch in seen), seen
        assert m.state == "DONE", f"{m.state}: {m.error}"
        assert aims[-1] < 7 and 35 not in aims[1:], aims
        assert c.status()["phase"] == "REST"
    finally:
        c.close()
        mb.stop()


def test_wrap_timeout_and_approach_default():
    """jetson-3: wrap and unwrap waits at least 90 s; fw-9: the BASE_SPEC 10.1 step 17 default, 30 s unknown."""
    assert MissionConfig().wrap_timeout_s >= 90
    assert approach_max_s(None, 0.3) == 30.0
    assert approach_max_s(1.5, 0.3) == pytest.approx(10 + 2.25 / 0.057, abs=0.06)


def test_dart_seated_reads_only_at_home():
    assert dart_seated({"sw": {"H": 1, "S": 1}}) is True
    assert dart_seated({"sw": {"H": 1}}) is False
    assert dart_seated({"sw": {"G": 1}}) is None                     # carriage drawn: unreadable
    assert dart_seated({}) is None


# ------------------------------------------------------------------ dart polling (fw-10)
def test_wait_phase_reads_the_poller_cache():
    """fw-10: with the App's 5 Hz poller running, wait_phase reads its cache instead of adding GETs (at most the first
    read, while the cache still predates the wait: a status from before the command never counts)."""
    from grappstar.comms.dart import DartError
    d = Dart("127.0.0.1:9", timeout=0.2, retries=0)
    gets = []

    def no_dart(path, timeout=None, retries=None):
        gets.append(path)
        raise DartError("no dart here")
    d._get = no_dart
    stop = threading.Event()

    def poller():                                                    # stands in for DartPoller
        while not stop.is_set():
            with d._lock:
                d.last, d.last_t = {"phase": "ANCHORED", "seq": "anchored", "event": "x"}, time.time()
            time.sleep(0.05)
    threading.Thread(target=poller, daemon=True).start()
    try:
        time.sleep(0.1)
        assert d.wait_phase("ANCHORED", 2.0)["phase"] == "ANCHORED"
        assert len(gets) <= 1, gets
        assert d.status_cached(0.3)["phase"] == "ANCHORED" and len(gets) <= 1
    finally:
        stop.set()
    time.sleep(0.1)
    with d._lock:
        d.last_t = time.time() - 1.0                                 # stale: a real GET
    with pytest.raises(DartError):
        d.status_cached(0.3)


# ------------------------------------------------------------------ the page (jetson-8, jetson-9, jetson-1)
def test_page_drag_range_pivot_hint_and_unwrap_button():
    import os
    p = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "grappstar", "ui", "static",
                     "index.html")
    html = open(p, encoding="utf-8").read()
    assert 'id="in-drag" type="number" inputmode="decimal" min="0.5" max="1"' in html
    assert "under the rail pivot" in html and "D29" in html
    assert 'data-action="unwrap"' in html
