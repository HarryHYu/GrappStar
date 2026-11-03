"""Review round 4: tests that show real bugs in grappstar/mission.py. Each test FAILS on the code as reviewed and
passes once the finding is fixed. A small fake base (no serial, no simulator) keeps each test under a few seconds."""
import threading
import time

from grappstar.comms.base import BaseError, BaseTimeout
from grappstar.jlog import JLog
from grappstar.mission import Mission, MissionConfig


class FakeBase:
    """Just enough of BaseClient for Mission: records every command, telemetry under a condition variable."""

    def __init__(self, tel):
        self.cv = threading.Condition()
        self.tel = dict(tel)
        self.tel_t = time.time()
        self.session = 0
        self.connected = True
        self.sent = []                  # command names, in order
        self.vals = {"host_timeout_s": 0}
        self.in_wait_tel = threading.Event()
        self.raise_on = {}              # cmd name -> exception to raise
        self._evs = []

    # commands
    def cmd(self, name, timeout=None, check=True, **kw):
        with self.cv:
            self.sent.append(name)
        if name in self.raise_on:
            raise self.raise_on[name]
        if name == "mode":
            if (self.tel.get("wraps") or 0) > 0 and kw.get("to") != "REEL":
                raise BaseError("mode", "WRAPS_ON")
        if name == "stop":
            with self.cv:
                self.tel["state"] = "IDLE"
                self.tel_t = time.time()
                self.cv.notify_all()
        if name == "set":
            self.vals[kw["key"]] = kw["val"]
        if name == "get":
            return {"ok": True, "val": self.vals.get(kw["key"])}
        if name == "ping":
            return {"ok": True, "fw": "fake"}
        if name == "status":
            return dict(self.tel, ok=True)
        return {"ok": True}

    def ping(self, **kw): return self.cmd("ping", **kw)
    def status(self, **kw): return self.cmd("status", **kw)
    def stop(self, **kw): return self.cmd("stop", **kw)
    def clear(self, **kw): return self.cmd("clear", **kw)
    def mode(self, to, **kw): return self.cmd("mode", to=to, **kw)
    def aim(self, deg, **kw): return self.cmd("aim", deg=deg, **kw)
    def wrap(self, **kw): return self.cmd("wrap", **kw)
    def unwrap(self, **kw): return self.cmd("unwrap", **kw)
    def set(self, key, val, **kw): return self.cmd("set", key=key, val=val, **kw)
    def get(self, key, **kw): return self.cmd("get", key=key, **kw)

    def safe_stop(self):
        self.stop()
        return True

    # events / telemetry
    def event_mark(self):
        return len(self._evs)

    def events(self, since=0):
        return self._evs[since:]

    def wait_event(self, names, timeout, since=None, **kw):
        time.sleep(min(timeout, 0.05))
        raise BaseTimeout("fake: no events")

    def wait_tel(self, pred, timeout, what="", fail_on_fault=True, session=None):
        t_end = time.monotonic() + timeout
        with self.cv:
            self.in_wait_tel.set()
            while True:
                if pred(self.tel):
                    return self.tel
                left = t_end - time.monotonic()
                if left <= 0:
                    raise BaseTimeout(what)
                self.cv.wait(left)


class FakeDart:
    last = {"phase": "IDLE"}
    last_t = 0.0

    def status(self):
        return {"phase": "IDLE"}

    def safe_stop(self, fast=False):
        return True

    def clock_offset(self):
        return None


def _mission(tmp_path, base, **cfg):
    c = MissionConfig(**cfg)
    return Mission(base, FakeDart(), {}, None, JLog(str(tmp_path / "m.jsonl")), cfg=c,
                   analyse=lambda *a, **k: {})


def _operator(m, answers, stop):
    """Answer each open prompt with the first matching choice from answers (else the last choice)."""
    def run():
        seen = set()
        while not stop.is_set():
            p = m.prompt
            if p is not None and p.answer is None and p.id not in seen:
                seen.add(p.id)
                pick = next((a for a in answers if a in p.choices), p.choices[-1])
                m.answer(p.id, pick)
            time.sleep(0.01)
    th = threading.Thread(target=run, daemon=True)
    th.start()
    return th


# ------------------------------------------------------------------ R4-1: a failed "Decock and re-aim" is FAILED
def test_decock_and_reaim_failure_is_a_prompt_not_failed(tmp_path):
    """_do calls decock_and_reaim() from inside its except handler, outside any try: a decock that times out (or is
    refused) escapes _do and the whole mission ends FAILED instead of asking Retry / Abort again."""
    base = FakeBase({"state": "IDLE", "mode": "LAUNCH", "drum_pos": "LAUNCH", "phase": "UNKNOWN"})
    base.raise_on["decock"] = BaseTimeout("no reply to decock in 1.0 s")
    m = _mission(tmp_path, base, host_timeout_s=0)

    def cock():
        raise BaseError("cock", "PHASE_UNKNOWN")
    m._sequence = lambda: m._do("COCK", cock)
    stop = threading.Event()
    picks = iter(["Decock and re-aim", "Abort"])        # first prompt: try the decock; the re-prompt: Abort

    def run():
        seen = set()
        while not stop.is_set():
            p = m.prompt
            if p is not None and p.answer is None and p.id not in seen:
                seen.add(p.id)
                m.answer(p.id, next(picks, "Abort"))
            time.sleep(0.01)
    threading.Thread(target=run, daemon=True).start()
    m.start()
    m.thread.join(10)
    stop.set()
    assert m.state == "ABORTED", f"{m.state}: {m.error}"    # asked again after the failed decock, not FAILED


# ------------------------------------------------------------------ R4-2: STOP makes base_mode's wait true, then mode
def test_stop_during_base_mode_wait_sends_no_motion(tmp_path):
    """base_mode waits for state IDLE/HOLD/FAULT before sending mode. The page STOP (act_stop: mission.abort, then
    base stop) itself makes the base IDLE, which ends that wait, and the mission thread then sends "mode": the
    drum starts moving after STOP (until _safe_stop catches it a moment later)."""
    base = FakeBase({"state": "MODE", "mode": "NONE", "drum_pos": "BETWEEN"})   # drum still settling
    m = _mission(tmp_path, base, host_timeout_s=0, mode_timeout_s=5)
    m._sequence = lambda: m.base_mode("LAUNCH")
    m.start()
    assert base.in_wait_tel.wait(3)
    time.sleep(0.05)
    # App.act_stop
    m.abort("operator STOP")
    base.safe_stop()
    i_stop = len(base.sent)
    m.thread.join(5)
    after = base.sent[i_stop:]
    assert "mode" not in after, f"motion command after STOP: {after}"


# ------------------------------------------------------------------ R4-3: host_timeout_s restore keeps a leaked value
def test_host_timeout_not_left_on_after_a_crashed_mission(tmp_path):
    """The keep-alive restores whatever host_timeout_s it read at start. If an earlier mission never restored it (the
    App crashed or was killed, which is exactly what the watchdog is for), the next mission reads the mission value
    1.5 and puts 1.5 back: from then on every manual move from the page longer than 1.5 s (mode, aim, decock) is
    ABORTED "host silent", because nothing pings the base outside a mission. The value is not in NVS, so it lasts
    until the base is power cycled."""
    base = FakeBase({"state": "IDLE", "mode": "LAUNCH", "drum_pos": "LAUNCH"})
    crashed = _mission(tmp_path, base, host_timeout_s=1.5)
    crashed._keepalive_start()          # mission 1: the process dies here (no _keepalive_stop)
    crashed._ka_stop.set()
    assert base.vals["host_timeout_s"] == 1.5
    m = _mission(tmp_path, base, host_timeout_s=1.5)
    m._sequence = lambda: None          # mission 2 after the App restart, ends normally
    m.start()
    m.thread.join(5)
    assert base.vals["host_timeout_s"] in (0, None), \
        f"host watchdog left at {base.vals['host_timeout_s']} s with no keep-alive running"


# ------------------------------------------------------------------ R4-4: recover with the dart seated but wrapped
def test_recover_unwraps_a_seated_dart(tmp_path):
    """The page offers Recover when the wraps are on even with the dart seated (abort during UNWRAP after the capstan
    seated it). _recover_sequence skips everything when SW_SEATED is closed and goes straight to LAUNCH_READY:
    mode LAUNCH is refused WRAPS_ON, and Retry repeats the same refusal forever. The page has no unwrap button."""
    base = FakeBase({"state": "IDLE", "mode": "NONE", "drum_pos": "WRAPPED", "wraps": 4.0, "sw": {"H": 1, "S": 1}})
    m = _mission(tmp_path, base, host_timeout_s=0)
    stop = threading.Event()
    _operator(m, ["Base may move", "Abort"], stop)
    m.start(recover=True)
    m.thread.join(10)
    stop.set()
    assert "unwrap" in base.sent, f"recover never unwraps: {base.sent} ({m.state}: {m.error})"
