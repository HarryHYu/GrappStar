"""Dart client and mission dart steps against the v4 API rules (firmware/dart/API.md), dart v31 round.

/status at most 5 Hz in total, WiFi gaps ridden out by time, the first-press stall lock, waiting for the frozen
flight log, preflight dart battery and sensor checks, the lens and capture delay handed to the analysis, and fault
codes in plain words.
"""
import threading
import time

import pytest

from grappstar.comms.dart import Dart, DartError, DartPoller, DartRefused, STATUS_MIN_PERIOD_S
from grappstar.comms.mock_dart import MockDart


@pytest.fixture
def md():
    m = MockDart(time_scale=5.0).start()
    yield m
    m.stop()


def _status_gets(md, t0, t1):
    return [r for r in md.requests if r[1] == "/status" and t0 <= r[0] <= t1]


def test_status_never_faster_than_5hz_in_total(md):
    """API.md: never poll /status faster than 5 Hz, whoever asks (poller, mission waits, page)."""
    d = Dart(md.host)
    p = DartPoller(d).start()
    stop = threading.Event()

    def hammer():
        while not stop.is_set():
            d.status()

    ths = [threading.Thread(target=hammer, daemon=True) for _ in range(3)]
    t0 = time.time()
    for t in ths:
        t.start()
    time.sleep(2.0)
    stop.set()
    p.stop()
    for t in ths:
        t.join(2)
    t1 = time.time()
    gets = sorted(r[0] for r in _status_gets(md, t0, t1))
    rate = len(gets) / (t1 - t0)
    assert rate <= 5.5, rate
    gaps = [b - a for a, b in zip(gets, gets[1:])]
    assert min(gaps) >= STATUS_MIN_PERIOD_S - 0.05        # spaced at sending; arrival jitters a little
    # a status right after a command is still fresh (spaced, not cached): arm() returns the new phase
    s = d.arm(True, auto=False)
    assert s["phase"] == "ARMED"


def test_wait_phase_rides_a_wifi_gap_then_gives_up_by_time(md):
    d = Dart(md.host, timeout=0.3, retries=0)
    d.status()
    md.gap_until = time.time() + 2.5          # 2.5 s with no answer at all
    t0 = time.time()
    s = d.wait_phase(("IDLE",), 10, poll=0.2, gap_s=6.0)
    assert s["phase"] == "IDLE" and time.time() - t0 >= 2.0       # it waited the gap out
    md.gap_until = time.time() + 30
    t0 = time.time()
    with pytest.raises(DartError, match="not answering for 3 s"):
        d.wait_phase(("ANCHORED",), 20, poll=0.2, gap_s=3.0)
    assert 2.5 < time.time() - t0 < 6.0
    md.gap_until = 0


def test_hold_first_press_stall_lock_is_retried_once(monkeypatch):
    """A 'stalled' on the very first press is the lock of an earlier stall: let go, press again (not the end)."""
    d = Dart("127.0.0.1:9")
    sent = []
    answers = iter([DartRefused("/key", "stalled: release the key first"), "", "", "",
                    DartRefused("/key", "stalled: release the key first")])

    def key(c, ms=350):
        sent.append((time.time(), c))
        a = next(answers)
        if isinstance(a, Exception):
            raise a
        return a
    stops = []
    monkeypatch.setattr(d, "key", key)
    monkeypatch.setattr(d, "stop", lambda disarm=True: stops.append(disarm))
    t0 = time.time()
    d.hold("@", 3.0, period=0.05)
    assert len(sent) == 5 and stops == [False]          # moved, ended at the real end stop, /stop?disarm=0
    assert sent[1][0] - sent[0][0] >= 0.6                # waited out the release window before pressing again
    # locked twice in a row: raise (someone else holds that key), and no /stop (nothing moved)
    answers2 = iter([DartRefused("/key", "stalled: release the key first")] * 2)
    monkeypatch.setattr(d, "key", lambda c, ms=350: (_ for _ in ()).throw(next(answers2)))
    stops.clear()
    with pytest.raises(DartRefused, match="first press twice"):
        d.hold("@", 3.0, period=0.05)
    assert stops == []


def test_flightlog_read_waits_for_the_frozen_log(tmp_path):
    from mission_harness import Rig
    r = Rig(tmp_path)
    try:
        m = r.m
        r.md.freeze_s = 20.0                  # dart seconds; the mock runs 20x fast: 1 s of wall time
        r.dart.arm(True, auto=False)
        r.md.launch()
        r.md.land()
        assert not r.dart.status()["log_frozen"]
        t0 = time.time()
        m._wait_log_frozen(timeout=5.0)
        assert r.dart.status()["log_frozen"]
        assert 0.5 < time.time() - t0 < 2.5
        # a firmware without the field (older) is not waited for
        r.md.freeze_s = 1e9
        r.md.land()
        t0 = time.time()
        m._wait_log_frozen(timeout=0.8)        # times out with a warning, never raises
        assert time.time() - t0 < 2.0
    finally:
        r.close()


def test_preflight_dart_battery_and_sensors(tmp_path):
    from grappstar.mission import StepFailed
    from mission_harness import Rig
    r = Rig(tmp_path)
    try:
        r.md.ina_ok = False
        with pytest.raises(StepFailed, match="INA219"):
            r.m.preflight()
        r.md.ina_ok = True
        r.md.rail_V = 5.8
        with pytest.raises(StepFailed, match="battery flat"):
            r.m.preflight()
        r.md.rail_V = 6.4
        r.auto_answer = True
        asked = []
        orig = r.m.ask

        def ask(text, choices, **kw):
            asked.append(text)
            return choices[0]
        r.m.ask = ask
        r.m.preflight()
        assert any("dart battery low (6.40 V" in a for a in asked), asked
        r.m.ask = orig
    finally:
        r.close()


def test_landing_zone_hands_lens_and_delay_to_the_analysis(tmp_path):
    from grappstar.mission import MissionConfig
    from grappstar.vision.lens import DartLens
    from mission_harness import Rig
    lens = DartLens.equidistant(140.0)
    cfg = MissionConfig(dart_lens=lens, video_latency_s=0.07)
    r = Rig(tmp_path, cfg=cfg)
    got = {}
    try:
        def analyse(frames, fl, outdir, **kw):
            got.update(kw)
            return {"terrain": {"cls": "hard_ground", "score": 80, "verdict": "GO", "reasons": []},
                    "flight": {}, "mosaic": None, "files": {}, "errors": []}
        r.m.analyse = analyse
        r.m.shot = {"t_fire": time.time(), "pitch_deg": 30.0, "dart_phase_at_arm": "ARMED"}
        r.dart.arm(True, auto=False)
        r.md.launch()
        r.md.land()
        r.m.ask = lambda text, choices, **kw: choices[0]
        assert r.m.landing_zone() == "anchor"
        assert got["dart_lens"] is lens and got["video_latency_s"] == 0.07 and got["aim_pitch_deg"] == 30.0
    finally:
        r.close()


def test_fault_codes_in_plain_words():
    from grappstar.mission import dart_fault_text
    assert "pods jammed" in dart_fault_text("dart FAULT PODS_JAM: release: pods stalled")
    assert "back or side" in dart_fault_text("dart FAULT NO_GROUND: ...")
    assert "could not right itself" in dart_fault_text("dart FAULT NOT_UPRIGHT: auto: not upright after 6 righting moves")
    assert "righting leg" in dart_fault_text("dart FAULT KICK_TIMEOUT: auto: leg did not reach its fold stop")
    assert dart_fault_text("dart did not reach ANCHORED in 60 s") == "dart did not reach ANCHORED in 60 s"


def test_mock_dart_freezes_the_log_3_s_after_landing(md):
    d = Dart(md.host)
    d.arm(True, auto=False)
    md.launch()
    md.land()
    fl = d.flightlog()
    assert fl["frozen"] is False and d.status()["log_frozen"] is False
    time.sleep(3.0 / md.scale + 0.25)
    assert d.status()["log_frozen"] and d.flightlog()["frozen"]


def test_landing_zone_waits_out_a_wifi_gap_for_the_flight_log(tmp_path):
    """A WiFi drop right after landing must not lose the flight log (it stays on the dart until the next arm)."""
    from mission_harness import Rig
    r = Rig(tmp_path)
    got = {}
    try:
        def analyse(frames, fl, outdir, **kw):
            got["fl"] = fl
            return {"terrain": {"cls": "hard_ground", "score": 80, "verdict": "GO", "reasons": []},
                    "flight": {}, "mosaic": None, "files": {}, "errors": []}
        r.m.analyse = analyse
        r.m.ask = lambda text, choices, **kw: choices[0]
        s = r.dart.arm(True, auto=False)
        r.m.shot = {"t_fire": time.time(), "pitch_deg": 30.0, "dart_phase_at_arm": "ARMED",
                    "dart_t_ms_at_arm": s["t_ms"]}
        r.md.launch()
        r.md.land()
        time.sleep(0.4)
        r.md.gap_until = time.time() + 3.0
        t0 = time.time()
        r.m.landing_zone()
        assert got["fl"] is not None and got["fl"].get("samples"), "flight log lost in the WiFi gap"
        assert time.time() - t0 >= 2.0
    finally:
        r.md.gap_until = 0
        r.close()


def test_v43_righting_then_not_upright_offers_force_and_battery_critical(tmp_path):
    """Dart v4.3 (v34): on its side or back /auto rights it by itself (no refusal, no prompt); a dart it cannot right
    ends in NOT_UPRIGHT, where the operator sees the video and may force the anchor; battery critical refuses."""
    from grappstar.mission import StepFailed
    from mission_harness import Rig
    r = Rig(tmp_path)
    try:
        asked = []

        def ask(text, choices, **kw):
            asked.append((text, choices))
            return "Force anchor"
        r.m.ask = ask
        for orient in ("side", "inverted"):
            r.md.orient = orient
            r.md.seq_log = []
            r.dart.auto()
            r.dart.wait_phase("ANCHORED", 10)
            seqs = r.md.seq_log
            assert r.md.orient == "upright" and not asked
            assert any(q.startswith("righting") for q in seqs), seqs
            assert ("righting: leg out" in seqs) == (orient == "inverted"), seqs
            r.dart.release()
            r.dart.wait_phase("IDLE", 10, want_seq="released")
        r.md.orient, r.md.cannot_right = "side", True
        r.m.anchor()
        assert "NOT_UPRIGHT" in asked[0][0] and "Force anchor" in asked[0][1]
        assert any("force=1" in q for _, p_, q in r.md.requests if p_ == "/auto")
        assert r.dart.status()["phase"] == "ANCHORED"
        r.dart.release()
        r.dart.wait_phase("IDLE", 10, want_seq="released")
        r.md.orient = "side"
        r.m.ask = lambda text, choices, **kw: "Release and reshoot"
        assert r.m.anchor() == "Release and reshoot"
        r.md.orient, r.md.cannot_right = "upright", False
        r.md.batt = "critical"
        r.m.dart_anchored = False
        with pytest.raises(StepFailed, match="battery low"):
            r.m.anchor()
    finally:
        r.close()


def test_tow_stops_when_the_dart_says_its_anchor_moves(tmp_path):
    from grappstar.mission import StepFailed
    from mission_harness import Rig
    r = Rig(tmp_path)
    try:
        r.dart.auto()
        r.dart.wait_phase("ANCHORED", 10)
        stops = []
        r.m.stop_settle = lambda: stops.append(time.time())
        r.md.moving = True
        m = r.base.event_mark()
        with pytest.raises(StepFailed, match="MOVING"):
            r.m._wait_limit_watching_anchor(m, 5.0)
        assert stops and r.m.shot.get("anchor_moved")
        r.md.moving = False
        time.sleep(0.5)                       # the poller's status is at most 0.3 s old
        assert not r.m._anchor_moving()
    finally:
        r.close()
