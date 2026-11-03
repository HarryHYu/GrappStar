import json

import pytest

from grappstar import jlog
from grappstar.mission import MissionConfig
from mission_harness import Rig, FakeCam

HAPPY = ["PREFLIGHT", "LAUNCH_MODE", "AIM", "COCK", "ARM", "FIRE", "FLIGHT", "LANDING_ZONE", "BASE_NEUTRAL", "WRAP",
         "ANCHOR", "TIDY", "FLY_IN", "CHECK", "TOW", "RELEASE", "RETRIEVE", "UNWRAP", "FINAL_APPROACH", "LAUNCH_READY", "DONE"]


@pytest.fixture
def rig(tmp_path):
    r = Rig(tmp_path)
    yield r
    r.close()


def test_happy_path(rig, tmp_path):
    m = rig.run()
    assert m.state == "DONE", (m.error, rig.answers[-3:])
    assert rig.steps() == HAPPY
    t = rig.base.tel
    assert t["mode"] == "LAUNCH" and t["drum_pos"] == "LAUNCH" and t["sw"].get("S") and t["wraps"] == 0
    assert rig.md.phase == "IDLE"
    assert m.shot["check"]["held"]
    # two FIRE confirmations, both danger
    fire = [a for a in rig.answers if "FIRE" in a[0]]
    assert len(fire) == 2
    # aim was the ballistic pitch
    assert abs(m.shot["pitch_deg"] - m.shot["solution"]["pitch_deg"]) < 0.01
    assert "dart" in m.shot["landing_sources"]
    # log is valid JSONL with steps, prompts, base traffic
    kinds = {r["kind"] for r in jlog.read(str(tmp_path / "mission.jsonl"))}
    assert {"step", "prompt", "answer"} <= kinds
    # the turn advice was given (bearing 3 deg right)
    assert any("Turn the rover 3 degrees right" in a[0] for a in rig.answers)


def test_no_go_reshoot_then_go(tmp_path):
    r = Rig(tmp_path, verdict=["NO_GO", "GO"])
    try:
        m = r.run(90)
        assert m.state == "DONE", m.error
        s = r.steps()
        assert s.count("FIRE") == 2 and s.count("FINAL_APPROACH") == 2 and m.reshoots == 1
        i = s.index("LANDING_ZONE")
        # 10.1: a NO GO comes before the wrap, so the unanchored dart comes home on the take-up only
        assert s[i + 1:i + 3] == ["BASE_NEUTRAL", "FINAL_APPROACH"]
        assert "LAUNCH_MODE" in s[i + 3:]
    finally:
        r.close()


def test_no_go_operator_anchors_anyway(tmp_path):
    r = Rig(tmp_path, verdict="NO_GO")
    r.policy["Landing zone"] = "Anchor anyway"
    try:
        m = r.run()
        assert m.state == "DONE" and r.steps() == HAPPY
    finally:
        r.close()


def test_abort_at_fire_confirmation(rig):
    rig.policy["Second confirmation"] = "Abort"
    m = rig.run()
    assert m.state == "ABORTED"
    assert not m.shot.get("fire_sent")                                   # never fired
    assert not any(e.get("evt") == "FIRED" for e in rig.base.events(0))
    assert "COCKED" in m.error                                           # the operator is told to decock


def test_stop_during_flight_wait(tmp_path):
    r = Rig(tmp_path, flight_s=30.0)
    try:
        r.m.start()
        import time
        t0 = time.time()
        while r.m.step != "FLIGHT" and time.time() - t0 < 30:
            time.sleep(0.02)
        r.m.abort()
        r.m.thread.join(5)
        assert r.m.state == "ABORTED"
    finally:
        r.close()


def test_preflight_refuses_without_target(tmp_path):
    r = Rig(tmp_path, target=None)
    r.policy["no target"] = "Abort"
    try:
        m = r.run()
        assert m.state == "ABORTED"
        assert any("no target" in a[0] for a in r.answers)
    finally:
        r.close()


def test_preflight_dart_unreachable_then_retry(tmp_path):
    r = Rig(tmp_path)
    r.md.down = True
    calls = []

    def fix(p):
        calls.append(1)
        r.md.down = False
        return "Retry"
    r.policy["dart unreachable"] = fix
    try:
        m = r.run()
        assert m.state == "DONE" and calls
    finally:
        r.close()


def test_unreachable_target(tmp_path):
    r = Rig(tmp_path, target=(40.0, 0.0))
    r.policy["not reachable"] = "Abort"
    try:
        m = r.run()
        assert m.state == "ABORTED"
        assert any("not reachable" in a[0] for a in r.answers)
    finally:
        r.close()


def test_drum_fault_on_mode_retry_clears(tmp_path):
    r = Rig(tmp_path)
    r.mb.fail_next["mode"] = "DRUM_TIMEOUT"
    try:
        m = r.run()
        assert m.state == "DONE", m.error
        assert any("DRUM_TIMEOUT" in a[0] and a[1] == "Retry" for a in r.answers)
    finally:
        r.close()


def test_stall_while_cocking(tmp_path):
    r = Rig(tmp_path)
    r.mb.fail_next["cock"] = "STALL"
    try:
        m = r.run(90)
        assert m.state == "DONE", m.error
        assert any("STALL" in a[0] for a in r.answers)
        assert r.steps().count("COCK") == 2 and r.steps().count("FIRE") == 1
    finally:
        r.close()


def test_landing_not_detected_operator_confirms(tmp_path):
    cfg = MissionConfig(land_timeout_s=1.0)
    r = Rig(tmp_path, cfg=cfg)
    r.md.no_landing = True
    r.world.flight = None
    def fire():
        r.md.launch()
        r.world.flight = lambda t: 7.0 * min(1.0, t / 1.1) + (0.5 * t if t > 1.1 else 0)  # line keeps creeping
    r.world.on_fire = fire
    try:
        def down(p):
            r.world.dart = "free"          # it is lying on the ground; only the detector missed it
            r.world.flight = lambda t: 7.0
            return "Dart is down"
        r.policy["Landing not detected"] = down
        r.policy["Landing zone"] = "Retrieve and reshoot"
        r.policy["Dart is home"] = "Abort"
        m = r.run()
        assert any("Landing not detected" in a[0] for a in r.answers)
        assert m.shot["landing_sources"] == ["operator"] or "LANDING_ZONE" in r.steps()
    finally:
        r.close()


def test_anchor_fails_release_and_reshoot(tmp_path):
    r = Rig(tmp_path)
    r.md.fail_anchor = True
    def second(p):
        r.md.fail_anchor = False
        return "Release and reshoot"
    r.policy["did not anchor"] = second
    try:
        m = r.run(90)
        assert m.state == "DONE", m.error
        assert m.reshoots == 1 and r.steps().count("ANCHOR") == 2
    finally:
        r.close()


def test_anchor_slips_on_check(tmp_path):
    r = Rig(tmp_path, anchor_holds=False)     # dart says anchored but the line just drags it home
    r.policy["Release and reshoot"] = "Release and reshoot"
    try:
        m = r.run(90)
        s = r.steps()
        assert "CHECK" in s and "TOW" not in s
        assert m.state == "DONE"
        assert r.base.tel["sw"].get("S")
    finally:
        r.close()


def test_snag_on_retrieve_then_retry(tmp_path):
    r = Rig(tmp_path)
    state = {"n": 0}
    orig = r.m.retrieve
    def retrieve():
        if state["n"] == 0:
            state["n"] = 1
            import time; time.sleep(0.1)     # let the rig's phase sync settle first
            r.world.dart = "anchored"; r.world.slack_m = 0.0     # line caught on a rock
            try:
                return orig()
            finally:
                r.world.dart = "free"
        return orig()
    r.m.retrieve = retrieve
    try:
        m = r.run(90)
        assert m.state == "DONE", m.error
        assert any("snag" in a[0] for a in r.answers)
    finally:
        r.close()


def test_release_fault_then_manual_retry(tmp_path):
    r = Rig(tmp_path)
    r.md.fail_release = True
    def fix(p):
        r.md.fail_release = False
        return "Retry"
    r.policy["release did not finish"] = fix
    try:
        m = r.run(90)
        assert m.state == "DONE", m.error
    finally:
        r.close()


def test_base_disconnect_mid_mission_abort(tmp_path):
    r = Rig(tmp_path)
    def pull(p):
        r.base.transport.broken = True
        return "Aim"
    r.policy["Aim now"] = pull
    r.policy["failed"] = "Abort"
    try:
        m = r.run()
        assert m.state == "ABORTED"
        assert any("AIM failed" in a[0] for a in r.answers)
    finally:
        r.close()


def test_base_fault_during_tow(tmp_path):
    r = Rig(tmp_path)
    r.mb.fail_next["reel"] = "OVERCURRENT"     # the check pull faults
    r.policy["failed"] = "Abort"
    try:
        m = r.run()
        assert m.state == "ABORTED"
        assert any("OVERCURRENT" in a[0] for a in r.answers)
        import time; time.sleep(0.3)
        assert r.base.tel["drum_pos"] in ("NEUTRAL", "BETWEEN") or r.base.tel["fault"] == "OVERCURRENT"
    finally:
        r.close()


def test_preflight_warns_without_video(tmp_path):
    r = Rig(tmp_path, cams={"dart": FakeCam(False), "rover": FakeCam(False)})
    try:
        m = r.run()
        assert m.state == "DONE"
        assert any("video not connected" in a[0] for a in r.answers)
    finally:
        r.close()


def test_auto_anchor_on_landing_mode(tmp_path):
    r = Rig(tmp_path, cfg=MissionConfig(auto_anchor_on_landing=True))
    try:
        m = r.run()
        assert m.state == "DONE", m.error
        assert any(path == "/arm" and "auto=1" in q for _, path, q in r.md.requests)
    finally:
        r.close()


def test_dart_reboot_at_impact_counts_as_landing(tmp_path):
    r = Rig(tmp_path)
    def land():
        r.md.reboot("brownout")          # power plug dropout at impact: no LANDED, no flight log
        r.world.dart = "free"
    r._land = land
    try:
        m = r.run(120)
        assert m.state == "DONE", m.error
        assert "dart" in m.shot.get("landing_sources", [])
    finally:
        r.close()


def test_tow_refused_without_load_cell_and_factory_constants(tmp_path):
    r = Rig(tmp_path)
    r.mb.load_cell = False                       # tension_n null; k_n_per_a still a factory value
    r.policy["TOW refused"] = "Release the dart"
    try:
        m = r.run(120)
        assert any("TOW refused" in a[0] for a in r.answers)
        assert m.state == "DONE"                 # released and brought home instead
        assert "RELEASE" in r.steps()
    finally:
        r.close()


def test_line_out_uses_video_range_only_with_imu(tmp_path):
    """R2-C2: a video-only range (9.2 m) must not become line_m; the solver's line out is used, capped by the table."""
    from grappstar.mission import Mission
    sent = {}

    class B:
        def set(self, k, v):
            sent[k] = v
    m = Mission(B(), None, {}, None, type("L", (), {"info": lambda *a, **k: None, "warn": lambda *a, **k: None,
                                                     "write": lambda *a, **k: None})(),
                analyse=lambda *a, **k: {})
    m.shot = {"solution": {"line_out_m": 4.6, "range_max_m": 4.9}}
    m.tell_line_out({"flight": {"range_m": 9.2, "source": "video"}})
    assert sent["line_m"] == 4.6
    m.tell_line_out({"flight": {"range_m": 4.2, "source": "video+imu"}})
    assert abs(sent["line_m"] - (4.2 * 1.05 + 0.3)) < 0.01
    m.tell_line_out({"flight": {"range_m": 12.0, "source": "imu"}})
    assert sent["line_m"] <= 4.9 * 1.1 + 0.5
