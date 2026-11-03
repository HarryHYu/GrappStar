"""Review round 3: tests that show real bugs in grappstar/mission.py against the base firmware (basesim) and the
mock base. Each test FAILS on the code as reviewed and passes once the finding is fixed."""
import functools
import json
import threading
import time

import pytest

from grappstar.mission import MissionConfig
from mission_harness import Rig

import test_mission_realsim as RS
from basesim_transport import find_basesim

needs_basesim = pytest.mark.skipif(find_basesim() is None, reason="basesim not built")


def _tx_log(base):
    out = []

    def on_line(d, o):
        if d == "tx" or "evt" in o or ("id" in o and o.get("ok") is False):
            out.append((d, o))
    base.on_line = on_line
    return out


# ------------------------------------------------------------------ F1: FINAL_APPROACH re-wraps instead of take-up
@pytest.mark.slow       # a whole mission on the real firmware: check_all.sh runs it
@needs_basesim
def test_final_approach_uses_takeup_not_a_second_wrap(tmp_path):
    """D22: reel on the capstan to about approach_m, unwrap, then the take-up until seated. base-1.0 has no
    "retrieve" command; its final approach is "reel until seated" in NEUTRAL (take-up only). The mission instead
    sends "retrieve" (UNKNOWN_CMD), wraps the capstan AGAIN, reels on the capstan and unwraps a second time."""
    r = RS.RealRig(tmp_path, line_out=8.0)
    r.m.cfg.approach_m = 2.5          # leaves line out after the unwrap take-up, so FINAL_APPROACH really runs
    log = _tx_log(r.base)
    try:
        r.m.start()
        r.m.thread.join(240)
        assert r.m.state == "DONE", r.m.error
        cmds = [o.get("cmd") for d, o in log if d == "tx" and o.get("cmd") not in ("ping", "status")]
        i_unwrap = cmds.index("unwrap")
        after = cmds[i_unwrap + 1:]
        # BASE_SPEC 10.1: the final approach is "retrieve" on the take-up in NEUTRAL (base-1.2 faults DART_STUCK
        # after a fixed 20 s on a long line, so one Retry may appear; base-1.2.1 honours max_s)
        assert "retrieve" in after, after
        assert "wrap" not in after, f"re-wrapped after UNWRAP (D22 final approach is on the take-up): {after}"
    finally:
        r.close()


# ------------------------------------------------------------------ F2: tow with a load cell: reel refused BUSY
@pytest.mark.slow       # a whole mission on the real firmware: check_all.sh runs it
@needs_basesim
def test_tow_with_load_cell_does_not_fail_busy(tmp_path, monkeypatch):
    """With a load cell the firmware winds again by itself when the held line slackens (tickHold, state REEL). The
    next tow round then sends reel, which the firmware refuses with BUSY: TOW fails and the operator gets a fault
    prompt on every slack round."""
    monkeypatch.setattr(RS, "start_basesim", functools.partial(RS.start_basesim, load_cell=True))
    r = RS.RealRig(tmp_path)

    class DriveRover(RS.RoverDrive):
        calls = 0

        def tow_assist(self, m, tn):
            self.calls += 1
            if self.calls >= 3:
                return "done"
            r.base.transport.sim(f"anchor {r.line_out - r.slack - 0.5 * self.calls}")   # rover drove 0.5 m in
            time.sleep(3.0 / RS.SPEED)
            return "slack"
    r.m.rover = DriveRover()
    try:
        r.m.start()
        r.m.thread.join(240)
        failed = [a for a in r.answers if "failed" in a[1]]
        assert not failed, failed
    finally:
        r.close()


# ------------------------------------------------------------------ F3: STOP while cocking: no Decock warning
@needs_basesim
def test_stop_while_cocking_warns_to_decock(tmp_path):
    """STOP during COCK leaves the spring part drawn (firmware phase UNKNOWN). The end-of-mission warning only looks at
    shot["cocked"], which is set after COCKED, so the operator is not told to Decock."""
    r = RS.RealRig(tmp_path)
    try:
        r.m.start()
        t0 = time.time()
        while r.m.step != "COCK":
            assert time.time() - t0 < 60
            time.sleep(0.005)
        r.base.wait_tel(lambda t: t.get("state") == "COCK" and (t.get("cock") or 0) > 0.3, 10)
        r.m.abort("operator STOP")
        r.base.safe_stop()                         # what App.act_stop does next
        r.m.thread.join(20)
        assert r.base.status()["phase"] == "UNKNOWN"
        assert r.m.state == "ABORTED"
        assert "Decock" in r.m.error, r.m.error
    finally:
        r.close()


# ------------------------------------------------------------------ F4: _do's stop cancels the firmware safing move
@needs_basesim
def test_fault_safing_move_is_not_cancelled(tmp_path):
    """On a fault the firmware drives the drum to NEUTRAL by itself (state SAFING). Mission._do sends "stop" on every
    BaseError, and cmdStop in SAFING halts the drum: it stays in LAUNCH (or between detents) and the prompt even says
    "The drum went to NEUTRAL"."""
    r = RS.RealRig(tmp_path)
    r._stop.set()                                  # replace the auto operator: hold the fault prompt open
    prompts = []

    def operator():
        while r.m.state in ("IDLE", "RUNNING"):
            p = r.m.prompt
            if p is not None and p.answer is None and p.id not in [q.id for q in prompts]:
                prompts.append(p)
                if "failed" not in p.text:
                    r.m.answer(p.id, p.choices[0])
            time.sleep(0.01)
    threading.Thread(target=operator, daemon=True).start()
    try:
        r.m.start()
        r.base.wait_tel(lambda t: t.get("state") == "AIM", 60)
        r.base.transport.sim("fault extraLoadNm 30")          # wiper overloaded while aiming: a fault
        t0 = time.time()
        while not any("failed" in p.text for p in prompts):
            assert time.time() - t0 < 30
            time.sleep(0.02)
        r.base.transport.sim("fault extraLoadNm 0")
        time.sleep(2.0)                                       # 16 s of base time: the safing move takes about 4 s
        t = r.base.tel
        assert t["drum_pos"] == "NEUTRAL", (t["state"], t["drum_pos"], [p.text for p in prompts][-1])
    finally:
        r.m.abort("test end")
        r.close()


# ------------------------------------------------------------------ F5: reshoot limit abandons the dart
def test_reshoot_limit_still_retrieves_the_dart(tmp_path):
    """reshoot() counts and aborts BEFORE bringing the dart home: past max_reshoots a NO GO leaves the dart on the
    ground with the line out (D22: a no-go retrieves the unanchored dart)."""
    r = Rig(tmp_path, verdict="NO_GO", cfg=MissionConfig(max_reshoots=0))
    r.policy["Landing zone"] = "Retrieve and reshoot"
    try:
        m = r.run()
        # 10.1: a NO GO before the wrap comes home on the take-up (FINAL_APPROACH); the capstan RETRIEVE is only
        # used when wrapped
        assert "FINAL_APPROACH" in r.steps(), (m.state, m.error, r.steps())
        assert r.base.tel["sw"].get("S"), "dart must be seated before the reshoot limit ends the mission"
    finally:
        r.close()
