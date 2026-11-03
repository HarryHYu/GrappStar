"""Round 5: BASE_NEUTRAL waited out the reset timeout after RAIL_LOW; final approach max_s from the firmware formula."""
import pytest

from grappstar.comms import mock_base
from grappstar.mission import approach_max_s
from mission_harness import Rig


def test_approach_max_s_mirrors_firmware():
    # base-1.2.1 approachLimit(): 10 + 1.5 x line / (0.29 x duty - 0.03), 20..600 s
    assert approach_max_s(3.0, 0.3) == pytest.approx(10 + 4.5 / 0.057, abs=0.06)
    assert approach_max_s(0.2, 0.3) == 20.0                    # at least 20 s
    assert approach_max_s(400.0, 0.3) == 600.0                 # at most 600 s
    assert approach_max_s(3.0, 0.9) == approach_max_s(3.0, 102 / 255.0)   # duty clamped to the take-up maximum
    assert approach_max_s(3.0, 0.1) == approach_max_s(3.0, 70 / 255.0)    # and to the reel minimum
    for line in (0.0, 1.5, 3.8, 12.0, 30.0):
        assert approach_max_s(line, 0.3) == pytest.approx(mock_base.approach_s(line, 0.3), abs=0.06)
    assert mock_base.approach_s(None, 0.3) == 30.0             # line_m unknown: approachMaxMs


def test_base_neutral_does_not_wait_for_a_reset_that_already_came(tmp_path):
    """RAIL_LOW consumed the shot mark; BASE_NEUTRAL then waited for a NEW RESET event and sat out the whole
    reset timeout (30 s on hardware) before going to NEUTRAL."""
    r = Rig(tmp_path)
    try:
        m = r.run(90)
        assert m.state == "DONE", m.error
        t = {h["state"]: h["t"] for h in m.history}
        assert t["WRAP"] - t["BASE_NEUTRAL"] < 8.0, t["WRAP"] - t["BASE_NEUTRAL"]
    finally:
        r.close()


def test_uncalibrated_k_without_the_flag_uses_params_py():
    """systest F28: without the base's tension_cal flag (older firmware), k_n_per_a is compared with the default read
    from base/spec/params.py (REEL tow_tension_model n_per_a), never a literal, plus the earlier firmware defaults."""
    from grappstar.mission import k_is_factory, params_default_k
    pk = params_default_k()
    assert pk is not None and 5 < pk < 30                      # read from the repo's params.py
    assert k_is_factory(pk, params_k=pk) and k_is_factory(round(pk, 1) + 0.01, params_k=pk)
    assert k_is_factory(21.5, params_k=21.5)                   # whatever params.py says becomes the default
    for k in (8.0, 17.0, 14.8, "17"):                          # earlier firmware defaults (a unit flashed long ago)
        assert k_is_factory(k, params_k=pk), k
    for k in (None, 11.2, 15.6, "x"):
        assert not k_is_factory(k, params_k=pk), k
    assert not k_is_factory(pk, tension_cal=True, params_k=pk)   # the base flag wins when present
    assert k_is_factory(11.2, tension_cal=False, params_k=pk)
    assert params_default_k("/nonexistent/params.py") is None


def _tow_rig(tmp_path, set_k=None, flag_supported=True):
    r = Rig(tmp_path)
    r.mb.load_cell = False                       # tension_n null: the tow limit is the current estimate
    if set_k is not None:
        with r.mb.lock:
            r.mb._cmd_set("k_n_per_a", set_k)    # CALIBRATION (f): tension_cal true
    if not flag_supported:
        real_get = r.base.get
        def get(key, *a, **kw):
            if key == "tension_cal":
                from grappstar.comms.base import BaseError
                raise BaseError("get", "BAD_KEY", {})
            return real_get(key, *a, **kw)
        r.base.get = get
    r.policy["TOW refused"] = "Release the dart"
    return r


def test_tow_follows_the_base_tension_cal_flag(tmp_path):
    # calibrated on the bench to a value that happens to equal the default: the flag says calibrated, so tow
    r = _tow_rig(tmp_path, set_k=13.1)
    try:
        m = r.run(120)
        assert m.state == "DONE", m.error
        assert not any("TOW refused" in a[0] for a in r.answers) and "TOW" in r.steps()
    finally:
        r.close()


def test_tow_refused_on_older_firmware_with_the_params_default_k(tmp_path):
    from grappstar.mission import params_default_k
    r = _tow_rig(tmp_path, set_k=params_default_k(), flag_supported=False)
    try:
        m = r.run(120)
        assert any("TOW refused" in a[0] for a in r.answers)
        assert m.state == "DONE" and "RELEASE" in r.steps()
    finally:
        r.close()


def test_tow_allowed_on_older_firmware_with_a_calibrated_k(tmp_path):
    r = _tow_rig(tmp_path, set_k=11.3, flag_supported=False)
    try:
        m = r.run(120)
        assert m.state == "DONE", m.error
        assert not any("TOW refused" in a[0] for a in r.answers)
    finally:
        r.close()


def test_f22_tow_rounds_stop_before_a_new_reel_without_load_cell(tmp_path):
    """systest F22: without a load cell the HOLD keeps its held estimate at the limit; a tow re-sent from HOLD
    answers LIMIT "already at the limit" and never takes up the rover's slack. Each later round must send stop
    (and see ABORTED) before its new reel, and must not skip the pull because of the held estimate."""
    from mission_harness import ScriptRover
    r = _tow_rig(tmp_path, set_k=11.3)                 # calibrated: the tow is allowed
    r.m.rover = ScriptRover(r, rounds=3)
    sent = []
    real = r.base.cmd
    def cmd(name, *a, **kw):
        if name in ("stop", "reel"):
            sent.append((name, kw.get("until")))
        return real(name, *a, **kw)
    r.base.cmd = cmd
    try:
        m = r.run(120)
        assert m.state == "DONE", m.error
        hist = r.steps()
        assert "TOW" in hist
        tows = [i for i, x in enumerate(sent) if x == ("reel", "tension")]
        assert len(tows) >= 3, sent                     # every round pulls (none skipped on the held estimate)
        for i in tows[1:]:
            assert sent[i - 1][0] == "stop", sent       # stop right before each later tow reel
    finally:
        r.close()
