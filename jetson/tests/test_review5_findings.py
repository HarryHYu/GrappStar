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


def test_factory_k_matches_every_firmware_default():
    """systest F28: D43 moved the firmware default to 14.8 (params 1.3.2: 13.1); the literal match on 8 and 17 no
    longer refused an uncalibrated tow."""
    from grappstar.mission import k_is_factory
    for k in (8.0, 17.0, 14.8, 13.1, 14.80001, "13.1"):
        assert k_is_factory(k), k
    for k in (None, 11.2, 15.6, "x"):
        assert not k_is_factory(k), k
    assert not k_is_factory(14.8, {"tension_cal": True})      # a base flag, when one exists, wins
    assert k_is_factory(11.2, {"tension_cal": False})
