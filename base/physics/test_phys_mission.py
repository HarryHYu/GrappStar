from common import Params
from mission import run


def test_mission_budget_sane():
    r = run(Params())
    assert 60 < r["total_s"] < 600
    assert 0 < r["total_wh"] < 5
    assert r["missions_per_pack"] > 10
    assert abs(sum(s["time_s"] for s in r["steps"]) - r["total_s"]) < 1e-9
