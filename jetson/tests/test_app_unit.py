"""App pieces that need no simulator."""
from grappstar.app import App


def test_launcher_state_rules():
    ls = App.launcher_state
    assert ls({"cock": 0, "sw": {"H": 1}}, "REST") == "REST"
    assert ls({"cock": 1, "sw": {}}, "COCKED") == "LOADED"
    assert ls({"cock": 0.4, "sw": {}}, "COCKING") == "LOADED"
    assert ls({"cock": None, "sw": {}}, "UNKNOWN") == "LOADED"          # r3_M #1: drawn carriage, phase unknown
    assert ls({"cock": None, "sw": {"H": 1}}, "UNKNOWN") == "NOT_KNOWN_SAFE"
    assert ls({}, None) == "UNKNOWN"


def test_act_set_caps_tensions_at_55_n():
    """D46 (spec 1.3.2): tow cap at the dart 55 N."""
    class Cfg:
        class mission:
            tow_limit_n = 40.0
    a = App.__new__(App)
    a.cfg = Cfg()
    a.target = None
    a._busy = lambda: False
    assert a.act_set("tow_limit_n", 55)["ok"] and a.cfg.mission.tow_limit_n == 55.0
    r = a.act_set("tow_limit_n", 56)
    assert not r["ok"] and "1 to 55" in r["msg"]
    assert not a.act_set("check_n", 60)["ok"]
