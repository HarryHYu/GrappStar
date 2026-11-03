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
