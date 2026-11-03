import json

import pytest

from grappstar.ballistics import Ballistics, make_placeholder


@pytest.fixture
def b():
    return Ballistics(make_placeholder(path=None), source="placeholder.json")


def test_load_default():
    bl = Ballistics.load()
    assert bl.rows and bl.pitch_min < bl.pitch_max


def test_solve_round_trip(b):
    for drag in (0.5, 0.75, 1.0):
        for p in (17.0, 22.5, 30.0, 33.0):
            r = b.at_pitch(p, drag)["range_m"]
            s = b.solve(r, drag, lob=False)
            assert s.reachable and abs(s.pitch_deg - p) < 0.6, (drag, p, s)
            assert s.t_flight_s > 0 and s.line_out_m >= r


def test_high_branch_default_low_optional(b):
    l = b.solve(8.0, 0.75)                   # D29: prefer the higher pitch
    s = b.solve(8.0, 0.75, lob=False)
    assert s.pitch_deg < 40 < l.pitch_deg
    assert l.t_flight_s > s.t_flight_s


def test_unreachable(b):
    s = b.solve(50.0)
    assert not s.reachable and "too far" in s.note
    s = b.solve(1.0)
    assert not s.reachable and "too close" in s.note
    assert not b.solve(-3).reachable


def test_drag_interpolation_and_clamp(b):
    r05 = b.at_pitch(30, 0.5)["range_m"]
    r10 = b.at_pitch(30, 1.0)["range_m"]
    r = b.at_pitch(30, 0.625)["range_m"]
    assert r10 < r < r05
    assert b.at_pitch(30, 5.0)["range_m"] == pytest.approx(r10)


def test_placeholder_flag_in_note(b):
    assert "PLACEHOLDER" in b.solve(7.0).note


def test_c3_file_format(tmp_path):
    d = {"version": 1, "pitch_min": 20, "pitch_max": 40, "drag_n": [0.5, 1.0],
         "table": [{"pitch_deg": 20, "v_muzzle": 10, "range_m": [6, 5], "t_flight_s": [0.8, 0.8], "apex_m": [1, 1]},
                   {"pitch_deg": 40, "v_muzzle": 10, "range_m": [9, 8], "t_flight_s": [1.3, 1.3], "apex_m": [2, 2]}]}
    p = tmp_path / "ballistics.json"
    p.write_text(json.dumps(d))
    bl = Ballistics.load(str(p))
    s = bl.solve(7.5, 0.5)
    assert s.reachable and s.pitch_deg == pytest.approx(30.0)
    assert "placeholder" not in s.source
    d["table"][0]["range_m"] = [6]
    p.write_text(json.dumps(d))
    with pytest.raises(ValueError):
        Ballistics.load(str(p))
