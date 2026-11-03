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
            s = b.solve(r, drag)
            s = b.solve(r, drag, low=True)          # these pitches are on the rising branch
            assert s.reachable and abs(s.pitch_deg - p) < 0.6, (drag, p, s)
            assert s.t_flight_s > 0
            sag = 8.0 * (s.apex_m * 0.25) ** 2 / (3.0 * r)
            assert s.line_out_m == pytest.approx((r + sag) * 1.10, abs=0.02)
            assert b.solve(r, drag, low=True, line_margin=1.0).line_out_m == pytest.approx(r + sag, abs=0.02)


def test_high_branch_default_low_optional(b):
    """D29 / params AIM.solver: prefer the higher pitch when two solutions exist."""
    h = b.solve(8.0, 0.75)
    lo = b.solve(8.0, 0.75, low=True)
    assert lo.pitch_deg < 40 < h.pitch_deg
    assert h.t_flight_s > lo.t_flight_s
    assert "high branch" in h.note and "low branch" in lo.note
    # the old lob flag still asks for the high branch and never forces the low one
    assert b.solve(8.0, 0.75, lob=True).pitch_deg == h.pitch_deg
    assert b.solve(8.0, 0.75, lob=False).pitch_deg == h.pitch_deg


def test_default_drag_is_light_clutch(b):
    from grappstar.ballistics import LIGHT_DRAG_N
    assert LIGHT_DRAG_N == 0.5
    assert b.solve(7.0).drag_n == 0.5
    assert b.at_pitch(30)["range_m"] == pytest.approx(b.at_pitch(30, 0.5)["range_m"])


def test_real_table_whole_window_and_high_pitch():
    """With the real C3 table (base/out/ballistics.json): window 6..47 and the higher pitch for long shots."""
    import os
    from grappstar.ballistics import REPO_TABLE
    if not os.path.exists(REPO_TABLE):
        pytest.skip("no base/out/ballistics.json")
    bl = Ballistics.load(REPO_TABLE)
    assert bl.pitch_min <= 6.0 + 1e-9 and bl.pitch_max >= 47.0 - 1e-9
    col = bl._col("range_m", 0.5)
    i_pk = max(range(len(col)), key=col.__getitem__)
    if bl.rows[i_pk]["pitch_deg"] >= bl.pitch_max:
        pytest.skip("table range does not peak inside the window")
    r = 0.5 * (col[i_pk] + max(col[-1], max(col[:i_pk])))
    r = min(col[i_pk] - 0.01, max(r, col[-1] + 0.01))
    s, lo = bl.solve(r), bl.solve(r, low=True)
    assert s.reachable and lo.reachable and s.pitch_deg > bl.rows[i_pk]["pitch_deg"] > lo.pitch_deg
    assert bl.at_pitch(s.pitch_deg, 0.5)["range_m"] == pytest.approx(r, abs=0.02)
    short = bl.solve(col[0] + 0.05)        # just past the 6 deg range: only the low end of the window reaches it
    assert short.reachable and short.pitch_deg < 10


def test_pitch_window_clamp():
    """Rows outside pitch_min..pitch_max are never chosen (the rail cannot go there)."""
    rows = [{"pitch_deg": float(p), "v_muzzle": 8.0, "range_m": [r, r], "t_flight_s": [p / 40] * 2, "apex_m": [1, 1]}
            for p, r in ((10, 4.0), (20, 6.0), (35, 8.0), (45, 9.0))]
    bl = Ballistics({"version": 1, "pitch_min": 20, "pitch_max": 35, "drag_n": [0.5, 1.0], "table": rows})
    s = bl.solve(8.6)
    assert not s.reachable and "too far" in s.note and s.range_max_m == 8.0
    s = bl.solve(5.0)
    assert not s.reachable and "too close" in s.note and s.range_min_m == 6.0
    s = bl.solve(7.0)
    assert s.reachable and 20 <= s.pitch_deg <= 35


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
