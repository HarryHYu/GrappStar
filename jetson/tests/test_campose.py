import math

import pytest
import textwrap

from grappstar import campose


def test_no_params_gives_defaults(tmp_path):
    m = campose.camera_model(None, params_path=str(tmp_path / "missing.py"))
    assert m.pose_source == "default guesses" and m.range_offset_m == 0.0


def test_params_framework_pose(tmp_path):
    p = tmp_path / "params.py"
    p.write_text(textwrap.dedent("""
        FRAMEWORK = dict(deck=dict(h=10), rover_cam=dict(pos=(120.0, 0.0, 380.0), tilt_deg=14.0))
        LAUNCH = dict(pivot=(40.0, 0.0, 200.0))
    """))
    m = campose.camera_model(None, params_path=str(p))
    assert abs(m.height_m - 0.38) < 1e-9 and m.pitch_deg == 14.0
    assert abs(m.range_offset_m - 0.08) < 1e-9
    assert "params.FRAMEWORK" in m.pose_source


def test_calibration_file_wins(tmp_path):
    from grappstar.sim.world import default_rover_model
    c = default_rover_model(); c.height_m = 0.5; c.pitch_deg = 9.0
    f = tmp_path / "cam.json"; c.save(str(f))
    p = tmp_path / "params.py"
    p.write_text("FRAMEWORK = dict(kiyo=dict(z=380.0, tilt_deg=14.0, x=100.0))\nLAUNCH = dict(pivot_x=60.0)\n")
    m = campose.camera_model(str(f), params_path=str(p))
    assert m.height_m == 0.5 and m.pitch_deg == 9.0
    assert abs(m.range_offset_m - 0.04) < 1e-9


def test_stub_params_none():
    assert campose.find_camera_pose(type("P", (), {"FRAMEWORK": None})) is None


def test_base_spec_v1_camera_entry(tmp_path):
    p = tmp_path / "params.py"
    p.write_text('FRAMEWORK = dict(deck_z=-165.0, camera=dict(model="Razer Kiyo X", pos=(150.0, -50.0, 100.0), '
                 'pitch_deg=-10.0, look="+X"))\n')
    m = campose.camera_model(None, params_path=str(p), deck_height_m=0.2)
    assert abs(m.height_m - (0.2 + 0.265)) < 1e-9
    assert m.pitch_deg == 10.0                       # down positive in CameraModel
    assert abs(m.range_offset_m - 0.15) < 1e-9


V1_PARAMS = ('FRAMEWORK = dict(deck_z=-165.0, camera=dict(model="Razer Kiyo X", pos=(150.0, -50.0, 100.0), '
             'pitch_deg=-10.0, look="+X"))\nLAUNCH = dict(rail_y=140.0)\nLAYOUT = dict(rail_plane_y=140.0)\n')


def test_v1_camera_lateral_offset_kept(tmp_path):
    p = tmp_path / "params.py"
    p.write_text(V1_PARAMS)
    pose = campose.find_camera_pose(campose.load_params(str(p)))
    assert pose["y_m"] == pytest.approx(-0.05) and pose["rail_y_m"] == pytest.approx(0.14)
    m = campose.camera_model(None, params_path=str(p))
    assert m.lateral_offset_m == pytest.approx(-0.19)          # camera 0.19 m right of the launch line
    assert m.range_offset_m == pytest.approx(0.15)


def test_rail_plane_from_layout_when_launch_has_none(tmp_path):
    p = tmp_path / "params.py"
    p.write_text(V1_PARAMS.replace("LAUNCH = dict(rail_y=140.0)\n", ""))
    assert campose.camera_model(None, params_path=str(p)).lateral_offset_m == pytest.approx(-0.19)


def test_pivot_target_straight_ahead_of_camera_is_right_of_launch_line(tmp_path):
    p = tmp_path / "params.py"
    p.write_text(V1_PARAMS)
    m = campose.camera_model(None, params_path=str(p))
    t = campose.pivot_target(m, (0.0, 4.0))
    assert t["x_right_m"] == pytest.approx(0.19) and t["y_fwd_m"] == pytest.approx(4.15)
    assert t["range_m"] == pytest.approx(math.hypot(0.19, 4.15))
    assert t["bearing_deg"] == pytest.approx(math.degrees(math.atan2(0.19, 4.15)))   # about 2.6 deg right
    # a target ON the launch line reads bearing 0 from the pivot, and -2.7 deg (left) from the camera
    t = campose.pivot_target(m, (-0.19, 3.85))
    assert t["bearing_deg"] == pytest.approx(0.0, abs=1e-9) and t["range_m"] == pytest.approx(4.0)
    assert campose.pivot_target(m, None) is None


def test_target_info_pivot_moves_range_bearing_and_advice(tmp_path):
    p = tmp_path / "params.py"
    p.write_text(V1_PARAMS)
    m = campose.camera_model(None, size=(1280, 720), params_path=str(p))
    u, v = m.ground_to_pixel(4.0, 0.0)          # straight ahead of the camera, 4 m
    info = campose.target_info_pivot(m, u, v)
    assert info["range_cam_m"] == pytest.approx(4.0, abs=0.02) and info["bearing_cam_deg"] == pytest.approx(0.0, abs=0.1)
    assert info["range_m"] == pytest.approx(4.15, abs=0.02)
    assert info["bearing_deg"] == pytest.approx(2.6, abs=0.15)
    assert "right" in info["advice"] and info["msg"].startswith(info["advice"])
    if info.get("range_lo_m") is not None:
        assert info["range_lo_m"] < info["range_m"] < info["range_hi_m"]
    # above the horizon: unchanged failure
    assert campose.target_info_pivot(m, 640, 0)["range_m"] is None


def test_ballistics_with_pivot_range_prefers_high_pitch(tmp_path):
    """D29 end to end: camera range + offsets -> pivot range -> the higher of two solutions."""
    from grappstar.ballistics import Ballistics
    p = tmp_path / "params.py"
    p.write_text(V1_PARAMS)
    m = campose.camera_model(None, params_path=str(p))
    rows = [{"pitch_deg": float(a), "v_muzzle": 7.4, "range_m": [r, r, r], "t_flight_s": [0.4 + a / 60] * 3,
             "apex_m": [a / 30] * 3} for a, r in ((6, 2.7), (20, 4.0), (38, 4.95), (47, 4.7))]
    b = Ballistics({"version": 1, "pitch_min": 6, "pitch_max": 47, "drag_n": [0.25, 0.5, 0.75], "table": rows})
    t = campose.pivot_target(m, (-0.19, 4.65))          # on the line, 4.80 m from the pivot
    assert t["range_m"] == pytest.approx(4.80)
    s = b.solve(t["range_m"])
    assert s.reachable and s.pitch_deg > 38          # high branch
    assert b.solve(4.65).pitch_deg != s.pitch_deg     # the camera range alone would aim elsewhere
