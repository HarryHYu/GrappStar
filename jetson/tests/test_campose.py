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
