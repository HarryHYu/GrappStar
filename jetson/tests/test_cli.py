import json

from grappstar import cli


def test_solve(capsys):
    assert cli.main(["solve", "3.5", "--drag", "0.75"]) in (0, 1)
    out = json.loads(capsys.readouterr().out)
    assert "pitch_deg" in out and "range_min_m" in out


def test_selftest():
    assert cli.main(["selftest"]) == 0


def test_base_via_pty(capsys):
    from grappstar.comms.mock_base import serve_pty
    mb = serve_pty(time_scale=10)
    try:
        assert cli.main(["base", "ping", "--port", mb.pty_path]) == 0
        assert '"ok": true' in capsys.readouterr().out
        assert cli.main(["base", "cmd", '{"cmd": "mode", "to": "LAUNCH"}', "--port", mb.pty_path]) == 0
    finally:
        mb.stop()


def test_dart_status(capsys):
    from grappstar.comms.mock_dart import MockDart
    md = MockDart().start()
    try:
        assert cli.main(["dart", "status", "--host", md.host]) == 0
        assert '"phase"' in capsys.readouterr().out
        assert cli.main(["dart", "status", "--host", "127.0.0.1:1", "--timeout", "0.2"]) == 2
    finally:
        md.stop()


def test_calibrate_default(tmp_path):
    out = tmp_path / "cam.json"
    assert cli.main(["calibrate", "default", "--height", "0.5", "--pitch", "12", "--out", str(out)]) == 0
    assert json.loads(out.read_text())["height_m"] == 0.5
