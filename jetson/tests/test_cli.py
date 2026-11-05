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


def test_settings_json_defaults(tmp_path, monkeypatch):
    import json as _j
    p = tmp_path / "settings.json"
    p.write_text(_j.dumps({"imu_axes": "y,-x,z", "tow_limit": 25}))
    monkeypatch.setattr(cli, "SETTINGS", str(p))
    seen = {}
    monkeypatch.setattr(cli, "cmd_serve", lambda a: seen.update(vars(a)) or 0)
    assert cli.main(["serve", "--sim", "--tow-limit", "30"]) == 0
    assert seen["imu_axes"] == "y,-x,z"      # from the file
    assert seen["tow_limit"] == 30.0          # the command line wins


def test_base_fire_one_connection(monkeypatch, capsys):
    from grappstar.comms.mock_base import serve_pty
    mb = serve_pty(time_scale=10)
    try:
        port = mb.pty_path
        import time; time.sleep(0.5)                 # past the base's BOOTING window
        assert cli.main(["base", "mode", "LAUNCH", "--port", port]) == 0
        import time
        t0 = time.time()
        while (mb.state != "IDLE" or mb.drum_running) and time.time() - t0 < 20:
            time.sleep(0.05)
        assert cli.main(["base", "aim", "30", "--port", port]) == 0
        t0 = time.time()
        while (mb.state != "IDLE" or mb.drum_running) and time.time() - t0 < 20:   # aim runs about 17 s of base time
            time.sleep(0.05)
        monkeypatch.setattr("builtins.input", lambda prompt="": "nope")
        assert cli.main(["base", "fire", "--port", port]) == 1      # not confirmed: nothing sent
        assert not any(r["cmd"] in ("arm", "fire") for r in mb.received)
        monkeypatch.setattr("builtins.input", lambda prompt="": "FIRE")
        rc = cli.main(["base", "fire", "--port", port])
        out = capsys.readouterr().out
        assert any(r["cmd"] == "fire" for r in mb.received), out
        assert rc == 0 and "FIRED" in out and "RESET" in out, out
    finally:
        mb.stop()


def test_one_drag_default_everywhere():
    """r2_C2 #1: the LIGHT 0.25 N clutch column (D49, D50) is the default in the CLI, the mission and the settings example."""
    import os
    from grappstar import mission
    ex = json.load(open(os.path.join(os.path.dirname(os.path.dirname(__file__)), "settings.example.json")))
    assert cli.DEFAULT_DRAG_N == mission.DEFAULT_DRAG_N == mission.MissionConfig().drag_n == ex["drag"] == 0.25


def test_console_line_parsing():
    assert cli.parse_console_line("  ") is None
    assert cli.parse_console_line("stop") == {"cmd": "stop"}
    assert cli.parse_console_line("reel duty=0.3 max_n=20 until=stop") == {"cmd": "reel", "duty": 0.3, "max_n": 20,
                                                                          "until": "stop"}
    assert cli.parse_console_line('{"cmd": "set", "key": "limit_n", "val": 30}')["val"] == 30
    import pytest
    with pytest.raises(ValueError):
        cli.parse_console_line("reel 0.3")
    with pytest.raises(ValueError):
        cli.parse_console_line('{"key": "x"}')


def test_base_console_one_connection_shows_tel_and_sends_commands():
    """docs: CALIBRATION (f) needs telemetry live while sending stop: one connection does both."""
    import io
    from grappstar.comms.base import BaseClient, pipe_pair
    from grappstar.comms.mock_base import MockBase
    import time
    a, b = pipe_pair()
    mb = MockBase(a, time_scale=10).start()
    c = BaseClient(b, reconnect_s=0.05).start()
    try:
        assert c.wait_connected(3.0)
        time.sleep(0.5)                              # past BOOTING
        out = io.StringIO()
        inp = io.StringIO('status\nbogus 1\n{"cmd": "get", "key": "fw"}\nmode to=LAUNCH\nquit\nstop\n')
        assert cli.base_console(c, inp, out, tel_every=0.0) == 0
        time.sleep(1.5)
        text = out.getvalue()
        assert "TEL state=" in text and "REPLY" in text and "ERROR" in text
        assert '"val": "base-1.3.1"' in text
        assert "EVENT" in text or '"ok": true' in text
        assert text.count("REPLY") == 3                # "stop" after quit is never sent
    finally:
        c.close()
        mb.stop()
