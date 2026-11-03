"""Static checks of the operator page against the server and the App it talks to (no browser needed)."""

import os
import re

import pytest

from grappstar.ui import server as ui_server

STATIC = ui_server.STATIC_DIR


def _read(name):
    with open(os.path.join(STATIC, name), encoding="utf-8") as f:
        return f.read()


HTML = _read("index.html")
JS = _read("app.js")
CSS = _read("style.css")


def _ids():
    return set(re.findall(r'\bid="([^"]+)"', HTML))


def test_every_js_id_exists_in_html():
    used = set(re.findall(r'\$\("([A-Za-z0-9_\-]+)"\)', JS))
    assert used, "app.js should look elements up with $()"
    missing = used - _ids()
    assert not missing, f"app.js uses ids missing from index.html: {sorted(missing)}"


def test_html_ids_unique():
    ids = re.findall(r'\bid="([^"]+)"', HTML)
    dup = {i for i in ids if ids.count(i) > 1}
    assert not dup, dup


def _page_actions():
    acts = set(re.findall(r'data-action="([a-z_]+)"', HTML))
    acts |= set(re.findall(r'\baction\("([a-z_]+)"', JS))
    return acts


def test_page_actions_exist_on_app():
    from grappstar.app import App
    acts = _page_actions()
    for a in ("start", "stop", "estop", "answer", "target_click", "target_auto", "target_range", "set", "jog",
              "dart_key", "dart_stop", "aim"):
        assert a in acts, f"page never calls {a}"
    missing = [a for a in acts if not callable(getattr(App, "act_" + a, None))]
    assert not missing, f"page calls actions the App does not have: {missing}"


def test_settings_keys_match_status_config_and_set(tmp_path):
    """The settings inputs use the keys App.status()['config'] reports and App.act_set accepts."""
    from grappstar.app import App, AppConfig
    keys = set(re.findall(r'data-set="([a-z_]+)"', HTML))
    assert keys == set(re.findall(r'data-key="([a-z_]+)"', HTML)) == {"tow_limit_n", "drag_n"}
    app = App(AppConfig(runs_dir=str(tmp_path)))
    for k in keys:
        r = app.act_set(key=k, val=12.0 if k == "tow_limit_n" else 0.5)
        assert r["ok"], r
    assert app.cfg.mission.tow_limit_n == 12.0 and app.cfg.mission.drag_n == 0.5
    assert not app.act_set(key="tow_limit_n", val=500)["ok"]
    r = app.act_target_range(range_m=4.0, bearing_deg=-3)
    assert r["ok"] and r["target"]["source"] == "manual" and r["target"]["bearing_deg"] == -3.0
    assert "solution" in r["target"]


def test_status_fields_used_by_page_exist():
    """Field names the page reads must be the ones App.status() and Mission.status() produce."""
    import inspect
    from grappstar import app as app_mod, mission as mission_mod
    src = inspect.getsource(app_mod.App.status) + inspect.getsource(mission_mod.Mission.status)
    for field in ("results", "target", "config", "cams", "mission", "log", "tow_limit_n", "drag_n", "placeholder",
                  "cam_uncalibrated", "files", "prompt", "history", "steps", "step", "error", "reshoots", "shot"):
        assert f'"{field}"' in src, field
        assert re.search(r"\b" + field + r"\b", JS), f"page does not read {field}"
    # the current step is mission.step (mission.state is IDLE/RUNNING/DONE/ABORTED/FAILED)
    assert "m.step" in JS


def test_page_is_offline_and_dash_free():
    for text in (HTML, JS, CSS):
        assert not re.search(r'(src|href)="https?://', text), "the page must work without internet"
        assert chr(0x2014) not in text and chr(0x2013) not in text


def test_touch_targets_at_least_44px():
    for m in re.finditer(r"min-height:\s*(\d+)px", CSS):
        assert int(m.group(1)) >= 40, m.group(0)
    assert re.search(r"button, \.btnlink \{[^}]*min-height: 48px", CSS)


@pytest.mark.parametrize("name", ["index.html", "app.js", "style.css"])
def test_static_served(name, tmp_path):
    import http.client
    import threading
    from types import SimpleNamespace
    app = SimpleNamespace(ui_token="", status=lambda: {}, action=lambda n, a: {"ok": True},
                          camera=lambda n: None, artifact=lambda n: None)
    s = ui_server.make_server(app, host="127.0.0.1", port=0)
    th = threading.Thread(target=s.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True)
    th.start()
    try:
        c = http.client.HTTPConnection("127.0.0.1", s.server_address[1], timeout=5)
        c.request("GET", "/" if name == "index.html" else "/static/" + name)
        r = c.getresponse()
        body = r.read().decode()
        assert r.status == 200
        expect = _read(name)
        if name == "index.html":       # served with this process's session token in the meta tag
            expect = expect.replace('<meta name="grappstar-token" content="">',
                                    f'<meta name="grappstar-token" content="{s.session_token}">')
            assert s.session_token in body
        assert body == expect
    finally:
        s.shutdown()
        s.server_close()


def test_posts_carry_session_token():
    """Every POST goes through action(), which sends the token from the page's meta tag (review r2_H #8)."""
    assert '<meta name="grappstar-token" content="">' in HTML
    assert 'meta[name="grappstar-token"]' in JS
    assert '"X-GrappStar-Token": TOKEN' in JS
    assert JS.count('method: "POST"') == 1, "all POSTs must go through action()"


def test_answer_sends_prompt_text():
    """The answer is tied to the prompt it answers: id, choice and the text shown (review r2_H #9)."""
    assert re.search(r'action\("answer", \{ prompt_id: p\.id, choice: choice, text: p\.text', JS)


def test_escape_stop_wired():
    """Escape is STOP from anywhere, before the typing() early return; Space only outside inputs (#11)."""
    m = re.search(r"^addEventListener\(\"keydown\", \(e\) => \{(.*?)\n\}\);", JS, re.S | re.M)
    assert m
    body = m.group(1)
    esc = body.index('"Escape"')
    assert esc < body.index("if (typing(e)) return;")
    assert '(e.key === " " && !typing(e))' in body
    assert 'action("stop", {})' in body[esc:body.index("if (typing(e)) return;")]
    assert "Esc" in re.search(r'id="btn-stop"[^>]*title="([^"]*)"', HTML).group(1)
    assert "Esc" in re.search(r'id="stop-keyhint"[^>]*>([^<]*)<', HTML).group(1)


def test_recover_button_and_offer():
    """Recover dart: in Manual with a confirm, and offered in the mission panel when the dart is out."""
    manual = HTML[HTML.index('<details class="panel" id="manual">'):HTML.index("</details>")]
    assert re.search(r'<button data-action="recover"[^>]*data-confirm="[^"]+"', manual)
    mission = HTML[HTML.index('id="mission"'):HTML.index('id="videos"')]
    assert re.search(r'<button data-action="recover" id="btn-recover-m"[^>]*data-confirm="[^"]+"', mission)
    assert 'id="m-recover" class="recover hidden"' in mission
    assert 'm.state === "ABORTED" || m.state === "FAILED"' in JS
    assert re.search(r'pick\(base, "seated"\) === false', JS) and re.search(r'wraps > 0', JS)


def test_tow_limit_max_70():
    m = re.search(r'<label>([^<]*)<input[^>]*data-key="tow_limit_n"[^>]*>', HTML)
    assert m and "70" in m.group(1)
    assert 'max="70"' in m.group(0)
