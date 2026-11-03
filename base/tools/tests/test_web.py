"""Visual tools on the demo: render (sheet + sections), sim and steps pages, track motion, body fragments."""
import base64
import glob
import json
import os
import re

import numpy as np
import pytest
from PIL import Image

from tools import render, sim, steps
from tools.assembly import Assembly
from tools.params import load_params
from tools.timeline import timeline

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EM = "—"


@pytest.fixture(scope="module")
def out(tmp_path_factory):
    d = str(tmp_path_factory.mktemp("web"))
    assert render.main([d, "--only", "demo"]) == 0
    sim.build(d, ["demo"], dt=0.1)
    steps.build(d, ["demo"])
    return d


def test_render_sheets(out):
    sheet = Image.open(os.path.join(out, "audit_sheet.png"))
    w, h = render.TILE
    assert sheet.size == (w * len(render.SHEET_AZ), h * len(render.SHEET_EL) + render.TITLE_H)
    a = np.asarray(sheet.convert("L"), np.float32)
    assert a.std() > 8                                       # not blank
    tiles = sorted(glob.glob(os.path.join(out, "views", "view_*.png")))
    assert len(tiles) == 24
    for f in tiles:                                          # every view shows something besides the background
        t = np.asarray(Image.open(f).convert("L"), np.float32)
        assert t.std() > 5, f
    sec = Image.open(os.path.join(out, "sections.png"))
    sw, sh = render.SEC_TILE
    assert sec.size == (sw * render.SEC_PER_AXIS, sh * 3 + render.TITLE_H)
    assert len(glob.glob(os.path.join(out, "views", "section_*.png"))) == 12
    assert np.asarray(sec.convert("L"), np.float32).std() > 8


def test_sim_page(out):
    page = open(os.path.join(out, "sim.html")).read()
    asm = Assembly.load(load_params(), ["demo"])
    for n in asm.parts:
        assert n in page
    for p in ("load", "fire", "winch"):
        assert f'"name":"{p}"' in page
    assert "cdnjs" not in page.split("<script>")[0]           # offline: three.js inline
    assert EM not in page


def _data(page):
    m = re.search(r"window\.CAD_DATA=(\{.*?\});\n", page, re.S)
    return json.loads(m.group(1))


def test_gear_track_moves(out):
    P = load_params()
    asm = Assembly.load(P, ["demo"])
    T = asm.tracks(timeline(P, 0.5))
    g = np.array(T["parts"]["demo_gear_1"]).reshape(-1, 4, 4)
    assert len(g) > 2 and np.abs(g[1:, :3, :3] - g[0, :3, :3]).max(axis=(1, 2)).max() > 0.1
    assert len(T["parts"]["demo_plate"]) == 1
    D = _data(open(os.path.join(out, "sim_body.html")).read())
    part = {p["name"]: p for p in D["parts"]}
    gi = part["demo_gear_1"]["group"]
    assert gi != 0 and part["demo_plate"]["group"] == 0
    q = np.frombuffer(base64.b64decode(D["tracks"]["groups"][gi]["q"]), np.int16).reshape(-1, 4)
    assert len(q) == D["tracks"]["n"] and np.abs(q - q[0]).max() > 1000
    # shaft and gear on one joint share the group
    assert part["demo_shaft_1"]["group"] == gi


def test_steps_page(out):
    page = open(os.path.join(out, "assembly.html")).read()
    D = _data(page)
    names = [n for s in D["steps"] for n in s["parts"]]
    assert sorted(names) == sorted(p["name"] for p in D["parts"])      # every part exactly once
    assert D["steps"][0]["title"] == "Press in the bearings"


def test_body_fragments(out):
    for stem in ("sim", "assembly"):
        b = open(os.path.join(out, f"{stem}_body.html")).read()
        assert b.lstrip().startswith("<title>") and "<!doctype" not in b.lower() and "<body" not in b
        assert '<meta name="claude:viewport-fit-sides" content="cover">' in b
        srcs = re.findall(r'<script[^>]*src="([^"]+)"', b)
        assert srcs == [render.THREE_CDN]
        assert "import(" not in b and "alert(" not in b and "confirm(" not in b and EM not in b
        assert "safe-area-inset-left" in b and ':root:not([data-theme="light"])' in b


def test_body_fragment_runs(out, tmp_path):
    """Wrap the sim body fragment in a document, serve three.js locally for the cdnjs URL, check it boots clean."""
    page = tmp_path / "wrap.html"
    page.write_text("<!doctype html><html><head><meta charset='utf-8'></head><body>"
                    + open(os.path.join(out, "sim_body.html")).read() + "</body></html>")
    shot = str(tmp_path / "body.png")
    res = render.run_shots(dict(url="file://" + str(page), width=400, height=800, ready="window.READY === true",
                                routes={render.THREE_CDN: os.path.join(render.WEB, "vendor", "three.min.js")},
                                shots=[dict(file=shot, ops=[["SIM_PAUSE"], ["SIM_SEEK", 12.0]])]))
    assert res["ok"], res["errors"]
    assert np.asarray(Image.open(shot).convert("L"), np.float32).std() > 8


def test_no_em_dashes_in_sources():
    files = [os.path.join(HERE, f) for f in ("render.py", "sim.py", "steps.py")]
    files += glob.glob(os.path.join(HERE, "web", "*.js")) + glob.glob(os.path.join(HERE, "web", "*.html"))
    for f in files:
        assert EM not in open(f, encoding="utf-8").read(), f
