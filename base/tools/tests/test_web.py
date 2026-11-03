"""Visual tools on the demo: render (sheet + sections), sim and steps pages, track motion, body fragments."""
import base64
import glob
import json
import math
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
    from tools.timeline import phases
    for p in [ph["name"] for ph in phases(load_params())]:
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
    J = D["tracks"]["joints"][gi - 1]
    assert J["name"] == asm.parts["demo_gear_1"].joint and J["kind"] == "rev"
    v = np.frombuffer(base64.b64decode(J["v"]), np.float64 if J["f64"] else np.float32)
    assert len(v) == D["tracks"]["n"] and np.ptp(v) > 10
    # shaft and gear on one joint share the group
    assert part["demo_shaft_1"]["group"] == gi


# ------------------------------------------------------------------ synthetic model: nested joints, fast gear, tether
def _synthetic():
    import types
    from cad.lib import Part, Joint, prim, tf
    blk = prim.box(-20, 20, -8, 8, -5, 5)
    joints = [Joint("arm0", None, "rev", (0, 0, 0), (0, 0, 1)),
              Joint("arm1", "arm0", "rev", (180, 0, 0), (0, 0, 1)),
              Joint("arm2", "arm1", "slide", (120, 0, 20), (1, 0, 0), base=tf.Rz(30)),
              Joint("arm3", "arm2", "rev", (40, 0, 0), (0, 1, 0)),
              Joint("fast", None, "rev", (0, 250, 0), (0, 0, 1), gear=("arm0", 15.0, 0.0))]
    parts = [Part("syn_plate", prim.box(-300, 300, -300, 300, -40, -30), "PLA", color="#9aa7b4"),
             Part("syn_disc", prim.tube(60, 20, 10, z=-30, seg=400), "PLA", color="#5a8fd0")]     # a larger mesh
    for j in joints:
        parts.append(Part(f"syn_{j.name}", blk, "PLA", joint=j.name, place=tf.T(25, 0, 0) @ tf.Rx(20), color="#d97a00"))

    def kin(P, S):
        t = S["t"]
        return {"arm0": 40.0 * t * t, "arm1": -40.0 * t, "arm2": 30.0 * math.sin(t), "arm3": 70.0 * t}

    def line_path(P, S):        # tether: point count changes between samples
        t = S["t"]
        pts = [(0, 0, 50), (300 + 100 * t, 0, 80 + 10 * t), (600 + 200 * t, 50, 60)]
        return pts if int(t * 10) % 7 else pts[:2]

    mod = types.SimpleNamespace(parts=lambda P: parts, joints=lambda P: joints, kin=kin, line_path=line_path)
    asm = Assembly(load_params(), {"syn": mod})
    assert not asm.errors, asm.errors
    states = [dict(t=round(0.1 * i, 3), phase="run") for i in range(41)]
    return asm, states


@pytest.fixture(scope="module")
def syn(tmp_path_factory):
    d = str(tmp_path_factory.mktemp("syn"))
    asm, states = _synthetic()
    asm.export(d)
    data = render.build_data(asm, d, states)
    full, body = render.write_pages(d, "sim", "sim_template.html", data, ["viewer_core.js", "sim_app.js"], "synthetic")
    return dict(dir=d, asm=asm, states=states, data=data, full=full, body=body)


POSE_JS = """(() => { SIM_PAUSE(); const V = SIM.viewer, out = {};
  for (const s of %s) { V.sample(s); const w = {}; for (const r of V.list) w[r.p.name] = Array.from(r.world.elements);
    out[String(s)] = { w, line: V.line() }; }
  return out; })()"""


def _expected(asm, states, s):
    vals = [asm.values(S) for S in states]
    i = min(int(s), len(states) - 2)
    f = s - i
    v = {k: (1 - f) * vals[i][k] + f * vals[i + 1][k] for k in vals[i]}
    JW = asm.joint_world(None, values=v)
    return {n: JW.get(p.joint, np.eye(4)) @ p.place for n, p in asm.parts.items()}, v


def test_synthetic_pose_accuracy(syn):
    """Nested chain and a joint turning > 180 deg per sample: the page matches Assembly.world at the samples and the
    joint-value interpolation between them (no wrong-way motion, nested joints stay on their pivots)."""
    asm, states = syn["asm"], syn["states"]
    vals = [asm.values(S) for S in states]
    assert max(abs(vals[i + 1]["fast"] - vals[i]["fast"]) for i in range(len(vals) - 1)) > 300   # aliasing case
    pts = [0, 7, 7.5, 12.25, 20.5, 33.75, 39.5, 40]
    res = render.run_shots(dict(url="file://" + syn["full"], width=640, height=480, ready="window.READY === true", shots=[],
                                eval=POSE_JS % json.dumps(pts)))
    assert res["ok"], res["errors"]
    worst = 0.0
    for s in pts:
        E, v = _expected(asm, states, s)
        got = res["result"][str(s) if str(s) in res["result"] else str(int(s))]["w"]
        for n, M in E.items():
            G = np.array(got[n]).reshape(4, 4).T
            worst = max(worst, np.abs(G[:3, 3] - M[:3, 3]).max(), 1000 * np.abs(G[:3, :3] - M[:3, :3]).max())
        # pivot: arm1's frame origin stays at (180, 0, 0) in arm0's frame
        A0 = np.array(got["syn_arm0"]).reshape(4, 4).T @ np.linalg.inv(asm.parts["syn_arm0"].place)
        A1 = np.array(got["syn_arm1"]).reshape(4, 4).T @ np.linalg.inv(asm.parts["syn_arm1"].place)
        assert np.abs(A1[:3, 3] - (A0 @ [180, 0, 0, 1])[:3]).max() < 0.01
        # fast joint: rotation about z by exactly the interpolated value (never the short way round)
        F = np.array(got["syn_fast"]).reshape(4, 4).T @ np.linalg.inv(asm.parts["syn_fast"].place)
        a = math.degrees(math.atan2(F[1, 0], F[0, 0]))
        assert abs((a - v["fast"] + 180) % 360 - 180) < 0.01, (s, a, v["fast"])
    assert worst < 0.01, worst


def test_synthetic_tether(syn):
    asm, states = syn["asm"], syn["states"]
    L = syn["data"]["tracks"]["line"]
    cnt = np.frombuffer(base64.b64decode(L["cnt"]), np.uint16)
    assert len(cnt) == len(states) and set(cnt) == {2, 3}
    pts = [10.5, 6.25, 6.75]       # equal counts (interpolated); 6 -> 7 changes the count (nearer sample)
    res = render.run_shots(dict(url="file://" + syn["full"], width=640, height=480, ready="window.READY === true", shots=[],
                                eval=POSE_JS % json.dumps(pts)))
    assert res["ok"], res["errors"]
    lines = [np.asarray(asm.line(S)) for S in states]
    got = {s: np.array(res["result"][str(s)]["line"]).reshape(-1, 3) for s in pts}
    assert np.abs(got[10.5] - (lines[10] + lines[11]) / 2).max() < 1e-3
    assert len(cnt) > 7 and cnt[6] != cnt[7]
    assert np.abs(got[6.25] - lines[6]).max() < 1e-3 and np.abs(got[6.75] - lines[7]).max() < 1e-3


def test_track_box_and_hull(syn):
    """box_track covers every sample (exact, from hull vertices) and is tighter than transformed local AABB corners."""
    asm, states, D = syn["asm"], syn["states"], syn["data"]
    from cad.lib import prim
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for S in states:
        W = asm.world(S)
        for n, p in asm.parts.items():
            v, _ = prim.mesh_of(p.solid)
            w = v @ W[n][:3, :3].T + W[n][:3, 3]
            lo, hi = np.minimum(lo, w.min(0)), np.maximum(hi, w.max(0))
    assert np.abs(np.array(D["box_track"]) - [lo, hi]).max() < 0.05


def test_size_limits(syn, tmp_path, monkeypatch):
    """Large data moves to web/*_ext_*.json with single arrays split across files; the page reassembles it; pages or
    files over the limits fail with a clear message."""
    d = str(tmp_path)
    monkeypatch.setattr(render, "INLINE_LIMIT", 20_000)
    monkeypatch.setattr(render, "FILE_LIMIT", 6_000)
    data = syn["data"]
    biggest = max(len(m["v"]) for m in data["meshes"].values())
    assert biggest > 6_000                                  # a single mesh array larger than one file
    full, body = render.write_pages(d, "sim", "sim_template.html", data, ["viewer_core.js", "sim_app.js"], "synthetic")
    files = sorted(glob.glob(os.path.join(d, "web", "sim_ext_*.json")))
    assert len(files) > 2 and all(os.path.getsize(f) <= 6_000 for f in files)
    B = _data(open(body).read())
    assert B["ext_files"] and isinstance(B["meshes"]["syn_disc"]["v"], dict) and B["meshes"]["syn_disc"]["v"]["n"] > 1
    srv, url = render.serve({"wrap.html": ("<!doctype html><html><head><meta charset='utf-8'></head><body>"
                                           + open(body).read() + "</body></html>").encode(),
                             **{"web/" + os.path.basename(f): f for f in files}})
    try:
        res = render.run_shots(dict(url=url + "wrap.html", width=640, height=480, ready="window.READY === true", shots=[],
                                    routes={render.THREE_CDN: os.path.join(render.WEB, "vendor", "three.min.js")},
                                    eval=POSE_JS % json.dumps([12.25])))
    finally:
        srv.shutdown()
    assert res["ok"], res["errors"]
    E, _ = _expected(syn["asm"], syn["states"], 12.25)
    got = res["result"]["12.25"]["w"]
    assert len(got) == len(E)
    for n, M in E.items():
        assert np.abs(np.array(got[n]).reshape(4, 4).T - M).max() < 1e-3
    monkeypatch.setattr(render, "PAGE_LIMIT", 10_000)
    with pytest.raises(render.PageTooLarge, match="limit for a published page"):
        render.write_pages(d, "sim", "sim_template.html", data, ["viewer_core.js", "sim_app.js"], "synthetic")
    monkeypatch.setattr(render, "PAGE_LIMIT", 16_000_000)
    monkeypatch.setattr(render, "FILE_MAX", 5_000)
    with pytest.raises(render.PageTooLarge, match="must stay below"):
        render.write_pages(d, "sim", "sim_template.html", data, ["viewer_core.js", "sim_app.js"], "synthetic")


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
                                shots=[dict(file=shot, ops=[["SIM_PAUSE"], ["SIM_SEEK", 12.0]])],
                                eval="[1, 2, 3].map(() => { document.querySelector('#theme').click(); return SIM.viewer.dark; })"))
    assert res["ok"], res["errors"]
    assert res["result"] == [False, True, False]            # auto (light here) -> light -> dark -> auto
    assert np.asarray(Image.open(shot).convert("L"), np.float32).std() > 8


def test_no_em_dashes_in_sources():
    files = [os.path.join(HERE, f) for f in ("render.py", "sim.py", "steps.py")]
    files += glob.glob(os.path.join(HERE, "web", "*.js")) + glob.glob(os.path.join(HERE, "web", "*.html"))
    for f in files:
        assert EM not in open(f, encoding="utf-8").read(), f
