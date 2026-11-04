"""PC tests for the reel CAD module (worker cad-reel), D49 rail reel. Run from base/:
python -m pytest -q cad/parts/test_reel.py"""
import math
import os
import sys

import numpy as np
import pytest

BASE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if BASE not in sys.path:
    sys.path.insert(0, BASE)

from tools.params import load_params  # noqa: E402
from cad.lib import prim, gears  # noqa: E402
from cad.parts import reel, reel_v7  # noqa: E402

P = load_params()


@pytest.fixture(scope="module")
def parts():
    return {p.name: p for p in reel.parts(P)}


def test_parts_valid(parts):
    for n, p in parts.items():
        assert n.startswith("reel_")
        assert p.solid.status().name == "NoError", n
        assert p.solid.volume() > 0, n
        if p.printed:
            assert prim.shells(p.solid) == 1, n


def test_print_fit_bed(parts):
    for n, p in parts.items():
        if not p.printed:
            continue
        (x0, y0, z0), (x1, y1, z1) = prim.bbox(prim.place(p.solid, p.print_tf))
        assert abs(z0) < 1e-6 and abs(x0) < 1e-6 and abs(y0) < 1e-6, n
        assert sorted((x1, y1))[0] <= 256 and sorted((x1, y1))[1] <= 256 * 1.414 and z1 <= 260, n


def test_everything_rides_the_rail(parts):
    """D49: no reel part is world-fixed; every joint chain ends at cad-launch's rail joint."""
    js = {j.name: j for j in reel.joints(P)}
    for n, p in parts.items():
        j = p.joint
        while j in js:
            j = js[j].parent
        assert j == "launch_rail", n


def test_joints_and_kin_cover_each_other():
    js = {j.name for j in reel.joints(P)}
    k = reel.kin(P, {})
    assert set(k) <= js
    for p in reel.parts(P):
        assert p.joint in js or p.joint == "launch_rail"


def test_axial_eyelet_on_the_dart_axis():
    """The one change to Harry's cup: the ring hole is axial at radius 26, so with the cup at home the eyelet sits on
    the dart axis (w 84.6) straight in line with the spool pack top."""
    assert reel_v7.RING_AT == pytest.approx(26.0)
    assert reel.CAP_W - reel_v7.RING_AT == pytest.approx(reel.LINE_W, abs=1e-6)
    sp = reel.line_spans(P, dict(wraps=0.0, aim_deg=6.0, line_out_m=2.0))
    pts = np.vstack([s[0] for s in sp])
    d = pts[-1] - pts[0]
    for p in pts[1:-1]:                       # collinear: spool -> eyelet rear -> eyelet front
        assert np.linalg.norm(np.cross(p - pts[0], d)) / np.linalg.norm(d) < 0.6
    assert reel_v7.TH_TOP == pytest.approx(90.0) and reel_v7.TH_RIM - reel_v7.TH_TOP == pytest.approx(57.9, abs=0.5)


def test_line_is_straight_and_pitch_independent_in_the_rail():
    """Spool, cup and dart all ride the rail: the line in the rail frame does not change with the pitch."""
    a = np.asarray(reel.line_path(P, dict(aim_deg=6.0)))
    b = np.asarray(reel.line_path(P, dict(aim_deg=47.0)))
    Ma = np.linalg.inv(reel._rail_to_world(P, dict(aim_deg=6.0)))
    Mb = np.linalg.inv(reel._rail_to_world(P, dict(aim_deg=47.0)))
    from cad.lib import tf
    assert np.allclose(tf.apply(Ma, a), tf.apply(Mb, b), atol=1e-6)


def test_capstan_train_ratio_and_sense():
    """2.307 capstan turns per REEL gear turn, wind-in (+ about +u) when the REEL gear turns forward."""
    k0 = reel.kin(P, dict(capstan_deg=None) if False else dict(reel_deg=0.0, drum_deg=347.0, aim_deg=6.0))
    k1 = reel.kin(P, dict(reel_deg=10.0, drum_deg=347.0, aim_deg=6.0))
    assert (k1["reel_capstan"] - k0["reel_capstan"]) / 10.0 == pytest.approx(P.REEL["capstan_ratio"], rel=1e-6)


def test_take_up_sense_and_ratio():
    """Forward wiper turns wind the spool shaft (take-up, - about +y) at 2.5 x the wiper."""
    k0 = reel.kin(P, dict(wiper_deg=0.0, drum_deg=41.0, aim_deg=6.0))
    k1 = reel.kin(P, dict(wiper_deg=10.0, drum_deg=41.0, aim_deg=6.0))
    assert (k1["reel_s"] - k0["reel_s"]) / 10.0 == pytest.approx(-2.5, rel=1e-6)


def test_swing_parks_on_backward_turns():
    k = reel.kin(P, dict(wiper_dir=-1, drum_deg=0.0))
    assert abs(k["reel_swing"]) == pytest.approx(reel.SWING_OUT_DEG)
    p0, p1 = reel.idler_uw(0.0), reel.idler_uw(k["reel_swing"])
    t1 = (reel.T1_U, reel.T1_W)
    assert math.dist(p1, t1) > math.dist(p0, t1) + 2.0


def test_mesh_centres():
    gy, gw = reel.gb_yw()
    assert math.dist((reel.UC_Y, reel.UC_W), (gy, gw)) == pytest.approx(gears.center_distance(1.5, 26, 32), abs=1e-6)
    assert math.dist(reel.CAP_YW, (gy, gw)) == pytest.approx(gears.center_distance(1.5, 32, 23), abs=1e-6)
    assert math.hypot(reel.C1_U, reel.C1_W) == pytest.approx(gears.center_distance(2.0, 50, 49), abs=0.01)
    assert math.hypot(reel.C1P_U, reel.C1P_W) == pytest.approx(gears.center_distance(1.5, 50, 46), abs=0.01)
    ip = reel.idler_uw(0.0)
    assert math.dist(ip, (reel.C1P_U, reel.C1P_W)) == pytest.approx(gears.center_distance(1.5, 46, 30), abs=1e-6)
    assert math.dist(ip, (reel.T1_U, reel.T1_W)) == pytest.approx(gears.center_distance(1.5, 30, 20), abs=1e-6)


def test_line_check_clear():
    assert reel.line_check(P, [0.0, 1.0, 2.5, float(reel.WRAPS)]) == []


def test_deck_rule_at_47_deg():
    """Study 4.2: nothing of the head below the deck plane at the top pitch."""
    assert reel.deck_check(P) == []


def test_cup_follows_drum_idler_seven_turns():
    if reel._drum_vals(P, {}) is None:
        pytest.skip("cad-drum module absent")
    c0 = reel.kin(P, dict(drum_deg=0.0))["reel_cup"]
    c1 = reel.kin(P, dict(drum_deg=302.0))["reel_cup"]
    assert c1 - c0 == pytest.approx(-7 * 360.0, abs=1e-3)        # positive drum still WRAPS (firmware sign kept)


def test_text_has_no_em_dash(parts):
    txt = [p.notes for p in parts.values()] + [s["title"] + s["text"] for s in reel.steps(P)]
    src = open(reel.__file__).read() + open(reel_v7.__file__).read()
    assert "—" not in "".join(txt) + src


def test_steps_name_real_parts_with_stages(parts):
    seen = set()
    for s in reel.steps(P):
        assert 220 <= s["stage"] <= 239
        for n in s["parts"]:
            if n.startswith("launch_line"):          # the line itself is cad-launch's model, threaded in the head step
                continue
            assert n in parts, n
            seen.add(n)
    assert seen == set(parts)


def test_wiring_steps_start_with_the_battery_unplugged():
    for s in reel.steps(P):
        if "SW_" in s["text"] and ("wire" in s["text"].lower() or "contact" in s["title"].lower()):
            assert s["text"].startswith("Battery unplugged"), s["title"]


def test_printed_parts_watertight_after_vertex_merge(parts):
    for n, p in parts.items():
        if p.printed:
            m = prim.to_trimesh(p.solid)
            m.merge_vertices()
            assert m.is_watertight, n


def test_lever_follows_the_cam():
    """LIGHT at LAUNCH, about 4 deg at FIRM (NEUTRAL, REEL); the FIRM screw moves toward the thrust ring."""
    assert reel.lever_deg(P, dict(drum_deg=0.0)) == pytest.approx(0.0, abs=1e-6)
    a = reel.lever_deg(P, dict(drum_deg=41.0))
    assert 3.0 < a < 5.0
    assert reel.lever_deg(P, dict(drum_deg=347.0)) == pytest.approx(a, abs=1e-6)


def test_line_path_is_the_spans():
    S = dict(wraps=7.0, aim_deg=6.0)
    spans = reel.line_spans(P, S)
    pts = np.asarray(reel.line_path(P, S))
    flat = [spans[0][0][0]]
    for sp_, allow, label in spans:
        assert allow and label
        for p in sp_:
            if np.linalg.norm(p - flat[-1]) > 1e-6:
                flat.append(p)
    assert np.allclose(np.asarray(flat), pts)
    assert np.allclose(pts[-1], reel.line_exit(P, S))


def test_tether_check_reports_a_part_across_the_line():
    """The whole-assembly tether check flags a part of another subsystem lying across a free span, and nothing on the
    reel by itself (its own parts touch the line only where each span allows)."""
    from cad.lib import Part
    from tools.assembly import Assembly
    from cad.parts import launch, drum
    asm = Assembly(P, {"reel": reel, "launch": launch, "drum": drum})
    S = [dict(phase="t", t=0.0, wraps=0.0, aim_deg=6.0), dict(phase="t", t=1.0, wraps=0.0, aim_deg=30.0)]
    assert reel.tether_check(P, asm, S) == []
    sp = {lab: pts for pts, _, lab in reel.line_spans(P, S[0])}
    c = sp["spool -> eyelet"].mean(axis=0)
    asm.parts["frame_test_block"] = Part("frame_test_block", prim.box(c[0] - 2, c[0] + 2, c[1] - 2, c[1] + 2,
                                                                      c[2] - 2, c[2] + 2), "PLA")
    fails = reel.tether_check(P, asm, [S[0]])
    assert len(fails) == 1 and "spool -> eyelet hits frame_test_block" in fails[0]
