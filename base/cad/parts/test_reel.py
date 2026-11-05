"""PC tests for the reel CAD module (worker cad-reel). Run from base/:  python -m pytest -q cad/parts/test_reel.py"""
import math
import os
import sys

import numpy as np
import pytest

BASE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if BASE not in sys.path:
    sys.path.insert(0, BASE)

from tools.params import load_params  # noqa: E402
from cad.lib import prim, line, fits  # noqa: E402
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
        assert x1 <= 256 and y1 <= 256 and z1 <= 260, n


def test_joints_and_kin_cover_each_other():
    js = {j.name for j in reel.joints(P)}
    k = reel.kin(P, {})
    assert set(k) <= js
    for p in reel.parts(P):
        assert p.joint in js


def test_forward_wiper_turns_bus_in_take_up_sense(monkeypatch):
    monkeypatch.setattr(reel, "_drum_vals", lambda P, S: None)        # my own S-key path (no cad-drum kin)
    a = reel.kin(P, dict(wiper_deg=0.0))
    b = reel.kin(P, dict(wiper_deg=36.0))
    assert b["reel_bus"] - a["reel_bus"] == pytest.approx(36.0 * reel.Z_BUSMAIN / reel.Z_C1)
    # reel shaft (clutch disk) turns + too, 2.5 x wiper
    assert b["reel_shaft"] - a["reel_shaft"] == pytest.approx(36.0 * 2.5, rel=1e-9)


def test_swing_parks_on_backward_turns():
    assert reel.kin(P, dict(wiper_dir=1))["reel_swing"] == 0.0
    assert reel.kin(P, dict(wiper_dir=-1))["reel_swing"] == -reel.SWING_OUT_DEG


def test_wraps_map_to_cup_minus_z():
    assert reel.kin(P, dict(wraps=7))["reel_cup"] == pytest.approx(-7 * 360.0)


def test_take_up_meshes_clear_at_many_angles():
    """bus gear / swing idler / reel gear: no tooth overlap at 13 bus angles over two tooth pitches, idler engaged."""
    for k in range(13):
        S = dict(bus_deg=k * 30.0 / 13.0)
        sol = reel._local_posed(P, S)
        assert (sol["reel_bus"] ^ sol["reel_gi"]).volume() < 1e-3
        assert (sol["reel_gi"] ^ sol["reel_gr"]).volume() < 1e-3


def test_parked_idler_is_out_of_mesh():
    sol = reel._local_posed(P, dict(wiper_dir=-1))
    from manifold3d import Manifold
    gap = sol["reel_gi"].min_gap(sol["reel_gr"], 5.0)
    assert gap > 1.0


def test_line_path_clear():
    assert reel.line_check(P, [0.0, 1.0, 1.25, 1.5, 1.75, float(reel.WRAPS)]) == []


def test_launch_line_bypasses_capstan():
    spans = reel._line_points(P, 0.0)
    pts = np.vstack([s[0] for s in spans])
    drum = reel._local_posed(P, {})["reel_drum"]
    assert line.line_hits(pts, {"d": drum})["d"] < 1e-6


def test_v7_rules():
    assert reel_v7.RING_AT - reel_v7.CUP_OUT == pytest.approx(0.5)      # ring 1 mm outside the groove line, 0.5 outside the wall
    assert reel_v7.EZP - reel_v7.CUP_D == pytest.approx(0.5)            # ring 0.5 in front of the face
    assert reel.EXIT[2] - (reel.Z_CUP + reel_v7.EZP) == pytest.approx(31.5)
    assert reel.Z_DRUM - reel.GUIDE[2] == pytest.approx(1.0)
    assert reel_v7.TH_RIM - reel_v7.TH_TOP == pytest.approx(57.9, abs=0.5)


def test_text_has_no_em_dash(parts):
    txt = [p.notes for p in parts.values()] + [s["title"] + s["text"] for s in reel.steps(P)]
    src = open(reel.__file__).read() + open(reel_v7.__file__).read()
    assert "\u2014" not in "".join(txt) + src


def test_steps_name_real_parts(parts):
    seen = set()
    for s in reel.steps(P):
        for n in s["parts"]:
            assert n in parts, n
            seen.add(n)
    assert seen == set(parts)


def test_line_path_world_contract():
    pts = reel.line_path(P, dict(wraps=0.0, line_out_m=4.6))
    assert len(pts) > 5 and all(len(p) == 3 for p in pts)
    assert pts[-1][1] == pytest.approx(reel.P2_WORLD_Y, abs=0.5)
    gaps = [np.linalg.norm(np.subtract(a, b)) for a, b in zip(pts[:-1], pts[1:])]
    assert max(gaps) < 400.0 and len(pts) < 250
    pts7 = reel.line_path(P, dict(wraps=7.0))
    assert len(pts7) > len(pts)


def test_clear_of_main_axis_stack():
    assert reel.main_axis_check(P) == []


def test_cup_follows_drum_idler_seven_turns():
    if reel._drum_vals(P, {}) is None:
        pytest.skip("cad-drum module absent")
    assert reel.kin(P, dict(drum_deg=0.0))["reel_cup"] == pytest.approx(0.0, abs=1e-6)
    assert reel.kin(P, dict(drum_deg=302.0))["reel_cup"] == pytest.approx(-7 * 360.0, abs=1e-3)


def test_printed_parts_watertight_after_vertex_merge(parts):
    for n, p in parts.items():
        if p.printed:
            m = prim.to_trimesh(p.solid)
            m.merge_vertices()
            assert m.is_watertight, n


def test_brake_linkage_gives_the_rocker_stroke():
    """cam lift 4 less the 0.2 gap turns the crank so arm B lifts the leg by the stroke the real rocker section
    needs for LIGHT -> FIRM (reel_calc.rocker_section), within 3 percent."""
    from cad.parts import reel_calc
    y_contact = reel.FIRM_SCREW[1]                                     # arm B lifts the FIRM screw
    need = reel_calc.rocker_section(dict(reel_calc.DEFAULTS), y_load=y_contact)["stroke"]
    got = reel.leg_push_mm(reel.BRAKE_STROKE - reel.BAR_GAP) - reel.FIRM_GAP
    assert got == pytest.approx(need, rel=0.03)
    assert reel.leg_push_mm(0.0) == pytest.approx(0.0, abs=1e-9)
    # FIRM adjustable over E 4 .. 9 GPa: g from 0 to 2.0 covers the needed stroke
    for E in (4000.0, 9000.0):
        st = reel_calc.rocker_section(dict(reel_calc.DEFAULTS), y_load=y_contact, E=E)["stroke"]
        assert 0.0 <= reel.leg_push_mm(reel.BRAKE_STROKE - reel.BAR_GAP) - st <= 2.0


def test_bar_follows_the_drum_cam():
    assert reel.bar_mm(P, dict(drum_deg=0.0)) == 0.0                       # LAUNCH: LIGHT
    assert reel.bar_mm(P, dict(drum_deg=347.0)) == pytest.approx(reel.BRAKE_STROKE - reel.BAR_GAP)   # REEL: FIRM
    cx, cy = reel.cam_contact(P)
    (x0, y0, z0), (x1, y1, z1) = prim.bbox(reel._bar())
    assert x1 == pytest.approx(cx - reel.BAR_GAP, abs=1e-3)
    assert y0 < cy < y1


def test_brake_parts_clear_through_the_stroke():
    """bar, crank and the wall (guides, boss, recess) never overlap from LIGHT to FIRM; arm A stays on the bar end."""
    from cad.lib import tf as _tf
    wall = {p.name: p for p in reel.parts(P)}["reel_wall"].solid
    crank0, bar0 = reel._crank(), reel._bar()
    for s in (0.0, 1.0, 2.0, 3.0, reel.BRAKE_STROKE - reel.BAR_GAP):
        cr = prim.place(crank0, _tf.T(reel.CRANK_X, 0, reel.CRANK_Z) @ _tf.R((0, 1, 0), reel.crank_deg(s)))
        br = bar0.translate((-s, 0, 0))
        assert (cr ^ wall).volume() < 1e-3, s
        assert (br ^ wall).volume() < 1e-3, s
        assert (cr ^ br).volume() < 0.1, s          # tip circle facets
        assert cr.min_gap(br, 1.0) < 0.1, s


def test_pulley2_swivel_keeps_the_line_in_its_groove():
    """D43 review: the line leaves pulley 2 toward cad-launch's pivot pulley within 2 deg of the groove plane at every
    aim (the swivel follows the line)."""
    o = np.asarray(reel.SP["origin"], float)
    for aim in (6.0, 20.0, 36.0, 47.0, 60.0):
        S = dict(wraps=0.0, aim_deg=aim)
        ph = math.radians(reel.swivel_deg(P, S))
        axis_r = np.array((-math.sin(ph), math.cos(ph), 0.0))              # pulley 2 axis (R) after the swivel
        axis_w = np.array((axis_r[0], -axis_r[2], axis_r[1]))
        pts = np.asarray(reel.line_path(P, S), float)
        k = [i for i, p in enumerate(pts) if abs(p[1] - reel.P2_WORLD_Y) < 0.6]
        d = pts[k[1]] - pts[k[0]]
        d = d / np.linalg.norm(d)
        assert abs(math.degrees(math.asin(abs(float(np.dot(d, axis_w)))))) < 2.0, aim


# ---------------------------------------------------------------------------------------------- cross-check fixes
def _to_world():
    sp = reel._spec(P)
    M = np.eye(4)
    M[:3, 0], M[:3, 1], M[:3, 2] = [np.asarray(a, float) for a in sp["axes"]]
    M[:3, 3] = sp["origin"]
    return M


def _foot_slice(parts, name):
    """cad-frame's view of a reel foot (_frame_others.classify_slice 0.5 above the deck): (outline, hole centres)."""
    from shapely.ops import unary_union
    from cad.parts import _frame_others as OTH
    s = prim.place(parts[name].solid, _to_world())
    polys, holes = OTH.classify_slice(s, P.LAYOUT["deck_top_z"] + 0.5)
    return unary_union(polys), holes


def test_p2_post_foot_holes_are_real_holes(parts):
    """cad_support-5: the pulley 2 post's four M3 foot holes are round holes inside its foot (they broke out of the
    old 17.7 mm foot), with at least 2.5 mm of solid round each, so cad-frame finds them and cuts the inserts."""
    from shapely.geometry import Point
    foot, holes = _foot_slice(parts, "reel_p2_post")
    assert len(holes) == 4
    for x, y in holes:
        assert foot.exterior.distance(Point(x, y)) - fits.M3_CLEAR_D / 2 >= 2.5, (x, y)


def test_p2_post_gussets_brace_the_column(parts):
    """The gussets are fused to the column (they stood 0.15 mm off its faces): one section above the foot."""
    from cad.parts import _frame_others as OTH
    s = prim.place(parts["reel_p2_post"].solid, _to_world())
    for h in (8.0, 20.0):
        polys, _ = OTH.classify_slice(s, P.LAYOUT["deck_top_z"] + h)
        assert len(polys) == 1, h


def test_deck_holes_are_the_foot_holes(parts):
    """deck_holes(P) (read by _frame_others.deck_contacts) lists exactly the M3 holes of the three reel feet."""
    sliced = []
    for n in ("reel_wall", "reel_exit_post", "reel_p2_post"):
        sliced += _foot_slice(parts, n)[1]
    dh = reel.deck_holes(P)
    assert len(dh) == len(sliced) == 15
    for x, y in dh:
        assert min(math.hypot(x - a, y - b) for a, b in sliced) < 0.05, (x, y)


def test_sw_bracket_has_pilots_under_the_switch_base_holes(parts):
    """cad_support-11: a d2.6 self tap pilot through the bracket plate under each of the switch base's four holes,
    solid round it."""
    tm = prim.to_trimesh(parts["reel_sw_bracket"].solid)
    M = reel.sw_frame()
    for x, y in reel.sw_base_holes():
        for z in (-0.5, -2.0, -3.5):                                  # through the 4 mm plate (switch frame)
            c = (M @ np.array((x, y, z, 1.0)))[:3]
            assert not tm.contains([c])[0], (x, y, z)
            ring = [(M @ np.array((x + 2.2 * math.cos(a), y + 2.2 * math.sin(a), z, 1.0)))[:3]
                    for a in np.linspace(0, 2 * math.pi, 8, endpoint=False)]
            assert tm.contains(ring).all(), (x, y, z)


def test_sw_arm_texts_follow_the_polarity(parts):
    """cad_support-7 / docs-3: SW_ARM is CLOSED on the cup gear's track and OPEN in its home notch (params SWITCHES,
    config.h armHomeOpen); the step and the notes must not describe a bump that closes it at home."""
    st = [s for s in reel.steps(P) if s["title"] == "SW_ARM contact"][0]["text"]
    assert "closed while the cup is away from home and opens at home" in st
    assert "notch" in st and "bump" not in st
    for n in ("reel_gk", "reel_sw_lever"):
        assert "bump" not in parts[n].notes and "notch" in parts[n].notes, n
    assert "opens" in str(P.SWITCHES["SW_ARM"]["closes"]).lower()


def test_pivot_pulley_centre_on_the_rail_normal():
    """cad_support-10: the pivot pulley centre is PIV_R up the rail normal from launch.line_entry (its bottom), at the
    rail pitch launch.kin poses: the same centre cad-launch builds the pulley on."""
    L = pytest.importorskip("cad.parts.launch")
    from cad.lib import tf
    for aim in (6.0, 21.0, 36.0):
        S = dict(aim_deg=aim)
        e, c, u = reel._pivot(P, S)
        pitch = L.kin(P, S)["launch_rail"]
        want = tf.apply(L._rail_to_world(P, pitch), np.array([L.LN.geom(L.L(P))["piv"]]))[0]
        assert np.allclose(c, want, atol=1e-6), aim
        assert np.linalg.norm(c - e) == pytest.approx(reel.PIV_R)
        assert abs(float(np.dot(c - e, u))) < 1e-9


def test_feed_to_the_pivot_pulley_clears_the_drum_shift_rail():
    """cad_support-10: the real run from pulley 2 to the tangent on the pivot pulley clears cad-drum's d10 shift rail
    by 0.5 mm or more over the low aims (the 6 deg tow pitch, D45, is the closest)."""
    D = pytest.importorskip("cad.parts.drum")
    g = D.G(P)
    jb = [j for j in D.joints(P) if j.name == "drum_base"][0]
    rod = prim.place(D.rail(g), jb.local(0.0))
    for aim in (6.0, 10.0, 15.0):
        sp = {lab: pts for pts, _, lab in reel.line_spans(P, dict(aim_deg=aim, wraps=0.0))}
        gap = line.line_tube(sp["pulley 2 -> pivot pulley"]).min_gap(rod, 5.0)
        assert gap >= 0.5, (aim, gap)


def test_line_follows_the_posed_cup():
    """The line's cup ring is where kin poses the cup (cad-drum's wrap idler), not at S["wraps"] (up to half a turn
    apart mid-wrap), so the line meets the posed parts. (In the first quarter turn the line is drawn on its launch
    path, ring at home: the turn model starts there.)"""
    from tools.timeline import phases, state_at
    ph = {p["name"]: p for p in phases(P)}["wrap"]
    cp = reel_v7.cup_line_points()
    M = _to_world()
    apart = 0.0
    for f in (0.3, 0.5, 0.8):
        S = state_at(P, ph["t0"] + f * (ph["t1"] - ph["t0"]))
        cup = reel.kin(P, S)["reel_cup"]
        assert -cup / 360.0 >= 0.25, f
        apart = max(apart, abs(-cup / 360.0 - S["wraps"]))
        want = (M @ np.r_[reel._rz(cp["ring_front"], cup) + np.array((0, 0, reel.Z_CUP)), 1.0])[:3]
        sp = {lab: pts for pts, _, lab in reel.line_spans(P, S)}
        assert np.allclose(sp["cup ring"][-1], want, atol=1e-6), f
    assert apart > 0.1                    # the samples include a state where S["wraps"] would put the ring elsewhere


def test_line_path_is_the_spans():
    """line_path (the cad-infra contract) is the line_spans points in order, every span names its contacts, and the
    reel's part ends at launch.line_entry."""
    for S in (dict(wraps=0.0, aim_deg=6.0), dict(wraps=7.0, aim_deg=6.0), dict(wraps=3.3, aim_deg=36.0)):
        spans = reel.line_spans(P, S)
        pts = np.asarray(reel.line_path(P, S), float)
        flat = [spans[0][0][0]]
        for sp_, allow, label in spans:
            assert allow and label
            for p in sp_:
                if np.linalg.norm(p - flat[-1]) > 1e-6:
                    flat.append(p)
        assert np.allclose(np.asarray(flat), pts)
        assert np.allclose(pts[-1], reel._pivot(P, S)[0], atol=1e-6)


def test_tether_check_reports_a_part_across_the_line():
    """The whole-assembly tether check flags a part of another subsystem lying across a free span, and nothing on the
    reel by itself (its own parts touch the line only where each span allows)."""
    from cad.lib import Part
    from tools.assembly import Assembly
    asm = Assembly(P, {"reel": reel})
    S = [dict(phase="t", t=0.0, wraps=0.0, aim_deg=6.0), dict(phase="t", t=1.0, wraps=7.0, aim_deg=6.0)]
    assert reel.tether_check(P, asm, S) == []
    sp = {lab: pts for pts, _, lab in reel.line_spans(P, S[0])}
    c = sp["pulley 1 -> pulley 2"].mean(axis=0)
    asm.parts["frame_test_block"] = Part("frame_test_block", prim.box(c[0] - 4, c[0] + 4, c[1] - 2, c[1] + 2,
                                                                      c[2] - 4, c[2] + 4), "PLA")
    fails = reel.tether_check(P, asm, S)
    assert len(fails) == 1 and "pulley 1 -> pulley 2 hits frame_test_block" in fails[0] and "in 2 of 2" in fails[0]
