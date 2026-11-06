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
        assert p.joint is None or p.joint in js


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


def _pairs_clear(pairs, states):
    for S in states:
        sol = reel._posed_world(P, S)
        for a, b in pairs:
            assert (sol[a] ^ sol[b]).volume() < 1e-3, (a, b, S)


def test_take_up_meshes_clear_at_many_angles():
    """bus gear / swing idler / reel gear: no tooth overlap at 13 bus angles over two tooth pitches, idler engaged."""
    _pairs_clear([("reel_bus", "reel_gi"), ("reel_gi", "reel_gr")], [dict(bus_deg=k * 30.0 / 13.0) for k in range(13)])


def test_d50_gear_trains_mesh_clear():
    """Every D50 spur pair (riser, REEL idler, clutch hub and T2, rear trains, take-up idler) keeps zero overlap over
    a tooth pitch of its chain: the phases (PHASES_D50) match the axes."""
    riser = [("reel_gk1", "reel_ik1"), ("reel_ik1", "reel_ik2"), ("reel_ik2", "reel_gk2"),
             ("reel_ga2", "reel_ikr"), ("reel_ikr", "reel_gkc")]
    cap = [("reel_ic", "reel_gc"), ("reel_ga", "reel_icr"), ("reel_icr", "reel_gc2")]
    tk = [("reel_hub", "reel_gt2"), ("reel_gt3", "reel_ti"), ("reel_ti", "reel_gs")]
    _pairs_clear(riser, [dict(wraps=k / 60.0) for k in range(4)])
    _pairs_clear(cap, [dict(reel_deg=k * 3.0, capstan_deg=k * 3.0 * 50 / 22) for k in range(4)])
    _pairs_clear(tk, [dict(spool_deg=k * 4.0) for k in range(4)])


def test_parked_idler_is_out_of_mesh():
    sol = reel._posed_world(P, dict(wiper_dir=-1))
    assert sol["reel_gi"].min_gap(sol["reel_gr"], 5.0) > 1.0


def test_line_path_clear():
    assert reel.line_check(P, [0.0, 1.0, 1.25, 1.5, 1.75, float(reel.WRAPS)]) == []


def test_unwrapped_line_is_straight_through_the_eyelet():
    """D50: at 0 wraps the line runs from the spool pack top through the axial eyelet on one straight line, which is
    the rail's dart axis at the low-aim pitch extended back (the floor segment is straight at the tow pitch)."""
    S = dict(wraps=0.0, aim_deg=6.01, line_out_m=2.0)
    sp = {lab: pts for pts, _, lab in reel.line_spans(P, S)}
    a, b = sp["spool -> cup eyelet"]
    c, d = sp["eyelet -> rail-rear pulley"]
    for v in (b - a, d - c):
        ang = math.degrees(math.atan2(v[2], v[0]))
        assert ang == pytest.approx(reel.SP["low_pitch"], abs=0.6)
        assert abs(v[1]) < 1e-6                                       # no sideways turn: all in y 140
    L = pytest.importorskip("cad.parts.launch")
    assert L.pulley_bend_deg(P, dict(aim_deg=6.01)) == pytest.approx(0.0, abs=0.5)
    assert 100.0 < L.pulley_bend_deg(P, dict(aim_deg=47.0)) < 135.0


def test_capstan_and_cup_senses():
    """Forward REEL wiper turns wind the capstan + about world +X (cup axis forward); the cup turns - about +X in the
    wrap; the capstan ratio stays 50/22 and the cup follows the front cup gear 1:1 (7 wraps)."""
    a, b = reel.kin(P, dict(reel_deg=0.0)), reel.kin(P, dict(reel_deg=22.0))
    assert b["reel_capstan"] - a["reel_capstan"] == pytest.approx(50.0, rel=1e-9)
    w = reel.kin(P, dict(wraps=7.0))
    assert w["reel_cupr"] == pytest.approx(-7 * 360.0) and w["reel_cup"] == pytest.approx(-7 * 360.0)
    k = reel.kin(P, dict(line_out_m=1.0))
    assert k["reel_spool_r"] == pytest.approx(k["reel_spool"]) and k["reel_spool"] < 0.0


def test_reel_unit_on_the_deck_extension(parts):
    """The reel unit stands on the deck extension (behind x -164) and inside its tile (bed 248 with margin)."""
    ru = P.REEL_UNIT["deck_extension"]["x"]
    for n in ("reel_cheek_i", "reel_cheek_o", "reel_p1_leg", "reel_p2_leg", "reel_t_bracket", "reel_post_t"):
        (x0, y0, z0), (x1, y1, z1) = prim.bbox(parts[n].solid)
        assert z0 == pytest.approx(P.LAYOUT["deck_top_z"], abs=1e-6), n
        assert ru[0] + 6.0 <= x0 and x1 <= ru[1], n


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
    assert len(pts) >= 4 and all(len(p) == 3 for p in pts)
    assert all(p[1] == pytest.approx(P.REEL_UNIT["line_y"], abs=1e-6) for p in pts)
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


def test_line_path_is_the_spans():
    """line_path (the cad-infra contract) is the line_spans points in order, every span names its contacts, and the
    reel's part ends at launch.line_entry."""
    L = pytest.importorskip("cad.parts.launch")
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
        assert np.allclose(pts[-1], L.line_entry(P, S), atol=1e-6)


def test_tether_check_reports_a_part_across_the_line():
    """The whole-assembly tether check flags a part of another subsystem lying across a free span, and nothing on the
    reel by itself (its own parts touch the line only where each span allows)."""
    from cad.lib import Part
    from tools.assembly import Assembly
    asm = Assembly(P, {"reel": reel})
    S = [dict(phase="t", t=0.0, wraps=0.0, aim_deg=6.0), dict(phase="t", t=1.0, wraps=7.0, aim_deg=6.0)]
    assert reel.tether_check(P, asm, S) == []
    sp = {lab: pts for pts, _, lab in reel.line_spans(P, S[0])}
    c = sp["eyelet -> rail-rear pulley"].mean(axis=0)
    asm.parts["frame_test_block"] = Part("frame_test_block", prim.box(c[0] - 4, c[0] + 4, c[1] - 2, c[1] + 2,
                                                                      c[2] - 4, c[2] + 4), "PLA")
    fails = reel.tether_check(P, asm, S)
    assert len(fails) == 1 and "eyelet -> rail-rear pulley hits frame_test_block" in fails[0] and "in 2 of 2" in fails[0]
