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
from cad.lib import prim, line  # noqa: E402
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


def test_forward_wiper_turns_bus_in_take_up_sense():
    a = reel.kin(P, dict(wiper_deg=0.0))
    b = reel.kin(P, dict(wiper_deg=36.0))
    assert b["reel_bus"] - a["reel_bus"] == pytest.approx(36.0 * 40 / 24)
    # reel shaft (clutch disk) turns + too, 2.5 x wiper
    assert b["reel_shaft"] - a["reel_shaft"] == pytest.approx(36.0 * 2.5, rel=1e-9)


def test_swing_parks_on_backward_turns():
    assert reel.kin(P, dict(wiper_dir=1))["reel_swing"] == 0.0
    assert reel.kin(P, dict(wiper_dir=-1))["reel_swing"] == -reel.SWING_OUT_DEG


def test_wraps_map_to_cup_minus_z():
    assert reel.kin(P, dict(wraps=7))["reel_cup"] == pytest.approx(-7 * 360.0)


def test_take_up_meshes_clear_at_many_angles():
    """c2 / idler / reel gear: no tooth overlap at 13 bus angles over two tooth pitches, idler engaged."""
    for k in range(13):
        S = dict(bus_deg=k * 30.0 / 13.0)
        sol = reel._local_posed(P, S)
        assert (sol["reel_c2"] ^ sol["reel_gi"]).volume() < 1e-3
        assert (sol["reel_gi"] ^ sol["reel_gr"]).volume() < 1e-3


def test_parked_idler_is_out_of_mesh():
    sol = reel._local_posed(P, dict(wiper_dir=-1))
    from manifold3d import Manifold
    gap = sol["reel_gi"].min_gap(sol["reel_gr"], 5.0)
    assert gap > 0.5


def test_line_path_clear():
    assert reel.line_check(P, [0.0, 1.0, 1.25, 1.5, 1.75, float(reel.WRAPS)]) == []


def test_launch_line_bypasses_capstan():
    spans = reel.line_path(P, 0.0)
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
    assert "—" not in "".join(txt) + src


def test_steps_name_real_parts(parts):
    seen = set()
    for s in reel.steps(P):
        for n in s["parts"]:
            assert n in parts, n
            seen.add(n)
    assert seen == set(parts)
