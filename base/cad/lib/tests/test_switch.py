"""Tests for cad.lib.switch: valid parts, open at rest by the gap, closes at travel, hard stop, no clashes."""
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..")))

import manifold3d as m3
import pytest

from cad.lib import switch as SW, prim, tf


def V(a, b):
    return (a ^ b).volume()


@pytest.fixture(scope="module", params=[(25, 0.8), (35, 1.2)])
def sw(request):
    L, tr = request.param
    return SW.contact_switch(L, travel=tr)


def test_parts_valid(sw):
    for k in ("base", "lever", "screw", "nut", "jam_nut"):
        s = sw[k]
        assert s.status() == m3.Error.NoError and s.volume() > 0
        assert prim.shells(s) == 1, k


def test_no_clash_at_rest(sw):
    b, lv = sw["base"], sw["lever"]
    assert V(b, lv) < 1e-6
    assert V(sw["nut"], b) < 1e-6
    assert V(sw["screw"], b) < 1e-6
    assert V(sw["jam_nut"], lv) < 1e-6
    # screw thread engages the tap hole: overlap only inside the boss
    ov = sw["screw"] ^ lv
    (_, _, z0), _ = prim.bbox(ov)
    assert z0 >= sw["info"]["z_lever"] - 1e-6


def test_open_at_rest_by_gap(sw):
    i = sw["info"]
    assert i["gap"] >= i["travel"]
    assert V(sw["screw"].translate((0, 0, -(i["gap"] - 0.02))), sw["nut"]) < 1e-6
    assert V(sw["screw"].translate((0, 0, -(i["gap"] + 0.02))), sw["nut"]) > 0
    assert abs((i["z_head_face"] - i["z_nut_face"]) - i["gap"]) < 1e-9


def test_closes_at_travel(sw):
    i = sw["info"]
    M = SW.lever_pose(i, i["travel"])
    head = prim.place(sw["screw"], M)
    assert V(head, sw["nut"]) < 1e-4                                       # touching, not into it
    assert V(prim.place(sw["screw"], tf.T(0, 0, -0.05) @ M), sw["nut"]) > 0  # within 0.05 mm
    # still open a bit before the travel
    assert V(prim.place(sw["screw"], SW.lever_pose(i, i["travel"] - 0.1)), sw["nut"]) < 1e-6


def test_hard_stop_and_info(sw):
    i = sw["info"]
    assert i["stop_z"] < i["z_lever"]
    assert i["stop_defl"] > i["pretravel"]           # never blocks closing
    assert i["strain_ok"] and i["strain_close"] < i["strain_max"]
    assert 0.2 < i["force_close_N"] < 20
    assert i["contact_force_N"] > 0
    # stop sits under the rigid tip, clear of the nut pillar and screw head
    x0, x1 = i["stop_x"]
    assert x0 > i["x_screw"] + SW.M3_HEAD_DK / 2


def test_short_lever_rejected():
    with pytest.raises(ValueError):
        SW.contact_switch(12)


def test_strain_limit_default(sw):
    i = sw["info"]
    assert SW.STRAIN_LIMIT == 0.008
    assert i["strain_max"] <= SW.STRAIN_LIMIT and SW.T_MIN <= i["t"] <= SW.T_MAX


def test_explicit_t_still_works():
    with pytest.warns(UserWarning, match="strain"):
        i = SW.contact_switch(25, t=1.6)["info"]
    assert i["t"] == 1.6 and not i["strain_ok"]


def test_wire_channel_clear_of_clamp_screws(sw):
    from shapely.geometry import Point, Polygon
    i = sw["info"]
    base = sw["base"]
    for z in (0.5, 3.0 - 1.0, 3.0 - 0.1):      # through the channel depth (base_t 3, channel 2 deep)
        g = Polygon()
        for q in sorted((Polygon(p) for p in base.slice(z).to_polygons()), key=lambda p: -p.area):
            g = g.symmetric_difference(q.buffer(0))
        for x in i["clamp_screw_x"]:
            # a screw of any length on the clamp axis (major d 3.0) keeps 0.8 mm of plastic to the channel
            assert g.contains(Point(x, 0).buffer(1.5 + 0.8, 64)), (z, x)
    assert i["channel_y"][0] > 1.5 + 0.8
    assert i["clamp_screw_len"] <= i["clamp_screw_max_len"]
    assert "M3x" in i["clamp_screw"] and "M3" in i["mount_screw"]
