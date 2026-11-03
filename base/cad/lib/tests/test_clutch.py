"""Tests for cad.lib.clutch: dog clutch engagement, slip clutch stack, helper numbers."""
import itertools
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..")))

import manifold3d as m3
import pytest

from cad.lib import clutch as C, prim, tf


def ok(s):
    assert s.status() == m3.Error.NoError
    assert s.volume() > 0
    assert prim.shells(s) == 1


@pytest.fixture(scope="module")
def dog():
    return C.dog_clutch()


def test_dog_valid(dog):
    ok(dog["driver"])
    ok(dog["driven"])


def test_dog_engaged(dog):
    i = dog["info"]
    drv = dog["driver"]
    dvn = prim.place(dog["driven"], C.dog_mate(i))
    assert (drv ^ dvn).volume() < 1e-6
    g = i["gap_deg"]
    # driver advanced in its drive direction (+Z) relative to the driven: drive flanks bear
    assert (prim.place(drv, tf.Rz(1.5 * g)) ^ dvn).volume() > 1.0
    # within the flank gap both ways it stays free
    assert (prim.place(drv, tf.Rz(0.5 * g)) ^ dvn).volume() < 1e-6
    assert (prim.place(drv, tf.Rz(-0.5 * g)) ^ dvn).volume() < 1e-6
    # partial engagement also clear
    assert (drv ^ prim.place(dog["driven"], C.dog_mate(i, 1.5))).volume() < 1e-6


def test_dog_backcut_hooks(dog):
    # the drive flank leans forward: the tooth is wider at the tip than at the root (ramp 90)
    i = dog["info"]
    assert i["top_w"] > i["root_w"]


def test_dog_ratchet_keyed():
    d = C.dog_clutch(od=50, id=30, n_teeth=8, ramp_deg=35, bore=("keyed", 20, 6))
    ok(d["driver"])
    i = d["info"]
    dvn = prim.place(d["driven"], C.dog_mate(i))
    assert (d["driver"] ^ dvn).volume() < 1e-6
    assert (prim.place(d["driver"], tf.Rz(1.5 * i["gap_deg"])) ^ dvn).volume() > 1.0


def test_dog_bad_params():
    with pytest.raises(ValueError):
        C.dog_clutch(n_teeth=30, ramp_deg=20)


@pytest.fixture(scope="module")
def slip():
    return C.slip_clutch()


def test_slip_parts_valid(slip):
    for n, s in slip["parts"].items():
        ok(s)


def test_slip_stack_no_intersection(slip):
    posed = {}
    for n, M in slip["place"].items():
        base = n.rsplit("_", 1)[0] if n.startswith(("screw_", "nut_")) else n
        posed[n] = prim.place(slip["parts"][base], M)
    for a, b in itertools.combinations(posed, 2):
        assert (posed[a] ^ posed[b]).volume() < 1e-4, (a, b)


def test_slip_printable_flat(slip):
    for n in ("hub_plate", "friction_disc", "pressure_plate", "flexure", "adjuster_plate"):
        lo, hi = prim.bbox(slip["parts"][n])
        assert abs(lo[2]) < 1e-6 and hi[0] - lo[0] < 256


def test_slip_numbers(slip):
    T = C.slip_torque(10.0, 0.3, 16.5, 25.0, 2)
    assert abs(T - 2 * 0.3 * 10 * 20.75 / 1000) < 1e-9
    assert C.slip_torque(10.0, 0.3, 16.5, 25.0, 2, "pressure") > T
    i = slip["info"]
    g = i["flex_geom"]
    F = C.flexure_force(0.5, g)
    assert 2.0 < F < 30.0
    assert C.flexure_stress(i["max_deflection"], g) < C.SIGMA_FLEX_PLA
    # payout brake about 0.5 to 1 N at a 20 mm reel radius (10 to 20 N mm) is reachable in the travel
    Tmax = C.slip_torque(C.flexure_force(i["max_deflection"], g), i["mu"], i["r_in"], i["r_out"], 2)
    assert Tmax > 0.02
    assert i["table"][0][1] == 0.0


def _slice(solid, z):
    from shapely.geometry import Polygon
    g = Polygon()
    for p in sorted((Polygon(q) for q in solid.slice(z).to_polygons() if len(q) >= 3), key=lambda p: -p.area):
        g = g.symmetric_difference(p.buffer(0))
    return g


def test_slip_min_wall(slip):
    # eroding by half of 1.6 mm keeps every hole separate (no wall thinner than 1.6 mm around the screw features)
    P = slip["parts"]
    assert slip["info"]["min_wall"] >= 1.6
    for name, z in (("hub_plate", 0.5), ("hub_plate", 4.0), ("adjuster_plate", 3.5), ("pressure_plate", 1.5)):
        g = _slice(P[name], z)
        e = g.buffer(-0.8)
        assert e.geom_type == "Polygon", name
        assert len(e.interiors) == len(g.interiors), (name, z)


def test_slip_flexure_no_bridges(slip):
    tm = prim.to_trimesh(slip["parts"]["flexure"])
    down = (tm.face_normals[:, 2] < -0.7) & (tm.triangles_center[:, 2] > 0.05)
    assert tm.area_faces[down].sum() < 0.1
    g = slip["info"]["flex_geom"]
    assert g["z_beam"] == 0.0


def test_slip_sustained_stress(slip):
    i = slip["info"]
    g = i["flex_geom"]
    assert C.SIGMA_FLEX_PLA_SUSTAINED == 10.0
    assert C.flexure_stress(i["travel"], g) <= C.SIGMA_FLEX_PLA_SUSTAINED + 1e-9
    assert max(r[3] for r in i["table"]) <= C.SIGMA_FLEX_PLA_SUSTAINED
    assert i["table"][-1][2] > 0.02          # still reaches the payout brake torque
    assert C.flexure_travel(g, C.E_PETG) >= i["travel"] - 1e-9


def test_slip_flexure_on_pads(slip):
    # the flexure beam rests on the pressure plate pads, upper bumps touch the adjuster: zero gap, no overlap
    P, M = slip["parts"], slip["place"]
    flex = prim.place(P["flexure"], M["flexure"])
    pp = prim.place(P["pressure_plate"], M["pressure_plate"])
    adj = prim.place(P["adjuster_plate"], M["adjuster_plate"])
    assert (flex ^ pp).volume() < 1e-4
    assert (flex.translate((0, 0, -0.05)) ^ pp).volume() > 1e-3
    assert (flex.translate((0, 0, 0.05)) ^ adj).volume() > 1e-3


def test_flexure_lower_bumps_option():
    s, g = C.flexure_ring(16.5, 23.0, n=4, t=0.6, gap=0.7, lower_bumps=True)
    ok(s)
    assert g["z_beam"] == 0.7 and abs(prim.bbox(s)[0][2]) < 1e-9


def test_dog_ramp_axial_free():
    d = C.dog_clutch(od=50, id=30, n_teeth=8, ramp_deg=35, bore=("keyed", 20, 6))
    i = d["info"]
    assert abs(i["axial_free"] - i["tip_gap"]) < 1e-9
    drv = d["driver"]
    dvn = prim.place(d["driven"], C.dog_mate(i))
    assert (drv ^ dvn.translate((0, 0, -(i["tip_gap"] - 0.02)))).volume() < 1e-6
    assert (drv ^ dvn.translate((0, 0, -(i["tip_gap"] + 0.05)))).volume() > 1e-4
    # a narrow ramp gap is reported honestly
    j = C.dog_clutch(od=50, id=30, n_teeth=8, ramp_deg=35, ramp_gap=0.3)["info"]
    assert j["axial_free"] < j["tip_gap"]
    assert C.dog_clutch()["info"]["axial_free"] == C.dog_clutch()["info"]["tip_gap"]


def test_flexure_ring_watertight_as_stl():
    from cad.lib import prim, clutch
    r, _ = clutch.flexure_ring(16.5, 25)
    t = prim.to_trimesh(r)
    t.merge_vertices()
    assert t.is_watertight
