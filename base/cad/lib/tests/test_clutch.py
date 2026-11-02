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
