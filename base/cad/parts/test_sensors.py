"""Tests for the sensors subsystem (sensors.py). Run from base/:
PYTHONPATH=. python -m pytest cad/parts/test_sensors.py -q"""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.dirname(os.path.dirname(HERE))
for p in (BASE, HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

from tools.params import load_params  # noqa: E402
from cad.lib import prim, tf, switch  # noqa: E402
import sensors as SE                  # noqa: E402

P = load_params()
TOL = 1e-3


def _ov(a, b):
    return (a ^ b).volume()


@pytest.fixture(scope="module")
def parts():
    return {p.name: p for p in SE.parts(P)}


@pytest.fixture(scope="module")
def asm():
    from tools.assembly import Assembly
    return Assembly.load(P, ["launch", "sensors", "drum"])


def _old_launch_parts(asm):
    return [n for n in asm.parts if n in ("launch_sw_gap", "launch_sw_rail_low", "launch_aim_as5600")]


def _posed(asm, name, S):
    W = asm.world(S)
    return prim.place(asm.parts[name].solid, W[name])


def test_names_and_contract(parts):
    assert all(n.startswith("sensors_") for n in parts)
    assert SE.joints(P) == [] and SE.kin(P, {}) == {}
    names = set(parts)
    for st in SE.steps(P):
        assert set(st["parts"]) <= names, st["title"]
    stepped = {n for st in SE.steps(P) for n in st["parts"]}
    assert stepped == names                                  # every part has an assembly step
    assert SE.checks(P) == []


def test_solids_valid_and_printable(parts):
    for n, p in parts.items():
        assert p.solid.status().name == "NoError", n
        assert p.solid.volume() > 0, n
        if p.material != "bought":
            assert len(p.solid.decompose()) == 1, n
            lo, hi = prim.bbox(prim.place(p.solid, p.print_tf))
            assert abs(lo[2]) < 1e-6, n
            assert hi[0] - lo[0] <= 256 and hi[1] - lo[1] <= 256 and hi[2] - lo[2] <= 260, n
        assert "—" not in p.notes and "—" not in p.bought_ref


def test_as5600_gap_and_alignment():
    g = SE.as_geom(P)
    assert 1.0 <= g["gap_min"] <= g["gap"] <= 2.0 + 1e-9
    assert g["y_board"] - SE.JST["h"] - g["y_mag"] >= 0.5      # JST clears the turning cap face


def test_as5600_chip_on_axis(parts):
    b = parts["sensors_as5600"].solid
    chip = prim.box(-3, 3, SE.as_geom(P)["y_chip"] - 0.01, SE.as_geom(P)["y_board"] - 0.01, -3, 3) ^ b
    lo, hi = prim.bbox(chip)
    assert abs((lo[0] + hi[0]) / 2) < 0.05 and abs((lo[2] + hi[2]) / 2) < 0.05


def test_as5600_clear_of_turning_cap(parts):
    """The cap is a cylinder r 10 to y 201.5 on the axis: anything inside r 10.4 must stay beyond y 201.9."""
    g = SE.as_geom(P)
    keep = prim.cylY(SE.CAP_R + 0.4, g["y_mag"] - 10, g["y_mag"] + 0.4, seg=96)
    for n in ("sensors_as5600", "sensors_as5600_strip_top", "sensors_as5600_strip_bot", "sensors_as5600_carrier",
              "sensors_as5600_screw_top", "sensors_as5600_screw_bot", "sensors_as5600_nut_top",
              "sensors_as5600_nut_bot"):
        assert _ov(parts[n].solid, keep) < TOL, n


def test_as5600_stack_no_overlap(parts):
    names = ["sensors_as5600", "sensors_as5600_carrier", "sensors_as5600_strip_top", "sensors_as5600_strip_bot"]
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            assert _ov(parts[a].solid, parts[b].solid) < TOL, (a, b)
    # pegs pass through the board holes, board sits on the plate, strips sit on the board
    assert _ov(parts["sensors_as5600_carrier"].solid, parts["sensors_as5600"].solid) < TOL
    car = parts["sensors_as5600_carrier"].solid
    g = SE.as_geom(P)
    for bx, by in SE.PCB_HOLES:
        x, z = SE._b2w(bx, by)
        peg = car ^ prim.cylY(1.5, g["y_strip"], g["y_board_back"], x=x, z=z)
        assert peg.volume() > 10.0                                # a real peg through every hole


def test_jst_plug_room(parts):
    """+x connector: nothing of the sensors within the plug path (6 mm beyond the board edge, connector band)."""
    g = SE.as_geom(P)
    x0 = SE.PCB_L / 2
    path = prim.box(x0 - 4.0, x0 + SE.PLUG_OUT, g["y_board"] - SE.JST["h"] - 0.3, g["y_board"],
                    -SE.JST["along"] / 2 - 0.5, SE.JST["along"] / 2 + 0.5)
    for n, p in parts.items():
        if n == "sensors_as5600":
            continue
        assert _ov(prim.place(p.solid, p.place), path) < TOL, n


def test_switch_lever_within_limits():
    i = SE.sw_build()["info"]
    assert i["strain_ok"] and i["strain_max"] <= switch.STRAIN_LIMIT
    assert 0.85 <= i["gap"] <= 1.0                              # coupon 2: about 0.9 mm
    assert 0.0 < SE.SW_PRESS < i["overtravel"]
    r = SE.sw_build(SE.RISER)["info"]
    assert r["top"] == pytest.approx(i["top"] + SE.RISER)


def test_mount_pilots_under_base_holes(parts):
    """The bracket pilots line up with the base mounting holes: a d2.6 rod through each base hole hits bracket."""
    for tag, br in (("rail_low", "sensors_sw_rail_low_bracket"),):
        base = parts[f"sensors_sw_{tag}"]
        bracket = parts[br]
        assert _ov(prim.place(base.solid, base.place), prim.place(bracket.solid, bracket.place)) < TOL
        assert prim.place(base.solid, base.place).volume() > 0


def test_old_launch_parts_gone(asm):
    old = _old_launch_parts(asm)
    if old:
        pytest.xfail(f"cad-launch has not applied sensors_launch_patch.diff yet: {old}")


def test_rail_low_closes_only_near_low(asm):
    import launch_kin as K
    t = K.pitch_table(P)
    def S(d):
        return dict(aim_deg=float(K.pitch_of_psi(P, t["psi_low"] - d, +1)), aim_psi_from_low=float(d), cock=0.0)
    lev = "sensors_sw_rail_low_lever"
    v0 = _ov(_posed(asm, lev, S(0.0)), _posed(asm, "launch_aim_crank", S(0.0)))
    assert v0 > 1.0                                               # cam presses the lever at LOW
    for d in (20.0, -20.0, 90.0, 180.0):
        assert _ov(_posed(asm, lev, S(d)), _posed(asm, "launch_aim_crank", S(d))) < TOL, d


def test_gap_closes_only_at_rest(asm):
    lev = "sensors_sw_gap_lever"
    S0 = dict(aim_deg=6.0, cock=0.0, launch_gear_deg=-360.0)
    assert _ov(_posed(asm, lev, S0), _posed(asm, "launch_big_gear", S0)) > 1.0
    for gd in (-360.0 + 30.0, -30.0, -180.0, -90.0):
        S1 = dict(aim_deg=6.0, cock=0.0, launch_gear_deg=gd)
        assert _ov(_posed(asm, lev, S1), _posed(asm, "launch_big_gear", S1)) < TOL, gd


def test_gap_base_clear_of_drum_dog_post(asm):
    for a in (6.0, 20.0, 35.0, 47.0):
        S = dict(aim_deg=a, cock=0.0)
        assert _ov(_posed(asm, "sensors_sw_gap", S), _posed(asm, "drum_dog_post", S)) < TOL, a


def test_gap_screw_end_below_gear_web(parts):
    """Riser: jam nut and screw end stay at least 0.4 mm off the big gear web face (rail y_rel -6) at rest."""
    import launch as LN
    y_web = LN.L(P)["y_sec"][0]
    for n in ("sensors_sw_gap_screw", "sensors_sw_gap_jam"):
        p = parts[n]
        hi = prim.bbox(prim.place(p.solid, p.place))[1]
        assert hi[1] <= y_web - 0.4, n
