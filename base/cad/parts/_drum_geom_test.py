"""Geometry tests for the drum subsystem (PC, about a minute). Run from base/:
    python -m pytest cad/parts/_drum_geom_test.py -q
Loads only the drum module (the LAUNCH dog ring's gear link to cad-launch is switched off)."""
import numpy as np
import pytest

import cad.parts.drum as D
from cad.lib import prim
from tools.params import load_params
from tools.assembly import Assembly


@pytest.fixture(scope="module")
def asm():
    D._launch_gear = lambda: None
    return Assembly.load(load_params(), ["drum"])


def posed(asm, name, S):
    W = asm.world(S)
    return prim.place(asm.parts[name].solid, W[name])


def overlap(asm, a, b, S):
    return (posed(asm, a, S) ^ posed(asm, b, S)).volume()


def test_parts_one_shell_and_print(asm):
    for n, p in asm.parts.items():
        assert p.solid.status().name == "NoError", n
        assert len(p.solid.decompose()) == 1, n
        if p.print_tf is not None:
            m = prim.place(p.solid, p.print_tf)
            lo, hi = np.array(m.bounding_box()[:3]), np.array(m.bounding_box()[3:])
            assert abs(lo[2]) < 1e-6, n
            assert np.all(hi - lo <= np.array([256.0, 256.0, 260.0])), n


@pytest.mark.parametrize("th", [-4, 0, 20, 41, 56, 61, 62, 63, 120, 200, 301, 302, 303, 314, 330, 347, 352])
def test_moving_pairs_clear(asm, th):
    S = dict(drum_deg=float(th), wiper_deg=0.0)
    pairs = [("drum_drum_a", "drum_idler"), ("drum_drum_a", "drum_idler_gear"), ("drum_drum_a", "drum_n20_pinion"),
             ("drum_drum_b", "drum_fork"), ("drum_fork", "drum_collar"), ("drum_fork", "drum_dog_post"),
             ("drum_collar", "drum_gear_reel"), ("drum_collar", "drum_dogring_launch")]
    pairs += [(f"drum_sw_lever_{i}", "drum_drum_b") for i in range(3)]
    pairs += [(f"drum_sw_lever_{i}", "drum_switch_block") for i in range(3)]
    for a, b in pairs:
        assert overlap(asm, a, b, S) < 1e-3, (th, a, b)


@pytest.mark.parametrize("w", [0, 10, 21, 30, 45, 60])
def test_index_lever_and_dogs_with_wiper(asm, w):
    for th in (0.0, 41.0, 347.0):
        S = dict(drum_deg=th, wiper_deg=float(w))
        assert overlap(asm, "drum_index_lever", "drum_coupler", S) < 1e-3
        assert overlap(asm, "drum_collar", "drum_gear_reel", S) < 1e-3
        assert overlap(asm, "drum_collar", "drum_dogring_launch", S) < 1e-3


def test_code_levers_follow_bands(asm):
    """The posed lever nub stays within 0.7 mm of its band at sampled angles (it rides, never hovers)."""
    for th in (0.0, 6.0, 20.0, 34.0, 50.0, 62.0, 180.0, 302.0, 342.0, 350.0):
        S = dict(drum_deg=th, wiper_deg=0.0)
        for i in range(3):
            d = posed(asm, "drum_drum_b", S)
            lv = posed(asm, f"drum_sw_lever_{i}", S)
            assert (d ^ lv).volume() < 1e-3
            assert d.min_gap(lv, 2.0) < 0.7, (th, i)


def test_idler_locked_at_home(asm):
    """At LAUNCH the wrap pinion cannot turn more than 3 deg either way (locking arc)."""
    S = dict(drum_deg=0.0, wiper_deg=0.0)
    v = asm.values(S)
    d = posed(asm, "drum_drum_a", S)
    for sgn in (1, -1):
        v2 = dict(v)
        v2["drum_idler"] += sgn * 3.0
        JW = asm.joint_world(S, v2)
        idl = prim.place(asm.parts["drum_idler"].solid, JW["drum_idler"] @ asm.parts["drum_idler"].place)
        assert (d ^ idl).volume() > 1e-3
