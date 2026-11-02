"""Tests for cad.lib.springs: posts valid, hook fits the post groove, spring model, force helper."""
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..")))

import manifold3d as m3
import numpy as np
import pytest

from cad.lib import springs as S, prim, tf


def ok(s):
    assert s.status() == m3.Error.NoError
    assert s.volume() > 0
    assert prim.shells(s) == 1


def V(a, b):
    return (a ^ b).volume()


@pytest.mark.parametrize("base", ["screw", "insert", None])
def test_post_valid(base):
    ok(S.anchor_post(base=base))


def test_spring_model():
    s = S.extension_spring_model(free_len=60)
    ok(s)
    (_, _, z0), (_, _, z1) = prim.bbox(s)
    assert abs(z0 + 2.0) < 1e-6 and abs(z1 - 62.0) < 1e-6      # hook inner edges at 0 and 60
    ok(S.extension_spring_model(free_len=60, extended_len=90))
    with pytest.raises(ValueError):
        S.extension_spring_model(free_len=20)


def _post_in_hook(post, info, dz=0.0):
    # post axis along +Y, groove centre on the hook plane y = 0, groove core touching the hook's inner edge (z = 0)
    return prim.place(post, tf.T(0, 0, info["core_d"] / 2 + dz) @ tf.T(0, -info["hook_z"], 0) @ tf.Rx(-90))


@pytest.mark.parametrize("wire", [2.0, 1.6])
def test_hook_fits_post(wire):
    post = S.anchor_post(wire=wire)
    info = S.anchor_post_info(wire=wire)
    spring = S.extension_spring_model(wire=wire, free_len=60)
    assert info["post_d"] < info["hook_id"]                        # hook drops over the lip
    assert info["groove_w"] > wire                                 # wire fits the groove
    p = _post_in_hook(post, info)
    assert V(p, spring) < 1e-2                                     # loaded hook sits on the groove core
    assert V(p.translate((0, 0, -0.1)), spring) > 0                # and bears on it
    # hook centred on the post: the lip passes through the loop
    centred = prim.place(post, tf.T(0, 0, info["hook_id"] / 2) @ tf.T(0, -info["hook_z"], 0) @ tf.Rx(-90))
    assert V(centred, spring) < 1e-6
    # loaded hook cannot ride off: sliding the post along its axis by the groove width hits the lip
    assert V(p.translate((0, -info["groove_w"], 0)), spring) > 0


def test_post_stress_and_force():
    i = S.anchor_post_info(force=60)
    assert i["stress_ok"] and i["stress_mpa"] > 0
    assert S.spring_force(0.5, 10, 2) == 7.0
    assert S.spring_force(0.5, -1, 2) == 0.0
    assert np.allclose(S.spring_force(0.5, [0, 1, -1], 2), [2.0, 2.5, 0.0])
