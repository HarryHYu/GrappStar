"""Cross-owner gear mesh check (tools.meshcheck via tools.audit)."""
import types

import pytest

from cad.lib import Part, Joint, Mesh, gears, tf
from tools.assembly import Assembly
from tools import audit, meshcheck
from tools.params import load_params


def mod(parts, joints, kin, **extra):
    m = types.SimpleNamespace(parts=lambda P: parts, joints=lambda P: joints, kin=kin)
    for k, v in extra.items():
        setattr(m, k, v)
    return m


def pair(cd, ratio=None, z0b=0.0, tilt=0.0, kind="spur"):
    g1 = gears.spur_gear(2, 20, 8)
    g2 = gears.spur_gear(2, 30, 8)
    js = [Joint("a_g", None, "rev", (0, 0, 0)), Joint("b_g", None, "rev", (cd, 0, z0b), base=tf.Rx(tilt),
                                                       gear=("a_g", ratio if ratio is not None else -20 / 30))]
    A = mod([Part("a_gear", g1, "PPA-CF", joint="a_g", print_tf=tf.I4)], [js[0]], lambda P, S: {"a_g": S.get("q", 0.0)})
    B = mod([Part("b_gear", g2, "PPA-CF", joint="b_g", print_tf=tf.I4)], [js[1]], lambda P, S: {},
            meshes=lambda P: [Mesh("a_gear", "b_gear", 2, 20, 30)])
    return Assembly(load_params(), {"a": A, "b": B})


CD = gears.center_distance(2, 20, 30)


def test_good_pair_passes():
    r = meshcheck.run(pair(CD), [{"q": 0.0}, {"q": 30.0}])
    assert not r["fails"], r["fails"]


@pytest.mark.parametrize("kw,word", [(dict(cd=CD + 60), "centre distance"), (dict(cd=CD, z0b=7.0), "face overlap"),
                                     (dict(cd=CD, tilt=3.0), "parallel"), (dict(cd=CD, ratio=-0.5), "ratio")])
def test_bad_pairs_fail(kw, word):
    r = meshcheck.run(pair(**kw), [{"q": 0.0}])
    assert any(word in f for f in r["fails"]), r["fails"]


def test_audit_not_clean_on_far_gears():
    rep = audit.run(pair(CD + 60), [{"q": 0.0}], jobs=1, log=lambda *a: None)
    assert not rep["clean"] and rep["mesh_fails"]


def test_geared_joint_without_mesh_warns():
    asm = pair(CD)
    asm.meshes = []
    r = meshcheck.run(asm, [{}])
    assert r["warns"] and not r["fails"]


def test_mesh_with_unloaded_part_is_skipped_not_failed():
    asm = pair(CD)
    asm.meshes = [Mesh("a_gear", "reel_bus", 2, 20, 30)]
    r = meshcheck.run(asm, [{}])
    assert not r["fails"] and r["meshes"][0]["skipped"] and any("not loaded" in w for w in r["warns"])
