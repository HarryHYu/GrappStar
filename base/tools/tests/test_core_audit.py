"""Tests of the core API, assembly kinematics and the audit on small synthetic subsystems."""
import types

import numpy as np
import pytest

from cad.lib import Part, Joint, Allow, Clearance, prim, tf
from tools.assembly import Assembly
from tools import audit
from tools.params import load_params
from tools.timeline import timeline, phases, duration


def mod(parts, joints, kin, **extra):
    m = types.SimpleNamespace(parts=lambda P: parts, joints=lambda P: joints, kin=kin)
    for k, v in extra.items():
        setattr(m, k, v)
    return m


def flat(z0=0):
    return tf.I4


def test_names_and_materials():
    with pytest.raises(ValueError):
        Part("Bad Name", prim.cyl(1, 1))
    with pytest.raises(ValueError):
        Part("ok", prim.cyl(1, 1), material="wood")
    with pytest.raises(ValueError):
        Joint("j", kind="hinge")
    p = Part("ok", prim.cbox(10, 10, 10), material="PLA", infill=1.0)
    assert abs(p.mass_g() - 1.24) < 1e-6
    assert Part("s", prim.cbox(10, 10, 10), material="steel").mass_g() == pytest.approx(7.85)
    assert Part("b", prim.cbox(1, 1, 1), material="bought").mass_g() is None


def test_joint_chain_and_gear():
    js = [Joint("a", None, "rev", (10, 0, 0), (0, 0, 1)),
          Joint("b", "a", "slide", (0, 5, 0), (1, 0, 0)),
          Joint("c", None, "rev", (0, 0, 0), (0, 0, 1), gear=("a", -2.0, 0.0))]
    ps = [Part("pb", prim.cbox(1, 1, 1), joint="b", print_tf=tf.I4)]
    asm = Assembly(None, {"x": mod(ps, js, lambda P, S: {"a": S["q"], "b": 3.0})})
    assert not asm.errors
    v = asm.values({"q": 90.0})
    assert v["c"] == -180.0
    W = asm.world({"q": 90.0})
    # b origin (0,5,0) in a's frame rotated 90 about z -> (-5,0,0); slide 3 along a-frame x -> (0,3,0); plus (10,0,0)
    assert np.allclose(W["pb"][:3, 3], [10 - 5, 3, 0], atol=1e-9)


def test_errors_reported():
    js = [Joint("a", "nope", "rev")]
    ps = [Part("p", prim.cbox(1, 1, 1), print_tf=tf.I4), Part("p", prim.cbox(1, 1, 1), joint="zz", print_tf=tf.I4)]
    asm = Assembly(None, {"x": mod(ps, js, lambda P, S: {})})
    txt = " ".join(asm.errors)
    assert "unknown parent" in txt and "unknown joint zz" in txt and "duplicate part p" in txt


def two_blocks(gap, allow=None, clear=None):
    base = Part("base_plate", prim.cbox(40, 40, 5), print_tf=tf.I4)
    blk = Part("blk", prim.cbox(10, 10, 10), joint="lift", print_tf=tf.I4)
    js = [Joint("lift", None, "slide", (0, 0, 5 + gap), (0, 0, 1))]
    extra = {}
    if allow:
        extra["allowed"] = lambda P: allow
    if clear:
        extra["clearances"] = lambda P: clear
    return Assembly(load_params(), {"t": mod([base, blk], js, lambda P, S: {"lift": S.get("z", 0.0)}, **extra)})


def test_audit_clean_touching():
    asm = two_blocks(0.0)
    rep = audit.run(asm, [{"z": 0.0}, {"z": 2.0}], jobs=1, log=lambda *a: None)
    assert rep["clean"], audit.summary(rep)


def test_audit_collision_and_allow():
    asm = two_blocks(-1.0)
    rep = audit.run(asm, [{"z": 0.0}], jobs=1, log=lambda *a: None)
    assert not rep["clean"] and rep["collisions"][0]["max_vol_mm3"] == pytest.approx(100.0, rel=1e-3)
    asm = two_blocks(-1.0, allow=[Allow("blk", "base_*", "press fit", max_vol=150)])
    rep = audit.run(asm, [{"z": 0.0}], jobs=1, log=lambda *a: None)
    assert rep["clean"] and rep["allowed_contacts"]
    asm = two_blocks(-1.0, allow=[Allow("blk", "base_*", "press fit", max_vol=50)])
    assert not audit.run(asm, [{"z": 0.0}], jobs=1, log=lambda *a: None)["clean"]


def test_audit_collision_in_later_state_found():
    asm = two_blocks(0.0)
    rep = audit.run(asm, [{"z": 0.0}, {"z": -2.0}], jobs=1, log=lambda *a: None)
    assert not rep["clean"] and rep["collisions"][0]["worst_state"] == 1


def test_audit_floating_and_clearance():
    asm = two_blocks(3.0, clear=[Clearance("blk", "base_plate", 2.5, "test")])
    rep = audit.run(asm, [{"z": 0.0}, {"z": -0.4}], jobs=1, log=lambda *a: None)
    assert {f["part"] for f in rep["floating"]} == {"base_plate", "blk"}
    c = rep["clearances"][0]
    assert c["gap"] == pytest.approx(2.6, abs=1e-3) and c["ok"]


def test_static_print_checks():
    tall = Part("tall", prim.cbox(10, 10, 300), print_tf=tf.I4)
    up = Part("up", prim.cbox(10, 10, 10), print_tf=tf.T(0, 0, 1))
    two = Part("two", prim.cbox(5, 5, 5) + prim.cbox(5, 5, 5, x=20), print_tf=tf.I4)
    ball = Part("ball", prim.Manifold.sphere(10, 64), print_tf=tf.T(0, 0, 10))
    r = {p.name: audit.static_part(p) for p in (tall, up, two, ball)}
    assert any("bed" in f for f in r["tall"]["fails"])
    assert any("min z" in f for f in r["up"]["fails"])
    assert any("shells" in f for f in r["two"]["fails"])
    assert any("flat face" in f for f in r["ball"]["fails"])
    ok = audit.static_part(Part("ok", prim.cbox(10, 10, 10), print_tf=tf.I4))
    assert not ok["fails"] and ok["overhang_mm2"] == 0


def test_overhang_measured():
    t = Part("t", prim.cbox(30, 10, 5, z=10) + prim.cbox(10, 10, 10), print_tf=tf.I4)
    r = audit.static_part(t)
    assert r["overhang_mm2"] == pytest.approx(200.0, abs=1)


def test_timeline():
    P = load_params()
    st = timeline(P, 0.5)
    assert abs(st[-1]["t"] - duration(P)) < 0.5 + 1e-9
    names = [p["name"] for p in phases(P)]
    assert {S["phase"] for S in st} <= set(names)
    assert max(S["line_out_m"] for S in st) > 0 and st[-1]["drum_deg"] == P.DRUM_POS["LAUNCH"]
    assert any(S["fired"] for S in st)


def test_subsystem_names_skip_helpers(tmp_path, monkeypatch):
    import cad.parts as pk
    from tools.assembly import subsystem_names
    for n in ("alpha.py", "alpha_kin.py", "NOTES.md", "data.json"):
        (tmp_path / n).write_text("")
    monkeypatch.setattr(pk, "__path__", [str(tmp_path)])
    assert subsystem_names() == ["alpha"]


# ------------------------------------------------------------------ review round 1 regressions
Q = dict(jobs=1, log=lambda *a: None)


def test_gear_follower_keeps_offset_without_master():
    js = [Joint("m", None, "rev"), Joint("f", None, "rev", gear=("m", 2.0, 9.0))]
    asm = Assembly(None, {"x": mod([], js, lambda P, S: {})})
    assert asm.values({}) == {"m": 0.0, "f": 9.0}


def test_gear_cycle_and_duplicate_kin_reported():
    js = [Joint("a", None, "rev", gear=("b", 1.0)), Joint("b", None, "rev", gear=("a", 1.0))]
    asm = Assembly(None, {"x": mod([], js, lambda P, S: {})})
    assert any("gear cycle" in e for e in asm.errors)
    js2 = [Joint("h", None, "rev")]
    asm = Assembly(None, {"x": mod([], js2, lambda P, S: {"h": 1}), "y": mod([], [], lambda P, S: {"h": 2})})
    with pytest.raises(ValueError):
        asm.values({})


def test_parent_cycle_does_not_crash():
    js = [Joint("a", "b", "rev"), Joint("b", "a", "rev")]
    ps = [Part("p", prim.cbox(1, 1, 1), joint="a", print_tf=tf.I4)]
    asm = Assembly(load_params(), {"x": mod(ps, js, lambda P, S: {})})
    rep = audit.run(asm, [{}], **Q)
    assert not rep["clean"] and any("cycle" in e for e in rep["errors"])


def test_most_restrictive_allow_wins_in_any_order():
    for allow in ([Allow("blk", "*", "broad"), Allow("blk", "base_plate", "tight", max_vol=1.0)],
                  [Allow("blk", "base_plate", "tight", max_vol=1.0), Allow("blk", "*", "broad")]):
        assert not audit.run(two_blocks(-1.0, allow=allow), [{"z": 0.0}], **Q)["clean"]


def test_clearance_fails_when_parts_intersect_even_if_allowed():
    asm = two_blocks(-1.0, allow=[Allow("blk", "base_plate", "x")], clear=[Clearance("blk", "base_plate", 0.0)])
    rep = audit.run(asm, [{"z": 0.0}], **Q)
    assert not rep["clean"] and not rep["clearances"][0]["ok"]


def test_floating_cluster_and_grounded_typo():
    base = Part("frame_base", prim.cbox(40, 40, 5), print_tf=tf.I4)
    a = Part("reel_a", prim.cbox(10, 10, 10, z=500), print_tf=tf.T(0, 0, -500))
    b = Part("reel_b", prim.cbox(10, 10, 10, z=510), print_tf=tf.T(0, 0, -510))
    for pats, expect_err in ((["frame_base"], False), (["frame_bse"], True)):
        m = mod([base, a, b], [], lambda P, S: {}, grounded=lambda P, pats=pats: pats)
        rep = audit.run(Assembly(load_params(), {"t": m}), [{}], **Q)
        assert not rep["clean"]
        assert any("matches no part" in e for e in rep["errors"]) == expect_err
        if not expect_err:
            assert {f["part"] for f in rep["floating"]} >= {"reel_a", "reel_b"}
    rep = audit.run(Assembly(load_params(), {"t": mod([base, a, b], [], lambda P, S: {})}), [{}], **Q)
    assert any("grounded" in w for w in rep["warns"])


def test_rigid_transform_required():
    mirror = np.diag([1.0, 1.0, -1.0, 1.0])
    ps = [Part("p", prim.cbox(1, 1, 1), print_tf=tf.T(0, 0, 1) @ mirror), Part("q", prim.cbox(1, 1, 1), place=np.diag([2.0, 2, 2, 1]))]
    asm = Assembly(None, {"x": mod(ps, [], lambda P, S: {})})
    assert sum("rigid" in e for e in asm.errors) == 2


def test_refine_limits_joint_steps():
    asm = two_blocks(0.0)
    js = [Joint("r", None, "rev")]
    asm2 = Assembly(None, {"x": mod([], js, lambda P, S: {"r": S["a"]})})
    st = audit.refine(asm2, [{"a": 0.0, "phase": "p"}, {"a": 10.0, "phase": "q"}])
    assert len(st) == 11 and st[5]["a"] == pytest.approx(5.0) and st[5]["phase"] == "p"
    st = audit.refine(asm, [{"z": 0.0}, {"z": 2.0}])
    assert len(st) == 5


def test_flat_face_and_overhang_warning():
    plate = Part("pl", prim.cbox(4, 5, 2) + prim.cbox(250, 250, 3, z=2), print_tf=tf.I4)
    r = audit.static_part(plate)
    assert any("flat face" in f for f in r["fails"]) and any("overhang" in w for w in r["warns"])


def test_params_real_import_error_not_hidden(tmp_path, monkeypatch):
    import sys as _s
    from tools import params as pm
    (tmp_path / "spec").mkdir()
    (tmp_path / "spec" / "__init__.py").write_text("")
    (tmp_path / "spec" / "params.py").write_text("import no_such_module_xyz\n")
    monkeypatch.syspath_prepend(str(tmp_path))
    for k in [k for k in _s.modules if k == "spec" or k.startswith("spec.")]:
        monkeypatch.delitem(_s.modules, k)
    with pytest.raises(ModuleNotFoundError):
        pm.load_params()
    for k in [k for k in _s.modules if k == "spec" or k.startswith("spec.")]:
        monkeypatch.delitem(_s.modules, k)


def test_custom_timeline_targets_and_unknown_names():
    import types as _t
    from tools.timeline import state_at
    P = _t.SimpleNamespace(TIMELINE=[("rest", 1.0, "", {}), ("swing", 2.0, "", {"drum_deg": 90.0, "aim_deg": 40.0})])
    assert state_at(P, 0.5)["drum_deg"] == 0.0
    assert state_at(P, 3.0)["drum_deg"] == pytest.approx(90.0) and state_at(P, 2.0)["phase"] == "swing"
    P2 = _t.SimpleNamespace(TIMELINE=[("aim_rail", 1.0, "")])
    with pytest.raises(ValueError):
        state_at(P2, 0.0)
    P3 = _t.SimpleNamespace(DRUM={"angles": {"LAUNCH": 0, "NEUTRAL": 40, "REEL": 80}})
    assert state_at(P3, 0.0)["drum_deg"] == 0


def test_line_path_joined_in_order():
    a = mod([], [], lambda P, S: {}, line_path=lambda P, S: [(0, 0, 0), (1, 0, 0)])
    b = mod([], [], lambda P, S: {}, line_path=lambda P, S: [(1, 0, 0), (5, 0, S["k"])])
    asm = Assembly(None, {"launch": b, "reel": a})
    assert asm.line({"k": 2.0}) == [[0, 0, 0], [1, 0, 0], [5, 0, 2.0]]
