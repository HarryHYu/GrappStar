"""Fit coupon (tools/coupon.py) and subsystem template (cad/parts/_template.py)."""
import importlib
import os

import pytest

from cad.lib import Part, prim
from tools import audit, coupon
from tools.assembly import Assembly, subsystem_names
from tools.params import load_params
from tools.timeline import timeline

EM_DASH, EN_DASH = chr(0x2014), chr(0x2013)
PARTS = coupon.coupon_parts()


@pytest.mark.parametrize("p", PARTS, ids=[p.name for p in PARTS])
def test_coupon_part_static_and_watertight(p):
    r = audit.static_part(p)
    assert r["fails"] == [], r["fails"]
    assert r["shells"] == 1
    tm = prim.to_trimesh(p.solid)
    tm.merge_vertices()
    assert tm.is_watertight


def test_coupon_set_complete_and_small():
    names = {p.name for p in PARTS}
    for prefix in ("coupon_a", "coupon_b", "coupon_c", "coupon_d", "coupon_e", "coupon_f", "coupon_g", "coupon_h"):
        assert any(n.startswith(prefix) for n in names), prefix
    assert sum(n.startswith("coupon_a") for n in names) == 3
    assert sum(n.endswith("key_bore") for n in names) == 3
    assert "coupon_b_key_shaft" in names
    for p in PARTS:
        assert p.material == "PLA" and p.printed
        lo, hi = prim.bbox(prim.place(p.solid, p.print_tf))
        dx, dy, dz = (hi[i] - lo[i] for i in range(3))
        assert abs(lo[2]) < 1e-6
        if "shaft" not in p.name:              # the keyed shaft stub prints upright (16 tall)
            assert max(dx, dy) <= 60.5 and min(dx, dy) <= 40.5 and dz <= 10.0, (p.name, dx, dy, dz)


def test_coupon_variants_follow_fits():
    from cad.lib import fits
    v = coupon.variants()
    assert v["rib"][1] == pytest.approx(fits.BRG_RIB_INTERF)
    assert v["rib"][0] == pytest.approx(fits.BRG_RIB_INTERF - 0.05)
    assert v["bore"][1] == pytest.approx(fits.SHAFT_IN_BORE)
    assert v["insert"] == [3.8, 4.0, 4.2] and v["nut"] == [5.6, 5.8, 6.0]
    assert v["holes"] == [2.5, 2.6, 2.7, 3.2, 3.4]


def test_stroke_font_labels():
    assert coupon.fmt(0.15) == ".15" and coupon.fmt(-0.05) == "-.05" and coupon.fmt(4.0, 1) == "4.0"
    cs = coupon.text_cs("A .15")
    assert cs.area() > 0
    with pytest.raises(ValueError):
        coupon.text_cs("q")


def test_coupon_write(tmp_path):
    rep = coupon.write(str(tmp_path), log=lambda *a: None)
    d = tmp_path / "coupon"
    assert rep["errors"] == []
    for p in PARTS:
        assert (d / f"{p.name}.stl").exists()
    assert rep["plates"] and all((d / pl["file"]).exists() for pl in rep["plates"])
    # every copy is a build item of its own (counted in 3D/3dmodel.model), under a unique object name
    from tools import pack
    names = [n for pl in rep["plates"] for n in pl["names"]]
    assert len(names) == len(set(names)) == sum(p.qty for p in PARTS)
    assert "coupon_f_c_clip__c1" in names and "coupon_f_c_clip__c2" in names
    for pl in rep["plates"]:
        assert pack.count_3mf_items(str(d / pl["file"])) == pl["objects"] == pl["build_items"]
    md = (d / "COUPON.md").read_text()
    assert "BRG_RIB_INTERF" in md and "M3_NUT_AF" in md and "GEAR_BACKLASH" in md
    for f in [d / "COUPON.md", os.path.abspath(coupon.__file__)]:
        t = open(f).read()
        assert EM_DASH not in t and EN_DASH not in t


# ---------------------------------------------------------------------------------------------- template
TPL = importlib.import_module("cad.parts._template")


def test_template_not_a_subsystem():
    assert "_template" not in subsystem_names()


def test_template_functions_run():
    P = load_params()
    ps = TPL.parts(P)
    js = TPL.joints(P)
    assert ps and js and all(isinstance(p, Part) for p in ps)
    assert all(p.name.startswith(TPL.PREFIX + "_") for p in ps)
    assert all(j.name.startswith(TPL.PREFIX + "_") for j in js)
    v = TPL.kin(P, {})
    assert set(v) <= {j.name for j in js}
    assert TPL.kin(P, {f"{TPL.PREFIX}_deg": 30.0})[f"{TPL.PREFIX}_arm"] == 30.0
    for fn in ("allowed", "clearances", "audit_states", "grounded", "steps"):
        assert getattr(TPL, fn)(P)


def test_template_assembly_audits_clean():
    P = load_params()
    asm = Assembly(P, {TPL.PREFIX: TPL})
    assert asm.errors == []
    states = audit.refine(asm, timeline(P, 0.1)) + asm.extra_states
    rep = audit.run(asm, states, jobs=1, log=lambda *a: None)
    assert rep["clean"], audit.summary(rep)
    assert rep["warns"] == [], rep["warns"]


def test_coupon_object_names_do_not_collide(tmp_path):
    """A part named like another part's copy (x qty 2 next to x_c1) keeps all its build items."""
    import trimesh
    from tools import pack
    items = [dict(name="x", material="PLA", dims=[10.0, 10.0, 5.0], qty=2),
             dict(name="x_c1", material="PLA", dims=[12.0, 12.0, 5.0], qty=1)]
    plates, errs = pack.pack_plates(items, pack.BED, 5.0, 5.0)
    assert errs == [] and len(plates) == 1
    m = trimesh.creation.box([10, 10, 5])
    m.apply_translation(-m.bounds[0])
    names = pack.object_names(plates[0]["items"], {"x": 2, "x_c1": 1})
    fn = str(tmp_path / "p.3mf")
    pack.write_3mf(fn, [(n, pack.place_on_plate(m, it)) for n, it in zip(names, plates[0]["items"])])
    assert pack.count_3mf_items(fn) == 3 and len(set(names)) == 3
