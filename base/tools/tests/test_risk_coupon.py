"""Day-one risk coupons (tools/risk_coupon.py): every part prints and passes the static audit, the coupon assemblies
do not collide, the plates fit the bed, the test sheet exists and is clean.

Nothing here calls subsystem code at import time: the coupons are built once in a fixture, and a coupon whose
subsystem source changed (risk_coupon skips it) is skipped here with the reason, never a collection error."""
import os
import warnings

import pytest

from cad.lib import prim
from tools import audit, pack, risk_coupon as rc

EM_DASH = chr(0x2014)
KEYS = sorted(rc.TITLES)


@pytest.fixture(scope="module")
def built():
    skipped = []
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        cs = rc.coupons(skipped=skipped, log=lambda *a: None)
    return {c["key"]: c for c in cs}, {d["key"]: d for d in skipped}, cs


def _coupon(built, key):
    by, skipped, _ = built
    if key in skipped:
        pytest.skip(f"coupon {key} skipped by risk_coupon: {skipped[key]['reason']}")
    return by[key]


def _src(modname, attr):
    """A subsystem function, or skip the test when it was renamed or removed."""
    import importlib
    try:
        mod = importlib.import_module(modname)
    except ImportError as e:
        pytest.skip(f"{modname} not importable: {e}")
    if not hasattr(mod, attr):
        pytest.skip(f"{modname}.{attr} no longer exists")
    return getattr(mod, attr)


@pytest.fixture(scope="module")
def written(tmp_path_factory, built):
    d = str(tmp_path_factory.mktemp("coupons"))
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        rep = rc.write(d, log=lambda *a: None)
    return d, rep


@pytest.mark.parametrize("key", KEYS)
def test_parts_static_and_watertight(built, key):
    c = _coupon(built, key)
    assert c["parts"] and c["title"] == rc.TITLES[key]
    for p in c["parts"]:
        assert p.name.startswith(f"rc{key}_")
        assert p.printed and p.material in ("PLA", "PPA-CF")
        r = audit.static_part(p)
        assert r["fails"] == [], (p.name, r["fails"])
        assert r["shells"] == 1, p.name
        tm = prim.to_trimesh(p.solid)
        tm.merge_vertices()
        assert tm.is_watertight, p.name
    for k in ("do", "ok", "fail", "uses", "bought"):
        assert c[k] and EM_DASH not in c[k]


def test_names_unique(built):
    names = [p.name for p in rc.all_parts(built[2])]
    assert len(names) == len(set(names))


def test_no_coupon_skipped(built):
    """Fails loudly (not a collection error) when a subsystem rename made risk_coupon leave a coupon out."""
    assert built[1] == {}, built[1]


def test_assemblies_do_not_collide(built):
    assert rc.interferences(built[2]) == []


def test_skip_guard(monkeypatch):
    """A missing subsystem function skips only that coupon, with a message."""
    from cad.parts import drum
    monkeypatch.delattr(drum, "dog_post", raising=False)
    skipped, msgs = [], []
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        cs = rc.coupons(skipped=skipped, log=msgs.append)
    assert [d["key"] for d in skipped] == ["3"] and "dog_post" in skipped[0]["reason"]
    assert "3" not in {c["key"] for c in cs} and len(cs) == len(KEYS) - 1
    assert any("SKIPPED" in m for m in msgs)
    assert "NOT IN THIS SET" in rc.skip_note(skipped)


def test_real_geometry_is_reused(built):
    P = rc.P_default()
    by = {p.name: p for p in rc.all_parts(built[2])}
    if "1" not in built[1]:
        assert by["rc1_arm"].solid.volume() == pytest.approx(_src("launch_aim", "arm_solid")(P).volume(), rel=1e-6)
        assert by["rc1_sun"].solid.volume() == pytest.approx(_src("launch_aim", "sun_solid")(P).volume(), rel=1e-6)
    if "3" not in built[1]:
        G, C = _src("cad.parts.drum", "G"), _src("cad.parts.drum", "C")
        g, c = G(P), C(P)
        assert by["rc3_fork"].solid.volume() == pytest.approx(_src("cad.parts.drum", "fork")(g, c).volume(), rel=1e-6)
        assert by["rc3_dog_post"].solid.volume() == pytest.approx(
            _src("cad.parts.drum", "dog_post")(g, c).volume(), rel=1e-6)
    c9 = _coupon(built, "9")
    g = _src("cad.parts.drum", "G")(P)
    assert by["rc9_shear_pin"].solid.volume() == pytest.approx(_src("cad.parts.drum", "shear_pin")(g).volume(),
                                                               rel=1e-6)
    assert 3.0 < c9["t_est"] < 7.0                       # about 5 N m (D8 / D26)


def test_sheet_numbers_are_measurable(built):
    c4, c9 = _coupon(built, "4"), _coupon(built, "9")
    assert " g holds, " in c4["ok"] and "luggage" not in c4["ok"] + c4["bought"]
    lo, hi = c9["kg_range"]
    assert c9["t_range"] == (3.5, 6.0)
    assert lo == pytest.approx(3.5 / (rc.LEVER_R / 1000) / 9.81) and f"{lo:.1f} and {hi:.1f} kg" in c9["ok"]
    assert "calipers" in _coupon(built, "2")["ok"] and "60 mm" in _coupon(built, "7")["ok"]
    s = rc.settings_text()
    assert f"{pack.PROFILE['PPA-CF']['walls']} walls" in s and "geometry only" in s


def test_outputs(written, built):
    d, rep = written
    assert rep["errors"] == []
    assert 1 <= len(rep["plates"]) <= 2
    assert {pl["material"] for pl in rep["plates"]} == {"PLA", "PPA-CF"}
    for pl in rep["plates"]:
        assert pl["material"] in pl["file"]
        assert pl["build_items"] == pl["objects"]
        assert min(pl["lo"][:2]) >= -1e-6
        assert pl["hi"][0] <= pack.BED[0] + 1e-6 and pl["hi"][1] <= pack.BED[1] + 1e-6
        assert pl["hi"][2] <= pack.BED[2]
        assert os.path.exists(os.path.join(d, pl["file"].replace(".3mf", ".png")))
    for p in rc.all_parts(built[2]):
        assert os.path.getsize(os.path.join(d, f"{p.name}.stl")) > 0
    md = open(os.path.join(d, "COUPON_TEST.md"), encoding="utf-8").read()
    assert EM_DASH not in md
    assert "Bambu Studio" in md and "setup_mac.sh" in md and "base/pack/fit_coupon" in md
    for c in built[2]:
        assert f"### {c['key']}. {c['title']}" in md
    pdf = os.path.join(d, "COUPON_TEST.pdf")
    assert os.path.getsize(pdf) > 1000 and rep["pdf_pages"] == 1
    assert EM_DASH.encode("utf-8") not in open(pdf, "rb").read()


def test_fit_coupon_written(tmp_path):
    fit = str(tmp_path / "fit_coupon")
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        r = rc.write_fit_coupon(fit, log=lambda *a: None)
    assert r["errors"] == []
    assert os.path.exists(os.path.join(fit, "COUPON.md")) and r["plates"]
    assert not os.path.exists(fit + ".tmp")


def test_shear_pin_prints_lying(built):
    """D44: the shear pin prints lying down (axis horizontal on the bed), like drum prints it."""
    c = _coupon(built, "9")
    p = {q.name: q for q in c["parts"]}["rc9_shear_pin"]
    lo, hi = prim.bbox(prim.place(p.solid, p.print_tf))
    dx, dy, dz = (hi[i] - lo[i] for i in range(3))
    assert dz < 6.0 and max(dx, dy) > 10.0, (dx, dy, dz)


def test_splitter_has_printed_drag_and_clip(built):
    c = _coupon(built, "1")
    names = {p.name for p in c["parts"]}
    assert {"rc1_drag_pad", "rc1_drag_shim", "rc1_idler_clip"} <= names
    assert "O-ring" not in c["bought"] + c["do"] and "e-clip" not in c["bought"] + c["do"]
