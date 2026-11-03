"""Day-one risk coupons (tools/risk_coupon.py): every part prints and passes the static audit, the coupon assemblies
do not collide, the plates fit the bed, the test sheet exists and is clean."""
import os
import warnings

import pytest

from cad.lib import prim
from tools import audit, pack, risk_coupon as rc

EM_DASH = chr(0x2014)

with warnings.catch_warnings():
    warnings.simplefilter("ignore")
    COUPONS = rc.coupons()
PARTS = rc.all_parts(COUPONS)


@pytest.fixture(scope="module")
def written(tmp_path_factory):
    d = str(tmp_path_factory.mktemp("coupons"))
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        rep = rc.write(d, log=lambda *a: None)
    return d, rep


@pytest.mark.parametrize("p", PARTS, ids=[p.name for p in PARTS])
def test_part_static_and_watertight(p):
    r = audit.static_part(p)
    assert r["fails"] == [], r["fails"]
    assert r["shells"] == 1
    tm = prim.to_trimesh(p.solid)
    tm.merge_vertices()
    assert tm.is_watertight
    assert p.printed and p.material in ("PLA", "PPA-CF")


def test_all_coupons_present_and_named():
    assert [c["key"] for c in COUPONS] == [str(k) for k in range(1, 10)]
    names = [p.name for p in PARTS]
    assert len(names) == len(set(names))
    for c in COUPONS:
        assert c["parts"] and all(p.name.startswith(f"rc{c['key']}_") for p in c["parts"])
        for k in ("title", "do", "ok", "fail", "uses", "bought"):
            assert c[k] and EM_DASH not in c[k]


def test_assemblies_do_not_collide():
    assert rc.interferences(COUPONS) == []


def test_real_geometry_is_reused():
    """Spot checks that the coupons carry the subsystems' own solids and numbers."""
    import launch_aim as LA
    from cad.parts import drum
    P = rc.P_default()
    by = {p.name: p for p in PARTS}
    assert by["rc1_arm"].solid.volume() == pytest.approx(LA.arm_solid(P).volume(), rel=1e-6)
    assert by["rc1_sun"].solid.volume() == pytest.approx(LA.sun_solid(P).volume(), rel=1e-6)
    g, c = drum.G(P), drum.C(P)
    assert by["rc3_fork"].solid.volume() == pytest.approx(drum.fork(g, c).volume(), rel=1e-6)
    assert by["rc9_shear_pin"].solid.volume() == pytest.approx(drum.shear_pin(g).volume(), rel=1e-6)
    sheet = rc.coupon_9(P)
    assert 3.0 < sheet["t_est"] < 7.0                       # about 5 N m (D8 / D26)


def test_outputs(written):
    d, rep = written
    assert rep["errors"] == []
    assert 1 <= len(rep["plates"]) <= 2
    assert {pl["material"] for pl in rep["plates"]} == {"PLA", "PPA-CF"}
    for pl in rep["plates"]:
        assert pl["build_items"] == pl["objects"]
        assert min(pl["lo"][:2]) >= -1e-6
        assert pl["hi"][0] <= pack.BED[0] + 1e-6 and pl["hi"][1] <= pack.BED[1] + 1e-6
        assert pl["hi"][2] <= pack.BED[2]
        assert os.path.exists(os.path.join(d, pl["file"].replace(".3mf", ".png")))
    for p in PARTS:
        assert os.path.getsize(os.path.join(d, f"{p.name}.stl")) > 0
    md = open(os.path.join(d, "COUPON_TEST.md"), encoding="utf-8").read()
    assert EM_DASH not in md
    for c in COUPONS:
        assert f"### {c['key']}. {c['title']}" in md
    pdf = os.path.join(d, "COUPON_TEST.pdf")
    assert os.path.getsize(pdf) > 1000 and rep["pdf_pages"] == 1
    assert EM_DASH.encode("utf-8") not in open(pdf, "rb").read()


def test_sheet_numbers_are_measurable():
    by = {c["key"]: c for c in COUPONS}
    assert " g holds, " in by["4"]["ok"] and "luggage" not in by["4"]["ok"] + by["4"]["bought"]
    lo, hi = by["9"]["kg_range"]
    assert by["9"]["t_range"] == (3.5, 6.0)
    assert lo == pytest.approx(3.5 / (rc.LEVER_R / 1000) / 9.81) and f"{lo:.1f} and {hi:.1f} kg" in by["9"]["ok"]
    assert "calipers" in by["2"]["ok"] and "60 mm" in by["7"]["ok"]
    s = rc.settings_text()
    from tools.pack import PROFILE
    assert f"{PROFILE['PPA-CF']['walls']} walls" in s and "geometry only" in s


def test_sheet_settings_and_fit_coupon(written, tmp_path):
    d, rep = written
    md = open(os.path.join(d, "COUPON_TEST.md"), encoding="utf-8").read()
    assert "Bambu Studio" in md and "setup_mac.sh" in md and "base/pack/fit_coupon" in md
    assert all(pl["material"] in pl["file"] for pl in rep["plates"])
    fit = str(tmp_path / "fit_coupon")
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        r = rc.write_fit_coupon(fit, log=lambda *a: None)
    assert r["errors"] == []
    assert os.path.exists(os.path.join(fit, "COUPON.md")) and r["plates"]
    assert not os.path.exists(fit + ".tmp")
