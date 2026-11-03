"""Output tools on the demo: pack, step_export, bom, guide (one shared export in a temp dir)."""
import csv
import glob
import os
import re
import shutil
import subprocess

import numpy as np
import pytest
import trimesh

from tools import audit, bom, guide, pack, step_export
from tools.assembly import Assembly
from tools.params import load_params
from tools.timeline import timeline

BASE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
STEP_PARTS = ["demo_plate", "demo_shaft_1", "demo_shaft_2", "demo_brg_1"]     # small ones: STEP I/O is slow


@pytest.fixture(scope="module")
def out(tmp_path_factory):
    d = str(tmp_path_factory.mktemp("demo_out"))
    P = load_params()
    asm = Assembly.load(P, ["demo"])
    asm.export(d)
    rep = audit.run(asm, timeline(P, 0.5)[:5] + asm.extra_states, jobs=2, log=lambda *a: None)
    import json
    with open(os.path.join(d, "audit.json"), "w") as fh:
        json.dump(rep, fh, default=str)
    pk = os.path.join(d, "pack")
    pack.run(d, pk, "demo", log=lambda *a: None)
    step_export.run(d, "demo", part_names=STEP_PARTS, jobs=2, log=lambda *a: None)
    bom.run(d, "demo", log=lambda *a: None)
    guide.run(d, "demo", "GrappStar demo build guide", pk, log=lambda *a: None)
    return d


def printed_parts():
    asm = Assembly.load(load_params(), ["demo"])
    return {n: p for n, p in asm.parts.items() if p.printed}


def test_stl_print_orientation(out):
    pp = printed_parts()
    files = sorted(glob.glob(os.path.join(out, "pack", "stl", "*.stl")))
    rep = pack.load_json(os.path.join(out, "pack", "print_report.json"))
    assert sorted(os.path.basename(f)[:-4] for f in files) == sorted(pp)
    for f in files:
        m = trimesh.load(f)
        name = os.path.basename(f)[:-4]
        src = trimesh.Trimesh(*map(np.asarray, (pp[name].solid.to_mesh().vert_properties[:, :3],
                                                pp[name].solid.to_mesh().tri_verts)))
        # the STL is as watertight as the model itself (a model touching itself along an edge is reported instead)
        assert m.is_watertight == src.is_watertight, f
        if not m.is_watertight:
            assert any(w.startswith(name + ":") for w in rep["warnings"]), name
        assert abs(m.bounds[0][2]) < 1e-4, (f, m.bounds[0][2])
        assert m.volume == pytest.approx(pp[os.path.basename(f)[:-4]].solid.volume(), rel=1e-3)


def test_plates(out):
    pp = printed_parts()
    plates = sorted(glob.glob(os.path.join(out, "pack", "plates", "*.3mf")))
    assert plates
    total = 0
    mats = set()
    for f in plates:
        mat = re.match(r"plate_\d+_(.+)\.3mf", os.path.basename(f)).group(1)
        mats.add(mat)
        n = pack.count_3mf_objects(f)
        sc = trimesh.load(f, file_type="3mf")
        assert len(sc.geometry) == n
        lo, hi = sc.bounds
        assert lo[0] >= 0 and lo[1] >= 0 and abs(lo[2]) < 1e-4 and hi[0] <= 256 and hi[1] <= 256 and hi[2] <= 260
        assert os.path.exists(f[:-4] + ".png")
        total += n
    assert total == sum(p.qty for p in pp.values())
    assert mats == {p.material for p in pp.values()}
    rep = pack.load_json(os.path.join(out, "pack", "print_report.json"))
    assert not rep["errors"]
    for fn in ("PRINT_SHEET.md", "PRINT_SHEET.pdf"):
        assert os.path.getsize(os.path.join(out, "pack", fn)) > 500


def test_plate_too_big_is_an_error():
    items = [dict(name="huge", material="PLA", dims=[300.0, 20.0, 10.0], qty=1),
             dict(name="ok", material="PLA", dims=[20.0, 20.0, 10.0], qty=3)]
    plates, errors = pack.pack_plates(items, (256, 256, 260), 5.0, 5.0)
    assert len(errors) == 1 and "huge" in errors[0]
    assert sum(len(p["items"]) for p in plates) == 3


def test_step_reimport(out):
    from build123d import import_step
    rep = pack.load_json(os.path.join(out, "step", "step_report.json"))
    assert sorted(r["part"] for r in rep["parts"]) == sorted(STEP_PARTS)
    for r in rep["parts"]:
        assert r["ok"], r
    assert rep["assembly"]["ok"] and rep["assembly"]["n_solids"] == len(STEP_PARTS)
    shp = import_step(os.path.join(out, "step", "demo_plate.step"))
    assert shp.is_valid and len(shp.solids()) == 1


def test_bom(out):
    with open(os.path.join(out, "BOM.csv")) as fh:
        rows = list(csv.DictReader(fh))
    b608 = [r for r in rows if r["section"] == "bought" and r["name_or_ref"].startswith("608")]
    assert len(b608) == 1 and int(b608[0]["qty"]) == 2
    assert len([r for r in rows if r["section"] == "printed"]) == len(printed_parts())
    md = open(os.path.join(out, "BOM.md")).read()
    assert "608ZZ" in md and "\u2014" not in md
    assert bom.parse_ref("4 x M3x10 SHCS; 608ZZ") == [(4, "M3x10 SHCS"), (1, "608ZZ")]
    assert bom.category("M3x10 SHCS ISO 4762") == "fasteners" and bom.category("608ZZ 8x22x7") == "bearings"


def test_guide(out):
    fn = os.path.join(out, "guide.pdf")
    data = open(fn, "rb").read()
    assert data.startswith(b"%PDF") and len(re.findall(rb"/Type\s*/Page[^s]", data)) >= 3
    if shutil.which("pdftotext"):
        txt = subprocess.run(["pdftotext", fn, "-"], capture_output=True, text=True, check=True).stdout
    else:
        try:
            from pypdf import PdfReader
            txt = "\n".join(p.extract_text() for p in PdfReader(fn).pages)
        except ImportError:
            pytest.skip("no pdftotext or pypdf to read the PDF text")
    for s in ("GrappStar demo build guide", "Safety", "battery unplugged", "Parts list", "608ZZ", "Print list",
              "Press in the bearings", "Audit summary", "CLEAN"):
        assert s in txt, s
    assert "\u2014" not in txt
    if shutil.which("pdffonts"):
        fonts = subprocess.run(["pdffonts", fn], capture_output=True, text=True).stdout
        assert "LiberationSans" in fonts or "DejaVuSans" in fonts


def test_no_em_dashes_in_outputs(out):
    for f in glob.glob(os.path.join(out, "**", "*.md"), recursive=True) + glob.glob(os.path.join(out, "**", "*.json"), recursive=True):
        assert "\u2014" not in open(f, encoding="utf-8").read(), f
    for f in ("pack.py", "step_export.py", "bom.py", "guide.py"):
        assert "\u2014" not in open(os.path.join(BASE, "tools", f), encoding="utf-8").read(), f
    assert "\u2014" not in open(os.path.join(BASE, "run_all.sh"), encoding="utf-8").read()
    assert "\u2014" not in open(os.path.abspath(__file__), encoding="utf-8").read()


def test_run_all_syntax():
    subprocess.run(["bash", "-n", os.path.join(BASE, "run_all.sh")], check=True)
