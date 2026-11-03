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
        n = pack.count_3mf_items(f)
        assert n == pack.count_3mf_objects(f)
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
    assert all(pl["build_items"] == len(pl["objects"]) for pl in rep["plates"])
    assert sum(t["count"] for t in rep["totals"].values()) == total and rep["unplaced_totals"] == {}
    assert all(pl["time_h"] > 0 for pl in rep["plates"])
    for fn in ("PRINT_SHEET.md", "PRINT_SHEET.pdf"):
        assert os.path.getsize(os.path.join(out, "pack", fn)) > 500


def test_plate_too_big_is_an_error():
    items = [dict(name="huge", material="PLA", dims=[300.0, 20.0, 10.0], qty=1),
             dict(name="ok", material="PLA", dims=[20.0, 20.0, 10.0], qty=3)]
    plates, errors = pack.pack_plates(items, (256, 256, 260), 5.0, 5.0)
    assert len(errors) == 1 and "huge" in errors[0]
    assert sum(len(p["items"]) for p in plates) == 3


def test_plate_fits_bed_not_margin_gets_own_plate():
    """Audit and pack agree on the bed: 250 x 250 fits the 256 bed, so it is placed alone and centred (warning)."""
    items = [dict(name="panel", material="PLA", dims=[250.0, 250.0, 4.0], qty=2),
             dict(name="long", material="PLA", dims=[20.0, 253.0, 4.0], qty=1),
             dict(name="ok", material="PLA", dims=[20.0, 20.0, 10.0], qty=3)]
    warns = []
    plates, errors = pack.pack_plates(items, (256, 256, 260), 5.0, 5.0, warns)
    assert errors == []
    assert len(warns) == 2 and any("panel" in w for w in warns) and any("long" in w for w in warns)
    solo = [p for p in plates if p.get("solo")]
    assert len(solo) == 3 and all(len(p["items"]) == 1 for p in solo)
    for p in solo:
        it = p["items"][0]
        m = trimesh.creation.box([*items[0 if it["name"] == "panel" else 1]["dims"]])
        m.apply_translation(-m.bounds[0])
        lo, hi = pack.place_on_plate(m, it).bounds
        assert lo[0] >= 0 and lo[1] >= 0 and hi[0] <= 256 and hi[1] <= 256
        assert lo[0] == pytest.approx(256 - hi[0]) and lo[1] == pytest.approx(256 - hi[1])     # centred
    assert sum(len(p["items"]) for p in plates) == 6
    _, errors = pack.pack_plates([dict(name="big", material="PLA", dims=[257.0, 20.0, 4.0], qty=1)], (256, 256, 260), 5, 5)
    assert len(errors) == 1


def test_step_reimport(out):
    from build123d import import_step
    rep = pack.load_json(os.path.join(out, "step", "step_report.json"))
    assert sorted(r["part"] for r in rep["parts"]) == sorted(STEP_PARTS)
    for r in rep["parts"]:
        assert r["ok"], r
    assert rep["assembly"]["ok"] and rep["assembly"]["n_solids"] == len(STEP_PARTS) and rep["assembly"]["bad"] == []
    assert rep["invalid"] == []
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
    # the guide's steps are the assembly page's steps: every part appears, in the same numbering
    from tools.steps import plan_steps
    page, _ = plan_steps(Assembly.load(load_params(), ["demo"]))
    gs, _ = guide.guide_steps(pack.load_out(out, "demo"), "demo")
    assert [(s["title"], s["parts"]) for s in gs] == [(s["title"], s["parts"]) for s in page]
    for n, s in enumerate(gs, 1):
        assert f"Step {n}. {s['title']}" in txt
        for p in s["parts"]:
            assert p in txt, p


@pytest.mark.skipif(not shutil.which("pdffonts"), reason="no pdffonts")
def test_pdfs_embed_only_our_font(out):
    for fn in (os.path.join(out, "guide.pdf"), os.path.join(out, "pack", "PRINT_SHEET.pdf")):
        lines = subprocess.run(["pdffonts", fn], capture_output=True, text=True, check=True).stdout.splitlines()[2:]
        names = [ln.split()[0] for ln in lines if ln.strip()]
        assert names and all(re.sub(r"^[A-Z]{6}\+", "", n).startswith(pack.fonts()[0]) for n in names), (fn, names)
        assert all(" yes " in ln for ln in lines if ln.strip()), lines          # embedded
        info = subprocess.run(["pdfinfo", fn], capture_output=True, text=True).stdout
        assert not re.search(r"^Producer:[ \t]*\S", info, re.M), info


def test_no_em_dashes_in_outputs(out):
    for f in glob.glob(os.path.join(out, "**", "*.md"), recursive=True) + glob.glob(os.path.join(out, "**", "*.json"), recursive=True):
        assert "\u2014" not in open(f, encoding="utf-8").read(), f
    for f in ("pack.py", "step_export.py", "bom.py", "guide.py"):
        assert "\u2014" not in open(os.path.join(BASE, "tools", f), encoding="utf-8").read(), f
    assert "\u2014" not in open(os.path.join(BASE, "run_all.sh"), encoding="utf-8").read()
    assert "\u2014" not in open(os.path.abspath(__file__), encoding="utf-8").read()


def test_run_all_syntax():
    subprocess.run(["bash", "-n", os.path.join(BASE, "run_all.sh")], check=True)


# ---------------------------------------------------------------------------------------------- review round 2
def test_step_invalid_fails_the_exit_code(tmp_path, monkeypatch):
    """A part or assembly that reimports invalid makes main return 2 (the run still writes every file)."""
    from build123d import export_step
    from cad.lib import prim
    s = prim.box(0, 10, 0, 10, 0, 10) + prim.box(10, 20, 10, 20, 0, 10)          # touches itself along an edge
    sol, _, _ = step_export.mesh_to_solid(*prim.mesh_of(s))
    fn = str(tmp_path / "edge.step")
    export_step(sol, fn)
    assert not step_export.check_step(fn, s.volume())["ok"]
    real = step_export.check_step
    monkeypatch.setattr(step_export, "check_step", lambda fn, v, *a, **k: dict(real(fn, v, *a, **k), ok=False))
    rc = step_export.main([str(tmp_path / "o"), "--only", "demo", "--parts", "demo_shaft_1", "--jobs", "1", "--no-assembly"])
    assert rc == 2
    assert pack.load_json(str(tmp_path / "o" / "step" / "step_report.json"))["invalid"] == ["demo_shaft_1"]


def test_step_assembly_checks_each_solid(tmp_path):
    """Two parts with swapped labels: the total volume matches, the per part check does not."""
    from build123d import Box, Compound, Location, export_step
    a, b = Box(10, 10, 10), Box(20, 20, 20).moved(Location((40, 0, 0)))
    a.label, b.label = "big", "small"
    c = Compound(children=[a, b])
    fn = str(tmp_path / "asm.step")
    export_step(c, fn)
    r = step_export.check_assembly_step(fn, {"small": 1000.0, "big": 8000.0})
    assert not r["ok"] and len(r["bad"]) == 2 and r["rel_err"] < 1e-6
    a.label, b.label = "small", "big"
    export_step(Compound(children=[a, b]), fn)
    assert step_export.check_assembly_step(fn, {"small": 1000.0, "big": 8000.0})["ok"]
    assert not step_export.check_assembly_step(fn, {"small": 1000.0, "big": 8000.0, "gone": 5.0})["ok"]


def test_guide_battery_rule_and_auto_steps():
    man = dict(subsystems=["s"], steps=[
        dict(title="Wire the motor", text="Connect the red lead to the switch.", parts=["s_motor"], subsystem="s"),
        dict(title="Solder", text="Battery unplugged. Solder the JST plug.", parts=["s_jst"], subsystem="s"),
        dict(title="Fit the base", text="Put the base down.", parts=["s_base"], subsystem="s")],
        parts=[dict(name=n, subsystem="s", step=st, joint=None, printed=True)
               for n, st in (("s_motor", None), ("s_jst", None), ("s_base", None), ("s_lid", 3), ("s_spare", None))])
    steps, notes = guide.guide_steps(man)
    assert [s["parts"] for s in steps] == [["s_motor"], ["s_jst"], ["s_base", "s_lid"], ["s_spare"]]
    assert steps[0]["text"] == "Battery unplugged. Connect the red lead to the switch."
    assert steps[1]["text"].count("Battery unplugged.") == 1
    assert not steps[2]["text"].startswith("Battery")
    assert len(notes) == 1 and "step 1" in notes[0]
    for t in ("Crimp the terminals", "Plug in the connector", "Route the wiring", "Check the battery"):
        assert guide.is_wiring(dict(title="", text=t)), t
    for t in ("Fit the shafts and gears", "Press the bearings", "Slide the unplugged end"):
        assert not guide.is_wiring(dict(title="", text=t)), t


def test_bom_parsing_categories_grouping(tmp_path):
    pr = bom.parse_ref
    assert pr("8 x 120 mm ground steel rod") == [(1, "8 x 120 mm ground steel rod")]
    assert pr("2xM3x8 BHCS") == [(2, "M3x8 BHCS")]
    assert pr("2 x 608ZZ 8x22x7; 4 x M3x10 SHCS") == [(2, "608ZZ 8x22x7"), (4, "M3x10 SHCS")]
    assert pr("10 x 3x8 dowel pin") == [(1, "10 x 3x8 dowel pin")]
    for r in ("608ZZ", "608-2RS", "693ZZ", "F693ZZ", "CSK20PP", "6204", "6806-2RS", "LM8UU linear bearing"):
        assert bom.category(r) == "bearings", r
    for r in ("100 mm M8 threaded rod", "120 mm ground steel rod", "12345 widget", "2020 extrusion 300 mm"):
        assert bom.category(r) != "bearings", r
    E, N = chr(0x2014), chr(0x2013)
    man = dict(parts=[
        dict(name="a", printed=True, material="PLA", qty=2, mass_g=1.0, subsystem="s", notes=f"Deck {E} near bed",
             bought_ref="2 x M3 nut; 1 x Cable | red"),
        dict(name="b", printed=False, material="bought", qty=1, subsystem="s", notes="", bought_ref=f"m3   NUT; Wire {N} 2 m"),
        dict(name="c", printed=False, material="steel", qty=2, subsystem="s", notes="", bought_ref="8 x 120 mm rod")])
    b = bom.build(man)
    by = {x["ref"].casefold(): x for x in b["bought"]}
    assert by["m3 nut"]["count"] == 5 and by["m3 nut"]["used_by"] == ["a", "b"]
    assert by["8 x 120 mm rod"]["count"] == 2
    bom.write(str(tmp_path), b)
    raw = (tmp_path / "BOM.csv").read_text() + (tmp_path / "BOM.md").read_text()
    assert E not in raw and N not in raw
    assert "Deck, near bed" in raw and "Wire, 2 m" in raw
    assert "Cable \\| red" in (tmp_path / "BOM.md").read_text()
    assert pack.clean(f"a {E} b{E}c 10{N}20 x {N} y") == "a, b, c 10-20 x, y"


def _fake_out(d):
    """A minimal OUT (asm.json + meshes) for pack.run: name collision, a margin-only part, a part bigger than the bed."""
    import json
    parts = []
    for name, dims, qty in (("x", (10, 10, 10), 2), ("x_c1", (20, 20, 5), 1), ("panel", (250, 250, 4), 1),
                            ("bar", (300, 10, 10), 1)):
        m = trimesh.creation.box(dims)
        m.apply_translation(-m.bounds[0])
        os.makedirs(os.path.join(d, "meshes"), exist_ok=True)
        np.savez(os.path.join(d, "meshes", f"{name}.npz"), v=m.vertices, f=m.faces)
        parts.append(dict(name=name, subsystem="t", material="PLA", qty=qty, printed=True, print_tf=np.eye(4).tolist(),
                          volume_mm3=float(m.volume), mass_g=1.0, notes=""))
    with open(os.path.join(d, "asm.json"), "w") as fh:
        json.dump(dict(subsystems=["t"], parts=parts, steps=[]), fh)


def test_pack_names_items_bedfit_totals(tmp_path):
    d = str(tmp_path)
    _fake_out(d)
    rep = pack.run(d, os.path.join(d, "pack"), log=lambda *a: None)
    assert len(rep["errors"]) == 1 and "bar" in rep["errors"][0]
    assert any("panel" in w and "own plate" in w for w in rep["warnings"])
    objs = [o for pl in rep["plates"] for o in pl["objects"]]
    assert sorted(objs) == ["panel", "x__c1", "x__c2", "x_c1"]
    for pl in rep["plates"]:
        assert pack.count_3mf_items(os.path.join(d, "pack", "plates", pl["file"])) == len(pl["objects"]) == pl["build_items"]
    assert rep["totals"]["PLA"]["count"] == 4 and rep["totals"]["PLA"]["plates"] == len(rep["plates"])
    assert rep["unplaced_totals"]["PLA"]["count"] == 1 and rep["unplaced_totals"]["PLA"]["parts"] == ["bar"]
    by = {r["name"]: r for r in rep["parts"]}
    assert rep["totals"]["PLA"]["mass_g"] == pytest.approx(by["x"]["mass_g"] * 2 + by["x_c1"]["mass_g"] + by["panel"]["mass_g"], abs=0.2)
    assert sum(pl["time_h"] for pl in rep["plates"]) == pytest.approx(rep["totals"]["PLA"]["time_h"], abs=0.02)
    md = open(os.path.join(d, "pack", "PRINT_SHEET.md")).read()
    assert "Not placed" in md and "| time h |" in md


def test_3mf_item_count_catches_trimesh_name_clash(tmp_path):
    m = trimesh.creation.box([5, 5, 5])
    sc = trimesh.Scene()
    for nm in ("x_c1", "x_c2", "x_c1"):                  # the old naming: part x qty 2 plus a part named x_c1
        sc.add_geometry(m.copy(), node_name=nm, geom_name=nm)
    fn = str(tmp_path / "dup.3mf")
    with open(fn, "wb") as fh:
        fh.write(trimesh.exchange.threemf.export_3MF(sc))
    assert pack.count_3mf_items(fn) == 2                 # one build item lost: the counter sees it
    names = pack.object_names([dict(name="x", copy=1), dict(name="x", copy=2), dict(name="x_c1", copy=1)],
                              {"x": 2, "x_c1": 1})
    assert len(set(names)) == 3
    assert pack.object_names([dict(name="a", copy=1), dict(name="a", copy=1)], {"a": 1}) == ["a", "a__2"]
    with pytest.raises(ValueError):
        pack.write_3mf(fn, [("a", m), ("a", m)])


def test_mass_and_time_from_profile():
    m = trimesh.creation.box([40, 40, 40])
    pla = pack.print_mass_g(m.volume, m.area, "PLA")
    ppa = pack.print_mass_g(m.volume, m.area, "PPA-CF")
    tpu = pack.print_mass_g(m.volume, m.area, "TPU")
    shell = m.area * 4 * pack.LINE_W
    assert pla == pytest.approx((shell + (m.volume - shell) * 0.40) * 1.24 / 1000)
    assert ppa > pla > tpu
    assert pack.PROFILE["PPA-CF"]["walls"] == 6 and pack.PROFILE["PPA-CF"]["infill"] == 0.60
    assert "6 walls" in " ".join(pack.SETTINGS["PPA-CF"]) and "60 % gyroid" in " ".join(pack.SETTINGS["PPA-CF"])
    assert "4 walls" in " ".join(pack.SETTINGS["PLA"]) and "40 % gyroid" in " ".join(pack.SETTINGS["PLA"])
    assert "3 walls, 25 % gyroid" in " ".join(pack.SETTINGS["TPU"])
    one = pack.print_time_h(10.0, 50.0, "PLA")
    assert pack.plate_time_h([10.0] * 4, 50.0, "PLA") < 4 * one - 3 * 50 / 0.2 * pack.LAYER_S / 3600 + 1e-9


def _fake_py(tmp_path, codes):
    """A stand-in python for run_all.sh: logs its args, exits with codes[module] (default 0)."""
    log = tmp_path / "calls.log"
    sh = tmp_path / "fakepy"
    cases = "".join(f'  {m}) exit {c} ;;\n' for m, c in codes.items())
    sh.write_text(f'#!/usr/bin/env bash\necho "$@" >> "{log}"\ncase "$2" in\n{cases}esac\nexit 0\n')
    sh.chmod(0o755)
    return str(sh), log


def test_run_all_problems_cleanup_jobs(tmp_path):
    out = tmp_path / "o"
    (out / "pack").mkdir(parents=True)
    (out / "views").mkdir()
    for f in ("guide.pdf", "BOM.md", "BOM.csv", "sim.html", "assembly_body.html", "audit_sheet.png", "keep.txt"):
        (out / f).write_text("stale")
    py, log = _fake_py(tmp_path, {"tools.audit": 1})
    r = subprocess.run(["bash", os.path.join(BASE, "run_all.sh"), "demo", str(out)], capture_output=True, text=True,
                       env=dict(os.environ, PY=py, JOBS="3"))
    assert r.returncode == 1 and "NOT CLEAN" in r.stdout
    assert sorted(os.listdir(out)) == ["audit.log", "keep.txt"]      # stale downstream outputs gone, audit log kept
    assert "tools.audit " + str(out) + " --only demo --jobs 3" in log.read_text()
    py, log = _fake_py(tmp_path, {"tools.step_export": 2})
    r = subprocess.run(["bash", os.path.join(BASE, "run_all.sh"), "base", str(out)], capture_output=True, text=True,
                       env={k: v for k, v in dict(os.environ, PY=py).items() if k != "JOBS"})
    assert r.returncode == 1 and "step export (exit 2)" in r.stdout.split("Problems:")[1]
    calls = log.read_text()
    assert "tools.step_export " + str(out) + " --jobs 2" in calls and "tools.guide" in calls   # the chain went on
    for bad in ("/", os.path.expanduser("~"), BASE, os.path.join(BASE, "tools")):
        r = subprocess.run(["bash", os.path.join(BASE, "run_all.sh"), "demo", bad], capture_output=True, text=True,
                           env=dict(os.environ, PY=py))
        assert r.returncode == 2 and "refusing" in r.stdout, bad
    sh = open(os.path.join(BASE, "run_all.sh")).read()
    assert '"${ONLY[@]}"' not in sh.replace('${ONLY[@]+"${ONLY[@]}"}', "")     # bash 3.2: no bare empty array


# ---------------------------------------------------------------------------------------------- review R2-I #3, #7
def _write_man(d, subsystems, printed=True, mtime=None):
    import json
    parts = [dict(name=f"{s}_p", subsystem=s, material="PLA", qty=1, printed=printed, volume_mm3=1000.0, mass_g=1.0,
                  notes="", bought_ref="" if printed else "M3 nut") for s in subsystems]
    fn = os.path.join(d, "asm.json")
    with open(fn, "w") as fh:
        json.dump(dict(subsystems=list(subsystems), parts=parts, steps=[]), fh)
    if mtime is not None:
        os.utime(fn, (mtime, mtime))


@pytest.fixture
def fake_export(monkeypatch):
    """Assembly.load stubbed: records the subsystems asked for, export writes a manifest of one part per subsystem
    (printed unless the subsystem is in the no_print set)."""
    calls, no_print = [], set()

    class Stub:
        def __init__(self, names):
            self.names = names

        def export(self, out):
            _write_man(out, self.names, printed=not (set(self.names) & no_print))

    def load(cls, P=None, names=None):
        calls.append(list(names or []))
        return Stub(list(names or ["base"]))
    monkeypatch.setattr(Assembly, "load", classmethod(load))
    return calls, no_print


def test_load_out_reexports_missing_lacking_or_stale_manifest(tmp_path, fake_export):
    calls, _ = fake_export
    d = str(tmp_path)
    future = 4.0e9                                       # newer than every source file
    man = pack.load_out(d, "demo", log=lambda *a: None)  # missing
    assert calls == [["demo"]] and [p["name"] for p in man["parts"]] == ["demo_p"]
    _write_man(d, ["frame"], mtime=future)               # a base export without demo
    man = pack.load_out(d, "demo", log=lambda *a: None)
    assert calls[-1] == ["demo"] and man["subsystems"] == ["demo"] and len(man["parts"]) == 1
    _write_man(d, ["demo"], mtime=1.0)                   # older than cad/parts and cad/lib
    pack.load_out(d, "demo", log=lambda *a: None)
    assert len(calls) == 3
    _write_man(d, ["demo", "frame"], mtime=future)       # fresh superset: filtered, not re-exported
    man = pack.load_out(d, "frame", log=lambda *a: None)
    assert len(calls) == 3 and [p["name"] for p in man["parts"]] == ["frame_p"]
    _write_man(d, ["frame"], mtime=1.0)                  # without --only an existing manifest is used as is
    assert pack.load_out(d, log=lambda *a: None)["subsystems"] == ["frame"] and len(calls) == 3


def test_no_printed_parts_fails_pack_bom_guide(tmp_path, fake_export):
    """Review R2-I #3: an --only request with no printed parts exits non-zero instead of writing empty outputs."""
    calls, no_print = fake_export
    no_print.add("demo")
    d = str(tmp_path)
    for main, args in ((pack.main, [d, os.path.join(d, "pack"), "--only", "demo"]), (bom.main, [d, "--only", "demo"]),
                       (guide.main, [d, "--only", "demo"])):
        _write_man(d, ["frame"], mtime=4.0e9)            # stale for demo: re-exported, still nothing printed
        with pytest.raises(SystemExit) as ei:
            main(args)
        assert ei.value.code not in (0, None) and "no printed parts" in str(ei.value.code)
    assert calls == [["demo"]] * 3
    assert not os.path.exists(os.path.join(d, "BOM.csv")) and not os.path.exists(os.path.join(d, "guide.pdf"))
    assert not os.path.exists(os.path.join(d, "pack", "print_report.json"))


def test_bom_totals_equal_pack_totals(out):
    """Review R2-I #7: BOM (and so the guide's parts list) uses the pack PROFILE mass, not Part.mass_g."""
    import json
    rep = json.load(open(os.path.join(out, "pack", "print_report.json")))
    b = bom.build(pack.load_out(out, "demo"), out)
    pk = {}
    for tot in (rep["totals"], rep["unplaced_totals"]):
        for mat, t in tot.items():
            pk[mat] = pk.get(mat, 0.0) + t["mass_g"]
    assert set(b["totals"]) == set(pk)
    for mat, t in b["totals"].items():
        assert t["mass_g"] == pytest.approx(pk[mat], abs=0.2), mat
    each = {r["name"]: r["mass_g"] for r in rep["parts"]}
    assert {r["name"]: r["mass_g"] for r in b["printed"]} == each
    with open(os.path.join(out, "BOM.csv")) as fh:
        csv_tot = {r["name_or_ref"]: float(r["mass_g_total"]) for r in csv.DictReader(fh) if r["section"] == "total"}
    assert csv_tot == {m: t["mass_g"] for m, t in b["totals"].items()}
