"""PC test for make_guide.py: python3 -m pytest -q docs/build/test_make_guide.py"""
import importlib.util
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("make_guide", os.path.join(HERE, "make_guide.py"))
mg = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mg)


def test_clean_removes_em_dashes():
    assert "—" not in mg.clean("a — b—c – d")


def test_demote_pushes_headings_down_and_keeps_code():
    md = "# Top\n## Sub\n```\n# not a heading\n```\n"
    out = mg.demote(md, 3, mg.ROOT)
    assert "### Top" in out and "#### Sub" in out and "\n# not a heading" in out


def test_missing_include_gives_placeholder():
    out = mg.d_include(["no/such/file.md"], 2)
    assert "NOT IN THE REPO YET" in out and "no/such/file.md" in out
    assert mg.d_include(["no/such/file.md", "optional"], 2) == ""


def test_dart_printlist_lists_plates():
    out = mg.printlist_dart()
    assert "plate_A_hull.3mf" in out and "hull_top" in out


def test_full_build_has_every_section_and_no_em_dash():
    md = mg.build_markdown()
    assert "—" not in md
    heads = re.findall(r"^# (\d+)\.", md, re.M)
    assert heads == [str(i) for i in range(1, 16)], heads
    for must in ("battery unplugged", "608", "20 A fuse", "9.0 V"):
        assert must.lower() in md.lower(), must


def test_base_pack_formats(tmp_path, monkeypatch):
    """The guide must pick up a base pack whatever form it lands in (csv, json, md) plus the coupon sheet."""
    monkeypatch.setattr(mg, "ROOT", str(tmp_path))
    pack = tmp_path / "base" / "pack"
    (pack / "coupon").mkdir(parents=True)
    assert "NOT IN THE REPO YET" in mg.printlist_base()
    (pack / "print_list.csv").write_text("part,material,plate\ndrum_drum,PPA-CF,1\n")
    (pack / "plate_1.3mf").write_text("x")
    out = mg.printlist_base()
    assert "drum_drum" in out and "plate_1.3mf" in out
    (pack / "coupon" / "COUPON.md").write_text("# Coupons\n\n## Tile A\nCheck the 608 seat.\n")
    inc = mg.d_includeglob(["base/**/coupon/COUPON.md", "first"], 2)
    assert "Tile A" in inc and "#### Tile A" in inc
    (pack / "print_report.json").write_text('{"parts": {"frame_deck": {"material": "PLA", "plate": 2}}}')
    (pack / "print_list.csv").unlink()
    assert "frame_deck" in mg.printlist_base()
