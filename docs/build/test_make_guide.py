"""PC test for make_guide.py: python3 -m pytest -q docs/build/test_make_guide.py"""
import importlib.util
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("make_guide", os.path.join(HERE, "make_guide.py"))
mg = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mg)


def test_clean_removes_em_dashes():
    assert "\u2014" not in mg.clean("a \u2014 b\u2014c \u2013 d")


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
    assert "\u2014" not in md
    heads = re.findall(r"^# (\d+)\.", md, re.M)
    assert heads == [str(i) for i in range(1, 17)], heads
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


AVOIDED = {0, 3, 45, 46, 19, 20, 26, 27, 28, 29, 30, 31, 32, 43, 44, 38, 48}


def test_every_gpio_in_the_guide_matches_pins_h():
    """Every 'GPIO n' / 'GPIOn' in the built guide and the diagram is a pin pins.h uses (or one it avoids on purpose)."""
    pins = set(mg.read_pins().values())
    assert pins, "pins.h not found"
    md = mg.build_markdown()
    used = {int(n) for n in re.findall(r"\bGPIO ?(\d+)\b", md)}
    bad = sorted(n for n in used if n not in pins | AVOIDED)
    assert not bad, f"GPIO numbers not in pins.h: {bad}"
    dot = open(os.path.join(HERE, "system_electrical.dot")).read()
    assert not re.search(r"(RPWM|LPWM|R_EN|L_EN|AIN1|AIN2|SDA|SCL|DT|SCK) \d", dot), "hard-coded GPIO in the diagram"
    for name in re.findall(r"\{(PIN_\w+)\}", dot):
        assert name in mg.read_pins(), name
