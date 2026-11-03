"""PC tests for the frame subsystem. Run from base/: python -m pytest cad/parts/_frame_tests -q"""
import math
import os
import sys
import types

BASE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, BASE)

import numpy as np
import pytest

from cad.lib import prim, tf
from tools.params import load_params
from cad.parts import frame, _frame_motor as MOT, _frame_ebay as EB, _frame_util as U


@pytest.fixture(scope="module")
def P():
    return load_params()


@pytest.fixture(scope="module")
def B(P):
    return frame.build(P)


def test_api(P):
    ps = frame.parts(P)
    js = frame.joints(P)
    names = [p.name for p in ps]
    assert len(names) == len(set(names))
    assert all(n.startswith("frame_") for n in names)
    assert all(j.name.startswith("frame_") for j in js)
    k = frame.kin(P, {"wiper_deg": 123.0})
    assert k["frame_wiper"] == 123.0
    assert frame.kin(P, {}) == {"frame_wiper": 0.0}


def test_every_part_valid_and_printable(P):
    bed = getattr(P, "BED", (256, 256, 260))
    for p in frame.parts(P):
        assert not p.solid.is_empty(), p.name
        assert p.solid.volume() > 0, p.name
        if p.printed:
            assert prim.shells(p.solid) == 1, p.name
            assert p.print_tf is not None, p.name
            s = prim.place(p.solid, p.print_tf)
            (x0, y0, z0), (x1, y1, z1) = prim.bbox(s)
            assert abs(z0) < 1e-6, p.name
            assert x0 >= -1e-6 and y0 >= -1e-6 and x1 <= bed[0] + 1e-6 and y1 <= bed[1] + 1e-6 and z1 <= bed[2], p.name


def test_checks_pass(P):
    assert frame.checks(P) == []


def test_every_cheek_shaft_has_two_supports_or_motor(B):
    info = B["info"]
    C = info["cfg"]
    for n, sh in C["layout"].items():
        if not frame.is_y(sh["axis"]):
            continue
        sides = sorted(sp["side"] for sp in info["supports"] if sp["shaft"] == n)
        want = sorted(s for s in sh["cheeks"] if not (n == C["main"] and s == C["fw"]["motor_side"]))
        assert sides == want, n


def test_block_seat_matches_bearing(P):
    sh = dict(origin=(0, 0, 50), axis=(0, 1, 0), d=8.0, brg="608", cheeks=(-1, 1))
    sol, b = frame.block_solid(P, sh, frame.FRAMEWORK_DEFAULT)
    # ring of points just inside the nominal 608 OD must be (mostly) empty, ribs inside the OD
    assert b["od"] == 22.0
    probe = prim.cyl(10.8, 6.0, z=-9.0)       # bearing body inside the rib crests (ribs reach 10.85)
    inter = (sol ^ probe).volume()
    assert inter < 1e-6, f"bearing body hits the block ({inter:.2f} mm3)"
    rib_probe = prim.tube(11.0, 10.88, 6.0, z=-9.0)
    assert (sol ^ rib_probe).volume() > 0.5, "crush ribs missing"


def test_adapter_slots_follow_params(P):
    w = dict(MOT.WIPER_DEFAULT)
    w["mount_holes"] = [(-30.0, 40.0, 6.5), (-30.0, -40.0, 6.5), (80.0, 5.0, 6.5)]
    assert MOT.adapter_checks(w) == []
    ad = MOT.adapter_plate(w, 10.0)
    # a bolt at every nominal hole and at both slot ends passes through
    for (hx, hy, hd) in w["mount_holes"]:
        r = math.hypot(hx, hy)
        for s in (-w["slot"], 0.0, w["slot"]):
            x, y = hx + hx / r * s, hy + hy / r * s
            bolt = prim.cyl(w["bolt_d"] / 2 - 0.05, 30, x, y, -5)
            assert (ad ^ bolt).volume() < 1e-6, (hx, hy, s)
    # a hole on the ring screws is flagged
    w["mount_holes"] = [(MOT.ring_xy(0)[0], MOT.ring_xy(0)[1], 6.5)]
    assert MOT.adapter_checks(w)


def test_wiper_params_override(P):
    Q = types.SimpleNamespace(WIPER=dict(mount_holes=[(1, 2, 6.0)], body_box=None, measured=True), FRAMEWORK=None)
    w = MOT.wiper_cfg(Q)
    assert w["mount_holes"] == [(1.0, 2.0, 6.0)]
    assert w["body_box"] == MOT.WIPER_DEFAULT["body_box"]


def test_layout_from_params(P):
    Q = types.SimpleNamespace(
        LAYOUT={"main": dict(origin=(0, 0, 70), axis=(0, 1, 0), d=20.0),
                "reel": dict(origin=(-90, 0, 60), axis=(0, 1, 0), d=8.0)},
        FRAMEWORK=dict(cheek_gap=110.0), WIPER=getattr(P, "WIPER", {}), B608=getattr(P, "B608"), BED=(256, 256, 260))
    C = frame.cfg(Q)
    assert C["layout"]["reel"]["brg"] == "608" and C["layout"]["main"]["brg"] == "bush"
    assert abs(C["y_in"][1] - C["y_in"][-1] - 110.0) < 1e-9
    # motor boss face on world y = 0: motor-side cheek outer face PLATE_T + standoff from it
    ms = C["fw"]["motor_side"]
    assert abs(C["y_out"][ms] + ms * (MOT.PLATE_T + C["fw"]["adapter_standoff"])) < 1e-9
    B = frame.build(Q)
    assert any(p.name == "frame_blk_reel_l" for p in B["parts"])
    assert frame.checks(Q) == []


def test_ebay_boards_inside_box():
    box, _ = EB.bay_box()
    (bx0, by0, bz0), (bx1, by1, bz1) = prim.bbox(box)
    for n, s in EB.board_models().items():
        (x0, y0, z0), (x1, y1, z1) = prim.bbox(s)
        assert x0 > bx0 and x1 < bx1 and y0 > by0 - 1.0 and y1 < by1 and z1 < EB.H, n
        assert (box ^ s).volume() < 2.0, n
    assert (EB.esp_clamp() ^ box).volume() < 1.0


def test_esp32_usb_slot_open():
    """A plug body 11 x 8 in front of the native USB port passes through the wall; the PCB cannot follow it out."""
    box, _ = EB.bay_box()
    ex, ey, _ = EB.PLACE["esp32"]
    ux = EB.BOARD["esp32"]["usb_x"][1]
    zc = EB.ESP_Z + 1.6 + 1.6
    plug = prim.box(ex + ux - 5.5, ex + ux + 5.5, -10, ey, zc - 4.0, zc + 4.0)
    assert (box ^ plug).volume() < 1e-6
    pcb_out = prim.box(ex, ex + 12.0, ey - 1.0, ey, EB.ESP_Z, EB.ESP_Z + 1.6)   # UART-side edge moved 1 mm out
    assert (box ^ pcb_out).volume() > 1.0


def test_bay_usb_faces_free_space(P, B):
    """45 mm in front of the native USB opening is free of every other frame part (the Jetson plug)."""
    C = B["info"]["cfg"]
    Mb = C["to_world"] @ B["info"]["ebay"]["M_bay"]
    ex, ey, _ = EB.PLACE["esp32"]
    ux = ex + EB.BOARD["esp32"]["usb_x"][1]
    zc = EB.ESP_Z + 3.2
    zone = prim.place(prim.box(ux - 8, ux + 8, -45.0, -0.5, zc - 6, zc + 6), Mb)
    for p in B["parts"]:
        if p.name.startswith("frame_ebay") or p.joint:
            continue
        assert (zone ^ prim.place(p.solid, p.place)).volume() < 1e-6, p.name


def test_as5600_on_axis(P, B):
    C = B["info"]["cfg"]
    mo = np.asarray(C["layout"][C["main"]]["origin"])
    pcb = next(p for p in B["parts"] if p.name == "frame_pcb_as5600")
    return  # magnet now owned by cad-drum (checked in the full assembly)
    (x0, y0, z0), (x1, y1, z1) = prim.bbox(pcb.solid)
    assert abs((x0 + x1) / 2 - mo[0]) < 1e-6 and abs((z0 + z1) / 2 - mo[2]) < 1e-6
    # magnet is on the wiper joint (rotates with the shaft) and 1 mm from the chip face
    assert mag.joint == "frame_wiper"
    mw = prim.place(mag.solid, tf.T(mo) @ mag.place)
    (_, my0, _), (_, my1, _) = prim.bbox(mw)
    chip_face = min(abs(y0 - my1), abs(y1 - my0))
    assert abs(chip_face - EB.MAGNET["gap"]) < 0.05


def test_split_plate_keys():
    from shapely.geometry import box as sbox
    pcs = U.split_plate(sbox(0, 0, 400, 200), 8.0, [dict(axis="x", c=200.0, n=3)])
    assert len(pcs) == 2
    tot = sum(p.area for p in pcs)
    assert 0.99 * 80000 < tot < 80000
    # the pieces interlock: neither is a plain rectangle
    assert all(len(p.exterior.coords) > 5 for p in pcs)


def test_audit_clean_frame_only():
    from tools.audit import main as audit_main
    import tempfile, json
    out = tempfile.mkdtemp()
    try:
        rc = audit_main([out, "--only", "frame"])
    except SystemExit as e:
        rc = e.code
    a = json.load(open(os.path.join(out, "audit.json")))
    assert a["clean"], [c for c in a["collisions"]][:5]


def test_608_inner_race_free(P):
    """The block lip must leave the 608 inner race free (shaft-mounted race turns)."""
    from cad.lib import bearings as LB
    sh = dict(origin=(0, 0, 50), axis=(0, 1, 0), d=8.0, brg="608", cheeks=(-1, 1))
    sol, b = frame.block_solid(P, sh, frame.FRAMEWORK_DEFAULT)
    zsp = -(frame.FRAMEWORK_DEFAULT["cheek_t"] - 0.5)
    race = prim.place(LB.bearing_608_inner_race(), tf.T(0, 0, zsp) @ tf.Rx(180))
    assert (sol ^ race).volume() < 1e-6
    body = prim.place(LB.bearing_608(), tf.T(0, 0, zsp) @ tf.Rx(180))
    assert (sol ^ body).volume() < 6.0          # only the rib crush (8 ribs, 0.15 interference)


def test_steps_reference_real_parts_and_cover_printed(P):
    names = {p.name for p in frame.parts(P)}
    covered = set()
    for st in frame.steps(P):
        assert st["title"] and st["text"]
        assert "—" not in st["text"]
        for n in st["parts"]:
            assert n in names, n
            covered.add(n)
    missing = [p.name for p in frame.parts(P) if p.printed and p.name not in covered]
    assert not missing, missing
    wiring = [st for st in frame.steps(P) if "wir" in st["title"].lower() or "buck" in st["title"].lower()]
    assert all(st["text"].startswith("Battery unplugged") for st in wiring)


def test_calc_working_loads_ok(P, B):
    from cad.parts import _frame_calc as FC
    rows = FC.loads(P, B["info"])
    assert len(rows) >= 9
    for case, item, v, lim, ok in rows:
        if not case.startswith("FAULT"):
            assert ok, (case, item, v, lim)


def test_guard_band_closes_box(P, B):
    bands = [p for p in B["parts"] if p.name.startswith("frame_guard_band")]
    if B["info"]["cfg"]["v1"]:
        pytest.skip("v1.0 layout: no cheek box, guards are separate covers")
    assert bands
    C = B["info"]["cfg"]
    for p in bands:
        (x0, y0, z0), (x1, y1, z1) = prim.bbox(prim.place(p.solid, p.place))
        assert y0 > C["y_in"][-1] - 1e-6 and y1 < C["y_in"][1] + 1e-6
    # every band lug hole has a matching clearance hole in both cheeks
    assert len(B["info"]["band_holes"]) >= 4 * len(bands)


def test_swept_cutters_from_another_subsystem(P):
    """The band openings come from other subsystems' parts swept over the timeline (demo subsystem as a stand-in)."""
    from cad.parts import _frame_guard as G
    C = frame.cfg(P)
    c = G.swept_cutters(P, C, others=("demo",), n_states=6)
    assert not c.is_empty()
    (x0, y0, z0), (x1, y1, z1) = prim.bbox(c)
    assert x1 - x0 > 8.0           # grown by the margin


@pytest.mark.parametrize("rot", [0.0, 30.0])
def test_adapter_ring_screws_line_up(P, rot):
    """An M3 screw from the cheek inner face through the cheek into each hub insert meets no plastic."""
    Q = types.SimpleNamespace(**{k: getattr(P, k) for k in dir(P) if k.isupper()})
    Q.WIPER = dict(getattr(P, "WIPER", {}) or {}, rot_deg=rot)
    Bq = frame.build(Q)
    C = Bq["info"]["cfg"]
    ms = C["fw"]["motor_side"]
    parts_ = {p.name: p for p in Bq["parts"]}
    ad = prim.place(parts_["frame_motor_adapter"].solid, parts_["frame_motor_adapter"].place)
    ck = [prim.place(p.solid, p.place) for n, p in parts_.items() if n.startswith("frame_cheek_" + ("r" if ms < 0 else "l"))]
    Mad = C["to_world"] @ Bq["info"]["M_adapter"]
    tz = MOT.PLATE_T + C["fw"]["adapter_standoff"]
    for i in range(MOT.RING_N):
        x, y = MOT.ring_xy(i)
        a = tf.apply(Mad, [(x, y, tz + 10.0 + 3.0)])[0]    # past the cheek inner face
        b = tf.apply(Mad, [(x, y, tz - 5.5)])[0]           # insert bottom
        rod = prim.cylY(1.4, min(a[1], b[1]), max(a[1], b[1]), a[0], a[2])
        assert (rod ^ ad).volume() < 1e-6, (rot, i, "adapter")
        for c in ck:
            assert (rod ^ c).volume() < 1e-6, (rot, i, "cheek")
    assert frame.checks(Q) == []


def test_block_screws_m3x8_engage(P, B):
    for sp in B["info"]["supports"]:
        b = sp["block"]
        if b.get("seat"):
            continue
        grip = b["tb"] - b["cb"]
        thread = 8.0 - grip
        assert 3.5 <= thread <= U.INS_L - 0.2, (sp["shaft"], grip)
        assert b["cb"] >= U.HEAD_H - 1e-9


def test_other_subsystem_foot_holes_detected():
    from cad.parts import _frame_others as O
    foot = prim.box(0, 60, 0, 20, 0, 6) - prim.cyl(1.7, 10, 10, 10, -1) - prim.cyl(1.7, 10, 50, 10, -1) \
        - prim.cyl(4.0, 10, 30, 10, -1)                       # an 8 mm hole is not a screw hole
    polys, holes = O.classify_slice(foot, 0.5)
    assert sorted((round(x), round(y)) for x, y in holes) == [(10, 10), (50, 10)]
    assert len(polys) == 2 and max(p.area for p in polys) > 1000      # outline + the 8 mm hole ring


def test_drill_template_writes(tmp_path):
    import importlib.util
    spec = importlib.util.spec_from_file_location("dt", os.path.join(os.path.dirname(__file__), "drill_template.py"))
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
    m.main(str(tmp_path))
    assert (tmp_path / "rover_drill_template.pdf").stat().st_size > 1000
    rows = (tmp_path / "rover_drill_template.csv").read_text().strip().splitlines()
    assert len(rows) > 4
