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
    assert C["y_in"][1] == 55.0
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
    """A USB plug body 12 x 8 in front of the native port must pass through the wall slot."""
    box, _ = EB.bay_box()
    ex, ey, _ = EB.PLACE["esp32"]
    for ux in EB.BOARD["esp32"]["usb_x"]:
        plug = prim.box(ex + ux - 5.5, ex + ux + 5.5, -10, ey, EB.ESP_Z + 1.6 - 2.5, EB.ESP_Z + 1.6 + 5.5)
        assert (box ^ plug).volume() < 1e-6, ux


def test_as5600_on_axis(P, B):
    C = B["info"]["cfg"]
    mo = np.asarray(C["layout"][C["main"]]["origin"])
    pcb = next(p for p in B["parts"] if p.name == "frame_pcb_as5600")
    mag = next(p for p in B["parts"] if p.name == "frame_as5600_magnet")
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
