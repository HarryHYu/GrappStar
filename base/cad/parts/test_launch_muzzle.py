"""Tests for the lip roller + retrieval funnel (launch_muzzle.py) and the spring guards (launch_guard.py).
Run from base/: PYTHONPATH=. python -m pytest cad/parts/test_launch_muzzle.py -q"""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.dirname(os.path.dirname(HERE))
for p in (BASE, HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

from tools.params import load_params  # noqa: E402
from cad.lib import prim, tf           # noqa: E402
import launch as LA                    # noqa: E402
import launch_stop as ST               # noqa: E402
import launch_muzzle as MZ             # noqa: E402
import launch_guard as GD              # noqa: E402

P = load_params()
N = LA.L(P)
TOL = 1e-3
BIG = 1000.0


def _ov(a, b):
    return (a ^ b).volume()


@pytest.fixture(scope="module")
def add():
    return MZ.muzzle_carriage_add(P, N)


@pytest.fixture(scope="module")
def carriage(add):
    return LA.carriage_solid(P) + add


@pytest.fixture(scope="module")
def rail():
    return LA.rail_solid(P) - GD.guard_rail_cut(N)


@pytest.fixture(scope="module")
def guards():
    return GD.guard_parts(P, N)


def _roller_parts():
    """Roller parts in the carriage local frame."""
    H = tf.T(-N["u_c_home"], 0, 0)
    return {p.name: prim.place(p.solid, tf.mul(H, p.place)) for p in MZ.muzzle_parts(P, N)}


# ------------------------------------------------------------------------------------------------ carriage add
def test_carriage_one_body_and_envelope(carriage, add):
    assert prim.shells(carriage) == 1
    lo, hi = prim.bbox(add)
    assert lo[1] >= -N["y_shoe1"] - 1e-6 and hi[1] <= N["y_shoe1"] + 1e-6         # |y| <= 21.5
    assert lo[2] >= N["w_rtip"] - 1e-6                                              # nothing below the rack tips
    assert hi[0] <= MZ.U_FRONT + 1e-6
    # nothing below w 47.6 in the whole carriage, and the body still prints across the bed diagonal
    clo, chi = prim.bbox(carriage)
    assert clo[2] >= N["w_rtip"] - 1e-6
    p = prim.place(carriage, LA.bed(carriage, tf.Rx(90), rz=45.0))
    lo, hi = prim.bbox(p)
    assert hi[0] - lo[0] <= 256.0 and hi[1] - lo[1] <= 256.0 and abs(lo[2]) < 1e-6


def test_roller_below_dart_and_above_rack(add):
    r = _roller_parts()["launch_muzzle_roller"]
    lo, hi = prim.bbox(r)
    assert hi[2] <= 62.5 + 1e-6
    assert lo[2] >= N["w_rtip"]
    # no carriage material at or above the dart belly in front of the cradle (dart flies off at w 63.6)
    belly = N["w_dart"] - 0.4 - N["r_hull"]
    front = add ^ LA.B(MZ.U_F1, 400, -50, 50, belly - 1.0, 300)
    assert front.volume() < TOL


def test_roller_parts_clear_of_fork(add):
    rp = _roller_parts()
    assert _ov(rp["launch_muzzle_roller"], add) < TOL
    assert _ov(rp["launch_muzzle_roller_brg"], add) < TOL
    assert _ov(rp["launch_muzzle_roller_axle"], add) < TOL
    for side in "lr":
        assert _ov(rp[f"launch_muzzle_washer_{side}"], add) < TOL
        assert _ov(rp[f"launch_muzzle_washer_{side}"], rp["launch_muzzle_roller"]) < TOL
        assert _ov(rp[f"launch_muzzle_washer_{side}"], rp["launch_muzzle_roller_brg"]) < TOL


def test_roller_drops_into_fork(add):
    """Roller with bearing and both washers moves straight down into place from above the fork (no printed boss in
    its way), and the M3 x 16 reaches through the nyloc nut (head at |y| 5.5, nut 4 thick)."""
    rp = _roller_parts()
    unit = rp["launch_muzzle_roller"] + rp["launch_muzzle_roller_brg"] + rp["launch_muzzle_washer_l"] + \
        rp["launch_muzzle_washer_r"]
    for dw in np.arange(0.0, 30.0, 1.0):
        assert _ov(unit.translate((0, 0, dw)), add) < TOL, dw
    lo, hi = prim.bbox(rp["launch_muzzle_roller_axle"])
    assert hi[1] >= MZ.Y_CHEEK1 + 4.0 + 0.5 and hi[1] <= MZ.Y_CHEEK1 + 4.0 + 2.5


def test_line_clear_over_carriage(carriage):
    """Line from the pusher hole to the roller top: nothing of the carriage within 1 mm of it (in front of the
    pusher), so the line touches only the roller groove."""
    import launch_line as LN
    p0 = (N["pusher_t"] + 0.5, 0.0, N["w_dart"] - 0.4)
    p1 = MZ.roller_top(N)
    tube = LN.seg(p0, p1, r=1.0)
    assert _ov(tube, carriage) < TOL


def test_funnel_clear_of_flying_dart(add):
    """Dart hull r 21 on the rail axis (w 84.6), moving forward: never touches the funnel or fork."""
    hull = prim.cylX(N["r_hull"], 0.0, 400.0, y=0.0, z=N["w_dart"] - 0.4, seg=128)
    assert _ov(hull, add) < 0.05


def test_funnel_guides_low_and_offcentre_tail_cap(add):
    """A tail cap (d 42) arriving 4 mm low or 3 mm off centre at the funnel mouth meets the funnel wall, but fits on the
    cradle axis further back (the cone narrows towards the cradle)."""
    cap_low = prim.cylX(21.0, MZ.U_F1 - 20.0, MZ.U_F1 - 10.0, y=0.0, z=N["w_dart"] - 0.4 - 4.0)
    cap_side = prim.cylX(21.0, MZ.U_F0 + 2.0, MZ.U_F0 + 12.0, y=3.0, z=N["w_dart"] - 0.4)
    assert _ov(cap_low, add) > 1.0
    assert _ov(cap_side, add) > 1.0


def test_carriage_add_clear_of_rail_over_travel(add, rail):
    """The extension never enters the rail (track, lips, cheeks, stop ledges) from 19 mm overshoot to full cock."""
    for tr in np.linspace(-ST.TRAVEL - 0.5, N["stroke"] + 0.5, 25):
        a = add.translate((N["u_c_home"] - tr, 0, 0))
        assert _ov(a, rail) < TOL, tr


def test_roller_clear_of_big_gear_at_full_cock():
    """Big gear tip circle (r 52.5 about the gear pin) vs the roller and the fork parts below the deck at full cock."""
    tr = N["stroke"] + 0.5
    uc = MZ.U_R + N["u_c_home"] - tr
    d = math.hypot(uc - N["cd"], MZ.W_R)
    assert d - (N["rb"] + N["m"]) - 7.0 > 1.5
    gear = prim.cylY(N["rb"] + N["m"], -6.0, 14.3, x=N["cd"], z=0.0, seg=180)
    add = MZ.muzzle_carriage_add(P, N).translate((N["u_c_home"] - tr, 0, 0))
    assert _ov(gear, add) < TOL


# ------------------------------------------------------------------------------------------------ guards
def test_guard_parts_print_and_bodies(guards):
    assert len(guards) == 8
    for p in guards:
        assert p.joint == "launch_rail"
        assert prim.shells(p.solid) == 1, p.name
        lo, hi = prim.bbox(prim.place(p.solid, p.print_tf))
        assert abs(lo[2]) < 1e-6
        d = np.array(hi) - np.array(lo)
        assert max(d[0], d[1]) <= 250.0 + 1e-6 and min(d[0], d[1]) <= 256.0 and d[2] <= 260.0, p.name


def _side(guards, side):
    return prim.union([p.solid for p in guards if p.name.startswith(f"launch_guard_{side}_")])


def test_guard_covers_springs(guards):
    """The side of both spring pairs (w 52..68) is closed from u -178 to 264 by the plates or the stop frame
    (rear leg, front legs); the rear end wall closes u -180.9; the top cover spans |y| 42..66 over segments a and c
    (rest of the top: stop springs); nothing inside |y| 66 between the brackets and the lips / cover."""
    for sg, side in ((1, "l"), (-1, "r")):
        g = _side(guards, side)
        cover = g + ST.frame_solid(N, side)
        for u in np.arange(-178.0, 264.0, 2.0):
            probe = prim.box(u, u + 1.0, *sorted((sg * 65.0, sg * 81.0)), 52.0, 68.0)
            assert _ov(probe, cover) > 0.5 * 2.0 * 16.0, (side, u)          # gaps under 1 mm only
        back = prim.box(GD.U_A[0], GD.U_A[0] + 2.0, *sorted((sg * 29.0, sg * 65.0)), 40.0, 85.0)
        assert _ov(back, g) > 0.95 * back.volume()
        for u0, u1 in ((GD.U_A[0], GD.U_A[1]), (GD.U_C[0], GD.U_C[1])):
            top = prim.box(u0, u1, *sorted((sg * 42.0, sg * 66.0)), 83.5, 85.5)
            assert _ov(top, g) > 0.95 * top.volume()
        inner = prim.box(-178.8, 300, *sorted((sg * 29.0, sg * 65.99)), 49.5, 66.9)
        assert _ov(g, inner) < TOL


def test_guard_clear_of_rail_stop_arms_dart(guards, rail):
    stops = prim.union([ST.frame_solid(N, s) + ST.spring_solid(N, s) for s in ("l", "r")])
    if hasattr(LA, "carriage_yoke_solid"):
        arms = [LA.carriage_yoke_solid(P)]
    else:
        arms = [LA.carriage_arm_solid(P, sg) for sg in (1, -1)]
    dart = [p for p in LA.parts(P) if p.name == "launch_dart"][0]
    dart = prim.place(dart.solid, dart.place).translate((0, 0, N["w_dart"]))      # rail frame at HOME
    for p in guards:
        assert _ov(p.solid, rail) < TOL, p.name
        assert _ov(p.solid, stops) < TOL, p.name
        for tr in list(np.linspace(-ST.TRAVEL, N["stroke"], 13)) + [N["stroke"] + 0.5]:
            for a in arms:
                assert _ov(p.solid, a.translate((N["u_c_home"] - tr, 0, 0))) < TOL, (p.name, tr)
            assert _ov(p.solid, dart.translate((-tr, 0, 0))) < TOL, (p.name, tr)
        for d in (100.0, 250.0):                                                    # flying off along the rail
            assert _ov(p.solid, dart.translate((d, 0, 0))) < TOL, (p.name, d)


def test_guard_inserts_inward_with_stop_frame_fitted(guards, rail):
    """Each segment reaches its place by a straight move inward along y (from 20 mm out) with the rail, the stop
    units and the springs (both pairs, as cylinders) present."""
    stops = prim.union([ST.frame_solid(N, s) + ST.spring_solid(N, s) for s in ("l", "r")])
    springs = prim.union([prim.cylX(8.0, -150.0, 264.0, y=sg * yy, z=N["w_spring"])
                          for sg in (1, -1) for yy in (N["y_spring"], LA.Y_SPRING2)])
    world = rail + stops + springs
    for p in guards:
        sg = 1.0 if "_l_" in p.name else -1.0
        for dy in np.arange(0.5, 20.0, 0.5):
            assert _ov(p.solid.translate((0, sg * dy, 0)), world) < TOL, (p.name, dy)


def test_bracket_screw_and_key_paths_clear(guards):
    """Head (d5.5) seat and hex key path along each wall bolt axis are free of guard material from the tab face out
    through the plate; the stop frame rear-foot bolt key paths (u 84.6, 90.6) are clear of the guard."""
    for p in guards:
        sg = 1.0 if "_l_" in p.name else -1.0
        seg = p.name.split("_")[-1]
        for b in GD.WALL_BRACKETS.get(seg, {}).get(sg, ()):
            u = GD._uh(b)[0]
            path = prim.cylY(2.9, *sorted((sg * (GD.Y_WALL + 4.05), sg * 80.0)), x=u, z=GD.W_BOLT)
            assert _ov(path, p.solid) < TOL, (p.name, u)
        for du, w in ST.REAR_BOLTS:
            path = prim.cylY(3.4, *sorted((sg * 35.0, sg * 81.0)), x=N["u_stop"] + du, z=w)
            assert _ov(path, p.solid) < TOL, (p.name, du)


def test_guard_brackets_touch_rail_and_pilots_in_material(guards, rail):
    """Each segment bears on the rail (contact within 0.05), every pilot has rail material round it, the edge bolt
    pilots stay clear of the spring anchor holes, and each screw length fits its pilot."""
    full = LA.rail_solid(P)          # may already carry the pilots (integrated): test the wall round each pilot
    for (u, y, w, ax) in GD.pilot_positions():
        if ax == "y":
            y0, y1 = sorted((y - np.sign(y) * 4.0, y))
            c = prim.cylY(2.5, y0, y1, x=u, z=w) - prim.cylY(1.31, y0 - 1, y1 + 1, x=u, z=w)
        else:
            c = prim.cyl(2.5, 4.0, x=u, y=y, z=w - 4.0) - prim.cyl(1.31, 6.0, x=u, y=y, z=w - 5.0)
        assert _ov(c, full) > 0.9 * c.volume(), (u, y, w)
    holes = prim.union([prim.cyl(LA.ANCHOR_D / 2 + 0.1, 60.0, x=uu, y=sg * yy, z=20.0)
                        for uu in (N["u_anchor"], N["u_anchor"] - LA.ANCHOR_F) for sg in (1, -1)
                        for yy in (N["y_spring"], LA.Y_SPRING2)])
    assert _ov(GD.guard_rail_cut(N), holes) < TOL
    # screw lengths: wall M3x12 through a 4 mm tab, 10.5 deep pilot; foot M3x10 through 3.5, 9.5 deep pilot;
    # edge M3x12 through plate + boss 4 mm, 9.5 deep pilot: each leaves 1..3 mm of pilot below the tip
    for L_s, grip, depth in ((12.0, 4.0, 10.5), (10.0, 3.5, 9.5), (12.0, GD.Y_OUT - GD.Y_EDGE, 9.5)):
        assert 1.0 <= depth - (L_s - grip) <= 3.0
    for p in guards:
        near = prim.union([p.solid.translate((0, dy, 0)) for dy in (-0.05, 0.05)] + [p.solid.translate((0, 0, -0.05))])
        assert _ov(near, rail) > 0.0, p.name


def test_no_em_dashes():
    for f in ("launch_muzzle.py", "launch_guard.py", "test_launch_muzzle.py"):
        with open(os.path.join(HERE, f), encoding="utf-8") as fh:
            assert chr(0x2014) not in fh.read(), f
