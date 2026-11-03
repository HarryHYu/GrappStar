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
    assert len(guards) == 6
    for p in guards:
        assert p.joint == "launch_rail"
        assert prim.shells(p.solid) == 1, p.name
        lo, hi = prim.bbox(prim.place(p.solid, p.print_tf))
        assert abs(lo[2]) < 1e-6
        d = np.array(hi) - np.array(lo)
        assert max(d[0], d[1]) <= 250.0 + 1e-6 and min(d[0], d[1]) <= 256.0 and d[2] <= 260.0, p.name


def test_guard_covers_springs(guards):
    """The plates cover the side of both spring pairs (w 52..68) from u -170 to 264 except where the stop frame front
    legs (u 240.2..248.2) close the side; inner face outside the D5 spring (|y| 64)."""
    for sg, side in ((1, "l"), (-1, "r")):
        g = prim.union([p.solid for p in guards if p.name.startswith(f"launch_guard_{side}_")])
        fr = ST.frame_solid(N, side)
        cover = g + fr
        for u in np.arange(-170.0, 264.0, 4.0):
            probe = prim.box(u, u + 1.0, *sorted((sg * 65.0, sg * 69.0)), 52.0, 68.0)
            assert _ov(probe, cover) > 0.9 * 1.0 * 16.0 * 1.0, (side, u)
        inner = prim.box(-200, 300, *sorted((sg * 29.0, sg * 65.99)), 49.5, 66.9)      # below the top lips
        lo, hi = prim.bbox(g)
        assert min(abs(lo[1]), abs(hi[1])) >= 28.0 - 1e-6
        # nothing inside |y| 66 between the brackets / bosses and the top lips: the springs have the space
        assert _ov(g, inner) < TOL


def test_guard_clear_of_rail_stop_and_arms(guards, rail):
    stops = prim.union([ST.frame_solid(N, s) + ST.spring_solid(N, s) for s in ("l", "r")])
    if hasattr(LA, "carriage_yoke_solid"):
        arms = [LA.carriage_yoke_solid(P)]
    else:
        arms = [LA.carriage_arm_solid(P, sg) for sg in (1, -1)]
    for p in guards:
        assert _ov(p.solid, rail) < TOL, p.name
        assert _ov(p.solid, stops) < TOL, p.name
        for tr in np.linspace(-ST.TRAVEL, N["stroke"], 13):
            for a in arms:
                assert _ov(p.solid, a.translate((N["u_c_home"] - tr, 0, 0))) < TOL, (p.name, tr)


def test_guard_brackets_touch_rail_and_pilots_in_material(guards, rail):
    """Each bracket bears on the rail (contact within 0.05) and every pilot has rail material round it."""
    full = LA.rail_solid(P)          # may already carry the pilots (integrated): test the wall round each pilot
    for (u, y, w, ax) in GD.pilot_positions():
        if ax == "y":
            y0, y1 = sorted((y - np.sign(y) * 4.0, y))
            c = prim.cylY(2.5, y0, y1, x=u, z=w) - prim.cylY(1.31, y0 - 1, y1 + 1, x=u, z=w)
        else:
            c = prim.cyl(2.5, 4.0, x=u, y=y, z=w - 4.0) - prim.cyl(1.31, 6.0, x=u, y=y, z=w - 5.0)
        assert _ov(c, full) > 0.9 * c.volume(), (u, y, w)
    for p in guards:
        grown = p.solid.translate((0, 0, 0))
        near = prim.union([grown.translate((0, dy, 0)) for dy in (-0.05, 0.05)] + [grown.translate((0, 0, -0.05))])
        assert _ov(near, rail) > 0.0, p.name


def test_no_em_dashes():
    for f in ("launch_muzzle.py", "launch_guard.py", "test_launch_muzzle.py"):
        with open(os.path.join(HERE, f), encoding="utf-8") as fh:
            assert chr(0x2014) not in fh.read(), f
