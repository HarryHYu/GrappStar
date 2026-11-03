import math
import os
import sys

BASE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
if BASE not in sys.path:
    sys.path.insert(0, BASE)

import manifold3d as m3
import pytest

from cad.lib import fits, prim, worm

M, Z, D1, FACE = 1.5, 24, 18.0, 10.0


def ok(s):
    assert s.status() == m3.Error.NoError
    assert len(s.decompose()) == 1
    assert s.volume() > 0


def test_info():
    inf = worm.worm_info(M, D1)
    assert abs(inf["lead_angle"] - math.degrees(math.atan(M / D1))) < 1e-9
    assert inf["self_locking"] and inf["back_efficiency"] < 0
    assert not worm.worm_info(2, 10, starts=2)["self_locking"]
    assert abs(worm.worm_center_distance(M, Z, D1) - (D1 + M * Z) / 2 - fits.GEAR_CENTER_EXTRA) < 1e-9
    assert worm.worm_pitch_d(1.5, bore=8) >= 8 + 2 * (1.25 * 1.5 + 1.6)


@pytest.fixture(scope="module")
def wheel():
    return worm.worm_wheel(M, Z, FACE, D1, bore=6, throated=True)


def test_worm_and_wheel_mesh(wheel):
    ok(wheel)
    L = 7 * math.pi * M
    wm = worm.worm(M, L, bore=5, d1=D1)
    ok(wm)
    gap = (fits.GEAR_BACKLASH + 2 * fits.GEAR_CENTER_EXTRA * math.tan(math.radians(20))) / 2
    da = math.degrees((gap + 0.08) / (M * Z / 2))
    for psi in (0, 37, 90, 160, 250, 333):
        W = prim.place(wm, worm.worm_pose(M, Z, D1, FACE, L, psi))
        phi = -psi / Z
        assert (W ^ wheel.rotate((0, 0, phi))).volume() < 1e-6
        hit = max((W ^ wheel.rotate((0, 0, phi + s * da))).volume() for s in (1, -1))
        assert hit > 1e-4


def test_plain_wheel_valid():
    w = worm.worm_wheel(1.0, 20, 6, 12.0, throated=False, hub=(10, 4), bore=4, bore_kind="d")
    ok(w)
