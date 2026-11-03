import os
import sys

BASE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
if BASE not in sys.path:
    sys.path.insert(0, BASE)

import manifold3d as m3
import pytest

from cad.lib import prim, screw


def ok(s):
    assert s.status() == m3.Error.NoError
    assert len(s.decompose()) == 1
    assert s.volume() > 0


def test_info():
    inf = screw.screw_info(12, 3)
    assert inf["d2"] == 10.5 and inf["d3"] == 8.5 and inf["lead"] == 3
    assert inf["self_locking"] and 0 < inf["efficiency"] < 0.5
    fast = screw.screw_info(12, 3, starts=4)
    assert fast["lead"] == 12 and fast["lead_angle"] > inf["lead_angle"]
    assert fast["efficiency"] > inf["efficiency"]


@pytest.mark.parametrize("d,p,starts", [(12, 3, 1), (10, 2, 2)])
def test_screw_nut(d, p, starts):
    L, LN = 24.0, 10.0
    lead = p * starts
    sc = screw.screw(d, p, L, starts=starts)
    nt = screw.nut(d, p, LN, starts=starts, hex_af=1.8 * d)
    ok(sc); ok(nt)
    lo, hi = prim.bbox(sc)
    assert abs(hi[0] - (d / 2 - screw.SCREW_CLEAR / 2)) < 0.05
    for t in (4.0, 4.0 + p, 7.3):
        n = nt.rotate((0, 0, 360.0 * t / lead)).translate((0, 0, t))
        assert (sc ^ n).volume() < 1e-6
    # wrong phase or negative clearance collides
    assert (sc ^ nt.rotate((0, 0, 360.0 * 4 / lead + 90 / starts)).translate((0, 0, 4))).volume() > 1
    scn = screw.screw(d, p, L, starts=starts, clearance=-0.2)
    ntn = screw.nut(d, p, LN, starts=starts, od=2 * d, clearance=-0.2)
    ok(ntn)
    assert (scn ^ ntn.rotate((0, 0, 360.0 * 4 / lead)).translate((0, 0, 4))).volume() > 0.1
