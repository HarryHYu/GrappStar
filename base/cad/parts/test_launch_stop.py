"""Tests for the printed PLA carriage stop (launch_stop.py). Run from base/:
PYTHONPATH=. python -m pytest cad/parts/test_launch_stop.py -q"""
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
from cad.lib import prim               # noqa: E402
import launch as LA                    # noqa: E402
import launch_stop as S                # noqa: E402

P = load_params()
N = LA.L(P)
TOL = 1e-3     # mm3: touching faces give zero volume


def _ov(a, b):
    return (a ^ b).volume()


@pytest.fixture(scope="module")
def parts():
    return S.stop_parts(P, N)


@pytest.fixture(scope="module")
def carriage():
    """Carriage body + bolted arm yoke, carriage local frame."""
    return LA.carriage_solid(P) + LA.carriage_yoke_solid(P)


def test_calc_meets_spec():
    c = S.stop_calc(P)
    assert c["travel_mm"] <= 24.0
    assert c["energy_j_at_travel"] >= 4.2
    assert c["peak_force_n"] <= 300.0
    assert c["peak_stress_mpa"] <= 35.0
    assert c["shear_mpa"] < 10.0
    # carriage settles on the ledges at u_stop: preload beats the main spring pull (about 20 N)
    assert c["preload_force_n"] >= 40.0
    assert 20.0 < c["volume_cm3"] < 80.0


def test_parts_valid_and_fit_bed(parts):
    assert len(parts) == 4
    for p in parts:
        assert p.joint == "launch_rail"
        assert prim.shells(p.solid) == 1, p.name
        assert p.solid.volume() > 1000.0
        lo, hi = prim.bbox(prim.place(p.solid, p.print_tf))
        assert abs(lo[2]) < 1e-6
        dims = sorted(np.array(hi) - np.array(lo))
        assert hi[2] - lo[2] <= 260.0
        assert max(hi[0] - lo[0], hi[1] - lo[1]) <= 256.0, (p.name, dims)


def test_gaps_never_close_before_hard_stop():
    lay = S._stack_layout(N)
    assert len(lay["voids"]) == S.n_leaves() + 1
    assert min(float(np.min(hi - lo)) for lo, hi, _ in lay["voids"]) >= S.G_MIN - 1e-9
    # every gap is at least the closure at TRAVEL (solid stack exactly there, not before)
    assert lay["u_front"] <= S.U_FW


def test_keepouts_static(parts):
    us = N["u_stop"]
    keep = []
    for sg in (1, -1):
        keep.append(("spring", LA.cyl_u(8.0, 70.0, 290.0, sg * N["y_spring"], N["w_spring"])))
        keep.append(("D5 spring", LA.cyl_u(8.0, 70.0, 290.0, sg * 56.0, N["w_spring"])))
        keep.append(("D5 band", LA.B(70.0, 290.0, *sorted((sg * 47.0, sg * 65.0)), 51.0, 69.0)))
        keep.append(("dart flank", LA.B(60.0, 400.0, *sorted((sg * 21.5, sg * 120.0)), 84.0, 200.0)))
        keep.append(("rail strap", LA.B(N["u_anchor"] - 8, N["u_anchor"] + 8, *sorted((sg * 30.0, sg * 47.0)), 30.0, 75.0)))
    keep.append(("cradle", LA.B(-170.0, 340.0, -24.4, 24.4, 53.0, 80.0)))
    for p in parts:
        for nm, k in keep:
            assert _ov(p.solid, k) < TOL, (p.name, nm)
    # rail (old TPU pad brackets removed) + stop add: only face contacts
    rail = LA.rail_solid(P)
    for p in parts:
        assert _ov(p.solid, rail) < TOL, p.name
    sp = [p.solid for p in parts if "spring" in p.name]
    fr = [p.solid for p in parts if "frame" in p.name]
    for a in sp:
        for b in fr:
            assert _ov(a, b) < TOL


def test_overshoot_arm_touches_only_contact_face(carriage):
    """Carriage at HOME + d (d = 0..TRAVEL): no volume overlap with any stop solid; the arm touches the pad of R
    (moved with it)."""
    for d in np.linspace(0.0, S.TRAVEL, 7):
        c = carriage.translate((N["u_c_home"] + d, 0, 0))
        for nm, sol, role in S.stop_overshoot_solids(P, N, d):
            assert _ov(c, sol) < TOL, (d, nm)
            if role == "contact":
                # touching: a 0.05 mm forward shift of the carriage must bite into the pad only
                bite = _ov(c.translate((0.05, 0, 0)), sol)
                assert bite > 0.0, (d, nm)


def test_plunger_stays_behind_moving_R():
    """Carriage parts other than the yoke (SW_SEATED plunger foot) never reach R, which moves with the arm."""
    pl = [p for p in LA.parts(P) if p.name == "launch_seat_plunger"][0]
    pls = prim.place(pl.solid, pl.place)
    for d in np.linspace(0.0, S.TRAVEL, 7):
        for nm, sol, role in S.stop_overshoot_solids(P, N, d):
            assert _ov(pls.translate((d, 0, 0)), sol) < TOL, (d, nm)


def test_audit_states_shape():
    st = S.stop_audit_states(P)
    assert st[0]["launch_travel"] == 0.0 and abs(st[-1]["launch_travel"] + S.TRAVEL) < 1e-9
