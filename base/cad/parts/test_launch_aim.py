"""pytest for launch_aim.py (run: kedge/.venv/bin/python -m pytest base/cad/parts/test_launch_aim.py -s).
Needs cad.lib on the path: base/ of this repo, $GRAPP_BASE, or /tmp/claude-0/run/base."""
from __future__ import annotations

import math
import os
import sys

import numpy as np
import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
for _b in (os.path.dirname(os.path.dirname(_HERE)), os.environ.get("GRAPP_BASE", ""), "/tmp/claude-0/run/base"):
    if _b and os.path.isdir(os.path.join(_b, "cad", "lib")):
        sys.path.insert(0, _b)
        break
sys.path.insert(0, _HERE)

from manifold3d import Error  # noqa: E402
from cad.lib import tf  # noqa: E402
import launch as LN  # noqa: E402
import launch_aim as AIM  # noqa: E402
import launch_gears as G  # noqa: E402
import launch_kin as K  # noqa: E402

P = None
PITCHES = list(np.arange(6.02, 47.0, 1.0)) + [47.0]


def _world(S):
    """Joint world frames from launch.joints + launch.kin (same rule as the assembly)."""
    J = {j.name: j for j in LN.joints(P)}
    v = LN.kin(P, S)
    out = {}

    def M(n):
        if n in out:
            return out[n]
        j = J[n]
        par = M(j.parent) if j.parent else np.eye(4)
        val = v.get(n, 0.0)
        mot = tf.R(j.axis, val) if j.kind == "rev" else tf.T(*(np.asarray(j.axis, float) * val))
        out[n] = par @ tf.T(*j.origin) @ (j.base if j.base is not None else np.eye(4)) @ mot
        return out[n]

    for n in J:
        M(n)
    return out, v


@pytest.mark.parametrize("p", PITCHES)
def test_pitch_matches_kin(p):
    v = AIM.kin_aim(P, {}, p)
    psi = -v["launch_aim_crank"]
    assert float(K.pitch_of_psi(P, psi, +1)) == pytest.approx(p, abs=0.02)


def test_psi_on_rising_branch():
    t = K.pitch_table(P)
    d = [AIM.psi_of_pitch(P, p)[1] for p in PITCHES]
    assert all(b >= a - 1e-9 for a, b in zip(d, d[1:]))
    assert d[0] >= 0 and d[-1] <= t["crank_rise_deg"] + 1e-6


@pytest.mark.parametrize("p", PITCHES)
def test_conrod_length_and_closure(p):
    W, v = _world(dict(aim_deg=p))
    pin = W["launch_aim_conrod"][:3, 3]
    lug = tf.apply(W["launch_rail"], [[AIM.A(P)["lug_u"], 0.0, 0.0]])[0]
    end = tf.apply(W["launch_aim_conrod"], [[AIM.A(P)["rod"], 0.0, 0.0]])[0]
    d = math.hypot(lug[0] - pin[0], lug[2] - pin[2])
    assert d == pytest.approx(156.0, abs=0.05)
    assert np.linalg.norm((end - lug)[[0, 2]]) < 0.05
    # crank pin agrees with launch_kin.crank_pin
    cx, cz = K.crank_pin(P, -v["launch_aim_crank"])
    assert pin[0] == pytest.approx(cx, abs=1e-6) and pin[2] - LN.L(P)["axis_z"] == pytest.approx(cz, abs=1e-6)


def test_worm_self_locking():
    n = AIM.A(P)
    lam = G.worm_lead_angle(n["m_w"], n["pd_w"], n["starts"])
    assert lam < 6.0
    assert K.worm_info(P)["self_locking"]


def test_ratio_chain_matches_kin():
    """Backward wiper (sleeve -) turns the crank so psi decreases, wiper/crank = launch_kin.wiper_per_crank."""
    a, b = AIM.kin_aim(P, {}, 20.0), AIM.kin_aim(P, {}, 21.0)
    ds = b["launch_sleeve"] - a["launch_sleeve"]
    dpsi = -(b["launch_aim_crank"] - a["launch_aim_crank"])
    assert dpsi < 0 and ds < 0
    assert ds / dpsi == pytest.approx(K.wiper_per_crank(P), rel=1e-6)
    assert K.wiper_per_crank(P) == pytest.approx(
        AIM.A(P)["z_w"] / AIM.A(P)["starts"] / AIM.A(P)["bevel_up"] / AIM.A(P)["split"])


def test_parts_valid_single_body():
    for p in AIM.parts_aim(P):
        s = p.solid
        assert s.status() == Error.NoError, p.name
        assert s.volume() > 0, p.name
        assert len(s.decompose()) == 1, p.name
        if p.material != "bought":
            assert p.print_tf is not None, p.name


def test_print_tf_on_bed():
    from cad.lib import prim
    for p in AIM.parts_aim(P):
        if p.print_tf is None:
            continue
        lo, hi = prim.bbox(prim.place(p.solid, p.print_tf))
        assert abs(lo[2]) < 1e-6, p.name
        sz = np.array(hi) - np.array(lo)
        assert sz[2] <= 260 and max(sz[:2]) <= 256, p.name


def test_meshes_phased():
    """Every gear pair is free of overlap over the sweep, and a misphased idler collides (control)."""
    import copy
    parts = {p.name: p for p in LN.parts(P)}
    pairs = [("launch_aim_sun", "launch_aim_idler"), ("launch_aim_idler", "launch_aim_output"),
             ("launch_aim_bevel30", "launch_aim_worm"), ("launch_aim_worm", "launch_aim_crank")]
    from cad.lib import prim

    def posed(name, W):
        p = parts[name]
        return prim.place(p.solid, (W[p.joint] if p.joint else np.eye(4)) @ p.place)

    for pv in (6.02, 18.0, 33.0, 47.0):
        W, _ = _world(dict(aim_deg=pv))
        for a, b in pairs:
            assert (posed(a, W) ^ posed(b, W)).volume() < 1e-3, (a, b, pv)
    W, _ = _world(dict(aim_deg=20.0))
    W2 = copy.deepcopy(W)
    W2["launch_aim_idler"] = W2["launch_aim_idler"] @ tf.R((0, 1, 0), 10.0)
    assert (posed("launch_aim_idler", W2) ^ posed("launch_aim_output", W2)).volume() > 1.0


def test_split_out_clear():
    """Forward turns: arm on its stop, idler clear of the output gear."""
    W, v = _world(dict(aim_deg=25.0, wiper_deg=77.0, aim_split="out"))
    n = AIM.A(P)
    ip = W["launch_aim_idler"][:3, 3]
    d = math.hypot(ip[0] - n["cs_x"], ip[2] - n["cs_z"])
    assert d > G.tip_r(1.5, 18) + G.tip_r(1.5, 30) + 1.0


def _sw_gap(d):
    """Gap (mm) between the crank cam and the SW_RAIL_LOW screw (modelled pressed) at crank d deg from LOW, and the
    overlap volume. Gap 0 = fully pressed (closed); gap >= TRAVEL = tip free."""
    from cad.lib import prim
    import launch_switch as SW
    parts = {p.name: p for p in AIM.parts_aim(P)}
    pv = float(K.pitch_of_psi(P, K.pitch_table(P)["psi_low"] - d, +1))
    W, _ = _world(dict(aim_deg=pv, aim_psi_from_low=d))
    cr = prim.place(parts["launch_aim_crank"].solid, W["launch_aim_crank"] @ parts["launch_aim_crank"].place)
    zc = AIM.A(P)["crank_z"]
    cr = cr ^ prim.box(85, 150, 180, 210, zc - 35, zc + 35)               # only the region round the switch (speed)
    sc = prim.place(parts["launch_sw_rail_low_screw"].solid, parts["launch_sw_rail_low_screw"].place)
    bd = prim.place(parts["launch_sw_rail_low"].solid, parts["launch_sw_rail_low"].place)
    return cr.min_gap(sc, 5.0), (cr ^ sc).volume(), (cr ^ bd).volume(), cr.min_gap(bd, 5.0)


def test_sw_rail_low_window():
    import launch_switch as SW
    closed = []
    for d in np.arange(-10.0, 10.01, 1.0):
        g, v_sc, v_bd, g_bd = _sw_gap(float(d))
        assert v_sc < 1e-3 and v_bd < 1e-3, d            # never pushes past the pressed position, never hits the body
        assert g_bd > 0.5, d
        if g < 0.05:                                  # full travel (screw set 0.05 proud closes it)
            closed.append(float(d))
    print(f"\nSW_RAIL_LOW closed for crank d from LOW in [{min(closed)}, {max(closed)}] deg")
    assert min(closed) >= -7.0 and max(closed) <= 7.0 and min(closed) <= -5.0 and max(closed) >= 5.0
    # released elsewhere: tip free beyond the window (cam below the free tip position)
    for d in (-12.0, 12.0, 30.0, 90.0, 180.0, 270.0):
        assert _sw_gap(d)[0] > SW.TRAVEL, d
    # the audit rest state (pitch 6.02 on the rising branch) is pressed
    d0 = AIM.psi_of_pitch(P, 6.02)[1]
    assert _sw_gap(d0)[0] < 0.05


def test_drum_interface_clear():
    """Housing and AS5600 board clear the main shaft d12 spigot (to y 198.5) and its magnet cap (r 10, y 195..201.5);
    the housing thrust face for the sleeve end is at y 193; the chip is on the axis 1 to 2 mm beyond the magnet."""
    from cad.lib import prim
    n = AIM.A(P)
    az = n["axis_z"]
    spig = prim.cylY(6.0, 96.0, 198.5, 0.0, az)
    cap = prim.cylY(10.0, 195.0, 201.5, 0.0, az)
    parts = {p.name: p for p in AIM.parts_aim(P)}
    for nm in ("launch_aim_housing", "launch_aim_as5600"):
        s = prim.place(parts[nm].solid, parts[nm].place)
        assert (s ^ spig).volume() < 1e-6 and (s ^ cap).volume() < 1e-6, nm
        assert s.min_gap(cap, 2.0) > 0.4, nm
    hs = prim.place(parts["launch_aim_housing"].solid, parts["launch_aim_housing"].place)
    lo, hi = prim.bbox(hs ^ prim.box(-9.9, 9.9, 150, 202, az - 9.9, az + 9.9))
    assert hi[1] < 195.0
    # thrust face: housing material just beyond the sleeve end (y 193) between r 6.25 and r 10
    assert (hs ^ prim.box(7.0, 9.0, 193.0, 194.0, az - 1, az + 1)).volume() > 1.0
    b = prim.bbox(prim.place(parts["launch_aim_as5600"].solid, tf.I4))
    assert 202.5 <= b[0][1] <= 203.5
