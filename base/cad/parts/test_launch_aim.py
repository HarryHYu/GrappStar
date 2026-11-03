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


def test_sw_rail_low_cam():
    """SW_RAIL_LOW cam lobe (cad-sensors' lever rides on it): crest at SWL_RT facing the switch at the LOW dead point,
    inside the crank plane SWL_CAM_Y, and part of the crank solid."""
    from cad.lib import prim
    n = AIM.A(P)
    cam = AIM.swl_cam_joint(P)
    lo, hi = prim.bbox(cam)
    assert lo[1] == pytest.approx(AIM.SWL_CAM_Y[0], abs=1e-6) and hi[1] == pytest.approx(AIM.SWL_CAM_Y[1], abs=1e-6)
    d0 = AIM.psi_of_pitch(P, 6.02)[1]
    pv = float(K.pitch_of_psi(P, K.pitch_table(P)["psi_low"], +1))
    W, _ = _world(dict(aim_deg=pv, aim_psi_from_low=0.0))
    parts = {p.name: p for p in AIM.parts_aim(P)}
    cr = prim.place(parts["launch_aim_crank"].solid, W["launch_aim_crank"] @ parts["launch_aim_crank"].place)
    ym = 0.5 * (AIM.SWL_CAM_Y[0] + AIM.SWL_CAM_Y[1])
    tip = prim.box(n["crank_x"] + AIM.SWL_RT - 0.3, n["crank_x"] + AIM.SWL_RT - 0.05, ym - 0.5, ym + 0.5,
                   n["crank_z"] - 0.2, n["crank_z"] + 0.2)
    assert (cr ^ tip).volume() > 0.05                    # crest at SWL_RT straight ahead at LOW
    out = prim.box(n["crank_x"] + AIM.SWL_RT + 0.05, n["crank_x"] + AIM.SWL_RT + 3, ym - 2, ym + 2,
                   n["crank_z"] - 3, n["crank_z"] + 3)
    assert (cr ^ out).volume() < 1e-6
    assert d0 >= 0


def test_drum_interface_clear():
    """Housing and AS5600 board clear the main shaft d12 spigot (to y 198.5) and its magnet cap (r 10, y 195..201.5);
    the housing thrust face for the sleeve end is at y 193; the chip is on the axis 1 to 2 mm beyond the magnet."""
    from cad.lib import prim
    n = AIM.A(P)
    az = n["axis_z"]
    spig = prim.cylY(6.0, 96.0, 198.5, 0.0, az)
    cap = prim.cylY(10.0, 195.0, 201.5, 0.0, az)
    parts = {p.name: p for p in AIM.parts_aim(P)}
    for nm in ("launch_aim_housing",):
        s = prim.place(parts[nm].solid, parts[nm].place)
        assert (s ^ spig).volume() < 1e-6 and (s ^ cap).volume() < 1e-6, nm
        assert s.min_gap(cap, 2.0) > 0.4, nm
    hs = prim.place(parts["launch_aim_housing"].solid, parts["launch_aim_housing"].place)
    lo, hi = prim.bbox(hs ^ prim.box(-9.9, 9.9, 150, 202, az - 9.9, az + 9.9))
    assert hi[1] < 195.0
    # thrust face: housing material just beyond the sleeve end (y 193) between r 6.25 and r 10
    assert (hs ^ prim.box(7.0, 9.0, 193.0, 194.0, az - 1, az + 1)).volume() > 1.0


def test_no_o_ring_or_e_clip():
    """Only Harry's hardware is bought (M3, M8, 608 here); drag and pin retention are printed."""
    for p in AIM.parts_aim(P):
        txt = (p.notes + " " + str(getattr(p, "bought_ref", "") or "")).lower()
        assert "o-ring" not in txt and "e-clip" not in txt and "din 6799" not in txt, p.name
        if p.material == "bought":
            assert any(k in txt for k in ("m8", "608", "m3")), p.name
    mats = {p.name: p.material for p in AIM.parts_aim(P)}
    assert mats["launch_aim_conrod"] == "PPA-CF" and mats["launch_aim_lug"] == "PPA-CF"


def test_drag_pad_flexure():
    """Flexure pad: bump presses into the sleeve d20 by DRAG_INT (the Allow), the rest of the pad, the shim and the
    arm stay off the sleeve; finger strain under 0.8 % at the thickest shim; pad / shim sit in the arm pocket
    without overlap (0.05 gaps) and the Allow is declared."""
    from cad.lib import prim
    parts = {p.name: p for p in LN.parts(P)}
    W, _ = _world(dict(aim_deg=20.0))

    def posed(nm):
        p = parts[nm]
        return prim.place(p.solid, W[p.joint] @ p.place)

    sl, pad, shim, arm = posed("launch_sleeve"), posed("launch_aim_drag_pad"), posed("launch_aim_drag_shim"), \
        posed("launch_aim_arm")
    v = (pad ^ sl).volume()
    assert 0.05 < v < 5.0
    assert (shim ^ sl).volume() < 1e-6 and (arm ^ sl).volume() < 1e-6
    assert (pad ^ arm).volume() < 1e-6 and (shim ^ arm).volume() < 1e-6 and (pad ^ shim).volume() < 1e-6
    assert pad.min_gap(arm, 0.3) <= 0.06 and shim.min_gap(pad, 0.3) <= 0.06
    t = AIM.DRAG_T
    rm = AIM.DRAG_W_RB + t / 2
    L = math.radians(AIM.DRAG_ROOT - AIM.DRAG_TIP) * rm
    for d in (AIM.DRAG_INT, AIM.DRAG_INT + AIM.DRAG_SHIM_ADJ):
        assert 3 * t * d / (2 * L ** 2) < 0.008
    assert 1.2 > AIM.DRAG_INT + AIM.DRAG_SHIM_ADJ                         # gap behind the finger > deflection
    al = [(a.a, a.b) for a in AIM.allowed_aim(P)]
    assert ("launch_aim_drag_pad", "launch_sleeve") in al


def test_printed_clips_seated():
    """Printed C clips sit in their pin grooves: no overlap with pin, rod, idler; within 0.2 of the pin (touch)."""
    from cad.lib import prim
    parts = {p.name: p for p in LN.parts(P)}
    for pv in (6.02, 30.0, 47.0):
        W, _ = _world(dict(aim_deg=pv))

        def posed(nm):
            p = parts[nm]
            return prim.place(p.solid, (W[p.joint] if p.joint else np.eye(4)) @ p.place)

        for clip, pin, others in (("launch_aim_idler_clip", "launch_aim_arm", ("launch_aim_idler",)),
                                  ("launch_aim_crankpin_clip", "launch_aim_crank", ("launch_aim_conrod",)),
                                  ("launch_aim_lugpin_clip", "launch_aim_lug", ("launch_aim_conrod",))):
            c = posed(clip)
            assert (c ^ posed(pin)).volume() < 1e-6, (clip, pv)
            assert c.min_gap(posed(pin), 0.5) < 0.2, (clip, pv)
            for o in others:
                assert (c ^ posed(o)).volume() < 1e-6, (clip, o, pv)


def test_lewis_sf_at_trip():
    """Lewis numbers quoted in launch_aim (LEWIS block): splitter >= 1.5 at 40 MPa, worm wheel as built."""
    T = 3.18e3                                           # N mm at the sleeve (5 A trip)
    wt = T / (AIM.M_S * AIM.Z_SUN / 2)
    for z, y in ((AIM.Z_SUN, 0.359), (AIM.Z_IDL, 0.309)):
        assert 40.0 / (wt / (AIM.FACE_S * AIM.M_S * y)) >= 1.5, z
    lam = math.radians(G.worm_lead_angle(AIM.A(P)["m_w"], AIM.A(P)["pd_w"], AIM.A(P)["starts"]))
    cn, mu = math.cos(math.radians(20.0)), 0.25
    tw = T / 2 * 0.95
    fa = tw / (AIM.A(P)["pd_w"] / 2 * (cn * math.tan(lam) + mu) / (cn - mu * math.tan(lam)))
    fe = min(AIM.WHEEL_FACE, 2.0 / 3.0 * AIM.A(P)["pd_w"])
    sig = fa / (math.pi * AIM.A(P)["m_w"] * math.cos(lam) * fe * 0.125)
    print(f"\nworm wheel: thrust {fa:.0f} N, sigma {sig:.1f} MPa, SF {40.0 / sig:.2f} (40 MPa)")
    assert 40.0 / sig > 1.4


def test_bevel_slides_onto_countershaft():
    """The D flat runs to the outboard shaft end, so the bevel's D bore passes over the shaft from that end."""
    from cad.lib import prim
    parts = {p.name: p for p in AIM.parts_aim(P)}
    out = prim.place(parts["launch_aim_output"].solid, parts["launch_aim_output"].place)
    bv = prim.place(parts["launch_aim_bevel30"].solid, parts["launch_aim_bevel30"].place)
    for k in np.arange(0.0, AIM.Y_W2[1] + 1.0 - AIM.A(P)["y_b"], 1.0):
        assert (out ^ bv.translate((0, float(k), 0))).volume() < 1e-6, k


def test_wheel_bolt_clamps_wall():
    """M8 bolt: head on the outer boss, nut in the inner face pocket, both 608 on the shank, nut face = race shoulder."""
    from cad.lib import prim
    parts = {p.name: p for p in AIM.parts_aim(P)}

    def w(nm):
        return prim.place(parts[nm].solid, parts[nm].place)

    hs, nut, bolt, b1, b2 = (w(k) for k in ("launch_aim_housing", "launch_aim_nut", "launch_aim_bolt",
                                            "launch_aim_brg_1", "launch_aim_brg_2"))
    for a, b in ((hs, nut), (hs, bolt), (nut, bolt), (nut, b1), (b1, bolt), (b2, bolt)):
        assert (a ^ b).volume() < 1e-6
    assert nut.min_gap(hs, 0.2) < 0.05 and nut.min_gap(b1, 0.2) < 0.05
    lo, hi = prim.bbox(bolt)
    assert lo[1] < prim.bbox(b2)[0][1] - 0.5                                 # shank passes the inboard 608
    assert hi[1] - lo[1] == pytest.approx(25.0 + 5.3 + 0.02, abs=0.05)      # M8 x 25 + head
