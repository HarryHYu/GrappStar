"""PC tests for the launcher (worker cad-launch). Run from base/: python -m pytest cad/parts/test_launch.py -q"""
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
import launch as L                    # noqa: E402
import launch_kin as K                # noqa: E402

P = load_params()


def test_stroke_within_spring_limit():
    d = K.lp(P)
    ext = d["preload"] + K.stroke(P)
    assert ext <= P.SPRING["max_ext"] + 1e-6
    assert d["preload"] >= 5.0


def test_release_matches_kin_stroke():
    t = L.release_table(P)
    assert t["theta_release"] is not None
    # real release travel within 1 mm of the formula used for the energy
    assert abs(t["travel_release"] - K.stroke(P)) < 1.0
    # spring never stretched past max_ext at the release
    assert K.lp(P)["preload"] + t["travel_release"] <= P.SPRING["max_ext"] + 1.0


def test_travel_monotonic_and_starts_at_home():
    t = L.release_table(P)
    tr = t["travel"]
    assert tr[0] == 0.0
    # springs keep the rack on the gear: it only creeps forward a little as the relieved last tooth rolls off
    assert np.all(np.diff(tr) >= -1.0)
    assert K.lp(P)["preload"] + tr.max() <= P.SPRING["max_ext"] + 1.0


def test_gap_covers_shot_and_reset():
    g = K.gears(P)
    rel = -L.gear_release_angle(P)
    # after release the gear must turn to 360 through the gap without driving the rack: gap must exceed what is left
    assert g["gap_deg"] > 360.0 - rel - 1e-6 - 10.0
    assert 360.0 - rel > 20.0                   # room for the wiper overrun during the shot


def test_wiper_per_shot():
    g = K.gears(P)
    assert abs(g["ratio"] * 360 - 553.85) < 0.1


def test_energy_and_force():
    E = K.spring_energy(P)
    assert 9.0 < E < 16.0
    assert K.gears(P)["torque_wiper"] < 5.0     # under the keyed-joint limit (physics F6)


def test_pitch_table_matches_v10():
    t = K.pitch_table(P)
    assert abs(t["pitch_min"] - 6.01) < 0.05
    assert abs(t["pitch_max"] - 47.06) < 0.05


def test_kin_values_rest_and_cocked():
    v0 = L.kin(P, dict(aim_deg=20.0, cock=0.0))
    v1 = L.kin(P, dict(aim_deg=20.0, cock=1.0))
    assert v0["launch_carriage"] == 0.0
    assert abs(v1["launch_carriage"] - K.stroke(P)) < 1e-6
    assert v1["launch_big_gear"] < v0["launch_big_gear"]          # cocking turns the big gear negative
    assert v1["launch_drive"] > v0["launch_drive"]                # driven by FORWARD wiper turns


@pytest.mark.parametrize("name", ["launch_carriage", "launch_big_gear", "launch_drive_gear", "launch_sleeve",
                                  "launch_rail_rear", "launch_rail_mid", "launch_rail_front", "launch_sw_home",
                                  "launch_sw_seated", "launch_gear_pin", "launch_gear_clip",
                                  "launch_seat_plunger", "launch_seat_pushrod", "launch_line_p"])
def test_parts_valid_single_body_and_fit_bed(name):
    parts = {p.name: p for p in L.parts(P)}
    p = parts[name]
    s = p.solid
    assert s.volume() > 0
    assert len(s.decompose()) == 1, f"{name} has {len(s.decompose())} bodies"
    if p.print_tf is not None:
        from cad.lib import prim
        lo, hi = prim.bbox(prim.place(s, p.print_tf))
        assert abs(lo[2]) < 1e-6
        size = np.array(hi) - np.array(lo)
        assert size[0] <= 256 and size[1] <= 256 and size[2] <= 260


def test_switch_positions_exist():
    names = {p.name for p in L.parts(P)}
    for sw in ("launch_sw_home", "launch_sw_seated"):
        assert sw in names


def test_big_gear_full_layer_has_all_teeth():
    """Review 1: the drive gear must mesh a FULL 40T layer (the sector layer has a 117 deg gap)."""
    n = L.L(P)
    g = L.big_gear_solid(P)
    ym = 0.5 * (n["y_full"][0] + n["y_full"][1]) - n["y_sec"][0]        # local z of the full-layer mid plane
    cs = g.slice(ym)
    ang = np.linspace(0, 2 * np.pi, 3600, endpoint=False)
    r_test = n["rb"] + 0.5 * n["m"]                                     # between pitch and tip radius
    from manifold3d import CrossSection
    hits = 0
    prev = False
    pts = [(r_test * np.cos(a), r_test * np.sin(a)) for a in ang]
    inside = [cs.contains(p) if hasattr(cs, "contains") else None for p in pts[:1]]
    # count tooth crossings by intersecting thin wedges
    for k in range(40):
        a = 2 * np.pi * k / 40
        x, y = r_test * np.cos(a), r_test * np.sin(a)
        dot = CrossSection.circle(0.3, 8).translate((x, y))
        hits += (cs ^ dot).area() > 0.05
    assert hits == 40


def test_timeline_continuity():
    """No backward jumps of the big gear, drive gear or sleeve between timeline states (review 1)."""
    from tools.timeline import timeline
    prev = None
    for S in timeline(P, 0.25):
        v = L.kin(P, S)
        if prev is not None:
            assert v["launch_big_gear"] <= prev["launch_big_gear"] + 1e-6, S.get("phase")
            assert v["launch_drive"] - prev["launch_drive"] > -60.0, S.get("phase")   # pitch changes move it a little
        prev = v


def test_kin_tolerates_none_and_missing():
    for S in ({}, {"aim_deg": None}, {"cock": None}, {"fired": None, "wiper_deg": None}, {"aim_psi_from_low": 120}):
        L.kin(P, S)


def test_sprag_outer_key_rib_fits_keyway():
    """Review r3 P #1: the drive gear rib must fit the CSK20 OUTER keyway (1.5 deep), not the 2.8 inner depth."""
    from cad.lib import bearings as BR, prim, tf
    dg = L.drive_gear_solid(P)
    sp = L.sprag_solid(P)
    assert (dg ^ sp).volume() < 1e-3
    real = prim.place(BR.bearing_csk20(okey_deg=0.0), tf.T(0, 0, 7.0))       # catalogue model, z -7..7
    assert (dg ^ real).volume() < 1e-3


def test_lip_levers_rest_gap_open_and_strain_ok():
    """SW_HOME / SW_SEATED levers are modelled at rest: the screw head stands the contact gap above the nut; the
    lever strain (cad/lib/switch.py beam model, closing, overtravel and screw backed off) stays under 0.8 percent;
    kin presses the lever only with the carriage at HOME (head then on the nut), and leaves it at rest when cocked."""
    from cad.lib import prim, tf
    import launch_switch as SW
    for side in ("seated", "home"):
        st = L.seat_geom(P, side)
        li = st["lever"]
        assert li["strain_max"] < 0.008 and li["strain_ok"]
        assert 0.3 <= li["gap"] <= 1.0 and li["overtravel"] > 0.1
        screw = prim.place(L.seat_screw_solid(P, side), st["M"])        # rest pose, rail frame
        nut = L.seat_nut_solid(P, side)
        assert (screw ^ nut).volume() < 1e-6
        assert abs(screw.min_gap(nut, 5.0) - li["gap"]) < 0.02           # contact open by the gap
        # pad above the HOME foot bottom by pretravel + overtravel at rest
        lo, hi = prim.bbox(prim.place(L.seat_lever_solid(P, side), st["M"]))
        assert abs(hi[2] - (st["w_tip"] + li["pretravel"] + li["overtravel"])) < 0.01
        # pressed pose (kin at HOME): head on the nut (closed), cocked: rest
        v_home = L.kin(P, dict(aim_deg=20.0, cock=0.0))[f"launch_sw_{side}_lever"]
        v_cock = L.kin(P, dict(aim_deg=20.0, cock=0.5))[f"launch_sw_{side}_lever"]
        assert v_cock == 0.0 and v_home > 0.0
        pv = st["pivot"]
        Mp = tf.mul(tf.T(pv), tf.R((0, 1, 0), v_home), tf.T(-pv[0], -pv[1], -pv[2]), st["M"])
        sp = prim.place(L.seat_screw_solid(P, side), Mp)
        assert (sp ^ nut).volume() < 1e-3 and sp.min_gap(nut, 5.0) < 0.01


def test_d49_line_from_the_cup_eyelet_on_the_dart_axis():
    """D49: no tether pulleys on the rail; the line comes straight from cad-reel's eyelet on the dart axis to the
    pusher, and the rail rear end carries the reel head flange."""
    names = {p.name for p in L.parts(P)}
    assert not any("pulley" in n and "muzzle" not in n for n in names)
    S = dict(aim_deg=20.0, cock=0.0)
    spans = L.line_path(P, S)
    e, pu = spans[0][0]
    from cad.lib import tf
    Minv = tf.inv(L._rail_to_world(P, L.kin(P, S)["launch_rail"]))
    e_r, pu_r = tf.apply(Minv, np.array([e, pu]))
    assert e_r[0] == pytest.approx(-183.0, abs=0.5) and abs(e_r[1]) < 0.01
    assert e_r[2] == pytest.approx(pu_r[2], abs=0.01) == L.L(P)["w_dart"] - 0.4
    assert "reel_cup" in spans[0][1]
