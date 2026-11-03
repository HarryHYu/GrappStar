"""Launcher + aim kinematics and sizing (worker cad-launch). Pure numpy, no CAD.

Design: Harry's v10 launcher (originals/handover_launcher_v10.md) sized to the real dart (KEDGE v30).
- Rail pivots on the main axis. Drive gear (sprag #1 inside) on the axis turns the big gear, which rides on the rail
  on its own pin and meshes the rack under the carriage. One big gear turn = cock + fire + reset.
- Aim: v10 crank-rocker scaled by AIM_K (pitch table identical to v10), driven only by backward wiper turns through a
  swing-arm direction splitter, a 2:1 bevel and a self-locking worm (the crank gear is the worm wheel).

Rail frame (used everywhere in this file): origin on the main axis, u along the rail towards the muzzle, w up
(perpendicular to the rail, in the launch plane), y left. World pitch p rotates the rail frame about +y... see
`rail_to_world`: x = u cos p - w sin p, z = u sin p + w cos p (pitch up = muzzle up).

Every number has a default here; when base/spec/params.py has LAUNCH / AIM blocks their values override these
(`lp(P)`). Shared numbers (spring, sprag, dart) are read from P, never copied.
"""
from __future__ import annotations

import math
import numpy as np

# ------------------------------------------------------------------------------------------------ local defaults
DEFAULTS = dict(
    # springs (P.SPRING gives k, F0, free length, max extension)
    preload=5.0,            # mm extension at home (carriage held on the front stop)
    n_lead=3,               # rest: tooth 0 this many pitches before the mesh centre (first contact with the carriage on its stop)
    # cocking gears
    m_rack=2.5,             # module of big gear and rack
    z_big=40,               # big gear full tooth count (pitch d 100)
    z_drive=26,             # drive gear on the main axis (holds sprag #1, 47 OD)
    face=12.0,              # gear face width
    # aim
    aim_k=0.6,              # scale of Harry's v10 crank-rocker (full-scale v10 numbers below)
    aim_rot=-90.0,           # deg, whole crank-rocker rotated about the pivot (pitch table unchanged)
    worm_wheel_z=20,        # crank gear = worm wheel teeth
    worm_starts=1,
    bevel_up=2.0,           # bevel speed-up from splitter output to worm shaft
    split_ratio=1.0,        # splitter sun : output (output turns per sun turn)
    m_worm=2.0,
    worm_pd=24.0,           # worm pitch diameter
)

# v10 full-scale crank-rocker (from originals/GrappStar_PISTON_SIMULATION.html, desk units / 0.3)
V10 = dict(cx=50.27 / 0.3, cy=29.02 / 0.3, rc=15.0 / 0.3, rod=78.0 / 0.3, rl=57.6 / 0.3, lug_ang=90.0)


def lp(P=None):
    d = dict(DEFAULTS)
    for blk in ("LAUNCH", "AIM"):
        b = getattr(P, blk, None) if P is not None else None
        if isinstance(b, dict):
            d.update({k: v for k, v in b.items() if k in d})
    return d


def spring(P):
    s = getattr(P, "SPRING", None) or dict(k_n_per_mm=0.125, F0_n=7.0, max_ext=250.0, free_len=165.1, qty=2)
    return s


# ------------------------------------------------------------------------------------------------ springs and energy
def spring_force(P, ext_mm):
    """Total force of all springs (N) at a given extension (mm), linear above the initial tension."""
    s = spring(P)
    e = np.asarray(ext_mm, float)
    return s.get("qty", 2) * np.where(e > 0, s["F0_n"] + s["k_n_per_mm"] * e, 0.0)


REL_EXTRA = 13.1     # mm of carriage travel past (n - 1) pitches before the relieved last tooth lets go (2D sweep)


def stroke_max(P):
    """Largest carriage travel the springs allow (mm)."""
    d = lp(P)
    return spring(P)["max_ext"] - d["preload"]


def stroke(P):
    """Real stroke: carriage travel at release for the chosen tooth count (<= stroke_max)."""
    d = lp(P)
    m = d["m_rack"]
    pitch = math.pi * m
    n = int(math.floor((stroke_max(P) - REL_EXTRA) / pitch)) + 1 - d["n_lead"]
    return (n - 1 + d["n_lead"]) * pitch + REL_EXTRA


def spring_energy(P):
    """Energy released between full cock and home (J)."""
    s = spring(P)
    d = lp(P)
    x0 = d["preload"]
    x1 = x0 + stroke(P)
    per = s["F0_n"] * (x1 - x0) + 0.5 * s["k_n_per_mm"] * (x1 ** 2 - x0 ** 2)
    return s.get("qty", 2) * per / 1000.0


# ------------------------------------------------------------------------------------------------ cocking gears
def gears(P):
    d = lp(P)
    m = d["m_rack"]
    rb = m * d["z_big"] / 2
    rd = m * d["z_drive"] / 2
    S = stroke_max(P)
    # teeth needed: rack travel while engaged ~ (teeth present - 1) pitches + contact on the last tooth
    pitch = math.pi * m
    # release travel measured on the real 2D teeth (launch.release_table): (n - 1) pitches + REL_EXTRA
    n_present = int(math.floor((S - REL_EXTRA) / pitch)) + 1 - d["n_lead"]
    gap_teeth = d["z_big"] - n_present
    return dict(m=m, r_big=rb, r_drive=rd, cd=rb + rd + 0.1, pitch=pitch, n_present=n_present,
                gap_teeth=gap_teeth, gap_deg=gap_teeth * 360.0 / d["z_big"],
                ratio=d["z_big"] / d["z_drive"],          # wiper turns per shot (1 big gear turn)
                torque_big=float(spring_force(P, lp(P)["preload"] + stroke(P))) * rb / 1000.0,
                torque_wiper=float(spring_force(P, lp(P)["preload"] + stroke(P))) * rb / 1000.0 / (d["z_big"] / d["z_drive"]))


def cock_vs_wiper(P, wiper_fwd_deg):
    """Carriage travel back from home (mm) vs forward wiper degrees from the home stop (rest: tooth 1 engaged).
    One shot = gears(P)['ratio'] * 360 wiper degrees. Release when travel reaches the stroke."""
    g = gears(P)
    big = np.asarray(wiper_fwd_deg, float) / g["ratio"]           # big gear deg
    per = 360.0
    ph = np.mod(big, per)
    travel = np.radians(ph) * g["r_big"]
    S = stroke(P)
    return np.where(travel < S, travel, 0.0)


# ------------------------------------------------------------------------------------------------ aim crank-rocker
def aim_geom(P):
    d = lp(P)
    k = d["aim_k"]
    a = math.radians(d["aim_rot"])
    cx, cy = V10["cx"] * k, V10["cy"] * k
    return dict(cu=cx * math.cos(a) - cy * math.sin(a), cw=cx * math.sin(a) + cy * math.cos(a),
                rc=V10["rc"] * k, rod=V10["rod"] * k, rl=V10["rl"] * k, lug_ang=V10["lug_ang"] + d["aim_rot"])


def pitch_of_psi(P, psi_deg, branch=+1):
    """Rail pitch (deg) for crank angle psi (deg, world frame, from +x towards +z about the crank centre).
    Crank centre and lug are in the WORLD launch plane (x forward, z up) relative to the pivot."""
    g = aim_geom(P)
    psi = np.radians(np.asarray(psi_deg, float))
    Cx = g["cu"] + g["rc"] * np.cos(psi)
    Cz = g["cw"] + g["rc"] * np.sin(psi)
    # lug L(p) = rl (cos(A + p), sin(A + p)); |L - C| = rod  ->  rl^2 + |C|^2 - 2 rl (Cx cos t + Cz sin t) = rod^2
    R = np.hypot(Cx, Cz)
    phi = np.arctan2(Cz, Cx)
    c = (g["rl"] ** 2 + R ** 2 - g["rod"] ** 2) / (2 * g["rl"] * R)
    t = phi + branch * np.arccos(np.clip(c, -1, 1))
    p = np.degrees(t) - g["lug_ang"]
    return (p + 180.0) % 360.0 - 180.0


def lug_pos(P, pitch_deg):
    g = aim_geom(P)
    t = math.radians(g["lug_ang"] + pitch_deg)
    return g["rl"] * math.cos(t), g["rl"] * math.sin(t)


def crank_pin(P, psi_deg):
    g = aim_geom(P)
    t = math.radians(psi_deg)
    return g["cu"] + g["rc"] * math.cos(t), g["cw"] + g["rc"] * math.sin(t)


def aim_sweep(P, n=721):
    psi = np.linspace(0, 360, n)
    p = pitch_of_psi(P, psi, branch=+1)
    return psi, p


def wiper_per_crank(P):
    """Backward wiper degrees per crank degree."""
    d = lp(P)
    return d["worm_wheel_z"] / d["worm_starts"] / d["bevel_up"] / d["split_ratio"]


def transmission_angle(P, psi_deg, branch=+1):
    """Angle between conrod and lug arm (deg); 90 is ideal, keep within 90 +- 50."""
    p = pitch_of_psi(P, psi_deg, branch)
    psi = np.asarray(psi_deg, float)
    out = []
    for a, b in zip(np.atleast_1d(psi), np.atleast_1d(p)):
        L = np.array(lug_pos(P, b))
        C = np.array(crank_pin(P, a))
        rod = C - L
        arm = L
        tang = np.array([-arm[1], arm[0]])
        cosang = abs(rod @ tang) / (np.linalg.norm(rod) * np.linalg.norm(tang))
        out.append(90.0 - math.degrees(math.acos(min(1.0, cosang))))   # 90 = rod along the lug's motion
    return np.array(out)


def pitch_table(P, step_deg=5.0):
    """Firmware table: backward wiper degrees from the LOW dead point (pitch minimum) vs pitch, over the rising
    stroke, then the falling stroke. Wiper backward -> crank psi decreases (v10 sense, clockwise)."""
    psi, p = aim_sweep(P, 3601)
    i_lo = int(np.argmin(p))
    i_hi = int(np.argmax(p))
    wpc = wiper_per_crank(P)
    rows = []
    # crank runs psi decreasing from psi_lo
    psi_lo, psi_hi = psi[i_lo], psi[i_hi]
    for dpsi in np.arange(0, 360.0 + 1e-9, step_deg):
        ps = (psi_lo - dpsi) % 360.0
        pv = float(pitch_of_psi(P, ps, +1))
        rows.append(dict(crank_from_low=float(dpsi), psi=float(ps), wiper_back_deg=float(dpsi * wpc), pitch=pv))
    rise = (psi_lo - psi_hi) % 360.0
    return dict(pitch_min=float(p[i_lo]), pitch_max=float(p[i_hi]), psi_low=float(psi_lo), psi_high=float(psi_hi),
                crank_rise_deg=float(rise), wiper_rise_deg=float(rise * wpc), wiper_per_crank=wpc, rows=rows)


def worm_info(P):
    d = lp(P)
    lead = math.pi * d["m_worm"] * d["worm_starts"]
    lam = math.degrees(math.atan(lead / (math.pi * d["worm_pd"])))
    return dict(lead=lead, lead_angle_deg=lam, self_locking=math.tan(math.radians(lam)) < 0.12,
                wheel_pd=d["m_worm"] * d["worm_wheel_z"])


def summary(P=None):
    g = gears(P)
    t = pitch_table(P)
    return dict(stroke=stroke(P), energy_J=spring_energy(P), f_max=float(spring_force(P, lp(P)["preload"] + stroke(P))),
                f_home=float(spring_force(P, lp(P)["preload"])), gears=g, aim=dict(
                    pitch_min=t["pitch_min"], pitch_max=t["pitch_max"], crank_rise=t["crank_rise_deg"],
                    wiper_rise=t["wiper_rise_deg"], wiper_per_crank=t["wiper_per_crank"],
                    geom=aim_geom(P)), worm=worm_info(P))


if __name__ == "__main__":
    import json
    print(json.dumps(summary(None), indent=1, default=float))
