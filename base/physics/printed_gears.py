"""Strength checks for the printed drive parts: Lewis bending on spur teeth, key and keyway, printed shaft torsion,
608 bearing, CSK20PP sprag. Pure functions; every result is a dict with a `pass` bool.

Lewis form factor Y: Shigley 10th ed. Table 14-2 (20 deg full depth, Y for sigma = Kv Wt / (F m Y)), linear
interpolation in N. The formula Y = 0.154 - 0.912/N is the pi-free (diametral pitch) form and is not used here.
Velocity factor: Barth, cut or milled profile, Kv = (6.1 + V) / 6.1, V pitch line speed in m/s (printed teeth are
treated like cut teeth: rough flanks, small errors).
"""
from __future__ import annotations

import math

from common import PPA_CF, Printed, Bearing608, Sprag, Shaft

# Shigley Table 14-2, 20 deg full depth
_Y_TABLE = [
    (12, 0.245), (13, 0.261), (14, 0.277), (15, 0.290), (16, 0.296), (17, 0.303), (18, 0.309), (19, 0.314),
    (20, 0.322), (21, 0.328), (22, 0.331), (24, 0.337), (26, 0.346), (28, 0.353), (30, 0.359), (34, 0.371),
    (38, 0.384), (43, 0.397), (50, 0.409), (60, 0.422), (75, 0.435), (100, 0.447), (150, 0.460), (300, 0.472),
    (400, 0.480), (100000, 0.485),
]


def lewis_y(teeth: float) -> float:
    """Lewis form factor, 20 deg full depth, from the table. Below 12 teeth: undercut, not allowed."""
    if teeth < 12:
        raise ValueError("fewer than 12 teeth undercuts at 20 deg full depth")
    for (n0, y0), (n1, y1) in zip(_Y_TABLE, _Y_TABLE[1:]):
        if n0 <= teeth <= n1:
            return y0 + (y1 - y0) * (teeth - n0) / (n1 - n0)
    return _Y_TABLE[-1][1]


def barth_kv(v_pitch: float) -> float:
    return (6.1 + v_pitch) / 6.1


def lewis(torque_nm: float, module_mm: float, teeth: int, face_mm: float, material: Printed = PPA_CF,
          rpm: float = 0.0) -> dict:
    """Lewis root bending for the gear carrying torque_nm with `teeth` teeth."""
    m = module_mm * 1e-3
    b = face_mm * 1e-3
    d = m * teeth
    wt = abs(torque_nm) / (d / 2)
    v = math.pi * d * rpm / 60
    kv = barth_kv(v)
    y = lewis_y(teeth)
    sigma = kv * wt / (b * m * y)
    sf = material.sigma_lewis / sigma if sigma > 0 else math.inf
    return dict(torque_nm=torque_nm, module_mm=module_mm, teeth=teeth, face_mm=face_mm, material=material.name,
                pitch_d_mm=d * 1e3, wt_n=wt, v_pitch=v, kv=kv, y=y, sigma_mpa=sigma / 1e6,
                allow_mpa=material.sigma_lewis / 1e6, sf=sf, **{"pass": sf >= 1.0})


def key(torque_nm: float, length_mm: float = 14.0, shaft: Shaft = Shaft(), key_h_mm: float = 6.0,
        material: Printed = PPA_CF, key_material: str = "steel", hub_printed: bool = True, hub_r_mm: float = 20.0,
        bearing_allow: float | None = None) -> dict:
    """Square key, half its height in the printed shaft keyway and half in the hub keyway. Length default = sprag
    width 14 mm. Printed failure modes:
      bearing_shaft / bearing_hub: keyway side wall, F / ((h/2) L)
      shear_shaft: the plastic behind the shaft keyway side wall shears out on the plane through the keyway bottom
        corner, parallel to the force, from the corner to the shaft surface: a = sqrt(R^2 - (R - h/2)^2) - w/2
      shear_hub: same behind the hub keyway, plane through its top corner out to the hub radius hub_r_mm
      shear_key: only when key_material is "printed" (a steel key does not govern)
    Bearing allowable defaults to material.sigma_lewis (conservative stand-in for printed compression with creep)."""
    L = length_mm * 1e-3
    w = shaft.key_w
    t = key_h_mm * 1e-3 / 2
    R = shaft.d / 2
    f = abs(torque_nm) / R
    b_allow = material.sigma_lewis if bearing_allow is None else bearing_allow
    modes = {}
    modes["bearing_shaft"] = (f / (t * L), b_allow)
    a_s = math.sqrt(R ** 2 - (R - t) ** 2) - w / 2
    modes["shear_shaft"] = (f / (a_s * L), material.tau_allow)
    if hub_printed:
        modes["bearing_hub"] = (f / (t * L), b_allow)
        rh = hub_r_mm * 1e-3
        a_h = math.sqrt(max(rh ** 2 - (R + t) ** 2, 0.0)) - w / 2
        modes["shear_hub"] = (f / (a_h * L) if a_h > 0 else math.inf, material.tau_allow)
    if key_material == "printed":
        modes["shear_key"] = (f / (w * L), material.tau_allow)
    out = {}
    for k, (stress, allow) in modes.items():
        out[k] = dict(stress_mpa=stress / 1e6, allow_mpa=allow / 1e6, sf=allow / stress if stress > 0 else math.inf)
    gov = min(out, key=lambda k: out[k]["sf"])
    sf = out[gov]["sf"]
    return dict(torque_nm=torque_nm, length_mm=length_mm, force_n=f, key_material=key_material,
                hub_printed=hub_printed, ligament_shaft_mm=a_s * 1e3, modes=out, governing=gov, sf=sf,
                t_max=abs(torque_nm) * sf if torque_nm else math.inf, **{"pass": sf >= 1.0})


def shaft_torsion(torque_nm: float, shaft: Shaft = Shaft(), material: Printed = PPA_CF, kt: float = 2.0,
                  kt_endmill: float = 3.0) -> dict:
    """tau = 16 T / (pi d^3). Three verdicts:
      pass:        T <= Shaft.t_allow (the agreed 12 N m)
      strict_pass: Kt 2 keyway (rounded, sled runner style), Kt x tau <= material.tau_allow (about 9.4 N m)
      endmill_pass: end-milled keyseat, Kt about 3 (about 6.3 N m)"""
    tau = 16 * abs(torque_nm) / (math.pi * shaft.d ** 3)
    base = material.tau_allow * math.pi * shaft.d ** 3 / 16
    t_kt, t_em = base / kt, base / kt_endmill
    T = abs(torque_nm)
    return dict(torque_nm=torque_nm, tau_nom_mpa=tau / 1e6, tau_peak_mpa=kt * tau / 1e6, kt=kt, kt_endmill=kt_endmill,
                t_allow=shaft.t_allow, t_allow_kt=t_kt, t_allow_endmill=t_em,
                sf=shaft.t_allow / T if T else math.inf, strict_pass=T <= t_kt, endmill_pass=T <= t_em,
                **{"pass": T <= shaft.t_allow})


def bearing608(radial_n: float, rpm: float, b: Bearing608 = Bearing608(), sealed: bool = False,
               s0_min: float = 1.0) -> dict:
    """Static safety C0/P0 (>= 1 normal duty), speed limit, L10 life in revolutions and hours at that rpm."""
    p = abs(radial_n)
    s0 = b.C0 / p if p > 0 else math.inf
    n_max = b.n_max_rpm_2rs if sealed else b.n_max_rpm_zz
    l10_rev = (b.C_dyn / p) ** 3 * 1e6 if p > 0 else math.inf
    l10_h = l10_rev / (rpm * 60) if rpm > 0 else math.inf
    return dict(radial_n=p, rpm=rpm, s0=s0, n_max_rpm=n_max, l10_rev=l10_rev, l10_h=l10_h,
                **{"pass": s0 >= s0_min and rpm <= n_max})


def sprag(torque_nm: float, rpm: float, s: Sprag = Sprag()) -> dict:
    return dict(torque_nm=torque_nm, rpm=rpm, t_rated=s.t_rated, n_max_rpm=s.n_max_rpm,
                sf=s.t_rated / abs(torque_nm) if torque_nm else math.inf,
                **{"pass": abs(torque_nm) <= s.t_rated and rpm <= s.n_max_rpm})


def mesh(torque_driver: float, ratio: float, module_mm: float, teeth_driver: int, face_mm: float,
         material: Printed = PPA_CF, rpm_driver: float = 0.0) -> dict:
    """One spur mesh. ratio = driven teeth / driver teeth. Checks both gears; the governing one is the lower sf."""
    n2 = max(12, round(teeth_driver * ratio))
    a = lewis(torque_driver, module_mm, teeth_driver, face_mm, material, rpm_driver)
    g = lewis(torque_driver * n2 / teeth_driver, module_mm, n2, face_mm, material, rpm_driver * teeth_driver / n2)
    gov = a if a["sf"] <= g["sf"] else g
    return dict(driver=a, driven=g, sf=gov["sf"], **{"pass": a["pass"] and g["pass"]})


if __name__ == "__main__":
    for t in (1.0, 5.0, 12.0):
        r = lewis(t, 2.0, 20, 12.0, PPA_CF, 45)
        print(f"T {t:5.1f} N m  m2 z20 b12 PPA-CF: sigma {r['sigma_mpa']:.1f} MPa  sf {r['sf']:.2f}  pass {r['pass']}")
    k = key(12.0)
    print(f"key 12 N m: governing {k['governing']} sf {k['sf']:.2f}, max {k['t_max']:.1f} N m")
    print("shaft 12 N m:", {k: round(v, 2) if isinstance(v, float) else v for k, v in shaft_torsion(12.0).items()})
