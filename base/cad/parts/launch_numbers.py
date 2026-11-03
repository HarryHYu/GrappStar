"""Print the launcher's key numbers as markdown (worker cad-launch). Run from base/:
    PYTHONPATH=. python cad/parts/launch_numbers.py
Used to keep launch_NOTES.md / launch_REPORT.md in step with the code."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.dirname(os.path.dirname(HERE))
for p in (BASE, HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

from tools.params import load_params  # noqa: E402
import launch as L                    # noqa: E402
import launch_kin as K                # noqa: E402
import launch_line as LN              # noqa: E402


def rows(P):
    n = L.L(P)
    d = K.lp(P)
    s = K.summary(P)
    g = s["gears"]
    t = L.release_table(P)
    sp = P.SPRING
    pre = d["preload"]
    ext = pre + s["stroke"]
    ps = {p.name: p for p in L.parts(P)}
    moving = ["launch_carriage", "launch_carriage_arm_l", "launch_carriage_arm_r", "launch_seat_plunger",
              "launch_seat_pushrod"]
    m_mov = sum(ps[k].mass_g() for k in moving if k in ps)
    lg = LN.geom(n)
    return [
        ("params version", getattr(P, "VERSION", "?")),
        ("springs", f"2 x {sp.get('name', '')}, k {sp['k_n_per_mm']} N/mm, F0 {sp['F0_n']} N each, max_ext {sp['max_ext']} mm"),
        ("stroke (travel at release / peak)", f"{t['travel_release']:.1f} / {max(t['travel']):.1f} mm (design value {s['stroke']:.1f})"),
        ("preload at HOME / extension at full cock", f"{pre:.1f} / {ext:.1f} mm"),
        ("force at HOME / full cock (both springs)", f"{s['f_home']:.1f} / {s['f_max']:.1f} N"),
        ("energy released", f"{s['energy_J']:.2f} J"),
        ("hook pin distance HOME / cocked", f"{n['L_home']:.1f} / {n['L_home'] + s['stroke']:.1f} mm"),
        ("big gear", f"m{g['m']} 40T: sector layer {g['n_present']} teeth (gap {g['gap_deg']:.0f} deg) + full 40T layer"),
        ("drive gear", f"{n['z_drive']}T, centre distance {g['cd']:.1f}"),
        ("wiper deg per shot / release at", f"{g['ratio'] * 360:.1f} / {-t['theta_release'] * g['ratio']:.1f} forward wiper deg from rest"),
        ("torque at full cock: big gear / wiper", f"{g['torque_big']:.2f} / {g['torque_wiper']:.2f} N m"),
        ("moving mass (carriage body + arms + seat parts)", f"{m_mov:.1f} g (dart 279 g)"),
        ("carriage rear face HOME / cocked (u)", f"{n['u_c_home']:.1f} / {n['u_c_home'] - s['stroke']:.1f}"),
        ("rail span (u) / splits", f"{n['u_track0']:.1f} .. {n['u_rail1']:.1f} / {n['split']}"),
        ("spring anchors (u, y, w)", f"{n['u_anchor']:.1f}, +-{n['y_spring']:.0f} (provision +-{L.Y_SPRING2:.0f}), {n['w_spring']:.0f}"),
        ("tether pulleys (rail frame)", f"top {tuple(round(v, 1) for v in lg['top'])}, bottom {tuple(round(v, 1) for v in lg['bot'])}, pivot {lg['piv']}"),
        ("aim", f"pitch {s['aim']['pitch_min']:.2f} .. {s['aim']['pitch_max']:.2f} deg, {s['aim']['wiper_per_crank']:.0f} wiper deg per crank deg, worm lead {s['worm']['lead_angle_deg']:.2f} deg"),
    ]


def main():
    P = load_params()
    print("| item | value |")
    print("|---|---|")
    for k, v in rows(P):
        print(f"| {k} | {v} |")


if __name__ == "__main__":
    main()
