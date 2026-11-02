"""Dart flight with drag and a tether paid off the storage reel.

The dart is a point mass with drag area CdA (KEDGE v30 folded, 15 g ballast: the kedge pitch-plane sim shows it flies
nose first with the tether on, so the point-mass model holds). The tether is a bead chain: line material leaves the
guide eye, beads are inserted at the eye as line pays out, every segment has cross-flow and skin-friction drag and
weight, beads that touch the ground stop there. Line tension at the eye comes from either
  reel_model=False   an ideal brake: the eye lets line go at exactly brake_n x eye_gain (no reel inertia)
  reel_model=True    the real storage reel (payout.Reel): inertia, slip-clutch torque, optional centrifugal shoes
Range is measured on level ground from the muzzle's ground point.

python flight.py [--v 8.7] [--pitch 40] [--brake 0.75] [--sweep] [--reel]
"""
from __future__ import annotations

import argparse
import json
import math
import os

import numpy as np

from common import G, RHO_AIR, Params, load_params

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "..", "out", "physics")


def simulate(p: Params, v0: float, pitch_deg: float, brake_n: float | None = None, tether: bool = True,
             reel_model: bool = False, seg: float = 0.40, ea_sim: float = 3000.0, dt: float = 1e-4,
             cda: float | None = None, wind: float = 0.0, record: bool = False, shoes: bool = False,
             shoe_k: float = 0.0, shoe_on: float = 0.0, tmax: float = 6.0, muzzle_xz: tuple | None = None):
    """One flight. wind: horizontal air speed in +x (a tailwind is positive)."""
    brake = p.flight["brake_n"] if brake_n is None else brake_n
    gain = p.flight["eye_gain"]
    lam, dl = p.line.lam, p.line.d
    cdn, cfa = p.line.cd_normal, p.line.cf_axial
    md = p.dart.mass
    cda = p.dart.cda if cda is None else cda
    th = math.radians(pitch_deg)
    # geometry: eye at x 0 (reel side), muzzle ahead of it
    eye = np.array([0.0, p.flight["reel_height"]])
    if muzzle_xz is None:
        muzzle = np.array([0.10, p.launch["muzzle_height"]])
    else:
        muzzle = np.array(muzzle_xz, float)
    air = np.array([wind, 0.0])
    vd0 = v0 * np.array([math.cos(th), math.sin(th)])

    nodes = muzzle[None, :].copy()           # last node is the dart
    vel = vd0[None, :].copy()
    d0 = float(np.linalg.norm(muzzle - eye))
    l0 = d0                                   # rest length of segment 0 (eye -> first node)
    reel = None
    if tether and reel_model:
        from payout import Reel
        reel = Reel(p, brake, shoes=shoes, shoe_k=shoe_k, shoe_on=shoe_on)
        reel.paid = d0
        reel.omega = v0 / reel.radius()
    paid = d0
    m_b = lam * seg
    k_seg = ea_sim / seg
    dt = min(dt, 0.55 * math.sqrt(m_b / k_seg))   # explicit stability on the stiffest segment
    t = 0.0
    apex = muzzle[1]
    t_dart_max = 0.0
    t_at_max = 0.0
    t_dart_f = 0.0
    t_reel_max = 0.0
    slack_max = 0.0
    hist = []
    hist_reel = []
    line_end_hit = False
    landed = False
    pay_rate = v0
    while t < tmax:
        n = len(nodes)
        P = np.vstack([eye, nodes])
        dif = P[1:] - P[:-1]
        ln = np.linalg.norm(dif, axis=1)
        ln = np.maximum(ln, 1e-6)
        u = dif / ln[:, None]
        rest = np.full(n, seg)
        rest[0] = max(l0, 1e-4)
        if tether:
            V = np.vstack([pay_rate * u[0], vel])
            dv = V[1:] - V[:-1]
            rel_axial = (dv * u).sum(1)
            k_i = ea_sim / rest
            T = k_i * (ln - rest)
            cd = 2 * 0.5 * np.sqrt(k_i * m_b)
            T = np.where(T > 0, T + cd * rel_axial, 0.0)
            T = np.maximum(T, 0.0)
            if reel is None:
                # ideal brake: segment 0 pays out to hold tension at brake x gain
                tb = brake * gain
                if T[0] > tb:
                    new_l0 = ln[0] / (1 + tb / ea_sim)
                    if new_l0 > l0:
                        pay_rate = 0.9 * pay_rate + 0.1 * (new_l0 - l0) / dt
                        paid += new_l0 - l0
                        l0 = new_l0
                    T[0] = tb
                else:
                    pay_rate *= 0.9
            # forces on nodes
            F = np.zeros_like(nodes)
            F -= T[:, None] * u                   # segment i pulls node i (P index i+1) back toward P index i
            F[:-1] += T[1:, None] * u[1:]         # segment i+1 pulls node i forward
            # line drag per segment, half to each end
            Vm = 0.5 * (V[1:] + V[:-1]) - air
            vt = (Vm * u).sum(1)
            vn = Vm - vt[:, None] * u
            vnm = np.linalg.norm(vn, axis=1)
            Fd = -0.5 * RHO_AIR * dl * ln[:, None] * (cdn * vnm[:, None] * vn + math.pi * cfa * np.abs(vt)[:, None] * vt[:, None] * u)
            F += 0.5 * Fd
            F[:-1] += 0.5 * Fd[1:]
            mass = np.full(n, m_b)
            mass[-1] = md + 0.5 * m_b
            F[:, 1] -= mass * G
            t_dart_now = float(T[-1])
            t_dart_f += (t_dart_now - t_dart_f) * dt / 0.005   # 5 ms filter: bead insertion jolts are numerical
            if t > 0.02:                          # skip the start-up transient of the chain
                if t_dart_f > t_dart_max:
                    t_dart_max, t_at_max = t_dart_f, t
            t_eye = float(T[0])
        else:
            F = np.zeros_like(nodes)
            mass = np.array([md])
            F[:, 1] -= md * G
            t_eye = 0.0
            t_dart_now = 0.0
        # dart drag
        vrel = vel[-1] - air
        sp = float(np.linalg.norm(vrel))
        F[-1] -= 0.5 * RHO_AIR * cda * sp * vrel
        vel += F / mass[:, None] * dt
        nodes += vel * dt
        # ground for line beads
        if n > 1:
            g_ = nodes[:-1, 1] < 0.0
            if g_.any():
                nodes[:-1][g_, 1] = 0.0
                vel[:-1][g_] = 0.0
        # reel
        if reel is not None:
            t_reel = t_eye / gain
            t_reel_max = max(t_reel_max, t_reel)
            dlp = reel.step(t_reel, dt)
            l0 += dlp
            paid += dlp
            pay_rate = dlp / dt
            slack_max = max(slack_max, l0 - float(np.linalg.norm(nodes[0] - eye)))
            if reel.paid >= reel.L_stored and not line_end_hit:
                line_end_hit = True
        elif tether and paid >= p.line.length:
            line_end_hit = True
        # insert a bead at the eye when segment 0 gets long
        if tether and l0 > 2 * seg:
            frac = (l0 - seg) / l0
            pos = eye + (nodes[0] - eye) * frac
            ve = pay_rate * u[0]
            vnew = ve + frac * (vel[0] - ve)      # material between the eye and node 0: interpolate
            nodes = np.vstack([pos, nodes])
            vel = np.vstack([vnew, vel])
            l0 -= seg
        apex = max(apex, float(nodes[-1, 1]))
        t += dt
        if record and int(round(t / dt)) % 100 == 0:
            hist.append((round(t, 4), float(nodes[-1, 0]), float(nodes[-1, 1]), float(np.linalg.norm(vel[-1])),
                         t_eye, t_dart_now))
            if reel is not None:
                hist_reel.append((round(t, 4), reel.omega * 60 / (2 * math.pi), t_eye / gain, reel.paid,
                                  max(0.0, l0 - float(np.linalg.norm(nodes[0] - eye)))))
        if nodes[-1, 1] <= 0.0:
            landed = True
            break
    vimp = vel[-1]
    res = dict(v0=v0, pitch_deg=pitch_deg, brake_n=brake if tether else 0.0, tether=tether, reel_model=reel_model,
               landed=landed, range_m=float(nodes[-1, 0] - muzzle[0]), x_from_eye_m=float(nodes[-1, 0]),
               t_flight_s=t, apex_m=apex, v_impact=float(np.linalg.norm(vimp)),
               impact_angle_deg=float(math.degrees(math.atan2(-vimp[1], vimp[0]))),
               line_paid_m=paid, t_dart_max_n=t_dart_max, t_at_max=t_at_max, line_end_hit=line_end_hit, cda=cda, wind=wind)
    if reel is not None:
        res["reel_rpm_at_landing"] = reel.omega * 60 / (2 * math.pi)
        # after landing the dart stops taking line; the reel spins down on its brake (+ shoes) and throws slack
        w = reel.omega
        r = reel.radius()
        I = reel.inertia()
        over = 0.0
        ts = 0.0
        h = 1e-4
        rpm_max = max([row[1] for row in hist_reel] + [w * 60 / (2 * math.pi), v0 / reel.r_full * 60 / (2 * math.pi)])
        while w > 0 and ts < 20:
            reel.omega = w
            w = max(0.0, w - reel.resist_torque() / I * h)
            over += w * r * h
            ts += h
        res.update(reel_rpm_max=rpm_max, overrun_after_landing_m=over, reel_stop_after_landing_s=ts,
                   t_reel_max_n=t_reel_max, slack_max_m=max(0.0, slack_max))
    if record:
        res["history"] = hist
        res["history_reel"] = hist_reel
    return res


def range_vs_pitch(p, v_of_pitch, pitches, brakes, **kw):
    rows = []
    for th in pitches:
        v = v_of_pitch(th)
        row = dict(pitch_deg=th, v_muzzle=v, range_m=[], t_flight_s=[], apex_m=[], v_impact=[], impact_angle_deg=[])
        for b in brakes:
            r = simulate(p, v, th, brake_n=b, **kw)
            for k in ("range_m", "t_flight_s", "apex_m", "v_impact", "impact_angle_deg"):
                row[k].append(r[k])
        rows.append(row)
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--v", type=float, default=None, help="muzzle speed m/s (default: launch.py at each pitch)")
    ap.add_argument("--pitch", type=float, default=40.0)
    ap.add_argument("--brake", type=float, default=None)
    ap.add_argument("--reel", action="store_true", help="couple the real reel instead of an ideal brake")
    ap.add_argument("--sweep", action="store_true", help="pitch sweep and brake sensitivity")
    ap.add_argument("--json", default=os.path.join(OUT_DIR, "flight.json"))
    a = ap.parse_args(argv)
    p = load_params()
    from launch import launch
    vcache = {}

    def v_of(th):
        if a.v is not None:
            return a.v
        if th not in vcache:
            vcache[th] = launch(p, th)["v_muzzle"]
        return vcache[th]

    out = dict(params_source=p.source)
    r = simulate(p, v_of(a.pitch), a.pitch, a.brake, reel_model=a.reel, record=True)
    out["shot"] = r
    print(f"params: {p.source}")
    print(f"pitch {a.pitch} v0 {r['v0']:.2f}: range {r['range_m']:.2f} m, apex {r['apex_m']:.2f} m, t {r['t_flight_s']:.2f} s, "
          f"impact {r['v_impact']:.2f} m/s at {r['impact_angle_deg']:.1f} deg, line out {r['line_paid_m']:.1f} m, "
          f"max line pull on dart {r['t_dart_max_n']:.2f} N")
    nt = simulate(p, r["v0"], a.pitch, tether=False)
    vac = simulate(p, r["v0"], a.pitch, tether=False, cda=0.0)
    print(f"  same shot without tether {nt['range_m']:.2f} m, vacuum {vac['range_m']:.2f} m")
    out["no_tether_m"] = nt["range_m"]
    out["vacuum_m"] = vac["range_m"]
    if a.sweep:
        pitches = list(range(int(p.flight["pitch_min"]), int(p.flight["pitch_max"]) + 1, 5))
        brakes = [0.0, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0]
        rows = range_vs_pitch(p, v_of, pitches, [0.5, 0.75, 1.0])
        out["pitch_sweep"] = rows
        print("pitch  v0    range@0.5/0.75/1.0 N      t_flight   apex   impact v / angle (0.75 N)")
        for row in rows:
            print(f"{row['pitch_deg']:5.0f} {row['v_muzzle']:5.2f}  " + " ".join(f"{x:5.2f}" for x in row["range_m"]) +
                  f"   {row['t_flight_s'][1]:5.2f}  {row['apex_m'][1]:5.2f}  {row['v_impact'][1]:5.2f} / {row['impact_angle_deg'][1]:5.1f}")
        best = max(rows, key=lambda q: q["range_m"][1])
        sens = []
        for b in brakes:
            rr = simulate(p, best["v_muzzle"], best["pitch_deg"], brake_n=b, tether=b > 0 or True)
            sens.append(dict(brake_n=b, range_m=rr["range_m"]))
        out["brake_sensitivity"] = dict(pitch_deg=best["pitch_deg"], rows=sens)
        print(f"brake sensitivity at pitch {best['pitch_deg']}: " + ", ".join(f"{s['brake_n']} N {s['range_m']:.2f} m" for s in sens))
        cs = []
        for c in p.dart.cda_range:
            rr = simulate(p, best["v_muzzle"], best["pitch_deg"], cda=c)
            cs.append(dict(cda=c, range_m=rr["range_m"]))
        out["cda_sensitivity"] = cs
        print("CdA sensitivity: " + ", ".join(f"{c['cda'] * 1e6:.0f} mm2 {c['range_m']:.2f} m" for c in cs))
    os.makedirs(os.path.dirname(a.json), exist_ok=True)
    json.dump(out, open(a.json, "w"), indent=1)
    return out


if __name__ == "__main__":
    main()
