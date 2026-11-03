"""Spring launcher: springs -> (optional pulley ratio) -> carriage -> dart, on a pitched rail.

Two spring models:
  lumped    each spring is massless plus 1/3 of its mass on the moving end (classic Rayleigh correction)
  chain     each spring is split into N masses joined by N stiff segments (captures surge, coil clash after the stop)
The chain model is the reference; `lumped` is kept as a hand check.

pulley_ratio r = carriage travel / spring extension change. r = 2 means a 2:1 block (the carriage moves twice as far
as the spring end, and feels half the force). r = 1 is direct.

python launcher.py [--pitch 35] [--stroke 0.30] [--ratio 1] [--model chain] [--sweep]
"""
from __future__ import annotations

import argparse
import json
import math
import os

import numpy as np

from common import G, Params, load_params

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "..", "out", "physics")


def reel_line_load(p: Params):
    """Line load on the dart during the stroke: (constant force N, extra effective mass kg).
    The storage reel must be spun up by the line; its inertia reflected to the line is I / r^2."""
    try:
        from payout import reel_props
        rp = reel_props(p)
        m_eff = rp["inertia_full"] / rp["r_full"] ** 2 * p.flight["eye_gain"]   # the eye amplifies the inertial pull too
    except Exception:
        m_eff = 0.010
    f = p.flight["brake_n"] * p.flight["eye_gain"]
    return f, m_eff


def launch(p: Params | None = None, pitch_deg: float | None = None, model: str = "chain", n_el: int = 16,
           dt: float = 1e-5, line_load: tuple | None = None, record: bool = False, **over):
    """Simulate one shot from full draw. Keyword overrides go into p.launch (stroke, pulley_ratio, preload_ext, ...)."""
    p = p or load_params()
    L = dict(p.launch, **over)
    th = math.radians(L["pitch_deg"] if pitch_deg is None else pitch_deg)
    sp = p.spring
    ns = p.n_springs
    r = L["pulley_ratio"]
    S = L["stroke"]
    e0 = L["preload_ext"]
    e_full = e0 + S / r
    eta = L["eta_pulley"] ** L["n_pulley_turns"]
    m_c = L["carriage_mass"]
    m_d = p.dart.mass
    f_line, m_line = line_load if line_load is not None else reel_line_load(p)
    mu = L["mu_rail"]
    cth, sth = math.cos(th), math.sin(th)

    lim = sp.limits()
    cap_mm = min(lim["ext_safe_mm"], L.get("ext_cap", 1e9) * 1000)
    res = dict(pitch_deg=math.degrees(th), stroke_m=S, pulley_ratio=r, preload_ext_mm=e0 * 1000, ext_full_mm=e_full * 1000,
               ext_safe_mm=lim["ext_safe_mm"], ext_cap_mm=cap_mm, over_safe=e_full * 1000 > cap_mm + 1e-6,
               f_spring_full_n=sp.force(e_full), f_spring_rest_n=sp.force(e0),
               over_catalogue_load=sp.force(e_full) > sp.catalogue_max_n,
               over_catalogue_defl=e_full * 1000 > sp.catalogue_max_defl_mm,
               f_carriage_full_n=ns * sp.force(e_full) * eta / r,
               energy_stored_j=ns * (sp.energy(e_full) - sp.energy(e0)), model=model)

    # friction + gravity per unit mass sliding on the rail (dart and carriage both slide)
    def resist(m):
        return m * G * (sth + mu * cth)

    trace = []
    if model == "lumped":
        m_s_eff = ns * sp.mass / 3 / r ** 2         # spring end moves 1/r of the carriage
        m_tot = m_c + m_d + m_line + m_s_eff
        x = v = t = 0.0
        a_pk = 0.0
        while x < S:
            e = e0 + (S - x) / r
            F = ns * sp.force(e) * eta / r - resist(m_c + m_d) - f_line
            a = F / m_tot
            a_pk = max(a_pk, a)
            v += a * dt
            x += v * dt
            t += dt
            if v < 0:
                res.update(fail="carriage stalled on the rail")
                return res
            if record and len(trace) < 1e6 and int(t / dt) % 50 == 0:
                trace.append((t, x, v, a))
        v_muz = v
        res.update(v_muzzle=v_muz, t_stroke_ms=t * 1000, a_peak_g=a_pk / G,
                   ke_dart_j=0.5 * m_d * v_muz ** 2, ke_carriage_j=0.5 * m_c * v_muz ** 2,
                   ke_spring_j=0.5 * m_s_eff * v_muz ** 2, v_spring_end_max=v_muz / r, coil_clash_v=0.0)
    else:
        # chain: per spring, node 0 fixed at the anchor, nodes 1..N move (spring-end coordinates, along the spring).
        # u_i = displacement of node i toward the anchor from the full-draw position (positive = shortening).
        N = n_el
        m_node = sp.mass / N
        m_masses = np.full(N + 1, m_node)
        m_masses[N] = m_node / 2                   # end node carries half a segment (consistent 1/3 mass factor)
        k_el = sp.k * N                            # each segment
        f0 = sp.f_init
        L_el_full = e_full / N                     # extension carried by each segment at full draw
        k_solid = 200.0 * k_el                     # coils touching (close wound body has zero extension at rest)
        c_el = 2 * 0.02 * math.sqrt(k_el * m_node) # light internal damping (2 percent)
        u = np.zeros(N + 1)
        w = np.zeros(N + 1)                        # node velocities (toward the anchor)
        # carriage + dart (+ reel line inertia) as one body while pushing; position along the rail from full draw
        xc = vc = 0.0
        md_attached = True
        xd = vd = 0.0
        k_cord = 3.0e5                             # cord between the spring end and the carriage, tension only
        f_full = sp.force(e_full)
        cord_pk_after_stop = 0.0
        t = 0.0
        a_pk = 0.0
        F_pk = 0.0
        clash_v = 0.0
        stopped = False
        stop_t = None
        v_muz = None
        ke_c_stop = 0.0
        ke_s_stop = 0.0
        x_stop = S
        pad = L["stop_pad_travel"]
        f_stop_pk = 0.0
        while True:
            # segment extensions (positive = stretched)
            ext = L_el_full - (u[1:] - u[:-1])
            # segment force (tension positive): initial tension acts only while coils are separated
            Fseg = np.where(ext > 0, f0 + k_el * ext, k_solid * ext)
            # damping on relative velocity
            Fseg = Fseg + c_el * (w[:-1] - w[1:])
            # node forces (+ = toward the anchor): segment i pulls node i+1 toward the anchor and node i away from it
            fn = np.zeros(N + 1)
            fn[1:] = Fseg - np.append(Fseg[1:], 0.0)
            # cord to carriage: cord length change = (spring end travel) - (carriage travel / r)
            # stiff tension-only cord, pre-stretched to carry the full-draw force at t = 0.
            # gap > 0: the spring end has shortened further than the carriage lets it (cord tighter).
            gap = u[N] - xc / r
            cord_t = max(0.0, f_full + k_cord * gap)
            if stopped:
                cord_pk_after_stop = max(cord_pk_after_stop, cord_t)
            fn[N] -= cord_t
            acc = fn / m_masses
            acc[0] = 0.0
            w += acc * dt
            u += w * dt
            u[0] = 0.0
            w[0] = 0.0
            # coil clash: any segment closed past solid with closing speed
            if (ext < 0).any():
                i = int(np.argmin(ext))
                clash_v = max(clash_v, abs(w[i + 1] - w[i]))
            # carriage (+ dart while pushing)
            if not stopped:
                m = m_c + (m_d + m_line if md_attached else 0.0)
                Fc = ns * cord_t * eta / r - resist(m_c) - (resist(m_d) + f_line if md_attached else 0.0)
                ac = Fc / m
                if md_attached:
                    a_pk = max(a_pk, ac)
                    F_pk = max(F_pk, ns * cord_t * eta / r)
                vc += ac * dt
                xc += vc * dt
                if not md_attached:
                    ad = -(resist(m_d) + f_line) / (m_d + m_line)
                    vd += ad * dt
                    xd += vd * dt
                elif ac < -(resist(m_d) + f_line) / (m_d + m_line):
                    md_attached = False            # carriage decelerates faster than the dart: dart flies off
                    xd, vd = xc, vc
                if xc >= S:
                    stopped = True
                    stop_t = t
                    ke_c_stop = 0.5 * m_c * vc ** 2
                    ke_s_stop = float(0.5 * (m_masses[1:] * w[1:] ** 2).sum()) * ns
                    # stop pad: carriage KE absorbed over the pad crush, roughly linear pad -> peak = 2 KE / travel
                    # pad absorbs the carriage KE plus the spring KE that arrives through the cord (upper bound)
                    f_stop_pk = 2 * (ke_c_stop + ke_s_stop) / pad if pad > 0 else float("inf")
                    if md_attached:
                        v_muz = vc
                        md_attached = False
                        xd, vd = xc, vc
                    vc = 0.0
            else:
                if v_muz is None:
                    ad = -(resist(m_d) + f_line) / (m_d + m_line)
                    vd += ad * dt
                    xd += vd * dt
            if v_muz is None and not md_attached and xd >= S:
                v_muz = vd
            if record and int(round(t / dt)) % 100 == 0:
                trace.append((t, xc, vc, float(u[N]), float(cord_t)))
            t += dt
            if vc < 0 and not stopped and t > 1e-3:
                res.update(fail="carriage stalled on the rail")
                return res
            if stopped and v_muz is not None and t > stop_t + 0.08:
                break
            if t > 1.0:
                res.update(fail="timeout")
                return res
        res.update(v_muzzle=v_muz, t_stroke_ms=stop_t * 1000, a_peak_g=a_pk / G, f_carriage_peak_n=F_pk,
                   ke_dart_j=0.5 * m_d * v_muz ** 2, ke_carriage_j=ke_c_stop, ke_spring_j=ke_s_stop,
                   coil_clash_v=clash_v, f_stop_peak_n=f_stop_pk,
                   f_cord_peak_after_stop_n=cord_pk_after_stop * ns * eta / r,
                   v_spring_end_max=float(np.abs(w).max()))
    res["efficiency"] = res["ke_dart_j"] / res["energy_stored_j"]
    res["line_load"] = dict(force_n=f_line, mass_eff_kg=m_line)
    if model == "lumped":
        res["f_stop_peak_n"] = 2 * (res["ke_carriage_j"] + res["ke_spring_j"]) / L["stop_pad_travel"]
    if record:
        res["trace"] = trace
    return res


def ratio_sweep(p: Params, stroke: float, ratios=(0.5, 0.75, 1.0, 1.5, 2.0, 3.0), pitch=35.0, cap_safe=True):
    """For a fixed carriage stroke, what pulley ratio gives the fastest shot without exceeding the safe extension?"""
    rows = []
    lim = min(p.spring.limits()["ext_safe_mm"] / 1000, p.launch.get("ext_cap", 1e9))
    for r in ratios:
        e0 = p.launch["preload_ext"]
        S = stroke
        if cap_safe and e0 + S / r > lim:
            S = (lim - e0) * r                     # rail longer than the springs can use at this ratio
        res = launch(p, pitch, stroke=S, pulley_ratio=r)
        rows.append(dict(ratio=r, stroke_used_m=S, ext_full_mm=res["ext_full_mm"], v_muzzle=res.get("v_muzzle"),
                         energy_j=res["energy_stored_j"], efficiency=res.get("efficiency"),
                         f_carriage_full_n=res["f_carriage_full_n"]))
    return rows


def best_stroke(p: Params, pitch=35.0, ratio=1.0):
    """Stroke that uses the springs to their safe extension at this ratio."""
    lim = min(p.spring.limits()["ext_safe_mm"] / 1000, p.launch.get("ext_cap", 1e9))
    return (lim - p.launch["preload_ext"]) * ratio


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--pitch", type=float, default=None)
    ap.add_argument("--stroke", type=float, default=None)
    ap.add_argument("--ratio", type=float, default=None)
    ap.add_argument("--model", default="chain", choices=["chain", "lumped"])
    ap.add_argument("--sweep", action="store_true", help="pulley ratio sweep at the given stroke")
    ap.add_argument("--json", default=os.path.join(OUT_DIR, "launch.json"))
    a = ap.parse_args(argv)
    p = load_params()
    over = {}
    if a.stroke is not None:
        over["stroke"] = a.stroke
    if a.ratio is not None:
        over["pulley_ratio"] = a.ratio
    res = launch(p, a.pitch, model=a.model, **over)
    sp = p.spring
    lim = sp.limits()
    print(f"params: {p.source}")
    print(f"spring: k {sp.k / 1000:.3f} N/mm (formula {sp.rate_formula:.3f}), Fi {sp.f_init} N, body coils {sp.n_body:.1f}, "
          f"mass {sp.mass * 1000:.1f} g, surge {sp.surge()['f_fixed_fixed_hz']:.1f} Hz")
    print(f"spring safe: {lim['f_safe_n']:.1f} N ({lim['governed_by']}), extension {lim['ext_safe_mm']:.0f} mm, "
          f"Sut {lim['sut_mpa']:.0f} MPa")
    for k in ("pitch_deg", "stroke_m", "pulley_ratio", "ext_full_mm", "over_safe", "f_carriage_full_n", "energy_stored_j",
              "f_spring_full_n", "over_catalogue_load", "over_catalogue_defl", "v_muzzle", "t_stroke_ms", "a_peak_g", "ke_dart_j", "efficiency", "ke_carriage_j", "f_stop_peak_n",
              "ke_spring_j", "f_cord_peak_after_stop_n", "coil_clash_v", "fail"):
        if k in res:
            v = res[k]
            print(f"  {k:20s} {v:.3f}" if isinstance(v, float) else f"  {k:20s} {v}")
    out = dict(params_source=p.source, spring=dict(k_n_mm=sp.k / 1000, rate_formula_n_mm=sp.rate_formula, f_init=sp.f_init,
               n_body=sp.n_body, mass_kg=sp.mass, limits=lim, surge=sp.surge()), shot=res)
    if a.sweep:
        S = res["stroke_m"]
        out["ratio_sweep"] = ratio_sweep(p, S, pitch=res["pitch_deg"])
        print("ratio sweep at stroke", S)
        for row in out["ratio_sweep"]:
            print("  ", {k: (round(v, 3) if isinstance(v, float) else v) for k, v in row.items()})
    os.makedirs(os.path.dirname(a.json), exist_ok=True)
    json.dump(out, open(a.json, "w"), indent=1)
    return out


if __name__ == "__main__":
    main()
