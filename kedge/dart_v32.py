"""v32 extras: tail boom and tether eye strength (tow 3 x 55 N, launch, landing), fins and flight without the tether.

  python dart_v32.py OUT   -> OUT/phys31/v32_checks.json
Boom sections are sliced from the tail_boom mesh (shapely, holes subtracted) every 2 mm from the tail cap face to the eye.
Tow case: 3 x the 55 N cap at the eye, along the line from the worst rover position: 1 m away in line height (line 21 deg up)
and 30 deg to the side, so 36 deg off the boom axis. PPA-CF printed along the boom: 80 MPa along the lines (dart_v32
assumption, Bambu PPA-CF XY tensile about 100 MPa), 30 MPa across the layers.
"""
import json, math, os, sys
import numpy as np
import dart_model as dm

SIG_ALONG, SIG_ACROSS = 80e6, 30e6
F_TOW = 3 * 55.0
OFF_AXIS_DEG = math.degrees(math.acos(math.cos(math.radians(21.1)) * math.cos(math.radians(30.0))))


def section(m, x):
    s = m.section(plane_origin=[x, 0, 0], plane_normal=[1, 0, 0])
    if s is None:
        return None
    pl, T = s.to_2D()
    polys = pl.polygons_full
    if not polys:
        return None
    from shapely.ops import unary_union
    poly = unary_union(polys)
    # section frame: to_2D maps (y, z) to some in-plane axes; second moments about the centroid in that frame
    c = np.array(poly.centroid.coords[0])

    def ring_I(coords):
        p = np.array(coords) - c
        x_, y_ = p[:, 0], p[:, 1]
        cr = x_[:-1] * y_[1:] - x_[1:] * y_[:-1]
        Ixx = np.sum((y_[:-1] ** 2 + y_[:-1] * y_[1:] + y_[1:] ** 2) * cr) / 12
        Iyy = np.sum((x_[:-1] ** 2 + x_[:-1] * x_[1:] + x_[1:] ** 2) * cr) / 12
        return Ixx, Iyy
    geoms = getattr(poly, "geoms", [poly])
    Ixx = Iyy = 0.0; ymax = xmax = 0.0
    for g in geoms:
        a, b = ring_I(g.exterior.coords); Ixx += abs(a); Iyy += abs(b)
        for h in g.interiors:
            a, b = ring_I(h.coords); Ixx -= abs(a); Iyy -= abs(b)
        p = np.array(g.exterior.coords) - c
        ymax = max(ymax, np.abs(p[:, 1]).max()); xmax = max(xmax, np.abs(p[:, 0]).max())
    return dict(A=poly.area, W_min=min(Ixx / ymax, Iyy / xmax), I_min=min(Ixx, Iyy))


def boom_check(d):
    names = [n for n in d.PP if "boom" in n and not d.PP[n].get("parent")]
    if "tail_boom" not in d.PP:
        return None
    m = d.mesh("tail_boom")
    x_eye = d.tether_pt[0]
    x0 = max(float(m.bounds[0, 0]) + 0.5, d.P.get("L", 152.0) - 0.5)
    rows = []
    for x in np.arange(x0, x_eye - 1.0, 1.0):
        s = section(m, x)
        if s is None:
            continue
        rows.append((float(x), s["A"], s["W_min"], s["I_min"]))
    th = math.radians(OFF_AXIS_DEG)
    F_lat, F_ax = F_TOW * math.sin(th), F_TOW * math.cos(th)
    worst = None
    for x, A, W, I in rows:
        M = F_lat * (x_eye - x) / 1000
        sig = M / (W * 1e-9) + F_ax / (A * 1e-6)
        if worst is None or sig > worst[1]:
            worst = (x, sig, A, W, M)
    # eye: the smallest section in the last 12 mm carries the pin hole: net tension under the full axial share
    tail = [r for r in rows if r[0] > x_eye - 12]
    eye = min(tail, key=lambda r: r[1]) if tail else None
    sig_eye = F_TOW / (eye[1] * 1e-6) if eye else None
    mb = d.part_mass("tail_boom", m)
    E = 8e9                                                   # PPA-CF printed, along the lines (assumed)
    Lb = (x_eye - x0) / 1000
    Imin = min(r[3] for r in rows) * 1e-12 if rows else 0
    return dict(parts=names, x_root_mm=round(x0, 1), x_eye_mm=round(x_eye, 1), free_length_mm=round(x_eye - x0, 1), mass_g=round(mb, 1),
                tow_case=dict(F_N=F_TOW, off_axis_deg=round(OFF_AXIS_DEG, 1), lateral_N=round(F_lat, 1), axial_N=round(F_ax, 1)),
                worst_section=dict(x_mm=round(worst[0], 1), stress_MPa=round(worst[1] / 1e6, 1), area_mm2=round(worst[2], 1), W_mm3=round(worst[3], 1),
                                   moment_Nm=round(worst[4], 2), SF_along_lines=round(SIG_ALONG / worst[1], 2), SF_across_layers=round(SIG_ACROSS / worst[1], 2)) if worst else None,
                eye_net_section=dict(x_mm=round(eye[0], 1), area_mm2=round(eye[1], 1), tension_MPa_at_3x55N=round(sig_eye / 1e6, 1),
                                     SF=round(SIG_ALONG / sig_eye, 2)) if eye else None,
                launch=dict(inertia_N=round(mb / 1000 * 18.9 * 9.81, 2), line_spin_up_pull_N=8.9,
                            buckling_if_pushed_N=round(math.pi ** 2 * E * Imin / (4 * Lb ** 2), 0) if Lb > 0 else None,
                            note="the launcher pushes the tail cap face; if the pusher bears on the eye instead, 60 N compression against the buckling load"),
                landing_tip_load_capacity_N=round(SIG_ALONG * worst[3] * 1e-9 / ((x_eye - worst[0]) / 1000), 0) if worst else None)


def fins_flight(d, out):
    """Aerodynamic centre and a no-tether flight with this build's fins (flight_sim strip model + fin slope)."""
    import flight_sim as fs
    import physics
    R = dict(fins=d.fins())
    R["cp_pitch_x"] = round(physics.aero_centre(d, "pitch"), 1)
    R["cp_yaw_x"] = round(physics.aero_centre(d, "yaw"), 1)
    R["cg_x"] = round(float(d.mp["cg"][0]), 1)
    D = 2 * d.P["R_OUT"]
    R["static_margin_cal"] = dict(pitch=round((R["cp_pitch_x"] - R["cg_x"]) / D, 2), yaw=round((R["cp_yaw_x"] - R["cg_x"]) / D, 2))
    p = dm.base_params()
    from launcher import launch
    for plane in ("pitch", "yaw"):
        b = fs.Body(d, d.ballast_g, plane)
        p.dart.mass = b.M
        v0 = float(launch(p, 34)["v_muzzle"])
        o = fs.run(b, v0, 34, None) if plane == "pitch" else fs.run_yaw(b, v0, None, 0.9)
        late = o[o[:, 0] > 0.15]
        R[f"no_tether_{plane}"] = dict(alpha_peak_deg=round(float(np.abs(late[:, 3]).max()), 1), alpha_end_deg=round(float(o[-1, 3]), 1),
                                       nose_first=bool(np.abs(o[:, 3]).max() < 45))
    return R


def run(out):
    d = dm.Dart(out)
    R = dict(source=d.out, tether_pt_mm=d.tether_pt, boom=boom_check(d), flight=fins_flight(d, out))
    os.makedirs(os.path.join(d.out, "phys31"), exist_ok=True)
    json.dump(R, open(os.path.join(d.out, "phys31", "v32_checks.json"), "w"), indent=1, default=float)
    return R


if __name__ == "__main__":
    print(json.dumps(run(sys.argv[1]), indent=1, default=float))
