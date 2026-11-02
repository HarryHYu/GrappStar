"""Face dog clutch and printed friction slip clutch. Units mm, N, N m, MPa, degrees.

Dog clutch frame (each half, identical shape): axis +Z, base disc z = [-face_thick, 0], teeth on the +Z face
z = [0, tooth_h]. The driven half is the same part flipped face down: place it in the driver frame with
dog_mate(info, e). The driver drives by turning +Z (right hand); its drive flank is the leading (+angle) side of
each tooth.

Tooth flanks are ruled surfaces through the axis direction (constant angle at every radius); flank angles are
given at the mean radius r_m = (od + id) / 4:
  backcut_deg  hook on the drive flank (tooth tip leans forward by tooth_h * tan(backcut)). Under drive torque
               the contact normal pulls the halves together, so load keeps the clutch engaged. 3 to 5 deg.
  ramp_deg     back flank angle from the face plane (90 = square, both directions positive; e.g. 35 = ratchet:
               in reverse the back flanks cam the halves apart, so the drum spring lets it slip).
The circumferential gap `clearance` (mm at r_m) is left on both flanks at full engagement, and `tip_gap` (mm)
between every tooth tip and the opposite root.

Slip clutch: see slip_clutch().
"""
import math

import numpy as np
from manifold3d import Manifold

from . import prim, tf, shafts, fasteners
from .fits import SHAFT_IN_BORE, M3_CLEAR_D, M3_NUT_H

E_PLA = 3500.0     # MPa
E_PETG = 2100.0
E_PPA_CF = 6000.0  # printed, in plane, conservative
E_TPU95 = 26.0
MU_PLA_PLA = 0.30  # dry, printed faces (estimate; check on the bench)
SIGMA_FLEX_PLA = 25.0   # MPa, allowable bending stress for a long-lived PLA flexure


# ---------------------------------------------------------------- dog clutch

def _subdiv(pts, step):
    out = []
    n = len(pts)
    for i in range(n):
        a = np.asarray(pts[i], float)
        b = np.asarray(pts[(i + 1) % n], float)
        k = max(1, int(math.ceil(np.linalg.norm(b - a) / step)))
        for j in range(k):
            out.append(tuple(a + (b - a) * j / k))
    return out


def _polar_tooth(poly_sz, ri, ro, r_m):
    """Solid {(r, theta, z): ri <= r <= ro, (r_m * theta, z) inside poly_sz} (one tooth, not crossing 2 pi)."""
    cs = prim.poly_cs(_subdiv(poly_sz, 0.4))
    m = Manifold.extrude(cs, ro - ri).translate((0, 0, ri))     # coords (s, z, r)

    def w(p):
        s, z, r = p[:, 0].copy(), p[:, 1].copy(), p[:, 2].copy()
        th = s / r_m
        out = np.empty_like(p)
        out[:, 0] = r * np.cos(th)
        out[:, 1] = r * np.sin(th)
        out[:, 2] = z
        return out

    t = m.warp_batch(w)
    if t.volume() < 0:   # never expected (the map keeps orientation), guard anyway
        raise RuntimeError("tooth warp flipped orientation")
    return t


def _bore_cut(spec, z0, z1, id_):
    """Bore cutter through z = [z0, z1]. spec: None (round id_), float d (round, literal diameter),
    ("keyed", d, key_w), ("d", d, flat), ("hex", af), or a Manifold cutter given in the half's frame."""
    L = z1 - z0
    if spec is None:
        return prim.cyl(id_ / 2, L + 2, z=z0 - 1)
    if isinstance(spec, Manifold):
        return spec
    if isinstance(spec, (int, float)):
        return prim.cyl(spec / 2, L + 2, z=z0 - 1)
    kind = spec[0]
    if kind == "keyed":
        c = shafts.keyed_bore(L, spec[1], spec[2] if len(spec) > 2 else shafts.KEY20_W)
    elif kind == "d":
        c = shafts.d_bore(L, spec[1], spec[2])
    elif kind == "hex":
        c = shafts.hex_bore(L, spec[1])
    else:
        raise ValueError(f"unknown bore spec {spec!r}")
    return c.translate((0, 0, z1))


def dog_clutch(od=40.0, id=24.0, n_teeth=6, tooth_h=4.0, ramp_deg=90.0, backcut_deg=4.0, face_thick=6.0,
               bore=None, bore_driven="same", clearance=0.3, tip_gap=0.4):
    """Face dog clutch pair. Returns {"driver": Manifold, "driven": Manifold, "info": dict}.

    od, id: tooth ring diameters (mm). With bore=None the part is an annulus id..od; with a bore spec (see _bore_cut:
    float, ("keyed", 20, 6), ("d", d, flat), ("hex", af) or a Manifold) the base disc is solid inside id and the
    bore is cut through. bore_driven: spec for the driven half ("same" = bore).
    Both halves are in the frame of the module docstring (teeth up). Place the driven half with dog_mate(info, e).
    info: r_m, pitch_deg, gap_deg (flank gap as an angle), root_w and top_w (mm at r_m), e_full, and the inputs.
    """
    if not 0 < ramp_deg <= 90:
        raise ValueError("ramp_deg must be in (0, 90]")
    ri, ro = id / 2, od / 2
    r_m = (ri + ro) / 2
    h = tooth_h
    P = 2 * math.pi * r_m / n_teeth
    cr = 0.0 if ramp_deg >= 90 else 1.0 / math.tan(math.radians(ramp_deg))
    tb = math.tan(math.radians(backcut_deg))
    z_full = h + tip_gap
    W0 = (P - 2 * clearance - z_full * (tb - cr)) / 2
    top_w = W0 + h * (tb - cr)
    if top_w < 0.8 or W0 < 0.8:
        raise ValueError(f"teeth too thin (root {W0:.2f}, top {top_w:.2f} mm): fewer teeth, steeper ramp or lower h")
    zb = -0.5
    poly = [(zb * cr, zb), (W0 + zb * tb, zb), (W0 + h * tb, h), (h * cr, h)]
    tooth = _polar_tooth(poly, ri, ro, r_m)
    teeth = prim.union([prim.place(tooth, tf.Rz(k * 360.0 / n_teeth)) for k in range(n_teeth)])

    def half(spec):
        if spec is None:
            base = prim.tube(ro, ri, face_thick, z=-face_thick)
        else:
            base = prim.cyl(ro, face_thick, z=-face_thick)
        s = base + teeth
        if spec is not None:
            s = s - _bore_cut(spec, -face_thick, h, id)
        return s

    drv = half(bore)
    dvn = drv if bore_driven == "same" else half(bore_driven)
    info = dict(od=od, id=id, n_teeth=n_teeth, tooth_h=h, ramp_deg=ramp_deg, backcut_deg=backcut_deg,
                face_thick=face_thick, clearance=clearance, tip_gap=tip_gap, r_m=r_m, pitch_deg=360.0 / n_teeth,
                gap_deg=math.degrees(clearance / r_m), root_w=W0, top_w=top_w, e_full=h - tip_gap,
                _tb=tb, _cr=cr)
    return {"driver": drv, "driven": dvn, "info": info}


def dog_mate(info, e=None):
    """4x4 placing the driven half (own frame, teeth up) into the driver frame, engaged by depth e (mm, default the
    full depth tooth_h - tip_gap; 0 = tips just touching plane, negative = apart). The drive flanks keep the gap
    `clearance` at every e; the driven base face sits at z = 2 * tooth_h - e."""
    h = info["tooth_h"]
    if e is None:
        e = info["e_full"]
    if e > info["e_full"] + 1e-9:
        raise ValueError(f"e {e} beyond full depth {info['e_full']}")
    z_off = 2 * h - e
    W0 = info["root_w"]
    phi_s = 2 * W0 + z_off * info["_tb"] + info["clearance"]
    phi = math.degrees(phi_s / info["r_m"])
    return tf.T(0, 0, z_off) @ tf.Rz(phi) @ tf.Rx(180)


# ---------------------------------------------------------------- slip clutch helpers

def slip_torque(F_axial, mu, r_in, r_out, n_faces=2, model="wear"):
    """Slip torque in N m of an annular friction clutch: axial force F_axial (N), friction coefficient mu, friction
    annulus r_in..r_out (mm), n_faces sliding faces. model "wear" (uniform wear, r_eff = (r_in + r_out) / 2,
    the worn-in value) or "pressure" (uniform pressure, r_eff = 2/3 (ro^3 - ri^3) / (ro^2 - ri^2), new faces)."""
    if model == "wear":
        r_eff = (r_in + r_out) / 2
    elif model == "pressure":
        r_eff = 2.0 / 3.0 * (r_out ** 3 - r_in ** 3) / (r_out ** 2 - r_in ** 2)
    else:
        raise ValueError("model must be wear or pressure")
    return n_faces * mu * F_axial * r_eff / 1000.0


def flexure_force(deflection, geom, E=E_PLA):
    """Total axial force (N) of the bump-ring flexure at axial deflection (mm).

    geom: dict n (spans = upper bumps), L (free span between lower bumps, mm), b (radial beam width), t (beam
    thickness). Each span is a fixed-fixed beam (ring continuity) loaded at mid-span: k = 192 E I / L^3, I = b t^3/12.
    Straight-beam estimate of a curved beam (slightly soft-biased, torsion ignored); calibrate on the bench.
    E: E_PLA 3500, E_PETG 2100, E_TPU95 26 MPa.
    """
    I = geom["b"] * geom["t"] ** 3 / 12.0
    k = 192.0 * E * I / geom["L"] ** 3
    return geom["n"] * k * deflection


def flexure_stress(deflection, geom, E=E_PLA):
    """Peak bending stress (MPa) in the flexure beams at deflection (mm): sigma = 12 E t d / L^2."""
    return 12.0 * E * geom["t"] * deflection / geom["L"] ** 2


# ---------------------------------------------------------------- slip clutch geometry

def _sector(ri, ro, a_deg, w_deg, z0, z1):
    """Annular sector ri..ro centred on angle a_deg, angular width w_deg, z = [z0, z1]."""
    n = max(4, int(w_deg / 2) + 2)
    a = np.radians(np.linspace(a_deg - w_deg / 2, a_deg + w_deg / 2, n))
    pts = [(ri * math.cos(t), ri * math.sin(t)) for t in a] + [(ro * math.cos(t), ro * math.sin(t)) for t in a[::-1]]
    return prim.extrude(prim.poly_cs(pts), z1 - z0, z=z0)


def flexure_ring(ri, ro, n=4, t=0.6, gap=0.8, bump_w=4.0, up_extra=0.4, phase_deg=0.0):
    """Printed bump-ring flexure (Belleville substitute), printable flat.

    Frame: axis +Z, bottom at z = 0. n lower bumps (z = [0, gap]) at angles phase + k * 360/n stand on the plate
    below; a continuous beam annulus ri..ro (z = [gap, gap + t]) bridges between them; n upper bumps
    (z = [gap + t, 2 gap + t + up_extra]) sit mid-span. Pushing the upper bumps down bends each span (max useful
    deflection = gap, then the beam bottoms on the plate: a hard stop). bump_w: bump arc width at mid radius (mm).
    Returns (solid, geom) with geom for flexure_force / flexure_stress.
    """
    r_m = (ri + ro) / 2
    wd = math.degrees(bump_w / r_m)
    ring = prim.tube(ro, ri, t, z=gap)
    lo = [_sector(ri, ro, phase_deg + k * 360.0 / n, wd, 0.0, gap + 0.01) for k in range(n)]
    up = [_sector(ri, ro, phase_deg + (k + 0.5) * 360.0 / n, wd, gap + t - 0.01, 2 * gap + t + up_extra)
          for k in range(n)]
    s = prim.union([ring] + lo + up)
    L = 2 * math.pi * r_m / n - bump_w
    geom = dict(n=n, L=L, b=ro - ri, t=t, gap=gap, height=2 * gap + t + up_extra, r_m=r_m)
    return s, geom


def slip_clutch(shaft_d=20.0, key_w=6.0, r_in=16.5, r_out=25.0, hub_t=6.0, disc_t=3.0, plate_t=3.0, adj_t=4.0,
                n_screws=3, bc_r=14.0, screw_deg0=60.0, flex_n=4, flex_t=0.6, flex_gap=0.7, flex_ro=23.0,
                n_grooves=12, groove_w=1.2, groove_d=0.4, n_lugs=3, lug_w=6.0, lug_h=3.0, mu=MU_PLA_PLA, E=E_PLA):
    """Printed friction slip clutch on a keyed shaft, all parts printable flat. Axis +Z, stack from z = 0 up.

    Stack (bottom to top): hub_plate (keyed, M3 heads counterbored from below, reaction face), friction_disc
    (driven member: grooved both faces, n_lugs drive lugs outside r_out engage the reel or gear, free on the shaft,
    radially located by its lugs / the reel), pressure_plate (keyed, slides axially), flexure (bump ring),
    adjuster_plate (keyed, M3 nut traps on top). n_screws M3 SHCS on bolt circle bc_r pass up through the hub,
    pressure and adjuster plates (inside the friction disc and flexure IDs, which they also locate radially).
    Tightening the nuts pulls the adjuster down and deflects the flexure: F = flexure_force(d, geom), slip torque
    T = slip_torque(F, mu, r_in, r_out, 2). Two friction faces (both faces of the friction disc).

    Returns dict:
      parts:  name -> Manifold, each in its own frame (bottom face at z = 0, axis +Z): hub_plate, friction_disc,
              pressure_plate, flexure, adjuster_plate, screw (one M3 SHCS, model frame), nut (one M3 nut).
      stack:  name -> z of the part bottom at zero preload (flexure just touching).
      place:  name -> 4x4 (part frame -> clutch frame) for the printed parts, plus screw_1.., nut_1.. for the
              bought parts.
      info:   flex_geom, screw_len, torque_per_mm (N m per mm of nut travel), table of (deflection mm, F N,
              T N m, stress MPa), height, and the inputs. Defaults: about 16 N/mm, 0.2 N m/mm, 0.7 mm travel
              (24 MPa at bottoming). Preload d moves the adjuster plate (and screws' nuts) down by d.
    """
    R = r_out
    parts = {}
    key = ("keyed", shaft_d, key_w)
    ang = [screw_deg0 + k * 360.0 / n_screws for k in range(n_screws)]
    sxy = [(bc_r * math.cos(math.radians(a)), bc_r * math.sin(math.radians(a))) for a in ang]
    head_depth = fasteners.M3_SHCS_HEAD_H + 0.2

    # hub plate
    hub = prim.cyl(R, hub_t) - _bore_cut(key, 0, hub_t, 0)
    cb = fasteners.m3_counterbore(hub_t, head_depth)
    hub = hub - prim.union([prim.place(cb, tf.T(x, y, 0) @ tf.Rx(180)) for x, y in sxy])
    parts["hub_plate"] = hub

    # friction disc
    disc = prim.tube(R, r_in, disc_t)
    lugs = [_sector(R - 0.5, R + lug_h, a, math.degrees(lug_w / R), 0, disc_t)
            for a in np.arange(n_lugs) * 360.0 / n_lugs]
    disc = prim.union([disc] + lugs)
    if n_grooves:
        gr = []
        for k in range(n_grooves):
            a = (k + 0.5) * 360.0 / n_grooves
            for z0, z1 in ((-1, groove_d), (disc_t - groove_d, disc_t + 1)):
                gr.append(prim.place(prim.box(r_in - 1, R + 0.01, -groove_w / 2, groove_w / 2, z0, z1), tf.Rz(a)))
        disc = disc - prim.union(gr)
    parts["friction_disc"] = disc

    clear = [prim.cyl(M3_CLEAR_D / 2, 40, x=x, y=y, z=-20) for x, y in sxy]

    # pressure plate
    parts["pressure_plate"] = prim.cyl(R, plate_t) - _bore_cut(key, 0, plate_t, 0) - prim.union(clear)

    # flexure
    flex, geom = flexure_ring(r_in, flex_ro, flex_n, flex_t, flex_gap, phase_deg=0.0)
    parts["flexure"] = flex

    # adjuster plate with nut traps opening on top
    adj = prim.cyl(R, adj_t) - _bore_cut(key, 0, adj_t, 0) - prim.union(clear)
    nt = fasteners.m3_nut_trap(0.0, adj_t + 1)
    adj = adj - prim.union([nt.translate((x, y, adj_t)) for x, y in sxy])
    parts["adjuster_plate"] = adj

    z = {}
    z["hub_plate"] = 0.0
    z["friction_disc"] = hub_t
    z["pressure_plate"] = hub_t + disc_t
    z["flexure"] = hub_t + disc_t + plate_t
    z["adjuster_plate"] = z["flexure"] + geom["height"]
    top = z["adjuster_plate"] + adj_t

    need = top - head_depth + 1.0 + flex_gap     # through the nut with 1 mm to spare at full preload
    screw_len = next((L for L in (10, 12, 16, 20, 25, 30, 35, 40, 45, 50) if L >= need), math.ceil(need))
    parts["screw"] = fasteners.m3_shcs(screw_len)
    parts["nut"] = fasteners.m3_nut()

    place = {n: tf.T(0, 0, zz) for n, zz in z.items()}
    for i, (x, y) in enumerate(sxy, 1):
        place[f"screw_{i}"] = tf.T(x, y, head_depth) @ tf.Rx(180)
        place[f"nut_{i}"] = tf.T(x, y, top - (M3_NUT_H - fasteners.M3_NUT_H_NOM) / 2)

    table = []
    for d in np.linspace(0, flex_gap, 5):
        F = flexure_force(d, geom, E)
        table.append((round(float(d), 3), round(float(F), 2), round(float(slip_torque(F, mu, r_in, r_out, 2)), 4),
                      round(float(flexure_stress(d, geom, E)), 1)))
    info = dict(flex_geom=geom, screw_len=screw_len, screw_xy=sxy, height=top, mu=mu, E=E, r_in=r_in, r_out=r_out,
                n_faces=2, torque_per_mm=slip_torque(flexure_force(1.0, geom, E), mu, r_in, r_out, 2),
                table=table, max_deflection=flex_gap, shaft_d=shaft_d, key_w=key_w)
    return {"parts": parts, "stack": z, "place": place, "info": info}
