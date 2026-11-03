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
The circumferential gap `clearance` (mm at r_m) is left on the drive flanks at full engagement, and `tip_gap` (mm)
between every tooth tip and the opposite root. The back (ramp) flanks get ramp_gap (default
max(clearance, tip_gap (cot(ramp) - tan(backcut)) + RAMP_GAP_MARGIN)): pushing the halves further together closes
a ramp flank gap at the axial rate cot(ramp) - tan(backcut), so this default lets the halves travel the full
tip_gap before the ramp flanks touch (info["axial_free"], the axial travel left at full engagement, equals tip_gap;
with a smaller ramp_gap the halves bottom on the ramp flanks first and info["axial_free"] says by how much).

Slip clutch: see slip_clutch().
"""
import math

import numpy as np
from manifold3d import Manifold

from . import prim, tf, shafts, fasteners
from .fits import SHAFT_IN_BORE, M3_CLEAR_D, M3_NUT_H, M3_NUT_AF, M3_HEAD_D

E_PLA = 3500.0     # MPa
E_PETG = 2100.0
E_PPA_CF = 6000.0  # printed, in plane, conservative
E_TPU95 = 26.0
MU_PLA_PLA = 0.30  # dry, printed faces (estimate; check on the bench)
SIGMA_FLEX_PLA = 25.0   # MPa, allowable short-term bending stress of a PLA flexure (do not hold it there)
RAMP_GAP_MARGIN = 0.05  # mm added to the default ramp flank gap (mesh chords of the warped flanks)
SIGMA_FLEX_PLA_SUSTAINED = 10.0   # MPa, PLA flexure held deflected for weeks (creep); sizes the usable travel


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
               bore=None, bore_driven="same", clearance=0.3, tip_gap=0.4, ramp_gap=None):
    """Face dog clutch pair. Returns {"driver": Manifold, "driven": Manifold, "info": dict}.

    od, id: tooth ring diameters (mm). With bore=None the part is an annulus id..od; with a bore spec (see _bore_cut:
    float, ("keyed", 20, 6), ("d", d, flat), ("hex", af) or a Manifold) the base disc is solid inside id and the
    bore is cut through. bore_driven: spec for the driven half ("same" = bore).
    Both halves are in the frame of the module docstring (teeth up). Place the driven half with dog_mate(info, e).
    ramp_gap: circumferential gap (mm at r_m) on the back flanks at full engagement, default
    max(clearance, tip_gap (cot(ramp) - tan(backcut)) + RAMP_GAP_MARGIN) so that the halves never bottom on the
    ramp flanks before the tip gap closes (see the module docstring).
    info: r_m, pitch_deg, gap_deg (drive flank gap as an angle), ramp_gap (mm), axial_free (axial travel left at
    full engagement before any contact: tip_gap with the default ramp_gap), root_w and top_w (mm at r_m), e_full,
    and the inputs.
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
    close = cr - tb              # axial rate at which pushing the halves together closes the ramp flank gap
    if ramp_gap is None:
        ramp_gap = max(clearance, tip_gap * close + RAMP_GAP_MARGIN)
    axial_free = tip_gap if close <= 1e-12 else min(tip_gap, ramp_gap / close)
    W0 = (P - clearance - ramp_gap - z_full * (tb - cr)) / 2
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
                gap_deg=math.degrees(clearance / r_m), ramp_gap=ramp_gap, axial_free=axial_free, root_w=W0,
                top_w=top_w, e_full=h - tip_gap,
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


def flexure_pads(ri, ro, n=3, gap=0.5, bump_w=4.0, phase_deg=0.0):
    """The n support pads of flexure_ring, to union onto the top face of the plate the flexure rests on.

    Frame: axis +Z, pads z = [0, gap] (translate to the plate top), annular sectors ri..ro of arc width bump_w at mid
    radius, centred on phase + k * 360/n. Printed as part of that plate (bosses on its top face, no overhang)."""
    r_m = (ri + ro) / 2
    wd = math.degrees(bump_w / r_m)
    return prim.union([_sector(ri, ro, phase_deg + k * 360.0 / n, wd, 0.0, gap) for k in range(n)])


def flexure_ring(ri, ro, n=3, t=0.8, gap=0.5, bump_w=4.0, up_extra=0.4, phase_deg=0.0, lower_bumps=False,
                 tabs=None, tab_hole=M3_CLEAR_D, tab_wall=1.4):
    """Printed bump-ring flexure (Belleville substitute), printable flat with no bridges.

    Frame: axis +Z, bottom at z = 0. A continuous beam annulus ri..ro (z = [0, t]) prints directly on the bed; n upper
    bumps (z = [t, t + gap + up_extra]) at angles phase + (k + 0.5) * 360/n sit mid-span. The beam rests on n
    support pads at phase + k * 360/n that belong to the plate below (flexure_pads(ri, ro, n, gap, bump_w,
    phase_deg), height gap), so no part of the flexure hangs in the air while printing. Pushing the upper bumps
    down bends each span (max useful deflection = gap, then the beam bottoms on the plate between the pads: a hard
    stop). bump_w: pad / bump arc width at mid radius (mm).
    lower_bumps=True restores the old one-piece version (the beam on lower bumps z = [0, gap], beam z = [gap,
    gap + t]); it is NOT printable flat (each span is a bridge gap above the bed), use only on edge or in TPU.
    tabs: optional list of (x, y) screw positions inside ri: a tab with a tab_hole clearance hole reaches in to each,
    joined to the ring at its angle (put the pads at those angles: the tab sits on a support, not mid-span); the
    screws then keep the ring from turning off its pads.
    Returns (solid, geom) with geom for flexure_force / flexure_stress (geom["z_beam"]: beam bottom in this frame).
    """
    r_m = (ri + ro) / 2
    wd = math.degrees(bump_w / r_m)
    zb = gap if lower_bumps else 0.0
    ring = prim.tube(ro, ri, t, z=zb)
    up = [_sector(ri, ro, phase_deg + (k + 0.5) * 360.0 / n, wd, zb + t - 0.01, zb + t + gap + up_extra)
          for k in range(n)]
    extra = []
    if lower_bumps:
        extra = [_sector(ri, ro, phase_deg + k * 360.0 / n, wd, 0.0, gap + 0.01) for k in range(n)]
    s = prim.union([ring] + up + extra)
    if tabs:
        hr = tab_hole / 2
        for x, y in tabs:
            rc = math.hypot(x, y)
            a = math.degrees(math.atan2(y, x))
            w = tab_hole + 2 * tab_wall
            tab = prim.box(rc - hr - tab_wall, ri + 0.5, -w / 2, w / 2, zb, zb + t)
            tab = tab + prim.cyl(w / 2, t, x=rc, z=zb)
            s = s + prim.place(tab, tf.Rz(a))
            s = s - prim.cyl(hr, t + 2, x=x, y=y, z=zb - 1)
    L = 2 * math.pi * r_m / n - bump_w
    geom = dict(n=n, L=L, b=ro - ri, t=t, gap=gap, height=zb + t + gap + up_extra, r_m=r_m, z_beam=zb,
                phase_deg=phase_deg, bump_w=bump_w, lower_bumps=lower_bumps)
    return prim.clean(s), geom


def flexure_travel(geom, E=E_PLA, sigma=SIGMA_FLEX_PLA_SUSTAINED):
    """Deflection (mm) at which the flexure beams reach sigma (MPa), capped at the hard stop geom["gap"].
    Default the sustained PLA limit (creep); PETG creeps less: E=E_PETG gives 1.7x the travel at the same stress."""
    return min(geom["gap"], sigma * geom["L"] ** 2 / (12.0 * E * geom["t"]))


def slip_clutch(shaft_d=20.0, key_w=6.0, r_in=18.5, r_out=27.0, hub_t=6.0, disc_t=3.0, plate_t=3.0, adj_t=4.0,
                n_screws=3, bc_r=16.0, screw_deg0=60.0, flex_n=3, flex_t=0.8, flex_gap=0.5, flex_ro=25.0,
                n_grooves=12, groove_w=1.2, groove_d=0.4, n_lugs=3, lug_w=6.0, lug_h=3.0, mu=MU_PLA_PLA, E=E_PLA,
                sigma_sustained=SIGMA_FLEX_PLA_SUSTAINED):
    """Printed friction slip clutch on a keyed shaft, all parts printable flat without bridges. Axis +Z, stack
    from z = 0 up.

    Stack (bottom to top): hub_plate (keyed, M3 heads counterbored from below, reaction face), friction_disc
    (driven member: grooved both faces, n_lugs drive lugs outside r_out engage the reel or gear, free on the shaft,
    radially located by its lugs / the reel), pressure_plate (keyed, slides axially, carries the flex_n flexure
    support pads on top), flexure (bump ring printed beam down, tabs on the screws keep it on its pads),
    adjuster_plate (keyed, M3 nut traps on top). n_screws M3 SHCS on bolt circle bc_r pass up through the hub,
    pressure plate, flexure tabs and adjuster plate (inside the friction disc ID, which they locate radially).
    Tightening the nuts pulls the adjuster down and deflects the flexure: F = flexure_force(d, geom), slip torque
    T = slip_torque(F, mu, r_in, r_out, 2). Two friction faces (both faces of the friction disc).
    The flexure pads sit at the screw angles (flex_n a multiple of n_screws). Walls: bc_r 16 leaves 2.5 mm between
    the nut traps / counterbores and the 20.3 keyed bore (info["min_wall"]).
    Flexure stress: PLA creeps under a held load, so the usable travel (info["travel"], the table) is limited to
    sigma_sustained (default SIGMA_FLEX_PLA_SUSTAINED = 10 MPa); the hard stop flex_gap is sized to about that.
    PETG option: print the flexure in PETG and pass E=E_PETG (less creep, softer: same travel, lower force).

    Returns dict:
      parts:  name -> Manifold, each in its own frame (bottom face at z = 0, axis +Z): hub_plate, friction_disc,
              pressure_plate, flexure, adjuster_plate, screw (one M3 SHCS, model frame), nut (one M3 nut).
      stack:  name -> z of the part bottom at zero preload (flexure just touching).
      place:  name -> 4x4 (part frame -> clutch frame) for the printed parts, plus screw_1.., nut_1.. for the
              bought parts.
      info:   flex_geom, screw_len, torque_per_mm (N m per mm of nut travel), travel (usable deflection at
              sigma_sustained), table of (deflection mm, F N, T N m, stress MPa) from 0 to travel, height,
              min_wall, and the inputs. Defaults: about 7.8 N/mm, 0.1 N m/mm, 0.5 mm travel (9.8 MPa at the
              stop), slip torque up to about 0.05 N m. Preload d moves the adjuster plate (and the nuts) down by d.
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

    # flexure (printed beam down) and its support pads on the pressure plate
    flex, geom = flexure_ring(r_in, flex_ro, flex_n, flex_t, flex_gap, phase_deg=screw_deg0, tabs=sxy)
    parts["flexure"] = flex
    pads = flexure_pads(r_in, flex_ro, flex_n, flex_gap, geom["bump_w"], screw_deg0).translate((0, 0, plate_t))

    # pressure plate
    pp = prim.union([prim.cyl(R, plate_t), pads])
    parts["pressure_plate"] = pp - _bore_cut(key, 0, plate_t, 0) - prim.union(clear)

    # adjuster plate with nut traps opening on top
    adj = prim.cyl(R, adj_t) - _bore_cut(key, 0, adj_t, 0) - prim.union(clear)
    nt = fasteners.m3_nut_trap(0.0, adj_t + 1)
    adj = adj - prim.union([nt.translate((x, y, adj_t)) for x, y in sxy])
    parts["adjuster_plate"] = adj

    z = {}
    z["hub_plate"] = 0.0
    z["friction_disc"] = hub_t
    z["pressure_plate"] = hub_t + disc_t
    z["flexure"] = hub_t + disc_t + plate_t + flex_gap
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

    travel = flexure_travel(geom, E, sigma_sustained)
    table = []
    for d in np.linspace(0, travel, 5):
        F = flexure_force(d, geom, E)
        table.append((round(float(d), 3), round(float(F), 2), round(float(slip_torque(F, mu, r_in, r_out, 2)), 4),
                      round(float(flexure_stress(d, geom, E)), 1)))
    # thinnest wall between the keyed bore (bore + keyway) and the nut traps / counterbores
    r_bore = (shaft_d + SHAFT_IN_BORE) / 2
    r_feat = max(M3_NUT_AF / math.sqrt(3), M3_HEAD_D / 2)      # nut trap corner, counterbore
    min_wall = bc_r - r_feat - r_bore
    info = dict(flex_geom=geom, screw_len=screw_len, screw_xy=sxy, height=top, mu=mu, E=E, r_in=r_in, r_out=r_out,
                n_faces=2, torque_per_mm=slip_torque(flexure_force(1.0, geom, E), mu, r_in, r_out, 2),
                table=table, travel=travel, sigma_sustained=sigma_sustained, max_deflection=flex_gap,
                shaft_d=shaft_d, key_w=key_w, min_wall=min_wall)
    return {"parts": parts, "stack": z, "place": place, "info": info}
