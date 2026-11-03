"""Drum subsystem (worker cad-drum): main shaft, coupler, dog clutch, REEL output gear, LAUNCH dog ring,
N20 selector drum with groove 1 (fork), track 2 (reel brake preload cam), Gray code bands, wrap segment gear with
locking arc, wrap idler, compliant fork, shift rail, brackets, switch levers, N20 cradle, hand lever.

Local frame L (joint drum_base): z along the main axis (world +Y, s = distance from the wiper boss face),
x = world +X (forward), y = world -Z (DOWN). The drum axis is at L (0, DRUM_OFF, z). Timing, codes and loads are in
_drum_kin.py (shared with firmware docs and tests). Notes: drum_NOTES.md.
"""
import math

import numpy as np
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union
from shapely import affinity

from cad.lib import Part, Joint, Allow, Clearance, prim, tf, fits, gears
from cad.lib.clutch import _polar_tooth
from cad.parts import _drum_kin as K

# ------------------------------------------------------------------------------------------------ geometry numbers
GEOM = dict(
    main_height=165.0,      # main axis above the deck (coordinator, 3 Nov)
    drum_off=72.0,          # drum axis below the main axis
    s0_world_y=0.0,         # world y of the wiper boss face (s = 0)
    # coupler
    cpl_s=(0.5, 36.0), cpl_od=40.0, sock_s=22.0,
    hub_od=26.0, hub_s=(0.5, 21.0), nut_s=14.5, shear_s=11.5, shear_d=4.0,     # D8 torque limiter (shear pin)
    bus_m=1.5, bus_z=40, bus_s=(1.0, 9.0),                                       # D15 take-up bus gear
    # main shaft
    hex_af=22.0, hex_s=(22.0, 96.0), spig_d=12.0, spig_end=160.0, xpin_s=29.0,
    # REEL sleeve and gear
    rsl_flange=(36.0, 40.0, 44.0), rsl_journal=(40.0, 54.0, 30.0),
    out_m=2.0, out_z=40, out_face=14.0, rgear_s=40.5,
    # dogs (ring radii from _drum_kin)
    dog_face_reel=54.5,     # REEL gear face (dog roots) ; dog tips at +dog_h
    collar_s=(62.0, 88.0), collar_od=60.0, groove=(72.0, 78.0), groove_r=26.0,
    ring_face=95.5,         # LAUNCH dog ring dog roots; dog tips at -dog_h
    ring_s=(95.5, 101.0), ring_od=62.0, ring_id=27.0, ring_lug=(6.0, 3.0, 15.0, 21.0), ring_screw_r=28.0,
    # drum (s ranges)
    # drum: two printed parts on a fixed d10 axle. drum A: gear, segment, lock, cam, neck, hex spigot;
    # drum B: groove barrel, code bands, hand lever hex boss
    axle_d=10.0, axle_s=(7.0, 131.0), drumA_s=(16.0, 72.0), drumB_s=(66.0, 123.7), spig_af=18.0,
    dgear=(16.0, 24.0), seg=(25.0, 33.0), lock=(33.0, 39.0), neck_d=20.0, neck_s=(52.0, 66.0),
    cam=(41.0, 52.0), cam_r0=22.0, barrel=(66.0, 96.0), barrel_r=30.0, groove_w=6.4, groove_d=4.0,
    bands=((97.0, 103.0), (104.0, 110.0), (111.0, 117.0)), band_r0=22.0, bump=3.0, nub_r=1.5,
    hexboss=(117.0, 123.7, 14.0),
    # fork
    fork_s=75.0, fork_over=0.25, fork_t=2.8, yoke_r=(31.0, 37.0), pin_d=6.0, pin_off=9.5, hub_len=14.0,
    rail_xy=(45.0, 38.0), rail_d=10.0, rail_s=(7.0, 131.0),
    # world-fixed angles (L frame xy plane, deg; y is down): follower up = -90, idler behind = 180
    phi_pin=-90.0, phi_idler=180.0, phi_brake=200.0, phi_sw=0.0, phi_n20=30.0,
    # brackets (s)
    brA=(7.0, 15.7), brB=(124.0, 131.0), brI=(63.0, 69.0),
    idl=(25.0, 70.0), idl_gear=(54.0, 62.0), idl_j=10.0,
    lock_RL=38.0, lock_rl=11.0, lock_c=0.25,
    sw_L=30.0,
)


def G(P=None):
    g = dict(GEOM)
    d = getattr(P, "DRUM", None) if P is not None else None
    if isinstance(d, dict) and isinstance(d.get("geom"), dict):
        g.update({k: v for k, v in d["geom"].items() if k in g})
    lay = getattr(P, "LAYOUT", None) if P is not None else None
    if isinstance(lay, dict) and isinstance(lay.get("main"), dict) and lay["main"].get("origin") is not None:
        o = lay["main"]["origin"]
        g["main_height"] = float(o[2])
        g["s0_world_y"] = float(o[1])
    return g


def C(P=None):
    return K.cfg(P)


# ------------------------------------------------------------------------------------------------ derived
def deck_y(g):
    return g["main_height"]                       # deck plane in L (y down from the main axis)


def drum_xy(g):
    return (0.0, g["drum_off"])


def a_seg(c):
    return gears.center_distance(c["seg_m"], c["seg_z"], c["pin_z"], 0.0, c["pin_x"])


def a_n20(c):
    return gears.center_distance(c["drum_gear_m"], c["drum_gear_z"], c["n20_z"], c["drum_gear_x"], c["n20_x"])


def idler_xy(g, c):
    a = a_seg(c)
    p = math.radians(g["phi_idler"])
    return (a * math.cos(p), g["drum_off"] + a * math.sin(p))


def n20_xy(g, c):
    a = a_n20(c)
    p = math.radians(g["phi_n20"])
    return (a * math.cos(p), g["drum_off"] + a * math.sin(p))


def seg_phase(g, c):
    """Rotation (deg) of the segment gear on the drum so its teeth sit on the right indices."""
    pitch = 360.0 / c["seg_z"]
    a0 = g["phi_idler"] - c["seg_start"] - pitch           # local angle of the first present tooth
    return a0 % pitch


def seg_present(g, c):
    pitch = 360.0 / c["seg_z"]
    d = seg_phase(g, c)
    out = []
    # seg_teeth pitches span the window (4 pinion turns); one tooth fewer is cut so that the first tooth only
    # starts contact at seg_start and the last one leaves at the window end (contact ratio about 1.4)
    for j in range(c["seg_teeth"] - 1):
        a = g["phi_idler"] - c["seg_start"] - (j + 1) * pitch     # local angle of tooth j
        out.append(int(round((a - d) / pitch)) % c["seg_z"])
    return out


def mesh2(z1, z2, th1, phase1):
    """Angle of gear 2 (centre at direction phase1 from gear 1) meshing gear 1 at th1 (tooth-0 conventions)."""
    return phase1 + 180.0 + 180.0 / z2 - (th1 - phase1) * z1 / z2


def pinion_angle(theta, g, c):
    """Wrap idler joint angle at drum angle theta (locked outside the toothed arc)."""
    t0 = c["seg_start"]
    t1 = t0 + K.seg_span(c)
    tt = min(max(theta, t0), t1)
    return mesh2(c["seg_z"], c["pin_z"], tt + seg_phase(g, c), g["phi_idler"])


def n20_angle(theta, g, c):
    return mesh2(c["drum_gear_z"], c["n20_z"], theta, g["phi_n20"])


# ------------------------------------------------------------------------------------------------ helpers
def ring(ro, ri, s0, s1, seg=None):
    return prim.tube(ro, ri, s1 - s0, z=s0, seg=seg)


def cyl(r, s0, s1, x=0.0, y=0.0, seg=None):
    return prim.cyl(r, s1 - s0, x=x, y=y, z=s0, seg=seg)


def hex_cs(af):
    r = af / math.sqrt(3)
    return prim.poly_cs([(r * math.cos(math.radians(60 * k)), r * math.sin(math.radians(60 * k))) for k in range(6)])


def hexprism(af, s0, s1):
    """Hex with flats at +-y (vertices on +-x)."""
    return prim.extrude(hex_cs(af), s1 - s0, z=s0)


def shp_extrude(poly, s0, s1):
    return prim.extrude(prim.shapely_cs(poly), s1 - s0, z=s0)


def polar_profile(rfun, n=720):
    """Closed polygon r(alpha) with alpha in degrees, local frame."""
    pts = []
    for i in range(n):
        a = 360.0 * i / n
        r = rfun(a)
        pts.append((r * math.cos(math.radians(a)), r * math.sin(math.radians(a))))
    return Polygon(pts)


def theta_of_local(alpha, phi_w, c):
    """Drum angle at which world-fixed direction phi_w sees drum-local angle alpha; mapped into the travel window."""
    th = (phi_w - alpha - c["stop_lo"]) % 360.0 + c["stop_lo"]
    return th


def bed(solid, R):
    """print_tf: rotate by R then translate so the part sits at z = 0 near the origin."""
    m = prim.place(solid, R)
    lo, hi = m.bounding_box()[:3], m.bounding_box()[3:]
    return tf.T(-0.5 * (lo[0] + hi[0]), -0.5 * (lo[1] + hi[1]), -lo[2]) @ R


def dog_teeth(c, s_root, facing, phase_deg):
    """Ring of dogs on a face: roots at s_root, tips at s_root + facing * dog_h. Back-cut both flanks, roofed tips."""
    h, n = c["dog_h"], c["dog_n"]
    ri, ro = c["dog_r_in"], c["dog_r_out"]
    rm = 0.5 * (ri + ro)
    wr = math.radians(c["dog_deg"]) * rm
    tb = math.tan(math.radians(c["dog_backcut"]))
    rf = c["dog_roof"]
    zb = -0.6

    def hw(z):
        return wr / 2 + z * tb

    poly = [(-hw(zb), zb), (hw(zb), zb), (hw(h - rf), h - rf), (hw(h - rf) - rf, h), (-(hw(h - rf) - rf), h),
            (-hw(h - rf), h - rf)]
    tooth = _polar_tooth(poly, ri, ro, rm)
    teeth = prim.union([prim.place(tooth, tf.Rz(phase_deg + k * 360.0 / n)) for k in range(n)])
    if facing < 0:
        teeth = teeth.mirror((0, 0, 1))
    return teeth.translate((0, 0, s_root))


# ------------------------------------------------------------------------------------------------ main shaft group
def coupler_hub(g, P):
    """Inner coupler hub (PPA-CF): wiper shaft bore (parametric WIPER), M8 nut pocket, half of the shear pin hole."""
    W = getattr(P, "WIPER", {}) or {}
    d = float(W.get("shaft_d", 10.0)); fl = float(W.get("shaft_flat", 8.0)); L = float(W.get("shaft_len", 20.0))
    h0, h1 = g["hub_s"]
    body = cyl(g["hub_od"] / 2, h0, h1)
    dd = prim.extrude(prim.shapely_cs(Point(0, 0).buffer((d + fits.HOLE_SLIDE) / 2, 64)
                                      .intersection(Polygon([(-20, -(fl + 0.2) / 2), (20, -(fl + 0.2) / 2),
                                                             (20, (fl + 0.2) / 2), (-20, (fl + 0.2) / 2)]))),
                      g["nut_s"] - h0 + 1.0, z=h0 - 1.0)
    cut = [dd, cyl(4.3, h0 - 1, h1 + 1), cyl(9.6, g["nut_s"], h1 + 1)]   # round pocket: M8 nut + 13 mm socket
    r = g["hub_od"] / 2
    cut.append(prim.place(cyl(g["shear_d"] / 2, r - 4.0, r + 1.0, seg=32), tf.T(0, 0, g["shear_s"]) @ tf.Ry(90)))
    return prim.diff(body, cut)


def coupler(g, P):
    """Outer coupler (PPA-CF): runs on the hub, driven only through the shear pin (D8 torque limiter, about 5 N m),
    carries the take-up bus gear (D15, after the limiter) and the hex socket for the main shaft."""
    s0, s1 = g["cpl_s"]
    b0, b1 = g["bus_s"]
    body = prim.union([cyl(g["cpl_od"] / 2, b1 - 0.01, s1), cyl(g["hub_od"] / 2 + 3.0, s0, b1 + 0.01)])
    bus = gears.spur_gear(g["bus_m"], g["bus_z"], b1 - b0).translate((0, 0, b0))
    body = prim.union([body, bus])
    h0, h1 = g["hub_s"]
    cut = [cyl((g["hub_od"] + fits.SHAFT_IN_BORE) / 2, s0 - 1, h1 + 0.15),
           cyl(9.8, h1, g["sock_s"] + 0.5),                                          # 13 mm socket reaches the nut
           hexprism(g["hex_af"] + fits.HOLE_SLIDE, g["sock_s"], s1 + 1)]
    cut.append(prim.place(cyl(1.7, -30, 30), tf.T(0, 0, g["xpin_s"]) @ tf.Ry(90)))          # M3 cross screw
    cut.append(prim.place(prim.extrude(hex_cs(5.9), 3.0, z=0), tf.T(g["cpl_od"] / 2 - 2.6, 0, g["xpin_s"]) @ tf.Ry(90)))
    cut.append(prim.place(cyl((g["shear_d"] + 0.1) / 2, 10.0, 30.0, seg=32), tf.T(0, 0, g["shear_s"]) @ tf.Ry(90)))
    return prim.diff(body, cut)


def shear_pin(g):
    """D8 torque limiter: PLA pin d4 in single shear at r 13 (about 5 N m at 30 MPa). Print lying (spares x 5)."""
    r = g["hub_od"] / 2
    pin = cyl(g["shear_d"] / 2 - 0.02, r - 3.9, g["cpl_od"] / 2 + 0.01, seg=32)
    head = cyl(g["shear_d"] / 2 + 1.0, g["cpl_od"] / 2, g["cpl_od"] / 2 + 1.2, seg=32)
    p = prim.union([pin, head])
    return prim.place(p, tf.T(0, 0, g["shear_s"]) @ tf.Ry(90))


def main_shaft(g):
    h0, h1 = g["hex_s"]
    sh = prim.union([hexprism(g["hex_af"], h0, h1), cyl(g["spig_d"] / 2, h1 - 0.5, g["spig_end"])])
    sh = sh - prim.place(cyl(1.7, -30, 30), tf.T(0, 0, g["xpin_s"]) @ tf.Ry(90))
    sh = sh - prim.place(cyl(1.7, -30, 30, seg=24), tf.T(0, 0, g["spig_end"] - 6.0) @ tf.Ry(90))
    return sh


def reel_sleeve(g):
    f0, f1, fd = g["rsl_flange"]
    j0, j1, jd = g["rsl_journal"]
    s = prim.union([cyl(fd / 2, f0, f1), cyl(jd / 2, f0, j1)])
    return s - hexprism(g["hex_af"] + fits.SHAFT_IN_BORE, f0 - 1, j1 + 1)


def reel_gear(g, c):
    face = g["out_face"]
    gr = gears.spur_gear(g["out_m"], g["out_z"], face, bore=g["rsl_journal"][2] + fits.SHAFT_IN_BORE)
    gr = gr.translate((0, 0, g["rgear_s"]))
    hub = ring(c["dog_r_out"] + 1.0, (g["rsl_journal"][2] + fits.SHAFT_IN_BORE) / 2, g["rgear_s"] + face - 0.5,
               g["dog_face_reel"])
    teeth = dog_teeth(c, g["dog_face_reel"], +1, 45.0)
    # inner lip bears on the sleeve end: the gear is held between the sleeve flange and the sleeve end
    lip = ring(c["dog_r_in"] - 0.5, 13.0, g["rsl_journal"][1] + 0.05, g["dog_face_reel"] + 1.5)
    return prim.union([gr, hub, teeth, lip])


def collar(g, c):
    s0, s1 = g["collar_s"]
    h = c["dog_h"]
    b0, b1 = s0 + h, s1 - h
    body = cyl(g["collar_od"] / 2, b0, b1)
    body = body - ring(g["collar_od"] / 2 + 1, g["groove_r"], g["groove"][0], g["groove"][1])
    for z in (b0, b1):
        pass
    teeth = [dog_teeth(c, b0, -1, 0.0), dog_teeth(c, b1, +1, 0.0)]
    col = prim.union([body] + teeth)
    col = col - hexprism(g["hex_af"] + fits.SHAFT_IN_BORE, s0 - 1, s1 + 1)
    # chamfer the hex entry
    return col


def dog_ring(g, c):
    s0, s1 = g["ring_s"]
    body = ring(g["ring_od"] / 2, g["ring_id"] / 2, s0, s1)
    teeth = dog_teeth(c, g["ring_face"], -1, 45.0)
    lw, lh, lr0, lr1 = g["ring_lug"]
    r = prim.union([body, teeth])
    cuts = []
    for k in range(3):                         # 3 key slots in the back face for the launch sleeve lugs
        a = 60.0 + 120.0 * k
        cuts.append(prim.place(prim.box(lr0 - 0.2, lr1 + 0.2, -lw / 2 - 0.2, lw / 2 + 0.2, s1 - lh - 0.2, s1 + 1),
                               tf.Rz(a)))
    for k in range(3):
        a = math.radians(120.0 * k)
        x, y = g["ring_screw_r"] * math.cos(a), g["ring_screw_r"] * math.sin(a)
        cuts.append(cyl(1.7, s0 - 1, s1 + 1, x, y, seg=24))
        cuts.append(cyl(3.0, s0 - 1, s0 + 3.2, x, y, seg=32))
    return prim.diff(r, cuts)


# ------------------------------------------------------------------------------------------------ drum
def lock_cam_poly(g, c):
    """Wrap pinion lock cam in the idler part frame: disc r_l with a scallop that faces the drum at home."""
    a = a_seg(c)
    home = pinion_angle(c["seg_start"], g, c)
    # drum centre direction from the idler (world-fixed): phi_idler + 180; in part frame: minus home angle
    dirw = math.radians(g["phi_idler"] + 180.0 - home)
    cx, cy = a * math.cos(dirw), a * math.sin(dirw)
    disc = Point(0, 0).buffer(g["lock_rl"], 128)
    scal = Point(cx, cy).buffer(g["lock_RL"] + g["lock_c"], 512)
    p = disc.difference(scal)
    if p.geom_type != "Polygon":
        p = max(p.geoms, key=lambda q: q.area)
    return p


def lock_layer_poly(g, c):
    """Drum lock layer: convex ring R_L minus the area the pinion lock cam sweeps (plus clearance), drum frame."""
    cam = lock_cam_poly(g, c).buffer(g["lock_c"], 32)
    ix, iy = idler_xy(g, c)
    dx, dy = drum_xy(g)
    t0 = c["seg_start"]
    t1 = c["seg_start"] + K.seg_span(c)
    sweeps = []
    th = t0
    while th <= t1 + 1e-9:
        psi = pinion_angle(th, g, c)
        p = affinity.rotate(cam, psi, origin=(0, 0))
        p = affinity.translate(p, ix - dx, iy - dy)
        p = affinity.rotate(p, -th, origin=(0, 0))
        sweeps.append(p)
        th += 0.5
    disc = Point(0, 0).buffer(g["lock_RL"], 512)
    out = disc.difference(unary_union(sweeps))
    out = out.union(Point(0, 0).buffer(g["neck_d"] / 2 + 6.0, 64))
    if out.geom_type != "Polygon":
        out = max(out.geoms, key=lambda q: q.area)
    return out


def groove_cutter(g, c):
    """Swept follower pin (groove width) in the drum frame."""
    r_out = g["barrel_r"] + 2.0
    r_in = g["barrel_r"] - g["groove_d"]
    w = g["groove_w"] / 2
    th = c["stop_lo"]
    pts = []
    while th <= c["stop_hi"] + 1e-9:
        pts.append(th)
        th += 1.0
    if pts[-1] < c["stop_hi"]:
        pts.append(c["stop_hi"])
    pins = []
    for t in pts:
        a = g["phi_pin"] - t
        s = fork_planes(g)["pin"] - K.collar_x(t, c) * (1.0 + g["fork_over"] / c["stroke"])
        pin = prim.cyl(w, r_out - r_in, z=r_in, seg=24)
        pins.append(prim.place(pin, tf.T(0, 0, s) @ tf.Rz(a) @ tf.Ry(90)))
    hulls = [prim.union([pins[i], pins[i + 1]]).hull() for i in range(len(pins) - 1)]
    return prim.union(hulls)


def nub_centre_r(g, c, bit, alpha):
    """Radius of the lever nub centre over drum-local angle alpha (designed path; the band is its inner envelope)."""
    th = theta_of_local(alpha, g["phi_sw"], c)
    return g["band_r0"] + g["nub_r"] + 0.3 + g["bump"] * K.code_lift(th, bit, c)


def band_poly(g, c, bit):
    path = polar_profile(lambda a: nub_centre_r(g, c, bit, a), n=1440)
    p = path.buffer(-(g["nub_r"] + 0.3), 16)
    if p.geom_type != "Polygon":
        p = max(p.geoms, key=lambda q: q.area)
    return p


def cam_radius(g, c, alpha):
    th = theta_of_local(alpha, g["phi_brake"], c)
    if th > c["stop_hi"]:                       # unreachable gap: blend back to the start value
        u = (th - c["stop_hi"]) / (c["stop_lo"] + 360.0 - c["stop_hi"])
        v = K.brake_lift(c["stop_hi"], c) * (1 - u) + K.brake_lift(c["stop_lo"], c) * u
    else:
        v = K.brake_lift(th, c)
    return g["cam_r0"] + v


def drum_a(g, c):
    """Drum part A (motor side), prints gear face down: N20 gear, segment gear, locking arc, brake cam, neck,
    hex spigot into drum B. Turns on the fixed axle."""
    a0, a1 = g["drumA_s"]
    parts = [cyl(g["neck_d"] / 2, a0, g["neck_s"][1])]
    dg = gears.spur_gear(c["drum_gear_m"], c["drum_gear_z"], g["dgear"][1] - g["dgear"][0], shift=c["drum_gear_x"])
    parts.append(dg.translate((0, 0, g["dgear"][0])))
    sg = gears.spur_gear(c["seg_m"], c["seg_z"], g["seg"][1] - g["seg"][0], present=seg_present(g, c))
    parts.append(prim.place(sg, tf.T(0, 0, g["seg"][0] - 0.01) @ tf.Rz(seg_phase(g, c))))
    parts.append(shp_extrude(lock_layer_poly(g, c), g["lock"][0] + 0.3, g["lock"][1]))
    parts.append(cyl(g["lock_RL"] - 6.0, g["seg"][1] - 0.01, g["lock"][0] + 0.31))
    parts.append(shp_extrude(polar_profile(lambda a: cam_radius(g, c, a)), g["cam"][0] - 0.01, g["cam"][1]))
    parts.append(hexprism(g["spig_af"], g["neck_s"][1] - 0.01, a1))
    d = prim.union(parts)
    return d - cyl((g["axle_d"] + fits.SHAFT_IN_BORE) / 2, a0 - 1, a1 + 1)


def drum_b(g, c):
    """Drum part B (outboard), prints barrel face down: groove 1 barrel, code bands A B C, hand lever hex boss."""
    b0, b1 = g["drumB_s"]
    barrel = cyl(g["barrel_r"], g["barrel"][0], g["barrel"][1])
    parts = [barrel - groove_cutter(g, c), cyl(g["band_r0"] - 1.0, g["barrel"][1] - 0.01, g["bands"][0][0] + 0.01)]
    for i, (s0, s1) in enumerate(g["bands"]):
        parts.append(shp_extrude(band_poly(g, c, i), s0 - 0.01, s1 + 0.01))
        nxt = g["bands"][i + 1][0] if i < 2 else g["hexboss"][0]
        parts.append(cyl(g["band_r0"] - 1.0, s1, nxt + 0.01))
    h0, h1, af = g["hexboss"]
    parts.append(hexprism(af, h0, h1))
    d = prim.union(parts)
    d = d - hexprism(g["spig_af"] + fits.HOLE_SLIDE, b0 - 1, g["drumA_s"][1] + 0.3)
    return d - cyl((g["axle_d"] + fits.SHAFT_IN_BORE) / 2, b0 - 1, b1 + 1)


def axle(g):
    """Fixed drum axle d10 (PPA-CF), clamped in brackets A and B; 1 mm print flat on +x."""
    dx, dy = drum_xy(g)
    a0, a1 = g["axle_s"]
    a = cyl(g["axle_d"] / 2, a0, a1, dx, dy)
    return a - prim.box(dx + g["axle_d"] / 2 - 1.0, dx + 10, dy - 10, dy + 10, a0 - 1, a1 + 1)


def idler(g, c):
    """Wrap pinion + lock cam + idler output gear + journal, one printed part (frame: idler axis, s along z)."""
    s0, s1 = g["idl"]
    face = g["seg"][1] - g["seg"][0]
    pin = gears.spur_gear(c["seg_m"], c["pin_z"], face, shift=c["pin_x"]).translate((0, 0, g["seg"][0]))
    cam = shp_extrude(lock_cam_poly(g, c), g["lock"][0], g["lock"][1])
    shaft = cyl(g["idl_j"] / 2, s0, s1)
    sh = prim.union([pin, cam, shaft])
    # D flat under the output gear (gear keyed by the flat, pushed against the cam shoulder side)
    flat = prim.box(g["idl_j"] / 2 - 1.0, 10, -10, 10, g["idl_gear"][0] - 2.0, g["idl_gear"][1] + 0.5)
    return sh - flat


def idler_gear(g, c):
    """Wrap idler output gear 42T m1.5 with a D bore onto the idler shaft flat (frame: idler axis)."""
    og = gears.spur_gear(c["idl_m"], c["idl_z"], g["idl_gear"][1] - g["idl_gear"][0]).translate(
        (0, 0, g["idl_gear"][0]))
    r = (g["idl_j"] + fits.HOLE_SLIDE) / 2
    bore = shp_extrude(Point(0, 0).buffer(r, 64).difference(
        Polygon([(g["idl_j"] / 2 - 1.0 + 0.1, -20), (20, -20), (20, 20), (g["idl_j"] / 2 - 1.0 + 0.1, 20)])),
        g["idl_gear"][0] - 1, g["idl_gear"][1] + 1)
    return og - bore


def n20_pinion(g, c):
    face = g["dgear"][1] - g["dgear"][0]
    p = gears.spur_gear(c["drum_gear_m"], c["n20_z"], face, shift=c["n20_x"])
    return p.translate((0, 0, g["dgear"][0])) - cyl(1.5 + 0.05, g["dgear"][0] - 1, g["dgear"][1] + 1, seg=24)


# ------------------------------------------------------------------------------------------------ fork, rail, brackets
def fork_planes(g):
    """Fork axial planes at NEUTRAL: bed plane sb (pads, yoke and tab sit on it), yoke top, follower pin s."""
    pw = (g["groove"][1] - g["groove"][0]) - 0.6
    sb = 0.5 * (g["groove"][0] + g["groove"][1]) - pw / 2
    return dict(pw=pw, sb=sb, yt=sb + g["fork_t"], pin=sb + g["pin_off"], gap=5.5, prong=4.0)


def fork(g, c):
    """Fork at NEUTRAL in L. Prints flat on the plane sb: the thin yoke (lower half ring round the collar groove,
    pads at 3 and 9 o'clock) is the spring; hub, arm to the rail sleeve, follower pin down into groove 1, a rigid post
    with the +s prong. The -s prong is the separate drum_fork_stop. Prongs stop the yoke tab at 5.5 mm either way;
    an M3 screw in each prong (heads set 3 mm from the tab) touches the M3 nut captive in the tab: SW_DOG."""
    F = fork_planes(g)
    sb, yt, pw = F["sb"], F["yt"], F["pw"]
    r0, r1 = g["yoke_r"]
    half = Point(0, 0).buffer(r1, 256).difference(Point(0, 0).buffer(r0, 256)).intersection(
        Polygon([(-60, -1.0), (60, -1.0), (60, 60), (-60, 60)]))
    yoke = shp_extrude(half, sb, yt)
    pads = []
    for sx in (-1, 1):
        x0, x1 = sorted((sx * (g["groove_r"] + 0.5), sx * (r0 + 0.5)))
        pads.append(prim.box(x0, x1, -4.0, 1.0, sb, sb + pw))
        pads.append(prim.box(min(sx * r0, sx * r1), max(sx * r0, sx * r1), -4.0, 1.0, sb, yt))
    dy = g["drum_off"] - g["barrel_r"]            # drum top (y) under the main axis
    rx, ry = g["rail_xy"]
    hub = prim.box(-6, 6, r1 - 1.0, dy - 0.6, sb, sb + g["hub_len"])
    arm = prim.box(-6, rx, ry - 4, dy - 0.6, sb, sb + g["hub_len"])
    sleeve = cyl(g["rail_d"] / 2 + 4.0, sb, sb + 20, rx, ry)
    pin = prim.place(cyl(g["pin_d"] / 2, 0, g["groove_d"] - 0.5 + 0.6, seg=32),
                     tf.T(0, dy - 0.6, F["pin"]) @ tf.Rx(-90))
    tab = prim.box(r0, r1 + 7.0, -1.0, 5.0, sb, yt)
    a0, a1 = yt + F["gap"], yt + F["gap"] + F["prong"]
    prong = prim.box(r1 + 1.5, r1 + 8.0, -1.0, ry - 3.0, a0, a1)
    post = prim.box(r1 + 1.5, r1 + 12.0, 20.0, ry - 3.0, sb, a1)
    f = prim.union([yoke] + pads + [hub, arm, sleeve, pin, tab, post, prong])
    f = f - cyl((g["rail_d"] + fits.HOLE_SLIDE) / 2, sb - 1, sb + 21, rx, ry)
    cx, cy = r1 + 4.0, 2.0
    f = f - prim.place(prim.extrude(hex_cs(5.6), yt - sb + 2, z=sb - 1), tf.T(cx, cy, 0))     # nut in the tab
    f = f - cyl(1.25, a0 - 1, a1 + 1, cx, cy, seg=24)                                          # M3 tap, prong
    f = f - prim.place(cyl(1.25, -1, 9, seg=24), tf.T(r1 + 12.0 - 8.0, 22.5, sb + 5.0) @ tf.Ry(90))  # stop screw
    return f


def fork_stop(g, c):
    """-s prong of the fork, screwed (M3, along x) to the outer face of the fork post."""
    F = fork_planes(g)
    sb, yt = F["sb"], F["yt"]
    r0, r1 = g["yoke_r"]
    rx, ry = g["rail_xy"]
    b0, b1 = sb - F["gap"] - F["prong"], sb - F["gap"]
    prong = prim.box(r1 + 1.5, r1 + 8.0, -1.0, 27.0, b0, b1)
    bridge = prim.box(r1 + 12.0, r1 + 16.0, 18.0, 27.0, b0, sb + 10.0)
    web = prim.box(r1 + 1.5, r1 + 16.0, 18.0, 27.0, b0, b1)
    st = prim.union([prong, bridge, web])
    cx, cy = r1 + 4.0, 2.0
    st = st - cyl(1.25, b0 - 1, b1 + 1, cx, cy, seg=24)
    st = st - prim.place(cyl(1.7, -1, 20, seg=24), tf.T(r1 + 10.0, 22.5, sb + 5.0) @ tf.Ry(90))
    return st


def rail(g):
    """Shift rail d10 with a 1 mm flat on the outboard side (+x) to print on; the fork sleeve rides the round."""
    rx, ry = g["rail_xy"]
    r = cyl(g["rail_d"] / 2, g["rail_s"][0], g["rail_s"][1], rx, ry)
    return r - prim.box(rx + g["rail_d"] / 2 - 1.0, rx + 10, ry - 10, ry + 10, g["rail_s"][0] - 1, g["rail_s"][1] + 1)


def bracket(g, c, which):
    """Bracket A (motor side) or B (outboard): drum journal bore, rail bore, feet on the deck."""
    s0, s1 = g["brA"] if which == "A" else g["brB"]
    dx, dy = drum_xy(g)
    rx, ry = g["rail_xy"]
    yd = deck_y(g)
    outline = unary_union([
        Point(dx, dy).buffer(g["axle_d"] / 2 + 6, 64),
        Point(rx, ry).buffer(g["rail_d"] / 2 + 5, 48),
        Polygon([(dx - 14, dy), (dx + 14, dy), (dx + 24, yd), (dx - 24, yd)]),
        Polygon([(rx - 7, ry), (rx + 7, ry), (dx + 14, dy + 4), (dx + 4, dy - 8)]),
        Polygon([(-32, yd - 6), (32, yd - 6), (32, yd), (-32, yd)]),
    ])
    b = shp_extrude(outline, s0, s1)
    cuts = [cyl((g["axle_d"] + fits.HOLE_PRESS) / 2, s0 - 1, s1 + 1, dx, dy),
            cyl((g["rail_d"] + fits.HOLE_PRESS) / 2, s0 - 1, s1 + 1, rx, ry)]
    for x in (-26.0, 26.0):
        cuts.append(prim.place(cyl(1.7, -1, 8, seg=24), tf.T(x, yd + 0.5, 0.5 * (s0 + s1)) @ tf.Rx(90)))
    return prim.diff(b, cuts)


def bracket_idler(g, c):
    s0, s1 = g["brI"]
    ix, iy = idler_xy(g, c)
    yd = deck_y(g)
    outline = unary_union([Point(ix, iy).buffer(g["idl_j"] / 2 + 6, 64),
                           Polygon([(ix - 10, iy), (ix + 10, iy), (ix + 18, yd), (ix - 18, yd)])])
    outline = unary_union([outline, Polygon([(ix - 26, yd - 6), (ix + 26, yd - 6), (ix + 26, yd), (ix - 26, yd)])])
    b = shp_extrude(outline, s0, s1)
    cuts = [cyl((g["idl_j"] + fits.SHAFT_IN_BORE) / 2, s0 - 1, s1 + 1, ix, iy)]
    for x in (ix - 21.0, ix + 21.0):
        cuts.append(prim.place(cyl(1.7, -1, 8, seg=24), tf.T(x, yd + 0.5, 0.5 * (s0 + s1)) @ tf.Rx(90)))
    return prim.diff(b, cuts)


def sw_x0(g):
    """Switch block front face (x from the drum axis): lever beam outer face + M3 head 3.0 + contact gap 1.1."""
    rx = g["band_r0"] + g["nub_r"] + 0.3 + 1.0
    bx = g["bump"] + 0.3 - g["nub_r"] - 0.3 + 0.8
    return rx + bx + 2.4 + 3.0 + 1.1


def switch_block(g, c):
    """Block on the +X side of the code bands: lever feet screw to it, an M3 nut per lever is the fixed contact."""
    dx, dy = drum_xy(g)
    yd = deck_y(g)
    s0 = g["bands"][0][0] - 3.0
    s1 = g["bands"][-1][1]
    x0 = dx + sw_x0(g)
    blk = prim.box(x0, x0 + 14.0, dy + 2.0, yd, s0, s1)
    foot = prim.box(x0 - 2.0, x0 + 24.0, yd - 6.0, yd, s0, s1)
    b = prim.union([blk, foot])
    cuts = []
    for i, (w0, w1) in enumerate(g["bands"]):
        sc = 0.5 * (w0 + w1)
        # M3 nut contact facing the lever screw head (nut flush with the face), wire hole behind it
        cuts.append(prim.place(prim.extrude(hex_cs(5.6), 2.6, z=-0.01), tf.T(x0, dy + 8.0, sc) @ tf.Ry(90)))
        cuts.append(prim.place(cyl(1.7, -1, 16, seg=24), tf.T(x0, dy + 8.0, sc) @ tf.Ry(90)))
        # lever foot screw (tapped, vertical)
        rx, ry = sw_lever_root(g, i)
        bx = g["bump"] + 0.3 - g["nub_r"] - 0.3 + 0.8
        cuts.append(cyl(1.25, ry + 1.0, ry + 20.0, rx + bx + 6.0, 0, seg=24).translate((0, 0, 0)) if False else
                    prim.place(cyl(1.25, -1, 14, seg=24), tf.T(rx + bx + 6.0, ry + 9.0, sc) @ tf.Rx(-90)))
    for sc in (s0 + 5.0, s1 - 5.0):            # 2 x M3 to the deck
        cuts.append(prim.place(cyl(1.7, -1, 8, seg=24), tf.T(x0 + 19.0, yd + 0.5, sc) @ tf.Rx(90)))
    return prim.diff(b, cuts)


def sw_lever(g, c, i):
    """Code switch lever i in its joint frame: root at the origin, round nub (radius nub_r, axis along s) at the tip
    (-1, -sw_L) riding its band. PLA flexure, clamped to the switch block by its foot (M3)."""
    b0, b1 = g["bands"][i]
    L = g["sw_L"]
    w0, w1 = b0 + 0.6, b1 - 0.6
    bx = g["bump"] + 0.3 - g["nub_r"] - 0.3 + 0.8      # beam inner face clears the bump tops (r0 + bump) by 0.8
    beam = prim.box(bx, bx + 2.4, -L, 6.0, w0, w1)
    nub = cyl(g["nub_r"], w0, w1, -1.0, -L, seg=32)
    arm = prim.box(-1.0, bx + 0.5, -L - 1.5, -L + 1.5, w0, w1)
    foot = prim.box(bx, bx + 10.0, 2.0, 10.0, w0, w1)
    lv = prim.union([beam, nub, arm, foot])
    # contact: M3 screw tapped through the beam 8 mm below the nub, head toward the block nut (gap 1.1 at rest)
    lv = lv - prim.place(cyl(1.25, -1, 6, seg=24), tf.T(bx - 0.5, -L + 8.0, 0.5 * (w0 + w1)) @ tf.Ry(90))
    lv = lv - prim.place(cyl(1.7, -1, 20, seg=24), tf.T(bx + 6.0, 6.0, 0.5 * (w0 + w1)) @ tf.Rx(90))   # foot screw
    return lv


def sw_lever_root(g, i):
    dx, dy = drum_xy(g)
    x = dx + g["band_r0"] + g["nub_r"] + 0.3 + 1.0
    return (x, dy + g["sw_L"])


def magnet_holder(g):
    """AS5600 magnet cap on the outboard end of the main shaft spigot: d6 x 2.5 diametric magnet pocket on the end
    face, M3 cross screw through cap and spigot. The frame owns the AS5600 board bracket (chip 2 mm off the cap face)."""
    e = g["spig_end"]
    cap = cyl(10.0, e - 12.0, e + 4.0)
    cuts = [cyl(g["spig_d"] / 2 + fits.HOLE_SLIDE / 2, e - 13.0, e + 0.01),
            cyl(3.0 + 0.1, e + 4.0 - 2.6, e + 5.0, seg=48),
            prim.place(cyl(1.7, -15, 15, seg=24), tf.T(0, 0, e - 6.0) @ tf.Ry(90))]
    return prim.diff(cap, cuts)


def lever_hand(g):
    h0, h1, af = g["hexboss"]
    hub = cyl(af / 2 + 5, h0 + 0.5, h1)
    arm = prim.box(0, 28, -6, 6, h0 + 0.5, h1)
    tip = cyl(6, h0 + 0.5, h1, 28, 0)
    L = prim.union([hub, arm, tip])
    L = L - cyl(3.5, h0, h1 + 1, 27, 0)            # finger hole (or an M3 bolt as a knob)
    return L - hexprism(af + fits.HOLE_SLIDE, h0, h1 + 1)


def n20_model(g, c, P):
    """Right-angle worm N20 envelope (bought), in the N20 joint frame (output axis = z)."""
    N = getattr(P, "N20_WORM", {}) or {}
    bw = float(N.get("box_w", 9.94)); bh = float(N.get("box_h", 11.93))
    ln = float(N.get("len_no_shaft", 33.04)); tip = float(N.get("shaft_tip_to_tip", 29.97))
    zc = g["dgear"][1] + 0.5 + bw / 2
    box = prim.box(-6.0, 6.0, -bh / 2, bh / 2, zc - bw / 2, zc + bw / 2)
    mot = prim.box(6.0, ln - 6.0, -5.0, 5.0, zc - bw / 2 + 0.5, zc + bw / 2 - 0.5)
    sh = cyl(1.5, zc - tip / 2, zc + tip / 2, seg=24)
    return prim.union([box, mot, sh])


def n20_cradle(g, c, P):
    """Lower half shell round the N20 envelope plus a post to the deck. Slotted foot (M3 thumb screw) lets the
    pinion slide out of mesh for the hand lever."""
    nx, ny = n20_xy(g, c)
    env = prim.place(n20_model(g, c, P), tf.T(nx, ny, 0) @ tf.Rz(g["phi_n20"]))
    b = env.bounding_box()
    yd = deck_y(g)
    ymid = 0.5 * (b[1] + b[4])
    shell = prim.box(b[0] - 3.0, b[3] + 3.0, ymid, yd, b[2] + 0.6, b[5] - 0.6)
    cuts = [env,
            cyl(gears.spur_info(c["drum_gear_m"], c["drum_gear_z"], c["drum_gear_x"])["r_tip"] + 1.5,
                -1, 200, 0.0, g["drum_off"]),
            cyl(gears.spur_info(c["drum_gear_m"], c["n20_z"], c["n20_x"])["r_tip"] + 1.0, b[2] - 1,
                g["dgear"][1] + 0.5, nx, ny)]
    cr = prim.diff(shell, cuts)
    parts = cr.decompose()
    cr = max(parts, key=lambda m: m.volume())
    # hand fallback: M3 thumb screw in a slot (slide 4 mm in +x: the pinion leaves the mesh); 2 x M3 strap screws
    zc = 0.5 * (b[2] + b[5])
    xs = b[3] - 2.0
    slot = prim.union([prim.place(cyl(1.8, -1, 30, seg=24), tf.T(xs + dxs, yd + 0.5, zc) @ tf.Rx(90))
                       for dxs in np.linspace(0.0, 4.0, 9)])
    straps = [prim.place(cyl(1.25, -1, 12, seg=24), tf.T(x, ymid - 0.01, zc) @ tf.Rx(-90))
              for x in (b[0] - 1.5, b[3] + 1.5)]
    return prim.diff(cr, [slot] + straps)


def n20_strap(g, c, P):
    """Strap over the N20 (2 x M3 into the cradle): holds the motor down against the torque reaction."""
    nx, ny = n20_xy(g, c)
    env = prim.place(n20_model(g, c, P), tf.T(nx, ny, 0) @ tf.Rz(g["phi_n20"]))
    b = env.bounding_box()
    ymid = 0.5 * (b[1] + b[4])
    zc = 0.5 * (b[2] + b[5])
    st = prim.box(b[0] - 3.0, b[3] + 3.0, b[1] - 3.0, ymid, zc - 4.0, zc + 4.0)
    st = st - env
    st = st - cyl(gears.spur_info(c["drum_gear_m"], c["drum_gear_z"], c["drum_gear_x"])["r_tip"] + 1.5,
                  -1, 200, 0.0, g["drum_off"])
    for x in (b[0] - 1.5, b[3] + 1.5):
        st = st - prim.place(cyl(1.7, -1, 30, seg=24), tf.T(x, ymid + 0.01, zc) @ tf.Rx(90))
    parts = st.decompose()
    return max(parts, key=lambda m: m.volume())


# ------------------------------------------------------------------------------------------------ module contract
COL = dict(ppa="#3a3f45", pla="#c8ccd2", gear="#d97a00", drum="#3d7cc9", fork="#7fb069", sw="#e0c040")


def parts(P):
    g, c = G(P), C(P)
    dx, dy = drum_xy(g)
    I = tf.I4
    out = []
    ch = coupler_hub(g, P)
    out.append(Part("drum_coupler_hub", ch, "PPA-CF", joint="drum_main", print_tf=bed(ch, I), color=COL["ppa"],
                    notes="Coupler hub on the wiper shaft (bore parametric: WIPER), M8 nut inside, shear pin hole"))
    cp = coupler(g, P)
    out.append(Part("drum_coupler", cp, "PPA-CF", joint="drum_main", print_tf=bed(cp, I), color=COL["ppa"],
                    notes="Coupler: 5 N m shear pin limiter (D8), take-up bus gear 40T m1.5 (D15), hex socket, M3 cross screw"))
    sp = shear_pin(g)
    out.append(Part("drum_shear_pin", sp, "PLA", joint="drum_main", print_tf=bed(sp, tf.Ry(90)), color=COL["sw"],
                    qty=6, notes="Torque limiter shear pin d4 (about 5 N m); print 6, 5 are spares"))
    sh = main_shaft(g)
    out.append(Part("drum_shaft", sh, "PPA-CF", joint="drum_main", print_tf=bed(sh, tf.Rx(90) @ tf.Ry(90)),
                    color=COL["ppa"], notes="Main shaft: hex 22 AF (collar, REEL sleeve), d12 spigot into the launch sleeve; print on a hex flat"))
    rs = reel_sleeve(g)
    out.append(Part("drum_sleeve_reel", rs, "PPA-CF", joint="drum_main", print_tf=bed(rs, I), color=COL["ppa"],
                    notes="REEL sleeve: keyed on the hex, journal for the REEL gear, flange thrusts on the coupler"))
    rg = reel_gear(g, c)
    out.append(Part("drum_gear_reel", rg, "PPA-CF", joint="drum_out_reel", print_tf=bed(rg, I), color=COL["gear"],
                    notes="REEL output gear m3 24T, free on its sleeve, dog face toward the collar"))
    co = collar(g, c)
    out.append(Part("drum_collar", co, "PPA-CF", joint="drum_collar", print_tf=bed(co, I), color=COL["ppa"],
                    notes="Dog collar: hex bore, 4 back-cut roofed dogs each face, fork groove"))
    dr = dog_ring(g, c)
    out.append(Part("drum_dogring_launch", dr, "PPA-CF", joint="drum_out_launch", print_tf=bed(dr, tf.Rx(180)),
                    color=COL["gear"], notes="LAUNCH dog ring: 3 lugs + 3 x M3 onto the cad-launch launch sleeve flange"))
    da = drum_a(g, c)
    out.append(Part("drum_drum_a", da, "PLA", joint="drum_drum", print_tf=bed(da, I), color=COL["drum"],
                    notes="Drum A: N20 gear 120T m0.8, wrap segment gear + locking arc, brake cam, hex spigot"))
    dbb = drum_b(g, c)
    out.append(Part("drum_drum_b", dbb, "PLA", joint="drum_drum", print_tf=bed(dbb, I), color=COL["drum"],
                    notes="Drum B: shift groove barrel, Gray code bands A B C, hand lever hex boss"))
    ax = axle(g)
    out.append(Part("drum_axle", ax, "PPA-CF", joint="drum_base", print_tf=bed(ax, tf.Ry(90)), color=COL["ppa"],
                    notes="Fixed drum axle d10, pressed into brackets A and B"))
    ig = idler_gear(g, c)
    out.append(Part("drum_idler_gear", ig, "PPA-CF", joint="drum_idler", print_tf=bed(ig, I), color=COL["gear"],
                    notes="Wrap idler output gear 42T m1.5 (D bore) to the cad-reel cup ring 24T"))
    idl = idler(g, c)
    out.append(Part("drum_idler", idl, "PPA-CF", joint="drum_idler", print_tf=bed(idl, I), color=COL["gear"],
                    notes="Wrap idler shaft: pinion 10T m1.25 (locked at home by the drum lock arc), lock cam, D flat for the output gear"))
    npn = n20_pinion(g, c)
    out.append(Part("drum_n20_pinion", npn, "PPA-CF", joint="drum_n20", print_tf=bed(npn, I), color=COL["gear"],
                    notes="N20 pinion 10T m1, pressed on the N20 D shaft"))
    out.append(Part("drum_n20", n20_model(g, c, P), "bought", joint="drum_n20_body", color="#8a8f96",
                    bought_ref="N20 right-angle self-locking worm gearmotor 12 V 27 rpm (have)",
                    notes="Selector drum motor, DRV8833 #3 at 9 V"))
    cr = n20_cradle(g, c, P)
    out.append(Part("drum_n20_cradle", cr, "PLA", joint="drum_base", print_tf=bed(cr, I), color=COL["pla"],
                    notes="N20 cradle: slotted foot, slide out of mesh for the hand lever"))
    ns = n20_strap(g, c, P)
    out.append(Part("drum_n20_strap", ns, "PLA", joint="drum_base", print_tf=bed(ns, tf.Rx(180)), color=COL["pla"],
                    notes="Strap over the N20, 2 x M3 into the cradle"))
    fk = fork(g, c)
    out.append(Part("drum_fork", fk, "PLA", joint="drum_fork", print_tf=bed(fk, I), color=COL["fork"],
                    notes="Compliant shift fork: thin yoke is the spring (about 1.6 N/mm), follower pin in groove 1"))
    fs = fork_stop(g, c)
    out.append(Part("drum_fork_stop", fs, "PLA", joint="drum_fork", print_tf=bed(fs, tf.Ry(90)), color=COL["fork"],
                    notes="Fork -s prong (yoke hard stop and SW_DOG screw), 1 x M3 into the fork post"))
    rl = rail(g)
    out.append(Part("drum_rail", rl, "PPA-CF", joint="drum_base", print_tf=bed(rl, tf.Ry(90)), color=COL["ppa"],
                    notes="Shift rail d10, pressed into brackets A and B"))
    for w in ("A", "B"):
        b = bracket(g, c, w)
        out.append(Part(f"drum_bracket_{w.lower()}", b, "PLA", joint="drum_base", print_tf=bed(b, I), color=COL["pla"],
                        notes=f"Drum bracket {w}: drum journal, shift rail, 2 x M3 to the deck"))
    bi = bracket_idler(g, c)
    out.append(Part("drum_bracket_idler", bi, "PLA", joint="drum_base", print_tf=bed(bi, I), color=COL["pla"],
                    notes="Wrap idler bearing post, 2 x M3 to the deck"))
    sb = switch_block(g, c)
    out.append(Part("drum_switch_block", sb, "PLA", joint="drum_base", print_tf=bed(sb, I), color=COL["pla"],
                    notes="Code switch block: 3 lever clamps, 3 M3 nut contacts"))
    for i in range(3):
        lv = sw_lever(g, c, i)
        out.append(Part(f"drum_sw_lever_{i}", lv, "PLA", joint=f"drum_sw{i}", print_tf=bed(lv, I), color=COL["sw"],
                        notes=f"Code switch lever b{i}: flexure, M3 screw head contact"))
    mh = magnet_holder(g)
    out.append(Part("drum_magnet_cap", mh, "PLA", joint="drum_main", print_tf=bed(mh, tf.Rx(180)), color=COL["pla"],
                    notes="AS5600 magnet cap on the main shaft end (6 x 2.5 diametric magnet, M3 cross screw)"))
    lh = lever_hand(g)
    out.append(Part("drum_hand_lever", lh, "PLA", joint="drum_drum", print_tf=bed(lh, I), color=COL["pla"],
                    notes="Hand lever on the drum hex boss (fallback; slide the N20 out of mesh first)"))
    return out


def joints(P):
    g, c = G(P), C(P)
    dx, dy = drum_xy(g)
    ix, iy = idler_xy(g, c)
    nx, ny = n20_xy(g, c)
    js = [
        Joint("drum_base", None, "fixed", (0.0, g["s0_world_y"], g["main_height"]), (0, 0, 1), base=tf.Rx(-90)),
        Joint("drum_main", "drum_base", "rev", (0, 0, 0), (0, 0, 1)),
        Joint("drum_out_reel", "drum_base", "rev", (0, 0, 0), (0, 0, 1)),
        Joint("drum_out_launch", "drum_base", "rev", (0, 0, 0), (0, 0, 1)),
        Joint("drum_collar_slide", "drum_base", "slide", (0, 0, 0), (0, 0, 1), limits=(-c["stroke"] - 0.01, c["stroke"] + 0.01)),
        Joint("drum_collar", "drum_collar_slide", "rev", (0, 0, 0), (0, 0, 1)),
        Joint("drum_fork", "drum_base", "slide", (0, 0, 0), (0, 0, 1),
              limits=(-c["stroke"] - g["fork_over"] - 0.01, c["stroke"] + g["fork_over"] + 0.01)),
        Joint("drum_drum", "drum_base", "rev", (dx, dy, 0), (0, 0, 1), limits=(c["stop_lo"], c["stop_hi"])),
        Joint("drum_idler", "drum_base", "rev", (ix, iy, 0), (0, 0, 1)),
        Joint("drum_n20", "drum_base", "rev", (nx, ny, 0), (0, 0, 1)),
        Joint("drum_n20_body", "drum_base", "fixed", (nx, ny, 0), (0, 0, 1), base=tf.Rz(g["phi_n20"])),
    ]
    for i in range(3):
        rx, ry = sw_lever_root(g, i)
        js.append(Joint(f"drum_sw{i}", "drum_base", "rev", (rx, ry, 0), (0, 0, 1), limits=(-0.5, 8.0)))
    return js


def drum_theta(S, c):
    if "drum_deg" in S and S["drum_deg"] is not None:
        return float(S["drum_deg"])
    m = str(S.get("mode", "LAUNCH")).upper()
    return {"LAUNCH": c["launch"], "NEUTRAL": c["neutral"], "REEL": c["reel"], "WRAPPED": c["wrapped"]}.get(m, c["launch"])


def lever_deg(theta, g, c, i):
    """Code lever deflection (deg about its root): the nub centre follows nub_centre_r exactly."""
    lift = nub_centre_r(g, c, i, g["phi_sw"] - theta) - (g["band_r0"] + g["nub_r"] + 0.3)
    return math.degrees(math.asin(min(1.0, lift / g["sw_L"])))


def kin(P, S):
    g, c = G(P), C(P)
    th = drum_theta(S, c)
    w = float(S.get("wiper_deg", 0.0))
    x = K.collar_x(th, c)
    md = K.mode(th, c)
    v = {
        "drum_drum": th,
        "drum_main": w,
        "drum_collar": w,
        "drum_collar_slide": -x,
        "drum_fork": -x * (1.0 + g["fork_over"] / c["stroke"]),
        "drum_idler": pinion_angle(th, g, c),
        "drum_n20": n20_angle(th, g, c),
        "drum_out_reel": w if md in ("REEL", "WRAPPED") else float(S.get("reel_out_deg", w)),
        "drum_out_launch": w if md == "LAUNCH" else float(S.get("launch_out_deg", w)),
    }
    for i in range(3):
        v[f"drum_sw{i}"] = lever_deg(th, g, c, i)
    return v


def allowed(P):
    return [
        Allow("drum_shaft", "drum_coupler", "hex spigot in the coupler socket (sliding fit)", max_vol=1.0),
        Allow("drum_rail", "drum_bracket_*", "rail pressed into the brackets"),
        Allow("drum_n20_pinion", "drum_n20", "pinion pressed on the N20 shaft"),
        Allow("drum_n20", "drum_n20_cradle", "N20 seated in its cradle", max_vol=1.0),
        Allow("drum_n20", "drum_n20_strap", "strap over the N20", max_vol=1.0),
        Allow("drum_idler_gear", "drum_idler", "gear on the D flat", max_vol=1.0),
        Allow("drum_sw_lever_*", "drum_switch_block", "lever foot clamped to the block (M3)"),
        Allow("drum_hand_lever", "drum_drum_b", "lever on the hex boss", max_vol=1.0),
        Allow("drum_axle", "drum_bracket_*", "axle pressed into the brackets"),
        Allow("drum_drum_a", "drum_drum_b", "hex spigot in its socket", max_vol=1.0),
        Allow("drum_shear_pin", "drum_coupler*", "shear pin in its holes", max_vol=2.0),
        Allow("drum_fork_stop", "drum_fork", "stop screwed to the fork post", max_vol=1.0),
        Allow("drum_magnet_cap", "drum_shaft", "cap on the spigot end", max_vol=1.0),
    ]


def clearances(P):
    return [
        Clearance("drum_drum_a", "drum_idler", 0.0, "segment gear, locking arc and wrap pinion"),
        Clearance("drum_drum_a", "drum_idler_gear", 0.8, "idler output gear over the drum neck"),
        Clearance("drum_drum_a", "drum_n20_pinion", 0.0, "N20 mesh"),
        Clearance("drum_drum_b", "drum_fork", 0.05, "follower pin in groove 1"),
        Clearance("drum_drum_b", "drum_sw_lever_*", 0.0, "code levers ride the bands"),
        Clearance("drum_collar", "drum_gear_reel", 0.0, "dogs"),
        Clearance("drum_collar", "drum_dogring_launch", 0.0, "dogs"),
    ]


def audit_states(P):
    c = C(P)
    out = []
    th = int(math.ceil(c["stop_lo"]))
    while th <= int(math.floor(c["stop_hi"])):
        out.append(dict(phase="drum_sweep", t=0.0, drum_deg=float(th), wiper_deg=0.0))
        th += 1
    for w in range(0, 360, 15):
        for m in ("LAUNCH", "NEUTRAL", "REEL", "WRAPPED"):
            out.append(dict(phase="drum_wiper", t=0.0, drum_deg=c[m.lower()], wiper_deg=float(w)))
    return out


def grounded(P):
    # coupler: on the wiper shaft (frame motor); dog ring: bolted to the cad-launch sleeve (frame outboard seat)
    return ["drum_bracket_*", "drum_switch_block", "drum_n20_cradle", "drum_coupler", "drum_dogring_launch"]


def steps(P):
    """Assembly steps, in order, plain language, hardware per step."""
    return [
        dict(title="Drum: brackets, axle and rail",
             text="Bolt bracket A to the deck (2 x M3 x 12). Push the drum axle into bracket A (flat facing forward) and "
                  "the shift rail into its hole (flat facing forward). Do not fit bracket B yet.",
             parts=["drum_bracket_a", "drum_axle", "drum_rail"]),
        dict(title="Drum: fork",
             text="Press an M3 nut into the hex hole in the yoke tab. Slide the fork sleeve onto the rail, yoke facing "
                  "the motor side. Screw the fork stop to the fork post (1 x M3 x 16). Thread an M3 x 12 into each "
                  "prong until its head is 3 mm from the nut (a 3 mm drill shank is the gauge). Battery unplugged: "
                  "one wire on the nut (GND), one wire joining both screws (SW_DOG).",
             parts=["drum_fork", "drum_fork_stop"]),
        dict(title="Drum: idler",
             text="Push the idler output gear onto the idler shaft flat (gear teeth face away from the pinion). Bolt "
                  "the idler post to the deck (2 x M3 x 12) and slide the idler journal into it, pinion toward the "
                  "motor.",
             parts=["drum_idler_gear", "drum_idler", "drum_bracket_idler"]),
        dict(title="Drum: drum A and drum B",
             text="Turn the idler until the flat-sided lock cam faces the drum position. Slide drum A onto the axle, "
                  "gear first, with mark L at the top: its lock ring must slip into the lock cam scallop (the idler "
                  "can then no longer turn). Slide drum B on, hex socket over the drum A spigot, so the fork pin "
                  "drops into the groove. Press bracket B onto the axle and rail ends and bolt it down (2 x M3 x 12). "
                  "The drum should turn by hand with no axial play over 0.5 mm.",
             parts=["drum_drum_a", "drum_drum_b", "drum_bracket_b"]),
        dict(title="Drum: N20",
             text="Press the pinion onto the N20 output shaft (flat to flat), flush with the shaft end. Lay the N20 in "
                  "the cradle, fit the strap (2 x M3 x 16). Put the cradle on the deck with the M3 thumb screw in its "
                  "slot, push it toward the drum until the pinion meshes with a hair of play, tighten.",
             parts=["drum_n20_pinion", "drum_n20", "drum_n20_cradle", "drum_n20_strap"]),
        dict(title="Drum: code switches",
             text="Press an M3 nut into each of the 3 front pockets of the switch block and bolt the block to the deck "
                  "(2 x M3 x 12). Screw each lever foot to the block (1 x M3 x 12), nub on its band. Thread an M3 x 8 "
                  "into each lever, head toward the nut, until the gap is 1.1 mm with the nub on a low part of the "
                  "band (feeler: a strip of 1 mm card). Battery unplugged: nuts to GND, screws to SW_DRUM_A (nearest "
                  "the motor), SW_DRUM_B, SW_DRUM_C.",
             parts=["drum_switch_block", "drum_sw_lever_0", "drum_sw_lever_1", "drum_sw_lever_2"]),
        dict(title="Main shaft: coupler and limiter",
             text="Push the coupler hub onto the wiper output shaft (flats in line). Drop the M8 nut onto the thread "
                  "and tighten it with a 13 mm socket. Slide the outer coupler over the hub, line up the holes and "
                  "push a shear pin in (head outside). Keep the 5 spare pins with the base.",
             parts=["drum_coupler_hub", "drum_coupler", "drum_shear_pin"]),
        dict(title="Main shaft: stack",
             text="Push the REEL sleeve into the REEL gear from the gear's plain side (the lip stops it). On the "
                  "bench, slide onto the main shaft from the spigot end: the sleeve and gear (flange first), then the "
                  "collar (either way round). Lower the shaft so the collar groove drops between the fork pads, push "
                  "the hex end into the coupler socket, fit the M3 x 40 cross screw and nyloc. The REEL gear must "
                  "spin free on its sleeve.",
             parts=["drum_shaft", "drum_collar", "drum_gear_reel", "drum_sleeve_reel"]),
        dict(title="Main shaft: LAUNCH dog ring",
             text="Fit the LAUNCH dog ring on the launch sleeve flange (sleeve lugs in the ring slots, 3 x M3 x 12 "
                  "into the nut traps) before the launch sleeve goes on the spigot (launcher steps).",
             parts=["drum_dogring_launch"]),
        dict(title="Main shaft: magnet cap and hand lever",
             text="Glue the AS5600 magnet into the cap, push the cap on the spigot end, fit the M3 x 25 cross screw. "
                  "Slide the hand lever onto the drum hex boss. Hand use: battery unplugged, slacken the line, loosen "
                  "the N20 thumb screw, slide the cradle 4 mm away from the drum, turn the lever to the mark, slide "
                  "the cradle back and tighten.",
             parts=["drum_magnet_cap", "drum_hand_lever"]),
    ]
