"""Drum subsystem (worker cad-drum): main shaft, coupler, dog clutch, REEL output gear, LAUNCH dog ring,
N20 selector drum with groove 1 (fork), track 2 (reel brake preload cam), Gray code bands, wrap segment gear with
locking arc, wrap idler, compliant fork, shift rail, brackets, switch levers, N20 cradle, hand lever.

Local frame L (joint drum_base): z along the main axis (world +Y, s = distance from the wiper boss face),
x = world +X (forward), y = world -Z (DOWN). The drum axis is at L (0, DRUM_OFF, z). Timing, codes and loads are in
_drum_kin.py (shared with firmware docs and tests). Notes: drum_NOTES.md.
"""
import math

import numpy as np
from shapely.geometry import Point, Polygon, LineString
from shapely.ops import unary_union
from shapely import affinity

from cad.lib import Part, Joint, Allow, Clearance, prim, tf, fits, gears
from cad.lib.clutch import _polar_tooth
from cad.parts import _drum_kin as K

# ------------------------------------------------------------------------------------------------ geometry numbers
GEOM = dict(
    # placement (BASE_SPEC v1.0 / params.LAYOUT; G() reads params when present)
    main_origin=(0.0, 0.0, 0.0),   # world point of s = 0 on the main axis (adapter plate face)
    drum_xy=(60.0, 112.0),         # drum axis in L (x forward, y DOWN): world (60, z -112)
    deck_y=165.0,                  # deck top in L (world z -165)
    cap_xy=(-18.66, 69.64),        # cad-reel capstan axis in L (for the wrap idler mesh)
    # coupler (D8 shear pin limiter, D15 bus gear)
    cpl_s=(0.5, 48.0), cpl_od=40.0, sock_s=30.0,
    hub_od=26.0, hub_s=(0.5, 21.0), nut_s=16.0, shear_s=13.0, shear_d=4.0,
    bus_m=1.5, bus_z=50, bus_s=(40.0, 48.0),
    idx_s=(3.0, 9.0), idx_r0=20.0, idx_bump=2.5, idx_n=6, idx_w=24.0, idx_ramp=4.0,   # D12 SW_INDEX cam
    # main shaft
    hex_af=22.0, hex_s=(30.0, 106.0), spig_d=12.0, spig_end=198.5, xpin_s=35.0,
    cap_s=(195.0, 201.5),      # D23: magnet face at y 201.5, outboard of the launch sleeve (y 111..193)
    # REEL sleeve and gear (params.DRUM.reel_output m2 50T face 12, plane 50.5..62.5)
    rsl_flange=(48.0, 50.0, 44.0), rsl_journal=(50.0, 62.0, 30.0),
    out_m=2.0, out_z=50, out_face=12.0, rgear_s=50.5,
    dog_face_reel=64.5,     # REEL gear dog roots (hub 62.5..64.5); tips at +dog_h
    collar_s=(72.0, 98.0), collar_od=60.0, groove=(82.0, 88.0), groove_r=26.0,
    ring_face=105.5,        # LAUNCH dog ring dog roots; tips at -dog_h (100.5)
    ring_s=(105.5, 111.0), ring_od=62.0, ring_id=27.0, ring_lug=(6.0, 3.0, 15.0, 21.0), ring_screw_r=28.5,
    # drum: two printed parts on a fixed d10 axle
    axle_d=10.0, axle_s=(1.0, 142.0), drumA_s=(8.0, 83.0), drumB_s=(77.3, 135.7), spig_af=16.0,
    dgear=(8.0, 16.0), cam=(18.0, 29.0), seg=(52.0, 60.0), lock=(60.0, 66.0), neck_d=20.0, neck_s=(29.0, 77.0),
    cam_r0=30.0, barrel=(77.3, 106.0), barrel_r=30.0, groove_w=6.4, groove_d=4.0,
    bands=((108.0, 114.0), (115.0, 121.0), (122.0, 128.0)), band_r0=22.0, bump=3.0, nub_r=1.5,
    hexboss=(128.7, 135.7, 14.0),
    # fork (yoke round the collar groove, long carrier to the drum groove)
    fork_s=85.0, fork_over=0.0, fork_t=2.8, yoke_r=(31.0, 37.0), pin_d=6.0, pin_off=9.5, hub_len=14.0,
    rail_xy=(25.0, 58.0), rail_d=10.0, rail_s=(1.0, 142.0),
    dogpost_s=(63.0, 71.0), dog_touch=0.5,       # SW_DOG_L / SW_DOG_R close 0.5 mm before full engagement
    # world-fixed angles round the drum axis (L xy plane, deg; y is down). phi_pin and phi_idler are derived in G()
    phi_pin=-118.0, phi_idler=-170.8, phi_brake=180.0, phi_sw=0.0, phi_n20=-40.0, n20_motor_deg=-90.0,
    phi_idx=0.0,
    # brackets (s)
    brA=(1.0, 7.7), brB=(136.0, 142.0), brI=(78.0, 85.0),
    idl=(40.0, 86.0), idl_gear=(68.5, 76.5), idl_j=10.0, idler_side=1, brI2=(40.5, 46.5),
    lock_RL=38.0, lock_rl=11.0, lock_c=0.25,
    sw_L=30.0,
)


def _circ2(p0, r0, p1, r1, side):
    d = math.dist(p0, p1)
    a = (r0 * r0 - r1 * r1 + d * d) / (2 * d)
    h = math.sqrt(max(0.0, r0 * r0 - a * a))
    ux, uy = (p1[0] - p0[0]) / d, (p1[1] - p0[1]) / d
    return (p0[0] + a * ux + side * h * uy, p0[1] + a * uy - side * h * ux)


def G(P=None):
    g = dict(GEOM)
    d = getattr(P, "DRUM", None) if P is not None else None
    if isinstance(d, dict):
        if isinstance(d.get("geom"), dict):
            g.update({k: v for k, v in d["geom"].items() if k in g})
        ro = d.get("reel_output")
        if isinstance(ro, dict):
            g["out_m"] = float(ro.get("module", g["out_m"]))
            g["out_z"] = int(ro.get("teeth", g["out_z"]))
            g["out_face"] = float(ro.get("face", g["out_face"]))
            if ro.get("plane_y"):
                g["rgear_s"] = float(ro["plane_y"][0])
    lay = getattr(P, "LAYOUT", None) if P is not None else None
    if isinstance(lay, dict):
        ax = lay.get("axes", {})
        mo = (ax.get("main") or {}).get("origin")
        if mo is not None:
            g["main_origin"] = tuple(float(v) for v in mo)
        ox, oy, oz = g["main_origin"]
        do = (ax.get("drum") or {}).get("origin")
        if do is not None:
            g["drum_xy"] = (float(do[0]) - ox, -(float(do[2]) - oz))
        co = (ax.get("capstan") or {}).get("origin")
        if co is not None:
            g["cap_xy"] = (float(co[0]) - ox, -(float(co[2]) - oz))
        if lay.get("deck_top_z") is not None:
            g["deck_y"] = -(float(lay["deck_top_z"]) - oz)
    c = K.cfg(P)
    dx, dy = g["drum_xy"]
    g["phi_pin"] = math.degrees(math.atan2(-dy, -dx))
    if g.get("cap_xy") is not None:
        ring_a = gears.center_distance(c["idl_m"], c["idl_z"], c["ring_z"])
        ix, iy = _circ2(g["drum_xy"], a_seg(c), g["cap_xy"], ring_a, g["idler_side"])
        g["phi_idler"] = math.degrees(math.atan2(iy - dy, ix - dx))
    return g


def C(P=None):
    return K.cfg(P)


# ------------------------------------------------------------------------------------------------ derived
def deck_y(g):
    return g["deck_y"]                            # deck plane in L (y down from the main axis)


def drum_xy(g):
    return tuple(g["drum_xy"])


def a_seg(c):
    return gears.center_distance(c["seg_m"], c["seg_z"], c["pin_z"], 0.0, c["pin_x"])


def a_n20(c):
    return gears.center_distance(c["drum_gear_m"], c["drum_gear_z"], c["n20_z"], c["drum_gear_x"], c["n20_x"])


def idler_xy(g, c):
    a = a_seg(c)
    p = math.radians(g["phi_idler"])
    dx, dy = drum_xy(g)
    return (dx + a * math.cos(p), dy + a * math.sin(p))


def n20_xy(g, c):
    a = a_n20(c)
    p = math.radians(g["phi_n20"])
    dx, dy = drum_xy(g)
    return (dx + a * math.cos(p), dy + a * math.sin(p))


def tip_short_for(m, z, x):
    """Smallest tip shortening (coefficient) that leaves a printable tip land (0.4 mm and 0.25 m)."""
    need = max(0.4, 0.25 * m)
    k = 0.0
    while k < 0.6:
        st = gears.spur_info(m, z, x, tip_short=k)["s_tip"]
        if st == st and st >= need:
            return round(k, 3)
        k += 0.02
    return 0.6


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
    """Drum angle at which world-fixed direction phi_w sees drum-local angle alpha. The 360 deg window is cut in the
    middle of the unreachable gap between the two hard stops, so every reachable angle maps to itself."""
    g0 = 0.5 * (c["stop_hi"] + c["stop_lo"] + 360.0) - 360.0
    return (phi_w - alpha - g0) % 360.0 + g0


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
    body = cyl(g["cpl_od"] / 2, s0, b0 + 0.01)
    bus = gears.spur_gear(g["bus_m"], g["bus_z"], b1 - b0).translate((0, 0, b0))
    i0, i1 = g["idx_s"]
    cam = shp_extrude(idx_band_poly(g), i0, i1)
    body = prim.union([body, bus, cam])
    h0, h1 = g["hub_s"]
    cut = [cyl((g["hub_od"] + fits.SHAFT_IN_BORE) / 2, s0 - 1, h1 + 0.15),
           cyl(9.8, h1, g["sock_s"] + 0.5),                                          # 13 mm socket reaches the nut
           hexprism(g["hex_af"] + fits.HOLE_SLIDE, g["sock_s"], s1 + 1)]
    cut.append(prim.place(cyl(1.7, -30, 30), tf.T(0, 0, g["xpin_s"]) @ tf.Ry(90)))          # M3 cross screw
    cut.append(prim.place(prim.extrude(hex_cs(5.9), 3.0, z=0), tf.T(g["cpl_od"] / 2 - 2.6, 0, g["xpin_s"]) @ tf.Ry(90)))
    cut.append(prim.place(cyl((g["shear_d"] + 0.1) / 2, 10.0, 30.0, seg=32), tf.T(0, 0, g["shear_s"]) @ tf.Ry(90)))
    for a in (90.0, 270.0):        # tapped holes for the REEL sleeve screws
        x, y = 18.5 * math.cos(math.radians(a)), 18.5 * math.sin(math.radians(a))
        cut.append(cyl(1.25, s1 - 8.0, s1 + 1, x, y, seg=24))
    return prim.diff(body, cut)


def idx_centre_r(g, alpha):
    """SW_INDEX cam: nub centre radius over coupler-local angle alpha (idx_n bumps, 50 percent width idx_w)."""
    per = 360.0 / g["idx_n"]
    u = (alpha % per) - per / 2                  # bump centred mid period
    d = g["idx_w"] / 2 - abs(u)
    f = min(1.0, max(0.0, 0.5 + d / (2 * g["idx_ramp"])))
    return g["idx_r0"] + g["nub_r"] + 0.3 + g["idx_bump"] * f


def idx_band_poly(g):
    path = polar_profile(lambda a: idx_centre_r(g, a), n=1440)
    p = path.buffer(-(g["nub_r"] + 0.3), 16)
    if p.geom_type != "Polygon":
        p = max(p.geoms, key=lambda q: q.area)
    return p


def shear_pin(g):
    """D8 torque limiter: PLA pin d4 in single shear at r 13 (about 5 N m at 30 MPa). Print lying (spares x 5)."""
    r = g["hub_od"] / 2
    pin = cyl(g["shear_d"] / 2 - 0.02, r - 3.9, g["cpl_od"] / 2 + 0.01, seg=32)
    head = cyl(g["shear_d"] / 2 + 1.0, g["cpl_od"] / 2, g["cpl_od"] / 2 + 1.2, seg=32)
    p = prim.union([pin, head])
    p = p - prim.box(-20, -g["shear_d"] / 2 + 0.4, -20, 20, -5, 50)      # 0.4 mm flat: prints lying, shear across layers
    return prim.place(p, tf.T(0, 0, g["shear_s"]) @ tf.Ry(90))


def main_shaft(g):
    h0, h1 = g["hex_s"]
    sh = prim.union([hexprism(g["hex_af"], h0, h1), cyl(g["spig_d"] / 2, h1 - 0.5, g["spig_end"])])
    sh = sh - prim.place(cyl(1.7, -30, 30), tf.T(0, 0, g["xpin_s"]) @ tf.Ry(90))
    e = g["spig_end"]
    sh = sh - prim.box(g["spig_d"] / 2 - 1.0, 20, -20, 20, g["cap_s"][0] - 0.5, e + 1)      # flat for the cap screw
    return sh


def reel_sleeve(g):
    f0, f1, fd = g["rsl_flange"]
    j0, j1, jd = g["rsl_journal"]
    s = prim.union([cyl(fd / 2, f0, f1), cyl(jd / 2, f0, j1)])
    s = s - hexprism(g["hex_af"] + fits.SHAFT_IN_BORE, f0 - 1, j1 + 1)
    for a in (90.0, 270.0):        # 2 x M3 x 10 through the flange into the coupler end face (axial retention)
        x, y = 18.5 * math.cos(math.radians(a)), 18.5 * math.sin(math.radians(a))
        s = s - cyl(1.7, f0 - 1, f1 + 1, x, y, seg=24) - cyl(3.0, f1 - 0.01, f1 + 3.0, x, y, seg=32)
    return s


def reel_gear(g, c):
    face = g["out_face"]
    gr = gears.spur_gear(g["out_m"], g["out_z"], face, bore=g["rsl_journal"][2] + fits.SHAFT_IN_BORE)
    gr = gr.translate((0, 0, g["rgear_s"]))
    hub = ring(c["dog_r_out"] + 1.0, (g["rsl_journal"][2] + fits.SHAFT_IN_BORE) / 2, g["rgear_s"] + face - 0.5,
               g["dog_face_reel"] + 0.01)
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
        cuts.append(cyl(1.25, s0 + 0.6, s1 + 1, x, y, seg=24))     # M3 tapped from the sleeve side
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
    if th > c["stop_hi"] or th < c["stop_lo"]:  # unreachable gap: blend from the high stop value to the low one
        tt = th if th > c["stop_hi"] else th + 360.0
        u = (tt - c["stop_hi"]) / (c["stop_lo"] + 360.0 - c["stop_hi"])
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
    parts.append(shp_extrude(lock_layer_poly(g, c), g["seg"][1] + 0.05, g["lock"][1]))
    parts.append(shp_extrude(polar_profile(lambda a: cam_radius(g, c, a)), g["cam"][0] - 0.01, g["cam"][1]))
    parts.append(hexprism(g["spig_af"], g["neck_s"][1] - 0.01, a1))
    # 45 deg cones under every outward step (prints without support when standing on the gear face)
    rc = g["cam_r0"]
    parts.append(prim.cyl(rc - 0.5 - (g["cam"][0] - a0), g["cam"][0] - a0, z=a0, r2=rc - 0.5))
    rs = gears.spur_info(c["seg_m"], c["seg_z"])["r_root"] - 0.5
    h = rs - g["neck_d"] / 2
    parts.append(prim.cyl(g["neck_d"] / 2, h, z=g["seg"][0] - h, r2=rs))
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
    pin = gears.spur_gear(c["seg_m"], c["pin_z"], face, shift=c["pin_x"],
                          tip_short=tip_short_for(c["seg_m"], c["pin_z"], c["pin_x"])).translate((0, 0, g["seg"][0]))
    cam = shp_extrude(lock_cam_poly(g, c), g["lock"][0] + 0.3, g["lock"][1])
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
    p = gears.spur_gear(c["drum_gear_m"], c["n20_z"], face, shift=c["n20_x"],
                        tip_short=tip_short_for(c["drum_gear_m"], c["n20_z"], c["n20_x"]))
    bore = shp_extrude(Point(0, 0).buffer(1.5 + 0.05, 32).difference(
        Polygon([(1.0 + 0.05, -5), (5, -5), (5, 5), (1.0 + 0.05, 5)])), g["dgear"][0] - 1, g["dgear"][1] + 1)
    return p.translate((0, 0, g["dgear"][0])) - bore          # D bore: N20 shaft d3 with a 2.5 flat (measure)


# ------------------------------------------------------------------------------------------------ fork, rail, brackets
def fork_planes(g):
    """Fork axial planes at NEUTRAL: bed plane sb (pads, yoke and tab sit on it), yoke top, follower pin s."""
    pw = (g["groove"][1] - g["groove"][0]) - 0.6
    sb = 0.5 * (g["groove"][0] + g["groove"][1]) - pw / 2
    return dict(pw=pw, sb=sb, yt=sb + g["fork_t"], pin=sb + g["pin_off"], gap=5.5, prong=4.0)


def fork(g, c):
    """Fork at NEUTRAL in L. Prints flat on the pad plane sb. The thin yoke (lower half ring round the collar groove,
    pads at 3 and 9 o'clock) is the spring; a hub under the yoke, an arm to the rail sleeve (30 long against racking),
    an arm to the follower pin block at the drum, pin into groove 1. The yoke tab (+x, nut inside) touches the
    SW_DOG_L / SW_DOG_R screw heads on the fixed dog post only when the collar is fully engaged."""
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
    tab = prim.box(r0, r1 + 7.0, -1.0, 5.0, sb, yt)
    rx, ry = g["rail_xy"]
    hl = g["hub_len"]
    hub = prim.box(-6, 6, r1 - 1.0, r1 + 7.0, sb, sb + hl)
    arm1 = shp_extrude(LineString([(0.0, r1 + 3.0), (rx, ry)]).buffer(5.0), sb, sb + hl)
    sleeve = cyl(g["rail_d"] / 2 + 4.0, sb, sb + 40.0, rx, ry)
    dx, dy = drum_xy(g)
    ph = math.radians(g["phi_pin"])
    ux, uy = math.cos(ph), math.sin(ph)                 # from the drum axis toward the main axis
    tx, ty = -uy, ux
    rb0, rb1 = g["barrel_r"] + 0.8, g["barrel_r"] + 9.0
    pe = (dx + ux * rb1, dy + uy * rb1)
    arm2 = shp_extrude(LineString([(rx, ry), pe]).buffer(4.5), sb + 2.0, sb + hl)
    blk = Polygon([(dx + ux * rb0 + tx * 5, dy + uy * rb0 + ty * 5), (dx + ux * rb1 + tx * 5, dy + uy * rb1 + ty * 5),
                   (dx + ux * rb1 - tx * 5, dy + uy * rb1 - ty * 5), (dx + ux * rb0 - tx * 5, dy + uy * rb0 - ty * 5)])
    ps = F["pin"]
    block = shp_extrude(blk, max(sb + 2.0, ps - 4.5), sb + hl)
    rp0 = g["barrel_r"] - g["groove_d"] + 0.5
    pin = prim.place(cyl(g["pin_d"] / 2, 0.0, rb0 + 0.5 - rp0, seg=32),
                     tf.T(dx, dy, ps) @ tf.Rz(g["phi_pin"]) @ tf.T(rp0, 0, 0) @ tf.Ry(90))
    f = prim.union([yoke] + pads + [tab, hub, arm1, sleeve, arm2, block, pin])
    f = f - cyl((g["rail_d"] + fits.HOLE_SLIDE) / 2, sb - 1, sb + 41, rx, ry)
    f = f - prim.place(prim.extrude(hex_cs(5.6), yt - sb + 2, z=sb - 1), tf.T(r1 + 4.0, 2.0, 0))     # nut in the tab
    return f


def dog_post(g, c):
    """Fixed post clamped on the shift rail (s dogpost_s) with a bar along the yoke tab. Two bosses carry the
    SW_DOG_R (REEL side) and SW_DOG_L (LAUNCH side) M3 screws, heads set to touch the tab nut dog_touch before the
    collar is fully engaged. Open = not engaged (tip to tip, NEUTRAL, or a jammed shift)."""
    F = fork_planes(g)
    sb, yt = F["sb"], F["yt"]
    r0, r1 = g["yoke_r"]
    rx, ry = g["rail_xy"]
    p0, p1 = g["dogpost_s"]
    st = c["stroke"]
    head = 3.0
    sR = sb - st + g["dog_touch"] - head          # REEL boss ends here
    sL = yt + st - g["dog_touch"] + head          # LAUNCH boss starts here
    bx0 = r1 + 7.5
    clamp = cyl(g["rail_d"] / 2 + 5.0, p0, p1, rx, ry)
    arm = shp_extrude(LineString([(rx, ry), (bx0 + 2.25, 4.0)]).buffer(4.0), p0, p1)
    bar = prim.box(bx0, bx0 + 4.5, -3.0, 7.0, p0, sL + 6.0)
    bR = prim.box(r1 + 1.0, bx0 + 0.01, -1.0, 5.0, p0, sR)
    bL = prim.box(r1 + 1.0, bx0 + 0.01, -1.0, 5.0, sL, sL + 6.0)
    d = prim.union([clamp, arm, bar, bR, bL])
    d = d - cyl((g["rail_d"] + fits.HOLE_PRESS) / 2, p0 - 1, p1 + 1, rx, ry)
    d = d - cyl(1.25, p0 - 1, sL + 7.0, r1 + 4.0, 2.0, seg=24)       # M3 tapped through both bosses
    d = d - prim.place(cyl(1.7, -1, 30, seg=24), tf.T(rx + 9.0, ry, 0.5 * (p0 + p1)) @ tf.Ry(90))   # clamp screw
    return d


def rail(g):
    """Shift rail d10 with a 1 mm flat on the outboard side (+x) to print on; the fork sleeve rides the round."""
    rx, ry = g["rail_xy"]
    r = cyl(g["rail_d"] / 2, g["rail_s"][0], g["rail_s"][1], rx, ry)
    return r - prim.box(rx + g["rail_d"] / 2 - 1.0, rx + 10, ry - 10, ry + 10, g["rail_s"][0] - 1, g["rail_s"][1] + 1)


def bracket(g, c, which):
    """Bracket A (motor side, with the N20 ear) or B (outboard): drum axle and shift rail pressed in, feet on the deck."""
    s0, s1 = g["brA"] if which == "A" else g["brB"]
    dx, dy = drum_xy(g)
    rx, ry = g["rail_xy"]
    yd = deck_y(g)
    pa = Point(dx, dy).buffer(g["axle_d"] / 2 + 6, 64)
    pr = Point(rx, ry).buffer(g["rail_d"] / 2 + 5, 48)
    shapes = [pa, pr, pa.union(pr).convex_hull,
              Polygon([(dx - 16, dy), (dx + 16, dy), (dx + 26, yd), (dx - 26, yd)]),
              Polygon([(dx - 34, yd - 6), (dx + 34, yd - 6), (dx + 34, yd), (dx - 34, yd)])]
    if which == "A":
        nx, ny = n20_xy(g, c)
        shapes.append(pa.union(Point(nx, ny).buffer(16.0, 48)).convex_hull)
    b = shp_extrude(unary_union(shapes), s0, s1)
    cuts = [cyl((g["axle_d"] + fits.HOLE_PRESS) / 2, s0 - 1, s1 + 1, dx, dy),
            cyl((g["rail_d"] + fits.HOLE_PRESS) / 2, s0 - 1, s1 + 1, rx, ry)]
    for x in (dx - 28.0, dx + 28.0):
        cuts.append(prim.place(cyl(1.7, -1, 8, seg=24), tf.T(x, yd + 0.5, 0.5 * (s0 + s1)) @ tf.Rx(90)))
    if which == "A":
        nx, ny = n20_xy(g, c)
        cuts.append(cyl(2.5, s0 - 1, s1 + 1, nx, ny, seg=24))                      # N20 shaft end
        for (px, py, kind) in cradle_holes(g, c):
            if kind == "slot":
                cuts.append(prim.union([cyl(1.25, s0 - 1, s1 + 1, px + dd, py, seg=24)
                                        for dd in np.linspace(-3.0, 3.0, 7)]).hull())
            elif kind == "nut":
                cuts.append(prim.place(prim.extrude(hex_cs(5.6), 2.6, z=s1 - 2.6), tf.T(px, py, 0)))
                cuts.append(cyl(1.7, s0 - 1, s1 + 1, px, py, seg=24))
            else:
                cuts.append(cyl(1.25, s0 - 1, s1 + 1, px, py, seg=24))
    return prim.diff(b, cuts)


def bracket_idler(g, c, which="I"):
    """Wrap idler bearing posts: I at the journal (outboard of the output gear), I2 at the shaft's motor end."""
    s0, s1 = g["brI"] if which == "I" else g["brI2"]
    ix, iy = idler_xy(g, c)
    yd = deck_y(g)
    outline = unary_union([Point(ix, iy).buffer(g["idl_j"] / 2 + 6, 64),
                           Polygon([(ix - 10, iy), (ix + 8, iy), (ix + 10, yd), (ix - 20, yd)]),
                           Polygon([(ix - 30, yd - 6), (ix + 12, yd - 6), (ix + 12, yd), (ix - 30, yd)])])
    b = shp_extrude(outline, s0, s1)
    cuts = [cyl((g["idl_j"] + fits.SHAFT_IN_BORE) / 2, s0 - 1, s1 + 1, ix, iy)]
    for x in (ix - 25.0, ix + 7.0):
        cuts.append(prim.place(cyl(1.7, -1, 8, seg=24), tf.T(x, yd + 0.5, 0.5 * (s0 + s1)) @ tf.Rx(90)))
    return prim.diff(b, cuts)


# ------------------------------------------------------------------------------------------------ printed contacts
# A contact lever: flexure beam rooted below the cam level, round nub (radius nub_r) riding a band, M3 x 6 screw in a
# boss 8 mm below the nub, head toward an M3 nut in the block. The band is the inner envelope of the designed nub path,
# so the lever follows exactly. The contact closes at 80 percent of the bump lift (small overtravel, soft 1.6 beam).
LEVER_BT = 1.6


def lever_bx(g, bump):
    return bump - g["nub_r"] + 0.8                 # beam inner face clears the bump tops by 0.8


def lever_gap(g, bump):
    L = g["sw_L"]
    return 0.8 * bump * (L - 8.0) / L


def lever_root(g, cx, cy, r0):
    return (cx + r0 + g["nub_r"] + 0.3 + 1.0, cy + g["sw_L"])


def lever_x0(g, r0, bump):
    """Block front face, x from the cam axis."""
    return r0 + g["nub_r"] + 0.3 + 1.0 + lever_bx(g, bump) + 4.0 + 3.0 + lever_gap(g, bump)


def contact_lever(g, w0, w1, bump):
    L = g["sw_L"]
    bx = lever_bx(g, bump)
    beam = prim.box(bx, bx + LEVER_BT, -L, 6.0, w0, w1)
    boss = prim.box(bx, bx + 4.0, -L + 5.0, -L + 11.0, w0, w1)
    nub = cyl(g["nub_r"], w0, w1, -1.0, -L, seg=32)
    arm = prim.box(-1.0, bx + 0.5, -L - 1.5, -L + 1.5, w0, w1)
    foot = prim.box(bx, bx + 10.0, 2.0, 10.0, w0, w1)
    lv = prim.union([beam, boss, nub, arm, foot])
    sc = 0.5 * (w0 + w1)
    lv = lv - prim.place(cyl(1.25, -1, 6, seg=24), tf.T(bx - 0.5, -L + 8.0, sc) @ tf.Ry(90))      # M3 x 6, tapped
    lv = lv - prim.place(cyl(1.7, -1, 20, seg=24), tf.T(bx + 6.0, 6.0, sc) @ tf.Rx(90))          # foot screw
    return lv


def contact_block(g, cx, cy, r0, bump, bands, y_top, y_bot, s0, s1):
    x0 = cx + lever_x0(g, r0, bump)
    b = prim.box(x0, x0 + 14.0, y_top, y_bot, s0, s1)
    cuts = []
    for (w0, w1) in bands:
        sc = 0.5 * (w0 + w1)
        cuts.append(prim.place(prim.extrude(hex_cs(5.6), 2.6, z=-0.01), tf.T(x0, cy + 8.0, sc) @ tf.Ry(90)))
        cuts.append(prim.place(cyl(1.7, -1, 16, seg=24), tf.T(x0, cy + 8.0, sc) @ tf.Ry(90)))
        rx, ry = lever_root(g, cx, cy, r0)
        cuts.append(prim.place(cyl(1.25, -1, 14, seg=24), tf.T(rx + lever_bx(g, bump) + 6.0, ry + 9.0, sc) @ tf.Rx(-90)))
    return b, cuts


def switch_block(g, c):
    """Code switch block on the +X side of the code bands (2 x M3 to the deck)."""
    dx, dy = drum_xy(g)
    yd = deck_y(g)
    s0, s1 = g["bands"][0][0] - 3.0, g["bands"][-1][1]
    b, cuts = contact_block(g, dx, dy, g["band_r0"], g["bump"], g["bands"], dy + 2.0, yd, s0, s1)
    x0 = dx + lever_x0(g, g["band_r0"], g["bump"])
    foot = prim.box(x0 - 2.0, x0 + 24.0, yd - 6.0, yd, s0, s1)
    for sc in (s0 + 5.0, s1 - 5.0):
        cuts.append(prim.place(cyl(1.7, -1, 8, seg=24), tf.T(x0 + 19.0, yd + 0.5, sc) @ tf.Rx(90)))
    return prim.diff(prim.union([b, foot]), cuts)


def index_block(g):
    """SW_INDEX block in front of the coupler cam, screwed (2 x M3) to the front face of the frame adapter plate."""
    i0, i1 = g["idx_s"]
    s0, s1 = 0.5, i1 + 3.0
    b, cuts = contact_block(g, 0.0, 0.0, g["idx_r0"], g["idx_bump"], [g["idx_s"]], -12.0, g["sw_L"] + 12.0, s0, s1)
    x0 = lever_x0(g, g["idx_r0"], g["idx_bump"])
    for y in (-6.0, g["sw_L"] + 6.0):
        cuts.append(cyl(1.25, s0 - 1, s1 + 1, x0 + 10.0, y, seg=24))
    return prim.diff(b, cuts)


def sw_lever(g, c, i):
    b0, b1 = g["bands"][i]
    return contact_lever(g, b0 + 0.6, b1 - 0.6, g["bump"])


def sw_lever_root(g, i):
    dx, dy = drum_xy(g)
    return lever_root(g, dx, dy, g["band_r0"])


def idx_lever(g):
    i0, i1 = g["idx_s"]
    return contact_lever(g, i0 + 0.3, i1 - 0.3, g["idx_bump"])


def magnet_holder(g):
    """AS5600 magnet cap on the spigot end (s cap_s): D bore on the spigot flat, radial M3 x 6 onto the flat, magnet
    pocket d6.2 x 2.6 in the end face (face at params DRUM.magnet_face_y)."""
    c0, c1 = g["cap_s"]
    e = g["spig_end"]
    cap = cyl(10.0, c0, c1)
    bore = shp_extrude(Point(0, 0).buffer(g["spig_d"] / 2 + fits.HOLE_SLIDE / 2, 64).difference(
        Polygon([(g["spig_d"] / 2 - 1.0 + 0.1, -20), (20, -20), (20, 20), (g["spig_d"] / 2 - 1.0 + 0.1, 20)])),
        c0 - 1, e + 0.2)
    cuts = [bore, cyl(3.0 + 0.1, c1 - 2.6, c1 + 1, seg=48),
            prim.place(cyl(1.25, 0, 12, seg=24), tf.T(0, 0, 0.5 * (c0 + e)) @ tf.Ry(90))]
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
    """Right-angle worm N20 envelope (bought), in the N20 body frame (output axis = z, motor along +x)."""
    N = getattr(P, "N20_WORM", {}) or {}
    bw = float(N.get("box_w", 9.94)); bh = float(N.get("box_h", 11.93))
    ln = float(N.get("len_no_shaft", 33.04)); tip = float(N.get("shaft_tip_to_tip", 29.97))
    zc = g["dgear"][1] + 0.5 + bw / 2
    box = prim.box(-6.0, 6.0, -bh / 2, bh / 2, zc - bw / 2, zc + bw / 2)
    mot = prim.box(6.0, ln - 6.0, -5.0, 5.0, zc - bw / 2 + 0.5, zc + bw / 2 - 0.5)
    sh = cyl(1.5, zc - tip / 2, zc + tip / 2, seg=24)
    return prim.union([box, mot, sh])


def n20_frame(g, c):
    nx, ny = n20_xy(g, c)
    return tf.T(nx, ny, 0) @ tf.Rz(g["n20_motor_deg"])


def cradle_holes(g, c):
    """Cradle to bracket A ear: pivot screw (tapped in the ear), swing screw (slot in the ear), SW_N20 contact
    (screw in the cradle touching a nut in the ear when swung in). In L xy."""
    M = n20_frame(g, c)
    out = []
    for (x, y, kind) in ((-9.0, 10.0, "pivot"), (-9.0, -10.0, "slot"), (12.0, 10.0, "nut")):
        p = M @ np.array([x, y, 0.0, 1.0])
        out.append((float(p[0]), float(p[1]), kind))
    return out


def n20_cradle(g, c, P):
    """Cradle round the N20 gearbox and motor, open at +s (the N20 slides in, the strap closes it), screwed to the
    bracket A ear. Hand fallback: loosen the swing screw, swing the cradle about the pivot screw (pinion out of mesh,
    SW_N20 opens), turn the hand lever, swing back."""
    N = getattr(P, "N20_WORM", {}) or {}
    bw = float(N.get("box_w", 9.94)); bh = float(N.get("box_h", 11.93)); ln = float(N.get("len_no_shaft", 33.04))
    zc = g["dgear"][1] + 0.5 + bw / 2
    foot = unary_union([Polygon([(-6, -bh / 2), (6, -bh / 2), (6, bh / 2), (-6, bh / 2)]),
                        Polygon([(6, -5), (ln - 6, -5), (ln - 6, 5), (6, 5)])])
    outer = foot.buffer(3.0, join_style=2)
    plate = Polygon([(-13, -14), (16, -14), (16, 14), (-13, 14)])
    M = n20_frame(g, c)
    s_top = zc + bw / 2 + 0.3
    body = shp_extrude(outer, g["brA"][1], s_top)
    base = shp_extrude(plate, g["brA"][1], g["brA"][1] + 4.0)
    cr = prim.union([body, base])
    cav = shp_extrude(foot.buffer(0.15, join_style=2), zc - bw / 2 - 0.15, s_top + 1)
    cr = cr - cav
    cr = cr - cyl(gears.spur_info(c["drum_gear_m"], c["n20_z"], c["n20_x"])["r_tip"] + 1.2, g["brA"][1] - 1,
                  zc - bw / 2 + 0.01, seg=48)
    for (x, y, kind) in ((-9.0, 10.0, "pivot"), (-9.0, -10.0, "slot"), (12.0, 10.0, "nut")):
        cr = cr - cyl(1.7 if kind != "nut" else 1.25, g["brA"][1] - 1, g["brA"][1] + 5, x, y, seg=24)
    for (x, y) in ((-7.5, 0.0), (ln - 3.0, 0.0)):
        cr = cr - cyl(1.25, s_top - 6.0, s_top + 1, x, y, seg=24)          # strap screws
    cr = prim.place(cr, M)
    rd = gears.spur_info(c["drum_gear_m"], c["drum_gear_z"], c["drum_gear_x"])["r_tip"] + 1.5
    dx, dy = drum_xy(g)
    cr = cr - cyl(rd, g["brA"][1] - 1, g["dgear"][1] + 0.6, dx, dy)
    return max(cr.decompose(), key=lambda m: m.volume())


def n20_strap(g, c, P):
    """End plate closing the cradle at +s (2 x M3 x 8)."""
    N = getattr(P, "N20_WORM", {}) or {}
    bw = float(N.get("box_w", 9.94)); bh = float(N.get("box_h", 11.93)); ln = float(N.get("len_no_shaft", 33.04))
    zc = g["dgear"][1] + 0.5 + bw / 2
    s_top = zc + bw / 2 + 0.3
    foot = unary_union([Polygon([(-6, -bh / 2), (6, -bh / 2), (6, bh / 2), (-6, bh / 2)]),
                        Polygon([(6, -5), (ln - 6, -5), (ln - 6, 5), (6, 5)])])
    st = shp_extrude(foot.buffer(3.0, join_style=2), s_top, s_top + 3.0)
    tip = float(N.get("shaft_tip_to_tip", 29.97))
    st = st - cyl(2.0, s_top - 1, s_top + 4, seg=24)                         # N20 rear shaft end
    for (x, y) in ((-7.5, 0.0), (ln - 3.0, 0.0)):
        st = st - cyl(1.7, s_top - 1, s_top + 4, x, y, seg=24)
    return prim.place(st, n20_frame(g, c))


# ------------------------------------------------------------------------------------------------ module contract
COL = dict(ppa="#3a3f45", pla="#c8ccd2", gear="#d97a00", drum="#3d7cc9", fork="#7fb069", sw="#e0c040")


def parts(P):
    g, c = G(P), C(P)
    I = tf.I4
    out = []

    def add(name, solid, mat, joint, R=None, color="pla", notes="", **kw):
        out.append(Part(name, solid, mat, joint=joint, print_tf=None if mat == "bought" else bed(solid, I if R is None else R),
                        color=COL.get(color, color), notes=notes, **kw))

    add("drum_coupler_hub", coupler_hub(g, P), "PPA-CF", "drum_main", color="ppa",
        notes="Coupler hub on the wiper shaft (bore parametric: WIPER), M8 nut inside, shear pin hole")
    add("drum_coupler", coupler(g, P), "PPA-CF", "drum_main", R=tf.Rx(180), color="ppa",
        notes="Coupler: 5 N m shear pin limiter (D8), bus gear 40T m1.5 (D15), SW_INDEX cam, hex socket; prints bus gear down")
    add("drum_shear_pin", shear_pin(g), "PLA", "drum_main", R=tf.Ry(180), color="sw", qty=6,
        notes="Torque limiter shear pin d4 (about 5 N m), prints lying; print 6, 5 are spares")
    add("drum_shaft", main_shaft(g), "PPA-CF", "drum_main", R=tf.Rx(90) @ tf.Ry(90), color="ppa",
        notes="Main shaft: hex 22 AF, d12 spigot into the launch sleeve; prints on a hex flat (support under the spigot)")
    add("drum_sleeve_reel", reel_sleeve(g), "PPA-CF", "drum_main", color="ppa",
        notes="REEL sleeve: keyed on the hex, journal for the REEL gear, 2 x M3 to the coupler end face")
    add("drum_gear_reel", reel_gear(g, c), "PPA-CF", "drum_out_reel", color="gear",
        notes="REEL output gear m2 50T face 12 (D20), free on its sleeve, dog face toward the collar")
    add("drum_collar", collar(g, c), "PPA-CF", "drum_collar", color="ppa",
        notes="Dog collar: hex bore, 4 back-cut roofed dogs each face, fork groove (bridges over the lower dogs)")
    add("drum_dogring_launch", dog_ring(g, c), "PPA-CF", "drum_out_launch", R=tf.Rx(180), color="gear",
        notes="LAUNCH dog ring: 3 slots for the launch sleeve lugs, 3 x M3 from the sleeve side")
    add("drum_drum_a", drum_a(g, c), "PLA", "drum_drum", color="drum",
        notes="Drum A: N20 gear 120T m0.8, brake cam, wrap segment gear + locking arc, hex spigot")
    add("drum_drum_b", drum_b(g, c), "PLA", "drum_drum", color="drum",
        notes="Drum B: shift groove barrel, Gray code bands A B C, hand lever hex boss")
    add("drum_axle", axle(g), "PPA-CF", "drum_base", R=tf.Ry(90), color="ppa",
        notes="Fixed drum axle d10, pressed into brackets A and B")
    add("drum_idler_gear", idler_gear(g, c), "PPA-CF", "drum_idler", color="gear",
        notes="Wrap idler output gear 42T m1.5 (D bore) to the cad-reel cup ring 24T: 7 cup turns")
    add("drum_idler", idler(g, c), "PPA-CF", "drum_idler", color="gear",
        notes="Wrap idler shaft: pinion 10T m1.25 (locked at home by the drum lock arc), lock cam, two journals")
    add("drum_n20_pinion", n20_pinion(g, c), "PPA-CF", "drum_n20", color="gear",
        notes="N20 pinion 10T m0.8, D bore pressed on the N20 shaft")
    add("drum_n20", n20_model(g, c, P), "bought", "drum_n20_body", color="#8a8f96",
        bought_ref="N20 right-angle self-locking worm gearmotor 12 V 27 rpm (have)",
        notes="Selector drum motor, DRV8833 #3 at 9 V")
    add("drum_n20_cradle", n20_cradle(g, c, P), "PLA", "drum_base", color="pla",
        notes="N20 cradle on the bracket A ear: pivot + swing screw (hand fallback), SW_N20 contact")
    add("drum_n20_strap", n20_strap(g, c, P), "PLA", "drum_base", color="pla",
        notes="End plate closing the N20 cradle, 2 x M3 x 8")
    add("drum_fork", fork(g, c), "PLA", "drum_fork", color="fork",
        notes="Compliant shift fork: thin yoke is the spring (about 1.5 N/mm), follower pin in groove 1, contact tab")
    add("drum_dog_post", dog_post(g, c), "PLA", "drum_base", color="pla",
        notes="Dog contact post on the shift rail: SW_DOG_R and SW_DOG_L screws (collar fully engaged)")
    add("drum_rail", rail(g), "PPA-CF", "drum_base", R=tf.Ry(90), color="ppa",
        notes="Shift rail d10, pressed into brackets A and B")
    for w in ("A", "B"):
        add(f"drum_bracket_{w.lower()}", bracket(g, c, w), "PLA", "drum_base", color="pla",
            notes=f"Drum bracket {w}: axle, shift rail" + (", N20 ear" if w == "A" else "") + ", 2 x M3 to the deck")
    add("drum_bracket_idler", bracket_idler(g, c, "I"), "PLA", "drum_base", color="pla",
        notes="Wrap idler journal post, 2 x M3 to the deck")
    add("drum_bracket_idler2", bracket_idler(g, c, "I2"), "PLA", "drum_base", color="pla",
        notes="Wrap idler second journal post (motor end), 2 x M3 to the deck")
    add("drum_switch_block", switch_block(g, c), "PLA", "drum_base", color="pla",
        notes="Code switch block: 3 lever feet, 3 M3 nut contacts (SW_DRUM_A, B, C)")
    for i in range(3):
        add(f"drum_sw_lever_{i}", sw_lever(g, c, i), "PLA", f"drum_sw{i}", color="sw",
            notes=f"Code switch lever {'ABC'[i]}: flexure, round nub, M3 x 6 contact screw")
    add("drum_index_block", index_block(g), "PLA", "drum_base", color="pla",
        notes="SW_INDEX block: 2 x M3 into the frame adapter plate front face, M3 nut contact")
    add("drum_index_lever", idx_lever(g), "PLA", "drum_idx", color="sw",
        notes="SW_INDEX lever on the 6-bump coupler cam (D12 fallback encoder)")
    add("drum_magnet_cap", magnet_holder(g), "PLA", "drum_main", R=tf.Rx(180), color="pla",
        notes="AS5600 magnet cap on the spigot end (6 x 2.5 diametric or edge-on disc), M3 x 6 onto the flat")
    add("drum_hand_lever", lever_hand(g), "PLA", "drum_drum", color="pla",
        notes="Hand lever on the drum hex boss (fallback: swing the N20 out first)")
    return out


def joints(P):
    g, c = G(P), C(P)
    dx, dy = drum_xy(g)
    ix, iy = idler_xy(g, c)
    nx, ny = n20_xy(g, c)
    js = [
        Joint("drum_base", None, "fixed", tuple(g["main_origin"]), (0, 0, 1), base=tf.Rx(-90)),
        Joint("drum_main", "drum_base", "rev", (0, 0, 0), (0, 0, 1)),
        Joint("drum_out_reel", "drum_base", "rev", (0, 0, 0), (0, 0, 1)),
        Joint("drum_out_launch", "drum_base", "rev", (0, 0, 0), (0, 0, 1), gear=_launch_gear()),
        Joint("drum_collar_slide", "drum_base", "slide", (0, 0, 0), (0, 0, 1), limits=(-c["stroke"] - 0.01, c["stroke"] + 0.01)),
        Joint("drum_collar", "drum_collar_slide", "rev", (0, 0, 0), (0, 0, 1), gear=("drum_main", 1.0, 0.0)),
        Joint("drum_fork", "drum_base", "slide", (0, 0, 0), (0, 0, 1), limits=(-c["stroke"] - 0.01, c["stroke"] + 0.01)),
        Joint("drum_drum", "drum_base", "rev", (dx, dy, 0), (0, 0, 1), limits=(c["stop_lo"], c["stop_hi"])),
        Joint("drum_idler", "drum_base", "rev", (ix, iy, 0), (0, 0, 1)),
        Joint("drum_n20", "drum_base", "rev", (nx, ny, 0), (0, 0, 1)),
        Joint("drum_n20_body", "drum_base", "fixed", (nx, ny, 0), (0, 0, 1), base=tf.Rz(g["n20_motor_deg"])),
    ]
    for i in range(3):
        rx, ry = sw_lever_root(g, i)
        js.append(Joint(f"drum_sw{i}", "drum_base", "rev", (rx, ry, 0), (0, 0, 1), limits=(-0.5, 8.0)))
    rx, ry = lever_root(g, 0.0, 0.0, g["idx_r0"])
    js.append(Joint("drum_idx", "drum_base", "rev", (rx, ry, 0), (0, 0, 1), limits=(-0.5, 8.0)))
    return js


def _launch_gear():
    """The LAUNCH dog ring is bolted to cad-launch's launch sleeve: follow its joint when that module is present."""
    import os
    fn = os.path.join(os.path.dirname(os.path.abspath(__file__)), "launch.py")
    try:
        if '"launch_sleeve"' in open(fn).read():
            return ("launch_sleeve", 1.0, 0.0)
    except OSError:
        pass
    return None


_LAUNCH_MOD = []


def main_angle(P, S):
    """Main shaft angle for state S. The LAUNCH dog ring is bolted to cad-launch's sleeve, so when that module is
    present its own launch_sleeve value is the master (one angle for every part on the main axis); else wiper_deg."""
    if _launch_gear() is not None:
        try:
            if not _LAUNCH_MOD:
                import importlib
                _LAUNCH_MOD.append(importlib.import_module("cad.parts.launch"))
            v = _LAUNCH_MOD[0].kin(P, S).get("launch_sleeve")
            if v is not None:
                return float(v)
        except Exception:                     # launch module broken: fall back to the timeline wiper angle
            pass
    return float(S.get("wiper_deg", 0.0) or 0.0)


def drum_theta(S, c):
    if "drum_deg" in S and S["drum_deg"] is not None:
        return float(S["drum_deg"])
    m = str(S.get("mode", "LAUNCH")).upper()
    return {"LAUNCH": c["launch"], "NEUTRAL": c["neutral"], "REEL": c["reel"], "WRAPPED": c["wrapped"]}.get(m, c["launch"])


def lever_deg(theta, g, c, i):
    """Code lever deflection (deg about its root): the nub centre follows nub_centre_r exactly."""
    lift = nub_centre_r(g, c, i, g["phi_sw"] - theta) - (g["band_r0"] + g["nub_r"] + 0.3)
    return math.degrees(math.asin(min(1.0, lift / g["sw_L"])))


def idx_lever_deg(wiper, g):
    lift = idx_centre_r(g, g["phi_idx"] - wiper) - (g["idx_r0"] + g["nub_r"] + 0.3)
    return math.degrees(math.asin(min(1.0, lift / g["sw_L"])))


def kin(P, S):
    g, c = G(P), C(P)
    th = drum_theta(S, c)
    w = main_angle(P, S)
    x = K.collar_x(th, c)
    md = K.mode(th, c)
    v = {
        "drum_drum": th,
        "drum_main": w,
        "drum_collar_slide": -x,
        "drum_fork": -x,
        "drum_idler": pinion_angle(th, g, c),
        "drum_n20": n20_angle(th, g, c),
        "drum_out_reel": w if md == "REEL" else float(S.get("reel_out_deg", w)),
        "drum_out_launch": w if md == "LAUNCH" else float(S.get("launch_out_deg", w)),
        "drum_idx": idx_lever_deg(w, g),
    }
    for i in range(3):
        v[f"drum_sw{i}"] = lever_deg(th, g, c, i)
    if _launch_gear() is not None:
        v.pop("drum_out_launch")              # follows launch_sleeve through Joint.gear
    return v


def allowed(P):
    return [
        Allow("drum_shaft", "drum_coupler", "hex spigot in the coupler socket (sliding fit)", max_vol=1.0),
        Allow("drum_rail", "drum_bracket_*", "rail pressed into the brackets"),
        Allow("drum_rail", "drum_dog_post", "dog post clamped on the rail"),
        Allow("drum_n20_pinion", "drum_n20", "pinion pressed on the N20 shaft"),
        Allow("drum_n20", "drum_n20_cradle", "N20 seated in its cradle", max_vol=1.0),
        Allow("drum_n20", "drum_n20_strap", "strap closes the cradle", max_vol=1.0),
        Allow("drum_n20", "drum_bracket_a", "N20 rear shaft end in the ear hole", max_vol=1.0),
        Allow("drum_idler_gear", "drum_idler", "gear on the D flat", max_vol=1.0),
        Allow("drum_sw_lever_*", "drum_switch_block", "lever foot clamped to the block (M3)"),
        Allow("drum_index_lever", "drum_index_block", "lever foot clamped to the block (M3)"),
        Allow("drum_hand_lever", "drum_drum_b", "lever on the hex boss", max_vol=1.0),
        Allow("drum_axle", "drum_bracket_*", "axle pressed into the brackets"),
        Allow("drum_drum_a", "drum_drum_b", "hex spigot in its socket", max_vol=1.0),
        Allow("drum_shear_pin", "drum_coupler*", "shear pin in its holes", max_vol=2.0),
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
        Clearance("drum_fork", "drum_dog_post", 0.0, "contact tab between the dog contact bosses"),
    ]


def audit_states(P):
    c = C(P)
    out = []
    th = int(math.ceil(c["stop_lo"]))
    while th <= int(math.floor(c["stop_hi"])):
        out.append(dict(phase="drum_sweep", t=0.0, drum_deg=float(th), wiper_deg=0.0))
        th += 1
    for w in range(0, 360, 15):
        for m in ("launch", "neutral", "wrapped", "reel"):
            out.append(dict(phase="drum_wiper", t=0.0, drum_deg=c[m], wiper_deg=float(w)))
    for w in range(0, 61, 2):
        out.append(dict(phase="drum_index", t=0.0, drum_deg=c["neutral"], wiper_deg=float(w)))
    return out


def grounded(P):
    # coupler: on the wiper shaft (frame motor); dog ring: bolted to the cad-launch sleeve; index block: frame plate
    return ["drum_bracket_*", "drum_switch_block", "drum_coupler*", "drum_dogring_launch", "drum_index_block"]


def steps(P):
    """Assembly steps, in order, plain language, hardware per step."""
    return [
        dict(title="Main shaft: coupler and limiter",
             text="Push the coupler hub onto the wiper output shaft (flats in line). Drop the M8 nut on the thread and "
                  "tighten it with a 13 mm socket (1/4 inch drive). Slide the outer coupler over the hub, line up the "
                  "holes, push a shear pin in (flat side toward the motor, head outside) and put a piece of tape round "
                  "the coupler over the head. Keep the 5 spare pins with the base. A broken pin stub comes out with an "
                  "M3 screw turned into it.",
             parts=["drum_coupler_hub", "drum_coupler", "drum_shear_pin"]),
        dict(title="Main shaft: index contact",
             text="Press an M3 nut into the index block pocket, screw the index lever foot to the block (M3 x 12), "
                  "thread an M3 x 6 into the lever boss, head toward the nut. Screw the block to the front face of the "
                  "motor adapter plate (2 x M3 x 16) with the nub on the coupler cam. Set the gap so the screw head "
                  "touches the nut near the top of a bump (about 1.5 mm on a low part). Battery unplugged: nut to GND, "
                  "screw to SW_INDEX.",
             parts=["drum_index_block", "drum_index_lever"]),
        dict(title="Drum: bracket A, axle, rail, drum A",
             text="Bolt bracket A to the deck (2 x M3 x 12). Press the drum axle and the shift rail into it (both "
                  "flats facing forward). Slide drum A onto the axle, gear first, and turn it so the gap in its lock "
                  "ring faces back toward the idler position (about drum 180 on the pointer).",
             parts=["drum_bracket_a", "drum_axle", "drum_rail", "drum_drum_a"]),
        dict(title="Drum: wrap idler",
             text="Push the idler output gear onto the idler shaft flat. Push the idler journals into the two idler "
                  "posts. Lower the posts with the idler onto the deck so the small pinion meshes the drum A segment "
                  "teeth, and bolt them down (2 x M3 x 12 each). Turn drum A back to LAUNCH (mark L at the top): the "
                  "flat-sided lock cam must sit in the drum lock ring and the idler can no longer turn by hand.",
             parts=["drum_idler_gear", "drum_idler", "drum_bracket_idler", "drum_bracket_idler2"]),
        dict(title="Drum: fork and dog contacts",
             text="Press an M3 nut into the hex hole of the yoke tab. Slide the dog post onto the rail (bosses toward "
                  "the motor side of the tab), then the fork sleeve, yoke facing the main shaft. Tighten the dog post "
                  "clamp screw (M3 x 16) at s 63 to 71. Thread an M3 x 12 into each dog post boss, head toward the tab. "
                  "Battery unplugged: nut wire to GND, REEL side screw to SW_DOG_R, LAUNCH side screw to SW_DOG_L.",
             parts=["drum_dog_post", "drum_fork"]),
        dict(title="Drum: drum B, hand lever, bracket B",
             text="Swing the fork up on the rail, slide drum B onto the axle (hex socket over the drum A spigot), "
                  "swing the fork down so its pin drops into the groove. Slide the hand lever onto the hex boss. Press "
                  "bracket B onto the axle and rail ends and bolt it down (2 x M3 x 12). The drum turns by hand with "
                  "under 0.5 mm end play.",
             parts=["drum_drum_b", "drum_hand_lever", "drum_bracket_b"]),
        dict(title="Drum: N20",
             text="Press the pinion onto the N20 output shaft (flat to flat), flush with the shaft end. Slide the N20 "
                  "into the cradle from the open end, fit the end plate (2 x M3 x 8). Screw the cradle to the bracket A "
                  "ear: pivot screw M3 x 12 in the round hole, swing screw M3 x 12 in the slot, pinion meshed with a "
                  "hair of play. Thread the SW_N20 screw (M3 x 10) through the cradle so its head touches the ear nut "
                  "when the cradle is swung in. Battery unplugged: nut to GND, screw to SW_N20.",
             parts=["drum_n20_pinion", "drum_n20", "drum_n20_cradle", "drum_n20_strap"]),
        dict(title="Drum: code switches",
             text="Press an M3 nut into each of the 3 front pockets of the switch block and bolt it to the deck "
                  "(2 x M3 x 12). Screw each lever foot to the block (M3 x 12), nub on its band. Thread an M3 x 6 into "
                  "each lever boss, head toward the nut, so the head touches the nut near the top of a bump (gap about "
                  "1.8 mm with the nub low). Battery unplugged: nuts to GND, screws to SW_DRUM_A (nearest the motor), "
                  "SW_DRUM_B, SW_DRUM_C.",
             parts=["drum_switch_block", "drum_sw_lever_0", "drum_sw_lever_1", "drum_sw_lever_2"]),
        dict(title="Main shaft: stack",
             text="Push the REEL sleeve into the REEL gear from the plain side (the lip stops it). On the bench slide "
                  "onto the main shaft from the spigot end: sleeve and gear (flange first), then the collar. Lower the "
                  "shaft so the collar groove drops between the fork pads, push the hex end into the coupler socket, "
                  "fit the M3 x 45 cross screw and nyloc, and screw the sleeve flange to the coupler (2 x M3 x 10). "
                  "The REEL gear must spin free on its sleeve.",
             parts=["drum_shaft", "drum_collar", "drum_gear_reel", "drum_sleeve_reel"]),
        dict(title="Main shaft: LAUNCH dog ring and magnet cap",
             text="Fit the LAUNCH dog ring on the launch sleeve flange (sleeve lugs in the ring slots, 3 x M3 x 10 from "
                  "the sleeve side into the ring) before the launch sleeve goes on the spigot (launcher steps). After "
                  "the launcher: glue the AS5600 magnet into the cap, push the cap on the spigot end (flat to flat) and "
                  "fit the M3 x 6 onto the flat. Hand use of the drum: battery unplugged, slacken the line, loosen the "
                  "swing screw, swing the N20 out, turn the lever, swing back and tighten.",
             parts=["drum_dogring_launch", "drum_magnet_cap"]),
    ]
