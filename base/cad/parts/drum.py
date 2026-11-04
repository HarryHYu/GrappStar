"""Drum subsystem (worker cad-drum): main shaft, coupler, dog clutch, REEL output gear, LAUNCH dog ring (world-fixed
main axis, frame L), and since D49 the N20 selector drum ON THE RAIL HEAD (groove 1 for the follower, track 2 for the
spool clutch lever, Gray code bands, wrap segment gear with locking arc), the wrap idler W, the follower slider and
link, and on the rail bridge the bell crank, shift rail, compliant fork and dog post.

Frames
- L (joint drum_base, world-fixed): z along the main axis (world +Y, s = world y), x = world +X (forward), y = world
  -Z (DOWN). Main shaft, coupler, REEL gear, collar, LAUNCH dog ring, magnet cap, SW_INDEX (frame-fixed, D49).
- Ld (joint drum_head, on cad-launch's rail): the old drum frame put on the rail head: x = -w (down), y = -yl (inboard),
  z = s = -180 - u (rearward); the drum axis at Ld (60, 112) = rail (y 52, w 150). The "deck" of the old brackets is the
  reel head spine's inner face (Ld y 172 = world y -8).
- Lb (joint drum_bridge, on the rail): x = u, y = -w, z = world y; at pitch 0 it is L. The fork yoke sits round the
  collar groove as before; the shift rail is at (u -40, w -52).
Timing, codes and loads are in _drum_kin.py (shared with firmware docs and tests). Notes: drum_NOTES.md.
"""
import math

import numpy as np
from shapely.geometry import Point, Polygon, LineString
from shapely.ops import unary_union
from shapely import affinity

from cad.lib import Part, Joint, Allow, Clearance, Mesh, prim, tf, fits, gears
from cad.lib.clutch import _polar_tooth
from cad.parts import _drum_kin as K

# ------------------------------------------------------------------------------------------------ geometry numbers
GEOM = dict(
    # placement (BASE_SPEC v1.0 / params.LAYOUT; G() reads params when present)
    main_origin=(0.0, 0.0, 0.0),   # world point of s = 0 on the main axis (adapter plate face)
    drum_xy=(60.0, 112.0),         # drum axis in L (x forward, y DOWN): world (60, z -112)
    deck_y=180.0,                  # D49: reel head spine inner face in Ld (world y -16)
    cap_xy=(99.4, 24.0),           # cad-reel capstan axis in Ld (rail y 140, w 110.6), for the wrap idler mesh
    # coupler (D8 shear pin limiter, D15 bus gear)
    cpl_s=(0.5, 62.0), cpl_od=40.0, sock_s=28.0,      # the outer coupler carries the REEL gear journal (s 50..62)
    # coupler hub on the wiper shaft (D48): G() derives wiper_face_s, flat_end_s, hub_s[0] and nut_s from params WIPER
    # (boss_face_y and the shaft numbers, cad-frame motor model defaults, not measured); these are the default values:
    # boss face y -10, output boss top y -6, round shaft to -4, flats to 10, M8 thread 10..22
    wiper_face_s=-6.0,       # wiper output boss top (boss face + hub_h); the hub starts 1.0 above it, in the puck bore
    flat_end_s=10.0,         # end of the flats (boss face + shaft_len): the D bore runs to 0.5 past it
    hub_od=26.0, hub_s=(-5.0, 27.5), nut_s=13.0, shear_s=9.0, shear_d=4.0,   # M8 nut on the thread (13..19.5)
    bus_m=1.5, bus_z=50, bus_s=(40.0, 48.0),
    cpl_rmax=28.5,           # coupler radius below s 40: reel_wall top edge is 29.6 under the axis (s 3..40)
    idx_s=(33.5, 38.5), idx_r0=26.0, idx_bump=2.5, idx_n=6, idx_w=24.0, idx_ramp=4.0,   # D12 SW_INDEX cam
    idx_psi=0.0,             # SW_INDEX lever and block on the +x side of the cam; block screwed to the motor plate (D49)
    cheek_face_s=32.0,       # inner face of the motor-side cheek (frame_cheek_r y 22..32, coupler hole r 34; cad-frame
                             # carves the cheek round every drum part, so nothing of mine is fixed to it)
    # main shaft
    hex_af=22.0, hex_s=(28.0, 106.0), spig_d=12.0, spig_end=198.3,
    cap_s=(195.0, 201.5),      # D23: magnet face at y 201.5, outboard of the launch sleeve (y 111..193)
    # REEL sleeve and gear (params.DRUM.reel_output m2 50T face 12, plane 50.5..62.5)
    rsl_flange=(48.0, 50.3, 40.0), rsl_journal=(50.0, 62.0, 33.0),   # thrust step and REEL journal on the coupler
    washer=(62.05, 63.5, 12.9, 17.55), washer_screw_r=13.8,             # REEL gear retaining washer, 2 x M3 x 6
    # (hex bore on the shaft: the M3 heads at r 13.8 on the flat lines clear the flats, r 11.05 > 11.0)
    out_m=2.0, out_z=50, out_face=12.0, rgear_s=50.5,
    dog_face_reel=64.5,     # REEL gear dog roots (hub 62.5..64.5); tips at +dog_h
    collar_s=(72.0, 98.0), collar_od=60.0, groove=(82.0, 88.0), groove_r=26.0,
    ring_face=105.5,        # LAUNCH dog ring dog roots; tips at -dog_h (100.5)
    ring_s=(105.5, 111.0), ring_od=62.0, ring_id=27.0, ring_lug=(6.0, 3.0, 15.0, 21.0), ring_screw_r=28.5,
    # D49 rail drum (Ld frame, s = -180 - u): two printed parts on a fixed d10 axle from the reel head front plate
    # (s 1.2 .. 7.2, it plays bracket A) to bracket B; axial order gear, barrel, cam, bands, segment, lock
    axle_d=10.0, axle_s=(1.2, 96.5), drumA_s=(8.0, 47.5), drumB_s=(47.5, 90.0), spig_af=16.0,
    dgear=(8.0, 16.0), cam=(49.0, 57.0), seg=(77.0, 84.0), lock=(84.0, 90.0), neck_d=20.0, neck_s=(16.0, 17.0),
    cam_r0=30.0, barrel=(17.0, 47.5), barrel_r=30.0, groove_w=6.4, groove_d=4.0,
    bands=((58.0, 63.0), (64.0, 69.0), (70.0, 75.0)), band_r0=22.0, bump=3.0, nub_r=1.5,
    pin_s=32.25, groove_sign=1.0,           # follower pin plane at NEUTRAL; the groove moves it +x (rearward)
    # follower: guide rod parallel to the drum at Ld (rod_xy), slider with the pin into groove 1 and the link lug
    rod_d=10.0, rod_xy=(50.28, 167.15), slider_len=20.0, pin_d=6.0, link_a=(62.0, 171.0), link_d=8.0,
    # world-fixed angles round the drum axis (Ld xy plane, deg; x = down). phi_pin and phi_idler are derived in G()
    phi_pin=100.0, phi_idler=-170.8, phi_brake=0.0, phi_sw=180.0, phi_n20=20.0, n20_motor_deg=0.0,
    brA=(1.2, 7.2), brB=(90.5, 96.5),
    idl=(76.5, 131.0), idl_gear=(97.5, 104.5), idl_p2=(125.0, 131.0),
    idl_j=10.0, idler_side=1, idl_mid=12.0,
    lock_RL=38.0, lock_rl=11.0, lock_c=0.25,
    # bridge (Lb frame): shift rail, fork, bell crank (pivot parallel to w), dog post
    rail_xy=(-40.0, 52.0), rail_d=10.0, rail_s=(30.0, 118.0), rail_boss_s=(30.0, 40.0),
    fork_s=85.0, fork_over=0.0, fork_t=2.4, yoke_r=(31.0, 37.0), hub_len=14.0,
    fork_sleeve=(52.0, 103.0), dogpost_s=(112.0, 118.0), dog_touch=0.5,
    crank_pivot=(-85.0, 60.0), crank_arm=45.0, crank_w=(-71.0, -65.0), shelf_w=(-78.0, -72.0),
    lug_w=(-64.5, -61.0),
    sw_L=36.0,              # contact lever length: 0.66 percent peak strain (cad.lib.switch model, limit 0.8)
)


def _circ2(p0, r0, p1, r1, side):
    d = math.dist(p0, p1)
    a = (r0 * r0 - r1 * r1 + d * d) / (2 * d)
    h = math.sqrt(max(0.0, r0 * r0 - a * a))
    ux, uy = (p1[0] - p0[0]) / d, (p1[1] - p0[1]) / d
    return (p0[0] + a * ux + side * h * uy, p0[1] + a * uy - side * h * ux)


NUT_M8_H = 6.5                # M8 nut height (DIN 934)


def _hub_on_wiper(g, P):
    """D48: the coupler hub follows the wiper shaft. params WIPER boss_face_y is the boss face (world y); hub_h,
    shaft_len and thread_len come from params WIPER with cad-frame's motor model defaults (the function the frame
    builds frame_wiper_shaft from), so Harry's measurements move the shaft and the hub together. The hub stands 1.0
    above the output boss (0.5 once the nut has pulled it onto the end of the flats), the D bore runs from 2.5 above
    the boss to 0.5 past the flats, the M8 nut sits on the thread with two pitches of thread above it, at least 2 mm
    above the D bore end and inside the hub."""
    from cad.parts import _frame_motor as FM
    w = FM.wiper_cfg(P)
    boss = float(w["boss_face_y"]) - float(g["main_origin"][1])
    top = boss + float(w["hub_h"])
    flat_end = boss + float(w["shaft_len"])
    thread_end = flat_end + float(w["thread_len"])
    h1 = g["hub_s"][1]
    g["wiper_face_s"], g["flat_end_s"], g["hub_s"] = top, flat_end, (top + 1.0, h1)
    g["nut_s"] = min(max(flat_end + 2.5, thread_end - NUT_M8_H - 2.5), h1 - NUT_M8_H)


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
        mo = ((lay.get("axes", {}) or {}).get("main") or {}).get("origin")
        if mo is not None:
            g["main_origin"] = tuple(float(v) for v in mo)
    rr = getattr(P, "RAIL_REEL", None) if P is not None else None
    if isinstance(rr, dict):
        dyw = (rr.get("drum") or {}).get("axis_yw")
        if dyw is not None:
            g["head_o"] = (-180.0, (float(dyw[0]) - 140.0) + g["drum_xy"][1], float(dyw[1]) + g["drum_xy"][0])
        cyw = (rr.get("capstan") or {}).get("axis_yw")
        if cyw is not None:
            ho = g.get("head_o", HEAD_O)
            g["cap_xy"] = (ho[2] - float(cyw[1]), ho[1] - (float(cyw[0]) - 140.0))
    g.setdefault("head_o", HEAD_O)
    _hub_on_wiper(g, P)
    c = K.cfg(P)
    dx, dy = g["drum_xy"]
    if g.get("cap_xy") is not None:
        ring_a = gears.center_distance(c["idl_m"], c["idl_z"], c["ring_z"])
        cands = [_circ2(g["drum_xy"], a_seg(c), g["cap_xy"], ring_a, sd) for sd in (1, -1)]
        ix, iy = max(cands, key=lambda q: q[0])          # below the drum, behind G_a (study W at y 82.8, w 118.6)
        g["phi_idler"] = math.degrees(math.atan2(iy - dy, ix - dx))
    return g


HEAD_O = (-180.0, 24.0, 210.0)     # rail (u, yl, w) of the Ld origin: Ld (x, y, s) -> (u0 - s, yl0 - y, w0 - x)


def head_M(g=None):
    """4x4 Ld -> rail frame (cad-launch's launch_rail joint frame: u, yl = y - 140, w)."""
    o = (g or {}).get("head_o", HEAD_O)
    M = np.eye(4)
    M[:3, 0], M[:3, 1], M[:3, 2] = (0, 0, -1), (0, -1, 0), (-1, 0, 0)
    M[:3, 3] = o
    return M


def bridge_M():
    """4x4 Lb -> rail frame: x = u, y = -w, z = world y (origin on the main axis at world y 0)."""
    M = np.eye(4)
    M[:3, 0], M[:3, 1], M[:3, 2] = (1, 0, 0), (0, 0, -1), (0, 1, 0)
    M[:3, 3] = (0.0, -140.0, 0.0)
    return M


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
        st = gears.spur_info(m, z, x, backlash=fits.GEAR_BACKLASH, tip_short=k)["s_tip"]
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
    """Inner coupler hub (PPA-CF): wiper shaft bore (parametric WIPER, placed from the boss face datum by G, D48),
    M8 nut pocket, half of the shear pin hole."""
    W = getattr(P, "WIPER", {}) or {}
    d = float(W.get("shaft_d", 10.0)); fl = float(W.get("shaft_flat", 8.0))
    h0, h1 = g["hub_s"]
    body = cyl(g["hub_od"] / 2, h0, h1)
    f0 = g["wiper_face_s"] + 2.5                                        # round shaft below, flats above
    f1 = g["flat_end_s"] + 0.5       # flats end at WIPER.shaft_len from the boss face (D48), M8 thread beyond
    dd = prim.extrude(prim.shapely_cs(Point(0, 0).buffer((d + fits.HOLE_SLIDE) / 2, 64)
                                      .intersection(Polygon([(-20, -(fl + 0.2) / 2), (20, -(fl + 0.2) / 2),
                                                             (20, (fl + 0.2) / 2), (-20, (fl + 0.2) / 2)]))),
                      f1 - f0, z=f0)
    cut = [dd, cyl((d + fits.HOLE_SLIDE) / 2, h0 - 1, f0 + 0.01), cyl(4.3, h0 - 1, h1 + 1),
           cyl(9.6, g["nut_s"], h1 + 1)]                                # round pocket: M8 nut + 13 mm socket
    r = g["hub_od"] / 2
    cut.append(prim.place(cyl(g["shear_d"] / 2, r - 4.0, r + 1.0, seg=32), tf.T(0, 0, g["shear_s"]) @ tf.Ry(90)))
    return prim.diff(body, cut)


def coupler(g, P):
    """Outer coupler (PPA-CF): runs on the hub, driven only through the shear pin (D8 torque limiter, about 5 N m),
    carries the take-up bus gear (D15, after the limiter) and the hex socket for the main shaft."""
    s0, s1 = g["cpl_s"]
    b0, b1 = g["bus_s"]
    f0, f1, fd = g["rsl_flange"]
    j0, j1, jd = g["rsl_journal"]
    i0, i1 = g["idx_s"]
    r0, r1 = g["cpl_od"] / 2, g["idx_r0"]
    rm = g["cpl_rmax"]
    # 45 deg cones (prints hub end down, no support): r 20 up to the index cam base r 26, then r 28.5 under the bus gear.
    # The bus gear ring overhangs r 28.5..39 at s 40: tree support there (the reel_wall top edge is 29.6 under the axis)
    cone1 = prim.cyl(r0, rm - r0, z=i0 - (rm - r0), r2=rm)          # up to r 28.5 under the cam: the bumps sit on it
    cone2 = prim.cyl(r1, rm - r1, z=i1, r2=rm)
    body = prim.union([cyl(r0, s0, i0 - (rm - r0) + 0.01), cone1, cyl(r1, i0 - 0.01, i1 + 0.01), cone2,
                       cyl(rm, i1 + rm - r1 - 0.01, b0 + 0.01), cyl(fd / 2, b1 - 0.01, f1), cyl(jd / 2, f1 - 0.01, j1)])
    bus = gears.spur_gear(g["bus_m"], g["bus_z"], b1 - b0).translate((0, 0, b0))
    cam = shp_extrude(idx_band_poly(g), i0, i1)
    body = prim.union([body, bus, cam])
    h0, h1 = g["hub_s"]
    cut = [cyl((g["hub_od"] + fits.SHAFT_IN_BORE) / 2, s0 - 1, h1 + 0.15),
           prim.cyl(13.2, 3.4, z=h1 + 0.149, r2=9.8),                                # 45 deg roof over the hub end
           cyl(9.8, h1, g["sock_s"] + 0.5),                                          # 13 mm socket reaches the nut
           hexprism(g["hex_af"] + fits.HOLE_SLIDE, g["sock_s"], s1 + 1)]
    # no shaft retention screw (its hole would sit inside the frame cheek, out of reach): the shaft is captured by the
    # magnet cap against the launch sleeve end (2 mm) and its hex shoulder against the sleeve flange (5 mm)
    cut.append(prim.place(cyl((g["shear_d"] + 0.1) / 2, 10.0, 30.0, seg=32), tf.T(0, 0, g["shear_s"]) @ tf.Ry(90)))
    for a in (90.0, 270.0):        # tapped holes for the REEL gear retaining washer (on the hex flats' radial lines)
        x, y = g["washer_screw_r"] * math.cos(math.radians(a)), g["washer_screw_r"] * math.sin(math.radians(a))
        cut.append(cyl(1.25, j1 - 6.0, j1 + 1, x, y, seg=24))
    return prim.diff(body, cut)


def reel_washer(g):
    """REEL gear retaining washer on the coupler journal end (2 x M3 x 6): holds the gear toward +s."""
    w0, w1, ri, ro = g["washer"]
    w = cyl(ro, w0, w1) - hexprism(g["hex_af"] + fits.HOLE_SLIDE, w0 - 1, w1 + 1)   # keyed on the shaft hex
    for a in (90.0, 270.0):
        x, y = g["washer_screw_r"] * math.cos(math.radians(a)), g["washer_screw_r"] * math.sin(math.radians(a))
        w = w - cyl(1.7, w0 - 1, w1 + 1, x, y, seg=24)
    return w


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
    head = cyl(g["shear_d"] / 2 + 0.5, g["cpl_od"] / 2, g["cpl_od"] / 2 + 1.2, seg=32)
    p = prim.union([pin, head])
    p = p - prim.box(-20, -g["shear_d"] / 2 + 0.4, -20, 20, -5, 50)      # 0.4 mm flat: prints lying, shear across layers
    return prim.place(p, tf.T(0, 0, g["shear_s"]) @ tf.Ry(90))


def main_shaft(g):
    h0, h1 = g["hex_s"]
    sh = prim.union([hexprism(g["hex_af"], h0, h1), cyl(g["spig_d"] / 2, h1 - 0.5, g["spig_end"])])
    sh = sh - cyl(4.8, h0 - 1, h0 + 5.0)                                # room for the wiper shaft thread end (to s 32)
    e = g["spig_end"]
    sh = sh - prim.box(g["spig_d"] / 2 - 1.0, 20, -20, 20, g["cap_s"][0] - 0.5, e + 1)      # flat for the cap screw
    return sh


def reel_gear(g, c):
    face = g["out_face"]
    gr = gears.spur_gear(g["out_m"], g["out_z"], face, bore=g["rsl_journal"][2] + fits.SHAFT_IN_BORE)
    gr = gr.translate((0, 0, g["rgear_s"]))
    hub = ring(c["dog_r_out"] + 1.0, (g["rsl_journal"][2] + fits.SHAFT_IN_BORE) / 2, g["rgear_s"] + face - 0.5,
               g["dog_face_reel"] + 0.01)
    teeth = dog_teeth(c, g["dog_face_reel"], +1, 45.0)
    gr = prim.union([gr, hub, teeth])
    # bore steps out to r 17.7 past the journal end: the retaining washer and its screw heads sit inside the hub;
    # the gear is held between the coupler thrust step (s 50.3) and the washer (s 62.05)
    return gr - cyl(17.7, g["rsl_journal"][1], g["dog_face_reel"] + 1.0)


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
        cuts.append(cyl(1.25, s0 - 1, s1 + 1, x, y, seg=24))       # M3 tapped through from the sleeve side (M3 x 10)
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
        s = g["pin_s"] + g["groove_sign"] * K.collar_x(t, c)
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
    if th > c["stop_hi"] or th < c["stop_lo"]:
        # unreachable gap: a radial cliff 0.5 deg past the high stop, LIGHT for the rest, so the whole LAUNCH zone up
        # to the -4 stop stays LIGHT (3.5 deg from the cliff; REEL 347 is 5.5 deg from it)
        tt = th if th > c["stop_hi"] else th + 360.0
        v = K.brake_lift(c["stop_hi"], c) if tt < c["stop_hi"] + 0.5 else K.brake_lift(c["stop_lo"], c)
    else:
        v = K.brake_lift(th, c)
    return g["cam_r0"] + v


def drum_a(g, c):
    """Drum part A (front), prints gear face down: N20 gear 120T, groove 1 barrel, hex spigot into drum B. Turns on
    the fixed axle."""
    a0, a1 = g["drumA_s"]
    dg = gears.spur_gear(c["drum_gear_m"], c["drum_gear_z"], g["dgear"][1] - g["dgear"][0], shift=c["drum_gear_x"])
    parts = [dg.translate((0, 0, g["dgear"][0])),
             cyl(g["barrel_r"], g["dgear"][1] - 0.01, g["barrel"][1]) - groove_cutter(g, c)]
    sp0 = g["barrel"][1] - 0.01
    parts.append(hexprism(g["spig_af"], sp0, sp0 + 3.5) - prim.box(7.5, 20, -20, 20, sp0 - 1, sp0 + 5))
    d = prim.union(parts)
    return d - cyl((g["axle_d"] + fits.SHAFT_IN_BORE) / 2, a0 - 1, sp0 + 5)


def drum_b(g, c):
    """Drum part B (rear), prints lock face down: brake cam (track 2), code bands A B C, wrap segment gear and the
    locking arc, socket for drum A's spigot."""
    b0, b1 = g["drumB_s"]
    parts = [shp_extrude(polar_profile(lambda a: cam_radius(g, c, a)), g["cam"][0], g["cam"][1]),
             cyl(g["band_r0"] - 1.1, g["cam"][0] + 0.3, g["seg"][0] + 0.01)]   # inside the cam start: no tangent rim edge
    for i, (s0, s1) in enumerate(g["bands"]):
        parts.append(shp_extrude(band_poly(g, c, i), s0 - 0.01, s1 + 0.01))
    sg = gears.spur_gear(c["seg_m"], c["seg_z"], g["seg"][1] - g["seg"][0], present=seg_present(g, c))
    parts.append(prim.place(sg, tf.T(0, 0, g["seg"][0] - 0.01) @ tf.Rz(seg_phase(g, c))))
    parts.append(shp_extrude(lock_layer_poly(g, c), g["seg"][1] - 0.05, g["lock"][1]))
    d = prim.union(parts)
    sp0 = g["barrel"][1] - 0.01
    d = d - (hexprism(g["spig_af"] + fits.HOLE_SLIDE, b0 - 1, sp0 + 3.8)
             - prim.box(7.5 + fits.HOLE_SLIDE / 2, 20, -20, 20, b0 - 2, sp0 + 4))   # keyed corner
    return d - cyl((g["axle_d"] + fits.SHAFT_IN_BORE) / 2, b0 - 1, b1 + 1)


def axle(g):
    """Fixed drum axle d10 (PPA-CF), pressed into the reel head front plate and bracket B; 1 mm print flat on +x."""
    dx, dy = drum_xy(g)
    a0, a1 = g["axle_s"]
    a = cyl(g["axle_d"] / 2, a0, a1, dx, dy)
    return a - prim.box(dx + g["axle_d"] / 2 - 1.0, dx + 10, dy - 10, dy + 10, a0 - 1, a1 + 1)


def idler(g, c):
    """Wrap idler W shaft (PPA-CF), frame: idler axis, s along z: pinion 10T m1.25 at the segment, lock cam, d10
    journal in bracket B, d12 D-flat section for the 49T output gear, d10 journal in the reel head's P2."""
    s0, s1 = g["idl"]
    face = g["seg"][1] - g["seg"][0] - 0.3
    pin = gears.spur_gear(c["seg_m"], c["pin_z"], face, shift=c["pin_x"],
                          tip_short=tip_short_for(c["seg_m"], c["pin_z"], c["pin_x"])).translate((0, 0, g["seg"][0]))
    cam = shp_extrude(lock_cam_poly(g, c), g["lock"][0] + 0.3, g["lock"][1])
    disc = shp_extrude(lock_cam_poly(g, c), g["seg"][1], g["lock"][0] + 0.35)
    shaft = cyl(g["idl_j"] / 2, s0, s1)
    mid = cyl(g["idl_mid"] / 2, g["brB"][1] + 0.1, g["idl_p2"][0] - 0.1)
    sh = prim.union([pin, cam, disc, shaft, mid])
    flat = prim.box(g["idl_mid"] / 2 - 1.0, 10, -10, 10, g["idl_gear"][0] - 0.5, g["idl_p2"][0])
    return sh - flat


def idler_gear(g, c):
    """Wrap idler output gear 49T m1.5 with a D bore onto the idler shaft's d12 flat (frame: idler axis)."""
    og = gears.spur_gear(c["idl_m"], c["idl_z"], g["idl_gear"][1] - g["idl_gear"][0]).translate((0, 0, g["idl_gear"][0]))
    r = (g["idl_mid"] + fits.HOLE_SLIDE) / 2
    bore = shp_extrude(Point(0, 0).buffer(r, 64).difference(
        Polygon([(g["idl_mid"] / 2 - 1.0 + 0.1, -20), (20, -20), (20, 20), (g["idl_mid"] / 2 - 1.0 + 0.1, 20)])),
        g["idl_gear"][0] - 1, g["idl_gear"][1] + 1)
    return og - bore


def n20_pinion(g, c):
    face = g["dgear"][1] - g["dgear"][0]
    p = gears.spur_gear(c["drum_gear_m"], c["n20_z"], face, shift=c["n20_x"],
                        tip_short=tip_short_for(c["drum_gear_m"], c["n20_z"], c["n20_x"]))
    bore = shp_extrude(Point(0, 0).buffer(1.5 + 0.05, 32).difference(
        Polygon([(1.0 + 0.05, -5), (5, -5), (5, 5), (1.0 + 0.05, 5)])), g["dgear"][0] - 1, g["dgear"][1] + 1)
    return p.translate((0, 0, g["dgear"][0])) - bore          # D bore: N20 shaft d3 with a 2.5 flat (measure)


# ------------------------------------------------------------------------------------------------ follower, link, bracket B
def rod(g):
    """Follower guide rod d10 (PPA-CF), Ld: pressed into the reel head front plate and bracket B; 1 mm print flat."""
    rx, ry = g["rod_xy"]
    a0, a1 = g["axle_s"]
    r = cyl(g["rod_d"] / 2, a0, a1, rx, ry)
    return r - prim.box(rx - 10, rx - g["rod_d"] / 2 + 1.0, ry - 10, ry + 10, a0 - 1, a1 + 1)


def follower(g, c):
    """Follower slider (PLA), Ld at NEUTRAL: sleeve on the guide rod, pin block with the d6 pin into groove 1 (on the
    drum barrel at phi_pin), lug for the link's top eye (M3 screw). Prints on its sleeve end."""
    dx, dy = drum_xy(g)
    rx, ry = g["rod_xy"]
    ps = g["pin_s"]
    L = g["slider_len"]
    sl = cyl(g["rod_d"] / 2 + 4.0, ps - L / 2 + 3.0, ps + L / 2, rx, ry)    # front end clear of the drum gear at LAUNCH
    ph = math.radians(g["phi_pin"])
    ux, uy = math.cos(ph), math.sin(ph)
    tx, ty = -uy, ux
    rb0 = g["barrel_r"] + 0.8
    rb1 = math.hypot(rx - dx, ry - dy)
    blk = Polygon([(dx + ux * rb0 + tx * 5, dy + uy * rb0 + ty * 5), (dx + ux * rb1 + tx * 5, dy + uy * rb1 + ty * 5),
                   (dx + ux * rb1 - tx * 5, dy + uy * rb1 - ty * 5), (dx + ux * rb0 - tx * 5, dy + uy * rb0 - ty * 5)])
    block = shp_extrude(blk, ps - 5.0, ps + 5.0)
    rp0 = g["barrel_r"] - g["groove_d"] + 0.5
    pin = prim.place(cyl(g["pin_d"] / 2, 0.0, rb0 + 0.5 - rp0, seg=32),
                     tf.T(dx, dy, ps) @ tf.Rz(g["phi_pin"]) @ tf.T(rp0, 0, 0) @ tf.Ry(90))
    ax, ay = g["link_a"]
    lug = shp_extrude(LineString([(rx, ry), (ax, ay)]).buffer(5.0), ps + L / 2 - 6.0, ps + L / 2)
    f = prim.union([sl, block, pin, lug])
    f = f - cyl((g["rod_d"] + fits.HOLE_SLIDE) / 2, ps - L / 2 - 1, ps + L / 2 + 1, rx, ry)
    f = f - cyl(1.25, ps + L / 2 - 7, ps + L / 2 + 1, ax, ay, seg=24)          # link eye screw, tapped
    return f


def link_ends(g, P=None):
    """(A, B): the link's eye centres in the RAIL frame at NEUTRAL: A on the follower lug, B on the bell crank's input
    tip (crank plane centre)."""
    ax, ay = g["link_a"]
    A = (head_M(g) @ np.array([ax, ay, g["pin_s"] + g["slider_len"] / 2 + 2.0, 1.0]))[:3]
    cx, cs = g["crank_pivot"]
    wc = 0.5 * sum(g["crank_w"])
    B = np.array([cx, cs - g["crank_arm"] - 140.0, wc])
    return A, B


def link(g, P=None):
    """Link (bought 8 mm steel rod with two printed eyes), RAIL frame at NEUTRAL, from the follower lug to the bell crank
    input tip. Modelled as one straight rod with the eye bosses."""
    A, B = link_ends(g, P)
    v = B - A
    Lr = float(np.linalg.norm(v))
    rodm = prim.place(prim.cyl(g["link_d"] / 2, Lr - 22.0, z=11.0), tf.frame(tuple(A), tuple(v / Lr)))
    return rodm, A, B


def link_eye(g, end):
    """Printed eye (PLA) pressed on an end of the link rod: a d8 socket 10 deep and a flat eye with an M3 hole. RAIL
    frame at NEUTRAL; end 0 = follower (top), 1 = crank."""
    A, B = link_ends(g)
    v = (B - A) / np.linalg.norm(B - A)
    p, d = (A, v) if end == 0 else (B, -v)
    M = tf.frame(tuple(p), tuple(d))
    eye = prim.union([prim.cyl(6.5, 21.0, z=-1.0), prim.cyl(6.0, 3.0, z=-3.0)])
    eye = eye - prim.cyl(g["link_d"] / 2 + 0.05, 11.0, z=10.0) - prim.cyl(1.7, 6.0, z=-4.0)
    return prim.place(eye, M)


def link_eye_bed(g, end):
    """Print rotation for a link eye: its axis to +z, the flat end (eye plate) on the bed."""
    A, B = link_ends(g)
    v = (B - A) / np.linalg.norm(B - A)
    p, d = (A, v) if end == 0 else (B, -v)
    M = tf.frame(tuple(p), tuple(d))
    R = np.eye(4)
    R[:3, :3] = M[:3, :3].T
    return R


def bracket_b(g, c):
    """Bracket B (PLA), Ld s 90.5 .. 96.5: drum axle and guide rod pressed in, wrap idler journal, foot screwed to the
    reel head spine (2 x M3 x 12 from outside the spine into tapped holes)."""
    s0, s1 = g["brB"]
    dx, dy = drum_xy(g)
    rx, ry = g["rod_xy"]
    ix, iy = idler_xy(g, c)
    yd = deck_y(g)
    pa = Point(dx, dy).buffer(g["axle_d"] / 2 + 6, 64)
    pr = Point(rx, ry).buffer(g["rod_d"] / 2 + 5, 48)
    pi = Point(ix, iy).buffer(g["idl_j"] / 2 + 5, 48)
    shapes = [unary_union([pa, pr, pi]).convex_hull,
              Polygon([(dx - 16, dy), (dx + 16, dy), (dx + 26, yd), (dx - 26, yd)]),
              Polygon([(dx - 34, yd - 6), (dx + 34, yd - 6), (dx + 34, yd), (dx - 34, yd)])]
    b = shp_extrude(unary_union(shapes), s0, s1)
    cuts = [cyl((g["axle_d"] + fits.HOLE_PRESS) / 2, s0 - 1, s1 + 1, dx, dy),
            cyl((g["rod_d"] + fits.HOLE_PRESS) / 2, s0 - 1, s1 + 1, rx, ry),
            cyl((g["idl_j"] + fits.SHAFT_IN_BORE) / 2, s0 - 1, s1 + 1, ix, iy)]
    for x in (dx - 28.0, dx + 28.0):
        cuts.append(prim.place(cyl(1.25, -1, 10, seg=24), tf.T(x, yd + 0.5, 0.5 * (s0 + s1)) @ tf.Rx(90)))
    return prim.diff(b, cuts)


def head_front_holes(g, c):
    """Cutters (Ld) for the reel head front plate, which plays bracket A: drum axle and guide rod press holes, the
    N20 shaft arc slot, the cradle pivot and swing screw holes (tapped), the SW_N20 nut pocket and its wire groove."""
    s0, s1 = g["brA"]
    dx, dy = drum_xy(g)
    rx, ry = g["rod_xy"]
    nx, ny = n20_xy(g, c)
    cuts = [cyl((g["axle_d"] + fits.HOLE_PRESS) / 2, s0 - 1, s1 + 1, dx, dy),
            cyl((g["rod_d"] + fits.HOLE_PRESS) / 2, s0 - 1, s1 + 1, rx, ry)]
    pv = [q for q in cradle_holes(g, c) if q[2] == "pivot"][0]
    rr = math.dist((nx, ny), (pv[0], pv[1]))
    a0 = math.atan2(ny - pv[1], nx - pv[0])
    pts = [cyl(2.5, s0 - 1, s1 + 1, pv[0] + rr * math.cos(a0 + math.radians(da)),
               pv[1] + rr * math.sin(a0 + math.radians(da)), seg=24) for da in np.linspace(0, 15, 7)]
    cuts.append(prim.union([prim.union([pts[i], pts[i + 1]]).hull() for i in range(len(pts) - 1)]))
    for (px, py, kind) in cradle_holes(g, c):
        if kind == "nut":
            cuts.append(prim.extrude(hex_cs(5.6), 2.7, z=s1 - 2.6).translate((px, py, 0)))
            cuts.append(prim.box(px, px + 30.0, py - 0.8, py + 0.8, s1 - 1.6, s1 + 1))
        else:
            cuts.append(cyl(1.25, s0 - 1, s1 + 1, px, py, seg=24))
    return cuts


def head_switch_holes(g):
    """Ld (x, s) of the screws from outside the spine into bracket B's foot and the code switch block's foot."""
    dx, _ = drum_xy(g)
    s0, s1 = g["bands"][0][0] - 3.0, g["bands"][-1][1] + 1.0
    x0 = dx + lever_x0(g, g["band_r0"], g["bump"])
    out = [(x, 0.5 * sum(g["brB"])) for x in (dx - 28.0, dx + 28.0)]
    out += [(2 * dx - (x0 + 19.0), sc) for sc in (s0 + 5.0, s1 - 5.0)]
    return out


# ------------------------------------------------------------------------------------------------ bridge: shift rail, fork, bell crank, dog post
def fork_planes(g):
    """Fork axial planes at NEUTRAL: bed plane sb (pads, yoke and tab sit on it), yoke top, groove centre."""
    pw = (g["groove"][1] - g["groove"][0]) - 0.6
    sb = 0.5 * (g["groove"][0] + g["groove"][1]) - pw / 2
    return dict(pw=pw, sb=sb, yt=sb + g["fork_t"])


def shift_rail(g):
    """Shift rail d10 (PPA-CF), Lb: pressed into the bridge plate boss, carries the fork sleeve and the dog post; 1 mm
    print flat on the -x side."""
    rx, ry = g["rail_xy"]
    r = cyl(g["rail_d"] / 2, g["rail_s"][0], g["rail_s"][1], rx, ry)
    return r - prim.box(rx - 10, rx - g["rail_d"] / 2 + 1.0, ry - 10, ry + 10, g["rail_s"][0] - 1, g["rail_s"][1] + 1)


def fork(g, c):
    """Fork at NEUTRAL in Lb (PLA): thin yoke (lower half ring round the collar groove, pads at 3 and 9 o'clock) is
    the spring; hub and arm to the sleeve on the shift rail; the lug under the sleeve takes the bell crank's output pin
    in a slot along u; the tab (-x, nut inside) touches the SW_DOG_L / SW_DOG_R screws on the dog post only when the
    collar is fully engaged. Prints standing on the sleeve end."""
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
    tab = prim.box(-(r1 + 8.0), -r0, -2.6, 6.6, sb, yt)
    rx, ry = g["rail_xy"]
    hl = g["hub_len"]
    hub = prim.box(-6, 6, r1 - 1.0, r1 + 7.0, sb, sb + hl)
    arm1 = shp_extrude(LineString([(0.0, r1 + 3.0), (rx, ry)]).buffer(5.0), sb, sb + hl)
    fs0, fs1 = g["fork_sleeve"]
    sleeve = cyl(g["rail_d"] / 2 + 4.0, fs0, fs1, rx, ry)
    cx, cs = g["crank_pivot"]
    lw0, lw1 = g["lug_w"]
    lug = prim.box(rx - 6.0, rx + 6.0, ry + g["rail_d"] / 2 + 2.0, -lw0, cs - 5.0, cs + 5.0)
    f = prim.union([yoke] + pads + [tab, hub, arm1, sleeve, lug])
    f = f - cyl((g["rail_d"] + fits.HOLE_SLIDE) / 2, fs0 - 1, fs1 + 1, rx, ry)
    f = f - prim.box(rx - 3.0, rx + 3.0, -lw1 - 0.5, -lw0 + 1.0, cs - 1.75, cs + 1.75)      # slot for the crank pin
    f = f - prim.place(prim.extrude(hex_cs(5.6), yt - sb + 2, z=sb - 1), tf.T(-(r1 + 4.0), 2.0, 0) @ tf.Rz(30))
    return f


def dog_post(g, c):
    """Dog contact post (PLA), Lb: clamped on the shift rail beyond the fork sleeve, a bar along the yoke tab (-x) with
    the SW_DOG_R (REEL side) and SW_DOG_L (LAUNCH side) M3 screws, heads set to touch the tab nut dog_touch before the
    collar is fully engaged. Open = not engaged."""
    F = fork_planes(g)
    sb, yt = F["sb"], F["yt"]
    r0, r1 = g["yoke_r"]
    rx, ry = g["rail_xy"]
    p0, p1 = g["dogpost_s"]
    st = c["stroke"]
    head = 3.0
    sR = sb - st + g["dog_touch"] - head
    sL = yt + st - g["dog_touch"] + head
    bx0 = -(r1 + 8.5)
    clamp = cyl(g["rail_d"] / 2 + 5.0, p0, p1, rx, ry)
    arm = shp_extrude(LineString([(rx, ry), (bx0 - 2.25, 4.0)]).buffer(4.0), p0, p1)
    b0 = sR - 3.0
    bar = prim.box(bx0 - 4.5, bx0, -3.0, 7.0, b0, p1)
    bR = prim.box(bx0 - 0.01, -(r1 + 1.0), -1.0, 5.0, b0, sR)
    bL = prim.box(bx0 - 0.01, -(r1 + 1.0), -1.0, 5.0, sL, sL + 6.0)
    d = prim.union([clamp, arm, bar, bR, bL])
    d = d - cyl((g["rail_d"] + fits.HOLE_SLIDE) / 2, p0 - 1, p1 + 1, rx, ry)
    d = d - cyl(1.25, b0 - 1, sL + 7.0, -(r1 + 4.0), 2.0, seg=24)
    d = d - prim.place(cyl(1.25, 0, 12, seg=24), tf.T(rx - 4.0, ry, 0.5 * (p0 + p1)) @ tf.Ry(-90))
    return d


def crank(g):
    """Bell crank (PPA-CF), Lb at NEUTRAL: pivot parallel to w at crank_pivot, input arm along -s to the link eye
    (M3), output arm along +x to the pin that rides the fork lug slot. 1:1 (arms crank_arm). Prints flat."""
    cx, cs = g["crank_pivot"]
    a = g["crank_arm"]
    rx, _ = g["rail_xy"]
    w0, w1 = g["crank_w"]
    y0, y1 = -w1, -w0                                   # Lb y = -w
    poly = unary_union([Point(cx, cs).buffer(7.0, 48), LineString([(cx, cs), (cx, cs - a)]).buffer(4.5),
                        Point(cx, cs - a).buffer(5.5, 32), LineString([(cx, cs), (rx, cs)]).buffer(4.5),
                        Point(rx, cs).buffer(4.5, 32)])
    # the crank lies in the plane y0..y1: build in (x, s) and turn so the extrusion runs along Lb y
    body = prim.extrude(prim.shapely_cs(poly), y1 - y0)
    M = np.eye(4)
    M[:3, 0], M[:3, 1], M[:3, 2] = (1, 0, 0), (0, 0, 1), (0, -1, 0)      # x = x, y = s, z = -y
    M[:3, 3] = (0, y1, 0)
    body = prim.place(body, M)
    pin = prim.cylY(1.5, -g["lug_w"][1] + 0.4, y0 + 0.01, x=rx, z=cs, seg=24)
    body = prim.union([body, pin])
    body = body - prim.cylY(1.7, y0 - 1, y1 + 1, x=cx, z=cs, seg=24)            # pivot screw, clearance
    body = body - prim.cylY(1.25, y0 - 1, y1 + 1, x=cx, z=cs - a, seg=24)       # link eye screw, tapped
    return body


def bridge_adds(P=None):
    """Solids (RAIL frame) cad-reel unions into its bridge plate: the shift rail boss, the tongue down to the crank
    shelf, the shelf and its pivot boss."""
    g = G(P)
    Mb = bridge_M()
    rx, ry = g["rail_xy"]
    b0, b1 = g["rail_boss_s"]
    cx, cs = g["crank_pivot"]
    sw0, sw1 = g["shelf_w"]
    adds = [cyl(g["rail_d"] / 2 + 5.0, b0, b1, rx, ry),
            prim.box(rx - 15.5, rx - 4.3, ry, -sw0, b0, 35.0 - 0.3),             # tongue (in the plate thickness)
            prim.box(cx - 10.0, rx - 5.0, -sw1, -sw0, b0, cs + 7.0)]             # shelf under the crank
    return [prim.place(a, Mb) for a in adds]


def bridge_cuts(P=None):
    g = G(P)
    Mb = bridge_M()
    rx, ry = g["rail_xy"]
    b0, b1 = g["rail_boss_s"]
    cx, cs = g["crank_pivot"]
    sw0, sw1 = g["shelf_w"]
    cuts = [cyl((g["rail_d"] + fits.HOLE_PRESS) / 2, b0 - 1, b1 + 1, rx, ry),
            prim.cylY(1.25, -sw1 - 1.0, -sw0 + 1.0, x=cx, z=cs, seg=24)]
    return [prim.place(c_, Mb) for c_ in cuts]


def bridge_bosses(P=None):
    """Shapely (u, w) circles cad-reel adds to the bridge outline (the shift rail boss)."""
    g = G(P)
    rx, ry = g["rail_xy"]
    return [Point(rx, -ry).buffer(g["rail_d"] / 2 + 5.0, 48)]


def head_keepout(P=None, margin=3.0):
    """Keep-out (RAIL frame) of the rail drum group: drum, code switch block and levers, guide rod and slider, N20 and
    cradle, bracket B, the W idler; cad-reel cuts its plates round it."""
    g, c = G(P), C(P)
    dx, dy = drum_xy(g)
    rx, ry = g["rod_xy"]
    ix, iy = idler_xy(g, c)
    s0 = g["dgear"][0] - 0.5
    ko = [cyl(51.0 + margin, g["dgear"][0] - 0.5, g["dgear"][1] + 0.5, dx, dy),
          cyl(34.5 + margin, g["dgear"][1], g["lock"][1] + 0.5, dx, dy),
          cyl(39.5 + margin, g["seg"][0] - 0.5, g["lock"][1] + 0.5, dx, dy),
          cyl(9.5 + margin, s0, g["brB"][0], rx, ry),
          cyl(9.0 + margin, g["seg"][0] - 0.5, g["idl"][1], ix, iy)]
    x0 = dx + lever_x0(g, g["band_r0"], g["bump"])
    bx = 2 * dx - x0
    ko.append(prim.box(bx - 16.0 - margin, dx, dy - 40.0, deck_y(g) + 1, g["bands"][0][0] - 4.0, g["bands"][-1][1] + 2.0))
    return prim.place(prim.union(ko), head_M(g))


def head_p2_cuts(P, u0, u1):
    """Cutters (RAIL frame) through cad-reel's P2: the wrap idler W rear journal."""
    g, c = G(P), C(P)
    ix, iy = idler_xy(g, c)
    return [prim.place(cyl((g["idl_j"] + fits.SHAFT_IN_BORE) / 2, g["idl_p2"][0] - 2.0, g["idl_p2"][1] + 2.0, ix, iy),
                       head_M(g))]


def head_front_cuts(P=None):
    """Cutters (RAIL frame) through cad-reel's head front plate (it plays bracket A): axle, guide rod, N20 cradle."""
    g, c = G(P), C(P)
    return [prim.place(x, head_M(g)) for x in head_front_holes(g, c)]


def head_spine_holes(P=None):
    """(u, w) RAIL frame of the M3 screws through cad-reel's spine into bracket B and the code switch block."""
    g = G(P)
    o = g["head_o"]
    return [(o[0] - s_, o[2] - x) for x, s_ in head_switch_holes(g)]


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
    sc = 0.5 * (w0 + w1)
    t0, t1 = min(w0, sc - 3.0), max(w1, sc + 3.0)           # boss and foot 6 thick round their screw holes
    boss = prim.box(bx, bx + 4.0, -L + 5.0, -L + 11.0, t0, t1)
    nub = cyl(g["nub_r"], w0, w1, -1.0, -L, seg=32)
    arm = prim.box(-1.0, bx + 0.5, -L - 1.5, -L + 1.5, w0, w1)
    fx = bx + 4.0 + 3.0 + lever_gap(g, bump)          # block front face in the lever frame
    foot = prim.box(bx, fx, 2.0, 10.0, t0, t1)
    lv = prim.union([beam, boss, nub, arm, foot])
    lv = lv - prim.place(cyl(1.25, -1, 6, seg=24), tf.T(bx - 0.5, -L + 8.0, sc) @ tf.Ry(90))      # M3 x 6, tapped
    lv = lv - prim.place(cyl(1.7, -1, 30, seg=24), tf.T(bx - 1.0, 6.0, sc) @ tf.Ry(90))          # foot screw along x
    return lv


def contact_block(g, cx, cy, r0, bump, bands, y_top, y_bot, s0, s1):
    x0 = cx + lever_x0(g, r0, bump)
    b = prim.box(x0, x0 + 14.0, y_top, y_bot, s0, s1)
    cuts = []
    for (w0, w1) in bands:
        sc = 0.5 * (w0 + w1)
        cuts.append(prim.place(prim.extrude(hex_cs(5.6), 2.6, z=-0.01), tf.T(x0, cy + 8.0, sc) @ tf.Ry(90) @ tf.Rz(30)))
        cuts.append(prim.place(cyl(1.7, -1, 16, seg=24), tf.T(x0, cy + 8.0, sc) @ tf.Ry(90)))
        rx, ry = lever_root(g, cx, cy, r0)
        cuts.append(prim.place(cyl(1.25, -1, 12, seg=24), tf.T(x0 - 0.5, ry + 6.0, sc) @ tf.Ry(90)))   # foot screw, tapped
    return b, cuts


def sw_mirror(g):
    """The code contacts sit on the -x (back) side of the drum (reel_p2_post fills the front): everything is built
    on the +x side as for the index contact, then mirrored about the plane x = drum x."""
    dx, _ = drum_xy(g)
    return tf.T(dx, 0, 0) @ np.diag([-1.0, 1.0, 1.0, 1.0]) @ tf.T(-dx, 0, 0)


def switch_block(g, c):
    """Code switch block on the -X (back) side of the code bands (2 x M3 to the deck)."""
    dx, dy = drum_xy(g)
    yd = deck_y(g)
    s0, s1 = g["bands"][0][0] - 3.0, g["bands"][-1][1] + 1.0     # top 1.0 above band C (hand lever from 129.2)
    b, cuts = contact_block(g, dx, dy, g["band_r0"], g["bump"], g["bands"], dy + 2.0, yd, s0, s1)
    x0 = dx + lever_x0(g, g["band_r0"], g["bump"])
    foot = prim.box(x0 - 2.0, x0 + 24.0, yd - 6.0, yd, s0, s1)
    for sc in (s0 + 5.0, s1 - 5.0):
        cuts.append(prim.place(cyl(1.7, -1, 8, seg=24), tf.T(x0 + 19.0, yd + 0.5, sc) @ tf.Rx(90)))
    return prim.place(prim.diff(prim.union([b, foot]), cuts), sw_mirror(g))


def idx_R(g):
    """L frame transform of the SW_INDEX lever and block: built along +x from the axis, turned to idx_psi."""
    return tf.Rz(g["idx_psi"])


IDX_S = (0.0, 39.5)          # D49: frame-fixed column from the motor plate front face (world y 0) to the cam


def idx_mount_xy(g):
    """L (x, y) of the two M3 x 10 screws through the block flange into cad-frame's inserts on the motor plate front
    face (params DRUM sw_index_mount gives the same points as world (x, z))."""
    x0 = lever_x0(g, g["idx_r0"], g["idx_bump"])
    return [(x0 + 19.0, -4.0), (x0 + 19.0, 44.0)]


def index_block(g):
    """SW_INDEX block beside the coupler cam (+x): a column from the motor plate front face to the cam with a flange
    screwed to the plate (2 x M3 x 10 into the frame's inserts): read from the frame, never from the rail (D49)."""
    s0, s1 = IDX_S
    b, cuts = contact_block(g, 0.0, 0.0, g["idx_r0"], g["idx_bump"], [g["idx_s"]], -12.0, g["sw_L"] + 20.0, s0, s1)
    x0 = lever_x0(g, g["idx_r0"], g["idx_bump"])
    flange = prim.box(x0 + 13.9, x0 + 25.0, -12.0, g["sw_L"] + 20.0, s0, s0 + 5.0)
    b = prim.union([b, flange])
    for (x, y) in idx_mount_xy(g):
        cuts.append(cyl(1.7, s0 - 1, s0 + 6, x, y, seg=24))
    cuts.append(cyl(39.8, g["bus_s"][0] - 0.5, s1 + 1))
    cuts.append(cyl(35.5, s0 - 1, s1 + 1))                          # coupler
    return prim.place(prim.diff(b, cuts), idx_R(g))


def sw_lever(g, c, i):
    """Mirrored lever (x -> -x about its pivot): its joint turns about -z, so the same angle lifts it the same way."""
    b0, b1 = g["bands"][i]
    return prim.place(contact_lever(g, b0 + 0.6, b1 - 0.6, g["bump"]), np.diag([-1.0, 1.0, 1.0, 1.0]))


def sw_lever_root(g, i):
    dx, dy = drum_xy(g)
    rx, ry = lever_root(g, dx, dy, g["band_r0"])
    return 2 * dx - rx, ry


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
            prim.place(cyl(1.25, 0, 12, seg=24), tf.T(0, 0, e - 1.4) @ tf.Ry(90))]
    return prim.diff(cap, cuts)


def n20_model(g, c, P):
    """Right-angle worm N20 envelope (bought), in the N20 body frame (output axis = z, motor along +x)."""
    N = getattr(P, "N20_WORM", {}) or {}
    bw = float(N.get("box_w", 9.94)); bh = float(N.get("box_h", 11.93))
    ln = float(N.get("len_no_shaft", 33.04)); tip = float(N.get("shaft_tip_to_tip", 29.97))
    zc = g["dgear"][1] + 0.5 + bw / 2
    box = prim.box(-6.0, 6.0, -bh / 2, bh / 2, zc - bw / 2, zc + bw / 2)
    mot = prim.box(6.0, ln - 6.0, -5.0, 5.0, zc - bw / 2 + 0.5, zc + bw / 2 - 0.5)
    sh = cyl(1.5, zc - tip / 2, zc + tip / 2, seg=24) - prim.box(1.0, 5, -5, 5, zc - tip, zc + tip)   # D flat
    return prim.union([box, mot, sh])


def n20_frame(g, c):
    nx, ny = n20_xy(g, c)
    return tf.T(nx, ny, 0) @ tf.Rz(g["n20_motor_deg"])


def cradle_holes(g, c):
    """Cradle to bracket A ear, at full mesh: pivot screw and swing screw (both tapped in the ear; the swing screw runs
    in an arc slot in the cradle plate), SW_N20 nut (pocket in the ear front face; the screw is tapped in the cradle
    plate 2.5 mm off the nut centre so its tip lands on a flat). In L xy."""
    M = n20_frame(g, c)
    out = []
    for (x, y, kind) in ((-11.0, 12.0, "pivot"), (14.0, -11.5, "slot"), (0.0, -13.0, "nut")):
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
    plate = Polygon([(-15, -16), (18, -16), (18, 16), (-15, 16)])
    M = n20_frame(g, c)
    s_top = zc + bw / 2 + 0.3
    body = shp_extrude(outer, g["brA"][1], s_top)
    base = shp_extrude(plate, g["brA"][1], g["brA"][1] + 4.0)
    cr = prim.union([body, base])
    cav = shp_extrude(foot.buffer(0.15, join_style=2), zc - bw / 2 - 0.15, s_top + 1)
    cr = cr - cav
    cr = cr - cyl(gears.spur_info(c["drum_gear_m"], c["n20_z"], c["n20_x"])["r_tip"] + 1.2, g["brA"][1] - 1,
                  zc - bw / 2 + 0.01, seg=48)
    pvx, pvy = -11.0, 12.0
    cr = cr - cyl(1.7, g["brA"][1] - 1, g["brA"][1] + 5, pvx, pvy, seg=24)                  # pivot, clearance
    # swing: arc slot about the pivot; the ear's tapped point moves -15 deg in the cradle frame when it swings out
    rr, a0 = math.hypot(14.0 - pvx, -11.5 - pvy), math.atan2(-11.5 - pvy, 14.0 - pvx)
    pts = [cyl(1.8, g["brA"][1] - 1, g["brA"][1] + 5, pvx + rr * math.cos(a0 - math.radians(da)),
               pvy + rr * math.sin(a0 - math.radians(da)), seg=24) for da in np.linspace(0, 15, 7)]
    cr = cr - prim.union([prim.union([pts[i], pts[i + 1]]).hull() for i in range(len(pts) - 1)])
    # SW_N20: M3 x 8 tapped in the plate from +s, tip on a flat of the nut in the ear (closed only at full mesh)
    cr = cr - cyl(1.25, g["brA"][1] - 1, g["brA"][1] + 5, 2.5, -13.0, seg=24)
    # strap lugs: the strap screws sit in r 4 lugs clear of the N20 cavity
    cr = prim.union([cr] + [cyl(4.0, g["brA"][1], s_top, x, 0.0) for x in (-11.5, 34.0)])
    for (x, y) in ((-11.5, 0.0), (34.0, 0.0)):
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
    st = prim.union([shp_extrude(foot.buffer(3.0, join_style=2), s_top, s_top + 3.0)] +
                    [cyl(4.0, s_top, s_top + 3.0, x, 0.0) for x in (-11.5, 34.0)])
    tip = float(N.get("shaft_tip_to_tip", 29.97))
    st = st - cyl(2.0, s_top - 1, s_top + 4, seg=24)                         # N20 rear shaft end
    for (x, y) in ((-11.5, 0.0), (34.0, 0.0)):
        st = st - cyl(1.7, s_top - 1, s_top + 4, x, y, seg=24)
    return prim.place(st, n20_frame(g, c))


# ------------------------------------------------------------------------------------------------ module contract
COL = dict(ppa="#3a3f45", pla="#c8ccd2", gear="#d97a00", drum="#3d7cc9", fork="#7fb069", sw="#e0c040")


def parts(P):
    g, c = G(P), C(P)
    I = tf.I4
    out = []
    MH = head_M(g)
    MB = bridge_M()

    def add(name, solid, mat, joint, R=None, color="pla", notes="", **kw):
        solid = prim.clean(solid) if hasattr(prim, "clean") else solid
        out.append(Part(name, solid, mat, joint=joint, print_tf=None if mat in ("bought", "steel") else bed(solid, I if R is None else R),
                        color=COL.get(color, color), notes=notes, **kw))

    # main axis (world-fixed frame L)
    add("drum_coupler_hub", coupler_hub(g, P), "PPA-CF", "drum_main", color="ppa",
        notes="Coupler hub on the wiper shaft (bore parametric: WIPER), M8 nut inside, shear pin hole")
    add("drum_coupler", coupler(g, P), "PPA-CF", "drum_main", color="ppa",
        notes="Coupler: 5 N m shear pin limiter (D8), bus gear 50T m1.5 (D25), SW_INDEX cam, REEL journal, hex socket; prints hub end down, tree support under the bus gear ring")
    add("drum_shear_pin", shear_pin(g), "PLA", "drum_main", R=tf.Ry(180), color="sw", qty=6,
        infill=1.0, notes="Torque limiter shear pin d4 (about 5 N m), prints lying; print 6, 5 are spares")
    add("drum_shaft", main_shaft(g), "PPA-CF", "drum_main", R=tf.Rx(90) @ tf.Ry(90), color="ppa",
        infill=1.0, notes="Main shaft: hex 22 AF, d12 spigot into the launch sleeve; prints on a hex flat (support under the spigot)")
    add("drum_reel_washer", reel_washer(g), "PPA-CF", "drum_main", color="ppa",
        notes="REEL gear retaining washer on the coupler journal end, 2 x M3 x 6")
    add("drum_gear_reel", reel_gear(g, c), "PPA-CF", "drum_out_reel", color="gear",
        notes="REEL output gear m2 50T face 12 (D20), free on its sleeve, dog face toward the collar; meshes C1 on the rail bridge (D49)")
    add("drum_collar", collar(g, c), "PPA-CF", "drum_collar", color="ppa",
        infill=1.0, notes="Dog collar: hex bore, 4 back-cut roofed dogs each face, fork groove (bridges over the lower dogs)")
    add("drum_dogring_launch", dog_ring(g, c), "PPA-CF", "drum_out_launch", R=tf.Rx(180), color="gear",
        infill=1.0, notes="LAUNCH dog ring: 3 slots for the launch sleeve lugs, 3 x M3 from the sleeve side")
    add("drum_index_block", index_block(g), "PLA", "drum_base", color="pla", bought_ref="2 x M3x10 SHCS",
        notes="SW_INDEX block: column on the motor plate front face (2 x M3 x 10 into the frame inserts), M3 nut contact")
    add("drum_index_lever", idx_lever(g), "PLA", "drum_idx", color="sw",
        notes="SW_INDEX lever on the 6-bump coupler cam (D12 fallback encoder)")
    add("drum_magnet_cap", magnet_holder(g), "PLA", "drum_main", R=tf.Rx(180), color="pla",
        notes="AS5600 magnet cap on the spigot end (6 x 2.5 diametric or edge-on disc), M3 x 6 onto the flat")
    # rail head (frame Ld on the rail)
    add("drum_drum_a", drum_a(g, c), "PLA", "drum_drum", color="drum",
        notes="Drum A: N20 gear 120T m0.8, groove 1 barrel (follower), hex spigot")
    add("drum_drum_b", drum_b(g, c), "PLA", "drum_drum", R=tf.Rx(180), color="drum",
        notes="Drum B: brake cam (spool clutch lever), Gray code bands A B C, wrap segment gear + locking arc")
    add("drum_axle", axle(g), "PPA-CF", "drum_head", R=tf.Ry(90), color="ppa",
        notes="Fixed drum axle d10, pressed into the reel head front plate and bracket B")
    add("drum_rod", rod(g), "PPA-CF", "drum_head", R=tf.Ry(-90), color="ppa",
        notes="Follower guide rod d10, pressed into the reel head front plate and bracket B")
    add("drum_follower", follower(g, c), "PLA", "drum_follow", color="fork",
        notes="Follower slider: pin in groove 1, slides along the guide rod; the link takes its motion to the bridge")
    add("drum_idler_gear", idler_gear(g, c), "PPA-CF", "drum_idler", color="gear",
        notes="Wrap idler output gear 49T m1.5 (D bore) to the cad-reel cup gear 28T: 7 cup turns")
    add("drum_idler", idler(g, c), "PPA-CF", "drum_idler", color="gear",
        notes="Wrap idler shaft: pinion 10T m1.25 (locked at home by the drum lock arc), lock cam, journals in bracket B and P2")
    add("drum_n20_pinion", n20_pinion(g, c), "PPA-CF", "drum_n20", color="gear",
        notes="N20 pinion 10T m0.8, D bore pressed on the N20 shaft")
    add("drum_n20", n20_model(g, c, P), "bought", "drum_n20_body", color="#8a8f96",
        bought_ref="N20 right-angle self-locking worm gearmotor 12 V 27 rpm (have)",
        notes="Selector drum motor, DRV8833 #3 at 9 V; rides the rail (strap it: it sees every shot)")
    add("drum_n20_cradle", n20_cradle(g, c, P), "PLA", "drum_head", color="pla",
        notes="N20 cradle on the reel head front plate: pivot + swing screw (hand fallback), SW_N20 contact")
    add("drum_n20_strap", n20_strap(g, c, P), "PLA", "drum_head", color="pla",
        notes="End plate closing the N20 cradle, 2 x M3 x 8")
    add("drum_bracket_b", bracket_b(g, c), "PLA", "drum_head", color="pla", bought_ref="2 x M3x12 SHCS (through the spine)",
        notes="Drum bracket B: axle, guide rod, wrap idler journal; foot screwed to the reel head spine")
    add("drum_switch_block", switch_block(g, c), "PLA", "drum_head", color="pla", bought_ref="2 x M3x12 SHCS (through the spine)",
        notes="Code switch block on top of the drum: 3 lever feet, 3 M3 nut contacts (SW_DRUM_A, B, C)")
    for i in range(3):
        add(f"drum_sw_lever_{i}", sw_lever(g, c, i), "PLA", f"drum_sw{i}", color="sw",
            notes=f"Code switch lever {'ABC'[i]}: flexure, round nub, M3 x 6 contact screw")
    lk, _, _ = link(g, P)
    add("drum_link", lk, "steel", "drum_link", color="#8a8f96", bought_ref="8 mm steel rod, cut to the length on the print sheet",
        notes="Link rod from the follower to the bell crank on the bridge (8 mm rod, printed eyes)")
    for e in (0, 1):
        add(f"drum_link_eye_{e + 1}", link_eye(g, e), "PLA", "drum_link", R=link_eye_bed(g, e), color="fork",
            bought_ref="M3x16 SHCS + nyloc nut", notes="Link eye pressed on the rod end, M3 screw through the "
                                                      + ("follower lug" if e == 0 else "bell crank input arm"))
    # rail bridge (frame Lb on the rail)
    add("drum_shift_rail", shift_rail(g), "PPA-CF", "drum_bridge", R=tf.Ry(-90), color="ppa",
        notes="Shift rail d10 on the rail bridge (pressed into the bridge plate boss)")
    add("drum_fork", fork(g, c), "PLA", "drum_fork", color="fork",
        notes="Compliant shift fork: thin yoke is the spring (about 1.5 N/mm), lug for the bell crank pin, contact tab")
    add("drum_dog_post", dog_post(g, c), "PLA", "drum_bridge", R=tf.Rx(180), color="pla",
        notes="Dog contact post on the shift rail: SW_DOG_R and SW_DOG_L screws (collar fully engaged)")
    add("drum_crank", crank(g), "PPA-CF", "drum_crank", R=tf.Rx(-90), place=tf.T(-g["crank_pivot"][0], 0, -g["crank_pivot"][1]), color="ppa", bought_ref="M3x16 SHCS (pivot, tapped into the shelf)",
        notes="Bell crank 1:1 on the bridge shelf: turns the link's motion along the rail into the fork's shift")
    return out


def joints(P):
    g, c = G(P), C(P)
    dx, dy = drum_xy(g)
    ix, iy = idler_xy(g, c)
    nx, ny = n20_xy(g, c)
    cx, cs = g["crank_pivot"]
    js = [
        Joint("drum_base", None, "fixed", tuple(g["main_origin"]), (0, 0, 1), base=tf.Rx(-90)),
        Joint("drum_main", "drum_base", "rev", (0, 0, 0), (0, 0, 1)),
        Joint("drum_out_reel", "drum_base", "rev", (0, 0, 0), (0, 0, 1)),
        Joint("drum_out_launch", "drum_base", "rev", (0, 0, 0), (0, 0, 1), gear=_launch_gear()),
        Joint("drum_collar_slide", "drum_base", "slide", (0, 0, 0), (0, 0, 1), limits=(-c["stroke"] - 0.01, c["stroke"] + 0.01)),
        Joint("drum_collar", "drum_collar_slide", "rev", (0, 0, 0), (0, 0, 1), gear=("drum_main", 1.0, 0.0)),
        Joint("drum_head", "launch_rail", "fixed", (0, 0, 0), (0, 0, 1), base=head_M(g)),
        Joint("drum_bridge", "launch_rail", "fixed", (0, 0, 0), (0, 0, 1), base=bridge_M()),
        Joint("drum_drum", "drum_head", "rev", (dx, dy, 0), (0, 0, 1), limits=(c["stop_lo"], c["stop_hi"])),
        Joint("drum_idler", "drum_head", "rev", (ix, iy, 0), (0, 0, 1)),
        Joint("drum_n20", "drum_head", "rev", (nx, ny, 0), (0, 0, 1)),
        Joint("drum_n20_body", "drum_head", "fixed", (nx, ny, 0), (0, 0, 1), base=tf.Rz(g["n20_motor_deg"])),
        Joint("drum_follow", "drum_head", "slide", (0, 0, 0), (0, 0, 1), limits=(-c["stroke"] - 0.01, c["stroke"] + 0.01)),
        Joint("drum_link", "launch_rail", "slide", (0, 0, 0), (-1, 0, 0), limits=(-c["stroke"] - 0.01, c["stroke"] + 0.01)),
        Joint("drum_fork", "drum_bridge", "slide", (0, 0, 0), (0, 0, 1), limits=(-c["stroke"] - 0.01, c["stroke"] + 0.01)),
        Joint("drum_crank", "drum_bridge", "rev", (cx, 0, cs), (0, 1, 0), limits=(-10.0, 10.0)),
    ]
    for i in range(3):
        rx, ry = sw_lever_root(g, i)
        js.append(Joint(f"drum_sw{i}", "drum_head", "rev", (rx, ry, 0), (0, 0, -1), limits=(-0.5, 8.0)))
    rx, ry = lever_root(g, 0.0, 0.0, g["idx_r0"])
    p = idx_R(g) @ np.array([rx, ry, 0.0, 1.0])
    js.append(Joint("drum_idx", "drum_base", "rev", (p[0], p[1], 0), (0, 0, 1), base=idx_R(g), limits=(-0.5, 8.0)))
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


def meshes(P):
    """Gear meshes for cad-infra's mesh check (gear axis = part-local +Z; frames put z 0 at each gear's face start)."""
    g, c = G(P), C(P)
    T = tf.T
    out = [
        Mesh("drum_drum_a", "drum_n20_pinion", c["drum_gear_m"], c["drum_gear_z"], c["n20_z"], c["drum_gear_x"],
             c["n20_x"], face_a=g["dgear"][1] - g["dgear"][0], face_b=g["dgear"][1] - g["dgear"][0],
             frame_a=T(0, 0, g["dgear"][0]), frame_b=T(0, 0, g["dgear"][0]), why="N20 pinion 10T to drum gear 120T"),
        Mesh("drum_drum_a", "drum_idler", c["seg_m"], c["seg_z"], c["pin_z"], 0.0, c["pin_x"],
             face_a=g["seg"][1] - g["seg"][0], face_b=g["seg"][1] - g["seg"][0] - 0.3,
             frame_a=T(0, 0, g["seg"][0]), frame_b=T(0, 0, g["seg"][0]), why="segment gear to wrap pinion (WRAP only)"),
    ]
    # the segment gear only has teeth over the WRAP zone (meshcheck reads Mesh.when when present)
    out[1].when = lambda S: c["seg_start"] - 2 <= drum_theta(S, c) <= c["seg_start"] + K.seg_span(c) + 2
    import os
    # cross-owner meshes with cad-reel (tools.meshcheck skips a mesh whose other part is not loaded)
    if os.environ.get("DRUM_REEL_MESHES", "1") == "1":
        out += [
            Mesh("drum_coupler", "reel_c1p", g["bus_m"], g["bus_z"], 46, face_a=g["bus_s"][1] - g["bus_s"][0], face_b=6.0,
                 frame_a=T(0, 0, g["bus_s"][0]), frame_b=T(0, 0, 40.0 - 140.0), why="bus gear 50T to cad-reel c1' 46T on the rail bridge"),
            Mesh("drum_gear_reel", "reel_c1", g["out_m"], g["out_z"], 49, face_a=g["out_face"], face_b=12.0,
                 frame_a=T(0, 0, g["rgear_s"]), frame_b=T(0, 0, 50.5 - 140.0), why="REEL gear 50T to cad-reel C1 49T"),
            Mesh("drum_idler_gear", "reel_gk", c["idl_m"], c["idl_z"], c["ring_z"],
                 face_a=g["idl_gear"][1] - g["idl_gear"][0], face_b=7.0, frame_a=T(0, 0, g["idl_gear"][0]),
                 frame_b=tf.I4, why="wrap idler 49T to cad-reel cup gear 28T"),
        ]
    return out


def drum_theta(S, c):
    if "drum_deg" in S and S["drum_deg"] is not None:
        return float(S["drum_deg"])
    m = str(S.get("mode", "LAUNCH")).upper()
    return {"LAUNCH": c["launch"], "NEUTRAL": c["neutral"], "REEL": c["reel"], "WRAPPED": c["wrapped"]}.get(m, c["launch"])


def _lever_solve(f, lo=-0.5, hi=8.0):
    """Lever angle (deg) where f(a) = nub centre radius - designed path radius crosses 0 (f grows with a)."""
    if f(lo) >= 0.0:
        return lo
    if f(hi) <= 0.0:
        return hi
    for _ in range(40):
        m = 0.5 * (lo + hi)
        if f(m) < 0.0:
            lo = m
        else:
            hi = m
    return 0.5 * (lo + hi)


def _nub(root, L, a):
    """Nub centre after turning the lever by a (deg) about its root, in the frame it was built in."""
    ca, sa = math.cos(math.radians(a)), math.sin(math.radians(a))
    return root[0] + (-1.0 * ca + L * sa), root[1] + (-1.0 * sa - L * ca)


def lever_deg(theta, g, c, i):
    """Code lever angle (deg about its root): solved so the nub centre sits exactly on the designed path (the nub
    swings on an arc, so a plain asin(lift / L) is off by up to 0.5 mm on the steep code ramps)."""
    dx, dy = drum_xy(g)
    root = lever_root(g, dx, dy, g["band_r0"])
    L = g["sw_L"]

    def f(a):
        bx, by = _nub(root, L, a)
        px, py = 2 * dx - bx, by                       # mirrored to the drum back
        beta = math.degrees(math.atan2(py - dy, px - dx))
        return math.hypot(px - dx, py - dy) - nub_centre_r(g, c, i, beta - theta)
    return _lever_solve(f)


def idx_lever_deg(wiper, g):
    """SW_INDEX lever angle, solved like lever_deg (nub exactly on the designed path)."""
    root = lever_root(g, 0.0, 0.0, g["idx_r0"])
    L = g["sw_L"]
    R = idx_R(g)

    def f(a):
        bx, by = _nub(root, L, a)
        p = R @ np.array([bx, by, 0.0, 1.0])
        beta = math.degrees(math.atan2(p[1], p[0]))
        return math.hypot(p[0], p[1]) - idx_centre_r(g, beta - wiper)
    return _lever_solve(f)


# Tooth phase of the cad-reel gears that mesh mine (measured with the exact meshes at params 1.2.6; centre of the
# zero-overlap window). My gears: tooth 0 on world +x at joint angle 0, joint axis world +y. In cad-reel's joints:
#   reel_bus     = (50 / 46) * wiper_deg + 3.91       (my bus gear 50T m1.5 on drum_main = wiper_deg)
#   reel_capstan = (50 / 22) * reel_deg + 8.18        (my REEL gear 50T m2 on drum_out_reel = reel_deg)
#   reel_cup     = (42 / 24) * idler + 3.01           (my 42T idler gear on drum_idler = reel_idler_deg(P, S))
REEL_MESH = dict(bus_offset=3.91, capstan_offset=8.18, cup_offset=3.01)


def reel_idler_deg(P, S):
    """drum_idler joint angle (the 42T wrap idler gear) for state S: cad-reel's cup gear follows it."""
    g, c = G(P), C(P)
    return pinion_angle(drum_theta(S, c), g, c)


def reel_cup_deg(P, S):
    """cad-reel cup gear joint angle that meshes my 42T idler gear in state S (reel_cup joint convention)."""
    return (42.0 / 24.0) * reel_idler_deg(P, S) + REEL_MESH["cup_offset"]


def kin(P, S):
    g, c = G(P), C(P)
    th = drum_theta(S, c)
    # params TIMELINE: wiper_deg drives the main shaft, coupler, bus gear and collar (always); sleeve_deg the LAUNCH dog
    # ring (else it follows launch_sleeve through Joint.gear: it is bolted to the sleeve); reel_deg the REEL gear.
    if S.get("wiper_deg") is not None:
        w = float(S["wiper_deg"])
    elif K.mode(th, c) == "LAUNCH":
        w = main_angle(P, S)      # no wiper_deg (cad-launch's own sweeps): the engaged shaft turns with the sleeve
    elif K.mode(th, c) == "REEL" and S.get("reel_deg") is not None:
        w = float(S["reel_deg"])  # engaged in REEL: the shaft turns with the REEL gear
    else:
        w = 0.0
    x = K.collar_x(th, c)
    md = K.mode(th, c)
    v = {
        "drum_drum": th,
        "drum_main": w,
        "drum_collar_slide": -x,
        "drum_fork": -x,
        "drum_follow": g["groove_sign"] * x,
        "drum_link": g["groove_sign"] * x,
        "drum_crank": math.degrees(math.asin(max(-1.0, min(1.0, g["groove_sign"] * x / g["crank_arm"])))),
        "drum_idler": pinion_angle(th, g, c),
        "drum_n20": n20_angle(th, g, c),
        "drum_out_reel": float(S["reel_deg"]) if S.get("reel_deg") is not None else (w if md == "REEL" else 0.0),
        "drum_out_launch": float(S["sleeve_deg"]) if S.get("sleeve_deg") is not None else (w if md == "LAUNCH" else 0.0),
        "drum_idx": idx_lever_deg(w, g),
    }
    for i in range(3):
        v[f"drum_sw{i}"] = lever_deg(th, g, c, i)
    if _launch_gear() is not None and S.get("sleeve_deg") is None and (md != "LAUNCH" or S.get("wiper_deg") is None):
        v.pop("drum_out_launch")              # no sleeve_deg in S: follow launch_sleeve through Joint.gear
    return v


def allowed(P):
    return [
        Allow("drum_shaft", "drum_coupler", "hex spigot in the coupler socket (sliding fit)", max_vol=1.0),
        Allow("drum_n20_pinion", "drum_n20", "pinion pressed on the N20 shaft"),
        Allow("drum_n20", "drum_n20_cradle", "N20 seated in its cradle", max_vol=1.0),
        Allow("drum_n20", "drum_n20_strap", "strap closes the cradle", max_vol=1.0),
        Allow("drum_n20", "reel_head_front", "N20 rear shaft end in the front plate slot", max_vol=1.0),
        Allow("drum_n20_cradle", "reel_head_front", "cradle screwed to the front plate", max_vol=1.0),
        Allow("drum_idler_gear", "drum_idler", "gear on the D flat", max_vol=1.0),
        Allow("drum_idler", "drum_bracket_b", "W journal in bracket B"),
        Allow("drum_idler", "reel_head_p2", "W journal in P2"),
        Allow("drum_axle", "drum_bracket_b", "axle pressed into bracket B"),
        Allow("drum_axle", "reel_head_front", "axle pressed into the front plate"),
        Allow("drum_rod", "drum_bracket_b", "rod pressed into bracket B"),
        Allow("drum_rod", "reel_head_front", "rod pressed into the front plate"),
        Allow("drum_rod", "drum_follower", "slider on the rod"),
        Allow("drum_drum_a", "drum_drum_b", "hex spigot in its socket", max_vol=1.0),
        Allow("drum_drum_*", "drum_axle", "drum turns on the axle"),
        Allow("drum_follower", "drum_drum_a", "pin in groove 1", max_vol=0.5),
        Allow("drum_link", "drum_link_eye_*", "rod pressed into the eyes"),
        Allow("drum_link_eye_1", "drum_follower", "eye screwed to the follower lug (M3)"),
        Allow("drum_link_eye_2", "drum_crank", "eye screwed to the crank input arm (M3)"),
        Allow("drum_shift_rail", "reel_bridge", "rail pressed into the bridge boss"),
        Allow("drum_shift_rail", "drum_fork", "fork sleeve slides on the rail"),
        Allow("drum_shift_rail", "drum_dog_post", "dog post clamped on the rail"),
        Allow("drum_crank", "drum_fork", "crank pin in the fork lug slot", max_vol=1.0),
        Allow("drum_bracket_b", "reel_head_spine", "bracket foot screwed to the spine", max_vol=1.0),
        Allow("drum_switch_block", "reel_head_spine", "block foot screwed to the spine", max_vol=1.0),
        Allow("drum_shear_pin", "drum_coupler*", "shear pin in its holes", max_vol=2.0),
        Allow("drum_magnet_cap", "drum_shaft", "cap on the spigot end", max_vol=1.0),
    ]


def clearances(P):
    return [
        Clearance("drum_drum_b", "drum_idler", 0.0, "segment gear, locking arc and wrap pinion"),
        Clearance("drum_drum_a", "drum_n20_pinion", 0.0, "N20 mesh"),
        Clearance("drum_drum_a", "drum_follower", 0.05, "follower pin in groove 1"),
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
    for a in (6.0, 25.0, 47.0):                 # the bridge and head pitch with the rail (aim only in LAUNCH)
        for th in (c["launch"], c["neutral"], c["reel"]):
            out.append(dict(phase="drum_pitch", t=0.0, aim_deg=a, drum_deg=th))
    return out


def grounded(P):
    # coupler: on the wiper shaft (frame motor); dog ring: bolted to the cad-launch sleeve; index block: motor plate
    return ["drum_coupler*", "drum_dogring_launch", "drum_index_block"]


def steps(P):
    """Assembly steps, in order, plain language, hardware per step. Main axis 130..149, bridge 220..224, head
    225..239 (params ASSEMBLY_STAGES)."""
    g = G(P)
    return [
        dict(title="Main shaft: coupler and limiter", stage=130,
             text="Push the coupler hub onto the wiper output shaft (flats in line) until it stops on the end of the "
                  "flats; its lower end goes into the puck bore, just clear of the motor's output boss. Drop the M8 nut "
                  "into the hub pocket onto the thread (it sits about %.0f mm down the pocket) and tighten it with a 13 mm "
                  "socket (1/4 inch drive). Slide the outer coupler over the hub, line up the holes, push a shear pin in "
                  "(head outside, its flat facing the outboard end) and put a piece of tape round the coupler over the "
                  "head. Print the pins lying on their flat (D44: shear across the layers). Keep the 5 spare pins with the "
                  "base. A broken pin stub comes out with an M3 screw turned into it. Then slide the REEL gear onto the "
                  "coupler journal (plain face first) and screw the retaining washer to the journal end (2 x M3 x 6): "
                  "the gear must spin free." % (g["hub_s"][1] - g["nut_s"] - NUT_M8_H),
             parts=["drum_coupler_hub", "drum_coupler", "drum_shear_pin", "drum_gear_reel", "drum_reel_washer"]),
        dict(title="Main shaft: index contact", stage=131,
             text="Press an M3 nut into the index block pocket, screw the index lever foot to the block (M3 x 12), thread "
                  "an M3 x 6 into the lever boss, head toward the nut. Screw the block's flange to the motor plate front "
                  "face with two M3 x 10 into the frame inserts, nub on the coupler cam (D49: SW_INDEX stays on the frame, "
                  "never on the rail). Set the gap so the screw head touches the nut near the top of a bump (about 1.5 mm "
                  "on a low part). Battery unplugged: nut to GND, screw to SW_INDEX.",
             parts=["drum_index_block", "drum_index_lever"]),
        dict(title="Main shaft: stack", stage=140,
             text="Slide the collar onto the main shaft from the spigot end and push the shaft about 42 mm into the "
                  "coupler socket. The shaft is held endwise later by the launch sleeve and the magnet cap.",
             parts=["drum_shaft", "drum_collar"]),
        dict(title="Main shaft: LAUNCH dog ring and magnet cap", stage=141,
             text="Fit the LAUNCH dog ring on the launch sleeve flange (sleeve lugs in the ring slots, 3 x M3 x 10 from "
                  "the sleeve side into the ring) before the launch sleeve goes on the spigot (launcher steps). After "
                  "the launcher: glue the AS5600 magnet into the cap, push the cap on the spigot end (flat to flat) and "
                  "fit the M3 x 6 onto the flat.",
             parts=["drum_dogring_launch", "drum_magnet_cap"]),
        dict(title="Bridge: shift rail, fork, bell crank and dog contacts", stage=224,
             text="Press the shift rail into the bridge plate boss (flat facing the rail). Press an M3 nut into the hex "
                  "hole of the fork's tab. Thread an M3 x 8 into each dog post boss, heads toward each other. Slide the "
                  "fork sleeve onto the rail with the yoke under the collar and drop the yoke pads into the collar groove, "
                  "then slide the dog post onto the rail end and tighten its clamp screw (M3 x 8, tapped) on the rail "
                  "flat. Put the bell crank on the shelf with its pin up in the fork lug's slot and fit the M3 x 16 pivot "
                  "screw (tapped into the shelf, the crank must swing). Battery unplugged: nut wire to GND, REEL side "
                  "screw to SW_DOG_R, LAUNCH side screw to SW_DOG_L; the wires run in the flex loop to the frame.",
             parts=["drum_shift_rail", "drum_fork", "drum_dog_post", "drum_crank"]),
        dict(title="Head: drum, guide rod and follower", stage=229,
             text="Press the drum axle and the follower guide rod into the reel head front plate (flats as on the print "
                  "sheet). Slide drum A onto the axle, gear first, then the follower onto the rod with its pin in drum "
                  "A's groove, then drum B (hex socket over the spigot; it fits one way only). Press bracket B onto the "
                  "axle and rod ends with the wrap idler shaft in its journal and screw the bracket's foot to the spine "
                  "(2 x M3 x 12 from outside the spine).",
             parts=["drum_axle", "drum_rod", "drum_drum_a", "drum_follower", "drum_drum_b", "drum_bracket_b"]),
        dict(title="Head: wrap idler", stage=230,
             text="Push the 49-tooth output gear onto the idler shaft's flat (D bore). Feed the shaft into bracket B "
                  "from the front so the small pinion sits at the drum's segment teeth and the lock cam over the lock "
                  "ring; its rear journal goes into P2 when P2 goes on. Turn the drum to LAUNCH: the flat-sided lock cam "
                  "must sit in the drum lock ring and the idler can no longer turn by hand.",
             parts=["drum_idler", "drum_idler_gear"]),
        dict(title="Head: N20", stage=231,
             text="Press the pinion onto the N20 output shaft (flat to flat) leaving 1.5 mm of shaft below it (that end "
                  "runs in the front plate slot). Slide the N20 into the cradle from the open end, fit the end plate "
                  "(2 x M3 x 8). Press an M3 nut into the pocket in the front plate's rear face, wire along its groove. "
                  "Screw the cradle to the front plate: pivot screw M3 x 12 through the round hole, swing screw M3 x 12 "
                  "through the arc slot at its full-mesh end, both tapped. Turn the SW_N20 screw (M3 x 8, tapped in the "
                  "cradle plate) in until its tip just touches the nut. Strap the N20 body to the cradle with a cable tie "
                  "(it rides the rail and sees every shot). Battery unplugged: nut to GND, screw to SW_N20.",
             parts=["drum_n20_pinion", "drum_n20", "drum_n20_cradle", "drum_n20_strap"]),
        dict(title="Head: code switches", stage=235,
             text="Press an M3 nut into each of the 3 pockets of the switch block and screw its foot to the spine "
                  "(2 x M3 x 12 from outside the spine). Screw each lever foot to the block (M3 x 12), nub on its band. "
                  "Thread an M3 x 6 into each lever boss, head toward the nut, so the head touches the nut near the top "
                  "of a bump (gap about 1.8 mm with the nub low). Battery unplugged: nuts to GND, screws to SW_DRUM_A "
                  "(nearest the drum gear), SW_DRUM_B, SW_DRUM_C.",
             parts=["drum_switch_block", "drum_sw_lever_0", "drum_sw_lever_1", "drum_sw_lever_2"]),
        dict(title="Link from the follower to the bell crank", stage=236,
             text="Cut the 8 mm steel rod to the length on the print sheet, deburr the ends and press a printed eye onto "
                  "each end, both eye holes parallel. Pass the rod through the window in the head front plate, screw "
                  "the top eye to the follower lug and the bottom eye to the bell crank's input arm (M3 x 16 and nyloc "
                  "nut each, snug: the eyes must swing). Turn the drum by hand from LAUNCH to REEL: the follower slides "
                  "14 mm, the crank turns and the collar moves from the LAUNCH dogs to the REEL dogs.",
             parts=["drum_link", "drum_link_eye_1", "drum_link_eye_2"]),
    ]
