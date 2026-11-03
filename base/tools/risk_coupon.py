"""Day-one risk coupons: small prints of the risky printed mechanisms of the base, made from the SAME geometry
functions and numbers the subsystems use, so a pass on the coupon means a pass on the part.

python -m tools.risk_coupon [PACKDIR]          (default base/pack/coupons, run from base/)

Writes into PACKDIR
  rc<k>_<part>.stl        one per printed part, print orientation, min corner at (0, 0, 0)
  risk_plate_<n>_<material>.3mf  the parts packed on the 256 x 256 bed, one plate per material (tools.pack helpers)
  risk_plate_<n>_<material>.png  top-down preview of that plate
and the base fit coupon (tools.coupon.write) into base/pack/fit_coupon (--fit DIR, '' to skip).
  COUPON_TEST.md / .pdf   the one-page test sheet: what to do, what pass looks like, what to change if it fails

Coupons (k):
  1 swing-arm direction splitter (launch_aim: sun, arm, idler as built; output spur on a pin)
  2 printed contact switch (cad.lib.switch.contact_switch)
  3 dog clutch: REEL face, thin collar, compliant fork, fork stop, short rail (drum.py functions)
  4 face slip clutch with the flexure spring pack (cad.lib.clutch.slip_clutch) on a keyed stub
  5 bearing seats: 608, F693ZZ, 693ZZ tile and the CSK20PP keyed seat (cad.lib.bearings, reel numbers)
  6 M3 heat-set, nut trap, clearance and tap holes (cad.lib.fasteners)
  7 20 mm key rib in a keyed bore tile (cad.lib.shafts), d12 spigot slice in a launch sleeve slice
  8 gear meshes m1.25, m1.5, m2 (cad.lib.gears) and m2.5 (launch_gears) on pins at their centre distances
  9 shear-pin torque limiter (drum.py coupler_hub / coupler slices with levers, drum.shear_pin)

Only this file owns the coupons; no subsystem file is edited. Every part passes tools.audit.static_part.

    from tools.risk_coupon import coupons
    cs = coupons()      # list of dicts: key, title, parts (cad.lib.Part), asm (list of {name: 4x4}), sheet text
"""
from __future__ import annotations

import argparse
import glob
import math
import os
import sys

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PARTS_DIR = os.path.join(BASE, "cad", "parts")
for _p in (BASE, PARTS_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np  # noqa: E402
from shapely.geometry import LineString, Point, Polygon  # noqa: E402
from shapely.ops import unary_union  # noqa: E402

from cad.lib import Part, prim, tf, fits, gears, bearings, shafts, fasteners, clutch, switch  # noqa: E402
from tools.coupon import label  # noqa: E402  (the fit coupon's stroke font)

SUB = "risk_coupon"
COL = {"PLA": "#e8e2c8", "PPA-CF": "#3a3f45"}
DEFAULT_OUT = os.path.join(BASE, "pack", "coupons")
GEAR_FACE = 6.0            # coupon gear face (the real faces are 8 to 14; the tooth geometry is the same)


# ---------------------------------------------------------------------------------------------- helpers
def bed(solid, R=None):
    """print_tf: rotate by R, then put the part on z = 0 centred in x y."""
    M = R if R is not None else tf.I4
    lo, hi = prim.bbox(prim.place(solid, M))
    return tf.mul(tf.T(-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]), M)


def stl_watertight(solid):
    tm = prim.to_trimesh(solid)
    tm.merge_vertices()
    return bool(tm.is_watertight)


def stl_clean(solid):
    """Snap vertices to a 0.1 um grid and collapse the resulting zero-length edges (manifold simplify). Some library
    solids (clutch.flexure_ring) carry vertex pairs 1e-6 mm apart: a valid manifold, but an STL reader merging
    vertices sees self-touching edges. The volume changes by under 0.001 mm3. Only used when the STL check fails."""
    if stl_watertight(solid):
        return solid
    return solid.warp_batch(lambda p: np.round(p, 4)).simplify(1e-6)


def min_box_rz(solid, R=None):
    """Rotation about z (deg, 0..85 in 5 deg steps) after R that gives the smallest bed footprint rectangle."""
    v, _ = prim.mesh_of(prim.place(solid, R if R is not None else tf.I4))
    xy = np.asarray(v)[:, :2]
    best = (None, 0.0)
    for a in np.arange(0.0, 90.0, 5.0):
        t = math.radians(a)
        p = xy @ np.array([[math.cos(t), math.sin(t)], [-math.sin(t), math.cos(t)]])
        area = float(np.ptp(p[:, 0]) * np.ptp(p[:, 1]))
        if best[0] is None or area < best[0] - 1e-6:
            best = (area, a)
    return best[1]


ORIENT_CANDIDATES = (tf.I4, tf.Rx(180), tf.Rx(90), tf.Rx(-90), tf.Ry(90), tf.Ry(-90))


def flat_ok(solid, R):
    """(passes the audit's bed check, overhang mm2) for print rotation R."""
    from tools.audit import print_check
    r = print_check(solid, bed(solid, R), (256.0, 256.0, 260.0))
    return not r["_fails"], r["overhang_mm2"]


def orient(solid, R):
    """R if the part sits on a real flat face that way (tools.audit.print_check), else the axis rotation that does,
    with the least overhang. Keeps a coupon printable when a subsystem changes a part's lowest face."""
    if flat_ok(solid, R)[0]:
        return R
    ok = [(flat_ok(solid, C)[1], k) for k, C in enumerate(ORIENT_CANDIDATES) if flat_ok(solid, C)[0]]
    return ORIENT_CANDIDATES[min(ok)[1]] if ok else R


def mk(name, solid, mat, notes, R=None, qty=1, keep_R=False):
    """keep_R: use R as given (an owner-mandated print orientation), no flat-face auto flip."""
    solid = stl_clean(solid)
    R0 = R if (keep_R and R is not None) else orient(solid, R if R is not None else tf.I4)
    R = tf.mul(tf.Rz(min_box_rz(solid, R0)), R0)          # smallest footprint for the packer
    return Part(name, solid, mat, qty=qty, print_tf=bed(solid, R), color=COL[mat], subsystem=SUB, notes=notes)


def hull_plate(circles, z0, z1):
    poly = unary_union([Point(x, y).buffer(r, 64) for x, y, r in circles]).convex_hull
    return prim.extrude(prim.shapely_cs(poly), z1 - z0, z=z0)


def P_default():
    from tools.params import load_params
    return load_params()


def pin_post(r, h, boss_r, boss_h, x=0.0, y=0.0):
    """Printed pin standing on a plate top (z = 0): shoulder boss, pin with a 0.5 top chamfer."""
    pin = prim.revolve_pts([(0, 0), (r, 0), (r, h - 0.5), (r - 0.5, h), (0, h)])
    if boss_h > 0:
        pin = pin + prim.cyl(boss_r, boss_h)
    return pin.translate((x, y, -0.01))


# ---------------------------------------------------------------------------------------------- 1 splitter
def coupon_1(P):
    import launch_aim as LA
    import launch_gears as LG
    import launch as LN
    n = LA.A(P)
    arm = LA.arm_solid(P)
    pad, shim, clip = LA.drag_pad_solid(P), LA.drag_shim_solid(P), LA.pin_clip_solid(P)
    dz = LA.Y_ARM0 - LA.Y_SUN                         # arm bottom above the sun bottom (as built)
    z_clip = LA._clip_y0(LA.Y_SUN) - LA.Y_ARM0        # clip low face in the arm frame (as built)
    low = min(prim.bbox(arm)[0][2], z_clip)           # lowest point of arm pin / clip in the arm frame
    zs = max(3.0, 1.0 - dz - low)                     # sun / idler / output bottom: pin end and clip 1 mm off the plate
    za = zs + dz                                      # arm bottom
    zc = za + (LA.Y_ARM1 - LA.Y_ARM0) + 3.0           # crank bottom
    ct = 6.0
    ztop = zc + ct
    zf0, zf1 = za + 0.1, zc - 0.5                     # stop finger in the arm plane
    rp, fit, face = LA.PIN_R, LA.FIT, LA.FACE_S
    ox, oy = n["cs_x"], -(n["cs_z"] - n["axis_z"])    # output axis in the alpha frame (as built)
    px, py = LA._stop_post_xy(P)                      # stop post centre (as built)
    phi = math.atan2(py, px)
    ux, uy = math.cos(phi), math.sin(phi)
    cx, cy = 62.0 * ux, 62.0 * uy                     # finger column, outside the idler swing
    holes = [(cx - 5 * ux, cy - 5 * uy), (cx + 5 * ux, cy + 5 * uy)]
    # base plate (PLA) with the centre pin, the output pin and the finger column
    col = prim.place(prim.box(-9, 9, -5, 5, 0, zf0), tf.mul(tf.T(cx, cy, 0), tf.Rz(math.degrees(phi))))
    plate = prim.union([hull_plate([(0, 0, 15), (ox, oy, 9), (cx, cy, 11)], -4.0, 0.0),
                        pin_post(rp, ztop - 0.2, rp, 0.0),
                        pin_post(rp, zs + face + 1.5, 7.0, zs, ox, oy), col.translate((0, 0, -0.01))])
    plate = plate - prim.union([fasteners.m3_tap(8.0).translate((x, y, zf0)) for x, y in holes])
    plate = plate + label("1", 0.5 * cx, 0.5 * cy, 0.0)
    # sleeve stub (PPA-CF): d20 with the as-built integral rib (launch.RIB_W, RIB_R) under the sun and the crank
    sl = prim.cyl(13.0, zs) + prim.cyl(10.0, ztop)
    sl = sl + prim.box(9.0, LN.RIB_R, -LN.RIB_W / 2, LN.RIB_W / 2, zs - 0.01, zs + face)
    sl = sl + prim.box(9.0, LN.RIB_R, -LN.RIB_W / 2, LN.RIB_W / 2, zc, ztop)
    sl = sl - prim.cyl(rp + fit / 2, ztop + 2, z=-1)
    sun, idl = LA.sun_solid(P), LA.idler_solid(P)
    out = LG.spur(LA.M_S, LA.Z_OUT, face, bore=2 * rp + fit)
    # crank (PLA): hub with the sun's keyway cut, bar, knob
    cr = prim.union([prim.cyl(15.0, ct), prim.box(0, 30, -5, 5, 0, ct), prim.cyl(5.0, ct, x=30),
                     prim.cyl(4.0, 14.0, x=30, z=ct - 0.01)])
    cr = cr - prim.cyl(10.0 + fit / 2, ct + 2, z=-1) - prim.box(9.0, 13.2, -3.2, 3.2, -1, ct + 1)
    # stop finger (PLA): rounded end r 4 at the as-built stop post, two M3 into the column
    fpoly = unary_union([LineString([(px, py), (cx, cy)]).buffer(4.0, 32),
                         Polygon([(cx + a * 9 * ux - b * 5 * uy, cy + a * 9 * uy + b * 5 * ux)
                                  for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))])])
    fg = prim.extrude(prim.shapely_cs(fpoly), zf1 - zf0, z=zf0)
    fg = fg - prim.union([fasteners.m3_clear(zf1 - zf0 + 1).translate((x, y, zf1)) for x, y in holes])
    parts = [
        mk("rc1_plate", plate, "PLA", "Splitter base: centre pin d8, output pin d8, stop finger column (2 x M3 tap)"),
        mk("rc1_sleeve", sl, "PPA-CF", "Sleeve stub d20 with the integral key rib (launch.RIB_W, RIB_R) under the sun and the crank"),
        mk("rc1_sun", sun, "PPA-CF", "launch_aim.sun_solid: sun m1.5 z30 as built"),
        mk("rc1_arm", arm, "PPA-CF", "launch_aim.arm_solid: swing arm as built (drag pad pocket, d8 pin, clip groove)",
           R=tf.Rx(180)),
        mk("rc1_drag_pad", pad, "PLA", "launch_aim.drag_pad_solid: printed flexure drag pad as built"),
        mk("rc1_drag_shim", shim, "PLA", "launch_aim.drag_shim_solid: nominal preload shim as built"),
        mk("rc1_idler_clip", clip, "PLA", "launch_aim.pin_clip_solid: printed C clip for the d8 idler pin"),
        mk("rc1_idler", idl, "PPA-CF", "launch_aim.idler_solid: idler m1.5 z18 as built"),
        mk("rc1_output", out, "PPA-CF", f"Output m1.5 z30 (launch_gears.spur, face {face:g}) on a d8 pin"),
        mk("rc1_crank", cr, "PPA-CF", "Hand crank, keyed on the sleeve rib like the sun"),
        mk("rc1_stop", fg, "PLA", "Stop finger: rounded end at the as-built stop post position",
           R=tf.Rz(-math.degrees(phi))),
    ]

    def state(alpha):
        Ra = tf.Rz(alpha)
        return {"rc1_plate": tf.I4, "rc1_sleeve": tf.I4, "rc1_sun": tf.T(0, 0, zs),
                "rc1_arm": tf.mul(Ra, tf.T(0, 0, za)), "rc1_idler": tf.mul(Ra, tf.T(n["cd_si"], 0, zs)),
                "rc1_output": tf.T(ox, oy, zs), "rc1_crank": tf.T(0, 0, zc), "rc1_stop": tf.I4,
                "rc1_drag_pad": tf.mul(Ra, tf.T(0, 0, za)), "rc1_drag_shim": tf.mul(Ra, tf.T(0, 0, za)),
                "rc1_idler_clip": tf.mul(Ra, tf.T(n["cd_si"], 0, za + z_clip))}
    return dict(
        key="1", title="Swing-arm direction splitter", parts=parts,
        asm=[state(n["alpha_e"]), state(n["alpha_d"])],
        mesh=[("rc1_sun", "rc1_idler"), ("rc1_idler", "rc1_output")],
        allow=[("rc1_drag_pad", "rc1_sleeve"), ("rc1_idler_clip", "rc1_arm")],   # pad preload bump, clip in its groove
        uses=(f"launch_aim.sun_solid, arm_solid, idler_solid (m{LA.M_S:g} z{LA.Z_SUN}/{LA.Z_IDL}/{LA.Z_OUT}, face "
              f"{face:g}, FIT {fit}), launch_gears.spur for the output, launch_aim.A: centre distances "
              f"{n['cd_si']:.2f} / {n['cd_io']:.2f}, output at ({ox:.1f}, {oy:.1f}), engaged arm "
              f"{n['alpha_e']:.1f} deg, stop {n['alpha_d']:.1f} deg (launch_aim._stop_post_xy), sleeve rib "
              f"launch.RIB_W {LN.RIB_W:g} / RIB_R {LN.RIB_R:g}; drag_pad_solid, drag_shim_solid (DRAG_INT "
              f"{LA.DRAG_INT:g}, shim up to +{LA.DRAG_SHIM_ADJ:g}), pin_clip_solid"),
        bought="2 x M3x16 (stop finger); everything else printed",
        do=("Push the drag pad root block and the shim into the arm hub pocket (finger in its recess). On the plate: "
            "sleeve on the centre pin, sun on its rib, arm on the sleeve above the sun with its pin down through the "
            "idler, printed C clip into the pin groove, output on its pin, crank on top, stop finger on the column "
            "(2 x M3x16). Turn the crank 10 turns each way."),
        ok=("Clockwise seen from above: the arm swings the idler into the output at once and the output turns, no "
            "skipped teeth. Anticlockwise: the arm swings out to the finger within about 20 deg of crank, the idler "
            "runs free and the output stays still. No jam at engagement. The pad drags the arm reliably both ways and "
            "the clip stays on."),
        fail=("launch_aim.py: FIT (pin and bore play), DRAG_INT and DRAG_SHIM_ADJ (drag: or print a thicker shim), "
              "clips.py for the C clip, A() alpha_d = alpha_e + 20 "
              "(swing out), _stop_post_xy; launch_gears.py center_distance extra / spur backlash (mesh)."),
    )


# ---------------------------------------------------------------------------------------------- 2 contact switch
SW_LEVER_LEN = 30.0        # drum GEOM sw_L (code switch lever length); no subsystem calls contact_switch yet


def coupon_2(P):
    from cad.parts import drum
    L = drum.G(P).get("sw_L", SW_LEVER_LEN)
    sw = switch.contact_switch(L)
    i = sw["info"]
    parts = [mk("rc2_switch_base", sw["base"], "PLA", "contact_switch base: pad, nut pillar, hard stop, wire channel"),
             mk("rc2_switch_lever", sw["lever"], "PLA", "contact_switch lever, printed on its side (Y up)",
                R=tf.Rx(90))]
    return dict(
        key="2", title="Printed contact switch", parts=parts, asm=[{"rc2_switch_base": tf.I4, "rc2_switch_lever": tf.I4}],
        mesh=[],
        uses=(f"cad.lib.switch.contact_switch(lever_len={L:g}) with the library defaults (travel {i['travel']:g}, "
              f"overtravel {i['overtravel']:g}, beam t {i['t']:g}): gap {i['gap']:.2f}, pretravel at the tip "
              f"{i['pretravel']:.2f}, close force {i['force_close_N']:.2f} N, strain {i['strain_max']:.4f}"),
        bought=f"{i['screw']}, M3 nut, {i['clamp_screw']}, 2 wires, multimeter",
        do=(f"Nut into the pillar trap (drop of glue), lever clamped with 2 M3 self tappers, contact screw head down "
            f"through the lever, set the head {i['gap']:.1f} mm above the nut (0.5 mm per turn), lock the jam nut. "
            f"Meter on beep between screw and nut. Press the lever tip 100 times."),
        ok=(f"Measure the tip travel with calipers (depth rod from the lever tip down to the base top, note the reading "
            f"at rest, press until it beeps, read again): beep within {i['pretravel'] - 0.35:.1f} to "
            f"{i['pretravel'] + 0.25:.1f} mm of tip travel (design {i['pretravel']:.2f}), silent when let go, tip stops "
            "on the hard stop without cracking, lever back flat, same after 100 presses."),
        fail=("cad/lib/switch.py contact_switch args travel, overtravel, t, b; STRAIN_LIMIT, NUT_PRESS (loose nut), "
              "SCREW_LENGTHS."),
    )


# ---------------------------------------------------------------------------------------------- 3 dog clutch
def coupon_3(P):
    from cad.parts import drum
    g, c = drum.G(P), drum.C(P)
    h, st = c["dog_h"], c["stroke"]
    F = drum.fork_planes(g)
    fk, dp = drum.fork(g, c), drum.dog_post(g, c)
    flo, fhi = prim.bbox(fk)
    S1 = g["dog_face_reel"] - 4.5                         # REEL face bottom (L frame kept, s = z)
    SP = min(S1, flo[2] - st - 1.0)                       # plate top: below the fork's lowest point when engaged
    S0 = SP - 5.0
    jr = g["rsl_journal"][2] / 2
    rx, ry = g["rail_xy"]
    s0 = g["collar_s"][0]
    b0, b1 = s0 + h, g["groove"][1] + 2.0                 # thin collar: dogs at b0, body to past the groove
    hex_top = b1 + st + 3.0
    rail_top = max(fhi[2] + st + 3.0, g["dogpost_s"][1] + 1.0)   # past the fork travel and the dog post clamp
    base = prim.union([hull_plate([(0, 0, 20), (rx, ry, 9)], S0, SP),
                       drum.cyl(jr + 3.0, SP - 0.01, S1),            # shoulder the REEL face rests on
                       drum.cyl(jr, S1 - 0.01, g["dog_face_reel"]),
                       drum.hexprism(g["hex_af"], g["dog_face_reel"] - 0.01, hex_top)])
    base = base - drum.cyl((g["rail_d"] + fits.HOLE_PRESS) / 2, S0 - 1, SP + 1, rx, ry)
    rr = math.hypot(rx, ry)
    lx, ly = rx / rr * 49.0, ry / rr * 49.0              # label outside the face handle sweep (r 38)
    base = base + label("3", lx, ly, SP)
    # REEL face: the REEL gear's dog face (ring r dog_r_out + 1, bore journal + SHAFT_IN_BORE) with a turning handle
    hand_top = min(S1 + 3.0, g["dogpost_s"][0] - 0.5)
    fb = prim.union([drum.ring(c["dog_r_out"] + 1.0, (2 * jr + fits.SHAFT_IN_BORE) / 2, S1, g["dog_face_reel"]),
                     prim.box(-38, -20, -6, 6, S1, hand_top) - drum.cyl(jr + 1, S1 - 1, S1 + 4),
                     drum.dog_teeth(c, g["dog_face_reel"], +1, 45.0)])
    co = drum.cyl(g["collar_od"] / 2, b0, b1)
    co = co - drum.ring(g["collar_od"] / 2 + 1, g["groove_r"], g["groove"][0], g["groove"][1])
    co = prim.union([co, drum.dog_teeth(c, b0, -1, 0.0)])
    co = co - drum.hexprism(g["hex_af"] + fits.SHAFT_IN_BORE, s0 - 1, b1 + 1)
    rl = drum.rail(dict(g, rail_s=(S0, rail_top)))
    parts = [
        mk("rc3_base", base, "PLA", "Base: d30 journal (REEL sleeve) and hex 22 AF (main shaft), rail hole (press)"),
        mk("rc3_reel_face", fb, "PPA-CF", "REEL gear dog face (drum.dog_teeth, phase 45) with a handle"),
        mk("rc3_collar", co, "PPA-CF", "Collar lower half: drum.dog_teeth facing down, fork groove, hex bore",
           R=tf.Rx(180)),
        mk("rc3_fork", fk, "PLA", "drum.fork as built: compliant yoke, rail sleeve, follower pin, M3 nut in the tab"),
        mk("rc3_dog_post", dp, "PLA", "drum.dog_post as built: SW_DOG_L / SW_DOG_R screw bosses, clamps on the rail",
           R=tf.Rx(180)),
        mk("rc3_rail", rl, "PPA-CF", f"drum.rail profile d10 with the print flat, cut to {rail_top - S0:.0f} long",
           R=tf.Ry(90)),
    ]

    def state(e):
        d = tf.T(0, 0, -e)
        return {"rc3_base": tf.I4, "rc3_reel_face": tf.I4, "rc3_collar": d, "rc3_fork": d, "rc3_dog_post": tf.I4,
                "rc3_rail": tf.I4}
    return dict(
        key="3", title="Dog clutch and compliant fork", parts=parts, asm=[state(0.0), state(st)], mesh=[],
        allow=[("rc3_base", "rc3_rail"), ("rc3_dog_post", "rc3_rail")],
        uses=(f"drum.dog_teeth (dog_n {c['dog_n']}, dog_h {h:g}, dog_deg {c['dog_deg']:g}, backcut "
              f"{c['dog_backcut']:g}, roof {c['dog_roof']:g}, r {c['dog_r_in']:g}..{c['dog_r_out']:g}), drum.fork, "
              f"drum.dog_post (dog_touch {g['dog_touch']:g}), drum.rail, drum.hexprism, GEOM journal d{2 * jr:g}, "
              f"groove {g['groove']}, stroke {st:g}, neutral gap {c['dog_gap']:g}"),
        bought="M3 nut (yoke tab), 2 x M3x12 (SW_DOG screws in the post), 1 x M3 (post clamp), multimeter",
        do=("Press the rail into the base. REEL face on the journal (dogs up), collar on the hex (dogs down), fork pads "
            "in the collar groove and its sleeve on the rail, dog post clamped on the rail below the fork, SW_DOG screws "
            f"in the post set {g['dog_touch']:g} mm short of the tab nut at full engagement. Push the fork {st:g} mm down "
            "with the dogs tip to tip, then turn the REEL face until they drop in. Turn the face hard both ways, then "
            "pull the fork back. Meter on beep between the lower screw and the tab nut."),
        ok=("Yoke flexes when tip to tip (no jam, roofs slide off), dogs drop in within a quarter turn, under hand torque "
            "the dogs pull in, never walk out, a light pull on the fork releases them, the yoke springs back flat; the "
            "meter beeps only with the dogs fully in, never tip to tip."),
        fail=("_drum_kin.py DEFAULTS dog_backcut, dog_roof, dog_deg, dog_gap, fork_k; drum.py GEOM fork_t, yoke_r, "
              "groove, dog_touch, dogpost_s; fits.py SHAFT_IN_BORE (collar on the hex, face on the journal), HOLE_SLIDE "
              "(fork on the rail)."),
    )


# ---------------------------------------------------------------------------------------------- 4 slip clutch
def key_stub_len(sc):
    return float(math.ceil(sc["info"]["height"]) + 12.0)


def slip_bands(info, r_lug, band=0.25):
    """Hanging weight pass bands (grams at r_lug) per quarter turn of the nuts, from slip_clutch's torque table:
    the design weight -band must hold, +band must slip."""
    out = []
    names = {0.125: "1/4 turn", 0.25: "1/2 turn", 0.375: "3/4 turn", 0.5: "1 turn"}
    for d, F, T, s in info["table"]:
        if d <= 0:
            continue
        grams = T / (r_lug / 1000.0) / 9.81 * 1000.0
        lo, hi = 5 * math.floor(grams * (1 - band) / 5), 5 * math.ceil(grams * (1 + band) / 5)
        nm = names.get(round(d, 3), f"{d:.2f} mm")
        out.append(f"{nm}: {lo} g holds, {hi} g slips (design {grams:.0f} g, {T:.3f} N m)")
    return out


def coupon_4(P, sc=None):
    sc = sc or clutch.slip_clutch()
    info = sc["info"]
    names = ("hub_plate", "friction_disc", "pressure_plate", "flexure", "adjuster_plate")
    parts = [mk(f"rc4_{nm}", sc["parts"][nm], "PLA", f"clutch.slip_clutch {nm} (library defaults)") for nm in names]
    asm = {f"rc4_{nm}": sc["place"][nm] for nm in names}
    asm["rc7_key_stub"] = tf.T(0, 0, -12.0)
    r_lug = info["r_out"] + 1.5
    tab = slip_bands(info, r_lug)
    return dict(
        key="4", title="Face slip clutch with the flexure spring", parts=parts, asm=[asm], mesh=[],
        uses=(f"cad.lib.clutch.slip_clutch() defaults (reel does not use it: its clutch is the spool face plus the "
              f"PETG rocker): shaft d{info['shaft_d']:g} key {info['key_w']:g}, friction r {info['r_in']:g}.."
              f"{info['r_out']:g}, mu {info['mu']}, flexure t {info['flex_geom']['t']:g} gap "
              f"{info['flex_geom']['gap']:g}, {info['torque_per_mm']:.3f} N m per mm, travel {info['travel']:.2f}; on the "
              f"shared keyed stub of coupon 7"),
        bought=f"3 x M3x{info['screw_len']} SHCS, 3 x M3 nut, string, coins in a small bag, kitchen scale",
        table=tab, r_lug=r_lug,
        do=("Stack on the key stub: hub plate (counterbores down), friction disc, pressure plate (pads up), flexure (beam "
            "down, tabs on the screws), adjuster plate (traps up); 3 screws from below, nuts on top. Run the nuts down "
            "until the flexure just touches (zero), then turn all 3 nuts the same, 1/4 turn (0.125 mm) at a time. Clamp the "
            f"stub level in a vice, tie a string to a disc lug (r {r_lug:.1f} mm), lay it over the rim and hang a bag of "
            "coins weighed on a kitchen scale. Add or take coins to find the hold and slip weights at each setting."),
        ok=("At each setting the lower weight must not pull the disc round and the higher one must (smooth steady slip, "
            "no judder); same result 3 times; the flexure never bottoms before 1 turn. Bands: " + "; ".join(tab) + "."),
        fail=("cad/lib/clutch.py: slip_clutch args flex_t, flex_gap, flex_ro, r_in, r_out, n_grooves; MU_PLA_PLA, E_PLA, "
              "SIGMA_FLEX_PLA_SUSTAINED (creep: if the torque falls overnight)."),
    )


# ---------------------------------------------------------------------------------------------- 5 bearing seats
def coupon_5(P):
    from tools.coupon import tile
    s608 = bearings.seat_608(bottom="lip", lip_d=14.0)               # reel spool seat
    t = prim.place(tile(60, 28, bearings.B608_W + 2.0,
                        [(s608, -15.5, 0), (bearings.seat_f693(), 3.5, 0), (bearings.seat_693(), 15.5, 0)],
                        [("5", 25.5, 0)]), tf.I4)
    rb = bearings.CSK20_OD / 2 + bearings.BRG_RIB_BORE / 2 + 4.0      # reel capstan block wall
    hb = bearings.CSK20_W + 2.0
    ck = prim.cyl(rb, hb) - prim.place(bearings.seat_csk20(key_deg=90.0), tf.T(0, 0, hb))
    parts = [mk("rc5_seats_608_693", t, "PLA", "608 (reel spool seat), F693ZZ and 693ZZ crush-rib seats"),
             mk("rc5_csk20_seat", ck, "PLA", "CSK20PP seat with the printed key (reel capstan block, key_deg 90)")]
    return dict(
        key="5", title="Bearing seats", parts=parts, asm=[], mesh=[],
        uses=(f"cad.lib.bearings.seat_608(bottom lip, lip_d 14) as reel, seat_f693(), seat_693(), seat_csk20(key_deg 90) "
              f"as the reel capstan block (wall 4); fits BRG_RIB_INTERF {fits.BRG_RIB_INTERF}, BRG_RIB_BORE "
              f"{fits.BRG_RIB_BORE}, CSK20_KEY_RCLEAR {bearings.CSK20_KEY_RCLEAR}"),
        bought="608ZZ, F693ZZ, 693ZZ, CSK20PP",
        do=("Press each bearing in square with a flat block or a vice (tile left to right: 608, F693ZZ, 693ZZ); CSK20PP "
            "into the round boss with its outer keyway on the printed key."),
        ok=("Firm press, seated flat on the lip, stays in when tapped upside down, spins as freely as before; the CSK20PP "
            "key slides in without force, it still freewheels one way and locks the other."),
        fail=("fits.py BRG_RIB_INTERF, BRG_RIB_BORE (all seats); bearings.py CSK20_KEY_RCLEAR, CSK20_OKEY_W (key), "
              "KEY_CLEAR in fits.py. If the 608 is off, print tile A of the fit coupon (base/pack/fit_coupon, COUPON.md)."),
    )


# ---------------------------------------------------------------------------------------------- 6 fasteners
def coupon_6(P):
    from tools.coupon import tile
    t = tile(54, 18, 8.0, [(fasteners.m3_insert(), -12, 1.5), (fasteners.m3_nut_trap(bolt_len=9.0), 0, 1.5),
                           (fasteners.m3_clear(9.0), 11, 1.5), (fasteners.m3_tap(9.0), 20.5, 1.5)], [("6", -22.5, 0)])
    parts = [mk("rc6_m3_tile", t, "PLA", "M3 heat-set hole, nut trap, clearance and tap holes (fits.py values)")]
    return dict(
        key="6", title="M3 holes", parts=parts, asm=[], mesh=[],
        uses=(f"cad.lib.fasteners.m3_insert (d {fits.M3_INSERT_D}), m3_nut_trap (AF {fits.M3_NUT_AF:.1f}), m3_clear "
              f"(d {fits.M3_CLEAR_D}), m3_tap (d {fits.M3_TAP_D})"),
        bought="M3x4x5 heat-set insert, M3 nut, 2 x M3x10",
        do="Left to right: heat-set the insert (220 C), press the nut in, push a screw through, self-tap a screw in.",
        ok=("Insert flush and does not spin when the screw is tightened; nut presses in and holds; screw drops through "
            "the clearance hole; the tap hole cuts a thread and holds a hard pull without splitting."),
        fail="fits.py M3_INSERT_D, M3_NUT_AF, M3_CLEAR_D, M3_TAP_D (tiles C, D, E of the fit coupon in base/pack/fit_coupon bracket them).",
    )


# ---------------------------------------------------------------------------------------------- 7 key and spigot
def coupon_7(P, sc=None):
    from tools.coupon import tile
    from cad.parts import drum, reel as RL
    import launch as LN
    sc = sc or clutch.slip_clutch()
    L = key_stub_len(sc)
    stub = shafts.keyed_shaft(L, d=RL.C_D, key_w=shafts.KEY20_W, key_h=shafts.KEY20_H, chamfer=0.8)
    kb = tile(36, 36, 8.0, [(shafts.keyed_bore(8.0, d=RL.C_D, key_w=shafts.KEY20_W), -1.5, 3)], [("7", 0, -15.5)])
    g = drum.G(P)
    h1 = g["hex_s"][1]
    sp = drum.main_shaft(g) ^ prim.box(-20, 20, -20, 20, h1 - 8.0, h1 + 20.0)
    sv = LN.sleeve_solid(P) ^ prim.cyl(16.0, 16.0)
    parts = [mk("rc7_key_stub", stub, "PPA-CF", f"shafts.keyed_shaft d{RL.C_D:g} key {shafts.KEY20_W:g} (reel capstan "
                f"tube numbers), {L:g} long, also the coupon 4 clutch shaft"),
             mk("rc7_key_bore", kb, "PPA-CF", "shafts.keyed_bore d20 key 6, 8 long (keyed hubs on the sleeve rib are PPA-CF)"),
             mk("rc7_spigot", sp, "PPA-CF", "drum.main_shaft slice: hex 8 long + d12 spigot 20 long, on a hex flat",
                R=tf.mul(tf.Rx(90), tf.Ry(90))),
             mk("rc7_bushing", sv, "PPA-CF", f"launch.sleeve_solid slice 16 long: spigot bore d{LN.SPIGOT_BORE:g}")]
    return dict(
        key="7", title="Key rib and spigot", parts=parts, asm=[], mesh=[],
        uses=(f"cad.lib.shafts.keyed_shaft(d {RL.C_D:g}, key_w {shafts.KEY20_W:g}, key_h {shafts.KEY20_H:.1f}) and "
              f"keyed_bore (clear {fits.SHAFT_IN_BORE}); drum.main_shaft (spig_d {g['spig_d']:g}); launch.sleeve_solid "
              f"(SPIGOT_BORE {LN.SPIGOT_BORE:g})"),
        bought="CSK20PP from coupon 5 (inner keyway)",
        do=("Slide the key stub into the keyed tile and into the CSK20PP inner race (the real interface). Push the spigot "
            "into the sleeve slice and turn it."),
        ok=("Stub slides in by hand, key in the keyway, no visible rock. Play: clamp the tile, tape a ruler to the stub "
            "end, mark the ruler 60 mm from the axis, rock the stub both ways: the mark moves under 1 mm (about 1 deg). "
            "The spigot turns freely; rocked at its free end it moves under 0.5 mm."),
        fail=(f"fits.py KEY_CLEAR, SHAFT_IN_BORE; shafts.py KEY20_H, SHAFT_PRINT_UNDERSIZE; launch.py SPIGOT_BORE "
              f"(now {LN.SPIGOT_BORE:g}); drum.py GEOM spig_d."),
    )


# ---------------------------------------------------------------------------------------------- 8 gear meshes
def sector(solid, r_tip, center_deg, span_deg, hub_r, bore_d, face, to_deg):
    """Keep the teeth within span_deg about center_deg plus a hub, cut the bore, turn center_deg to to_deg."""
    a = np.radians(np.linspace(center_deg - span_deg / 2, center_deg + span_deg / 2, 48))
    R = r_tip + 2.0
    poly = unary_union([Polygon([(0, 0)] + [(R * math.cos(t), R * math.sin(t)) for t in a]),
                        Point(0, 0).buffer(hub_r, 64)])
    s = (solid ^ prim.extrude(prim.shapely_cs(poly), face + 2, z=-1)) - prim.cyl(bore_d / 2, face + 2, z=-1)
    return prim.place(s, tf.Rz(to_deg - center_deg))


def gear_plate(tag, cd, pins, face):
    """Plate z -3..0 (PPA-CF: the pins are the gear shafts, PPA-CF in the subsystems) with two pins (d, x) at 0 and cd, label tag."""
    s = [hull_plate([(0, 0, 9), (cd, 0, 9)], -3.0, 0.0)]
    for d, x in pins:
        s.append(pin_post(d / 2, face + 2.0, d / 2 + 2.5, 1.0, x, 0.0))
    return prim.union(s) + label(tag, cd / 2, 0.0, 0.0)


def coupon_8(P):
    from cad.parts import drum, reel as RL
    import launch as LN
    import launch_gears as LG
    c = drum.C(P)
    F = GEAR_FACE
    parts, asm, uses = [], {}, []
    b4, b5 = 4.0 + fits.SHAFT_IN_BORE, 5.0 + fits.SHAFT_IN_BORE

    def add(name, solid, mat, notes, M):
        parts.append(mk(name, solid, mat, notes))
        asm[name] = M

    # m1.25: drum wrap pinion z10 x0.4 (PPA-CF) on the drum segment gear z60 (PLA)
    m, zp, xp, zs = c["seg_m"], c["pin_z"], c["pin_x"], c["seg_z"]
    cd = gears.center_distance(m, zs, zp, 0.0, xp)
    seg = sector(gears.spur_gear(m, zs, F), m * zs / 2 + m, 0.0, 66.0, 8.0, b5, F, 0.0)
    ts = drum.tip_short_for(m, zp, xp)
    pin = gears.spur_gear(m, zp, F, shift=xp, bore=b4, tip_short=ts)
    y0 = 0.0
    add("rc8_m125_plate", gear_plate("1.25", cd, [(5.0, 0.0), (4.0, cd)], F), "PPA-CF", f"Pins at {cd:.2f}",
        tf.T(0, y0, 0))
    add("rc8_m125_seg", seg, "PLA", f"drum segment gear m{m:g} z{zs} sector", tf.T(0, y0, 1.0))
    add("rc8_m125_pinion", pin, "PPA-CF", f"drum wrap pinion m{m:g} z{zp} x{xp:g} tip_short {ts:g}", tf.T(cd, y0, 1.0))
    uses.append(f"m{m:g}: gears.spur_gear z{zs} sector / z{zp} shift {xp:g} tip_short {ts:g} (drum.tip_short_for), cd {cd:.2f} (drum.a_seg)")
    # m1.5: drum idler output z42 (PPA-CF) on the reel cup ring gear z24 (PPA-CF)
    m, z1, z2 = c["idl_m"], c["idl_z"], RL.Z_CUPG
    cd = gears.center_distance(m, z1, z2)
    y0 = 120.0
    add("rc8_m15_plate", gear_plate("1.5", cd, [(5.0, 0.0), (5.0, cd)], F), "PPA-CF", f"Pins at {cd:.2f}", tf.T(0, y0, 0))
    add("rc8_m15_z42", sector(gears.spur_gear(m, z1, F), m * z1 / 2 + m, 0.0, 60.0, 8.0, b5, F, 0.0), "PPA-CF",
        f"drum wrap idler output m{m:g} z{z1} sector", tf.T(0, y0, 1.0))
    add("rc8_m15_z24", gears.spur_gear(m, z2, F, bore=b5), "PPA-CF", f"reel cup ring gear m{m:g} z{z2}",
        tf.T(cd, y0, 1.0))
    uses.append(f"m{m:g}: gears.spur_gear z{z1} sector / z{z2}, cd {cd:.2f}")
    # m2: drum REEL output gear z50 (sector) on the reel capstan gear z22 (both PPA-CF)
    g = drum.G(P)
    m, z1, z2 = g["out_m"], g["out_z"], RL.Z_CAPG
    if RL.M_CAP != m or RL.Z_MAIN != z1:
        raise KeyError(f"drum REEL output m{m:g} z{z1} and reel M_CAP {RL.M_CAP:g} / Z_MAIN {RL.Z_MAIN} disagree")
    cd = gears.center_distance(m, z1, z2)
    y0 = 240.0
    add("rc8_m2_plate", gear_plate("2", cd, [(5.0, 0.0), (5.0, cd)], F), "PPA-CF", f"Pins at {cd:.2f}", tf.T(0, y0, 0))
    add("rc8_m2_z50", sector(gears.spur_gear(m, z1, F), m * z1 / 2 + m, 0.0, 46.0, 9.0, b5, F, 0.0), "PPA-CF",
        f"drum REEL output m{m:g} z{z1} sector", tf.T(0, y0, 1.0))
    add("rc8_m2_z22", gears.spur_gear(m, z2, F, bore=b5), "PPA-CF", f"reel capstan gear m{m:g} z{z2}",
        tf.T(cd, y0, 1.0))
    uses.append(f"m{m:g}: gears.spur_gear z{z1} sector (drum GEOM out_m, out_z) / z{z2} (reel Z_CAPG), cd {cd:.2f}")
    # m2.5: launch big gear (sector_gear_2d: tall tooth, relieved last tooth) on the drive gear z26 (launch_gears)
    n = LN.L(P)
    m, zb, zd, cd = n["m"], n["z_big"], n["z_drive"], n["cd"]
    pitch = 360.0 / zb
    k0, k1 = n["n_present"] - 6, n["n_present"]            # last 6 present teeth and the first gap
    big = prim.extrude(LG.sector_gear_2d(m, zb, n["n_present"], first_tooth_angle=n["fta"]), F)
    big = sector(big, m * zb / 2 + 1.3 * m, n["fta"] + 0.5 * (k0 + k1) * pitch, (k1 - k0 + 1) * pitch + 4.0, 9.0,
                 b5, F, 0.0)
    drv = sector(LG.spur(m, zd, F), m * zd / 2 + m, 180.0, 95.0, 9.0, b5, F, 180.0)
    y0 = 360.0
    add("rc8_m25_plate", gear_plate("2.5", cd, [(5.0, 0.0), (5.0, cd)], F), "PPA-CF", f"Pins at {cd:.2f}", tf.T(0, y0, 0))
    add("rc8_m25_big", big, "PPA-CF", f"launch big gear m{m:g} z{zb} sector: teeth {k0}..{k1 - 1} (tall, relieved)",
        tf.T(0, y0, 1.0))
    add("rc8_m25_drive", drv, "PPA-CF", f"launch drive gear m{m:g} z{zd} sector", tf.T(cd, y0, 1.0))
    uses.append(f"m{m:g}: launch_gears.sector_gear_2d z{zb} (n_present {n['n_present']}, last 6 + gap) / "
                f"launch_gears.spur z{zd}, cd {cd:.2f} (launch L)")
    mesh = [("rc8_m125_seg", "rc8_m125_pinion"), ("rc8_m15_z42", "rc8_m15_z24"), ("rc8_m2_z50", "rc8_m2_z22"),
            ("rc8_m25_big", "rc8_m25_drive")]
    return dict(
        key="8", title="Gear meshes m1.25, m1.5, m2, m2.5", parts=parts, asm=[asm], mesh=mesh,
        uses="; ".join(uses) + f"; coupon face {F:g}, fits GEAR_BACKLASH {fits.GEAR_BACKLASH}, GEAR_CENTER_EXTRA "
                               f"{fits.GEAR_CENTER_EXTRA}",
        bought="none",
        do="Each pair on its plate (label = module), flat faces down; turn or rock through every tooth.",
        ok=("Smooth, no tight spot, a little play when rocked; m2.5: the tall tooth and the cut-down last tooth pass "
            "without catching, then the mesh lets go at the gap."),
        fail=("fits.py GEAR_BACKLASH, GEAR_CENTER_EXTRA (m1.25, m1.5, m2); launch_gears.py center_distance extra, "
              "spur backlash, sector_gear_2d tall_factor (m2.5); _drum_kin.py pin_x (m1.25 pinion)."),
    )


# ---------------------------------------------------------------------------------------------- 9 shear pin
SHEAR_RANGE_NM = (3.5, 6.0)   # drum_NOTES: 30 MPa assumed, so the limit could be 3.5 to 6 N m
PACKINGS = ((5.0, 5.0), (4.0, 5.0), (3.0, 4.0), (3.0, 3.0))   # (spacing, bed margin) mm tried in turn
LEVER_R = 60.0             # string hole radius on both levers


def coupon_9(P):
    from cad.parts import drum
    g = drum.G(P)
    # slices around the pin plane, all from drum's current numbers: outer coupler slice s_lo..s_mid, the hub slice
    # runs on above it and carries its lever there (s_mid + 0.2 .. s_top)
    h0, h1 = g["hub_s"]
    ss, sd = g["shear_s"], g["shear_d"]
    s_lo = max(h0, ss - sd / 2 - 3.0)
    s_mid = min(h1 - 4.0, ss + sd / 2 + 3.0)
    s_top = min(h1, s_mid + 6.0)
    s_hi = s_top
    hub = drum.coupler_hub(g, P) ^ drum.cyl(30.0, s_lo, s_top)
    lev = prim.box(g["hub_od"] / 2 - 2.0, LEVER_R + 6, -8, 8, s_mid + 0.2, s_top) - \
        drum.cyl(2.2, s_mid, s_top + 1, LEVER_R, 0.0)
    hub = hub + lev
    outer = drum.coupler(g, P) ^ drum.cyl(30.0, s_lo, s_mid)
    lev2 = prim.box(-(LEVER_R + 6), -(g["cpl_od"] / 2 - 1.0), -8, 8, s_lo, s_mid) - \
        drum.cyl(2.2, s_lo - 1, s_mid + 1, -LEVER_R, 0.0)
    outer = outer + lev2
    pin = drum.shear_pin(g)
    r = g["hub_od"] / 2
    sig = 30.0
    t_est = sig * math.pi * (g["shear_d"] / 2) ** 2 * r / 1000.0
    kg = t_est / (LEVER_R / 1000.0) / 9.81
    kg_range = tuple(t / (LEVER_R / 1000.0) / 9.81 for t in SHEAR_RANGE_NM)
    parts = [mk("rc9_hub_lever", hub, "PPA-CF", "drum.coupler_hub slice (shear pin hole) "
                f"with a lever, string hole at r {LEVER_R:g}", R=tf.Rx(180)),
             mk("rc9_outer_lever", outer, "PPA-CF", "drum.coupler slice (shear pin hole) "
                f"with a lever, string hole at r {LEVER_R:g}"),
             mk("rc9_shear_pin", pin, "PLA", "drum.shear_pin as built (d4 PLA), printed LYING on its 0.4 mm flat as drum "
                "prints it (D44): the shear plane runs across the layers", R=tf.Ry(180), qty=3, keep_R=True)]
    return dict(
        key="9", title="Shear-pin torque limiter", parts=parts,
        asm=[{"rc9_hub_lever": tf.I4, "rc9_outer_lever": tf.I4, "rc9_shear_pin": tf.I4}], mesh=[],
        allow=[("rc9_hub_lever", "rc9_shear_pin"), ("rc9_outer_lever", "rc9_shear_pin")],
        uses=(f"drum.coupler_hub and drum.coupler (sliced s {s_lo:g}..{s_hi:g}), drum.shear_pin: d{g['shear_d']:g} PLA "
              f"in single shear at r {r:g} (shear_s {g['shear_s']:g}); {t_est:.1f} N m at {sig:g} MPa"),
        bought="string or a strap, luggage scale",
        t_est=t_est, kg=kg, t_range=SHEAR_RANGE_NM, kg_range=kg_range,
        do=(f"Print the pins lying down on their flat, as exported (D44: shear plane across the layers; never stand them "
            f"up). Hub slice into the outer slice, pin through both holes (head outside). Clamp the hub lever, hang the "
            f"luggage scale on the outer lever hole (r {LEVER_R:g} mm) at right angles and pull slowly until the pin "
            f"shears. Break all 3 pins. Torque = kg x 9.81 x {LEVER_R / 1000:g} N m."),
        ok=(f"Each pin shears cleanly between {kg_range[0]:.1f} and {kg_range[1]:.1f} kg on the scale "
            f"({SHEAR_RANGE_NM[0]:g} to {SHEAR_RANGE_NM[1]:g} N m, drum_NOTES; design {t_est:.1f} N m = {kg:.1f} kg), the "
            "3 pins within 20 percent of each other, both slices undamaged (holes still round); a fresh pin goes in by "
            "hand."),
        fail=("drum.py GEOM shear_d (torque goes with d squared), hub_od (radius), shear_s; fits for the hole: "
              "drum.coupler shear hole (shear_d + 0.1) and shear_pin (shear_d - 0.04). Spec D8 / D26 limit 5 N m."),
    )


# ---------------------------------------------------------------------------------------------- 10 magnet cap
def coupon_10(P):
    """Not a risk test: drum's real AS5600 magnet cap, printed early for the motor bench test."""
    from cad.parts import drum
    g = drum.G(P)
    cap = drum.magnet_holder(g)
    parts = [mk("rc10_magnet_cap", cap, "PLA", "drum.magnet_holder as built: AS5600 magnet cap on the d12 spigot end",
                R=tf.Rx(180))]
    return dict(
        key="10", title=TITLES["10"], parts=parts, asm=[], mesh=[],
        uses=(f"drum.magnet_holder (spig_d {g['spig_d']:g} with its flat, cap_s {g['cap_s']}, magnet pocket d6.2 x 2.6), "
              f"printed as drum prints it"),
        bought="6 x 2.5 mm diametric magnet (comes with the AS5600 board), M3x6, superglue",
        do=("Glue the magnet into the end pocket (flush, any way round), push the cap on the shaft end (flat to the "
            "flat), M3x6 onto the flat. For the bench: AS5600 chip on the shaft axis, 1 to 2 mm from the magnet face."),
        ok=("Cap does not wobble or slip on the shaft; the AS5600 angle follows the shaft smoothly through full turns "
            "(no jumps), magnet status OK (not too weak, not too strong) at 1 to 2 mm."),
        fail=("drum.py GEOM cap_s, spig_d; magnet_holder pocket d6.2 x 2.6 (magnet loose or tight); fits.py HOLE_SLIDE "
              "(cap on the spigot)."),
    )


# ---------------------------------------------------------------------------------------------- all
TITLES = {"1": "Swing-arm direction splitter", "2": "Printed contact switch", "3": "Dog clutch and compliant fork",
          "4": "Face slip clutch with the flexure spring", "5": "Bearing seats", "6": "M3 holes",
          "7": "Key rib and spigot", "8": "Gear meshes m1.25, m1.5, m2, m2.5", "9": "Shear-pin torque limiter",
          "10": "AS5600 magnet cap (bench test)"}
# a renamed or removed function, constant or module in a subsystem file skips only the coupon that uses it
SOURCE_ERRORS = (AttributeError, ImportError, KeyError, NameError, TypeError)


def coupons(P=None, skipped=None, log=print):
    """Build every coupon. A coupon whose subsystem source (launch_aim, launch_gears, launch, drum, reel) no longer
    has a function or number it needs is left out with a printed message and, when `skipped` (a list) is given,
    a dict(key, title, reason) appended to it for the sheet."""
    P = P or P_default()
    sc = clutch.slip_clutch()
    fns = [("1", lambda: coupon_1(P)), ("2", lambda: coupon_2(P)), ("3", lambda: coupon_3(P)),
           ("4", lambda: coupon_4(P, sc)), ("5", lambda: coupon_5(P)), ("6", lambda: coupon_6(P)),
           ("7", lambda: coupon_7(P, sc)), ("8", lambda: coupon_8(P)), ("9", lambda: coupon_9(P)),
           ("10", lambda: coupon_10(P))]
    out = []
    for key, fn in fns:
        try:
            out.append(fn())
        except SOURCE_ERRORS as e:
            reason = f"{type(e).__name__}: {e}"
            log(f"risk coupon {key} ({TITLES[key]}) SKIPPED: its subsystem source changed ({reason}); update "
                f"coupon_{key} in tools/risk_coupon.py")
            if skipped is not None:
                skipped.append(dict(key=key, title=TITLES[key], reason=reason))
    return out


def all_parts(cs):
    return [p for c in cs for p in c["parts"]]


def interferences(cs, tol=0.05):
    """Overlap volume of every part pair (except meshing gears and listed press fits) in every assembly state of each coupon: [(coupon, a, b, vol)]."""
    out = []
    for c in cs:
        by = {p.name: p for p in all_parts(cs)}
        if any(nm not in by for st in c["asm"] for nm in st):
            continue                                          # a part from a skipped coupon (4 uses 7's stub)
        skip = {frozenset(x) for x in c["mesh"] + c.get("allow", [])}
        for st in c["asm"]:
            names = sorted(st)
            posed = {nm: prim.place(by[nm].solid, st[nm]) for nm in names}
            for i, a in enumerate(names):
                for b in names[i + 1:]:
                    if frozenset((a, b)) in skip:
                        continue
                    v = (posed[a] ^ posed[b]).volume()
                    if v > tol:
                        out.append((c["key"], a, b, round(v, 3)))
    return out


# ---------------------------------------------------------------------------------------------- sheet
def _clean(s):
    from tools.pack import clean
    return clean(s)


FIT_COUPON_DIR = os.path.join(BASE, "pack", "fit_coupon")


def settings_text():
    """Print settings, numbers from tools.pack.PROFILE (the one profile table behind PRINT_SHEET)."""
    from tools.pack import PROFILE
    a, b = PROFILE["PLA"], PROFILE["PPA-CF"]
    return (f"PLA: 0.4 mm nozzle, 0.2 mm layers, {a['walls']} walls, {100 * a['infill']:.0f} % gyroid infill unless "
            f"noted, {a['top']} top and {a['bottom']} bottom layers, hole and XY compensation 0, no supports. "
            f"PPA-CF: hardened steel nozzle (never brass), filament dried (80 C, 8 h) and fed from a dry box, 0.2 mm "
            f"layers, {b['walls']} walls, {100 * b['infill']:.0f} % gyroid infill, {b['top']} top and {b['bottom']} "
            f"bottom layers, hole and XY compensation 0, no supports. The 3MF files carry geometry only: set the "
            f"material and these settings in Bambu Studio.")


RERUN = ("To change a number: on the Mac, after bash kedge/setup_mac.sh (from the repo root), cd base and run "
         "../kedge/.venv/bin/python -m tools.risk_coupon (it rewrites base/pack/coupons and base/pack/fit_coupon). Easier: give the failing result to a "
         "Claude session and let it change the numbers. Change one number at a time and reprint only that coupon.")


def skip_note(skipped):
    return "" if not skipped else ("NOT IN THIS SET (a subsystem file changed; tools/risk_coupon.py needs an update): "
                                   + "; ".join(f"{d['key']} {d['title']} ({d['reason']})" for d in skipped) + ".")


def sheet_md(cs, plates, skipped=()) -> str:
    L = ["# Day-one risk coupons: test sheet", "",
         "Print these before the big parts. Each coupon is made from the same geometry functions and numbers as the "
         "real part, so a pass here is a pass there. Let parts cool 10 minutes before testing. The fit coupon "
         "(tolerance tiles A to H, base/pack/fit_coupon/COUPON.md) is printed alongside.", "",
         "## Settings", "", settings_text(), "", "## If a coupon fails", "", RERUN, "",
         "## Plates", "", "| plate | material | objects |", "|---|---|---|"]
    if skipped:
        L[4:4] = ["## Missing coupons", "", skip_note(skipped), ""]
    for pl in plates:
        L.append(f"| {pl['file']} | {pl['material']} | {pl['objects']} |")
    L += ["", "## Tests", ""]
    for c in cs:
        L += [f"### {c['key']}. {c['title']}", "",
              f"- Parts: {', '.join(p.name + (f' x{p.qty}' if p.qty > 1 else '') for p in c['parts'])}",
              f"- Bought: {c['bought']}",
              f"- Do: {c['do']}",
              f"- Pass: {c['ok']}",
              f"- If it fails, change: {c['fail']}",
              f"- Made from: {c['uses']}"]
        L.append("")
    L += ["## Parts", "", "| part | material | qty | print bbox mm | what |", "|---|---|---|---|---|"]
    for p in all_parts(cs):
        lo, hi = prim.bbox(prim.place(p.solid, p.print_tf))
        L.append(f"| {p.name} | {p.material} | {p.qty} | {' x '.join(f'{hi[i] - lo[i]:.1f}' for i in range(3))} | "
                 f"{p.notes} |")
    L.append("")
    return _clean("\n".join(L))


def sheet_pdf(fn, cs, plates, fs=6.6, skipped=()):
    from reportlab.lib.units import mm
    from tools import pack
    st, Pp, tbl, note = pack.pdf_kit()
    S = [Pp("Day-one risk coupons: test sheet", "H2"),
         Pp("Print before the big parts; every coupon uses the real part's geometry functions and numbers. Plates: "
            + ", ".join(f"{pl['file']} ({pl['objects']} objects)" for pl in plates)
            + "; the fit coupon is in base/pack/fit_coupon. " + settings_text(), "SM"),
         Pp(RERUN, "SM")]
    if skipped:
        S.append(Pp(skip_note(skipped), "SM"))
    rows = [["#", "do", "pass", "if it fails, change (file: name)"]]
    for c in cs:
        ok = c["ok"]
        rows.append([f"{c['key']} {c['title']}. Bought: {c['bought']}", c["do"], ok, c["fail"]])
    S.append(tbl(rows, [30, 58, 48, 46], fs=fs))
    if note:
        S.append(Pp(note, "SM"))
    doc = pack.doc_template(fn, "Risk coupon test sheet", leftMargin=8 * mm, rightMargin=8 * mm, topMargin=8 * mm,
                            bottomMargin=8 * mm)
    doc.build(S, canvasmaker=pack.canvasmaker())


def pdf_pages(fn) -> int:
    import re
    with open(fn, "rb") as fh:
        return len(re.findall(rb"/Type\s*/Page[^s]", fh.read()))


# ---------------------------------------------------------------------------------------------- main
def write(out_dir: str = DEFAULT_OUT, P=None, log=print) -> dict:
    from tools import pack
    os.makedirs(out_dir, exist_ok=True)
    for pat in ("rc[0-9]*_*.stl", "risk_plate_*", "COUPON_TEST.*"):
        for f in glob.glob(os.path.join(out_dir, pat)):
            os.remove(f)
    skipped = []
    cs = coupons(P, skipped, log)
    parts = all_parts(cs)
    meshes, items = {}, []
    for p in parts:
        m = prim.to_trimesh(prim.place(p.solid, p.print_tf))
        m.apply_translation(-m.bounds[0])
        m.export(os.path.join(out_dir, f"{p.name}.stl"))
        meshes[p.name] = m
        items.append(dict(name=p.name, material=p.material, dims=[float(x) for x in m.extents], qty=p.qty))
    rep = dict(parts=len(parts), plates=[], errors=[], warnings=[], skipped=skipped)
    # one plate per material if the parts allow it: tighten the spacing step by step (subsystem moves change sizes)
    n_mat = len({it["material"] for it in items})
    for spacing, margin in PACKINGS:
        warn = []
        plates, errs = pack.pack_plates(items, pack.BED, spacing, margin, warn)
        if len(plates) <= n_mat:
            break
    rep["warnings"] += warn
    rep["spacing"], rep["margin"] = spacing, margin
    rep["errors"] += errs
    qty = {p.name: p.qty for p in parts}
    for i, pl in enumerate(plates, 1):
        names = pack.object_names(pl["items"], qty)
        objs = [(on, pack.place_on_plate(meshes[it["name"]], it)) for on, it in zip(names, pl["items"])]
        fn = os.path.join(out_dir, f"risk_plate_{i}_{pl['material']}.3mf")
        pack.write_3mf(fn, objs)
        nb = pack.count_3mf_items(fn)
        if nb != len(objs):
            rep["errors"].append(f"risk_plate_{i}.3mf reopens with {nb} build items, expected {len(objs)}")
        pack.plate_png(fn[:-4] + ".png",
                       f"risk coupon plate {i}, {pl['material']} ({len(objs)} objects)", objs, pack.BED)
        lo = np.min([m.bounds[0] for _, m in objs], axis=0)
        hi = np.max([m.bounds[1] for _, m in objs], axis=0)
        rep["plates"].append(dict(file=os.path.basename(fn), material=pl["material"], objects=len(objs),
                                  build_items=nb, lo=lo.tolist(), hi=hi.tolist()))
    with open(os.path.join(out_dir, "COUPON_TEST.md"), "w") as fh:
        fh.write(sheet_md(cs, rep["plates"], skipped))
    pdf = os.path.join(out_dir, "COUPON_TEST.pdf")
    for fs in (6.6, 6.2, 5.8, 5.4, 5.0):
        sheet_pdf(pdf, cs, rep["plates"], fs, skipped)
        if pdf_pages(pdf) == 1:
            break
    rep["pdf_pages"] = pdf_pages(pdf)
    log(f"risk coupon: {len(parts)} parts, {len(rep['plates'])} plate(s) -> {out_dir}")
    for pl in rep["plates"]:
        log(f"  {pl['file']}: {pl['material']}, {pl['objects']} objects")
    for w in rep["warnings"]:
        log(f"  WARN {w}")
    for e in rep["errors"]:
        log(f"  ERROR {e}")
    return rep


def write_fit_coupon(dest: str = FIT_COUPON_DIR, log=print) -> dict:
    """The base fit coupon (tools.coupon, not edited here) into dest: coupon.write puts it in <dir>/coupon, so it is
    written next to dest and moved."""
    import shutil
    from tools import coupon
    tmp = dest.rstrip("/") + ".tmp"
    shutil.rmtree(tmp, ignore_errors=True)
    rep = coupon.write(tmp, log=log)
    shutil.rmtree(dest, ignore_errors=True)
    shutil.move(os.path.join(tmp, "coupon"), dest)
    shutil.rmtree(tmp, ignore_errors=True)
    return rep


def main(argv=None):
    ap = argparse.ArgumentParser(description="Write the day-one risk coupons (STL, 3MF, PNG, COUPON_TEST.md/.pdf) and "
                                             "the fit coupon (base/pack/fit_coupon)")
    ap.add_argument("out", nargs="?", default=DEFAULT_OUT)
    ap.add_argument("--fit", default=FIT_COUPON_DIR, help="fit coupon directory ('' to skip)")
    a = ap.parse_args(argv)
    rep = write(a.out)
    errs = list(rep["errors"])
    if a.fit:
        errs += write_fit_coupon(a.fit).get("errors", [])
    return 2 if errs else 0


if __name__ == "__main__":
    sys.exit(main())
