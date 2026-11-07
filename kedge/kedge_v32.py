"""
KEDGE dart v32 generator (build123d).  Run: KV=v32 python3 kedge_v32.py out32   (KOV='{json}' overrides any parameter for variants)
v32 = v31 with Harry's anchoring layout: each drill gearmotor sits INSIDE its hollow auger sleeve (motor fixed in the arm at the
hinge, shaft drives the sleeve's tip hub through a jammed nut pair), kick leg on the centreline (hinge top centre, kick N20 beside
it), tether on a PPA-CF tail boom with a steel-pin eye, anchored by the tail screws into the hull.
Frame (mm): X along the hull, nose tip x=0, tail +X.  Y across, Z up.  Dart lying flat => ground z=-R_OUT.
Moving parts are built in their own LOCAL frame (pivot at origin, hinge axis = Y) and placed by parts.json.
Outputs per part: STL (print), STEP (B-rep), .npz (exact tessellation for the sweep), plus parts.json, meshes.json.
"""
import json, math, os, sys, time
import numpy as np
from build123d import *
import trimesh

OUT = sys.argv[1] if len(sys.argv) > 1 else "out32"
os.makedirs(OUT, exist_ok=True)
T0 = time.time()

# ------------------------------------------------------------------ parameters (measured values marked M)
P = dict(
    R_OUT=21.0, R_BAY=19.5, WALL=1.5,
    NOSE_L=20.0, NOSE_R0=11.0, NOSE_SPIG=7.0, TAIL_L=11.0, TAIL_SPIG=4.4,
    # camera  M: 13.99 w x 14.59 h, 18.34 lens front to connector back
    CAM_W=13.99, CAM_H=14.59, CAM_D=18.34, CAM_LENS_D=8.0, CAM_LENS=2.7, CAM_HOLE=9.8, NOSE_LIP=1.2,
    # worm N20  M: 12 x 10 body, 11.93 x 9.94 box, 33.04 long, shaft 29.97 tip to tip.  Pockets are square 12.4 so either face can be up.
    N20_L=33.04, N20_POCK=12.4, N20_BOX=12.0, N20_TIP=29.97 / 2, SHAFT_D=3.0,
    # kick
    SADDLE_X0=23.8, KICK_HUB_D=12.0, KICK_HUB_W=11.5, KICK_POLE_D=8.0, KICK_POLE_L=72.0, KICK_DEG=200.75, KICK_REST_DEG=1.5, KICK_MY=-11.9, KICK_RAISE=0.0,
    KICK_CH_SWEEP=25.0, KICK_CH_GROW=0.25,  # flute in the nose top for the pole: its underside at the rest angle grown 0.25 (everything it passes over the last 25 deg lies above); the flute floor is the kick rest
    FOOT_TILT=20.75, FOOT_RISE=3.4,          # foot housing tilted about its output shaft so the foot plants vertical (= KICK_DEG - 180), raised so it folds clear of the hull
    FOOT_AUGER_L=68.0, FOOT_TIP=10.0,
    # drill pair
    DRILL_X=56.0, DRILL_PIV_Z=12.0, DRILL_PIV_Y=None, DEPLOY_Y=None, DEPLOY_DEG=45.0, DEPLOY_TARGET=45.0, HORN_GAP=0.15, HEAD_IN=21.6,
    CPL_D=7.0, BOSS_D=11.0,
    # drill motor  M: gearbox 11.95, 24.27 face to back, shaft 2.84 x 56.03
    MOTOR_W=12.0, MOTOR_H=10.0, MOTOR_L=24.27, DSHAFT_L=56.03, DSHAFT_D=2.84,
    AUGER_L=100.0, AUGER_D=13.0, AUGER_CORE=6.0, AUGER_PITCH=11.0, AUGER_TIP=12.0,
    FLIGHT_D=22.0, D_PITCH=15.0, SLEEVE_BORE=16.3, SLEEVE_WALL=1.2, MOTOR_GB_L=9.0,   # drill sleeve (dart-physics: OD 22, pitch 15)
    BOOM_L=28.0, BOOM_W=9.0,   # 28: the longest that clears the launcher rail rear and top pulley at full cock (50 needs 22 mm more there)
    # F693ZZ  M: 3 x 8 x 4, flange 9.5 x 0.6 (catalogue)
    BRG_OD=8.0, BRG_W=4.0, BRG_FL_D=9.5, BRG_FL_T=0.6,
    NUT_AF=5.5, NUT_T=2.4, NYLOC_T=4.0, BUSH_GAP=0.6, POD_PS=4.4, DTERM=1.2, BOSS_FACE_D=4.6,
    # power and boards (M)
    CELL_D=18.66, CELL_L=36.87, CELL_Y=9.6, CONTACT=3.6,
    SCREW_CLR=3.3, SCREW_PILOT=2.5, RAD_Z=3.0,
    KEEL_Z=-17.0,
    TERM=1.6,                 # N20 end-cap terminal tabs (bent) + solder: room behind every motor back end
    BALLAST_G=0.0,            # nose ballast mass (dart-physics: at 7 m/s it adds nothing, the tether keeps it nose first)
)
P.update(LAYOUT="cells_fwd", DISC_T=2.0, DISC_CELL_T=3.0, LIGHT=0)
P.update(json.loads(os.environ.get("KOV", "{}")))           # variant overrides, e.g. KOV='{"AUGER_L":66,"LAYOUT":"cells_fwd"}'
R, RB, W = P["R_OUT"], P["R_BAY"], P["WALL"]
if P["DRILL_PIV_Y"] is None:                                # sleeve axis: the flight tip passes the Ø42 hull with 0.8 while swinging
    P["DRILL_PIV_Y"] = round(R + P["FLIGHT_D"] / 2 + 0.8, 2)
# deploy N20 as far outboard as the bay allows (12 x 12 box corner 0.3 inside the bay wall), so +y has room for the shelf
P["DEPLOY_Y"] = -round(math.sqrt((RB - 0.3) ** 2 - (P["DRILL_PIV_Z"] + 6.0) ** 2) - 6.0, 2)
GROUND = -R
CLR = 0.2                                  # FDM fit clearance per side (X2D, PLA)

# x stations along the hull
X_NOSE = P["NOSE_L"]                       # hull starts
X_D1 = (27.5, 30.5)                        # tray disc 1 (screwed)
X_D2 = (62.6, 62.6 + (P["DISC_CELL_T"] if P["LAYOUT"] == "cells_fwd" else P["DISC_T"]))
BZ, CZ = 26.5, P["CELL_L"] + P["CONTACT"]
if P["LAYOUT"] == "cells_fwd":                # disc 2 | cells | disc 3 | boards | disc 4
    X_CELL = (X_D2[1], X_D2[1] + CZ); X_D3 = (X_CELL[1], X_CELL[1] + P["DISC_T"])
    X_BOARD = (X_D3[1], X_D3[1] + BZ); X_D4 = (X_BOARD[1], X_BOARD[1] + P["DISC_T"])
    X_CF, X_CR, X_BF, X_BR = X_D2, X_D3, X_D3, X_D4
else:                                         # disc 2 | boards | disc 3 | cells | disc 4
    X_BOARD = (X_D2[1], X_D2[1] + BZ); X_D3 = (X_BOARD[1], X_BOARD[1] + P["DISC_CELL_T"])
    X_CELL = (X_D3[1], X_D3[1] + CZ); X_D4 = (X_CELL[1], X_CELL[1] + P["DISC_T"])
    X_BF, X_BR, X_CF, X_CR = X_D2, X_D3, X_D3, X_D4
X_TAILSP = (X_D4[1], X_D4[1] + P["TAIL_SPIG"])
L = round(X_TAILSP[1] + P["TAIL_L"], 2)
X_HULL = (X_NOSE, X_TAILSP[1])
P.update(L=L, X_D1=X_D1, X_D2=X_D2, X_BOARD=X_BOARD, X_D3=X_D3, X_CELL=X_CELL, X_D4=X_D4, X_TAILSP=X_TAILSP)

# ------------------------------------------------------------------ helpers
def bx(x0, x1, y0, y1, z0, z1):
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)

def cylx(d, l, x0, y=0, z=0):
    return Pos(x0 + l / 2, y, z) * Cylinder(d / 2, l, rotation=(0, 90, 0))

def cyly(d, l, y0, x=0, z=0):
    return Pos(x, y0 + l / 2, z) * Cylinder(d / 2, l, rotation=(90, 0, 0))

def cylz(d, l, z0, x=0, y=0):
    return Pos(x, y, z0 + l / 2) * Cylinder(d / 2, l)

def tear_y(d, l, y0, x=0, z=0):
    """horizontal hole along Y with a 45 deg teardrop roof (+Z), prints without support"""
    r = d / 2; a = r / math.sqrt(2)
    sk = Circle(r) + Polygon((-a, a), (a, a), (0, r * math.sqrt(2)), align=None)
    s = extrude(Plane.XZ * sk, amount=l)                      # XZ normal is -Y: extrudes toward -Y
    return Pos(x, y0 + l, z) * s

def tear_x(d, l, x0, y=0, z=0):
    r = d / 2; a = r / math.sqrt(2)
    sk = Circle(r) + Polygon((-a, a), (a, a), (0, r * math.sqrt(2)), align=None)
    s = extrude(Plane.YZ * sk, amount=l)                      # YZ normal +X
    return Pos(x0, y, z) * s

def rrect_x(w, h, l, x0, y=0, z=0, r=2.5):
    """rounded-rect prism along +X: w across Y, h across Z"""
    s = extrude(Plane.YZ * RectangleRounded(w, h, r), amount=l)
    return Pos(x0, y, z) * s

def rbox(x0, x1, y0, y1, z0, z1, r):
    b = bx(x0, x1, y0, y1, z0, z1)
    try:
        return fillet(b.edges(), r)
    except Exception:
        return b

def safe_fillet(solid, edges, r, tag):
    try:
        out = fillet(edges, r)
        if out.is_valid:
            return out
    except Exception as e:
        print("  fillet skipped:", tag, str(e)[:60])
    return solid

def chord_half(z, rr=RB):
    return math.sqrt(max(rr * rr - z * z, 0.0))

# ---- tangent-arc outlines: the convex hull of circles as true arcs joined by their outer tangent lines (no polygon facets)
def arc_hull_segs(circles):
    """circles [(x, z, r)] (r may be 0: a point) in the world x-z plane -> CCW list of ('arc', (x, z, r), a0, a1) / ('line', p, q)"""
    from shapely.geometry import Point
    from shapely.ops import unary_union
    from shapely.geometry.polygon import orient
    g = orient(unary_union([Point(x, z).buffer(max(r, 1e-3), 256) for x, z, r in circles]).convex_hull, 1.0)
    seq = []                                                  # which circle each hull vertex lies on, in CCW order
    for v in list(g.exterior.coords)[:-1]:
        i = min(range(len(circles)), key=lambda k: abs(math.hypot(v[0] - circles[k][0], v[1] - circles[k][1]) - circles[k][2]))
        if not seq or seq[-1] != i: seq.append(i)
    while len(seq) > 1 and seq[0] == seq[-1]: seq.pop()
    n = len(seq); tang = []
    for k in range(n):                                        # outer tangent: same outward normal at both ends
        (x1, z1, r1), (x2, z2, r2) = circles[seq[k]], circles[seq[(k + 1) % n]]
        D = math.hypot(x2 - x1, z2 - z1); ph = math.atan2(z2 - z1, x2 - x1)
        th = ph - math.acos(max(-1.0, min(1.0, (r1 - r2) / D)))
        tang.append(((x1 + r1 * math.cos(th), z1 + r1 * math.sin(th)), (x2 + r2 * math.cos(th), z2 + r2 * math.sin(th)), th))
    segs = []
    for k in range(n):
        c = circles[seq[k]]; a0 = tang[k - 1][2]; a1 = tang[k][2]
        while a1 < a0: a1 += 2 * math.pi
        if c[2] > 1e-6 and a1 - a0 > 1e-4: segs.append(("arc", c, a0, a1))
        segs.append(("line", tang[k][0], tang[k][1]))
    return segs

def arc_prism(circles, y0, y1):
    """tangent-arc outline in x-z extruded along y from y0 to y1"""
    edges = []
    for sg in arc_hull_segs(circles):
        if sg[0] == "line":
            p, q = sg[1], sg[2]
            if math.hypot(q[0] - p[0], q[1] - p[1]) > 1e-4: edges.append(Edge.make_line((p[0], y0, p[1]), (q[0], y0, q[1])))
        else:
            (cx, cz, r), a0, a1 = sg[1], sg[2], sg[3]
            pt = lambda a: (cx + r * math.cos(a), y0, cz + r * math.sin(a))
            edges.append(Edge.make_three_point_arc(pt(a0), pt((a0 + a1) / 2), pt(a1)))
    return extrude(Face(Wire(edges)), amount=y1 - y0, dir=(0, 1, 0))

def yfillet(sol, ys, r, tag, zmin=-99.0):
    """round the outline edges lying in the planes y = ys (the side faces of an arc_prism)"""
    ed = [e for e in sol.edges() if any(abs(e.position_at(t).Y - yy) < 1e-3 for yy in ys for t in (0.0, 0.5, 1.0)) and
          all(abs(e.position_at(t).Y - e.position_at(0.0).Y) < 1e-3 for t in (0.5, 1.0)) and e.center().Z > zmin]
    return safe_fillet(sol, ed, r, tag)

def skirt(feat, r=1.5, pad=4.0, z0=0.0, xlim=(-1e9, 1e9)):
    """blend a feature standing on the Ø42 skin into it with an r fillet: union with a local skin patch, fillet the edges where
    the feature meets the skin. Returns (patch + feature, the patch box) so the caller swaps the box out of the full shell."""
    bb = feat.bounding_box()
    xa, xb, ya, yb, za = max(bb.min.X - pad, xlim[0]), min(bb.max.X + pad, xlim[1]), bb.min.Y - pad, bb.max.Y + pad, max(bb.min.Z - pad, z0)
    box = bx(xa, xb, ya, yb, za, bb.max.Z + pad)
    patch = (cylx(2 * R, xb - xa + 2, xa - 1) - cylx(2 * RB, xb - xa + 4, xa - 2)) & box
    u = patch + feat
    ins = lambda q: xa + 0.3 < q.X < xb - 0.3 and ya + 0.3 < q.Y < yb - 0.3 and q.Z > za + 0.3
    ed = [e for e in u.edges() if all(abs(math.hypot(q.Y, q.Z) - R) < 0.02 and ins(q) for q in [e.position_at(t) for t in (0.0, 0.25, 0.5, 0.75, 1.0)])]
    return safe_fillet(u, ed, r, "skirt"), box

parts = []
FIX = {}            # name -> (solid) for later reference

def mesh_of(solid, tol=0.02):
    tmp = os.path.join(OUT, "_tmp.stl")
    export_stl(solid, tmp, tolerance=tol, angular_tolerance=0.15)
    m = trimesh.load(tmp, process=True)
    m.update_faces(m.nondegenerate_faces(height=1e-6)); m.merge_vertices(digits_vertex=4); m.remove_unreferenced_vertices()
    return m

def add(name, solid, group, color, pos=(0, 0, 0), axis=None, deg=(0, 0), note="", printable=True, parent=None,
        orient=None, step=True, tol=0.02):
    if isinstance(solid, trimesh.Trimesh):
        m = solid.copy()
        m.export(os.path.join(OUT, name + ".stl"))
        if step and printable:
            from meshstep import mesh_to_solid
            export_step(mesh_to_solid(m), os.path.join(OUT, name + ".step"))
        np.savez_compressed(os.path.join(OUT, name + ".npz"), v=m.vertices, f=m.faces)
        ext = m.bounding_box.extents
        parts.append(dict(name=name, file=name + ".stl", group=group, color=color, pos=[float(v) for v in pos], axis=axis,
                          deg=list(deg), note=note, parent=parent, printable=printable, orient=orient,
                          dims=[round(float(e), 1) for e in ext], volume_cm3=round(float(m.volume) / 1000, 2),
                          watertight=bool(m.is_watertight), nsolids=len(m.split(only_watertight=False))))
        FIX[name] = m
        print(f"{name:20s} {np.round(ext,1)} vol {m.volume/1000:6.2f} cm3 wt={m.is_watertight} bodies={len(m.split(only_watertight=False))} (mesh) t={time.time()-T0:.0f}s")
        return
    if isinstance(solid, (list, tuple)):
        solid = Compound(children=list(solid))
    if hasattr(solid, "solids") and len(solid.solids()) > 1 and printable:
        print(f"  WARNING {name}: {len(solid.solids())} separate solids")
        for so in solid.solids():
            bb = so.bounding_box(); print(f"     vol {so.volume:9.2f}  x {bb.min.X:6.1f}..{bb.max.X:6.1f} y {bb.min.Y:6.1f}..{bb.max.Y:6.1f} z {bb.min.Z:6.1f}..{bb.max.Z:6.1f}")
    m = mesh_of(solid, tol)
    if not m.is_watertight:
        trimesh.repair.fill_holes(m); trimesh.repair.fix_normals(m)
    m.export(os.path.join(OUT, name + ".stl"))
    if step and printable:
        export_step(solid, os.path.join(OUT, name + ".step"))
    np.savez_compressed(os.path.join(OUT, name + ".npz"), v=m.vertices, f=m.faces)
    ext = m.bounding_box.extents
    parts.append(dict(name=name, file=name + ".stl", group=group, color=color, pos=[float(v) for v in pos], axis=axis,
                      deg=list(deg), note=note, parent=parent, printable=printable, orient=orient,
                      dims=[round(float(e), 1) for e in ext], volume_cm3=round(float(solid.volume) / 1000, 2),
                      watertight=bool(m.is_watertight), nsolids=len(solid.solids()) if hasattr(solid, "solids") else 1))
    FIX[name] = solid
    print(f"{name:20s} {np.round(ext,1)} vol {solid.volume/1000:6.2f} cm3 wt={m.is_watertight} solids={len(solid.solids())} t={time.time()-T0:.0f}s")

# ================================================================== DRILL ARM + SLEEVE AUGER layout (local: pivot at origin, auger axis = local +x)
HH = 12.8                                                  # arm head: round Ø12.8 on the coupler + flat-bottomed front (the stop face)
Y_C = P["DRILL_PIV_Y"]                                     # sleeve axis |y|
POD_FACE = P["HEAD_IN"]                                    # head inner face |y| (boss face 0.4 inboard)
HEAD_YL = (POD_FACE - Y_C, 3.0)                            # head y range in the LEFT arm's local frame
CB_D, CB_DEPTH = 10.4, 1.2                                 # counterbore in the head outer face for the printed cap
CORE_D = P["SLEEVE_BORE"] + 2 * P["SLEEVE_WALL"]           # sleeve core OD (bore clears the gearbox diagonal 15.62 by 0.34 a side)
ASH_X0, ASH_X1 = 7.0, 8.5                                    # thrust shoulder on the arm (the sleeve base face runs on it)
S0 = ASH_X1 + 0.2                                           # sleeve base
BW1 = ASH_X1 + 1.5                                          # bushing back wall (wire exit)
MB = BW1 + P["DTERM"]                                      # motor back (end cap, tabs bent up)
GB0 = MB + P["MOTOR_L"] - P["MOTOR_GB_L"]                  # gearbox rear face (can 15.27, gearbox 9)
XS = MB + P["MOTOR_L"]                                     # gearbox face
SHAFT_END = XS + P["DSHAFT_L"]
BUSH_X1 = GB0 - 0.3; BUSH_D = P["SLEEVE_BORE"] - 0.5       # stationary bushing round the motor can (the sleeve runs on it)
X_NYF = SHAFT_END - P["NYLOC_T"] + 0.4                     # nyloc B seat face (outside the hub wall, reached through the tip)
X_NA = (X_NYF - 2.0 - P["NUT_T"] - 0.2, X_NYF - 2.0)       # nut A pocket (inside the 2 mm hub wall): jammed pair = keyed both ways
X_PLUG = X_NA[0] - 1.5                                     # tip plug (45 deg roof inside)
TIP = S0 + P["AUGER_L"]
P.update(POD_XS=XS, POD_S0=S0, POD_TIP=TIP, CORE_D=CORE_D, SHAFT_END=SHAFT_END)
# ================================================================== KICK geometry (needed by the hull)
pk = P["N20_POCK"]
SX0 = P["SADDLE_X0"]
SAD_W = pk + 2 * 1.3                                        # 15.0
SAD_H = pk + 2 * 1.3
KICK_STUB = 9.94 / 2 + 1.0                                  # kick N20 -y stub trimmed to 1 mm past the worm box face (measured box width 9.94)
SAD_RF, SAD_RR = 7.4, 6.5                                    # saddle side profile: arc round the worm box (on the pivot), arc round the motor tail
KICK_Z = R + SAD_H / 2 - 1.0 + P["KICK_RAISE"]               # v32c: not raised (KICK_RAISE 0); the planted pole lies in a flute in the nose top
KICK_X = SX0 + 1.3 + P["N20_BOX"] / 2 + 0.2                  # output shaft = centre of the worm box
KMY = P["KICK_MY"]                                           # kick N20 beside the centreline (right side); its +y stub carries the hub
HUB_Y0 = KMY + SAD_W / 2 + 0.4                               # hub inner face, 0.4 off the saddle wall
KICK_Y = 0.0                                                 # pole and foot on the dart centreline
HUB_Y1 = HUB_Y0 + P["KICK_HUB_W"]
EAR_Y0 = HUB_Y1 + 0.4
EAR_T = P["BRG_W"] - P["BRG_FL_T"] + 1.0                     # 4.4 fin; the bearing stands 0.4 proud of its inner face onto the hub boss
EAR_Y1 = EAR_Y0 + EAR_T
BRG_EAR_Y = HUB_Y1                                          # ear F693ZZ inner face (touches the hub boss); flange in a counterbore
P.update(KICK_X=KICK_X, KICK_Y=KICK_Y, KICK_Z=KICK_Z, EAR_Y0=EAR_Y0, EAR_Y1=EAR_Y1)
FL_Y0, FL_Y1, FL_R = -P["KICK_POLE_D"] / 2 - 0.3, HUB_Y1 + 0.3, 3.8   # pole flute across y, its floor corner radius (pole underside corners r 3.5: 0.25 all round)
_ta = math.radians(P["KICK_DEG"] + P["KICK_REST_DEG"] - 180.0)
FL_X1 = min(KICK_X - (KICK_Z - (P["KICK_POLE_D"] / 2 + P["KICK_CH_GROW"]) / math.cos(_ta) - R) / math.tan(_ta), X_NOSE + P["NOSE_SPIG"] - 0.3)   # flute floor leaves the skin

CSINK_SUNK = 0.8                                            # head sunk so its edge is flush on the curved skin
def csink(xs, s, zz):
    """90 deg countersink + short counterbore for an M3 flat head on the Ø42 skin (s = +1 / -1 side)"""
    yo = s * chord_half(zz, R); yc = yo - s * CSINK_SUNK
    cone = Solid.make_cone(3.25, 1.5, 1.75, Plane(origin=(xs, yc, zz), z_dir=(0, -s, 0)))
    cb = cyly(6.5, CSINK_SUNK + 1.5, min(yc, yo + s * 1.5), x=xs, z=zz)
    return [cone, cb]
def seat(xs, s, zz):
    """flat-bottomed pocket in the part under the hull so the sunk flat head never touches it"""
    yo = s * chord_half(zz, R); y_in = yo - s * (CSINK_SUNK + 1.75 + 0.3)
    return cyly(5.0, abs(yo - y_in) + 1, min(yo, y_in) - (0.5 if s > 0 else 0.5), x=xs, z=zz)
NOSE_LOCK_X = P["NOSE_L"] + 2.7                             # rib x 22.7..24.7

def prism_xz(pts, y0, y1):
    """polygon in the world x-z plane extruded along y from y0 to y1"""
    face = Plane.XZ * Polygon(*[(x, z) for x, z in pts], align=None)       # Plane.XZ: local (x, y) = world (x, z), normal -Y
    sol = extrude(face, amount=y1 - y0)                                    # direction follows the polygon winding:
    return Pos(0, y0 - sol.bounding_box().min.Y, 0) * sol                  # place it on y0..y1 whichever way it went

def poly_prism(g, y0, y1):
    """shapely polygon(s) in the world x-z plane -> build123d solid on y0..y1"""
    from shapely.geometry import Polygon as SP
    gs = getattr(g, "geoms", [g]); out = None
    for p in gs:
        if p.area < 0.5: continue
        p = p.simplify(0.02)
        sol = prism_xz(list(p.exterior.coords)[:-1], y0, y1)
        for h in p.interiors:
            sol = sol - prism_xz(list(h.coords)[:-1], y0 - 1, y1 + 1)
        out = sol if out is None else out + sol
    return out

def pivot_blocks():
    """One faired block per flank round the drill pivot (replaces v31's boss, tombstone, horn and fold lug): a teardrop in x-z
    (tangent arcs and lines: front r 9.4, back r 8.4, flat on the split plane), its side edges rounded r 1.2 and skirted into the
    skin r 1.5. Inboard of the arm head it is the coupler boss; over the inner 6 mm of the head it is carved by the swept arm (head,
    neck, shoulder, sleeve) from folded to DEPLOY_DEG, grown HORN_GAP: what is left in front of the head is the DEPLOY STOP face,
    under it the FOLD STOP face (both about 1 deg past the ends of the swing). Tow turns the pods further down: that moment goes
    into this block, not the deploy worm or the couplers. Returns (blocks, carve): the carve is cut after the skirt."""
    from shapely.geometry import Point, box as sbox
    from shapely.ops import unary_union
    from shapely import affinity
    zp, xp = P["DRILL_PIV_Z"], P["DRILL_X"]
    circ = [(xp - 1.6, zp + 1.4, 9.4), (xp, zp, 8.4), (xp - 6.0, -1.0, 1.5), (xp + 6.0, -1.0, 1.5)]
    prof = unary_union([Point(0, 0).buffer(HH / 2, 96), sbox(-HH / 2, -HH / 2, 0, 0), sbox(-1.05, -HH / 2 - 0.05, ASH_X1, HH / 2 + 0.05),
                        sbox(ASH_X0 - (CORE_D - BUSH_D) / 2 - 0.05, -CORE_D / 2 - 0.05, ASH_X1, CORE_D / 2 + 0.05), sbox(S0, -P["FLIGHT_D"] / 2, S0 + 8.0, P["FLIGHT_D"] / 2)])
    sw = unary_union([affinity.rotate(prof, -a, origin=(0, 0)) for a in np.arange(0.0, P["DEPLOY_DEG"] + 0.01, 0.5)])
    sw = affinity.translate(sw.buffer(P["HORN_GAP"], 32), xp, zp)
    yi, yo = chord_half(zp) - 0.5, POD_FACE + 6.0
    blocks = carve = None
    for sg in (1, -1):
        a_ = (yi, yo) if sg > 0 else (-yo, -yi)
        blk = yfillet(arc_prism(circ, *a_), a_, 1.2, "pivot block") & bx(xp - 20, xp + 20, -40, 40, 0, 40)
        c_ = (POD_FACE - 0.4, yo + 1.0) if sg > 0 else (-yo - 1.0, -POD_FACE + 0.4)
        cv = poly_prism(sw, *c_)
        blocks = blk if blocks is None else blocks + blk
        carve = cv if carve is None else carve + cv
    return blocks, carve

def kick_channel(grow, y0, y1, r):
    """flute for the kick pole in the nose and the hull top: the pole's underside at the rest angle (KICK_DEG + KICK_REST_DEG),
    grown, as a bar with its floor corners rounded r (the pole's underside corners are rounded to fit); everything the pole
    passes over the last KICK_CH_SWEEP deg before the plant lies above that floor. The floor is the KICK REST: tow lifts the
    nose, the soil pushes the planted foot down and turns the leg on past the plant; the pole then lies in the flute and that
    load goes into the nose (clamped by the hull rib) instead of the kick worm and the PLA D-bore on its shaft."""
    lx0, hz = P["KICK_HUB_D"] / 2 - 1.0, P["KICK_POLE_D"] / 2 + grow
    bar = bx(lx0, lx0 + P["KICK_POLE_L"] + 10, y0, y1, -60, hz)
    bar = safe_fillet(bar, [e for e in bar.edges().filter_by(Axis.X) if abs(e.center().Z - hz) < 1e-3], r, "flute")
    return Pos(KICK_X, 0, KICK_Z) * (Rot(0, -(P["KICK_DEG"] + P["KICK_REST_DEG"]), 0) * bar)

def lip_fillet(sol, on_skin, y0, y1, r, tag):
    """round the lips where the flute meets an outer skin"""
    ed = [e for e in sol.edges() if e.length > 5 and all(on_skin(q) and y0 - 0.05 < q.Y < y1 + 0.05 and q.Z > 5 for q in [e.position_at(t) for t in (0.0, 0.25, 0.5, 0.75, 1.0)])]
    return safe_fillet(sol, ed, r, tag)

# ================================================================== HULL
def build_hull():
    hx0, hx1 = X_HULL
    shell = cylx(2 * R, hx1 - hx0, hx0) - cylx(2 * RB, hx1 - hx0 + 2, hx0 - 1)
    # kick saddle beside the centreline (y KICK_MY): square pocket for the worm N20 lying along X, worm box at the nose end, open
    # at the top. Low fairing: tangent arcs round the worm box and the motor's tail, a sloped front from the nose shoulder and a
    # sloped tail, side edges rounded r 1.2, skirted into the skin r 1.5 (one family with the ear and the pivot blocks)
    sl = 1.3 + P["N20_L"] + P["TERM"] + 1.3
    x_end = SX0 + sl
    sy = (KMY - SAD_W / 2, KMY + SAD_W / 2)
    sad_c = [(KICK_X, KICK_Z, SAD_RF), (x_end - SAD_RR, KICK_Z + 5.5 - SAD_RR, SAD_RR), (X_NOSE + 1.6, 10.0, 1.0), (x_end + 3.0, 10.0, 1.0)]
    saddle = yfillet(yfillet(arc_prism(sad_c, *sy), (sy[0],), 4.5, "saddle outboard"), (sy[1],), 1.2, "saddle") & bx(X_NOSE + 0.01, hx1, -60, 60, 0, 80)   # round outboard shoulder
    # ear: teardrop fin on the hub's outboard side carrying an F693ZZ for the hub screw (same arcs, edges and skirt)
    ear_c = [(KICK_X, KICK_Z, P["BRG_FL_D"] / 2 + 2.2), (KICK_X - 9.0, 12.0, 1.0), (KICK_X + 9.0, 12.0, 1.0)]
    ear = yfillet(arc_prism(ear_c, EAR_Y0, EAR_Y1), (EAR_Y0, EAR_Y1), 1.2, "ear")
    blocks, carve = pivot_blocks()
    sk1, box1 = skirt(saddle + ear, xlim=X_HULL)
    sk2, box2 = skirt(blocks, xlim=X_HULL)
    body = (shell - box1 - box2) + sk1 + sk2
    body = body - carve                                          # pivot-block stop faces
    body = body - kick_channel(P["KICK_CH_GROW"], FL_Y0, FL_Y1, FL_R)   # the pole drops into the hull top at the plant ...
    body = body - bx(X_NOSE - 1, FL_X1, FL_Y0, FL_Y1, 12.0, 40)          # ... and over the nose spigot the wall is slotted clean (the spigot carries the flute floor)
    body = body - cylx(2 * RB, hx1 - hx0 + 2, hx0 - 1)          # nothing intrudes into the bay
    # internal rib that sits in the nose spigot groove: the closed clamshell locks the nose axially
    body = body + (cylx(2 * RB + 0.2, 2.0, NOSE_LOCK_X) - cylx(2 * (RB - 1.0), 4.0, NOSE_LOCK_X - 1))
    cuts = []
    # N20 pocket (open top) + the output shaft's +y stub slot (its -y stub is trimmed inside the pocket: KICK_STUB). No strap: the
    # square worm box keys the motor in the square pocket (it cannot roll more than about 3 deg before its corner meets the floor),
    # the +y stub is held by the hub in the ear bearing; a drop of CA on the can stops it rattling (the folded pole leaves no room
    # for a tie over the motor)
    px0 = SX0 + 1.3
    cuts.append(bx(px0, px0 + P["N20_L"] + P["TERM"], KMY - pk / 2 - CLR, KMY + pk / 2 + CLR, KICK_Z - pk / 2 - CLR, KICK_Z + 30))
    cuts.append(bx(KICK_X - (P["SHAFT_D"] + 0.6) / 2, KICK_X + (P["SHAFT_D"] + 0.6) / 2, KMY, KMY + 9, KICK_Z, KICK_Z + 30))   # entry slot (inboard wall): the +y stub drops in from the top
    cuts.append(tear_y(P["SHAFT_D"] + 1.2, 9, KMY, x=KICK_X, z=KICK_Z))
    # wire holes: kick N20 leads straight down from its end-cap tabs; foot leads up through the skin under the folded pole
    cuts.append(cylz(4.4, KICK_Z - 12.0, 12.0, x=px0 + P["N20_L"] + 0.3, y=KMY))
    cuts.append(cylz(4.0, 30, 8, x=42.0, y=2.0))
    # ear bearing: Ø8 bore through, flange recess on the outer face
    cuts.append(tear_y(P["BRG_OD"] + 0.15, EAR_T + 2, EAR_Y0 - 1, x=KICK_X, z=KICK_Z))
    cuts.append(cyly(P["BRG_FL_D"] + 0.3, EAR_Y1 + 1 - (BRG_EAR_Y + P["BRG_W"] - P["BRG_FL_T"] - 0.05), BRG_EAR_Y + P["BRG_W"] - P["BRG_FL_T"] - 0.05, x=KICK_X, z=KICK_Z))
    # pivot blocks: coupler bores
    cuts.append(tear_y(P["CPL_D"] + 0.4, 2 * R + 20, -R - 10, x=P["DRILL_X"], z=P["DRILL_PIV_Z"]))
    # drill-motor wire holes on both flanks just aft of the pivot blocks (the leads leave the arm neck at the motor's end cap,
    # a few mm from the hinge) into the cell bay above the VTX
    for s in (-1, 1):
        cuts.append(cyly(4.0, 6.0, 8.5 if s > 0 else -14.5, x=P["DRILL_X"] + 10.5, z=16.5))
    # radial screws: disc 1 (x mid) and tail spigot, z = +-3 both sides
    for xs in ((X_D1[0] + X_D1[1]) / 2, (X_TAILSP[0] + X_TAILSP[1]) / 2):
        for s in (-1, 1):
            for zz in (-P["RAD_Z"], P["RAD_Z"]):
                cuts.append(cyly(P["SCREW_CLR"], 14, (RB - 2) if s > 0 else -(RB - 2) - 14 + 0, x=xs, z=zz))
                cuts += csink(xs, s, zz)
    # XIAO USB-C through the -y flank (both halves, across the split): flash and debug with the hull closed. 12.5 x 7 fits a
    # normal USB-C overmold; the receptacle face is 2.6 inside the skin
    xu = X_BOARD[0] + 0.4 + 17.74 / 2
    cuts.append(bx(xu - 6.25, xu + 6.25, -R - 2, -RB + 0.3, -2.17 - 3.5, -2.17 + 3.5))
    for c in cuts:
        body = body - c
    return body

print("hull ...")
hull = build_hull()
top = hull & bx(-10, L + 10, -60, 60, 0, 80)
bot = hull & bx(-10, L + 10, -60, 60, -80, 0)
add("hull_top", top, "hull", "#9aa5b1", orient="split face down (on the bed), saddle up",
    note="Clamshell upper half: skin; kick saddle beside the centreline (low fairing round the worm N20, which drops in with its stub down the inboard slot); teardrop ear fin with an F693ZZ for the hub screw; two faired pivot blocks (coupler bores; their carved faces are the deploy and fold stops of the drill arms); saddle, ear and blocks are tangent-arc outlines with r 1.2 edges and r 1.5 skirts into the skin; a slot over the nose spigot where the kick pole lands; wire holes. Radial M3 clearance at z=+3 into tray disc 1 and the tail spigot")
add("hull_bottom", bot, "hull", "#7d8794", orient="split face down, belly up",
    note="Clamshell lower half: plain Ø42 cylinder (launcher cradle surface). Radial M3 clearance at z=-3 into tray disc 1 and the tail spigot")

# ================================================================== NOSE (hollow, 1.5 walls)
def build_nose():
    nl, r0 = P["NOSE_L"], P["NOSE_R0"]
    outer = Solid.make_cone(r0, R, nl, Plane.YZ)               # along +X from x=0
    spig = cylx(2 * RB - 2 * CLR, P["NOSE_SPIG"], nl - 0.01)
    body = outer + spig
    body = safe_fillet(body, body.edges().group_by(Axis.X)[0], 1.5, "nose tip")
    env_ = body
    # hollow: inner cone offset by the wall, open at the back
    k = (R - r0) / nl
    xi0, xi1 = W * 1.6, nl - 1.5
    inner = Solid.make_cone(r0 - W * 1.12 + k * xi0, r0 - W * 1.12 + k * xi1, xi1 - xi0, Plane.YZ.offset(xi0))
    inner_sp = cylx(2 * RB - 2 * CLR - 2 * 3.0, P["NOSE_SPIG"] + 2, nl - 1.5)          # 3 mm spigot wall carries the lock groove
    body = body - inner - inner_sp
    # camera tube: walls around the pocket from the tip lip back to the spigot end
    cw, ch = P["CAM_W"] + 2 * CLR, P["CAM_H"] + 2 * CLR
    tube = rrect_x(cw + 2 * 1.2, ch + 2 * 1.2, nl + P["NOSE_SPIG"] - 1.0, 0.5, r=1.5)
    body = body + (tube & (outer + spig))
    # pole channel: a solid strip under it first (the groove never opens into the cavity), then the groove itself
    body = body + (kick_channel(P["KICK_CH_GROW"] + 1.6, FL_Y0 - 1.6, FL_Y1 + 1.6, FL_R + 1.6) & env_)
    body = body - bx(P["NOSE_LIP"] + P["CAM_LENS"], nl + P["NOSE_SPIG"] + 5, -cw / 2, cw / 2, -ch / 2, ch / 2)
    body = body - cylx(P["CAM_HOLE"], P["NOSE_LIP"] + P["CAM_LENS"] + 1, -1)
    body = body - (cylx(2 * R + 2, 2.4, NOSE_LOCK_X - 0.2) - cylx(2 * (RB - 1.0) - 0.4, 4, NOSE_LOCK_X - 1))   # lock groove for the hull rib
    body = body - kick_channel(P["KICK_CH_GROW"], FL_Y0, FL_Y1, FL_R)
    kc = (R - r0) / nl
    body = lip_fillet(body, lambda q: -0.05 < q.X < nl + 0.05 and abs(math.hypot(q.Y, q.Z) - (r0 + kc * q.X)) < 0.03, FL_Y0, FL_Y1, 1.2, "nose flute lips")
    return body

print("nose ...")
nose = build_nose()
add("nose", nose, "hull", "#b8c0ca", orient="tip down on the bed (flat Ø22), spigot up",
    note="Blunt hollow nose: Ø22 flat tip with the Ø9.8 lens hole, Caddx Ant pocket 14.4 x 15.0 (slides in from the back, 1.2 lip + lens recess hold it), 1.5 walls; a round-cornered flute along the top (solid underneath) where the planted kick pole lies: its floor is the kick rest; spigot with a groove that the hull rib locks when the clamshell closes")

# ================================================================== TAIL CAP (screwed, carries the tether)
JST_TOP_X = (X_TAILSP[1] + 0.48, X_TAILSP[1] + 0.48 + 6.1)     # header slot along x (5.75 header + 0.35)
BOOM_SH = 7.6                                                   # boom shank square (clears the trimmed power-socket pins)
X_ANC = (X_TAILSP[0] + 0.13, X_TAILSP[0] + 0.13 + 3.6)          # boom anchor disc in the tail spigot (the 4 tail screws go into it)
def build_tail():
    x0 = X_TAILSP[0]; xc = X_TAILSP[1]
    cap = cylx(2 * R, P["TAIL_L"], xc)
    cap = safe_fillet(cap, cap.edges().group_by(Axis.X)[-1], 2.5, "tail edge")
    sp = cylx(2 * RB - 2 * CLR, P["TAIL_SPIG"] + 0.01, x0)
    body = cap + sp
    # hollow: 1.5 wall, 2.4 end wall; spigot ring 3.0 thick radially (screws bite into it)
    body = body - cylx(2 * RB - 2 * CLR - 6.0, P["TAIL_SPIG"] + P["TAIL_L"] - 2.4, x0 - 0.01)
    END_T = 3.5
    r0c = RB - CLR - 3.0; rc = R - W; hc = rc - r0c                      # 45 deg roof from the spigot bore out to the cap wall
    body = body - Solid.make_cone(r0c, rc, hc, Plane.YZ.offset(xc + 1.5))
    body = body - cylx(2 * rc, max(L - END_T - (xc + 1.5 + hc), 0.1), xc + 1.5 + hc - 0.01)
    # tether cross hole through the boss (teardrop roof toward -X = up when printed end-down)
    body = body - cylx(2 * RB - 2 * CLR - 6.0, P["TAIL_SPIG"] + 1.5 + 0.01, x0 - 0.01)
    # power loop socket seat (top): the JST-XH 2-pin header sits on a floor at z 6.5 with its pins down through an opening,
    # the shorting plug goes in from the top through a window in the cap and ends under the skin (recessed: nothing sticks
    # out, the launcher cradle and barrel never touch it, and the plug can be pulled or pushed with the dart on the launcher)
    body = body + (bx(xc - 0.55, L - END_T + 0.01, -5.2, 5.2, 4.5, 30) & cylx(2 * RB - 2 * CLR, L - xc + 1.5, xc - 1.5))
    body = body - bx(*JST_TOP_X, -3.85, 3.85, 6.5, R + 1) - bx(JST_TOP_X[0] + 0.9, JST_TOP_X[1] - 0.9, -2.8, 2.8, 3.0, 7.0)
    # charge socket seat (bottom, header recessed 1.4 under the skin; the plug is only in on the bench)
    body = body + (bx(xc - 0.55, L - END_T + 0.01, -6.6, 6.6, -30, -10.5) & cylx(2 * RB - 2 * CLR, L - xc + 1.5, xc - 1.5))
    body = body - bx(*JST_TOP_X, -5.1, 5.1, -R - 1, -12.0) - bx(JST_TOP_X[0] + 0.9, JST_TOP_X[1] - 0.9, -4.0, 4.0, -12.5, -9.5)
    body = body - bx(L - END_T - 1, L + 1, -BOOM_SH / 2 - 0.2, BOOM_SH / 2 + 0.2, -BOOM_SH / 2 - 0.2, BOOM_SH / 2 + 0.2)   # boom shank: keyed square
    body = body - Solid.make_cone(BOOM_SH / 2 + 1.6, BOOM_SH / 2 + 0.2, 1.4, Plane(origin=(L + 0.01, 0, 0), z_dir=(-1, 0, 0)))  # lead-in chamfer
    # radial pilot holes for the 4 hull screws
    xs = (X_TAILSP[0] + X_TAILSP[1]) / 2
    for s in (-1, 1):
        for zz in (-P["RAD_Z"], P["RAD_Z"]):
            body = body - cyly(P["SCREW_PILOT"], 7, (RB - 6) if s > 0 else -(RB + 1), x=xs, z=zz)
            body = body - seat(xs, s, zz)                 # clearance for the sunk screw head in the spigot ring
    return body

print("tail ...")
tail = build_tail()
add("tail_cap", tail, "hull", "#b8c0ca", orient="end face down, spigot up (no supports)",
    note="Tail cap: 4 radial M3 from the hull through its spigot ring into the boom anchor disc inside it; the boom's square shank runs through a keyed square hole in the end wall (the pusher face is still the flat ring round it). Top: recessed seat for the JST-XH 2-pin power loop socket (the ON plug, reachable with the dart on the launcher). Bottom: recessed seat for the JST-XH 3-pin charge socket (bench only)")

# ================================================================== TAIL BOOM (tether stinger) + its anchor disc
def build_boom():
    """PPA-CF boom on the axis: a 9 x 12 head inside the anchor disc, a 7.6 x 7.6 shank through the tail cap end wall, a 10 x 10
    9 x 9 rounded boom BOOM_L long behind the tail, and a clevis eye at the tip: the tether loop runs round a steel M3 x 8 countersunk
    screw between two tines, open aft and to both sides (the line can swing 180 deg left-right, its exit edges rounded). Every
    section fits the launcher pusher's Ø14 hole, so the boom slides out of it at the shot."""
    bw, x1 = P["BOOM_W"], L + P["BOOM_L"]
    head = bx(X_ANC[0] + 0.15, X_ANC[0] + 2.35, -bw / 2, bw / 2, -6.0, 6.0)
    shank = bx(X_ANC[0] + 2.2, L + 1.0, -BOOM_SH / 2, BOOM_SH / 2, -BOOM_SH / 2, BOOM_SH / 2)
    boom = rrect_x(bw + 1.0, bw + 1.0, 6.0, L + 0.5, r=2.8)                         # 10 x 10 collar out of the tail cap (diag 12.4 < Ø14 pusher hole)
    boom = boom + rrect_x(bw, bw, x1 - L - 5.0 - 6.0, L + 6.0, r=2.2)               # 9 x 9 stinger to the eye
    tip = Pos(x1 - 5.0, 0, 0) * Cylinder(bw / 2, bw)                      # round end in plan, axis z (the pin axis)
    tip = tip & bx(x1 - 5.0, x1 + 1, -bw / 2, bw / 2, -bw / 2, bw / 2)
    tip = tip + bx(x1 - 5.5, x1 - 5.0, -bw / 2, bw / 2, -bw / 2, bw / 2)
    b = head + shank + boom + tip
    b = b - bx(x1 - 9.0, x1 + 1, -bw, bw, -1.5, 2.5)                       # clevis gap for the line loop (top tine 2.0, bottom 3.0)
    b = b - Pos(x1 - 9.0, 0, 0.5) * Cylinder(2.0, 20, rotation=(90, 0, 0)) # rounded gap root
    b = b - cylz(P["SCREW_CLR"], 4.0, 2.4, x=x1 - 5.0)                     # pin: clearance in the top tine
    b = b - Solid.make_cone(1.65, 3.15, 1.6, Plane(origin=(x1 - 5.0, 0, bw / 2 - 1.6), z_dir=(0, 0, 1)))   # countersink, head flush
    b = b - cylz(P["SCREW_PILOT"], 4.0, -bw / 2 - 1, x=x1 - 5.0)           # pilot in the bottom tine (the screw threads in)
    for sz in (1, -1):                                                      # 45 deg lead on the tip: it enters the pusher hole on retrieval
        b = b - prism_xz([(x1 - 1.8, sz * bw / 2 + sz * 0.01), (x1 + 0.5, sz * bw / 2 + sz * 0.01), (x1 + 0.5, sz * (bw / 2 - 2.3))], -bw, bw)
    return b

def build_anchor():
    d = cylx(2 * (RB - CLR - 3.0) - 0.4, X_ANC[1] - X_ANC[0], X_ANC[0])
    d = d - bx(X_ANC[0] - 1, X_ANC[1] + 1, -BOOM_SH / 2 - 0.2, BOOM_SH / 2 + 0.2, -BOOM_SH / 2 - 0.2, BOOM_SH / 2 + 0.2)
    d = d - bx(X_ANC[0] - 1, X_ANC[0] + 2.4, -P["BOOM_W"] / 2 - 0.2, P["BOOM_W"] / 2 + 0.2, -6.2, 6.2)       # head recess
    for zz0, zz1 in ((9.0, 17.0), (-17.0, -9.0)):                                                        # wire notches to the seats
        d = d - bx(X_ANC[0] - 1, X_ANC[1] + 1, -4.6, 4.6, zz0, zz1)
    xs_ = (X_TAILSP[0] + X_TAILSP[1]) / 2
    for s_ in (-1, 1):
        for zz in (-P["RAD_Z"], P["RAD_Z"]):
            d = d - cyly(P["SCREW_PILOT"], 6, (RB - 8) if s_ > 0 else -(RB - 2), x=xs_, z=zz)
    return d

print("boom ...")
add("tail_boom", build_boom(), "hull", "#2f3b45", orient="lying on its side (layers along the boom), PPA-CF",
    note="Tail boom (tether stinger), PPA-CF: %g mm behind the tail, 9 x 9 rounded, clevis eye with a steel M3 x 8 countersunk screw as the line's wear pin (flush both sides). The tow load goes head -> anchor disc -> the 4 tail screws -> hull; the square shank is keyed in the tail cap" % P["BOOM_L"])
add("boom_anchor", build_anchor(), "hull", "#2f3b45", orient="flat on the bed, PPA-CF",
    note="Boom anchor disc Ø32.2 x 3.6 inside the tail spigot: holds the boom head in a recess; the four radial tail screws run through the hull and the tail cap spigot into it, so the tether pulls on the hull, not on the tail cap")

# ================================================================== TRAY: one printed spine, nose to tail
DECK_A, DECK_B, DECK_C = -13.8, -5.0, 4.0          # floor TOP surfaces in the board zone
FT = 1.2                                            # floor / deck thickness
KEEL_CH = 8.4                                       # keel channel height under the cells: 9.2 x 8.4, top 0.5 under the cells
KZ = P["KEEL_Z"]

BOARD = {   # name: (Lx, Wy, Hz) measured; headers and terminal removed. v31: XIAO turned 90 deg (USB-C faces -y, 22.75 incl. the
            # USB-C lip), BNO085 turned 90 deg into the sled; DRV8833 #3 is off the dart (firmware v4)
    "xiao": (17.74, 22.75, 4.49), "drv8833_1": (20.31, 12.70, 3.5), "drv8833_2": (20.31, 12.70, 3.5),
    "ina219": (25.69, 20.32, 4.70), "bno085": (22.77, 25.72, 4.77),
    "mp1584": (22.26, 16.96, 3.98), "vtx": (42.25, 19.91, 6.01), "bms_2s": (40.8, 8.0, 2.1),
}
xb0 = X_BOARD[0] + 0.4
LAY = {}
gB = 0.8
Y_USB = 18.3                                        # XIAO USB-C receptacle face at y -18.3 (hull inner wall -18.85 at the deck)
LAY["xiao"] = (xb0, -Y_USB + BOARD["xiao"][1] / 2, DECK_B)
LAY["drv8833_1"] = (xb0, -Y_USB + BOARD["xiao"][1] + 0.3 + BOARD["drv8833_1"][1] / 2, DECK_B)
wC = BOARD["drv8833_2"][1] + BOARD["ina219"][1] + gB
LAY["drv8833_2"] = (xb0, -wC / 2 + BOARD["drv8833_2"][1] / 2, DECK_C)
LAY["ina219"] = (xb0, wC / 2 - BOARD["ina219"][1] / 2, DECK_C)
SLED_X0 = X_D1[1] + 0.5
SHELF_Z = -5.0
LAY["mp1584"] = (SLED_X0, 0.9 + BOARD["mp1584"][1] / 2, SHELF_Z)      # 5 V buck on the +y shelf beside the deploy cradle
SH_X1 = P["DRILL_X"] - 0.4                                               # sled rear wall x SH_X1-1.2 .. SH_X1
LAY["bno085"] = (X_D1[1] + 0.25, 0.0, -14.0)                             # IMU in the sled on two rails, under the MP1584 shelf
VTX_Z = 10.3
LAY["vtx"] = (X_CF[0] + (X_CR[1] - X_CF[0] - BOARD["vtx"][0]) / 2, 0.0, VTX_Z)
LAY["bms_2s"] = (X_CF[0] + (X_CR[1] - X_CF[0] - BOARD["bms_2s"][0]) / 2, 0.0, KZ + 0.15)

def disc(x0, x1, screws=False, windows=()):
    d = cylx(2 * RB - 2 * CLR, x1 - x0, x0)
    for w in windows:
        d = d - w
    if screws:
        xm = (x0 + x1) / 2
        for s in (-1, 1):
            for zz in (-P["RAD_Z"], P["RAD_Z"]):
                d = d - cyly(P["SCREW_PILOT"], 8, (RB - 8) if s > 0 else -RB - 0.5, x=xm, z=zz)
                d = d - seat(xm, s, zz)
    return d

def plate(x0, x1, ztop, t=FT, margin=0.8, ylim=None):
    zz = ztop - t if ztop - t < 0 < ztop else (ztop if ztop > 0 else ztop - t)
    half = chord_half(max(abs(ztop), abs(ztop - t))) - margin
    y0, y1 = (-half, half) if ylim is None else (max(-half, ylim[0]), min(half, ylim[1]))
    return bx(x0, x1, y0, y1, ztop - t, ztop)

def build_tray():
    win_top = bx(-1, 1, -6, 6, 10, 18)     # template, moved per disc
    def wins(x0, x1, extra=(), top_w=6.0, top_z=11.0, top_off=0.0):
        ws = [bx(x0 - 1, x1 + 1, -top_w + top_off, top_w + top_off, top_z, 30),            # top notch, open to the rim (camera, VTX, kick leads)
              bx(x0 - 1, x1 + 1, -5, 5, -3.0, 2.5)]                   # centre window (I2C, motor leads)
        return ws + list(extra)
    parts_ = []
    # keel, full length
    kx0, kx1 = X_D1[0], X_D4[1]
    keel = plate(kx0, kx1, KZ, t=1.2, margin=0.5)
    parts_.append(keel)
    # discs
    dyy, zcc, pkc = P["DEPLOY_Y"], P["DRILL_PIV_Z"], pk / 2 + CLR + 0.2
    d1 = disc(*X_D1, screws=True, windows=wins(*X_D1, top_off=4.0, extra=[bx(X_D1[0] - 1, X_D1[1] + 1, dyy - pkc, dyy + pkc, zcc - pkc, 30),   # deploy N20 body passes through
                                                                       bx(X_D1[0] - 1, X_D1[1] + 1, dyy - 4.0, dyy + 4.0, -3.0, zcc - pkc + 0.5)]))  # its leads drop to the centre window
    vn = lambda x0, x1: [bx(x0 - 1, x1 + 1, -4.6, 4.6, KZ, KZ + KEEL_CH)]                              # keel channel: BMS stick + the harness under the cells
    def cdisc(xx):                                   # discs bounding the cell bay get the VTX notch and the BMS channel
        if xx in (X_CF, X_CR): return disc(*xx, windows=wins(*xx, vn(*xx), top_w=10.6, top_z=VTX_Z - 1.2))
        return disc(*xx, windows=wins(*xx, vn(*xx)))          # disc 4: keel window to the charge socket
    d2, d3, d4 = cdisc(X_D2), cdisc(X_D3), cdisc(X_D4)
    parts_ += [d1, d2, d3, d4]
    # board zone floors (bridge between disc 2 and disc 3)
    for zt in (DECK_B, DECK_C):                      # deck A is gone (BNO085 moved to the sled): the bay floor is open for the harness
        p = plate(X_BF[1] - 0.01, X_BR[0] + 0.01, zt)
        hw_ = 5.5 if P["LIGHT"] else 4.0
        x1_ = X_BOARD[1] - (0.6 if zt == DECK_B else 4)      # deck B: the slot runs on behind the XIAO = wire chase down to the keel
        p = p - bx(X_BOARD[0] + (3 if P["LIGHT"] else 4), x1_, -hw_, hw_, zt - 2, zt + 1)       # lightening / wire slot
        parts_.append(p)
    # BNO085 rails in the sled (the leads to disc 1 and the deploy motor run along the keel between them)
    for s in (-1, 1):
        parts_.append(bx(X_D1[1] - 0.01, SH_X1 - 1.2 + 0.01, s * 8.4, s * 9.6, KZ - 0.1, -14.0))
    # sled: deploy-motor cradle (cantilever from disc 1), DRV#3 shelf with a rear wall standing on the keel
    dy = P["DEPLOY_Y"]; zc = P["DRILL_PIV_Z"]; pkh = pk / 2 + CLR
    cx0, cx1 = X_D1[1] - 0.01, P["DRILL_X"] - P["CPL_D"] / 2 - 0.6
    cradle = bx(cx0, cx1, dy - pkh - 1.2, dy + pkh + 1.2, zc - pkh - 1.2, zc + 2.0)
    cradle = cradle - bx(cx0 - 1, cx1 + 1, dy - pkh, dy + pkh, zc - pkh, zc + 10)
    parts_.append(bx(cx0, SH_X1, dy - pkh - 1.2, dy + pkh + 1.2, zc - pkh - 1.2, zc - pkh))            # cradle floor runs on to the rear wall (bridge)
    cradle = cradle - bx(cx0 + 8, cx0 + 11, -30, 30, zc - pkh - 2, zc - pkh + 0.6)    # zip-tie groove under the cradle
    parts_.append(cradle)
    sh_x1 = SH_X1                                             # rear wall x 54.4..55.6: below the couplers (z<3.5) and the worm box (z>1)
    shelf = plate(cx0, sh_x1, SHELF_Z, ylim=(0.5, 30))
    parts_.append(shelf)
    parts_.append(bx(sh_x1 - 1.2, sh_x1, dy - pkh - 1.2, chord_half(SHELF_Z) - 1.0, KZ - 0.1, SHELF_Z - 0.01))   # rear wall (shelf)
    parts_.append(bx(sh_x1 - 1.2, sh_x1, dy - pkh - 1.2, dy + pkh + 1.2, KZ - 0.1, zc - pkh - 0.01))             # rear wall up to the cradle floor
    # cell cradle: two saddles on the keel + VTX deck over the cells
    cy, cr = P["CELL_Y"], P["CELL_D"] / 2 + CLR
    for xs_ in (X_CELL[0] + 8, X_CELL[1] - 10):
        s = bx(xs_, xs_ + 2.0, -RB + 2, RB - 2, KZ - 0.1, -4.0)
        s = s - cylx(2 * cr, 4, xs_ - 1, y=cy) - cylx(2 * cr, 4, xs_ - 1, y=-cy)
        s = s - bx(xs_ - 1, xs_ + 3, -4.6, 4.6, KZ - 1, KZ + KEEL_CH)              # keel channel (BMS stick + harness)
        s = s & cylx(2 * RB - 2 * CLR, 4, xs_ - 1)
        parts_.append(s)
    vdeck = plate(X_CF[0] - 0.01, X_CR[1] + 0.01, VTX_Z, t=0.8, ylim=(-10.8, 10.8))
    parts_.append(vdeck)
    t = parts_[0]
    for p in parts_[1:]:
        t = t + p
    t = t & cylx(2 * RB - 2 * CLR, L, -1)
    t = t & bx(0, L, -30, 30, KZ - 1.2, 30)                  # flat bottom = the keel plane, prints keel down
    t = t - bx(SH_X1 - 2.0, SH_X1 + 1.0, -4.6, 4.6, KZ, KZ + KEEL_CH)   # keel channel through the sled rear wall (deploy and BNO085 leads)
    # battery contacts. Launch (nose-first, ~80 g) throws the cells AFT, so the aft contacts are rigid plates and the
    # springs sit on the forward disc: launch presses the cells harder onto the plates and contact never opens.
    t = t - bx(X_CF[1] - 0.8, X_CF[1] + 0.01, -cy - 6, cy + 6, -6, 6)                # front: two springs joined (cell 1 + to cell 2 -)
    for s in (-1, 1):
        t = t - bx(X_CR[0] - 0.01, X_CR[0] + 0.8, s * cy - 6, s * cy + 6, -6, 6)    # rear: rigid plates (B- cell 1, B+ cell 2)
        t = t - cylx(2.5, 5, X_CR[0] - 1, y=s * cy, z=-7.5)                           # lead holes
    return t

print("tray ...")
tray = build_tray()
add("tray", tray, "hull", "#c9b37e", orient="keel down on the bed; discs stand up, decks bridge 26 mm between discs",
    note="One-piece spine nose to tail: disc 1 (4 radial M3 from the hull), sled (deploy-N20 cradle, MP1584 on the keel, DRV8833 #3 shelf), disc 2, 3 decks (BNO085 / XIAO + DRV#1 / DRV#2 + INA219), disc 3, cell bay (2 x 18350 side by side on saddles, BMS stick under them, VTX deck over them), disc 4. Battery contact recesses for a 2xAA-holder contact set")

# ================================================================== KICK LEG (local frame: pivot at origin, axis Y; folded pole along +X)
# ---- augers: trimesh + manifold (the swept helix does not survive OCC booleans; STEP of augers is faceted)
from trimesh.creation import cylinder as tcyl, cone as tcone
from trimesh.transformations import rotation_matrix as RM, translation_matrix as TM, reflection_matrix
def tU(ms): return trimesh.boolean.union(ms, engine="manifold")
def tD(a, bs): return trimesh.boolean.difference([a] + list(bs), engine="manifold")
def tcylx(d, l, x0, y=0, z=0, n=96):
    c = tcyl(radius=d / 2, height=l, sections=n); c.apply_transform(RM(math.pi / 2, [0, 1, 0])); c.apply_transform(TM([x0 + l / 2, y, z])); return c
def tbox(x0, x1, y0, y1, z0, z1):
    b = trimesh.creation.box(extents=[x1 - x0, y1 - y0, z1 - z0]); b.apply_transform(TM([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])); return b
def helical_flight(r_core, depth, pitch, length, t_root=2.0, t_tip=1.2, steps_per_turn=120, embed=0.8):
    n = max(4, int(length / pitch * steps_per_turn)); V, F = [], []
    for i in range(n + 1):
        x = length * i / n; th = 2 * math.pi * x / pitch
        rc = r_core(x); ri = max(rc - embed, 0.15); ro = rc + depth(x); c, s_ = math.cos(th), math.sin(th)
        for rr, dx in ((ri, -t_tip / 2 - (ro - ri)), (ro, -t_tip / 2), (ro, t_tip / 2), (ri, t_tip / 2 + 0.6)):
            V.append([x + dx, rr * c, rr * s_])
    for i in range(n):
        a_ = 4 * i
        for k in range(4):
            p0, p1 = a_ + k, a_ + (k + 1) % 4; q0, q1 = p0 + 4, p1 + 4
            F.append([p0, q0, p1]); F.append([p1, q0, q1])
    F.append([0, 1, 2]); F.append([0, 2, 3]); e = 4 * n; F.append([e, e + 2, e + 1]); F.append([e, e + 3, e + 2])
    m = trimesh.Trimesh(vertices=np.array(V), faces=np.array(F), process=True); m.fix_normals(); return m
def screw_auger(flight_d, length, pitch, core_d, tip_len, lefthand=False, bore=None, nut_x=None, base_boss=False, dbore=None):
    body_l = length - tip_len; rc0 = core_d / 2
    core = tcylx(core_d, body_l, 0, n=64)
    tip = tcone(radius=rc0, height=tip_len, sections=96); tip.apply_transform(RM(math.pi / 2, [0, 1, 0])); tip.apply_transform(TM([body_l, 0, 0]))
    r_core = lambda x: rc0 if x <= body_l else rc0 * (1 - (x - body_l) / tip_len)
    dep0 = (flight_d - core_d) / 2
    depth = lambda x: dep0 if x <= body_l else dep0 * (1 - 0.75 * (x - body_l) / tip_len)
    x_end = body_l + tip_len * (1 - 1.0 / rc0)
    fl = helical_flight(r_core, depth, pitch, x_end)
    fl = trimesh.boolean.intersection([fl, tbox(0.02, length + 5, -20, 20, -20, 20)], engine="manifold")   # 0.02 over the core end: no coplanar faces
    if lefthand:
        fl.apply_transform(reflection_matrix([0, 0, 0], [0, 1, 0])); fl.invert() if fl.volume < 0 else None; fl.fix_normals()
    ms = [core, tip, fl]
    if base_boss: ms.append(tcylx(P["BOSS_FACE_D"], 1.1, -0.8, n=48))   # inner race only (shield and outer ring stay free); 0.3 into the core
    if nut_x is not None: ms.append(tcylx(10.4, P["NYLOC_T"] + 3, nut_x - 1.5))
    if dbore is not None: ms.append(tcylx(9.0, 9.0, 0, n=96))
    a = tU(ms)
    cuts = []
    if bore is not None: cuts.append(tcylx(bore, (nut_x if nut_x else 9) + 2.5, -2, n=32))
    if nut_x is not None:
        af = P["NUT_AF"] + 0.35
        nut = tcyl(radius=af / math.sqrt(3), height=P["NYLOC_T"] + 0.3, sections=6); nut.apply_transform(RM(math.pi / 2, [0, 1, 0])); nut.apply_transform(TM([nut_x + (P["NYLOC_T"] + 0.3) / 2, 0, 0]))
        cuts += [nut, tbox(nut_x + 0.05, nut_x + P["NYLOC_T"] + 0.25, -af / 2 - 0.08, af / 2 + 0.08, 0.3, 12)]   # slot a hair wider than the hex flats: no coplanar faces
    if dbore is not None:
        cuts.append(tD(tcylx(dbore, 10, -1, n=32), [tbox(-2, 10, -5, 5, dbore / 2 - 0.5, 5)]))
        g = tcyl(radius=1.25, height=8, sections=24); g.apply_transform(TM([5.0, 0, 3.0])); cuts.append(g)
    return tD(a, cuts) if cuts else a

WEB_RUN = 13.0                                                       # the web runs this far along the tilted housing's floor
FOOT_YC = HUB_Y1 - (P["N20_POCK"] + 2.6) / 2                         # foot housing / foot N20 centreline = 0: the foot plants on the dart axis; outboard faces flush = print datum
def foot_pt(fbox, x, z):
    """housing frame -> leg frame (x-z): the foot housing is tilted FOOT_TILT about its output shaft and raised FOOT_RISE"""
    t = math.radians(P["FOOT_TILT"]); dx = x - fbox
    return (fbox + dx * math.cos(t) + z * math.sin(t), P["FOOT_RISE"] - dx * math.sin(t) + z * math.cos(t))
def foot_loc(fbox, s):
    return Pos(fbox, 0, P["FOOT_RISE"]) * (Rot(0, P["FOOT_TILT"], 0) * (Pos(-fbox, 0, 0) * s))
def foot_T(fbox):
    return TM([fbox, 0, P["FOOT_RISE"]]) @ RM(math.radians(P["FOOT_TILT"]), [0, 1, 0]) @ TM([-fbox, 0, 0])

def build_kick():
    hub = cyly(P["KICK_HUB_D"], P["KICK_HUB_W"] - 0.4, HUB_Y0)                        # outer face recessed 0.4 ...
    hub = safe_fillet(hub, hub.edges(), 0.8, "hub")
    hub = hub + cyly(P["BOSS_FACE_D"] - 0.2, 0.42, HUB_Y1 - 0.41)                     # ... except a boss on the ear bearing's INNER race, flush with the print datum
    pole_x0 = P["KICK_HUB_D"] / 2 - 1.0
    hw2 = HUB_Y1
    xe = pole_x0 + P["KICK_POLE_L"]
    pe = xe - 14.0                                                                     # the pole runs into the web here
    pole = bx(pole_x0, pe, 0, hw2, -P["KICK_POLE_D"] / 2, P["KICK_POLE_D"] / 2)
    pole = safe_fillet(pole, [e for e in pole.edges().filter_by(Axis.X) if abs(e.center().Y - hw2) < 1e-3], FL_R - 0.3, "pole corners")   # fits the flute
    pole = pole + cylx(P["KICK_POLE_D"], pe - pole_x0, pole_x0)
    # foot housing (built in its own frame, then tilted about the output shaft so the foot plants vertical): square pocket for
    # the foot worm N20 in line with the housing, output shaft along the housing's +Z
    fh = pk + 2.6
    fx0 = xe - 4
    fbox = fx0 + 4 + 1.3 + P["N20_L"] + 0.6 - P["N20_BOX"] / 2 - 0.2      # foot output-shaft x (box at the far end)
    house = rrect_x(fh, fh, P["N20_L"] + 0.6 + 2.6 + 4, fx0, y=FOOT_YC, r=2.5)
    house = safe_fillet(house, house.edges().group_by(Axis.X)[-1], 1.5, "foot end")
    hcuts = [
        bx(fx0 + 4 + 1.3 - P["TERM"], fx0 + 4 + 1.3 + P["N20_L"] + 0.6, -20, FOOT_YC + pk / 2 + CLR, -pk / 2 - CLR, pk / 2 + CLR),   # pocket (+ terminal room), open on -y (inboard; up when printing)
        cylz(P["SHAFT_D"] + 1.0, 12, pk / 2 - 1, x=fbox, y=FOOT_YC),                                         # output shaft up through the top wall
        bx(fbox - (P["SHAFT_D"] + 0.6) / 2, fbox + (P["SHAFT_D"] + 0.6) / 2, -20, FOOT_YC, pk / 2 - 1, pk / 2 + 11),   # slot: shaft slides in from the open side
        cylz(P["SHAFT_D"] + 1.0, 4, -pk / 2 - 2.5, x=fbox, y=FOOT_YC),                                                  # trimmed lower stub clearance
        bx(fbox - (P["SHAFT_D"] + 1.0) / 2, fbox + (P["SHAFT_D"] + 1.0) / 2, -20, FOOT_YC, -pk / 2 - 2.5, -pk / 2 + 1.5),
        bx(fx0 + 19, fx0 + 22, -20, 20, -fh / 2 - 1, -fh / 2 + 0.8),                                         # strap groove (on the can, clear of the web)
    ]
    # web: tangent arcs from the pole end up to the raised housing rear and along its floor (carries the drilling push into the pole)
    wc = [(pe, 0.0, P["KICK_POLE_D"] / 2)] + [foot_pt(fbox, x_, z_) + (2.5,) for x_, z_ in
          ((fx0 + 2.5, fh / 2 - 2.5), (fx0 + 2.5, -fh / 2 + 2.5), (fx0 + WEB_RUN, -fh / 2 + 2.5))]
    web = arc_prism(wc, -P["KICK_POLE_D"] / 2, hw2)
    web = yfillet(web, (-P["KICK_POLE_D"] / 2,), P["KICK_POLE_D"] / 2 - 0.1, "web")          # its -y side rounds like the pole ...
    web = yfillet(web, (hw2,), FL_R - 0.3, "web datum side")                                  # ... and its +y edges like the pole corners
    leg = hub + pole + web + foot_loc(fbox, house)
    for c in hcuts:
        leg = leg - foot_loc(fbox, c)
    pr = foot_pt(fbox, fx0 + 4 + 1.3 - P["TERM"] + 1.0, 0.0)                                         # pocket rear (motor end cap)
    wa = (xe - 12.0, 0.0); wd = (pr[0] - wa[0], pr[1] - wa[1]); wl = math.hypot(*wd)
    cuts = [
        cylx(4.6, xe - 12.0 - pole_x0 - 3 + 2.3, pole_x0 + 3),                                              # wire channel through the pole ...
        Solid.make_cylinder(2.3, wl + 1.0, Plane(origin=(wa[0], 0, wa[1]), z_dir=(wd[0] / wl, 0, wd[1] / wl))),   # ... up through the web into the pocket
        Pos(pole_x0 + 7.5, -3, 0) * Cylinder(1.9, 8, rotation=(90, 0, 0)),                                    # wire exit near the hub (toward the hull)
        Pos(0, 0, 0) * D_bore(P["SHAFT_D"] + 0.25, (KMY + P["N20_TIP"] + 0.5) - (HUB_Y0 - 0.5), HUB_Y0 - 0.5),   # D-bore on the kick N20's +y stub
        cyly(P["SCREW_PILOT"], 7.0, HUB_Y1 + 1.0 - 7.0 + 0.2),   # M3 pilot from the outer end through the inner-race boss
    ]
    for c in cuts:
        leg = leg - c
    return leg, fx0, fbox

def D_bore(d, depth, y0):
    c = cyly(d, depth, y0)
    flat = bx(-3, 3, y0 - 1, y0 + depth + 1, d / 2 - 0.5, 4)      # D flat 0.5 deep (N20 shaft flat)
    return c - flat

print("kick ...")
kick, FX0, FBOX = build_kick()
add("kick_leg", kick, "legs", "#e07b39", pos=(KICK_X, KICK_Y, KICK_Z), axis="y", deg=(0, P["KICK_DEG"]),
    orient="on its side (pole axis horizontal, hub face down)",
    note="Kick leg on the dart centreline: Ø12 x 11.5 hub on the +y stub of the kick N20 (which sits beside it), M3 x 8 screw from outside through the ear F693ZZ into the hub end; Ø8 pole (flat-sided to the print datum, those corners r 3.5 to fit the nose flute) with a Ø4.6 wire channel, a web up to the foot housing, which is tilted %.2f deg about the foot N20's output shaft so the foot auger plants vertical on the dart axis ahead of the nose (output up and leaning aft when folded)" % P["FOOT_TILT"])

ztop_h = pk / 2 + 1.3
fa = screw_auger(P["AUGER_D"], P["FOOT_AUGER_L"], P["AUGER_PITCH"], P["AUGER_CORE"], P["FOOT_TIP"], lefthand=False, dbore=P["SHAFT_D"] + 0.15)
fa.apply_transform(RM(-math.pi / 2, [0, 1, 0]))            # +X -> +Z
fa.apply_transform(TM([FBOX, FOOT_YC, ztop_h + 0.3]))           # 0.3 over the housing: drilling push bottoms the hub on the housing, not the worm gearbox
fa.apply_transform(foot_T(FBOX))                                # tilted with the housing: vertical at the plant
add("foot_auger", fa, "legs", "#b23a48", parent="kick_leg", orient="base down",
    note="Foot auger Ø%g x %g, core Ø%g, right-hand flight, D-bore on the foot N20 output shaft (7 mm engaged), M3 grub screw in the hub; its base runs 0.3 over the housing so drilling push goes into the housing, not the worm gearbox" % (P["AUGER_D"], P["FOOT_AUGER_L"], P["AUGER_CORE"]))

# ================================================================== DRILL ARMS + SLEEVE AUGERS (local: pivot at origin, axis Y; auger along +X)
def D_bore_tear(d, y0, y1):
    """D bore along y (flat at +z, the coupler's flat) with a 45 deg teardrop toward +x: the arm prints head down (local -x on the bed)"""
    r = d / 2; a = r / math.sqrt(2)
    sk = Circle(r) + Polygon((a, -a), (a, a), (r * math.sqrt(2), 0), align=None)
    sol = Pos(0, y1, 0) * extrude(Plane.XZ * sk, amount=y1 - y0)
    return sol - bx(-5, 5, y0 - 1, y1 + 1, r - 0.6, 5)

def build_arm(side):
    """Drill arm: round head on the coupler (flat front = the stop face), a short neck, the thrust shoulder the sleeve runs on, and
    the stationary bushing that holds the drill gearmotor by its can (12 x 10 flats key it against rotation). Built for the left
    side and mirrored for the right."""
    y0, y1 = HEAD_YL
    head = cyly(HH, y1 - y0, y0) + bx(-HH / 2, 0, y0, y1, -HH / 2, 0)
    neck = bx(0.0, ASH_X1, -BUSH_D / 2, y1, -HH / 2, HH / 2)                       # starts where its top is tangent to the head (no ledge)
    shoulder = cylx(CORE_D, ASH_X1 - ASH_X0, ASH_X0)
    under = Solid.make_cone(BUSH_D / 2, CORE_D / 2, (CORE_D - BUSH_D) / 2, Plane.YZ.offset(ASH_X0 - (CORE_D - BUSH_D) / 2))   # 45 deg under the shoulder
    bush = cylx(BUSH_D, BUSH_X1 - ASH_X1 + 0.01, ASH_X1 - 0.01)
    hn = head + neck                                                                # head and neck: stop corner r 0.5, face edges r 1.2 (as the pivot blocks)
    hn = safe_fillet(hn, [e for e in hn.edges().filter_by(Axis.Y) if abs(e.center().X + HH / 2) < 1e-3 and abs(e.center().Z + HH / 2) < 1e-3], 0.5, "arm stop corner")
    hn = yfillet(yfillet(hn, (y0,), 1.2, "arm inner face"), (y1,), 1.2, "arm outer face")
    arm = hn + under + shoulder + bush
    can_hole = cylx(P["MOTOR_W"] + 2 * CLR, BUSH_X1 - BW1 + 1, BW1) & bx(BW1 - 1, BUSH_X1 + 2, -P["MOTOR_H"] / 2 - CLR, P["MOTOR_H"] / 2 + CLR, -10, 10)
    cuts = [D_bore_tear(P["CPL_D"] + 0.25, y0 - 1, y1 + 1),
            cyly(CB_D, CB_DEPTH + 1, y1 - CB_DEPTH),                                       # counterbore for the pod cap
            can_hole,                                                                       # can socket: flats at +-y
            cylx(4.5, BW1 - 2.0 + 0.5, 2.0, y=0, z=2.5),                                    # wire channel from the end cap ...
            cylz(4.5, 10, 2.5, x=3.5, y=0)]                                                 # ... out of the neck top (folded: up)
    for c in cuts:
        arm = arm - c
    if side < 0:
        arm = arm.mirror(Plane.XZ)
    return arm

def sleeve_auger(lefthand):
    """Hollow auger sleeve round the stationary motor: tube (bore SLEEVE_BORE, core CORE_D) from S0, closed tip plug with the jammed
    nut pair (nut A in a side slot inside the 2 mm hub wall, nyloc B tightened on it through the tip bore), flights FLIGHT_D at D_PITCH
    (1.2 at the edge, 2.0 at the root, 45 deg lower flank: prints standing on its base)."""
    L_ = TIP - S0
    core = tcylx(CORE_D, L_ - 6.0, 0, n=128)
    r_c = CORE_D / 2
    nose = tcone(radius=r_c, height=30.0, sections=128); nose.apply_transform(RM(math.pi / 2, [0, 1, 0])); nose.apply_transform(TM([L_ - 6.0, 0, 0]))
    nose = tD(nose, [tbox(L_, L_ + 30, -20, 20, -20, 20)])                       # tip: 6 mm taper to 0.7 of the core (front ring round the Ø9.4 bore)
    rc = lambda x: r_c if x <= L_ - 6.0 else r_c - 0.2 * r_c * (x - (L_ - 6.0)) / 6.0
    dep = lambda x: (P["FLIGHT_D"] / 2 - r_c) * (1.0 if x <= L_ - 9.0 else max(0.0, (L_ - 1.0 - x) / 8.0))
    fl = helical_flight(rc, dep, P["D_PITCH"], L_ - 1.0, t_root=2.0, t_tip=1.2)
    fl = trimesh.boolean.intersection([fl, tbox(0.02, L_ + 5, -20, 20, -20, 20)], engine="manifold")
    if lefthand:
        fl.apply_transform(reflection_matrix([0, 0, 0], [0, 1, 0])); fl.invert() if fl.volume < 0 else None; fl.fix_normals()
    au = tU([core, nose, fl])
    xp = X_PLUG - S0; xa0, xa1 = X_NA[0] - S0, X_NA[1] - S0; xn = X_NYF - S0
    xb = xp - (P["SLEEVE_BORE"] / 2 - 1.7)                                     # bore ends in a 45 deg roof that closes at the plug
    bore = tcylx(P["SLEEVE_BORE"], xb + 1.0, -1.0, n=128)
    roof = tcone(radius=P["SLEEVE_BORE"] / 2, height=P["SLEEVE_BORE"] / 2, sections=128); roof.apply_transform(RM(math.pi / 2, [0, 1, 0])); roof.apply_transform(TM([xb - 0.01, 0, 0]))
    af = P["NUT_AF"] + 0.35
    nut = tcyl(radius=af / math.sqrt(3), height=xa1 - xa0, sections=6); nut.apply_transform(RM(math.pi / 2, [0, 1, 0])); nut.apply_transform(TM([(xa0 + xa1) / 2, 0, 0]))
    slot = tbox(xa0, xa1, -af / 2 - 0.05, af / 2 + 0.05, 0.0, 20.0)                # nut A goes in sideways (local +z)
    cuts = [bore, roof, tcylx(P["DSHAFT_D"] + 0.55, xn - xp + 2.0, xp - 1.0, n=32), nut, slot, tcylx(12.5, L_ - xn + 2.0, xn, n=64)]   # nut-driver bore to nyloc B, light tip
    return tD(au, cuts)

def drill(prefix, side, color):
    arm = build_arm(side)
    pos = (P["DRILL_X"], side * Y_C, P["DRILL_PIV_Z"])
    add(prefix, arm, "legs", color, pos=pos, axis="y", deg=(0, P["DEPLOY_DEG"]), orient="standing on the head's flat front face (local -x down), no supports",
        note="Drill arm: round head on the Ø7 D coupler (flat front face = stop face on the hull's pivot block), short neck with the motor-lead channel, thrust shoulder for the sleeve, stationary bushing Ø%.1f that holds the drill gearmotor by its can (flats key it; a drop of CA holds it axially), %.1f mm terminal room behind it (bend the tabs up)" % (BUSH_D, P["DTERM"]))
    lh = side < 0
    au = sleeve_auger(lh)
    au.apply_transform(TM([S0, 0, 0]))
    tipc = tcone(radius=CORE_D / 2 * 0.8, height=7.0, sections=96); tipc.apply_transform(RM(math.pi / 2, [0, 1, 0])); tipc.apply_transform(TM([TIP, 0, 0]))
    tipc = tD(tU([tipc, tcylx(12.4, 4.0, TIP - 4.0, n=96)]), [tbox(TIP + 5.5, TIP + 9, -10, 10, -10, 10)])   # rounded-off cone + Ø12.4 press-in spigot
    add(prefix + "_tip", tipc, "legs", "#b23a48", parent=prefix, orient="spigot up",
        note="Press-in tip cap (PLA, a drop of CA): closes the nut-driver bore over the nyloc so the sleeve ends in a clean point; pull it with pliers to service the drill")
    add(prefix + "_auger", au, "legs", "#b23a48", parent=prefix, orient="standing on its base, brim 5 mm",
        note="Sleeve auger Ø%g flights (pitch %g, %s-hand: the pair counter-rotates and both screw in) on a Ø%.1f core, %g long from the arm shoulder; it turns round the stationary motor (bore %.1f over the 15.6 gearbox diagonal) and runs on the arm's bushing; the tip hub takes the motor shaft through a jammed M3 nut (side slot) + nyloc (through the tip bore): keyed in both drilling directions" % (P["FLIGHT_D"], P["D_PITCH"], "left" if lh else "right", CORE_D, P["AUGER_L"], P["SLEEVE_BORE"]))
    can = cylx(P["MOTOR_W"], GB0 - MB, MB) & bx(MB - 1, GB0 + 1, -P["MOTOR_H"] / 2, P["MOTOR_H"] / 2, -10, 10)
    gbx = bx(GB0, XS, -P["MOTOR_H"] / 2, P["MOTOR_H"] / 2, -P["MOTOR_W"] / 2, P["MOTOR_W"] / 2)
    add(prefix + "_motor", can + gbx, "electronics", "#6c757d", parent=prefix, printable=False, note="Drill gearmotor 12 x 10, gearbox 11.95, 24.27 face to back (measured): fixed in the arm, inside the sleeve")
    add(prefix + "_shaft", cylx(P["DSHAFT_D"], P["DSHAFT_L"], XS), "electronics", "#d0d4d8", parent=prefix, printable=False,
        note="M3 threaded motor shaft Ø2.84 x 56.03 (measured), runs free inside the sleeve to the tip hub")
    nut = trimesh.creation.cylinder(radius=P["NUT_AF"] / math.sqrt(3), height=P["NUT_T"], sections=6); nut.apply_transform(RM(math.pi / 2, [0, 1, 0])); nut.apply_transform(TM([X_NA[0] + 0.1 + P["NUT_T"] / 2, 0, 0])); nut = tD(nut, [tcylx(3.0, 6, X_NA[0] - 1)])
    add(prefix + "_nut", nut, "electronics", "#777", parent=prefix, printable=False, note="M3 nut A in the hub's side slot (threadlocker)")
    ny = trimesh.creation.cylinder(radius=P["NUT_AF"] / math.sqrt(3), height=P["NYLOC_T"], sections=6); ny.apply_transform(RM(math.pi / 2, [0, 1, 0])); ny.apply_transform(TM([X_NYF + 0.02 + P["NYLOC_T"] / 2, 0, 0])); ny = tD(ny, [tcylx(3.0, 8, X_NYF - 2)])
    add(prefix + "_nyloc", ny, "electronics", "#777", parent=prefix, printable=False, note="M3 nyloc B tightened against nut A through the 2 mm hub wall (threadlocker): the jam keys the sleeve to the shaft both ways")

print("drills ...")
drill("drill_leg_L", +1, "#d9534f")
drill("drill_leg_R", -1, "#c0392b")

# couplers (rotate with the pods about the axle line)
def coupler(y0, y1, sock_from_inner, sock_depth):
    lo, hi = sorted((y0, y1))
    c = cyly(P["CPL_D"], hi - lo, lo)
    c = c - bx(-5, 5, lo - 1, hi + 1, P["CPL_D"] / 2 - 0.6, 5)                  # D flat, full length
    yi = y0
    sd = sock_depth
    sock = D_bore(P["SHAFT_D"] + 0.2, sd, yi if y1 > y0 else yi - sd)
    c = c - sock
    yo = y1
    pil = cyly(P["SCREW_PILOT"], 7, yo - 7 if y1 > y0 else yo)
    c = c - pil
    c = safe_fillet(c, c.edges().group_by(Axis.Y)[0 if y1 > y0 else -1], 0.4, "cpl")
    return c

tipL = P["DEPLOY_Y"] + P["N20_TIP"]; tipR = P["DEPLOY_Y"] - P["N20_TIP"]
yo = Y_C + HEAD_YL[1] - CB_DEPTH + 0.3                       # coupler end 0.3 proud of the counterbore floor: the cap seats on the coupler, the arm floats 0.7
yLi = P["DEPLOY_Y"] + P["N20_BOX"] / 2 + 0.3
cL = coupler(yLi, yo, True, tipL - yLi + 0.5)
yRi = P["DEPLOY_Y"] - P["N20_BOX"] / 2 - 0.3
cR = coupler(yRi, -yo, True, yRi - tipR + 0.5)
for nm, c in (("coupler_L", cL), ("coupler_R", cR)):
    add(nm, c.moved(Location((0, 0, 0))), "legs", "#444", pos=(P["DRILL_X"], 0, P["DRILL_PIV_Z"]), axis="y", deg=(0, P["DEPLOY_DEG"]),
        orient="lying on its D flat (layers along the axle), PPA-CF",
        note="Ø7 D coupler (PPA-CF): D-socket on the deploy N20 output stub, runs through the pivot block bore into the arm head; its outer end stands 0.3 proud in the head counterbore, the printed pod cap and an M3 x 8 screw seat on it (the arm floats 0.7, never clamped)")

# ================================================================== ENVELOPES (not printed)
def env(name, solid, color, note, parent=None):
    add(name, solid, "electronics", color, note=note, printable=False, parent=parent, step=False)

for i, s in enumerate((-1, 1)):
    env("cell_%d" % (i + 1), cylx(P["CELL_D"], P["CELL_L"] - 0.72, X_CELL[1] - 0.8 - P["CELL_L"], y=s * P["CELL_Y"]) + cylx(6, 0.72, X_CELL[1] - 0.8 - 0.72, y=s * P["CELL_Y"]),
        "#f4d35e", "18350 Ø18.66 x 36.87 incl. button (measured), 2S, contacts both ends")
px0 = SX0 + 1.3
env("n20_kick", bx(px0, px0 + P["N20_BOX"], KMY - 6, KMY + 6, KICK_Z - 6, KICK_Z + 6) + bx(px0 + P["N20_BOX"], px0 + P["N20_L"], KMY - 6, KMY + 6, KICK_Z - 5, KICK_Z + 5),
    "#8e9aaf", "KICK worm N20 (33.04 long), worm box at the nose end, beside the centreline; output shaft across Y = the kick pivot, its +y stub carries the hub")
add("kick_shaft", D_bore(P["SHAFT_D"], P["N20_TIP"] + KICK_STUB, KMY - KICK_STUB), "electronics", "#d0d4d8", parent="kick_leg", printable=False, step=False, note="kick N20 output shaft (turns with the leg): the +y stub carries the hub; the unused -y stub is cut to 1 mm past the worm box (it stays inside the saddle pocket)")
dxx, dzz, dy = P["DRILL_X"], P["DRILL_PIV_Z"], P["DEPLOY_Y"]
env("n20_deploy", bx(dxx - 6, dxx + 6, dy - 6, dy + 6, dzz - 6, dzz + 6) + bx(dxx - 6 - (P["N20_L"] - 12), dxx - 6, dy - 6, dy + 5, dzz - 5, dzz + 5),
    "#8e9aaf", "DEPLOY worm N20 on the axle line, body forward in the tray cradle")
add("deploy_shaft", D_bore(P["SHAFT_D"], 2 * P["N20_TIP"], dy - P["N20_TIP"]), "electronics", "#d0d4d8", pos=(dxx, 0, dzz), axis="y", deg=(0, P["DEPLOY_DEG"]), printable=False, step=False, note="deploy N20 output shaft (turns with the couplers), both stubs in the couplers")
for name in ("xiao", "drv8833_1", "drv8833_2", "ina219", "bno085", "mp1584", "vtx", "bms_2s"):
    lx, ly, lz = BOARD[name]; x, y, z = LAYOUT_ = LAY[name]
    env(name, bx(x, x + lx, y - ly / 2, y + ly / 2, z, z + lz), {"xiao": "#2a9d8f", "ina219": "#f3722c", "bno085": "#f8961e", "mp1584": "#577590", "vtx": "#9b5de5", "bms_2s": "#90be6d"}.get(name, "#43aa8b"),
        "%s %.2f x %.2f x %.2f (measured)" % (name, lx, ly, lz))
cx0 = P["NOSE_LIP"] + P["CAM_LENS"]
env("camera", bx(cx0, cx0 + P["CAM_D"] - P["CAM_LENS"], -P["CAM_W"] / 2, P["CAM_W"] / 2, -P["CAM_H"] / 2, P["CAM_H"] / 2) + cylx(P["CAM_LENS_D"], P["CAM_LENS"], cx0 - P["CAM_LENS"]),
    "#1d3557", "Caddx Ant 13.99 x 14.59, 18.34 front to back (measured)")
fx_p = FX0 + 4 + 1.3
env("n20_foot", foot_loc(FBOX, bx(fx_p, fx_p + P["N20_L"] - P["N20_BOX"], FOOT_YC - 5, FOOT_YC + 5, -5, 5) + bx(fx_p + P["N20_L"] - P["N20_BOX"], fx_p + P["N20_L"], FOOT_YC - 6, FOOT_YC + 6, -6, 6)),
    "#8e9aaf", "FOOT worm N20 in the housing, output up when folded (lower stub trimmed to 2 mm or single-shaft version)", parent="kick_leg")
add("foot_shaft", foot_loc(FBOX, cylz(P["SHAFT_D"], 9.0, 6.0, x=FBOX, y=FOOT_YC) - bx(FBOX - 3, FBOX - P["SHAFT_D"] / 2 + 0.5, FOOT_YC - 3, FOOT_YC + 3, 0, 20)), "electronics", "#d0d4d8", parent="kick_leg", printable=False, step=False, note="foot N20 output shaft, 9 proud, D flat facing the hull nose side")


# ================================================================== HARDWARE (every bought part, so the model has everything in it)
def hw(name, solid, color, note, parent=None, pos=(0, 0, 0), axis=None, deg=(0, 0)):
    add(name, solid, "electronics", color, note=note, printable=False, parent=parent, step=False, pos=pos, axis=axis, deg=deg)
def screw_y(y_head, y_tip, x, z, dhead=5.5, hh=1.8, flat=False):
    s = 1 if y_tip > y_head else -1
    if flat: y_head = y_head + s * CSINK_SUNK
    shank = cyly(2.9, abs(y_tip - y_head), min(y_head, y_tip), x=x, z=z)
    if flat:                                      # DIN 7991 countersunk head, flush with the skin
        head = Solid.make_cone(3.0, 1.45, 1.55, Plane(origin=(x, y_head + s * 0.05, z), z_dir=(0, s, 0)))
    else:
        head = cyly(dhead, hh, y_head - hh if s > 0 else y_head, x=x, z=z)
    return shank + head
xs1 = (X_D1[0] + X_D1[1]) / 2; xst = (X_TAILSP[0] + X_TAILSP[1]) / 2
scr = []
for (sy, zz) in ((+1, -P["RAD_Z"]), (+1, P["RAD_Z"]), (-1, P["RAD_Z"]), (-1, -P["RAD_Z"])):
    yh = sy * chord_half(zz, R) ; scr.append(screw_y(yh, yh - sy * 8, xs1, zz, flat=True))
for sy in (-1, 1):
    for zz in (-P["RAD_Z"], P["RAD_Z"]):
        yh = sy * chord_half(zz, R); scr.append(screw_y(yh, yh - sy * 8, xst, zz, flat=True))
s_all = scr[0]
for q in scr[1:]: s_all = s_all + q
hw("screws_radial", s_all, "#aab3bd", "8 x M3 x 8 countersunk (flat-head) radial screws, flush with the skin: 4 into tray disc 1, 4 through the tail cap spigot into the boom anchor disc (they carry the tow)")
xpin = L + P["BOOM_L"] - 5.0
hw("boom_pin", Solid.make_cone(1.45, 3.0, 1.6, Plane(origin=(xpin, 0, P["BOOM_W"] / 2 - 1.6), z_dir=(0, 0, 1))) + cylz(2.9, 8.0 - 1.6, P["BOOM_W"] / 2 - 8.0, x=xpin),
   "#555555", "M3 x 8 countersunk screw: the tether's wear pin in the boom eye (head flush in the top tine, threads 2 mm into the bottom tine, nothing proud; threadlocker)")
eb = cyly(P["BRG_FL_D"], P["BRG_FL_T"], BRG_EAR_Y + P["BRG_W"] - P["BRG_FL_T"], x=KICK_X, z=KICK_Z) + cyly(P["BRG_OD"], P["BRG_W"] - P["BRG_FL_T"], BRG_EAR_Y, x=KICK_X, z=KICK_Z) - cyly(3.0, 8, BRG_EAR_Y - 2, x=KICK_X, z=KICK_Z)
hw("brg_ear", eb, "#9aa5b1", "F693ZZ in the ear, flange down in a counterbore on the outer face, inner face 0.4 proud of the fin onto the hub's inner-race boss")
yl0 = P["KICK_HUB_W"] / 2
hw("kick_hub_screw", screw_y(BRG_EAR_Y + P["BRG_W"] - KICK_Y, BRG_EAR_Y + P["BRG_W"] - KICK_Y - 8.0, 0, 0), "#555555", "M3 x 8 through the ear bearing into the hub end (turns with the leg; its tip stops 0.4 short of the kick N20 shaft end); the head clamps the inner race onto the hub's printed boss", parent="kick_leg")
CAP_T = 1.5
for nm, sg in (("coupler_L", 1), ("coupler_R", -1)):
    yo2 = sg * yo                                            # coupler outer end face
    cap = cyly(10.0, CAP_T, yo2 if sg > 0 else yo2 - CAP_T) - cyly(P["SCREW_CLR"], CAP_T + 2, (yo2 if sg > 0 else yo2 - CAP_T) - 1)
    cap = safe_fillet(cap, cap.edges().group_by(Axis.Y)[-1 if sg > 0 else 0], 0.4, "cap")
    add("pod_cap_" + nm[-1], cap, "legs", "#444", parent=nm, orient="flat on the bed",
        note="Printed pod cap Ø10 x 1.5: sits on the coupler end inside the pod counterbore, the M3 x 8 screw goes through it into the coupler; it overlaps the pod head by 1.4 mm so the pod cannot walk off the coupler (an M3 washer, Ø7, is smaller than the Ø7.25 pod bore and would not hold it)")
    yh = yo2 + sg * CAP_T
    hw(nm + "_screw", screw_y(yh, yh - sg * 8.0, 0, 0), "#555555", "M3 x 8 through the pod cap into the coupler end (turns with it)", parent=nm)
cy_ = P["CELL_Y"]
spr = None
for sy in (-1, 1):
    sp = cylx(9.0, 3.0, X_CELL[0] - 0.8 + 0.01, y=sy * cy_)
    spr = sp if spr is None else spr + sp
spr = spr + bx(X_CELL[0] - 0.8, X_CELL[0] - 0.3, -cy_, cy_, -1, 1)
hw("contacts_front_springs", spr, "#c0c0c0", "2xAA-holder springs joined by wire 2 (front of the cell bay)")
pl = bx(X_CELL[1] - 0.6, X_CELL[1] + 0.2 - 0.01, cy_ - 5.5, cy_ + 5.5, -5.5, 5.5) + bx(X_CELL[1] - 0.6, X_CELL[1] + 0.2 - 0.01, -cy_ - 5.5, -cy_ + 5.5, -5.5, 5.5)
hw("contacts_rear_plates", pl, "#c0c0c0", "flat contact plates at the back of the cell bay (B- cell 1, B+ cell 2)")
xc_ = X_TAILSP[1] + 0.8
jx0, jx1 = JST_TOP_X[0] + 0.17, JST_TOP_X[1] - 0.17
hw("jst_charge", bx(jx0, jx1, -4.95, 4.95, -19.0, -12.0) + bx(jx0 + 1.0, jx1 - 1.0, -3.8, 3.8, -12.0, -9.6), "#f5f5f5",
   "JST-XH 3-pin charge socket (B-, BM, B+), header body 5.75 x 9.9 x 7.0 plus pins, recessed in the tail cap bottom seat; plug in on the bench only")
hw("jst_loop", bx(jx0, jx1, -3.7, 3.7, 6.5, 13.5) + bx(jx0 + 1.0, jx1 - 1.0, -2.6, 2.6, 4.2, 6.5) + bx(jx0, jx1, -3.7, 3.7, 13.5, 19.6), "#f5f5f5",
   "JST-XH 2-pin power loop socket (header 5.75 x 7.4 x 7.0, pins trimmed to 2.3 after soldering: the boom shank runs under them) with the shorting plug and its wire loop on top (ends at z 19.6, under the skin at 20.7): the ON switch, top of the tail cap")
hw("antenna", cylx(4.0, 22, X_TAILSP[0] - 20, z=12.8), "#222222", "VTX u.FL 5.8 GHz antenna along the top through the disc 4 notch, tip ahead of the power socket seat (PLA is transparent to 5.8 GHz)")
hw("xiao_antenna", bx(X_BOARD[0] + 3.0, X_BOARD[0] + 23.0, -9.5, 2.5, 16.5, 16.9), "#c8a000",
   "XIAO 2.4 GHz FPC antenna (comes with the XIAO ESP32S3; about 20 x 12, NOT measured): stick it flat to the inside of the hull top over the board bay, u.FL to the XIAO")
cx_ = SLED_X0 + 0.5
hw("capacitor", cylx(6.3, 11.0, cx_, y=9.4, z=2.6), "#334488", "470 to 1000 uF across the MP1584 5 V output (long leg to OUT+)")
# ballast: 15 g of steel shot + epoxy, filling the nose cavity from the tip
nl_ = P["NOSE_L"]
env_ = Solid.make_cone(P["NOSE_R0"], R, nl_, Plane.YZ) + cylx(2 * RB - 2 * CLR, P["NOSE_SPIG"], nl_ - 0.01)
cav = env_ - FIX["nose"] - bx(P["NOSE_LIP"] + P["CAM_LENS"] - 0.2, nl_ + P["NOSE_SPIG"] + 5, -(P["CAM_W"] / 2 + 1.6), P["CAM_W"] / 2 + 1.6, -(P["CAM_H"] / 2 + 1.6), P["CAM_H"] / 2 + 1.6) - cylx(P["CAM_HOLE"] + 0.4, 6, -1)
target = max(P["BALLAST_G"], 0.0) / 5.0 * 1000          # g at ~5 g/cm3 for shot in epoxy -> mm3
lo, hi = 2.0, nl_ + P["NOSE_SPIG"]
for _ in range(18):
    mid = (lo + hi) / 2; v = (cav & bx(-1, mid, -30, 30, -30, 30)).volume
    lo, hi = (mid, hi) if v < target else (lo, mid)
if P["BALLAST_G"] > 0:
    ball = cav & bx(-1, hi, -30, 30, -30, 30)
    P["BALLAST_X_END"] = round(hi, 1)
    hw("ballast", ball, "#7a6a55", "%g g nose ballast: steel shot or M6 nuts in 5-minute epoxy, poured with the nose standing on its tip (fills to x %.1f)" % (P["BALLAST_G"], hi))

mid = lambda xx: round((xx[0] + xx[1]) / 2, 2)
P["WIRE_PTS"] = dict(           # harness way points (wires.py, wireroom.py; firmware/dart/tools/wires_v4.py may read them)
    chase=[X_BOARD[1] - 4.3, 0, -8], keel_d2=[mid(X_D2), 0, -13], keel_d3=[mid(X_D3), 0, -13], keel_d4=[mid(X_D4), 0, -13],
    sled=[58.5, 0, -10], win_d1=[mid(X_D1), P["DEPLOY_Y"], 0], win_d4=[mid(X_D4), 0, 0],
    top_d1=[mid(X_D1), 4.0, 17], top_d2=[mid(X_D2), 0, 17], top_d3=[mid(X_D3), 0, 17],
    tail_top=[mid(JST_TOP_X), 0, 4.0], tail_bot=[mid(JST_TOP_X), 0, -9.5],
    flankL=[P["DRILL_X"] + 10.5, 11.5, 16.5], flankR=[P["DRILL_X"] + 10.5, -11.5, 16.5],
    kick_hole=[SX0 + 1.3 + P["N20_L"] + 0.3, KMY, 17.0], foot_hole=[42.0, 2.0, 17.0],
    deploy_tabs=[P["DRILL_X"] - 6 - (P["N20_L"] - 12) - 0.8, P["DEPLOY_Y"], P["DRILL_PIV_Z"]])
P["X_BOARD_BAY"] = list(X_BOARD); P["JST_TOP_X"] = list(JST_TOP_X)
json.dump(dict(version=os.environ.get("KV", "v27"), P=P, ground=GROUND, parts=parts), open(os.path.join(OUT, "parts.json"), "w"), indent=1)
print("parts:", len(parts), "L =", L, "t =", round(time.time() - T0), "s")
