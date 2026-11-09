"""
KEDGE dart v34 generator (build123d).  Run: KV=v34 python3 kedge_v34.py out34   (KOV='{json}' overrides any parameter)
v34 = the self-righting layout (Harry, 9 Nov 2026, from the righting study kedge/sim_righting):
- drill pods at the REAR (pivot x 132, 10 above the axis), folded FORWARD along the flanks, swinging down 50 deg to a carved
  hard stop, so the drills enter the soil near the tail under the tether; v33 tapered gimlet augers (mirrored, hands kept);
- tether: a low bridle to an eye lug on each pod's arm head (no tail boom, no tail cap: the hull halves close the tail);
- a TOP RIGHTING LEG (no kick foot, no foot motor): a slim blade flush in a groove along the hull top, hinged on the
  centreline near the nose (x 33, z 14.5) on two couplers driven by its own worm N20 under a flush knuckle, an arch pad
  flush on the tail top, swing 0 to 178 deg (the far rest is the leg lying in the groove forward of the hinge);
- low centre of mass: the two cells in line on the keel, both worm N20s pointing down, boards on decks over the cells.
Frame (mm): X along the hull, nose tip x=0, tail +X.  Y across (+Y = left), Z up.  Dart lying flat => ground z=-R_OUT.
Moving parts are built in their own LOCAL frame (pivot at origin, hinge axis = Y) and placed by parts.json:
world = Translate(pos) * RotY(sgn * deg) * local.  Both moving groups here use sgn -1 (the leg swings up over the nose,
the pods swing down from pointing forward).
Outputs per part: STL (print), STEP (B-rep), .npz (exact tessellation for the sweep), plus parts.json.
"""
import json, math, os, sys, time
import numpy as np
from build123d import *
import build123d as _bd
import trimesh

OUT = sys.argv[1] if len(sys.argv) > 1 else "out34"
os.makedirs(OUT, exist_ok=True)
T0 = time.time()

# ------------------------------------------------------------------ parameters (measured values marked M)
P = dict(
    R_OUT=21.0, R_BAY=19.5, WALL=1.5, L_HULL=152.0, END_T=2.5,
    NOSE_L=20.0, NOSE_R0=11.0, NOSE_SPIG=6.6,
    # camera  M: 13.99 w x 14.59 h, 18.34 lens front to connector back (CAM_Z: 2.2 below the axis, under the leg's nose flute)
    CAM_W=13.99, CAM_H=14.59, CAM_D=18.34, CAM_LENS_D=8.0, CAM_LENS=2.7, CAM_HOLE=9.8, NOSE_LIP=1.2, CAM_Z=-2.4,
    # worm N20  M: body 12 x 10, worm box 9.94 (along the output shaft) x 11.93, 33.04 long box+motor, shaft 29.97 tip to tip
    N20_L=33.04, N20_BOX=12.0, N20_BOXW=9.94, N20_BOXH=11.93, N20_CAN=(12.0, 10.0), N20_TIP=29.97 / 2, SHAFT_D=3.0,
    # righting leg
    LEG_X=33.0, LEG_Z=14.5, LEG_DEG=178.0, LEG_REST_DEG=1.5, LEG_STUB=10.8,    # hinge; swing; far rest 1.5 deg past; N20 stubs trimmed to |y| 10
    LEG_W=8.5, LEG_W_TIP=6.6, LEG_FLARE=14.0, LEG_U0=0.8, LEG_U1=3.4, LEG_UX=78.0,                            # blade width, underside (local z) at the root / the rear, and where it rises
    LEG_RB=10.2, LEG_CHEEK_R=5.6, CH_Y0=7.8, CH_Y1=11.2, PK_Y0=7.6, PK_Y1=11.4,   # bridge inner radius, cheeks, knuckle walls, pockets
    LEG_CPL_D=6.0, LEG_PLUG_D=7.5, PAD_X0=143.5, PAD_X1=149.3, PAD_DEG=62.0, PAD_T=2.0,        # couplers; arch pad on the tail top
    # drill pair (rear)
    DRILL_X=132.0, DRILL_PIV_Z=9.5, DRILL_PIV_Y=None, DEPLOY_DEG=50.0, DEPLOY_TARGET=50.0, HORN_GAP=0.1, HEAD_IN=21.6,
    CPL_D=7.0, BOSS_D=11.0, EYE_FWD=3.9, EYE_DROP=6.8, EYE_D=3.2,     # bridle eye on the arm head: forward of and below the pivot when folded (stays above the launcher stop springs at z -1.1)
    # drill motor  M: gearbox 11.95, 24.27 face to back, shaft 2.84 x 56.03
    MOTOR_W=12.0, MOTOR_H=10.0, MOTOR_L=24.27, DSHAFT_L=56.03, DSHAFT_D=2.84,
    AUGER_L=100.0, AUGER_D=13.0, AUGER_CORE=6.0, AUGER_PITCH=11.0, AUGER_TIP=12.0,
    SLEEVE_BORE=16.3, SLEEVE_WALL=1.2, MOTOR_GB_L=9.0,
    # v33 tapered drill (provisional: Harry is bench-testing printed bits T12/T16/T20)
    FLIGHT_D=16.0, D_PITCH=12.0, D_CORE0=9.6, D_CORE1=6.0, D_TAPER=14.0, D_TAPER_F=22.0, D_TIP_R=0.3,
    D_LOAD_DEG=45.0, D_LEAD_DEG=30.0, D_LAND=0.6, D_SPINE_BORE=2.6, D_SPINE_WALL=0.8, D_NECK=9.0,
    NUT_AF=5.5, NUT_T=2.4, NYLOC_T=4.0, BUSH_GAP=0.6, POD_PS=4.4, DTERM=1.2, BOSS_FACE_D=4.6,
    BRG_OD=8.0, BRG_W=4.0, BRG_FL_D=9.5, BRG_FL_T=0.6,
    # power and boards (M)
    CELL_D=18.66, CELL_L=36.87, CELL_Z=-10.0, CONTACT=3.6, MIDPLATE=0.3,
    SCREW_CLR=3.3, SCREW_PILOT=2.5, RAD_Z=3.0,
    TERM=1.6, BALLAST_G=14.6, BAL_RHO=9.0, BAL_ZTOP=3.0,                # nose ballast putty (g; putty density g/cm3: tungsten about 9 to 11, lead about 7); pocket top z
    # dart-cad5 review fixes
    SOCK_D=3.05, SOCK_FLAT=1.03,           # press-fit D socket on the N20 stubs (Ø3.0, flat at 1.0 from the axis)
    CPL_BORE=7.05, LEG_CPL_BORE=6.05,      # press-fit D bores: arm head on the Ø7 pod coupler, cheek on the Ø6 leg coupler
    SCR_X1=39.8, BOSS_T=6.0, SCREW_L=8.0,  # disc-1 screw station (clear of the XIAO), screw bosses 6 thick, M3 x 8 countersunk
    ROPE_TH=65.0, ROPE_END=(4.2, 3.2), ROPE_D=1.2, GROOVE_W=1.6, GROOVE_D=1.4, BRIDLE_L=78.0,   # bridle grooves in the tail
    LOOP_DZ=-1.2,                          # left (ON loop) socket seat lowered so the bridle leg passes 3 mm over the plug + wire loop
    CAP_D=8.0, CAP_L=12.0,                 # electrolytic can modelled at the worst likely size (470 to 1000 uF)
)
P.update(LAYOUT="cells_inline", LIGHT=0)
P.update(json.loads(os.environ.get("KOV", "{}")))
P["KICK_DEG"] = P["LEG_DEG"]                       # alias read by older tools (massprops, viewer): the leg angle
R, RB, W = P["R_OUT"], P["R_BAY"], P["WALL"]
HUB_D = P["SLEEVE_BORE"] + 2 * P["SLEEVE_WALL"]
if P["DRILL_PIV_Y"] is None:
    P["DRILL_PIV_Y"] = round(R + max(P["FLIGHT_D"], HUB_D) / 2 + 0.8, 2)
GROUND = -R
CLR = 0.2
L = P["L_HULL"]

# x stations along the hull (v34: leg N20 | disc 1 | cells in line | rear disc | deploy N20 | tail)
X_NOSE = P["NOSE_L"]
X_SPIG1 = X_NOSE + P["NOSE_SPIG"]                        # nose spigot ends (26)
LX, LZ = P["LEG_X"], P["LEG_Z"]
X_LCR = (LX - 5.0 - 0.2 - 1.0, LX + 5.0 + 0.2 + 1.2)    # leg N20 cradle tube (motor can 10 along x; front wall 1.0, just behind the nose spigot)
X_D1 = (X_LCR[1], X_LCR[1] + 3.0)                        # disc 1 (4 radial screws), front contact springs on its rear face
X_C1 = (X_D1[1] + P["CONTACT"], X_D1[1] + P["CONTACT"] + P["CELL_L"])
X_MID = (X_C1[1], X_C1[1] + P["MIDPLATE"])               # BM plate web between the cells
X_C2 = (X_MID[1], X_MID[1] + P["CELL_L"])
X_D2 = (X_C2[1] + 1.0, X_C2[1] + 3.0)                    # rear disc (4 radial screws), rigid plates on its front face
DX = P["DRILL_X"]
X_DCR = (DX - 5.0 - 0.2 - 1.2, DX + 5.0 + 0.2 + 1.2)    # deploy N20 cradle tube
X_END = L - P["END_T"]                                    # inside face of the tail end wall
X_SOCK = (P["PAD_X0"] + 0.15, P["PAD_X0"] + 0.15 + 5.75 + 0.3)   # flank socket seats (JST-XH header 5.75 along x)
assert X_D2[1] < DX - P["N20_BOXH"] / 2 - 0.3, ("cells collide with the deploy N20", X_D2, DX)
P.update(L=L, X_D1=X_D1, X_C1=X_C1, X_MID=X_MID, X_C2=X_C2, X_D2=X_D2, X_LCR=X_LCR, X_DCR=X_DCR, X_SOCK=X_SOCK)
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

def safe_chamfer(solid, edges, d, tag):
    try:
        out = chamfer(edges, d)
        if out.is_valid:
            return out
    except Exception as e:
        print("  chamfer skipped:", tag, str(e)[:60])
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

def _extra(sgn, joint, mass_g):
    e = {}
    if sgn is not None: e["sgn"] = sgn
    if joint is not None: e["joint"] = joint
    if mass_g is not None: e["mass_g"] = mass_g
    return e

def add(name, solid, group, color, pos=(0, 0, 0), axis=None, deg=(0, 0), note="", printable=True, parent=None,
        orient=None, step=True, tol=0.02, sgn=None, joint=None, mass_g=None):
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
                          watertight=bool(m.is_watertight), nsolids=len(m.split(only_watertight=False)), **_extra(sgn, joint, mass_g)))
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
                      watertight=bool(m.is_watertight), nsolids=len(solid.solids()) if hasattr(solid, "solids") else 1, **_extra(sgn, joint, mass_g)))
    FIX[name] = solid
    print(f"{name:20s} {np.round(ext,1)} vol {solid.volume/1000:6.2f} cm3 wt={m.is_watertight} solids={len(solid.solids())} t={time.time()-T0:.0f}s")

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
    return cyly(5.0, abs(yo - y_in) + 1, y_in if s > 0 else yo - 1, x=xs, z=zz)
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

# ---- augers: trimesh + manifold (the swept helix does not survive OCC booleans; STEP of augers is faceted)
from trimesh.creation import cylinder as tcyl, cone as tcone
from trimesh.transformations import rotation_matrix as RM, translation_matrix as TM, reflection_matrix
def tU(ms): return trimesh.boolean.union(ms, engine="manifold")
def tD(a, bs): return trimesh.boolean.difference([a] + list(bs), engine="manifold")
def tcylx(d, l, x0, y=0, z=0, n=96):
    c = tcyl(radius=d / 2, height=l, sections=n); c.apply_transform(RM(math.pi / 2, [0, 1, 0])); c.apply_transform(TM([x0 + l / 2, y, z])); return c
def tbox(x0, x1, y0, y1, z0, z1):
    b = trimesh.creation.box(extents=[x1 - x0, y1 - y0, z1 - z0]); b.apply_transform(TM([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])); return b

# ---- v33 tapered augers: revolved body + swept buttress flight in manifold3d (built along +z, turned to +x)
import manifold3d as m3
def m3_of(tm): return m3.Manifold(m3.Mesh(vert_properties=np.asarray(tm.vertices, np.float32), tri_verts=np.asarray(tm.faces, np.uint32)))
def tm_of(man):
    me = man.to_mesh(); v = np.asarray(me.vert_properties)[:, :3]; t = np.asarray(me.tri_verts)
    mf, mt = np.asarray(me.merge_from_vert, np.int64), np.asarray(me.merge_to_vert, np.int64)
    if len(mf): rm = np.arange(len(v)); rm[mf] = mt; t = rm[t]                  # apply manifold's merge vectors (seam duplicates)
    m = trimesh.Trimesh(v, t, process=False); m.remove_unreferenced_vertices(); return m   # no tolerance merge: it pinches thin slivers
def fillet_pts(p0, p1, p2, rad, n=12):
    """arc of radius rad replacing the corner p1 of the polyline p0-p1-p2 (2D)"""
    p0, p1, p2 = (np.array(q, float) for q in (p0, p1, p2))
    u = (p0 - p1) / np.linalg.norm(p0 - p1); v = (p2 - p1) / np.linalg.norm(p2 - p1)
    a = math.acos(max(-1.0, min(1.0, float(u @ v)))) / 2
    if a > math.pi / 2 - 1e-3: return [tuple(p1)]
    t = rad / math.tan(a); t1, t2 = p1 + u * t, p1 + v * t
    b = (u + v) / np.linalg.norm(u + v); c = p1 + b * rad / math.sin(a)
    a1, a2 = math.atan2(*(t1 - c)[::-1]), math.atan2(*(t2 - c)[::-1])
    da = (a2 - a1 + math.pi) % (2 * math.pi) - math.pi
    return [tuple(c + rad * np.array([math.cos(a1 + da * k / n), math.sin(a1 + da * k / n)])) for k in range(n + 1)]
def round_poly(pts, radii):
    """pts: list of (r, z); radii: {index: radius} corners to round"""
    out = []
    for i, p in enumerate(pts):
        if i in radii and 0 < i < len(pts) - 1: out += fillet_pts(pts[i - 1], p, pts[i + 1], radii[i])
        else: out.append(tuple(p))
    return out
def revolve(pts, segs=144):
    cs = m3.CrossSection([pts])
    if cs.area() < 0: cs = m3.CrossSection([pts[::-1]])
    return m3.Manifold.revolve(cs, segs)
def buttress_flight(z0, z1, pitch, r_body, r_out, load_deg=45.0, lead_deg=30.0, land=0.6, lefthand=False, steps_per_turn=128, embed=0.4, fade=0.25, nroot=6):
    """anchor (buttress) flight on the z axis, crest centre line z0..z1: quad profile in (r, z) swept along the helix. The load face
    (toward -z, the motor) is load_deg off radial: 45 deg prints tip up without supports; the leading face (toward the point) lead_deg.
    The roots sit `embed` inside the body r_body(z) at their own z; the crest r_out(z); depth fades in/out over `fade` turns."""
    turns = (z1 - z0) / pitch; n = max(8, int(turns * steps_per_turn))
    tl, tc = math.tan(math.radians(load_deg)), math.tan(math.radians(lead_deg)); rings = []
    for i in range(n + 1):
        t = i / n; th = 2 * math.pi * turns * t * (-1 if lefthand else 1); zc = z0 + (z1 - z0) * t
        rc = r_body(zc) - embed; f = min(1.0, turns * t / fade, turns * (1 - t) / fade)
        ro = rc + 0.3 + max(0.0, r_out(zc) - rc - 0.3) * f; d = ro - rc
        za, zb = zc - land / 2 - d * tl, zc + land / 2 + d * tc
        ra, rb = min(r_body(za) - embed, ro - 0.25), min(r_body(zb) - embed, ro - 0.25)
        c, s_ = math.cos(th), math.sin(th)
        root = [(min(r_body(z_) - embed, ro - 0.25), z_) for z_ in np.linspace(zb, za, nroot + 2)[1:-1]]   # root follows the body (no tunnel over a concave corner)
        rings.append([(r * c, r * s_, z) for r, z in [(ra, za), (ro, zc - land / 2), (ro, zc + land / 2), (rb, zb)] + root])
    q = 4 + nroot; V = np.array([p for ring in rings for p in ring], float); F = []
    for i in range(n):
        a_, b_ = q * i, q * (i + 1)
        for k in range(q):
            k2 = (k + 1) % q; F += [[a_ + k, b_ + k, b_ + k2], [a_ + k, b_ + k2, a_ + k2]]
    e = q * n
    for k in range(1, q - 1): F += [[0, k + 1, k], [e, e + k, e + k + 1]]
    tm = trimesh.Trimesh(V, np.array(F), process=True); trimesh.repair.fix_winding(tm); trimesh.repair.fix_normals(tm)
    if tm.volume < 0: tm.invert()
    assert tm.is_watertight and tm.is_winding_consistent, "flight mesh not closed"
    return m3_of(tm)
Z2X = RM(math.pi / 2, [0, 1, 0])                                     # built along +z -> along +x (a proper rotation: hands kept)



def mirror_x(s):
    """build123d solid built for a pod pointing +x -> the same pod pointing -x (forward); local origin = pivot"""
    return s.mirror(Plane.YZ)

def mirror_x_tm(m):
    m = m.copy(); m.apply_transform(np.diag([-1.0, 1.0, 1.0, 1.0]))
    if m.volume < 0: m.invert()
    return m

# ================================================================== DRILL ARM + AUGER layout (built in v33's local frame: pivot at origin,
# auger along local +x; then mirrored so the pod points FORWARD (-x) when folded)
HH = 12.8
Y_C = P["DRILL_PIV_Y"]
POD_FACE = P["HEAD_IN"]
HEAD_YL = (POD_FACE - Y_C, 3.0)
CB_D, CB_DEPTH = 10.4, 1.2
CORE_D = HUB_D
ASH_X0, ASH_X1 = 7.0, 8.5
S0 = ASH_X1 + 0.2
BW1 = ASH_X1 + 1.5
MB = BW1 + P["DTERM"]
GB0 = MB + P["MOTOR_L"] - P["MOTOR_GB_L"]
XS = MB + P["MOTOR_L"]
SHAFT_END = XS + P["DSHAFT_L"]
BUSH_X1 = GB0 - 0.3; BUSH_D = P["SLEEVE_BORE"] - 0.5
R_BORE = P["SLEEVE_BORE"] / 2
X_ROOF0 = XS + 0.3
HEX_R = (P["NUT_AF"] + 0.35) / math.sqrt(3)
X_NA = (X_ROOF0 + R_BORE - HEX_R, X_ROOF0 + R_BORE - HEX_R + P["NUT_T"] + 0.2)
XE0 = XS + 1.0
XR = XE0 + P["D_NECK"]
SPINE_END = SHAFT_END + 0.5
_rc1, _rt, _rs = P["D_CORE1"] / 2, P["D_TIP_R"], P["D_SPINE_BORE"] / 2 + P["D_SPINE_WALL"]
XT0 = SPINE_END - (_rc1 - _rs) / (_rc1 - _rt) * P["D_TAPER"]
TIP = XT0 + P["D_TAPER"]
XC0 = TIP - P["D_TAPER_F"]
P["AUGER_L"] = round(TIP - S0, 2); P["AUGER_TIP"] = P["D_TAPER"]
P.update(POD_XS=XS, POD_S0=S0, POD_TIP=round(TIP, 2), CORE_D=round((P["D_CORE0"] + P["D_CORE1"]) / 2, 2), HUB_D=HUB_D, SHAFT_END=SHAFT_END,
         D_HOLD=[round(XR, 2), round(XC0, 2)], D_POINT_PAST_SPINE=round(TIP - SHAFT_END, 2), POD_DIR=-1)
EYE_Y = (HEAD_YL[1] - 5.0, HEAD_YL[1])                   # bridle eye lug: the outboard 5 mm of the arm head (left pod local y)

# ================================================================== RIGHTING LEG geometry (local frame: hinge at origin, folded blade along +x)
HZ = LZ                                                   # hinge height above the hull axis
def z_top(y):                                             # hull crown height in the leg's local frame
    return math.sqrt(max(R * R - y * y, 0.0)) - HZ
U0, U1, UX = P["LEG_U0"], P["LEG_U1"], P["LEG_UX"]
LEG_PX0, LEG_PX1 = P["PAD_X0"] - LX, P["PAD_X1"] - LX     # pad in local x
def leg_under(x):                                         # blade underside (local z) along the blade
    return U0 if x <= UX else (U1 if x >= UX + 13.0 else U0 + (U1 - U0) * (1 - math.cos(math.pi * (x - UX) / 13.0)) / 2)
BR_X0, BR_X1 = 8.0, 21.0                                  # bridge: full width between the tines, then narrowing to the blade
TINE_C = (11.0, 3.0, 3.0)                                 # tine: tangent arcs from the cheek circle to this circle (local x, z, r)

def section_yz(y0, y1, zb, n=24, zt_off=0.0, grow=0.0):
    """leg cross-section in the y-z plane: flat bottom zb, top = the hull crown arc (+zt_off), |y| from y0 to y1"""
    ys = np.linspace(y0 - grow, y1 + grow, n)
    pts = [(y0 - grow, zb - grow), (y1 + grow, zb - grow)] + [(yy, z_top(min(abs(yy), R - 0.5)) + zt_off + grow) for yy in ys[::-1]]
    return pts

def prism_x(pts_yz, x0, x1):
    """polygon in the y-z plane extruded along +x from x0 to x1"""
    face = Plane.YZ * Polygon(*pts_yz, align=None)
    sol = extrude(face, amount=x1 - x0)
    bb = sol.bounding_box()
    return Pos(x0 - bb.min.X, 0, 0) * sol

def blade_hw(x):
    """blade half width along local x: full LEG_W at the root, tapering to LEG_W_TIP/2, then a tangent flare into the pad"""
    h0, h1, hp = P["LEG_W"] / 2, P["LEG_W_TIP"] / 2, P["LEG_FLARE"] / 2
    xa, xb, xc = 30.0, LEG_PX0 - 14.0, LEG_PX0 + 0.6
    if x <= xa: return h0
    if x <= xb: return h0 + (h1 - h0) * (1 - math.cos(math.pi * (x - xa) / (xb - xa))) / 2
    return h1 + (hp - h1) * (1 - math.cos(math.pi * min(1.0, (x - xb) / (xc - xb)))) / 2

def blade_plan(off, x0, x1, n=80):
    """solid: the blade's plan outline offset by off, extruded through the leg's height (local frame)"""
    xs = np.linspace(x0, x1, n)
    pts = [(x, -(blade_hw(x) + off)) for x in xs] + [(x, blade_hw(x) + off) for x in xs[::-1]]
    return extrude(Plane.XY.offset(-30) * Polygon(*pts, align=None), amount=60)

def build_leg():
    """Righting leg, one printed part (PLA): two round cheeks (on the two couplers, in the side pockets), tines curving aft to a
    bridge behind the knuckle, a slim blade whose top IS the hull crown (flush in the top groove) and a thin arch pad that is a
    patch of the hull skin on the tail top. Everything in the knuckle zone (|y| < pocket) stays outside r LEG_RB of the hinge,
    so it swings over the knuckle."""
    hw = P["LEG_W"] / 2
    blade = prism_x(section_yz(-P["LEG_FLARE"] / 2 - 0.5, P["LEG_FLARE"] / 2 + 0.5, U0), BR_X0, LEG_PX0 + 0.6) & blade_plan(0.0, BR_X0 - 1, LEG_PX0 + 0.6)
    pts = [(UX + 13.0 * k / 12, leg_under(UX + 13.0 * k / 12)) for k in range(13)]
    rise = prism_xz([(UX - 0.01, U0 - 3)] + pts + [(LEG_PX0 + 2, U1), (LEG_PX0 + 2, U0 - 3)], -P["LEG_FLARE"] / 2 - 1, P["LEG_FLARE"] / 2 + 1)
    blade = blade - rise
    # I-section lightening: the blade is an inverted channel (1.3 skin on top, 1.5 side walls) where it is deep enough; open to the groove floor
    ch_pts = [(x_, leg_under(x_) - 1.0) for x_ in np.linspace(24.0, 104.0, 41)] + [(104.0, z_top(0) - 1.3), (24.0, z_top(0) - 1.3)]
    chan = prism_xz(ch_pts, -hw, hw) & blade_plan(-1.5, 20.0, 108.0)
    chan = safe_fillet(chan, [e for e in chan.edges().filter_by(Axis.Y)], 0.8, "channel ends")
    blade = blade - chan
    # bridge: full width at the tines, narrowing to the blade with tangent S-curves in plan (x-y)
    bw = P["CH_Y1"]
    brs = prism_x(section_yz(-bw, bw, U0), BR_X0, BR_X1 + 7.0)
    plan_sol = extrude(Plane.XY.offset(-5) * Polygon(*plan_pts(bw, hw), align=None), amount=20)
    bridge = brs & plan_sol
    body = blade + bridge
    body = body - cyly(2 * P["LEG_RB"], 2 * P["PK_Y0"] + 0.2, -P["PK_Y0"] - 0.1)                # clear of the knuckle (swings over it)
    # cheeks + tines (both sides)
    for s in (1, -1):
        ya, yb = sorted((s * P["CH_Y0"], s * P["CH_Y1"]))
        ch = cyly(2 * P["LEG_CHEEK_R"], yb - ya, ya)
        tine = arc_prism([(0.0, 0.0, P["LEG_CHEEK_R"]), TINE_C], ya, yb)
        ct = yfillet(ch + tine, (ya, yb), 0.6, "cheek faces")
        body = body + ct
    # arch pad: a patch of the hull skin (outer r 21, PAD_T thick) over +-PAD_DEG on the tail top, edges rounded
    pa = math.radians(P["PAD_DEG"])
    arc = (cylx(2 * R - 0.04, LEG_PX1 - LEG_PX0, LEG_PX0, z=-HZ) - cylx(2 * (R - P["PAD_T"]), LEG_PX1 - LEG_PX0 + 2, LEG_PX0 - 1, z=-HZ))
    wedge = prism_x([(0, -HZ), (-30 * math.sin(pa), -HZ + 30 * math.cos(pa)), (0, -HZ + 30), (30 * math.sin(pa), -HZ + 30 * math.cos(pa))], LEG_PX0 - 1, LEG_PX1 + 1)
    pad = arc & wedge
    pad = safe_fillet(pad, pad.edges(), 0.9, "pad edges")
    body = body + pad
    # coupler D-bores through the cheeks (flat toward local +z): press fit on the Ø6 leg coupler (CA)
    for s in (1, -1):
        ya, yb = sorted((s * P["CH_Y0"], s * P["CH_Y1"]))
        body = body - D_bore_y(P["LEG_CPL_BORE"], ya - 1, yb + 1)
    return body

def plan_pts(bw, hw):
    """bridge plan outline (x, y): full width bw from BR_X0 to 13, tangent S-curves down to the blade half width hw by BR_X1"""
    out = [(BR_X0 - 0.5, -bw), (13.0, -bw)]
    n = 16
    for k in range(1, n + 1):
        t = k / n; xx = 13.0 + (BR_X1 - 13.0) * t; yy = bw + (hw - bw) * (1 - math.cos(math.pi * t)) / 2
        out.append((xx, -yy))
    out += [(BR_X1 + 8.0, -hw), (BR_X1 + 8.0, hw)]
    for k in range(n, 0, -1):
        t = k / n; xx = 13.0 + (BR_X1 - 13.0) * t; yy = bw + (hw - bw) * (1 - math.cos(math.pi * t)) / 2
        out.append((xx, yy))
    out += [(13.0, bw), (BR_X0 - 0.5, bw)]
    return out

def D_bore_y(d, y0, y1, flat=None):
    """D bore along y, flat at +z (0.5 deep, or `flat` from the axis)"""
    c = cyly(d, y1 - y0, y0)
    return c - bx(-d, d, y0 - 1, y1 + 1, d / 2 - 0.5 if flat is None else flat, d)

def leg_front_solid(g):
    """the leg's blade root (local x < 36, |y| <= the blade half width), grown by g, for the forward bed"""
    hw = P["LEG_W"] / 2
    blade_f = prism_x(section_yz(-hw, hw, U0, grow=g), BR_X0 - g, 36.0)
    return blade_f - cyly(2 * (P["LEG_RB"] - g), 2 * (P["PK_Y0"] - g) - 0.02, -(P["PK_Y0"] - g) + 0.01)

def leg_profile_xz(kind, grow=0.0):
    """shapely x-z profiles of the leg (local frame) for the hull sweeps: 'tine' = cheek circle + tine; 'bridge' = the bridge
    (between local z U0 and the crown, from r LEG_RB out to BR_X1)"""
    from shapely.geometry import Point, Polygon as SP
    from shapely.ops import unary_union
    if kind == "tine":
        g = unary_union([Point(0, 0).buffer(P["LEG_CHEEK_R"], 128), Point(TINE_C[0], TINE_C[1]).buffer(TINE_C[2], 64)]).convex_hull
    elif kind == "bridge_centre":
        g = SP([(BR_X0 - 0.5, U0), (BR_X1 + 8.0, U0), (BR_X1 + 8.0, z_top(0) + 0.15), (BR_X0 - 0.5, z_top(0) + 0.15)]) - Point(0, 0).buffer(P["LEG_RB"], 256)
    elif kind == "bridge_outer":
        g = SP([(BR_X0 - 0.5, U0), (BR_X1 + 8.0, U0), (BR_X1 + 8.0, z_top(P["LEG_W"] / 2 - 0.2) + 0.05), (BR_X0 - 0.5, z_top(P["LEG_W"] / 2 - 0.2) + 0.05)]) - Point(0, 0).buffer(P["LEG_RB"], 256)
    else:
        g = SP([(BR_X0 - 0.5, U0), (BR_X1 + 8.0, U0), (BR_X1 + 8.0, z_top(0) + 0.05), (BR_X0 - 0.5, z_top(0) + 0.05)]) - Point(0, 0).buffer(P["LEG_RB"], 256)
    return g.buffer(grow, 32) if grow else g

def swept_xz(g, a0, a1, step=1.0):
    """union of g rotated CCW (x-z plane) from a0 to a1 deg about the local origin: the leg's world rotation RotY(-a) is CCW in x-z"""
    from shapely.ops import unary_union
    from shapely import affinity
    angs = list(np.arange(a0, a1 - 1e-6, step)) + [a1]
    return unary_union([affinity.rotate(g, a, origin=(0, 0)) for a in angs])

def place_leg(s, deg):
    """leg-local solid -> world at leg angle deg (sgn -1 about +y)"""
    return Pos(LX, 0, LZ) * (Rot(0, -deg, 0) * s)

print("leg ...")
LEG_BODY = build_leg()

# ================================================================== BOARDS (measured L x W x H, headers and terminal removed)
DECK1, DECK2 = 1.0, 7.2                       # deck top surfaces: deck 1 is the tray's backbone right over the cells, deck 2 carries VTX + buck
FT = 1.2
BOARD = {"xiao": (17.74, 22.75, 4.49), "drv8833_1": (12.70, 3.5, 20.31), "drv8833_2": (12.70, 3.5, 20.31),
         "ina219": (25.69, 20.32, 4.70), "bno085": (22.77, 25.72, 4.77), "mp1584": (22.26, 16.96, 3.98),
         "vtx": (42.25, 19.91, 6.01), "bms_2s": (40.8, 8.0, 2.1)}       # DRVs stand on edge beside the deploy motor (x 12.7, y 3.5, z 20.31)
LAY = {}                                       # name: (x0, y centre, z0)
XB0 = X_D1[1] + 0.6
Y_USB = 20.3                                   # XIAO USB-C receptacle face at y -20.3 (0.4 under the skin at its height)
LAY["xiao"] = (XB0, -Y_USB + BOARD["xiao"][1] / 2, DECK1)
LAY["ina219"] = (XB0 + 17.74 + 1.0, -17.6 + BOARD["ina219"][1] / 2, DECK1)
LAY["bms_2s"] = (XB0 + 3.0, 3.2 + BOARD["bms_2s"][1] / 2, DECK1)
LAY["bno085"] = (LAY["ina219"][0] + 25.69 + 2.4, 0.0, DECK1)
LAY["vtx"] = (LX + 16.0, 0.0, DECK2)              # behind the leg pockets
LAY["mp1584"] = (LAY["vtx"][0] + 42.25 + 1.0, 0.0, DECK2)
for nm, sg in (("drv8833_1", 1), ("drv8833_2", -1)):
    LAY[nm] = (DX - 6.35, sg * (7.6 + 1.75), -14.6)
XU = LAY["xiao"][0] + 17.74 / 2               # USB-C centre x
ZU = DECK1 + 1.0 + 3.26 / 2                    # USB-C centre z (PCB 1.0 + receptacle 3.26)
X_DECK2 = (X_D1[1] - 0.01, max(LAY["mp1584"][0] + 22.26, LAY["bno085"][0] + 22.77) + 1.6)
P.update(DECK1=DECK1, DECK2=DECK2, LAY={k: [round(float(c), 2) for c in v] for k, v in LAY.items()})

def pivot_blocks():
    """One faired block per flank round the drill pivot (v33 method, mirrored for the forward-folding pod): a teardrop in x-z
    (tangent arcs, flat on the split plane), side edges r 1.2, skirted into the skin r 1.5. Inboard of the arm head it is the
    coupler boss; over the inner 6 mm of the head it is carved by the swept arm (head, neck, shoulder, hub) from folded to
    DEPLOY_DEG, grown HORN_GAP: what is left behind the head is the DEPLOY STOP, under it the FOLD STOP (both about 1 deg past
    the ends of the swing). Returns (blocks, carve)."""
    from shapely.geometry import Point, box as sbox
    from shapely.ops import unary_union
    from shapely import affinity
    zp, xp = P["DRILL_PIV_Z"], P["DRILL_X"]
    circ = [(xp + 1.0, zp + 0.6, 8.6), (xp, zp, 7.8), (xp + 6.5, -1.0, 1.5), (xp - 5.0, -1.0, 1.5)]
    prof = unary_union([Point(0, 0).buffer(HH / 2, 96), sbox(-HH / 2, -HH / 2, 0, 0), sbox(-1.05, -HH / 2 - 0.05, ASH_X1, HH / 2 + 0.05),
                        sbox(ASH_X0 - (CORE_D - BUSH_D) / 2 - 0.05, -CORE_D / 2 - 0.05, ASH_X1, CORE_D / 2 + 0.05), sbox(S0, -HUB_D / 2, S0 + 8.0, HUB_D / 2)])
    prof = affinity.scale(prof, -1.0, 1.0, origin=(0, 0))                     # mirrored: the pod points -x
    sw = unary_union([affinity.rotate(prof, a, origin=(0, 0)) for a in np.arange(0.0, P["DEPLOY_DEG"] + 0.01, 0.5)])   # deploy = CCW in x-z
    sw = affinity.translate(sw.buffer(P["HORN_GAP"], 32), xp, zp)
    yi, yo = chord_half(zp) - 0.5, POD_FACE + 6.0
    blocks = carve = None
    for sg in (1, -1):
        a_ = (yi, yo) if sg > 0 else (-yo, -yi)
        blk = (yfillet(arc_prism(circ, *a_), a_, 1.2, "pivot block") & bx(xp - 20, xp + 20, -40, 40, 0, 40)) - cylx(2 * RB, 60, xp - 30)   # never into the bay
        c_ = (POD_FACE - 0.4, yo + 1.0) if sg > 0 else (-yo - 1.0, -POD_FACE + 0.4)
        cv = poly_prism(sw, *c_)
        blocks = blk if blocks is None else blocks + blk
        carve = cv if carve is None else carve + cv
    return blocks, carve

def D_bore_tear(d, y0, y1):
    r = d / 2; a = r / math.sqrt(2)
    sk = Circle(r) + Polygon((a, -a), (a, a), (r * math.sqrt(2), 0), align=None)
    sol = Pos(0, y1, 0) * extrude(Plane.XZ * sk, amount=y1 - y0)
    return sol - bx(-5, 5, y0 - 1, y1 + 1, r - 0.6, 5)

def build_arm(side):
    """Drill arm (v33, built pointing +x, mirrored to point forward): round head on the Ø7 D coupler (flat rear face = the deploy
    stop face), short neck, thrust shoulder, the stationary bushing that holds the drill gearmotor by its can; v34 adds the bridle
    EYE LUG on the head's outboard 5 mm, hanging below the pivot (EYE_DROP), so the tow line pulls the pod into its deploy stop."""
    y0, y1 = HEAD_YL
    head = cyly(HH, y1 - y0, y0) + bx(-HH / 2, 0, y0, y1, -HH / 2, 0)
    neck = bx(0.0, ASH_X1, -BUSH_D / 2, y1, -HH / 2, HH / 2)
    shoulder = cylx(CORE_D, ASH_X1 - ASH_X0, ASH_X0)
    under = Solid.make_cone(BUSH_D / 2, CORE_D / 2, (CORE_D - BUSH_D) / 2, Plane.YZ.offset(ASH_X0 - (CORE_D - BUSH_D) / 2))
    bush = cylx(BUSH_D, BUSH_X1 - ASH_X1 + 0.01, ASH_X1 - 0.01)
    hn = head + neck
    hn = safe_fillet(hn, [e for e in hn.edges().filter_by(Axis.Y) if abs(e.center().X + HH / 2) < 1e-3 and abs(e.center().Z + HH / 2) < 1e-3], 0.5, "arm stop corner")
    hn = yfillet(yfillet(hn, (y0,), 1.2, "arm inner face"), (y1,), 1.2, "arm outer face")
    ey0, ey1 = EYE_Y
    lug = yfillet(arc_prism([(0.0, 0.0, HH / 2 - 0.2), (P["EYE_FWD"], -P["EYE_DROP"], P["EYE_D"] / 2 + 1.6)], ey0, ey1), (ey0, ey1), 1.0, "eye lug")
    arm = hn + lug + under + shoulder + bush
    can_hole = cylx(P["MOTOR_W"] + 2 * CLR, BUSH_X1 - BW1 + 1, BW1) & bx(BW1 - 1, BUSH_X1 + 2, -P["MOTOR_H"] / 2 - CLR, P["MOTOR_H"] / 2 + CLR, -10, 10)
    eye = cyly(P["EYE_D"], ey1 - ey0 + 2, ey0 - 1, x=P["EYE_FWD"], z=-P["EYE_DROP"])
    eyec = [Solid.make_cone(P["EYE_D"] / 2 + 0.7, P["EYE_D"] / 2, 0.7, Plane(origin=(P["EYE_FWD"], yy, -P["EYE_DROP"]), z_dir=(0, d, 0))) for yy, d in ((ey0 - 0.01, 1), (ey1 + 0.01, -1))]
    cuts = [D_bore_tear(P["CPL_BORE"], y0 - 1, y1 + 1), cyly(CB_D, CB_DEPTH + 1, y1 - CB_DEPTH), can_hole,
            cylx(4.5, BW1 - 2.0 + 0.5, 2.0, y=0, z=2.5), cylz(4.5, 10, 2.5, x=3.5, y=0), eye] + eyec
    for c in cuts:
        arm = arm - c
    if side < 0:
        arm = arm.mirror(Plane.XZ)
    return mirror_x(arm)

# ================================================================== the leg's footprint in the hull (cuts and the material round them)
KN_X = (LX - P["N20_BOXH"] / 2 - 0.2, LX + P["N20_BOXH"] / 2 + 0.2)     # knuckle cavity (the leg N20's worm box) x
KN_Y = P["N20_BOXW"] / 2 + 0.2                                           # over the worm box
KN_YC = P["N20_CAN"][0] / 2 + 0.2                                         # over the motor can (12 wide) under it; the knuckle walls start here
KN_TOP = LZ + P["N20_BOX"] / 2 + 0.2
def leg_cuts(grow):
    """solids (world frame) the leg needs out of the hull/nose, grown by `grow`: pockets (cheeks + tines swept over the whole swing),
    the bridge swept over the knuckle, the folded blade's groove, the blade at the far rest (flute forward of the hinge), the pad"""
    from shapely import affinity
    a0, a1 = -2.0, P["LEG_DEG"] + P["LEG_REST_DEG"]
    out = []
    tine = affinity.translate(swept_xz(leg_profile_xz("tine"), a0, a1), LX, LZ).buffer(0.25 + grow, 32)
    for s in (1, -1):
        out.append(poly_prism(tine, *sorted((s * (P["PK_Y0"] - grow), s * (P["PK_Y1"] + grow)))))
    brg = affinity.translate(swept_xz(leg_profile_xz("bridge"), 0.0, 120.0), LX, LZ).buffer(0.25 + grow, 32)   # aft half: under the folded leg
    out.append(poly_prism(brg, -P["PK_Y1"] - grow, P["PK_Y1"] + grow))
    # forward half: outside the blade a smooth sweep of the bridge (its top there is at most the crown height at the blade edge), and the
    # blade root's true shape in the middle, so the bed the root rests in at the far rest is a concave saddle that mirrors its top
    bro = affinity.translate(swept_xz(leg_profile_xz("bridge_outer"), 115.0, a1), LX, LZ).buffer(0.25 + grow, 32)
    for s_ in (1, -1):
        out.append(poly_prism(bro, *sorted((s_ * max(P["LEG_W"] / 2 - 0.37 - grow, 0.3), s_ * (P["PK_Y1"] + grow)))))
    brc = affinity.translate(swept_xz(leg_profile_xz("bridge_centre"), 115.0, a1), LX, LZ).buffer(0.25 + grow, 32)
    out.append(poly_prism(brc, -P["LEG_W"] / 2 - 0.15 - grow, P["LEG_W"] / 2 + 0.15 + grow))
    # folded blade groove: floor 0.15 under the blade, from the bridge to the pad (vertical walls, open to the sky)
    xs = np.linspace(LX + BR_X1, P["PAD_X0"] + 1.0, 60)
    g_pts = [(x, LZ + leg_under(x - LX) - 0.15 - grow) for x in xs] + [(xs[-1], 30), (xs[0], 30)]
    out.append(prism_xz(g_pts, -P["LEG_FLARE"] / 2 - 2 - grow, P["LEG_FLARE"] / 2 + 2 + grow) & (Pos(LX, 0, LZ) * blade_plan(0.3 + grow, BR_X1 - 2, LEG_PX0 + 1.5)))
    # blade at the far rest (forward of the hinge): the blade section grown, rotated to LEG_DEG + LEG_REST_DEG
    hw = P["LEG_W"] / 2
    fl = prism_x(section_yz(-hw, hw, U0, grow=0.25 + grow), BR_X1 + 7.0, 60.0)              # (the root is in the swept bed above)
    out.append(place_leg(fl, a1))
    # pad recess (hull only, the nose never sees it): the pad grown
    pa = math.radians(P["PAD_DEG"] + 1.0) + grow / R
    arc = cylx(2 * R + 2, LEG_PX1 - LEG_PX0 + 0.5 + 2 * grow, P["PAD_X0"] - 0.25 - grow) - cylx(2 * (R - P["PAD_T"] - 0.25 - grow), 40, P["PAD_X0"] - 5)
    wedge = prism_x([(0, 0), (-40 * math.sin(pa), 40 * math.cos(pa)), (0, 40), (40 * math.sin(pa), 40 * math.cos(pa))], P["PAD_X0"] - 3, P["PAD_X1"] + 3)
    out.append(arc & wedge)
    return out

# cell saddle stations on the tray (U rings)
RING_XS = (X_C1[0] + 7, X_C1[1] - 9, X_C2[0] + 9, X_C2[1] - 7)
SAD_R = P["CELL_D"] / 2 + 0.27                             # U ring inner radius: opens 19.2 at the bottom (cells 18.66 slide in, no snap)

# ================================================================== BRIDLE (stowed on the launcher): each leg runs from its eye lug (bowline on the
# outboard face) aft past the arm head's outboard rear edge, climbs to the tail edge at ROPE_TH, rounds the edge in a groove, crosses the
# end wall in a groove to ROPE_END (inside the pusher's Ø14 tether hole) and turns aft to the sliding knot on the axis behind the tail.
# The grooves only guide the stowed legs: towed (anchored) the legs lift out and run straight from the eyes to the knot.
GW_, GD_ = P["GROOVE_W"], P["GROOVE_D"]
RG_ = GW_ / 2                                       # groove bottom radius
ROPE_R = P["ROPE_D"] / 2
FIL_R = 2.5                                         # tail edge fillet
RC_ROPE = R - GD_ + ROPE_R                          # rope centre radius in a groove on the skin (resting on the groove bottom)
def _rad(sg, th):
    return np.array([0.0, sg * math.sin(th), math.cos(th)])
def rope_stowed(sg):
    """stowed leg centre line (world, folded) for side sg: list of points, plus the index where the grooves start"""
    th = math.radians(P["ROPE_TH"])
    ey = Y_C + (EYE_Y[0] + EYE_Y[1]) / 2
    C = np.array([DX - P["EYE_FWD"], sg * ey, P["DRILL_PIV_Z"] - P["EYE_DROP"]])        # eye centre
    E = np.array([C[0], sg * (Y_C + EYE_Y[1] + 1.85), C[2]])                          # the leg leaves the bowline on the lug's outboard face
    R1 = np.array([X_END, 0, 0]) + RC_ROPE * _rad(sg, th)                              # groove start (the tail fillet begins at x X_END)
    # plan view: wrap round the arm head's rounded outboard rear edge (vertical, r 1.2) at rope-centre radius 1.2 + ROPE_R
    cc = np.array([DX + HH / 2 - 1.2, sg * (Y_C + HEAD_YL[1] - 1.2)]); rr = 1.2 + ROPE_R
    def tang(p, s):
        d = p - cc; Ld = float(np.linalg.norm(d)); a = math.atan2(d[1], d[0]); b = math.acos(rr / Ld)
        return cc + rr * np.array([math.cos(a + s * b), math.sin(a + s * b)])
    t1, t2 = tang(E[:2], -sg), tang(R1[:2], sg)
    a1, a2 = math.atan2(*(t1 - cc)[::-1]), math.atan2(*(t2 - cc)[::-1]); da = (a2 - a1 + math.pi) % (2 * math.pi) - math.pi
    plan = [E[:2]] + [cc + rr * np.array([math.cos(a1 + da * k / 6), math.sin(a1 + da * k / 6)]) for k in range(7)] + [R1[:2]]
    s_ = np.r_[0, np.cumsum(np.linalg.norm(np.diff(np.array(plan), axis=0), axis=1))]
    free = [np.array([q[0], q[1], E[2] + (R1[2] - E[2]) * si / s_[-1]]) for q, si in zip(plan, s_)]   # straight in the unrolled plane
    # round the edge: rope centre on radius FIL_R - GD_ + ROPE_R about the fillet's centre line, in the meridian plane at th
    fc = np.array([X_END, 0, 0]) + (R - FIL_R) * _rad(sg, th); re = FIL_R - GD_ + ROPE_R
    edge = [fc + re * (math.cos(a) * _rad(sg, th) + math.sin(a) * np.array([1.0, 0, 0])) for a in np.linspace(0, math.pi / 2, 7)][1:]
    xf_ = L - GD_ + ROPE_R
    B = np.array([xf_, sg * P["ROPE_END"][0], P["ROPE_END"][1]])
    return [C] + free + edge + [B], len(free)
def rope_len(pts):
    return float(np.sum(np.linalg.norm(np.diff(np.array(pts), axis=0), axis=1)))
_rl, _ = rope_stowed(1)
_l0 = rope_len(_rl); _B = _rl[-1]
_d = math.sqrt(max(1.0, (P["BRIDLE_L"] - _l0) ** 2 - _B[1] ** 2 - _B[2] ** 2))           # knot on the axis behind the tail face
KNOT_STOW = np.array([_B[0] + _d, 0.0, 0.0])
_eyeL = np.array(rope_stowed(1)[0][0])
def _deployed(p, deg):
    """folded world point on the left pod -> deployed world point (RotY(sgn * deg) about the pivot, sgn -1)"""
    q = np.array(p) - np.array([DX, 0, P["DRILL_PIV_Z"]]); a = math.radians(-deg)
    return np.array([DX + q[0] * math.cos(a) + q[2] * math.sin(a), p[1], P["DRILL_PIV_Z"] - q[0] * math.sin(a) + q[2] * math.cos(a)])
E_DEP = _deployed(rope_stowed(1)[0][1], P["DEPLOY_DEG"])
KNOT_DEP = np.array([E_DEP[0] + math.sqrt(P["BRIDLE_L"] ** 2 - E_DEP[1] ** 2), 0.0, E_DEP[2]])      # towed level, legs straight
P["ROPE"] = dict(left=[[round(float(c), 3) for c in q] for q in _rl], knot_stowed=[round(float(c), 2) for c in KNOT_STOW],
                 knot_behind_tail_mm=round(float(KNOT_STOW[0] - L), 1), stowed_len_mm=round(_l0 + float(np.linalg.norm(KNOT_STOW - _B)), 1),
                 eye_deployed=[round(float(c), 2) for c in _deployed(_eyeL, P["DEPLOY_DEG"])], leg_start_deployed=[round(float(c), 2) for c in E_DEP],
                 knot_deployed=[round(float(c), 2) for c in KNOT_DEP], knot_deployed_behind_tail_mm=round(float(KNOT_DEP[0] - L), 1),
                 rope_d=P["ROPE_D"], eye_knot=[round(float(c), 2) for c in (_rl[1] + np.array([0, 0.45, 0]))], eye_knot_r=2.2, knot_r=2.0)
print("rope: stowed eye->groove end %.1f mm, knot %.1f behind the tail face; anchored knot %.1f behind" % (_l0, KNOT_STOW[0] - L, KNOT_DEP[0] - L))

def _slot(p0, p1, up, ext=3.0, cap0=False, cap1=True):
    """groove cutter along p0->p1: circle r RG_ on the segment plus a slab GW_ wide out along `up` (made perpendicular); round ends where capped"""
    p0, p1, up = (np.array(v, float) for v in (p0, p1, up))
    d = p1 - p0; Lr = float(np.linalg.norm(d)); u = d / Lr
    up = up - (up @ u) * u; up = up / np.linalg.norm(up)
    s = Pos(Lr / 2, 0, 0) * Cylinder(RG_, Lr, rotation=(0, 90, 0)) + Pos(Lr / 2, 0, ext / 2) * Box(Lr, GW_, ext)
    for on, xx in ((cap0, 0.0), (cap1, Lr)):
        if on:
            s = s + Pos(xx, 0, 0) * Sphere(RG_) + Pos(xx, 0, ext / 2) * Cylinder(RG_, ext)
    return Plane(origin=tuple(p0), x_dir=tuple(u), z_dir=tuple(up)) * s

def rope_grooves():
    """cutters for both sides: a lead-in on the flank along the leg's approach, the groove round the tail edge (the profile revolved about the
    fillet's centre line), the groove across the end wall to ROPE_END; and the rib inside the wall behind the lead-in and the edge"""
    th = math.radians(P["ROPE_TH"]); cut = None; rib = None
    gc = R - (GD_ - RG_)                                                  # groove circle centre radius (0.6 under the skin)
    for sg in (1, -1):
        pts, nf = rope_stowed(sg)
        p1 = np.array([X_END, 0, 0]) + gc * _rad(sg, th)
        a_, b_ = pts[nf - 1], pts[nf]                                        # the free span's last segment (to the groove start)
        p0 = b_ + (a_ - b_) * min(1.0, 2.2 / float(np.linalg.norm(a_ - b_)))   # lead-in starts 2.2 mm up the approach line
        p0 = p0 + (gc - RC_ROPE) * _rad(sg, math.atan2(sg * p0[1], p0[2]))
        lead = _slot(p0, p1 + (p1 - p0) / np.linalg.norm(p1 - p0) * 0.9, _rad(sg, th), cap0=True, cap1=True)   # runs on past the fillet start
        a = np.array([0.0, math.cos(th), -sg * math.sin(th)])                 # tangential axis: +90 deg about it turns the radial into +x
        prof = Plane(origin=tuple(p1), x_dir=tuple(a), z_dir=(1, 0, 0)) * (Circle(RG_) + Pos(0, 1.5) * Rectangle(GW_, 3.0))
        fc = np.array([X_END, 0, 0]) + (R - FIL_R) * _rad(sg, th)
        edge = _bd.revolve(prof.faces(), axis=Axis(tuple(fc), tuple(a)), revolution_arc=90)
        A = fc + (FIL_R - (GD_ - RG_)) * np.array([1.0, 0, 0]) + 0.15 * _rad(sg, th)
        B = np.array([L - (GD_ - RG_), sg * P["ROPE_END"][0], P["ROPE_END"][1]])
        face = _slot(A, B, (1, 0, 0), cap0=False, cap1=True)
        c_ = lead + edge + face
        cut = c_ if cut is None else cut + c_
        w0, w1 = math.radians(P["ROPE_TH"] - 5.0), math.radians(P["ROPE_TH"] + 9.0)
        wedge = prism_x([(0, 0)] + [(sg * 30 * math.sin(t_), 30 * math.cos(t_)) for t_ in np.linspace(w0, w1, 4)], X_END - 4.0, X_END + 0.05)
        r_ = (cylx(2 * (RB + 0.1), 4.05, X_END - 4.0) - cylx(2 * 18.0, 6, X_END - 5.0)) & wedge
        rib = r_ if rib is None else rib + r_
    return cut, rib

# ================================================================== HULL (clamshell, integral tail end wall)
def build_hull():
    hx0 = X_NOSE
    env = cylx(2 * R, L - hx0, hx0)
    env = safe_fillet(env, env.edges().group_by(Axis.X)[-1], 2.5, "tail edge")
    shell = env - cylx(2 * RB, X_END - hx0 + 1, hx0 - 1)
    # pivot blocks (deploy stop + fold stop + coupler bosses), skirted into the skin
    blocks, carve = pivot_blocks()
    sk2, box2 = skirt(blocks, xlim=(hx0, X_END))
    body = (shell - box2) + sk2
    # material round the leg's cuts (groove floor and walls, pocket walls, knuckle roof), the knuckle blister, the pad band,
    # the socket seats; all kept inside the Ø42 envelope
    fill = None
    for f in leg_cuts(1.0):
        fill = f if fill is None else fill + f
    fill = (fill & cylx(2 * R - 0.02, L - hx0 - 4, hx0 + 0.5)) - cylx(2 * RB + 0.2, X_SPIG1 - hx0 + 1.25, hx0 - 1)   # never into the nose spigot
    blister = bx(KN_X[0] - 0.6, KN_X[1] + 0.6, -KN_Y - 0.6, KN_Y + 0.6, LZ, KN_TOP + 0.6)
    blister = safe_fillet(blister, [e for e in blister.edges() if e.center().Z > KN_TOP], 1.6, "blister")
    blister = blister - cylx(2 * RB + 0.2, X_SPIG1 - hx0 + 1.25, hx0 - 1)                    # never into the nose spigot
    body = body + fill + blister
    for sg in (1, -1):                                                         # flank socket seats (JST-XH headers clamped across the split)
        z0_, z1_ = SEAT_Z[sg][0] - 1.2, SEAT_Z[sg][1] + 1.2
        body = body + (bx(X_SOCK[0] - 1.2, X_SOCK[1] + 1.2, sg * 7.0, sg * 21.0, z0_, z1_) & cylx(2 * RB + 0.6, 20, X_SOCK[0] - 5))   # inside the wall only: the skin stays one surface
    body = body + GROOVE_RIB                                                   # behind the bridle lead-in and the tail-edge groove
    body = body + (cylx(2 * RB + 0.2, 2.0, NOSE_LOCK_X) - cylx(2 * (RB - 1.0), 4.0, NOSE_LOCK_X - 1))   # nose lock rib (cut by the flute like the wall)
    body = body - carve
    for c in leg_cuts(0.0):
        body = body - c
    body = body - bx(*KN_X, -KN_Y, KN_Y, -5, KN_TOP)                                   # knuckle cavity: the worm box
    body = body - bx(LX - 5.2, LX + 5.2, -KN_YC, KN_YC, -14, LZ - 6.0 + 0.01)               # ... and the motor can under it
    # couplers: Ø6.3 bores from the knuckle wall out through the skin; stub slots from below (the tray tongues close them)
    for s in (1, -1):
        ya, yb = sorted((s * (KN_Y - 0.1), s * (P["PK_Y1"] + 0.5)))
        body = body - cyly(P["LEG_CPL_D"] + 0.3, yb - ya, ya, x=LX, z=LZ)                       # coupler bore: knuckle wall + pocket
        ya, yb = sorted((s * P["PK_Y1"], s * (R + 2)))
        body = body - cyly(P["LEG_PLUG_D"] + 0.1, yb - ya, ya, x=LX, z=LZ)                      # counterbore for the skin plug
        body = body - bx(LX - 1.75, LX + 1.75, s * (KN_YC - 0.05), s * (P["LEG_STUB"] + 0.4), -5, LZ)
    cuts = []
    cuts.append(tear_y(P["CPL_D"] + 0.4, 2 * R + 20, -R - 10, x=DX, z=P["DRILL_PIV_Z"]))          # pod coupler bores
    for s in (-1, 1):                                                                       # drill leads: tight bores (seal with silicone)
        cuts.append(cyly(3.0, 7.0, 8.5 if s > 0 else -15.5, x=DX - 10.5, z=16.5))
    for xs in SCR_X:                                                                         # radial screws into the bosses on disc 1 and the rear disc
        for s in (-1, 1):
            for zz in (-P["RAD_Z"], P["RAD_Z"]):
                cuts.append(cyly(P["SCREW_CLR"], 14, (RB - 2) if s > 0 else -(RB - 2) - 14, x=xs, z=zz))
                cuts += csink(xs, s, zz)
    # XIAO USB-C: window round the receptacle (8.94 x 3.26), its face 0.4 under the skin, open down to the split so the hull top drops
    # straight on (the tray's tab fills the notch under the receptacle)
    cuts.append(bx(XU - 4.65, XU + 4.65, -R - 2, -RB + 1.2, 0.0, ZU + 1.8))
    # flank socket seats: header pocket + plug opening to the skin (left 2-pin ON loop 7.9 tall, lowered LOOP_DZ; right 3-pin charge 10.2 tall)
    for sg in (1, -1):
        za, zb = SEAT_Z[sg]
        op = Pos((X_SOCK[0] + X_SOCK[1]) / 2, 0, (za + zb) / 2) * extrude(Plane.XZ.offset(-sg * 8.0) * RectangleRounded(X_SOCK[1] - X_SOCK[0], zb - za, 0.4), amount=14.0 * 1, dir=(0, sg, 0))
        cuts.append(op)
        ph = 2.6 if sg > 0 else 3.5
        cuts.append(bx(X_SOCK[0] + 0.9, X_SOCK[1] - 0.9, sg * 5.0, sg * 8.5, (za + zb) / 2 - ph, (za + zb) / 2 + ph))      # pins / solder tails
    # bridle grooves (tail edge and end wall) and the slots in both end walls that the tray's tail key sits in (aligns the unfastened tail)
    cuts.append(GROOVES)
    cuts.append(bx(X_END - 0.1, X_END + 1.0, -4.15, 4.15, DECK1 - 1.55, DECK1 + 0.15))
    # seam groove: the tray's deck 1 edges sit in it, clamped by the two halves, and back the split seam from inside
    sg_ = (cylx(2 * (RB + 0.45), X_SOCK[0] - 1.5 - X_LCR[0] + 0.1, X_LCR[0] - 0.1) - cylx(2 * RB - 0.2, X_SOCK[0] + 2 - X_LCR[0], X_LCR[0] - 1)) & bx(0, L, -60, 60, DECK1 - 1.4 - 0.1, DECK1 + 0.1)
    cuts.append(sg_)
    for c in cuts:
        body = body - c
    return body

def split_hull(hull):
    top = hull & bx(-10, L + 10, -60, 60, 0, 80)
    bot = hull & bx(-10, L + 10, -60, 60, -80, 0)
    return top, bot

SCR_X = (P["SCR_X1"], (X_D2[0] + X_D2[1]) / 2)                     # radial screw stations (disc-1 station clear of the XIAO)
SEAT_Z = {1: (P["LOOP_DZ"] - 3.95, P["LOOP_DZ"] + 3.95), -1: (-5.1, 5.1)}   # flank socket openings (z): left 2-pin 7.9 tall lowered, right 3-pin 10.2
print("rails, putty, grooves ...")
GROOVES, GROOVE_RIB = rope_grooves()
print("hull ...")
hull = build_hull()
top, bot = split_hull(hull)
add("hull_top", top, "hull", "#9aa5b1", orient="split face down (on the bed), crown up",
    note="Clamshell upper half with the integral tail end wall: skin; the dorsal groove where the righting leg lies flush (its floor is the leg's fold stop), the knuckle (a 0.6 mm blister over the leg N20's worm box), two side pockets for the leg's cheeks, the couplers' bores through the knuckle walls and the outer walls, the flute forward of the hinge (far rest %g deg), the arch recess for the pad on the tail top; two faired pivot blocks (pod coupler bores; their carved faces are the deploy and fold stops); tight bores for the drill leads; the XIAO USB-C notch (open to the split, so the half drops straight on); upper halves of the two flank socket seats; the two bridle grooves (round the tail edge and across the end wall) with a rib inside behind each; the upper half of the tail key slot in the end wall. The seam is a butt joint: deck 1's edges in the seam groove, the nose spigot and the tray's tail key in the end-wall slot align the halves. Radial M3 clearance at z +3 into the bosses on disc 1 and the rear disc" % (P["LEG_DEG"] + P["LEG_REST_DEG"]))
add("hull_bottom", bot, "hull", "#7d8794", orient="split face down, belly up",
    note="Clamshell lower half with the integral tail end wall: plain Ø42 belly (launcher cradle surface; the cells rest on it, located by the tray's U saddles), lower halves of the pivot blocks' skirts, of the two flank socket seats and of the tail key slot. Radial M3 clearance at z -3 into the bosses on disc 1 and the rear disc")

# ================================================================== NOSE (hollow, 1.5 walls, camera 2.2 below the axis under the leg's flute)
NOSE_K = (R - P["NOSE_R0"]) / P["NOSE_L"]
NOSE_XI = (W * 1.6, P["NOSE_L"] - 1.5)                       # inner cone (the nose's hollow) x range
def nose_ri(x):
    return P["NOSE_R0"] - W * 1.12 + NOSE_K * x                # inner cone radius
def nose_tube():
    cw, ch = P["CAM_W"] + 2 * CLR, P["CAM_H"] + 2 * CLR
    return rrect_x(cw + 2 * 1.2, ch + 2 * 1.2, P["NOSE_L"] + P["NOSE_SPIG"] - 1.0, 0.5, z=P["CAM_Z"], r=1.5)
def putty_collar(xe):
    """retaining collar on the inner cone wall behind the putty: a 45 deg ramp (prints tip down without support) narrowing the hollow 1.0 mm,
    flat on its aft side; below BAL_ZTOP only"""
    pts = [(xe, nose_ri(xe) + 0.3), (xe, nose_ri(xe) - 0.02), (xe + 1.0, nose_ri(xe + 1.0) - 1.0), (xe + 1.0, nose_ri(xe + 1.0) + 0.3)]
    ring = _bd.revolve(Plane.XZ * Polygon(*pts, align=None), axis=Axis.X, revolution_arc=360)
    return ring & bx(xe - 1, xe + 2, -30, 30, -30, P["BAL_ZTOP"])
def putty_region(xe):
    """the nose ballast: the lower part of the hollow round the camera tube, from the front of the hollow back to the collar at xe"""
    cone = Solid.make_cone(nose_ri(NOSE_XI[0]), nose_ri(xe + 1.0), xe + 1.0 - NOSE_XI[0], Plane.YZ.offset(NOSE_XI[0]))
    return (cone & bx(0, xe + 1.0, -30, 30, -30, P["BAL_ZTOP"])) - nose_tube() - putty_collar(xe)
_lo, _hi = 6.0, NOSE_XI[1] - 1.2                               # solve the collar station for the putty volume
for _ in range(14):
    _m = (_lo + _hi) / 2
    if putty_region(_m).volume / 1000 * P["BAL_RHO"] < P["BALLAST_G"]: _lo = _m
    else: _hi = _m
PUTTY_XE = round(_hi, 2)
P["BAL_XE"] = PUTTY_XE
print("nose putty: collar at x %.2f" % PUTTY_XE)

def build_nose():
    nl, r0 = P["NOSE_L"], P["NOSE_R0"]
    outer = Solid.make_cone(r0, R, nl, Plane.YZ)
    spig = cylx(2 * RB - 2 * CLR, P["NOSE_SPIG"], nl - 0.01)
    body = outer + spig
    body = safe_fillet(body, body.edges().group_by(Axis.X)[0], 1.5, "nose tip")
    env_ = body
    k = (R - r0) / nl
    xi0, xi1 = W * 1.6, nl - 1.5
    inner = Solid.make_cone(r0 - W * 1.12 + k * xi0, r0 - W * 1.12 + k * xi1, xi1 - xi0, Plane.YZ.offset(xi0))
    inner_sp = cylx(2 * RB - 2 * CLR - 2 * 3.0, P["NOSE_SPIG"] + 2, nl - 1.5)
    body = body - inner - inner_sp
    cz = P["CAM_Z"]
    cw, ch = P["CAM_W"] + 2 * CLR, P["CAM_H"] + 2 * CLR
    tube = rrect_x(cw + 2 * 1.2, ch + 2 * 1.2, nl + P["NOSE_SPIG"] - 1.0, 0.5, z=cz, r=1.5)
    body = body + (tube & (outer + spig))
    # the leg's flute: solid material under it first (the nose stays closed over the camera), then the cuts
    fill = None
    for f in leg_cuts(1.3)[:-1]:
        fill = f if fill is None else fill + f
    body = body + (fill & env_)
    body = body - bx(P["NOSE_LIP"] + P["CAM_LENS"], nl + P["NOSE_SPIG"] + 5, -cw / 2, cw / 2, cz - ch / 2, cz + ch / 2)
    body = body - cylx(P["CAM_HOLE"], P["NOSE_LIP"] + P["CAM_LENS"] + 1, -1, z=cz)
    body = body - (cylx(2 * R + 2, 2.4, NOSE_LOCK_X - 0.2) - cylx(2 * (RB - 1.0) - 0.4, 4, NOSE_LOCK_X - 1))
    for c in leg_cuts(0.0)[:-1]:
        body = body - c
    body = body + putty_collar(PUTTY_XE)                                    # retains the nose putty against the launch push
    return body

print("nose ...")
nose = build_nose()
add("nose", nose, "hull", "#b8c0ca", orient="tip down on the bed (flat Ø22), spigot up",
    note="Blunt hollow nose: Ø22 flat tip with the Ø9.8 lens hole, Caddx Ant pocket 14.4 x 15.0 (slides in from the back, 1.2 lip + lens recess hold it; 2.2 below the axis), 1.5 walls; the BALLAST POCKET: the lower part of the hollow round the camera tube from the front of the hollow back to a retaining collar at x %.1f, filled with %.1f g of putty from the back (through the spigot) before the nose goes on; the forward end of the dorsal flute where the righting leg lies at its far rest (solid under it, 1.2 over the camera tube); spigot with a groove that the hull rib locks when the clamshell closes" % (PUTTY_XE, P["BALLAST_G"]))

# ================================================================== TRAY: one printed spine, deck 1 over the in-line cells
def disc(x0, x1, screws=False, windows=(), xs=None):
    """tray disc; with screws: four solid bosses BOSS_T thick (x) round the radial screw stations at xs, the Ø2.5 pilot 1.5 mm or more
    inside solid all round over the whole bite, 1.5 deeper than the screw tip"""
    rd = RB - CLR
    d = cylx(2 * rd, x1 - x0, x0)
    if screws:
        bt, zb = P["BOSS_T"] / 2, P["SCREW_PILOT"] / 2 + 1.5
        for s in (-1, 1):
            for zz in (-P["RAD_Z"], P["RAD_Z"]):
                d = d + (bx(xs - bt, xs + bt, s * 9.8, s * RB, zz - zb, zz + zb) & cylx(2 * rd, 2 * bt + 2, xs - bt - 1))
    for w in windows:
        d = d - w
    if screws:
        for s in (-1, 1):
            for zz in (-P["RAD_Z"], P["RAD_Z"]):
                d = d - cyly(P["SCREW_PILOT"], RB + 0.5 - 10.5, 10.5 if s > 0 else -RB - 0.5, x=xs, z=zz)
                d = d - seat(xs, s, zz)
    return d

def build_tray():
    """One printed spine (prints deck 1 down, cradles and saddles up): deck 1 runs nose to tail right over the cells, its edges sit in
    the hull's seam groove (they back the split seam from inside and the halves clamp them); disc 1 and the rear disc take the 8 radial
    screws and the cell contacts; the leg N20 cradle at the front (motor pointing down) with two tongues that close the stub slots in the
    hull's knuckle walls and pocket floors; cell saddles hanging under deck 1 (open at the bottom: the cells rest on the hull bottom);
    deck 2 on three walls for the VTX and the buck; the deploy N20 cradle at the rear (motor pointing down) with the two DRV8833 standing
    beside it."""
    ps = []
    cz = P["CELL_Z"]; cr = P["CELL_D"] / 2 + 0.25
    # deck 1: edges out to r RB + 0.35 (in the seam groove)
    x0, x1 = X_LCR[0], X_SOCK[0] - 1.7
    d1 = cylx(2 * (RB + 0.35), x1 - x0, x0) & bx(x0, x1, -30, 30, DECK1 - 1.4, DECK1)
    d1 = d1 - bx(X_DCR[0] - 0.2, X_DCR[1] + 0.2, -11.4, 11.4, -5, 5)                        # deploy N20 and the standing DRVs pass through
    d1 = d1 - bx(LX - 5.2, LX + 5.2, -6.2, 6.2, -5, 5)                                       # leg N20 can
    d1 = d1 - bx(X_MID[0] - 0.6, X_MID[1] + 0.6, -2.0, 2.0, -5, 5)                           # slot for the BM strip's tab at the cell joint
    for xw in (X_D1[1] + 2.0, X_D2[0] - 9.0):                                                # wire slots down to the corridors beside the cells
        for s in (1, -1):
            d1 = d1 - bx(xw, xw + 6.0, s * 11.0, s * 16.5, -5, 5)
    ps.append(d1)
    # discs (wire windows in the corridors beside the cells and at the top)
    def wins(xa, xb):          # corridor windows stop under the lower screw bosses (z -5.75)
        return [bx(xa - 1, xb + 1, s * 10.4, s * 25.0, -15.0, -6.0) for s in (1, -1)] + [bx(xa - 1, xb + 1, -6.0, 6.0, 9.0, 30)]
    ps.append(disc(*X_D1, screws=True, xs=SCR_X[0], windows=wins(*X_D1) + [bx(X_D1[0] - 1, X_D1[1] + 1, -6.2, 6.2, -13.4, 9), bx(X_D1[0] - 1, X_D1[1] + 1, -13.2, 13.2, 7.0, 30)]))
    ps.append(disc(*X_D2, screws=True, xs=SCR_X[1], windows=wins(*X_D2) + [bx(X_D2[0] - 1, X_D2[1] + 1, -7.2, 7.2, 2.0, 30)]))
    # cell saddles: U rings under deck 1 (arch over the cell, straight legs down to the print plane, 19.2 open at the bottom: the cells
    # slide in without spreading the ring; the belly carries them)
    ro_, h_ = SAD_R + 1.2, cz - TRAY_Z0 + 1.0
    u_prof = (Pos(0, cz) * Circle(ro_) + Pos(0, cz - h_ / 2) * Rectangle(2 * ro_, h_)) - (Pos(0, cz) * Circle(SAD_R) + Pos(0, cz - h_ / 2 - 0.5) * Rectangle(2 * SAD_R, h_ + 1.0))
    for xs_ in RING_XS:
        ring = extrude(Plane.YZ.offset(xs_) * u_prof, amount=2.0)
        ps.append(ring + bx(xs_, xs_ + 2.0, -3.0, 3.0, cz + SAD_R - 0.05, DECK1 - 1.0))
    # tail key: deck 1 continues as a tongue into the slots in both halves' end walls (aligns the unfastened tail); a web under it to print
    ps.append(bx(X_SOCK[0] - 1.8, X_END + 0.85, -4.0, 4.0, DECK1 - 1.4, DECK1))
    ps.append(bx(X_DCR[1] + 0.6, X_END - 0.2, -0.6, 0.6, TRAY_Z0, DECK1 - 1.39))
    # leg N20 cradle (tube round the can, floor under it) + the two tongues
    lc = bx(*X_LCR, -7.4, 7.4, TRAY_Z0, LZ - 6.0 - 1.3) - bx(LX - 5.2, LX + 5.2, -6.2, 6.2, -13.0, 20) - bx(LX - 5.2, LX + 5.2, -6.2, 6.2, TRAY_Z0 - 1, -14.0)
    lc = lc + bx(*X_LCR, -6.2, 6.2, LZ - 6.0 - 1.4, LZ - 6.0 - 0.2) - bx(LX - 5.2, LX + 5.2, -6.2, 6.2, -13.0, 20)   # the worm box rests on the front and back walls
    lc = lc - bx(LX - 2.5, LX + 2.5, -9, 9, -16, -11.0)                                      # lead exit at the can's end cap
    ps.append(lc & cylx(2 * RB - 0.6, 40, X_LCR[0] - 1))
    for s in (1, -1):
        ya, yb = sorted((s * (KN_YC + 0.1), s * (P["PK_Y0"] - 0.25)))                         # in the knuckle wall's slot, up to the coupler bore
        t = bx(LX - 1.6, LX + 1.6, ya, yb, LZ - 6.0 - 1.4, LZ - P["LEG_CPL_D"] / 2 - 0.15)
        t = t - cyly(P["LEG_CPL_D"] + 0.3, 3, min(ya, yb) - 0.5, x=LX, z=LZ)
        ps.append(t)
        ya, yb = sorted((s * (P["PK_Y0"] - 0.3), s * (P["LEG_STUB"] + 0.35)))                 # in the pocket floor's slot
        ps.append(bx(LX - 1.6, LX + 1.6, ya, yb, LZ - 6.0 - 1.4, POCKET_FLOOR - 0.05))
    # deck 2 on three walls (front = disc 1, a mid wall between the INA219 and the BNO085, an end wall)
    xd0, xd1 = X_DECK2
    ps.append(bx(xd0, xd1, -11.0, 11.0, DECK2 - FT, DECK2) & cylx(2 * RB - 1.2, 200, 0))
    for xw in (LAY["bno085"][0] - 1.6, LAY["bno085"][0] + 22.77 + 0.4):
        ps.append(bx(xw, xw + 1.2, -3.5, 3.5, DECK1 - 0.01, DECK2 - FT + 0.01))
    # deploy N20 cradle (tube round the can, motor pointing down) + web to the rear disc
    dcz = P["DRILL_PIV_Z"] - P["N20_BOX"] / 2                                               # worm box bottom: it rests on the tube's front and back walls
    dc = bx(*X_DCR, -7.4, 7.4, TRAY_Z0, dcz - 0.2) - bx(DX - 5.2, DX + 5.2, -6.2, 6.2, -30, 20)   # tube round the can (the can hangs in it)
    dc = dc - bx(X_DCR[0] - 1, X_DCR[1] + 1, -7.5, 7.5, dcz - 1.6, dcz)                      # the tube's side walls stop under the box sides
    dc = dc + bx(*X_DCR, -6.2, 6.2, dcz - 1.6, dcz - 0.2) - bx(DX - 5.2, DX + 5.2, -6.2, 6.2, -30, 20)
    ps.append(dc & cylx(2 * RB - 0.6, 40, X_DCR[0] - 1))
    ps.append(bx(X_D2[1] - 0.01, X_DCR[0] + 0.01, -7.4, 7.4, TRAY_Z0, DECK1))
    t = ps[0]
    for p in ps[1:]:
        t = t + p
    t = t & cylx(2 * (RB + 0.35), L, -1) & bx(-10, L + 10, -40, 40, TRAY_Z0, 40)           # flat underside = the print plane
    # USB-C tab: fills the hull top's notch under the XIAO receptacle (out to the skin, 0.15 under the receptacle)
    t = t + (bx(XU - 4.55, XU + 4.55, -(R - 0.05), -19.05, 0.05, ZU - 3.26 / 2 - 0.15) & cylx(2 * (R - 0.05), 20, XU - 10))
    # battery contacts: front springs on disc 1's rear face, the BM strip between the cells, rear plates on the rear disc
    t = t - cylx(10.0, 0.8, X_D1[1] - 0.8 + 0.01, z=cz)
    t = t - cylx(10.0, 0.8, X_D2[0] - 0.01, z=cz)
    t = t - cylx(2.5, 6, X_D1[0] - 1, y=4.0, z=cz) - cylx(2.5, 6, X_D2[0] - 2, y=4.0, z=cz)    # contact lead holes
    return t

TRAY_Z0 = -16.0                                              # the tray's flat underside (prints on it)
POCKET_FLOOR = LZ - P["LEG_CHEEK_R"] - 0.25                 # pocket floor top (the cheek's lowest point, grown)
print("tray ...")
tray = build_tray()
add("tray", tray, "hull", "#c9b37e", orient="flat underside (z -16: saddle feet, disc bottoms, cradle tubes) on the bed; deck 1 bridges between the saddles and discs, deck 2 between its walls (enable thick bridges)",
    note="One-piece spine: deck 1 nose to tail right over the two cells (its edges sit in the hull's seam groove, backing the split seam; at the tail it continues as a key into the slots in both end walls), disc 1 and the rear disc with a solid 6 mm boss at each of the 8 radial M3 screws, the leg N20 cradle at the front (motor pointing down) with two tongues that close the stub slots, four U cell saddles (open 19.2 at the bottom, the cells slide in), deck 2 for the VTX and the MP1584, the deploy N20 cradle at the rear (motor pointing down) with the two DRV8833 standing beside it, a tab under the XIAO USB-C that fills the hull top's notch; contact recesses (springs front, BM strip between the cells, plates rear)")

# ================================================================== RIGHTING LEG part + its couplers
add("righting_leg", LEG_BODY, "legs", "#e07b39", pos=(LX, 0, LZ), axis="y", deg=(0, P["LEG_DEG"]), sgn=-1, joint="leg",
    orient="upside down (the blade crown on the bed, cheeks up), brim 5 mm (the crown touches the bed along a 5 mm strip only), tree supports under the ends of the pad arch",
    note="Righting leg (PLA): a slim blade %g wide whose top is the hull crown (flush in the dorsal groove), an arch pad that is a patch of the hull skin on the tail top (%.0f wide), and a fork root: two round cheeks on the two Ø6 couplers in the side pockets, tines curving aft into a bridge that swings over the knuckle. Swing 0 to %g deg over the nose (far rest %g); hinge %g from the nose tip, pad centre %.0f from the hinge. Each cheek is a press fit (D bore %.2f) on its coupler, a drop of CA" % (P["LEG_W"], 2 * R * math.sin(math.radians(P["PAD_DEG"])), P["LEG_DEG"], P["LEG_DEG"] + P["LEG_REST_DEG"], LX, (LEG_PX0 + LEG_PX1) / 2, P["LEG_CPL_BORE"]))
for nm, s in (("leg_coupler_L", 1), ("leg_coupler_R", -1)):
    y0, y1 = sorted((s * (KN_Y + 0.15), s * (P["PK_Y1"] + 0.95)))
    c = cyly(P["LEG_CPL_D"], y1 - y0, y0) - bx(-4, 4, y0 - 1, y1 + 1, P["LEG_CPL_D"] / 2 - 0.5, 4)        # D flat: keys the cheek's D bore
    ys0 = s * (KN_Y + 0.1)
    sock = D_bore_y(P["SOCK_D"], *sorted((ys0, s * (P["LEG_STUB"] + 0.3))), flat=P["SOCK_FLAT"])           # press-fit D socket on the trimmed N20 stub
    c = c - sock
    c = safe_fillet(c, c.edges().group_by(Axis.Y)[-1 if s > 0 else 0], 0.4, "leg cpl")
    add(nm, c, "legs", "#444", parent="righting_leg",
        orient="lying on its D flat, PPA-CF",
        note="Ø6 D coupler (PPA-CF): pressed in from outside through the plug counterbore, the cheek (press fit, CA) and the knuckle wall bore onto the leg N20's trimmed stub (press-fit D socket Ø%.2f, flat at %.2f, CA): no play between the motor and the leg; it runs in the knuckle wall bore and in the skin plug's blind bore, so the hull carries the leg; the plug and the worm box hold it axially" % (P["SOCK_D"], P["SOCK_FLAT"]))
for nm, s in (("leg_plug_L", 1), ("leg_plug_R", -1)):
    ya, yb = sorted((s * (P["PK_Y1"] + 0.02), s * (R + 1)))
    pl = cyly(P["LEG_PLUG_D"], yb - ya, ya, x=LX, z=LZ) & cylx(2 * R - 0.1, 30, LX - 15)          # outer face = the skin
    ya, yb = sorted((s * (P["PK_Y1"] - 0.1), s * (P["PK_Y1"] + 1.15)))
    pl = pl - cyly(P["LEG_CPL_D"] + 0.3, yb - ya, ya, x=LX, z=LZ)                                 # blind bore: the coupler's outer bearing
    add(nm, pl, "hull", "#9aa5b1", orient="outer face up (a domed button), no supports",
        note="Skin plug Ø7.5 (PLA): pressed flush into the counterbore over the leg coupler (a drop of CA); its blind bore is the coupler's outer bearing and holds it in. Pry it out to pull the coupler for service")
# ================================================================== DRILL ARMS + AUGERS (rear pods, folded forward)
def taper_auger(lefthand):
    hr, rc0, rc1 = HUB_D / 2, P["D_CORE0"] / 2, P["D_CORE1"] / 2
    neck = lambda z: rc0 + (hr - rc0) * (1 + math.cos(math.pi * (z - XE0) / (XR - XE0))) / 2
    prof = [(0.0, S0), (hr - 0.4, S0), (hr, S0 + 0.4)] + [(neck(z_), z_) for z_ in np.linspace(XE0, XR, 25)] + [(rc1, XT0), (P["D_TIP_R"], TIP), (0.0, TIP)]
    body = revolve(round_poly(prof, {len(prof) - 3: 25.0}))
    def r_body(z):
        if z <= XE0: return hr
        if z <= XR: return neck(z)
        if z <= XT0: return rc0 + (rc1 - rc0) * (z - XR) / (XT0 - XR)
        return rc1 + (P["D_TIP_R"] - rc1) * min(1.0, (z - XT0) / P["D_TAPER"])
    rf = P["FLIGHT_D"] / 2
    r_out = lambda z: rf if z <= XC0 else rf + (P["D_TIP_R"] + 0.6 - rf) * min(1.0, (z - XC0) / P["D_TAPER_F"])
    z0 = XE0 + (XR - XE0) * math.acos(2 * (rf - 0.5 - rc0) / (hr - rc0) - 1) / math.pi + P["D_LAND"]
    fl = buttress_flight(z0, TIP - 2.0, P["D_PITCH"], r_body, r_out, P["D_LOAD_DEG"], P["D_LEAD_DEG"], P["D_LAND"], lefthand=lefthand)
    cav = revolve([(0.0, S0 - 1.0), (R_BORE, S0 - 1.0), (R_BORE, X_ROOF0), (P["D_SPINE_BORE"] / 2, X_ROOF0 + R_BORE - P["D_SPINE_BORE"] / 2),
                   (P["D_SPINE_BORE"] / 2, SPINE_END), (0.0, SPINE_END + P["D_SPINE_BORE"] / 2)], 72)
    hexp = m3.CrossSection.circle(HEX_R, 6).extrude(X_NA[1] - X_NA[0] + 0.5).translate([0, 0, X_NA[0] - 0.5])
    au = (body + fl) - cav - hexp
    assert str(au.status()).endswith("NoError"), au.status()
    m = tm_of(au); m.apply_transform(Z2X); return m

def drill(prefix, side, color):
    arm = build_arm(side)
    pos = (DX, side * Y_C, P["DRILL_PIV_Z"])
    add(prefix, arm, "legs", color, pos=pos, axis="y", deg=(0, P["DEPLOY_DEG"]), sgn=-1, joint="deploy",
        orient="standing on the head's flat rear face (stop face) down, no supports",
        note="Drill arm (rear pod, folds forward): round head pressed on the Ø7 D coupler (flat rear face = stop face on the hull's pivot block), bridle EYE lug on the head's outboard 5 mm (Ø%.1f, %g below the pivot: the tow pulls the pod into its deploy stop), short neck with the motor-lead channel, thrust shoulder for the auger hub, stationary bushing Ø%.1f that holds the drill gearmotor by its can" % (P["EYE_D"], P["EYE_DROP"], BUSH_D))
    want_right = side > 0                                                    # left pod right-hand (firmware DRILL_SIGN +1, harness unchanged)
    au = mirror_x_tm(taper_auger(lefthand=want_right))                       # mirroring flips the hand: build the opposite one
    add(prefix + "_auger", au, "legs", "#b23a48", parent=prefix, orient="standing on its base (point up), brim 5 mm",
        note="Tapered drill auger, %s-hand: Ø%.1f hub round the stationary motor (runs on the arm's bushing, base face on the arm shoulder), gimlet core Ø%g -> Ø%g, anchor flight Ø%g pitch %g (load face toward the motor 45 deg), point %g; the motor's M3 shaft is its spine (nut A pressed in the hub, self-tapped bore)" % ("right" if want_right else "left", HUB_D, P["D_CORE0"], P["D_CORE1"], P["FLIGHT_D"], P["D_PITCH"], P["D_TAPER"]))
    can = cylx(P["MOTOR_W"], GB0 - MB, MB) & bx(MB - 1, GB0 + 1, -P["MOTOR_H"] / 2, P["MOTOR_H"] / 2, -10, 10)
    gbx = bx(GB0, XS, -P["MOTOR_H"] / 2, P["MOTOR_H"] / 2, -P["MOTOR_W"] / 2, P["MOTOR_W"] / 2)
    add(prefix + "_motor", mirror_x(can + gbx), "electronics", "#6c757d", parent=prefix, printable=False, note="Drill gearmotor 12 x 10, gearbox 11.95, 24.27 face to back (measured): fixed in the arm, inside the auger hub")
    add(prefix + "_shaft", mirror_x(cylx(P["DSHAFT_D"], P["DSHAFT_L"], XS)), "electronics", "#d0d4d8", parent=prefix, printable=False,
        note="M3 threaded motor shaft Ø2.84 x 56.03 (measured): the auger's steel spine")
    nut = trimesh.creation.cylinder(radius=P["NUT_AF"] / math.sqrt(3), height=P["NUT_T"], sections=6); nut.apply_transform(RM(math.pi / 2, [0, 1, 0])); nut.apply_transform(TM([X_NA[1] - 0.1 - P["NUT_T"] / 2, 0, 0])); nut = tD(nut, [tcylx(3.0, 6, X_NA[0] - 1)])
    add(prefix + "_nut", mirror_x_tm(nut), "electronics", "#777", parent=prefix, printable=False, note="M3 nut A pressed into the hex pocket at the end of the hub bore")

print("drills ...")
drill("drill_leg_L", +1, "#d9534f")
drill("drill_leg_R", -1, "#c0392b")

def coupler(y0, y1, sock_depth):
    lo, hi = sorted((y0, y1))
    c = cyly(P["CPL_D"], hi - lo, lo)
    c = c - bx(-5, 5, lo - 1, hi + 1, P["CPL_D"] / 2 - 0.6, 5)
    sock = D_bore(P["SOCK_D"], sock_depth, y0 if y1 > y0 else y0 - sock_depth, flat=P["SOCK_FLAT"])   # press fit on the deploy N20 stub
    c = c - sock
    pil = cyly(P["SCREW_PILOT"], 7, y1 - 7 if y1 > y0 else y1)
    c = c - pil
    c = safe_fillet(c, c.edges().group_by(Axis.Y)[0 if y1 > y0 else -1], 0.4, "cpl")
    return c

def D_bore(d, depth, y0, flat=None):
    c = cyly(d, depth, y0)
    return c - bx(-3, 3, y0 - 1, y0 + depth + 1, d / 2 - 0.5 if flat is None else flat, 4)

DY_ = 0.0                                                    # deploy N20 on the centreline, output shaft = the pod axle
tipL = DY_ + P["N20_TIP"]; tipR = DY_ - P["N20_TIP"]
yo = Y_C + HEAD_YL[1] - CB_DEPTH + 0.3
yLi = DY_ + P["N20_BOXW"] / 2 + 0.3
cL = coupler(yLi, yo, tipL - yLi + 0.5)
yRi = DY_ - P["N20_BOXW"] / 2 - 0.3
cR = coupler(yRi, -yo, yRi - tipR + 0.5)
for nm, c in (("coupler_L", cL), ("coupler_R", cR)):
    add(nm, c, "legs", "#444", pos=(DX, 0, P["DRILL_PIV_Z"]), axis="y", deg=(0, P["DEPLOY_DEG"]), sgn=-1, joint="deploy",
        orient="lying on its D flat (layers along the axle), PPA-CF",
        note="Ø7 D coupler (PPA-CF): press-fit D socket (Ø%.2f, flat at %.2f, CA) on the deploy N20 stub, runs in the pivot block bore (the hull carries the pod), press fit into the arm head's D bore (%.2f, CA): the axle is stiff, so both pods load the one worm together; the printed pod cap and an M3 x 8 screw seat on its outer end" % (P["SOCK_D"], P["SOCK_FLAT"], P["CPL_BORE"]))

# ================================================================== ENVELOPES (not printed)
def env(name, solid, color, note, parent=None):
    add(name, solid, "electronics", color, note=note, printable=False, parent=parent, step=False)

cz = P["CELL_Z"]
for i, (xa, xb) in enumerate((X_C1, X_C2)):
    env("cell_%d" % (i + 1), cylx(P["CELL_D"], P["CELL_L"] - 0.72, xa, z=cz) + cylx(6, 0.72, xb - 0.72, z=cz),
        "#f4d35e", "18350 Ø18.66 x 36.87 incl. button (measured); the two in line on the keel, button aft (front spring B-, BM strip between, rear plate B+)")
def n20_vertical(xc, zc):
    """worm N20 with its motor pointing DOWN: worm box (9.94 along the output shaft y, 11.93 along x) centred on the output, can under it"""
    bxh, byw = P["N20_BOXH"] / 2, P["N20_BOXW"] / 2
    box = bx(xc - bxh, xc + bxh, -byw, byw, zc - P["N20_BOX"] / 2, zc + P["N20_BOX"] / 2)
    can = bx(xc - P["N20_CAN"][1] / 2, xc + P["N20_CAN"][1] / 2, -P["N20_CAN"][0] / 2, P["N20_CAN"][0] / 2, zc - P["N20_L"] + P["N20_BOX"] / 2, zc - P["N20_BOX"] / 2)
    return box + can
env("n20_leg", n20_vertical(LX, LZ), "#8e9aaf", "LEG worm N20 (the old kick motor), motor pointing down in the tray cradle, worm box in the knuckle under the 0.6 mm blister; output shaft = the leg hinge, both stubs trimmed to 5.8 mm past the box (|y| %.1f), driving the two couplers" % P["LEG_STUB"])
add("leg_shaft", D_bore_y(P["SHAFT_D"], -P["LEG_STUB"], P["LEG_STUB"]), "electronics", "#d0d4d8", parent="righting_leg", printable=False, step=False, note="leg N20 output shaft (turns with the leg), stubs trimmed to |y| %.1f" % P["LEG_STUB"])
env("n20_deploy", n20_vertical(DX, P["DRILL_PIV_Z"]), "#8e9aaf", "DEPLOY worm N20 on the pod axle, motor pointing down in the tray's rear cradle (low CG)")
add("deploy_shaft", D_bore_y(P["SHAFT_D"], -P["N20_TIP"], P["N20_TIP"]), "electronics", "#d0d4d8", pos=(DX, 0, P["DRILL_PIV_Z"]), axis="y", deg=(0, P["DEPLOY_DEG"]), sgn=-1, joint="deploy",
    printable=False, step=False, note="deploy N20 output shaft (turns with the couplers), both stubs in the couplers")
xx0, xyc, xz0 = LAY["xiao"]; xy0 = -Y_USB + 1.3
env("xiao", bx(xx0, xx0 + 17.74, xy0, xy0 + 21.45, xz0, xz0 + 1.0) + bx(XU - 4.47, XU + 4.47, -Y_USB, -Y_USB + 7.3, xz0 + 1.0, xz0 + 1.0 + 3.26)
    + bx(xx0 + 1.0, xx0 + 16.74, xy0 + 7.5, xy0 + 20.5, xz0 + 1.0, xz0 + 4.49), "#2a9d8f", "xiao 17.74 x 21.45 board (+1.3 USB-C lip), 4.49 thick (measured); USB-C to the right flank window")
for name in ("drv8833_1", "drv8833_2", "ina219", "bno085", "mp1584", "vtx", "bms_2s"):
    lx_, ly_, lz_ = BOARD[name]; x, y, z = LAY[name]
    env(name, bx(x, x + lx_, y - ly_ / 2, y + ly_ / 2, z, z + lz_), {"xiao": "#2a9d8f", "ina219": "#f3722c", "bno085": "#f8961e", "mp1584": "#577590", "vtx": "#9b5de5", "bms_2s": "#90be6d"}.get(name, "#43aa8b"),
        "%s %.2f x %.2f x %.2f (measured)" % (name, *BOARD[name]))
cx0 = P["NOSE_LIP"] + P["CAM_LENS"]; czc = P["CAM_Z"]
env("camera", bx(cx0, cx0 + P["CAM_D"] - P["CAM_LENS"], -P["CAM_W"] / 2, P["CAM_W"] / 2, czc - P["CAM_H"] / 2, czc + P["CAM_H"] / 2) + cylx(P["CAM_LENS_D"], P["CAM_LENS"], cx0 - P["CAM_LENS"], z=czc),
    "#1d3557", "Caddx Ant 13.99 x 14.59, 18.34 front to back (measured), 2.2 below the axis")

# ================================================================== HARDWARE (every bought part, so the model has everything in it)
def hw(name, solid, color, note, parent=None, pos=(0, 0, 0), axis=None, deg=(0, 0), **kw):
    add(name, solid, "electronics", color, note=note, printable=False, parent=parent, step=False, pos=pos, axis=axis, deg=deg, **kw)
def screw_y(y_head, y_tip, x, z, dhead=5.5, hh=1.8, flat=False):
    s = 1 if y_tip > y_head else -1
    if flat: y_head = y_head + s * CSINK_SUNK
    shank = cyly(2.9, abs(y_tip - y_head), min(y_head, y_tip), x=x, z=z)
    if flat:
        head = Solid.make_cone(3.0, 1.45, 1.55, Plane(origin=(x, y_head + s * 0.05, z), z_dir=(0, s, 0)))
    else:
        head = cyly(dhead, hh, y_head - hh if s > 0 else y_head, x=x, z=z)
    return shank + head
s_all = None; SCREWS = []
for xs_ in SCR_X:
    for sy in (-1, 1):
        for zz in (-P["RAD_Z"], P["RAD_Z"]):
            yh = sy * chord_half(zz, R); yt = yh - sy * (CSINK_SUNK + P["SCREW_L"])           # 8 mm from the sunk head top to the tip
            q = screw_y(yh, yt, xs_, zz, flat=True)
            s_all = q if s_all is None else s_all + q
            SCREWS.append([round(xs_, 2), sy, zz, round(yt, 3)])
P["SCREWS"] = SCREWS
hw("screws_radial", s_all, "#aab3bd", "8 x M3 x 8 countersunk radial screws, flush with the skin: 4 into the bosses on tray disc 1 (x %.1f), 4 into the bosses on the rear disc (x %.1f) (z +-3, through each half)" % SCR_X)
CAP_T = 1.5
for nm, sg in (("coupler_L", 1), ("coupler_R", -1)):
    yo2 = sg * yo
    cap = cyly(10.0, CAP_T, yo2 if sg > 0 else yo2 - CAP_T) - cyly(P["SCREW_CLR"], CAP_T + 2, (yo2 if sg > 0 else yo2 - CAP_T) - 1)
    cap = safe_fillet(cap, cap.edges().group_by(Axis.Y)[-1 if sg > 0 else 0], 0.4, "cap")
    add("pod_cap_" + nm[-1], cap, "legs", "#444", parent=nm, orient="flat on the bed",
        note="Printed pod cap Ø10 x 1.5 on the coupler end inside the arm head counterbore; the M3 x 8 screw goes through it into the coupler; it overlaps the head so the pod cannot walk off")
    yh = yo2 + sg * CAP_T
    hw(nm + "_screw", screw_y(yh, yh - sg * 8.0, 0, 0), "#555555", "M3 x 8 through the pod cap into the coupler end (turns with it)", parent=nm)
spr = cylx(9.0, 3.0, X_C1[0] - 3.6 + 0.3, z=cz)
hw("contacts_front_spring", spr, "#c0c0c0", "battery spring (B-) on disc 1's rear face, against cell 1's base")
hw("contacts_mid_strip", cylx(9.0, 0.3, X_MID[0], z=cz) + bx(X_MID[0], X_MID[1], -1.5, 1.5, cz + 4.3, cz + 9.6), "#c0c0c0",
   "BM tap: a 0.3 nickel strip disc between cell 1's button and cell 2's base, its tab up to the BMS BM wire (solder it to the tab before the cells go in)")
hw("contacts_rear_plate", cylx(9.0, 0.8, X_C2[1] + 0.1, z=cz), "#c0c0c0", "rigid contact plate (B+) on the rear disc's front face (launch pushes the cells onto it)")
for nm, sg, wide, txt in (("jst_loop", 1, 7.4, "JST-XH 2-pin power loop socket B2B-XH-A (the shorting plug is the ON switch): header recessed in the left flank seat behind the pivot block (%.1f below the split's centre so the bridle leg passes over the plug's wire loop)" % -P["LOOP_DZ"]),
                          ("jst_charge", -1, 9.9, "JST-XH 3-pin charge socket B3B-XH-A, 9.9 wide (B-, BM, B+): header recessed in the right flank seat (opening 10.2, pin pocket 7.0); plug in on the bench only")):
    hz = wide / 2; zc_ = (SEAT_Z[sg][0] + SEAT_Z[sg][1]) / 2
    hdr = bx(X_SOCK[0] + 0.15, X_SOCK[1] - 0.15, sg * 8.3, sg * 15.3, zc_ - hz, zc_ + hz)
    hw(nm, hdr, "#f5f5f5", txt)
zc_ = (SEAT_Z[1][0] + SEAT_Z[1][1]) / 2                     # ON shorting plug + a 10 mm wire loop straight out (envelope for the bridle checks)
P["PLUG_ENV"] = [[round(X_SOCK[0] + 0.15, 2), 15.3, round(zc_ - 3.7, 2)], [round(X_SOCK[1] - 0.15, 2), R + 10.0, round(zc_ + 3.7, 2)]]
hw("antenna", cylx(4.0, 22, LAY["bno085"][0], y=15.6, z=DECK1 + 2.1), "#222222", "VTX u.FL 5.8 GHz antenna on deck 1 beside the BNO085, along the left flank (PLA is transparent to 5.8 GHz)")
hw("xiao_antenna", bx(LAY["bno085"][0], LAY["bno085"][0] + 20.0, -17.2, -16.8, DECK1 + 0.5, DECK1 + 7.0), "#c8a000",
   "XIAO 2.4 GHz FPC antenna (about 20 x 12, NOT measured, folded to 6.5): stuck to the inside of the right flank beside the BNO085")
hw("capacitor", cylz(P["CAP_D"], P["CAP_L"], DECK1 + 0.05, x=X_SOCK[0] + 0.35, y=0.0), "#334488",
   "470 to 1000 uF electrolytic (modelled Ø%g x %g, the worst likely can) standing on the tray's tail key behind the deploy motor, a dab of glue; leads to the MP1584 5 V output (long leg to OUT+)" % (P["CAP_D"], P["CAP_L"]))

bp = putty_region(PUTTY_XE) - nose
P["BALLAST_G"] = round(bp.volume / 1000 * P["BAL_RHO"], 1)
add("ballast", bp, "electronics", "#7a6a55", printable=False, step=False, mass_g=P["BALLAST_G"],
    note="%.1f g of lead/tungsten putty (Harry's ballast putty, %.1f cm3 at %g g/cm3) in the nose: pressed in from the back through the spigot, front first, round and under the camera tube up to the retaining collar at x %.1f, before the nose goes on. It moves the centre of mass forward for flight (pitch and yaw)" % (P["BALLAST_G"], bp.volume / 1000, P["BAL_RHO"], PUTTY_XE))
mid = lambda xx: round((xx[0] + xx[1]) / 2, 2)
P["WIRE_PTS"] = dict(
    corr_L=[mid(X_C1), 13.0, -8.0], corr_R=[mid(X_C1), -13.0, -8.0], corr_L2=[mid(X_C2), 13.0, -8.0], corr_R2=[mid(X_C2), -13.0, -8.0],
    win_d1_L=[mid(X_D1), 13.5, -9.0], win_d1_R=[mid(X_D1), -13.5, -9.0], win_d2_L=[mid(X_D2), 13.5, -9.0], win_d2_R=[mid(X_D2), -13.5, -9.0],
    top_d1=[mid(X_D1), 0, 12.0], top_d2=[mid(X_D2), 0, 8.0],
    sock_L=[mid(X_SOCK), 8.0, 0.0], sock_R=[mid(X_SOCK), -8.0, 0.0],
    flankL=[DX - 10.5, 11.5, 16.5], flankR=[DX - 10.5, -11.5, 16.5],
    leg_tabs=[LX, 0.0, LZ - P["N20_L"] + 6.0], deploy_tabs=[DX, 0.0, P["DRILL_PIV_Z"] - P["N20_L"] + 6.0])
P["X_BOARD_BAY"] = [X_D1[1], X_D2[0]]
json.dump(dict(version=os.environ.get("KV", "v34"), P=P, ground=GROUND, parts=parts), open(os.path.join(OUT, "parts.json"), "w"), indent=1)
print("parts:", len(parts), "L =", L, "t =", round(time.time() - T0), "s")
