"""
KEDGE dart v29 generator (build123d).  Run: python3 kedge_v27.py out29   (KOV='{json}' overrides any parameter for variants)
Frame (mm): X along the hull, nose tip x=0, tail +X.  Y across, Z up.  Dart lying flat => ground z=-R_OUT.
Moving parts are built in their own LOCAL frame (pivot at origin, hinge axis = Y) and placed by parts.json.
Outputs per part: STL (print), STEP (B-rep), .npz (exact tessellation for the sweep), plus parts.json, meshes.json.
"""
import json, math, os, sys, time
import numpy as np
from build123d import *
import trimesh

OUT = sys.argv[1] if len(sys.argv) > 1 else "out27"
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
    SADDLE_X0=23.8, KICK_HUB_D=12.0, KICK_HUB_W=14.0, KICK_POLE_D=8.0, KICK_POLE_L=72.0, KICK_DEG=200.0,
    FOOT_AUGER_L=40.0, FOOT_TIP=10.0,
    # drill pair
    DRILL_X=56.0, DRILL_PIV_Z=10.0, DRILL_PIV_Y=30.5, DEPLOY_Y=None, DEPLOY_DEG=38.0,
    CPL_D=7.0, BOSS_D=11.0,
    # drill motor  M: gearbox 11.95, 24.27 face to back, shaft 2.84 x 56.03
    MOTOR_W=12.0, MOTOR_H=10.0, MOTOR_L=24.27, DSHAFT_L=56.03, DSHAFT_D=2.84,
    AUGER_L=70.0, AUGER_D=16.0, AUGER_CORE=5.5, AUGER_PITCH=11.0, AUGER_TIP=12.0,
    # F693ZZ  M: 3 x 8 x 4, flange 9.5 x 0.6 (catalogue)
    BRG_OD=8.0, BRG_W=4.0, BRG_FL_D=9.5, BRG_FL_T=0.6,
    NUT_AF=5.5, NUT_T=2.4, NYLOC_T=4.0,
    # power and boards (M)
    CELL_D=18.66, CELL_L=36.87, CELL_Y=9.6, CONTACT=3.6,
    SCREW_CLR=3.3, SCREW_PILOT=2.5, RAD_Z=3.0,
    KEEL_Z=-17.0,
)
P.update(LAYOUT="cells_fwd", DISC_T=2.0, DISC_CELL_T=3.0, LIGHT=0)
P.update(json.loads(os.environ.get("KOV", "{}")))           # variant overrides, e.g. KOV='{"AUGER_L":66,"LAYOUT":"cells_fwd"}'
R, RB, W = P["R_OUT"], P["R_BAY"], P["WALL"]
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

# ================================================================== KICK geometry (needed by the hull)
pk = P["N20_POCK"]
SX0 = P["SADDLE_X0"]
SAD_W = pk + 2 * 1.3                                        # 15.0
SAD_H = pk + 2 * 1.3
KICK_Z = R + SAD_H / 2 - 1.0                                 # saddle centre, sunk 1 mm into the skin
KICK_X = SX0 + 1.3 + P["N20_BOX"] / 2 + 0.2                  # output shaft = centre of the worm box
HUB_Y0 = SAD_W / 2 + 0.4                                     # hub inner face, 0.4 off the saddle wall
KICK_Y = HUB_Y0 + P["KICK_HUB_W"] / 2                        # pole centreline
HUB_Y1 = HUB_Y0 + P["KICK_HUB_W"]
EAR_Y0 = HUB_Y1 + 0.4
EAR_T = P["BRG_W"] - P["BRG_FL_T"] + 1.0                     # 4.4: bearing body + 1 lip... flange sits on the outer face
EAR_Y1 = EAR_Y0 + EAR_T
P.update(KICK_X=KICK_X, KICK_Y=KICK_Y, KICK_Z=KICK_Z, EAR_Y0=EAR_Y0, EAR_Y1=EAR_Y1)

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
# ================================================================== HULL
def build_hull():
    hx0, hx1 = X_HULL
    shell = cylx(2 * R, hx1 - hx0, hx0) - cylx(2 * RB, hx1 - hx0 + 2, hx0 - 1)
    # kick saddle: square pocket for the worm N20 lying along X, worm box at the nose end, open at the top
    sl = 1.3 + P["N20_L"] + 0.6 + 1.3
    saddle = rrect_x(SAD_W, SAD_H, sl, SX0, z=KICK_Z, r=3.0)
    saddle = safe_fillet(saddle, saddle.edges().group_by(Axis.X)[-1], 1.5, "saddle end")
    # ear: fin outboard of the hub carrying an F693ZZ for the hub screw
    ex0, ex1 = KICK_X - 7.5, KICK_X + 7.5
    ear_top = KICK_Z + P["BRG_FL_D"] / 2 + 2.2
    root_top = KICK_Z - P["KICK_HUB_D"] / 2 - 3.2
    ear = rbox(ex0, ex1, EAR_Y0, EAR_Y1, -0.5, ear_top, 1.2) & bx(-10, 200, -60, 60, 0, 80)
    root = rbox(ex0 - 3, ex1 + 3, 15.0, EAR_Y1, -0.5, root_top, 1.2) & bx(-10, 200, -60, 60, 0, 80)
    # drill axle bosses
    zb = P["DRILL_PIV_Z"]; pod_face = P["DRILL_PIV_Y"] - (P["MOTOR_W"] + 2.6) / 2
    yi = chord_half(zb) - 0.5
    bosses = [cyly(P["BOSS_D"], pod_face - 0.4 - yi, yi, x=P["DRILL_X"], z=zb),
              cyly(P["BOSS_D"], pod_face - 0.4 - yi, -(pod_face - 0.4), x=P["DRILL_X"], z=zb)]
    body = shell + saddle + ear + root
    for s_ in (-1, 1):                                       # tombstone: boss carried down to the split plane, prints flat
        y0_, y1_ = (yi, pod_face - 0.4) if s_ > 0 else (-(pod_face - 0.4), -yi)
        bosses.append(bx(P["DRILL_X"] - P["BOSS_D"] / 2, P["DRILL_X"] + P["BOSS_D"] / 2, y0_, y1_, 0.0, zb))
    for b in bosses:
        body = body + b
    body = body - cylx(2 * RB, hx1 - hx0 + 2, hx0 - 1)          # nothing intrudes into the bay
    # internal rib that sits in the nose spigot groove: the closed clamshell locks the nose axially
    body = body + (cylx(2 * RB + 0.2, 2.0, NOSE_LOCK_X) - cylx(2 * (RB - 1.0), 4.0, NOSE_LOCK_X - 1))
    cuts = []
    # N20 pocket (open top) + strap groove + shaft holes both sides (biax worm, 10 proud each way)
    px0 = SX0 + 1.3
    cuts.append(bx(px0, px0 + P["N20_L"] + 0.6, -pk / 2 - CLR, pk / 2 + CLR, KICK_Z - pk / 2 - CLR, KICK_Z + 30))
    for s_ in (-1, 1):                                       # zip-tie slots beside the saddle's tail end: tie runs over the motor and under the skin
        cuts.append(bx(px0 + P["N20_L"] - 8, px0 + P["N20_L"] - 4, s_ * (SAD_W / 2 + 0.6) - 1.25, s_ * (SAD_W / 2 + 0.6) + 1.25, 10, 30))
    cuts.append(tear_y(P["SHAFT_D"] + 1.2, 40, -20, x=KICK_X, z=KICK_Z))
    # bay under the saddle: wire hole for the kick N20, and foot-wire hole beside it
    cuts.append(cylz(5.0, 12, R - 6, x=px0 + P["N20_L"] - 4, y=0))
    cuts.append(Pos(42, KICK_Y - 1, 0) * Rot(0, 0, 0) * cylz(4.0, 30, 8))
    # ear bearing: Ø8 bore through, flange recess on the outer face
    cuts.append(tear_y(P["BRG_OD"] + 0.15, EAR_T + 2, EAR_Y0 - 1, x=KICK_X, z=KICK_Z))
    cuts.append(cyly(P["BRG_FL_D"] + 0.3, P["BRG_FL_T"] + 0.05, EAR_Y1 - P["BRG_FL_T"], x=KICK_X, z=KICK_Z))
    # bosses: coupler bores
    cuts.append(tear_y(P["CPL_D"] + 0.4, 2 * R + 20, -R - 10, x=P["DRILL_X"], z=zb))
    # drill-motor wire holes on both flanks just ahead of the bosses
    for s in (-1, 1):
        cuts.append(Pos(P["DRILL_X"] - 9, s * 19.5, 9.0) * Cylinder(2.0, 8, rotation=(90, 0, 0)))
    # radial screws: disc 1 (x mid) and tail spigot, z = +-3 both sides
    for xs in ((X_D1[0] + X_D1[1]) / 2, (X_TAILSP[0] + X_TAILSP[1]) / 2):
        for s in (-1, 1):
            for zz in (-P["RAD_Z"], P["RAD_Z"]):
                cuts.append(cyly(P["SCREW_CLR"], 14, (RB - 2) if s > 0 else -(RB - 2) - 14 + 0, x=xs, z=zz))
                cuts += csink(xs, s, zz)
    # the +y,z=+3 disc-1 screw goes through the ear root: give it a clear path to the outside
    cuts.append(cyly(P["SCREW_CLR"], 12, EAR_Y1 - 10, x=(X_D1[0] + X_D1[1]) / 2, z=P["RAD_Z"]))
    for c in cuts:
        body = body - c
    return body

print("hull ...")
hull = build_hull()
top = hull & bx(-10, L + 10, -60, 60, 0, 80)
bot = hull & bx(-10, L + 10, -60, 60, -80, 0)
add("hull_top", top, "hull", "#9aa5b1", orient="split face down (on the bed), saddle up",
    note="Clamshell upper half: skin, kick saddle (worm N20 drops in, strap groove, Ø3 shaft holes both sides), ear fin with an F693ZZ for the hub screw, both drill-axle bosses (Ø7 coupler bores, teardrop roofs), wire holes. Radial M3 clearance at z=+3 into tray disc 1 and the tail spigot")
add("hull_bottom", bot, "hull", "#7d8794", orient="split face down, belly up",
    note="Clamshell lower half: plain Ø42 cylinder (launcher cradle surface). Radial M3 clearance at z=-3 into tray disc 1 and the tail spigot")

# ================================================================== NOSE (hollow, 1.5 walls)
def build_nose():
    nl, r0 = P["NOSE_L"], P["NOSE_R0"]
    outer = Solid.make_cone(r0, R, nl, Plane.YZ)               # along +X from x=0
    spig = cylx(2 * RB - 2 * CLR, P["NOSE_SPIG"], nl - 0.01)
    body = outer + spig
    body = safe_fillet(body, body.edges().group_by(Axis.X)[0], 1.5, "nose tip")
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
    body = body - bx(P["NOSE_LIP"] + P["CAM_LENS"], nl + P["NOSE_SPIG"] + 5, -cw / 2, cw / 2, -ch / 2, ch / 2)
    body = body - cylx(P["CAM_HOLE"], P["NOSE_LIP"] + P["CAM_LENS"] + 1, -1)
    body = body - (cylx(2 * R + 2, 2.4, NOSE_LOCK_X - 0.2) - cylx(2 * (RB - 1.0) - 0.4, 4, NOSE_LOCK_X - 1))   # lock groove for the hull rib
    # cable slot at the back of the camera tube (connector exits the back)
    return body

print("nose ...")
nose = build_nose()
add("nose", nose, "hull", "#b8c0ca", orient="tip down on the bed (flat Ø22), spigot up",
    note="Blunt hollow nose: Ø22 flat tip with the Ø9.8 lens hole, Caddx Ant pocket 14.4 x 15.0 (slides in from the back, 1.2 lip + lens recess hold it), 1.5 walls; spigot with a groove that the hull rib locks when the clamshell closes")

# ================================================================== TAIL CAP (screwed, carries the tether)
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
    for s_ in (-1, 1):                                       # tether: in one hole, round the 4.4 mm web, out the other
        body = body - cylx(4.2, END_T + 2, L - END_T - 1, y=s_ * 4.3)
        body = body - Solid.make_cone(3.1, 2.1, 1.0, Plane.YZ.offset(L - 1.0)).moved(Location((0, s_ * 4.3, 0))) if False else body
    body = body - cylx(2 * RB - 2 * CLR - 6.0, P["TAIL_SPIG"] + 1.5 + 0.01, x0 - 0.01)
    # radial pilot holes for the 4 hull screws
    xs = (X_TAILSP[0] + X_TAILSP[1]) / 2
    for s in (-1, 1):
        for zz in (-P["RAD_Z"], P["RAD_Z"]):
            body = body - cyly(P["SCREW_PILOT"], 7, (RB - 6) if s > 0 else -(RB + 1), x=xs, z=zz)
            body = body - seat(xs, s, zz)                 # clearance for the sunk screw head in the spigot ring
    # JST-XH 3-pin charge socket + 2-pin power loop window (bottom), u.FL antenna slot (top)
    body = body - bx(xc + 0.5, L - END_T, -9.8, 9.8, -R - 1, -12.5)
    body = body - bx(xc + 0.5, L - END_T, -2.0, 2.0, 12.0, R + 1)
    return body

print("tail ...")
tail = build_tail()
add("tail_cap", tail, "hull", "#b8c0ca", orient="end face down, spigot up (no supports)",
    note="Tail cap: carries the tether (two Ø4.2 holes through a 5 mm end wall: line goes in one, round the web, out the other, knot), 4 radial M3 from the hull into its spigot ring so the tether load goes into the hull, window underneath for the JST-XH charge socket and the 2-pin power loop, u.FL antenna slot on top")

# ================================================================== TRAY: one printed spine, nose to tail
DECK_A, DECK_B, DECK_C = -13.8, -5.0, 4.0          # floor TOP surfaces in the board zone
FT = 1.2                                            # floor / deck thickness
KZ = P["KEEL_Z"]

BOARD = {   # name: (Lx, Wy, Hz) measured; headers and terminal removed
    "xiao": (21.45, 17.74, 4.49), "drv8833_1": (20.31, 12.70, 3.5), "drv8833_2": (20.31, 12.70, 3.5),
    "drv8833_3": (20.31, 12.70, 3.5), "ina219": (25.69, 20.32, 4.70), "bno085": (25.72, 22.77, 4.77),
    "mp1584": (22.26, 16.96, 3.98), "vtx": (42.25, 19.91, 6.01), "bms_2s": (40.8, 8.0, 2.1),
}
xb0 = X_BOARD[0] + 0.4
LAY = {}
LAY["bno085"] = (xb0, 0.0, DECK_A)
gB = 0.8; wB = BOARD["xiao"][1] + BOARD["drv8833_1"][1] + gB
LAY["xiao"] = (xb0, -wB / 2 + BOARD["xiao"][1] / 2, DECK_B)
LAY["drv8833_1"] = (xb0, wB / 2 - BOARD["drv8833_1"][1] / 2, DECK_B)
wC = BOARD["drv8833_2"][1] + BOARD["ina219"][1] + gB
LAY["drv8833_2"] = (xb0, -wC / 2 + BOARD["drv8833_2"][1] / 2, DECK_C)
LAY["ina219"] = (xb0, wC / 2 - BOARD["ina219"][1] / 2, DECK_C)
SLED_X0 = X_D1[1] + 0.5
SHELF_Z = -5.0
LAY["mp1584"] = (SLED_X0, 0.9 + BOARD["mp1584"][1] / 2, SHELF_Z)      # 5 V buck on the +y shelf beside the deploy cradle
LAY["drv8833_3"] = (SLED_X0 + 0.5, 0.0, KZ)                              # DRV#3 on the keel under it
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
                if not (s > 0 and zz > 0):                 # the ear-root screw is a pan head
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
    d1 = disc(*X_D1, screws=True, windows=wins(*X_D1, top_off=4.0, extra=[bx(X_D1[0] - 1, X_D1[1] + 1, dyy - pkc, dyy + pkc, zcc - pkc, 30)]))   # deploy N20 body passes through
    vn = lambda x0, x1: [bx(x0 - 1, x1 + 1, -4.6, 4.6, KZ, KZ + 2.8)]                                  # BMS stick channel
    def cdisc(xx):                                   # discs bounding the cell bay get the VTX notch and the BMS channel
        if xx in (X_CF, X_CR): return disc(*xx, windows=wins(*xx, vn(*xx), top_w=10.6, top_z=VTX_Z - 1.2))
        return disc(*xx, windows=wins(*xx))
    d2, d3, d4 = cdisc(X_D2), cdisc(X_D3), cdisc(X_D4)
    parts_ += [d1, d2, d3, d4]
    # board zone floors (bridge between disc 2 and disc 3)
    for zt in (DECK_A, DECK_B, DECK_C):
        p = plate(X_BF[1] - 0.01, X_BR[0] + 0.01, zt)
        hw_ = 5.5 if P["LIGHT"] else 4.0
        p = p - bx(X_BOARD[0] + (3 if P["LIGHT"] else 4), X_BOARD[1] - (3 if P["LIGHT"] else 4), -hw_, hw_, zt - 2, zt + 1)       # lightening / wire slot
        parts_.append(p)
    # deck A sits on posts from the keel
    for s in (-1, 1):
        parts_.append(bx(X_BOARD[0] + 8, X_BOARD[0] + 10, s * 5 - 1, s * 5 + 1, KZ - 0.1, DECK_A - FT + 0.1))
    # sled: deploy-motor cradle (cantilever from disc 1), DRV#3 shelf with a rear wall standing on the keel
    dy = P["DEPLOY_Y"]; zc = P["DRILL_PIV_Z"]; pkh = pk / 2 + CLR
    cx0, cx1 = X_D1[1] - 0.01, P["DRILL_X"] - P["CPL_D"] / 2 - 0.6
    cradle = bx(cx0, cx1, dy - pkh - 1.2, dy + pkh + 1.2, zc - pkh - 1.2, zc + 2.0)
    cradle = cradle - bx(cx0 - 1, cx1 + 1, dy - pkh, dy + pkh, zc - pkh, zc + 10)
    parts_.append(bx(cx0, P["DRILL_X"] - 1.2, dy - pkh - 1.2, dy + pkh + 1.2, zc - pkh - 1.2, zc - pkh))            # cradle floor runs on to the rear wall (bridge)
    cradle = cradle - bx(cx0 + 8, cx0 + 11, -30, 30, zc - pkh - 2, zc - pkh + 0.6)    # zip-tie groove under the cradle
    parts_.append(cradle)
    sh_x1 = P["DRILL_X"] - 1.2                                # rear wall x 53.6..54.8: below the couplers (z<3.5) and the worm box (z>1)
    shelf = plate(cx0, sh_x1, SHELF_Z, ylim=(0.5, 30))
    parts_.append(shelf)
    parts_.append(bx(sh_x1 - 1.2, sh_x1, dy - pkh - 1.2, chord_half(SHELF_Z) - 1.0, KZ - 0.1, SHELF_Z - 0.01))   # rear wall (shelf)
    parts_.append(bx(sh_x1 - 1.2, sh_x1, dy - pkh - 1.2, dy + pkh + 1.2, KZ - 0.1, zc - pkh - 0.01))             # rear wall up to the cradle floor
    # cell cradle: two saddles on the keel + VTX deck over the cells
    cy, cr = P["CELL_Y"], P["CELL_D"] / 2 + CLR
    for xs_ in (X_CELL[0] + 8, X_CELL[1] - 10):
        s = bx(xs_, xs_ + 2.0, -RB + 2, RB - 2, KZ - 0.1, -4.0)
        s = s - cylx(2 * cr, 4, xs_ - 1, y=cy) - cylx(2 * cr, 4, xs_ - 1, y=-cy)
        s = s - bx(xs_ - 1, xs_ + 3, -4.6, 4.6, KZ - 1, KZ + 2.8)                  # BMS stick channel
        s = s & cylx(2 * RB - 2 * CLR, 4, xs_ - 1)
        parts_.append(s)
    vdeck = plate(X_CF[0] - 0.01, X_CR[1] + 0.01, VTX_Z, t=0.8, ylim=(-10.8, 10.8))
    parts_.append(vdeck)
    t = parts_[0]
    for p in parts_[1:]:
        t = t + p
    t = t & cylx(2 * RB - 2 * CLR, L, -1)
    t = t & bx(0, L, -30, 30, KZ - 1.2, 30)                  # flat bottom = the keel plane, prints keel down
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
    fl = trimesh.boolean.intersection([fl, tbox(0.0, length + 5, -20, 20, -20, 20)], engine="manifold")
    if lefthand:
        fl.apply_transform(reflection_matrix([0, 0, 0], [0, 1, 0])); fl.invert() if fl.volume < 0 else None; fl.fix_normals()
    ms = [core, tip, fl]
    if base_boss: ms.append(tcylx(5.0, 1.1, -0.8, n=48))   # runs 0.3 into the core: no coplanar end faces (STEP stays valid)
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

FOOT_YC = P["KICK_HUB_W"] / 2 - (P["N20_POCK"] + 2.6) / 2           # foot housing / foot N20 centreline (local y); outboard faces flush = print datum
def build_kick():
    hub = cyly(P["KICK_HUB_D"], P["KICK_HUB_W"], -P["KICK_HUB_W"] / 2)
    hub = safe_fillet(hub, hub.edges(), 0.8, "hub")
    pole_x0 = P["KICK_HUB_D"] / 2 - 1.0
    hw2 = P["KICK_HUB_W"] / 2
    pole = cylx(P["KICK_POLE_D"], P["KICK_POLE_L"], pole_x0) + bx(pole_x0, pole_x0 + P["KICK_POLE_L"], 0, hw2, -P["KICK_POLE_D"] / 2, P["KICK_POLE_D"] / 2)
    xe = pole_x0 + P["KICK_POLE_L"]
    # foot housing: square pocket for the foot worm N20 lying in line with the pole, output shaft along local +Z
    fh = pk + 2.6
    fx0 = xe - 4
    house = rrect_x(fh, fh, P["N20_L"] + 0.6 + 2.6 + 4, fx0, y=FOOT_YC, r=2.5)
    house = safe_fillet(house, house.edges().group_by(Axis.X)[-1], 1.5, "foot end")
    elbow = bx(fx0 - 4, fx0 + 1, -P["KICK_POLE_D"] / 2 - 1, hw2, -P["KICK_POLE_D"] / 2 - 1, P["KICK_POLE_D"] / 2 + 1)
    leg = hub + pole + house + elbow
    leg = safe_fillet(leg, leg.edges().filter_by(lambda e: True).group_by(Axis.X)[0], 0.5, "x") if False else leg
    fbox = fx0 + 4 + 1.3 + P["N20_L"] + 0.6 - P["N20_BOX"] / 2 - 0.2      # foot output-shaft x (box at the far end)
    cuts = [
        bx(fx0 + 4 + 1.3, fx0 + 4 + 1.3 + P["N20_L"] + 0.6, -20, FOOT_YC + pk / 2 + CLR, -pk / 2 - CLR, pk / 2 + CLR),   # pocket, open on -y (inboard; up when printing)
        cylz(P["SHAFT_D"] + 1.0, 12, pk / 2 - 1, x=fbox, y=FOOT_YC),                                         # output shaft up through the top wall
        cylz(P["SHAFT_D"] + 1.0, 4, -pk / 2 - 2.5, x=fbox, y=FOOT_YC),                                                  # trimmed lower stub clearance
        bx(fx0 + 15, fx0 + 18, -20, 20, -fh / 2 - 1, -fh / 2 + 0.8),                                         # strap groove
        cylx(4.6, P["KICK_POLE_L"] + 6, pole_x0 + 3),                                                        # wire channel through the pole
        Pos(pole_x0 + 6, -3, 0) * Cylinder(1.9, 8, rotation=(90, 0, 0)),                                      # wire exit near the hub (toward the hull)
        Pos(0, 0, 0) * D_bore(P["SHAFT_D"] + 0.25, 9.6, -P["KICK_HUB_W"] / 2 - 0.5),                       # D-bore on the kick N20 shaft
        cyly(P["SCREW_PILOT"], 5.9, P["KICK_HUB_W"] / 2 - 5.9 + 0.2),   # M3 pilot from the outer end (an M3 washer spaces the hub off the bearing)
    ]
    if P["LIGHT"]:                                 # pockets in the keel from the flat (bed) face, ribs every 13 mm
        for xp in np.arange(pole_x0 + 8, xe - 14, 13.0):
            cuts.append(bx(xp, xp + 10, P["KICK_POLE_D"] / 2 + 0.6, P["KICK_HUB_W"] / 2 + 1, -2.6, 2.6))
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
    note="Kick leg: Ø12 x 14 hub, D-bore on the worm N20 output shaft (1:1 pivot), M3 x 10 screw from outside through the ear F693ZZ into the hub end; Ø8 pole with a Ø4.6 wire channel, foot housing with the foot worm N20 in line, output shaft up when folded / down when deployed")

ztop_h = pk / 2 + 1.3
fa = screw_auger(P["AUGER_D"], P["FOOT_AUGER_L"], P["AUGER_PITCH"], P["AUGER_CORE"], P["FOOT_TIP"], lefthand=False, dbore=P["SHAFT_D"] + 0.15)
fa.apply_transform(RM(-math.pi / 2, [0, 1, 0]))            # +X -> +Z
fa.apply_transform(TM([FBOX, FOOT_YC, ztop_h + 0.8]))
add("foot_auger", fa, "legs", "#b23a48", parent="kick_leg", orient="base down",
    note="Foot auger Ø14 x 40, right-hand flight, D-bore on the foot N20 output shaft, M3 grub screw in the hub")

# ================================================================== DRILL PODS (local: pivot at origin, axis Y; folded along +X)
HW, HH = P["MOTOR_W"] + 2 * CLR + 2.4, P["MOTOR_H"] + 2 * CLR + 2.4      # 14.8 x 12.8 housing (1.2 walls)
XS = 4.4 + P["MOTOR_L"]                                    # motor face (motor back clears the Ø7 coupler)
XA = XS + P["NUT_T"] + 0.2 + P["BRG_W"] + 0.6 + (P["BRG_W"] - P["BRG_FL_T"])   # nose end face
P.update(POD_XS=XS, POD_XA=XA)

def build_pod():
    head = cyly(HH, HW, -HW / 2) + bx(-HH / 2, 0, -HW / 2, HW / 2, -HH / 2, 0)     # flat-bottomed head: prints upright, no support
    housing = rrect_x(HW, HH, XA + 1, -1, r=2.5)
    housing = safe_fillet(housing, housing.edges().group_by(Axis.X)[-1], 1.6, "pod nose")
    pod = head + housing
    cuts = [
        D_bore(P["CPL_D"] + 0.25, HW + 2, -HW / 2 - 1).rotate(Axis.Y, 0),                               # coupler D bore
        bx(XS - P["MOTOR_L"] - 0.3, XS + P["NUT_T"] + 0.2, -P["MOTOR_W"] / 2 - CLR, P["MOTOR_W"] / 2 + CLR,
           -P["MOTOR_H"] / 2 - CLR, 20),                                                                  # motor pocket + nut gap, open top
        bx(XS - 13, XS - 10, -20, 20, -HH / 2 - 1, -HH / 2 + 0.8),                                        # zip-tie groove
        cylx(P["SHAFT_D"] + 0.8, XA - XS + 2, XS - 1),                                                    # shaft clearance
        cylx(P["BRG_OD"] + 0.15, P["BRG_W"] - P["BRG_FL_T"] + 0.05, XS + P["NUT_T"] + 0.2 + P["BRG_FL_T"]),  # inner bearing body
        cylx(P["BRG_FL_D"] + 0.3, P["BRG_FL_T"] + 0.05, XS + P["NUT_T"] + 0.2 - 0.05),                    # inner flange (faces the motor)
        cylx(P["BRG_OD"] + 0.15, P["BRG_W"] - P["BRG_FL_T"] + 0.05, XA - (P["BRG_W"] - P["BRG_FL_T"])),  # outer bearing body (flange on the nose face)
    ]
    for c in cuts:
        pod = pod - c
    return pod

def D_bore_y_flat_z(d, l, y0):
    return D_bore(d, l, y0)

AUG_X0 = XA + P["BRG_FL_T"]                               # auger base boss touches the outer inner race
SHAFT_END = XS + P["DSHAFT_L"]
NUT_X = SHAFT_END - P["NYLOC_T"] + 0.6 - AUG_X0           # nyloc so the shaft end sits 0.6 short of the nut's far face

def drill(prefix, side, color):
    pod = build_pod()
    pos = (P["DRILL_X"], side * P["DRILL_PIV_Y"], P["DRILL_PIV_Z"])
    add(prefix, pod, "legs", color, pos=pos, axis="y", deg=(0, P["DEPLOY_DEG"]), orient="upright, motor pocket open to the top (as installed folded)",
        note="Drill pod: round clevis head on the Ø7 D coupler, 14.8 x 12.8 housing (motor drops in from the top, zip-tie groove), nut gap, inner F693ZZ (flange toward the motor), outer F693ZZ (flange on the nose face): the shaft nut against the inner race takes the auger's pull, the auger base against the outer race takes push. Gearbox carries torque only")
    lh = side < 0
    au = screw_auger(P["AUGER_D"], P["AUGER_L"], P["AUGER_PITCH"], P["AUGER_CORE"], P["AUGER_TIP"], lefthand=lh,
                     bore=P["DSHAFT_D"] + 0.45, nut_x=NUT_X, base_boss=True)
    au.apply_transform(TM([AUG_X0 + 0.8, 0, 0]))
    add(prefix + "_auger", au, "legs", "#b23a48", parent=prefix, orient="base down, nut slot sideways",
        note="Auger Ø14 x %g, %s-hand flight (the pair counter-rotates, so the two are mirror images and both screw IN), Ø5.5 core, captive M3 nyloc in a side slot near the point, base boss bears on the outer bearing's inner race" % (P["AUGER_L"], "left" if lh else "right"))
    env = lambda s: s
    add(prefix + "_motor", bx(XS - P["MOTOR_L"], XS, -P["MOTOR_W"] / 2, P["MOTOR_W"] / 2, -P["MOTOR_H"] / 2, P["MOTOR_H"] / 2),
        "electronics", "#6c757d", parent=prefix, printable=False, note="Drill motor 12 x 10 x 24.27 (measured)")
    add(prefix + "_shaft", cylx(P["DSHAFT_D"], P["DSHAFT_L"], XS), "electronics", "#d0d4d8", parent=prefix, printable=False,
        note="M3 threaded motor shaft Ø2.84 x 56.03 (measured)")
    xb1 = XS + P["NUT_T"] + 0.2
    b1 = cylx(P["BRG_FL_D"], P["BRG_FL_T"], xb1) + cylx(P["BRG_OD"], P["BRG_W"] - P["BRG_FL_T"], xb1 + P["BRG_FL_T"]) - cylx(3.0, 6, xb1 - 1)
    b2 = cylx(P["BRG_OD"], P["BRG_W"] - P["BRG_FL_T"], XA - (P["BRG_W"] - P["BRG_FL_T"])) + cylx(P["BRG_FL_D"], P["BRG_FL_T"], XA) - cylx(3.0, 8, XA - 5)
    add(prefix + "_brg1", b1, "electronics", "#9aa5b1", parent=prefix, printable=False, note="F693ZZ inner, flange toward the motor")
    add(prefix + "_brg2", b2, "electronics", "#9aa5b1", parent=prefix, printable=False, note="F693ZZ outer, flange on the nose face")
    nut = extrude(Plane.YZ.offset(XS + 0.1) * RegularPolygon(P["NUT_AF"] / 2, 6, major_radius=False), amount=P["NUT_T"]) - cylx(3, 4, XS - 1)
    add(prefix + "_nut", nut, "electronics", "#777", parent=prefix, printable=False, note="M3 nut on the shaft, tightened against the inner bearing's inner race (threadlocker)")
    ny = trimesh.creation.cylinder(radius=P["NUT_AF"] / math.sqrt(3), height=P["NYLOC_T"], sections=6); ny.apply_transform(RM(math.pi / 2, [0, 1, 0])); ny.apply_transform(TM([AUG_X0 + 0.8 + NUT_X + 0.15 + P["NYLOC_T"] / 2, 0, 0])); ny = tD(ny, [tcylx(3.0, 8, AUG_X0 + NUT_X - 2)])
    add(prefix + "_nyloc", ny, "electronics", "#777", parent=prefix, printable=False, note="M3 nyloc captive in the auger, threadlocker on it")

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
yo = P["DRILL_PIV_Y"] + HW / 2 - 0.3
cL = coupler(P["DEPLOY_Y"] + pk / 2 + 0.3 - 0.0 - (pk / 2 - P["N20_BOX"] / 2) + 0.0, yo, True, 0)
yLi = P["DEPLOY_Y"] + P["N20_BOX"] / 2 + 0.3
cL = coupler(yLi, yo, True, tipL - yLi + 0.5)
yRi = P["DEPLOY_Y"] - P["N20_BOX"] / 2 - 0.3
cR = coupler(yRi, -yo, True, yRi - tipR + 0.5)
for nm, c in (("coupler_L", cL), ("coupler_R", cR)):
    add(nm, c.moved(Location((0, 0, 0))), "legs", "#444", pos=(P["DRILL_X"], 0, P["DRILL_PIV_Z"]), axis="y", deg=(0, P["DEPLOY_DEG"]),
        orient="standing on its socket end",
        note="Ø7 D coupler: D-socket on the deploy N20 output stub, runs through the hull boss into the pod head; M3 x 8 + washer in its outer end clamps the pod on")

# ================================================================== ENVELOPES (not printed)
def env(name, solid, color, note, parent=None):
    add(name, solid, "electronics", color, note=note, printable=False, parent=parent, step=False)

for i, s in enumerate((-1, 1)):
    env("cell_%d" % (i + 1), cylx(P["CELL_D"], P["CELL_L"] - 0.72, X_CELL[1] - 0.8 - P["CELL_L"], y=s * P["CELL_Y"]) + cylx(6, 0.72, X_CELL[1] - 0.8 - 0.72, y=s * P["CELL_Y"]),
        "#f4d35e", "18350 Ø18.66 x 36.87 incl. button (measured), 2S, contacts both ends")
px0 = SX0 + 1.3
env("n20_kick", bx(px0, px0 + P["N20_BOX"], -6, 6, KICK_Z - 6, KICK_Z + 6) + bx(px0 + P["N20_BOX"], px0 + P["N20_L"], -6, 6, KICK_Z - 5, KICK_Z + 5),
    "#8e9aaf", "KICK worm N20 (33.04 long), worm box at the nose end; output shaft across Y = the kick pivot")
add("kick_shaft", D_bore(P["SHAFT_D"], 2 * P["N20_TIP"], -P["N20_TIP"] - KICK_Y), "electronics", "#d0d4d8", parent="kick_leg", printable=False, step=False, note="kick N20 output shaft (turns with the leg), 29.97 tip to tip")
dxx, dzz, dy = P["DRILL_X"], P["DRILL_PIV_Z"], P["DEPLOY_Y"]
env("n20_deploy", bx(dxx - 6, dxx + 6, dy - 6, dy + 6, dzz - 6, dzz + 6) + bx(dxx - 6 - (P["N20_L"] - 12), dxx - 6, dy - 6, dy + 5, dzz - 5, dzz + 5),
    "#8e9aaf", "DEPLOY worm N20 on the axle line, body forward in the tray cradle")
add("deploy_shaft", D_bore(P["SHAFT_D"], 2 * P["N20_TIP"], dy - P["N20_TIP"]), "electronics", "#d0d4d8", pos=(dxx, 0, dzz), axis="y", deg=(0, P["DEPLOY_DEG"]), printable=False, step=False, note="deploy N20 output shaft (turns with the couplers), both stubs in the couplers")
for name in ("xiao", "drv8833_1", "drv8833_2", "drv8833_3", "ina219", "bno085", "mp1584", "vtx", "bms_2s"):
    lx, ly, lz = BOARD[name]; x, y, z = LAYOUT_ = LAY[name]
    env(name, bx(x, x + lx, y - ly / 2, y + ly / 2, z, z + lz), {"xiao": "#2a9d8f", "ina219": "#f3722c", "bno085": "#f8961e", "mp1584": "#577590", "vtx": "#9b5de5", "bms_2s": "#90be6d"}.get(name, "#43aa8b"),
        "%s %.2f x %.2f x %.2f (measured)" % (name, lx, ly, lz))
cx0 = P["NOSE_LIP"] + P["CAM_LENS"]
env("camera", bx(cx0, cx0 + P["CAM_D"] - P["CAM_LENS"], -P["CAM_W"] / 2, P["CAM_W"] / 2, -P["CAM_H"] / 2, P["CAM_H"] / 2) + cylx(P["CAM_LENS_D"], P["CAM_LENS"], cx0 - P["CAM_LENS"]),
    "#1d3557", "Caddx Ant 13.99 x 14.59, 18.34 front to back (measured)")
fx_p = FX0 + 4 + 1.3
env("n20_foot", bx(fx_p, fx_p + P["N20_L"] - P["N20_BOX"], FOOT_YC - 5, FOOT_YC + 5, -5, 5) + bx(fx_p + P["N20_L"] - P["N20_BOX"], fx_p + P["N20_L"], FOOT_YC - 6, FOOT_YC + 6, -6, 6),
    "#8e9aaf", "FOOT worm N20 in the housing, output up when folded (lower stub trimmed to 2 mm or single-shaft version)", parent="kick_leg")
add("foot_shaft", cylz(P["SHAFT_D"], 9.0, 6.0, x=FBOX, y=FOOT_YC) - bx(FBOX - 3, FBOX - P["SHAFT_D"] / 2 + 0.5, FOOT_YC - 3, FOOT_YC + 3, 0, 20), "electronics", "#d0d4d8", parent="kick_leg", printable=False, step=False, note="foot N20 output shaft, 9 proud, D flat facing the hull nose side")


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
for (sy, zz) in ((+1, -P["RAD_Z"]), (-1, P["RAD_Z"]), (-1, -P["RAD_Z"])):
    yh = sy * chord_half(zz, R) ; scr.append(screw_y(yh, yh - sy * 8, xs1, zz, flat=True))
for sy in (-1, 1):
    for zz in (-P["RAD_Z"], P["RAD_Z"]):
        yh = sy * chord_half(zz, R); scr.append(screw_y(yh, yh - sy * 8, xst, zz, flat=True))
s_all = scr[0]
for q in scr[1:]: s_all = s_all + q
hw("screws_radial", s_all, "#555555", "7 x M3 x 8 countersunk (flat-head) radial screws, flush with the skin: 3 into tray disc 1, 4 into the tail spigot")
hw("screw_ear_root", screw_y(EAR_Y1, EAR_Y1 - 12, xs1, P["RAD_Z"]), "#555555", "M3 x 12 through the ear root into tray disc 1")
eb = cyly(P["BRG_FL_D"], P["BRG_FL_T"], EAR_Y1 - P["BRG_FL_T"], x=KICK_X, z=KICK_Z) + cyly(P["BRG_OD"], P["BRG_W"] - P["BRG_FL_T"], EAR_Y1 - P["BRG_W"], x=KICK_X, z=KICK_Z) - cyly(3.0, 8, EAR_Y1 - 6, x=KICK_X, z=KICK_Z)
hw("brg_ear", eb, "#9aa5b1", "F693ZZ in the ear, flange on the outer face")
yl0 = P["KICK_HUB_W"] / 2
hw("kick_hub_screw", screw_y(EAR_Y1 - KICK_Y, yl0 - 5.7, 0, 0), "#555555", "M3 x 10 through the ear bearing into the hub end (turns with the leg)", parent="kick_leg")
hw("kick_washer", cyly(6.0, 0.4, yl0 + 0.02, x=0, z=0) - cyly(3.2, 2, yl0 - 0.5), "#888888", "M3 washer between hub and ear bearing inner race", parent="kick_leg")
yo_ = P["DRILL_PIV_Y"] + HW / 2
for nm, sg in (("coupler_L", 1), ("coupler_R", -1)):
    yo2 = sg * yo_
    hw(nm + "_screw", screw_y(yo2 + sg * 0.6, yo2 - sg * 6.5, 0, 0), "#555555", "M3 x 8 + washer clamps the pod onto the coupler (turns with it)", parent=nm)
    hw(nm + "_washer", cyly(7.0, 0.6, yo2 if sg > 0 else yo2 - 0.6, x=0, z=0) - cyly(3.2, 2, yo2 - 1), "#888888", "M3 washer", parent=nm)
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
hw("jst_charge", bx(xc_ + 0.3, xc_ + 6.05, -9.4, 0.5, -18.6, -12.8), "#f5f5f5", "JST-XH 3-pin charge socket in the tail window")
hw("jst_loop", bx(xc_ + 0.3, xc_ + 6.05, 1.0, 8.4, -18.6, -12.8), "#f5f5f5", "JST-XH 2-pin power loop socket + shorting plug in the tail window")
hw("antenna", cylx(4.0, 22, X_TAILSP[0] - 14, z=14.5) + bx(L - 7, L - 6.95 + 0.01, -1.9, 1.9, 13, 19) * 0 if False else cylx(4.0, 22, X_TAILSP[0] - 16, z=12.8), "#222222", "u.FL dipole along the top, out through the tail slot")
cx_ = SLED_X0 + 0.5
hw("capacitor", cylx(6.3, 11.0, cx_, y=9.4, z=2.6), "#334488", "470 to 1000 uF across the MP1584 5 V output (long leg to OUT+)")
# ballast: 15 g of steel shot + epoxy, filling the nose cavity from the tip
nl_ = P["NOSE_L"]
env_ = Solid.make_cone(P["NOSE_R0"], R, nl_, Plane.YZ) + cylx(2 * RB - 2 * CLR, P["NOSE_SPIG"], nl_ - 0.01)
cav = env_ - FIX["nose"] - bx(P["NOSE_LIP"] + P["CAM_LENS"] - 0.2, nl_ + P["NOSE_SPIG"] + 5, -(P["CAM_W"] / 2 + 1.6), P["CAM_W"] / 2 + 1.6, -(P["CAM_H"] / 2 + 1.6), P["CAM_H"] / 2 + 1.6) - cylx(P["CAM_HOLE"] + 0.4, 6, -1)
target = 15.0 / 5.0 * 1000          # 15 g at ~5 g/cm3 for shot in epoxy -> mm3
lo, hi = 2.0, nl_ + P["NOSE_SPIG"]
for _ in range(18):
    mid = (lo + hi) / 2; v = (cav & bx(-1, mid, -30, 30, -30, 30)).volume
    lo, hi = (mid, hi) if v < target else (lo, mid)
ball = cav & bx(-1, hi, -30, 30, -30, 30)
P["BALLAST_X_END"] = round(hi, 1)
hw("ballast", ball, "#7a6a55", "15 g nose ballast: steel shot or M6 nuts in 5-minute epoxy, poured with the nose standing on its tip (fills to x %.1f)" % hi)

json.dump(dict(version=os.environ.get("KV", "v27"), P=P, ground=GROUND, parts=parts), open(os.path.join(OUT, "parts.json"), "w"), indent=1)
print("parts:", len(parts), "L =", L, "t =", round(time.time() - T0), "s")
