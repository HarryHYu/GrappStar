"""
KEDGE probe v26 — parametric model.  Every number lives in P.  Run: python3 kedge_v26.py out/
Coordinates (dart frame, mm): X along hull axis, nose tip at x=0, tail at x=L.
Y transverse, Z up.  Dart lying flat on the ground => ground plane z = -R_OUT.
Each moving part is exported in its own LOCAL frame with its pivot at the origin;
parts.json carries the placement so the viewer and the assembly use the same numbers.
"""
import json, math, os, sys
import numpy as np
import trimesh
from trimesh.creation import box, cylinder, cone, icosphere
from trimesh.transformations import rotation_matrix as RM, translation_matrix as TM

OUT = sys.argv[1] if len(sys.argv) > 1 else "out"
os.makedirs(OUT, exist_ok=True)

# ------------------------------------------------------------------ parameters
P = dict(
    R_OUT=21.0, R_BAY=19.5, L=167.0,          # hull Ø42, wall 1.5
    NOSE_L=20.0, NOSE_R0=11.0, TAIL_L=8.0,     # blunt nose: Ø22 flat at the tip -> Ø42 at NOSE_L
    SPLIT_Z=0.0,                                # clamshell parting plane
    CAM_W=14.0, CAM_H=14.6, CAM_D=15.6, CAM_LENS=2.7, CAM_X=3.7, CAM_LENS_D=9.8,
    # kick leg: EXTERNAL. The self-locking worm N20 lies in a saddle on the hull top; its D output shaft (along +y) IS the pivot.
    # Leg hub on the shaft, outer stub in an ear. Pole runs tailward along the hull top folded; swings 0-200° over the nose.
    KICK_X=30.0, KICK_Y=12.0, SADDLE_X0=24.0, SADDLE_W=14.6, SADDLE_H=12.6, HUB_D=12.0, HUB_W=8.0, EAR_T=4.0,
    POLE_D=8.0, KICK_POLE_L=85.0, FOOT_AUGER_L=40.0, FOOT_TIP=10.0,   # foot: fixed right-angle worm N20 in line with the pole, auger across it (up folded, down deployed)
    # N20 worm motors (measured 12 x 10 body, 10 x 12 worm box, 33 long without shafts)
    N20_W=10.0, N20_H=12.0, N20_BOX=12.0, N20_L=33.0,
    # drill pair (transverse through-axle) right behind the nose; deploy N20 off-centre on the axle, body forward
    DRILL_X=56.0, DRILL_PIV_Y=29.0, DRILL_PIV_Z=5.0, DEPLOY_Y=-10.9,
    AXLE_D=3.0, AXLE_L=60.0, BOSS_D=10.0,
    DRILL_POD_L=27.0, DRILL_POD_W=15.0, BRG_WALL=9.0, AUGER_L=54.0, AUGER_D=14.0, AUGER_CORE=5.5, AUGER_PITCH=11.0, AUGER_FLIGHT_T=1.4, AUGER_TIP=12.0, SHAFT_L=56.0, SHAFT_D=3.0, BRG_OD=8.0, BRG_W=4.0,
    MOTOR_W=12.0, MOTOR_H=10.0, MOTOR_L=24.3,
    DEPLOY_DEG=38.0, KICK_DEG=200.0,
    # internals (measured)
    CELL_D=18.7, CELL_L=36.9, CELL_X0=118.0, CELL_Y=9.6,
    STACK_X0=66.0, STACK_L=46.0, FLOOR_T=1.6, DISC_T=4.0,
    SCREW_HOLE=2.5, SCREW_CLR=3.2, RAD_Z=3.0, RACK_X=(30.0, 60.0),
)
R, RB, L = P["R_OUT"], P["R_BAY"], P["L"]
GROUND = -R

# --------------------------------------------------------------- helpers
def U(parts):
    parts = [p for p in parts if p is not None]
    m = trimesh.boolean.union(parts, engine="manifold") if len(parts) > 1 else parts[0].copy()
    return m

def D(a, bs):
    bs = [b for b in bs if b is not None]
    return trimesh.boolean.difference([a] + bs, engine="manifold") if bs else a

def cyl_x(d, length, x0, y=0, z=0, n=96):          # cylinder along +X from x0
    c = cylinder(radius=d / 2, height=length, sections=n)
    c.apply_transform(RM(math.pi / 2, [0, 1, 0]))
    c.apply_transform(TM([x0 + length / 2, y, z]))
    return c

def cyl_y(d, length, y0, x=0, z=0, n=96):          # along +Y from y0
    c = cylinder(radius=d / 2, height=length, sections=n)
    c.apply_transform(RM(-math.pi / 2, [1, 0, 0]))
    c.apply_transform(TM([x, y0 + length / 2, z]))
    return c

def cyl_z(d, length, z0, x=0, y=0, n=96):
    c = cylinder(radius=d / 2, height=length, sections=n)
    c.apply_transform(TM([x, y, z0 + length / 2]))
    return c

def bx(x0, x1, y0, y1, z0, z1):
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    b = box(extents=[x1 - x0, y1 - y0, z1 - z0])
    b.apply_transform(TM([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]))
    return b

def halfspace_z(sign, z=0.0):                    # big box keeping z>=z (sign +1) or z<=z
    return bx(-50, L + 50, -50, 50, z, z + 200) if sign > 0 else bx(-50, L + 50, -50, 50, z - 200, z)

def rrect_x(w, h, length, x0, y=0, z=0, r=2.0):           # rounded-rectangle prism along +X (w across y, h across z)
    from shapely.geometry import box as sbox
    g = trimesh.creation.extrude_polygon(sbox(-h / 2 + r, -w / 2 + r, h / 2 - r, w / 2 - r).buffer(r, resolution=12), length)
    g.apply_transform(RM(math.pi / 2, [0, 1, 0]))            # extrude axis z -> x; polygon x -> -z (h), polygon y -> y (w)
    g.apply_transform(TM([x0, y, z]))
    return g

def rrect_z(w, h, length, z0, x=0, y=0, r=2.0):           # rounded-rectangle prism along +Z (w across x, h across y)
    from shapely.geometry import box as sbox
    g = trimesh.creation.extrude_polygon(sbox(-w / 2 + r, -h / 2 + r, w / 2 - r, h / 2 - r).buffer(r, resolution=12), length)
    g.apply_transform(TM([x, y, z0]))
    return g

def spur_y(teeth, module, width, y0, x=0, z=0, hole=0):      # simple trapezoid-tooth spur gear, axis along Y
    r = teeth * module / 2; ra = r + module; rf = r - 1.25 * module
    pts = []
    for i in range(teeth):
        a = 2 * math.pi * i / teeth; p = 2 * math.pi / teeth
        for f, rr in ((0.0, rf), (0.18, rf), (0.3, ra), (0.5, ra), (0.62, rf), (0.82, rf)):
            aa = a + f * p; pts.append([rr * math.cos(aa), rr * math.sin(aa)])
    from shapely.geometry import Polygon
    g = trimesh.creation.extrude_polygon(Polygon(pts), width)
    g.apply_transform(RM(math.pi / 2, [1, 0, 0]))
    g.apply_transform(TM([x, y0 + width, z]))
    if hole: g = D(g, [cyl_y(hole, width + 2, y0 - 1, x=x, z=z)])
    return g

parts = []   # (name, mesh, meta)

def add(name, mesh, group, color, pos=(0, 0, 0), axis=None, deg=(0, 0), note="", printable=True, parent=None):
    mesh.export(os.path.join(OUT, name + ".stl"))
    np.savez_compressed(os.path.join(OUT, name + ".npz"), v=mesh.vertices, f=mesh.faces)
    ext = mesh.bounding_box.extents
    parts.append(dict(name=name, file=name + ".stl", group=group, color=color,
                      pos=list(map(float, pos)), axis=axis, deg=list(deg), note=note, parent=parent,
                      printable=printable, dims=[round(float(e), 1) for e in ext],
                      volume_cm3=round(float(mesh.volume) / 1000, 2)))
    print(f"{name:22s} {ext.round(1)}  vol {mesh.volume/1000:.1f} cm3  watertight={mesh.is_watertight}")

# ------------------------------------------------------------ hull
def hull_shell():
    shell = D(cyl_x(2 * R, L - P["NOSE_L"] - P["TAIL_L"], P["NOSE_L"], n=192),
              [cyl_x(2 * RB, L, -1, n=192)])
    # kick saddle on the hull top: the worm N20 lies in it along x, worm box at the nose end, D-shaft out the +y side = the leg pivot
    sx0 = P["SADDLE_X0"]; sw, shh = P["SADDLE_W"], P["SADDLE_H"]
    zs_ = R + shh / 2 - 1.0                                                                 # saddle centre, sunk 1 mm into the skin
    saddle = rrect_x(sw, shh, P["N20_L"] + 2.6 + 2, sx0, z=zs_, r=2.5)
    ey0 = P["KICK_Y"] + P["HUB_W"] / 2 + 0.4
    ear = D(bx(P["KICK_X"] - 8, P["KICK_X"] + 8, ey0, ey0 + P["EAR_T"], 4, zs_ + 6), [cyl_x(2 * RB + 0.2, 40, P["KICK_X"] - 20, n=192)])   # outer ear: a fin rising from the skin beyond the hub, carries the hub stub
    earfoot = D(bx(max(P["KICK_X"] - 14, P["NOSE_L"] + 1), P["KICK_X"] + 14, ey0 - 2, ey0 + P["EAR_T"] + 2, 4, 15), [cyl_x(2 * RB + 0.2, 60, P["KICK_X"] - 30, n=192)])   # ear root, wider on the skin
    saddle_cut = [bx(sx0 + 1.3, sx0 + 1.3 + P["N20_L"], -P["N20_H"] / 2 - 0.2, P["N20_H"] / 2 + 0.2, zs_ - P["N20_W"] / 2 - 0.2, zs_ + 30),   # N20 pocket, open at the top: motor drops in, strap over it
                  cyl_y(P["SHAFT_D"] + 0.8, 30, 0, x=P["KICK_X"], z=zs_),                                                               # output shaft out the +y wall
                  cyl_y(P["SHAFT_D"] + 0.4, 10, P["KICK_Y"] + P["HUB_W"] / 2, x=P["KICK_X"], z=zs_),                                     # stub bore in the ear
                  bx(sx0 + 14, sx0 + 17, -20, 20, zs_ - shh / 2 - 1, zs_ - shh / 2 + 0.8)]                                               # strap groove
    P["KICK_Z"] = zs_
    # drill-pair axle bosses: from inside the wall out to just short of the leg pod, through hole for the axle
    zb = P["DRILL_PIV_Z"]; yb = P["DRILL_PIV_Y"] - (P["MOTOR_W"] + 2.6) / 2 - 0.4      # boss runs out to 0.4 short of the pod face
    yi = math.sqrt(RB ** 2 - zb ** 2) - 0.3
    bosses = [cyl_y(P["BOSS_D"], yb - yi, yi, x=P["DRILL_X"], z=zb), cyl_y(P["BOSS_D"], yb - yi, -yb, x=P["DRILL_X"], z=zb)]
    axle_hole = cyl_y(P["AXLE_D"] + 0.4, 80, -40, x=P["DRILL_X"], z=zb)
    # clamshell fixing: radial M3 clearance holes at z=+-3 (one pair per half) into the three internal rings
    sh = []
    for x in RING_XS:
        for sgn in (-1, 1):
            for zz in (-P["RAD_Z"], P["RAD_Z"]):
                sh.append(cyl_y(P["SCREW_CLR"], 6, sgn * (RB - 2), x=x, z=zz, n=16))
    body = U([shell, saddle, ear, earfoot] + bosses)
    body = D(body, [axle_hole] + sh + saddle_cut)
    return body

x0, SL = P["STACK_X0"], P["STACK_L"]
RING_XS = (x0 - P["DISC_T"] / 2, x0 + SL + P["DISC_T"] / 2)
hull = hull_shell()
top = D(hull, [halfspace_z(-1, P["SPLIT_Z"])])
bot = D(hull, [halfspace_z(+1, P["SPLIT_Z"])])
add("hull_top", top, "hull", "#9aa5b1", note="Clamshell upper half: plain skin with the kick saddle on top (worm N20 drops in, strap over it, D-shaft out the side is the leg pivot) and the outer ear; 4 radial M3 holes at z=+3 into the tray discs")
add("hull_bottom", bot, "hull", "#7d8794", note="Clamshell lower half: drill-axle bosses; 6 radial M3 holes at z=-3 into the three rings (mid ring, tray front and rear discs)")

# nose: frustum Ø2*NOSE_R0 flat at the tip -> Ø30 at x=NOSE_L; camera on the centreline looking out a hole in the flat
nl, r0 = P["NOSE_L"], P["NOSE_R0"]
hfull = nl * R / (R - r0)                                           # apex distance behind x=NOSE_L for the full cone
nose = cone(radius=R, height=hfull, sections=192)
nose.apply_transform(RM(-math.pi / 2, [0, 1, 0]))                 # apex toward -x
nose.apply_transform(TM([nl, 0, 0]))                              # base at x=NOSE_L, apex at x = NOSE_L - hfull
nose = D(nose, [bx(-80, 0, -20, 20, -20, 20)])                    # chop the tip -> flat Ø2*r0 at x=0
plug = cyl_x(2 * RB - 0.3, 8, nl - 0.01, n=192)                     # spigot into the hull bore
nose = U([nose, plug])
cw, ch, cd = P["CAM_W"], P["CAM_H"], P["CAM_D"]
cam = bx(P["CAM_X"] - 1.2, nl + 9, -cw / 2 - 0.2, cw / 2 + 0.2, -ch / 2 - 0.2, ch / 2 + 0.2)   # camera pocket, open to the bay behind
lens = cyl_x(P["CAM_LENS_D"], 12, -1)                                                       # lens hole through the flat, on axis
nose = D(nose, [cam, lens, cyl_x(2 * RB - 0.3 - 4, 7, nl + 1.5)])
add("nose", nose, "hull", "#b8c0ca", note="Blunt nose, Ø%g flat at the tip with the lens hole on the centreline; Caddx Ant pocket %g x %g behind it (camera slides in from the back, 1.2 mm lip holds it); spigot into the hull" % (2 * r0, cw, ch))

# tail cap: spigot + cap, tether eye, JST-XH charge socket window, antenna notch
tail = U([cyl_x(2 * R, P["TAIL_L"], L - P["TAIL_L"], n=192), cyl_x(2 * RB - 0.3, 4, L - P["TAIL_L"] - 4, n=192)])   # spigot x L-12..L-8
eye = D(cyl_y(10, 5, -2.5, x=L + 3, z=0), [cyl_y(5, 7, -3.5, x=L + 3, z=0)])
tail = U([tail, cyl_x(6, 4, L - 0.5, n=24), eye])
tail = D(tail, [bx(L - 20, L - 3, -5, 5, -9, -3),                # JST-XH 2.5 mm 3-pin charge socket window
                bx(L - 20, L + 1, -2, 2, 6, 12),                 # u.FL dipole slot
                cyl_x(2 * RB - 0.3 - 4, 5, L - P["TAIL_L"] - 4.5),   # hollow spigot
                cyl_x(2 * R - 3.6, P["TAIL_L"] - 2.2, L - P["TAIL_L"] - 0.5, n=192)])   # hollow cap, 1.8 wall, 1.8 end
add("tail_cap", tail, "hull", "#b8c0ca", note="Tail cap: tether eye, charge socket window, antenna slot, spigot")
# (sabot rings removed)

# ------------------------------------------------------------ electronics tray (3 decks, measured boards, headers and screw terminal removed)
G = 1.5                                                            # gap between boards along x
# board: (name, deck, length x, width y, height z, colour, note)   decks: L low, M mid, H high, H2 high stepped (over the BNO)
BOARDS = [
    ("vtx",       "L", 42.3, 19.9, 6.0, "#9b5de5", "HGLRC Zeus nano VTX 42.3 x 19.9 x 6 (diamond, 13.6 at the waist), u.FL pigtail to the tail"),
    ("ina219",    "L", 25.7, 20.3, 4.7, "#f3722c", "INA219 25.7 x 20.3 x 4.7, screw terminal removed, wires on the pads"),
    ("bms_2s",    "M", 40.8, 8.0, 2.1, "#90be6d", "OTOTEC 2S 8A BMS stick 40.8 x 8 x 2.1, flat beside the VTX"),
    ("drv8833_1", "M", 20.3, 12.7, 3.5, "#43aa8b", "DRV8833 20.3 x 12.7, headers off; side by side with #2"),
    ("drv8833_2", "M", 20.3, 12.7, 3.5, "#43aa8b", "DRV8833 20.3 x 12.7, headers off; side by side with #1"),
    ("bno085",    "M", 25.7, 22.8, 4.8, "#f8961e", "BNO085 25.7 x 22.8 x 4.8"),
    ("xiao",      "H", 21.5, 17.8, 4.5, "#2a9d8f", "XIAO ESP32S3 21.5 x 17.8 x 4.5 (22.75 with the USB-C lip), headers off"),
    ("mp1584_1",  "H", 22.3, 17.0, 4.0, "#577590", "MP1584 #1 22.3 x 17 x 4 (8 V motor rail)"),
    ("mp1584_2",  "H", 22.3, 17.0, 4.0, "#577590", "MP1584 #2 22.3 x 17 x 4 (5 V rail)"),
    ("drv8833_3", "H2", 20.3, 12.7, 3.5, "#43aa8b", "DRV8833 #3 20.3 x 12.7, headers off, on the stepped-up part of the top deck over the BNO"),
]
DECKS = (-13.0, -5.2, 2.6, 10.4)                                   # deck tops, 4 flat decks in the Ø39 bay
ROWS = [(1, ["drv8833_1", "xiao"]), (1, ["drv8833_2", "drv8833_3"]), (2, ["vtx", "bms_2s"]), (3, ["bno085"])]   # INA219 and both MP1584 live in the rack beside the deploy drive
DIM = {b[0]: b[2:5] for b in BOARDS}
LAYOUT = {}                                                        # name -> (x, y, z)
xcur = {d: x0 + 1 for d in range(4)}
for d, names in ROWS:
    ws = [DIM[n][1] for n in names]; Wt = sum(ws) + G * (len(ws) - 1); y = -Wt / 2
    for n, w in zip(names, ws):
        LAYOUT[n] = (xcur[d], y + w / 2, DECKS[d]); y += w + G
    xcur[d] += max(DIM[n][0] for n in names) + G
assert max(xcur.values()) <= x0 + SL, (xcur, x0 + SL)
# boards packed around the N20 drives: INA219 on a shelf above them (turned 90°), both MP1584 in a cradle on the hull floor below them
cx0, cx1 = P["RACK_X"]
LAYOUT["mp1584_1"] = (cx0 + 2, -1.0, -16.9)                        # floor of the rack
LAYOUT["mp1584_2"] = (cx0 + 2, 6.5, -11.1)                         # middle shelf, beside the deploy body (y -2..15)
LAYOUT["ina219"] = (cx0 + 2.2, 7.0, -5.4); DIM["ina219"] = (25.7, 20.3, 4.7)   # top shelf

def floor_plate(x1, x2, z):
    zz = z - P["FLOOR_T"] if z < 0 else z
    half = math.sqrt(max(RB ** 2 - zz ** 2, 1)) - 0.8
    return bx(x1, x2, -half, half, z - P["FLOOR_T"], z)
floors = [floor_plate(x0, x0 + SL, z) for z in DECKS[1:]]
ends = []
DT = P["DISC_T"]
for xe in (x0 - DT, x0 + SL):
    e = D(cyl_x(2 * RB - 0.4, DT, xe, n=192), [bx(xe - 1, xe + 6, -8, 8, -9, 9), bx(xe - 1, xe + 6, -8, 8, 13.5, 30), bx(xe - 1, xe + 6, -8, 8, -30, -16)]
          + [cyl_y(P["SCREW_HOLE"], 12, sgn * 7.5, x=xe + DT / 2, z=zz, n=16) for sgn in (-1, 1) for zz in (-P["RAD_Z"], P["RAD_Z"])])   # radial M3 tapping holes
    ends.append(e)
tray = U(floors + ends)
# shelf over the N20 drives (rests on the kick N20, spacer block over the lower deploy N20) and a cradle on the hull floor under them
rack = U([bx(cx0, cx1, -10.5, 10.5, -19.4, -16.9),                     # floor: MP1584 #1
          bx(cx0, cx1, -2.5, 17.5, -12.7, -11.1),                     # middle shelf: MP1584 #2
          bx(cx0, cx1, -3.0, 17.8, -7.0, -5.4),                       # top shelf: INA219
          bx(cx0, cx0 + 1.5, -3.0, 17.8, -19.0, -5.4),                # end wall, nose side
          bx(cx1 - 1.5, cx1, -3.0, 17.8, -19.0, -5.4)])               # end wall, tail side
rack = trimesh.boolean.intersection([rack, cyl_x(2 * RB - 0.2, 60, cx0 - 1, n=192)], engine="manifold")
add("rack", rack, "hull", "#c9b37e", note="Rack beside the deploy drive (trimmed to the bore): MP1584 #1 on the floor, MP1584 #2 on the middle shelf, INA219 on top, all on the +y side of the deploy N20 body. Zip-tied / glued")
add("tray", tray, "hull", "#c9b37e", note="4-deck carrier, %g long, deck tops z = %s: DRV8833 + XIAO, two DRV8833 side by side; VTX + BMS stick; BNO085 on top. Two 4 mm end discs with wire windows and radial M3 tapping holes: the hull halves screw into them; slides in from the tail" % (SL, DECKS))

# ------------------------------------------------------------ kick leg (local frame: pivot = N20 D-shaft at the origin along y, folded = pole along +X)
hub = cyl_y(P["HUB_D"], P["HUB_W"], -P["HUB_W"] / 2, n=96)
hub = D(hub, [D(cyl_y(P["SHAFT_D"] + 0.25, P["HUB_W"] + 2, -P["HUB_W"] / 2 - 1), [bx(-5, 5, -10, 10, 1.0, 5)])])   # D-bore for the N20 output shaft, right through
stub = cyl_y(P["SHAFT_D"], P["EAR_T"] + 0.6, P["HUB_W"] / 2 - 0.01, n=48)                              # printed Ø3 stub into the outer ear
yc, dz = 0.0, 0.0
pole = D(cyl_x(P["POLE_D"], P["KICK_POLE_L"], P["HUB_D"] / 2 - 1, y=yc, z=dz), [cyl_x(P["POLE_D"] - 3.0, P["KICK_POLE_L"] - 8, P["HUB_D"] / 2 + 3, y=yc, z=dz)])   # tube, 1.5 wall
xe = P["HUB_D"] / 2 - 1 + P["KICK_POLE_L"]
# foot: a right-angle worm N20 lying IN LINE with the pole; its output shaft points across the pole (local +z = up folded, DOWN deployed)
nb, nw, nh, nl20 = P["N20_BOX"], P["N20_W"], P["N20_H"], P["N20_L"]
FH_Z = 0.0
fh_w, fh_h = nh + 2.6, nw + 2.6                                                       # 14.6 (y) x 12.6 (z)
fx0 = xe - 4
fhouse = rrect_x(fh_w, fh_h, nl20 + 2.6 + 4, fx0, y=yc, z=dz + FH_Z, r=2.5)
fx_box = fx0 + 4 + 1.3 + nl20 - nb / 2
fhouse = D(fhouse, [bx(fx0 + 4, fx0 + 4 + 1.3 + nl20, yc - nh / 2 - 0.2, yc + 20, dz + FH_Z - nw / 2 - 0.2, dz + FH_Z + nw / 2 + 0.2),   # N20 pocket, open on the +y side
                    cyl_z(P["SHAFT_D"] + 0.8, 10, dz + FH_Z + nw / 2 - 1, x=fx_box, y=yc),                                          # output shaft hole in the top wall
                    bx(fx0 + 12, fx0 + 15, yc - 20, yc + 20, dz + FH_Z - fh_h / 2 - 1, dz + FH_Z - fh_h / 2 + 0.8)])                 # strap groove
elbow = U([fhouse, cyl_x(P["POLE_D"] + 2, 8, fx0 - 4, y=yc, z=dz)])
fx = fx_box
kick = U([hub, stub, pole, elbow])
add("kick_leg", kick, "legs", "#e07b39", pos=(P["KICK_X"], P["KICK_Y"], P["KICK_Z"]), axis="y", deg=(0, P["KICK_DEG"]),
    note="Kick leg print, fully outside the hull: Ø%g hub with a D-bore on the worm N20 output shaft (the pivot), printed stub into the outer ear, Ø%g tube pole %g long, foot housing on the end. Folded it lies on the hull top; the worm swings it 0-200° over the nose until the foot auger meets the ground, then the auger pulls it the rest of the way" % (P["HUB_D"], P["POLE_D"], P["KICK_POLE_L"]))

# ------------------------------------------------------------ drill leg (local: axle at origin along Y, pole along +X when folded)
def helical_flight(r_core, depth, pitch, length, thick=2.0, steps_per_turn=96, embed=0.8):
    """Watertight single-start screw flight along +X from x=0.  r_core(x) and depth(x) are callables so the
    flight follows a stepped / conical core: inner edge sits embed mm inside the core, outer edge depth(x) proud."""
    n = max(4, int(length / pitch * steps_per_turn))
    V, F = [], []
    for i in range(n + 1):
        x = length * i / n
        t = 2 * math.pi * x / pitch
        rc = r_core(x); ri = max(rc - embed, 0.15); ro = rc + depth(x)
        c, s = math.cos(t), math.sin(t)
        for rr, dx in ((ri, -thick / 2), (ro, -thick / 2), (ro, thick / 2), (ri, thick / 2)):
            V.append([x + dx, rr * c, rr * s])
    for i in range(n):
        a = 4 * i
        for k in range(4):
            p0, p1 = a + k, a + (k + 1) % 4
            q0, q1 = p0 + 4, p1 + 4
            F.append([p0, q0, p1]); F.append([p1, q0, q1])
    F.append([0, 1, 2]); F.append([0, 2, 3])
    e = 4 * n
    F.append([e, e + 2, e + 1]); F.append([e, e + 3, e + 2])
    m = trimesh.Trimesh(vertices=np.array(V), faces=np.array(F), process=True)
    m.fix_normals()
    return m

def screw_auger(flight_d, length, pitch=5.0, core_d=5.0, tip_len=8.0, bore_d=3.3, nyloc_x=None, hub_d=8.6):
    """Printed soil auger along +X from x=0.  Ø core_d core, cone tip, ONE continuous helical flight
    (depth (flight_d-core_d)/2) that follows the cone down to a Ø2 point.  Short Ø hub_d hub around the
    M3 nyloc pocket at the shaft end; bore from the base into the pocket; solid beyond.  One manifold solid."""
    body_l = length - tip_len
    rc0 = core_d / 2
    nut = nyloc_x is not None
    if nyloc_x is None:
        nyloc_x = 2.0
    core = cyl_x(core_d, body_l, 0, n=48)
    hub = cyl_x(hub_d, 5.0, nyloc_x - 0.7, n=96) if nut else cyl_x(hub_d, 9.0, 0, n=96)
    tip = cone(radius=rc0, height=tip_len, sections=96)
    tip.apply_transform(RM(math.pi / 2, [0, 1, 0])); tip.apply_transform(TM([body_l, 0, 0]))
    def r_core(x):
        if x <= body_l: return rc0
        return rc0 * (1 - (x - body_l) / tip_len)
    dep0 = (flight_d - core_d) / 2
    def depth(x):
        if x <= body_l: return dep0
        return dep0 * (1 - 0.75 * (x - body_l) / tip_len)
    x_end = body_l + tip_len * (1 - 1.0 / rc0)               # stop where the cone is Ø2
    fl = helical_flight(r_core, depth, pitch, x_end, thick=P["AUGER_FLIGHT_T"])
    a = U([core, hub, tip, fl])
    if nut:
        cuts = [cyl_x(bore_d, nyloc_x + 1.5, -1), cyl_x(6.4, 3.6, nyloc_x, n=6)]
    else:
        cuts = [D(cyl_x(bore_d, 10, -1), [bx(-2, 10, -5, 5, bore_d / 2 - 0.5, 5)])]        # 9 mm D-bore for the N20 output shaft
    return D(a, cuts)

def drill_assembly(prefix, parent_pos, parent_axis, color):
    """Local frame: pivot at origin, leg along +X when folded.  The WHOLE leg beyond the pod is the auger:
    motor in the pod, its M3 x 56 shaft runs out through a bearing wall (two 693ZZ), the full-length printed
    auger threads on and is clamped by a nyloc inside its tip.  Adds parts; returns nothing."""
    PL, PW, BW = P["DRILL_POD_L"], P["DRILL_POD_W"], P["BRG_WALL"]
    MW, MH, ML = P["MOTOR_W"], P["MOTOR_H"], P["MOTOR_L"]
    xs = PL                                  # motor face = shaft start
    xa = PL + BW                             # auger starts right outside the bearing wall
    hw, hh = MW + 2.6, MH + 2.6                                                     # housing 14.6 x 12.6, 1.3 walls
    head = cyl_y(hh, hw, -hw / 2, n=96)                                             # round clevis head on the axle
    housing = rrect_x(hw, hh, PL + 1, -1, r=2.5)                                    # rounded motor housing
    nose = cyl_x(hh - 1, BW, PL, n=96)                                              # bearing nose, 11.6 round
    taper = cone(radius=(hh - 1) / 2, height=14, sections=96); taper.apply_transform(RM(math.pi / 2, [0, 1, 0])); taper.apply_transform(TM([xa - 14 + 8, 0, 0]))
    nose = trimesh.boolean.intersection([nose, U([cyl_x(hh - 1, 6, PL, n=96), taper])], engine="manifold")   # tapers to ~Ø8 at the auger
    pod = U([head, housing, nose])
    pod = D(pod, [cyl_y(P["AXLE_D"] + 0.25, 30, -15),                                   # axle bore
                  bx(-2, 2, -15, 15, 1.0, 5),                                           # D-flat key (matches the axle flat)
                  bx(PL - ML - 0.5, PL, -MW / 2 - 0.2, MW / 2 + 0.2, -MH / 2 - 0.2, 20),  # motor pocket 12.4 x 10.4 x 24.8, open at the top: motor drops in, zip tie over it
                  rrect_x(hw + 2, 2.0, 3.0, PL - 12, r=0.9).apply_transform(RM(0, [1, 0, 0])) if False else bx(PL - 13, PL - 10, -20, 20, -hh / 2 - 1, -hh / 2 + 0.8),   # zip-tie groove under the housing
                  cyl_x(P["SHAFT_D"] + 0.6, BW + 2, xs - 1),                            # shaft clearance through the bearing nose
                  cyl_x(P["BRG_OD"] + 0.15, P["BRG_W"], xs + 0.2),                      # 693ZZ seat, inner
                  cyl_x(P["BRG_OD"] + 0.15, P["BRG_W"], xa - P["BRG_W"] - 0.2)])        # 693ZZ seat, outer
    add(prefix, pod, "legs", color, pos=parent_pos, axis=parent_axis, deg=(0, P["DEPLOY_DEG"]),
        note="Drill leg pod print: round clevis head on the axle (D-keyed), rounded 14.6 x 12.6 housing with a 1.3 mm wall, motor drops in from the top (zip-tie groove), tapered bearing nose with two 693ZZ the shaft runs through" )
    # full-length printed auger on the shaft
    auger = screw_auger(P["AUGER_D"], P["AUGER_L"], pitch=P["AUGER_PITCH"], core_d=P["AUGER_CORE"],
                        nyloc_x=P["SHAFT_L"] - BW - 3.6, hub_d=9.0, tip_len=P["AUGER_TIP"])
    auger.apply_transform(TM([xa, 0, 0]))
    add(prefix + "_auger", auger, "legs", "#b23a48", parent=prefix,
        note="Full-length printed soil auger Ø%g x %g: Ø%g core, 4 mm deep 2 mm thick single-start %g mm-pitch flight the whole way, following the cone to the point. M3 bore from the base; the shaft end lands in a hex pocket inside a Ø9.6 hub and an M3 nyloc there clamps the auger against the outer bearing. The auger IS the leg; the pair counter-rotates (one motor wired reversed)" % (P["AUGER_D"], P["AUGER_L"], P["AUGER_CORE"], P["AUGER_PITCH"]))
    # envelopes riding with the leg
    add(prefix + "_motor", bx(PL - ML, PL, -MW / 2, MW / 2, -MH / 2, MH / 2), "electronics", "#6c757d", parent=prefix, printable=False,
        note="Drill motor (measured 12 x 10 x 24.3) in the pod; its M3 x 56 shaft drives the auger")
    add(prefix + "_shaft", cyl_x(P["SHAFT_D"], P["SHAFT_L"], xs, n=16), "electronics", "#d0d4d8", parent=prefix, printable=False,
        note="M3 threaded motor shaft, 56 long: motor face -> two bearings in the pod wall -> up the auger core -> nyloc in the tip")
    for i, xb in enumerate((xs + 0.2, xa - P["BRG_W"] - 0.2)):
        add(prefix + "_brg%d" % (i + 1), D(cyl_x(P["BRG_OD"], P["BRG_W"], xb, n=96), [cyl_x(3.0, 6, xb - 1)]),
            "electronics", "#9aa5b1", parent=prefix, printable=False, note="693ZZ bearing 3 x 8 x 4 pressed into the pod's bearing wall")
    add(prefix + "_nyloc", D(cyl_x(6.2, 3, xs + P["SHAFT_L"] - 3.0, n=6), [cyl_x(3, 5, xs + P["SHAFT_L"] - 4)]),
        "electronics", "#777", parent=prefix, printable=False, note="M3 nyloc inside the auger tip hub, clamps the auger on the shaft")

for sgn, nm in ((1, "drill_leg_L"), (-1, "drill_leg_R")):
    drill_assembly(nm, (P["DRILL_X"], sgn * P["DRILL_PIV_Y"], P["DRILL_PIV_Z"]), "y", "#d9534f" if sgn > 0 else "#c0392b")

# through-axle (printed D-shaft, biax worm N20 drives it)
axle = cyl_y(P["AXLE_D"], P["AXLE_L"], -P["AXLE_L"] / 2, n=24)
axle = D(axle, [bx(-5, 5, -P["AXLE_L"], P["AXLE_L"], 1.0, 5)])    # D flat
add("axle", axle, "legs", "#444", pos=(P["DRILL_X"], 0, P["DRILL_PIV_Z"]), axis="y", deg=(0, P["DEPLOY_DEG"]),
    note="Ø3 D-shaft through-axle, 56 long; legs keyed on the ends; F693ZZ flanged bearings in the hull bosses")

# ------------------------------------------------------------ electronics envelopes (NOT for printing)
E = []
def env(name, mesh, color, note):
    add(name, mesh, "electronics", color, note=note, printable=False)

cx = P["CELL_X0"]
for i, sy in enumerate((-1, 1)):
    env("cell_%d" % (i + 1), U([cyl_x(P["CELL_D"], P["CELL_L"] - 0.7, cx, y=sy * P["CELL_Y"], n=96), cyl_x(6, 0.7, cx + P["CELL_L"] - 0.7, y=sy * P["CELL_Y"])]),
        "#f4d35e", "18350 cell Ø18.7 x 36.9 incl. the button (36.15 body), measured; the two sit side by side")

# kick N20 (self-locking worm) in the saddle on the hull top; deploy N20 (biax worm) on the through-axle
nw, nh, nb, nl20 = P["N20_W"], P["N20_H"], P["N20_BOX"], P["N20_L"]
sx0 = P["SADDLE_X0"]; zs_ = P["KICK_Z"]
env("n20_kick", U([bx(sx0 + 1.3, sx0 + 1.3 + nb, -nh / 2, nh / 2, zs_ - nw / 2, zs_ + nw / 2),
                   bx(sx0 + 1.3 + nb, sx0 + 1.3 + nl20, -nh / 2, nh / 2, zs_ - nw / 2, zs_ + nw / 2)]),
    "#8e9aaf", "KICK DRIVE: self-locking worm N20 (12 x 10 x 33) lying in the saddle on the hull top, worm box at the nose end; its D output shaft points +y and carries the leg hub directly, 1:1, no gears. Self-locking: holds any angle unpowered")
env("kick_shaft", D(cyl_y(P["SHAFT_D"], 9, nh / 2, x=P["KICK_X"], z=zs_), [bx(0, 60, -20, 20, zs_ + 1.0, zs_ + 5)]), "#d0d4d8", "N20 D output shaft, 9 proud of the box = kick pivot")
dxx, dzz, dy = P["DRILL_X"], P["DRILL_PIV_Z"], P["DEPLOY_Y"]
env("n20_deploy", U([bx(dxx - nb / 2, dxx + nb / 2, dy - nw / 2, dy + nw / 2, dzz - nh / 2, dzz + nh / 2),
                     bx(dxx - nb / 2 - (nl20 - nb), dxx - nb / 2, dy - nw / 2, dy + nw / 2, dzz - nh / 2, dzz + nh / 2)]),
    "#8e9aaf", "DEPLOY DRIVE: biax worm N20 (12 x 10, 33 long) on the through-axle at x=%g, off-centre at y=%g, body FORWARD to x=%g beside the kick N20" % (dxx, dy, dxx - nb / 2 - (nl20 - nb)))

for name, deck, lx, ly, lz, color, note in BOARDS:
    x, y, z = LAYOUT[name]
    lx, ly, lz = DIM[name]
    env(name, bx(x, x + lx, y - ly / 2, y + ly / 2, z, z + lz), color, note)
camx = P["CAM_X"]
env("camera", U([bx(camx, camx + cd, -cw / 2, cw / 2, -ch / 2, ch / 2), cyl_x(8, P["CAM_LENS"], camx - P["CAM_LENS"])]), "#1d3557",
    "Caddx Ant (measured) 14 x 14.6 body, 15.6 deep incl. the rear connector, Ø8 lens 2.7 proud; on the centreline looking out of the nose flat")
zt = dz + FH_Z + nw / 2 + 1.3                                                    # housing top face
add("n20_foot", U([bx(fx_box - nb / 2, fx_box + nb / 2, yc - nh / 2, yc + nh / 2, dz + FH_Z - nw / 2, dz + FH_Z + nw / 2),
                   bx(fx0 + 5.3, fx_box - nb / 2, yc - nh / 2, yc + nh / 2, dz + FH_Z - nw / 2, dz + FH_Z + nw / 2)]),
    "electronics", "#8e9aaf", parent="kick_leg", printable=False,
    note="FOOT DRIVE: a third right-angle worm N20 (same 12 x 10 x 33 as the kick one, ONE MORE TO BUY) lying in line with the pole; output D-shaft points across the pole: up when folded, DOWN into the ground when deployed")
add("foot_shaft", cyl_z(P["SHAFT_D"], 10, zt - 1.3, x=fx_box, y=yc, n=16), "electronics", "#d0d4d8", parent="kick_leg", printable=False,
    note="N20 Ø3 D output shaft, ~9 proud of the box; the auger's D-bore sits on it, M3 grub screw in the hub")
fa = screw_auger(P["AUGER_D"], P["FOOT_AUGER_L"], pitch=P["AUGER_PITCH"], core_d=P["AUGER_CORE"], nyloc_x=None, hub_d=9.0, tip_len=P["FOOT_TIP"], bore_d=3.1)
fa = D(fa, [cyl_y(2.6, 10, 0, x=5.0, z=0, n=16)])                                  # M3 grub screw hole into the D-bore (radial, at 5 mm up the hub)
fa.apply_transform(RM(-math.pi / 2, [0, 1, 0]))            # +X -> +Z
fa.apply_transform(TM([fx_box, yc, zt + 1.0]))              # base 1 mm above the housing top
add("foot_auger", fa, "legs", "#b23a48", parent="kick_leg",
    note="Foot auger Ø%g x %g (same profile as the leg augers), D-bore on the worm N20 output shaft, grub screw in the hub" % (P["AUGER_D"], P["FOOT_AUGER_L"]))
json.dump(dict(P=P, ground=GROUND, parts=parts), open(os.path.join(OUT, "parts.json"), "w"), indent=1)
print("parts:", len(parts))
