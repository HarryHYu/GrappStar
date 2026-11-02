"""GrappStar ground station generator: launcher + reel on one wiper motor.
python3 gs_gen.py OUTDIR
Frames: world X forward, Y left, Z up, base plate top Z=0. Rail frame: x=u along the rail, z=w, origin on the main
shaft axis, pitched up by theta about Y. PG frame: origin on the program gear axis. Sled frame: x=u-P (P = pusher face).
Reel frame: origin on the reel axis at the rotor face plane, the axis is world Y."""
import json, math, os, sys
import numpy as np
import trimesh
from shapely.geometry import Polygon, box as sbox, Point
from shapely.ops import unary_union
from shapely import affinity
from geom import *

OUT = sys.argv[1] if len(sys.argv) > 1 else "out_gs"
os.makedirs(OUT, exist_ok=True)

# ------------------------------------------------------------------ parameters
P = dict(
    H=80.0,              # main shaft height above the base plate top
    DART_W=95.0,         # dart axis above the main shaft (rail frame)
    TH_LOW=4.0, TH_FIRE=35.0,          # 35 deg: same range as 40 (drag + tether), less lift work and a gentler cam
    SHAFT_D=12.0, BRG_OD=28.0, BRG_W=8.0,            # 6001-2RS
    OWC_OD=18.0, OWC_W=16.0,                          # HF1216
    M=3.0, Z_PIN=14, Z_PG=30, PG_U=66.0, GEAR_W=14.0, PG_TEETH=18, BETA0=12.0,
    RACK_W=45.0,         # rack pitch line height
    P_D=38.0,            # pusher face at full draw
    CR_RI=22.0, CR_RO=25.5,                           # cradle half pipe
    SHOE_L=40.0,
    MR=2.0, Z_MAIN=58, Z_STUB=39,                    # reel drive gears
    REEL_FACE_Y=-14.0,   # rotor face plane (world y)
    SP_CORE=36.0, SP_FL=58.0, SP_W=26.0,
    ROT_OD=66.0, PIN_OUT=3.0,
    SPRING_Y=30.5, SPRING_W=30.0, SPRING_OD=12.6, SPRING_L0=238.0, SPRING_PRE=15.0,
    FRAME_Y=96.0, PLATE_T=8.0, BASE_T=8.0,
    CAM_Y0=66.0, CAM_Y1=74.0, ROLL_R=11.0,
    U_REAR=-150.0,
)
H = P["H"]; DW = P["DART_W"]
r_pg = P["M"] * P["Z_PG"] / 2; r_pin = P["M"] * P["Z_PIN"] / 2
assert abs(r_pg + r_pin - P["PG_U"]) < 1e-6
PITCH = math.pi * P["M"]


def pg_poly_local():
    g = involute_gear_2d(P["M"], P["Z_PG"], present=list(range(P["PG_TEETH"])))
    g = affinity.rotate(g, 90 - P["BETA0"], origin=(0, 0))
    g = affinity.scale(g, xfact=-1, yfact=1, origin=(0, 0))
    return affinity.rotate(g, 2 * (90 - P["BETA0"]) - 180, origin=(0, 0))


def rack_poly_local(n):
    """rack at rest relative to the PG centre: tooth k=-1 at u=-p/2, tooth k=0 removed"""
    r = rack_2d(P["M"], n, body=58 - P["RACK_W"])
    r = affinity.translate(r, -0.5 * PITCH, P["RACK_W"])
    return r.difference(sbox(-0.5 * PITCH + PITCH * 0.5 + 0.2, P["RACK_W"] - 3.2, -0.5 * PITCH + PITCH * 1.5 - 0.2, P["RACK_W"] + 3.6))


def rack_travel_table():
    """push simulation: the springs pull the rack forward onto the teeth. Returns [(phi, d)], release phi, peak d"""
    g0 = pg_poly_local(); rk_ = rack_poly_local(P["PG_TEETH"] + 4)
    d, tab = 0.0, []
    for phi in np.arange(-40, 330, 0.5):
        g = affinity.rotate(g0, phi, origin=(0, 0))
        while affinity.translate(rk_, -d, 0).intersection(g).area > 1e-4:
            d += 0.02
        free = all(affinity.translate(rk_, -(d - k * 0.25), 0).intersection(g).area < 1e-4 for k in range(1, 13))
        tab.append((float(phi), float(d)))
        if phi > 200 and free:
            return tab, float(phi), float(d)
    raise RuntimeError("no release")


RT, PH_REL, D_PEAK = rack_travel_table()
P["STROKE"] = D_PEAK; P["PH_REL"] = PH_REL; P["RACK_TAB"] = RT
print("release at phi %.1f, peak travel %.2f" % (PH_REL, D_PEAK))
P["P_R"] = P["P_D"] + P["STROKE"]
r_main = P["MR"] * P["Z_MAIN"] / 2; r_stub = P["MR"] * P["Z_STUB"] / 2
P["REEL_U"] = -math.sqrt((r_main + r_stub) ** 2 - DW ** 2)
P["U_MUZ"] = P["P_R"] + 152.0 + 12.0
P["PULLEY_U"] = P["U_MUZ"] - 32.0
P["ANCHOR_U"] = P["PULLEY_U"] - (P["SPRING_L0"] + P["SPRING_PRE"] + P["STROKE"]) - 46.0   # spring end stays 30 mm clear of the forks
STROKE, P_D, P_R = P["STROKE"], P["P_D"], P["P_R"]
RU = P["REEL_U"]
print("stroke %.1f  P_R %.1f  reel_u %.1f  muzzle %.1f  anchor %.1f" % (STROKE, P_R, RU, P["U_MUZ"], P["ANCHOR_U"]))

parts = {}


def add(name, solid, frame, group, color, printed=True, note="", qty=1):
    parts[name] = dict(solid=solid, frame=frame, group=group, color=color, printed=printed, note=note, qty=qty)


def dflat(sol, y0, y1, r_bore, x=0.0, z=0.0):
    """add the flat of a D bore back into a round bore: material above z = r_bore - 1 (the shaft has a 1 mm flat there)"""
    return sol + (cylY(r_bore + 0.05, y0, y1, x, z) ^ box(x - r_bore - 1, x + r_bore + 1, y0, y1, z + r_bore - 1.0, z + r_bore + 1))


def grub(sol, y, r_hub, r_bore, nut=True):
    """radial M3 grub screw (into a ground flat) at axial position y, pointing +z, with an M3 nut slot"""
    sol = sol - cyl(1.6 if nut else 1.25, r_hub + 1 - r_bore + 0.5, 0, y, r_bore - 0.5)
    if nut and r_hub - r_bore > 6:
        sol = sol - box(-2.95, 2.95, y - 1.35, y + 1.35, r_bore + 1.6, r_hub + 1)
    return sol


def cylZ_spring(x, y, z0, z1):
    return cyl(2.0, z1 - z0, x, y, z0, seg=24)


def microswitch(name, frame, x0, y0, z0, axes, lever, note):
    """KW11-3Z micro switch envelope 20 x 6.4 x 10.3 (+ lever strip). axes: which world/rail axes the 20 / 6.4 / 10.3 run along."""
    L = {"x": 0, "y": 1, "z": 2}
    d = [0, 0, 0]; d[L[axes[0]]] = 20.0; d[L[axes[1]]] = 6.4; d[L[axes[2]]] = 10.3
    body = box(x0, x0 + d[0], y0, y0 + d[1], z0, z0 + d[2])
    add(name, body + lever, frame, "electronics", "#202428", printed=False, note=note)


def bearing(x, y0, z, od, w, idd=12.0, axis="y"):
    """Ball bearing envelope ring along Y."""
    return cylY(od / 2, y0, y0 + w, x, z) - cylY(idd / 2, y0 - 1, y0 + w + 1, x, z)


# ================================================================= RAIL (rail frame)
def cradle_section():
    ann = Point(0, DW).buffer(P["CR_RO"], 128).difference(Point(0, DW).buffer(P["CR_RI"], 128))
    return ann.intersection(sbox(-60, 0, 60, 81.0))


def side_section(wall_w0=30.0):
    """Shelves, lips and side walls, both sides, (y, w)."""
    g = []
    for s in (1, -1):
        def B(y0, y1, w0, w1):
            return sbox(min(s * y0, s * y1), w0, max(s * y0, s * y1), w1)
        g += [B(18.0, 42.5, 77, 81), B(42, 48, wall_w0, 92), B(38, 42.5, 89, 92)]
    return unary_union(g)


def channel_section():
    g = []
    for s in (1, -1):
        g.append(sbox(min(s * 8.5, s * 12.5), 38, max(s * 8.5, s * 12.5), 74.0))
    g.append(sbox(-12.5, 34, 12.5, 38))       # floor under the rack (cut away where the PG comes through)
    return unary_union(g)


def ext_u(sec, u0, u1):
    """extrude a (y, w) section along u from u0 to u1 -> rail frame solid (x=u, y, z=w)."""
    s = extrude_z(sec, u0, u1)        # (X=y, Y=w, Z=u)
    # map X->y, Y->z, Z->x
    return s.rotate((0, 0, 0)).transform(np.array([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0]], dtype=float))


U_CR0 = 25.0                      # cradle starts forward of the reel
U_M = P["U_MUZ"]
rail_parts = []
cr = ext_u(cradle_section(), U_CR0, U_M)
# stem slot in the cradle bottom
cr = cr - box(P_D - 18, P_R + 16, -6.6, 6.6, DW - 30, DW - 15)     # runs 16 mm past rest: room for the pads to compress
sides = ext_u(side_section(80.5), U_CR0 + 10, 160.5) + ext_u(side_section(30.0), 160.0, U_M)
chan = ext_u(channel_section(), U_CR0, U_M - 20)
def channel_low():
    g = [sbox(min(s * 8.5, s * 12.5), 38, max(s * 8.5, s * 12.5), 58.5) for s in (1, -1)]
    g.append(sbox(-12.5, 34, 12.5, 38))
    return unary_union(g)
chan = chan + ext_u(channel_low(), P["U_REAR"], U_CR0 + 0.5)
# opening in the channel floor for the program gear
chan = chan - box(P["PG_U"] - 52, P["PG_U"] + 50, -13, 13, 33, 39)
for uu in (120.0, -39.0):
    for yy in ((-2.5, 2.5) if uu > 0 else (-5.0, 5.0)):
        chan = chan - cyl(1.65, 6, uu, yy, 33) - cyl(1.65, 1.8, uu, yy, 36.3, r2=3.3)
# rack grooves in the channel walls (ribs on the rack ride here)
for s in (1, -1):
    chan = chan - box(P["U_REAR"] - 1, U_M, min(s * 7.5, s * 10), max(s * 7.5, s * 10), 52, 56)

# rear cheeks: carry the main shaft bearings (pivot) and the PG stub bearings
def cheek(s):
    y0, y1 = (38.0, 46.0) if s > 0 else (-46.0, -38.0)
    prof = unary_union([Point(0, 0).buffer(26, 64), Point(P["PG_U"], 0).buffer(26, 64),
                        sbox(-20, -24, P["PG_U"], 24), sbox(-45, 14, 160, 40), sbox(20, 30, 160, 76.5)])
    c = extrude_xz(prof, y0, y1)
    c = c - cylY(P["BRG_OD"] / 2, y0 - 1, y1 + 1, 0, 0) - cylY(P["BRG_OD"] / 2, y0 - 1, y1 + 1, P["PG_U"], 0)
    return c

cheekL, cheekR = cheek(-1), cheek(1)
REEL_PAD_HOLES = [(-30.0, 30.0), (5.0, 30.0), (-12.0, 46.0)]       # (u, w): M4 along y, bracket to the cheek pad
pad = box(-40, 15, -59.0, -45.5, 22, 52)
for (uu, ww) in REEL_PAD_HOLES:
    pad = pad - cylY(2.8, -60, -51, uu, ww) - cylY(2.2, -52, -45, uu, ww)   # M4 heat-set insert (5.6 x 8) from the bracket face
cheekL = cheekL + pad
# backstop housing boss on the +y cheek around the PG stub: HF1216 next to the bearing
SLEEVE_R = 12.0           # HF1216 pressed into a machined 18 x 24 steel sleeve (3 mm wall), sleeve bonded with Loctite 648
boss = cylY(17, 46, 46 + P["OWC_W"], P["PG_U"], 0) - cylY(SLEEVE_R, 45, 47 + P["OWC_W"], P["PG_U"], 0)
cheekR = cheekR + boss
bossL = cylY(17, -46 - P["OWC_W"], -46, P["PG_U"], 0) - cylY(SLEEVE_R, -47 - P["OWC_W"], -45, P["PG_U"], 0)
cheekL = cheekL + bossL
# cross ties between cheeks (front and rear of the PG) and the spring anchor bar
# ties pass UNDER the springs (spring centre w 30, OD 12); centre posts carry the rail channel
ties = union([box(116, 138, -41.9, 41.9, 6, 22.5), box(116, 126, -12.5, 12.5, 20, 34.0)])
# spring anchor: its own part (bed size), between the cheeks behind the pivot, 2 x M4 through each cheek into its ends
anchor_bar = box(P["ANCHOR_U"] - 8, -26.0, -37.7, 37.7, 14, 20)      # stays under the springs (spring bottom w 23.7)
for s_ in (1, -1):                                                   # thick ends for the cheek bolts
    anchor_bar = anchor_bar + box(-44.0, -26.0, min(s_ * 26.0, s_ * 37.9), max(s_ * 26.0, s_ * 37.7), 14.0, 23.4)
for s_ in (1, -1):                                                   # hook blocks around the spring axis, 3 mm steel pins
    anchor_bar = anchor_bar + box(P["ANCHOR_U"] - 8, P["ANCHOR_U"] + 20, min(s_ * 23.0, s_ * 37.7), max(s_ * 23.0, s_ * 37.7), 14, 37)
    anchor_bar = anchor_bar - box(P["ANCHOR_U"] - 5, P["ANCHOR_U"] + 17, min(s_ * 27.0, s_ * 34.0), max(s_ * 27.0, s_ * 34.0), 20, 38)   # hook slot
anchor_bar = anchor_bar + box(-45, -33, -12.5, 12.5, 14.0, 34.0)    # rear post: rail piece A screws onto it
rear_block = union([cheekL, cheekR, ties])
for uu in (-40.0, -30.0):                         # 2 x M4 through each cheek into the bar ends (tapped 3.4 / insert)
    rear_block = rear_block - cylY(2.2, -47, 47, uu, 17.5)
    for s_ in (1, -1):                                                 # M4 x 20 from outside each cheek into a nut in an open trap
        anchor_bar = anchor_bar - cylY(2.2, min(s_ * 24.0, s_ * 40.0), max(s_ * 24.0, s_ * 40.0), uu, 17.5)
        anchor_bar = anchor_bar - box(uu - 3.6, uu + 3.6, min(s_ * 29.6, s_ * 33.0), max(s_ * 29.6, s_ * 33.0), 13.5, 25.0)   # nut trap, open at the top
# 3 x M4 per side: down through the wall strip and shelf into the cheek top (heat-set inserts in the cheek)
CHEEK_BOLTS = [(uu, s_, 86.0) for uu in (85.0, 132.0, 150.0) for s_ in (1, -1)]   # (u, side, w): M4 along y
for (uu, s_, ww) in CHEEK_BOLTS:
    tab = box(uu - 7, uu + 7, min(s_ * 45.5, s_ * 52.0), max(s_ * 45.5, s_ * 52.0), 60.0, 92.0) - box(uu - 8, uu + 8, min(s_ * 45.5, s_ * 48.2), max(s_ * 45.5, s_ * 48.2), 80.2, 93)
    rear_block = rear_block + tab - cylY(2.2, min(s_ * 47, s_ * 53), max(s_ * 47, s_ * 53), uu, ww)
# the rail pieces sit on the ties: 2 x M3 countersunk down through the channel floor into each tie
for yy in (-2.5, 2.5):
    rear_block = rear_block - cyl(1.25, 14, 120.0, yy, 22)
for yy in (-5.0, 5.0):
    anchor_bar = anchor_bar - cyl(1.25, 14, -39.0, yy, 22)
# spring anchor hooks: holes in the anchor bar
for du in (0.0, 12.0):                             # 2 hook pins (3 mm steel) across each slot: preload adjust
    anchor_bar = anchor_bar - cylY(1.55, -39, 39, P["ANCHOR_U"] + du, P["SPRING_W"])

# muzzle: pulley forks and a flared mouth
pul_u, pul_r = P["PULLEY_U"], (85.0 - P["SPRING_W"]) / 2
pul_w = (85.0 + P["SPRING_W"]) / 2
muz = []
for s in (1, -1):
    muz.append(box(pul_u - 16, pul_u + 16, min(s * 21.0, s * 26.0), max(s * 21.0, s * 26.0), pul_w - pul_r - 8, 70.0))
    muz.append(box(pul_u - 16, pul_u + 16, min(s * 35.5, s * 42), max(s * 35.5, s * 42), pul_w - pul_r - 8, 81))
    muz.append(box(pul_u - 10, pul_u + 10, min(s * 35.5, s * 42), max(s * 35.5, s * 42), 81, 90.5))   # outer fork above the pulley: keeps the cord in
    muz.append(box(pul_u - 16, pul_u + 16, min(s * 21.0, s * 42), max(s * 21.0, s * 42), pul_w - pul_r - 12, pul_w - pul_r - 8))
muzzle = union(muz)
for s in (1, -1):
    muzzle = muzzle - cylY(1.7, 17 if s > 0 else -49, 49 if s > 0 else -17, pul_u, pul_w)      # M3 x 35 from outside the wall, nyloc inside

sides = sides - box(pul_u - pul_r - 4, pul_u + pul_r + 4, -36.0, -25.8, 76, 82) - box(pul_u - pul_r - 4, pul_u + pul_r + 4, 25.8, 36.0, 76, 82)
rail_beam = union([cr, sides, chan, muzzle])
for s in (1, -1):                                  # pulley axle holes right through the side wall (M3 x 35 from outside)
    rail_beam = rail_beam - cylY(1.7, 17 if s > 0 else -49, 49 if s > 0 else -17, pul_u, pul_w)
add("rail_beam", rail_beam, "rail", "launcher", "#c9d1d9", note="cradle + shelves + rack channel, printed in 3 bolted sections")
add("rail_rear", rear_block, "rail", "launcher", "#9aa4ae", note="cheeks: pivot bearings, PG bearings, backstop bosses, ties, reel pad")
add("spring_anchor", anchor_bar, "rail", "launcher", "#9aa4ae", note="spring anchor bar (PPA-CF, printed flat) behind the pivot, 2 x M4 x 20 through each cheek into nuts in open traps")

# ================================================================= MAIN SHAFT parts (world, rotate with motor)
Y_SH0, Y_SH1 = -112.0, 128.0
add("main_shaft", cylY(6, Y_SH0, Y_SH1), "main", "hardware", "#8a8f94", printed=False, note="12 mm hardened linear rod, 240 mm")
pin_poly = involute_gear_2d(P["M"], P["Z_PIN"])
pinion = extrude_xz(pin_poly, -P["GEAR_W"] / 2, P["GEAR_W"] / 2)
pinion = pinion + cylY(14.5, -16, 16)
pinion = pinion + Manifold.cylinder(2.75, 17.25, 14.5, 96).rotate((-90, 0, 0)).translate((0, P["GEAR_W"] / 2, 0)) \
    + Manifold.cylinder(2.75, 17.25, 14.5, 96).rotate((90, 0, 0)).translate((0, -P["GEAR_W"] / 2, 0))   # 45 deg skirts: teeth print without support
pinion = pinion - cylY(SLEEVE_R, -17, 17)   # hub carries 2 x HF1216 (A) in a steel sleeve
add("drive_pinion", pinion, "main_pin", "launcher", "#e0a030", note="m3 z14, HF1216 (A) pressed in")
add("owc_A", bearing(0, -16, 0, 24.0, 32.0), "main_pin", "hardware", "#555555", printed=False, note="2 x HF1216 one way (drive) in a machined 18 x 24 x 32 steel sleeve")
mg_poly = involute_gear_2d(P["MR"], P["Z_MAIN"])
y_g0, y_g1 = -85.0, -77.0
main_gear = extrude_xz(mg_poly, y_g0, y_g1) + cylY(14, y_g1, y_g1 + 8)
main_gear = main_gear - cylY(6.1, y_g0 - 1, y_g1 + 9)             # round bore: it sits mid shaft, an M3 grub bites a spot flat
# clamp: radial M3 screw hole into the hub
main_gear = grub(main_gear, y_g1 + 4, 14, 6)
main_gear = grub(main_gear.rotate((0, 90, 0)), y_g1 + 4, 14, 6).rotate((0, -90, 0))      # second grub at 90 deg: two spot flats
add("reel_main_gear", main_gear, "main", "reel", "#e0a030", note="m2 z58, round bore, 2 x M3 grub + nut at 90 deg onto spot flats on the main shaft")

# ================================================================= PROGRAM GEAR (pg frame)
pres = list(range(P["PG_TEETH"]))
pg_poly = involute_gear_2d(P["M"], P["Z_PG"], present=pres)
# orient: tooth k at local angle -(k*12 + BETA0) measured CCW from +w (top). gear poly has tooth 0 on +x.
# local frame of pg in (x=u, z=w); tooth 0 must point to angle 90 - BETA0 deg from +x (CCW from +x)
pg_poly = affinity.rotate(pg_poly, 90 - P["BETA0"], origin=(0, 0))
# shapely polygon is (x, y) -> we want (u, w): our extrude_xz takes (x, z) so this is right, but the gear rotates
# CCW in (u, w) with phi; teeth further CW (negative angle) come later -> mirror the tooth order
pg_poly = affinity.scale(pg_poly, xfact=-1, yfact=1, origin=(0, 0))
pg_poly = affinity.rotate(pg_poly, 2 * (90 - P["BETA0"]) - 180, origin=(0, 0))
pg = extrude_xz(pg_poly, -P["GEAR_W"] / 2, P["GEAR_W"] / 2)
pg = pg + cylY(17, -P["GEAR_W"] / 2, P["GEAR_W"] / 2) - cylY(6.1, -30, 30)
HUB_PCD_R = 12.0                                   # 12 mm rigid flange coupling: 4 holes on a 24 mm circle
for k_ in range(4):
    a_ = math.radians(45 + 90 * k_)
    pg = pg - cylY(2.8, -P["GEAR_W"] / 2 - 1, -P["GEAR_W"] / 2 + 8, HUB_PCD_R * math.cos(a_), HUB_PCD_R * math.sin(a_))   # M4 heat-set inserts
add("program_gear", pg, "pg", "launcher", "#f0b040", note="m3 z30, 18 teeth present, bolted to a steel flange hub (4 x M4 into inserts); the hub grips the stub with 2 x M5 set screws on flats at 90 deg")
add("pg_stub", cylY(6, -64, P["CAM_Y1"] + 16), "pg", "hardware", "#8a8f94", printed=False, note="12 mm hardened rod, 154 mm")
FL_HUB = "12 mm rigid flange coupling, steel (flange 32 x 5, hub 22, 4 holes on a 24 circle, set screws)"
add("pg_hub", cylY(16, -P["GEAR_W"] / 2 - 5, -P["GEAR_W"] / 2) + cylY(11, -P["GEAR_W"] / 2 - 18, -P["GEAR_W"] / 2 - 5) - cylY(6.05, -40, 0),
    "pg", "hardware", "#a0a4a8", printed=False, note=FL_HUB + ": carries the whole cam and trip torque into the stub")
add("cam_hub", cylY(16, P["CAM_Y1"], P["CAM_Y1"] + 5) + cylY(11, P["CAM_Y1"] + 5, P["CAM_Y1"] + 16) - cylY(6.05, 60, 95),
    "pg", "hardware", "#a0a4a8", printed=False, note=FL_HUB + " (hub sawn down to 11 mm): the cam bolts to it")
add("owc_C", bearing(0, 46, 0, 24.0, P["OWC_W"]) + bearing(0, -62, 0, 24.0, P["OWC_W"]), "pg", "hardware", "#555555", printed=False,
    note="2 x HF1216 backstop, one in each cheek boss, each in a machined 18 x 24 steel sleeve")

# ================================================================= kinematics helpers
def rail_to_world(u, w, th):
    t = math.radians(th)
    return (u * math.cos(t) - w * math.sin(t), H + u * math.sin(t) + w * math.cos(t))


def world_to_rail(X, Z, th):
    t = math.radians(th); z = Z - H
    return (X * math.cos(t) + z * math.sin(t), -X * math.sin(t) + z * math.cos(t))


BLEND = 0.25
def smooth(t):
    """constant velocity with sine-squared blends at both ends: peak slope only 1.33 x the mean. The rail is a planet
    carrier (the program gear rides on it round the drive pinion), so the motor torque grows as 1 / (1 - (14/30) dtheta/dphi):
    a flat lift slope keeps that factor low."""
    t = min(1, max(0, t)); b = BLEND; vm = 1 / (1 - b)
    def up(x): return vm * (x / 2 - b / (2 * math.pi) * math.sin(math.pi * x / b))
    if t < b: return up(t)
    if t > 1 - b: return 1 - up(1 - t)
    return vm * (b / 2 + (t - b))


PH_LIFT0 = PH_REL + 6.0
PH_LIFT1 = PH_LIFT0 + 50.0
PH_TRIP, PH_DROP0 = PH_LIFT1 + 1.0, PH_LIFT1 + 2.0
PH_DROP1 = PH_DROP0 + 28.0
P.update(PH_LIFT0=PH_LIFT0, PH_LIFT1=PH_LIFT1, PH_DROP0=PH_DROP0, PH_DROP1=PH_DROP1)


def theta_prog(phi):
    phi %= 360
    if phi < PH_DROP1 - 360:
        phi += 360                      # the drop runs past 360 into the next turn
    lo, hi = P["TH_LOW"], P["TH_FIRE"]
    if phi < PH_LIFT0 or phi >= PH_DROP1:
        return lo
    if phi < PH_LIFT1:
        return lo + (hi - lo) * smooth((phi - PH_LIFT0) / (PH_LIFT1 - PH_LIFT0))
    if phi < PH_DROP0:
        return hi
    return hi - (hi - lo) * smooth((phi - PH_DROP0) / (PH_DROP1 - PH_DROP0))


# ================================================================= LIFT CAM (pg frame), envelope of the frame roller
def roller_world():
    return P["ROLL_X"], P["ROLL_Z"]


def cam_profile(rx, rz, rr, n=720):
    """Cam (in pg frame, (u, w) about the pg axis, rotating CCW with phi) that keeps the roller tangent."""
    big = Point(0, 0).buffer(90, 256)
    cut = []
    for i in range(n):
        phi = 360.0 * i / n
        th = theta_prog(phi)
        # bumper holds the rail at TH_LOW: the cam stays 0.6 mm clear there
        u, w = world_to_rail(rx, rz, th)
        du, dw = u - P["PG_U"], w - 0.0
        # into the gear frame (rotate by -phi)
        a = -math.radians(phi)
        cu, cw = du * math.cos(a) - dw * math.sin(a), du * math.sin(a) + dw * math.cos(a)
        extra = 0.6 if th <= P["TH_LOW"] + 1e-6 else 0.0
        cut.append(Point(cu, cw).buffer(rr + extra, 32))
    cam = big.difference(unary_union(cut))
    # keep the piece around the hub
    if cam.geom_type == "MultiPolygon":
        cam = [g for g in cam.geoms if g.contains(Point(0, 0))][0]
    return cam


best = None
for rx in np.arange(30, 181, 5):
    for rz in np.arange(H - 110, H + 1, 5):
        rmin, rmax, ok = 1e9, 0, True
        for phi in np.arange(0, 360, 4):
            u, w = world_to_rail(rx, rz, theta_prog(phi))
            d = math.hypot(u - P["PG_U"], w)
            rmin = min(rmin, d); rmax = max(rmax, d)
        # roller must sit below the PG (gravity closes the cam) and outside the gear and hub
        if rmin - P["ROLL_R"] < 23 or rz > H - 5 or rmax - P["ROLL_R"] > 55 or rz - P["ROLL_R"] < 12:
            continue
        score = rmax
        if best is None or score < best[0]:
            best = (score, rx, rz, rmin, rmax)
P["ROLL_X"], P["ROLL_Z"] = float(best[1]), float(best[2])
print("roller at X %.0f Z %.0f, cam radius %.1f..%.1f" % (best[1], best[2], best[3] - P["ROLL_R"], best[4] - P["ROLL_R"]))
cam_poly = cam_profile(P["ROLL_X"], P["ROLL_Z"], P["ROLL_R"])
cam_poly = cam_poly.buffer(-0.3).buffer(0.3)
cam = extrude_xz(cam_poly, P["CAM_Y0"], P["CAM_Y1"]) + cylY(14, P["CAM_Y0"] - 3.5, P["CAM_Y1"]) - cylY(6.1, 0, 80)
for k_ in range(4):
    a_ = math.radians(45 + 90 * k_)
    cam = cam - cylY(2.8, P["CAM_Y1"] - 8, P["CAM_Y1"] + 1, HUB_PCD_R * math.cos(a_), HUB_PCD_R * math.sin(a_))   # M4 heat-set inserts for the flange hub
add("lift_cam", cam, "pg", "launcher", "#d06030", note="lift cam (PPA-CF), envelope of the roller, bolted to a steel flange hub on the stub (4 x M4)")
P["cam_rmax"] = float(max(math.hypot(x, y) for x, y in cam_poly.exterior.coords))

# ================================================================= SLED + RACK (sled frame: x = u - P)
KEEL = (P["SHOE_L"] - 18.0, P["SHOE_L"], -34.5, -29.5, 64.0)   # sled-frame x0, x1, y0, y1, bottom w (x1 = pad face)
STEM_Y, STEM_X0 = 6.0, -16.0


def cylZ_m3(x, y):
    return cyl(1.6, 30, x, y, 40)


def sled_solid():
    # pusher: disc segment behind the dart tail, centre hole for the line
    pus = cylX(21.5, -8, 0, 0, DW) ^ box(-9, 1, -30, 30, DW - 22, DW + 12)
    pus = pus - cylX(7, -9, 1, 0, DW)                                   # 14 mm: the bridle knot sits inside the tail cap, never between
    pus = pus - Manifold.cylinder(1.01, 8.0, 7.0, 48).rotate((0, 90, 0)).translate((-8.005, 0, DW)) \
              - Manifold.cylinder(1.01, 7.0, 8.0, 48).rotate((0, 90, 0)).translate((-1.005, 0, DW))      # 1 mm chamfers: no square edge on the line
    arms = box(-16, 0, -40.5, 40.5, 81.5, 88.5)
    shoes = [box(-16, P["SHOE_L"], min(s * 26.5, s * 40.5), max(s * 26.5, s * 40.5), 81.5, 88.5) for s in (1, -1)]
    # keel on the -y shoe, through the shelf slot: its front face is the sear catch. Tall lead-in at the rear bottom:
    # while cocking it meets the sloped front of the hook and turns it down.
    x0, x1, y0, y1, wb = KEEL
    keel = box(x0, x1, y0, y1, wb, 82)
    ch = Manifold.extrude(CrossSection([[(x0 - 1, wb - 1), (x0 + 9, wb - 1), (x0 - 1, wb + 13)]]), 12).rotate((90, 0, 0)).translate((0, y1 + 3, 0))
    keel = keel - ch
    chf = Manifold.extrude(CrossSection([[(x1 - 0.8, wb - 1), (x1 + 1, wb - 1), (x1 + 1, wb + 0.8)]]), 12).rotate((90, 0, 0)).translate((0, y1 + 3, 0))
    keel = keel - chf                                                   # 0.8 mm chamfer, bottom front edge: the hook gets up sooner
    for (xa, xb) in ((x1, x1 + 3.0), (x0, x0 - 3.0)):                  # 3 mm root fillets front and back (along u only)
        keel = keel + Manifold.extrude(CrossSection([[(xa, 78.6), (xb, 81.6), (xa, 81.6)] if xb > xa else [(xa, 78.6), (xa, 81.6), (xb, 81.6)]]), y1 - y0).rotate((90, 0, 0)).translate((0, y1, 0))
    stem = box(STEM_X0, 0, -STEM_Y, STEM_Y, 56, DW - 18) + box(STEM_X0, 0, -STEM_Y, STEM_Y, DW - 22, DW - 16)
    stem = stem + box(STEM_X0, -9, -STEM_Y, STEM_Y, DW - 17, 82.0)                 # bridge stem to arms behind the pusher
    sl = union([pus, arms, stem, keel] + shoes)
    sl = sl - cyl(1.7, 40, -12.5, 0, 55.0) - cyl(3.0, 10, -12.5, 0, 85.5) - cyl(1.55, 7, -4, 0, 55.0)  # M3 x 35 from the top (head in a pocket), 3 x 12 dowel
    sl = sl - cylX(P["CR_RO"] + 0.6, 0.01, 60, 0, DW)
    for s in (1, -1):
        sl = sl - cylX(1.3, -17, P["SHOE_L"] + 1, s * P["SPRING_Y"], 85)           # cord through the whole shoe
        sl = sl - box(-17, -11, s * P["SPRING_Y"] - 3, s * P["SPRING_Y"] + 3, 83.0, 88.6)   # knot pocket at the back
    return sl


add("sled", sled_solid(), "sled", "launcher", "#4a90d9", note="carriage (PPA-CF): pusher, shoes, 12 x 16 stem, 18 mm keel, printed on its side so the keel bends along its layers (sear catch); cord through each shoe, knot in the back pocket")

# rack: teeth centres at rest u = PG_U + (k+0.5)*pitch, k=-1..PG_TEETH ; built in sled frame (x = u - P_R at rest)
n_rack = P["PG_TEETH"] + 2
rk0 = P["PG_U"] - 0.5 * PITCH - P_R             # first tooth centre in sled frame (k = -1)
rk = rack_2d(P["M"], n_rack, body=56 - P["RACK_W"])
# rack_2d: x along, y: teeth down to -m, body up -> map (x, y) -> (u, w - RACK_W)
rk = affinity.translate(rk, rk0, P["RACK_W"])
rack = extrude_xz(rk, -6, 6)
# tooth k=0 removed: the first PG tooth swings into a standing rack and would hit it
rack = rack - box(rk0 + PITCH * 0.5 + 0.2, rk0 + PITCH * 1.5 - 0.2, -7, 7, P["RACK_W"] - 3.2, P["RACK_W"] + 3.6)
for s in (1, -1):
    rack = rack + box(rk0 - PITCH / 2, rk0 + (n_rack - 0.5) * PITCH, min(s * 6, s * 9.4), max(s * 6, s * 9.4), 52.4, 55.6)
rack = rack + box(STEM_X0, 0, -6, 6, 52, 56) - cyl(1.25, 7, -12.5, 0, 49.5) - cyl(1.55, 6.5, -4, 0, 50.0)   # tapped M3 + dowel hole
add("rack", rack, "sled", "launcher", "#3a78c0", note="m3 rack (PPA-CF), %d teeth: an M3 x 35 from the top of the carriage and a 3 x 12 dowel" % n_rack)

def sear_at_(d, sol=None):
    pu, pw = SEAR_PIV
    return (sol or parts["sear"]["solid"]).translate((-pu, 0, -pw)).rotate((0, -d, 0)).translate((pu, 0, pw))


# ================================================================= SEAR (rail frame, -y side, inside the left cheek)
# Clevis: inner plate (4 mm) and outer plate (5 mm, in a pocket of the cheek) either side of the keel path, joined by
# the hook. M4 shoulder bolt pivot in double shear: left cheek + lug under the shelf. The catch face is an arc centred
# 3 mm below the pivot: the keel load passes under the pivot and pulls the hook up against its stop (self locking), and
# the face slides on itself when the hook turns, so it does not push into the keel. A small spring on the tail lifts the
# hook onto an up-stop; the outer arm presses SW_COCKED there. The trip is a cord from the outer arm to an adjustable
# anchor on the base: as the rail lifts the cord goes taut and pulls the hook down at the top of the lift.
OVERTRAVEL = 1.8                  # free run from full draw onto the hook: small, so the catch is gentle
P["OVERTRAVEL"] = OVERTRAVEL
KEEL_FRONT_FD = P_D + P["SHOE_L"]            # keel catch face u at peak draw
SEAR_PIV = (KEEL_FRONT_FD + OVERTRAVEL - 20.0, 70.5)
ARC_C = (SEAR_PIV[0], SEAR_PIV[1] - 3.0)      # catch-face arc centre
R_C = 20.0
HOOK = (SEAR_PIV[0] + R_C, SEAR_PIV[0] + R_C + 12.0, 60.0, 69.5)   # u0 (catch at w 67.5), u1, w0, w1
PL_IN, PL_OUT = (-28.6, -24.6), (-40.2, -35.2)  # plate y ranges; keel runs in y -34.5..-29.5
CORD_A = (SEAR_PIV[0] + 50.0, SEAR_PIV[1] - 12.0)   # 3 mm steel tie pin in the outer plate (rail frame, at delta 0)
CORD_Y = -50.0                                 # the pin sticks out through an arc slot in the cheek; the cord runs in this plane
DOWN_STOP = -24.5                              # hard down-stop for the sear
STOP_U = (SEAR_PIV[0] + 26.0, SEAR_PIV[0] + 34.0)
SWC_U = SEAR_PIV[0] + 42.0
P.update(ARC_C=list(ARC_C), R_C=R_C)


def hook_profile():
    pu, pw = SEAR_PIV; u0, u1, w0, w1 = HOOK
    arc = [(ARC_C[0] + R_C * math.cos(a), ARC_C[1] + R_C * math.sin(a)) for a in np.radians(np.linspace(math.degrees(math.asin((w0 - ARC_C[1]) / R_C)), math.degrees(math.asin((w1 - ARC_C[1]) / R_C)), 12))]
    hp = Polygon(arc + [(u1 - 7.0, w1), (u1, w0)])          # arc catch face, flat top, sloped lead-in front
    cu = arc[-1][0]                                          # 1 mm 45 deg chamfer on the rear top corner: rises past the keel sooner
    return hp.difference(Polygon([(cu - 3, w1 - 4), (cu + 2, w1 + 1), (cu - 3, w1 + 1)]))


def sear_solid():
    pu, pw = SEAR_PIV
    hp = hook_profile()
    arm_bot = pw - 7.5
    prof = unary_union([Point(pu, pw).buffer(5.5, 32), sbox(pu, arm_bot, HOOK[1] - 4, pw - 1.0), hp,
                        sbox(pu - 11, pw - 6, pu, pw - 0.5)])
    outer_prof = unary_union([prof, sbox(HOOK[1] - 6, CORD_A[1] - 5, CORD_A[0] + 2, pw - 1.0), Point(CORD_A[0], CORD_A[1]).buffer(5.0, 32)])
    inner = extrude_xz(prof, PL_IN[0], PL_IN[1])
    outer = extrude_xz(outer_prof, PL_OUT[0], PL_OUT[1])
    hook = extrude_xz(hp.buffer(0.01), PL_OUT[1] - 0.3, PL_IN[0] + 0.3)
    bridge = box(pu - 11, pu - 5, PL_OUT[1] - 0.3, PL_IN[0] + 0.3, pw - 6, pw - 0.5)    # tail bridge
    sv = union([inner, outer, hook, bridge])
    sv = sv - cylY(2.6, -42, -20, pu, pw)                                               # 5 mm shoulder of the bolt (running fit)
    sv = sv + cylY(3.5, PL_OUT[1] - 0.1, PL_OUT[1] + 2.2, CORD_A[0], CORD_A[1])          # boss: 7.2 mm of press length for the pin
    sv = sv - cylY(1.45, -42, PL_OUT[1] + 2.0, CORD_A[0], CORD_A[1])                     # 3 mm grooved tie pin, pressed
    sv = sv - box(pu - 9.5, pu - 6.5, -32.9, -30.3, pw - 6.5, pw - 1.5)                # slot in the tail bridge for the spring hook
    sv = sv - cylY(1.05, -42, -20, pu - 8, pw - 3.3)                                    # 2 mm steel pin: spring hook
    return sv


add("sear", sear_solid(), "sear", "launcher", "#c03030", note="clevis sear (PPA-CF): arc catch face, spring lifts it onto its stop, a cord trips it")
LUG_Y = (-21.0, -16.0)              # the 25 mm shoulder runs from the cheek outer face (y -46) to the lug
add("sear_pin", None, "rail", "hardware", "#8a8f94", printed=False,
    note="shoulder screw 5 x 25, M4 x 8 thread, head 9 x 4 (ISO 7379 style, e.g. GANTER ISO 7379-5-25-M4) into a captive M4 nut in the lug")
parts["sear_pin"]["solid"] = (cylY(2.5, -46.0, LUG_Y[0], SEAR_PIV[0], SEAR_PIV[1]) + cylY(2.0, LUG_Y[0], LUG_Y[0] + 8.0, SEAR_PIV[0], SEAR_PIV[1])
                              + cylY(4.5, -50.0, -46.0, SEAR_PIV[0], SEAR_PIV[1]) + box(SEAR_PIV[0] - 3.5, SEAR_PIV[0] + 3.5, -19.3, -16.1, SEAR_PIV[1] - 4.0, SEAR_PIV[1] + 4.0))
add("sear_spring", cyl(3.0, SEAR_PIV[1] - 6.0 - 48.0, SEAR_PIV[0] - 8, -31.6, 48.0, seg=24), "rail", "hardware", "#c0c4c8", printed=False,
    note="extension spring OD 6, wire 0.7, 16 mm hook to hook, about 3 N initial tension, about 1 N/mm: 8 N fitted (65 N mm on the sear), on 2 mm steel pins: tail down = hook up")
add("cord_pin", cylY(1.5, CORD_Y - 3.0, PL_OUT[1] + 2.0, CORD_A[0], CORD_A[1]) + cylY(3.0, CORD_Y - 2.4, CORD_Y - 1.8, CORD_A[0], CORD_A[1]), "sear", "hardware", "#8a8f94", printed=False,
    note="3 x 20 grooved pin (or 3 mm rod with a groove) pressed into the outer plate boss; out through the cheek arc slot, an e-clip keeps the trip cord on")
lug = box(SEAR_PIV[0] - 8, SEAR_PIV[0] + 8, LUG_Y[0], LUG_Y[1], SEAR_PIV[1] - 7, 77.2) - cylY(2.15, -25, -15, SEAR_PIV[0], SEAR_PIV[1])
lug = lug - box(SEAR_PIV[0] - 3.6, SEAR_PIV[0] + 3.6, -19.4, -15.9, SEAR_PIV[1] - 4.2, SEAR_PIV[1] + 4.2)   # captive M4 nut, pressed in from the +y face
rb = parts["rail_beam"]["solid"] + lug
rb = rb - box(P_D + KEEL[0] - 2, P_R + P["SHOE_L"] + 16, -35.1, -28.9, 76, 82)                       # keel slot in the shelf
parts["rail_beam"]["solid"] = rb
rr = parts["rail_rear"]["solid"] - cylY(2.55, -47, -37, SEAR_PIV[0], SEAR_PIV[1])
rr = rr - box(SEAR_PIV[0] - 16, CORD_A[0] + 9, -41.2, -37.9, 28.0, 76.6)                           # pocket for the outer plate
parts["rail_beam"]["solid"] = parts["rail_beam"]["solid"] + box(STOP_U[0], STOP_U[1], -40.6, -35.4, SEAR_PIV[1] - 1.0, 77.5)   # up-stop hangs from the shelf (arm top meets it)
dstop = box(SEAR_PIV[0] + 20, SEAR_PIV[0] + 45, -41.3, -37.9, 30.0, 52.0)                          # down-stop: the outer plate lands on it at DOWN_STOP
dstop = dstop - union([sear_at_(dd) for dd in np.arange(0.0, DOWN_STOP - 0.01, -0.5)])
rr = rr + dstop
spost = box(SEAR_PIV[0] - 14, SEAR_PIV[0] - 2, -41.1, -36.0, 37.5, 52.0) + box(SEAR_PIV[0] - 12, SEAR_PIV[0] - 4, -36.1, -24.7, 44.0, 48.0)
spost = spost - cylY(1.05, -42, -24, SEAR_PIV[0] - 8, 46.0)                                          # 2 mm pin: spring lower hook
spost = spost - box(SEAR_PIV[0] - 9.5, SEAR_PIV[0] - 6.5, -32.9, -30.3, 43.0, 49.0)                    # slot for the hook loop
for du_ in (-11.0, -5.0):
    spost = spost - cylY(1.25, -41, -35, SEAR_PIV[0] + du_, 40.6)                                   # tapped M3 in the post
    rr = rr - cylY(1.7, -47, -40, SEAR_PIV[0] + du_, 40.6)                                         # 2 x M3 x 10 from outside the cheek
add("sear_spring_post", spost, "rail", "launcher", "#c03030", note="sear spring post (PPA-CF): 2 x M3 x 10 from outside the left cheek, 2 mm pin for the spring")
arc = []
for dd in np.arange(2.0, DOWN_STOP - 2.01, -1.0):                                                   # arc slot: the tie pin swings in it
    a_ = math.radians(dd); du_, dw_ = CORD_A[0] - SEAR_PIV[0], CORD_A[1] - SEAR_PIV[1]
    arc.append(cylY(2.4, -47, -40, SEAR_PIV[0] + du_ * math.cos(a_) - dw_ * math.sin(a_), SEAR_PIV[1] + du_ * math.sin(a_) + dw_ * math.cos(a_)))
rr = rr - union(arc)
rr = rr - box(SWC_U, SWC_U + 8, -47, -40, SEAR_PIV[1] - 4.5, SEAR_PIV[1] + 1.5)                    # window: SW_COCKED lever reaches the arm
for du_ in (5.25, 14.75):                                             # KW11 holes: M2 self tapping into the cheek
    rr = rr - cylY(0.9, -46.5, -41.5, SWC_U - 6 + du_, SEAR_PIV[1] - 1.0 + 0.2 + 4.0)
parts["rail_rear"]["solid"] = rr
# SW_COCKED: outside the cheek, its lever through the window rests on the outer arm top: pressed (closed, COM + NO) while
# the hook sits on its stop; the arm moves away as soon as the hook goes down, so nothing can jam
arm_top = SEAR_PIV[1] - 1.0
microswitch("sw_cocked", "rail", SWC_U - 6, -52.6, arm_top + 0.2, ("x", "y", "z"),
            box(SWC_U + 2, SWC_U + 6, -46.2, -40.4, arm_top + 0.15, arm_top + 1.2),
            "SW_COCKED: KW11 outside the left cheek, COM + NO, lever on the sear arm: closed while the hook is up")

# ================================================================= REEL (reel frame: origin on the axis at the face plane,
# axis = world Y, local coords (x=u-REEL_U, y=y-REEL_FACE_Y, z=w-DART_W))
RF = P["REEL_FACE_Y"]
sp_core, sp_fl, sp_w = P["SP_CORE"] / 2, P["SP_FL"] / 2, P["SP_W"]
y_fl_front, y_fl_rear = -16.0, -16.0 - sp_w
def spool_solid():
    # core tapers 4 deg (smaller at the back): the turns the rotor lays at the front slide back under tension
    core = Manifold.cylinder(y_fl_front - y_fl_rear, sp_core, sp_core + (y_fl_front - y_fl_rear) * math.tan(math.radians(8)), 96).rotate((-90, 0, 0)).translate((0, y_fl_rear, 0))
    s = core + cylY(30.0, y_fl_front - 3, y_fl_front) + cylY(sp_fl + 2, y_fl_rear - 3, y_fl_rear)     # front flange 0.5 mm inside the skirt
    s = s + (cylY(12.0, y_fl_front, -13.7) - cylY(4.15, -20, 0))                                       # shoulder: the drag disc cannot back off
    # front boss for the drag disc + O-ring
    s = s + cylY(10, y_fl_front, -6.5)
    s = s - cylY(4.15, -60, 0)                      # shaft runs through (printed bushing fit)
    s = s - cylY(11.0, y_fl_rear - 4, y_fl_rear + 6)  # rear recess (light)
    s = s + (cylY(11.0, y_fl_rear - 3, y_fl_rear + 6) - cylY(6.5, y_fl_rear - 4, y_fl_rear + 7))
    # O-ring groove on the front boss (2 mm cord)
    s = s - (cylY(11, -13.2, -11.2) - cylY(9.0, -13.7, -10.7))
    # 3 M3 holes in the rear flange into the bracket
    for k in range(3):
        a = math.radians(90 + 120 * k)
        s = s - cylY(1.25, y_fl_rear - 4, y_fl_rear + 9, 14.5 * math.cos(a), 14.5 * math.sin(a))  # tapped M3 inside the core: M3 x 14 from the bracket
    return s


def rotor_solid():
    R = P["ROT_OD"] / 2
    face = cylY(R, -3, 0)
    skirt = cylY(R, -16 - 4, 0) - cylY(R - 2.5, -21, -3)
    hub = cylY(8, -5.5, 0)
    r = union([face, skirt, hub]) - cylY(4.05, -10, 1)
    # rounded face edge: chamfer approximation
    r = r - (cylY(R + 2, -1.25, 0.02) - Manifold.cylinder(1.27, R, R - 1.27, 96).rotate((-90, 0, 0)).translate((0, -1.25, 0)))   # 1.25 chamfer, face edge (line runs over it)
    r = r - (cylY(R + 2, -20.1, -18.75) - Manifold.cylinder(1.36, R - 1.35, R, 96).rotate((-90, 0, 0)).translate((0, -20.1, 0)))   # skirt rear lip, outside
    r = r - (Manifold.cylinder(0.9, R - 1.6, R - 2.5, 96).rotate((-90, 0, 0)).translate((0, -20.05, 0)) - cylY(R - 2.5, -21, -18))     # and inside
    r = r - Manifold.cylinder(0.8, 2.6, 3.4, 48).rotate((0, 90, 0)).translate((R - 0.79, -8, 0))                                    # pin hole mouth chamfer
    # pickup pin guide: radial hole in the skirt at y=-10, angle 0 (+x)
    r = r - cylX(2.6, R - 4, R + 1, -8, 0)
    # grub screw hole in the hub
    r = dflat(r, -5.5, 0, 4.05)
    r = grub(r, -3.0, 8, 4, nut=False)
    return r


def drag_disc_solid():
    d = cylY(30.0, -13.5, -10.8) - cylY(10.6, -16, -10)       # 0.4 mm O-ring squeeze: defined drag
    # spiral slot: pin peg moves r 23 -> 26.5 over 25 deg
    pts = []
    r_in = P["ROT_OD"] / 2 - 4.8 - 3.5            # slot runs 1.2 mm inside the rim at the out dwell
    for t in np.linspace(0, 1, 80):          # 60 deg dwell in, 25 deg spiral, 30 deg dwell out: a rail drop with the motor stopped
        ang = -115 * t                         # turns the rotor 53.5 deg backward relative to the rail: the pin stays in
        r = r_in if ang > -60 else (r_in + 3.5 if ang < -85 else r_in + 3.5 * (-60 - ang) / 25)
        a = math.radians(ang + 60)
        pts.append((r * math.cos(a), r * math.sin(a)))
    from shapely.geometry import LineString
    slot = LineString(pts).buffer(1.3, 16)
    d = d - extrude_xz(slot, -14, -10)
    return d


def pin_solid():
    R = P["ROT_OD"] / 2
    p = cylX(2.45, R - 5.7, R + P["PIN_OUT"], -8, 0)
    peg = cylY(1.15, -13.2, -8, R - 4.8, 0)
    return p + peg


def cover_solid():
    R = P["ROT_OD"] / 2
    c = cylY(R + 7, -24, 4) - cylY(R + 4.5, -25, 1.5)
    # exit eye: dia 4.4 coned to 6.4 both faces
    c = c - cylY(3.05, -1, 6)                                # seat for a ceramic rod guide ring (OD 6, smooth bore, round both ends)
    # window at the bottom (line can be threaded, debris falls out)
    c = c - box(-R - 10, R + 10, -30, 6, -R - 10, -36.5)
    # two legs back to the bracket
    for a in (60, 200):
        ar = math.radians(a)
        x, z = (R + 5.5) * math.cos(ar), (R + 5.5) * math.sin(ar)
        c = c + cylY(4, y_fl_rear - 3.0, -22, x, z)
        c = c - cylY(1.25, y_fl_rear - 3.5, y_fl_rear + 9, x, z)
    return c


def bracket_solid():
    # plate behind the spool rear flange; bolts to the -y cheek
    R = P["ROT_OD"] / 2
    yb0, yb1 = y_fl_rear - 9, y_fl_rear - 3
    prof = unary_union([Point(0, 0).buffer(40, 96), sbox(-40, -DW + 20, 40, 0)])
    b = extrude_xz(prof, yb0, yb1)
    b = b - cylY(4.15, yb0 - 1, yb1 + 1)
    for k in range(3):
        a = math.radians(90 + 120 * k)
        b = b - cylY(1.6, yb0 - 1, yb1 + 1, 14.5 * math.cos(a), 14.5 * math.sin(a))
    for a in (60, 200):
        ar = math.radians(a)
        b = b + cylY(4.6, yb0, yb1, (R + 5.5) * math.cos(ar), (R + 5.5) * math.sin(ar))           # boss: the leg hole stays inside
        b = b - cylY(1.6, yb0 - 1, yb1 + 1, (R + 5.5) * math.cos(ar), (R + 5.5) * math.sin(ar))
    for (uu, ww) in REEL_PAD_HOLES:
        b = b - cylY(2.2, yb0 - 1, yb1 + 1, uu - P["REEL_U"], ww - DW)
    # bearing boss for the stub shaft outboard
    b = b + (cylY(9, yb0 - 8, yb0) - cylY(4.15, yb0 - 9, yb0 + 1))
    return b


reel_parts = dict(spool=spool_solid(), rotor=rotor_solid(), drag_disc=drag_disc_solid(), pickup_pin=pin_solid(),
                  reel_cover=cover_solid(), reel_bracket=bracket_solid())
add("spool", reel_parts["spool"], "reel", "reel", "#7b68ee", note="stationary spool, 20 m of 1 mm Dyneema")
add("rotor", reel_parts["rotor"], "rotor", "reel", "#9b88ff", note="spincast rotor (PPA-CF, solid): M3 grub on the D flat + a 2 mm steel cross pin through hub and stub")
add("drag_disc", reel_parts["drag_disc"], "drag", "reel", "#5a5ad0", note="friction disc on the O-ring, spiral slot moves the pin")
add("pickup_pin", reel_parts["pickup_pin"], "pin", "reel", "#ff4060", note="pickup slider (PPA-CF, solid): out on F, in on B; its line post is a smooth 3 mm steel dowel, its peg a 2 mm steel dowel")
add("reel_cover", reel_parts["reel_cover"], "reel", "reel", "#d0d0e8", note="front cone, ceramic eye ring on the axis")
add("eye_ring", cylY(3.0, 0.8, 4.8) - cylY(1.9, 0, 6), "reel", "hardware", "#f0f0f0", printed=False,
    note="ceramic (SiC or alumina) fishing rod guide insert, OD 6, ID about 3.8, glued in the cover: the line never touches print")
add("reel_bracket", reel_parts["reel_bracket"], "reel", "reel", "#9aa4ae", note="holds spool, cover, stub shaft; bolts to the -y cheek")
yb0 = y_fl_rear - 9
add("reel_stub", cylY(4, yb0 - 22, -1), "rotor", "hardware", "#8a8f94", printed=False, note="8 mm steel rod 72 mm, D flats at both ends")
sg = extrude_xz(involute_gear_2d(P["MR"], P["Z_STUB"]), yb0 - 20, yb0 - 12) + cylY(9, yb0 - 12, yb0 - 9)
sg = dflat(sg, yb0 - 20, yb0 - 9, 4.05)
sg = grub(sg, yb0 - 10.5, 9, 4, nut=False)
sg = sg - cylY(4.05, yb0 - 21, yb0 - 8)
add("reel_stub_gear", sg, "rotor", "reel", "#e0a030", note="m2 z39 on the 8 mm stub (M3 grub)")
add("oring", cylY(11, -13.2, -11.2) - cylY(9, -13.7, -10.7), "reel", "hardware", "#222222", printed=False, note="O-ring 18 x 2 nitrile (ID 18, cord 2)")
P["REEL_GEAR_Y"] = RF + yb0 - 16
# check gear planes line up: stub gear centre plane (world) vs main gear plane
print("stub gear plane y %.1f  main gear plane y %.1f" % (RF + yb0 - 16, (y_g0 + y_g1) / 2))

# ================================================================= FRAME (world)
FY = P["FRAME_Y"]; T = P["PLATE_T"]
def side_plate(s):
    y0, y1 = (FY, FY + T) if s > 0 else (-FY - T, -FY)
    prof = unary_union([Point(0, H).buffer(24, 64), sbox(-70, 0, 70, H), sbox(-24, H - 10, 24, H)])
    pl = extrude_xz(prof, y0, y1) - cylY(P["BRG_OD"] / 2, y0 - 1, y1 + 1, 0, H)
    # lightening window
    pl = pl - extrude_xz(Polygon([(-50, 12), (50, 12), (16, H - 26), (-16, H - 26)]), y0 - 1, y1 + 1)
    # foot holes
    for x in (-58, 58):
        pass
    return pl


def cylZ_hole(x, y):
    return cyl(1.7, 40, x, y, -1)


DECK_HOLES = []          # M4 holes: through a frame part foot, heat-set insert in the tile under it
plates = [side_plate(1), side_plate(-1)]
for i, pl in enumerate(plates):
    y0_, y1_ = (FY if i == 0 else -FY - T) - 12, (FY + T if i == 0 else -FY) + 12
    pl = pl + box(-70, 70, y0_, y1_, 0, 6)
    for hx in (-62, 62):
        for hy in (y0_ + 5, y1_ - 5):
            pl = pl - cyl(2.2, 8, hx, hy, -1)
            DECK_HOLES.append((hx, hy))
    plates[i] = pl
add("frame_plate_R", plates[0], "world", "frame", "#b0b8c0", note="+y side: main shaft bearing, motor side")
add("frame_plate_L", plates[1], "world", "frame", "#b0b8c0", note="-y side: main shaft bearing")

# roller post (+y side): cam roller on 2 x 625ZZ and a 5 mm steel pin supported on both sides (U bracket)
rx, rz = P["ROLL_X"], P["ROLL_Z"]
c0, c1 = P["CAM_Y0"] - 1.2, P["CAM_Y1"] + 1.2
post = union([box(rx - 10, rx + 10, c1 + 0.3, c1 + 8.3, 0, rz + 10), cylY(10, c1 + 0.3, c1 + 8.3, rx, rz),
              box(rx - 10, rx + 10, c0 - 4.3, c0 - 0.3, 0, rz + 10), cylY(10, c0 - 4.3, c0 - 0.3, rx, rz),
              box(rx - 10, rx + 10, c0 - 4.3, c1 + 8.3, 0, rz - 18),
              box(rx - 22, rx + 22, c0 - 6, c1 + 20, 0, 6)])
post = post - cylY(4.05, c0 - 5, c1 + 9, rx, rz)
add("roller_post", post, "world", "frame", "#b0b8c0", note="cam roller U bracket, 5 mm pin supported both sides")
ymid = (P["CAM_Y0"] + P["CAM_Y1"]) / 2
roller = cylY(P["ROLL_R"], ymid - 3.5, ymid + 3.5, rx, rz) - cylY(4.0, c0 - 1, c1 + 1, rx, rz)
add("cam_roller", roller, "world", "hardware", "#606060", printed=False, note="608ZZ (8 x 22 x 7) is the roller itself: steel on the PPA-CF cam, two M8 washers either side")
add("roller_pin", cylY(4.0, c0 - 4.3, c1 + 7.6, rx, rz), "world", "hardware", "#8a8f94", printed=False, note="8 x 22 steel pin: push it in from the inside before the rail goes on")

# bumper post: rail rests here at TH_LOW (cam 0.6 clear). Bumper under the rail beam at u=200 under the side walls
BU = 205.0
bx, bz = rail_to_world(BU, 30.0, P["TH_LOW"])
bump = []
for s in (1, -1):
    bump.append(box(bx - 12, bx + 12, min(s * 40, s * 50), max(s * 40, s * 50), 0, bz - 15.7))
    bump.append(box(bx - 24, bx + 20, min(s * 36, s * 54), max(s * 36, s * 54), 0, 6))
bump.append(box(bx + 6, bx + 12, 50, 60, bz - 32, bz - 16.5))         # boss for SW_LOW
for s in (1, -1):                                                     # wider top for the 16 mm bumper
    bump.append(box(bx - 12, bx + 12, min(s * 37.5, s * 52.5), max(s * 37.5, s * 52.5), bz - 25.7, bz - 15.7))
bump_s = union(bump)
for s in (1, -1):
    for hx in (bx - 19, bx + 16):
        bump_s = bump_s - cyl(2.2, 8, hx, s * 45, -1)
        DECK_HOLES.append((hx, s * 45))
    bump_s = bump_s - cyl(4.0, 10.5, bx, s * 45, bz - 26.0)          # M6 heat-set insert (8 x 10) for the bumper stud
add("bumper_posts", bump_s, "world", "frame", "#b0b8c0", note="rail rests here at 4 deg on 2 bumpers 16 x 15 (M6 stud into an M6 heat-set insert)")
for s in (1, -1):
    add("rubber_foot_%s" % ("R" if s > 0 else "L"), cyl(8.0, 15.0, bx, s * 45, bz - 15.7), "world", "hardware", "#202020", printed=False, note="bumper 16 dia x 15 (sorbothane or 60A rubber, M6 stud): the rail free-falls about 0.9 J onto these every shot")

# ================================================================= cord trip
def cord_A_world(delta, th):
    a = math.radians(delta); pu, pw = SEAR_PIV
    du, dw = CORD_A[0] - pu, CORD_A[1] - pw
    u = pu + du * math.cos(a) - dw * math.sin(a); w = pw + du * math.sin(a) + dw * math.cos(a)
    X, Z = rail_to_world(u, w, th); return np.array([X, CORD_Y, Z])


DELTA_REL = None   # set below from the meshes


def cord_delta(th, F, L):
    """sear angle forced by the cord at rail angle th (0 when slack)"""
    if np.linalg.norm(cord_A_world(0, th) - F) <= L: return 0.0
    lo, hi = -40.0, 0.0
    for _ in range(40):
        m = 0.5 * (lo + hi)
        if np.linalg.norm(cord_A_world(m, th) - F) > L: hi = m
        else: lo = m
    return lo

# ---- sear kinematics from the real meshes (no assumed rules): hook vs keel, release angle, creep
SEAR0 = parts["sear"]["solid"]; SLED0 = parts["sled"]["solid"]
def sear_at(d):
    pu, pw = SEAR_PIV
    return SEAR0.translate((-pu, 0, -pw)).rotate((0, -d, 0)).translate((pu, 0, pw))
def clash(d, sp):
    return (sear_at(d) ^ SLED0.translate((sp, 0, 0))).volume() > 0.02
s_set = P_D + OVERTRAVEL
while clash(0.0, s_set) and s_set > P_D:          # where the keel really comes to rest on the arc face
    s_set -= 0.02
P["S_SETTLED"] = s_set - 0.05
keel_tab = []
for sp in np.concatenate([np.arange(s_set + 0.02, s_set + 25.0, 0.2), np.arange(s_set + 25.0, s_set + 75.0, 0.5)]):
    d = 0.0
    while clash(d, sp) and d > -40:
        d -= 0.25
    keel_tab.append([float(sp), float(d - 0.9) if d < 0 else 0.0])      # 0.9 deg margin for interpolation
# release: hook clear of the keel over the whole forward run from the settled position
DELTA_REL = 0.0
while any(clash(DELTA_REL, s_set + ds) for ds in (0.5, 3, 6, 10, 14)) and DELTA_REL > -40:
    DELTA_REL -= 0.25
creep_tab = []
for d in np.arange(0, DELTA_REL - 0.01, -0.5):
    ds = 0.0
    while clash(float(d), s_set - ds) and ds < 8:
        ds += 0.1
    creep_tab.append([float(d), float(ds)])
P.update(SEAR_KEEL_TAB=keel_tab, DELTA_REL=DELTA_REL, CREEP_TAB=creep_tab, S_SET=s_set)
print("sear: keel lifts it to %.1f deg at most, release at %.2f deg, max creep %.1f mm" % (min(d for _, d in keel_tab), DELTA_REL, max(c for _, c in creep_tab)))
# ---- cord anchor: search the eye position so the cord goes taut late, trips firmly near the top of the lift, never drives
# the sear past its down-stop, and never touches a part (the cord runs in the plane y = CORD_Y, outside the cheek)
TRIP_TH = P["TH_FIRE"] - 1.25         # full release by here (a long cord only fires later, up to about 4 mm)
RR_CHK = parts["rail_rear"]["solid"] + parts["sw_cocked"]["solid"] + parts["rail_beam"]["solid"]
def cord_clear(F, L, ths):
    for th in ths:
        dl = cord_delta(th, F, L)
        if dl >= 0: continue
        A = cord_A_world(dl, th)
        # cord in rail frame: rotate the world points back
        pts = []
        for Pt in (A, F):
            u, w = world_to_rail(Pt[0], Pt[2], th); pts.append((u, w))
        (u0, w0), (u1, w1) = pts
        n = int(math.hypot(u1 - u0, w1 - w0) / 3) + 2
        segs = []
        for k in range(n):
            t0_, t1_ = k / n, (k + 1) / n
            if t0_ < 0.03: t0_ = 0.03                      # the knot end on the pin itself
            ua, wa = u0 + (u1 - u0) * t0_, w0 + (w1 - w0) * t0_
            ub, wb = u0 + (u1 - u0) * t1_, w0 + (w1 - w0) * t1_
            segs.append(box(min(ua, ub) - 0.8, max(ua, ub) + 0.8, CORD_Y - 0.8, CORD_Y + 0.8, min(wa, wb) - 0.8, max(wa, wb) + 0.8))
        if (union(segs[:-2]) ^ RR_CHK).volume() > 0.05:
            return False
    return True
best, cands = None, []
for FX in np.arange(110, 151, 4):        # behind the left bumper post, clear of the electronics box
    for FZ in np.arange(46, 91, 4):
        F = np.array([FX, CORD_Y, FZ])
        L = np.linalg.norm(cord_A_world(DELTA_REL - 2.0, TRIP_TH) - F)
        tab = [(th, cord_delta(th, F, L)) for th in np.arange(P["TH_LOW"], P["TH_FIRE"] + 0.01, 0.5)]
        taut = next((th for th, d in tab if d < -0.1), 99)
        slope = abs(cord_delta(TRIP_TH, F, L) - cord_delta(TRIP_TH - 2, F, L)) / 2
        if taut < 24 or slope < 2.0 or cord_delta(P["TH_FIRE"], F, L) < DOWN_STOP + 2.5:
            continue
        cands.append((-FZ, FX, F, L, taut, slope))
for c in sorted(cands, key=lambda c: (c[0], -c[1]), reverse=True):     # lowest eye first
    _, _, F, L, taut, slope = c
    if cord_clear(F, L, np.arange(c[4], P["TH_FIRE"] + 0.01, 2.0)):
        best = c; break
if best is None:
    raise RuntimeError("no cord anchor found")
_, _, CORD_F, CORD_L, taut, slope = best
cord_tab = [[float(th), float(cord_delta(th, CORD_F, CORD_L))] for th in np.arange(0, 46.01, 0.25)]
th_rel = next(th for th, d in cord_tab if d <= DELTA_REL)
P.update(CORD_F=CORD_F.tolist(), CORD_L=float(CORD_L), SEAR_CORD_TAB=cord_tab, TH_RELEASE=float(th_rel), CORD_Y=CORD_Y, DOWN_STOP=DOWN_STOP)
print("cord anchor X %.0f Z %.0f, cord %.1f mm: taut at %.1f deg, %.1f deg sear per deg rail, shot at rail %.1f deg, sear %.1f at %.0f" %
      (CORD_F[0], CORD_F[2], CORD_L, taut, slope, th_rel, cord_delta(P["TH_FIRE"], CORD_F, CORD_L), P["TH_FIRE"]))
FX, FZ = float(CORD_F[0]), float(CORD_F[2])
# post: column in front of the eye, a shelf under it. The M4 eye bolt drops through the shelf: a nut on the shank above
# the shelf sets the eye height (positive, threaded: turn it to fire sooner or later), a preloaded compression spring and
# nut under the shelf let the eye give if the cord is ever overdriven (the sear sits on its down-stop then).
SH_T = FZ - 9.0                       # shelf top (eye ring bottom + adjuster nut)
post = union([box(FX + 6, FX + 16, -57.0, -41.0, 0, SH_T), box(FX - 7, FX + 16, -55.0, -43.0, SH_T - 6, SH_T),
              box(FX - 10, FX + 24, -64.0, -34.0, 0, 6), box(FX + 6, FX + 16, -64.0, -34.0, 0, 14)])
post = post - cyl(2.3, 8, FX, CORD_Y, SH_T - 7) - cyl(5.0, 8, FX, CORD_Y, -1)        # shelf hole; clearance under the bolt end
TRIP_FEET = [(FX - 5, -59.0), (FX + 20, -59.0), (FX - 5, -39.0), (FX + 20, -39.0)]
for (hx, hy) in TRIP_FEET:
    post = post - cyl(2.2, 8, hx, hy, -1)
EYE_L = 35.0                         # M4 x 35 eye bolt: the shank ends 5 mm above the base, clear of the flange
add("trip_anchor", post, "world", "frame", "#b0b8c0", note="trip cord anchor (PPA-CF): M4 x 35 eye bolt through the shelf, height nut above, preloaded spring below")
add("trip_eye", union([cylY(6.0, CORD_Y - 1.5, CORD_Y + 1.5, FX, FZ) - cylY(3.5, CORD_Y - 2, CORD_Y + 2, FX, FZ),
                       cyl(2.0, EYE_L, FX, CORD_Y, FZ - 6 - EYE_L), cyl(3.6, 3.2, FX, CORD_Y, SH_T),
                       cyl(3.6, 3.2, FX, CORD_Y, SH_T - 26.0 - 3.2)]), "world", "hardware", "#8a8f94", printed=False,
    note="M4 x 35 eye bolt, height nut + washer on the shelf, nut under the spring")
add("trip_spring", cyl(5.0, 20.0, FX, CORD_Y, SH_T - 6 - 20.0) - cyl(2.6, 22, FX, CORD_Y, SH_T - 27), "world", "hardware", "#c0c4c8", printed=False,
    note="compression spring 5 ID x 10 OD x 25 free, about 15 N/mm (light die spring), preloaded 5 mm = 75 N: gives only if the cord is overdriven")
FX0, FX1, TY0, TY1 = FX - 10, FX + 24, -64.0, -34.0

# ---- rail check strap: the dart leaves 95 mm above the pivot and kicks the rail muzzle-up (about 0.29 N m s). A 2 mm
# Dyneema strap from a pin on the right cheek to a second sprung eye post goes taut at rail 36 deg (the cam tops out at 35)
# so the frame takes the kick, not the trip cord. Same post as the trip anchor, mirrored to y +50.
STRAP_A, STRAP_Y, STRAP_TH = (150.0, 20.0), 50.0, P["TH_FIRE"] + 2.5
SX, SZ = 130.0, 72.0
def strap_A_world(th):
    X, Z = rail_to_world(STRAP_A[0], STRAP_A[1], th); return np.array([X, STRAP_Y, Z])
SF = np.array([SX, STRAP_Y, SZ])
STRAP_L = float(np.linalg.norm(strap_A_world(STRAP_TH) - SF))
dists = [float(np.linalg.norm(strap_A_world(t) - SF)) for t in np.arange(P["TH_LOW"], STRAP_TH + 0.01, 0.5)]
assert all(b > a for a, b in zip(dists, dists[1:])), "strap must tighten only as the rail rises"
P.update(STRAP_A=list(STRAP_A), STRAP_F=SF.tolist(), STRAP_L=STRAP_L, STRAP_TH=STRAP_TH)
print("check strap: taut at rail %.1f deg, %.1f mm between centres (slack %.1f mm at 4 deg)" % (STRAP_TH, STRAP_L, STRAP_L - dists[0]))
SSH = SZ - 9.0
spost_ = union([box(SX + 6, SX + 16, 41.0, 57.0, 0, SSH), box(SX - 7, SX + 16, 43.0, 55.0, SSH - 6, SSH),
                box(SX - 10, SX + 24, 34.0, 64.0, 0, 6), box(SX + 6, SX + 16, 34.0, 64.0, 0, 14)])
spost_ = spost_ - cyl(2.3, 8, SX, STRAP_Y, SSH - 7) - cyl(5.0, 8, SX, STRAP_Y, -1)
STRAP_FEET = [(SX - 5, 59.0), (SX + 20, 59.0), (SX - 5, 39.0), (SX + 20, 39.0)]
for (hx, hy) in STRAP_FEET:
    spost_ = spost_ - cyl(2.2, 8, hx, hy, -1)
add("strap_anchor", spost_, "world", "frame", "#b0b8c0", note="rail check strap anchor (PPA-CF): M4 x 60 eye bolt, height nut, long preloaded die spring: takes the recoil kick")
add("strap_eye", union([cylY(6.0, STRAP_Y - 1.5, STRAP_Y + 1.5, SX, SZ) - cylY(3.5, STRAP_Y - 2, STRAP_Y + 2, SX, SZ),
                        cyl(2.0, 60.0, SX, STRAP_Y, SZ - 6 - 60.0), cyl(3.6, 3.2, SX, STRAP_Y, SSH),
                        cyl(3.6, 3.2, SX, STRAP_Y, SSH - 6.0 - 35.0 - 3.2)]), "world", "hardware", "#8a8f94", printed=False,
    note="M4 x 60 eye bolt, height nut + washer on the shelf, nut under the spring")
add("strap_spring", cyl(5.0, 35.0, SX, STRAP_Y, SSH - 6 - 35.0) - cyl(2.6, 37, SX, STRAP_Y, SSH - 42), "world", "hardware", "#c0c4c8", printed=False,
    note="die spring 5 ID x 10 OD x 40 free (light, about 15 N/mm, 16 mm travel), preloaded 5 mm: absorbs about 1.5 J of rail kick over 10 mm")
add("strap_pin", cylY(1.5, 40.0, STRAP_Y + 3.0, STRAP_A[0], STRAP_A[1]) + cylY(3.0, STRAP_Y + 1.8, STRAP_Y + 2.4, STRAP_A[0], STRAP_A[1]),
    "rail", "hardware", "#8a8f94", printed=False, note="3 x 16 grooved pin pressed into the right cheek, e-clip: the check strap ties on")
parts["rail_rear"]["solid"] = parts["rail_rear"]["solid"] - cylY(1.45, 39.0, 47.0, STRAP_A[0], STRAP_A[1])
DECK_HOLES.extend(STRAP_FEET)
SX0, SX1, SY0, SY1 = SX - 10, SX + 24, 34.0, 64.0
DECK_HOLES.extend(TRIP_FEET)

# base plates
# (base tiles are built at the end, after every footprint is known)

# wiper motor envelope and adapter
WY = FY + T + 20
add("wiper_motor", union([box(-60, 60, WY + 24, WY + 84, H - 45, H + 45), box(-72, -60, WY + 30, WY + 78, H - 41, H + 31), cylX(36, -182, -72, WY + 54, H - 5)]),
    "world", "hardware", "#333840", printed=False, note="12 V car wiper motor (envelope)")
coupler = (cylY(11, FY + T + 0.5, WY + 14) + cylY(38, WY + 8, WY + 14) - cylY(6.05, FY, WY + 15)
           - box(-3.3, 3.3, WY + 7, WY + 15, 14, 36) - box(-3.3, 3.3, WY + 7, WY + 15, -36, -14))
coupler = dflat(coupler, FY + T + 0.5, WY + 14, 6.05)
for yy in (FY + T + 5, FY + T + 13):
    coupler = coupler - cyl(1.65, 8, 0, yy, 4.5)              # 2 x M4 grub (tapped 3.3) onto the shaft flat
add("wiper_coupler", coupler, "main", "frame", "#e0a030",
    note="clamps the D flat at the main shaft right end (2 x M4 grubs), bolts to the wiper's own crank arm through two slots")
mp = box(-70, 70, WY + 18, WY + 24, 0, H + 60) - cylY(22, WY + 17, WY + 25, 0, H)
for k in range(3):
    a = math.radians(90 + 120 * k)
    for r in np.arange(30, 56, 1.0):
        mp = mp - cylY(3.3, WY + 17, WY + 25, r * math.cos(a), H + r * math.sin(a))
mp = mp + box(-70, 70, FY + T + 13, WY + 24, 0, 8)
for hx in (-62, -50, 50, 62):                     # y 136: clear of the y 128 tile seam, outside the coupler disc
    mp = mp - cyl(2.2, 10, hx, FY + T + 32, -1); DECK_HOLES.append((hx, FY + T + 32))
add("motor_plate", mp, "world", "frame", "#b0b8c0", note="wiper adapter: 3 radial slots fit most wiper bolt circles (r 30-55)")

# ================================================================= non printed: springs, cords, bearings, dart
for s, nm in ((1, "R"), (-1, "L")):
    pl_ = cylY(pul_r + 3.0, min(s * 26.5, s * 34.5), max(s * 26.5, s * 34.5), pul_u, pul_w) - (cylY(pul_r + 3.1, min(s * 28.0, s * 33.0), max(s * 28.0, s * 33.0), pul_u, pul_w)
           - cylY(pul_r - 0.5, -60, 60, pul_u, pul_w))                                          # 3 mm flanges, cord groove
    add("pulley_%s" % nm, pl_ - cylY(4.05, -40, 40, pul_u, pul_w),
        "rail", "launcher", "#e0e0e0", note="flanged, grooved cord pulley (slack cord cannot jump off) on 2x F693ZZ, M3 x 35 axle from outside the side wall, nyloc inside")
for y0 in (38.0, -46.0):
    add("brg_pivot_%s" % ("R" if y0 > 0 else "L"), bearing(0, y0, 0, P["BRG_OD"], P["BRG_W"]), "rail", "hardware", "#666666", printed=False, note="6001-2RS pivot")
    add("brg_pg_%s" % ("R" if y0 > 0 else "L"), bearing(P["PG_U"], y0, 0, P["BRG_OD"], P["BRG_W"]), "rail", "hardware", "#666666", printed=False, note="6001-2RS PG stub")
for s in (1, -1):
    add("brg_frame_%s" % ("R" if s > 0 else "L"), bearing(0, FY if s > 0 else -FY - T, H, P["BRG_OD"], P["BRG_W"]), "world", "hardware", "#666666", printed=False, note="6001-2RS frame")

# ================================================================= ADDITIONS: stops, switches, sensor, electronics, saddle
# carriage end stop: the carriage arrives with about 3.1 J. Two 90A urethane pads 40 mm long (about 24 % strain, about
# 700 N total), the rear 27 mm sunk in pockets in solid blocks that are one piece with the side wall and the lip
U_STOP = P_R + P["SHOE_L"]
stops = []
for sg_ in (1, -1):
    blk = box(U_STOP + 14, U_STOP + 54, min(sg_ * 24.5, sg_ * 42.2), max(sg_ * 24.5, sg_ * 42.2), 81.0, 89.5)
    blk = blk - box(U_STOP + 13, U_STOP + 40.5, min(sg_ * 26.0, sg_ * 41.0), max(sg_ * 26.0, sg_ * 41.0), 81.3, 88.2)   # pad pocket, room to bulge
    blk = blk - cylX(2.5, U_STOP, U_STOP + 60, sg_ * P["SPRING_Y"], 85.0)                                               # the spring cord runs through
    stops.append(blk)
    add("stop_pad_%s" % ("R" if sg_ > 0 else "L"), box(U_STOP + 0.2, U_STOP + 40, min(sg_ * 26.5, sg_ * 40.5), max(sg_ * 26.5, sg_ * 40.5), 81.5, 88.0)
        - cylX(2.0, U_STOP, U_STOP + 41, sg_ * P["SPRING_Y"], 85.0),
        "rail", "hardware", "#202020", printed=False, note="urethane stop 40 x 14 x 6.5 (90A) with a 4 mm hole along it for the spring cord; rear 27 mm in its pocket")
parts["rail_beam"]["solid"] = parts["rail_beam"]["solid"] + union(stops)
# muzzle zone past the carriage stops: no shoe lips or shelves, cradle edges down to w 76, walls down to w 80, with a 45 deg
# lead-in. A folded dart coming back up the horn on retrieve fits through here rolled up to about +-47 deg (was +-8).
MZ0 = U_STOP + 56.0
cuts = []
for uu in np.arange(MZ0, U_M + 2.0, 2.0):
    wc = max(76.0, 92.0 - (uu - MZ0))
    for sg_ in (1, -1):
        cuts.append(box(uu, uu + 2.0, min(sg_ * 18.0, sg_ * 42.2), max(sg_ * 18.0, sg_ * 42.2), max(wc, 77.0), 93.0))
        cuts.append(box(uu, uu + 2.0, min(sg_ * 42.0, sg_ * 48.5), max(sg_ * 42.0, sg_ * 48.5), max(wc, 80.0), 93.0))
    cuts.append(box(uu, uu + 2.0, -26.0, 26.0, max(wc, 76.0), 82.0))
parts["rail_beam"]["solid"] = parts["rail_beam"]["solid"] - union(cuts)
P["MUZZLE_OPEN_U"] = MZ0
# retrieve roll funnel: from the open muzzle back to the shoe lips the free space is the envelope of the folded dart rolled
# up to +-theta(u), theta narrowing from 47 deg at the muzzle to 8 deg where the lips start. A rolled dart dragged in tail
# first is turned upright by these surfaces instead of jamming on the lip ends.
def dart_outline():
    G = json.load(open(f"{OUT}/dart_groups.json")); pts = []
    for n_, g_ in G.items():
        if n_.startswith("_"): continue
        v_ = np.load(f"{OUT}/{n_}.npz")["v"].astype(float)
        if g_["pivot"] is not None: v_ = v_ + np.array(g_["pivot"])
        pts.append(v_)
    A_ = np.vstack(pts); polys = []
    for x0_ in np.arange(A_[:, 0].min(), A_[:, 0].max(), 2.0):
        s_ = A_[(A_[:, 0] >= x0_) & (A_[:, 0] < x0_ + 2)]
        for lo, hi in ((-99, -22), (-22, 22), (22, 99)):
            t_ = s_[(s_[:, 1] >= lo) & (s_[:, 1] < hi)]
            if len(t_) > 3:
                polys.append(MultiPoint([(-q[1], q[2]) for q in t_]).convex_hull)     # rail (y, w - axis): the dart frame is turned 180 deg
    return unary_union(polys)
from shapely.geometry import MultiPoint
if os.path.exists(f"{OUT}/dart_groups.json"):
    DOUT = dart_outline()
    FUN0, FUN1, TH0, TH1 = MZ0, U_M - 12.0, 8.0, 47.0
    fun = []
    for uu in np.arange(FUN0, FUN1, 2.0):
        thf = TH0 + (TH1 - TH0) * (uu - FUN0) / (FUN1 - FUN0)
        env = unary_union([affinity.translate(affinity.rotate(DOUT, a_, origin=(0, 0)), 0, ax_)
                           for a_ in np.arange(-thf, thf + 0.01, 2.0) for ax_ in (94.0, 95.0, 96.0)]).buffer(0.8)
        sec = sbox(-42.2, 76.0, 42.2, 92.0).difference(env)
        for sg_ in (1, -1):
            sec = sec.difference(Point(sg_ * P["SPRING_Y"], 85.0).buffer(3.0))            # spring cords run here
            if uu + 2 > pul_u - pul_r - 4 and uu < pul_u + pul_r + 4:
                sec = sec.difference(sbox(min(sg_ * 25.0, sg_ * 36.5), 0, max(sg_ * 25.0, sg_ * 36.5), 93))   # pulleys
        sec = sec.buffer(-0.3, join_style=2).buffer(0.3, join_style=2).simplify(0.3)
        if not sec.is_empty:
            fun.append(ext_u(sec, uu, uu + 2.05))
    parts["rail_beam"]["solid"] = parts["rail_beam"]["solid"] + union(fun)
    print("retrieve funnel: %d slices, roll window 8 -> 47 deg over u %.0f..%.0f" % (len(fun), FUN0, FUN1))

# SW_SEATED: roller lever up through the cradle 4 mm in front of the seated tail; only the seated dart covers it
SU = P_D + OVERTRAVEL + 2.0          # roller meets the tail cap 2 mm from its end face, behind the JST window
microswitch("sw_seated", "rail", SU - 3, 15.5, 55.0, ("x", "y", "z"),
            box(SU - 1, SU + 1, 11.0, 16.0, 65.3, 80.4), "SW_SEATED: KW11 roller lever through the cradle, closed only when the dart tail is within 4 mm of the pusher")
parts["rail_beam"]["solid"] = parts["rail_beam"]["solid"] - box(SU - 3, SU + 3, 9.4, 16.5, 64, 81) + \
    (box(SU - 5, SU + 19, 12.0, 24.0, 51.0, 55.0)) + box(SU - 5, SU + 19, 21.95, 24.9, 51.0, 63.0)
for du_ in (5.25, 14.75):                                             # KW11 mounting holes: M2 self tapping into the web
    parts["rail_beam"]["solid"] = parts["rail_beam"]["solid"] - cylY(0.9, 21, 25.5, SU - 3 + du_, 55.0 + 2.9)

# SW_LOW: on the +y bumper post, its lever touches the rail side wall bottom when the rail is down
bxw, bzw = rail_to_world(205.0, 30.0, P["TH_LOW"])
for dy_ in (5.25, 14.75):
    parts["bumper_posts"]["solid"] = parts["bumper_posts"]["solid"] - cylX(0.9, bxw + 5, bxw + 12.5, 40.0 + dy_, bzw - 28.0 + 2.9)
microswitch("sw_low", "world", bxw + 12.0, 40.0, bzw - 6.0 - 22.0, ("y", "x", "z"),
            box(bxw + 13.0, bxw + 15.0, 43.0, 47.0, bzw - 12.0, bzw - 0.9), "SW_LOW: KW11 lever switch, closed while the rail rests on the bumpers")

# AS5600 on the main shaft's -y end (magnet glued in the end, board 2 mm away)
add("as5600_magnet", cylY(3, Y_SH0 - 2.5, Y_SH0), "main", "electronics", "#7a2020", printed=False, note="6 x 2.5 diametral magnet in a 3 mm printed or brass cap on the shaft end (not glued straight onto the hardened steel)")
add("as5600_board", box(-11.5, 11.5, Y_SH0 - 6.6, Y_SH0 - 5.0, H - 11.5, H + 11.5), "world", "electronics", "#1f6f3f", printed=False, note="AS5600 board, chip on the shaft axis")
asb = box(-18, 18, -FY - T - 22.0, -FY - T, H - 18, H + 18) - box(-12.5, 12.5, Y_SH0 - 6.8, Y_SH0 - 4.8, H - 12.5, H + 12.5) \
      - cylY(8.0, -FY - T - 23, -FY - T + 1, 0, H) - box(-12.5, 12.5, -FY - T - 23, Y_SH0 - 6.6, H - 9, H + 9)
for (hx, hz) in ((-15, H - 15), (15, H - 15), (-15, H + 15), (15, H + 15)):
    asb = asb - cylY(1.7, -FY - T - 23, -FY - T + 1, hx, hz)
    parts["frame_plate_L"]["solid"] = parts["frame_plate_L"]["solid"] - cylY(1.25, -FY - T - 1, -FY + 1, hx, hz)
add("as5600_bracket", asb, "world", "frame", "#b0b8c0", note="screws to the left plate, holds the AS5600 over the shaft end")

# electronics box under the muzzle (the rail is high above it at every angle)
EX0, EX1, EY0, EY1 = 225.0, 309.0, -76.0, 76.0
ebox = box(EX0, EX1, EY0, EY1, 0, 52) - box(EX0 + 3, EX1 - 3, EY0 + 3, EY1 - 3, 3, 53)
ebox = ebox - box(EX1 - 4, EX1 + 1, -14, 14, 10, 26)            # XT60 window
ebox = ebox - box(EX0 + 60, EX0 + 76, EY1 - 4, EY1 + 1, 8, 22)    # wiper 14 AWG window, +y, next to the BTS7960 terminals
ebox = ebox - box(EX0 + 12, EX0 + 30, EY0 - 1, EY0 + 4, 4, 16)    # USB-C slot for the ESP32, -y wall
ebox = ebox - box(EX0 - 1, EX0 + 4, -14, 14, 6, 22)               # sensor cable window, -x face, below the vents
for yy in (-40.0, 0.0, 40.0):
    ebox = ebox - box(EX0 - 1, EX0 + 4, yy - 9, yy + 9, 30, 46)  # vents
for (hx, hy) in ((EX0 + 15, 8.0), (EX0 + 45, 8.0), (EX1 - 13, -66.0), (EX1 - 13, 66.0)):
    ebox = ebox - cyl(2.2, 6, hx, hy, -1); DECK_HOLES.append((hx, hy))
add("ebox", ebox, "world", "frame", "#9aa4ae", note="electronics box: ESP32-S3 DevKitC-1, BTS7960, fuse, XT60")
add("ebox_lid", box(EX0, EX1, EY0, EY1, 52, 55) - box(EX0 + 10, EX1 - 10, -2, 2, 51, 56), "world", "frame", "#c9d1d9", note="lid, snaps on")
add("bts7960", box(EX0 + 8, EX0 + 58, EY1 - 58, EY1 - 8, 3, 46), "world", "electronics", "#1d4f91", printed=False, note="BTS7960 43 A driver (envelope)")
add("esp32s3", box(EX0 + 8, EX0 + 34, EY0 + 8, EY0 + 72, 3, 16), "world", "electronics", "#1b1b1b", printed=False, note="ESP32-S3 DevKitC-1 (envelope)")

# motor saddle under the wiper motor can
WY = FY + T + 20
can_y, can_z, can_r = WY + 54, H - 5, 36.0
sad = box(-150, -90, can_y - 30, can_y + 30, 0, 50.0) - cylX(can_r + 0.5, -151, -89, can_y, can_z)
for hx in (-144, -96):
    for hy in (can_y - 24, can_y + 24):
        sad = sad - cyl(2.2, 10, hx, hy, -1) - cyl(4.2, 50, hx, hy, 8)
        DECK_HOLES.append((hx, hy))
sad = sad - box(-136, -104, can_y - 31, can_y + 31, 8, 14)          # strap slot (zip tie / hose clamp)
add("motor_saddle", sad, "world", "frame", "#b0b8c0", note="cradles the wiper motor can (lower half), a hose clamp or strap through the slot holds it down; 4 x M4 into the tile")

# base tiles (each bolts to the rover deck with its own 4 holes; the frame parts tie them)
bt = P["BASE_T"]
foot = []
FEET = []          # frame part footprints on the tiles (deck screw heads stay out from under them)
def fp(x0, x1, y0, y1): foot.append(box(x0 - 6, x1 + 6, y0 - 6, y1 + 6, -bt, 0)); FEET.append((x0, x1, y0, y1))
fp(-70, 70, FY - 12, FY + T + 12); fp(-70, 70, -FY - T - 12, -FY + 12)       # side plates
fp(rx - 22, rx + 22, P["CAM_Y1"] - 4, P["CAM_Y1"] + 20)
fp(bxw - 20, bxw + 20, 36, 54); fp(bxw - 20, bxw + 20, -54, -36)              # bumper posts
fp(FX0, FX1, TY0, TY1)                                                        # trip cord anchor post
fp(SX0, SX1, SY0, SY1)                                                        # rail check strap post
fp(-70, 70, FY + T + 13, WY + 24)                                             # motor plate foot
fp(EX0, EX1, EY0, EY1); fp(-150, -90, can_y - 30, can_y + 30)
plate_all = box(-160, 315, -132, 128, -bt, 0) + box(-160, 72, 128, 214, -bt, 0)
pock = []
for xx in np.arange(-150, 310, 46):
    for yy in np.arange(-124, 210, 46):
        pock.append(box(xx, xx + 34, yy, yy + 34, -bt - 1, 1))
plate_all = (plate_all - union(pock)) + (union(foot) ^ plate_all)
plate_all = plate_all + (box(-160, 315, -132, 128, -bt, 0) - box(-152, 307, -124, 120, -bt - 1, 1))   # rim
plate_all = plate_all + (box(-160, 72, 128, 214, -bt, 0) - box(-152, 64, 128, 206, -bt - 1, 1))
TILES = [("base_1", (-160, 72, -132, 0)), ("base_2", (-160, 72, 0, 128)), ("base_3", (72, 315, -132, 0)), ("base_4", (72, 315, 0, 128)), ("base_5", (-160, 72, 128, 214))]
for nm, (x0, x1, y0, y1) in TILES:
    t_ = plate_all ^ box(x0 + 0.25, x1 - 0.25, y0 + 0.25, y1 - 0.25, -bt - 1, 1)
    t_ = t_ + (box(x0 + 0.25, x1 - 0.25, y0 + 0.25, y1 - 0.25, -bt, 0) - box(x0 + 8, x1 - 8, y0 + 8, y1 - 8, -bt - 1, 1))   # tile rim
    TILE_HOLES = []
    for (hx, hy) in ((x0 + 14, y0 + 14), (x1 - 14, y0 + 14), (x0 + 14, y1 - 14), (x1 - 14, y1 - 14)):
        sx, sy = (1 if hx < (x0 + x1) / 2 else -1), (1 if hy < (y0 + y1) / 2 else -1)
        cands_ = [(hx + sx * 6 * (k % 4), hy + sy * 6 * (k // 4)) for k in range(16)]
        clear_ = lambda c: all(math.hypot(c[0] - a, c[1] - b) >= 12 for a, b in DECK_HOLES)
        under_ = lambda c: any(fx0 - 4 <= c[0] <= fx1 + 4 and fy0 - 4 <= c[1] <= fy1 + 4 for (fx0, fx1, fy0, fy1) in FEET)
        pick_ = next((c for c in cands_ if clear_(c) and not under_(c)), None) or next((c for c in cands_ if clear_(c)), None)
        if pick_: hx, hy = pick_                  # clear of every frame insert (12 mm), preferably not under a foot (heads are flush anyway)
        TILE_HOLES.append((hx, hy)); P.setdefault("TILE_HOLES", []).append([float(hx), float(hy)])
        t_ = t_ + cyl(7, bt, hx, hy, -bt) - cyl(2.2, bt + 2, hx, hy, -bt - 1) - cyl(4.2, 5, hx, hy, -4.0)   # M4 cap head counterbored flush
    for (hx, hy) in DECK_HOLES:
        t_ = t_ - cyl(2.8, bt + 2, hx, hy, -bt - 1)                   # M4 heat-set insert
    add(nm, t_, "world", "frame", "#80888f", note="base tile: 4 x M4 to the rover deck; heat-set M4 inserts take the frame parts")

# lightening windows
lw = []
for uu in np.arange(140, P["U_MUZ"] - 60, 44):
    if abs(uu - 205) < 30 or abs(uu - SU) < 20:
        continue
    for sg_ in (1, -1):
        lw.append(box(uu, uu + 30, min(sg_ * 41, sg_ * 49), max(sg_ * 41, sg_ * 49), 42, 70))
parts["rail_beam"]["solid"] = parts["rail_beam"]["solid"] - union(lw)
def spokes(sol, cx, cz, r0, r1, n, y0, y1, frame_xz=True):
    holes = []
    for k_ in range(n):
        a_ = 2 * math.pi * (k_ + 0.5) / n
        rm = (r0 + r1) / 2; rr = (r1 - r0) / 2
        holes.append(cylY(rr, y0 - 1, y1 + 1, cx + rm * math.cos(a_), cz + rm * math.sin(a_)))
    return sol - union(holes)
parts["reel_main_gear"]["solid"] = spokes(parts["reel_main_gear"]["solid"], 0, 0, 18, 44, 5, y_g0, y_g1)
parts["reel_stub_gear"]["solid"] = spokes(parts["reel_stub_gear"]["solid"], 0, 0, 13, 32, 5, -200, 200)
parts["program_gear"]["solid"] = spokes(parts["program_gear"]["solid"], 0, 0, 20, 30, 6, -8, 8)     # rim stays 11 mm deep under the teeth
mpw = []
for (xx, zz) in ((-48, 22), (48, 22), (-52, H + 40), (52, H + 40)):
    mpw.append(box(xx - 12, xx + 12, WY + 17, WY + 25, zz - 12, zz + 12))
parts["motor_plate"]["solid"] = parts["motor_plate"]["solid"] - union(mpw)

# ================================================================= axial location: spacer sleeves (printed)
def sleeve(r_o, r_i, y0, y1):
    return cylY(r_o, y0, y1) - cylY(r_i, y0 - 1, y1 + 1)
add("pinion_spacer_R", sleeve(8.6, 6.15, 16.3, 37.7), "main", "launcher", "#d8d8d8", note="keeps the drive pinion centred: pinion to right pivot bearing inner ring")
add("pinion_spacer_L", sleeve(8.6, 6.15, -37.7, -16.3), "main", "launcher", "#d8d8d8", note="pinion to left pivot bearing inner ring")
add("pg_spacer_L", sleeve(8.6, 6.15, -37.7, -P["GEAR_W"] / 2 - 18.3), "pg", "launcher", "#d8d8d8", note="flange hub to left cheek bearing inner ring")
add("pg_spacer_R", sleeve(8.6, 6.15, 7.3, 37.7), "pg", "launcher", "#d8d8d8", note="program gear to right cheek bearing inner ring")
add("shaft_spacer_L", sleeve(8.6, 6.15, -95.6, -85.3), "main", "reel", "#d8d8d8", note="reel main gear to left frame bearing inner ring")

# rail beam: matching M4 holes for the cheek bolts (through the wall strip, outside the shoe track)
for (uu, s_, ww) in CHEEK_BOLTS:                 # heat-set insert holes (5.6 x 6) in the wall strip
    parts["rail_beam"]["solid"] = parts["rail_beam"]["solid"] - cylY(2.8, min(s_ * 43.5, s_ * 48.5), max(s_ * 43.5, s_ * 48.5), uu, ww)
# main shaft stack: every piece is held between the coupler (clamped, right) and the reel gear (clamped, left)
add("shaft_spacer_R", sleeve(8.6, 6.15, 46.3, 95.7), "main", "launcher", "#d8d8d8", note="right pivot bearing to right frame bearing")
add("shaft_spacer_M", sleeve(8.6, 6.15, -68.7, -46.3), "main", "reel", "#d8d8d8", note="left pivot bearing to the reel gear hub")

# ================================================================= spring and cord envelopes (full stretch, for the audit)
for sg_, nm in ((1, "R"), (-1, "L")):
    add("spring_env_%s" % nm, cylX(P["SPRING_OD"] / 2, P["ANCHOR_U"] + 22, pul_u - 30, sg_ * P["SPRING_Y"], P["SPRING_W"], seg=32)
        + cylX(1.2, pul_u - 31, pul_u, sg_ * P["SPRING_Y"], P["SPRING_W"], seg=16),
        "rail", "hardware", "#c0c4c8", printed=False, note="extension spring + lower cord run, envelope at full stretch")
    add("cord_env_%s" % nm, cylX(1.2, P_D + P["SHOE_L"], pul_u, sg_ * P["SPRING_Y"], 85.0, seg=16),
        "rail", "hardware", "#e0d0a0", printed=False, note="2 mm cord, shoe to pulley, envelope over the stroke")

# ================================================================= muzzle horn: guides the dart up over the lip on retrieve
# a quarter-circle trough from the cradle floor at the muzzle, curving forward and down; it stays below the flight path
R_H = 70.0; W_FLOOR = DW - P["CR_RI"]                      # inner floor height at the lip (73.0)
wc = W_FLOOR - R_H
def horn_section(alpha):
    """U trough cross-section placed on the arc at angle alpha (0 = at the lip, horizontal)"""
    prof = unary_union([sbox(-45, -2.6, 45, 0), sbox(-45, -2.6, -42.4, 10), sbox(42.4, -2.6, 45, 10)])
    pts = []
    for (yy, nn) in prof.exterior.coords:
        r = R_H + nn                                       # nn up = away from the arc centre (the dart rides on top)
        pts.append((U_M + r * math.sin(alpha), yy, wc + r * math.cos(alpha)))
    return np.array(pts)
segs = []
N_H = 16
for i in range(N_H):
    a0, a1 = math.radians(80 * i / N_H), math.radians(80 * (i + 1) / N_H)    # 80 deg: past vertical, without 200 mm tall supports under rail piece C
    pts = np.vstack([horn_section(a0), horn_section(a1)])
    segs.append(Manifold.hull_points(pts.tolist()))
horn = union(segs)
# the side walls must not rise into the pod path above the cradle edge: trim everything above w 81 near the lip
horn = horn - box(U_M - 1, U_M + 30, -60, 60, 81.0, 200)
parts["rail_beam"]["solid"] = parts["rail_beam"]["solid"] + horn + box(U_M - 8, U_M + 1, -20, 20, W_FLOOR - 2.6 - 5, W_FLOOR - 2.6)

# ================================================================= rail beam joints: bolted flanges at the print splits
from shapely.geometry import Polygon as SPoly, MultiPolygon as SMulti
def rail_slice(sol, uc):
    cs = sol.rotate((0, -90, 0)).slice(uc)
    polys = []
    for ring in cs.to_polygons():
        pts = [(p[1], -p[0]) for p in ring]          # (Y', -X') = (y, w)
        if len(pts) >= 3:
            polys.append(SPoly(pts))
    # even-odd assemble
    out = None
    for pg_ in sorted(polys, key=lambda q: -abs(q.area)):
        pg_ = pg_.buffer(0)
        out = pg_ if out is None else out.symmetric_difference(pg_)
    return out
KEEP = unary_union([Point(0, DW).buffer(22.6, 64), sbox(-41.5, 80.8, 41.5, 89.6), sbox(-60, 92.0, 60, 200),
                    sbox(-8.5, 36, 8.5, 70.6), sbox(-5.2, 54, 5.2, 79), sbox(21, 20, 38, 40), sbox(-38, 20, -21, 40),
                    sbox(-38, 82, -21, 89.6), sbox(21, 82, 38, 89.6), sbox(-10.4, 51.0, 10.4, 57.0), sbox(-80, 40, -48.2, 100), sbox(-35.2, 64.5, -28.8, 82.5)])
SPLITS = [40.0, 265.0]
P["RAIL_SPLITS"] = SPLITS
for uc in SPLITS[1:]:
    sec = rail_slice(parts["rail_beam"]["solid"], uc)
    fl = sec.buffer(6, join_style=2).difference(KEEP).union(sec.difference(KEEP))
    fl = fl.buffer(-0.5).buffer(0.5)
    if fl.geom_type == "MultiPolygon":
        fl = SMulti([g for g in fl.geoms if g.area > 30])
    # bolt holes: well inside the flange, spread out
    inner = fl.buffer(-5.0)
    cands = []
    xs0, ys0, xs1, ys1 = inner.bounds
    for yy in np.arange(xs0, xs1, 2.0):
        for ww in np.arange(ys0, ys1, 2.0):
            if inner.contains(Point(yy, ww)):
                cands.append((yy, ww))
    holes = []
    if cands:
        holes.append(min(cands, key=lambda c: c[1]))
        while len(holes) < 8:
            c = max(cands, key=lambda c: min(math.hypot(c[0] - h[0], c[1] - h[1]) for h in holes))
            if min(math.hypot(c[0] - h[0], c[1] - h[1]) for h in holes) < 14: break
            holes.append(c)
    flange = ext_u(fl, uc - 6, uc + 6)
    for (yy, ww) in holes:
        flange = flange - cylX(2.2, uc - 7, uc + 7, yy, ww)
    rb = parts["rail_beam"]["solid"] + flange
    for (yy, ww) in holes:
        rb = rb - cylX(2.2, uc - 7, uc + 7, yy, ww)
    parts["rail_beam"]["solid"] = rb
    print("rail split at u %.0f: flange %.0f mm2, %d M4 bolts" % (uc, fl.area, len(holes)))
    P.setdefault("RAIL_BOLTS", []).append([uc, holes])

# ================================================================= merge frame towers into the side plates (fewer parts)
parts["frame_plate_R"]["solid"] = parts["frame_plate_R"]["solid"] + parts.pop("roller_post")["solid"] + \
    box(rx - 10, rx + 10, P["CAM_Y1"] + 9.0, FY + 1.0, rz - 10, rz + 10)
parts["frame_plate_R"]["note"] += "; carries the cam roller post"

# ================================================================= split the cheeks: each cheek prints flat, the tie bolts between them
TIE_CUT = box(115.0, 139.0, -37.9, 37.9, -1.0, 41.0)
rr = parts.pop("rail_rear")["solid"]
tie = rr ^ TIE_CUT
chk = rr - TIE_CUT
cL = chk ^ box(-300, 300, -300, -38.0, -300, 300); cR = chk ^ box(-300, 300, 38.0, 300, -300, 300)   # flat inner faces: they print face down
for uu in (120.0, 132.0):                         # 2 x M4 x 20 per side: through the cheek into a nut in an open trap in the tie
    for s_ in (1, -1):
        hole = cylY(2.2, min(s_ * 25.0, s_ * 47.0), max(s_ * 25.0, s_ * 47.0), uu, 17.5)
        cL = cL - hole; cR = cR - hole; tie = tie - hole
        tie = tie - box(uu - 3.6, uu + 3.6, min(s_ * 29.6, s_ * 33.0), max(s_ * 29.6, s_ * 33.0), 13.5, 23.0)
add("cheek_L", cL, "rail", "launcher", "#9aa4ae", note="left cheek (PPA-CF, printed flat): pivot + PG bearings, backstop boss, sear pocket, stops, cord arc slot, reel pad")
add("cheek_R", cR, "rail", "launcher", "#9aa4ae", note="right cheek (PPA-CF, printed flat): pivot + PG bearings, backstop boss")
add("cheek_tie", tie, "rail", "launcher", "#9aa4ae", note="tie between the cheeks under the springs; rail piece B screws onto its post")
print("cheeks: L %d body, R %d body, tie %d body" % (len(cL.decompose()), len(cR.decompose()), len(tie.decompose())))

# ================================================================= gear phases (2D, exact polygons)
def best_phase(fixed, moving_at, period, n=240):
    best = None
    for i in range(n):
        ph = period * i / n
        a = fixed.intersection(moving_at(ph)).area
        if best is None or a < best[0]:
            best = (a, ph)
    return best

# pinion vs program gear at theta 0, phi_ref
phi_ref = 100.0
pgw = affinity.translate(affinity.rotate(pg_poly, phi_ref, origin=(0, 0)), P["PG_U"], 0)
pin_at = lambda ph: affinity.rotate(pin_poly, -phi_ref * P["Z_PG"] / P["Z_PIN"] + ph, origin=(0, 0))
a_pin, P["PIN_PHASE"] = best_phase(pgw, pin_at, 360.0 / P["Z_PIN"])
# stub vs main reel gear at theta 0, alpha 0
stub_poly = involute_gear_2d(P["MR"], P["Z_STUB"])
stub_at = lambda ph: affinity.translate(affinity.rotate(stub_poly, ph, origin=(0, 0)), P["REEL_U"], DW)
a_stub, P["STUB_PHASE"] = best_phase(mg_poly, stub_at, 360.0 / P["Z_STUB"])
# rack vs pg over the stroke
rk_rest = rk
worst = 0
for phi, dd in RT[::4]:
    sh = -dd
    rr = affinity.translate(rk_rest, P_R + sh, 0)
    gg = affinity.translate(affinity.rotate(pg_poly, phi, origin=(0, 0)), P["PG_U"], 0)
    worst = max(worst, rr.intersection(gg).area)
print("phases: pinion %.2f deg (overlap %.3f mm2), stub %.2f deg (overlap %.3f mm2), rack-PG worst overlap %.3f mm2" % (P["PIN_PHASE"], a_pin, P["STUB_PHASE"], a_stub, worst))

# ================================================================= export
P["DECK_HOLES"] = [list(map(float, h)) for h in DECK_HOLES]
meta = {"P": P, "parts": []}
for name, d in parts.items():
    s = d["solid"]
    mesh = s.to_mesh()
    v = np.asarray(mesh.vert_properties)[:, :3]; f = np.asarray(mesh.tri_verts)
    tm = trimesh.Trimesh(v, f, process=False)
    if len(v) == 0:
        print("EMPTY PART", name); continue
    np.savez_compressed(f"{OUT}/{name}.npz", v=v.astype(np.float32), f=f.astype(np.int32))
    tm.export(f"{OUT}/{name}.stl")
    meta["parts"].append(dict(name=name, frame=d["frame"], group=d["group"], color=d["color"], printed=d["printed"],
                              note=d["note"], qty=d["qty"], volume_cm3=round(s.volume() / 1000, 2),
                              genus=s.genus(), nverts=len(v), bbox=[round(x, 1) for x in tm.bounds.flatten()],
                              watertight=bool(tm.is_watertight)))
json.dump(meta, open(f"{OUT}/gs_parts.json", "w"), indent=1)
print("parts", len(parts), "printed cm3 %.1f" % sum(p["volume_cm3"] for p in meta["parts"] if p["printed"]))
# extra metadata for kinematics
P["SEAR_PIV"] = list(SEAR_PIV); P["CORD_A"] = list(CORD_A); P["HOOK"] = list(HOOK); P["KEEL"] = list(KEEL); P["DART_L"] = 151.97
P["PH_TRIP"] = next(ph for ph in np.arange(PH_LIFT0, PH_LIFT1 + 4, 0.1) if theta_prog(ph) >= P["TH_RELEASE"])
P["CAM_POLY"] = [list(map(float, c)) for c in cam_poly.exterior.coords]
meta["P"] = P
json.dump(meta, open(f"{OUT}/gs_parts.json", "w"), indent=1)
