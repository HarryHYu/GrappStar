"""Motors: both N20 gearmotor types (models + mounts) and the car wiper motor interface (plate, coupler, envelope).

N20 numbers from Harry's calipers (kedge/CLAUDE.md section 2). Not measured, assumed (marked _ASSUMED below):
worm output shaft position inside the box and its D flat, straight gearbox length, face holes and face boss.
Wiper motor: NOT measured yet. Every WIPER_* default is a placeholder typical of a car wiper motor; pass the real
numbers once Harry calipers it.
"""
import math

from manifold3d import CrossSection

from . import fits, prim, tf

# right-angle worm N20 (measured)
N20W_BODY_D = 12.0            # motor body round dia
N20W_BODY_FLAT = 10.0         # across the body flats
N20W_BOX_W = 9.94             # worm box width, along the output shaft
N20W_BOX_H = 11.93            # worm box height
N20W_BOX_L = 12.0             # worm box length along the motor axis ("10 x 12")
N20W_LEN = 33.04              # box + motor, no shafts
N20W_SHAFT_TIPS = 29.97       # biaxial output, tip to tip
N20W_SHAFT_D = 3.0
N20W_SHAFT_FLAT_ASSUMED = 2.5     # D flat (across), datasheet typical
N20W_OUT_X_ASSUMED = 0.0          # output axis at the box centre along the motor axis (caliper it)
N20W_OUT_Y_ASSUMED = 0.0          # motor axis height relative to the output axis (caliper it)

# straight N20 (Bringsmart drill gearmotor, measured)
N20S_W = 12.0                 # 11.95 tall gearbox, 12 body dia
N20S_FLAT = 10.0
N20S_GB_H = 11.95
N20S_LEN = 24.27              # gearbox face to motor back
N20S_SHAFT_D = 2.84           # M3 threaded
N20S_SHAFT_L = 56.03          # past the face
N20S_GB_L_ASSUMED = 9.0       # gearbox length (datasheet typical, ratio dependent)
N20S_HOLE_SPACING_ASSUMED = 9.0   # two M1.6 face holes (datasheet), along the 12 mm side
N20S_HOLE_D_ASSUMED = 1.6
N20S_BOSS_D_ASSUMED = 4.0     # face bushing boss
N20S_BOSS_H_ASSUMED = 0.8
N20_TERM_L = 2.5              # terminal tabs behind the motor back

# wiper motor placeholders (NOT measured)
WIPER_DEFAULTS = {
    "shaft_d": 8.0,           # output shaft (double flat, 6 across flats) or 6 mm taper on some models
    "flat": 1.0,              # flat depth per side (None = round)
    "shaft_len": 25.0,        # above the boss top
    "thread_d": 6.0,          # M6 nut end
    "thread_len": 10.0,
    "boss_d": 20.0,           # gearbox output boss around the shaft (plate pilot)
    "boss_h": 5.0,
    "pcd": 60.0,              # three M6 mounting bolts
    "n_bolts": 3,
    "bolt_d": 6.0,
    "case_d": 95.0,           # round gear case below the mounting face
    "case_h": 45.0,
    "can_d": 55.0,            # motor can, axis along +X under the case
    "can_len": 110.0,
    "can_x0": 15.0,
    "can_y": -18.0,
}
M6_NUT_AF = 10.0
M6_NUT_H = 5.2
KEY20_W = 6.0                 # DIN 6885 key for a 20 mm shaft
KEY20_HUB_T2 = 2.8


def _dprofile_x(r, flat_half, x0, x1, y=0.0, z=0.0):
    """D-ended motor body along X: round radius r clipped to |z - z0| <= flat_half."""
    c = prim.cylX(r, x0, x1, y=y, z=z)
    return c ^ prim.box(x0 - 1, x1 + 1, y - r - 1, y + r + 1, z - flat_half, z + flat_half)


def _dprofile_z(r, flat_half, z0, z1, flats_axis="y"):
    c = prim.cyl(r, z1 - z0, z=z0)
    if flats_axis == "y":
        return c ^ prim.box(-r - 1, r + 1, -flat_half, flat_half, z0 - 1, z1 + 1)
    return c ^ prim.box(-flat_half, flat_half, -r - 1, r + 1, z0 - 1, z1 + 1)


def _dshaft(d, flat_across, z0, z1):
    """Round shaft along Z with one D flat (flat on +Y side at y = flat_across - d/2)."""
    c = prim.cyl(d / 2, z1 - z0, z=z0, seg=32)
    return c ^ prim.box(-d, d, -d, flat_across - d / 2, z0 - 1, z1 + 1)


def n20_worm(out_x=N20W_OUT_X_ASSUMED, out_y=N20W_OUT_Y_ASSUMED, shaft_flat=N20W_SHAFT_FLAT_ASSUMED):
    """Right-angle worm N20 (bought) model. Frame: output shaft axis = Z through the origin (biaxial, tips at
    z = +-14.985), box centred on z (width 9.94 along Z), box 12 long along X and 11.93 tall along Y; motor body
    along +X (D profile, flats along Z), motor axis at y = out_y. out_x = output axis position from the box centre."""
    bx0 = -N20W_BOX_L / 2 - out_x
    bx1 = bx0 + N20W_BOX_L
    box = prim.box(bx0, bx1, -N20W_BOX_H / 2 + out_y, N20W_BOX_H / 2 + out_y, -N20W_BOX_W / 2, N20W_BOX_W / 2)
    motor = _dprofile_x(N20W_BODY_D / 2, N20W_BODY_FLAT / 2, bx1 - 0.01, bx0 + N20W_LEN, y=out_y)
    tabs = prim.box(bx0 + N20W_LEN - 0.01, bx0 + N20W_LEN + N20_TERM_L, out_y - 3.5, out_y + 3.5, -0.4, 0.4)
    h = N20W_SHAFT_TIPS / 2
    shaft = _dshaft(N20W_SHAFT_D, shaft_flat, -h, h)
    return prim.union([box, motor, tabs, shaft])


def _worm_envelope(clear, out_x, out_y, shaft_clear, shaft_ext):
    bx0 = -N20W_BOX_L / 2 - out_x
    bx1 = bx0 + N20W_BOX_L
    c = clear
    box = prim.box(bx0 - c, bx1 + c, -N20W_BOX_H / 2 + out_y - c, N20W_BOX_H / 2 + out_y + c,
                   -N20W_BOX_W / 2 - c, N20W_BOX_W / 2 + c)
    motor = _dprofile_x(N20W_BODY_D / 2 + c, N20W_BODY_FLAT / 2 + c, bx1, bx0 + N20W_LEN + c, y=out_y)
    wires = prim.box(bx0 + N20W_LEN, bx0 + N20W_LEN + N20_TERM_L + 6.0, out_y - 4.5, out_y + 4.5, -2.5, 2.5)
    h = N20W_SHAFT_TIPS / 2 + shaft_ext
    shaft = prim.cyl((N20W_SHAFT_D + shaft_clear) / 2, 2 * h, z=-h, seg=32)
    return prim.union([box, motor, wires, shaft]), (bx0, bx0 + N20W_LEN)


def n20_worm_mount(clear=0.3, wall=2.0, shaft_clear=1.0, out_x=N20W_OUT_X_ASSUMED, out_y=N20W_OUT_Y_ASSUMED):
    """Mount for the worm N20, same frame as n20_worm(). Returns dict:
    cutter: envelope + clear, output shaft clearance holes both sides (d 3 + shaft_clear, to the shaft tips + 5),
            wire exit behind the motor; subtract from a host;
    cradle: printed lower half (y <= out_y), motor drops in from +Y, four M3 tap holes;
    clamp:  printed upper half (y >= out_y), four M3 clearance holes, screws from +Y into the cradle;
    info:   outer box and screw positions."""
    cut, (x0, x1) = _worm_envelope(clear, out_x, out_y, shaft_clear, 5.0)
    zin = N20W_BOX_W / 2 + clear
    zs = zin + wall + fits.M3_CLEAR_D / 2 + 0.6       # screw rows outside the box faces
    zo = zs + fits.M3_CLEAR_D / 2 + wall
    yo = N20W_BOX_H / 2 + clear + wall
    ox0, ox1 = x0 - clear - wall, x1 + clear + wall
    sx = (x0 + 1.0, x1 - 3.0)       # clear of the output shaft hole at x = -out_x
    cradle = prim.box(ox0, ox1, out_y - yo, out_y, -zo, zo) - cut
    clamp = prim.box(ox0, ox1, out_y, out_y + yo, -zo, zo) - cut
    holes_tap, holes_clr = [], []
    for x in sx:
        for z in (-zs, zs):
            holes_tap.append(prim.place(prim.cyl(fits.M3_TAP_D / 2, yo - 1.0, seg=24),
                                        tf.T(x, out_y + 0.01, z) @ tf.Rx(90)))
            holes_clr.append(prim.place(prim.cyl(fits.M3_CLEAR_D / 2, yo + 2, seg=24),
                                        tf.T(x, out_y - 1, z) @ tf.Rx(-90)))
    cradle = prim.diff(cradle, holes_tap)
    clamp = prim.diff(clamp, holes_clr)
    info = {"box": ((ox0, out_y - yo, -zo), (ox1, out_y + yo, zo)), "screws": [(x, z) for x in sx for z in (-zs, zs)],
            "screw": "4 x M3x(%.0f) SHCS, self tap into the cradle" % (yo + 4), "split_y": out_y,
            "assumed": "output shaft position in the box and D flat not measured"}
    return {"cutter": cut, "cradle": cradle, "clamp": clamp, "info": info}


def n20_straight(gb_len=N20S_GB_L_ASSUMED):
    """Straight N20 gearmotor (Bringsmart drill type, bought) model. Frame: output shaft along +Z on the axis,
    gearbox face at z = 0, gearbox 11.95 along X x 10 along Y, motor body D profile (flats along Y) back to
    z = -24.27, terminals behind; shaft d 2.84 (M3 thread, plain here) to z = 56.03; face boss and two M1.6 face
    holes at x = +-4.5 (assumed)."""
    gb = prim.box(-N20S_GB_H / 2, N20S_GB_H / 2, -N20S_FLAT / 2, N20S_FLAT / 2, -gb_len, 0)
    motor = _dprofile_z(N20S_W / 2, N20S_FLAT / 2, -N20S_LEN, -gb_len + 0.01)
    tabs = prim.box(-0.4, 0.4, -3.5, 3.5, -N20S_LEN - N20_TERM_L, -N20S_LEN + 0.01)
    boss = prim.cyl(N20S_BOSS_D_ASSUMED / 2, N20S_BOSS_H_ASSUMED, z=-0.01, seg=32)
    shaft = prim.cyl(N20S_SHAFT_D / 2, N20S_SHAFT_L, seg=32)
    holes = [prim.cyl(N20S_HOLE_D_ASSUMED * 0.8 / 2, 3.0, x=s * N20S_HOLE_SPACING_ASSUMED / 2, z=-3.0 + 0.01,
                      seg=16) for s in (-1, 1)]
    return prim.diff(prim.union([gb, motor, tabs, boss, shaft]), holes)


def n20_mount(plate_t=3.0, clear=0.3, wall=2.0, screw_d=N20S_HOLE_D_ASSUMED, gb_len=N20S_GB_L_ASSUMED,
              cradle=True):
    """Bracket for the straight N20, same frame as n20_straight(). Face plate z 0..plate_t with the shaft/boss
    hole (boss d + 0.4), two face screw holes (screw_d + 0.2, assumed M1.6 at 9 mm, counterbored on top) and two
    M3 clearance holes to the host at x = +-(half width - 3); cradle trough under the body (y < 0) back to the
    motor end. Returns dict bracket (one printed shell), cutter (host pocket: envelope + clear, shaft and boss
    hole, face screw holes), info."""
    w = N20S_GB_H / 2 + clear + wall
    pw = w + fits.M3_CLEAR_D + 2.5
    ph = N20S_FLAT / 2 + clear + wall + 2.0
    plate = prim.box(-pw, pw, -ph, ph, 0, plate_t)
    hx = N20S_HOLE_SPACING_ASSUMED / 2
    cut_plate = [prim.cyl((N20S_BOSS_D_ASSUMED + 0.4) / 2, plate_t + 2, z=-1, seg=32)]
    for s in (-1, 1):
        cut_plate.append(prim.cyl((screw_d + 0.2) / 2, plate_t + 2, x=s * hx, z=-1, seg=16))
        cut_plate.append(prim.cyl((2 * screw_d + 0.4) / 2, 1.0 + 1, x=s * hx, z=plate_t - 1.0, seg=16))
        cut_plate.append(prim.cyl(fits.M3_CLEAR_D / 2, plate_t + 2, x=s * (pw - 3.0), z=-1, seg=24))
    env = prim.union([
        prim.box(-N20S_GB_H / 2 - clear, N20S_GB_H / 2 + clear, -N20S_FLAT / 2 - clear, N20S_FLAT / 2 + clear,
                 -gb_len - 0.01, 0.0),
        _dprofile_z(N20S_W / 2 + clear, N20S_FLAT / 2 + clear, -N20S_LEN - clear, -gb_len + 0.01),
        prim.box(-4.5, 4.5, -4.5, 4.5, -N20S_LEN - N20_TERM_L - 6, -N20S_LEN),
    ])
    cutter = prim.union([env, prim.cyl((N20S_BOSS_D_ASSUMED + 0.4) / 2, plate_t + 2, z=-0.5, seg=32),
                         prim.cyl((N20S_SHAFT_D + 0.6) / 2, N20S_SHAFT_L + 1, seg=24)] +
                        [prim.cyl((screw_d + 0.2) / 2, plate_t + 3, x=s * hx, z=-1, seg=16) for s in (-1, 1)])
    parts = [plate - prim.union(cut_plate)]
    if cradle:
        y1 = -N20S_FLAT / 2 - clear - wall
        parts.append(prim.box(-w, w, y1, 0.0, -N20S_LEN - clear - wall, 0.01) - cutter)
    bracket = prim.union(parts)
    info = {"plate": (2 * pw, 2 * ph, plate_t), "face_screws": f"2 x M{screw_d:g}x{plate_t + 2:.0f} (assumed)",
            "host_screws": [(s * (pw - 3.0), 0.0) for s in (-1, 1)],
            "assumed": "face holes M1.6 at 9 mm, boss d 4, gearbox length 9 from the N20 datasheet"}
    return {"bracket": bracket, "cutter": cutter, "info": info}


# ---------------------------------------------------------------- wiper motor

def _wp(params):
    p = dict(WIPER_DEFAULTS)
    if params:
        p.update(params)
    return p


def _hole_shape(x, y, d, slot, slot_deg):
    """2D hole (CrossSection): circle d at (x, y), stretched radially by slot (mm) and along the arc by slot_deg."""
    r = math.hypot(x, y)
    a0 = math.atan2(y, x) if r > 1e-9 else 0.0
    na = max(1, int(abs(slot_deg) / 4) + 1) if slot_deg else 1
    angs = [a0 + math.radians(slot_deg) * (i / na - 0.5) for i in range(na + 1)] if slot_deg else [a0]
    rs = (r - slot / 2, r + slot / 2) if slot else (r,)
    pts = [(rr * math.cos(a), rr * math.sin(a)) for a in angs for rr in rs]
    if r < 1e-9 and slot:
        pts = [(-slot / 2, 0.0), (slot / 2, 0.0)]
    circ = [prim.circle_cs(d / 2, px, py, seg=32) for px, py in pts]
    if len(circ) == 1:
        return circ[0]
    out = None
    for i in range(len(circ) - 1):
        h = CrossSection.batch_hull([circ[i], circ[i + 1]])
        out = h if out is None else out + h
    if len(rs) == 2 and len(angs) > 1:   # fill between the two arcs
        for i in range(len(angs) - 1):
            out = out + CrossSection.batch_hull([circ[2 * i], circ[2 * i + 1], circ[2 * i + 2], circ[2 * i + 3]])
    return out


def wiper_plate(holes=None, pcd=None, n=None, bolt_d=None, boss_d=None, size=(100.0, 100.0), t=6.0,
                slot=0.0, slot_deg=0.0, phase_deg=90.0, frame_holes=None, frame_hole_d=5.5, corner_r=6.0,
                fit=fits.HOLE_CLEAR):
    """Adapter plate between the wiper motor gearbox face and the frame. Frame: plate z 0..t, the motor's mounting
    face against z = 0 (motor below), output axis = +Z through the origin. holes: list of (x, y, d) bolt holes;
    else pcd + n bolts of d bolt_d starting at phase_deg. Every bolt hole gets d + fit and is stretched radially by
    slot (mm) and along the arc by slot_deg (adjustment). Boss pilot hole boss_d + fit on the axis. frame_holes:
    list of (x, y) for frame bolts (default four corners, inset 8), d frame_hole_d. Defaults: WIPER_DEFAULTS
    (placeholders, wiper not measured)."""
    p = WIPER_DEFAULTS
    pcd = p["pcd"] if pcd is None else pcd
    n = p["n_bolts"] if n is None else n
    bolt_d = p["bolt_d"] if bolt_d is None else bolt_d
    boss_d = p["boss_d"] if boss_d is None else boss_d
    if holes is None:
        holes = [(pcd / 2 * math.cos(math.radians(phase_deg + 360.0 * i / n)),
                  pcd / 2 * math.sin(math.radians(phase_deg + 360.0 * i / n)), bolt_d) for i in range(n)]
    sx, sy = size
    if frame_holes is None:
        frame_holes = [(sx2 * (sx / 2 - 8), sy2 * (sy / 2 - 8)) for sx2 in (-1, 1) for sy2 in (-1, 1)]
    plate = prim.rbox(sx, sy, t, corner_r)
    cs = prim.circle_cs((boss_d + fit) / 2)
    for (x, y, d) in holes:
        cs = cs + _hole_shape(x, y, d + fit, slot, slot_deg)
    for (x, y) in frame_holes:
        cs = cs + prim.circle_cs((frame_hole_d) / 2, x, y, seg=32)
    return plate - prim.extrude(cs, t + 2, z=-1)


def wiper_coupler(shaft_d=None, flat=None, thread="M6", length=None, out="key", out_d=20.0, out_len=20.0,
                  in_len=12.0, web=4.0, od=None, flats=2, key_w=KEY20_W, key_t2=KEY20_HUB_T2, out_flat=None,
                  taper_deg=None, grub=None, fit=fits.HOLE_SLIDE, out_fit=0.10):
    """Coupler from the wiper output shaft to the 20 mm main shaft. Frame: axis +Z, wiper shaft enters at z = 0.
    z 0..in_len: input bore shaft_d + fit with `flats` flats of depth `flat` (None = round), or a taper of included
    angle taper_deg narrowing upward from shaft_d. Web (in_len..in_len + web) with a thread clearance hole; the wiper
    M6 nut tightens on the web from above (nut chamber = the output bore, a socket goes down it before the main
    shaft). thread=None: no web hole beyond the shaft, an M3 grub screw + nut trap bears on the flat instead.
    Output bore out_d + out_fit over out_len above the nut chamber: out="key" (keyway key_w + 2 x KEY_CLEAR, hub
    depth key_t2) or out="d" (D bore, flat out_flat across, default 0.85 out_d). length overrides out_len (total)."""
    p = WIPER_DEFAULTS
    shaft_d = p["shaft_d"] if shaft_d is None else shaft_d
    flat = p["flat"] if flat is None and taper_deg is None else flat
    thread_d = float(thread.upper().lstrip("M")) if thread else None
    grub = (thread is None) if grub is None else grub
    chamber = (M6_NUT_H if thread_d else 0.0) + (thread_d or 0) * 0.5 + 2.0 if thread_d else 0.0
    z_web = in_len
    z_ch = in_len + web
    z_out = z_ch + chamber
    if length is not None:
        out_len = length - z_out
        if out_len < 5.0:
            raise ValueError("coupler length too short")
    L = z_out + out_len
    od = max(out_d + 12.0, shaft_d + 14.0) if od is None else od
    body = prim.cyl(od / 2, L)
    body = body - prim.chamfer_ring(od / 2, L, 1.0, top=True) - prim.chamfer_ring(od / 2, L, 1.0, top=False)
    cuts = []
    rb = (shaft_d + fit) / 2
    if taper_deg:
        tn = math.tan(math.radians(taper_deg / 2))
        cuts.append(prim.cyl(rb + tn, in_len + 1, z=-1, r2=rb - in_len * tn))
    else:
        bore = prim.cyl(rb, in_len + 1, z=-1, seg=48)
        if flat:
            yf = shaft_d / 2 - flat + fit / 2
            keep = prim.box(-od, od, -yf if flats >= 2 else -od, yf, -2, in_len + 2)
            bore = bore ^ keep
        cuts.append(bore)
    if thread_d:
        cuts.append(prim.cyl((thread_d + fits.HOLE_CLEAR + 0.25) / 2, web + 2, z=z_web - 1, seg=32))
    if grub:
        zg = in_len / 2
        yf = (shaft_d / 2 - (flat or 0) + fit / 2) if flat else rb
        cuts.append(prim.place(prim.cyl(fits.M3_CLEAR_D / 2, od, seg=24), tf.T(0, yf - 0.5, zg) @ tf.Rx(-90)))
        nut = prim.extrude(CrossSection.circle(fits.M3_NUT_AF / math.sqrt(3), 6), fits.M3_NUT_H)
        yn = yf + 2.0
        cuts.append(prim.place(nut, tf.T(0, yn, zg) @ tf.Rx(-90)) +
                    prim.box(-fits.M3_NUT_AF / 2, fits.M3_NUT_AF / 2, yn, yn + fits.M3_NUT_H, zg, L + 1))
    ro = (out_d + out_fit) / 2
    top = prim.cyl(ro, L - z_ch + 1, z=z_ch, seg=96)
    if out == "key":
        kw = key_w + 2 * fits.KEY_CLEAR
        top = top + prim.box(-kw / 2, kw / 2, 0, out_d / 2 + key_t2 + 0.1, z_out, L + 1)
    elif out == "d":
        f = 0.85 * out_d if out_flat is None else out_flat
        top = prim.cyl(ro, chamber + 0.01, z=z_ch, seg=96) + \
            (prim.cyl(ro, out_len + 1, z=z_out, seg=96) ^ prim.box(-od, od, -od, f - out_d / 2 + out_fit / 2,
                                                                   z_out - 1, L + 2))
    else:
        raise ValueError("out must be 'key' or 'd'")
    cuts.append(top)
    return body - prim.union(cuts)


def wiper_coupler_info(shaft_d=None, thread="M6", in_len=12.0, web=4.0, out_len=20.0, length=None):
    """z levels of wiper_coupler(): web, nut chamber, output bore start, total length."""
    thread_d = float(thread.upper().lstrip("M")) if thread else None
    chamber = (M6_NUT_H + thread_d * 0.5 + 2.0) if thread_d else 0.0
    z_out = in_len + web + chamber
    L = length if length is not None else z_out + out_len
    return {"z_web": in_len, "z_chamber": in_len + web, "z_out": z_out, "length": L,
            "nut_end": "wiper M6 nut on the web, socket through the output bore" if thread_d else "M3 grub on flat"}


def wiper_coupler_z(params=None, in_len=12.0, gap=0.5):
    """z (envelope / plate frame) of the coupler's bottom face so its input bore ends gap above the start of the
    wiper thread (the flat part of the shaft stays inside the bore): place with tf.T(0, 0, wiper_coupler_z(...))."""
    p = _wp(params)
    return p["boss_h"] + p["shaft_len"] - p["thread_len"] - in_len + gap


def wiper_motor_envelope(params=None):
    """Simple wiper motor collision model (bought). Frame: mounting face at z = 0, output axis = +Z through the
    origin; gear case below (z < 0), boss and shaft above (boss to boss_h, shaft with flats to boss_h + shaft_len,
    last thread_len at thread_d); motor can along +X under the case. params overrides WIPER_DEFAULTS
    (placeholders, NOT measured)."""
    p = _wp(params)
    case = prim.cyl(p["case_d"] / 2, p["case_h"], z=-p["case_h"])
    can = prim.cylX(p["can_d"] / 2, p["can_x0"], p["can_x0"] + p["can_len"], y=p["can_y"], z=-p["case_h"] / 2)
    can = can ^ prim.box(-1e3, 1e3, -1e3, 1e3, -p["case_h"] - p["can_d"], -0.5)
    boss = prim.cyl(p["boss_d"] / 2, p["boss_h"], z=-0.01)
    z0 = p["boss_h"] - 0.01
    zt = p["boss_h"] + p["shaft_len"] - p["thread_len"]
    shaft = prim.cyl(p["shaft_d"] / 2, zt - z0, z=z0, seg=48)
    if p.get("flat"):
        yf = p["shaft_d"] / 2 - p["flat"]
        shaft = shaft ^ prim.box(-50, 50, -yf, yf, z0 - 1, zt + 1)
    thread = prim.cyl(p["thread_d"] / 2, p["thread_len"] + 0.01, z=zt - 0.01, seg=32)
    return prim.union([case, can, boss, shaft, thread])
