"""Selector drum, dog clutch and main shaft: numbers, timing and kinematics (no geometry).

Owner: worker cad-drum. Pure Python, no CAD imports, so firmware, physics and tests can use it.

Conventions
- Drum angle theta (deg): 0 = LAUNCH, about 40 = NEUTRAL, about 80 = REEL, then the wrap sector, then WRAPPED.
  Positive theta = N20 forward (DRV8833 #3 IN1 high) = toward REEL / WRAP.
- Collar position x (mm) along the main shaft axis: 0 = NEUTRAL (centred), -stroke = LAUNCH gear engaged,
  +stroke = REEL gear engaged.
- Brake track lift (mm): radial lift of the track-2 cam under the cad-reel lever follower; 0 = light.
- Wrap: psi = wrap pinion angle (deg), ring = wrap ring angle (deg); wraps = ring / 360.

All values come from DEFAULTS, overridden key by key by params.DRUM (a dict) when the coordinator sets it.
"""
import math

DEFAULTS = dict(
    # ---------------------------------------------------------------- drum angles (deg)
    stop_lo=-10.0,          # groove end (hard stop) below LAUNCH
    launch=0.0,
    neutral=40.0,
    reel=87.0,
    wrapped=338.0,
    stop_hi=346.0,          # groove end (hard stop) past WRAPPED. stop_hi - stop_lo < 360 so the code disc is unique
    tr_ln=(6.0, 34.0),      # groove 1 transition LAUNCH -> NEUTRAL
    tr_nr=(46.0, 74.0),     # groove 1 transition NEUTRAL -> REEL
    # ---------------------------------------------------------------- dog clutch
    stroke=7.0,             # collar travel from NEUTRAL to either gear (mm) = neutral gap + engagement
    dog_h=5.0,              # dog height on each face (mm)
    dog_gap=2.5,            # tip to tip gap at NEUTRAL (mm)
    dog_n=4,                # dogs per face
    dog_deg=30.0,           # angular width of a dog at its root (deg); gap = 360/n - dog_deg
    dog_backcut=4.0,        # drive face back-cut (deg): torque pulls the dogs together
    dog_roof=1.2,           # tip chamfer depth (mm) on each side of the crest: tip to tip contacts slide off
    dog_r_in=18.0, dog_r_out=26.0,   # dog ring radii (mm)
    fork_k=1.6,             # fork flexure rate (N/mm): compliant when the dogs land tip to tip
    fork_free=6.0,          # flexure travel before its hard stop (mm), > dog_h so a blocked shift never stalls
    # ---------------------------------------------------------------- track 2 (storage reel slip clutch preload)
    brake_lift=4.0,         # full radial lift (mm) = FIRM
    brake_mid=0.5,          # fraction of full lift held while wrapping (wrapping pulls line off the reel)
    brake_ramps=((46.0, 98.0, 0.0, 0.5), (326.0, 337.0, 0.5, 1.0)),  # (theta0, theta1, frac0, frac1)
    # ---------------------------------------------------------------- wrap drive (segment gear with locking arc)
    seg_m=1.25, seg_z=64,   # drum segment gear: module, tooth count of the full circle it is cut from
    seg_teeth=40,           # teeth actually present (the toothed arc)
    seg_start=100.0,        # drum angle where the wrap pinion starts to turn
    pin_z=10, pin_x=0.4,    # wrap pinion (locked at home by the locking arc), profile shift
    idl_m=1.5, idl_z=48,    # wrap idler output gear on the pinion shaft (meshes the cad-reel wrap ring gear)
    ring_z=48,              # cad-reel wrap ring gear tooth count (wraps = turns of pinion * idl_z / ring_z)
    # ---------------------------------------------------------------- N20 drive
    n20_z=10, n20_x=0.3, drum_gear_z=90, drum_gear_m=1.0, drum_gear_x=-0.3,
    n20_rpm_12v=27.0, n20_v=9.0,        # DRV8833 fed by an MP1584 buck set to 9 V
    n20_stall_nm_9v=0.20,               # estimate, see stall_estimate()
    n20_work_frac=0.35,                 # keep the working load under this fraction of stall
    # ---------------------------------------------------------------- switch code (Gray code, bit = contact closed)
    # bit arcs in drum angle; each bit is closed inside any of its arcs
    code_arcs={
        "b0": ((-10.0, 36.0), (76.0, 327.0)),
        "b1": ((4.0, 98.0),),
        "b2": ((44.0, 347.0),),
    },
    sw_tol=1.5,             # expected edge tolerance (deg) of a printed contact on a bump ramp
)

# code (b2 b1 b0 as an int) -> sector name. 0 = all open = wire off or no bump: never valid.
CODE_TABLE = {
    0b001: "LAUNCH",
    0b011: "LN",            # between LAUNCH and NEUTRAL
    0b010: "NEUTRAL",
    0b110: "NR",            # between NEUTRAL and REEL
    0b111: "REEL",
    0b101: "WRAPPING",
    0b100: "WRAPPED",
    0b000: "INVALID",
}


def cfg(P=None):
    """DEFAULTS overridden by params.DRUM (dict) if present."""
    d = dict(DEFAULTS)
    over = getattr(P, "DRUM", None) if P is not None else None
    if isinstance(over, dict):
        d.update({k: v for k, v in over.items() if k in d})
    return d


# ------------------------------------------------------------------------------------------------ profiles
def _harm(u):
    """Harmonic rise 0..1 for u in 0..1 (zero slope at both ends)."""
    u = min(1.0, max(0.0, u))
    return 0.5 - 0.5 * math.cos(math.pi * u)


def collar_x(theta, c=None):
    """Collar position (mm) commanded by groove 1 at drum angle theta."""
    c = c or DEFAULTS
    s = c["stroke"]
    a0, a1 = c["tr_ln"]
    b0, b1 = c["tr_nr"]
    if theta <= a0:
        return -s
    if theta <= a1:
        return -s + s * _harm((theta - a0) / (a1 - a0))
    if theta <= b0:
        return 0.0
    if theta <= b1:
        return s * _harm((theta - b0) / (b1 - b0))
    return s


def collar_slope(theta, c=None, h=0.01):
    """dx/dtheta in mm per rad."""
    return (collar_x(theta + h, c) - collar_x(theta - h, c)) / math.radians(2 * h)


def brake_frac(theta, c=None):
    """Track 2 lift fraction 0 (light) .. 1 (firm)."""
    c = c or DEFAULTS
    f = 0.0
    for t0, t1, f0, f1 in c["brake_ramps"]:
        if theta >= t1:
            f = f1
        elif theta > t0:
            return f0 + (f1 - f0) * _harm((theta - t0) / (t1 - t0))
    return f


def brake_lift(theta, c=None):
    c = c or DEFAULTS
    return c["brake_lift"] * brake_frac(theta, c)


def seg_span(c=None):
    c = c or DEFAULTS
    return 360.0 * c["seg_teeth"] / c["seg_z"]


def wrap_turns_pinion(c=None):
    c = c or DEFAULTS
    return c["seg_teeth"] / c["pin_z"]


def wraps_total(c=None):
    c = c or DEFAULTS
    return wrap_turns_pinion(c) * c["idl_z"] / c["ring_z"]


def pinion_deg(theta, c=None):
    """Wrap pinion angle (deg, + = same sense as a full mesh would give for drum +), 0 at home.
    Locked (0) before seg_start, linear across the toothed arc, locked at full turns after it."""
    c = c or DEFAULTS
    t0 = c["seg_start"]
    t1 = t0 + seg_span(c)
    tt = min(max(theta, t0), t1)
    return (tt - t0) * c["seg_z"] / c["pin_z"]


def ring_deg(theta, c=None):
    c = c or DEFAULTS
    return pinion_deg(theta, c) * c["idl_z"] / c["ring_z"]


def wraps(theta, c=None):
    return ring_deg(theta, c) / 360.0


def code(theta, c=None):
    """Switch code (int, b2 b1 b0) at drum angle theta (ideal edges)."""
    c = c or DEFAULTS
    v = 0
    for i, name in enumerate(("b0", "b1", "b2")):
        if any(a <= theta < b for a, b in c["code_arcs"][name]):
            v |= 1 << i
    return v


def sector(theta, c=None):
    return CODE_TABLE[code(theta, c)]


def edges(c=None):
    """Sorted list of (theta, code_below, code_above) where the code changes inside the travel."""
    c = c or DEFAULTS
    pts = sorted({a for arcs in c["code_arcs"].values() for ab in arcs for a in ab
                  if c["stop_lo"] < a < c["stop_hi"]})
    return [(t, code(t - 1e-6, c), code(t + 1e-6, c)) for t in pts]


def drum_pos(theta, c=None):
    """Telemetry drum_pos name (contract C1) at an exact angle."""
    s = sector(theta, c)
    return {"LAUNCH": "LAUNCH", "NEUTRAL": "NEUTRAL", "REEL": "REEL", "WRAPPED": "WRAPPED"}.get(s, "BETWEEN")


def mode(theta, c=None):
    """Mechanical mode: which gear the collar engages."""
    c = c or DEFAULTS
    x = collar_x(theta, c)
    e = c["dog_gap"]                      # dogs start to overlap once the collar has moved past the gap
    if x <= -e:
        return "LAUNCH"
    if x >= e:
        return "WRAPPED" if wraps(theta, c) > 1e-6 else "REEL"
    return "NEUTRAL"


# ------------------------------------------------------------------------------------------------ N20 and torque
def stall_estimate(v=9.0):
    """N20 worm gearmotor output stall torque estimate (N m) at supply v.

    From the bench data (kedge/CLAUDE.md section 2): stall 0.18 A, free 0.025 A at 8 V, 27 rpm at 12 V. Assume the
    usual 12 V N20 motor (about 15,000 rpm free, so Ke = 12 V / 1571 rad/s = 7.6 mV s), gear ratio 15000/27 = 556,
    efficiency of a self-locking worm stage plus spur stages 0.30 (a self-locking worm is under 0.5).
    Motor stall torque scales with V: Ke * (I_stall - I_free) * v / 8.
    """
    ke = 12.0 / (15000.0 * 2 * math.pi / 60.0)
    t_motor = ke * (0.18 - 0.025) * v / 8.0
    return t_motor * (15000.0 / 27.0) * 0.30


def ratio(c=None):
    c = c or DEFAULTS
    return c["drum_gear_z"] / c["n20_z"]


def drum_dps(c=None, load_frac=0.2):
    """Drum speed (deg/s) at supply n20_v with a light load (speed drops linearly with load)."""
    c = c or DEFAULTS
    rpm = c["n20_rpm_12v"] * c["n20_v"] / 12.0 * (1.0 - load_frac)
    return rpm * 6.0 / ratio(c)


def loads(c=None):
    """Drum torque budget (N m at the drum). Returns dict of named peak loads and the N20 capacity."""
    c = c or DEFAULTS
    eta_groove = 0.7          # follower pin sliding in a printed groove
    eta_gear = 0.9            # one printed spur mesh
    # fork: flexure force at full block (fork_free) plus 3 N rail friction; groove slope peak
    f_fork = c["fork_k"] * c["fork_free"] + 3.0
    slope = max(abs(collar_slope(t, c)) for t in range(int(c["stop_lo"]), int(c["stop_hi"])))
    t_fork = f_fork * slope / 1000.0 / eta_groove
    # disengaging under a residual train torque (0.5 N m at the dog radius, back-cut 4 deg, mu 0.3)
    r_dog = 0.5 * (c["dog_r_in"] + c["dog_r_out"]) / 1000.0
    f_tan = 0.5 / r_dog
    f_dis = f_tan * (0.3 + math.tan(math.radians(c["dog_backcut"]))) + 3.0
    t_dis = f_dis * slope / 1000.0 / eta_groove
    # track 2: lever follower force (cad-reel to confirm), 25 N at full lift; cam slope peak
    f_brk = 25.0
    bslope = max(abs((brake_lift(t + 0.01, c) - brake_lift(t - 0.01, c)) / math.radians(0.02))
                 for t in range(int(c["stop_lo"]), int(c["stop_hi"])))
    t_brk = f_brk * bslope / 1000.0 / eta_groove
    # wrap: ring torque = line tension from the reel brake at mid lift (1.0 N) at the wrap radius (36 mm) plus
    # 0.04 N m friction; geared up by the wrap train (ring speed / drum speed)
    up = (c["seg_z"] / c["pin_z"]) * (c["idl_z"] / c["ring_z"])
    t_ring = 1.0 * 0.036 + 0.04
    t_wrap = t_ring * up / (eta_gear ** 2)
    cap_stall = c["n20_stall_nm_9v"] * ratio(c) * eta_gear
    # combined load along the travel (the loads that act at the same drum angle add up)
    t0 = c["seg_start"]
    t1 = t0 + seg_span(c)
    prof = []
    th = c["stop_lo"]
    while th <= c["stop_hi"]:
        sl = abs(collar_slope(th, c)) / 1000.0
        bs = abs((brake_lift(th + 0.01, c) - brake_lift(th - 0.01, c)) / math.radians(0.02)) / 1000.0
        tq = max(f_fork, f_dis) * sl / eta_groove + f_brk * bs / eta_groove + (t_wrap if t0 <= th <= t1 else 0.0)
        prof.append((th, tq))
        th += 0.5
    worst = max(prof, key=lambda p: p[1])
    return dict(
        fork_shift=t_fork, fork_disengage=t_dis, brake_ramp=t_brk, wrap=t_wrap,
        worst=worst[1], worst_at=worst[0],
        cap_stall=cap_stall, cap_work=c["n20_work_frac"] * cap_stall,
        f_fork=f_fork, f_disengage=f_dis, groove_slope_mm_per_rad=slope, wrap_gear_up=up,
    )


def timing(c=None):
    """Firmware timing table: nominal seconds for each move at drum_dps()."""
    c = c or DEFAULTS
    v = drum_dps(c)
    return dict(
        dps=v,
        launch_to_neutral=(c["neutral"] - c["launch"]) / v,
        neutral_to_reel=(c["reel"] - c["neutral"]) / v,
        launch_to_reel=(c["reel"] - c["launch"]) / v,
        wrap=(c["wrapped"] - c["reel"]) / v,
        full_travel=(c["stop_hi"] - c["stop_lo"]) / v,
    )


def stop_plan(c=None):
    """For each target and direction: the code edge to watch and the extra angle to run after it."""
    c = c or DEFAULTS
    es = edges(c)
    plans = []
    for name in ("launch", "neutral", "reel", "wrapped"):
        tgt = c[name]
        below = [e for e in es if e[0] < tgt]
        above = [e for e in es if e[0] > tgt]
        if below:
            e = below[-1]
            plans.append(dict(target=name.upper(), dir=+1, edge=e[0], code_from=e[1], code_to=e[2],
                              extra_deg=tgt - e[0]))
        if above:
            e = above[0]
            plans.append(dict(target=name.upper(), dir=-1, edge=e[0], code_from=e[2], code_to=e[1],
                              extra_deg=e[0] - tgt))
    return plans


def state(theta, c=None):
    """Everything the assembly needs at drum angle theta."""
    c = c or DEFAULTS
    return dict(theta=theta, collar_x=collar_x(theta, c), brake_lift=brake_lift(theta, c),
                pinion_deg=pinion_deg(theta, c), ring_deg=ring_deg(theta, c), wraps=wraps(theta, c),
                code=code(theta, c), sector=sector(theta, c), mode=mode(theta, c))


if __name__ == "__main__":
    c = DEFAULTS
    print("stall estimate at 9 V: %.3f N m" % stall_estimate(9.0))
    print("ratio", ratio(c), "drum deg/s %.1f" % drum_dps(c))
    print("segment span %.1f deg, pinion turns %.2f, wraps %.2f" % (seg_span(c), wrap_turns_pinion(c), wraps_total(c)))
    for k, v in loads(c).items():
        print("  %-24s %.3f" % (k, v))
    for k, v in timing(c).items():
        print("  %-24s %.2f" % (k, v))
    for e in edges(c):
        print("edge %6.1f  %s -> %s" % (e[0], CODE_TABLE[e[1]], CODE_TABLE[e[2]]))
    for p in stop_plan(c):
        print(p)
