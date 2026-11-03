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
    # ---------------------------------------------------------------- drum angles (deg). Order (BASE_SPEC D16):
    # LAUNCH, NEUTRAL, WRAP (collar stays in NEUTRAL), WRAPPED (still NEUTRAL, cup locked), then REEL.
    # zones per params.DRUM (BASE_SPEC v1.0). End stops: params asks (-8, 358) = 366 deg, impossible on a single turn
    # with an absolute code; the groove ends are at -4 and 352 (4 deg unreachable gap for the code ramps).
    stop_lo=-4.0,           # groove end (hard stop) below LAUNCH
    launch=0.0,
    neutral=41.0,
    wrap_start=56.0,        # park before the wrap (params target 62 sits on a code edge; 56 is mid WRAP_PREP)
    wrapped=314.0,          # WRAPPED park: wraps done, cup locked, collar NEUTRAL (ramp from 318), clutch FIRM from 312
    reel=347.0,
    stop_hi=352.0,
    tr_ln=(8.0, 32.0),      # groove 1 transition LAUNCH -> NEUTRAL
    tr_nr=(318.0, 340.0),   # groove 1 transition NEUTRAL -> REEL (after the last wrap and the re-lock)
    # ---------------------------------------------------------------- dog clutch
    stroke=7.0,             # collar travel from NEUTRAL to either side (mm) = neutral gap + engagement
    dog_h=5.0,              # dog height on each face (mm)
    dog_gap=2.5,            # tip to tip gap at NEUTRAL (mm)
    dog_n=4,                # dogs per face
    dog_deg=36.0,           # angular width of a dog at its root (deg)
    dog_backcut=4.0,        # flank back-cut (deg), both flanks: torque pulls the dogs together either way
    dog_roof=1.2,           # tip chamfer (mm) on each side of the crest
    dog_r_in=18.0, dog_r_out=26.0,
    fork_k=1.5,             # fork yoke rate (N/mm): compliant when the dogs land tip to tip
    fork_free=4.5,          # yoke deflection when the dogs land tip to tip (stroke - dog_gap)
    # ---------------------------------------------------------------- track 2 (storage reel slip clutch preload)
    brake_lift=4.0,         # full radial lift (mm) = FIRM; 0 = LIGHT (cad-reel lever, 10 N design force)
    brake_force=15.0,       # cad-reel pushrod at FIRM 5 N (D24): 7.8 N push at 3 N scaled to 13 N plus friction; cad-reel to confirm
    # (theta0, theta1, frac0, frac1): LAUNCH light, NEUTRAL firm (tidy), WRAP light, WRAPPED and REEL firm
    # FIRM before the WRAPPED park for the tidy: ramp 302..312 (302..309 would need 1.2 N m at FIRM 5 N, margin 1.8 < 2)
    brake_ramps=((30.0, 40.0, 0.0, 1.0), (50.0, 62.0, 1.0, 0.0), (302.0, 312.0, 0.0, 1.0)),  # first ramp after the collar shift
    brake_blend=0.2,        # ramp profile: constant slope with harmonic blends over this fraction at each end
    # ---------------------------------------------------------------- wrap drive (segment gear with locking arc)
    seg_m=1.25, seg_z=60,   # drum segment gear: module, tooth count of the full circle it is cut from
    seg_teeth=40,           # window = 40 pitches = 240 deg = 4 pinion turns
    seg_start=62.0,         # drum angle where the wrap pinion starts to turn
    pin_z=10, pin_x=0.4,    # wrap pinion (locked at home by the locking arc), profile shift
    idl_m=1.5, idl_z=42,    # wrap idler output gear on the pinion shaft (meshes the cad-reel cup ring gear)
    ring_z=24,              # cad-reel cup ring gear: cup turns = 4 * 42 / 24 = 7 (BASE_SPEC: 7 cup turns)
    cup_torque=0.06,        # N m at the cup while laying wraps with the brake LIGHT (cad-reel to confirm)
    # ---------------------------------------------------------------- N20 drive
    n20_z=10, n20_x=0.3, drum_gear_z=120, drum_gear_m=0.8, drum_gear_x=-0.3,
    n20_rpm_12v=27.0, n20_v=9.0,        # DRV8833 fed by an MP1584 buck set to 9 V
    n20_stall_nm_9v=0.20,               # estimate, see stall_estimate()
    n20_work_frac=0.35,                 # keep the working load under this fraction of stall
    # ---------------------------------------------------------------- switch code (Gray code, bit = contact closed)
    # params.DRUM.codes (A B C): LAUNCH 100, L_TO_N 110, NEUTRAL 010, WRAP_PREP 011, WRAP 001, N_TO_R 101, REEL 111
    code_arcs={
        "b0": ((-8.0, 34.0), (302.0, 362.0)),     # A
        "b1": ((6.0, 62.0), (342.0, 354.0)),      # B
        "b2": ((50.0, 354.0),),                   # C
    },
    code_ramp=2.0,          # bump ramp half width (deg): nub centre lift 50 percent at the arc edge
    sw_tol=1.5,             # expected edge tolerance (deg)
)

# code (b2 b1 b0 as an int) -> sector name. 0 = all open = wire off or no bump: never valid.
CODE_TABLE = {               # int = b2 b1 b0 = C B A; names = params.DRUM zones
    0b001: "LAUNCH",
    0b011: "L_TO_N",
    0b010: "NEUTRAL",
    0b110: "WRAP_PREP",
    0b100: "WRAP",
    0b101: "N_TO_R",
    0b111: "REEL",
    0b000: "INVALID",
}


def code_abc(code_int):
    """params.DRUM.codes string (bits A B C) for a code int (b2 b1 b0 = C B A)."""
    return "".join("1" if code_int >> b & 1 else "0" for b in (0, 1, 2))


def cfg(P=None):
    """DEFAULTS overridden by params.DRUM (dict) if present."""
    d = dict(DEFAULTS)
    over = getattr(P, "DRUM", None) if P is not None else None
    if isinstance(over, dict):
        d.update({k: v for k, v in over.items() if k in d})
        ang = over.get("angles")
        if isinstance(ang, dict):
            for k in ("LAUNCH", "NEUTRAL", "WRAPPED", "REEL"):
                if ang.get(k) is not None:
                    d[k.lower()] = float(ang[k])
        tg = over.get("targets")
        if isinstance(tg, dict):           # params v1.0 targets; the two on code edges are kept off the edges
            for k, kk in (("LAUNCH", "launch"), ("NEUTRAL", "neutral"), ("REEL", "reel")):
                if tg.get(k) is not None:
                    d[kk] = float(tg[k])
    return d


# ------------------------------------------------------------------------------------------------ profiles
def _harm(u):
    """Harmonic rise 0..1 for u in 0..1 (zero slope at both ends)."""
    u = min(1.0, max(0.0, u))
    return 0.5 - 0.5 * math.cos(math.pi * u)


def collar_x(theta, c=None):
    """Collar position (mm) commanded by groove 1 at drum angle theta (-stroke LAUNCH, 0 NEUTRAL, +stroke REEL)."""
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


def _trap(u, b):
    """0..1 rise with constant slope in the middle and harmonic blends of width b at both ends (peak slope 1/(1-b))."""
    u = min(1.0, max(0.0, u))
    if b <= 0:
        return u
    v = 1.0 / (1.0 - b)                     # middle slope
    if u < b:
        return _blend(u, b, v)
    if u > 1 - b:
        return 1.0 - _blend(1.0 - u, b, v)
    return _blend(b, b, v) + v * (u - b)


def _blend(u, b, v):
    """Distance covered from rest with the slope rising as v sin^2(pi u / (2 b)) over 0..b."""
    return v * (u / 2 - (b / (2 * math.pi)) * math.sin(math.pi * u / b))


def brake_frac(theta, c=None):
    """Track 2 lift fraction 0 (light) .. 1 (firm)."""
    c = c or DEFAULTS
    f = 0.0
    for t0, t1, f0, f1 in c["brake_ramps"]:
        if theta >= t1:
            f = f1
        elif theta > t0:
            return f0 + (f1 - f0) * _trap((theta - t0) / (t1 - t0), c.get("brake_blend", 0.2))
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
    if s in ("LAUNCH", "NEUTRAL", "REEL"):
        return s
    if s == "N_TO_R" and K_wrapped_flat(theta, c):
        return "WRAPPED"
    return "BETWEEN"


def K_wrapped_flat(theta, c=None):
    c = c or DEFAULTS
    return c["seg_start"] + seg_span(c) <= theta <= c["tr_nr"][0]


def code_lift(theta, bit, c=None):
    """Nub centre lift fraction 0..1 of code band `bit` at drum angle theta (ramps centred on the arc edges)."""
    c = c or DEFAULTS
    h = c["code_ramp"]
    best = 0.0
    for a0, a1 in c["code_arcs"]["b%d" % bit]:
        d = min(theta - a0, a1 - theta)
        best = max(best, min(1.0, max(0.0, 0.5 + d / (2 * h))))
    return best


def mode(theta, c=None):
    """Mechanical mode: which gear the collar engages (LAUNCH, NEUTRAL, REEL)."""
    c = c or DEFAULTS
    x = collar_x(theta, c)
    e = c["dog_gap"]                      # dogs start to overlap once the collar has moved past the gap
    if x <= -e:
        return "LAUNCH"
    if x >= e:
        return "REEL"
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


def cam_eta(slope_mm_per_rad, r_mm, mu=0.3):
    """Efficiency of a cam (or groove) pushing a sliding follower: tan(phi) / tan(phi + rho), phi = pressure angle."""
    if slope_mm_per_rad <= 1e-9:
        return 1.0
    phi = math.atan(slope_mm_per_rad / r_mm)
    rho = math.atan(mu)
    return math.tan(phi) / math.tan(min(phi + rho, math.radians(89.0)))


def loads(c=None, r_groove=30.0, r_cam=32.0, rack_arm=45.0, sleeve_len=40.0, mu=0.3, residual_nm=0.2):
    """Drum torque budget (N m at the drum) along the travel.

    Fork: yoke force at a full tip-to-tip block (fork_k x fork_free) plus 3 N rail friction, times the racking factor
    of the long carrier (pin 45 mm off the rail, 40 mm sleeve: 1 + 2 mu arm / sleeve), through the groove at its local
    pressure angle. Disengage: 0.2 N m residual train torque after the firmware unload jog. Brake: cad-reel follower force through the cam at its pressure angle. Wrap: cup torque geared up by
    the wrap train. Loads at the same angle add. N20 margin rule (params.DRUM.n20_ratio_min_margin): stall / peak >= 2."""
    c = c or DEFAULTS
    eta_gear = 0.9
    rack = 1.0 + 2.0 * mu * rack_arm / sleeve_len
    f_fork = (c["fork_k"] * c["fork_free"] + 3.0) * rack
    r_dog = 0.5 * (c["dog_r_in"] + c["dog_r_out"]) / 1000.0
    f_dis = ((residual_nm / r_dog) * (mu + math.tan(math.radians(c["dog_backcut"]))) + 3.0) * rack
    f_brk = c["brake_force"]
    up = (c["seg_z"] / c["pin_z"]) * (c["idl_z"] / c["ring_z"])
    t_wrap = c["cup_torque"] * up / (eta_gear ** 2)
    cap_stall = c["n20_stall_nm_9v"] * ratio(c) * eta_gear
    t0 = c["seg_start"]
    t1 = t0 + seg_span(c)
    prof = []
    pk = dict(fork=0.0, brake=0.0)
    th = c["stop_lo"]
    while th <= c["stop_hi"]:
        sl = abs(collar_slope(th, c))
        bs = abs((brake_lift(th + 0.01, c) - brake_lift(th - 0.01, c)) / math.radians(0.02))
        tf_ = max(f_fork, f_dis) * sl / 1000.0 / cam_eta(sl, r_groove, mu)
        tb = f_brk * bs / 1000.0 / cam_eta(bs, r_cam, mu)
        pk["fork"] = max(pk["fork"], tf_)
        pk["brake"] = max(pk["brake"], tb)
        prof.append((th, tf_ + tb + (t_wrap if t0 <= th <= t1 else 0.0)))
        th += 0.5
    worst = max(prof, key=lambda p: p[1])
    slope = max(abs(collar_slope(t, c)) for t in range(int(c["stop_lo"]), int(c["stop_hi"])))
    return dict(
        fork_shift=pk["fork"], brake_ramp=pk["brake"], wrap=t_wrap,
        worst=worst[1], worst_at=worst[0], margin=cap_stall / worst[1],
        cap_stall=cap_stall, cap_work=c["n20_work_frac"] * cap_stall,
        f_fork=f_fork, f_disengage=f_dis, groove_slope_mm_per_rad=slope, wrap_gear_up=up,
        groove_pressure_deg=math.degrees(math.atan(slope / r_groove)), racking=rack,
    )


def timing(c=None):
    """Firmware timing table: nominal seconds for each move at drum_dps()."""
    c = c or DEFAULTS
    v = drum_dps(c)
    return dict(
        dps=v,
        launch_to_neutral=(c["neutral"] - c["launch"]) / v,
        neutral_to_wrapped=(c["wrapped"] - c["neutral"]) / v,
        wrap_window=seg_span(c) / v,
        wrapped_to_reel=(c["reel"] - c["wrapped"]) / v,
        launch_to_reel=(c["reel"] - c["launch"]) / v,
        full_travel=(c["stop_hi"] - c["stop_lo"]) / v,
    )


def stop_plan(c=None):
    """For each target and direction: the code edge to watch and the extra angle to run after it."""
    c = c or DEFAULTS
    es = edges(c)
    plans = []
    for name in ("launch", "neutral", "wrap_start", "wrapped", "reel"):
        tgt = c[name]
        below = [e for e in es if e[0] < tgt]
        above = [e for e in es if e[0] > tgt]
        if below:
            e = below[-1]
            plans.append(dict(target=name.upper(), dir=+1, edge=e[0], code_from=e[1], code_to=e[2],
                              extra_deg=tgt - e[0]))
        if above:
            e = above[0]
            p = dict(target=name.upper(), dir=-1, edge=e[0], code_from=e[2], code_to=e[1], extra_deg=e[0] - tgt)
            if p["extra_deg"] > 20.0 and below:
                # too far to run by time: pass the target down to the edge below it, then come back up
                eb = below[-1]
                p = dict(target=name.upper(), dir=-1, method="pass_and_return", edge=eb[0], code_from=eb[2],
                         code_to=eb[1], extra_deg=tgt - eb[0], note="drive - until the edge, then + by extra_deg")
            plans.append(p)
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


def firmware_table(c=None):
    """Everything fw-base needs, JSON-ready (written to drum_table.json by `python _drum_kin.py --json`)."""
    c = c or DEFAULTS
    return dict(
        version="drum v4 (BASE_SPEC v1.0)",
        positions={k.upper(): c[k] for k in ("launch", "neutral", "wrap_start", "wrapped", "reel")},
        stops=[c["stop_lo"], c["stop_hi"]],
        code_table_ABC={code_abc(k): v for k, v in CODE_TABLE.items()},
        code_bits={"SW_DRUM_A": "b0", "SW_DRUM_B": "b1", "SW_DRUM_C": "b2"},
        code_arcs={k: [list(a) for a in v] for k, v in c["code_arcs"].items()},
        edges=[dict(deg=e[0], below=code_abc(e[1]), above=code_abc(e[2])) for e in edges(c)],
        stop_plan=stop_plan(c),
        timing_s=timing(c),
        dps_nominal=drum_dps(c),
        wrap_window=[c["seg_start"], c["seg_start"] + seg_span(c)],
        cup_turns=wraps_total(c),
        collar_transitions=dict(LN=list(c["tr_ln"]), NR=list(c["tr_nr"])),
        brake_ramps=[list(r) for r in c["brake_ramps"]],
        rules=[
            "Positive drum = N20 forward (DRV8833 #3 AIN1 high) = toward REEL = wrap direction.",
            "Wiper stopped before every drum move, except the fly-in into REEL: creep the wiper forward over 318..340.",
            "Check SW_DOG_L / SW_DOG_R during and after every move: LAUNCH needs DOG_L closed, REEL needs DOG_R closed, "
            "NEUTRAL / WRAP_PREP / WRAP / WRAPPED need both open; never enter the wrap window with either closed.",
            "Dog not closed in LAUNCH: jog the wiper +-8 deg; in REEL: creep forward only (sprag #2 blocks backward).",
            "Leaving LAUNCH or REEL under load: unload jog 8 deg against the last load first. SWITCH_DISAGREE after 3 tries.",
            "Fault from REEL or N_TO_R: go to WRAPPED (309), never back through the wrap window. Fault from LAUNCH or L_TO_N: NEUTRAL.",
            "Boot: LAUNCH / L_TO_N / NEUTRAL / WRAP_PREP -> NEUTRAL; WRAP -> stop and report; N_TO_R / REEL -> WRAPPED.",
            "Unwrap (WRAPPED back to NEUTRAL): drum negative with the wiper creeping forward (bus take-up).",
            "SW_N20 open = hand fallback active: no drum moves.",
            "Code 000 = DRUM_SENSOR fault. Timeout = 2 x nominal + 2 s: DRUM_TIMEOUT.",
            "SW_INDEX: 6 pulses per wiper turn (degraded mode without the AS5600 magnet).",
        ],
    )


if __name__ == "__main__" and "--json" in __import__("sys").argv:
    import json, os
    fn = os.path.join(os.path.dirname(os.path.abspath(__file__)), "drum_table.json")
    with open(fn, "w") as fh:
        json.dump(firmware_table(), fh, indent=1)
    print(fn)
