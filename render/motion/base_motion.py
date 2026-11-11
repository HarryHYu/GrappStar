"""Base station kinematics for the render: the D50 base's own joints, kin() and line_path() (base/cad/parts/*.py via
a joints-only tools.assembly.Assembly), posed with base/out/base/asm.json's part list (names, joint, place).

    B = BaseKin()
    W = B.world(S)                 # {asm part name: 4x4 base frame, mm}
    pts, exit_i = B.line_fixed(S)  # tether from the storage spool to the pusher hole (base frame, mm)
    S = B.state(tb)                # master state at base time tb (s) from the render timeline below

Base frame: BASE_SPEC section 2 (mm, origin on the wiper / main shaft axis, +X launch, +Z up, deck top z -165).
"""
from __future__ import annotations

import copy
import importlib
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
BASE = os.path.join(ROOT, "base")
for p in (BASE, os.path.join(BASE, "cad", "parts")):
    if p not in sys.path:
        sys.path.insert(0, p)

from tools.params import load_params  # noqa: E402
from tools.assembly import Assembly  # noqa: E402
from tools import timeline as TL  # noqa: E402

SUBS = ["drum", "frame", "launch", "reel", "sensors"]          # base/out/base/asm.json "subsystems" (full D50 build)
SKIP_PARTS = {"launch_dart"}                                    # replaced by the KEDGE v34 dart (dart:* objects)

# base -> world (metres): the deck underside (base z -173 mm) on the board top (world z 0); the deck footprint
# (x -410.5..269.4, y -137.8..234.8 incl. the D50 extension tile) centred on the world origin; +X = launch azimuth.
BASE_TO_WORLD = np.array([[1.0, 0.0, 0.0, 0.0705],
                          [0.0, 1.0, 0.0, -0.0485],
                          [0.0, 0.0, 1.0, 0.173],
                          [0.0, 0.0, 0.0, 1.0]])

W34 = None      # wiper degrees backward from LOW to the 34 deg aim (set by BaseKin from the aim table)


class BaseKin:
    def __init__(self, aim_deg=34.0):
        P = load_params()
        self.P = P
        mods = {n: importlib.import_module(f"cad.parts.{n}") for n in SUBS}
        A = Assembly.__new__(Assembly)            # joints and kin only: no CAD solids are built
        A.P, A.modules, A.joints = P, mods, {}
        for m in mods.values():
            for j in m.joints(P):
                A.joints[j.name] = j
        self.A = A
        self.asm = json.load(open(os.path.join(BASE, "out", "base", "asm.json")))
        self.parts = [(p["name"], p["joint"], np.array(p["place"], float)) for p in self.asm["parts"]
                      if p["name"] not in SKIP_PARTS]
        aj = {j["name"]: j for j in self.asm["joints"]}
        bad = [n for n, j in A.joints.items() if n not in aj or j.parent != aj[n]["parent"]
               or np.abs(np.array(j.origin) - np.array(aj[n]["origin"])).max() > 1e-6
               or np.abs(np.array(j.axis) - np.array(aj[n]["axis"])).max() > 1e-9]
        if bad or set(aj) - set(A.joints):
            raise RuntimeError(f"base joints differ from asm.json: {bad} {set(aj) - set(A.joints)}")
        import launch as LA
        import launch_kin as K
        import launch_aim as AIM
        import launch_line as LN
        import launch_muzzle as MZ
        import reel as RL
        self.LA, self.K, self.AIM, self.LN, self.MZ, self.RL = LA, K, AIM, LN, MZ, RL
        self.n = LA.L(P)
        self.g = K.gears(P)
        self.ratio = self.g["ratio"]
        self.g0 = float(LA.gear_angle_for_travel(P, 0.0))            # gear at HOME, cock start (-2.0)
        self.g_rel = float(LA.gear_release_angle(P))                 # relieved tooth lets go (-281.25)
        self.rt = LA.release_table(P)
        self.stroke = float(self.n["stroke"])
        global W34
        self.aim_deg = float(aim_deg)
        d34 = float(AIM.psi_of_pitch(P, self.aim_deg)[1])          # crank deg from LOW on the rising stroke
        W34 = self._tune_aim(d34 * K.wiper_per_crank(P))
        self.W34 = W34
        self.build_timeline()

    # ------------------------------------------------------------------ posing
    def joint_world(self, S):
        return self.A.joint_world(S)

    def values(self, S):
        return self.A.values(S)

    def world(self, S, JW=None):
        JW = JW if JW is not None else self.A.joint_world(S)
        I = np.eye(4)
        return {n: (JW[j] if j else I) @ pl for n, j, pl in self.parts}

    def pitch(self, S):
        return float(self.A.values(S)["launch_rail"])

    def snap_pitch(self, deg):
        """the rail pitch the aim train actually reaches nearest deg (kin locks the crank to the sun teeth) and its wiper"""
        keep = (self.aim_deg, getattr(self, "aim_end_pitch", None))
        self.aim_deg = float(deg)
        d = float(self.AIM.psi_of_pitch(self.P, self.aim_deg)[1])
        w = self._tune_aim(d * self.K.wiper_per_crank(self.P))
        p = self.aim_end_pitch
        self.aim_deg, self.aim_end_pitch = keep
        return p, w

    def _tune_aim(self, w0):
        """wiper backward degrees for the aim pitch: kin() locks the crank to the sun teeth, so search near w0"""
        best = None
        tgt = self.aim_deg

        def pitch_of(w):
            return self.pitch(dict(wiper_deg=-w, sleeve_deg=-w, aim_deg=tgt, cock=0.0, fired=0, phase="aim",
                                   drum_deg=0.0, reel_deg=0.0, line_out_m=0.0, wraps=0.0, mode="LAUNCH"))
        for dw in np.arange(-8.0, 8.01, 0.25):
            p = pitch_of(w0 + dw)
            if best is None or abs(p - tgt) < abs(best[1] - tgt):
                best = (w0 + dw, p)
        for dw in np.arange(-0.25, 0.2501, 0.01):
            p = pitch_of(best[0] + dw)
            if abs(p - tgt) < abs(best[1] - tgt):
                best = (best[0] + dw, p)
        self.aim_end_pitch = best[1]
        return best[0]

    # ------------------------------------------------------------------ line
    def rail_W(self, pitch):
        return self.LA._rail_to_world(self.P, pitch)

    def line_fixed(self, S):
        """Tether from the storage spool to the pusher hole (base frame mm, N x 3) and the index range of the capstan
        turns (None when unwrapped): reel.line_spans (spool, capstan turns, cup eyelet, floor segment) + the launch span
        round the rail-rear pulley and along the rail to the pusher (launch.line_path span 1)."""
        pts, wr = [], None
        for span, _, label in self.RL.line_spans(self.P, S):
            i0 = len(pts)
            for p in np.asarray(span, float):
                if not pts or np.linalg.norm(p - pts[-1]) > 1e-6:
                    pts.append(p)
            if label == "capstan turns":
                wr = (max(0, i0 - 1), len(pts) - 1)
        sp = self.LA.line_path(self.P, S)
        for p in np.asarray(sp[0][0], float):
            if np.linalg.norm(p - pts[-1]) > 1e-6:
                pts.append(p)
        return np.array(pts), wr

    def roller(self, S, travel=None):
        """muzzle lip roller (centre u, w in the rail frame, line path radius) for the carriage travel"""
        v = self.A.values(S)
        tr = v["launch_carriage"] if travel is None else travel
        return self.MZ.U_R + self.n["u_c_home"] - tr, self.MZ.W_R, self.LN.R_PATH

    def spring_ends(self, S, JW=None):
        """Per launch spring (l, r): the anchor pin hook centre (rail) and the carriage peg hook centre, base frame mm.
        The coils run from HOOK_L (12 mm) inside each pin centre (the base's segment model, launch.parts)."""
        JW = JW if JW is not None else self.A.joint_world(S)
        n = self.n
        out = {}
        for sg, side in ((1, "l"), (-1, "r")):
            y = sg * n["y_spring"]
            a = JW["launch_rail"] @ np.array([n["u_anchor"], y, n["w_spring"], 1.0])
            c = JW["launch_carriage"] @ np.array([n["u_c_home"] + n["u_peg_c"], y, n["w_spring"], 1.0])
            out[side] = (a[:3], c[:3])
        return out

    # ------------------------------------------------------------------ the render timeline (base time, s)
    def build_timeline(self):
        """Phases in tools.timeline's custom format (name, dur, text, targets), values eased by TL._state_custom.
        Numbers follow base/spec/params.py TIMELINE (D50), with the aim at 34 deg instead of 36. The shot itself
        (release, stroke, carriage stop, reset) is not eased: fire_state() builds it from the launcher physics."""
        W = self.W34
        wc0 = -W                        # wiper at cock start (rail at 34 deg, gear at HOME -2.0)
        self.wc0 = wc0
        w_fire_end = wc0 + 553.8        # one big gear turn per shot (base TIMELINE): gear at rest -360 in its gap
        w_low = -3046.2                 # rest of the aim cycle backward: 3600 from LOW less the 553.8 forward (D45)
        w_tidy = w_low + 430.2
        self.phases = [
            ("load", 4.0, "idle, dart seated, drum LAUNCH, rail low",
             dict(wiper_deg=0.0, sleeve_deg=0.0, reel_deg=0.0, drum_deg=0.0, aim_deg=6.02, cock=0.0, fired=0, wraps=0,
                  line_out_m=0.0, dart_flight=0.0, dart_return=0.0, anchored=0, mode="LAUNCH", drum_pos="LAUNCH")),
            ("aim", 5.0, "wiper backward: crank-rocker lifts the rail to the aim pitch", dict(wiper_deg=-W, sleeve_deg=-W, aim_deg=self.aim_end_pitch)),
            ("arm", 0.5, "aimed, worm holds", dict()),
            ("cock", 5.0, "wiper forward 425 deg: big gear draws the carriage back 241 mm",
             dict(wiper_deg=wc0 + 425.0, sleeve_deg=wc0 + 425.0, cock=1.0)),
            ("cocked", 1.0, "full cock, held on the tooth plateau", dict()),
            # fire_state() covers the shot; the next phase starts from its end state
            ("flight", 0.01, "dart away (the shot itself: fire_state)",
             dict(wiper_deg=w_fire_end, sleeve_deg=w_fire_end, cock=0.0, fired=1)),
            ("rail_low", 4.0, "D45: wiper backward over the top of the aim cycle, rail to 6 deg",
             dict(aim_deg=6.0, wiper_deg=w_low, sleeve_deg=w_low)),
            ("neutral", 8.0, "unload jog, drum to NEUTRAL", dict(drum_deg=41.0, mode="NEUTRAL", drum_pos="NEUTRAL")),
            ("wrap", 49.0, "drum turns the wrap cup 7 times round the capstan", dict(drum_deg=302.0, wraps=7, drum_pos="BETWEEN")),
            ("wrapped", 2.5, "drum to the WRAPPED park 314", dict(drum_deg=314.0, drum_pos="WRAPPED")),
            ("anchor", 1.0, "waiting for ANCHORED", dict(anchored=1)),
            ("tidy", 3.0, "wiper forward (collar NEUTRAL): the take-up pulls the slack through the free capstan", dict(wiper_deg=w_tidy)),
            ("reel_engage", 6.5, "drum into REEL while the wiper creeps forward", dict(drum_deg=347.0, wiper_deg=w_tidy + 6.0,
                                                                                    reel_deg=0.0, mode="REEL", drum_pos="REEL")),
            ("tow", 6.0, "capstan winds until the line is tight, worm holds", dict()),
        ]
        t0 = 0.0
        self.phases_d = [(n, d) for n, d, _, _ in self.phases]
        self.t0 = {}
        for name, d, _, _ in self.phases:
            self.t0[name] = t0
            t0 += d
        self.t_end = t0

    def state(self, tb, phase_override=None):
        """master state at base time tb (s) on the render timeline (the shot excluded: see fire_state)"""
        P = self.P
        S0 = dict(t=tb, wiper_deg=0.0, drum_deg=0.0, line_out_m=0.0, mode="LAUNCH", aim_deg=6.02, cock=0.0,
                  fired=False, wraps=0.0, phase="load")
        S = TL._state_custom(S0, [(n, d, t, g) for n, d, t, g in self.phases], tb)
        if phase_override:
            S["phase"] = phase_override
        S["t"] = tb
        return S

    # ------------------------------------------------------------------ the shot
    def fire_state(self, S_cocked, tau, trace, w_rate=270.0, w_rate_loaded=225.0, stop=None, wprof=None):
        """State during the shot, tau (s) after the wiper passes full cock (425 fwd). The relieved last tooth lets go at
        wiper 429.6 fwd (gear -281.25, release table); the carriage then follows the launcher physics trace
        (base/physics launcher.py, t, x) home, overshoots into the PLA stop flexure and settles; the wiper keeps turning
        and carries the big gear on through its gap to rest (-360) at 553.8 fwd (base TIMELINE). Returns (S, info)."""
        wc0, ratio = self.wc0, self.ratio
        w_rel = wc0 + (self.g0 - self.g_rel) * ratio            # 429.6 fwd
        t_rel = (w_rel - (wc0 + 425.0)) / w_rate_loaded
        w_end = wc0 + 553.8
        if wprof is not None:                                   # real motor profile (RealSchedule): (t, w) from full cock
            t_rel = float(np.interp(w_rel, wprof[1], wprof[0]))
        S = dict(S_cocked)
        S["phase"] = "fire"
        S["fired"] = 1
        S["cock"] = 1.0
        info = dict(t_rel=t_rel)
        if tau < t_rel:
            w = wc0 + 425.0 + w_rate_loaded * tau if wprof is None else float(np.interp(tau, wprof[0], wprof[1]))
            g = self.g0 - (w - wc0) / ratio
            xs, ys = -np.asarray(self.rt["theta"]), np.asarray(self.rt["travel"])
            k = int(np.clip(np.searchsorted(xs, -g), 1, len(xs) - 1))
            travel = float(max(ys[k - 1], ys[k]))
            travel = min(travel, self.stroke)
            S.update(wiper_deg=w, sleeve_deg=w, launch_gear_deg=g, launch_travel=travel)
            info.update(stage="release", travel=travel, dart_u=0.0)
            return S, info
        ts = tau - t_rel
        w = min(w_end, w_rel + w_rate * ts) if wprof is None else float(np.interp(tau, wprof[0], wprof[1]))
        g = self.g_rel - (w - w_rel) * (360.0 + self.g_rel) / (w_end - w_rel)      # -281.25 -> -360 over the reset
        tt, xx = trace[:, 0], trace[:, 1] * 1000.0
        T_st = float(tt[np.argmax(xx >= xx.max() - 1e-6)])
        rel0 = float(self.rt["travel_release"])
        if ts <= T_st:
            x = float(np.interp(ts, tt, xx))
            travel = rel0 * (1.0 - x / xx.max())
        else:
            travel = carriage_stop(ts - T_st, stop or getattr(self, "stop_model", None))
        S.update(wiper_deg=w, sleeve_deg=w, launch_gear_deg=g, launch_travel=travel)
        if ts >= T_st and w >= w_end - 1e-3:
            S["phase"] = "flight"
            S.pop("launch_gear_deg")
            S.pop("launch_travel")
            S["cock"] = 0.0
        info.update(stage="stroke" if ts <= T_st else "stop", travel=travel, T_stroke=T_st)
        return S, info


def carriage_stop_model(v_c, e=0.25, k=15000.0, m=0.0805 + 2 * 0.053 / 3, mu=0.3):
    """Carriage after it reaches HOME at v_c (m/s): 1-DOF model of the carriage (80.5 g + a third of the two 53 g
    springs) on the PLA stop flexure (linear, 15 kN/m: about 19 mm and 285 N for the 2.7 J, base physics launch /
    recoil, BASE_SPEC D26) with damping on its rebound only (hysteresis), tuned to a restitution e (assumed 0.25: printed
    PLA flexure), the two springs pulling it home (7 N + 0.125 N/mm each over the 23.8 mm preload) and rail friction
    (mu 0.3 on its weight). Returns dict(t, u) with u = carriage travel in mm (negative = past HOME into the flexure)."""

    u0 = 2.0 * (7.0 + 0.125 * 23.8) / k                   # the springs hold the carriage this far into the flexure at HOME (u 0)

    def sim(c, record=False):
        dt = 1e-5
        u, v = 0.0, -v_c
        T, U = [0.0], [0.0]
        t, v_out, settle = 0.0, None, None
        while t < 0.4:
            Fs = -2.0 * (7.0 + 0.125 * (23.8 + u * 1000.0))     # springs pull toward HOME (-u)
            Ff = 0.0
            if u < u0:
                Ff = max(0.0, k * (u0 - u) - (c * v if v > 0 else 0.0))   # the flexure pushes only; damped on the way out
            Fr = -math.copysign(mu * m * 9.81, v) if abs(v) > 1e-4 else 0.0
            u_prev = u
            v += (Fs + Ff + Fr) / m * dt
            u += v * dt
            t += dt
            if u_prev < u0 <= u and v > 0 and v_out is None:
                v_out = v
            if settle is None and v_out is not None and abs(u) < 2e-5 and abs(v) < 0.01:
                settle = t
            if record and t - T[-1] >= 2e-4:
                T.append(t)
                U.append(u * 1000.0)
            if settle is not None and t > settle + 0.003:
                U[-1] = 0.0
                break
            if v_out is None and u < 0 and v > 0 and t > 0.05:
                break
        return v_out or 0.0, T, U, settle or t
    lo, hi = 0.0, 200.0
    for _ in range(40):
        c = 0.5 * (lo + hi)
        vo, *_ = sim(c)
        if vo > e * v_c:
            lo = c
        else:
            hi = c
    vo, T, U, settle = sim(0.5 * (lo + hi), True)
    U = np.array(U)
    return dict(t=np.array(T), u=U, umin=float(U.min()), umax=float(U.max()), t_settle=float(settle), e=e, v_out=vo,
                c=0.5 * (lo + hi))


def carriage_stop(t, stop=None):
    """carriage travel (mm) t s after it reaches HOME, from carriage_stop_model (0 before and after)"""
    if t < 0 or stop is None:
        return 0.0
    if t > stop["t"][-1]:
        return 0.0
    return float(np.interp(t, stop["t"], stop["u"]))


def to_world_m(M_mm):
    """base frame 4x4 (mm) -> world 4x4 (m)"""
    M = np.array(M_mm, float).copy()
    M[:3, 3] *= 0.001
    return BASE_TO_WORLD @ M


def pts_world_m(p_mm):
    p = np.asarray(p_mm, float) * 0.001
    return p @ BASE_TO_WORLD[:3, :3].T + BASE_TO_WORLD[:3, 3]


# ====================================================================== real-time motor moves (v2: 1:1 time)
WIPER_N0 = 270.0          # deg/s, wiper no-load speed (params WIPER n0_rpm 45)
WIPER_TS = 15.0           # N m, wiper stall torque (params WIPER stall_nm)
WIPER_TAU = 0.06          # s, spin-up time constant (motor + gearbox + base train inertia)
WIPER_BRAKE = 4000.0      # deg/s^2, stop on the BTS7960 brake (270 deg/s stops in about 70 ms, 9 deg)
DRUM_RATE = 8.1           # deg/s, selector drum on its N20 (params DRUM n20 drum_deg_per_s)
DRUM_RAMP = 0.08          # s
AIM_LOAD = 1.2            # N m at the wiper running backward in LAUNCH (aim train; the firmware trips at 3 A = 1.36 N m)
FRICTION = 0.3            # N m, light running load of the trains


def motor_move(w0, w1, load=lambda w: FRICTION, duty=1.0, dt=1e-4):
    """Wiper from w0 to w1 (deg, from rest to rest): DC motor (no-load speed x duty less n0 / Ts per N m of load),
    first-order spin-up, braked stop. Returns arrays (t from 0, w)."""
    d = 1.0 if w1 >= w0 else -1.0
    w, om, t = float(w0), 0.0, 0.0
    T, W = [0.0], [w]
    while True:
        rem = (w1 - w) * d
        if rem <= 1e-4:
            break
        oss = max(5.0, duty * WIPER_N0 - WIPER_N0 / WIPER_TS * load(w))
        om += (oss - om) * dt / WIPER_TAU
        om = min(om, (2.0 * WIPER_BRAKE * rem) ** 0.5)
        w += d * min(om * dt, rem)
        t += dt
        T.append(t)
        W.append(w)
    return np.array(T), np.array(W)


def drum_move(d0, d1):
    """selector drum d0 -> d1 at the N20's 8.1 deg/s with short ramps: (t, d)"""
    L = abs(d1 - d0)
    v = DRUM_RATE
    tr = DRUM_RAMP
    tc = max(0.0, L / v - tr)
    tt = np.linspace(0.0, tc + 2 * tr, max(3, int((tc + 2 * tr) / 0.01) + 1))
    s = np.where(tt < tr, 0.5 * v / tr * tt ** 2,
                 np.where(tt < tr + tc, 0.5 * v * tr + v * (tt - tr),
                          L - 0.5 * v / tr * np.maximum(0.0, tc + 2 * tr - tt) ** 2))
    s = np.clip(s * (L / max(s[-1], 1e-9)), 0, L)
    return tt, d0 + np.sign(d1 - d0) * s


class Channel:
    """piecewise value over time: hold between moves, moves as (t, value) arrays"""

    def __init__(self, v0):
        self.T = [0.0]
        self.V = [float(v0)]

    def hold_until(self, t):
        if t > self.T[-1]:
            self.T.append(t)
            self.V.append(self.V[-1])

    def move(self, t0, arr_t, arr_v):
        self.hold_until(t0)
        self.T += list(t0 + np.asarray(arr_t[1:]))
        self.V += list(np.asarray(arr_v[1:]))
        return self.T[-1]

    def at(self, t):
        return float(np.interp(t, self.T, self.V))

    @property
    def end(self):
        return self.T[-1]


class RealSchedule:
    """The base's real-time sequence (firmware C1 order, BASE_SPEC 10.1) with motor-limited moves:
    idle, aim (wiper backward), arm pause, cock (wiper forward against the springs), hold, the shot (fire_state),
    then after LANDED: rail low over the top of the aim cycle (D45), unload jog, drum to NEUTRAL, the 7-turn wrap, the
    WRAPPED park; after ANCHORED: tidy (take-up, wiper forward in NEUTRAL), REEL fly-in (drum to 347 with a wiper creep),
    tow (capstan wind until tight, the worm then holds)."""

    def __init__(self, B, t_idle=3.0, t_arm=0.5, t_hold=0.8):
        self.B = B
        W = B.W34
        self.wiper = Channel(0.0)
        self.drum = Channel(0.0)
        self.reel = Channel(0.0)
        self.events = {}
        t = t_idle
        self.events["aim_start"] = t
        tt, ww = motor_move(0.0, -W, lambda w: AIM_LOAD)
        t = self.wiper.move(t, tt, ww)
        self.events["aimed"] = t
        t += t_arm
        self.events["cock_start"] = t
        self.wc0 = -W
        n = B.n
        rb = B.g["r_big"] / 1000.0
        stroke = B.stroke

        def cock_load(w):
            x = min(max((w - self.wc0) / 425.0, 0.0), 1.0) * stroke
            F = 2.0 * (7.0 + 0.125 * (n["preload"] + x))
            return F * rb / B.ratio / 0.85 + FRICTION
        self.cock_load = cock_load
        tt, ww = motor_move(-W, -W + 425.0, cock_load)
        t = self.wiper.move(t, tt, ww)
        self.events["cocked"] = t
        t += t_hold
        self.t_fire0 = t
        # the shot: wiper restarts from full cock, lets go at 429.6, carries the gear through its gap to 553.8
        w_rel = self.wc0 + (B.g0 - B.g_rel) * B.ratio
        tt, ww = motor_move(self.wc0 + 425.0, self.wc0 + 553.8,
                            lambda w: cock_load(w) if w < w_rel else FRICTION)
        self.fire_t, self.fire_w = tt, ww
        self.w_rel = w_rel
        self.t_rel = float(np.interp(w_rel, ww, tt))
        t_end = self.wiper.move(t, tt, ww)
        self.t_fire_end = t_end
        self.events["fire"] = t + self.t_rel

    def post_landing(self, t_landed, t_anchored, tidy_until_taut):
        """moves after the dart lands (t_landed = Jetson LANDED verdict) and anchors; tidy_until_taut(t_start) -> the
        take-up duration (from the rope); returns the schedule end (tow start)"""
        t = self.post_landing_head(t_landed, t_anchored)
        return self.post_landing_tail(t, tidy_until_taut(t))

    def post_landing_head(self, t_landed, t_anchored):
        """rail low, unload jog, drum to NEUTRAL, the wrap, the park, then wait for ANCHORED: returns the tidy start"""
        B = self.B
        t = max(t_landed, self.t_fire_end)
        self.events["base_rail_low_start"] = t
        w = self.wiper.V[-1]
        w_low = -3046.2
        tt, ww = motor_move(w, w_low, lambda x: AIM_LOAD)
        t = self.wiper.move(t, tt, ww)
        self.sleeve_freeze = t                                   # the collar leaves LAUNCH after this
        tt, ww = motor_move(w_low, w_low - 6.0, lambda x: AIM_LOAD)      # unload jog inside the dog window
        t = self.wiper.move(t + 0.3, tt, ww)
        self.events["base_drum_to_neutral"] = t + 0.3
        tt, dd = drum_move(0.0, 41.0)
        t = self.drum.move(t + 0.3, tt, dd)
        self.events["base_wrap_start"] = t + 0.2
        tt, dd = drum_move(41.0, 302.0)
        t = self.drum.move(t + 0.2, tt, dd)
        self.events["base_wrap_done"] = t
        tt, dd = drum_move(302.0, 314.0)
        t = self.drum.move(t + 0.2, tt, dd)
        self.events["base_wrapped_park"] = t
        t = max(t + 0.5, t_anchored + 0.5)
        self.events["tidy_start"] = t
        return t

    def takeup_speed(self):
        """tidy take-up line speed (m/s): wiper at duty 0.6 against the FIRM slip clutch, spool 2.5 x the wiper"""
        rate = 0.6 * WIPER_N0 - WIPER_N0 / WIPER_TS * (5.0 * 0.0274 * 2.5 + FRICTION)
        return math.radians(rate) * 2.5 * 0.0274

    def post_landing_tail(self, t, dur):
        """tidy for dur s, REEL fly-in, then the tow starts: returns the tow start"""
        # tidy: wiper forward in NEUTRAL at duty 0.6 against the FIRM slip clutch (5 N at the line, 2.5 x overdrive)
        w0 = self.wiper.V[-1]
        rate = 0.6 * WIPER_N0 - WIPER_N0 / WIPER_TS * (5.0 * 0.0274 * 2.5 + FRICTION)
        tt, ww = motor_move(w0, w0 + rate * dur, lambda x: 5.0 * 0.0274 * 2.5 + FRICTION, duty=0.6)
        t = self.wiper.move(t, tt, ww)
        self.events["tidy_done"] = t
        # REEL fly-in: drum 314 -> 347 while the wiper creeps forward 6 deg (dog bite)
        t += 0.4
        self.events["reel_engage"] = t
        tt, dd = drum_move(314.0, 347.0)
        te = self.drum.move(t, tt, dd)
        w0 = self.wiper.V[-1]
        tt, ww = motor_move(w0, w0 + 6.0, lambda x: FRICTION, duty=0.1)
        self.wiper.move(te - tt[-1], tt, ww)
        t = te + 0.4
        self.events["tow_start"] = t
        self.t_tow = t
        return t

    def tow(self, t0, take_deg_fn, t1):
        """capstan (REEL gear, = wiper in REEL) follows the take-in the tension needs, from t0 to t1"""
        ts = np.arange(t0, t1 + 1e-9, 0.01)
        w0, r0 = self.wiper.V[-1], self.reel.V[-1]
        self.wiper.hold_until(t0)
        self.reel.hold_until(t0)
        for t in ts[1:]:
            d = take_deg_fn(t)
            self.wiper.T.append(t)
            self.wiper.V.append(w0 + d)
            self.reel.T.append(t)
            self.reel.V.append(r0 + d)

    def state(self, t, aim_pitch, S_cocked):
        """base master state at real time t"""
        B = self.B
        if self.t_fire0 <= t < self.t_fire_end:
            return None                                          # the shot: make_motion uses fire_state
        w = self.wiper.at(t)
        d = self.drum.at(t)
        sleeve = w if t <= getattr(self, "sleeve_freeze", 1e9) else self.wiper.at(self.sleeve_freeze)
        S = dict(t=t, wiper_deg=w, sleeve_deg=sleeve, aim_deg=aim_pitch, drum_deg=d, reel_deg=self.reel.at(t),
                 line_out_m=0.0, dart_flight=0.0, dart_return=0.0, anchored=0, wraps=0.0)
        ev = self.events
        if t < ev["aim_start"]:
            S.update(phase="load", cock=0.0, fired=0)
        elif t < ev["cock_start"]:
            S.update(phase="aim", cock=0.0, fired=0)
        elif t < self.t_fire0:
            c = min(max((w - self.wc0) / 425.0, 0.0), 1.0)
            S.update(phase="cock" if c < 1.0 - 1e-9 else "cocked", cock=c, fired=0)
        else:
            S.update(phase="flight" if t < ev.get("base_rail_low_start", 1e9) else "after", cock=0.0, fired=1)
        if d < 8.0:
            S.update(mode="LAUNCH", drum_pos="LAUNCH")
        elif d < 50.0:
            S.update(mode="NEUTRAL", drum_pos="NEUTRAL")
        elif d < 340.0:
            S.update(mode="NEUTRAL", drum_pos="BETWEEN" if d < 310 else "WRAPPED")
        else:
            S.update(mode="REEL", drum_pos="REEL")
        return S
