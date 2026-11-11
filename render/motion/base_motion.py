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
    def fire_state(self, S_cocked, tau, trace, w_rate=270.0, w_rate_loaded=225.0, stop=None):
        """State during the shot, tau (s) after the wiper passes full cock (425 fwd). The relieved last tooth lets go at
        wiper 429.6 fwd (gear -281.25, release table); the carriage then follows the launcher physics trace
        (base/physics launcher.py, t, x) home, overshoots into the PLA stop flexure and settles; the wiper keeps turning
        and carries the big gear on through its gap to rest (-360) at 553.8 fwd (base TIMELINE). Returns (S, info)."""
        wc0, ratio = self.wc0, self.ratio
        w_rel = wc0 + (self.g0 - self.g_rel) * ratio            # 429.6 fwd
        t_rel = (w_rel - (wc0 + 425.0)) / w_rate_loaded
        w_end = wc0 + 553.8
        S = dict(S_cocked)
        S["phase"] = "fire"
        S["fired"] = 1
        S["cock"] = 1.0
        info = dict(t_rel=t_rel)
        if tau < t_rel:
            w = wc0 + 425.0 + w_rate_loaded * tau
            g = self.g0 - (w - wc0) / ratio
            xs, ys = -np.asarray(self.rt["theta"]), np.asarray(self.rt["travel"])
            k = int(np.clip(np.searchsorted(xs, -g), 1, len(xs) - 1))
            travel = float(max(ys[k - 1], ys[k]))
            travel = min(travel, self.stroke)
            S.update(wiper_deg=w, sleeve_deg=w, launch_gear_deg=g, launch_travel=travel)
            info.update(stage="release", travel=travel, dart_u=0.0)
            return S, info
        ts = tau - t_rel
        w = min(w_end, w_rel + w_rate * ts)
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
        if ts >= T_st and w >= w_end:
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
