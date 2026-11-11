"""Launch -> free flight -> sim start state, for one set of launch inputs (rail pitch, muzzle speed, tip-off pitch / yaw
rate, initial pitch / yaw offset, cross-wind). Shared by search_landing.py (the search) and make_motion.py (the film).

The dart leaves the base carriage at the rail pitch (the base's own posed dart frame), slides on the cradle at the
muzzle speed until its CG passes the lip roller, then flies (dart_motion.Flight3D: flight34 physics in 3D, no roll
mechanism) until the lowest point of the folded dart is HANDOVER_Z above the ground. That state (pose, velocity,
angular velocity) is the righting sim's start: no blend, no artificial roll."""
from __future__ import annotations

import math

import numpy as np

import base_motion as BM
import dart_motion as DM

HANDOVER_Z = 0.02          # m: lowest point of the folded dart above the ground when the sim takes over (free fall to contact)


class LaunchGeom:
    """the dart at HOME on the carriage at 34 deg (base kinematics), turned about the main axis for other pitches"""

    def __init__(self, B, S_cocked, CGX):
        place = [np.array(p["place"], float) for p in B.asm["parts"] if p["name"] == "launch_dart"][0]
        # canonical state: no sleeve given, so kin takes the rail pitch as given (34.0 exactly): the same geometry
        # whatever aim the BaseKin was built for (the search and the film agree to the last bit)
        S_home = dict(aim_deg=34.0, cock=0.0, fired=1, phase="fire", launch_travel=0.0, launch_gear_deg=B.g_rel,
                      drum_deg=0.0, reel_deg=0.0, line_out_m=0.0, wraps=0.0, mode="LAUNCH")
        JW = B.joint_world(S_home)
        self.W34 = BM.to_world_m(JW["launch_dart_rot"] @ place)
        self.pitch34 = B.pitch(S_home)
        self.axis_pt = BM.pts_world_m(np.array([0.0, B.n["Y0"], B.n["axis_z"]]))
        n = B.n
        self.u_cg_home = n["u_tail_home"] + 152.0 - CGX * 1000.0
        self.u_sep = B.MZ.U_R + n["u_c_home"]
        self.slide = (self.u_sep - self.u_cg_home) / 1000.0       # m along the rail, CG from HOME to the lip roller
        self.B = B

    def home(self, pitch):
        """dart frame (4x4 world m) at HOME on the carriage at this rail pitch"""
        d = math.radians(pitch - self.pitch34)
        R = DM.Ry(-d)
        M = self.W34.copy()
        M[:3, :3] = R @ M[:3, :3]
        M[:3, 3] = self.axis_pt + R @ (M[:3, 3] - self.axis_pt)
        return M

    def u_dir(self, pitch):
        p = math.radians(pitch)
        return np.array([math.cos(p), 0.0, math.sin(p)])


class Case:
    def __init__(self, geom, inp, cloud, FL=None):
        self.inp = dict(inp)
        p, v0 = inp["rail_deg"], inp["v0"]
        self.Wh = geom.home(p)
        self.R_rail = self.Wh[:3, :3]
        self.u = geom.u_dir(p)
        self.slide = geom.slide
        FL = DM.Flight3D(v0=v0, q0=inp["q0"], r0=inp["r0"], dth=inp["dth"], dpsi=inp["dpsi"], wind_y=inp["wind_y"])
        self.FL = FL
        self.cg_d = FL.cg.copy()                                   # mass34 CG in the dart frame (m)
        root_sep = self.Wh[:3, 3] + self.u * self.slide
        cg_sep = root_sep + self.R_rail @ self.cg_d
        self.root_sep = root_sep
        cl = cloud

        def stop(t, x, th, psi):
            if x[2] > 0.35:
                return False
            R = DM.R_flight(th, 0.0, psi)
            z = (cl @ R.T)[:, 2] + (x[2] - (R @ self.cg_d)[2])
            return float(z.min()) <= HANDOVER_Z
        self.rows = FL.run3(cg_sep, p, stop)
        self.t_fl = float(self.rows[-1, 0])

    def pose(self, tf_):
        """dart:root (p, R) at flight time tf_ (0 = leaving the cradle)"""
        r = self.rows
        c = [np.interp(tf_, r[:, 0], r[:, j]) for j in range(1, 6)]
        R = DM.R_flight(c[3], 0.0, c[4])
        cg = np.array(c[:3])
        return cg - R @ self.cg_d, R

    def end_state(self):
        r = self.rows[-1]
        th, psi = r[4], r[5]
        R = DM.R_flight(th, 0.0, psi)
        cg = r[1:4]
        v = r[6:9]
        w = DM.Flight3D.omega(th, psi, r[9], r[10])
        root = cg - R @ self.cg_d
        return dict(root=root, R=R, cg=cg, v=v, w=w)

    def sim_init(self, CGX, reel_world):
        """sim start state (sim world = C^T world: x launch direction, y up, z right; same origin)"""
        e = self.end_state()
        C = DM.C_SIM
        R_body = C.T @ e["R"] @ DM.M_D2S.T
        p_body = C.T @ (e["root"] + e["R"] @ np.array([CGX, 0.0, 0.0]))
        return dict(p=p_body.tolist(), q=DM.R2q(R_body).tolist(), v=(C.T @ e["v"]).tolist(), w=(C.T @ e["w"]).tolist(),
                    cg=(C.T @ e["cg"]).tolist(), reel=(C.T @ np.asarray(reel_world, float)).tolist())

    def summary(self):
        e = self.end_state()
        r = self.rows[-1]
        path = math.degrees(math.atan2(r[8], math.hypot(r[6], r[7])))
        return dict(t_flight=round(self.t_fl, 4), cg=np.round(e["cg"], 4).tolist(), speed=round(float(np.linalg.norm(e["v"])), 3),
                    path_deg=round(path, 2), body_pitch_deg=round(math.degrees(r[4]), 2), heading_deg=round(math.degrees(r[5]), 2),
                    pitch_rate=round(float(r[9]), 3), yaw_rate=round(float(r[10]), 3))
