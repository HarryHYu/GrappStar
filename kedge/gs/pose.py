"""Kinematics of the ground station. State -> 4x4 transform per frame. Shared by the audit, the tables and the sim.
Rotation convention: ry(a) rotates about +Y by a degrees CCW in the (x, z) plane (x toward z), i.e. nose up."""
import json, math
import numpy as np
from shapely.geometry import Point, Polygon
from shapely import affinity


def ry(a_deg, cx=0.0, cz=0.0):
    a = math.radians(a_deg); c, s = math.cos(a), math.sin(a)
    M = np.eye(4)
    # CCW in (x, z): x' = c x - s z, z' = s x + c z
    M[0, 0], M[0, 2], M[2, 0], M[2, 2] = c, -s, s, c
    T1 = np.eye(4); T1[0, 3], T1[2, 3] = cx, cz
    T0 = np.eye(4); T0[0, 3], T0[2, 3] = -cx, -cz
    return T1 @ M @ T0


def tr(x=0.0, y=0.0, z=0.0):
    M = np.eye(4); M[:3, 3] = (x, y, z); return M


def _interp(tab, x):
    if x <= tab[0][0]: return tab[0][1]
    if x >= tab[-1][0]: return tab[-1][1]
    lo, hi = 0, len(tab) - 1
    while hi - lo > 1:
        m = (lo + hi) // 2
        if tab[m][0] <= x: lo = m
        else: hi = m
    (x0, y0), (x1, y1) = tab[lo], tab[hi]
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0)


def sear_angle(P, th, s, keel=True):
    """sear angle (deg, negative = hook down) from the mesh-solved tables: keel contact and trip cord, whichever is lower"""
    d = _interp(P["SEAR_CORD_TAB"], th)
    if keel:
        kt = P["SEAR_KEEL_TAB"]
        if kt[0][0] <= s <= kt[-1][0]:          # below the table start the keel rests on the hook face: no push
            d = min(d, _interp(kt, s))
    return d


def creep(P, d):
    """carriage pushed back (mm) by the hook face when the sear turns d while holding the carriage"""
    ct = P["CREEP_TAB"]
    return _interp(sorted(ct), d) if d < 0 else 0.0


class Kin:
    def __init__(self, P):
        self.P = P
        self.r_pg = P["M"] * P["Z_PG"] / 2

    def sled_pos(self, phi, fired):
        """pusher face u from the exact rack push table (covers phi -40 .. release); after release it sits on the
        sear 0.5 mm forward of the peak; after the shot it is forward (P_R) until a tooth meets the rack again."""
        P = self.P; tab = P["RACK_TAB"]
        phi %= 360
        x = phi - 360 if phi >= 360 + tab[0][0] else phi
        if tab[0][0] <= x < P["PH_REL"]:
            i = min(len(tab) - 2, int((x - tab[0][0]) / 0.5)); (p0, d0), (p1, d1) = tab[i], tab[i + 1]
            return P["P_R"] - (d0 + (d1 - d0) * (x - p0) / (p1 - p0))
        if fired or x < tab[0][0]:
            return P["P_R"]
        return P.get("S_SETTLED", P["P_R"] - (P["STROKE"] - P.get("OVERTRAVEL", 0.5)))   # resting on the hook face (mesh solved)

    def hook_creep(self, d_deg):
        """the hook face swings back as the sear turns (d < 0): it pushes the carriage back this much (mm)"""
        P = self.P; sp = P["SEAR_PIV"]; hk = P["HOOK"]; a = math.radians(d_deg); c = 0.0
        for w in np.linspace(P["KEEL"][4], hk[3], 9):
            du, dw = hk[0] - sp[0], w - sp[1]
            u = sp[0] + du * math.cos(a) - dw * math.sin(a)
            wn = sp[1] + du * math.sin(a) + dw * math.cos(a)
            if wn > P["KEEL"][4] - 0.01:
                c = max(c, hk[0] - u)
        return c

    def frames(self, st):
        """st: dict(theta, phi, alpha (main shaft CCW deg), s (pusher u), rot (rotor rel deg), drag (deg),
        pin (0 in .. 1 out), sear (deg), dart (4x4 in world or None for 'on the sled'))"""
        P = self.P; H = P["H"]
        th = st["theta"]
        Rr = tr(0, 0, H) @ ry(th)                        # rail frame -> world
        F = {"world": np.eye(4), "rail": Rr}
        F["pg"] = Rr @ tr(P["PG_U"], 0, 0) @ ry(st["phi"])
        F["main"] = tr(0, 0, H) @ ry(st["alpha"])
        F["main_pin"] = tr(0, 0, H) @ ry(th - st["phi"] * P["Z_PG"] / P["Z_PIN"] + P.get("PIN_PHASE", 0))
        F["sled"] = Rr @ tr(st["s"], 0, 0)
        F["reel"] = Rr @ tr(P["REEL_U"], P["REEL_FACE_Y"], P["DART_W"])
        F["rotor"] = F["reel"] @ ry(st["rot"])
        F["drag"] = F["reel"] @ ry(st["drag"])
        F["pin"] = F["rotor"] @ tr(-3.5 * (1 - st["pin"]), 0, 0)
        sp = P["SEAR_PIV"]
        F["sear"] = Rr @ ry(st["sear"], sp[0], sp[1])
        # dart on the sled: tail face at the pusher, nose forward: dart (x,y,z) -> rail (s + L - x, -y, DW + z)
        L = P["DART_L"]
        Dm = np.array([[-1, 0, 0, st["s"] + L], [0, -1, 0, 0], [0, 0, 1, P["DART_W"]], [0, 0, 0, 1]], float)
        F["dart"] = st["dart"] if st.get("dart") is not None else Rr @ Dm
        for g, pv in P.get("DART_PIV", {}).items():
            ang = st.get("kick", 0.0) if "kick" in g else -st.get("dep", 0.0)
            F[g] = F["dart"] @ tr(*pv) @ ry(ang)
        return F


def theta_table(P, cam_poly, n=720):
    """Real rail angle vs phi: the rail sits on the bumper at TH_LOW or on the cam, whichever is higher."""
    rx, rz, rr = P["ROLL_X"], P["ROLL_Z"], P["ROLL_R"]
    out = []
    for i in range(n + 1):
        phi = 360.0 * i / n
        lo, hi = P["TH_LOW"], P["TH_FIRE"] + 5
        def pen(th):
            t = math.radians(th); z = rz - P["H"]
            u = rx * math.cos(t) + z * math.sin(t); w = -rx * math.sin(t) + z * math.cos(t)
            du, dw = u - P["PG_U"], w
            a = -math.radians(phi)
            cu, cw = du * math.cos(a) - dw * math.sin(a), du * math.sin(a) + dw * math.cos(a)
            return cam_poly.distance(Point(cu, cw)) < rr - 1e-3 or cam_poly.contains(Point(cu, cw))
        if not pen(lo):
            out.append((phi, lo)); continue
        a, b = lo, hi
        for _ in range(40):
            m = 0.5 * (a + b)
            if pen(m): a = m
            else: b = m
        out.append((phi, b))
    return out
