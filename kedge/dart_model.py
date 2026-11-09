"""KEDGE dart v31 physics: shared model (mass properties, aero profiles, motors, soils, base-station numbers).

Every assumption is a named constant here so a measured value replaces it in one place. Used by physics.py,
flight_sim.py, dart_landing.py, dart_anchor.py and dart_phys31.py.

Frames: dart body frame = CAD frame: nose tip x = 0, tail x = 152, y left (kick side), z up, hull axis y = z = 0,
belly z = -21 lying flat. The dart flies nose first, toward body -x. Units: SI unless a name says mm or g.
"""
import json, math, os, sys
import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix as RM, translation_matrix as TM

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
BASE_PHYS = os.path.join(ROOT, "base", "physics")
G = 9.81
RHO_AIR = 1.20

# ------------------------------------------------------------------------------------------------ mass model
RHO_PLA = 1.24e-3            # g/mm3
WALL = 1.7                   # mm, 4 perimeters x 0.42
INFILL = 0.40
BOUGHT_G = {                 # grams, catalogue / bench values (WEIGH on the day; a part's "mass_g" in parts.json overrides)
    "n20_kick": 10.5, "n20_deploy": 10.5, "n20_foot": 10.5, "drill_leg_L_motor": 11.0, "drill_leg_R_motor": 11.0,
    "cell_1": 22.0, "cell_2": 22.0, "xiao": 3.0, "drv8833_1": 1.8, "drv8833_2": 1.8, "drv8833_3": 1.8, "ina219": 2.2,
    "bno085": 2.6, "mp1584": 2.0, "vtx": 3.6, "bms_2s": 1.5, "camera": 2.2,
    "drill_leg_L_brg1": 0.8, "drill_leg_L_brg2": 0.8, "drill_leg_R_brg1": 0.8, "drill_leg_R_brg2": 0.8,
    "drill_leg_L_nut": 0.5, "drill_leg_R_nut": 0.5, "drill_leg_L_nyloc": 0.6, "drill_leg_R_nyloc": 0.6,
    "brg_ear": 0.8, "screws_radial": 3.5, "screw_ear_root": 0.8, "kick_hub_screw": 0.7, "kick_washer": 0.1,
    "coupler_L_screw": 0.6, "coupler_L_washer": 0.1, "coupler_R_screw": 0.6, "coupler_R_washer": 0.1,
    "contacts_front_springs": 1.0, "contacts_rear_plates": 0.6, "jst_charge": 0.8, "jst_loop": 1.2, "antenna": 0.6,
    "capacitor": 1.0, "xiao_antenna": 0.6,
}
# shafts are inside the motor masses; the ballast is set by ballast_g (its mesh gives the position)
ZERO_G = {"drill_leg_L_shaft", "drill_leg_R_shaft", "kick_shaft", "deploy_shaft", "foot_shaft", "ballast"}
EXTRA = [("wiring + solder", 12.0, (85.0, 0.0, 0.0)), ("glue, tape, heat shrink, zip ties", 3.0, (90.0, 0.0, 0.0)),
         ("tether knot and 0.2 m of line inside", 0.5, (147.0, 0.0, 0.0))]
BALLAST_G_DEFAULT = 15.0     # CLAUDE.md section 3 (steel shot in epoxy, fills the nose tip to x 13.7)


def out_dir(prefer=None):
    """kedge/out31 when dart-cad has written parts.json there, else out30 (v30 geometry = v31 start point)."""
    for d in ([prefer] if prefer else []) + [os.path.join(HERE, "out31"), os.path.join(HERE, "out30")]:
        if d and os.path.exists(os.path.join(d, "parts.json")) and os.path.exists(os.path.join(d, "hull_top.npz")):
            return d
    raise SystemExit("no parts.json with meshes in out31 or out30")


class Dart:
    """Geometry + mass model of one dart build (an OUT folder of the kedge pipeline)."""

    def __init__(self, out=None, ballast_g=BALLAST_G_DEFAULT, ballast_x=None):
        self.out = out_dir(out)
        J = json.load(open(os.path.join(self.out, "parts.json")))
        self.J, self.P, self.ground = J, J["P"], J.get("ground", -21.0)
        self.PP = {p["name"]: p for p in J["parts"]}
        self.kids = {}
        for n, p in self.PP.items():
            if p.get("parent"):
                self.kids.setdefault(p["parent"], []).append(n)
        self._mesh = {}
        self.ballast_g = self.P.get("BALLAST_G", ballast_g) if ballast_g == BALLAST_G_DEFAULT else ballast_g   # dart-cad's value wins unless set
        self.ballast_x = ballast_x            # None = ballast mesh centroid
        self.fold = self.assembly(0.0, 0.0)
        self.mp = self.mass_props(self.fold)
        # tether attachment: the eye of a tail boom when the build has one (v32), else the tail cap knot on the axis
        tp = [n for n in self.PP if any(k in n for k in ("eye", "boom")) and not self.PP[n].get("parent")]
        if tp:
            v = np.vstack([self.mesh(n).vertices for n in tp])
            far = v[v[:, 0] > v[:, 0].max() - 3.0]
            self.tether_pt = (float(v[:, 0].max()), float(far[:, 2].mean()))
        else:
            self.tether_pt = (float(self.P.get("L", 152.0)), 0.0)

    def fins(self):
        """Fins (parts named *fin*): x centre, area, span, plane ('pitch' = horizontal fin, 'yaw' = vertical) and the
        normal-force slope C = CNa x area (m2) per fin from the panel-pair formula 2 pi AR / (2 + sqrt(4 + AR^2)),
        AR = 2 s^2 / A, with the body interference 1 + r / (s + r)."""
        out = []
        R = self.P["R_OUT"]
        for n, m in self.fold:
            if "fin" not in n or n.endswith("_fine"):
                continue
            v = m.vertices
            ey, ez = np.ptp(v[:, 1]), np.ptp(v[:, 2])
            r = np.hypot(v[:, 1], v[:, 2])
            s_ = max(r.max() - R, 1.0) / 1000
            A = m.area / 2 * 1e-6
            AR = 2 * s_ ** 2 / A
            K = 1 + (R / 1000) / (s_ + R / 1000)
            C = 2 * math.pi * AR / (2 + math.sqrt(4 + AR ** 2)) * A * K
            out.append(dict(name=n, x_mm=float(m.center_mass[0] if m.is_watertight else m.bounds.mean(0)[0]), x0=float(v[:, 0].min()), x1=float(v[:, 0].max()),
                            span_mm=s_ * 1000, area_mm2=A * 1e6, plane="pitch" if ey > ez else "yaw", C_m2=C))
        return out

    # ---------------------------------------------------------------- geometry
    def mesh(self, n):
        if n not in self._mesh:
            z = np.load(os.path.join(self.out, n + ".npz"))
            self._mesh[n] = trimesh.Trimesh(z["v"], z["f"], process=False)
        return self._mesh[n]

    def fam(self, r):
        o = [r]
        for k in self.kids.get(r, []):
            o.extend(self.fam(k))
        return o

    def sgn(self, n):
        """Rotation sign about +y of a moving root: parts.json "sgn" (v34 on), else the v31-v33 rule (kick -1, pods +1)."""
        return self.PP[n].get("sgn", -1 if n == "kick_leg" else 1)

    def is_leg(self, n):
        """True for the kick / righting leg root (parts.json "joint" == "leg" from v34 on), False for the deploy group."""
        return n == "kick_leg" or self.PP[n].get("joint") == "leg"

    def T(self, n, d):
        return TM(self.PP[n]["pos"]) @ RM(math.radians(self.sgn(n) * d), [0, 1, 0])

    def assembly(self, kick_deg, dep_deg):
        """[(name, mesh in the dart frame)] at a kick angle and a drill deploy angle (deg)."""
        items = []
        for n, p in self.PP.items():
            if p.get("parent"):
                continue
            if p["axis"]:
                Tm = self.T(n, kick_deg if self.is_leg(n) else dep_deg)
                for c in self.fam(n):
                    items.append((c, self.mesh(c).copy().apply_transform(Tm)))
            else:
                items.append((n, self.mesh(n)))
        return items

    def part_mass(self, n, m):
        p = self.PP.get(n, {})
        if p.get("mass_g") is not None:
            return float(p["mass_g"])
        if n == "ballast":
            return self.ballast_g
        if p.get("printable"):
            V = m.volume
            shell = min(V, m.area * WALL)          # perimeters on every face, the rest at INFILL (as v29/v30)
            return RHO_PLA * (shell + INFILL * max(0.0, V - shell))
        if n in ZERO_G:
            return 0.0
        return BOUGHT_G.get(n, 0.0)

    def mass_props(self, items):
        """Mass (g), CG (mm), inertia about the CG (g mm2, own inertia of every part included), rows."""
        rows = []
        for n, m in items:
            mm = self.part_mass(n, m)
            if mm <= 0:
                continue
            solid = m.is_watertight and m.volume > 1
            c = np.array(m.center_mass if solid else m.bounds.mean(0))
            if n == "ballast" and self.ballast_x is not None:
                c = np.array([self.ballast_x, 0.0, 0.0])
            Io = (m.moment_inertia * mm / m.volume) if solid else np.zeros((3, 3))
            rows.append((n, mm, c, Io))
        for nm, mm, c in EXTRA:
            rows.append((nm, mm, np.array(c, float), np.zeros((3, 3))))
        M = sum(r[1] for r in rows)
        cg = sum(r[1] * r[2] for r in rows) / M
        I = np.zeros((3, 3))
        for n, mm, c, Io in rows:
            d = c - cg
            I += Io + mm * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
        printed = sum(r[1] for r in rows if self.PP.get(r[0], {}).get("printable"))
        return dict(M=M, cg=cg, I=I, rows=rows, printed=printed, bought=M - printed - (self.ballast_g if any(r[0] == "ballast" for r in rows) else 0.0))

    def points(self, kick_deg=0.0, dep_deg=0.0, n_per=1500, seed=1, dens=0.05):
        """Surface sample of the assembly (mm, dart frame) for silhouettes and ground contact."""
        rng = np.random.default_rng(seed)
        pts = []
        for n, m in self.assembly(kick_deg, dep_deg):
            k = int(np.clip(m.area * dens, 50, n_per))
            if m.area > 20:
                p, _ = trimesh.sample.sample_surface(m, k, seed=int(rng.integers(1 << 30)))
                pts.append(p)
            pts.append(m.vertices[:: max(1, len(m.vertices) // 200)])
        return np.vstack(pts)

    def hull_points(self, kick_deg=0.0, dep_deg=0.0):
        P = self.points(kick_deg, dep_deg)
        h = trimesh.convex.convex_hull(P)
        return np.asarray(h.vertices), h

    # ---------------------------------------------------------------- aero profiles (folded)
    def profiles(self, pitch_mm=1.0):
        """Per x station (1 mm): top-view width (y extent, sees flow along z), side height (z extent, sees flow along y),
        their centroids, and the filled cross-section area S(x) (all parts, hull interior counted solid)."""
        from scipy.ndimage import binary_closing, binary_fill_holes
        P = self.points(0, 0, n_per=80000, dens=3.0)
        x0, x1 = -2.0, float(P[:, 0].max()) + 2
        xs = np.arange(x0, x1, pitch_mm)
        ix = np.clip(np.round((P[:, 0] - x0) / pitch_mm).astype(int), 0, len(xs) - 1)
        lo, n = -100.0, 200
        iy = np.clip(np.round(P[:, 1] - lo).astype(int), 0, n - 1)
        iz = np.clip(np.round(P[:, 2] - lo).astype(int), 0, n - 1)
        vox = np.zeros((len(xs), n, n), bool)
        vox[ix, iy, iz] = True
        yy, zz = np.meshgrid(np.arange(n) + lo, np.arange(n) + lo, indexing="ij")
        rr = np.hypot(yy, zz)
        rhull = np.zeros(len(xs))                     # outer radius of the hull shell per station (nose, hull, tail cap)
        for hn in ("hull_top", "hull_bottom", "nose", "tail_cap"):
            if hn in self.PP:
                v = self.mesh(hn).vertices
                hp, _ = trimesh.sample.sample_surface(self.mesh(hn), 40000, seed=3)
                v = np.vstack([v, hp])
                j = np.clip(np.round((v[:, 0] - x0) / pitch_mm).astype(int), 0, len(xs) - 1)
                r_ = np.hypot(v[:, 1], v[:, 2]); k_ = r_ <= self.P["R_OUT"] + 0.3      # skin only (not saddle, ear, bosses)
                np.maximum.at(rhull, j[k_], r_[k_])
        S = np.zeros(len(xs)); top = np.zeros(len(xs)); side = np.zeros(len(xs)); yc = np.zeros(len(xs)); zc = np.zeros(len(xs))
        topimg = np.zeros((len(xs), n), bool); sideimg = np.zeros((len(xs), n), bool)
        for i, x in enumerate(xs):
            sl = binary_closing(vox[i], structure=np.ones((3, 3)), iterations=2)
            if rhull[i] > 0:
                sl |= rr <= rhull[i]
            sl = binary_fill_holes(sl)
            S[i] = sl.sum()
            topimg[i] = sl.any(1); sideimg[i] = sl.any(0)
        topimg = binary_fill_holes(binary_closing(topimg, np.ones((3, 3))))
        sideimg = binary_fill_holes(binary_closing(sideimg, np.ones((3, 3))))
        ax = np.arange(n) + lo
        for i in range(len(xs)):
            top[i] = topimg[i].sum(); side[i] = sideimg[i].sum()
            yc[i] = (ax * topimg[i]).sum() / top[i] if top[i] else 0.0
            zc[i] = (ax * sideimg[i]).sum() / side[i] if side[i] else 0.0
        return dict(rhull=rhull, xs=xs, top=top, side=side, yc=yc, zc=zc, S=S, topimg=topimg, sideimg=sideimg, ax=ax)


# ------------------------------------------------------------------------------------------------ base station numbers
def base_params():
    """base/physics Params overlaid with base/spec/params.py (the real launcher, reel and line)."""
    if BASE_PHYS not in sys.path:
        sys.path.insert(0, BASE_PHYS)
    from common import load_params
    return load_params()


def base_spec():
    import importlib.util
    spec = importlib.util.spec_from_file_location("base_spec_params", os.path.join(ROOT, "base", "spec", "params.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def launch_trace(pitch_deg=34.0, dart_mass_kg=None):
    """Real launcher shot (base/physics/launcher.py chain-spring model) for the dart mass: carriage speed trace,
    muzzle speed and the line pull while the reel spins up."""
    p = base_params()
    if dart_mass_kg is not None:
        p.dart.mass = dart_mass_kg
    from launcher import launch, reel_line_load
    r = launch(p, pitch_deg, record=True)
    tr = np.array([(t, x, v) for t, x, v, *_ in r["trace"]], float)
    return p, r, tr, reel_line_load(p)


# ------------------------------------------------------------------------------------------------ motors
# Measured at 8 V on the bench (CLAUDE.md section 2): worm N20 20-27 mA free, about 180 mA stall; drill pair 60-70 mA
# free, about 700 mA stall (both); foot motor about 40 mA free, 500-600 mA stall. Speeds are the 12 V listings
# (base/spec/params.py N20_WORM rpm 27, N20_STRAIGHT rpm 100). Output torque from a DC motor model: winding R = V / I_stall,
# output torque constant Kt = (V - I_free R) / w_free (gear included), torque = eta_load x Kt x (I - I_free).
MOTORS = {
    "worm": dict(rpm12=27.0, i_free=0.025, i_stall=0.180, eta=0.40, eta_rng=(0.30, 0.50),
                 note="self-locking worm (lead below the friction angle) so load efficiency under 0.5; 0.40 used"),
    "drill": dict(rpm12=100.0, i_free=0.0325, i_stall=0.350, eta=0.60, eta_rng=(0.50, 0.70),
                  note="5-stage spur N20 gearbox, about 0.9 per stage"),
    "foot": dict(rpm12=27.0, i_free=0.040, i_stall=0.550, eta=0.40, eta_rng=(0.30, 0.50),
                 note="measured as the third motor; if it really is the 27 rpm worm its stall torque is 3 x the kick worm: "
                      "MEASURE (lever + luggage scale)"),
}


BENCH_FILE = os.path.join(HERE, "bench31.json")   # written by dart_pulltest.py torque (measured values win)


def bench():
    try:
        return json.load(open(BENCH_FILE))
    except Exception:
        return {}


def motor(kind, V=7.4):
    m = MOTORS[kind]
    key = {"worm": "worm_stall_Nm", "drill": "drill_stall_Nm", "foot": "foot_stall_Nm"}[kind]
    B = bench().get(key)
    if B:                                   # measured stall torque: back out the load efficiency at that voltage
        R_ = 8.0 / m["i_stall"]; w8 = m["rpm12"] * 8.0 / 12.0 * 2 * math.pi / 60; Kt_ = (8.0 - m["i_free"] * R_) / w8
        m = dict(m, eta=B["Nm"] / (Kt_ * (B["V"] / R_ - m["i_free"])), note=m["note"] + "; eta from the bench stall torque")
    R = 8.0 / m["i_stall"]
    w8 = m["rpm12"] * 8.0 / 12.0 * 2 * math.pi / 60
    Kt = (8.0 - m["i_free"] * R) / w8
    i_st = V / R
    T = m["eta"] * Kt * (i_st - m["i_free"])
    w_free = (V - m["i_free"] * R) / Kt
    return dict(kind=kind, V=V, R_ohm=round(R, 1), Kt_out_NmA=round(Kt, 3), stall_A=round(i_st, 3),
                stall_Nm=round(T, 3), stall_Nm_range=[round(e * Kt * (i_st - m["i_free"]), 3) for e in m["eta_rng"]],
                free_rpm=round(w_free * 60 / (2 * math.pi), 1), free_rad_s=round(w_free, 3), eta=m["eta"], note=m["note"])


def motor_current_for(kind, torque_Nm, V=7.4):
    m = MOTORS[kind]
    R = 8.0 / m["i_stall"]
    w8 = m["rpm12"] * 8.0 / 12.0 * 2 * math.pi / 60
    Kt = (8.0 - m["i_free"] * R) / w8
    return m["i_free"] + torque_Nm / (m["eta"] * Kt)


# ------------------------------------------------------------------------------------------------ soils
# gamma kN/m3, phi deg, c (cohesion or undrained shear strength) kPa, delta = soil-PLA interface friction deg,
# q_dyn: dynamic bearing pressure under a fast blunt nose (penetrometer-type values, kPa), mu: sliding friction
# of the hull on the surface, k_s: subgrade modulus (MN/m3) for the impact contact. Reviewed values:
#  firm clay: BS EN ISO 14688-2 "firm" is cu 40 to 75 kPa (v29 used 30: that is "soft"); 40 used (low end).
#  moist loam: silty sandy topsoil, phi 28 to 30, apparent cohesion from suction 3 to 10 kPa; 5 used.
#  dry sand: loose to medium, phi 30 to 34; 32 used (Kp 3.25).
SOIL = {
    "dry sand":   dict(gamma=16.0, phi=32.0, c=0.0, delta=22.0, q_dyn=250.0, mu=0.55, k_s=20.0, kind="sand"),
    "moist loam": dict(gamma=18.0, phi=28.0, c=5.0, delta=20.0, q_dyn=600.0, mu=0.55, k_s=40.0, kind="c-phi"),
    "firm clay":  dict(gamma=19.0, phi=0.0, c=40.0, delta=0.0, q_dyn=1000.0, mu=0.45, k_s=60.0, kind="clay"),
}
SURFACE = {   # impact surfaces: bearing pressure the nose sees (kPa) and the contact stiffness
    "soft soil (loam, tilled)": dict(q=600.0, k_s=40.0, mu=0.55, e=0.05),
    "grass on loam": dict(q=400.0, k_s=25.0, mu=0.45, e=0.15),
    "gravel (10 to 20 mm)": dict(q=3000.0, k_s=200.0, mu=0.60, e=0.25),
    "hard ground (dry compacted, concrete)": dict(q=60000.0, k_s=30000.0, mu=0.40, e=0.30),
}
PLA = dict(sigma_t=45e6, sigma_t_layers=25e6, sigma_c=55e6, tau=25e6, E=3.3e9)
