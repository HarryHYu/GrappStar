"""Full mission timeline for the HD sim, computed from the mechanism and physics (not hand animated).
python timeline.py out -> out/timeline.json
Every sample: real time, sim time, phase, phi, theta, alpha (main shaft), s, rotor, pin, sear, dart pose, kick, dep,
line out, tension, rover x, motor dir/rpm/current/torque."""
import json, math, sys
import numpy as np
sys.path.insert(0, ".")
from pose import Kin

OUT = sys.argv[1]
J = json.load(open(f"{OUT}/gs_parts.json")); P = J["P"]
PH = json.load(open(f"{OUT}/physics_gs.json"))
theta_tab = [t for _, t in json.load(open(f"{OUT}/theta_table.json"))["theta"]]
DG = json.load(open(f"{OUT}/dart_groups.json"))
P["DART_KICK_DEG"] = DG["_P"]["KICK_DEG"]; P["DART_DEPLOY_DEG"] = DG["_P"]["DEPLOY_DEG"]
K = Kin(P)
G = 9.81
i_pg = P["Z_PG"] / P["Z_PIN"]; ratio_reel = P["Z_MAIN"] / P["Z_STUB"]
W = PH["motor"]["wiper_assumed"]
ETA = 0.92
DECK = 150.0          # rover deck height: the base underside sits on it


def theta_of(phi):
    n = len(theta_tab) - 1; x = (phi % 360) / 360 * n; i = int(x); f = x - i
    return theta_tab[i] * (1 - f) + theta_tab[min(n, i + 1)] * f


def motor(Tm):
    Tm = max(0.0, Tm)
    n = W["n0"] * max(0.02, 1 - Tm / W["Ts"]); I = W["I0"] + (W["Is"] - W["I0"]) * min(1, Tm / W["Ts"])
    return n, I


# PG torque map (cocking + lift) from the physics cycle
load = np.zeros(721)
for phi, T, k in PH["cycle"]:
    j = int(round((phi % 360) * 2)) % 720
    load[j] = max(load[j], T)
for j in range(720):                       # fill holes
    if load[j] == 0:
        load[j] = max(load[j - 1], 0.0) if j % 2 else load[j]
def T_motor_B(phi):
    return 0.35 + load[int(round((phi % 360) * 2)) % 720] / i_pg / ETA      # 0.35 N m drag (reel rotor + drag disc + bearings)


from pose import sear_angle as _SA, creep as _CREEP


def sear_angle(th, s, keel=True):
    return _SA(P, th, s, keel)


S = []          # samples
state = dict(t=0.0, ts=0.0, phase="", phi=P["PH_REL"] + 3.0, alpha=0.0, s=None, pin=0.0, drag=0.0, kick=0.0, dep=0.0,
             line_out=0.0, tension=0.0, rover=0.0, dart=None, fired=False, mdir=0, rpm=0.0, I=0.0, Tm=0.0, spool_m=20.0,
             rot_extra=0.0, line_pts=None)
state["alpha"] = theta_of(state["phi"]) - state["phi"] * i_pg
PHASES = []


def push(dt_real, compress=1.0):
    st = state
    st["t"] += dt_real; st["ts"] += dt_real / compress
    th = theta_of(st["phi"])
    s = K.sled_pos(st["phi"], st["fired"]) if st["s"] is None else st["s"]
    sa = sear_angle(th, s)
    if st["s"] is None and not st["fired"] and (st["phi"] % 360) >= P["PH_REL"] and sa < 0:
        s -= _CREEP(P, sa)
    rot = P["STUB_PHASE"] - (st["alpha"] - th) * ratio_reel
    S.append(dict(t=round(st["t"], 4), ts=round(st["ts"], 4), ph=st["phase"], phi=round(st["phi"], 3), th=round(th, 3),
                  a=round(st["alpha"], 2), s=round(s, 2), rot=round(rot % 360, 2), pin=round(st["pin"], 3), drag=round(st["drag"], 2),
                  sear=round(sa, 2), dart=st["dart"], k=round(st["kick"], 2), d=round(st["dep"], 2),
                  lo=round(st["line_out"], 3), T=round(st["tension"], 2), rv=round(st["rover"], 2), md=st["mdir"],
                  rpm=round(st["rpm"], 1), I=round(st["I"], 2), Tm=round(st["Tm"], 2), sp=round(st["spool_m"], 3),
                  lp=st["line_pts"]))


def phase(name, title, text):
    state["phase"] = name
    PHASES.append(dict(id=name, title=title, text=text, ts=round(state["ts"], 3), t=round(state["t"], 3)))


def run_B_to(phi_target, compress=1.0, dt=1 / 60, stop_cond=None):
    """motor backward: PG follows the motor through bearing A; pin retracts."""
    st = state; st["mdir"] = -1
    while st["phi"] < phi_target - 1e-6:
        Tm = T_motor_B(st["phi"]); n, I = motor(Tm)
        fpr = max(0.0, (theta_of(st["phi"] + 0.5) - theta_of(st["phi"])) / 0.5)
        dphi = n / 60 * 360 / (i_pg - fpr) * dt * compress          # planet carrier: the lifting rail eats part of the shaft turn
        dphi = min(dphi, phi_target - st["phi"])
        if stop_cond:                                   # sub-steps so the stop (the shot) lands on the right phi
            k = max(1, int(math.ceil(dphi / 0.1))); hit = False
            for _ in range(k):
                st["phi"] += dphi / k; st["alpha"] -= dphi / k * (i_pg - fpr)
                if stop_cond(): hit = True; break
            st["pin"] = max(0.0, st["pin"] - 0.25 * compress); st["rpm"], st["I"], st["Tm"] = n, I, Tm
            push(dt * compress, compress)
            if hit: break
            continue
        st["phi"] += dphi; st["alpha"] -= dphi * (i_pg - fpr)
        st["pin"] = max(0.0, st["pin"] - 0.25 * compress); st["drag"] = (st["drag"] - dphi * i_pg * ratio_reel * 0) % 360
        st["rpm"], st["I"], st["Tm"] = n, I, Tm
        push(dt * compress, compress)
        if stop_cond and stop_cond():
            break
    st["mdir"] = 0; st["rpm"] = 0; st["I"] = 0; st["Tm"] = 0


def hold(secs, compress=1.0, dt=1 / 60, fn=None):
    n = max(1, int(round(secs / (dt * compress))))
    for i in range(n):
        if fn: fn(i / max(1, n - 1))
        push(dt * compress, compress)


# --------------------------------------------------------------- world geometry helpers (mm, world frame of the rig)
def rail_world(u, w, th):
    t = math.radians(th); return np.array([u * math.cos(t) - w * math.sin(t), 0.0, P["H"] + u * math.sin(t) + w * math.cos(t)])


def eye_world(th):
    # the cover exit eye: on the reel axis, front face
    p = rail_world(P["REEL_U"], P["DART_W"], th); p[1] = P["REEL_FACE_Y"] + 4.0; return p


def muzzle_world(th):
    return rail_world(P["U_MUZ"], P["DART_W"] - 8, th)


def dart_pose_world(cg, pitch_deg, roll=0.0):
    """dart matrix: dart x from nose (0) to tail, CG at x=82.5; nose points along +X world when pitch 0."""
    xcg = PH_CG
    a = math.radians(pitch_deg); c, s_ = math.cos(a), math.sin(a)
    # dart frame -> world: x_d -> -X (nose forward), y_d -> -Y, z_d -> Z, then pitch nose up about Y
    R0 = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]], float)
    Rp = np.array([[c, 0, -s_], [0, 1, 0], [s_, 0, c]], float)
    R = Rp @ R0
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = np.array(cg) - R @ np.array([xcg, 0, 0])
    return [round(x, 3) for x in M.flatten().tolist()]


PH_CG = 82.5
try:
    PH_CG = json.load(open("../out30/physics.json"))["cg_mm"][0]
except Exception:
    pass
L_DART = P["DART_L"]
GROUND_Z = -P["BASE_T"] - DECK          # field ground in rig coords

# =============================================================== MISSION
phase("ready", "1. Ready", "Carriage cocked and held by the sear, dart seated against the pusher, rail down at 4 deg. Motor stopped: the wiper worm is self locking.")
state["phi"] = (P["PH_REL"] + P["PH_LIFT0"]) / 2; state["alpha"] = theta_of(state["phi"]) - state["phi"] * i_pg
hold(2.0)

phase("lift", "2. Fire: motor backward", "Bearing A locks: the program gear turns. Its cam lifts the rail on the frame roller, 4 to 35 deg, slowly: the rail carries the program gear round the pinion. The reel rotor turns backward too, which pulls the pickup pin in: the line is free.")
trip_phi = None
def tripped():
    return state["phi"] >= P["PH_TRIP"]          # the cord pulls the hook clear here (same number as physics and firmware)
run_B_to(P["PH_DROP1"], compress=1.0, stop_cond=tripped)
trip_phi = state["phi"]; th_fire = theta_of(trip_phi)

# ---- launch (slow motion): carriage + dart from the physics trace
phase("launch", "3. Shot", "The trip cord from the base goes taut as the rail lifts, pulls the clevis sear down until its hook clears the keel, the springs pull the carriage 208 mm in %.0f ms. Dart leaves at %.1f m/s." %
      (PH["launch"]["t_ms"], PH["launch"]["v_muzzle"]))
s_full = K.sled_pos(trip_phi, False)
trace = PH["launch_trace"]
SLOW = 40.0
tl0 = state["t"]
for tt, xx, vv in trace:
    st = state
    st["s"] = s_full + xx * 1000
    st["line_out"] = 0.0
    push(max(1e-5, tt - (st["t"] - tl0)), 1.0 / SLOW)
state["s"] = None; state["fired"] = True
# ---- flight: dart pose along the physics trajectory, line pays out (end pull), rail drops on the cam (overrun)
phase("flight", "4. Flight", "End pull: the spool never turns, the line peels off its front over the still rotor and out of the eye (about 0.8 N). Range %.1f m, time of flight %.2f s." %
      (PH["flight"]["range_m"], PH["flight"]["tof_s"]))
traj = PH["traj"]
th = th_fire
sep_u = P["P_R"] + L_DART - PH_CG
cg0 = rail_world(sep_u, P["DART_W"], th)
x0r, z0r = traj[0][1], traj[0][2]
# rail drop dynamics: the rail falls on the cam under gravity, PG overruns bearing A
I_rail = (PH["masses_g"]["rail_assembly"] / 1000 + 0.06) * 0.17 ** 2 + 0.004
m_rail = PH["masses_g"]["rail_assembly"] / 1000 + 0.06
om = 0.0
hist = []
for k in range(1, len(traj)):
    t0, x0, z0 = traj[k - 1]; t1, x1, z1 = traj[k]
    dt = t1 - t0
    cg = cg0 + np.array([(x1 - x0r) * 1000, 0, (z1 - z0r) * 1000])
    pitch = math.degrees(math.atan2(z1 - z0, x1 - x0))
    state["dart"] = dart_pose_world(cg.tolist(), pitch)
    hist.append(cg.tolist())
    # drop: phi advances by gravity (cam constrained) or the motor, whichever is faster
    phi = state["phi"]
    if phi < P["PH_DROP1"] - 0.5:
        dthdphi = (theta_of(phi + 0.5) - theta_of(phi)) / 0.5
        Tg = -m_rail * G * 0.17 * math.cos(math.radians(theta_of(phi))) * dthdphi * 180 / math.pi * 0.85
        Ieff = I_rail * (dthdphi) ** 2 + 0.00017      # gear-side inertia only: the drop is close to a free fall
        om += max(0.0, Tg) / Ieff * dt
        n_m, _ = motor(T_motor_B(phi))
        om_m = math.radians(n_m / 60 * 360 / i_pg)
        om = max(om, om_m)
        dphi = math.degrees(om * dt)
        state["phi"] = min(P["PH_DROP1"], phi + dphi); state["alpha"] -= n_m / 60 * 360 * dt
        state["mdir"] = -1; state["rpm"], state["I"], state["Tm"] = n_m, motor(T_motor_B(phi))[1], T_motor_B(phi)
    else:
        state["mdir"] = 0; state["rpm"] = 0; state["I"] = 0; state["Tm"] = 0
    state["line_out"] = sum(math.dist(hist[i - 1], hist[i]) for i in range(1, len(hist))) / 1000 if len(hist) > 1 else 0
    state["spool_m"] = 20.0 - state["line_out"]
    state["line_pts"] = [[round(c, 1) for c in p] for p in hist[::max(1, len(hist) // 40)]] + [[round(c, 1) for c in hist[-1]]]
    push(dt, 1.0)
land_cg = np.array(hist[-1])
# ---- landing settle: dart lands nose first, falls flat on its belly
phase("land", "5. Landing", "Nose ballast keeps it nose first; it drops onto its belly. The tail cap end carries the tether.")
pitch_end = math.degrees(math.atan2(traj[-1][2] - traj[-2][2], traj[-1][1] - traj[-2][1]))
cg_flat = land_cg.copy(); cg_flat[2] = GROUND_Z + 21.0
def settle(f):
    p = pitch_end * (1 - f) ** 2
    c = land_cg * (1 - f) + cg_flat * f + np.array([60 * f, 0, 0])
    state["dart"] = dart_pose_world(c.tolist(), p)
hold(0.6, 1.0, fn=settle)
dart_flat_cg = (cg_flat + np.array([60, 0, 0])).tolist()
state["line_pts"] = None
# the rest of the B run: the rail free-fell, the PG overran to the drop end and bearing A (a roller clutch) relocked at
# once; the firmware stops B as SW_LOW closes and re-zeroes phi there (about 333)
state["phi"] = max(state["phi"], P["PH_DROP1"] + 0.5)

# ---- anchoring (dart firmware sequence, compressed)
phase("anchor", "6. Anchor (dart)", "KEDGE on its own: kick leg swings %.0f deg over the nose and screws the foot in, then the pods swing down %.2f deg while the counter rotating augers drill in. Shown 4x faster." %
      (P["DART_KICK_DEG"], P["DART_DEPLOY_DEG"]))
def anchor(f):
    k = min(1, f / 0.6); d = max(0, (f - 0.55) / 0.45)
    state["kick"] = P["DART_KICK_DEG"] * (0.5 - 0.5 * math.cos(math.pi * k))
    state["dep"] = P["DART_DEPLOY_DEG"] * (0.5 - 0.5 * math.cos(math.pi * min(1, d)))
hold(16.0, 4.0, fn=anchor)

# ---- tow: motor forward, pin out, rotor winds; rover pulled toward the anchor
phase("tow", "7. Tow (self recovery)", "Motor forward: bearing A freewheels, the backstop holds the launcher. The rotor's drag disc pushes the pickup pin out, it catches the line and lays it on the still spool. Tension held at the 20 N limit. Shown 10x faster.")
dist = 0.0; TOW_M = 2.5
state["mdir"] = 1
while dist < TOW_M:
    dt = 1 / 60 * 10
    T_line = min(20.0, 4.0 + dist * 40)
    T_rotor = T_line * P["ROT_OD"] / 2000 * 1.25; Tm = 0.3 + T_rotor * ratio_reel / ETA
    n, I = motor(Tm)
    r_sp = (P["SP_CORE"] / 2 + 1 + 4 * (state["spool_m"] / 20)) / 1000
    v = n * ratio_reel / 60 * 2 * math.pi * r_sp
    dl = v * dt
    state["alpha"] += n / 60 * 360 * dt
    state["pin"] = min(1.0, state["pin"] + 0.5)
    state["line_out"] -= dl; state["spool_m"] += dl; dist += dl
    state["rover"] += dl * 1000
    state["tension"] = T_line; state["rpm"], state["I"], state["Tm"] = n, I, Tm
    push(dt, 10.0)
state["mdir"] = 0; state["rpm"] = 0; state["I"] = 0; state["Tm"] = 0
hold(1.0)
state["tension"] = 0.0

phase("release", "8. Release (dart)", "Drills unscrew, pods fold up, kick leg swings back over the top. Shown 4x faster.")
def release(f):
    d = 1 - min(1, f / 0.45); k = 1 - max(0, (f - 0.4) / 0.6)
    state["dep"] = P["DART_DEPLOY_DEG"] * (0.5 - 0.5 * math.cos(math.pi * d))
    state["kick"] = P["DART_KICK_DEG"] * (0.5 - 0.5 * math.cos(math.pi * max(0, k)))
hold(16.0, 4.0, fn=release)
state["kick"] = 0; state["dep"] = 0

phase("cock", "9. Cock: motor backward", "Program gear teeth pick up the rack and draw the carriage back %.0f mm against the springs (%.0f to %.0f N). The keel lead-in pushes the sear hook down, the spring snaps it up in front of the keel, the carriage runs about 1.4 mm onto it. Stops in the dwell before the lift." % (P["STROKE"], PH["springs"]["F_rest_N"], PH["springs"]["F_full_N"]))
run_B_to(360 + (P["PH_REL"] + P["PH_LIFT0"]) / 2, 1.0)
state["phi"] -= 360
state["fired"] = False

phase("retrieve", "10. Retrieve: motor forward", "The reel winds the dart back tail first; it is dragged up the muzzle onto the cradle and slides back until its tail meets the pusher (seated switch / current spike stops the motor). Shown 12x faster.")
# dart path: along the ground to under the muzzle, then up onto the cradle and back to the pusher
mz = muzzle_world(P["TH_LOW"]) + np.array([state["rover"], 0, 0])
seat_cg_rail = (P["S_SETTLED"] + L_DART - PH_CG, P["DART_W"])
seat = rail_world(seat_cg_rail[0], seat_cg_rail[1], P["TH_LOW"]) + np.array([state["rover"], 0, 0])
start = np.array(dart_flat_cg)
path = []
R_H = 70.0; wc = (P["DART_W"] - P["CR_RI"]) - R_H; RA = R_H + 21.0          # horn arc and the dart axis riding on it
def horn_pt(alpha):
    u = P["U_MUZ"] + RA * math.sin(alpha); w = wc + RA * math.cos(alpha)
    return rail_world(u, w, P["TH_LOW"]) + np.array([state["rover"], 0, 0])
h_end = horn_pt(math.radians(72))
grnd_end = np.array([h_end[0] + 40, 0, GROUND_Z + 21])
for f in np.linspace(0, 1, 120):
    path.append((start * (1 - f) + grnd_end * f, 0.0))
# lifted off the ground to the horn end (hanging tail up), then over the convex horn onto the cradle
for f in np.linspace(0, 1, 30):
    path.append((grnd_end * (1 - f) + h_end * f, -72.0 * f))
for f in np.linspace(0, 1, 50):
    al = math.radians(72 * (1 - f))
    path.append((horn_pt(al), -math.degrees(al) + P["TH_LOW"] * f))
up1 = rail_world(P["U_MUZ"] - 10, P["DART_W"], P["TH_LOW"]) + np.array([state["rover"], 0, 0])
for f in np.linspace(0, 1, 60):
    path.append((up1 * (1 - f) + seat * f, P["TH_LOW"]))
L_total = sum(np.linalg.norm(path[i][0] - path[i - 1][0]) for i in range(1, len(path))) / 1000
state["mdir"] = 1
k = 0; acc = 0.0
pts = [p for p, _ in path]
cum = [0.0]
for i in range(1, len(pts)):
    cum.append(cum[-1] + np.linalg.norm(pts[i] - pts[i - 1]) / 1000)
traveled = 0.0
while traveled < cum[-1] - 1e-6:
    dt = 1 / 60 * 12
    Tm = 0.3 + 6.0 * P["ROT_OD"] / 2000 * 1.25 * ratio_reel / ETA
    n, I = motor(Tm)
    r_sp = (P["SP_CORE"] / 2 + 1 + 4 * (state["spool_m"] / 20)) / 1000
    v = n * ratio_reel / 60 * 2 * math.pi * r_sp
    traveled = min(cum[-1], traveled + v * dt)
    j = int(np.searchsorted(cum, traveled)); j = max(1, min(len(pts) - 1, j))
    f = (traveled - cum[j - 1]) / max(1e-9, cum[j] - cum[j - 1])
    pos = pts[j - 1] * (1 - f) + pts[j] * f
    pitch = path[j - 1][1] * (1 - f) + path[j][1] * f
    state["dart"] = dart_pose_world(pos.tolist(), pitch)
    state["alpha"] += n / 60 * 360 * dt; state["pin"] = 1.0
    state["line_out"] = max(0.0, cum[-1] - traveled); state["spool_m"] = 20.0 - state["line_out"]
    state["tension"] = 6.0; state["rpm"], state["I"], state["Tm"] = n, I, Tm
    push(dt, 12.0)
state["mdir"] = 0; state["rpm"] = 0; state["I"] = 0; state["Tm"] = 0; state["tension"] = 0
state["dart"] = None
phase("ready2", "11. Ready again", "Seated, cocked, rail down. Next shot needs only the motor backward. No hand operation.")
hold(3.0)

out = dict(samples=S, phases=PHASES, deck=DECK, ground_z=GROUND_Z, trip_phi=trip_phi, th_fire=th_fire,
           summary=dict(v_muzzle=PH["launch"]["v_muzzle"], range_m=PH["flight"]["range_m"], tof=PH["flight"]["tof_s"],
                        total_real_s=state["t"], total_sim_s=state["ts"]))
json.dump(out, open(f"{OUT}/timeline.json", "w"), separators=(",", ":"))
print("samples", len(S), "real %.1fs  sim %.1fs  trip at phi %.1f theta %.1f" % (state["t"], state["ts"], trip_phi, th_fire))
for p in PHASES: print("  %-9s ts %6.2f  t %7.2f" % (p["id"], p["ts"], p["t"]))
