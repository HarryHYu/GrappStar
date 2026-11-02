"""Ground station physics: launch (springs, carriage, dart, friction, rail angle), flight (drag, tether payout),
motor load over the program cycle, reel tow. python physics_gs.py out  -> out/physics_gs.json + plots"""
import json, math, sys
import numpy as np

OUT = sys.argv[1]
J = json.load(open(f"{OUT}/gs_parts.json")); P = J["P"]
vol = {p["name"]: p["volume_cm3"] for p in J["parts"]}
G = 9.81
RHO_PLA = 1.24
FILL = {"gear": 1.0, "pin": 1.0, "struct": 0.55}   # effective density factor (4 walls, 40% infill ~ 0.55; gears solid)


def mass_g(name, kind="struct"):
    return vol[name] * RHO_PLA * FILL[kind]


# ------------------------------------------------------------------ launch
m_dart = 0.277 + 0.015 * 0      # KEDGE v30 with 15 g ballast (physics.json of the dart pipeline: 277 g)
try:
    import os as _os
    dj = json.load(open(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "..", "out30", "physics.json")))
    m_dart = (dj["mass_g"] + 15.0) / 1000         # v30 printed + bought, plus the 15 g nose ballast
except Exception as e:
    print("dart mass: using 277 g (", e, ")")
m_sled = (mass_g("sled") + mass_g("rack", "gear")) / 1000
# springs: design spec (per spring)
SPR = dict(k=0.36, F0=21.0, L0=238.0, wire=1.6, od=12.6, coils=134)
n_coils = SPR["coils"]
m_spring = n_coils * math.pi * (SPR["od"] - SPR["wire"]) / 1000 * (math.pi / 4 * (SPR["wire"] / 1000) ** 2) * 7850
m_eff = m_sled + 2 * m_spring / 3
k_tot = 2 * SPR["k"] * 1000     # N/m
F_rest = 2 * SPR["F0"]
S = P["STROKE"] / 1000
F_full = F_rest + k_tot * S
E_store = S * (F_rest + F_full) / 2
TH_SHOT = P.get("TH_RELEASE", P["TH_FIRE"])      # the dart leaves at the trip angle, not the cam top
th = math.radians(TH_SHOT)
MU_CR, MU_SH = 0.30, 0.25        # PLA on PLA: dart on cradle, shoes/rack in guides
# pulley losses: 2 pulleys on F693ZZ, cord over 180 deg: ~3% each
ETA_PUL = 0.97

def launch(dt=1e-5):
    x, v, t = 0.0, 0.0, 0.0            # x = carriage travel from full draw
    out = []
    while x < S:
        F_spring = ETA_PUL * (F_full - k_tot * x)
        m = m_eff + m_dart
        N = (m_dart + m_sled) * G * math.cos(th)
        # during the shot the cords pull at w 85 against the dart inertia at w 95: a small couple on the shoes (lever 10 mm)
        F_lip = MU_SH * F_spring * (95 - 85) / 48.0
        F_fric = MU_CR * m_dart * G * math.cos(th) + MU_SH * m_sled * G * math.cos(th) + F_lip
        a = (F_spring - F_fric) / m - G * math.sin(th)
        v += a * dt; x += v * dt; t += dt
        out.append((t, x, v, a))
    return out

tr = launch()
t_launch, _, v_muz, _ = tr[-1]
a_peak = max(a for *_, a in tr)
E_dart = 0.5 * m_dart * v_muz ** 2
# carriage stops on the front bumper (springs at preload): impact energy
E_carriage = 0.5 * m_sled * v_muz ** 2      # only the carriage KE goes into the pads (the spring KE stays in the springs as surge)

# ------------------------------------------------------------------ flight with drag and end-pull payout drag
A_FRONT = 2864e-6       # folded silhouette (barrel.json)
CD = 1.0
RHO_AIR = 1.2
LAMBDA = 0.6e-3         # 1 mm Dyneema kg/m
F_PEEL = 0.8            # end pull peel + friction over the rounded rotor edges and the ceramic eye (about 6 rad of wrap, mu 0.15)
H0 = (P["H"] + P["U_MUZ"] * math.sin(th)) / 1000 + 0.15   # muzzle height + assumed rover deck 150 mm


def flight(v0, ang_deg, dt=1e-3, brake=0.0):
    a = math.radians(ang_deg)
    x, z = 0.0, H0; vx, vz = v0 * math.cos(a), v0 * math.sin(a)
    traj = [(0.0, x, z, vx, vz)]; t = 0.0; L = 0.0
    while z > 0 and t < 10:
        v = math.hypot(vx, vz)
        Fd = 0.5 * RHO_AIR * CD * A_FRONT * v * v
        # tether: peel force + momentum flux of the line being pulled off (lambda v^2), directed back along the path
        Ft = F_PEEL + LAMBDA * v * v + brake
        ax = -(Fd + Ft) * vx / v / m_dart
        az = -G - (Fd + Ft) * vz / v / m_dart
        vx += ax * dt; vz += az * dt; x += vx * dt; z += vz * dt; t += dt; L += v * dt
        traj.append((t, x, z, vx, vz))
    return traj, x, t, L

traj, rng, tof, line_out = flight(v_muz, TH_SHOT)
traj_vac, rng_vac, _, _ = flight(v_muz, TH_SHOT, dt=1e-3, brake=-F_PEEL)
# sweep of launch angles for the record
ang_sweep = [(a, flight(v_muz, a)[1]) for a in range(15, 61, 5)]

# ------------------------------------------------------------------ motor load over the PG cycle
r_pg = P["M"] * P["Z_PG"] / 2 / 1000
i_pg = P["Z_PG"] / P["Z_PIN"]
ETA_GEAR = 0.92                     # printed spur mesh
tab = P["RACK_TAB"]
m_rail = (mass_g("rail_beam") + mass_g("cheek_L") + mass_g("cheek_R") + mass_g("cheek_tie") + mass_g("spring_anchor") + mass_g("spool") + mass_g("rotor") + mass_g("reel_cover") + mass_g("reel_bracket")
          + mass_g("program_gear", "gear") + mass_g("lift_cam", "gear") + mass_g("reel_stub_gear", "gear") + mass_g("sear")) / 1000 + 0.12   # + rods, bearings, springs
cg_u = 0.170                         # rail CG from the pivot (m), estimated from the part layout
theta_tab = [t for _, t in json.load(open(f"{OUT}/theta_table.json"))["theta"]]
cyc = []
for i in range(1, len(tab)):
    p0, d0 = tab[i - 1]; p1, d1 = tab[i]
    dd = (d1 - d0) / 1000; dphi = math.radians(p1 - p0)
    F0_ = F_rest + k_tot * d1 / 1000
    F = F0_ * (1 + MU_SH * (85 - 45) / 48.0) + MU_SH * m_sled * G     # cocking: cords at w 85, rack at w 45 press the shoes on the lips
    T_pg = F * dd / dphi / ETA_GEAR if dphi > 0 else 0
    cyc.append((p1 % 360, T_pg, "cock"))
# lift: dU/dphi of the rail weight (+ dart) through the cam
m_lift = m_rail + m_dart + m_sled
for i in range(1, len(theta_tab)):
    phi0 = 360 * (i - 1) / (len(theta_tab) - 1); phi1 = 360 * i / (len(theta_tab) - 1)
    t0, t1 = math.radians(theta_tab[i - 1]), math.radians(theta_tab[i])
    dU = m_lift * G * cg_u * (math.sin(t1) - math.sin(t0))
    if dU > 0:
        cyc.append((phi1, dU / math.radians(phi1 - phi0) / 0.85, "lift"))
# sear trip: the cord turns the sear against the catch (keel load lever + face friction) and the sear spring, over the
# rail lift from cord taut to release; as PG torque through the cam: dE/dphi. It ADDS to the lift torque at the same phi.
E_trip = abs(P["DELTA_REL"]) * math.pi / 180 * (0.0065 * F_full + 0.3 * F_full * 0.020 + 0.08)
th_taut = next(th for th, d in P["SEAR_CORD_TAB"] if d < -0.1)
ph_taut = next(i * 360 / (len(theta_tab) - 1) for i, t in enumerate(theta_tab) if 200 < i * 360 / (len(theta_tab) - 1) < 330 and t >= th_taut)
trip_deg = max(1.0, P["PH_TRIP"] - ph_taut)
T_trip = E_trip / math.radians(trip_deg) / 0.85
lift_at = {}
for p_, t_, k_ in cyc:
    if k_ == "lift": lift_at[int(p_)] = max(lift_at.get(int(p_), 0), t_)
cyc += [(ph, T_trip + lift_at.get(int(ph), 0.0), "trip") for ph in np.arange(ph_taut, P["PH_TRIP"], 0.5)]
# planet carrier: the program gear rides on the rail round the drive pinion, so while the rail lifts the motor turns only
# (i_pg - dtheta/dphi) per PG degree instead of i_pg: the tooth torque on the PG grows by i_pg / (i_pg - f')
def fprime(phi):
    n_ = len(theta_tab) - 1; j = int(round((phi % 360) / 360 * n_))
    return (theta_tab[min(n_, j + 1)] - theta_tab[max(0, j - 1)]) / (2 * 360 / n_)
amp_max = 1.0
cyc2 = []
for p_, t_, k_ in cyc:
    f_ = fprime(p_) if k_ != "cock" else 0.0
    a_ = i_pg / (i_pg - f_) if f_ > 0 else 1.0
    amp_max = max(amp_max, a_)
    cyc2.append((p_, t_ * a_, k_))
cyc = cyc2
T_pg_max = max(t for _, t, _ in cyc)
T_motor_max = T_pg_max / i_pg / ETA_GEAR
# wiper motor assumption (typical 12 V car wiper, low speed): no-load 45 rpm, stall 15 N m, stall 18 A, free 1.5 A
WIPER = dict(n0=45.0, Ts=15.0, Is=18.0, I0=1.5)
def motor_at(T):
    n = WIPER["n0"] * (1 - T / WIPER["Ts"]); I = WIPER["I0"] + (WIPER["Is"] - WIPER["I0"]) * T / WIPER["Ts"]
    return n, I
n_cock, I_cock = motor_at(T_motor_max)
# cycle time: integrate over the PG cycle at the local torque
loadmap = np.zeros(361)
for phi, T, _ in cyc:
    loadmap[int(phi) % 360] = max(loadmap[int(phi) % 360], T)
t_cycle = 0.0
for deg in range(360):
    n, _ = motor_at(loadmap[deg] / i_pg / ETA_GEAR)
    t_cycle += (i_pg - max(0.0, fprime(deg))) / (n / 60 * 360)
# ------------------------------------------------------------------ reel
r_spool_eff = (P["SP_CORE"] / 2 + 3) / 1000
ratio = P["Z_MAIN"] / P["Z_STUB"]
def tow(T_line):
    T_rotor = T_line * (P["ROT_OD"] / 2 / 1000) * 2.0       # wrap friction: pusher hole, eye (ceramic), face edge: about 4.8 rad, mu 0.15
    T_mot = T_rotor * ratio / ETA_GEAR
    n, I = motor_at(T_mot)
    v = n * ratio / 60 * 2 * math.pi * r_spool_eff
    return T_mot, n, I, v
tow_rows = [(T,) + tow(T) for T in (5, 10, 20, 30, 45, 60)]
T_line_stall = WIPER["Ts"] * ETA_GEAR / ratio / (P["ROT_OD"] / 2 / 1000 * 2.0)

# range for each spring anchor hole (the rear hole is the design point; the front hole adds 12 mm of preload stretch)
def range_for(extra_mm):
    global F_rest, F_full
    Fr0, Ff0 = F_rest, F_full
    F_rest = Fr0 + k_tot * extra_mm / 1000; F_full = Ff0 + k_tot * extra_mm / 1000
    tr_ = launch(); v_ = tr_[-1][2]
    rg = flight(v_, TH_SHOT)[1]
    F_rest, F_full = Fr0, Ff0
    return v_, rg
hole_table = [dict(hole=h, extra_mm=e, v=round(range_for(e)[0], 2), range_m=round(range_for(e)[1], 2)) for h, e in (("rear (design)", 0.0), ("front", 12.0))]

R = dict(
    anchor_holes=hole_table,
    masses_g=dict(dart=m_dart * 1000, sled_rack=m_sled * 1000, spring_each=m_spring * 1000, rail_assembly=m_rail * 1000),
    sear_trip=dict(energy_J=E_trip, phi_taut=ph_taut, phi_shot=P["PH_TRIP"], T_pg_trip_plus_lift_Nm=max(t for _, t, k in cyc if k == "trip")),
    springs=dict(SPR, k_total_N_per_mm=k_tot / 1000, F_rest_N=F_rest, F_full_N=F_full, stroke_mm=P["STROKE"], energy_J=E_store),
    launch=dict(v_muzzle=v_muz, t_ms=t_launch * 1000, a_peak_g=a_peak / G, E_dart_J=E_dart, efficiency=E_dart / E_store,
                carriage_impact_J=E_carriage, angle_deg=TH_SHOT),
    flight=dict(range_m=rng, tof_s=tof, line_out_m=line_out, launch_height_m=H0, range_no_tether_m=rng_vac, angle_sweep=ang_sweep),
    motor=dict(planet_amp_max=amp_max, T_pg_max_Nm=T_pg_max, T_motor_max_Nm=T_motor_max, rpm_at_peak=n_cock, I_at_peak_A=I_cock, cycle_s=t_cycle,
               wiper_assumed=WIPER, ratio_pg=i_pg),
    reel=dict(ratio=ratio, rows=[dict(T_line_N=r[0], T_motor_Nm=r[1], rpm=r[2], I_A=r[3], line_m_s=r[4]) for r in tow_rows],
              line_tension_at_stall_N=T_line_stall),
)
json.dump(dict(R, traj=[(t, x, z) for t, x, z, _, _ in traj[::10]], launch_trace=[(t, x, v) for t, x, v, _ in tr[::200]],
               cycle=[(p, t, k) for p, t, k in cyc[::2]]), open(f"{OUT}/physics_gs.json", "w"), indent=1)
print(json.dumps(R, indent=1))
