"""Collision sweep of the ground station over the whole program cycle (exact manifold booleans on the saved meshes).
python audit_gs.py out [step_deg]"""
import json, math, sys, itertools, time
import numpy as np
import manifold3d as m3
from manifold3d import Manifold, Mesh
sys.path.insert(0, ".")
from pose import Kin, sear_angle as SA, creep as CREEP

OUT = sys.argv[1]; STEP = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
J = json.load(open(f"{OUT}/gs_parts.json")); P = J["P"]
theta = [t for _, t in json.load(open(f"{OUT}/theta_table.json"))["theta"]]
K = Kin(P)


def load(name):
    z = np.load(f"{OUT}/{name}.npz")
    return Manifold(Mesh(vert_properties=z["v"].astype(np.float32), tri_verts=z["f"].astype(np.uint32)))


parts = {p["name"]: p for p in J["parts"]}
DG = json.load(open(f"{OUT}/dart_groups.json"))
P["DART_PIV"] = {g: d["pivot"] for g, d in DG.items() if not g.startswith("_") and d["pivot"]}
for g, d in DG.items():
    if g.startswith("_"): continue
    parts[g] = dict(name=g, frame=g if d["pivot"] else "dart", group="dart", printed=False)
solids = {n: load(n) for n in parts}
for n, s in solids.items():
    if s.status() != m3.Error.NoError:
        print("BAD MESH", n, s.status())

# pairs that touch by design (shaft in bore, bearing in seat, gear mesh pairs checked separately)
ALLOW = {frozenset(x) for x in [
    ("main_shaft", "owc_A"), ("main_shaft", "brg_pivot_R"), ("main_shaft", "brg_pivot_L"), ("main_shaft", "brg_frame_R"), ("main_shaft", "brg_frame_L"),
    ("main_shaft", "reel_main_gear"), ("main_shaft", "pinion_spacer_R"), ("main_shaft", "pinion_spacer_L"), ("main_shaft", "shaft_spacer_L"), ("main_shaft", "shaft_spacer_R"), ("main_shaft", "shaft_spacer_M"),
    ("pg_stub", "pg_spacer_L"), ("pg_stub", "pg_spacer_R"), ("main_shaft", "wiper_coupler"), ("owc_A", "drive_pinion"),
    ("pg_stub", "program_gear"), ("pg_stub", "lift_cam"), ("pg_stub", "owc_C"), ("pg_stub", "brg_pg_R"), ("pg_stub", "brg_pg_L"),
    ("rail_rear", "brg_pivot_R"), ("rail_rear", "brg_pivot_L"), ("rail_rear", "brg_pg_R"), ("rail_rear", "brg_pg_L"), ("rail_rear", "owc_C"),
    ("frame_plate_R", "brg_frame_R"), ("frame_plate_L", "brg_frame_L"),
    ("reel_stub", "rotor"), ("reel_stub", "spool"), ("reel_stub", "reel_bracket"), ("reel_stub", "reel_stub_gear"),
    ("spool", "oring"), ("drag_disc", "oring"), ("rotor", "pickup_pin"), ("drag_disc", "pickup_pin"),
    ("spool", "reel_bracket"), ("reel_cover", "reel_bracket"),
    ("rubber_foot_R", "bumper_posts"), ("rubber_foot_L", "bumper_posts"),
    ("rail_rear", "reel_bracket"), ("motor_plate", "wiper_motor"),
    ("wiper_coupler", "wiper_motor"),
    ("sled", "keel_pad"), ("sear", "cord_pin"), ("trip_anchor", "trip_eye"), ("trip_anchor", "trip_spring"), ("trip_eye", "trip_spring"),
    ("frame_plate_R", "roller_pin"), ("sear_spring_post", "sear_spring"), ("pg_stub", "pg_hub"), ("pg_stub", "cam_hub"), ("pg_hub", "program_gear"), ("cam_hub", "lift_cam"), ("pg_hub", "pg_spacer_L"), ("strap_anchor", "strap_eye"), ("sled", "stop_pad_R"), ("sled", "stop_pad_L"), ("strap_anchor", "strap_spring"), ("strap_eye", "strap_spring"), ("rail_rear", "strap_pin"),

]}
ALLOW |= {frozenset(("sled", "sw_seated")), frozenset(("dart_body_hull", "sw_seated")), frozenset(("dart_body_elec", "sw_seated"))}   # switch actuation
ALLOW |= {frozenset(("sear", "sear_pin")), frozenset(("rail_rear", "sear_pin")), frozenset(("rail_beam", "sear_pin")),
          frozenset(("sear", "sear_spring")), frozenset(("rail_rear", "sear_spring")), frozenset(("roller_post", "roller_pin")), frozenset(("cam_roller", "roller_pin"))}
for nm in ("R", "L"):   # the cord ties to the shoe, runs over the pulley and hooks into the anchor bar
    ALLOW |= {frozenset(("cord_env_" + nm, "sled")), frozenset(("cord_env_" + nm, "pulley_" + nm)), frozenset(("spring_env_" + nm, "pulley_" + nm)),
              frozenset(("cord_env_" + nm, "spring_env_" + nm))}
# the cheeks were one part (rail_rear): every allowance it had applies to each piece
for pr in list(ALLOW):
    if "rail_rear" in pr:
        other = [x for x in pr if x != "rail_rear"]
        for c in ("cheek_L", "cheek_R", "cheek_tie"):
            ALLOW.add(frozenset([c] + (other or [c])))
ALLOW |= {frozenset(("cheek_L", "cheek_tie")), frozenset(("cheek_R", "cheek_tie"))}
GEAR_PAIRS = {frozenset(("drive_pinion", "program_gear")), frozenset(("reel_main_gear", "reel_stub_gear")), frozenset(("rack", "program_gear"))}


def state_at(phi, fired, dart_on=True):
    th = theta[int(round((phi % 360) / 360 * (len(theta) - 1)))]
    s = K.sled_pos(phi, fired)
    alpha = th - phi * P["Z_PG"] / P["Z_PIN"]
    rot = P["STUB_PHASE"] - (alpha - th) * P["Z_MAIN"] / P["Z_STUB"]
    sear = SA(P, th, s)
    if not fired and (phi % 360) >= P["PH_REL"] and sear < 0:
        s -= CREEP(P, sear)
    return dict(theta=th, phi=phi, alpha=alpha, s=s, rot=rot, drag=0, pin=0, sear=sear, dart=None)


def posed(F, name):
    fr = parts[name]["frame"]
    M = F.get(fr, F["world"])
    return solids[name].transform(M[:3, :].astype(np.float64))


hits = {}
t0 = time.time()
states = []
for phi in np.arange(0, 360, STEP):
    fired = phi >= P["PH_TRIP"]
    states.append((phi, fired, True))
names = list(parts)
# static world parts never collide with each other in a changing way: check once
# carriage driven into its end-stop pads: sled 3 to 9 mm past rest, dart gone (pads compress about 9 mm)
OVERRUN = {(-1.0 - k, True, False): 3.0 * (k + 1) for k in range(3)}
states += list(OVERRUN)
for phi, fired, dart_on in states:
    st = state_at(phi if phi >= 0 else P["PH_DROP1"], fired)
    if (phi, fired, dart_on) in OVERRUN:
        st["s"] += OVERRUN[(phi, fired, dart_on)]
    F = K.frames(st)
    ps = {n: posed(F, n) for n in names if (dart_on or not n.startswith("dart"))}
    bb = {n: s.bounding_box() for n, s in ps.items()}
    for a, b in itertools.combinations(ps, 2):
        pr = frozenset((a, b))
        if pr in ALLOW or (a.startswith("dart") and b.startswith("dart")):
            continue
        A, B = bb[a], bb[b]
        if A[3] < B[0] or B[3] < A[0] or A[4] < B[1] or B[4] < A[1] or A[5] < B[2] or B[5] < A[2]:
            continue
        v = (ps[a] ^ ps[b]).volume()
        lim = 3.0 if pr in GEAR_PAIRS else 0.5
        if v > lim:
            k = tuple(sorted((a, b)))
            if k not in hits or hits[k][0] < v:
                hits[k] = (v, phi)
res = sorted(((v, k, ph) for k, (v, ph) in hits.items()), reverse=True)
for v, k, ph in res:
    print("COLLISION %-22s %-22s %8.1f mm3 at phi %.0f" % (k[0], k[1], v, ph))
print("states", len(states), "time %.0fs" % (time.time() - t0))
print("CLEAN" if not res else "NOT CLEAN (%d pairs)" % len(res))
json.dump(dict(collisions=[dict(a=k[0], b=k[1], mm3=v, phi=ph) for v, k, ph in res], step=STEP), open(f"{OUT}/audit_gs.json", "w"), indent=1)
