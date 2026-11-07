"""Exact-mesh audit for KEDGE: collision sweep, ground plane, folded envelope, belly, printability stats.
python3 audit27.py out27  -> prints report, writes out27/audit.json"""
import json, math, sys, time, itertools
import numpy as np, trimesh, manifold3d as mf
from trimesh.transformations import rotation_matrix as RM, translation_matrix as TM

OUT = sys.argv[1] if len(sys.argv) > 1 else "out27"
STEP_K = float(sys.argv[2]) if len(sys.argv) > 2 else 10
STEP_D = float(sys.argv[3]) if len(sys.argv) > 3 else 2
J = json.load(open(OUT + "/parts.json")); P = J["P"]; G = J["ground"]
PP = {p["name"]: p for p in J["parts"]}
RAW = {}
for n in PP:
    z = np.load(f"{OUT}/{n}.npz"); RAW[n] = (np.asarray(z["v"], float), np.asarray(z["f"], np.int64))
kids = {}
for n, p in PP.items():
    if p.get("parent"): kids.setdefault(p["parent"], []).append(n)
roots_moving = [n for n, p in PP.items() if p["axis"] and not p.get("parent")]
fixed = [n for n, p in PP.items() if not p["axis"] and not p.get("parent")]
KICK = "kick_leg"
DEPLOY = [n for n in roots_moving if n != KICK]           # drill pods, couplers, deploy shaft

def T_of(name, deg):
    p = PP[name]; sgn = -1 if name == KICK else 1
    return TM(p["pos"]) @ RM(math.radians(sgn * deg), [0, 1, 0])

def family(root):
    out = [root]
    for k in kids.get(root, []): out += family(k)
    return out

def xf(name, T):
    v, f = RAW[name]; vv = (np.c_[v, np.ones(len(v))] @ T.T)[:, :3]; return vv, f

MF_CACHE = {}
def to_mf(v, f):
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(v, np.float32), tri_verts=np.asarray(f, np.uint32)))

def vol_overlap(a, b):
    (va, fa), (vb, fb) = a, b
    # cheap AABB reject
    if (va.min(0) > vb.max(0)).any() or (vb.min(0) > va.max(0)).any(): return 0.0
    try:
        ma, mb = to_mf(va, fa), to_mf(vb, fb)
        if ma.status() != mf.Error.NoError or mb.status() != mf.Error.NoError: return -1.0     # non-manifold input: report, never pass silently
        i = ma ^ mb
        return float(i.volume())
    except Exception as e:
        return -1.0

def pose_family(root, deg):
    T = T_of(root, deg)
    return {n: xf(n, T) for n in family(root)}

FIXED = {n: xf(n, np.eye(4)) for n in fixed}
hits = {}
OWN = {tuple(sorted(p)) for p in [("kick_shaft", "n20_kick"), ("deploy_shaft", "n20_deploy"), ("foot_shaft", "n20_foot")]}
THREAD = {tuple(sorted(p)) for p in [("screws_radial", "tray"), ("screws_radial", "tail_cap"), ("screw_ear_root", "tray"), ("kick_hub_screw", "kick_leg"),
                                     ("coupler_L_screw", "coupler_L"), ("coupler_R_screw", "coupler_R"), ("ballast", "nose")]}
threads = {}
def rec(a, b, tag, v, thr=0.5):
    if tuple(sorted((a, b))) in OWN: return            # a motor's own output shaft runs through its own gearbox
    if tuple(sorted((a, b))) in THREAD:                 # self-tapping screw threads cutting into their pilot hole: designed
        threads[" | ".join(sorted((a, b)))] = round(max(v, threads.get(" | ".join(sorted((a, b))), 0)), 1); return
    if v > thr or v < -0.9: hits.setdefault(tuple(sorted((a, b))), []).append((tag, round(v, 1)))

t0 = time.time()
# 1 static fixed vs fixed
for a, b in itertools.combinations(fixed, 2):
    rec(a, b, "static", vol_overlap(FIXED[a], FIXED[b]))
# 2 children vs parent / siblings (in their own frame)
for root in roots_moving:
    fam = pose_family(root, 0)
    for a, b in itertools.combinations(fam, 2):
        rec(a, b, "family", vol_overlap(fam[a], fam[b]))
# 3 kick sweep vs fixed and vs folded drill families
KMAX = max(200.0, P["KICK_DEG"]); DMAX = max(38.0, P["DEPLOY_DEG"])
kdegs = sorted(set(list(np.arange(0, KMAX + 0.01, STEP_K)) + [P["KICK_DEG"], 200.0]))
folded_deploy = {}
for r in DEPLOY: folded_deploy.update(pose_family(r, 0))
for k in kdegs:
    fam = pose_family(KICK, k)
    for a, ma in fam.items():
        for b, mb in FIXED.items(): rec(a, b, f"k{k:.0f}", vol_overlap(ma, mb))
        for b, mb in folded_deploy.items(): rec(a, b, f"k{k:.0f}", vol_overlap(ma, mb))
# 4 deploy sweep vs fixed, vs each other, vs kick folded AND kick deployed
ddegs = sorted(set(list(np.arange(0, DMAX + 0.01, STEP_D)) + [P["DEPLOY_DEG"], 38.0]))
kick0 = pose_family(KICK, 0); kickD = pose_family(KICK, P["KICK_DEG"])
for d in ddegs:
    fams = {}
    for r in DEPLOY: fams.update({n: (r, m) for n, m in pose_family(r, d).items()})
    for a, (ra, ma) in fams.items():
        for b, mb in FIXED.items(): rec(a, b, f"d{d:.0f}", vol_overlap(ma, mb))
        for b, mb in list(kick0.items()) + list(kickD.items()): rec(a, b, f"d{d:.0f}", vol_overlap(ma, mb))
    for (a, (ra, ma)), (b, (rb, mb)) in itertools.combinations(fams.items(), 2):
        if ra != rb: rec(a, b, f"d{d:.0f}", vol_overlap(ma, mb))
# 5 cross sweep: every kick angle (10 deg) against every drill angle (4 deg), in case the order of operations changes
for k in np.arange(0, KMAX + 0.01, 10):
    kf = pose_family(KICK, k)
    for d in np.arange(0, DMAX + 0.01, 4):
        df = {}
        for r in DEPLOY: df.update(pose_family(r, d))
        for a_, ma in kf.items():
            for b_, mb in df.items(): rec(a_, b_, f"k{k:.0f}d{d:.0f}", vol_overlap(ma, mb))
t_sweep = time.time() - t0

# 5 ground: deployed pose
rep = {"collisions": {f"{a} | {b}": v[:8] for (a, b), v in hits.items()}, "thread_engagement_mm3": threads}
dep = {}
dep.update(pose_family(KICK, P["KICK_DEG"]))
for r in DEPLOY: dep.update(pose_family(r, P["DEPLOY_DEG"]))
dep.update(FIXED)
ground = {}
for n, (v, f) in dep.items():
    zmin = float(v[:, 2].min()); ground[n] = round(zmin - G, 2)
augers = [n for n in dep if any(k in n for k in ("auger", "_shaft", "nyloc")) and n not in ("kick_shaft", "deploy_shaft")]
gbad = {n: d for n, d in ground.items() if n not in augers and d < -0.3}
rep["ground_below"] = gbad
rep["auger_depth_mm"] = {n: round(-ground[n], 1) for n in augers}
rep["foot_housing_clear_mm"] = ground[KICK]
rep["pod_clear_mm"] = {n: ground[n] for n in DEPLOY if n.startswith("drill_leg")}
# 6 folded envelope + belly rule (below z=0 nothing outside r=21.05 except allowed)
fold = {}
fold.update(pose_family(KICK, 0))
for r in DEPLOY: fold.update(pose_family(r, 0))
fold.update(FIXED)
allv = np.vstack([v for v, f in fold.values()])
rep["folded_bbox"] = [round(float(x), 1) for x in allv.min(0)] + [round(float(x), 1) for x in allv.max(0)]
belly = {}
for n, (v, f) in fold.items():
    tm = trimesh.Trimesh(v, f, process=False); pts = np.vstack([v, tm.sample(20000)])
    m = pts[:, 2] < -0.01; r = np.hypot(pts[m, 1], pts[m, 2]) if m.any() else np.array([0])
    if r.max() > R_ if (R_ := P["R_OUT"] + 0.05) else False: belly[n] = round(float(r.max()), 2)
rep["belly_outside_r21"] = belly
rep["folded_min_z_below_ground"] = {n: ground_ for n, (v, f) in fold.items() if (ground_ := round(float(v[:, 2].min()) - G, 2)) < -0.05}
# 7 mass
pr = [p for p in J["parts"] if p["printable"]]
rep["printed_cm3"] = round(sum(p["volume_cm3"] for p in pr), 2)
rep["printed_g_solid_PLA"] = round(rep["printed_cm3"] * 1.24, 1)
rep["watertight_fail"] = [p["name"] for p in J["parts"] if not p.get("watertight", True)]
rep["multi_body"] = [p["name"] for p in J["parts"] if p["printable"] and p.get("nsolids", 1) > 1]   # bought-part groups (7 screws, 2 plates) are many bodies by design
rep["kick_range"] = [0, KMAX]; rep["deploy_range"] = [0, DMAX]
rep["L"] = P["L"]; rep["sweep_s"] = round(t_sweep, 1); rep["kick_steps"] = len(kdegs); rep["deploy_steps"] = len(ddegs)
# 8 hard stops (v31): they must touch within a few degrees past the end of each swing, and never before
stops = {}
hull_top = FIXED.get("hull_top")
def ov_with(fam, names):
    return round(sum(max(vol_overlap(fam[n], hull_top), 0.0) for n in names if n in fam), 2)
if hull_top is not None and "HORN_GAP" in P:
    pods = [n for n in DEPLOY if n.startswith("drill_leg")]
    for d, tag in ((P["DEPLOY_DEG"], "deploy_at_angle"), (P["DEPLOY_DEG"] + 2.5, "deploy_horn_+2.5deg"), (-0.0, "fold_at_0"), (-3.0, "fold_lug_-3deg")):
        f = {}
        for r in pods: f.update(pose_family(r, d))
        stops[tag] = ov_with(f, pods)
    kr = P["KICK_DEG"] + P.get("KICK_REST_DEG", 0)
    for k, tag in ((kr - 0.5, "kick_rest_-0.5deg"), (kr + 2.0, "kick_rest_+2deg")):
        stops[tag] = ov_with(pose_family(KICK, k), [KICK])
    stop_ok = stops["deploy_at_angle"] == 0 and stops["fold_at_0"] == 0 and stops["kick_rest_-0.5deg"] == 0 and \
        stops["deploy_horn_+2.5deg"] > 1 and stops["fold_lug_-3deg"] > 1 and stops["kick_rest_+2deg"] > 1
    stops["ok"] = bool(stop_ok)
rep["hard_stops_mm3"] = stops
json.dump(rep, open(OUT + "/audit.json", "w"), indent=1)
print(json.dumps(rep, indent=1))
print("CLEAN" if not hits and not gbad and not belly and not rep["folded_min_z_below_ground"] and not rep["watertight_fail"] and not rep["multi_body"]
      and stops.get("ok", True) else "NOT CLEAN")
