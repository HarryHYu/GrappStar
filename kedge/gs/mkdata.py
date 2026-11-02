"""Pack meshes + kinematic tables for the three.js pages. python mkdata.py out"""
import base64, json, sys, math
import numpy as np
from shapely.geometry import Polygon
sys.path.insert(0, ".")
from pose import theta_table

OUT = sys.argv[1]
J = json.load(open(f"{OUT}/gs_parts.json")); P = J["P"]
cam = Polygon(P["CAM_POLY"])
tt = theta_table(P, cam, 720)
dev = max(abs(t - 0) for _, t in tt)
parts = []


def enc(a):
    return base64.b64encode(np.ascontiguousarray(a).tobytes()).decode()


for p in J["parts"]:
    z = np.load(f"{OUT}/{p['name']}.npz")
    parts.append(dict(name=p["name"], frame=p["frame"], color=p["color"], group=p["group"], printed=p["printed"], note=p["note"],
                      v=enc(z["v"].astype(np.float32)), f=enc(z["f"].astype(np.uint32))))
DG = json.load(open(f"{OUT}/dart_groups.json"))
COL = dict(dart_body_hull="#c9cdd3", dart_body_elec="#2e7d4f", dart_kick="#e8803a", dart_kick_aug="#c4384a", dart_drillL="#e85a4a",
           dart_drillR="#e85a4a", dart_drillL_aug="#c4384a", dart_drillR_aug="#c4384a", dart_cpl="#9a9fa6")
P["DART_PIV"] = {g: d["pivot"] for g, d in DG.items() if not g.startswith("_") and d["pivot"]}
P["DART_KICK_DEG"] = DG["_P"]["KICK_DEG"]; P["DART_DEPLOY_DEG"] = DG["_P"]["DEPLOY_DEG"]
for g, d in DG.items():
    if g.startswith("_"): continue
    z = np.load(f"{OUT}/{g}.npz")
    parts.append(dict(name=g, frame=g if d["pivot"] else "dart", color=COL.get(g, "#bbbbbb"), group="dart", printed=False, note="KEDGE v30",
                      v=enc(z["v"].astype(np.float32)), f=enc(z["f"].astype(np.uint32))))
D = dict(P={k: v for k, v in P.items() if k != "CAM_POLY"}, parts=parts, theta=[t for _, t in tt])
open(f"{OUT}/gs_data.js", "w").write("window.GS=" + json.dumps(D) + ";")
th = [t for _, t in tt]
print("theta table: min %.2f max %.2f; at phi 300 %.2f, 330 %.2f, 335 %.2f" % (min(th), max(th), th[600], th[660], th[670]))
json.dump(dict(theta=tt), open(f"{OUT}/theta_table.json", "w"))
# the shot happens where the REAL rail angle (from the cam) reaches the cord release angle
P["PH_TRIP"] = next(ph for ph, t in tt if P["PH_LIFT0"] <= ph <= P["PH_DROP0"] + 5 and t >= P["TH_RELEASE"])
J["P"] = {k: v for k, v in P.items() if k not in ("DART_PIV", "DART_KICK_DEG", "DART_DEPLOY_DEG")} | {k: P[k] for k in ("DART_PIV", "DART_KICK_DEG", "DART_DEPLOY_DEG")}
json.dump(J, open(f"{OUT}/gs_parts.json", "w"), indent=1)
print("shot at phi %.1f (real cam), rail %.2f" % (P["PH_TRIP"], P["TH_RELEASE"]))
