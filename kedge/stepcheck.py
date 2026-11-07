"""Reimport every STEP of a print pack and report validity, solid count and volume against the STL.
python3 stepcheck.py pack_v31  -> pack_v31/step_check.json, prints ALL STEP VALID or the failures."""
import sys, os, json, glob, trimesh
from build123d import import_step
PK = sys.argv[1]; rep = {}
for f in sorted(glob.glob(f"{PK}/*.step")):
    n = os.path.basename(f)[:-5]
    if "assembly" in n: continue
    try:
        s = import_step(f); sol = s.solids(); ok = bool(s.is_valid) and len(sol) >= 1
        st = f"{PK}/{n}.stl"; v_stl = trimesh.load(st).volume if os.path.exists(st) else None
        rep[n] = dict(valid=ok, solids=len(sol), vol_step=round(s.volume, 1), vol_stl=round(v_stl, 1) if v_stl else None)
    except Exception as e:
        rep[n] = dict(valid=False, error=str(e)[:120])
json.dump(rep, open(f"{PK}/step_check.json", "w"), indent=1)
bad = [n for n, r in rep.items() if not r["valid"] or r.get("solids", 1) != 1]
for n, r in rep.items(): print(f"{n:22s} {r}")
print("ALL STEP VALID" if not bad else "STEP PROBLEMS: " + ", ".join(bad))
