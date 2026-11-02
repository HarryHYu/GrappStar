"""Faceted STEP for every printed ground station part (from the print-oriented STLs). python step_gs.py ../pack_gs"""
import sys, glob, os, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import trimesh
from meshstep import mesh_to_solid
from build123d import export_step
PK = sys.argv[1]
for f in sorted(glob.glob(f"{PK}/print/*.stl"), key=os.path.getsize):
    out = f"{PK}/step/" + os.path.basename(f).replace(".stl", ".step")
    if os.path.exists(out) and os.path.getmtime(out) > os.path.getmtime(f):
        continue
    t = time.time(); m = trimesh.load(f)
    try:
        S = mesh_to_solid(m); export_step(S, out); print("%-28s %6d faces %5.1fs valid %s" % (os.path.basename(f), len(m.faces), time.time() - t, S.is_valid), flush=True)
    except Exception as e:
        print("FAILED", f, e, flush=True)
