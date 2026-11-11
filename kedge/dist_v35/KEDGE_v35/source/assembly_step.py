"""Folded-pose assembly STEP of the whole dart (printed parts as true B-rep, bought parts as envelopes) for launcher design in CAD.
python3 assembly_step.py out29 pack_v29"""
import json, sys, os, numpy as np, trimesh
from build123d import import_step, Compound, Location, export_step, Pos
from meshstep import mesh_to_solid
OUT, PK = sys.argv[1], sys.argv[2]
J = json.load(open(OUT + "/parts.json")); PP = {p["name"]: p for p in J["parts"]}
def world_pos(n):
    p = PP[n]; base = np.array(p["pos"], float)
    return base + (world_pos(p["parent"]) if p.get("parent") else 0)
kids = []
for n, p in PP.items():
    f = f"{OUT}/{n}.step"
    if os.path.exists(f): s = import_step(f)
    else:
        z = np.load(f"{OUT}/{n}.npz"); m = trimesh.Trimesh(z["v"], z["f"], process=False)
        if len(m.faces) > 6000: m = m.convex_hull
        s = mesh_to_solid(m)
    s = s.moved(Location(tuple(world_pos(n)))); s.label = n
    kids.append(s)
asm = Compound(children=kids); asm.label = f"KEDGE_{J.get('version','')}_folded"
export_step(asm, f"{PK}/KEDGE_{J.get('version','')}_assembly_folded.step")
print("assembly parts:", len(kids), os.path.getsize(f"{PK}/KEDGE_{J.get('version','')}_assembly_folded.step") // 1000, "kB")
