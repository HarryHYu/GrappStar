import json, sys, numpy as np, trimesh
OUT = sys.argv[1]; J = json.load(open(OUT + '/parts.json')); M = {}
for p in J['parts']:
    z = np.load(f"{OUT}/{p['name']}.npz"); m = trimesh.Trimesh(z['v'], z['f'], process=False)
    # smooth shading with a crease angle: split vertices on sharp edges
    m2 = m.smooth_shaded if len(m.faces) < 60000 else m
    try:
        m2 = trimesh.graph.smooth_shade(m, angle=np.radians(35))
    except Exception:
        m2 = m
    v = np.round(m2.vertices, 2); n = np.round(m2.vertex_normals, 3)
    M[p['name']] = dict(v=v.ravel().tolist(), n=n.ravel().tolist(), f=m2.faces.ravel().tolist())
json.dump(M, open(OUT + '/meshes.json', 'w'), separators=(',', ':'))
import os; print('meshes.json', round(os.path.getsize(OUT + '/meshes.json') / 1e6, 1), 'MB')
