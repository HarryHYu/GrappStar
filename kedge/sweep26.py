import json, math, numpy as np, trimesh, sys
from trimesh.transformations import rotation_matrix as RM, translation_matrix as TM
OUT = sys.argv[1] if len(sys.argv) > 1 else 'out'
J = json.load(open(OUT + '/parts.json'))
def ld(p):
    z = np.load(OUT + '/' + p['name'] + '.npz'); return trimesh.Trimesh(vertices=z['v'], faces=z['f'], process=False)
M = {p['name']: ld(p) for p in J['parts']}
PP = {p['name']: p for p in J['parts']}
kids = {n: PP[n]['parent'] for n in M if PP[n].get('parent')}
fixed = [n for n in M if PP[n]['axis'] is None and not PP[n].get('parent')]
def assemble(name, T):
    """list of (partname, mesh) for name and all descendants, each transformed by T (parent chain applied)."""
    out = [(name, M[name].copy().apply_transform(T))]
    for k, par in kids.items():
        if par == name:
            Tk = T.copy()
            if PP[k].get('axis'):
                Tk = T @ TM(PP[k]['pos'])            # child hinge at its folded pose (deg 0)
            out += assemble(k, Tk)
    return out
def posed(name, deg):
    p = PP[name]; sgn = -1 if name == 'kick_leg' else 1
    T = TM(p['pos']) @ RM(math.radians(sgn * deg), [0, 1, 0])
    return assemble(name, T)
def vol(a, b):
    try:
        i = trimesh.boolean.intersection([a, b], engine='manifold'); return i.volume if i is not None and len(i.faces) else 0
    except Exception as e:
        print('BOOLEAN FAILED', e); return -1
bad = {}
def check(ms, name_fixed, m_fixed, tag):
    for pn, pm in ms:
        v = vol(pm, m_fixed)
        if v > 2: bad.setdefault((pn, name_fixed), []).append((tag, round(v)))
for k in range(0, 181, 20):
    km = posed('kick_leg', k)
    for f in fixed: check(km, f, M[f], k)
for d in range(0, int(PP['drill_leg_L']['deg'][1]) + 1, 9):
    for leg in ('drill_leg_L', 'drill_leg_R'):
        lm = posed(leg, d)
        for f in fixed: check(lm, f, M[f], d)
    for pn, pm in posed('drill_leg_L', d):
        for qn, qm in posed('drill_leg_R', d):
            v = vol(pm, qm)
            if v > 2: bad.setdefault((pn, qn), []).append((d, round(v)))
# drill legs vs the folded kick leg family
for d in (0, int(PP['drill_leg_L']['deg'][1])):
    for leg in ('drill_leg_L', 'drill_leg_R'):
        for pn, pm in posed(leg, d):
            for qn, qm in posed('kick_leg', 0):
                v = vol(pm, qm)
                if v > 2: bad.setdefault((pn, qn), []).append((d, round(v)))
# foot knee: foot family swung to its stop vs the kick leg body (in the kick leg frame)
if 'foot' in M:
    for fd in (0, 30, 60, 90):
        T = TM(PP['foot']['pos']) @ RM(math.radians(fd), [0, 1, 0])
        for pn, pm in assemble('foot', T):
            v = vol(pm, M['kick_leg'])
            if v > 2: bad.setdefault((pn + '@knee', 'kick_leg'), []).append((fd, round(v)))
# static: fixed vs fixed
names = fixed
for i in range(len(names)):
    for j in range(i + 1, len(names)):
        v = vol(M[names[i]], M[names[j]])
        if v > 2: bad.setdefault((names[i], names[j]), []).append(('static', round(v)))
for k, v in bad.items(): print(k, v[:6])
print('clean' if not bad else f'{len(bad)} collisions')
