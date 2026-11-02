import json, math, sys, numpy as np, trimesh
from trimesh.transformations import rotation_matrix as RM, translation_matrix as TM
OUT=sys.argv[1]; J=json.load(open(OUT+'/parts.json')); P=J['P']; PP={p['name']:p for p in J['parts']}
M={n:trimesh.Trimesh(*[np.load(f'{OUT}/{n}.npz')[k] for k in ('v','f')],process=False) for n in PP}
kids={}
for n,p in PP.items():
    if p.get('parent'): kids.setdefault(p['parent'],[]).append(n)
def fam(r): 
    o=[r]; [o.extend(fam(k)) for k in kids.get(r,[])]; return o
fixed=[n for n,p in PP.items() if not p['axis'] and not p.get('parent')]
movers=[n for n,p in PP.items() if p['axis'] and not p.get('parent')]
def T(n,d): return TM(PP[n]['pos'])@RM(math.radians((-1 if n=='kick_leg' else 1)*d),[0,1,0])
PQ={n:trimesh.proximity.ProximityQuery(M[n]) for n in fixed}
SAMP={n:M[n].sample(1500, return_index=False) for n in PP}
res={}
def gap(pts, n):
    b=M[n].bounds
    m=((pts>b[0]-2)&(pts<b[1]+2)).all(1)
    if not m.any(): return None
    _,d,_=PQ[n].on_surface(pts[m]); return float(d.min())
for r in movers:
    rng=np.linspace(0,P['KICK_DEG'],15) if r=='kick_leg' else np.linspace(0,P['DEPLOY_DEG'],8)
    for d in rng:
        Tm=T(r,d)
        for c in fam(r):
            pts=(np.c_[SAMP[c],np.ones(len(SAMP[c]))]@Tm.T)[:,:3]
            for f in fixed:
                g=gap(pts,f)
                if g is not None and g<1.0:
                    k=(c,f); res[k]=min(res.get(k,9),g)
for (a,b),g in sorted(res.items(), key=lambda x:x[1]): print(f'{g:5.2f}  {a} -> {b}')
