import json, math, sys, numpy as np
from trimesh.transformations import rotation_matrix as RM, translation_matrix as TM
OUT=sys.argv[1]; J=json.load(open(OUT+'/parts.json')); P=J['P']; G=J['ground']; PP={p['name']:p for p in J['parts']}
def V(n): return np.load(f'{OUT}/{n}.npz')['v']
def minz(names, root, deg, sgn):
    T=TM(PP[root]['pos'])@RM(math.radians(sgn*deg),[0,1,0]); z=[]
    for n in names:
        v=V(n); z.append(((np.c_[v,np.ones(len(v))]@T.T)[:,2]).min())
    return min(z)-G
pod=['drill_leg_L','drill_leg_L_brg1','drill_leg_L_brg2','drill_leg_L_motor','drill_leg_L_nut']; aug=['drill_leg_L_auger']
pod=[n for n in pod if n in PP]
best=None
for d in np.arange(30,60.01,0.25):
    c=minz(pod,'drill_leg_L',d,1)
    if c>=0.5: best=d
assert best is not None, 'no deploy angle passes the ground check'
print('deploy_deg', best, 'pod clear', round(minz(pod,'drill_leg_L',best,1),2), 'auger depth', round(-minz(aug,'drill_leg_L',best,1),1))
kb=None
for k in np.arange(190,230.01,0.25):
    c=minz(['kick_leg','n20_foot'],'kick_leg',k,-1)
    if c>=0.0: kb=k
assert kb is not None, 'no kick angle passes the ground check'
print('kick_deg', kb, 'housing clear', round(minz(['kick_leg'],'kick_leg',kb,-1),2), 'foot auger depth', round(-minz(['foot_auger'],'kick_leg',kb,-1),1))

P['DEPLOY_DEG']=float(best); P['KICK_DEG']=float(kb)
for p in J['parts']:
    if p['axis']:
        p['deg']=[0, float(kb) if p['name']=='kick_leg' else float(best)]
json.dump(J, open(OUT+'/parts.json','w'), indent=1)
print('written')
