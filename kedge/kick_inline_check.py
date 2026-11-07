"""Side-view check of a straight kick leg with an in-line foot at 22 / 45 / 70 deg plant against the nose, hull and camera (v33 analysis of Harry's 250 deg request).
   .venv/bin/python kick_inline_check.py OUT OUT/review/27_kick_inline_check.png"""
import sys, json, math, numpy as np, trimesh, manifold3d as m3
from shapely.geometry import Polygon
from shapely.ops import unary_union
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
O=sys.argv[1]; OUTPNG=sys.argv[2]
J=json.load(open(O+'/parts.json')); P=J['P']
def load(n):
    z=np.load(f'{O}/{n}.npz'); return np.asarray(z['v'],float), np.asarray(z['f'])
def M(n):
    v,f=load(n); return m3.Manifold(m3.Mesh(vert_properties=np.asarray(v,np.float32),tri_verts=np.asarray(f,np.uint32)))
names=['nose','hull_top','hull_bottom','tail_cap','camera','tray']
sil=[]
for n in names:
    v,f=load(n); tri=v[f][:,:,[0,2]]
    polys=[Polygon(t) for t in tri if Polygon(t).area>1e-4]
    sil.append(unary_union(polys).buffer(0.05))
S=unary_union(sil)
body=M('nose')+M('hull_top')+M('hull_bottom')+M('camera')
kx,kz=P['KICK_X'],P['KICK_Z']
fig,ax=plt.subplots(figsize=(10,6.2))
for g in getattr(S,'geoms',[S]):
    x,y=g.exterior.xy; ax.fill(x,y,color='#c8ccd2',ec='black',lw=0.6)
cam=[g for g in [unary_union([Polygon(t) for t in load('camera')[0][load('camera')[1]][:,:,[0,2]] if Polygon(t).area>1e-4])]]
for g in cam:
    x,y=g.exterior.xy; ax.fill(x,y,color='#555',ec='black',lw=0.5)
ax.axhline(P['R_OUT']*-1,color='black',lw=0.8); ax.text(118,-24.5,'ground (dart lying flat)',fontsize=8)
res=[]
for phi,col in ((22,'#2a7'),(45,'#d80'),(70,'#c22')):
    K=180+phi; c,s=math.cos(math.radians(K)),math.sin(math.radians(K))
    bar=m3.Manifold.cube([150,8,8]).translate([6,-4,-4]).transform([[c,0,-s,kx],[0,1,0,0],[s,0,c,kz]])
    ov=(bar^body).volume(); res.append((phi,ov))
    L=150; d=np.array([math.cos(math.radians(phi))*-1, -math.sin(math.radians(phi))]); nrm=np.array([-d[1],d[0]])
    p0=np.array([kx,kz])+d*6; p1=np.array([kx,kz])+d*L
    q=[p0+nrm*4,p1+nrm*4,p1-nrm*4,p0-nrm*4]
    ax.fill([a[0] for a in q],[a[1] for a in q],color=col,alpha=0.45,ec=col)
    ax.text(*(p1+np.array([3,-3])),f'{phi} deg (kick {K}):\n{ov:.0f} mm3 inside nose/hull/camera',color=col,fontsize=8,ha='left',va='top')
ax.plot(kx,kz,'ko'); ax.text(kx+2,kz+3,'kick hinge (x %.1f, z %.1f)'%(kx,kz),fontsize=8)
ax.set_aspect('equal'); ax.set_xlim(-125,190); ax.set_ylim(-130,60); ax.grid(lw=0.3)
ax.set_title('Straight kick leg with an in-line foot, side view: only plants up to about 24 deg clear the nose',fontsize=10)
ax.set_xlabel('x mm (nose tip 0)'); ax.set_ylabel('z mm')
plt.tight_layout(); plt.savefig(OUTPNG,dpi=110); print(res)
