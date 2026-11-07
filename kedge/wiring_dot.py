import json, sys, re, subprocess
PK=sys.argv[1]; W=json.load(open(PK+'/wires.json'))['wires']
W=[[r['n'],r['fr'],r['to'],r['col']] if isinstance(r,dict) else r for r in W]   # v31 wires.json rows are dicts
COL={'black':'#222222','red':'#d0312d','white':'#999999','orange':'#e58a1f','yellow':'#d4b106','red/white':'#e07070','blue':'#1f5fd0','green':'#1e9e48','purple':'#8a3fd1','grey':'#777777'}
def node(s):
    s=s.split('(')[0].strip()
    for k in ['XIAO','DRV#1','DRV#2','DRV#3','INA219','BNO085','MP1584','BMS','VTX','Camera','Charge socket','Power loop socket','Deploy N20','Kick N20','Foot N20','Drill motor LEFT','Drill motor RIGHT','Cell']:
        if s.startswith(k): return k
    return s
def pad(s):
    s=re.sub(r'\(.*?\)','',s).strip()
    n=node(s); p=s[len(n):].strip() if s.startswith(n) else s
    return p.replace('SWAPPED','').strip() or '+'
lines=['digraph G { rankdir=LR; splines=true; nodesep=0.25; ranksep=1.1; bgcolor="white";',
 'node [shape=box, style="rounded,filled", fillcolor="#f2f2f2", fontname="Arial", fontsize=11, color="#555555"];',
 'edge [fontname="Arial", fontsize=8, penwidth=1.4, arrowhead=none];']
for n in ['Cell','BMS','Charge socket','Power loop socket','INA219','MP1584','XIAO','DRV#1','DRV#2','BNO085','VTX','Camera','Deploy N20','Kick N20','Drill motor LEFT','Drill motor RIGHT','Foot N20']:
    lines.append(f'"{n}";')
lines.append('{rank=same; "Cell"; "Charge socket"; "Power loop socket";} {rank=same; "DRV#1"; "DRV#2"; "BNO085";} {rank=same; "Deploy N20"; "Kick N20"; "Drill motor LEFT"; "Drill motor RIGHT"; "Foot N20"; "Camera";}')
for r in W:
    a,b=node(r[1]),node(r[2])
    if a==b: continue
    lab=f'{pad(r[1])} - {pad(r[2])}'
    lines.append(f'"{a}" -> "{b}" [color="{COL.get(r[3],"#444")}", label="{r[0]}: {lab}"];')
lines.append('}')
open(PK+'/wiring.dot','w').write('\n'.join(lines))
subprocess.run(['dot','-Tpng','-Gdpi=170',PK+'/wiring.dot','-o',PK+'/wiring.png'],check=True)
print('ok')
