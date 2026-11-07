"""Dart wire cut list on the v31 geometry: the firmware v4 harness (DRV8833 #3 off the dart, 50 wires, same numbering and
colours as firmware/dart/wiring/wire_cut_list.csv) routed through the v31 passages:
  KEEL  the 9.2 x 8.4 channel under the cells (discs 2 and 3, both cell saddles) with the BMS stick, on through disc 4 to
        the charge socket; it joins the open board-bay floor (deck A is gone) and the deck B wire chase behind the XIAO,
        and the sled behind the deploy motor (x 55..62);
  TOP   over the VTX through the top notches of discs 2, 3, 4 (camera, VTX, DRV#2 motor leads, foot leads);
  holes drill leads x 59.5 z 16.5 both flanks (behind the bosses, into the pod tops at the motor end caps), kick leads
        straight down from the saddle's end-cap tabs, foot leads up through the skin at x 42, deploy leads through the
        disc 1 notch.
Length rule (as v29/v4): Manhattan length along the route x 1.15 + 30 mm, rounded up to 5 mm.
python3 wires.py out31 pack_v31  -> wire_cut_list.csv, wires.json (+ route and zone of every wire, for wireroom.py)"""
import json, numpy as np, math, csv, sys
OUT = sys.argv[1]; PK = sys.argv[2]
J = json.load(open(OUT + '/parts.json')); P = J['P']; PP = {p['name']: p for p in J['parts']}
WP = {k: np.array(v, float) for k, v in P['WIRE_PTS'].items()}
def bb(n):
    v = np.load(f'{OUT}/{n}.npz')['v']; return v.min(0), v.max(0)
def c(n, fx=0.5, fy=0.5, fz=1.0):
    a, b = bb(n); return a + (b - a) * np.array([fx, fy, fz])
X = dict(xiao_pins=c('xiao', 0.5, 0.6), xiao_5v=c('xiao', 0.2, 0.9), drv1=c('drv8833_1'), drv2=c('drv8833_2'), ina=c('ina219'),
         bno=c('bno085'), mp=c('mp1584'), vtx=c('vtx', 0.1, 0.5), bms_f=c('bms_2s', 0.05, 0.5, 0.0), bms_r=c('bms_2s', 0.95, 0.5, 0.0),
         cam=c('camera', 1.0, 0.5, 0.5), dep=WP['deploy_tabs'], d3=np.array([P['X_CELL'][0], 0, 0]), d4=np.array([P['X_CELL'][1], 0, -7.5]))
X.update(WP)
def route(*names):
    return [X[n] if isinstance(n, str) else np.array(n, float) for n in names]
def L(pts, extra=0):
    d = sum(np.abs(pts[i + 1] - pts[i]).sum() for i in range(len(pts) - 1))
    return int(math.ceil((d * 1.15 + 30 + extra) / 5.0) * 5)
W = []
def w(fr, to, col, g, names, note='', extra=0, fixed=None):
    pts = route(*names) if names else []
    W.append(dict(n=len(W) + 1, fr=fr, to=to, col=col, g=g, len=fixed if fixed else L(pts, extra), note=note, route=list(names)))
BB = ['drv1', 'chase', 'keel_d3', 'keel_d2', 'sled']               # board bay -> keel -> sled
BT = ['drv2', 'top_d3', 'top_d2']                                   # deck C -> over the VTX
w('Rear plate, cell 1 (-)', 'BMS B-', 'black', '22 AWG', ['d4', 'bms_r'], 'solder to the plate tab before it goes in its recess')
w('Front spring, cell 1 (+)', 'Front spring, cell 2 (-)', 'red', '22 AWG', [], 'the series link: full battery current; glue both springs in the front recess', fixed=45)
w('Front spring pair (cell 1 + / cell 2 -)', 'BMS BM', 'white', '26 AWG', ['d3', 'keel_d2', 'bms_f'], 'midpoint sense')
w('Rear plate, cell 2 (+)', 'BMS B+', 'red', '22 AWG', ['d4', 'bms_r'], '')
for i, (pin, col) in enumerate((('B-', 'black'), ('BM', 'white'), ('B+', 'red'))):
    w('BMS ' + pin, 'Charge socket pin %d (tail cap, bottom seat)' % (i + 1), col, '26 AWG', ['bms_r', 'keel_d3', 'keel_d4', 'tail_bot'],
      'JST-XH 3-pin, balance charge; along the keel channel' if i == 0 else '')
w('BMS P+', 'Power loop socket pin 1 (tail cap, top seat)', 'red', '22 AWG', ['bms_r', 'keel_d3', 'keel_d4', 'tail_top'], 'JST-XH 2-pin on top of the tail cap; the shorting plug is the ON switch')
w('Power loop socket pin 2', 'INA219 VIN+', 'red', '22 AWG', ['tail_top', 'win_d4', 'ina'], 'main + after the loop')
w('INA219 VIN+ (same pad)', 'MP1584 IN+', 'red', '22 AWG', ['ina', 'chase', 'keel_d3', 'keel_d2', 'sled', 'mp'], 'buck taps before the shunt; keel channel to the sled')
w('INA219 VIN-', 'DRV#1 VM', 'red', '22 AWG', ['ina', 'drv1'], 'motor rail 6.0-8.4 V')
w('DRV#1 VM', 'DRV#2 VM', 'red', '22 AWG', ['drv1', 'drv2'], 'daisy chain ends here (no DRV#3)')
w('BMS P-', 'MP1584 IN-', 'black', '22 AWG', ['bms_f', 'keel_d2', 'sled', 'mp'], 'main ground')
w('MP1584 IN- (same pad)', 'DRV#1 GND', 'black', '22 AWG', ['mp', 'sled', 'keel_d2', 'keel_d3', 'chase', 'drv1'], '')
w('DRV#1 GND', 'DRV#2 GND', 'black', '22 AWG', ['drv1', 'drv2'], '')
w('MP1584 OUT+ (5.04 V)', 'XIAO 5V via a Schottky diode (stripe toward the XIAO)', 'orange', '24 AWG', ['mp', 'sled', 'keel_d2', 'keel_d3', 'chase', 'xiao_5v'], 'set and check 5.04 V BEFORE connecting; 1N5817 or SS14 in heat shrink')
w('MP1584 OUT-', 'XIAO GND', 'black', '24 AWG', ['mp', 'sled', 'keel_d2', 'keel_d3', 'chase', 'xiao_5v'], '')
w('MP1584 OUT+', 'VTX 5V in', 'orange', '26 AWG', ['mp', 'top_d2', 'vtx'], 'up beside the deploy motor, over the top')
w('MP1584 OUT-', 'VTX GND', 'black', '26 AWG', ['mp', 'top_d2', 'vtx'], '')
for nm, col in (('Camera + (red)', 'orange'), ('Camera - (black)', 'black'), ('VTX video in', 'yellow')):
    fr = {'orange': 'VTX 5V in (same pad)', 'black': 'VTX GND (same pad)', 'yellow': 'Camera video (yellow)'}[col]
    w(fr, nm, col, '28 AWG', ['vtx', 'top_d2', 'top_d1', 'cam'], 'forward along the top through the disc notches' if col == 'orange' else ('twist with the camera ground' if col == 'yellow' else ''))
w('XIAO 3V3', 'DRV#1 SLP / nSLEEP / EEP pad (NOT VCC or VIN)', 'red/white', '28 AWG', ['xiao_pins', 'drv1'], 'wakes the driver; meter check first (WIRING.md step B6)')
w('DRV#1 SLP', 'DRV#2 SLP', 'red/white', '28 AWG', ['drv1', 'drv2'], '')
w('XIAO 3V3', 'INA219 VCC', 'red/white', '28 AWG', ['xiao_pins', 'ina'], '')
w('INA219 VCC', 'BNO085 VIN', 'red/white', '28 AWG', ['ina', 'chase', 'keel_d3', 'keel_d2', 'sled', 'bno'], 'the BNO085 is in the sled now: keel channel')
w('XIAO GND', 'INA219 GND', 'black', '28 AWG', ['xiao_pins', 'ina'], '')
w('INA219 GND', 'BNO085 GND', 'black', '28 AWG', ['ina', 'chase', 'keel_d3', 'keel_d2', 'sled', 'bno'], '')
w('XIAO D4', 'INA219 SDA', 'blue', '28 AWG', ['xiao_pins', 'ina'], 'I2C SDA')
w('INA219 SDA', 'BNO085 SDA', 'blue', '28 AWG', ['ina', 'chase', 'keel_d3', 'keel_d2', 'sled', 'bno'], 'twist SDA with GND along the keel')
w('XIAO D5', 'INA219 SCL', 'green', '28 AWG', ['xiao_pins', 'ina'], 'I2C SCL')
w('INA219 SCL', 'BNO085 SCL', 'green', '28 AWG', ['ina', 'chase', 'keel_d3', 'keel_d2', 'sled', 'bno'], '')
for pin, to, col, what in (('D0', 'DRV#1 AIN1', 'purple', 'deploy'), ('D1', 'DRV#1 AIN2', 'grey', 'deploy'), ('D2', 'DRV#1 BIN1', 'purple', 'kick'), ('D3', 'DRV#1 BIN2', 'grey', 'kick')):
    w('XIAO ' + pin, to, col, '28 AWG', ['xiao_pins', 'drv1'], what)
for pin, to, col, what in (('D8', 'DRV#2 AIN1 (only AIN1: NO bridge to BIN1)', 'purple', 'drills'), ('D9', 'DRV#2 AIN2 (only AIN2: NO bridge to BIN2)', 'grey', 'drills'),
                           ('D6', 'DRV#2 BIN1', 'purple', 'foot (slow worm, harmless if it twitches at reset)'), ('D7', 'DRV#2 BIN2', 'grey', 'foot')):
    w('XIAO ' + pin, to, col, '28 AWG', ['xiao_pins', 'drv2'], what)
w('DRV#1 AOUT1', 'Deploy N20 + (red)', 'red', '26 AWG', BB + ['bno', 'win_d1', 'dep'], 'keel channel, along the keel under the BNO085, up the disc 1 notch to the end-cap tabs')
w('DRV#1 AOUT2', 'Deploy N20 - (black)', 'black', '26 AWG', BB + ['bno', 'win_d1', 'dep'], '')
w('DRV#1 BOUT1', 'Kick N20 + (red)', 'red', '26 AWG', BB + ['kick_hole'], 'keel channel, up behind the deploy motor, through the saddle floor hole to the end-cap tabs', extra=10)
w('DRV#1 BOUT2', 'Kick N20 - (black)', 'black', '26 AWG', BB + ['kick_hole'], '', extra=10)
pod = 12
w('DRV#2 AOUT1', 'Drill motor LEFT + (red)', 'red', '26 AWG', BT + ['flankL'], 'over the top, out the left hole behind the boss, 25 mm loop into the pod top at the end cap', extra=pod + 25)
w('DRV#2 AOUT2', 'Drill motor LEFT - (black)', 'black', '26 AWG', BT + ['flankL'], '', extra=pod + 25)
w('DRV#2 AOUT1 (same pad as the LEFT red)', 'Drill motor RIGHT - (black)  SWAPPED', 'black', '26 AWG', BT + ['flankR'], 'reversed on purpose: the pair counter-rotates', extra=pod + 25)
w('DRV#2 AOUT2 (same pad as the LEFT black)', 'Drill motor RIGHT + (red)  SWAPPED', 'red', '26 AWG', BT + ['flankR'], '', extra=pod + 25)
leg = P['KICK_POLE_L'] + 20
w('DRV#2 BOUT1', 'Foot N20 + (red)', 'red', '26 AWG', BT + ['foot_hole'], 'over the top, out the skin hole at x 42, 60 mm loop round the hub, through the pole channel', extra=leg + 60)
w('DRV#2 BOUT2', 'Foot N20 - (black)', 'black', '26 AWG', BT + ['foot_hole'], 'silicone wire, it flexes every deploy', extra=leg + 60)
assert len(W) == 50, len(W)
with open(f'{PK}/wire_cut_list.csv', 'w', newline='') as f:
    cw = csv.writer(f); cw.writerow(['#', 'from', 'to', 'colour', 'gauge', 'cut length mm', 'note', 'route'])
    for r in W: cw.writerow([r['n'], r['fr'], r['to'], r['col'], r['g'], r['len'], r['note'], ' > '.join(x if isinstance(x, str) else 'pt' for x in r['route'])])
json.dump(dict(wires=W, points={k: [round(float(t), 2) for t in v] for k, v in X.items()}), open(f'{PK}/wires.json', 'w'), indent=1, default=str)
tot = {}
for r in W: tot[(r['g'], r['col'])] = tot.get((r['g'], r['col']), 0) + r['len']
print(len(W), 'wires; total mm by gauge/colour:', {f'{g} {c}': v for (g, c), v in sorted(tot.items())})
