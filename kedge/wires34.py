# v34 harness (run from wires.py when P has LEG_DEG). Firmware v4 numbering less the foot channel: 45 wires.
# Passages: the two LOW corridors beside the in-line cells (|y| 10.4..17.5, z -15..-3) through the side windows of disc 1 and the rear
# disc; deck 1 (boards) and deck 2 (VTX, buck); under the deploy cradle (z -17.5) between the two DRV8833; the drill leads out of the
# two tight holes at x DRILL_X - 10.5, z 16.5. Length rule as v29/v4: Manhattan length x 1.15 + 30 mm, rounded up to 5 mm.
# v35 (P.POD_DIR 1): the deploy cradle and the two DRV8833 sit at the FRONT (between disc 1 and disc 1b), the front spring on disc 1b's
# rear face, the drill holes at x DRILL_X + 10.5; the boards reach the drivers through disc 1b's top window; LEFT drill leads swapped.
import json, numpy as np, math, csv, sys
WP = {k: np.array(v, float) for k, v in P['WIRE_PTS'].items()}
def bb(n):
    v = np.load(f'{OUT}/{n}.npz')['v']; return v.min(0), v.max(0)
def c(n, fx=0.5, fy=0.5, fz=1.0):
    a, b = bb(n); return a + (b - a) * np.array([fx, fy, fz])
cz = P['CELL_Z']; D1 = P['X_D1']; D2 = P['X_D2']; V35 = P.get('POD_DIR') == 1; DS = P.get('X_D1B', D1)   # DS: the disc with the spring
X = dict(xiao_pins=c('xiao', 0.5, 0.6), xiao_5v=c('xiao', 0.2, 0.9), drv1=c('drv8833_1', 0.5, 0.5, 0.2), drv2=c('drv8833_2', 0.5, 0.5, 0.2),
         ina=c('ina219'), bno=c('bno085', 0.1, 0.5), mp=c('mp1584', 0.5, 0.5), vtx=c('vtx', 0.05, 0.5), vtx_r=c('vtx', 0.95, 0.5),
         bms_f=c('bms_2s', 0.05, 0.5), bms_r=c('bms_2s', 0.95, 0.5), cam=c('camera', 1.0, 0.5, 0.5),
         sprg=np.array([DS[1], 4.0, cz]), sprg_fr=np.array([DS[0] - 1.0, 11.0, cz]), mid=np.array([P['X_MID'][0], 0.0, P['DECK1']]),
         plate=np.array([P['X_C2'][1] + 0.5, 4.0, cz]), plate_bk=np.array([D2[1] + 1.0, 4.0, cz]),
         side_f_L=np.array([DS[0] - 2.0 + (1.0 if V35 else 0.0), 11.5, 0.0]), side_f_R=np.array([D1[0] - 2.0, -11.5, 0.0]), side_c_L=np.array([D1[0] - 2.0, 11.5, 0.0]),
         win_tL=np.array([DS[0] + 1.0, 9.4, 7.5]), win_tR=np.array([DS[0] + 1.0, -9.4, 7.5]), cross_f=np.array([DS[0] + 1.0, 0.0, 8.0]),
         under=np.array([P['DRILL_X'], 0.0, -17.6]), cross=np.array([D2[1] + 1.0, 0.0, 8.0]))
X.update(WP)
def route(*names): return [X[n] for n in names]
def L(pts, extra=0):
    d = sum(np.abs(pts[i + 1] - pts[i]).sum() for i in range(len(pts) - 1))
    return int(math.ceil((d * 1.15 + 30 + extra) / 5.0) * 5)
W = []
def w(fr, to, col, g, names, note='', extra=0, fixed=None):
    pts = route(*names) if names else []
    W.append(dict(n=len(W) + 1, fr=fr, to=to, col=col, g=g, len=fixed if fixed else L(pts, extra), note=note, route=list(names)))
CL = ['corr_L', 'corr_L2', 'win_d2_L']            # left corridor, front to rear disc
CR = ['corr_R', 'corr_R2', 'win_d2_R']
if V35:                                           # v35: the drivers are right in front of disc 1b, the boards just behind it
    CL, CR = ['win_tL'], ['win_tR']
w('Front spring (B-, cell 1 base)', 'BMS B-', 'black', '22 AWG', ['sprg', 'sprg_fr', 'side_f_L', 'bms_f'], 'through the lead hole in disc 1b, up beside the deploy cradle, onto deck 1' if V35 else 'through the lead hole in disc 1, up beside the leg cradle, onto deck 1')
w('BM strip tab (between the cells)', 'BMS BM', 'white', '26 AWG', ['mid', 'bms_f'], 'the strip tab comes up through the slot in deck 1 at the cell joint; solder it before the cells go in')
w('Rear plate (B+, cell 2 button)', 'BMS B+', 'red', '22 AWG', ['plate', 'plate_bk', 'cross', 'bms_r'], 'through the lead hole in the rear disc, up behind it, forward on deck 1')
for i, (pin, col) in enumerate((('B-', 'black'), ('BM', 'white'), ('B+', 'red'))):
    w('BMS ' + pin, 'Charge socket pin %d (right flank seat)' % (i + 1), col, '26 AWG', ['bms_r', 'corr_R2', 'win_d2_R', 'sock_R'],
      ('JST-XH 3-pin, balance charge; right corridor, through the rear disc, into the seat' if V35 else 'JST-XH 3-pin, balance charge; right corridor, past DRV#2, into the seat') if i == 0 else '')
w('BMS P+', 'Power loop socket pin 1 (left flank seat)', 'red', '22 AWG', ['bms_r', 'corr_L2', 'win_d2_L', 'sock_L'], 'JST-XH 2-pin; the shorting plug is the ON switch')
w('Power loop socket pin 2', 'INA219 VIN+', 'red', '22 AWG', ['sock_L', 'win_d2_L', 'corr_L2', 'ina'], 'main + after the loop')
w('INA219 VIN+ (same pad)', 'MP1584 IN+', 'red', '22 AWG', ['ina', 'mp'], 'buck taps before the shunt (up to deck 2)')
w('INA219 VIN-', 'DRV#1 VM', 'red', '22 AWG', ['ina'] + CL + ['drv1'], 'motor rail 6.0-8.4 V, forward through disc 1b' if V35 else 'motor rail 6.0-8.4 V, left corridor')
w('DRV#1 VM', 'DRV#2 VM', 'red', '22 AWG', ['drv1', 'under', 'drv2'], 'under the deploy cradle')
w('BMS P-', 'MP1584 IN-', 'black', '22 AWG', ['bms_r', 'mp'], 'main ground')
w('MP1584 IN- (same pad)', 'DRV#1 GND', 'black', '22 AWG', ['mp', 'win_tL', 'drv1'] if V35 else ['mp', 'corr_L2', 'win_d2_L', 'drv1'], '')
w('DRV#1 GND', 'DRV#2 GND', 'black', '22 AWG', ['drv1', 'under', 'drv2'], '')
w('MP1584 OUT+ (5.04 V)', 'XIAO 5V via a Schottky diode (stripe toward the XIAO)', 'orange', '24 AWG', ['mp', 'vtx_r', 'xiao_5v'], 'set and check 5.04 V BEFORE connecting; 1N5817 or SS14 in heat shrink')
w('MP1584 OUT-', 'XIAO GND', 'black', '24 AWG', ['mp', 'vtx_r', 'xiao_5v'], '')
w('MP1584 OUT+', 'VTX 5V in', 'orange', '26 AWG', ['mp', 'vtx_r'], 'both on deck 2')
w('MP1584 OUT-', 'VTX GND', 'black', '26 AWG', ['mp', 'vtx_r'], '')
for nm, col in (('Camera + (red)', 'orange'), ('Camera - (black)', 'black'), ('VTX video in', 'yellow')):
    fr = {'orange': 'VTX 5V in (same pad)', 'black': 'VTX GND (same pad)', 'yellow': 'Camera video (yellow)'}[col]
    w(fr, nm, col, '28 AWG', ['vtx', 'win_tL', 'win_d1_L', 'side_c_L', 'cam'] if V35 else ['vtx', 'win_d1_L', 'side_f_L', 'cam'], ('forward over disc 1b, through disc 1, beside the leg cradle into the nose' if V35 else 'forward through disc 1, beside the leg cradle into the nose') if col == 'orange' else ('twist with the camera ground' if col == 'yellow' else ''))
w('XIAO 3V3', 'DRV#1 SLP / nSLEEP / EEP pad (NOT VCC or VIN)', 'red/white', '28 AWG', ['xiao_pins'] + CL + ['drv1'], 'wakes the driver; meter check first (WIRING.md step B6)')
w('DRV#1 SLP', 'DRV#2 SLP', 'red/white', '28 AWG', ['drv1', 'under', 'drv2'], '')
w('XIAO 3V3', 'INA219 VCC', 'red/white', '28 AWG', ['xiao_pins', 'ina'], '')
w('INA219 VCC', 'BNO085 VIN', 'red/white', '28 AWG', ['ina', 'bno'], 'deck 1 neighbours')
w('XIAO GND', 'INA219 GND', 'black', '28 AWG', ['xiao_pins', 'ina'], '')
w('INA219 GND', 'BNO085 GND', 'black', '28 AWG', ['ina', 'bno'], '')
w('XIAO D4', 'INA219 SDA', 'blue', '28 AWG', ['xiao_pins', 'ina'], 'I2C SDA')
w('INA219 SDA', 'BNO085 SDA', 'blue', '28 AWG', ['ina', 'bno'], '')
w('XIAO D5', 'INA219 SCL', 'green', '28 AWG', ['xiao_pins', 'ina'], 'I2C SCL')
w('INA219 SCL', 'BNO085 SCL', 'green', '28 AWG', ['ina', 'bno'], '')
for pin, to, col, what in (('D0', 'DRV#1 AIN1', 'purple', 'deploy'), ('D1', 'DRV#1 AIN2', 'grey', 'deploy'), ('D2', 'DRV#1 BIN1', 'purple', 'righting leg (was kick)'), ('D3', 'DRV#1 BIN2', 'grey', 'righting leg (was kick)')):
    w('XIAO ' + pin, to, col, '28 AWG', ['xiao_pins'] + CL + ['drv1'], what)
for pin, to, col, what in (('D8', 'DRV#2 AIN1 (only AIN1: NO bridge to BIN1)', 'purple', 'drills'), ('D9', 'DRV#2 AIN2 (only AIN2: NO bridge to BIN2)', 'grey', 'drills')):
    w('XIAO ' + pin, to, col, '28 AWG', ['xiao_pins'] + CR + ['drv2'], what)
w('DRV#1 AOUT1', 'Deploy N20 + (red)', 'red', '26 AWG', ['drv1', 'deploy_tabs'], 'down to the end-cap tabs under the deploy cradle' + ('. v35 deploys the OTHER way round from v34: key 1 must turn the stub COUNTER-clockwise seen from the left; if not (or on a motor wired as v34), swap 38 and 39' if V35 else ''))
w('DRV#1 AOUT2', 'Deploy N20 - (black)', 'black', '26 AWG', ['drv1', 'deploy_tabs'], '')
LEGR = ['drv1', 'win_d1_L', 'leg_tabs'] if V35 else ['drv1', 'win_d2_L', 'corr_L2', 'corr_L', 'win_d1_L', 'leg_tabs']
w('DRV#1 BOUT1', 'Leg N20 + (red)', 'red', '26 AWG', LEGR, 'forward through disc 1\'s left window to the end-cap tabs under the leg cradle' if V35 else 'left corridor forward, through disc 1, to the end-cap tabs under the leg cradle')
w('DRV#1 BOUT2', 'Leg N20 - (black)', 'black', '26 AWG', LEGR, '')
pod = 12
if V35:   # the v34 pods turned round and swapped sides: the RIGHT pod has the right-hand auger (normal leads), the LEFT one is swapped
    w('DRV#2 AOUT1', 'Drill motor RIGHT + (red)', 'red', '26 AWG', ['drv2', 'cross_f', 'flankR'], 'up through disc 1b\'s top window, out the right tight hole, 25 mm loop into the pod neck at the end cap (silicone wire)', extra=pod + 25)
    w('DRV#2 AOUT2', 'Drill motor RIGHT - (black)', 'black', '26 AWG', ['drv2', 'cross_f', 'flankR'], '', extra=pod + 25)
    w('DRV#2 AOUT1 (same pad as the RIGHT red)', 'Drill motor LEFT - (black)  SWAPPED', 'black', '26 AWG', ['drv2', 'cross_f', 'flankL'], 'reversed on purpose: the pair counter-rotates (the left pod has the left-hand auger)', extra=pod + 25)
    w('DRV#2 AOUT2 (same pad as the RIGHT black)', 'Drill motor LEFT + (red)  SWAPPED', 'red', '26 AWG', ['drv2', 'cross_f', 'flankL'], '', extra=pod + 25)
else:
    w('DRV#2 AOUT1', 'Drill motor LEFT + (red)', 'red', '26 AWG', ['drv2', 'cross', 'flankL'], 'up in front of the deploy motor, out the left tight hole, 25 mm loop into the pod neck at the end cap (silicone wire)', extra=pod + 25)
    w('DRV#2 AOUT2', 'Drill motor LEFT - (black)', 'black', '26 AWG', ['drv2', 'cross', 'flankL'], '', extra=pod + 25)
    w('DRV#2 AOUT1 (same pad as the LEFT red)', 'Drill motor RIGHT - (black)  SWAPPED', 'black', '26 AWG', ['drv2', 'flankR'], 'reversed on purpose: the pair counter-rotates', extra=pod + 25)
    w('DRV#2 AOUT2 (same pad as the LEFT black)', 'Drill motor RIGHT + (red)  SWAPPED', 'red', '26 AWG', ['drv2', 'flankR'], '', extra=pod + 25)
assert len(W) == 45, len(W)
with open(f'{PK}/wire_cut_list.csv', 'w', newline='') as f:
    cw = csv.writer(f); cw.writerow(['#', 'from', 'to', 'colour', 'gauge', 'cut length mm', 'note', 'route'])
    for r in W: cw.writerow([r['n'], r['fr'], r['to'], r['col'], r['g'], r['len'], r['note'], ' > '.join(r['route'])])
json.dump(dict(wires=W, points={k: [round(float(t), 2) for t in v] for k, v in X.items()}), open(f'{PK}/wires.json', 'w'), indent=1, default=str)
tot = {}
for r in W: tot[(r['g'], r['col'])] = tot.get((r['g'], r['col']), 0) + r['len']
print(len(W), 'wires; total mm by gauge/colour:', {f'{g} {c}': v for (g, c), v in sorted(tot.items())})
