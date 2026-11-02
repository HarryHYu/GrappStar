import json, numpy as np, math, csv, sys
OUT=sys.argv[1]; PK=sys.argv[2]
J=json.load(open(OUT+'/parts.json')); P=J['P']
def bb(n):
    v=np.load(f'{OUT}/{n}.npz')['v']; return v.min(0), v.max(0)
def c(n, fx=0.5, fy=0.5, fz=1.0):
    a,b=bb(n); return a+(b-a)*np.array([fx,fy,fz])
X=dict(
 xiao_pins=c('xiao',0.5,0.5), xiao_5v=c('xiao',0.95,0.1), drv1=c('drv8833_1'), drv2=c('drv8833_2'), drv3=c('drv8833_3'),
 ina=c('ina219'), bno=c('bno085'), mp=c('mp1584'), vtx=c('vtx',0.1,0.5), bms_f=c('bms_2s',0.05,0.5,0.0), bms_r=c('bms_2s',0.95,0.5,0.0),
 cam=c('camera',1.0,0.5,0.5), dep=c('n20_deploy',0.05,0.5,0.5),
 kick_hole=np.array([P['SADDLE_X0']+1.3+P['N20_L']-4, 0, P['R_BAY']-1]),
 foot_hole=np.array([42, P['KICK_Y']-1, 16]),
 flankL=np.array([P['DRILL_X']-9, 18.5, 9]), flankR=np.array([P['DRILL_X']-9, -18.5, 9]),
 tail=np.array([P['L']-6, 0, -16]), d4=np.array([P['X_CELL'][1], 0, -7.5]), d3=np.array([P['X_CELL'][0], 0, 0]),
)
def L(*pts, extra=0):
    d=sum(np.abs(np.array(pts[i+1])-np.array(pts[i])).sum() for i in range(len(pts)-1))
    return int(math.ceil((d*1.15+30+extra)/5.0)*5)
W=[]  # (id, from, to, colour, gauge, length, note)
def w(fr, to, col, g, ln, note=''): W.append([len(W)+1, fr, to, col, g, ln, note])
# --- battery and power
w('Rear plate, cell 1 (-)', 'BMS B-', 'black', '22 AWG', L(X['d4'], X['bms_r']), 'solder to the plate tab before it goes in its recess')
w('Front spring, cell 1 (+)', 'Front spring, cell 2 (-)', 'red', '22 AWG', 45, 'the series link: carries the full battery current; glue both springs in the front recess')
w('Front spring pair (cell 1 + / cell 2 -)', 'BMS BM', 'white', '26 AWG', L(X['d3'], X['bms_f']), 'the two springs are joined by a short wire; this is the midpoint sense')
w('Rear plate, cell 2 (+)', 'BMS B+', 'red', '22 AWG', L(X['d4'], X['bms_r']), '')
w('BMS B-', 'Charge socket pin 1 (tail window)', 'black', '26 AWG', L(X['bms_r'], X['tail']), 'JST-XH 3-pin, balance charge')
w('BMS BM', 'Charge socket pin 2', 'white', '26 AWG', L(X['bms_f'], X['tail']), '')
w('BMS B+', 'Charge socket pin 3', 'red', '26 AWG', L(X['bms_r'], X['tail']), '')
w('BMS P+', 'Power loop socket pin 1 (tail window)', 'red', '22 AWG', L(X['bms_r'], X['tail']), 'JST-XH 2-pin; the shorting plug is the ON switch')
w('Power loop socket pin 2', 'INA219 VIN+', 'red', '22 AWG', L(X['tail'], X['ina']), 'main + after the loop')
w('INA219 VIN+ (same pad)', 'MP1584 IN+', 'red', '22 AWG', L(X['ina'], X['mp']), 'buck taps before the shunt, so INA219 reads motor current only')
w('INA219 VIN-', 'DRV#1 VM', 'red', '22 AWG', L(X['ina'], X['drv1']), 'motor rail 6.0-8.4 V')
w('DRV#1 VM', 'DRV#2 VM', 'red', '22 AWG', L(X['drv1'], X['drv2']), 'daisy chain')
w('DRV#2 VM', 'DRV#3 VM', 'red', '22 AWG', L(X['drv2'], X['drv3']), 'forward into the sled (keel)')
w('BMS P-', 'MP1584 IN-', 'black', '22 AWG', L(X['bms_r'], X['mp']), 'main ground')
w('MP1584 IN- (same pad)', 'DRV#1 GND', 'black', '22 AWG', L(X['mp'], X['drv1']), '')
w('DRV#1 GND', 'DRV#2 GND', 'black', '22 AWG', L(X['drv1'], X['drv2']), '')
w('DRV#2 GND', 'DRV#3 GND', 'black', '22 AWG', L(X['drv2'], X['drv3']), '')
w('MP1584 OUT+ (5.04 V)', 'XIAO 5V via a Schottky diode (stripe toward the XIAO)', 'orange', '24 AWG', L(X['mp'], X['xiao_5v']), 'set and check 5.04 V BEFORE connecting; the diode stops the buck back-feeding USB (1N5817 or SS14, in heat shrink)')
w('MP1584 OUT-', 'XIAO GND', 'black', '24 AWG', L(X['mp'], X['xiao_5v']), '')
w('MP1584 OUT+', 'VTX 5V in', 'orange', '26 AWG', L(X['mp'], X['vtx']), '')
w('MP1584 OUT-', 'VTX GND', 'black', '26 AWG', L(X['mp'], X['vtx']), '')
w('VTX 5V in (same pad)', 'Camera + (red)', 'orange', '28 AWG', L(X['vtx'], X['cam']), 'runs forward along the top through the disc notches')
w('VTX GND (same pad)', 'Camera - (black)', 'black', '28 AWG', L(X['vtx'], X['cam']), '')
w('Camera video (yellow)', 'VTX video in', 'yellow', '28 AWG', L(X['vtx'], X['cam']), 'twist with the camera ground')
w('XIAO 3V3', 'DRV#1 SLP / nSLEEP / EEP pad (NOT VCC or VIN)', 'red/white', '28 AWG', L(X['xiao_pins'], X['drv1']), 'wakes the driver. Check first: if the pad you picked beeps to VIN/VM on the meter, it is the motor supply: do NOT connect 3V3 to it')
w('DRV#1 SLP', 'DRV#2 SLP', 'red/white', '28 AWG', L(X['drv1'], X['drv2']), '')
w('DRV#2 SLP', 'DRV#3 SLP', 'red/white', '28 AWG', L(X['drv2'], X['drv3']), '')
w('XIAO 3V3', 'INA219 VCC', 'red/white', '28 AWG', L(X['xiao_pins'], X['ina']), '')
w('INA219 VCC', 'BNO085 VIN', 'red/white', '28 AWG', L(X['ina'], X['bno']), '')
w('XIAO GND', 'INA219 GND', 'black', '28 AWG', L(X['xiao_pins'], X['ina']), '')
w('INA219 GND', 'BNO085 GND', 'black', '28 AWG', L(X['ina'], X['bno']), '')
# --- signals
w('XIAO D4', 'INA219 SDA', 'blue', '28 AWG', L(X['xiao_pins'], X['ina']), 'I2C SDA')
w('INA219 SDA', 'BNO085 SDA', 'blue', '28 AWG', L(X['ina'], X['bno']), '')
w('XIAO D5', 'INA219 SCL', 'green', '28 AWG', L(X['xiao_pins'], X['ina']), 'I2C SCL')
w('INA219 SCL', 'BNO085 SCL', 'green', '28 AWG', L(X['ina'], X['bno']), '')
w('XIAO D0', 'DRV#1 AIN1', 'purple', '28 AWG', L(X['xiao_pins'], X['drv1']), 'deploy')
w('XIAO D1', 'DRV#1 AIN2', 'grey', '28 AWG', L(X['xiao_pins'], X['drv1']), 'deploy')
w('XIAO D2', 'DRV#1 BIN1', 'purple', '28 AWG', L(X['xiao_pins'], X['drv1']), 'kick')
w('XIAO D3', 'DRV#1 BIN2', 'grey', '28 AWG', L(X['xiao_pins'], X['drv1']), 'kick')
w('XIAO D8', 'DRV#2 AIN1 + BIN1 (bridge the two pads)', 'purple', '28 AWG', L(X['xiao_pins'], X['drv2']), 'drills (moved off D6/D7: those twitch at reset)')
w('XIAO D9', 'DRV#2 AIN2 + BIN2 (bridge the two pads)', 'grey', '28 AWG', L(X['xiao_pins'], X['drv2']), 'drills')
w('XIAO D6', 'DRV#3 AIN1', 'purple', '28 AWG', L(X['xiao_pins'], X['drv3']), 'foot (slow worm, harmless if it twitches at reset)')
w('XIAO D7', 'DRV#3 AIN2', 'grey', '28 AWG', L(X['xiao_pins'], X['drv3']), 'foot')
# --- motors
w('DRV#1 AOUT1', 'Deploy N20 + (red)', 'red', '26 AWG', L(X['drv1'], X['dep']), 'motor sits in the sled cradle')
w('DRV#1 AOUT2', 'Deploy N20 - (black)', 'black', '26 AWG', L(X['drv1'], X['dep']), '')
w('DRV#1 BOUT1', 'Kick N20 + (red)', 'red', '26 AWG', L(X['drv1'], X['kick_hole'], X['kick_hole'] + [0, 0, 8]), 'out through the hole in the saddle floor')
w('DRV#1 BOUT2', 'Kick N20 - (black)', 'black', '26 AWG', L(X['drv1'], X['kick_hole'], X['kick_hole'] + [0, 0, 8]), '')
pod = 15
w('DRV#2 AOUT1', 'Drill motor LEFT + (red)', 'red', '26 AWG', L(X['drv2'], X['flankL'], extra=pod + 30), 'out the left flank hole, 30 mm loop for the pod swing')
w('DRV#2 AOUT2', 'Drill motor LEFT - (black)', 'black', '26 AWG', L(X['drv2'], X['flankL'], extra=pod + 30), '')
w('DRV#2 BOUT1', 'Drill motor RIGHT - (black)  SWAPPED', 'black', '26 AWG', L(X['drv2'], X['flankR'], extra=pod + 30), 'reversed on purpose: the pair counter-rotates')
w('DRV#2 BOUT2', 'Drill motor RIGHT + (red)  SWAPPED', 'red', '26 AWG', L(X['drv2'], X['flankR'], extra=pod + 30), '')
leg = P['KICK_POLE_L'] + 40
w('DRV#3 AOUT1', 'Foot N20 + (red)', 'red', '26 AWG', L(X['drv3'], X['foot_hole'], extra=leg + 60), 'out the skin hole, 60 mm loop round the hub, through the pole channel')
w('DRV#3 AOUT2', 'Foot N20 - (black)', 'black', '26 AWG', L(X['drv3'], X['foot_hole'], extra=leg + 60), 'silicone wire, it flexes every deploy')
with open(f'{PK}/wire_cut_list.csv', 'w', newline='') as f:
    cw = csv.writer(f); cw.writerow(['#', 'from', 'to', 'colour', 'gauge', 'cut length mm', 'note']); cw.writerows(W)
tot = {}
for r in W: tot[(r[3], r[4])] = tot.get((r[3], r[4]), 0) + r[5]
json.dump(dict(wires=W, totals={f'{k[1]} {k[0]}': v for k, v in sorted(tot.items(), key=lambda x: x[0][1])}), open(f'{PK}/wires.json', 'w'), indent=1)
print(len(W), 'wires'); [print(r) for r in W[:6]]; print(tot)
