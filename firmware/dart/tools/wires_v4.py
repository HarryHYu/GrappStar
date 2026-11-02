#!/usr/bin/env python3
"""Dart wire cut list v4 (DRV8833 #3 removed) and wiring diagram.

Same length rule as kedge/wires.py (Manhattan route x 1.15 + 30 mm, rounded up to 5 mm) on the v30 geometry
(kedge/out30, the default build). Writes into firmware/dart/wiring/:
  wire_cut_list.csv, wires.json, changes_from_v29.csv, wiring.dot, wiring.svg, wiring.png
Usage: python3 firmware/dart/tools/wires_v4.py [path/to/kedge/out30] [outdir]
"""
import csv, json, math, os, re, subprocess, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "kedge", "out30")
PK = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, "..", "wiring")
os.makedirs(PK, exist_ok=True)
P = json.load(open(os.path.join(OUT, "parts.json")))["P"]


def bb(n):
    v = np.load(os.path.join(OUT, f"{n}.npz"))["v"]
    return v.min(0), v.max(0)


def c(n, fx=0.5, fy=0.5, fz=1.0):
    a, b = bb(n)
    return a + (b - a) * np.array([fx, fy, fz])


X = dict(
    xiao_pins=c("xiao", 0.5, 0.5), xiao_5v=c("xiao", 0.95, 0.1), drv1=c("drv8833_1"), drv2=c("drv8833_2"),
    ina=c("ina219"), bno=c("bno085"), mp=c("mp1584"), vtx=c("vtx", 0.1, 0.5), bms_f=c("bms_2s", 0.05, 0.5, 0.0),
    bms_r=c("bms_2s", 0.95, 0.5, 0.0), cam=c("camera", 1.0, 0.5, 0.5), dep=c("n20_deploy", 0.05, 0.5, 0.5),
    kick_hole=np.array([P["SADDLE_X0"] + 1.3 + P["N20_L"] - 4, 0, P["R_BAY"] - 1]),
    foot_hole=np.array([42, P["KICK_Y"] - 1, 16]),
    flankL=np.array([P["DRILL_X"] - 9, 18.5, 9]), flankR=np.array([P["DRILL_X"] - 9, -18.5, 9]),
    tail=np.array([P["L"] - 6, 0, -16]), d4=np.array([P["X_CELL"][1], 0, -7.5]), d3=np.array([P["X_CELL"][0], 0, 0]),
)


def L(*pts, extra=0):
    d = sum(np.abs(np.array(pts[i + 1]) - np.array(pts[i])).sum() for i in range(len(pts) - 1))
    return int(math.ceil((d * 1.15 + 30 + extra) / 5.0) * 5)


W = []  # [id, from, to, colour, gauge, length, note, v29 id or ""]


def w(fr, to, col, g, ln, note="", old=""):
    W.append([len(W) + 1, fr, to, col, g, ln, note, old])


# --- battery and power (unchanged except the DRV#3 links are gone)
w("Rear plate, cell 1 (-)", "BMS B-", "black", "22 AWG", L(X["d4"], X["bms_r"]), "solder to the plate tab before it goes in its recess", 1)
w("Front spring, cell 1 (+)", "Front spring, cell 2 (-)", "red", "22 AWG", 45, "the series link: carries the full battery current; glue both springs in the front recess", 2)
w("Front spring pair (cell 1 + / cell 2 -)", "BMS BM", "white", "26 AWG", L(X["d3"], X["bms_f"]), "midpoint sense", 3)
w("Rear plate, cell 2 (+)", "BMS B+", "red", "22 AWG", L(X["d4"], X["bms_r"]), "", 4)
w("BMS B-", "Charge socket pin 1 (tail window)", "black", "26 AWG", L(X["bms_r"], X["tail"]), "JST-XH 3-pin, balance charge", 5)
w("BMS BM", "Charge socket pin 2", "white", "26 AWG", L(X["bms_f"], X["tail"]), "", 6)
w("BMS B+", "Charge socket pin 3", "red", "26 AWG", L(X["bms_r"], X["tail"]), "", 7)
w("BMS P+", "Power loop socket pin 1 (tail window)", "red", "22 AWG", L(X["bms_r"], X["tail"]), "JST-XH 2-pin; the shorting plug is the ON switch", 8)
w("Power loop socket pin 2", "INA219 VIN+", "red", "22 AWG", L(X["tail"], X["ina"]), "main + after the loop", 9)
w("INA219 VIN+ (same pad)", "MP1584 IN+", "red", "22 AWG", L(X["ina"], X["mp"]), "buck taps before the shunt, so INA219 reads motor current only", 10)
w("INA219 VIN-", "DRV#1 VM", "red", "22 AWG", L(X["ina"], X["drv1"]), "motor rail 6.0-8.4 V", 11)
w("DRV#1 VM", "DRV#2 VM", "red", "22 AWG", L(X["drv1"], X["drv2"]), "daisy chain ends here (no DRV#3)", 12)
w("BMS P-", "MP1584 IN-", "black", "22 AWG", L(X["bms_r"], X["mp"]), "main ground", 14)
w("MP1584 IN- (same pad)", "DRV#1 GND", "black", "22 AWG", L(X["mp"], X["drv1"]), "", 15)
w("DRV#1 GND", "DRV#2 GND", "black", "22 AWG", L(X["drv1"], X["drv2"]), "", 16)
w("MP1584 OUT+ (5.04 V)", "XIAO 5V via a Schottky diode (stripe toward the XIAO)", "orange", "24 AWG", L(X["mp"], X["xiao_5v"]), "set and check 5.04 V BEFORE connecting; 1N5817 or SS14 in heat shrink", 18)
w("MP1584 OUT-", "XIAO GND", "black", "24 AWG", L(X["mp"], X["xiao_5v"]), "", 19)
w("MP1584 OUT+", "VTX 5V in", "orange", "26 AWG", L(X["mp"], X["vtx"]), "", 20)
w("MP1584 OUT-", "VTX GND", "black", "26 AWG", L(X["mp"], X["vtx"]), "", 21)
w("VTX 5V in (same pad)", "Camera + (red)", "orange", "28 AWG", L(X["vtx"], X["cam"]), "runs forward along the top through the disc notches", 22)
w("VTX GND (same pad)", "Camera - (black)", "black", "28 AWG", L(X["vtx"], X["cam"]), "", 23)
w("Camera video (yellow)", "VTX video in", "yellow", "28 AWG", L(X["vtx"], X["cam"]), "twist with the camera ground", 24)
w("XIAO 3V3", "DRV#1 SLP / nSLEEP / EEP pad (NOT VCC or VIN)", "red/white", "28 AWG", L(X["xiao_pins"], X["drv1"]), "wakes the driver; meter check first (WIRING.md step 6)", 25)
w("DRV#1 SLP", "DRV#2 SLP", "red/white", "28 AWG", L(X["drv1"], X["drv2"]), "", 26)
w("XIAO 3V3", "INA219 VCC", "red/white", "28 AWG", L(X["xiao_pins"], X["ina"]), "", 28)
w("INA219 VCC", "BNO085 VIN", "red/white", "28 AWG", L(X["ina"], X["bno"]), "", 29)
w("XIAO GND", "INA219 GND", "black", "28 AWG", L(X["xiao_pins"], X["ina"]), "", 30)
w("INA219 GND", "BNO085 GND", "black", "28 AWG", L(X["ina"], X["bno"]), "", 31)
# --- signals
w("XIAO D4", "INA219 SDA", "blue", "28 AWG", L(X["xiao_pins"], X["ina"]), "I2C SDA", 32)
w("INA219 SDA", "BNO085 SDA", "blue", "28 AWG", L(X["ina"], X["bno"]), "", 33)
w("XIAO D5", "INA219 SCL", "green", "28 AWG", L(X["xiao_pins"], X["ina"]), "I2C SCL", 34)
w("INA219 SCL", "BNO085 SCL", "green", "28 AWG", L(X["ina"], X["bno"]), "", 35)
w("XIAO D0", "DRV#1 AIN1", "purple", "28 AWG", L(X["xiao_pins"], X["drv1"]), "deploy", 36)
w("XIAO D1", "DRV#1 AIN2", "grey", "28 AWG", L(X["xiao_pins"], X["drv1"]), "deploy", 37)
w("XIAO D2", "DRV#1 BIN1", "purple", "28 AWG", L(X["xiao_pins"], X["drv1"]), "kick", 38)
w("XIAO D3", "DRV#1 BIN2", "grey", "28 AWG", L(X["xiao_pins"], X["drv1"]), "kick", 39)
w("XIAO D8", "DRV#2 AIN1 (only AIN1: NO bridge to BIN1)", "purple", "28 AWG", L(X["xiao_pins"], X["drv2"]), "drills", 40)
w("XIAO D9", "DRV#2 AIN2 (only AIN2: NO bridge to BIN2)", "grey", "28 AWG", L(X["xiao_pins"], X["drv2"]), "drills", 41)
w("XIAO D6", "DRV#2 BIN1", "purple", "28 AWG", L(X["xiao_pins"], X["drv2"]), "foot (slow worm, harmless if it twitches at reset)", 42)
w("XIAO D7", "DRV#2 BIN2", "grey", "28 AWG", L(X["xiao_pins"], X["drv2"]), "foot", 43)
# --- motors
w("DRV#1 AOUT1", "Deploy N20 + (red)", "red", "26 AWG", L(X["drv1"], X["dep"]), "motor sits in the sled cradle", 44)
w("DRV#1 AOUT2", "Deploy N20 - (black)", "black", "26 AWG", L(X["drv1"], X["dep"]), "", 45)
w("DRV#1 BOUT1", "Kick N20 + (red)", "red", "26 AWG", L(X["drv1"], X["kick_hole"], X["kick_hole"] + [0, 0, 8]), "out through the hole in the saddle floor", 46)
w("DRV#1 BOUT2", "Kick N20 - (black)", "black", "26 AWG", L(X["drv1"], X["kick_hole"], X["kick_hole"] + [0, 0, 8]), "", 47)
pod = 15
w("DRV#2 AOUT1", "Drill motor LEFT + (red)", "red", "26 AWG", L(X["drv2"], X["flankL"], extra=pod + 30), "out the left flank hole, 30 mm loop for the pod swing", 48)
w("DRV#2 AOUT2", "Drill motor LEFT - (black)", "black", "26 AWG", L(X["drv2"], X["flankL"], extra=pod + 30), "", 49)
w("DRV#2 AOUT1 (same pad as the LEFT red)", "Drill motor RIGHT - (black)  SWAPPED", "black", "26 AWG", L(X["drv2"], X["flankR"], extra=pod + 30), "reversed on purpose: the pair counter-rotates", 50)
w("DRV#2 AOUT2 (same pad as the LEFT black)", "Drill motor RIGHT + (red)  SWAPPED", "red", "26 AWG", L(X["drv2"], X["flankR"], extra=pod + 30), "", 51)
leg = P["KICK_POLE_L"] + 40
w("DRV#2 BOUT1", "Foot N20 + (red)", "red", "26 AWG", L(X["drv2"], X["foot_hole"], extra=leg + 60), "forward along the top through the disc notches, out the skin hole, 60 mm loop round the hub, through the pole channel", 52)
w("DRV#2 BOUT2", "Foot N20 - (black)", "black", "26 AWG", L(X["drv2"], X["foot_hole"], extra=leg + 60), "silicone wire, it flexes every deploy", 53)

# v29 list (kedge/CLAUDE.md Appendix A) for the change table
V29 = {}
try:
    txt = open(os.path.join(ROOT, "kedge", "CLAUDE.md")).read()
    block = txt.split("## APPENDIX A")[1].split("```")[1]
    for row in csv.reader(block.strip().splitlines()[1:]):
        V29[int(row[0])] = row
except Exception as e:  # noqa
    print("warning: could not read the v29 list:", e)

with open(os.path.join(PK, "wire_cut_list.csv"), "w", newline="") as f:
    cw = csv.writer(f)
    cw.writerow(["#", "from", "to", "colour", "gauge", "cut length mm", "note", "v29 #"])
    cw.writerows(W)

changes = []
used = {r[7] for r in W}
for k, row in sorted(V29.items()):
    if k not in used:
        changes.append([k, "REMOVE", f"{row[1]} -> {row[2]}", "DRV#3 is no longer on the dart"])
for r in W:
    o = V29.get(r[7])
    if not o:
        continue
    if (o[1], o[2]) != (r[1], r[2]) or int(o[5]) != r[5]:
        changes.append([r[7], f"CHANGE (now #{r[0]})", f"was {o[1]} -> {o[2]} ({o[5]} mm); now {r[1]} -> {r[2]} ({r[5]} mm)", r[6]])
with open(os.path.join(PK, "changes_from_v29.csv"), "w", newline="") as f:
    cw = csv.writer(f)
    cw.writerow(["v29 #", "action", "detail", "note"])
    cw.writerows(changes)

tot = {}
for r in W:
    tot[(r[3], r[4])] = tot.get((r[3], r[4]), 0) + r[5]
json.dump(dict(wires=W, totals={f"{k[1]} {k[0]}": v for k, v in sorted(tot.items(), key=lambda x: x[0][1])}, changes=changes),
          open(os.path.join(PK, "wires.json"), "w"), indent=1)

# --- diagram
COL = {"black": "#222222", "red": "#d0312d", "white": "#999999", "orange": "#e58a1f", "yellow": "#d4b106", "red/white": "#e07070",
       "blue": "#1f5fd0", "green": "#1e9e48", "purple": "#8a3fd1", "grey": "#777777"}
NODES = ["XIAO", "DRV#1", "DRV#2", "INA219", "BNO085", "MP1584", "BMS", "VTX", "Camera", "Charge socket", "Power loop socket",
         "Deploy N20", "Kick N20", "Foot N20", "Drill motor LEFT", "Drill motor RIGHT"]
LABEL = {"XIAO": "XIAO ESP32S3", "DRV#1": "DRV8833 #1\nA = deploy, B = kick", "DRV#2": "DRV8833 #2\nA = both drills, B = foot",
         "INA219": "INA219 (0x40)", "BNO085": "BNO085 (0x4A)", "MP1584": "MP1584 buck 5.04 V", "BMS": "2S BMS", "Cell": "2 x 18350 (2S)"}


def node(s):
    s = s.split("(")[0].strip()
    for k in NODES + ["Rear plate", "Front spring"]:
        if s.startswith(k):
            return "Cell" if k in ("Rear plate", "Front spring") else k
    return s


def pad(s):
    s = re.sub(r"\(.*?\)", "", s).strip()
    n = node(s)
    p = s[len(n):].strip() if s.startswith(n) else s
    return p.replace("SWAPPED", "").replace("via a Schottky diode", "(diode)").strip() or "+"


lines = ['digraph G { rankdir=LR; splines=true; nodesep=0.3; ranksep=1.2; bgcolor="white";',
         'label="KEDGE dart wiring v4: DRV8833 #3 removed (its pocket stays empty). Every wire: battery unplugged."; labelloc=t; fontname="Arial"; fontsize=16;',
         'node [shape=box, style="rounded,filled", fillcolor="#f2f2f2", fontname="Arial", fontsize=11, color="#555555"];',
         'edge [fontname="Arial", fontsize=8, penwidth=1.4, arrowhead=none];']
for n in ["Cell"] + NODES:
    lab = LABEL.get(n, n).replace("\n", "\\n")
    fill = "#ffe9c7" if n == "DRV#2" else "#f2f2f2"
    lines.append(f'"{n}" [label="{lab}", fillcolor="{fill}"];')
lines.append('{rank=same; "Cell"; "Charge socket"; "Power loop socket";} '
             '{rank=same; "Deploy N20"; "Kick N20"; "Drill motor LEFT"; "Drill motor RIGHT"; "Foot N20"; "Camera";}')
for r in W:
    a, b = node(r[1]), node(r[2])
    if a == b:
        continue
    lab = f"{r[0]}: {pad(r[1])} - {pad(r[2])}"
    lines.append(f'"{a}" -> "{b}" [minlen={2 if {a, b} == {"DRV#1", "DRV#2"} else 1}, color="{COL.get(r[3], "#444")}", label="{lab}"];')
lines.append("}")
dot = os.path.join(PK, "wiring.dot")
open(dot, "w").write("\n".join(lines))
subprocess.run(["dot", "-Tsvg", dot, "-o", os.path.join(PK, "wiring.svg")], check=True)
subprocess.run(["dot", "-Tpng", "-Gdpi=150", dot, "-o", os.path.join(PK, "wiring.png")], check=True)
print(len(W), "wires;", len(changes), "changes from v29; totals:", {f"{k[1]} {k[0]}": v for k, v in tot.items()})
for ch in changes:
    print("  ", ch)
