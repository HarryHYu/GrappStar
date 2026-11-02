"""Ground station wiring diagram. python wiring_gs.py ../pack_gs"""
import sys, subprocess
PK = sys.argv[1]
E = [("LiPo3S", "Fuse20A", "+", "red"), ("Fuse20A", "BTS7960", "B+", "red"), ("LiPo3S", "BTS7960", "- / B-", "black"),
     ("BTS7960", "Wiper", "M+ / M-", "red"), ("ESP32", "BTS7960", "GPIO4 RPWM (F reel)", "purple"), ("ESP32", "BTS7960", "GPIO5 LPWM (B launcher)", "grey40"),
     ("ESP32", "BTS7960", "GPIO6 R_EN, GPIO7 L_EN", "purple"), ("ESP32", "BTS7960", "3V3 VCC, GND", "orange"),
     ("BTS7960", "Divider", "R_IS + L_IS", "brown"), ("Divider", "ESP32", "GPIO1 (current)", "brown"),
     ("ESP32", "AS5600", "3V3 GND, SDA GPIO8, SCL GPIO9, DIR to GND", "blue"),
     ("ESP32", "SW_LOW", "GPIO10 / GND", "darkgreen"), ("ESP32", "SW_COCKED", "GPIO11 / GND", "darkgreen"), ("ESP32", "SW_SEATED", "GPIO12 / GND", "darkgreen"),
     ("ESP32", "HX711", "3V3 / GND, GPIO13 DT, GPIO14 SCK (optional)", "teal"), ("Jetson", "ESP32", "WiFi 192.168.5.1 (GRAPPSTAR-GS)", "grey60"),
     ("BTS7960", "Buck5V", "IN+ / IN- from the B+ / B- screws", "red"), ("Buck5V", "ESP32", "5V via Schottky (stripe to ESP32) + GND, set 5.0 V first", "orange"),
     ("BTSnote", "BTS7960", "", "grey40")]
N = {"BTSnote": "on the BTS7960:\n10 k R_EN-GND, 10 k L_EN-GND\n1000 uF 25 V across B+ / B-", "LiPo3S": "3S LiPo\n(rover battery)", "Fuse20A": "XT60 lead #2\n20 A fuse", "BTS7960": "BTS7960\n43 A H-bridge", "Wiper": "wiper motor\n(main shaft)",
     "ESP32": "ESP32-S3\nDevKitC-1", "Divider": "1 k series, 680 R + 1 uF\nto BTS GND", "AS5600": "AS5600\non the shaft end", "SW_LOW": "SW_LOW\n(rail down)",
     "SW_COCKED": "SW_COCKED\n(sear, left cheek)", "SW_SEATED": "SW_SEATED\n(dart in cradle)", "HX711": "HX711 + load cell\n(calibration, optional)", "Jetson": "Jetson\n(rover brain)", "Buck5V": "MP1584 buck\n5.0 V (or rover 5 V)"}
d = ['digraph G { rankdir=LR; bgcolor=white; node [shape=box, fontname="Arial", fontsize=11, style=rounded]; edge [fontname="Arial", fontsize=9];']
for k, v in N.items():
    d.append('%s [label="%s"];' % (k, v))
for a, b, l, c in E:
    d.append('%s -> %s [label="%s", color="%s", fontcolor="%s", dir=none, penwidth=1.6];' % (a, b, l, c, c))
d.append("}")
open(f"{PK}/wiring_gs.dot", "w").write("\n".join(d))
subprocess.run(["dot", "-Tpng", "-Gdpi=170", f"{PK}/wiring_gs.dot", "-o", f"{PK}/wiring_gs.png"], check=True, stderr=subprocess.DEVNULL)
print("wiring ok")
