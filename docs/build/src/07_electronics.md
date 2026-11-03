# 7. Electronics and wiring (base and rover)

Battery unplugged for every step in this section. Two documents, used in this order:

1. **Base electronics bay, on the bench** (7.2, `firmware/base/WIRING.md` section 7, Parts A to D): set the buck,
   wire the ESP32, BTS7960, DRV8833, sensors and contacts, run the meter checks, first power with the ESP32 on the
   Mac's USB. Then `firmware/base/BENCH_TEST.md`.
2. **The whole rover** (7.1, the system page): the XT60 Y harness, the fuses, the Jetson and video, and the first
   power-on of everything together (its section 4). Do this once the base is on the rover, and again after any
   rewiring.

Where the two differ, the system page adds the rover parts (Y harness, 2 A and 1 A tap fuses, Jetson, RC832); the
ESP32 pins always come from WIRING.md.

## 7.1 The whole system

@include docs/build/SYSTEM_ELECTRICAL.md

![Whole system electrical diagram (bold = battery power, dashed = signal, dotted = radio)](docs/build/system_electrical.png)

## 7.2 Base ESP32 wiring (pin by pin)

@include firmware/base/WIRING.md
