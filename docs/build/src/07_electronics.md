# 7. Electronics and wiring (base and rover)

Battery unplugged for every step in this section. Build in this order: the power harness (Y lead, fuses), the base
electronics bay, the sensors and contacts, the Jetson and video. Then do the first power-on sequence (7.1 section 4)
before anything is connected to a motor.

## 7.1 The whole system

@include docs/build/SYSTEM_ELECTRICAL.md

![Whole system electrical diagram (bold = battery power, dashed = signal, dotted = radio)](docs/build/system_electrical.png)

## 7.2 Base ESP32 wiring (pin by pin)

@include firmware/base/WIRING.md
