# 8. Flashing both ESP32s

Rules for both boards: battery unplugged (dart: power loop plug out) before the USB cable goes in. Use a data USB
cable. Flash the dart XIAO before it goes into the hull (run sheet section 5).

## 8.1 Base: ESP32-S3 DevKitC-1-N8

@include firmware/base/README.md

## 8.2 Dart: XIAO ESP32S3

Arduino IDE: board XIAO_ESP32S3, Tools > USB CDC On Boot: Enabled (the sketch refuses to compile otherwise).
Library: Adafruit BNO08x (the INA219 library is no longer needed). Open `firmware/dart/kedge_dart/kedge_dart.ino`. After the first USB
upload you can upload over WiFi (port "kedge").

@include firmware/dart/README.md
