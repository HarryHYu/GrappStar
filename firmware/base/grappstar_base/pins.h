// GrappStar base station: every ESP32-S3 DevKitC-1-N8 pin in one place. See WIRING.md for the wiring table.
// Rules used to pick them: ADC1 only for analog (ADC2 is unusable with WiFi and is not needed), no strapping pins
// (0, 3, 45, 46), not the native USB pins (19, 20: USB CDC to the Jetson), not the flash pins (26 to 32), not the
// UART0 pins (43, 44), not the RGB LED pins (38 on v1.1 boards, 48 on v1.0 boards).
#pragma once

// BTS7960 (wiper motor), same pins as the old bench reel and ground station
#define PIN_RPWM      4    // wiper direction +1 (PWM 20 kHz)
#define PIN_LPWM      5    // wiper direction -1 (PWM 20 kHz)
#define PIN_REN       6    // R_EN, 10 k pull-down to GND on the BTS7960 header
#define PIN_LEN       7    // L_EN, 10 k pull-down to GND on the BTS7960 header
#define PIN_WIPER_IS  1    // R_IS + L_IS tied, 1 k series, 680 R to GND (ADC1_CH0)

// Battery voltage: 3S + through 100 k to the pin, 22 k from the pin to GND (12.6 V reads 2.27 V)
#define PIN_VBAT      2    // ADC1_CH1

// I2C: AS5600 on the wiper output shaft (address 0x36)
#define PIN_SDA       8
#define PIN_SCL       9

// DRV8833 #3 (selector drum N20), channel A only. VM from a buck at about 9 V (DRV8833 VM max 10.8 V)
#define PIN_DRUM_IN1  17   // AIN1 (PWM)
#define PIN_DRUM_IN2  18   // AIN2 (PWM)
#define PIN_DRUM_SLP  21   // nSLEEP / SLP / EEP: high = driver on. 10 k pull-down so it stays off while the ESP32 boots
#define PIN_DRUM_FLT  47   // nFAULT if the carrier has the pad (open drain, internal pull-up). Leave open if not

// Printed contact switches: M3 screw head touching an M3 nut, one side to the pin, other side to GND.
// Internal pull-up: closed = LOW. Which contact sits on which pin: Cfg::swBit in config.h (SwId order A..H).
#define PIN_SW_A      11
#define PIN_SW_B      12
#define PIN_SW_C      13
#define PIN_SW_D      14
#define PIN_SW_E      39
#define PIN_SW_F      40
#define PIN_SW_G      41
#define PIN_SW_H      42

// Optional HX711 load cell amplifier (found automatically at boot)
#define PIN_HX_DT     16
#define PIN_HX_SCK    15

#define PWM_HZ        20000
#define PWM_BITS      8
#define AS5600_ADDR   0x36
