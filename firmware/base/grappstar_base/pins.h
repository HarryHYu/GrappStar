// GrappStar base station: every ESP32-S3 DevKitC-1-N8 pin in one place. See WIRING.md for the wiring table.
// Rules used to pick them: ADC1 only for analog (ADC2 is unusable with WiFi and is not needed), no strapping pins
// (0, 3, 45, 46), not the native USB pins (19, 20: USB CDC to the Jetson), not the flash pins (26 to 32), not the
// UART0 pins (43, 44), not the RGB LED pins (38 on v1.1 boards, 48 on v1.0 boards). Follows params.SWITCHES except
// SW_HOME (params suggested GPIO38, the RGB LED on v1.1 boards: moved to GPIO21) and SW_INDEX (moved to GPIO35).
#pragma once

// BTS7960 (wiper motor), same pins as the old bench reel and ground station
#define PIN_RPWM      4    // wiper FORWARD (+1, cock / reel wind), PWM 20 kHz
#define PIN_LPWM      5    // wiper BACKWARD (-1, aim), PWM 20 kHz
#define PIN_REN       6    // R_EN, 10 k pull-down to GND on the BTS7960 header
#define PIN_LEN       7    // L_EN, 10 k pull-down to GND on the BTS7960 header
#define PIN_WIPER_IS  1    // R_IS + L_IS tied, 1 k series, 680 R to GND (ADC1_CH0)

// Battery voltage: 3S + through 100 k to the pin, 22 k from the pin to GND (12.6 V reads 2.27 V)
#define PIN_VBAT      2    // ADC1_CH1

// I2C: AS5600 on the main shaft end (address 0x36)
#define PIN_SDA       8
#define PIN_SCL       9

// DRV8833 #3 (Pololu 2130, selector drum N20), channel A only. VM from a buck at about 9 V (DRV8833 VM max 10.8 V).
// SLP is NOT wired (decision D9: the carrier pulls it high itself); nFAULT is not used.
#define PIN_DRUM_IN1  15   // AIN1 (PWM)
#define PIN_DRUM_IN2  16   // AIN2 (PWM)

// Printed contacts: M3 screw head touching an M3 nut, one side to the pin, other side to GND. Internal pull-up:
// closed = LOW. Order = SwId in config.h.
#define PIN_SW_A      10   // SW_DRUM_A
#define PIN_SW_B      11   // SW_DRUM_B
#define PIN_SW_C      12   // SW_DRUM_C
#define PIN_SW_D      13   // SW_DOG_L
#define PIN_SW_E      14   // SW_DOG_R
#define PIN_SW_F      47   // SW_N20 (N20 in mesh; open = hand lever latched out)
#define PIN_SW_G      21   // SW_HOME
#define PIN_SW_H      39   // SW_GAP
#define PIN_SW_I      40   // SW_RAIL_LOW
#define PIN_SW_J      41   // SW_SEATED
#define PIN_SW_K      42   // SW_ARM
#define PIN_SW_L      35   // SW_INDEX (only without the AS5600 magnet, D12; GPIO35 is free on the N8, no PSRAM)
#define N_SW_PINS     12

// Optional HX711 load cell amplifier (found automatically at boot)
#define PIN_HX_DT     17
#define PIN_HX_SCK    18

#define PWM_HZ        20000
#define PWM_BITS      8
#define AS5600_ADDR   0x36
