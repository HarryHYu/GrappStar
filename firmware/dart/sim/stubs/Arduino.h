// Host stub of the ESP32 Arduino core (core 3.x) for the KEDGE dart simulator.
// Only what kedge_dart.ino uses: timing, pins, LEDC PWM, String, Serial, constrain, the XIAO ESP32S3 Dn pin numbers.
// Like the real Arduino.h it also pulls in FreeRTOS, so the sketch can use semaphores and tasks without an include.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <string>
#include "freertos/FreeRTOS.h"

// ---- build configuration the real board package sets
#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 3
#endif
#ifndef ARDUINO_USB_CDC_ON_BOOT
#define ARDUINO_USB_CDC_ON_BOOT 1          // Tools > USB CDC On Boot: Enabled
#endif
#define PROGMEM
#define PGM_P const char*

// ---- constants
#define HIGH 1
#define LOW 0
#define INPUT 0x01
#define OUTPUT 0x03
#define INPUT_PULLUP 0x05
#define OUTPUT_OPEN_DRAIN 0x13
#define DEC 10
#define HEX 16
#define BIN 2
typedef uint8_t byte;

// Seeed XIAO ESP32S3 pins_arduino.h: Dn label -> GPIO number (constant expressions, as on the real board)
static const uint8_t D0 = 1, D1 = 2, D2 = 3, D3 = 4, D4 = 5, D5 = 6, D6 = 43, D7 = 44, D8 = 7, D9 = 8, D10 = 9;
static const uint8_t SDA = 5, SCL = 6;

template <class T, class L, class H>
inline T constrain(T x, L lo, H hi) { return x < lo ? (T)lo : (x > hi ? (T)hi : x); }

// ---- time: millis() is uint32_t so it wraps exactly like the ESP32 one (the simulator can start it near the wrap)
uint32_t millis();
uint32_t micros();
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);

// ---- GPIO and LEDC (core 3 API: channels are picked by pin). Both feed the simulated motor drivers.
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
bool ledcAttach(uint8_t pin, uint32_t freq, uint8_t resolution);
bool ledcWrite(uint8_t pin, uint32_t duty);

// ---- String: the subset of the Arduino String class the sketch uses, over std::string
class String {
 public:
  String() {}
  String(const char* s) : s_(s ? s : "") {}
  String(const std::string& s) : s_(s) {}
  explicit String(char c) : s_(1, c) {}
  String(unsigned char v, unsigned char base = 10) : s_(fmtU(v, base)) {}
  String(int v, unsigned char base = 10) : s_(fmtS(v, base)) {}
  String(unsigned int v, unsigned char base = 10) : s_(fmtU(v, base)) {}
  String(long v, unsigned char base = 10) : s_(fmtS(v, base)) {}
  String(unsigned long v, unsigned char base = 10) : s_(fmtU(v, base)) {}
  String(float v, unsigned char decimals = 2) : s_(fmtF(v, decimals)) {}
  String(double v, unsigned char decimals = 2) : s_(fmtF(v, decimals)) {}

  unsigned int length() const { return (unsigned int)s_.size(); }
  const char* c_str() const { return s_.c_str(); }
  char operator[](unsigned int i) const { return i < s_.size() ? s_[i] : 0; }
  long toInt() const { return atol(s_.c_str()); }
  float toFloat() const { return (float)atof(s_.c_str()); }
  bool startsWith(const String& p) const { return s_.compare(0, p.s_.size(), p.s_) == 0; }
  int indexOf(char c) const { size_t p = s_.find(c); return p == std::string::npos ? -1 : (int)p; }
  String substring(unsigned int from, unsigned int to = 0xFFFFFFFF) const {
    if (from >= s_.size()) return String();
    return String(s_.substr(from, (to > s_.size() ? s_.size() : to) - from));
  }

  String& operator+=(const String& o) { s_ += o.s_; return *this; }
  String& operator+=(const char* o) { s_ += (o ? o : ""); return *this; }
  String& operator+=(char c) { s_ += c; return *this; }
  bool operator==(const String& o) const { return s_ == o.s_; }
  bool operator==(const char* o) const { return s_ == (o ? o : ""); }
  bool operator!=(const String& o) const { return s_ != o.s_; }
  bool operator!=(const char* o) const { return !(*this == o); }
  const std::string& str() const { return s_; }

 private:
  std::string s_;
  static std::string fmtU(unsigned long v, unsigned char base);
  static std::string fmtS(long v, unsigned char base);
  static std::string fmtF(double v, unsigned char decimals);
};
inline String operator+(const String& a, const String& b) { String r(a); r += b; return r; }
inline String operator+(const String& a, const char* b) { String r(a); r += b; return r; }
inline String operator+(const char* a, const String& b) { String r(a); r += b; return r; }
inline String operator+(const String& a, char b) { String r(a); r += b; return r; }

// ---- IPAddress (what WiFi.softAPIP() returns)
class IPAddress {
 public:
  IPAddress(uint8_t a = 0, uint8_t b = 0, uint8_t c = 0, uint8_t d = 0) : o_{a, b, c, d} {}
  String toString() const { char b[16]; snprintf(b, sizeof b, "%u.%u.%u.%u", o_[0], o_[1], o_[2], o_[3]); return String(b); }
 private:
  uint8_t o_[4];
};

// ---- Serial (USB CDC). Output goes to stdout. Input comes from stdin (when it is a terminal or a pipe) and from the
// simulator's /sim/serial endpoint.
class SimSerial {
 public:
  void begin(unsigned long baud);
  void setTxTimeoutMs(uint32_t ms) { (void)ms; }
  void flush() {}
  explicit operator bool() const { return true; }   // the host is always "connected"
  int available();
  int read();
  size_t print(const char* s);
  size_t print(const String& s) { return print(s.c_str()); }
  size_t print(char c) { char b[2] = {c, 0}; return print(b); }
  size_t print(int v) { return print(String(v)); }
  size_t print(unsigned int v) { return print(String(v)); }
  size_t print(long v) { return print(String(v)); }
  size_t print(unsigned long v) { return print(String(v)); }
  size_t print(double v, int d = 2) { return print(String(v, (unsigned char)d)); }
  size_t print(const IPAddress& ip) { return print(ip.toString()); }
  template <class T> size_t println(const T& v) { size_t n = print(v); return n + print("\n"); }
  size_t println() { return print("\n"); }
  size_t printf(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
  void inject(const std::string& s);                 // simulator: queue bytes as if typed in the Serial Monitor
};
extern SimSerial Serial;
