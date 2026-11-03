// Host implementation of the Arduino.h stub: time, pins, LEDC, String formatting, Serial.
#include "Arduino.h"
#include <stdarg.h>
#include <unistd.h>
#include <chrono>
#include <deque>
#include <mutex>
#include <thread>
#include "sim_world.h"

uint32_t millis() { return sim::nowMs(); }
uint32_t micros() { return (uint32_t)sim::nowUs(); }
void delay(uint32_t ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
void delayMicroseconds(uint32_t us) { std::this_thread::sleep_for(std::chrono::microseconds(us)); }

// The motor pins are only ever driven LOW by digitalWrite (before LEDC takes over) and by LEDC afterwards.
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t pin, uint8_t val) { sim::setPin(pin, val ? 255 : 0); }
bool ledcAttach(uint8_t, uint32_t, uint8_t resolution) { return resolution == 8; }   // the plant assumes 8-bit duty
bool ledcWrite(uint8_t pin, uint32_t duty) { sim::setPin(pin, (int)duty); return true; }

// ---- String number formatting (same output as the Arduino core for the cases the sketch uses)
std::string String::fmtU(unsigned long v, unsigned char base) {
  if (base < 2 || base > 36) base = 10;
  char b[72]; int i = sizeof b - 1; b[i] = 0;
  do { int d = (int)(v % base); b[--i] = (char)(d < 10 ? '0' + d : 'a' + d - 10); v /= base; } while (v);
  return std::string(b + i);
}
std::string String::fmtS(long v, unsigned char base) {
  if (base == 10 && v < 0) return "-" + fmtU((unsigned long)(-(v + 1)) + 1, 10);
  return fmtU((unsigned long)v, base);     // like the core: negative numbers in other bases print as unsigned
}
std::string String::fmtF(double v, unsigned char decimals) {
  if (isnan(v)) return "nan";
  if (isinf(v)) return "inf";
  char b[64]; snprintf(b, sizeof b, "%.*f", (int)decimals, v); return b;
}

// ---- Serial
SimSerial Serial;
namespace {
std::mutex serialMu;
std::deque<char> rxQueue;
std::mutex outMu;
}
void SimSerial::begin(unsigned long) {
  // stdin feeds the Serial input when it is a terminal (type a key + Enter, as in the Serial Monitor)
  if (isatty(0)) {
    std::thread([] {
      char c;
      while (::read(0, &c, 1) == 1) { std::lock_guard<std::mutex> l(serialMu); rxQueue.push_back(c); }
    }).detach();
  }
}
void SimSerial::inject(const std::string& s) { std::lock_guard<std::mutex> l(serialMu); for (char c : s) rxQueue.push_back(c); }
int SimSerial::available() { std::lock_guard<std::mutex> l(serialMu); return (int)rxQueue.size(); }
int SimSerial::read() {
  std::lock_guard<std::mutex> l(serialMu);
  if (rxQueue.empty()) return -1;
  char c = rxQueue.front(); rxQueue.pop_front(); return (unsigned char)c;
}
size_t SimSerial::print(const char* s) {
  std::lock_guard<std::mutex> l(outMu);
  size_t n = fwrite(s, 1, strlen(s), stdout); fflush(stdout); return n;
}
size_t SimSerial::printf(const char* fmt, ...) {
  char b[512]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
  return print(b);
}
