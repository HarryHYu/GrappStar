// Host stub of ArduinoOTA: no network upload on the PC, every call is a no-op.
#pragma once
#include <functional>

typedef int ota_error_t;
class ArduinoOTAClass {
 public:
  typedef std::function<void()> THandlerFunction;
  ArduinoOTAClass& setHostname(const char*) { return *this; }
  ArduinoOTAClass& onStart(THandlerFunction fn) { start_ = fn; return *this; }
  ArduinoOTAClass& onError(std::function<void(ota_error_t)> fn) { err_ = fn; return *this; }
  void begin() {}
  void handle() {}
 private:
  THandlerFunction start_;   // kept so the sketch's lambda has an owner, never called
  std::function<void(ota_error_t)> err_;
};
inline ArduinoOTAClass ArduinoOTA;
