// Host stub of ArduinoOTA: no network upload on the PC, every call is a no-op.
#pragma once
#include <functional>

class ArduinoOTAClass {
 public:
  typedef std::function<void()> THandlerFunction;
  ArduinoOTAClass& setHostname(const char*) { return *this; }
  ArduinoOTAClass& onStart(THandlerFunction fn) { start_ = fn; return *this; }
  void begin() {}
  void handle() {}
 private:
  THandlerFunction start_;   // kept so the sketch's lambda has an owner, never called
};
inline ArduinoOTAClass ArduinoOTA;
