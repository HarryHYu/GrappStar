// Host stub of the ESP32 WiFi soft AP. Nothing is radioed: the HTTP server listens on the PC instead. The simulator
// can fire the "station left" event (GET /sim/disconnect) to exercise the sketch's link-lost handler.
#pragma once
#include <functional>
#include <vector>
#include "Arduino.h"

typedef enum {
  ARDUINO_EVENT_WIFI_READY = 0,
  ARDUINO_EVENT_WIFI_AP_START = 10,
  ARDUINO_EVENT_WIFI_AP_STOP,
  ARDUINO_EVENT_WIFI_AP_STACONNECTED,
  ARDUINO_EVENT_WIFI_AP_STADISCONNECTED,
  ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED,
  ARDUINO_EVENT_MAX = 50
} arduino_event_id_t;
typedef arduino_event_id_t WiFiEvent_t;
typedef union { uint8_t raw[32]; } arduino_event_info_t;
typedef arduino_event_info_t WiFiEventInfo_t;

typedef enum { WIFI_OFF = 0, WIFI_STA = 1, WIFI_AP = 2, WIFI_AP_STA = 3 } wifi_mode_t;

class WiFiClass {
 public:
  typedef std::function<void(arduino_event_id_t, arduino_event_info_t)> WiFiEventFuncCb;
  bool mode(wifi_mode_t m) { mode_ = m; return true; }
  bool softAP(const char* ssid, const char* pass = nullptr) { (void)ssid; (void)pass; return true; }
  IPAddress softAPIP() { return IPAddress(192, 168, 4, 1); }
  int onEvent(WiFiEventFuncCb cb, arduino_event_id_t event = ARDUINO_EVENT_MAX) {
    handlers_.push_back({cb, event}); return (int)handlers_.size();
  }
  void simFire(arduino_event_id_t event) {           // simulator: deliver an event to the matching handlers
    arduino_event_info_t info = {};
    for (auto& h : handlers_) if (h.event == event || h.event == ARDUINO_EVENT_MAX) h.cb(event, info);
  }
 private:
  struct H { WiFiEventFuncCb cb; arduino_event_id_t event; };
  std::vector<H> handlers_;
  wifi_mode_t mode_ = WIFI_OFF;
};
inline WiFiClass WiFi;
