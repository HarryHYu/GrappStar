// KEDGE dart host simulator: runs the real kedge_dart.ino (setup() then loop() forever) against the stub Arduino core
// in stubs/ and the simulated hardware in sim_world.cpp, with the dart's HTTP API on a local TCP port.
//
//   ./kedge_sim [--port N] [--reset poweron|software|brownout|panic|task_wdt|int_wdt|wdt|reset_pin|deepsleep|other]
//               [--no-ina] [--no-bno] [--millis-start N]
//
// Simulator control (GET, handled before the sketch sees the request, only for paths under /sim/):
//   /sim/state                      plant positions, motor drive and currents, INA219 registers, IMU, knobs (JSON)
//   /sim/set?key=val[&key=val...]   fault and plant knobs (list in sim_world.cpp, setKnob), replies with /sim/state
//   /sim/launch[?push_ms=40&flight_ms=900&impact=1]   play a launch + flight + landing accelerometer trace
//   /sim/trace?s=dur:ax:ay:az[:roll:pitch],...        play any trace (ms, m/s^2 chip axes, degrees)
//   /sim/disconnect                 the WiFi station leaves (fires ARDUINO_EVENT_WIFI_AP_STADISCONNECTED)
//   /sim/serial?c=<text>            type text into the Serial Monitor input
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
#include "Arduino.h"
#include "WebServer.h"
#include "WiFi.h"
#include "esp_system.h"
#include "sim_world.h"

void setup();   // from kedge_dart.ino
void loop();
extern esp_reset_reason_t simResetReason;

namespace {

void usage() {
  fprintf(stderr, "usage: kedge_sim [--port N] [--reset REASON] [--no-ina] [--no-bno] [--millis-start N]\n");
  exit(2);
}

bool parseReset(const char* s, esp_reset_reason_t& r) {
  static const struct { const char* name; esp_reset_reason_t r; } T[] = {
    {"poweron", ESP_RST_POWERON}, {"software", ESP_RST_SW}, {"brownout", ESP_RST_BROWNOUT}, {"panic", ESP_RST_PANIC},
    {"task_wdt", ESP_RST_TASK_WDT}, {"int_wdt", ESP_RST_INT_WDT}, {"wdt", ESP_RST_WDT}, {"reset_pin", ESP_RST_EXT},
    {"deepsleep", ESP_RST_DEEPSLEEP}, {"other", ESP_RST_UNKNOWN}};
  for (const auto& t : T) if (!strcmp(s, t.name)) { r = t.r; return true; }
  return false;
}

double numArg(WebServer& w, const char* name, double dflt) {
  String v = w.arg(name);
  return v.length() ? atof(v.c_str()) : dflt;
}

// "dur:ax:ay:az[:roll:pitch],..." -> steps; false on a malformed step
bool parseTrace(const std::string& s, std::vector<sim::TraceStep>& out) {
  size_t pos = 0;
  while (pos < s.size()) {
    size_t end = s.find(',', pos); if (end == std::string::npos) end = s.size();
    std::string step = s.substr(pos, end - pos);
    double v[6] = {0, 0, 0, 0, 0, 0}; int n = 0; size_t p = 0;
    while (n < 6 && p <= step.size()) {
      size_t c = step.find(':', p); if (c == std::string::npos) c = step.size();
      std::string f = step.substr(p, c - p); char* e = nullptr;
      v[n++] = strtod(f.c_str(), &e);
      if (f.empty() || *e) return false;
      p = c + 1;
    }
    if (n != 4 && n != 6) return false;
    out.push_back({v[0], v[1], v[2], v[3], v[4], v[5]});
    pos = end + 1;
  }
  return !out.empty();
}

// the /sim/ control side channel; true = handled (every /sim/ path is answered here, never by the sketch)
bool simControl(WebServer& w, const String& uri) {
  const std::string u = uri.str();
  if (u == "/sim/state") { w.send(200, "application/json", String(sim::stateJson())); return true; }
  if (u == "/sim/set") {
    std::string err;
    for (int i = 0; i < w.args(); i++) {
      if (!sim::setKnob(w.argName(i).str(), w.arg(i).str(), err)) { w.send(400, "text/plain", String(err + "\n")); return true; }
    }
    w.send(200, "application/json", String(sim::stateJson()));
    return true;
  }
  if (u == "/sim/launch") {
    sim::playTrace(sim::launchTrace(numArg(w, "push_ms", 40), numArg(w, "flight_ms", 900), numArg(w, "impact", 1) != 0));
    w.send(200, "text/plain", "launch trace playing\n");
    return true;
  }
  if (u == "/sim/trace") {
    std::vector<sim::TraceStep> steps;
    if (!parseTrace(w.arg("s").str(), steps)) { w.send(400, "text/plain", "want s=dur:ax:ay:az[:roll:pitch],...\n"); return true; }
    sim::playTrace(steps);
    w.send(200, "text/plain", "trace playing\n");
    return true;
  }
  if (u == "/sim/disconnect") { WiFi.simFire(ARDUINO_EVENT_WIFI_AP_STADISCONNECTED); w.send(200, "text/plain", "station left\n"); return true; }
  if (u == "/sim/serial") { Serial.inject(w.arg("c").str()); w.send(200, "text/plain", "typed\n"); return true; }
  w.send(404, "text/plain", "unknown simulator path; see sim_main.cpp\n");
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  signal(SIGPIPE, SIG_IGN);
  int port = 8080; uint32_t millisStart = 0;
  sim::BootOptions boot;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    bool more = i + 1 < argc;
    if (a == "--port" && more) port = atoi(argv[++i]);
    else if (a == "--reset" && more) { if (!parseReset(argv[++i], simResetReason)) usage(); }
    else if (a == "--no-ina") boot.inaPresent = false;
    else if (a == "--no-bno") boot.bnoPresent = false;
    else if (a == "--millis-start" && more) millisStart = (uint32_t)strtoul(argv[++i], nullptr, 0);
    else usage();
  }
  setvbuf(stdout, nullptr, _IOLBF, 0);
  sim::clockStart(millisStart);
  sim::boot(boot);
  simSetHttpPort(port);
  simSetHttpHook(simControl);
  setup();
  for (;;) loop();
}
