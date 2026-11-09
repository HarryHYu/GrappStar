// The simulated world: clock, motor plant, INA219, BNO085 trace. See sim_world.h.
//
// Motor plant, dart v34 (currents are rail mA at PWM duty 255 and scale with duty / 255; travel speed scales the same way):
//   kick    the RIGHTING LEG on the old kick channel: position 0..3800 ms of travel, 0 = folded (groove floor), 3800 =
//           its far stop (the bed in the nose top). free 25, lifting the dart 70, stalled at either end (or kick_jam) 185.
//   deploy  position 0..1400 (0 = pods up / stowed). free 30, rolling the dart 70, stalled at either end (or deploy_jam) 185.
//   drills  140 spinning in air, 420 digging (forward while the pods are low enough to reach the soil), 200 reversing,
//           700 when jammed (drill_jam).
//   foot    not wired on v34 (DRV#2 B): draws nothing.
// Attitude (knob pose=<tilt deg>[&side=-1|1]): the BNO085 reads gravity from it while no trace plays. Belly up (tilt 110
// or more) the leg meets the ground at 400 and lifts the dart until 2400, where it goes over onto a flank at 40 deg
// (side -1); on a flank (tilt 15 or more) the pods going down roll it onto its belly over 600 of travel. leg_air=1: the
// leg meets nothing (swings in air to its far stop).
// The motor pins are the XIAO ESP32S3 GPIOs from WIRING.md, written here independently of kedge_logic.h so a pin map
// mistake in the sketch shows up as the wrong motor moving.
#include "sim_world.h"
#include <math.h>
#include <stdio.h>
#include <algorithm>
#include <chrono>
#include <mutex>
#include <string>

namespace sim {
namespace {

std::recursive_mutex mu;   // the world lock (recursive: stateJson calls the helpers that also lock)
typedef std::lock_guard<std::recursive_mutex> Lock;

// ---------------------------------------------------------------------------------------------------------------- clock
std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
uint32_t millisAtBoot = 0;

// ---------------------------------------------------------------------------------------------------------------- plant
enum { DEPLOY, KICK, DRILLS, FOOT, NMOT };
const char* const MOTOR_NAME[NMOT] = {"deploy", "kick", "drills", "foot"};
const int PIN_A[NMOT] = {1, 3, 7, 43}, PIN_B[NMOT] = {2, 4, 8, 44};   // D0/D1, D2/D3, D8/D9, D6/D7

const double KICK_TRAVEL = 3800, DEPLOY_TRAVEL = 1400, DEPLOY_DIG = 900;
const double KICK_FREE = 25, KICK_LIFT = 70, STALL = 185, DEPLOY_FREE = 30, DEPLOY_ROLL = 70;
const double DRILL_AIR = 140, DRILL_DIG = 420, DRILL_REV = 200, DRILL_JAM = 700;
const double LEG_LIFT_AT = 400, LEG_OVER_AT = 2400, FLIP_TO = 40, ROLL_SPAN = 600;

int duty[64];                                 // per GPIO (0..48 on the ESP32-S3), duty 0..255
double kickPos = 0, deployPos = 0;
// attitude: tilt off belly down (deg), side (+1 / -1: which flank is down), belly up, the tilt a flank rest started from,
// pods travel where a roll began; attOn: the BNO085 reads this attitude (set by the pose knob, off when a trace plays)
double tilt = 0, flankRest = 0, invRest = 175, rollFrom = -1; int side = 1; bool inv = false, attOn = false;
uint64_t lastAdvanceUs = 0;
struct Knobs {
  bool inaDead = false, legAir = false, drillJam = false, kickJam = false, deployJam = false, imuDead = false;
  double railV = 7.6, accelNoise = 0.02, soilAt = -1;   // soilAt: v4.1 soil, see soilStuck
  bool drillGearReverse = false;                // v32 as built: direct drive (a tip planetary would reverse the sleeves)
  uint32_t imuXferUs = 0;
} K;
int wdtTrips = 0;

uint64_t deployOnUs = 0; int deployLastDir = 0;
int& pin(int gpio) { static int junk; return gpio >= 0 && gpio < 64 ? duty[gpio] : (junk = 0); }
// direction and duty of one DRV8833 channel: PWM on one input = run, both low = coast, both high = brake (stopped).
// The direction returned is the PART's (for the drills the sleeves'; a reversing gear stage in the tip, which v32 does
// not have, is the knob drill_gear_reverse); the motor shaft's own direction is motorDir.
void motorDir(int m, int& dir, int& d) {
  int a = pin(PIN_A[m]), b = pin(PIN_B[m]);
  // slow decay (firmware v4): one input high, the other PWM inverted; the old one-input PWM is still understood
  if (a == 255 && b < 255) { dir = +1; d = 255 - b; } else if (b == 255 && a < 255) { dir = -1; d = 255 - a; }
  else if (a && !b) { dir = +1; d = a; } else if (b && !a) { dir = -1; d = b; } else { dir = 0; d = 0; }
}
void drive(int m, int& dir, int& d) { motorDir(m, dir, d); if (m == DRILLS && K.drillGearReverse) dir = -dir; }
// soil (soil_at knob): past soilAt the pods only move for 300 ms after each (re)start of the deploy, with the drills cutting
bool soilStuck(uint64_t now) {
  int dd, d, dr, drd; drive(DEPLOY, dd, d); drive(DRILLS, dr, drd);
  return !inv && flankRest == 0 && K.soilAt >= 0 && dd > 0 && deployPos >= K.soilAt && deployPos < DEPLOY_TRAVEL && (now - deployOnUs > 300000 || dr <= 0);
}
bool lifting() { return inv && !K.legAir && kickPos >= LEG_LIFT_AT; }
void setPose(double t, int sd) { tilt = t; side = sd; inv = t >= 110; flankRest = (!inv && t >= 15) ? t : 0; if (inv) invRest = t; rollFrom = -1; attOn = true; }
// move the legs for the time since the last call, with the drive that was in force during that time
void advance() {
  uint64_t now = nowUs();
  double dt = (now - lastAdvanceUs) / 1000.0; lastAdvanceUs = now;
  int dir, d;
  drive(KICK, dir, d);
  if (!K.kickJam) kickPos = std::min(KICK_TRAVEL, std::max(0.0, kickPos + dir * d / 255.0 * dt));
  drive(DEPLOY, dir, d);
  if (dir != deployLastDir) { deployLastDir = dir; deployOnUs = now; }
  if (dir > 0 && !inv && flankRest > 0 && rollFrom < 0) rollFrom = deployPos;   // the pods meet the ground and push
  if (!K.deployJam && !soilStuck(now)) deployPos = std::min(DEPLOY_TRAVEL, std::max(0.0, deployPos + dir * d / 255.0 * dt));
  // attitude (the v34 righting, as the study sim sees it)
  if (inv) {
    if (lifting() && kickPos >= LEG_OVER_AT) { inv = false; tilt = flankRest = FLIP_TO; side = -1; rollFrom = -1; }
    else tilt = !lifting() ? invRest : invRest - (invRest - 90) * (kickPos - LEG_LIFT_AT) / (LEG_OVER_AT - LEG_LIFT_AT);
  } else if (flankRest > 0 && rollFrom >= 0) {
    double f = 1 - (deployPos - rollFrom) / ROLL_SPAN; f = f < 0 ? 0 : f > 1 ? 1 : f;
    tilt = flankRest * f;
    if (tilt < 5) { tilt = 0; flankRest = 0; rollFrom = -1; }   // on its belly: it stays there
  }
}
double motorMa(int m) {
  int dir, d; drive(m, dir, d);
  if (!dir) return 0;
  double ma = 0;
  switch (m) {
    case KICK: {
      bool stalled = K.kickJam || (dir > 0 && kickPos >= KICK_TRAVEL) || (dir < 0 && kickPos <= 0);
      ma = stalled ? STALL : (dir > 0 && lifting() ? KICK_LIFT : KICK_FREE);
      break;
    }
    case DEPLOY: {
      bool stalled = K.deployJam || soilStuck(nowUs()) || (dir > 0 && deployPos >= DEPLOY_TRAVEL) || (dir < 0 && deployPos <= 0);
      ma = stalled ? STALL : (dir > 0 && rollFrom >= 0 ? DEPLOY_ROLL : DEPLOY_FREE);
      break;
    }
    case DRILLS: ma = K.drillJam ? DRILL_JAM : dir < 0 ? DRILL_REV : (deployPos >= DEPLOY_DIG ? DRILL_DIG : DRILL_AIR); break;
    case FOOT: ma = 0; break;                  // v34: nothing on DRV#2 B
  }
  return ma * (d / 255.0) * (d / 255.0);   // slow decay: supply current ~ duty squared
}
double totalMa() { advance(); double s = 0; for (int m = 0; m < NMOT; m++) s += motorMa(m); return s; }

// ---------------------------------------------------------------------------------------------------------------- INA219
// 0.1 ohm shunt. Registers: 0x00 config, 0x01 shunt (10 uV LSB), 0x02 bus ((mV / 4) << 3 | CNVR | OVF), 0x03 power,
// 0x04 current (0.1 mA LSB when calibration == 4096, else 0 in this model), 0x05 calibration. Power-on: config 0x399F,
// calibration 0 (an INA219 that browned out reads 0 mA until it is calibrated again).
const uint8_t INA_ADDR = 0x40, BNO_ADDR = 0x4A;
const uint16_t INA_CONFIG_POR = 0x399F;
uint16_t inaConfig = INA_CONFIG_POR, inaCal = 0; uint8_t inaPtr = 0;
bool bnoPresent = true;
void inaPowerOn() { inaConfig = INA_CONFIG_POR; inaCal = 0; inaPtr = 0; }
uint16_t inaReg(uint8_t r) {
  double ma = totalMa();
  double rangeMv = 40.0 * (1 << ((inaConfig >> 11) & 3));          // PGA /1 /2 /4 /8 = 40 / 80 / 160 / 320 mV
  double shuntMv = ma * 0.1;                                       // I (mA) x 0.1 ohm
  bool ovf = fabs(shuntMv) > rangeMv;
  double sh = std::max(-rangeMv, std::min(rangeMv, shuntMv)) * 100;   // 10 uV LSB
  int16_t cur = inaCal == 4096 ? (int16_t)std::max(-32768.0, std::min(32767.0, std::round(ma * 10.0))) : 0;
  switch (r) {
    case 0x00: return inaConfig;
    case 0x01: return (uint16_t)(int16_t)lround(sh);
    case 0x02: return (uint16_t)(((uint16_t)(K.railV * 1000 / 4) << 3) | 0x2 | (ovf ? 1 : 0));
    case 0x03: return (uint16_t)(inaCal == 4096 ? lround(fabs(ma) * K.railV / 2.0) : 0);   // 2 mW LSB (20 x current LSB)
    case 0x04: return (uint16_t)cur;
    case 0x05: return inaCal;
    default: return 0;
  }
}

// ---------------------------------------------------------------------------------------------------------------- IMU
struct Sensor { uint32_t intervalUs = 0; uint64_t nextUs = 0; };
Sensor accel, grv;
const size_t IMU_FIFO_MAX = 32;                 // reports the chip holds for the host
std::vector<ImuReport> fifo;                    // due, not yet read, oldest first
uint32_t imuDropped = 0;
bool imuResetPending = false;
std::vector<TraceStep> trace; uint64_t traceT0Us = 0;
TraceStep rest = {0, 0, 0, 9.80665, 0, 0};
uint32_t noiseState = 12345;

double noise() {                                // deterministic, uniform in -1..1
  noiseState = noiseState * 1664525u + 1013904223u;
  return (noiseState >> 8) / 8388608.0 - 1.0;
}
// accel and roll/pitch of the trace at time t (us since boot)
TraceStep traceAt(uint64_t t) {
  if (trace.empty() || t < traceT0Us) return rest;
  double ms = (t - traceT0Us) / 1000.0, r0 = rest.roll, p0 = rest.pitch;
  for (const TraceStep& s : trace) {
    if (ms < s.durMs) {
      double f = s.durMs > 0 ? ms / s.durMs : 1;
      return {0, s.ax, s.ay, s.az, r0 + (s.roll - r0) * f, p0 + (s.pitch - p0) * f};
    }
    ms -= s.durMs; r0 = s.roll; p0 = s.pitch;
  }
  return trace.back();
}
bool traceRunning(uint64_t t) {
  double total = 0; for (const TraceStep& s : trace) total += s.durMs;
  return !trace.empty() && (t - traceT0Us) / 1000.0 < total;
}
// at rest with the plant's attitude: gravity for tilt (roll about the nose axis, chip x), roll = side * tilt
TraceStep attStep() { double r = side * tilt * M_PI / 180, g = 9.80665; return {0, 0, g * sin(r), g * cos(r), side * tilt, 0}; }
ImuReport makeReport(uint8_t sensor, uint64_t t) {
  TraceStep s = attOn && !traceRunning(t) ? attStep() : traceAt(t);
  ImuReport r; r.tUs = t; r.sensor = sensor;
  if (sensor == 0x01) {                        // SH2_ACCELEROMETER, m/s^2, gravity included
    r.v[0] = (float)(s.ax + K.accelNoise * noise()); r.v[1] = (float)(s.ay + K.accelNoise * noise());
    r.v[2] = (float)(s.az + K.accelNoise * noise()); r.v[3] = 0;
  } else {                                     // SH2_GAME_ROTATION_VECTOR from roll / pitch, yaw 0
    double cr = cos(s.roll * M_PI / 360), sr = sin(s.roll * M_PI / 360), cp = cos(s.pitch * M_PI / 360), sp = sin(s.pitch * M_PI / 360);
    r.v[0] = (float)(sr * cp); r.v[1] = (float)(cr * sp); r.v[2] = (float)(-sr * sp); r.v[3] = (float)(cr * cp);
  }
  return r;
}
void collect(Sensor& s, uint8_t id, uint64_t now, std::vector<ImuReport>& out) {
  if (!s.intervalUs) return;
  if (!s.nextUs || K.imuDead) { s.nextUs = now + s.intervalUs; return; }
  int n = 0;
  while (s.nextUs <= now && n < 80) { out.push_back(makeReport(id, s.nextUs)); s.nextUs += s.intervalUs; n++; }
  if (s.nextUs <= now) s.nextUs = now + s.intervalUs;   // far behind (debugger pause): the chip drops reports too
}

std::string fmt(const char* f, double v) { char b[64]; snprintf(b, sizeof b, f, v); return b; }

}  // namespace

// ---------------------------------------------------------------------------------------------------------------- API
void clockStart(uint32_t start) { t0 = std::chrono::steady_clock::now(); millisAtBoot = start; }
uint64_t nowUs() {
  return (uint64_t)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();
}
uint32_t nowMs() { return millisAtBoot + (uint32_t)(nowUs() / 1000); }

void setPin(int gpio, int d) { Lock l(mu); advance(); pin(gpio) = d < 0 ? 0 : (d > 255 ? 255 : d); }

bool i2cProbe(uint8_t addr) {
  Lock l(mu);
  return (addr == INA_ADDR && !K.inaDead) || (addr == BNO_ADDR && bnoPresent);
}
bool i2cWrite(uint8_t addr, const uint8_t* d, int n) {
  Lock l(mu);
  if (!i2cProbe(addr)) return false;
  if (addr != INA_ADDR || n < 1) return true;
  inaPtr = d[0];
  if (n >= 3) {
    uint16_t v = (uint16_t)(d[1] << 8 | d[2]);
    if (inaPtr == 0x00) { if (v & 0x8000) inaPowerOn(); else inaConfig = v; }   // bit 15 = reset
    else if (inaPtr == 0x05) inaCal = v & 0xFFFE;                              // bit 0 reads back 0
  }
  return true;
}
int i2cRead(uint8_t addr, uint8_t* d, int n) {
  Lock l(mu);
  if (!i2cProbe(addr)) return 0;
  if (addr != INA_ADDR) { for (int i = 0; i < n; i++) d[i] = 0; return n; }
  uint16_t v = inaReg(inaPtr);
  for (int i = 0; i < n; i++) d[i] = i == 0 ? v >> 8 : (i == 1 ? v & 0xFF : 0);
  return n;
}

bool imuPresent() { Lock l(mu); return bnoPresent; }
bool imuEnable(uint8_t sensor, uint32_t intervalUs) {
  Lock l(mu);
  if (!bnoPresent || K.imuDead) return false;
  Sensor* s = sensor == 0x01 ? &accel : (sensor == 0x08 ? &grv : nullptr);
  if (!s) return false;
  s->intervalUs = intervalUs; s->nextUs = 0;
  return true;
}
bool imuTakeReset() { Lock l(mu); bool r = imuResetPending; imuResetPending = false; return r; }
std::vector<ImuReport> imuNextTransfer() {
  Lock l(mu);
  std::vector<ImuReport> out;
  if (!bnoPresent) return out;
  uint64_t now = nowUs();
  collect(accel, 0x01, now, fifo);
  collect(grv, 0x08, now, fifo);
  std::stable_sort(fifo.begin(), fifo.end(), [](const ImuReport& a, const ImuReport& b) { return a.tUs < b.tUs; });
  if (fifo.size() > IMU_FIFO_MAX) { imuDropped += (uint32_t)(fifo.size() - IMU_FIFO_MAX); fifo.erase(fifo.begin(), fifo.end() - IMU_FIFO_MAX); }
  size_t n = 0;
  while (n < fifo.size() && fifo[n].tUs == fifo[0].tUs) n++;
  out.assign(fifo.begin(), fifo.begin() + n);
  fifo.erase(fifo.begin(), fifo.begin() + n);
  return out;
}
uint32_t imuTransferUs() { Lock l(mu); return K.imuXferUs; }

void playTrace(const std::vector<TraceStep>& steps) {
  Lock l(mu);
  uint64_t now = nowUs();
  rest = attOn ? attStep() : traceAt(now); rest.durMs = 0;   // the old trace's (or attitude's) values become the starting point
  trace = steps; traceT0Us = now; attOn = false; // a trace takes over the IMU (the pose knob gives it back to the plant)
}
// gravity in chip axes for a roll / pitch attitude (at rest)
static TraceStep still(double ms, double roll, double pitch) {
  double r = roll * M_PI / 180, p = pitch * M_PI / 180, g = 9.80665;
  return {ms, -g * sin(p), g * sin(r) * cos(p), g * cos(r) * cos(p), roll, pitch};
}
std::vector<TraceStep> launchTrace(double pushMs, double flightMs, bool impact) {
  std::vector<TraceStep> s;
  s.push_back(still(100, 0, 0));                       // on the launcher
  s.push_back({pushMs, 0, 0, 100, 0, 0});              // launch push, about 10 g (the chip clips at 8 g)
  s.push_back({flightMs, 0, 0, 1.2, 0, 25});           // ballistic: about 0.12 g, pitching nose down
  if (impact) {
    s.push_back({25, 30, 10, 110, 5, -3});             // impact (clipped)
    s.push_back({40, 0, 2, 25, 5, -3});                // bounce, about 2.6 g
  } else {
    s.push_back({300, 0, 0, 12, 5, -3});               // soft landing in grass: about 1.2 g, no spike
  }
  s.push_back(still(1000, 5, -3));                     // at rest on the ground, tilted
  return s;
}

void boot(const BootOptions& o) {
  Lock l(mu);
  bnoPresent = o.bnoPresent; K.inaDead = !o.inaPresent;
  inaPowerOn(); lastAdvanceUs = nowUs();
}

static bool parseBool(const std::string& v, bool& out) {
  if (v == "1" || v == "true" || v == "on") { out = true; return true; }
  if (v == "0" || v == "false" || v == "off") { out = false; return true; }
  return false;
}
// Knobs:
//   ina_dead=0|1     INA219 stops answering (NACK). Back to 0 = it powers up again uncalibrated, like after a brownout.
//   ina_brownout=1   INA219 loses its configuration and calibration (reads 0 mA until the sketch re-initialises it)
//   drill_jam, kick_jam, deploy_jam=0|1   that motor is jammed (stall current, no travel)
//   pose=<tilt deg>[&side=-1|1]  the dart rests this far off belly down (180 = belly up, 40 = on a flank); the BNO085
//                    reads it and the leg and the pods move it (see the top of this file). side alone flips the flank.
//   leg_air=0|1      belly up in a hollow: the leg meets nothing and swings in air to its far stop
//   rail_v=<volts>   motor rail voltage read by the INA219 (default 7.6)
//   kick_pos=<0..3800>, deploy_pos=<0..1400>        put the leg / the pods somewhere
//   imu_dead=0|1     BNO085 stops sending reports
//   bno_reset=1      BNO085 resets: all reports off until re-enabled, wasReset() true once
//   imu_xfer_us=<us> simulated I2C time of each sh2_service() call that moves data (default 0)
//   accel_noise=<m/s^2>  accelerometer noise amplitude (default 0.02)
//   drill_gear_reverse=0|1  a reversing gear stage in the sleeve tips (v32 as built: none, default 0)
//   soil_at=<0..1400>  v4.1 soil: past it the pods move only 300 ms after each deploy (re)start, with the drills on (-1 = off)
bool setKnob(const std::string& key, const std::string& v, std::string& err) {
  Lock l(mu);
  advance();
  bool b = false; char* end = nullptr; double x = strtod(v.c_str(), &end);
  bool num = !v.empty() && end && *end == 0;
  bool* flag = key == "drill_jam" ? &K.drillJam : key == "kick_jam" ? &K.kickJam
             : key == "deploy_jam" ? &K.deployJam : key == "leg_air" ? &K.legAir : nullptr;
  if (flag) { if (!parseBool(v, b)) { err = key + ": want 0 or 1"; return false; } *flag = b; return true; }
  if (key == "ina_dead") {
    if (!parseBool(v, b)) { err = "ina_dead: want 0 or 1"; return false; }
    if (K.inaDead && !b) inaPowerOn();
    K.inaDead = b; return true;
  }
  if (key == "ina_brownout") { inaPowerOn(); return true; }
  if (key == "imu_dead") { if (!parseBool(v, b)) { err = "imu_dead: want 0 or 1"; return false; } K.imuDead = b; return true; }
  if (key == "bno_reset") { accel = Sensor(); grv = Sensor(); fifo.clear(); imuResetPending = true; return true; }
  if (!num) { err = "unknown knob or not a number: " + key + "=" + v; return false; }
  if (key == "rail_v") { K.railV = x; return true; }
  if (key == "pose") { setPose(std::max(0.0, std::min(180.0, x)), side); return true; }
  if (key == "side") { side = x < 0 ? -1 : 1; attOn = true; return true; }
  if (key == "kick_pos") { kickPos = std::max(0.0, std::min(KICK_TRAVEL, x)); return true; }
  if (key == "deploy_pos") { deployPos = std::max(0.0, std::min(DEPLOY_TRAVEL, x)); return true; }
  if (key == "imu_xfer_us") { K.imuXferUs = (uint32_t)std::max(0.0, x); return true; }
  if (key == "accel_noise") { K.accelNoise = std::max(0.0, x); return true; }
  if (key == "soil_at") { K.soilAt = x; return true; }
  if (key == "drill_gear_reverse") { K.drillGearReverse = x != 0; return true; }
  err = "unknown knob: " + key; return false;
}

void noteWatchdogTrip() { Lock l(mu); wdtTrips++; }

std::string stateJson() {
  Lock l(mu);
  double total = totalMa();
  std::string j = "{\"t_ms\":" + std::to_string(nowMs()) + ",\"kick_pos\":" + fmt("%.0f", kickPos) +
                  ",\"deploy_pos\":" + fmt("%.0f", deployPos) + ",\"tilt\":" + fmt("%.0f", tilt) + ",\"motors\":{";
  for (int m = 0; m < NMOT; m++) {
    int dir, d; drive(m, dir, d);
    int md, mdd; motorDir(m, md, mdd);
    j += std::string(m ? "," : "") + "\"" + MOTOR_NAME[m] + "\":{\"dir\":" + std::to_string(dir) + ",\"motor_dir\":" + std::to_string(md) + ",\"duty\":" +
         std::to_string(d) + ",\"mA\":" + fmt("%.1f", motorMa(m)) + "}";
  }
  j += "},\"current_mA\":" + fmt("%.1f", total) + ",\"rail_v\":" + fmt("%.3f", K.railV);
  j += ",\"ina\":{\"present\":" + std::string(K.inaDead ? "false" : "true") + ",\"config\":" + std::to_string(inaConfig) +
       ",\"cal\":" + std::to_string(inaCal) + "}";
  j += ",\"imu\":{\"present\":" + std::string(bnoPresent ? "true" : "false") + ",\"dead\":" + (K.imuDead ? "true" : "false") +
       ",\"accel_interval_us\":" + std::to_string(accel.intervalUs) + ",\"grv_interval_us\":" + std::to_string(grv.intervalUs) +
       ",\"trace_running\":" + (traceRunning(nowUs()) ? "true" : "false") + ",\"xfer_us\":" + std::to_string(K.imuXferUs) + ",\"waiting\":" + std::to_string(fifo.size()) +
       ",\"dropped\":" + std::to_string(imuDropped) + "}";
  j += ",\"knobs\":{\"drill_jam\":" + std::string(K.drillJam ? "true" : "false") + ",\"leg_air\":" + (K.legAir ? "true" : "false") +
       ",\"kick_jam\":" + (K.kickJam ? "true" : "false") + ",\"deploy_jam\":" + (K.deployJam ? "true" : "false") +
       ",\"accel_noise\":" + fmt("%.3f", K.accelNoise) + "}";
  j += ",\"video_pin\":" + std::to_string(duty[9] ? 1 : 0);   // D10 (GPIO9): the optional video power switch
  j += ",\"wdt_trips\":" + std::to_string(wdtTrips) + "}";
  return j;
}

}  // namespace sim
