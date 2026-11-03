// KEDGE dart control logic v4: plain C++, no Arduino calls, so it is unit-tested on a PC (see ../test/test_logic.cpp).
// The sketch supplies the hardware through the Io struct and feeds IMU samples in.
//
// v4 changes (3 Nov 2026): DRV8833 #3 removed from the dart (it goes to the base station). DRV#2 channel A drives both
// drill motors in parallel, DRV#2 channel B drives the foot. New: release sequence, phase, fault codes, flight and
// landing detection, 12 s flight log ring buffer, per-motor stall, sequence time limits, low rail and sensor-loss
// handling.
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

namespace kedge {

static const char* const FW_VERSION = "kedge-dart-v4";

enum { DEPLOY = 0, KICK = 1, DRILLS = 2, FOOT = 3, NMOT = 4 };
static inline const char* motorName(int m) {
  static const char* n[] = {"deploy", "kick", "drills", "foot"};
  return (m >= 0 && m < NMOT) ? n[m] : "?";
}

// ---------------------------------------------------------------------------------------------------------------
// Pin map. GPIO numbers of the Seeed XIAO ESP32S3 (Dn labels in the strings). The sketch checks these numbers against
// the Arduino Dn macros at compile time, the PC test checks the layout.
struct PinMap { int gpioA, gpioB; const char* labelA; const char* labelB; int drv; char ch; };
static constexpr PinMap PINS[NMOT] = {
  {1, 2, "D0", "D1", 1, 'A'},     // deploy N20: DRV#1 AIN1 / AIN2
  {3, 4, "D2", "D3", 1, 'B'},     // kick N20:   DRV#1 BIN1 / BIN2
  {7, 8, "D8", "D9", 2, 'A'},     // drills:     DRV#2 AIN1 / AIN2, BOTH drill motors in parallel on AOUT1 / AOUT2
  {43, 44, "D6", "D7", 2, 'B'},   // foot N20:   DRV#2 BIN1 / BIN2 (GPIO43/44 twitch at reset: harmless on a slow worm)
};
static constexpr int PIN_SDA_GPIO = 5, PIN_SCL_GPIO = 6;   // D4 / D5: INA219 0x40 + BNO085 0x4A
static const int N_DRIVERS = 2;                        // DRV#3 is no longer on the dart
static const float DRV_CHANNEL_MAX_MA = 1200;          // DRV8833 continuous per channel (2 A peak, internal OCP)

// ---------------------------------------------------------------------------------------------------------------
struct Io {
  void (*setMotor)(int motor, int dir, int duty);   // dir -1/0/+1, duty 0..255
  float (*current_mA)();                            // total motor-rail current (INA219); NAN when the read failed
  float (*rail_V)();                                // motor rail volts; <= 0 when unknown
  void (*log)(const char* line);
};

// Replace the current numbers with bench values (BENCH_TEST.md steps 3 to 6, written in at step 10). Currents are the total read by
// the INA219 on the motor rail at 2S.
struct Tuning {
  uint32_t serialRunMs = 1500, wifiDeadmanMs = 350, wifiMaxRunMs = 500;
  float tripMa = 2000; uint32_t tripHoldMs = 10;        // all four stalled at 8.4 V is about 1700 mA
  float stallMa[NMOT] = {150, 150, 600, 450};           // deploy, kick, drill PAIR, foot (stall 180 / 180 / 700 / 550)
  float loadMa[NMOT] = {40, 70, 420, 130};              // normal working current (in soil), used when 3+ run at once
  float freeMa[NMOT] = {30, 30, 70, 45};               // free running current (bench, 8 V)
  float refV = 8.0f;                                    // the currents above are at this rail voltage; they scale with it
  float kickContactMa = 60;                             // foot auger tip meets the ground (kick free ~30 mA)
  uint32_t kickMaxMs = 15000, footAfterMs = 1500, deployMaxMs = 6000, drillAfterMs = 3000;
  uint32_t drillLeadMs = 3000, footLeadMs = 1500, footOutMs = 3000;   // release: mirror of the anchor timings
  uint32_t autoMaxMs = 35000, releaseMaxMs = 35000;     // whole-sequence limits
  uint32_t surgeIgnoreMs = 250, stallHoldMs = 200, contactHoldMs = 150;
  uint32_t podsMinMs = 700, kickBackMinMs = 1500;      // shortest real travel before an end-stop stall is believed
  uint32_t kickPlantMaxMs = 6000;                       // kick on the ground but no housing stall seen: carry on after this
  float riseFrac = 0.5f;                                // a worm stall in a sequence must also raise the total current
  float noCurrentFrac = 0.4f; uint32_t noCurrentMs = 500;   // a lone motor under 40 % of its free current: open wire, or a
                                                        // short chopped by the DRV8833 protection (about 9 mA); worms 20+
  float lowRailV = 5.6f; uint32_t lowRailMs = 500;      // 2S at 2.8 V per cell under load
  float armMinV = 6.6f;                                 // refuse to arm a flat battery (3.3 V per cell at rest)
  uint32_t armMaxMs = 15UL * 60 * 1000;                 // an arm expires after 15 min without a launch
  uint32_t podLeadMs = 300;                             // pods steps start with the deploy worm alone (open wire, at the stop)
  uint32_t kickSwingMs = 4300;                          // full kick swing at full duty (BENCH_TEST step 5): travel bookkeeping
  uint32_t manualMaxMs = 8000;                          // a manual motor (held key or repeats) never runs longer
  uint32_t manualMaxNoInaMs = 2000;                     // ...and without current sensing (no stall detection)
  int inaFailLimit = 3;                                 // consecutive failed current reads = sensor lost
};

enum Fault { F_NONE = 0, F_OVERCURRENT, F_KICK_TIMEOUT, F_SEQ_TIMEOUT, F_DRILL_JAM, F_FOOT_JAM, F_CURRENT_SENSOR, F_LOW_RAIL,
             F_BROWNOUT, F_WATCHDOG, F_NO_CURRENT, F_KICK_JAM, F_NFAULT };
static inline const char* faultName(int f) {
  static const char* n[] = {"", "OVERCURRENT", "KICK_TIMEOUT", "SEQ_TIMEOUT", "DRILL_JAM", "FOOT_JAM", "CURRENT_SENSOR",
                            "LOW_RAIL", "BROWNOUT", "WATCHDOG", "NO_CURRENT", "KICK_JAM"};
  return (f > 0 && f < F_NFAULT) ? n[f] : "";
}

enum Seq { IDLE, K_SWING, K_PLANT, F_AFTER, D_DEPLOY, D_AFTER, DONE, ABORTED, R_DRILL, R_PODS, R_FOOT, R_KICK, RELEASED, NSEQ };
static inline const char* seqName(int s) {
  static const char* n[] = {"idle", "kick swing", "kick + foot planting", "foot finishing", "deploy + drills", "drills finishing",
                            "anchored", "aborted", "release: drills out", "release: drills out + pods up", "release: foot out",
                            "release: kick back + foot out", "released"};
  return (s >= 0 && s < NSEQ) ? n[s] : "?";
}
enum Phase { P_IDLE, P_ARMED, P_FLIGHT, P_LANDED, P_ANCHORING, P_ANCHORED, P_RELEASING, P_FAULT };
static inline const char* phaseName(int p) {
  static const char* n[] = {"IDLE", "ARMED", "FLIGHT", "LANDED", "ANCHORING", "ANCHORED", "RELEASING", "FAULT"};
  return (p >= 0 && p <= P_FAULT) ? n[p] : "?";
}

// ---------------------------------------------------------------------------------------------------------------
// Launch and landing detector on |a| in g (BNO085 accelerometer, gravity included, saturates near 8 g).
//   launch:  |a| > launchG continuously for launchHoldMs (contract C2: 4 g for 20 ms)
//   landing: after free fall (|a| < freeFallG for freeFallMs in total) or pushOverMs after launch with |a| < launchG,
//            and minFlightMs, an impact over impactG, then still (|a| inside stillLo..stillHi) for stillMs. C2 said 6 g;
//            3.5 g is used because the sensor clips and filters short impacts (API.md). Landing time = the LAST impact.
//   fallbacks: no impact seen but still for softStillMs after the flight ("still"); nothing for flightTimeoutMs after
//            launch ("timeout": also covers a silent IMU, checked from tick()).
struct Flight {
  enum St { WAIT = 0, FLYING, IMPACTED, LANDED };
  enum Ev { NONE = 0, LAUNCH, LAND };
  float launchG = 4, impactG = 3.5f, freeFallG = 0.6f, stillLo = 0.85f, stillHi = 1.15f;
  uint32_t launchHoldMs = 20, sampleSlackMs = 5, freeFallMs = 60, pushOverMs = 400, minFlightMs = 400, stillMs = 1000, softStillMs = 2000, flightTimeoutMs = 15000;
  int st = WAIT; bool above = false, quiet = false, stillOn = false, launched = false;
  uint32_t tAbove = 0, tLaunch = 0, tImpact = 0, tStill = 0, tLand = 0, ffMs = 0, tPrev = 0;
  // a dropped dart is in free fall before it hits: a push within dropWindowMs after free fall (|a| < dropG for at least
  // dropFallMs) is not a launch. Rail vibration, tilts and the cocking jerk never read that low, so they do not matter.
  float dropG = 0.5f; uint32_t dropFallMs = 40, dropWindowMs = 400, ffStart = 0, lastFall = 0; bool inFall = false, hadFall = false, pushBad = false;
  void armedAt(uint32_t) { inFall = hadFall = pushBad = false; }
  const char* how = "";                       // impact | still | timeout | stop
  void reset() { st = WAIT; above = quiet = stillOn = launched = false; tAbove = tLaunch = tImpact = tStill = tLand = ffMs = tPrev = 0; how = ""; }
  bool fallback() const { return st == LANDED && (strcmp(how, "still") == 0 || strcmp(how, "timeout") == 0); }
  bool flying() const { return st == FLYING || st == IMPACTED; }
  int land(uint32_t t, const char* why) { st = LANDED; tLand = t; how = why; return LAND; }
  int update(float a, uint32_t now) {
    if (!(a == a)) return NONE;               // NaN sample
    switch (st) {
      case WAIT:
        if (a > launchG) {
          if (!above) {
            above = true; tAbove = now;
            if (inFall && (int32_t)(now - ffStart) >= (int32_t)dropFallMs) { hadFall = true; lastFall = now; }
            inFall = false;
            pushBad = hadFall && (int32_t)(now - lastFall) < (int32_t)dropWindowMs;   // a drop hitting the ground, not a shot
          }
          if (pushBad) break;
          // samples above 4 g spanning hold - one sample period = the push lasted about 20 ms
          if (now - tAbove + sampleSlackMs >= launchHoldMs) { st = FLYING; launched = true; tLaunch = tAbove; quiet = false; stillOn = false; ffMs = 0; tPrev = now; return LAUNCH; }
        } else {
          above = false; pushBad = false;
          if (a < dropG) { if (!inFall) { inFall = true; ffStart = now; } }
          else if (inFall) { if ((int32_t)(now - ffStart) >= (int32_t)dropFallMs) { hadFall = true; lastFall = now; } inFall = false; }
        }
        break;
      case FLYING:
        if ((int32_t)(now - tLaunch) >= (int32_t)flightTimeoutMs) return land(now, "timeout");
        // a real flight has free fall (|a| well under 1 g, drag and tether only); a bump on the bench does not
        // a tumbling dart may never read under 0.6 g (spin), so 400 ms after launch with the push over also counts
        if (!quiet) {
          uint32_t dt = now - tPrev; tPrev = now;
          if (a < freeFallG) { ffMs += dt > 20 ? 20 : dt; if (ffMs >= freeFallMs) quiet = true; }
          if ((int32_t)(now - tLaunch) >= (int32_t)pushOverMs && a < launchG) quiet = true;
          if (!quiet) break;
        }
        if ((int32_t)(now - tLaunch) < (int32_t)minFlightMs) break;
        if (a > impactG) { st = IMPACTED; tImpact = now; stillOn = false; break; }
        if (a > stillLo && a < stillHi) { if (!stillOn) { stillOn = true; tStill = now; } if (now - tStill >= softStillMs) return land(tStill, "still"); }
        else stillOn = false;
        break;
      case IMPACTED:
        if ((int32_t)(now - tLaunch) >= (int32_t)flightTimeoutMs) return land(tImpact, "timeout");
        if (a > impactG) { tImpact = now; stillOn = false; break; }
        if (a > stillLo && a < stillHi) { if (!stillOn) { stillOn = true; tStill = now; } if (now - tStill >= stillMs) return land(tImpact, "impact"); }
        else stillOn = false;
        break;
      default: break;
    }
    return NONE;
  }
  int poll(uint32_t now) {                    // called every tick: the timeout must fire even with no IMU samples
    if (flying() && (int32_t)(now - tLaunch) >= (int32_t)flightTimeoutMs) return land(st == IMPACTED ? tImpact : now, "timeout");
    return NONE;
  }
};

// ---------------------------------------------------------------------------------------------------------------
// Flight log: 50 Hz ring of (t, quaternion, acceleration). 12 s = 600 samples x 20 bytes = 12 kB.
struct Sample { uint32_t t; int16_t q[4]; int16_t a[3]; int16_t pad; };   // q in 1/16384, a in 0.01 m/s^2
template <int N>
struct FlightLog {
  Sample buf[N]; int head = 0, count = 0;
  bool frozen = false, freezePending = false; uint32_t freezeAt = 0, winStart = 0; bool haveWin = false;
  uint32_t nextT = 0; bool started = false;
  static int16_t q14(float v) { float x = v * 16384.f; if (x > 32767) x = 32767; if (x < -32767) x = -32767; return (int16_t)lrintf(x); }
  static int16_t a100(float v) { float x = v * 100.f; if (x > 32767) x = 32767; if (x < -32767) x = -32767; return (int16_t)lrintf(x); }
  void clear() { head = count = 0; frozen = freezePending = haveWin = started = false; freezeAt = winStart = nextT = 0; }
  void unfreeze() { frozen = freezePending = haveWin = false; }
  // offer a sample; stored when the 20 ms slot is due (keeps a 50 Hz grid, resyncs after a gap)
  bool offer(uint32_t now, const float q[4], const float a[3]) {
    if (frozen) return false;
    if (started && (int32_t)(now - nextT) < 0) return false;
    if (!started || now - nextT > 40) nextT = now;
    started = true; nextT += 20;
    Sample& s = buf[head];
    s.t = now; for (int i = 0; i < 4; i++) s.q[i] = q14(q[i]); for (int i = 0; i < 3; i++) s.a[i] = a100(a[i]); s.pad = 0;
    head = (head + 1) % N; if (count < N) count++;
    return true;
  }
  void window(uint32_t start) { winStart = start; haveWin = true; }
  void scheduleFreeze(uint32_t at) { freezePending = true; freezeAt = at; }
  void poll(uint32_t now) { if (freezePending && (int32_t)(now - freezeAt) >= 0) { frozen = true; freezePending = false; } }
  const Sample& at(int i) const { return buf[(head - count + i + N) % N]; }   // 0 = oldest
};
static const int LOG_N = 600;
typedef FlightLog<LOG_N> Log;

// Streams /flightlog JSON in chunks (no big String on the heap). Works on a snapshot copy of the log.
struct LogJson {
  const Log* L; int stage = 0, i = 0; const char* phase = ""; bool launched = false, landedF = false; uint32_t tLaunch = 0, tLand = 0;
  const char* how = ""; uint32_t lo = 0; bool first = true;
  void begin(const Log* log, const char* ph, bool hasLaunch, uint32_t tl, bool hasLand, uint32_t td, const char* howLanded) {
    L = log; stage = 0; i = 0; phase = ph; launched = hasLaunch; tLaunch = tl; landedF = hasLand; tLand = td; how = howLanded; first = true;
    lo = (L->haveWin && launched) ? L->winStart : 0;
  }
  bool keep(const Sample& s) const { return !(L->haveWin && launched) || (int32_t)(s.t - lo) >= 0; }
  int countKept() const { int n = 0; for (int k = 0; k < L->count; k++) if (keep(L->at(k))) n++; return n; }
  // fills buf (cap >= 256) and returns the byte count; 0 = finished
  int chunk(char* buf, int cap) {
    int n = 0;
    if (stage == 0) {
      char tl[16] = "null", td[16] = "null";
      if (launched) snprintf(tl, sizeof tl, "%lu", (unsigned long)tLaunch);
      if (landedF) snprintf(td, sizeof td, "%lu", (unsigned long)tLand);
      n = snprintf(buf, cap, "{\"hz\":50,\"frozen\":%s,\"phase\":\"%s\",\"t_launch_ms\":%s,\"t_land_ms\":%s,\"landing\":%s%s%s,"
                   "\"n\":%d,\"fields\":[\"t_ms\",\"qw\",\"qx\",\"qy\",\"qz\",\"ax\",\"ay\",\"az\"],\"samples\":[",
                   L->frozen ? "true" : "false", phase, tl, td, landedF ? "\"" : "", landedF ? how : "null", landedF ? "\"" : "", countKept());
      stage = 1; return n;
    }
    if (stage == 1) {
      while (i < L->count && cap - n > 100) {
        const Sample& s = L->at(i++);
        if (!keep(s)) continue;
        n += snprintf(buf + n, cap - n, "%s[%lu,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f]", first ? "" : ",", (unsigned long)s.t,
                      s.q[0] / 16384.f, s.q[1] / 16384.f, s.q[2] / 16384.f, s.q[3] / 16384.f, s.a[0] / 100.f, s.a[1] / 100.f, s.a[2] / 100.f);
        first = false;
      }
      if (i >= L->count) stage = 2;
      if (n) return n;
    }
    if (stage == 2) { stage = 3; return snprintf(buf, cap, "]}"); }
    return 0;
  }
};

// ---------------------------------------------------------------------------------------------------------------
struct Kedge {
  Io io; Tuning tu;
  int dir[NMOT] = {0, 0, 0, 0};
  uint32_t until[NMOT] = {0, 0, 0, 0};
  int spd = 255;
  int seq = IDLE; uint32_t seqT0 = 0, condT = 0, motT0[NMOT] = {0, 0, 0, 0}, seqStart = 0;
  float lastMa = 0, peakMa = 0, fMa = 0; char lastEvent[64] = "boot";   // fMa: filtered current for stall decisions
  int fault = F_NONE;
  bool armed = false, autoOnLanding = true, landed = false; uint32_t armT = 0;
  bool podLead = false; float kickPos = 0; uint32_t lastTickT = 0; bool tickSeen = false;   // kickPos: kick travel estimate, ms at full duty from stowed
  bool kickKnown = true;                                // kickPos is trusted (power-on with the legs stowed, or after a release)
  bool kickOverPair = false;                            // last pair reading said the kick (as the paused worm) is at stall
  float kickBackStart = 0, kickBackDone = 0;            // release: estimated travel out at the start, real travel counted back
  bool footStalled = false, drillsStalled = false;      // an auger stopped for a stall is not re-driven in the next step
  uint32_t warn = 0;                                    // warnings of the last sequence (bits, see warnNames)
  bool inaOk = true, imuOk = true; int inaFails = 0;
  uint32_t tripT = 0, lowT = 0, manT = 0; bool tripOn = false, lowOn = false, manOn = false;
  int stallLock[NMOT] = {0, 0, 0, 0}; uint32_t lastCmd[NMOT] = {0, 0, 0, 0};   // a held key that stalled stays off until released
  uint32_t heldStart[NMOT] = {0, 0, 0, 0};             // when the current manual hold began
  int lastKeyDir[NMOT] = {0, 0, 0, 0}; bool bothLock[NMOT] = {false, false, false, false};
  uint32_t pressT[NMOT][2] = {}; bool pressSeen[NMOT][2] = {};   // last press of each direction ([0] = -1, [1] = +1), refused or not                 // direction of the last accepted key per motor (hold continuity)
  // pair check state (two motors share the INA219, see pairCheck)
  uint32_t pulseT = 0; bool paused = false, readDone = false; int pz = -1, hitsMain = 0, hitsOther = 0, hitsNoRise = 0, pairKey = -1;
  float bothMa = -1, soloMa = -1, bothMin = -1, railV = -1; bool needRise = false;
  uint32_t lastRunMs[NMOT] = {0, 0, 0, 0};
  Flight flight; Log log;
  float accG = 1.0f, peakG = 0;                        // peakG: highest |a| since the last arm
  uint32_t lowT2 = 0; bool lowOn2 = false;

  bool seqRunning() const { return (seq >= K_SWING && seq <= D_AFTER) || (seq >= R_DRILL && seq <= R_KICK); }
  bool anchoring() const { return seq >= K_SWING && seq <= D_AFTER; }
  bool releasing() const { return seq >= R_DRILL && seq <= R_KICK; }
  int phase() const {
    if (fault) return P_FAULT;
    if (releasing()) return P_RELEASING;
    if (anchoring()) return P_ANCHORING;
    if (flight.flying()) return P_FLIGHT;
    if (seq == DONE) return P_ANCHORED;
    if (landed && (seq == IDLE || seq == ABORTED) && !running()) return P_LANDED;
    if (armed) return P_ARMED;
    return P_IDLE;
  }

  enum { W_FOOT_STALL = 1, W_DRILL_STALL = 2, W_PLANT_LIMIT = 4, W_PODS_LIMIT = 8, W_KICK_ALREADY = 16, W_PODS_ALREADY = 32 };
  static const char* warnName(int b) { static const char* n[] = {"foot_stalled", "drills_stalled", "kick_plant_time_limit", "pods_time_limit", "kick_already_down", "pods_already_at_stop"}; return n[b]; }
  int duty() const { return seqRunning() ? 255 : spd; }   // sequences always run at full duty (stall currents are for 255)
  static float fin(float v) { return v == v && v < 1e9f && v > -1e9f ? v : 0.f; }
  void ev(const char* s) { strncpy(lastEvent, s, sizeof(lastEvent) - 1); lastEvent[sizeof(lastEvent) - 1] = 0; if (io.log) io.log(s); }
  void setFault(int f, const char* msg, uint32_t now) {
    armed = false;                                    // a fault must never be cleared by the automatic landing /auto
    if (!seqRunning()) for (int m = 0; m < NMOT; m++) if (dir[m]) stallLock[m] = dir[m];   // a held key stays off until released
    stopAll(now); if (seqRunning()) seq = ABORTED; fault = f; ev(msg); }

  // threshold for motor m: bench stall current scaled by rail voltage, and by PWM duty for manual runs
  float vScale() const { if (!(railV > 3.f)) return 1.f; float k = railV / tu.refV; return k < 0.6f ? 0.6f : k > 1.15f ? 1.15f : k; }
  float lim(float ma) const { float d = duty() / 255.f; return ma * vScale() * d * d; }   // slow decay: stall supply current ~ duty^2
  // stall threshold for motor m: never below 1.5 x its free current (free supply current goes ~ duty, so at low speed
  // the duty^2 stall line would cut into normal running)
  float stallLim(int m) const { float d = duty() / 255.f, a = lim(tu.stallMa[m]), b = 1.5f * tu.freeMa[m] * vScale() * d; return a > b ? a : b; }

  // Pair check: two motors share the INA219. Every 550 ms the self-locking worm ("main") stops for 150 ms and holds
  // its position while the other motor keeps turning and is read alone at 110 ms; the main motor's own current is
  // (both, read just before the pause) - (other alone). Pausing the worm, not the auger, keeps an auger in soil
  // turning, so a stopped screw never fakes a stall. Returns bit 1 = main stalled, bit 2 = other stalled
  // (2 readings in a row each).
  int pairCheck(int main, int other, float mainLim, float otherLim, uint32_t now) {
    (void)other;                                      // the other motor keeps running; only its reading is used
    uint32_t ph = now - pulseT;
    if (!paused) {
      if (ph >= 380 && ph < 400) bothMa = fMa;
      if (ph >= 400) { paused = true; pz = main; readDone = false; pulseT = now; io.setMotor(main, 0, 0); }
      return 0;
    }
    int r = 0;
    if (!readDone && ph >= 110) {
      readDone = true; soloMa = fMa;
      if (bothMa >= 0) {
        // sequences: the total must also have risen over the phase's lowest reading (guards against the auger's
        // current changing while the worm pauses, which biases both - alone)
        bool rose = !needRise || (bothMin >= 0 && bothMa - bothMin > tu.riseFrac * (mainLim - lim(tu.freeMa[main])));
        hitsMain = (bothMa - soloMa > mainLim && rose) ? hitsMain + 1 : 0;
        if (main == KICK) kickOverPair = bothMa - soloMa > mainLim;
        // a worm already at its stop when the step starts never shows a rise: 3 of the last 4 readings over the line
        // also count (one noisy reading does not reset it; an auger change of up to about 100 mA cannot fake this)
        hitsNoRise = ((hitsNoRise << 1) | (bothMa - soloMa > mainLim ? 1 : 0)) & 0xF;
        if (__builtin_popcount(hitsNoRise) >= 3) hitsMain = 2;
        if (bothMin < 0 || bothMa < bothMin) bothMin = bothMa;
      }
      hitsOther = soloMa > otherLim ? hitsOther + 1 : 0;
      if (hitsMain >= 2) r |= 1;
      if (hitsOther >= 2) r |= 2;
    }
    if (ph >= 150) { endPause(); pulseT = now; }
    return r;
  }
  void endPause() { if (paused && pz >= 0 && dir[pz]) io.setMotor(pz, dir[pz], duty()); paused = false; pz = -1; bothMa = -1; }
  void pairReset(uint32_t now) { endPause(); pulseT = now; readDone = false; hitsMain = hitsOther = hitsNoRise = 0; soloMa = -1; bothMin = -1; condT = 0; kickOverPair = false; }
  // stall of main (and of other while it runs): pair check while both run, plain threshold once other has stopped
  int stallPair(int main, int other, uint32_t now) {
    if (dir[other] && dir[main]) { needRise = true; return pairCheck(main, other, stallLim(main), stallLim(other), now); }
    if (paused) endPause();
    return (now - motT0[main] > tu.surgeIgnoreMs && held(fMa > stallLim(main), tu.stallHoldMs, now)) ? 1 : 0;
  }
  bool soloStalled(int m, uint32_t now) { return dir[m] && now - motT0[m] > tu.surgeIgnoreMs && held(fMa > stallLim(m), tu.stallHoldMs, now); }

  void drive(int m, int d, uint32_t ms, uint32_t now) {
    if (d && dir[m] != d) motT0[m] = now;     // a repeat of the same command keeps its start time (stall timer)
    dir[m] = d; until[m] = d ? now + ms : 0;
    if (!d && m == pz) { paused = false; pz = -1; }
    if (d && paused && m == pz) return;       // held off by the pair check for a moment; it restarts the motor itself
    io.setMotor(m, d, d ? duty() : 0);
  }
  void stopAll(uint32_t now) { for (int m = 0; m < NMOT; m++) drive(m, 0, 0, now); paused = false; pz = -1; pairKey = -1; }
  bool running() const { for (int m = 0; m < NMOT; m++) if (dir[m]) return true; return false; }

  // boot: called once by the sketch with the reset cause (F_BROWNOUT / F_WATCHDOG / F_NONE)
  // kickStowed: true after a power-on (legs stowed, as README asks); false after a brownout, crash, OTA or software
  // reset, when the legs may be out (kick jam checks then wait for the next completed release)
  void boot(int resetFault, uint32_t now, bool kickStowed = true) {
    stopAll(now); kickKnown = kickStowed; kickPos = 0;
    if (resetFault) { fault = resetFault; char b[64]; snprintf(b, sizeof b, "boot after %s reset, motors off", faultName(resetFault)); ev(b); }
    else ev("boot, motors off");
  }

  // returns nullptr when accepted, else the reason
  // disarm=false (GET /stop?disarm=0) stops the motors but keeps the landing detector armed and flight tracking going
  const char* stop(uint32_t now, bool disarm = true) {
    bool wasSeq = seqRunning();
    // explicit STOP: a key that any client is still repeating (running, in a deadman gap, or refused while a sequence
    // ran) cannot restart its motor until it has been released
    if (disarm) for (int m = 0; m < NMOT; m++) {
      uint32_t w = tu.wifiDeadmanMs + 250;
      bool neg = pressSeen[m][0] && (int32_t)(now - pressT[m][0]) < (int32_t)w;
      bool pos = pressSeen[m][1] && (int32_t)(now - pressT[m][1]) < (int32_t)w;
      if (dir[m] && !wasSeq) { if (dir[m] > 0) pos = true; else neg = true; }
      if (pos && neg) { bothLock[m] = true; lastCmd[m] = now; }               // both keys of this motor still held: lock both
      else if (pos || neg) { int d = pos ? 1 : -1; stallLock[m] = d; lastCmd[m] = now; lastKeyDir[m] = d; }
    }
    stopAll(now);
    if (wasSeq) seq = ABORTED;
    if (disarm) {
      if (flight.flying()) { flight.land(now, "stop"); landed = true; uint32_t fr = now + 3000, cap = freezeCap(); log.scheduleFreeze((int32_t)(fr - cap) > 0 ? cap : fr); }
      armed = false;
    }
    fault = F_NONE;
    ev(wasSeq ? "stop: sequence aborted" : "stop");
    return nullptr;
  }
  const char* startAuto(uint32_t now) {
    if (flight.flying()) return "in flight";
    if (anchoring()) return nullptr;                  // already running: a repeated request is not an error
    if (seqRunning()) return "busy";
    if (seq == DONE) return "already anchored, release first";
    if (!inaOk) return "no current sensor";          // refuse before changing anything
    if (armed) { armed = false; ev("auto: disarmed first (the launch was not seen)"); }
    stopAll(now); pairReset(now); fault = F_NONE; armed = false; warn = 0; footStalled = drillsStalled = false;
    seq = K_SWING; seqT0 = seqStart = now; peakMa = 0; drive(KICK, +1, tu.kickMaxMs + 500, now); ev("auto: kick swing");
    return nullptr;
  }
  const char* startRelease(uint32_t now) {
    if (flight.flying()) return "in flight";
    if (releasing()) return nullptr;                  // already running
    if (seqRunning()) return "busy";
    if (!inaOk) return "no current sensor";
    stopAll(now); pairReset(now); fault = F_NONE; armed = false; warn = 0;
    seq = R_DRILL; seqT0 = seqStart = now; peakMa = 0; drive(DRILLS, -1, tu.drillLeadMs, now); ev("release: drills out");
    return nullptr;
  }
  const char* arm(bool on, bool withAuto, uint32_t now) {
    if (!on) { armed = false; ev("disarmed"); return nullptr; }
    if (seqRunning()) return "busy";
    if (flight.flying()) return "in flight";
    if (!imuOk) return "no IMU";
    if (withAuto && !inaOk) return "no current sensor";
    if (withAuto && !running() && railV >= 0.f && railV < tu.armMinV) return "battery low";   // includes 0 V (plug out); log-only arm still allowed
    // the moment of arming counts as still, so a shot right after the arm is still seen
    flight.reset(); flight.armedAt(now); armT = now; log.unfreeze(); landed = false; peakG = 0; armed = true; autoOnLanding = withAuto; fault = F_NONE;
    if (seq == ABORTED || seq == RELEASED || seq == DONE) seq = IDLE;   // a new shot
    ev(withAuto ? "armed: auto-anchor on landing" : "armed: log only");
    return nullptr;
  }

  // manual key: 1/! deploy, 2/@ kick, 3/# drills, 4/$ foot, space/s stop, +/- speed, a auto, r release, A arm toggle
  const char* key(char c, uint32_t runMs, uint32_t now) {
    if (c == ' ' || c == 's' || c == 'S') return stop(now);
    if (c == 'a') return startAuto(now);
    if (c == 'r') return startRelease(now);
    if (c == 'A') return armed ? arm(false, true, now) : arm(true, true, now);
    if (c == '+') { spd = spd + 25 > 255 ? 255 : spd + 25; return nullptr; }
    if (c == '-') { spd = spd - 25 < 50 ? 50 : spd - 25; return nullptr; }
    int m = -1, d = 0;
    switch (c) {
      case '1': m = DEPLOY; d = +1; break; case '!': m = DEPLOY; d = -1; break;
      case '2': m = KICK; d = +1; break;   case '@': m = KICK; d = -1; break;
      case '3': m = DRILLS; d = +1; break; case '#': m = DRILLS; d = -1; break;
      case '4': m = FOOT; d = +1; break;   case '$': m = FOOT; d = -1; break;
      default: return "unknown key";
    }
    uint32_t prevPressT = pressT[m][d > 0]; bool prevPressSeen = pressSeen[m][d > 0];   // for the per-direction lock
    pressT[m][d > 0] = now; pressSeen[m][d > 0] = true;
    if (seqRunning() || flight.flying() || fault) lastCmd[m] = now;   // a refused press still counts as "held" for the locks
    if (seqRunning()) return "busy";                  // a sequence owns the motors; stop first
    if (flight.flying()) return "in flight";
    if (fault) return "fault latched: send stop first";
    if (runMs > tu.manualMaxMs) runMs = tu.manualMaxMs;
    // same hold if this press comes within max(last run time, WiFi deadman) + 250 ms (so at least 600 ms)
    uint32_t win = (lastRunMs[m] > tu.wifiDeadmanMs ? lastRunMs[m] : tu.wifiDeadmanMs) + 250;
    if (win < tu.wifiDeadmanMs + 150) win = tu.wifiDeadmanMs + 150;
    bool repeat = now - lastCmd[m] < win; lastCmd[m] = now; lastRunMs[m] = runMs;
    // the lock holds while THIS direction's key keeps coming (presses of the other direction do not count)
    bool dirRepeat = prevPressSeen && (int32_t)(now - prevPressT) < (int32_t)win;
    if (stallLock[m] == d && dirRepeat) return "stalled: release the key first";
    if (stallLock[m] == d) stallLock[m] = 0;          // only a fresh press of the locked direction clears its lock
    // a repeat within the deadman window (even after a short gap that let the motor stop) continues the same hold:
    // the hold cap and the stall start-up time are not restarted by slow repeats
    // both directions of one motor held (1 and ! together): stop it and refuse both until neither has been pressed for a window
    if (bothLock[m]) { if (repeat) return "both directions held"; bothLock[m] = false; }
    if (repeat && lastKeyDir[m] == -d && dir[m] == -d) { bothLock[m] = true; drive(m, 0, 0, now); ev("both directions held: motor stopped"); return "both directions held"; }
    bool cont = repeat && lastKeyDir[m] == d; lastKeyDir[m] = d;
    if (!cont) heldStart[m] = now;
    uint32_t cap = inaOk ? tu.manualMaxMs : tu.manualMaxNoInaMs, used = now - heldStart[m];
    if (used >= cap) { stallLock[m] = d; return "stalled: release the key first"; }
    if (runMs > cap - used) runMs = cap - used;       // the whole hold, not each repeat, is capped
    if (seq == DONE || seq == RELEASED || seq == ABORTED) seq = IDLE;   // legs moved by hand: no longer a known state
    uint32_t keepT0 = motT0[m];
    drive(m, d, runMs, now);
    if (cont) motT0[m] = keepT0;
    return nullptr;
  }
  // WiFi station left: manual (hold-to-run) motors stop at once; a running sequence carries on (it is autonomous)
  void linkLost(uint32_t now) { if (!seqRunning() && running()) { stopAll(now); ev("WiFi client lost, motors stopped"); } }

  // one BNO085 accelerometer reading (|a| in g)
  void onAccel(float aG, uint32_t now) {
    if (!(aG == aG) || aG > 1e6f) return;            // NaN / inf from a bad read
    accG = aG; if (aG > peakG) peakG = aG;
    if (!armed && !flight.flying()) return;           // launch is only looked for while armed
    handleFlight(flight.update(aG, now), now);
  }
  // the last moment the whole window from 2 s before launch still fits in the 12 s ring
  uint32_t freezeCap() const { return flight.tLaunch - 2000 + (LOG_N - 1) * 20; }
  void handleFlight(int e, uint32_t now) {
    if (e == Flight::LAUNCH) {
      log.window(flight.tLaunch - 2000);             // modular: fine across the millis() wrap
      log.scheduleFreeze(freezeCap());               // never let the ring overwrite the start of the window
      if (running()) { stopAll(now); ev("launch: motors stopped"); }
      char b[64]; snprintf(b, sizeof b, "launch detected at %lu ms", (unsigned long)flight.tLaunch); ev(b);
    } else if (e == Flight::LAND) {
      landed = true;
      uint32_t fr = flight.tLand + 3000, cap = freezeCap();
      if ((int32_t)(fr - cap) > 0) fr = cap;         // a long (timeout) flight: keep launch and impact rather than the end
      if (!log.frozen) log.scheduleFreeze((int32_t)(fr - now) > 0 ? fr : now);
      char b[64]; snprintf(b, sizeof b, "landed (%s)", flight.how); ev(b);
      bool go = armed && autoOnLanding; armed = false;
      if (go && flight.fallback()) { go = false; ev("no impact seen: auto not started, send /auto"); }
      if (go) { const char* r = startAuto(now); if (r) { char m[64]; snprintf(m, sizeof m, "auto on landing refused: %s", r); ev(m); } }
    }
  }
  // 50 Hz flight log feed (the sketch calls it with the newest quaternion and acceleration)
  void imuSample(uint32_t now, const float q[4], const float a[3]) { log.offer(now, q, a); }

  bool held(bool cond, uint32_t hold, uint32_t now) { if (!cond) { condT = 0; return false; } if (!condT) condT = now; return now - condT >= hold; }
  static bool heldT(bool cond, bool& on, uint32_t& t, uint32_t hold, uint32_t now) {
    if (!cond) { on = false; return false; } if (!on) { on = true; t = now; } return now - t >= hold;
  }

  void tick(uint32_t now) {
    log.poll(now);
    if (armed || flight.flying()) handleFlight(flight.poll(now), now);

    float ma = io.current_mA ? io.current_mA() : 0;
    if (!(ma == ma)) {                                // NaN: failed read
      if (inaOk && ++inaFails >= tu.inaFailLimit) {
        inaOk = false;
        if (seqRunning()) setFault(F_CURRENT_SENSOR, "INA219 lost: sequence stopped", now);
        else ev("INA219 lost: manual keys only, timers still apply");
      }
    } else { inaFails = 0; if (!inaOk && io.current_mA) { inaOk = true; ev("INA219 back"); } }
    bool curOk = ma == ma;
    if (curOk) { lastMa = ma; if (ma > peakMa) peakMa = ma; fMa += 0.3f * (ma - fMa); if (!running()) fMa = ma; }
    float v = io.rail_V ? io.rail_V() : -1;
    railV = v;
    {                                                 // kick travel bookkeeping (only while the kick is really driven)
      uint32_t dt = tickSeen ? now - lastTickT : 0; lastTickT = now; tickSeen = true; if (dt > 50) dt = 50;
      // counts only real travel: not while paused, not while planting (the kick is on the ground), and outward not
      // while the kick alone draws stall current
      // stalled in either direction: alone, current over its stall line; in a pair, the last pair reading
      bool alone = dir[KICK] && !dir[DEPLOY] && !dir[DRILLS] && !dir[FOOT];
      bool stalled = alone ? fMa > stallLim(KICK) : kickOverPair;
      if (dir[KICK] && !(paused && pz == KICK) && seq != K_PLANT && !stalled) {
        float step = (float)dt * duty() / 255.f;
        kickPos += dir[KICK] * step;
        if (seq == R_KICK && dir[KICK] < 0) kickBackDone += step;
        if (kickPos < 0) kickPos = 0;
        if (kickPos > tu.kickSwingMs) kickPos = tu.kickSwingMs;
      }
    }
    if (armed && !flight.flying() && (int32_t)(now - armT) > (int32_t)tu.armMaxMs) { armed = false; ev("arm expired (15 min without a launch)"); }

    bool run = running();
    if (!curOk) {                                     // no reading this tick: timers only, current checks wait for the next
      if (paused && now - pulseT >= 150) { endPause(); pulseT = now; }   // never hold a worm paused longer than 150 ms
      // a step timer that ends now is a normal step end: the motor stops and the sequence moves on at the next good
      // read; only the kick's limits are faults (as with a good reading)
      for (int m = 0; m < NMOT; m++) if (dir[m] && (int32_t)(now - until[m]) >= 0) {
        if (m == KICK && (seq == K_SWING || seq == K_PLANT || seq == R_KICK)) { setFault(F_KICK_TIMEOUT, "kick time limit (no current reading)", now); return; }
        drive(m, 0, 0, now);
      }
      if (seqRunning() && now - seqStart > (anchoring() ? tu.autoMaxMs : tu.releaseMaxMs)) setFault(F_SEQ_TIMEOUT, "sequence time limit", now);
      return;
    }
    if (heldT(run && ma > tu.tripMa, tripOn, tripT, tu.tripHoldMs, now)) { setFault(F_OVERCURRENT, "TRIP: over current, all stopped", now); return; }
    if (heldT(run && v >= 0.f && v < tu.lowRailV, lowOn, lowT, tu.lowRailMs, now)) { setFault(F_LOW_RAIL, "LOW RAIL: battery flat, all stopped", now); return; }

    for (int m = 0; m < NMOT; m++) if (dir[m] && (int32_t)(now - until[m]) >= 0) {
      if (m == KICK && (seq == K_SWING || seq == K_PLANT)) { setFault(F_KICK_TIMEOUT, "auto: kick timed out", now); return; }
      if (m == KICK && seq == R_KICK) { setFault(F_KICK_TIMEOUT, "release: kick timed out", now); return; }
      if (!seqRunning() && now - heldStart[m] >= (inaOk ? tu.manualMaxMs : tu.manualMaxNoInaMs)) stallLock[m] = dir[m];   // hold cap reached: off until released
      drive(m, 0, 0, now);
    }

    if (seqRunning()) {
      uint32_t lim = anchoring() ? tu.autoMaxMs : tu.releaseMaxMs;
      if ((int32_t)(now - seqStart) > (int32_t)lim) { setFault(F_SEQ_TIMEOUT, "sequence time limit", now); return; }
    } else if (running()) {                           // manual: stop on stall
      int nrun = 0, ms[NMOT]; bool settled = true;
      for (int m = 0; m < NMOT; m++) if (dir[m]) { ms[nrun++] = m; if (now - motT0[m] < tu.surgeIgnoreMs) settled = false; }
      int stopMask = 0;
      if (nrun == 1) {
        if (heldT(settled && fMa > stallLim(ms[0]), manOn, manT, tu.stallHoldMs, now)) stopMask = 1 << ms[0];
      } else if (nrun == 2) {                         // pair check: pause the worm (drills are the only non-worm)
        int a = ms[0], b = ms[1], main = (a == DRILLS) ? b : a, other = (main == a) ? b : a;
        if (pairKey != main * 8 + other) { pairKey = main * 8 + other; pairReset(now); }
        needRise = false;                             // manual: often pressed straight into an end stop
        int r = pairCheck(main, other, stallLim(main), stallLim(other), now);
        if (r & 1) stopMask |= 1 << main;
        if (r & 2) stopMask |= 1 << other;
      } else {                                        // 3 or 4 at once: one stalled while the others work normally
        float sumLoad = 0, l = 1e9f;
        for (int k = 0; k < nrun; k++) sumLoad += tu.loadMa[ms[k]];
        for (int k = 0; k < nrun; k++) { float x = tu.stallMa[ms[k]] + sumLoad - tu.loadMa[ms[k]]; if (x < l) l = x; }
        if (heldT(settled && fMa > lim(l), manOn, manT, tu.stallHoldMs, now)) for (int k = 0; k < nrun; k++) stopMask |= 1 << ms[k];
      }
      if (nrun != 2 && pairKey >= 0) { pairKey = -1; endPause(); }
      if (stopMask) {
        char b[64]; int n = snprintf(b, sizeof b, "stalled:");
        for (int m = 0; m < NMOT; m++) if (stopMask & (1 << m)) { n += snprintf(b + n, sizeof b - n, " %s", motorName(m)); stallLock[m] = dir[m]; drive(m, 0, 0, now); }
        snprintf(b + n, sizeof b - n, " at %.0f mA, stopped", ma); ev(b); manOn = false;
      }
    } else { manOn = false; if (pairKey >= 0) { pairKey = -1; endPause(); } }

    // a single motor drawing well under its free current: open wire, or a short that the driver's own over-current
    // protection keeps chopping (the INA219 average then reads almost nothing)
    {
      int nrun = 0, only = -1; for (int m = 0; m < NMOT; m++) if (dir[m]) { nrun++; only = m; }
      bool low = nrun == 1 && !paused && now - motT0[only] > tu.surgeIgnoreMs && fMa < tu.noCurrentFrac * lim(tu.freeMa[only]);
      if (heldT(low, lowOn2, lowT2, tu.noCurrentMs, now)) {
        char b[64]; snprintf(b, sizeof b, "%s draws no current: wire open or shorted", motorName(only));
        if (seqRunning()) { setFault(F_NO_CURRENT, b, now); return; }
        stallLock[only] = dir[only]; drive(only, 0, 0, now); ev(b);
      }
    }

    uint32_t t = now - seqT0;
    switch (seq) {
      case K_SWING:
        if (t > tu.surgeIgnoreMs && held(fMa > lim(tu.kickContactMa), tu.contactHoldMs, now) && fMa > stallLim(KICK)) {
          // already at stall level at the first contact: the kick was down already (a retried /auto); plant the foot
          // alone. Only believed if the kick has travelled out; near stowed it is a jam (or wired backwards).
          if (kickKnown && kickPos < 0.25f * tu.kickSwingMs) { setFault(F_KICK_JAM, "auto: kick stalled near stowed (jammed or backwards)", now); return; }
          drive(KICK, 0, 0, now); drive(FOOT, +1, tu.footAfterMs * 2, now); seq = F_AFTER; seqT0 = now; condT = 0; pairReset(now);
          warn |= W_KICK_ALREADY; ev("auto: kick already down, foot planting");
        } else if (t > tu.surgeIgnoreMs && condT && now - condT >= tu.contactHoldMs) {
          drive(KICK, +1, tu.kickPlantMaxMs + 500, now); drive(FOOT, +1, tu.kickPlantMaxMs + 500, now); seq = K_PLANT; seqT0 = now; pairReset(now); ev("auto: foot touches, foot auger on");   // kickPos stays at the contact point
        }
        break;
      case K_PLANT: {   // kick keeps pushing while the foot screws in
        int r = stallPair(KICK, FOOT, now);
        if (r & 2) { drive(FOOT, 0, 0, now); warn |= W_FOOT_STALL; footStalled = true; ev("auto: foot stalled (stone?), kick carries on"); }
        bool lim2 = t >= tu.kickPlantMaxMs;
        if (lim2) warn |= W_PLANT_LIMIT;           // housing stall never seen: the kick is on the ground anyway
        if ((r & 1) || lim2) { drive(KICK, 0, 0, now); if (!footStalled) drive(FOOT, +1, tu.footAfterMs, now); seq = F_AFTER; seqT0 = now; pairReset(now);
          ev(lim2 ? "auto: kick plant time limit, foot finishing" : "auto: kick down, foot finishing"); }
        break;
      }
      case F_AFTER:
        if (soloStalled(FOOT, now)) { drive(FOOT, 0, 0, now); warn |= W_FOOT_STALL; ev("auto: foot stalled (full depth or stone)"); }
        if (!dir[FOOT]) { drive(DEPLOY, +1, tu.deployMaxMs, now); podLead = true; seq = D_DEPLOY; seqT0 = now; pairReset(now); ev("auto: deploy with drills spinning"); }
        break;
      case D_DEPLOY: {  // pods swing down while the augers cut
        if (podLead) {                                // first 300 ms: deploy worm alone (open wire? already down?)
          if (t < tu.podLeadMs) break;
          podLead = false;
          if (fMa < tu.noCurrentFrac * lim(tu.freeMa[DEPLOY])) { setFault(F_NO_CURRENT, "deploy draws no current: wire open or shorted", now); return; }
          // no "already down" shortcut here: augers touching soil without spinning could fake it; the pair check decides
          drive(DRILLS, +1, tu.deployMaxMs + tu.drillAfterMs, now); pairReset(now);
          break;
        }
        int r = stallPair(DEPLOY, DRILLS, now);
        if (t < tu.podsMinMs) r &= ~1;
        if (r & 2) { drive(DRILLS, 0, 0, now); warn |= W_DRILL_STALL; drillsStalled = true; ev("auto: drills stalled (stone?), pods carry on"); }
        if ((r & 1) || !dir[DEPLOY]) {
          bool st = r & 1; if (!st) warn |= W_PODS_LIMIT;
          drive(DEPLOY, 0, 0, now); if (!drillsStalled) drive(DRILLS, +1, tu.drillAfterMs, now); else drive(DRILLS, 0, 0, now); seq = D_AFTER; seqT0 = now; pairReset(now);
          ev(st ? "auto: pods down, drills finishing" : "auto: deploy time limit (down or stalled), drills finishing");
        }
        break;
      }
      case D_AFTER:
        if (soloStalled(DRILLS, now)) { drive(DRILLS, 0, 0, now); warn |= W_DRILL_STALL; ev("auto: drills stalled (full depth or stone)"); }
        if (!dir[DRILLS]) { seq = DONE; ev("auto: anchored, holding (worms self-lock)"); }
        break;
      // release = the anchor sequence backwards: drills unscrew, pods lift with drills still unscrewing, foot unscrews,
      // kick swings back with the foot still unscrewing
      case R_DRILL:
        if (soloStalled(DRILLS, now)) { setFault(F_DRILL_JAM, "release: drills jammed", now); return; }
        // the augers may still be threaded in soil: the drills keep unscrewing while the pods lift (no deploy-alone lead)
        if (!dir[DRILLS]) { drive(DRILLS, -1, tu.deployMaxMs, now); drive(DEPLOY, -1, tu.deployMaxMs, now); seq = R_PODS; seqT0 = now; pairReset(now); ev("release: pods up, drills still out"); }
        break;
      case R_PODS: {
        int r = stallPair(DEPLOY, DRILLS, now);
        if (t < tu.podsMinMs) r &= ~1;
        if (r & 2) { setFault(F_DRILL_JAM, "release: drills jammed", now); return; }
        if ((r & 1) || !dir[DEPLOY]) {
          bool st = r & 1; if (!st) warn |= W_PODS_LIMIT;
          drive(DEPLOY, 0, 0, now); drive(DRILLS, 0, 0, now); drive(FOOT, -1, tu.footLeadMs, now); seq = R_FOOT; seqT0 = now; pairReset(now);
          ev(st ? "release: pods up, foot out" : "release: pods time limit (stowed or stalled), foot out");
        }
        break;
      }
      case R_FOOT:
        if (soloStalled(FOOT, now)) { setFault(F_FOOT_JAM, "release: foot jammed", now); return; }
        if (!dir[FOOT]) { drive(FOOT, -1, tu.footOutMs, now); drive(KICK, -1, tu.kickMaxMs, now); seq = R_KICK; seqT0 = now; pairReset(now);
          kickBackStart = kickPos; kickBackDone = 0; ev("release: kick back, foot still out"); }
        break;
      case R_KICK: {
        int r = stallPair(KICK, FOOT, now);
        if (t < tu.kickBackMinMs) r &= ~1;
        if (r & 2) { setFault(F_FOOT_JAM, "release: foot jammed", now); return; }
        if (r & 1) {
          // the real travel back must cover most of the travel out (the estimate out is short: planting is not counted)
          if (kickKnown && kickBackDone < 0.6f * kickBackStart) { setFault(F_KICK_JAM, "release: kick stalled before it was back (jammed)", now); return; }
          stopAll(now); seq = RELEASED; kickPos = 0; kickKnown = true; ev("release: done, legs stowed");
        }
        break;
      }
      default: break;
    }
  }

  int status(char* buf, int n, float roll, float pitch, uint32_t now, float railV = -1, const char* reset = "") const {
    char tl[16] = "null", td[16] = "null";
    if (flight.launched) snprintf(tl, sizeof tl, "%lu", (unsigned long)flight.tLaunch);
    if (flight.st == Flight::LANDED) snprintf(td, sizeof td, "%lu", (unsigned long)flight.tLand);
    char wl[160] = ""; int k = 0;
    for (int b = 0; b < 6; b++) if (warn & (1 << b)) k += snprintf(wl + k, sizeof wl - k, "%s\"%s\"", k ? "," : "", warnName(b));
    return snprintf(buf, n, "{\"fw\":\"%s\",\"t_ms\":%lu,\"phase\":\"%s\",\"fault\":%s%s%s,\"rail_V\":%.2f,\"current_mA\":%.0f,\"peak_mA\":%.0f,\"speed\":%d,"
                    "\"motors\":{\"deploy\":%d,\"kick\":%d,\"drills\":%d,\"foot\":%d},\"seq\":\"%s\",\"armed\":%s,\"auto_on_landing\":%s,\"landed\":%s,"
                    "\"roll\":%.1f,\"pitch\":%.1f,\"accel_g\":%.2f,\"peak_g\":%.2f,\"ina_ok\":%s,\"imu_ok\":%s,\"t_launch_ms\":%s,\"t_land_ms\":%s,\"log_frozen\":%s,"
                    "\"reset\":\"%s\",\"warnings\":[%s],\"event\":\"%s\"}",
                    FW_VERSION, (unsigned long)now, phaseName(phase()), fault ? "\"" : "", fault ? faultName(fault) : "null", fault ? "\"" : "",
                    fin(railV >= 0 ? railV : (io.rail_V ? io.rail_V() : -1.f)), fin(lastMa), fin(peakMa), spd, dir[0], dir[1], dir[2], dir[3], seqName(seq),
                    armed ? "true" : "false", autoOnLanding ? "true" : "false", landed ? "true" : "false", fin(roll), fin(pitch), fin(accG), fin(peakG),
                    inaOk ? "true" : "false", imuOk ? "true" : "false", tl, td, log.frozen ? "true" : "false", reset, wl, lastEvent);
  }
};

}  // namespace kedge
