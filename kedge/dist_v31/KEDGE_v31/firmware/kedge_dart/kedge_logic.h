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

static const char* const FW_VERSION = "kedge-dart-v4.1";

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
// D10 (GPIO9): optional video power switch (WIRING.md "Optional: video power switch"). HIGH = camera and VTX on. With
// nothing fitted the pin drives nothing and the video simply stays on. GPIO9 is not a strapping pin and is quiet at boot.
static constexpr int PIN_VIDEO_GPIO = 9;
static const int N_DRIVERS = 2;                        // DRV#3 is no longer on the dart
static const float DRV_CHANNEL_MAX_MA = 1200;          // DRV8833 continuous per channel (2 A peak, internal OCP)

// ---------------------------------------------------------------------------------------------------------------
struct Io {
  void (*setMotor)(int motor, int dir, int duty);   // dir -1/0/+1, duty 0..255
  float (*current_mA)();                            // total motor-rail current (INA219); NAN when the read failed
  float (*rail_V)();                                // motor rail volts; <= 0 when unknown
  void (*log)(const char* line);
  void (*setVideo)(bool on);                        // optional video power switch (may be nullptr)
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
  uint32_t kickMaxMs = 15000, footAfterMs = 1500, deployMaxMs = 150000, drillAfterMs = 3000;   // deploy: milling, loam about 90 s (v4.1, dart-physics (9))
  uint32_t drillLeadMs = 3000, footLeadMs = 1500, footOutMs = 3000;   // release: mirror of the anchor timings
  uint32_t autoMaxMs = 180000, releaseMaxMs = 180000;   // whole-sequence limits (v4.1: room for the 150 s milling deploy)
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
  uint32_t podsWithDrillsMs = 600;                      // release: the drills turn with the lifting pods this long, then the pods go on alone
  uint32_t podsTravelMs = 1400;                         // pods stowed to down stop at full duty and refV (BENCH_TEST step 3)
  float noGroundFrac = 0.90f;                           // NO_GROUND: kick at this share of kickSwingMs (its swing in air) with no contact
  uint32_t kickSwingMs = 4300;                          // full kick swing at full duty and refV (BENCH_TEST step 5): travel bookkeeping
  uint32_t manualMaxMs = 8000;                          // a manual motor (held key or repeats) never runs longer
  uint32_t manualMaxNoInaMs = 2000;                     // ...and without current sensing (no stall detection)
  int inaFailLimit = 3;                                 // consecutive failed current reads = sensor lost
  // battery (2S 18350) at rest: the INA219 bus voltage with no motor running for battRestMs. Low = 3.4 V per cell (about
  // 10 % left: a warning), critical = 3.2 V per cell (a few percent: /auto refused so an anchor is never left without
  // the charge to release it, and the video is switched off to stretch what is left; /release and keys still work)
  float battLowV = 6.8f, battCritV = 6.4f, battHystV = 0.1f; uint32_t battRestMs = 1000;
  // orientation while still, from the BNO085 accelerometer. imuUpAxis = the chip axis that points UP when the dart lies
  // belly down: 1 = x, 2 = y, 3 = z, negative = the minus direction (BENCH_TEST step 7 checks it; default: the BNO085
  // board lies flat on its deck, parts up, so chip +z is up). Tilt = angle between that axis and straight up.
  int imuUpAxis = 3; float uprightMaxDeg = 70, invertedMinDeg = 120;
  // full orientation gate (dart-physics 7 Nov) once the chip axis pointing to the NOSE is known too (imuNoseAxis, same
  // coding; 0 = not known: the tilt-only rule above). |roll| under levelRollDeg: anchor as is; up to sideRollDeg: the
  // pods first swing about 20 deg with the drills off (levelMs at full duty), which levels the hull on hard ground, then
  // the anchor; beyond: on its side or upside down, not anchored. Nose pitch must be inside pitchMinDeg..pitchMaxDeg.
  // v31 CAD (dart-cad 7 Nov): the BNO085 lies flat in the sled turned 90 deg, board x across the dart, so the nose is chip
  // +y or -y: set imuNoseAxis = 2 or -2 from BENCH_TEST step 7 (left at 0 until then: tilt-only gate).
  int imuNoseAxis = 0; float levelRollDeg = 10, sideRollDeg = 60, pitchMinDeg = -25, pitchMaxDeg = 15;
  uint32_t levelMs = 650, levelSettleMs = 600;   // 650 ms = about 20 deg of the 43 (dart-physics correction to (6))
  // anchoring in soil (dart-physics 7 Nov): the augers sweep sideways through soil, so the deploy worm stalls in soil
  // long before its end stop. A deploy stall before podsDownFrac of podsTravelMs is soil: the deploy pauses millOffMs while
  // the drills keep cutting, then pushes again (milling). Travel is counted at full speed (a load factor from the pair
  // readings was tried and dropped, see tick). No travel gained for millNoProgressMs ends the step: taken as the stop
  // when the estimate is past podsStuckDownFrac, else warning pods_time_limit (firm soil).
  float podsDownFrac = 0.85f, podsStuckDownFrac = 0.6f; uint32_t millOffMs = 200, millNoProgressMs = 5000;
  uint32_t podsPlainMaxMs = 6000;                       // pods step limit while no milling has started (the v4 limit); deployMaxMs once milling
  float drillRelaxFrac = 0.85f; uint32_t drillRelaxMs = 500;   // drill pair near its stall: pause the deploy so the cut relaxes
  // release: drills that stall unscrewing are rocked (in for rockInMs, then out again) up to rockMax times before
  // DRILL_JAM; pods that stall on the way up before 60 % back are milled the same way, PODS_JAM only after
  // millNoProgressMs without progress
  uint32_t rockInMs = 300; int rockMax = 5;
  uint32_t kickPulseOnMs = 400, kickPulseOffMs = 200;   // kick pulses at stall while the foot finishes (dart-physics (8))
  uint32_t hotMillMs = 60000, hotRestMs = 60000;        // a pods step over 60 s heats the motors: /auto waits 60 s (dart-physics (9))
  // anchor motion while ANCHORED (the tow): tilt change over moveDeg, or |a| off 1 g by moveG, for moveHoldMs
  float moveDeg = 2, moveG = 0.15f; uint32_t moveHoldMs = 200;
};

enum Fault { F_NONE = 0, F_OVERCURRENT, F_KICK_TIMEOUT, F_SEQ_TIMEOUT, F_DRILL_JAM, F_FOOT_JAM, F_CURRENT_SENSOR, F_LOW_RAIL,
             F_BROWNOUT, F_WATCHDOG, F_NO_CURRENT, F_KICK_JAM, F_NO_GROUND, F_PODS_JAM, F_NOT_UPRIGHT, F_NFAULT };
static inline const char* faultName(int f) {
  static const char* n[] = {"", "OVERCURRENT", "KICK_TIMEOUT", "SEQ_TIMEOUT", "DRILL_JAM", "FOOT_JAM", "CURRENT_SENSOR",
                            "LOW_RAIL", "BROWNOUT", "WATCHDOG", "NO_CURRENT", "KICK_JAM", "NO_GROUND", "PODS_JAM", "NOT_UPRIGHT"};
  return (f > 0 && f < F_NFAULT) ? n[f] : "";
}

enum Seq { IDLE, K_SWING, K_PLANT, F_AFTER, D_DEPLOY, D_AFTER, DONE, ABORTED, R_DRILL, R_PODS, R_FOOT, R_KICK, RELEASED, K_LEVEL, NSEQ };
static inline const char* seqName(int s) {
  static const char* n[] = {"idle", "kick swing", "kick + foot planting", "foot finishing", "deploy + drills", "drills finishing",
                            "anchored", "aborted", "release: drills out", "release: drills out + pods up", "release: foot out",
                            "release: kick back + foot out", "released", "levelling: pods to 20 deg, drills off"};
  return (s >= 0 && s < NSEQ) ? n[s] : "?";
}
enum Phase { P_IDLE, P_ARMED, P_FLIGHT, P_LANDED, P_ANCHORING, P_ANCHORED, P_RELEASING, P_FAULT };
enum Batt { B_UNKNOWN, B_NONE, B_OK, B_LOW, B_CRIT };
static inline const char* battName(int b) { static const char* n[] = {"unknown", "none", "ok", "low", "critical"}; return (b >= 0 && b <= B_CRIT) ? n[b] : "?"; }
enum Orient { O_UNKNOWN, O_UPRIGHT, O_SIDE, O_INVERTED, O_LEANING, O_NOSE_DOWN, O_NOSE_UP, O_NORIENT };
static inline const char* orientName(int o) { static const char* n[] = {"unknown", "upright", "side", "inverted", "leaning", "nose_down", "nose_up"}; return (o >= 0 && o < O_NORIENT) ? n[o] : "?"; }
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
//            v4.1: minFlightMs 400 -> 250 and pushOverMs 400 -> 200. The real launcher (base/out/ballistics.json) flies
//            the dart for 0.37 s at the 6 deg low aim (about 0.42 s after the push starts): 400 ms missed that impact.
//   fallbacks: no impact seen but still for softStillMs after the flight ("still"); nothing for flightTimeoutMs after
//            launch ("timeout": also covers a silent IMU, checked from tick()).
struct Flight {
  enum St { WAIT = 0, FLYING, IMPACTED, LANDED };
  enum Ev { NONE = 0, LAUNCH, LAND };
  float launchG = 4, impactG = 3.5f, freeFallG = 0.6f, stillLo = 0.85f, stillHi = 1.15f;
  uint32_t launchHoldMs = 20, sampleSlackMs = 5, freeFallMs = 60, pushOverMs = 200, minFlightMs = 250, stillMs = 1000, softStillMs = 2000, flightTimeoutMs = 15000;
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
        // a tumbling dart may never read under 0.6 g (spin), so 200 ms after launch with the push over also counts
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
  bool armed = false, autoOnLanding = true, landed = false, landedFresh = false; uint32_t armT = 0;
  bool podLead = false; float kickPos = 0; uint32_t lastTickT = 0; bool tickSeen = false;   // kickPos: kick travel estimate, ms at full duty from stowed
  float kickSwing0 = 0;                                 // kickPos when this K_SWING started
  float kickRaw = 0;                                    // kick travel like kickPos but not clamped to kickSwingMs
  bool noGroundSeen = false;                            // NO_GROUND seen: the kick is at its end stop in air until it comes back
  bool capHit[NMOT] = {false, false, false, false};     // the current stall lock came from the hold cap (refusal text)
  bool leadLowOn = false; uint32_t leadLowT = 0;        // pods lead: deploy drawing nothing
  bool kickPlanted = false;                             // an anchor got past the kick swing since the last release (kick is on the ground)
  bool kickKnown = true;                                // kickPos is trusted (power-on with the legs stowed, or after a release)
  bool kickOverPair = false;                            // last pair reading said the kick (as the paused worm) is at stall
  bool deployOverPair = false;                          // same for the pods worm
  float deployPos = 0, deployOutOk = 0; bool deployKnown = true;         // pods travel estimate, ms at full duty from stowed (like kickPos)
  float deployBackStart = 0, deployBackDone = 0, deployBackOk = 0, kickBackOk = 0;   // release: travel back, last counted while not at stall
  bool freshHold[NMOT] = {false, false, false, false};  // presses were refused (busy, flight, fault): the next accepted one starts a new hold
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
  uint32_t pulseT = 0; bool paused = false, readDone = false; int pz = -1, hitsMain = 0, hitsOther = 0, hitsNoRise = 0, hitsRise = 0, nReads = 0, underRun = 0, pairKey = -1; bool riseSeen = false; float riseBase = 0;
  float bothMa = -1, soloMa = -1, bothMin = -1, diffMin = -1, bothRing[4] = {-1, -1, -1, -1}, railV = -1; bool needRise = false;
  uint32_t lastRunMs[NMOT] = {0, 0, 0, 0};
  Flight flight; Log log;
  float accG = 1.0f, peakG = 0;                        // peakG: highest |a| since the last arm
  uint32_t lowT2 = 0; bool lowOn2 = false;
  bool video = true;                                   // video power (camera + VTX), see PIN_VIDEO_GPIO
  float battV = -1; int batt = B_UNKNOWN; uint32_t lastRunT = 0; bool runSeen = false;   // battery at rest
  float upF = 1; int upN = 0; uint32_t upT = 0;        // orientation: filtered cosine of the tilt, still samples, last one
  float gF[3] = {0, 0, 1};                             // filtered gravity direction (unit vector, chip axes) while still
  // v4.1 anchoring and release state
  bool millPause = false, milled = false, rocking = false; uint32_t millUntil = 0, progT = 0, lastReadT = 0; float progPos = 0, deployEst = 0;
  int rocks = 0; float soilMa = -1; bool kickPulse = false; uint32_t millEndT = 0, millDurMs = 0;
  bool levelSettle = false; uint32_t levelT = 0;
  float g0[3] = {0, 0, 0}; bool g0ok = false, moving = false, moveOn = false; uint32_t moveT = 0;   // anchor motion (ANCHORED)

  // ---- orientation (BNO085 accelerometer while still)
  float tiltDeg() const { float c = upF > 1 ? 1 : upF < -1 ? -1 : upF; return acosf(c) * 57.29578f; }
  static float axisVal(const float v[3], int ax) { int i = ax < 0 ? -ax : ax; return (i >= 1 && i <= 3) ? v[i - 1] * (ax < 0 ? -1.f : 1.f) : 0.f; }
  bool noseKnown() const { int u = tu.imuUpAxis < 0 ? -tu.imuUpAxis : tu.imuUpAxis, n = tu.imuNoseAxis < 0 ? -tu.imuNoseAxis : tu.imuNoseAxis; return n >= 1 && n <= 3 && u >= 1 && u <= 3 && n != u; }
  // body roll and pitch in degrees from the filtered gravity (needs imuNoseAxis); left = up x nose
  void rollPitch(float& roll, float& pitch) const {
    int u = tu.imuUpAxis < 0 ? -tu.imuUpAxis : tu.imuUpAxis, n = tu.imuNoseAxis < 0 ? -tu.imuNoseAxis : tu.imuNoseAxis, k = 6 - u - n;
    float su = tu.imuUpAxis < 0 ? -1.f : 1.f, sn = tu.imuNoseAxis < 0 ? -1.f : 1.f;
    float eps = ((u % 3) + 1 == n) ? 1.f : -1.f;        // e_u x e_n = +e_k for cyclic (u, n, k), else -e_k
    float gU = su * gF[u - 1], gN = sn * gF[n - 1], gL = su * sn * eps * gF[k - 1];
    pitch = asinf(gN > 1 ? 1 : gN < -1 ? -1 : gN) * 57.29578f;
    roll = atan2f(gL, gU) * 57.29578f;
  }
  int orient(uint32_t now) const {
    if (!imuOk || upN < 20 || (int32_t)(now - upT) > 2000) return O_UNKNOWN;
    if (!noseKnown()) { float t = tiltDeg(); return t < tu.uprightMaxDeg ? O_UPRIGHT : t > tu.invertedMinDeg ? O_INVERTED : O_SIDE; }
    float roll, pitch; rollPitch(roll, pitch); float ar = fabsf(roll);
    if (pitch < -60.f) return O_NOSE_DOWN;              // steep: roll means little
    if (pitch > 60.f) return O_NOSE_UP;
    if (ar > tu.invertedMinDeg) return O_INVERTED;
    if (ar > tu.sideRollDeg) return O_SIDE;
    if (pitch < tu.pitchMinDeg) return O_NOSE_DOWN;
    if (pitch > tu.pitchMaxDeg) return O_NOSE_UP;
    return ar > tu.levelRollDeg ? O_LEANING : O_UPRIGHT;
  }
  static bool notUpright(int o) { return o == O_SIDE || o == O_INVERTED || o == O_NOSE_DOWN || o == O_NOSE_UP; }
  void orientSample(const float a[3], uint32_t now) {
    float n = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]) / 9.80665f;
    if (seq == DONE) motionCheck(n, now); else { moving = moveOn = false; g0ok = false; }
    int ax = tu.imuUpAxis < 0 ? -tu.imuUpAxis : tu.imuUpAxis;
    if (!(n > 0.85f && n < 1.15f) || ax < 1 || ax > 3) return;   // only while still (|a| near 1 g)
    float c = axisVal(a, tu.imuUpAxis) / (n * 9.80665f), u[3] = {a[0] / (n * 9.80665f), a[1] / (n * 9.80665f), a[2] / (n * 9.80665f)};
    if (upN == 0 || (int32_t)(now - upT) > 2000) { upF = c; upN = 0; for (int i = 0; i < 3; i++) gF[i] = u[i]; }
    else { upF += 0.05f * (c - upF); for (int i = 0; i < 3; i++) gF[i] += 0.05f * (u[i] - gF[i]); }   // about 0.1 s at 200 Hz
    if (upN < 1000) upN++;
    upT = now;
  }
  // anchored: the dart tilting or shaking means the anchor is moving under the tow (dart-physics: 2 deg or 0.15 g, 200 ms)
  void motionCheck(float nG, uint32_t now) {
    if (!imuOk) { moving = moveOn = false; return; }
    if (!g0ok) { if (upN >= 20 && (int32_t)(now - upT) < 500) { for (int i = 0; i < 3; i++) g0[i] = gF[i]; g0ok = true; } return; }
    float d = g0[0] * gF[0] + g0[1] * gF[1] + g0[2] * gF[2], ng = sqrtf(gF[0] * gF[0] + gF[1] * gF[1] + gF[2] * gF[2]);
    float ang = acosf(ng > 0 ? (d / ng > 1 ? 1 : d / ng < -1 ? -1 : d / ng) : 1) * 57.29578f;
    bool m = ang > tu.moveDeg || fabsf(nG - 1.f) > tu.moveG;
    bool was = moving;
    moving = heldT(m, moveOn, moveT, tu.moveHoldMs, now);
    if (moving && !was) { char b[64]; snprintf(b, sizeof b, "anchor moving (tilt %.1f deg, %.2f g)", ang, nG); ev(b); }
  }
  // ---- video power
  const char* setVideo(bool on, const char* why) {
    if (on != video || !why) { video = on; if (io.setVideo) io.setVideo(on); }
    if (why) { char b[64]; snprintf(b, sizeof b, "video %s (%s)", on ? "on" : "off", why); ev(b); }
    return nullptr;
  }
  // ---- battery at rest (called every tick with the rail reading)
  void battUpdate(float v, uint32_t now) {
    if (running()) { lastRunT = now; runSeen = true; return; }
    if (!(v == v) || v < 0.f) return;                  // no reading: keep the last state
    int b = batt;
    if (v < 3.f) { battV = v; b = B_NONE; }            // power loop plug out (USB only)
    else {
      if (runSeen && now - lastRunT < tu.battRestMs) return;   // the cells recover for a moment after a motor stops
      battV = battV < 3.f ? v : battV + 0.02f * (v - battV);   // about 0.25 s at 200 Hz
      bool wasCrit = batt == B_CRIT, wasLow = batt == B_LOW || wasCrit;
      if (battV < tu.battCritV || (wasCrit && battV < tu.battCritV + tu.battHystV)) b = B_CRIT;
      else if (battV < tu.battLowV || (wasLow && battV < tu.battLowV + tu.battHystV)) b = B_LOW;
      else b = B_OK;
    }
    if (b == batt) return;
    batt = b; char m[64];
    if (b == B_CRIT) { snprintf(m, sizeof m, "battery critical (%.2f V at rest): charge it", battV); ev(m); if (video) setVideo(false, "battery critical"); }
    else if (b == B_LOW) { snprintf(m, sizeof m, "battery low (%.2f V at rest)", battV); ev(m); }
    else if (b == B_OK) { snprintf(m, sizeof m, "battery ok (%.2f V at rest)", battV); ev(m); }
    else ev("no battery (rail under 3 V: power loop plug out?)");
  }

  bool seqRunning() const { return (seq >= K_SWING && seq <= D_AFTER) || (seq >= R_DRILL && seq <= R_KICK) || seq == K_LEVEL; }
  bool anchoring() const { return (seq >= K_SWING && seq <= D_AFTER) || seq == K_LEVEL; }
  bool releasing() const { return seq >= R_DRILL && seq <= R_KICK; }
  int phase() const {
    if (fault) return P_FAULT;
    if (releasing()) return P_RELEASING;
    if (anchoring()) return P_ANCHORING;
    if (flight.flying()) return P_FLIGHT;
    if (seq == DONE) return P_ANCHORED;
    if (landed && landedFresh && (seq == IDLE || seq == ABORTED) && !running()) return P_LANDED;   // not after a release or a hand move
    if (armed) return P_ARMED;
    return P_IDLE;
  }

  enum { W_FOOT_STALL = 1, W_DRILL_STALL = 2, W_PLANT_LIMIT = 4, W_PODS_LIMIT = 8, W_KICK_ALREADY = 16 };
  static const char* warnName(int b) { static const char* n[] = {"foot_stalled", "drills_stalled", "kick_plant_time_limit", "pods_time_limit", "kick_already_down"}; return n[b]; }
  int duty() const { return seqRunning() ? 255 : spd; }   // sequences always run at full duty (stall currents are for 255)
  static float fin(float v) { return v == v && v < 1e9f && v > -1e9f ? v : 0.f; }
  void ev(const char* s) { strncpy(lastEvent, s, sizeof(lastEvent) - 1); lastEvent[sizeof(lastEvent) - 1] = 0; if (io.log) io.log(s); }
  void setFault(int f, const char* msg, uint32_t now) {
    armed = false; if (seq == D_DEPLOY) { millDurMs = now - seqT0; millEndT = now; }   // heat bookkeeping (hotMillMs)                                    // a fault must never be cleared by the automatic landing /auto
    if (!seqRunning()) for (int m = 0; m < NMOT; m++) if (dir[m]) { stallLock[m] = dir[m]; capHit[m] = false; }   // a held key stays off until released
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
    uint32_t ph = now - pulseT;
    if (!paused) {
      if (ph >= 380 && ph < 400) bothMa = fMa;
      if (ph >= 400) { paused = true; pz = main; readDone = false; pulseT = now; io.setMotor(main, 0, 0); }
      return 0;
    }
    int r = 0;
    if (!readDone && ph >= 110) {
      readDone = true; soloMa = fMa; lastReadT = now;
      if (bothMa >= 0) {
        // sequences: the total must also have risen over the phase's lowest reading (guards against the auger's
        // current changing while the worm pauses, which biases both - alone). The rise asked for is a share of the
        // way from the worm's own lowest reading in this step (at least its free current) to the line, at least 25 mA,
        // so a worm already working hard on the ground still shows its stall
        float base = lim(tu.freeMa[main]); if (diffMin > base) base = diffMin;
        float need = tu.riseFrac * (mainLim - base); if (need < 25.f) need = 25.f;
        bool rose = !needRise || (bothMin >= 0 && bothMa - bothMin * vScale() > need)   // bothMin is kept at refV: a rail sag does not hide a rise
                    || soloMa < tu.noCurrentFrac * lim(tu.freeMa[other]);   // other draws nothing (open wire): nothing to fake with
        // once a reading showed the rise and the line together, later readings over the line count without the rise
        // (the ring of 4 forgets the level before the stall), until two readings in a row are under the line
        bool over = bothMa - soloMa > mainLim;
        if (!over) { if (++underRun >= 2) riseSeen = false; } else underRun = 0;
        bool hitR = over && (rose || (riseSeen && bothMa - riseBase * vScale() > need));   // still risen over the level before the rise
        if (over && rose && !riseSeen) { riseSeen = true; riseBase = bothMin; }
        hitsMain = hitR ? hitsMain + 1 : 0;
        hitsRise = ((hitsRise << 1) | (hitR ? 1 : 0)) & 0xF;   // one noisy miss does not reset it
        if (__builtin_popcount(hitsRise) >= 3 || __builtin_popcount(hitsRise & 7) >= 2) hitsMain = 2;   // 3 of 4, or 2 of the last 3 (noisy stall just over the line)
        if (main == KICK) kickOverPair = bothMa - soloMa > mainLim;
        if (main == DEPLOY) deployOverPair = bothMa - soloMa > mainLim;
        if (main == KICK && !kickOverPair) kickBackOk = kickBackDone;       // release: travel counted up to a reading under the line
        if (main == DEPLOY && !deployOverPair) { deployBackOk = deployBackDone; deployOutOk = deployPos; }
        // anchor: a reading over the line means the pods stopped somewhere since the last good reading; drop the travel
        // counted meanwhile (it was counted at full speed), so a soil stall is not mistaken for travel (v4.1 milling)
        if (main == DEPLOY && (!deployOverPair || deployPos > deployOutOk)) deployEst = deployPos;   // before the correction (at most one cycle long)
        if (main == DEPLOY && deployOverPair && seq == D_DEPLOY && deployPos > deployOutOk) deployPos = deployOutOk;
        if (other == DRILLS && seq == D_DEPLOY && deployPos > 0.4f * tu.podsTravelMs) {   // augers in the soil: soil reading
          float sm = soloMa / vScale(); if (sm > soilMa) soilMa = sm;
        }
        // a worm already at its stop when the step starts never shows a rise: 3 of the first 4 readings of the step
        // over the line also count (one noisy reading does not spoil it). Only at the start: a stop reached later
        // shows a rise, and an auger whose current drops during the pause could otherwise fake a stall
        if (nReads < 100) nReads++;
        hitsNoRise = ((hitsNoRise << 1) | (bothMa - soloMa > mainLim ? 1 : 0)) & 0xF;
        if (nReads <= 4 && __builtin_popcount(hitsNoRise) >= 3) hitsMain = 2;
        // lowest of the last 4 readings (not of the whole step): an auger whose current falls during the step
        // (crust, then soft soil) does not hide a later stall
        for (int k = 3; k > 0; k--) bothRing[k] = bothRing[k - 1];
        bothRing[0] = bothMa / vScale(); bothMin = -1;   // stored at refV
        for (int k = 0; k < 4; k++) if (bothRing[k] >= 0 && (bothMin < 0 || bothRing[k] < bothMin)) bothMin = bothRing[k];
        if (diffMin < 0 || bothMa - soloMa < diffMin) diffMin = bothMa - soloMa;
      }
      hitsOther = soloMa > otherLim ? hitsOther + 1 : 0;
      if (hitsMain >= 2) r |= 1;
      if (hitsOther >= 2) r |= 2;
    }
    if (ph >= 150) { endPause(); pulseT = now; }
    return r;
  }
  void endPause() { if (paused && pz >= 0 && dir[pz]) io.setMotor(pz, dir[pz], duty()); paused = false; pz = -1; bothMa = -1; }
  void pairReset(uint32_t now) { endPause(); pulseT = now; readDone = false; hitsMain = hitsOther = hitsNoRise = hitsRise = nReads = underRun = 0; riseSeen = false; soloMa = -1; bothMin = -1; diffMin = -1; for (float& b : bothRing) b = -1; condT = 0; kickOverPair = false; deployOverPair = false; }
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
    setVideo(true, nullptr);                          // camera and VTX on (if the switch is fitted)
    stopAll(now); kickKnown = kickStowed; kickPos = 0; kickRaw = 0; kickPlanted = false; noGroundSeen = false; deployKnown = kickStowed; deployPos = 0;
    if (resetFault) { fault = resetFault; char b[64]; snprintf(b, sizeof b, "boot after %s reset, motors off", faultName(resetFault)); ev(b); }
    else ev("boot, motors off");
  }

  // returns nullptr when accepted, else the reason
  // disarm=false (GET /stop?disarm=0) stops the motors but keeps the landing detector armed and flight tracking going
  const char* stop(uint32_t now, bool disarm = true) {
    bool wasSeq = seqRunning();
    if (seq == D_DEPLOY) { millDurMs = now - seqT0; millEndT = now; }   // heat bookkeeping (hotMillMs)
    // explicit STOP: a key that any client is still repeating (running, in a deadman gap, or refused while a sequence
    // ran) cannot restart its motor until it has been released
    if (disarm) for (int m = 0; m < NMOT; m++) {
      uint32_t w = tu.wifiDeadmanMs + 250;
      bool neg = pressSeen[m][0] && (int32_t)(now - pressT[m][0]) < (int32_t)w;
      bool pos = pressSeen[m][1] && (int32_t)(now - pressT[m][1]) < (int32_t)w;
      if (dir[m] && !wasSeq) { if (dir[m] > 0) pos = true; else neg = true; }
      if (pos && neg) { bothLock[m] = true; lastCmd[m] = now; }               // both keys of this motor still held: lock both
      else if (pos || neg) { int d = pos ? 1 : -1; stallLock[m] = d; capHit[m] = false; lastCmd[m] = now; lastKeyDir[m] = d; }
    }
    stopAll(now);
    if (wasSeq) seq = ABORTED;
    if (disarm) {
      if (flight.flying()) { flight.land(now, "stop"); landed = true; landedFresh = true; uint32_t fr = now + 3000, cap = freezeCap(); log.scheduleFreeze((int32_t)(fr - cap) > 0 ? cap : fr); }
      armed = false;
    }
    fault = F_NONE;
    ev(wasSeq ? "stop: sequence aborted" : "stop");
    return nullptr;
  }
  // force: /auto?force=1 (try even when the IMU says the dart is not upright)
  const char* startAuto(uint32_t now, bool force = false) {
    if (flight.flying()) return "in flight";
    if (anchoring()) return nullptr;                  // already running: a repeated request is not an error
    if (seqRunning()) return "busy";
    if (seq == DONE) return "already anchored, release first";
    if (!inaOk) return "no current sensor";          // refuse before changing anything
    if (batt == B_CRIT) return "battery low: charge it (release still works)";   // never anchor without the charge to let go
    if (millDurMs > tu.hotMillMs && now - millEndT < tu.hotRestMs) return "motors hot after a long milling: wait 60 s";
    int o = orient(now);
    if (!force) {                                     // on its side or back the kick swings through air: say so at once
      if (o == O_SIDE) return "not upright: on its side (force=1 tries anyway)";
      if (o == O_INVERTED) return "not upright: upside down (force=1 tries anyway)";
      if (o == O_NOSE_DOWN) return "not upright: nose down (force=1 tries anyway)";
      if (o == O_NOSE_UP) return "not upright: nose up (force=1 tries anyway)";
    }
    if (armed) { armed = false; ev("auto: disarmed first (the launch was not seen)"); }
    stopAll(now); pairReset(now); fault = F_NONE; armed = false; warn = 0; footStalled = drillsStalled = false;
    millPause = milled = rocking = false; rocks = 0; soilMa = -1; levelSettle = false;
    seqStart = now; peakMa = 0;
    if (o == O_LEANING && !force) {                   // leaning on a pod: the pods level the hull first (drills off)
      seq = K_LEVEL; seqT0 = now; drive(DEPLOY, +1, tu.levelMs, now); ev("auto: leaning, pods to 20 deg first (drills off)");
      return nullptr;
    }
    startSwing(now);
    return nullptr;
  }
  void startSwing(uint32_t now) { seq = K_SWING; seqT0 = now; condT = 0; pairReset(now); kickSwing0 = kickPos; drive(KICK, +1, tu.kickMaxMs + 500, now); ev("auto: kick swing"); }
  const char* startRelease(uint32_t now) {
    if (flight.flying()) return "in flight";
    if (releasing()) return nullptr;                  // already running
    if (seqRunning()) return "busy";
    if (!inaOk) return "no current sensor";
    stopAll(now); pairReset(now); fault = F_NONE; armed = false; warn = 0; millPause = rocking = false; rocks = 0;
    seq = R_DRILL; seqT0 = seqStart = now; peakMa = 0; drive(DRILLS, -1, tu.drillLeadMs, now); ev("release: drills out");
    return nullptr;
  }
  const char* arm(bool on, bool withAuto, uint32_t now) {
    if (!on) { armed = false; ev("disarmed"); return nullptr; }
    if (seqRunning()) return "busy";
    if (flight.flying()) return "in flight";
    if (!imuOk) return "no IMU";
    if (seq == DONE) return "anchored: release first";
    if ((kickKnown && kickPos > 0.1f * tu.kickSwingMs) || (deployKnown && deployPos > 0.1f * tu.podsTravelMs)) return "legs out: release first";   // legs out: arming would forget that
    if (withAuto && !inaOk) return "no current sensor";
    if (withAuto && !running() && railV >= 0.f && railV < tu.armMinV) return "battery low";   // includes 0 V (plug out); log-only arm still allowed
    // the moment of arming counts as still, so a shot right after the arm is still seen
    flight.reset(); flight.armedAt(now); armT = now; log.unfreeze(); landed = false; landedFresh = false; peakG = 0; armed = true; autoOnLanding = withAuto; fault = F_NONE;
    if (seq == ABORTED || seq == RELEASED) seq = IDLE;   // a new shot
    ev(withAuto ? "armed: auto-anchor on landing" : "armed: log only");
    if (!video) setVideo(true, "armed: the flight is filmed");
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
    if (seqRunning() || flight.flying() || fault) { lastCmd[m] = now; freshHold[m] = true; }   // a refused press still counts as "held" for the locks
    if (seqRunning()) return "busy";                  // a sequence owns the motors; stop first
    if (flight.flying()) return "in flight";
    if (fault) return "fault latched: send stop first";
    if (runMs > tu.manualMaxMs) runMs = tu.manualMaxMs;
    // same hold if this press comes within max(last run time, WiFi deadman) + 250 ms (so at least 600 ms)
    uint32_t win = (lastRunMs[m] > tu.wifiDeadmanMs ? lastRunMs[m] : tu.wifiDeadmanMs) + 250;
    if (win < tu.wifiDeadmanMs + 150) win = tu.wifiDeadmanMs + 150;
    bool repeat = now - lastCmd[m] < win; lastCmd[m] = now;
    // a press of the other direction (refused at its own hold limit, say) must not shrink the window with its shorter ms
    // and so restart the hold cap of the direction still being driven (v4.1, fuzz seed 8); an accepted press sets it below
    if (d == lastKeyDir[m] || !lastKeyDir[m]) lastRunMs[m] = runMs;
    // the lock holds while THIS direction's key keeps coming (presses of the other direction do not count)
    bool dirRepeat = prevPressSeen && (int32_t)(now - prevPressT) < (int32_t)win;
    if (stallLock[m] == d && dirRepeat) return capHit[m] ? "hold limit: release the key first" : "stalled: release the key first";
    if (stallLock[m] == d) { stallLock[m] = 0; capHit[m] = false; }          // only a fresh press of the locked direction clears its lock
    // a repeat within the deadman window (even after a short gap that let the motor stop) continues the same hold:
    // the hold cap and the stall start-up time are not restarted by slow repeats
    // both directions of one motor held (1 and ! together): stop it and refuse both until neither has been pressed for a window
    if (bothLock[m]) { if (repeat) return "both directions held"; bothLock[m] = false; }
    bool otherRecent = pressSeen[m][d < 0] && (int32_t)(now - pressT[m][d < 0]) < (int32_t)win;
    if (repeat && dirRepeat && otherRecent && lastKeyDir[m] == -d && dir[m] == -d) { bothLock[m] = true; drive(m, 0, 0, now); ev("both directions held: motor stopped"); return "both directions held"; }
    if (freshHold[m]) { repeat = false; freshHold[m] = false; }   // the first accepted press after refused ones starts the hold cap
    bool cont = repeat && lastKeyDir[m] == d; lastKeyDir[m] = d;
    if (!repeat) heldStart[m] = now;                  // per motor: switching direction does not restart the hold cap
    uint32_t cap = inaOk ? tu.manualMaxMs : tu.manualMaxNoInaMs, used = now - heldStart[m];
    if (used >= cap) { stallLock[m] = d; capHit[m] = true; return "hold limit: release the key first"; }
    if (runMs > cap - used) runMs = cap - used;       // the whole hold, not each repeat, is capped
    if (seq == DONE || seq == RELEASED || seq == ABORTED) { seq = IDLE; landedFresh = false; }   // legs moved by hand: no longer a known state (not LANDED either)
    uint32_t keepT0 = motT0[m];
    lastRunMs[m] = runMs;
    drive(m, d, runMs, now);
    if (cont) motT0[m] = keepT0;
    return nullptr;
  }
  // WiFi station left: manual (hold-to-run) motors stop at once; a running sequence carries on (it is autonomous)
  void linkLost(uint32_t now) { if (!seqRunning() && running()) { stopAll(now); ev("WiFi client lost, motors stopped"); } }
  // the web page let go of its last held key (or lost focus): stop the manual motors only. A running sequence carries on
  // and a latched fault stays (the key itself was refused; lifting a finger must not abort an anchor)
  void noGroundFault(uint32_t now) {   // the kick is at its own end stop now: its position is known, and it is not planted
    kickKnown = true; kickPos = tu.kickSwingMs; kickPlanted = false; noGroundSeen = true;
    setFault(F_NO_GROUND, "auto: kick reached its end stop without touching ground", now);
  }
  void keyRelease(uint32_t now) { if (!seqRunning() && running()) { stopAll(now); ev("keys released, motors stopped"); } }

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
      landed = true; landedFresh = true;
      uint32_t fr = flight.tLand + 3000, cap = freezeCap();
      if ((int32_t)(fr - cap) > 0) fr = cap;         // a long (timeout) flight: keep launch and impact rather than the end
      if (!log.frozen) log.scheduleFreeze((int32_t)(fr - now) > 0 ? fr : now);
      char b[64]; snprintf(b, sizeof b, "landed (%s)", flight.how); ev(b);
      bool go = armed && autoOnLanding; armed = false;
      if (go && flight.fallback()) { go = false; ev("no impact seen: auto not started, send /auto"); }
      if (go) { int o = orient(now); if (notUpright(o)) { go = false; char m[64]; snprintf(m, sizeof m, "landed %s: auto not started", o == O_SIDE ? "on its side" : o == O_INVERTED ? "upside down" : o == O_NOSE_DOWN ? "nose down" : "nose up"); ev(m); } }
      if (go) { const char* r = startAuto(now); if (r) { char m[64]; snprintf(m, sizeof m, "auto on landing refused: %s", r); ev(m); } }
    }
  }
  // 50 Hz flight log feed (the sketch calls it with the newest quaternion and acceleration)
  void imuSample(uint32_t now, const float q[4], const float a[3]) { orientSample(a, now); log.offer(now, q, a); }

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
    battUpdate(curOk ? v : NAN, now);
    {                                                 // kick travel bookkeeping (only while the kick is really driven)
      uint32_t dt = tickSeen ? now - lastTickT : 0; lastTickT = now; tickSeen = true; if (dt > 50) dt = 50;
      // counts only real travel: not while paused, not while planting (the kick is on the ground), and outward not
      // while the kick alone draws stall current
      // stalled in either direction: alone, current over its stall line; in a pair, the last pair reading
      bool alone = dir[KICK] && !dir[DEPLOY] && !dir[DRILLS] && !dir[FOOT];
      bool stalled = alone ? fMa > stallLim(KICK) : kickOverPair;
      // by hand with two or more other motors (or in a pair where the kick is not the one checked): no reading of the kick
      // alone, so a kick pushing on a stop would still be counted. Travel is no longer trusted until the next release
      if (dir[KICK] && !alone && !seqRunning() && !(pairKey >= 0 && pairKey / 8 == KICK) && kickKnown) { kickKnown = false; ev("kick travel unknown (driven with other motors) until the next release"); }
      // by hand, pushing out against a stop past the start-up surge (even in runs too short for the stall stop): on the ground or at its end stop
      if (!seqRunning() && dir[KICK] > 0 && stalled && now - motT0[KICK] > tu.surgeIgnoreMs && !noGroundSeen) kickPlanted = true;
      if (dir[KICK] && !(paused && pz == KICK) && seq != K_PLANT && !stalled) {
        float step = (float)dt * duty() / 255.f * vScale();   // speed scales with the rail (kickSwingMs is at refV)
        kickPos += dir[KICK] * step;
        if (dir[KICK] < 0 && step > 0) kickPlanted = false;   // coming back: no longer on the ground
        if (seq == R_KICK && dir[KICK] < 0) { kickBackDone += step; if (alone) kickBackOk = kickBackDone; }
        if (kickPos < 0) kickPos = 0;
        if (kickPos < 0.5f * tu.kickSwingMs) noGroundSeen = false;   // came well back from the end stop
        if (kickPos > tu.kickSwingMs) kickPos = tu.kickSwingMs;
        kickRaw += dir[KICK] * step; if (kickRaw < 0) kickRaw = 0;   // not clamped: the bench measurement of the swing
      }
      // pods travel, counted the same way (the pods worm is always the checked one in a pair)
      bool dAlone = dir[DEPLOY] && !dir[KICK] && !dir[DRILLS] && !dir[FOOT];
      // release, while the drills still turn with the lifting pods (0.6 s): counted in full (no reading of the pods alone yet)
      bool dStalled = dAlone ? fMa > stallLim(DEPLOY) : (seq == R_PODS ? false : deployOverPair);
      int nOn = (dir[DEPLOY] != 0) + (dir[KICK] != 0) + (dir[DRILLS] != 0) + (dir[FOOT] != 0);
      if (dir[DEPLOY] && nOn > 2 && !seqRunning() && deployKnown) { deployKnown = false; ev("pods travel unknown (driven with other motors) until the next release"); }
      if (dir[DEPLOY] && !(paused && pz == DEPLOY) && !dStalled) {   // v4.1: the pods lead counts too (alone, read every tick)
        // counted at full speed even under load (a load factor from the pair readings was tried: the auger's current
        // change in each pause biased it low, so the pods then milled their own end stop and release jams were missed)
        float step = (float)dt * duty() / 255.f * vScale();
        deployPos += dir[DEPLOY] * step;
        if (seq == R_PODS && dir[DEPLOY] < 0) { deployBackDone += step; deployBackOk = deployBackDone; }   // alone: per tick; with the drills: in full
        if (dAlone) deployOutOk = deployPos;
        if (deployPos < 0) deployPos = 0;
        if (deployPos > tu.podsTravelMs) deployPos = tu.podsTravelMs;
      }
      // presses older than a minute are forgotten (signed time compares on them would turn after 24.8 days)
      for (int m = 0; m < NMOT; m++) for (int k = 0; k < 2; k++) if (pressSeen[m][k] && now - pressT[m][k] > 60000u) pressSeen[m][k] = false;
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
      if (!seqRunning() && now - heldStart[m] >= (inaOk ? tu.manualMaxMs : tu.manualMaxNoInaMs)) { stallLock[m] = dir[m]; capHit[m] = true; }   // hold cap reached: off until released
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
        bool kickRawOut = dir[KICK] > 0;
        char b[64]; int n = snprintf(b, sizeof b, "stalled:");
        for (int m = 0; m < NMOT; m++) if (stopMask & (1 << m)) { n += snprintf(b + n, sizeof b - n, " %s", motorName(m)); stallLock[m] = dir[m]; capHit[m] = false; if (m == KICK && dir[m] > 0 && !noGroundSeen && nrun <= 2) kickPlanted = true; drive(m, 0, 0, now); }
        // (a kick driven out by hand until it stalls: ground or its own end stop cannot be told apart; the operator put it there.
        // Not from the 3-or-4-motor rule, which stops the whole group without knowing which motor stalled: fuzz seed 1 ep 5682)
        snprintf(b + n, sizeof b - n, " at %.0f mA, stopped", ma); ev(b); manOn = false;
        if ((stopMask & (1 << KICK)) && kickRawOut && kickKnown) {   // BENCH_TEST step 5 reads kickSwingMs from this
          char t[64]; snprintf(t, sizeof t, "kick travel from stowed %.0f ms (at 8.0 V)", kickRaw); ev(t);
        }
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
        stallLock[only] = dir[only]; capHit[only] = false; drive(only, 0, 0, now); ev(b);
      }
    }

    uint32_t t = now - seqT0;
    switch (seq) {
      case K_SWING:
        if (t > tu.surgeIgnoreMs && held(fMa > lim(tu.kickContactMa), tu.contactHoldMs, now) && fMa > stallLim(KICK)) {
          // stall with no contact phase. moved = this swing's travel; pos = travel from stowed (this swing alone when unknown)
          //   planted since the last release (a retried /auto)  -> already down: plant the foot alone
          //   pos at the noGround line or more                    -> its own end stop in air: NO_GROUND
          //   near stowed (travel known), or moved over 10 %      -> a jam (or wired backwards, or hard ground with no contact): KICK_JAM
          //   otherwise (barely moved, travel unknown or part way out) -> already down
          {
            float moved = kickPos - kickSwing0, pos = kickKnown ? kickPos : moved, S = tu.kickSwingMs;
            if (!kickPlanted && pos >= tu.noGroundFrac * S) { noGroundFault(now); return; }
            if (!kickPlanted && ((kickKnown && kickPos < 0.25f * S) || moved > 0.1f * S)) {
              setFault(F_KICK_JAM, kickKnown && kickPos < 0.25f * S ? "auto: kick stalled near stowed (jammed, backwards, or powered on with legs out)"
                                                                   : "auto: kick stalled part way out with no ground contact (jammed?)", now); return; }
          }
          drive(KICK, 0, 0, now); drive(FOOT, +1, tu.footAfterMs * 2, now); seq = F_AFTER; seqT0 = now; condT = 0; pairReset(now);
          kickPlanted = true; warn |= W_KICK_ALREADY; ev("auto: kick already down, foot planting");
        } else if (t > tu.surgeIgnoreMs && condT && now - condT >= tu.contactHoldMs) {
          // "contact" out at the end stop in air (a soft stop or a flexing leg): no ground there (geometry: ground comes before 80 %)
          if (!kickPlanted && (kickKnown ? kickPos : kickPos - kickSwing0) >= tu.noGroundFrac * tu.kickSwingMs) { noGroundFault(now); return; }
          drive(KICK, +1, tu.kickPlantMaxMs + 500, now); drive(FOOT, +1, tu.kickPlantMaxMs + 500, now); seq = K_PLANT; seqT0 = now; pairReset(now); kickPlanted = true; ev("auto: foot touches, foot auger on");   // kickPos stays at the contact point
        }
        break;
      case K_PLANT: {   // kick keeps pushing while the foot screws in
        int r = stallPair(KICK, FOOT, now);
        if (r & 2) { drive(FOOT, 0, 0, now); warn |= W_FOOT_STALL; footStalled = true; ev("auto: foot stalled (stone?), kick carries on"); }
        bool lim2 = t >= tu.kickPlantMaxMs;
        if (lim2) warn |= W_PLANT_LIMIT;           // housing stall never seen: the kick is on the ground anyway
        if ((r & 1) || lim2) { drive(KICK, 0, 0, now); if (!footStalled) drive(FOOT, +1, tu.footAfterMs, now); seq = F_AFTER; seqT0 = now; pairReset(now);
          kickPulse = (r & 1) && !footStalled;      // dart-physics (8): in loam the kick stalls a few degrees short; it pulses on while the foot screws in
          ev(lim2 ? "auto: kick plant time limit, foot finishing" : kickPulse ? "auto: kick down, foot finishing (kick pulsing)" : "auto: kick down, foot finishing"); }
        break;
      }
      case F_AFTER:
        if (kickPulse && dir[FOOT]) {               // kick pushes again in pulses (on kickPulseOnMs, off kickPulseOffMs) while the foot finishes
          uint32_t c = t % (tu.kickPulseOnMs + tu.kickPulseOffMs);
          if (c < tu.kickPulseOnMs) { if (!dir[KICK]) drive(KICK, +1, tu.kickPulseOnMs - c, now); }
          else if (dir[KICK]) drive(KICK, 0, 0, now);
        }
        if (!kickPulse && soloStalled(FOOT, now)) { drive(FOOT, 0, 0, now); warn |= W_FOOT_STALL; ev("auto: foot stalled (full depth or stone)"); }
        if (!dir[FOOT]) {
          if (dir[KICK]) drive(KICK, 0, 0, now);
          kickPulse = false; drive(DEPLOY, +1, tu.deployMaxMs, now); podLead = true; seq = D_DEPLOY; seqT0 = now; pairReset(now); deployOutOk = deployPos; ev("auto: deploy with drills spinning"); }
        break;
      case K_LEVEL:     // leaning on a pod: pods to about 20 deg with the drills off levels the hull (dart-physics)
        if (!levelSettle) { if (!dir[DEPLOY]) { levelSettle = true; levelT = now; } break; }
        if (now - levelT < tu.levelSettleMs) break;
        { int o = orient(now);
          if (notUpright(o)) { setFault(F_NOT_UPRIGHT, "auto: still not upright after levelling", now); return; }
          char b[64]; snprintf(b, sizeof b, "auto: levelled (%s)", orientName(o)); ev(b); }
        startSwing(now);
        break;
      case D_DEPLOY: {  // pods swing down while the augers cut
        if (podLead) {                                // first 300 ms: deploy worm alone (open wire? already down?)
          bool low = fMa < tu.noCurrentFrac * lim(tu.freeMa[DEPLOY]);   // judged over the last 100 ms+, not one sample
          if (t < 100 || !low) leadLowOn = false; else if (!leadLowOn) { leadLowOn = true; leadLowT = now; }
          if (t < tu.podLeadMs) break;
          podLead = false;
          if (leadLowOn && now - leadLowT >= 100) { setFault(F_NO_CURRENT, "deploy draws no current: wire open or shorted", now); return; }
          // no "already down" shortcut here: augers touching soil without spinning could fake it; the pair check decides
          drive(DRILLS, +1, tu.deployMaxMs + tu.drillAfterMs, now); pairReset(now); progT = now; progPos = deployPos;
          break;
        }
        if (deployPos - progPos >= 0.02f * tu.podsTravelMs) { progPos = deployPos; progT = now; }
        bool stuck = now - progT >= tu.millNoProgressMs;
        if (millPause) {                              // deploy held off: milling (soil) or letting the cut relax
          if (dir[DRILLS] && soloStalled(DRILLS, now)) { drive(DRILLS, 0, 0, now); warn |= W_DRILL_STALL; drillsStalled = true; ev("auto: drills stalled (stone?), pods carry on"); }
          if ((int32_t)(now - millUntil) < 0) break;
          millPause = false; int32_t left = (int32_t)tu.deployMaxMs - (int32_t)t;
          if (left > 0 && !stuck) { drive(DEPLOY, +1, (uint32_t)left, now); pairReset(now); break; }
          bool down = stuck && deployOutOk >= tu.podsStuckDownFrac * tu.podsTravelMs;
          if (!down) warn |= W_PODS_LIMIT; else deployPos = tu.podsTravelMs;
          if (!drillsStalled) drive(DRILLS, +1, tu.drillAfterMs, now);
          millDurMs = now - seqT0; millEndT = now; seq = D_AFTER; seqT0 = now; pairReset(now);
          ev(down ? "auto: pods down (no progress, past 60 %: the stop), drills finishing" : stuck ? "auto: pods made no progress for 5 s (firm soil), drills finishing" : "auto: deploy time limit, drills finishing");
          break;
        }
        int r = stallPair(DEPLOY, DRILLS, now);
        if (t < tu.podsMinMs) r &= ~1;
        if (r & 2) { drive(DRILLS, 0, 0, now); warn |= W_DRILL_STALL; drillsStalled = true; ev("auto: drills stalled (stone?), pods carry on"); }
        bool st = r & 1;
        // a deploy stall before most of the travel is soil, not the end stop: mill (dart-physics 7 Nov). Travel unknown
        // (after a reset): the first stall is the stop, as before
        // the drill pair near its stall (its last reading alone, during the worm's pause, over 85 % of its line): pause the
        // deploy drillRelaxMs so the cut relaxes (dart-physics 7 Nov). The total current cannot tell this from the pods at
        // their stop, so only the drills' own reading counts
        bool heavy = dir[DRILLS] && dir[DEPLOY] && lastReadT == now && soloMa > tu.drillRelaxFrac * stallLim(DRILLS) && !(r & 2);
        if ((st && deployKnown && deployEst < tu.podsDownFrac * tu.podsTravelMs && !stuck) || (heavy && !stuck)) {
          drive(DEPLOY, 0, 0, now); millPause = true; millUntil = now + (heavy ? tu.drillRelaxMs : tu.millOffMs);
          if (heavy && !st) ev("auto: drills near stall, deploy paused 0.5 s");
          else if (!milled) { milled = true; ev(heavy ? "auto: pods in soil, milling; drills near stall, deploy paused 0.5 s" : "auto: pods in soil, milling (deploy pulses, drills cutting)"); }
          break;
        }
        if (!milled && t >= tu.podsPlainMaxMs) { drive(DEPLOY, 0, 0, now); st = false; }   // no soil seen: the old 6 s limit
        if (st || !dir[DEPLOY] || stuck) {
          bool down = st || (stuck && deployOutOk >= tu.podsStuckDownFrac * tu.podsTravelMs);
          if (!down) warn |= W_PODS_LIMIT;
          if (down) deployPos = tu.podsTravelMs;    // taken as the down stop: the pods are fully out (release bookkeeping)
          drive(DEPLOY, 0, 0, now); if (!drillsStalled) drive(DRILLS, +1, tu.drillAfterMs, now); else drive(DRILLS, 0, 0, now); millDurMs = now - seqT0; millEndT = now; seq = D_AFTER; seqT0 = now; pairReset(now);
          ev(st ? "auto: pods down, drills finishing" : down ? "auto: pods down (no progress, past 60 %: the stop), drills finishing" : stuck ? "auto: pods made no progress for 5 s (firm soil), drills finishing" : "auto: deploy time limit (down or stalled), drills finishing");
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
        // drills that stall unscrewing are rocked: in for rockInMs, then out again (unscrewing needs about 1.3 x the
        // install torque, dart-physics 7 Nov); DRILL_JAM after rockMax rocks
        if (rocking) { if (!dir[DRILLS]) { rocking = false; condT = 0; drive(DRILLS, -1, tu.drillLeadMs, now); } break; }
        if (soloStalled(DRILLS, now)) {
          if (rocks >= tu.rockMax) { setFault(F_DRILL_JAM, "release: drills jammed (rocked 5 times)", now); return; }
          rocks++; rocking = true; condT = 0; drive(DRILLS, +1, tu.rockInMs, now);
          if (rocks == 1) ev("release: drills stuck, rocking them (in 0.3 s, out again)");
          break;
        }
        // the augers may still be threaded in soil: the drills keep unscrewing while the pods lift (no deploy-alone lead)
        if (!dir[DRILLS]) { drive(DRILLS, -1, tu.deployMaxMs, now); drive(DEPLOY, -1, tu.deployMaxMs, now); seq = R_PODS; seqT0 = now; pairReset(now); deployBackStart = deployPos; deployBackDone = deployBackOk = 0; progT = now; progPos = 0; milled = false; ev("release: pods up, drills still out"); }
        break;
      case R_PODS: {
        if (deployBackOk - progPos >= 0.02f * tu.podsTravelMs) { progPos = deployBackOk; progT = now; }
        bool stuck = now - progT >= tu.millNoProgressMs;
        if (millPause) {                              // milling up: deploy paused while the drills unscrew a little
          if ((int32_t)(now - millUntil) < 0) break;
          millPause = false;
          if (stuck) { setFault(F_PODS_JAM, "release: pods made no progress for 5 s (jammed)", now); return; }
          int32_t left = (int32_t)tu.deployMaxMs - (int32_t)t;
          if (left > 0) { drive(DEPLOY, -1, (uint32_t)left, now); pairReset(now); break; }
          warn |= W_PODS_LIMIT; drive(DRILLS, 0, 0, now); drive(FOOT, -1, tu.footLeadMs, now); seq = R_FOOT; seqT0 = now; pairReset(now);
          ev("release: pods time limit (stowed or stalled), foot out");
          break;
        }
        // the drills keep unscrewing for the first second, then stop: the pods finish alone, read every tick, so their
        // travel and their stop are exact (a jam part way up is told from the stowed stop)
        if (dir[DRILLS] && t >= tu.podsWithDrillsMs && hitsOther == 0) { drive(DRILLS, 0, 0, now); pairReset(now); }   // not while a drill jam reading is pending
        int r = stallPair(DEPLOY, DRILLS, now);
        if (t < tu.podsMinMs) r &= ~1;
        if (r & 2) { setFault(F_DRILL_JAM, "release: drills jammed", now); return; }
        if ((r & 1) || !dir[DEPLOY]) {
          bool st = r & 1; if (!st) warn |= W_PODS_LIMIT;
          // a stall before the pods came most of the way back: the soil still holds the augers (mill them up), and a
          // jam once no progress is made for millNoProgressMs
          if (st && deployKnown && deployBackOk < 0.6f * deployBackStart) {
            if (stuck) { setFault(F_PODS_JAM, "release: pods stalled before they were up (jammed)", now); return; }
            drive(DEPLOY, 0, 0, now); drive(DRILLS, -1, tu.millOffMs, now); millPause = true; millUntil = now + tu.millOffMs;
            if (!milled) { milled = true; ev("release: pods held in soil, milling up (drills unscrew in the pauses)"); }
            break;
          }
          drive(DEPLOY, 0, 0, now); drive(DRILLS, 0, 0, now); drive(FOOT, -1, tu.footLeadMs, now); seq = R_FOOT; seqT0 = now; pairReset(now);
          ev(st ? "release: pods up, foot out" : "release: pods time limit (stowed or stalled), foot out");
        }
        break;
      }
      case R_FOOT:
        if (soloStalled(FOOT, now)) { setFault(F_FOOT_JAM, "release: foot jammed", now); return; }
        if (!dir[FOOT]) { drive(FOOT, -1, tu.footOutMs, now); drive(KICK, -1, tu.kickMaxMs, now); seq = R_KICK; seqT0 = now; pairReset(now);
          kickBackStart = kickPos; kickBackDone = kickBackOk = 0; ev("release: kick back, foot still out"); }
        break;
      case R_KICK: {
        int r = stallPair(KICK, FOOT, now);
        if (t < tu.kickBackMinMs) r &= ~1;
        if (r & 2) { setFault(F_FOOT_JAM, "release: foot jammed", now); return; }
        if (r & 1) {
          // the real travel back must cover most of the travel out (the estimate out is short: planting is not counted)
          if (kickKnown && kickBackOk < 0.6f * kickBackStart) { setFault(F_KICK_JAM, "release: kick stalled before it was back (jammed)", now); return; }
          stopAll(now); seq = RELEASED; kickPos = 0; kickRaw = 0; kickKnown = true; kickPlanted = false; noGroundSeen = false; deployPos = 0; deployKnown = true; landedFresh = false; ev("release: done, legs stowed");
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
    for (int b = 0; b < 5; b++) if (warn & (1 << b)) k += snprintf(wl + k, sizeof wl - k, "%s\"%s\"", k ? "," : "", warnName(b));
    float rg = 0, pg = 0; if (noseKnown() && orient(now) != O_UNKNOWN) rollPitch(rg, pg);   // body roll / pitch from gravity
    return snprintf(buf, n, "{\"fw\":\"%s\",\"t_ms\":%lu,\"phase\":\"%s\",\"fault\":%s%s%s,\"rail_V\":%.2f,\"current_mA\":%.0f,\"peak_mA\":%.0f,\"speed\":%d,"
                    "\"motors\":{\"deploy\":%d,\"kick\":%d,\"drills\":%d,\"foot\":%d},\"seq\":\"%s\",\"armed\":%s,\"auto_on_landing\":%s,\"landed\":%s,"
                    "\"roll\":%.1f,\"pitch\":%.1f,\"accel_g\":%.2f,\"peak_g\":%.2f,\"ina_ok\":%s,\"imu_ok\":%s,\"t_launch_ms\":%s,\"t_land_ms\":%s,\"log_frozen\":%s,"
                    "\"reset\":\"%s\",\"warnings\":[%s],\"video\":%s,\"batt\":\"%s\",\"batt_V\":%.2f,\"orient\":\"%s\",\"tilt_deg\":%.0f,\"roll_g\":%.0f,\"pitch_g\":%.0f,\"soil_mA\":%.0f,\"moving\":%s,\"event\":\"%s\"}",
                    FW_VERSION, (unsigned long)now, phaseName(phase()), fault ? "\"" : "", fault ? faultName(fault) : "null", fault ? "\"" : "",
                    fin(railV >= 0 ? railV : (io.rail_V ? io.rail_V() : -1.f)), fin(lastMa), fin(peakMa), spd, dir[0], dir[1], dir[2], dir[3], seqName(seq),
                    armed ? "true" : "false", autoOnLanding ? "true" : "false", landed ? "true" : "false", fin(roll), fin(pitch), fin(accG), fin(peakG),
                    inaOk ? "true" : "false", imuOk ? "true" : "false", tl, td, log.frozen ? "true" : "false", reset, wl,
                    video ? "true" : "false", battName(batt), fin(battV), orientName(orient(now)), orient(now) == O_UNKNOWN ? 0.f : fin(tiltDeg()), fin(rg), fin(pg), fin(soilMa), moving ? "true" : "false", lastEvent);
  }
};

}  // namespace kedge
