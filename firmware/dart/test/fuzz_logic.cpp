// Deterministic randomized property test (fuzz) of the KEDGE dart logic (kedge_logic.h v4).
//
//   cd firmware/dart/test
//   g++ -std=c++17 -O2 -Wall -Wextra -I../kedge_dart -o fuzz_logic fuzz_logic.cpp
//   ./fuzz_logic [seed] [episodes] [only_episode]
//   ./fuzz_logic --quick [seed]          (1000 episodes, a few seconds: run_tests.sh)
//
// seed defaults to 20261103. Every episode draws from its own RNG stream made from (seed, episode index), so one
// episode can be replayed alone with full event output: ./fuzz_logic <seed> <episode+1> <episode>.
//
// Each episode builds a random plant (battery, read noise and failures, tick jitter, motor currents, auger coupling
// when the worm of a pair pauses, soil end positions), a random command stream (held keys, stop, auto, release, arm,
// speed, link loss, IMU traces with real flights, bench handling, drops and knocks) and, in chaos episodes, injected
// faults (open wire, jams, rail sag, INA219 dead or failing, short circuit, IMU silent). About a third of the episodes
// are nominal missions (no injected faults, nominal currents) that must anchor and release.
//
// Invariants checked on every tick and after every command:
//   1 hardware off while dir is 0; hardware equals dir except during the worm's 150 ms pair-check pause
//   2 run limits: manual hold 8 s (2 s without INA219), kick 15.5 s, pods 6 s, whole sequence 35 s
//   3 a motor driven into a hard stop (stalled) is stopped while the INA219 works, as API.md guarantees: within 3.0 s in a
//     sequence; within 2.5 s in manual when exactly one or two motors ran without change since the stall at speed 150+
//     (other manual stalls only have the 8 s hold cap of invariant 2 and are reported as information)
//   4 after stop() every motor is off at once
//   5 no motor runs while a fault is latched
//   6 no motor runs in FLIGHT
//   7 phase() is one of the 8 names and consistent with seq, armed, flight and fault (API.md table)
//   8 every status() and LogJson output is valid JSON (own validator below)
//   9 auto starts only after /auto or an impact landing while armed with auto
//  10 nominal episodes: /auto ends anchored with the legs fully out, then /release ends released with legs stowed
#include "kedge_logic.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdarg>
#include <cmath>
#include <chrono>
#include <deque>
#include <map>
#include <queue>
#include <string>
#include <vector>
using namespace kedge;

// ------------------------------------------------------------------------------------------------------- RNG
struct Rng {
  uint64_t s = 1;
  uint64_t next() { uint64_t z = (s += 0x9E3779B97F4A7C15ull); z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull; z = (z ^ (z >> 27)) * 0x94D049BB133111EBull; return z ^ (z >> 31); }
  double u() { return (double)(next() >> 11) * (1.0 / 9007199254740992.0); }
  double r(double a, double b) { return a + (b - a) * u(); }
  int i(int a, int b) { return a + (int)(next() % (uint64_t)(b - a + 1)); }   // inclusive
  bool p(double x) { return u() < x; }
};

// ------------------------------------------------------------------------------------------------------- plant
// Positions are ms of travel at 7.6 V and full duty. Kick: 0 stowed, ground contact at kickContact, housing on the
// ground at kickLand (hard stop). Pods: 0 stowed, deployEnd down (hard stops at both ends). Drills reach the soil when
// the pods pass drillTouch and stall at drillFull (full depth or stone). The foot digs once the kick touches the ground
// and stalls at footFull. Currents are mA at 8 V and full duty. DRV8833 slow decay: a stalled motor's supply current
// goes as duty^2, a turning motor's (free, pushing, digging: torque-limited) as duty; both scale with V/8.
struct Plant {
  float vbat = 7.6f, rint = 0.1f, noise = 0; double readFail = 0; int maxFailRun = 1000, failRun = 0;   // nominal: isolated failures only
  float kFree = 30, kPush = 80, kStall = 180, dFree = 30, dStall = 180;
  float drFree = 70, drDig = 400, drStall = 700, fFree = 40, fDig = 130, fStall = 550;
  float cDrill = 0, cFoot = 0;                         // auger current change while the worm of its pair is off
  float rate[NMOT] = {1, 1, 1, 1};
  double kickContact = 3000, kickLand = 4300, deployEnd = 1400, drillTouch = 900, drillFull = 3000, footFull = 3000;
  // injected faults
  int openWire = -1; bool noGround = false, drillJam = false, footJam = false, inaDead = false, shortOn = false, kickStrip = false;   // strip: gear slips
  double kickJamFwd = -1, kickJamBack = -1, deployJamFwd = -1; float sagV = 0; double burstFail = 0;
  // state
  int hwDir[NMOT] = {0, 0, 0, 0}, hwDuty[NMOT] = {0, 0, 0, 0};
  double kick = 0, deploy = 0, drillDepth = 0, footDepth = 0;
  float railNow = 7.6f, railCache = -1, total = 0;

  bool hwOn(int m) const { return hwDir[m] != 0 && hwDuty[m] > 0; }
  bool stalledDir(int m, int d) const {
    if (m == KICK && kickStrip) return false;          // a stripped gear spins free and never reaches a stop
    switch (m) {
      case KICK: return d > 0 ? (kick >= kickLand || (kickJamFwd >= 0 && kick >= kickJamFwd)) : d < 0 ? (kick <= 0 || (kickJamBack >= 0 && kick <= kickJamBack)) : false;
      case DEPLOY: return d > 0 ? (deploy >= deployEnd || (deployJamFwd >= 0 && deploy >= deployJamFwd)) : d < 0 ? deploy <= 0 : false;
      case DRILLS: return d > 0 ? (deploy >= drillTouch && drillDepth >= drillFull) : d < 0 ? drillJam : false;
      case FOOT: return d > 0 ? (kick >= kickContact && footDepth >= footFull) : d < 0 ? footJam : false;
    }
    return false;
  }
  float baseMa(int m) const {
    if (!hwOn(m) || m == openWire) return 0;
    int d = hwDir[m];
    if (stalledDir(m, d)) return m == KICK ? kStall : m == DEPLOY ? dStall : m == DRILLS ? drStall : fStall;
    if (m == KICK && kickStrip) return kFree;
    switch (m) {
      case KICK: return (d > 0 && kick >= kickContact) ? kPush : kFree;
      case DEPLOY: return dFree;
      case DRILLS: {
        bool soil = deploy >= drillTouch;
        if (d > 0 && soil) {
          float c = drFree + (drDig - drFree) * (float)fmin(1.0, 0.35 + drillDepth / 400.0);
          if (!hwOn(DEPLOY) && deploy < deployEnd) c *= 1 + cDrill;      // the pods stop pushing: cut load changes
          return c;
        }
        if (d < 0 && soil && drillDepth > 0) return 0.6f * drDig;
        return drFree;
      }
      case FOOT: {
        bool soil = kick >= kickContact;
        if (d > 0 && soil) {
          float c = fFree + (fDig - fFree) * (float)fmin(1.0, 0.35 + footDepth / 400.0);
          if (!hwOn(KICK) && kick < kickLand) c *= 1 + cFoot;           // the kick stops pressing the foot down
          return c;
        }
        if (d < 0 && soil && footDepth > 0) return 0.6f * fDig;
        return fFree;
      }
    }
    return 0;
  }
  void electrical() {
    double k = 0;                                      // mA per volt of rail
    for (int m = 0; m < NMOT; m++) {
      if (!hwOn(m)) continue;
      double du = hwDuty[m] / 255.0; bool st = m != openWire && stalledDir(m, hwDir[m]);
      k += baseMa(m) * (st ? du * du : du) / 8.0;
    }
    float vo = sagV > 0 ? sagV : vbat;
    railNow = (float)(vo / (1.0 + k * rint / 1000.0)); total = (float)(k * railNow);
    if (shortOn) { total = 3000; railNow = vo - 3.0f * rint; }
  }
  void step(double dt) {
    if (dt <= 0) return;
    electrical();
    for (int m = 0; m < NMOT; m++) {
      if (!hwOn(m) || m == openWire) continue;
      int d = hwDir[m]; if (stalledDir(m, d) || (m == KICK && kickStrip)) continue;
      double v = rate[m] * (hwDuty[m] / 255.0) * (railNow / 7.6) * dt;
      switch (m) {
        case KICK:
          if (d > 0) { double lim = kickLand; if (kickJamFwd >= 0 && kickJamFwd < lim) lim = kickJamFwd; if (kick < lim) kick = fmin(kick + v, lim); }
          else { double lo = kickJamBack >= 0 ? kickJamBack : 0; if (kick > lo) kick = fmax(kick - v, lo); }
          if (kick < kickContact) footDepth = 0;       // foot lifted off the ground
          break;
        case DEPLOY:
          if (d > 0) { double lim = deployEnd; if (deployJamFwd >= 0 && deployJamFwd < lim) lim = deployJamFwd; if (deploy < lim) deploy = fmin(deploy + v, lim); }
          else if (deploy > 0) deploy = fmax(deploy - v, 0.0);
          if (deploy < drillTouch) drillDepth = 0;     // augers lifted out of the soil
          break;
        case DRILLS:
          if (deploy >= drillTouch) drillDepth = d > 0 ? fmin(drillDepth + v, drillFull) : fmax(drillDepth - 1.2 * v, 0.0);
          break;
        case FOOT:
          if (kick >= kickContact) footDepth = d > 0 ? fmin(footDepth + v, footFull) : fmax(footDepth - 1.2 * v, 0.0);
          break;
      }
    }
  }
};

// ------------------------------------------------------------------------------------------------------- IMU traces
enum { I_REST, I_REAL, I_SOFT, I_SILFLIGHT, I_BENCH, I_DROP, I_KNOCK, I_SILENT };
static const char* imuName(int k) { static const char* n[] = {"rest", "real flight", "soft landing flight", "flight, IMU silent", "bench handling", "drop", "knock", "IMU silent"}; return n[k]; }
struct ImuProg {
  int kind = I_REST; float t0 = 0, len = 0;
  float push = 30, flight = 800, imp = 8, impDur = 8, settle = 200, bAt = -1, bG = 0, bDur = 0, silFrom = 0, silLen = 0;
  int style = 0; bool tether = false;
};

// ------------------------------------------------------------------------------------------------------- globals
static Kedge K; static Plant P; static Rng R; static ImuProg IMU;
static uint32_t ET = 0, BASE = 0;                      // episode time (ms) and the dart clock at episode start
static inline uint32_t NOWA() { return BASE + ET; }
static uint64_t SEED = 0; static int EPI = 0; static bool VERBOSE = false, NOMINAL_EP = false;
static long TICKS = 0, EP_TICKS = 0, CMDS = 0, EPISODES = 0, NOMINAL = 0; static double SIM_MS = 0;
static long VIOL[12], VIOLN[12], CHECKS[12];   // VIOLN: violations inside nominal missions
struct Stats { long launches = 0, impactLandings = 0, fallbackLandings = 0, autoByCmd = 0, autoByLanding = 0, anchored = 0, released = 0,
               nomAnch = 0, nomRel = 0, stallStops = 0, pairPauses = 0, holdsCapped = 0, faults[F_NFAULT] = {}, maxRun[4] = {};
               long pushArmed = 0, pushMissed = 0, jsonStatus = 0, jsonLog = 0; float pushLongestMissed = 0;
               long manOutside = 0, manOutsideOver = 0; uint32_t manOutsideMax = 0; long knownPods = 0; uint32_t knownPodsMax = 0; long knownLow = 0; } ST;

// event log (ring, repeats folded)
struct Ev { uint32_t t0, t1; std::string s; int rep; };
static std::deque<Ev> EVR;
static void evlog(const char* fmt, ...) {
  char b[240]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
  if (VERBOSE) printf("  %9.3f s  %s\n", ET / 1000.0, b);
  if (!EVR.empty() && EVR.back().s == b) { EVR.back().t1 = ET; EVR.back().rep++; return; }
  EVR.push_back({ET, ET, b, 1}); if (EVR.size() > 40) EVR.pop_front();
}
static void dumpEvents(int n) {
  int start = (int)EVR.size() - n; if (start < 0) start = 0;
  for (int k = start; k < (int)EVR.size(); k++) {
    const Ev& e = EVR[k];
    if (e.rep > 1) printf("      %8.3f-%8.3f s  %s  (x%d)\n", e.t0 / 1000.0, e.t1 / 1000.0, e.s.c_str(), e.rep);
    else printf("      %8.3f s           %s\n", e.t0 / 1000.0, e.s.c_str());
  }
}

static const char* INV_NAME[12] = {"", "hardware matches dir (pause only)", "run time limits", "stall stopped (seq 3.0 s, manual 2.5 s)",
  "stop() turns all off at once", "no motor while a fault is latched", "no motor in FLIGHT", "phase valid and consistent",
  "status / flightlog JSON valid", "auto only after /auto or impact landing", "nominal auto and release complete",
  "no ANCHORED: no ground, kick swung from stowed"};
struct SigInfo { long n, nNom; uint64_t seed; int ep; uint32_t t; std::string msg; };
static std::map<std::string, SigInfo> SIGS; static bool DUMPED[12]; static int ONELINERS = 0;
static void violate(int inv, const std::string& sig, const char* fmt, ...) {
  char msg[600]; va_list ap; va_start(ap, fmt); vsnprintf(msg, sizeof msg, fmt, ap); va_end(ap);
  VIOL[inv]++;
  if (NOMINAL_EP) VIOLN[inv]++;
  char key[300]; snprintf(key, sizeof key, "%02d | %s", inv, sig.c_str());
  auto it = SIGS.find(key);
  if (it != SIGS.end()) { it->second.n++; if (NOMINAL_EP) it->second.nNom++; return; }
  SIGS[key] = {1, NOMINAL_EP ? 1 : 0, SEED, EPI, ET, msg};
  evlog("VIOLATION %d: %s", inv, msg);
  if (!DUMPED[inv] || VERBOSE) {
    DUMPED[inv] = true;
    printf("\nVIOLATION of invariant %d (%s)\n  seed %llu  episode %d  t = %.3f s (dart clock %lu)\n  %s\n  reproduce: ./fuzz_logic %llu %d %d\n  last events:\n",
           inv, INV_NAME[inv], (unsigned long long)SEED, EPI, ET / 1000.0, (unsigned long)NOWA(), msg, (unsigned long long)SEED, EPI + 1, EPI);
    dumpEvents(25);
  } else if (ONELINERS++ < 40) {
    printf("new violation signature [inv %d] %s: seed %llu episode %d t %.3f s: %s\n", inv, sig.c_str(), (unsigned long long)SEED, EPI, ET / 1000.0, msg);
  }
}

// ------------------------------------------------------------------------------------------------------- JSON validator
struct JsonV {
  const char* p; const char* e;
  void ws() { while (p < e && (*p == ' ' || *p == '\n' || *p == '\t' || *p == '\r')) p++; }
  bool lit(const char* s) { size_t n = strlen(s); if ((size_t)(e - p) < n || strncmp(p, s, n) != 0) return false; p += n; return true; }
  bool str() {
    if (p >= e || *p != '"') return false;
    p++;
    while (p < e) {
      unsigned char c = (unsigned char)*p++;
      if (c == '"') return true;
      if (c < 0x20) return false;
      if (c == '\\') {
        if (p >= e) return false;
        char x = *p++;
        if (x == 'u') { for (int k = 0; k < 4; k++) { if (p >= e || !isxdigit((unsigned char)*p)) return false; p++; } }
        else if (!(x == '"' || x == '\\' || x == '/' || x == 'b' || x == 'f' || x == 'n' || x == 'r' || x == 't')) return false;
      }
    }
    return false;
  }
  bool digits() { if (p >= e || !isdigit((unsigned char)*p)) return false; while (p < e && isdigit((unsigned char)*p)) p++; return true; }
  bool num() {
    if (p < e && *p == '-') p++;
    if (p < e && *p == '0') p++; else if (!digits()) return false;
    if (p < e && *p == '.') { p++; if (!digits()) return false; }
    if (p < e && (*p == 'e' || *p == 'E')) { p++; if (p < e && (*p == '+' || *p == '-')) p++; if (!digits()) return false; }
    return true;
  }
  bool val(int depth) {
    ws(); if (p >= e || depth > 64) return false;
    char c = *p;
    if (c == '{') {
      p++; ws(); if (p < e && *p == '}') { p++; return true; }
      for (;;) {
        ws(); if (!str()) return false; ws();
        if (p >= e || *p != ':') return false;
        p++;
        if (!val(depth + 1)) return false;
        ws();
        if (p < e && *p == ',') { p++; continue; }
        if (p < e && *p == '}') { p++; return true; }
        return false;
      }
    }
    if (c == '[') {
      p++; ws(); if (p < e && *p == ']') { p++; return true; }
      for (;;) {
        if (!val(depth + 1)) return false;
        ws();
        if (p < e && *p == ',') { p++; continue; }
        if (p < e && *p == ']') { p++; return true; }
        return false;
      }
    }
    if (c == '"') return str();
    if (c == 't') return lit("true");
    if (c == 'f') return lit("false");
    if (c == 'n') return lit("null");
    return num();
  }
};
static bool validJson(const char* s, size_t n) { JsonV j{s, s + n}; if (!j.val(0)) return false; j.ws(); return j.p == j.e; }

// ------------------------------------------------------------------------------------------------------- io callbacks
static void setM(int m, int d, int duty) { if (m >= 0 && m < NMOT) { P.hwDir[m] = d; P.hwDuty[m] = duty; } }
static float readCurrent() {
  P.electrical();
  bool fail = R.p(P.readFail + P.burstFail) && P.failRun < P.maxFailRun;
  if (P.inaDead || fail) { P.failRun++; P.railCache = -1; return NAN; }
  P.failRun = 0; P.railCache = P.railNow;
  if (P.total > 3200) return 9999;                     // INA219 overflow flag, as the sketch reports it
  return P.total + (float)R.r(-P.noise, P.noise);
}
static float readRail() { return P.railCache; }
static void logCb(const char* s) { evlog("log: %s", s); }

// ------------------------------------------------------------------------------------------------------- harness state
struct Track {
  // invariant 2: run segments
  int lastDir[NMOT] = {0, 0, 0, 0}; bool lastSeq[NMOT] = {}, hadSeg[NMOT] = {}, segSeq[NMOT] = {}, segIna[NMOT] = {}, segFlag[NMOT] = {};
  int segDir[NMOT] = {0, 0, 0, 0}; uint32_t segStart[NMOT] = {}, lastOff[NMOT] = {};
  bool seqWas = false, seqAnch = false, seqFlag = false; uint32_t seqT0 = 0;
  // invariant 3: stalls
  bool stallOn[NMOT] = {}, stallGood[NMOT] = {}, stallFlag[NMOT] = {}; int stallDir[NMOT] = {}; uint32_t stallT0[NMOT] = {};
  int stallNmax[NMOT] = {}, stallNmin[NMOT] = {}, stallSpdMin[NMOT] = {}; bool stallSeq[NMOT] = {};
  int stallMask[NMOT] = {}; bool stallMaskChg[NMOT] = {}, stallInfo[NMOT] = {}, knownFlag[NMOT] = {};   // set of running motors at the stall
  // invariant 1: pause
  bool pauseOn = false; int pauseM = -1; uint32_t pauseT0 = 0;
  bool act[11] = {};
  int lastPhase = -1, lastFault = 0, lastSeqV = IDLE; uint32_t seqChangeT = 0; bool swingFromStow = false;
  float maxAccFlight = 0;
  bool linkLost = false, bno = true; uint32_t lastImu = 0;
  int pendingPushCheck = 0; float pushCheckAt = 0, pushLen = 0, pushT0 = 0;   // launch detection info check
  bool lastArmed = false; uint32_t armChangeT = 0;
};
static Track T;

struct Hold { char c; uint32_t next, end, period, ms; };
static std::vector<Hold> HOLDS;
struct Act { uint32_t t; int kind; int a; bool clear; long seq; };
struct ActCmp { bool operator()(const Act& x, const Act& y) const { return x.t != y.t ? x.t > y.t : x.seq > y.seq; } };
static std::priority_queue<Act, std::vector<Act>, ActCmp> ACTS; static long ACTSEQ = 0;

// ------------------------------------------------------------------------------------------------------- invariant checks
static int apiPhase() {                                // API.md "Phases" table, first match wins
  if (K.fault) return P_FAULT;
  if (K.seq >= R_DRILL && K.seq <= R_KICK) return P_RELEASING;
  if (K.seq >= K_SWING && K.seq <= D_AFTER) return P_ANCHORING;
  if (K.flight.flying()) return P_FLIGHT;
  if (K.seq == DONE) return P_ANCHORED;
  bool motor = false; for (int m = 0; m < NMOT; m++) if (K.dir[m]) motor = true;
  if (K.landed && K.landedFresh && (K.seq == IDLE || K.seq == ABORTED) && !motor) return P_LANDED;   // a manual motor after a landing shows IDLE
  if (K.armed) return P_ARMED;
  return P_IDLE;
}
static int nRunning() { int n = 0; for (int m = 0; m < NMOT; m++) if (K.dir[m]) n++; return n; }
static std::string modeCtx() {
  if (K.seqRunning()) return std::string("seq '") + seqName(K.seq) + "'";
  char b[80]; snprintf(b, sizeof b, "manual, %d motor(s) on, speed %s", nRunning(), K.spd == 255 ? "255" : K.spd >= 150 ? "150..230" : "50..125");
  return b;
}
static void edge(int inv, bool bad, const std::string& sig, const char* msg) {
  if (bad && !T.act[inv]) violate(inv, sig, "%s", msg);
  T.act[inv] = bad;
}

static void checkAll(bool afterStop) {
  // 4: stop turns everything off at once
  if (afterStop) {
    CHECKS[4]++;
    bool off = true; for (int m = 0; m < NMOT; m++) if (K.dir[m] || P.hwOn(m)) off = false;
    if (!off || K.seqRunning()) violate(4, "stop", "after stop(): dir %d %d %d %d, hardware %d %d %d %d, seq '%s'", K.dir[0], K.dir[1], K.dir[2], K.dir[3],
                                        P.hwOn(0) ? P.hwDir[0] : 0, P.hwOn(1) ? P.hwDir[1] : 0, P.hwOn(2) ? P.hwDir[2] : 0, P.hwOn(3) ? P.hwDir[3] : 0, seqName(K.seq));
  }
  // 1: hardware vs dir
  CHECKS[1]++;
  if (K.paused && K.pz >= 0) { if (!T.pauseOn || T.pauseM != K.pz) { T.pauseOn = true; T.pauseM = K.pz; T.pauseT0 = ET; ST.pairPauses++; } }
  else T.pauseOn = false;
  {
    bool bad = false; char msg[200] = ""; std::string sig;
    for (int m = 0; m < NMOT && !bad; m++) {
      bool on = P.hwOn(m);
      if (!K.dir[m]) { if (on) { bad = true; snprintf(msg, sizeof msg, "%s hardware on (dir %d duty %d) while logic dir is 0, %s", motorName(m), P.hwDir[m], P.hwDuty[m], modeCtx().c_str()); sig = std::string(motorName(m)) + " on with dir 0"; } }
      else if (!on || P.hwDir[m] != K.dir[m]) {
        bool ok = T.pauseOn && T.pauseM == m && m != DRILLS && !on && ET - T.pauseT0 <= 158;
        if (!ok) { bad = true; snprintf(msg, sizeof msg, "%s logic dir %d but hardware dir %d duty %d (paused %d pz %d, pause age %u ms), %s", motorName(m), K.dir[m], P.hwDir[m], P.hwDuty[m],
                                         K.paused, K.pz, T.pauseOn ? ET - T.pauseT0 : 0, modeCtx().c_str()); sig = std::string(motorName(m)) + " hardware differs from dir"; }
      }
    }
    edge(1, bad, sig, msg);
  }
  bool anyOn = false; for (int m = 0; m < NMOT; m++) if (K.dir[m] || P.hwOn(m)) anyOn = true;
  // 5: fault latched
  if (K.fault) { CHECKS[5]++; edge(5, anyOn, std::string("fault ") + faultName(K.fault), (std::string("motor running while fault ") + faultName(K.fault) + " is latched").c_str()); }
  else T.act[5] = false;
  if (K.fault && K.fault != T.lastFault) ST.faults[K.fault]++;
  T.lastFault = K.fault;
  // 6: flight
  if (K.flight.flying()) { CHECKS[6]++; edge(6, anyOn || K.seqRunning(), "flight", (std::string("motor running or sequence active in FLIGHT, seq '") + seqName(K.seq) + "'").c_str()); }
  else T.act[6] = false;
  // 7: phase
  {
    CHECKS[7]++;
    int ph = K.phase(); char msg[200] = ""; bool bad = false; std::string sig;
    static const char* names[] = {"IDLE", "ARMED", "FLIGHT", "LANDED", "ANCHORING", "ANCHORED", "RELEASING", "FAULT"};
    bool named = false; for (int k = 0; k < 8; k++) if (ph >= 0 && ph <= P_FAULT && strcmp(phaseName(ph), names[k]) == 0) named = true;
    if (!named) { bad = true; sig = "phase name"; snprintf(msg, sizeof msg, "phase() = %d is not one of the 8 names", ph); }
    else if (ph != apiPhase()) { bad = true; sig = std::string("phase ") + phaseName(ph) + " expected " + phaseName(apiPhase()); snprintf(msg, sizeof msg, "phase %s, API table gives %s (seq '%s' armed %d flying %d landed %d fault %d)", phaseName(ph), phaseName(apiPhase()), seqName(K.seq), K.armed, K.flight.flying(), K.landed, K.fault); }
    else if (K.seqRunning() && K.armed) { bad = true; sig = "armed during sequence"; snprintf(msg, sizeof msg, "armed is true while seq '%s' runs", seqName(K.seq)); }
    else if (K.landed != (K.flight.st == Flight::LANDED)) { bad = true; sig = "landed flag vs landing record"; snprintf(msg, sizeof msg, "landed %d but flight state %d (t_land_ms would disagree)", K.landed, K.flight.st); }
    edge(7, bad, sig, msg);
    if (ph != T.lastPhase) { evlog("phase %s (seq '%s')", phaseName(ph), seqName(K.seq)); T.lastPhase = ph; }
  }
  if (K.armed != T.lastArmed) { T.lastArmed = K.armed; if (K.armed) T.armChangeT = ET; }
  if (K.seq != T.lastSeqV && K.seq == K_SWING) T.swingFromStow = P.kick < 100;   // the firmware sees the whole swing
  if (K.seq != T.lastSeqV && K.seq == DONE && T.swingFromStow) { CHECKS[11]++; if (P.noGround) violate(11, "anchored with no ground under the kick", "seq anchored with no ground (kick end stop at %.0f, kick at %.0f), last event '%s'", P.kickLand, P.kick, K.lastEvent); }
  if (K.seq != T.lastSeqV) { if (K.seq == DONE) ST.anchored++; if (K.seq == RELEASED) ST.released++; T.lastSeqV = K.seq; T.seqChangeT = ET; }
  // 2: run limits
  {
    bool sq = K.seqRunning(), anch = K.anchoring();
    if (sq && (!T.seqWas || anch != T.seqAnch)) { T.seqT0 = ET; T.seqFlag = false; }
    T.seqWas = sq; T.seqAnch = anch;
    CHECKS[2]++;
    if (sq && !T.seqFlag && ET - T.seqT0 > 35010) { T.seqFlag = true; violate(2, std::string("whole sequence ") + (anch ? "anchor" : "release"), "sequence running for %u ms (limit 35 s), seq '%s'", ET - T.seqT0, seqName(K.seq)); }
    for (int m = 0; m < NMOT; m++) {
      int d = K.dir[m];
      if (d) {
        bool start;
        if (T.lastDir[m] == 0) start = !(T.hadSeg[m] && !sq && !T.segSeq[m] && T.segDir[m] == d && ET - T.lastOff[m] < 240);   // a key repeat after a short gap continues the hold
        else start = T.lastDir[m] != d || T.lastSeq[m] != sq;
        if (start) { T.segDir[m] = d; T.segSeq[m] = sq; T.segStart[m] = ET; T.segIna[m] = false; T.segFlag[m] = false; T.hadSeg[m] = true; }
        if (K.inaOk) T.segIna[m] = true;
        uint32_t lim = sq ? (m == KICK ? 15500u : m == DEPLOY ? 6000u : 35000u) : (T.segIna[m] ? 8000u : 2000u);
        uint32_t run = ET - T.segStart[m];
        if (run > ST.maxRun[m]) ST.maxRun[m] = run;
        if (!T.segFlag[m] && run > lim + 15) {   // one jittered tick (up to 7 ms) plus the sample boundary
          T.segFlag[m] = true;
          violate(2, std::string(motorName(m)) + (sq ? " in sequence" : T.segIna[m] ? " manual hold" : " manual hold without INA219"),
                  "%s has run %u ms (limit %u ms), %s", motorName(m), run, lim, modeCtx().c_str());
          if (getenv("FUZZ_DEBUG")) printf("DEBUG heldStart %u segStart %u until %u now %u ET %u\n", K.heldStart[m], T.segStart[m], K.until[m], ET, ET);
        }
      } else if (T.lastDir[m]) T.lastOff[m] = ET;
      T.lastDir[m] = d; T.lastSeq[m] = sq;
    }
  }
  // 3: stalled motor stopped while the INA219 works, as documented (API.md): within 3.0 s in a sequence; within 2.5 s
  // in manual when exactly one or two motors have run without change since the stall, at speed 150 or more. Other
  // manual stalls (3 or 4 motors, a changing set of motors, speed 125 or less) only have the 8 s hold cap (inv 2):
  // they are counted as information.
  auto endStall = [](int m) {
    if (T.stallOn[m] && !T.stallSeq[m] && (T.stallMaskChg[m] || T.stallNmax[m] > 2 || T.stallSpdMin[m] < 150)) ST.manOutside++;
    if (T.stallOn[m] && T.stallInfo[m]) { uint32_t dur = ET - T.stallT0[m]; if (dur > ST.manOutsideMax) ST.manOutsideMax = dur; }
    T.stallInfo[m] = false;
  };
  int runMask = 0; for (int k = 0; k < NMOT; k++) if (K.dir[k]) runMask |= 1 << k;
  for (int m = 0; m < NMOT; m++) {
    int d = K.dir[m];
    bool st = d && m != P.openWire && P.stalledDir(m, d);
    if (st) {
      if (!T.stallOn[m] || T.stallDir[m] != d || T.stallSeq[m] != K.seqRunning()) {   // a sequence taking over is a new run
        endStall(m);
        T.stallOn[m] = true; T.stallDir[m] = d; T.stallT0[m] = ET; T.stallGood[m] = true; T.stallFlag[m] = false; CHECKS[3]++;
        T.stallSeq[m] = K.seqRunning(); T.stallNmax[m] = 0; T.stallNmin[m] = 9; T.stallSpdMin[m] = 255;
        T.stallMask[m] = runMask; T.stallMaskChg[m] = false; T.knownFlag[m] = false;
      }
      { int n = nRunning(); if (n > T.stallNmax[m]) T.stallNmax[m] = n; if (n < T.stallNmin[m]) T.stallNmin[m] = n; if (K.spd < T.stallSpdMin[m]) T.stallSpdMin[m] = K.spd; }
      if (runMask != T.stallMask[m]) T.stallMaskChg[m] = true;
      if (!K.inaOk || P.inaDead || P.burstFail > 0 || P.shortOn) T.stallGood[m] = false;
      bool sq = T.stallSeq[m];
      bool covered = sq || (!T.stallMaskChg[m] && T.stallNmax[m] <= 2 && T.stallSpdMin[m] >= 150);
      uint32_t limit = sq ? 3000 : 2500;
      // documented known limit (API.md): pods at a stop or jammed while the drills' current rises during each pause by
      // more than the margin are only ended by the pods time limit. Still bounded: deployMaxMs + 0.5 s
      bool knownPods = m == DEPLOY && (K.seq == D_DEPLOY || K.seq == R_PODS) && P.cDrill > 0.05f;
      if (knownPods) limit = K.tu.deployMaxMs + 500;
      // documented tuning rule (BENCH_TEST step 10): the stall line is 80 % of the measured stall. A plant whose stall is
      // within 10 % of the line is outside it: coupling and noise can hide the stall. Still bounded by the step limit
      float plantStall = m == KICK ? P.kStall : m == DEPLOY ? P.dStall : m == DRILLS ? P.drStall : P.fStall;
      bool knownLow = sq && plantStall < 1.10f * K.tu.stallMa[m];
      if (knownLow && !knownPods) limit = std::max(K.tu.kickPlantMaxMs, K.tu.deployMaxMs) + 500;
      if (knownLow && !knownPods && ET - T.stallT0[m] > 3000 && !T.knownFlag[m]) { T.knownFlag[m] = true; ST.knownLow++; }
      if (knownPods && ET - T.stallT0[m] > 3000 && !T.knownFlag[m]) { T.knownFlag[m] = true; ST.knownPods++; }
      if (knownPods && ET - T.stallT0[m] > ST.knownPodsMax) ST.knownPodsMax = ET - T.stallT0[m];
      if (!T.stallFlag[m] && T.stallGood[m] && !covered && ET - T.stallT0[m] > 2500) {
        T.stallFlag[m] = true; T.stallInfo[m] = true; ST.manOutsideOver++;
        evlog("info: manual stall outside the documented guarantee still driven after 2.5 s (%s, %d to %d motors, lowest speed %d)", motorName(m), T.stallNmin[m], T.stallNmax[m], T.stallSpdMin[m]);
      }
      if (!T.stallFlag[m] && T.stallGood[m] && covered && ET - T.stallT0[m] > limit) {
        T.stallFlag[m] = true;
        const char* where = m == KICK ? (d > 0 ? (P.kick >= P.kickLand ? "housing on the ground" : "jammed going out") : (P.kick <= 0 ? "stowed stop" : "jammed going back"))
                          : m == DEPLOY ? (d > 0 ? (P.deploy >= P.deployEnd ? "pods-down stop" : "jammed going down") : "pods-up stop")
                          : m == DRILLS ? (d > 0 ? "full depth / stone" : "jammed unscrewing") : (d > 0 ? "full depth / stone" : "jammed unscrewing");
        // relative to the step now running: was the worm already stalled when it began, or how soon after
        int32_t into = (int32_t)(T.stallT0[m] - T.seqChangeT);
        const char* when = !K.seqRunning() ? "" : into <= 10 ? " (at the stop when the step began)" : into <= 420 ? " (reached the stop in the first 420 ms of the step)" : " (reached the stop later in the step)";
        std::string ctx;
        if (K.seqRunning()) ctx = modeCtx() + when;
        else {
          char b[120]; int sp = T.stallSpdMin[m];
          snprintf(b, sizeof b, "manual, %s motor(s) on during the stall, lowest speed %s", T.stallNmin[m] == T.stallNmax[m] ? std::to_string(T.stallNmax[m]).c_str() : (std::to_string(T.stallNmin[m]) + " to " + std::to_string(T.stallNmax[m]) + " (keys pulsing)").c_str(),
                   sp == 255 ? "255" : sp >= 150 ? "150..230" : "50..125");
          ctx = b;
        }
        violate(3, std::string(motorName(m)) + " at " + where + ", " + ctx, "%s (dir %d) at %s still driven %u ms after it stalled, %s%s, V %.2f, dir %d %d %d %d, coupling drills %+.0f%% foot %+.0f%%",
                motorName(m), d, where, ET - T.stallT0[m], modeCtx().c_str(), when, P.railNow, K.dir[0], K.dir[1], K.dir[2], K.dir[3], P.cDrill * 100, P.cFoot * 100);
      }
    } else {
      if (T.stallOn[m] && !d) ST.stallStops++;
      endStall(m);
      T.stallOn[m] = false;
    }
  }
}

// invariant 9: wrap every call that could start the anchor sequence
struct AutoWatch {
  bool anch, armed, autoOn, flying;
  void begin() { anch = K.anchoring(); armed = K.armed; autoOn = K.autoOnLanding; flying = K.flight.flying(); }
  void end(bool autoCmdOk) {
    if (!flying && K.flight.st == Flight::LANDED && strcmp(K.flight.how, "stop") != 0) {}   // no landing in this call
    if (flying && K.flight.st == Flight::LANDED) { if (K.flight.fallback()) ST.fallbackLandings++; else if (!strcmp(K.flight.how, "impact")) ST.impactLandings++; }
    if (anch || !K.anchoring()) return;
    CHECKS[9]++;
    bool landing = armed && autoOn && flying && K.flight.st == Flight::LANDED && strcmp(K.flight.how, "impact") == 0 && T.maxAccFlight > K.flight.impactG;
    if (autoCmdOk) ST.autoByCmd++;
    else if (landing) ST.autoByLanding++;
    else violate(9, "auto without /auto or impact landing", "anchor sequence started (seq '%s') without /auto: armed %d auto %d flying %d landing '%s' peak after 400 ms %.2f g",
                 seqName(K.seq), armed, autoOn, flying, K.flight.how, T.maxAccFlight);
  }
};

// invariant 8: status JSON (plus field consistency for 7)
static void checkStatus() {
  static const char* resets[] = {"poweron", "software", "brownout", "panic", "task_wdt", "int_wdt", "wdt", "reset_pin", "deepsleep", "other"};
  float roll = R.p(0.05) ? NAN : (float)R.r(-180, 180), pitch = R.p(0.05) ? INFINITY : (float)R.r(-90, 90);
  char b[1024]; int n = K.status(b, sizeof b, roll, pitch, NOWA(), P.railCache, resets[R.i(0, 9)]);
  CHECKS[8]++; ST.jsonStatus++;
  if (n < 0 || n >= (int)sizeof b || !validJson(b, strlen(b))) { violate(8, "status JSON", "status() not valid JSON (n = %d): %.400s", n, b); return; }
  char w[120]; bool ok = true; std::string what;
  snprintf(w, sizeof w, "\"phase\":\"%s\"", phaseName(K.phase())); if (!strstr(b, w)) { ok = false; what = w; }
  snprintf(w, sizeof w, "\"seq\":\"%s\"", seqName(K.seq)); if (!strstr(b, w)) { ok = false; what = w; }
  snprintf(w, sizeof w, "\"armed\":%s", K.armed ? "true" : "false"); if (!strstr(b, w)) { ok = false; what = w; }
  bool tlandNull = strstr(b, "\"t_land_ms\":null") != nullptr, landedT = strstr(b, "\"landed\":true") != nullptr;
  if (tlandNull == landedT) { ok = false; what = "landed vs t_land_ms"; }
  if ((strstr(b, "\"fault\":null") != nullptr) != (K.fault == 0)) { ok = false; what = "fault field"; }
  CHECKS[7]++;
  if (!ok) violate(7, "status fields: " + what, "status JSON disagrees with the state (%s): %.300s", what.c_str(), b);
}
// invariant 8: flight log JSON in chunks of a random size
static void checkLogJson() {
  static const int caps[] = {256, 300, 512, 1000, 4096};
  int cap = caps[R.i(0, 4)];
  LogJson w; w.begin(&K.log, phaseName(K.phase()), K.flight.launched, K.flight.tLaunch, K.flight.st == Flight::LANDED, K.flight.tLand, K.flight.how);
  std::string out; std::vector<char> buf(cap); bool over = false; int n, guard = 0;
  while ((n = w.chunk(buf.data(), cap)) > 0 && guard++ < 100000) { if (n >= cap) { over = true; n = cap - 1; } out.append(buf.data(), n); }
  CHECKS[8]++; ST.jsonLog++;
  const char* why = nullptr;
  if (over) why = "a chunk overflowed its buffer";
  else if (!validJson(out.data(), out.size())) why = "not valid JSON";
  else {
    size_t np = out.find("\"n\":"), sp = out.find("\"samples\":[");
    if (np == std::string::npos || sp == std::string::npos) why = "missing n or samples";
    else {
      long nn = strtol(out.c_str() + np + 4, nullptr, 10), cnt = 0; uint32_t lastT = 0; bool order = true, width = true, hasLaunch = false;
      for (size_t k = sp + 11; k < out.size(); k++) if (out[k] == '[') {
        cnt++; char* q = &out[k + 1]; uint32_t t = (uint32_t)strtoul(q, &q, 10); int f = 1;
        while (*q == ',') { strtod(q + 1, &q); f++; }
        if (f != 8) width = false;
        if (cnt > 1 && (int32_t)(t - lastT) <= 0) order = false;   // millis() order, wrap-aware
        if (K.flight.launched && abs((int32_t)(t - K.flight.tLaunch)) <= 60) hasLaunch = true;
        lastT = t;
      }
      bool frozenFlight = K.log.frozen && K.flight.launched && cnt > 0;
      if (cnt != nn) why = "n does not match the sample count";
      else if (frozenFlight && !hasLaunch) why = "frozen flight log no longer holds the launch"; else if (!width) why = "a sample does not have 8 fields"; else if (!order) why = "samples not in time order";
    }
  }
  if (why) violate(8, std::string("flightlog: ") + why, "LogJson (chunk cap %d, %zu bytes): %s: %.200s", cap, out.size(), why, out.c_str());
}

// ------------------------------------------------------------------------------------------------------- commands
static const char* cmdKey(char c, uint32_t ms, const char* label) {
  CMDS++; AutoWatch w; w.begin();
  const char* r = K.key(c, ms, NOWA());
  w.end(c == 'a' && r == nullptr);
  evlog("%s -> %s", label, r ? r : "ok");
  checkAll(c == ' ' || c == 's' || c == 'S');
  return r;
}
static const char* cmdArm(bool on, bool withAuto) {
  CMDS++; AutoWatch w; w.begin();
  const char* r = K.arm(on, withAuto, NOWA());
  w.end(false);
  evlog("/arm?on=%d&auto=%d -> %s", on, withAuto, r ? r : "ok");
  checkAll(false);
  return r;
}
static void cmdStop(bool disarm, const char* label) {
  CMDS++; K.stop(NOWA(), disarm); evlog("%s -> ok", label); checkAll(true);
}

static void startImu(int kind, uint32_t t0, bool nominalPush) {
  ImuProg g; g.kind = kind; g.t0 = (float)t0;
  switch (kind) {
    case I_REAL: case I_SOFT: case I_SILFLIGHT:
      g.push = nominalPush ? (float)R.r(30, 45) : (float)R.r(15, 45);
      g.flight = (float)R.r(600, 1500); g.style = R.p(0.25) ? 1 : 0; g.tether = g.flight > 900 && R.p(0.3);
      g.imp = (float)R.r(5, 8); g.impDur = (float)R.r(3, 15); g.settle = (float)R.r(100, 300);
      if (R.p(0.4)) { g.bAt = (float)R.r(50, 250); g.bG = (float)R.r(4, 7); g.bDur = (float)R.r(3, 8); }
      g.len = g.push + g.flight + fmaxf(g.settle, g.bAt + g.bDur) + 3000;
      if (kind == I_SILFLIGHT) { g.silFrom = g.push + (float)R.r(50, g.flight); g.silLen = (float)R.r(16000, 20000); g.len = g.silFrom + g.silLen + 1000; }
      break;
    case I_BENCH: g.len = (float)R.r(1000, 10000); break;
    case I_DROP: g.flight = (float)R.r(100, 450); g.imp = (float)R.r(6, 8); g.impDur = (float)R.r(5, 15); g.settle = (float)R.r(100, 300);
      if (R.p(0.5)) { g.bAt = (float)R.r(50, 200); g.bG = (float)R.r(4, 7); g.bDur = (float)R.r(3, 8); }
      g.len = g.flight + 2500; break;
    case I_KNOCK: g.push = (float)R.r(5, 40); g.len = g.push + 500; break;
    case I_SILENT: g.len = (float)R.r(500, 5000); break;
  }
  IMU = g; T.pendingPushCheck = 0;                     // a replaced trace never produced its push
  evlog("IMU: %s starts at %.3f s (push %.0f ms, flight %.0f ms, impact %.1f g)", imuName(kind), t0 / 1000.0, g.push, g.flight, g.imp);
  if ((kind == I_REAL || kind == I_SOFT || kind == I_SILFLIGHT) && K.armed && K.flight.st == Flight::WAIT && T.bno) { T.pendingPushCheck = 1; T.pushCheckAt = g.t0 + g.push + 60; T.pushLen = g.push; T.pushT0 = g.t0; }
}
static float restA() { return 1.0f + (float)R.r(-0.02, 0.02); }
static float imuAcc(float t) {
  const ImuProg& g = IMU; float d = t - g.t0;
  if (g.kind == I_REST || d < 0 || d >= g.len) return restA();
  switch (g.kind) {
    case I_REAL: case I_SOFT: case I_SILFLIGHT: {
      if (d < g.push) return 8.0f;                     // spring push, BNO085 saturated
      float f = d - g.push;
      if (f < g.flight) {
        if (g.tether && f >= g.flight * 0.5f && f < g.flight * 0.5f + 10) return 7.5f;   // tether jerk mid flight
        return g.style == 0 ? (float)R.r(0.1, 0.5) : (float)R.r(1.2, 1.7);              // ballistic or tumbling
      }
      if (g.kind == I_SOFT) return restA();            // grass: no impact spike
      float h = f - g.flight;
      if (h < g.impDur) return g.imp;
      if (g.bAt > 0 && h >= g.bAt && h < g.bAt + g.bDur) return g.bG;
      if (h < g.settle) return (float)R.r(1.3, 3.0);
      return restA();
    }
    case I_BENCH: { int di = (int)d; if (di % 530 < 6) return (float)R.r(2.5, 4.5); return 1.0f + 0.25f * sinf(d * 0.004f) + (float)R.r(-0.1, 0.1); }
    case I_DROP: {
      if (d < g.flight) return (float)R.r(0.02, 0.1);
      float h = d - g.flight;
      if (h < g.impDur) return g.imp;
      if (g.bAt > 0 && h >= g.bAt && h < g.bAt + g.bDur) return g.bG;
      if (h < g.settle) return (float)R.r(1.3, 2.5);
      return restA();
    }
    case I_KNOCK: return d < g.push ? 8.0f : restA();
  }
  return restA();
}
static bool imuSilent(float t) {
  float d = t - IMU.t0;
  if (IMU.kind == I_SILENT) return d >= 0 && d < IMU.len;
  if (IMU.kind == I_SILFLIGHT) return d >= IMU.silFrom && d < IMU.silFrom + IMU.silLen;
  return false;
}

enum { C_HOLD, C_MULTI, C_SERIAL, C_STOP, C_STOP0, C_SCAN, C_AUTO, C_RELEASE, C_ARMA, C_ARML, C_DISARM, C_TOGGLE, C_SPUP, C_SPDN, C_LINK, C_UNKNOWN, C_STATUS, C_FLOG, C_IMU, C_SINGLE, C_N };
enum { M_NONE, M_CHAOS, M_PRE, M_BENIGN, M_GAP, M_STATUS };
//                          HOLD MULT SER STOP ST0 SCAN AUTO REL ARMA ARML DIS TOG  +   -  LINK UNK STAT FLOG IMU SINGLE
static const int W_CHAOS[C_N]  = {28, 6,  3,  5,  2,  1,   7,  6,  5,   3,   2,  1,  3,  3,  2,   1,  6,   1,   8,  0};
static const int W_PRE[C_N]    = {30, 6,  3,  4,  1,  0,   0,  0,  0,   2,   2,  0,  3,  3,  2,   1,  5,   1,   3,  0};
static const int W_BENIGN[C_N] = {0,  0,  0,  0,  0,  0,   3,  2,  2,   1,   1,  1,  2,  2,  2,   1,  6,   1,   0,  3};
static const int W_GAP[C_N]    = {0,  0,  0,  0,  0,  0,   1,  0,  0,   0,   1,  0,  2,  2,  2,   1,  6,   1,   0,  0};
static const int W_STATUS[C_N] = {0,  0,  0,  0,  0,  0,   0,  0,  0,   0,   0,  0,  0,  0,  0,   0,  1,   0,   0,  0};
static int RAND_MODE = M_NONE;
static uint32_t NEXT_RAND = UINT32_MAX, NEXT_SCRIPT = UINT32_MAX;

static char randRunKey() { static const char k[] = "1!2@3#4$"; return k[R.i(0, 7)]; }
static void addHold(char c, uint32_t period, uint32_t ms, uint32_t len) { HOLDS.push_back({c, ET, ET + len, period, ms}); }

static void doRandomCmd() {
  const int* w = RAND_MODE == M_CHAOS ? W_CHAOS : RAND_MODE == M_PRE ? W_PRE : RAND_MODE == M_BENIGN ? W_BENIGN : RAND_MODE == M_GAP ? W_GAP : W_STATUS;
  int tot = 0; for (int k = 0; k < C_N; k++) tot += w[k];
  int x = R.i(0, tot - 1), c = 0; while (x >= w[c]) { x -= w[c]; c++; }
  if (RAND_MODE == M_BENIGN && !K.seqRunning()) c = C_STATUS;   // the step just ended: the script checks it next
  char lbl[64];
  switch (c) {
    case C_HOLD: { uint32_t len = R.p(0.2) ? (uint32_t)R.i(6000, 12000) : (uint32_t)R.i(100, 4000); addHold(randRunKey(), R.i(100, 700), R.i(50, 500), len); break; }
    case C_MULTI: {                                     // two to four keys held at once
      int n = R.i(2, 4), used = 0; uint32_t per = R.i(100, 700), ms = R.i(50, 500), len = R.i(500, 10000);
      for (int k = 0; k < n; k++) { int m = R.i(0, 3); if (used & (1 << m)) continue; used |= 1 << m; static const char f[] = "1234", b[] = "!@#$"; addHold(R.p(0.5) ? f[m] : b[m], per, ms, len); }
      break;
    }
    case C_SERIAL: { char k = randRunKey(); snprintf(lbl, sizeof lbl, "serial key %c", k); cmdKey(k, K.tu.serialRunMs, lbl); break; }
    case C_STOP: if (R.p(0.6)) HOLDS.clear(); cmdKey(R.p(0.8) ? ' ' : 's', 0, "/stop"); break;
    case C_STOP0: cmdStop(false, "/stop?disarm=0"); break;
    case C_SCAN: cmdStop(true, "key z (I2C scan: full stop)"); break;
    case C_AUTO: cmdKey('a', 0, "/auto"); break;
    case C_RELEASE: cmdKey('r', 0, "/release"); break;
    case C_ARMA: cmdArm(true, true); break;
    case C_ARML: cmdArm(true, false); break;
    case C_DISARM: cmdArm(false, true); break;
    case C_TOGGLE: cmdKey('A', 0, "key A (arm toggle)"); break;
    case C_SPUP: cmdKey('+', 0, "key +"); break;
    case C_SPDN: cmdKey('-', 0, "key -"); break;
    case C_LINK: HOLDS.clear(); T.linkLost = true; CMDS++; evlog("WiFi station left"); break;
    case C_UNKNOWN: cmdKey(R.p(0.5) ? 'x' : 'q', 350, "/key?c=x"); break;
    case C_STATUS: CMDS++; checkStatus(); break;
    case C_FLOG: CMDS++; checkLogJson(); break;
    case C_SINGLE: { char k = randRunKey(); snprintf(lbl, sizeof lbl, "/key?c=%c&ms=350", k); cmdKey(k, 350, lbl); break; }
    case C_IMU: {
      int kind;
      if (RAND_MODE == M_PRE) kind = R.p(0.8) ? I_BENCH : I_REST;
      else { static const int kw[] = {0, 30, 8, 4, 15, 10, 10, 6}; int t = 0; for (int k = 0; k < 8; k++) t += kw[k]; int y = R.i(0, t - 1); kind = 0; while (y >= kw[kind]) { y -= kw[kind]; kind++; } }
      uint32_t t0 = ET + R.i(0, 300);
      if ((kind == I_REAL || kind == I_SOFT || kind == I_SILFLIGHT || kind == I_DROP || kind == I_KNOCK) && R.p(0.65)) { cmdArm(true, R.p(0.6)); t0 = ET + R.i(300, 1500); }
      startImu(kind, t0, false);
      break;
    }
  }
}

// ------------------------------------------------------------------------------------------------------- faults
enum { X_OPEN, X_KJAM, X_KJAMB, X_DJAM, X_DRJAM, X_FJAM, X_SAG, X_SAGMILD, X_INADEAD, X_BURST, X_SHORT, X_STONE, X_IMUDEAD, X_STRIP, X_N };
static void applyAct(const Act& a) {
  char b[120] = "";
  switch (a.kind) {
    case X_OPEN: P.openWire = a.clear ? -1 : a.a; snprintf(b, sizeof b, "open wire on %s", motorName(a.a)); break;
    case X_KJAM: P.kickJamFwd = a.clear ? -1 : R.r(300, P.kickLand - 50); snprintf(b, sizeof b, "kick jammed going out at %.0f", P.kickJamFwd); break;
    case X_KJAMB: P.kickJamBack = a.clear ? -1 : R.r(100, P.kickLand - 200); snprintf(b, sizeof b, "kick jammed going back at %.0f", P.kickJamBack); break;
    case X_DJAM: P.deployJamFwd = a.clear ? -1 : R.r(200, P.deployEnd - 50); snprintf(b, sizeof b, "pods jammed going down at %.0f", P.deployJamFwd); break;
    case X_DRJAM: P.drillJam = !a.clear; snprintf(b, sizeof b, "drills jam when unscrewing"); break;
    case X_FJAM: P.footJam = !a.clear; snprintf(b, sizeof b, "foot jams when unscrewing"); break;
    case X_SAG: P.sagV = a.clear ? 0 : (float)R.r(4.5, 5.5); snprintf(b, sizeof b, "rail sag to %.2f V", P.sagV); break;
    case X_SAGMILD: P.sagV = a.clear ? 0 : (float)R.r(5.65, 6.0); snprintf(b, sizeof b, "mild rail sag to %.2f V", P.sagV); break;
    case X_INADEAD: P.inaDead = !a.clear; snprintf(b, sizeof b, "INA219 dead"); break;
    case X_BURST: P.burstFail = a.clear ? 0 : R.r(0.3, 0.9); snprintf(b, sizeof b, "INA219 read failure burst %.2f", P.burstFail); break;
    case X_SHORT: P.shortOn = !a.clear; snprintf(b, sizeof b, "short circuit (3 A)"); break;
    case X_STONE: P.drillFull = R.r(100, 600); snprintf(b, sizeof b, "stone under the drills at depth %.0f", P.drillFull); break;
    case X_IMUDEAD: T.bno = a.clear; snprintf(b, sizeof b, "BNO085 silent"); break;
    case X_STRIP: P.kickStrip = !a.clear; snprintf(b, sizeof b, "kick gear stripped (spins free, never moves)"); break;
  }
  evlog("inject: %s%s", a.clear ? "cleared: " : "", b);
}
static void scheduleFaults(uint32_t dur) {
  int n = R.p(0.25) ? 0 : R.i(1, 3);
  for (int k = 0; k < n; k++) {
    int kind = R.i(0, X_N - 1); uint32_t t = R.i(0, dur);
    int a = R.i(0, 3);
    ACTS.push({t, kind, a, false, ACTSEQ++});
    uint32_t clr = UINT32_MAX;
    if (kind == X_SAG) clr = t + R.i(100, 3000);
    else if (kind == X_BURST) clr = t + R.i(20, 500);
    else if (kind == X_SHORT) clr = t + R.i(5, 60);
    else if (kind != X_STONE && R.p(0.5)) clr = t + R.i(200, 8000);
    if (clr != UINT32_MAX) ACTS.push({clr, kind, a, true, ACTSEQ++});
  }
  if (R.p(0.04)) { P.inaDead = true; evlog("inject: INA219 missing from boot"); }
}

// ------------------------------------------------------------------------------------------------------- tick
static void doTick(uint32_t dt) {
  TICKS++; EP_TICKS++;
  uint32_t now = NOWA();
  if (T.bno && !imuSilent((float)ET)) {
    int n = dt >= 6 ? 3 : 2;                           // 400 Hz accelerometer, every sample stamped with the tick time
    for (int k = 0; k < n; k++) {
      float a = imuAcc((float)ET - (n - 1 - k) * 2.5f);
      AutoWatch w; w.begin(); int st0 = K.flight.st;
      K.onAccel(a, now);
      if (st0 == Flight::WAIT && K.flight.st == Flight::FLYING) { T.maxAccFlight = 0; ST.launches++; }
      if ((st0 == Flight::FLYING || st0 == Flight::IMPACTED) && (int32_t)(now - K.flight.tLaunch) >= 400 && a > T.maxAccFlight) T.maxAccFlight = a;
      w.end(false);
    }
    T.lastImu = ET;
  }
  bool fresh = T.bno && ET - T.lastImu < 1000;
  if (fresh != K.imuOk) { K.imuOk = fresh; evlog(fresh ? "BNO085 data back" : "BNO085 silent for 1 s"); }
  if (fresh) { float th = ET * 0.0003f, q[4] = {cosf(th), 0, 0, sinf(th)}, v[3] = {0.1f, -0.2f, K.accG * 9.81f}; K.imuSample(now, q, v); }
  if (T.linkLost) { T.linkLost = false; K.linkLost(now); }
  AutoWatch w; w.begin(); K.tick(now); w.end(false);
  checkAll(false);
  if (T.pendingPushCheck && (float)ET >= T.pushCheckAt) {
    T.pendingPushCheck = 0;
    if ((K.armed || K.flight.launched) && (float)T.armChangeT < T.pushT0 - 1 && T.bno && K.imuOk) {   // armed, IMU alive
      bool seen = K.flight.launched; float push = T.pushLen;
      if (!seen) evlog("info: armed launch push of %.0f ms not detected", push);
      ST.pushArmed++;
      if (!seen) { ST.pushMissed++; if (push > ST.pushLongestMissed) ST.pushLongestMissed = push; }
    }
  }
  if (EP_TICKS % 23 == 0) checkStatus();             // per-episode count: a replayed episode draws the same numbers
}

// ------------------------------------------------------------------------------------------------------- nominal mission script
enum { S_PRE, S_QUIET, S_FLY, S_ANCH, S_GAP, S_REL, S_TAIL, S_END };
static int SS = S_END; static uint32_t SS_T0 = 0;
static void toTail() { SS = S_TAIL; RAND_MODE = M_PRE; NEXT_RAND = ET + R.i(100, 1000); NEXT_SCRIPT = ET + R.i(0, 5000); }
static void scriptStep() {
  switch (SS) {
    case S_PRE: HOLDS.clear(); SS = S_QUIET; SS_T0 = ET; RAND_MODE = M_NONE; NEXT_RAND = UINT32_MAX; NEXT_SCRIPT = ET + 2000; break;
    case S_QUIET:
      if ((K.running() || K.flight.flying()) && ET - SS_T0 < 20000) { NEXT_SCRIPT = ET + 500; break; }
      // the legs moved by hand before the shot: the dart refuses to arm ("legs out: release first"), so take the /auto path
      if (R.p(0.5) || (K.kickKnown && K.kickPos > 0.1f * K.tu.kickSwingMs) || (K.deployKnown && K.deployPos > 0.1f * K.tu.podsTravelMs)) {
        if (K.armed) cmdKey(' ', 0, "/stop (disarm before a manual /auto, as a client must)");
        const char* r = cmdKey('a', 0, "/auto");
        if (r) { violate(10, std::string("auto refused: ") + r, "nominal: /auto refused at the start: %s", r); toTail(); break; }
        SS = S_ANCH; SS_T0 = ET; RAND_MODE = M_BENIGN; NEXT_RAND = ET + R.i(200, 1500); NEXT_SCRIPT = ET + 100;
      } else {
        const char* r = cmdArm(true, true);
        if (r) { violate(10, std::string("arm refused: ") + r, "nominal: /arm refused: %s", r); toTail(); break; }
        startImu(I_REAL, ET + R.i(300, 2000), true);
        SS = S_FLY; SS_T0 = ET; RAND_MODE = M_STATUS; NEXT_RAND = ET + R.i(200, 1500); NEXT_SCRIPT = ET + 100;
      }
      break;
    case S_FLY:
      if (K.anchoring()) { SS = S_ANCH; SS_T0 = ET; RAND_MODE = M_BENIGN; NEXT_SCRIPT = ET + 100; break; }
      if (ET - SS_T0 > 25000) {
        violate(10, "real flight did not start auto", "nominal: armed with auto, real flight (push %.0f ms, flight %.0f ms, impact %.1f g for %.0f ms) did not start auto: launched %d landing '%s' phase %s",
                IMU.push, IMU.flight, IMU.imp, IMU.impDur, K.flight.launched, K.flight.how, phaseName(K.phase()));
        const char* r = cmdKey('a', 0, "/auto");
        if (r) { toTail(); break; }
        SS = S_ANCH; SS_T0 = ET; RAND_MODE = M_BENIGN; NEXT_SCRIPT = ET + 100; break;
      }
      NEXT_SCRIPT = ET + 100; break;
    case S_ANCH:
      if (K.anchoring() && ET - SS_T0 < 40000) { NEXT_SCRIPT = ET + 100; break; }
      CHECKS[10]++;
      if (K.seq == DONE && !K.fault && P.kick >= P.kickLand && P.deploy >= P.deployEnd) {
        ST.nomAnch++; SS = S_GAP; RAND_MODE = M_GAP; NEXT_RAND = ET + R.i(100, 1000); NEXT_SCRIPT = ET + R.i(300, 3000);
      } else {
        violate(10, std::string("auto ended in '") + seqName(K.seq) + "' fault " + (K.fault ? faultName(K.fault) : "none") + (K.seq == DONE ? ", legs not fully out" : ""),
                "nominal /auto ended with seq '%s' fault '%s': kick %.0f of %.0f, pods %.0f of %.0f, last event '%s'",
                seqName(K.seq), faultName(K.fault), P.kick, P.kickLand, P.deploy, P.deployEnd, K.lastEvent);
        toTail();
      }
      break;
    case S_GAP: {
      const char* r = cmdKey('r', 0, "/release");
      if (r) { violate(10, std::string("release refused: ") + r, "nominal: /release refused after anchoring: %s", r); toTail(); break; }
      SS = S_REL; SS_T0 = ET; RAND_MODE = M_BENIGN; NEXT_RAND = ET + R.i(200, 1500); NEXT_SCRIPT = ET + 100; break;
    }
    case S_REL:
      if (K.releasing() && ET - SS_T0 < 40000) { NEXT_SCRIPT = ET + 100; break; }
      CHECKS[10]++;
      if (K.seq == RELEASED && !K.fault && P.kick <= 0 && P.deploy <= 0) ST.nomRel++;
      else violate(10, std::string("release ended in '") + seqName(K.seq) + "' fault " + (K.fault ? faultName(K.fault) : "none") + (K.seq == RELEASED ? ", legs not stowed" : ""),
                   "nominal /release ended with seq '%s' fault '%s': kick %.0f, pods %.0f, last event '%s'", seqName(K.seq), faultName(K.fault), P.kick, P.deploy, K.lastEvent);
      toTail(); break;
    case S_TAIL: SS = S_END; break;
  }
}

// ------------------------------------------------------------------------------------------------------- episode
static void randomPlant(bool nominal) {
  P = Plant();
  P.vbat = (float)(nominal ? R.r(6.6, 8.4) : R.r(6.2, 8.4));
  P.rint = (float)R.r(0.05, 0.2);
  P.noise = (float)R.r(0, 15);
  P.readFail = nominal ? (R.p(0.5) ? 0 : R.r(0, 0.01)) : (R.p(0.4) ? 0 : R.r(0, 0.05));
  if (nominal) P.maxFailRun = 2;                       // 3 failed reads in a row is a lost sensor (an injected fault)
  P.kFree = (float)R.r(20, 40); P.kStall = (float)R.r(160, 200); P.kPush = (float)R.r(70, 95);
  P.dFree = (float)R.r(20, 40); P.dStall = (float)R.r(160, 200);
  P.drFree = (float)R.r(60, 80); P.drStall = (float)R.r(650, 750); P.drDig = (float)R.r(150, 480);
  P.fFree = (float)R.r(35, 50); P.fStall = (float)R.r(500, 600); P.fDig = (float)R.r(90, 160);
  double c = nominal ? 0.10 : 0.15; P.cDrill = (float)R.r(-c, c); P.cFoot = (float)R.r(-c, c);
  for (int m = 0; m < NMOT; m++) P.rate[m] = (float)R.r(0.85, 1.15);
  P.kickContact = R.r(2000, 3500); P.kickLand = P.kickContact + R.r(1000, 1600);
  if (nominal) {   // documented geometry (API.md NO_GROUND, BENCH_TEST step 5): on flat ground the housing lands before 80 % of the swing in air
    double air = K.tu.kickSwingMs * P.rate[KICK] * 8.0 / 7.6;   // the air swing in plant units
    if (P.kickLand > 0.8 * air) { P.kickLand = 0.8 * air; if (P.kickContact > P.kickLand - 800) P.kickContact = P.kickLand - 800; }
  }
  P.deployEnd = R.r(1100, 1700); P.drillTouch = P.deployEnd * R.r(0.4, 0.8);
  P.drillFull = R.r(1500, 6000); P.footFull = R.r(1200, 5000);
  if (!nominal && R.p(0.08)) {   // on its back or side: the kick never touches ground and stops at its own outer end stop,
    // placed where the firmware's travel count (rate and the 7.6 V plant reference) reads kickSwingMs, within 5 %
    P.noGround = true; P.kickContact = 1e9; P.kickLand = K.tu.kickSwingMs * R.r(0.95, 1.05) * P.rate[KICK] * 8.0 / 7.6; }
  if (!nominal && R.p(0.3)) { P.kick = R.r(0, P.kickLand); P.deploy = R.r(0, P.deployEnd); }   // legs left anywhere
}

static void runEpisode(uint64_t seed, int ep) {
  SEED = seed; EPI = ep;
  R.s = seed * 0x9E3779B97F4A7C15ull ^ ((uint64_t)(ep + 1) * 0xD1B54A32D192ED03ull); R.next(); R.next();
  NOMINAL_EP = R.p(0.35);
  K = Kedge(); T = Track(); EP_TICKS = 0; HOLDS.clear(); ACTS = decltype(ACTS)(); EVR.clear(); IMU = ImuProg(); ET = 0;
  BASE = R.p(0.08) ? 0xFFFFFFFFu - (uint32_t)R.i(0, 60000) : (uint32_t)R.i(0, 200000);   // some episodes cross the millis() wrap
  randomPlant(NOMINAL_EP);
  K.io = {setM, readCurrent, readRail, logCb};
  T.bno = NOMINAL_EP ? true : !R.p(0.05);
  K.imuOk = T.bno;
  int bootF = (!NOMINAL_EP && R.p(0.05)) ? (R.p(0.5) ? F_BROWNOUT : F_WATCHDOG) : F_NONE;
  evlog("episode %d %s: battery %.2f V, Rint %.2f, noise %.1f mA, read fail %.3f, coupling drills %+.0f%% foot %+.0f%%, kick free %.0f push %.0f stall %.0f, "
        "pods free %.0f stall %.0f, drills free %.0f dig %.0f stall %.0f, foot free %.0f dig %.0f stall %.0f, dart clock %lu", ep, NOMINAL_EP ? "nominal" : "chaos",
        P.vbat, P.rint, P.noise, P.readFail, P.cDrill * 100, P.cFoot * 100, P.kFree, P.kPush, P.kStall, P.dFree, P.dStall, P.drFree, P.drDig, P.drStall, P.fFree, P.fDig, P.fStall, (unsigned long)BASE);
  evlog("soil: kick contact %.0f land %.0f, pods end %.0f, drills touch %.0f full %.0f, foot full %.0f, legs at kick %.0f pods %.0f%s%s",
        P.kickContact, P.kickLand, P.deployEnd, P.drillTouch, P.drillFull, P.footFull, P.kick, P.deploy, T.bno ? "" : ", BNO085 missing", bootF ? ", boot after reset fault" : "");
  K.boot(bootF, NOWA());
  uint32_t dur = 0;
  if (NOMINAL_EP) {
    NOMINAL++; SS = S_PRE; RAND_MODE = M_PRE; NEXT_RAND = R.i(100, 1500);
    NEXT_SCRIPT = R.p(0.5) ? 0 : (uint32_t)R.i(1000, 15000);
  } else {
    dur = R.i(15000, 70000); scheduleFaults(dur); SS = S_END; RAND_MODE = M_CHAOS; NEXT_RAND = R.i(50, 1500); NEXT_SCRIPT = UINT32_MAX;
  }
  checkAll(false);
  if (!NOMINAL_EP && R.p(0.3)) { int n = R.i(1, 8); for (int k = 0; k < n; k++) cmdKey('-', 0, "key - (start at a lower speed)"); }
  int logChecks = R.i(0, 2); std::vector<uint32_t> logAt; for (int k = 0; k < logChecks; k++) logAt.push_back(R.i(0, NOMINAL_EP ? 60000 : dur));
  uint32_t nextTick = R.i(4, 7), lastTick = 0;
  for (;;) {
    if (NOMINAL_EP ? SS == S_END : ET >= dur) break;
    uint32_t tc = UINT32_MAX;
    if (!ACTS.empty()) tc = ACTS.top().t;
    for (auto& h : HOLDS) if (h.next < tc) tc = h.next;
    if (RAND_MODE != M_NONE && NEXT_RAND < tc) tc = NEXT_RAND;
    if (NOMINAL_EP && NEXT_SCRIPT < tc) tc = NEXT_SCRIPT;
    if (tc < nextTick) {
      if (tc > ET) { P.step(tc - ET); ET = tc; }
      while (!ACTS.empty() && ACTS.top().t <= ET) { Act a = ACTS.top(); ACTS.pop(); applyAct(a); }
      for (size_t k = 0; k < HOLDS.size(); k++) {
        Hold& h = HOLDS[k];
        if (h.next > ET) continue;
        char lbl[48]; snprintf(lbl, sizeof lbl, "/key?c=%c&ms=%u (held, every %u ms)", h.c, h.ms, h.period);
        h.next += h.period;
        cmdKey(h.c, h.ms, lbl);                        // may clear HOLDS (a stop never comes from here, but be safe)
        if (k >= HOLDS.size()) break;
      }
      bool ended = false;
      for (size_t k = 0; k < HOLDS.size();) if (HOLDS[k].next > HOLDS[k].end) { HOLDS.erase(HOLDS.begin() + k); ended = true; } else k++;
      if (ended && HOLDS.empty() && R.p(0.5)) cmdStop(false, "/key?c=%20&keep=1 (web page: last held key released)");
      if (RAND_MODE != M_NONE && NEXT_RAND <= ET) {
        doRandomCmd();
        NEXT_RAND = ET + (RAND_MODE == M_CHAOS ? R.i(50, 2000) : R.i(150, 1500));
      }
      if (NOMINAL_EP && NEXT_SCRIPT <= ET) scriptStep();
    } else {
      P.step(nextTick - ET); ET = nextTick;
      doTick(nextTick - lastTick); lastTick = nextTick;
      for (auto& t : logAt) if (t && ET >= t) { t = 0; checkLogJson(); }
      nextTick += R.i(4, 7);
    }
  }
  checkLogJson(); checkStatus();
  SIM_MS += ET; EPISODES++;
}

// ------------------------------------------------------------------------------------------------------- launch sweep
// Shortest push over 4 g that the launch detector always sees, with the samples of each tick stamped with the tick time
// (as the sketch does): every push length from 10 to 50 ms at many random phases, for jittered and fixed tick periods.
static int shortestAlwaysSeen(uint64_t seed, int tMin, int tMax, int trials) {
  Rng r; r.s = seed ^ 0xA5A5A5A5DEADBEEFull; int best = -1;
  for (int len = 50; len >= 10; len--) {
    bool all = true;
    for (int k = 0; k < trials && all; k++) {
      Flight f; f.reset(); double p0 = 100 + r.r(0, 10); uint32_t t = 0, prev = 0;
      while (t < p0 + len + 100) {
        prev = t; t += (uint32_t)r.i(tMin, tMax);
        int n = t - prev >= 6 ? 3 : 2;
        for (int j = 0; j < n; j++) { double ts = t - (n - 1 - j) * 2.5; f.update(ts >= p0 && ts < p0 + len ? 8.0f : 1.0f, t); }
      }
      if (f.st == Flight::WAIT) all = false;
    }
    if (!all) break;
    best = len;
  }
  return best;
}

// ------------------------------------------------------------------------------------------------------- main
int main(int argc, char** argv) {
  bool quick = false; std::vector<char*> pos;
  for (int k = 1; k < argc; k++) { if (!strcmp(argv[k], "--quick")) quick = true; else pos.push_back(argv[k]); }
  uint64_t seed = pos.size() > 0 ? strtoull(pos[0], nullptr, 0) : 20261103ull;
  int eps = pos.size() > 1 ? atoi(pos[1]) : quick ? 1000 : 8000;
  int only = pos.size() > 2 ? atoi(pos[2]) : -1;
  VERBOSE = only >= 0;
  auto t0 = std::chrono::steady_clock::now();
  printf("KEDGE dart logic fuzz: seed %llu, %d episode(s)%s\n", (unsigned long long)seed, eps, VERBOSE ? " (single episode, verbose)" : "");
  for (int e = 0; e < eps; e++) { if (only >= 0 && e != only) continue; runEpisode(seed, e); }
  double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

  printf("\n================ summary\n");
  printf("seed %llu: %ld episodes (%ld nominal missions, %ld chaos), %ld ticks, %ld commands, %.2f h simulated, %.1f s wall\n",
         (unsigned long long)seed, EPISODES, NOMINAL, EPISODES - NOMINAL, TICKS, CMDS, SIM_MS / 3.6e6, secs);
  printf("coverage: %ld launches, %ld impact landings, %ld fallback landings, auto started by /auto %ld and by landing %ld, anchored %ld, released %ld,\n"
         "          %ld stalls stopped, %ld pair-check pauses, longest run deploy %lu kick %lu drills %lu foot %lu ms, %ld status and %ld flightlog JSON checks\n",
         ST.launches, ST.impactLandings, ST.fallbackLandings, ST.autoByCmd, ST.autoByLanding, ST.anchored, ST.released, ST.stallStops, ST.pairPauses,
         (unsigned long)ST.maxRun[0], (unsigned long)ST.maxRun[1], (unsigned long)ST.maxRun[2], (unsigned long)ST.maxRun[3], ST.jsonStatus, ST.jsonLog);
  printf("faults latched:");
  for (int f = 1; f < F_NFAULT; f++) printf(" %s %ld", faultName(f), ST.faults[f]);
  printf("\nnominal missions: %ld anchored fully, %ld released stowed\n", ST.nomAnch, ST.nomRel);
  printf("info (not an invariant): manual stalls outside the documented 2.5 s guarantee (3 or 4 motors, motors changing, speed <= 125): %ld,\n"
         "          %ld of them driven over 2.5 s, longest %lu ms (bounded by the 8 s hold cap, invariant 2)\n", ST.manOutside, ST.manOutsideOver, (unsigned long)ST.manOutsideMax);
  printf("info (not an invariant): known limit, pods stalled with the drills' current rising in each pause: %ld driven over 3 s,\n"
         "          longest %lu ms (bounded by the 6 s pods limit, checked in invariant 3)\n", ST.knownPods, (unsigned long)ST.knownPodsMax);
  printf("info (not an invariant): plant stall within 10 %% of the firmware line (outside the 80 %% tuning rule): %ld sequence stalls\n"
         "          driven over 3 s (bounded by the 6 s step limits, checked in invariant 3)\n", ST.knownLow);
  int sweepTrials = quick ? 300 : 2000;
  int sJit = shortestAlwaysSeen(seed, 4, 7, sweepTrials), s5 = shortestAlwaysSeen(seed, 5, 5, sweepTrials);
  printf("info (not an invariant): launch push always detected from %d ms with 4..7 ms tick jitter, from %d ms with fixed 5 ms ticks\n"
         "          (API.md says 22 ms, which assumes 5 ms ticks); in episodes %ld armed pushes, %ld missed, longest missed %.0f ms\n",
         sJit, s5, ST.pushArmed, ST.pushMissed, ST.pushLongestMissed);
  printf("\ninvariant                                          checks   violations   (in nominal missions)\n");
  long total = 0;
  for (int i = 1; i <= 11; i++) { printf("%2d %-44s %10ld %10ld   %10ld\n", i, INV_NAME[i], CHECKS[i], VIOL[i], VIOLN[i]); total += VIOL[i]; }
  if (!SIGS.empty()) {
    printf("\nviolation signatures (count, first seed / episode / time):\n");
    for (auto& s : SIGS) printf("  [%s] x%ld (%ld in nominal missions)  first: seed %llu episode %d t %.3f s\n      %s\n", s.first.c_str(), s.second.n, s.second.nNom, (unsigned long long)s.second.seed, s.second.ep, s.second.t / 1000.0, s.second.msg.c_str());
  }
  printf("\n%s\n", total ? "FUZZ FAIL" : "FUZZ PASS");
  return total ? 1 : 0;
}
