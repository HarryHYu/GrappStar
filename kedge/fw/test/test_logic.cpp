// PC tests of the KEDGE dart logic (kedge_logic.h v4) against a simple plant model.
//   cd firmware/dart/test && ./run_tests.sh       (or: g++ -std=c++17 -O1 -Wall -I../kedge_dart -o test_logic test_logic.cpp && ./test_logic)
// Writes out/status_*.json and out/flightlog_*.json for the JSON check in run_tests.sh.
#include "kedge_logic.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <functional>
#include <string>
#include <vector>
#include <sys/stat.h>
using namespace kedge;

// ------------------------------------------------------------------------------------------------- plant model
// Positions are "ms of travel". Kick 0 = stowed, contact at 3000, housing on the ground at 4300 (stall).
// Deploy 0 = stowed, 1400 = pods fully down (stall at both ends). Currents in mA at 2S.
struct Plant {
  int mdir[4] = {0, 0, 0, 0}, duty[4] = {0, 0, 0, 0};
  double kick = 0, deploy = 0;
  uint32_t kickContact = 3000, kickLand = 4300, deployEnd = 1400;
  bool shortCircuit = false, spike = false, inaDead = false, drillJam = false, footJam = false, kickNoStow = false, drillStallFwd = false, footStallFwd = false;
  float railV = 7.6f;
  float drillPaused = 380, footPaused = 130;
  int open = -1;                               // this motor's wire is open (draws nothing)
  double deployJamBackAt = -1;                  // going up, the pods jam here (stall current)
  double kickJamBackAt = -1;                    // coming back, the kick jams here and draws stall current   // auger current while the worm of the pair is paused (coupling)
  float motorMa(int m) const {
    int d = mdir[m]; if (!d || m == open) return 0;
    switch (m) {
      case KICK:
        if (d > 0) return kick < kickContact ? 25 : kick < kickLand ? 70 : 185;
        if (kickJamBackAt >= 0 && kick <= kickJamBackAt) return 185;
        return (kick > 0 || kickNoStow) ? 30 : 185;
      case FOOT:
        if (footJam && d < 0) return 600;
        if (d > 0) return footStallFwd ? 550 : kick >= kickContact ? (mdir[KICK] > 0 || kick >= kickLand ? 130 : footPaused) : 40;
        return 60;
      case DEPLOY:
        if (d > 0) return deploy < deployEnd ? 30 : 185;
        if (deployJamBackAt >= 0 && deploy <= deployJamBackAt) return 185;
        return deploy > 0 ? 30 : 185;
      case DRILLS:
        // digging once the augers are in the soil; slightly less while the pods pause (the auger keeps cutting)
        if (d > 0) return drillStallFwd ? 700 : deploy > 200 ? ((mdir[DEPLOY] > 0 || deploy >= deployEnd) ? 420 : drillPaused) : (mdir[DEPLOY] > 0 ? 300 : 140);
        return drillJam ? 750 : 200;
    }
    return 0;
  }
  float total() const { float s = 0; for (int m = 0; m < 4; m++) s += motorMa(m) * (duty[m] / 255.f) * (duty[m] / 255.f) * railV / 8.0f; return shortCircuit || spike ? 3000 : s; }
  void step(uint32_t dt) {
    if (mdir[KICK] > 0 && kick < kickLand) kick += dt;
    if (mdir[KICK] < 0 && kick > 0 && !kickNoStow && !(kickJamBackAt >= 0 && kick <= kickJamBackAt)) kick -= dt;
    if (kick < 0) kick = 0;
    if (kickNoStow && mdir[KICK] < 0 && kick > 2000) kick -= dt;   // jams half way back
    if (mdir[DEPLOY] > 0 && deploy < deployEnd) deploy += dt;
    if (mdir[DEPLOY] < 0 && deploy > 0 && !(deployJamBackAt >= 0 && deploy <= deployJamBackAt)) deploy -= dt;
    if (deploy < 0) deploy = 0;
  }
};

static Plant P; static uint32_t NOW; static std::vector<std::string> LOG;
static std::vector<std::vector<int>> DUTY;   // not used beyond direction, kept for debugging
static void setM(int m, int d, int duty) { P.mdir[m] = duty ? d : 0; P.duty[m] = duty; }
static float cur() { return P.inaDead ? NAN : P.total(); }
static float rail() { return P.railV; }
static void lg(const char* s) { char b[160]; snprintf(b, sizeof b, "%6u ms  %s", NOW, s); LOG.push_back(b); }
static int VIDEO_PIN = -1;                     // last level written to the video switch pin (-1 = never)
static void vid(bool on) { VIDEO_PIN = on ? 1 : 0; }
static float GRAV[3] = {0, 0, 9.81f};          // gravity reaction in chip axes while still (chip +z up = belly down)

static Kedge K;
static void fresh() { K = Kedge(); P = Plant(); NOW = 0; LOG.clear(); VIDEO_PIN = -1; GRAV[0] = GRAV[1] = 0; GRAV[2] = 9.81f; K.io = {setM, cur, rail, lg, vid}; K.boot(F_NONE, NOW); }

// IMU feed: |a| trace in g. imuOn=false = BNO085 silent.
static std::function<float(uint32_t)> ACC = [](uint32_t) { return 1.0f; };
static bool IMU_ON = true;
static void run(uint32_t ms) {
  for (uint32_t i = 0; i < ms; i += 5) {
    NOW += 5; P.step(5);
    if (IMU_ON) {
      float a = ACC(NOW); K.onAccel(a, NOW);
      float q[4] = {1, 0, 0, 0}, v[3] = {GRAV[0] / 9.81f * a * 9.81f, GRAV[1] / 9.81f * a * 9.81f, GRAV[2] / 9.81f * a * 9.81f};
      K.imuSample(NOW, q, v);
    }
    K.tick(NOW);
  }
}
static bool allOff() { for (int m = 0; m < 4; m++) if (P.mdir[m]) return false; return true; }
static uint32_t when(const char* s) { for (auto& l : LOG) if (l.find(s) != std::string::npos) return (uint32_t)atoi(l.c_str()); return 0; }
static bool logged(const char* s) { for (auto& l : LOG) if (l.find(s) != std::string::npos) return true; return false; }
static void dumpLog() { for (auto& l : LOG) printf("     %s\n", l.c_str()); }
static std::string ph() { return phaseName(K.phase()); }

static int fails = 0, checks = 0;
#define CHECK(c, msg) do { checks++; if (!(c)) { printf("FAIL: %s  (%s:%d)\n", msg, __FILE__, __LINE__); fails++; } else printf("ok:   %s\n", msg); } while (0)
static void section(const char* s) { printf("\n== %s\n", s); }

// runs the plant until the sequence stops or the limit passes; records each phase seen
static std::vector<std::string> PHASES;
static void runWatch(uint32_t ms) {
  for (uint32_t t = 0; t < ms; t += 5) {
    run(5);
    std::string p = ph(); if (PHASES.empty() || PHASES.back() != p) PHASES.push_back(p);
  }
}
static std::string phases() { std::string s; for (auto& p : PHASES) { if (!s.empty()) s += ">"; s += p; } return s; }

static std::string statusJson() { static char b[1400]; K.status(b, sizeof b, 1.5f, -3.2f, NOW, 7.6f, "poweron"); return b; }
static std::string logJson(int cap = 512) {
  LogJson w; w.begin(&K.log, phaseName(K.phase()), K.flight.launched, K.flight.tLaunch, K.flight.st == Flight::LANDED, K.flight.tLand, K.flight.how);
  std::string out; std::vector<char> buf(cap); int n, maxN = 0;
  while ((n = w.chunk(buf.data(), cap)) > 0) { out.append(buf.data(), n); if (n > maxN) maxN = n; }
  if (maxN >= cap) out += "CHUNK_OVERFLOW";
  return out;
}
static void save(const char* name, const std::string& s) { mkdir("out", 0755); std::string p = std::string("out/") + name; FILE* f = fopen(p.c_str(), "w"); if (f) { fputs(s.c_str(), f); fclose(f); } }
static int countSamples(const std::string& j) { size_t p = j.find("\"samples\":["); if (p == std::string::npos) return -1; int n = 0; for (size_t i = p + 11; i < j.size(); i++) if (j[i] == '[') n++; return n; }

// synthetic flight traces (|a| in g), t0 = launch start
static float flightTrace(uint32_t t, uint32_t t0) {
  if (t < t0) return 1.0f + 0.03f * sinf(t * 0.05f);                // on the rail, small vibration
  uint32_t d = t - t0;
  if (d < 25) return 8.0f;                                           // spring push, BNO085 saturates at 8 g
  if (d < 1000) return 0.4f + 0.2f * sinf(d * 0.07f);                // ballistic flight: drag and spin only
  if (d < 1015) return 8.0f;                                         // impact (saturated)
  if (d < 1150) return 2.0f + 0.8f * sinf(d * 0.3f);                 // settles
  return 1.0f + 0.02f * sinf(d * 0.11f);                             // still on the ground
}

int main() {
  // ===================================================================================================== pin map
  section("pin map (DRV8833 #3 freed)");
  {
    bool distinct = true; std::vector<int> g;
    for (int m = 0; m < NMOT; m++) { g.push_back(PINS[m].gpioA); g.push_back(PINS[m].gpioB); }
    g.push_back(PIN_SDA_GPIO); g.push_back(PIN_SCL_GPIO);
    for (size_t i = 0; i < g.size(); i++) for (size_t j = i + 1; j < g.size(); j++) if (g[i] == g[j]) distinct = false;
    CHECK(distinct, "8 motor pins + SDA/SCL are 10 different GPIOs");
    bool onlyTwo = N_DRIVERS == 2; for (int m = 0; m < NMOT; m++) onlyTwo &= PINS[m].drv == 1 || PINS[m].drv == 2;
    CHECK(onlyTwo, "only DRV#1 and DRV#2 are used: DRV#3 is off the dart");
    CHECK(PINS[DEPLOY].drv == 1 && PINS[DEPLOY].ch == 'A' && PINS[KICK].drv == 1 && PINS[KICK].ch == 'B', "DRV#1 A = deploy, B = kick (unchanged)");
    CHECK(PINS[DRILLS].drv == 2 && PINS[DRILLS].ch == 'A', "DRV#2 channel A = both drill motors");
    CHECK(PINS[FOOT].drv == 2 && PINS[FOOT].ch == 'B', "DRV#2 channel B = foot");
    bool uniq = true; for (int a = 0; a < NMOT; a++) for (int b = a + 1; b < NMOT; b++) if (PINS[a].drv == PINS[b].drv && PINS[a].ch == PINS[b].ch) uniq = false;
    CHECK(uniq, "every motor has its own driver channel");
    // XIAO ESP32S3 Dn -> GPIO (Seeed pinout): D0 1, D1 2, D2 3, D3 4, D4 5, D5 6, D6 43, D7 44, D8 7, D9 8, D10 9
    auto gpioOf = [](const char* l) { static const int map[] = {1, 2, 3, 4, 5, 6, 43, 44, 7, 8, 9}; int n = atoi(l + 1); return (n >= 0 && n <= 10) ? map[n] : -1; };
    bool labels = true; for (int m = 0; m < NMOT; m++) labels &= gpioOf(PINS[m].labelA) == PINS[m].gpioA && gpioOf(PINS[m].labelB) == PINS[m].gpioB;
    CHECK(labels && PIN_SDA_GPIO == 5 && PIN_SCL_GPIO == 6, "Dn labels match the XIAO ESP32S3 GPIO numbers (SDA D4, SCL D5)");
    CHECK(PINS[FOOT].gpioA == 43 && PINS[DRILLS].gpioA != 43 && PINS[DRILLS].gpioB != 44, "reset-twitch pins D6/D7 only on the slow foot worm, drills on D8/D9");
    Tuning t;
    CHECK(700 < DRV_CHANNEL_MAX_MA && t.stallMa[DRILLS] < 700, "drill pair stall 700 mA fits one 1.2 A channel; drill stall threshold below it");
    CHECK(t.tripMa > 180 + 180 + 700 + 550 && t.tripMa < 3200, "trip above all four stalled (1610 mA) and inside the INA219 3.2 A range");
    CHECK(sizeof(Sample) == 20 && sizeof(Log) < 12300, "flight log RAM: 600 x 20 B samples, under 12.3 kB");
  }

  // ===================================================================================================== boot
  section("boot");
  { fresh(); CHECK(allOff() && ph() == "IDLE" && K.fault == F_NONE, "boot: all motors off, phase IDLE");
    fresh(); K.boot(F_BROWNOUT, NOW); CHECK(allOff() && ph() == "FAULT" && std::string(faultName(K.fault)) == "BROWNOUT", "boot after brownout: motors off, phase FAULT (BROWNOUT)");
    K.stop(NOW); CHECK(ph() == "IDLE" && K.fault == F_NONE, "stop clears the boot fault");
    fresh(); K.boot(F_WATCHDOG, NOW); CHECK(ph() == "FAULT" && std::string(faultName(K.fault)) == "WATCHDOG", "boot after watchdog reset: phase FAULT (WATCHDOG)"); }

  // ===================================================================================================== auto anchor
  section("auto anchor");
  { fresh(); PHASES.clear(); PHASES.push_back(ph()); CHECK(K.startAuto(NOW) == nullptr, "auto accepted from IDLE"); runWatch(20000);
    dumpLog();
    CHECK(K.seq == DONE && ph() == "ANCHORED", "auto sequence reaches anchored, phase ANCHORED");
    uint32_t tf = when("foot touches"), tk = when("kick down"), td = when("deploy with"), tp = when("pods down"), ta = when("anchored, holding");
    CHECK(tf >= 3000 && tf < 3400, "foot auger starts when the foot meets the ground (kick current rise)");
    CHECK(tk >= 4300 && tk < 5800, "kick stops after the housing lands (stall read with the foot paused)");
    CHECK(td > tk, "deploy starts only after the foot is planted");
    CHECK(tp >= 1400 + td && tp < td + 4000, "deploy stops on its own stall although the drills draw more");
    CHECK(ta > tp && ta >= tp + 2900, "drills finish 3 s after the pods stop");
    CHECK(allOff(), "everything off at the end");
    CHECK(phases() == "IDLE>ANCHORING>ANCHORED", "phases IDLE > ANCHORING > ANCHORED");
    CHECK(K.startAuto(NOW) != nullptr && std::string(K.startAuto(NOW)) == "already anchored, release first", "auto refused when already anchored"); }
  { fresh(); K.startAuto(NOW); run(1000); CHECK(K.key('1', 1500, NOW) != nullptr && P.mdir[DEPLOY] == 0, "manual keys refused while auto runs");
    CHECK(K.startRelease(NOW) != nullptr, "release refused while auto runs (busy)");
    uint32_t s0 = K.seqStart; run(100); CHECK(K.startAuto(NOW) == nullptr && K.seqStart == s0 && K.seq == K_SWING, "repeated /auto while anchoring: accepted, not restarted");
    K.key(' ', 0, NOW); run(10); CHECK(allOff() && K.seq == ABORTED && ph() == "IDLE", "space aborts the auto sequence, everything off"); }
  { fresh(); P.drillStallFwd = false; K.startAuto(NOW); run(9000); P.drillStallFwd = true; run(4000);
    CHECK(K.seq == DONE && logged("drills stalled"), "drills stalling in the finishing phase stop early and count as anchored"); }
  { fresh(); P.kickContact = 99999; P.kickLand = 99999; K.startAuto(NOW); run(17000);
    CHECK(K.seq == ABORTED && std::string(faultName(K.fault)) == "KICK_TIMEOUT" && ph() == "FAULT" && allOff(), "kick never meets the ground: KICK_TIMEOUT fault, all off"); }

  // ===================================================================================================== release
  section("release");
  { fresh(); K.startAuto(NOW); run(20000); LOG.clear(); PHASES.clear();
    uint32_t t0 = NOW; CHECK(K.startRelease(NOW) == nullptr, "release accepted from ANCHORED");
    run(100); CHECK(P.mdir[DRILLS] == -1 && P.mdir[DEPLOY] == 0 && P.mdir[KICK] == 0 && P.mdir[FOOT] == 0, "release step 1: drills unscrew alone");
    CHECK(K.startRelease(NOW) == nullptr && K.seqStart == t0 && K.startAuto(NOW) != nullptr, "repeated /release accepted without restart; /auto refused while releasing");
    run(3000); CHECK(P.mdir[DRILLS] == -1 && P.mdir[DEPLOY] == -1, "release step 2: pods lift while the drills keep unscrewing (no deploy-alone lead: augers may still be in soil)");
    runWatch(20000);
    dumpLog();
    uint32_t tp = when("release: pods up, foot out"), tk = when("release: kick back"), td = when("release: done");
    CHECK(tp >= t0 + 3000 + 1400 && tp < t0 + 3000 + 3500, "pods stop on the stowed-end stall");
    CHECK(tk >= tp + 1500 && tk < tp + 1600, "foot unscrews alone 1.5 s, then the kick swings back");
    CHECK(td >= tk + 4300 && td < tk + 5900, "kick stops on the stowed stall (read with the foot paused)");
    CHECK(K.seq == RELEASED && allOff() && P.kick == 0 && P.deploy == 0, "released: legs stowed, all motors off");
    CHECK(phases() == "RELEASING>IDLE", "phases RELEASING > IDLE");
    std::string s = statusJson(); CHECK(s.find("\"seq\":\"released\"") != std::string::npos, "status seq 'released'"); }
  { fresh(); K.startAuto(NOW); run(20000); P.drillJam = true; K.startRelease(NOW); run(1500);
    CHECK(std::string(faultName(K.fault)) == "DRILL_JAM" && ph() == "FAULT" && allOff(), "drills jammed on the way out: DRILL_JAM, all off"); }
  { fresh(); K.startAuto(NOW); run(20000); P.footJam = true; K.startRelease(NOW); run(8000);
    CHECK(std::string(faultName(K.fault)) == "FOOT_JAM" && allOff(), "foot jammed on the way out: FOOT_JAM, all off"); }
  { fresh(); K.startAuto(NOW); run(20000); P.kickNoStow = true; P.kick = 4300; K.startRelease(NOW); run(30000);
    CHECK(std::string(faultName(K.fault)) == "KICK_TIMEOUT" && allOff(), "kick never reaches the stow stop: KICK_TIMEOUT within 15 s"); }
  { fresh(); K.tu.releaseMaxMs = 6000; K.startAuto(NOW); run(20000); K.startRelease(NOW); run(8000);
    CHECK(std::string(faultName(K.fault)) == "SEQ_TIMEOUT" && allOff(), "whole-sequence time limit: SEQ_TIMEOUT, all off"); }
  { fresh(); K.startRelease(NOW); run(30000); CHECK(K.seq == RELEASED, "release from IDLE with legs already stowed still ends (stall at once)"); }

  // ===================================================================================================== manual keys
  section("manual keys, deadman, stall");
  { fresh(); for (int i = 0; i < 10; i++) { K.key('2', 350, NOW); run(150); } CHECK(P.mdir[KICK] == 1, "WiFi hold: repeats keep the kick running");
    run(400); CHECK(P.mdir[KICK] == 0, "WiFi hold: key released, stops within the 350 ms deadman"); }
  { fresh(); K.key('4', 1500, NOW); run(1400); CHECK(P.mdir[FOOT] == 1, "serial key runs 1.5 s"); run(200); CHECK(P.mdir[FOOT] == 0, "serial key stops after 1.5 s"); }
  { fresh(); K.key('3', 999999, NOW); run(7900); CHECK(P.mdir[DRILLS] == 1, "manual run capped..."); run(200); CHECK(P.mdir[DRILLS] == 0, "...at 8 s whatever ms was asked"); }
  { fresh(); for (int i = 0; i < 70; i++) { K.key('3', 350, NOW); run(150); }
    CHECK(P.mdir[DRILLS] == 0, "held key (repeats every 150 ms) stops at the 8 s hold cap");
    CHECK(K.key('3', 350, NOW) != nullptr && P.mdir[DRILLS] == 0, "...and stays off while the key is still held");
    run(700); CHECK(K.key('3', 350, NOW) == nullptr, "released and pressed again: runs"); }
  { fresh(); P.inaDead = true; run(20); for (int i = 0; i < 20; i++) { K.key('1', 350, NOW); run(150); }
    CHECK(!K.inaOk && P.mdir[DEPLOY] == 0, "no current sensor: a held key stops after 2 s"); }
  { fresh(); K.key('-', 0, NOW); K.key('-', 0, NOW); K.key('-', 0, NOW); K.key('-', 0, NOW);   // speed 155
    P.kick = 0; K.key('@', 1500, NOW); run(700);
    CHECK(P.mdir[KICK] == 0 && logged("stalled: kick"), "stall threshold scales with speed (kick stall still caught at speed 155 in the plant)");
    K.startAuto(NOW); run(10); CHECK(K.duty() == 255, "sequences always run at full duty"); }
  { fresh(); P.deploy = 1400; K.key('1', 5000, NOW); K.key('3', 5000, NOW); run(1300);
    CHECK(P.mdir[DEPLOY] == 0 && logged("stalled"), "two manual motors: one stalled (deploy in its end + drills working) is caught"); }
  { fresh(); for (int i = 0; i < 20; i++) { K.key('@', 350, NOW); run(150); }   // kick already stowed: held into the stop
    CHECK(P.mdir[KICK] == 0 && logged("stalled: kick"), "held key into an end stop: stall stops it although repeats keep coming");
    run(600); CHECK(K.key('@', 350, NOW) == nullptr, "after the key is released it may be tried again");
    run(100); K.stop(NOW, false); CHECK(K.key('2', 350, NOW) == nullptr && P.mdir[KICK] == 1, "after a stop the other direction is never locked"); }
  { fresh(); P.deploy = 1400; K.key('1', 1500, NOW); run(600); CHECK(P.mdir[DEPLOY] == 0 && logged("stalled: deploy"), "deploy driven into its end: stall stop"); }
  { fresh(); K.key('3', 1500, NOW); K.key('2', 1500, NOW); run(100); K.linkLost(NOW); CHECK(allOff() && logged("WiFi client lost"), "WiFi client lost: manual motors stop at once"); }
  { fresh(); K.startAuto(NOW); run(500); K.linkLost(NOW); CHECK(K.seq == K_SWING && P.mdir[KICK] == 1, "WiFi lost during auto: the sequence carries on"); }
  { fresh(); CHECK(std::string(K.key('x', 100, NOW)) == "unknown key", "unknown key refused"); }

  // ===================================================================================================== faults
  section("faults");
  { fresh(); K.key('3', 1500, NOW); run(100); P.shortCircuit = true; run(30); P.shortCircuit = false;
    CHECK(P.mdir[DRILLS] == 0 && ph() == "FAULT" && std::string(faultName(K.fault)) == "OVERCURRENT", "over-current trip: all off, FAULT OVERCURRENT");
    K.stop(NOW); CHECK(ph() == "IDLE", "stop clears the fault"); }
  { fresh(); K.key('3', 1500, NOW); run(300); P.spike = true; run(5); P.spike = false; run(50); CHECK(P.mdir[DRILLS] == 1 && K.fault == F_NONE, "one 5 ms current spike does not trip"); }
  { fresh(); K.key('3', 1500, NOW); run(100); P.railV = 5.2f; run(700); CHECK(allOff() && std::string(faultName(K.fault)) == "LOW_RAIL", "rail under 5.6 V for 0.5 s while driving: LOW_RAIL, all off"); }
  { fresh(); P.railV = 5.2f; run(1000); CHECK(K.fault == F_NONE, "low rail while idle is not a fault"); }
  { fresh(); K.startAuto(NOW); run(2000); P.inaDead = true; run(20);
    CHECK(std::string(faultName(K.fault)) == "CURRENT_SENSOR" && allOff() && !K.inaOk, "INA219 lost during auto: CURRENT_SENSOR fault, all off");
    CHECK(K.startAuto(NOW) != nullptr && K.startRelease(NOW) != nullptr, "auto and release refused without the current sensor");
    CHECK(K.key('2', 350, NOW) != nullptr, "run keys refused while the fault is latched");
    K.stop(NOW); run(700); CHECK(K.key('2', 350, NOW) == nullptr && P.mdir[KICK] == 1, "after stop (and the key released), manual keys work without the current sensor");
    run(400); CHECK(P.mdir[KICK] == 0, "...and still stop on the deadman");
    P.inaDead = false; run(10); CHECK(K.inaOk, "INA219 readings back: sensor OK again"); }
  { fresh(); P.inaDead = true; run(10); P.inaDead = false; run(10); CHECK(K.inaOk, "one or two failed reads are tolerated"); }

  // ===================================================================================================== flight
  section("flight and landing detection (synthetic IMU traces)");
  { fresh(); IMU_ON = true; uint32_t t0 = 3000; ACC = [&](uint32_t t) { return flightTrace(t, t0); };
    CHECK(K.arm(true, true, NOW) == nullptr && ph() == "ARMED", "arm: phase ARMED");
    PHASES.clear(); runWatch(t0 + 10);
    CHECK(ph() == "ARMED", "4+ g for 10 ms: still ARMED");
    runWatch(25); CHECK(ph() == "FLIGHT" && K.flight.tLaunch == t0, "launch: |a| over 4 g for 20 ms, launch time = start of the push");
    CHECK(K.startAuto(NOW) != nullptr && std::string(K.startAuto(NOW)) == "in flight", "auto refused during FLIGHT");
    CHECK(K.key('3', 350, NOW) != nullptr && P.mdir[DRILLS] == 0, "motor keys refused during FLIGHT");
    CHECK(K.startRelease(NOW) != nullptr, "release refused during FLIGHT");
    runWatch(1000); CHECK(ph() == "FLIGHT", "still FLIGHT just after the impact");
    runWatch(1500);
    CHECK(K.landed && K.flight.tLand >= t0 + 1000 && K.flight.tLand < t0 + 1015 && std::string(K.flight.how) == "impact", "landing: impact over 6 g then still 1 s; landing time = the impact");
    CHECK(ph() == "ANCHORING", "armed with auto: anchoring starts by itself on landing");
    std::string s = statusJson(); save("status_anchoring.json", s);
    CHECK(s.find("\"phase\":\"ANCHORING\"") != std::string::npos && s.find("\"t_launch_ms\":3000") != std::string::npos, "status JSON has phase and t_launch_ms");
    runWatch(20000);
    CHECK(phases() == "ARMED>FLIGHT>ANCHORING>ANCHORED", "phases ARMED > FLIGHT > ANCHORING > ANCHORED");
    // flight log
    CHECK(K.log.frozen, "flight log frozen");
    const Sample& first = K.log.at(0); (void)first;
    std::string j = logJson(); save("flightlog_impact.json", j);
    int n = countSamples(j);
    uint32_t lo = K.flight.tLaunch - 2000, hi = K.flight.tLand + 3000;
    bool inWin = true; uint32_t tmin = 0xffffffff, tmax = 0; int kept = 0;
    for (int i = 0; i < K.log.count; i++) { uint32_t t = K.log.at(i).t; if ((int32_t)(t - lo) >= 0) { kept++; if (t < tmin) tmin = t; if (t > tmax) tmax = t; if (t > hi + 20) inWin = false; } }
    printf("     log window %u..%u ms, launch %u, land %u, %d samples\n", tmin, tmax, K.flight.tLaunch, K.flight.tLand, n);
    CHECK(tmin >= lo && tmin < lo + 25, "log starts 2 s before launch detection");
    CHECK(inWin && tmax >= hi - 20, "log ends 3 s after landing, then frozen");
    CHECK(n == kept && n >= 290 && n <= 305, "about 50 Hz: (2 + 1 + 3) s window = about 300 samples");
    CHECK(j.rfind("{\"hz\":50,", 0) == 0 && j.size() > 2 && j.substr(j.size() - 2) == "]}" && j.find("CHUNK_OVERFLOW") == std::string::npos, "flightlog JSON shape, no chunk overflow");
    CHECK(j.find("\"landing\":\"impact\"") != std::string::npos, "flightlog says how it landed");
    int before = K.log.head; run(2000); CHECK(K.log.head == before, "frozen log ignores new samples");
    if (K.seq == DONE) { CHECK(K.arm(true, false, NOW) != nullptr, "arming while ANCHORED is refused (release first)"); K.key('!', 100, NOW); run(300);
      CHECK(K.arm(true, false, NOW) != nullptr, "...and still refused after a key tap with the legs out ('legs out: release first')");
      K.startRelease(NOW); run(40000); }
    CHECK(K.arm(true, false, NOW) == nullptr, "re-arm after the release");
    CHECK(!K.log.frozen, "re-arming unfreezes the log"); }
  { fresh(); ACC = [](uint32_t t) { return (t > 2000 && t < 2015) ? 4.5f : 1.0f; }; K.arm(true, true, NOW); run(4000);
    CHECK(ph() == "ARMED", "a 15 ms knock over 4 g is not a launch"); }
  { fresh(); ACC = [](uint32_t t) { return flightTrace(t, 1000); }; run(5000); CHECK(ph() == "IDLE" && !K.flight.launched, "not armed: a launch trace is ignored"); }
  { fresh(); ACC = [](uint32_t t) { return t < 1000 ? 1.0f : t < 1030 ? 8.0f : t < 1800 ? 0.3f : 1.0f; };   // grass: no impact spike
    K.arm(true, false, NOW); PHASES.clear(); runWatch(5000);
    CHECK(K.landed && std::string(K.flight.how) == "still" && K.flight.tLand >= 1800 && K.flight.tLand < 1810, "soft landing without an impact spike: landed after 2 s still");
    CHECK(ph() == "LANDED" && phases() == "ARMED>FLIGHT>LANDED", "log-only arm: phases ARMED > FLIGHT > LANDED, no auto"); }
  { fresh(); ACC = [](uint32_t t) { if (t < 1000) return 1.0f; if (t < 1030) return 8.0f; if (t < 1500) return 0.5f;
      if (t < 1510) return 7.5f;
      if (t < 1700) return 0.5f + 0.4f * sinf(t * 0.1f);   // tether jerk mid-flight, still flying
      if (t < 1715) return 8.0f;
      if (t < 3500) return (t / 40) % 2 ? 1.6f : 0.6f;     // tumbles after the impact
      if (t < 3510) return 6.5f;
      return 1.0f; };                                       // last bounce, then still
    K.arm(true, false, NOW); run(8000);
    CHECK(K.landed && K.flight.tLand >= 3500 && K.flight.tLand < 3510, "tether jerk and tumbling: landing time = the last impact before stillness"); }
  { fresh(); ACC = [](uint32_t t) { return t < 1000 ? 1.0f : t < 1040 ? 8.0f : 0.5f; }; K.arm(true, false, NOW); run(1100); IMU_ON = false; run(16000); IMU_ON = true;
    CHECK(K.landed && std::string(K.flight.how) == "timeout" && ph() == "LANDED", "IMU silent in flight: landing assumed after the 15 s flight timeout"); }
  { fresh(); ACC = [](uint32_t t) { return t < 1000 ? 1.0f : t < 1100 ? 8.0f : 0.5f; }; K.arm(true, true, NOW); run(1500);
    CHECK(ph() == "FLIGHT", "long saturated push (100 ms) is not taken as the impact");
    K.stop(NOW, false); CHECK(ph() == "FLIGHT" && K.armed, "stop with disarm=0 keeps flight tracking and the arm");
    K.stop(NOW); CHECK(ph() != "FLIGHT" && !K.armed && std::string(K.flight.how) == "stop", "stop during FLIGHT ends flight tracking and disarms");
    run(3100); CHECK(K.log.frozen, "...and the log freezes 3 s later"); CHECK(K.key('2', 350, NOW) == nullptr, "manual keys work again after stop"); }
  { fresh(); for (int i = 0; i < 2000; i++) { NOW += 5; float a = 1.0f + ((i % 37) == 0 ? 2.5f : 0) + ((i % 11) == 0 ? -0.3f : 0); K.onAccel(a, NOW); K.tick(NOW); }
    CHECK(!K.flight.launched, "armed? no; bench handling never triggers"); }
  { fresh(); K.arm(true, false, NOW); for (int i = 0; i < 4000; i++) { NOW += 5; float a = 1.0f + ((i % 37) < 2 ? 3.5f : 0); K.onAccel(a, NOW); K.tick(NOW); }
    CHECK(!K.flight.launched, "armed: repeated 10 ms knocks to 4.5 g never trigger"); }
  { fresh(); K.imuOk = false; CHECK(K.arm(true, true, NOW) != nullptr && ph() == "IDLE", "arm refused without the BNO085"); }
  { fresh(); K.inaOk = false; CHECK(K.arm(true, true, NOW) != nullptr && K.arm(true, false, NOW) == nullptr, "arm with auto refused without the INA219, log-only arm allowed"); }
  { fresh(); K.key('A', 0, NOW); CHECK(K.armed, "key A arms"); K.key('A', 0, NOW); CHECK(!K.armed && ph() == "IDLE", "key A again disarms"); }

  // ===================================================================================================== ring buffer
  section("flight log ring buffer");
  { fresh(); ACC = [](uint32_t) { return 1.0f; }; run(1000);
    CHECK(K.log.count >= 49 && K.log.count <= 51, "50 Hz grid from a 200 Hz feed");
    run(20000); CHECK(K.log.count == LOG_N, "ring holds 600 samples (12 s)");
    uint32_t span = K.log.at(LOG_N - 1).t - K.log.at(0).t; CHECK(span >= 11960 && span <= 12000, "ring spans 12 s, oldest dropped");
    bool mono = true; for (int i = 1; i < K.log.count; i++) if (K.log.at(i).t <= K.log.at(i - 1).t) mono = false; CHECK(mono, "samples in time order across the wrap");
    std::string j = logJson(); save("flightlog_live.json", j);
    CHECK(j.find("\"frozen\":false") != std::string::npos && j.find("\"t_launch_ms\":null") != std::string::npos && countSamples(j) == LOG_N, "not frozen: whole live ring, t_launch_ms null"); }
  { Log L; L.clear(); float q[4] = {0.7071f, 0.f, -0.7071f, 0.f}, a[3] = {-12.34f, 0.05f, 78.4f};
    L.offer(100, q, a); const Sample& s = L.at(0);
    CHECK(fabsf(s.q[0] / 16384.f - q[0]) < 1e-4f && fabsf(s.q[2] / 16384.f - q[2]) < 1e-4f, "quaternion stored to 1e-4");
    CHECK(fabsf(s.a[0] / 100.f - a[0]) < 0.006f && fabsf(s.a[2] / 100.f - a[2]) < 0.006f, "acceleration stored to 0.01 m/s^2 (8 g = 78.5 fits)");
    float big[3] = {500, -500, 0}; L.offer(200, q, big); CHECK(L.at(1).a[0] == 32767 && L.at(1).a[1] == -32767, "out-of-range values clamp, no wrap"); }
  { Log L; L.clear(); float q[4] = {1, 0, 0, 0}, a[3] = {0, 0, 9.81f}; int stored = 0;
    for (uint32_t t = 0; t < 1000; t += 7) stored += L.offer(t, q, a);
    CHECK(stored >= 49 && stored <= 51, "uneven 7 ms feed still logs 50 per second");
    uint32_t t = 5000; stored = L.offer(t, q, a); CHECK(stored == 1 && L.at(L.count - 1).t == 5000, "after a gap the grid restarts at once"); }
  { fresh(); ACC = [](uint32_t t) { return t < 15000 ? 1.0f : t < 15030 ? 8.0f : t < 25000 ? 0.4f + 0.3f * sinf(t * 0.01f) : t < 25015 ? 8.0f : 1.0f; };
    K.arm(true, false, NOW); run(32000);
    std::string j = logJson(256); save("flightlog_long.json", j);
    CHECK(K.landed && K.log.frozen && countSamples(j) >= LOG_N - 5 && K.log.at(0).t >= K.flight.tLaunch - 2020 && K.log.at(0).t <= K.flight.tLaunch - 1980,
          "10 s flight: window longer than 12 s keeps 2 s before launch and the first 10 s (launch never lost)");
    CHECK(j.find("CHUNK_OVERFLOW") == std::string::npos, "small 256 B chunks never overflow"); }
  { fresh(); std::string j = logJson(); save("flightlog_empty.json", j); CHECK(countSamples(j) == 0 && j.find("\"n\":0") != std::string::npos, "empty log is valid JSON with no samples"); }

  // ===================================================================================================== review round 1 regressions
  section("review round 1 regressions");
  { fresh(); K.onAccel(NAN, NOW); K.onAccel(INFINITY, NOW); static char b[1024]; K.status(b, sizeof b, NAN, INFINITY, NOW, NAN, "x");
    std::string st = b; save("status_nan.json", st); CHECK(st.find("nan") == std::string::npos && st.find("inf") == std::string::npos, "NaN / inf inputs never reach the status JSON"); }
  { fresh(); K.startAuto(NOW); run(20000); CHECK(K.arm(true, true, NOW) != nullptr && ph() == "ANCHORED", "arming while ANCHORED is refused (legs out, release first)");
    K.startRelease(NOW); run(40000); CHECK(K.arm(true, true, NOW) == nullptr && ph() == "ARMED", "after the release, arming starts a new shot (phase ARMED)");
    uint32_t t0 = NOW + 1000; ACC = [&](uint32_t t) { return flightTrace(t, t0); }; run(4000); CHECK(ph() == "ANCHORING", "...and auto runs on the next landing"); }
  { fresh(); NOW = 0xFFFFFFFFu - 4000; K.boot(F_NONE, NOW); K.arm(true, false, NOW); uint32_t t0 = 1000;   // launch 5 s later, just after the wrap
    ACC = [&](uint32_t t) { return flightTrace(t, t0); }; run(12000);
    std::string j = logJson(); int n = countSamples(j); printf("     wrap: %d samples\n", n);
    CHECK(K.landed && K.log.frozen && n >= 290 && n <= 305, "launch just after the millis() wrap keeps the full 2 s before launch"); }
  { fresh(); P.kickLand = 1000000000; K.startAuto(NOW); run(3500); P.footStallFwd = true; uint32_t t1 = NOW; run(3000);
    CHECK(P.mdir[FOOT] == 0 && logged("foot stalled (stone?)"), "foot stalls while the kick plants: foot stopped");
    CHECK(when("foot stalled (stone?)") - t1 < 1500, "...within 1.5 s"); }
  { fresh(); K.startAuto(NOW); while (K.seq != D_DEPLOY && NOW < 20000) run(5); run(300); P.drillStallFwd = true; run(1500);
    CHECK(logged("drills stalled (stone?)"), "drills stall while the pods swing: drills stopped, pods carry on");
    P.drillStallFwd = false; run(15000); CHECK(K.seq == DONE, "...and the sequence still anchors"); }
  { fresh(); K.startAuto(NOW); run(20000); K.startRelease(NOW); run(3300); P.drillJam = true; run(1500);
    CHECK(std::string(faultName(K.fault)) == "DRILL_JAM" && allOff(), "drills jam while the pods lift: DRILL_JAM, all off"); }
  { fresh(); K.startAuto(NOW); run(20000); K.startRelease(NOW); while (K.seq != R_KICK && NOW < 60000) run(5); run(300); P.footJam = true; run(1500);
    CHECK(std::string(faultName(K.fault)) == "FOOT_JAM" && allOff(), "foot jams while the kick swings back: FOOT_JAM, all off"); }
  { fresh(); ACC = [](uint32_t t) { return t < 1000 ? 1.0f : t < 1450 ? 0.05f : t < 1480 ? 7.0f : 1.0f; };   // dropped onto a cushion
    K.arm(true, true, NOW); run(20000);
    CHECK(!K.flight.launched && ph() == "ARMED" && allOff(), "armed dart dropped on a cushion: not a launch at all (free fall before the hit), auto NOT started");
    fresh(); ACC = [](uint32_t t) { return t < 1000 ? 1.0f : t < 1030 ? 8.0f : 1.0f; }; K.arm(true, true, NOW); run(20000);
    CHECK(!K.running() && K.seq == IDLE, "armed dart knocked hard on the bench: auto NOT started"); }
  { for (int ms : {15, 20, 40}) { fresh(); ACC = [ms](uint32_t t) { return t >= 1000 && t < 1000u + ms ? 8.0f : t > 1000u + ms && t < 1600 ? 0.3f : 1.0f; };
      K.arm(true, false, NOW); run(1700); char m[96]; snprintf(m, sizeof m, "%d ms push over 4 g: %s", ms, ms >= 20 ? "launch" : "no launch");
      CHECK(K.flight.launched == (ms >= 20), m); } }
  { fresh(); uint32_t t0 = 2000; ACC = [&](uint32_t t) { return flightTrace(t, t0); }; K.arm(true, true, NOW); run(4500);
    K.stop(NOW); CHECK(K.seq == ABORTED && ph() == "LANDED", "landed, auto stopped by hand: phase LANDED (not IDLE)"); }
  { fresh(); ACC = [](uint32_t t) { if (t < 1000) return 1.0f; uint32_t d = t - 1000; if (d < 40) return 8.0f; if (d < 800) return 0.25f; if (d < 810) return 8.0f; return 1.0f; };   // base launcher: 28 g for 39 ms
    K.arm(true, true, NOW); run(3000); CHECK(K.flight.launched && K.landed && ph() == "ANCHORING", "base launcher shot (28 g, 39 ms push, 0.8 s flight): launch, landing, auto"); }

  // ===================================================================================================== review round 2 regressions
  section("review round 2 regressions");
  { fresh(); P.kick = 0; for (int i = 0; i < 40; i++) { K.key('@', 350, NOW); run(400); }   // repeats slower than the deadman
    CHECK(P.mdir[KICK] == 0 && logged("stalled: kick"), "key repeats every 400 ms (slower than the 350 ms deadman): stall still stops it"); }
  { fresh(); for (int i = 0; i < 40; i++) { K.key('3', 350, NOW); run(400); }
    CHECK(P.mdir[DRILLS] == 0 && NOW > 8000, "key repeats every 400 ms: the 8 s hold cap still applies"); }
  { for (float v : {6.4f, 7.4f, 8.4f}) { fresh(); P.railV = v; K.startAuto(NOW); run(25000); char m[96];
      snprintf(m, sizeof m, "battery at %.1f V: auto anchors (thresholds scale with rail voltage)", v); CHECK(K.seq == DONE, m);
      K.startRelease(NOW); run(30000); snprintf(m, sizeof m, "battery at %.1f V: release ends stowed", v); CHECK(K.seq == RELEASED && P.kick == 0 && P.deploy == 0, m); } }
  { fresh(); P.kick = 0; int n = 0; K.io.current_mA = []() { static int k = 0; return (++k % 3 == 0) ? NAN : P.total(); };
    for (int i = 0; i < 20; i++) { K.key('@', 350, NOW); run(150); n++; }
    CHECK(P.mdir[KICK] == 0 && logged("stalled: kick") && when("stalled: kick") < 1500, "1 failed current read in 3: a stalled kick still stops within 1.5 s"); (void)n; }
  { fresh(); K.key('2', 5000, NOW); K.key('3', 5000, NOW); P.kick = 4300; run(1500);
    CHECK(P.mdir[KICK] == 0 && P.mdir[DRILLS] == 1, "manual kick + drills in air: the stalled kick is stopped, the drills keep running");
    fresh(); P.kick = 0; K.key('@', 5000, NOW); K.key('$', 5000, NOW); run(1500);
    CHECK(P.mdir[KICK] == 0 && P.mdir[FOOT] == -1, "manual kick + foot: the stalled kick is stopped, the foot keeps running"); }
  { fresh(); K.key('2', 5000, NOW); K.key('4', 5000, NOW); run(2000);
    CHECK(P.mdir[KICK] == 1 && P.mdir[FOOT] == 1, "manual kick + foot both working (no stall): both keep running"); }
  { fresh(); ACC = [](uint32_t t) { if (t < 1000) return 1.0f; uint32_t d = t - 1000; if (d < 30) return 8.0f; if (d < 900) return 1.4f + 0.3f * sinf(d * 0.1f);   // tumbling: never under 0.6 g
      if (d < 915) return 8.0f;
      return 1.0f; };
    K.arm(true, true, NOW); run(3500); CHECK(K.landed && std::string(K.flight.how) == "impact" && ph() == "ANCHORING", "tumbling flight (1.1 to 1.7 g): impact landing, auto starts"); }

  { fresh(); for (int i = 0; i < 4; i++) { K.key('3', 350, NOW); run(150); } P.railV = 5.0f; for (int i = 0; i < 8; i++) { K.key('3', 350, NOW); run(150); }
    CHECK(P.mdir[DRILLS] == 0 && std::string(faultName(K.fault)) == "LOW_RAIL" && !logged("LOW RAIL: battery flat, all stopped\n"), "fault while a key is held: the held key does not restart the motor");
    int trips = 0; for (auto& l : LOG) if (l.find("LOW RAIL") != std::string::npos) trips++; CHECK(trips == 1, "...exactly one LOW_RAIL trip during the hold"); }
  { fresh(); K.arm(true, true, NOW); ACC = [](uint32_t t) { return t < 1000 ? 1.0f : t < 1030 ? 8.0f : t < 1800 ? 0.3f : t < 1815 ? 8.0f : 1.0f; };
    run(1010); for (int i = 0; i < 6; i++) { NOW += 5; K.onAccel(8.0f, NOW + 1); K.tick(NOW); }   // IMU event stamped 1 ms after the tick time
    CHECK(K.flight.flying(), "an IMU timestamp 1 ms ahead of the tick does not end the flight (no unsigned wrap)"); }

  // ===================================================================================================== review round 3 regressions
  section("review round 3 regressions");
  { fresh(); P.drillPaused = 336; K.startAuto(NOW); run(25000);   // drills drop 20 % (84 mA) whenever the pods pause
    CHECK(K.seq == DONE && P.deploy >= 1400, "drill current drops 20 % while the pods pause: pods still go fully down");
    fresh(); P.footPaused = 75; K.startAuto(NOW); run(25000);       // foot drops 40 % whenever the kick pauses
    CHECK(K.seq == DONE && P.kick >= 4300, "foot current drops 40 % while the kick pauses: kick still plants fully"); }
  { fresh(); P.footPaused = 160; K.startAuto(NOW); run(25000);       // foot rises when the kick pauses: kick stall may be missed
    CHECK(K.seq == DONE && K.fault == F_NONE, "foot current rises while the kick pauses: plant time limit, still anchors, no fault"); }
  { fresh(); P.kick = 0; for (int i = 0; i < 50; i++) { K.key('@', 500, NOW); run(560); }
    CHECK(P.mdir[KICK] == 0 && logged("stalled: kick"), "ms=500 repeats every 560 ms into an end stop: stall stop holds (window = ms + 250)");
    fresh(); for (int i = 0; i < 40; i++) { K.key('3', 500, NOW); run(560); } CHECK(P.mdir[DRILLS] == 0 && NOW > 8000, "...and the 8 s hold cap holds"); }
  { fresh(); srand(7); K.io.current_mA = []() { return P.total() + (rand() % 31 - 15); }; K.spd = 125; P.kick = 0;
    for (int i = 0; i < 20; i++) { K.key('@', 350, NOW); run(150); }
    CHECK(P.mdir[KICK] == 0 && logged("stalled: kick"), "+/-15 mA read noise at speed 125: held kick into its stop still stalls out"); }
  { fresh(); ACC = [](uint32_t t) { if (t < 1000) return 1.0f; uint32_t d = t - 1000; if (d < 200) return 0.02f; if (d < 222) return 8.0f; if (d < 350) return 0.02f; if (d < 358) return 8.0f; return 1.0f; };
    K.arm(true, true, NOW); run(5000); CHECK(!K.running() && K.seq == IDLE, "armed dart dropped 20 cm onto a desk with a bounce: auto NOT started"); }
  { fresh(); K.boot(F_BROWNOUT, NOW); CHECK(K.key('3', 350, NOW) != nullptr && P.mdir[DRILLS] == 0, "boot fault latched: run keys refused until stop"); }
  { fresh(); K.arm(true, false, NOW); K.key('3', 1500, NOW); ACC = [](uint32_t t) { return t < 400 ? 1.0f : t < 440 ? 8.0f : 0.3f; }; run(500);
    CHECK(K.flight.flying() && P.mdir[DRILLS] == 0 && logged("launch: motors stopped"), "a manual motor running at launch is stopped"); }

  // ===================================================================================================== review round 4 (hardware) regressions
  section("review round 4 (hardware) regressions");
  { fresh(); P.open = KICK; K.startAuto(NOW); run(2000);
    CHECK(std::string(faultName(K.fault)) == "NO_CURRENT" && allOff(), "kick wire open (or shorted, driver chopping): NO_CURRENT fault within 1 s, all off");
    fresh(); P.open = DRILLS; K.key('3', 1500, NOW); run(1000);
    CHECK(P.mdir[DRILLS] == 0 && logged("drills draws no current") && K.fault == F_NONE, "manual key on an open drill wire: stopped with a message, no fault latched"); }
  { fresh(); K.key('-', 0, NOW); K.key('-', 0, NOW); K.key('2', 1500, NOW); run(1000);
    CHECK(P.mdir[KICK] == 1 && !logged("no current"), "free-running kick at speed 205 is not mistaken for an open wire"); }
  { fresh(); ACC = [](uint32_t t) { if (t < 1000) return 1.0f; uint32_t d = t - 1000; if (d < 30) return 8.0f; if (d < 900) return 0.4f; if (d < 905) return 5.0f; return 1.0f; };
    K.arm(true, true, NOW); run(3500);
    CHECK(K.landed && std::string(K.flight.how) == "impact" && ph() == "ANCHORING", "impact read at only 5 g (sensor filter): still an impact landing, auto starts");
    std::string st = statusJson(); CHECK(st.find("\"peak_g\":8.00") != std::string::npos, "status peak_g holds the highest |a| since arming"); }
  { fresh(); K.key('3', 1500, NOW); run(100); CHECK(P.duty[DRILLS] == 255, "manual at speed 255 drives full duty (slow decay with a DC input)"); }

  // ===================================================================================================== review round 5 regressions
  section("review round 5 regressions");
  { fresh(); K.startAuto(NOW); run(20000); K.startRelease(NOW);
    K.io.current_mA = []() { return (NOW == 23000) ? NAN : P.total(); }; run(25000);
    CHECK(K.seq == RELEASED && K.fault == F_NONE, "one failed current read at a step end does not abort the release"); }
  { fresh(); P.deploy = 1400; for (int i = 0; i < 100; i++) { K.key('1', 350, NOW); K.key('!', 350, NOW); run(150); }
    int on = 0; for (int i = 0; i < 20; i++) { K.key('1', 350, NOW); K.key('!', 350, NOW); run(150); if (P.mdir[DEPLOY]) on++; }
    CHECK(on == 0 && logged("both directions held"), "both directions of one motor held: stopped and kept off"); }
  { fresh(); K.key('-', 0, NOW); for (int i = 0; i < 8; i++) K.key('-', 0, NOW); K.key('2', 1500, NOW); run(1200);   // speed 50
    CHECK(P.mdir[KICK] == 1 && !logged("stalled"), "lowest speed: a free-running kick is not taken as stalled"); }
  { fresh(); ACC = [](uint32_t t) { return flightTrace(t, 2000); }; K.arm(true, true, NOW); run(2100);
    K.key('3', 350, NOW); K.stop(NOW, false); run(3000);   // web page: key refused in flight, then its keyup stop (keep arm)
    CHECK(ph() == "ANCHORING", "a key tapped in FLIGHT and its keyup stop (disarm=0) do not cancel auto on landing"); }
  { fresh(); uint32_t t0 = 2000; ACC = [&](uint32_t t) { return flightTrace(t, t0); }; K.arm(true, false, NOW); run(5000);
    K.key('2', 1500, NOW); run(100); CHECK(ph() == "IDLE", "landed, then a manual motor runs: phase IDLE, not LANDED"); run(1500); CHECK(ph() == "LANDED", "...and LANDED again when it stops"); }

  { fresh(); K.startRelease(NOW); run(30000);   // legs already stowed: pods and kick start at their stops
    int podsOn = 0; (void)podsOn;
    uint32_t tp = when("release: pods up"), ta = when("pods up, foot out"), tk = when("release: kick back"), td = when("release: done");
    printf("     stowed release: pods step %u ms, kick step %u ms\n", ta - tp, td - tk);
    CHECK(K.seq == RELEASED && ta && ta - tp < 2500 && td - tk < 2500, "release with legs already stowed: each worm stops within 2.5 s"); }
  { fresh(); K.startAuto(NOW); while (K.seq != D_DEPLOY) run(5); run(1000); K.stop(NOW); run(500); LOG.clear(); uint32_t t0 = NOW; K.startAuto(NOW); run(25000);
    uint32_t tk = when("kick already down"); printf("     retry: kick off after %u ms\n", tk - t0);
    CHECK(K.seq == DONE && tk && tk - t0 < 1000, "auto retried with the kick already planted: kick stopped within 1 s, foot planted alone, anchored"); }

  // ===================================================================================================== coordinator review R1-B
  section("coordinator review R1-B regressions");
  { fresh(); ACC = [](uint32_t t) { if (t < 5000) return 1.0f; if (t < 5040) return 8.0f; if (t < 6540) return 0.2f; if (t < 6555) return 6.0f;
      return ((t - 6555) % 800) < 40 ? 1.4f : 1.0f; };   // impact, then nudged every 0.8 s: never still, lands by timeout
    K.arm(true, false, NOW); run(25000);
    printf("     timeout landing: tLaunch %u tLand %u how %s, log %u..%u\n", K.flight.tLaunch, K.flight.tLand, K.flight.how, K.log.at(0).t, K.log.at(K.log.count - 1).t);
    CHECK(std::string(K.flight.how) == "timeout" && K.log.frozen && K.log.at(0).t <= K.flight.tLaunch - 1980 && K.log.at(K.log.count - 1).t >= K.flight.tLand,
          "timeout landing after an impact: the frozen log still holds 2 s before launch through the impact"); }
  for (uint32_t fl : {7000u, 8500u, 10000u}) {   // impact landings 7 to 10 s after launch: the freeze cap keeps the launch
    fresh(); static uint32_t F; F = fl;
    ACC = [](uint32_t t) { if (t < 5000) return 1.0f; if (t < 5040) return 8.0f; if (t < 5000 + F) return 0.2f; if (t < 5015 + F) return 6.0f; return 1.0f; };
    K.arm(true, false, NOW); run(5000 + fl + 6000);
    bool impactIn = false; for (int k = 0; k < K.log.count; k++) if (K.log.at(k).t >= 5000 + fl - 20 && K.log.at(k).t <= 5000 + fl + 40) impactIn = true;
    char m[140]; snprintf(m, sizeof m, "landing %u ms after launch: frozen log keeps 2 s before launch (first %u, launch %u)%s", fl, K.log.at(0).t, K.flight.tLaunch,
                          fl <= 9900 ? " and the impact" : "");
    CHECK(K.landed && K.log.frozen && K.log.at(0).t >= K.flight.tLaunch - 2020 && K.log.at(0).t <= K.flight.tLaunch - 1980 && (impactIn || fl > 9900), m); }
  { fresh(); for (int i = 0; i < 260; i++) { K.key(i % 2 ? '!' : '1', 350, NOW); run(75); }
    CHECK(P.mdir[DEPLOY] == 0 && logged("both directions held"), "R1-B #1: '1' and '!' alternating every 75 ms: stopped, not driven for 20 s"); }

  // ===================================================================================================== failure-mode walk gaps
  section("failure-mode walk gap fixes");
  { fresh(); K.key('2', 1500, NOW); run(200); K.stop(NOW); for (int i = 0; i < 4; i++) { K.key('2', 350, NOW); run(150); }
    CHECK(P.mdir[KICK] == 0, "a stop also locks a key another client keeps repeating"); run(700); CHECK(K.key('2', 350, NOW) == nullptr, "...until that key is released"); }
  { fresh(); K.arm(true, false, NOW); CHECK(K.startAuto(NOW) == nullptr && !K.armed && K.seq == K_SWING && logged("disarmed first"), "/auto while ARMED (launch never seen): disarms and starts"); }
  { fresh(); P.railV = 6.3f; run(20); CHECK(K.arm(true, true, NOW) != nullptr && std::string(K.arm(true, true, NOW)) == "battery low", "arming with auto refused on a flat battery (6.3 V at rest)");
    CHECK(K.arm(true, false, NOW) == nullptr, "...a log-only arm is still allowed (no motors)"); }
  { fresh(); K.key('3', 1500, NOW); run(100); P.railV = 0.0f; run(700); CHECK(std::string(faultName(K.fault)) == "LOW_RAIL", "motor rail reading 0 V while driving: LOW_RAIL"); }
  { fresh(); ACC = [](uint32_t) { return 1.0f; }; K.arm(true, false, NOW); run(15UL * 60 * 1000 + 1000); CHECK(!K.armed && logged("arm expired"), "an arm expires after 15 min without a launch"); }
  { fresh(); ACC = [](uint32_t t) { if (t < 1000) return 1.0f; if (t < 1500) return 0.02f; if (t < 1530) return 8.0f; if (t < 1800) return 0.02f; if (t < 1810) return 8.0f; return 1.0f; };   // 1.2 m drop, bounce
    K.arm(true, true, NOW); run(5000); CHECK(!K.flight.launched && allOff(), "armed dart dropped 1.2 m with a bounce: not a launch"); }
  { fresh(); P.open = DEPLOY; K.startAuto(NOW); run(20000);
    CHECK(std::string(faultName(K.fault)) == "NO_CURRENT" && logged("deploy draws no current"), "deploy wire open: NO_CURRENT at the start of the pods step"); }
  { fresh(); P.kickNoStow = true; P.kick = 4300; K.kickPos = 4300; K.startRelease(NOW); run(30000);
    CHECK(std::string(faultName(K.fault)) == "KICK_JAM" || std::string(faultName(K.fault)) == "KICK_TIMEOUT", "kick jams half way back: KICK_JAM (not 'released')"); }
  { fresh(); P.kickContact = 0; P.kickLand = 0; K.startAuto(NOW); run(5000);   // kick blocked right at stowed (jam or backwards)
    CHECK(std::string(faultName(K.fault)) == "KICK_JAM" && allOff(), "kick stalled near stowed at the start of /auto: KICK_JAM, not 'kick already down'"); }
  { fresh(); P.footStallFwd = false; K.startAuto(NOW); run(25000); std::string st = statusJson();
    CHECK(st.find("\"warnings\":[]") != std::string::npos, "clean anchor: no warnings");
    fresh(); P.drillStallFwd = true; K.startAuto(NOW); run(25000); st = statusJson(); save("status_warnings.json", st);
    CHECK(st.find("drills_stalled") != std::string::npos, "drills stopped by a stone: status warnings list says so"); }

  // ===================================================================================================== review round 8 regressions
  section("review round 8 regressions");
  { for (int ramp : {0, 3, 8, 15, 25}) { fresh(); ACC = [ramp](uint32_t t) { if (t < 2000) return 1.0f + 0.25f * sinf(t * 0.9f);   // rail vibration 0.18 g rms
        if (t < 2000u + ramp) return 1.0f + 7.0f * (t - 2000) / (float)(ramp ? ramp : 1);   // ramp up through 1.3..4 g
        if (t < 2045u + ramp) return 8.0f; if (t < 2800) return 0.3f; if (t < 2810) return 6.0f; return 1.0f; };
      K.arm(true, true, NOW); run(5000); char m[100]; snprintf(m, sizeof m, "vibrating rail (0.18 g rms) and a %d ms ramp into the push: launch seen, auto runs", ramp);
      CHECK(K.flight.launched && ph() == "ANCHORING", m); } }
  { fresh(); ACC = [](uint32_t t) { if (t < 2000) return 1.0f; if (t < 2020) return 2.5f; if (t < 2060) return 8.0f; if (t < 2800) return 0.3f; if (t < 2810) return 6.0f; return 1.0f; };
    K.arm(true, false, NOW); run(5000); CHECK(K.flight.launched, "a single 2.5 g sample before the push does not block the launch"); }
  { fresh(); K.startAuto(NOW); run(20000); CHECK(K.seq == DONE, "anchored"); K.boot(F_BROWNOUT, NOW, false); K.stop(NOW); K.startAuto(NOW); run(20000);
    CHECK(K.fault != F_KICK_JAM && K.seq == DONE, "reboot with the kick planted, then /stop and /auto: no false KICK_JAM (position unknown after a non power-on reset)"); }
  { fresh(); P.kickContact = 2000; P.kickLand = 2600; P.footPaused = 200; K.startAuto(NOW); run(25000); K.startRelease(NOW); run(30000);
    CHECK(K.seq == RELEASED && K.fault == F_NONE, "near ground and a hidden housing stall (plant time limit): release ends 'released', no false KICK_JAM"); }
  { fresh(); for (int i = 0; i < 6; i++) { K.key('2', 350, NOW); run(150); }   // web page: hold, release (keep=1 stop), tap again soon
    K.stop(NOW, false); run(300); CHECK(K.key('2', 350, NOW) == nullptr && P.mdir[KICK] == 1, "page key release (keep=1) then the same key 300 ms later: runs (no lock)"); }
  { fresh(); K.key('2', 350, NOW); run(370); K.stop(NOW); run(30); CHECK(K.key('2', 350, NOW) != nullptr && P.mdir[KICK] == 0, "STOP during a deadman gap still locks the repeating key"); }
  { fresh(); K.startAuto(NOW); run(1000); for (int i = 0; i < 3; i++) { K.key('1', 350, NOW); run(150); }   // phone holds 1 during /auto (refused busy)
    K.stop(NOW); K.key('1', 350, NOW); run(100); CHECK(P.mdir[DEPLOY] == 0, "STOP of a sequence locks a key that was held (refused) during it"); }
  { fresh(); K.startAuto(NOW); while (K.seq != D_DEPLOY) run(5); run(700); K.stop(NOW); run(500); P.deploy = 1000; K.startAuto(NOW); run(25000);
    CHECK(K.seq == DONE && P.deploy >= 1400, "retry after an abort with the pods half down: pods go fully down (no false 'already down')"); }
  { fresh(); ACC = [](uint32_t t) { if (t < 3000) return 1.0f; if (t < 3040) return 8.0f; if (t < 11500) return 0.4f; if (t < 11515) return 6.0f; return 1.0f; };
    K.arm(true, false, NOW); run(9500); K.stop(NOW); run(4000);
    CHECK(K.log.frozen && K.log.at(0).t <= K.flight.tLaunch - 1980, "stop 6.5 s into a long flight: the frozen log still starts 2 s before launch"); }

  // ===================================================================================================== review round 9 regressions
  section("review round 9 regressions");
  { for (double at : {3800.0, 3400.0, 3000.0}) { fresh(); K.startAuto(NOW); run(20000); P.kickJamBackAt = at; K.startRelease(NOW); run(30000);
      char m[100]; snprintf(m, sizeof m, "kick jams on the way back at %.0f of 4300 (stall current): KICK_JAM, not 'released'", at);
      CHECK(std::string(faultName(K.fault)) == "KICK_JAM" && K.seq != RELEASED && allOff(), m); } }
  { fresh(); K.startAuto(NOW); run(20000); K.startRelease(NOW); run(30000); CHECK(K.seq == RELEASED && K.fault == F_NONE, "normal release after a normal anchor: no false KICK_JAM"); }
  { fresh(); K.arm(true, true, NOW); K.key('3', 1500, NOW); run(100); P.shortCircuit = true; run(30); P.shortCircuit = false;
    CHECK(K.fault == F_OVERCURRENT && !K.armed, "a fault while armed disarms (landing cannot clear it and drive motors)");
    ACC = [](uint32_t t) { return flightTrace(t, NOW + 500 > t ? 0 : 0); }; run(100); CHECK(allOff(), "...motors stay off"); }
  { fresh(); K.startAuto(NOW); while (K.seq != D_DEPLOY) run(5); run(400); P.drillStallFwd = true; run(2000);
    int restarts = 0; for (auto& l : LOG) if (l.find("drills stalled") != std::string::npos) restarts++;
    run(10000); restarts = 0; for (auto& l : LOG) if (l.find("drills stalled") != std::string::npos) restarts++;
    CHECK(restarts == 1 && K.seq == DONE, "drills stalled while the pods swing are not driven again in the finishing step"); P.drillStallFwd = false; }
  { fresh(); P.kick = 4300; for (int i = 0; i < 6; i++) { K.key('2', 350, NOW); run(150); }   // kick held into the ground: stall lock
    K.key('@', 350, NOW); run(150); K.stop(NOW, false);                                      // a brief opposite tap
    int on = 0; for (int i = 0; i < 6; i++) { K.key('2', 350, NOW); run(150); if (P.mdir[KICK] > 0) on++; }
    CHECK(on == 0, "a brief opposite-direction tap does not clear the stall lock of the held key"); }
  { fresh(); for (int i = 0; i < 4; i++) { K.key('@', 350, NOW); K.key('2', 350, NOW); run(150); }   // both kick keys being pressed
    K.stop(NOW); K.key('@', 350, NOW); run(50); K.key('2', 350, NOW); run(50);
    CHECK(P.mdir[KICK] == 0, "STOP with both directions of a motor being pressed locks both"); }
  { fresh(); for (int i = 0; i < 6; i++) { K.key('@', 350, NOW); run(150); }   // kick held into stowed: locked '@'
    run(800); for (int i = 0; i < 6; i++) { K.key('2', 350, NOW); run(150); } K.stop(NOW, false); run(100);   // jog out
    bool ok = K.key('@', 350, NOW) == nullptr; run(50);
    CHECK(ok && P.mdir[KICK] == -1, "an old '@' lock does not refuse a fresh '@' right after '2' presses (per-direction lock)"); }
  { fresh(); K.startAuto(NOW); run(20000); K.bothLock[DEPLOY] = true; K.lastCmd[DEPLOY] = NOW; K.key('1', 350, NOW);
    CHECK(K.seq == DONE, "a refused key does not change seq (still anchored)"); }

  // ===================================================================================================== review round 10
  section("review round 10 regressions");
  { fresh(); P.kickContact = P.kickLand = 4300; bool augers = false; K.startAuto(NOW);   // landed on its back: kick swings through air
    for (int i = 0; i < 2000 && K.seq != IDLE && !K.fault; i++) { run(5); if (P.mdir[FOOT] || P.mdir[DRILLS] || P.mdir[DEPLOY]) augers = true; }
    CHECK(std::string(faultName(K.fault)) == "NO_GROUND" && !augers && allOff(), "kick swings its whole travel through air to its end stop: NO_GROUND, foot, pods and drills never driven");
    K.startRelease(NOW); run(30000); CHECK(K.seq == RELEASED && P.kick == 0 && K.fault == F_NONE, "...and /release brings the kick back (released)"); }
  { fresh(); P.footPaused = 75; P.kickLand = 5800; K.startAuto(NOW);   // the foot draws 55 mA less while the kick pauses
    while (K.seq != K_PLANT && NOW < 20000) run(5); while (K.seq == K_PLANT && NOW < 30000) run(5);
    printf("     plant ended with the kick at %.0f of 5800\n", P.kick);
    CHECK(P.kick >= 5700, "a foot current drop of 55 mA in the kick's pause does not fake a kick stall (kick reaches the housing)"); run(20000);
    CHECK(K.seq == DONE, "...and the anchor completes"); }
  { fresh(); for (int i = 0; i < 15; i++) { K.key('2', 350, NOW); run(150); }   // kick out held, then reversed with no stop between
    int on = 0; for (int i = 0; i < 8; i++) { K.key('@', 350, NOW); run(150); if (P.mdir[KICK] < 0) on++; }
    CHECK(on == 8 && !K.bothLock[KICK], "reversing a held key without a stop reverses the motor (no 'both directions held' latch)"); }
  { fresh(); for (int i = 0; i < 6; i++) { K.key('2', 350, NOW); K.key('@', 350, NOW); run(150); }
    CHECK(P.mdir[KICK] == 0 && K.bothLock[KICK], "both kick keys repeated together still lock the motor"); }
  { fresh(); K.startAuto(NOW); run(20000); CHECK(K.seq == DONE && K.arm(true, false, NOW) != nullptr && !K.armed, "arm (even log-only) is refused while ANCHORED"); }
  // review round 12: the firmware measures the kick swing for BENCH_TEST step 5
  { fresh(); P.kickContact = P.kickLand = 4300; P.railV = 8.0f; for (int i = 0; i < 60 && !when("kick travel from stowed"); i++) { K.key('2', 350, NOW); run(150); }
    std::string e; for (auto& l : LOG) if (l.find("kick travel from stowed") != std::string::npos) e = l;
    float ms = e.empty() ? 0 : atof(e.c_str() + e.find("stowed") + 7); printf("     %s\n", e.c_str());
    CHECK(ms > 4300 * 0.95f && ms < 4300 * 1.05f, "a kick held out in air to its end stop reports its travel (within 5 % of the real 4300 ms)"); }
  // review round 12 (logic)
  auto augerRun = [&](int maxTicks) { bool a = false; for (int i = 0; i < maxTicks && K.seq != IDLE && !K.fault; i++) { run(5); if (P.mdir[FOOT] || P.mdir[DRILLS] || P.mdir[DEPLOY]) a = true; } return a; };
  { fresh(); P.kickContact = P.kickLand = 1500; K.startAuto(NOW); bool a = augerRun(3000);   // jams in air at 35 % of the swing
    CHECK(std::string(faultName(K.fault)) == "KICK_JAM" && !a, "a kick jammed part way out in air: KICK_JAM, not 'already down' (augers never run)"); }
  { fresh(); P.kickContact = P.kickLand = 4300; K.startAuto(NOW); augerRun(3000); K.stop(NOW); run(700);
    for (int i = 0; i < 3; i++) { K.key('2', 350, NOW); run(150); } run(600); K.startAuto(NOW); bool a = augerRun(3000);
    CHECK(std::string(faultName(K.fault)) == "NO_GROUND" && !a, "after NO_GROUND, a '2' tap at the end stop does not make the next /auto anchor in air"); }
  { fresh(); K.boot(F_NONE, NOW, false); P.kickContact = P.kickLand = 4300; K.startAuto(NOW); augerRun(3000); K.stop(NOW); run(700);
    K.startAuto(NOW); bool a = augerRun(3000);
    CHECK(std::string(faultName(K.fault)) == "NO_GROUND" && !a, "travel unknown (after a reset): a retry after NO_GROUND is NO_GROUND again"); }
  { fresh(); for (int i = 0; i < 25; i++) { K.key('@', 350, NOW); run(150); } run(700);   // '@' hold ends stalled, then '2' and '@' again within the cap
    const char* e = nullptr; for (int i = 0; i < 60 && !e; i++) { e = K.key(i < 30 ? '2' : '@', 350, NOW); run(150); }
    CHECK(e && std::string(e).rfind("hold limit", 0) == 0, "the 8 s hold cap refuses with 'hold limit', not 'stalled' (a client can tell it from an end stop)"); }

  // review round 13 (whole logic)
  for (double at : {1300.0, 1000.0, 700.0}) {   // pods jam on the way up before 60 % of the way (the last 40 % is a documented limit)
    fresh(); K.startAuto(NOW); run(20000); P.deployJamBackAt = at; K.startRelease(NOW); run(30000);
    char m[120]; snprintf(m, sizeof m, "pods jam at %.0f of 1400 on the way up: PODS_JAM, not 'legs stowed'", at);
    CHECK(std::string(faultName(K.fault)) == "PODS_JAM" && K.seq != RELEASED && allOff(), m); }
  { fresh(); K.startAuto(NOW); run(20000); K.startRelease(NOW); run(30000); CHECK(K.seq == RELEASED && K.fault == F_NONE, "a normal release still ends released (no false PODS_JAM)"); }
  for (double at : {1600.0, 1700.0}) {   // kick jams coming back at 37 to 40 % of the real return
    fresh(); K.startAuto(NOW); run(20000); P.kickJamBackAt = 4300 - at; K.startRelease(NOW); run(30000);
    char m[120]; snprintf(m, sizeof m, "kick jams %.0f ms into its 4300 ms return: KICK_JAM (travel counted only up to a reading under the line)", at);
    CHECK(std::string(faultName(K.fault)) == "KICK_JAM" && K.seq != RELEASED, m); }
  { fresh(); K.startAuto(NOW); while (K.seq != D_DEPLOY) run(5); run(1500); K.stop(NOW); run(700);
    CHECK(K.arm(true, true, NOW) != nullptr && !K.armed, "an anchor stopped with the pods down: arm refused ('legs out: release first')"); }
  { fresh(); uint32_t t0 = 2000; ACC = [&](uint32_t t) { return flightTrace(t, t0); }; K.arm(true, false, NOW); run(8000); K.startAuto(NOW); run(20000);
    K.startRelease(NOW); run(30000); ACC = [](uint32_t) { return 1.0f; }; K.key('1', 100, NOW); run(300);
    CHECK(ph() == "IDLE", "after a release, a key tap does not bring back phase LANDED"); }
  { fresh(); K.startAuto(NOW); for (int i = 0; i < 200 && K.seqRunning(); i++) { K.key('$', 350, NOW); run(150); }
    run(20000); K.stop(NOW, false); const char* e = nullptr; for (int i = 0; i < 3; i++) { e = K.key('$', 350, NOW); run(150); }
    CHECK(e == nullptr || std::string(e).rfind("hold limit", 0) != 0, "a key held through a sequence starts a fresh hold afterwards (no instant 'hold limit')"); }
  { fresh(); K.key('@', 350, NOW); run(500); NOW += 25u * 24 * 3600 * 1000; run(100); K.stop(NOW); run(700); K.stop(NOW, false); P.kick = 2000;
    run(10); const char* e = K.key('@', 350, NOW); run(150);
    CHECK(e == nullptr, "a key pressed 25 days ago does not lock that key at the next stop (old presses expire)"); }

  // coordinator review r2_B2 #5
  { fresh(); K.startAuto(NOW); run(500); CHECK(K.key('3', 350, NOW) != nullptr, "a run key during ANCHORING is refused (busy)");
    K.keyRelease(NOW); run(100); CHECK(K.anchoring() && P.mdir[KICK] > 0, "...and the page's key-release stop does not abort the anchor"); }
  { fresh(); K.key('3', 350, NOW); run(100); K.keyRelease(NOW); CHECK(allOff() && K.seq == IDLE, "the key-release stop stops a manual motor"); }
  // review round 11
  { fresh(); P.kickContact = P.kickLand = 4300; K.startAuto(NOW); run(10000); K.stop(NOW); run(700);   // NO_GROUND, then Retry
    bool augers = false; K.startAuto(NOW);
    for (int i = 0; i < 2000 && K.seq != IDLE && !K.fault; i++) { run(5); if (P.mdir[FOOT] || P.mdir[DRILLS] || P.mdir[DEPLOY]) augers = true; }
    CHECK(std::string(faultName(K.fault)) == "NO_GROUND" && !augers, "a retried /auto after NO_GROUND is NO_GROUND again (augers never run in air)"); }
  { fresh(); P.kickContact = P.kickLand = 4300; K.startAuto(NOW); run(2000); K.stop(NOW); run(700);   // swing stopped half way, then retried
    bool augers = false; K.startAuto(NOW);
    for (int i = 0; i < 2000 && K.seq != IDLE && !K.fault; i++) { run(5); if (P.mdir[FOOT] || P.mdir[DRILLS] || P.mdir[DEPLOY]) augers = true; }
    CHECK(std::string(faultName(K.fault)) == "NO_GROUND" && !augers, "an /auto stopped half way through the air and retried: NO_GROUND"); }
  { fresh(); K.startAuto(NOW); while (K.seq != D_DEPLOY) run(5); run(1000); K.stop(NOW); run(700); K.startAuto(NOW); run(25000);
    CHECK(K.seq == DONE && K.fault == F_NONE, "a retried /auto after a real plant still takes 'kick already down' (no false NO_GROUND)"); }
  { fresh(); P.deploy = P.deployEnd; uint32_t drivenAtStop = 0;   // pods at their down stop, keys alternated every 300 ms
    for (int i = 0; i < 60; i++) { K.key(i % 2 ? '!' : '1', 350, NOW); for (int k = 0; k < 60; k++) { run(5); if (P.mdir[DEPLOY] > 0) drivenAtStop += 5; } }
    printf("     alternating 1 / ! for 18 s: driven into the stop %u ms\n", drivenAtStop);
    CHECK(drivenAtStop <= 8500, "alternating 1 and ! every 300 ms: the hold cap counts per motor, so the stop is not driven without limit"); }


  // ===================================================================================================== v4.1
  section("v4.1: short flights (real launcher), orientation, battery at rest, video switch");
  { fresh(); CHECK(VIDEO_PIN == 1 && K.video, "boot switches the video on (D10 high)"); }
  // the real launcher: 0.37 s flight at the 6 deg low aim, about 0.42 s from the start of the push to the impact
  for (uint32_t fl : {330u, 370u, 600u}) {
    fresh(); uint32_t t0 = 2000;
    ACC = [&](uint32_t t) { if (t < t0) return 1.0f; uint32_t d = t - t0; if (d < 50) return 8.0f; if (d < 50 + fl) return 0.3f; if (d < 60 + fl) return 7.0f; return 1.0f; };
    K.arm(true, false, NOW); run(t0 + 50 + fl + 1500 - NOW);
    char m[96]; snprintf(m, sizeof m, "%u ms flight after a 50 ms push: impact landing (not the 'still' fallback)", fl);
    CHECK(K.landed && std::string(K.flight.how) == "impact" && K.flight.tLand >= t0 + 50 + fl - 5 && K.flight.tLand <= t0 + 60 + fl, m);
  }
  { fresh(); uint32_t t0 = 2000;   // tumbling (never under 0.6 g), 0.35 s flight: the push-over rule must make it quiet in time
    ACC = [&](uint32_t t) { if (t < t0) return 1.0f; uint32_t d = t - t0; if (d < 45) return 8.0f; if (d < 395) return 1.4f; if (d < 405) return 7.0f; return 1.0f; };
    K.arm(true, true, NOW); run(5000);
    CHECK(std::string(K.flight.how) == "impact" && (K.anchoring() || K.seq == DONE), "tumbling 0.35 s flight: impact landing starts the auto anchor"); }
  { fresh(); uint32_t t0 = 2000;   // a 20 ms knock while armed is a launch, but a 'bounce' 150 ms later is not an impact
    ACC = [&](uint32_t t) { if (t < t0) return 1.0f; uint32_t d = t - t0; if (d < 25) return 8.0f; if (d < 150) return 0.3f; if (d < 160) return 7.0f; return 1.0f; };
    K.arm(true, true, NOW); run(5000);
    CHECK(std::string(K.flight.how) != "impact" && !K.anchoring() && K.seq == IDLE, "an 'impact' 150 ms after launch (under minFlightMs 250) is not a landing impact: no auto"); }
  // orientation
  { fresh(); ACC = [](uint32_t) { return 1.0f; }; run(500);
    char b[1400]; K.status(b, sizeof b, 0, 0, NOW, 7.6f, "poweron"); std::string s = b;
    CHECK(K.orient(NOW) == O_UPRIGHT && s.find("\"orient\":\"upright\"") != std::string::npos && s.find("\"tilt_deg\":0") != std::string::npos, "belly down: orient upright, tilt 0 in /status"); }
  { fresh(); GRAV[0] = 9.81f; GRAV[2] = 0; run(500);   // on its side
    const char* r = K.startAuto(NOW);
    CHECK(K.orient(NOW) == O_SIDE && r && std::string(r).find("not upright: on its side") == 0 && K.seq == IDLE && allOff(), "on its side: /auto refused 'not upright: on its side', nothing moves");
    r = K.startAuto(NOW, true);
    CHECK(!r && K.seq == K_SWING, "on its side: /auto?force=1 starts anyway"); }
  { fresh(); GRAV[2] = -9.81f; run(500);
    const char* r = K.startAuto(NOW);
    CHECK(K.orient(NOW) == O_INVERTED && r && std::string(r).find("not upright: upside down") == 0, "upside down: /auto refused 'not upright: upside down'"); }
  { fresh(); GRAV[0] = -9.81f * 0.866f; GRAV[2] = 9.81f * 0.5f; run(500);   // nose buried at 60 deg: still counts as upright
    CHECK(K.orient(NOW) == O_UPRIGHT && !K.startAuto(NOW), "nose down 60 deg (buried nose): upright, /auto runs"); }
  { fresh(); K.imuOk = false; GRAV[2] = -9.81f; run(500);
    CHECK(K.orient(NOW) == O_UNKNOWN && !K.startAuto(NOW), "BNO085 silent: orientation unknown, /auto is not blocked by it"); }
  { fresh(); K.tu.imuUpAxis = -1; GRAV[0] = -9.81f; GRAV[2] = 0; run(500);   // a board mounted with chip -x up (BENCH_TEST step 7)
    CHECK(K.orient(NOW) == O_UPRIGHT, "imuUpAxis -1: chip -x up reads upright"); }
  { fresh(); uint32_t t0 = 2000;   // lands on its side after an impact: auto on landing does not start
    ACC = [&](uint32_t t) { return flightTrace(t, t0); };
    K.arm(true, true, NOW); run(1000 + t0 - NOW);
    for (int i = 0; i < 1000 && !K.flight.flying(); i++) run(5);
    GRAV[0] = 0; GRAV[1] = -9.81f; GRAV[2] = 0; run(4000);
    CHECK(K.landed && std::string(K.flight.how) == "impact" && K.seq == IDLE && allOff() && ph() == "LANDED" && logged("landed on its side: auto not started"),
          "impact landing on its side: auto not started, phase LANDED, event says why"); }
  // battery at rest
  { fresh(); P.railV = 6.7f; run(1500);
    CHECK(K.batt == B_LOW && logged("battery low") && !K.startAuto(NOW), "6.7 V at rest: battery low warning, /auto still runs"); }
  { fresh(); P.railV = 6.3f; run(1500); const char* r = K.startAuto(NOW);
    CHECK(K.batt == B_CRIT && r && std::string(r).find("battery low") == 0 && K.seq == IDLE, "6.3 V at rest: battery critical, /auto refused 'battery low'");
    CHECK(!K.video && VIDEO_PIN == 0 && logged("video off (battery critical)"), "battery critical switches the video off");
    CHECK(!K.startRelease(NOW) && K.seq == R_DRILL, "battery critical: /release still runs"); }
  { fresh(); P.railV = 6.3f; run(1500); P.railV = 6.45f; run(1500);
    CHECK(K.batt == B_CRIT, "hysteresis: 6.45 V after critical stays critical (0.1 V)"); P.railV = 6.6f; run(1500);
    CHECK(K.batt == B_LOW, "6.6 V after critical: low"); P.railV = 7.4f; run(1500); CHECK(K.batt == B_OK && logged("battery ok"), "7.4 V: ok again"); }
  { fresh(); K.startAuto(NOW); P.railV = 6.0f; run(300);   // sag under load: not a resting reading
    CHECK(K.batt == B_OK || K.batt == B_UNKNOWN, "a sag while a motor runs is not counted as the resting battery"); }
  { fresh(); P.railV = 0.0f; run(200); CHECK(K.batt == B_NONE, "rail 0 V (plug out, USB only): batt none"); }
  // video
  { fresh(); K.setVideo(false, "test"); CHECK(!K.video && VIDEO_PIN == 0, "/video?on=0 switches it off");
    K.arm(true, false, NOW); CHECK(K.video && VIDEO_PIN == 1 && logged("video on (armed"), "arming switches the video back on (the flight is filmed)"); }
  { fresh(); std::string s = statusJson();
    CHECK(s.size() < 1000 && s.find("\"video\":true") != std::string::npos && s.find("\"batt\":") != std::string::npos, "status JSON has video, batt, batt_V, orient, tilt_deg and stays under 1000 bytes"); }

  // ===================================================================================================== whole mission
  section("whole mission phases");
  { fresh(); uint32_t t0 = 2000; ACC = [&](uint32_t t) { return flightTrace(t, t0); }; PHASES.clear();
    PHASES.push_back(ph()); K.arm(true, true, NOW); runWatch(25000); K.startRelease(NOW); runWatch(25000);
    CHECK(phases() == "IDLE>ARMED>FLIGHT>ANCHORING>ANCHORED>RELEASING>IDLE", "IDLE > ARMED > FLIGHT > ANCHORING > ANCHORED > RELEASING > IDLE");
    std::string s = statusJson(); save("status_released.json", s);
    fresh(); K.boot(F_BROWNOUT, NOW); save("status_fault.json", statusJson()); }

  printf("\n%d checks, %s\n", checks, fails ? "FAILURES" : "ALL PASS");
  if (fails) printf("%d FAILED\n", fails);
  return fails ? 1 : 0;
}
