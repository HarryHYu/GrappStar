// PC simulation of the KEDGE control logic against a simple plant model.  g++ -std=c++17 -I../kedge_dart test_logic.cpp && ./a.out
#include "kedge_logic.h"
#include <cstdio>
#include <cassert>
#include <string>
#include <vector>
using namespace kedge;
static int mdir[4]; static uint32_t NOW; static uint32_t runT[4];   // time each motor has actually been powered
static std::vector<std::string> LOG;
struct Plant { uint32_t kickContact = 3000, kickLand = 4300, deployStall = 1400; bool shortCircuit = false; } PL;
static float motorMa(int m) {
  if (!mdir[m]) return 0;
  switch (m) {
    case KICK:   return runT[KICK] < PL.kickContact ? 25 : runT[KICK] < PL.kickLand ? 70 : 185;
    case FOOT:   return runT[KICK] >= PL.kickContact ? 130 : 40;
    case DEPLOY: return runT[DEPLOY] < PL.deployStall ? 30 : 185;
    case DRILLS: return mdir[DEPLOY] || runT[DEPLOY] >= PL.deployStall ? 420 : 140;
  }
  return 0;
}
static float cur() { float s = 0; for (int m = 0; m < 4; m++) s += motorMa(m); return PL.shortCircuit ? 3000 : s; }
static void setM(int m, int d, int duty) { mdir[m] = duty ? d : 0; }
static float rail() { return 7.6f; }
static void lg(const char* s) { char b[128]; snprintf(b, sizeof b, "%6u ms  %s", NOW, s); LOG.push_back(b); }
static Kedge make() { Kedge k; k.io = {setM, cur, rail, lg}; for (int m = 0; m < 4; m++) { mdir[m] = 0; runT[m] = 0; } NOW = 0; LOG.clear(); return k; }
static bool DBG = false;
static void run(Kedge& k, uint32_t ms) { for (uint32_t i = 0; i < ms; i += 5) { NOW += 5; for (int m = 0; m < 4; m++) if (mdir[m]) runT[m] += 5; k.tick(NOW);
  if (DBG && k.seq == D_DEPLOY && NOW % 50 == 0) printf("  t=%u ma=%.0f paused=%d solo=%.0f hits=%d mdir=%d%d%d%d runTdep=%u\n", NOW, k.lastMa, k.paused, k.lastSolo, k.stallHits, mdir[0],mdir[1],mdir[2],mdir[3], runT[0]); } }
static int fails = 0;
#define CHECK(c, msg) do { if (!(c)) { printf("FAIL: %s\n", msg); fails++; } else printf("ok:   %s\n", msg); } while (0)
static uint32_t when(const char* s) { for (auto& l : LOG) if (l.find(s) != std::string::npos) return (uint32_t)atoi(l.c_str()); return 0; }
int main() {
  { Kedge k = make(); k.startAuto(NOW); run(k, 20000);
    for (auto& l : LOG) printf("   %s\n", l.c_str());
    CHECK(k.seq == DONE, "auto sequence reaches 'anchored'");
    uint32_t tf = when("foot touches"), tk = when("kick down"), td = when("deploy with"), tp = when("pods down"), ta = when("anchored");
    CHECK(tf >= 3000 && tf < 3400, "foot auger starts when the foot meets the ground (kick current rise)");
    CHECK(tk >= 4300 && tk < 5800, "kick stops after the housing lands (stall read with the foot paused)");
    CHECK(td > tk, "deploy starts only after the foot is planted");
    CHECK(tp >= 1400 + td && tp < td + 3000, "deploy stops on its own stall even though the drills draw more current");
    CHECK(ta > tp, "drills finish after the pods stop");
    bool allOff = true; for (int m = 0; m < 4; m++) allOff &= mdir[m] == 0; CHECK(allOff, "everything off at the end"); }
  { Kedge k = make(); k.startAuto(NOW); run(k, 1000); k.key('1', 1500, NOW); CHECK(mdir[DEPLOY] == 0, "manual keys are ignored while auto runs");
    k.key(' ', 0, NOW); run(k, 10); bool off = true; for (int m = 0; m < 4; m++) off &= mdir[m] == 0; CHECK(off && k.seq == ABORTED, "space aborts the auto sequence and stops everything"); }
  { Kedge k = make(); k.key('3', 1500, NOW); run(k, 100); PL.shortCircuit = true; run(k, 30); PL.shortCircuit = false; CHECK(mdir[DRILLS] == 0, "over-current trip stops the motors"); }
  { Kedge k = make(); for (int i = 0; i < 10; i++) { k.key('2', 350, NOW); run(k, 150); } CHECK(mdir[KICK] == 1, "WiFi hold: repeats keep the kick running");
    run(k, 400); CHECK(mdir[KICK] == 0, "WiFi hold: key released -> stops within the 350 ms deadman"); }
  { Kedge k = make(); k.key('4', 1500, NOW); run(k, 1400); CHECK(mdir[FOOT] == 1, "serial key runs 1.5 s"); run(k, 200); CHECK(mdir[FOOT] == 0, "serial key stops after 1.5 s"); }
  { Kedge k = make(); PL.kickContact = 99999; k.startAuto(NOW); run(k, 17000); CHECK(k.seq == ABORTED, "kick never meets the ground -> aborts at the time limit, nothing left running"); PL.kickContact = 3000; }
  { Landing L; uint32_t t = 0; bool landed = false; auto feed = [&](float a, uint32_t ms) { for (uint32_t i = 0; i < ms; i += 10) { t += 10; landed |= L.update(a, t); } };
    feed(1.0, 500); feed(12, 60); feed(0.3, 900); feed(15, 30); feed(1.4, 200); feed(1.0, 1600); CHECK(landed, "landing detector: launch spike, flight, impact, stillness -> landed");
    Landing L2; t = 0; landed = false; for (int i = 0; i < 300; i++) { t += 10; landed |= L2.update(1.0f + (i % 7 == 0 ? 0.3f : 0), t); } CHECK(!landed, "landing detector: bench handling alone never triggers"); }
  { Landing L; uint32_t t = 0; bool landed = false; auto feed = [&](float a, uint32_t ms) { for (uint32_t i = 0; i < ms; i += 10) { t += 10; landed |= L.update(a, t); } };
    feed(1.0, 300); feed(12, 40); feed(0.2, 800); feed(1.0, 1700); CHECK(landed, "landing detector: soft landing with the impact spike missed still triggers");
    L.reset(); landed = false; feed(1.0, 3000); CHECK(!landed && L.state == 0, "landing detector: reset re-arms it (A pressed again)"); }
  { Kedge k = make(); char b[400]; k.status(b, sizeof b, 1.5, -3.2, false, 1234); printf("   status: %s\n", b); CHECK(std::string(b).find("\"seq\":\"idle\"") != std::string::npos, "status JSON"); }
  printf(fails ? "\n%d FAILED\n" : "\nALL PASS\n", fails);
  return fails ? 1 : 0;
}
