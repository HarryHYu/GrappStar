// Randomized property test (fuzzer) for the GrappStar base firmware logic, against the simulation in sim.h.
// For each seed: a fresh rig (sim + gb::Base), then a few hundred random steps mixing valid and invalid command lines,
// time, fault injection in the sim, power cuts (Base rebuilt from the saved record, sim kept) and complete missions
// (launch: mode, aim, cock, arm, fire; reel: wrap, reel, unwrap) so the firing and reel paths really run.
// Invariants are checked after every command and every 5 ms tick (see check* below).
//
// Build and run:  g++ -std=c++17 -O1 -g -fsanitize=address,undefined -I. -o test_fuzz test_fuzz.cpp && ./test_fuzz [seeds] [first]
// Replay one seed with the whole log:  FUZZ_LOG=1 ./test_fuzz 1 <seed>   (FUZZ_TRACE=1 adds firmware vs sim truth)
// Exit code 1 when any invariant was violated.
#include <stdio.h>
#include <stdlib.h>
#include <deque>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <vector>
#include "sim.h"

Sim S;

namespace {

using namespace gb;

// ------------------------------------------------------------------------------------------------ state of one run
std::mt19937_64 rng;
uint32_t seedNow = 0;
std::unique_ptr<Base> B;
Tun tunStore; bool tunHas = false;
void fuzzSaveTun(const Tun& t) { tunStore = t; tunHas = true; }

std::deque<std::string> logRing;          // last commands, replies, events, injections
bool verbose = false, trace = false;
void logl(const std::string& s) {
  char p[32]; snprintf(p, sizeof p, "[%7u] ", (unsigned)S.t);
  std::string l = p + s;
  if (verbose) printf("  %s\n", l.c_str());
  logRing.push_back(l); if (logRing.size() > 30) logRing.pop_front();
}
uint32_t pendingAdvance = 0;              // time advances are folded into one log line
void flushAdvance() { if (pendingAdvance) { char b[48]; snprintf(b, sizeof b, "+%u ms", (unsigned)pendingAdvance); pendingAdvance = 0; logl(b); } }

std::set<std::string> tags;               // hazards present in this seed (stuck contact, forced rest, miscalibration...)

// ------------------------------------------------------------------------------------------------ violations
struct Kind { int count = 0; std::vector<uint32_t> seeds; };
std::map<std::string, Kind> viol;
int stepNow = 0;
// a violation in a seed where a sensor was lying or the base was miscalibrated is filed apart ("[hazard]"): the firmware
// cannot always see through those, the clean ones are firmware bugs
void violation(const std::string& kind0, const std::string& detail) {
  bool proto = kind0.rfind("BAD_", 0) == 0 || kind0.rfind("ID_", 0) == 0 || kind0.rfind("REPLY_", 0) == 0;
  std::string kind = kind0 + (tags.empty() || proto ? "" : " [hazard]");
  Kind& k = viol[kind];
  k.count++;
  if (k.seeds.empty() || k.seeds.back() != seedNow) { if (k.seeds.size() < 12) k.seeds.push_back(seedNow); }
  if (k.count == 1) {
    flushAdvance();
    std::string tg; for (auto& t : tags) tg += t + " ";
    printf("\n*** VIOLATION %s (seed %u, step %d, t=%u ms): %s\n    hazards in this seed: %s\n    state %s phase %s fault %s, sim drum %.2f deg cockPh %.1f fired %d dogTip %d wDir %d dDir %d\n    last 30 lines:\n",
           kind.c_str(), (unsigned)seedNow, stepNow, (unsigned)S.t, detail.c_str(), tg.empty() ? "none" : tg.c_str(),
           STATE_NAME[B->state], PHASE_NAME[B->phase], B->fault ? FAULT_NAME[B->fault] : "-", S.drumDeg, S.cockPh, (int)S.fired, (int)S.dogTip, S.wDir, S.dDir);
    for (auto& l : logRing) printf("      %s\n", l.c_str());
  }
}

// ------------------------------------------------------------------------------------------------ JSON line checks
bool validLine(const char* l, const char* what, bool logIt) {
  size_t n = strlen(l);
  JObj o;
  bool ok = n > 0 && n <= 256 && l[0] == '{' && !strchr(l, '\n') && jsonParse(l, (int)n, &o);
  if (!ok) violation(std::string("BAD_") + what, std::string("len ") + std::to_string(n) + ": " + std::string(l).substr(0, 300));
  return ok;
}
size_t outSeen = 0;
void scanEvents() {
  for (; outSeen < S.out.size(); outSeen++) {
    const std::string& e = S.out[outSeen];
    flushAdvance(); logl("evt " + e);
    validLine(e.c_str(), "EVENT", true);
  }
  if (S.out.size() > 64) { S.out.clear(); outSeen = 0; }
}

// ------------------------------------------------------------------------------------------------ invariants per tick
bool fireAuth = false;                    // the base is in FIRE because a fire command with a valid token was accepted
void checkStates(const char* where) {
  int st = B->state;
  if (st != S_FIRE) fireAuth = false;
  else if (!fireAuth) violation("FIRE_WITHOUT_TOKEN", std::string("state FIRE without an accepted fire command (") + where + ")");
}
void tick() {
  int prevState = B->state;
  int shots0 = S.shots;
  S.step(5);
  // the drive inputs in force during this step were set by the base in prevState
  if (S.shots > shots0 && prevState != S_FIRE)
    violation("SHOT_OUTSIDE_FIRE", std::string("sim fired while the base was in ") + STATE_NAME[prevState]);
  if (S.dryFires) { violation("DRY_FIRE", "the collar left the LAUNCH dwell with the spring drawn"); S.dryFires = 0; }
  // unwrapping without the take-up leaves loose line between the reel and the capstan
  if (S.slack > 0.15) { violation("UNWRAP_SLACK", "loose line " + std::to_string(S.slack) + " m (unwrap without take-up)"); S.slack = 0; }
  // backward with the REEL dogs engaged is only wrong once the collar presses the backward flank (sprag #2 holds the
  // capstan): the centering jog moves inside the free dog window by design
  if (S.wDir == -S.c.reelDir && S.wDuty > 0 && S.inReel() && S.win <= 1e-6)
    violation("BACKWARD_IN_REEL", "wiper driven backward with the REEL dogs engaged (true drum " + std::to_string(S.drumDeg) + ")");
  B->tick(S.t);
  if (S.saveSeq != B->saveDoneSeq && (int32_t)(S.t - S.saveT) >= 20) B->saveDoneSeq = S.saveSeq;   // NVS write done
  int st = B->state;
  if ((st == S_IDLE || st == S_HOLD || st == S_FAULT) && (S.wDir != 0 || S.dDir != 0))
    violation("MOTOR_IN_REST_STATE", std::string("state ") + STATE_NAME[st] + " wDir " + std::to_string(S.wDir) + " dDir " + std::to_string(S.dDir));
  if ((st == S_FAULT || st == S_SAFING) && S.wDir != 0)
    violation("WIPER_AFTER_FAULT", std::string("wiper still driven in ") + STATE_NAME[st]);
  checkStates("tick");
  if (trace && (S.wDir || S.dDir) && S.t % 50 == 0)
    printf("    trace t=%u %s %s fw: cock %.1f aim %.1f enc %.1f drum %.1f sec %d | sim: cockPh %.1f aimPh %.1f wiper %.1f drum %.2f sw 0x%02x win %.1f wDir %d\n", (unsigned)S.t,
           STATE_NAME[B->state], PHASE_NAME[B->phase], B->cockPhase, B->aimPhase, B->enc.deg(), B->drumDeg, B->sec, S.cockPh, S.aimPh, S.wiperDeg, S.drumDeg, S.contactsRaw(), S.win, S.wDir);
  scanEvents();
  if (B->telDue(S.t)) { char tel[300]; B->telemetry(tel, sizeof tel); validLine(tel, "TELEMETRY", false); }
}
void run(uint32_t ms) { for (uint32_t i = 0; i < ms; i += 5) tick(); }
bool resting() { return B->state == S_IDLE || B->state == S_FAULT || B->state == S_HOLD; }
bool waitRest(uint32_t maxMs) { for (uint32_t i = 0; i < maxMs; i += 5) { tick(); if (resting()) return true; } return false; }

// ------------------------------------------------------------------------------------------------ commands
const std::vector<std::string> calibKeys = {"amps_per_volt", "offset_a", "hx_n_per_count", "drum_dps", "aim_home_deg", "trip_a", "enc_sign"};
std::string lastReply;
uint32_t lastToken = 0; bool haveToken = false;
std::string cmd(const std::string& line) {
  flushAdvance();
  logl("> " + (line.size() > 120 ? line.substr(0, 120) + "...(" + std::to_string(line.size()) + " bytes)" : line));
  char rep[300]; memset(rep, 0x55, sizeof rep); rep[sizeof rep - 1] = 0;
  B->handleLine(line.c_str(), (int)line.size(), rep, sizeof rep);
  std::string r(rep);
  logl("< " + r);
  // exactly one reply line, valid JSON, at most 256 bytes, numeric ids echoed
  if (validLine(rep, "REPLY", true)) {
    JObj ro; jsonParse(rep, (int)strlen(rep), &ro);
    bool okv; if (!ro.boolean("ok", &okv)) violation("REPLY_NO_OK", r);
    if (line.size() <= 256) {
      JObj in;
      if (jsonParse(line.c_str(), (int)line.size(), &in)) {
        const JVal* id = in.find("id");
        const JVal* rid = ro.find("id");
        if (!rid) violation("REPLY_NO_ID", r);
        else if (id && id->t == J_NUM && !(id->n == floor(id->n) && fabs(id->n) < 9007199254740992.0)) {
          if (r.find("\"err\":\"BAD_ID\"") == std::string::npos) violation("INEXACT_ID_NOT_REFUSED", r);   // PROTOCOL: BAD_ID
        }
        else if (id && id->t == J_NUM) {
          if (rid->t != J_NUM) violation("ID_NOT_ECHOED", "sent id " + std::to_string(id->n) + ", reply " + r);
          else {
            bool integ = id->n == floor(id->n) && fabs(id->n) < 1e15;
            double tol = integ ? 0 : fmax(1e-6, fabs(id->n) * 1e-9);
            if (fabs(rid->n - id->n) > tol) {
              char b[400]; snprintf(b, sizeof b, "sent id %.17g, echoed %.17g: %s", id->n, rid->n, r.c_str());
              violation(integ ? "ID_NOT_ECHOED" : "ID_NOT_ECHOED_NONINTEGER", b);
            }
          }
        }
      }
    }
    if (okv) {
      const char* p = strstr(rep, "\"token\":");
      if (p) { lastToken = (uint32_t)strtoul(p + 8, nullptr, 10); haveToken = true; }
    }
    // a fire accepted with a valid token authorises the FIRE state
    JObj in; const char* c = nullptr;
    if (okv && line.size() <= 256 && jsonParse(line.c_str(), (int)line.size(), &in) && in.str("cmd", &c)) {
      if (!strcmp(c, "fire")) fireAuth = true;
      const char* key = nullptr; bool force = false;
      if (!strcmp(c, "set") && in.str("key", &key))
        for (auto& ck : calibKeys) if (ck == key) tags.insert("miscal:" + ck);   // calibration values changed at random
      if (!strcmp(c, "cal") && in.boolean("force", &force) && force) tags.insert("forced_rest");
    }
  }
  lastReply = r;
  checkStates("command");
  scanEvents();
  return r;
}
bool replyOk() { return lastReply.find("\"ok\":true") != std::string::npos; }

// ------------------------------------------------------------------------------------------------ random helpers
int ri(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
double rd(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
bool pr(double p) { return rd(0, 1) < p; }
template <class T> const T& pick(const std::vector<T>& v) { return v[ri(0, (int)v.size() - 1)]; }
std::string fmtd(double v) { char b[64]; snprintf(b, sizeof b, "%.17g", v); return b; }
std::string fmtShort(double v) { char b[64]; snprintf(b, sizeof b, "%.4g", v); return b; }

long idCounter = 0;
std::string randString(int n) {
  static const char cs[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 _-.:{}[],";
  std::string s; for (int i = 0; i < n; i++) s += cs[ri(0, (int)sizeof cs - 2)]; return s;
}
std::string idJson() {
  int k = ri(0, 30);
  if (k < 14) return std::to_string(++idCounter);
  switch (k) {
    case 14: return std::to_string(ri(-1000, 1000));
    case 15: return "0";
    case 16: return "-0";
    case 17: return "999999999999999";          // largest integer echoed as integer
    case 18: return "1e15";
    case 19: return fmtShort(rd(-1000, 1000));
    case 20: return pick(std::vector<std::string>{"1e300", "-1e300", "1e-7", "-1e-7", "123456789.123456789", "4.5e20", "1.5"});
    case 21: return "\"" + randString(ri(0, 20)) + "\"";
    case 22: return "\"" + randString(ri(20, 70)) + "\"";
    case 23: return "\"a\\\"b\\\\c\\u00e9\\n\"";
    case 24: return "null";
    case 25: return pr(0.5) ? "true" : "false";
    case 26: return "{\"x\":[1,2,{\"y\":null}]}";
    case 27: return "[]";
    default: return "";                          // missing
  }
}
// a numeric argument around [lo, hi] with edge cases, or a wrong type
std::string numJson(double lo, double hi) {
  int k = ri(0, 24);
  if (k < 10) return fmtShort(rd(lo, hi));
  switch (k) {
    case 10: return fmtd(lo);
    case 11: return fmtd(hi);
    case 12: return fmtd(lo - 1e-3);
    case 13: return fmtd(hi + 1e-3);
    case 14: return "0";
    case 15: return "-0";
    case 16: return pick(std::vector<std::string>{"1", "-1", "2", "0.5"});
    case 17: return pick(std::vector<std::string>{"1e308", "-1e308", "1e-308", "4e9", "-4e9", "1e309"});
    case 18: return std::to_string(ri(-100000, 100000));
    case 19: return "\"" + fmtShort(rd(lo, hi)) + "\"";
    case 20: return pr(0.5) ? "true" : "false";
    case 21: return "null";
    case 22: return "{}";
    default: return fmtd(rd(lo - (hi - lo), hi + (hi - lo)));
  }
}
std::string strJson(const std::vector<std::string>& good) {
  int k = ri(0, 9);
  if (k < 7) return "\"" + pick(good) + "\"";
  if (k == 7) return "\"" + randString(ri(0, 12)) + "\"";
  if (k == 8) return pick(std::vector<std::string>{"1", "null", "true", "[]"});
  return "\"" + randString(ri(60, 80)) + "\"";
}
// build a command object from key/value pairs, with a random id, random extra keys and spacing
std::string obj(const std::string& c, std::vector<std::pair<std::string, std::string>> kv) {
  std::vector<std::pair<std::string, std::string>> all;
  std::string id = idJson(); if (!id.empty()) all.push_back({"id", id});
  if (!c.empty()) all.push_back({"cmd", "\"" + c + "\""});
  for (auto& p : kv) if (!p.second.empty()) all.push_back(p);
  if (pr(0.08)) all.push_back({"extra", pr(0.5) ? "[1,{\"a\":2}]" : "\"x\""});
  if (pr(0.03)) for (int i = 0; i < 14; i++) all.push_back({"k" + std::to_string(i), std::to_string(i)});   // over 12 keys
  if (!all.empty() && pr(0.15)) std::swap(all[ri(0, (int)all.size() - 1)], all[ri(0, (int)all.size() - 1)]);
  bool sp = pr(0.2);
  std::string s = sp ? "{ " : "{";
  for (size_t i = 0; i < all.size(); i++) { if (i) s += sp ? ", " : ","; s += "\"" + all[i].first + "\"" + (sp ? ": " : ":") + all[i].second; }
  s += sp ? " }" : "}";
  return s;
}

const std::vector<std::string> benignKeys = {"limit_n", "k_n_per_a", "free_a", "aim_tol", "aim_offset", "line_m", "cock_trip_a",
                                             "vbat_low", "host_timeout_s", "require_seated", "vbat_wired"};
struct Range { double lo, hi; };
Range keyRange(const std::string& k) {
  static std::map<std::string, Range> m = {{"limit_n", {1, 150}}, {"k_n_per_a", {0.1, 200}}, {"free_a", {0, 10}}, {"amps_per_volt", {1, 200}},
      {"offset_a", {0, 5}}, {"hx_n_per_count", {1e-7, 1}}, {"drum_dps", {5, 720}}, {"aim_tol", {0.1, 5}}, {"aim_offset", {-20, 20}},
      {"line_m", {0, 100}}, {"trip_a", {3, 19}}, {"cock_trip_a", {3, 19}}, {"vbat_low", {6, 12}}, {"aim_home_deg", {-7200, 7200}},
      {"host_timeout_s", {0, 600}}, {"require_seated", {0, 1}}, {"vbat_wired", {0, 1}}, {"enc_sign", {-1, 1}}};
  auto it = m.find(k); return it == m.end() ? Range{0, 1} : it->second;
}

std::string randomCommand() {
  int k = ri(0, 23);
  switch (k) {
    case 0: return obj("ping", {});
    case 1: return obj("status", {});
    case 2: return obj("stop", {});
    case 3: return obj(pr(0.3) ? "estop" : "stop", {});
    case 4: return obj("clear", {});
    case 5: case 6: return obj("mode", {{"to", pr(0.1) ? std::string() : strJson({"LAUNCH", "NEUTRAL", "REEL", "WRAPPED", "launch", ""})}});
    case 7: return obj("aim", {{"deg", pr(0.05) ? std::string() : numJson(6, 47.1)}});
    case 8: return obj("cock", {});
    case 9: return obj("arm", {});
    case 10: {
      std::string t;
      int m = ri(0, 7);
      if (m < 3 && haveToken) t = std::to_string(lastToken);
      else if (m == 3 && haveToken) t = "\"" + std::to_string(lastToken) + "\"";
      else if (m == 4 && haveToken) t = std::to_string(lastToken + 1);
      else if (m == 5) t = "";
      else t = numJson(100000, 999999);
      return obj("fire", {{"token", t}});
    }
    case 11: if (pr(0.3)) return obj("tidy", {{"ms", pr(0.5) ? std::string() : numJson(-100, 20000)}});
             if (pr(0.2)) return obj(pr(0.5) ? "retrieve" : "decock", {});
             return obj("wrap", {});
    case 12: return obj("unwrap", {});
    case 13: case 14:
      return obj("reel", {{"duty", pr(0.3) ? std::string() : numJson(0, 1)}, {"max_n", pr(0.3) ? std::string() : numJson(0, 150)},
                          {"until", pr(0.3) ? std::string() : strJson({"stop", "seated", "tension"})}});
    case 15: case 16:
      return obj("jog", {{"motor", pr(0.05) ? std::string() : strJson({"wiper", "drum"})},
                         {"dir", pr(0.7) ? std::string(pr(0.5) ? "1" : "-1") : numJson(-1, 1)}, {"ms", pr(0.3) ? std::string() : numJson(0, 1500)}});
    case 17: {
      std::string w = strJson({"current", "free", "tare", "aim", "aim_home", "aim_forget", "cock_rest", "decock", "decock", "drum"});
      std::vector<std::pair<std::string, std::string>> kv = {{"what", pr(0.05) ? std::string() : w}};
      if (w == "\"aim\"" && pr(0.8)) kv.push_back({"deg", numJson(6, 47.1)});
      if (w == "\"cock_rest\"" && pr(0.05)) { kv.push_back({"force", "true"}); }
      return obj("cal", kv);
    }
    case 18: case 19: {
      std::string key = pr(0.1) ? pick(calibKeys) : pr(0.9) ? pick(benignKeys) : randString(ri(0, 8));
      Range r = keyRange(key);
      std::string val = (key == "require_seated" || key == "vbat_wired") && pr(0.7) ? (pr(0.5) ? "true" : "false") : numJson(r.lo, r.hi);
      return obj("set", {{"key", pr(0.05) ? std::string() : "\"" + key + "\""}, {"val", pr(0.05) ? std::string() : val}});
    }
    case 20: {
      std::vector<std::string> ks = benignKeys; ks.insert(ks.end(), calibKeys.begin(), calibKeys.end());
      ks.push_back("fw"); ks.push_back("params"); ks.push_back("pitch_range"); ks.push_back("nope");
      return obj("get", {{"key", pr(0.05) ? std::string() : "\"" + pick(ks) + "\""}});
    }
    case 21: return obj(pick(std::vector<std::string>{"fly", "PING", "", "fire\\u0000"}), {});
    case 22: return obj("", {{"what", "1"}});        // no cmd
    default: return obj("status", {});
  }
}
std::string garbage() {
  int k = ri(0, 9);
  switch (k) {
    case 0: { std::string s; int n = ri(0, 80); for (int i = 0; i < n; i++) s += (char)ri(1, 255); return s; }
    case 1: { std::string s = randomCommand(); return s.substr(0, ri(0, (int)s.size())); }
    case 2: { std::string s = "{\"id\":1,\"cmd\":\"ping\",\"pad\":\"" + std::string(ri(240, 400), 'x') + "\"}"; return s; }
    case 3: return std::string(ri(257, 600), '{');
    case 4: return pick(std::vector<std::string>{"", "{}", "[]", "null", "{\"cmd\":}", "{\"cmd\":\"ping\",}", "{\"cmd\":\"ping\"} x", "ets Jun  8 2016 00:22:57"});
    case 5: { std::string s = "{\"id\":2,\"cmd\":\"ping\"}"; s[ri(0, (int)s.size() - 1)] = (char)ri(0, 255); return s; }
    case 6: { std::string s = "{\"id\":3,\"cmd\":\"status\",\"n\":" + std::string(ri(1, 3), '[') + "1" + std::string(ri(0, 3), ']') + "}"; return s; }
    case 7: { std::string s = "{\"cmd\":\"ping\",\"d\":"; for (int i = 0; i < 12; i++) s += "{\"a\":"; s += "1"; for (int i = 0; i < 12; i++) s += "}"; return s + "}"; }
    case 8: { std::string s = "{\"id\":\"" + std::string(ri(200, 254), 'y') + "\",\"cmd\":\"ping\"}"; return s.size() > 256 ? s.substr(0, 256) : s; }
    default: { std::string s = randomCommand(); s.insert(ri(0, (int)s.size()), 1, '\0'); return s; }
  }
}

// ------------------------------------------------------------------------------------------------ rig
extern bool indexSeed;
void buildBase() {
  B.reset(new Base());
  if (indexSeed) B->c.swBit[SW_INDEX] = 11;   // D12: no magnet, the 6-bump index cam is the angle source
  S.c = B->c;
  Io io = simIo(); io.saveTun = fuzzSaveTun; B->io = io;
  B->saveAck = true;                       // like the sketch: motors wait for the flash write (acked 20 ms later in tick())
  B->w.c.ampsPerVolt = (float)S.isAmpsPerVolt;
  B->restoreTun(tunHas ? &tunStore : nullptr);
  B->begin(S.t);
  B->restore(S.nvsHas ? &S.nvs : nullptr);
  fireAuth = false;
}
void powerCut() {
  flushAdvance(); logl("POWER CUT");
  S.wDir = 0; S.wDuty = 0; S.dDir = 0; S.dDuty = 0; S.drumAwake = false;   // the bridges lose power first
  S.out.clear(); outSeen = 0;
  buildBase();
  run(100);
}
bool indexSeed = false;
void freshRig() {
  S = Sim(); S.encMissing = indexSeed; S.inertiaMs = seedNow % 3 == 1 ? 40 : seedNow % 3 == 2 ? 100 : 0; /* wiper inertia and coast in 2 of 3 seeds */ tunHas = false; logRing.clear(); tags.clear(); outSeen = 0; haveToken = false; idCounter = 0; pendingAdvance = 0;
  buildBase();
  run(100);
  cmd("{\"id\":0,\"cmd\":\"cal\",\"what\":\"cock_rest\"}");   // a new build starts at rest
}

// ------------------------------------------------------------------------------------------------ fault injection
void healAll() {
  S.drumJam = false; S.drumJamAbove = 1e9; S.swStuckClosed = 0; S.swStuckOpen = 0; S.swChatter = 0; S.encMissing = indexSeed;
  S.encGlitchEvery = 0; S.magnetWeak = false; S.extraLoadNm = 0; S.vbatOverride = -1; S.toothOnTooth = false; S.searBroken = false; S.handLatched = false;
  tags.erase("stuck_contact");             // (a belief it left behind may survive, but the sensor tells the truth again)
  flushAdvance(); logl("inject: heal all");
}
void injectFault() {
  int k = ri(0, 14); std::string d;
  switch (k) {
    case 0: S.drumJam = !S.drumJam; d = std::string("drumJam=") + (S.drumJam ? "1" : "0"); break;
    case 1: S.drumJamAbove = pr(0.5) ? 1e9 : rd(0, 340); d = "drumJamAbove=" + fmtShort(S.drumJamAbove); break;
    case 2: { int b = ri(0, 11); S.swStuckClosed ^= (1 << b); S.swStuckOpen &= ~S.swStuckClosed; d = "swStuckClosed=" + std::to_string(S.swStuckClosed); tags.insert("stuck_contact"); break; }
    case 3: { int b = ri(0, 11); S.swStuckOpen ^= (1 << b); S.swStuckClosed &= ~S.swStuckOpen; d = "swStuckOpen=" + std::to_string(S.swStuckOpen); tags.insert("stuck_contact"); break; }
    case 4: { int b = ri(0, 11); S.swChatter ^= (1 << b); d = "swChatter=" + std::to_string(S.swChatter); break; }
    case 5: if (indexSeed) break; S.encMissing = !S.encMissing; d = std::string("encMissing=") + (S.encMissing ? "1" : "0"); break;
    case 6: S.encGlitchEvery = pr(0.5) ? 0 : ri(3, 50); S.magnetWeak = pr(0.3); d = "encGlitchEvery=" + std::to_string(S.encGlitchEvery); break;
    case 7: S.extraLoadNm = pick(std::vector<double>{0, 0, 3, 8, 14.5, 30}); d = "extraLoadNm=" + fmtShort(S.extraLoadNm); break;
    case 8: S.vbatOverride = pick(std::vector<double>{-1, -1, 12.0, 9.5, 8.0, 5.0, 0.3}); d = "vbatOverride=" + fmtShort(S.vbatOverride); break;
    case 9: S.toothOnTooth = !S.toothOnTooth; S.toothNeedWiper = pr(0.8) ? rd(2, 30) : 1e9; d = std::string("toothOnTooth=") + (S.toothOnTooth ? "1" : "0") + " need " + fmtShort(S.toothNeedWiper); break;
    case 10: if (pr(0.5)) { S.lineOut = 0; S.seatAtLine = 0; S.anchorTautAt = -1; d = "dart seated"; } else { S.lineOut = rd(0.5, 15); S.seatAtLine = 0; S.anchorTautAt = -1; d = "dart out " + fmtShort(S.lineOut); } break;
    case 11: S.anchorTautAt = S.lineOut - rd(0, 1); S.anchorK = rd(20, 100); d = "tow anchor taut at " + fmtShort(S.anchorTautAt); break;
    case 12: S.hxFitted = !S.hxFitted; d = std::string("hxFitted=") + (S.hxFitted ? "1" : "0"); break;
    case 13: S.handLatched = !S.handLatched; d = std::string("handLatched=") + (S.handLatched ? "1" : "0"); break;
    default: S.searBroken = pr(0.5); d = std::string("searBroken=") + (S.searBroken ? "1" : "0"); break;
  }
  flushAdvance(); logl("inject: " + d);
}

// ------------------------------------------------------------------------------------------------ missions
std::string idn() { return std::to_string(++idCounter); }
void missionLaunch() {
  flushAdvance(); logl("-- mission launch");
  if (B->state == S_FAULT) cmd("{\"id\":" + idn() + ",\"cmd\":\"clear\"}");
  if (!resting()) { cmd("{\"id\":" + idn() + ",\"cmd\":\"stop\"}"); run(250); }
  cmd("{\"id\":" + idn() + ",\"cmd\":\"mode\",\"to\":\"LAUNCH\"}"); waitRest(25000);
  if (B->phase == PH_UNKNOWN) {
    if (B->closed(SW_HOME)) cmd("{\"id\":" + idn() + ",\"cmd\":\"cock\"}");
    else cmd("{\"id\":" + idn() + ",\"cmd\":\"cal\",\"what\":\"decock\"}");
    waitRest(40000);
  }
  if (B->phase == PH_REST || pr(0.2)) {
    double deg = rd(7, 46);
    cmd("{\"id\":" + idn() + ",\"cmd\":\"aim\",\"deg\":" + fmtShort(deg) + "}"); waitRest(60000);
  }
  if (pr(0.5)) { cmd("{\"id\":" + idn() + ",\"cmd\":\"cock\"}"); waitRest(30000); }
  if (pr(0.85)) { S.lineOut = 0; S.seatAtLine = 0; S.anchorTautAt = -1; flushAdvance(); logl("inject: dart seated"); run(50); }
  cmd("{\"id\":" + idn() + ",\"cmd\":\"arm\"}");
  bool armed = replyOk();
  if (pr(0.1)) run(ri(0, 4000));                  // maybe let it expire
  std::string tok = armed ? std::to_string(lastToken) : std::to_string(ri(100000, 999999));
  if (pr(0.05)) tok = std::to_string(lastToken + 1);
  cmd("{\"id\":" + idn() + ",\"cmd\":\"fire\",\"token\":" + tok + "}");
  if (pr(0.15)) { run(ri(5, 3000)); cmd("{\"id\":" + idn() + ",\"cmd\":\"" + (pr(0.5) ? "stop" : "estop") + "\"}"); }
  waitRest(30000);
  if (pr(0.5)) cmd("{\"id\":" + idn() + ",\"cmd\":\"set\",\"key\":\"line_m\",\"val\":" + fmtShort(rd(0, 20)) + "}");
}
void missionReel() {
  flushAdvance(); logl("-- mission reel");
  if (B->state == S_FAULT) cmd("{\"id\":" + idn() + ",\"cmd\":\"clear\"}");
  if (!resting()) { cmd("{\"id\":" + idn() + ",\"cmd\":\"stop\"}"); run(250); }
  if (B->wraps < 1) {
    if (pr(0.7)) { cmd("{\"id\":" + idn() + ",\"cmd\":\"mode\",\"to\":\"NEUTRAL\"}"); waitRest(30000); }   // D22: or wrap straight from LAUNCH
    if (pr(0.3)) { cmd("{\"id\":" + idn() + ",\"cmd\":\"reel\",\"duty\":0.2,\"until\":\"slip\"}"); run(ri(500, 6000)); }
    cmd("{\"id\":" + idn() + ",\"cmd\":\"wrap\"}");
    if (pr(0.2)) { run(ri(100, 40000)); cmd("{\"id\":" + idn() + ",\"cmd\":\"" + (pr(0.5) ? "stop" : "estop") + "\"}"); } else waitRest(120000);
  }
  if (B->drumPos() == DP_WRAPPED || pr(0.1)) {     // D22: tidy in the WRAPPED park, then the fly-in to REEL
    if (pr(0.6)) { cmd("{\"id\":" + idn() + ",\"cmd\":\"tidy\"" + (pr(0.5) ? ",\"ms\":" + std::to_string(ri(0, 9000)) : std::string()) + "}"); waitRest(20000); }
    cmd("{\"id\":" + idn() + ",\"cmd\":\"mode\",\"to\":\"REEL\"}");
    if (pr(0.1)) { run(ri(100, 5000)); cmd("{\"id\":" + idn() + ",\"cmd\":\"estop\"}"); }
    waitRest(30000);
  }
  if (pr(0.4)) { S.lineOut = rd(2, 15); S.anchorTautAt = S.lineOut - rd(0.1, 1); S.anchorK = rd(20, 100); flushAdvance(); logl("inject: tow anchor"); }
  else { S.lineOut = rd(0.5, 6); S.seatAtLine = 0; S.anchorTautAt = -1; flushAdvance(); logl("inject: dart out, slack line"); }
  std::string u = pick(std::vector<std::string>{"stop", "seated", "tension"});
  cmd("{\"id\":" + idn() + ",\"cmd\":\"reel\",\"duty\":" + fmtShort(rd(0.2, 1)) + ",\"max_n\":" + fmtShort(rd(5, 60)) + ",\"until\":\"" + u + "\"}");
  run(ri(500, 15000));
  if (B->state == S_HOLD && pr(0.5)) { S.anchorTautAt = -1; flushAdvance(); logl("inject: dart released"); }
  cmd("{\"id\":" + idn() + ",\"cmd\":\"stop\"}");
  if (pr(0.7)) { cmd("{\"id\":" + idn() + ",\"cmd\":\"unwrap\"}"); waitRest(60000);
    if (pr(0.6)) { S.lineOut = rd(0, 2); S.anchorTautAt = pr(0.2) ? S.lineOut - 0.2 : -1; cmd("{\"id\":" + idn() + ",\"cmd\":\"reel\",\"duty\":0.3,\"until\":\"seated\"}"); run(ri(500, 25000)); } }
}

// ------------------------------------------------------------------------------------------------ one seed
void runSeed(uint32_t seed, int steps) {
  seedNow = seed; rng.seed(seed * 0x9E3779B97F4A7C15ull + 1);
  indexSeed = seed % 5 == 0;                // every fifth seed runs without the AS5600 magnet (SW_INDEX mode, D12)
  freshRig();
  double faultRate = pick(std::vector<double>{0.0, 0.02, 0.05, 0.12});
  for (stepNow = 0; stepNow < steps; stepNow++) {
    double x = rd(0, 1);
    if ((x -= 0.30) < 0) cmd(randomCommand());
    else if ((x -= 0.06) < 0) cmd(garbage());
    else if ((x -= 0.38) < 0) {
      int m = ri(0, 19); uint32_t ms = m < 10 ? ri(1, 10) * 5 : m < 17 ? ri(10, 100) * 5 : m < 19 ? ri(100, 600) * 5 : ri(600, 3000) * 5;
      pendingAdvance += ms; run(ms);
    }
    else if ((x -= faultRate) < 0) injectFault();
    else if ((x -= 0.03) < 0) healAll();
    else if ((x -= 0.015) < 0) powerCut();
    else if ((x -= 0.04) < 0) missionLaunch();
    else if ((x -= 0.015) < 0) missionReel();
    else if ((x -= 0.02) < 0) waitRest(30000);
    else { pendingAdvance += 5; run(5); }
    if (S.out.size() > 64) { S.out.clear(); outSeen = 0; }
  }
}

}  // namespace

int main(int argc, char** argv) {
  int seeds = argc > 1 ? atoi(argv[1]) : 300;
  uint32_t first = argc > 2 ? (uint32_t)strtoul(argv[2], nullptr, 10) : 1;
  int steps = getenv("FUZZ_STEPS") ? atoi(getenv("FUZZ_STEPS")) : 300;
  verbose = getenv("FUZZ_LOG") != nullptr;
  trace = getenv("FUZZ_TRACE") != nullptr;      // with FUZZ_LOG: firmware against sim truth every 50 ms while a motor runs
  long shotsTotal = 0, firedSeeds = 0; uint64_t simMs = 0;
  for (int i = 0; i < seeds; i++) {
    runSeed(first + i, steps);
    shotsTotal += S.shots; if (S.shots) firedSeeds++; simMs += S.t;
  }
  printf("\nfuzz: %d seeds (%u..%u), %d steps each, %.1f h simulated, %ld shots in %ld seeds\n", seeds, (unsigned)first,
         (unsigned)(first + seeds - 1), steps, simMs / 3.6e6, shotsTotal, firedSeeds);
  if (viol.empty()) { printf("fuzz: no invariant violated\n"); return 0; }
  for (auto& v : viol) {
    printf("fuzz: VIOLATION %-26s x%d seeds:", v.first.c_str(), v.second.count);
    for (auto s : v.second.seeds) printf(" %u", (unsigned)s);
    printf("\n");
  }
  return 1;
}
