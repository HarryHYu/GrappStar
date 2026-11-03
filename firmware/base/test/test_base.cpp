// PC tests for the GrappStar base firmware logic. Build and run: ./run_tests.sh (or see the README).
// Every mode and fault path runs against the simulation in sim.h.
#include <stdio.h>
#include <functional>
#include <string>
#include "sim.h"

Sim S;
static int pass = 0, fail = 0;
static const char* curTest = "";
#define CHECK(cond, msg) do { if (cond) pass++; else { fail++; printf("  FAIL [%s] %s (line %d)\n", curTest, msg, __LINE__); } } while (0)

struct Rig {
  gb::Base B;
  explicit Rig(bool fresh = true, std::function<void(gb::Cfg&)> cfgEdit = nullptr) {
    if (fresh) { S = Sim(); }
    if (cfgEdit) cfgEdit(B.c);
    S.c = B.c;
    B.io = simIo();
    B.w.c.ampsPerVolt = (float)S.isAmpsPerVolt;
    B.begin(S.t);
    B.restore(S.nvsHas ? &S.nvs : nullptr);
    run(100);
    if (fresh && restRig) cmd("{\"id\":0,\"cmd\":\"cal\",\"what\":\"cock_rest\"}");   // a new build starts at rest
  }
  static bool restRig;
  void tick() { S.step(5); B.tick(S.t); }
  void run(uint32_t ms) { for (uint32_t i = 0; i < ms; i += 5) tick(); }
  bool until(std::function<bool()> p, uint32_t maxMs) { for (uint32_t i = 0; i < maxMs; i += 5) { tick(); if (p()) return true; } return false; }
  std::string cmd(const std::string& j) { char rep[300]; B.handleLine(j.c_str(), (int)j.size(), rep, sizeof rep); return rep; }
  bool ok(const std::string& j) { std::string r = cmd(j); bool o = r.find("\"ok\":true") != std::string::npos; if (!o) printf("    (reply %s)\n", r.c_str()); return o; }
  std::string err(const std::string& j) { std::string r = cmd(j); size_t p = r.find("\"err\":\""); if (p == std::string::npos) return ""; p += 7; return r.substr(p, r.find('"', p) - p); }
  bool evt(const char* name) { for (auto& l : S.out) if (l.find(std::string("\"evt\":\"") + name + "\"") != std::string::npos) return true; return false; }
  std::string lastEvt(const char* name) { for (int i = (int)S.out.size() - 1; i >= 0; i--) if (S.out[i].find(std::string("\"evt\":\"") + name + "\"") != std::string::npos) return S.out[i]; return ""; }
  bool idle() const { return B.state == gb::S_IDLE; }
  // helpers to reach mission states
  bool toMode(const char* m) { if (!ok(std::string("{\"id\":1,\"cmd\":\"mode\",\"to\":\"") + m + "\"}")) return false; return until([&] { return idle() || B.state == gb::S_FAULT; }, 20000) && idle(); }
  bool homeAimAndCock() {          // aim first (needs the carriage home), then cock (forward does not move the aim)
    if (!toMode("LAUNCH")) return false;
    if (!ok("{\"id\":3,\"cmd\":\"aim\",\"deg\":35}")) return false;
    if (!until([&] { return idle() || B.state == gb::S_FAULT; }, 60000) || !B.aimed) return false;
    if (!ok("{\"id\":2,\"cmd\":\"cock\"}")) return false;
    return until([&] { return idle() || B.state == gb::S_FAULT; }, 30000) && idle() && B.aimed;
  }
};

bool Rig::restRig = true;

static void T(const char* n) { curTest = n; printf("%s\n", n); }

// ------------------------------------------------------------------------------------------------ unit tests
static void testJson() {
  T("json: parser and builder");
  gb::JObj o;
  const char* a = "{\"id\": 17, \"cmd\": \"aim\", \"deg\": 32.5}";
  CHECK(gb::jsonParse(a, (int)strlen(a), &o), "parses a command");
  double d; const char* s;
  CHECK(o.num("id", &d) && d == 17, "id");
  CHECK(o.str("cmd", &s) && !strcmp(s, "aim"), "cmd");
  CHECK(o.num("deg", &d) && d == 32.5, "deg");
  const char* b = "{\"id\":1,\"cmd\":\"x\",\"nested\":{\"a\":[1,2,{\"b\":null}]},\"s\":\"q\\\"u\\n\"}";
  CHECK(gb::jsonParse(b, (int)strlen(b), &o), "nested values are skipped");
  CHECK(o.str("s", &s) && !strcmp(s, "q\"u\n"), "escapes");
  const char* bad[] = {"", "{", "{\"a\":}", "{\"a\":1,}", "[1]", "{\"a\":1} x", "{a:1}", "{\"a\":tru}", "{\"a\":\"x}", "{\"a\":01e}", "{\"a\":-}", "{\"a\":1e999}"};
  for (auto x : bad) CHECK(!gb::jsonParse(x, (int)strlen(x), &o), x);
  const char* e = "  {}  ";
  CHECK(gb::jsonParse(e, (int)strlen(e), &o) && o.count == 0, "empty object");
  const char* dup = "{\"a\":1,\"a\":2}";
  CHECK(gb::jsonParse(dup, (int)strlen(dup), &o) && o.num("a", &d) && d == 2, "duplicate key: last wins");
  char buf[64]; gb::JOut j(buf, sizeof buf); j.open(); j.num("x", 12.50, 2); j.num("y", -0.0001, 2); j.str("s", "a\"b"); j.null("n"); j.close();
  CHECK(!strcmp(buf, "{\"x\":12.5,\"y\":0,\"s\":\"a\\\"b\",\"n\":null}"), buf);
  char tiny[8]; gb::JOut k(tiny, sizeof tiny); k.open(); k.str("long", "overflow"); k.close();
  CHECK(k.bad && strlen(tiny) < 8, "builder never overflows");
}

static void testSwitches() {
  T("switches: debounce and chatter");
  gb::Switches s; s.debounceMs = 15;
  uint32_t t = 0; s.update(0, t);
  for (int i = 0; i < 2; i++) { t += 5; s.update(1, t); }
  CHECK(!s.closed(0), "not closed before the debounce time");
  for (int i = 0; i < 3; i++) { t += 5; s.update(1, t); }
  CHECK(s.closed(0), "closed after holding 15 ms");
  bool sawRise = false; gb::Switches s2; s2.update(0, 0); for (uint32_t tt = 5; tt < 50; tt += 5) { s2.update(1, tt); if (s2.rose(0)) sawRise = true; }
  CHECK(sawRise, "rise edge reported once");
  // bounce: 1 0 1 0 ... for 40 ms then steady 1
  gb::Switches s3; s3.update(0, 0); uint32_t tt = 0;
  for (int i = 0; i < 8; i++) { tt += 5; s3.update(i & 1, tt); }
  CHECK(!s3.closed(0), "bounce does not close");
  for (int i = 0; i < 5; i++) { tt += 5; s3.update(1, tt); }
  CHECK(s3.closed(0), "closes after the bounce settles");
  // chatter: toggling every tick for 100 ms
  gb::Switches s4; s4.update(0, 0); tt = 0;
  for (int i = 0; i < 20; i++) { tt += 5; s4.update(i & 1, tt); }
  CHECK(s4.chattering(0), "chatter detected");
  for (int i = 0; i < 250; i++) { tt += 5; s4.update(1, tt); }
  CHECK(!s4.chattering(0), "chatter clears after a quiet second");
}

static void testAngle() {
  T("as5600: unwrap, glitch, loss");
  gb::AngleTracker a; uint8_t md = gb::AS_MD;
  int raw = 4000; a.feed(raw, md, 5);
  for (int i = 0; i < 100; i++) { raw = (raw + 30) % 4096; a.feed(raw, md, 5); }
  CHECK(a.count == 3000, "counts across the 4095 -> 0 wrap");
  a.feed(2000, md, 5);
  CHECK(a.count == 3000 && a.glitches == 1, "single glitch ignored");
  a.feed((raw + 30) % 4096, md, 5); raw = (raw + 30) % 4096;
  CHECK(a.count == 3030, "continues after a glitch");
  for (int i = 0; i < 4; i++) a.feed(-1, 0, 5);
  CHECK(!a.ok, "lost after 4 bad reads");
  a.feed(100, md, 5);
  CHECK(a.ok && a.count == 3030, "comes back without counting the jump");
  a.feed(100, 0, 5); a.feed(100, 0, 5); a.feed(100, 0, 5);
  CHECK(a.ok, "3 reads without the magnet are tolerated");
  a.feed(100, 0, 5);
  CHECK(!a.ok, "magnet missing = lost");
  gb::AngleTracker r; r.feed(0, md, 5); int rr = 0;
  for (int i = 0; i < 80; i++) { rr = (rr + 23) % 4096; r.feed(rr, md, 5); }
  CHECK(fabsf(r.rateDps - 23 * 360.f / 4096 * 200) < 20, "rate in deg/s");
  gb::AngleTracker n; n.sign = -1; n.feed(1000, md, 5); n.feed(1010, md, 5);
  CHECK(n.count == -10, "sign applied");
}

static void testWiperDrive() {
  T("bts7960 drive: ramp, gap, scaling, trip");
  gb::WiperDrive w; uint32_t t = 0; int hd = 0, hu = 0;
  static int *pd, *pu; pd = &hd; pu = &hu;
  auto hw = [](int d, int u) { *pd = d; *pu = u; };
  w.want(1, 255);
  w.tick(t += 5, 5, 0, NAN, hw);
  CHECK(hd == 1 && hu < 10, "starts at low duty");
  for (int i = 0; i < 70; i++) w.tick(t += 5, 5, 0, NAN, hw);
  CHECK(hu == 255, "ramped to full in 300 ms");
  w.want(-1, 255); w.tick(t += 5, 5, 0, NAN, hw);
  CHECK(hd == 0, "reverse request stops first");
  for (int i = 0; i < 30; i++) w.tick(t += 5, 5, 0, NAN, hw);
  CHECK(hd == 0, "still stopped inside the 200 ms gap");
  for (int i = 0; i < 20; i++) w.tick(t += 5, 5, 0, NAN, hw);
  CHECK(hd == -1, "reverses after the gap");
  // scaling: at duty 128 the IS average is half the motor current
  gb::WiperDrive s; s.want(1, 128); for (int i = 0; i < 100; i++) s.tick(t += 5, 5, 4.0 / 12.5 * 128 / 255, 100, hw);
  CHECK(fabsf(s.amps - 4.0f) < 0.1f, "IS reading scaled back by the duty");
  gb::WiperDrive o; o.want(1, 255); for (int i = 0; i < 70; i++) o.tick(t += 5, 5, 10.0 / 12.5 * o.outDuty / 255, 100, hw);
  CHECK(!o.overcurrent, "10 A during the ramp: no trip");
  o.tick(t += 5, 5, 20.0 / 12.5, 100, hw); CHECK(!o.overcurrent, "one sample over is not a trip");
  o.tick(t += 5, 5, 20.0 / 12.5, 100, hw); CHECK(o.overcurrent && hd == 0, "two samples over: trip, motor off");
  o.want(1, 255); o.tick(t += 5, 5, 0, 100, hw); CHECK(hd == 0, "stays off until the latch is cleared");
  gb::WiperDrive st; st.want(1, 255); for (int i = 0; i < 70; i++) st.tick(t += 5, 5, 0.3, 100, hw);
  for (int i = 0; i < 90; i++) st.tick(t += 5, 5, 0.3, 0, hw);
  CHECK(st.stalled && hd == 0, "stall: hard drive, shaft still for 400 ms");
}

static void testPersist() {
  T("persistence: crc and layout");
  gb::Saved s; s.drumDeg = 60; s.wraps = 2; gb::sealSaved(&s);
  CHECK(gb::validSaved(&s), "sealed record valid");
  s.wraps = 3; CHECK(!gb::validSaved(&s), "edited record invalid");
}

// ------------------------------------------------------------------------------------------------ mission tests
static void testProtocolBasics() {
  T("protocol: one reply per command, errors");
  Rig R;
  CHECK(R.cmd("{\"id\":17,\"cmd\":\"ping\"}").find("{\"id\":17,\"ok\":true") == 0, "ping reply starts with id and ok");
  CHECK(R.cmd("garbage") == "{\"id\":null,\"ok\":false,\"err\":\"BAD_JSON\"}", "bad json");
  std::string longl(300, ' '); longl = "{\"id\":1,\"cmd\":\"ping\",\"x\":\"" + std::string(260, 'a') + "\"}";
  CHECK(R.err(longl) == "TOO_LONG", "line over 256 bytes");
  CHECK(R.err("{\"id\":2}") == "NO_CMD", "no cmd");
  CHECK(R.err("{\"id\":3,\"cmd\":\"dance\"}") == "UNKNOWN_CMD", "unknown cmd");
  CHECK(R.cmd("{\"id\":\"abc\",\"cmd\":\"ping\"}").find("\"id\":\"abc\"") != std::string::npos, "string id echoed");
  CHECK(R.err("{\"id\":4,\"cmd\":\"mode\",\"to\":\"SIDEWAYS\"}") == "BAD_ARG", "bad mode");
  CHECK(R.err("{\"id\":5,\"cmd\":\"mode\"}") == "BAD_ARG", "mode without to");
  std::string st = R.cmd("{\"id\":6,\"cmd\":\"status\"}");
  CHECK(st.find("\"phase\"") != std::string::npos && st.find("\"magnet\":\"OK\"") != std::string::npos, "status has extra fields");
  R.B.fault = gb::F_SWITCH_DISAGREE; strcpy(R.B.faultWhy, "a long fault reason, forty seven chars long ok");
  st = R.cmd("{\"id\":123456789012,\"cmd\":\"status\"}"); R.B.fault = 0;
  CHECK(st.size() <= 256 && st.find("\"id\":123456789012") == 1, "status worst case within 256 bytes");
  st = R.cmd("{\"id\":\"" + std::string(60, 'x') + "\",\"cmd\":\"status\"}");
  CHECK(st.size() <= 256 && st.find("\"ok\":true") != std::string::npos, "long string id: short reply, still ok");
  CHECK(R.ok("{\"id\":7,\"cmd\":\"set\",\"key\":\"limit_n\",\"val\":30}") && R.B.c.limitN == 30, "set limit");
  CHECK(R.err("{\"id\":8,\"cmd\":\"set\",\"key\":\"limit_n\",\"val\":900}") == "OUT_OF_RANGE", "set out of range");
  CHECK(R.err("{\"id\":9,\"cmd\":\"set\",\"key\":\"nope\",\"val\":1}") == "BAD_KEY", "bad key");
  CHECK(R.cmd("{\"id\":10,\"cmd\":\"get\",\"key\":\"limit_n\"}").find("\"val\":30") != std::string::npos, "get");
  CHECK(R.evt("BOOT"), "boot event");
  char tel[300]; R.B.telemetry(tel, sizeof tel);
  CHECK(std::string(tel).find("{\"tel\":{\"state\":\"IDLE\"") == 0, tel);
  CHECK(strlen(tel) <= 256, "telemetry within 256 bytes");
  CHECK(std::string(tel).find("\"sw\":{\"a\":1,\"H\":1,\"W\":1}") != std::string::npos, tel);   // drum code 001, carriage home, wrap arm home
}

static void testTelemetryWorstCase() {
  T("telemetry: worst case fits 256 bytes");
  Rig R;
  // longest realistic line: moving, every value long, the contacts a running system can have closed at once
  R.B.drumKnown = true; R.B.drumDeg = -1234.5f; R.B.enc.count = -4096000; R.B.enc.rateDps = -400; R.B.state = gb::S_UNWRAP;
  R.B.wrapsKnown = true; R.B.wraps = 12; R.B.lineM = 99.99f; R.B.amps = 19.9f; R.B.aimKnown = true; R.B.phase = gb::PH_COCKING; R.B.cockPhase = 300;
  R.B.hxSeen = true; R.B.hxT = R.B.now; R.B.hxN = -1234.5f; R.B.vbat = 12.6f;
  S.swStuckClosed = 0xF9; R.B.c.vbatLow = 0; R.run(30); R.B.state = gb::S_UNWRAP; R.B.hxT = R.B.now; R.B.enc.rateDps = -400; R.B.secBad = false;
  char tel[400]; R.B.telemetry(tel, 400);
  { char big[700]; R.B.telBuild(big, 700, false); printf("    realistic worst case %zu bytes: %s\n", strlen(big), big); }
  CHECK(strlen(tel) <= 256 && std::string(tel).find("\"trunc\"") == std::string::npos, tel);
  gb::JObj o; CHECK(gb::jsonParse(tel, (int)strlen(tel), &o), "valid JSON");
  // absurd values: still valid JSON within 256 bytes
  R.B.enc.count = -99999999; R.B.enc.rateDps = -1e8f; R.B.fault = gb::F_SWITCH_DISAGREE; R.B.state = gb::S_FAULT; S.swStuckClosed = 0x7F; R.run(30);
  R.B.telemetry(tel, 400);
  CHECK(strlen(tel) <= 256 && gb::jsonParse(tel, (int)strlen(tel), &o), "absurd state: still a valid line within 256 bytes");
}

static void testModeChanges() {
  T("mode: LAUNCH / NEUTRAL / REEL with verification");
  Rig R;                                   // sim drum starts at 0 = LAUNCH
  CHECK(R.B.drumPos() == gb::DP_LAUNCH && R.B.drumKnown, "boots at LAUNCH from the contact");
  CHECK(R.toMode("REEL"), "LAUNCH -> REEL");
  CHECK(R.B.drumPos() == gb::DP_REEL && fabs(S.drumDeg - 80) < 4, "drum stopped at 80 (REEL stop point)");
  CHECK(R.evt("MODE") && R.lastEvt("MODE").find("\"ok\":true") != std::string::npos, "MODE event ok");
  CHECK(fabs(R.B.c.drumDps - 10) < 1.5, "drum speed learned from the edge timing");
  CHECK(R.toMode("NEUTRAL"), "REEL -> NEUTRAL");
  CHECK(R.toMode("LAUNCH"), "NEUTRAL -> LAUNCH");
  S.out.clear(); CHECK(R.ok("{\"id\":1,\"cmd\":\"mode\",\"to\":\"LAUNCH\"}") && R.evt("MODE"), "already there: ok + MODE at once");
  CHECK(R.ok("{\"id\":1,\"cmd\":\"mode\",\"to\":\"REEL\"}"), "start a move");
  CHECK(R.err("{\"id\":2,\"cmd\":\"mode\",\"to\":\"LAUNCH\"}") == "BUSY", "second move while moving: BUSY");
  CHECK(R.err("{\"id\":2,\"cmd\":\"aim\",\"deg\":30}") == "BUSY", "aim while moving: BUSY");
  R.until([&] { return R.idle(); }, 20000);
  CHECK(R.B.drumPos() == gb::DP_REEL, "arrived");
}

static void testDrumFaults() {
  T("drum faults: jam, wire off, sector jump, wrong direction, tooth on tooth, chatter");
  { Rig R; S.drumJam = true;
    R.ok("{\"id\":1,\"cmd\":\"mode\",\"to\":\"REEL\"}");
    R.until([&] { return R.B.state == gb::S_FAULT; }, 30000);
    CHECK(R.B.fault == gb::F_DRUM_TIMEOUT, "jammed drum: DRUM_TIMEOUT"); CHECK(S.wDir == 0 && S.dDir == 0, "everything off");
    CHECK(R.B.drumPos() == gb::DP_LAUNCH, "drum_pos still LAUNCH (estimate stays inside the sector)"); }
  { Rig R; S.swStuckOpen = 1 << 1;             // SW_DRUM_B wire off: NEUTRAL shows 000
    R.ok("{\"id\":1,\"cmd\":\"mode\",\"to\":\"REEL\"}");
    R.until([&] { return R.B.state == gb::S_FAULT; }, 30000);
    CHECK(R.B.fault == gb::F_DRUM_SENSOR, "wire off: DRUM_SENSOR (code 000)"); }
  { Rig R; S.swStuckClosed = 1 << 2; R.run(400);  // SW_DRUM_C stuck closed at LAUNCH: 101 = jump to WRAPPING
    CHECK(R.B.fault == gb::F_DRUM_SENSOR, "code jump over sectors: DRUM_SENSOR");
    CHECK(R.err("{\"id\":1,\"cmd\":\"clear\"}") == "STILL_PRESENT", "clear refused while present");
    S.swStuckClosed = 0; R.run(100);
    CHECK(R.ok("{\"id\":1,\"cmd\":\"clear\"}") && R.idle(), "clear once fixed"); }
  { Rig R; R.B.c.drumSign = -1;                  // N20 wired the other way round
    R.ok("{\"id\":1,\"cmd\":\"mode\",\"to\":\"REEL\"}");
    R.until([&] { return R.B.state == gb::S_FAULT; }, 30000);
    CHECK(R.B.fault == gb::F_DRUM_SENSOR || R.B.fault == gb::F_DRUM_TIMEOUT, "drum_sign wrong: stopped with a drum fault"); }
  { Rig R; R.toMode("NEUTRAL"); S.toothOnTooth = true; S.out.clear();
    CHECK(R.toMode("LAUNCH"), "tooth on tooth: SW_DOG wiggle lets the dogs drop in");
    CHECK(!S.dogTip && S.wDir == 0, "dogs engaged, wiper stopped");
    CHECK(!R.B.aimKnown, "wiggle into LAUNCH forgets the aim (re-home)");
    S.toothOnTooth = true; CHECK(R.toMode("REEL"), "tooth on tooth into REEL: forward wiggle"); }
  { Rig R; S.toothOnTooth = true; S.toothNeedWiper = 1e9; R.toMode("NEUTRAL"); R.ok("{\"id\":1,\"cmd\":\"mode\",\"to\":\"LAUNCH\"}");
    R.until([&] { return R.B.state == gb::S_FAULT; }, 30000);
    CHECK(R.B.fault == gb::F_DRUM_TIMEOUT, "dogs never drop in: DRUM_TIMEOUT after 60 deg of wiggle"); }
  { Rig R; S.swChatter = 1 << 1; R.ok("{\"id\":1,\"cmd\":\"mode\",\"to\":\"REEL\"}");
    R.until([&] { return R.B.state == gb::S_FAULT; }, 10000);
    CHECK(R.B.fault == gb::F_SWITCH_DISAGREE || R.B.fault == gb::F_DRUM_SENSOR, "chattering code contact during a move: fault"); }
}

static void testAim() {
  T("aim: home, table, tolerance, refusals");
  Rig::restRig = false; Rig R; Rig::restRig = true;
  CHECK(R.err("{\"id\":1,\"cmd\":\"aim\",\"deg\":30}") == "PHASE_UNKNOWN", "launch phase unknown at first boot: aim refused");
  R.ok("{\"id\":1,\"cmd\":\"cal\",\"what\":\"cock_rest\"}");
  CHECK(R.err("{\"id\":1,\"cmd\":\"aim\",\"deg\":5}") == "OUT_OF_RANGE", "below the pitch range");
  CHECK(R.err("{\"id\":1,\"cmd\":\"aim\",\"deg\":80}") == "OUT_OF_RANGE", "above the pitch range");
  S.aimPh = 123;
  CHECK(R.ok("{\"id\":1,\"cmd\":\"aim\",\"deg\":32.5}"), "aim accepted (homes first)");
  R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 60000);
  CHECK(R.B.aimed && R.lastEvt("AIMED").find("\"ok\":true") != std::string::npos, "AIMED ok");
  double truePitch = R.B.pitchAt((float)S.aimPh) - R.B.aimOffset;
  CHECK(fabs(truePitch - 32.5) < 1.0, "true pitch within 1 deg of the target");
  CHECK(fabs(R.B.pitch() - truePitch) < 0.6, "firmware pitch agrees with the truth");
  for (double target : {45.0, 20.0, 46.5, 7.5, 15.5}) {
    char b[80]; snprintf(b, sizeof b, "{\"id\":1,\"cmd\":\"aim\",\"deg\":%.1f}", target);
    R.ok(b); R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 60000);
    double tp = R.B.pitchAt((float)S.aimPh) - R.B.aimOffset;
    CHECK(R.B.aimed && fabs(tp - target) < 1.0, b);
  }
  R.toMode("NEUTRAL");
  CHECK(R.err("{\"id\":1,\"cmd\":\"aim\",\"deg\":30}") == "NOT_IN_LAUNCH", "aim outside LAUNCH");
  { Rig Q; Q.B.c.swBit[gb::SW_RAIL_LOW] = -1; Q.ok("{\"id\":1,\"cmd\":\"cal\",\"what\":\"cock_rest\"}");
    CHECK(Q.err("{\"id\":1,\"cmd\":\"aim\",\"deg\":30}") == "AIM_UNKNOWN", "no home switch and no cal: AIM_UNKNOWN"); }
}

static void testCockFireReset() {
  T("cock, arm, fire, reset");
  Rig R;
  R.ok("{\"id\":1,\"cmd\":\"cal\",\"what\":\"cock_rest\"}");
  CHECK(R.err("{\"id\":1,\"cmd\":\"arm\"}") == "NOT_AIMED", "arm before aiming");
  CHECK(R.homeAimAndCock(), "mode LAUNCH, cock, aim");
  CHECK(R.B.phase == gb::PH_COCKED && R.evt("COCKED"), "COCKED");
  CHECK(fabs(S.cockPh - R.B.c.cockEndDeg) < 6, "stopped at the cock end");
  S.seatAtLine = 0; S.lineOut = 0; R.run(50);   // dart seated
  CHECK(R.err("{\"id\":1,\"cmd\":\"fire\",\"token\":123456}") == "NOT_ARMED", "fire without arm");
  std::string a = R.cmd("{\"id\":2,\"cmd\":\"arm\"}");
  CHECK(a.find("\"token\":") != std::string::npos, "arm returns a token");
  long tok = atol(a.c_str() + a.find("\"token\":") + 8);
  CHECK(R.err("{\"id\":3,\"cmd\":\"fire\",\"token\":1}") == "BAD_TOKEN", "wrong token refused");
  CHECK(R.err("{\"id\":3,\"cmd\":\"fire\",\"token\":" + std::to_string(tok) + "}") == "NOT_ARMED", "a wrong token burns the arm");
  a = R.cmd("{\"id\":2,\"cmd\":\"arm\"}"); tok = atol(a.c_str() + a.find("\"token\":") + 8);
  R.run(3100);
  CHECK(R.err("{\"id\":3,\"cmd\":\"fire\",\"token\":" + std::to_string(tok) + "}") == "TOKEN_EXPIRED", "token expires after 3 s");
  a = R.cmd("{\"id\":2,\"cmd\":\"arm\"}"); tok = atol(a.c_str() + a.find("\"token\":") + 8);
  CHECK(R.ok("{\"id\":4,\"cmd\":\"fire\",\"token\":" + std::to_string(tok) + "}"), "fire with the token");
  R.until([&] { return R.evt("FIRED") || R.B.state == gb::S_FAULT; }, 20000);
  CHECK(R.evt("FIRED") && S.shots == 1, "FIRED event, one shot");
  R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 20000);
  CHECK(R.B.phase == gb::PH_REST && R.evt("RESET"), "reset to rest");
  CHECK(fabs(S.cockPh) < 8 && !S.fired, "sim train back at rest");
  CHECK(!R.B.aimed, "aim must be redone after a shot");
  CHECK(R.err("{\"id\":5,\"cmd\":\"fire\",\"token\":" + std::to_string(tok) + "}") == "NOT_ARMED", "token single use");
}

static void testFireRefusals() {
  T("fire refusals");
  Rig R; R.ok("{\"id\":1,\"cmd\":\"cal\",\"what\":\"cock_rest\"}");
  R.homeAimAndCock();
  S.seatAtLine = -1; S.lineOut = 5;            // dart not seated
  CHECK(R.err("{\"id\":1,\"cmd\":\"arm\"}") == "NOT_SEATED", "not seated");
  S.seatAtLine = 0; S.lineOut = 0; R.run(50);
  R.B.aimed = false;
  CHECK(R.err("{\"id\":1,\"cmd\":\"arm\"}") == "NOT_AIMED", "not aimed");
  R.B.aimed = true;
  { std::string a = R.cmd("{\"id\":2,\"cmd\":\"arm\"}"); long tok = atol(a.c_str() + a.find("\"token\":") + 8);
    S.lineOut = 3; R.run(50);                  // dart pulled out between arm and fire
    CHECK(R.err("{\"id\":3,\"cmd\":\"fire\",\"token\":" + std::to_string(tok) + "}") == "NOT_SEATED", "rechecked at fire"); }
  S.lineOut = 0; R.run(50);
  CHECK(R.err("{\"id\":1,\"cmd\":\"mode\",\"to\":\"NEUTRAL\"}") == "LAUNCH_BUSY", "no shift while cocked (no sear: the worm holds the spring)");
  CHECK(R.ok("{\"id\":1,\"cmd\":\"cal\",\"what\":\"decock\"}"), "decock");
  R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 30000);
  CHECK(R.B.phase == gb::PH_REST && S.shots == 0 && S.cockPh < S.c.homeOpenDeg && !R.B.aimKnown, "decocked to rest without a shot, aim to redo");
  CHECK(R.toMode("NEUTRAL"), "then the shift is allowed");
  CHECK(R.err("{\"id\":1,\"cmd\":\"arm\"}") == "NOT_IN_LAUNCH", "not in LAUNCH");
  CHECK(R.err("{\"id\":1,\"cmd\":\"cock\"}") == "NOT_IN_LAUNCH", "cock outside LAUNCH");
}

static void testNoRelease() {
  T("fire: sear never trips = BAD_STATE, launch train held");
  Rig R; R.ok("{\"id\":1,\"cmd\":\"cal\",\"what\":\"cock_rest\"}"); R.homeAimAndCock();
  S.lineOut = 0; S.searBroken = true; R.run(50);
  std::string a = R.cmd("{\"id\":2,\"cmd\":\"arm\"}"); long tok = atol(a.c_str() + a.find("\"token\":") + 8);
  R.ok("{\"id\":4,\"cmd\":\"fire\",\"token\":" + std::to_string(tok) + "}");
  R.until([&] { return R.B.state == gb::S_FAULT; }, 20000);
  CHECK(R.B.fault == gb::F_BAD_STATE, "BAD_STATE");
  CHECK(R.B.drumPos() == gb::DP_LAUNCH, "drum held in LAUNCH (train loaded, worm holds it)");
  CHECK(R.lastEvt("MODE").find("DRUM_HELD") != std::string::npos, "MODE DRUM_HELD event says why");
}

static void testWrapReelUnwrap() {
  T("wrap, reel until seated, unwrap");
  Rig R; R.toMode("REEL");
  S.lineOut = 6; S.seatAtLine = 0;
  CHECK(R.err("{\"id\":1,\"cmd\":\"reel\",\"until\":\"seated\"}") == "NOT_WRAPPED", "reel refused without wraps");
  CHECK(R.ok("{\"id\":1,\"cmd\":\"wrap\"}"), "wrap");
  CHECK(R.err("{\"id\":2,\"cmd\":\"mode\",\"to\":\"LAUNCH\"}") == "BUSY", "busy while wrapping");
  R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 60000);
  CHECK(R.evt("WRAPPED") && R.B.wraps == R.B.c.wrapsFull, "WRAPPED with 4 wraps");
  CHECK(fabs(S.drumDeg - 338) < 6, "drum at the WRAPPED stop point");
  CHECK(S.wrapsTrue() == R.B.c.wrapsFull, "true wraps match");
  CHECK(R.B.drumPos() == gb::DP_WRAPPED, "drum_pos WRAPPED");
  CHECK(R.err("{\"id\":2,\"cmd\":\"mode\",\"to\":\"LAUNCH\"}") == "WRAPS_ON", "no mode change with wraps on");
  S.out.clear(); CHECK(R.ok("{\"id\":2,\"cmd\":\"wrap\"}") && R.evt("WRAPPED"), "wrap when wrapped: ok at once");
  CHECK(R.err("{\"id\":3,\"cmd\":\"reel\",\"duty\":1.5}") == "BAD_ARG", "duty over 1");
  CHECK(R.err("{\"id\":3,\"cmd\":\"reel\",\"until\":\"forever\"}") == "BAD_ARG", "bad until");
  R.B.lineM = 6; R.B.lineKnown = true;
  CHECK(R.ok("{\"id\":3,\"cmd\":\"reel\",\"duty\":0.9,\"max_n\":30,\"until\":\"seated\"}"), "reel until seated");
  R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 400000);
  CHECK(R.evt("SEATED") && S.seated(), "SEATED");
  CHECK(R.B.lineM == 0, "line_m zeroed at seat");
  CHECK(R.ok("{\"id\":4,\"cmd\":\"unwrap\"}"), "unwrap");
  R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 60000);
  CHECK(R.evt("UNWRAPPED") && R.B.wraps == 0 && R.B.drumPos() == gb::DP_REEL, "UNWRAPPED at REEL");
  CHECK(S.wrapsTrue() == 0, "true wraps 0");
  CHECK(R.toMode("LAUNCH"), "then LAUNCH is allowed");
}

static void testTow() {
  T("tow: tension limit by current, hold");
  Rig R; R.toMode("REEL"); R.ok("{\"id\":1,\"cmd\":\"wrap\"}"); R.until([&] { return R.idle(); }, 60000);
  S.lineOut = 8; S.anchorTautAt = 7.5; S.anchorK = 60;
  R.B.c.kNperA = (float)(1.0 / (S.capstanR / S.c.capstanPerWiper) * S.stallNm / (S.stallA - S.freeA));   // matched constants
  R.B.c.freeA = 1.5f + (float)((S.reelFreeNm + 0.2) / S.stallNm * (S.stallA - S.freeA));
  CHECK(R.ok("{\"id\":1,\"cmd\":\"reel\",\"duty\":1,\"max_n\":40,\"until\":\"tension\"}"), "tow to 40 N");
  R.until([&] { return R.B.state == gb::S_HOLD || R.B.state == gb::S_FAULT; }, 400000);
  CHECK(R.B.state == gb::S_HOLD && R.evt("LIMIT"), "holding at the limit");
  printf("    true tension %.1f N, estimate %.1f N\n", S.tension(), R.B.tensionN);
  CHECK(S.tension() > 25 && S.tension() < 60, "true tension near 40 N");
  CHECK(S.wDir == 0, "motor off while holding (worm holds)");
  CHECK(R.err("{\"id\":1,\"cmd\":\"unwrap\"}") == "HOLDING", "no unwrap while holding");
  CHECK(R.ok("{\"id\":1,\"cmd\":\"stop\"}") && R.idle(), "stop leaves hold");
}

static void testTowHx() {
  T("tow with the HX711: limit, resume when the line slackens");
  Rig R; S.hxFitted = true; R.run(100);
  R.ok("{\"id\":1,\"cmd\":\"cal\",\"what\":\"tare\"}");
  R.toMode("REEL"); R.ok("{\"id\":1,\"cmd\":\"wrap\"}"); R.until([&] { return R.idle(); }, 60000);
  S.lineOut = 8; S.anchorTautAt = 7.8; S.anchorK = 80;
  R.ok("{\"id\":1,\"cmd\":\"reel\",\"duty\":1,\"max_n\":30,\"until\":\"tension\"}");
  R.until([&] { return R.B.state == gb::S_HOLD || R.B.state == gb::S_FAULT; }, 400000);
  CHECK(R.B.state == gb::S_HOLD && S.tension() >= 29 && S.tension() < 36, "holds at 30 N by the load cell");
  char tel[300]; R.B.telemetry(tel, sizeof tel);
  CHECK(std::string(tel).find("\"tension_n\":null") == std::string::npos, "tension_n reported with a load cell");
  S.anchorTautAt -= 0.2;                       // anchor crept: line slackens
  R.until([&] { return R.B.state == gb::S_REEL; }, 2000);
  CHECK(R.B.state == gb::S_REEL, "winds again below 60% of the limit");
  R.until([&] { return R.B.state == gb::S_HOLD; }, 60000);
  CHECK(R.B.state == gb::S_HOLD, "and holds again");
}

static void testStallOvercurrent() {
  T("stall and over-current");
  { Rig R; R.ok("{\"id\":1,\"cmd\":\"cal\",\"what\":\"cock_rest\"}"); R.toMode("LAUNCH");
    S.extraLoadNm = 30; R.ok("{\"id\":1,\"cmd\":\"cock\"}");
    R.until([&] { return R.B.state == gb::S_FAULT; }, 10000);
    CHECK(R.B.fault == gb::F_STALL || R.B.fault == gb::F_OVERCURRENT, "jammed launch train: STALL or OVERCURRENT");
    CHECK(R.evt("FAULT") && S.wDir == 0, "FAULT event, wiper off");
    CHECK(R.B.drumPos() == gb::DP_LAUNCH, "partly cocked: the drum holds LAUNCH"); }
  { Rig R; R.toMode("REEL"); R.ok("{\"id\":1,\"cmd\":\"wrap\"}"); R.until([&] { return R.idle(); }, 60000);
    S.extraLoadNm = 14.5; R.ok("{\"id\":1,\"cmd\":\"reel\",\"duty\":1,\"max_n\":100,\"until\":\"seated\"}");
    R.until([&] { return R.B.state == gb::S_FAULT || R.B.state == gb::S_IDLE; }, 10000);
    CHECK(R.B.state == gb::S_FAULT, "stalled retrieve: fault");
    CHECK(R.B.drumPos() == gb::DP_WRAPPED, "wraps on: the drum holds (capstan keeps the line)"); }
}

static void testAngleSensorFault() {
  T("AS5600 lost while running");
  Rig R; R.ok("{\"id\":1,\"cmd\":\"cal\",\"what\":\"cock_rest\"}"); R.toMode("LAUNCH");
  R.ok("{\"id\":1,\"cmd\":\"cock\"}"); R.run(500);
  S.encMissing = true;
  R.until([&] { return R.B.state == gb::S_FAULT; }, 2000);
  CHECK(R.B.fault == gb::F_ANGLE_SENSOR && S.wDir == 0, "ANGLE_SENSOR, wiper off");
  CHECK(R.err("{\"id\":1,\"cmd\":\"clear\"}") == "STILL_PRESENT", "clear refused while missing");
  S.encMissing = false; R.run(100);
  CHECK(R.ok("{\"id\":1,\"cmd\":\"clear\"}"), "clear when back");
  CHECK(R.B.phase == gb::PH_COCKING || R.B.phase == gb::PH_UNKNOWN, "phase not trusted as rest");
  { Rig Q; S.encMissing = true; Q.run(100); Q.toMode("LAUNCH");
    CHECK(Q.err("{\"id\":1,\"cmd\":\"cock\"}") == "ANGLE_SENSOR", "wiper commands refused without the AS5600"); }
  { Rig Q; S.encGlitchEvery = 37; Q.ok("{\"id\":1,\"cmd\":\"cal\",\"what\":\"cock_rest\"}");
    CHECK(Q.homeAimAndCock(), "occasional I2C glitches are ignored");
    CHECK(fabs(S.cockPh - Q.B.c.cockEndDeg) < 8, "cock end still right with glitches"); }
}

static void testBrownout() {
  T("brownout and no battery");
  { Rig R; R.ok("{\"id\":1,\"cmd\":\"cal\",\"what\":\"cock_rest\"}"); R.toMode("LAUNCH");
    R.ok("{\"id\":1,\"cmd\":\"cock\"}"); R.run(400); S.vbatOverride = 9.0;
    R.until([&] { return R.B.state == gb::S_FAULT; }, 3000);
    CHECK(R.B.fault == gb::F_BROWNOUT, "BROWNOUT under load"); }
  { Rig R; S.vbatOverride = 0.3; R.run(50);
    CHECK(R.err("{\"id\":1,\"cmd\":\"mode\",\"to\":\"REEL\"}") == "NO_POWER", "no battery: refused"); }
}

static void testStopEstop() {
  T("stop and estop");
  { Rig R; R.ok("{\"id\":1,\"cmd\":\"mode\",\"to\":\"REEL\"}"); R.run(2500);
    CHECK(R.ok("{\"id\":2,\"cmd\":\"stop\"}") && S.dDir == 0 && R.idle(), "stop: drum holds where it is");
    CHECK(R.B.drumPos() == gb::DP_BETWEEN, "between detents");
    CHECK(R.evt("ABORTED"), "ABORTED event");
    CHECK(R.toMode("REEL"), "can continue from between"); }
  { Rig R; R.toMode("REEL"); R.ok("{\"id\":2,\"cmd\":\"estop\"}");
    R.until([&] { return R.idle(); }, 20000);
    CHECK(R.B.drumPos() == gb::DP_NEUTRAL, "estop: drum to NEUTRAL"); }
  { Rig R; R.toMode("REEL"); R.ok("{\"id\":1,\"cmd\":\"wrap\"}"); R.until([&] { return R.idle(); }, 60000);
    R.ok("{\"id\":2,\"cmd\":\"estop\"}"); R.run(500);
    CHECK(R.B.drumPos() == gb::DP_WRAPPED && R.lastEvt("MODE").find("DRUM_HELD") != std::string::npos, "estop with wraps on: drum held"); }
}

static void testJog() {
  T("jog: deadman, refusals");
  Rig R; R.ok("{\"id\":1,\"cmd\":\"cal\",\"what\":\"cock_rest\"}"); R.toMode("LAUNCH");
  CHECK(R.ok("{\"id\":1,\"cmd\":\"jog\",\"motor\":\"wiper\",\"dir\":-1,\"ms\":300}"), "jog wiper (aim direction)");
  R.run(100); CHECK(S.wDir == -1, "running");
  R.run(400); CHECK(S.wDir == 0 && R.idle(), "stops after ms");
  CHECK(R.ok("{\"id\":1,\"cmd\":\"jog\",\"motor\":\"wiper\",\"dir\":-1,\"ms\":99999}"), "long jog accepted");
  R.run(1600); CHECK(S.wDir == 0, "capped at 1.5 s");
  R.homeAimAndCock();
  CHECK(R.err("{\"id\":1,\"cmd\":\"jog\",\"motor\":\"wiper\",\"dir\":1,\"ms\":300}") == "WOULD_FIRE", "forward jog refused when cocked");
  CHECK(R.err("{\"id\":1,\"cmd\":\"jog\",\"motor\":\"wiper\",\"dir\":-1,\"ms\":300}") == "NOT_AT_REST", "backward jog refused when cocked (spring would unwind)");
  CHECK(R.err("{\"id\":1,\"cmd\":\"jog\",\"motor\":\"arm\",\"dir\":1}") == "BAD_ARG", "bad motor");
  CHECK(R.err("{\"id\":1,\"cmd\":\"jog\",\"motor\":\"drum\",\"dir\":2}") == "BAD_ARG", "bad dir");
  CHECK(R.ok("{\"id\":1,\"cmd\":\"jog\",\"motor\":\"drum\",\"dir\":1,\"ms\":600}"), "jog drum");
  R.run(800); CHECK(S.dDir == 0 && R.B.drumPos() == gb::DP_BETWEEN, "drum stopped between");
}

static void testPowerLoss() {
  T("power loss mid-phase: recovery");
  // 1: power cut while the drum moves -> the code says the sector, the move can be repeated
  { Rig R; R.ok("{\"id\":1,\"cmd\":\"mode\",\"to\":\"REEL\"}"); R.run(2500);
    CHECK(S.nvsHas && (S.nvs.moving & 2), "record says moving during the drum move");
    S.out.clear(); S.dDir = 0; S.wDir = 0;
    Rig R2(false);
    CHECK(R2.B.drumKnown && R2.B.drumPos() == gb::DP_BETWEEN, "drum sector known from the code after the cut");
    CHECK(R2.toMode("REEL"), "the move completes after the cut"); }
  // 2: power cut mid-cock -> phase UNKNOWN, cock runs until the sear closes
  { Rig R; R.ok("{\"id\":1,\"cmd\":\"cal\",\"what\":\"cock_rest\"}"); R.toMode("LAUNCH"); R.ok("{\"id\":1,\"cmd\":\"cock\"}"); R.run(2500);
    double ph = S.cockPh; S.wDir = 0;
    Rig R2(false);
    CHECK(R2.B.phase == gb::PH_UNKNOWN && ph > 10, "phase UNKNOWN after a cut mid-cock");
    CHECK(R2.err("{\"id\":1,\"cmd\":\"aim\",\"deg\":30}") == "PHASE_UNKNOWN", "aim refused with the phase unknown");
    CHECK(R2.err("{\"id\":1,\"cmd\":\"mode\",\"to\":\"NEUTRAL\"}") == "LAUNCH_BUSY", "no mode change with the launch train loaded");
    CHECK(R2.err("{\"id\":1,\"cmd\":\"cock\"}") == "LOADED_UNKNOWN", "cock refused: carriage drawn, angle unknown");
    CHECK(R2.ok("{\"id\":1,\"cmd\":\"cal\",\"what\":\"decock\"}"), "decock");
    R2.until([&] { return R2.idle() || R2.B.state == gb::S_FAULT; }, 40000);
    CHECK(R2.B.phase == gb::PH_REST && S.shots == 0, "decocked to rest, no shot");
    R2.ok("{\"id\":1,\"cmd\":\"cock\"}"); R2.until([&] { return R2.idle() || R2.B.state == gb::S_FAULT; }, 40000);
    CHECK(R2.B.phase == gb::PH_COCKED && fabs(S.cockPh - R2.B.c.cockEndDeg) < 8 && S.shots == 0, "then cocks normally"); }
  // 2b: cut early in the cock (carriage still home) -> cock creeps until SW_HOME opens, then knows the angle
  { Rig R; R.toMode("LAUNCH"); R.ok("{\"id\":1,\"cmd\":\"cock\"}"); R.until([&] { return S.cockPh > 1.0; }, 5000); S.wDir = 0;
    Rig R2(false);
    CHECK(R2.B.phase == gb::PH_UNKNOWN, "phase UNKNOWN");
    CHECK(R2.ok("{\"id\":1,\"cmd\":\"cock\"}"), "cock (recovery creep)");
    R2.until([&] { return R2.idle() || R2.B.state == gb::S_FAULT; }, 40000);
    CHECK(R2.B.phase == gb::PH_COCKED && fabs(S.cockPh - R2.B.c.cockEndDeg) < 8 && S.shots == 0, "cocked at the right angle, no shot"); }
  // 3: power cut while cocked and idle -> stays known (record + sear contact)
  { Rig R; R.ok("{\"id\":1,\"cmd\":\"cal\",\"what\":\"cock_rest\"}"); R.homeAimAndCock(); R.run(500);
    Rig R2(false);
    CHECK(R2.B.phase == gb::PH_COCKED && R2.B.aimKnown, "idle cut: cocked and aim known");
    CHECK(fabs(R2.B.pitch() - R.B.pitch()) < 0.3, "pitch restored"); }
  // 4: someone turned the wiper by hand while off -> aim unknown
  { Rig R; R.ok("{\"id\":1,\"cmd\":\"cal\",\"what\":\"cock_rest\"}"); R.homeAimAndCock(); R.run(500);
    S.wiperDeg += 40;
    Rig R2(false);
    CHECK(!R2.B.aimKnown, "wiper moved while off: aim unknown"); }
  // 5: cut mid-wrap -> sector WRAPPING known, wraps estimated, mode refused, unwrap works
  { Rig R; R.toMode("REEL"); R.ok("{\"id\":1,\"cmd\":\"wrap\"}"); R.run(12000); S.dDir = 0;
    Rig R2(false);
    CHECK(R2.B.wrapsKnown && R2.B.sec == gb::SEC_WRAPPING, "WRAPPING sector after the cut");
    CHECK(R2.err("{\"id\":1,\"cmd\":\"mode\",\"to\":\"LAUNCH\"}") == "WRAPS_ON", "mode refused");
    CHECK(R2.ok("{\"id\":1,\"cmd\":\"unwrap\"}"), "unwrap");
    R2.until([&] { return R2.idle() || R2.B.state == gb::S_FAULT; }, 60000);
    CHECK(R2.B.drumPos() == gb::DP_REEL && R2.B.wraps == 0 && S.wrapsTrue() == 0, "back at REEL, no wraps"); }
  // 6: corrupt record -> nothing trusted
  { Rig R; R.run(200); S.nvs.crc ^= 1; Rig R2(false);
    CHECK(R2.B.phase == gb::PH_UNKNOWN && R2.evt("BOOT"), "corrupt record ignored"); }
}

static void testSaveGate() {
  T("motion waits for the moving record to reach flash (the flash write stalls the tick)");
  Rig R; R.B.saveAck = true; R.B.saveDoneSeq = R.B.last.seq;
  R.ok("{\"id\":1,\"cmd\":\"mode\",\"to\":\"REEL\"}"); R.run(300);
  CHECK(S.wDir == 0 && S.dDir == 0, "no motor starts before the record is written");
  R.B.saveDoneSeq = R.B.last.seq; R.run(300);
  CHECK(S.wDir != 0 || S.dDir != 0, "starts once the sketch confirms the write");
}

static void testPersistWear() {
  T("persistence: no flash writes while idle");
  Rig R; R.run(1000); int s0 = S.saves; R.run(5000);
  CHECK(S.saves == s0, "idle: zero writes");
  R.toMode("REEL"); CHECK(S.saves - s0 <= 3, "one move: at most 3 writes");
}

static void testHostTimeout() {
  T("host watchdog (opt in)");
  Rig R; R.toMode("REEL"); R.ok("{\"id\":1,\"cmd\":\"wrap\"}"); R.until([&] { return R.idle(); }, 60000);
  R.ok("{\"id\":1,\"cmd\":\"set\",\"key\":\"host_timeout_s\",\"val\":2}");
  S.lineOut = 10; R.ok("{\"id\":2,\"cmd\":\"reel\",\"until\":\"stop\"}");
  R.run(1500); CHECK(S.wDir == 1, "running while the host is quiet for 1.5 s");
  R.ok("{\"id\":3,\"cmd\":\"ping\"}"); R.run(1500); CHECK(S.wDir == 1, "a ping keeps it going");
  R.run(1000); CHECK(S.wDir == 0 && R.idle() && R.lastEvt("ABORTED").find("host silent") != std::string::npos, "stops 2 s after the last line");
}

static void testTelemetryRate() {
  T("telemetry rate");
  Rig R; int n = 0; for (uint32_t t = 0; t < 1000; t += 5) { R.tick(); if (R.B.telDue(S.t)) n++; }
  CHECK(n >= 9 && n <= 11, "10 Hz idle");
  R.ok("{\"id\":1,\"cmd\":\"mode\",\"to\":\"REEL\"}"); n = 0;
  for (uint32_t t = 0; t < 1000; t += 5) { R.tick(); if (R.B.telDue(S.t)) n++; }
  CHECK(n >= 19 && n <= 21, "20 Hz moving");
}

static void testFullMission() {
  T("full mission: launch, payout, reel mode, wrap, tow, retrieve, unwrap, launch again");
  Rig R; R.ok("{\"id\":1,\"cmd\":\"cal\",\"what\":\"cock_rest\"}");
  S.lineOut = 0; S.seatAtLine = 0;
  CHECK(R.homeAimAndCock(), "cocked and aimed");
  std::string a = R.cmd("{\"id\":2,\"cmd\":\"arm\"}"); long tok = atol(a.c_str() + a.find("\"token\":") + 8);
  R.ok("{\"id\":4,\"cmd\":\"fire\",\"token\":" + std::to_string(tok) + "}");
  R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 30000);
  CHECK(S.shots == 1 && R.B.phase == gb::PH_REST, "shot and reset");
  CHECK(R.toMode("REEL"), "to REEL");
  R.ok("{\"id\":5,\"cmd\":\"wrap\"}"); R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 60000);
  CHECK(R.B.wraps == R.B.c.wrapsFull, "wrapped");
  R.ok("{\"id\":6,\"cmd\":\"set\",\"key\":\"line_m\",\"val\":8}");
  S.anchorTautAt = 7.6; S.anchorK = 50;
  R.B.c.kNperA = (float)(1.0 / (S.capstanR / S.c.capstanPerWiper) * S.stallNm / (S.stallA - S.freeA));
  R.B.c.freeA = 1.5f + (float)((S.reelFreeNm + 0.2) / S.stallNm * (S.stallA - S.freeA));
  R.ok("{\"id\":7,\"cmd\":\"reel\",\"duty\":1,\"max_n\":30,\"until\":\"tension\"}");
  R.until([&] { return R.B.state == gb::S_HOLD || R.B.state == gb::S_FAULT; }, 400000);
  CHECK(R.B.state == gb::S_HOLD, "towed to the limit");
  R.ok("{\"id\":8,\"cmd\":\"stop\"}");
  S.anchorTautAt = -1; S.seatAtLine = 0;       // dart released
  R.ok("{\"id\":9,\"cmd\":\"reel\",\"duty\":0.8,\"max_n\":30,\"until\":\"seated\"}");
  R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 400000);
  CHECK(R.evt("SEATED"), "retrieved and seated");
  R.ok("{\"id\":10,\"cmd\":\"unwrap\"}"); R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 60000);
  CHECK(R.B.wraps == 0, "unwrapped");
  CHECK(R.toMode("LAUNCH"), "back to LAUNCH");
  printf("    before 2nd cock: fw cock %.1f sim cock %.1f fired %d\n", R.B.cockPhase, S.cockPh, S.fired);
  R.ok("{\"id\":11,\"cmd\":\"cock\"}"); R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 30000);
  printf("    after: fw cock %.1f sim cock %.1f\n", R.B.cockPhase, S.cockPh);
  CHECK(R.B.phase == gb::PH_COCKED, "cocked again for the next shot");
  CHECK(R.B.fault == gb::F_NONE, "no fault in the whole mission");
  size_t maxLen = 0; for (auto& l : S.out) maxLen = l.size() > maxLen ? l.size() : maxLen;
  CHECK(maxLen <= 256, "every event line within 256 bytes");
}

int main(int argc, char** argv) {
  if (argc > 1) { std::string w = argv[1];
    if (w == "cock") testCockFireReset(); else if (w == "aim") testAim(); else if (w == "wrap") testWrapReelUnwrap(); else if (w == "jog") testJog(); else if (w == "drum") testDrumFaults(); else if (w == "power") testPowerLoss(); else if (w == "stall") testStallOvercurrent(); else if (w == "enc") testAngleSensorFault(); else if (w == "tow") testTow(); else if (w == "mission") testFullMission();
    printf("%d passed, %d failed\n", pass, fail); return fail ? 1 : 0; }
  testJson(); testSwitches(); testAngle(); testWiperDrive(); testPersist();
  testProtocolBasics(); testTelemetryWorstCase(); testModeChanges(); testDrumFaults(); testAim(); testCockFireReset(); testFireRefusals();
  testNoRelease(); testWrapReelUnwrap(); testTow(); testTowHx(); testStallOvercurrent(); testAngleSensorFault(); testBrownout();
  testStopEstop(); testJog(); testPowerLoss(); testSaveGate(); testPersistWear(); testTelemetryRate(); testHostTimeout(); testFullMission();
  printf("\n%d checks, %d passed, %d failed: %s\n", pass + fail, pass, fail, fail ? "FAILURES" : "ALL PASS");
  return fail ? 1 : 0;
}
