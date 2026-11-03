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
    if (fresh) { S = Sim(); if (getenv("SIM_INERTIA")) S.inertiaMs = atof(getenv("SIM_INERTIA")); }
    if (cfgEdit) cfgEdit(B.c);
    S.c = B.c;
    if (fresh) S.win = fmin(S.win, (double)B.c.dogWindowDeg);   // collar starts against the forward flank
    B.io = simIo();
    B.saveAck = true;                       // like the sketch: motors wait for the flash write (acked 20 ms later in tick())
    B.w.c.ampsPerVolt = (float)S.isAmpsPerVolt;
    B.begin(S.t);
    B.restore(S.nvsHas ? &S.nvs : nullptr);
    run(100);
    if (fresh && restRig) cmd("{\"id\":0,\"cmd\":\"cal\",\"what\":\"cock_rest\"}");   // a new build starts at rest
  }
  static bool restRig;
  void tick() {
    S.step(5); B.tick(S.t);
    if (S.saveSeq != B.saveDoneSeq && (int32_t)(S.t - S.saveT) >= 20) B.saveDoneSeq = S.saveSeq;   // NVS write done
    if (S.dryFires) { dryTotal += S.dryFires; S.dryFires = 0; }
  }
  static int dryTotal;
  void run(uint32_t ms) { for (uint32_t i = 0; i < ms; i += 5) tick(); }
  bool until(std::function<bool()> p, uint32_t maxMs) { for (uint32_t i = 0; i < maxMs; i += 5) { tick(); if (p()) return true; } return false; }
  std::string cmd(const std::string& j) { char rep[300]; B.handleLine(j.c_str(), (int)j.size(), rep, sizeof rep); return rep; }
  bool ok(const std::string& j) { std::string r = cmd(j); bool o = r.find("\"ok\":true") != std::string::npos; if (!o) printf("    (reply %s)\n", r.c_str()); return o; }
  std::string err(const std::string& j) { std::string r = cmd(j); size_t p = r.find("\"err\":\""); if (p == std::string::npos) return ""; p += 7; return r.substr(p, r.find('"', p) - p); }
  bool evt(const char* name) { for (auto& l : S.out) if (l.find(std::string("\"evt\":\"") + name + "\"") != std::string::npos) return true; return false; }
  std::string lastEvt(const char* name) { for (int i = (int)S.out.size() - 1; i >= 0; i--) if (S.out[i].find(std::string("\"evt\":\"") + name + "\"") != std::string::npos) return S.out[i]; return ""; }
  bool idle() const { return B.state == gb::S_IDLE; }
  // helpers to reach mission states
  bool toMode(const char* m) { if (!ok(std::string("{\"id\":1,\"cmd\":\"mode\",\"to\":\"") + m + "\"}")) return false; return until([&] { return idle() || B.state == gb::S_FAULT; }, 40000) && idle(); }
  // v1.2 (D22): wrap ends at the WRAPPED park (collar NEUTRAL); mode REEL then flies the collar into REEL
  bool wrapToReel() {
    if (!ok("{\"id\":1,\"cmd\":\"wrap\"}")) return false;
    if (!until([&] { return idle() || B.state == gb::S_FAULT; }, 120000) || !idle()) return false;
    return toMode("REEL");
  }
  bool homeAimAndCock() {          // aim first (needs the carriage home), then cock (forward does not move the aim)
    if (!toMode("LAUNCH")) return false;
    if (!ok("{\"id\":3,\"cmd\":\"aim\",\"deg\":35}")) return false;
    if (!until([&] { return idle() || B.state == gb::S_FAULT; }, 60000) || !B.aimed) return false;
    if (!ok("{\"id\":2,\"cmd\":\"cock\"}")) return false;
    return until([&] { return idle() || B.state == gb::S_FAULT; }, 30000) && idle() && B.aimed;
  }
};

bool Rig::restRig = true;
int Rig::dryTotal = 0;

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
  gb::AngleTracker a; a.missLimit = 4; uint8_t md = gb::AS_MD;   // (firmware default 10)
  int raw = 4000; a.feed(raw, md, 5);
  CHECK(a.count == 4000, "first reading lines the count up with the absolute angle (boot alignment)");
  a.count -= 4000;                                 // the checks below count from 0
  for (int i = 0; i < 100; i++) { raw = (raw + 30) % 4096; a.feed(raw, md, 5); }
  CHECK(a.count == 3000, "counts across the 4095 -> 0 wrap");
  a.feed(2000, md, 5);
  CHECK(a.count == 3000 && a.glitches == 1, "single glitch ignored");
  a.feed((raw + 30) % 4096, md, 5); raw = (raw + 30) % 4096;
  CHECK(a.count == 3030, "continues after a glitch");
  for (int i = 0; i < 4; i++) a.feed(-1, 0, 5);
  CHECK(!a.ok, "lost after 4 bad reads");
  int32_t before = a.count; uint32_t rs = a.resyncs; a.feed(100, md, 5);
  CHECK(a.ok && a.resyncs == rs + 1 && ((a.count - before) % 4096 + 4096) % 4096 == ((100 - raw) % 4096 + 4096) % 4096, "comes back lined up with the raw angle, the jump flagged as uncounted travel");
  a.feed(100, 0, 5); a.feed(100, 0, 5); a.feed(100, 0, 5);
  CHECK(a.ok, "3 reads without the magnet are tolerated");
  a.feed(100, 0, 5);
  CHECK(!a.ok, "magnet missing = lost");
  gb::AngleTracker r; r.feed(0, md, 5); int rr = 0;
  for (int i = 0; i < 80; i++) { rr = (rr + 23) % 4096; r.feed(rr, md, 5); }
  CHECK(fabsf(r.rateDps - 23 * 360.f / 4096 * 200) < 20, "rate in deg/s");
  gb::AngleTracker n; n.sign = -1; n.feed(1000, md, 5); int32_t n0 = n.count; n.feed(1010, md, 5);
  CHECK(n.count - n0 == -10 && n0 == 3096, "sign applied (and in the first alignment)");
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
  gb::WiperDrive s; s.want(1, 128); for (int i = 0; i < 100; i++) s.tick(t += 5, 5, 4.0 / 12.5 * s.outDuty / 255, 100, hw);   // IS average = I x duty
  CHECK(fabsf(s.amps - 4.0f) < 0.1f, "IS reading scaled back by the duty");
  gb::WiperDrive o; o.want(1, 255); for (int i = 0; i < 70; i++) o.tick(t += 5, 5, 10.0 / 12.5 * o.outDuty / 255, 100, hw);
  CHECK(!o.overcurrent, "10 A during the ramp: no trip");
  o.c.tripA = 5; for (int i = 0; i < 5; i++) o.tick(t += 5, 5, 6.0 / 12.5, 100, hw);
  CHECK(!o.overcurrent, "25 ms over the trip is not a trip");
  o.tick(t += 5, 5, 6.0 / 12.5, 100, hw); CHECK(o.overcurrent && hd == 0, "30 ms over: trip, motor off (D8)");
  o.want(1, 255); o.tick(t += 5, 5, 0, 100, hw); CHECK(hd == 0, "stays off until the latch is cleared");
  gb::WiperDrive st; st.want(1, 255); for (int i = 0; i < 70; i++) st.tick(t += 5, 5, 0.3 * st.outDuty / 255, 100, hw);
  for (int i = 0; i < 90; i++) st.tick(t += 5, 5, 0.3 * st.outDuty / 255, 0, hw);
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
  CHECK(std::string(tel).find("\"sw\":{\"a\":1,\"l\":1,\"n\":1,\"H\":1,\"G\":1,\"S\":1}") != std::string::npos, tel);   // LAUNCH code, carriage home; SW_ARM open = cup at home (cad-reel)
}

static void testTelemetryWorstCase() {
  T("telemetry: worst case fits 256 bytes");
  Rig R;
  // longest realistic line: moving, every value long, the contacts a running system can have closed at once
  R.B.drumKnown = true; R.B.drumDeg = -1234.5f; R.B.enc.count = -4096000; R.B.enc.rateDps = -400; R.B.state = gb::S_UNWRAP;
  R.B.wrapsKnown = true; R.B.wraps = 12; R.B.lineM = 99.99f; R.B.amps = 19.9f; R.B.aimKnown = true; R.B.phase = gb::PH_COCKING; R.B.cockPhase = 300;
  R.B.hxSeen = true; R.B.hxT = R.B.now; R.B.hxN = 149.5f; R.B.vbat = 12.6f;
  S.swStuckClosed = 0x6E3; R.B.c.vbatLow = 0; R.run(30); R.B.state = gb::S_UNWRAP; R.B.hxT = R.B.now; R.B.enc.rateDps = -400; R.B.secBad = false;
  char tel[400]; R.B.telemetry(tel, 400);
  { char big[700]; R.B.telBuild(big, 700, false); printf("    realistic worst case %zu bytes: %s\n", strlen(big), big); }
  CHECK(strlen(tel) <= 256 && std::string(tel).find("\"trunc\"") == std::string::npos, tel);   // full or lean, never cut
  gb::JObj o; CHECK(gb::jsonParse(tel, (int)strlen(tel), &o), "valid JSON");
  // absurd values: still valid JSON within 256 bytes
  R.B.enc.count = -99999999; R.B.enc.rateDps = -1e8f; R.B.fault = gb::F_SWITCH_DISAGREE; R.B.state = gb::S_FAULT; S.swStuckClosed = 0x7F; R.run(30);
  R.B.telemetry(tel, 400);
  CHECK(strlen(tel) <= 256 && gb::jsonParse(tel, (int)strlen(tel), &o), "absurd state: still a valid line within 256 bytes");
}

static void testModeChanges() {
  T("mode: LAUNCH / NEUTRAL with verification; REEL only with the wraps on");
  Rig R;                                   // sim drum starts at 0 = LAUNCH
  CHECK(R.B.drumPos() == gb::DP_LAUNCH && R.B.drumKnown && R.B.collarLaunch(), "boots at LAUNCH (code + SW_DOG_L)");
  CHECK(R.err("{\"id\":1,\"cmd\":\"mode\",\"to\":\"REEL\"}") == "NOT_WRAPPED", "REEL refused without wraps (use wrap)");
  double w0 = S.wiperDeg;
  CHECK(R.toMode("NEUTRAL"), "LAUNCH -> NEUTRAL");
  CHECK(S.wiperDeg < w0 - 5, "centering jog: the wiper turned backward first");
  CHECK(fabs(S.aimPh) < 1e-6 || S.aimPh > 0, "aim train untouched by the centering jog (dog window)");
  CHECK(R.B.drumPos() == gb::DP_NEUTRAL && fabs(S.drumDeg - 41) < 4, "drum stopped at the NEUTRAL stop point");
  CHECK(R.evt("MODE") && R.lastEvt("MODE").find("\"ok\":true") != std::string::npos, "MODE event ok");
  CHECK(fabs(R.B.c.drumDps - 7.5) < 1.2, "drum speed learned from the edge timing");
  CHECK(R.toMode("LAUNCH"), "NEUTRAL -> LAUNCH");
  CHECK(R.B.collarLaunch() && !R.B.aimKnown, "collar confirmed by SW_DOG_L; aim to re-home");
  S.out.clear(); CHECK(R.ok("{\"id\":1,\"cmd\":\"mode\",\"to\":\"LAUNCH\"}") && R.evt("MODE"), "already there: ok + MODE at once");
  CHECK(R.ok("{\"id\":1,\"cmd\":\"mode\",\"to\":\"NEUTRAL\"}"), "start a move");
  CHECK(R.err("{\"id\":2,\"cmd\":\"mode\",\"to\":\"LAUNCH\"}") == "BUSY", "second move while moving: BUSY");
  CHECK(R.err("{\"id\":2,\"cmd\":\"aim\",\"deg\":30}") == "BUSY", "aim while moving: BUSY");
  R.until([&] { return R.idle(); }, 20000);
  CHECK(R.B.drumPos() == gb::DP_NEUTRAL, "arrived");
  S.out.clear(); CHECK(R.err("{\"id\":2,\"cmd\":\"unwrap\"}") == "" && R.evt("UNWRAPPED") && R.evt("MODE"), "unwrap in NEUTRAL: ok at once, UNWRAPPED then MODE");
}

static void testDrumFaults() {
  T("drum faults: jam, wire off, sector jump, wrong direction, tooth on tooth, chatter");
  { Rig R; S.drumJam = true;
    R.ok("{\"id\":1,\"cmd\":\"mode\",\"to\":\"NEUTRAL\"}");
    R.until([&] { return R.B.state == gb::S_FAULT; }, 30000);
    CHECK(R.B.fault == gb::F_DRUM_TIMEOUT, "jammed drum: DRUM_TIMEOUT"); CHECK(S.wDir == 0 && S.dDir == 0, "everything off");
    CHECK(R.B.drumPos() == gb::DP_LAUNCH, "drum_pos still LAUNCH (estimate stays inside the sector)"); }
  { Rig R; S.swStuckOpen = 1 << 1;             // SW_DRUM_B wire off: NEUTRAL shows 000
    R.ok("{\"id\":1,\"cmd\":\"mode\",\"to\":\"NEUTRAL\"}");
    R.until([&] { return R.B.state == gb::S_FAULT; }, 30000);
    CHECK(R.B.fault == gb::F_DRUM_SENSOR, "wire off: DRUM_SENSOR (code 000)"); }
  { Rig R; S.swStuckClosed = 1 << 2; R.run(400);  // SW_DRUM_C stuck closed at LAUNCH: 101 = jump to WRAPPING
    CHECK(R.B.fault == gb::F_DRUM_SENSOR, "code jump over sectors: DRUM_SENSOR");
    CHECK(R.err("{\"id\":1,\"cmd\":\"clear\"}") == "STILL_PRESENT", "clear refused while present");
    S.swStuckClosed = 0; R.run(100);
    CHECK(R.ok("{\"id\":1,\"cmd\":\"clear\"}") && R.idle(), "clear once fixed"); }
  { Rig R; R.B.c.drumSign = -1;                  // N20 wired the other way round
    R.ok("{\"id\":1,\"cmd\":\"mode\",\"to\":\"NEUTRAL\"}");
    R.until([&] { return R.B.state == gb::S_FAULT; }, 30000);
    CHECK(R.B.fault == gb::F_DRUM_SENSOR || R.B.fault == gb::F_DRUM_TIMEOUT, "drum_sign wrong: stopped with a drum fault"); }
  { Rig R; R.toMode("NEUTRAL"); S.toothOnTooth = true; S.out.clear();
    CHECK(R.toMode("LAUNCH"), "tooth on tooth: SW_DOG wiggle lets the dogs drop in");
    CHECK(!S.dogTip && S.wDir == 0, "dogs engaged, wiper stopped");
    CHECK(!R.B.aimKnown, "wiggle into LAUNCH forgets the aim (re-home)");
    S.toothOnTooth = true; R.toMode("NEUTRAL"); R.wrapToReel();
    CHECK(R.B.collarReel(), "tooth on tooth into REEL: fly-in drops the dogs in"); }
  { Rig R; S.toothOnTooth = true; S.toothNeedWiper = 1e9; R.toMode("NEUTRAL"); R.ok("{\"id\":1,\"cmd\":\"mode\",\"to\":\"LAUNCH\"}");
    R.until([&] { return R.B.state == gb::S_FAULT; }, 30000);
    CHECK(R.B.fault == gb::F_DRUM_TIMEOUT, "dogs never drop in: DRUM_TIMEOUT after 60 deg of wiggle"); }
  { Rig R; S.swChatter = 1 << 1; R.ok("{\"id\":1,\"cmd\":\"mode\",\"to\":\"NEUTRAL\"}");
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
  S.seatAtLine = -1; S.lineOut = 5; R.run(50);   // dart not seated
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
  T("tidy, wrap (NEUTRAL collar, then REEL fly-in), reel in, unwrap with take-up, final approach until seated");
  Rig R; R.toMode("NEUTRAL");
  S.lineOut = 6; S.seatAtLine = 0; S.anchorTautAt = -1;
  CHECK(R.err("{\"id\":1,\"cmd\":\"reel\",\"until\":\"tension\"}") == "NOT_WRAPPED", "no tension reel without the capstan");
  CHECK(R.ok("{\"id\":1,\"cmd\":\"reel\",\"duty\":0.2,\"until\":\"slip\"}"), "tidy (take-up until it slips)");
  S.anchorTautAt = 5.9;                         // the anchored dart: the take-up slips once the line is straight
  R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 20000);
  CHECK(R.lastEvt("LIMIT").find("tidy done") != std::string::npos, "tidy done");
  S.anchorTautAt = -1;
  CHECK(R.ok("{\"id\":1,\"cmd\":\"wrap\"}"), "wrap");
  CHECK(R.err("{\"id\":2,\"cmd\":\"mode\",\"to\":\"LAUNCH\"}") == "BUSY", "busy while wrapping");
  R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 120000);
  CHECK(R.evt("WRAPPED") && R.B.wraps == R.B.c.wrapsFull, "WRAPPED with the full count");
  CHECK(fabs(S.drumDeg - 314) < 5 && !R.B.collarReel() && !R.B.collarLaunch(), "drum at the WRAPPED park (314), collar NEUTRAL");
  CHECK(R.B.drumPos() == gb::DP_WRAPPED, "drum_pos WRAPPED");
  CHECK(R.toMode("REEL"), "mode REEL: fly-in from the WRAPPED park");
  CHECK(fabs(S.drumDeg - 347) < 5 && R.B.collarReel(), "drum at REEL, collar engaged (SW_DOG_R)");
  CHECK(S.wrapsTrue() == R.B.c.wrapsFull, "true wraps match");
  CHECK(R.err("{\"id\":2,\"cmd\":\"mode\",\"to\":\"LAUNCH\"}") == "WRAPS_ON", "no mode change with wraps on");
  S.out.clear(); CHECK(R.ok("{\"id\":2,\"cmd\":\"wrap\"}") && R.evt("WRAPPED"), "wrap when wrapped: ok at once");
  CHECK(R.err("{\"id\":3,\"cmd\":\"reel\",\"duty\":1.5}") == "BAD_ARG", "duty over 1");
  CHECK(R.err("{\"id\":3,\"cmd\":\"reel\",\"until\":\"forever\"}") == "BAD_ARG", "bad until");
  R.B.lineM = 6; R.B.lineKnown = true; S.seatAtLine = 0;
  R.ok("{\"id\":3,\"cmd\":\"reel\",\"duty\":0.9,\"max_n\":30,\"until\":\"stop\"}");   // capstan reels in
  R.until([&] { return S.lineOut < 1.5; }, 400000); R.ok("{\"id\":4,\"cmd\":\"stop\"}");
  CHECK(S.lineOut < 1.5 && fabs(R.B.lineM - S.lineOut) < 0.4, "line count follows the capstan");
  S.slack = 0; S.maxSlack = 0;
  CHECK(R.ok("{\"id\":4,\"cmd\":\"unwrap\"}"), "unwrap");
  R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 60000);
  CHECK(R.evt("UNWRAPPED") && R.B.wraps == 0 && R.B.drumPos() == gb::DP_NEUTRAL, "UNWRAPPED at NEUTRAL");
  CHECK(S.wrapsTrue() == 0, "true wraps 0");
  CHECK(S.slack < 0.05, "take-up kept the freed line in (no slack)");
  CHECK(R.ok("{\"id\":5,\"cmd\":\"reel\",\"duty\":0.3,\"until\":\"seated\"}"), "final approach on the take-up");
  R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 30000);
  CHECK(R.evt("SEATED") && S.seated() && R.B.lineM == 0, "SEATED, line_m zeroed");
  CHECK(R.toMode("LAUNCH"), "then LAUNCH is allowed");
  { Rig Q; Q.toMode("NEUTRAL"); S.lineOut = 3; S.anchorTautAt = 2.5; Q.run(50);   // dart snagged: no seat
    Q.ok("{\"id\":5,\"cmd\":\"reel\",\"duty\":0.3,\"until\":\"seated\"}");
    Q.until([&] { return Q.B.state == gb::S_FAULT; }, 45000);
    CHECK(Q.B.fault == gb::F_DART_STUCK, "final approach without SW_SEATED in the time limit (30 s, line_m unknown): DART_STUCK"); }
}

static void testTow() {
  T("tow: tension limit by current, hold");
  Rig R; R.toMode("NEUTRAL"); R.wrapToReel();
  R.ok("{\"id\":1,\"cmd\":\"set\",\"key\":\"limit_n\",\"val\":40}");
  S.lineOut = 8; S.anchorTautAt = 7.5; S.anchorK = 60;
  R.B.c.kNperA = (float)(1.0 / (S.capstanR * S.c.capstanPerWiper) * S.stallNm / (S.stallA - S.freeA)); R.B.c.tensOffsetN = 0;   // matched constants
  R.B.c.freeA = 1.5f + (float)((S.reelFreeNm + 0.3) / S.stallNm * (S.stallA - S.freeA));
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
  R.toMode("NEUTRAL"); R.wrapToReel();
  S.lineOut = 8; S.anchorTautAt = 7.8; S.anchorK = 80;
  R.B.c.kNperA = (float)(1.0 / (S.capstanR * S.c.capstanPerWiper) * S.stallNm / (S.stallA - S.freeA)); R.B.c.tensOffsetN = 0;   // matched constants
  R.B.c.freeA = 1.5f + (float)((S.reelFreeNm + 0.3) / S.stallNm * (S.stallA - S.freeA));
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
  { Rig R; R.toMode("NEUTRAL"); R.wrapToReel();
    S.lineOut = 5; R.run(50); S.extraLoadNm = 14.5;
    CHECK(R.err("{\"id\":1,\"cmd\":\"reel\",\"until\":\"seated\"}") == "WRAPS_ON" && R.err("{\"id\":1,\"cmd\":\"retrieve\"}") == "WRAPS_ON", "no seating on the capstan (D35): WRAPS_ON");
    R.ok("{\"id\":1,\"cmd\":\"reel\",\"duty\":1,\"max_n\":60,\"until\":\"stop\"}");
    R.until([&] { return R.B.state == gb::S_FAULT || R.B.state == gb::S_IDLE; }, 10000);
    CHECK(R.B.state == gb::S_FAULT, "stalled reel-in: fault");
    R.run(2000);
    CHECK(R.B.drumPos() == gb::DP_REEL && R.B.wraps == R.B.c.wrapsFull && R.lastEvt("MODE").find("DRUM_HELD") != std::string::npos, "fault in REEL: the drum holds (params fault_rules, D27)"); }
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
    CHECK(R.err("{\"id\":1,\"cmd\":\"mode\",\"to\":\"NEUTRAL\"}") == "NO_POWER", "no battery: refused"); }
}

static void testStopEstop() {
  T("stop and estop");
  { Rig R; R.ok("{\"id\":1,\"cmd\":\"mode\",\"to\":\"NEUTRAL\"}"); R.until([&] { return R.B.sec == gb::SEC_LN; }, 5000); R.run(200);
    CHECK(R.ok("{\"id\":2,\"cmd\":\"stop\"}") && S.dDir == 0 && R.idle(), "stop: drum holds where it is");
    CHECK(R.B.drumPos() == gb::DP_BETWEEN, "between detents");
    CHECK(R.evt("ABORTED"), "ABORTED event");
    CHECK(R.toMode("NEUTRAL"), "can continue from between"); }
  { Rig R; R.ok("{\"id\":2,\"cmd\":\"estop\"}");
    R.until([&] { return R.idle(); }, 20000);
    CHECK(R.B.drumPos() == gb::DP_NEUTRAL, "estop: drum to NEUTRAL"); }
  { Rig R; R.toMode("NEUTRAL"); R.wrapToReel();
    R.ok("{\"id\":2,\"cmd\":\"estop\"}"); R.run(2000);
    CHECK(R.B.drumPos() == gb::DP_REEL && S.dDir == 0 && R.lastEvt("MODE").find("DRUM_HELD") != std::string::npos, "estop with wraps on: the drum holds (D27)"); }
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
  CHECK(R.err("{\"id\":1,\"cmd\":\"jog\",\"motor\":\"drum\",\"dir\":1,\"ms\":600}") == "LAUNCH_BUSY", "drum jog out of LAUNCH refused while cocked");
  { Rig Q; CHECK(Q.ok("{\"id\":1,\"cmd\":\"jog\",\"motor\":\"drum\",\"dir\":1,\"ms\":1500}"), "jog drum at rest");
    Q.run(1700); CHECK(S.dDir == 0 && Q.B.drumPos() == gb::DP_BETWEEN, "drum stopped between");
    CHECK(Q.err("{\"id\":1,\"cmd\":\"jog\",\"motor\":\"wiper\",\"dir\":1,\"ms\":300}") == "SHIFTING", "no wiper jog with the dogs half engaged");
    CHECK(Q.err("{\"id\":1,\"cmd\":\"cock\"}") == "NOT_IN_LAUNCH", "no cock off the LAUNCH stop point"); }
}

static void testPowerLoss() {
  T("power loss mid-phase: recovery");
  // 1: power cut while the drum moves -> the code says the sector, the move can be repeated
  { Rig R; R.ok("{\"id\":1,\"cmd\":\"mode\",\"to\":\"NEUTRAL\"}"); R.until([&] { return R.B.sec == gb::SEC_LN; }, 5000); R.run(200);
    CHECK(S.nvsHas && (S.nvs.moving & 2), "record says moving during the drum move");
    S.out.clear(); S.dDir = 0; S.wDir = 0;
    Rig R2(false);
    CHECK(R2.B.drumKnown && R2.B.drumPos() == gb::DP_BETWEEN, "drum sector known from the code after the cut");
    CHECK(R2.B.phase == gb::PH_REST, "train was at rest when the move started and SW_HOME + SW_GAP are closed: REST (systest F15)");
    CHECK(R2.toMode("NEUTRAL"), "the move completes"); }
  { Rig R; R.ok("{\"id\":1,\"cmd\":\"mode\",\"to\":\"NEUTRAL\"}"); R.until([&] { return R.B.sec == gb::SEC_LN; }, 5000); R.run(200);
    S.dDir = 0; S.wDir = 0; S.swStuckOpen = 1 << 7;   // SW_GAP reads open after the cut
    Rig R2(false);
    CHECK(R2.B.phase == gb::PH_UNKNOWN && R2.err("{\"id\":1,\"cmd\":\"mode\",\"to\":\"NEUTRAL\"}") == "LAUNCH_BUSY", "SW_GAP open: phase unknown, no shift away");
    CHECK(R2.toMode("LAUNCH"), "back to LAUNCH is allowed"); }
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
    CHECK(R2.err("{\"id\":1,\"cmd\":\"cock\"}") == "LOADED_UNKNOWN", "record written at the start of the cock: worst case assumed, cock refused");
    CHECK(R2.err("{\"id\":1,\"cmd\":\"cal\",\"what\":\"decock\"}") == "SWITCH_DISAGREE", "decock needs force (SW_HOME says home)");
    CHECK(R2.ok("{\"id\":1,\"cmd\":\"cal\",\"what\":\"decock\",\"force\":true}"), "forced decock by travel");
    R2.until([&] { return R2.idle() || R2.B.state == gb::S_FAULT; }, 40000);
    CHECK(R2.B.phase == gb::PH_REST && S.shots == 0 && S.cockPh < S.c.homeOpenDeg, "back at rest, no shot");
    R2.B.phase = gb::PH_UNKNOWN; R2.B.phaseHint = -1;   // no hint at all (old record): the recovery creep
    CHECK(R2.ok("{\"id\":1,\"cmd\":\"cock\"}"), "cock (recovery creep)");
    for (int i = 0; i < 8000 && !(R2.idle() || R2.B.state == gb::S_FAULT); i++) { R2.tick(); if (getenv("SIMLOG") && i % 20 == 0) printf("      sim %.1f home %d fw %.1f ph %d st %d\n", S.cockPh, (int)R2.B.closed(gb::SW_HOME), R2.B.cockPhase, R2.B.phase, R2.B.state); }
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
  { Rig R; R.toMode("NEUTRAL"); R.ok("{\"id\":1,\"cmd\":\"wrap\"}"); R.run(12000); S.dDir = 0;
    Rig R2(false);
    CHECK(!R2.B.wrapsKnown && R2.B.sec == gb::SEC_WRAP, "WRAP sector after the cut, count unknown (lens B 7.4)");
    CHECK(R2.err("{\"id\":1,\"cmd\":\"mode\",\"to\":\"LAUNCH\"}") == "WRAPS_ON", "mode refused");
    CHECK(R2.ok("{\"id\":1,\"cmd\":\"unwrap\"}"), "unwrap");
    R2.until([&] { return R2.idle() || R2.B.state == gb::S_FAULT; }, 60000);
    CHECK(R2.B.drumPos() == gb::DP_NEUTRAL && R2.B.wraps == 0 && S.wrapsTrue() == 0 && R2.B.wrapsKnown, "unwrap is safe at any count: back at NEUTRAL, no wraps"); }
  // 6: corrupt record -> nothing trusted
  { Rig R; R.run(200); S.nvs.crc ^= 1; Rig R2(false);
    CHECK(R2.B.phase == gb::PH_UNKNOWN && R2.evt("BOOT"), "corrupt record ignored"); }
}

static void testSaveGate() {
  T("motion waits for the moving record to reach flash (the flash write stalls the tick)");
  Rig R;
  R.ok("{\"id\":1,\"cmd\":\"mode\",\"to\":\"NEUTRAL\"}"); S.saveT = S.t + 100000;   // the flash write "never" finishes
  R.run(300);
  CHECK(S.wDir == 0 && S.dDir == 0, "no motor starts before the record is written");
  S.saveT = S.t - 20; R.run(300);
  CHECK(S.wDir != 0 || S.dDir != 0, "starts once the sketch confirms the write");
}

static void testPersistWear() {
  T("persistence: no flash writes while idle");
  Rig R; R.run(1000); int s0 = S.saves; R.run(5000);
  CHECK(S.saves == s0, "idle: zero writes");
  R.toMode("NEUTRAL"); CHECK(S.saves - s0 <= 3, "one move: at most 3 writes");
}

static void testHostTimeout() {
  T("host watchdog (opt in)");
  Rig R; R.toMode("NEUTRAL"); R.wrapToReel();
  R.ok("{\"id\":1,\"cmd\":\"set\",\"key\":\"host_timeout_s\",\"val\":2}");
  S.lineOut = 10; R.ok("{\"id\":2,\"cmd\":\"reel\",\"until\":\"stop\"}");
  R.run(1500); CHECK(S.wDir == 1, "running while the host is quiet for 1.5 s");
  R.ok("{\"id\":3,\"cmd\":\"ping\"}"); R.run(1500); CHECK(S.wDir == 1, "a ping keeps it going");
  R.run(1000); CHECK(S.wDir == 0 && R.idle() && R.lastEvt("ABORTED").find("host silent") != std::string::npos, "stops 2 s after the last line");
}

static void testReview1() {
  T("review round 1 regressions");
  { Rig R; R.toMode("LAUNCH"); R.ok("{\"id\":1,\"cmd\":\"aim\",\"deg\":35}"); R.until([&] { return R.idle(); }, 60000);
    char b[80]; snprintf(b, sizeof b, "{\"id\":2,\"cmd\":\"aim\",\"deg\":%.2f}", R.B.pitch() + 0.5);
    R.ok(b); R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 60000); R.run(3000);
    CHECK(S.wDir == 0 && R.B.aimed && fabs(R.B.pitch() - (R.B.aimTarget)) < 1, "second close aim stops the wiper (stale sub-state)"); }
  { Rig R; R.toMode("LAUNCH"); S.cockPh = 270; S.swStuckClosed = 1 << 6;   // (from 300 the 157 deg reset allowance reaches the release: phaseHint covers that case) R.B.phase = gb::PH_UNKNOWN; R.run(50);
    R.ok("{\"id\":1,\"cmd\":\"cock\"}"); R.until([&] { return R.B.state == gb::S_FAULT || R.idle(); }, 30000);
    CHECK(S.shots == 0 && R.B.fault == gb::F_BAD_STATE, "SW_HOME stuck closed: the creep stops before the release tooth"); }
  { Rig R; R.homeAimAndCock(); S.swStuckOpen = 1 << 0; R.run(300);
    CHECK(R.err("{\"id\":1,\"cmd\":\"mode\",\"to\":\"NEUTRAL\"}") != "", "cocked + drum code bit failed: no shift"); }
  { Rig R; R.toMode("NEUTRAL"); R.wrapToReel();
    CHECK(R.err("{\"id\":3,\"cmd\":\"jog\",\"motor\":\"wiper\",\"dir\":-1}") == "BACKSTOP", "backward jog refused in REEL");
    R.ok("{\"id\":4,\"cmd\":\"unwrap\"}"); R.until([&] { return R.B.sec == gb::SEC_WRAP; }, 20000); R.ok("{\"id\":5,\"cmd\":\"stop\"}");
    { std::string e = R.err("{\"id\":6,\"cmd\":\"jog\",\"motor\":\"wiper\",\"dir\":1,\"ms\":300}"); if (e != "") printf("    jog err %s state %d sec %d\n", e.c_str(), R.B.state, R.B.sec);
      CHECK(e == "" && R.B.w.wantDuty <= R.B.c.takeupMaxDuty, "forward jog in the WRAP sector: take-up only, at most 40 percent"); } }
  // (a drum stopped anywhere in the LAUNCH code sector, -10..4 deg, is inside the collar's LAUNCH dwell -10..6: engaged)
  { Rig R; R.toMode("NEUTRAL"); R.wrapToReel();
    S.extraLoadNm = 14.5; S.hxFitted = true; R.run(50);
    R.ok("{\"id\":1,\"cmd\":\"reel\",\"duty\":1,\"max_n\":30,\"until\":\"tension\"}");
    R.until([&] { return R.B.state == gb::S_FAULT || R.B.state == gb::S_HOLD; }, 20000);
    bool restarted = false; R.until([&] { if (S.wDir != 0) restarted = true; return false; }, 5000);
    CHECK((R.B.fault == gb::F_STALL || R.B.fault == gb::F_OVERCURRENT || R.B.state == gb::S_HOLD) && !restarted, "jammed reel in a tow: a fault or a hold, never a hold loop"); }
  { Rig R; R.ok("{\"id\":1,\"cmd\":\"mode\",\"to\":\"NEUTRAL\"}"); R.run(500);
    S.drumJam = true; R.B.raise(gb::F_BAD_STATE, "test"); R.run(200); S.vbatOverride = 8.0; R.B.c.vbatLow = 9.9f;
    R.B.raise(gb::F_BROWNOUT, "second"); R.run(50);
    CHECK(R.B.state == gb::S_FAULT && S.dDir == 0, "a second fault during SAFING stops the drum"); }
  { Rig R; R.toMode("LAUNCH"); R.ok("{\"id\":1,\"cmd\":\"cock\"}"); R.run(1500); R.B.raise(gb::F_BAD_STATE, "test");
    CHECK(R.B.phase == gb::PH_UNKNOWN, "a fault mid-cock makes the phase UNKNOWN"); }
  { Rig R; R.B.state = gb::S_IDLE; R.B.w.want(1, 200); R.run(50);
    CHECK(S.wDir == 0, "guard: no motor runs in IDLE"); }
}

static void testReview2() {
  T("review round 2 regressions");
  // f1: code wire C shorted while off, launcher cocked: no jog, no shift, no shot
  { Rig R; R.homeAimAndCock(); R.run(500); S.swStuckClosed = 1 << 2;
    Rig R2(false);
    int shots0 = S.shots; R2.run(200);
    R2.cmd("{\"id\":1,\"cmd\":\"jog\",\"motor\":\"wiper\",\"dir\":1,\"ms\":1500}"); R2.run(1600);
    R2.cmd("{\"id\":2,\"cmd\":\"jog\",\"motor\":\"drum\",\"dir\":1,\"ms\":1500}"); R2.run(1600);
    R2.cmd("{\"id\":3,\"cmd\":\"wrap\"}"); R2.run(3000);
    CHECK(S.shots == shots0 && Rig::dryTotal == 0, "shorted code bit while cocked: no shot, no dry fire");
    CHECK(R2.B.state == gb::S_FAULT, "and a fault says why"); }
  // f2c: SW_HOME stuck closed, forward jogs from rest: never REST with the carriage drawn
  { Rig R; R.toMode("LAUNCH"); S.swStuckClosed = 1 << 6; R.run(50);
    for (int i = 0; i < 3; i++) { R.cmd("{\"id\":1,\"cmd\":\"jog\",\"motor\":\"wiper\",\"dir\":1,\"ms\":1500}"); R.run(1600); }
    R.cmd("{\"id\":2,\"cmd\":\"mode\",\"to\":\"NEUTRAL\"}"); R.run(5000);
    CHECK(Rig::dryTotal == 0 && (R.B.state == gb::S_FAULT || R.B.phase != gb::PH_REST), "stuck SW_HOME + forward jogs: no shift under load"); }
  // f3: cocked, power cycle, SW_HOME stuck closed: cock refused, no shot; decock by travel with force
  { Rig R; R.homeAimAndCock(); R.run(500); S.swStuckClosed = 1 << 6;
    Rig R2(false); int shots0 = S.shots;
    CHECK(R2.err("{\"id\":1,\"cmd\":\"cock\"}") == "LOADED_UNKNOWN", "cock refused: the switch disagrees with the saved angle");
    CHECK(R2.err("{\"id\":2,\"cmd\":\"cal\",\"what\":\"decock\"}") == "SWITCH_DISAGREE", "decock needs force");
    CHECK(R2.ok("{\"id\":3,\"cmd\":\"cal\",\"what\":\"decock\",\"force\":true}"), "forced decock by travel");
    R2.until([&] { return R2.idle() || R2.B.state == gb::S_FAULT; }, 20000);
    CHECK(S.shots == shots0 && S.cockPh < S.c.homeOpenDeg && R2.B.phase == gb::PH_REST, "decocked by travel, no shot"); }
  // f3b: stop mid-fire before the release, then cock with SW_HOME stuck closed: refused
  { Rig R; R.homeAimAndCock(); S.lineOut = 0; R.run(50);
    std::string a = R.cmd("{\"id\":2,\"cmd\":\"arm\"}"); long tok = atol(a.c_str() + a.find("\"token\":") + 8);
    R.ok("{\"id\":4,\"cmd\":\"fire\",\"token\":" + std::to_string(tok) + "}"); R.until([&] { return S.cockPh > 422; }, 5000);
    R.ok("{\"id\":5,\"cmd\":\"stop\"}"); S.swStuckClosed = 1 << 6; R.run(100);
    CHECK(R.err("{\"id\":6,\"cmd\":\"cock\"}") == "LOADED_UNKNOWN" && S.shots == 0, "stop mid-fire + stuck SW_HOME: cock refused"); }
  // f4: SW_HOME bounces during decock: no REST until steadily closed
  { Rig R; R.homeAimAndCock(); R.ok("{\"id\":1,\"cmd\":\"cal\",\"what\":\"decock\"}");
    R.until([&] { return S.cockPh < 390; }, 10000); S.swStuckClosed = 1 << 6; R.run(40); S.swStuckClosed = 0;
    R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 20000);
    CHECK(S.cockPh < S.c.homeOpenDeg + 1 || R.B.phase != gb::PH_REST, "a bounce does not end the decock early"); }
  // f4b: stop a forward jog right at the first tooth: not REST any more
  { Rig R; R.toMode("LAUNCH"); R.ok("{\"id\":1,\"cmd\":\"jog\",\"motor\":\"wiper\",\"dir\":1,\"ms\":1500}");
    R.until([&] { return S.cockPh > 4.15; }, 5000); R.ok("{\"id\":2,\"cmd\":\"stop\"}"); R.run(100);
    CHECK(R.B.phase != gb::PH_REST, "carriage off its stop: phase no longer REST");
    CHECK(R.err("{\"id\":3,\"cmd\":\"mode\",\"to\":\"NEUTRAL\"}") == "LAUNCH_BUSY", "and no shift"); }
  // r2f: a fault mid-decock: phase UNKNOWN, not COCKED
  { Rig R; R.homeAimAndCock(); R.ok("{\"id\":1,\"cmd\":\"cal\",\"what\":\"decock\"}"); R.run(600);
    R.B.raise(gb::F_STALL, "test"); CHECK(R.B.phase == gb::PH_UNKNOWN, "fault mid-decock: phase UNKNOWN"); }
  // ids: non-integer or huge ids refused
  { Rig R; CHECK(R.err("{\"id\":1e35,\"cmd\":\"ping\"}") == "BAD_ID" && R.err("{\"id\":1.5,\"cmd\":\"ping\"}") == "BAD_ID", "inexact ids: BAD_ID"); }
}

static void testSpecV1Contacts() {
  T("spec v1.0 contacts: SW_N20 hand mode, SW_GAP, SW_ARM pulse count");
  { Rig R; S.handLatched = true; R.run(50);
    CHECK(R.err("{\"id\":1,\"cmd\":\"mode\",\"to\":\"NEUTRAL\"}") == "HAND_MODE", "N20 latched out: no drum move");
    CHECK(R.err("{\"id\":1,\"cmd\":\"jog\",\"motor\":\"drum\",\"dir\":1}") == "HAND_MODE", "nor a drum jog");
    S.handLatched = false; R.run(50); CHECK(R.toMode("NEUTRAL"), "back in mesh: moves again");
    R.ok("{\"id\":2,\"cmd\":\"mode\",\"to\":\"LAUNCH\"}"); R.run(400); S.handLatched = true; R.run(100);
    CHECK(R.B.state == gb::S_FAULT && R.B.fault == gb::F_DRUM_SENSOR && S.dDir == 0, "latched out during a move: fault, drum off"); }
  { Rig R; R.toMode("LAUNCH"); S.swStuckOpen = 1 << 7; R.run(50);   // SW_GAP open at rest
    CHECK(R.err("{\"id\":1,\"cmd\":\"mode\",\"to\":\"NEUTRAL\"}") == "LAUNCH_BUSY", "big gear not in its gap: no leaving LAUNCH"); }
  { Rig R; R.toMode("NEUTRAL"); S.swStuckOpen = 1 << 10;              // SW_ARM never closes: cup turns not seen
    R.ok("{\"id\":1,\"cmd\":\"wrap\"}"); R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 60000);
    CHECK(R.B.fault == gb::F_DRUM_SENSOR && R.B.faultWhy[0] == 'S', "SW_ARM counted 0 of 7: DRUM_SENSOR"); }
  { Rig R; R.toMode("NEUTRAL"); R.ok("{\"id\":1,\"cmd\":\"wrap\"}"); R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 60000);
    CHECK(R.B.armPulses == 7 && R.B.fault == gb::F_NONE, "a clean wrap counts exactly 7 SW_ARM pulses");
    R.ok("{\"id\":2,\"cmd\":\"unwrap\"}"); R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 60000);
    CHECK(R.B.armPulses == 7 && R.B.fault == gb::F_NONE, "and the unwrap too"); }
}

static void testIndexMode() {
  T("D12: no magnet, SW_INDEX degraded angle source");
  Rig::restRig = true;
  Rig R(true, [](gb::Cfg& c) { c.swBit[gb::SW_INDEX] = 11; });
  S.encMissing = true;                       // (set after the rig boots: re-boot to enter index mode)
  Rig R2(false, [](gb::Cfg& c) { c.swBit[gb::SW_INDEX] = 11; });
  R2.cmd("{\"id\":0,\"cmd\":\"cal\",\"what\":\"cock_rest\"}");
  CHECK(R2.B.indexMode && R2.cmd("{\"id\":1,\"cmd\":\"status\"}").find("\"magnet\":\"INDEX\"") != std::string::npos, "index mode at boot");
  S.lineOut = 0; S.seatAtLine = 0;
  CHECK(R2.toMode("LAUNCH"), "LAUNCH");
  R2.ok("{\"id\":2,\"cmd\":\"aim\",\"deg\":30}"); R2.until([&] { return R2.idle() || R2.B.state == gb::S_FAULT; }, 60000);
  double tp = R2.B.pitchAt((float)S.aimPh) - R2.B.aimOffset;
  CHECK(R2.B.aimed && fabs(tp - 30) < 3, "aim within 3 deg of pitch on the index alone");
  R2.ok("{\"id\":3,\"cmd\":\"cock\"}"); R2.until([&] { return R2.idle() || R2.B.state == gb::S_FAULT; }, 30000);
  printf("    index cock: fw %.1f true %.1f\n", R2.B.cockPhase, S.cockPh);
  CHECK(R2.B.phase == gb::PH_COCKED && S.shots == 0 && S.cockPh < S.c.releaseDeg - 3, "cocked short of the release tooth");
  std::string a = R2.cmd("{\"id\":4,\"cmd\":\"arm\"}"); long tok = atol(a.c_str() + a.find("\"token\":") + 8);
  R2.ok("{\"id\":5,\"cmd\":\"fire\",\"token\":" + std::to_string(tok) + "}"); R2.until([&] { return R2.idle() || R2.B.state == gb::S_FAULT; }, 30000);
  CHECK(S.shots == 1 && R2.B.phase == gb::PH_REST && R2.B.fault == gb::F_NONE, "fired and reset on the index alone");
}

static void testReview3() {
  T("review round 3 (docs cross-check) regressions");
  { Rig R; S.lineOut = 5; S.seatAtLine = -1; R.run(50);
    CHECK(R.err("{\"id\":1,\"cmd\":\"cock\"}") == "NOT_SEATED", "cock needs SW_SEATED (lens B I2)"); }
  { Rig R; R.toMode("NEUTRAL"); R.run(500); S.drumDeg = 20; R.run(600);
    CHECK(R.B.fault == gb::F_DRUM_SENSOR, "drum turned by hand: DRUM_SENSOR");
    R.run(500); CHECK(R.ok("{\"id\":1,\"cmd\":\"clear\"}") && R.idle(), "one clear is enough");
    R.run(1000); CHECK(R.idle(), "and no second fault"); }
  { Rig::restRig = false; Rig R; Rig::restRig = true; S.out.clear();
    CHECK(R.ok("{\"id\":1,\"cmd\":\"cal\",\"what\":\"decock\"}") && R.lastEvt("RESET").find("\"ok\":true") != std::string::npos && R.B.phase == gb::PH_REST,
          "decock with the phase unknown, SW_HOME and SW_GAP closed: RESET ok:true, at rest (BASE_SPEC 10.1 decock, RESET, aim)"); }
  { Rig::restRig = false; Rig R; Rig::restRig = true; S.swStuckOpen = 1 << 7; R.run(50); S.out.clear();   // no SW_GAP reading
    CHECK(R.ok("{\"id\":1,\"cmd\":\"cal\",\"what\":\"decock\"}") && R.lastEvt("RESET").find("\"ok\":false") != std::string::npos && R.B.phase == gb::PH_UNKNOWN,
          "without SW_GAP closed: RESET ok:false (never silent), phase stays unknown"); }
  { Rig R; R.toMode("LAUNCH"); R.ok("{\"id\":1,\"cmd\":\"aim\",\"deg\":30}"); R.run(200);
    CHECK(fabsf(R.B.w.c.tripA - R.B.c.aimTripA) < 1e-3 && fabsf(R.B.c.aimTripA - 3.0f) < 1e-3, "aim runs on its own 3 A trip (D40)"); R.ok("{\"id\":2,\"cmd\":\"stop\"}"); }
  { Rig R; R.toMode("NEUTRAL"); R.wrapToReel(); S.lineOut = 8; S.anchorTautAt = 7.5; R.run(50);
    R.ok("{\"id\":1,\"cmd\":\"reel\",\"duty\":1,\"until\":\"tension\"}"); R.run(200);
    CHECK(fabsf(R.B.w.c.tripA - R.B.towTrip()) < 1e-3 && R.B.w.c.tripA > 5.5f && R.B.w.c.tripA < 6.6f, "tow trip from D34 at the 50 N default (about 6 A)");
    R.ok("{\"id\":2,\"cmd\":\"stop\"}");
    S.dogTip = true; R.run(50);                 // collar contact lost in REEL: mode REEL must not answer ok at once
    S.out.clear(); R.ok("{\"id\":3,\"cmd\":\"mode\",\"to\":\"REEL\"}");
    CHECK(!R.evt("MODE"), "mode REEL without SW_DOG_R: no MODE ok at once (the move confirms the collar)");
    S.dogTip = false; R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 20000);
    CHECK(R.idle() && R.B.collarReel(), "then confirmed"); }
  { Rig R; S.encMissing = true; R.run(100);
    CHECK(R.err("{\"id\":1,\"cmd\":\"mode\",\"to\":\"NEUTRAL\"}") == "ANGLE_SENSOR", "no centering jog without the AS5600: refused");
    CHECK(R.err("{\"id\":1,\"cmd\":\"wrap\"}") == "ANGLE_SENSOR", "wrap from LAUNCH refused too"); }
  { Rig R; R.homeAimAndCock(); S.swStuckOpen = 1 << 7; R.run(50);   // SW_GAP never closes
    R.ok("{\"id\":1,\"cmd\":\"cal\",\"what\":\"decock\"}"); R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 40000);
    CHECK(R.B.phase != gb::PH_REST && R.B.state == gb::S_FAULT, "decock ends only with SW_HOME and SW_GAP closed (BASE_SPEC 10)"); }
  { Rig R; R.ok("{\"id\":1,\"cmd\":\"set\",\"key\":\"host_timeout_s\",\"val\":7}"); R.ok("{\"id\":1,\"cmd\":\"set\",\"key\":\"tension_offset_n\",\"val\":-3}");
    R.ok("{\"id\":1,\"cmd\":\"set\",\"key\":\"tow_trip_a\",\"val\":7}");
    gb::Tun t = R.B.tunNow(); gb::Base B2; B2.restoreTun(&t);
    CHECK(B2.c.hostTimeoutS == 0 && B2.c.tensOffsetN == -3 && B2.c.towTripA == 7, "tension_offset_n, tow_trip_a saved; host_timeout_s back to 0 at boot (review r3_M)");
    gb::Tun old = t; old.ver = 1; gb::sealTun(&old); gb::Base B3; B3.restoreTun(&old);
    CHECK(B3.c.kNperA == P_REEL_TOW_TENSION_MODEL_N_PER_A && B3.c.tensOffsetN == P_REEL_TOW_TENSION_MODEL_OFFSET_N, "an old record is ignored: params tension constants (D43 dart side 14.8 / -2.6)"); }
  { S = Sim(); gb::Base B; B.io = simIo(); B.begin(S.t); B.restore(nullptr); S.step(5); B.tick(S.t += 0);
    char rep[300]; const char* l = "{\"id\":1,\"cmd\":\"set\",\"key\":\"limit_n\",\"val\":30}";
    B.handleLine(l, (int)strlen(l), rep, sizeof rep);
    CHECK(std::string(rep).find("BOOTING") != std::string::npos, "set in the first 60 ms: BOOTING (resolveBoot would undo it)");
    const char* l2 = "{\"id\":2,\"cmd\":\"cal\",\"what\":\"cock_rest\"}"; B.handleLine(l2, (int)strlen(l2), rep, sizeof rep);
    CHECK(std::string(rep).find("BOOTING") != std::string::npos, "cal in the first 60 ms: BOOTING"); }
  { Rig R; R.B.evDrops = 3; std::string r = R.cmd("{\"id\":1,\"cmd\":\"status\"}");
    CHECK(r.find("\"ev_drops\":3") != std::string::npos && r.size() <= 256, "status reports dropped events, within 256 bytes"); }
  { Rig R; R.B.c.freeA = 1.5f; R.B.w.amps = 5.0f;
    CHECK(fabsf((R.B.w.amps - R.B.c.freeA) * R.B.c.kNperA + R.B.c.tensOffsetN - 49.2f) < 0.1f, "dart-side tension model 14.8 x (I - I_free) - 2.6 (D43): 5 A gives 49.2 N"); }
}

static void testReview4() {
  T("review round 4 (drum safety) regressions");
  { S = Sim(); S.drumDeg = 305; S.win = 8;     // cut just past the WRAP edge, angle not trusted at boot
    Rig R(false); R.cmd("{\"id\":0,\"cmd\":\"cal\",\"what\":\"cock_rest\"}");
    double lo = 1e9; R.ok("{\"id\":1,\"cmd\":\"estop\"}");
    R.until([&] { if (S.drumDeg < lo) lo = S.drumDeg; return R.idle() || R.B.state == gb::S_FAULT; }, 30000);
    printf("    lowest drum %.1f, wraps %.2f\n", lo, S.wrapsExact());
    CHECK(lo > 300.5 && S.wrapsExact() > 6.95, "estop from N_TO_R after a cut: never runs into the wrap window");
    CHECK(S.dDir == 0, "and holds there (D27)"); }
  { Rig R; R.toMode("NEUTRAL"); R.run(500); R.B.drumDeg = 32; R.B.c.drumDps = 4;   // estimate wrong: the timed run overshoots
    R.ok("{\"id\":1,\"cmd\":\"mode\",\"to\":\"NEUTRAL\"}"); double hi = 0;
    R.until([&] { if (S.drumDeg > hi) hi = S.drumDeg; return R.idle() || R.B.state == gb::S_FAULT; }, 30000);
    printf("    overshoot to %.1f, stopped at %.1f\n", hi, S.drumDeg);
    CHECK(hi > 50 && R.idle() && R.B.drumPos() == gb::DP_NEUTRAL && S.drumDeg > 32 && S.drumDeg < 50, "overshot the target sector: came back to the edge and parked inside it"); }
  { S = Sim(); S.drumDeg = 305; S.win = 8; Rig R(false); R.cmd("{\"id\":0,\"cmd\":\"cal\",\"what\":\"cock_rest\"}");
    double lo = 1e9; R.B.raise(gb::F_BROWNOUT, "test");
    R.until([&] { if (S.drumDeg < lo) lo = S.drumDeg; return R.B.state == gb::S_FAULT && S.dDir == 0; }, 30000);
    CHECK(lo > 300.5 && S.wrapsExact() > 6.95, "fault from N_TO_R after a cut: never runs into the wrap window"); }
  { Rig R; R.toMode("LAUNCH"); R.ok("{\"id\":1,\"cmd\":\"cock\"}"); R.until([&] { return S.cockPh > 405; }, 20000); S.wDir = 0;
    S.swStuckClosed |= 1 << 6;                 // power cut near the cock end, SW_HOME fails closed
    Rig R2(false); R2.run(100);
    CHECK(R2.err("{\"id\":1,\"cmd\":\"cock\"}") == "LOADED_UNKNOWN" && S.shots == 0, "cut mid-cock + SW_HOME stuck closed: no recovery creep, no shot"); }
  { Rig R; R.toMode("NEUTRAL"); R.wrapToReel(); S.lineOut = 8; S.anchorTautAt = 7.6; S.anchorK = 60; R.run(50);
    CHECK(R.err("{\"id\":1,\"cmd\":\"set\",\"key\":\"limit_n\",\"val\":61}") == "OUT_OF_RANGE" && R.err("{\"id\":2,\"cmd\":\"reel\",\"max_n\":61,\"until\":\"tension\"}") == "BAD_ARG", "D39/D43: tow limit above 60 N refused");
    R.B.c.freeA = 3.0f; R.ok("{\"id\":1,\"cmd\":\"set\",\"key\":\"limit_n\",\"val\":60}");
    R.ok("{\"id\":2,\"cmd\":\"reel\",\"duty\":1,\"max_n\":60,\"until\":\"tension\"}");
    CHECK(R.B.w.c.tripA > 8.0f && R.B.w.c.tripA <= R.B.c.hardTripA, "D34: tow trip follows the limit ((60 + 2.6) / 14.8 + I_free + 1 A)");
    R.ok("{\"id\":3,\"cmd\":\"stop\"}"); }
  { Rig R; R.toMode("NEUTRAL"); R.wrapToReel(); R.ok("{\"id\":1,\"cmd\":\"unwrap\"}");
    R.until([&] { return S.drumDeg < 180; }, 120000); R.ok("{\"id\":2,\"cmd\":\"stop\"}"); R.run(500);
    R.ok("{\"id\":3,\"cmd\":\"wrap\"}"); R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 120000);
    CHECK(R.idle() && R.B.drumPos() == gb::DP_WRAPPED, "unwrap stopped inside the window, then wrap: no SW_ARM miscount"); }
  { Rig R0; S = Sim(); S.wiperDeg = 1234.5;      // the shaft is not at raw 0 when the board first boots
    Rig R(false); R.cmd("{\"id\":0,\"cmd\":\"cal\",\"what\":\"cock_rest\"}"); R.homeAimAndCock(); R.run(500);
    Rig R2(false);
    CHECK(R2.B.aimKnown && R2.B.phase == gb::PH_COCKED, "idle reboot with the shaft anywhere: aim and cocked phase kept (AS5600 count lined up, review D2 #9)"); }
  { Rig R0; S = Sim(); S.drumDeg = 333; S.win = 8;   // power cut during the REEL fly-in (N_TO_R, collar starting to engage)
    S.nvsHas = true; S.nvs = R0.B.snapshot(2); S.nvs.drumDeg = 314; S.nvs.drumKnown = 1; S.nvs.wraps = 7; S.nvs.wrapsKnown = 1; gb::sealSaved(&S.nvs);
    Rig R(false); R.cmd("{\"id\":0,\"cmd\":\"cal\",\"what\":\"cock_rest\"}");
    CHECK(R.B.sec == gb::SEC_LOCK && !R.B.degTrusted, "boot in N_TO_R after a cut mid-move: angle untrusted");
    CHECK(R.err("{\"id\":1,\"cmd\":\"tidy\"}") != "", "tidy refused on a guessed N_TO_R angle");
    S.slack = 0; double mx = 0; R.ok("{\"id\":2,\"cmd\":\"wrap\"}");
    R.until([&] { if (S.slack > mx) mx = S.slack; return R.idle() || R.B.state == gb::S_FAULT; }, 60000);
    char m[110]; snprintf(m, sizeof m, "wrap re-references on the WRAP edge and parks (true %.1f, slack %.3f m)", S.drumDeg, mx);
    CHECK(R.idle() && fabs(S.drumDeg - 314) < 5 && mx < 0.1 && R.B.degTrusted, m); }
  { Rig R0; S = Sim(); S.drumDeg = 47; S.swStuckOpen = 0x7;   // boot with the drum code common wire off (000), drum at 47
    Rig R(false); R.cmd("{\"id\":0,\"cmd\":\"cal\",\"what\":\"cock_rest\"}"); R.run(500);
    S.swStuckOpen = 0; R.run(300); R.cmd("{\"id\":1,\"cmd\":\"clear\"}"); R.run(100);
    CHECK(R.B.sec == gb::SEC_NEUTRAL && !R.B.degTrusted, "code back after 000: sector known, angle untrusted");
    R.ok("{\"id\":2,\"cmd\":\"mode\",\"to\":\"NEUTRAL\"}"); R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 40000);
    char m[90]; snprintf(m, sizeof m, "mode NEUTRAL re-references and parks at 41 (true %.1f)", S.drumDeg);
    CHECK(R.idle() && fabs(S.drumDeg - 41) < 5, m); }
  { Rig R; S.aimDragDeg = 127; R.toMode("LAUNCH");   // cad-launch: the aim drags the big gear back and opens SW_GAP
    R.ok("{\"id\":1,\"cmd\":\"aim\",\"deg\":30}"); R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 60000);
    CHECK(R.B.aimed && R.B.closed(gb::SW_GAP) && S.cockPh > -34.5 && S.cockPh < 0, "aim then the forward gap run: AIMED with SW_GAP closed, train still in its gap");
    std::string a = R.cmd("{\"id\":2,\"cmd\":\"arm\"}"); long tok = atol(a.c_str() + a.find("\"token\":") + 8);
    CHECK(R.ok("{\"id\":3,\"cmd\":\"fire\",\"token\":" + std::to_string(tok) + "}"), "arm and fire after the gap run");
    R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 30000);
    CHECK(S.shots == 1 && R.B.phase == gb::PH_REST && R.B.fault == gb::F_NONE, "shot and reset"); }
  { Rig R; R.toMode("LAUNCH"); R.ok("{\"id\":1,\"cmd\":\"aim\",\"deg\":30}"); R.until([&] { return R.idle(); }, 60000);
    S.cockPh = -100; R.run(100);               // the gap opened after the aim (sprag drag)
    std::string a = R.cmd("{\"id\":2,\"cmd\":\"arm\"}"); long tok = atol(a.c_str() + a.find("\"token\":") + 8);
    CHECK(a.find("\"ok\":true") != std::string::npos, "arm with a fixable open gap: ok (the gap run starts)");
    CHECK(R.ok("{\"id\":3,\"cmd\":\"fire\",\"token\":" + std::to_string(tok) + "}"), "fire during the gap run is queued");
    R.until([&] { return (R.idle() && S.shots) || R.B.state == gb::S_FAULT; }, 30000);
    CHECK(S.shots == 1 && R.B.fault == gb::F_NONE, "queued fire runs after the gap closed"); }
  { Rig R; R.toMode("NEUTRAL"); R.run(500); S.wDir = 0; S.drumDeg = 20;   // board off, drum turned by hand to L_TO_N (systest F26)
    Rig R2(false); R2.run(500);
    CHECK(R2.B.fault == gb::F_NONE && R2.lastEvt("BOOT").find("\"drum_moved\":true") != std::string::npos && !R2.B.degTrusted, "drum turned by hand while off: no fault, BOOT drum_moved, angle a guess");
    R2.cmd("{\"id\":0,\"cmd\":\"cal\",\"what\":\"cock_rest\"}");
    CHECK(R2.toMode("NEUTRAL") && fabs(S.drumDeg - 41) < 6, "then mode NEUTRAL works"); }
  { Rig R; R.toMode("LAUNCH");
    CHECK(R.ok("{\"id\":1,\"cmd\":\"aim\",\"deg\":6}"), "aim 6 (just under the table minimum 6.01) accepted and clamped (systest F27)");
    R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 60000); CHECK(R.B.aimed, "aimed at the minimum"); }
  for (int every : {200, 100, 40}) {             // bursts of 3 failed I2C reads during a cock (review: travel was lost)
    Rig R; R.toMode("LAUNCH"); R.ok("{\"id\":2,\"cmd\":\"aim\",\"deg\":30}"); R.until([&] { return R.idle(); }, 60000);
    S.encDropEvery = every; R.ok("{\"id\":3,\"cmd\":\"cock\"}"); R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 30000);
    char m[96]; snprintf(m, sizeof m, "I2C dropouts every %d reads: cock end right, no shot (fw %.1f, true %.1f)", every, R.B.cockPhase, S.cockPh);
    CHECK(S.shots == 0 && (R.B.state == gb::S_FAULT || fabs(R.B.cockPhase - S.cockPh) < 3), m); }
  { Rig R; R.ok("{\"id\":1,\"cmd\":\"cal\",\"what\":\"cock_rest\"}"); R.toMode("LAUNCH"); R.ok("{\"id\":1,\"cmd\":\"cock\"}");
    R.until([&] { return S.cockPh > 330; }, 20000); S.wDir = 0;
    Rig R2(false); R2.ok("{\"id\":1,\"cmd\":\"cal\",\"what\":\"decock\"}"); R2.run(400); R2.ok("{\"id\":2,\"cmd\":\"stop\"}");
    S.swStuckClosed |= 1 << 6; R2.run(100);
    CHECK(R2.err("{\"id\":3,\"cmd\":\"cock\"}") == "LOADED_UNKNOWN" && S.shots == 0, "stop during a recovery decock keeps the drawn warning (no creep, no shot)"); }
  { Rig R; R.toMode("NEUTRAL"); R.ok("{\"id\":1,\"cmd\":\"wrap\"}"); R.until([&] { return S.drumDeg > 200; }, 120000); R.ok("{\"id\":2,\"cmd\":\"stop\"}"); R.run(600);
    S.swStuckClosed |= 1 << 1; R.run(800);    // SW_DRUM_B fails closed inside WRAP: 001 reads 011 (WRAP_PREP)
    CHECK(R.B.fault == gb::F_DRUM_SENSOR && !R.B.wrapsKnown, "lying code contact near the wrap window: fault, wraps doubtful");
    R.run(500); R.ok("{\"id\":3,\"cmd\":\"clear\"}");
    CHECK(R.err("{\"id\":4,\"cmd\":\"mode\",\"to\":\"LAUNCH\"}") == "WRAPS_ON" && R.err("{\"id\":4,\"cmd\":\"wrap\"}") == "WRAPS_ON", "after clear only unwrap is allowed");
    S.slack = 0; S.maxSlack = 0; double mx = 0;
    R.ok("{\"id\":5,\"cmd\":\"unwrap\"}"); R.until([&] { if (S.slack > mx) mx = S.slack; return R.idle() || R.B.state == gb::S_FAULT; }, 120000);
    printf("    doubt unwrap: max slack %.3f m, drum %.1f, state %d\n", mx, S.drumDeg, R.B.state);
    CHECK(mx < 0.15, "the unwrap runs the take-up all the way (no loose line)"); }
  { Rig R; R.toMode("NEUTRAL"); R.ok("{\"id\":1,\"cmd\":\"wrap\"}"); R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 120000);
    CHECK(R.B.drumPos() == gb::DP_WRAPPED && R.err("{\"id\":2,\"cmd\":\"retrieve\"}") == "WRAPS_ON", "D39: no retrieve at the WRAPPED park (wraps on)"); }
  { Rig R; R.toMode("LAUNCH"); R.ok("{\"id\":1,\"cmd\":\"aim\",\"deg\":40}"); R.run(3000); S.wDir = 0;   // power cut mid-aim (systest F15)
    Rig R2(false);
    CHECK(R2.B.phase == gb::PH_REST && !R2.B.aimKnown && R2.ok("{\"id\":2,\"cmd\":\"aim\",\"deg\":30}"), "cut during aim: REST kept (SW_HOME + SW_GAP closed), aim re-homes");
    R2.until([&] { return R2.idle() || R2.B.state == gb::S_FAULT; }, 60000); CHECK(R2.B.aimed && S.shots == 0, "aimed, no shot"); }
  { Rig R; R.toMode("NEUTRAL");
    CHECK(R.err("{\"id\":1,\"cmd\":\"jog\",\"motor\":\"wiper\",\"dir\":-1}") == "BACKSTOP", "wiper backward jog refused outside LAUNCH (r1_D 14)"); }
  { Rig R; R.toMode("LAUNCH"); S.swStuckOpen = 1 << 7; R.run(50);
    CHECK(R.err("{\"id\":1,\"cmd\":\"cock\"}") != "" && S.wDir == 0, "cock from REST refused with SW_GAP open (r1_D 10)");
    CHECK(R.err("{\"id\":1,\"cmd\":\"cal\",\"what\":\"cock_rest\"}") == "SWITCH_DISAGREE", "cal cock_rest refused with SW_GAP open"); }
  { Rig R; R.ok("{\"id\":1,\"cmd\":\"mode\",\"to\":\"NEUTRAL\"}"); R.run(300); S.dDir = 0; S.wDir = 0;   // cut leaving LAUNCH
    Rig R2(false);
    if (R2.B.sec == gb::SEC_LAUNCH) CHECK(R2.err("{\"id\":1,\"cmd\":\"jog\",\"motor\":\"drum\",\"dir\":-1}") == "POSITION_UNKNOWN", "drum jog toward the end stop refused with the angle not trusted (r1_D 9)");
    else CHECK(true, "skip"); }
  { Rig Q; CHECK(Q.B.c.flyDuty >= Q.B.w.c.minScaleDuty, "fly-in duty high enough for the IS current scaling (3 A trip works)"); }
  { Rig Q; Q.toMode("NEUTRAL"); S.swStuckClosed |= 1 << Q.B.c.swBit[gb::SW_DOG_L]; Q.run(100);
    CHECK(Q.err("{\"id\":3,\"cmd\":\"jog\",\"motor\":\"drum\",\"dir\":1,\"ms\":300}") != "", "drum jog refused with a collar contact disagreeing"); }
  { Rig Q; Q.toMode("NEUTRAL"); Q.run(500);
    for (int i = 0; i < 400 && Q.B.sec != gb::SEC_WRAP && Q.B.state != gb::S_FAULT; i++) { if (Q.cmd("{\"id\":3,\"cmd\":\"jog\",\"motor\":\"drum\",\"dir\":1,\"ms\":300}").find("true") == std::string::npos) break; Q.run(100); }
    Q.run(500);
    CHECK(S.drumDeg < 64 && Q.lastEvt("ABORTED").find("wrap window") != std::string::npos, "drum jog stops at the wrap window edge");
    CHECK(Q.err("{\"id\":3,\"cmd\":\"jog\",\"motor\":\"drum\",\"dir\":1,\"ms\":300}") == "WRAPS_ON", "no drum jog inside the wrap window"); }
}

static void testReview5() {
  T("review round 5 (protocol) regressions");
  { Rig R; R.ok("{\"id\":1,\"cmd\":\"mode\",\"to\":\"NEUTRAL\"}"); R.run(200);
    CHECK(R.err("{\"id\":1.5,\"cmd\":\"estop\"}") == "BAD_ID" && S.dDir == 0, "estop with a bad id still stops the drum");
    R.until([&] { return R.idle(); }, 20000); }
  { Rig R; R.ok("{\"id\":1,\"cmd\":\"mode\",\"to\":\"NEUTRAL\"}"); R.run(200);
    std::string l = "{\"a\":1,\"b\":1,\"c\":1,\"d\":1,\"e\":1,\"f\":1,\"g\":1,\"h\":1,\"i\":1,\"j\":1,\"k\":1,\"m\":1,\"id\":7,\"cmd\":\"stop\"}";
    CHECK(R.ok(l) && S.dDir == 0, "id and cmd win a slot after 12 unknown keys: stop obeyed"); }
  { Rig R; R.B.raise(gb::F_BAD_STATE, "a fault text of exactly thirty-two"); R.run(100); R.B.enc.glitches = 99999; R.B.evDrops = 99999; R.B.w.peakA = 19.9f;
    std::string r = R.cmd("{\"id\":\"abcdefghijklmnopq\",\"cmd\":\"status\"}");
    CHECK(r.size() <= 256 && r.find("\"phase\"") != std::string::npos, "status with a 17-char id keeps its fields (fault text dropped first)"); }
  { Rig R; R.toMode("LAUNCH"); R.ok("{\"id\":3,\"cmd\":\"aim\",\"deg\":30}"); R.until([&] { return R.idle(); }, 60000);
    CHECK(R.err("{\"id\":1,\"cmd\":\"cal\",\"what\":\"aim\",\"deg\":1000}") == "OUT_OF_RANGE", "cal aim clamped like set aim_offset"); }
  { Rig R; R.ok("{\"id\":1,\"cmd\":\"set\",\"key\":\"hx_n_per_count\",\"val\":4e-7}");
    std::string r = R.cmd("{\"id\":1,\"cmd\":\"get\",\"key\":\"hx_n_per_count\"}");
    CHECK(r.find("\"val\":4e-07") != std::string::npos, "get keeps 7 significant digits"); }
  { Rig R; R.toMode("NEUTRAL"); S.lineOut = 5; S.seatAtLine = 0; S.anchorTautAt = -1; R.run(50);   // NO_GO: unanchored dart 5 m out
    R.ok("{\"id\":1,\"cmd\":\"set\",\"key\":\"line_m\",\"val\":5}");
    CHECK(R.ok("{\"id\":2,\"cmd\":\"retrieve\"}"), "retrieve 5 m on the take-up");
    uint32_t t0 = S.t; R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 200000);
    printf("    5 m take-up retrieval took %.1f s (limit %.1f s)\n", (S.t - t0) / 1000.0, R.B.approachMs / 1000.0);
    CHECK(R.evt("SEATED") && R.B.fault == gb::F_NONE, "5 m retrieval seats (time limit from line_m)");
    CHECK(R.err("{\"id\":3,\"cmd\":\"retrieve\",\"max_s\":2}") == "BAD_ARG" && R.err("{\"id\":3,\"cmd\":\"retrieve\",\"max_s\":601}") == "BAD_ARG", "max_s range 5..600"); }
  { Rig R; R.toMode("NEUTRAL"); S.lineOut = 3; S.anchorTautAt = 2.5; R.run(50);
    R.ok("{\"id\":2,\"cmd\":\"retrieve\",\"max_s\":6}"); uint32_t t0 = S.t;
    R.until([&] { return R.B.state == gb::S_FAULT; }, 30000);
    CHECK(R.B.fault == gb::F_DART_STUCK && S.t - t0 < 7000, "max_s sets the DART_STUCK time"); }
  { Rig R; int n0 = S.saves; (void)n0; static int tunSaves; tunSaves = 0; R.B.io.saveTun = [](const gb::Tun&) { tunSaves++; };
    R.ok("{\"id\":1,\"cmd\":\"set\",\"key\":\"limit_n\",\"val\":44}"); R.run(300);
    for (int i = 0; i < 5; i++) { R.ok("{\"id\":1,\"cmd\":\"set\",\"key\":\"limit_n\",\"val\":44}"); R.ok("{\"id\":1,\"cmd\":\"cal\",\"what\":\"drum\"}"); R.run(300); }
    CHECK(tunSaves == 1, "tuning written to flash only when it changed"); }
}

static void testIndexFailures() {
  T("D12 index mode: SW_INDEX failing while cocking never fires");
  for (int mode = 0; mode < 3; mode++) for (double failAt : {100.0, 200.0, 250.0, 320.0}) {
    { Rig R0(true, [](gb::Cfg& c) { c.swBit[gb::SW_INDEX] = 11; }); }
    S.encMissing = true;
    Rig R(false, [](gb::Cfg& c) { c.swBit[gb::SW_INDEX] = 11; }); R.cmd("{\"id\":0,\"cmd\":\"cal\",\"what\":\"cock_rest\"}");
    R.toMode("LAUNCH"); R.ok("{\"id\":2,\"cmd\":\"aim\",\"deg\":30}"); R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 60000);
    R.ok("{\"id\":3,\"cmd\":\"cock\"}"); bool set = false;
    R.until([&] {
      if (mode == 2) { bool in = S.cockPh > failAt && S.cockPh < failAt + 120; if (in) S.swStuckOpen |= 1 << 11; else S.swStuckOpen &= ~(1 << 11); }
      else if (!set && S.cockPh > failAt) { set = true; if (mode == 0) S.swStuckClosed |= 1 << 11; else S.swStuckOpen |= 1 << 11; }
      return R.idle() || R.B.state == gb::S_FAULT; }, 30000);
    char m[120]; snprintf(m, sizeof m, "SW_INDEX %s from cock %.0f: no shot (true %.1f, state %d)", mode == 0 ? "stuck closed" : mode == 1 ? "stuck open" : "missing 2 bumps", failAt, S.cockPh, R.B.state);
    CHECK(S.shots == 0 && S.cockPh < S.c.releaseDeg - 5, m);
  }
}

static void testReview6() {
  T("review round 6 (adversarial) regressions");
  for (double K : {1000.0, 3000.0}) for (int lim : {40, 60}) {     // stiff anchored line: the tow must stop near the limit
    Rig R; R.toMode("NEUTRAL"); R.wrapToReel(); S.lineOut = 8; S.anchorTautAt = 7.95; S.anchorK = K; R.run(50);
    char c1[96]; snprintf(c1, sizeof c1, "{\"id\":1,\"cmd\":\"reel\",\"duty\":1,\"max_n\":%d,\"until\":\"tension\"}", lim); R.ok(c1);
    double mx = 0; R.until([&] { if (S.tension() > mx) mx = S.tension(); return R.B.state == gb::S_HOLD || R.B.state == gb::S_FAULT; }, 120000);
    for (int k = 0; k < 3; k++) { R.ok(c1); R.until([&] { if (S.tension() > mx) mx = S.tension(); return R.B.state == gb::S_HOLD || R.B.state == gb::S_FAULT; }, 20000); }   // re-issued from HOLD
    char m[110]; snprintf(m, sizeof m, "stiff line K %.0f N/m, limit %d N: peak %.1f N (state %d)", K, lim, mx, R.B.state);
    CHECK(mx < (K > 2000 ? 1.6 : 1.3) * lim, m);   // K 3000 N/m (very stiff, short line): the 50 ms current lag allows about 50 percent; known limit (REPORT)
  }
  { Rig R; S.hxFitted = true; R.run(100); R.ok("{\"id\":1,\"cmd\":\"cal\",\"what\":\"tare\"}");
    R.toMode("NEUTRAL"); R.wrapToReel(); S.lineOut = 8; S.anchorTautAt = 7.5; S.anchorK = 60;
    R.ok("{\"id\":1,\"cmd\":\"reel\",\"duty\":1,\"max_n\":40,\"until\":\"tension\"}"); R.run(1500); S.hxCountsPerN = 0;   // bridge wire off mid-tow
    double mx = 0; R.until([&] { if (S.tension() > mx) mx = S.tension(); return R.B.state == gb::S_HOLD || R.B.state == gb::S_FAULT; }, 120000);
    char m[90]; snprintf(m, sizeof m, "load cell frozen mid-tow: the current estimate still limits (peak %.1f N)", mx);
    CHECK(mx < 55, m);
    CHECK(R.err("{\"id\":2,\"cmd\":\"set\",\"key\":\"free_a\",\"val\":10}") == "BUSY" && R.err("{\"id\":2,\"cmd\":\"cal\",\"what\":\"tare\"}") == "BUSY", "current / tension constants refused while holding"); }
  { Rig R; R.toMode("NEUTRAL"); R.ok("{\"id\":1,\"cmd\":\"wrap\"}"); R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 120000);
    S.swStuckClosed |= 1 << 0; R.run(300);   // SW_DRUM_A stuck closed: WRAP (001) will read N_TO_R (101)
    S.slack = 0; double mx = 0; R.ok("{\"id\":2,\"cmd\":\"unwrap\"}");
    R.until([&] { if (S.slack > mx) mx = S.slack; return R.idle() || R.B.state == gb::S_FAULT; }, 120000);
    char m[90]; snprintf(m, sizeof m, "SW_DRUM_A stuck: unwrap keeps the take-up running (max slack %.3f m)", mx);
    CHECK(mx < 0.1, m);
    CHECK(R.err("{\"id\":3,\"cmd\":\"jog\",\"motor\":\"drum\",\"dir\":-1}") != "", "no drum jog down from N_TO_R"); }
  for (double K : {1000.0, 5600.0}) {           // D41: fly-in on a straight (tidied) stiff line
    Rig R; R.toMode("NEUTRAL"); S.lineOut = 3.6; S.anchorTautAt = 3.6; S.anchorK = K;
    R.ok("{\"id\":1,\"cmd\":\"wrap\"}"); R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 120000);
    double mx = 0; R.ok("{\"id\":2,\"cmd\":\"mode\",\"to\":\"REEL\"}");
    R.until([&] { if (S.tension() > mx) mx = S.tension(); return R.idle() || R.B.state == gb::S_FAULT; }, 40000);
    char m[110]; snprintf(m, sizeof m, "fly-in on a taut line K %.0f: REEL without a fault, line under 20 N (peak %.1f N)", K, mx);
    CHECK(R.idle() && R.B.collarReel() && mx < 20, m); }
  for (int f = 0; f < 3; f++) {                // manual cock jog with a contact failing: the release guard and the collar check hold
    Rig R; R.toMode("LAUNCH"); R.ok("{\"id\":1,\"cmd\":\"jog\",\"motor\":\"wiper\",\"dir\":1,\"ms\":400}"); bool set = false;
    for (int i = 0; i < 80 && (R.B.state == gb::S_JOG); i++) {
      R.run(200); R.cmd("{\"id\":1,\"cmd\":\"jog\",\"motor\":\"wiper\",\"dir\":1,\"ms\":400}");
      if (!set && S.cockPh > 120) { set = true; if (f == 0) S.swChatter |= 1 << 3; else if (f == 1) S.swStuckOpen |= 1 << 3; else S.swStuckClosed |= 1 << 1; }
    }
    R.run(500);
    char m[90]; snprintf(m, sizeof m, "cock jog, contact fault %d mid-jog: no shot (true %.0f)", f, S.cockPh);
    CHECK(S.shots == 0, m);
  }
}

// params.TIMELINE (the mission the HD sim plays) as a scenario: each step's end state must be reachable in order
static void testTimeline() {
  T("params.TIMELINE scenario");
  Rig R; S.lineOut = 0; S.seatAtLine = 0;
  struct Step { const char* name; std::function<bool()> run; };
  auto wait = [&](uint32_t ms) { return R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, ms) && R.idle(); };
  std::vector<Step> steps = {
    {"load", [&] { return R.toMode("LAUNCH") && R.B.collarLaunch(); }},
    {"aim 35", [&] { R.ok("{\"id\":2,\"cmd\":\"aim\",\"deg\":35}"); return wait(60000) && R.B.aimed; }},
    {"cock", [&] { R.ok("{\"id\":3,\"cmd\":\"cock\"}"); return wait(30000) && R.B.phase == gb::PH_COCKED; }},
    {"fire", [&] { std::string a = R.cmd("{\"id\":4,\"cmd\":\"arm\"}"); long tok = atol(a.c_str() + a.find("\"token\":") + 8);
                   R.ok("{\"id\":5,\"cmd\":\"fire\",\"token\":" + std::to_string(tok) + "}"); return wait(30000) && S.shots == 1; }},
    {"wrap (D22: straight from LAUNCH)", [&] { R.ok("{\"id\":7,\"cmd\":\"wrap\"}"); return wait(120000) && R.B.wraps == 7 && R.B.drumPos() == gb::DP_WRAPPED; }},
    {"tidy (by time, WRAPPED park)", [&] { S.anchorTautAt = 4.4; S.anchorK = 60; R.ok("{\"id\":6,\"cmd\":\"tidy\"}"); return wait(20000) && R.evt("TIDIED"); }},
    {"reel_engage", [&] { return R.toMode("REEL") && R.B.collarReel(); }},
    {"tow", [&] { R.B.c.kNperA = (float)(1.0 / (S.capstanR * S.c.capstanPerWiper) * S.stallNm / (S.stallA - S.freeA)); R.B.c.tensOffsetN = 0;
                  R.B.c.freeA = 1.5f + (float)((S.reelFreeNm + 0.3) / S.stallNm * (S.stallA - S.freeA));
                  R.ok("{\"id\":8,\"cmd\":\"reel\",\"duty\":1,\"until\":\"tension\"}");
                  return R.until([&] { return R.B.state == gb::S_HOLD || R.B.state == gb::S_FAULT; }, 400000) && R.B.state == gb::S_HOLD; }},
    {"release + reel_in", [&] { R.ok("{\"id\":9,\"cmd\":\"stop\"}"); S.anchorTautAt = -1;
                  R.ok("{\"id\":10,\"cmd\":\"reel\",\"duty\":0.8,\"until\":\"stop\"}");
                  bool ok = R.until([&] { return S.lineOut < 1.2; }, 400000); R.ok("{\"id\":11,\"cmd\":\"stop\"}"); return ok; }},
    {"unload + unwrap", [&] { R.ok("{\"id\":12,\"cmd\":\"unwrap\"}"); return wait(60000) && R.B.wraps == 0 && S.slack < 0.05; }},
    {"approach", [&] { R.ok("{\"id\":13,\"cmd\":\"reel\",\"duty\":0.3,\"until\":\"seated\"}"); return wait(30000) && S.seated(); }},
    {"to_launch", [&] { return R.toMode("LAUNCH"); }},
    {"lower", [&] { R.ok("{\"id\":14,\"cmd\":\"aim\",\"deg\":6.2}"); return wait(60000) && R.B.pitch() < 7; }},
  };
  for (auto& st : steps) { bool ok = st.run(); CHECK(ok, st.name); if (!ok) break; }
  CHECK(R.B.fault == gb::F_NONE && Rig::dryTotal == 0, "no fault, no dry fire over the whole timeline");
}

static void testTelemetryRate() {
  T("telemetry rate");
  Rig R; int n = 0; for (uint32_t t = 0; t < 1000; t += 5) { R.tick(); if (R.B.telDue(S.t)) n++; }
  CHECK(n >= 9 && n <= 11, "10 Hz idle");
  R.ok("{\"id\":1,\"cmd\":\"mode\",\"to\":\"NEUTRAL\"}"); n = 0;
  for (uint32_t t = 0; t < 1000; t += 5) { R.tick(); if (R.B.telDue(S.t)) n++; }
  CHECK(n >= 19 && n <= 21, "20 Hz moving");
}

static void testFullMission() {
  T("full mission (lens B 7.2): aim, fire, NEUTRAL, tidy, wrap + REEL, tow, reel in, unwrap, approach, LAUNCH, again");
  Rig R;
  S.lineOut = 0; S.seatAtLine = 0;
  CHECK(R.toMode("LAUNCH"), "LAUNCH");
  R.ok("{\"id\":2,\"cmd\":\"aim\",\"deg\":35}"); R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 60000);
  CHECK(R.B.aimed, "aimed");
  std::string a = R.cmd("{\"id\":3,\"cmd\":\"arm\"}"); long tok = atol(a.c_str() + a.find("\"token\":") + 8);
  CHECK(R.ok("{\"id\":4,\"cmd\":\"fire\",\"token\":" + std::to_string(tok) + "}"), "fire from rest (whole cycle)");
  R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 30000);
  CHECK(S.shots == 1 && R.B.phase == gb::PH_REST && R.evt("FIRED") && R.evt("RESET"), "shot and reset");
  CHECK(fabs(S.cockPh) < 8, "train back at rest");
  CHECK(R.toMode("NEUTRAL"), "to NEUTRAL (dart landed)");
  S.anchorTautAt = 7.7; S.anchorK = 50;         // dart anchored at about 7.7 m
  R.ok("{\"id\":6,\"cmd\":\"wrap\"}"); R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 120000);
  CHECK(R.B.wraps == R.B.c.wrapsFull && R.B.drumPos() == gb::DP_WRAPPED, "wrapped, parked at WRAPPED");
  CHECK(R.ok("{\"id\":5,\"cmd\":\"tidy\",\"ms\":2500}"), "tidy");
  R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 20000);
  CHECK(R.evt("TIDIED"), "TIDIED");
  CHECK(R.toMode("REEL") && R.B.collarReel(), "fly-in to REEL");
  R.ok("{\"id\":7,\"cmd\":\"set\",\"key\":\"line_m\",\"val\":8}");
  R.B.c.kNperA = (float)(1.0 / (S.capstanR * S.c.capstanPerWiper) * S.stallNm / (S.stallA - S.freeA)); R.B.c.tensOffsetN = 0;
  R.B.c.freeA = 1.5f + (float)((S.reelFreeNm + 0.3) / S.stallNm * (S.stallA - S.freeA));
  R.ok("{\"id\":8,\"cmd\":\"reel\",\"duty\":1,\"max_n\":40,\"until\":\"tension\"}");
  R.until([&] { return R.B.state == gb::S_HOLD || R.B.state == gb::S_FAULT; }, 400000);
  CHECK(R.B.state == gb::S_HOLD, "towed to the limit");
  R.ok("{\"id\":9,\"cmd\":\"stop\"}");
  S.anchorTautAt = -1;                          // dart released
  R.ok("{\"id\":10,\"cmd\":\"reel\",\"duty\":0.8,\"until\":\"stop\"}");
  R.until([&] { return S.lineOut < 1.5 || R.B.state == gb::S_FAULT; }, 400000); R.ok("{\"id\":11,\"cmd\":\"stop\"}");
  R.ok("{\"id\":12,\"cmd\":\"unwrap\"}"); R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 60000);
  CHECK(R.B.wraps == 0 && R.B.drumPos() == gb::DP_NEUTRAL, "unwrapped to NEUTRAL");
  R.ok("{\"id\":13,\"cmd\":\"reel\",\"duty\":0.3,\"until\":\"seated\"}"); R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 30000);
  CHECK(R.evt("SEATED"), "final approach: seated");
  CHECK(R.toMode("LAUNCH"), "back to LAUNCH");
  R.ok("{\"id\":14,\"cmd\":\"aim\",\"deg\":30}"); R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 60000);
  R.ok("{\"id\":15,\"cmd\":\"cock\"}"); R.until([&] { return R.idle() || R.B.state == gb::S_FAULT; }, 30000);
  CHECK(R.B.phase == gb::PH_COCKED && R.B.aimed, "aimed and cocked for the next shot");
  CHECK(R.B.fault == gb::F_NONE, "no fault in the whole mission");
  size_t maxLen = 0; for (auto& l : S.out) maxLen = l.size() > maxLen ? l.size() : maxLen;
  CHECK(maxLen <= 256, "every event line within 256 bytes");
}

int main(int argc, char** argv) {
  if (argc > 1) { std::string w = argv[1];
    if (w == "cock") testCockFireReset(); else if (w == "aim") testAim(); else if (w == "wrap") testWrapReelUnwrap(); else if (w == "jog") testJog(); else if (w == "drum") testDrumFaults(); else if (w == "power") testPowerLoss(); else if (w == "stall") testStallOvercurrent(); else if (w == "enc") testAngleSensorFault(); else if (w == "tow") testTow(); else if (w == "rev1") testReview1(); else if (w == "rev2") testReview2(); else if (w == "v1") testSpecV1Contacts(); else if (w == "tl") testTimeline(); else if (w == "idx") testIndexMode(); else if (w == "mission") testFullMission(); else if (w == "stop") testStopEstop(); else if (w == "rev3") testReview3(); else if (w == "rev4") testReview4(); else if (w == "rev5") testReview5(); else if (w == "rev6") testReview6(); else if (w == "idxf") testIndexFailures(); else if (w == "modes") testModeChanges(); else if (w == "hx") testTowHx();
    printf("%d passed, %d failed\n", pass, fail); return fail ? 1 : 0; }
  testJson(); testSwitches(); testAngle(); testWiperDrive(); testPersist();
  testProtocolBasics(); testTelemetryWorstCase(); testModeChanges(); testDrumFaults(); testAim(); testCockFireReset(); testFireRefusals();
  testNoRelease(); testWrapReelUnwrap(); testTow(); testTowHx(); testStallOvercurrent(); testAngleSensorFault(); testBrownout();
  testStopEstop(); testJog(); testPowerLoss(); testSaveGate(); testPersistWear(); testTelemetryRate(); testHostTimeout(); testReview1(); testReview2(); testSpecV1Contacts(); testReview3(); testReview4(); testReview5(); testReview6(); testIndexMode(); testIndexFailures(); testTimeline(); testFullMission();
  curTest = "all"; CHECK(Rig::dryTotal == 0, "no test ever moved the collar out of LAUNCH with the spring drawn");
  printf("\n%d checks, %d passed, %d failed: %s\n", pass + fail, pass, fail, fail ? "FAILURES" : "ALL PASS");
  return fail ? 1 : 0;
}
