// GrappStar system test: the REAL base firmware logic (firmware/base/grappstar_base/base_logic.h, the same header the
// ESP32 sketch and the host tests compile) running against the firmware tests' plant model (firmware/base/test/sim.h),
// with the sketch's USB serial replaced by a pseudo-terminal. The Jetson code opens the pty exactly like
// /dev/ttyACM0, so it speaks contract C1 over a real serial device.
//
//   base_shim --link PATH --ctl PORT --state DIR [--speed X] [--fresh]
//
// --link   a symlink to the pty slave is (re)created here; it disappears while the USB is "unplugged"
// --ctl    TCP port on 127.0.0.1 for the test harness (one command per line, one JSON reply per line):
//            get                      plant truth + firmware state
//            set KEY VAL [KEY VAL..]  plant knobs and fault injection (see setKnob)
//            cmd {json}               run a C1 line straight into the firmware (bench setup), returns the reply
//            power off | power on     MCU power cut (plant keeps its state, NVS keeps what was written) / boot
//            unplug | replug          USB cable out / in (the USB powers the board: same as power off / on)
//            glitch                   USB re-enumeration without a reset (pty closed and reopened)
//            quit
// --state  folder for nvs.bin, tun.bin and plant.bin (plant.bin is rewritten every 100 ms of sim time, so a
//          kill -9 of this process followed by a restart without --fresh is a power loss with the plant intact)
// --speed  sim time / wall time (default 1)
//
// Wiring like the sketch: tick every 5 ms (plant step, then B.tick), the reply to a line before any event it raised,
// events never dropped (queued), telemetry dropped when the pty buffer is full, NVS writes take 20 ms and saveAck is on.
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <deque>
#include <string>

#include "../firmware/base/test/sim.h"

Sim S;

static gb::Base* B = nullptr;
static bool powered = false;
static std::string linkPath, stateDir;
static int masterFd = -1, slaveKeepFd = -1;
static std::deque<std::string> evq;                  // the sketch's event ring
static uint64_t evDrops = 0, telDrops = 0, linesIn = 0, linesOut = 0;
static bool nvsPending = false; static gb::Saved nvsBuf; static uint32_t nvsT = 0;
static uint32_t bootCount = 0;
static int dryTotal = 0; static double peakTension = 0, peakAmps = 0; static bool plantGeomSet = false;
static FILE* lgf = stdout;

// ------------------------------------------------------------------------------------------------ files
static std::string sp(const char* n) { return stateDir + "/" + n; }
static bool readFile(const std::string& p, void* buf, size_t n) {
  FILE* f = fopen(p.c_str(), "rb"); if (!f) return false;
  bool ok = fread(buf, 1, n, f) == n; fclose(f); return ok;
}
static void writeFile(const std::string& p, const void* buf, size_t n) {
  std::string tmp = p + ".tmp";
  FILE* f = fopen(tmp.c_str(), "wb"); if (!f) return;
  fwrite(buf, 1, n, f); fclose(f); rename(tmp.c_str(), p.c_str());
}

// plant truth that a power cut does not change (the worms self-lock): saved so a killed process can resume
struct Plant {
  uint32_t magic = 0x504c4e54;
  double wiperDeg, aimPh, cockPh, drumDeg, lineOut, anchorTautAt, anchorK, seatAtLine, win, slack, toothWiperAcc, lineOutAfterShot;
  int fired, shots, dogTip;
  int drumJam, encMissing, searBroken, toothOnTooth, dryFires; double extraLoadNm, vbatOverride, maxSlack, drumJamAbove;
};
static void savePlant() {
  Plant p; p.wiperDeg = S.wiperDeg; p.aimPh = S.aimPh; p.cockPh = S.cockPh; p.drumDeg = S.drumDeg; p.lineOut = S.lineOut;
  p.anchorTautAt = S.anchorTautAt; p.anchorK = S.anchorK; p.seatAtLine = S.seatAtLine; p.win = S.win; p.slack = S.slack;
  p.toothWiperAcc = S.toothWiperAcc; p.lineOutAfterShot = S.lineOutAfterShot; p.fired = S.fired; p.shots = S.shots; p.dogTip = S.dogTip;
  p.drumJam = S.drumJam; p.encMissing = S.encMissing; p.searBroken = S.searBroken; p.toothOnTooth = S.toothOnTooth; p.dryFires = dryTotal;
  p.extraLoadNm = S.extraLoadNm; p.vbatOverride = S.vbatOverride; p.maxSlack = S.maxSlack; p.drumJamAbove = S.drumJamAbove;
  writeFile(sp("plant.bin"), &p, sizeof p);
}
static bool loadPlant() {
  Plant p; if (!readFile(sp("plant.bin"), &p, sizeof p) || p.magic != 0x504c4e54) return false;
  S.wiperDeg = p.wiperDeg; S.aimPh = p.aimPh; S.cockPh = p.cockPh; S.drumDeg = p.drumDeg; S.lineOut = p.lineOut;
  S.anchorTautAt = p.anchorTautAt; S.anchorK = p.anchorK; S.seatAtLine = p.seatAtLine; S.win = p.win; S.slack = p.slack;
  S.toothWiperAcc = p.toothWiperAcc; S.lineOutAfterShot = p.lineOutAfterShot; S.fired = p.fired; S.shots = p.shots; S.dogTip = p.dogTip;
  S.drumJam = p.drumJam; S.encMissing = p.encMissing; S.searBroken = p.searBroken; S.toothOnTooth = p.toothOnTooth; dryTotal = p.dryFires;
  S.extraLoadNm = p.extraLoadNm; S.vbatOverride = p.vbatOverride; S.maxSlack = p.maxSlack; S.drumJamAbove = p.drumJamAbove;
  return true;
}

// ------------------------------------------------------------------------------------------------ pty
static void openPty() {
  masterFd = posix_openpt(O_RDWR | O_NOCTTY);
  if (masterFd < 0 || grantpt(masterFd) || unlockpt(masterFd)) { perror("pty"); exit(1); }
  const char* name = ptsname(masterFd);
  // keep one slave fd open (raw): the master never sees EIO while the Jetson reopens the port, and no echo
  slaveKeepFd = open(name, O_RDWR | O_NOCTTY);
  struct termios t; tcgetattr(slaveKeepFd, &t); cfmakeraw(&t); tcsetattr(slaveKeepFd, TCSANOW, &t);
  fcntl(masterFd, F_SETFL, fcntl(masterFd, F_GETFL) | O_NONBLOCK);
  unlink(linkPath.c_str());
  if (symlink(name, linkPath.c_str())) { perror("symlink"); exit(1); }
  fprintf(lgf, "[shim] pty %s -> %s\n", linkPath.c_str(), name);
}
static void closePty() {
  if (masterFd >= 0) { close(masterFd); masterFd = -1; }
  if (slaveKeepFd >= 0) { close(slaveKeepFd); slaveKeepFd = -1; }
  unlink(linkPath.c_str());
}
// whole line or nothing (like writeLine in the sketch). Returns false when there is no room.
static bool writeLine(const char* s) {
  if (masterFd < 0) return false;
  std::string l = std::string(s) + "\n";
  // room check: a pty buffer holds a few kB; a partial write is completed with a short blocking retry
  ssize_t n = write(masterFd, l.data(), l.size());
  if (n < 0) return false;
  size_t done = (size_t)n;
  for (int tries = 0; done < l.size() && tries < 200; tries++) {
    usleep(500); n = write(masterFd, l.data() + done, l.size() - done); if (n > 0) done += (size_t)n;
  }
  linesOut++;
  return done == l.size();
}

// ------------------------------------------------------------------------------------------------ firmware glue
static void shimSave(const gb::Saved& s) { if (!nvsPending) nvsT = S.t; nvsBuf = s; nvsPending = true; S.saves++; S.saveSeq = s.seq; S.saveT = S.t; }
static void shimSaveTun(const gb::Tun& t) { writeFile(sp("tun.bin"), &t, sizeof t); }
static void shimOut(const char* l) {
  if (evq.size() >= 24) { evDrops++; return; }      // the sketch's ring (RING 24): drop the newest
  evq.push_back(l);
  fprintf(lgf, "[%8u] EVT %s\n", (unsigned)S.t, l);
}
static void drainEvents() {
  while (!evq.empty()) { if (!writeLine(evq.front().c_str())) break; evq.pop_front(); }
}

static bool running = false; static uint32_t setupEnd = 0; static uint32_t helloDelayMs = 2100;
// power on: the USB port enumerates at once, setup() runs (1.5 s waiting for the host, 0.6 s probing the HX711), then
// HELLO and the tick task. Lines the host sends meanwhile wait in the CDC buffer (not read until loop() runs).
static void powerOn() {
  delete B; B = nullptr;
  evq.clear(); nvsPending = false;
  powered = true; running = false; bootCount++;
  setupEnd = S.t + helloDelayMs;
  openPty();
  fprintf(lgf, "[%8u] POWER ON #%u (setup until %u)\n", (unsigned)S.t, bootCount, (unsigned)setupEnd);
}
static void finishSetup() {
  B = new gb::Base();
  gb::Io io = simIo(); io.save = shimSave; io.saveTun = shimSaveTun; io.out = shimOut;
  B->io = io;
  B->saveAck = true;
  B->w.c.ampsPerVolt = (float)S.isAmpsPerVolt;
  gb::Tun tun; bool haveTun = readFile(sp("tun.bin"), &tun, sizeof tun);
  B->restoreTun(haveTun ? &tun : nullptr);
  if (!plantGeomSet) { S.c = B->c; plantGeomSet = true; }   // the plant's geometry is physical: fixed at the first boot
  B->begin(S.t);
  gb::Saved rec; bool haveRec = readFile(sp("nvs.bin"), &rec, sizeof rec);
  B->restore(haveRec ? &rec : nullptr);
  running = true;
  char hello[200];
  snprintf(hello, sizeof hello, "{\"evt\":\"HELLO\",\"fw\":\"%s\",\"reset\":\"POWERON\",\"hx711\":%s,\"as5600\":%s,\"pwm\":true}",
           GB_FW_VERSION, S.hxFitted ? "true" : "false", S.encMissing ? "false" : "true");
  writeLine(hello);
  fprintf(lgf, "[%8u] BOOT #%u (nvs %s, tun %s)\n", (unsigned)S.t, bootCount, haveRec ? "yes" : "no", haveTun ? "yes" : "no");
}
static void powerOff() {
  if (!powered) return;
  powered = false; running = false;
  S.wDir = 0; S.wDuty = 0; S.dDir = 0; S.dDuty = 0; S.drumAwake = false;   // bridges lose their inputs (pull-downs)
  nvsPending = false;                                                      // an NVS write in flight is lost
  closePty();
  fprintf(lgf, "[%8u] POWER OFF\n", (unsigned)S.t);
}

static std::string rxLine; static bool rxOver = false;
static void serviceSerial() {
  if (masterFd < 0) return;
  char buf[512];
  for (;;) {
    ssize_t n = read(masterFd, buf, sizeof buf);
    if (n <= 0) break;
    for (ssize_t i = 0; i < n; i++) {
      char ch = buf[i];
      if (ch == '\n' || ch == '\r') {
        if (!rxLine.empty() || rxOver) {
          char rep[300];
          linesIn++;
          if (rxOver) snprintf(rep, sizeof rep, "{\"id\":null,\"ok\":false,\"err\":\"TOO_LONG\"}");
          else B->handleLine(rxLine.c_str(), (int)rxLine.size(), rep, sizeof rep);
          fprintf(lgf, "[%8u] RX  %s\n[%8u] TX  %s\n", (unsigned)S.t, rxLine.c_str(), (unsigned)S.t, rep);
          writeLine(rep);
          drainEvents();
        }
        rxLine.clear(); rxOver = false;
        continue;
      }
      if ((int)rxLine.size() < gb::JSON_MAX_LINE) rxLine += ch; else rxOver = true;
    }
  }
}

static uint32_t lastPlantSave = 0;
// Plant additions on top of sim.h (D22): laying a capstan turn draws one capstan circumference of line from the dart
// side. A free dart gets tugged (line out falls); an anchored dart side only gives what slack it has, then the cup
// cannot turn (the drum stalls in the wrap sector). A forward wiper with the collar in neither train (tidy) pulls an
// anchored dart side straight through the free-turning capstan, down to taut.
static int wrapBlocks = 0; static double dartTug = 0; static bool wrapDraw = true;
static void plantExtra(double drumBefore, double wrapsBefore, double wiperBefore) {
  double dw = S.wrapsExact() - wrapsBefore;
  if (dw > 1e-9 && wrapDraw) {
    double need = dw * 2 * M_PI * S.capstanR;
    if (S.anchorTautAt >= 0) {
      double avail = S.lineOut - S.anchorTautAt;
      if (need > avail + 1e-9) { S.drumDeg = drumBefore; wrapBlocks++; }
      else S.lineOut -= need;
    } else { S.lineOut = fmax(0.0, S.lineOut - need); dartTug += need; }
  }
  double ad = fabs(S.wiperDeg - wiperBefore);
  if (S.wDir > 0 && ad > 0 && !S.inLaunch() && !S.inReel() && S.anchorTautAt >= 0 && S.lineOut > S.anchorTautAt)
    S.lineOut = fmax(S.anchorTautAt, S.lineOut - ad / 360.0 * 0.322);
}

static void tickOnce() {
  double drumBefore = S.drumDeg, wrapsBefore = S.wrapsExact(), wiperBefore = S.wiperDeg;
  S.step(5);
  plantExtra(drumBefore, wrapsBefore, wiperBefore);
  if (powered && !running && (int32_t)(S.t - setupEnd) >= 0) finishSetup();
  if (S.dryFires) { dryTotal += S.dryFires; S.dryFires = 0; }
  peakTension = fmax(peakTension, S.tension()); peakAmps = fmax(peakAmps, S.amps);
  if (!running) return;
  B->tick(S.t);
  if (nvsPending && (int32_t)(S.t - nvsT) >= 20) {   // NVS write done
    writeFile(sp("nvs.bin"), &nvsBuf, sizeof nvsBuf);
    B->saveDoneSeq = nvsBuf.seq; nvsPending = false;
  }
}

// ------------------------------------------------------------------------------------------------ control socket
static int ctlListen = -1, ctlFd = -1; static std::string ctlBuf;
static double num(const char* v) { return atof(v); }
static bool setKnob(const std::string& k, const char* v) {
  if (k == "lineOut") S.lineOut = num(v);
  else if (k == "lineOutAfterShot") S.lineOutAfterShot = num(v);
  else if (k == "anchorTautAt") S.anchorTautAt = num(v);
  else if (k == "anchorK") S.anchorK = num(v);
  else if (k == "seatAtLine") S.seatAtLine = num(v);
  else if (k == "drumJam") S.drumJam = num(v) != 0;
  else if (k == "drumJamAbove") S.drumJamAbove = num(v);
  else if (k == "drumDpsTrue") S.drumDpsTrue = num(v);
  else if (k == "toothOnTooth") S.toothOnTooth = num(v) != 0;
  else if (k == "toothNeedWiper") S.toothNeedWiper = num(v);
  else if (k == "extraLoadNm") S.extraLoadNm = num(v);
  else if (k == "cockMaxNm") S.cockMaxNm = num(v);
  else if (k == "encMissing") S.encMissing = num(v) != 0;
  else if (k == "encGlitchEvery") S.encGlitchEvery = (int)num(v);
  else if (k == "magnetWeak") S.magnetWeak = num(v) != 0;
  else if (k == "searBroken") S.searBroken = num(v) != 0;
  else if (k == "vbatOverride") S.vbatOverride = num(v);
  else if (k == "swStuckOpen") S.swStuckOpen = (uint16_t)num(v);
  else if (k == "swStuckClosed") S.swStuckClosed = (uint16_t)num(v);
  else if (k == "swChatter") S.swChatter = (uint16_t)num(v);
  else if (k == "drumDeg") S.drumDeg = num(v);              // bench: drum turned by hand (manual lever) before the run
  else if (k == "wrapDraw") wrapDraw = num(v) != 0;
  else if (k == "hxFitted") S.hxFitted = num(v) != 0;
  else return false;
  return true;
}
static std::string getJson() {
  char b[1400];
  const char* st = "OFF"; static const char* SN[] = {"IDLE", "MODE", "AIM", "COCK", "FIRE", "WRAP", "UNWRAP", "REEL", "HOLD", "JOG", "DECOCK", "SAFING", "FAULT"};
  if (running) st = (B->state >= 0 && B->state < 13) ? SN[B->state] : "?";
  snprintf(b, sizeof b,
           "{\"t\":%u,\"powered\":%s,\"boots\":%u,\"state\":\"%s\",\"fault\":%d,\"shots\":%d,\"fired\":%s,\"lineOut\":%.3f,"
           "\"tension\":%.2f,\"seated\":%s,\"drumDeg\":%.2f,\"wiperDeg\":%.1f,\"aimPh\":%.1f,\"cockPh\":%.1f,\"wraps\":%.2f,"
           "\"inLaunch\":%s,\"inReel\":%s,\"dogTip\":%s,\"anchorTautAt\":%.3f,\"slack\":%.3f,\"maxSlack\":%.3f,\"amps\":%.2f,"
           "\"wDir\":%d,\"wDuty\":%d,\"dDir\":%d,\"dryFires\":%d,\"evDrops\":%llu,\"linesIn\":%llu,\"linesOut\":%llu,\"evq\":%zu,"
           "\"pty\":%s,\"saves\":%d,\"kNperA_cal\":%.4f,\"freeA_cal\":%.4f,\"peakTension\":%.2f,\"peakAmps\":%.2f,\"wrapBlocks\":%d,\"dartTug\":%.3f}",
           (unsigned)S.t, powered ? "true" : "false", bootCount, st, running ? B->fault : -1, S.shots, S.fired ? "true" : "false",
           S.lineOut, S.tension(), S.seated() ? "true" : "false", S.drumDeg, S.wiperDeg, S.aimPh, S.cockPh, S.wrapsExact(),
           S.inLaunch() ? "true" : "false", S.inReel() ? "true" : "false", S.dogTip ? "true" : "false", S.anchorTautAt, S.slack,
           S.maxSlack, S.amps, S.wDir, S.wDuty, S.dDir, dryTotal, (unsigned long long)evDrops, (unsigned long long)linesIn,
           (unsigned long long)linesOut, evq.size(), masterFd >= 0 ? "true" : "false", S.saves,
           1.0 / (S.capstanR * S.c.capstanPerWiper) * S.stallNm / (S.stallA - S.freeA), S.freeA + (S.reelFreeNm + 0.3) / S.stallNm * (S.stallA - S.freeA), peakTension, peakAmps, wrapBlocks, dartTug);
  return b;
}
static bool quitReq = false;
static std::string ctlCommand(const std::string& line) {
  char w[64] = ""; sscanf(line.c_str(), "%63s", w);
  std::string c = w;
  if (c == "get") return getJson();
  if (c == "peakreset") { std::string r = getJson(); peakTension = S.tension(); peakAmps = S.amps; return r; }   // returns the peaks since the last reset
  if (c == "set") {
    const char* p = line.c_str() + 3; char k[64], v[64]; int n;
    while (sscanf(p, "%63s %63s%n", k, v, &n) == 2) { if (!setKnob(k, v)) return std::string("{\"ok\":false,\"err\":\"bad key ") + k + "\"}"; p += n; }
    fprintf(lgf, "[%8u] CTL %s\n", (unsigned)S.t, line.c_str());
    return "{\"ok\":true}";
  }
  if (c == "cmd") {
    if (!running) return "{\"ok\":false,\"err\":\"powered off\"}";
    std::string j = line.substr(4); char rep[300];
    B->handleLine(j.c_str(), (int)j.size(), rep, sizeof rep);
    fprintf(lgf, "[%8u] CTL-RX %s\n[%8u] CTL-TX %s\n", (unsigned)S.t, j.c_str(), (unsigned)S.t, rep);
    return rep;
  }
  if (line == "power off" || c == "unplug") { powerOff(); return "{\"ok\":true}"; }
  if (line == "power on" || c == "replug") { if (!powered) powerOn(); return "{\"ok\":true}"; }
  if (c == "glitch") { if (powered) { closePty(); usleep(300000); openPty(); } fprintf(lgf, "[%8u] USB GLITCH\n", (unsigned)S.t); return "{\"ok\":true}"; }
  if (c == "inject") {                               // raw bytes onto the serial line (escapes: \n \xHH), for wrong / malformed replies
    std::string raw, a = line.size() > 7 ? line.substr(7) : "";
    for (size_t i = 0; i < a.size(); i++) {
      if (a[i] == '\\' && i + 1 < a.size() && a[i + 1] == 'n') { raw += '\n'; i++; }
      else if (a[i] == '\\' && i + 3 < a.size() && a[i + 1] == 'x') { raw += (char)strtol(a.substr(i + 2, 2).c_str(), nullptr, 16); i += 3; }
      else raw += a[i];
    }
    if (masterFd >= 0 && write(masterFd, raw.data(), raw.size()) < 0) {}
    fprintf(lgf, "[%8u] INJECT %s\n", (unsigned)S.t, a.c_str());
    return "{\"ok\":true}";
  }
  if (c == "quit") { quitReq = true; return "{\"ok\":true}"; }
  return "{\"ok\":false,\"err\":\"unknown\"}";
}
static void serviceCtl() {
  if (ctlFd < 0) {
    int fd = accept(ctlListen, nullptr, nullptr);
    if (fd >= 0) { fcntl(fd, F_SETFL, O_NONBLOCK); ctlFd = fd; ctlBuf.clear(); }
    return;
  }
  char buf[1024];
  ssize_t n = read(ctlFd, buf, sizeof buf);
  if (n == 0 || (n < 0 && errno != EAGAIN)) { close(ctlFd); ctlFd = -1; return; }
  if (n > 0) ctlBuf.append(buf, n);
  size_t p;
  while ((p = ctlBuf.find('\n')) != std::string::npos) {
    std::string l = ctlBuf.substr(0, p); ctlBuf.erase(0, p + 1);
    std::string r = ctlCommand(l) + "\n";
    if (write(ctlFd, r.data(), r.size()) < 0) {}
  }
}

static double wallMs() { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6; }

int main(int argc, char** argv) {
  signal(SIGPIPE, SIG_IGN);
  int ctlPort = 0; double speed = 1.0; bool fresh = false;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--link" && i + 1 < argc) linkPath = argv[++i];
    else if (a == "--ctl" && i + 1 < argc) ctlPort = atoi(argv[++i]);
    else if (a == "--state" && i + 1 < argc) stateDir = argv[++i];
    else if (a == "--speed" && i + 1 < argc) speed = atof(argv[++i]);
    else if (a == "--fresh") fresh = true;
    else if (a == "--hello-delay" && i + 1 < argc) helloDelayMs = (uint32_t)atoi(argv[++i]);
    else { fprintf(stderr, "usage: base_shim --link PATH --ctl PORT --state DIR [--speed X] [--fresh]\n"); return 2; }
  }
  if (linkPath.empty() || !ctlPort || stateDir.empty()) { fprintf(stderr, "need --link, --ctl, --state\n"); return 2; }
  setvbuf(stdout, nullptr, _IOLBF, 0);
  mkdir(stateDir.c_str(), 0755);
  S = Sim();
  { gb::Base tmp; tmp.begin(0); S.c = tmp.c; plantGeomSet = true; }   // factory geometry (begin loads the aim table)
  S.lineOut = 0; S.seatAtLine = 0;                     // a new build: dart seated on the launcher
  if (fresh) { unlink(sp("nvs.bin").c_str()); unlink(sp("tun.bin").c_str()); unlink(sp("plant.bin").c_str()); }
  else if (loadPlant()) fprintf(lgf, "[shim] plant state restored (process restart = power loss)\n");

  ctlListen = socket(AF_INET, SOCK_STREAM, 0);
  int one = 1; setsockopt(ctlListen, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
  sockaddr_in sa{}; sa.sin_family = AF_INET; sa.sin_port = htons(ctlPort); sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (bind(ctlListen, (sockaddr*)&sa, sizeof sa) || listen(ctlListen, 4)) { perror("ctl bind"); return 1; }
  fcntl(ctlListen, F_SETFL, O_NONBLOCK);

  powerOn();
  double w0 = wallMs(); uint32_t t0 = S.t;
  while (!quitReq) {
    double target = t0 + (wallMs() - w0) * speed;
    int n = 0;
    while (S.t + 5 <= target && n < 400) {
      tickOnce(); n++;
      if (running) {
        serviceSerial();                                 // lines are handled between ticks, like loop() on core 0
        drainEvents();
        if (B->telDue(S.t)) { char tb[300]; B->telemetry(tb, sizeof tb); if (!writeLine(tb)) telDrops++; }
      }
      if (S.t - lastPlantSave >= 100) { savePlant(); lastPlantSave = S.t; }
    }
    if (running) serviceSerial();
    serviceCtl();
    usleep(1000);
  }
  savePlant();
  closePty();
  return 0;
}
