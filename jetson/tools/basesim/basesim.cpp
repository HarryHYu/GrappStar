// basesim: firmware-in-the-loop simulator of the GrappStar BASE station.
//
// Runs the REAL base firmware logic (firmware/base/grappstar_base/base_logic.h) against the firmware's own physics
// model (firmware/base/test/sim.h) in real time (or faster with --speed) and speaks the C1 JSON-lines protocol on
// stdin/stdout, or on a pseudo terminal with --pty, exactly like the ESP32 USB port. See README.md.
//
// The glue copies what the sketch (grappstar_base.ino) and the firmware PC tests (test/test_base.cpp, struct Rig) do:
// 5 ms tick, saveAck with the flash write acknowledged 20 ms later, HELLO at start, reply first then the events the
// command raised, telemetry when B.telDue() says so.
//
// Lines starting with "#sim " never reach the firmware; they control the scenario (README.md).
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <string>
#include <vector>

#include "sim.h"

Sim S;

namespace {

gb::Base B;
int outFd = 1;
bool ptyMode = false;
unsigned long dropped = 0;
int dryFiresTotal = 0;
volatile sig_atomic_t quit = 0;

void onSignal(int) { quit = 1; }

// write one whole line (or nothing, in pty mode when nobody reads: like the sketch dropping telemetry)
void writeLine(const std::string& s) {
  if (s.size() > (size_t)gb::JSON_MAX_LINE) { dropped++; return; }   // the sketch never sends a line over 256 bytes
  std::string l = s + "\n";
  size_t off = 0;
  while (off < l.size()) {
    ssize_t n = write(outFd, l.data() + off, l.size() - off);
    if (n > 0) { off += (size_t)n; continue; }
    if (n < 0 && errno == EINTR) continue;
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      if (off == 0) { dropped++; return; }                  // nothing written: drop the line
      struct pollfd p = {outFd, POLLOUT, 0}; poll(&p, 1, 50); continue;   // half written: finish it
    }
    if (!ptyMode) { quit = 1; return; }                     // stdout closed (EPIPE): the client went away
    dropped++; return;
  }
}

void flushEvents() {
  for (auto& l : S.out) writeLine(l);
  S.out.clear();
  if (S.dryFires) { dryFiresTotal += S.dryFires; S.dryFires = 0; }
}

// one 5 ms tick, exactly as test_base.cpp Rig::tick
void tick() {
  S.step(5); B.tick(S.t);
  if (S.saveSeq != B.saveDoneSeq && (int32_t)(S.t - S.saveT) >= 20) B.saveDoneSeq = S.saveSeq;   // NVS write done
  flushEvents();
  if (B.telDue(S.t)) { char tel[300]; B.telemetry(tel, sizeof tel); writeLine(tel); }
}

// ------------------------------------------------------------------------------------------------ scenario control
struct Field { const char* name; char kind; void* p; };   // kind: d double, b bool, i int, u uint8
std::vector<Field> fields() {
  return {
      {"lineOut", 'd', &S.lineOut}, {"anchorTautAt", 'd', &S.anchorTautAt}, {"anchorK", 'd', &S.anchorK},
      {"seatAtLine", 'd', &S.seatAtLine}, {"lineOutAfterShot", 'd', &S.lineOutAfterShot}, {"slack", 'd', &S.slack},
      {"drumJam", 'b', &S.drumJam}, {"drumJamAbove", 'd', &S.drumJamAbove}, {"toothOnTooth", 'b', &S.toothOnTooth},
      {"toothNeedWiper", 'd', &S.toothNeedWiper}, {"extraLoadNm", 'd', &S.extraLoadNm}, {"searBroken", 'b', &S.searBroken},
      {"encMissing", 'b', &S.encMissing}, {"encGlitchEvery", 'i', &S.encGlitchEvery}, {"magnetWeak", 'b', &S.magnetWeak},
      {"encSign", 'i', &S.encSign}, {"swStuckClosed", 'u', &S.swStuckClosed}, {"swStuckOpen", 'u', &S.swStuckOpen},
      {"swChatter", 'u', &S.swChatter}, {"vbatOverride", 'd', &S.vbatOverride}, {"vbat0", 'd', &S.vbat0},
      {"vbatWired", 'b', &S.vbatWired}, {"hxFitted", 'b', &S.hxFitted}, {"drumDpsTrue", 'd', &S.drumDpsTrue},
      {"aimLoadNm", 'd', &S.aimLoadNm}, {"cockMaxNm", 'd', &S.cockMaxNm}, {"reelFreeNm", 'd', &S.reelFreeNm},
  };
}

void simErr(const char* fmt, const char* a) { fprintf(stderr, "basesim: "); fprintf(stderr, fmt, a); fprintf(stderr, "\n"); }

bool setField(const char* name, const char* val) {
  char* end = nullptr; double v = strtod(val, &end);
  if (!strcmp(val, "true") || !strcmp(val, "on")) v = 1; else if (!strcmp(val, "false") || !strcmp(val, "off")) v = 0;
  else if (!end || end == val || *end) { simErr("bad value '%s'", val); return false; }
  for (auto& f : fields()) if (!strcmp(f.name, name)) {
    switch (f.kind) {
      case 'd': *(double*)f.p = v; break;
      case 'b': *(bool*)f.p = v != 0; break;
      case 'i': *(int*)f.p = (int)v; break;
      case 'u': *(uint8_t*)f.p = (uint8_t)(int)v; break;
    }
    return true;
  }
  simErr("unknown sim field '%s'", name);
  return false;
}

void dump() {
  char b[400];
  snprintf(b, sizeof b,
           "{\"sim\":{\"t\":%u,\"line_out\":%.3f,\"anchor_taut_at\":%.3f,\"seat_at\":%.3f,\"seated\":%s,\"tension_n\":%.2f,"
           "\"shots\":%d,\"drum_deg\":%.2f,\"wiper_deg\":%.1f,\"aim_ph\":%.1f,\"cock_ph\":%.1f,\"wraps\":%d,"
           "\"slack\":%.3f,\"dry_fires\":%d,\"dropped\":%lu}}",
           (unsigned)S.t, S.lineOut, S.anchorTautAt, S.seatAtLine, S.seated() ? "true" : "false", S.tension(), S.shots,
           S.drumDeg, S.wiperDeg, S.aimPh, S.cockPh, S.wrapsTrue(), S.slack, dryFiresTotal, dropped);
  writeLine(b);
}

void simCommand(const char* rest) {
  char a[64] = "", b[64] = "", c[64] = "";
  int n = sscanf(rest, "%63s %63s %63s", a, b, c);
  if (n < 1) return;
  if (!strcmp(a, "anchor") && n >= 2) { setField("anchorTautAt", b); return; }
  if (!strcmp(a, "free")) { S.anchorTautAt = -1; return; }
  if (!strcmp(a, "seat") && n >= 2) { setField("seatAtLine", b); return; }
  if ((!strcmp(a, "set") || !strcmp(a, "fault")) && n >= 3) { setField(b, c); return; }
  if (!strcmp(a, "dump")) { dump(); return; }
  simErr("unknown #sim command '%s'", rest);
}

// ------------------------------------------------------------------------------------------------ line input
struct LineIn {
  char line[gb::JSON_MAX_LINE + 2]; int len = 0; bool over = false;
  std::string raw;                                    // the whole line, also when too long (for "#sim ")
};

void handleLine(LineIn& L) {
  if (L.raw.compare(0, 5, "#sim ") == 0 || L.raw == "#sim") { simCommand(L.raw.c_str() + 4); return; }
  char rep[300];
  if (L.over) snprintf(rep, sizeof rep, "{\"id\":null,\"ok\":false,\"err\":\"TOO_LONG\"}");
  else { L.line[L.len] = 0; B.handleLine(L.line, L.len, rep, sizeof rep); }
  writeLine(rep);                                     // the reply first, then any event the command raised
  flushEvents();
}

void feed(LineIn& L, const char* buf, ssize_t n) {
  for (ssize_t i = 0; i < n; i++) {
    char ch = buf[i];
    if (ch == '\n' || ch == '\r') {
      if (L.len > 0 || L.over) handleLine(L);
      L.len = 0; L.over = false; L.raw.clear();
      continue;
    }
    if (L.raw.size() < 4096) L.raw += ch;
    if (L.len < gb::JSON_MAX_LINE) L.line[L.len++] = ch; else L.over = true;
  }
}

double wallMs() { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6; }

void usage() {
  fprintf(stderr,
          "usage: basesim [--speed X] [--pty] [--line-out M] [--anchor-taut-at M] [--seat-at M] [--load-cell]\n"
          "  C1 JSON lines on stdin/stdout (or on a pty with --pty: its path is printed on stderr).\n"
          "  Lines starting with '#sim ' control the scenario: anchor M | free | seat M | set NAME VAL |\n"
          "  fault NAME VAL | dump. See README.md.\n");
}

}  // namespace

int main(int argc, char** argv) {
  double speed = 1.0, lineOut = 0.0, anchor = -1.0, seatAt = 0.0; bool loadCell = false;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto val = [&](double* d) { if (i + 1 >= argc) { usage(); exit(2); } *d = atof(argv[++i]); };
    if (a == "--speed") val(&speed);
    else if (a == "--line-out") val(&lineOut);
    else if (a == "--anchor-taut-at") val(&anchor);
    else if (a == "--seat-at") val(&seatAt);
    else if (a == "--load-cell") loadCell = true;
    else if (a == "--pty") ptyMode = true;
    else if (a == "-h" || a == "--help") { usage(); return 0; }
    else { usage(); return 2; }
  }
  if (!(speed > 0 && speed <= 1000)) { fprintf(stderr, "basesim: --speed must be in (0, 1000]\n"); return 2; }
  signal(SIGPIPE, SIG_IGN); signal(SIGINT, onSignal); signal(SIGTERM, onSignal); signal(SIGHUP, onSignal);

  int inFd = 0, slaveFd = -1;
  if (ptyMode) {
    int m = posix_openpt(O_RDWR | O_NOCTTY);
    if (m < 0 || grantpt(m) || unlockpt(m)) { perror("basesim: pty"); return 1; }
    const char* name = ptsname(m);
    if (!name) { perror("basesim: ptsname"); return 1; }
    // keep our own slave open: the master then never sees a hang-up while no client has the port open
    slaveFd = open(name, O_RDWR | O_NOCTTY);
    if (slaveFd < 0) { perror("basesim: open slave"); return 1; }
    struct termios t; tcgetattr(slaveFd, &t); cfmakeraw(&t); tcsetattr(slaveFd, TCSANOW, &t);
    fcntl(m, F_SETFL, fcntl(m, F_GETFL) | O_NONBLOCK);
    inFd = outFd = m;
    fprintf(stderr, "%s\n", name); fflush(stderr);
  }

  // ---- the rig (test_base.cpp Rig): fresh sim, firmware config copied into the sim, saveAck, begin, restore
  S = Sim();
  S.c = B.c;
  S.lineOut = lineOut; S.anchorTautAt = anchor; S.seatAtLine = seatAt; S.hxFitted = loadCell;
  B.io = simIo();
  B.saveAck = true;
  B.w.c.ampsPerVolt = (float)S.isAmpsPerVolt;
  // a calibrated bench (CALIBRATION.md): the tension estimate constants match the simulated motor (as test_base.cpp tow)
  B.c.kNperA = (float)(1.0 / (S.capstanR * S.c.capstanPerWiper) * S.stallNm / (S.stallA - S.freeA));
  B.c.freeA = 1.5f + (float)((S.reelFreeNm + 0.3) / S.stallNm * (S.stallA - S.freeA));
  B.begin(S.t);
  B.restore(nullptr);
  {
    char hello[200];
    snprintf(hello, sizeof hello, "{\"evt\":\"HELLO\",\"fw\":\"%s\",\"reset\":\"POWERON\",\"hx711\":%s,\"as5600\":true,\"pwm\":true}",
             GB_FW_VERSION, loadCell ? "true" : "false");
    writeLine(hello);
  }
  // a new build is declared at rest once (test_base.cpp does the same); runs silently before the link is served
  for (int i = 0; i < 20; i++) { S.step(5); B.tick(S.t); if (S.saveSeq != B.saveDoneSeq && (int32_t)(S.t - S.saveT) >= 20) B.saveDoneSeq = S.saveSeq; }
  { char rep[300]; const char* c = "{\"id\":0,\"cmd\":\"cal\",\"what\":\"cock_rest\"}"; B.handleLine(c, (int)strlen(c), rep, sizeof rep); }
  flushEvents();                                      // BOOT

  LineIn L;
  bool stdinOpen = true;
  double w0 = wallMs(); uint32_t t0 = S.t;
  while (!quit) {
    double target = t0 + (wallMs() - w0) * speed;     // sim ms that should have elapsed
    int n = 0;
    while (S.t + 5 <= target && !quit) {
      tick();
      if (++n >= 2000) { w0 = wallMs(); t0 = S.t; break; }   // could not keep up: slip instead of bursting forever
    }
    double waitSim = (S.t + 5) - (t0 + (wallMs() - w0) * speed);
    double waitMs = waitSim / speed; if (waitMs < 0) waitMs = 0; if (waitMs > 50) waitMs = 50;
    struct pollfd p[2]; int np = 0;
    int idxIn = -1, idxStd = -1;
    if (ptyMode) { p[np] = {inFd, POLLIN, 0}; idxIn = np++; if (stdinOpen) { p[np] = {0, POLLIN, 0}; idxStd = np++; } }
    else if (stdinOpen) { p[np] = {0, POLLIN, 0}; idxStd = np++; }
    struct timespec ts = {(time_t)(waitMs / 1000), (long)((waitMs - (long)(waitMs / 1000) * 1000) * 1e6)};
    int r = ppoll(p, np, &ts, nullptr);
    if (r < 0) { if (errno == EINTR) continue; perror("basesim: poll"); break; }
    char buf[1024];
    if (idxIn >= 0 && (p[idxIn].revents & POLLIN)) {
      ssize_t k = read(inFd, buf, sizeof buf);
      if (k > 0) feed(L, buf, k);
    }
    if (idxStd >= 0 && (p[idxStd].revents & (POLLIN | POLLHUP | POLLERR))) {
      ssize_t k = read(0, buf, sizeof buf);
      if (k > 0) feed(L, buf, k);
      else if (k == 0 || (k < 0 && errno != EINTR && errno != EAGAIN)) {
        stdinOpen = false;
        if (!ptyMode) break;                          // stdio mode: the client closed our stdin
      }
    }
  }
  if (slaveFd >= 0) close(slaveFd);
  if (dryFiresTotal) fprintf(stderr, "basesim: WARNING the collar left LAUNCH with the spring drawn %d times\n", dryFiresTotal);
  return 0;
}
