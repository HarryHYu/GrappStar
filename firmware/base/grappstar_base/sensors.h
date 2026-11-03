// Sensor logic for the base: debounced printed contact switches with chatter detection, and the AS5600 multi-turn
// angle tracker with magnet health and rate. Plain C++ (no Arduino calls), tested on a PC by test/test_base.cpp.
#pragma once
#include <stdint.h>
#include <math.h>

namespace gb {

// ---------------------------------------------------------------------------------------------------------------
// Printed contacts (M3 screw head on an M3 nut) bounce and chatter much more than a microswitch, more so with the
// wiper running. Each bit: the raw level must hold debounceMs before the debounced level follows. A bit whose raw
// level changed more than chatterEdges times inside chatterWinMs is marked chattering until it has been quiet for
// chatterQuietMs; the logic then treats that contact as unreliable (SWITCH_DISAGREE if it matters).
static const int MAX_SW = 8;

struct Switches {
  uint32_t debounceMs = 15, chatterWinMs = 250, chatterQuietMs = 1000; int chatterEdges = 10;
  uint8_t stable = 0, prev = 0, cand = 0, raw = 0, chatter = 0;
  uint32_t candT[MAX_SW] = {0}, edgeT[MAX_SW][16] = {{0}}, lastEdge[MAX_SW] = {0};
  uint8_t edgeHead[MAX_SW] = {0}; uint16_t edgeCount[MAX_SW] = {0};
  bool inited = false;

  void update(uint8_t r, uint32_t now) {
    if (!inited) { stable = prev = cand = raw = r; inited = true; for (int i = 0; i < MAX_SW; i++) candT[i] = now; return; }
    prev = stable;
    uint8_t changed = r ^ raw; raw = r;
    for (int i = 0; i < MAX_SW; i++) {
      uint8_t m = 1u << i;
      if (changed & m) {
        candT[i] = now; lastEdge[i] = now;
        edgeT[i][edgeHead[i]] = now; edgeHead[i] = (edgeHead[i] + 1) & 15; if (edgeCount[i] < 16) edgeCount[i]++;
        int n = 0;                                           // edges inside the window
        for (int k = 0; k < edgeCount[i]; k++) if (now - edgeT[i][k] <= chatterWinMs) n++;
        if (n > chatterEdges && chatterEdges < 16) chatter |= m;
      }
      if ((chatter & m) && now - lastEdge[i] >= chatterQuietMs) { chatter &= ~m; edgeCount[i] = 0; }
      if (((raw ^ stable) & m) && now - candT[i] >= debounceMs) stable = (stable & ~m) | (raw & m);
    }
    cand = raw;
  }
  bool closed(int i) const { return i >= 0 && i < MAX_SW && (stable >> i) & 1; }
  bool rose(int i) const { return i >= 0 && i < MAX_SW && ((stable & ~prev) >> i) & 1; }     // just closed
  bool fell(int i) const { return i >= 0 && i < MAX_SW && ((~stable & prev) >> i) & 1; }     // just opened
  bool chattering(int i) const { return i >= 0 && i < MAX_SW && (chatter >> i) & 1; }
};

// ---------------------------------------------------------------------------------------------------------------
// AS5600: 12-bit absolute angle of the wiper output shaft. The tracker unwraps it into a multi-turn count, rejects
// single-sample glitches (a step bigger than maxStepDeg in one tick is physically impossible for a wiper at about
// 60 rpm), watches the magnet status (MD detected, ML too weak, MH too strong) and filters the rate.
enum { AS_MH = 0x08, AS_ML = 0x10, AS_MD = 0x20 };

struct AngleTracker {
  int sign = +1;                // +1 if the raw count rises while the wiper runs dir +1 (CALIBRATION.md step 3)
  float maxStepDeg = 45;        // per tick (5 ms): 45 deg in 5 ms = 1500 rpm, far above the wiper
  int missLimit = 4;            // this many bad reads in a row = sensor lost
  int32_t count = 0;            // multi-turn, 4096 per output turn, sign applied
  int lastRaw = 0; bool have = false, ok = false; int miss = 0, glitchRun = 0;
  uint32_t glitches = 0, misses = 0; uint8_t status = 0;
  float rateDps = 0;            // filtered deg/s
  int32_t rateRef = 0; uint32_t rateAcc = 0;

  // raw: 0..4095, or -1 when the I2C read failed. st: STATUS register (only meaningful when raw >= 0).
  void feed(int raw, uint8_t st, uint32_t dtMs) {
    if (raw >= 0) status = st;
    bool magnet = raw >= 0 && (st & AS_MD);
    if (!magnet) {
      misses++;
      if (++miss >= missLimit) { ok = false; rateDps = 0; }
      return;
    }
    miss = 0;
    if (!have || !ok) {                                      // first reading or back after a loss: take it as is
      lastRaw = raw & 4095; have = true; ok = true; glitchRun = 0; rateRef = count; rateAcc = 0; return;
    }
    int d = (raw & 4095) - lastRaw;
    if (d > 2047) d -= 4096;
    if (d < -2048) d += 4096;
    float stepDeg = fabsf(d * 360.f / 4096.f);
    if (stepDeg > maxStepDeg) {                              // glitch: ignore, unless it persists (then resync, uncounted)
      glitches++;
      if (++glitchRun < 3) return;
      lastRaw = raw & 4095; glitchRun = 0; return;
    }
    glitchRun = 0;
    lastRaw = raw & 4095;
    count += d * sign;
    rateAcc += dtMs;
    if (rateAcc >= 50) {                                     // rate over 50 ms windows, light smoothing
      float r = (count - rateRef) * 360.f / 4096.f * 1000.f / rateAcc;
      rateDps += (r - rateDps) * 0.5f;
      rateRef = count; rateAcc = 0;
    }
  }
  float deg() const { return count * 360.f / 4096.f; }
  float rpm() const { return rateDps / 6.f; }
  bool magnetWeak() const { return status & AS_ML; }
  bool magnetStrong() const { return status & AS_MH; }
  // set the multi-turn count so that deg() == d, keeping the current raw position (used after a restore)
  void setDeg(float d) { count = (int32_t)lroundf(d * 4096.f / 360.f); rateRef = count; }
  // absolute angle within one turn (0..360), sign applied; -1 if no reading yet
  float absDeg() const { if (!have) return -1; float a = (sign > 0 ? lastRaw : (4096 - lastRaw) % 4096) * 360.f / 4096.f; return a; }
};

// Battery and wiper-current scaling helpers --------------------------------------------------------------------
struct Lowpass { float y = 0; bool init = false; float a = 0.2f; float step(float x) { if (!init) { y = x; init = true; } else y += (x - y) * a; return y; } };

}  // namespace gb
