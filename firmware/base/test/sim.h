// PC simulation of the base station hardware for the firmware tests: wiper motor (DC motor line, self-locking worm),
// AS5600, BTS7960 IS output, the selector drum N20 with its detent contacts and wrap index, the aim train, the
// cock / sear / release train, the capstan with a line model (slack retrieve or anchored tow), the battery and an
// optional HX711. Faults can be injected. One global instance (the firmware Io uses plain function pointers).
#pragma once
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
#include "../grappstar_base/base_logic.h"

struct Sim {
  gb::Cfg c;                       // copy of the firmware config (geometry)
  // ---- truth
  double wiperDeg = 0;             // wiper output angle
  double aimPh = 0, cockPh = 0;    // train phases (deg of wiper travel)
  double drumDeg = 0;              // drum angle
  bool fired = false; int shots = 0;
  double lineOut = 5.0;            // m out
  double anchorTautAt = -1;        // tow: line length at which the anchored line goes taut (-1 = slack retrieve)
  double anchorK = 40;             // N per m of capstan travel after taut
  double seatAtLine = 0.0;         // retrieve: dart seats when lineOut reaches this
  // ---- drive inputs
  int wDir = 0, wDuty = 0, dDir = 0, dDuty = 0; bool drumAwake = false;
  // ---- motor model
  double n0rpm = 45, stallNm = 15, stallA = 18, freeA = 1.5, vbat0 = 12.4, rint = 0.05;
  double amps = 0, wRateDps = 0;
  double drumDpsTrue = 10.0;       // a little slower than the firmware's 10.8 estimate
  double aimLoadNm = 1.0, cockMaxNm = 6.0, reelFreeNm = 0.6;
  double capstanR = 0.015;         // m
  // ---- sensors
  int encSign = +1; bool encMissing = false; int encGlitchEvery = 0; uint32_t encReads = 0; bool magnetWeak = false;
  double isAmpsPerVolt = 12.5;     // must match the firmware's WiperCfg
  bool hxFitted = false; double hxCountsPerN = 21000; double hxOffset = 1234;
  bool vbatWired = true; double vbatOverride = -1;
  // ---- fault injection
  uint8_t swStuckClosed = 0, swStuckOpen = 0;   // physical bits
  uint8_t swChatter = 0; uint32_t chatterSeed = 1;
  bool drumJam = false;            // drum N20 does not move
  double drumJamAbove = 1e9;       // drum cannot pass above this angle (jam)
  bool toothOnTooth = false; double toothNeedWiper = 10; double toothWiperAcc = 0;   // LAUNCH dog blocked until the wiper turns
  double extraLoadNm = 0;          // stall injection
  bool searBroken = false;         // the relieved tooth never lets go (no release)
  // ---- log
  std::vector<std::string> out;
  gb::Saved nvs; bool nvsHas = false; int saves = 0; uint32_t saveSeq = 0, saveT = 0;
  uint32_t t = 0;

  // collar dwells (drum_NOTES.md section 6): LAUNCH engaged -10..6, REEL engaged 74..346; a tip to tip landing
  // blocks the train until the wiper turned toothNeedWiper
  bool inLaunch() const { return drumDeg <= 6 && !dogTip; }
  bool inReel() const { return drumDeg >= 74 && !dogTip; }
  double wrapsExact() const { if (drumDeg <= c.wrapFromDeg) return 0; if (drumDeg >= c.wrapToDeg) return c.wrapsFull; return c.wrapsFull * (drumDeg - c.wrapFromDeg) / (c.wrapToDeg - c.wrapFromDeg); }
  int wrapsTrue() const { return (int)floor(wrapsExact() + 0.15); }
  // each code edge switches a little off its nominal angle (printed bumps): fixed offsets up to +-edgeJitter deg
  double edgeJitter = 1.5; double edgeOff[8] = {0.9, -1.2, 0.4, -0.7, 1.4, -0.3, 0.6, -1.0};
  int sectorTrue() const { int k = 0; while (k < gb::SEC_COUNT - 1 && drumDeg >= c.edgeDeg[k] + edgeOff[k] * edgeJitter / 1.5) k++; return k; }
  int dryFires = 0;               // the collar left the LAUNCH dwell with the spring drawn (must never happen)
  bool dogTip = false;
  double cockLoad() const {        // the rack is engaged from homeOpenDeg to the release tooth
    if (fired || cockPh < c.homeOpenDeg) return 0.5;
    double f = (cockPh - c.homeOpenDeg) / (c.releaseDeg - c.homeOpenDeg); if (f > 1) f = 1;
    return 0.5 + cockMaxNm * (0.3 + 0.7 * f);
  }
  double tension() const {
    if (anchorTautAt < 0) return lineOut > seatAtLine + 0.02 ? 1.0 : 0;   // slack retrieve: light take-up
    double reeled = anchorTautAt - lineOut; return reeled > 0 ? reeled * anchorK : 0.5;
  }
  bool seated() const { return anchorTautAt < 0 && lineOut <= seatAtLine + 0.01; }

  uint8_t contactsRaw() {
    uint8_t b = 0;
    auto setb = [&](int id, bool v) { int bit = c.swBit[id]; if (bit >= 0 && v) b |= (1 << bit); };
    int code = c.secCode[sectorTrue()];
    setb(gb::SW_DRUM_A, code & 1); setb(gb::SW_DRUM_B, code & 2); setb(gb::SW_DRUM_C, code & 4);
    setb(gb::SW_DOG, dogTip);
    double wf = wrapsExact() - floor(wrapsExact());
    setb(gb::SW_WRAP_HOME, wf < 0.03 || wf > 0.97);
    setb(gb::SW_SEATED, seated());
    setb(gb::SW_HOME, fired || cockPh < c.homeOpenDeg);
    double am = fmod(aimPh - c.aimHomeDeg, c.aimCycleDeg); if (am < 0) am += c.aimCycleDeg;
    setb(gb::SW_RAIL_LOW, am < 8);
    b |= swStuckClosed; b &= ~swStuckOpen;
    if (swChatter) { chatterSeed = chatterSeed * 1103515245u + 12345u; if ((chatterSeed >> 16) & 1) b ^= swChatter; }
    return b;
  }

  // advance the truth by dt ms
  void step(uint32_t dt) {
    double s = dt / 1000.0;
    // ---- wiper
    double load = 0.2 + extraLoadNm;
    if (wDir != 0) {
      if (inLaunch()) load += (wDir == c.aimDir) ? aimLoadNm : cockLoad();
      else if (inReel() && wDir != c.reelDir) load += 1000;                // sprag #2 backstop: cannot turn out
      else if (inReel() && wDir == c.reelDir && wrapsTrue() > 0) load += reelFreeNm + tension() * capstanR / c.capstanPerWiper;
    }
    double v = vbat();
    double frac = wDuty / 255.0 * v / 12.6;
    // DC motor: torque = kt (I - I0) whatever the duty; speed = n0 (frac - load / stallNm); at stall I = stallA x frac
    double rpm = 0;
    if (wDir != 0 && wDuty > 0) {
      rpm = n0rpm * (frac - load / stallNm);
      if (rpm < 0) rpm = 0;
      amps = freeA + (stallA - freeA) * load / stallNm;
      double cap = fmax(stallA * frac, freeA);
      if (amps > cap || rpm == 0) amps = rpm == 0 ? cap : amps;
    } else amps = 0;
    double dDeg = wDir * rpm * 6.0 * s;
    wRateDps = wDir * rpm * 6.0;
    wiperDeg += dDeg;
    double ad = fabs(dDeg);
    if (ad > 0) {
      if (inLaunch()) {
        if (wDir == c.aimDir) {
          aimPh = fmod(aimPh + ad, c.aimCycleDeg);
          if (!fired && cockPh > c.homeOpenDeg - 1) cockPh = fmax(c.homeOpenDeg - 1, cockPh - ad);   // drawn spring follows back (decock)
        } else {
          double before = cockPh; cockPh += ad;
          if (!fired && !searBroken && before < c.releaseDeg && cockPh >= c.releaseDeg) { fired = true; shots++; lineOut = lineOutAfterShot; }
          if (fired && cockPh >= c.cycleDeg - 5) { cockPh -= c.cycleDeg; fired = false; }
        }
      } else if (dogTip) { toothWiperAcc += ad; if (toothWiperAcc >= toothNeedWiper) { dogTip = false; toothWiperAcc = 0; } }
      else if (inReel() && wDir == c.reelDir && wrapsTrue() > 0) { lineOut -= ad / 360.0 * c.capstanPerWiper * 2 * M_PI * capstanR; if (lineOut < 0) lineOut = 0; }
    }
    // ---- drum
    if (dDir != 0 && drumAwake && !drumJam) {
      double nd = drumDeg + dDir * c.drumSign * drumDpsTrue * (dDuty / 255.0) * s;
      if (nd > drumJamAbove) nd = drumJamAbove;
      if (nd < c.secLoDeg) nd = c.secLoDeg; if (nd > c.secHiDeg) nd = c.secHiDeg;   // groove ends
      // the collar lands tip to tip on the way into a dwell (when toothOnTooth is set): once per entry
      if (toothOnTooth && ((drumDeg > 6 && nd <= 6) || (drumDeg < 74 && nd >= 74))) { dogTip = true; toothWiperAcc = 0; }
      if (nd > 6 && nd < 74) dogTip = false;
      if (drumDeg <= 6 && nd > 6 && !fired && cockPh > c.homeOpenDeg + 1 && cockPh < c.releaseDeg) { dryFires++; cockPh = 0; }
      drumDeg = nd;
    }
    t += dt;
  }
  double lineOutAfterShot = 8.0;
  double vbat() const { if (vbatOverride >= 0) return vbatOverride; return vbat0 - rint * amps; }
};

extern Sim S;

// Io callbacks into the global sim
inline void simWiper(int dir, int duty) { S.wDir = dir; S.wDuty = duty; }
inline void simDrum(int dir, int duty) { S.dDir = dir; S.dDuty = duty; }
inline void simDrumSleep(bool a) { S.drumAwake = a; }
inline int simAngle(uint8_t* st) {
  S.encReads++;
  if (S.encMissing) { *st = 0; return -1; }
  if (S.encGlitchEvery && S.encReads % S.encGlitchEvery == 0) { *st = gb::AS_MD; return (int)((S.encReads * 977) % 4096); }
  double a = fmod(S.wiperDeg * S.encSign, 360.0); if (a < 0) a += 360;
  *st = gb::AS_MD | (S.magnetWeak ? gb::AS_ML : 0);
  return ((int)lround(a * 4096.0 / 360.0)) & 4095;
}
inline float simIs() {
  if (!S.wDir || S.wDuty == 0) return 0.002f;
  return (float)(S.amps / S.isAmpsPerVolt * (S.wDuty / 255.0));   // IS reports only during the on time
}
inline float simVbat() { return S.vbatWired ? (float)S.vbat() : NAN; }
inline uint8_t simSw() { return S.contactsRaw(); }
inline float simHx() { return S.hxFitted ? (float)(S.hxOffset + S.tension() * S.hxCountsPerN) : NAN; }
inline void simSave(const gb::Saved& s) { S.nvs = s; S.nvsHas = true; S.saves++; S.saveSeq = s.seq; S.saveT = S.t; }
inline void simOut(const char* l) { S.out.push_back(l); if (getenv("SIMLOG")) printf("      [%6u] %s\n", (unsigned)S.t, l); }
inline uint32_t simRand() { static uint32_t x = 12345; x = x * 1664525u + 1013904223u; return x; }

inline gb::Io simIo() {
  gb::Io io; io.wiper = simWiper; io.drum = simDrum; io.drumSleep = simDrumSleep; io.angle = simAngle; io.wiperIsVolts = simIs;
  io.vbat = simVbat; io.switches = simSw; io.hxRaw = simHx; io.save = simSave; io.out = simOut; io.rand32 = simRand; return io;
}
