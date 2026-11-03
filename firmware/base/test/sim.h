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
  double lineOut = 0.0;            // m out (0 = dart seated in the launcher)
  double anchorTautAt = -1;        // tow: line length at which the anchored line goes taut (-1 = slack retrieve)
  double anchorK = 40;             // N per m of capstan travel after taut
  double seatAtLine = 0.0;         // retrieve: dart seats when lineOut reaches this
  // ---- drive inputs
  int wDir = 0, wDuty = 0, dDir = 0, dDuty = 0; bool drumAwake = false;
  // ---- motor model
  double n0rpm = 45, stallNm = 15, stallA = 18, freeA = 1.5, vbat0 = 12.4, rint = 0.05;
  double amps = 0, wRateDps = 0;
  double inertiaMs = 0;            // wiper speed lag and coast time constant (0 = instant, the old model)
  double vel = 0;                  // signed wiper speed, deg/s (with inertia)
  double drumDpsTrue = 7.5;        // a little slower than the firmware 8.1 estimate
  double aimLoadNm = 0.5, cockMaxNm = 2.5, reelFreeNm = 0.4;
  double capstanR = 0.0205;        // m (capstan d 40 + line)
  // ---- sensors
  int encSign = +1; bool encMissing = false; int encGlitchEvery = 0; int encDropEvery = 0, encDropBurst = 3; uint32_t encReads = 0; bool magnetWeak = false;
  double isAmpsPerVolt = 12.5;     // must match the firmware's WiperCfg
  bool hxFitted = false; double hxCountsPerN = 21000; double hxOffset = 1234;
  bool vbatWired = true; double vbatOverride = -1;
  // ---- fault injection
  uint16_t swStuckClosed = 0, swStuckOpen = 0;   // physical bits
  uint16_t swChatter = 0; uint32_t chatterSeed = 1;
  bool drumJam = false;            // drum N20 does not move
  double drumJamAbove = 1e9;       // drum cannot pass above this angle (jam)
  bool toothOnTooth = false; double toothNeedWiper = 10; double toothWiperAcc = 0;   // LAUNCH dog blocked until the wiper turns
  double aimDragDeg = 0;           // backward aim drags the big gear back up to this far, wiper deg (cad-launch review: up to
                                   // 127 big-gear deg = 195.4 wiper deg)
  double extraLoadNm = 0;          // stall injection
  bool searBroken = false;         // the relieved tooth never lets go (no release)
  // ---- contacts as cad-sensors and cad-launch build them, independent of the firmware's own constants (so a firmware
  // default that does not match the CAD shows up here): SW_GAP is closed while the big gear is within +-14.9 big-gear
  // deg of rest (+-22.9 wiper deg: mesh check of the bump against the contact pretravel, sensors_NOTES about +-15);
  // SW_RAIL_LOW within +-9.76 crank deg of the LOW dead point, aim phase 0 (+-97.6 wiper deg, sensors_NOTES about +-9);
  // SW_SEATED only with the carriage on its front stop (launch.py seat_geom: the plunger presses the rail-lip lever at
  // HOME) and the dart seated
  double gapWinDeg = 14.9 * 40.0 / 26.0, lowWinDeg = 97.6;
  // ---- log
  std::vector<std::string> out;
  gb::Saved nvs; bool nvsHas = false; int saves = 0; uint32_t saveSeq = 0, saveT = 0;
  uint32_t t = 0;

  // collar (lens B table 4.1): engaged in LAUNCH below lnEngageDeg, in REEL above nrEngageDeg; a tip to tip landing
  // blocks the train until the wiper turned toothNeedWiper. The dogs have a free window (dogWindowDeg).
  bool inLaunch() const { return drumDeg <= c.lnEngageDeg && !dogTip; }
  bool inReel() const { return drumDeg >= c.nrEngageDeg && !dogTip; }
  double win = 30;                 // collar position in its dog window (0 = backward flank)
  double slack = 0, maxSlack = 0;  // line slack between the reel and the dart side (unwrap without take-up makes it)
  double wrapsExact() const { if (drumDeg <= c.wrapFromDeg) return 0; if (drumDeg >= c.wrapToDeg) return c.wrapsFull; return c.wrapsFull * (drumDeg - c.wrapFromDeg) / (c.wrapToDeg - c.wrapFromDeg); }
  int wrapsTrue() const { return (int)floor(wrapsExact() + 0.15); }
  // each code edge switches a little off its nominal angle (printed bumps): fixed offsets up to +-edgeJitter deg
  double edgeJitter = 1.5; double edgeOff[8] = {0.9, -1.2, 0.4, -0.7, 1.4, -0.3, 0.6, -1.0};
  int sectorTrue() const { int k = 0; while (k < gb::SEC_COUNT - 1 && drumDeg >= c.edgeDeg[k] + edgeOff[k] * edgeJitter / 1.5) k++; return k; }
  int dryFires = 0;               // the collar left the LAUNCH dwell with the spring drawn (must never happen)
  bool dogTip = false; bool handLatched = false;
  double cockLoad() const {        // the rack is engaged from homeOpenDeg to the release tooth
    if (fired || cockPh < c.homeOpenDeg) return 0.5;
    double f = (cockPh - c.homeOpenDeg) / (c.releaseDeg - c.homeOpenDeg); if (f > 1) f = 1;
    return 0.5 + cockMaxNm * (0.3 + 0.7 * f);
  }
  double tension() const {
    if (anchorTautAt < 0) return lineOut > seatAtLine + 0.02 ? 1.0 : 0;   // slack retrieve: light take-up
    double reeled = anchorTautAt - lineOut; return reeled > 0 ? reeled * anchorK : 0.5;
  }
  bool seated() const { return anchorTautAt < 0 && lineOut <= seatAtLine + 0.01; }   // the dart itself (truth)
  bool carriageHome() const { return fired || cockPh < c.homeOpenDeg; }

  uint16_t contactsRaw() {
    uint16_t b = 0;
    auto setb = [&](int id, bool v) { int bit = c.swBit[id]; if (bit >= 0 && v) b |= (1 << bit); };
    int code = c.secCode[sectorTrue()];
    setb(gb::SW_DRUM_A, code & 1); setb(gb::SW_DRUM_B, code & 2); setb(gb::SW_DRUM_C, code & 4);
    setb(gb::SW_DOG_L, inLaunch()); setb(gb::SW_DOG_R, inReel());
    double wf = wrapsExact() - floor(wrapsExact());
    setb(gb::SW_ARM, c.armHomeOpen ? !(wf < 0.014 || wf > 0.986) : (wf < 0.03 || wf > 0.97));   // cad-reel: opens +-5 deg at home
    { double m = fmod(wiperDeg, 60.0); if (m < 0) m += 60; setb(gb::SW_INDEX, m < 10); }   // 6-bump index cam
    setb(gb::SW_N20, !handLatched);
    double g = fmod(cockPh, c.cycleDeg); if (g < 0) g += c.cycleDeg;
    setb(gb::SW_GAP, !fired ? (g >= c.cycleDeg - gapWinDeg || g < gapWinDeg) : (cockPh >= c.cycleDeg - gapWinDeg));
    setb(gb::SW_SEATED, seated() && carriageHome());
    setb(gb::SW_HOME, carriageHome());
    double am = fmod(aimPh, c.aimCycleDeg); if (am < 0) am += c.aimCycleDeg;
    setb(gb::SW_RAIL_LOW, am < lowWinDeg || am >= c.aimCycleDeg - lowWinDeg);
    b |= swStuckClosed; b &= ~swStuckOpen;
    if (swChatter) { chatterSeed = chatterSeed * 1103515245u + 12345u; if ((chatterSeed >> 16) & 1) b ^= swChatter; }
    return b;
  }

  // advance the truth by dt ms
  void step(uint32_t dt) {
    double s = dt / 1000.0;
    bool engaged = inLaunch() || inReel();
    // ---- wiper: the load only acts once the collar is against a flank
    bool atFlank = engaged && ((wDir > 0 && win >= c.dogWindowDeg - 1e-6) || (wDir < 0 && win <= 1e-6));
    double load = 0.2 + extraLoadNm;
    if (wDir > 0) load += 0.1;                                       // take-up bus (forward in every mode)
    if (wDir != 0 && atFlank) {
      if (inLaunch()) load += (wDir == c.aimDir) ? aimLoadNm : cockLoad();
      else if (inReel() && wDir != c.reelDir) load += 1000;          // sprag #2 backstop: cannot turn out
      else if (inReel() && wDir == c.reelDir) load += reelFreeNm + tension() * capstanR * c.capstanPerWiper;
    }
    double v = vbat();
    double frac = wDuty / 255.0 * v / 12.6;
    double rpm = 0;
    if (wDir != 0 && wDuty > 0) {
      rpm = n0rpm * (frac - load / stallNm); if (rpm < 0) rpm = 0;
      amps = freeA + (stallA - freeA) * load / stallNm;
      double cap = fmax(stallA * frac, freeA);
      if (amps > cap || rpm == 0) amps = rpm == 0 ? cap : amps;
    } else amps = 0;
    double tgt = wDir * rpm * 6.0;
    if (inertiaMs > 0) {                              // first-order lag; a load at the flank stops a coast faster
      double k = fmin(1.0, dt / inertiaMs); vel += (tgt - vel) * k;
      if (wDir == 0 && engaged && ((vel > 0 && win >= c.dogWindowDeg - 1e-6) || (vel < 0 && win <= 1e-6)) && (inLaunch() || inReel())) vel *= 0.5;
      if (inReel() && vel * c.reelDir < 0 && win <= 1e-6) vel = 0;   // sprag #2
      if (fabs(vel) < 0.5 && wDir == 0) vel = 0;
    } else vel = tgt;
    double dDeg = vel * s;
    wRateDps = vel;
    int mDir = dDeg > 0 ? 1 : dDeg < 0 ? -1 : 0;     // the way the shaft actually turns (coast included)
    wiperDeg += dDeg;
    double ad = fabs(dDeg);
    if (ad > 0) {
      if (mDir > 0) { double t = ad / 360.0 * 0.39; slack = fmax(0, slack - t); }   // take-up: 2.5 x a 50 mm core per turn   // take-up winds slack in (clutch slips beyond)
      if (dogTip) { toothWiperAcc += ad; if (toothWiperAcc >= toothNeedWiper) { dogTip = false; toothWiperAcc = 0; } }
      else if (engaged) {
        double drive = 0;                                            // travel beyond the window
        if (mDir > 0) { double room = fmax(0.0, c.dogWindowDeg - win); double u = fmin(room, ad); win += u; drive = ad - u; }
        else { double room = fmax(0.0, win); double u = fmin(room, ad); win -= u; drive = ad - u; }
        if (drive > 0 && inLaunch()) {
          if (mDir == c.aimDir) {
            aimPh = fmod(aimPh + drive, c.aimCycleDeg);
            if (!fired && cockPh > c.homeOpenDeg - 1) cockPh = fmax(c.homeOpenDeg - 1, cockPh - drive);   // drawn spring follows back (decock)
            else if (!fired && aimDragDeg > 0) cockPh = fmax(-aimDragDeg, cockPh - drive);   // cad-launch: the big gear follows the sprag drag back (gap opens)
          } else {
            double before = cockPh; cockPh += drive;
            if (!fired && !searBroken && before < c.releaseDeg && cockPh >= c.releaseDeg) { fired = true; shots++; lineOut = lineOutAfterShot; }
            if (fired && cockPh >= c.cycleDeg - 5) { cockPh -= c.cycleDeg; fired = false; }
          }
        } else if (drive > 0 && inReel() && mDir == c.reelDir) {
          lineOut -= drive / 360.0 * c.capstanPerWiper * 2 * M_PI * capstanR; if (lineOut < 0) lineOut = 0;
        }
      } else if (mDir > 0 && anchorTautAt < 0 && lineOut > seatAtLine) {   // NEUTRAL: take-up pulls a free dart home
        lineOut = fmax(seatAtLine, lineOut - ad / 360.0 * 0.39);
      }
    }
    // ---- drum
    if (dDir != 0 && drumAwake && !drumJam) {
      double nd = drumDeg + dDir * c.drumSign * drumDpsTrue * (dDuty / 255.0) * s;
      if (nd > drumJamAbove && nd > drumDeg) nd = fmax(drumDeg, fmin(nd, drumJamAbove));   // a jam blocks forward motion only
      if (nd < c.secLoDeg) nd = c.secLoDeg; if (nd > c.secHiDeg) nd = c.secHiDeg;   // groove ends
      if (toothOnTooth && ((drumDeg > c.lnEngageDeg && nd <= c.lnEngageDeg) || (drumDeg < c.nrEngageDeg && nd >= c.nrEngageDeg))) { dogTip = true; toothWiperAcc = 0; }
      if (nd > c.lnEngageDeg && nd < c.nrEngageDeg) dogTip = false;
      if (drumDeg <= c.lnEngageDeg && nd > c.lnEngageDeg && !fired && cockPh > c.homeOpenDeg && cockPh < c.releaseDeg) { dryFires++; cockPh = 0; }
      // unwrapping frees one capstan turn of line per wrap: slack unless the take-up winds it in
      double w0 = wrapsExact(); double d0 = drumDeg; drumDeg = nd; double w1 = wrapsExact(); drumDeg = d0;
      if (w1 < w0) { slack += (w0 - w1) * M_PI * capstanR * 2; maxSlack = fmax(maxSlack, slack); }
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
  if (S.encDropEvery && (S.encReads % S.encDropEvery) < (uint32_t)S.encDropBurst) return -1;   // I2C read failed (burst)
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
inline uint16_t simSw() { return S.contactsRaw(); }
inline float simHx() { return S.hxFitted ? (float)(S.hxOffset + S.tension() * S.hxCountsPerN) : NAN; }
inline void simSave(const gb::Saved& s) { S.nvs = s; S.nvsHas = true; S.saves++; S.saveSeq = s.seq; S.saveT = S.t; }
inline void simOut(const char* l) { S.out.push_back(l); if (getenv("SIMLOG")) printf("      [%6u] %s\n", (unsigned)S.t, l); }
inline uint32_t simRand() { static uint32_t x = 12345; x = x * 1664525u + 1013904223u; return x; }

inline gb::Io simIo() {
  gb::Io io; io.wiper = simWiper; io.drum = simDrum; io.drumSleep = simDrumSleep; io.angle = simAngle; io.wiperIsVolts = simIs;
  io.vbat = simVbat; io.switches = simSw; io.hxRaw = simHx; io.save = simSave; io.out = simOut; io.rand32 = simRand; return io;
}
