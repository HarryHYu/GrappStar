// Motor drive logic. Plain C++ (no Arduino calls): the sketch passes the hardware in through the callbacks.
//  WiperDrive: BTS7960 on the 3S. Soft start ramp (wiper inrush blew a 10 A fuse), a stopped gap before any
//    direction change, current from the IS pins scaled by the duty, over-current trip and stall detection.
//  DrumDrive: DRV8833 channel A for the selector drum N20 (worm, self-locking). Short ramp, brake on stop,
//    sleep when idle.
#pragma once
#include <stdint.h>
#include <math.h>

namespace gb {

struct WiperCfg {
  uint32_t rampMs = 300;          // 0 -> 255 duty
  uint32_t reverseGapMs = 200;    // stopped this long before the direction may change
  float ampsPerVolt = 12.5f;      // BTS7960 IS: 8500 x 1680 / (1680 x 680) with no onboard IS resistor. CALIBRATION.md
  float offsetA = 0;              // learned with "cal current" (motor stopped)
  int minScaleDuty = 60;          // below this duty the IS reading is too short a pulse to scale reliably
  float tripA = 18; int tripSamples = 2;      // over-current: this many ticks in a row over tripA
  float stallDutyFrac = 0.30f;    // stall check only while driving above this duty
  float stallDps = 15;            // shaft slower than this (deg/s) ...
  uint32_t stallMs = 400;         // ... for this long (after the ramp) = stall
};

struct WiperDrive {
  WiperCfg c;
  int wantDir = 0, wantDuty = 0;  // what the logic asks for
  int outDir = 0, lastDir = 0;    // what the bridge gets
  float outDuty = 0;
  uint32_t stopT = 0, startT = 0, slowT = 0;
  bool slow = false;
  float ampsRaw = 0, amps = 0, peakA = 0; int tripN = 0;
  bool overcurrent = false, stalled = false;     // latched until the next start

  void want(int dir, int duty) { wantDir = dir; wantDuty = dir ? (duty < 0 ? 0 : duty > 255 ? 255 : duty) : 0; }
  bool running() const { return outDir != 0; }
  // stop at once: both PWM low = both low-side FETs on = brake; the worm then self-locks
  void halt(uint32_t now, void (*hw)(int, int)) {
    wantDir = 0; wantDuty = 0;
    if (outDir) stopT = now;
    outDir = 0; outDuty = 0;
    if (hw) hw(0, 0);
  }
  // isVolts: IS pin volts. rateDps: shaft rate from the AS5600 (NAN if unknown).
  // allowStart false: a stopped motor stays stopped this tick (a running one carries on)
  void tick(uint32_t now, uint32_t dt, float isVolts, float rateDps, void (*hw)(int, int), bool allowStart = true) {
    // current: the IS pin only reports while the high side conducts, so its average is I x duty. Scale back.
    float a = isVolts * c.ampsPerVolt - c.offsetA; if (a < 0) a = 0;
    if (outDir && outDuty >= c.minScaleDuty) a *= 255.f / outDuty;
    ampsRaw = a;
    amps += (a - amps) * 0.3f;
    if (outDir) {
      if (amps > peakA) peakA = amps;
      if (ampsRaw > c.tripA) { if (++tripN >= c.tripSamples) overcurrent = true; } else tripN = 0;
    } else tripN = 0;

    if (outDir && (wantDir != outDir || wantDuty == 0)) {   // stop first; a new direction waits for the gap
      int wd = wantDir, wu = wantDuty; halt(now, hw); wantDir = wd; wantDuty = wu;
    }
    if (!outDir && wantDir && wantDuty > 0 && !overcurrent && !stalled && allowStart &&
        (lastDir == 0 || wantDir == lastDir || now - stopT >= c.reverseGapMs)) {
      outDir = lastDir = wantDir; outDuty = 0; startT = now; slow = false; peakA = 0;
    }
    if (outDir) {
      if (wantDuty > outDuty) { outDuty += 255.f * dt / c.rampMs; if (outDuty > wantDuty) outDuty = (float)wantDuty; }
      else outDuty = (float)wantDuty;
      // stall: well into the run, driving hard, shaft not turning
      bool hard = outDuty >= c.stallDutyFrac * 255.f && now - startT >= c.rampMs;
      bool still = !isnan(rateDps) && fabsf(rateDps) < c.stallDps;
      if (hard && still) { if (!slow) { slow = true; slowT = now; } else if (now - slowT >= c.stallMs) stalled = true; }
      else slow = false;
    }
    if (overcurrent || stalled) { if (outDir) { stopT = now; } outDir = 0; outDuty = 0; wantDir = 0; wantDuty = 0; }
    if (hw) hw(outDir, (int)outDuty);
  }
  void clearLatches() { overcurrent = false; stalled = false; tripN = 0; slow = false; }
};

struct DrumCfg {
  int duty = 255;                 // full speed (VM about 9 V from the buck)
  uint32_t rampMs = 60;
  uint32_t reverseGapMs = 80;
  uint32_t sleepAfterMs = 500;    // nSLEEP low this long after the last stop (worm holds the drum unpowered)
};

struct DrumDrive {
  DrumCfg c;
  int wantDir = 0, outDir = 0, lastDir = 0; float outDuty = 0; uint32_t stopT = 0, startT = 0; bool awake = false;

  void want(int dir) { wantDir = dir > 0 ? 1 : dir < 0 ? -1 : 0; }
  bool running() const { return outDir != 0; }
  void halt(uint32_t now, void (*hw)(int, int)) {
    wantDir = 0; if (outDir) stopT = now; outDir = 0; outDuty = 0; if (hw) hw(0, 0);
  }
  void tick(uint32_t now, uint32_t dt, void (*hw)(int, int), void (*sleepHw)(bool), bool allowStart = true) {
    if (outDir && wantDir != outDir) { int w = wantDir; halt(now, hw); wantDir = w; }
    if (!outDir && wantDir && allowStart && (lastDir == 0 || wantDir == lastDir || now - stopT >= c.reverseGapMs)) {
      if (!awake) { awake = true; if (sleepHw) sleepHw(true); }
      outDir = lastDir = wantDir; outDuty = 0; startT = now;
    }
    if (outDir) { outDuty += 255.f * dt / c.rampMs; if (outDuty > c.duty) outDuty = (float)c.duty; }
    if (!outDir && awake && now - stopT >= c.sleepAfterMs) { awake = false; if (sleepHw) sleepHw(false); }
    if (hw) hw(outDir, (int)outDuty);
  }
};

}  // namespace gb
