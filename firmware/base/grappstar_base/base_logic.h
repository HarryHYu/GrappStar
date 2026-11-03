// GrappStar base station logic: one wiper motor (BTS7960 + AS5600) and the N20 selector drum (DRV8833).
// Plain C++ (no Arduino calls): the sketch supplies the hardware through Io, the PC tests supply a simulation.
// Protocol: PROTOCOL.md (contract C1, JSON lines over USB CDC). Mechanism: base/spec/BASE_SPEC.md.
//
// Drum travel (one line, params.DRUM): LAUNCH -- L_TO_N -- NEUTRAL -- WRAP_PREP -- WRAP (the cup lays 7 turns) --
// N_TO_R (WRAPPED park 314, collar still NEUTRAL) -- REEL. The order is physical: the capstan is wrapped with the collar
// in NEUTRAL, REEL is only reached with the wraps on, and LAUNCH only with them off.
// LAUNCH: wiper aimDir turns the aim train (pitch from the aim table), the other direction cocks, fires and resets.
// REEL (and WRAPPED): wiper reelDir winds the capstan. NEUTRAL: the wiper turns nothing (the safe state).
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "config.h"
#include "json_lite.h"
#include "sensors.h"
#include "drives.h"
#include "persist.h"

namespace gb {

#define GB_FW_VERSION "base-1.2.1"

enum Fault { F_NONE, F_STALL, F_OVERCURRENT, F_DRUM_TIMEOUT, F_DRUM_SENSOR, F_ANGLE_SENSOR, F_SWITCH_DISAGREE, F_BROWNOUT, F_BAD_STATE, F_DART_STUCK };
static const char* const FAULT_NAME[] = {nullptr, "STALL", "OVERCURRENT", "DRUM_TIMEOUT", "DRUM_SENSOR", "ANGLE_SENSOR",
                                         "SWITCH_DISAGREE", "BROWNOUT", "BAD_STATE", "DART_STUCK"};

enum State { S_IDLE, S_DRUM, S_AIM, S_COCK, S_FIRE, S_WRAP, S_UNWRAP, S_REEL, S_HOLD, S_JOG, S_CALDRUM, S_SAFING, S_FAULT, S_DECOCK };
static const char* const STATE_NAME[] = {"IDLE", "MODE", "AIM", "COCK", "FIRE", "WRAP", "UNWRAP", "REEL", "HOLD", "JOG",
                                         "CAL", "SAFING", "FAULT", "DECOCK"};

enum Phase { PH_REST, PH_COCKING, PH_COCKED, PH_FIRING, PH_RESETTING, PH_UNKNOWN };
static const char* const PHASE_NAME[] = {"REST", "COCKING", "COCKED", "FIRING", "RESETTING", "UNKNOWN"};

enum Until { U_STOP, U_SEATED, U_TENSION, U_SLIP, U_TIME };

struct Io {
  void (*wiper)(int dir, int duty) = nullptr;     // BTS7960: dir +1 RPWM, -1 LPWM, 0 both low (brake); duty 0..255
  void (*drum)(int dir, int duty) = nullptr;      // DRV8833 channel A: dir +1 / -1, 0 = brake (both inputs high)
  void (*drumSleep)(bool awake) = nullptr;        // nSLEEP
  int (*angle)(uint8_t* status) = nullptr;        // AS5600 raw 0..4095 or -1 on an I2C error; *status = STATUS register
  float (*wiperIsVolts)() = nullptr;              // volts at the IS pin
  float (*vbat)() = nullptr;                      // battery volts, NAN if not wired
  uint16_t (*switches)() = nullptr;               // bit i = contact on PIN_SW_(A+i) closed
  float (*hxRaw)() = nullptr;                     // HX711 counts, NAN if absent or no new sample
  void (*save)(const Saved& s) = nullptr;         // persist (the sketch writes NVS outside the tick)
  void (*saveTun)(const Tun& t) = nullptr;        // bench settings (set / cal)
  void (*out)(const char* line) = nullptr;        // event line to the host (queued by the sketch)
  uint32_t (*rand32)() = nullptr;                 // token source (esp_random on the ESP32)
};

struct Base {
  Io io; Cfg c;
  WiperDrive w; DrumDrive d; Switches sw; AngleTracker enc;
  // aim table (pitch against aim-train wiper deg)
  float aimW[AIM_TAB_MAX], aimP[AIM_TAB_MAX]; int aimN = 0;

  // ---- live state
  int state = S_IDLE, fault = F_NONE; char faultWhy[48] = "";
  uint32_t now = 0, stT = 0, bootT = 0; bool booted = false, bootPending = true; Saved boot; bool bootValid = false;
  // wiper bookkeeping
  float lastWiperDeg = 0; bool wiperInit = false; float lashLeft = 0; int lashDir = 0;
  float aimPhase = 0; bool aimKnown = false; float aimOffset = 0; bool aimed = false; float aimTarget = NAN;
  float cockPhase = 0; int phase = PH_UNKNOWN;
  float lineM = 0; bool lineKnown = true;
  float amps = 0, tensionN = 0, tCur = 0, hxOff = 0, hxN = NAN; uint32_t hxT = 0; bool hxSeen = false;
  float vbat = NAN; uint32_t lowT = 0; bool lowRun = false;
  // drum bookkeeping
  int sec = SEC_INVALID, prevSec = SEC_INVALID; uint32_t secBadT = 0; bool secBad = false; uint32_t edgeT = 0; float edgeAt = 0; bool edgeRun = false;
  float drumDeg = 0; bool drumKnown = false; int wraps = 0; bool wrapsKnown = true; bool wrapsDoubt = false;
  // current operation
  uint32_t opT = 0, legT = 0; float opFrom = 0, opTravel = 0; int sub = 0;
  int jogMotor = 0, jogDir = 0; uint32_t jogUntil = 0; bool jogLaunch = false;   // jogLaunch: a wiper jog that started in LAUNCH
  int until = U_STOP; float maxN = 20; int reelDutyCmd = 220; bool holdLimit = false;
  float phaseHint = -1; bool decockByTravel = false; float decockTravel = 0; uint32_t homeT = 0;
  bool homeSeen = false, releaseSeen = false; float relPeakA = 0; int aimTries = 0; bool homing = false;
  // arm token
  uint32_t token = 0, tokenT = 0; bool tokenLive = false, tokenExpired = false;
  // persistence
  // saveAck (set by the sketch): a motor may only START once the "moving" record is in flash, because an NVS
  // write stalls both cores (and this tick) for tens of ms; better before the motor runs than during its ramp
  bool saveAck = false; volatile uint32_t saveDoneSeq = 0;
  bool startOk() const { return !saveAck || !savedOnce || saveDoneSeq == last.seq; }
  Saved last; bool savedOnce = false; uint8_t movingBits = 0; uint32_t quietT = 0; bool dirty = false; uint32_t seq = 0;
  // telemetry pacing
  uint32_t telT = 0;
  char evBuf[257];                // events at most 256 bytes

  // ================================================================== setup
  void begin(uint32_t t) {
    now = bootT = stT = t; telT = t;
    enc.sign = c.wiperSign; sw.debounceMs = c.debounceMs;
    w.c.tripA = c.tripA; win = c.dogWindowDeg;   // collar position in its dog window (dogWindowDeg = forward flank)
    loadAimTable();
  }
  void loadAimTable() {
#ifdef P_AIMTAB_N
    aimN = P_AIMTAB_N > AIM_TAB_MAX ? AIM_TAB_MAX : P_AIMTAB_N;
    for (int i = 0; i < aimN; i++) { aimW[i] = P_AIMTAB_WIPER[i]; aimP[i] = P_AIMTAB_PITCH[i]; }
    c.aimCycleDeg = P_AIMTAB_CYCLE_DEG;
#else
    // launch_NOTES.md section 7: pitch every 100 backward wiper deg from the low dead point over one aim cycle (3600)
    static const float T[37] = {6.01f, 6.24f, 6.93f, 8.12f, 9.83f, 12.10f, 14.93f, 18.32f, 22.22f, 26.50f, 30.98f, 35.39f, 39.42f,
      42.76f, 45.19f, 46.62f, 47.06f, 46.59f, 45.37f, 43.52f, 41.20f, 38.54f, 35.64f, 32.59f, 29.49f, 26.39f, 23.36f, 20.46f,
      17.72f, 15.19f, 12.91f, 10.91f, 9.21f, 7.85f, 6.85f, 6.23f, 6.01f};
    aimN = 37;
    for (int i = 0; i < aimN; i++) { aimW[i] = c.aimCycleDeg * i / 36.f; aimP[i] = T[i]; }
#endif
  }
  Tun tunNow() const {
    Tun t; t.encSign = enc.sign > 0 ? 1 : 0; t.requireSeated = (c.requireSeated ? 1 : 0) | (c.vbatWired ? 0 : 2);
    t.ampsPerVolt = w.c.ampsPerVolt; t.offsetA = w.c.offsetA; t.freeA = c.freeA; t.kNperA = c.kNperA; t.hxNPerCount = c.hxNPerCount;
    t.hxOff = hxOff; t.limitN = c.limitN; t.drumDps = c.drumDps; t.aimOffset = aimOffset; t.aimTol = c.aimTolDeg; t.tripA = c.tripA;
    t.cockTripA = c.cockTripA; t.vbatLow = c.vbatLow; t.aimHomeDeg = c.aimHomeDeg;
    t.tensOffsetN = c.tensOffsetN; t.towTripA = c.towTripA; t.hostTimeoutS = 0;   // host_timeout_s is never saved (review r3_M: 0 at every boot)
    sealTun(&t); return t;
  }
  // the sketch calls this before begin() with what saveTun() stored (or nullptr)
  void restoreTun(const Tun* t) {
    if (!t || !validTun(t)) return;
    lastTun = *t;
    enc.sign = c.wiperSign = t->encSign ? 1 : -1; c.requireSeated = true;   /* require_seated is never kept off across a boot (review D2) */ c.vbatWired = !(t->requireSeated & 2); w.c.ampsPerVolt = t->ampsPerVolt; w.c.offsetA = t->offsetA;
    c.freeA = t->freeA; c.kNperA = t->kNperA; c.hxNPerCount = t->hxNPerCount; hxOff = t->hxOff; c.limitN = t->limitN; c.drumDps = t->drumDps;
    aimOffset = t->aimOffset; c.aimTolDeg = t->aimTol; c.tripA = t->tripA; c.cockTripA = t->cockTripA; c.vbatLow = t->vbatLow; c.aimHomeDeg = t->aimHomeDeg;
    c.tensOffsetN = t->tensOffsetN; c.towTripA = t->towTripA;
  }
  bool tunDirty = false; Tun lastTun;              // lastTun: what flash holds (restoreTun / last save)
  uint32_t evDrops = 0;                              // set by the sketch: events dropped by the USB ring (status ev_drops)
  // the sketch calls this before the first tick with what save() stored (or nullptr)
  void restore(const Saved* s) { if (s && validSaved(s)) { boot = *s; bootValid = true; } else bootValid = false; }

  // ================================================================== helpers
  bool fitted(int id) const { return c.swBit[id] >= 0; }
  bool closed(int id) const { return fitted(id) && sw.closed(c.swBit[id]); }
  bool rose(int id) const { return fitted(id) && sw.rose(c.swBit[id]); }
  bool fell(int id) const { return fitted(id) && sw.fell(c.swBit[id]); }
  // wiper travel (deg, along dir) since the raw edge of a contact: undoes the debounce delay
  float sinceEdge(int id, int dir) const { return fitted(id) ? (enc.deg() - sw.edgeAux(c.swBit[id])) * dir : 0; }
  bool chat(int id) const { return fitted(id) && sw.chattering(c.swBit[id]); }
  // ---- drum sector from the code bumps (bit 0 = SW_DRUM_A, 1 = SW_DRUM_B, 2 = SW_DRUM_C, debounced)
  uint8_t codeNow() const { return (closed(SW_DRUM_A) ? 1 : 0) | (closed(SW_DRUM_B) ? 2 : 0) | (closed(SW_DRUM_C) ? 4 : 0); }
  int sectorOf(uint8_t cd) const { for (int k = 0; k < SEC_COUNT; k++) if (cd == c.secCode[k]) return k; return SEC_INVALID; }
  float secLo(int s) const { return s <= 0 ? c.secLoDeg : c.edgeDeg[s - 1]; }
  float secHi(int s) const { return s >= SEC_COUNT - 1 ? c.secHiDeg : c.edgeDeg[s]; }
  static int targetSec(int t) { return t == DP_LAUNCH ? SEC_LAUNCH : t == DP_NEUTRAL ? SEC_NEUTRAL : t == DP_REEL ? SEC_REEL : SEC_LOCK; }
  int detentClosed() const {          // DP_LAUNCH / DP_NEUTRAL / DP_REEL when the drum is in that sector, else -1
    return sec == SEC_LAUNCH ? DP_LAUNCH : sec == SEC_NEUTRAL ? DP_NEUTRAL : sec == SEC_REEL ? DP_REEL : -1;
  }
  int drumPos() const {
    switch (sec) {
      case SEC_LAUNCH: return DP_LAUNCH; case SEC_NEUTRAL: return DP_NEUTRAL; case SEC_REEL: return DP_REEL;
      case SEC_LOCK: return DP_WRAPPED; case SEC_INVALID: return DP_UNKNOWN; default: return DP_BETWEEN;
    }
  }
  // collar state: from the drum sector, confirmed by the dog contacts when fitted
  bool collarLaunch() const { return sec == SEC_LAUNCH && !secBad && (!fitted(SW_DOG_L) || closed(SW_DOG_L)) && !closed(SW_DOG_R); }
  bool collarReel() const { return sec == SEC_REEL && !secBad && (!fitted(SW_DOG_R) || closed(SW_DOG_R)) && !closed(SW_DOG_L); }
  // which train the wiper may be driving (for the wiper direction rules): engaged or possibly engaged
  bool maybeLaunch() const { return sec == SEC_LAUNCH || sec == SEC_LN || sec == SEC_INVALID || secBad || closed(SW_DOG_L); }
  bool maybeReel() const { return sec == SEC_REEL || sec == SEC_LOCK || sec == SEC_INVALID || secBad || closed(SW_DOG_R); }
  const char* modeName() const {
    if (collarLaunch()) return "LAUNCH"; if (collarReel()) return "REEL";
    if ((sec == SEC_NEUTRAL || sec == SEC_PREP || sec == SEC_WRAP || (sec == SEC_LOCK && drumDeg < c.collarNdeg)) && !closed(SW_DOG_L) && !closed(SW_DOG_R)) return "NEUTRAL";
    return "NONE";
  }
  int wrapsAt(int s, float deg) const {
    if (s == SEC_LOCK || s == SEC_REEL) return c.wrapsFull;
    if (s != SEC_WRAP) return 0;
    float f = (deg - c.wrapFromDeg) / (c.wrapToDeg - c.wrapFromDeg) * c.wrapsFull;
    int n = (int)floorf(f + 0.15f); return n < 0 ? 0 : n > c.wrapsFull ? c.wrapsFull : n;
  }
  bool moving() const { return w.running() || d.running() || (state != S_IDLE && state != S_HOLD && state != S_FAULT); }
  int cockDir() const { return -c.aimDir; }
  bool powerOk() const { return isnan(vbat) || vbat >= c.vbatPresent; }
  static float wrap360(float a) { a = fmodf(a, 360.f); return a < 0 ? a + 360.f : a; }
  static float wrap180(float a) { a = wrap360(a); return a >= 180.f ? a - 360.f : a; }
  float wrapCyc(float a, float cyc) const { a = fmodf(a, cyc); return a < 0 ? a + cyc : a; }
  // pitch at an aim phase (periodic table, linear interpolation)
  float pitchAt(float ph) const {
    if (aimN < 2) return NAN;
    float x = wrapCyc(ph, c.aimCycleDeg);
    if (x <= aimW[0]) return aimP[0] + aimOffset;
    for (int i = 1; i < aimN; i++) if (x <= aimW[i]) { float f = (x - aimW[i - 1]) / (aimW[i] - aimW[i - 1]); return aimP[i - 1] + (aimP[i] - aimP[i - 1]) * f + aimOffset; }
    return aimP[aimN - 1] + aimOffset;
  }
  float pitch() const { return aimKnown ? pitchAt(aimPhase) : NAN; }
  // smallest forward aim travel (deg of wiper in aimDir) that brings the pitch to target; -1 if never within a cycle
  float aimTravelTo(float target) const {
    float step = 0.25f, best = -1; float prev = pitchAt(aimPhase) - target;
    if (fabsf(prev) <= c.aimTolDeg * 0.25f) return 0;
    for (float t = step; t <= c.aimCycleDeg + step; t += step) {
      float v = pitchAt(aimPhase + t) - target;
      if (v == 0 || (v > 0) != (prev > 0)) { best = t - step * fabsf(v) / (fabsf(v) + fabsf(prev) + 1e-9f); break; }
      prev = v;
    }
    return best;
  }
  float pitchRange(bool hi) const {
    float m = hi ? -1e9f : 1e9f;
    for (int i = 0; i < aimN; i++) { float p = aimP[i] + aimOffset; if (hi ? p > m : p < m) m = p; }
    return m;
  }

  // ================================================================== events and faults
  void emit(const char* line) { if (io.out) io.out(line); }
  void evSimple(const char* name) { char b[96]; snprintf(b, sizeof b, "{\"evt\":\"%s\",\"t\":%lu}", name, (unsigned long)now); emit(b); }
  void evBegin(JOut& o, const char* name) { o.open(); o.str("evt", name); o.integer("t", (long)now); }
  void evEnd(JOut& o) { o.close(); emit(o.bad ? "{\"evt\":\"OVERFLOW\"}" : o.b); }
  void evMode(bool ok) {
    JOut o(evBuf, sizeof evBuf); evBegin(o, "MODE"); o.boolean("ok", ok); o.str("mode", modeName());
    o.str("drum_pos", DRUM_POS_NAME[drumPos()]); evEnd(o);
  }
  void evAborted(const char* what, const char* why) {
    JOut o(evBuf, sizeof evBuf); evBegin(o, "ABORTED"); o.str("what", what); o.str("why", why); evEnd(o);
  }

  void raise(int f, const char* why) {
    w.halt(now, io.wiper);
    if (state == S_SAFING) { d.halt(now, io.drum); state = S_FAULT; stT = now; evHeld(why); dirty = true; return; }   // a second fault stops the safing move
    if (state == S_FAULT) { return; }                 // the first fault wins
    fault = f; strncpy(faultWhy, why, sizeof faultWhy - 1); faultWhy[sizeof faultWhy - 1] = 0;
    // a fault during any launch-train motion: the phase is no longer trusted, but the last count is kept as a warning
    if (phase == PH_COCKING || phase == PH_FIRING || phase == PH_RESETTING || state == S_DECOCK || state == S_COCK || state == S_FIRE || (state == S_JOG && jogMotor == 1 && collarLaunch())) {
      if (phase != PH_UNKNOWN || cockPhase > phaseHint) phaseHint = cockPhase;
      phase = PH_UNKNOWN;
    }
    tokenLive = false; homing = false;
    { JOut o(evBuf, sizeof evBuf); evBegin(o, "FAULT"); o.str("code", FAULT_NAME[f]); o.str("why", why); evEnd(o); }
    if (f == F_STALL) evSimple("STALL");
    // any fault: wiper off (done), drum to NEUTRAL when that is safe, otherwise the drum holds (worm self-locks)
    const char* hold = safeToNeutral(f);
    int st = safeTarget();
    if (!hold && !atPark(st)) { if (startDrumMove(st, true)) { state = S_SAFING; return; } hold = "drum move failed"; }
    d.halt(now, io.drum);
    state = S_FAULT; stT = now;
    if (hold && !atPark(st)) evHeld(hold);
    else if (atPark(st)) evMode(true);               // already safe
    dirty = true;
  }
  // safe drum target after a fault: NEUTRAL (with the wraps on the drum holds instead, safeToNeutral; the WRAPPED
  // branch stays as a guard should the rule change back)
  int safeTarget() const { return (sec == SEC_REEL || sec == SEC_LOCK) ? DP_WRAPPED : DP_NEUTRAL; }
  bool atPark(int t) const { return sec == targetSec(t) && fabsf(drumDeg - c.targetDeg[t]) < 4 && !d.running(); }
  void evHeld(const char* why) {
    JOut o(evBuf, sizeof evBuf); evBegin(o, "MODE"); o.boolean("ok", false); o.str("mode", modeName()); o.str("drum_pos", DRUM_POS_NAME[drumPos()]);
    o.str("err", "DRUM_HELD"); o.str("why", why); evEnd(o);
  }
  void extFault(int f, const char* why) { if (state != S_FAULT && state != S_SAFING && booted) raise(f, why); }
  // nullptr if the drum may go to NEUTRAL now, else why it must hold
  const char* safeToNeutral(int f) const {
    if (f == F_DRUM_TIMEOUT || f == F_DRUM_SENSOR) return "drum fault";
    if (sec == SEC_INVALID || secBad) return "drum code invalid";
    if (handMode()) return "N20 latched out (hand mode)";
    if (!wrapsKnown) return "wraps unknown";
    if (sec >= SEC_WRAP) return "wraps on: the drum holds, the operator unwraps when safe";   // params.DRUM.fault_rules (D27)
    if (loadedKnown() || (sec <= SEC_LN && maybeLoaded()))
      return "launch train not at rest: the wiper worm holds the spring (a shift under load could dry fire)";
    return nullptr;
  }

  // ================================================================== persistence
  Saved snapshot(uint8_t movingMask) const {
    Saved s; s.moving = movingMask; s.drumKnown = drumKnown; s.drumDeg = drumDeg; s.wraps = (int16_t)wraps; s.wrapsKnown = wrapsKnown;
    s.wiperKnown = enc.ok || wiperInit; s.wiperDeg = enc.deg(); s.aimPhase = aimPhase; s.aimKnown = aimKnown; s.cockPhase = phase == PH_UNKNOWN ? phaseHint : cockPhase;   // unknown: keep the warning angle
    s.cockKnown = phase != PH_UNKNOWN; s.phase = (uint8_t)phase; s.lineM = lineKnown ? lineM : -1.f; s.pad = 0;
    return s;
  }
  void persist(uint8_t movingMask) {
    Saved s = snapshot(movingMask);
    if (savedOnce && sameSaved(s, last)) return;
    s.seq = ++seq; sealSaved(&s); last = s; savedOnce = true;
    if (io.save) io.save(s);
  }
  // called at every motion start: the record says "moving" (positions not trusted) until everything stops
  void markMoving(uint8_t bits) { movingBits |= bits; persist(movingBits); }
  static const uint8_t MV_WIPER = 1, MV_DRUM = 2;

  // first tick after the switches and AS5600 settled: decide what is known
  // ---- D12 degraded angle source: SW_INDEX closes 6 times per shaft turn (every 60 deg). Between bumps the angle is
  // interpolated from the last measured bump-to-bump rate (nominal speed before that), never more than 59 deg past
  // the last bump; each bump snaps the angle to the nearest multiple of 60. Direction = the commanded drive.
  bool indexMode = false, idxOverdue = false; float idxDeg = 0, idxRate = 0, idxLast = 0; uint32_t idxEdgeT = 0; int idxDir = 0;
  void indexTick(uint32_t dt) {
    int dir = w.outDir ? w.outDir : (w.lastDir && now - w.stopT < 120 ? w.lastDir : 0);   // short coast after a stop
    if (dir && rose(SW_INDEX)) {
      float k = roundf(idxDeg / 60.f); float snapped = k * 60.f;
      if (idxDir == dir && idxEdgeT && now > idxEdgeT) { float r = 60000.f / (float)(now - idxEdgeT); if (r > 20 && r < 600) idxRate = r; }
      idxDeg = snapped; idxLast = snapped; idxEdgeT = now; idxDir = dir;
    } else if (dir) {
      // err high: the larger of the measured bump rate and the nominal speed at this duty (stops early, never late)
      float nominal = c.n0Rpm * 6.f * (w.outDuty > 0 ? w.outDuty : 128) / 255.f;
      float rate = idxRate > 0 && idxDir == dir ? fmaxf(idxRate, nominal) : nominal;
      if (!w.outDir) rate *= 0.5f;                    // coasting
      float nd = idxDeg + dir * rate * dt / 1000.f;
      if (fabsf(nd - idxLast) <= 59.f) idxDeg = nd;
      // driving and no bump for 3 x the time 60 deg should take: the cam or its contact failed
      // after a bump in this run: 1.5 x the measured bump period (a missed bump is found within about 90 deg, so a cock
      // that stops a bump early never reaches the release); before the first bump: slowest plausible speed + window
      bool inRun = idxEdgeT && (int32_t)(idxEdgeT - w.startT) > 0 && idxRate > 0 && idxDir == dir;
      uint32_t lim = inRun ? (uint32_t)(1.5f * 60000.f / idxRate)
                           : (uint32_t)((60.f + c.dogWindowDeg) / fmaxf(0.5f * nominal, 30.f) * 1500.f) + 300;
      uint32_t ref = inRun ? idxEdgeT : w.startT;
      if (w.outDir && w.outDuty > 60 && now - ref > lim) idxOverdue = true;
    }
    enc.ok = true; enc.have = true; enc.status = AS_MD;
    enc.count = (int32_t)lroundf(idxDeg * 4096.f / 360.f);
    enc.rateDps = w.outDir ? w.outDir * (idxRate > 0 ? idxRate : c.n0Rpm * 6.f * w.outDuty / 255.f) : 0;
  }
  void resolveBoot() {
    bootPending = false; booted = true;
    if (!enc.ok && fitted(SW_INDEX)) {                 // no magnet: degraded angle source, coarser checks
      indexMode = true; idxDeg = 0; enc.count = 0; c.homeWinDeg += 60;
      c.cockEndDeg = c.releaseDeg - 60; c.cockSlowFrom = c.cockEndDeg - 40;   // the count can be 30 deg out: cock stops a bump early
      if (c.aimTolDeg < 2.0f) c.aimTolDeg = 2.0f;    // D12: looser aim tolerance (30 wiper deg is up to about 1.3 deg of pitch)
    }
    const Saved* s = bootValid ? &boot : nullptr;
    bool wiperTrust = s && !(s->moving & MV_WIPER) && s->wiperKnown;
    bool drumTrust = s && !(s->moving & MV_DRUM);
    // the drum code is absolute: the sector is known whatever happened; the angle inside it comes from the record
    // when the drum was not moving and the record agrees with the sector, else the sector middle
    if (sec != SEC_INVALID) {
      drumKnown = true;
      if (drumTrust && s->drumKnown && s->drumDeg >= secLo(sec) - 1 && s->drumDeg <= secHi(sec) + 1) { drumDeg = s->drumDeg; degTrusted = true; }
      else {
        if (drumTrust && s->drumKnown) bootDrumMismatch = true;   // the drum cannot move while off (worm): a code contact changed
        int k = detentClosed();
        drumDeg = k >= 0 ? c.targetDeg[k] : 0.5f * (fmaxf(secLo(sec), c.targetDeg[0]) + fminf(secHi(sec), c.targetDeg[2])); degTrusted = k >= 0 && !(s && (s->moving & MV_DRUM));   // cut mid-move: the park angle is only a guess
      }
      wraps = wrapsAt(sec, drumDeg); wrapsKnown = sec != SEC_WRAP || degTrusted;
    } else { drumKnown = false; wrapsKnown = false; wraps = 0; }
    if (s && s->lineM >= 0) { lineM = s->lineM; lineKnown = true; } else { lineKnown = false; }
    // the record is written as a motion starts: a cut while the wiper turned (reel, take-up) leaves line_m stale
    if (s && (s->moving & MV_WIPER) && sec >= SEC_NEUTRAL) lineKnown = false;
    // wiper: the AS5600 knows the angle inside one turn; the record knows the turns, if nothing moved since
    bool wiperOk = false;
    if (enc.ok && wiperTrust && !indexMode) {
      float diff = wrap180(enc.absDeg() - wrap360(s->wiperDeg));
      if (fabsf(diff) < 3.0f) { enc.setDeg(s->wiperDeg + diff); wiperOk = true; }
    }
    if (enc.ok) wiperInit = true;
    lastWiperDeg = enc.deg();
    if (wiperOk) {
      aimPhase = s->aimPhase; aimKnown = s->aimKnown; aimOffset = aimOffset;
      phase = s->cockKnown ? (int)s->phase : (int)PH_UNKNOWN; cockPhase = s->cockPhase;
      if (!s->cockKnown && s->cockPhase > 0) { phaseHint = s->cockPhase; cockPhase = 0; }
      // never trust a mid-cycle stop. The record was written as the motion started, so its angle is the START angle:
      // assume the worst (drawn to the cock end) so a lying SW_HOME can never let a recovery creep reach the release
      if (phase == PH_COCKING || phase == PH_FIRING) { phaseHint = fmaxf(cockPhase, c.cockEndDeg); phase = PH_UNKNOWN; }
      else if (phase == PH_RESETTING) { phaseHint = cockPhase; phase = PH_UNKNOWN; }
    } else {
      aimKnown = false; phase = PH_UNKNOWN;
      if (s && !s->cockKnown && s->cockPhase > 0) phaseHint = s->cockPhase;
      if (s && s->cockKnown && (s->phase == PH_COCKED || s->phase == PH_COCKING || s->phase == PH_FIRING)) phaseHint = s->phase == PH_COCKED ? c.cockEndDeg : fmaxf(s->cockPhase, c.cockEndDeg);   // a stale angle is still a warning
      // systest F15: the wiper was turning with the launch train at rest (aim, centering jog, reel, take-up: a cock or
      // fire records COCKING / FIRING before it starts). With SW_HOME and SW_GAP closed the springs are still relaxed
      if (s && s->cockKnown && s->phase == PH_REST && fitted(SW_HOME) && closed(SW_HOME) && (!fitted(SW_GAP) || closed(SW_GAP))) { phase = PH_REST; cockPhase = 0; phaseHint = -1; }
    }
    // SW_HOME (carriage forward) must agree with the record: rest = closed, cocked = open
    if (fitted(SW_HOME)) {
      if (phase == PH_REST && (!closed(SW_HOME) || (fitted(SW_GAP) && !closed(SW_GAP)))) phase = PH_UNKNOWN;
      if (phase == PH_COCKED && closed(SW_HOME)) { phaseHint = c.cockEndDeg; phase = PH_UNKNOWN; }   // SW_HOME lying, or a shot happened
    }
    JOut o(evBuf, sizeof evBuf); evBegin(o, "BOOT"); o.str("fw", GB_FW_VERSION); o.boolean("record", s != nullptr);
    o.boolean("was_moving", s && s->moving); o.boolean("drum_known", drumKnown); o.boolean("wraps_known", wrapsKnown);
    o.boolean("aim_known", aimKnown); o.str("phase", PHASE_NAME[phase]); o.str("angle", indexMode ? "INDEX" : enc.ok ? "AS5600" : "NONE"); evEnd(o);
    movingBits = 0; persist(0);
  }

  // ================================================================== sensors
  void readSensors(uint32_t dt) {
    if (indexMode) {                                 // D12: no magnet, the 6-bump index cam stands in for the AS5600
      sw.update(io.switches ? io.switches() : 0, now, enc.deg());
      indexTick(dt);
    } else {
      uint8_t st = 0; int raw = io.angle ? io.angle(&st) : -1;
      enc.feed(raw, st, dt);
      sw.update(io.switches ? io.switches() : 0, now, enc.deg());
    }
    updateSector();
    float isv = io.wiperIsVolts ? io.wiperIsVolts() : 0;
    w.tick(now, dt, isv, enc.ok ? enc.rateDps : NAN, io.wiper, startOk());   // ramp, gap, current, stall latches
    amps = w.amps;
    vbat = io.vbat && c.vbatWired ? io.vbat() : NAN;
    float hx = io.hxRaw ? io.hxRaw() : NAN;
    if (!isnan(hx)) { hxN = (hx - hxOff) * c.hxNPerCount; hxT = now; hxSeen = true; }
    bool winding = w.outDir == c.reelDir && sec == SEC_REEL && now - w.startT > c.tensionIgnoreMs;
    float tc = winding ? (w.amps - c.freeA) * c.kNperA + c.tensOffsetN : 0; if (tc < 0) tc = 0;
    float al = (float)dt / c.tensionAvgMs; if (al > 1) al = 1;   // D33: tow current averaged over about 200 ms
    // a restart from HOLD keeps the held estimate through the start-up blanking (no ratchet past the limit)
    // (it only tapers the start); the first measured sample then replaces it
    if (winding && !windPrev) tCur = tc;
    windPrev = winding;
    tCur = winding ? tCur + (tc - tCur) * al : ((state == S_HOLD || state == S_REEL) ? tCur : 0);
    tRaw = winding ? tc : 0;                          // unfiltered: a stiff line rises faster than the 200 ms average
    tensionN = hxLive() ? fmaxf(hxN, tCur) : tCur;    // a frozen or unplugged load cell never hides the current estimate
  }
  bool hxLive() const { return hxSeen && now - hxT < 500; }
  float tRaw = 0; bool windPrev = false; uint32_t rawOverT = 0;

  // wiper motion -> trains. Only while running or just stopped (coast). The collar dogs have a free window
  // (dogWindowDeg): after a reversal the wiper first crosses the window and drives nothing. win = 0 touching the
  // backward-drive flank, win = window touching the forward-drive flank. Only when engaged (LAUNCH or REEL).
  float win = 16; bool winKnown = false;
  void trackWiper() {
    if (!enc.ok) return;
    float deg = enc.deg();
    if (!wiperInit) { lastWiperDeg = deg; wiperInit = true; return; }
    float dd = deg - lastWiperDeg; lastWiperDeg = deg;
    if (dd == 0) return;
    bool live = w.running() || (w.lastDir != 0 && now - w.stopT < 400);
    if (!live) return;
    bool launch = collarLaunch(), reel = collarReel();
    if (!launch && !reel) { if (fabsf(dd) > 0.5f) winKnown = false; return; }   // NEUTRAL: the trains do not move
    // D12 index mode: a bump snaps the interpolated angle back. A step against the drive is a correction of the travel
    // already counted into the driven train, never motion of the other train (or the dog window)
    if (indexMode && w.lastDir != 0 && dd * w.lastDir < 0) {
      float corr = fabsf(dd);
      if (launch && w.lastDir == cockDir()) { cockPhase -= corr; if (cockPhase < 0) cockPhase = 0; }
      else if (launch) aimPhase = wrapCyc(aimPhase - corr, c.aimCycleDeg);
      return;
    }
    float fwd = 0, back = 0;                         // train travel after the window
    if (dd > 0) { float room = c.dogWindowDeg - win; float use = dd < room ? dd : room; win += use; fwd = dd - use; }
    else { float room = win; float use = -dd < room ? -dd : room; win -= use; back = -dd - use; }
    if (launch) {
      if (back > 0) {
        if (lashDir != -1) { lashLeft = c.aimLashDeg; lashDir = -1; }   // the splitter arm swings over first
        float use = back; if (lashLeft > 0) { float l = use < lashLeft ? use : lashLeft; lashLeft -= l; use -= l; }
        if (c.aimDir < 0) aimPhase = wrapCyc(aimPhase + use, c.aimCycleDeg);
        if (phase != PH_REST && cockPhase > c.homeOpenDeg - 1) cockPhase = fmaxf(c.homeOpenDeg - 1, cockPhase - back);   // drawn spring follows back
      }
      if (fwd > 0) { lashDir = 1; if (c.aimDir > 0) aimPhase = wrapCyc(aimPhase + fwd, c.aimCycleDeg); else cockPhase += fwd; }
    } else if (reel && fwd > 0 && c.reelDir > 0) {
      lineM -= fwd / 360.f * c.capstanPerWiper * (float)M_PI * c.capstanDmm / 1000.f; if (lineM < 0) lineM = 0;
    }
  }

  // sector from the code. Adjacent changes are edges (snap the angle estimate, learn the speed); code 000 or a jump
  // over a sector keeps the last sector and starts the DRUM_SENSOR timer (watch()).
  void updateSector() {
    int s = sectorOf(codeNow());
    if (s == SEC_INVALID || (sec != SEC_INVALID && abs(s - sec) > 1 && booted)) { if (!secBad) { secBad = true; secBadT = now; } badSeen = true; return; }
    secBad = false;
    // after a bad code (000 or a jump) the drum may have moved unseen: the next sector or edge is not a trusted angle
    bool afterBad = badSeen; badSeen = false;
    if (s == sec) { if (afterBad) degTrusted = false; return; }
    int was = sec; prevSec = sec; sec = s; secChangeT = now;
    if (was == SEC_INVALID || !booted) { drumKnown = true; return; }
    int dir = s > was ? 1 : -1;
    float e = c.edgeDeg[s > was ? was : s];
    uint32_t te = 0; for (int id = SW_DRUM_A; id <= SW_DRUM_C; id++) if (fitted(id)) { uint32_t x = sw.edgeTime(c.swBit[id]); if (te == 0 || (int32_t)(x - te) > 0) te = x; }
    if (edgeRun && d.running() && edgeAt != e) {          // two edges in one run: real speed
      float ms = (float)(te - edgeT);
      if (ms > 100) { float v = fabsf(e - edgeAt) / (ms / 1000.f); if (v > 0.3f * c.drumDps && v < 3 * c.drumDps) c.drumDps = 0.5f * c.drumDps + 0.5f * v; }
    }
    // SW_ARM: one pulse per cup turn across the WRAP sector; a full pass must count all of them
    if (s == SEC_WRAP && (was == SEC_PREP || was == SEC_LOCK)) { armPulses = 0; armFrom = was; }
    // only a whole pass in one go counts (entered at one end, left at the other, never stopped inside)
    if (was == SEC_WRAP && (s == SEC_PREP || s == SEC_LOCK) && s != armFrom && fitted(SW_ARM) && d.running() && armCounting) {
      if (armPulses != c.wrapsFull) armBad = true;
    }
    armCounting = s == SEC_WRAP && (was == SEC_PREP || was == SEC_LOCK) ? (state == S_WRAP || state == S_UNWRAP) : (s == SEC_WRAP ? armCounting : false);
    edgeT = te; edgeAt = e; edgeRun = d.running() && !afterBad; degTrusted = !afterBad;
    if ((was == SEC_WRAP || s == SEC_WRAP) && !wrapsDoubt) wrapsKnown = true;   // an edge of the WRAP sector gives the exact angle
    drumDeg = e + dir * c.drumDps * (now - te) / 1000.f; drumKnown = true;
  }
  // dead reckoning inside the sector (never leaves the sector's angle range)
  void trackDrum(uint32_t dt) {
    int dir = d.outDir * c.drumSign;
    if (!d.running()) edgeRun = false;
    if (sec == SEC_INVALID) return;
    if (dir) drumDeg += dir * c.drumDps * dt / 1000.f;
    float lo = secLo(sec), hi = secHi(sec);
    if (drumDeg < lo) drumDeg = lo; if (drumDeg > hi) drumDeg = hi;
    wraps = wrapsAt(sec, drumDeg);
    if (sec != SEC_WRAP && !wrapsDoubt) wrapsKnown = true;           // outside the WRAP sector the count follows from the sector
    if (sec == SEC_WRAP && (c.armHomeOpen ? fell(SW_ARM) : rose(SW_ARM)) && d.running()) armPulses++;   // one per cup arrival at home
  }

  // ================================================================== watchdogs (every tick)
  void watch() {
    if (state == S_FAULT) { if (sec != SEC_INVALID) quietSec = sec; return; }   // a hand-turned drum: one clear is enough
    if (w.overcurrent) { char b[48]; snprintf(b, sizeof b, "wiper over %.1f A", w.c.tripA); w.clearLatches(); onOvercurrent(b); return; }
    if (w.stalled) { w.clearLatches(); onStall(); return; }
    if (!enc.ok && (w.running() || state == S_AIM || state == S_COCK || state == S_FIRE || state == S_REEL)) { raise(F_ANGLE_SENSOR, "AS5600 lost while the wiper runs"); return; }
    if (secBad && now - secBadT > c.invalidMs) { secBadT = now; raise(F_DRUM_SENSOR, sectorOf(codeNow()) == SEC_INVALID ? "drum code 000 (wire off or bump missing)" : "drum code jumped over a sector"); return; }
    if (handMode() && d.running()) { raise(F_DRUM_SENSOR, "N20 latched out during a drum move"); return; }
    if (closed(SW_DOG_L) && closed(SW_DOG_R)) { if (now - dogBothT > 200) { raise(F_SWITCH_DISAGREE, "SW_DOG_L and SW_DOG_R both closed"); return; } }
    else dogBothT = now;
    bool drumMoveState = state == S_DRUM || state == S_SAFING || state == S_WRAP || state == S_UNWRAP;
    if (drumMoveState && (chat(SW_DRUM_A) || chat(SW_DRUM_B) || chat(SW_DRUM_C))) { raise(F_SWITCH_DISAGREE, "drum code contact chattering"); return; }
    bool running = w.running() || d.running();
    if (!isnan(vbat) && running && vbat < c.vbatLow) {
      if (!lowRun) { lowRun = true; lowT = now; } else if (now - lowT >= c.vbatLowMs) { raise(F_BROWNOUT, "battery sagged under load"); return; }
    } else lowRun = false;
    if (tokenLive && now - tokenT > c.armMs) { tokenLive = false; tokenExpired = true; }
    // the drum cannot move by itself (worm): a code change with the N20 off for 300 ms means a lying contact
    if (sec != quietSec) {
      if (!d.running() && now - d.stopT > 300 && booted && quietSec != SEC_INVALID && !(state == S_JOG && jogMotor == 2)) {
        // a code contact may be lying near the wrap window: the count is doubtful until an unwrap reaches NEUTRAL
        if (quietSec >= SEC_PREP || sec >= SEC_PREP) { wrapsDoubt = true; wrapsKnown = false; }
        quietSec = sec; raise(F_DRUM_SENSOR, "drum code changed with the drum still"); return;
      }
      quietSec = sec;
    }
    // a launch action needs the collar in LAUNCH all the time
    if ((state == S_AIM || state == S_COCK || state == S_FIRE || state == S_DECOCK || (state == S_JOG && jogMotor == 1 && jogLaunch)) && (!collarLaunch() || chat(SW_DOG_L))) { raise(F_BAD_STATE, "collar left LAUNCH during a launch action"); return; }
    // AS5600 plausibility while the wiper is driven hard: many rejected samples, or counting the wrong way (enc_sign)
    if (w.running() && w.outDuty > 100 && now - w.startT > 400 && enc.ok && !indexMode) {
      if (enc.rateDps * w.outDir < -30) { if (!encWrongT) encWrongT = now; else if (now - encWrongT > 200) { raise(F_ANGLE_SENSOR, "AS5600 counts against the drive (enc_sign?)"); return; } }
      else encWrongT = 0;
    } else encWrongT = 0;
    if (enc.resyncs != resyncRef) {                   // a jump taken uncounted: the travel is unknown
      resyncRef = enc.resyncs;
      if (w.running() || state == S_AIM || state == S_COCK || state == S_FIRE || state == S_DECOCK) { raise(F_ANGLE_SENSOR, "AS5600 angle jumped while turning (travel lost)"); return; }
      aimKnown = false; aimed = false;               // idle (hand turned?): the aim must be re-homed
      if (phase == PH_COCKED || phase == PH_COCKING) { phaseHint = c.cockEndDeg; phase = PH_UNKNOWN; }
    }
    if (enc.glitches != glitchRef) { uint32_t n = enc.glitches - glitchRef; glitchRef = enc.glitches; glitchWin += n; }
    if (now - glitchT >= 1000) { if (glitchWin > 20 && w.running()) { glitchWin = 0; glitchT = now; raise(F_ANGLE_SENSOR, "AS5600 glitching (over 20 bad samples a second)"); return; } glitchWin = 0; glitchT = now; }
    // launch train invariants, every tick:
    // the carriage left its stop while the phase says rest (a jog or stop inside the switch debounce): not rest any more
    if (phase == PH_REST && fitted(SW_HOME) && !closed(SW_HOME) && state != S_COCK && state != S_FIRE) { phaseHint = cockPhase > c.homeOpenDeg ? cockPhase : c.homeOpenDeg + 1; phase = PH_UNKNOWN; dirty = true; }
    // forward travel past the first tooth from rest is cocking, whatever the switch says
    if (phase == PH_REST && cockPhase > c.homeOpenDeg + c.homeWinDeg) {
      if (fitted(SW_HOME) && closed(SW_HOME)) { raise(F_SWITCH_DISAGREE, "SW_HOME still closed past the first tooth"); return; }
      phase = PH_COCKING; dirty = true;
    }
    // the drum code says away from LAUNCH while the spring is known drawn: a code contact is lying
    if (loadedKnown() && sec != SEC_INVALID && sec > SEC_LN && booted) { raise(F_BAD_STATE, "drum code away from LAUNCH but the spring is drawn"); return; }
    if (idxOverdue) { idxOverdue = false; idxEdgeT = now; raise(F_ANGLE_SENSOR, "SW_INDEX bump overdue (index cam or contact)"); return; }
    // the cup only turns inside the WRAP window: SW_ARM moving anywhere else (1.5 s after the last code edge) means a lying
    // drum code contact, and the drum may be unwinding wraps without the take-up
    if (sec != SEC_WRAP && sec != SEC_INVALID && !secBad && fitted(SW_ARM) && (rose(SW_ARM) || fell(SW_ARM)) && now - secChangeT > 1500) {
      wrapsDoubt = true; wrapsKnown = false;
      raise(F_DRUM_SENSOR, "SW_ARM moved outside the wrap window (drum code contact?)"); return;
    }
    if (armBad) { armBad = false; char b[48]; snprintf(b, sizeof b, "SW_ARM counted %d of %d cup turns", armPulses, c.wrapsFull); raise(F_DRUM_SENSOR, b); return; }
    if (bootDrumMismatch) { bootDrumMismatch = false; raise(F_DRUM_SENSOR, "drum code disagrees with the saved drum angle"); return; }
    // optional host watchdog: no line from the Jetson for host_timeout_s while a motor runs = stop (0 = off)
    if (c.hostTimeoutS > 0 && running && now - hostT > (uint32_t)(c.hostTimeoutS * 1000)) { cmdStop(false); evAborted("HOST", "host silent"); hostT = now; }
  }
  uint32_t hostT = 0;
  uint32_t secChangeT = 0; int armPulses = 0, armFrom = SEC_INVALID; bool armBad = false, armCounting = false;
  int quietSec = SEC_INVALID; uint32_t resyncRef = 0; uint32_t encWrongT = 0, glitchRef = 0, glitchWin = 0, glitchT = 0;
  uint32_t lastMultiOk = 0, dogBothT = 0; bool bootDrumMismatch = false; bool degTrusted = true; bool badSeen = false;
  void onOvercurrent(const char* why) { raise(F_OVERCURRENT, why); }   // always a fault (also in a tow: the limit acts first)
  void onStall() {
    // a tow that stalls close to its limit is at the limit (the motor ran out of torque at the minimum duty): hold
    if (state == S_REEL && until == U_TENSION && tensionN >= 0.8f * maxN) { w.halt(now, io.wiper); enterHold("stalled at the limit"); return; }
    raise(F_STALL, "wiper stalled");
  }

  // ================================================================== drum moves
  // One mover for mode changes, wrap, unwrap and safing (lens B 7.2): centering jog when leaving an engaged collar,
  // run to the target sector's edge, run on the extra angle by time; while unwrapping the wiper turns forward slowly in
  // the WRAP sector (take-up only); while the collar drops into REEL the wiper creeps forward (fly-in); at the end the
  // dog contacts must confirm the collar (LAUNCH: wiggle BACKWARD if needed, never forward: that would cock).
  int mvTarget = -1, mvDir = 0, mvStage = 0, mvSafing = 0; uint32_t mvT = 0, mvLegT = 0, mvSecT = 0, mvEnd = 0; float mvExpect = 0, mvFrom = 0;
  bool flyBit = false; uint32_t flyT = 0, flyPauseT = 0; float flyRef = 0, flyLegFrom = 0;
  int mvLastSec = SEC_INVALID; bool mvTakeup = false; int mvWiper = 0; bool mvTrusted = false; float mvEdgeExpect = 0; bool mvRefind = false; float mvRefEdge = 0;
  bool handMode() const { return fitted(SW_N20) && !closed(SW_N20); }   // N20 latched out: the drum is turned by hand
  bool startDrumMove(int target, bool safing) {
    if (sec == SEC_INVALID || secBad || handMode()) return false;
    int ts = targetSec(target);
    mvTarget = target; mvSafing = safing; mvDir = ts > sec ? 1 : ts < sec ? -1 : (c.targetDeg[target] >= drumDeg ? 1 : -1);
    mvTakeup = mvDir < 0 && (sec >= SEC_WRAP || (wrapsDoubt && sec >= SEC_PREP));   // any downward move from LOCK / REEL   // unwrapping: take up the freed line
    float from = drumDeg;
    if (!degTrusted) from = mvDir > 0 ? fmaxf(secLo(sec), c.secLoDeg) : fminf(secHi(sec), c.secHiDeg);   // worst case inside the sector
    mvExpect = fabsf(c.targetDeg[target] - from) / c.drumDps * 1000.f;
    mvTrusted = degTrusted; mvRefind = false; flyBit = false; flyT = 0; flyPauseT = 0;        // the 'came too early' check needs a known start angle
    { int tsec = targetSec(target); float edge = mvDir > 0 ? secLo(tsec) : secHi(tsec);   // where the target code starts
      mvEdgeExpect = (tsec == sec) ? 0 : fabsf(edge - from) / c.drumDps * 1000.f; }
    mvT = mvLegT = mvSecT = now; mvLastSec = sec; mvFrom = enc.deg(); mvWiper = 0;
    markMoving(MV_DRUM);
    if (sec == SEC_WRAP) armCounting = false;        // a move that starts inside the window is not a whole pass
    bool engaged = (sec == SEC_LAUNCH || sec == SEC_REEL) && ts != sec;
    if (engaged && c.centerDeg > 0 && enc.ok && !safing) {   // a fault turns the wiper off: safing never jogs       // centering jog: collar off both flanks before it moves
      // in REEL first take the window up FORWARD (the capstan only winds in), so the jog back can never reach the
      // backward flank (sprag #2 would hold it: a stall); in LAUNCH only backward (forward would cock)
      if (sec == SEC_REEL && !collarReel()) { /* dogs on their tips, not engaged: nothing to centre */ }
      else if (sec == SEC_REEL) { mvStage = 5; wiper(c.reelDir, c.flyDuty, c.jogTripA); return true; }
      else { mvStage = 3; wiper(c.aimDir < 0 ? -1 : 1, c.jogDuty, c.jogTripA); return true; }
    }
    if (ts == sec) { mvStage = 1; mvEnd = now + (uint32_t)(fabsf(c.targetDeg[target] - drumDeg) / c.drumDps * 1000.f); }
    else mvStage = 0;
    d.want(mvDir * c.drumSign);
    return true;
  }
  // start the wiper for a helper motion with its own current trip
  void wiper(int dir, int duty, float trip) { w.c.tripA = trip; w.want(dir, duty); if (mvWiper != dir) { mvWiper = dir; markMoving(MV_WIPER); } }
  uint32_t mvTimeout() const { return (uint32_t)(mvExpect * c.drumTimeFactor) + c.drumTimeMarginMs; }
  // wiper helpers during the run: take-up in the WRAP / PREP sectors while unwrapping, fly-in in the last part of LOCK
  void mvWiperRule() {
    bool take = mvTakeup && (sec == SEC_WRAP || sec == SEC_PREP || (sec == SEC_LOCK && drumDeg < c.collarNdeg));   // a lying LOCK code still takes up
    bool fly = targetSec(mvTarget) == SEC_REEL && (sec == SEC_REEL || (sec == SEC_LOCK && drumDeg >= c.nrEngageDeg - 18));
    if (take) wiper(c.reelDir, c.takeupDuty, c.takeupTripA);
    else if (fly) {
      // D41: creep only until the dogs bite (SW_DOG_R closes, or the current rises flyBiteA over the creep's running
      // level: on a straight line about 3.4 N per wiper degree after the bite); after that the drum finishes alone
      if (!flyBit && collarReel()) flyBit = true;
      if (!flyBit) {
        if (!flyT || !w.wantDir) { flyT = now; flyRef = 0; }
        wiper(c.reelDir, c.flyDuty, c.jogTripA);
        if (now - flyT > 250) { flyRef = flyRef == 0 ? w.amps : flyRef + (fminf(w.amps, flyRef + 0.3f) - flyRef) * 0.05f;
          if (now - flyT > 400 && w.amps > flyRef + c.flyBiteA) flyBit = true; }
      }
      if (flyBit && w.wantDir) { w.halt(now, io.wiper); mvWiper = 0; }
    }
    else if (w.wantDir) { w.halt(now, io.wiper); mvWiper = 0; }
  }
  // 0 running, 1 arrived, -1 failed (fault raised, or for safing reported)
  int tickDrumMove() {
    bool safing = mvSafing;
    int ts = targetSec(mvTarget);
    if (mvStage == 5) {                               // REEL: window taken up forward first
      if (fabsf(enc.deg() - mvFrom) >= c.dogWindowDeg || now - mvLegT > 3000) {
        w.halt(now, io.wiper); mvWiper = 0; win = c.dogWindowDeg; mvFrom = enc.deg(); mvLegT = now; mvStage = 3;
        wiper(-c.reelDir, c.jogDuty, c.jogTripA);
      }
      return 0;
    }
    if (mvStage == 3) {                               // centering jog
      // also bounded in time (1.5 x the free-speed time): a noisy angle must never carry the jog past the window
      uint32_t capMs = (uint32_t)(c.centerDeg / (c.n0Rpm * 6.f * c.jogDuty / 255.f) * 1500.f) + c.drumTimeMarginMs / 10 + 200;
      if (fabsf(enc.deg() - mvFrom) >= c.centerDeg || now - mvLegT > capMs) {
        w.halt(now, io.wiper); mvWiper = 0; win = c.dogWindowDeg * 0.5f;
        mvStage = 0; mvT = mvLegT = mvSecT = now; d.want(mvDir * c.drumSign);
      }
      return 0;
    }
    // collar contacts checked during every move: DOG_L only up to L_TO_N, DOG_R only from N_TO_R (never in the wrap window)
    if ((mvStage == 0 || mvStage == 1) && ((closed(SW_DOG_L) && sec > SEC_LN) || (closed(SW_DOG_R) && sec < SEC_LOCK))) {
      failDrum(safing, F_SWITCH_DISAGREE, "collar contact disagrees with the drum code"); return -1;
    }
    if (mvStage == 0) {
      if (sec != mvLastSec) {
        if ((sec - mvLastSec) * mvDir < 0) { failDrum(safing, F_DRUM_SENSOR, "drum turned the wrong way (drum_sign?)"); return -1; }
        mvLastSec = sec; mvSecT = now;
      }
      mvWiperRule();
      if (sec == ts) {
        float el = (float)(now - mvT);
        if (mvTrusted && el < c.drumEarlyFrac * mvEdgeExpect && mvEdgeExpect > 1500) { failDrum(safing, F_DRUM_SENSOR, "target code came far too early"); return -1; }
        float extra = fabsf(c.targetDeg[mvTarget] - edgeAt) / c.drumDps * 1000.f;
        uint32_t since = now - edgeT; mvEnd = now + (extra > since ? (uint32_t)(extra - since) : 0);
        mvStage = 1; return 0;
      }
      float span = secHi(sec) - secLo(sec); if (sec == SEC_LAUNCH || sec == SEC_REEL) span = 20;
      if (now - mvSecT > (uint32_t)(span / c.drumDps * 1000.f * c.drumTimeFactor) + c.drumTimeMarginMs) { failDrum(safing, F_DRUM_TIMEOUT, "drum not turning (no code edge)"); return -1; }
      if (now - mvT > mvTimeout()) { failDrum(safing, F_DRUM_TIMEOUT, "drum did not reach the target in time"); return -1; }
      return 0;
    }
    if (mvStage == 1) {
      mvWiperRule();
      if (sec != ts) {                                // left the target sector early (start angle or drum_dps wrong): stop,
        d.halt(now, io.drum);                         // come back to the edge just crossed, then run on from there
        if (mvRefind || sec == SEC_INVALID) { failDrum(safing, F_DRUM_SENSOR, "drum overshot the target sector"); return -1; }
        mvRefind = true; mvRefEdge = sec < ts ? secLo(ts) : secHi(ts); mvDir = sec < ts ? 1 : -1;
        if (w.wantDir) { w.halt(now, io.wiper); mvWiper = 0; }
        mvStage = 6; mvLegT = now; d.want(mvDir * c.drumSign); return 0;
      }
      if ((int32_t)(now - mvEnd) < 0) return 0;
      d.halt(now, io.drum);
      if (sec != ts) { failDrum(safing, F_DRUM_SENSOR, "drum overshot the target sector"); return -1; }
      drumDeg = c.targetDeg[mvTarget];
      mvStage = 4; mvLegT = now; mvFrom = enc.deg();
      return 0;
    }
    if (mvStage == 6) {                               // back to the edge just crossed
      if (sec == ts) { drumDeg = mvRefEdge; mvEnd = now + (uint32_t)(fabsf(c.targetDeg[mvTarget] - mvRefEdge) / c.drumDps * 1000.f); mvStage = 1; return 0; }
      if (now - mvLegT > (uint32_t)(10.f / c.drumDps * 1000.f) + c.drumTimeMarginMs) { failDrum(safing, F_DRUM_TIMEOUT, "drum did not come back to the target sector"); return -1; }
      return 0;
    }
    if (mvStage == 4) {                               // collar check (dog contacts), fly-in / wiggle until engaged
      bool okL = !fitted(SW_DOG_L) || closed(SW_DOG_L), okR = !fitted(SW_DOG_R) || closed(SW_DOG_R);
      if (closed(SW_DOG_L) && closed(SW_DOG_R)) { failDrum(safing, F_SWITCH_DISAGREE, "SW_DOG_L and SW_DOG_R both closed"); return -1; }
      if (ts == SEC_NEUTRAL || ts == SEC_LOCK) {      // NEUTRAL and the WRAPPED park: the collar must be free
        if (now - mvLegT < 100) return 0;
        if (closed(SW_DOG_L) || closed(SW_DOG_R)) { failDrum(safing, F_SWITCH_DISAGREE, "collar still engaged at NEUTRAL"); return -1; }
        return 1;
      }
      bool engaged = ts == SEC_LAUNCH ? okL && !closed(SW_DOG_R) : okR && !closed(SW_DOG_L);
      if (engaged && now - mvLegT >= 60) {
        w.halt(now, io.wiper); mvWiper = 0;
        if (ts == SEC_LAUNCH) { aimKnown = false; aimed = false; win = c.dogWindowDeg * 0.5f; }   // dog window unknown: re-home the aim
        else win = c.dogWindowDeg;                    // fly-in ends on the forward flank
        return 1;
      }
      if (engaged) return 0;
      if (safing) { w.halt(now, io.wiper); return 1; }   // safing never turns the wiper
      if (ts == SEC_LAUNCH && maybeLoaded()) { failDrum(safing, F_BAD_STATE, "LAUNCH collar not confirmed with the spring possibly drawn"); return -1; }
      float lim = ts == SEC_LAUNCH ? c.flyLaunchMaxDeg : c.flyMaxDeg;
      if (fabsf(enc.deg() - mvFrom) >= lim || now - mvLegT > 6000 + (ts == SEC_REEL ? 6000 : 0)) { w.halt(now, io.wiper); failDrum(safing, F_DRUM_TIMEOUT, "dogs did not engage (collar contact open)"); return -1; }
      if (ts == SEC_LAUNCH) { wiper(c.aimDir, c.flyDuty, c.jogTripA); return 0; }
      // D41: REEL not confirmed after the drum move: wind in at most 5 deg at a time, limited by the tension estimate
      // (20 N, like a tow start) instead of the 3 A jog trip; pause between legs to read SW_DOG_R
      if (tRaw >= c.flyMaxN) { w.halt(now, io.wiper); failDrum(safing, F_DRUM_TIMEOUT, "dogs did not engage: line tension reached the fly-in limit"); return -1; }
      if (w.wantDir) {
        if (fabsf(enc.deg() - flyLegFrom) >= c.flyLegDeg) { w.halt(now, io.wiper); mvWiper = 0; flyPauseT = now; }
      } else if (now - flyPauseT >= 200) {
        flyLegFrom = enc.deg(); float need = (c.flyMaxN - c.tensOffsetN) / c.kNperA + c.freeA + 1.0f;
        wiper(c.reelDir, c.flyDuty, fminf(fmaxf(c.jogTripA, need), c.hardTripA));
      }
      return 0;
    }
    return -1;
  }
  void failDrum(bool safing, int f, const char* why) {
    d.halt(now, io.drum); w.halt(now, io.wiper); mvWiper = 0;
    if (safing && state == S_SAFING) {               // the fault is already latched: report the failed move, stay in FAULT
      state = S_FAULT; stT = now;
      JOut o(evBuf, sizeof evBuf); evBegin(o, "MODE"); o.boolean("ok", false); o.str("mode", modeName()); o.str("drum_pos", DRUM_POS_NAME[drumPos()]);
      o.str("err", FAULT_NAME[f]); o.str("why", why); evEnd(o); dirty = true; return;
    }
    raise(f, why);                                   // a failed estop move is a real drum fault too
  }

  // ================================================================== commands
  // refusal helper: returns error code or nullptr
  const char* idleCheck(bool needEnc) const {
    if (state == S_FAULT || state == S_SAFING) return "FAULTED";
    if (state != S_IDLE && state != S_HOLD) return "BUSY";
    if (!booted) return "BOOTING";
    if (!powerOk()) return "NO_POWER";
    if (needEnc && !enc.ok) return "ANGLE_SENSOR";
    return nullptr;
  }
  // the spring may be drawn: the launch phase is not REST, or SW_HOME (carriage home) is open
  bool loadedKnown() const { return phase == PH_COCKING || phase == PH_COCKED || phase == PH_FIRING; }
  bool maybeLoaded() const { return phase != PH_REST || cockPhase > c.homeOpenDeg + 0.5f || (fitted(SW_HOME) && !closed(SW_HOME)); }
  // lens B I1: the drum may leave LAUNCH only with the launcher at rest and the carriage home (springs relaxed)
  const char* launchExitCheck() const {
    if (maybeLoaded()) return "LAUNCH_BUSY";
    if (fitted(SW_GAP) && !closed(SW_GAP)) return "LAUNCH_BUSY";   // big gear must sit in its gap (lens B I1)
    return nullptr;
  }
  const char* cmdMode(const char* to) {
    int t = !strcmp(to, "LAUNCH") ? DP_LAUNCH : !strcmp(to, "NEUTRAL") ? DP_NEUTRAL : !strcmp(to, "REEL") ? DP_REEL : -1;
    if (t < 0) return "BAD_ARG";
    if (const char* e = idleCheck(false)) return e;
    if (state == S_HOLD) return "HOLDING";
    if (sec == SEC_INVALID || secBad) return "POSITION_UNKNOWN";
    if (handMode()) return "HAND_MODE";
    // the centering jog and the fly-in need the wiper angle: without it the collar may leave a loaded flank or never seat
    if (!enc.ok && (t == DP_REEL || ((sec == SEC_LAUNCH || sec == SEC_REEL) && detentClosed() != t))) return "ANGLE_SENSOR";
    if (wrapsDoubt) return "WRAPS_ON";               // drum code doubted near the wrap window: unwrap first
    if (t == DP_REEL) {                              // REEL engages only with the wraps on (D16): from LOCK, or use "wrap"
      if (sec == SEC_REEL && collarReel()) { evMode(true); return nullptr; }
      if (sec < SEC_LOCK) return "NOT_WRAPPED";
    } else {
      if (sec >= SEC_WRAP) return "WRAPS_ON";        // unwrap first
    }
    int k = detentClosed();
    bool collarOk = t == DP_LAUNCH ? collarLaunch() : t == DP_REEL ? collarReel() : (!closed(SW_DOG_L) && !closed(SW_DOG_R));
    if (k == t && fabsf(drumDeg - c.targetDeg[t]) < 3 && collarOk) { evMode(true); return nullptr; }   // else run the move: it confirms the collar
    // the worm holds the spring: never shift while the launch train is not at rest, unless the drum is positively
    // away from LAUNCH (a failed code bit must not open the way)
    if ((sec <= SEC_LN || secBad || closed(SW_DOG_L) || loadedKnown()) && t != DP_LAUNCH) { if (const char* e = launchExitCheck()) return e; }
    if (w.running()) return "BUSY";
    if (!startDrumMove(t, false)) return "POSITION_UNKNOWN";
    aimed = false; tokenLive = false;
    state = S_DRUM; stT = now;
    return nullptr;
  }
  // in LAUNCH, at the stop point, drum still: the dogs are fully engaged (a drum left off centre needs "mode LAUNCH")
  bool launchCentred() const { return collarLaunch() && !d.running() && fabsf(drumDeg - c.targetDeg[DP_LAUNCH]) <= 6; }
  const char* cmdAim(double deg) {
    if (const char* e = idleCheck(true)) return e;
    if (state == S_HOLD) return "HOLDING";
    if (!launchCentred()) return "NOT_IN_LAUNCH";
    if (phase == PH_UNKNOWN) return "PHASE_UNKNOWN";
    if (c.aimNeedsRest && phase != PH_REST) return "NOT_AT_REST";
    float lo = fmaxf(c.pitchMin, pitchRange(false)), hi = fminf(c.pitchMax, pitchRange(true));
    if (!(deg >= lo - 1e-3 && deg <= hi + 1e-3)) return "OUT_OF_RANGE";
    // the table's ends are turning points (no sign change to find): aim a quarter tolerance inside them
    float q = c.aimTolDeg * 0.25f; if (deg < lo + q) deg = lo + q; if (deg > hi - q) deg = hi - q;
    aimTarget = (float)deg; aimed = false; tokenLive = false; aimTries = 0; sub = 0;
    if (indexMode) aimKnown = false;                 // D12: re-zero on SW_RAIL_LOW before every aim (the index count drifts)
    if (!aimKnown) {
      if (!fitted(SW_RAIL_LOW)) return "AIM_UNKNOWN";
      homing = true; opFrom = enc.deg(); w.c.tripA = c.aimTripA; w.want(c.aimDir, c.aimDuty);   // the edge is placed by sinceEdge(): full speed is fine
    } else {
      homing = false;
      if (!planAim()) return "OUT_OF_RANGE";
    }
    markMoving(MV_WIPER); state = S_AIM; stT = opT = now;
    return nullptr;
  }
  bool planAim() {
    float t = aimTravelTo(aimTarget);
    if (t < 0) return false;
    opFrom = enc.deg(); opTravel = t;
    if (t <= c.aimCoastDeg * 0.5f) { opTravel = 0; return true; }
    w.c.tripA = c.aimTripA; w.want(c.aimDir, t < c.aimSlowDeg ? c.aimSlowDuty : c.aimDuty);
    return true;
  }
  const char* cmdCock() {
    if (const char* e = idleCheck(true)) return e;
    if (state == S_HOLD) return "HOLDING";
    if (!launchCentred()) return "NOT_IN_LAUNCH";
    if (!fitted(SW_HOME)) return "NO_SENSOR";
    if (phase == PH_COCKED) { if (!closed(SW_HOME)) { evCocked(true); return nullptr; } return "SWITCH_DISAGREE"; }
    if (phase == PH_REST && !closed(SW_HOME)) return "SWITCH_DISAGREE";
    if (phase == PH_UNKNOWN && !closed(SW_HOME)) return "LOADED_UNKNOWN";   // carriage drawn, angle unknown: cal decock first
    if (phase == PH_UNKNOWN && fitted(SW_GAP) && !closed(SW_GAP)) return "LOADED_UNKNOWN";   // the creep needs the gear in its gap too (a lying SW_HOME)
    if (c.requireSeated && !closed(SW_SEATED)) return fitted(SW_SEATED) ? "NOT_SEATED" : "NO_SENSOR";   // lens B I2
    // SW_HOME says home but the last known angle says drawn (before the release): the switch is lying, a creep could fire
    if (phase == PH_UNKNOWN && phaseHint > c.homeOpenDeg + c.homeWinDeg && phaseHint < c.releaseDeg + c.releaseWinDeg) return "LOADED_UNKNOWN";
    homeSeen = false; tokenLive = false;
    if (phase == PH_UNKNOWN) opFrom = enc.deg();      // recovery: creep forward until SW_HOME opens (first tooth = reference)
    else { if (phase == PH_REST && fitted(SW_GAP) && !closed(SW_GAP)) return "SWITCH_DISAGREE"; phase = PH_COCKING; }   // REST needs the gear in its gap
    w.c.tripA = c.cockTripA;
    w.want(cockDir(), phase == PH_UNKNOWN ? c.cockSlowDuty : c.cockDuty);
    markMoving(MV_WIPER); state = S_COCK; stT = opT = now;
    return nullptr;
  }
  void evCocked(bool ok) { JOut o(evBuf, sizeof evBuf); evBegin(o, "COCKED"); o.boolean("ok", ok); o.num("cock_deg", cockPhase, 1); evEnd(o); }
  // everything "fire" needs, except the token
  const char* fireCheck() const {
    if (const char* e = idleCheck(true)) return e;
    if (state == S_HOLD) return "HOLDING";
    if (!launchCentred()) return "NOT_IN_LAUNCH";
    if (!wrapsKnown || wraps > 0) return "WRAPS_ON";
    if (!fitted(SW_HOME)) return "NO_SENSOR";
    // cocked (carriage drawn) or at rest (carriage home: fire then runs the whole cycle: draw, release, reset)
    bool gapOk = !fitted(SW_GAP) || (phase == PH_REST ? closed(SW_GAP) : !closed(SW_GAP));
    if (!((phase == PH_COCKED && !closed(SW_HOME)) || (phase == PH_REST && closed(SW_HOME))) || !gapOk) return "NOT_COCKED";
    if (!aimKnown || !aimed || isnan(aimTarget) || fabsf(pitch() - aimTarget) > c.aimTolDeg) return "NOT_AIMED";
    if (c.requireSeated && !closed(SW_SEATED)) return fitted(SW_SEATED) ? "NOT_SEATED" : "NO_SENSOR";
    return nullptr;
  }
  const char* cmdArm(uint32_t* tok) {
    if (const char* e = fireCheck()) return e;
    uint32_t r = io.rand32 ? io.rand32() : (now * 2654435761u) ^ 0x5bd1e995u;
    token = (r % 900000u) + 100000u;              // six digits, never 0
    tokenT = now; tokenLive = true; tokenExpired = false; *tok = token;
    return nullptr;
  }
  const char* cmdFire(double tok) {
    if (!tokenLive) return tokenExpired ? "TOKEN_EXPIRED" : "NOT_ARMED";
    if (now - tokenT > c.armMs) { tokenLive = false; tokenExpired = false; return "TOKEN_EXPIRED"; }
    tokenExpired = false;
    if (tok != (double)token) { tokenLive = false; return "BAD_TOKEN"; }   // a wrong token burns the arm
    tokenLive = false;
    if (const char* e = fireCheck()) return e;
    releaseSeen = false; relPeakA = 0; homeSeen = phase == PH_COCKED; phase = PH_FIRING; opFrom = cockPhase;
    w.c.tripA = c.cockTripA;                         // the spring load peaks at the release (D8: 6.5 A while cocking)
    w.want(cockDir(), c.fireDuty); markMoving(MV_WIPER); state = S_FIRE; stT = opT = now;
    return nullptr;
  }
  // wrap (D22): from LAUNCH or NEUTRAL through the WRAP sector (the cup lays the wraps, collar NEUTRAL) to the WRAPPED park
  // (314, collar still NEUTRAL). "mode REEL" then flies the collar into REEL.
  const char* cmdWrap() {
    if (const char* e = idleCheck(false)) return e;
    if (state == S_HOLD) return "HOLDING";
    if (wrapsDoubt) return "WRAPS_ON";               // drum code doubted near the wrap window: unwrap first
    // in N_TO_R with the angle a guess (boot after a cut): run to the 314 park (the mover re-finds the WRAP edge if needed)
    bool lockGuess = sec == SEC_LOCK && !degTrusted;
    if ((sec == SEC_LOCK && !lockGuess) || sec == SEC_REEL) { JOut o(evBuf, sizeof evBuf); evBegin(o, "WRAPPED"); o.boolean("ok", true); o.integer("wraps", wraps); evEnd(o); return nullptr; }
    if (sec == SEC_INVALID || secBad) return "POSITION_UNKNOWN";
    if (sec <= SEC_LN || closed(SW_DOG_L) || loadedKnown()) { if (const char* e = launchExitCheck()) return e; }   // D22: wrap straight from LAUNCH
    if (handMode()) return "HAND_MODE";
    if (loadedKnown()) return "LAUNCH_BUSY";
    if (w.running()) return "BUSY";
    if (!enc.ok && sec == SEC_LAUNCH) return "ANGLE_SENSOR";   // the centering jog needs the wiper angle
    if (!startDrumMove(DP_WRAPPED, false)) return "POSITION_UNKNOWN";   // park at WRAPPED (314): collar NEUTRAL, cup locked
    aimed = false; tokenLive = false;
    state = S_WRAP; stT = now;
    return nullptr;
  }
  // unwrap: centering jog, drum back to NEUTRAL; the wiper turns forward slowly in the WRAP sector (take-up only)
  const char* cmdUnwrap() {
    if (const char* e = idleCheck(false)) return e;
    if (state == S_HOLD) return "HOLDING";
    if (sec == SEC_NEUTRAL || sec == SEC_PREP) {
      if (sec == SEC_NEUTRAL) { wrapsDoubt = false; wrapsKnown = true; JOut o(evBuf, sizeof evBuf); evBegin(o, "UNWRAPPED"); o.boolean("ok", true); evEnd(o); evMode(true); return nullptr; }   // always UNWRAPPED then MODE (BASE_SPEC 10.1 step 16)
    } else if (sec < SEC_PREP || sec == SEC_INVALID || secBad) return sec == SEC_INVALID || secBad ? "POSITION_UNKNOWN" : "NOT_WRAPPED";
    if (w.running()) return "BUSY";
    if (!enc.ok) return "ANGLE_SENSOR";              // the take-up needs the wiper
    if (handMode()) return "HAND_MODE";
    if (loadedKnown()) return "LAUNCH_BUSY";
    if (!startDrumMove(DP_NEUTRAL, false)) return "POSITION_UNKNOWN";
    state = S_UNWRAP; stT = now;
    return nullptr;
  }
  // reel: in REEL the capstan winds (tow, retrieve). In NEUTRAL only the take-up turns (forward, at most 40 percent):
  // the final approach until SW_SEATED (DART_STUCK after 20 s) or the tidy before wrapping (until "slip").
  bool takeupOnly = false; uint32_t approachMs = 20000;
  // final approach time limit (DART_STUCK; a snag cannot be seen in the current, physics): "max_s" 5..600, default
  // 10 s + 1.5 x line_m / slowest take-up speed at the duty (physics), at least 20 s; 30 s with line_m unknown; at most 600 s
  bool approachLimit(const JObj& o, double duty) {
    double ms;
    if (o.num("max_s", &ms)) { if (!(ms >= 5 && ms <= 600)) return false; approachMs = (uint32_t)(ms * 1000); return true; }
    float d = (float)duty; if (d * 255 < c.reelMinDuty) d = c.reelMinDuty / 255.f; if (d * 255 > c.takeupMaxDuty) d = c.takeupMaxDuty / 255.f;
    float v = c.takeupMpsPerDuty * d - c.takeupMpsOff; if (v < 0.02f) v = 0.02f;
    float sec = lineKnown ? fmaxf(20.f, 10.f + 1.5f * lineM / v) : c.approachMaxMs / 1000.f; if (sec > 600) sec = 600;
    approachMs = (uint32_t)(sec * 1000); return true;
  }
  // D34: the tow trip sits above the current the limit needs, I = (limit + 10.4) / 17 + I_free, with a 1 A margin;
  // never below tow_trip_a (6.5 A, D33) and never above the hard trip
  // D34: the tow trip sits 1 A above the current max_n needs, I = (max_n - tension_offset_n) / k_n_per_a + free_a; never
  // below the 5 A reel trip, never above the hard trip
  float towTrip() const { float need = (maxN - c.tensOffsetN) / c.kNperA + c.freeA + 1.0f; return fminf(fmaxf(c.tripA, need), c.hardTripA); }
  const char* cmdReel(double duty, double mxn, const char* u) {
    if (const char* e = idleCheck(true)) return e;
    int un = !u || !strcmp(u, "stop") ? U_STOP : !strcmp(u, "seated") ? U_SEATED : !strcmp(u, "tension") ? U_TENSION : !strcmp(u, "slip") ? U_SLIP : -1;
    if (un < 0) return "BAD_ARG";
    if (!(duty > 0 && duty <= 1)) return "BAD_ARG";
    if (!(mxn > 0)) return "BAD_ARG";
    bool neutral = (sec == SEC_NEUTRAL || (sec == SEC_LOCK && drumDeg < c.collarNdeg)) && !closed(SW_DOG_L) && !closed(SW_DOG_R) && !secBad;
    if (!collarReel() && !neutral) return "NOT_WRAPPED";
    if (neutral && un == U_TENSION) return "NOT_WRAPPED";   // no tension without the capstan
    if (!neutral && un == U_SLIP) return "BAD_ARG";
    if (un == U_SEATED && sec != SEC_NEUTRAL) return "WRAPS_ON";   // D35 / D39: seat the dart only in NEUTRAL with the wraps off   // D35: never seat the dart on the capstan: unwrap first
    if (mxn > c.limitMaxN) return "BAD_ARG";          // D39: tow limit at most 70 N
    if (un == U_SEATED && !fitted(SW_SEATED)) return "NO_SENSOR";
    // a tow re-sent while holding at (or near) its limit stays in HOLD: re-pulling from the held line ratchets it up
    if (state == S_HOLD && un == U_TENSION && until == U_TENSION && tensionN >= 0.95f * (float)mxn) { maxN = (float)mxn; enterHold("already at the limit"); return nullptr; }
    takeupOnly = neutral;
    until = un; maxN = (float)mxn; reelDutyCmd = (int)lround(duty * 255); if (reelDutyCmd < c.reelMinDuty) reelDutyCmd = c.reelMinDuty;
    if (takeupOnly && reelDutyCmd > c.takeupMaxDuty) reelDutyCmd = c.takeupMaxDuty;
    if (until == U_SEATED && closed(SW_SEATED)) { w.halt(now, io.wiper); state = S_IDLE; evSeated(); return nullptr; }
    holdLimit = false; slipT = now;
    w.c.tripA = takeupOnly ? c.takeupTripA : until == U_TENSION ? towTrip() : c.tripA;   // D8 / D33 / D34
    w.want(c.reelDir, reelDutyCmd); markMoving(MV_WIPER); state = S_REEL; stT = opT = now;
    return nullptr;
  }
  uint32_t slipT = 0, tidyUntil = 0;
  void evSeated() { lineM = 0; lineKnown = true; JOut o(evBuf, sizeof evBuf); evBegin(o, "SEATED"); evEnd(o); dirty = true; }
  void enterHold(const char* why) {
    if (tRaw > tCur) { tCur = tRaw; if (!hxLive()) tensionN = tCur; }   // hold the higher estimate (raw cut: the average lags)
    state = S_HOLD; stT = now; holdLimit = true;
    JOut o(evBuf, sizeof evBuf); evBegin(o, "LIMIT"); o.num("tension_n", tensionN, 1); o.num("max_n", maxN, 1); o.str("why", why);
    o.boolean("holding", until == U_TENSION); evEnd(o);
    if (until != U_TENSION) state = S_IDLE;
    dirty = true;
  }
  const char* cmdJog(const char* motor, double dir, double ms) {
    bool wiper = motor && !strcmp(motor, "wiper"), drum = motor && !strcmp(motor, "drum");
    if (!wiper && !drum) return "BAD_ARG";
    if (!(dir == 1 || dir == -1)) return "BAD_ARG";
    if (!(ms > 0)) return "BAD_ARG";
    if (ms > c.jogMaxMs) ms = c.jogMaxMs;
    int dd = (int)dir;
    if (state == S_JOG && jogMotor == (wiper ? 1 : 2) && jogDir == dd) { jogUntil = now + (uint32_t)ms; return nullptr; }   // deadman refresh
    if (const char* e = idleCheck(wiper)) return e;
    if (state == S_HOLD) return "HOLDING";
    if (wiper) {
      if (d.running()) return "BUSY";
      int p = drumPos();
      if (sec == SEC_LN || sec == SEC_LOCK || sec == SEC_INVALID || secBad) return "SHIFTING";   // dogs half engaged or position unknown
      if (closed(SW_DOG_L) && closed(SW_DOG_R)) return "SWITCH_DISAGREE";
      if ((p == DP_LAUNCH || closed(SW_DOG_L)) && !launchCentred()) return "NOT_IN_LAUNCH";
      if (sec >= SEC_NEUTRAL && sec <= SEC_WRAP && closed(SW_DOG_L)) return "SWITCH_DISAGREE";
      if (loadedKnown() && p != DP_LAUNCH) return "LAUNCH_BUSY";   // code says away from LAUNCH but the spring is drawn
      if (p == DP_LAUNCH && dd == cockDir() && (phase == PH_COCKED || phase == PH_UNKNOWN || phase == PH_FIRING || phase == PH_RESETTING)) return "WOULD_FIRE";
      if (p == DP_LAUNCH && dd == cockDir() && !fitted(SW_HOME)) return "NO_SENSOR";
      if (p == DP_LAUNCH && dd == c.aimDir && phase != PH_REST) return "NOT_AT_REST";   // the drawn spring would unwind: cal decock
      if ((sec >= SEC_REEL || closed(SW_DOG_R)) && dd != c.reelDir) return "BACKSTOP";  // sprag #2: the capstan never turns out
      if (p != DP_LAUNCH && dd != c.reelDir) return "BACKSTOP";   // BASE_SPEC 10: backward only in LAUNCH (fights the bus one-way)
      bool takeup = sec >= SEC_NEUTRAL && sec <= SEC_WRAP && dd == c.reelDir;              // I4: take-up only, at most 40 percent
      w.c.tripA = takeup ? c.takeupTripA : (p == DP_LAUNCH && dd == cockDir()) ? c.cockTripA : c.jogTripA; w.want(dd, takeup && c.jogDuty > c.takeupMaxDuty ? c.takeupMaxDuty : c.jogDuty); markMoving(MV_WIPER);
      if (p == DP_LAUNCH && dd == c.aimDir) aimed = false;
    } else {
      if (w.running()) return "BUSY";
      if (sec == SEC_INVALID || secBad) return "POSITION_UNKNOWN";
      if (handMode()) return "HAND_MODE";
      if ((sec <= SEC_LN || loadedKnown()) && !(sec == SEC_LAUNCH && dd < 0)) { if (const char* e = launchExitCheck()) return e; }   // never shift under load
      if ((sec == SEC_REEL || sec == SEC_LOCK) && dd < 0) return "WRAPS_ON";   // down from N_TO_R / REEL only by unwrap (with the take-up)
      if (sec == SEC_LOCK && dd > 0) return "SHIFTING";    // into REEL only by mode REEL (the fly-in)
      if (sec == SEC_WRAP || wrapsDoubt) return "WRAPS_ON";
      if (!degTrusted && ((sec == SEC_LAUNCH && dd < 0) || (sec == SEC_REEL && dd > 0))) return "POSITION_UNKNOWN";   // end stop guard needs the angle   // inside the wrap window only wrap / unwrap (with the take-up) move the drum
      if ((closed(SW_DOG_L) && sec > SEC_LN) || (closed(SW_DOG_R) && sec < SEC_LOCK)) return "SWITCH_DISAGREE";
      d.want(dd * c.drumSign); markMoving(MV_DRUM); aimed = false;
    }
    tokenLive = false;
    jogLaunch = wiper && drumPos() == DP_LAUNCH;
    jogMotor = wiper ? 1 : 2; jogDir = dd; jogUntil = now + (uint32_t)ms; state = S_JOG; stT = now;
    return nullptr;
  }
  const char* cmdCal(const char* what, const JObj& o) {
    if (!what) return "BAD_ARG";
    if (!strcmp(what, "current")) {
      if (w.running()) return "BUSY";
      float off = w.c.offsetA + w.amps; if (off < 0 || off > 5) return "OUT_OF_RANGE";   // same range as set offset_a
      w.c.offsetA = off; w.amps = 0; return nullptr;
    }
    if (!strcmp(what, "tare")) { if (!hxSeen) return "NO_SENSOR"; if (moving() || state == S_HOLD) return "BUSY"; hxOff += hxN / c.hxNPerCount; hxN = 0; return nullptr; }
    if (!strcmp(what, "free")) {
      if (!w.running() || now - w.startT < 800) return "NOT_RUNNING";
      if (state == S_HOLD || (state == S_REEL && until == U_TENSION)) return "BUSY";   // free current only while reeling slack
      if (w.amps > 10) return "OUT_OF_RANGE";   // same range as set free_a
      c.freeA = w.amps; return nullptr;
    }
    if (!strcmp(what, "aim")) {
      double m; if (!o.num("deg", &m)) return "BAD_ARG";
      if (!aimKnown) return "AIM_UNKNOWN";
      float tab = pitchAt(aimPhase) - aimOffset; float off = (float)m - tab;
      if (!(off >= -20 && off <= 20)) return "OUT_OF_RANGE";   // same range as set aim_offset
      aimOffset = off; aimed = false; return nullptr;
    }
    if (!strcmp(what, "aim_home")) {                  // declare the aim train at its home (operator checked)
      if (moving()) return "BUSY";
      aimPhase = c.aimHomeDeg; aimKnown = true; aimed = false; dirty = true; return nullptr;
    }
    if (!strcmp(what, "aim_forget")) { if (moving()) return "BUSY"; aimKnown = false; aimed = false; dirty = true; return nullptr; }   // next aim re-homes
    if (!strcmp(what, "cock_rest")) {                 // declare the launch train at rest (carriage forward, uncocked)
      if (moving()) return "BUSY";
      bool force = false; o.boolean("force", &force);  // force: SW_HOME known broken, the operator checked by eye
      if (fitted(SW_HOME) && !closed(SW_HOME) && !force) return "SWITCH_DISAGREE";
      if (fitted(SW_GAP) && !closed(SW_GAP) && !force) return "SWITCH_DISAGREE";   // big gear must sit in its gap
      phase = PH_REST; cockPhase = 0; phaseHint = -1; dirty = true; return nullptr;
    }
    if (!strcmp(what, "decock")) {                    // carriage drawn with the angle unknown (or a cocked shot called off):
      if (const char* e = idleCheck(true)) return e;  // run BACKWARD so the spring unwinds under the worm until SW_HOME closes.
      if (detentClosed() != DP_LAUNCH) return "NOT_IN_LAUNCH";   // backward also turns the aim train: aim must be redone
      if (!fitted(SW_HOME)) return "NO_SENSOR";
      bool force = false; o.boolean("force", &force);
      decockByTravel = false;
      if (closed(SW_HOME)) {
        if (phase == PH_REST) { JOut e(evBuf, sizeof evBuf); evBegin(e, "RESET"); e.boolean("ok", true); e.str("how", "decock"); evEnd(e); return nullptr; }
        if (phase == PH_UNKNOWN && !(phaseHint > c.homeOpenDeg + c.homeWinDeg && phaseHint < c.releaseDeg + c.releaseWinDeg)) {
          // carriage home, springs relaxed: nothing to unwind; the next cock finds the angle. Still answer with an event
          if (fitted(SW_GAP) && closed(SW_GAP)) {      // carriage home and the big gear in its gap: the springs are relaxed
            phase = PH_REST; cockPhase = 0; phaseHint = -1; dirty = true;
            JOut e(evBuf, sizeof evBuf); evBegin(e, "RESET"); e.boolean("ok", true); e.str("how", "decock"); e.str("why", "SW_HOME and SW_GAP closed: at rest"); evEnd(e); return nullptr;
          }
          JOut e(evBuf, sizeof evBuf); evBegin(e, "RESET"); e.boolean("ok", false); e.str("how", "decock"); e.str("why", "carriage home: nothing to decock, cock finds the angle"); evEnd(e); return nullptr;
        }
        if (!force) return "SWITCH_DISAGREE";        // the spring should be drawn but SW_HOME says home: lying switch
        decockByTravel = true;                       // forced: run back by the travel the angle says, then call it rest
        decockTravel = (phaseHint > 0 ? phaseHint : c.cockEndDeg) - c.homeOpenDeg + c.dogWindowDeg + 30;   // window, then the train
      }
      opFrom = enc.deg(); homeSeen = false; homeT = now; aimKnown = false; aimed = false; tokenLive = false;
      w.c.tripA = c.aimTripA; w.want(c.aimDir, c.decockDuty); markMoving(MV_WIPER); state = S_DECOCK; stT = opT = now; return nullptr;
    }
    if (!strcmp(what, "drum")) {                      // the drum code is absolute: report where it is
      if (sec == SEC_INVALID || secBad) return "POSITION_UNKNOWN";
      evMode(true); return nullptr;
    }
    return "BAD_ARG";
  }
  // settable keys: name, pointer, min, max
  struct Key { const char* k; float* f; float lo, hi; };
  int keys(Key* out) {
    int n = 0;
    out[n++] = {"limit_n", &c.limitN, 1, c.limitMaxN};
    out[n++] = {"k_n_per_a", &c.kNperA, 0.1f, 200};
    out[n++] = {"tension_offset_n", &c.tensOffsetN, -50, 50};
    out[n++] = {"free_a", &c.freeA, 0, 10};
    out[n++] = {"amps_per_volt", &w.c.ampsPerVolt, 1, 200};
    out[n++] = {"offset_a", &w.c.offsetA, 0, 5};
    out[n++] = {"hx_n_per_count", &c.hxNPerCount, 1e-7f, 1};
    out[n++] = {"drum_dps", &c.drumDps, 5, 720};
    out[n++] = {"aim_tol", &c.aimTolDeg, 0.1f, 5};
    out[n++] = {"aim_offset", &aimOffset, -20, 20};
    out[n++] = {"line_m", &lineM, 0, 100};
    out[n++] = {"trip_a", &c.tripA, 3, 19};
    out[n++] = {"cock_trip_a", &c.cockTripA, 3, 19};
    out[n++] = {"tow_trip_a", &c.towTripA, 3, 19};
    out[n++] = {"vbat_low", &c.vbatLow, 6, 12};
    out[n++] = {"aim_home_deg", &c.aimHomeDeg, -7200, 7200};
    out[n++] = {"host_timeout_s", &c.hostTimeoutS, 0, 600};
    return n;
  }
  const char* cmdSet(const char* k, double v) {
    if (!k) return "BAD_ARG";
    if (!strcmp(k, "require_seated")) { c.requireSeated = v != 0; return nullptr; }
    if (!strcmp(k, "vbat_wired")) { c.vbatWired = v != 0; return nullptr; }
    if (!strcmp(k, "enc_sign")) { if (v != 1 && v != -1) return "BAD_ARG"; if (moving()) return "BUSY"; enc.sign = (int)v; enc.count = -enc.count; lastWiperDeg = enc.deg(); return nullptr; }
    Key ks[20]; int n = keys(ks);
    for (int i = 0; i < n; i++) if (!strcmp(ks[i].k, k)) {
      if (!(v >= ks[i].lo - 1e-6 * fabs(ks[i].lo) && v <= ks[i].hi + 1e-6 * fabs(ks[i].hi))) return "OUT_OF_RANGE";
      // the current and tension constants and the trips never change under a running motor or a held tow
      static const char* const live[] = {"k_n_per_a", "tension_offset_n", "free_a", "amps_per_volt", "offset_a", "hx_n_per_count", "trip_a", "cock_trip_a", "tow_trip_a"};
      for (const char* l : live) if (!strcmp(k, l) && (moving() || state == S_HOLD)) return "BUSY";
      *ks[i].f = (float)v;
      if (!strcmp(k, "line_m")) { lineKnown = true; dirty = true; }
      return nullptr;
    }
    return "BAD_KEY";
  }

  // ================================================================== protocol
  // one line in, exactly one reply out (into rep). Long actions reply when accepted; completion is an event.
  void handleLine(const char* line, int len, char* rep, int cap) {
    if (cap > JSON_MAX_LINE + 1) cap = JSON_MAX_LINE + 1;   // C1: a reply is at most 256 bytes too
    hostT = now;
    JOut r(rep, cap);
    JObj o;
    if (len > JSON_MAX_LINE) { snprintf(rep, cap, "{\"id\":null,\"ok\":false,\"err\":\"TOO_LONG\"}"); return; }
    if (!jsonParse(line, len, &o)) { snprintf(rep, cap, "{\"id\":null,\"ok\":false,\"err\":\"BAD_JSON\"}"); return; }
    r.open();
    const JVal* id = o.find("id");
    if (!id || id->t == J_NULL || id->t == J_SKIP || id->t == J_BOOL) r.null("id");
    else if (id->t == J_NUM) {
      if (!(id->n == floor(id->n) && fabs(id->n) < 9007199254740992.0)) {
        const char* c0 = nullptr;                    // a stop is always obeyed, whatever the id looks like
        if (o.str("cmd", &c0) && (!strcmp(c0, "stop") || !strcmp(c0, "estop"))) cmdStop(!strcmp(c0, "estop"));
        snprintf(rep, cap, "{\"id\":null,\"ok\":false,\"err\":\"BAD_ID\"}"); return;
      }
      char t[24]; snprintf(t, sizeof t, "%.0f", id->n); r.keyRaw("id"); r.raw(t);
    }
    else r.str("id", id->s);
    const char* cmd = nullptr;
    if (!o.str("cmd", &cmd)) { r.boolean("ok", false); r.str("err", "NO_CMD"); r.close(); return; }
    const char* err = nullptr; bool extra = false;
    double a = 0, b = 0, m = 0; const char* s = nullptr; const char* s2 = nullptr;
    uint32_t tok = 0; bool tokOut = false;
    if (!strcmp(cmd, "ping")) {}
    else if (!strcmp(cmd, "status")) { extra = true; }
    else if (!strcmp(cmd, "stop")) cmdStop(false);
    else if (!strcmp(cmd, "estop")) cmdStop(true);
    else if (!strcmp(cmd, "clear")) err = cmdClear();
    else if (!strcmp(cmd, "mode")) err = o.str("to", &s) ? cmdMode(s) : "BAD_ARG";
    else if (!strcmp(cmd, "aim")) err = o.num("deg", &a) ? cmdAim(a) : "BAD_ARG";
    else if (!strcmp(cmd, "arm")) { err = cmdArm(&tok); tokOut = !err; }
    else if (!strcmp(cmd, "fire")) err = o.num("token", &a) ? cmdFire(a) : (o.str("token", &s) ? cmdFire(atof(s)) : "BAD_ARG");
    else if (!strcmp(cmd, "cock")) err = cmdCock();
    else if (!strcmp(cmd, "wrap")) err = cmdWrap();
    else if (!strcmp(cmd, "unwrap")) err = cmdUnwrap();
    else if (!strcmp(cmd, "reel")) {
      if (!o.num("duty", &a)) a = c.reelDuty / 255.0;
      if (!o.num("max_n", &m)) m = c.limitN;
      o.str("until", &s2);
      if (!approachLimit(o, a)) err = "BAD_ARG"; else err = cmdReel(a, m, s2);
    }
    else if (!strcmp(cmd, "jog")) { if (!o.num("ms", &b)) b = 300; err = (o.str("motor", &s) && o.num("dir", &a)) ? cmdJog(s, a, b) : "BAD_ARG"; }
    else if (!strcmp(cmd, "tidy")) {               // D22: take-up only, forward by time (collar NEUTRAL)
      if (!o.num("ms", &b)) b = c.tidyMs; if (!o.num("duty", &a)) a = 0.3;
      bool neutral = (sec == SEC_NEUTRAL || (sec == SEC_LOCK && drumDeg < c.collarNdeg)) && !closed(SW_DOG_L) && !closed(SW_DOG_R);
      if (!(b > 0 && b <= 60000)) err = "BAD_ARG";
      else if (sec == SEC_INVALID || secBad) err = "POSITION_UNKNOWN";
      else if (!neutral) err = "NOT_IN_NEUTRAL";
      else { err = cmdReel(a, c.limitN, "stop"); if (!err) { until = U_TIME; tidyUntil = now + (uint32_t)b; } }
    }
    else if (!strcmp(cmd, "retrieve")) {           // final approach until SW_SEATED on the take-up (D35: refused with the wraps on)
      if (!o.num("duty", &a)) a = 0.3; if (!o.num("max_n", &m)) m = c.limitN;
      err = approachLimit(o, a) ? cmdReel(a, m, "seated") : "BAD_ARG";
    }
    else if (!strcmp(cmd, "decock")) { err = cmdCal("decock", o); }
    else if (!strcmp(cmd, "cal") && bootPending && !(o.str("what", &s) && !strcmp(s, "drum"))) err = "BOOTING";   // resolveBoot() would undo it
    else if (!strcmp(cmd, "set") && bootPending) err = "BOOTING";
    else if (!strcmp(cmd, "cal")) { err = o.str("what", &s) ? cmdCal(s, o) : "BAD_ARG"; if (!err) tunDirty = true; }
    else if (!strcmp(cmd, "set")) { bool bv; if (o.str("key", &s) && (o.num("val", &a) || (o.boolean("val", &bv) && ((a = bv ? 1 : 0), true)))) err = cmdSet(s, a); else err = "BAD_ARG"; if (!err) tunDirty = true; }
    else if (!strcmp(cmd, "get")) {
      if (!o.str("key", &s)) err = "BAD_ARG";
      else if (!getKey(s, r)) err = "BAD_KEY";
    }
    else err = "UNKNOWN_CMD";
    r.boolean("ok", err == nullptr);
    if (err) r.str("err", err);
    if (tokOut) { r.integer("token", (long)tok); r.integer("valid_ms", (long)c.armMs); }
    if (!err && !strcmp(cmd, "ping")) { r.str("fw", GB_FW_VERSION); r.integer("t", (long)now); }
    if (extra) statusFields(r);
    r.close();
    if (r.bad) {                                     // too long (a very long string id): id, ok and err only
      JOut m(rep, cap); m.open();
      if (id && id->t == J_STR) { char t[24]; strncpy(t, id->s, 23); t[23] = 0; m.str("id", t); }
      else if (id && id->t == J_NUM) m.num("id", id->n, 0); else m.null("id");
      m.boolean("ok", err == nullptr); if (err) m.str("err", err); else if (tokOut) { m.integer("token", (long)tok); }
      if (extra && !err) statusFields(m, false);     // status without the fault text first
      m.close();
      if (m.bad && extra) { JOut q(rep, cap); q.open();
        if (id && id->t == J_STR) { char t[24]; strncpy(t, id->s, 23); t[23] = 0; q.str("id", t); }
        else if (id && id->t == J_NUM) q.num("id", id->n, 0); else q.null("id");
        q.boolean("ok", true); q.close(); m.bad = q.bad; }
      if (m.bad) snprintf(rep, cap, "{\"id\":null,\"ok\":false,\"err\":\"REPLY_OVERFLOW\"}");
    }
  }
  bool getKey(const char* k, JOut& r) {
    if (!strcmp(k, "fw")) { r.str("val", GB_FW_VERSION); return true; }
    if (!strcmp(k, "params")) {
#ifdef P_VERSION
      r.str("val", P_VERSION);
#else
      r.str("val", "none");
#endif
      return true;
    }
    if (!strcmp(k, "require_seated")) { r.boolean("val", c.requireSeated); return true; }
    if (!strcmp(k, "vbat_wired")) { r.boolean("val", c.vbatWired); return true; }
    if (!strcmp(k, "enc_sign")) { r.integer("val", enc.sign); return true; }
    if (!strcmp(k, "pitch_range")) { r.num("min", fmaxf(c.pitchMin, pitchRange(false)), 2); r.num("max", fminf(c.pitchMax, pitchRange(true)), 2); return true; }
    Key ks[20]; int n = keys(ks);
    for (int i = 0; i < n; i++) if (!strcmp(ks[i].k, k)) { char t[24]; snprintf(t, sizeof t, "%.7g", (double)*ks[i].f); r.keyRaw("val"); r.raw(t); return true; }
    return false;
  }
  void statusFields(JOut& r, bool full = true) {
    r.str("phase", PHASE_NAME[phase]); r.boolean("aimed", aimed); r.boolean("aim_known", aimKnown);
    r.boolean("drum_known", drumKnown); r.boolean("wraps_known", wrapsKnown);
    r.str("magnet", indexMode ? "INDEX" : !enc.ok ? "NONE" : enc.magnetWeak() ? "WEAK" : enc.magnetStrong() ? "STRONG" : "OK");
    r.integer("glitches", (long)(enc.glitches > 99999 ? 99999 : enc.glitches)); r.boolean("hx", hxLive()); r.num("peak_a", w.peakA, 1);
    r.str("fw", GB_FW_VERSION); r.integer("ev_drops", (long)(evDrops > 99999 ? 99999 : evDrops));   // events lost (host not reading)
    if (fault && full) { char wy[33]; strncpy(wy, faultWhy, 32); wy[32] = 0; r.str("why", wy); }
  }
  void cmdStop(bool estop) {
    w.halt(now, io.wiper); d.halt(now, io.drum); tokenLive = false; homing = false;
    int was = state;
    if (state == S_FAULT || state == S_SAFING) { state = S_FAULT; }
    else {
      if (was != S_IDLE) evAborted(STATE_NAME[was], estop ? "estop" : "stop");
      if (was == S_COCK || was == S_FIRE || was == S_DECOCK) { phaseHint = phase == PH_UNKNOWN ? fmaxf(phaseHint, cockPhase) : cockPhase; phase = PH_UNKNOWN; }   // stopped mid-cycle: re-establish (cock or cal decock)
      state = S_IDLE;
    }
    if (estop && state != S_FAULT) {
      mvSafing = 1;
      const char* hold = safeToNeutral(F_NONE);
      int st = safeTarget();
      if (!hold && !atPark(st) && startDrumMove(st, true)) { state = S_DRUM; stT = now; }
      else if (hold && !atPark(st)) evHeld(hold);
      else if (atPark(st)) evMode(true);
    }
    dirty = true;
  }
  const char* cmdClear() {
    if (state != S_FAULT) return state == S_SAFING ? "BUSY" : nullptr;
    if (fault == F_ANGLE_SENSOR && !enc.ok) return "STILL_PRESENT";
    if (fault == F_DRUM_SENSOR && (secBad || sec == SEC_INVALID)) return "STILL_PRESENT";
    if (fault == F_BROWNOUT && !isnan(vbat) && vbat < c.vbatLow) return "STILL_PRESENT";
    w.clearLatches(); fault = F_NONE; faultWhy[0] = 0; state = S_IDLE; stT = now;
    return nullptr;
  }

  // ================================================================== telemetry
  // C1 limits a line to 256 bytes. Normal lines are about 200. If a line would be longer (only in absurd states), it is
  // rebuilt with fewer decimals, and if still too long a short form with "trunc": true is sent.
  void telemetry(char* buf, int cap) {
    if (cap > 257) cap = 257;
    if (telBuild(buf, cap, false)) return;
    if (telBuild(buf, cap, true)) return;
    JOut o(buf, cap); o.open(); o.obj("tel"); o.str("state", STATE_NAME[state]); o.str("mode", modeName());
    o.str("drum_pos", DRUM_POS_NAME[drumPos()]); if (fault) o.str("fault", FAULT_NAME[fault]); else o.null("fault");
    o.boolean("trunc", true); o.close(); o.close();
  }
  // lean form (only when the full line would pass 256 bytes): fewer decimals and null fields left out (absent = null)
  bool telBuild(char* buf, int cap, bool lean) {
    int d1 = lean ? 0 : 1, d2 = lean ? 1 : 2;
    JOut o(buf, cap); o.open(); o.obj("tel");
    auto nul = [&](const char* k) { if (!lean) o.null(k); };
    o.str("state", STATE_NAME[state]); o.str("mode", modeName());
    if (drumKnown) o.num("drum_deg", drumDeg, d1); else nul("drum_deg");
    o.str("drum_pos", DRUM_POS_NAME[drumPos()]);
    if (enc.ok || wiperInit) o.num("wiper_deg", enc.deg(), d1); else nul("wiper_deg");
    if (enc.ok) o.num("rpm", enc.rpm(), d1); else nul("rpm");
    o.num("amps", amps, d1);
    float p = pitch(); if (isnan(p)) nul("pitch_deg"); else o.num("pitch_deg", p, d2);
    float ck = phase == PH_REST ? 0 : phase == PH_COCKED || phase == PH_FIRING ? 1 : phase == PH_COCKING ? fminf(1, fmaxf(0, cockPhase / c.cockEndDeg)) :
               phase == PH_RESETTING ? fmaxf(0, 1 - (cockPhase - c.releaseDeg) / fmaxf(1, c.cycleDeg - c.releaseDeg)) : NAN;
    if (isnan(ck)) nul("cock"); else o.num("cock", ck, 2);
    if (wrapsKnown) o.integer("wraps", wraps); else nul("wraps");
    if (lineKnown) o.num("line_m", lineM, d2); else nul("line_m");
    if (hxLive()) o.num("tension_n", hxN, d1); else nul("tension_n");
    o.obj("sw");                                     // closed contacts only (absent = open), short keys: PROTOCOL.md
    for (int i = 0; i < SW_COUNT; i++) if (closed(i)) o.integer(SW_TEL[i], 1);
    o.close();
    if (fault) o.str("fault", FAULT_NAME[fault]); else nul("fault");
    if (isnan(vbat)) nul("vbat"); else o.num("vbat", vbat, 1);
    if (lean) o.boolean("lean", true);
    o.close(); o.close();
    return !o.bad;
  }
  // the sketch calls this from its loop: true when a telemetry line is due (10 Hz idle, 20 Hz moving)
  bool telDue(uint32_t t) {
    uint32_t per = moving() ? c.telFastMs : c.telSlowMs;
    if (t - telT >= per) { telT = t; return true; }
    return false;
  }

  // ================================================================== the tick (every 5 ms)
  void tick(uint32_t t) {
    uint32_t dt = booted ? t - now : c.tickMs; now = t;
    if (dt > 100) dt = 100;
    readSensors(dt);
    if (bootPending) {
      d.tick(now, dt, io.drum, io.drumSleep);
      if (now - bootT >= 60) resolveBoot();
      return;
    }
    trackWiper();
    trackDrum(dt);
    watch();
    switch (state) {
      case S_DRUM: case S_SAFING: tickDrumState(); break;
      case S_AIM: tickAim(); break;
      case S_COCK: tickCock(); break;
      case S_FIRE: tickFire(); break;
      case S_WRAP: tickWrap(); break;
      case S_UNWRAP: tickUnwrap(); break;
      case S_REEL: tickReel(); break;
      case S_HOLD: tickHold(); break;
      case S_JOG: tickJog(); break;
      case S_DECOCK: tickDecock(); break;
      default: break;
    }
    d.tick(now, dt, io.drum, io.drumSleep, startOk());
    // guard: in a resting state no motor may run (a missed halt anywhere is caught here within one tick)
    if (state == S_IDLE || state == S_HOLD || state == S_FAULT) {
      if (w.wantDir || w.running()) w.halt(now, io.wiper);
      if (d.wantDir || d.running()) d.halt(now, io.drum);
    }
    // keep the multi-turn count small (float precision): whole turns only, the angle inside a turn is unchanged
    if (!w.running() && (enc.count > 4096 * 1000 || enc.count < -4096 * 1000)) {
      int32_t k = enc.count / (4096 * 1000) * (4096 * 1000); enc.count -= k; enc.rateRef -= k; lastWiperDeg -= k * 360.f / 4096.f;
    }
    // persistence: once everything is still for 300 ms, write the final positions (moving = 0)
    bool still = !w.running() && !d.running() && (state == S_IDLE || state == S_HOLD || state == S_FAULT);
    if (!still) quietT = now;
    if (still && now - quietT >= 300 && (movingBits || dirty)) { movingBits = 0; dirty = false; persist(0); }
    if (still && tunDirty) { tunDirty = false; Tun t = tunNow();   // write only what changed (flash wear)
      if (memcmp(&t, &lastTun, sizeof t) != 0) { lastTun = t; if (io.saveTun) io.saveTun(t); } }
  }

  void tickDrumState() {
    int r = tickDrumMove();
    if (r == 0) return;
    if (r > 0) {
      if (state == S_SAFING) { state = S_FAULT; stT = now; evMode(true); dirty = true; return; }
      state = S_IDLE; stT = now; evMode(true); dirty = true;
    }
  }
  void tickAim() {
    float trav = (enc.deg() - opFrom) * c.aimDir;
    if (homing) {
      if (rose(SW_RAIL_LOW)) {
        w.halt(now, io.wiper); aimPhase = wrapCyc(c.aimHomeDeg + sinceEdge(SW_RAIL_LOW, c.aimDir), c.aimCycleDeg); aimKnown = true; win = 0; homing = false; sub = 3; legT = now; return;
      }
      if (trav > c.aimCycleDeg * 1.25f + c.aimLashDeg) { raise(F_BAD_STATE, "SW_RAIL_LOW never closed in an aim cycle"); return; }
      return;
    }
    if (sub == 3) {                                  // just homed: wait for the shaft to stop, then plan
      if (now - legT < 150) return;
      sub = 0; if (!planAim()) { w.halt(now, io.wiper); state = S_IDLE; evAimed(false, "OUT_OF_RANGE"); return; }
      return;
    }
    if (sub == 4) {                                  // stopped: settle then check
      if (now - legT < 150) return;
      float err = pitch() - aimTarget;
      if (fabsf(err) <= c.aimTolDeg) { aimed = true; state = S_IDLE; stT = now; evAimed(true, nullptr); dirty = true; return; }
      if (++aimTries >= 3) { state = S_IDLE; evAimed(false, "AIM_MISS"); dirty = true; return; }
      sub = 0; if (!planAim()) { state = S_IDLE; evAimed(false, "OUT_OF_RANGE"); return; }
      return;
    }
    float rem = opTravel - trav;
    if (rem <= c.aimCoastDeg) { w.halt(now, io.wiper); sub = 4; legT = now; return; }
    w.want(c.aimDir, rem < c.aimSlowDeg ? c.aimSlowDuty : c.aimDuty);
    if (trav > c.aimMaxDeg) raise(F_BAD_STATE, "aim ran too far");
  }
  void evAimed(bool ok, const char* err) {
    JOut o(evBuf, sizeof evBuf); evBegin(o, "AIMED"); o.boolean("ok", ok);
    float p = pitch(); if (isnan(p)) o.null("pitch_deg"); else o.num("pitch_deg", p, 2);
    o.num("target_deg", aimTarget, 2); if (err) o.str("err", err); evEnd(o);
  }
  void tickCock() {
    if (phase == PH_UNKNOWN) {                       // recovery run: until SW_HOME opens (the first tooth takes the rack)
      float trav = (enc.deg() - opFrom) * cockDir();
      if (fell(SW_HOME)) { cockPhase = c.homeOpenDeg + sinceEdge(SW_HOME, cockDir()); win = c.dogWindowDeg;   /* the train moved: collar on the forward flank */ phase = PH_COCKING; homeSeen = true; w.want(cockDir(), c.cockDuty); return; }
      // the train is either short of the first tooth or resetting after a release: SW_HOME must open within the reset
      // gap plus the first tooth. Any further and SW_HOME is lying: stop well before the release tooth could come round
      if (trav > c.cycleDeg - c.releaseDeg + c.homeOpenDeg + c.homeWinDeg + 20) { raise(F_BAD_STATE, "SW_HOME never opened (stuck closed?)"); return; }
      if (now - opT > c.cockMaxMs * 2) { raise(F_BAD_STATE, "cock recovery took too long"); return; }
      return;
    }
    if (fell(SW_HOME)) {                             // re-zero on the first tooth
      float at = cockPhase - sinceEdge(SW_HOME, cockDir());
      if (fabsf(at - c.homeOpenDeg) > c.homeWinDeg) { raise(F_SWITCH_DISAGREE, "SW_HOME opened at the wrong angle"); return; }
      cockPhase = c.homeOpenDeg + sinceEdge(SW_HOME, cockDir()); win = c.dogWindowDeg;   /* the train moved: collar on the forward flank */ homeSeen = true;
    }
    if (!homeSeen && closed(SW_HOME) && cockPhase > c.homeOpenDeg + c.homeWinDeg) { raise(F_BAD_STATE, "carriage not drawn (SW_HOME still closed): rack or dog not engaged"); return; }
    if (homeSeen && closed(SW_HOME) && cockPhase < c.releaseDeg - c.releaseWinDeg) { raise(F_BAD_STATE, "carriage slipped home while cocking"); return; }
    if (cockPhase >= c.cockEndDeg - c.aimCoastDeg) {
      w.halt(now, io.wiper);
      w.c.tripA = c.tripA;
      if (closed(SW_HOME)) { raise(F_BAD_STATE, "SW_HOME closed at the end of cocking"); return; }
      phase = PH_COCKED; phaseHint = -1; state = S_IDLE; stT = now; evCocked(true); dirty = true; return;
    }
    w.want(cockDir(), cockPhase >= c.cockSlowFrom ? c.cockSlowDuty : c.cockDuty);
    if (now - opT > c.cockMaxMs) raise(F_BAD_STATE, "cocking took too long");
  }
  void tickFire() {
    if (phase == PH_FIRING) {
      if (w.amps > relPeakA) relPeakA = w.amps;
      if (!homeSeen) {                               // fired from rest: the draw first, re-zeroed on the first tooth
        if (fell(SW_HOME)) {
          float at = cockPhase - sinceEdge(SW_HOME, cockDir());
          if (fabsf(at - c.homeOpenDeg) > c.homeWinDeg) { raise(F_SWITCH_DISAGREE, "SW_HOME opened at the wrong angle"); return; }
          cockPhase = c.homeOpenDeg + sinceEdge(SW_HOME, cockDir()); win = c.dogWindowDeg;   /* the train moved: collar on the forward flank */ homeSeen = true;
        } else if (closed(SW_HOME) && cockPhase > c.homeOpenDeg + c.homeWinDeg) { raise(F_BAD_STATE, "carriage not drawn (SW_HOME still closed): rack or dog not engaged"); return; }
        if (now - opT > c.fireMaxMs) raise(F_BAD_STATE, "fire cycle took too long");
        return;
      }
      if (rose(SW_HOME)) {                           // the relieved tooth let the rack go: carriage home = shot
        float at = cockPhase - sinceEdge(SW_HOME, cockDir());
        releaseSeen = true; phase = PH_RESETTING; aimed = false; lineKnown = false;
        JOut o(evBuf, sizeof evBuf); evBegin(o, "FIRED"); float p = pitch(); if (isnan(p)) o.null("pitch_deg"); else o.num("pitch_deg", p, 2);
        o.num("cock_deg", at, 1); evEnd(o);
        if (fabsf(at - c.releaseDeg) > c.releaseWinDeg * 2 + (indexMode ? 60 : 0)) { raise(F_BAD_STATE, "release at the wrong angle"); return; }
        if (indexMode) cockPhase = c.releaseDeg + sinceEdge(SW_HOME, cockDir());   // re-zero on the release
        w.c.tripA = c.tripA; w.want(cockDir(), c.resetDuty);
        return;
      }
      // index mode counts in 60 deg steps and errs high (a cock then stops early, never late): allow one more step
      if (cockPhase > c.releaseDeg + c.releaseWinDeg + (indexMode ? 60 : 0)) { raise(F_BAD_STATE, "no release seen (SW_HOME did not close)"); return; }
    } else if (phase == PH_RESETTING) {
      if (rose(SW_GAP) && cockPhase > c.releaseDeg) cockPhase = c.gapCloseDeg + sinceEdge(SW_GAP, cockDir());   // re-zero entering the gap
      if (!closed(SW_HOME) && cockPhase < c.cycleDeg + c.homeOpenDeg - c.homeWinDeg) { raise(F_BAD_STATE, "SW_HOME opened while resetting"); return; }
      if (cockPhase >= c.cycleDeg - c.aimCoastDeg) {
        if (fitted(SW_HOME) && !closed(SW_HOME)) { raise(F_BAD_STATE, "reset ended with the carriage off its stop"); return; }
        if (fitted(SW_GAP) && !closed(SW_GAP)) { raise(F_BAD_STATE, "reset ended with the big gear out of its gap"); return; }
        w.halt(now, io.wiper); cockPhase -= c.cycleDeg; if (cockPhase < 0) cockPhase = 0;
        phase = PH_REST; state = S_IDLE; stT = now;
        JOut o(evBuf, sizeof evBuf); evBegin(o, "RESET"); o.boolean("ok", true); evEnd(o); dirty = true; return;
      }
    }
    if (now - opT > c.fireMaxMs) raise(F_BAD_STATE, "fire cycle took too long");
  }
  void tickWrap() {
    int r = tickDrumMove();
    if (r > 0) { state = S_IDLE; stT = now; JOut o(evBuf, sizeof evBuf); evBegin(o, "WRAPPED"); o.boolean("ok", true); o.integer("wraps", wraps); evEnd(o); dirty = true; }
  }
  void tickUnwrap() {
    int r = tickDrumMove();
    if (r > 0) { state = S_IDLE; stT = now; wrapsDoubt = false; wrapsKnown = true; JOut o(evBuf, sizeof evBuf); evBegin(o, "UNWRAPPED"); o.boolean("ok", true); evEnd(o); evMode(true); dirty = true; }
  }
  void tickReel() {
    if (until == U_SEATED && rose(SW_SEATED)) { w.halt(now, io.wiper); state = S_IDLE; stT = now; evSeated(); return; }
    if (takeupOnly) {                                // NEUTRAL: take-up only
      if (until == U_SEATED && now - opT > approachMs) { char b[48]; snprintf(b, sizeof b, "final approach: no SW_SEATED in %u s", (unsigned)(approachMs / 1000)); raise(F_DART_STUCK, b); return; }
      if (until == U_TIME && (int32_t)(now - tidyUntil) >= 0) {
        w.halt(now, io.wiper); state = S_IDLE; stT = now; JOut o(evBuf, sizeof evBuf); evBegin(o, "TIDIED"); o.boolean("ok", true); evEnd(o); dirty = true; return;
      }
      if (until == U_SLIP) {                         // tidy: the take-up slips at a steady current once the line is straight
        if (fabsf(w.amps - tidyA) > 0.3f || now - w.startT < 500) { tidyA = w.amps; slipT = now; }
        if (now - slipT >= c.tidySteadyMs) { w.halt(now, io.wiper); state = S_IDLE; stT = now;
          JOut o(evBuf, sizeof evBuf); evBegin(o, "LIMIT"); o.null("tension_n"); o.num("max_n", maxN, 1); o.str("why", "tidy done (take-up slipping)"); o.boolean("holding", false); evEnd(o); dirty = true; return; }
        if (now - opT > c.tidyMaxMs) { w.halt(now, io.wiper); state = S_IDLE; evAborted("REEL", "timeout"); dirty = true; return; }
      }
      w.want(c.reelDir, reelDutyCmd);
      if (now - opT > c.reelMaxMs) { w.halt(now, io.wiper); state = S_IDLE; evAborted("REEL", "timeout"); dirty = true; }
      return;
    }
    // the 200 ms average (D33) or the raw estimate held at the limit for rawHoldMs (a stiff line rises in tens of ms)
    if (tRaw < maxN) rawOverT = now;
    float frac = maxN > 0 ? fmaxf(tensionN, tRaw) / maxN : 0;
    bool over = tensionN >= maxN || now - rawOverT >= c.rawHoldMs;
    if (over && now - w.startT > c.tensionIgnoreMs && now - stT > c.tensionIgnoreMs) { w.halt(now, io.wiper); enterHold("tension limit"); return; }
    int duty = reelDutyCmd;
    if (frac > 1 - c.taperFrac) { int dd = (int)(reelDutyCmd * (1 - frac) / c.taperFrac); duty = dd < c.reelMinDuty ? c.reelMinDuty : dd; }
    if (until == U_SEATED && lineKnown && lineM < c.seatSlowM && duty > c.seatSlowDuty) duty = c.seatSlowDuty;
    w.want(c.reelDir, duty);
    if (now - opT > c.reelMaxMs) { w.halt(now, io.wiper); state = S_IDLE; evAborted("REEL", "timeout"); dirty = true; }
  }
  float tidyA = 0;
  void tickHold() {
    if (until == U_TENSION && hxLive() && hxN < c.resumeFrac * maxN && now - stT > 1000) {   // load cell: line slackened, wind again
      holdLimit = false; w.want(c.reelDir, reelDutyCmd); markMoving(MV_WIPER); state = S_REEL; stT = now;
    }
  }
  void tickJog() {
    if (jogMotor == 1 && jogLaunch && jogDir == cockDir()) {   // the release guard follows where the jog started
      if (fell(SW_HOME) && phase == PH_REST) {
        float at = cockPhase - sinceEdge(SW_HOME, cockDir());
        if (fabsf(at - c.homeOpenDeg) > c.homeWinDeg + c.dogWindowDeg) { raise(F_SWITCH_DISAGREE, "SW_HOME opened at the wrong angle"); return; }
        phase = PH_COCKING; cockPhase = c.homeOpenDeg + sinceEdge(SW_HOME, cockDir()); win = c.dogWindowDeg;   // the train moved: collar on the forward flank
      }
      if (cockPhase >= c.cockEndDeg - c.releaseWinDeg) {
        w.halt(now, io.wiper); state = S_IDLE; evAborted("JOG", "would pass the release tooth"); dirty = true; return;
      }
    }
    if (jogMotor == 2) {                              // stay off the groove ends (hard stops)
      int dirL = jogDir * c.drumSign; (void)dirL;
      if ((jogDir < 0 && sec == SEC_LAUNCH && drumDeg <= c.secLoDeg + 2) || (jogDir > 0 && sec == SEC_REEL && drumDeg >= c.secHiDeg - 2)) {
        d.halt(now, io.drum); state = S_IDLE; stT = now; evAborted("JOG", "drum end stop"); dirty = true; return;
      }
      // the collar contacts are checked during every drum move, jogs included
      if ((closed(SW_DOG_L) && sec > SEC_LN) || (closed(SW_DOG_R) && sec < SEC_LOCK)) { raise(F_SWITCH_DISAGREE, "collar contact disagrees with the drum code"); return; }
      // a jog never enters the wrap window: "wrap" and "unwrap" (with the take-up) cross it
      if (sec == SEC_WRAP) { d.halt(now, io.drum); state = S_IDLE; stT = now; evAborted("JOG", "wrap window: use wrap or unwrap"); dirty = true; return; }
    }
    if ((int32_t)(now - jogUntil) >= 0) {
      if (jogMotor == 1) w.halt(now, io.wiper); else d.halt(now, io.drum);
      state = S_IDLE; stT = now; dirty = true;
    }
  }
  void tickDecock() {
    float trav = (enc.deg() - opFrom) * c.aimDir;
    if (decockByTravel) {
      if (now - opT > c.cockMaxMs * 2) { raise(F_BAD_STATE, "decock by travel took too long"); return; }
      if (trav >= decockTravel) { w.halt(now, io.wiper); phase = PH_REST; cockPhase = 0; phaseHint = -1; state = S_IDLE; stT = now; dirty = true;
        JOut o(evBuf, sizeof evBuf); evBegin(o, "RESET"); o.boolean("ok", true); o.str("how", "decock by travel"); evEnd(o); }
      return;
    }
    if (chat(SW_HOME)) { raise(F_SWITCH_DISAGREE, "SW_HOME chattering during decock"); return; }
    // SW_HOME must stay closed over the whole run-on: a bounce is not "home"
    if (closed(SW_HOME)) { if (!homeSeen) { homeSeen = true; homeFrom = enc.deg(); } }
    else homeSeen = false;
    if (homeSeen && (enc.deg() - homeFrom) * c.aimDir >= c.decockExtraDeg && (!fitted(SW_GAP) || closed(SW_GAP))) {   // BASE_SPEC 10: SW_HOME and SW_GAP
      // the carriage stopped at its front stop as SW_HOME closed; after that sprag #1 freewheels, so the big gear sits
      // just short of the first-tooth angle
      w.halt(now, io.wiper); phase = PH_REST; cockPhase = c.homeOpenDeg - 2; phaseHint = -1; state = S_IDLE; stT = now; dirty = true;
      JOut o(evBuf, sizeof evBuf); evBegin(o, "RESET"); o.boolean("ok", true); o.str("how", "decock"); evEnd(o); return;
    }
    if (trav > c.cockEndDeg * 1.2f + c.dogWindowDeg + 90 || now - opT > c.cockMaxMs * 2) { raise(F_BAD_STATE, homeSeen ? "decock: SW_GAP never closed" : "decock: SW_HOME never closed"); return; }
  }
  float homeFrom = 0;
};

}  // namespace gb
