// GrappStar base station logic: one wiper motor (BTS7960 + AS5600) and the N20 selector drum (DRV8833).
// Plain C++ (no Arduino calls): the sketch supplies the hardware through Io, the PC tests supply a simulation.
// Protocol: PROTOCOL.md (contract C1, JSON lines over USB CDC). Mechanism: base/spec/BASE_SPEC.md.
//
// Drum travel (one line): LAUNCH -- NEUTRAL -- REEL -- wrap zone (wrap arm orbits the capstan, one wrap per orbit).
// The order is physical: the capstan cannot be wrapped in launch mode, the mode cannot change with wraps on.
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

#define GB_FW_VERSION "base-1.0"

enum Fault { F_NONE, F_STALL, F_OVERCURRENT, F_DRUM_TIMEOUT, F_DRUM_SENSOR, F_ANGLE_SENSOR, F_SWITCH_DISAGREE, F_BROWNOUT, F_BAD_STATE };
static const char* const FAULT_NAME[] = {nullptr, "STALL", "OVERCURRENT", "DRUM_TIMEOUT", "DRUM_SENSOR", "ANGLE_SENSOR",
                                         "SWITCH_DISAGREE", "BROWNOUT", "BAD_STATE"};

enum State { S_IDLE, S_DRUM, S_AIM, S_COCK, S_FIRE, S_WRAP, S_UNWRAP, S_REEL, S_HOLD, S_JOG, S_CALDRUM, S_SAFING, S_FAULT, S_DECOCK };
static const char* const STATE_NAME[] = {"IDLE", "MODE", "AIM", "COCK", "FIRE", "WRAP", "UNWRAP", "REEL", "HOLD", "JOG",
                                         "CAL", "SAFING", "FAULT", "DECOCK"};

enum Phase { PH_REST, PH_COCKING, PH_COCKED, PH_FIRING, PH_RESETTING, PH_UNKNOWN };
static const char* const PHASE_NAME[] = {"REST", "COCKING", "COCKED", "FIRING", "RESETTING", "UNKNOWN"};

enum Until { U_STOP, U_SEATED, U_TENSION };

struct Io {
  void (*wiper)(int dir, int duty) = nullptr;     // BTS7960: dir +1 RPWM, -1 LPWM, 0 both low (brake); duty 0..255
  void (*drum)(int dir, int duty) = nullptr;      // DRV8833 channel A: dir +1 / -1, 0 = brake (both inputs high)
  void (*drumSleep)(bool awake) = nullptr;        // nSLEEP
  int (*angle)(uint8_t* status) = nullptr;        // AS5600 raw 0..4095 or -1 on an I2C error; *status = STATUS register
  float (*wiperIsVolts)() = nullptr;              // volts at the IS pin
  float (*vbat)() = nullptr;                      // battery volts, NAN if not wired
  uint8_t (*switches)() = nullptr;                // bit i = contact on PIN_SW_(A+i) closed
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
  float drumDeg = 0; bool drumKnown = false; int wraps = 0; bool wrapsKnown = true;
  // current operation
  uint32_t opT = 0, legT = 0; float opFrom = 0, opTravel = 0; int sub = 0;
  int jogMotor = 0, jogDir = 0; uint32_t jogUntil = 0;
  int until = U_STOP; float maxN = 20; int reelDutyCmd = 220; bool holdLimit = false;
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
  char evBuf[260];

  // ================================================================== setup
  void begin(uint32_t t) {
    now = bootT = stT = t; telT = t;
    enc.sign = c.wiperSign; sw.debounceMs = c.debounceMs;
    w.c.tripA = c.tripA;
    loadAimTable();
  }
  void loadAimTable() {
#ifdef P_AIM_TABLE_N
    aimN = P_AIM_TABLE_N > AIM_TAB_MAX ? AIM_TAB_MAX : P_AIM_TABLE_N;
    for (int i = 0; i < aimN; i++) { aimW[i] = P_AIM_TABLE_WIPER[i]; aimP[i] = P_AIM_TABLE_PITCH[i]; }
    c.aimCycleDeg = P_AIM_CYCLE_DEG;
#else
    // placeholder until params.py has AIM: a crank-rocker like curve, pitch min at phase 0, max at half a cycle
    aimN = 73;
    for (int i = 0; i < aimN; i++) {
      float ph = c.aimCycleDeg * i / (aimN - 1);
      aimW[i] = ph; aimP[i] = c.pitchMin + (c.pitchMax - c.pitchMin) * 0.5f * (1 - cosf(ph * (float)M_PI / 180.f * 360.f / c.aimCycleDeg));
    }
#endif
  }
  Tun tunNow() const {
    Tun t; t.encSign = enc.sign > 0 ? 1 : 0; t.requireSeated = (c.requireSeated ? 1 : 0) | (c.vbatWired ? 0 : 2);
    t.ampsPerVolt = w.c.ampsPerVolt; t.offsetA = w.c.offsetA; t.freeA = c.freeA; t.kNperA = c.kNperA; t.hxNPerCount = c.hxNPerCount;
    t.hxOff = hxOff; t.limitN = c.limitN; t.drumDps = c.drumDps; t.aimOffset = aimOffset; t.aimTol = c.aimTolDeg; t.tripA = c.tripA;
    t.cockTripA = c.cockTripA; t.vbatLow = c.vbatLow; t.aimHomeDeg = c.aimHomeDeg; sealTun(&t); return t;
  }
  // the sketch calls this before begin() with what saveTun() stored (or nullptr)
  void restoreTun(const Tun* t) {
    if (!t || !validTun(t)) return;
    enc.sign = c.wiperSign = t->encSign ? 1 : -1; c.requireSeated = t->requireSeated & 1; c.vbatWired = !(t->requireSeated & 2); w.c.ampsPerVolt = t->ampsPerVolt; w.c.offsetA = t->offsetA;
    c.freeA = t->freeA; c.kNperA = t->kNperA; c.hxNPerCount = t->hxNPerCount; hxOff = t->hxOff; c.limitN = t->limitN; c.drumDps = t->drumDps;
    aimOffset = t->aimOffset; c.aimTolDeg = t->aimTol; c.tripA = t->tripA; c.cockTripA = t->cockTripA; c.vbatLow = t->vbatLow; c.aimHomeDeg = t->aimHomeDeg;
  }
  bool tunDirty = false;
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
  static int targetSec(int t) { return t == DP_LAUNCH ? SEC_LAUNCH : t == DP_NEUTRAL ? SEC_NEUTRAL : t == DP_REEL ? SEC_REEL : SEC_WRAPPED; }
  int detentClosed() const {          // DP_LAUNCH / DP_NEUTRAL / DP_REEL when the drum is in that sector, else -1
    return sec == SEC_LAUNCH ? DP_LAUNCH : sec == SEC_NEUTRAL ? DP_NEUTRAL : sec == SEC_REEL ? DP_REEL : -1;
  }
  int drumPos() const {
    switch (sec) {
      case SEC_LAUNCH: return DP_LAUNCH; case SEC_NEUTRAL: return DP_NEUTRAL; case SEC_REEL: return DP_REEL;
      case SEC_WRAPPED: return DP_WRAPPED; case SEC_INVALID: return DP_UNKNOWN; default: return DP_BETWEEN;
    }
  }
  const char* modeName() const {
    if (sec == SEC_LAUNCH) return "LAUNCH"; if (sec == SEC_NEUTRAL) return "NEUTRAL";
    if (sec == SEC_REEL || sec == SEC_WRAPPING || sec == SEC_WRAPPED) return "REEL";   // collar stays in REEL over the dwell
    return "NONE";
  }
  int wrapsAt(int s, float deg) const {
    if (s == SEC_WRAPPED) return c.wrapsFull;
    if (s != SEC_WRAPPING) return 0;
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
    if (phase == PH_COCKING || phase == PH_FIRING || phase == PH_RESETTING) phase = PH_UNKNOWN;   // re-established by cock / decock
    tokenLive = false; homing = false;
    { JOut o(evBuf, sizeof evBuf); evBegin(o, "FAULT"); o.str("code", FAULT_NAME[f]); o.str("why", why); evEnd(o); }
    if (f == F_STALL) evSimple("STALL");
    // any fault: wiper off (done), drum to NEUTRAL when that is safe, otherwise the drum holds (worm self-locks)
    const char* hold = safeToNeutral(f);
    if (!hold && detentClosed() != DP_NEUTRAL) { if (startDrumMove(DP_NEUTRAL, true)) { state = S_SAFING; return; } hold = "drum move failed"; }
    d.halt(now, io.drum);
    state = S_FAULT; stT = now;
    if (hold && detentClosed() != DP_NEUTRAL) evHeld(hold);
    else if (detentClosed() == DP_NEUTRAL) evMode(true);   // already safe
    dirty = true;
  }
  void evHeld(const char* why) {
    JOut o(evBuf, sizeof evBuf); evBegin(o, "MODE"); o.boolean("ok", false); o.str("mode", modeName()); o.str("drum_pos", DRUM_POS_NAME[drumPos()]);
    o.str("err", "DRUM_HELD"); o.str("why", why); evEnd(o);
  }
  void extFault(int f, const char* why) { if (state != S_FAULT && state != S_SAFING && booted) raise(f, why); }
  // nullptr if the drum may go to NEUTRAL now, else why it must hold
  const char* safeToNeutral(int f) const {
    if (f == F_DRUM_TIMEOUT || f == F_DRUM_SENSOR) return "drum fault";
    if (sec == SEC_INVALID || secBad) return "drum code invalid";
    if (!wrapsKnown) return "wraps unknown";
    if (sec >= SEC_WRAPPING || wraps > 0) return "wraps on: the capstan holds the line";
    if (drumPos() == DP_LAUNCH && phase != PH_REST)
      return "launch train not at rest: the wiper worm holds the spring (a shift under load could dry fire)";
    return nullptr;
  }

  // ================================================================== persistence
  Saved snapshot(uint8_t movingMask) const {
    Saved s; s.moving = movingMask; s.drumKnown = drumKnown; s.drumDeg = drumDeg; s.wraps = (int16_t)wraps; s.wrapsKnown = wrapsKnown;
    s.wiperKnown = enc.ok || wiperInit; s.wiperDeg = enc.deg(); s.aimPhase = aimPhase; s.aimKnown = aimKnown; s.cockPhase = cockPhase;
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
  void resolveBoot() {
    bootPending = false; booted = true;
    const Saved* s = bootValid ? &boot : nullptr;
    bool wiperTrust = s && !(s->moving & MV_WIPER) && s->wiperKnown;
    bool drumTrust = s && !(s->moving & MV_DRUM);
    // the drum code is absolute: the sector is known whatever happened; the angle inside it comes from the record
    // when the drum was not moving and the record agrees with the sector, else the sector middle
    if (sec != SEC_INVALID) {
      drumKnown = true;
      if (drumTrust && s->drumKnown && s->drumDeg >= secLo(sec) && s->drumDeg <= secHi(sec)) drumDeg = s->drumDeg;
      else drumDeg = 0.5f * (fmaxf(secLo(sec), c.targetDeg[0]) + fminf(secHi(sec), c.targetDeg[3]));
      wraps = wrapsAt(sec, drumDeg); wrapsKnown = true;
    } else { drumKnown = false; wrapsKnown = false; wraps = 0; }
    if (s && s->lineM >= 0) { lineM = s->lineM; lineKnown = true; } else { lineKnown = false; }
    // wiper: the AS5600 knows the angle inside one turn; the record knows the turns, if nothing moved since
    bool wiperOk = false;
    if (enc.ok && wiperTrust) {
      float diff = wrap180(enc.absDeg() - wrap360(s->wiperDeg));
      if (fabsf(diff) < 3.0f) { enc.setDeg(s->wiperDeg + diff); wiperOk = true; }
    }
    if (enc.ok) wiperInit = true;
    lastWiperDeg = enc.deg();
    if (wiperOk) {
      aimPhase = s->aimPhase; aimKnown = s->aimKnown; aimOffset = aimOffset;
      phase = s->cockKnown ? (int)s->phase : (int)PH_UNKNOWN; cockPhase = s->cockPhase;
      if (phase == PH_COCKING || phase == PH_FIRING || phase == PH_RESETTING) phase = PH_UNKNOWN;   // never trust a mid-cycle stop
    } else { aimKnown = false; phase = PH_UNKNOWN; }
    // SW_HOME (carriage forward) must agree with the record: rest = closed, cocked = open
    if (fitted(SW_HOME)) {
      if (phase == PH_REST && !closed(SW_HOME)) phase = PH_UNKNOWN;
      if (phase == PH_COCKED && closed(SW_HOME)) phase = PH_UNKNOWN;
    }
    JOut o(evBuf, sizeof evBuf); evBegin(o, "BOOT"); o.str("fw", GB_FW_VERSION); o.boolean("record", s != nullptr);
    o.boolean("was_moving", s && s->moving); o.boolean("drum_known", drumKnown); o.boolean("wraps_known", wrapsKnown);
    o.boolean("aim_known", aimKnown); o.str("phase", PHASE_NAME[phase]); evEnd(o);
    movingBits = 0; persist(0);
  }

  // ================================================================== sensors
  void readSensors(uint32_t dt) {
    uint8_t st = 0; int raw = io.angle ? io.angle(&st) : -1;
    enc.feed(raw, st, dt);
    sw.update(io.switches ? io.switches() : 0, now, enc.deg());
    updateSector();
    float isv = io.wiperIsVolts ? io.wiperIsVolts() : 0;
    w.tick(now, dt, isv, enc.ok ? enc.rateDps : NAN, io.wiper, startOk());   // ramp, gap, current, stall latches
    amps = w.amps;
    vbat = io.vbat && c.vbatWired ? io.vbat() : NAN;
    float hx = io.hxRaw ? io.hxRaw() : NAN;
    if (!isnan(hx)) { hxN = (hx - hxOff) * c.hxNPerCount; hxT = now; hxSeen = true; }
    bool winding = w.outDir == c.reelDir && (drumPos() == DP_WRAPPED || drumPos() == DP_REEL) && now - w.startT > c.tensionIgnoreMs;
    float tc = winding ? (w.amps - c.freeA) * c.kNperA : 0; if (tc < 0) tc = 0;
    tCur = winding ? tCur + (tc - tCur) * 0.2f : (state == S_HOLD ? tCur : 0);
    tensionN = hxLive() ? hxN : tCur;
  }
  bool hxLive() const { return hxSeen && now - hxT < 500; }

  // wiper motion -> trains. Only while running or just stopped (coast), only along the last drive direction.
  void trackWiper() {
    if (!enc.ok) return;
    float deg = enc.deg();
    if (!wiperInit) { lastWiperDeg = deg; wiperInit = true; return; }
    float dd = deg - lastWiperDeg; lastWiperDeg = deg;
    if (dd == 0) return;
    bool live = w.running() || (w.lastDir != 0 && now - w.stopT < 400);
    if (!live) return;
    float along = dd * w.lastDir;               // + = travel in the drive direction
    int p = drumPos();
    if (p == DP_LAUNCH) {
      if (w.lastDir == c.aimDir) {
        if (lashDir != c.aimDir) { lashLeft = c.aimLashDeg; lashDir = c.aimDir; }   // the splitter arm swings over first
        float use = along;
        if (lashLeft > 0 && use > 0) { float l = use < lashLeft ? use : lashLeft; lashLeft -= l; use -= l; }
        aimPhase = wrapCyc(aimPhase + use, c.aimCycleDeg);
      } else { cockPhase += along; lashDir = w.lastDir; }
    } else if ((p == DP_REEL || p == DP_WRAPPED) && w.lastDir == c.reelDir && wraps > 0) {
      float m = along / 360.f * c.capstanPerWiper * (float)M_PI * c.capstanDmm / 1000.f;
      lineM -= m; if (lineM < 0) lineM = 0;
    } else if (p == DP_BETWEEN || p == DP_UNKNOWN) {
      // a dog half engaged may have moved a train by an unknown amount
      if (fabsf(along) > 2) { if (aimKnown && w.lastDir == c.aimDir) aimKnown = false; }
    }
  }

  // sector from the code. Adjacent changes are edges (snap the angle estimate, learn the speed); code 000 or a jump
  // over a sector keeps the last sector and starts the DRUM_SENSOR timer (watch()).
  void updateSector() {
    int s = sectorOf(codeNow());
    if (s == SEC_INVALID || (sec != SEC_INVALID && abs(s - sec) > 1 && booted)) { if (!secBad) { secBad = true; secBadT = now; } return; }
    secBad = false;
    if (s == sec) return;
    int was = sec; prevSec = sec; sec = s;
    if (was == SEC_INVALID || !booted) { drumKnown = true; return; }
    int dir = s > was ? 1 : -1;
    float e = c.edgeDeg[s > was ? was : s];
    uint32_t te = 0; for (int id = SW_DRUM_A; id <= SW_DRUM_C; id++) if (fitted(id)) { uint32_t x = sw.edgeTime(c.swBit[id]); if (te == 0 || (int32_t)(x - te) > 0) te = x; }
    if (edgeRun && d.running() && edgeAt != e) {          // two edges in one run: real speed
      float ms = (float)(te - edgeT);
      if (ms > 100) { float v = fabsf(e - edgeAt) / (ms / 1000.f); if (v > 0.3f * c.drumDps && v < 3 * c.drumDps) c.drumDps = 0.5f * c.drumDps + 0.5f * v; }
    }
    edgeT = te; edgeAt = e; edgeRun = d.running();
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
    wraps = wrapsAt(sec, drumDeg); wrapsKnown = true;
  }

  // ================================================================== watchdogs (every tick)
  void watch() {
    if (state == S_FAULT) return;
    if (w.overcurrent) { char b[48]; snprintf(b, sizeof b, "wiper over %.0f A", w.c.tripA); w.clearLatches(); onOvercurrent(b); return; }
    if (w.stalled) { w.clearLatches(); onStall(); return; }
    if (!enc.ok && (w.running() || state == S_AIM || state == S_COCK || state == S_FIRE || state == S_REEL)) { raise(F_ANGLE_SENSOR, "AS5600 lost while the wiper runs"); return; }
    if (secBad && now - secBadT > c.invalidMs) { secBadT = now; raise(F_DRUM_SENSOR, sectorOf(codeNow()) == SEC_INVALID ? "drum code 000 (wire off or bump missing)" : "drum code jumped over a sector"); return; }
    bool drumMoveState = state == S_DRUM || state == S_SAFING || state == S_WRAP || state == S_UNWRAP;
    if (drumMoveState && (chat(SW_DRUM_A) || chat(SW_DRUM_B) || chat(SW_DRUM_C))) { raise(F_SWITCH_DISAGREE, "drum code contact chattering"); return; }
    bool running = w.running() || d.running();
    if (!isnan(vbat) && running && vbat < c.vbatLow) {
      if (!lowRun) { lowRun = true; lowT = now; } else if (now - lowT >= c.vbatLowMs) { raise(F_BROWNOUT, "battery sagged under load"); return; }
    } else lowRun = false;
    if (tokenLive && now - tokenT > c.armMs) { tokenLive = false; tokenExpired = true; }
    // optional host watchdog: no line from the Jetson for host_timeout_s while a motor runs = stop (0 = off)
    if (c.hostTimeoutS > 0 && running && now - hostT > (uint32_t)(c.hostTimeoutS * 1000)) { cmdStop(false); evAborted("HOST", "host silent"); hostT = now; }
  }
  uint32_t hostT = 0;
  uint32_t lastMultiOk = 0;
  void onOvercurrent(const char* why) { raise(F_OVERCURRENT, why); }   // always a fault (also in a tow: the limit acts first)
  void onStall() { raise(F_STALL, "wiper stalled"); }

  // ================================================================== drum moves
  // One mover for mode changes, wrap, unwrap and safing (drum_NOTES.md section 7): optional unload of the dogs, run
  // until the target sector's edge, run on the extra angle by time, check the sector, and after a shift into LAUNCH
  // or REEL wiggle the wiper while SW_DOG says the dogs sit tip to tip.
  int mvTarget = -1, mvDir = 0, mvStage = 0, mvSafing = 0; uint32_t mvT = 0, mvLegT = 0, mvSecT = 0, mvEnd = 0; float mvExpect = 0, mvFrom = 0;
  int mvLastSec = SEC_INVALID;
  bool startDrumMove(int target, bool safing) {
    if (sec == SEC_INVALID) return false;
    int ts = targetSec(target);
    mvTarget = target; mvSafing = safing; mvDir = ts > sec ? 1 : ts < sec ? -1 : (c.targetDeg[target] >= drumDeg ? 1 : -1);
    mvExpect = fabsf(c.targetDeg[target] - drumDeg) / c.drumDps * 1000.f;
    mvT = mvLegT = mvSecT = now; mvLastSec = sec; mvFrom = enc.deg();
    if (ts == sec) { mvStage = 1; mvEnd = now + (uint32_t)(fabsf(c.targetDeg[target] - drumDeg) / c.drumDps * 1000.f); }   // inside: centre
    else if (sec == SEC_LAUNCH && c.unloadDeg > 0 && enc.ok && !safing) { mvStage = 3; w.c.tripA = c.tripA; w.want(c.aimDir, c.nudgeDuty); markMoving(MV_WIPER); markMoving(MV_DRUM); return true; }
    else mvStage = 0;
    d.want(mvDir * c.drumSign); markMoving(MV_DRUM);
    return true;
  }
  uint32_t mvTimeout() const { return (uint32_t)(mvExpect * c.drumTimeFactor) + c.drumTimeMarginMs; }
  // 0 running, 1 arrived, -1 failed (fault raised, or for safing reported)
  int tickDrumMove() {
    bool safing = mvSafing;
    int ts = targetSec(mvTarget);
    if (mvStage == 3) {                               // unload the dogs: a few deg backward (sprag #1 slips at rest)
      if (fabsf(enc.deg() - mvFrom) >= c.unloadDeg || now - mvLegT > 500) { w.halt(now, io.wiper); mvStage = 0; mvT = mvLegT = mvSecT = now; d.want(mvDir * c.drumSign); }
      return 0;
    }
    if (mvStage == 0) {
      if (sec != mvLastSec) {
        if ((sec - mvLastSec) * mvDir < 0) { failDrum(safing, F_DRUM_SENSOR, "drum turned the wrong way (drum_sign?)"); return -1; }
        mvLastSec = sec; mvSecT = now;
      }
      if (sec == ts) {
        float el = (float)(now - mvT);
        if (el < c.drumEarlyFrac * mvExpect && mvExpect > 1500) { failDrum(safing, F_DRUM_SENSOR, "target code came far too early"); return -1; }
        float extra = fabsf(c.targetDeg[mvTarget] - edgeAt) / c.drumDps * 1000.f;
        uint32_t since = now - edgeT; mvEnd = now + (extra > since ? (uint32_t)(extra - since) : 0);
        mvStage = 1; return 0;
      }
      // no edge for much longer than the current sector takes to cross: the drum is not turning
      float span = secHi(sec) - secLo(sec); if (sec == SEC_LAUNCH || sec == SEC_WRAPPED) span = 30;
      if (now - mvSecT > (uint32_t)(span / c.drumDps * 1000.f * c.drumTimeFactor) + c.drumTimeMarginMs) { failDrum(safing, F_DRUM_TIMEOUT, "drum not turning (no code edge)"); return -1; }
      if (now - mvT > mvTimeout()) { failDrum(safing, F_DRUM_TIMEOUT, "drum did not reach the target in time"); return -1; }
      return 0;
    }
    if (mvStage == 1) {
      if ((int32_t)(now - mvEnd) < 0) return 0;
      d.halt(now, io.drum);
      if (sec != ts) { failDrum(safing, F_DRUM_SENSOR, "drum overshot the target sector"); return -1; }
      drumDeg = c.targetDeg[mvTarget];
      if ((ts == SEC_LAUNCH || ts == SEC_REEL) && fitted(SW_DOG) && !safing) { mvStage = 4; mvLegT = now; return 0; }
      return 1;
    }
    if (mvStage == 4) {                               // let SW_DOG settle, then wiggle if the dogs are tip to tip
      if (now - mvLegT < 100) return 0;
      if (!closed(SW_DOG)) return 1;
      mvStage = 2; mvFrom = enc.deg(); mvLegT = now;
      w.c.tripA = c.tripA; w.want(ts == SEC_LAUNCH ? c.aimDir : c.reelDir, c.dogDuty); markMoving(MV_WIPER);   // never forward in LAUNCH
      return 0;
    }
    if (mvStage == 2) {
      if (!closed(SW_DOG)) {                          // dogs dropped in
        w.halt(now, io.wiper); if (ts == SEC_LAUNCH) { aimKnown = false; aimed = false; }
        return 1;
      }
      if (fabsf(enc.deg() - mvFrom) >= c.dogMaxDeg || now - mvLegT > 4000) { w.halt(now, io.wiper); failDrum(safing, F_DRUM_TIMEOUT, "dogs stayed tip to tip (SW_DOG)"); return -1; }
      return 0;
    }
    return -1;
  }
  void failDrum(bool safing, int f, const char* why) {
    d.halt(now, io.drum); w.halt(now, io.wiper);
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
  const char* cmdMode(const char* to) {
    int t = !strcmp(to, "LAUNCH") ? DP_LAUNCH : !strcmp(to, "NEUTRAL") ? DP_NEUTRAL : !strcmp(to, "REEL") ? DP_REEL : -1;
    if (t < 0) return "BAD_ARG";
    if (const char* e = idleCheck(false)) return e;
    if (state == S_HOLD) return "HOLDING";
    if (sec == SEC_INVALID || secBad) return "POSITION_UNKNOWN";
    if (sec >= SEC_WRAPPING) return "WRAPS_ON";
    int k = detentClosed();
    if (k == t && fabsf(drumDeg - c.targetDeg[t]) < 3) { evMode(true); return nullptr; }
    // the worm holds the spring: never shift while the launch train is not at rest, unless the drum is positively
    // away from LAUNCH (a failed code bit must not open the way)
    if (phase != PH_REST && (sec <= SEC_LN || secBad) && t != DP_LAUNCH) return "LAUNCH_BUSY";
    if (w.running()) return "BUSY";
    if (!startDrumMove(t, false)) return "POSITION_UNKNOWN";
    aimed = false; tokenLive = false;
    state = S_DRUM; stT = now;
    return nullptr;
  }
  // in LAUNCH, at the stop point, drum still: the dogs are fully engaged (a drum left off centre needs "mode LAUNCH")
  bool launchCentred() const { return sec == SEC_LAUNCH && !secBad && !d.running() && fabsf(drumDeg - c.targetDeg[DP_LAUNCH]) <= 5; }
  const char* cmdAim(double deg) {
    if (const char* e = idleCheck(true)) return e;
    if (state == S_HOLD) return "HOLDING";
    if (!launchCentred()) return "NOT_IN_LAUNCH";
    if (phase == PH_UNKNOWN) return "PHASE_UNKNOWN";
    if (c.aimNeedsRest && phase != PH_REST) return "NOT_AT_REST";
    float lo = fmaxf(c.pitchMin, pitchRange(false)), hi = fminf(c.pitchMax, pitchRange(true));
    if (!(deg >= lo - 1e-3 && deg <= hi + 1e-3)) return "OUT_OF_RANGE";
    aimTarget = (float)deg; aimed = false; tokenLive = false; aimTries = 0; sub = 0;
    if (!aimKnown) {
      if (!fitted(SW_RAIL_LOW)) return "AIM_UNKNOWN";
      homing = true; opFrom = enc.deg(); w.want(c.aimDir, c.aimDuty);   // the edge is placed by sinceEdge(): full speed is fine
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
    w.want(c.aimDir, t < c.aimSlowDeg ? c.aimSlowDuty : c.aimDuty);
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
    homeSeen = false; tokenLive = false;
    if (phase == PH_UNKNOWN) opFrom = enc.deg();      // recovery: creep forward until SW_HOME opens (first tooth = reference)
    else phase = PH_COCKING;
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
    if (!((phase == PH_COCKED && !closed(SW_HOME)) || (phase == PH_REST && closed(SW_HOME)))) return "NOT_COCKED";
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
    w.c.tripA = homeSeen ? c.tripA : c.cockTripA;
    w.want(cockDir(), c.fireDuty); markMoving(MV_WIPER); state = S_FIRE; stT = opT = now;
    return nullptr;
  }
  const char* cmdWrap() {
    if (const char* e = idleCheck(false)) return e;
    if (state == S_HOLD) return "HOLDING";
    if (sec == SEC_WRAPPED) { JOut o(evBuf, sizeof evBuf); evBegin(o, "WRAPPED"); o.boolean("ok", true); o.integer("wraps", wraps); evEnd(o); return nullptr; }
    if (sec != SEC_REEL && sec != SEC_WRAPPING) return "NOT_IN_REEL";
    if (w.running()) return "BUSY";
    if (!startDrumMove(DP_WRAPPED, false)) return "POSITION_UNKNOWN";
    state = S_WRAP; stT = now;
    return nullptr;
  }
  const char* cmdUnwrap() {
    if (const char* e = idleCheck(false)) return e;
    if (state == S_HOLD) return "HOLDING";
    if (sec == SEC_REEL) { JOut o(evBuf, sizeof evBuf); evBegin(o, "UNWRAPPED"); o.boolean("ok", true); evEnd(o); return nullptr; }
    if (sec != SEC_WRAPPING && sec != SEC_WRAPPED) return "NOT_WRAPPED";
    if (w.running()) return "BUSY";
    if (!startDrumMove(DP_REEL, false)) return "POSITION_UNKNOWN";
    state = S_UNWRAP; stT = now;
    return nullptr;
  }
  const char* cmdReel(double duty, double mxn, const char* u) {
    if (const char* e = idleCheck(true)) return e;
    if (drumPos() != DP_WRAPPED || !wrapsKnown || wraps < 1) return "NOT_WRAPPED";
    int un = !u || !strcmp(u, "stop") ? U_STOP : !strcmp(u, "seated") ? U_SEATED : !strcmp(u, "tension") ? U_TENSION : -1;
    if (un < 0) return "BAD_ARG";
    if (!(duty > 0 && duty <= 1)) return "BAD_ARG";
    if (!(mxn > 0)) return "BAD_ARG";
    if (mxn > c.limitMaxN) mxn = c.limitMaxN;
    if (un == U_SEATED && !fitted(SW_SEATED)) return "NO_SENSOR";
    until = un; maxN = (float)mxn; reelDutyCmd = (int)lround(duty * 255); if (reelDutyCmd < c.reelMinDuty) reelDutyCmd = c.reelMinDuty;
    if (until == U_SEATED && closed(SW_SEATED)) { w.halt(now, io.wiper); state = S_IDLE; evSeated(); return nullptr; }
    holdLimit = false; w.c.tripA = c.tripA;
    w.want(c.reelDir, reelDutyCmd); markMoving(MV_WIPER); state = S_REEL; stT = opT = now;
    return nullptr;
  }
  void evSeated() { lineM = 0; lineKnown = true; JOut o(evBuf, sizeof evBuf); evBegin(o, "SEATED"); evEnd(o); dirty = true; }
  void enterHold(const char* why) {
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
      if (sec == SEC_LN || sec == SEC_NR || sec == SEC_INVALID || secBad) return "SHIFTING";   // dogs half engaged or position unknown
      if (p == DP_LAUNCH && !launchCentred()) return "NOT_IN_LAUNCH";
      if (p == DP_LAUNCH && dd == cockDir() && (phase == PH_COCKED || phase == PH_UNKNOWN || phase == PH_FIRING || phase == PH_RESETTING)) return "WOULD_FIRE";
      if (p == DP_LAUNCH && dd == c.aimDir && phase != PH_REST) return "NOT_AT_REST";   // the drawn spring would unwind: cal decock
      if (sec >= SEC_REEL && dd != c.reelDir) return "BACKSTOP";                        // sprag #2: the capstan never turns out
      w.c.tripA = c.tripA; w.want(dd, c.jogDuty); markMoving(MV_WIPER);
      if (p == DP_LAUNCH && dd == c.aimDir) aimed = false;
    } else {
      if (w.running()) return "BUSY";
      if (sec == SEC_INVALID || secBad) return "POSITION_UNKNOWN";
      if (phase != PH_REST && sec <= SEC_LN && !(sec == SEC_LAUNCH && dd < 0)) return "LAUNCH_BUSY";   // never shift under load
      d.want(dd * c.drumSign); markMoving(MV_DRUM); aimed = false;
    }
    tokenLive = false;
    jogMotor = wiper ? 1 : 2; jogDir = dd; jogUntil = now + (uint32_t)ms; state = S_JOG; stT = now;
    return nullptr;
  }
  const char* cmdCal(const char* what, const JObj& o) {
    if (!what) return "BAD_ARG";
    if (!strcmp(what, "current")) {
      if (w.running()) return "BUSY";
      w.c.offsetA += w.amps; if (w.c.offsetA < 0) w.c.offsetA = 0; w.amps = 0; return nullptr;
    }
    if (!strcmp(what, "tare")) { if (!hxSeen) return "NO_SENSOR"; hxOff += hxN / c.hxNPerCount; hxN = 0; return nullptr; }
    if (!strcmp(what, "free")) {
      if (!w.running() || now - w.startT < 800) return "NOT_RUNNING";
      c.freeA = w.amps; return nullptr;
    }
    if (!strcmp(what, "aim")) {
      double m; if (!o.num("deg", &m)) return "BAD_ARG";
      if (!aimKnown) return "AIM_UNKNOWN";
      float tab = pitchAt(aimPhase) - aimOffset; aimOffset = (float)m - tab; aimed = false; return nullptr;
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
      phase = PH_REST; cockPhase = 0; dirty = true; return nullptr;
    }
    if (!strcmp(what, "decock")) {                    // carriage drawn with the angle unknown (or a cocked shot called off):
      if (const char* e = idleCheck(true)) return e;  // run BACKWARD so the spring unwinds under the worm until SW_HOME closes.
      if (detentClosed() != DP_LAUNCH) return "NOT_IN_LAUNCH";   // backward also turns the aim train: aim must be redone
      if (!fitted(SW_HOME)) return "NO_SENSOR";
      if (closed(SW_HOME)) {                          // carriage already home (an unknown phase is found by "cock")
        JOut o(evBuf, sizeof evBuf); evBegin(o, "RESET"); o.boolean("ok", true); o.str("how", "decock"); evEnd(o); return nullptr;
      }
      opFrom = enc.deg(); homeSeen = false; aimKnown = false; aimed = false; tokenLive = false;
      w.c.tripA = c.tripA; w.want(c.aimDir, c.decockDuty); markMoving(MV_WIPER); state = S_DECOCK; stT = opT = now; return nullptr;
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
      if (!(v >= ks[i].lo && v <= ks[i].hi)) return "OUT_OF_RANGE";
      *ks[i].f = (float)v;
      if (!strcmp(k, "line_m")) { lineKnown = true; dirty = true; }
      if (!strcmp(k, "trip_a") && state != S_COCK && state != S_FIRE) w.c.tripA = c.tripA;
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
    else if (id->t == J_NUM) { if (id->n == floor(id->n) && fabs(id->n) < 1e15) { char t[24]; snprintf(t, sizeof t, "%.0f", id->n); r.keyRaw("id"); r.raw(t); } else r.num("id", id->n, 6); }
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
      err = cmdReel(a, m, s2);
    }
    else if (!strcmp(cmd, "jog")) { if (!o.num("ms", &b)) b = 300; err = (o.str("motor", &s) && o.num("dir", &a)) ? cmdJog(s, a, b) : "BAD_ARG"; }
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
      m.close();
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
    for (int i = 0; i < n; i++) if (!strcmp(ks[i].k, k)) { r.num("val", *ks[i].f, 6); return true; }
    return false;
  }
  void statusFields(JOut& r) {
    r.str("phase", PHASE_NAME[phase]); r.boolean("aimed", aimed); r.boolean("aim_known", aimKnown);
    r.boolean("drum_known", drumKnown); r.boolean("wraps_known", wrapsKnown);
    r.str("magnet", !enc.ok ? "NONE" : enc.magnetWeak() ? "WEAK" : enc.magnetStrong() ? "STRONG" : "OK");
    r.integer("glitches", (long)(enc.glitches > 99999 ? 99999 : enc.glitches)); r.boolean("hx", hxLive()); r.num("peak_a", w.peakA, 1);
    r.str("fw", GB_FW_VERSION);
    if (fault) { char wy[33]; strncpy(wy, faultWhy, 32); wy[32] = 0; r.str("why", wy); }
  }
  void cmdStop(bool estop) {
    w.halt(now, io.wiper); d.halt(now, io.drum); tokenLive = false; homing = false;
    int was = state;
    if (state == S_FAULT || state == S_SAFING) { state = S_FAULT; }
    else {
      if (was != S_IDLE) evAborted(STATE_NAME[was], estop ? "estop" : "stop");
      if (was == S_COCK || was == S_FIRE || was == S_DECOCK) phase = PH_UNKNOWN;   // stopped mid-cycle: re-establish (cock or cal decock)
      state = S_IDLE;
    }
    if (estop && state != S_FAULT) {
      mvSafing = 1;
      const char* hold = safeToNeutral(F_NONE);
      if (!hold && detentClosed() != DP_NEUTRAL && startDrumMove(DP_NEUTRAL, true)) { state = S_DRUM; stT = now; }
      else if (hold && detentClosed() != DP_NEUTRAL) evHeld(hold);
      else if (detentClosed() == DP_NEUTRAL) evMode(true);
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
  bool telBuild(char* buf, int cap, bool lean) {
    int d1 = lean ? 0 : 1, d2 = lean ? 1 : 2;
    JOut o(buf, cap); o.open(); o.obj("tel");
    o.str("state", STATE_NAME[state]); o.str("mode", modeName());
    if (drumKnown) o.num("drum_deg", drumDeg, d1); else o.null("drum_deg");
    o.str("drum_pos", DRUM_POS_NAME[drumPos()]);
    if (enc.ok || wiperInit) o.num("wiper_deg", enc.deg(), d1); else o.null("wiper_deg");
    o.num("rpm", enc.ok ? enc.rpm() : NAN, d1); o.num("amps", amps, d1);
    float p = pitch(); if (isnan(p)) o.null("pitch_deg"); else o.num("pitch_deg", p, d2);
    float ck = phase == PH_REST ? 0 : phase == PH_COCKED || phase == PH_FIRING ? 1 : phase == PH_COCKING ? fminf(1, fmaxf(0, cockPhase / c.cockEndDeg)) :
               phase == PH_RESETTING ? fmaxf(0, 1 - (cockPhase - c.releaseDeg) / fmaxf(1, c.cycleDeg - c.releaseDeg)) : NAN;
    if (isnan(ck)) o.null("cock"); else o.num("cock", ck, 2);
    if (wrapsKnown) o.integer("wraps", wraps); else o.null("wraps");
    if (lineKnown) o.num("line_m", lineM, d2); else o.null("line_m");
    if (hxLive()) o.num("tension_n", hxN, d1); else o.null("tension_n");
    o.obj("sw");                                     // closed contacts only (absent = open), one-letter keys: PROTOCOL.md
    for (int i = 0; i < SW_COUNT; i++) if (closed(i)) o.integer(SW_TEL[i], 1);
    o.close();
    if (fault) o.str("fault", FAULT_NAME[fault]); else o.null("fault");
    if (isnan(vbat)) o.null("vbat"); else o.num("vbat", vbat, 1);
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
    if (still && tunDirty) { tunDirty = false; if (io.saveTun) io.saveTun(tunNow()); }
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
        w.halt(now, io.wiper); aimPhase = wrapCyc(c.aimHomeDeg + sinceEdge(SW_RAIL_LOW, c.aimDir), c.aimCycleDeg); aimKnown = true; homing = false; sub = 3; legT = now; return;
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
      if (fell(SW_HOME)) { cockPhase = c.homeOpenDeg + sinceEdge(SW_HOME, cockDir()); phase = PH_COCKING; homeSeen = true; w.want(cockDir(), c.cockDuty); return; }
      // the train is either short of the first tooth or resetting after a release: SW_HOME must open within the reset
      // gap plus the first tooth. Any further and SW_HOME is lying: stop well before the release tooth could come round
      if (trav > c.cycleDeg - c.releaseDeg + c.homeOpenDeg + c.homeWinDeg + 20) { raise(F_BAD_STATE, "SW_HOME never opened (stuck closed?)"); return; }
      if (now - opT > c.cockMaxMs * 2) { raise(F_BAD_STATE, "cock recovery took too long"); return; }
      return;
    }
    if (fell(SW_HOME)) {                             // re-zero on the first tooth
      float at = cockPhase - sinceEdge(SW_HOME, cockDir());
      if (fabsf(at - c.homeOpenDeg) > c.homeWinDeg) { raise(F_SWITCH_DISAGREE, "SW_HOME opened at the wrong angle"); return; }
      cockPhase = c.homeOpenDeg + sinceEdge(SW_HOME, cockDir()); homeSeen = true;
    }
    if (!homeSeen && closed(SW_HOME) && cockPhase > c.homeOpenDeg + c.homeWinDeg) { raise(F_BAD_STATE, "carriage not drawn (SW_HOME still closed): rack or dog not engaged"); return; }
    if (homeSeen && closed(SW_HOME) && cockPhase < c.releaseDeg - c.releaseWinDeg) { raise(F_BAD_STATE, "carriage slipped home while cocking"); return; }
    if (cockPhase >= c.cockEndDeg - c.aimCoastDeg) {
      w.halt(now, io.wiper);
      w.c.tripA = c.tripA;
      if (closed(SW_HOME)) { raise(F_BAD_STATE, "SW_HOME closed at the end of cocking"); return; }
      phase = PH_COCKED; state = S_IDLE; stT = now; evCocked(true); dirty = true; return;
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
          cockPhase = c.homeOpenDeg + sinceEdge(SW_HOME, cockDir()); homeSeen = true;
        } else if (closed(SW_HOME) && cockPhase > c.homeOpenDeg + c.homeWinDeg) { raise(F_BAD_STATE, "carriage not drawn (SW_HOME still closed): rack or dog not engaged"); return; }
        if (now - opT > c.fireMaxMs) raise(F_BAD_STATE, "fire cycle took too long");
        return;
      }
      if (cockPhase > c.cockEndDeg - 20) w.c.tripA = c.tripA;
      if (rose(SW_HOME)) {                           // the relieved tooth let the rack go: carriage home = shot
        float at = cockPhase - sinceEdge(SW_HOME, cockDir());
        releaseSeen = true; phase = PH_RESETTING; aimed = false; lineKnown = false;
        JOut o(evBuf, sizeof evBuf); evBegin(o, "FIRED"); float p = pitch(); if (isnan(p)) o.null("pitch_deg"); else o.num("pitch_deg", p, 2);
        o.num("cock_deg", at, 1); evEnd(o);
        if (fabsf(at - c.releaseDeg) > c.releaseWinDeg * 2) { raise(F_BAD_STATE, "release at the wrong angle"); return; }
        w.want(cockDir(), c.resetDuty);
        return;
      }
      if (cockPhase > c.releaseDeg + c.releaseWinDeg) { raise(F_BAD_STATE, "no release seen (SW_HOME did not close)"); return; }
    } else if (phase == PH_RESETTING) {
      if (!closed(SW_HOME) && cockPhase < c.cycleDeg + c.homeOpenDeg - c.homeWinDeg) { raise(F_BAD_STATE, "SW_HOME opened while resetting"); return; }
      if (cockPhase >= c.cycleDeg - c.aimCoastDeg) {
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
    if (r > 0) { state = S_IDLE; stT = now; JOut o(evBuf, sizeof evBuf); evBegin(o, "UNWRAPPED"); o.boolean("ok", true); evEnd(o); dirty = true; }
  }
  void tickReel() {
    if (until == U_SEATED && rose(SW_SEATED)) { w.halt(now, io.wiper); state = S_IDLE; stT = now; evSeated(); return; }
    float frac = maxN > 0 ? tensionN / maxN : 0;
    if (frac >= 1.0f && now - w.startT > c.tensionIgnoreMs) { w.halt(now, io.wiper); enterHold("tension limit"); return; }
    int duty = reelDutyCmd;
    if (frac > 1 - c.taperFrac) { int dd = (int)(reelDutyCmd * (1 - frac) / c.taperFrac); duty = dd < c.reelMinDuty ? c.reelMinDuty : dd; }
    if (until == U_SEATED && lineKnown && lineM < c.seatSlowM && duty > c.seatSlowDuty) duty = c.seatSlowDuty;
    w.want(c.reelDir, duty);
    if (now - opT > c.reelMaxMs) { w.halt(now, io.wiper); state = S_IDLE; evAborted("REEL", "timeout"); dirty = true; }
  }
  void tickHold() {
    if (until == U_TENSION && hxLive() && hxN < c.resumeFrac * maxN && now - stT > 1000) {   // load cell: line slackened, wind again
      holdLimit = false; w.want(c.reelDir, reelDutyCmd); markMoving(MV_WIPER); state = S_REEL; stT = now;
    }
  }
  void tickJog() {
    if (jogMotor == 1 && drumPos() == DP_LAUNCH && jogDir == cockDir()) {
      if (fell(SW_HOME) && phase == PH_REST) { phase = PH_COCKING; cockPhase = c.homeOpenDeg + sinceEdge(SW_HOME, cockDir()); }
      if (cockPhase >= c.cockEndDeg - c.releaseWinDeg) {
        w.halt(now, io.wiper); state = S_IDLE; evAborted("JOG", "would pass the release tooth"); dirty = true; return;
      }
    }
    if (jogMotor == 2) {                              // stay off the groove ends (hard stops)
      int dirL = jogDir * c.drumSign; (void)dirL;
      if ((jogDir < 0 && sec == SEC_LAUNCH && drumDeg <= c.targetDeg[DP_LAUNCH] - 4) || (jogDir > 0 && sec == SEC_WRAPPED && drumDeg >= c.targetDeg[DP_WRAPPED] + 4)) {
        d.halt(now, io.drum); state = S_IDLE; stT = now; evAborted("JOG", "drum end stop"); dirty = true; return;
      }
    }
    if ((int32_t)(now - jogUntil) >= 0) {
      if (jogMotor == 1) w.halt(now, io.wiper); else d.halt(now, io.drum);
      state = S_IDLE; stT = now; dirty = true;
    }
  }
  void tickDecock() {
    float trav = (enc.deg() - opFrom) * c.aimDir;
    if (!homeSeen && closed(SW_HOME)) { homeSeen = true; opFrom = enc.deg(); return; }
    if (homeSeen && trav >= c.decockExtraDeg) {
      // the carriage stopped at its front stop as SW_HOME closed; after that sprag #1 freewheels, so the big gear sits
      // just short of the first-tooth angle
      w.halt(now, io.wiper); phase = PH_REST; cockPhase = c.homeOpenDeg - 2; state = S_IDLE; stT = now; dirty = true;
      JOut o(evBuf, sizeof evBuf); evBegin(o, "RESET"); o.boolean("ok", true); o.str("how", "decock"); evEnd(o); return;
    }
    if (!homeSeen && trav > c.cockEndDeg * 1.2f + 90) { raise(F_BAD_STATE, "decock: SW_HOME never closed"); return; }
  }

};

}  // namespace gb
