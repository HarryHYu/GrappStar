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
  uint8_t code = 0, prevCode = 0; uint32_t badT = 0;
  float drumDeg = 0; bool drumKnown = false; int wraps = 0; bool wrapsKnown = true;
  int lastSnap = -1; uint32_t lastSnapT = 0; bool snapClean = false;
  // current operation
  int drumTarget = -1; int drumDir = 0; uint32_t opT = 0, legT = 0; float opFrom = 0, opGoal = 0, opTravel = 0;
  float expectMs = 0; bool settling = false; uint32_t settleT = 0; int tries = 0; int sub = 0; bool estopMove = false;
  bool leftStart = false; int startContact = -1; int wrapGoal = 0;
  int calK = 0, wrapIdxK = -1; int jogMotor = 0, jogDir = 0; uint32_t jogUntil = 0;
  int until = U_STOP; float maxN = 20; int reelDutyCmd = 220; bool holdLimit = false;
  bool homeSeen = false, releaseSeen = false; float relPeakA = 0; int aimTries = 0; bool homing = false;
  // arm token
  uint32_t token = 0, tokenT = 0; bool tokenLive = false;
  // persistence
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
    Tun t; t.encSign = enc.sign > 0 ? 1 : 0; t.requireSeated = c.requireSeated;
    t.ampsPerVolt = w.c.ampsPerVolt; t.offsetA = w.c.offsetA; t.freeA = c.freeA; t.kNperA = c.kNperA; t.hxNPerCount = c.hxNPerCount;
    t.hxOff = hxOff; t.limitN = c.limitN; t.drumDps = c.drumDps; t.aimOffset = aimOffset; t.aimTol = c.aimTolDeg; t.tripA = c.tripA;
    t.cockTripA = c.cockTripA; t.vbatLow = c.vbatLow; t.aimHomeDeg = c.aimHomeDeg; sealTun(&t); return t;
  }
  // the sketch calls this before begin() with what saveTun() stored (or nullptr)
  void restoreTun(const Tun* t) {
    if (!t || !validTun(t)) return;
    enc.sign = c.wiperSign = t->encSign ? 1 : -1; c.requireSeated = t->requireSeated; w.c.ampsPerVolt = t->ampsPerVolt; w.c.offsetA = t->offsetA;
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
  // drum code from the bumps: bit 0 = SW_DRUM_A, 1 = SW_DRUM_B, 2 = SW_DRUM_C (debounced)
  uint8_t codeNow() const { return (closed(SW_DRUM_A) ? 1 : 0) | (closed(SW_DRUM_B) ? 2 : 0) | (closed(SW_DRUM_C) ? 4 : 0); }
  bool codeBad() const { return !((c.codeValid >> code) & 1); }
  int detentClosed() const {          // -1 none (between or the wrap dwell), 0..2 the detent the code says
    for (int k = 0; k < 3; k++) if (code == c.posCode[k]) return k;
    return -1;
  }
  bool entered(int k) const { return code == c.posCode[k] && prevCode != c.posCode[k]; }
  uint32_t codeEdgeTime() const {     // raw edge time of the bump that changed last
    uint32_t t = 0; for (int id = SW_DRUM_A; id <= SW_DRUM_C; id++) if (fitted(id)) { uint32_t e = sw.edgeTime(c.swBit[id]); if ((int32_t)(e - t) > 0 || t == 0) t = e; }
    return t;
  }
  int drumPos() const {
    int k = detentClosed();
    if (k >= 0) return k;
    if (codeBad() || !drumKnown) return DP_UNKNOWN;
    if (drumDeg > c.drumPosDeg[DP_REEL] + c.drumWinDeg && wrapsKnown && wraps >= 1) return DP_WRAPPED;
    return DP_BETWEEN;
  }
  const char* modeName() const {
    int p = drumPos();
    if (p == DP_LAUNCH) return "LAUNCH"; if (p == DP_NEUTRAL) return "NEUTRAL"; if (p == DP_REEL || p == DP_WRAPPED) return "REEL";
    return "NONE";
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
  void evMode(bool ok, const char* err = nullptr) {
    JOut o(evBuf, sizeof evBuf); evBegin(o, "MODE"); o.boolean("ok", ok); o.str("mode", modeName());
    o.str("drum_pos", DRUM_POS_NAME[drumPos()]); if (err) o.str("err", err); evEnd(o);
  }
  void evAborted(const char* what, const char* why) {
    JOut o(evBuf, sizeof evBuf); evBegin(o, "ABORTED"); o.str("what", what); o.str("why", why); evEnd(o);
  }

  void raise(int f, const char* why) {
    w.halt(now, io.wiper);
    if (state == S_FAULT || state == S_SAFING) { return; }   // the first fault wins
    fault = f; strncpy(faultWhy, why, sizeof faultWhy - 1); faultWhy[sizeof faultWhy - 1] = 0;
    tokenLive = false; homing = false;
    { JOut o(evBuf, sizeof evBuf); evBegin(o, "FAULT"); o.str("code", FAULT_NAME[f]); o.str("why", why); evEnd(o); }
    if (f == F_STALL) evSimple("STALL");
    // any fault: wiper off (done), drum to NEUTRAL when that is safe, otherwise the drum holds (worm self-locks)
    const char* hold = safeToNeutral(f);
    if (!hold && detentClosed() != DP_NEUTRAL) { if (startDrumMove(DP_NEUTRAL, true)) { state = S_SAFING; return; } hold = "drum move failed"; }
    d.halt(now, io.drum);
    state = S_FAULT; stT = now;
    if (hold && detentClosed() != DP_NEUTRAL) { JOut o(evBuf, sizeof evBuf); evBegin(o, "MODE"); o.boolean("ok", false); o.str("mode", modeName()); o.str("err", "DRUM_HELD"); o.str("why", hold); evEnd(o); }
    dirty = true;
  }
  // nullptr if the drum may go to NEUTRAL now, else why it must hold
  const char* safeToNeutral(int f) const {
    if (f == F_DRUM_TIMEOUT || f == F_DRUM_SENSOR) return "drum fault";
    if (f == F_SWITCH_DISAGREE && codeBad()) return "drum code contacts disagree";
    if (!drumKnown && detentClosed() < 0) return "drum position unknown";
    if (!wrapsKnown) return "wraps unknown";
    if (wraps > 0 || drumPos() == DP_WRAPPED) return "wraps on: the capstan holds the line";
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
    int k = detentClosed();
    if (k >= 0) { drumKnown = true; drumDeg = c.drumPosDeg[k]; wraps = 0; wrapsKnown = true; }
    else if (drumTrust && s->drumKnown) { drumKnown = true; drumDeg = s->drumDeg; wraps = s->wraps; wrapsKnown = s->wrapsKnown; }
    else { drumKnown = false; wrapsKnown = s && drumTrust ? (bool)s->wrapsKnown : false; wraps = wrapsKnown ? s->wraps : 0; }
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
    prevCode = code; code = codeNow();
    float isv = io.wiperIsVolts ? io.wiperIsVolts() : 0;
    w.tick(now, dt, isv, enc.ok ? enc.rateDps : NAN, io.wiper);   // ramp, gap, current, stall latches
    amps = w.amps;
    vbat = io.vbat ? io.vbat() : NAN;
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

  // drum dead reckoning between contacts; contacts snap the estimate
  void trackDrum(uint32_t dt) {
    int dir = d.outDir * c.drumSign;            // logical: + toward REEL
    if (dir && drumKnown) drumDeg += dir * c.drumDps * dt / 1000.f;
    for (int k = 0; k < 3; k++) {
      if (entered(k)) {
        if (lastSnap >= 0 && lastSnap != k && snapClean && d.running()) {   // learn the real speed
          float ms = (float)(now - lastSnapT);
          if (ms > 50) { float v = fabsf(c.drumPosDeg[k] - c.drumPosDeg[lastSnap]) / (ms / 1000.f); if (v > 0.3f * c.drumDps && v < 3 * c.drumDps) c.drumDps = 0.5f * c.drumDps + 0.5f * v; }
        }
        // a code bump starts drumWinDeg before the detent centre: snap to that edge (plus the travel since the raw edge)
        float late = dir ? dir * c.drumDps * (now - codeEdgeTime()) / 1000.f : 0;
        drumDeg = dir ? c.drumPosDeg[k] - dir * c.drumWinDeg + late : c.drumPosDeg[k]; drumKnown = true; lastSnap = k; lastSnapT = now; snapClean = d.running();
        wraps = 0; wrapsKnown = true;
      }
    }
    if (!d.running()) snapClean = false;
    // wraps from the drum angle; the wrap index contact snaps to whole wraps
    if (drumKnown) {
      if (fitted(SW_WRAP_HOME) && rose(SW_WRAP_HOME) && dir != 0) {
        float k = roundf((drumDeg - c.wrapStartDeg + dir * c.wrapWinDeg) / c.degPerWrap);
        if (k >= 1) { drumDeg = c.wrapStartDeg + k * c.degPerWrap - dir * c.wrapWinDeg + dir * c.drumDps * (now - sw.edgeTime(c.swBit[SW_WRAP_HOME])) / 1000.f; wrapIdxK = (int)k; }
        (void)0;
      }
      float laid = (drumDeg - c.wrapStartDeg) / c.degPerWrap;
      int n = laid <= 0 ? 0 : (int)floorf(laid + 0.15f);  // a wrap counts once the arm is 85% round
      if (dir != 0 || state == S_WRAP || state == S_UNWRAP) { wraps = n; wrapsKnown = true; }
    }
  }

  // ================================================================== watchdogs (every tick)
  void watch() {
    if (state == S_FAULT) return;
    if (w.overcurrent) { char b[48]; snprintf(b, sizeof b, "wiper over %.0f A", w.c.tripA); w.clearLatches(); onOvercurrent(b); return; }
    if (w.stalled) { w.clearLatches(); onStall(); return; }
    if (!enc.ok && (w.running() || state == S_AIM || state == S_COCK || state == S_FIRE || state == S_REEL)) { raise(F_ANGLE_SENSOR, "AS5600 lost while the wiper runs"); return; }
    if (codeBad()) { if (now - lastMultiOk > 100) { raise(F_SWITCH_DISAGREE, "drum code bumps give an impossible code"); return; } }
    else lastMultiOk = now;
    bool drumMoveState = state == S_DRUM || state == S_SAFING || state == S_WRAP || state == S_UNWRAP || state == S_CALDRUM;
    if (drumMoveState && (chat(SW_DRUM_A) || chat(SW_DRUM_B) || chat(SW_DRUM_C))) { raise(F_SWITCH_DISAGREE, "drum code contact chattering"); return; }
    bool running = w.running() || d.running();
    if (!isnan(vbat) && running && vbat < c.vbatLow) {
      if (!lowRun) { lowRun = true; lowT = now; } else if (now - lowT >= c.vbatLowMs) { raise(F_BROWNOUT, "battery sagged under load"); return; }
    } else lowRun = false;
    if (tokenLive && now - tokenT > c.armMs) tokenLive = false;
    // optional host watchdog: no line from the Jetson for host_timeout_s while a motor runs = stop (0 = off)
    if (c.hostTimeoutS > 0 && running && now - hostT > (uint32_t)(c.hostTimeoutS * 1000)) { cmdStop(false); evAborted("HOST", "host silent"); hostT = now; }
  }
  uint32_t hostT = 0;
  uint32_t lastMultiOk = 0;
  void onOvercurrent(const char* why) {
    if (state == S_REEL && until == U_TENSION) { w.halt(now, io.wiper); enterHold("over current at the limit"); return; }
    raise(F_OVERCURRENT, why);
  }
  void onStall() {
    if (state == S_REEL && until == U_TENSION) { w.halt(now, io.wiper); enterHold("stalled at the limit"); return; }
    raise(F_STALL, "wiper stalled");
  }

  // ================================================================== drum moves
  float drumExpectMs(float fromDeg, float toDeg) const { return fabsf(toDeg - fromDeg) / c.drumDps * 1000.f; }
  // start a drum move to a detent; false if it cannot start (position unknown)
  bool startDrumMove(int target, bool safing) {
    int k = detentClosed();
    float from;
    if (k >= 0) from = c.drumPosDeg[k]; else if (drumKnown) from = drumDeg; else return false;
    drumTarget = target; float to = c.drumPosDeg[target];
    drumDir = to > from ? 1 : -1; startContact = k; leftStart = startContact < 0;
    expectMs = drumExpectMs(from, to); opT = legT = now; settling = false; tries = 0; sub = 0;
    opFrom = from;
    d.want(drumDir * c.drumSign); markMoving(MV_DRUM);
    (void)safing;
    return true;
  }
  uint32_t drumTimeout() const { return (uint32_t)(expectMs * c.drumTimeFactor) + c.drumTimeMarginMs; }
  // returns 0 running, 1 arrived, -1 failed (fault raised or reported)
  int tickDrumMove(bool safing) {
    if (sub == 1) {                                    // backing off before a retry
      if (now - legT >= 150) { d.halt(now, io.drum); sub = 2; legT = now; w.want(drumTarget == DP_REEL ? c.reelDir : c.aimDir, c.nudgeDuty); nudgeFrom = enc.deg(); markMoving(MV_WIPER); }
      return 0;
    }
    if (sub == 2) {                                    // nudging the wiper so the dog teeth line up
      if (fabsf(enc.deg() - nudgeFrom) >= c.nudgeDeg || now - legT > 1500) {
        w.halt(now, io.wiper); if (drumTarget == DP_LAUNCH) { aimKnown = false; aimed = false; }
        sub = 0; legT = now; d.want(drumDir * c.drumSign);
      }
      return 0;
    }
    if (settling) {
      if (centred(c.drumPosDeg[drumTarget], drumDir)) {
        d.halt(now, io.drum);
        if (detentClosed() != drumTarget) { failDrum(safing, F_DRUM_SENSOR, "drum code lost the detent after arriving"); return -1; }
        drumDeg = c.drumPosDeg[drumTarget]; drumKnown = true;
        return 1;
      }
      return 0;
    }
    // wrong contacts: only the ones between start and target may close on the way
    for (int k = 0; k < 3; k++) {
      if (!entered(k) || k == drumTarget) continue;
      float pk = c.drumPosDeg[k]; float lo = fminf(opFrom, c.drumPosDeg[drumTarget]) - 1, hi = fmaxf(opFrom, c.drumPosDeg[drumTarget]) + 1;
      if (pk < lo || pk > hi) { char b[48]; snprintf(b, sizeof b, "%s code seen on the way", DRUM_POS_NAME[k]); failDrum(safing, F_DRUM_SENSOR, b); return -1; }
    }
    if (startContact >= 0) {
      if (!leftStart && detentClosed() != startContact) leftStart = true;
      uint32_t leaveMs = (uint32_t)(2.5f * c.drumWinDeg / c.drumDps * 1000.f) + 700;
      if (!leftStart && now - legT > leaveMs) return retryOrFail(safing, "drum did not leave its detent");
    }
    if (entered(drumTarget) || (detentClosed() == drumTarget && leftStart && d.running() && now - opT > 30)) {
      float el = (float)(now - opT);
      if (el < c.drumEarlyFrac * expectMs && expectMs > 300 && tries == 0) { failDrum(safing, F_DRUM_SENSOR, "target contact closed too early"); return -1; }
      settling = true; settleT = now; return 0;
    }
    if (now - legT > drumTimeout()) return retryOrFail(safing, "drum did not reach the detent in time");
    return 0;
  }
  float nudgeFrom = 0;
  // after a contact closed: keep driving until the estimate reaches the detent centre (bounded in time)
  bool centred(float pos, int dir) const {
    uint32_t maxMs = (uint32_t)(2.f * c.drumWinDeg / c.drumDps * 1000.f) + 100;
    return dir * (drumDeg - pos) >= 0 || now - settleT >= maxMs;
  }
  int retryOrFail(bool safing, const char* why) {
    if (tries < c.drumRetries && !safing) {
      tries++; d.want(-drumDir * c.drumSign); sub = 1; legT = now; return 0;
    }
    failDrum(safing, F_DRUM_TIMEOUT, why); return -1;
  }
  void failDrum(bool safing, int f, const char* why) {
    d.halt(now, io.drum);
    if (safing) { state = S_FAULT; stT = now; evMode(false, why); dirty = true; return; }
    raise(f, why);
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
    if (!wrapsKnown) return "POSITION_UNKNOWN";
    if (wraps > 0 || drumPos() == DP_WRAPPED) return "WRAPS_ON";
    int k = detentClosed();
    if (codeBad()) return "SWITCH_DISAGREE";
    if (k == t) { evMode(true); return nullptr; }
    if (k < 0 && !drumKnown) return "POSITION_UNKNOWN";
    if (drumPos() == DP_LAUNCH && phase != PH_REST) return "LAUNCH_BUSY";   // the worm holds the spring: never shift under load
    if (w.running()) return "BUSY";
    if (!startDrumMove(t, false)) return "POSITION_UNKNOWN";
    aimed = false; tokenLive = false;
    state = S_DRUM; stT = now; estopMove = false;
    return nullptr;
  }
  const char* cmdAim(double deg) {
    if (const char* e = idleCheck(true)) return e;
    if (state == S_HOLD) return "HOLDING";
    if (detentClosed() != DP_LAUNCH) return "NOT_IN_LAUNCH";
    if (phase == PH_UNKNOWN) return "PHASE_UNKNOWN";
    if (c.aimNeedsRest && phase != PH_REST) return "NOT_AT_REST";
    float lo = fmaxf(c.pitchMin, pitchRange(false)), hi = fminf(c.pitchMax, pitchRange(true));
    if (!(deg >= lo - 1e-3 && deg <= hi + 1e-3)) return "OUT_OF_RANGE";
    aimTarget = (float)deg; aimed = false; tokenLive = false; aimTries = 0;
    if (!aimKnown) {
      if (!fitted(SW_RAIL_LOW)) return "AIM_UNKNOWN";
      homing = true; opFrom = enc.deg(); w.want(c.aimDir, c.aimSlowDuty);
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
    if (detentClosed() != DP_LAUNCH) return "NOT_IN_LAUNCH";
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
    if (detentClosed() != DP_LAUNCH) return "NOT_IN_LAUNCH";
    if (!wrapsKnown || wraps > 0) return "WRAPS_ON";
    if (phase != PH_COCKED || closed(SW_HOME) || !fitted(SW_HOME)) return "NOT_COCKED";
    if (!aimKnown || !aimed || isnan(aimTarget) || fabsf(pitch() - aimTarget) > c.aimTolDeg) return "NOT_AIMED";
    if (c.requireSeated && !closed(SW_SEATED)) return fitted(SW_SEATED) ? "NOT_SEATED" : "NO_SENSOR";
    return nullptr;
  }
  const char* cmdArm(uint32_t* tok) {
    if (const char* e = fireCheck()) return e;
    uint32_t r = io.rand32 ? io.rand32() : (now * 2654435761u) ^ 0x5bd1e995u;
    token = (r % 900000u) + 100000u;              // six digits, never 0
    tokenT = now; tokenLive = true; *tok = token;
    return nullptr;
  }
  const char* cmdFire(double tok) {
    if (!tokenLive) return "NOT_ARMED";
    if (now - tokenT > c.armMs) { tokenLive = false; return "TOKEN_EXPIRED"; }
    if (tok != (double)token) { tokenLive = false; return "BAD_TOKEN"; }   // a wrong token burns the arm
    tokenLive = false;
    if (const char* e = fireCheck()) return e;
    releaseSeen = false; relPeakA = 0; phase = PH_FIRING; w.c.tripA = c.tripA; opFrom = cockPhase;
    w.want(cockDir(), c.fireDuty); markMoving(MV_WIPER); state = S_FIRE; stT = opT = now;
    return nullptr;
  }
  const char* cmdWrap() {
    if (const char* e = idleCheck(false)) return e;
    if (state == S_HOLD) return "HOLDING";
    if (detentClosed() != DP_REEL) return "NOT_IN_REEL";
    if (!wrapsKnown || wraps != 0) return "WRAPS_ON";
    if (w.running()) return "BUSY";
    wrapGoal = c.wrapsTarget; if (wrapGoal > c.maxWraps) wrapGoal = c.maxWraps;
    opGoal = c.wrapStartDeg + wrapGoal * c.degPerWrap;
    drumDeg = c.drumPosDeg[DP_REEL]; drumKnown = true;
    expectMs = drumExpectMs(drumDeg, opGoal); opT = legT = now; settling = false; wrapIdxK = -1;
    d.want(c.drumSign); markMoving(MV_DRUM); state = S_WRAP; stT = now;
    return nullptr;
  }
  const char* cmdUnwrap() {
    if (const char* e = idleCheck(false)) return e;
    if (state == S_HOLD) return "HOLDING";
    if (detentClosed() == DP_REEL && wrapsKnown && wraps == 0) { JOut o(evBuf, sizeof evBuf); evBegin(o, "UNWRAPPED"); o.boolean("ok", true); evEnd(o); return nullptr; }
    if (!drumKnown) return "POSITION_UNKNOWN";
    if (drumDeg < c.drumPosDeg[DP_REEL] - c.drumWinDeg) return "NOT_WRAPPED";
    if (w.running()) return "BUSY";
    expectMs = drumExpectMs(drumDeg, c.drumPosDeg[DP_REEL]); opT = legT = now; settling = false;
    d.want(-c.drumSign); markMoving(MV_DRUM); state = S_UNWRAP; stT = now;
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
      if (p == DP_LAUNCH && dd == cockDir() && (phase == PH_COCKED || phase == PH_UNKNOWN || phase == PH_FIRING)) return "WOULD_FIRE";
      if (p == DP_LAUNCH && dd == c.aimDir && phase != PH_REST) return "NOT_AT_REST";   // the drawn spring would unwind: cal decock
      if ((p == DP_REEL || p == DP_WRAPPED) && dd != c.reelDir) return "BACKSTOP";       // sprag #2: the capstan never turns out
      w.c.tripA = c.tripA; w.want(dd, c.jogDuty); markMoving(MV_WIPER);
      if (p == DP_LAUNCH && dd == c.aimDir) aimed = false;
    } else {
      if (w.running()) return "BUSY";
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
    if (!strcmp(what, "cock_rest")) {                 // declare the launch train at rest (carriage forward, uncocked)
      if (moving()) return "BUSY";
      if (fitted(SW_HOME) && !closed(SW_HOME)) return "SWITCH_DISAGREE";
      phase = PH_REST; cockPhase = 0; dirty = true; return nullptr;
    }
    if (!strcmp(what, "decock")) {                    // carriage drawn with the angle unknown (or a cocked shot called off):
      if (const char* e = idleCheck(true)) return e;  // run BACKWARD so the spring unwinds under the worm until SW_HOME closes.
      if (detentClosed() != DP_LAUNCH) return "NOT_IN_LAUNCH";   // backward also turns the aim train: aim must be redone
      if (!fitted(SW_HOME)) return "NO_SENSOR";
      if (closed(SW_HOME)) return nullptr;            // carriage already home (an unknown phase is found by "cock")
      opFrom = enc.deg(); homeSeen = false; aimKnown = false; aimed = false; tokenLive = false;
      w.c.tripA = c.tripA; w.want(c.aimDir, c.decockDuty); markMoving(MV_WIPER); state = S_DECOCK; stT = opT = now; return nullptr;
    }
    if (!strcmp(what, "drum")) {                      // home search toward LAUNCH until any detent contact closes
      if (const char* e = idleCheck(false)) return e;
      if (wrapsKnown && wraps > 0) return "WRAPS_ON";
      if (detentClosed() >= 0) { drumKnown = true; drumDeg = c.drumPosDeg[detentClosed()]; evMode(true); return nullptr; }
      expectMs = drumExpectMs(c.drumPosDeg[DP_LAUNCH], c.wrapStartDeg + c.maxWraps * c.degPerWrap); opT = legT = now;
      settling = false; d.want(-c.drumSign); markMoving(MV_DRUM); state = S_CALDRUM; stT = now; return nullptr;
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
    out[n++] = {"aim_home_deg", &c.aimHomeDeg, -720, 720};
    out[n++] = {"host_timeout_s", &c.hostTimeoutS, 0, 600};
    return n;
  }
  const char* cmdSet(const char* k, double v) {
    if (!k) return "BAD_ARG";
    if (!strcmp(k, "require_seated")) { c.requireSeated = v != 0; return nullptr; }
    if (!strcmp(k, "enc_sign")) { if (v != 1 && v != -1) return "BAD_ARG"; if (moving()) return "BUSY"; enc.sign = (int)v; enc.count = -enc.count; lastWiperDeg = enc.deg(); return nullptr; }
    Key ks[20]; int n = keys(ks);
    for (int i = 0; i < n; i++) if (!strcmp(ks[i].k, k)) {
      if (!(v >= ks[i].lo && v <= ks[i].hi)) return "OUT_OF_RANGE";
      *ks[i].f = (float)v;
      if (!strcmp(k, "line_m")) lineKnown = true;
      if (!strcmp(k, "trip_a")) w.c.tripA = c.tripA;
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
      const char* hold = safeToNeutral(F_NONE);
      if (!hold && detentClosed() != DP_NEUTRAL && startDrumMove(DP_NEUTRAL, true)) { state = S_DRUM; estopMove = true; stT = now; }
      else if (hold && detentClosed() != DP_NEUTRAL) { JOut o(evBuf, sizeof evBuf); evBegin(o, "MODE"); o.boolean("ok", false); o.str("mode", modeName()); o.str("err", "DRUM_HELD"); o.str("why", hold); evEnd(o); }
    }
    dirty = true;
  }
  const char* cmdClear() {
    if (state != S_FAULT) return state == S_SAFING ? "BUSY" : nullptr;
    if (fault == F_ANGLE_SENSOR && !enc.ok) return "STILL_PRESENT";
    if (fault == F_SWITCH_DISAGREE && codeBad()) return "STILL_PRESENT";
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
    static const char* const L[SW_COUNT] = {"L", "N", "R", "W", "S", "C", "A"};
    for (int i = 0; i < SW_COUNT; i++) if (closed(i)) o.integer(L[i], 1);
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
      case S_CALDRUM: tickCalDrum(); break;
      case S_DECOCK: tickDecock(); break;
      default: break;
    }
    d.tick(now, dt, io.drum, io.drumSleep);
    // persistence: once everything is still for 300 ms, write the final positions (moving = 0)
    bool still = !w.running() && !d.running() && (state == S_IDLE || state == S_HOLD || state == S_FAULT);
    if (!still) quietT = now;
    if (still && now - quietT >= 300 && (movingBits || dirty)) { movingBits = 0; dirty = false; persist(0); }
    if (still && tunDirty) { tunDirty = false; if (io.saveTun) io.saveTun(tunNow()); }
  }

  void tickDrumState() {
    bool safing = state == S_SAFING || estopMove;
    int r = tickDrumMove(safing);
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
      if (trav > c.cycleDeg * 1.1f) { raise(F_BAD_STATE, "SW_HOME never opened in a full big gear turn"); return; }
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
    // stop when the estimate reaches the target wrap centre. With the index contact fitted, the target wrap's index
    // edge must have been seen (it snaps the estimate), else the count is not trusted
    if (drumDeg >= opGoal) {
      bool idxOk = !fitted(SW_WRAP_HOME) || wrapIdxK == wrapGoal;
      if (idxOk) {
        d.halt(now, io.drum);
        wraps = wrapGoal; wrapsKnown = true;
        state = S_IDLE; stT = now;
        JOut o(evBuf, sizeof evBuf); evBegin(o, "WRAPPED"); o.boolean("ok", true); o.integer("wraps", wraps); evEnd(o); dirty = true; return;
      }
      if (drumDeg > opGoal + c.degPerWrap * 0.25f) { raise(F_DRUM_SENSOR, "wrap index contact not seen"); return; }
    }
    if (entered(DP_REEL) || entered(DP_NEUTRAL) || entered(DP_LAUNCH)) { raise(F_DRUM_SENSOR, "detent code seen while wrapping"); return; }
    if (now - legT > (uint32_t)(expectMs * c.drumTimeFactor) + c.drumTimeMarginMs) { raise(F_DRUM_TIMEOUT, "wrap took too long"); return; }
  }
  void tickUnwrap() {
    if (entered(DP_REEL) && !settling) { settling = true; settleT = now; }
    if (settling && centred(c.drumPosDeg[DP_REEL], -1)) {
      d.halt(now, io.drum);
      if (detentClosed() != DP_REEL) { raise(F_DRUM_SENSOR, "REEL code lost after unwrapping"); return; }
      wraps = 0; wrapsKnown = true; drumDeg = c.drumPosDeg[DP_REEL]; state = S_IDLE; stT = now;
      JOut o(evBuf, sizeof evBuf); evBegin(o, "UNWRAPPED"); o.boolean("ok", true); evEnd(o); dirty = true; return;
    }
    if (entered(DP_NEUTRAL) || entered(DP_LAUNCH)) { raise(F_DRUM_SENSOR, "passed REEL without its code"); return; }
    if (now - legT > (uint32_t)(expectMs * c.drumTimeFactor) + c.drumTimeMarginMs) { raise(F_DRUM_TIMEOUT, "unwrap took too long"); return; }
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
    if (until == U_TENSION && hxLive() && hxN < c.resumeFrac * maxN) {   // load cell shows the line slackened: wind again
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
    if (jogMotor == 2 && wrapsKnown && wraps > 0 && drumPos() != DP_WRAPPED && drumPos() != DP_BETWEEN) {}
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
  void tickCalDrum() {
    int k = detentClosed();
    if (k >= 0 && !settling) { settling = true; settleT = now; calK = k; }
    if (settling && centred(c.drumPosDeg[calK], -1)) {
      d.halt(now, io.drum); drumKnown = true; drumDeg = c.drumPosDeg[calK]; wraps = 0; wrapsKnown = true;
      state = S_IDLE; stT = now; evMode(true); dirty = true; return;
    }
    if (now - opT > (uint32_t)(expectMs * c.drumTimeFactor) + c.drumTimeMarginMs) { raise(F_DRUM_TIMEOUT, "drum home search found no detent"); return; }
  }
};

}  // namespace gb
