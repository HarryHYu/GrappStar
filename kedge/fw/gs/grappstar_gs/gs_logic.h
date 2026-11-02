// GrappStar ground station logic (launcher + reel on ONE wiper motor): plain C++, no Arduino calls, so it can be
// unit-tested on a PC (see fw/test/test_gs.cpp). The sketch supplies the hardware through the Io struct.
// Mechanism: gs/ARCHITECTURE.md. F (motor +, RPWM) = reel wind. B (motor -, LPWM) = launcher program gear (PG).
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

namespace gs {

// bit set = switch closed (pin pulled to GND). SW_COCKED is a KW11 on COM + NO outside the left cheek: the sear arm presses
// its lever while the hook sits up on its stop, so CLOSED = hook up, OPEN = hook pushed down (keel riding over it while
// cocking, trip cord pulling the sear arm while firing). SW_SEATED closes
// when the dart tail is within 4 mm of the pusher.
enum { SW_LOW = 1, SW_COCKED = 2, SW_SEATED = 4 };

struct Io {
  void (*setMotor)(int dir, int duty);   // dir +1 F, -1 B, 0 stop (brake); duty 0..255
  int (*angleRaw)();                     // AS5600 raw angle 0..4095, -1 = missing or no magnet
  float (*isVolts)();                    // volts at GPIO1 (BTS7960 R_IS + L_IS through the divider)
  float (*hxRaw)();                      // HX711 raw counts, NAN if absent or no new sample
  uint8_t (*switches)();                 // raw switch bits (SW_*), debounced here
  void (*log)(const char* line);
  void (*save)(float phi, bool known, bool probCocked);   // optional: keep phi over a power cycle (worm holds the gear)
};

struct Tuning {
  uint32_t serialRunMs = 1500, wifiDeadmanMs = 350;
  uint32_t rampMs = 300;                 // soft start: PWM 0 -> 255 takes this long (wiper inrush blew a 10 A fuse)
  uint32_t reverseGapMs = 200;           // stopped this long before the direction may change
  uint32_t debounceMs = 20;
  int encSign = +1;                      // +1 if the AS5600 count rises while running F. Flip to -1 if 'i' shows it falling
  // current sense. REPLACE WITH BENCH VALUE: see README "Tuning"
  float isAmpsPerVolt = 12.5f;           // motor amps per volt at GPIO1 (kILIS 8500, 680R bottom resistor)
  float isOffsetA = 0;                   // learned by 'z' with the motor stopped
  float tripA = 18; int tripSamples = 2; // 2 samples in a row (10 ms) over 18 A = trip
  float cockTripA = 12; uint32_t cockStallMs = 300;   // tighter while cocking (springs loading)
  // stall: driving above 30% PWM but the AS5600 has not moved stallCounts in stallMs
  float stallDutyFrac = 0.30f; uint32_t stallMs = 600; int stallCounts = 25;
  // program gear (deg of PG). pgRatio = main shaft turns per PG turn (pinion z14 -> PG z30)
  float pgRatio = 30.0f / 14.0f;
  // CAD after review round 6 (mesh-solved): first tooth meets the rack at 340, carriage released at 246.5 (208.2 mm) and
  // runs 1.5 mm onto the hook, lift 252.5 -> 302.5 (4 -> 35 deg, blended constant speed), trip cord taut at 284.5 (rail 25.3), SW_COCKED opens 284.8,
  // shot at 294.5 (rail 33.0), drop 304.5 -> 332.5 (an overrun: the PG runs ahead of the pinion)
  float phiSwOpen = 215.2f, phiSwCocked = 241.2f;   // SW_COCKED opens (keel pushes the hook down) / closes again (keel passed, hook up)
  float phiCocked = 249.5f, phiCockMax = 252.5f;     // stop in the dwell / lift starts
  float phiCockSlow = 230; int cockSlowDuty = 130;   // slow from 230 so the hook drops in and the stop is precise. 30% (77) stalls
                                                     // at the full-draw load with the CAD wiper; 130 (51%) is about 1/3 speed
  float phiDrawn0 = 340, phiDrawn1 = 241;            // carriage partly drawn, held only by backstop C: F refused (unless force).
                                                     // 241 is also where the cocked zone starts (jog b refused, finish allowed, READY)
  float phiTripWin0 = 278, phiTripWin1 = 300;        // SW_COCKED must open in here while firing (trip cord taut, opens at 284.8)
  float phiLowOpenCock = 257.5f;                     // SW_LOW opens here (rail 4.5 deg): if it opens while cocking, it was cocked
  float phiTripOpen = 285;                           // SW_COCKED opens here while firing (trip cord taut at 284.8): used if a cock finds it
  float phiDrop = 290.3f;                            // pinion-equivalent phi when SW_LOW closes after the drop (304.5 less the rail fall 30.5 x 14/30)
  float phiDropEnd = 333.0f;                         // after the free-fall drop the PG sits at about 331..335 (roller clutch: no dead band)
  float phiTop0 = 285.0f, phiTop1 = 302.5f; int topDuty = 170; int dropDuty = 120; float phiDropStart = 304.5f; uint32_t dropMaxMs = 300;   // fire: ease off over the top blend so the rail does not leave the cam
  int lowerDuty = 160; float lowerMaxDeg = 90;       // "lower": B until SW_LOW closes (lift side 257.5 -> drop 304.5 is 47 deg, plus margin)
  float keelWinDeg = 20;                             // cocking: SW_COCKED must open within this of phiSwOpen, else phi was wrong
  float saveDeg = 0.5f;                              // save phi to flash when it moved this much (AS5600 jitter is not a reason to write)
  float phiTrip = 294.5f, phiFireEnd = 338;   // fire run stops at 338, before the first tooth meets the rack at 340
  float fireExtraDeg = 1.5f;             // waiting for the rail-down switch: never past 339.5
  float shotDropFrac = 0.70f; uint32_t shotDropMs = 40;
  int cockDuty = 255, fireDuty = 255, towDuty = 230, retrieveDuty = 200, jogDuty = 160;
  // reel: stationary spool, rotor 58/39 = 1.487x main shaft, each rotor turn lays one wrap
  float rotorRatio = 58.0f / 39.0f, coreD_mm = 36, lineD_mm = 1.0f, lineTotalM = 20; int turnsPerLayer = 21;   // gears z58/z39, usable spool width 23
  float payoutMps = 10.7f, payoutMaxM = 12;   // flight model: 10.7 m/s, 9.6 m out, range 8.5 m. PAYOUT line estimate by time (reel is passive). /cmd?c=line&v= overrides
  // tension. REPLACE WITH BENCH VALUES
  float iFreeA = 1.5f;                   // free running current winding a slack line ('z' while jogging f learns it)
  float kNperA = 8.5f;                   // line newtons per amp above free running (luggage scale test). CAD model value, wrap friction x2.0
  float hxNPerCount = 1.0f / 21000;      // HX711 counts -> N (hang a known weight)
  float limitN = 20, taperFrac = 0.30f; int minDuty = 80;
  uint32_t tensionIgnoreMs = 350;        // after a start the current is ramp inrush, not tension (TOW creeps at minDuty meanwhile)
  float resumeFrac = 0.6f;               // HX711 only: holding winds again if the line slackens below 60% of the limit
  // retrieve
  float retrieveStallA = 4.0f; uint32_t retrieveStallMs = 150;
  float retrieveSlowM = 1.5f; int retrieveSlowDuty = 110;   // last 1.5 m of the line estimate: slow, so the tail meets the pusher gently
  uint32_t cockMaxMs = 15000, fireMaxMs = 10000, towMaxMs = 180000, retrieveMaxMs = 180000;
  bool requireSeatedToFire = true;       // no dry fires
};

enum State { IDLE, COCKING, READY, FIRING, PAYOUT, TOW, RETRIEVE, JOG, LOWER, FAULT };
static const char* stateName(int s) {
  static const char* n[] = {"idle", "cocking", "ready", "firing", "payout", "tow", "retrieve", "jog", "lower", "fault"};
  return n[s];
}
enum Cycle { C_OFF, C_FIRE, C_WAIT_ANCHOR, C_TOW, C_WAIT_RELEASE, C_COCK, C_RETRIEVE };
static const char* cycleName(int c) {
  static const char* n[] = {"off", "fire", "wait anchored", "tow", "wait released", "cock", "retrieve"};
  return n[c];
}

static float wrap360(float a) { a = fmodf(a, 360.f); return a < 0 ? a + 360.f : a; }
static float wrap180(float a) { a = wrap360(a); return a >= 180.f ? a - 360.f : a; }

// spool fill: line stored on the spool (m) -> diameter of the layer the next wrap goes on / comes off (mm)
static float layerD(float storedM, const Tuning& t) {
  float s = storedM * 1000.f;
  for (int n = 0; n < 40; n++) {
    float d = t.coreD_mm + (2 * n + 1) * t.lineD_mm, len = t.turnsPerLayer * (float)M_PI * d;
    if (s < len) return d;
    s -= len;
  }
  return t.coreD_mm + 81 * t.lineD_mm;
}

// rail elevation estimate from the program (cam follows the table in ARCHITECTURE.md section 2)
// rail elevation from the CAD cam (gs/out/theta_table.json sampled every 2.5 deg from 252.5 to 340), 4 deg elsewhere
static float thetaOf(float phi) {
  static const float T[36] = {4.00f, 4.00f, 4.50f, 5.54f, 7.17f, 9.17f, 11.24f, 13.30f, 15.37f, 17.44f, 19.50f, 21.57f,
    23.64f, 25.70f, 27.77f, 29.84f, 31.80f, 33.34f, 34.40f, 34.93f, 35.00f, 34.80f, 33.86f, 31.86f,
    28.37f, 24.68f, 20.99f, 17.30f, 13.61f, 9.92f, 6.42f, 4.00f, 4.00f, 4.00f, 4.00f, 4.00f};
  float x = (phi - 252.5f) / 2.5f;
  if (x <= 0 || x >= 35) return 4;
  int i = (int)x; float f = x - i;
  return T[i] + (T[i + 1] - T[i]) * f;
}

struct GS {
  Io io; Tuning tu;
  // motor layer: want* = what the logic asks for, out* = what the bridge gets (ramped, reversal gap enforced)
  int wantDir = 0, wantDuty = 0, outDir = 0, lastDir = 0; float outDuty = 0; uint32_t stopT = 0, startT = 0;
  // sensors
  bool inited = false, encOk = false; int encMiss = 0, lastRaw = 0; int32_t count = 0;
  float phiU = 0; bool phiKnown = false; float turnsF = 0, turnsB = 0;
  float lineOut = 0;                     // m of line off the spool
  uint8_t sw = 0, swPrev = 0, swCand = 0; uint32_t swCandT = 0; float swCandPhiU = 0;   // phi when the raw switch changed
  float iA = 0, iF = 0, peakA = 0, liftPeak = 0; int tripN = 0;
  float tension = 0, tCur = 0, hxOff = 0, hxN = 0, hxLastRaw = 0; uint32_t hxT = 0; bool hxSeen = false;
  int32_t stallRef = 0; uint32_t stallT = 0;
  // state
  int state = IDLE, prevState = IDLE, cyc = C_OFF; uint32_t stT = 0, condT = 0, jogUntil = 0;
  float target = 0; bool swSeen = false, swOpened = false, shot = false, liftSeen = false, dropSeen = false, holding = false, tripSeen = false, finishing = false; uint32_t dropT = 0;
  bool probCocked = false, swOpenSeen = false; bool forceF = false; uint32_t forceT = 0;   // probCocked: SW_COCKED closed again after an opening, rail down
  bool rawOk = false, jogOpened = false, cockProb = false, lowDown = false; char armKey = 0; uint32_t armT = 0;
  float savedPhiU = -1e9f; bool savedKnown = false, savedProb = false;
  uint32_t shotT = 0; float lineAtShot = 0, towDist = 0, towLine0 = 0, cycleTowDist = 0;
  bool anchored = false, released = false; int jogSpd = 160;
  char lastEvent[72] = "boot", fault[48] = "";

  void ev(const char* s) { strncpy(lastEvent, s, sizeof(lastEvent) - 1); lastEvent[sizeof(lastEvent) - 1] = 0; if (io.log) io.log(s); }
  float phi() const { return wrap360(phiU); }
  float theta() const { return phiKnown ? thetaOf(phi()) : (closed(SW_LOW) ? 4.f : -1.f); }
  bool closed(uint8_t b) const { return sw & b; }
  bool closedEdge(uint8_t b) const { return (sw & b) && !(swPrev & b); }
  bool openedEdge(uint8_t b) const { return !(sw & b) && (swPrev & b); }
  bool running() const { return state == COCKING || state == FIRING || state == RETRIEVE || (state == TOW && !holding) || state == JOG || state == LOWER; }
  bool useHx(uint32_t now) const { return hxSeen && now - hxT < 500; }
  void markRunning() { if (io.save) io.save(phi(), false, probCocked); savedKnown = false; }   // flash says "unknown" until this run ends
  void reZero(float ref) { phiU += wrap180(ref - phi()); phiKnown = true; }
  void reZeroAtEdge(float ref) { phiU += wrap180(ref - wrap360(swCandPhiU)); phiKnown = true; }   // undo the debounce delay
  // the sketch calls this before the first tick with what save() stored
  void restore(float p, bool known, bool prob) { phiU = savedPhiU = p; phiKnown = savedKnown = known; probCocked = savedProb = prob; }
  bool cockedPhi() const { float p = phi(); return phiKnown && p >= tu.phiDrawn1 && p <= tu.phiCockMax; }
  // stopped: does phi agree with the switches? (a power cut mid-run leaves the last stop's phi in flash)
  bool phiAgrees() const {
    if (!phiKnown) return true;
    float p = phi(), th = thetaOf(p);
    if (!closed(SW_LOW) && th < 4.05f) return false;                   // rail up, phi says on the cam base circle
    if (closed(SW_LOW) && th > 8) return false;                         // rail down, phi says lifted
    if (closed(SW_LOW) && !closed(SW_COCKED) && !(p >= tu.phiSwOpen - tu.keelWinDeg && p <= tu.phiSwCocked + 1)) return false;   // hook down, rail down: keel on the hook only
    if (p >= tu.phiSwCocked + 1 && p <= tu.phiCockMax && !closed(SW_COCKED)) return false;   // hook is always up in the dwell
    return true;
  }
  void checkPhi(const char* where) { if (!phiAgrees()) { phiKnown = false; char b[72]; snprintf(b, sizeof b, "%s: phi disagrees with the switches, now unknown", where); ev(b); } }
  // jog b / B with phi unknown: refused in the cocked or lifting zone, when probably cocked, and whenever the rail is up
  bool bRefused() const { float p = phi(); return (phiKnown && p >= tu.phiDrawn1 && p <= tu.phiFireEnd) || (!phiKnown && probCocked) || !closed(SW_LOW); }
  void rebase() {                                  // keep phiU small (float precision); stopped only
    float d = phi() - phiU; phiU += d; savedPhiU += d; target += d; swCandPhiU += d;
  }
  bool partlyDrawn() const { float p = phi(); return phiKnown && (p > tu.phiDrawn0 || p < tu.phiDrawn1); }
  bool fOk(uint32_t now) {                         // F with the carriage partly drawn would only be held by backstop C
    if (!partlyDrawn()) return true;
    if (forceF && now - forceT < 10000) { forceF = false; return true; }
    ev("F refused: carriage partly drawn (phi 340..241). cock first, or force"); return false;
  }

  // ---------- motor layer ----------
  void want(int d, int duty) { wantDir = d; wantDuty = d ? duty : 0; }
  void halt(uint32_t now) {                          // immediate stop (BTS7960 brakes, worm self-locks)
    wantDir = 0; wantDuty = 0;
    if (outDir) stopT = now;
    outDir = 0; outDuty = 0; if (io.setMotor) io.setMotor(0, 0);
  }
  void motorTick(uint32_t now, uint32_t dt) {
    if (outDir && (wantDir != outDir || wantDuty == 0)) { int wd = wantDir, wu = wantDuty; halt(now); wantDir = wd; wantDuty = wu; }   // keep a pending new direction
    if (!outDir && wantDir && wantDuty > 0 && (lastDir == 0 || wantDir == lastDir || now - stopT >= tu.reverseGapMs)) {
      outDir = lastDir = wantDir; outDuty = 0; startT = now; stallT = now; stallRef = count;
    }
    if (outDir) {
      if (wantDuty > outDuty) { outDuty += 255.f * dt / tu.rampMs; if (outDuty > wantDuty) outDuty = (float)wantDuty; }
      else outDuty = (float)wantDuty;
    }
    if (io.setMotor) io.setMotor(outDir, (int)outDuty);
  }

  // ---------- sensors ----------
  void readSensors(uint32_t now) {
    uint8_t raw = io.switches ? io.switches() : 0;
    if (!inited) { sw = swPrev = swCand = raw; }
    swPrev = sw;
    if (raw != swCand) { swCand = raw; swCandT = now; swCandPhiU = phiU; }
    else if (swCand != sw && now - swCandT >= tu.debounceMs) sw = swCand;
    if (openedEdge(SW_COCKED)) { probCocked = false; swOpenSeen = true; }
    // keel passed: SW_COCKED closes with the rail already down. After a shot the trip cord goes slack while the rail is
    // still up (SW_LOW closes later), so that closing does not mean cocked
    if (closedEdge(SW_COCKED) && swOpenSeen && closed(SW_LOW) && (swPrev & SW_LOW)) probCocked = true;

    int r = io.angleRaw ? io.angleRaw() : -1;
    if (r < 0) {
      if (++encMiss > 3) {
        // lost during (or just after) a B run: the PG moved uncounted, so phi is no longer known
        if (encOk && phiKnown && (outDir < 0 || (lastDir < 0 && now - stopT < 500))) { phiKnown = false; ev("AS5600 lost during a B run: phi unknown"); }
        encOk = false;
      }
    }
    else {
      if (!rawOk || (!encOk && encMiss > 3)) lastRaw = r;   // first reading, or came back: do not count the jump
      rawOk = true;
      encMiss = 0; encOk = true;
      int d = r - lastRaw; if (d > 2047) d -= 4096; if (d < -2048) d += 4096;
      lastRaw = r; d *= tu.encSign; count += d;
      float dT = d / 4096.f;
      if (lastDir < 0) {                                                       // PG moves only on B (bearing A locks)
        // planet carrier: the PG rides on the rail round the pinion, so while the cam lifts the rail a shaft turn moves the
        // PG by 1 / (pgRatio - dtheta/dphi) instead of 1 / pgRatio (only while the rail is up: SW_LOW open)
        float fp = 0;
        if (phiKnown && !closed(SW_LOW)) { float p0 = phi(); fp = (thetaOf(p0 + 0.5f) - thetaOf(p0)) / 0.5f; if (fp < 0) fp = 0; }
        phiU += -dT * 360.f / (tu.pgRatio - fp); turnsB += -dT;
      }
      else if (lastDir > 0) {                                                  // reel rotor winds only on F
        turnsF += dT;
        float stored = tu.lineTotalM - lineOut;
        lineOut -= dT * tu.rotorRatio * (float)M_PI * layerD(stored, tu) / 1000.f;
        if (lineOut < 0) lineOut = 0; if (lineOut > tu.lineTotalM) lineOut = tu.lineTotalM;
      }
    }

    float v = io.isVolts ? io.isVolts() : 0;
    iA = v * tu.isAmpsPerVolt - tu.isOffsetA; if (iA < 0) iA = 0;
    if (outDir && outDuty >= 60) iA *= 255.f / outDuty;   // BTS7960 IS only reports while the high side conducts: scale back to motor current
    iF += (iA - iF) * 0.2f;
    if (outDir && iF > peakA) peakA = iF;

    float hx = io.hxRaw ? io.hxRaw() : NAN;
    if (!isnan(hx)) { hxLastRaw = hx; hxN = (hx - hxOff) * tu.hxNPerCount; hxT = now; hxSeen = true; }
    bool winding = outDir > 0 && now - startT > tu.tensionIgnoreMs;
    float tc = winding ? (iF - tu.iFreeA) * tu.kNperA : 0; if (tc < 0) tc = 0;
    tCur = winding ? tCur + (tc - tCur) * 0.2f : (holding ? tCur : 0);   // holding: keep the last estimate
    tension = useHx(now) ? hxN : tCur;
    inited = true;
  }

  // ---------- faults ----------
  void doFault(const char* why, uint32_t now) {
    halt(now); cyc = C_OFF; holding = false; state = FAULT;
    strncpy(fault, why, sizeof(fault) - 1); fault[sizeof(fault) - 1] = 0;
    char b[72]; snprintf(b, sizeof b, "FAULT: %s", why); ev(b);
  }
  bool needEnc(uint32_t now) { if (encOk) return true; doFault("AS5600 missing", now); return false; }

  // READY if phi is in the cocked dwell and SW_COCKED (hook up) and SW_LOW are closed, otherwise IDLE
  void settle(uint32_t now) {
    halt(now); holding = false; rebase();
    // SW_COCKED is also closed at rest (hook up, carriage forward), so READY needs phi in the cocked dwell too
    state = closed(SW_COCKED) && closed(SW_LOW) && cockedPhi() ? READY : IDLE;
    stT = now;
  }
  bool canStart() const { return state == IDLE || state == READY || state == PAYOUT || (state == TOW && holding); }

  // ---------- actions ----------
  bool startCock(uint32_t now) {
    if (!canStart()) { ev("busy: stop first"); return false; }
    if (!needEnc(now)) return false;
    if (closed(SW_COCKED) && closed(SW_LOW) && cockedPhi()) { settle(now); ev("already cocked"); return true; }
    float p = phi();
    // in the dwell past 239.8 the target would wrap to a whole PG turn (lift and fire): never start from there
    if (phiKnown && p >= tu.phiSwCocked + 1 && p <= tu.phiCockMax) { ev("cock refused: phi says cocked but a switch disagrees. Check SW_COCKED / SW_LOW"); return false; }
    if (!closed(SW_LOW)) { ev(phiKnown ? "cock refused: rail is up (SW_LOW open). Use finish" : "cock refused: rail is up, phi unknown. Use lower"); return false; }
    checkPhi("cock"); p = phi();
    if (phiKnown && p > tu.phiCockMax && p < tu.phiDropEnd - 2) { ev("refused: rail is in the lift/drop zone. Use finish"); return false; }
    swSeen = false; holding = false;
    cockProb = !phiKnown && probCocked;            // then the next SW_COCKED opening can only be the trip cord
    swOpened = !closed(SW_COCKED) && !cockProb;    // starting with the keel on the hook (rail down): the next closing is the 241.2 edge
    target = phiKnown ? phiU + wrap360(tu.phiCocked - p) : phiU + 275;   // unknown: search for the sear switch (338 -> 249.5 is 271.5)
    markRunning();
    state = COCKING; stT = now; want(-1, tu.cockDuty); ev("cocking"); return true;
  }
  bool startFire(uint32_t now) {
    if (state != READY) { ev("refused: fire only from READY (cock first)"); return false; }
    if (!(closed(SW_COCKED) && closed(SW_LOW) && cockedPhi())) { settle(now); ev("refused: no longer cocked (switches changed since READY)"); return false; }
    if (tu.requireSeatedToFire && !closed(SW_SEATED)) { ev("refused: dart not seated"); return false; }
    if (!needEnc(now)) return false;
    shot = liftSeen = dropSeen = tripSeen = finishing = false; dropT = 0; peakA = liftPeak = 0; condT = 0; anchored = false;
    target = phiU + wrap360(tu.phiFireEnd - phi());
    markRunning();
    state = FIRING; stT = now; want(-1, tu.fireDuty); ev("firing"); return true;
  }
  // finish: run B to 338 from anywhere in 241..338 (it fires if cocked, with or without a dart). Only on request
  bool startFinish(uint32_t now) {
    if (!canStart()) { ev("busy: stop first"); return false; }
    if (!needEnc(now)) return false;
    checkPhi("finish"); float p = phi();
    if (!phiKnown && !closed(SW_LOW)) { ev("finish refused: phi unknown with the rail up. Use lower"); return false; }
    if (!phiKnown) { ev("finish refused: phi unknown. Use cock (it stops if the rail starts to lift)"); return false; }
    if (p < tu.phiDrawn1 || p >= tu.phiFireEnd) { ev("finish refused: only from phi 241..338"); return false; }
    if (closed(SW_LOW) && p >= tu.phiDropEnd - 2) { ev("finish: already done (rail down after the drop)"); return false; }
    shot = liftSeen = dropSeen = tripSeen = false; finishing = true; peakA = liftPeak = 0; condT = 0;
    tripSeen = p > tu.phiTripWin1 && !closed(SW_LOW) && !closed(SW_COCKED); dropT = 0;   // past the window, cord still taut: it tripped in the stopped run
    liftSeen = !closed(SW_LOW);                       // starting with the rail up: the drop edge still has to be caught
    target = phiU + (tu.phiFireEnd - p);
    markRunning();
    state = FIRING; stT = now; want(-1, tu.fireDuty); ev("finish: running B to 338"); return true;
  }
  // lower: rail up with phi unknown (no other command may move B then). Slow B until SW_LOW closes; that is the drop at
  // 288.5, so phi is re-zeroed there and the run goes on to 338 like a fire run (the pinion catches up with the PG).
  // From the lift side it passes the trip cord: it FIRES if cocked. Only on request. Cocking limits (12 A, 300 ms)
  bool startLower(uint32_t now) {
    if (!canStart()) { ev("busy: stop first"); return false; }
    if (!needEnc(now)) return false;
    checkPhi("lower");
    if (closed(SW_LOW)) { ev("lower refused: rail is already down"); return false; }
    if (phiKnown) { ev("lower refused: phi is known. Use finish"); return false; }
    lowDown = false; holding = false; target = phiU + tu.lowerMaxDeg;
    markRunning();
    state = LOWER; stT = now; want(-1, tu.lowerDuty); ev("lower: B slowly until the rail is down (fires if cocked)"); return true;
  }
  bool startTow(float distM, uint32_t now) {
    if (!canStart()) { ev("busy: stop first"); return false; }
    if (!needEnc(now) || !fOk(now)) return false;
    towDist = distM > 0 ? distM : 0; towLine0 = lineOut; holding = false;
    state = TOW; stT = now; want(+1, tu.towDuty);
    char b[72]; snprintf(b, sizeof b, "tow, limit %.1f N%s", tu.limitN, towDist > 0 ? ", distance set" : ""); ev(b); return true;
  }
  bool startRetrieve(uint32_t now) {
    if (!canStart()) { ev("busy: stop first"); return false; }
    if (!needEnc(now) || !fOk(now)) return false;
    holding = false; condT = 0;
    if (closed(SW_SEATED)) { lineOut = 0; settle(now); ev("dart already seated"); return true; }
    state = RETRIEVE; stT = now; want(+1, tu.retrieveDuty); ev("retrieve"); return true;
  }
  bool startCycle(float towM, uint32_t now) {
    if (state != READY) { ev("refused: cycle starts from READY"); return false; }
    cycleTowDist = towM; anchored = released = false;
    if (!startFire(now)) return false;
    cyc = C_FIRE; ev("cycle: fire"); return true;
  }
  void stopAll(uint32_t now) {
    bool wasCyc = cyc != C_OFF; cyc = C_OFF;
    if (state == FAULT) { halt(now); return; }
    if (state == PAYOUT) { halt(now); state = IDLE; ev("stop"); return; }
    settle(now); ev(wasCyc ? "cycle aborted, stopped" : "stop");
  }

  // command names mirror /cmd?c=...&v=...
  bool cmd(const char* c, float v, uint32_t now) {
    if (!strcmp(c, "stop")) { stopAll(now); return true; }
    if (!strcmp(c, "clear")) { if (state == FAULT) { fault[0] = 0; settle(now); ev("fault cleared"); } return true; }
    if (!strcmp(c, "limit")) { if (v >= 2 && v <= 50) { tu.limitN = v; char b[40]; snprintf(b, sizeof b, "limit %.1f N", v); ev(b); return true; } ev("limit must be 2..50 N (reel gears and grubs)"); return false; }
    if (!strcmp(c, "anchored")) { anchored = true; ev("anchored flag set"); return true; }
    if (!strcmp(c, "released")) { released = true; ev("released flag set"); return true; }
    if (!strcmp(c, "line")) { if (v >= 0 && v <= tu.lineTotalM) { lineOut = v; ev("line estimate set"); return true; } return false; }
    if (!strcmp(c, "zero")) return zero(now);
    if (!strcmp(c, "force")) { forceF = true; forceT = now; ev("force: next F allowed with the carriage partly drawn (10 s)"); return true; }
    if (state == FAULT) { ev("in FAULT: clear first"); return false; }
    if (cyc != C_OFF) { ev("cycle running: stop first"); return false; }
    if (!strcmp(c, "cock")) return startCock(now);
    if (!strcmp(c, "fire")) return startFire(now);
    if (!strcmp(c, "tow")) return startTow(v, now);
    if (!strcmp(c, "retrieve")) return startRetrieve(now);
    if (!strcmp(c, "cycle")) return startCycle(v, now);
    if (!strcmp(c, "finish")) return startFinish(now);
    if (!strcmp(c, "lower")) return startLower(now);
    ev("unknown command"); return false;
  }
  // 'z': motor stopped = zero the current offset and tare the HX711; jogging f = learn the free running current
  bool zero(uint32_t now) {
    char b[72];
    if (!outDir && (state == TOW || cyc != C_OFF)) { ev("z refused while towing or in a cycle: the line may be loaded (HX711 tare)"); return false; }
    if (!outDir) { tu.isOffsetA += iF; iF = 0; hxOff = hxLastRaw; snprintf(b, sizeof b, "zeroed: isOffsetA %.2f A", tu.isOffsetA); }
    else if (outDir > 0 && now - startT > tu.tensionIgnoreMs) { tu.iFreeA = iF; snprintf(b, sizeof b, "learned iFreeA %.2f A", tu.iFreeA); }
    else { snprintf(b, sizeof b, "z: stop the motor, or jog f steadily on a slack line"); ev(b); return false; }
    ev(b); return true;
  }

  // manual keys. f/b = hold-to-run jog (deadman runMs), the rest are commands
  void key(char c, uint32_t runMs, uint32_t now) {
    char armed = armKey; armKey = 0;               // F, E, y need the same key twice within 3 s (a stray Serial byte must not fire)
    switch (c) {
      case ' ': case 's': case 'S': cmd("stop", 0, now); return;
      case 'x': cmd("clear", 0, now); return;
      case 'c': cmd("cock", 0, now); return;
      case 'F': case 'E': case 'y': case 'L': {
        const char* what = c == 'F' ? "fire" : c == 'E' ? "finish" : c == 'L' ? "lower" : "cycle";
        if (armed == c && now - armT < 3000) { cmd(what, 0, now); return; }
        armKey = c; armT = now; char b[72]; snprintf(b, sizeof b, "press %c again within 3 s to %s", c, what); ev(b); return;
      }
      case 't': cmd("tow", 0, now); return;
      case 'r': cmd("retrieve", 0, now); return;
      case 'n': cmd("anchored", 0, now); return;
      case 'm': cmd("released", 0, now); return;
      case 'z': cmd("zero", 0, now); return;
      case 'O': cmd("force", 0, now); return;
      case '+': jogSpd = jogSpd + 25 > 255 ? 255 : jogSpd + 25; return;
      case '-': jogSpd = jogSpd - 25 < 50 ? 50 : jogSpd - 25; return;
      case 'f': case 'b': break;
      default: return;
    }
    int d = c == 'f' ? +1 : -1;
    if (cyc != C_OFF || (state != IDLE && state != READY && state != PAYOUT && state != JOG && !(state == TOW && holding))) { if (state != JOG) ev("jog refused: busy or FAULT"); return; }
    if (d < 0 && state != JOG) checkPhi("jog b");
    if (d < 0 && bRefused()) {
      if (state != JOG) ev("jog b refused: cocked, lifting or rail up (phi 241..338). Use fire or finish"); return;
    }
    if (d > 0 && (state != JOG || wantDir <= 0) && !fOk(now)) return;   // also when a b jog turns into f
    if (!needEnc(now)) return;
    if (state != JOG || d != wantDir) jogOpened = false;
    if (d < 0 && (state != JOG || wantDir >= 0)) markRunning();   // a power cut during a b jog must not restore a stale phi
    if (state != JOG) { prevState = state; state = JOG; holding = false; }
    jogUntil = now + runMs; want(d, jogSpd);
  }

  // ---------- 5 ms tick ----------
  void tick(uint32_t now, uint32_t dt = 5) {
    bool first = !inited;
    readSensors(now);
    if (first) { lastDir = 0; checkPhi("power up"); settle(now); if (!encOk) ev("AS5600 missing"); }

    // safety, every tick, whatever the state
    bool ck = state == COCKING || state == LOWER;   // cocking limits (12 A, 300 ms) for lower too
    tripN = (outDir && iA > (ck ? tu.cockTripA : tu.tripA)) ? tripN + 1 : 0;
    if (tripN >= tu.tripSamples) { doFault("over current", now); return; }
    if (outDir && !encOk) { doFault("AS5600 lost while driving", now); return; }
    if (outDir && outDuty >= tu.stallDutyFrac * 255) {
      int32_t m = count - stallRef; if (m < 0) m = -m;
      if (m >= tu.stallCounts) { stallRef = count; stallT = now; }
      else if (now - stallT >= (ck ? tu.cockStallMs : tu.stallMs)) {
        if (state == TOW) { halt(now); holding = true; ev("tow stalled below the limit: holding (check kNperA)"); }
        else { doFault("stall: no shaft motion", now); return; }
      }
    } else { stallT = now; stallRef = count; }

    // the rail can land after a stop (or a fault) in the 0.2 s free fall: the count then lags the PG by up to 28 deg
    if (!outDir && phiKnown && closedEdge(SW_LOW) && phi() >= tu.phiTop0 && phi() < tu.phiDropEnd) { reZeroAtEdge(tu.phiDropEnd); ev("rail landed while stopped: phi re-zeroed to the drop end"); }
    uint32_t t = now - stT;
    switch (state) {
      case JOG:
        if (outDir < 0 || wantDir < 0) {                     // jog b: the key check alone is not enough, phi moves while it runs
          if (openedEdge(SW_COCKED)) jogOpened = true;
          if (openedEdge(SW_LOW)) { halt(now); reZero(tu.phiLowOpenCock); doFault("rail lifted during jog b: stopped. Use finish", now); break; }
          if (closedEdge(SW_COCKED) && jogOpened && closed(SW_LOW)) { reZeroAtEdge(tu.phiSwCocked); settle(now); ev("jog b stopped: sear hook up behind the keel"); break; }
          if (bRefused()) { settle(now); ev("jog b stopped: reached the cocked zone (phi 241)"); break; }
        }
        if ((int32_t)(now - jogUntil) >= 0) settle(now);   // deadman: key released
        break;
      case COCKING:
        if (openedEdge(SW_COCKED)) {
          if (cockProb) { halt(now); reZero(tu.phiTripOpen); doFault("sear pushed down: launcher was cocked, rail lifting. Use finish", now); break; }
          if (phiKnown && !swOpened && fabsf(wrap180(wrap360(swCandPhiU) - tu.phiSwOpen)) > tu.keelWinDeg) {
            halt(now); phiKnown = false; doFault("sear switch opened at the wrong phi: phi was wrong, now unknown", now); break;
          }
          swOpened = true; ev("sear hook down (keel riding over)");
        }
        if (closedEdge(SW_COCKED) && swOpened && !swSeen) {   // keel passed, hook up behind it: re-zero here
          reZeroAtEdge(tu.phiSwCocked); swSeen = true; target = phiU + (tu.phiCocked - phi()); ev("sear hook up behind the keel");
        }
        if (openedEdge(SW_LOW)) {                              // rail lifting: we started already cocked
          halt(now); reZero(tu.phiLowOpenCock); doFault("rail lifted while cocking: was already cocked. Use finish", now); break;
        }
        if (phiKnown && (swSeen || phi() >= tu.phiCockSlow) && phi() < tu.phiCockMax && phiU > target - 30) want(-1, tu.cockSlowDuty);
        if (phiU >= target) {
          if (swSeen) { settle(now); ev(state == READY ? "cocked: READY" : "cocked but rail not down"); }
          else if (phiU >= target + (tu.phiCockMax - tu.phiCocked)) doFault("sear switch never re-closed", now);
        }
        if (state == COCKING && t > tu.cockMaxMs) doFault("cocking timed out", now);
        break;
      case FIRING: {
        float p = phi();
        if (openedEdge(SW_LOW)) liftSeen = true;
        if (p >= tu.phiCockMax && iF > liftPeak) liftPeak = iF;   // lift region only: start inrush is not the peak
        bool win = p >= tu.phiTripWin0 && p <= tu.phiTripWin1;
        const char* why = nullptr;
        if (win && !closed(SW_COCKED) && !tripSeen) { tripSeen = true; if (!shot) why = "shot: sear tripped (SW_COCKED open)"; }
        // fallback: past the trip angle with the lift load gone. The shot itself does not load the PG, so this only
        // says the rail reached the trip angle; a stuck sear is caught at the end (SW_COCKED never opened)
        bool pastTrip = p >= tu.phiTrip && p < tu.phiFireEnd + tu.fireExtraDeg;
        if (!shot && pastTrip && liftPeak > tu.iFreeA * 1.2f && iF < liftPeak * tu.shotDropFrac) { if (!condT) condT = now; if (now - condT >= tu.shotDropMs && !why) why = "shot: past the trip, lift load gone"; } else condT = 0;
        // rail back down: the PG free-fell to the end of the drop and bearing A (a roller clutch) relocked at once, so the
        // shaft count no longer tells phi. Re-zero to the drop end and stop here: the next cock re-zeroes at the sear switch
        if (liftSeen && !dropSeen && (p >= tu.phiTop0 || tripSeen) && closedEdge(SW_LOW)) { dropSeen = true; reZeroAtEdge(tu.phiDropEnd); target = phiU; if (!shot && !why) why = "shot: rail back down"; }
        // switch sanity: SW_LOW must open as the rail lifts and close at the drop, or phi past here is fiction (rack at 340)
        if (!liftSeen && p >= tu.phiLowOpenCock + 10 && p < 340) { halt(now); doFault("SW_LOW did not open as the rail lifted: check the switch, then finish", now); break; }
        // the free fall takes about 0.2 s while the shaft count runs on, so allow up to 330 before calling the switch dead
        if (liftSeen && !dropSeen && !dropT && p >= tu.phiDropStart && p < 340) dropT = now ? now : 1;
        if (liftSeen && !dropSeen && dropT && now - dropT > tu.dropMaxMs && !closed(SW_LOW)) { halt(now); reZero(tu.phiDropEnd + 2); doFault("SW_LOW did not close at the drop: check the switch", now); break; }
        if (!dropSeen) want(-1, p >= tu.phiTop1 ? tu.dropDuty : p >= tu.phiTop0 ? (tu.topDuty < tu.fireDuty ? tu.topDuty : tu.fireDuty) : tu.fireDuty);
        if (why) { shot = true; shotT = now; lineAtShot = lineOut; ev(why); }
        if (phiU >= target) {
          if (!tripSeen) doFault("sear did not trip", now);
          else if (shot && closed(SW_LOW) && finishing) { settle(now); ev("finished: rail down"); }
          else if (shot && closed(SW_LOW)) { halt(now); rebase(); state = PAYOUT; stT = now; ev("fired: PAYOUT"); }
          else if (phiU >= target + tu.fireExtraDeg) doFault(shot ? "rail not down after the shot" : "shot not seen", now);
        }
        if (state == FIRING && t > tu.fireMaxMs) doFault("firing timed out", now);
        break;
      }
      case LOWER:
        if (openedEdge(SW_COCKED)) { probCocked = false; ev("lower: sear cord pulled (shot, if it was cocked)"); }
        if (!lowDown) {
          if (closed(SW_LOW)) { reZeroAtEdge(tu.phiDropEnd); lowDown = true; target = phiU; ev("lower: rail down, phi re-zeroed to the drop end"); }
          else if (phiU >= target) { doFault("lower: rail did not come down", now); break; }
        } else if (phiU >= target) { settle(now); ev("lowered"); break; }
        if (t > tu.cockMaxMs) doFault("lower timed out", now);
        break;
      case PAYOUT: {
        float el = (now - shotT) / 1000.f * tu.payoutMps; if (el > tu.payoutMaxM) el = tu.payoutMaxM;
        float l = lineAtShot + el; if (l > tu.lineTotalM) l = tu.lineTotalM;
        if (l > lineOut) lineOut = l;
        break;
      }
      case TOW:
        // a hard tow puts a nose-up torque on the rail through the reel drive: if it lifts off the bumpers, stop and hold
        if (!closed(SW_LOW) && !holding) { halt(now); holding = true; ev("rail lifting off the bumpers: tow holding (lower the limit)"); break; }
        if (holding) {
          if (useHx(now) && tension < tu.limitN * tu.resumeFrac) { holding = false; want(+1, tu.towDuty); ev("line slack: winding again"); }
          break;
        }
        if (towDist > 0 && towLine0 - lineOut >= towDist) { halt(now); holding = true; ev("tow distance reached: holding"); break; }
        {
          float frac = tension / tu.limitN;
          if (frac >= 1) { halt(now); holding = true; char b[72]; snprintf(b, sizeof b, "holding at limit %.1f N", tension); ev(b); break; }
          int d = tu.towDuty;
          if (frac > 1 - tu.taperFrac) d = tu.minDuty + (int)((tu.towDuty - tu.minDuty) * (1 - frac) / tu.taperFrac);
          if (!useHx(now) && (!outDir || now - startT <= tu.tensionIgnoreMs)) d = tu.minDuty;   // blind window: creep, the line may already be taut
          want(+1, d);
        }
        if (t > tu.towMaxMs) { halt(now); holding = true; ev("tow timed out: holding"); }
        break;
      case RETRIEVE:
        if (closed(SW_SEATED)) { lineOut = 0; settle(now); ev(state == READY ? "dart seated: READY" : "dart seated (not cocked)"); break; }
        if (now - startT > tu.tensionIgnoreMs && outDir > 0 && iF > tu.retrieveStallA) { if (!condT) condT = now; if (now - condT >= tu.retrieveStallMs) { settle(now); ev("retrieve stopped on current spike (seated switch?)"); break; } }
        else condT = 0;
        want(+1, lineOut < tu.retrieveSlowM ? tu.retrieveSlowDuty : tu.retrieveDuty);
        if (t > tu.retrieveMaxMs) doFault("retrieve timed out", now);
        break;
      default: break;
    }

    // cycle helper: fire -> anchored flag -> tow -> released flag -> cock -> retrieve -> READY
    switch (cyc) {
      case C_FIRE: if (state == PAYOUT) { cyc = C_WAIT_ANCHOR; ev("cycle: waiting for anchored"); } else if (state != FIRING) cyc = C_OFF; break;
      case C_WAIT_ANCHOR: if (anchored) { if (startTow(cycleTowDist, now)) cyc = C_TOW; else cyc = C_OFF; } break;
      case C_TOW: if (state == TOW && holding) { cyc = C_WAIT_RELEASE; released = false; ev("cycle: waiting for released"); } else if (state != TOW) cyc = C_OFF; break;
      case C_WAIT_RELEASE: if (released) { if (startCock(now)) cyc = C_COCK; else cyc = C_OFF; } break;
      case C_COCK: if (state == READY) { if (startRetrieve(now)) cyc = C_RETRIEVE; else cyc = C_OFF; } else if (state != COCKING) cyc = C_OFF; break;
      case C_RETRIEVE: if (state != RETRIEVE) { ev(state == READY ? "cycle complete: READY" : "cycle ended, not READY"); cyc = C_OFF; } break;
      default: break;
    }

    motorTick(now, dt);
    if (!outDir && now - stopT > 100 && (fabsf(phiU - savedPhiU) > tu.saveDeg || phiKnown != savedKnown || probCocked != savedProb)) {
      savedPhiU = phiU; savedKnown = phiKnown; savedProb = probCocked;
      if (io.save) io.save(phi(), phiKnown, probCocked);
    }
  }

  int status(char* buf, int n, uint32_t now) const {
    return snprintf(buf, n,
      "{\"t_ms\":%lu,\"state\":\"%s\",\"cycle\":\"%s\",\"phi\":%.1f,\"phi_known\":%s,\"theta_est\":%.1f,\"turns\":%.3f,\"turns_f\":%.2f,\"turns_b\":%.2f,"
      "\"line_m\":%.2f,\"current_A\":%.2f,\"peak_A\":%.2f,\"tension_N\":%.1f,\"tension_src\":\"%s\",\"limit_N\":%.1f,\"holding\":%s,\"dir\":%d,\"duty\":%d,"
      "\"switches\":{\"low\":%s,\"cocked\":%s,\"seated\":%s},\"enc_ok\":%s,\"anchored\":%s,\"released\":%s,\"shot\":%s,\"fault\":\"%s\",\"event\":\"%s\"}",
      (unsigned long)now, stateName(state), cycleName(cyc), phi(), phiKnown ? "true" : "false", theta(), count / 4096.0, turnsF, turnsB,
      lineOut, iF, peakA, tension, useHx(now) ? "hx711" : "current", tu.limitN, holding ? "true" : "false", outDir, (int)outDuty,
      closed(SW_LOW) ? "true" : "false", closed(SW_COCKED) ? "true" : "false", closed(SW_SEATED) ? "true" : "false",
      encOk ? "true" : "false", anchored ? "true" : "false", released ? "true" : "false", shot ? "true" : "false", fault, lastEvent);
  }
};

}  // namespace gs
