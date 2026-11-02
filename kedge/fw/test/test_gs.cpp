// PC simulation of the GrappStar ground station logic against a plant model of the one-motor launcher + reel.
// cd fw/test && g++ -std=c++17 -O1 -I../gs/grappstar_gs -o test_gs test_gs.cpp && ./test_gs
#include "gs_logic.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>
#include <vector>
using namespace gs;

// ---------- plant ----------
// Numbers from the final CAD after the sear review (gs/out/gs_parts.json, physics_gs.json): springs 42 -> 192 N
// (0.72 N/mm), shoe friction x1.21, sear trip cord (round 6 CAD): taut at 284.8 (rail 25.7), shot at 294.5 (rail 33.0), 0.67 J assumed over that stretch, lift with the planet-carrier factor, launch 10.7 m/s, 9.9 m of line. Wiper as assumed there: 12 V, no load 45 rpm,
// stall 15 N m at 18 A, free 1.5 A. V*duty = I*R + KE*w, T = KT*(I - I0).
static const double VBAT = 12, R = VBAT / 18.0, KT = 15.0 / 16.5, I0 = 1.5, KE = (VBAT - R * I0) / (45 * 2 * M_PI / 60);
static const double RATIO = 30.0 / 14.0, ETA = 0.92, FRIC = 0.2, RR = 58.0 / 39.0, ROT_R = 0.033;
static const double M_LIFT = 0.942 + 0.277 + 0.055, CG_U = 0.170;   // rail assembly + dart + sled (kg), rail CG (m)
// carriage travel (mm) vs PG angle from the exact tooth polygons, every 5 deg from -40 to 245 (then 208.24 to 246.5)
static const double RACK[58] = {0, 0, 0, 0, 0, 5.44, 10.48, 14.78, 18.74, 22.68, 26.60, 30.52, 34.46, 38.38, 42.30, 46.24, 50.16,
  54.08, 58.02, 61.94, 65.86, 69.80, 73.72, 77.66, 81.58, 85.50, 89.44, 93.36, 97.28, 101.22, 105.14, 109.06, 113.00, 116.92,
  120.84, 124.78, 128.70, 132.62, 136.56, 140.48, 144.40, 148.34, 152.26, 156.18, 160.12, 164.04, 167.98, 171.90, 175.82,
  179.76, 183.68, 187.60, 191.54, 195.44, 199.20, 202.72, 205.98, 208.24};
struct Plant {
  double ms = 0;            // main shaft, turns (+ = F)
  double pin = 338;         // PG angle the pinion has driven to (deg, unwrapped)
  double pg = 338;          // true PG angle (deg, unwrapped): runs ahead of pin in the drop (overrun on bearing A)
  bool cocked = false, seated = true, fired = false, searStuck = false;   // cocked = carriage held behind the hook
  double lineOut = 0, dartD = 0, flyTo = 0;  // m
  bool anchoredDart = false; double holdN = 45, kLine = 300, dragN = 3;
  bool jitter = false, blocked = false, encMissing = false, shortCircuit = false, swCockBroken = false, swSeatBroken = false, lowStuck = false, hx = false;
  int dir = 0, duty = 0; double I = 0, tension = 0, overwind = 0, extraI = 0;
  int hxDiv = 0;
} P;
static uint32_t NOW = 0;
static std::vector<std::string> LOG;
static int lastF = -100000, lastB = -100000; static int gapViolations = 0, minGap = 1 << 30, rampViolations = 0; static int prevDuty = 0, prevDir = 0;

static double pgPhase() { return fmod(fmod(P.pg, 360) + 360, 360); }
static double rackQ(double p) { return p >= 300 ? p - 360 : p; }     // 300..360 = -60..0 (before the first tooth)
static double rackDraw(double q) {
  if (q <= -40) return 0; if (q >= 245) return 208.24;
  double x = (q + 40) / 5; int i = (int)x; return RACK[i] + (RACK[i + 1] - RACK[i]) * (x - i);
}
static double plantLayerD(double stored) {   // independent copy of the spool rule (mm): core 36, 1 mm line, 21 turns a layer
  double s = stored * 1000;
  for (int n = 0; n < 40; n++) { double d = 36 + (2 * n + 1) * 1.0, len = 21 * M_PI * d; if (s < len) return d; s -= len; }
  return 117;
}
static double loadTorque() {                 // N.m at the main shaft, resisting motion
  if (P.dir < 0) {
    if (P.pin < P.pg - 0.01) return FRIC;      // pinion catching up after the overrun: free
    double p = pgPhase(), q = rackQ(p), tpg = 0.3;
    if (q > -40 && q < 246.5 && !(P.cocked && q > 245)) {   // teeth drawing the carriage: F dd/dphi
      double d = rackDraw(q), dd = (rackDraw(q + 0.5) - rackDraw(q - 0.5)) / 1000, F = (42 + 0.72 * d) * 1.21;   // shoe friction
      tpg += F * dd / (M_PI / 180) / ETA;   // dd over 1 deg
    }
    double th0 = thetaOf((float)p - 0.5f) * M_PI / 180, th1 = thetaOf((float)p + 0.5f) * M_PI / 180;   // CAD cam table
    double dU = M_LIFT * 9.81 * CG_U * (sin(th1) - sin(th0));
    double lift = 0;
    if (dU > 0) lift += dU / (M_PI / 180) / 0.85;
    if (P.cocked && p >= 284.8 && p < 294.5) lift += 0.67 / (9.7 * M_PI / 180);   // trip cord pulling the sear arm down (energy assumed)
    double fp = (thetaOf((float)p + 0.5f) - thetaOf((float)p)) / 0.5; if (fp < 0) fp = 0;
    tpg += lift * RATIO / (RATIO - fp);                                           // planet carrier: the rail rides round the pinion
    return tpg / RATIO / ETA + FRIC;
  }
  if (P.dir > 0) return P.tension * ROT_R * 2.0 * RR / ETA + FRIC;   // wrap friction x2.0 (pusher hole, eye, face edge), as physics_gs.py
  return 0;
}
static void updateTension() {
  if (P.anchoredDart) {
    double t = P.kLine * (P.dartD - P.lineOut);
    if (t > P.holdN) { P.dartD = P.lineOut + P.holdN / P.kLine; t = P.holdN; }   // dart pulls through the soil
    P.tension = t > 0 ? t : 0; return;
  }
  if (P.seated) { P.tension = P.overwind > 0 ? P.dragN + 20000 * P.overwind : 0; return; }
  if (P.lineOut < P.dartD) P.dartD = P.lineOut;                                  // dragged home
  if (P.dartD <= 0.001 && !P.seated && P.fired && P.flyTo == 0) { P.seated = true; P.dartD = 0; }
  P.tension = P.dartD > 0 && P.lineOut <= P.dartD + 1e-6 ? P.dragN : 0;
}
static double peakMainT = 0, peakAt = 0;
static void plantStep(double dt) {
  // flight: line pays out off the end of the stationary spool (CAD flight: 12.4 m out, dart lands at 10.8 m)
  if (P.flyTo > 0) { P.lineOut += 10.8 * dt; P.dartD = P.lineOut; if (P.lineOut >= P.flyTo) { P.lineOut = P.flyTo; P.dartD = 10.0; P.flyTo = 0; } }
  updateTension();
  double V = VBAT * P.duty / 255.0, T = loadTorque(), w = 0;
  if (P.dir < 0 && P.duty && T > peakMainT) { peakMainT = T; peakAt = pgPhase(); }
  if (!P.dir || !P.duty) { P.I = 0; }
  else {
    w = (V - R * (T / KT + I0)) / KE;
    if (w <= 0 || P.blocked) { w = 0; P.I = V / R; } else P.I = (V - KE * w) / R;
  }
  if (P.shortCircuit) P.I = 30;
  if (P.dir && P.duty) P.I += P.extraI;
  double dturn = w / (2 * M_PI) * dt * (P.dir > 0 ? 1 : -1);
  P.ms += dturn;
  if (P.dir < 0) {
    double pp = pgPhase(), fp = (thetaOf((float)pp + 0.5f) - thetaOf((float)pp)) / 0.5;
    if (fp < 0 || P.pin < P.pg - 0.01) fp = 0;                    // planet carrier: only while the pinion drives the lifting rail
    double dpg = -dturn * 360 / (RATIO - fp); P.pin += dpg;
    if (P.pin >= P.pg) {
      double before = pgPhase(); P.pg = P.pin; double p = pgPhase();
      if (!P.cocked && before < 241.2 && p >= 241.2 && p < 280) P.cocked = true;             // keel passed, hook up behind it
      if (before < 294.5 && p >= 294.5 && p < 310 && P.cocked && !P.searStuck) {              // trip cord lets the carriage go
        bool home = P.seated || (P.fired && P.flyTo == 0 && !P.anchoredDart && P.dartD < 0.004);
        P.cocked = false; P.fired = true; if (home) { P.seated = false; P.flyTo = 11.5; }
      }
      if (before < 304.5 && p >= 304.5 && p < 320) { P.pg += 333.5 - p; P.pin = P.pg; }     // rail free-falls, PG overruns to about 333.5; roller clutch relocks at once
    }
  }
  if (P.dir > 0) {
    double stored = 20 - P.lineOut, dl = dturn * RR * M_PI * plantLayerD(stored) / 1000;
    if (P.seated && P.lineOut <= 0) P.overwind += dl; else { P.lineOut -= dl; if (P.lineOut < 0) { P.overwind += -P.lineOut; P.lineOut = 0; } }
  }
}
// ---------- Io ----------
static void setM(int d, int duty) {
  if (d != prevDir && d != 0) {
    int last = d > 0 ? lastB : lastF; int gap = (int)NOW - last;
    if (prevDir == 0 && gap < 200 && ((d > 0 && lastB > lastF) || (d < 0 && lastF > lastB))) gapViolations++;
    if ((d > 0 && lastB > lastF) || (d < 0 && lastF > lastB)) if (gap < minGap) minGap = gap;
  }
  if (d && d == prevDir && duty - prevDuty > 5) rampViolations++;
  if (d && prevDir == 0 && duty > 5) rampViolations++;
  if (d > 0 && duty) lastF = NOW; if (d < 0 && duty) lastB = NOW;
  P.dir = d; P.duty = duty; prevDir = duty ? d : 0; prevDuty = duty;
}
static int angle() { if (P.encMissing) return -1; long c = (long)floor(P.ms * 4096) + (P.jitter ? rand() % 5 - 2 : 0); return (int)(((c % 4096) + 4096) % 4096); }
static float isV() { return (float)(P.I * (P.dir ? P.duty / 255.0 : 1.0) / 12.5 + 0.004 * sin(NOW * 0.37)); }   // IS averages over the PWM; a little ADC noise
static float hxRaw() { if (!P.hx) return NAN; if (++P.hxDiv < 20) return NAN; P.hxDiv = 0; return (float)(5000 + P.tension * 21000); }   // 10 SPS
static uint8_t sws() {
  double p = pgPhase(); uint8_t b = 0;
  if (thetaOf((float)p) < 4.5f || P.lowStuck) b |= SW_LOW;   // rail within 0.5 deg of down
  // SW_COCKED: KW11 on COM + NO outside the left cheek, pressed by the sear arm while the hook is up on its stop
  bool keelOnHook = p >= 215.2 && p < 241.2;                                       // only reached while cocking
  bool tripPush = p >= 252.5 && p < 338.5 && thetaOf((float)p) >= 25.7 && !P.searStuck;   // trip cord taut from rail 25.7 deg (phi 284.8)
  if (!keelOnHook && !tripPush && !P.swCockBroken) b |= SW_COCKED;
  bool near = P.seated || (P.fired && P.flyTo == 0 && !P.anchoredDart && P.dartD < 0.004);   // tail within 4 mm of the pusher
  if (near && !P.swSeatBroken) b |= SW_SEATED;
  return b;
}
static void lg(const char* s) { char b[128]; snprintf(b, sizeof b, "%7u ms  %s", NOW, s); LOG.push_back(b); }

static float SV_PHI = -1; static bool SV_KNOWN = false, SV_PROB = false; static int SV_N = 0;
static void saveCb(float p, bool k, bool pr) { SV_PHI = p; SV_KNOWN = k; SV_PROB = pr; SV_N++; }
static GS make() {
  P = Plant(); NOW = 0; LOG.clear(); lastF = lastB = -100000; prevDir = prevDuty = 0; SV_N = 0;
  GS g; g.io = {setM, angle, isV, hxRaw, sws, lg, saveCb}; return g;
}
static bool seatedSw() { return sws() & SW_SEATED; }
static double maxTension = 0;
static void run(GS& g, uint32_t ms) { for (uint32_t i = 0; i < ms; i += 5) { NOW += 5; for (int k = 0; k < 5; k++) plantStep(0.001); if (P.tension > maxTension) maxTension = P.tension; g.tick(NOW); } }
template <class F> static bool runUntil(GS& g, uint32_t ms, F cond) { for (uint32_t i = 0; i < ms; i += 5) { run(g, 5); if (cond()) return true; } return false; }

static int fails = 0;
#define CHECK(c, msg) do { if (!(c)) { printf("FAIL: %s\n", msg); fails++; } else printf("ok:   %s\n", msg); } while (0)
static bool has(const char* s) { for (auto& l : LOG) if (l.find(s) != std::string::npos) return true; return false; }
static void dump() { for (auto& l : LOG) printf("   %s\n", l.c_str()); }
static double phiErr(GS& g) { double e = fmod(g.phi() - pgPhase() + 540, 360) - 180; return fabs(e); }
// as Harry would: learn the free current on a slack line, then kNperA for the middle of the spool
static void calibrate(GS& g) { g.tu.iFreeA = (float)(I0 + FRIC / KT); g.tu.kNperA = (float)(KT / (ROT_R * 2.0 * RR / ETA)); }
static void toReady(GS& g) { run(g, 50); g.cmd("cock", 0, NOW); runUntil(g, 15000, [&] { return g.state != COCKING; }); }

int main() {
  // spool geometry
  { Tuning t; CHECK(fabsf(layerD(0, t) - 37) < 0.01f && fabsf(layerD(19.99f, t) - 51) < 0.01f, "spool: empty layer D37, full (20 m) layer D51 (8 layers of 21 turns)");
    double perTurnFull = RR * M_PI * 49 / 1000; printf("   line per main-shaft F turn: %.3f m full, %.3f m empty\n", perTurnFull, RR * M_PI * 37 / 1000); }

  // boot + cock from rest (phi unknown, found by the sear switch)
  { GS g = make(); run(g, 50);
    CHECK(g.state == IDLE && !g.phiKnown && g.encOk, "boot at rest: IDLE, phi unknown until a switch is seen");
    g.cmd("cock", 0, NOW); runUntil(g, 15000, [&] { return g.state != COCKING; }); dump();
    CHECK(g.state == READY && P.cocked, "cock: B until the sear switch, stop at COCKED -> READY");
    CHECK(fabs(pgPhase() - 249.5) < 2 && phiErr(g) < 1.5, "cock: stops at phi 249.5 in the dwell (plant within 2 deg, tracking within 1.5 deg)");
    printf("   peak main-shaft torque while cocking %.2f N m at PG %.1f (CAD 5.90)\n", peakMainT, peakAt);
    CHECK(peakMainT > 5.3 && peakMainT < 6.5, "plant: peak cocking torque matches the CAD (about 5.9 N m)");
    CHECK(has("hook down") && has("hook up behind"), "cock: SW_COCKED opens (keel on the hook) then closes (re-zero at 241.2)");
    run(g, 200); CHECK(SV_N > 0 && fabsf(SV_PHI - g.phi()) < 0.1f && SV_KNOWN, "phi saved for the next power up after the stop");
    printf("   plant PG %.1f deg, logic phi %.1f, theta_est %.1f\n", pgPhase(), g.phi(), g.theta());
    P.seated = false; P.fired = true; P.lineOut = P.dartD = 3; run(g, 50);   // some line out, so winding does not pull a seated dart
    double phi0 = g.phi(), pg0 = pgPhase();
    for (int i = 0; i < 14; i++) { g.key('f', 350, NOW); run(g, 150); }
    run(g, 600);
    printf("   f jog: turnsF %.2f, phi %.2f -> %.2f, plant %.2f -> %.2f, state %s\n", g.turnsF, phi0, g.phi(), pg0, pgPhase(), stateName(g.state));
    CHECK(g.turnsF > 0.5f && fabs(g.phi() - phi0) < 0.01 && fabs(pgPhase() - pg0) < 0.01, "phi ignores F rotation (2 s of reel wind, phi unchanged)");
    CHECK(g.state == READY, "after the f jog: still READY (switches say cocked)");
    P.seated = true; P.fired = false; P.lineOut = P.dartD = 0; run(g, 50); g.cmd("stop", 0, NOW);
    CHECK(g.state == READY, "dart seated again: READY");
    g.key('b', 350, NOW); run(g, 50); CHECK(P.dir == 0 && has("jog b refused"), "jog b refused in READY (would lift and fire)");
    P.seated = false; run(g, 50); g.cmd("fire", 0, NOW); run(g, 50); CHECK(g.state == READY && P.dir == 0, "fire refused when the dart is not seated"); P.seated = true; run(g, 50);
    LOG.clear(); g.cmd("fire", 0, NOW); runUntil(g, 10000, [&] { return g.state != FIRING; }); dump();
    CHECK(g.state == PAYOUT && P.fired, "fire: B from COCKED, shot detected, stop -> PAYOUT");
    CHECK(phiErr(g) < 1.5 && pgPhase() < 340 && (sws() & SW_LOW), "fire: stops as the rail lands, phi re-zeroed at the SW_LOW edge (tracking within 1.5 deg), before the first tooth (340)");
    CHECK(has("shot: "), "fire: shot evidence logged");
    run(g, 100); float l1 = g.lineOut; run(g, 1000);
    CHECK(g.lineOut > l1 && g.lineOut <= g.tu.payoutMaxM, "payout: line estimate grows with time, capped");
    printf("   phi after fire %.1f (plant %.1f), line est %.1f m (plant %.1f m)\n", g.phi(), pgPhase(), g.lineOut, P.lineOut); }

  // cocking from 350 with phi known: count + switch
  { GS g = make(); toReady(g); g.cmd("fire", 0, NOW); runUntil(g, 10000, [&] { return g.state != FIRING; }); run(g, 2000);
    g.cmd("stop", 0, NOW); P.seated = true; P.lineOut = 0; P.flyTo = 0; run(g, 300);   // pretend the dart was put back
    LOG.clear(); g.cmd("cock", 0, NOW); runUntil(g, 15000, [&] { return g.state != COCKING; });
    CHECK(g.state == READY && fabs(pgPhase() - 249.5) < 2 && phiErr(g) < 1.5, "re-cock after a shot: 333 through 360 to 249.5, READY");
    P.cocked = false; P.pg = P.pin = 330.5; run(g, 50); g.phiU += 360 * 5 + 330.5 - g.phi(); g.state = IDLE; LOG.clear(); g.cmd("cock", 0, NOW);
    CHECK(g.state == IDLE && P.dir == 0 && has("lift/drop zone"), "cock refused when phi is in the lift/drop zone (252.5..331, rail down)");
    P.pg = P.pin = 300; run(g, 50); g.phiU += 300 - g.phi(); LOG.clear(); g.cmd("cock", 0, NOW); run(g, 50);
    CHECK(g.state == IDLE && P.dir == 0 && has("rail is up"), "cock refused while the rail is up (SW_LOW open)"); }

  // rail-down switch stuck closed: the sear opening alone is the shot evidence
  { GS g = make(); toReady(g); P.lowStuck = true; run(g, 50); LOG.clear();
    g.cmd("fire", 0, NOW); runUntil(g, 10000, [&] { return g.state != FIRING; });
    CHECK(g.state == FAULT && strstr(g.fault, "did not open") && pgPhase() < 300, "SW_LOW stuck closed: FAULT early in the lift (phi past it would be fiction), never into the rack"); }

  // sear stuck: the rail goes up and down but the carriage stays on the sear
  { GS g = make(); toReady(g); P.searStuck = true; LOG.clear(); g.cmd("fire", 0, NOW); runUntil(g, 10000, [&] { return g.state != FIRING; });
    CHECK(g.state == FAULT && strstr(g.fault, "sear did not trip") && P.dir == 0 && pgPhase() < 340, "sear stuck (SW_COCKED never opens 270..292): FAULT at 338, before the first tooth"); }

  // sear switch broken: fault, no run-on into the lift
  { GS g = make(); P.swCockBroken = true; run(g, 50); g.phiKnown = true; g.phiU = 338; g.cmd("cock", 0, NOW); runUntil(g, 15000, [&] { return g.state != COCKING; });
    printf("   broken sear switch: stopped at PG %.1f, rail %.2f deg\n", pgPhase(), thetaOf((float)pgPhase()));
    CHECK(g.state == FAULT && thetaOf((float)pgPhase()) < 5, "sear switch never closes: FAULT before the rail lifts"); }

  // tow with the current estimate: taper and limit
  { GS g = make(); calibrate(g); toReady(g); g.cmd("fire", 0, NOW); runUntil(g, 10000, [&] { return g.state != FIRING; }); run(g, 2500);
    P.anchoredDart = true; g.cmd("line", 11.5f, NOW); LOG.clear(); maxTension = 0;
    g.cmd("tow", 0, NOW); int dutyAt = -1;
    runUntil(g, 120000, [&] { if (dutyAt < 0 && P.tension > 0.88 * g.tu.limitN) dutyAt = P.duty; return g.holding || g.state != TOW; }); dump();
    printf("   plant tension at hold %.1f N (max %.1f), estimate %.1f N, duty when tension passed 88%%: %d of %d\n", P.tension, maxTension, g.tension, dutyAt, g.tu.towDuty);
    CHECK(g.state == TOW && g.holding && P.dir == 0, "tow: stops and holds at the limit (worm self-locks)");
    CHECK(P.tension > 15 && maxTension < 25, "tow: plant line tension 15..25 N for a 20 N limit");
    CHECK(dutyAt >= 0 && dutyAt < g.tu.towDuty, "tow: speed tapers over the last 30% of the limit");
    CHECK(fabs(P.dartD - 10.0) < 0.01, "tow: dart (45 N holding) never pulled out");
    double t0 = P.tension; g.cmd("limit", 10, NOW); g.cmd("tow", 0, NOW); runUntil(g, 5000, [&] { return g.holding; }); run(g, 50);
    CHECK(g.holding && P.tension - t0 < 6, "re-tow at a lower limit from a taut line: holds at once (blind start window < 6 N)");
    g.cmd("limit", 500, NOW); CHECK(g.tu.limitN == 10, "limit outside 2..150 N refused"); }

  // tow with HX711: preferred, accurate, resumes when the line slackens
  { GS g = make(); P.hx = true; toReady(g); g.cmd("zero", 0, NOW); g.cmd("fire", 0, NOW); runUntil(g, 10000, [&] { return g.state != FIRING; }); run(g, 2500);
    P.anchoredDart = true; maxTension = 0; g.cmd("tow", 0, NOW); runUntil(g, 120000, [&] { return g.holding; });
    char b[700]; g.status(b, sizeof b, NOW);
    printf("   HX711: hold at plant %.1f N (max %.1f)\n", P.tension, maxTension);
    CHECK(std::string(b).find("\"tension_src\":\"hx711\"") != std::string::npos, "HX711 present: used for tension");
    CHECK(g.holding && P.tension > 17 && maxTension < 24.5, "HX711 tow: holds 17..24.5 N for a 20 N limit");
    { float off0 = g.hxOff; bool z = g.cmd("zero", 0, NOW); run(g, 200);
      CHECK(!z && g.hxOff == off0 && g.holding && P.dir == 0, "review fix: z (HX711 tare) refused while holding a loaded line"); }
    P.dartD -= 0.06; run(g, 400);
    CHECK(g.lastEvent[0] && has("winding again"), "HX711: line slackens below 60% -> winds again");
    runUntil(g, 30000, [&] { return g.holding && P.dir == 0; }); CHECK(g.holding && P.tension > 15, "HX711: holds again at the limit"); }

  // tow to a distance (dart not anchored, drags)
  { GS g = make(); calibrate(g); toReady(g); g.cmd("fire", 0, NOW); runUntil(g, 10000, [&] { return g.state != FIRING; }); run(g, 2500);
    g.cmd("line", (float)P.lineOut, NOW); double l0 = P.lineOut; g.cmd("tow", 2.0f, NOW); runUntil(g, 60000, [&] { return g.holding; });
    printf("   tow 2 m: plant wound %.3f m\n", l0 - P.lineOut);
    CHECK(g.holding && fabs((l0 - P.lineOut) - 2.0) < 0.1, "tow distance 2 m: stops within 0.1 m (spool fill model)"); }

  // retrieve: seated switch, then broken switch -> current spike
  { GS g = make(); calibrate(g); toReady(g); g.cmd("fire", 0, NOW); runUntil(g, 10000, [&] { return g.state != FIRING; }); run(g, 2500);
    g.cmd("cock", 0, NOW); runUntil(g, 15000, [&] { return g.state != COCKING; });
    CHECK(g.state == IDLE || g.state == READY, "cock with the dart out (line free on B)");
    double lineBefore = P.lineOut; LOG.clear(); g.cmd("retrieve", 0, NOW); runUntil(g, 150000, [&] { return g.state != RETRIEVE; }); dump();
    CHECK(seatedSw() && g.state == READY && P.dir == 0, "retrieve: F until SW_SEATED -> READY");
    printf("   retrieve stopped with the tail %.1f mm from the pusher\n", P.dartD * 1000);
    CHECK(P.overwind < 0.001 && P.dartD <= 0.004, "retrieve: stops within 4 mm of the pusher (switch 4 mm ahead), no load on the tail");
    CHECK(fabs(lineBefore - 11.5) < 0.01, "plant flight: 11.5 m of line out"); }
  { GS g = make(); calibrate(g); P.swSeatBroken = true; toReady(g); g.cmd("fire", 0, NOW); runUntil(g, 10000, [&] { return g.state != FIRING; }); run(g, 2500);
    LOG.clear(); maxTension = 0; g.cmd("retrieve", 0, NOW); runUntil(g, 150000, [&] { return g.state != RETRIEVE; });
    printf("   broken seated switch: stop at %.1f N line tension, overwind %.1f mm\n", maxTension, P.overwind * 1000);
    CHECK(P.seated && P.dir == 0 && has("current spike") && maxTension < 400, "retrieve: seated switch broken -> stops on the current spike (150 ms)"); }

  // review safety: F refused with the carriage partly drawn, force, jog b refused 241..338, finish
  { GS g = make(); run(g, 50); g.phiKnown = true; g.phiU = 338; g.cmd("cock", 0, NOW);
    runUntil(g, 3000, [&] { float p = g.phi(); return p > 100 && p < 200; }); g.cmd("stop", 0, NOW); run(g, 300);
    printf("   stopped part-way at phi %.1f (plant %.1f)\n", g.phi(), pgPhase());
    P.seated = false; P.fired = true; P.lineOut = P.dartD = 3; run(g, 50);
    LOG.clear(); g.cmd("tow", 0, NOW); g.cmd("retrieve", 0, NOW); g.key('f', 350, NOW); run(g, 300);
    CHECK(g.state == IDLE && P.dir == 0 && has("F refused"), "carriage partly drawn: tow, retrieve and jog f refused");
    g.cmd("force", 0, NOW); g.key('f', 350, NOW); run(g, 300);
    CHECK(g.turnsF > 0.05f, "force: the next F is allowed");
    run(g, 400); g.key('f', 350, NOW); run(g, 300); CHECK(P.dir == 0, "force is used up after one F command");
    g.cmd("cock", 0, NOW); runUntil(g, 15000, [&] { return g.state != COCKING; }); P.seated = true; P.fired = false; P.lineOut = P.dartD = 0; run(g, 50); g.cmd("stop", 0, NOW);
    CHECK(g.state == READY, "cock from part-way: READY");
    LOG.clear(); g.key('b', 350, NOW); run(g, 50); CHECK(P.dir == 0 && has("jog b refused"), "jog b refused at phi 249.5 (241..338)");
    P.seated = false; run(g, 50); LOG.clear(); g.cmd("finish", 0, NOW); runUntil(g, 10000, [&] { return g.state != FIRING; });
    CHECK(g.state == IDLE && fabs(g.phi() - 333) < 1.5 && has("finished: rail down"), "finish (asked for): runs B until the rail lands, without a dart, ends IDLE"); }

  // slow cocking near the end, tighter trip and stall while cocking
  { GS g = make(); run(g, 50); g.phiKnown = true; g.phiU = 338; g.cmd("cock", 0, NOW); int dMid = -1, dEnd = -1;
    runUntil(g, 15000, [&] { float p = g.phi(); if (p > 150 && p < 200) dMid = P.duty; if (p > 240 && p < 248) dEnd = P.duty; return g.state != COCKING; });
    printf("   cocking duty %d mid-draw, %d from 230 to the stop\n", dMid, dEnd);
    CHECK(dMid == g.tu.cockDuty && dEnd == g.tu.cockSlowDuty && g.state == READY, "cocking slows from phi 230 to 249.5");
    GS h = make(); run(h, 50); h.cmd("cock", 0, NOW); run(h, 600); P.extraI = 10; run(h, 30); P.extraI = 0;
    CHECK(h.state == FAULT && strstr(h.fault, "over current"), "cocking: 12 A trip (a 13 A load trips, 18 A elsewhere)");
    GS k = make(); k.tu.cockDuty = 150; run(k, 50); k.cmd("cock", 0, NOW); run(k, 600); P.blocked = true; uint32_t tb = NOW;
    runUntil(k, 2000, [&] { return k.state == FAULT; }); P.blocked = false;
    printf("   cocking stall fault after %u ms\n", NOW - tb);
    CHECK(k.state == FAULT && strstr(k.fault, "stall") && NOW - tb < 400, "cocking: stall fault after 300 ms (600 ms elsewhere)"); }

  // power up: phi restored from flash; without it, a cocked launcher is found safely
  { GS g = make(); toReady(g); run(g, 300); float sp = SV_PHI; bool sk = SV_KNOWN, spr = SV_PROB; double pg = P.pg; bool ck = P.cocked;
    GS h = make(); P.pg = P.pin = pg; P.cocked = ck; h.restore(sp, sk, spr); run(h, 50);
    CHECK(h.state == READY && fabs(h.phi() - 249.5) < 1.5, "power up with the saved phi: READY at once");
    GS u = make(); P.pg = P.pin = pg; P.cocked = ck; run(u, 50);
    CHECK(u.state == IDLE && !u.phiKnown, "power up cocked, nothing saved: IDLE, phi unknown (the switch cannot tell)");
    u.probCocked = true; LOG.clear(); u.key('b', 350, NOW); run(u, 50); CHECK(P.dir == 0 && has("jog b refused"), "phi unknown and probably cocked: jog b refused");
    LOG.clear(); u.cmd("cock", 0, NOW); runUntil(u, 5000, [&] { return u.state != COCKING; });
    printf("   unknown-phi cock on a cocked launcher: stopped at PG %.1f, rail %.2f deg, fired %d\n", pgPhase(), thetaOf((float)pgPhase()), P.fired);
    CHECK(u.state == FAULT && strstr(u.fault, "rail lifted") && !P.fired && thetaOf((float)pgPhase()) < 6 && u.phiKnown,
          "cock with phi unknown on a cocked launcher: stops as the rail starts to lift, no shot, phi now known");
    u.cmd("clear", 0, NOW); LOG.clear(); u.cmd("finish", 0, NOW); runUntil(u, 10000, [&] { return u.state != FIRING; });
    CHECK(P.fired && fabs(u.phi() - 333) < 1.5 && pgPhase() < 340, "then finish (rail already up) fires it and stops as the rail lands, before the first tooth"); }

  // faults
  { GS g = make(); run(g, 50); g.key('b', 1500, NOW); run(g, 500); P.blocked = true; uint32_t tb = NOW;   // jog b: not cocking, 600 ms
    runUntil(g, 3000, [&] { return g.state == FAULT; }); uint32_t tf = NOW - tb;
    printf("   stall fault after %u ms\n", tf);
    CHECK(g.state == FAULT && P.dir == 0 && strstr(g.fault, "stall") && tf >= 570 && tf < 800, "stall: no AS5600 motion for 600 ms at > 30% PWM -> stop + FAULT");
    g.key('f', 350, NOW); run(g, 50); CHECK(P.dir == 0, "FAULT: jog refused until cleared");
    g.cmd("cock", 0, NOW); run(g, 50); CHECK(P.dir == 0, "FAULT: commands refused until cleared");
    P.blocked = false; g.cmd("clear", 0, NOW); CHECK(g.state == IDLE, "clear: FAULT -> IDLE"); }
  { GS g = make(); run(g, 50); g.cmd("cock", 0, NOW); run(g, 500); P.blocked = true;
    runUntil(g, 3000, [&] { return g.state == FAULT; });
    printf("   full-duty stall: %s\n", g.fault);
    CHECK(g.state == FAULT && P.dir == 0, "full-duty stall (wiper stall current = 18 A) -> FAULT (trip or stall), motor off"); }
  { GS g = make(); run(g, 50); g.key('f', 1500, NOW); run(g, 400); P.shortCircuit = true; run(g, 15); P.shortCircuit = false;
    CHECK(g.state == FAULT && P.dir == 0, "over current 18 A (2 samples) -> trip within 10 ms"); }
  { GS g = make(); P.encMissing = true; run(g, 50); CHECK(!g.encOk && has("AS5600 missing"), "AS5600 missing at boot: reported");
    g.cmd("cock", 0, NOW); run(g, 50); CHECK(g.state == FAULT && P.dir == 0, "AS5600 missing: motion command -> FAULT, motor never starts"); }
  { GS g = make(); run(g, 50); g.key('f', 1500, NOW); run(g, 400); P.encMissing = true; run(g, 30);
    CHECK(g.state == FAULT && P.dir == 0, "AS5600 lost while driving -> FAULT"); }

  // direction change delay and soft start (checked on every motor command of every test above too)
  { GS g = make(); run(g, 50); minGap = 1 << 30; g.key('f', 1500, NOW); run(g, 600); int sumDuty = P.duty;
    g.key(' ', 0, NOW); g.key('b', 1500, NOW); uint32_t t0 = NOW; runUntil(g, 1000, [&] { return P.dir < 0; });
    printf("   F -> B: B started %u ms after the stop, min gap seen %d ms\n", NOW - t0, minGap);
    CHECK(NOW - t0 >= 200 && minGap >= 200, "never reverses without 200 ms stopped");
    (void)sumDuty;
    GS h = make(); run(h, 50); h.key('f', 1500, NOW); run(h, 100); int d100 = P.duty; run(h, 250); int d350 = P.duty;
    printf("   soft start: duty %d at 100 ms, %d at 350 ms (target %d)\n", d100, d350, h.jogSpd);
    CHECK(d100 <= 255 * 100 / 300 + 5 && d350 == h.jogSpd, "soft start: PWM ramps 0 -> full over 300 ms"); }

  // deadman
  { GS g = make(); P.seated = false; P.fired = true; P.lineOut = P.dartD = 3; run(g, 50); for (int i = 0; i < 10; i++) { g.key('f', 350, NOW); run(g, 150); }
    CHECK(P.dir > 0, "WiFi hold: repeats keep the jog running"); run(g, 360); CHECK(P.dir == 0, "WiFi hold: released -> stops within the 350 ms deadman");
    g.key('b', 1500, NOW); run(g, 1400); CHECK(P.dir < 0, "serial key runs 1.5 s"); run(g, 150); CHECK(P.dir == 0, "serial key stops after 1.5 s"); }

  // full cycle helper: fire -> anchored -> tow -> released -> cock -> retrieve -> READY
  { GS g = make(); calibrate(g); toReady(g); LOG.clear();
    g.cmd("cycle", 0, NOW); CHECK(g.cyc == C_FIRE && g.state == FIRING, "cycle: starts by firing");
    runUntil(g, 10000, [&] { return g.cyc == C_WAIT_ANCHOR; }); run(g, 2500);
    CHECK(g.cyc == C_WAIT_ANCHOR && P.dir == 0, "cycle: waits for the anchored flag with the motor off");
    g.cmd("tow", 0, NOW); CHECK(g.state == PAYOUT, "cycle: manual commands refused while it runs");
    P.anchoredDart = true; g.cmd("anchored", 0, NOW);
    runUntil(g, 120000, [&] { return g.cyc == C_WAIT_RELEASE; });
    CHECK(g.cyc == C_WAIT_RELEASE && g.holding && P.tension > 15 && P.tension < 25, "cycle: tows to the limit and holds");
    run(g, 1000); P.anchoredDart = false; g.cmd("released", 0, NOW);
    runUntil(g, 200000, [&] { return g.cyc == C_OFF; }); dump();
    CHECK(g.state == READY && seatedSw() && P.cocked && has("cycle complete"), "cycle: cocks, retrieves the dart, ends READY");
    CHECK(phiErr(g) < 1.5, "cycle: phi tracking still within 1.5 deg of the plant");
    g.cmd("cycle", 0, NOW); run(g, 1000); g.cmd("stop", 0, NOW); run(g, 10); CHECK(g.cyc == C_OFF && P.dir == 0, "stop aborts a running cycle");
    char b[800]; g.status(b, sizeof b, NOW); printf("   status: %s\n", b);
    CHECK(std::string(b).find("\"state\":") != std::string::npos && std::string(b).find("\"tension_N\":") != std::string::npos, "status JSON"); }

  // ---------- review fixes (3 Nov 2026) ----------
  // jog b from part-way (phi known): even a long key press stops at 241, never lifts the rail
  { GS g = make(); run(g, 50); g.phiKnown = true; g.phiU = 338; g.cmd("cock", 0, NOW);
    runUntil(g, 3000, [&] { float p = g.phi(); return p > 195 && p < 205; }); g.cmd("stop", 0, NOW); run(g, 300); LOG.clear();
    g.key('b', 5000, NOW); run(g, 5200);
    printf("   jog b 5 s from phi 200: stopped at PG %.1f, rail %.2f deg, fired %d\n", pgPhase(), thetaOf((float)pgPhase()), P.fired);
    CHECK(!P.fired && P.dir == 0 && thetaOf((float)pgPhase()) < 4.5 && has("jog b stopped"), "review fix: a long jog b stops at phi 241 (checked every tick, not only at the key press)"); }
  // jog b held for 30 s at rest with phi unknown: stops when the sear closes behind the keel, never fires
  { GS g = make(); run(g, 50); LOG.clear();
    for (int i = 0; i < 200; i++) { g.key('b', 350, NOW); run(g, 150); } run(g, 500);
    printf("   jog b held 30 s, phi unknown: PG %.1f, cocked %d, fired %d, state %s\n", pgPhase(), P.cocked, P.fired, stateName(g.state));
    CHECK(!P.fired && P.cocked && P.dir == 0 && has("hook up behind the keel") && phiErr(g) < 1.5, "review fix: held jog b with phi unknown stops cocked (re-zeroed at 241.2), no shot"); }
  // stale saved phi (power cut mid-cock): jog b is stopped by SW_LOW opening, no shot
  { GS h = make(); P.pg = P.pin = 249.5; P.cocked = true; h.restore(200, true, false); run(h, 50); LOG.clear();
    for (int i = 0; i < 70; i++) { h.key('b', 350, NOW); run(h, 150); }
    printf("   stale phi 200 (really cocked): jog b stopped at PG %.1f, rail %.2f deg, fired %d\n", pgPhase(), thetaOf((float)pgPhase()), P.fired);
    CHECK(!P.fired && h.state == FAULT && has("rail lifted during jog b") && thetaOf((float)pgPhase()) < 6, "review fix: stale phi, jog b: stops as the rail starts to lift, no shot"); }
  // power cut mid-fire (rail up), flash says 249.5: phi unknown at power up, every B refused
  { GS h = make(); P.pg = P.pin = 300; P.fired = true; P.seated = false; h.restore(249.5f, true, false); run(h, 50);
    CHECK(!h.phiKnown && has("power up: phi disagrees"), "review fix: saved phi disagrees with SW_LOW at power up -> phi unknown");
    LOG.clear(); h.cmd("finish", 0, NOW); h.cmd("cock", 0, NOW); h.key('b', 1500, NOW); run(h, 500);
    CHECK(P.dir == 0 && fabs(pgPhase() - 300) < 0.01 && has("finish refused") && has("rail is up") && has("jog b refused"), "review fix: rail up with phi unknown: finish, cock and jog b all refused"); }
  // cock from phi 251 with the sear switch open and SW_LOW stuck: the target would wrap a whole turn (lift + fire)
  { GS g = make(); toReady(g); P.pg += 1.5; P.pin += 1.5; g.phiU += 1.5f; P.swCockBroken = true; P.lowStuck = true; run(g, 50); LOG.clear();
    g.cmd("cock", 0, NOW); runUntil(g, 15000, [&] { return g.state != COCKING; });
    CHECK(!P.fired && P.dir == 0 && has("cock refused: phi says cocked"), "review fix: cock refused at phi 239.8..252.5 when a switch disagrees (no whole-turn target)"); }
  // fire re-checks the switches, READY can be stale
  { GS g = make(); toReady(g); P.swCockBroken = true; run(g, 50); LOG.clear(); bool ok = g.cmd("fire", 0, NOW); run(g, 500);
    CHECK(!ok && !P.fired && P.dir == 0 && g.state == IDLE, "review fix: fire refused when SW_COCKED opened after READY"); }
  // probCocked: set by the keel closing (rail down), not by the trip cord going slack after a shot
  { GS g = make(); toReady(g); bool pc = g.probCocked; g.cmd("fire", 0, NOW); runUntil(g, 10000, [&] { return g.state != FIRING; });
    CHECK(pc && g.state == PAYOUT && !g.probCocked, "review fix: probCocked after cocking, cleared (not set again) by the shot"); }
  // unknown phi, probably cocked, SW_LOW stuck closed: cock stops at the trip-cord opening, no shot
  { GS g = make(); toReady(g); double pg = P.pg; GS u = make(); P.pg = P.pin = pg; P.cocked = true; P.lowStuck = true; run(u, 50);
    u.probCocked = true; LOG.clear(); u.cmd("cock", 0, NOW); runUntil(u, 8000, [&] { return u.state != COCKING; });
    printf("   unknown phi, cocked, SW_LOW stuck: cock stopped at PG %.1f, fired %d\n", pgPhase(), P.fired);
    CHECK(!P.fired && u.state == FAULT && strstr(u.fault, "sear pushed down") && u.phiKnown, "review fix: SW_LOW stuck + probably cocked: cock stops at the sear opening, no shot"); }
  // stale phi off by 22 deg while cocking from rest: FAULT at the opening, then a re-cock finds READY
  { GS g = make(); run(g, 50); g.phiKnown = true; g.phiU = 0; LOG.clear(); g.cmd("cock", 0, NOW); runUntil(g, 15000, [&] { return g.state != COCKING; });
    CHECK(g.state == FAULT && strstr(g.fault, "wrong phi") && !g.phiKnown && !P.fired, "review fix: sear opens far from 215.2 while cocking -> stop, phi unknown");
    g.cmd("clear", 0, NOW); g.cmd("cock", 0, NOW); runUntil(g, 15000, [&] { return g.state != COCKING; });
    CHECK(g.state == READY && P.cocked && phiErr(g) < 1.5, "review fix: re-cock from the keel on the hook (phi unknown): READY"); }
  // AS5600 lost during a B jog: phi no longer trusted
  { GS g = make(); run(g, 50); g.phiKnown = true; g.phiU = 339; g.key('b', 1500, NOW); run(g, 400); P.encMissing = true; run(g, 50);
    CHECK(g.state == FAULT && !g.phiKnown, "review fix: AS5600 lost during B -> FAULT and phi unknown"); }
  // Serial: F, E and y need the same key twice within 3 s
  { GS g = make(); toReady(g); LOG.clear(); g.key('F', 0, NOW); run(g, 300); g.key('q', 0, NOW); g.key('F', 0, NOW); run(g, 300);
    CHECK(g.state == READY && !P.fired && has("press F again"), "review fix: one stray F (or F, other, F) does not fire");
    g.key('F', 0, NOW); runUntil(g, 10000, [&] { return g.state != FIRING; });
    CHECK(g.state == PAYOUT && P.fired, "review fix: F twice within 3 s fires"); }
  // a b jog turned into f with the carriage partly drawn: still refused
  { GS g = make(); run(g, 50); g.phiKnown = true; g.phiU = 338; g.cmd("cock", 0, NOW);
    runUntil(g, 3000, [&] { float p = g.phi(); return p > 100 && p < 150; }); g.cmd("stop", 0, NOW); run(g, 300);
    P.seated = false; P.fired = true; P.lineOut = P.dartD = 3; run(g, 50); LOG.clear(); float tf = g.turnsF;
    g.key('b', 350, NOW); run(g, 100); g.key('f', 350, NOW); run(g, 1000);
    CHECK(has("F refused") && g.turnsF == tf, "review fix: jog b -> f while in JOG still checks the partly drawn carriage"); }
  // AS5600 jitter while stopped does not write flash; phiU stays small (float precision)
  { GS g = make(); toReady(g); run(g, 500); int n0 = SV_N; P.jitter = true; run(g, 20000); P.jitter = false;
    CHECK(SV_N == n0 && g.phiKnown, "review fix: AS5600 jitter while stopped: no flash writes in 20 s");
    CHECK(fabsf(g.phiU) < 360, "review fix: phiU rebased to 0..360 at every stop"); }

  // geometry of the second sear review: switch phases seen by the logic
  { GS g = make(); run(g, 50); g.phiKnown = true; g.phiU = 338; float pOpen = -1, pClose = -1; uint8_t prev = sws();
    g.cmd("cock", 0, NOW); runUntil(g, 15000, [&] { uint8_t b = sws(); if ((prev & SW_COCKED) && !(b & SW_COCKED)) pOpen = (float)pgPhase(); if (!(prev & SW_COCKED) && (b & SW_COCKED)) pClose = (float)pgPhase(); prev = b; return g.state != COCKING; });
    printf("   cocking: SW_COCKED opened at PG %.1f, closed at %.1f, stopped at %.1f\n", pOpen, pClose, pgPhase());
    CHECK(fabs(pOpen - 215.2) < 1 && fabs(pClose - 241.2) < 1 && g.state == READY && fabs(pgPhase() - 249.5) < 2, "second review: SW_COCKED opens 215.2, closes 241.2, cocking stops at 249.5");
    float tOpen = -1, tShot = -1; prev = sws(); bool f0 = P.fired; g.cmd("fire", 0, NOW);
    runUntil(g, 10000, [&] { uint8_t b = sws(); if ((prev & SW_COCKED) && !(b & SW_COCKED) && tOpen < 0) tOpen = (float)pgPhase(); if (!f0 && P.fired && tShot < 0) tShot = (float)(P.pin); prev = b; return g.state != FIRING; });
    printf("   fire: SW_COCKED opened at PG %.1f (rail %.1f), shot at PG %.1f\n", tOpen, thetaOf(tOpen), fmod(tShot, 360.0));
    CHECK(fabs(tOpen - 284.8) < 1 && g.state == PAYOUT && has("sear tripped"), "round 6: trip cord opens SW_COCKED at 284.8 (rail 25.7), shot, PAYOUT at 338"); }
  // lower: rail up with phi unknown (power cut mid-fire run), cocked: it fires and ends at 338, phi known
  { GS g = make(); toReady(g); double pg = P.pg; GS h = make(); P.pg = P.pin = pg + 20.5; P.cocked = true; run(h, 50);   // PG 270, rail 21.9 deg
    CHECK(!h.phiKnown && !(sws() & SW_LOW), "lower setup: rail up, phi unknown");
    double p0 = pgPhase(); LOG.clear(); h.cmd("finish", 0, NOW); h.cmd("cock", 0, NOW); h.key('b', 1500, NOW); run(h, 300);
    CHECK(P.dir == 0 && fabs(pgPhase() - p0) < 0.01 && has("Use lower"), "lower setup: finish, cock and jog b refused, the refusals point to lower");
    h.cmd("lower", 0, NOW); int dMax = 0; runUntil(h, 15000, [&] { if (P.duty > dMax) dMax = P.duty; return h.state != LOWER; }); dump();
    printf("   lower: ended %s at phi %.1f (plant %.1f, pinion %.1f), max duty %d, fired %d\n", stateName(h.state), h.phi(), pgPhase(), fmod(P.pin, 360.0), dMax, P.fired);
    CHECK(h.state == IDLE && h.phiKnown && phiErr(h) < 1.5 && pgPhase() < 340 && (sws() & SW_LOW) && dMax == h.tu.lowerDuty && has("lowered"),
          "lower: B until SW_LOW closes, re-zero to the drop end, phi known, before the first tooth");
    CHECK(P.fired && has("sear cord pulled"), "lower from the lift side: fires if cocked (documented)");
    LOG.clear(); bool ok = h.cmd("lower", 0, NOW); CHECK(!ok && has("already down"), "lower refused with the rail down"); }
  { GS g = make(); toReady(g); double pg = P.pg; GS h = make(); P.pg = P.pin = pg + 20.5; P.cocked = false; P.fired = true; P.seated = false; run(h, 50);
    h.cmd("lower", 0, NOW); runUntil(h, 15000, [&] { return h.state != LOWER; });
    CHECK(h.state == IDLE && phiErr(h) < 1.5 && pgPhase() < 340, "lower, already shot: rail down, ends at the drop end");
    h.phiU += 270 - h.phi(); P.pg = P.pin = pg + 380.5; run(h, 50); LOG.clear(); bool ok = h.cmd("lower", 0, NOW);
    CHECK(!ok && P.dir == 0 && has("phi is known"), "lower refused when phi is known (use finish)"); }
  { GS g = make(); toReady(g); double pg = P.pg; GS h = make(); P.pg = P.pin = pg + 20.5; P.cocked = true; run(h, 50);
    h.cmd("lower", 0, NOW); run(h, 400); P.extraI = 11; run(h, 30); P.extraI = 0;
    CHECK(h.state == FAULT && strstr(h.fault, "over current") && !P.fired, "lower: 12 A cocking trip applies");
    GS k = make(); P.pg = P.pin = pg + 20.5; P.cocked = true; run(k, 50); k.cmd("lower", 0, NOW); run(k, 400); P.blocked = true; uint32_t tb = NOW;
    runUntil(k, 2000, [&] { return k.state == FAULT; }); P.blocked = false;
    CHECK(k.state == FAULT && strstr(k.fault, "stall") && NOW - tb < 400, "lower: 300 ms cocking stall limit applies"); }

  printf("   global: %d reversal-gap violations, %d soft-start violations\n", gapViolations, rampViolations);
  CHECK(gapViolations == 0 && rampViolations == 0, "all tests: no reversal under 200 ms, no PWM step over the ramp");
  printf(fails ? "\n%d FAILED\n" : "\nALL PASS\n", fails);
  return fails ? 1 : 0;
}
