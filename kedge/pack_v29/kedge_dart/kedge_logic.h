// KEDGE dart control logic: plain C++, no Arduino calls, so it can be unit-tested on a PC (see test/test_logic.cpp).
// The sketch supplies the hardware through the Io struct.
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>

namespace kedge {

enum { DEPLOY = 0, KICK = 1, DRILLS = 2, FOOT = 3, NMOT = 4 };

struct Io {
  void (*setMotor)(int motor, int dir, int duty);   // dir -1/0/+1, duty 0..255
  float (*current_mA)();                            // total motor-rail current (INA219)
  float (*rail_V)();
  void (*log)(const char* line);
};

struct Tuning {
  uint32_t serialRunMs = 1500, wifiDeadmanMs = 350;
  float tripMa = 2500;
  float kickContactMa = 60;      // foot auger tip meets the ground (free ~25 mA)
  float kickStallMa = 150;       // foot housing on the ground (stall ~180 mA)
  float deployStallMa = 150;     // pods stop in the soil / at the end
  uint32_t kickMaxMs = 15000, footAfterMs = 1500, deployMaxMs = 6000, drillAfterMs = 3000;
  uint32_t surgeIgnoreMs = 250, stallHoldMs = 200, contactHoldMs = 150;
};

enum Seq { IDLE, K_SWING, K_PLANT, F_AFTER, D_DEPLOY, D_AFTER, DONE, ABORTED };
static const char* seqName(int s) {
  static const char* n[] = {"idle", "kick swing", "kick + foot planting", "foot finishing", "deploy + drills", "drills finishing", "anchored", "aborted"};
  return n[s];
}

struct Kedge {
  Io io; Tuning tu;
  int dir[NMOT] = {0, 0, 0, 0};
  uint32_t until[NMOT] = {0, 0, 0, 0};
  int spd = 255;
  int seq = IDLE; uint32_t seqT0 = 0, condT = 0, motT0[NMOT] = {0, 0, 0, 0};
  float lastMa = 0, peakMa = 0; char lastEvent[64] = "boot";
  bool armedOnLanding = false;
  uint32_t pulseT = 0; int stallHits = 0; bool paused = false; float lastSolo = -1;
  // run 400 ms with the helper on, then pause the helper 150 ms and read the main motor alone at 110 ms; true after 2 stall reads
  bool pulse(int helper, float stallMa, uint32_t now) {
    uint32_t ph = now - pulseT;
    if (!paused && ph >= 400) { paused = true; pulseT = now; io.setMotor(helper, 0, 0); return false; }
    if (paused && ph >= 110 && ph < 150 && lastSolo < 0) { lastSolo = lastMa; stallHits = lastSolo > stallMa ? stallHits + 1 : 0; return stallHits >= 2; }
    if (paused && ph >= 150) { paused = false; pulseT = now; lastSolo = -1; if (dir[helper]) io.setMotor(helper, dir[helper], spd); }
    if (!paused) lastSolo = -1;
    return false;
  }

  void ev(const char* s) { strncpy(lastEvent, s, sizeof(lastEvent) - 1); if (io.log) io.log(s); }
  void drive(int m, int d, uint32_t ms, uint32_t now) {
    dir[m] = d; until[m] = d ? now + ms : 0; if (d) motT0[m] = now;
    io.setMotor(m, d, d ? spd : 0);
  }
  void stopAll(uint32_t now) { for (int m = 0; m < NMOT; m++) drive(m, 0, 0, now); }
  bool running() const { for (int m = 0; m < NMOT; m++) if (dir[m]) return true; return false; }

  // manual key: 1/! deploy, 2/@ kick, 3/# drills, 4/$ foot, space/s stop, +/- speed, a auto, A arm auto-on-landing
  void key(char c, uint32_t runMs, uint32_t now) {
    if (seq != IDLE && seq != DONE && seq != ABORTED && c != ' ' && c != 's' && c != 'S') return;   // auto owns the motors
    switch (c) {
      case '1': drive(DEPLOY, +1, runMs, now); break; case '!': drive(DEPLOY, -1, runMs, now); break;
      case '2': drive(KICK, +1, runMs, now); break;   case '@': drive(KICK, -1, runMs, now); break;
      case '3': drive(DRILLS, +1, runMs, now); break; case '#': drive(DRILLS, -1, runMs, now); break;
      case '4': drive(FOOT, +1, runMs, now); break;   case '$': drive(FOOT, -1, runMs, now); break;
      case ' ': case 's': case 'S': stopAll(now); if (seq != IDLE && seq != DONE) { seq = ABORTED; ev("auto aborted"); } else ev("stop"); break;
      case '+': spd = spd + 25 > 255 ? 255 : spd + 25; break;
      case '-': spd = spd - 25 < 50 ? 50 : spd - 25; break;
      case 'a': startAuto(now); break;
      case 'A': armedOnLanding = !armedOnLanding; ev(armedOnLanding ? "auto armed for landing" : "auto disarmed"); break;
      default: break;
    }
  }
  void startAuto(uint32_t now) { stopAll(now); paused = false; lastSolo = -1; stallHits = 0; seq = K_SWING; seqT0 = now; condT = 0; peakMa = 0; drive(KICK, +1, tu.kickMaxMs + 500, now); ev("auto: kick swing"); }
  // returns true once cond has held continuously for hold ms
  bool held(bool cond, uint32_t hold, uint32_t now) { if (!cond) { condT = 0; return false; } if (!condT) condT = now; return now - condT >= hold; }

  void tick(uint32_t now) {
    float ma = io.current_mA ? io.current_mA() : 0; lastMa = ma; if (ma > peakMa) peakMa = ma;
    if (running() && ma > tu.tripMa) { stopAll(now); if (seq != IDLE && seq != DONE) seq = ABORTED; ev("TRIP: over current, all stopped"); return; }
    for (int m = 0; m < NMOT; m++) if (dir[m] && (int32_t)(now - until[m]) >= 0) {
      if (seq == IDLE || seq == DONE || seq == ABORTED) drive(m, 0, 0, now);
      else if (m == KICK && (seq == K_SWING || seq == K_PLANT)) { stopAll(now); seq = ABORTED; ev("auto: kick timed out"); return; }
      else drive(m, 0, 0, now);
    }
    uint32_t t = now - seqT0;
    switch (seq) {
      case K_SWING:
        if (t > tu.surgeIgnoreMs && held(ma > tu.kickContactMa, tu.contactHoldMs, now)) {
          drive(FOOT, +1, tu.kickMaxMs, now); seq = K_PLANT; seqT0 = now; condT = 0; pulseT = now; stallHits = 0; paused = false; lastSolo = -1; ev("auto: foot touches, foot auger on");
        }
        break;
      case K_PLANT:   // kick keeps pushing while the foot auger screws in. The INA219 sees the sum, so every 400 ms the foot
                      // pauses for 150 ms and the kick-only current is read; two stall readings in a row = housing on the ground.
        if (pulse(FOOT, tu.kickStallMa, now)) {
          drive(KICK, 0, 0, now); drive(FOOT, +1, tu.footAfterMs, now); seq = F_AFTER; seqT0 = now; ev("auto: kick down, foot finishing");
        }
        break;
      case F_AFTER:
        if (!dir[FOOT]) { drive(DRILLS, +1, tu.deployMaxMs + tu.drillAfterMs, now); drive(DEPLOY, +1, tu.deployMaxMs, now); seq = D_DEPLOY; seqT0 = now; pulseT = now; stallHits = 0; paused = false; lastSolo = -1; ev("auto: deploy with drills spinning"); }
        break;
      case D_DEPLOY:  // pods swing down while the augers cut; same trick: drills pause briefly to read the deploy worm alone
        if (bool st = pulse(DRILLS, tu.deployStallMa, now); st || !dir[DEPLOY]) {
          drive(DEPLOY, 0, 0, now); drive(DRILLS, +1, tu.drillAfterMs, now); seq = D_AFTER; seqT0 = now; ev(st ? "auto: pods down, drills finishing" : "auto: deploy time limit, drills finishing");
        }
        break;
      case D_AFTER:
        if (!dir[DRILLS]) { seq = DONE; ev("auto: anchored, holding (worms self-lock)"); }
        break;
      default: break;
    }
  }
  int status(char* buf, int n, float roll, float pitch, bool landed, uint32_t now, float railV = -1) const {
    return snprintf(buf, n, "{\"t_ms\":%lu,\"rail_V\":%.2f,\"current_mA\":%.0f,\"peak_mA\":%.0f,\"speed\":%d,\"motors\":{\"deploy\":%d,\"kick\":%d,\"drills\":%d,\"foot\":%d},"
                    "\"seq\":\"%s\",\"armed\":%s,\"landed\":%s,\"roll\":%.1f,\"pitch\":%.1f,\"event\":\"%s\"}",
                    (unsigned long)now, railV >= 0 ? railV : (io.rail_V ? io.rail_V() : 0.f), lastMa, peakMa, spd, dir[0], dir[1], dir[2], dir[3], seqName(seq),
                    armedOnLanding ? "true" : "false", landed ? "true" : "false", roll, pitch, lastEvent);
  }
};

// launch / landing detector on |a| in g: launch spike, flight, impact spike, then still for stillMs
struct Landing {
  int state = 0; uint32_t t = 0, tq = 0;   // 0 wait launch, 1 flying, 2 impacted, 3 landed
  void reset() { state = 0; t = tq = 0; }
  float launchG = 6, impactG = 6, stillLo = 0.85, stillHi = 1.15; uint32_t minFlightMs = 300, stillMs = 1500;
  bool update(float a, uint32_t now) {
    switch (state) {
      case 0: if (a > launchG) { state = 1; t = now; } break;
      case 1: if (now - t > minFlightMs && a > impactG) { state = 2; t = now; }
              else if (now - t > minFlightMs && a > stillLo && a < stillHi) { if (!tq) tq = now; if (now - tq > stillMs) { state = 3; return true; } }   // impact spike missed: still for stillMs after flight
              else tq = 0;
              break;
      case 2: if (a < stillLo || a > stillHi) t = now; else if (now - t > stillMs) { state = 3; return true; } break;
    }
    return false;
  }
};

}  // namespace kedge
