// Firmware configuration. Spec numbers come from params_gen.h (generated from base/spec/params.py by
// tools/gen_params.py); everything here has a default so a missing spec value never breaks the build.
// Bench values (current scale, free current, N per amp) are runtime settings: "set" over the protocol, kept in NVS.
#pragma once
#include <stdint.h>
#include "params_gen.h"

namespace gb {

// Logical contacts. Each maps onto one of the 8 switch pins (bit 0..7 = PIN_SW_A..H) or -1 when not fitted.
enum SwId { SW_LAUNCH, SW_NEUTRAL, SW_REEL, SW_WRAP, SW_SEATED, SW_COCKED, SW_AIM_HOME, SW_COUNT };
static const char* const SW_KEY[SW_COUNT] = {"launch", "neutral", "reel", "wrap", "seated", "cocked", "aim_home"};

// drum positions along its travel (drum deg from the LAUNCH detent, rising toward REEL and the wrap zone)
enum DrumPos { DP_LAUNCH, DP_NEUTRAL, DP_REEL, DP_WRAPPED, DP_BETWEEN, DP_UNKNOWN };
static const char* const DRUM_POS_NAME[] = {"LAUNCH", "NEUTRAL", "REEL", "WRAPPED", "BETWEEN", "UNKNOWN"};

static const int AIM_TAB_MAX = 200;

struct Cfg {
  // ---- tick
  uint32_t tickMs = 5;

  // ---- switches: physical bit for each logical contact (-1 = not fitted)
  int8_t swBit[SW_COUNT] = {0, 1, 2, 3, 4, 5, 6};   // PIN_SW_A..G in that order (WIRING.md)
  uint32_t debounceMs = 15;

  // ---- wiper (BTS7960)
  float stallA =
#ifdef P_WIPER_STALL_A
      P_WIPER_STALL_A;
#else
      18.0f;
#endif
  float freeA =
#ifdef P_WIPER_FREE_A
      P_WIPER_FREE_A;
#else
      1.5f;
#endif
  float n0Rpm =
#ifdef P_WIPER_N0_RPM
      P_WIPER_N0_RPM;
#else
      45.0f;
#endif
  float tripA = 17.0f;             // over-current trip (fuse is 20 A, wiper stall about 18 A)
  float cockTripA = 15.0f;         // while cocking (spring loading): tighter
  int wiperSign = +1;              // AS5600 count direction (CALIBRATION.md step 3)

  // which wiper direction does what. LAUNCH mode: aimDir aims (one-way element 1), the other direction cocks /
  // fires / resets (sprag #1). REEL mode: reelDir winds the capstan (sprag #2); the other direction is refused.
  int aimDir = +1;
  int reelDir = +1;
  int aimDuty = 200, aimSlowDuty = 110; float aimSlowDeg = 25; float aimCoastDeg = 1.0f;
  int cockDuty = 255, cockSlowDuty = 140, fireDuty = 255, resetDuty = 200, jogDuty = 150, nudgeDuty = 90;

  // ---- aim train: pitch (deg) against aim-train wiper angle (deg) over one aim cycle
  float aimCycleDeg = 360.0f;
  float aimTolDeg = 0.75f;          // AIMED if within this
  float pitchMin = 15.0f, pitchMax = 60.0f;
  float aimHomeDeg = 0.0f;          // aim phase where SW_AIM_HOME closes
  float aimMaxDeg = 760.0f;         // longest aim run (two cycles) before STALL / BAD_STATE

  // ---- shot cycle (wiper deg in the cock direction from rest)
  float cycleDeg = 720.0f;          // one full shot cycle: rest -> cocked -> release -> reset -> rest
  float cockEndDeg = 540.0f;        // stop here: carriage on the sear (SW_COCKED closes just before)
  float cockSlowFrom = 500.0f;      // slow from here so the stop is precise
  float searWinDeg = 25.0f;         // SW_COCKED must close within this before cockEndDeg
  float releaseDeg = 570.0f;        // sear trips (SW_COCKED opens) about here while firing
  float releaseWinDeg = 25.0f;
  uint32_t cockMaxMs = 20000, fireMaxMs = 15000;

  // ---- drum (N20 worm on DRV8833)
  float drumPosDeg[3] = {0.0f, 60.0f, 120.0f};   // LAUNCH, NEUTRAL, REEL detents
  float wrapStartDeg = 150.0f;      // the wrap arm starts laying line past here
  float degPerWrap = 360.0f;        // drum travel per capstan wrap
  int wrapsTarget = 4;              // wraps laid by "wrap"
  int maxWraps = 6;
  float drumDps = 60.0f;            // drum speed estimate (deg/s); re-learned at every contact-to-contact move
  int drumSign = +1;                // DRV8833 direction that moves LAUNCH -> REEL
  float drumWinDeg = 12.0f;         // a detent contact is closed within +-this of its position
  uint32_t drumSettleMs = 60;       // keep driving this long after the contact closes (centres the detent)
  float drumTimeFactor = 2.0f; uint32_t drumTimeMarginMs = 1500;   // timeout = expected x factor + margin
  float drumEarlyFrac = 0.25f;      // arriving faster than this fraction of the expected time = suspicious contact
  int drumRetries = 1;              // dog tooth-on-tooth: back off, nudge the wiper, try again this many times
  float nudgeDeg = 20.0f;           // wiper nudge between tries (wiper in NEUTRAL turns nothing)

  // ---- reel
  float capstanDmm = 30.0f;         // effective capstan diameter (line centre)
  float capstanPerWiper = 1.0f;     // capstan turns per wiper output turn
  int reelDuty = 220; int reelMinDuty = 70; float taperFrac = 0.30f;
  float kNperA = 8.0f;              // line N per amp above free running. Bench: CALIBRATION.md
  float limitN = 20.0f, limitMaxN = 150.0f;
  float resumeFrac = 0.6f;          // holding winds again below this fraction of the limit (load cell only)
  uint32_t tensionIgnoreMs = 350;   // ramp inrush is not tension
  float seatSlowM = 1.0f; int seatSlowDuty = 110;
  float lineTotalM = 20.0f;
  uint32_t reelMaxMs = 240000;
  float hxNPerCount = 1.0f / 21000.0f;

  // ---- power
  float vbatLow = 9.9f;             // 3.3 V per cell under load: BROWNOUT if below for vbatLowMs while a motor runs
  uint32_t vbatLowMs = 300;
  float vbatPresent = 6.0f;         // below this the battery is unplugged (USB only): motors refused (NO_POWER)
  float vbatScale = (100.0f + 22.0f) / 22.0f;   // divider 100 k / 22 k

  // ---- protocol
  uint32_t armMs = 3000;            // arm token lifetime
  uint32_t jogMaxMs = 1500;
  uint32_t telSlowMs = 100, telFastMs = 50;     // 10 Hz idle, 20 Hz while anything moves
  bool requireSeated = true;        // fire needs SW_SEATED closed
};

// placeholder aim table until params.py has AIM: a crank-rocker like sine between pitchMin and pitchMax
#ifdef P_AIM_TABLE_N
static const int AIM_DEF_N = P_AIM_TABLE_N;
#endif

}  // namespace gb
