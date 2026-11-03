// Firmware configuration. Spec numbers come from params_gen.h (generated from base/spec/params.py by
// tools/gen_params.py); everything here has a default so a missing spec value never breaks the build.
// Bench values (current scale, free current, N per amp) are runtime settings: "set" over the protocol, kept in NVS.
#pragma once
#include <stdint.h>
#include "params_gen.h"

namespace gb {

// Contacts (names from BASE_SPEC.md section 4). Each maps onto one of the 8 switch pins (bit 0..7 = PIN_SW_A..H) or
// -1 when not fitted. SW_DRUM_A/B/C are code bumps on the selector drum: the code they make says the drum position.
enum SwId { SW_DRUM_A, SW_DRUM_B, SW_DRUM_C, SW_SEATED, SW_HOME, SW_RAIL_LOW, SW_WRAP_HOME, SW_COUNT };
static const char* const SW_NAME[SW_COUNT] = {"SW_DRUM_A", "SW_DRUM_B", "SW_DRUM_C", "SW_SEATED", "SW_HOME", "SW_RAIL_LOW", "SW_WRAP_HOME"};
static const char* const SW_TEL[SW_COUNT] = {"a", "b", "c", "S", "H", "L", "W"};   // telemetry keys (PROTOCOL.md)

// drum positions along its travel (drum deg from the LAUNCH detent, rising toward REEL and the wrap zone)
enum DrumPos { DP_LAUNCH, DP_NEUTRAL, DP_REEL, DP_WRAPPED, DP_BETWEEN, DP_UNKNOWN };
static const char* const DRUM_POS_NAME[] = {"LAUNCH", "NEUTRAL", "REEL", "WRAPPED", "BETWEEN", "UNKNOWN"};

static const int AIM_TAB_MAX = 200;

struct Cfg {
  // ---- tick
  uint32_t tickMs = 5;

  // ---- switches: physical bit for each logical contact (-1 = not fitted)
  int8_t swBit[SW_COUNT] = {0, 1, -1, 3, 4, 5, 6};  // PIN_SW_A..G in that order (WIRING.md); SW_DRUM_C not fitted
  // drum code: bit 0 = SW_DRUM_A, bit 1 = SW_DRUM_B, bit 2 = SW_DRUM_C (placeholder Gray code until BASE_SPEC v1.0)
  uint8_t posCode[3] = {0x1, 0x3, 0x2};             // LAUNCH, NEUTRAL, REEL
  uint8_t codeValid = 0x0F;                         // bit n set = code n can occur (0 = between bumps); others = SWITCH_DISAGREE
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

  // which wiper direction does what (BASE_SPEC section 1; wiper +1 = FORWARD = RPWM). LAUNCH: FORWARD cocks, fires and
  // resets through sprag #1; BACKWARD aims through the swing-arm splitter and the aim worm. REEL: FORWARD winds the
  // capstan; BACKWARD is refused (sprag #2 backstops the capstan: the wiper would stall against it).
  int aimDir = -1;
  int reelDir = +1;
  float aimLashDeg = 0.0f;          // wiper travel after reversing before the swing-arm splitter drives the aim (spec TBD)
  int aimDuty = 200, aimSlowDuty = 110; float aimSlowDeg = 25; float aimCoastDeg = 1.0f;
  int cockDuty = 255, cockSlowDuty = 140, fireDuty = 255, resetDuty = 200, jogDuty = 150, nudgeDuty = 90;

  // ---- aim train: pitch (deg) against aim-train wiper angle (deg) over one aim cycle
  float aimCycleDeg = 360.0f;
  float aimTolDeg = 0.75f;          // AIMED if within this
  float pitchMin = 15.0f, pitchMax = 60.0f;
  float aimHomeDeg = 0.0f;          // aim phase where SW_RAIL_LOW closes (moving in aimDir)
  float aimMaxDeg = 760.0f;         // longest aim run (two cycles) before STALL / BAD_STATE

  // ---- shot cycle: wiper deg FORWARD from rest, one big gear turn per shot (BASE_SPEC section 1). No sear: the
  // drawn carriage is held by sprag #1 and the wiper worm, released by the relieved last tooth.
  float cycleDeg = 720.0f;          // one big gear turn: rest -> draw -> release -> reset -> rest
  float homeOpenDeg = 40.0f;        // SW_HOME opens here (first tooth starts drawing the carriage back)
  float homeWinDeg = 25.0f;         // ... within this
  float cockEndDeg = 560.0f;        // cocked: stop here, just before the relieved tooth
  float cockSlowFrom = 520.0f;      // slow from here so the stop is precise
  float releaseDeg = 580.0f;        // the relieved tooth lets the rack go: carriage flies forward, SW_HOME closes
  float releaseWinDeg = 20.0f;
  float decockExtraDeg = 15.0f;     // decock: run on this far after SW_HOME closes
  uint32_t cockMaxMs = 20000, fireMaxMs = 15000;
  int decockDuty = 120;

  // ---- drum (N20 worm on DRV8833)
  float drumPosDeg[3] = {0.0f, 60.0f, 120.0f};   // LAUNCH, NEUTRAL, REEL detents
  float wrapStartDeg = 150.0f;      // the wrap arm starts laying line past here
  float degPerWrap = 360.0f;        // drum travel per capstan wrap
  int wrapsTarget = 4;              // wraps laid by "wrap"
  int maxWraps = 6;
  float drumDps = 60.0f;            // drum speed estimate (deg/s); re-learned at every contact-to-contact move
  int drumSign = +1;                // DRV8833 direction that moves LAUNCH -> REEL
  float drumWinDeg = 10.0f;         // a detent contact is closed within +-this of its centre; after it closes the drum
                                    // drives on by this much (dead reckoning) to sit centred
  float wrapWinDeg = 6.0f;          // the wrap index contact is closed within +-this of each whole wrap
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
  bool aimNeedsRest = true;         // aim only with the carriage home: backward drive would let the drawn spring unwind
};

// placeholder aim table until params.py has AIM: a crank-rocker like sine between pitchMin and pitchMax
#ifdef P_AIM_TABLE_N
static const int AIM_DEF_N = P_AIM_TABLE_N;
#endif

}  // namespace gb
