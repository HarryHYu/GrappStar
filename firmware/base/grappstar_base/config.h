// Firmware configuration. Spec numbers come from params_gen.h (generated from base/spec/params.py by
// tools/gen_params.py); everything here has a default so a missing spec value never breaks the build.
// Bench values (current scale, free current, N per amp) are runtime settings: "set" over the protocol, kept in NVS.
#pragma once
#include <stdint.h>
#include "params_gen.h"

namespace gb {

// Contacts (names from BASE_SPEC.md section 4 and base/cad/parts/drum_NOTES.md section 8). Each maps onto one of the
// 8 switch pins (bit 0..7 = PIN_SW_A..H) or -1 when not fitted. SW_DRUM_A/B/C read a 3-bit Gray code off the drum
// (A = band b0, B = b1, C = b2). SW_DOG closes when the dog collar landed tip to tip (fork spring deflected).
enum SwId { SW_DRUM_A, SW_DRUM_B, SW_DRUM_C, SW_DOG, SW_SEATED, SW_HOME, SW_RAIL_LOW, SW_WRAP_HOME, SW_COUNT };
static const char* const SW_NAME[SW_COUNT] = {"SW_DRUM_A", "SW_DRUM_B", "SW_DRUM_C", "SW_DOG", "SW_SEATED", "SW_HOME", "SW_RAIL_LOW", "SW_WRAP_HOME"};
static const char* const SW_TEL[SW_COUNT] = {"a", "b", "c", "D", "S", "H", "L", "W"};   // telemetry keys (PROTOCOL.md)

// Drum sectors in travel order (drum angle rising). The code bumps give the sector absolutely (Gray code: adjacent
// sectors differ by one bit). Code 000 never occurs in travel: a wire off or a bump missing.
enum Sector { SEC_LAUNCH, SEC_LN, SEC_NEUTRAL, SEC_NR, SEC_REEL, SEC_WRAPPING, SEC_WRAPPED, SEC_COUNT, SEC_INVALID = 9 };
static const char* const SEC_NAME[SEC_COUNT] = {"LAUNCH", "LN", "NEUTRAL", "NR", "REEL", "WRAPPING", "WRAPPED"};

// drum positions reported in telemetry
enum DrumPos { DP_LAUNCH, DP_NEUTRAL, DP_REEL, DP_WRAPPED, DP_BETWEEN, DP_UNKNOWN };
static const char* const DRUM_POS_NAME[] = {"LAUNCH", "NEUTRAL", "REEL", "WRAPPED", "BETWEEN", "UNKNOWN"};

static const int AIM_TAB_MAX = 200;

struct Cfg {
  // ---- tick
  uint32_t tickMs = 5;

  // ---- switches: physical bit for each logical contact (-1 = not fitted): PIN_SW_A..H in SwId order (WIRING.md)
  int8_t swBit[SW_COUNT] = {0, 1, 2, 3, 4, 5, 6, 7};
  uint32_t debounceMs = 20;

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
  int wiperSign = +1;              // AS5600 count direction (CALIBRATION.md step a)

  // which wiper direction does what (BASE_SPEC section 1; wiper +1 = FORWARD = RPWM). LAUNCH: FORWARD cocks, fires and
  // resets through sprag #1; BACKWARD aims through the swing-arm splitter and the aim worm. REEL: FORWARD winds the
  // capstan; BACKWARD is refused (sprag #2 backstops the capstan: the wiper would stall against it).
  int aimDir = -1;
  int reelDir = +1;
  float aimLashDeg = 0.0f;          // wiper travel after reversing before the swing-arm splitter drives the aim (spec TBD)
  int aimDuty = 200, aimSlowDuty = 110; float aimSlowDeg = 60; float aimCoastDeg = 1.0f;
  int cockDuty = 255, cockSlowDuty = 140, fireDuty = 255, resetDuty = 200, jogDuty = 150, nudgeDuty = 90;

  // ---- aim train: pitch (deg) against aim-train wiper angle (deg) over one aim cycle
  float aimCycleDeg = 3600.0f;      // launch_NOTES.md section 7: 10 wiper turns per aim cycle
  float aimTolDeg = 0.75f;          // AIMED if within this
  float pitchMin = 6.0f, pitchMax = 47.1f;
  float aimHomeDeg = 3365.0f;       // aim phase where SW_RAIL_LOW closes on the backward run (pitch falls through 7 deg)
  float aimMaxDeg = 7600.0f;        // longest aim run (two cycles) before BAD_STATE

  // ---- shot cycle: wiper deg FORWARD from rest, one big gear turn per shot (BASE_SPEC section 1). No sear: the
  // drawn carriage is held by sprag #1 and the wiper worm, released by the relieved last tooth.
  // launch_NOTES.md section 3: big gear 40 T, drive gear 26 T, 33 teeth present: 553.8 wiper deg per shot,
  // carriage released after the last tooth (33/40 of a turn = 457 wiper deg), reset over the 63 deg gap
  float cycleDeg = 553.8f;          // one big gear turn: rest -> draw -> release -> reset -> rest
  float homeOpenDeg = 3.0f;         // SW_HOME opens here (tooth 1 is engaged at rest: the carriage leaves its stop at once)
  float homeWinDeg = 12.0f;         // ... within this
  float cockEndDeg = 440.0f;        // cocked: stop here, short of the release tooth
  float cockSlowFrom = 400.0f;      // slow from here so the stop is precise
  float releaseDeg = 457.0f;        // the relieved tooth lets the rack go: carriage flies forward, SW_HOME closes
  float releaseWinDeg = 15.0f;
  float decockExtraDeg = 15.0f;     // decock: run on this far after SW_HOME closes
  uint32_t cockMaxMs = 20000, fireMaxMs = 20000;
  int decockDuty = 120;

  // ---- drum (N20 worm on DRV8833 #3), base/cad/parts/drum_NOTES.md sections 6 and 7
  uint8_t secCode[SEC_COUNT] = {0x1, 0x3, 0x2, 0x6, 0x7, 0x5, 0x4};   // code b2 b1 b0 of each sector
  float edgeDeg[SEC_COUNT - 1] = {4, 36, 44, 76, 98, 327};           // drum deg of the edge between sector i and i+1
  float secLoDeg = -10, secHiDeg = 346;                               // hard stops (groove ends): never driven into
  float targetDeg[4] = {0, 40, 80, 338};                              // stop points: LAUNCH, NEUTRAL, REEL, WRAPPED
  float wrapFromDeg = 100, wrapToDeg = 325;                           // the segment gear lays the wraps linearly here
  int wrapsFull = 4;                                                  // wraps at WRAPPED (fixed by the gears)
  float drumDps = 10.8f;            // drum speed estimate (deg/s); re-learned between every two edges in one run
  int drumSign = +1;                // DRV8833 direction that turns the drum toward REEL (CALIBRATION.md)
  float drumTimeFactor = 2.0f; uint32_t drumTimeMarginMs = 2000;   // timeout = expected x factor + margin
  float drumEarlyFrac = 0.25f;      // reaching the target edge faster than this fraction of expected = bad contact
  uint32_t invalidMs = 150;         // code 000 (or a jump over a sector) for this long = DRUM_SENSOR
  // dogs tip to tip after a shift into LAUNCH or REEL (SW_DOG closed): wiggle the wiper until the dogs drop in
  float dogStepDeg = 15, dogMaxDeg = 60; int dogDuty = 77;   // 30 percent duty
  float unloadDeg = 3;              // before shifting out of LAUNCH: wiper this far BACKWARD to take the load off the dogs

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
  bool vbatWired = true;            // the divider is fitted (WIRING.md). Off: no battery checks (set vbat_wired false)

  // ---- protocol
  uint32_t armMs = 3000;            // arm token lifetime
  uint32_t jogMaxMs = 1500;
  float hostTimeoutS = 0;           // 0 = off. Else: no line from the Jetson for this long while a motor runs = stop
  uint32_t telSlowMs = 100, telFastMs = 50;     // 10 Hz idle, 20 Hz while anything moves
  bool requireSeated = true;        // fire needs SW_SEATED closed
  bool aimNeedsRest = true;         // aim only with the carriage home: backward drive would let the drawn spring unwind
};

// placeholder aim table until params.py has AIM: a crank-rocker like sine between pitchMin and pitchMax
#ifdef P_AIM_TABLE_N
static const int AIM_DEF_N = P_AIM_TABLE_N;
#endif

}  // namespace gb
