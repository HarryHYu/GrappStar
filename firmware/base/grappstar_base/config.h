// Firmware configuration. Spec numbers come from params_gen.h (generated from base/spec/params.py by
// tools/gen_params.py); everything here has a default so a missing spec value never breaks the build.
// Bench values (current scale, free current, N per amp) are runtime settings: "set" over the protocol, kept in NVS.
#pragma once
#include <stdint.h>
#include "params_gen.h"

namespace gb {

// Contacts (BASE_SPEC v1.0 section 4, params.SWITCHES). Each maps onto one switch pin (bit 0..11 = PIN_SW_A..L) or -1
// when not fitted. SW_DRUM_A/B/C: 3-bit Gray code of the drum zone. SW_DOG_L / SW_DOG_R: collar really engaged in LAUNCH
// / REEL. SW_N20: N20 in mesh (open = hand lever latched out: the firmware never drives the drum). SW_HOME: carriage on
// its front stop. SW_GAP: big gear in its toothless gap (rest). SW_RAIL_LOW: crank low dead point. SW_SEATED: dart tail
// on the pusher. SW_ARM: wrap cup ring at home (one pulse per cup turn). SW_INDEX: 6-bump cam (no-magnet fallback, D12).
enum SwId { SW_DRUM_A, SW_DRUM_B, SW_DRUM_C, SW_DOG_L, SW_DOG_R, SW_N20, SW_HOME, SW_GAP, SW_RAIL_LOW, SW_SEATED, SW_ARM, SW_INDEX, SW_COUNT };
static const char* const SW_NAME[SW_COUNT] = {"SW_DRUM_A", "SW_DRUM_B", "SW_DRUM_C", "SW_DOG_L", "SW_DOG_R", "SW_N20", "SW_HOME", "SW_GAP",
                                              "SW_RAIL_LOW", "SW_SEATED", "SW_ARM", "SW_INDEX"};
static const char* const SW_TEL[SW_COUNT] = {"a", "b", "c", "l", "r", "n", "H", "G", "L", "S", "W", "I"};   // telemetry keys (PROTOCOL.md)

// Drum sectors in travel order (drum angle rising), decision D16 / lens B table 4.1: LAUNCH, ramp L to N, NEUTRAL,
// wrap prep (cup lock releases), WRAP (cup lays the wraps, collar in NEUTRAL), LOCK (cup locked, collar ramps N to R),
// REEL (wrapped and engaged). The code bumps give the sector absolutely (Gray: adjacent sectors differ by one bit).
// Code 000 never occurs in travel: a broken common wire or a missing bump.
enum Sector { SEC_LAUNCH, SEC_LN, SEC_NEUTRAL, SEC_PREP, SEC_WRAP, SEC_LOCK, SEC_REEL, SEC_COUNT, SEC_INVALID = 9 };
static const char* const SEC_NAME[SEC_COUNT] = {"LAUNCH", "L_TO_N", "NEUTRAL", "WRAP_PREP", "WRAP", "N_TO_R", "REEL"};

// drum positions reported in telemetry
enum DrumPos { DP_LAUNCH, DP_NEUTRAL, DP_REEL, DP_WRAPPED, DP_BETWEEN, DP_UNKNOWN };
static const char* const DRUM_POS_NAME[] = {"LAUNCH", "NEUTRAL", "REEL", "WRAPPED", "BETWEEN", "UNKNOWN"};

static const int AIM_TAB_MAX = 200;

struct Cfg {
  // ---- tick
  uint32_t tickMs = 5;

  // ---- switches: physical bit for each logical contact (-1 = not fitted): PIN_SW_A..L in SwId order (WIRING.md)
  int8_t swBit[SW_COUNT] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, -1};   // SW_INDEX only without the AS5600 magnet
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
  // current trips per state (decision D8, lens B I5); a printed torque limiter in the coupler slips at about 5 N m
#ifndef P_REEL_CURRENT_TRIPS_A_AIM
#define P_REEL_CURRENT_TRIPS_A_AIM 5.0f
#endif
#ifndef P_REEL_CURRENT_TRIPS_A_COCK
#define P_REEL_CURRENT_TRIPS_A_COCK 6.5f
#endif
#ifndef P_REEL_CURRENT_TRIPS_A_TOW
#define P_REEL_CURRENT_TRIPS_A_TOW 6.5f
#endif
#ifndef P_REEL_TOW_TENSION_MODEL_N_PER_A
#define P_REEL_TOW_TENSION_MODEL_N_PER_A 17.0f
#endif
#ifndef P_REEL_TOW_TENSION_MODEL_OFFSET_N
#define P_REEL_TOW_TENSION_MODEL_OFFSET_N -10.4f
#endif
  float tripA = P_REEL_CURRENT_TRIPS_A_AIM;     // aim, reel-in (params.REEL.current_trips_a, D8 / D33)
  float cockTripA = P_REEL_CURRENT_TRIPS_A_COCK;          // cock and fire (peak about 5.1 A)
  float towTripA = P_REEL_CURRENT_TRIPS_A_TOW;           // tow (reel until tension): a 50 N tow draws about 5.1 A; the tension estimate is the limit
  float takeupTripA = 2.5f;        // take-up only (NEUTRAL / WRAP)
  float jogTripA = 3.0f;           // jogs, centering jog, fly-in
  float hardTripA = 17.0f;         // never exceeded in any state (fuse 20 A)
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
  float pitchMin =
#ifdef P_AIM_PITCH_MIN
      P_AIM_PITCH_MIN;
#else
      6.0f;
#endif
  float pitchMax =
#ifdef P_AIM_PITCH_MAX
      P_AIM_PITCH_MAX;
#else
      47.0f;
#endif
  float aimHomeDeg = 3365.0f;       // aim phase where SW_RAIL_LOW closes on the backward run (pitch falls through 7 deg)
  float aimMaxDeg = 7600.0f;        // longest aim run (two cycles) before BAD_STATE

  // ---- shot cycle: wiper deg FORWARD from rest, one big gear turn per shot (BASE_SPEC section 1). No sear: the
  // drawn carriage is held by sprag #1 and the wiper worm, released by the relieved last tooth.
  // launch_NOTES.md section 3: big gear 40 T, drive gear 26 T, 33 teeth present: 553.8 wiper deg per shot,
  // carriage released after the last tooth (33/40 of a turn = 457 wiper deg), reset over the 63 deg gap
  // launch_NOTES.md section 3 (11:50 tooth-contact sweep): first rack contact about 2.5 big gear deg after rest,
  // release at big gear 281.25 deg = 432.7 wiper deg, reset through the 117 deg gap
  float cycleDeg =
#ifdef P_LAUNCH_WIPER_TURNS_PER_SHOT
      P_LAUNCH_WIPER_TURNS_PER_SHOT * 360.0f;   // 553.8
#else
      553.8f;
#endif
  float homeOpenDeg = 4.0f;         // SW_HOME opens here (first tooth takes the rack)
  float homeWinDeg = 12.0f;         // ... within this
  float cockEndDeg = 415.0f;        // cocked: stop here, short of the release tooth
  float cockSlowFrom = 380.0f;      // slow from here so the stop is precise
  float releaseDeg = 432.7f;        // the last tooth lets the rack go: carriage flies forward, SW_HOME closes
  float releaseWinDeg = 15.0f;
  float gapCloseDeg = 520.0f;       // SW_GAP closes here on the reset run (big gear entering its gap; the rest point is 0)
  float decockExtraDeg = 15.0f;     // decock: run on this far after SW_HOME closes
  uint32_t cockMaxMs = 20000, fireMaxMs = 20000;
  int decockDuty = 120;

  // ---- drum (N20 worm on DRV8833 #3): lens B table 4.1 (numbers come from params.py DRUM once BASE_SPEC v1.0 has them)
  uint8_t secCode[SEC_COUNT] = {0x1, 0x3, 0x2, 0x6, 0x4, 0x5, 0x7};   // bit0 A, bit1 B, bit2 C: 100 110 010 011 001 101 111 (A B C)
  float edgeDeg[SEC_COUNT - 1] = {8, 32, 50, 62, 302, 340};          // drum deg of the edge between sector i and i+1 (params.DRUM.zones)
  float secLoDeg = -4, secHiDeg = 352;                                // end stops (params.DRUM.end_stops): never driven into
  float targetDeg[4] =
#if defined(P_DRUM_TARGETS_LAUNCH) && defined(P_DRUM_TARGETS_NEUTRAL) && defined(P_DRUM_TARGETS_REEL) && defined(P_DRUM_TARGETS_WRAPPED)
      {P_DRUM_TARGETS_LAUNCH, P_DRUM_TARGETS_NEUTRAL, P_DRUM_TARGETS_REEL, P_DRUM_TARGETS_WRAPPED};
#else
      {0, 41, 347, 314};
#endif                             // park points (params.DRUM.targets): LAUNCH, NEUTRAL, REEL, WRAPPED
  float collarNdeg = 318;                                             // in N_TO_R the collar stays NEUTRAL below this
  float wrapFromDeg = 62, wrapToDeg = 302;                            // the cup lays the wraps linearly here
  int wrapsFull =
#ifdef P_DRUM_WRAP_TURNS
      (int)P_DRUM_WRAP_TURNS;
#else
      7;                                                              // D4 / D16: 7 cup turns
#endif
  float lnEngageDeg = 12, nrEngageDeg = 336;                          // collar still engaged in LAUNCH below / in REEL above
#ifndef P_DRUM_N20_DRUM_DEG_PER_S
#define P_DRUM_N20_DRUM_DEG_PER_S 8.1f
#endif
  float drumDps = P_DRUM_N20_DRUM_DEG_PER_S;             // drum speed estimate (deg/s, cad-drum's N20 drive); re-learned between every two edges
  int drumSign = +1;                // DRV8833 direction that turns the drum toward REEL (BENCH_TEST.md)
  float drumTimeFactor = 1.5f; uint32_t drumTimeMarginMs = 1500;   // lens B I6: 1.5 x nominal (+ margin for the first moves)
  float drumEarlyFrac = 0.25f;      // reaching the target edge faster than this fraction of expected = bad contact
  uint32_t invalidMs = 150;         // code 000 (or a jump over a sector) for this long = DRUM_SENSOR
  // centering jog (lens B 3.7): before every shift the wiper turns this far BACKWARD so the collar sits in the middle of
  // its free dog window and touches no flank (in REEL sprag #2 holds the line meanwhile)
  float dogWindowDeg = 16;          // free play of the collar dogs (unload jog = half of it)
  float centerDeg = 8;              // unload (centering) jog, params.DRUM.unload_jog_deg
  // fly-in (lens B 7.2 steps 8, 15): the wiper creeps forward while the collar drops into REEL; into LAUNCH the collar
  // dropping in is checked with SW_DOG_L and, if needed, the wiper is turned BACKWARD (never forward: that would cock)
  int flyDuty = 64;                 // 25 percent (about 20 percent asked; at least minScaleDuty 60 so the 3 A trip sees the real current)
  float flyMaxDeg = 120;            // at most this much wiper travel for the dogs to drop in
  float flyLaunchMaxDeg = 40;
  // take-up while unwrapping (lens B step 13): wiper forward at 40 percent while the drum is in the WRAP sector
  int takeupDuty = 102; int takeupMaxDuty = 102;   // 40 percent (I4: forward in NEUTRAL or WRAP at most 40 percent)
  uint32_t approachMaxMs = 30000;   // final approach until SW_SEATED with line_m unknown, else DART_STUCK (max_s overrides)
  // take-up line speed in NEUTRAL, slowest case (most line out), physics retrieve.json: 0.058 m/s at duty 0.3, 0.113 at 0.5,
  // 0.26 at 1.0; modelled as takeupMpsPerDuty x duty - takeupMpsOff
  float takeupMpsPerDuty = 0.29f, takeupMpsOff = 0.03f;
  uint32_t tidySteadyMs = 2000; uint32_t tidyMaxMs = 15000;   // reel until slip: forward until the take-up slips steadily
  uint32_t tidyMs = 3000;           // "tidy" default run time (D22: tidy by time)

  // ---- reel (decision D3: capstan 1.667 x wiper; lens B: 258 mm of line per wiper turn)
  float capstanDmm =
#ifdef P_REEL_CAPSTAN_D
      P_REEL_CAPSTAN_D + 1.0f;      // capstan d 40 + line 1.0 = line-centre diameter
#else
      41.0f;
#endif
  float capstanPerWiper =
#ifdef P_REEL_CAPSTAN_RATIO
      P_REEL_CAPSTAN_RATIO;         // 50 / 22 = 2.27
#else
      50.0f / 22.0f;
#endif
  int reelDuty = 220; int reelMinDuty = 70; float taperFrac = 0.30f;
  float kNperA = P_REEL_TOW_TENSION_MODEL_N_PER_A;             // physics v1.2 (FIRM 5 N): line N = 17.0 x (I - I_free) - 10.4. Bench: CALIBRATION.md
  float tensOffsetN = P_REEL_TOW_TENSION_MODEL_OFFSET_N;
  uint32_t rawHoldMs = 60;          // the unfiltered tension estimate at or over max_n this long holds the tow
  float tensionAvgMs = 200;         // D33: the tension estimate is averaged over this long
  float limitN =
#ifdef P_REEL_TOW_LIMIT_N
      P_REEL_TOW_LIMIT_N;
#else
      50.0f;
#endif
#ifndef P_REEL_TOW_LIMIT_MAX_N
#define P_REEL_TOW_LIMIT_MAX_N 70.0f
#endif
  float limitMaxN = P_REEL_TOW_LIMIT_MAX_N;   // D39: above 70 N the 6.5 A trip and the shear pin are reached (BAD_ARG beyond)
  bool armHomeOpen = true;          // cad-reel: SW_ARM is closed while the cup is away and OPENS for about +-5 deg at home
  float resumeFrac = 0.6f;          // holding winds again below this fraction of the limit (load cell only)
  uint32_t tensionIgnoreMs = 150;   // ramp inrush is not tension (the trips ignore 50 ms; a stiff line needs an early look)
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
#ifdef P_AIMTAB_N
static const int AIM_DEF_N = P_AIMTAB_N;
#endif

}  // namespace gb
