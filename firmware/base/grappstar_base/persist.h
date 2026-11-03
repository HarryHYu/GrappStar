// What survives a power cycle (ESP32 NVS through Preferences in the sketch). The worm gears on the wiper and on the
// drum N20 both self-lock, so the mechanism stays where it stopped. The rule from the old ground station:
//  - at every motion start the record is written with moving = 1 ("position unknown" until the motion ends);
//  - when everything has stopped it is written again with the final positions and moving = 0.
// So after a power cut mid-motion the base comes up knowing only what its sensors can prove (contacts, the AS5600
// angle inside one turn), never a stale number. Plain C++, tested on a PC.
#pragma once
#include <stdint.h>
#include <string.h>

namespace gb {

static const uint32_t SAVE_MAGIC = 0x47534231;   // "GSB1"
static const uint16_t SAVE_VER = 3;

struct Saved {
  uint32_t magic = SAVE_MAGIC;
  uint16_t ver = SAVE_VER;
  uint8_t moving = 0;           // 1 = a motion was running when this was written: positions below are NOT trusted
  uint8_t drumKnown = 0;
  float drumDeg = 0;            // drum angle estimate (deg of drum from the LAUNCH contact)
  int16_t wraps = 0;            // capstan wraps laid by the wrap arm
  uint8_t wrapsKnown = 1;
  uint8_t wiperKnown = 0;
  float wiperDeg = 0;           // wiper output multi-turn angle (AS5600 count, deg)
  float aimPhase = 0;           // aim train phase (deg of wiper travel in the aim direction, 0 = home)
  float cockPhase = 0;          // cock train phase (deg of wiper travel in the cock direction within one shot cycle)
  uint8_t aimKnown = 0, cockKnown = 0;
  uint8_t phase = 0;            // shot cycle phase (enum ShotPhase in base.h)
  uint8_t pad = 0;
  float lineM = 0;              // line out (estimate)
  uint32_t seq = 0;             // write counter
  uint32_t crc = 0;
};

static_assert(sizeof(Saved) == 44, "Saved layout changed: bump SAVE_VER");

static uint32_t crc32(const uint8_t* d, int n) {
  uint32_t c = 0xFFFFFFFFu;
  for (int i = 0; i < n; i++) { c ^= d[i]; for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1))); }
  return ~c;
}
static void sealSaved(Saved* s) { s->magic = SAVE_MAGIC; s->ver = SAVE_VER; s->crc = crc32((const uint8_t*)s, (int)(sizeof(Saved) - 4)); }
static bool validSaved(const Saved* s) {
  return s->magic == SAVE_MAGIC && s->ver == SAVE_VER && s->crc == crc32((const uint8_t*)s, (int)(sizeof(Saved) - 4));
}
static bool sameSaved(const Saved& a, const Saved& b) {   // ignoring seq / crc
  Saved x = a, y = b; x.seq = y.seq = 0; x.crc = y.crc = 0; return memcmp(&x, &y, sizeof(Saved)) == 0;
}

// Bench settings changed with "set" or learned with "cal" (kept apart from the position record, written rarely)
static const uint32_t TUN_MAGIC = 0x47535431;   // "GST1"
struct Tun {
  uint32_t magic = TUN_MAGIC; uint16_t ver = 1; uint8_t encSign = 1; uint8_t requireSeated = 1;
  float ampsPerVolt = 0, offsetA = 0, freeA = 0, kNperA = 0, hxNPerCount = 0, hxOff = 0, limitN = 0, drumDps = 0;
  float aimOffset = 0, aimTol = 0, tripA = 0, cockTripA = 0, vbatLow = 0, aimHomeDeg = 0;
  uint32_t crc = 0;
};
static_assert(sizeof(Tun) == 68, "Tun layout changed: bump ver");
static void sealTun(Tun* t) { t->magic = TUN_MAGIC; t->crc = crc32((const uint8_t*)t, (int)(sizeof(Tun) - 4)); }
static bool validTun(const Tun* t) { return t->magic == TUN_MAGIC && t->ver == 1 && t->crc == crc32((const uint8_t*)t, (int)(sizeof(Tun) - 4)); }

}  // namespace gb
