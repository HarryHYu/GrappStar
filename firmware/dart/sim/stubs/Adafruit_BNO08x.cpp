// Host implementation of the BNO085 / SH-2 stub. sh2_service() asks the simulated IMU for the reports that fell due
// since the last call, packs each into SH-2 report bytes and hands them to the registered sensor callback, the same
// path the real library takes (the callback runs on the caller's thread, inside sh2_service()).
#include "Adafruit_BNO08x.h"
#include <math.h>
#include <chrono>
#include <thread>
#include "sim_world.h"

namespace {
sh2_SensorCallback_t* callback = nullptr;
void* cookie = nullptr;
uint8_t seq[256];

void put16(uint8_t* p, float v, int q) {             // fixed point Q<q>, little endian, saturating
  double x = round(v * (double)(1 << q));
  if (x > 32767) x = 32767;
  if (x < -32768) x = -32768;
  int16_t i = (int16_t)x; p[0] = (uint8_t)(i & 0xFF); p[1] = (uint8_t)((uint16_t)i >> 8);
}
float get16(const uint8_t* p, int q) { return (int16_t)(p[0] | p[1] << 8) / (float)(1 << q); }
const float ACCEL_CLIP = 78.5f;                      // the BNO085 accelerometer saturates near 8 g
}  // namespace

bool Adafruit_BNO08x::begin_I2C(uint8_t addr, void*, int32_t) { return addr == 0x4A && sim::imuPresent(); }
bool Adafruit_BNO08x::enableReport(sh2_SensorId_t sensor, uint32_t interval_us) { return sim::imuEnable(sensor, interval_us); }
bool Adafruit_BNO08x::wasReset() { return sim::imuTakeReset(); }

int sh2_setSensorCallback(sh2_SensorCallback_t* cb, void* c) { callback = cb; cookie = c; return SH2_OK; }

void sh2_service(void) {
  std::vector<sim::ImuReport> due = sim::imuDue();
  if (due.empty()) return;
  if (uint32_t us = sim::imuTransferUs()) std::this_thread::sleep_for(std::chrono::microseconds(us));   // I2C read time
  for (const sim::ImuReport& r : due) {
    sh2_SensorEvent_t e = {};
    e.timestamp_uS = r.tUs; e.reportId = r.sensor;
    e.report[0] = r.sensor; e.report[1] = seq[r.sensor]++; e.report[2] = 3; e.report[3] = 0;   // id, seq, status, delay
    if (r.sensor == SH2_ACCELEROMETER) {
      for (int i = 0; i < 3; i++) put16(&e.report[4 + 2 * i], fmaxf(-ACCEL_CLIP, fminf(ACCEL_CLIP, r.v[i])), 8);
      e.len = 10;
    } else {
      for (int i = 0; i < 4; i++) put16(&e.report[4 + 2 * i], r.v[i], 14);   // i, j, k, real
      e.len = 12;
    }
    if (callback) callback(cookie, &e);
  }
}

int sh2_decodeSensorEvent(sh2_SensorValue_t* v, const sh2_SensorEvent_t* e) {
  *v = sh2_SensorValue_t();
  v->sensorId = e->reportId; v->sequence = e->report[1]; v->status = e->report[2] & 3; v->timestamp = e->timestamp_uS;
  switch (e->reportId) {
    case SH2_ACCELEROMETER:
      v->un.accelerometer = {get16(&e->report[4], 8), get16(&e->report[6], 8), get16(&e->report[8], 8)};
      return SH2_OK;
    case SH2_GAME_ROTATION_VECTOR:
      v->un.gameRotationVector = {get16(&e->report[4], 14), get16(&e->report[6], 14), get16(&e->report[8], 14), get16(&e->report[10], 14)};
      return SH2_OK;
    default: return SH2_ERR_BAD_PARAM;
  }
}
