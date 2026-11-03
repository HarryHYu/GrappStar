// Host stub of Adafruit_BNO08x plus the parts of the CEVA SH-2 API (sh2.h, sh2_SensorValue.h, sh2_err.h) the sketch
// uses. Reports come from the simulated IMU in sim_world (a scriptable acceleration / attitude trace). They are packed
// into SH-2 report bytes in the real fixed-point formats (accelerometer Q8 m/s^2, game rotation vector Q14) and
// unpacked by sh2_decodeSensorEvent, so the sketch sees the same quantisation as on the chip.
#pragma once
#include <stdint.h>

// ---- sh2_err.h
#define SH2_OK 0
#define SH2_ERR (-1)
#define SH2_ERR_BAD_PARAM (-2)

// ---- sh2.h
typedef uint8_t sh2_SensorId_t;
enum {
  SH2_RAW_ACCELEROMETER = 0x14,
  SH2_ACCELEROMETER = 0x01,
  SH2_GYROSCOPE_CALIBRATED = 0x02,
  SH2_MAGNETIC_FIELD_CALIBRATED = 0x03,
  SH2_LINEAR_ACCELERATION = 0x04,
  SH2_ROTATION_VECTOR = 0x05,
  SH2_GRAVITY = 0x06,
  SH2_GAME_ROTATION_VECTOR = 0x08,
};
#define SH2_MAX_SENSOR_EVENT_LEN 60
typedef struct sh2_SensorEvent {
  uint64_t timestamp_uS;
  int64_t delay_uS;
  uint8_t len;
  uint8_t reportId;
  uint8_t report[SH2_MAX_SENSOR_EVENT_LEN];
} sh2_SensorEvent_t;
typedef void(sh2_SensorCallback_t)(void* cookie, sh2_SensorEvent_t* pEvent);
int sh2_setSensorCallback(sh2_SensorCallback_t* callback, void* cookie);
void sh2_service(void);

// ---- sh2_SensorValue.h
typedef struct { float x, y, z; } sh2_Accelerometer_t;
typedef struct { float i, j, k, real; } sh2_RotationVector_t;
typedef struct sh2_SensorValue {
  uint8_t sensorId;
  uint8_t sequence;
  uint8_t status;
  uint64_t timestamp;
  uint32_t delay;
  union {
    sh2_Accelerometer_t accelerometer;
    sh2_RotationVector_t gameRotationVector;
  } un;
} sh2_SensorValue_t;
int sh2_decodeSensorEvent(sh2_SensorValue_t* value, const sh2_SensorEvent_t* event);

// ---- Adafruit_BNO08x
class Adafruit_BNO08x {
 public:
  explicit Adafruit_BNO08x(int8_t resetPin = -1) { (void)resetPin; }
  bool begin_I2C(uint8_t addr = 0x4A, void* wire = nullptr, int32_t sensorId = 0);
  bool enableReport(sh2_SensorId_t sensor, uint32_t interval_us = 10000);
  bool wasReset();
};
