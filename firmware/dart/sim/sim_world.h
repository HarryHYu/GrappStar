// The simulated world behind the stub headers: the clock, the motor plant (4 DRV8833 channels and the legs they move),
// the INA219 on the motor rail, and the BNO085 report stream. All of it is shared between the sketch's threads (loop,
// tick task) and the simulator's /sim/ control endpoints, so every entry point takes the world lock.
#pragma once
#include <stdint.h>
#include <string>
#include <vector>

namespace sim {

// ---- clock: millis() = start + elapsed real time (start can be set near 2^32 to test the wrap)
void clockStart(uint32_t millisAtBoot);
uint32_t nowMs();
uint64_t nowUs();                       // microseconds since boot, 64 bit

// ---- motor pins (LEDC duty 0..255 or digitalWrite 0 / 255)
void setPin(int gpio, int duty);

// ---- I2C bus: false / 0 = the address did not ACK
bool i2cProbe(uint8_t addr);
bool i2cWrite(uint8_t addr, const uint8_t* data, int n);
int i2cRead(uint8_t addr, uint8_t* data, int n);

// ---- BNO085
struct ImuReport { uint64_t tUs; uint8_t sensor; float v[4]; };   // accel x,y,z or quaternion i,j,k,real
bool imuPresent();
bool imuEnable(uint8_t sensor, uint32_t intervalUs);
bool imuTakeReset();                    // true once after a simulated chip reset
// One SHTP transfer, the unit one sh2_service() call reads: the oldest reports waiting in the chip that share a
// timestamp (an accelerometer report, a game rotation vector report, or both when they fall due together). Reports
// fall due on their own clock and wait in a bounded FIFO in the chip; when the host reads too slowly the oldest are
// dropped (counted in /sim/state "imu.dropped"). Empty = nothing waiting.
std::vector<ImuReport> imuNextTransfer();
uint32_t imuTransferUs();               // simulated I2C time of one sh2_service() call that moved data

// One step of an acceleration / attitude trace: hold accel (m/s^2, chip axes) for durMs while roll and pitch move
// linearly from the previous step's end values to these ones. After the last step its values hold.
struct TraceStep { double durMs, ax, ay, az, roll, pitch; };
void playTrace(const std::vector<TraceStep>& steps);
std::vector<TraceStep> launchTrace(double pushMs, double flightMs, bool impact);

// ---- boot options and fault knobs
struct BootOptions { bool inaPresent = true, bnoPresent = true; };
void boot(const BootOptions& o);
// key=value knobs (see sim_world.cpp for the list). Returns false and fills err for an unknown key or bad value.
bool setKnob(const std::string& key, const std::string& value, std::string& err);
std::string stateJson();               // plant positions, motor drive, currents, INA219 registers, knobs, IMU
void noteWatchdogTrip();

}  // namespace sim
