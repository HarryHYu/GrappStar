// Host stub of the ESP32 TwoWire (I2C master). Transfers go to the simulated bus in sim_world: an INA219 at 0x40 and,
// for address scans only, the BNO085 at 0x4A (the BNO085 stub talks SH-2 directly, not through this bus).
// Return codes follow the ESP32 core: endTransmission 0 = ok, 2 = address NACK.
#pragma once
#include <stdint.h>
#include <stddef.h>

class TwoWire {
 public:
  bool begin(int sda = -1, int scl = -1, uint32_t frequency = 0) { (void)sda; (void)scl; (void)frequency; return true; }
  bool setClock(uint32_t hz) { (void)hz; return true; }
  void setTimeOut(uint16_t ms) { (void)ms; }
  void end() {}
  void beginTransmission(uint16_t address);
  size_t write(uint8_t b);
  uint8_t endTransmission(bool sendStop = true);
  uint8_t requestFrom(uint16_t address, uint8_t quantity, bool sendStop = true);
  int available() { return rxLen_ - rxPos_; }
  int read() { return rxPos_ < rxLen_ ? rx_[rxPos_++] : -1; }

 private:
  uint16_t addr_ = 0;
  uint8_t tx_[32]; int txLen_ = 0;
  uint8_t rx_[32]; int rxLen_ = 0, rxPos_ = 0;
};
extern TwoWire Wire;
