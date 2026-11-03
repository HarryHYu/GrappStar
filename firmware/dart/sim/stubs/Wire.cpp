// Host implementation of the TwoWire stub over the simulated I2C bus (sim_world).
#include "Wire.h"
#include "sim_world.h"

TwoWire Wire;

void TwoWire::beginTransmission(uint16_t address) { addr_ = address; txLen_ = 0; }
size_t TwoWire::write(uint8_t b) {
  if (txLen_ >= (int)sizeof tx_) return 0;
  tx_[txLen_++] = b; return 1;
}
uint8_t TwoWire::endTransmission(bool) {
  bool ok = txLen_ ? sim::i2cWrite((uint8_t)addr_, tx_, txLen_) : sim::i2cProbe((uint8_t)addr_);
  txLen_ = 0;
  return ok ? 0 : 2;                                  // 2 = NACK on the address
}
uint8_t TwoWire::requestFrom(uint16_t address, uint8_t quantity, bool) {
  if (quantity > sizeof rx_) quantity = sizeof rx_;
  rxLen_ = sim::i2cRead((uint8_t)address, rx_, quantity); rxPos_ = 0;
  return (uint8_t)rxLen_;
}
