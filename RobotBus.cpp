#include "RobotBus.h"
#include "Config.h"
#include <Wire.h>
bool RobotBus::begin() {
  bool ok = Wire.begin(Config::SDA_PIN, Config::SCL_PIN, Config::I2C_HZ);
  Wire.setTimeOut(Config::I2C_TIMEOUT_MS);
  return ok;
}
bool RobotBus::select(uint8_t reg) {
  Wire.beginTransmission(Config::ADDRESS);
  size_t n = Wire.write(reg);
  uint8_t status = Wire.endTransmission(true);
  return n == 1 && status == 0;
}
bool RobotBus::motors(int left, int right) {
  left = constrain(left, -Config::PWM_LIMIT, Config::PWM_LIMIT);
  right = constrain(right, -Config::PWM_LIMIT, Config::PWM_LIMIT);
  uint16_t l = static_cast<uint16_t>(static_cast<int16_t>(left));
  uint16_t r = static_cast<uint16_t>(static_cast<int16_t>(right));
  uint8_t bytes[4] = {uint8_t(l), uint8_t(l >> 8), uint8_t(r), uint8_t(r >> 8)};
  if (!select(0x06)) return false;
  Wire.beginTransmission(Config::ADDRESS);
  size_t n = Wire.write(bytes, sizeof bytes);
  uint8_t status = Wire.endTransmission(true);
  return n == sizeof bytes && status == 0;
}
bool RobotBus::encoders(uint32_t &leftBits, uint32_t &rightBits) {
  if (!select(0x03)) return false;
  size_t n = Wire.requestFrom(Config::ADDRESS, size_t(8), true);
  if (n != 8) { while (Wire.available()) Wire.read(); return false; }
  uint8_t bytes[8];
  for (unsigned i = 0; i < 8; ++i) {
    if (!Wire.available()) return false;
    bytes[i] = uint8_t(Wire.read());
  }
  uint32_t values[2] = {};
  for (unsigned c = 0; c < 2; ++c)
    for (unsigned i = 0; i < 4; ++i)
      values[c] |= uint32_t(bytes[c * 4 + i]) << (i * 8);
  leftBits = values[0]; rightBits = values[1];
  return true;
}

bool RobotBus::team(const char *name) {
  if (!name || !name[0]) return false;
  uint8_t data[32] = {};
  unsigned n = 0;
  for (; name[n]; ++n) {
    if (n >= 31 || uint8_t(name[n]) < 32 || uint8_t(name[n]) > 126) return false;
    data[n] = uint8_t(name[n]);
  }
  if (!select(0x08)) return false;
  Wire.beginTransmission(Config::ADDRESS);
  size_t written = Wire.write(data, sizeof data);
  uint8_t status = Wire.endTransmission(true);
  return written == sizeof data && status == 0;
}
