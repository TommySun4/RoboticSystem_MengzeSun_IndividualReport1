#pragma once
#include <Arduino.h>
class RobotBus {
public:
  bool begin();
  bool team(const char *name);
  bool motors(int left, int right);
  bool encoders(uint32_t &leftBits, uint32_t &rightBits);
private:
  bool select(uint8_t reg);
};
