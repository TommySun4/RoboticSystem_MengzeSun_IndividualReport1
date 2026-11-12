#pragma once
#include <stdint.h>
class EncoderSpeed {
public:
  void reset() { ready = false; rpm = 0; }
  float update(uint32_t bits, uint32_t now, float cpr, int sign) {
    if (!ready) { previous = bits; timestamp = now; ready = true; return rpm = 0; }
    uint32_t dt = now - timestamp;
    if (!dt) return rpm;
    // Modular subtraction handles signed encoder rollover without signed overflow.
    uint32_t difference = bits - previous;
    int64_t delta = difference <= 0x7fffffffU ? int64_t(difference)
                                            : int64_t(difference) - 0x100000000LL;
    rpm = float(delta) * float(sign) * 60000.0f / (cpr * float(dt));
    previous = bits; timestamp = now;
    return rpm;
  }
private:
  bool ready = false;
  uint32_t previous = 0, timestamp = 0;
  float rpm = 0;
};
