#pragma once
#include <stdint.h>
#include <stdlib.h>

enum class Mode : uint8_t { Start, Ramp, Sequence, Manual, Speed, Deadband,
                           Saturation, Gain, Nonlinear, Step, Continued };
enum class Phase : uint8_t { Idle, Rest, Run, Settle, Measure, Initial,
                            Final, Cool, Done, Aborted };
struct Settings {
  Mode mode = Mode::Manual;
  char wheel = 'L';
  int direction = 1, cap = 60, pwm = 60;
  int low = 0, high = 60, increment = 10, repeats = 3;
  int u0 = 30, u1 = 60;
  uint32_t rest = 1000, settle = 500, measure = 500, cool = 10000;
  uint32_t initial = 700, finalHold = 700;
};
struct Demand { int left = 0, right = 0; };
class Experiment {
public:
  Settings cfg;
  Phase phase = Phase::Idle;
  Demand demand;
  uint32_t batch = 0, trial = 0, began = 0, phaseAt = 0, commandAt = 0;
  uint32_t quietAt = 0;
  int level = 0, repeat = 0;
  bool active = false, restFault = false;
  bool valid() const {
    if (cfg.cap < 1 || cfg.cap > 200 || cfg.pwm < 0 || cfg.pwm > cfg.cap ||
        (cfg.direction != 1 && cfg.direction != -1) ||
        (cfg.wheel != 'L' && cfg.wheel != 'R' && cfg.wheel != 'B') ||
        cfg.repeats < 1 || cfg.repeats > 20 || cfg.low < 0 || cfg.high > cfg.cap ||
        cfg.high < cfg.low || cfg.increment < 1 || cfg.increment > 200 ||
        cfg.u0 < 0 || cfg.u1 < 0 || cfg.u0 > cfg.cap || cfg.u1 > cfg.cap ||
        cfg.rest < 500 || cfg.rest > 5000 || cfg.settle < 100 ||
        cfg.measure < 200 || cfg.settle + cfg.measure > 2000 ||
        cfg.cool < 2000 || cfg.cool > 60000 || cfg.initial < 300 ||
        cfg.finalHold < 300 || cfg.initial + cfg.finalHold > 2000) return false;
    if (cfg.mode >= Mode::Deadband && cfg.wheel == 'B') return false;
    if (cfg.mode == Mode::Deadband && cfg.high > 100) return false;
    if (cfg.mode <= Mode::Speed && cfg.pwm > 100) return false;
    if (cfg.mode == Mode::Continued && (cfg.high > 100 ||
        uint32_t((cfg.high-cfg.low)/cfg.increment+1)*(cfg.settle+cfg.measure)>8000)) return false;
    return true;
  }
  bool start(uint32_t now) {
    if (active || !valid()) return false;
    ++batch; trial = 1; repeat = 1;
    level = cfg.mode == Mode::Continued ? cfg.high : cfg.low;
    began = phaseAt = quietAt = commandAt = now;
    restFault = false; active = true; demand = Demand{};
    phase = Phase::Rest;
    return true;
  }
  void stop(bool abort = false) {
    demand = Demand{}; active = false;
    phase = abort ? Phase::Aborted : Phase::Done;
  }
  void observe(bool moved, uint32_t now) { if (moved) quietAt = now; }
  void tick(uint32_t now) {
    if (!active) return;
    if (now - began > 1800000UL) { restFault = true; stop(true); return; }
    uint32_t elapsed = now - phaseAt;
    if (phase == Phase::Rest) {
      if (elapsed >= cfg.rest && now - quietAt >= 300) beginPowered(now);
      else if (elapsed > cfg.rest + 5000) { restFault = true; stop(true); }
      return;
    }
    if (phase == Phase::Cool) {
      if (elapsed >= cfg.cool) {
        if (!next()) stop();
        else enter(Phase::Rest, now);
      }
      return;
    }
    switch (cfg.mode) {
      case Mode::Start:
        if (elapsed >= 3000) finish(now);
        break;
      case Mode::Ramp: {
        if (elapsed >= 6000) { finish(now); break; }
        int value = elapsed < 3000 ? int(uint64_t(cfg.pwm) * elapsed / 3000)
                                      : int(uint64_t(cfg.pwm) * (6000 - elapsed) / 3000);
        apply(value, now); break;
      }
      case Mode::Sequence:
        if (elapsed >= 9000) finish(now);
        else {
          bool turn = elapsed % 3000 >= 2000;
          demand.left = cfg.direction * cfg.pwm;
          demand.right = cfg.direction * cfg.pwm * (turn ? -1 : 1);
        }
        break;
      case Mode::Manual:
        if (elapsed >= 30000) finish(now);
        break;
      case Mode::Speed:
        if (elapsed >= 3000) finish(now);
        break;
      case Mode::Step:
        if (phase == Phase::Initial && elapsed >= cfg.initial) {
          enter(Phase::Final, now); apply(cfg.u1, now);
        } else if (phase == Phase::Final && elapsed >= cfg.finalHold) finish(now);
        break;
      default:
        if (phase == Phase::Settle && elapsed >= cfg.settle) enter(Phase::Measure, now);
        else if (phase == Phase::Measure && elapsed >= cfg.measure) {
          if (cfg.mode == Mode::Continued && level - cfg.increment >= cfg.low) {
            level -= cfg.increment; ++trial; enter(Phase::Settle, now); apply(level, now);
          } else finish(now);
        }
        break;
    }
  }
private:
  void enter(Phase p, uint32_t now) { phase = p; phaseAt = now; }
  void apply(int magnitude, uint32_t now) {
    int value = magnitude * cfg.direction;
    Demand nextDemand;
    if (cfg.wheel != 'R') nextDemand.left = value;
    if (cfg.wheel != 'L' || cfg.mode == Mode::Sequence) nextDemand.right = value;
    if (cfg.mode == Mode::Start || cfg.mode == Mode::Ramp) {
      nextDemand.left = value; nextDemand.right = -value;
    }
    if (nextDemand.left != demand.left || nextDemand.right != demand.right) commandAt = now;
    demand = nextDemand;
  }
  void beginPowered(uint32_t now) {
    commandAt = now;
    if (cfg.mode == Mode::Step) { enter(Phase::Initial, now); apply(cfg.u0, now); }
    else if (cfg.mode >= Mode::Deadband) { enter(Phase::Settle, now); apply(level, now); }
    else {
      enter(Phase::Run, now);
      if (cfg.mode != Mode::Manual) apply(cfg.mode == Mode::Ramp ? 0 : cfg.pwm, now);
    }
  }
  void finish(uint32_t now) { demand = Demand{}; commandAt = now; enter(Phase::Cool, now); }
  bool next() {
    bool sweep = cfg.mode >= Mode::Deadband && cfg.mode != Mode::Step;
    if (sweep && cfg.mode != Mode::Continued && level + cfg.increment <= cfg.high) {
      level += cfg.increment;
    } else {
      if (++repeat > cfg.repeats) return false;
      level = cfg.mode == Mode::Continued ? cfg.high : cfg.low;
    }
    ++trial; return true;
  }
};
