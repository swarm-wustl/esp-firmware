#ifndef PID_H
#define PID_H

#include <algorithm>
#include <utility>

namespace Pid {
struct Gains {
  double kp;
  double ki;
  double kd;
};

// a negative gain turns the loop into positive feedback, so it never becomes a
// value
consteval Gains gains(double kp, double ki, double kd) {
  if (kp < 0.0 || ki < 0.0 || kd < 0.0) {
    std::unreachable();
  }

  return Gains{kp, ki, kd};
}

constexpr bool usable(const Gains &gains) {
  return gains.kp >= 0.0 && gains.ki >= 0.0 && gains.kd >= 0.0;
}

struct State {
  double integral;
  double measured;
};

struct Sample {
  double setpoint;
  double measured;
  double feedforward;
};

struct Step {
  State state;
  double output;
};

// derivative on the measurement rather than the error: a new setpoint is a
// step, and differentiating it would kick the output for one period. The
// integral only grows while the output has room -- accumulating against the
// limit winds it up and the loop overshoots once the error flips
constexpr Step step(const Gains &gains, const State &state,
                    const Sample &sample, double dt, double limit = 1.0) {
  const double error = sample.setpoint - sample.measured;
  const double integral = state.integral + error * dt;
  const double rate = (sample.measured - state.measured) / dt;

  const double raw = sample.feedforward + gains.kp * error +
                     gains.ki * integral - gains.kd * rate;
  const double output = std::clamp(raw, -limit, limit);

  const bool winding = raw != output && error * raw > 0.0;

  return {.state = {.integral = winding ? state.integral : integral,
                    .measured = sample.measured},
          .output = output};
}
} // namespace Pid

#endif
