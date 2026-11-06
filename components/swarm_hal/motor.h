#ifndef MOTOR_H
#define MOTOR_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <utility>

/*
Motor Namespace
Defines the abstract types used for a motor.
This does not represent any physical hardware.
Related abstract types would be for IMU, Control (PID), Robot State (IDLE,
ACTIVE), Battery, DistanceSensor, Pose, etc.
*/
namespace Motor {
enum class Direction { FORWARD, REVERSE, STOP };

enum class Name {
  LEFT,
  RIGHT,
  UPPER_LEFT,
  UPPER_RIGHT,
  LOWER_LEFT,
  LOWER_RIGHT,
  FRONT_UMC_A,
  FRONT_UMC_B
};

struct Command {
  Name name;
  Direction dir;
  double pwm_ratio;
};

template <size_t N> struct Frame {
  std::array<Command, N> commands;
};

// duty is a fraction of the rail, not of the motor's rating: a 6V motor on
// a 12V driver sees its rated voltage at half duty and twice it at full
struct Supply {
  double rated;
  double rail;

  constexpr double duty_limit() const {
    return rated < rail ? rated / rail : 1.0;
  }
};

constexpr bool usable(const Supply &supply) {
  return supply.rated > 0.0 && supply.rail > 0.0;
}

constexpr Command command_for(Name name, double velocity) {
  return {name,
          velocity > 0.0   ? Direction::FORWARD
          : velocity < 0.0 ? Direction::REVERSE
                           : Direction::STOP,
          std::abs(velocity)};
}

// motors wired to move as one: no geometry to invert, just an effort in
// [-1, 1] whose sign is the direction. A motor mounted facing the other
// way has its in_a/in_b swapped in the pin table, not here
// TODO: add a support for voltage limit like for PID?
template <auto Names> constexpr Frame<Names.size()> ganged(double effort) {
  return [effort]<size_t... I>(std::index_sequence<I...>) {
    const double clamped = std::clamp(effort, -1.0, 1.0);

    return Frame<Names.size()>{{command_for(Names[I], clamped)...}};
  }(std::make_index_sequence<Names.size()>{});
}
} // namespace Motor

#endif
