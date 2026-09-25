#ifndef DRIVE_H
#define DRIVE_H

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

#include "motor.h"

namespace Drive {
enum class Style : uint8_t { DIFFERENTIAL, ACKERMANN, OMNI, MECANUM };

struct Twist {
  double linear_x;
  double linear_y;
  double angular_z;
};

// one row of the inverse-kinematic matrix H: a wheel's velocity is a linear
// combination of the body twist (Modern Robotics, u = H(0) * V_b). Every
// fixed-orientation-wheel platform -- differential, skid, omni, mecanum -- is
// exactly this; steered platforms (Ackermann, swerve) are not and need their
// own transform
struct Wheel {
  Motor::Name name;
  double linear_x;
  double linear_y;
  double angular_z;
};

template <Style S> constexpr auto wheels();

template <Style S> inline constexpr size_t motor_count = wheels<S>().size();

// the roster and the style name the same motors, one each. A motor the
// geometry never commands would sit enabled and idle, so it belongs in its own
// declaration rather than the drive roster
template <auto Names, Style S> consteval bool covers() {
  constexpr auto rows = wheels<S>();

  if (Names.size() != rows.size()) {
    return false;
  }

  return std::ranges::all_of(rows, [](const Wheel &wheel) {
    return std::ranges::count(Names, wheel.name) == 1;
  });
}

template <auto Names, Style S>
concept covered_by = covers<Names, S>();

template <Style S> struct Frame {
  std::array<Motor::Command, motor_count<S>> commands;
};

constexpr double magnitude(double v) { return v < 0.0 ? -v : v; }

constexpr double wheel_velocity(const Wheel &wheel, const Twist &twist) {
  return wheel.linear_x * twist.linear_x + wheel.linear_y * twist.linear_y +
         wheel.angular_z * twist.angular_z;
}

// scaling every wheel by the same factor keeps the mix -- and so the heading --
// that clamping each one independently would distort
template <size_t N>
constexpr double saturation_divisor(const std::array<double, N> &velocities) {
  double peak = 1.0;

  for (double v : velocities) {
    peak = magnitude(v) > peak ? magnitude(v) : peak;
  }

  return peak;
}

constexpr Motor::Command command_for(Motor::Name name, double velocity) {
  return {name,
          velocity > 0.0   ? Motor::Direction::FORWARD
          : velocity < 0.0 ? Motor::Direction::REVERSE
                           : Motor::Direction::STOP,
          magnitude(velocity)};
}

// TODO: actually have this be inverse kinematics...
// then have a separate stage that converts vel -> duty cycles for motors
template <Style S> constexpr Frame<S> inverse_kinematics(const Twist &twist) {
  constexpr auto rows = wheels<S>();

  std::array<double, motor_count<S>> velocities{};

  for (size_t i = 0; i < rows.size(); ++i) {
    velocities[i] = wheel_velocity(rows[i], twist);
  }

  const double divisor = saturation_divisor(velocities);

  Frame<S> frame{};

  for (size_t i = 0; i < rows.size(); ++i) {
    frame.commands[i] = command_for(rows[i].name, velocities[i] / divisor);
  }

  return frame;
}
} // namespace Drive

#endif
