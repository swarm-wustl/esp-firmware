#ifndef DRIVE_H
#define DRIVE_H

#include <algorithm>
#include <array>
#include <concepts>
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
template <Style S, size_t N>
consteval bool covers_roster(const std::array<Motor::Name, N> &names) {
  constexpr auto rows = wheels<S>();

  if (names.size() != rows.size()) {
    return false;
  }

  return std::ranges::all_of(rows, [&names](const Wheel &wheel) {
    return std::ranges::count(names, wheel.name) == 1;
  });
}

template <auto Names, Style S> consteval bool covers() {
  return covers_roster<S>(Names);
}

template <auto Names, Style S>
concept covered_by = covers<Names, S>();

template <Style S> using Frame = Motor::Frame<motor_count<S>>;

// rad/s
template <Style S>
using WheelAngularVelocities = std::array<double, motor_count<S>>;

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
    peak = Motor::magnitude(v) > peak ? Motor::magnitude(v) : peak;
  }

  return peak;
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
    frame.commands[i] =
        Motor::command_for(rows[i].name, velocities[i] / divisor);
  }

  return frame;
}

template <Style S>
constexpr auto inverse_kinematics_pure(const Twist &twist)
    -> WheelAngularVelocities<S> {
  constexpr auto rows = wheels<S>();

  WheelAngularVelocities<S> angular_velocities{};

  for (size_t i = 0; i < rows.size(); ++i) {
    angular_velocities[i] = wheel_velocity(rows[i], twist);
  }

  return angular_velocities;
}

template <std::floating_point T, size_t N>
constexpr auto desaturate(std::array<T, N> data, T limit = 1)
    -> std::array<T, N> {
  // Find the absolute peak (+/-)
  const T peak = std::ranges::fold_left(
      data, limit, [](T acc, T v) { return std::max(acc, v < 0 ? -v : v); });

  std::ranges::transform(data, data.begin(),
                         [scale = limit / peak](T v) { return v * scale; });

  return data;
}

template <Style S>
constexpr auto to_commands(const WheelAngularVelocities<S> &velocities)
    -> Frame<S> {
  // TODO: consider holding names in the angular velocities struct directly
  constexpr auto rows = wheels<S>();
  auto desaturated = desaturate(velocities);

  Frame<S> res{};

  for (size_t i = 0; i < motor_count<S>; ++i) {
    res.commands[i] = Motor::command_for(rows[i].name, desaturated[i]);
  }

  return res;
}

} // namespace Drive

#endif
