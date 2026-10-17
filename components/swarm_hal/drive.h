#ifndef DRIVE_H
#define DRIVE_H

#include <algorithm>
#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <ranges>
#include <type_traits>
#include <utility>

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

struct WheelVelocity {
  Motor::Name name;
  double angular_velocity; // rad/s
};

template <Style S>
using WheelVelocities = std::array<WheelVelocity, motor_count<S>>;

// std::views::transform loses the size N of the array
// So collecting back into an array is impossible
// This helper method circumvents that
// Rust has the same issue which is why array::map exists (outside of
// Iterator::map)
template <typename T, size_t N, std::invocable<const T &> F>
constexpr auto map(const std::array<T, N> &in, F f)
    -> std::array<std::invoke_result_t<F, const T &>, N> {
  return [&]<size_t... I>(std::index_sequence<I...>) {
    return std::array<std::invoke_result_t<F, const T &>, N>{f(in[I])...};
  }(std::make_index_sequence<N>{});
}

constexpr auto wheel_velocity(const Wheel &wheel, const Twist &twist)
    -> WheelVelocity {
  return {.name = wheel.name,
          .angular_velocity = wheel.linear_x * twist.linear_x +
                              wheel.linear_y * twist.linear_y +
                              wheel.angular_z * twist.angular_z};
}

template <Style S>
constexpr auto inverse_kinematics(const Twist &twist) -> WheelVelocities<S> {
  return map(wheels<S>(), [&twist](const Wheel &wheel) {
    return wheel_velocity(wheel, twist);
  });
}

// Scale every wheel by the same factor to keep their relative ratio in-tact
template <size_t N>
constexpr auto desaturate(const std::array<WheelVelocity, N> &velocities,
                          double limit = 1.0) -> std::array<WheelVelocity, N> {
  const double peak = std::ranges::fold_left(
      velocities | std::views::transform(&WheelVelocity::angular_velocity),
      limit,
      [](double acc, double v) { return std::max(acc, Motor::magnitude(v)); });

  return map(velocities, [scale = limit / peak](const WheelVelocity &wheel) {
    return WheelVelocity{wheel.name, wheel.angular_velocity * scale};
  });
}

template <Style S>
constexpr auto to_commands(const WheelVelocities<S> &velocities) -> Frame<S> {
  return {map(desaturate(velocities), [](const WheelVelocity &wheel) {
    return Motor::command_for(wheel.name, wheel.angular_velocity);
  })};
}

} // namespace Drive

#endif
