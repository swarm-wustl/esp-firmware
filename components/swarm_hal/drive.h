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
#include "pid.h"

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
                          double limit) -> std::array<WheelVelocity, N> {
  const double peak = std::ranges::fold_left(
      velocities | std::views::transform(&WheelVelocity::angular_velocity),
      limit,
      [](double acc, double v) { return std::max(acc, Motor::magnitude(v)); });

  return map(velocities, [scale = limit / peak](const WheelVelocity &wheel) {
    return WheelVelocity{wheel.name, wheel.angular_velocity * scale};
  });
}

struct Control {
  Motor::Supply supply;
  double max_wheel_speed; // rad/s at the rated voltage
  double period;          // s
  Pid::Gains gains;
};

constexpr bool usable(const Control &control) {
  return Motor::usable(control.supply) && control.max_wheel_speed > 0.0 &&
         control.period > 0.0 && Pid::usable(control.gains);
}

consteval Control control(Motor::Supply supply, double max_wheel_speed,
                          double period, Pid::Gains gains) {
  const Control out{supply, max_wheel_speed, period, gains};

  if (!usable(out)) {
    std::unreachable();
  }

  return out;
}

template <Style S>
using Regulators = std::array<Pid::State, motor_count<S>>;

template <Style S> struct Regulated {
  Regulators<S> regulators;
  Frame<S> frame;
};

template <size_t N>
constexpr double velocity_of(const std::array<WheelVelocity, N> &velocities,
                             Motor::Name name) {
  const auto it = std::ranges::find(velocities, name, &WheelVelocity::name);

  return it != velocities.end() ? it->angular_velocity : 0.0;
}

// a stopped wheel coasts instead of fighting to hold zero, and drops its
// integral so the next command starts clean
constexpr auto regulate_wheel(const Control &control, const Pid::State &state,
                              double target, double measured) -> Pid::Step {
  if (target == 0.0) {
    return {.state = {.integral = 0.0, .measured = measured}, .output = 0.0};
  }

  const double limit = control.supply.duty_limit();

  return Pid::step(control.gains, state,
                   {.setpoint = target,
                    .measured = measured,
                    .feedforward = target / control.max_wheel_speed * limit},
                   control.period, limit);
}

// targets are capped at the wheel's top speed before the loop sees them: an
// unreachable setpoint would wind the integral up and break the wheel ratio
// desaturate exists to keep
template <Style S, Control C>
  requires(usable(C))
constexpr auto regulate(const Regulators<S> &regulators,
                        const WheelVelocities<S> &targets,
                        const WheelVelocities<S> &measured) -> Regulated<S> {
  const auto capped = desaturate(targets, C.max_wheel_speed);

  return [&]<size_t... I>(std::index_sequence<I...>) {
    const std::array steps{
        regulate_wheel(C, regulators[I], capped[I].angular_velocity,
                       velocity_of(measured, capped[I].name))...};

    return Regulated<S>{
        .regulators = {steps[I].state...},
        .frame = {{Motor::command_for(capped[I].name, steps[I].output)...}}};
  }(std::make_index_sequence<motor_count<S>>{});
}

} // namespace Drive

#endif
