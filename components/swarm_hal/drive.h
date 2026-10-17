#ifndef DRIVE_H
#define DRIVE_H

#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <ranges>
#include <type_traits>
#include <utility>

#include "arrays.h"
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

constexpr auto wheel_velocity(const Wheel &wheel, const Twist &twist)
    -> WheelVelocity {
  return {.name = wheel.name,
          .angular_velocity = wheel.linear_x * twist.linear_x +
                              wheel.linear_y * twist.linear_y +
                              wheel.angular_z * twist.angular_z};
}

template <Style S>
constexpr auto inverse_kinematics(const Twist &twist) -> WheelVelocities<S> {
  return Arrays::map(wheels<S>(), [&twist](const Wheel &wheel) {
    return wheel_velocity(wheel, twist);
  });
}

template <size_t N>
constexpr double velocity_of(const std::array<WheelVelocity, N> &velocities,
                             Motor::Name name) {
  const auto it = std::ranges::find(velocities, name, &WheelVelocity::name);

  return it != velocities.end() ? it->angular_velocity : 0.0;
}

constexpr Twist operator+(const Twist &a, const Twist &b) {
  return {a.linear_x + b.linear_x, a.linear_y + b.linear_y,
          a.angular_z + b.angular_z};
}

constexpr Twist operator*(const Twist &twist, double k) {
  return {twist.linear_x * k, twist.linear_y * k, twist.angular_z * k};
}

// the body twist one rad/s of this wheel contributes, i.e. one column of the
// pseudo-inverse of H
struct Contribution {
  Motor::Name name;
  Twist twist;
};

namespace detail {
using Matrix3 = std::array<std::array<double, 3>, 3>;

constexpr std::array<double, 3> coefficients(const Wheel &wheel) {
  return {wheel.linear_x, wheel.linear_y, wheel.angular_z};
}

consteval double determinant(const Matrix3 &m) {
  return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
         m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
         m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

consteval Matrix3 inverse(const Matrix3 &m) {
  const double det = determinant(m);

  // wheels that cannot tell two motions apart -- two identical rows, say --
  // have no forward kinematics, and that should not build
  if (det == 0.0) {
    std::unreachable();
  }

  const auto cofactor = [&m](size_t r, size_t c) {
    const size_t r0 = (r + 1) % 3, r1 = (r + 2) % 3;
    const size_t c0 = (c + 1) % 3, c1 = (c + 2) % 3;

    return m[r0][c0] * m[r1][c1] - m[r0][c1] * m[r1][c0];
  };

  Matrix3 out{};

  for (size_t r = 0; r < 3; ++r) {
    for (size_t c = 0; c < 3; ++c) {
      out[r][c] = cofactor(c, r) / det;
    }
  }

  return out;
}

// a nonholonomic base has a motion no wheel produces, so H^T H is singular in
// that axis. A unit on its diagonal leaves the other block untouched and makes
// the inverse exist, and since H^T is zero in that row the axis comes out zero
consteval Matrix3 normal(const auto &rows) {
  Matrix3 m{};

  for (const Wheel &wheel : rows) {
    const auto h = coefficients(wheel);

    for (size_t r = 0; r < 3; ++r) {
      for (size_t c = 0; c < 3; ++c) {
        m[r][c] += h[r] * h[c];
      }
    }
  }

  for (size_t axis = 0; axis < 3; ++axis) {
    if (m[axis][axis] == 0.0) {
      m[axis][axis] = 1.0;
    }
  }

  return m;
}
} // namespace detail

template <Style S> consteval auto contributions() {
  constexpr auto rows = wheels<S>();
  constexpr auto inv = detail::inverse(detail::normal(rows));

  return Arrays::map(rows, [&inv](const Wheel &wheel) {
    const auto h = detail::coefficients(wheel);
    const auto axis = [&](size_t r) {
      return inv[r][0] * h[0] + inv[r][1] * h[1] + inv[r][2] * h[2];
    };

    return Contribution{wheel.name, {axis(0), axis(1), axis(2)}};
  });
}

// least squares: with more wheels than the body has freedoms -- a skid steer,
// a mecanum -- a slipping wheel is outvoted rather than trusted
template <Style S>
constexpr auto forward_kinematics(const WheelVelocities<S> &measured)
    -> Twist {
  constexpr auto columns = contributions<S>();

  return std::ranges::fold_left(
      columns, Twist{}, [&measured](const Twist &acc, const Contribution &c) {
        return acc + c.twist * velocity_of(measured, c.name);
      });
}

// Scale every wheel by the same factor to keep their relative ratio in-tact
template <size_t N>
constexpr auto desaturate(const std::array<WheelVelocity, N> &velocities,
                          double limit) -> std::array<WheelVelocity, N> {
  const double peak = std::ranges::fold_left(
      velocities | std::views::transform(&WheelVelocity::angular_velocity),
      limit,
      [](double acc, double v) { return std::max(acc, std::abs(v)); });

  return Arrays::map(velocities, [scale = limit / peak](const WheelVelocity &wheel) {
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
