#ifndef SIM_WORLD_H
#define SIM_WORLD_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>

#include "arrays.h"
#include "differential_drive.h"
#include "encoder.h"
#include "motor.h"
#include "robot.h"

// the plant the firmware's controller drives. Its kinematics are written out
// here rather than taken from Drive:: on purpose: a sign error shared by the
// controller and the world it is tested against would cancel and pass
namespace World {
static_assert(Robot::STYLE == Drive::Style::DIFFERENTIAL,
              "the world only models a differential base");

// the ESP32's PCNT, which is the counter the sim stands in for, decodes X4
inline constexpr Encoder::Spec ENCODER{Robot::WHEEL_ENCODER,
                                       Encoder::Decoding::X4};

constexpr size_t N = Drive::motor_count<Robot::STYLE>;

struct Wheel {
  Motor::Name name;
  double gain;  // top speed relative to Robot::MAX_WHEEL_SPEED
  double speed; // rad/s at the rim
  double angle; // rad, what the encoder sees
};

struct Pose {
  double x;
  double y;
  double theta;
};

struct State {
  std::array<Wheel, N> wheels;
  Pose pose;
  double yaw_rate;
};

// fractional ground speed lost or gained against the rim: the encoders turn
// with the wheel, the robot moves with the ground
struct Slip {
  double left;
  double right;
};

// two motors off one production line are never the same speed. A positive
// mismatch slows the right wheel, so an open-loop robot pulls right and the
// integral has something to do
inline State initial(double mismatch, const Pose &start) {
  return {.wheels = Arrays::map(
              Drive::wheels<Robot::STYLE>(),
              [mismatch](const Drive::Wheel &w) {
                const double side = w.angular_z > 0.0 ? -1.0 : 1.0;
                return Wheel{w.name, 1.0 + side * mismatch / 2.0, 0.0, 0.0};
              }),
          .pose = start,
          .yaw_rate = 0.0};
}

constexpr double signed_duty(const Motor::Command &command) {
  switch (command.dir) {
  case Motor::Direction::FORWARD:
    return command.pwm_ratio;
  case Motor::Direction::REVERSE:
    return -command.pwm_ratio;
  case Motor::Direction::STOP:
    return 0.0;
  }

  return 0.0;
}

constexpr double duty_for(const Motor::Frame<N> &frame, Motor::Name name) {
  const auto it =
      std::ranges::find(frame.commands, name, &Motor::Command::name);

  return it == frame.commands.end() ? 0.0 : signed_duty(*it);
}

// first-order lag toward the speed this duty holds at steady state. Full
// duty_limit is the rated voltage, which is where MAX_WHEEL_SPEED was measured
inline Wheel spin(const Wheel &wheel, double duty, double time_constant,
                  double dt) {
  const double steady = duty / Robot::WHEEL_SUPPLY.duty_limit() *
                        Robot::MAX_WHEEL_SPEED * wheel.gain;
  const double speed =
      steady + (wheel.speed - steady) * std::exp(-dt / time_constant);

  return {wheel.name, wheel.gain, speed,
          wheel.angle + (wheel.speed + speed) / 2.0 * dt};
}

constexpr double rim_speed(const State &state, Motor::Name name) {
  const auto it = std::ranges::find(state.wheels, name, &Wheel::name);

  return it == state.wheels.end() ? 0.0 : it->speed;
}

inline State step(const State &state, const Motor::Frame<N> &frame,
                  const Slip &slip, double time_constant, double dt) {
  const auto wheels = Arrays::map(state.wheels, [&](const Wheel &wheel) {
    return spin(wheel, duty_for(frame, wheel.name), time_constant, dt);
  });

  const State spun{wheels, state.pose, state.yaw_rate};
  const double left = rim_speed(spun, Motor::Name::LEFT) * (1.0 + slip.left);
  const double right = rim_speed(spun, Motor::Name::RIGHT) * (1.0 + slip.right);

  const double v = Drive::kWheelRadius * (left + right) / 2.0;
  const double w =
      Drive::kWheelRadius * (right - left) / (2.0 * Drive::kHalfTrack);
  const double mid = state.pose.theta + w * dt / 2.0;

  return {.wheels = wheels,
          .pose = {.x = state.pose.x + v * std::cos(mid) * dt,
                   .y = state.pose.y + v * std::sin(mid) * dt,
                   .theta = std::remainder(state.pose.theta + w * dt,
                                           2.0 * std::numbers::pi)},
          .yaw_rate = w};
}

// cumulative, like a PCNT unit: the firmware's Tracker takes the difference
inline std::array<Encoder::Reading, N> counts(const State &state) {
  return Arrays::map(state.wheels, [](const Wheel &wheel) {
    const double per_radian =
        ENCODER.counts_per_rev() / (2.0 * std::numbers::pi);

    return Encoder::Reading{
        wheel.name, static_cast<int32_t>(std::floor(wheel.angle * per_radian))};
  });
}
} // namespace World

#endif
