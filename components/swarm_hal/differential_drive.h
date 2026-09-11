#ifndef DIFFERENTIAL_DRIVE_H
#define DIFFERENTIAL_DRIVE_H

#include <algorithm>
#include <array>

#include "drive.h"
#include "motor.h"

namespace Drive {
template <>
constexpr std::array<Motor::Name, 2> motor_names<Style::DIFFERENTIAL>() {
  return {Motor::Name::LEFT, Motor::Name::RIGHT};
}

template <>
inline std::array<Motor::Command, 2>
convert_twist<Style::DIFFERENTIAL>(const geometry_msgs__msg__Twist &msg) {
  // TODO: handle angular later
  double linear_velocity = msg.linear.x;

  Motor::Direction dir;
  double pwm_ratio;

  if (linear_velocity < 0) {
    dir = Motor::Direction::REVERSE;
    pwm_ratio = linear_velocity * -1.0;
  } else if (linear_velocity > 0) {
    dir = Motor::Direction::FORWARD;
    pwm_ratio = linear_velocity;
  } else {
    dir = Motor::Direction::STOP;
    pwm_ratio = 0.0;
  }

  pwm_ratio = std::clamp(pwm_ratio, 0.0, 1.0);

  return std::array<Motor::Command, 2>{
      Motor::Command{Motor::Name::LEFT, dir, pwm_ratio},
      Motor::Command{Motor::Name::RIGHT, dir, pwm_ratio}};
}
} // namespace Drive

#endif
