// Written with Claude
#ifndef DIFFERENTIAL_DRIVE_H
#define DIFFERENTIAL_DRIVE_H

#include <algorithm>
#include <array>
#include <cstddef>

#include <geometry_msgs/msg/twist.h>

#include "drive.h"
#include "motor.h"

namespace Drive {
class Differential {
public:
  Differential() = delete;

  static Type type() { return Type::DIFFERENTIAL; }

  template <size_t MotorCount>
  static std::array<Motor::Command, MotorCount>
  convert_twist(geometry_msgs__msg__Twist msg) {
    static_assert(MotorCount == 2, "differential drive turns two motors");

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

    return std::array<Motor::Command, MotorCount>{
        Motor::Command{Motor::Name::LEFT, dir, pwm_ratio},
        Motor::Command{Motor::Name::RIGHT, dir, pwm_ratio}};
  }
};
} // namespace Drive

#endif
