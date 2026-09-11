#ifndef DRIVE_H
#define DRIVE_H

#include <array>
#include <cstddef>
#include <cstdint>

#include <geometry_msgs/msg/twist.h>

#include "motor.h"

namespace Drive {
enum class Style : uint8_t { DIFFERENTIAL, ACKERMANN, OMNI };

constexpr size_t motor_count(Style style) {
  switch (style) {
  case Style::DIFFERENTIAL:
    return 2;
  case Style::ACKERMANN:
    return 2;
  case Style::OMNI:
    return 4;
  }

  return 0;
}

template <Style S>
constexpr std::array<Motor::Name, motor_count(S)> motor_names() = delete;

template <Style S>
std::array<Motor::Command, motor_count(S)>
convert_twist(const geometry_msgs__msg__Twist &msg) = delete;
} // namespace Drive

#endif
