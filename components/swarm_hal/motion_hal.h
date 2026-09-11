// Written with Claude
#ifndef MOTION_HAL_H
#define MOTION_HAL_H

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <expected>

#include <geometry_msgs/msg/twist.h>

#include "drive.h"
#include "motor.h"

/*
The design philosophy implemented here is as follows:
- Define abstract types: Motor, etc.
- Use the abstract types to define HAL types
- Use the HAL-defined traits and types to implement a specific driver (e.g., ESP32 Motor Driver)
- Create a global instance (or instances) of the specific driver in the hardware namespace
- Use the global instances throughout application logic (e.g., Consumer task)
*/

namespace HAL {
enum class MotorError : uint8_t { UnknownMotor };

template <typename MotorDriver>
concept MotorDriverTrait = requires(MotorDriver driver, Motor::Command cmd) {
  { driver.run(cmd) } -> std::same_as<std::expected<void, MotorError>>;
  { driver.stop() } -> std::same_as<void>;
};

template <typename DriveStyle, size_t MotorCount>
concept DriveStyleTrait = requires(geometry_msgs__msg__Twist twist_msg) {
  { DriveStyle::type() } -> std::same_as<Drive::Type>;
  {
    DriveStyle::template convert_twist<MotorCount>(twist_msg)
  } -> std::same_as<std::array<Motor::Command, MotorCount>>;
};
} // namespace HAL

#endif
