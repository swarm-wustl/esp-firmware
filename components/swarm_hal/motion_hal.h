// Written with Claude
#ifndef MOTION_HAL_H
#define MOTION_HAL_H

#include <concepts>

#include "motor.h"

namespace HAL {
template <typename MotorDriver>
concept MotorDriverTrait = requires(MotorDriver driver, Motor::Command cmd) {
  { driver.run(cmd) } -> std::same_as<void>;
  { driver.stop() } -> std::same_as<void>;
};
} // namespace HAL

#endif
