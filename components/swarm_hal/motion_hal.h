// Written with Claude
#ifndef MOTION_HAL_H
#define MOTION_HAL_H

#include <concepts>

#include "drive.h"
#include "motor.h"

namespace HAL {
template <typename MotorDriver, auto Names>
concept MotorDriverTrait =
    requires(MotorDriver driver, Drive::Frame<Names> frame) {
      { driver.run(frame) } -> std::same_as<void>;
      { driver.stop() } -> std::same_as<void>;
    };
} // namespace HAL

#endif
