#ifndef DIFFERENTIAL_DRIVE_H
#define DIFFERENTIAL_DRIVE_H

#include <array>

#include "drive.h"
#include "motor.h"
#include "sdkconfig.h"

namespace Drive {
// TODO: properly measure these
inline constexpr double kWheelRadius = CONFIG_K_WHEEL_RADIUS_MM / 1000.0; // m
inline constexpr double kHalfTrack = CONFIG_K_HALF_TRACK_MM / 1000.0;     // m

static_assert(kWheelRadius > 0.0, "Set wheel radius in menuconfig");
static_assert(kHalfTrack > 0.0, "Set half track in menuconfig");

template <> constexpr auto wheels<Style::DIFFERENTIAL>() {
  return std::array{
      Wheel{Motor::Name::LEFT, 1.0 / kWheelRadius, 0.0,
            -kHalfTrack / kWheelRadius},
      Wheel{Motor::Name::RIGHT, 1.0 / kWheelRadius, 0.0,
            +kHalfTrack / kWheelRadius},
  };
}
} // namespace Drive

#endif
