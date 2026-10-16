#ifndef DIFFERENTIAL_DRIVE_H
#define DIFFERENTIAL_DRIVE_H

#include <array>

#include "drive.h"
#include "motor.h"

namespace Drive {
// TODO: properly measure these
inline constexpr double kWheelRadius = 0.5;
inline constexpr double kHalfTrack = 0.5;

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
