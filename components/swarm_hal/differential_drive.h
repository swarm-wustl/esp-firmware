#ifndef DIFFERENTIAL_DRIVE_H
#define DIFFERENTIAL_DRIVE_H

#include <array>

#include "drive.h"
#include "motor.h"

namespace Drive {
// half the wheel track. Until wheel velocity is calibrated against duty cycle
// this is "duty per unit of angular_z", not metres -- see kinematics units in
// CLAUDE.md
inline constexpr double kDifferentialHalfTrack = 0.5;

template <> constexpr auto wheels<Style::DIFFERENTIAL>() {
  return std::array{
      Wheel{Motor::Name::LEFT, 1.0, 0.0, -kDifferentialHalfTrack},
      Wheel{Motor::Name::RIGHT, 1.0, 0.0, +kDifferentialHalfTrack},
  };
}
} // namespace Drive

#endif
