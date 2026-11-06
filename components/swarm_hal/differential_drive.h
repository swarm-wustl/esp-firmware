#ifndef DIFFERENTIAL_DRIVE_H
#define DIFFERENTIAL_DRIVE_H

#include <array>

#include "drive.h"
#include "motor.h"
#include "sdkconfig.h"


namespace Drive {
// TODO: properly measure these
  inline constexpr double K_WHEEL_RADIUS = CONFIG_K_WHEEL_RADIUS_MM / 1000.0; // m
  inline constexpr double K_HALF_TRACK   = CONFIG_K_HALF_TRACK_MM / 1000.0;   // m

  static_assert(K_WHEEL_RADIUS > 0.0, "Set wheel radius in menuconfig");
  static_assert(K_HALF_TRACK > 0.0, "Set half track in menuconfig");


  template <> constexpr auto wheels<Style::DIFFERENTIAL>() {
    return std::array{
        Wheel{Motor::Name::LEFT, 1.0 / K_WHEEL_RADIUS, 0.0,
              -K_HALF_TRACK / K_WHEEL_RADIUS},
        Wheel{Motor::Name::RIGHT, 1.0 / K_WHEEL_RADIUS, 0.0,
              +K_HALF_TRACK / K_WHEEL_RADIUS},
    };
  }
} // namespace Drive

#endif
