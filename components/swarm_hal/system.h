// Written with Claude
#ifndef SYSTEM_H
#define SYSTEM_H

#include <utility>

#include "assembly.h"
#include "drive.h"
#include "motion_hal.h"
#include "resources.h"

namespace Swarm {
// the relation between an independently declared geometry and an
// independently declared actuator: can this driver serve this style?
template <typename Driver, Drive::Style S>
concept drives =
    HAL::MotorDriverTrait<Driver, S> && Drive::covered_by<Driver::motors, S>;

template <typename... Decls>
concept no_resource_conflicts = HAL::no_conflicts(HAL::claims_of<Decls...>());

template <Drive::Style S, typename Driver, typename... Peripherals>
struct chassis_impl {
  static constexpr Drive::Style style = S;

  using driver_t = Driver;
  using frame_t = Drive::Frame<S>;

  template <typename T, typename... Args> static T make(Args &&...args) {
    return T{Assembly{}, std::forward<Args>(args)...};
  }

  template <typename Gpio, typename Pwm> static Driver motors(Gpio gpio, Pwm pwm) {
    return Driver{Assembly{}, std::move(gpio), std::move(pwm)};
  }
};

template <Drive::Style S, typename Driver, typename... Peripherals>
  requires drives<Driver, S> && no_resource_conflicts<Driver, Peripherals...>
using chassis = chassis_impl<S, Driver, Peripherals...>;
} // namespace Swarm

#endif
