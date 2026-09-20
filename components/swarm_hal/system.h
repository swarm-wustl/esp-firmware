// Written with Claude
#ifndef SYSTEM_H
#define SYSTEM_H

#include <concepts>
#include <utility>

#include "assembly.h"
#include "drive.h"
#include "motion_hal.h"
#include "resources.h"

namespace Swarm {
template <typename... Decls>
concept no_resource_conflicts = HAL::no_conflicts(HAL::claims_of<Decls...>());

// the relation between an independently declared geometry and an
// independently declared actuator: can this driver serve this style?
template <typename Driver, Drive::Style S>
concept drives =
    HAL::MotorDriverTrait<Driver, S> && Drive::covered_by<Driver::motors, S>;

template <HAL::Claiming... Peripherals> struct system_impl {
  template <typename T>
  static constexpr bool declared = (std::same_as<T, Peripherals> || ...);

  template <typename T, typename... Args>
    requires declared<T>
  static T make(Args &&...args) {
    return T{Assembly{}, std::forward<Args>(args)...};
  }
};

template <HAL::Claiming... Peripherals>
  requires no_resource_conflicts<Peripherals...>
using system = system_impl<Peripherals...>;

template <Drive::Style S, typename Driver, HAL::Claiming... Peripherals>
struct chassis_impl : system_impl<Driver, Peripherals...> {
  static constexpr Drive::Style style = S;

  using driver_t = Driver;
  using frame_t = Drive::Frame<S>;
};

template <Drive::Style S, typename Driver, HAL::Claiming... Peripherals>
  requires drives<Driver, S> && no_resource_conflicts<Driver, Peripherals...>
using chassis = chassis_impl<S, Driver, Peripherals...>;
} // namespace Swarm

#endif
