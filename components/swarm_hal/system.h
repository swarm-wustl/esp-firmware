#ifndef SYSTEM_H
#define SYSTEM_H

#include <array>
#include <concepts>
#include <cstddef>
#include <optional>
#include <tuple>
#include <utility>

#include <cassert>

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

// what a peripheral wants handed to it, by type. Anything that needs no
// siblings says nothing and gets needs<>
template <typename... Ts> struct needs {};

template <typename T> struct needs_of {
  using type = needs<>;
};

template <typename T>
  requires requires { typename T::needs; }
struct needs_of<T> {
  using type = typename T::needs;
};

template <HAL::Claiming... Peripherals> struct system_impl {
  template <typename T>
  static constexpr bool declared = (std::same_as<T, Peripherals> || ...);

  // This method ensures only things declared in the system at comptime
  // can be created at runtime. We also pass in the Assembly{} tag/passkey
  // so that any class opting in can include Assembly in their ctor,
  // which prevents said class from being made outside of this method.
  //
  // This is better than friendship because the passkey can be passed along
  // to sub-classes (e.g., DWM making GPIO and SPI), doesn't require explicit
  // friend declarations for every new subscriber, works with
  // emplace/factories/etc. The passkey is transferable!
  template <typename T, typename... Args>
    requires declared<T>
  static T make(Args &&...args) {
    return T{Assembly{}, std::forward<Args>(args)...};
  }

  template <typename T> static constexpr size_t index_of() {
    constexpr std::array<bool, sizeof...(Peripherals)> is{
        std::same_as<T, Peripherals>...};

    for (size_t i = 0; i < is.size(); ++i) {
      if (is[i]) {
        return i;
      }
    }

    return is.size();
  }

  // every peripheral, built once, in pack order. optional lets an immovable
  // device be constructed in place
  class Instances {
  public:
    template <typename T>
      requires declared<T>
    T &get() {
      return *std::get<std::optional<T>>(store_);
    }

  private:
    std::tuple<std::optional<Peripherals>...> store_;

    friend struct system_impl;
  };

  static Instances &take() {
    static Instances instances;
    static bool taken = false;

    assert(!taken && "peripherals taken twice");
    taken = true;

    (emplace<Peripherals>(instances), ...);

    return instances;
  }

private:
  template <typename T, typename... Needs>
  static void emplace_needing(Instances &into, needs<Needs...>) {
    static_assert(((index_of<Needs>() < index_of<T>()) && ...),
                  "a peripheral must be listed after the peripherals it needs");

    std::get<std::optional<T>>(into.store_)
        .emplace(Assembly{}, into.template get<Needs>()...);
  }

  template <typename T> static void emplace(Instances &into) {
    emplace_needing<T>(into, typename needs_of<T>::type{});
  }
};

template <HAL::Claiming... Peripherals>
  requires no_resource_conflicts<Peripherals...>
using system = system_impl<Peripherals...>;

template <Drive::Style S, typename Driver, HAL::Claiming... Peripherals>
  requires drives<Driver, S> && no_resource_conflicts<Driver, Peripherals...>
struct chassis : system_impl<Driver, Peripherals...> {
  static constexpr Drive::Style style = S;

  using driver_t = Driver;
  using frame_t = Drive::Frame<S>;
};
} // namespace Swarm

#endif
