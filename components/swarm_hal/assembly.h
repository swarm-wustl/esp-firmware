#ifndef ASSEMBLY_H
#define ASSEMBLY_H

#include <array>
#include <concepts>
#include <utility>

#include "arrays.h"
#include "fixed_string.h"
#include "pins.h"
#include "resources.h"

namespace Swarm {
// the sub-devices a peripheral builds for itself. Listing one is what lets the
// owner construct it, and what folds its claims into the owner's, so the two
// can't drift apart
template <HAL::Claiming... Ts> struct owns {};

template <typename T> struct owns_of {
  using type = owns<>;
};

template <typename T>
  requires requires { typename T::owns; }
struct owns_of<T> {
  using type = typename T::owns;
};

template <typename Sub, typename List> constexpr bool listed = false;

template <typename Sub, typename... Ts>
constexpr bool listed<Sub, owns<Ts...>> = (std::same_as<Sub, Ts> || ...);

template <typename Sub, typename Owner>
concept owned_by = listed<Sub, typename owns_of<Owner>::type>;

template <HAL::Claiming T> consteval auto declared_claims() {
  if constexpr (HAL::ClaimsDeclared<T>) {
    return T::claims;
  } else {
    return std::array<HAL::Claim, 0>{};
  }
}

template <HAL::Claiming T> consteval auto tree_claims() {
  return []<typename... Subs>(owns<Subs...>) {
    return Arrays::concat(declared_claims<T>(), HAL::pin_claims<T>(),
                          tree_claims<Subs>()...);
  }(typename owns_of<T>::type{});
}

template <HAL::Claiming... Decls> consteval auto claims_of() {
  return Arrays::concat(std::array<HAL::Claim, 0>{}, tree_claims<Decls>()...);
}

template <typename... Decls>
concept no_resource_conflicts = HAL::no_conflicts(claims_of<Decls...>());

// the conflict check lives on the friend itself, so no route to an Assembly
// skips it
template <HAL::Claiming... Peripherals>
  requires no_resource_conflicts<Peripherals...>
struct system_impl;

template <typename T> class Assembly;

template <typename Sub, typename Owner, typename... Args>
  requires owned_by<Sub, Owner>
Sub make(Assembly<Owner>, Args &&...args) {
  return Sub{Assembly<Sub>{}, std::forward<Args>(args)...};
}

// the key to constructing one T. Only a system that declared T can mint one,
// or the key of a T's owner -- so nothing that owns a pin can exist outside a
// config that declared it, and a key can't be handed down to a sub-device the
// owner never listed
template <typename T> class Assembly {
  Assembly() = default;

  template <typename Sub, typename Owner, typename... Args>
    requires owned_by<Sub, Owner>
  friend Sub make(Assembly<Owner>, Args &&...args);

  template <HAL::Claiming... Peripherals>
    requires no_resource_conflicts<Peripherals...>
  friend struct system_impl;
};

namespace detail {
// the only reader of a PinSet. The lookup is a constant whatever the key, so
// a pin costs what the bare number did; the key only decides who may ask
struct mint {
  template <Strings::fixed_string L, HAL::PublishesPins T>
  static constexpr HAL::Pin pin(Assembly<T>) {
    constexpr HAL::Pin found = T::pins[L];

    return found;
  }

  template <Strings::fixed_string L, HAL::PublishesPinRows T>
  static constexpr const auto &column(Assembly<T>) {
    static constexpr auto found =
        []<size_t... I>(std::index_sequence<I...>) {
          return std::array<HAL::Pin, sizeof...(I)>{
              T::pin_rows[I].pins()[L]...};
        }(std::make_index_sequence<T::pin_rows.size()>{});

    return found;
  }
};
} // namespace detail

template <Strings::fixed_string L, HAL::PublishesPins T>
constexpr HAL::Pin pin(Assembly<T> key) {
  return detail::mint::pin<L>(key);
}

// one pin per row, in row order, so a slot indexes it like it indexes the
// table
template <Strings::fixed_string L, HAL::PublishesPinRows T>
constexpr const auto &pin_column(Assembly<T> key) {
  return detail::mint::column<L>(key);
}
} // namespace Swarm

#endif
