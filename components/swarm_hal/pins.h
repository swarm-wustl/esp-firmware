#ifndef PINS_H
#define PINS_H

#include <array>
#include <cstddef>
#include <string_view>
#include <utility>

#include "arrays.h"
#include "fixed_string.h"
#include "resources.h"

namespace Swarm::detail {
struct mint;
}

namespace HAL {
using Strings::fixed_string;

inline constexpr size_t kMaxLabel = 16;

// padded to kMaxLabel when stored in a NamedPin so every entry of a PinSet has
// the same type
using Label = fixed_string<kMaxLabel>;

// the only way to name a pin. Its constructor is private to PinSet, whose
// lookup is private to Swarm's mint, which only reads the set a declaration
// publishes as `pins` or `pin_rows` and only for the holder of its key -- so
// every Pin in the program is one a system's conflict check has seen
class Pin {
public:
  constexpr int number() const { return number_; }

private:
  constexpr explicit Pin(int number) : number_{number} {}

  int number_;

  template <size_t N> friend struct PinSet;
};

struct NamedPin {
  Label label;
  int number;
  Use use = Use::Exclusive;
};

template <size_t N> struct PinSet {
  std::array<NamedPin, N> entries;

  consteval auto claims() const {
    std::array<Claim, N> out{};

    for (size_t i = 0; i < N; ++i) {
      out[i] = Claim{Resource::Gpio, entries[i].number, entries[i].use};
    }

    return out;
  }

private:
  template <size_t M> consteval Pin operator[](fixed_string<M> label) const {
    for (const NamedPin &entry : entries) {
      if (entry.label.view() == label.view()) {
        return Pin{entry.number};
      }
    }

    // an unmatched pin will trigger a compile error
    std::unreachable();
  }

  friend struct Swarm::detail::mint;
};

template <std::same_as<NamedPin>... Entries>
consteval auto pins(Entries... es) {
  const std::array<Label, sizeof...(Entries)> labels{es.label...};

  // a repeated label would make the lookup silently resolve to whichever
  // entry came first while still claiming both pins
  if (Arrays::distinct(labels)) {
    return PinSet<sizeof...(Entries)>{{es...}};
  }

  std::unreachable();
}

template <typename T>
concept PublishesPins = requires {
  { T::pins.claims() } -> ClaimList;
};

// a table of rows, each with its own pins() -- a motor's in_a/in_b/enable, an
// encoder's a/b
template <typename T>
concept PublishesPinRows = requires {
  { T::pin_rows[0].pins().claims() } -> ClaimList;
};

// anything the system config is handed must say what hardware it takes, even
// if that is nothing
template <typename T>
concept Claiming =
    ClaimsDeclared<T> || PublishesPins<T> || PublishesPinRows<T>;

template <typename T> consteval auto pin_claims() {
  if constexpr (PublishesPins<T>) {
    return T::pins.claims();
  } else if constexpr (PublishesPinRows<T>) {
    return []<size_t... I>(std::index_sequence<I...>) {
      return Arrays::concat(std::array<Claim, 0>{},
                            T::pin_rows[I].pins().claims()...);
    }(std::make_index_sequence<T::pin_rows.size()>{});
  } else {
    return std::array<Claim, 0>{};
  }
}
} // namespace HAL

#endif
