#ifndef PINS_H
#define PINS_H

#include <array>
#include <cstddef>
#include <string_view>
#include <utility>

#include "arrays.h"
#include "fixed_string.h"
#include "resources.h"

namespace HAL {
using Strings::fixed_string;

inline constexpr size_t kMaxLabel = 16;

// padded to kMaxLabel when stored in a NamedPin so every entry of a PinSet has
// the same type
using Label = fixed_string<kMaxLabel>;

template <fixed_string S> constexpr auto operator""_p() { return S; }

// the only way to name a pin. Its constructor is private to PinSet, so every
// Pin in the program came out of a declaration that also published the claim
//
// TODO: that only holds by convention. operator[] works on any PinSet, so a
// free-standing HAL::pins(...) that no declared peripheral publishes still
// mints Pins whose claims never reach a system's conflict check. Close it by
// minting from the key instead: a device publishes `pins` (or `pin_rows` for a
// table), claims_of folds them in, and Swarm::pin<"cs">(Assembly<T>) becomes
// the only friend of operator[]. Devices that need a pin after construction
// keep their key as a [[no_unique_address]] member
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

  template <size_t M> consteval Pin operator[](fixed_string<M> label) const {
    for (const NamedPin &entry : entries) {
      if (entry.label.view() == label.view()) {
        return Pin{entry.number};
      }
    }

    // an unmatched pin will trigger a compile error
    std::unreachable();
  }
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

template <auto Table, fixed_string L> consteval auto pin_column() {
  return [&]<size_t... I>(std::index_sequence<I...>) {
    return std::array<Pin, sizeof...(I)>{Table[I].pins()[L]...};
  }(std::make_index_sequence<Table.size()>{});
}

// a row of a pin table takes its pins plus whatever one channel it names -- a
// PWM channel, a PCNT unit
template <auto Table, typename F> consteval auto row_claims(F channel) {
  return [&]<size_t... I>(std::index_sequence<I...>) {
    return Arrays::concat(Arrays::concat(Table[I].pins().claims(),
                                         std::array{channel(Table[I])})...);
  }(std::make_index_sequence<Table.size()>{});
}
} // namespace HAL

#endif
