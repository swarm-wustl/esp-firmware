#ifndef PINS_H
#define PINS_H

#include <array>
#include <cstddef>
#include <string_view>
#include <utility>

#include "resources.h"

namespace HAL {
inline constexpr size_t kMaxLabel = 16;

// one string type for both jobs: deduced to its exact length when used as a
// template parameter, padded to kMaxLabel when stored in a NamedPin so every
// entry of a PinSet has the same type
template <size_t N = kMaxLabel> struct fixed_string {
  char value[N]{};

  constexpr fixed_string() = default;

  template <size_t M>
    requires(M <= N)
  consteval fixed_string(const char (&text)[M]) {
    for (size_t i = 0; i < M; ++i) {
      value[i] = text[i];
    }
  }

  constexpr std::string_view view() const { return value; }

  constexpr bool operator==(const fixed_string &) const = default;
};

// template deduction guide since CTAD no longer applies
// this helps the compiler know what N should be given M
template <size_t M> fixed_string(const char (&)[M]) -> fixed_string<M>;

template <fixed_string S> constexpr auto operator""_p() { return S; }

// the only way to name a pin. Its constructor is private to PinSet, so every
// Pin in the program came out of a declaration that also published the claim
class Pin {
public:
  constexpr int number() const { return number_; }

private:
  constexpr explicit Pin(int number) : number_{number} {}

  int number_;

  template <size_t N> friend struct PinSet;
};

struct NamedPin {
  fixed_string<> label;
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
  const std::array<fixed_string<>, sizeof...(Entries)> labels{es.label...};

  // a repeated label would make the lookup silently resolve to whichever
  // entry came first while still claiming both pins
  if (unique_names(labels)) {
    return PinSet<sizeof...(Entries)>{{es...}};
  }

  std::unreachable();
}
} // namespace HAL

#endif
