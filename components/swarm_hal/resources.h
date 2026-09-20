// Written with Claude
#ifndef RESOURCES_H
#define RESOURCES_H

#include <algorithm>
#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <ranges>

namespace HAL {
enum class Resource : uint8_t { Gpio, PwmChannel, LedcTimer };

// a pin driven by one peripheral is Exclusive; one deliberately tied together
// -- a shared standby line, the MOSI/SCLK of a bus -- is Shared. Two Exclusive
// claims on the same resource conflict, as does mixing the two
enum class Use : uint8_t { Exclusive, Shared };

struct Claim {
  Resource resource;
  int id;
  Use use;
};

constexpr bool same_resource(const Claim &a, const Claim &b) {
  return a.resource == b.resource && a.id == b.id;
}

template <size_t N>
consteval bool no_conflicts(const std::array<Claim, N> &claims) {
  for (size_t i = 0; i < N; ++i) {
    for (size_t j = i + 1; j < N; ++j) {
      if (!same_resource(claims[i], claims[j])) {
        continue;
      }

      if (claims[i].use == Use::Exclusive || claims[j].use == Use::Exclusive) {
        return false;
      }
    }
  }

  return true;
}

// anything the system config is handed must say what hardware it takes, even
// if that is nothing
template <typename T>
concept Claiming = requires {
  { T::claims } -> std::ranges::range;
  requires std::same_as<std::ranges::range_value_t<decltype(T::claims)>, Claim>;
};

template <Claiming... Decls> consteval auto claims_of() {
  constexpr size_t total = (0 + ... + Decls::claims.size());

  std::array<Claim, total> all{};
  size_t next = 0;

  const auto append = [&all, &next](const auto &claims) {
    for (const Claim &claim : claims) {
      all[next++] = claim;
    }
  };

  (append(Decls::claims), ...);

  return all;
}

template <typename Name, size_t N>
consteval bool unique_names(const std::array<Name, N> &names) {
  for (size_t i = 0; i < N; ++i) {
    for (size_t j = i + 1; j < N; ++j) {
      if (names[i] == names[j]) {
        return false;
      }
    }
  }

  return true;
}
} // namespace HAL

#endif
