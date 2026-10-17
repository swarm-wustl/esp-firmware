#ifndef ARRAYS_H
#define ARRAYS_H

#include <array>
#include <concepts>
#include <cstddef>
#include <functional>
#include <type_traits>
#include <utility>

namespace Arrays {
// std::views::transform loses the size N of the array
// So collecting back into an array is impossible
// This helper method circumvents that
// Rust has the same issue which is why array::map exists (outside of
// Iterator::map)
template <typename T, size_t N, std::invocable<const T &> F>
constexpr auto map(const std::array<T, N> &in, F f) {
  using U = std::remove_cvref_t<std::invoke_result_t<F, const T &>>;

  return [&]<size_t... I>(std::index_sequence<I...>) {
    return std::array<U, N>{std::invoke(f, in[I])...};
  }(std::make_index_sequence<N>{});
}

// a device that owns a sub-device owns its claims too: an L298N driver holds a
// PWM, a DW1000 holds an SPI, and the config has to see all of it
template <typename T, size_t... Ns>
constexpr auto concat(const std::array<T, Ns> &...arrays) {
  std::array<T, (0 + ... + Ns)> all{};
  size_t next = 0;

  const auto append = [&all, &next](const auto &items) {
    for (const T &item : items) {
      all[next++] = item;
    }
  };

  (append(arrays), ...);

  return all;
}

template <typename T, size_t N>
constexpr bool distinct(const std::array<T, N> &items) {
  for (size_t i = 0; i < N; ++i) {
    for (size_t j = i + 1; j < N; ++j) {
      if (items[i] == items[j]) {
        return false;
      }
    }
  }

  return true;
}
} // namespace Arrays

#endif
