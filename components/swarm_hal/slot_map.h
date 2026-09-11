// Written with Claude
#ifndef SLOT_MAP_H
#define SLOT_MAP_H

#include <algorithm>
#include <array>
#include <cstddef>

namespace HAL {
namespace detail {
template <auto From, auto To> constexpr auto raw_slot_map() {
  std::array<size_t, To.size()> found{};

  for (size_t i = 0; i < To.size(); ++i) {
    auto it = std::ranges::find(From, To[i]);
    found[i] = static_cast<size_t>(it - From.begin());
  }

  return found;
}

template <auto From, auto To> constexpr bool is_bijection() {
  if (From.size() != To.size()) {
    return false;
  }

  std::array<size_t, To.size()> sorted = raw_slot_map<From, To>();
  std::ranges::sort(sorted);

  for (size_t i = 0; i < sorted.size(); ++i) {
    if (sorted[i] != i) {
      return false;
    }
  }

  return true;
}
} // namespace detail

template <auto From, auto To>
constexpr std::array<size_t, To.size()> slot_map() {
  static_assert(From.size() == To.size(), "the two lists disagree on length");
  static_assert(detail::is_bijection<From, To>(),
                "the two lists must name the same set, each entry exactly "
                "once");

  return detail::raw_slot_map<From, To>();
}
} // namespace HAL

#endif
