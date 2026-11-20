#ifndef FIXED_STRING_H
#define FIXED_STRING_H

#include <cstddef>
#include <string_view>

namespace Strings {
// one string type for both jobs: deduced to its exact length when used as a
// template parameter, padded to a fixed width when stored so every entry of a
// table has the same type
template <size_t N> struct fixed_string {
  char value[N]{};

  constexpr fixed_string() = default;

  template <size_t M>
    requires(M <= N)
  consteval fixed_string(const char (&text)[M]) {
    for (size_t i = 0; i < M; ++i) {
      value[i] = text[i];
    }
  }

  template <size_t M>
    requires(M < N)
  consteval fixed_string(const fixed_string<M> &other)
      : fixed_string(other.value) {}

  constexpr std::string_view view() const { return value; }

  constexpr operator const char *() const { return value; }

  constexpr bool operator==(const fixed_string &) const = default;
};

// template deduction guide since CTAD no longer applies
// this helps the compiler know what N should be given M
template <size_t M> fixed_string(const char (&)[M]) -> fixed_string<M>;

// sized for both terminators, of which the result keeps one, so a padded
// operand only leaves slack at the end
template <size_t A, size_t B>
consteval fixed_string<A + B - 1> operator+(const fixed_string<A> &a,
                                            const fixed_string<B> &b) {
  fixed_string<A + B - 1> out{};
  size_t i = 0;

  for (const char c : a.view()) {
    out.value[i++] = c;
  }

  for (const char c : b.view()) {
    out.value[i++] = c;
  }

  return out;
}

template <size_t A, size_t B>
consteval auto operator+(const fixed_string<A> &a, const char (&b)[B]) {
  return a + fixed_string<B>{b};
}

template <size_t A, size_t B>
consteval auto operator+(const char (&a)[A], const fixed_string<B> &b) {
  return fixed_string<A>{a} + b;
}

static_assert((fixed_string{"swarm_1"} + "/" + "base_link").view() ==
              "swarm_1/base_link");
static_assert((fixed_string<16>{"cs"} + "!").view() == "cs!");
} // namespace Strings

#endif
