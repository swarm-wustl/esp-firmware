#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <cstdint>
#include <optional>

namespace Watchdog {
template <typename T> struct Policy {
  T safe;
  uint32_t timeout_ms;
  uint32_t period_ms;
};

template <typename T> struct Feed {
  T value;
  uint32_t silent_ms;
};

// expiry replaces the value rather than masking it, so a counter that wraps
// after 49 days of silence cannot bring the stale value back
template <typename T>
constexpr Feed<T> observe(const Feed<T> &feed, const std::optional<T> &fresh,
                          const Policy<T> &policy) {
  if (fresh) {
    return {*fresh, 0};
  }

  const uint32_t silent = feed.silent_ms + policy.period_ms;

  return {silent >= policy.timeout_ms ? policy.safe : feed.value, silent};
}
} // namespace Watchdog

#endif
