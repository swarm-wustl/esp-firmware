#include "unity.h"
#include "watchdog.h"

#include <optional>

namespace {
constexpr Watchdog::Policy<int> kPolicy{
    .safe = 0, .timeout_ms = 100, .period_ms = 20};

constexpr Watchdog::Feed<int> kCommanded{.value = 7, .silent_ms = 0};

constexpr auto silence(Watchdog::Feed<int> feed, int periods) {
  for (int i = 0; i < periods; ++i) {
    feed = Watchdog::observe(feed, std::optional<int>{}, kPolicy);
  }

  return feed;
}
} // namespace

TEST_CASE("watchdog: a value stands until the timeout", "[watchdog]") {
  TEST_ASSERT_EQUAL_INT(7, silence(kCommanded, 4).value);
}

TEST_CASE("watchdog: silence for the timeout falls back to safe",
          "[watchdog]") {
  TEST_ASSERT_EQUAL_INT(0, silence(kCommanded, 5).value);
}

TEST_CASE("watchdog: a fresh value restarts the timeout", "[watchdog]") {
  const auto fed =
      Watchdog::observe(silence(kCommanded, 4), std::optional{9}, kPolicy);

  TEST_ASSERT_EQUAL_INT(9, fed.value);
  TEST_ASSERT_EQUAL_UINT32(0, fed.silent_ms);
  TEST_ASSERT_EQUAL_INT(9, silence(fed, 4).value);
}

TEST_CASE("watchdog: a fresh value recovers from expiry", "[watchdog]") {
  const auto fed =
      Watchdog::observe(silence(kCommanded, 10), std::optional{9}, kPolicy);

  TEST_ASSERT_EQUAL_INT(9, fed.value);
}

TEST_CASE("watchdog: the silence counter wrapping stays safe", "[watchdog]") {
  const auto wrapped = Watchdog::observe(
      Watchdog::Feed<int>{.value = 0, .silent_ms = UINT32_MAX - 5},
      std::optional<int>{}, kPolicy);

  TEST_ASSERT_TRUE(wrapped.silent_ms < kPolicy.timeout_ms);
  TEST_ASSERT_EQUAL_INT(0, wrapped.value);
}
