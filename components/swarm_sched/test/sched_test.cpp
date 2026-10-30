#include "swarm_sched.h"
#include "unity.h"

#include <atomic>
#include <cstdint>
#include <expected>

namespace {
constexpr uint32_t kPeriodMs = 20;

void pause_ms(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }
} // namespace

TEST_CASE("sched: a raised signal runs the pipeline once", "[sched]") {
  static Sched::signal raised;

  int runs = 0;
  auto pipe = Sched::on(raised) | Sched::to([&runs] { ++runs; });

  TEST_ASSERT_FALSE(pipe.wait(0));

  raised.raise();

  TEST_ASSERT_TRUE(pipe.wait(0));
  pipe.run();

  TEST_ASSERT_EQUAL_INT(1, runs);
  TEST_ASSERT_FALSE(pipe.wait(0));
}

TEST_CASE("sched: edges raised before the wait collapse into one", "[sched]") {
  static Sched::signal raised;

  raised.raise();
  raised.raise();
  raised.raise();

  TEST_ASSERT_TRUE(raised.wait(0));
  TEST_ASSERT_FALSE(raised.wait(0));
}

TEST_CASE("sched: a handler's error counts as a drop", "[sched]") {
  static Sched::signal raised;

  auto pipe = Sched::on(raised) | Sched::to([]() -> std::expected<void, int> {
                return std::unexpected{1};
              });

  pipe.run();

  TEST_ASSERT_EQUAL_UINT32(1, pipe.drops());
}

TEST_CASE("sched: ticks_until survives the counter wrapping", "[sched]") {
  TEST_ASSERT_EQUAL_UINT32(5, Sched::detail::ticks_until(10, 15));
  TEST_ASSERT_EQUAL_UINT32(0, Sched::detail::ticks_until(15, 15));
  TEST_ASSERT_EQUAL_UINT32(0, Sched::detail::ticks_until(20, 15));
  TEST_ASSERT_EQUAL_UINT32(
      4, Sched::detail::ticks_until(static_cast<TickType_t>(-2), 2));
}

TEST_CASE("sched: an interrupt-only task sleeps until raised", "[sched]") {
  static Sched::signal raised;
  static std::atomic<int> runs{0};

  static auto worker = Sched::make_task<4096, configMAX_PRIORITIES - 2>(
      Sched::on(raised) | Sched::to([] { ++runs; }));

  worker.spawn("sched_irq");

  pause_ms(5 * kPeriodMs);
  TEST_ASSERT_EQUAL_INT(0, runs.load());

  raised.raise();
  pause_ms(kPeriodMs);
  TEST_ASSERT_EQUAL_INT(1, runs.load());

  raised.raise();
  pause_ms(kPeriodMs);
  TEST_ASSERT_EQUAL_INT(2, runs.load());
}

TEST_CASE("sched: periods keep running beside an interrupt", "[sched]") {
  static Sched::signal raised;
  static std::atomic<int> ticks{0};
  static std::atomic<int> interrupts{0};

  static auto mixed = Sched::make_task<4096, configMAX_PRIORITIES - 2>(
      Sched::every<kPeriodMs> | Sched::to([] { ++ticks; }),
      Sched::on(raised) | Sched::to([] { ++interrupts; }));

  mixed.spawn("sched_mixed");

  pause_ms(10 * kPeriodMs);

  const int quiet = ticks.load();

  TEST_ASSERT_INT_WITHIN(2, 10, quiet);
  TEST_ASSERT_EQUAL_INT(0, interrupts.load());

  // raised well inside the period, so a task that only noticed the signal at
  // its next tick would show nothing yet
  raised.raise();
  pause_ms(kPeriodMs / 2);
  TEST_ASSERT_EQUAL_INT(1, interrupts.load());

  pause_ms(10 * kPeriodMs);

  TEST_ASSERT_INT_WITHIN(2, 20, ticks.load());
}
