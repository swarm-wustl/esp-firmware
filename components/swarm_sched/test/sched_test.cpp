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

TEST_CASE("sched: on_start runs before the first pipeline", "[sched]") {
  static Sched::signal raised;
  static std::atomic<int> order{0};
  static std::atomic<int> started_at{0};
  static std::atomic<int> ran_at{0};

  static std::atomic<int> calls{0};

  static auto worker = Sched::make_task<4096, configMAX_PRIORITIES - 2>(
      Sched::on_start([] {
        ++calls;
        started_at = ++order;
        return true;
      }) |
      Sched::on(raised) | Sched::to([] { ran_at = ++order; }));

  raised.raise();
  worker.spawn("sched_start");
  pause_ms(kPeriodMs);

  TEST_ASSERT_EQUAL_INT(1, calls.load());
  TEST_ASSERT_EQUAL_INT(1, started_at.load());
  TEST_ASSERT_EQUAL_INT(2, ran_at.load());
}

TEST_CASE("sched: a failed on_start abandons the task", "[sched]") {
  static std::atomic<int> second_setup{0};
  static std::atomic<int> ticks{0};

  static auto worker = Sched::make_task<4096, configMAX_PRIORITIES - 2>(
      Sched::on_start([] { return false; }) | Sched::every<kPeriodMs> |
          Sched::to([] { ++ticks; }),
      Sched::on_start([] {
        ++second_setup;
        return true;
      }) | Sched::every<kPeriodMs> |
          Sched::to([] { ++ticks; }));

  worker.spawn("sched_abandon");
  pause_ms(3 * kPeriodMs);

  TEST_ASSERT_EQUAL_INT(0, second_setup.load());
  TEST_ASSERT_EQUAL_INT(0, ticks.load());
}

TEST_CASE("sched: on_start takes an expected<void, E>", "[sched]") {
  auto ok = Sched::on_start([]() -> std::expected<void, int> { return {}; }) |
            Sched::every<kPeriodMs> | Sched::to([] {});
  auto bad = Sched::on_start([]() -> std::expected<void, int> {
               return std::unexpected{1};
             }) |
             Sched::every<kPeriodMs> | Sched::to([] {});

  TEST_ASSERT_TRUE(ok.start());
  TEST_ASSERT_FALSE(bad.start());
}

TEST_CASE("sched: a pipeline with no on_start starts", "[sched]") {
  auto pipe = Sched::every<kPeriodMs> | Sched::to([] {});

  TEST_ASSERT_TRUE(pipe.start());
}
