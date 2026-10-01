#include "unity.h"

#include <array>

#include "differential_drive.h"
#include "encoder.h"
#include "encoders.h"
#include "system.h"

namespace {
constexpr Encoder::Spec kX4{Encoder::fit0485, Encoder::Decoding::X4};
constexpr Encoder::Spec kX2{Encoder::fit0485, Encoder::Decoding::X2};

struct MockCounter {
  static constexpr auto decoding = Encoder::Decoding::X4;
  static constexpr std::array<HAL::Claim, 0> claims{};

  // the bank owns its counter, so the test reaches the mock's state through
  // the type rather than the instance
  static inline std::array<int32_t, 8> counts{};
  static inline std::array<int, 8> configured{};

  explicit MockCounter(Swarm::Assembly) {}

  void configure_unit(int unit, HAL::Pin a, HAL::Pin b) {
    configured[unit] = a.number() * 100 + b.number();
  }

  int32_t count(int unit) { return counts[unit]; }
  void clear(int unit) { counts[unit] = 0; }

  // configuration happens once, when the bank is built, and Unity may run any
  // case first -- so only the counts are clearable
  static void reset() { counts = {}; }
};

constexpr auto kTable = Encoders::for_style<Drive::Style::DIFFERENTIAL>(
    Encoders::Row{Motor::Name::LEFT, 35, 39, 0},
    Encoders::Row{Motor::Name::RIGHT, 36, 15, 1});

using TestBank = Encoders::Bank<MockCounter, kTable, Encoder::fit0485>;
using TestSystem = Swarm::system<TestBank>;

static_assert(Drive::covered_by<TestBank::motors, Drive::Style::DIFFERENTIAL>);
static_assert(TestBank::spec.counts_per_rev() == 5880);
static_assert(HAL::no_conflicts(TestBank::claims));

TestBank &bank() {
  static TestBank &instance = TestSystem::take().get<TestBank>();

  return instance;
}
} // namespace

TEST_CASE("FIT0485 counts per revolution follow the decoding", "[encoder]") {
  TEST_ASSERT_EQUAL_UINT32(2940, kX2.counts_per_rev());
  TEST_ASSERT_EQUAL_UINT32(5880, kX4.counts_per_rev());
  TEST_ASSERT_EQUAL_UINT32(
      1470, (Encoder::Spec{Encoder::fit0485, Encoder::Decoding::X1})
                .counts_per_rev());
}

TEST_CASE("a full output revolution is one revolution of travel", "[encoder]") {
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, Encoder::revolutions(kX4, 5880));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, -0.5, Encoder::revolutions(kX4, -2940));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, Encoder::revolutions(kX4, 0));
}

TEST_CASE("radians and revolutions agree", "[encoder]") {
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 2.0 * std::numbers::pi,
                            Encoder::radians(kX4, 5880));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, std::numbers::pi,
                            Encoder::radians(kX2, 1470));
}

TEST_CASE("velocity divides travel by the interval", "[encoder]") {
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 2.0, Encoder::rev_per_second(kX4, 5880, 0.5));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 120.0, Encoder::rpm(kX4, 5880, 0.5));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 4.0 * std::numbers::pi,
                            Encoder::rad_per_second(kX4, 5880, 0.5));
}

TEST_CASE("a zero interval yields no velocity rather than infinity",
          "[encoder]") {
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, Encoder::rev_per_second(kX4, 5880, 0.0));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, Encoder::rpm(kX4, 5880, -1.0));
}

TEST_CASE("a tracker reports the delta between successive counts",
          "[encoder]") {
  Encoder::Tracker tracker;

  TEST_ASSERT_EQUAL_INT32(100, tracker.advance(100));
  TEST_ASSERT_EQUAL_INT32(50, tracker.advance(150));
  TEST_ASSERT_EQUAL_INT32(0, tracker.advance(150));
  TEST_ASSERT_EQUAL_INT32(-150, tracker.advance(0));
  TEST_ASSERT_EQUAL_INT32(0, tracker.last());
}

TEST_CASE("a tracker survives counter wraparound", "[encoder]") {
  Encoder::Tracker tracker;

  TEST_ASSERT_EQUAL_INT32(INT32_MAX, tracker.advance(INT32_MAX));
  // one more count past the top wraps to INT32_MIN, which is still +1 of travel
  TEST_ASSERT_EQUAL_INT32(1, tracker.advance(INT32_MIN));
}

TEST_CASE("a bank configures one unit per wheel", "[encoder]") {
  MockCounter::reset();
  bank();

  TEST_ASSERT_EQUAL_INT(35 * 100 + 39, MockCounter::configured[0]);
  TEST_ASSERT_EQUAL_INT(36 * 100 + 15, MockCounter::configured[1]);
}

TEST_CASE("a bank reads cumulative counts per named motor", "[encoder]") {
  MockCounter::reset();

  MockCounter::counts[0] = 5880;
  MockCounter::counts[1] = -2940;

  const auto readings = bank().read();

  TEST_ASSERT_EQUAL(Motor::Name::LEFT, readings[0].name);
  TEST_ASSERT_EQUAL_INT32(5880, readings[0].counts);
  TEST_ASSERT_EQUAL(Motor::Name::RIGHT, readings[1].name);
  TEST_ASSERT_EQUAL_INT32(-2940, readings[1].counts);
}

TEST_CASE("advance reports travel since the previous call", "[encoder]") {
  MockCounter::reset();
  bank().reset();

  MockCounter::counts[0] = 1000;

  TEST_ASSERT_EQUAL_INT32(1000, bank().advance()[0].counts);

  MockCounter::counts[0] = 1500;

  TEST_ASSERT_EQUAL_INT32(500, bank().advance()[0].counts);
  TEST_ASSERT_EQUAL_INT32(0, bank().advance()[0].counts);
}

TEST_CASE("reset zeroes the hardware and the tracker together", "[encoder]") {
  MockCounter::reset();

  MockCounter::counts[0] = 4000;

  (void)bank().advance();

  bank().reset();

  TEST_ASSERT_EQUAL_INT32(0, MockCounter::counts[0]);

  MockCounter::counts[0] = 60;

  TEST_ASSERT_EQUAL_INT32(60, bank().advance()[0].counts);
}
