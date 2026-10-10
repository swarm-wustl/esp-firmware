#include "motor.h"
#include "unity.h"

#include <array>
#include <type_traits>

namespace {
constexpr auto kPair =
    std::array{Motor::Name::FRONT_UMC_A, Motor::Name::FRONT_UMC_B};

static_assert(std::is_trivially_copyable_v<Motor::Frame<2>>);
static_assert(Motor::ganged<kPair>(1.0).commands.size() == kPair.size());

void check(const Motor::Command &cmd, Motor::Name name, Motor::Direction dir,
           double ratio) {
  TEST_ASSERT_EQUAL_INT(static_cast<int>(name), static_cast<int>(cmd.name));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(dir), static_cast<int>(cmd.dir));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, ratio, cmd.pwm_ratio);
}
} // namespace

TEST_CASE("ganged: a positive effort drives every motor forward", "[motor]") {
  const auto frame = Motor::ganged<kPair>(0.75);

  check(frame.commands[0], Motor::Name::FRONT_UMC_A, Motor::Direction::FORWARD,
        0.75);
  check(frame.commands[1], Motor::Name::FRONT_UMC_B, Motor::Direction::FORWARD,
        0.75);
}

TEST_CASE("ganged: a negative effort reverses every motor", "[motor]") {
  const auto frame = Motor::ganged<kPair>(-0.25);

  check(frame.commands[0], Motor::Name::FRONT_UMC_A, Motor::Direction::REVERSE,
        0.25);
  check(frame.commands[1], Motor::Name::FRONT_UMC_B, Motor::Direction::REVERSE,
        0.25);
}

TEST_CASE("ganged: zero effort stops every motor", "[motor]") {
  const auto frame = Motor::ganged<kPair>(0.0);

  check(frame.commands[0], Motor::Name::FRONT_UMC_A, Motor::Direction::STOP,
        0.0);
  check(frame.commands[1], Motor::Name::FRONT_UMC_B, Motor::Direction::STOP,
        0.0);
}

// a publisher is free to send 5.0; the duty it becomes is not
TEST_CASE("ganged: effort saturates at full duty", "[motor]") {
  const auto high = Motor::ganged<kPair>(5.0);
  const auto low = Motor::ganged<kPair>(-5.0);

  check(high.commands[0], Motor::Name::FRONT_UMC_A, Motor::Direction::FORWARD,
        1.0);
  check(low.commands[0], Motor::Name::FRONT_UMC_A, Motor::Direction::REVERSE,
        1.0);
}
