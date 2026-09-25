#include "differential_drive.h"
#include "drive.h"
#include "unity.h"

#include <type_traits>

namespace {
constexpr auto kFrame = Drive::inverse_kinematics<Drive::Style::DIFFERENTIAL>(
    Drive::Twist{1.0, 0.0, 0.0});

static_assert(Drive::motor_count<Drive::Style::DIFFERENTIAL> == 2);

// the consumer queue memcpys its items, so a frame has to survive that
static_assert(std::is_trivially_copyable_v<Drive::Frame<Drive::Style::DIFFERENTIAL>>);
static_assert(std::is_trivially_copyable_v<Motor::Command>);
static_assert(kFrame.commands[0].name == Motor::Name::LEFT);
static_assert(kFrame.commands[0].dir == Motor::Direction::FORWARD);

constexpr Motor::Command command_for(const auto &frame, Motor::Name name) {
  for (const Motor::Command &cmd : frame.commands) {
    if (cmd.name == name) {
      return cmd;
    }
  }

  return {name, Motor::Direction::STOP, -1.0};
}

void check(const Motor::Command &cmd, Motor::Direction dir, double ratio) {
  TEST_ASSERT_EQUAL_INT(static_cast<int>(dir), static_cast<int>(cmd.dir));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, ratio, cmd.pwm_ratio);
}
} // namespace

TEST_CASE("differential: forward twist drives both wheels forward", "[drive]") {
  const auto frame = Drive::inverse_kinematics<Drive::Style::DIFFERENTIAL>(
      Drive::Twist{1.0, 0.0, 0.0});

  check(command_for(frame, Motor::Name::LEFT), Motor::Direction::FORWARD, 1.0);
  check(command_for(frame, Motor::Name::RIGHT), Motor::Direction::FORWARD, 1.0);
}

TEST_CASE("differential: negative linear reverses both wheels", "[drive]") {
  const auto frame = Drive::inverse_kinematics<Drive::Style::DIFFERENTIAL>(
      Drive::Twist{-0.5, 0.0, 0.0});

  check(command_for(frame, Motor::Name::LEFT), Motor::Direction::REVERSE, 0.5);
  check(command_for(frame, Motor::Name::RIGHT), Motor::Direction::REVERSE, 0.5);
}

TEST_CASE("differential: pure rotation counter-rotates the wheels", "[drive]") {
  const auto frame = Drive::inverse_kinematics<Drive::Style::DIFFERENTIAL>(
      Drive::Twist{0.0, 0.0, 1.0});

  check(command_for(frame, Motor::Name::LEFT), Motor::Direction::REVERSE,
        Drive::kDifferentialHalfTrack);
  check(command_for(frame, Motor::Name::RIGHT), Motor::Direction::FORWARD,
        Drive::kDifferentialHalfTrack);
}

TEST_CASE("differential: zero twist stops both wheels", "[drive]") {
  const auto frame = Drive::inverse_kinematics<Drive::Style::DIFFERENTIAL>(
      Drive::Twist{0.0, 0.0, 0.0});

  check(command_for(frame, Motor::Name::LEFT), Motor::Direction::STOP, 0.0);
  check(command_for(frame, Motor::Name::RIGHT), Motor::Direction::STOP, 0.0);
}

// scaling all wheels by the same divisor keeps the turn radius; clamping each
// one alone would straighten the arc out
TEST_CASE("differential: saturation preserves the wheel ratio", "[drive]") {
  const auto frame = Drive::inverse_kinematics<Drive::Style::DIFFERENTIAL>(
      Drive::Twist{1.0, 0.0, 1.0});

  const auto left = command_for(frame, Motor::Name::LEFT);
  const auto right = command_for(frame, Motor::Name::RIGHT);

  check(right, Motor::Direction::FORWARD, 1.0);
  check(left, Motor::Direction::FORWARD, (1.0 - 0.5) / (1.0 + 0.5));
  TEST_ASSERT_TRUE(left.pwm_ratio <= 1.0 && right.pwm_ratio <= 1.0);
}

TEST_CASE("differential: linear_y does not move a nonholonomic base",
          "[drive]") {
  const auto frame = Drive::inverse_kinematics<Drive::Style::DIFFERENTIAL>(
      Drive::Twist{0.0, 1.0, 0.0});

  check(command_for(frame, Motor::Name::LEFT), Motor::Direction::STOP, 0.0);
  check(command_for(frame, Motor::Name::RIGHT), Motor::Direction::STOP, 0.0);
}
