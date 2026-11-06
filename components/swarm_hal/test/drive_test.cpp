#include "differential_drive.h"
#include "drive.h"
#include "unity.h"

#include <algorithm>
#include <type_traits>

namespace {
constexpr auto S = Drive::Style::DIFFERENTIAL;

constexpr auto kControl = Drive::control({.rated = 6.0, .rail = 12.0}, 10.0,
                                         0.02, Pid::gains(0.1, 1.0, 0.0));

constexpr double kLimit = kControl.supply.duty_limit();

static_assert(kLimit == 0.5);

constexpr Drive::Regulators<S> kRest{};

constexpr auto targets(const Drive::Twist &twist) {
  return Drive::inverse_kinematics<S>(twist);
}

// the wheels running exactly as asked, so only the feedforward is left
constexpr auto on_target(const Drive::Twist &twist) {
  return Drive::regulate<S, kControl>(kRest, targets(twist), targets(twist));
}

constexpr Drive::WheelVelocities<S> kStill{{{Motor::Name::LEFT, 0.0},
                                            {Motor::Name::RIGHT, 0.0}}};

constexpr auto kFrame = on_target(Drive::Twist{1.0, 0.0, 0.0}).frame;

static_assert(Drive::motor_count<S> == 2);

// FreeRTOS queues memcpy their items, so a frame has to survive that
static_assert(std::is_trivially_copyable_v<Drive::Frame<S>>);
static_assert(std::is_trivially_copyable_v<Drive::WheelVelocities<S>>);
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

TEST_CASE("differential: on target, duty is speed over top speed", "[drive]") {
  const auto frame = on_target(Drive::Twist{0.2, 0.0, 0.0}).frame;
  const double expected =
      0.2 / Drive::kWheelRadius / kControl.max_wheel_speed * kLimit;

  check(command_for(frame, Motor::Name::LEFT), Motor::Direction::FORWARD,
        expected);
  check(command_for(frame, Motor::Name::RIGHT), Motor::Direction::FORWARD,
        expected);
}

TEST_CASE("differential: negative linear reverses both wheels", "[drive]") {
  const auto frame = on_target(Drive::Twist{-0.2, 0.0, 0.0}).frame;
  const double expected =
      0.2 / Drive::kWheelRadius / kControl.max_wheel_speed * kLimit;

  check(command_for(frame, Motor::Name::LEFT), Motor::Direction::REVERSE,
        expected);
  check(command_for(frame, Motor::Name::RIGHT), Motor::Direction::REVERSE,
        expected);
}

TEST_CASE("differential: pure rotation counter-rotates the wheels", "[drive]") {
  const auto frame = on_target(Drive::Twist{0.0, 0.0, 1.0}).frame;
  const double expected =
      Drive::kHalfTrack / Drive::kWheelRadius / kControl.max_wheel_speed * kLimit;

  check(command_for(frame, Motor::Name::LEFT), Motor::Direction::REVERSE,
        expected);
  check(command_for(frame, Motor::Name::RIGHT), Motor::Direction::FORWARD,
        expected);
}

TEST_CASE("differential: zero twist stops both wheels and clears the integral",
          "[drive]") {
  const Drive::Regulators<S> wound{{{.integral = 3.0, .measured = 0.0},
                                    {.integral = -3.0, .measured = 0.0}}};

  const auto [regulators, frame] = Drive::regulate<S, kControl>(
      wound, targets(Drive::Twist{0.0, 0.0, 0.0}), kStill);

  check(command_for(frame, Motor::Name::LEFT), Motor::Direction::STOP, 0.0);
  check(command_for(frame, Motor::Name::RIGHT), Motor::Direction::STOP, 0.0);

  for (const Pid::State &state : regulators) {
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.0, state.integral);
  }
}

// scaling all wheels by the same divisor keeps the turn radius; clamping each
// one alone would straighten the arc out
TEST_CASE("differential: saturation preserves the wheel ratio", "[drive]") {
  const auto capped = Drive::desaturate(targets(Drive::Twist{10.0, 0.0, 10.0}),
                                        kControl.max_wheel_speed);

  const auto frame =
      Drive::regulate<S, kControl>(kRest, targets(Drive::Twist{10.0, 0.0, 10.0}),
                                   capped)
          .frame;

  const auto left = command_for(frame, Motor::Name::LEFT);
  const auto right = command_for(frame, Motor::Name::RIGHT);

  check(right, Motor::Direction::FORWARD, kLimit);
  check(left, Motor::Direction::FORWARD,
        kLimit * (1.0 - Drive::kHalfTrack) / (1.0 + Drive::kHalfTrack));
}

TEST_CASE("differential: a slow wheel gets more than the feedforward",
          "[drive]") {
  const auto twist = Drive::Twist{0.2, 0.0, 0.0};
  const auto [regulators, frame] =
      Drive::regulate<S, kControl>(kRest, targets(twist), kStill);

  const double feedforward =
      0.2 / Drive::kWheelRadius / kControl.max_wheel_speed * kLimit;

  for (const Motor::Command &cmd : frame.commands) {
    TEST_ASSERT_TRUE(cmd.pwm_ratio > feedforward);
  }

  for (const Pid::State &state : regulators) {
    TEST_ASSERT_TRUE(state.integral > 0.0);
  }
}

TEST_CASE("differential: measurements are matched by name, not position",
          "[drive]") {
  const auto twist = Drive::Twist{0.0, 0.0, 1.0};
  const auto wanted = targets(twist);

  const Drive::WheelVelocities<S> swapped{{wanted[1], wanted[0]}};

  const auto frame =
      Drive::regulate<S, kControl>(kRest, wanted, swapped).frame;
  const double expected =
      Drive::kHalfTrack / Drive::kWheelRadius / kControl.max_wheel_speed * kLimit;

  check(command_for(frame, Motor::Name::LEFT), Motor::Direction::REVERSE,
        expected);
  check(command_for(frame, Motor::Name::RIGHT), Motor::Direction::FORWARD,
        expected);
}

TEST_CASE("differential: a stalled wheel never gets more than its rated "
          "voltage",
          "[drive]") {
  const Drive::Regulators<S> wound{{{.integral = 100.0, .measured = 0.0},
                                    {.integral = 100.0, .measured = 0.0}}};

  const auto frame =
      Drive::regulate<S, kControl>(wound, targets(Drive::Twist{1.0, 0.0, 0.0}),
                                   kStill)
          .frame;

  for (const Motor::Command &cmd : frame.commands) {
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, kLimit, cmd.pwm_ratio);
  }
}

TEST_CASE("differential: linear_y does not move a nonholonomic base",
          "[drive]") {
  const auto frame = on_target(Drive::Twist{0.0, 1.0, 0.0}).frame;

  check(command_for(frame, Motor::Name::LEFT), Motor::Direction::STOP, 0.0);
  check(command_for(frame, Motor::Name::RIGHT), Motor::Direction::STOP, 0.0);
}
