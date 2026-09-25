#include "differential_drive.h"
#include "drive.h"
#include "resources.h"
#include "unity.h"

namespace {
using HAL::Claim;
using HAL::Resource;
using HAL::Use;

constexpr std::array kDistinct{
    Claim{Resource::Gpio, 16, Use::Exclusive},
    Claim{Resource::Gpio, 17, Use::Exclusive},
};

constexpr std::array kExclusiveTwice{
    Claim{Resource::Gpio, 16, Use::Exclusive},
    Claim{Resource::Gpio, 16, Use::Exclusive},
};

constexpr std::array kSharedTwice{
    Claim{Resource::Gpio, 0, Use::Shared},
    Claim{Resource::Gpio, 0, Use::Shared},
};

constexpr std::array kMixed{
    Claim{Resource::Gpio, 0, Use::Shared},
    Claim{Resource::Gpio, 0, Use::Exclusive},
};

// a GPIO and a PWM channel that happen to share a number are unrelated
constexpr std::array kSameIdDifferentKind{
    Claim{Resource::Gpio, 1, Use::Exclusive},
    Claim{Resource::PwmChannel, 1, Use::Exclusive},
};

struct GoodDeclaration {
  static constexpr std::array claims{Claim{Resource::Gpio, 5, Use::Exclusive}};
};

struct ClaimsNothing {
  static constexpr std::array<Claim, 0> claims{};
};

struct NotADeclaration {};

struct WrongElementType {
  static constexpr std::array claims{1, 2, 3};
};

constexpr std::array kDifferentialRoster{Motor::Name::LEFT,
                                         Motor::Name::RIGHT};
constexpr std::array kMissingRight{Motor::Name::LEFT, Motor::Name::UPPER_LEFT};
constexpr std::array kExtraMotor{Motor::Name::LEFT, Motor::Name::RIGHT,
                                 Motor::Name::UPPER_LEFT};
constexpr std::array kDuplicated{Motor::Name::LEFT, Motor::Name::LEFT};
} // namespace

TEST_CASE("declarations must say what hardware they take", "[resources]") {
  static_assert(HAL::Claiming<GoodDeclaration>);
  static_assert(HAL::Claiming<ClaimsNothing>);
  static_assert(!HAL::Claiming<NotADeclaration>);
  static_assert(!HAL::Claiming<WrongElementType>);
  TEST_ASSERT_TRUE(true);
}

TEST_CASE("claims: distinct resources never conflict", "[resources]") {
  static_assert(HAL::no_conflicts(kDistinct));
  static_assert(HAL::no_conflicts(kSameIdDifferentKind));
  TEST_ASSERT_TRUE(true);
}

TEST_CASE("claims: two exclusive claims on one resource conflict",
          "[resources]") {
  static_assert(!HAL::no_conflicts(kExclusiveTwice));
  TEST_ASSERT_TRUE(true);
}

TEST_CASE("claims: shared claims coexist, mixing with exclusive does not",
          "[resources]") {
  static_assert(HAL::no_conflicts(kSharedTwice));
  static_assert(!HAL::no_conflicts(kMixed));
  TEST_ASSERT_TRUE(true);
}

TEST_CASE("roster: names must be unique", "[resources]") {
  static_assert(HAL::unique_names(kDifferentialRoster));
  static_assert(!HAL::unique_names(kDuplicated));
  TEST_ASSERT_TRUE(true);
}

TEST_CASE("roster: names exactly the motors the style commands", "[resources]") {
  static_assert(Drive::covers<kDifferentialRoster, Drive::Style::DIFFERENTIAL>());
  static_assert(!Drive::covers<kMissingRight, Drive::Style::DIFFERENTIAL>());
  static_assert(!Drive::covers<kExtraMotor, Drive::Style::DIFFERENTIAL>());
  static_assert(!Drive::covers<kDuplicated, Drive::Style::DIFFERENTIAL>());
  TEST_ASSERT_TRUE(true);
}
