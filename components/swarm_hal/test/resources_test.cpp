#include "assembly.h"
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

struct Leaf {
  static constexpr std::array claims{Claim{Resource::Gpio, 5, Use::Exclusive}};
  explicit Leaf(Swarm::Assembly<Leaf>) {}
};

struct SharedLeaf {
  static constexpr std::array claims{Claim{Resource::LedcTimer, 0, Use::Shared}};
  explicit SharedLeaf(Swarm::Assembly<SharedLeaf>) {}
};

struct Parent {
  static constexpr std::array claims{Claim{Resource::Gpio, 6, Use::Exclusive}};
  using owns = Swarm::owns<Leaf, SharedLeaf>;
};

struct Grandparent {
  static constexpr std::array<Claim, 0> claims{};
  using owns = Swarm::owns<Parent>;
};

struct AlsoOnFive {
  static constexpr std::array claims{Claim{Resource::Gpio, 5, Use::Exclusive}};
};

struct AlsoSharesTimer {
  static constexpr std::array<Claim, 0> claims{};
  using owns = Swarm::owns<SharedLeaf>;
};

template <typename Sub, typename Owner>
concept makes = requires(Swarm::Assembly<Owner> key) { Swarm::make<Sub>(key); };

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

TEST_CASE("claims: an owner carries what it owns, all the way down",
          "[resources]") {
  static_assert(Swarm::claims_of<Leaf>().size() == 1);
  static_assert(Swarm::claims_of<Parent>().size() == 3);
  static_assert(Swarm::claims_of<Grandparent>().size() == 3);
  TEST_ASSERT_TRUE(true);
}

TEST_CASE("claims: a sub-device collides like a declared one", "[resources]") {
  static_assert(!Swarm::no_resource_conflicts<Parent, AlsoOnFive>);
  static_assert(!Swarm::no_resource_conflicts<Grandparent, AlsoOnFive>);
  static_assert(Swarm::no_resource_conflicts<Parent, AlsoSharesTimer>);
  TEST_ASSERT_TRUE(true);
}

TEST_CASE("keys: an owner builds only what it lists", "[resources]") {
  static_assert(makes<Leaf, Parent>);
  static_assert(!makes<Leaf, Grandparent>);
  static_assert(!makes<AlsoOnFive, Parent>);
  static_assert(!std::default_initializable<Swarm::Assembly<Leaf>>);
  static_assert(!std::constructible_from<Leaf, Swarm::Assembly<Parent>>);
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
  static_assert(Arrays::distinct(kDifferentialRoster));
  static_assert(!Arrays::distinct(kDuplicated));
  TEST_ASSERT_TRUE(true);
}

TEST_CASE("roster: names exactly the motors the style commands", "[resources]") {
  static_assert(Drive::covers<kDifferentialRoster, Drive::Style::DIFFERENTIAL>());
  static_assert(!Drive::covers<kMissingRight, Drive::Style::DIFFERENTIAL>());
  static_assert(!Drive::covers<kExtraMotor, Drive::Style::DIFFERENTIAL>());
  static_assert(!Drive::covers<kDuplicated, Drive::Style::DIFFERENTIAL>());
  TEST_ASSERT_TRUE(true);
}
