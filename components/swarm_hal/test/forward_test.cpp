#include "differential_drive.h"
#include "drive.h"
#include "unity.h"

#include <array>

// a holonomic platform no robot uses yet, declared here so the forward
// kinematics is checked against a geometry with all three freedoms
namespace Drive {
inline constexpr double kMecanumRadius = 0.04;
inline constexpr double kMecanumReach = 0.15 + 0.12;

template <> constexpr auto wheels<Style::MECANUM>() {
  constexpr double k = 1.0 / kMecanumRadius;

  return std::array{
      Wheel{Motor::Name::UPPER_LEFT, k, -k, -kMecanumReach * k},
      Wheel{Motor::Name::UPPER_RIGHT, k, k, kMecanumReach * k},
      Wheel{Motor::Name::LOWER_LEFT, k, k, -kMecanumReach * k},
      Wheel{Motor::Name::LOWER_RIGHT, k, -k, kMecanumReach * k},
  };
}
} // namespace Drive

namespace {
constexpr auto D = Drive::Style::DIFFERENTIAL;
constexpr auto M = Drive::Style::MECANUM;

constexpr bool near(double a, double b) {
  return (a > b ? a - b : b - a) < 1e-9;
}

constexpr bool near(const Drive::Twist &a, const Drive::Twist &b) {
  return near(a.linear_x, b.linear_x) && near(a.linear_y, b.linear_y) &&
         near(a.angular_z, b.angular_z);
}

template <Drive::Style S> constexpr auto round_trip(const Drive::Twist &twist) {
  return Drive::forward_kinematics<S>(Drive::inverse_kinematics<S>(twist));
}

// v = R(wl + wr)/2 and w = R(wr - wl)/2b, the textbook differential pair
constexpr auto kDifferential = Drive::contributions<D>();

static_assert(kDifferential[0].name == Motor::Name::LEFT);
static_assert(near(kDifferential[0].twist,
                   {Drive::kWheelRadius / 2.0, 0.0,
                    -Drive::kWheelRadius / (2.0 * Drive::kHalfTrack)}));
static_assert(near(kDifferential[1].twist,
                   {Drive::kWheelRadius / 2.0, 0.0,
                    Drive::kWheelRadius / (2.0 * Drive::kHalfTrack)}));

void check(const Drive::Twist &expected, const Drive::Twist &actual) {
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, expected.linear_x, actual.linear_x);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, expected.linear_y, actual.linear_y);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, expected.angular_z, actual.angular_z);
}
} // namespace

TEST_CASE("forward: differential recovers the twist it was driven with",
          "[drive]") {
  check({0.3, 0.0, -0.8}, round_trip<D>({0.3, 0.0, -0.8}));
}

TEST_CASE("forward: differential reports no sideways motion", "[drive]") {
  check({0.3, 0.0, 0.5}, round_trip<D>({0.3, 1.0, 0.5}));
}

TEST_CASE("forward: equal wheel speeds are a straight line", "[drive]") {
  const Drive::WheelVelocities<D> wheels{{{Motor::Name::LEFT, 2.0},
                                          {Motor::Name::RIGHT, 2.0}}};

  check({2.0 * Drive::kWheelRadius, 0.0, 0.0},
        Drive::forward_kinematics<D>(wheels));
}

TEST_CASE("forward: wheels are read by name, not position", "[drive]") {
  const Drive::WheelVelocities<D> ordered{{{Motor::Name::LEFT, -1.0},
                                           {Motor::Name::RIGHT, 1.0}}};
  const Drive::WheelVelocities<D> swapped{{{Motor::Name::RIGHT, 1.0},
                                           {Motor::Name::LEFT, -1.0}}};

  check(Drive::forward_kinematics<D>(ordered),
        Drive::forward_kinematics<D>(swapped));
}

TEST_CASE("forward: mecanum recovers all three freedoms", "[drive]") {
  check({0.2, -0.35, 1.1}, round_trip<M>({0.2, -0.35, 1.1}));
}

// four wheels, three freedoms: one slipping wheel is averaged against the
// other three instead of being taken at its word
TEST_CASE("forward: a slipping mecanum wheel is outvoted", "[drive]") {
  auto wheels = Drive::inverse_kinematics<M>({0.5, 0.0, 0.0});
  wheels[0].angular_velocity += 4.0;

  const auto twist = Drive::forward_kinematics<M>(wheels);

  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.5 + 4.0 * Drive::kMecanumRadius / 4.0,
                            twist.linear_x);
}
