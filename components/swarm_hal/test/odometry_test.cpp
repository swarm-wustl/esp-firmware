#include "odometry.h"
#include "unity.h"

#include <numbers>

namespace {
constexpr double kPi = std::numbers::pi;

constexpr Odometry::Pose kOrigin{0.0, 0.0, 0.0};

void check(const Odometry::Pose &expected, const Odometry::Pose &actual) {
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, expected.x, actual.x);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, expected.y, actual.y);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, expected.theta, actual.theta);
}
} // namespace

TEST_CASE("odometry: a straight twist moves along the heading", "[odometry]") {
  check({2.0, 0.0, 0.0}, Odometry::integrate(kOrigin, {1.0, 0.0, 0.0}, 2.0));
  check({0.0, 2.0, kPi / 2.0},
        Odometry::integrate({0.0, 0.0, kPi / 2.0}, {1.0, 0.0, 0.0}, 2.0));
}

TEST_CASE("odometry: a pure rotation turns in place", "[odometry]") {
  check({0.0, 0.0, 1.0}, Odometry::integrate(kOrigin, {0.0, 0.0, 1.0}, 1.0));
}

TEST_CASE("odometry: a quarter turn of radius one ends at (1, 1)",
          "[odometry]") {
  check({1.0, 1.0, kPi / 2.0},
        Odometry::integrate(kOrigin, {1.0, 0.0, 1.0}, kPi / 2.0));
}

TEST_CASE("odometry: a full circle comes back to the start", "[odometry]") {
  const auto pose = Odometry::integrate(kOrigin, {0.5, 0.0, 1.0}, 2.0 * kPi);

  check({0.0, 0.0, 0.0}, pose);
}

// the closed form is exact for a constant twist, so how often the loop
// samples it must not change where the robot ends up
TEST_CASE("odometry: many short steps land where one long step does",
          "[odometry]") {
  const Drive::Twist twist{0.4, 0.1, -0.7};

  Odometry::Pose stepped = kOrigin;
  for (int i = 0; i < 100; ++i) {
    stepped = Odometry::integrate(stepped, twist, 0.02);
  }

  check(Odometry::integrate(kOrigin, twist, 2.0), stepped);
}

TEST_CASE("odometry: heading wraps into [-pi, pi]", "[odometry]") {
  const auto pose = Odometry::integrate({0.0, 0.0, 3.0}, {0.0, 0.0, 1.0}, 1.0);

  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 4.0 - 2.0 * kPi, pose.theta);
}
