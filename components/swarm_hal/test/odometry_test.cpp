#include "odometry.h"
#include "unity.h"

#include <cmath>
#include <numbers>
#include <optional>

namespace {
constexpr double kPi = std::numbers::pi;

constexpr Odometry::Pose kOrigin{0.0, 0.0, 0.0};

void check(const Odometry::Pose &expected, const Odometry::Pose &actual) {
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, expected.x, actual.x);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, expected.y, actual.y);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, expected.theta, actual.theta);
}

void check(const Drive::Twist &expected, const Drive::Twist &actual) {
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, expected.linear_x, actual.linear_x);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, expected.linear_y, actual.linear_y);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, expected.angular_z, actual.angular_z);
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

TEST_CASE("odometry: a heading is a unit quaternion about z", "[odometry]") {
  const auto [z, w] = Odometry::heading(kPi / 2.0);

  TEST_ASSERT_DOUBLE_WITHIN(1e-9, std::sqrt(0.5), z);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, std::sqrt(0.5), w);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, z * z + w * w);
}

TEST_CASE("odometry: heading wraps into [-pi, pi]", "[odometry]") {
  const auto pose = Odometry::integrate({0.0, 0.0, 3.0}, {0.0, 0.0, 1.0}, 1.0);

  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 4.0 - 2.0 * kPi, pose.theta);
}

TEST_CASE("odometry: a gyro rate replaces only the encoder's angular_z",
          "[odometry]") {
  const Drive::Twist body =
      Odometry::with_yaw_rate({1.0, 2.0, 3.0}, std::optional{0.5});

  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, body.linear_x);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 2.0, body.linear_y);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.5, body.angular_z);
}

TEST_CASE("odometry: no gyro rate leaves the encoder's angular_z",
          "[odometry]") {
  const Drive::Twist body =
      Odometry::with_yaw_rate({1.0, 2.0, 3.0}, std::nullopt);

  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 3.0, body.angular_z);
}

TEST_CASE("odometry: twist_between recovers the twist integrate was given",
          "[odometry]") {
  const Odometry::Pose start{0.3, -0.2, 2.5};

  for (const Drive::Twist twist :
       {Drive::Twist{0.2, 0.0, 0.0}, Drive::Twist{0.15, 0.0, 1.3},
        Drive::Twist{-0.1, 0.0, -0.7}, Drive::Twist{0.0, 0.0, 2.0},
        Drive::Twist{0.1, 0.05, 0.4}}) {
    const auto end = Odometry::integrate(start, twist, 0.2);

    check(twist, Odometry::twist_between(start, end, 0.2));
  }
}

TEST_CASE("odometry: integrating twist_between lands on the later pose",
          "[odometry]") {
  const Odometry::Pose from{1.0, 2.0, -3.0};
  const Odometry::Pose to{1.04, 1.97, 2.9};

  check(to,
        Odometry::integrate(from, Odometry::twist_between(from, to, 0.2), 0.2));
}

TEST_CASE("odometry: twist_between over no time is still", "[odometry]") {
  check(Drive::Twist{}, Odometry::twist_between(kOrigin, {1.0, 0.0, 0.0}, 0.0));
}
