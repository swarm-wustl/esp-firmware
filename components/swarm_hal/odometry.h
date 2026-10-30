#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <cmath>
#include <numbers>

#include "drive.h"

namespace Odometry {
struct Pose {
  double x;     // m
  double y;     // m
  double theta; // rad, in [-pi, pi]
};

// a planar heading as a quaternion: a rotation about z alone, so x and y are
// zero
struct Heading {
  double z;
  double w;
};

constexpr Heading heading(double theta) {
  return {.z = std::sin(theta / 2.0), .w = std::cos(theta / 2.0)};
}

constexpr double wrap(double angle) {
  return std::remainder(angle, 2.0 * std::numbers::pi);
}

// a twist held for dt traces an arc, not a chord. Stepping along the chord
// (x += v cos(theta) dt) drifts outward on every turn; the closed form is
// exact for a constant twist, so the error is only how constant it really was
// Basically, from the arc length and its angle, compute the radius and use that
// instead of naive v*dt Euler-type integration.
constexpr Pose integrate(const Pose &pose, const Drive::Twist &body,
                         double dt) {
  const double turn = body.angular_z * dt;

  // sin(turn)/w and (1 - cos(turn))/w are 0/0 on a straight line; their limits
  // are dt and 0
  const bool straight = std::abs(turn) < 1e-9;
  const double along = straight ? dt : std::sin(turn) / body.angular_z;
  const double across =
      straight ? 0.0 : (1.0 - std::cos(turn)) / body.angular_z;

  const double forward = body.linear_x * along - body.linear_y * across;
  const double left = body.linear_x * across + body.linear_y * along;

  const double c = std::cos(pose.theta);
  const double s = std::sin(pose.theta);

  return {.x = pose.x + c * forward - s * left,
          .y = pose.y + s * forward + c * left,
          .theta = wrap(pose.theta + turn)};
}
} // namespace Odometry

#endif
