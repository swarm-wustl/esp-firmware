#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <cmath>
#include <numbers>
#include <optional>

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

// wheels slip most while turning, so a gyro's rate outranks the encoders' when
// there is one. Without it the encoders still give a heading, just a worse one
constexpr Drive::Twist with_yaw_rate(const Drive::Twist &body,
                                     std::optional<double> yaw_rate) {
  return {body.linear_x, body.linear_y, yaw_rate.value_or(body.angular_z)};
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
// the constant twist that carries `from` to `to` in dt: integrate's inverse, so
// a consumer integrating it at a slow publish rate retraces the path the
// control loop integrated at its own, instead of extrapolating one sample.
// A turn past pi in one interval aliases, which no wheel reaches in 200ms
constexpr Drive::Twist twist_between(const Pose &from, const Pose &to,
                                     double dt) {
  if (dt <= 0.0) {
    return {};
  }

  const double turn = wrap(to.theta - from.theta);
  const double angular_z = turn / dt;

  const bool straight = std::abs(turn) < 1e-9;
  const double along = straight ? dt : std::sin(turn) / angular_z;
  const double across = straight ? 0.0 : (1.0 - std::cos(turn)) / angular_z;

  const double c = std::cos(from.theta);
  const double s = std::sin(from.theta);
  const double forward = c * (to.x - from.x) + s * (to.y - from.y);
  const double left = -s * (to.x - from.x) + c * (to.y - from.y);

  // integrate maps (vx, vy) through [[along, -across], [across, along]], a
  // rotation and a scale, so its inverse is the transpose over the determinant
  const double scale = along * along + across * across;

  return {.linear_x = (along * forward + across * left) / scale,
          .linear_y = (along * left - across * forward) / scale,
          .angular_z = angular_z};
}
} // namespace Odometry

#endif
