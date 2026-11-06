#ifndef BASE_H
#define BASE_H

#include <optional>

#include "drive.h"
#include "odometry.h"

namespace Base {
template <Drive::Style S> struct State {
  Drive::Regulators<S> regulators{};
  Odometry::Pose pose{};
};

template <Drive::Style S> struct Step {
  State<S> state;
  Drive::Frame<S> frame;
};

template <Drive::Style S, Drive::Control C>
constexpr auto step(const State<S> &state,
                    const Drive::WheelVelocities<S> &targets,
                    const Drive::WheelVelocities<S> &measured,
                    std::optional<double> yaw_rate) -> Step<S> {
  const auto [regulators, frame] =
      Drive::regulate<S, C>(state.regulators, targets, measured);

  const auto pose = Odometry::integrate(
      state.pose,
      Odometry::with_yaw_rate(Drive::forward_kinematics<S>(measured), yaw_rate),
      C.period);

  return {{regulators, pose}, frame};
}
} // namespace Base

#endif
