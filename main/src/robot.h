#ifndef ROBOT_H
#define ROBOT_H

// shared with the rclcpp sim in sim/, so no IDF or ROS headers here. The
// message helpers are templates because micro-ROS's C structs and rclcpp's
// classes have the same field names

#include <array>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <numbers>
#include <optional>
#include <string_view>

#include "arrays.h"
#include "base.h"
#include "differential_drive.h"
#include "encoder.h"
#include "odometry.h"

namespace Robot {
inline constexpr char NODE[] = "base";
inline constexpr char CMD_VEL_TOPIC[] = "cmd_vel";
inline constexpr char ODOM_TOPIC[] = "odom/twist";
inline constexpr char UWB_TOPIC[] = "uwb/range";

// frame names within one robot. Every robot's TF tree shares the graph, so
// each is published under the robot's namespace (frame_in) and only map is
// common to all of them
inline constexpr char ODOM_FRAME[] = "odom";
inline constexpr char BASE_FRAME[] = "base_link";
inline constexpr char UWB_FRAME[] = "uwb_link";

template <size_t N> struct FrameName {
  char value[N]{};
};

// "swarm_1" + "base_link" -> "swarm_1/base_link". The board's namespace is
// fixed at build time, so its frames are built here rather than at boot; the
// sim joins the same way at runtime, once per robot
template <size_t A, size_t B>
consteval FrameName<A + B> frame_in(const char (&ns)[A],
                                    const char (&frame)[B]) {
  FrameName<A + B> out{};
  size_t i = 0;

  for (size_t j = 0; j + 1 < A; ++j) {
    out.value[i++] = ns[j];
  }

  if (A > 1) {
    out.value[i++] = '/';
  }

  for (size_t j = 0; j < B; ++j) {
    out.value[i++] = frame[j];
  }

  return out;
}

static_assert(std::string_view{frame_in("swarm_1", "base_link").value} ==
              "swarm_1/base_link");
static_assert(std::string_view{frame_in("", "base_link").value} == "base_link");

constexpr Drive::Style STYLE = Drive::Style::DIFFERENTIAL;

constexpr Encoder::Geometry WHEEL_ENCODER = Encoder::fit0485;

// 20ms at X4 is ~150 counts per period at top speed, so the measured velocity
// is not mostly quantisation noise the way it would be at 5ms
constexpr uint32_t DRIVE_PERIOD_MS = 20;

// a robot that has lost its server must not keep driving while the others plan
// around it. Long enough to ride out a WiFi hiccup, so cmd_vel has to be
// streamed faster than this
constexpr uint32_t CMD_VEL_TIMEOUT_MS = 500;

// FIT0485 is a 6V motor on a 12V rail, deliberately overdriven to 8V for
// speed. Watch the motor temperature before pushing this further
// TODO: the rail sags with the battery; measure it under load
constexpr Motor::Supply WHEEL_SUPPLY{.rated = 8.0, .rail = 12.0};

// FIT0485 is listed at 75rpm no-load at 6V but measured ~43rpm on the bench;
// speed tracks voltage, so it scales with the overdrive. The feedforward only
// has to be close, the integral takes up the rest
// TODO: measure at the rated voltage, and tune the gains
constexpr double MAX_WHEEL_SPEED =
    43.0 * WHEEL_SUPPLY.rated / 6.0 * 2.0 * std::numbers::pi / 60.0;

constexpr auto CONTROL =
    Drive::control(WHEEL_SUPPLY, MAX_WHEEL_SPEED, DRIVE_PERIOD_MS / 1000.0,
                   Pid::gains(0.05, 0.5, 0.0));

constexpr uint32_t ODOMETRY_PERIOD_MS = 200;

template <Encoder::Spec Spec, size_t N>
constexpr auto wheel_velocities(const std::array<Encoder::Reading, N> &counts) {
  return Arrays::map(counts, [](const Encoder::Reading &r) {
    return Drive::WheelVelocity{
        r.name, Encoder::rad_per_second(Spec, r.counts, CONTROL.period)};
  });
}

template <Encoder::Spec Spec>
constexpr auto
drive(const Base::State<STYLE> &state,
      const Drive::WheelVelocities<STYLE> &targets,
      const std::array<Encoder::Reading, Drive::motor_count<STYLE>> &counts,
      std::optional<double> yaw_rate) -> Base::Step<STYLE> {
  return Base::step<STYLE, CONTROL>(state, targets,
                                    wheel_velocities<Spec>(counts), yaw_rate);
}

template <typename Msg> constexpr Drive::Twist read_twist(const Msg &msg) {
  return {msg.linear.x, msg.linear.y, msg.angular.z};
}

consteval std::array<double, 36> diagonal(std::array<double, 6> variance) {
  std::array<double, 36> out{};

  for (size_t i = 0; i < variance.size(); ++i) {
    out[i * 6 + i] = variance[i];
  }

  return out;
}

// row-major 6x6 over (vx, vy, vz, wx, wy, wz). Velocity error does not
// accumulate, so unlike a pose's a constant is the right shape. vz, wx and wy
// are never measured, so they get a variance large enough that an EKF ignores
// them rather than one of zero, which it would take as certainty. vy is small
// rather than unmeasured: a differential base cannot slide sideways, and
// saying so is a measurement
// TODO: placeholders (~2 cm/s, ~0.05 rad/s). Calibrate wheel radius and gyro
// bias first -- a systematic error is a bias, which no covariance can carry --
// then set these from the spread that remains on the bench (wz at rest, vx
// over a taped distance), and check them against how hard UWB fixes pull
inline constexpr std::array<double, 36> kTwistCovariance =
    diagonal({4e-4, 1e-4, 1e6, 1e6, 1e6, 2.5e-3});

template <typename Msg> void fill_twist(Msg &msg, const Drive::Twist &twist) {
  msg.twist.twist.linear.x = twist.linear_x;
  msg.twist.twist.linear.y = twist.linear_y;
  msg.twist.twist.linear.z = 0.0;
  msg.twist.twist.angular.x = 0.0;
  msg.twist.twist.angular.y = 0.0;
  msg.twist.twist.angular.z = twist.angular_z;

  std::ranges::copy(kTwistCovariance, std::ranges::begin(msg.twist.covariance));
}

// sensor_msgs/Range INFRARED, which the C and C++ bindings spell differently
inline constexpr uint8_t kRangeInfrared = 1;

template <typename Msg> void fill_range(Msg &msg, const double &meters) {
  // Range was written for sonar/IR; UWB has no radiation_type of its own, and
  // INFRARED is the conventional stand-in until this moves to a swarm_msgs type
  msg.radiation_type = kRangeInfrared;
  msg.field_of_view = 6.2831853f;
  msg.min_range = 0.0f;
  msg.max_range = 300.0f;
  msg.range = static_cast<float>(meters);
}
} // namespace Robot

#endif
