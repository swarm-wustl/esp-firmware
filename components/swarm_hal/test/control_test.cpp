#include "base.h"
#include "differential_drive.h"
#include "unity.h"
#include "watchdog.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <optional>

namespace {
constexpr auto D = Drive::Style::DIFFERENTIAL;

constexpr double kTopSpeed = 75.0 * 2.0 * std::numbers::pi / 60.0;
constexpr double kPeriod = 0.02;
constexpr double kTimeConstant = 0.1;
constexpr size_t kSubsteps = 20;

constexpr auto kControl = Drive::control({.rated = 6.0, .rail = 12.0}, kTopSpeed,
                                         kPeriod, Pid::gains(0.05, 0.5, 0.0));

enum class Heading { Encoders, Gyro };

constexpr double signed_duty(const Motor::Command &command) {
  switch (command.dir) {
  case Motor::Direction::FORWARD:
    return command.pwm_ratio;
  case Motor::Direction::REVERSE:
    return -command.pwm_ratio;
  case Motor::Direction::STOP:
    return 0.0;
  }

  return 0.0;
}

// `strength` is how far the real motor is from the top speed the loop was told,
// and `traction` how much of each wheel's turn reaches the floor
struct Plant {
  double strength = 1.0;
  std::array<double, 2> traction{1.0, 1.0};
};

// a first-order motor under a body that may not follow its wheels
class Sim {
public:
  explicit Sim(Plant plant = {}) : plant_{plant} {}

  void run(const Drive::Twist &twist, double seconds,
           Heading heading = Heading::Encoders) {
    const auto steps = static_cast<size_t>(seconds / kPeriod);

    for (size_t i = 0; i < steps; ++i) {
      tick(twist, heading);
    }
  }

  void tick(const Drive::Twist &twist, Heading heading) {
    const auto [next, frame] = Base::step<D, kControl>(
        state_, Drive::inverse_kinematics<D>(twist), measured_,
        heading == Heading::Gyro ? std::optional{yaw_rate_} : std::nullopt);

    state_ = next;
    frame_ = frame;

    advance();
  }

  double speed(size_t wheel) const { return speeds_[wheel]; }
  double heading_error() const { return Odometry::wrap(pose().theta - truth_.theta); }
  double position_error() const {
    return std::hypot(pose().x - truth_.x, pose().y - truth_.y);
  }

  const Odometry::Pose &pose() const { return state_.pose; }
  const Odometry::Pose &truth() const { return truth_; }
  const Base::State<D> &state() const { return state_; }
  const Drive::Frame<D> &frame() const { return frame_; }

private:
  Drive::Twist ground_twist() const {
    return Drive::forward_kinematics<D>(
        velocities(speeds_[0] * plant_.traction[0],
                   speeds_[1] * plant_.traction[1]));
  }

  static Drive::WheelVelocities<D> velocities(double left, double right) {
    return {Drive::WheelVelocity{Motor::Name::LEFT, left},
            Drive::WheelVelocity{Motor::Name::RIGHT, right}};
  }

  void advance() {
    constexpr double h = kPeriod / kSubsteps;
    const double per_duty =
        plant_.strength * kTopSpeed / kControl.supply.duty_limit();

    std::array<double, 2> turned{};

    for (size_t i = 0; i < kSubsteps; ++i) {
      for (size_t w = 0; w < speeds_.size(); ++w) {
        speeds_[w] += (per_duty * signed_duty(frame_.commands[w]) - speeds_[w]) *
                      h / kTimeConstant;
        turned[w] += speeds_[w] * h;
      }

      truth_ = Odometry::integrate(truth_, ground_twist(), h);
    }

    // encoders count across the period, so the loop sees its mean speed rather
    // than the speed at the end of it
    measured_ = velocities(turned[0] / kPeriod, turned[1] / kPeriod);
    yaw_rate_ = ground_twist().angular_z;
  }

  Plant plant_;
  Base::State<D> state_{};
  Drive::Frame<D> frame_{};
  Drive::WheelVelocities<D> measured_ = velocities(0.0, 0.0);
  std::array<double, 2> speeds_{};
  double yaw_rate_{};
  Odometry::Pose truth_{};
};

constexpr Drive::Twist forward(double wheel_speed) {
  return {wheel_speed * Drive::kWheelRadius, 0.0, 0.0};
}

constexpr Drive::Twist kArc{0.5, 0.0, 0.5};
} // namespace

TEST_CASE("control: wheels settle on target when the motor is weaker than told",
          "[control]") {
  Sim sim{{.strength = 0.85}};

  sim.run(kArc, 5.0);

  const auto targets = Drive::inverse_kinematics<D>(kArc);

  TEST_ASSERT_DOUBLE_WITHIN(0.02, targets[0].angular_velocity, sim.speed(0));
  TEST_ASSERT_DOUBLE_WITHIN(0.02, targets[1].angular_velocity, sim.speed(1));
}

TEST_CASE("control: wheels settle on target when the motor is stronger than told",
          "[control]") {
  Sim sim{{.strength = 1.15}};

  sim.run(kArc, 5.0);

  const auto targets = Drive::inverse_kinematics<D>(kArc);

  TEST_ASSERT_DOUBLE_WITHIN(0.02, targets[0].angular_velocity, sim.speed(0));
  TEST_ASSERT_DOUBLE_WITHIN(0.02, targets[1].angular_velocity, sim.speed(1));
}

TEST_CASE("control: a wheel in reverse settles too", "[control]") {
  Sim sim{{.strength = 0.85}};

  sim.run(forward(-3.0), 5.0);

  TEST_ASSERT_DOUBLE_WITHIN(0.02, -3.0, sim.speed(0));
  TEST_ASSERT_DOUBLE_WITHIN(0.02, -3.0, sim.speed(1));
}

TEST_CASE("control: time spent saturated does not wind the integral up",
          "[control]") {
  Sim sim{{.strength = 0.85}};

  sim.run(forward(100.0), 10.0);
  sim.run(forward(3.0), 1.0);

  TEST_ASSERT_DOUBLE_WITHIN(0.1, 3.0, sim.speed(0));
  TEST_ASSERT_DOUBLE_WITHIN(0.1, 3.0, sim.speed(1));
}

TEST_CASE("control: an unreachable twist keeps the wheel ratio", "[control]") {
  Sim sim{{.strength = 1.1}};

  const Drive::Twist twist{20.0, 0.0, 10.0};

  sim.run(twist, 5.0);

  const auto targets = Drive::inverse_kinematics<D>(twist);

  TEST_ASSERT_DOUBLE_WITHIN(
      0.01, targets[0].angular_velocity / targets[1].angular_velocity,
      sim.speed(0) / sim.speed(1));
  TEST_ASSERT_DOUBLE_WITHIN(0.02, kTopSpeed, sim.speed(1));
}

TEST_CASE("control: a zero twist coasts and clears the integral", "[control]") {
  Sim sim{{.strength = 0.85}};

  sim.run(forward(3.0), 2.0);
  sim.run({0.0, 0.0, 0.0}, 2.0);

  for (const Motor::Command &command : sim.frame().commands) {
    TEST_ASSERT_TRUE(command.dir == Motor::Direction::STOP);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, command.pwm_ratio);
  }

  for (const Pid::State &regulator : sim.state().regulators) {
    TEST_ASSERT_EQUAL_DOUBLE(0.0, regulator.integral);
  }

  TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, sim.speed(0));
  TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, sim.speed(1));
}

TEST_CASE("control: losing the command stream coasts the robot to a stop",
          "[control]") {
  constexpr Watchdog::Policy<Drive::Twist> policy{
      .safe = {}, .timeout_ms = 500, .period_ms = 20};

  Sim sim{{.strength = 0.85}};

  sim.run(forward(3.0), 2.0);

  Watchdog::Feed<Drive::Twist> feed{forward(3.0), 0};

  for (size_t i = 0; i < 24; ++i) {
    feed = Watchdog::observe(feed, std::optional<Drive::Twist>{}, policy);
    sim.tick(feed.value, Heading::Encoders);
  }

  TEST_ASSERT_DOUBLE_WITHIN(0.1, 3.0, sim.speed(0));

  for (size_t i = 0; i < 100; ++i) {
    feed = Watchdog::observe(feed, std::optional<Drive::Twist>{}, policy);
    sim.tick(feed.value, Heading::Encoders);
  }

  TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, sim.speed(0));
  TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, sim.speed(1));
}

TEST_CASE("control: odometry tracks the true pose round a square", "[control]") {
  Sim sim{{.strength = 0.85}};

  for (size_t side = 0; side < 4; ++side) {
    sim.run({0.5, 0.0, 0.0}, 4.0);
    sim.run({0.0, 0.0, 0.0}, 1.0);
    sim.run({0.0, 0.0, 0.5}, std::numbers::pi);
    sim.run({0.0, 0.0, 0.0}, 1.0);
  }

  TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, sim.position_error());
  TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, sim.heading_error());
}

TEST_CASE("control: wheel slip corrupts an encoder heading but not a gyro one",
          "[control]") {
  const Plant slipping{.strength = 0.85, .traction = {1.0, 0.8}};

  Sim encoders{slipping};
  Sim gyro{slipping};

  encoders.run(kArc, 4.0, Heading::Encoders);
  gyro.run(kArc, 4.0, Heading::Gyro);

  TEST_ASSERT_TRUE(std::abs(encoders.heading_error()) > 0.1);
  TEST_ASSERT_DOUBLE_WITHIN(0.02, 0.0, gyro.heading_error());
  TEST_ASSERT_TRUE(gyro.position_error() < encoders.position_error());
}
