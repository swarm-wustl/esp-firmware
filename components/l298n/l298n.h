// Written with Claude
#ifndef L298N_H
#define L298N_H

#include <algorithm>
#include <array>
#include <cstddef>
#include <expected>
#include <utility>

#include "motor.h"
#include "swarm_hal.h"

namespace L298N {
struct MotorPins {
  Motor::Name name;
  int in_a;
  int in_b;
  int enable;
  int pwm_channel;
};

template <HAL::GenericGPIOController GPIO, HAL::GenericPWMController PWM,
          size_t MotorCount>
class MotorDriver {
public:
  MotorDriver(GPIO gpio, PWM pwm, std::array<MotorPins, MotorCount> motors,
              int standby)
      : gpio_(std::move(gpio)), pwm_(std::move(pwm)), motors_(motors),
        standby_(standby) {
    gpio_.set_direction(standby_, HAL::PinMode::Output);

    for (const MotorPins &motor : motors_) {
      gpio_.set_direction(motor.in_a, HAL::PinMode::Output);
      gpio_.set_direction(motor.in_b, HAL::PinMode::Output);
      pwm_.configure_channel(motor.pwm_channel, motor.enable);
    }
  }

  MotorDriver(const MotorDriver &) = delete;
  MotorDriver &operator=(const MotorDriver &) = delete;

  MotorDriver(MotorDriver &&) = default;
  MotorDriver &operator=(MotorDriver &&) = default;

  [[nodiscard]] std::expected<void, HAL::MotorError>
  run(const Motor::Command &cmd) {
    const MotorPins *motor = find(cmd.name);

    if (motor == nullptr) {
      return std::unexpected{HAL::MotorError::UnknownMotor};
    }

    HAL::Voltage level_a = HAL::Voltage::LOW;
    HAL::Voltage level_b = HAL::Voltage::LOW;

    switch (cmd.dir) {
    case Motor::Direction::FORWARD:
      level_a = HAL::Voltage::HIGH;
      break;

    case Motor::Direction::REVERSE:
      level_b = HAL::Voltage::HIGH;
      break;

    default:
      break;
    }

    gpio_.set_level(motor->in_a, level_a);
    gpio_.set_level(motor->in_b, level_b);
    gpio_.set_level(standby_, HAL::Voltage::HIGH);

    pwm_.set_duty_ratio(motor->pwm_channel, std::clamp(cmd.pwm_ratio, 0.0, 1.0));

    return {};
  }

  void stop() {
    for (const MotorPins &motor : motors_) {
      gpio_.set_level(motor.in_a, HAL::Voltage::LOW);
      gpio_.set_level(motor.in_b, HAL::Voltage::LOW);
      pwm_.set_duty_ratio(motor.pwm_channel, 0.0);
    }

    gpio_.set_level(standby_, HAL::Voltage::LOW);
  }

private:
  GPIO gpio_;
  PWM pwm_;
  std::array<MotorPins, MotorCount> motors_;
  int standby_;

  const MotorPins *find(Motor::Name name) const {
    auto it = std::ranges::find(motors_, name, &MotorPins::name);
    return it == motors_.end() ? nullptr : &*it;
  }
};
} // namespace L298N

#endif
