// Written with Claude
#ifndef L298N_H
#define L298N_H

#include <algorithm>
#include <array>
#include <cstddef>
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
          auto Pins>
class MotorDriver {
public:
  static constexpr size_t MotorCount = Pins.size();

  MotorDriver(GPIO gpio, PWM pwm, int standby)
      : gpio_(std::move(gpio)), pwm_(std::move(pwm)), standby_(standby) {
    gpio_.set_direction(standby_, HAL::PinMode::Output);

    for (const MotorPins &motor : Pins) {
      gpio_.set_direction(motor.in_a, HAL::PinMode::Output);
      gpio_.set_direction(motor.in_b, HAL::PinMode::Output);
      pwm_.configure_channel(motor.pwm_channel, motor.enable);
    }
  }

  MotorDriver(const MotorDriver &) = delete;
  MotorDriver &operator=(const MotorDriver &) = delete;

  MotorDriver(MotorDriver &&) = default;
  MotorDriver &operator=(MotorDriver &&) = default;

  template <auto FrameNames>
  void run(const Drive::Frame<FrameNames> &frame) {
    constexpr std::array<size_t, MotorCount> slots =
        HAL::slot_map<FrameNames, pin_names()>();

    for (size_t i = 0; i < MotorCount; ++i) {
      apply(Pins[i], frame.commands[slots[i]]);
    }

    gpio_.set_level(standby_, HAL::Voltage::HIGH);
  }

  void stop() {
    for (const MotorPins &motor : Pins) {
      apply(motor, Motor::Command{Motor::Direction::STOP, 0.0});
    }

    gpio_.set_level(standby_, HAL::Voltage::LOW);
  }

private:
  static constexpr std::array<Motor::Name, MotorCount> pin_names() {
    std::array<Motor::Name, MotorCount> names{};

    for (size_t i = 0; i < MotorCount; ++i) {
      names[i] = Pins[i].name;
    }

    return names;
  }

  GPIO gpio_;
  PWM pwm_;
  int standby_;

  void apply(const MotorPins &pins, const Motor::Command &cmd) {
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

    gpio_.set_level(pins.in_a, level_a);
    gpio_.set_level(pins.in_b, level_b);
    pwm_.set_duty_ratio(pins.pwm_channel, std::clamp(cmd.pwm_ratio, 0.0, 1.0));
  }
};
} // namespace L298N

#endif
