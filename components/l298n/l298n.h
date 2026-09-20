// Written with Claude
#ifndef L298N_H
#define L298N_H

#include <algorithm>
#include <array>
#include <concepts>
#include <cstddef>
#include <utility>

#include "drive.h"
#include "motor.h"
#include "swarm_hal.h"

namespace L298N {
struct MotorPins {
  Motor::Name name;
  int in_a;
  int in_b;
  int enable;
  int pwm_channel;
  int standby;
};

template <Drive::Style S, size_t N>
consteval bool pins_match_roles(const std::array<MotorPins, N> &pins) {
  constexpr auto rows = Drive::wheels<S>();

  if (N != rows.size()) {
    return false;
  }

  return std::ranges::all_of(rows, [&pins](const Drive::Wheel &wheel) {
    return std::ranges::count(pins, wheel.name, &MotorPins::name) == 1;
  });
}

// a Config used as a template argument has to be structural, so its members
// stay public and the driver re-checks the invariant rather than trusting that
// this was the only way one got built
template <Drive::Style S, size_t N> struct Config {
  static constexpr Drive::Style style = S;

  std::array<MotorPins, N> pins;
};

// -fno-exceptions rules out `throw`, and the condition isn't constant in this
// context so `static_assert` can't see it either. An undefined consteval call
// fails the constant evaluation and names itself in the diagnostic
consteval void pin_table_does_not_match_drive_style();

template <Drive::Style S, std::same_as<MotorPins>... Pins>
consteval auto with_drive_style(Pins... pins) {
  const std::array table{pins...};

  if (!pins_match_roles<S>(table)) {
    pin_table_does_not_match_drive_style();
  }

  return Config<S, sizeof...(Pins)>{table};
}

template <HAL::GenericGPIOController GPIO, HAL::GenericPWMController PWM,
          auto Cfg>
class MotorDriver {
  static constexpr Drive::Style Style = decltype(Cfg)::style;
  static constexpr auto Pins = Cfg.pins;

  static_assert(pins_match_roles<Style>(Pins),
                "pin table and drive style must name the same motors, one row "
                "each");

public:
  MotorDriver(GPIO gpio, PWM pwm)
      : gpio_(std::move(gpio)), pwm_(std::move(pwm)) {
    for (const MotorPins &motor : Pins) {
      gpio_.set_direction(motor.in_a, HAL::PinMode::Output);
      gpio_.set_direction(motor.in_b, HAL::PinMode::Output);
      gpio_.set_direction(motor.standby, HAL::PinMode::Output);
      pwm_.configure_channel(motor.pwm_channel, motor.enable);
    }
  }

  MotorDriver(const MotorDriver &) = delete;
  MotorDriver &operator=(const MotorDriver &) = delete;

  MotorDriver(MotorDriver &&) = default;
  MotorDriver &operator=(MotorDriver &&) = default;

  void run(const Drive::Frame<Style> &frame) {
    for (const Motor::Command &cmd : frame.commands) {
      apply(*std::ranges::find(Pins, cmd.name, &MotorPins::name), cmd);
    }

    set_standby(HAL::Voltage::HIGH);
  }

  void stop() {
    for (const MotorPins &motor : Pins) {
      apply(motor, Motor::Command{motor.name, Motor::Direction::STOP, 0.0});
    }

    set_standby(HAL::Voltage::LOW);
  }

private:
  GPIO gpio_;
  PWM pwm_;

  void set_standby(HAL::Voltage level) {
    for (const MotorPins &motor : Pins) {
      gpio_.set_level(motor.standby, level);
    }
  }

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
