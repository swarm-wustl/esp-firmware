// Written with Claude
#ifndef L298N_H
#define L298N_H

#include <algorithm>
#include <array>
#include <concepts>
#include <cstddef>
#include <utility>

#include "assembly.h"
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

template <size_t N>
consteval auto roster_of(const std::array<MotorPins, N> &pins) {
  std::array<Motor::Name, N> names{};

  for (size_t i = 0; i < N; ++i) {
    names[i] = pins[i].name;
  }

  return names;
}

template <size_t N>
consteval auto claims_of(const std::array<MotorPins, N> &pins) {
  std::array<HAL::Claim, N * 5> claims{};
  size_t next = 0;

  for (const MotorPins &motor : pins) {
    claims[next++] = {HAL::Resource::Gpio, motor.in_a, HAL::Use::Exclusive};
    claims[next++] = {HAL::Resource::Gpio, motor.in_b, HAL::Use::Exclusive};
    claims[next++] = {HAL::Resource::Gpio, motor.enable, HAL::Use::Exclusive};
    claims[next++] = {HAL::Resource::PwmChannel, motor.pwm_channel,
                      HAL::Use::Exclusive};
    // one STBY drives both halves of an L298N, so motors are expected to share
    claims[next++] = {HAL::Resource::Gpio, motor.standby, HAL::Use::Shared};
  }

  return claims;
}

consteval void two_motors_share_a_name();

template <std::same_as<MotorPins>... Pins> consteval auto motors(Pins... pins) {
  const std::array table{pins...};

  if (!HAL::unique_names(roster_of(table))) {
    two_motors_share_a_name();
  }

  return table;
}

template <HAL::GenericGPIOController GPIO, HAL::GenericPWMController PWM,
          auto Pins>
  requires HAL::Claiming<PWM>
class MotorDriver {
public:
  static constexpr auto motors = roster_of(Pins);

  // the LEDC timer belongs to the PWM this driver owns
  static constexpr auto claims = HAL::concat(claims_of(Pins), PWM::claims);

  explicit MotorDriver(Swarm::Assembly assembly) : pwm_(assembly) {
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

  template <Drive::Style S> void run(const Drive::Frame<S> &frame) {
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
  GPIO gpio_{};
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
