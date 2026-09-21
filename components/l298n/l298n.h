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
using HAL::operator""_p;

struct MotorPins {
  Motor::Name name;
  int in_a;
  int in_b;
  int enable;
  int pwm_channel;
  int standby;

  // one STBY drives both halves of an L298N, so motors are expected to share it
  consteval auto pins() const {
    return HAL::pins(HAL::NamedPin{"in_a", in_a},
                     HAL::NamedPin{"in_b", in_b},
                     HAL::NamedPin{"enable", enable},
                     HAL::NamedPin{"standby", standby, HAL::Use::Shared});
  }
};

template <auto Table, HAL::fixed_string L> consteval auto pin_column() {
  return [&]<size_t... I>(std::index_sequence<I...>) {
    return std::array<HAL::Pin, sizeof...(I)>{
        Table[I].pins()[L]...};
  }(std::make_index_sequence<Table.size()>{});
}

template <size_t N>
consteval auto roster_of(const std::array<MotorPins, N> &pins) {
  std::array<Motor::Name, N> names{};

  for (size_t i = 0; i < N; ++i) {
    names[i] = pins[i].name;
  }

  return names;
}

template <size_t N>
consteval auto claims_of(const std::array<MotorPins, N> &table) {
  std::array<HAL::Claim, N * 5> claims{};
  size_t next = 0;

  for (const MotorPins &motor : table) {
    for (const HAL::Claim &claim : motor.pins().claims()) {
      claims[next++] = claim;
    }

    claims[next++] = {HAL::Resource::PwmChannel, motor.pwm_channel,
                      HAL::Use::Exclusive};
  }

  return claims;
}

template <std::same_as<MotorPins>... Pins> consteval auto motors(Pins... pins) {
  const std::array table{pins...};

  // two rows naming the same motor are not a constant expression, so the
  // declaration fails to build
  if (HAL::unique_names(roster_of(table))) {
    return table;
  }

  std::unreachable();
}

template <HAL::GenericGPIOController GPIO, HAL::GenericPWMController PWM,
          auto Pins>
  requires HAL::Claiming<PWM>
class MotorDriver {
public:
  static constexpr auto motors = roster_of(Pins);

  // the LEDC timer belongs to the PWM this driver owns
  static constexpr auto claims = HAL::concat(claims_of(Pins), PWM::claims);

  explicit MotorDriver(Swarm::Assembly assembly)
      : gpio_{assembly}, pwm_{assembly} {
    for (size_t i = 0; i < Pins.size(); ++i) {
      gpio_.set_direction(kInA[i], HAL::PinMode::Output);
      gpio_.set_direction(kInB[i], HAL::PinMode::Output);
      gpio_.set_direction(kStandby[i], HAL::PinMode::Output);
      pwm_.configure_channel(Pins[i].pwm_channel, kEnable[i]);
    }
  }

  MotorDriver(const MotorDriver &) = delete;
  MotorDriver &operator=(const MotorDriver &) = delete;

  MotorDriver(MotorDriver &&) = default;
  MotorDriver &operator=(MotorDriver &&) = default;

  template <Drive::Style S> void run(const Drive::Frame<S> &frame) {
    for (const Motor::Command &cmd : frame.commands) {
      apply(slot_of(cmd.name), cmd);
    }

    set_standby(HAL::Voltage::HIGH);
  }

  void stop() {
    for (size_t i = 0; i < Pins.size(); ++i) {
      apply(i, Motor::Command{Pins[i].name, Motor::Direction::STOP, 0.0});
    }

    set_standby(HAL::Voltage::LOW);
  }

private:
  // resolved once, at compile time: a label the declaration does not carry is
  // a build error, and nothing here can name a pin that was never claimed
  static constexpr auto kInA = pin_column<Pins, "in_a">();
  static constexpr auto kInB = pin_column<Pins, "in_b">();
  static constexpr auto kEnable = pin_column<Pins, "enable">();
  static constexpr auto kStandby = pin_column<Pins, "standby">();

  GPIO gpio_;
  PWM pwm_;

  static constexpr size_t slot_of(Motor::Name name) {
    return static_cast<size_t>(
        std::ranges::find(Pins, name, &MotorPins::name) - Pins.begin());
  }

  void set_standby(HAL::Voltage level) {
    for (const HAL::Pin &standby : kStandby) {
      gpio_.set_level(standby, level);
    }
  }

  void apply(size_t slot, const Motor::Command &cmd) {
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

    gpio_.set_level(kInA[slot], level_a);
    gpio_.set_level(kInB[slot], level_b);
    pwm_.set_duty_ratio(Pins[slot].pwm_channel,
                        std::clamp(cmd.pwm_ratio, 0.0, 1.0));
  }
};
} // namespace L298N

#endif
