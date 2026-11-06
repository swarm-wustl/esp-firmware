#ifndef L298N_H
#define L298N_H

#include <algorithm>
#include <array>
#include <concepts>
#include <cstddef>
#include <utility>

#include "arrays.h"
#include "assembly.h"
#include "motor.h"
#include "swarm_hal.h"

namespace L298N {
struct MotorPins {
  Motor::Name name;
  int in_a;
  int in_b;
  int enable;
  int pwm_channel;

  consteval auto pins() const {
    return HAL::pins(HAL::NamedPin{"in_a", in_a},
                     HAL::NamedPin{"in_b", in_b},
                     HAL::NamedPin{"enable", enable});
  }
};

template <std::same_as<MotorPins>... Pins> consteval auto motors(Pins... pins) {
  const std::array table{pins...};

  // two rows naming the same motor are not a constant expression, so the
  // declaration fails to build
  if (Arrays::distinct(Arrays::map(table, &MotorPins::name))) {
    return table;
  }

  std::unreachable();
}

template <HAL::GenericGPIOController GPIO, HAL::GenericPWMController PWM,
          auto Pins>
  requires HAL::Claiming<PWM>
class MotorDriver {
public:
  static constexpr auto motors = Arrays::map(Pins, &MotorPins::name);

  // the LEDC timer belongs to the PWM this driver owns
  static constexpr auto claims = Arrays::concat(
      HAL::row_claims<Pins>([](const MotorPins &motor) {
        return HAL::Claim{HAL::Resource::PwmChannel, motor.pwm_channel,
                          HAL::Use::Exclusive};
      }),
      PWM::claims);

  explicit MotorDriver(Swarm::Assembly assembly)
      : gpio_{assembly}, pwm_{assembly} {
    for (size_t i = 0; i < Pins.size(); ++i) {
      gpio_.set_direction(kInA[i], HAL::PinMode::Output);
      gpio_.set_direction(kInB[i], HAL::PinMode::Output);
      pwm_.configure_channel(Pins[i].pwm_channel, kEnable[i]);
    }
  }

  MotorDriver(const MotorDriver &) = delete;
  MotorDriver &operator=(const MotorDriver &) = delete;

  MotorDriver(MotorDriver &&) = default;
  MotorDriver &operator=(MotorDriver &&) = default;

  template <size_t N>
    requires(N <= Pins.size())
  void run(const Motor::Frame<N> &frame) {
    for (const Motor::Command &cmd : frame.commands) {
      apply(slot_of(cmd.name), cmd);
    }
  }

  void stop() {
    for (size_t i = 0; i < Pins.size(); ++i) {
      apply(i, Motor::Command{Pins[i].name, Motor::Direction::STOP, 0.0});
    }
  }

private:
  // resolved once, at compile time: a label the declaration does not carry is
  // a build error, and nothing here can name a pin that was never claimed
  static constexpr auto kInA = HAL::pin_column<Pins, "in_a">();
  static constexpr auto kInB = HAL::pin_column<Pins, "in_b">();
  static constexpr auto kEnable = HAL::pin_column<Pins, "enable">();

  GPIO gpio_;
  PWM pwm_;

  static constexpr size_t slot_of(Motor::Name name) {
    return static_cast<size_t>(
        std::ranges::find(Pins, name, &MotorPins::name) - Pins.begin());
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
