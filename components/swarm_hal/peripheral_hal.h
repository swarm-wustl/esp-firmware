// Written with Claude
#ifndef PERIPHERAL_HAL_H
#define PERIPHERAL_HAL_H

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>

namespace HAL {
enum class Voltage : uint32_t { HIGH = 1, LOW = 0 };

constexpr uint32_t to_level(HAL::Voltage voltage) {
  return static_cast<uint32_t>(voltage);
}

enum class PinMode : uint8_t { Input, Output };

enum class SpiError : uint8_t { TransferFailed, Timeout };

template <typename SPI>
concept GenericSPIController =
    requires(SPI spi, std::span<std::byte> rx, std::span<const std::byte> tx) {
      {
        spi.transfer_halfduplex(tx, rx)
      } -> std::same_as<std::expected<void, SpiError>>;
    };

template <typename GPIO>
concept GenericGPIOController =
    requires(GPIO gpio, int pin, PinMode mode, Voltage voltage, int ms) {
      { gpio.set_direction(pin, mode) } -> std::same_as<void>;
      { gpio.set_level(pin, voltage) } -> std::same_as<void>;
      { gpio.delay_ms(ms) } -> std::same_as<void>;
    };
} // namespace HAL

#endif
