#ifndef ESP32_H
#define ESP32_H

#include <array>
#include <memory>
#include <tuple>

#include <driver/spi_common.h>
#include <driver/spi_master.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "assembly.h"
#include "driver/gpio.h"
#include "swarm_hal.h"

namespace ESP32 {
// the bus owns the peripheral: spi_bus_initialize/free happen once here, and
// devices attach to it. SPI2_HOST is Exclusive -- a second initialize on the
// same host fails at runtime and only gets logged
class SpiBus {
public:
  static constexpr spi_host_device_t host = SPI2_HOST;
  static constexpr int sck = 18;
  static constexpr int miso = 19;
  static constexpr int mosi = 23;

  static constexpr std::array claims{
      HAL::Claim{HAL::Resource::SpiHost, static_cast<int>(host),
                 HAL::Use::Exclusive},
      HAL::Claim{HAL::Resource::Gpio, sck, HAL::Use::Shared},
      HAL::Claim{HAL::Resource::Gpio, miso, HAL::Use::Shared},
      HAL::Claim{HAL::Resource::Gpio, mosi, HAL::Use::Shared},
  };

  explicit SpiBus(Swarm::Assembly);
  ~SpiBus();

  SpiBus(const SpiBus &) = delete;
  void operator=(const SpiBus &) = delete;

  SpiBus(SpiBus &&other);
  SpiBus &operator=(SpiBus &&other);

private:
  bool owns_bus_{};

  void swap(SpiBus &other);
};

// the untemplated half, so the IDF calls stay in spi.cpp
class SpiDevice {
public:
  SpiDevice(Swarm::Assembly, SpiBus &bus, int cs);
  ~SpiDevice();

  SpiDevice(const SpiDevice &) = delete;
  void operator=(const SpiDevice &) = delete;

  SpiDevice(SpiDevice &&other);
  SpiDevice &operator=(SpiDevice &&other);

  std::expected<void, HAL::SpiError>
  transfer_halfduplex(std::span<const std::byte> tx, std::span<std::byte> rx);

private:
  int cs_{};
  spi_device_handle_t dev_handle_{};

  void swap(SpiDevice &other);
};

// a device on a bus. The chip select is part of the type, so the device claims
// it rather than leaving it to whoever happens to construct one
template <int Cs> class SPI {
public:
  static constexpr std::array claims{
      HAL::Claim{HAL::Resource::Gpio, Cs, HAL::Use::Exclusive},
  };

  SPI(Swarm::Assembly assembly, SpiBus &bus) : device_{assembly, bus, Cs} {}

  std::expected<void, HAL::SpiError>
  transfer_halfduplex(std::span<const std::byte> tx, std::span<std::byte> rx) {
    return device_.transfer_halfduplex(tx, rx);
  }

private:
  SpiDevice device_;
};

class GPIO {
public:
  // an accessor, not an owner: it takes no hardware of its own, but it is
  // built through the config like every other peripheral
  static constexpr std::array<HAL::Claim, 0> claims{};

  explicit GPIO(Swarm::Assembly) {}
  ~GPIO() = default;

  GPIO(const GPIO &) = delete;
  void operator=(const GPIO &) = delete;

  GPIO(GPIO &&) = default;
  GPIO &operator=(GPIO &&) = default;

  void set_direction(HAL::Pin pin, HAL::PinMode mode) {
    gpio_set_direction(static_cast<gpio_num_t>(pin.number()),
                       mode == HAL::PinMode::Output ? GPIO_MODE_OUTPUT
                                                    : GPIO_MODE_INPUT);
  }
  void set_level(HAL::Pin pin, HAL::Voltage level) {
    gpio_set_level(static_cast<gpio_num_t>(pin.number()), HAL::to_level(level));
  }
  void delay_ms(int ms) {
    // a nonzero delay must be at least one tick, else pdMS_TO_TICKS rounds sub-
    // tick values to 0 -> vTaskDelay(0) never yields -> idle task starves
    TickType_t ticks = pdMS_TO_TICKS(ms);
    if (ms > 0 && ticks == 0) {
      ticks = 1;
    }
    vTaskDelay(ticks);
  }
};

class PWM {
public:
  // the constructor configures one LEDC timer for the whole channel group, so
  // a second PWM wanting a different frequency would silently retune the first
  static constexpr int timer = 0;
  static constexpr uint32_t frequency_hz = 1000;

  static constexpr std::array claims{
      HAL::Claim{HAL::Resource::LedcTimer, timer, HAL::Use::Shared},
  };

  PWM(Swarm::Assembly);

  PWM(const PWM &) = delete;
  void operator=(const PWM &) = delete;

  PWM(PWM &&) = default;
  PWM &operator=(PWM &&) = default;

  void configure_channel(int channel, HAL::Pin pin);
  void set_duty_ratio(int channel, double ratio);
};
} // namespace ESP32

#endif
