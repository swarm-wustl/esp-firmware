#ifndef ESP32_H
#define ESP32_H

#include <array>
#include <memory>
#include <tuple>

#include <driver/i2c_master.h>
#include <driver/pulse_cnt.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <expected>
#include <span>

#include "assembly.h"
#include "driver/gpio.h"
#include "encoder.h"
#include "swarm_hal.h"

namespace ESP32 {
using HAL::operator""_p;

// the bus owns the peripheral: spi_bus_initialize/free happen once here, and
// devices attach to it. SPI2_HOST is Exclusive -- a second initialize on the
// same host fails at runtime and only gets logged
class SpiBus {
public:
  static constexpr spi_host_device_t host = SPI2_HOST;
  static constexpr int sck = 18;
  // the carrier wires MISO to 19 and MOSI to 23, which NEWMCPCBV2 gives to M3's
  // PWM and the UMC's BIN2. 12 and 14 are free and wired to nothing, which is
  // what keeps the bus declarable while the ranging tasks are unspawned -- see
  // the TODO in main.cpp
  static constexpr int miso = 12;
  static constexpr int mosi = 14;

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

// a device on a bus. The chip select arrives as a HAL::Pin, which can only be
// minted from a declaration -- so it is claimed by whoever declared it
class SPI {
public:
  static constexpr std::array<HAL::Claim, 0> claims{};

  SPI(Swarm::Assembly, SpiBus &bus, HAL::Pin cs);
  ~SPI();

  SPI(const SPI &) = delete;
  void operator=(const SPI &) = delete;

  SPI(SPI &&other);
  SPI &operator=(SPI &&other);

  std::expected<void, HAL::SpiError>
  transfer_halfduplex(std::span<const std::byte> tx, std::span<std::byte> rx);

private:
  int cs_{};
  spi_device_handle_t dev_handle_{};

  void swap(SPI &other);
};

namespace detail {
// the IDF calls live in i2c.cpp; the templates below only carry the
// description, so nothing drags driver internals through the pin tables
i2c_master_bus_handle_t i2c_open_bus(int port, int sda, int scl);
void i2c_close_bus(i2c_master_bus_handle_t bus);

i2c_master_dev_handle_t i2c_add_device(i2c_master_bus_handle_t bus,
                                       uint8_t address, uint32_t speed_hz);
void i2c_remove_device(i2c_master_dev_handle_t device);

std::expected<void, HAL::I2cError> i2c_write(i2c_master_dev_handle_t device,
                                             std::span<const std::byte> tx,
                                             int timeout_ms);
std::expected<void, HAL::I2cError>
i2c_write_read(i2c_master_dev_handle_t device, std::span<const std::byte> tx,
               std::span<std::byte> rx, int timeout_ms);
} // namespace detail

// one master port for every device on the two wires, same shape as SpiBus:
// the port is Exclusive, SDA and SCL are Shared because that is the point of
// a bus
template <auto Pins, int Port = 0> class I2cBus {
public:
  static constexpr int port = Port;

  static constexpr auto claims =
      HAL::concat(Pins.claims(),
                  std::array{HAL::Claim{HAL::Resource::I2cPort, Port,
                                        HAL::Use::Exclusive}});

  explicit I2cBus(Swarm::Assembly)
      : handle_{detail::i2c_open_bus(Port, Pins["sda"_p].number(),
                                     Pins["scl"_p].number())} {}

  ~I2cBus() { detail::i2c_close_bus(handle_); }

  I2cBus(const I2cBus &) = delete;
  void operator=(const I2cBus &) = delete;
  I2cBus(I2cBus &&) = delete;
  void operator=(I2cBus &&) = delete;

  i2c_master_bus_handle_t handle() const { return handle_; }

private:
  i2c_master_bus_handle_t handle_{};
};

template <typename Bus> class I2C {
public:
  static constexpr std::array<HAL::Claim, 0> claims{};

  // 400 kHz fast mode: every part on the UMC bus supports it, and the MPU6050
  // tolerates it on its 1 MHz-capable lines
  static constexpr uint32_t speed_hz = 400'000;
  static constexpr int timeout_ms = 1000;

  I2C(Swarm::Assembly, Bus &bus, uint8_t address)
      : device_{detail::i2c_add_device(bus.handle(), address, speed_hz)} {}

  ~I2C() { detail::i2c_remove_device(device_); }

  I2C(const I2C &) = delete;
  void operator=(const I2C &) = delete;
  I2C(I2C &&) = delete;
  void operator=(I2C &&) = delete;

  std::expected<void, HAL::I2cError> write(std::span<const std::byte> tx) {
    return detail::i2c_write(device_, tx, timeout_ms);
  }

  std::expected<void, HAL::I2cError> write_read(std::span<const std::byte> tx,
                                                std::span<std::byte> rx) {
    return detail::i2c_write_read(device_, tx, rx, timeout_ms);
  }

private:
  i2c_master_dev_handle_t device_{};
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

  // above audible and well inside the TB6612's 100 kHz ceiling. 10-bit duty at
  // this rate needs 20.48 MHz of the 80 MHz LEDC clock, so the resolution holds
  static constexpr uint32_t frequency_hz = 20000;

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

// PCNT in quadrature: two channels per unit, each counting both edges of one
// signal with the other as the level input, so every edge of the pair moves the
// count and its sign is the direction
class QuadratureCounter {
public:
  static constexpr auto decoding = Encoder::Decoding::X4;

  // the units are claimed by whoever declares the encoder rows, so this holds
  // no hardware of its own
  static constexpr std::array<HAL::Claim, 0> claims{};

  static constexpr size_t max_units = 8;

  // the hardware counter is 16-bit, so the driver accumulates at these limits
  // and the running 32-bit total is what gets read back
  static constexpr int low_limit = -32768;
  static constexpr int high_limit = 32767;

  // a hall edge can chatter; anything shorter than this is not motion. 1us
  // allows ~170 output rev/s at X4, far past the 1.25 rev/s this gearbox turns
  static constexpr uint32_t glitch_filter_ns = 1000;

  explicit QuadratureCounter(Swarm::Assembly);
  ~QuadratureCounter();

  QuadratureCounter(const QuadratureCounter &) = delete;
  void operator=(const QuadratureCounter &) = delete;

  QuadratureCounter(QuadratureCounter &&other);
  QuadratureCounter &operator=(QuadratureCounter &&other);

  void configure_unit(int unit, HAL::Pin a, HAL::Pin b);
  int32_t count(int unit);
  void clear(int unit);

private:
  std::array<pcnt_unit_handle_t, max_units> units_{};

  void swap(QuadratureCounter &other);
};
} // namespace ESP32

#endif
