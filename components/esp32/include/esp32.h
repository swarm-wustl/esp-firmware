#ifndef ESP32_H
#define ESP32_H

#include <memory>

#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include <geometry_msgs/msg/twist.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "log.h"
#include "swarm_hal.h"

namespace ESP32 {
// TODO: move this outside of ESP32
class L298NMotorDriver {
public:
  L298NMotorDriver();

  L298NMotorDriver(const L298NMotorDriver &) = delete;
  L298NMotorDriver &operator=(const L298NMotorDriver &) = delete;

  L298NMotorDriver(L298NMotorDriver &&) = default;
  L298NMotorDriver &operator=(L298NMotorDriver &&) = default;

  void run(const Motor::Command &cmd);
  void stop();
};

// TODO: move this outside of ESP32
class DifferentialDriveController {
public:
  DifferentialDriveController() = delete;

  static Drive::Type type() { return Drive::Type::DIFFERENTIAL; }

  template <size_t MotorCount>
  static std::array<Motor::Command, MotorCount>
  convert_twist(geometry_msgs__msg__Twist msg);
};

class SPI {
public:
  SPI(int cs);
  ~SPI();

  SPI(const SPI &) = delete;
  void operator=(const SPI &) = delete;

  SPI(SPI &&other);
  SPI &operator=(SPI &&other);

  std::expected<void, HAL::SpiError>
  transfer_halfduplex(std::span<const std::byte> tx, std::span<std::byte> rx);

private:
  int cs_{};
  bool owns_spi_line{};
  spi_device_handle_t dev_handle_{};

  void swap(SPI &other);
};

class GPIO {
public:
  GPIO() = default;
  ~GPIO() = default;

  GPIO(const GPIO &) = delete;
  void operator=(const GPIO &) = delete;

  GPIO(GPIO &&) = default;
  GPIO &operator=(GPIO &&) = default;

  void set_direction(int pin, HAL::PinMode mode) {
    gpio_set_direction(static_cast<gpio_num_t>(pin),
                       mode == HAL::PinMode::Output ? GPIO_MODE_OUTPUT
                                                    : GPIO_MODE_INPUT);
  }
  void set_level(int pin, HAL::Voltage level) {
    gpio_set_level(static_cast<gpio_num_t>(pin), HAL::to_level(level));
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
} // namespace ESP32

#endif
