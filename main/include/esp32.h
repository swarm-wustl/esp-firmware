#ifndef ESP32_H
#define ESP32_H

// TODO: move this entire file to a component maybe (like microros is)

#include <geometry_msgs/msg/twist.h>
#include "driver/i2c_master.h"

#include "hal.h"

namespace ESP32 {
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

class DifferentialDriveController {
public:
  DifferentialDriveController() = delete;

  static Drive::Type type() { return Drive::Type::DIFFERENTIAL; }

  template <size_t MotorCount>

  static std::array<Motor::Command, MotorCount>
  convert_twist(geometry_msgs__msg__Twist msg);

  static std::pair<double, double> calculate_drive_targets(double linear_x,
                                                           double angular_z);
  static Motor::Command create_command(Motor::Name name, double value);
};

class MPU6050Driver {
private:
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    bool initialized;

    // Defines pins
    static constexpr gpio_num_t I2C_SDA_PIN = GPIO_NUM_15;
    static constexpr gpio_num_t I2C_SCL_PIN = GPIO_NUM_14;
    static constexpr uint8_t MPU6050_ADDR = 0x68;

public:
    MPU6050Driver();
    bool initialize();
    HAL::IMUData read();
};
} // namespace ESP32

#endif
