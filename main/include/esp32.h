#ifndef ESP32_H
#define ESP32_H

#include <geometry_msgs/msg/twist.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "swarm_hal.h"
#include "driver/gpio.h"

namespace ESP32 {
	// TODO: move this outside of ESP32
    class L298NMotorDriver {
    public:
        L298NMotorDriver();

        L298NMotorDriver(const L298NMotorDriver&) = delete;
        L298NMotorDriver& operator=(const L298NMotorDriver&) = delete;

        L298NMotorDriver(L298NMotorDriver&&) = default;
        L298NMotorDriver& operator=(L298NMotorDriver&&) = default;
        
        void run(const Motor::Command& cmd);
        void stop();
    };

    // TODO: move this outside of ESP32
    class DifferentialDriveController {
    public:
        DifferentialDriveController() = delete;

        static Drive::Type type() {
            return Drive::Type::DIFFERENTIAL;
        }

        template <size_t MotorCount>
        static std::array<Motor::Command, MotorCount> convert_twist(geometry_msgs__msg__Twist msg);

        static std::pair<double, double> calculate_drive_targets(double linear_x, double angular_z);
        static Motor::Command create_command(Motor::Name name, double value);
    };

    class SPI {
    public:
        SPI(int cs);
        ~SPI();

        SPI(const SPI&) = delete;
        void operator=(const SPI&) = delete;

        SPI(SPI&&) = default;
        SPI& operator=(SPI&&) = default;

        esp_err_t transfer_halfduplex(std::span<const std::byte> tx, std::span<std::byte> rx);

    private:
        int cs_{};
        spi_device_handle_t dev_handle_{};
    };

    class GPIO {
    public:
        GPIO() = default;
        ~GPIO() = default;

        GPIO(const GPIO&) = delete;
        void operator=(const GPIO&) = delete;

        GPIO(GPIO&&) = default;
        GPIO& operator=(GPIO&&) = default;

        void set_direction(gpio_num_t pin, gpio_mode_t mode) { gpio_set_direction(pin, mode); }
        void set_level(gpio_num_t pin, HAL::Voltage level) { gpio_set_level(pin, HAL::to_level(level)); }
        void delay_ms(int ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }
    };
} // namespace ESP32

#endif
