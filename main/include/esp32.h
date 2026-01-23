#ifndef ESP32_H
#define ESP32_H

#include <geometry_msgs/msg/twist.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>

#include "hal.h"
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
}

#endif