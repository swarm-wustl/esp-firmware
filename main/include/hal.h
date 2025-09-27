#ifndef HAL_H
#define HAL_H

#include <array>
#include <type_traits>
#include <cstdint>

#include <geometry_msgs/msg/twist.h>

#include "motor.h"
#include "drive.h"

/*
The design philosophy implemented here is as follows:
- Define abstract types: Motor, etc.
- Use the abstract types to define HAL types
- Use the HAL-defined traits and types to implement a specific driver (e.g., ESP32 Motor Driver)
- Create a global instance (or instances) of the specific driver in the hardware namespace
- Use the global instances throughout application logic (e.g., Consumer task)
*/

/*
HAL Interface
The traits and enums defined in this namespace can be used to create drivers for any board, i.e., ESP32.
*/
namespace HAL {
    enum class Voltage : uint32_t {
        HIGH = 1,
        LOW = 0
    };

    constexpr uint32_t to_level(HAL::Voltage voltage) {
        return static_cast<uint32_t>(voltage);
    }

    template <typename MotorDriver>
    concept MotorDriverTrait = requires(
        MotorDriver driver,
        Motor::Command cmd
    ) {
        { driver.run(cmd) } -> std::same_as<void>;
        { driver.stop() } -> std::same_as<void>;
    };

    template <typename DriveStyle, size_t MotorCount>
    concept DriveStyleTrait = requires (
        geometry_msgs__msg__Twist twist_msg
    ) {
        { DriveStyle::type() } -> std::same_as<Drive::Type>;
        { DriveStyle::template convert_twist<MotorCount>(twist_msg) } -> std::same_as<std::array<Motor::Command, MotorCount>>;
    };
}

#endif