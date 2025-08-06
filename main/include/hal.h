#ifndef HAL_H
#define HAL_H

#include <array>
#include <type_traits>

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
    enum class Voltage {
        HIGH = 1,
        LOW = 0
    };

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
        std::array<Motor::Command, MotorCount> cmd_list
    ) {
        { DriveStyle::type() } -> std::same_as<Drive::Type>;
        // TODO: actually take twist message and return proper type
        { DriveStyle::convert_twist(cmd_list) } -> std::same_as<void>;
    };
}

#endif