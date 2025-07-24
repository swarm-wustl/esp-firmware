#ifndef HAL_H
#define HAL_H

#include <type_traits>

#include "motor.h"

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

    template <typename T>
    using MotorDriverTrait = std::conjunction<
        std::is_invocable_r<void, decltype(&T::run), T&, Motor::Command>,
        std::is_invocable_r<void, decltype(&T::stop), T&>
    >;

    // TODO: actually take twist message
    template <typename T>
    using DriveStyleTrait = std::conjunction<
        std::negation<std::is_constructible<T>>,
        std::is_invocable_r<void, decltype(&T::convert_twist)>
    >;
}

#endif