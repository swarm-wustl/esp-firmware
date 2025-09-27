#ifndef ESP32_H
#define ESP32_H

// TODO: move this entire file to a component maybe (like microros is)

#include <geometry_msgs/msg/twist.h>

#include "hal.h"

namespace ESP32 {
    class L298NMotorDriver {
    public:
        L298NMotorDriver();
        
        void run(const Motor::Command& cmd);
        void stop();
    };

    class DifferentialDriveController {
    public:
        DifferentialDriveController() = delete;

        static Drive::Type type() {
            return Drive::Type::DIFFERENTIAL;
        }

        template <size_t MotorCount>
        static std::array<Motor::Command, MotorCount> convert_twist(geometry_msgs__msg__Twist msg);
    };
}

#endif