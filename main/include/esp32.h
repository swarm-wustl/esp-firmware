#ifndef ESP32_H
#define ESP32_H

// TODO: move this entire file to a component maybe

#include "hal.h"

namespace ESP32 {
    class MotorDriver {
    public:
        MotorDriver();
        
        void run(Motor::Command cmd);
        void stop();
    };

    class DifferentialDrive {
    public:
        static void convert_twist();
    };
}

static_assert(HAL::MotorDriverTrait<ESP32::MotorDriver>::value, "Invalid motor driver");
static_assert(HAL::DriveStyleTrait<ESP32::DifferentialDrive>::value, "Invalid drive style");


#endif