#ifndef ESP32_H
#define ESP32_H

// TODO: move this entire file to a component maybe

#include "hal.h"

namespace ESP32 {
    class L298NMotorDriver {
    public:
        // TODO
        L298NMotorDriver() {}
        
        void run(Motor::Command cmd);
        void stop();
    };

    class DifferentialDriveController {
    public:
        DifferentialDriveController() = delete;

        // TODO
        static void convert_twist() {}
    };
}

#endif
