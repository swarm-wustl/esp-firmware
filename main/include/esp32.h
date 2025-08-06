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

    class DriveController {
    public:
        DriveController() = delete;

        static Drive::Type type() {
            return Drive::Type::DIFFERENTIAL;
        }

        template <size_t MotorCount>
        static void convert_twist(std::array<Motor::Command, MotorCount> cmd_list);
    };


    // // TODO
    // template <>
    // void DriveController::convert_twist(std::array<Motor::Command, 4> cmd_list) {}
}

#endif
