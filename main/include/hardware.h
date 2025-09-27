#ifndef HARDWARE_H
#define HARDWARE_H

#include "esp32.h"

/*
Hardware Namespace
The types declared here represent the physical hardware of the system.
This namespace should only rely on board-specific implementation types (e.g., ESP32).
*/
namespace HW {
    constexpr size_t MOTOR_COUNT = 2;

    using DriveStyle = ESP32::DifferentialDriveController;
    using MotorDriver = ESP32::L298NMotorDriver;
}

#endif