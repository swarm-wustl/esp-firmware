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

    // TODO: create a HAL::ESP32 that handles microcontroller-specific interactions
    // Then, anything like a MotorDriver, UWB sensor, etc. uses that HAL.
    // Adds another layer of abstraction and makes it much easier to implement a motor driver and swap out the hardware
    // Additionally, keeps it easy to create a new driver
    using DriveStyle = ESP32::DifferentialDriveController;
    using MotorDriver = ESP32::L298NMotorDriver;
}

#endif