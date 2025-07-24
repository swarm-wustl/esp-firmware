#ifndef HARDWARE_H
#define HARDWARE_H

#include "esp32.h"

/*
Hardware Namespace
The extern variables declared here represent the physical hardware of the system.
This namespace should only rely on board-specific implementation types (e.g., ESP32).
*/
namespace HW {
    extern ESP32::MotorDriver           motor_driver;
    extern ESP32::DifferentialDrive     drive_style;
}

#endif