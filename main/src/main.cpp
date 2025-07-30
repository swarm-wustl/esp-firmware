#include "consumer.h"
#include "hardware.h"

/*
Main Function
Describe the physical layout of the system.
For example, you could have multiple motor drivers, sensors, etc.
The types used should only be taken from hardware.h's defintions.
*/
extern "C" void app_main(void) {
    HW::MotorDriver motor_driver;

    Consumer::spin<HW::MotorDriver, HW::DriveStyle>(motor_driver);
}
