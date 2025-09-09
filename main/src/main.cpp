#include "consumer.h"
#include "hardware.h"

// TODO: make templated and move to consumer.h?
static void consumerTaskWrapper(void* pvParameters) {
    HW::MotorDriver driver = *reinterpret_cast<HW::MotorDriver*>(pvParameters);
    Consumer::spin(driver);
    vTaskDelete(nullptr); // delete task when done
}

/*
Main Function
Describe the physical layout of the system.
For example, you could have multiple motor drivers, sensors, etc.
The types used should only be taken from hardware.h's defintions.
*/
extern "C" void app_main(void) {
    HW::MotorDriver motor_driver;

    log("Hello world!");

    xTaskCreate(
        consumerTaskWrapper,
        "consumer_task",
        2048,
        (void*)&motor_driver,
        configMAX_PRIORITIES - 1,
        NULL
    );
}
