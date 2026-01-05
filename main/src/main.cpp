#include "consumer.h"
#include "hardware.h"
#include "queue.h"
#include "ros.h"
#include "sensor.h"
#include "dwm.h"

#include "freertos/FreeRTOS.h"
#include <memory>

// TODO: make templated and move to consumer.h?
// TODO: make struct so we can pass multiple parameters

struct ConsumerTaskData {
    HW::MotorDriver motorDriver;
    Consumer::QueueType queue; 
};

static void consumerTaskWrapper(void* pvParameters) {
    ConsumerTaskData* data = reinterpret_cast<ConsumerTaskData*>(pvParameters);

    Consumer::spin(
        data->motorDriver,
        data->queue
    );

    vTaskDelete(nullptr);
}

static void rosTaskWrapper(void* pvParameters) {
    Consumer::QueueType* queue = reinterpret_cast<Consumer::QueueType*>(pvParameters);

    ROS::spin(*queue);

    vTaskDelete(nullptr);
}

/*
Main Function
Describe the physical layout of the system.
For example, you could have multiple motor drivers, sensors, etc.
The types used should only be taken from hardware.h's defintions.
*/
extern "C" void app_main(void) {
    log("Testing UWB");

    HW::SPI spi{4}; // TODO: put CS pin in a config somewhere
    DWM dwm_sensor{std::move(spi)};

    while (true) {}
}
