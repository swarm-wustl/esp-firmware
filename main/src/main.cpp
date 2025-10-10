#include "consumer.h"
#include "hardware.h"
#include "queue.h"
#include "ros.h"
#include "sensor.h"

#include "freertos/FreeRTOS.h"

// TODO: make templated and move to consumer.h?
// TODO: make struct so we can pass multiple parameters

struct ConsumerTaskData {
    HW::MotorDriver motorDriver;
    Queue<Consumer::MessageTag, Consumer::MessageBody, Consumer::CONSUMER_QUEUE_SIZE> queue; 
};

static void consumerTaskWrapper(void* pvParameters) {
    ConsumerTaskData data = *reinterpret_cast<ConsumerTaskData*>(pvParameters);

    Consumer::spin(
        std::move(data.motorDriver),
        std::move(data.queue)
    );

    vTaskDelete(nullptr);
}

/*
Main Function
Describe the physical layout of the system.
For example, you could have multiple motor drivers, sensors, etc.
The types used should only be taken from hardware.h's defintions.
*/
extern "C" void app_main(void) {
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    /*HW::MotorDriver motor_driver;
    Queue<Consumer::MessageTag, Consumer::MessageBody, Consumer::CONSUMER_QUEUE_SIZE> queue;

    ConsumerTaskData consumerTaskData {
        motor_driver,
        queue
    };

    log("Hello world!");

    xTaskCreate(
        ROS::spin,
        "uros_task",
        4096, // TODO: see https://github.com/micro-ROS/micro_ros_espidf_component/blob/cd1da2b3d7d73f48743a2c42ac0e915cd751bb74/examples/int32_publisher/main/main.c#L105
        (void*)&queue,
        configMAX_PRIORITIES - 1,
        NULL
    );

    xTaskCreate(
        consumerTaskWrapper,
        "consumer_task",
        4096,
        (void*)&consumerTaskData,
        configMAX_PRIORITIES - 1,
        NULL
    );*/

    // Create sensor task and register the task handle for the timers
    // TODO: wrap this in a function?
    TaskHandle_t sensorTaskHandle;
    xTaskCreate(
        Sensor::spin,
        "sensors_task",
        4096,
        NULL,
        configMAX_PRIORITIES - 1,
        &sensorTaskHandle
    );
    Sensor::registerSensorTaskHandle(sensorTaskHandle);
    log("Registered handle");

    // TODO: make constant time
    Sensor uwb(UWB_ID, "uwb_sensor", 1000);
}
