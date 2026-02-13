#include "consumer.h"
#include "hardware.h"
#include "queue.h"
#include "ros.h"
#include "sensor.h"
#include "i2c.cpp"

#include "freertos/FreeRTOS.h"
#include <memory>

// TODO: make templated and move to consumer.h?
// TODO: make struct so we can pass multiple parameters

// struct ConsumerTaskData {
//     HW::MotorDriver motorDriver;
//     Consumer::QueueType queue; 
// };

// static void consumerTaskWrapper(void* pvParameters) {
//     ConsumerTaskData* data = reinterpret_cast<ConsumerTaskData*>(pvParameters);

//     Consumer::spin(
//         data->motorDriver,
//         data->queue
//     );

//     vTaskDelete(nullptr);
// }

// static void rosTaskWrapper(void* pvParameters) {
//     Consumer::QueueType* queue = reinterpret_cast<Consumer::QueueType*>(pvParameters);

//     ROS::spin(*queue);

//     vTaskDelete(nullptr);
// }

/*
Main Function
Describe the physical layout of the system.
For example, you could have multiple motor drivers, sensors, etc.
The types used should only be taken from hardware.h's defintions.
*/
extern "C" void app_main(void) {
// #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
//     ESP_ERROR_CHECK(uros_network_interface_initialize());
// #endif

//     // Make the struct static so it lives as long as the program (incase mani() ever terminates)
//     static ConsumerTaskData consumerTaskData {
//         HW::MotorDriver{},
//         Consumer::QueueType{}
//     };

//     log("Hello world!");

//     xTaskCreate(
//         rosTaskWrapper,
//         "uros_task",
//         4096, // TODO: see https://github.com/micro-ROS/micro_ros_espidf_component/blob/cd1da2b3d7d73f48743a2c42ac0e915cd751bb74/examples/int32_publisher/main/main.c#L105
//         (void*)&consumerTaskData.queue,
//         configMAX_PRIORITIES - 1,
//         NULL
//     );

//     xTaskCreate(
//         consumerTaskWrapper,
//         "consumer_task",
//         4096,
//         (void*)&consumerTaskData,
//         configMAX_PRIORITIES - 1,
//         NULL
//     );

    // Create sensor task and register the task handle for the timers
    // TODO: wrap this in a function?
    /*TaskHandle_t sensorTaskHandle;
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
    Sensor uwb(UWB_ID, "uwb_sensor", 1000);*/

    log("Testing I2C");
    uint8_t data[2];
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    /* Read the MPU6050 WHO_AM_I register, on power up the register should have the value 0x71 */
    ESP_ERROR_CHECK(mpu9250_register_read(IMU_WHO_AM_I_ADDR , data, 1));
    ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);
    ESP_LOGI(TAG, "I2C read successfully");

    /* Demonstrate writing by reseting the MPU6050 */
    ESP_ERROR_CHECK(imu_register_write_byte(IMU_PWR_MGMT_1, 1 << IMU_PWR_MGMT_1_RESET_BIT));
    ESP_LOGI(TAG, "I2C written successfully");

    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C unitialized successfully");
}