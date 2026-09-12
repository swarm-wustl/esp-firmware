#include "consumer.h"
#include "dwm.h"
#include "esp32.h"
#include "ros.h"
#include "i2c.h"

#include "esp_log.h"

#include <uros_network_interfaces.h>

#include "freertos/FreeRTOS.h"
#include <optional>
#include <utility>
#include <memory>
#include <chrono>
#include <thread>

static const char *TAG = "main";

namespace HW {
constexpr size_t MOTOR_COUNT = 2;

using DriveStyle = ESP32::DifferentialDriveController;
using MotorDriver = ESP32::L298NMotorDriver;
using SPI = ESP32::SPI;
using GPIO = ESP32::GPIO;

static_assert(HAL::MotorDriverTrait<MotorDriver>);
static_assert(HAL::DriveStyleTrait<DriveStyle, MOTOR_COUNT>);
} // namespace HW

// TODO: make templated and move to consumer.h?
// TODO: make struct so we can pass multiple parameters

struct ConsumerTaskData {
  HW::MotorDriver motorDriver;
  Consumer::QueueType queue;
};

static void consumerTaskWrapper(void *pvParameters) {
  ConsumerTaskData *data = reinterpret_cast<ConsumerTaskData *>(pvParameters);

  Consumer::spin(data->motorDriver, data->queue);

  vTaskDelete(nullptr);
}

static void onTwist(const geometry_msgs__msg__Twist &twist,
                    Consumer::QueueType &queue) {
  std::array<Motor::Command, HW::MOTOR_COUNT> motor_commands =
      HW::DriveStyle::convert_twist<HW::MOTOR_COUNT>(twist);

  for (Motor::Command cmd : motor_commands) {
    if (!queue.push(Consumer::MessageTag::MOTOR_COMMAND,
                    Consumer::MessageBody{.motor_cmd = cmd})) {
      ESP_LOGE(TAG, "Dropped motor command: consumer queue full");
    }
  }
}

static void rosTaskWrapper(void *pvParameters) {
  Consumer::QueueType *queue =
      reinterpret_cast<Consumer::QueueType *>(pvParameters);

  ROS::spin(*queue, onTwist);

  vTaskDelete(nullptr);
}

extern "C" void app_main(void) {
//   ESP_LOGI(TAG, "Testing UWB");
//   ESP_LOGI(TAG, "FreeRTOS tick: %d Hz", CONFIG_FREERTOS_HZ);

//   HW::SPI spi{GPIO_NUM_4}; // TODO: put pins in a config somewhere
//   HW::GPIO gpio{};
//   DWM dwm_sensor{std::move(spi), std::move(gpio), GPIO_NUM_27, GPIO_NUM_34};

//   // bring-up: flip to false on the responder board
//   constexpr bool kInitiator = true;

//   if (auto id = dwm_sensor.get_device_id()) {
//     ESP_LOGI(TAG, "DW1000 id: 0x%08lX", static_cast<unsigned long>(*id));
//   } else {
//     ESP_LOGE(TAG, "DW1000 id read failed");
//   }

//   if (auto r = dwm_sensor.configure(); r) {
//     ESP_LOGI(TAG, "DW1000 configured");
//   } else {
//     ESP_LOGE(TAG, "DW1000 configure failed");
//   }

//   while (true) {
//     if constexpr (kInitiator) {
//       if (auto d = dwm_sensor.range()) {
//         ESP_LOGI(TAG, "range: %d cm", static_cast<int>(*d * 100.0));
//       } else {
//         ESP_LOGE(TAG, "range failed");
//       }
//       vTaskDelay(pdMS_TO_TICKS(200));
//     } else {
//       auto r = dwm_sensor.respond();
//       ESP_LOGI(TAG, "respond: %s", r ? "ok" : "fail");
//       vTaskDelay(pdMS_TO_TICKS(10));
//     }
//   }
// #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) ||                                \
//     defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
//   ESP_ERROR_CHECK(uros_network_interface_initialize());
// #endif

//   std::optional<Consumer::QueueType> queue = Consumer::QueueType::create();

//   if (!queue) {
//     ESP_LOGE(TAG, "Unable to create consumer queue");
//     return;
//   }

//   // Make the struct static so it lives as long as the program (incase mani()
//   // ever terminates)
//   static ConsumerTaskData consumerTaskData{HW::MotorDriver{},
//                                            std::move(*queue)};

//   ESP_LOGI(TAG, "Hello world!");

//   xTaskCreate(
//       rosTaskWrapper, "uros_task",
//       4096, // TODO: see
//             // https://github.com/micro-ROS/micro_ros_espidf_component/blob/cd1da2b3d7d73f48743a2c42ac0e915cd751bb74/examples/int32_publisher/main/main.c#L105
//       (void *)&consumerTaskData.queue, configMAX_PRIORITIES - 1, NULL);

//   xTaskCreate(consumerTaskWrapper, "consumer_task", 4096,
//               (void *)&consumerTaskData, configMAX_PRIORITIES - 1, NULL);

  log("Testing I2C");
   uint8_t data[2];
   ESP_ERROR_CHECK(i2c_master_init());


  
   ESP_LOGI(TAG, "I2C initialized successfully");


   //turn off sleep mode
   ESP_ERROR_CHECK(imu_register_write_byte(IMU_PWR_MGMT_1, 0x00));
   ESP_LOGI(TAG, "MPU6050 awakened");


   // give sensor time to stabilize
   vTaskDelay(pdMS_TO_TICKS(100));


   /* Read the MPU6050 WHO_AM_I register, on power up the register should have the value 0x71 */
   ESP_ERROR_CHECK(mpu6050_register_read(IMU_WHO_AM_I_ADDR , data, 1));
   ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);


   int16_t gyroY;
   int16_t gyroZ;




   int16_t accelX;
   int16_t accelY;
   int16_t accelZ;


   while (1){
       ESP_ERROR_CHECK(imu_read_gyroscope_data(&gyroX, &gyroY, &gyroZ));


       // read in angles per second
       ESP_LOGI(TAG, "GYROX = %.4f", ((float)gyroX)/LSBSENS_GYRO);
       ESP_LOGI(TAG, "GYROY = %.4f", ((float)gyroY)/LSBSENS_GYRO);
       ESP_LOGI(TAG, "GYROZ = %.4f", ((float)gyroZ)/LSBSENS_GYRO);
       ESP_LOGI(TAG, "Gyroscope read successfully");


       ESP_ERROR_CHECK(imu_read_accelerometer_data(&accelX, &accelY, &accelZ));


       //read in g (9.81 m/s^2)
       ESP_LOGI(TAG, "ACCELX = %.4f", ((float)accelX)/LSBSENS_ACCEL);
       ESP_LOGI(TAG, "ACCELY = %.4f", ((float)accelY)/LSBSENS_ACCEL);
       ESP_LOGI(TAG, "ACCELZ = %.4f", ((float)accelZ)/LSBSENS_ACCEL);
       ESP_LOGI(TAG, "Accelerometer read successfully");
      
       vTaskDelay(pdMS_TO_TICKS(100));
    }
}
