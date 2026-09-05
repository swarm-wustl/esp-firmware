#include "consumer.h"
#include "dwm.h"
#include "esp32.h"
#include "ros.h"

#include <uros_network_interfaces.h>

#include "freertos/FreeRTOS.h"
#include <memory>

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
    queue.pushToQueue(Consumer::MessageTag::MOTOR_COMMAND,
                      Consumer::MessageBody{.motor_cmd = cmd});
  }
}

static void rosTaskWrapper(void *pvParameters) {
  Consumer::QueueType *queue =
      reinterpret_cast<Consumer::QueueType *>(pvParameters);

  ROS::spin(*queue, onTwist);

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
  log("FreeRTOS tick: %d Hz", CONFIG_FREERTOS_HZ);

  HW::SPI spi{GPIO_NUM_4}; // TODO: put pins in a config somewhere
  HW::GPIO gpio{};
  DWM dwm_sensor{std::move(spi), std::move(gpio), GPIO_NUM_27, GPIO_NUM_34};

  // bring-up: flip to false on the responder board
  constexpr bool kInitiator = true;

  if (auto id = dwm_sensor.get_device_id()) {
    log("DW1000 id: 0x%08lX", static_cast<unsigned long>(*id));
  } else {
    log("DW1000 id read failed");
  }

  if (auto r = dwm_sensor.configure(); r) {
    log("DW1000 configured");
  } else {
    log("DW1000 configure failed");
  }

  while (true) {
    if constexpr (kInitiator) {
      if (auto d = dwm_sensor.range()) {
        log("range: %d cm", static_cast<int>(*d * 100.0));
      } else {
        log("range failed");
      }
      vTaskDelay(pdMS_TO_TICKS(200));
    } else {
      auto r = dwm_sensor.respond();
      log("respond: %s", r ? "ok" : "fail");
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) ||                                \
    defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
  ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

  // Make the struct static so it lives as long as the program (incase mani()
  // ever terminates)
  static ConsumerTaskData consumerTaskData{HW::MotorDriver{},
                                           Consumer::QueueType{}};

  log("Hello world!");

  xTaskCreate(
      rosTaskWrapper, "uros_task",
      4096, // TODO: see
            // https://github.com/micro-ROS/micro_ros_espidf_component/blob/cd1da2b3d7d73f48743a2c42ac0e915cd751bb74/examples/int32_publisher/main/main.c#L105
      (void *)&consumerTaskData.queue, configMAX_PRIORITIES - 1, NULL);

  xTaskCreate(consumerTaskWrapper, "consumer_task", 4096,
              (void *)&consumerTaskData, configMAX_PRIORITIES - 1, NULL);
}
