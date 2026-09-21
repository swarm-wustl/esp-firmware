#include "consumer.h"
#include "differential_drive.h"
#include "dwm.h"
#include "esp32.h"
#include "l298n.h"
#include "ros.h"
#include "system.h"

#include "esp_log.h"

#include <uros_network_interfaces.h>

#include "freertos/FreeRTOS.h"
#include <algorithm>
#include <optional>
#include <utility>

static const char *TAG = "main";

namespace HW {
using SPI = ESP32::SPI;
using SpiBus = ESP32::SpiBus;
using GPIO = ESP32::GPIO;
using PWM = ESP32::PWM;

constexpr auto MOTOR_PINS =
    L298N::motors(L298N::MotorPins{Motor::Name::LEFT, GPIO_NUM_16, GPIO_NUM_17,
                                   GPIO_NUM_25, 0, GPIO_NUM_0},
                  L298N::MotorPins{Motor::Name::RIGHT, GPIO_NUM_32, GPIO_NUM_33,
                                   GPIO_NUM_5, 1, GPIO_NUM_0});

using MotorDriver = L298N::MotorDriver<GPIO, PWM, MOTOR_PINS>;

constexpr auto DWM_PINS = HAL::pins(HAL::NamedPin{"cs", GPIO_NUM_4},
                                    HAL::NamedPin{"reset", GPIO_NUM_27},
                                    HAL::NamedPin{"irq", GPIO_NUM_34});

using Dwm = DWM<SPI, GPIO, DWM_PINS>;

using Chassis =
    Swarm::chassis<Drive::Style::DIFFERENTIAL, MotorDriver, SpiBus, Dwm>;

constexpr Drive::Style DRIVE_STYLE = Chassis::style;

using QueueType = Consumer::QueueType<DRIVE_STYLE>;
} // namespace HW

// TODO: make templated and move to consumer.h?
struct ConsumerTaskData {
  HW::MotorDriver motorDriver;
  HW::QueueType queue;
};

static void consumerTaskWrapper(void *pvParameters) {
  ConsumerTaskData *data = reinterpret_cast<ConsumerTaskData *>(pvParameters);

  Consumer::spin(data->motorDriver, data->queue);

  vTaskDelete(nullptr);
}

static void onTwist(const geometry_msgs__msg__Twist &twist, void *context) {
  HW::QueueType &queue = *reinterpret_cast<HW::QueueType *>(context);

  const Drive::Twist body_twist{twist.linear.x, twist.linear.y,
                                twist.angular.z};

  if (!queue.push(Consumer::MessageTag::MOTOR_FRAME,
                  Consumer::MessageBody<HW::DRIVE_STYLE>{
                      .motor_frame = Drive::inverse_kinematics<HW::DRIVE_STYLE>(
                          body_twist)})) {
    ESP_LOGE(TAG, "Dropped motor frame: consumer queue full");
  }
}

static void rosTaskWrapper(void *pvParameters) {
  ROS::spin(pvParameters, onTwist);

  vTaskDelete(nullptr);
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "Testing UWB");
  ESP_LOGI(TAG, "FreeRTOS tick: %d Hz", CONFIG_FREERTOS_HZ);

  HW::SpiBus spi_bus = HW::Chassis::make<HW::SpiBus>();
  HW::Dwm dwm_sensor = HW::Chassis::make<HW::Dwm>(spi_bus);

  // bring-up: flip to false on the responder board
  constexpr bool kInitiator = true;

  if (auto id = dwm_sensor.get_device_id()) {
    ESP_LOGI(TAG, "DW1000 id: 0x%08lX", static_cast<unsigned long>(*id));
  } else {
    ESP_LOGE(TAG, "DW1000 id read failed");
  }

  if (auto r = dwm_sensor.configure(); r) {
    ESP_LOGI(TAG, "DW1000 configured");
  } else {
    ESP_LOGE(TAG, "DW1000 configure failed");
  }

  while (true) {
    if constexpr (kInitiator) {
      if (auto d = dwm_sensor.range()) {
        ESP_LOGI(TAG, "range: %d cm", static_cast<int>(*d * 100.0));
      } else {
        ESP_LOGE(TAG, "range failed");
      }
      vTaskDelay(pdMS_TO_TICKS(200));
    } else {
      auto r = dwm_sensor.respond();
      ESP_LOGI(TAG, "respond: %s", r ? "ok" : "fail");
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) ||                                \
    defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
  ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

  std::optional<HW::QueueType> queue = HW::QueueType::create();

  if (!queue) {
    ESP_LOGE(TAG, "Unable to create consumer queue");
    return;
  }

  static ConsumerTaskData consumerTaskData{HW::Chassis::make<HW::MotorDriver>(),
                                           std::move(*queue)};

  ESP_LOGI(TAG, "Hello world!");

  xTaskCreate(
      rosTaskWrapper, "uros_task",
      4096, // TODO: see
            // https://github.com/micro-ROS/micro_ros_espidf_component/blob/cd1da2b3d7d73f48743a2c42ac0e915cd751bb74/examples/int32_publisher/main/main.c#L105
      (void *)&consumerTaskData.queue, configMAX_PRIORITIES - 1, NULL);

  xTaskCreate(consumerTaskWrapper, "consumer_task", 4096,
              (void *)&consumerTaskData, configMAX_PRIORITIES - 1, NULL);
}
