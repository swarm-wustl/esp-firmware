#include "consumer.h"
#include "dwm.h"
#include "differential_drive.h"
#include "esp32.h"
#include "l298n.h"
#include "ros.h"

#include "esp_log.h"

#include <uros_network_interfaces.h>

#include "freertos/FreeRTOS.h"
#include <algorithm>
#include <optional>
#include <utility>

static const char *TAG = "main";

template <size_t N>
static constexpr bool covers(const std::array<L298N::MotorPins, N> &pins,
                             const std::array<Motor::Name, N> &names) {
  return std::ranges::all_of(names, [&pins](Motor::Name name) {
    return std::ranges::find(pins, name, &L298N::MotorPins::name) != pins.end();
  });
}

namespace HW {
constexpr Drive::Style DRIVE_STYLE = Drive::Style::DIFFERENTIAL;
constexpr size_t MOTOR_COUNT = Drive::motor_count(DRIVE_STYLE);

using SPI = ESP32::SPI;
using GPIO = ESP32::GPIO;
using PWM = ESP32::PWM;
using MotorDriver = L298N::MotorDriver<GPIO, PWM, MOTOR_COUNT>;

constexpr int STANDBY_PIN = GPIO_NUM_0;

constexpr std::array<L298N::MotorPins, MOTOR_COUNT> MOTOR_PINS{
    L298N::MotorPins{Motor::Name::LEFT, GPIO_NUM_16, GPIO_NUM_17, GPIO_NUM_4, 0},
    L298N::MotorPins{Motor::Name::RIGHT, GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_5, 1},
};

static_assert(HAL::MotorDriverTrait<MotorDriver>);
static_assert(covers(MOTOR_PINS, Drive::motor_names<DRIVE_STYLE>()));
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
      Drive::convert_twist<HW::DRIVE_STYLE>(twist);

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
  ESP_LOGI(TAG, "Testing UWB");
  ESP_LOGI(TAG, "FreeRTOS tick: %d Hz", CONFIG_FREERTOS_HZ);

  HW::SPI spi{GPIO_NUM_4}; // TODO: put pins in a config somewhere
  HW::GPIO gpio{};
  DWM dwm_sensor{std::move(spi), std::move(gpio), GPIO_NUM_27, GPIO_NUM_34};

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

  std::optional<Consumer::QueueType> queue = Consumer::QueueType::create();

  if (!queue) {
    ESP_LOGE(TAG, "Unable to create consumer queue");
    return;
  }

  // Make the struct static so it lives as long as the program (incase mani()
  // ever terminates)
  static ConsumerTaskData consumerTaskData{
      HW::MotorDriver{HW::GPIO{}, HW::PWM{}, HW::MOTOR_PINS, HW::STANDBY_PIN},
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
