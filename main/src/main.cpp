#include "differential_drive.h"
#include "dwm.h"
#include "esp32.h"
#include "l298n.h"
#include "session.h"
#include "swarm_sched.h"
#include "system.h"

#include "esp_log.h"

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/range.h>
#include <uros_network_interfaces.h>

#include "freertos/FreeRTOS.h"

static const char *TAG = "main";

SWARM_ROS_MESSAGE(geometry_msgs, msg, Twist)
SWARM_ROS_MESSAGE(sensor_msgs, msg, Range)

static void fill_range(sensor_msgs__msg__Range &msg, const double &meters) {
  // Range was written for sonar/IR; UWB has no radiation_type of its own, and
  // INFRARED is the conventional stand-in until this moves to a swarm_msgs type
  msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
  msg.field_of_view = 6.2831853f;
  msg.min_range = 0.0f;
  msg.max_range = 300.0f;
  msg.range = static_cast<float>(meters);
}

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

using Dwm = DWM<SPI, GPIO, DWM_PINS, SpiBus>;

using Chassis =
    Swarm::chassis<Drive::Style::DIFFERENTIAL, MotorDriver, SpiBus, Dwm>;

constexpr Drive::Style DRIVE_STYLE = Chassis::style;

using CmdVel = ROS::subscribes<geometry_msgs__msg__Twist, "cmd_vel">;
using UwbRange = ROS::streams<sensor_msgs__msg__Range, "uwb/range", "uwb_link",
                              double, &fill_range>;

using Node = ROS::node<"base", CONFIG_SWARM_ROS_NAMESPACE, CmdVel, UwbRange>;

using CmdChannel = Sched::channel<Drive::Twist, 4>;

// bring-up: flip to false on the responder board
constexpr bool kInitiator = true;

// motion is arithmetic plus two GPIO writes, so it shares the fast task. UWB
// blocks up to ~40ms inside range()'s two 20ms receives, which would eat eight
// motion cycles, so it gets a task of its own
constexpr uint32_t MOTION_PERIOD_MS = 5;
constexpr uint32_t EXECUTOR_PERIOD_MS = 10;
constexpr uint32_t POLL_PERIOD_MS = kInitiator ? 200 : 10;
} // namespace HW

static HW::CmdChannel commands;
static ROS::session<HW::Node> session;

static auto onTwist = [](const geometry_msgs__msg__Twist &twist) {
  const Drive::Twist body{twist.linear.x, twist.linear.y, twist.angular.z};

  if (!commands.push(body, 0)) {
    ESP_LOGW(TAG, "Dropped twist: command channel full");
  }
};


static const char *TAG = "UWB_IRQ";

// Configuration
constexpr gpio_num_t kUwbIrqPin = GPIO_NUM_34;
static TaskHandle_t s_uwb_task_handle = nullptr;

// =============================================================================
// 1. HARDWARE ISR (Runs in IRAM, takes < 1 microsecond)
// =============================================================================
extern "C" void IRAM_ATTR dwm_gpio_isr_handler(void *arg) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (s_uwb_task_handle != nullptr) {
    // Unblock the worker task instantly
    vTaskNotifyGiveFromISR(s_uwb_task_handle, &xHigherPriorityTaskWoken);
  }

  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

// =============================================================================
// 2. UWB WORKER TASK (Handles SPI & Response Logic)
// =============================================================================
void uwb_worker_task(void *pvParameters) {
  // Grab pointer to your driver instance passed via xTaskCreate
  auto *dwm = static_cast<DWM *>(pvParameters);

  // Store task handle so the ISR knows who to wake up
  s_uwb_task_handle = xTaskGetCurrentTaskHandle();

  ESP_LOGI(TAG, "UWB Worker Task ready and sleeping...");

  while (true) {
    // Sleep with 0% CPU usage until the GPIO ISR fires
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // -------------------------------------------------------------------------
    // Hardware Response Logic
    // -------------------------------------------------------------------------
    // 1. Read status and process frame (e.g. DWM::respond() or range logic)
    if (auto r = dwm->respond(); r) {
      ESP_LOGI(TAG, "UWB respond successful");
    } else {
      ESP_LOGE(TAG, "UWB respond failed or invalid frame");
    }

    // 2. IMPORTANT: Write-1-to-Clear (W1C) status register over SPI
    // dwm->clear_status_flags(); 
    // Once status is cleared, the DW1000 will pull its IRQ line LOW,
    // making it ready to fire the next rising edge.
  }
}



extern "C" void app_main(void) {
  ESP_LOGI(TAG, "FreeRTOS tick: %d Hz", CONFIG_FREERTOS_HZ);

  auto &peripherals = HW::Chassis::take();
  auto &dwm_sensor = peripherals.get<HW::Dwm>();
  auto &motors = peripherals.get<HW::MotorDriver>();

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

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) ||                                \
    defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
  ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

  static auto motion = Sched::make_task<4096, configMAX_PRIORITIES - 2>(
      Sched::latest<HW::MOTION_PERIOD_MS>(commands) |
      Sched::then(Drive::inverse_kinematics<HW::DRIVE_STYLE>) |
      Sched::to([&motors](const Drive::Frame<HW::DRIVE_STYLE> &frame) {
        motors.run(frame);
      }));

  // init runs on the uros task, not here: a dead agent must not stop motion
  // from spawning
  static auto ros = Sched::make_task<16000, configMAX_PRIORITIES - 1>(
      Sched::every<HW::EXECUTOR_PERIOD_MS> | Sched::to([] { session.poll(); }));

  // TODO: encapsulate this
  ros.on_start([] {
    if (!session.init() || !session.on<HW::CmdVel>(onTwist)) {
      ESP_LOGE(TAG, "micro-ROS session init failed");
      return false;
    }

    return true;
  });

  // TODO: maybe make this part of construction so we don't have two-phase?
  ros.spawn("uros");
  motion.spawn("motion");


// Configure GPIO Interrupt Pin
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << kUwbIrqPin),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE, // DW1000 IRQ is Active-HIGH
      .intr_type = GPIO_INTR_POSEDGE,        // Trigger on rising edge (TXFRS/RXFCG)
  };
  gpio_config(&io_conf);

  // Install GPIO ISR service & register handler
  gpio_install_isr_service(0);
  gpio_isr_handler_add(kUwbIrqPin, dwm_gpio_isr_handler, nullptr);

  // Spawn dedicated worker task (High priority)
  xTaskCreate(
      uwb_worker_task,
      "uwb_worker",
      4096,
      &dwm_sensor,               // Pass driver instance pointer
      configMAX_PRIORITIES - 2, // High priority to handle IRQ immediately
      nullptr
  );

  // if constexpr (HW::kInitiator) {
  //   static auto ranging = Sched::make_task<4096, configMAX_PRIORITIES - 3>(
  //       Sched::every<HW::POLL_PERIOD_MS> |
  //       Sched::then([&dwm_sensor] { return dwm_sensor.range(); }) |
  //       Sched::to([](double meters) {
  //         if (!session.send<HW::UwbRange>(meters)) {
  //           ESP_LOGW(TAG, "Dropped range: publish channel full");
  //         }
  //       }));

  //   ranging.spawn("ranging");
  // } else {
  //   // the responder measures t_reply on its own crystal and drift multiplies
  //   // it, so this task must not be preempted between rx(poll) and tx(reply)
  //   static auto responding = Sched::make_task<4096, configMAX_PRIORITIES - 1>(
  //       Sched::every<HW::POLL_PERIOD_MS> |
  //       Sched::to([&dwm_sensor] { return dwm_sensor.respond(); }));

  //   responding.spawn("responding");
  // }
}
