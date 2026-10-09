#include "differential_drive.h"
#include "encoders.h"
#include "esp32.h"
#include "l298n.h"
#include "session.h"
#include "swarm_sched.h"
#include "system.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"

#include <geometry_msgs/msg/twist.h>
#include <uros_network_interfaces.h>

#include "freertos/FreeRTOS.h"

static const char *TAG = "main";

SWARM_ROS_MESSAGE(geometry_msgs, msg, Twist)

namespace HW {
using GPIO = ESP32::GPIO;
using PWM = ESP32::PWM;

// TODO: NEWMCPCBV2 routes both TB6612s over pins the Makerfabs ESP32 UWB
// carrier has already spoken for, so the DW1000 is off the chassis until the
// board is reworked -- see `git show 962dca0:main/src/main.cpp` for the wiring
// that goes back once it is. The two that need a cut-and-jumper, because the
// DW1000 drives them and would fight the ESP32:
//   GPIO19  M3 PWMA   vs SPI_MISO
//   GPIO34  M3 EncA   vs DW_IRQ
// and three where two ESP32 outputs share a net, so each function corrupts the
// other but nothing is damaged:
//   GPIO4   M1 PWMA   vs DW_CS
//   GPIO23  M2 BIN2   vs SPI_MOSI
//   GPIO27  M3 AIN2   vs DW_RST
// M4 (25/32/33, encoder 36/39) is the only channel clear of all of it.

// M3 and M4 are the only motors with encoders, so they are the differential
// pair; M1/M2 are unpopulated and claim nothing. LEFT/RIGHT is a guess at which
// connector is which side -- swap the two names if the robot spins on a
// forward twist
constexpr auto MOTOR_PINS = L298N::motors(
    L298N::MotorPins{Motor::Name::LEFT, GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_19,
                     0},
    L298N::MotorPins{Motor::Name::RIGHT, GPIO_NUM_32, GPIO_NUM_33, GPIO_NUM_25,
                     1});

using MotorDriver = L298N::MotorDriver<GPIO, PWM, MOTOR_PINS>;

constexpr Drive::Style STYLE = Drive::Style::DIFFERENTIAL;

// one PCNT unit per wheel. 34/35/36/39 are input-only and have no internal
// pull-up, so an open-collector hall output needs an external one
constexpr auto ENCODER_ROWS = Encoders::for_style<STYLE>(
    Encoders::Row{Motor::Name::LEFT, 34, 35, 0},
    Encoders::Row{Motor::Name::RIGHT, 36, 39, 1});

using EncoderBank =
    Encoders::Bank<ESP32::QuadratureCounter, ENCODER_ROWS, Encoder::fit0485>;

using Chassis = Swarm::chassis<STYLE, MotorDriver, EncoderBank>;

using CmdVel = ROS::subscribes<geometry_msgs__msg__Twist, "cmd_vel">;

using Node = ROS::node<"base", CONFIG_SWARM_ROS_NAMESPACE, CmdVel>;

using CmdChannel = Sched::channel<Drive::Twist, 4>;

// motion is arithmetic plus two GPIO writes, so it shares the fast task
constexpr uint32_t MOTION_PERIOD_MS = 5;
constexpr uint32_t EXECUTOR_PERIOD_MS = 10;
constexpr uint32_t ODOMETRY_PERIOD_MS = 50;
} // namespace HW

static HW::CmdChannel commands;
static ROS::session<HW::Node> session;

static constexpr const char *label(Motor::Name name) {
  switch (name) {
  case Motor::Name::LEFT:
    return "left";
  case Motor::Name::RIGHT:
    return "right";
  default:
    return "motor";
  }
}

static auto onTwist = [](const geometry_msgs__msg__Twist &twist) {
  const Drive::Twist body{twist.linear.x, twist.linear.y, twist.angular.z};

  // bring-up only: remove once cmd_vel is known to reach the wheels
  ESP_LOGI(TAG, "twist x=%.2f z=%.2f", body.linear_x, body.angular_z);

  if (!commands.push(body, 0)) {
    ESP_LOGW(TAG, "Dropped twist: command channel full");
  }
};

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "FreeRTOS tick: %d Hz", CONFIG_FREERTOS_HZ);

  auto &peripherals = HW::Chassis::take();
  auto &motors = peripherals.get<HW::MotorDriver>();
  // auto &encoders = peripherals.get<HW::EncoderBank>();

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) ||                                \
    defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
  ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

  static auto motion = Sched::make_task<4096, configMAX_PRIORITIES - 2>(
      Sched::latest<HW::MOTION_PERIOD_MS>(commands) |
      Sched::then(Drive::inverse_kinematics<HW::STYLE>) |
      Sched::to([&motors](const Drive::Frame<HW::STYLE> &frame) {
        // bring-up only: remove once cmd_vel is known to reach the wheels
        for (const Motor::Command &cmd : frame.commands) {
          ESP_LOGI(TAG, "%s dir=%d duty=%.2f", label(cmd.name),
                   static_cast<int>(cmd.dir), cmd.pwm_ratio);
        }

        motors.run(frame);
      }));

  // 20 Hz x 2 wheels drowns the micro-ROS session messages on the console
  // static auto odometry = Sched::make_task<4096, configMAX_PRIORITIES - 4>(
  //     Sched::every<HW::ODOMETRY_PERIOD_MS> |
  //     Sched::then([&encoders] { return encoders.advance(); }) |
  //     Sched::to([](const auto &deltas) {
  //       constexpr double seconds = HW::ODOMETRY_PERIOD_MS / 1000.0;
  //
  //       for (const Encoder::Reading &wheel : deltas) {
  //         ESP_LOGI(TAG, "%s %+5ld counts %+7.2f rpm", label(wheel.name),
  //                  static_cast<long>(wheel.counts),
  //                  Encoder::rpm(HW::EncoderBank::spec, wheel.counts,
  //                  seconds));
  //       }
  //     }));

  // init runs on the uros task, not here: a dead agent must not stop motion
  // from spawning
  static auto ros = Sched::make_task<16000, configMAX_PRIORITIES - 1>(
      Sched::every<HW::EXECUTOR_PERIOD_MS> | Sched::to([] { session.poll(); }));

  // TODO: encapsulate this
  //
  // a false from on_start deletes the task, so one lost handshake used to cost
  // cmd_vel until the next power cycle. init() unwinds itself on failure, so
  // retrying is just a delay
  ros.on_start([] {
    ESP_LOGI(TAG, "heap before session init: %lu free, %lu largest block",
             static_cast<unsigned long>(esp_get_free_heap_size()),
             static_cast<unsigned long>(
                 heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));

    while (!session.init() || !session.on<HW::CmdVel>(onTwist)) {
      session.fini();

      ESP_LOGW(TAG, "micro-ROS session init failed; retrying agent %s:%s",
               CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);

      vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "micro-ROS session up: /%s/base subscribed to cmd_vel",
             CONFIG_SWARM_ROS_NAMESPACE);

    return true;
  });

  // TODO: maybe make this part of construction so we don't have two-phase?
  ros.spawn("uros");
  motion.spawn("motion");
  // odometry.spawn("odometry");
}
