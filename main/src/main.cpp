#include "differential_drive.h"
#include "dwm.h"
#include "encoders.h"
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

// TODO: NEWMCPCBV2 routes both TB6612s over pins the Makerfabs ESP32 UWB
// carrier has already spoken for, so the motors keep the contested pins and the
// DW1000 below is declared on free ones instead. That makes DWM_PINS and
// SpiBus::miso descriptions of nothing -- the radio is still physically wired
// to the carrier's pins -- so the ranging tasks stay unspawned until the board
// is reworked. The carrier's real wiring, which has to come back with it:
//   GPIO19  M3 PWMA   vs SPI_MISO    cut-and-jumper: the DW1000 drives it
//   GPIO34  M3 EncA   vs DW_IRQ      cut-and-jumper: the DW1000 drives it
//   GPIO4   M1 PWMA   vs DW_CS       two ESP32 outputs on one net
//   GPIO23  M2 BIN2   vs SPI_MOSI    two ESP32 outputs on one net
//   GPIO27  M3 AIN2   vs DW_RST      two ESP32 outputs on one net
// M4 (25/32/33, encoder 36/39) is the only channel clear of all of it.

// M3 and M4 are the only motors with encoders, so they are the differential
// pair; M1/M2 are unpopulated and claim nothing. LEFT/RIGHT is a guess at which
// connector is which side -- swap the two names if the robot spins on a
// forward twist
constexpr auto MOTOR_PINS =
    L298N::motors(L298N::MotorPins{Motor::Name::LEFT, GPIO_NUM_26, GPIO_NUM_27,
                                   GPIO_NUM_19, 0},
                  L298N::MotorPins{Motor::Name::RIGHT, GPIO_NUM_32, GPIO_NUM_33,
                                   GPIO_NUM_25, 1});

using MotorDriver = L298N::MotorDriver<GPIO, PWM, MOTOR_PINS>;

constexpr Drive::Style STYLE = Drive::Style::DIFFERENTIAL;

// one PCNT unit per wheel. 34/35/36/39 are input-only and have no internal
// pull-up, so an open-collector hall output needs an external one
constexpr auto ENCODER_ROWS =
    Encoders::for_style<STYLE>(Encoders::Row{Motor::Name::LEFT, 34, 35, 0},
                               Encoders::Row{Motor::Name::RIGHT, 36, 39, 1});

using EncoderBank =
    Encoders::Bank<ESP32::QuadratureCounter, ENCODER_ROWS, Encoder::fit0485>;

// placeholder pins: free of the motors, wired to nothing. See the TODO above
constexpr auto DWM_PINS = HAL::pins(HAL::NamedPin{"cs", GPIO_NUM_4},
                                    HAL::NamedPin{"reset", GPIO_NUM_21},
                                    HAL::NamedPin{"irq", GPIO_NUM_13});

using Dwm = DWM<SPI, GPIO, DWM_PINS, SpiBus>;

using Chassis =
    Swarm::chassis<STYLE, MotorDriver, SpiBus, Dwm, EncoderBank>;

using CmdVel = ROS::subscribes<geometry_msgs__msg__Twist, "cmd_vel">;

using UwbRange = ROS::streams<sensor_msgs__msg__Range, "uwb/range", "uwb_link",
                              double, &fill_range>;

using Node = ROS::node<"base", CONFIG_SWARM_ROS_NAMESPACE, CmdVel, UwbRange>;

using CmdChannel = Sched::channel<Drive::Twist, 4>;

// motion is arithmetic plus two GPIO writes, so it shares the fast task
constexpr uint32_t MOTION_PERIOD_MS = 5;
constexpr uint32_t EXECUTOR_PERIOD_MS = 10;
constexpr uint32_t ODOMETRY_PERIOD_MS = 500;
constexpr uint32_t POLL_PERIOD_MS = 100;

// bring-up: flip to false on the responder board
constexpr bool kInitiator = true;
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

  if (!commands.push(body, 0)) {
    ESP_LOGW(TAG, "Dropped twist: command channel full");
  }
};

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "FreeRTOS tick: %d Hz", CONFIG_FREERTOS_HZ);

  auto &peripherals = HW::Chassis::take();
  auto &dwm_sensor = peripherals.get<HW::Dwm>();
  auto &motors = peripherals.get<HW::MotorDriver>();
  auto &encoders = peripherals.get<HW::EncoderBank>();

  // these two talk to the bus, so they report failure until the DW1000 is on
  // pins it is actually wired to
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
      Sched::then(Drive::inverse_kinematics<HW::STYLE>) |
      Sched::to([&motors](const Drive::Frame<HW::STYLE> &frame) {
        motors.run(frame);
      }));

  static auto odometry = Sched::make_task<4096, configMAX_PRIORITIES - 4>(
      Sched::every<HW::ODOMETRY_PERIOD_MS> |
      Sched::then([&encoders] { return encoders.advance(); }) |
      Sched::to([](const auto &deltas) {
        constexpr double seconds = HW::ODOMETRY_PERIOD_MS / 1000.0;

        for (const Encoder::Reading &wheel : deltas) {
          ESP_LOGI(TAG, "%s %+5ld counts %+7.2f rpm", label(wheel.name),
                   static_cast<long>(wheel.counts),
                   Encoder::rpm(HW::EncoderBank::spec, wheel.counts, seconds));
        }
      }));

  // init runs on the uros task, not here: a dead agent must not stop motion
  // from spawning
  static auto ros = Sched::make_task<16000, configMAX_PRIORITIES - 1>(
      Sched::every<HW::EXECUTOR_PERIOD_MS> |
      Sched::to([] { return session.poll(); }));

  // TODO: encapsulate this
  // a false from on_start deletes the task, so one lost handshake used to cost
  // cmd_vel until the next power cycle. init() unwinds itself on failure, so
  // retrying is just a delay
  ros.on_start([] {
    // TODO: this should really be encapsulated and part of ros declaration.
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

  ros.spawn("uros");
  motion.spawn("motion");
  odometry.spawn("odometry");

  if constexpr (HW::kInitiator) {
    [[maybe_unused]] static auto ranging = Sched::make_task<4096, configMAX_PRIORITIES - 3>(
        Sched::every<HW::POLL_PERIOD_MS> |
        Sched::then([&dwm_sensor] { return dwm_sensor.range(); }) |
        Sched::to([](double meters) {
          if (!session.send<HW::UwbRange>(meters)) {
            ESP_LOGW(TAG, "Dropped range: publish channel full");
          }
        }));

    // ranging.spawn("ranging");
  } else {
    // the responder measures t_reply on its own crystal and drift multiplies
    // it, so this task must not be preempted between rx(poll) and tx(reply)
    [[maybe_unused]] static auto responding = Sched::make_task<4096, configMAX_PRIORITIES - 1>(
        Sched::every<HW::POLL_PERIOD_MS> |
        Sched::to([&dwm_sensor] { return dwm_sensor.respond(); }));

    // responding.spawn("responding");
  }
}
