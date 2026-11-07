#include "base.h"
#include "differential_drive.h"
#include "dwm.h"
#include "encoder_bank.h"
#include "esp32.h"
#include "l298n.h"
#include "mpu6050.h"
#include "odometry.h"
#include "robot.h"
#include "sdkconfig.h"
#include "session.h"
#include "swarm_sched.h"
#include "system.h"

#ifdef CONFIG_K_INITIATOR
  constexpr bool K_INITIATOR = true;
#else
  constexpr bool K_INITIATOR = false;
#endif

#include "esp_log.h"

#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/twist_with_covariance_stamped.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/float32.h>
#include <uros_network_interfaces.h>

#include "freertos/FreeRTOS.h"

#include <array>
#include <cstddef>
#include <numbers>
#include <optional>
#include <utility>

  static const char *TAG = "main";

  SWARM_ROS_MESSAGE(geometry_msgs, msg, Twist)
  SWARM_ROS_MESSAGE(geometry_msgs, msg, TwistWithCovarianceStamped)
  SWARM_ROS_MESSAGE(sensor_msgs, msg, Range)
  SWARM_ROS_MESSAGE(std_msgs, msg, Float32)

  static_assert(Robot::kRangeInfrared == sensor_msgs__msg__Range__INFRARED);

  static double read_effort(const std_msgs__msg__Float32 &msg) {
    ESP_LOGI(TAG, "umc/front: %+.2f", static_cast<double>(msg.data));

    return msg.data;
}

namespace HW {
using SPI = ESP32::SPI;
using SpiBus = ESP32::SpiBus;
using GPIO = ESP32::GPIO;
using PWM = ESP32::PWM;

// TODO: NEWMCPCBV2 routes both TB6612s over pins the Makerfabs ESP32 UWB
// carrier has already spoken for, so the motors keep the contested pins and the
// DW1000 below is declared on free ones instead. That makes DWM_PINS and
// SpiBus::miso/mosi descriptions of nothing -- the radio is still physically
// wired to the carrier's pins -- so the ranging tasks stay unspawned until the
// board is reworked. The carrier's real wiring, which has to come back with it:
//   GPIO19  M3 PWMA   vs SPI_MISO    cut-and-jumper: the DW1000 drives it
//   GPIO34  M3 EncA   vs DW_IRQ      cut-and-jumper: the DW1000 drives it
//   GPIO4   M1 PWMA   vs DW_CS       two ESP32 outputs on one net
//   GPIO23  M2 BIN2   vs SPI_MOSI    two ESP32 outputs on one net
//   GPIO27  M3 AIN2   vs DW_RST      two ESP32 outputs on one net
// M4 (25/32/33, encoder 36/39) is the only channel clear of all of it.

// M3 and M4 are the only motors with encoders, so they are the differential
// pair; M1/M2 are the UMC below. LEFT/RIGHT is a guess at which connector is
// which side -- swap the two names if the robot spins on a forward twist
// TODO: no longer L298N, fix to proper name. but implementation is identical
constexpr auto MOTOR_PINS =
    L298N::motors(L298N::MotorPins{Motor::Name::LEFT, GPIO_NUM_26, GPIO_NUM_27,
                                   GPIO_NUM_19, 0},
                  L298N::MotorPins{Motor::Name::RIGHT, GPIO_NUM_32, GPIO_NUM_33,
                                   GPIO_NUM_25, 1});

using MotorDriver = L298N::MotorDriver<GPIO, PWM, MOTOR_PINS>;

// the universal module connector: two high-torque motors raising and lowering
// the face that modules bolt to. No encoder and no geometry -- the pair always
// moves together, so the command is one open-loop effort
//
// U2 drives the pair: M1 is channel A (AIN1/AIN2/PWMA), M2 is channel B
// (BIN1/BIN2/PWMB), both through header J2. Swap a row's in_a/in_b if its
// motor raises the face on a negative effort
constexpr auto UMC_PINS =
    L298N::motors(L298N::MotorPins{Motor::Name::FRONT_UMC_A, GPIO_NUM_16,
                                   GPIO_NUM_17, GPIO_NUM_4, 2},
                  L298N::MotorPins{Motor::Name::FRONT_UMC_B, GPIO_NUM_22,
                                   GPIO_NUM_23, GPIO_NUM_5, 3});

using UmcDriver = L298N::MotorDriver<GPIO, PWM, UMC_PINS>;

// one PCNT unit per wheel. 34/35/36/39 are input-only and have no internal
// pull-up, so an open-collector hall output needs an external one
constexpr auto ENCODER_ROWS = Encoders::for_style<Robot::STYLE>(
    Encoders::Row{Motor::Name::LEFT, GPIO_NUM_35, GPIO_NUM_34, 0},
    Encoders::Row{Motor::Name::RIGHT, GPIO_NUM_39, GPIO_NUM_36, 1});

using EncoderBank = Encoders::Bank<ESP32::QuadratureCounter, ENCODER_ROWS,
                                   Robot::WHEEL_ENCODER>;

// placeholder pins: free of the motors, wired to nothing. See the TODO above
constexpr auto DWM_PINS = HAL::pins(HAL::NamedPin{"cs", GPIO_NUM_15},
                                    HAL::NamedPin{"reset", GPIO_NUM_21},
                                    HAL::NamedPin{"irq", GPIO_NUM_13});

using Dwm = DWM<SPI, GPIO, DWM_PINS, SpiBus>;

// placeholder pins, free of everything above and wired to nothing: the MPU6050
// shares the UMC's I2C bus, whose SDA/SCL NEWMCPCBV2 routes to pins the motors
// and the carrier have already spoken for. Both are strapping pins, so this
// pair is declarable rather than usable -- the imu task below may not work
// until the real pair goes in
// TODO: replace with the UMC header's SDA/SCL
constexpr auto I2C_PINS =
    HAL::pins(HAL::NamedPin{"sda", GPIO_NUM_18, HAL::Use::Shared},
              HAL::NamedPin{"scl", GPIO_NUM_14, HAL::Use::Shared});

using I2cBus = ESP32::I2cBus<I2C_PINS>;
using I2C = ESP32::I2C<I2cBus>;

// +-500 dps and +-4 g leave headroom over what a wheeled chassis does, and the
// 44 Hz filter at 100 Hz output keeps the frame buzz out of the gyro
constexpr auto IMU_CONFIG =
    MPU6050::config({.gyro = Imu::GyroRange::Dps500,
                     .accel = Imu::AccelRange::G4,
                     .dlpf = MPU6050::Dlpf::Hz44,
                     .sample_rate_divider = 9});

using Mpu = MPU6050::Sensor<I2C, GPIO, IMU_CONFIG, I2cBus>;

using Chassis = Swarm::chassis<Robot::STYLE, MotorDriver, SpiBus, Dwm,
                               EncoderBank, UmcDriver, I2cBus, Mpu>;

using CmdVel =
    ROS::receives<geometry_msgs__msg__Twist, Robot::CMD_VEL_TOPIC, Drive::Twist,
                  &Robot::read_twist<geometry_msgs__msg__Twist>>;

using UmcCmd =
    ROS::receives<std_msgs__msg__Float32, "umc/front", double, &read_effort>;

constexpr auto BASE_FRAME =
    Robot::frame_in(CONFIG_SWARM_ROS_NAMESPACE, Robot::BASE_FRAME);
constexpr auto UWB_FRAME =
    Robot::frame_in(CONFIG_SWARM_ROS_NAMESPACE, Robot::UWB_FRAME);

using UwbRange =
    ROS::streams<sensor_msgs__msg__Range, Robot::UWB_TOPIC, UWB_FRAME,
                 double, &Robot::fill_range<sensor_msgs__msg__Range>>;

// the mean twist over each publish period rather than a sample of it, so an
// EKF integrating at 5Hz retraces what the control loop integrated at 50Hz.
// Velocity rather than pose because its error does not grow with distance,
// and TwistWithCovarianceStamped because nav_msgs/Odometry (~720 octets)
// outgrows the 512-octet XRCE MTU, which a best-effort stream cannot fragment
using OdomTwist = ROS::streams<
    geometry_msgs__msg__TwistWithCovarianceStamped, Robot::ODOM_TOPIC,
    BASE_FRAME, Drive::Twist,
    &Robot::fill_twist<geometry_msgs__msg__TwistWithCovarianceStamped>>;

using Node = ROS::node<Robot::NODE, CONFIG_SWARM_ROS_NAMESPACE, CmdVel, UmcCmd,
                       UwbRange, OdomTwist>;

using PoseMailbox = Sched::channel<Odometry::Pose, 1>;
using YawRateMailbox = Sched::channel<double, 1>;

// motion is arithmetic plus two GPIO writes, so it shares the fast task
constexpr uint32_t MOTION_PERIOD_MS = 5;

constexpr uint32_t EXECUTOR_PERIOD_MS = 10;

// one yaw rate per control period, so the heading has no gaps to hold across
constexpr uint32_t IMU_PERIOD_MS = Robot::DRIVE_PERIOD_MS;

constexpr uint32_t POLL_PERIOD_MS = 100;
} // namespace HW

static HW::PoseMailbox poses;
static HW::YawRateMailbox yaw_rates;
static ROS::session<HW::Node> session;

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "FreeRTOS tick: %d Hz", CONFIG_FREERTOS_HZ);

  auto &peripherals = HW::Chassis::take();
  auto &dwm_sensor = peripherals.get<HW::Dwm>();
  auto &motors = peripherals.get<HW::MotorDriver>();
  auto &encoders = peripherals.get<HW::EncoderBank>();
  auto &umc = peripherals.get<HW::UmcDriver>();
  auto &imu_sensor = peripherals.get<HW::Mpu>();

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
      // the loop integrates odometry, so it has to run from boot rather than
      // from the first cmd_vel. A zero twist coasts the wheels while it does
      Sched::watched<Robot::DRIVE_PERIOD_MS, Robot::CMD_VEL_TIMEOUT_MS>(
          session.inbox<HW::CmdVel>(), Drive::Twist{}) |
          Sched::then(Drive::inverse_kinematics<Robot::STYLE>) |
          Sched::to(
              [&motors, &encoders, state = Base::State<Robot::STYLE>{}](
                  const Drive::WheelVelocities<Robot::STYLE> &targets) mutable {
                // I2C stays out of this loop: a stalled bus empties the mailbox
                // and the heading falls back to the encoders
                const auto [next, frame] = Robot::drive<HW::EncoderBank::spec>(
                    state, targets, encoders.advance(), yaw_rates.pop(0));
                state = next;

                motors.run(frame);
                poses.overwrite(state.pose);
              }),
      Sched::latest<HW::MOTION_PERIOD_MS>(session.inbox<HW::UmcCmd>()) |
          Sched::then(Motor::ganged<HW::UmcDriver::motors>) |
          Sched::to([&umc](const auto &frame) {
            for (const Motor::Command &cmd : frame.commands) {
              ESP_LOGI(TAG, "umc motor %d: dir %d duty %.2f",
                       static_cast<int>(cmd.name), static_cast<int>(cmd.dir),
                       cmd.pwm_ratio);
            }

            umc.run(frame);
          }));

  static auto odometry = Sched::make_task<4096, configMAX_PRIORITIES - 4>(
      Sched::latest<Robot::ODOMETRY_PERIOD_MS>(poses) |
      Sched::to([last = std::optional<Odometry::Pose>{}](
                    const Odometry::Pose &pose) mutable {
        const auto previous = std::exchange(last, pose);

        if (!previous) {
          return;
        }

        // false until the session is up as well, so at warning level this
        // would fire five times a second whenever the agent is away
        if (!session.send<HW::OdomTwist>(Odometry::twist_between(
                *previous, pose, Robot::ODOMETRY_PERIOD_MS / 1000.0))) {
          ESP_LOGD(TAG, "Dropped twist");
        }
      }));

  // init runs on the uros task, not here: a dead agent must not stop motion
  // from spawning
  static auto ros = Sched::make_task<16000, configMAX_PRIORITIES - 1>(
      ROS::serve<HW::EXECUTOR_PERIOD_MS>(session));

  // the bias estimate assumes the robot is still, so calibration belongs in
  // on_start rather than anywhere a command could already be moving it
  static auto imu = Sched::make_task<4096, configMAX_PRIORITIES - 4>(
      Sched::on_start([&imu_sensor] {
        while (true) {
          if (auto id = imu_sensor.who_am_i()) {
            ESP_LOGI(TAG, "MPU6050 id: 0x%02X", *id);
            break;
          } else {
            ESP_LOGW(TAG, "MPU6050 id read failed; retrying");
            vTaskDelay(pdMS_TO_TICKS(1000));
          }
        }

        if (!imu_sensor.configure()) {
          ESP_LOGE(TAG, "MPU6050 configure failed");

          return false;
        }

        ESP_LOGI(TAG, "MPU6050 calibrating -- hold still");

        if (const auto offsets = imu_sensor.calibrate()) {
          ESP_LOGI(TAG, "MPU6050 gyro bias: %d %d %d counts", offsets->gyro.x,
                   offsets->gyro.y, offsets->gyro.z);
        } else {
          ESP_LOGE(TAG, "MPU6050 calibration failed");

          return false;
        }

        return true;
      }) |
      Sched::every<HW::IMU_PERIOD_MS> |
      Sched::then([&imu_sensor] { return imu_sensor.read_sample(); }) |
      Sched::to([](const Imu::Sample &sample) {
        // TODO: check the sign on the bench -- a part mounted upside down
        // needs this negated, and Imu::Axis has no way to say so yet
        yaw_rates.overwrite(Imu::yaw_rate(sample, HW::Mpu::spec.up));

        ESP_LOGD(TAG, "gyro %+7.2f %+7.2f %+7.2f dps  accel %+6.3f %+6.3f "
                      "%+6.3f g  %.1f C",
                 sample.gyro.x, sample.gyro.y, sample.gyro.z, sample.accel.x,
                 sample.accel.y, sample.accel.z, sample.temperature_c);
      }));

  ros.spawn("uros");
  motion.spawn("motion");
  odometry.spawn("odometry");
  // may not work until I2C_PINS are corrected: expect who_am_i retries
  imu.spawn("imu");

  if constexpr (K_INITIATOR) {
    ESP_LOGI(TAG, "k initiator true");
    [[maybe_unused]] static auto ranging =
        Sched::make_task<4096, configMAX_PRIORITIES - 3>(
            Sched::every<HW::POLL_PERIOD_MS> |
            Sched::then([&dwm_sensor] { return dwm_sensor.range(); }) |
            Sched::to([](double meters) {
              if (!session.send<HW::UwbRange>(meters)) {
                ESP_LOGW(TAG, "Dropped range: publish channel full");
              }
            }));

    // ranging.spawn("ranging");
  } else {
    ESP_LOGI(TAG, "k initiator false");
    // the responder measures t_reply on its own crystal and drift multiplies
    // it, so this task must not be preempted between rx(poll) and tx(reply)
    [[maybe_unused]] static auto responding =
        Sched::make_task<4096, configMAX_PRIORITIES - 1>(
            Sched::every<HW::POLL_PERIOD_MS> |
            Sched::to([&dwm_sensor] { return dwm_sensor.respond(); }));

    // responding.spawn("responding");
  }
}
