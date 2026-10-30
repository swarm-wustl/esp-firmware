#include "base.h"
#include "differential_drive.h"
#include "dwm.h"
#include "encoders.h"
#include "esp32.h"
#include "l298n.h"
#include "mpu6050.h"
#include "odometry.h"
#include "session.h"
#include "swarm_sched.h"
#include "system.h"

#include "esp_log.h"

#include <geometry_msgs/msg/pose_with_covariance_stamped.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/float32.h>
#include <uros_network_interfaces.h>

#include "freertos/FreeRTOS.h"

#include <array>
#include <cstddef>
#include <numbers>

static const char *TAG = "main";

SWARM_ROS_MESSAGE(geometry_msgs, msg, PoseWithCovarianceStamped)
SWARM_ROS_MESSAGE(geometry_msgs, msg, Twist)
SWARM_ROS_MESSAGE(sensor_msgs, msg, Range)
SWARM_ROS_MESSAGE(std_msgs, msg, Float32)

static void fill_range(sensor_msgs__msg__Range &msg, const double &meters) {
  // Range was written for sonar/IR; UWB has no radiation_type of its own, and
  // INFRARED is the conventional stand-in until this moves to a swarm_msgs type
  msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
  msg.field_of_view = 6.2831853f;
  msg.min_range = 0.0f;
  msg.max_range = 300.0f;
  msg.range = static_cast<float>(meters);
}

// row-major 6x6 over (x, y, z, roll, pitch, yaw). A planar base never measures
// z, roll or pitch, so those are given a variance large enough that an EKF
// ignores them rather than one of zero, which it would take as certainty
// TODO: odometry drift grows with distance, so a constant is a placeholder.
// Scale it with distance travelled once the EKF is up and its residuals show
// how far off this is
static constexpr std::array<double, 36> kPoseCovariance = [] {
  constexpr std::array<double, 6> variance{0.01, 0.01, 1e6, 1e6, 1e6, 0.05};

  std::array<double, 36> out{};

  for (size_t i = 0; i < variance.size(); ++i) {
    out[i * 6 + i] = variance[i];
  }

  return out;
}();

static void fill_pose(geometry_msgs__msg__PoseWithCovarianceStamped &msg,
                      const Odometry::Pose &pose) {
  const auto [z, w] = Odometry::heading(pose.theta);

  msg.pose.pose.position.x = pose.x;
  msg.pose.pose.position.y = pose.y;
  msg.pose.pose.position.z = 0.0;
  msg.pose.pose.orientation.x = 0.0;
  msg.pose.pose.orientation.y = 0.0;
  msg.pose.pose.orientation.z = z;
  msg.pose.pose.orientation.w = w;

  std::ranges::copy(kPoseCovariance, msg.pose.covariance);
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

constexpr Drive::Style STYLE = Drive::Style::DIFFERENTIAL;

// one PCNT unit per wheel. 34/35/36/39 are input-only and have no internal
// pull-up, so an open-collector hall output needs an external one
constexpr auto ENCODER_ROWS =
    Encoders::for_style<STYLE>(Encoders::Row{Motor::Name::LEFT, 34, 35, 0},
                               Encoders::Row{Motor::Name::RIGHT, 36, 39, 1});

using EncoderBank =
    Encoders::Bank<ESP32::QuadratureCounter, ENCODER_ROWS, Encoder::fit0485>;

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

using Chassis = Swarm::chassis<STYLE, MotorDriver, SpiBus, Dwm, EncoderBank,
                               UmcDriver, I2cBus, Mpu>;

using CmdVel = ROS::subscribes<geometry_msgs__msg__Twist, "cmd_vel">;

using UmcCmd = ROS::subscribes<std_msgs__msg__Float32, "umc/front">;

using UwbRange = ROS::streams<sensor_msgs__msg__Range, "uwb/range", "uwb_link",
                              double, &fill_range>;

// integrated every control period on the robot, so a slow publish rate loses
// nothing the way sampling a twist slowly would
using OdomPose =
    ROS::streams<geometry_msgs__msg__PoseWithCovarianceStamped, "odom/pose",
                 "odom", Odometry::Pose, &fill_pose>;

using Node = ROS::node<"base", CONFIG_SWARM_ROS_NAMESPACE, CmdVel, UmcCmd,
                       UwbRange, OdomPose>;

using CmdChannel = Sched::channel<Drive::Twist, 4>;
using UmcChannel = Sched::channel<double, 4>;
using PoseMailbox = Sched::channel<Odometry::Pose, 1>;
using YawRateMailbox = Sched::channel<double, 1>;

// motion is arithmetic plus two GPIO writes, so it shares the fast task
constexpr uint32_t MOTION_PERIOD_MS = 5;

// 20ms at X4 is ~150 counts per period at top speed, so the measured velocity
// is not mostly quantisation noise the way it would be at 5ms
constexpr uint32_t DRIVE_PERIOD_MS = 20;

// a robot that has lost its server must not keep driving while the others plan
// around it. Long enough to ride out a WiFi hiccup, so cmd_vel has to be
// streamed faster than this
constexpr uint32_t CMD_VEL_TIMEOUT_MS = 500;

// FIT0485 is a 6V motor on a 12V rail, so the loop never asks for more than
// half duty
// TODO: the rail sags with the battery; measure it under load
constexpr Motor::Supply WHEEL_SUPPLY{.rated = 6.0, .rail = 12.0};

// FIT0485 is listed at 75rpm no-load at 6V; the feedforward only has to be
// close, the integral takes up the rest
// TODO: measure at the rated voltage, and tune the gains
constexpr double MAX_WHEEL_SPEED = 75.0 * 2.0 * std::numbers::pi / 60.0;

constexpr auto CONTROL =
    Drive::control(WHEEL_SUPPLY, MAX_WHEEL_SPEED, DRIVE_PERIOD_MS / 1000.0,
                   Pid::gains(0.05, 0.5, 0.0));
constexpr uint32_t EXECUTOR_PERIOD_MS = 10;
constexpr uint32_t ODOMETRY_PERIOD_MS = 200;

// one yaw rate per control period, so the heading has no gaps to hold across
constexpr uint32_t IMU_PERIOD_MS = DRIVE_PERIOD_MS;

constexpr uint32_t POLL_PERIOD_MS = 100;

// bring-up: flip to false on the responder board
// TODO: make part of idf.py menuconfig
constexpr bool kInitiator = true;
} // namespace HW

static HW::CmdChannel commands;
static HW::UmcChannel umc_commands;
static HW::PoseMailbox poses;
static HW::YawRateMailbox yaw_rates;
static ROS::session<HW::Node> session;

static auto onTwist = [](const geometry_msgs__msg__Twist &twist) {
  const Drive::Twist body{twist.linear.x, twist.linear.y, twist.angular.z};

  if (!commands.push(body, 0)) {
    ESP_LOGW(TAG, "Dropped twist: command channel full");
  }
};

// TODO: maybe make a clean wrapper to handle channel pushing, instead of
// needing onTwist, onUmc, etc. dont like having these separate.
static auto onUmc = [](const std_msgs__msg__Float32 &effort) {
  ESP_LOGI(TAG, "umc/front: %+.2f", static_cast<double>(effort.data));

  if (!umc_commands.push(effort.data, 0)) {
    ESP_LOGW(TAG, "Dropped UMC effort: command channel full");
  }
};

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
      Sched::watched<HW::DRIVE_PERIOD_MS, HW::CMD_VEL_TIMEOUT_MS>(
          commands, Drive::Twist{}) |
          Sched::then(Drive::inverse_kinematics<HW::STYLE>) |
          Sched::to(
              [&motors, &encoders, state = Base::State<HW::STYLE>{}](
                  const Drive::WheelVelocities<HW::STYLE> &targets) mutable {
                const auto measured = Arrays::map(
                    encoders.advance(), [](const Encoder::Reading &r) {
                      return Drive::WheelVelocity{
                          r.name, Encoder::rad_per_second(HW::EncoderBank::spec,
                                                          r.counts,
                                                          HW::CONTROL.period)};
                    });

                // I2C stays out of this loop: a stalled bus empties the mailbox
                // and the heading falls back to the encoders
                const auto [next, frame] = Base::step<HW::STYLE, HW::CONTROL>(
                    state, targets, measured, yaw_rates.pop(0));
                state = next;

                motors.run(frame);
                poses.overwrite(state.pose);
              }),
      Sched::latest<HW::MOTION_PERIOD_MS>(umc_commands) |
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
      Sched::latest<HW::ODOMETRY_PERIOD_MS>(poses) |
      Sched::to([](const Odometry::Pose &pose) {
        // false until the session is up as well, so at warning level this
        // would fire five times a second whenever the agent is away
        if (!session.send<HW::OdomPose>(pose)) {
          ESP_LOGD(TAG, "Dropped pose");
        }
      }));

  // init runs on the uros task, not here: a dead agent must not stop motion
  // from spawning
  static auto ros = Sched::make_task<16000, configMAX_PRIORITIES - 1>(
      // a false from on_start deletes the task, so one lost handshake used to
      // cost cmd_vel until the next power cycle. init() unwinds itself on
      // failure, so retrying is just a delay
      Sched::on_start([] {
        // TODO: this should really be encapsulated and part of ros declaration.
        // See how streams includes the callback in its template parameters
        while (!session.init() || !session.on<HW::CmdVel>(onTwist) ||
               !session.on<HW::UmcCmd>(onUmc)) {
          session.fini();

          ESP_LOGW(TAG, "micro-ROS session init failed; retrying agent %s:%s",
                   CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);

          vTaskDelay(pdMS_TO_TICKS(1000));
        }

        ESP_LOGI(TAG,
                 "micro-ROS session up: /%s/base subscribed to cmd_vel, "
                 "umc/front; publishing odom/pose",
                 CONFIG_SWARM_ROS_NAMESPACE);

        return true;
      }) |
      Sched::every<HW::EXECUTOR_PERIOD_MS> |
      Sched::to([] { return session.poll(); }));

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

  if constexpr (HW::kInitiator) {
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
    // the responder measures t_reply on its own crystal and drift multiplies
    // it, so this task must not be preempted between rx(poll) and tx(reply)
    [[maybe_unused]] static auto responding =
        Sched::make_task<4096, configMAX_PRIORITIES - 1>(
            Sched::every<HW::POLL_PERIOD_MS> |
            Sched::to([&dwm_sensor] { return dwm_sensor.respond(); }));

    // responding.spawn("responding");
  }
}
