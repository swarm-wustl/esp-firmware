#include "esp32.h"

#include "error.h"
#include "hardware.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/i2c_master.h"

#include <algorithm>
#include <cmath>
#include <cstring>

// ~10 cm as of 2026-01-24
#define WHEELBASE 0.10 // dist between wheels (m)
#define ANGULAR_MULTIPLIER 3.0

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT
#define LEDC_FREQUENCY 1000 // Hz

#define LEFT_MOTOR_CHANNEL LEDC_CHANNEL_0
#define RIGHT_MOTOR_CHANNEL LEDC_CHANNEL_1
#define LOWER_LEFT_MOTOR_CHANNEL LEDC_CHANNEL_2
#define LOWER_RIGHT_MOTOR_CHANNEL LEDC_CHANNEL_3

// u3
#define ENA GPIO_NUM_19
#define IN1 GPIO_NUM_26
#define IN2 GPIO_NUM_27

#define ENB GPIO_NUM_25
#define IN3 GPIO_NUM_32
#define IN4 GPIO_NUM_33

// u2
#define ENC GPIO_NUM_4
#define IN5 GPIO_NUM_16
#define IN6 GPIO_NUM_17

#define END GPIO_NUM_5
#define IN7 GPIO_NUM_22
#define IN8 GPIO_NUM_23

#define STBY GPIO_NUM_0
#define STBY_LOWER GPIO_NUM_25

#define GPIO_BITMASK                                                           \
  ((1ULL << ENA) | (1ULL << IN1) | (1ULL << IN2) | (1ULL << ENB) |             \
   (1ULL << IN3) | (1ULL << IN4) | (1ULL << ENC) | (1ULL << IN5) |             \
   (1ULL << IN6) | (1ULL << END) | (1ULL << IN7) | (1ULL << IN8) |             \
   (1ULL << STBY) | (1ULL << STBY_LOWER))

static void setup_pwm() {
  // LEDC timer is used for LED dimming via PWM
  // but works well for motor control
  ledc_timer_config_t ledc_timer = {.speed_mode = LEDC_MODE,
									.duty_resolution = LEDC_DUTY_RES,
									.timer_num = LEDC_TIMER,
									.freq_hz = LEDC_FREQUENCY,
									.clk_cfg = LEDC_AUTO_CLK,
									.deconfigure = false};
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  // Motor to pin mapping
  struct MotorConfig {
    gpio_num_t pin;
    ledc_channel_t channel;
  };

  MotorConfig motors[] = {{ENA, LEFT_MOTOR_CHANNEL},
                          {ENB, RIGHT_MOTOR_CHANNEL},
                          {ENC, LOWER_LEFT_MOTOR_CHANNEL},
                          {END, LOWER_RIGHT_MOTOR_CHANNEL}};

  // Configure motors
  for (const auto &motor : motors) {
    ledc_channel_config_t channel_config = {.gpio_num = motor.pin,
                                            .speed_mode = LEDC_LOW_SPEED_MODE,
                                            .channel = motor.channel,
                                            .intr_type = LEDC_INTR_DISABLE,
                                            .timer_sel = LEDC_TIMER_0,
                                            .duty = 0,
                                            .hpoint = 0,
                                            .flags = {.output_invert = 0}};
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
  }
}

static uint32_t compute_duty_cycle(float pwm_ratio) {
  // [0, 1] --> [0, (2 ^ resolution) - 1]
  return pwm_ratio * ((1 << LEDC_DUTY_RES) - 1);
}

ESP32::L298NMotorDriver::L298NMotorDriver() {
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = GPIO_BITMASK;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&io_conf);

  setup_pwm();
}

void ESP32::L298NMotorDriver::run(const Motor::Command &cmd) {
  uint32_t level_a, level_b;

  switch (cmd.dir) {
  case Motor::Direction::FORWARD:
    level_a = HAL::to_level(HAL::Voltage::HIGH);
    level_b = HAL::to_level(HAL::Voltage::LOW);
    break;

  case Motor::Direction::REVERSE:
    level_a = HAL::to_level(HAL::Voltage::LOW);
    level_b = HAL::to_level(HAL::Voltage::HIGH);
    break;

  default:
    level_a = HAL::to_level(HAL::Voltage::LOW);
    level_b = HAL::to_level(HAL::Voltage::LOW);
    break;
  }

  gpio_num_t pin_a, pin_b;
  ledc_channel_t pwm_channel;

  switch (cmd.name) {
  case Motor::Name::LEFT:
    pin_a = IN1;
    pin_b = IN2;
    pwm_channel = LEFT_MOTOR_CHANNEL;
    break;

  case Motor::Name::RIGHT:
    pin_a = IN3;
    pin_b = IN4;
    pwm_channel = RIGHT_MOTOR_CHANNEL;
    break;

  case Motor::Name::LOWER_LEFT:
    pin_a = IN5;
    pin_b = IN6;
    pwm_channel = LOWER_LEFT_MOTOR_CHANNEL;
    log("ran motor lower left");
    break;

  case Motor::Name::LOWER_RIGHT:
    pin_a = IN7;
    pin_b = IN8;
    pwm_channel = LOWER_RIGHT_MOTOR_CHANNEL;
    log("ran motor lower right");
    break;

  default:
    log("Unable to run motor command: unknown channel name");
    return;
  }

  gpio_set_level(pin_a, level_a);
  gpio_set_level(pin_b, level_b);
  gpio_set_level(STBY, 1);
  gpio_set_level(STBY_LOWER, 1);

  uint32_t duty_cycle = compute_duty_cycle(cmd.pwm_ratio);
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, pwm_channel, duty_cycle));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, pwm_channel));

  log("ran motor!");
}

template <>
std::array<Motor::Command, HW::MOTOR_COUNT>
ESP32::DifferentialDriveController::convert_twist(
    geometry_msgs__msg__Twist msg) {

  auto [target_L, target_R] =
      calculate_drive_targets(msg.linear.x, msg.angular.z);

  // Overload linear y (strafing) to rotate the front face
  double target_lower =
      (msg.linear.y == 0.0) ? 0.0 : std::copysign(1.0, msg.linear.y);

  return std::array<Motor::Command, HW::MOTOR_COUNT>{
      create_command(Motor::Name::LEFT, target_L),
      create_command(Motor::Name::RIGHT, -target_R),
      create_command(Motor::Name::LOWER_LEFT, target_lower),
      create_command(Motor::Name::LOWER_RIGHT, target_lower),
  };
}

std::pair<double, double>
ESP32::DifferentialDriveController::calculate_drive_targets(double linear_x,
                                                            double angular_z) {
  // v_R = v + (L * w) / 2
  // v_L = v - (L * w) / 2
  double angular_component = (ANGULAR_MULTIPLIER * angular_z * WHEELBASE) / 2.0;

  return {
      linear_x - angular_component, // Left
      linear_x + angular_component  // Right
  };
}

Motor::Command
ESP32::DifferentialDriveController::create_command(Motor::Name name,
                                                   double value) {
  Motor::Direction dir = Motor::Direction::STOP;

  if (value > 0) {
    dir = Motor::Direction::FORWARD;
  } else if (value < 0) {
    dir = Motor::Direction::REVERSE;
  }

  // TODO: PID controller
  double pwm = std::clamp(std::abs(value), 0.0, 1.0);

  return Motor::Command{name, dir, pwm};
}

// MPU6050 Register addresses
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B

// Scale factors for default settings (±2g accel, ±250°/s gyro)
#define ACCEL_SCALE (9.81f / 16384.0f)    // Convert to m/s² (±2g range)
#define GYRO_SCALE  (3.14159265f / 180.0f / 131.0f)  // Convert to rad/s (±250°/s range)

ESP32::MPU6050Driver::MPU6050Driver()
    : bus_handle(nullptr), dev_handle(nullptr), initialized(false) {}

bool ESP32::MPU6050Driver::initialize() {
  if (initialized) {
    return true;
  }

  // Configure I2C master bus
  i2c_master_bus_config_t bus_config = {};
  bus_config.i2c_port = I2C_NUM_0;
  bus_config.sda_io_num = I2C_SDA_PIN;
  bus_config.scl_io_num = I2C_SCL_PIN;
  bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_config.glitch_ignore_cnt = 7;
  bus_config.flags.enable_internal_pullup = true;

  esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle);
  if (err != ESP_OK) {
    log("Failed to create I2C master bus: %d", err);
    return false;
  }

  // Configure MPU6050 device
  i2c_device_config_t dev_config = {};
  dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dev_config.device_address = MPU6050_ADDR;
  dev_config.scl_speed_hz = 400000;  // 400kHz

  err = i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle);
  if (err != ESP_OK) {
    log("Failed to add MPU6050 device: %d", err);
    return false;
  }

  // Wake up MPU6050 (write 0x00 to PWR_MGMT_1)
  uint8_t wake_cmd[] = {MPU6050_REG_PWR_MGMT_1, 0x00};
  err = i2c_master_transmit(dev_handle, wake_cmd, sizeof(wake_cmd), 100);
  if (err != ESP_OK) {
    log("Failed to wake MPU6050: %d", err);
    return false;
  }

  // Configure gyroscope (±250°/s, default)
  uint8_t gyro_cfg[] = {MPU6050_REG_GYRO_CONFIG, 0x00};
  err = i2c_master_transmit(dev_handle, gyro_cfg, sizeof(gyro_cfg), 100);
  if (err != ESP_OK) {
    log("Failed to configure gyroscope: %d", err);
    return false;
  }

  // Configure accelerometer (±2g, default)
  uint8_t accel_cfg[] = {MPU6050_REG_ACCEL_CONFIG, 0x00};
  err = i2c_master_transmit(dev_handle, accel_cfg, sizeof(accel_cfg), 100);
  if (err != ESP_OK) {
    log("Failed to configure accelerometer: %d", err);
    return false;
  }

  initialized = true;
  log("MPU6050 initialized successfully");
  return true;
}

HAL::IMUData ESP32::MPU6050Driver::read() {
  HAL::IMUData data = {0, 0, 0, 0, 0, 0};

  if (!initialized) {
    return data;
  }

  // Read 14 bytes starting from ACCEL_XOUT_H (accel: 6 bytes, temp: 2 bytes, gyro: 6 bytes)
  uint8_t reg = MPU6050_REG_ACCEL_XOUT_H;
  uint8_t buffer[14];

  esp_err_t err = i2c_master_transmit_receive(dev_handle, &reg, 1, buffer, sizeof(buffer), 100);
  if (err != ESP_OK) {
    log("Failed to read MPU6050 data: %d", err);
    return data;
  }

  // Parse accelerometer data (big-endian)
  int16_t accel_x_raw = (buffer[0] << 8) | buffer[1];
  int16_t accel_y_raw = (buffer[2] << 8) | buffer[3];
  int16_t accel_z_raw = (buffer[4] << 8) | buffer[5];
  // buffer[6] and buffer[7] are temperature, skip

  // Parse gyroscope data (big-endian)
  int16_t gyro_x_raw = (buffer[8] << 8) | buffer[9];
  int16_t gyro_y_raw = (buffer[10] << 8) | buffer[11];
  int16_t gyro_z_raw = (buffer[12] << 8) | buffer[13];

  // Convert to physical units
  data.accel_x = accel_x_raw * ACCEL_SCALE;
  data.accel_y = accel_y_raw * ACCEL_SCALE;
  data.accel_z = accel_z_raw * ACCEL_SCALE;

  data.gyro_x = gyro_x_raw * GYRO_SCALE;
  data.gyro_y = gyro_y_raw * GYRO_SCALE;
  data.gyro_z = gyro_z_raw * GYRO_SCALE;

  return data;
}
