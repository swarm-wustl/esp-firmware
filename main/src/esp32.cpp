#include "esp32.h"

#include "error.h"
#include "hardware.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"

#include <algorithm>
#include <cmath>

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
      create_command(Motor::Name::RIGHT, target_R),
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
