#include "esp32.h"

#include "hardware.h"
#include "error.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"

#include <algorithm>

// ~10 cm as of 2026-01-24
#define WHEELBASE 0.10 // dist between wheels (m)
#define ANGULAR_MULTIPLIER 2.0

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT
#define LEDC_FREQUENCY 1000 // Hz

#define LEFT_MOTOR_CHANNEL LEDC_CHANNEL_0
#define RIGHT_MOTOR_CHANNEL LEDC_CHANNEL_1
#define LOWER_LEFT_MOTOR_CHANNEL LEDC_CHANNEL_2
#define LOWER_RIGHT_MOTOR_CHANNEL LEDC_CHANNEL_3

#define ENA GPIO_NUM_4
#define IN1 GPIO_NUM_16
#define IN2 GPIO_NUM_17

#define ENB GPIO_NUM_5
#define IN3 GPIO_NUM_18
#define IN4 GPIO_NUM_19

#define ENC GPIO_NUM_26
#define IN5 GPIO_NUM_32
#define IN6 GPIO_NUM_33

#define END GPIO_NUM_27
#define IN7 GPIO_NUM_14
#define IN8 GPIO_NUM_15

#define STBY GPIO_NUM_0
#define STBY_LOWER GPIO_NUM_25

#define GPIO_BITMASK                                                           \
  ((1ULL << ENA) | (1ULL << IN1) | (1ULL << IN2) | (1ULL << ENB) |             \
   (1ULL << IN3) | (1ULL << IN4) | (1ULL << ENC) | (1ULL << IN5) |             \
   (1ULL << IN6) | (1ULL << END) | (1ULL << IN7) | (1ULL << IN8) |             \
   (1ULL << STBY) | (1ULL << STBY_LOWER))

#define FLOAT_TOLERANCE 0.01

static void setup_pwm() {
  ledc_timer_config_t ledc_timer = {
      .speed_mode = LEDC_MODE,
      .duty_resolution = LEDC_DUTY_RES,
      .timer_num = LEDC_TIMER,
      .freq_hz = LEDC_FREQUENCY,
      .clk_cfg = LEDC_AUTO_CLK,
      .deconfigure      = false // Preston originally had this commented out -- not sure why
  };
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  ledc_channel_config_t ledc_channel_left_motor = {
      .gpio_num = ENA,                   // Assign GPIO pin
      .speed_mode = LEDC_LOW_SPEED_MODE, // LEDC low-speed mode
      .channel = LEFT_MOTOR_CHANNEL,     // Set motor channel
      .intr_type = LEDC_INTR_DISABLE,    // Disable interrupts
      .timer_sel = LEDC_TIMER_0,         // Use timer 0
      .duty = 0,                         // Initial duty cycle (0%)
      .hpoint = 0,                       // Default hpoint value
      .flags = {.output_invert = 0}      // No output inversion
  };
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_left_motor));

  ledc_channel_config_t ledc_channel_right_motor = {
      .gpio_num = ENB,                   // Assign GPIO pin
      .speed_mode = LEDC_LOW_SPEED_MODE, // LEDC low-speed mode
      .channel = RIGHT_MOTOR_CHANNEL,    // Set motor channel
      .intr_type = LEDC_INTR_DISABLE,    // Disable interrupts
      .timer_sel = LEDC_TIMER_0,         // Use timer 0
      .duty = 0,                         // Initial duty cycle (0%)
      .hpoint = 0,                       // Default hpoint value
      .flags = {.output_invert = 0}      // No output inversion
  };
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_right_motor));

  ledc_channel_config_t ledc_channel_lower_left_motor = {
      .gpio_num = ENC,                     // Assign GPIO pin
      .speed_mode = LEDC_LOW_SPEED_MODE,   // LEDC low-speed mode
      .channel = LOWER_LEFT_MOTOR_CHANNEL, // Set motor channel
      .intr_type = LEDC_INTR_DISABLE,      // Disable interrupts
      .timer_sel = LEDC_TIMER_0,           // Use timer 0
      .duty = 0,                           // Initial duty cycle (0%)
      .hpoint = 0,                         // Default hpoint value
      .flags = {.output_invert = 0}        // No output inversion
  };
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_lower_left_motor));

  ledc_channel_config_t ledc_channel_lower_right_motor = {
      .gpio_num = END,                      // Assign GPIO pin
      .speed_mode = LEDC_LOW_SPEED_MODE,    // LEDC low-speed mode
      .channel = LOWER_RIGHT_MOTOR_CHANNEL, // Set motor channel
      .intr_type = LEDC_INTR_DISABLE,       // Disable interrupts
      .timer_sel = LEDC_TIMER_0,            // Use timer 0
      .duty = 0,                            // Initial duty cycle (0%)
      .hpoint = 0,                          // Default hpoint value
      .flags = {.output_invert = 0}         // No output inversion
  };
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_lower_right_motor));
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
std::array<Motor::Command, HW::MOTOR_COUNT> ESP32::DifferentialDriveController::convert_twist(
    geometry_msgs__msg__Twist msg) {
  // TODO: this should set target RPMs for each motor and use PID with encoder
  // feedback to ensure that that's being met Linear and angular velocity are
  // given in m/s and rad/s from ROS2 The units here are all kind of fake
  double linear_velocity = msg.linear.x;
  double angular_velocity = msg.angular.z; // yaw

  Motor::Direction dir_R;
  Motor::Direction dir_L;

  // v_R = v + \frac{L\omega}{2}
  // v_L = v - \frac{L\omega}{2}
  double pwm_ratio_R =
      linear_velocity + ANGULAR_MULTIPLIER * angular_velocity * WHEELBASE / 2;
  double pwm_ratio_L =
      linear_velocity - ANGULAR_MULTIPLIER * angular_velocity * WHEELBASE / 2;

  if (pwm_ratio_R > 0) {
    dir_R = Motor::Direction::FORWARD;
  } else if (pwm_ratio_R < 0) {
    dir_R = Motor::Direction::REVERSE;
  } else {
    dir_R = Motor::Direction::STOP;
  }

  if (pwm_ratio_L > 0) {
    dir_L = Motor::Direction::FORWARD;
  } else if (pwm_ratio_L < 0) {
    dir_L = Motor::Direction::REVERSE;
  } else {
    dir_L = Motor::Direction::STOP;
  }

  // PWM ratio must be [0, 1]
  pwm_ratio_R = std::clamp(std::abs(pwm_ratio_R), 0.0, 1.0);
  pwm_ratio_L = std::clamp(std::abs(pwm_ratio_L), 0.0, 1.0);

  /// Lower motors
  double front_turn_velocity = msg.linear.y;

  Motor::Direction dir_R_lower;
  Motor::Direction dir_L_lower;
  double pwm_ratio_L_lower = 0.5;
  double pwm_ratio_R_lower = 0.5;

  if (front_turn_velocity > 0) {
	dir_R_lower = Motor::Direction::FORWARD;
	dir_L_lower = Motor::Direction::FORWARD;
  } else if (front_turn_velocity < 0) {
	dir_R_lower = Motor::Direction::REVERSE;
	dir_L_lower = Motor::Direction::REVERSE;
  } else {
	dir_R_lower = Motor::Direction::STOP;
	dir_L_lower = Motor::Direction::STOP;
  }

  return std::array<Motor::Command, HW::MOTOR_COUNT>{
      Motor::Command{Motor::Name::LEFT, dir_L, pwm_ratio_L},
      Motor::Command{Motor::Name::RIGHT, dir_R, pwm_ratio_R},
      Motor::Command{Motor::Name::LOWER_LEFT, dir_L_lower, pwm_ratio_L_lower},
      Motor::Command{Motor::Name::LOWER_RIGHT, dir_R_lower, pwm_ratio_R_lower},
  };
}
