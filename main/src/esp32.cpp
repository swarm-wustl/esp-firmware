#include "esp32.h"

#include "error.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/ledc.h"

#define WHEELBASE 0.5   // dist between wheels (m) (we're gonna fake it)

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT
#define LEDC_FREQUENCY 1000 // Hz

#define LEFT_MOTOR_CHANNEL LEDC_CHANNEL_0
#define RIGHT_MOTOR_CHANNEL LEDC_CHANNEL_1

#define ENA GPIO_NUM_19
#define IN1 GPIO_NUM_22
#define IN2 GPIO_NUM_3 

#define ENB GPIO_NUM_18
#define IN3 GPIO_NUM_1
#define IN4 GPIO_NUM_21

#define STBY GPIO_NUM_23

#define GPIO_BITMASK ( \
    (1ULL << ENA) | (1ULL << IN1) | (1ULL << IN2) | \
    (1ULL << ENB) | (1ULL << IN3) | (1ULL << IN4) | \
    (1ULL << STBY) \
)

#define FLOAT_TOLERANCE 0.01

static void setup_pwm() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK,
        .deconfigure      = false
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel_left_motor = {
        .gpio_num       = ENA,                      // Assign GPIO pin
        .speed_mode     = LEDC_LOW_SPEED_MODE,      // LEDC low-speed mode
        .channel        = LEFT_MOTOR_CHANNEL,       // Set motor channel
        .intr_type      = LEDC_INTR_DISABLE,        // Disable interrupts
        .timer_sel      = LEDC_TIMER_0,             // Use timer 0
        .duty           = 0,                        // Initial duty cycle (0%)
        .hpoint         = 0,                        // Default hpoint value
        .flags          = { .output_invert = 0 }    // No output inversion
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_left_motor));

    ledc_channel_config_t ledc_channel_right_motor = {
        .gpio_num       = ENB,                      // Assign GPIO pin
        .speed_mode     = LEDC_LOW_SPEED_MODE,      // LEDC low-speed mode
        .channel        = RIGHT_MOTOR_CHANNEL,      // Set motor channel
        .intr_type      = LEDC_INTR_DISABLE,        // Disable interrupts
        .timer_sel      = LEDC_TIMER_0,             // Use timer 0
        .duty           = 0,                        // Initial duty cycle (0%)
        .hpoint         = 0,                        // Default hpoint value
        .flags          = { .output_invert = 0 }    // No output inversion
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_right_motor));
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

void ESP32::L298NMotorDriver::run(const Motor::Command& cmd) {
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

        default:
            log("Unable to run motor command: unknown channel name");
            return;
    }

    gpio_set_level(pin_a, level_a);
    gpio_set_level(pin_b, level_b);
    gpio_set_level(STBY, 1);

    uint32_t duty_cycle = compute_duty_cycle(cmd.pwm_ratio);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, pwm_channel, duty_cycle));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, pwm_channel));

    log("ran motor!");
}

template <>
void ESP32::DriveController::convert_twist<2>(std::array<Motor::Command, 2> cmd_list) {
    // TODO
}