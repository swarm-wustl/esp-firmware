#include "esp32.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include <driver/ledc.h>

#define WHEELBASE 0.5   // dist between wheels (m) (we're gonna fake it)

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT
#define LEDC_FREQUENCY 1000 // Hz

#define LEFT_MOTOR_CHANNEL LEDC_CHANNEL_0
#define RIGHT_MOTOR_CHANNEL LEDC_CHANNEL_1

#define INNER_LEFT_MOTOR_CHANNEL LEDC_CHANNEL_3
#define INNER_RIGHT_MOTOR_CHANNEL LEDC_CHANNEL_3

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

static double compute_duty_cycle(float pwm_ratio) {
    // [0, 1] --> [0, (2 ^ resolution) - 1]
    return pwm_ratio * ((1 << LEDC_DUTY_RES) - 1);
}