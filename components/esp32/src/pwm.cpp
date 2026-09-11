// Written with Claude
#include "esp32.h"

#include "driver/ledc.h"

#include <algorithm>

static constexpr ledc_mode_t MODE = LEDC_LOW_SPEED_MODE;
static constexpr ledc_timer_t TIMER = LEDC_TIMER_0;
static constexpr ledc_timer_bit_t DUTY_RESOLUTION = LEDC_TIMER_10_BIT;
static constexpr uint32_t FREQUENCY_HZ = 1000;

ESP32::PWM::PWM() {
  ledc_timer_config_t timer = {
      .speed_mode = MODE,
      .duty_resolution = DUTY_RESOLUTION,
      .timer_num = TIMER,
      .freq_hz = FREQUENCY_HZ,
      .clk_cfg = LEDC_AUTO_CLK,
      .deconfigure = false,
  };
  ESP_ERROR_CHECK(ledc_timer_config(&timer));
}

void ESP32::PWM::configure_channel(int channel, int pin) {
  ledc_channel_config_t config = {
      .gpio_num = pin,
      .speed_mode = MODE,
      .channel = static_cast<ledc_channel_t>(channel),
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = TIMER,
      .duty = 0,
      .hpoint = 0,
      .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
      .flags = {.output_invert = 0},
  };
  ESP_ERROR_CHECK(ledc_channel_config(&config));
}

void ESP32::PWM::set_duty_ratio(int channel, double ratio) {
  const uint32_t max_duty = (1U << DUTY_RESOLUTION) - 1U;
  const uint32_t duty = std::clamp(ratio, 0.0, 1.0) * max_duty;

  ESP_ERROR_CHECK(ledc_set_duty(MODE, static_cast<ledc_channel_t>(channel), duty));
  ESP_ERROR_CHECK(ledc_update_duty(MODE, static_cast<ledc_channel_t>(channel)));
}
