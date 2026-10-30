#include "esp32.h"

#include "esp_err.h"

namespace ESP32::detail {
void gpio_on_rising_edge(int pin, void (*isr)(void *), void *arg) {
  const auto number = static_cast<gpio_num_t>(pin);

  // the service is per-chip and a second install reports INVALID_STATE, which
  // only means another pin got here first
  if (const esp_err_t err = gpio_install_isr_service(0);
      err != ESP_ERR_INVALID_STATE) {
    ESP_ERROR_CHECK(err);
  }

  const gpio_config_t config{
      .pin_bit_mask = 1ULL << pin,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
      .intr_type = GPIO_INTR_POSEDGE,
  };

  ESP_ERROR_CHECK(gpio_config(&config));
  ESP_ERROR_CHECK(gpio_isr_handler_add(number, isr, arg));
}
} // namespace ESP32::detail
