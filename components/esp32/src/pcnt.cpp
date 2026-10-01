#include "esp32.h"

#include "esp_log.h"

#include <utility>

namespace {
constexpr const char *TAG = "pcnt";
}

ESP32::QuadratureCounter::QuadratureCounter(Swarm::Assembly) {}

ESP32::QuadratureCounter::~QuadratureCounter() {
  for (pcnt_unit_handle_t &unit : units_) {
    if (unit == nullptr) {
      continue;
    }

    pcnt_unit_stop(unit);
    pcnt_unit_disable(unit);
    pcnt_del_unit(unit);
    unit = nullptr;
  }
}

ESP32::QuadratureCounter::QuadratureCounter(QuadratureCounter &&other) {
  swap(other);
}

ESP32::QuadratureCounter &
ESP32::QuadratureCounter::operator=(QuadratureCounter &&other) {
  if (this != &other) {
    swap(other);
  }

  return *this;
}

void ESP32::QuadratureCounter::swap(QuadratureCounter &other) {
  std::swap(units_, other.units_);
}

void ESP32::QuadratureCounter::configure_unit(int unit, HAL::Pin a,
                                              HAL::Pin b) {
  const size_t slot = static_cast<size_t>(unit);

  if (slot >= max_units || units_[slot] != nullptr) {
    ESP_LOGE(TAG, "unit %d unavailable", unit);
    return;
  }

  pcnt_unit_config_t config = {
      .low_limit = low_limit,
      .high_limit = high_limit,
      .intr_priority = 0,
      .flags = {.accum_count = 1},
  };

  if (pcnt_new_unit(&config, &units_[slot]) != ESP_OK) {
    ESP_LOGE(TAG, "unit %d create failed", unit);
    units_[slot] = nullptr;
    return;
  }

  const pcnt_glitch_filter_config_t filter = {.max_glitch_ns =
                                                  glitch_filter_ns};
  pcnt_unit_set_glitch_filter(units_[slot], &filter);

  // accumulation only happens on a watch point, so the limits have to be ones
  pcnt_unit_add_watch_point(units_[slot], low_limit);
  pcnt_unit_add_watch_point(units_[slot], high_limit);

  const pcnt_chan_config_t on_a = {
      .edge_gpio_num = a.number(),
      .level_gpio_num = b.number(),
      .flags = {},
  };

  const pcnt_chan_config_t on_b = {
      .edge_gpio_num = b.number(),
      .level_gpio_num = a.number(),
      .flags = {},
  };

  pcnt_channel_handle_t channel_a = nullptr;
  pcnt_channel_handle_t channel_b = nullptr;

  pcnt_new_channel(units_[slot], &on_a, &channel_a);
  pcnt_new_channel(units_[slot], &on_b, &channel_b);

  // the pair of edge actions gives direction; the level action flips the sign
  // when the other signal is low, which is what makes it count four per period
  pcnt_channel_set_edge_action(channel_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                               PCNT_CHANNEL_EDGE_ACTION_INCREASE);
  pcnt_channel_set_level_action(channel_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

  pcnt_channel_set_edge_action(channel_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                               PCNT_CHANNEL_EDGE_ACTION_DECREASE);
  pcnt_channel_set_level_action(channel_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

  pcnt_unit_enable(units_[slot]);
  pcnt_unit_clear_count(units_[slot]);
  pcnt_unit_start(units_[slot]);
}

int32_t ESP32::QuadratureCounter::count(int unit) {
  const size_t slot = static_cast<size_t>(unit);

  if (slot >= max_units || units_[slot] == nullptr) {
    return 0;
  }

  int value = 0;
  pcnt_unit_get_count(units_[slot], &value);

  return static_cast<int32_t>(value);
}

void ESP32::QuadratureCounter::clear(int unit) {
  const size_t slot = static_cast<size_t>(unit);

  if (slot >= max_units || units_[slot] == nullptr) {
    return;
  }

  pcnt_unit_clear_count(units_[slot]);
}
