#include "adc.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "soc/soc_caps.h"

#include <array>

namespace ESP32 {
namespace {

constexpr const char *TAG = "adc1";

// 64 samples to average out the ESP32 ADC's noise
constexpr int SAMPLES = 64;

// 12 dB reads up to roughly 3.1 V; nothing past that is trustworthy
constexpr adc_atten_t ATTEN = ADC_ATTEN_DB_12;
constexpr adc_bitwidth_t WIDTH = ADC_BITWIDTH_12;

struct State {
  adc_oneshot_unit_handle_t unit = nullptr;
  adc_cali_handle_t cali = nullptr;
  std::array<bool, SOC_ADC_MAX_CHANNEL_NUM> configured{};
};

State state;

adc_cali_handle_t make_calibration() {
  adc_cali_handle_t handle = nullptr;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  const adc_cali_curve_fitting_config_t config{
      .unit_id = ADC_UNIT_1,
      .chan = ADC_CHANNEL_0,
      .atten = ATTEN,
      .bitwidth = WIDTH,
  };

  if (adc_cali_create_scheme_curve_fitting(&config, &handle) != ESP_OK) {
    handle = nullptr;
  }
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  const adc_cali_line_fitting_config_t config{
      .unit_id = ADC_UNIT_1,
      .atten = ATTEN,
      .bitwidth = WIDTH,
      .default_vref = 1100,
  };

  if (adc_cali_create_scheme_line_fitting(&config, &handle) != ESP_OK) {
    handle = nullptr;
  }
#endif

  return handle;
}

bool ensure_unit() {
  if (state.unit != nullptr) {
    return true;
  }

  const adc_oneshot_unit_init_cfg_t config{
      .unit_id = ADC_UNIT_1,
      .clk_src = static_cast<adc_oneshot_clk_src_t>(0),
      .ulp_mode = ADC_ULP_MODE_DISABLE,
  };

  if (adc_oneshot_new_unit(&config, &state.unit) != ESP_OK) {
    ESP_LOGE(TAG, "ADC1 unit init failed");
    state.unit = nullptr;

    return false;
  }

  state.cali = make_calibration();

  if (state.cali == nullptr) {
    ESP_LOGW(TAG, "no ADC calibration scheme; readings are uncalibrated");
  }

  return true;
}

} // namespace

std::optional<double> Adc1::read_volts(int gpio) {
  if (!ensure_unit()) {
    return std::nullopt;
  }

  adc_unit_t unit;
  adc_channel_t channel;

  if (adc_oneshot_io_to_channel(gpio, &unit, &channel) != ESP_OK ||
      unit != ADC_UNIT_1) {
    ESP_LOGE(TAG, "GPIO%d is not an ADC1 pin", gpio);

    return std::nullopt;
  }

  if (!state.configured[channel]) {
    const adc_oneshot_chan_cfg_t config{.atten = ATTEN, .bitwidth = WIDTH};

    if (adc_oneshot_config_channel(state.unit, channel, &config) != ESP_OK) {
      ESP_LOGE(TAG, "GPIO%d channel config failed", gpio);

      return std::nullopt;
    }

    state.configured[channel] = true;
  }

  int sum = 0;

  for (int i = 0; i < SAMPLES; ++i) {
    int raw = 0;

    if (adc_oneshot_read(state.unit, channel, &raw) != ESP_OK) {
      return std::nullopt;
    }

    sum += raw;
  }

  const int average = sum / SAMPLES;

  if (state.cali == nullptr) {
    // uncalibrated fallback: full scale is about 3.1 V at 12 dB
    return average * 3.1 / ((1 << WIDTH) - 1);
  }

  int millivolts = 0;

  if (adc_cali_raw_to_voltage(state.cali, average, &millivolts) != ESP_OK) {
    return std::nullopt;
  }

  return millivolts / 1000.0;
}

} // namespace ESP32
