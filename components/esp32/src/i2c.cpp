#include "esp32.h"

#include "esp_log.h"

static const char *TAG = "i2c";

namespace ESP32::detail {
static HAL::I2cError from_esp_err(esp_err_t err) {
  switch (err) {
  case ESP_ERR_TIMEOUT:
    return HAL::I2cError::Timeout;
  case ESP_ERR_INVALID_STATE:
  case ESP_FAIL:
    return HAL::I2cError::Nack;
  default:
    return HAL::I2cError::TransferFailed;
  }
}

i2c_master_bus_handle_t i2c_open_bus(int port, int sda, int scl) {
  i2c_master_bus_config_t config{};
  config.i2c_port = port;
  config.sda_io_num = static_cast<gpio_num_t>(sda);
  config.scl_io_num = static_cast<gpio_num_t>(scl);
  config.clk_source = I2C_CLK_SRC_DEFAULT;
  // 7 cycles of the source clock: the IDF default, short enough to pass a
  // 400 kHz edge and long enough to swallow bus chatter
  config.glitch_ignore_cnt = 7;
  config.flags.enable_internal_pullup = true;

  i2c_master_bus_handle_t handle{};

  if (esp_err_t err = i2c_new_master_bus(&config, &handle); err != ESP_OK) {
    ESP_LOGE(TAG, "bus %d init failed: %s", port, esp_err_to_name(err));

    return nullptr;
  }

  return handle;
}

void i2c_close_bus(i2c_master_bus_handle_t bus) {
  if (bus != nullptr) {
    i2c_del_master_bus(bus);
  }
}

i2c_master_dev_handle_t i2c_add_device(i2c_master_bus_handle_t bus,
                                       uint8_t address, uint32_t speed_hz) {
  if (bus == nullptr) {
    return nullptr;
  }

  i2c_device_config_t config{};
  config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  config.device_address = address;
  config.scl_speed_hz = speed_hz;

  i2c_master_dev_handle_t device{};

  if (esp_err_t err = i2c_master_bus_add_device(bus, &config, &device);
      err != ESP_OK) {
    ESP_LOGE(TAG, "device 0x%02X add failed: %s", address,
             esp_err_to_name(err));

    return nullptr;
  }

  return device;
}

void i2c_remove_device(i2c_master_dev_handle_t device) {
  if (device != nullptr) {
    i2c_master_bus_rm_device(device);
  }
}

std::expected<void, HAL::I2cError> i2c_write(i2c_master_dev_handle_t device,
                                             std::span<const std::byte> tx,
                                             int timeout_ms) {
  if (device == nullptr) {
    return std::unexpected(HAL::I2cError::TransferFailed);
  }

  const esp_err_t err = i2c_master_transmit(
      device, reinterpret_cast<const uint8_t *>(tx.data()), tx.size(),
      timeout_ms);

  if (err != ESP_OK) {
    return std::unexpected(from_esp_err(err));
  }

  return {};
}

std::expected<void, HAL::I2cError>
i2c_write_read(i2c_master_dev_handle_t device, std::span<const std::byte> tx,
               std::span<std::byte> rx, int timeout_ms) {
  if (device == nullptr) {
    return std::unexpected(HAL::I2cError::TransferFailed);
  }

  const esp_err_t err = i2c_master_transmit_receive(
      device, reinterpret_cast<const uint8_t *>(tx.data()), tx.size(),
      reinterpret_cast<uint8_t *>(rx.data()), rx.size(), timeout_ms);

  if (err != ESP_OK) {
    return std::unexpected(from_esp_err(err));
  }

  return {};
}
} // namespace ESP32::detail
