#include "driver/spi_common.h"
#include "esp32.h"

#include "esp_log.h"
#include <algorithm>
#include <memory>
#include <ranges>
#include <utility>
#include <vector>

static const char *TAG = "spi";

namespace ESP32 {
constexpr int SPI_SCK = SpiBus::sck;
constexpr int SPI_MISO = SpiBus::miso;
constexpr int SPI_MOSI = SpiBus::mosi;
constexpr spi_host_device_t HOST = SpiBus::host;

inline constexpr size_t BYTES_TO_BITS(size_t bytes) { return bytes * 8; }

static HAL::SpiError from_esp_err(esp_err_t err) {
  switch (err) {
  case ESP_ERR_TIMEOUT:
    return HAL::SpiError::Timeout;
  default:
    return HAL::SpiError::TransferFailed;
  }
}

SpiBus::SpiBus(Swarm::Assembly) : owns_bus_{true} {
  spi_bus_config_t config{
      .mosi_io_num = SPI_MOSI,
      .miso_io_num = SPI_MISO,
      .sclk_io_num = SPI_SCK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .data4_io_num = -1,
      .data5_io_num = -1,
      .data6_io_num = -1,
      .data7_io_num = -1,
      .max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE,
      .flags = SPICOMMON_BUSFLAG_MASTER,        // TODO
      .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO, // TODO
      .intr_flags = 0,                          // TODO
  };

  // TODO: throw?
  ESP_LOGI(TAG, "SPI init: %d",
           spi_bus_initialize(HOST, &config, SPI_DMA_CH_AUTO));
}

SpiBus::~SpiBus() {
  if (owns_bus_) {
    ESP_LOGI(TAG, "SPI deinit: %d", spi_bus_free(HOST));
  }
}

SpiBus::SpiBus(SpiBus &&other) : owns_bus_{std::exchange(other.owns_bus_, false)} {}

SpiBus &SpiBus::operator=(SpiBus &&other) {
  if (this != &other) {
    SpiBus temp{std::move(other)};
    swap(temp);
  }

  return *this;
}

void SpiBus::swap(SpiBus &other) { std::swap(owns_bus_, other.owns_bus_); }

SPI::SPI(Swarm::Assembly, SpiBus &, HAL::Pin cs) : cs_{cs.number()} {
  spi_device_interface_config_t dev_config{
      // Command and address bits are for specific command and address phases of
      // SPI
      // These exist in more complex SPI devices, such as a flash device
      // For a simpler slave device like the DW1000, everything is handled in
      // the TX buffer
      // So, we can ignore these two phases
      .command_bits = 0,
      .address_bits = 0,

      .dummy_bits = 0,
      .mode = 0,
      .duty_cycle_pos = 0,
      .cs_ena_pretrans = 0,
      .cs_ena_posttrans = 0,
      // DW1000 needs SPI <= 3 MHz during init (esp. while the LDE load forces
      // the 19.2 MHz XTI clock). 2 MHz is safe for the whole session; can raise
      // to 20 MHz post-init later. TODO: slow-init then switch to fast.
      .clock_speed_hz = 2 * 1000 * 1000,
      .input_delay_ns = 0,
      .spics_io_num = cs_,
      .flags = 0,
      .queue_size = 4, // TODO: queue size
      .pre_cb = NULL,
      .post_cb = NULL};

  ESP_LOGI(TAG, "SPI add device: %d",
           spi_bus_add_device(HOST, &dev_config, &dev_handle_));
}

SPI::~SPI() {
  if (dev_handle_) {
    ESP_LOGI(TAG, "SPI remove device: %d", spi_bus_remove_device(dev_handle_));
    dev_handle_ = nullptr;
  }
}

SPI::SPI(SPI &&other)
    : cs_{std::exchange(other.cs_, -1)},
      dev_handle_{std::exchange(other.dev_handle_, nullptr)} {}

SPI &SPI::operator=(SPI &&other) {
  if (this != &other) {
    SPI temp{std::move(other)};
    swap(temp);
  }

  return *this;
}

std::expected<void, HAL::SpiError>
SPI::transfer_halfduplex(std::span<const std::byte> tx,
                         std::span<std::byte> rx) {
  // Use full-duplex since it allows large transfers via DMA channels (unlike
  // half) To simulate half-duplex transfers, we first do a tx transfer, then an
  // rx. This causes 2 transactions rather than 1, but makes large (e.g., 1024
  // bytes) reads/writes possible.
  size_t total = tx.size_bytes() + rx.size_bytes();

  std::vector<std::byte> tx_buf(total, std::byte{0});
  std::vector<std::byte> rx_buf(total, std::byte{0});

  // copy header into tx_buf, rest is zeros (dummy bytes)
  std::ranges::copy(tx, tx_buf.begin());

  spi_transaction_t transaction = {.length = BYTES_TO_BITS(total),
                                   .tx_buffer = tx_buf.data(),
                                   .rx_buffer = rx_buf.data()};

  esp_err_t res = spi_device_transmit(dev_handle_, &transaction);
  if (unlikely(res != ESP_OK)) {
    ESP_LOGE(TAG, "SPI transfer failed: %d", res);
    return std::unexpected(from_esp_err(res));
  }

  // response starts after the header bytes
  std::ranges::copy(rx_buf | std::views::drop(tx.size_bytes()), rx.begin());

  return {};
}

void SPI::swap(SPI &other) {
  std::swap(cs_, other.cs_);
  std::swap(dev_handle_, other.dev_handle_);
}

} // namespace ESP32
