#include "esp32.h"

#include "error.h"

namespace ESP32 {
    constexpr int SPI_SCK = 18;
    constexpr int SPI_MISO = 19;
    constexpr int SPI_MOSI = 23;

    SPI::SPI(int cs) : cs_{cs} {
        spi_bus_config_t config {
            SPI_MOSI,
            SPI_MISO,
            SPI_SCK,
            -1,
            -1,
            -1,
            -1,
            -1,
            -1,
            SOC_SPI_MAXIMUM_BUFFER_SIZE,
            SPICOMMON_BUSFLAG_MASTER, // TODO
            0  // TODO
        };

        spi_device_interface_config_t dev_config {
            // Command and address bits are for specific command and address phases of SPI
            // These exist in more complex SPI devices, such as a flash device
            // For a simpler slave device like the DW1000, everything is handled in the TX buffer
            // So, we can ignore these two phases
            .command_bits = 0,
            .address_bits = 0,

            .dummy_bits = 0,
            .mode = 0,
            .duty_cycle_pos = 0,
            .cs_ena_pretrans = 5,
            .cs_ena_posttrans = 5,
            .clock_speed_hz = APB_CLK_FREQ / 80, // 1 MHz
            .input_delay_ns = 50,
            .spics_io_num = cs_,
            .flags = 0,
            .queue_size = 4, // TODO: queue size
            .pre_cb = NULL,
            .post_cb = NULL
        };

        // TODO: throw?
        // TODO: dynamically choose host/port?
        log("SPI init: %d", spi_bus_initialize(SPI2_HOST, &config, SPI_DMA_DISABLED));
        log("SPI add device: %d", spi_bus_add_device(SPI2_HOST, &dev_config, &dev_handle_)); 
    }

    SPI::~SPI() {
        // TODO: throw?
        log("SPI deinit: %d", spi_bus_remove_device(dev_handle_));
    }

    esp_err_t SPI::read(std::span<std::byte> rx) {
        return ESP_OK;
    }

    esp_err_t SPI::write(std::span<const std::byte> tx) {
        return ESP_OK;
    }
}