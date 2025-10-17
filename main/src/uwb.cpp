#include "uwb.h"

#include "error.h"
#include "driver/spi_master.h"
#include "hal/spi_types.h"

#define DWM_REG_DEV_ID 0x00
#define DWM_REG_TRANSMIT_DATA_BUFFER 0x09

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23

#define DW_CS 4
#define PIN_RST 27
#define PIN_IRQ 34

void uwb_init() {
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
        8,
        64,
        0,
        0,
        0,
        0,
        0,
        APB_CLK_FREQ / 80,
        0,
        DW_CS,
        0,
        4, // TODO: queue size
        NULL,
        NULL
    };

    spi_device_handle_t dev_handle;

    log("SPI init: %d", spi_bus_initialize(SPI2_HOST, &config, SPI_DMA_CH_AUTO));
    log("SPI add device: %d", spi_bus_add_device(SPI2_HOST, &dev_config, &dev_handle));

    uint8_t tx_buf[5] = { 0x00, 0, 0, 0, 0 };  // header + dummy bytes
    uint8_t rx_buf[5] = { 0 };

    spi_transaction_t t = {
        .length = 8 * 5,              // bits to send
        .rxlength = 8 * 5,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };

    log("SPI transaction: %d", spi_device_transmit(dev_handle, &t));
    
    uint32_t id = 0;
    for (int i = 1; i <= 4; ++i) {
        id |= rx_buf[i] << (8 * (i - 1));
    }
    log("ID received: %X", id);
}