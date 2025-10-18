#include "uwb.h"

#include "error.h"
#include <cstring>

#define DWM_REG_DEV_ID 0x00
#define DWM_REG_TRANSMIT_DATA_BUFFER 0x09

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23

#define DW_CS 4
#define PIN_RST 27
#define PIN_IRQ 34
#define BYTES_TO_BITS 8

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
        // Command and address bits are for specific command and address phases of SPI
        // These exist in more complex SPI devices, such as a flash device
        // For a simpler slave device like the DW1000, everything is handled in the TX buffer
        // So, we can ignore these two phases
        .command_bits = 0,
        .address_bits = 0,
        
        .dummy_bits = 0,
        .mode = 0,
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = APB_CLK_FREQ / 80,
        .input_delay_ns = 0,
        .spics_io_num = DW_CS,
        .flags = SPI_DEVICE_HALFDUPLEX,
        .queue_size = 4, // TODO: queue size
        .pre_cb = NULL,
        .post_cb = NULL
    };

    spi_device_handle_t dev_handle;

    log("SPI init: %d", spi_bus_initialize(SPI2_HOST, &config, SPI_DMA_DISABLED));
    log("SPI add device: %d", spi_bus_add_device(SPI2_HOST, &dev_config, &dev_handle));

    uint8_t rx_buf[4];
    log("SPI transaction: %d", uwb_read_reg(DWM_REG_DEV_ID, rx_buf, 4, dev_handle));
    
    uint32_t id = 0;
    for (int i = 0; i < 4; ++i) {
        id |= rx_buf[i] << (8 * i);
    }
    log("ID received: %X", id);
}

esp_err_t uwb_read_reg(uint8_t reg, uint8_t* rx, size_t len, spi_device_handle_t dev_handle) {
    // Reg is maximum 6 bits
    reg &= 0x3F;

    spi_transaction_t transaction = {
        .length = BYTES_TO_BITS * 1,
        .rxlength = BYTES_TO_BITS * len,
        .tx_buffer = &reg,
        .rx_buffer = rx,
    };

    return spi_device_transmit(dev_handle, &transaction);
}

esp_err_t uwb_transmit(uint8_t* tx, size_t len, spi_device_handle_t dev_handle) {
    const char* payload = "hello";

    dwm_transmit_frame_control_t tx_fctrl;
    
    SET_FIELD(tx_fctrl.raw, 0, 7, strlen(payload) + 2); // add 2 for CRC at end
    SET_FIELD(tx_fctrl.raw, 7, 3, 0);
    SET_FIELD(tx_fctrl.raw, 10, 3, 0);
    SET_FIELD(tx_fctrl.raw, 13, 2, 0); // 110 kbps
    SET_FIELD(tx_fctrl.raw, 15, 1, 0);
    SET_FIELD(tx_fctrl.raw, 16, 2, 0b10); // 64 MHz
    SET_FIELD(tx_fctrl.raw, 18, 2, 0b01); // 64 symbol preamble
    SET_FIELD(tx_fctrl.raw, 20, 2, 0);
    SET_FIELD(tx_fctrl.raw, 22, 10, 0);
}