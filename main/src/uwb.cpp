#include "uwb.h"

#include "error.h"
#include <cstring>

#define DWM_REG_DEV_ID 0x00
#define DWM_REG_TRANSMIT_FRAME_CONTROL 0x08
#define DWM_REG_TRANSMIT_DATA_BUFFER 0x09
#define DWM_REG_SYSTEM_CONTROL 0x0D
#define DWM_REG_SYSTEM_EVENT_STATUS 0x0F
#define DWM_REG_RECEIVE_DATA_BUFFER 0x11

#define DWM_SYS_CTRL_TXSTRT (1 << 1)
#define DWM_SYS_CTRL_RXENAB (1 << 8)

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

    char* temp = "hi";
    ESP_ERROR_CHECK(uwb_transmit((uint8_t*)temp, 3, dev_handle));
    log("yay! transmitted message");

    /*char buf[3];
    ESP_ERROR_CHECK(uwb_receive((uint8_t*)buf, 3, dev_handle));
    log("received: %s", buf);*/
}

// TODO: handle sub-register reads
esp_err_t uwb_read_reg(uint8_t reg, uint8_t* rx, size_t len, spi_device_handle_t dev_handle) {
    // Lower 6 bits store actual register
    // MSbit = 0 represents read
    reg = 0x00 | (reg & 0x3F);

    spi_transaction_t transaction = {
        .length = BYTES_TO_BITS * 1,
        .rxlength = BYTES_TO_BITS * len,
        .tx_buffer = &reg,
        .rx_buffer = rx,
    };

    return spi_device_transmit(dev_handle, &transaction);
}

// TODO: handle sub-register writes
esp_err_t uwb_write_reg(uint8_t reg, uint8_t* tx, size_t len, spi_device_handle_t dev_handle) {
    // Lower 6 bits store actual register
    // MSbit = 1 represents write
    reg = 0x80 | (reg & 0x3F);

    uint8_t buf[1 + len];
    buf[0] = reg;
    memcpy(buf + 1, tx, len);

    spi_transaction_t transaction = {
        .length = BYTES_TO_BITS * (1 + len),
        .rxlength = 0,
        .tx_buffer = buf,
        .rx_buffer = NULL,
    };

    return spi_device_transmit(dev_handle, &transaction);
}

esp_err_t uwb_transmit(uint8_t* tx, size_t len, spi_device_handle_t dev_handle) {
    uint32_t raw = 0;
    uint8_t ifsdelay = 0;

    SET_FIELD<uint32_t>(raw, 0, 7, len + 2); // add 2 for CRC at end
    SET_FIELD<uint32_t>(raw, 7, 3, 0);
    SET_FIELD<uint32_t>(raw, 10, 3, 0);
    SET_FIELD<uint32_t>(raw, 13, 2, 2); // 6.8 Mbps
    SET_FIELD<uint32_t>(raw, 15, 1, 0);
    SET_FIELD<uint32_t>(raw, 16, 2, 1); // 16 MHz
    SET_FIELD<uint32_t>(raw, 18, 2, 3); // 1024 symbols 
    SET_FIELD<uint32_t>(raw, 20, 2, 0);
    SET_FIELD<uint32_t>(raw, 22, 10, 0);
    SET_FIELD<uint8_t>(ifsdelay, 0, 8, 0);

    // Can't take a reference to a packed struct's field so need to assign the fields after setting them in local variables
    dwm_transmit_frame_control_t tx_fctrl {
        .raw = raw,
        .ifsdelay = ifsdelay
    };
    
    // Write configuration
    ESP_ERROR_CHECK(uwb_write_reg(DWM_REG_TRANSMIT_FRAME_CONTROL, (uint8_t*)&tx_fctrl, sizeof(tx_fctrl), dev_handle)); // TODO: make size 5 constant

    // Write payload data
    ESP_ERROR_CHECK(uwb_write_reg(DWM_REG_TRANSMIT_DATA_BUFFER, (uint8_t*)tx, len, dev_handle));

    // Write TXSTRT bit
    dwm_system_control_t sys_ctrl = DWM_SYS_CTRL_TXSTRT;
    ESP_ERROR_CHECK(uwb_write_reg(DWM_REG_SYSTEM_CONTROL, (uint8_t*)&sys_ctrl, sizeof(sys_ctrl), dev_handle));

    // Wait for reg 0x0F TXFRS bit
    uint8_t sys_status[5];
    do {
        ESP_ERROR_CHECK(uwb_read_reg(DWM_REG_SYSTEM_EVENT_STATUS, (uint8_t*)&sys_status, sizeof(sys_status), dev_handle));
        log("polled status bit");
    } while (((sys_status[0] >> 7) & 1) == 0);

    return ESP_OK;
}

esp_err_t uwb_receive(uint8_t* rx, size_t len, spi_device_handle_t dev_handle) {
    // TODO: right now, receiving only works with default tx settings
    // fix this to take in settings or something?
    
    // Write RXENAB bit
    dwm_system_control_t sys_ctrl = DWM_SYS_CTRL_RXENAB;
    ESP_ERROR_CHECK(uwb_write_reg(DWM_REG_SYSTEM_CONTROL, (uint8_t*)&sys_ctrl, sizeof(sys_ctrl), dev_handle));

    // Wait for reg 0x0F RXDFR bit
    uint8_t sys_status[5];
    do {
        ESP_ERROR_CHECK(uwb_read_reg(DWM_REG_SYSTEM_EVENT_STATUS, (uint8_t*)sys_status, sizeof(sys_status), dev_handle));
        printf("SYS_STATUS: %02X %02X %02X %02X %02X\n",
        sys_status[4], sys_status[3], sys_status[2], sys_status[1], sys_status[0]);
    } while (((sys_status[1] >> 5) & 1) == 0);

    ESP_ERROR_CHECK(uwb_read_reg(DWM_REG_RECEIVE_DATA_BUFFER, (uint8_t*)rx, len, dev_handle));

    return ESP_OK;
}