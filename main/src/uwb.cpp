#include "uwb.h"

#include "error.h"
#include "freertos/task.h"
#include <cstring>

#define DWM_REG_DEV_ID                  0x00
#define DWM_REG_SYSTEM_TIME_COUNTER     0x06
#define DWM_REG_TRANSMIT_FRAME_CONTROL  0x08
#define DWM_REG_TRANSMIT_DATA_BUFFER    0x09
#define DWM_REG_DELAYED_SEND_RECEIVE    0x0A
#define DWM_REG_SYSTEM_CONTROL          0x0D
#define DWM_REG_SYSTEM_EVENT_STATUS     0x0F
#define DWM_REG_RECEIVE_DATA_BUFFER     0x11
#define DWM_REG_RX_TIME                 0x15
#define DWM_REG_TX_TIME                 0x17
#define DWM_REG_CHANNEL_CONTROL         0x1F

#define DWM_SIZE_BYTES_DELAYED_SEND_RECEIVE 5

#define DWM_SYS_CTRL_TXSTRT (1 << 1)
#define DWM_SYS_CTRL_TXDLYS (1 << 2)
#define DWM_SYS_CTRL_RXENAB (1 << 8)

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23

#define DW_CS 4
#define PIN_RST 27
#define PIN_IRQ 34
#define BYTES_TO_BITS 8

#define DWM_PS_TO_DEVICE_TIME (1.0 / 15.65)
#define MS_TO_PS (1000000000ULL)
#define DWM_MS_TO_DEVICE_TIME_SCALE (63897600ULL)

#define PING_MSG "ping"
#define PING_MSG_LEN 4
#define DELAY_CONST 500  // ms

#define DWM_SYS_STATUS_TXFRB (1 << 4)
#define DWM_SYS_STATUS_TXPRS (1 << 5)
#define DWM_SYS_STATUS_TXPHS (1 << 6)
#define DWM_SYS_STATUS_TXFRS (1 << 7)

static esp_err_t get_time(uint64_t* tv, spi_device_handle_t dev_handle, uint8_t reg = DWM_REG_SYSTEM_TIME_COUNTER) {
    // Read the current system time
    uint8_t sys_time[5];
    ESP_ERROR_CHECK(uwb_read_reg(reg, sys_time, sizeof(sys_time), dev_handle));

    // Load the current time into a single variable
    uint64_t current_time = 0;
    for (int i = 0; i < sizeof(sys_time); ++i) {
        current_time |= ((uint64_t)sys_time[i] << (8 * i));
    }

    // Store the current time in the output parameter
    current_time &= 0x000000FFFFFFFFFFUL;
    *tv = current_time;
    
    return ESP_OK;
}

static void NodeA(spi_device_handle_t dev_handle) {
    // TODO: make this not delayed transmit
    ESP_ERROR_CHECK(uwb_delayed_transmit((uint8_t*)PING_MSG, PING_MSG_LEN, 300, dev_handle));
    log("Transmitted ping message");

    uint64_t tx_time;
    ESP_ERROR_CHECK(get_time(&tx_time, dev_handle, DWM_REG_TX_TIME));
    
    uint8_t rx[PING_MSG_LEN];
    ESP_ERROR_CHECK(uwb_receive((uint8_t*)rx, PING_MSG_LEN, dev_handle));
    log("Received ping message");

    // Wait for LDEDONE bit
    uint8_t sys_status[5];
    do {
        ESP_ERROR_CHECK(uwb_read_reg(DWM_REG_SYSTEM_EVENT_STATUS, (uint8_t*)&sys_status, sizeof(sys_status), dev_handle));
        log("polled ldedone bit: %X %X %X %X %X", sys_status[4], sys_status[3], sys_status[2], sys_status[1], sys_status[0]);
    } while (((sys_status[1] >> 2) & 1) == 0);
    
    uint64_t rx_time;
    ESP_ERROR_CHECK(get_time(&rx_time, dev_handle, DWM_REG_RX_TIME));
    log("Delta time is %llu - %llu = %llu", rx_time, tx_time, rx_time - tx_time);
    
    double tof = (((rx_time - tx_time) / (63.8976e9 / 1000.0)) - DELAY_CONST) / 2.0;
    log("DISTANCE: %f feet", tof * 983571.056); // ms * (ft/ms)
}

static void NodeB(spi_device_handle_t dev_handle) {
    
    uint8_t rx[PING_MSG_LEN];
    ESP_ERROR_CHECK(uwb_receive((uint8_t*)rx, PING_MSG_LEN, dev_handle));

    char printbuf[PING_MSG_LEN + 1];
    memcpy(printbuf, rx, PING_MSG_LEN);
    printbuf[PING_MSG_LEN] = '\0';
    log("Received: %s", printbuf);
    ESP_ERROR_CHECK(uwb_delayed_transmit((uint8_t*)PING_MSG, PING_MSG_LEN, DELAY_CONST, dev_handle));
}

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
        .cs_ena_pretrans = 2,
        .cs_ena_posttrans = 2,
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

    // Channel control register
    // Example: Channel 5, PRF 64MHz, preamble code 9 (commonly used)
    /*uint32_t chan_ctrl = 0;
    SET_FIELD<uint32_t>(chan_ctrl, 0, 4, 5);  // Channel 5 (TX)
    SET_FIELD<uint32_t>(chan_ctrl, 4, 4, 5);  // Channel 5 (RX)
    SET_FIELD<uint32_t>(chan_ctrl, 18, 2, 2); // PRF 64 MHz
    SET_FIELD<uint32_t>(chan_ctrl, 22, 5, 9); // TX preamble code
    SET_FIELD<uint32_t>(chan_ctrl, 27, 5, 9); // RX preamble code
    uwb_write_reg(DWM_REG_CHANNEL_CONTROL, (uint8_t*)&chan_ctrl, sizeof(chan_ctrl), dev_handle);*/

    uint8_t sys_mask[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uwb_write_reg(DWM_REG_SYSTEM_EVENT_STATUS, sys_mask, 5, dev_handle);

    // NodeA(dev_handle);
    NodeB(dev_handle);
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
    // Clear TXFRS bit (and other relevant flags)
    uint64_t sys_status_mask = DWM_SYS_STATUS_TXFRB | DWM_SYS_STATUS_TXPRS | DWM_SYS_STATUS_TXPHS | DWM_SYS_STATUS_TXFRS;
    ESP_ERROR_CHECK(uwb_write_reg(DWM_REG_SYSTEM_EVENT_STATUS, (uint8_t*)&sys_status_mask, 5, dev_handle));
    
    uint32_t raw = 0;
    uint8_t ifsdelay = 0;

    // TODO: make constants because this is awful
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
    ESP_ERROR_CHECK(uwb_write_reg(DWM_REG_TRANSMIT_FRAME_CONTROL, (uint8_t*)&tx_fctrl, sizeof(tx_fctrl), dev_handle));

    // Write payload data
    ESP_ERROR_CHECK(uwb_write_reg(DWM_REG_TRANSMIT_DATA_BUFFER, (uint8_t*)tx, len, dev_handle));

    // Write TXSTRT bit
    dwm_system_control_t sys_ctrl = DWM_SYS_CTRL_TXSTRT;
    ESP_ERROR_CHECK(uwb_write_reg(DWM_REG_SYSTEM_CONTROL, (uint8_t*)&sys_ctrl, sizeof(sys_ctrl), dev_handle));

    // Wait for reg 0x0F TXFRS bit
    uint8_t sys_status[5];
    do {
        ESP_ERROR_CHECK(uwb_read_reg(DWM_REG_SYSTEM_EVENT_STATUS, (uint8_t*)&sys_status, sizeof(sys_status), dev_handle));
        log("polled status bit: %d", ((sys_status[0] >> 7) & 1));
    } while (((sys_status[0] >> 7) & 1) == 0);

    log("Transmit sent!");

    return ESP_OK;
}

esp_err_t uwb_delayed_transmit(uint8_t* tx, size_t len, uint64_t delay_ms, spi_device_handle_t dev_handle) {
    // Clear TXFRS bit (and other relevant flags)
    uint64_t sys_status_mask = DWM_SYS_STATUS_TXFRB | DWM_SYS_STATUS_TXPRS | DWM_SYS_STATUS_TXPHS | DWM_SYS_STATUS_TXFRS;
    ESP_ERROR_CHECK(uwb_write_reg(DWM_REG_SYSTEM_EVENT_STATUS, (uint8_t*)&sys_status_mask, 5, dev_handle));
    vTaskDelay(pdMS_TO_TICKS(5));

    // Read the current system time
    uint64_t current_time;
    ESP_ERROR_CHECK(get_time(&current_time, dev_handle));

    // Load the delay based on the current time
    uint64_t delay_dtu = (uint64_t)(delay_ms * DWM_MS_TO_DEVICE_TIME_SCALE);

    // Calculate delayed time first, then mask only the result
    // Compute delayed time
    uint64_t delayed_time = current_time + delay_dtu;

    // Round delayed time up for precision
    // Additionally, mask lower 9 bits since they are ignored
    delayed_time = (delayed_time + 0x1FF) & ~0x1FFULL;

    log("Current time: %llu (0x%llX)", current_time, current_time);
    log("Delay DTU: %llu (0x%llX)", delay_dtu, delay_dtu);
    log("Delayed time: %llu (0x%llX)", delayed_time, delayed_time);

    // Store delayed time in register
    ESP_ERROR_CHECK(uwb_write_reg(DWM_REG_DELAYED_SEND_RECEIVE, (uint8_t*)&delayed_time, DWM_SIZE_BYTES_DELAYED_SEND_RECEIVE, dev_handle));

    // Read it back to verify
    uint8_t readback[5];
    ESP_ERROR_CHECK(uwb_read_reg(DWM_REG_DELAYED_SEND_RECEIVE, readback, DWM_SIZE_BYTES_DELAYED_SEND_RECEIVE, dev_handle));
    uint64_t readback_time = 0;
    for (int i = 0; i < 5; ++i)
    {
        readback_time |= ((uint64_t)readback[i] << (8 * i));
    }
    log("Delayed time written:  %llu (0x%llX)", delayed_time, delayed_time);
    log("Delayed time readback: %llu (0x%llX)", readback_time, readback_time);

    if (readback_time != delayed_time)
    {
        log("ERROR: Readback doesn't match! Register write failed or wrong register!");
    }

    //
    // TODO: leverage the regular uwb_transmit function instead of copying code
    //

    uint32_t raw = 0;
    uint8_t ifsdelay = 0;

    // TODO: make constants because this is awful
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
    ESP_ERROR_CHECK(uwb_write_reg(DWM_REG_TRANSMIT_FRAME_CONTROL, (uint8_t*)&tx_fctrl, sizeof(tx_fctrl), dev_handle));

    // Write payload data
    ESP_ERROR_CHECK(uwb_write_reg(DWM_REG_TRANSMIT_DATA_BUFFER, (uint8_t*)tx, len, dev_handle));

    // Write system control bits
    dwm_system_control_t sys_ctrl = (DWM_SYS_CTRL_TXDLYS | DWM_SYS_CTRL_TXSTRT);
    ESP_ERROR_CHECK(uwb_write_reg(DWM_REG_SYSTEM_CONTROL, (uint8_t*)&sys_ctrl, sizeof(sys_ctrl), dev_handle));
    
    log("Initiated transmit with sys_ctrl=%d, waiting...", sys_ctrl);
    
    // Wait for reg 0x0F TXFRS bit
    uint8_t sys_status[5];
    do {
        ESP_ERROR_CHECK(uwb_read_reg(DWM_REG_SYSTEM_EVENT_STATUS, (uint8_t*)&sys_status, sizeof(sys_status), dev_handle));
        // log("SYS_STATUS BIT: %d", (sys_status[0] >> 7) & 1);
    } while (((sys_status[0] >> 7) & 1) == 0);

    log("Transmit sent!");
    
    // Check HPDWARN bit (bit 10, which is bit 2 of byte 1)
    if ((sys_status[1] >> 2) & 1)
    {
        log("ERROR: HPDWARN bit set!");
    }
    else
    {
        log("HPDWARN was NOT set");
    }

    // Read current time again to see how much time has actually passed
    uint64_t current_time_after;
    ESP_ERROR_CHECK(get_time(&current_time_after, dev_handle, DWM_REG_TX_TIME));

    uint64_t elapsed_dtu;
    if (current_time_after >= current_time) {
        elapsed_dtu = current_time_after - current_time;
    } else {
        // Wraparound happened
        elapsed_dtu = (current_time_after + (1ULL << 40)) - current_time;
    }

    log("Expected TX time:     %llu (0x%llX)", delayed_time, delayed_time);
    log("Actual TX time:       %llu (0x%llX)", current_time_after, current_time_after);
    log("Elapsed time in loop: %llu DTU (%.3f ms)",
        elapsed_dtu,
        elapsed_dtu / (63.8976e9 / 1000.0));

    return ESP_OK;
}

esp_err_t uwb_receive(uint8_t* rx, size_t len, spi_device_handle_t dev_handle) {
    // TODO: right now, receiving only works with default tx settings
    // fix this to take in settings or something?

    // Clear RXDFR bit (and other relevant flags)
    /*
        setBit(_sysstatus, LEN_SYS_STATUS, RXDFR_BIT, true);
        setBit(_sysstatus, LEN_SYS_STATUS, LDEDONE_BIT, true);
        setBit(_sysstatus, LEN_SYS_STATUS, LDEERR_BIT, true);
        setBit(_sysstatus, LEN_SYS_STATUS, RXPHE_BIT, true);
        setBit(_sysstatus, LEN_SYS_STATUS, RXFCE_BIT, true);
        setBit(_sysstatus, LEN_SYS_STATUS, RXFCG_BIT, true);
        setBit(_sysstatus, LEN_SYS_STATUS, RXRFSL_BIT, true);
    */
    uint64_t sys_status_mask = (1 << 13) | (1 << 10) | (1 << 18) | (1 << 12) | (1 << 15) | (1 << 14) | (1 << 16);
    ESP_ERROR_CHECK(uwb_write_reg(DWM_REG_SYSTEM_EVENT_STATUS, (uint8_t*)&sys_status_mask, 5, dev_handle));

    // Write RXENAB bit
    dwm_system_control_t sys_ctrl = DWM_SYS_CTRL_RXENAB;
    ESP_ERROR_CHECK(uwb_write_reg(DWM_REG_SYSTEM_CONTROL, (uint8_t*)&sys_ctrl, sizeof(sys_ctrl), dev_handle));

    // Wait for reg 0x0F RXDFR bit
    uint8_t sys_status[5];
    do {
        ESP_ERROR_CHECK(uwb_read_reg(DWM_REG_SYSTEM_EVENT_STATUS, (uint8_t*)sys_status, sizeof(sys_status), dev_handle));
        // printf("SYS_STATUS: %02X %02X %02X %02X %02X\n",
        // sys_status[4], sys_status[3], sys_status[2], sys_status[1], sys_status[0]);
        // printf("ERROR FLAG BITS: RXFTO %d, RXPTO: %d, RXPHE %d, RXFCE: %d",((sys_status[1] >> 5) & 1),((sys_status[1] >> 5) & 1),((sys_status[1] >> 5) & 1),((sys_status[1] >> 5) & 1));
    } while (((sys_status[1] >> 5) & 1) == 0);

    ESP_ERROR_CHECK(uwb_read_reg(DWM_REG_RECEIVE_DATA_BUFFER, (uint8_t*)rx, len, dev_handle));

    return ESP_OK;
}
