#include "uwb.h"

#include "error.h"
#include "freertos/task.h"
#include <cstring>
#include <driver/gpio.h>

#define DWM_REG_DEV_ID                  0x00
#define DWM_REG_SYS_CFG                 0x04
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
#define DWM_REG_POWER_MANAGEMENT 0x36
#define DWM_REG_OTP_MEMORY 0x2D

#define DWM_SUB_REG_OTP_CTRL 0x06

#define DWM_SIZE_BYTES_DELAYED_SEND_RECEIVE 5

#define DWM_SYS_CTRL_TXSTRT (1 << 1)
#define DWM_SYS_CTRL_TXDLYS (1 << 2)
#define DWM_SYS_CTRL_RXENAB (1 << 8)

#define DWM_SYS_CFG_PHR_MODE_START 16
#define DWM_SYS_CFG_PHR_MODE_LEN 2
#define DWM_PHR_MODE_STANDARD_FRAME 0b00
#define DWM_PHR_MODE_EXTENDED_FRAME 0b11

#define DWM_SYS_CFG_DIS_STXP_START 18
#define DWM_SYS_CFG_DIS_STXP_LEN 1
#define DWM_SYS_CFG_DIS_STXP_DEFAULT 1 // disable smart power (active low)

#define DWM_SYS_CFG_FFEN_START 0
#define DWM_SYS_CFG_FFEN_LEN 1
#define DWM_SYS_CFG_FFEN_DEFAULT 0

#define DWM_CHAN_CTRL_TX_CHAN_START 0
#define DWM_CHAN_CTRL_TX_CHAN_LEN 4
#define DWM_CHAN_CTRL_RX_CHAN_START 4
#define DWM_CHAN_CTRL_RX_CHAN_LEN 4
#define DWM_CHAN_CTR_DEFAULT_CHANNEL 5

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23

#define DW_CS 4
#define PIN_RST (gpio_num_t)27
#define PIN_IRQ (gpio_num_t)34
#define BYTES_TO_BITS 8

#define DWM_PS_TO_DEVICE_TIME (1.0 / 15.65)
#define MS_TO_PS (1000000000ULL)
#define DWM_MS_TO_DEVICE_TIME_SCALE (63897600ULL)

#define PING_MSG "ping"
#define PING_MSG_LEN 4
#define DELAY_CONST_MS 30  // ms

#define DWM_SYS_STATUS_TXFRB (1 << 4)
#define DWM_SYS_STATUS_TXPRS (1 << 5)
#define DWM_SYS_STATUS_TXPHS (1 << 6)
#define DWM_SYS_STATUS_TXFRS (1 << 7)

static void uwb_hard_reset() {
    gpio_set_direction(PIN_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));  // hold low for >10ms
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));  // wait for startup
}

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
//TODO if still not working add subregister read and check LDERUNE when receiving
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
    /*uint8_t sys_status[5];
    do {
        ESP_ERROR_CHECK(uwb_read_reg(DWM_REG_SYSTEM_EVENT_STATUS, (uint8_t*)&sys_status, sizeof(sys_status), dev_handle));
        log("polled ldedone bit: %X %X %X %X %X", sys_status[4], sys_status[3], sys_status[2], sys_status[1], sys_status[0]);
    } while (((sys_status[1] >> 2) & 1) == 0);*/
    
    uint64_t rx_time;
    ESP_ERROR_CHECK(get_time(&rx_time, dev_handle, DWM_REG_RX_TIME));

    uint64_t delta_time_dtu = rx_time - tx_time; // DTU
    double delta_time_us = delta_time_dtu * 1.565e-5; // microseconds
    double tof_us = (delta_time_us - (DELAY_CONST_MS * 1000.0)) / 2.0; // tof one-way in microseconds
    double distance_ft = tof_us * 983.57105643045;
    
    log("Delta time: %f us", delta_time_us);
    log("TOF: %f us", tof_us);
    log("DISTANCE: %f feet", distance_ft); // ms * (ft/ms)
}

static void NodeB(spi_device_handle_t dev_handle) {
    
    uint8_t rx[PING_MSG_LEN];
    ESP_ERROR_CHECK(uwb_receive((uint8_t*)rx, PING_MSG_LEN, dev_handle));

    char printbuf[PING_MSG_LEN + 1];
    memcpy(printbuf, rx, PING_MSG_LEN);
    printbuf[PING_MSG_LEN] = '\0';
    log("Received: %s", printbuf);
    ESP_ERROR_CHECK(uwb_delayed_transmit((uint8_t*)PING_MSG, PING_MSG_LEN, DELAY_CONST_MS, dev_handle));
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
        .cs_ena_pretrans = 5,
        .cs_ena_posttrans = 5,
        .clock_speed_hz = APB_CLK_FREQ / 80,
        .input_delay_ns = 10,
        .spics_io_num = DW_CS,
        .flags = SPI_DEVICE_HALFDUPLEX,
        .queue_size = 4, // TODO: queue size
        .pre_cb = NULL,
        .post_cb = NULL
    };

    spi_device_handle_t dev_handle;

    log("SPI init: %d", spi_bus_initialize(SPI2_HOST, &config, SPI_DMA_DISABLED));
    log("SPI add device: %d", spi_bus_add_device(SPI2_HOST, &dev_config, &dev_handle));

    // Hard reset the device
    uwb_hard_reset();
    vTaskDelay(pdMS_TO_TICKS(1000));

    uint8_t rx_buf[4];
    log("SPI transaction: %d", uwb_read_reg(DWM_REG_DEV_ID, rx_buf, 4, dev_handle));
    
    uint32_t id = 0;
    for (int i = 0; i < 4; ++i) {
        id |= rx_buf[i] << (8 * i);
    }
    log("ID received: %X", id);

    // LDE algorithm loaded to allow rx timestamping

    // Step 1
    uint8_t buf[2] = {0x01, 0x03};
    uwb_write_reg(DWM_REG_POWER_MANAGEMENT, (uint8_t*)buf, sizeof(buf), dev_handle);

    // Step 2
    buf[1] = 0x80;
    buf[0] = 0x00;
    uwb_write_subreg(DWM_REG_OTP_MEMORY, DWM_SUB_REG_OTP_CTRL, (uint8_t*)buf, sizeof(buf), dev_handle);
    vTaskDelay(pdMS_TO_TICKS(1));

    // Step 3
    buf[1] = 0x02;
    buf[0] = 0x00;
    uwb_write_reg(DWM_REG_POWER_MANAGEMENT, (uint8_t*)buf, sizeof(buf), dev_handle);

    uint8_t sys_status[5];

    ESP_ERROR_CHECK(uwb_read_reg(DWM_REG_SYSTEM_EVENT_STATUS, (uint8_t*)&sys_status, sizeof(sys_status), dev_handle));
    log("polled sys status: %X %X %X %X %X", sys_status[4], sys_status[3], sys_status[2], sys_status[1], sys_status[0]);

    uint8_t sys_mask[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uwb_write_reg(DWM_REG_SYSTEM_EVENT_STATUS, sys_mask, 5, dev_handle);

    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_ERROR_CHECK(uwb_read_reg(DWM_REG_SYSTEM_EVENT_STATUS, (uint8_t*)&sys_status, sizeof(sys_status), dev_handle));
    log("polled sys status: %X %X %X %X %X", sys_status[4], sys_status[3], sys_status[2], sys_status[1], sys_status[0]);

    NodeA(dev_handle);
    // NodeB(dev_handle);

    /*char* msg = "hello world";
    ESP_ERROR_CHECK(uwb_transmit((uint8_t*)msg, 12, dev_handle));
    log("yay transmit");*/

    // char recv[12];
    // log("try recv");
    // ESP_ERROR_CHECK(uwb_receive((uint8_t*)recv, 12, dev_handle));
    // log("yay receive: %s", recv);
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

// TODO: needs to take 16-bit subreg
esp_err_t uwb_write_subreg(uint8_t reg, uint8_t subreg, uint8_t* tx, size_t len, spi_device_handle_t dev_handle) {
    // Lower 6 bits store actual register
    // MSbit = 1 represents write
    // Second MSbit = 1 represent sub-index is present
    reg = 0b11000000 | (reg & 0x3F);

    // Extended address disabled (bit 7)
    // TODO: conditionally enable this when needed
    subreg = (subreg & 0x7F);

    uint8_t buf[2 + len];
    buf[0] = reg;
    buf[1] = subreg;
    memcpy(buf + 2, tx, len);

    spi_transaction_t transaction = {
        .length = BYTES_TO_BITS * (2 + len),
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

    // Write payload data
    ESP_ERROR_CHECK(uwb_write_reg(DWM_REG_TRANSMIT_DATA_BUFFER, (uint8_t*)tx, len, dev_handle));

    // Grab current transmit frame control register
    uint32_t transmit_frame_control;
    ESP_ERROR_CHECK(uwb_read_reg(DWM_REG_TRANSMIT_FRAME_CONTROL, (uint8_t*)&transmit_frame_control, sizeof(transmit_frame_control), dev_handle));

    // By default, frame check is enabled
    // Therefore, add 2 for the CRC
    // TODO: make this dynamic check
    SET_FIELD<uint32_t>(transmit_frame_control, 0, 8, len + 2);
    ESP_ERROR_CHECK(uwb_write_reg(DWM_REG_TRANSMIT_FRAME_CONTROL, (uint8_t*)&transmit_frame_control, sizeof(transmit_frame_control), dev_handle));

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

    // Clear TXFRS bit (and other relevant flags)
    uint64_t sys_status_mask = DWM_SYS_STATUS_TXFRB | DWM_SYS_STATUS_TXPRS | DWM_SYS_STATUS_TXPHS | DWM_SYS_STATUS_TXFRS;
    ESP_ERROR_CHECK(uwb_write_reg(DWM_REG_SYSTEM_EVENT_STATUS, (uint8_t*)&sys_status_mask, 5, dev_handle));

    // Write payload data
    ESP_ERROR_CHECK(uwb_write_reg(DWM_REG_TRANSMIT_DATA_BUFFER, (uint8_t*)tx, len, dev_handle));
    
    // Grab current transmit frame control register
    uint32_t transmit_frame_control;
    ESP_ERROR_CHECK(uwb_read_reg(DWM_REG_TRANSMIT_FRAME_CONTROL, (uint8_t*)&transmit_frame_control, sizeof(transmit_frame_control), dev_handle));

    // By default, frame check is enabled
    // Therefore, add 2 for the CRC
    // TODO: make this dynamic check
    SET_FIELD<uint32_t>(transmit_frame_control, 0, 8, len + 2);
    ESP_ERROR_CHECK(uwb_write_reg(DWM_REG_TRANSMIT_FRAME_CONTROL, (uint8_t*)&transmit_frame_control, sizeof(transmit_frame_control), dev_handle));

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

    // Clear system status bits
    uint64_t sys_status_mask = (1 << 13) | (1 << 10) | (1 << 18) | (1 << 12) | (1 << 15) | (1 << 14) | (1 << 16);
    ESP_ERROR_CHECK(uwb_write_reg(DWM_REG_SYSTEM_EVENT_STATUS, (uint8_t*)&sys_status_mask, 5, dev_handle));

    // Write RXENAB bit
    dwm_system_control_t sys_ctrl;
    ESP_ERROR_CHECK(uwb_read_reg(DWM_REG_SYSTEM_CONTROL, (uint8_t*)&sys_ctrl, sizeof(sys_ctrl), dev_handle));
    sys_ctrl |= DWM_SYS_CTRL_RXENAB;
    ESP_ERROR_CHECK(uwb_write_reg(DWM_REG_SYSTEM_CONTROL, (uint8_t*)&sys_ctrl, sizeof(sys_ctrl), dev_handle));

    // Wait for reg 0x0F RXDFR bit
    uint8_t sys_status[5];
    do {
        ESP_ERROR_CHECK(uwb_read_reg(DWM_REG_SYSTEM_EVENT_STATUS, (uint8_t*)sys_status, sizeof(sys_status), dev_handle));
        vTaskDelay(pdTICKS_TO_MS(1)); // prevent task starvation
    } while (((sys_status[1] >> 5) & 1) == 0);

    ESP_ERROR_CHECK(uwb_read_reg(DWM_REG_RECEIVE_DATA_BUFFER, (uint8_t*)rx, len, dev_handle));

    return ESP_OK;
}
