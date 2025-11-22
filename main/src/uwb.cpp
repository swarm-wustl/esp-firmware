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

#define DWM_MS_TO_DEVICE_TIME_SCALE (63897763.57827476) 

#define PING_MSG "ping"
#define PING_MSG_LEN 4
#define DELAY_CONST_MS 50  // ms

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

    // TODO: maybe make this a bit more of a dynamic check?
    if (reg == DWM_REG_SYSTEM_TIME_COUNTER) {
        ESP_ERROR_CHECK(uwb_read_reg(reg, sys_time, sizeof(sys_time), dev_handle));
    } else {
        // 32-bits from subreg 0x00, upper 8 bits from subreg 0x04
        ESP_ERROR_CHECK(uwb_read_subreg(reg, 0x00, sys_time, 4, dev_handle));
        ESP_ERROR_CHECK(uwb_read_subreg(reg, 0x04, &sys_time[4], 1, dev_handle));
    }

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
    // -------------------------------------------------------
    // 1. TRANSMIT THE POLL ("PING") MESSAGE
    // -------------------------------------------------------
    // Sends the initial ranging poll packet over UWB.
    ESP_ERROR_CHECK(uwb_transmit((uint8_t*)PING_MSG, PING_MSG_LEN, dev_handle));
    log("Transmitted ping message");
    // -------------------------------------------------------
    // 2. READ TRANSMIT TIMESTAMP (T_SP)
    // -------------------------------------------------------
    // After transmitting, read the hardware timestamp (40-bit)
    // that records exactly when the Poll left the antenna.
    uint64_t tx_time;
    ESP_ERROR_CHECK(get_time(&tx_time, dev_handle, DWM_REG_TX_TIME));
    uint64_t TSP = tx_time;
    // -------------------------------------------------------
    // 3. RECEIVE THE RESPONDER'S PROCESSING DELAY (t_delta)
    // -------------------------------------------------------
    // t_delta represents the responder's "turnaround time":
    // the time between receiving our Poll and sending back their reply.
    // uint8_t t_delta_buf[5];
    // ESP_ERROR_CHECK(uwb_receive((uint8_t*)t_delta_buf, 5, dev_handle));
    // log("Received t_delta message");
    uint8_t rx[PING_MSG_LEN];
    ESP_ERROR_CHECK(uwb_receive((uint8_t*)rx, PING_MSG_LEN, dev_handle));
    log("Received ping message");
    // -------------------------------------------------------
    // 4. CONVERT RESPONDER'S t_delta (5 bytes) INTO 64-BIT INT
    // -------------------------------------------------------
    // uint64_t t_delta = 0;
    // for (int i = 0; i < sizeof(t_delta_buf); ++i) {
    //     t_delta |= ((uint64_t)t_delta_buf[i] << (8 * i));
    // }

    // log("t_delta factor: %llu", t_delta);
  
    // -------------------------------------------------------
    // 5. WAIT FOR LDEDONE — CONFIRM THE RX STAMP IS VALID
    // -------------------------------------------------------
    // The DW1000 sets the LDE_DONE bit when the RX timestamp is ready.
    uint8_t sys_status[5];
    do {
        ESP_ERROR_CHECK(uwb_read_reg(DWM_REG_SYSTEM_EVENT_STATUS, (uint8_t*)&sys_status, sizeof(sys_status), dev_handle));
        log("polled ldedone bit: %X %X %X %X %X", sys_status[4], sys_status[3], sys_status[2], sys_status[1], sys_status[0]);
        // LDE_DONE is bit 18 → located in sys_status[1] bit 2
    } while (((sys_status[1] >> 2) & 1) == 0);
    
    // -------------------------------------------------------
    // 6. READ THE RECEIVE TIMESTAMP (T_RR)
    // -------------------------------------------------------
    // This timestamp marks the arrival time of the responder's message.
    uint64_t rx_time;
    ESP_ERROR_CHECK(get_time(&rx_time, dev_handle, DWM_REG_RX_TIME));
    uint64_t TRR = rx_time;

    
    

    // -------------------------------------------------------
    // 7. COMPUTE ROUND-TRIP DELTA IN DEVICE TIME UNITS (DTU)
    // -------------------------------------------------------
    // Account for possible timestamp wraparound (40-bit counter).
    // uint64_t delta_time_dtu;
    // if (rx_time >= tx_time) {
    //     delta_time_dtu = rx_time - tx_time - t_delta;
    // } else {
    //     // Counter wrapped around
        
    //     delta_time_dtu = ((1ULL << 40) - tx_time - t_delta) + rx_time;
    // }

    // -------------------------------------------------------
    // 8. CONVERT FROM DTU → MICROSECONDS
    // -------------------------------------------------------
    // 1 DTU = 15.65 ps → or 1.565e-5 microseconds
    //double delta_time_us = delta_time_dtu * 1.565e-5; // microseconds
    // -------------------------------------------------------
    // 9. COMPUTE ONE-WAY TIME OF FLIGHT
    // -------------------------------------------------------
    // Single-sided ranging:
    // TOF = (round_trip_time - t_delta) / 2
    //double tof_us = (delta_time_us) / 2.0; // tof one-way in microseconds
    // -------------------------------------------------------
    // 10. CONVERT TO DISTANCE (FEET)
    // -------------------------------------------------------
    // 1 microsecond = 983.571056 feet in vacuum
    //double distance_ft = tof_us * 983.57105643045;

    //transmit ping two
    
    ESP_ERROR_CHECK(uwb_delayed_transmit((uint8_t*)PING_MSG, PING_MSG_LEN, dev_handle));
    log("Transmitted ping message");


    ESP_ERROR_CHECK(get_time(&tx_time, dev_handle, DWM_REG_TX_TIME)); //
    uint64_t TSF = tx_time;


    //send delta time dtu and new tx time

    

    // -------------------------------------------------------
    // 11. DEBUG LOG OUTPUT
    // -------------------------------------------------------
    log("TX time: %llu DTU --- RX time: %llu DTU", tx_time, rx_time);
    log("Delta time: %f us", delta_time_us);
    log("TOF: %f us", tof_us);
    log("DISTANCE: %f feet", distance_ft); // ms * (ft/ms)
}

static void NodeB(spi_device_handle_t dev_handle) {
    //Wait for transmit from node A.
    uint8_t rx[PING_MSG_LEN];
    ESP_ERROR_CHECK(uwb_receive((uint8_t*)rx, PING_MSG_LEN, dev_handle));


    //Store receive time
    uint64_t rx_time;
    ESP_ERROR_CHECK(get_time(&rx_time, dev_handle, DWM_REG_RX_TIME));
    uint64_t TRP = rx_time;

    char printbuf[PING_MSG_LEN + 1];
    memcpy(printbuf, rx, PING_MSG_LEN);
    printbuf[PING_MSG_LEN] = '\0';
    log("Received: %s", printbuf);

    //Store transmit time, TSR
    uint64_t tx_time;
    ESP_ERROR_CHECK(get_time(&tx_time, dev_handle, DWM_REG_TX_TIME));
    uint64_t TSR = tx_time;
    ESP_ERROR_CHECK(uwb_delayed_transmit((uint8_t*)PING_MSG, PING_MSG_LEN, DELAY_CONST_MS, dev_handle));
    

    //REceive TRT initiator round trip and Initiator delay;
    uint8_t t_delta_buf[10];
    ESP_ERROR_CHECK(uwb_receive((uint8_t*)t_delta_buf, 10, dev_handle));
    log("Received t_delta message");
    
    uint64_t t_delta = 0;
    uint64_t TRT = 0;
    for (int i = 0; i < sizeof(t_delta_buf)/2; ++i) {
        t_delta |= ((uint64_t)t_delta_buf[i] << (8 * i));
    }
      for (int i = sizeof(t_delta_buf)/2; i < sizeof(t_delta_buf); ++i) {
        TRT |= ((uint64_t)t_delta_buf[i] << (8 * i));
    }

    log("t_delta factor: %llu", t_delta);
    log("TRT is: %llu", TRT);
     // Wait for LDEDONE bit confirm RX stamp is good
    uint8_t sys_status[5];
    do {
        ESP_ERROR_CHECK(uwb_read_reg(DWM_REG_SYSTEM_EVENT_STATUS, (uint8_t*)&sys_status, sizeof(sys_status), dev_handle));
        log("polled ldedone bit: %X %X %X %X %X", sys_status[4], sys_status[3], sys_status[2], sys_status[1], sys_status[0]);
    } while (((sys_status[1] >> 2) & 1) == 0);
    
    //Store final receive time.
    ESP_ERROR_CHECK(get_time(&rx_time, dev_handle, DWM_REG_RX_TIME));
    uint64_t TRF = rx_time;
 

    //account for wrap, calculate responder round trip delay time TART and responder responce time. TRRT
    uint64_t TRRT = (TSR - TRP) & ((1ULL << 40) - 1);
    uint64_t TART = (TSR - TRF)  & ((1ULL << 40) - 1);

    //Subtract initation and responder delay from round trip times
    uint64_t delta_time_dtu = (((TRT - t_delta) + (TART-TRRT)))/4;



    // uint64_t delta_time_dtu;TRT
    // if (rx_time >= tx_time) {
    //     delta_time_dtu = rx_time - tx_time - t_delta;
    // } else {
    //     // Counter wrapped around
    //     delta_time_dtu = ((1ULL << 40) - tx_time - t_delta) + rx_time;
    // }

    double delta_time_us = delta_time_dtu * 1.565e-5; // microseconds
    double tof_us = (delta_time_us) / 2.0; // tof one-way in microseconds
    double distance_ft = tof_us * 983.57105643045;

    log("TRT time %llu DTU, TART time %llu DTU, Responder TRSP: %llu DTU, Initiator TRSP %llu DTU",TRT,TART,TRRT,t_delta)
    log("TX time: %llu DTU --- RX time: %llu DTU", tx_time, rx_time);
    log("Delta time: %f us", delta_time_us);
    log("TOF: %f us", tof_us);
    log("DISTANCE: %f feet", distance_ft); // ms * (ft/ms)



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
        .clock_speed_hz = APB_CLK_FREQ / 80, // 1 MHz
        .input_delay_ns = 50,
        .spics_io_num = DW_CS,
        .flags = 0,
        .queue_size = 4, // TODO: queue size
        .pre_cb = NULL,
        .post_cb = NULL
    };

    // Hard reset the device
    uwb_hard_reset();
    vTaskDelay(pdMS_TO_TICKS(10));

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

    // LDE algorithm loaded to allow rx timestamping

    // Step 1
    uint8_t buf[2] = {0x01, 0x03};
    uwb_write_subreg(DWM_REG_POWER_MANAGEMENT, 0x00, (uint8_t*)buf, sizeof(buf), dev_handle);

    // Step 2
    buf[1] = 0x80;
    buf[0] = 0x00;
    uwb_write_subreg(DWM_REG_OTP_MEMORY, DWM_SUB_REG_OTP_CTRL, (uint8_t*)buf, sizeof(buf), dev_handle);
    vTaskDelay(pdMS_TO_TICKS(1));

    // Step 3
    buf[1] = 0x02;
    buf[0] = 0x00;
    uwb_write_subreg(DWM_REG_POWER_MANAGEMENT, 0x00, (uint8_t*)buf, sizeof(buf), dev_handle);

    // Re-add SPI device with upped clock speed
    // LDE load must happen at < 3 MHz
    dev_config.clock_speed_hz = SPI_MASTER_FREQ_11M;
    ESP_ERROR_CHECK(spi_bus_remove_device(dev_handle));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_config, &dev_handle));

    uint8_t sys_status[5];
    ESP_ERROR_CHECK(uwb_read_subreg(DWM_REG_SYSTEM_EVENT_STATUS, 0x00, (uint8_t*)&sys_status, sizeof(sys_status), dev_handle));
    log("polled sys status: %X %X %X %X %X", sys_status[4], sys_status[3], sys_status[2], sys_status[1], sys_status[0]);

    // uint8_t sys_mask[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    // uwb_write_reg(DWM_REG_SYSTEM_EVENT_STATUS, sys_mask, 5, dev_handle);
    // vTaskDelay(pdMS_TO_TICKS(1000));
    // ESP_ERROR_CHECK(uwb_read_reg(DWM_REG_SYSTEM_EVENT_STATUS, (uint8_t*)&sys_status, sizeof(sys_status), dev_handle));
    // log("polled sys status: %X %X %X %X %X", sys_status[4], sys_status[3], sys_status[2], sys_status[1], sys_status[0]);

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

esp_err_t uwb_read_reg(uint8_t reg, uint8_t* rx, size_t len, spi_device_handle_t dev_handle) {
    // Lower 6 bits store actual register
    // MSbit = 0 represents read
    reg = 0x00 | (reg & 0x3F);

    size_t total_len = 1 + len;

    uint8_t txtemp[total_len], rxtemp[total_len];
    
    txtemp[0] = reg;

    spi_transaction_t transaction = {
        .length = BYTES_TO_BITS * total_len,
        .tx_buffer = txtemp,
        .rx_buffer = rxtemp,
    };

    ESP_ERROR_CHECK(spi_device_transmit(dev_handle, &transaction));

    memcpy(rx, rxtemp + 1, len);

    return ESP_OK;
}

// TODO: needs to take 16-bit subreg
esp_err_t uwb_read_subreg(uint8_t reg, uint8_t subreg, uint8_t* rx, size_t len, spi_device_handle_t dev_handle) {
    // Lower 6 bits store actual register
    // MSbit = 0 represents read
    // Second MSbit = 1 represent sub-index is present
    reg = 0b01000000 | (reg & 0x3F);

    // Extended address disabled (bit 7)
    // TODO: conditionally enable this when needed
    subreg = (subreg & 0x7F);

    size_t total_len = 2 + len;

    uint8_t txtemp[total_len], rxtemp[total_len];

    txtemp[0] = reg;
    txtemp[1] = subreg;    

    spi_transaction_t transaction = {
        .length = BYTES_TO_BITS * total_len,
        .tx_buffer = txtemp,
        .rx_buffer = rxtemp,
    };

    ESP_ERROR_CHECK(spi_device_transmit(dev_handle, &transaction));

    memcpy(rx, rxtemp + 2, len);

    return ESP_OK;
}

esp_err_t uwb_write_reg(uint8_t reg, uint8_t* tx, size_t len, spi_device_handle_t dev_handle) {
    // Lower 6 bits store actual register
    // MSbit = 1 represents write
    reg = 0x80 | (reg & 0x3F);

    size_t total_len = 1 + len;

    uint8_t buf[total_len];

    buf[0] = reg;
    memcpy(buf + 1, tx, len);

    spi_transaction_t transaction = {
        .length = BYTES_TO_BITS * total_len,
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

    size_t total_len = 2 + len;

    uint8_t buf[total_len];
    buf[0] = reg;
    buf[1] = subreg;
    memcpy(buf + 2, tx, len);

    spi_transaction_t transaction = {
        .length = BYTES_TO_BITS * total_len,
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

// TODO: remove tx and len parameters?
esp_err_t uwb_delayed_transmit(uint8_t*, size_t, uint64_t delay_ms, spi_device_handle_t dev_handle) {
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
    delayed_time &= ((1ULL << 40) - 1);

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

    // Create payload
    constexpr size_t len = 5;
    uint8_t tx[len];

    // Wait for LDEDONE bit
    uint8_t sys_status[5];
    do {
        ESP_ERROR_CHECK(uwb_read_reg(DWM_REG_SYSTEM_EVENT_STATUS, (uint8_t*)&sys_status, sizeof(sys_status), dev_handle));
        log("polled ldedone bit: %X %X %X %X %X", sys_status[4], sys_status[3], sys_status[2], sys_status[1], sys_status[0]);
    } while (((sys_status[1] >> 2) & 1) == 0);

    // Get RX time
    uint64_t rx_time;
    ESP_ERROR_CHECK(get_time(&rx_time, dev_handle, DWM_REG_RX_TIME));

    // Compute delta time factor to send
    uint64_t wrap = (1ULL << 40);
    uint64_t t_delta;

    if (delayed_time >= rx_time)
        t_delta = delayed_time - rx_time;
    else
        t_delta = (wrap - rx_time) + delayed_time;
    // uint64_t t_delta = delayed_time - rx_time;
    memcpy(tx, (uint8_t*)&t_delta, len);
    log("Delta time factor: %llu DTU", t_delta);

    // Write payload
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
    do {
        ESP_ERROR_CHECK(uwb_read_reg(DWM_REG_SYSTEM_EVENT_STATUS, (uint8_t*)&sys_status, sizeof(sys_status), dev_handle));
        // log("SYS_STATUS BIT: %d", (sys_status[0] >> 7) & 1);
        vTaskDelay(pdTICKS_TO_MS(1));
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
