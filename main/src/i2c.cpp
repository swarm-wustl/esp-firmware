// #include "driver/i2c.h"

// #define I2C_MASTER_SCL_IO    14  // Default I2C clock
// #define I2C_MASTER_SDA_IO    4// Default I2C data
// #define TEST_I2C_PORT        0   // I2C port number (0 or 1)

// i2c_master_bus_config_t i2c_mst_config = {
//     .clk_source = I2C_CLK_SRC_DEFAULT,
//     .i2c_port = TEST_I2C_PORT,
//     .scl_io_num = I2C_MASTER_SCL_IO,
//     .sda_io_num = I2C_MASTER_SDA_IO,
//     .glitch_ignore_cnt = 7,
//     .flags.enable_internal_pullup = true,
// };

// i2c_master_bus_handle_t bus_handle;
// ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

// i2c_device_config_t dev_cfg = {
//     .dev_addr_length = I2C_ADDR_BIT_LEN_7,
//     .device_address = 0x58,
//     .scl_speed_hz = 100000,
// };

// i2c_master_dev_handle_t dev_handle;
// ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
// ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, data_wr, DATA_LENGTH, -1));

// this code uses version 4.4 of the i2c driver:https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32/api-reference/peripherals/i2c.html
// The IMU datasheet and register map/info is listed below:
// https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
// https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "i2c-simple-example";

#define I2C_MASTER_SCL_IO           14      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           4      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          100000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000
#define FLAGS_ALLOCATED             0

#define IMU_SENSOR_ADDR             0x68       /*!< Slave address of the MPU6050 */
#define IMU_WHO_AM_I_ADDR           0x75
#define IMU_PWR_MGMT_1              0x6B
#define IMU_PWR_MGMT_1_RESET_BIT    7


/**
 * @brief Read a sequence of bytes from a MPU6050 sensor registers
 */
static esp_err_t mpu9250_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, IMU_SENSOR_ADDR, &reg_addr, 1, data, len, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}


static esp_err_t imu_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, IMU_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);

    return ret;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    // int i2c_master_port = I2C_MASTER_NUM;

    // i2c_config_t conf = {
    //     .mode = I2C_MODE_MASTER,
    //     .sda_io_num = I2C_MASTER_SDA_IO,
    //     .scl_io_num = I2C_MASTER_SCL_IO,
    //     .sda_pullup_en = GPIO_PULLUP_ENABLE,
    //     .scl_pullup_en = GPIO_PULLUP_ENABLE,
    //     .master.clk_speed = I2C_MASTER_FREQ_HZ,
    // };

    // i2c_param_config(i2c_master_port, &conf);

    // return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, FLAGS_ALLOCATED);

   
    int i2c_master_port = I2C_MASTER_NUM;

    // Initialize the configuration struct
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    // Apply the configuration
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %d", err);
        return err;
    }

    // Install the I2C driver
    err = i2c_driver_install(
        i2c_master_port, 
        conf.mode, 
        I2C_MASTER_RX_BUF_DISABLE, 
        I2C_MASTER_TX_BUF_DISABLE, 
        FLAGS_ALLOCATED
    );
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %d", err);
    }

    return err;

}
