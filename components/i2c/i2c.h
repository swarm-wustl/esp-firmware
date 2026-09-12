// this code uses version 4.4 of the i2c driver:
// https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32/api-reference/peripherals/i2c.html
// The IMU datasheet and register map/info is listed below:
// https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
// https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf


#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"




static esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len);




static esp_err_t imu_register_write_byte(uint8_t reg_addr, uint8_t data);
static esp_err_t imu_read_gyroscope_data(int16_t *gx, int16_t *gy, int16_t *gz);


static esp_err_t imu_read_accelerometer_data(int16_t *gx, int16_t *gy, int16_t *gz);


static esp_err_t i2c_master_init(void);
