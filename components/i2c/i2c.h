// this code uses version 4.4 of the i2c driver:
// https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32/api-reference/peripherals/i2c.html
// The IMU datasheet and register map/info is listed below:
// https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
// https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf


#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO           14      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           4      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          100000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000
#define FLAGS_ALLOCATED             0


#define IMU_SENSOR_ADDR             0x68       /*!< Slave address of the MPU6050 */
#define IMU_WHO_AM_I_ADDR           0x75
#define IMU_PWR_MGMT_1              0x6B
#define IMU_PWR_MGMT_1_RESET_BIT    7


//Gyroscope measurement bits - they are organized in a big-endian byte order
#define GYRO_XOUT_H                 0x43
#define GYRO_XOUT_L                 0x44
#define GYRO_YOUT_H                 0x45
#define GYRO_YOUT_L                 0x46
#define GYRO_ZOUT_H                 0x47
#define GYRO_ZOUT_L                 0x48


//Accelerometer measurement bits - they are organized in a big-endian byte order
#define ACCEL_XOUT_H                 0x3B
#define ACCEL_XOUT_L                 0x3C
#define ACCEL_YOUT_H                 0x3D
#define ACCEL_YOUT_L                 0x3E
#define ACCEL_ZOUT_H                 0x3F
#define ACCEL_ZOUT_L                 0x40




esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len);




esp_err_t imu_register_write_byte(uint8_t reg_addr, uint8_t data);
esp_err_t imu_read_gyroscope_data(int16_t *gx, int16_t *gy, int16_t *gz);


esp_err_t imu_read_accelerometer_data(int16_t *gx, int16_t *gy, int16_t *gz);


esp_err_t i2c_master_init(void);
