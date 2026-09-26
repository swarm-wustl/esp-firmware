// this code uses version 4.4 of the i2c driver:
// https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32/api-reference/peripherals/i2c.html
// The IMU datasheet and register map/info is listed below:
// https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
// https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf


#include "i2c.h"
#include <unordered_map>


static const char *TAG = "i2c-simple-example";

/**
* @brief Read a sequence of bytes from a MPU6050 sensor registers
*/
esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
   return i2c_master_write_read_device(I2C_MASTER_NUM, IMU_SENSOR_ADDR, &reg_addr, 1, data, len, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}




esp_err_t imu_register_write_byte(uint8_t reg_addr, uint8_t data)
{
   int ret;
   uint8_t write_buf[2] = {reg_addr, data};


   ret = i2c_master_write_to_device(I2C_MASTER_NUM, IMU_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);


   return ret;
}

esp_err_t imu_read_gyroscope_data(int16_t *gx, int16_t *gy, int16_t *gz){
   uint8_t raw[6];
   esp_err_t ret = mpu6050_register_read(GYRO_XOUT_H, raw, 6);
   if (ret != ESP_OK) return ret;


   *gx = (int16_t)((raw[0] << 8) | raw[1]);
   *gy = (int16_t)((raw[2] << 8) | raw[3]);
   *gz = (int16_t)((raw[4] << 8) | raw[5]);


   return ESP_OK;
}


esp_err_t imu_read_accelerometer_data(int16_t *gx, int16_t *gy, int16_t *gz){
   uint8_t raw[6];
   esp_err_t ret = mpu6050_register_read(ACCEL_XOUT_H, raw, 6);
   if (ret != ESP_OK) return ret;


   *gx = (int16_t)((raw[0] << 8) | raw[1]);
   *gy = (int16_t)((raw[2] << 8) | raw[3]);
   *gz = (int16_t)((raw[4] << 8) | raw[5]);


   return ESP_OK;
}

esp_err_t imu_calibrate_readings(std::unordered_map<std::string, int16_t> *accel_offset_data, std::unordered_map<std::string, int16_t> *gyro_offset_data){
  int16_t gx_total = 0;
  int16_t gy_total = 0;
  int16_t gz_total = 0;

  int16_t ax_high= INT16_MIN;
  int16_t ax_low= INT16_MAX;
  
  int16_t ay_high = INT16_MIN;
  int16_t ay_low = INT16_MAX;
  
  int16_t az_high = INT16_MIN;
  int16_t az_low = INT16_MAX;
  
  int16_t gx_offset = 0;
  int16_t gy_offset = 0;
  int16_t gz_offset = 0;

  int16_t ax_offset = 0;
  int16_t ay_offset = 0;
  int16_t az_offset = 0;

  int16_t ax_scale = 0;
  int16_t ay_scale = 0;
  int16_t az_scale = 0;

  int16_t ax, ay, az, gx, gy, gz;
  for(int i = 0; i <= NUM_SAMPLES; ++i){

    
      ESP_ERROR_CHECK(imu_read_gyroscope_data(&gx, &gy, &gz));
      ESP_ERROR_CHECK(imu_read_accelerometer_data(&ax, &ay, &az));

      gx_total += gx 
      gy_total += gy
      gz_total += gz

      ax_low = std::min(ax, ax_low);
      ax_high = std::max(ax, ax_high);


      ay_low = std::min(ay, ay_low);
      ay_high = std::max(ay, ay_high);


      az_low = std::min(az, az_low);
      az_high = std::max(az, az_high);
 }

  gx_offset = gx_total/NUM_SAMPLES;
  gy_offset = gy_total/NUM_SAMPLES;
  gz_offset = gz_total/NUM_SAMPLES;


  ax_offset = (ax_high + ax_low)/2;
  ay_offset = (ay_high + ay_low)/2; 
  az_offset = (az_high + az_low)/2;

  ax_scale = LSBSENS_ACCEL/(ax_high - ax_low);
  ay_scale = LSBSENS_ACCEL/(ay_high - ay_low);
  az_scale = LSBSENS_ACCEL/(az_high - az_low);

  *gyro_offset_data["gx_offset"] = gx_offset;
  *gyro_offset_data["gy_offset"] = gy_offset;
  *gyro_offset_data["gz_offset"] = gz_offset;


  *accel_offset_data["ax_offset"] = ax_offset;
  *accel_offset_data["ay_offset"] = ay_offset;
  *accel_offset_data["az_offset"] = az_offset;


  *accel_offset_data["ax_scale"] = ax_scale;
  *accel_offset_data["ay_scale"] = ay_scale;
  *accel_offset_data["az_scale"] = az_scale;


}


/**
* @brief i2c master initialization
*/
esp_err_t i2c_master_init(void)
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


 
   i2c_port_t i2c_master_port = I2C_MASTER_NUM;


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
