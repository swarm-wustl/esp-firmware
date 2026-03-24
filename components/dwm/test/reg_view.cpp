#include "dwm.h"
#include "esp32.h"
#include "unity.h"

// TODO: make a mock DWM SPI struct

TEST_CASE("Test read device ID register view", "[dwm_reg]") {
  ESP32::SPI spi{GPIO_NUM_4};
  DWMRegisterView<ESP32::SPI, DWMRegisterID::DEV_ID> dev_id_reg{spi};

  TEST_ASSERT_EQUAL(dev_id_reg.size(), 4);
  TEST_ASSERT_EQUAL(dev_id_reg.value(), 0xDECA0130);
}

TEST_CASE("Test write device ID register view", "[dwm_reg]") {
  ESP32::SPI spi{GPIO_NUM_4};
  DWMRegisterView<ESP32::SPI, DWMRegisterID::DEV_ID> dev_id_reg{spi};

  // ID register is read-only, so value should stay the same
  TEST_ASSERT_EQUAL(dev_id_reg.value(), 0xDECA0130);
  dev_id_reg |= 0xFFFFFFFF;
  TEST_ASSERT_EQUAL(dev_id_reg.value(), 0xDECA0130);
}
