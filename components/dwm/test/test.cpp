#include "dwm.h"
#include "esp32.h"
#include "unity.h"

TEST_CASE("Read DWM1000 ID", "[dwm]") {
  ESP32::SPI spi{GPIO_NUM_4};
  ESP32::GPIO gpio{};
  DWM dwm_device{std::move(spi), std::move(gpio), GPIO_NUM_27, GPIO_NUM_34};

  auto id = dwm_device.get_device_id();
  TEST_ASSERT_EQUAL(id, 0xDECA0130);
}
