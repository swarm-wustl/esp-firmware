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

TEST_CASE("TX_FCTRL PRF write, read-back, and reset to default", "[dwm]") {
  // Keep in a blocked scope so it de-inits everything at the end of the block
  {
    ESP32::SPI spi{GPIO_NUM_4};
    ESP32::GPIO gpio{};
    // DWM dwm_test{std::move(spi), std::move(gpio), 13};
    DWM dwm_device{std::move(spi), std::move(gpio), GPIO_NUM_27, GPIO_NUM_34};

    // Check default PRF is 16 MHz per datasheet section 2.5
    TEST_ASSERT_EQUAL(dwm_device.get_tx_prf(), PRF::MHZ_16);

    // Write a different value
    dwm_device.set_tx_prf(PRF::MHZ_64);
    TEST_ASSERT_EQUAL(dwm_device.get_tx_prf(), PRF::MHZ_64);
  }

  // Hard reset and verify default restored
  // Note: DWM constructor calls hard_reset(), so just reconstruct
  {
    ESP32::SPI spi{GPIO_NUM_4};
    ESP32::GPIO gpio{};
    DWM dwm_device{std::move(spi), std::move(gpio), GPIO_NUM_27, GPIO_NUM_34};
    TEST_ASSERT_EQUAL(dwm_device.get_tx_prf(), PRF::MHZ_16);
  }
}
