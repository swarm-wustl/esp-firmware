#include "dwm.h"
#include "esp32.h"
#include "unity.h"

TEST_CASE("Read DWM1000 ID", "[dwm]") {
  ESP32::SPI spi{GPIO_NUM_4};
  ESP32::GPIO gpio{};
  DWM dwm_device{std::move(spi), std::move(gpio), GPIO_NUM_27, GPIO_NUM_34};

  auto id = dwm_device.get_device_id();
  TEST_ASSERT_TRUE(id.has_value());
  TEST_ASSERT_EQUAL(0xDECA0130, *id);
}

TEST_CASE("Set and read network ID and device address", "[dwm]") {
    ESP32::SPI spi{GPIO_NUM_4};
    ESP32::GPIO gpio{};
    DWM dwm_device{std::move(spi), std::move(gpio), GPIO_NUM_27, GPIO_NUM_34};

    // 1. Configure network (device_address = 0x5678, network_id = 0xDECA)
    auto config_res = dwm_device.configure_network(0x5678, 0xDECA);
    TEST_ASSERT_TRUE(config_res.has_value()); // Ensures SPI write succeeded

    // 2. Read back configuration
    auto panadr = dwm_device.read_network();
    TEST_ASSERT_TRUE(panadr.has_value());

    // 3. Verify extracted components
    if (panadr.has_value()) {
        uint16_t short_address = static_cast<uint16_t>(*panadr & 0xFFFF);
        uint16_t pan_id        = static_cast<uint16_t>(*panadr >> 16);

        // Check full 32-bit register packing
        TEST_ASSERT_EQUAL_HEX32(0xDECA5678, *panadr);

        // Check individual 16-bit fields
        TEST_ASSERT_EQUAL_HEX16(0x5678, short_address);
        TEST_ASSERT_EQUAL_HEX16(0xDECA, pan_id);
    }
}


TEST_CASE("TX_FCTRL PRF write, read-back, and reset to default", "[dwm]") {
  // Keep in a blocked scope so it de-inits everything at the end of the block
  {
    ESP32::SPI spi{GPIO_NUM_4};
    ESP32::GPIO gpio{};
    // DWM dwm_test{std::move(spi), std::move(gpio), 13};
    DWM dwm_device{std::move(spi), std::move(gpio), GPIO_NUM_27, GPIO_NUM_34};

    // Check default PRF is 16 MHz per datasheet section 2.5
    auto default_prf = dwm_device.get_tx_prf();
    TEST_ASSERT_TRUE(default_prf.has_value());
    TEST_ASSERT_EQUAL(PRF::MHZ_16, *default_prf);

    // Write a different value
    TEST_ASSERT_TRUE(dwm_device.set_tx_prf(PRF::MHZ_64).has_value());

    auto new_prf = dwm_device.get_tx_prf();
    TEST_ASSERT_TRUE(new_prf.has_value());
    TEST_ASSERT_EQUAL(PRF::MHZ_64, *new_prf);
  }

  // Hard reset and verify default restored
  // Note: DWM constructor calls hard_reset(), so just reconstruct
  {
    ESP32::SPI spi{GPIO_NUM_4};
    ESP32::GPIO gpio{};
    DWM dwm_device{std::move(spi), std::move(gpio), GPIO_NUM_27, GPIO_NUM_34};

    auto reset_prf = dwm_device.get_tx_prf();
    TEST_ASSERT_TRUE(reset_prf.has_value());
    TEST_ASSERT_EQUAL(PRF::MHZ_16, *reset_prf);
  }
}

TEST_CASE("Read TX and RX timestamps", "[dwm]") {
  ESP32::SPI spi{GPIO_NUM_4};
  ESP32::GPIO gpio{};
  DWM dwm_device{std::move(spi), std::move(gpio), GPIO_NUM_27, GPIO_NUM_34};

  // no TX/RX has occurred so the stamps are meaningless, but the register read
  // itself should still succeed over SPI
  TEST_ASSERT_TRUE(dwm_device.get_rx_timestamp().has_value());
  TEST_ASSERT_TRUE(dwm_device.get_tx_timestamp().has_value());
}
