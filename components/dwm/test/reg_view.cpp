#include "dwm.h"
#include "esp32.h"
#include "unity.h"
#include <algorithm>
#include <ranges>

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

TEST_CASE("Test write and read-back TX buffer", "[dwm_reg]") {
  ESP32::SPI spi{GPIO_NUM_4};
  DWMRegisterView<ESP32::SPI, DWMRegisterID::TX_BUFFER> tx_buf_reg{spi};

  std::array<std::byte, 1024> test_data, response_data;

  // Generate test values [0, 1, .., 1023]
  std::ranges::copy(std::views::iota(0, 1024) |
                        std::views::transform([](int i) {
                          return std::byte{static_cast<uint8_t>(i)};
                        }),
                    test_data.begin());

  tx_buf_reg.write_data(std::span{test_data});

  // Try to read back data
  std::ranges::copy(tx_buf_reg.value(), response_data.begin());
  TEST_ASSERT(test_data == response_data);
}
