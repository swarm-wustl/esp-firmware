#include "dwm.h"
#include "esp32.h"
#include "unity.h"
#include <algorithm>
#include <ranges>

// TODO: figure out a much better way to do this
void hard_reset() {
  ESP32::GPIO gpio_{};
  auto rst_pin_ = GPIO_NUM_27;

  gpio_num_t rst = static_cast<gpio_num_t>(rst_pin_);

  gpio_.set_direction(rst, GPIO_MODE_OUTPUT);
  gpio_.set_level(rst, HAL::Voltage::LOW);
  gpio_.delay_ms(10);

  // Release — set to input, let internal pull-up take over
  gpio_.set_direction(rst, GPIO_MODE_INPUT);
  gpio_.delay_ms(10);
}

TEST_CASE("Test read device ID register view", "[dwm_reg]") {
  hard_reset();
  ESP32::SPI spi{GPIO_NUM_4};
  DWMRegisterView<ESP32::SPI, DWMRegisterID::DEV_ID> dev_id_reg{spi};

  TEST_ASSERT_EQUAL(dev_id_reg.size(), 4);

  auto id = dev_id_reg.read();
  TEST_ASSERT_TRUE(id.has_value());
  TEST_ASSERT_EQUAL(0xDECA0130, *id);
}

TEST_CASE("Test write and read-back TX buffer", "[dwm_reg]") {
  hard_reset();
  ESP32::SPI spi{GPIO_NUM_4};
  DWMRegisterView<ESP32::SPI, DWMRegisterID::TX_BUFFER> tx_buf_reg{spi};

  std::array<std::array<std::byte, 1024>, 2> test_data;

  // Generate random values in range [0, 255]
  std::ranges::generate(test_data[0], []() {
    return std::byte{static_cast<uint8_t>(std::rand() % 256)};
  });

  // Write data, read back the response, and compare
  TEST_ASSERT_TRUE(tx_buf_reg.write_data(std::span{test_data[0]}).has_value());

  auto readback = tx_buf_reg.read();
  TEST_ASSERT_TRUE(readback.has_value());
  TEST_ASSERT(std::ranges::equal(test_data[0], *readback));

  // Re-generate random values in range [0, 255]
  std::ranges::generate(test_data[1], []() {
    return std::byte{static_cast<uint8_t>(std::rand() % 256)};
  });

  TEST_ASSERT_TRUE(tx_buf_reg.write_data(std::span{test_data[1]}).has_value());

  // Read back from the device, still the new test data
  readback = tx_buf_reg.read();
  TEST_ASSERT_TRUE(readback.has_value());
  TEST_ASSERT(std::ranges::equal(test_data[1], *readback));
}
