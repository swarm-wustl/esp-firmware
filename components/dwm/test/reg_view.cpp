#include "dwm.h"
#include "dwm_data.h"
#include "esp32.h"
#include "unity.h"
#include <algorithm>
#include <ranges>

// TODO: make a mock DWM SPI struct

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
  TEST_ASSERT_EQUAL(dev_id_reg.value(), 0xDECA0130);
}

TEST_CASE("Test write device ID register view", "[dwm_reg]") {
  hard_reset();
  ESP32::SPI spi{GPIO_NUM_4};
  DWMRegisterView<ESP32::SPI, DWMRegisterID::DEV_ID> dev_id_reg{spi};

  // ID register is read-only, so value should stay the same
  TEST_ASSERT_EQUAL(dev_id_reg.value(), 0xDECA0130);
  dev_id_reg |= 0xFFFFFFFF;
  dev_id_reg.read_data();
  TEST_ASSERT_EQUAL(dev_id_reg.value(), 0xDECA0130);
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
  tx_buf_reg.write_data(std::span{test_data[0]});
  tx_buf_reg.read_data();
  TEST_ASSERT(std::ranges::equal(test_data[0], tx_buf_reg.value()));

  // Re-generate random values in range [0, 255]
  std::ranges::generate(test_data[1], []() {
    return std::byte{static_cast<uint8_t>(std::rand() % 256)};
  });

  // Write but don't read, should be old test data still
  tx_buf_reg.write_data(std::span{test_data[1]});
  TEST_ASSERT(std::ranges::equal(test_data[0], tx_buf_reg.value()));

  // Read back response, should now equal new test data
  tx_buf_reg.read_data();
  TEST_ASSERT(std::ranges::equal(test_data[1], tx_buf_reg.value()));
}

// --- DWMData unit tests (pure value type, no hardware needed) ---

TEST_CASE("DWMData stores integers little-endian", "[dwm_data]") {
  DWMData<4> d{static_cast<uint32_t>(0xDECA0130)};

  // Little-endian: least-significant byte first
  TEST_ASSERT_EQUAL_HEX8(0x30, static_cast<uint8_t>(d.byte(0)));
  TEST_ASSERT_EQUAL_HEX8(0x01, static_cast<uint8_t>(d.byte(1)));
  TEST_ASSERT_EQUAL_HEX8(0xCA, static_cast<uint8_t>(d.byte(2)));
  TEST_ASSERT_EQUAL_HEX8(0xDE, static_cast<uint8_t>(d.byte(3)));

  // Round-trips back to the same integer
  TEST_ASSERT_EQUAL_HEX32(0xDECA0130, d.to_uint());
}

TEST_CASE("DWMData supports sub-uint64 widths (5-byte register)", "[dwm_data]") {
  // DW1000 timestamp registers are 40 bits / 5 bytes
  DWMData<5> d{static_cast<uint64_t>(0xAA'BBCCDDEEULL)};
  TEST_ASSERT_EQUAL_HEX64(0xAABBCCDDEEULL, d.to_uint());

  // High bytes beyond the 5-byte width are dropped on construction
  DWMData<5> truncated{static_cast<uint64_t>(0xFF'FF'AABBCCDDEEULL)};
  TEST_ASSERT_EQUAL_HEX64(0xAABBCCDDEEULL, truncated.to_uint());
}

TEST_CASE("DWMData bit access", "[dwm_data]") {
  DWMData<2> d{static_cast<uint16_t>(0b1010'0000'0000'0101)};

  // Absolute bit numbering
  TEST_ASSERT_EQUAL(1, d.bit(0));
  TEST_ASSERT_EQUAL(0, d.bit(1));
  TEST_ASSERT_EQUAL(1, d.bit(2));
  TEST_ASSERT_EQUAL(1, d.bit(15));
  TEST_ASSERT_EQUAL(0, d.bit(14));
  TEST_ASSERT_EQUAL(1, d.bit(13));

  // Byte + offset form
  TEST_ASSERT_EQUAL(1, d.bit(0, 0)); // byte 0, bit 0
  TEST_ASSERT_EQUAL(1, d.bit(1, 7)); // byte 1, bit 7
}

TEST_CASE("DWMData bit_range read (Verilog x[hi:lo])", "[dwm_data]") {
  DWMData<4> d{static_cast<uint32_t>(0xABCD1234)};

  TEST_ASSERT_EQUAL_HEX(0x4, d.bit_range(3, 0));
  TEST_ASSERT_EQUAL_HEX(0x34, d.bit_range(7, 0));
  TEST_ASSERT_EQUAL_HEX(0xABCD, d.bit_range(31, 16));
  // Single-bit range. 0x...4 == 0b0100, so bit 0 is 0 and bit 2 is 1.
  TEST_ASSERT_EQUAL_HEX(0x0, d.bit_range(0, 0));
  TEST_ASSERT_EQUAL_HEX(0x1, d.bit_range(2, 2));
}

TEST_CASE("DWMData write_bit_range leaves other bits untouched", "[dwm_data]") {
  DWMData<4> d{static_cast<uint32_t>(0x0000'0000)};

  d.write_bit_range(7, 0, 0xFF);
  TEST_ASSERT_EQUAL_HEX32(0x0000'00FF, d.to_uint());

  // Writing a higher range must not disturb the low byte
  d.write_bit_range(15, 8, 0xAB);
  TEST_ASSERT_EQUAL_HEX32(0x0000'ABFF, d.to_uint());

  // Overwriting an existing range clears it first
  d.write_bit_range(7, 0, 0x12);
  TEST_ASSERT_EQUAL_HEX32(0x0000'AB12, d.to_uint());

  // Value wider than the range is masked to the range width
  d.write_bit_range(3, 0, 0xFF);
  TEST_ASSERT_EQUAL_HEX32(0x0000'AB1F, d.to_uint());
}

TEST_CASE("DWMData equality and span round-trip", "[dwm_data]") {
  DWMData<4> a{static_cast<uint32_t>(0x11223344)};
  DWMData<4> b{static_cast<uint32_t>(0x11223344)};
  DWMData<4> c{static_cast<uint32_t>(0x55667788)};

  TEST_ASSERT(a == b);
  TEST_ASSERT(!(a == c));

  // span() exposes the underlying bytes for the SPI layer
  TEST_ASSERT_EQUAL(4, a.span().size());
}
