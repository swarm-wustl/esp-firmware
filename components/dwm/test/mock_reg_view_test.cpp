#include "dwm.h"
#include "mock_spi.h"
#include "peripheral_hal.h"
#include "unity.h"
#include <cstddef>
#include <vector>

static_assert(HAL::GenericSPIController<MockSPI>);

TEST_CASE("read() serves canned bytes little-endian", "[dwm_mock]") {
  MockSPI spi;
  spi.read_response = {std::byte{0x30}, std::byte{0x01}, std::byte{0xCA},
                       std::byte{0xDE}};

  DWMRegisterView<MockSPI, DWMRegisterID::DEV_ID> reg{spi};
  auto v = reg.read();

  TEST_ASSERT_TRUE(v.has_value());
  TEST_ASSERT_EQUAL_HEX32(0xDECA0130, *v);
}

TEST_CASE("read() propagates a transfer failure", "[dwm_mock]") {
  MockSPI spi;
  spi.fail_with = HAL::SpiError::TransferFailed;

  DWMRegisterView<MockSPI, DWMRegisterID::DEV_ID> reg{spi};
  auto v = reg.read();

  TEST_ASSERT_FALSE(v.has_value());
  TEST_ASSERT_TRUE(v.error() == HAL::SpiError::TransferFailed);
}

TEST_CASE("write_data() propagates a transfer failure", "[dwm_mock]") {
  MockSPI spi;
  spi.fail_with = HAL::SpiError::Timeout;

  DWMRegisterView<MockSPI, DWMRegisterID::TX_FCTRL> reg{spi};
  DWMData<5> payload{static_cast<uint64_t>(0x1234)};
  auto res = reg.write_data(payload.span());

  TEST_ASSERT_FALSE(res.has_value());
  TEST_ASSERT_TRUE(res.error() == HAL::SpiError::Timeout);
}

TEST_CASE("write_bit_range short-circuits when the read fails", "[dwm_mock]") {
  MockSPI spi;
  spi.fail_with = HAL::SpiError::TransferFailed;

  DWMRegisterView<MockSPI, DWMRegisterID::TX_FCTRL> reg{spi};
  auto res = reg.write_bit_range(17, 16, 0b10);

  TEST_ASSERT_FALSE(res.has_value());
  TEST_ASSERT_TRUE(res.error() == HAL::SpiError::TransferFailed);
  // the read failed, so the write transfer must never have happened
  TEST_ASSERT_EQUAL(1, spi.transfer_count);
  TEST_ASSERT_TRUE(spi.last_write.empty());
}

TEST_CASE("write_bit_range reads, modifies, then writes back", "[dwm_mock]") {
  MockSPI spi;
  spi.read_response = std::vector<std::byte>(5, std::byte{0});

  DWMRegisterView<MockSPI, DWMRegisterID::TX_FCTRL> reg{spi};
  auto res = reg.write_bit_range(17, 16, 0b10);

  TEST_ASSERT_TRUE(res.has_value());
  TEST_ASSERT_EQUAL(2, spi.transfer_count); // read then write

  // last_write = write header (0x80 | 0x08) then the 5 register bytes.
  // bits 17:16 = 0b10 => value 0x20000 => little-endian byte[2] = 0x02
  TEST_ASSERT_EQUAL(6, spi.last_write.size());
  TEST_ASSERT_EQUAL_HEX8(0x88, static_cast<uint8_t>(spi.last_write[0]));
  TEST_ASSERT_EQUAL_HEX8(0x00, static_cast<uint8_t>(spi.last_write[1]));
  TEST_ASSERT_EQUAL_HEX8(0x00, static_cast<uint8_t>(spi.last_write[2]));
  TEST_ASSERT_EQUAL_HEX8(0x02, static_cast<uint8_t>(spi.last_write[3]));
}
