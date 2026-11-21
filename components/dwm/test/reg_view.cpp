#include "dwm.h"
#include "esp32.h"
#include "system.h"
#include "unity.h"

namespace {
constexpr auto DWM_PINS = HAL::pins(HAL::NamedPin{"cs", GPIO_NUM_4},
                                    HAL::NamedPin{"reset", GPIO_NUM_27},
                                    HAL::NamedPin{"irq", GPIO_NUM_34});

using Dwm = DWM<ESP32::SPI, ESP32::GPIO, DWM_PINS, ESP32::SpiBus>;
using TestSystem = Swarm::system<ESP32::SpiBus, Dwm>;
} // namespace

TEST_CASE("Test read device ID register", "[dwm_reg]") {
  auto bus = TestSystem::make<ESP32::SpiBus>();
  auto dwm = TestSystem::make<Dwm>(bus);

  static_assert(dw1000::Map::DEV_ID::bytes == 4);

  auto id = dwm.read<dw1000::Map::DEV_ID>();
  TEST_ASSERT_TRUE(id.has_value());
  TEST_ASSERT_EQUAL(0xDECA0130, id->word);
  TEST_ASSERT_EQUAL(0xDECA, id->get<dw1000::RIDTAG>());
}

TEST_CASE("Test write and read-back TX buffer", "[dwm_reg]") {
  auto bus = TestSystem::make<ESP32::SpiBus>();
  auto dwm = TestSystem::make<Dwm>(bus);

  std::array<std::array<std::byte, dw1000::Map::TX_BUFFER::bytes>, 2> test_data;
  std::array<std::byte, dw1000::Map::TX_BUFFER::bytes> readback{};

  // Generate random values in range [0, 255]
  std::ranges::generate(test_data[0], []() {
    return std::byte{static_cast<uint8_t>(std::rand() % 256)};
  });

  // Write data, read back the response, and compare
  TEST_ASSERT_TRUE(
      dwm.write<dw1000::Map::TX_BUFFER>(std::span{test_data[0]}).has_value());
  TEST_ASSERT_TRUE(dwm.read<dw1000::Map::TX_BUFFER>(std::span{readback}).has_value());
  TEST_ASSERT(std::ranges::equal(test_data[0], readback));

  // Re-generate random values in range [0, 255]
  std::ranges::generate(test_data[1], []() {
    return std::byte{static_cast<uint8_t>(std::rand() % 256)};
  });

  TEST_ASSERT_TRUE(
      dwm.write<dw1000::Map::TX_BUFFER>(std::span{test_data[1]}).has_value());

  // Read back from the device, still the new test data
  TEST_ASSERT_TRUE(dwm.read<dw1000::Map::TX_BUFFER>(std::span{readback}).has_value());
  TEST_ASSERT(std::ranges::equal(test_data[1], readback));
}
