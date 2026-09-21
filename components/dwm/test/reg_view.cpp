#include "dwm.h"
#include "esp32.h"
#include "system.h"
#include "unity.h"

namespace {
using HAL::operator""_p;

constexpr auto DWM_PINS = HAL::pins(HAL::NamedPin{"cs", GPIO_NUM_4},
                                    HAL::NamedPin{"reset", GPIO_NUM_27},
                                    HAL::NamedPin{"irq", GPIO_NUM_34});

// resetting the DW1000 without a DWM means driving its reset line directly,
// which needs an Assembly -- so it is a declared peripheral like anything else
struct HardReset {
  // only the reset line: the chip select belongs to the SPI device type
  static constexpr std::array claims{HAL::Claim{
      HAL::Resource::Gpio, DWM_PINS["reset"_p].number(), HAL::Use::Exclusive}};

  explicit HardReset(Swarm::Assembly assembly) {
    ESP32::GPIO gpio{assembly};
    constexpr HAL::Pin reset = DWM_PINS["reset"_p];

    gpio.set_direction(reset, HAL::PinMode::Output);
    gpio.set_level(reset, HAL::Voltage::LOW);
    gpio.delay_ms(10);

    // Release -- set to input, let internal pull-up take over
    gpio.set_direction(reset, HAL::PinMode::Input);
    gpio.delay_ms(10);
  }
};

// these tests drive the register view over a bare SPI, not a whole DW1000
using TestSystem = Swarm::system<ESP32::SpiBus, ESP32::SPI, HardReset>;
} // namespace

TEST_CASE("Test read device ID register view", "[dwm_reg]") {
  TestSystem::make<HardReset>();
  auto bus = TestSystem::make<ESP32::SpiBus>();
  auto spi = TestSystem::make<ESP32::SPI>(bus, DWM_PINS["cs"_p]);
  DWMRegisterView<ESP32::SPI, DWMRegisterID::DEV_ID> dev_id_reg{spi};

  TEST_ASSERT_EQUAL(dev_id_reg.size(), 4);

  auto id = dev_id_reg.read();
  TEST_ASSERT_TRUE(id.has_value());
  TEST_ASSERT_EQUAL(0xDECA0130, *id);
}

TEST_CASE("Test write and read-back TX buffer", "[dwm_reg]") {
  TestSystem::make<HardReset>();
  auto bus = TestSystem::make<ESP32::SpiBus>();
  auto spi = TestSystem::make<ESP32::SPI>(bus, DWM_PINS["cs"_p]);
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
