#ifndef MPU6050_MOCK_I2C_H
#define MPU6050_MOCK_I2C_H

#include <array>
#include <cstddef>
#include <expected>
#include <optional>
#include <span>
#include <vector>

#include "assembly.h"
#include "peripheral_hal.h"
#include "resources.h"

struct Transcript {
  std::vector<std::vector<std::byte>> writes;
  std::vector<std::byte> read_response;
  std::optional<HAL::I2cError> fail_with;
  int reads = 0;

  void clear() {
    writes.clear();
    read_response.clear();
    fail_with.reset();
    reads = 0;
  }
};

struct MockBus {
  static constexpr std::array<HAL::Claim, 0> claims{};

  explicit MockBus(Swarm::Assembly) {}

  MockBus(const MockBus &) = delete;
  void operator=(const MockBus &) = delete;

  Transcript transcript;
};

class MockI2C {
public:
  static constexpr std::array<HAL::Claim, 0> claims{};

  MockI2C(Swarm::Assembly, MockBus &bus, uint8_t address)
      : transcript_{&bus.transcript}, address_{address} {}

  std::expected<void, HAL::I2cError> write(std::span<const std::byte> tx) {
    if (transcript_->fail_with) {
      return std::unexpected(*transcript_->fail_with);
    }

    transcript_->writes.emplace_back(tx.begin(), tx.end());

    return {};
  }

  std::expected<void, HAL::I2cError> write_read(std::span<const std::byte> tx,
                                                std::span<std::byte> rx) {
    ++transcript_->reads;

    if (transcript_->fail_with) {
      return std::unexpected(*transcript_->fail_with);
    }

    transcript_->writes.emplace_back(tx.begin(), tx.end());

    for (size_t i = 0; i < rx.size(); ++i) {
      rx[i] = i < transcript_->read_response.size()
                  ? transcript_->read_response[i]
                  : std::byte{0};
    }

    return {};
  }

  uint8_t address() const { return address_; }

private:
  Transcript *transcript_;
  uint8_t address_;
};

class MockGPIO {
public:
  static constexpr std::array<HAL::Claim, 0> claims{};

  explicit MockGPIO(Swarm::Assembly) {}

  void set_direction(HAL::Pin, HAL::PinMode) {}
  void set_level(HAL::Pin, HAL::Voltage) {}
  void delay_ms(int) {}
};

#endif
