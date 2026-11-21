#ifndef DWM_MOCK_SPI_H
#define DWM_MOCK_SPI_H

#include <array>
#include <cstddef>
#include <expected>
#include <optional>
#include <span>
#include <vector>

#include "assembly.h"
#include "peripheral_hal.h"
#include "pins.h"
#include "resources.h"

// the mpu6050 tests link into the same binary with their own Transcript and
// MockGPIO, so these stay out of the global namespace
namespace DwmMock {
struct Transfer {
  std::vector<std::byte> tx;
  size_t rx;
};

struct Transcript {
  std::vector<Transfer> transfers;
  std::vector<std::byte> read_response;
  std::optional<HAL::SpiError> fail_with;

  void clear() {
    transfers.clear();
    read_response.clear();
    fail_with.reset();
  }

  std::vector<std::vector<std::byte>> writes() const {
    std::vector<std::vector<std::byte>> out;

    for (const Transfer &t : transfers) {
      if (t.rx == 0) {
        out.push_back(t.tx);
      }
    }

    return out;
  }
};

struct MockSpiBus {
  static constexpr std::array<HAL::Claim, 0> claims{};

  explicit MockSpiBus(Swarm::Assembly<MockSpiBus>) {}

  MockSpiBus(const MockSpiBus &) = delete;
  void operator=(const MockSpiBus &) = delete;

  Transcript transcript;
};

class MockSPI {
public:
  static constexpr std::array<HAL::Claim, 0> claims{};

  MockSPI(Swarm::Assembly<MockSPI>, MockSpiBus &bus, HAL::Pin)
      : transcript_{&bus.transcript} {}

  std::expected<void, HAL::SpiError>
  transfer_halfduplex(std::span<const std::byte> tx, std::span<std::byte> rx) {
    transcript_->transfers.push_back({{tx.begin(), tx.end()}, rx.size()});

    if (transcript_->fail_with) {
      return std::unexpected(*transcript_->fail_with);
    }

    for (size_t i = 0; i < rx.size(); ++i) {
      rx[i] = i < transcript_->read_response.size()
                  ? transcript_->read_response[i]
                  : std::byte{0};
    }

    return {};
  }

private:
  Transcript *transcript_;
};

class MockGPIO {
public:
  static constexpr std::array<HAL::Claim, 0> claims{};

  explicit MockGPIO(Swarm::Assembly<MockGPIO>) {}

  void set_direction(HAL::Pin, HAL::PinMode) {}
  void set_level(HAL::Pin, HAL::Voltage) {}
  void delay_ms(int) {}
};
} // namespace DwmMock

#endif
