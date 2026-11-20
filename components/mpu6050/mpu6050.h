#ifndef MPU6050_H
#define MPU6050_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>
#include <utility>

#include "arrays.h"
#include "assembly.h"
#include "imu.h"
#include "peripheral_hal.h"
#include "registers.h"
#include "resources.h"
#include "system.h"

// register numbers, layouts and conversions come from the MPU-6000/6050
// register map (RM-MPU-6000A-00) and datasheet (PS-MPU-6000A-00):
// https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
// https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
namespace MPU6050 {
enum class Reg : uint8_t {
  SMPLRT_DIV = 0x19,
  CONFIG = 0x1A,
  GYRO_CONFIG = 0x1B,
  ACCEL_CONFIG = 0x1C,
  ACCEL_XOUT_H = 0x3B,
  PWR_MGMT_1 = 0x6B,
  WHO_AM_I = 0x75,
};

// PWR_MGMT_1 CLKSEL=1: the gyro X PLL, which the register map recommends over
// the internal 8 MHz oscillator for stability
enum class Clock : uint8_t { Internal = 0, PllGyroX = 1 };

// the on-chip low-pass. 0 is the widest band and the 8 kHz gyro rate; 3 is
// 42 Hz, which is where a chassis-mounted part stops reporting frame buzz
enum class Dlpf : uint8_t {
  Hz260 = 0,
  Hz184 = 1,
  Hz94 = 2,
  Hz44 = 3,
  Hz21 = 4,
  Hz10 = 5,
  Hz5 = 6,
};

// the sample rate is the gyro output rate over (1 + divider), and DLPF_CFG
// picks that rate: 8 kHz undivided at Hz260, 1 kHz otherwise
struct Config {
  Imu::GyroRange gyro = Imu::GyroRange::Dps250;
  Imu::AccelRange accel = Imu::AccelRange::G2;
  Dlpf dlpf = Dlpf::Hz44;
  uint8_t sample_rate_divider = 9;
  Imu::Axis up = Imu::Axis::Z;

  constexpr uint32_t sample_rate_hz() const {
    const uint32_t gyro_rate = dlpf == Dlpf::Hz260 ? 8000 : 1000;

    return gyro_rate / (1u + sample_rate_divider);
  }
};

consteval Config config(Config value) {
  if (value.dlpf > Dlpf::Hz5) {
    std::unreachable();
  }

  return value;
}

using SMPLRT_DIV = Registers::field<"SMPLRT_DIV", uint8_t, 7, 0>;
using DLPF_CFG = Registers::field<"DLPF_CFG", Dlpf, 2, 0>;
using FS_SEL = Registers::field<"FS_SEL", Imu::GyroRange, 4, 3>;
using AFS_SEL = Registers::field<"AFS_SEL", Imu::AccelRange, 4, 3>;
using DEVICE_RESET = Registers::field<"DEVICE_RESET", bool, 7, 7>;
using SLEEP = Registers::field<"SLEEP", bool, 6, 6>;
using CLKSEL = Registers::field<"CLKSEL", Clock, 2, 0>;
using WHO_AM_I = Registers::field<"WHO_AM_I", uint8_t, 7, 0>;

namespace Map {
using Registers::Access;

using SMPLRT_DIV = Registers::reg<Reg::SMPLRT_DIV, {}, MPU6050::SMPLRT_DIV>;
using CONFIG = Registers::reg<Reg::CONFIG, {}, DLPF_CFG>;
using GYRO_CONFIG = Registers::reg<Reg::GYRO_CONFIG, {}, FS_SEL>;
using ACCEL_CONFIG = Registers::reg<Reg::ACCEL_CONFIG, {}, AFS_SEL>;
// accel, temperature and gyro sit in one contiguous big-endian block, so a
// sample is a single 14-octet burst rather than six reads
using ACCEL_XOUT_H = Registers::reg<
    Reg::ACCEL_XOUT_H, {.bytes = 14, .access = Access::ReadOnly}>;
// the part powers up asleep
using PWR_MGMT_1 = Registers::reg<Reg::PWR_MGMT_1, {.reset = 0x40},
                                  DEVICE_RESET, SLEEP, CLKSEL>;
using WHO_AM_I = Registers::reg<Reg::WHO_AM_I,
                                {.access = Access::ReadOnly, .reset = 0x68},
                                MPU6050::WHO_AM_I>;
} // namespace Map

// AD0 tied low or high. Nothing else on the part is address-selectable, so a
// second MPU6050 on the same bus is the only reason to pass High
enum class Address : uint8_t { Low = 0x68, High = 0x69 };

// WHO_AM_I reports the 7-bit address with bit 0 cleared, so a High part reads
// back 0x68 as well
inline constexpr uint8_t kWhoAmI = Map::WHO_AM_I::reset;

template <HAL::GenericI2CController I2C, HAL::GenericGPIOController GPIO,
          Config Cfg, typename Bus, Address Addr = Address::Low>
class Sensor {
public:
  static constexpr std::array<HAL::Claim, 0> claims{};

  using owns = Swarm::owns<I2C, GPIO>;
  using needs = Swarm::needs<Bus>;

  static constexpr Config spec = Cfg;

  Sensor(Swarm::Assembly<Sensor> key, Bus &bus)
      : i2c_{Swarm::make<I2C>(key, bus, static_cast<uint8_t>(Addr))},
        gpio_{Swarm::make<GPIO>(key)} {}

  Sensor(const Sensor &) = delete;
  void operator=(const Sensor &) = delete;
  Sensor(Sensor &&) = delete;
  void operator=(Sensor &&) = delete;

  [[nodiscard]] std::expected<uint8_t, HAL::I2cError> who_am_i() {
    return read<Map::WHO_AM_I>().transform(
        [](const auto &id) { return id.template get<WHO_AM_I>(); });
  }

  [[nodiscard]] std::expected<void, HAL::I2cError> configure() {
    // the part wakes into sleep with the internal oscillator selected, and a
    // reset needs ~100 ms before any register write sticks
    return write<Map::PWR_MGMT_1>(DEVICE_RESET{true})
        .and_then([this] {
          gpio_.delay_ms(100);

          return write<Map::PWR_MGMT_1>(SLEEP{false},
                                        CLKSEL{Clock::PllGyroX});
        })
        .and_then([this] {
          gpio_.delay_ms(10);

          return write<Map::CONFIG>(DLPF_CFG{Cfg.dlpf});
        })
        .and_then([this] {
          return write<Map::SMPLRT_DIV>(SMPLRT_DIV{Cfg.sample_rate_divider});
        })
        .and_then([this] { return write<Map::GYRO_CONFIG>(FS_SEL{Cfg.gyro}); })
        .and_then(
            [this] { return write<Map::ACCEL_CONFIG>(AFS_SEL{Cfg.accel}); });
  }

  [[nodiscard]] std::expected<Imu::Reading, HAL::I2cError> read_raw() {
    return read<Map::ACCEL_XOUT_H>().transform(decode);
  }

  [[nodiscard]] std::expected<Imu::Sample, HAL::I2cError> read_sample() {
    return read_raw().transform([this](const Imu::Reading &reading) {
      return Imu::convert(reading, offsets_, Cfg.gyro, Cfg.accel);
    });
  }

  // the mean of N readings taken at rest, which the samples above are then
  // corrected by. The caller holds the robot still; nothing here can check
  // that it did
  template <size_t N = 100>
  [[nodiscard]] std::expected<Imu::Offsets, HAL::I2cError> calibrate() {
    Imu::Accumulator accumulator;

    for (size_t i = 0; i < N; ++i) {
      const auto reading = read_raw();

      if (!reading) {
        return std::unexpected(reading.error());
      }

      accumulator.add(*reading);

      gpio_.delay_ms(period_ms);
    }

    offsets_ = accumulator.offsets(Cfg.accel, Cfg.up);

    return offsets_;
  }

  const Imu::Offsets &offsets() const { return offsets_; }
  void set_offsets(const Imu::Offsets &offsets) { offsets_ = offsets; }

private:
  // one sample interval, so a calibration pass reads each sample once rather
  // than the same one twice
  static constexpr int period_ms =
      static_cast<int>(1000 / Cfg.sample_rate_hz()) + 1;

  static constexpr Imu::Reading decode(const std::array<std::byte, 14> &raw) {
    return {Imu::Axes{Imu::be16(raw[0], raw[1]), Imu::be16(raw[2], raw[3]),
                      Imu::be16(raw[4], raw[5])},
            Imu::Axes{Imu::be16(raw[8], raw[9]), Imu::be16(raw[10], raw[11]),
                      Imu::be16(raw[12], raw[13])},
            Imu::be16(raw[6], raw[7])};
  }

  template <Registers::at<Reg> R> [[nodiscard]] auto read() {
    const std::array tx{static_cast<std::byte>(R::address)};
    std::array<std::byte, R::bytes> rx{};

    return i2c_.write_read(tx, rx).transform(
        [rx] { return Registers::decode<R>(rx); });
  }

  template <Registers::at<Reg> R, Registers::Field... Fs>
    requires Registers::writable_with<R, Fs...>
  [[nodiscard]] std::expected<void, HAL::I2cError> write(Fs... fs) {
    return i2c_.write(Registers::frame<R>(
        std::array{static_cast<std::byte>(R::address)}, fs...));
  }

  I2C i2c_;
  GPIO gpio_;
  Imu::Offsets offsets_{};
};
} // namespace MPU6050

#endif
