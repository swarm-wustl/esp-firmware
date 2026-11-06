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

enum class Access : uint8_t { ReadOnly, ReadWrite };

template <Reg R> struct Info;

template <> struct Info<Reg::SMPLRT_DIV> {
  static constexpr size_t size = 1;
  static constexpr Access access = Access::ReadWrite;
};
template <> struct Info<Reg::CONFIG> {
  static constexpr size_t size = 1;
  static constexpr Access access = Access::ReadWrite;
};
template <> struct Info<Reg::GYRO_CONFIG> {
  static constexpr size_t size = 1;
  static constexpr Access access = Access::ReadWrite;
};
template <> struct Info<Reg::ACCEL_CONFIG> {
  static constexpr size_t size = 1;
  static constexpr Access access = Access::ReadWrite;
};
// accel, temperature and gyro sit in one contiguous big-endian block, so a
// sample is a single 14-octet burst rather than six reads
template <> struct Info<Reg::ACCEL_XOUT_H> {
  static constexpr size_t size = 14;
  static constexpr Access access = Access::ReadOnly;
};
template <> struct Info<Reg::PWR_MGMT_1> {
  static constexpr size_t size = 1;
  static constexpr Access access = Access::ReadWrite;
};
template <> struct Info<Reg::WHO_AM_I> {
  static constexpr size_t size = 1;
  static constexpr Access access = Access::ReadOnly;
};

template <Reg R>
concept Writable = Info<R>::access == Access::ReadWrite;

struct Field {
  uint8_t width;
  uint8_t offset;

  constexpr uint8_t mask() const {
    return static_cast<uint8_t>(((1u << width) - 1u) << offset);
  }

  constexpr uint8_t encode(uint8_t value) const {
    return static_cast<uint8_t>((value << offset) & mask());
  }
};

inline constexpr Field FS_SEL{2, 3};
inline constexpr Field AFS_SEL{2, 3};
inline constexpr Field DLPF_CFG{3, 0};
inline constexpr Field CLKSEL{3, 0};
inline constexpr Field SLEEP{1, 6};
inline constexpr Field DEVICE_RESET{1, 7};

// PWR_MGMT_1 CLKSEL=1: the gyro X PLL, which the register map recommends over
// the internal 8 MHz oscillator for stability
inline constexpr uint8_t PLL_GYRO_X = 1;

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

// AD0 tied low or high. Nothing else on the part is address-selectable, so a
// second MPU6050 on the same bus is the only reason to pass High
enum class Address : uint8_t { Low = 0x68, High = 0x69 };

// WHO_AM_I reports the 7-bit address with bit 0 cleared, so a High part reads
// back 0x68 as well
inline constexpr uint8_t kWhoAmI = 0x68;

template <HAL::GenericI2CController I2C, HAL::GenericGPIOController GPIO,
          Config Cfg, typename Bus, Address Addr = Address::Low>
class Sensor {
public:
  static constexpr auto claims = Arrays::concat(I2C::claims, GPIO::claims);

  using needs = Swarm::needs<Bus>;

  static constexpr Config spec = Cfg;

  Sensor(Swarm::Assembly assembly, Bus &bus)
      : i2c_{assembly, bus, static_cast<uint8_t>(Addr)}, gpio_{assembly} {}

  Sensor(const Sensor &) = delete;
  void operator=(const Sensor &) = delete;
  Sensor(Sensor &&) = delete;
  void operator=(Sensor &&) = delete;

  [[nodiscard]] std::expected<uint8_t, HAL::I2cError> who_am_i() {
    return read<Reg::WHO_AM_I>().transform(
        [](const auto &bytes) { return static_cast<uint8_t>(bytes[0]); });
  }

  [[nodiscard]] std::expected<void, HAL::I2cError> configure() {
    // the part wakes into sleep with the internal oscillator selected, and a
    // reset needs ~100 ms before any register write sticks
    return write<Reg::PWR_MGMT_1>(DEVICE_RESET.encode(1))
        .and_then([this] {
          gpio_.delay_ms(100);

          return write<Reg::PWR_MGMT_1>(SLEEP.encode(0) |
                                        CLKSEL.encode(PLL_GYRO_X));
        })
        .and_then([this] {
          gpio_.delay_ms(10);

          return write<Reg::CONFIG>(
              DLPF_CFG.encode(static_cast<uint8_t>(Cfg.dlpf)));
        })
        .and_then([this] {
          return write<Reg::SMPLRT_DIV>(Cfg.sample_rate_divider);
        })
        .and_then([this] {
          return write<Reg::GYRO_CONFIG>(
              FS_SEL.encode(static_cast<uint8_t>(Cfg.gyro)));
        })
        .and_then([this] {
          return write<Reg::ACCEL_CONFIG>(
              AFS_SEL.encode(static_cast<uint8_t>(Cfg.accel)));
        });
  }

  [[nodiscard]] std::expected<Imu::Reading, HAL::I2cError> read_raw() {
    return read<Reg::ACCEL_XOUT_H>().transform(decode);
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

  template <Reg R>
  [[nodiscard]] std::expected<std::array<std::byte, Info<R>::size>,
                              HAL::I2cError>
  read() {
    const std::array tx{static_cast<std::byte>(R)};
    std::array<std::byte, Info<R>::size> rx{};

    return i2c_.write_read(tx, rx).transform([rx] { return rx; });
  }

  template <Reg R>
  [[nodiscard]] std::expected<void, HAL::I2cError> write(uint8_t value)
    requires Writable<R> && (Info<R>::size == 1)
  {
    const std::array tx{static_cast<std::byte>(R),
                        static_cast<std::byte>(value)};

    return i2c_.write(tx);
  }

  I2C i2c_;
  GPIO gpio_;
  Imu::Offsets offsets_{};
};
} // namespace MPU6050

#endif
