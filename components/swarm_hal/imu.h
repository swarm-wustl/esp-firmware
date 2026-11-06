#ifndef IMU_H
#define IMU_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <utility>

namespace Imu {
struct Vec3 {
  double x{};
  double y{};
  double z{};

  constexpr bool operator==(const Vec3 &) const = default;
};

struct Axes {
  int16_t x{};
  int16_t y{};
  int16_t z{};

  constexpr bool operator==(const Axes &) const = default;
};

enum class Axis : uint8_t { X, Y, Z };

struct Reading {
  Axes accel;
  Axes gyro;
  int16_t temperature;
};

struct Offsets {
  Axes accel{};
  Axes gyro{};
};

struct Sample {
  Vec3 accel;
  Vec3 gyro;
  double temperature_c{};
};

enum class GyroRange : uint8_t { Dps250, Dps500, Dps1000, Dps2000 };
enum class AccelRange : uint8_t { G2, G4, G8, G16 };

constexpr double full_scale(GyroRange range) {
  return 250.0 * static_cast<double>(1u << static_cast<uint8_t>(range));
}

constexpr double full_scale(AccelRange range) {
  return 2.0 * static_cast<double>(1u << static_cast<uint8_t>(range));
}

// a measurement is 16-bit two's complement spread over the selected full
// scale, so sensitivity is a consequence of the range and never declared
inline constexpr double kHalfScaleCounts = 32768.0;

constexpr double lsb_per_dps(GyroRange range) {
  return kHalfScaleCounts / full_scale(range);
}

constexpr double lsb_per_g(AccelRange range) {
  return kHalfScaleCounts / full_scale(range);
}

template <typename V> constexpr auto component(const V &v, Axis axis) {
  switch (axis) {
  case Axis::X:
    return v.x;
  case Axis::Y:
    return v.y;
  case Axis::Z:
    return v.z;
  }

  std::unreachable();
}

constexpr Vec3 scaled(const Axes &counts, const Axes &offset, double lsb) {
  return {(counts.x - offset.x) / lsb, (counts.y - offset.y) / lsb,
          (counts.z - offset.z) / lsb};
}

// MPU-6000 register map 4.18: 340 LSB/degC about a 36.53 degC intercept
constexpr double celsius(int16_t raw) {
  return static_cast<double>(raw) / 340.0 + 36.53;
}

constexpr Sample convert(const Reading &reading, const Offsets &offsets,
                         GyroRange gyro, AccelRange accel) {
  return {scaled(reading.accel, offsets.accel, lsb_per_g(accel)),
          scaled(reading.gyro, offsets.gyro, lsb_per_dps(gyro)),
          celsius(reading.temperature)};
}

constexpr double yaw_rate(const Sample &sample, Axis up) {
  return component(sample.gyro, up) * std::numbers::pi / 180.0;
}

constexpr int16_t saturate(int32_t value) {
  return static_cast<int16_t>(value < INT16_MIN   ? INT16_MIN
                              : value > INT16_MAX ? INT16_MAX
                                                  : value);
}

// a bias estimate is the mean of readings taken at rest. Gravity is a real
// acceleration, not an offset, so the axis pointing up keeps its 1 g
class Accumulator {
public:
  constexpr void add(const Reading &reading) {
    accel_[0] += reading.accel.x;
    accel_[1] += reading.accel.y;
    accel_[2] += reading.accel.z;

    gyro_[0] += reading.gyro.x;
    gyro_[1] += reading.gyro.y;
    gyro_[2] += reading.gyro.z;

    ++count_;
  }

  constexpr size_t count() const { return count_; }

  constexpr Offsets offsets(AccelRange range, Axis up = Axis::Z) const {
    if (count_ == 0) {
      return {};
    }

    const int32_t gravity = static_cast<int32_t>(lsb_per_g(range));

    return {Axes{saturate(mean(accel_[0]) -
                          (up == Axis::X ? gravity : 0)),
                 saturate(mean(accel_[1]) - (up == Axis::Y ? gravity : 0)),
                 saturate(mean(accel_[2]) - (up == Axis::Z ? gravity : 0))},
            Axes{saturate(mean(gyro_[0])), saturate(mean(gyro_[1])),
                 saturate(mean(gyro_[2]))}};
  }

private:
  constexpr int32_t mean(int32_t sum) const {
    return sum / static_cast<int32_t>(count_);
  }

  std::array<int32_t, 3> accel_{};
  std::array<int32_t, 3> gyro_{};
  size_t count_{};
};

constexpr int16_t be16(std::byte high, std::byte low) {
  return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) |
                              static_cast<uint16_t>(low));
}
} // namespace Imu

#endif
