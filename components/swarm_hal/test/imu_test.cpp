#include "imu.h"
#include "unity.h"

#include <type_traits>

namespace {
static_assert(std::is_trivially_copyable_v<Imu::Sample>);

static_assert(Imu::lsb_per_dps(Imu::GyroRange::Dps250) == 131.072);
static_assert(Imu::lsb_per_dps(Imu::GyroRange::Dps2000) == 16.384);
static_assert(Imu::lsb_per_g(Imu::AccelRange::G2) == 16384.0);
static_assert(Imu::lsb_per_g(Imu::AccelRange::G16) == 2048.0);

constexpr Imu::Reading kAtRest{Imu::Axes{10, -20, 16434}, Imu::Axes{5, -5, 1},
                               0};
} // namespace

TEST_CASE("full scale doubles with each range step", "[imu]") {
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 500.0,
                            Imu::full_scale(Imu::GyroRange::Dps500));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 8.0, Imu::full_scale(Imu::AccelRange::G8));
}

TEST_CASE("be16: two big-endian bytes make a signed count", "[imu]") {
  TEST_ASSERT_EQUAL_INT16(-1, Imu::be16(std::byte{0xFF}, std::byte{0xFF}));
  TEST_ASSERT_EQUAL_INT16(256, Imu::be16(std::byte{0x01}, std::byte{0x00}));
  TEST_ASSERT_EQUAL_INT16(-32768, Imu::be16(std::byte{0x80}, std::byte{0x00}));
}

TEST_CASE("convert: counts scale by the selected range", "[imu]") {
  const Imu::Sample sample =
      Imu::convert(Imu::Reading{Imu::Axes{0, 0, 16384}, Imu::Axes{131, 0, 0},
                                340},
                   Imu::Offsets{}, Imu::GyroRange::Dps250,
                   Imu::AccelRange::G2);

  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, sample.accel.z);
  TEST_ASSERT_DOUBLE_WITHIN(1e-2, 1.0, sample.gyro.x);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 37.53, sample.temperature_c);
}

TEST_CASE("convert: an offset shifts the reading it was measured from",
          "[imu]") {
  Imu::Accumulator accumulator;

  for (int i = 0; i < 8; ++i) {
    accumulator.add(kAtRest);
  }

  const Imu::Offsets offsets = accumulator.offsets(Imu::AccelRange::G2);
  const Imu::Sample sample = Imu::convert(kAtRest, offsets,
                                          Imu::GyroRange::Dps250,
                                          Imu::AccelRange::G2);

  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, sample.accel.x);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, sample.accel.y);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, sample.accel.z);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, sample.gyro.x);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, sample.gyro.z);
}

TEST_CASE("accumulator: gravity stays on whichever axis points up", "[imu]") {
  Imu::Accumulator accumulator;
  accumulator.add(Imu::Reading{Imu::Axes{16384, 0, 0}, Imu::Axes{}, 0});

  const Imu::Offsets on_x = accumulator.offsets(Imu::AccelRange::G2,
                                                Imu::Axis::X);

  TEST_ASSERT_EQUAL_INT16(0, on_x.accel.x);

  const Imu::Offsets on_z = accumulator.offsets(Imu::AccelRange::G2);

  TEST_ASSERT_EQUAL_INT16(16384, on_z.accel.x);
  TEST_ASSERT_EQUAL_INT16(-16384, on_z.accel.z);
}

TEST_CASE("accumulator: no samples means no offsets", "[imu]") {
  const Imu::Accumulator accumulator;

  TEST_ASSERT_EQUAL_UINT32(0, accumulator.count());
  TEST_ASSERT_EQUAL_INT16(0, accumulator.offsets(Imu::AccelRange::G2).accel.z);
}

TEST_CASE("imu: yaw rate is the up axis in rad/s", "[imu]") {
  const Imu::Sample sample{.gyro = {.x = 180.0, .y = 360.0, .z = 90.0}};

  TEST_ASSERT_DOUBLE_WITHIN(1e-9, std::numbers::pi / 2.0,
                            Imu::yaw_rate(sample, Imu::Axis::Z));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 2.0 * std::numbers::pi,
                            Imu::yaw_rate(sample, Imu::Axis::Y));
}
