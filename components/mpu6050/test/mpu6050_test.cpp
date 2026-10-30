#include "mock_i2c.h"
#include "mpu6050.h"
#include "system.h"
#include "unity.h"

#include <array>
#include <cstddef>

namespace {
constexpr auto kConfig =
    MPU6050::config({.gyro = Imu::GyroRange::Dps500,
                     .accel = Imu::AccelRange::G4,
                     .dlpf = MPU6050::Dlpf::Hz44,
                     .sample_rate_divider = 9});

using Sensor = MPU6050::Sensor<MockI2C, MockGPIO, kConfig, MockBus>;
using Sys = Swarm::system<MockBus, Sensor>;

static_assert(HAL::GenericI2CController<MockI2C>);
static_assert(kConfig.sample_rate_hz() == 100);

// Swarm::system::take() is once-per-type, and Sensor is immovable, so every
// case shares the one instance and resets the transcript instead
struct Fixture {
  Sensor &sensor;
  Transcript &transcript;

  static Fixture get() {
    static Sys::Instances &instances = Sys::take();

    Fixture fixture{instances.get<Sensor>(),
                    instances.get<MockBus>().transcript};
    fixture.transcript.clear();
    fixture.sensor.set_offsets({});

    return fixture;
  }
};

std::array<std::byte, 14> burst(int16_t ax, int16_t ay, int16_t az,
                                int16_t temp, int16_t gx, int16_t gy,
                                int16_t gz) {
  const std::array<int16_t, 7> words{ax, ay, az, temp, gx, gy, gz};
  std::array<std::byte, 14> out{};

  for (size_t i = 0; i < words.size(); ++i) {
    out[2 * i] = static_cast<std::byte>(static_cast<uint16_t>(words[i]) >> 8);
    out[2 * i + 1] = static_cast<std::byte>(words[i] & 0xFF);
  }

  return out;
}

void respond_with(Transcript &transcript, const std::array<std::byte, 14> &b) {
  transcript.read_response.assign(b.begin(), b.end());
}

void check_write(const std::vector<std::byte> &write, MPU6050::Reg reg,
                 uint8_t value) {
  TEST_ASSERT_EQUAL_UINT32(2, write.size());
  TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(reg),
                         static_cast<uint8_t>(write[0]));
  TEST_ASSERT_EQUAL_HEX8(value, static_cast<uint8_t>(write[1]));
}
} // namespace

TEST_CASE("configure: the ranges and filter reach their registers",
          "[mpu6050]") {
  Fixture fixture = Fixture::get();

  TEST_ASSERT_TRUE(fixture.sensor.configure().has_value());

  const auto &writes = fixture.transcript.writes;
  TEST_ASSERT_EQUAL_UINT32(6, writes.size());

  check_write(writes[0], MPU6050::Reg::PWR_MGMT_1, 0x80);
  check_write(writes[1], MPU6050::Reg::PWR_MGMT_1, MPU6050::PLL_GYRO_X);
  check_write(writes[2], MPU6050::Reg::CONFIG,
              static_cast<uint8_t>(MPU6050::Dlpf::Hz44));
  check_write(writes[3], MPU6050::Reg::SMPLRT_DIV, 9);
  check_write(writes[4], MPU6050::Reg::GYRO_CONFIG, 0x08);
  check_write(writes[5], MPU6050::Reg::ACCEL_CONFIG, 0x08);
}

TEST_CASE("configure: a failed write stops the sequence", "[mpu6050]") {
  Fixture fixture = Fixture::get();
  fixture.transcript.fail_with = HAL::I2cError::Nack;

  const auto result = fixture.sensor.configure();

  TEST_ASSERT_FALSE(result.has_value());
  TEST_ASSERT_EQUAL_INT(static_cast<int>(HAL::I2cError::Nack),
                        static_cast<int>(result.error()));
  TEST_ASSERT_EQUAL_UINT32(0, fixture.transcript.writes.size());
}

TEST_CASE("read_raw: one burst becomes big-endian accel, temp and gyro",
          "[mpu6050]") {
  Fixture fixture = Fixture::get();
  respond_with(fixture.transcript, burst(-1, 2, 8192, 340, 6554, -6554, 0));

  const auto reading = fixture.sensor.read_raw();
  TEST_ASSERT_TRUE(reading.has_value());

  TEST_ASSERT_EQUAL_INT16(-1, reading->accel.x);
  TEST_ASSERT_EQUAL_INT16(2, reading->accel.y);
  TEST_ASSERT_EQUAL_INT16(8192, reading->accel.z);
  TEST_ASSERT_EQUAL_INT16(340, reading->temperature);
  TEST_ASSERT_EQUAL_INT16(6554, reading->gyro.x);
  TEST_ASSERT_EQUAL_INT16(-6554, reading->gyro.y);
  TEST_ASSERT_EQUAL_INT16(0, reading->gyro.z);

  TEST_ASSERT_EQUAL_UINT32(1, fixture.transcript.reads);
  TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(MPU6050::Reg::ACCEL_XOUT_H),
                         static_cast<uint8_t>(fixture.transcript.writes[0][0]));
}

TEST_CASE("read_sample: counts become g, dps and degrees", "[mpu6050]") {
  Fixture fixture = Fixture::get();
  respond_with(fixture.transcript, burst(0, 0, 8192, 0, 6554, 0, 0));

  const auto sample = fixture.sensor.read_sample();
  TEST_ASSERT_TRUE(sample.has_value());

  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, sample->accel.z);
  TEST_ASSERT_DOUBLE_WITHIN(1e-2, 100.0, sample->gyro.x);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 36.53, sample->temperature_c);
}

TEST_CASE("calibrate: the bias it finds zeroes the next sample but keeps 1 g",
          "[mpu6050]") {
  Fixture fixture = Fixture::get();
  respond_with(fixture.transcript, burst(120, -40, 8292, 0, 17, -9, 3));

  const auto offsets = fixture.sensor.calibrate<4>();
  TEST_ASSERT_TRUE(offsets.has_value());

  TEST_ASSERT_EQUAL_INT16(120, offsets->accel.x);
  TEST_ASSERT_EQUAL_INT16(-40, offsets->accel.y);
  TEST_ASSERT_EQUAL_INT16(100, offsets->accel.z);
  TEST_ASSERT_EQUAL_INT16(17, offsets->gyro.x);
  TEST_ASSERT_EQUAL_INT16(-9, offsets->gyro.y);
  TEST_ASSERT_EQUAL_INT16(3, offsets->gyro.z);

  const auto sample = fixture.sensor.read_sample();
  TEST_ASSERT_TRUE(sample.has_value());

  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, sample->accel.x);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, sample->accel.y);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, sample->accel.z);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, sample->gyro.x);
}

TEST_CASE("who_am_i: the identity register reads back one byte", "[mpu6050]") {
  Fixture fixture = Fixture::get();
  fixture.transcript.read_response = {std::byte{MPU6050::kWhoAmI}};

  const auto id = fixture.sensor.who_am_i();

  TEST_ASSERT_TRUE(id.has_value());
  TEST_ASSERT_EQUAL_HEX8(MPU6050::kWhoAmI, *id);
}
