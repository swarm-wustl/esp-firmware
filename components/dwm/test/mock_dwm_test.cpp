#include "dwm.h"
#include "mock_spi.h"
#include "peripheral_hal.h"
#include "system.h"
#include "unity.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>
#include <initializer_list>
#include <tuple>
#include <utility>
#include <vector>

namespace {
using namespace DwmMock;
namespace Map = dw1000::Map;

constexpr auto kPins = HAL::pins(HAL::NamedPin{"cs", 15},
                                 HAL::NamedPin{"reset", 21},
                                 HAL::NamedPin{"irq", 13});

using Dwm = DWM<MockSPI, MockGPIO, kPins, MockSpiBus>;
using Sys = Swarm::system<MockSpiBus, Dwm>;

static_assert(HAL::GenericSPIController<MockSPI>);

// Swarm::system::take() is once-per-type, and DWM is immovable, so every case
// shares the one instance and resets the transcript instead
struct Fixture {
  Dwm &dwm;
  Transcript &transcript;

  static Fixture get() {
    static Sys::Instances &instances = Sys::take();

    Fixture fixture{instances.get<Dwm>(), instances.get<MockSpiBus>().transcript};
    fixture.transcript.clear();

    return fixture;
  }
};

std::vector<std::byte> bytes(std::initializer_list<int> values) {
  std::vector<std::byte> out;

  for (const int v : values) {
    out.push_back(static_cast<std::byte>(v));
  }

  return out;
}

void check_bytes(const std::vector<std::byte> &expected,
                 const std::vector<std::byte> &actual) {
  TEST_ASSERT_EQUAL_UINT32(expected.size(), actual.size());

  for (size_t i = 0; i < expected.size(); ++i) {
    TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(expected[i]),
                           static_cast<uint8_t>(actual[i]));
  }
}

template <typename R, typename... Fs>
concept writes = requires(Dwm &d, Fs... fs) { d.template write<R>(fs...); };

template <typename R, typename... Fs>
concept modifies = requires(Dwm &d, Fs... fs) { d.template modify<R>(fs...); };

template <typename R, typename Byte, size_t N>
concept writes_bytes =
    requires(Dwm &d, std::span<Byte, N> data) { d.template write<R>(data); };

template <typename R, size_t N>
concept reads_bytes =
    requires(Dwm &d, std::span<std::byte, N> out) { d.template read<R>(out); };

template <size_t N>
concept transmits =
    requires(Dwm &d, const std::array<std::byte, N> &p) { d.transmit(p); };

namespace Other {
enum class Address : uint8_t {};
using SLEEP = Registers::field<"SLEEP", bool, 6, 6>;
using PWR = Registers::reg<Address{0x6B}, {.reset = 0x40}, SLEEP>;
} // namespace Other
} // namespace

TEST_CASE("dw1000: the SPI header carries the sub-address in 1 to 3 octets",
          "[dwm_mock]") {
  constexpr auto plain = dw1000::header<dw1000::Address{0x0F}>(true);
  constexpr auto short_sub = dw1000::header<dw1000::Address{0x2D, 0x06}>(true);
  constexpr auto long_sub =
      dw1000::header<dw1000::Address{0x2E, 0x1804}>(false);

  static_assert(plain.size() == 1 && plain[0] == std::byte{0x8F});
  static_assert(short_sub.size() == 2 && short_sub[0] == std::byte{0xED} &&
                short_sub[1] == std::byte{0x06});
  static_assert(long_sub.size() == 3 && long_sub[0] == std::byte{0x6E} &&
                long_sub[1] == std::byte{0x84} &&
                long_sub[2] == std::byte{0x30});
  TEST_ASSERT_TRUE(true);
}

TEST_CASE("dw1000: the map encodes what the hand-computed constants did",
          "[dwm_mock]") {
  // CHAN_CTRL_VALUE, LDE load steps L-1, L-2 and L-3 (manual Table 4)
  static_assert(std::get<16>(dw1000::DEFAULT_CONFIG).word == 0x21040055);
  static_assert(Registers::write_word<Map::PMSC_CTRL0>(
                    dw1000::SYSCLKS{dw1000::SysClock::Xti},
                    dw1000::LDE_LOAD_CLKS{0b11}) == 0x0301);
  static_assert(Registers::write_word<Map::OTP_CTRL>(dw1000::LDELOAD{true}) ==
                0x8000);
  static_assert(Registers::write_word<Map::PMSC_CTRL0>() == 0x0200);
  TEST_ASSERT_TRUE(true);
}

TEST_CASE("dw1000: SYS_CFG keeps its reset bits beside the filter's",
          "[dwm_mock]") {
  // HIRQ_POL and DIS_DRXB are what the part resets to (manual 7.2.6)
  static_assert(Registers::write_word<Map::SYS_CFG>() == 0x00001200);
  static_assert(Registers::write_word<Map::SYS_CFG>(dw1000::FFEN{true},
                                                    dw1000::FFAD{true},
                                                    dw1000::RXAUTR{true}) ==
                0x20001209);
  static_assert(Registers::write_word<Map::SYS_STATUS>(dw1000::AFFREJ{true}) ==
                uint64_t{1} << 29);
  TEST_ASSERT_TRUE(true);
}

TEST_CASE("set_identity: PANADR is the short address, then the PAN",
          "[dwm_mock]") {
  Fixture fixture = Fixture::get();

  TEST_ASSERT_TRUE(fixture.dwm
                       .set_identity({.pan = dw1000::PanId{0xDECA},
                                      .address = dw1000::ShortAddress{0x0A01}})
                       .has_value());

  const auto writes = fixture.transcript.writes();
  TEST_ASSERT_EQUAL_UINT32(1, writes.size());
  check_bytes(bytes({0x83, 0x01, 0x0A, 0xCA, 0xDE}), writes[0]);
}

TEST_CASE("dw1000: illegal register operations don't compile", "[dwm_mock]") {
  static_assert(writes<Map::SYS_STATUS, dw1000::TXFRS>);
  static_assert(!writes<Map::SYS_STATUS, dw1000::IRQS>);
  static_assert(!modifies<Map::SYS_STATUS, dw1000::TXFRS>);
  static_assert(!writes<Map::DEV_ID>);
  static_assert(!writes<Map::SYS_CTRL, dw1000::TXFRS>);
  static_assert(modifies<Map::TX_FCTRL, dw1000::TXPRF>);
  static_assert(!writes<Other::PWR, Other::SLEEP>);
  TEST_ASSERT_TRUE(true);
}

TEST_CASE("dw1000: a block's length is checked where it's written",
          "[dwm_mock]") {
  static_assert(writes_bytes<Map::TX_BUFFER, const std::byte, 1024>);
  static_assert(writes_bytes<Map::TX_BUFFER, std::byte, 9>);
  static_assert(!writes_bytes<Map::TX_BUFFER, const std::byte, 1025>);
  static_assert(!writes_bytes<Map::TX_BUFFER, const std::byte,
                              std::dynamic_extent>);
  static_assert(!writes_bytes<Map::RX_BUFFER, const std::byte, 9>);
  static_assert(!writes_bytes<Map::SYS_CTRL, const std::byte, 4>);

  static_assert(reads_bytes<Map::RX_BUFFER, 1024>);
  static_assert(!reads_bytes<Map::RX_BUFFER, 1025>);
  static_assert(!reads_bytes<Map::RX_BUFFER, std::dynamic_extent>);

  // TFLEN is 7 bits and counts the 2-octet FCS
  static_assert(transmits<125>);
  static_assert(!transmits<126>);
  TEST_ASSERT_TRUE(true);
}

TEST_CASE("receive: a frame longer than the buffer is cut to the buffer",
          "[dwm_mock]") {
  Fixture fixture = Fixture::get();
  // RXFCG set, and as RX_FINFO: RXFLEN 0x40 -- a 62-octet payload
  fixture.transcript.read_response = bytes({0x40, 0x40, 0x00, 0x00, 0x00});

  std::array<std::byte, 9> out{};
  const auto n = fixture.dwm.receive(out);

  TEST_ASSERT_TRUE(n.has_value());
  TEST_ASSERT_EQUAL_UINT32(9, *n);

  const auto &transfers = fixture.transcript.transfers;
  const auto buffer_read = std::ranges::find_if(transfers, [](const Transfer &t) {
    return t.tx.size() == 1 && t.tx[0] == std::byte{0x11};
  });
  TEST_ASSERT_TRUE(buffer_read != transfers.end());
  TEST_ASSERT_EQUAL_UINT32(9, buffer_read->rx);
}

TEST_CASE("device id: four octets read little-endian", "[dwm_mock]") {
  Fixture fixture = Fixture::get();
  fixture.transcript.read_response = bytes({0x30, 0x01, 0xCA, 0xDE});

  const auto id = fixture.dwm.get_device_id();

  TEST_ASSERT_TRUE(id.has_value());
  TEST_ASSERT_EQUAL_HEX32(0xDECA0130, *id);
  TEST_ASSERT_EQUAL_UINT32(1, fixture.transcript.transfers.size());
  check_bytes(bytes({0x00}), fixture.transcript.transfers[0].tx);
  TEST_ASSERT_EQUAL_UINT32(4, fixture.transcript.transfers[0].rx);
}

TEST_CASE("timestamp: all 40 bits come back, low ones included",
          "[dwm_mock]") {
  Fixture fixture = Fixture::get();
  fixture.transcript.read_response = bytes({0x01, 0x02, 0x03, 0x04, 0xFF});

  const auto t = fixture.dwm.get_rx_timestamp();

  TEST_ASSERT_TRUE(t.has_value());
  TEST_ASSERT_EQUAL_HEX64(0xFF04030201, std::to_underlying(*t));
  check_bytes(bytes({0x15}), fixture.transcript.transfers[0].tx);
  TEST_ASSERT_EQUAL_UINT32(5, fixture.transcript.transfers[0].rx);
}

TEST_CASE("timestamp: a difference across the 40-bit wrap stays small",
          "[dwm_mock]") {
  using dw1000::Ticks;
  using dw1000::Timestamp;

  static_assert(Timestamp{0x30} - Timestamp{0x10} == Ticks{0x20});
  static_assert(Timestamp{0x10} - Timestamp{0xFF'FFFF'FFF0} == Ticks{0x20});
}

TEST_CASE("device id: a failed transfer reaches the caller", "[dwm_mock]") {
  Fixture fixture = Fixture::get();
  fixture.transcript.fail_with = HAL::SpiError::TransferFailed;

  const auto id = fixture.dwm.get_device_id();

  TEST_ASSERT_FALSE(id.has_value());
  TEST_ASSERT_TRUE(id.error() == HAL::SpiError::TransferFailed);
}

TEST_CASE("set_tx_prf: reads TX_FCTRL, writes it back with only TXPRF changed",
          "[dwm_mock]") {
  Fixture fixture = Fixture::get();
  // the reset value: TFLEN 12, 6.8 Mbps, 16 MHz, preamble 128
  fixture.transcript.read_response = bytes({0x0C, 0x40, 0x15, 0x00, 0x00});

  TEST_ASSERT_TRUE(fixture.dwm.set_tx_prf(PRF::MHZ_64).has_value());

  const auto &transfers = fixture.transcript.transfers;
  TEST_ASSERT_EQUAL_UINT32(2, transfers.size());
  check_bytes(bytes({0x08}), transfers[0].tx);
  check_bytes(bytes({0x88, 0x0C, 0x40, 0x16, 0x00, 0x00}), transfers[1].tx);
}

TEST_CASE("set_tx_prf: a failed read never writes", "[dwm_mock]") {
  Fixture fixture = Fixture::get();
  fixture.transcript.fail_with = HAL::SpiError::TransferFailed;

  const auto result = fixture.dwm.set_tx_prf(PRF::MHZ_64);

  TEST_ASSERT_FALSE(result.has_value());
  TEST_ASSERT_TRUE(result.error() == HAL::SpiError::TransferFailed);
  TEST_ASSERT_EQUAL_UINT32(1, fixture.transcript.transfers.size());
}

TEST_CASE("configure: sends the bytes the hand-written table did",
          "[dwm_mock]") {
  Fixture fixture = Fixture::get();

  TEST_ASSERT_TRUE(fixture.dwm.configure().has_value());

  const std::vector<std::vector<std::byte>> expected{
      bytes({0xB6, 0x01, 0x03}),                   // PMSC_CTRL0 = 0x0301
      bytes({0xED, 0x06, 0x00, 0x80}),             // OTP_CTRL LDELOAD
      bytes({0xB6, 0x00, 0x02}),                   // PMSC_CTRL0 = 0x0200
      bytes({0xE3, 0x04, 0x70, 0x88}),             // AGC_TUNE1
      bytes({0xE3, 0x0C, 0x07, 0xA9, 0x02, 0x25}), // AGC_TUNE2
      bytes({0xE3, 0x12, 0x35, 0x00}),             // AGC_TUNE3
      bytes({0xE7, 0x02, 0x01, 0x00}),             // DRX_TUNE0b
      bytes({0xE7, 0x04, 0x87, 0x00}),             // DRX_TUNE1a
      bytes({0xE7, 0x06, 0x20, 0x00}),             // DRX_TUNE1b
      bytes({0xE7, 0x08, 0x52, 0x00, 0x1A, 0x33}), // DRX_TUNE2
      bytes({0xE7, 0x26, 0x28, 0x00}),             // DRX_TUNE4H
      bytes({0xE8, 0x0B, 0xD8}),                   // RF_RXCTRLH
      bytes({0xE8, 0x0C, 0xE0, 0x3F, 0x1E, 0x00}), // RF_TXCTRL
      bytes({0xEA, 0x0B, 0xC0}),                   // TC_PGDELAY
      bytes({0xEB, 0x07, 0x1D, 0x04, 0x00, 0x08}), // FS_PLLCFG
      bytes({0xEB, 0x0B, 0xBE}),                   // FS_PLLTUNE
      bytes({0xEE, 0x86, 0x10, 0x0D}),             // LDE_CFG1
      bytes({0xEE, 0x86, 0x30, 0x07, 0x16}),       // LDE_CFG2
      bytes({0xEE, 0x84, 0x50, 0x8E, 0x42}),       // LDE_REPC
      bytes({0x9F, 0x55, 0x00, 0x04, 0x21}),       // CHAN_CTRL
      bytes({0x98, 0x34, 0x40}),                   // TX_ANTD
      bytes({0xEE, 0x84, 0x30, 0x34, 0x40}),       // LDE_RXANTD
      // TX_FCTRL, each modified from the mock's all-zero read: TXPRF, TXBR,
      // then TXPSR and PE together
      bytes({0x88, 0x00, 0x00, 0x01, 0x00, 0x00}),
      bytes({0x88, 0x00, 0x40, 0x00, 0x00, 0x00}),
      bytes({0x88, 0x00, 0x00, 0x14, 0x00, 0x00}),
  };

  const auto writes = fixture.transcript.writes();
  TEST_ASSERT_EQUAL_UINT32(expected.size(), writes.size());

  for (size_t i = 0; i < expected.size(); ++i) {
    check_bytes(expected[i], writes[i]);
  }
}

TEST_CASE("transmit: strobes are whole SYS_CTRL writes, the clear is all of "
          "SYS_STATUS",
          "[dwm_mock]") {
  Fixture fixture = Fixture::get();
  // TXFRS set, so the poll ends on its first read
  fixture.transcript.read_response = bytes({0x80, 0x00, 0x00, 0x00, 0x00});

  const std::array payload{std::byte{0x01}};
  TEST_ASSERT_TRUE(fixture.dwm.transmit(payload).has_value());

  const auto writes = fixture.transcript.writes();
  TEST_ASSERT_EQUAL_UINT32(5, writes.size());
  check_bytes(bytes({0x8D, 0x40, 0x00, 0x00, 0x00}), writes[0]); // TRXOFF
  check_bytes(bytes({0x89, 0x01}), writes[1]);                   // TX_BUFFER
  // TFLEN = 3 over the read's 0x80, which is TFLE's low bit and stays
  check_bytes(bytes({0x88, 0x83, 0x00, 0x00, 0x00, 0x00}), writes[2]);
  check_bytes(bytes({0x8D, 0x02, 0x00, 0x00, 0x00}), writes[3]); // TXSTRT
  check_bytes(bytes({0x8F, 0x80, 0x00, 0x00, 0x00, 0x00}), writes[4]);
}

TEST_CASE("receive: frame length is RXFLE over RXFLEN, less the FCS",
          "[dwm_mock]") {
  Fixture fixture = Fixture::get();
  // one response serves every read: RXFCG set for the poll, and as RX_FINFO
  // the same octets give RXFLEN 0x40 | RXFLE 0 -- a 64-octet frame
  fixture.transcript.read_response = bytes({0x40, 0x40, 0x00, 0x00, 0x00});

  std::array<std::byte, 128> out{};
  const auto n = fixture.dwm.receive(out);

  TEST_ASSERT_TRUE(n.has_value());
  TEST_ASSERT_EQUAL_UINT32(62, *n);
}
