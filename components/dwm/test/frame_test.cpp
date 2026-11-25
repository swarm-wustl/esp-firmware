#include "dwm_frames.h"
#include "unity.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <span>
#include <utility>

namespace {
namespace mac = dw1000::mac;
using dw1000::PanId;
using dw1000::ShortAddress;

constexpr dw1000::Identity kAnchor{.pan = PanId{0xDECA},
                                   .address = ShortAddress{0x0A01}};

constexpr mac::Header kToAnchor{.sequence = 7,
                                .pan = kAnchor.pan,
                                .destination = kAnchor.address,
                                .source = ShortAddress{0x0001}};

template <size_t N>
void check_octets(std::initializer_list<int> expected,
                  const std::array<std::byte, N> &actual) {
  TEST_ASSERT_EQUAL_UINT32(expected.size(), N);

  size_t i = 0;
  for (const int e : expected) {
    TEST_ASSERT_EQUAL_HEX8(e, static_cast<uint8_t>(actual[i++]));
  }
}

using Pair = mac::layout<mac::SEQUENCE, mac::KIND>;

template <typename... As>
concept encodes = requires(As... as) { Pair::encode(as...); };

template <typename... Cs>
concept lays_out = requires { typename mac::layout<Cs...>; };

template <typename F>
concept poll_gets = requires(const mac::Poll::wire::values &v) {
  mac::Poll::wire::get<F>(v);
};

using NIBBLE = Registers::field<"NIBBLE", uint8_t, 3, 0>;
using HIGH = Registers::field<"HIGH", uint8_t, 15, 8>;
using HOLDS_SEQUENCE = Registers::reg<mac::Part{}, {.bytes = 1}, mac::SEQUENCE>;
} // namespace

TEST_CASE("mac: frame control is a PAN-compressed data frame, short at both "
          "ends",
          "[dwm_frame]") {
  // 0x8841, what arduino-dw1000 and bitcraze's lps firmware send
  TEST_ASSERT_EQUAL_HEX16(0x8841, mac::DATA_FRAME.word());
}

TEST_CASE("mac: a frame's size is its parts'", "[dwm_frame]") {
  static_assert(mac::Poll::wire::bytes == 10);
  static_assert(mac::Final::wire::bytes == 15);
  static_assert(mac::Final::wire::offsets.back() == 10);
  TEST_ASSERT_TRUE(true);
}

TEST_CASE("mac: encode takes each chunk once, in layout order",
          "[dwm_frame]") {
  static_assert(encodes<mac::SEQUENCE, mac::KIND>);
  static_assert(!encodes<mac::KIND, mac::SEQUENCE>);
  static_assert(!encodes<mac::SEQUENCE>);
  static_assert(!encodes<mac::SEQUENCE, mac::SEQUENCE>);

  check_octets({0x01, 0x01},
               Pair::encode(mac::SEQUENCE{1}, mac::KIND{mac::Kind::Poll}));
}

TEST_CASE("mac: a layout takes whole-octet fields and registers, each field "
          "named once",
          "[dwm_frame]") {
  static_assert(lays_out<mac::FRAME_CONTROL, mac::SEQUENCE>);
  static_assert(!lays_out<NIBBLE>);
  static_assert(!lays_out<HIGH>);
  static_assert(!lays_out<mac::SEQUENCE, mac::SEQUENCE>);
  // SEQUENCE once on its own and once inside a register
  static_assert(!lays_out<HOLDS_SEQUENCE, mac::SEQUENCE>);
  TEST_ASSERT_TRUE(true);
}

TEST_CASE("mac: get finds a field by name, alone or inside a register",
          "[dwm_frame]") {
  using Wire = mac::Poll::wire;
  const auto raw = mac::encode(kToAnchor, mac::Poll{});
  const auto values = Wire::decode(raw);

  TEST_ASSERT_EQUAL_UINT8(7, Wire::get<mac::SEQUENCE>(values));
  TEST_ASSERT_TRUE(Wire::get<mac::FRAME_TYPE>(values) == mac::FrameType::Data);
  TEST_ASSERT_TRUE(Wire::get<mac::DEST_MODE>(values) == mac::AddressMode::Short);
  TEST_ASSERT_TRUE(Wire::get<mac::PAN_COMPRESS>(values));

  static_assert(poll_gets<mac::SRC_ADDR>);
  static_assert(!poll_gets<mac::REPLY_TIME>);
}

TEST_CASE("mac: a poll is the header, its kind and nothing else",
          "[dwm_frame]") {
  check_octets({0x41, 0x88, 0x07, 0xCA, 0xDE, 0x01, 0x0A, 0x01, 0x00, 0x01},
               mac::encode(kToAnchor, mac::Poll{}));
}

TEST_CASE("mac: a final carries all 40 bits of the reply time",
          "[dwm_frame]") {
  const auto raw =
      mac::encode(kToAnchor, mac::Final{mac::REPLY_TIME{0xAB'1234'5678}});

  check_octets({0x41, 0x88, 0x07, 0xCA, 0xDE, 0x01, 0x0A, 0x01, 0x00, 0x03,
                0x78, 0x56, 0x34, 0x12, 0xAB},
               raw);

  const auto decoded = mac::decode<mac::Final>(raw);
  TEST_ASSERT_TRUE(decoded.has_value());
  TEST_ASSERT_EQUAL_HEX64(0xAB'1234'5678,
                          decoded->body.get<mac::REPLY_TIME>());
  TEST_ASSERT_EQUAL_UINT8(7, decoded->header.sequence);
  TEST_ASSERT_EQUAL_HEX16(0xDECA, std::to_underlying(decoded->header.pan));
  TEST_ASSERT_EQUAL_HEX16(0x0A01,
                          std::to_underlying(decoded->header.destination));
  TEST_ASSERT_EQUAL_HEX16(0x0001, std::to_underlying(decoded->header.source));
}

TEST_CASE("mac: decode rejects a frame of the wrong length", "[dwm_frame]") {
  const auto raw = mac::encode(kToAnchor, mac::Poll{});

  const auto short_by_one =
      mac::decode<mac::Poll>(std::span{raw}.first(raw.size() - 1));
  TEST_ASSERT_FALSE(short_by_one.has_value());
  TEST_ASSERT_TRUE(short_by_one.error() == mac::FrameError::Length);

  // the right header with the wrong kind's length is still a length error
  const auto as_final = mac::decode<mac::Final>(raw);
  TEST_ASSERT_FALSE(as_final.has_value());
  TEST_ASSERT_TRUE(as_final.error() == mac::FrameError::Length);
}

TEST_CASE("mac: decode rejects a frame control it doesn't send",
          "[dwm_frame]") {
  auto ack = mac::encode(kToAnchor, mac::Poll{});
  ack[0] = std::byte{0x42};

  auto extended = mac::encode(kToAnchor, mac::Poll{});
  extended[1] = std::byte{0xCC};

  for (const auto &raw : {ack, extended}) {
    const auto decoded = mac::decode<mac::Poll>(raw);
    TEST_ASSERT_FALSE(decoded.has_value());
    TEST_ASSERT_TRUE(decoded.error() == mac::FrameError::Control);
  }
}

TEST_CASE("mac: decode rejects a frame of another kind", "[dwm_frame]") {
  const auto decoded =
      mac::decode<mac::Reply>(mac::encode(kToAnchor, mac::Poll{}));

  TEST_ASSERT_FALSE(decoded.has_value());
  TEST_ASSERT_TRUE(decoded.error() == mac::FrameError::Kind);
}

TEST_CASE("mac: a node hears its own address and broadcast, nothing else",
          "[dwm_frame]") {
  auto to = [](PanId pan, ShortAddress destination) {
    return mac::Header{.sequence = 0,
                       .pan = pan,
                       .destination = destination,
                       .source = ShortAddress{0x0001}};
  };

  TEST_ASSERT_TRUE(mac::accepts(kAnchor, to(kAnchor.pan, kAnchor.address)));
  TEST_ASSERT_TRUE(mac::accepts(kAnchor, to(kAnchor.pan, dw1000::BROADCAST)));
  TEST_ASSERT_TRUE(mac::accepts(kAnchor, to(dw1000::ANY_PAN, kAnchor.address)));
  TEST_ASSERT_FALSE(mac::accepts(kAnchor, to(kAnchor.pan, ShortAddress{0x0A02})));
  TEST_ASSERT_FALSE(mac::accepts(kAnchor, to(PanId{0xBEEF}, kAnchor.address)));
}
