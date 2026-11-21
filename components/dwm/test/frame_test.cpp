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

using Pair = mac::layout<mac::part<mac::SEQUENCE>, mac::part<mac::KIND>>;

constexpr auto kSeq = Registers::set<mac::part<mac::SEQUENCE>>(mac::SEQUENCE{1});
constexpr auto kKind =
    Registers::set<mac::part<mac::KIND>>(mac::KIND{mac::Kind::Poll});

template <typename... As>
concept encodes = requires(As... as) { Pair::encode(as...); };
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

TEST_CASE("mac: a layout takes each part once, in any order", "[dwm_frame]") {
  static_assert(encodes<decltype(kSeq), decltype(kKind)>);
  static_assert(encodes<decltype(kKind), decltype(kSeq)>);
  static_assert(!encodes<decltype(kSeq)>);
  static_assert(!encodes<decltype(kSeq), decltype(kSeq)>);

  check_octets({0x01, 0x01}, Pair::encode(kKind, kSeq));
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
