#include "register_layouts.h"
#include "unity.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <initializer_list>

namespace {
using Registers::Endian;
using Registers::Offset;

enum class Mode : uint8_t { Off, Slow, Fast };

using READY = Registers::field<"READY", bool, 0, 0>;
using MODE = Registers::field<"MODE", Mode, 2, 1>;
using TAG = Registers::field<"TAG", uint8_t, 7, 0>;
using COUNT = Registers::field<"COUNT", uint16_t, 15, 0>;
using LOW = Registers::field<"LOW", uint8_t, 7, 0>;
using HIGH = Registers::field<"HIGH", uint8_t, 15, 8>;
using NIBBLE = Registers::field<"NIBBLE", uint8_t, 3, 0>;

using CONTROL = Registers::reg<Offset{}, {}, READY, MODE>;
using BIG = Registers::reg<Offset{}, {.bytes = 2, .endian = Endian::Big}, LOW, HIGH>;
using HOLDS_TAG = Registers::reg<Offset{}, {}, TAG>;
using BURST = Registers::reg<Offset{}, {.bytes = 14}>;

using Packet = Registers::layout<CONTROL, TAG, COUNT>;

template <typename... As>
concept encodes = requires(As... as) { Packet::encode(as...); };

template <typename... Ps>
concept lays_out = requires { typename Registers::layout<Ps...>; };

template <typename F>
concept gets = requires(const Packet::values &v) { Packet::get<F>(v); };

constexpr auto kControl = Registers::set<CONTROL>(READY{true}, MODE{Mode::Fast});

template <size_t N>
void check_octets(std::initializer_list<int> expected, const std::array<std::byte, N> &actual) {
  TEST_ASSERT_EQUAL_UINT32(expected.size(), N);

  size_t i = 0;
  for (const int e : expected) {
    TEST_ASSERT_EQUAL_HEX8(e, static_cast<uint8_t>(actual[i++]));
  }
}
} // namespace

TEST_CASE("layout: parts sit back to back in the order listed", "[layout]") {
  static_assert(Packet::bytes == 4);
  static_assert(Packet::offsets == std::array<size_t, 3>{0, 1, 2});

  check_octets({0x05, 0xAB, 0x34, 0x12}, Packet::encode(kControl, TAG{0xAB}, COUNT{0x1234}));
}

TEST_CASE("layout: encode takes each part once, in layout order", "[layout]") {
  static_assert(encodes<Registers::assignment<CONTROL>, TAG, COUNT>);
  static_assert(!encodes<Registers::assignment<CONTROL>, COUNT, TAG>);
  static_assert(!encodes<Registers::assignment<CONTROL>, TAG>);
  // a register part is a checked write, not a reading
  static_assert(!encodes<Registers::value<CONTROL>, TAG, COUNT>);
  TEST_ASSERT_TRUE(true);
}

TEST_CASE("layout: a register part keeps its own byte order", "[layout]") {
  using Mixed = Registers::layout<TAG, BIG>;

  check_octets({0x01, 0x12, 0x34},
               Mixed::encode(TAG{1}, Registers::set<BIG>(LOW{0x34}, HIGH{0x12})));
}

TEST_CASE("layout: get finds a field by name, alone or inside a register", "[layout]") {
  const auto values = Packet::decode(Packet::encode(kControl, TAG{0xAB}, COUNT{0x1234}));

  TEST_ASSERT_EQUAL_HEX8(0xAB, Packet::get<TAG>(values));
  TEST_ASSERT_EQUAL_HEX16(0x1234, Packet::get<COUNT>(values));
  TEST_ASSERT_TRUE(Packet::get<READY>(values));
  TEST_ASSERT_TRUE(Packet::get<MODE>(values) == Mode::Fast);

  static_assert(gets<COUNT>);
  static_assert(!gets<LOW>);
}

TEST_CASE("layout: takes whole-octet fields and registers, each field named once", "[layout]") {
  static_assert(lays_out<CONTROL, TAG>);
  static_assert(!lays_out<NIBBLE>);
  // whole octets, but not from bit 0
  static_assert(!lays_out<HIGH>);
  static_assert(!lays_out<BURST>);
  static_assert(!lays_out<TAG, TAG>);
  // TAG once on its own and once inside a register
  static_assert(!lays_out<HOLDS_TAG, TAG>);
  TEST_ASSERT_TRUE(true);
}
