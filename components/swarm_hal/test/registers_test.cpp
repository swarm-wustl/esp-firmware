#include "registers.h"
#include "unity.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace {
using Registers::Access;
using Registers::Endian;

enum class Addr : uint8_t { Power, Status, Wide, Id, Burst };
enum class Other : uint8_t { Power };

enum class Mode : uint8_t { Off, Slow, Fast, Max };

using RESET = Registers::field<"RESET", bool, 7, 7>;
using SLEEP = Registers::field<"SLEEP", bool, 6, 6>;
using MODE = Registers::field<"MODE", Mode, 1, 0>;
using LOW = Registers::field<"LOW", uint8_t, 7, 0>;
using HIGH = Registers::field<"HIGH", uint8_t, 15, 8>;
using DONE = Registers::field<"DONE", bool, 0, 0>;
using PENDING = Registers::field<"PENDING", bool, 2, 2, Access::ReadOnly>;
using ERROR = Registers::field<"ERROR", bool, 1, 1>;

// shaped like SLEEP but a different field: the name is the identity
using ASLEEP = Registers::field<"ASLEEP", bool, 6, 6>;

using POWER =
    Registers::reg<Addr::Power, {.reset = 0x40}, RESET, SLEEP, MODE>;
using STATUS = Registers::reg<Addr::Status,
                              {.access = Access::WriteOneClear, .reset = 0x03},
                              DONE, ERROR, PENDING>;
using WIDE_LE = Registers::reg<Addr::Wide, {.bytes = 2}, LOW, HIGH>;
using WIDE_BE =
    Registers::reg<Addr::Wide, {.bytes = 2, .endian = Endian::Big}, LOW, HIGH>;
using ID = Registers::reg<Addr::Id, {.access = Access::ReadOnly, .reset = 0x68},
                          LOW>;
using BURST =
    Registers::reg<Addr::Burst, {.bytes = 14, .access = Access::ReadOnly}>;
using FOREIGN = Registers::reg<Other::Power, {.reset = 0x40}, SLEEP>;

template <typename R, typename... Fs>
concept writes = requires(Fs... fs) { Registers::write_word<R>(fs...); };

template <typename R, typename... Fs>
concept sets = requires(Fs... fs) { Registers::set<R>(fs...); };

template <typename R>
concept forges = requires(typename R::word_type w) { Registers::assignment<R>{w}; };

template <typename R, typename... Fs>
concept modifies =
    requires(Fs... fs) { Registers::modify_word<R>(uint64_t{}, fs...); };

template <typename R, typename F>
concept reads = requires(Registers::value<R> v) { v.template get<F>(); };

template <auto Address, Registers::Layout L, typename... Fs>
concept describes = requires { typename Registers::reg<Address, L, Fs...>; };

using OVERLAPS_SLEEP = Registers::field<"OVERLAP", uint8_t, 6, 5>;
using PAST_ONE_OCTET = Registers::field<"PAST", uint8_t, 9, 8>;
} // namespace

TEST_CASE("registers: unnamed fields take the reset value", "[registers]") {
  static_assert(Registers::write_word<POWER>(MODE{Mode::Fast}) == 0x42);
  static_assert(Registers::write_word<POWER>(SLEEP{false}, MODE{Mode::Fast}) ==
                0x02);
  static_assert(Registers::write_word<POWER>(RESET{true}) == 0xC0);
  static_assert(Registers::write_word<POWER>() == 0x40);
  TEST_ASSERT_TRUE(true);
}

TEST_CASE("registers: write-one-to-clear never clears what it wasn't told to",
          "[registers]") {
  static_assert(Registers::write_word<STATUS>(DONE{true}) == 0x01);
  static_assert(Registers::write_word<STATUS>() == 0x00);
  TEST_ASSERT_TRUE(true);
}

TEST_CASE("registers: modify keeps every field it doesn't name",
          "[registers]") {
  static_assert(Registers::modify_word<POWER>(0x83, SLEEP{true}) == 0xC3);
  static_assert(Registers::modify_word<POWER>(0xC3, MODE{Mode::Off}) == 0xC0);
  TEST_ASSERT_TRUE(true);
}

TEST_CASE("registers: a value wider than its field is cut to the field",
          "[registers]") {
  static_assert(Registers::write_word<POWER>(MODE{static_cast<Mode>(0xFF)}) ==
                0x43);
  TEST_ASSERT_TRUE(true);
}

TEST_CASE("registers: octet order follows the register's endianness",
          "[registers]") {
  constexpr auto le = Registers::encode<WIDE_LE>(LOW{0x34}, HIGH{0x12});
  constexpr auto be = Registers::encode<WIDE_BE>(LOW{0x34}, HIGH{0x12});

  static_assert(le[0] == std::byte{0x34} && le[1] == std::byte{0x12});
  static_assert(be[0] == std::byte{0x12} && be[1] == std::byte{0x34});
  static_assert(Registers::from_bytes<WIDE_LE>(le) == 0x1234);
  static_assert(Registers::from_bytes<WIDE_BE>(be) == 0x1234);

  constexpr auto framed =
      Registers::frame<WIDE_BE>(std::array{std::byte{0xAA}}, LOW{0x34});
  static_assert(framed.size() == 3 && framed[0] == std::byte{0xAA} &&
                framed[1] == std::byte{0x00} && framed[2] == std::byte{0x34});
  TEST_ASSERT_TRUE(true);
}

TEST_CASE("registers: a read decodes each field to its own type",
          "[registers]") {
  constexpr auto power = Registers::decode<POWER>({std::byte{0x43}});

  static_assert(power.get<MODE>() == Mode::Max);
  static_assert(power.get<SLEEP>());
  static_assert(!power.get<RESET>());

  constexpr std::array<std::byte, 14> raw{std::byte{1}, std::byte{2}};
  static_assert(Registers::decode<BURST>(raw) == raw);
  TEST_ASSERT_TRUE(true);
}

TEST_CASE("registers: illegal writes don't compile", "[registers]") {
  static_assert(writes<POWER, SLEEP, MODE>);
  static_assert(!writes<POWER, ASLEEP>);
  static_assert(!writes<POWER, LOW>);
  static_assert(!writes<POWER, SLEEP, SLEEP>);
  static_assert(!writes<ID, LOW>);
  static_assert(!writes<BURST>);
  static_assert(writes<STATUS, DONE>);
  TEST_ASSERT_TRUE(true);
}

TEST_CASE("registers: a read-only field can't be written, even in a writable "
          "register",
          "[registers]") {
  static_assert(writes<STATUS, DONE, ERROR>);
  static_assert(!writes<STATUS, PENDING>);
  static_assert(!writes<STATUS, DONE, PENDING>);
  static_assert(reads<STATUS, PENDING>);
  TEST_ASSERT_TRUE(true);
}

TEST_CASE("registers: a table entry is a write settled at compile time",
          "[registers]") {
  constexpr auto entry = Registers::set<POWER>(MODE{Mode::Fast});
  static_assert(entry.word() == 0x42);

  constexpr auto framed =
      Registers::frame<POWER>(std::array{std::byte{0x6B}}, entry);
  static_assert(framed[0] == std::byte{0x6B} && framed[1] == std::byte{0x42});

  static_assert(sets<POWER, MODE>);
  static_assert(!sets<POWER, LOW>);
  static_assert(!sets<ID, LOW>);
  // a reading, or any number, can't pass for a checked write
  static_assert(!forges<POWER>);
  static_assert(!forges<ID>);
  static_assert(!std::is_default_constructible_v<Registers::assignment<POWER>>);
  TEST_ASSERT_TRUE(true);
}

TEST_CASE("registers: write-one-to-clear has no read-modify-write",
          "[registers]") {
  static_assert(modifies<POWER, SLEEP>);
  static_assert(!modifies<STATUS, DONE>);
  static_assert(!modifies<ID, LOW>);
  TEST_ASSERT_TRUE(true);
}

TEST_CASE("registers: only a register's own fields can be read from it",
          "[registers]") {
  static_assert(reads<POWER, SLEEP>);
  static_assert(reads<ID, LOW>);
  static_assert(!reads<POWER, ASLEEP>);
  static_assert(!reads<POWER, LOW>);
  TEST_ASSERT_TRUE(true);
}

TEST_CASE("registers: a driver takes only its own registers", "[registers]") {
  static_assert(Registers::at<POWER, Addr>);
  static_assert(!Registers::at<FOREIGN, Addr>);
  static_assert(!Registers::at<SLEEP, Addr>);
  TEST_ASSERT_TRUE(true);
}

TEST_CASE("registers: a description that can't be true doesn't exist",
          "[registers]") {
  static_assert(describes<Addr::Power, {}, SLEEP, MODE>);
  static_assert(!describes<Addr::Power, {}, SLEEP, OVERLAPS_SLEEP>);
  static_assert(!describes<Addr::Power, {}, PAST_ONE_OCTET>);
  static_assert(!describes<Addr::Power, {.reset = 0x100}, SLEEP>);
  static_assert(!describes<Addr::Power, {.bytes = 0}>);
  static_assert(!describes<Addr::Burst, {.bytes = 14}, SLEEP>);
  TEST_ASSERT_TRUE(true);
}
