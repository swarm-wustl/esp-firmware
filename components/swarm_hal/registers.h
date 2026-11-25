#ifndef REGISTERS_H
#define REGISTERS_H

#include <algorithm>
#include <array>
#include <bit>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <tuple>
#include <type_traits>
#include <utility>

#include "fixed_string.h"

// everything below is description and arithmetic that should leave no trace in
// the binary. -Og, the IDF default, won't inline it on its own, so it's forced
#define REGISTERS_INLINE [[gnu::always_inline]] inline

namespace Registers {
using Strings::fixed_string;

enum class Access : uint8_t { ReadOnly, ReadWrite, WriteOneClear };
enum class Endian : uint8_t { Little, Big };

template <typename T>
concept FieldValue =
    std::same_as<T, bool> || std::unsigned_integral<T> || std::is_enum_v<T>;

template <FieldValue T> constexpr size_t digits() {
  if constexpr (std::is_enum_v<T>) {
    return std::numeric_limits<std::underlying_type_t<T>>::digits;
  } else {
    return std::numeric_limits<T>::digits;
  }
}

template <FieldValue T> REGISTERS_INLINE constexpr uint64_t to_raw(T value) {
  if constexpr (std::is_enum_v<T>) {
    return static_cast<uint64_t>(std::to_underlying(value));
  } else {
    return static_cast<uint64_t>(value);
  }
}

// the name is what tells two fields apart: SLEEP and some other one-bit flag
// at bit 6 of another register would otherwise be the same type
//
// a field's access only ever narrows its register's: a read-only status bit in
// an otherwise write-one-to-clear register
template <fixed_string Name, FieldValue T, size_t Msb, size_t Lsb,
          Access A = Access::ReadWrite>
  requires(Msb >= Lsb && Msb < 64 && Msb - Lsb + 1 <= digits<T>())
struct field {
  using type = T;

  static constexpr auto name = Name;
  static constexpr Access access = A;
  static constexpr size_t width = Msb - Lsb + 1;
  static constexpr uint64_t mask =
      (width == 64 ? ~uint64_t{0} : (uint64_t{1} << width) - 1) << Lsb;

  T value;

  REGISTERS_INLINE constexpr uint64_t bits() const {
    return (to_raw(value) << Lsb) & mask;
  }

  REGISTERS_INLINE static constexpr T from(uint64_t word) {
    return static_cast<T>((word & mask) >> Lsb);
  }
};

template <typename F> constexpr bool is_field = false;

template <fixed_string Name, typename T, size_t Msb, size_t Lsb, Access A>
constexpr bool is_field<field<Name, T, Msb, Lsb, A>> = true;

template <typename F>
concept Field = is_field<F>;

struct Layout {
  size_t bytes = 1;
  Access access = Access::ReadWrite;
  uint64_t reset = 0;
  Endian endian = Endian::Little;
};

template <Field... Fs> constexpr uint64_t mask_of() {
  return (uint64_t{0} | ... | Fs::mask);
}

template <Field... Fs> constexpr bool disjoint() {
  return std::popcount(mask_of<Fs...>()) == (0 + ... + std::popcount(Fs::mask));
}

template <Layout L> constexpr bool within(uint64_t bits) {
  if constexpr (L.bytes >= 8) {
    return true;
  } else {
    return (bits >> (8 * L.bytes)) == 0;
  }
}

// fields live in a word, so only a register of at most eight octets has any;
// anything longer is a block of bytes, like a burst or a frame buffer
template <Layout L, Field... Fs>
constexpr bool describable =
    L.bytes > 0 && (L.bytes <= 8 || (sizeof...(Fs) == 0 && L.reset == 0)) &&
    within<L>(L.reset) && within<L>(mask_of<Fs...>()) && disjoint<Fs...>();

// the narrowest integer that holds the register, so a one-octet read isn't
// carried around as a 64-bit pair on a 32-bit core
template <size_t Bytes>
using word_for = std::conditional_t<
    Bytes <= 1, uint8_t,
    std::conditional_t<Bytes <= 2, uint16_t,
                       std::conditional_t<Bytes <= 4, uint32_t, uint64_t>>>;

template <auto Address, Layout L, Field... Fs>
  requires describable<L, Fs...>
struct reg {
  // only a word has one; a block is bytes all the way through
  using word_type = std::conditional_t<(L.bytes <= 8), word_for<L.bytes>, void>;

  using address_type = decltype(Address);

  static constexpr address_type address = Address;
  static constexpr size_t bytes = L.bytes;
  static constexpr Access access = L.access;
  static constexpr uint64_t reset = L.reset;
  static constexpr Endian endian = L.endian;

  using fields = std::tuple<Fs...>;

  template <typename F>
  static constexpr bool holds = (std::same_as<F, Fs> || ...);
};

template <typename R> constexpr bool is_reg = false;

template <auto Address, Layout L, typename... Fs>
constexpr bool is_reg<reg<Address, L, Fs...>> = true;

template <typename R>
concept Register = is_reg<R>;

template <typename R>
concept Word = Register<R> && R::bytes <= 8;

template <typename R>
concept Block = Register<R> && R::bytes > 8;

// a driver accepts only registers addressed in its own type, so a DWM
// register can't be handed to the MPU6050
template <typename R, typename Address>
concept at = Register<R> && std::same_as<typename R::address_type, Address>;

template <typename R, typename... Fs>
concept writable_with =
    Word<R> && R::access != Access::ReadOnly &&
    (R::template holds<Fs> && ...) &&
    ((Fs::access != Access::ReadOnly) && ...) && disjoint<Fs...>();

// a read-modify-write of a write-one-to-clear register writes back every bit
// that was set, clearing events nobody asked to clear
template <typename R, typename... Fs>
concept modifiable_with =
    writable_with<R, Fs...> && R::access == Access::ReadWrite;

// unnamed fields take the register's reset value, as if it had just come out
// of reset -- except on write-one-to-clear, where a 1 is an action
template <typename R, Field... Fs>
  requires writable_with<R, Fs...>
REGISTERS_INLINE constexpr uint64_t write_word(Fs... fs) {
  constexpr uint64_t base = R::access == Access::WriteOneClear ? 0 : R::reset;

  return (base & ~mask_of<Fs...>()) | (uint64_t{0} | ... | fs.bits());
}

template <typename R, Field... Fs>
  requires modifiable_with<R, Fs...>
REGISTERS_INLINE constexpr uint64_t modify_word(uint64_t current, Fs... fs) {
  return (current & ~mask_of<Fs...>()) | (uint64_t{0} | ... | fs.bits());
}

template <Word R, size_t Octet>
REGISTERS_INLINE constexpr std::byte octet(uint64_t word) {
  constexpr size_t shift =
      8 * (R::endian == Endian::Little ? Octet : R::bytes - 1 - Octet);

  return static_cast<std::byte>((word >> shift) & 0xFF);
}

// packs rather than loops: -Og keeps a loop as a loop, shifting a 64-bit word
// by a variable, where an expanded pack shifts by constants that fold away
template <Word R, size_t H, size_t... Hs, size_t... Is>
REGISTERS_INLINE constexpr std::array<std::byte, H + R::bytes>
prefixed(const std::array<std::byte, H> &header, uint64_t word,
         std::index_sequence<Hs...>, std::index_sequence<Is...>) {
  return {header[Hs]..., octet<R, Is>(word)...};
}

template <Word R, size_t H>
REGISTERS_INLINE constexpr std::array<std::byte, H + R::bytes>
prefixed(const std::array<std::byte, H> &header, uint64_t word) {
  return prefixed<R>(header, word, std::make_index_sequence<H>{},
                     std::make_index_sequence<R::bytes>{});
}

template <Word R>
REGISTERS_INLINE constexpr std::array<std::byte, R::bytes>
to_bytes(uint64_t word) {
  return prefixed<R>(std::array<std::byte, 0>{}, word);
}

// whatever is already exactly R's octets -- an array a bus filled, a slice of a
// frame -- read in place. Taking a span would cost a call per octet at -Og
template <typename B, size_t N>
concept octets_of = std::convertible_to<const B &, std::span<const std::byte, N>>;

template <Word R, octets_of<R::bytes> B, size_t... Is>
REGISTERS_INLINE constexpr uint64_t
from_bytes(const B &raw, std::index_sequence<Is...>) {
  return (uint64_t{0} | ... |
          (static_cast<uint64_t>(raw[Is])
           << 8 * (R::endian == Endian::Little ? Is : R::bytes - 1 - Is)));
}

template <Word R, octets_of<R::bytes> B>
REGISTERS_INLINE constexpr uint64_t from_bytes(const B &raw) {
  return from_bytes<R>(raw, std::make_index_sequence<R::bytes>{});
}

template <Word R> struct value {
  typename R::word_type word;

  template <Field F>
    requires(R::template holds<F>)
  REGISTERS_INLINE constexpr typename F::type get() const {
    return F::from(word);
  }
};

template <typename R, Field... Fs>
  requires writable_with<R, Fs...>
REGISTERS_INLINE constexpr std::array<std::byte, R::bytes> encode(Fs... fs) {
  return to_bytes<R>(write_word<R>(fs...));
}

template <Word R> class assignment;

template <typename R, Field... Fs>
  requires writable_with<R, Fs...>
REGISTERS_INLINE constexpr assignment<R> set(Fs... fs);

// one write, settled at compile time: what a configuration table is a list of.
// It carries its register in its type, so a table entry is checked like any
// other write. Only set makes one, so a reading can't pass for a checked write
// -- write it back to a write-one-to-clear register and every event clears
template <Word R> class assignment {
public:
  REGISTERS_INLINE constexpr typename R::word_type word() const {
    return word_;
  }

private:
  REGISTERS_INLINE constexpr explicit assignment(typename R::word_type word)
      : word_{word} {}

  typename R::word_type word_;

  template <typename R2, Field... Fs>
    requires writable_with<R2, Fs...>
  friend constexpr assignment<R2> set(Fs... fs);
};

template <typename R, Field... Fs>
  requires writable_with<R, Fs...>
REGISTERS_INLINE constexpr assignment<R> set(Fs... fs) {
  return assignment<R>{
      static_cast<typename R::word_type>(write_word<R>(fs...))};
}

// what a bus write sends: the device's own header -- a register number, an
// SPI command -- followed by the value
template <typename R, size_t H, Field... Fs>
  requires writable_with<R, Fs...>
REGISTERS_INLINE constexpr std::array<std::byte, H + R::bytes>
frame(const std::array<std::byte, H> &header, Fs... fs) {
  return prefixed<R>(header, write_word<R>(fs...));
}

template <Word R, size_t H>
REGISTERS_INLINE constexpr std::array<std::byte, H + R::bytes>
frame(const std::array<std::byte, H> &header, assignment<R> a) {
  return prefixed<R>(header, a.word());
}

template <Word R, size_t H>
REGISTERS_INLINE constexpr std::array<std::byte, H + R::bytes>
frame(const std::array<std::byte, H> &header, value<R> v) {
  return prefixed<R>(header, v.word);
}

template <Register R, octets_of<R::bytes> B>
REGISTERS_INLINE constexpr auto decode(const B &raw) {
  if constexpr (Word<R>) {
    return value<R>{static_cast<typename R::word_type>(from_bytes<R>(raw))};
  } else if constexpr (std::same_as<B, std::array<std::byte, R::bytes>>) {
    return raw;
  } else {
    std::array<std::byte, R::bytes> out{};
    std::ranges::copy(raw, out.begin());
    return out;
  }
}
} // namespace Registers

#undef REGISTERS_INLINE

#endif
