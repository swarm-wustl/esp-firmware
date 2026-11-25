#ifndef REGISTER_LAYOUTS_H
#define REGISTER_LAYOUTS_H

#include <algorithm>
#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <numeric>
#include <tuple>
#include <type_traits>
#include <utility>

#include "bits.h"
#include "registers.h"

namespace Registers {
// a register in a layout sits at an offset rather than at an address on a
// chip, and its own address type keeps it out of every driver's read
enum class Offset : uint8_t {};

template <typename F>
concept OctetField =
    Field<F> && F::width % BITS_PER_BYTE == 0 && (F::mask & 1) == 1;

template <typename P>
concept Part = OctetField<P> || Word<P>;

namespace detail {
template <typename P> struct part;

template <OctetField F> struct part<F> {
  // wraps a reg field in a reg to reuse decode/other generic register code
  using reg = Registers::reg<Offset{}, {.bytes = F::width / BITS_PER_BYTE}, F>;
  using in = F;
  using out = F;
  using fields = std::tuple<F>;

  template <typename G> static constexpr bool names = std::same_as<F, G>;

  static constexpr uint64_t word(F f) { return f.bits(); }
  static constexpr F read(uint64_t word) { return F{F::from(word)}; }

  template <typename G> static constexpr typename G::type get(const F &f) {
    return f.value;
  }
};

template <Word R> struct part<R> {
  using reg = R;
  using in = assignment<R>;
  using out = value<R>;
  using fields = typename R::fields;

  template <typename G> static constexpr bool names = R::template holds<G>;

  static constexpr uint64_t word(in a) { return a.word(); }
  static constexpr out read(uint64_t word) {
    return out{static_cast<typename R::word_type>(word)};
  }

  template <typename G> static constexpr typename G::type get(const out &v) {
    return v.template get<G>();
  }
};

template <typename G, typename... Ps>
constexpr size_t holders =
    (size_t{0} + ... + size_t{part<Ps>::template names<G>});

template <typename Fields, typename... Ps> struct named_once;

template <typename... Gs, typename... Ps>
struct named_once<std::tuple<Gs...>, Ps...>
    : std::bool_constant<((holders<Gs, Ps...> == 1) && ...)> {};

template <typename T, typename... Ts>
constexpr size_t count_of = (size_t{0} + ... + std::same_as<T, Ts>);
} // namespace detail

// every field name in exactly one part, on its own or inside a register, is
// what lets get<F> find it without knowing how the parts are grouped.
// lets us take in either fields or entire registers, with disjointedness
// still holding.
template <typename... Ps>
concept Layable =
    (Part<Ps> && ...) && ((detail::count_of<Ps, Ps...> == 1) && ...) &&
    (detail::named_once<typename detail::part<Ps>::fields, Ps...>::value &&
     ...);

// an ordered sequence of fields and registers packed into one byte buffer, each
// at an offset derived from the order rather than an address on a chip. It
// encodes and decodes the exact bytes, and reads any field by name. Used for
// wire formats such as the DW1000's 802.15.4 frames
template <typename... Ps>
  requires Layable<Ps...>
struct layout {
  static constexpr size_t bytes =
      (size_t{0} + ... + detail::part<Ps>::reg::bytes);

  static constexpr std::array<size_t, sizeof...(Ps)> offsets = [] {
    constexpr std::array<size_t, sizeof...(Ps)> sizes{
        detail::part<Ps>::reg::bytes...};
    std::array<size_t, sizeof...(Ps)> out{};
    std::exclusive_scan(sizes.begin(), sizes.end(), out.begin(), size_t{0});
    return out;
  }();

  using values = std::tuple<typename detail::part<Ps>::out...>;

  static constexpr std::array<std::byte, bytes>
  encode(typename detail::part<Ps>::in... parts) {
    return place(std::index_sequence_for<Ps...>{}, parts...);
  }

  template <octets_of<bytes> B> static constexpr values decode(const B &raw) {
    return slice(raw, std::index_sequence_for<Ps...>{});
  }

  template <Field F>
    requires(detail::holders<F, Ps...> == 1)
  static constexpr typename F::type get(const values &values) {
    using P = std::tuple_element_t<holder<F>, std::tuple<Ps...>>;
    return detail::part<P>::template get<F>(std::get<holder<F>>(values));
  }

private:
  template <Field F>
    requires(detail::holders<F, Ps...> == 1)
  static constexpr size_t holder = [] {
    constexpr std::array<bool, sizeof...(Ps)> holds{
        detail::part<Ps>::template names<F>...};
    return static_cast<size_t>(std::ranges::find(holds, true) - holds.begin());
  }();

  template <size_t... Is>
  static constexpr std::array<std::byte, bytes>
  place(std::index_sequence<Is...>, typename detail::part<Ps>::in... parts) {
    std::array<std::byte, bytes> out{};
    (place_bytes<typename detail::part<Ps>::reg, offsets[Is]>(
         out, detail::part<Ps>::word(parts)),
     ...);
    return out;
  }

  template <typename B, size_t... Is>
  static constexpr values slice(const B &raw, std::index_sequence<Is...>) {
    return {detail::part<Ps>::read(
        from_bytes<typename detail::part<Ps>::reg, offsets[Is]>(raw))...};
  }
};
} // namespace Registers

#endif
