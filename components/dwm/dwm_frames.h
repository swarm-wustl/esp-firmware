#ifndef DWM_FRAMES_H
#define DWM_FRAMES_H

#include <algorithm>
#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <numeric>
#include <span>
#include <tuple>
#include <type_traits>
#include <utility>

#include "dwm_regs.h"
#include "registers.h"

// IEEE 802.15.4 data frames (manual 11) in the one shape the DW1000's frame
// filter matches against PANADR (manual 5.2.1): one PAN, short addresses at
// both ends
namespace dw1000::mac {
// a part sits at an offset in a frame rather than at an address on the chip,
// and its own address type keeps it out of DWM::read
enum class Part : uint8_t {};

enum class FrameType : uint8_t {
  Beacon = 0b000,
  Data = 0b001,
  Ack = 0b010,
  Command = 0b011
};

enum class AddressMode : uint8_t { None = 0b00, Short = 0b10, Extended = 0b11 };

using FRAME_TYPE = Registers::field<"FRAME_TYPE", FrameType, 2, 0>;
using SECURITY = Registers::field<"SECURITY", bool, 3, 3>;
using PENDING = Registers::field<"PENDING", bool, 4, 4>;
using ACK_REQUEST = Registers::field<"ACK_REQUEST", bool, 5, 5>;
using PAN_COMPRESS = Registers::field<"PAN_COMPRESS", bool, 6, 6>;
using DEST_MODE = Registers::field<"DEST_MODE", AddressMode, 11, 10>;
using FRAME_VERSION = Registers::field<"FRAME_VERSION", uint8_t, 13, 12>;
using SRC_MODE = Registers::field<"SRC_MODE", AddressMode, 15, 14>;

using FRAME_CONTROL = Registers::reg<Part{}, {.bytes = 2}, FRAME_TYPE, SECURITY,
                                     PENDING, ACK_REQUEST, PAN_COMPRESS,
                                     DEST_MODE, FRAME_VERSION, SRC_MODE>;

inline constexpr auto DATA_FRAME = Registers::set<FRAME_CONTROL>(
    FRAME_TYPE{FrameType::Data}, PAN_COMPRESS{true},
    DEST_MODE{AddressMode::Short}, SRC_MODE{AddressMode::Short});

enum class Kind : uint8_t { Poll = 0x01, Reply = 0x02, Final = 0x03 };

using SEQUENCE = Registers::field<"SEQUENCE", uint8_t, 7, 0>;
using DEST_PAN = Registers::field<"DEST_PAN", PanId, 15, 0>;
using DEST_ADDR = Registers::field<"DEST_ADDR", ShortAddress, 15, 0>;
using SRC_ADDR = Registers::field<"SRC_ADDR", ShortAddress, 15, 0>;
using KIND = Registers::field<"KIND", Kind, 7, 0>;
using REPLY_TIME =
    Registers::field<"REPLY_TIME", uint64_t, RX_STAMP::width - 1, 0>;

template <typename F>
concept OctetField =
    Registers::Field<F> && F::width % 8 == 0 && (F::mask & 1) == 1;

template <typename C>
concept Chunk = OctetField<C> || Registers::Word<C>;

namespace detail {
template <OctetField F>
using part = Registers::reg<Part{}, {.bytes = F::width / 8}, F>;

// a field chunk is wrapped in a register of its own width, so both kinds reach
// the bytes through the same Registers code
template <typename C> struct chunk;

template <OctetField F> struct chunk<F> {
  using reg = part<F>;
  using in = F;
  using out = F;
  using fields = std::tuple<F>;

  template <typename G> static constexpr bool names = std::same_as<F, G>;

  static constexpr uint64_t word(F f) { return f.bits(); }
  static constexpr F read(Registers::value<reg> v) {
    return F{v.template get<F>()};
  }

  template <typename G> static constexpr typename G::type get(const F &f) {
    return f.value;
  }
};

template <Registers::Word R> struct chunk<R> {
  using reg = R;
  using in = Registers::assignment<R>;
  using out = Registers::value<R>;
  using fields = typename R::fields;

  template <typename G> static constexpr bool names = R::template holds<G>;

  static constexpr uint64_t word(in a) { return a.word(); }
  static constexpr out read(out v) { return v; }

  template <typename G> static constexpr typename G::type get(const out &v) {
    return v.template get<G>();
  }
};

template <typename G, typename... Cs>
constexpr size_t holders =
    (size_t{0} + ... + size_t{chunk<Cs>::template names<G>});

template <typename Fields, typename... Cs> struct named_once;

template <typename... Gs, typename... Cs>
struct named_once<std::tuple<Gs...>, Cs...>
    : std::bool_constant<((holders<Gs, Cs...> == 1) && ...)> {};

template <typename T, typename... Ts>
constexpr size_t count_of = (size_t{0} + ... + std::same_as<T, Ts>);
} // namespace detail

// every field name in exactly one chunk, on its own or inside a register, is
// what lets get<F> find it without knowing how the chunks are grouped
template <typename... Cs>
concept Layable =
    (Chunk<Cs> && ...) && ((detail::count_of<Cs, Cs...> == 1) && ...) &&
    (detail::named_once<typename detail::chunk<Cs>::fields, Cs...>::value &&
     ...);

// chunks back to back in the order listed. A field chunk goes in and comes out
// as itself, a register chunk as its assignment and its value
template <typename... Cs>
  requires Layable<Cs...>
struct layout {
  static constexpr size_t bytes =
      (size_t{0} + ... + detail::chunk<Cs>::reg::bytes);

  static constexpr std::array<size_t, sizeof...(Cs)> offsets = [] {
    constexpr std::array<size_t, sizeof...(Cs)> sizes{
        detail::chunk<Cs>::reg::bytes...};
    std::array<size_t, sizeof...(Cs)> out{};
    std::exclusive_scan(sizes.begin(), sizes.end(), out.begin(), size_t{0});
    return out;
  }();

  using values = std::tuple<typename detail::chunk<Cs>::out...>;

  static constexpr std::array<std::byte, bytes>
  encode(typename detail::chunk<Cs>::in... chunks) {
    return place(std::index_sequence_for<Cs...>{}, chunks...);
  }

  static constexpr values decode(std::span<const std::byte, bytes> raw) {
    return slice(raw, std::index_sequence_for<Cs...>{});
  }

  template <Registers::Field F>
    requires(detail::holders<F, Cs...> == 1)
  static constexpr typename F::type get(const values &values) {
    using C = std::tuple_element_t<holder<F>, std::tuple<Cs...>>;
    return detail::chunk<C>::template get<F>(std::get<holder<F>>(values));
  }

private:
  template <Registers::Field F>
    requires(detail::holders<F, Cs...> == 1)
  static constexpr size_t holder = [] {
    constexpr std::array<bool, sizeof...(Cs)> holds{
        detail::chunk<Cs>::template names<F>...};
    return static_cast<size_t>(std::ranges::find(holds, true) - holds.begin());
  }();

  template <size_t... Is>
  static constexpr std::array<std::byte, bytes>
  place(std::index_sequence<Is...>, typename detail::chunk<Cs>::in... chunks) {
    std::array<std::byte, bytes> out{};
    (std::ranges::copy(Registers::to_bytes<typename detail::chunk<Cs>::reg>(
                           detail::chunk<Cs>::word(chunks)),
                       out.begin() + offsets[Is]),
     ...);
    return out;
  }

  template <size_t... Is>
  static constexpr values slice(std::span<const std::byte, bytes> raw,
                                std::index_sequence<Is...>) {
    return {detail::chunk<Cs>::read(
        Registers::decode<typename detail::chunk<Cs>::reg>(
            raw.template subspan<offsets[Is], detail::chunk<Cs>::reg::bytes>()))...};
  }
};

struct Header {
  uint8_t sequence;
  PanId pan;
  ShortAddress destination;
  ShortAddress source;
};

template <Kind K, Registers::Field... Fs> struct message {
  static constexpr Kind kind = K;

  // PAN_COMPRESS: with both addresses present the source PAN is left out and
  // taken to be the destination's
  using wire = layout<FRAME_CONTROL, SEQUENCE, DEST_PAN, DEST_ADDR, SRC_ADDR,
                      KIND, Fs...>;

  std::tuple<Fs...> fields;

  constexpr message(Fs... fs) : fields{fs...} {}

  template <Registers::Field F>
    requires(std::same_as<F, Fs> || ...)
  constexpr typename F::type get() const {
    return std::get<F>(fields).value;
  }

  static constexpr message from(const typename wire::values &values) {
    return message{Fs{wire::template get<Fs>(values)}...};
  }
};

using Poll = message<Kind::Poll>;
using Reply = message<Kind::Reply>;
using Final = message<Kind::Final, REPLY_TIME>;

template <typename M> constexpr bool is_message = false;

template <Kind K, Registers::Field... Fs>
constexpr bool is_message<message<K, Fs...>> = true;

template <typename M>
concept Message = is_message<M>;

template <Message M> struct frame {
  Header header;
  M body;
};

enum class FrameError : uint8_t { Length, Control, Kind };

template <Message M>
constexpr std::array<std::byte, M::wire::bytes> encode(const Header &header,
                                                       const M &body) {
  return std::apply(
      [&header](auto... fs) {
        return M::wire::encode(DATA_FRAME, SEQUENCE{header.sequence},
                               DEST_PAN{header.pan},
                               DEST_ADDR{header.destination},
                               SRC_ADDR{header.source}, KIND{M::kind}, fs...);
      },
      body.fields);
}

template <Message M>
constexpr std::expected<frame<M>, FrameError>
decode(std::span<const std::byte> raw) {
  using Wire = typename M::wire;

  if (raw.size() != Wire::bytes) {
    return std::unexpected(FrameError::Length);
  }

  const auto values = Wire::decode(raw.first<Wire::bytes>());

  if (std::get<Registers::value<FRAME_CONTROL>>(values).word !=
      DATA_FRAME.word()) {
    return std::unexpected(FrameError::Control);
  }
  if (Wire::template get<KIND>(values) != M::kind) {
    return std::unexpected(FrameError::Kind);
  }

  return frame<M>{
      .header = {.sequence = Wire::template get<SEQUENCE>(values),
                 .pan = Wire::template get<DEST_PAN>(values),
                 .destination = Wire::template get<DEST_ADDR>(values),
                 .source = Wire::template get<SRC_ADDR>(values)},
      .body = M::from(values)};
}

// the filter's rules (manual 5.2.1) as they apply to this one frame shape, so
// software and FFEN agree on what a node hears
constexpr bool accepts(Identity self, const Header &header) {
  return (header.pan == self.pan || header.pan == ANY_PAN) &&
         (header.destination == self.address ||
          header.destination == BROADCAST);
}
} // namespace dw1000::mac

#endif
