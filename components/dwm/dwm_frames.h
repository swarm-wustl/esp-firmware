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

using FRAME_CONTROL =
    Registers::reg<Part{}, {.bytes = 2}, FRAME_TYPE, SECURITY, PENDING,
                   ACK_REQUEST, PAN_COMPRESS, DEST_MODE, FRAME_VERSION,
                   SRC_MODE>;

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

template <Registers::Field F>
using part = Registers::reg<Part{}, {.bytes = F::width / 8}, F>;

template <size_t N>
constexpr std::array<std::byte, N> octets(std::span<const std::byte, N> raw) {
  std::array<std::byte, N> out{};
  std::ranges::copy(raw, out.begin());
  return out;
}

template <typename T, typename... Ts>
constexpr size_t count_of =
    (size_t{0} + ... + std::same_as<T, std::remove_cvref_t<Ts>>);

// parts back to back in the order listed. encode takes one assignment per part
// in any order, and a part missing or given twice doesn't compile
template <Registers::Word... Rs> struct layout {
  static constexpr size_t bytes = (size_t{0} + ... + Rs::bytes);

  static constexpr std::array<size_t, sizeof...(Rs)> offsets = [] {
    constexpr std::array<size_t, sizeof...(Rs)> sizes{Rs::bytes...};
    std::array<size_t, sizeof...(Rs)> out{};
    std::exclusive_scan(sizes.begin(), sizes.end(), out.begin(), size_t{0});
    return out;
  }();

  using values = std::tuple<Registers::value<Rs>...>;

  template <typename... As>
    requires(sizeof...(As) == sizeof...(Rs) &&
             ((count_of<Registers::assignment<Rs>, As...> == 1) && ...))
  static constexpr std::array<std::byte, bytes> encode(As... assignments) {
    return place(std::tuple<As...>{assignments...},
                 std::index_sequence_for<Rs...>{});
  }

  static constexpr values decode(std::span<const std::byte, bytes> raw) {
    return slice(raw, std::index_sequence_for<Rs...>{});
  }

private:
  template <typename Given, size_t... Is>
  static constexpr std::array<std::byte, bytes>
  place(const Given &given, std::index_sequence<Is...>) {
    std::array<std::byte, bytes> out{};
    (std::ranges::copy(
         Registers::to_bytes<Rs>(std::get<Registers::assignment<Rs>>(given).word),
         out.begin() + offsets[Is]),
     ...);
    return out;
  }

  template <size_t... Is>
  static constexpr values slice(std::span<const std::byte, bytes> raw,
                                std::index_sequence<Is...>) {
    return {Registers::decode<Rs>(
        octets(raw.template subspan<offsets[Is], Rs::bytes>()))...};
  }
};

template <Registers::Field F, typename Values>
constexpr typename F::type unpack(const Values &values) {
  return std::get<Registers::value<part<F>>>(values).template get<F>();
}

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
  using wire = layout<FRAME_CONTROL, part<SEQUENCE>, part<DEST_PAN>,
                      part<DEST_ADDR>, part<SRC_ADDR>, part<KIND>, part<Fs>...>;

  std::tuple<Fs...> fields;

  constexpr message(Fs... fs) : fields{fs...} {}

  template <Registers::Field F>
    requires(std::same_as<F, Fs> || ...)
  constexpr typename F::type get() const {
    return std::get<F>(fields).value;
  }

  template <typename Values>
  static constexpr message from(const Values &values) {
    return message{Fs{unpack<Fs>(values)}...};
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
        return M::wire::encode(
            DATA_FRAME,
            Registers::set<part<SEQUENCE>>(SEQUENCE{header.sequence}),
            Registers::set<part<DEST_PAN>>(DEST_PAN{header.pan}),
            Registers::set<part<DEST_ADDR>>(DEST_ADDR{header.destination}),
            Registers::set<part<SRC_ADDR>>(SRC_ADDR{header.source}),
            Registers::set<part<KIND>>(KIND{M::kind}),
            Registers::set<part<decltype(fs)>>(fs)...);
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

  const auto values = Wire::decode(raw.template first<Wire::bytes>());

  if (std::get<Registers::value<FRAME_CONTROL>>(values).word !=
      DATA_FRAME.word) {
    return std::unexpected(FrameError::Control);
  }
  if (unpack<KIND>(values) != M::kind) {
    return std::unexpected(FrameError::Kind);
  }

  return frame<M>{.header = {.sequence = unpack<SEQUENCE>(values),
                             .pan = unpack<DEST_PAN>(values),
                             .destination = unpack<DEST_ADDR>(values),
                             .source = unpack<SRC_ADDR>(values)},
                  .body = M::from(values)};
}

// the filter's rules (manual 5.2.1) as they apply to this one frame shape, so
// software and FFEN agree on what a node hears
constexpr bool accepts(Identity self, const Header &header) {
  return (header.pan == self.pan || header.pan == ANY_PAN) &&
         (header.destination == self.address || header.destination == BROADCAST);
}
} // namespace dw1000::mac

#endif
