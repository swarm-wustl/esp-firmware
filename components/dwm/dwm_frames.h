#ifndef DWM_FRAMES_H
#define DWM_FRAMES_H

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>
#include <tuple>

#include "dwm_regs.h"
#include "register_layouts.h"
#include "registers.h"

// IEEE 802.15.4 data frames (manual 11) in the one shape the DW1000's frame
// filter matches against PANADR (manual 5.2.1): one PAN, short addresses at
// both ends
namespace dw1000::mac {
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
    Registers::reg<Registers::Offset{}, {.bytes = 2}, FRAME_TYPE, SECURITY,
                   PENDING, ACK_REQUEST, PAN_COMPRESS, DEST_MODE, FRAME_VERSION,
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
  using wire = Registers::layout<FRAME_CONTROL, SEQUENCE, DEST_PAN, DEST_ADDR, SRC_ADDR,
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

// TODO next steps for addressed ranging:
// 3. each board's identity: Kconfig short address + PAN, and an address ->
//    anchor_N frame rule in robot.h so uwb/range names the peer
// 4. DWM transmit/receive in terms of mac::encode / mac::decode<M>, replacing
//    the one-octet RANGE_POLL/RANGE_REPLY frames and the hand-rolled t_reply
//    loops in dwm.h; check idf.py size once frames are in the app
// 5. addressed range(anchor) / respond(): poll -> reply -> final, drop frames
//    that fail mac::accepts or carry the wrong sequence
// 6. [dwm_mock] tests pinning every octet of that exchange
// 7. bench: software filter first, then SYS_CFG FFEN + FFAD + RXAUTR (without
//    RXAUTR a rejected frame leaves the receiver idle)
