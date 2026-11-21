#ifndef DWM_H
#define DWM_H

#include "arrays.h"
#include "assembly.h"
#include "dwm_regs.h"
#include "peripheral_hal.h"
#include "registers.h"
#include "resources.h"
#include "system.h"
#include <algorithm>
#include <array>
#include <cstring>
#include <expected>
#include <span>
#include <string_view>
#include <utility>
#include <tuple>
#include <type_traits>

// TODO: add noexcept to classes

template <HAL::GenericSPIController SPI, HAL::GenericGPIOController GPIO,
          auto Pins, typename Bus>
class DWM {
public:
  static constexpr auto pins = Pins;

  using owns = Swarm::owns<SPI, GPIO>;
  using needs = Swarm::needs<Bus>;

  DWM(Swarm::Assembly<DWM> key, Bus &bus)
      : key_{key}, spi_{Swarm::make<SPI>(key, bus, Swarm::pin<"cs">(key))},
        gpio_{Swarm::make<GPIO>(key)} {
    hard_reset();
  }

  ~DWM() = default;
  DWM(const DWM &) = delete;
  void operator=(const DWM &) = delete;
  DWM(DWM &&) = delete;
  void operator=(DWM &&) = delete;

  enum class PreambleLength : uint8_t {
    LEN_64 = 0b01'00,
    LEN_128 = 0b01'01,
    LEN_256 = 0b01'10,
    LEN_512 = 0b01'11,
    LEN_1024 = 0b10'00,
    LEN_1536 = 0b10'01,
    LEN_2048 = 0b10'10,
    LEN_4096 = 0b11'00
  };

  static constexpr uint16_t PreambleLengthToUInt(PreambleLength pl) noexcept {
    switch (pl) {
    case PreambleLength::LEN_64:
      return 64;
    case PreambleLength::LEN_128:
      return 128;
    case PreambleLength::LEN_256:
      return 256;
    case PreambleLength::LEN_512:
      return 512;
    case PreambleLength::LEN_1024:
      return 1024;
    case PreambleLength::LEN_1536:
      return 1536;
    case PreambleLength::LEN_2048:
      return 2048;
    case PreambleLength::LEN_4096:
      return 4096;
    default:
      __builtin_unreachable();
    }

    return 0;
  }

  template <HAL::InterruptSignal Signal> void raise_on_irq(Signal &signal) {
    gpio_.raise_on_rising_edge(Swarm::pin<"irq">(key_), signal);
  }

  std::expected<uint32_t, HAL::SpiError> get_device_id() {
    return read<dw1000::Map::DEV_ID>().transform([](auto id) { return id.word; });
  }

  /*
   * Pulse Repetition Frequency
   */
  std::expected<PRF, HAL::SpiError> get_tx_prf() {
    return read<dw1000::Map::TX_FCTRL>().transform(
        [](auto fctrl) { return fctrl.template get<dw1000::TXPRF>(); });
  }

  std::expected<void, HAL::SpiError> set_tx_prf(PRF prf) {
    return modify<dw1000::Map::TX_FCTRL>(dw1000::TXPRF{prf});
  }

  // valid only once LDEDONE is set for the corresponding reception
  std::expected<dw1000::Timestamp, HAL::SpiError> get_rx_timestamp() {
    return read<dw1000::Map::RX_TIME>().transform(
        [](auto time) { return time.template get<dw1000::RX_STAMP>(); });
  }

  std::expected<dw1000::Timestamp, HAL::SpiError> get_tx_timestamp() {
    return read<dw1000::Map::TX_TIME>().transform(
        [](auto time) { return time.template get<dw1000::TX_STAMP>(); });
  }

  template <Registers::at<dw1000::Address> R>
    requires Registers::Word<R>
  [[nodiscard]] std::expected<Registers::value<R>, HAL::SpiError> read() {
    std::array<std::byte, R::bytes> rx{};

    return spi_.transfer_halfduplex(dw1000::header<R::address>(false), rx)
        .transform([&rx] { return Registers::decode<R>(rx); });
  }

  template <Registers::at<dw1000::Address> R, Registers::Field... Fs>
    requires Registers::writable_with<R, Fs...>
  [[nodiscard]] std::expected<void, HAL::SpiError> write(Fs... fs) {
    return spi_.transfer_halfduplex(
        Registers::frame<R>(dw1000::header<R::address>(true), fs...), {});
  }

  template <Registers::at<dw1000::Address> R>
  [[nodiscard]] std::expected<void, HAL::SpiError>
  write(Registers::assignment<R> assignment) {
    return spi_.transfer_halfduplex(
        Registers::frame<R>(dw1000::header<R::address>(true), assignment), {});
  }

  // two SPI transactions: the read is what keeps the fields it doesn't name
  template <Registers::at<dw1000::Address> R, Registers::Field... Fs>
    requires Registers::modifiable_with<R, Fs...>
  [[nodiscard]] std::expected<void, HAL::SpiError> modify(Fs... fs) {
    return read<R>().and_then([&](Registers::value<R> current) {
      const Registers::value<R> next{static_cast<typename R::word_type>(
          Registers::modify_word<R>(current.word, fs...))};

      return spi_.transfer_halfduplex(
          Registers::frame<R>(dw1000::header<R::address>(true), next), {});
    });
  }

  // a block is bytes, not fields: the span's extent is its length, so a
  // buffer longer than the register doesn't compile
  template <Registers::at<dw1000::Address> R, typename Byte, size_t N>
    requires Registers::Block<R> &&
             (R::access != Registers::Access::ReadOnly) &&
             std::same_as<std::remove_const_t<Byte>, std::byte> &&
             (N != std::dynamic_extent) && (N <= R::bytes)
  [[nodiscard]] std::expected<void, HAL::SpiError>
  write(std::span<Byte, N> data) {
    constexpr auto header = dw1000::header<R::address>(true);

    std::array<std::byte, header.size() + N> tx{};
    std::ranges::copy(header, tx.begin());
    std::ranges::copy(data, tx.begin() + header.size());

    return spi_.transfer_halfduplex(tx, {});
  }

  // the first `count` octets, capped at the buffer: a count that only arrives
  // at runtime (a received frame's length) can't overrun what the extent
  // already bounded
  template <Registers::at<dw1000::Address> R, size_t N>
    requires Registers::Block<R> && (N != std::dynamic_extent) &&
             (N <= R::bytes)
  [[nodiscard]] std::expected<void, HAL::SpiError>
  read(std::span<std::byte, N> out, size_t count = N) {
    return spi_.transfer_halfduplex(dw1000::header<R::address>(false),
                                    out.first(std::min(count, N)));
  }

  /*
   * Full device init for the default mode (channel 5, 16 MHz PRF, preamble 128,
   * 6.8 Mbps). Order matters: LDE microcode must be loaded from OTP before the
   * receiver is used, or RX timestamps are garbage (manual 2.5.5.10).
   */
  std::expected<void, HAL::SpiError> configure() {
    hard_reset();
    return load_lde()
        .and_then([this] { return write_all(dw1000::DEFAULT_CONFIG); })
        .and_then([this] { return set_tx_prf(PRF::MHZ_16); })
        .and_then([this] { return set_tx_bit_rate(BitRate::MBPS_68); })
        .and_then(
            [this] { return set_tx_preamble_length(PreambleLength::LEN_128); });
  }

  // Write payload to TX_BUFFER, set the frame length, start TX, wait for TXFRS.
  // TFLEN counts the payload and its 2-octet FCS
  template <size_t N>
    requires(N + 2 < (1u << dw1000::TFLEN::width))
  std::expected<void, HAL::SpiError>
  transmit(const std::array<std::byte, N> &payload) {
    uint16_t frame_len = static_cast<uint16_t>(payload.size() + 2); // +2 FCS
    // abort any in-progress RX/TX first, else a stuck transceiver ignores
    // TXSTRT
    return force_idle()
        .and_then(
            [&] { return write<dw1000::Map::TX_BUFFER>(std::span{payload}); })
        .and_then([&] {
          return modify<dw1000::Map::TX_FCTRL>(
              dw1000::TFLEN{static_cast<uint8_t>(frame_len)});
        })
        .and_then([this] { return write<dw1000::Map::SYS_CTRL>(dw1000::TXSTRT{true}); })
        .and_then([this] { return poll_status<dw1000::TXFRS>(10); })
        .and_then([this] { return clear_status<dw1000::TXFRS>(); });
  }

  std::expected<void, HAL::SpiError> start_receive() {
    return force_idle().and_then(
        [this] { return write<dw1000::Map::SYS_CTRL>(dw1000::RXENAB{true}); });
  }

  // Enable RX, wait for a good frame, read it into `out`. Returns payload
  // length (FCS stripped), capped to out.size(). raw SYS_STATUS, for bring-up
  // diagnostics
  std::expected<uint64_t, HAL::SpiError> read_sys_status() {
    return read_status();
  }

  template <size_t N>
    requires(N <= dw1000::Map::RX_BUFFER::bytes)
  std::expected<size_t, HAL::SpiError> receive(std::array<std::byte, N> &out,
                                               int timeout_ms = 100) {
    // clear any stale RX event bits first, else a leftover RXFCG makes
    // poll_status return immediately with no real frame
    return clear_status<dw1000::RXFCG, dw1000::RXDFR>(dw1000::RxErrors{})
        .and_then([this] { return start_receive(); })
        .and_then([&]() -> std::expected<size_t, HAL::SpiError> {
          if (auto r = poll_status<dw1000::RXFCG>(timeout_ms); !r) {
            return std::unexpected(r.error());
          }

          auto finfo = read<dw1000::Map::RX_FINFO>();
          if (!finfo) {
            return std::unexpected(finfo.error());
          }
          const size_t len = frame_length(*finfo);

          size_t payload = len >= 2 ? len - 2u : 0;
          size_t n = std::min(payload, out.size());
          if (auto d = read<dw1000::Map::RX_BUFFER>(std::span{out}, n); !d) {
            return std::unexpected(d.error());
          }
          if (auto c = clear_status<dw1000::RXFCG, dw1000::RXDFR>(); !c) {
            return std::unexpected(c.error());
          }
          return n;
        });
  }

  /*
   * Single-sided two-way ranging, initiator side. Sends a poll, receives the
   * responder's reply-time, and returns distance in meters.
   *   t_round = rx(reply) - tx(poll)          [measured here]
   *   t_reply = tx(reply) - rx(poll)          [measured by responder, sent
   * back] tof     = (t_round - t_reply) / 2
   */
  // TODO: split this function up into pure parts (I/O, decode, math)
  // also have it read cleaner with monadic chaining where applicable
  std::expected<double, HAL::SpiError> range() {
    std::array<std::byte, 1> poll{RANGE_POLL};
    if (auto r = transmit(poll); !r) {
      return std::unexpected(r.error());
    }
    auto t_poll_tx = get_tx_timestamp();
    if (!t_poll_tx) {
      return std::unexpected(t_poll_tx.error());
    }

    // timing reply: its rx timestamp is t_round's end
    std::array<std::byte, 1> reply{};
    if (auto n = receive(reply, 20); !n) {
      return std::unexpected(n.error());
    }
    auto t_reply_rx = get_rx_timestamp();
    if (!t_reply_rx) {
      return std::unexpected(t_reply_rx.error());
    }

    // final frame carries the responder's t_reply (8 LE bytes after the type)
    std::array<std::byte, 9> final_frame{};
    if (auto n = receive(final_frame, 20); !n) {
      return std::unexpected(n.error());
    }
    uint64_t t_reply = 0;
    for (int i = 0; i < 8; ++i) {
      t_reply |=
          static_cast<uint64_t>(std::to_integer<uint8_t>(final_frame[1 + i]))
          << (8 * i);
    }

    uint64_t t_round = (*t_reply_rx - *t_poll_tx).count();
    double tof =
        (static_cast<double>(t_round) - static_cast<double>(t_reply)) / 2.0;
    return tof * SECONDS_PER_TICK * SPEED_OF_LIGHT;
  }

  // Responder side: wait for a poll, send a timing reply, then a final frame
  // carrying t_reply = tx(reply) - rx(poll).
  std::expected<void, HAL::SpiError> respond(int timeout_ms = 1000) {
    std::array<std::byte, 1> poll{};
    if (auto n = receive(poll, timeout_ms); !n) {
      return std::unexpected(n.error());
    }
    auto t_poll_rx = get_rx_timestamp();
    if (!t_poll_rx) {
      return std::unexpected(t_poll_rx.error());
    }

    std::array<std::byte, 1> reply{RANGE_REPLY};
    if (auto r = transmit(reply); !r) {
      return std::unexpected(r.error());
    }
    auto t_reply_tx = get_tx_timestamp();
    if (!t_reply_tx) {
      return std::unexpected(t_reply_tx.error());
    }

    uint64_t t_reply = (*t_reply_tx - *t_poll_rx).count();
    std::array<std::byte, 9> final_frame{RANGE_REPLY};
    for (int i = 0; i < 8; ++i) {
      final_frame[1 + i] = std::byte((t_reply >> (8 * i)) & 0xFF);
    }
    return transmit(final_frame);
  }

private:
  static constexpr double SECONDS_PER_TICK = 1.0 / 63'897'600'000.0;
  static constexpr double SPEED_OF_LIGHT = 299'792'458.0;
  static constexpr std::byte RANGE_POLL{0x01};
  static constexpr std::byte RANGE_REPLY{0x02};

  std::expected<uint64_t, HAL::SpiError> read_status() {
    return read<dw1000::Map::SYS_STATUS>().transform([](auto status) {
      return static_cast<uint64_t>(status.word);
    });
  }

  // RXFLE extends RXFLEN past the standard 127 octets
  static constexpr size_t frame_length(Registers::value<dw1000::Map::RX_FINFO> finfo) {
    return (static_cast<size_t>(finfo.template get<dw1000::RXFLE>()) << 7) |
           finfo.template get<dw1000::RXFLEN>();
  }

  template <typename... Es>
  static constexpr bool any_of(Registers::value<dw1000::Map::SYS_STATUS> status,
                               std::tuple<Es...>) {
    return (status.template get<Es>() || ...);
  }

  // SYS_STATUS is write-1-to-clear; our event bits live in the low 4 octets
  template <typename... Fs, typename... Es>
  std::expected<void, HAL::SpiError> clear_status(std::tuple<Es...> = {}) {
    return write<dw1000::Map::SYS_STATUS>(Fs{true}..., Es{true}...);
  }

  // abort any in-progress TX/RX and return the transceiver to IDLE
  std::expected<void, HAL::SpiError> force_idle() {
    return write<dw1000::Map::SYS_CTRL>(dw1000::TRXOFF{true});
  }

  // poll SYS_STATUS until `mask` is set, an RX error appears, or we time out
  // TODO: distinct DWMError for timeout / rx-error vs a genuine SPI failure
  template <typename Event>
  std::expected<void, HAL::SpiError> poll_status(int timeout_ms) {
    for (int i = 0; i < timeout_ms; ++i) {
      auto s = read<dw1000::Map::SYS_STATUS>();
      if (!s) {
        return std::unexpected(s.error());
      }
      if (s->template get<Event>()) {
        return {};
      }
      if (any_of(*s, dw1000::RxErrors{})) {
        return std::unexpected(HAL::SpiError::TransferFailed);
      }
      gpio_.delay_ms(1);
    }
    return std::unexpected(HAL::SpiError::Timeout);
  }

  // manual 2.5.5.10 Table 4: force sys clock, kick OTP->LDE, restore clock
  std::expected<void, HAL::SpiError> load_lde() {
    return write<dw1000::Map::PMSC_CTRL0>(dw1000::SYSCLKS{dw1000::SysClock::Xti},
                                  dw1000::LDE_LOAD_CLKS{0b11})
        .and_then([this] { return write<dw1000::Map::OTP_CTRL>(dw1000::LDELOAD{true}); })
        .and_then([this]() -> std::expected<void, HAL::SpiError> {
          gpio_.delay_ms(1); // >= 150 us
          return {};
        })
        .and_then([this] { return write<dw1000::Map::PMSC_CTRL0>(); });
  }

  // in order, stopping at the first write that fails
  template <typename... Rs>
  std::expected<void, HAL::SpiError>
  write_all(const std::tuple<Registers::assignment<Rs>...> &table) {
    return std::apply(
        [this](const auto &...entries) {
          std::expected<void, HAL::SpiError> result{};
          ((result = write(entries), result.has_value()) && ...);
          return result;
        },
        table);
  }

  void hard_reset() {
    const HAL::Pin reset = Swarm::pin<"reset">(key_);

    gpio_.set_direction(reset, HAL::PinMode::Output);
    gpio_.set_level(reset, HAL::Voltage::LOW);
    gpio_.delay_ms(10);
    gpio_.set_level(reset, HAL::Voltage::HIGH);
    gpio_.delay_ms(10);
  }

  std::expected<BitRate, HAL::SpiError> get_tx_bit_rate() {
    return read<dw1000::Map::TX_FCTRL>().transform(
        [](auto fctrl) { return fctrl.template get<dw1000::TXBR>(); });
  }

  std::expected<void, HAL::SpiError> set_tx_bit_rate(BitRate br) {
    return modify<dw1000::Map::TX_FCTRL>(dw1000::TXBR{br});
  }

  std::expected<uint16_t, HAL::SpiError> get_tx_preamble_length() {
    return read<dw1000::Map::TX_FCTRL>().transform([](auto fctrl) {
      uint8_t raw_psr = fctrl.template get<dw1000::TXPSR>();
      uint8_t raw_pe = fctrl.template get<dw1000::PE>();
      uint8_t psr_pe_combined = (raw_psr << 2) | raw_pe;

      return PreambleLengthToUInt(static_cast<PreambleLength>(psr_pe_combined));
    });
  }

  std::expected<void, HAL::SpiError> set_tx_preamble_length(PreambleLength pl) {
    uint8_t psr_pe_combined = static_cast<uint8_t>(pl);
    uint8_t raw_psr = (psr_pe_combined >> 2) & 0b11;
    uint8_t raw_pe = psr_pe_combined & 0b11;

    return modify<dw1000::Map::TX_FCTRL>(dw1000::TXPSR{raw_psr}, dw1000::PE{raw_pe});
  }

  [[no_unique_address]] Swarm::Assembly<DWM> key_;
  SPI spi_;
  GPIO gpio_;
};

#endif
