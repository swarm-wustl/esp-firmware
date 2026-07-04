#ifndef DWM_H
#define DWM_H

#include "dwm_data.h"
#include "dwm_regs.h"
#include "peripheral_hal.h"
#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cstring>
#include <expected>
#include <span>
#include <string_view>
#include <utility>
#include <vector>

// TODO: add noexcept to classes

enum class DWMRegisterID : uint8_t {
  DEV_ID = 0x00,
  TX_FCTRL = 0x08,
  SYS_TIME = 0x06,
  SYSTEM_EVENT_STATUS = 0x0F,
  RX_TIME = 0x15,
  TX_TIME = 0x17,
  TX_BUFFER = 0x09,
};

class DWMTimestamp {
  // full 40-bit counter. do NOT drop the low bits: the sub-nanosecond flight
  // time that ranging measures lives in them (1 m ~= 213 of these ~15.65 ps
  // ticks, and the low 9 bits span ~8 ns). masking them zeroes the measurement.
  static constexpr uint64_t DW1000_40BIT_MASK = 0xFF'FF'FF'FF'FFULL;

public:
  using Duration = std::chrono::duration<
      uint64_t, std::ratio<1, 63'897'600'000>>; // each bit = ~15.65 ps

  DWMTimestamp() = default;
  DWMTimestamp(uint64_t raw_time) : raw_time_{raw_time & DW1000_40BIT_MASK} {}

  Duration operator-(const DWMTimestamp &other) const {
    return Duration{(raw_time_ - other.raw_time_) & DW1000_40BIT_MASK};
  }

private:
  uint64_t raw_time_{};
};

// TODO: consider moving this into RegisterInfo
template <DWMRegisterID ID>
concept IsTimestampRegister =
    ID == DWMRegisterID::SYS_TIME || ID == DWMRegisterID::TX_TIME ||
    ID == DWMRegisterID::RX_TIME;

enum class RegAccess : uint8_t { ReadOnly, ReadWrite, WriteOneClear };

template <DWMRegisterID ID> struct RegisterInfo;
template <> struct RegisterInfo<DWMRegisterID::DEV_ID> {
  static constexpr size_t size = 4;
  static constexpr RegAccess access = RegAccess::ReadOnly;
};
template <> struct RegisterInfo<DWMRegisterID::SYSTEM_EVENT_STATUS> {
  static constexpr size_t size = 5;
  static constexpr RegAccess access = RegAccess::WriteOneClear;
};
template <> struct RegisterInfo<DWMRegisterID::SYS_TIME> {
  static constexpr size_t size = 5;
  static constexpr RegAccess access = RegAccess::ReadOnly;
};
template <> struct RegisterInfo<DWMRegisterID::RX_TIME> {
  // 5 = the RX_STAMP subfield at offset 0, not the full 14-octet register
  static constexpr size_t size = 5;
  static constexpr RegAccess access = RegAccess::ReadOnly;
};
template <> struct RegisterInfo<DWMRegisterID::TX_TIME> {
  // 5 = the TX_STAMP subfield at offset 0, not the full 10-octet register
  static constexpr size_t size = 5;
  static constexpr RegAccess access = RegAccess::ReadOnly;
};
template <> struct RegisterInfo<DWMRegisterID::TX_FCTRL> {
  static constexpr size_t size = 5;
  static constexpr RegAccess access = RegAccess::ReadWrite;
};
template <> struct RegisterInfo<DWMRegisterID::TX_BUFFER> {
  static constexpr size_t size = 1024;
  static constexpr RegAccess access = RegAccess::ReadWrite;
};

template <DWMRegisterID ID>
concept IsWritable = RegisterInfo<ID>::access != RegAccess::ReadOnly;
template <DWMRegisterID ID>
concept IsReadWrite = RegisterInfo<ID>::access == RegAccess::ReadWrite;
template <DWMRegisterID ID>
concept IsWriteOneClear = RegisterInfo<ID>::access == RegAccess::WriteOneClear;

template <HAL::GenericSPIController SPI, DWMRegisterID ID>
class DWMRegisterView {
  static constexpr size_t size_ = RegisterInfo<ID>::size;

  // static so ValueType can decltype the result before the class is complete
  static auto interpret(const DWMData<size_> &data) {
    if constexpr (size_ <= sizeof(uint64_t)) {
      if constexpr (IsTimestampRegister<ID>) {
        return DWMTimestamp{data.to_uint()};
      } else {
        return data.to_uint();
      }
    } else {
      return data.span();
    }
  }

public:
  // return type of an object returned from read(), derived from interpret()
  // based on regtype, fully known at comptime
  using ValueType = decltype(interpret(std::declval<const DWMData<size_> &>()));

  // TODO: constructor that takes in data?
  explicit DWMRegisterView(SPI &spi) : spi_{spi} {}

  ~DWMRegisterView() = default;

  DWMRegisterView(const DWMRegisterView &other) = default;
  DWMRegisterView &operator=(const DWMRegisterView &) = default;

  DWMRegisterView(DWMRegisterView &&) = delete;
  void operator=(DWMRegisterView &&) = delete;

  [[nodiscard]] std::expected<ValueType, HAL::SpiError> read() {
    return read_into_cache().transform([this] { return interpret(data_); });
  }

  [[nodiscard]] std::expected<void, HAL::SpiError> operator|=(uint64_t flags)
    requires IsWritable<ID> && (size_ <= sizeof(uint64_t))
  {
    if constexpr (IsWriteOneClear<ID>) {
      // writing a 1 clears the bit, so write flags directly with no read-back
      return write_data(flags);
    } else {
      return read_into_cache().and_then(
          [&] { return write_data(data_.to_uint() | flags); });
    }
  }

  [[nodiscard]] std::expected<void, HAL::SpiError> operator&=(uint64_t flags)
    requires IsReadWrite<ID> && (size_ <= sizeof(uint64_t))
  {
    return read_into_cache().and_then(
        [&] { return write_data(data_.to_uint() & flags); });
  }

  // TODO
  [[nodiscard]] std::expected<void, HAL::SpiError> operator+=(uint64_t)
    requires IsTimestampRegister<ID>
  {
    return {};
  }

  [[nodiscard]] std::expected<void, HAL::SpiError>
  write_bit_range(uint8_t hi, uint8_t lo, uint64_t value)
    requires IsReadWrite<ID> && (size_ <= sizeof(uint64_t))
  {
    return read_into_cache().and_then([&] {
      DWMData<size_> new_data{data_};
      new_data.write_bit_range(hi, lo, value);
      return write_data(new_data.span());
    });
  }

  consteval size_t size() const { return size_; }

  // TODO: consider removing this and other cases of std::integral auto
  // It might just be adding complexity for no reason (ig bit_cast
  // optimization..?)
  [[nodiscard]] std::expected<void, HAL::SpiError>
  write_data(std::integral auto new_value)
    requires IsWritable<ID> && (size_ <= sizeof(uint64_t))
  {
    return write_data(DWMData<size_>{new_value}.span());
  }

  // TODO: maybe make the span have a dynamic_extent, and allow writes <= size_?
  [[nodiscard]] std::expected<void, HAL::SpiError>
  write_data(std::span<const std::byte, size_> new_data)
    requires IsWritable<ID>
  {
    // header: MSbit = 1 for write, lower 6 bits = register id
    uint8_t reg = 0x80 | (static_cast<uint8_t>(ID) & 0x3F);

    std::array<std::byte, size_ + 1> tx{};
    tx[0] = std::byte{reg};
    std::ranges::copy(new_data, tx.begin() + 1);

    return spi_.transfer_halfduplex(tx, {}).transform([&] {
      // WOC registers must be re-read instead of trusting the written value
      if constexpr (IsReadWrite<ID>) {
        std::ranges::copy(new_data, data_.span().begin());
      }
    });
  }

private:
  SPI &spi_;
  DWMData<size_> data_{};

  std::expected<void, HAL::SpiError> read_into_cache() {
    // header: MSbit = 0 for read, lower 6 bits = register id
    uint8_t reg = 0x00 | (static_cast<uint8_t>(ID) & 0x3F);

    std::array<const std::byte, 1> tx{std::byte{reg}};
    return spi_.transfer_halfduplex(tx, data_.span());
  }
};

// TODO: add this, and other related classes, to some sort of DWM namespace
enum class PRF : uint8_t { MHZ_4 = 0b00, MHZ_16 = 0b01, MHZ_64 = 0b10 };

constexpr std::string_view PRFToString(PRF prf) noexcept {
  using namespace std::string_view_literals;

  switch (prf) {
  case PRF::MHZ_4:
    return "4 MHz"sv;
  case PRF::MHZ_16:
    return "16 MHz"sv;
  case PRF::MHZ_64:
    return "64 MHz"sv;
  default:
    __builtin_unreachable();
  }

  return "UNKNOWN PRF"sv;
}

template <HAL::GenericSPIController SPI, HAL::GenericGPIOController GPIO>
class DWM {
  static_assert(std::endian::native == std::endian::little,
                "DWM1000 requires little-endian architecture");

public:
  // TODO: make GPIO rvalue ref?
  DWM(SPI &&spi, GPIO gpio, uint8_t rst_pin, uint8_t irq_pin)
      : spi_{std::move(spi)}, gpio_{std::move(gpio)}, rst_pin_{rst_pin},
        irq_pin_{irq_pin} {
    hard_reset();
  }

  ~DWM() = default;
  DWM(const DWM &) = delete;
  void operator=(const DWM &) = delete;
  DWM(DWM &&) = delete;
  void operator=(DWM &&) = delete;

  enum class BitRate : uint8_t {
    KBPS_100 = 0b00,
    KBPS_850 = 0b01,
    MBPS_68 = 0b10
  };

  static constexpr std::string_view BitRateToString(BitRate br) noexcept {
    using namespace std::string_view_literals; // Allows for ""sv suffix

    switch (br) {
    case BitRate::KBPS_100:
      return "110 kbps"sv;
    case BitRate::KBPS_850:
      return "850 kbps"sv;
    case BitRate::MBPS_68:
      return "6.8 Mbps"sv;
    default:
      __builtin_unreachable();
    }

    return "UNKNOWN BITRATE"sv;
  }

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

  std::expected<uint32_t, HAL::SpiError> get_device_id() {
    return get_reg_view<DWMRegisterID::DEV_ID>().read().transform(
        [](uint64_t raw) { return static_cast<uint32_t>(raw); });
  }

  /*
   * Pulse Repetition Frequency
   */
  std::expected<PRF, HAL::SpiError> get_tx_prf() {
    return get_reg_view<DWMRegisterID::TX_FCTRL>().read().transform(
        [](uint64_t raw) {
          return static_cast<PRF>((raw >> 16) & 0b11); // TODO: constants?
        });
  }

  std::expected<void, HAL::SpiError> set_tx_prf(PRF prf) {
    return get_reg_view<DWMRegisterID::TX_FCTRL>().write_bit_range(
        17, 16, static_cast<uint64_t>(prf));
  }

  // valid only once LDEDONE is set for the corresponding reception
  std::expected<DWMTimestamp, HAL::SpiError> get_rx_timestamp() {
    return get_reg_view<DWMRegisterID::RX_TIME>().read();
  }

  std::expected<DWMTimestamp, HAL::SpiError> get_tx_timestamp() {
    return get_reg_view<DWMRegisterID::TX_TIME>().read();
  }

  /*
   * Full device init for the default mode (channel 5, 16 MHz PRF, preamble 128,
   * 6.8 Mbps). Order matters: LDE microcode must be loaded from OTP before the
   * receiver is used, or RX timestamps are garbage (manual 2.5.5.10).
   */
  std::expected<void, HAL::SpiError> configure() {
    hard_reset();
    return load_lde()
        .and_then([this] { return write_config_table(); })
        .and_then([this] {
          return write_sub_value(dw1000::CHAN_CTRL, 0, dw1000::CHAN_CTRL_VALUE, 4);
        })
        .and_then([this] {
          return write_sub_value(dw1000::TX_ANTD, 0, dw1000::ANTENNA_DELAY, 2);
        })
        .and_then([this] {
          // RX antenna delay (LDE_RXANTD, 0x2E:1804) -- balances TX_ANTD, else
          // ~half the antenna delay stays uncompensated as a fixed offset
          return write_sub_value(0x2E, 0x1804, dw1000::ANTENNA_DELAY, 2);
        })
        .and_then([this] { return set_tx_prf(PRF::MHZ_16); })
        .and_then([this] { return set_tx_bit_rate(BitRate::MBPS_68); })
        .and_then([this] { return set_tx_preamble_length(PreambleLength::LEN_128); });
  }

  // Write payload to TX_BUFFER, set the frame length, start TX, wait for TXFRS.
  std::expected<void, HAL::SpiError> transmit(std::span<const std::byte> payload) {
    uint16_t frame_len = static_cast<uint16_t>(payload.size() + 2); // +2 FCS
    auto tx_fctrl = get_reg_view<DWMRegisterID::TX_FCTRL>();
    // abort any in-progress RX/TX first, else a stuck transceiver ignores TXSTRT
    return force_idle()
        .and_then([&] { return write_sub(dw1000::TX_BUFFER, 0, payload); })
        .and_then([&] { return tx_fctrl.write_bit_range(6, 0, frame_len); })
        .and_then([this] {
          return write_sub_value(dw1000::SYS_CTRL, 0, dw1000::TXSTRT, 1);
        })
        .and_then([this] { return poll_status(dw1000::TXFRS, 10); })
        .and_then([this] { return clear_status(dw1000::TXFRS); });
  }

  std::expected<void, HAL::SpiError> start_receive() {
    return force_idle().and_then([this] {
      return write_sub_value(dw1000::SYS_CTRL, 0, dw1000::RXENAB, 2);
    });
  }

  // Enable RX, wait for a good frame, read it into `out`. Returns payload length
  // (FCS stripped), capped to out.size().
  // raw SYS_STATUS, for bring-up diagnostics
  std::expected<uint64_t, HAL::SpiError> read_sys_status() {
    return read_status();
  }

  std::expected<size_t, HAL::SpiError> receive(std::span<std::byte> out,
                                               int timeout_ms = 100) {
    // clear any stale RX event bits first, else a leftover RXFCG makes
    // poll_status return immediately with no real frame
    return clear_status(dw1000::RXFCG | dw1000::RXDFR | dw1000::RX_ERROR)
        .and_then([this] { return start_receive(); })
        .and_then([&]() -> std::expected<size_t, HAL::SpiError> {
          if (auto r = poll_status(dw1000::RXFCG, timeout_ms); !r) {
            return std::unexpected(r.error());
          }

          std::array<std::byte, 4> finfo{};
          if (auto f = read_sub(dw1000::RX_FINFO, 0, finfo); !f) {
            return std::unexpected(f.error());
          }
          uint16_t len = static_cast<uint16_t>(
              ((std::to_integer<uint16_t>(finfo[1]) << 8) |
               std::to_integer<uint16_t>(finfo[0])) &
              0x03FF);

          size_t payload = len >= 2 ? len - 2u : 0;
          size_t n = std::min(payload, out.size());
          if (auto d = read_sub(dw1000::RX_BUFFER, 0, out.first(n)); !d) {
            return std::unexpected(d.error());
          }
          if (auto c = clear_status(dw1000::RXFCG | dw1000::RXDFR); !c) {
            return std::unexpected(c.error());
          }
          return n;
        });
  }

  /*
   * Single-sided two-way ranging, initiator side. Sends a poll, receives the
   * responder's reply-time, and returns distance in meters.
   *   t_round = rx(reply) - tx(poll)          [measured here]
   *   t_reply = tx(reply) - rx(poll)          [measured by responder, sent back]
   *   tof     = (t_round - t_reply) / 2
   */
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
  template <DWMRegisterID ID> using Register = DWMRegisterView<SPI, ID>;

  template <DWMRegisterID ID> Register<ID> get_reg_view() {
    return Register<ID>{spi_};
  }

  static constexpr double SECONDS_PER_TICK = 1.0 / 63'897'600'000.0;
  static constexpr double SPEED_OF_LIGHT = 299'792'458.0;
  static constexpr std::byte RANGE_POLL{0x01};
  static constexpr std::byte RANGE_REPLY{0x02};

  // sub-addressed SPI header (manual 2.2.1.2): 1 octet non-indexed, 2 for an
  // offset <= 0x7F, 3 with the extended-address flag for larger offsets
  static uint8_t make_header(std::array<std::byte, 3> &hdr, bool write,
                             uint8_t reg, uint16_t offset) {
    uint8_t b0 = (write ? 0x80 : 0x00) | (reg & 0x3F) | (offset ? 0x40 : 0x00);
    hdr[0] = std::byte{b0};
    if (offset == 0) {
      return 1;
    }
    if (offset <= 0x7F) {
      hdr[1] = std::byte(offset & 0x7F);
      return 2;
    }
    hdr[1] = std::byte(0x80 | (offset & 0x7F));
    hdr[2] = std::byte((offset >> 7) & 0xFF);
    return 3;
  }

  std::expected<void, HAL::SpiError>
  write_sub(uint8_t reg, uint16_t offset, std::span<const std::byte> data) {
    std::array<std::byte, 3> hdr{};
    uint8_t hlen = make_header(hdr, true, reg, offset);
    std::vector<std::byte> tx(hlen + data.size());
    std::copy_n(hdr.begin(), hlen, tx.begin());
    std::ranges::copy(data, tx.begin() + hlen);
    return spi_.transfer_halfduplex(tx, {});
  }

  std::expected<void, HAL::SpiError>
  read_sub(uint8_t reg, uint16_t offset, std::span<std::byte> out) {
    std::array<std::byte, 3> hdr{};
    uint8_t hlen = make_header(hdr, false, reg, offset);
    return spi_.transfer_halfduplex(std::span<const std::byte>{hdr.data(), hlen},
                                    out);
  }

  std::expected<void, HAL::SpiError>
  write_sub_value(uint8_t reg, uint16_t offset, uint32_t value, uint8_t size) {
    std::array<std::byte, 4> bytes{};
    for (uint8_t i = 0; i < size; ++i) {
      bytes[i] = std::byte((value >> (8 * i)) & 0xFF);
    }
    return write_sub(reg, offset, std::span<const std::byte>{bytes.data(), size});
  }

  std::expected<uint64_t, HAL::SpiError> read_status() {
    std::array<std::byte, 5> b{};
    return read_sub(dw1000::SYS_STATUS, 0, b).transform([&] {
      uint64_t v = 0;
      for (int i = 0; i < 5; ++i) {
        v |= static_cast<uint64_t>(std::to_integer<uint8_t>(b[i])) << (8 * i);
      }
      return v;
    });
  }

  // SYS_STATUS is write-1-to-clear; our event bits live in the low 4 octets
  std::expected<void, HAL::SpiError> clear_status(uint32_t bits) {
    return write_sub_value(dw1000::SYS_STATUS, 0, bits, 4);
  }

  // abort any in-progress TX/RX and return the transceiver to IDLE
  std::expected<void, HAL::SpiError> force_idle() {
    return write_sub_value(dw1000::SYS_CTRL, 0, dw1000::TRXOFF, 1);
  }

  // poll SYS_STATUS until `mask` is set, an RX error appears, or we time out
  // TODO: distinct DWMError for timeout / rx-error vs a genuine SPI failure
  std::expected<void, HAL::SpiError> poll_status(uint32_t mask, int timeout_ms) {
    for (int i = 0; i < timeout_ms; ++i) {
      auto s = read_status();
      if (!s) {
        return std::unexpected(s.error());
      }
      if (*s & mask) {
        return {};
      }
      if (*s & dw1000::RX_ERROR) {
        return std::unexpected(HAL::SpiError::TransferFailed);
      }
      gpio_.delay_ms(1);
    }
    return std::unexpected(HAL::SpiError::Timeout);
  }

  // manual 2.5.5.10 Table 4: force sys clock, kick OTP->LDE, restore clock
  std::expected<void, HAL::SpiError> load_lde() {
    return write_sub_value(dw1000::PMSC, 0x00, 0x0301, 2)
        .and_then(
            [this] { return write_sub_value(dw1000::OTP_IF, 0x06, 0x8000, 2); })
        .and_then([this]() -> std::expected<void, HAL::SpiError> {
          gpio_.delay_ms(1); // >= 150 us
          return {};
        })
        .and_then(
            [this] { return write_sub_value(dw1000::PMSC, 0x00, 0x0200, 2); });
  }

  std::expected<void, HAL::SpiError> write_config_table() {
    for (const auto &w : dw1000::DEFAULT_CONFIG) {
      if (auto r = write_sub_value(w.reg, w.offset, w.value, w.size); !r) {
        return r;
      }
    }
    return {};
  }

  void hard_reset() {
    gpio_.set_direction(rst_pin_, HAL::PinMode::Output);
    gpio_.set_level(rst_pin_, HAL::Voltage::LOW);
    gpio_.delay_ms(10);
    gpio_.set_level(rst_pin_, HAL::Voltage::HIGH);
    gpio_.delay_ms(10);
  }

  std::expected<std::string_view, HAL::SpiError> get_tx_bit_rate() {
    return get_reg_view<DWMRegisterID::TX_FCTRL>().read().transform(
        [](uint64_t raw) {
          return BitRateToString(
              static_cast<BitRate>((raw >> 13) & 0b11)); // TODO: constants?
        });
  }

  std::expected<void, HAL::SpiError> set_tx_bit_rate(BitRate br) {
    return get_reg_view<DWMRegisterID::TX_FCTRL>().write_bit_range(
        14, 13, static_cast<uint64_t>(br));
  }

  std::expected<uint16_t, HAL::SpiError> get_tx_preamble_length() {
    return get_reg_view<DWMRegisterID::TX_FCTRL>().read().transform(
        [](uint64_t raw) {
          uint8_t raw_psr = (raw >> 18) & 0b11; // TODO: constants?
          uint8_t raw_pe = (raw >> 20) & 0b11;  // TODO: constants?
          uint8_t psr_pe_combined = (raw_psr << 2) | raw_pe;

          return PreambleLengthToUInt(
              static_cast<PreambleLength>(psr_pe_combined));
        });
  }

  std::expected<void, HAL::SpiError> set_tx_preamble_length(PreambleLength pl) {
    uint8_t psr_pe_combined = static_cast<uint8_t>(pl);
    uint8_t raw_psr = (psr_pe_combined >> 2) & 0b11;
    uint8_t raw_pe = psr_pe_combined & 0b11;

    auto tx_fctrl = get_reg_view<DWMRegisterID::TX_FCTRL>();
    return tx_fctrl.write_bit_range(19, 18, raw_psr).and_then([&] {
      return tx_fctrl.write_bit_range(21, 20, raw_pe);
    });
  }

  SPI spi_;
  GPIO gpio_;
  uint8_t rst_pin_{};
  uint8_t irq_pin_{};
};

#endif
