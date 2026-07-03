#ifndef DWM_H
#define DWM_H

#include "dwm_data.h"
#include "peripheral_hal.h"
#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cstring>
#include <expected>
#include <string_view>
#include <utility>

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
  // Keeps bits [39:9], clears bits [63:40] and [8:0]
  static constexpr uint64_t DW1000_40BIT_MASK = 0xFF'FF'FF'FF'FFULL;
  static constexpr uint64_t DW1000_LOW_9BITS_MASK = 0x1FFULL;
  static constexpr uint64_t DW1000_TIMESTAMP_MASK =
      DW1000_40BIT_MASK & ~DW1000_LOW_9BITS_MASK;

public:
  using Duration = std::chrono::duration<
      uint64_t, std::ratio<1, 63'897'600'000>>; // each bit = ~15.65 ps

  DWMTimestamp() = default;
  DWMTimestamp(uint64_t raw_time)
      : raw_time_{raw_time & DW1000_TIMESTAMP_MASK} {}

  Duration operator-(const DWMTimestamp &other) const {
    return Duration{(raw_time_ - other.raw_time_) & DW1000_TIMESTAMP_MASK};
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

  [[nodiscard]] std::expected<ValueType, esp_err_t> read() {
    return read_into_cache().transform([this] { return interpret(data_); });
  }

  [[nodiscard]] std::expected<void, esp_err_t> operator|=(uint64_t flags)
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

  [[nodiscard]] std::expected<void, esp_err_t> operator&=(uint64_t flags)
    requires IsReadWrite<ID> && (size_ <= sizeof(uint64_t))
  {
    return read_into_cache().and_then(
        [&] { return write_data(data_.to_uint() & flags); });
  }

  // TODO
  [[nodiscard]] std::expected<void, esp_err_t> operator+=(uint64_t)
    requires IsTimestampRegister<ID>
  {
    return {};
  }

  [[nodiscard]] std::expected<void, esp_err_t>
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
  [[nodiscard]] std::expected<void, esp_err_t>
  write_data(std::integral auto new_value)
    requires IsWritable<ID> && (size_ <= sizeof(uint64_t))
  {
    return write_data(DWMData<size_>{new_value}.span());
  }

  // TODO: maybe make the span have a dynamic_extent, and allow writes <= size_?
  [[nodiscard]] std::expected<void, esp_err_t>
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

  std::expected<void, esp_err_t> read_into_cache() {
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

  std::expected<uint32_t, esp_err_t> get_device_id() {
    return get_reg_view<DWMRegisterID::DEV_ID>().read().transform(
        [](uint64_t raw) { return static_cast<uint32_t>(raw); });
  }

  /*
   * Pulse Repetition Frequency
   */
  std::expected<PRF, esp_err_t> get_tx_prf() {
    return get_reg_view<DWMRegisterID::TX_FCTRL>().read().transform(
        [](uint64_t raw) {
          return static_cast<PRF>((raw >> 16) & 0b11); // TODO: constants?
        });
  }

  std::expected<void, esp_err_t> set_tx_prf(PRF prf) {
    return get_reg_view<DWMRegisterID::TX_FCTRL>().write_bit_range(
        17, 16, static_cast<uint64_t>(prf));
  }

  // valid only once LDEDONE is set for the corresponding reception
  std::expected<DWMTimestamp, esp_err_t> get_rx_timestamp() {
    return get_reg_view<DWMRegisterID::RX_TIME>().read();
  }

  std::expected<DWMTimestamp, esp_err_t> get_tx_timestamp() {
    return get_reg_view<DWMRegisterID::TX_TIME>().read();
  }

private:
  template <DWMRegisterID ID> using Register = DWMRegisterView<SPI, ID>;

  template <DWMRegisterID ID> Register<ID> get_reg_view() {
    return Register<ID>{spi_};
  }

  void hard_reset() {
    gpio_num_t rst = static_cast<gpio_num_t>(rst_pin_);

    gpio_.set_direction(rst, GPIO_MODE_OUTPUT);
    gpio_.set_level(rst, HAL::Voltage::LOW);
    gpio_.delay_ms(10);
    gpio_.set_level(rst, HAL::Voltage::HIGH);
    gpio_.delay_ms(10);
  }

  std::expected<std::string_view, esp_err_t> get_tx_bit_rate() {
    return get_reg_view<DWMRegisterID::TX_FCTRL>().read().transform(
        [](uint64_t raw) {
          return BitRateToString(
              static_cast<BitRate>((raw >> 13) & 0b11)); // TODO: constants?
        });
  }

  std::expected<void, esp_err_t> set_tx_bit_rate(BitRate br) {
    return get_reg_view<DWMRegisterID::TX_FCTRL>().write_bit_range(
        14, 13, static_cast<uint64_t>(br));
  }

  std::expected<uint16_t, esp_err_t> get_tx_preamble_length() {
    return get_reg_view<DWMRegisterID::TX_FCTRL>().read().transform(
        [](uint64_t raw) {
          uint8_t raw_psr = (raw >> 18) & 0b11; // TODO: constants?
          uint8_t raw_pe = (raw >> 20) & 0b11;  // TODO: constants?
          uint8_t psr_pe_combined = (raw_psr << 2) | raw_pe;

          return PreambleLengthToUInt(
              static_cast<PreambleLength>(psr_pe_combined));
        });
  }

  std::expected<void, esp_err_t> set_tx_preamble_length(PreambleLength pl) {
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
