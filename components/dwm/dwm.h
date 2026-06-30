#ifndef DWM_H
#define DWM_H

#include "dwm_data.h"
#include "swarm_hal.h"
#include <array>
#include <bit>
#include <chrono>
#include <cstring>
#include <string_view>

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

public:
  explicit DWMRegisterView(SPI &spi) : spi_{spi} { read_data(); }

  // TODO: constructor that takes in data?

  ~DWMRegisterView() = default;

  DWMRegisterView(const DWMRegisterView &other) = default;
  DWMRegisterView &operator=(const DWMRegisterView &) = default;

  DWMRegisterView(DWMRegisterView &&) = delete;
  void operator=(DWMRegisterView &&) = delete;

  /*
   * For a normal RW register this is a read-modify-write OR. For a
   * write-1-to-clear register, writing a 1 clears the bit, so we write the
   * flags directly with no read-back.
   */
  DWMRegisterView &operator|=(uint64_t flags)
    requires IsWritable<ID> && (size_ <= sizeof(uint64_t))
  {
    if constexpr (IsWriteOneClear<ID>) {
      write_data(flags);
    } else {
      write_data(data_.to_uint() | flags);
    }

    return *this;
  }

  DWMRegisterView &operator&=(uint64_t flags)
    requires IsReadWrite<ID> && (size_ <= sizeof(uint64_t))
  {
    write_data(data_.to_uint() & flags);

    return *this;
  }

  // TODO
  DWMRegisterView &operator+=(uint64_t)
    requires IsTimestampRegister<ID>
  {
    return *this;
  }

  /*
   * Access the locally-cached register bytes for bit/byte-level reads, e.g.
   * reg.data().bit_range(17, 16). The byte twiddling lives on DWMData; this
   * view is responsible only for SPI framing and persisting writes to the chip.
   */
  const DWMData<size_> &data() const { return data_; }

  /*
   * Read-modify-write a bit range and persist it to the device.
   * (Unlike DWMData::write_bit_range, this also pushes the result over SPI.)
   */
  DWMRegisterView &write_bit_range(uint8_t hi, uint8_t lo, uint64_t value)
    requires IsReadWrite<ID> && (size_ <= sizeof(uint64_t))
  {
    DWMData<size_> new_data{data_};
    new_data.write_bit_range(hi, lo, value);
    write_data(new_data.span());

    return *this;
  }

  auto value() const {
    if constexpr (size_ <= sizeof(uint64_t)) {
      auto res = data_.to_uint();

      if constexpr (IsTimestampRegister<ID>) {
        return DWMTimestamp{res};
      } else {
        return res;
      }
    } else {
      return data_.span();
    }
  }

  consteval size_t size() const { return size_; }

  void read_data() {
    // Lower 6 bits store actual register
    // MSbit = 0 represents read
    uint8_t reg = 0x00 | (static_cast<uint8_t>(ID) & 0x3F);

    // Store in single-value array to be compatible with SPI controller API
    std::array<const std::byte, 1> tx{std::byte{reg}};

    // Initiate SPI transfer
    // TODO: error handle
    spi_.transfer_halfduplex(tx, data_.span());
  }

  // TODO: consider removing this and other cases of std::integral auto
  // It might just be adding complexity for no reason (ig bit_cast
  // optimization..?)
  void write_data(std::integral auto new_value)
    requires IsWritable<ID> && (size_ <= sizeof(uint64_t))
  {
    write_data(DWMData<size_>{new_value}.span());
  }

  // TODO: maybe make the span have a dynamic_extent, and allow writes <= size_?
  void write_data(std::span<const std::byte, size_> new_data)
    requires IsWritable<ID>
  {
    // Lower 6 bits store actual register
    // MSbit = 1 represents write
    uint8_t reg = 0x80 | (static_cast<uint8_t>(ID) & 0x3F);

    // Create a std::array one larger than our data
    // This is because the first byte in the transfer needs to be the register
    std::array<std::byte, size_ + 1> tx{};

    // Store the register in byte 0, then copy the rest of the data
    // TODO: use std::ranges::copy and std::ranges in general instead
    auto it = tx.begin();
    *it = std::byte{reg};
    std::copy(new_data.begin(), new_data.end(), ++it);

    // Initiate SPI transfer
    // TODO: error handle
    spi_.transfer_halfduplex(tx, {});

    // For RW registers the written value is the new on-chip state, so keep the
    // cache coherent. Write-1-to-clear registers must be re-read instead.
    if constexpr (IsReadWrite<ID>) {
      std::copy(new_data.begin(), new_data.end(), data_.span().begin());
    }

    // TODO: 'debug' mode that does read-back and asserts that the cached value
    // equals the read-back
  }

private:
  SPI &spi_{};
  DWMData<size_> data_{};
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

  auto get_device_id() { return get_reg_view<DWMRegisterID::DEV_ID>().value(); }

  /*
   * Pulse Repetition Frequency
   */
  PRF get_tx_prf() {
    auto tx_fctrl = get_reg_view<DWMRegisterID::TX_FCTRL>();
    uint8_t raw_prf = tx_fctrl.data().bit_range(17, 16); // TODO: constants?

    return static_cast<PRF>(raw_prf);
  }

  void set_tx_prf(PRF prf) {
    auto tx_fctrl = get_reg_view<DWMRegisterID::TX_FCTRL>();
    tx_fctrl.write_bit_range(17, 16, static_cast<uint64_t>(prf));
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

  std::string_view get_tx_bit_rate() const {
    auto tx_fctrl = get_reg_view<DWMRegisterID::TX_FCTRL>();
    uint8_t raw_bit_rate =
        tx_fctrl.data().bit_range(14, 13); // TODO: constants?

    return BitRateToString(static_cast<BitRate>(raw_bit_rate));
  }

  void set_tx_bit_rate(BitRate br) {
    auto tx_fctrl = get_reg_view<DWMRegisterID::TX_FCTRL>();
    tx_fctrl.write_bit_range(14, 13, static_cast<uint64_t>(br));
  }

  uint16_t get_tx_preamble_length() const {
    auto tx_fctrl = get_reg_view<DWMRegisterID::TX_FCTRL>();

    uint8_t raw_psr = tx_fctrl.data().bit_range(19, 18); // TODO: constants?
    uint8_t raw_pe = tx_fctrl.data().bit_range(21, 20);  // TODO: constants?
    uint8_t psr_pe_combined = (raw_psr << 2) | raw_pe;

    return PreambleLengthToUInt(static_cast<PreambleLength>(psr_pe_combined));
  }

  void set_tx_preamble_length(PreambleLength pl) {
    uint8_t psr_pe_combined = static_cast<uint8_t>(pl);
    uint8_t raw_psr = (psr_pe_combined >> 2) & 0b11;
    uint8_t raw_pe = psr_pe_combined & 0b11;

    auto tx_fctrl = get_reg_view<DWMRegisterID::TX_FCTRL>();
    tx_fctrl.write_bit_range(19, 18, raw_psr);
    tx_fctrl.write_bit_range(21, 20, raw_pe);
  }

  SPI spi_;
  GPIO gpio_;
  uint8_t rst_pin_{};
  uint8_t irq_pin_{};
};

#endif
