#ifndef DWM_H
#define DWM_H

#include "swarm_hal.h"
#include <bit>
#include <chrono>
#include <cstring>
#include <string>

// TODO: add noexcept to classes

// Allows for static_assert<false, ..> - like behavior
// Without this weird hack, the static_assert is evaluated every time
// instead of conditionally based on the branching
// (always returns an error even when it shouldn't).
// Should be properly fixed in C++26, but this is a workaround
// https://en.cppreference.com/w/cpp/language/static_assert.html
template <auto V> constexpr bool dependent_false = false;

enum class DWMRegisterID : uint8_t {
  DEV_ID = 0x00,
  TX_FCTRL = 0x08,
  SYS_TIME = 0x06,
  SYSTEM_EVENT_STATUS = 0x0F,
  RX_TIME = 0x15,
  TX_TIME = 0x17,
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

template <HAL::GenericSPIController SPI, DWMRegisterID ID>
class DWMRegisterView {
  static constexpr size_t size_ = []() consteval {
    if constexpr (ID == DWMRegisterID::DEV_ID)
      return 4;
    else if constexpr (ID == DWMRegisterID::SYSTEM_EVENT_STATUS)
      return 5;
    else if constexpr (ID == DWMRegisterID::SYS_TIME)
      return 5;
    else if constexpr (ID == DWMRegisterID::TX_FCTRL)
      return 5;
    else
      static_assert(dependent_false<ID>, "Register size unspecified");
  }();

public:
  explicit DWMRegisterView(SPI &spi) : spi_{spi} { read_data(); }

  // TODO: constructor that takes in data?

  ~DWMRegisterView() = default;

  DWMRegisterView(const DWMRegisterView &other) = default;
  DWMRegisterView &operator=(const DWMRegisterView &) = default;

  DWMRegisterView(DWMRegisterView &&) = delete;
  void operator=(DWMRegisterView &&) = delete;

  /*
   * Clear register values by writing flags.
   * This is for registers that have status bits/bytes that are cleared by
   * writing 1 to them.
   */
  DWMRegisterView &clear_flags(uint64_t flags)
    requires(size_ <= sizeof(uint64_t))
  {
    // For the DW1000 in particular, when we write flags, we are CLEARING
    // values. Thus, we don't OR the flags with the original value, we just
    // write the flags directly.
    write_data(flags);

    return *this;
  }

  /*
   * OR: used to OR values to a register.
   * This is primarily for configuring registers.
   */
  DWMRegisterView &operator|=(uint64_t flags)
    requires(size_ <= sizeof(uint64_t))
  {
    // Copy the current data and OR the flags onto it
    uint64_t new_value = flatten_data(data_) | flags;
    write_data(new_value);

    return *this;
  }

  /*
   * AND: used to AND values to a register.
   * This is primarily for configuring registers.
   */
  DWMRegisterView &operator&=(uint64_t flags)
    requires(size_ <= sizeof(uint64_t))
  {
    // Copy the current data and AND the flags onto it
    uint64_t new_value = flatten_data(data_) & flags;
    write_data(new_value);

    return *this;
  }

  /*
   * Equals: used to assign a value to a register.
   */
  DWMRegisterView &operator=(std::integral auto new_value)
    requires(size_ <= sizeof(uint64_t))
  {
    write_data(new_value);

    return *this;
  }

  // TODO
  DWMRegisterView &operator+=(uint64_t)
    requires IsTimestampRegister<ID>
  {
    return *this;
  }

  /*
   * Get the specified byte from data
   */
  std::byte byte(size_t byte_index) const {
    // TODO: some sort of oob check?
    return data_[byte_index];
  }

  // TODO: once supported, switch to multi-dimension operator[]
  // Supposed to be in C++23 but I guess ESP-IDF is a bit behind on features
  /*
   * Get the specified bit from data, given a byte and bit offset
   */
  uint8_t bit(size_t byte_index, size_t bit_offset) const {
    return static_cast<uint8_t>((data_[byte_index] >> bit_offset)) & 1;
  }

  /*
   * Get the specified bit from data, given a bit number
   */
  uint8_t bit(size_t bit_number) const {
    return bit(bit_number / 8, bit_number % 8);
  }

  /*
   * Get the specified data from a bit range
   * Inspired by Verilog syntax, e.g., x[15:12]
   */
  uint64_t bit_range(uint8_t hi, uint8_t lo) const
    requires(size_ <= sizeof(uint64_t))
  {
    uint64_t raw_data = flatten_data(data_) >> lo;
    uint64_t mask = (1ULL << (1 + hi - lo)) - 1;
    return raw_data & mask;
  }

  DWMRegisterView &write_bit_range(uint8_t hi, uint8_t lo, uint64_t value)
    requires(size_ <= sizeof(uint64_t))
  {
    // First, clear the bits in the given bit range
    uint64_t raw_data = flatten_data(data_);
    uint64_t mask = (1ULL << (1 + hi - lo)) - 1;
    raw_data &= ~(mask << lo);

    // Then, write the new data in
    raw_data |= (value & mask) << lo;
    write_data(raw_data);

    return *this;
  }

  auto value()
    requires(size_ <= sizeof(uint64_t))
  {
    read_data();

    auto res = flatten_data(data_);

    if constexpr (IsTimestampRegister<ID>) {
      return DWMTimestamp{res};
    } else {
      return res;
    }
  }

  consteval size_t size() const { return size_; }

private:
  void read_data() {
    // Lower 6 bits store actual register
    // MSbit = 0 represents read
    uint8_t reg = 0x00 | (static_cast<uint8_t>(ID) & 0x3F);

    // Store in single-value array to be compatible with SPI controller API
    std::array<const std::byte, 1> tx{std::byte{reg}};

    // Initiate SPI transfer
    // TODO: error handle
    spi_.transfer_halfduplex(tx, data_);
  }

  // TODO: consider removing this and other cases of std::integral auto
  // It might just be adding complexity for no reason (ig bit_cast
  // optimization..?)
  void write_data(std::integral auto new_value) {
    write_data(std::move(pack_data(new_value)));
  }

  void write_data(std::span<const std::byte, size_> new_data) {
    // Lower 6 bits store actual register
    // MSbit = 1 represents write
    uint8_t reg = 0x80 | (static_cast<uint8_t>(ID) & 0x3F);

    // Create a std::array one larger than our data
    // This is because the first byte in the transfer needs to be the register
    std::array<std::byte, size_ + 1> tx{};

    // Store the register in byte 0, then copy the rest of the data
    auto it = tx.begin();
    *it = std::byte{reg};
    std::copy(new_data.begin(), new_data.end(), ++it);

    // Initiate SPI transfer
    // TODO: error handle
    spi_.transfer_halfduplex(tx, {});

    // Lastly, read data to get updated register value
    // This is for a few reasons:
    // 1) some registers are read-only, and writes should do nothing
    // 2) some registers clear values by writing 1 to them (so the local array's
    // state would be inverted) 3) we want the most updated register state after
    // writing!
    read_data();
  }

  static auto flatten_data(std::span<std::byte, size_> data)
    requires(size_ <= sizeof(uint64_t))
  {
    if constexpr (size_ == sizeof(uint16_t)) {
      return std::bit_cast<uint16_t>(data);
    } else if constexpr (size_ == sizeof(uint32_t)) {
      return std::bit_cast<uint32_t>(data);
    } else {
      // std::bit_cast requires an exact size-match
      // Therefore, std::memcpy is necessary since many DW1000 regs are 5 bytes
      // in size (rather than uint64_t's 8 bytes)
      uint64_t res{};
      std::memcpy(&res, data.data(), size_);
      return res;
    }
  }

  static std::array<std::byte, size_> pack_data(std::integral auto val) {
    // std::bit_cast optimization
    if constexpr (sizeof(val) == size_) {
      return std::bit_cast<std::array<std::byte, size_>>(val);
    } else {
      std::array<std::byte, size_> new_data{};
      std::memcpy(new_data.data(), &val, size_);
      return new_data;
    }
  }

  SPI &spi_{};
  std::array<std::byte, size_> data_{};
};

template <HAL::GenericSPIController SPI, HAL::GenericGPIOController GPIO>
class DWM {
  static_assert(std::endian::native == std::endian::little,
                "DWM1000 requires little-endian architecture");

public:
  DWM(SPI &&spi, GPIO gpio, uint8_t rst_pin, uint8_t irq_pin)
      : spi_{std::move(spi)}, gpio_{std::move(gpio)}, rst_pin_{rst_pin},
        irq_pin_{irq_pin} {
    hard_reset();
    // auto id_reg = get_reg_view<DWM_REG_DEV_ID>();
    // // log("Reg size: %u", id_reg.size());
    // // log("Reg value: %X", id_reg.value());
    // id_reg |= 0xFFFFFF;
    // // log("Reg value (should be same): %X", id_reg.value());
    //
    // auto sys_status_reg = get_reg_view<DWM_REG_SYSTEM_EVENT_STATUS>();
    // // log("Value before: %llX", sys_status_reg.value());
    // sys_status_reg.clear_flags(0xFF);
    // // log("Value after: %llX", sys_status_reg.value());
    //
    // auto tx_fctrl = get_reg_view<DWM_REG_TX_FCTRL>();
    //
    // // log("Current transmit bit rate: %X %X", ((tx_fctrl.bit(14) << 1) |
    // // tx_fctrl.bit(13)), tx_fctrl.bit_range(14, 13)); logf("Bit rate, PRF,
    // // preamble length (but nice!):", tx_bit_rate(), tx_prf(),
    // // tx_preamble_length());
    //
    // set_tx_bit_rate(BitRate::KBPS_100);
    // set_tx_prf(PRF::MHZ_4);
    // set_tx_preamble_length(PreambleLength::LEN_2048);
    //
    // // logf("New bit rate, PRF, preamble length:", tx_bit_rate(), "--",
    // // tx_prf(), "--", tx_preamble_length());
    // hard_reset();
    // // logf("Reset bit rate, PRF, preamble length:", tx_bit_rate(), "--",
    // // tx_prf(), "--", tx_preamble_length());
    //
    // /* *** */
    //
    // auto sys_time_reg = get_reg_view<DWM_REG_SYS_TIME>();
    //
    // // Use the DW1000's own timestamp for precise intervals
    // auto start = sys_time_reg.value();
    // auto target_duration = std::chrono::milliseconds{300};
    //
    // while (true) {
    //   auto current = sys_time_reg.value();
    //   auto elapsed = current - start;
    //
    //   if (elapsed >= target_duration) {
    //     auto us =
    //     std::chrono::duration_cast<std::chrono::microseconds>(elapsed)
    //                   .count();
    //     // logf("DELTA SYS TIME:", us, "microseconds");
    //     start = current; // Reset for next interval
    //   }
    //
    //   gpio_.delay_ms(50); // Small delay to not busy-wait
    // }
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

  enum class PRF : uint8_t { MHZ_4 = 0b00, MHZ_16 = 0b01, MHZ_64 = 0b10 };

  static constexpr std::string_view PRFToString(PRF prf) noexcept {
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

  auto get_device_id() const {
    return get_reg_view<DWMRegisterID::DEV_ID>().value();
  }

  /*
   * Pulse Repetition Frequency
   */
  PRF tx_prf() const {
    auto tx_fctrl = get_reg_view<DWMRegisterID::TX_FCTRL>();
    uint8_t raw_prf = tx_fctrl.bit_range(17, 16); // TODO: constants?

    return static_cast<PRF>(raw_prf);
  }

  void set_tx_prf(PRF prf) {
    auto tx_fctrl = get_reg_view<DWMRegisterID::TX_FCTRL>();
    tx_fctrl.write_bit_range(17, 16, static_cast<uint64_t>(prf));
  }

private:
  template <DWMRegisterID ID> using Register = DWMRegisterView<SPI, ID>;

  template <DWMRegisterID ID> Register<ID> get_reg_view() const {
    return Register<ID>{const_cast<SPI &>(spi_)};
  }

  void hard_reset() {
    gpio_num_t rst = static_cast<gpio_num_t>(rst_pin_);

    gpio_.set_direction(rst, GPIO_MODE_OUTPUT);
    gpio_.set_level(rst, HAL::Voltage::LOW);
    gpio_.delay_ms(10);
    gpio_.set_level(rst, HAL::Voltage::HIGH);
    gpio_.delay_ms(10);
  }

  std::string_view tx_bit_rate() const {
    auto tx_fctrl = get_reg_view<DWMRegisterID::TX_FCTRL>();
    uint8_t raw_bit_rate = tx_fctrl.bit_range(14, 13); // TODO: constants?

    return BitRateToString(static_cast<BitRate>(raw_bit_rate));
  }

  void set_tx_bit_rate(BitRate br) {
    auto tx_fctrl = get_reg_view<DWMRegisterID::TX_FCTRL>();
    tx_fctrl.write_bit_range(14, 13, static_cast<uint64_t>(br));
  }

  uint16_t tx_preamble_length() const {
    auto tx_fctrl = get_reg_view<DWMRegisterID::TX_FCTRL>();

    uint8_t raw_psr = tx_fctrl.bit_range(19, 18); // TODO: constants?
    uint8_t raw_pe = tx_fctrl.bit_range(21, 20);  // TODO: constants?
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
