#ifndef DWM_DATA_H
#define DWM_DATA_H

#include <algorithm>
#include <array>
#include <bit>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

/*
 * DWMData<N>
 *
 * A fixed-size, little-endian byte buffer representing the raw bytes of a
 * DW1000 register (as they appear on the SPI wire). It owns all the
 * byte/bit-twiddling that the register layer needs, with zero dependency on
 * SPI or any hardware — making it a pure value type that is trivially
 * unit-testable on the host.
 *
 * The DW1000 is little-endian, so the in-memory byte order matches the wire
 * order and integer conversions are a plain memcpy. This assumption is asserted
 * here, in one place, rather than being implicit in the register layer.
 *
 * Integer operations (to_uint, bit_range, write_bit_range) are only available
 * when the buffer fits in a uint64_t (N <= 8). Larger buffers (e.g. the
 * 1024-byte TX_BUFFER) still support span/byte/bit access.
 */
template <size_t N> class DWMData {
  static_assert(
      std::endian::native == std::endian::little,
      "DWMData assumes a little-endian host (DW1000 is little-endian)");

public:
  DWMData() = default;

  /*
   * Construct from an integer, storing it little-endian. If the integer is
   * wider than the buffer, the high bytes are dropped (only the low N bytes are
   * kept). This matches how DW1000 register writes work — you write a value
   * that is known to fit in the register's width.
   */
  explicit DWMData(std::integral auto val)
    requires(N <= sizeof(uint64_t))
  {
    assign_uint(val);
  }

  // --- raw access (used by the SPI layer) ---

  std::span<std::byte, N> span() { return bytes_; }
  std::span<const std::byte, N> span() const { return bytes_; }

  std::byte byte(size_t byte_index) const {
    // TODO: some sort of oob check?
    return bytes_[byte_index];
  }

  // --- integer view (only when the buffer fits in a uint64_t) ---

  uint64_t to_uint() const
    requires(N <= sizeof(uint64_t))
  {
    // std::memcpy handles the (common) case where N < 8, e.g. 5-byte registers,
    // which std::bit_cast cannot do because it requires an exact size match.
    uint64_t res{};
    std::memcpy(&res, bytes_.data(), N);
    return res;
  }

  // --- bit access ---

  /*
   * Get the specified bit, given a byte index and bit offset within that byte.
   */
  uint8_t bit(size_t byte_index, size_t bit_offset) const {
    return static_cast<uint8_t>(bytes_[byte_index] >> bit_offset) & 1;
  }

  /*
   * Get the specified bit, given an absolute bit number.
   */
  uint8_t bit(size_t bit_number) const {
    return bit(bit_number / 8, bit_number % 8);
  }

  /*
   * Read a contiguous range of bits, inspired by Verilog syntax, e.g. x[15:12].
   */
  uint64_t bit_range(uint8_t hi, uint8_t lo) const
    requires(N <= sizeof(uint64_t))
  {
    uint64_t raw = to_uint() >> lo;
    uint64_t mask = (1ULL << (1 + hi - lo)) - 1;
    return raw & mask;
  }

  /*
   * Write a contiguous range of bits, leaving the surrounding bits untouched.
   */
  void write_bit_range(uint8_t hi, uint8_t lo, uint64_t value)
    requires(N <= sizeof(uint64_t))
  {
    uint64_t raw = to_uint();
    uint64_t mask = (1ULL << (1 + hi - lo)) - 1;

    // Clear the target range, then write the masked value in.
    raw &= ~(mask << lo);
    raw |= (value & mask) << lo;

    assign_uint(raw);
  }

  bool operator==(const DWMData &) const = default;

private:
  std::array<std::byte, N> bytes_{};

  void assign_uint(std::integral auto val) {
    // std::bit_cast optimization when sizes match exactly.
    if constexpr (sizeof(val) == N) {
      bytes_ = std::bit_cast<std::array<std::byte, N>>(val);
    } else {
      bytes_ = {};
      std::memcpy(bytes_.data(), &val, std::min(sizeof(val), N));
    }
  }
};

#endif
