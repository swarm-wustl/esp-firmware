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

// thin data wrapper for DWM binary data
// handles basic operations like comptime size checks,
// grabbing bits, writing bit ranges, etc.
template <size_t N> class DWMData {
  static_assert(
      std::endian::native == std::endian::little,
      "DWMData assumes a little-endian host (DW1000 is little-endian)");

public:
  DWMData() = default;

  explicit DWMData(std::integral auto val)
    requires(N <= sizeof(uint64_t))
  {
    assign_uint(val);
  }

  std::span<std::byte, N> span() { return bytes_; }
  std::span<const std::byte, N> span() const { return bytes_; }

  std::byte byte(size_t byte_index) const {
    // TODO: some sort of oob check?
    return bytes_[byte_index];
  }

  uint64_t to_uint() const
    requires(N <= sizeof(uint64_t))
  {
    uint64_t res{};
    std::memcpy(&res, bytes_.data(), N);
    return res;
  }

  uint8_t bit(size_t byte_index, size_t bit_offset) const {
    return static_cast<uint8_t>(bytes_[byte_index] >> bit_offset) & 1;
  }

  uint8_t bit(size_t bit_number) const {
    return bit(bit_number / 8, bit_number % 8);
  }

  uint64_t bit_range(uint8_t hi, uint8_t lo) const
    requires(N <= sizeof(uint64_t))
  {
    uint64_t raw = to_uint() >> lo;
    uint64_t mask = (1ULL << (1 + hi - lo)) - 1;
    return raw & mask;
  }

  void write_bit_range(uint8_t hi, uint8_t lo, uint64_t value)
    requires(N <= sizeof(uint64_t))
  {
    uint64_t raw = to_uint();
    uint64_t mask = (1ULL << (1 + hi - lo)) - 1;

    // Clear the target range, then write the masked value in
    raw &= ~(mask << lo);
    raw |= (value & mask) << lo;

    assign_uint(raw);
  }

  bool operator==(const DWMData &) const = default;

private:
  std::array<std::byte, N> bytes_{};

  void assign_uint(std::integral auto val) {
    // std::bit_cast optimization
    // might not actually help, but only works when sizes match anyway
    if constexpr (sizeof(val) == N) {
      bytes_ = std::bit_cast<std::array<std::byte, N>>(val);
    } else {
      bytes_ = {};
      std::memcpy(bytes_.data(), &val, std::min(sizeof(val), N));
    }
  }
};

#endif
