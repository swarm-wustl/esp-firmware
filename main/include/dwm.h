#ifndef DWM_H
#define DWM_H

#include "hal.h"
#include <bit>
#include <cstring>
#include <string>

// TODO: add noexcept to classes

// Allows for static_assert<false, ..> - like behavior
// Should be properly fixed in C++26, but this is a workaround
// https://en.cppreference.com/w/cpp/language/static_assert.html
template<class>
constexpr bool dependent_false = false;

static constexpr uint8_t DWM_REG_DEV_ID = 0x00;
static constexpr uint8_t DWM_REG_SYSTEM_EVENT_STATUS = 0x0F;
static constexpr uint8_t DWM_REG_SYS_TIME = 0x06;
static constexpr uint8_t DWM_REG_RX_TIME = 0x15;
static constexpr uint8_t DWM_REG_TX_TIME = 0x17;
static constexpr uint8_t DWM_REG_TX_FCTRL = 0x08;

template <uint8_t ID>
concept IsTimestampRegister = 
    ID == DWM_REG_SYS_TIME ||
    ID == DWM_REG_TX_TIME || 
    ID == DWM_REG_RX_TIME;

template <HAL::GenericSPIController SPI, uint8_t ID>
class DWMRegisterView {
    static constexpr size_t size_ = []() constexpr {
        if constexpr (ID == DWM_REG_DEV_ID) return 4;
        else if constexpr (ID == DWM_REG_SYSTEM_EVENT_STATUS) return 5;
        else if constexpr (ID == DWM_REG_SYS_TIME) return 5;
        else if constexpr (ID == DWM_REG_TX_FCTRL) return 5;
        else static_assert(dependent_false<void>, "Register size unspecified");
    }();

public:
    DWMRegisterView(SPI& spi) : 
        spi_{spi} 
    {
        read_data();
    }

    // TODO: constructor that takes in data?

    ~DWMRegisterView() = default;

    DWMRegisterView(const DWMRegisterView& other) = default;
    DWMRegisterView& operator=(const DWMRegisterView&) = default;

    DWMRegisterView(DWMRegisterView&&) = delete;
    void operator=(DWMRegisterView&&) = delete;

    /*
    * XOR: used to clear values by writing flags.
    * This is for registers that have status bits/bytes that are cleared by writing 1 to them.
    */
    DWMRegisterView& operator^=(uint64_t flags) requires (size_ <= sizeof(uint64_t)) {
        // For the DW1000 in particular, when we write flags, we are CLEARING values.
        // Thus, we don't OR the flags with the original value,
        // we just write the flags directly.
        write_data(flags);

        return *this;
    }

    /*
    * OR: used to OR values to a register.
    * This is primarily for configuring registers.
    */
    DWMRegisterView& operator|=(uint64_t flags) requires (size_ <= sizeof(uint64_t)) {
        // Copy the current data and OR the flags onto it
        uint64_t new_value = flatten_data(data_) | flags;
        write_data(new_value);
        
        return *this;
    }

    /*
    * AND: used to AND values to a register.
    * This is primarily for configuring registers.
    */
    DWMRegisterView& operator&=(uint64_t flags) requires (size_ <= sizeof(uint64_t)) {
        // Copy the current data and AND the flags onto it
        uint64_t new_value = flatten_data(data_) & flags;
        write_data(new_value);

        return *this;
    }

    // TODO
    DWMRegisterView& operator+=(uint64_t) requires IsTimestampRegister<ID> {
        return *this;
    }

    /* 
    * Get the specified byte from data 
    */
    std::byte operator[](size_t byte_index) const {
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
    uint64_t bit_range(uint8_t hi, uint8_t lo) const requires (size_ <= sizeof(uint64_t)) {
        uint64_t raw_data = flatten_data(data_) >> lo;
        uint64_t mask = (1ULL << ((hi - lo) + 1)) - 1;
        return raw_data & mask;
    }

    auto value() requires(size_ <= sizeof(uint64_t)) {
        read_data();
        return flatten_data(data_);
    }

    constexpr size_t size() const {
        return size_;
    }

private: 
    void read_data() {
        // Lower 6 bits store actual register
        // MSbit = 0 represents read
        uint8_t reg = 0x00 | (ID & 0x3F);

        // Store in single-value array to be compatible with SPI controller API
        std::array<const std::byte, 1> tx{std::byte{reg}};

        // Initiate SPI transfer
        // TODO: error handle
        spi_.transfer_halfduplex(tx, data_);
    }

    void write_data(std::integral auto new_value) {
        write_data(pack_data(new_value));
    }

    void write_data(std::array<std::byte, size_> new_data) {
        // Lower 6 bits store actual register
        // MSbit = 1 represents write
        uint8_t reg = 0x80 | (ID & 0x3F);

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
        // 2) some registers clear values by writing 1 to them (so the local array's state would be inverted)
        // 3) we want the most updated register state after writing!
        read_data();
    }

    static auto flatten_data(std::array<std::byte, size_> data) requires (size_ <= sizeof(uint64_t)) {
        if constexpr (size_ == sizeof(uint16_t)) {
            return std::bit_cast<uint16_t>(data);
        } else if constexpr (size_ == sizeof(uint32_t)) {
            return std::bit_cast<uint32_t>(data);
        } else {
            // std::bit_cast requires an exact size-match
            // Therefore, std::memcpy is necessary since many DW1000 regs are 5 bytes in size
            // (rather than uint64_t's 8 bytes)
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

    SPI& spi_{};
    std::array<std::byte, size_> data_{};
};

template <HAL::GenericSPIController SPI>
class DWM {
public:
    DWM(SPI spi, uint8_t rst_pin, uint8_t irq_pin);
    ~DWM() = default;
    DWM(const DWM&) = delete;
    void operator=(const DWM&) = delete;
    DWM(DWM&&) = delete;
    void operator=(DWM&&) = delete;

private:
    template <uint8_t ID>
    using Register = DWMRegisterView<SPI, ID>;

    template <uint8_t ID>
    Register<ID> get_reg_view() const {
        return Register<ID>{const_cast<SPI&>(spi_)};
    }

    void read_reg(uint8_t reg, std::span<std::byte> rx);

    void hard_reset();

    std::string_view tx_bit_rate() const;
    /*
    * Pulse Repetition Frequency
    */
    std::string_view tx_prf() const;
    uint16_t tx_preamble_length() const;

    SPI spi_;
    uint8_t rst_pin_{};
    uint8_t irq_pin_{};
};

#endif