#ifndef DWM_H
#define DWM_H

#include "hal.h"
#include <bit>

// TODO: add noexcept to classes

// Allows for static_assert<false, ..> - like behavior
// Should be properly fixed in C++26, but this is a workaround
// https://en.cppreference.com/w/cpp/language/static_assert.html
template<class>
constexpr bool dependent_false = false;

static constexpr size_t DWM_REG_DEV_ID = 0x00;
static constexpr size_t DWM_LEN_DEV_ID = 0x04;

template <HAL::GenericSPIController SPI, uint8_t ID>
class DWMRegisterView {
public:
    DWMRegisterView(SPI& spi) : 
        spi_{spi} 
    {
        fill_data();
    }

    // TODO: constructor that takes in data?
    ~DWMRegisterView() = default;

    DWMRegisterView(const DWMRegisterView&);
    DWMRegisterView& operator=(const DWMRegisterView&);

    DWMRegisterView(DWMRegisterView&&) = delete;
    void operator=(DWMRegisterView&&) = delete;

    // DWMRegisterView& operator=(std::)

    size_t value() const {
        static_assert(size_ <= sizeof(size_t), "Register does not fit within size_t"); 
        return std::bit_cast<size_t>(data_);
    }

    constexpr size_t size() const {
        return size_;
    }

private:
    void fill_data() {
        // Lower 6 bits store actual register
        // MSbit = 0 represents read
        uint8_t reg = 0x00 | (ID & 0x3F);

        // Store in single-value array to be compatible with SPI controller API
        std::array<const std::byte, 1> tx{std::byte{reg}};

        // Initiate SPI transfer
        // TODO: error handle
        spi_.transfer_halfduplex(tx, data_);
    }

    static constexpr size_t size_ = []() constexpr {
        if constexpr (ID == DWM_REG_DEV_ID) return 4;
        else static_assert(dependent_false<void>, "Register size unspecified");
    }();

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
    constexpr Register<ID> get_reg_view() {
        return Register<ID>{spi_};
    }

    void read_reg(uint8_t reg, std::span<std::byte> rx);

    void hard_reset();

    SPI spi_;
    uint8_t rst_pin_{};
    uint8_t irq_pin_{};
};

#endif