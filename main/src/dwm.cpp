#include "dwm.h"

#include "esp32.h"
#include "hardware.h"
#include "error.h"

#include <vector>
#include <bit>

static constexpr size_t DWM_REG_DEV_ID = 0x00;
static constexpr size_t DWM_LEN_DEV_ID = 0x04;

template <HAL::GenericSPIController SPI>
DWM<SPI>::DWM(SPI spi) : spi_{std::move(spi)} {
    // // Hard reset the device
    // uwb_hard_reset();
    // vTaskDelay(pdMS_TO_TICKS(10));

    std::array<std::byte, DWM_LEN_DEV_ID> rx{};
    read_reg(DWM_REG_DEV_ID, rx);
    
    log("ID received: %X", std::bit_cast<uint32_t>(rx));
}

template <HAL::GenericSPIController SPI>
void DWM<SPI>::read_reg(uint8_t reg, std::span<std::byte> rx) {
    // Lower 6 bits store actual register
    // MSbit = 0 represents read
    reg = 0x00 | (reg & 0x3F);

    // Store in single-value array to be compatible with SPI controller API
    std::array<const std::byte, 1> tx{std::byte{reg}};

    // Initiate SPI transfer
    // TODO: error handle
    spi_.transfer_halfduplex(tx, rx);
}

// Allows for templated definition in .cpp file
template class DWM<HW::SPI>;