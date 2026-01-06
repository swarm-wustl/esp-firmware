#include "dwm.h"

#include "esp32.h"
#include "hardware.h"
#include "error.h"
#include "freertos/task.h"

#include <vector>

template <HAL::GenericSPIController SPI>
DWM<SPI>::DWM(SPI spi, uint8_t rst_pin, uint8_t irq_pin) : 
    spi_{std::move(spi)},
    rst_pin_{rst_pin}, irq_pin_{irq_pin}
{
    hard_reset();

    // std::array<std::byte, DWM_LEN_DEV_ID> rx{};
    // read_reg(DWM_REG_DEV_ID, rx);
    // log("ID received: %X", std::bit_cast<uint32_t>(rx));

    auto id_reg = get_reg_view<DWM_REG_DEV_ID>();
    log("Reg size: %u", id_reg.size());
    log("Reg value: %X", id_reg.value());
    id_reg |= 0xFFFFFF;
    log("Reg value (should be same): %X", id_reg.value());
    
    auto sys_status_reg = get_reg_view<DWM_REG_SYSTEM_EVENT_STATUS>();
    log("Value before: %llX", sys_status_reg.value());
    sys_status_reg.clear_flags(0xFF);
    log("Value after: %llX", sys_status_reg.value());

    auto tx_fctrl = get_reg_view<DWM_REG_TX_FCTRL>();
    log("Current transmit bit rate: %X %X", ((tx_fctrl.bit(14) << 1) | tx_fctrl.bit(13)), tx_fctrl.bit_range(14, 13));
    logf("Bit rate, PRF (but nice!):", tx_bit_rate(), tx_prf(), tx_preamble_length());

    auto sys_time_reg = get_reg_view<DWM_REG_SYS_TIME>();
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        log("SYS TIME: %llX", sys_time_reg.value());
    }
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

// TODO: this should use some sort of HAL::GenericGPIOController
// should not directly interact with ESP32 HAL!
template <HAL::GenericSPIController SPI>
void DWM<SPI>::hard_reset() {
    gpio_num_t rst = static_cast<gpio_num_t>(rst_pin_);

    gpio_set_direction(rst, GPIO_MODE_OUTPUT);
    gpio_set_level(rst, 0);
    vTaskDelay(pdMS_TO_TICKS(10));  // hold low for >10ms
    gpio_set_level(rst, 1);
    vTaskDelay(pdMS_TO_TICKS(10));  // wait for startup
}

template <HAL::GenericSPIController SPI>
std::string_view DWM<SPI>::tx_bit_rate() const {
    using namespace std::string_view_literals;   // Allows for ""sv suffix

    auto tx_fctrl = get_reg_view<DWM_REG_TX_FCTRL>();
    uint8_t raw_bit_rate = tx_fctrl.bit_range(14, 13); // TODO: constants? 
    
    switch (raw_bit_rate) {
        case 0b00: return "110 kbps"sv;
        case 0b01: return "850 kbps"sv;
        case 0b10: return "6.8 Mbps"sv;
        case 0b11: assert("ERROR: reserved register value"); break;
        default: assert("Unexpected behavior: 2-bit value was not matched"); break;
    }

    return {};
}

template <HAL::GenericSPIController SPI>
std::string_view DWM<SPI>::tx_prf() const {
    using namespace std::string_view_literals;   // Allows for ""sv suffix

    auto tx_fctrl = get_reg_view<DWM_REG_TX_FCTRL>();
    uint8_t raw_prf = tx_fctrl.bit_range(17, 16); // TODO: constants? 
    
    switch (raw_prf) {
        case 0b00: return "4 MHz"sv;
        case 0b01: return "16 MHz"sv;
        case 0b10: return "64 MHz"sv;
        case 0b11: assert("ERROR: reserved register value"); break;
        default: assert("Unexpected behavior: 2-bit value was not matched"); break;
    }

    return {};
}

template <HAL::GenericSPIController SPI>
uint16_t DWM<SPI>::tx_preamble_length() const {
    auto tx_fctrl = get_reg_view<DWM_REG_TX_FCTRL>();
    uint8_t raw_psr = tx_fctrl.bit_range(19, 18); // TODO: constants? 
    uint8_t raw_pe = tx_fctrl.bit_range(21, 20);  // TODO: constants?
    
    switch ((raw_psr << 2) | raw_pe) {
        case 0b01'00: return 64;
        case 0b01'01: return 128;
        case 0b01'10: return 256;
        case 0b01'11: return 512;
        case 0b10'00: return 1024;
        case 0b10'01: return 1536;
        case 0b10'10: return 2048;
        case 0b11'00: return 4096;
        default: assert("Unexpected behavior: 4-bit value was not matched"); break;
    }

    return {};
}

// Allows for templated definition in .cpp file
template class DWM<HW::SPI>;