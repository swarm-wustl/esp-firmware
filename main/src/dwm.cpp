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
    logf("Bit rate, PRF, preamble length (but nice!):", tx_bit_rate(), tx_prf(), tx_preamble_length());

    set_tx_bit_rate(BitRate::KBPS_100);
    set_tx_prf(PRF::MHZ_4);
    set_tx_preamble_length(PreambleLength::LEN_2048);

    logf("New bit rate, PRF, preamble length:", tx_bit_rate(), "--", tx_prf(), "--", tx_preamble_length());
    hard_reset();
    logf("Reset bit rate, PRF, preamble length:", tx_bit_rate(), "--", tx_prf(), "--", tx_preamble_length());

    /* *** */

    auto sys_time_reg = get_reg_view<DWM_REG_SYS_TIME>();

    // Use the DW1000's own timestamp for precise intervals
    auto start = sys_time_reg.value();
    auto target_duration = std::chrono::milliseconds{300};

    while (true) {
        auto current = sys_time_reg.value();
        auto elapsed = current - start;
        
        if (elapsed >= target_duration) {
            auto us = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
            logf("DELTA SYS TIME:", us, "microseconds");
            start = current;  // Reset for next interval
        }
        
        vTaskDelay(1);  // Small delay to not busy-wait
    }
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
    auto tx_fctrl = get_reg_view<DWM_REG_TX_FCTRL>();
    uint8_t raw_bit_rate = tx_fctrl.bit_range(14, 13); // TODO: constants? 

    return BitRateToString(static_cast<BitRate>(raw_bit_rate)); 
}

template <HAL::GenericSPIController SPI>
void DWM<SPI>::set_tx_bit_rate(BitRate br) {
    auto tx_fctrl = get_reg_view<DWM_REG_TX_FCTRL>();
    tx_fctrl.write_bit_range(14, 13, static_cast<uint64_t>(br));
}

template <HAL::GenericSPIController SPI>
std::string_view DWM<SPI>::tx_prf() const {
    auto tx_fctrl = get_reg_view<DWM_REG_TX_FCTRL>();
    uint8_t raw_prf = tx_fctrl.bit_range(17, 16); // TODO: constants? 
    
    return PRFToString(static_cast<PRF>(raw_prf));
}

template <HAL::GenericSPIController SPI>
void DWM<SPI>::set_tx_prf(PRF prf) {
    auto tx_fctrl = get_reg_view<DWM_REG_TX_FCTRL>();
    tx_fctrl.write_bit_range(17, 16, static_cast<uint64_t>(prf));
}

template <HAL::GenericSPIController SPI>
uint16_t DWM<SPI>::tx_preamble_length() const {
    auto tx_fctrl = get_reg_view<DWM_REG_TX_FCTRL>();

    uint8_t raw_psr = tx_fctrl.bit_range(19, 18); // TODO: constants? 
    uint8_t raw_pe = tx_fctrl.bit_range(21, 20);  // TODO: constants?
    uint8_t psr_pe_combined = (raw_psr << 2) | raw_pe;

    return PreambleLengthToUInt(static_cast<PreambleLength>(psr_pe_combined));
}

template <HAL::GenericSPIController SPI>
void DWM<SPI>::set_tx_preamble_length(PreambleLength pl) {
    uint8_t psr_pe_combined = static_cast<uint8_t>(pl);
    uint8_t raw_psr = (psr_pe_combined >> 2) & 0b11;
    uint8_t raw_pe = psr_pe_combined & 0b11;
    
    auto tx_fctrl = get_reg_view<DWM_REG_TX_FCTRL>();
    tx_fctrl.write_bit_range(19, 18, raw_psr);
    tx_fctrl.write_bit_range(21, 20, raw_pe);
}

// Allows for templated definition in .cpp file
template class DWM<HW::SPI>;