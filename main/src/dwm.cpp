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
    log("Reg size: %d", id_reg.size());
    log("Reg value: %llX", id_reg.value());
    id_reg |= 0xFFFFFF;
    log("Reg value (should be same): %X", static_cast<uint32_t>(id_reg.value()));
    
    auto sys_status_reg = get_reg_view<DWM_REG_SYSTEM_EVENT_STATUS>();
    log("Value before: %X %X %X %X %X", sys_status_reg[4], sys_status_reg[3], sys_status_reg[2], sys_status_reg[1], sys_status_reg[0]);
    sys_status_reg |= 0xFF;
    log("Value after: %X %X %X %X %X", sys_status_reg[4], sys_status_reg[3], sys_status_reg[2], sys_status_reg[1], sys_status_reg[0]);
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

// Allows for templated definition in .cpp file
template class DWM<HW::SPI>;