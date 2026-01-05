#ifndef DWM_H
#define DWM_H

#include "hal.h"

template <HAL::GenericSPIController SPI>
class DWM {
public:
    DWM(SPI spi);
    ~DWM() = default;
    DWM(const DWM&) = delete;
    void operator=(const DWM&) = delete;
    DWM(DWM&&) = delete;
    void operator=(DWM&&) = delete;

private:
    void read_reg(uint8_t reg, std::span<std::byte> rx);

    SPI spi_{};
};

#endif