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
    SPI spi_{};
};

#endif