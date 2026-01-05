#include "dwm.h"
#include "esp32.h"
#include "hardware.h"

/*DWM::DWM(SPI spi) : spi_{spi} {

}*/

template <HAL::GenericSPIController SPI>
DWM<SPI>::DWM(SPI spi) : spi_{std::move(spi)} {

}

// Allows for templated definition in .cpp file
template class DWM<HW::SPI>;