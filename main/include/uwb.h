#ifndef UWB_H
#define UWB_H

#include "hal/spi_types.h"
#include "driver/spi_master.h"
#include <cstddef>
#include <cstdint>

void uwb_init();

esp_err_t uwb_read_reg(uint8_t reg, uint8_t* rx, size_t len, spi_device_handle_t dev_handle);

#endif