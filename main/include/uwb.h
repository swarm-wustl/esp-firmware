#ifndef UWB_H
#define UWB_H

#include "hal/spi_types.h"
#include "driver/spi_master.h"
#include <cstddef>
#include <cstdint>

struct __attribute__((packed)) dwm_transmit_frame_control_t {
    uint32_t raw;
    uint8_t ifsdelay;
};

template <typename T>
static inline T GET_FIELD(T data, uint8_t start_bit, uint8_t len_bits) {
    T bitmask = (1UL << len_bits) - 1;
    return (data >> start_bit) & bitmask;
}

template <typename T>
static inline void SET_FIELD(T& data, uint8_t start_bit, uint8_t len_bits, T value) {
    T bitmask = (1UL << len_bits) - 1;
    data &= ~(bitmask << start_bit);
    data |= value << start_bit;
}

void uwb_init();

esp_err_t uwb_read_reg(uint8_t reg, uint8_t* rx, size_t len, spi_device_handle_t dev_handle);
esp_err_t uwb_write_reg(uint8_t reg, uint8_t* tx, size_t len, spi_device_handle_t dev_handle);

esp_err_t uwb_transmit(uint8_t* tx, size_t len, spi_device_handle_t dev_handle);

#endif