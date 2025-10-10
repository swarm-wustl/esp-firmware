#ifndef SENSORS_H
#define SENSORS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include <array>

constexpr uint32_t UWB_ID = 0;
constexpr uint32_t IMU_ID = 1;
constexpr uint32_t SENSOR_COUNT = 2;

// TODO: consider making this a class per sensor where each has a single StaticTimer_t
// can avoid all this weird extern stuff then
namespace Sensors {
    extern StaticTimer_t sensor_buffers[SENSOR_COUNT];
    
    void spin(void* context);
    void registerSensorTimer(TimerHandle_t timer);
    void registerSensorTaskHandle(TaskHandle_t taskHandle);
};

#endif
