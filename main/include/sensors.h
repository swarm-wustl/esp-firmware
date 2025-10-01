#ifndef SENSORS_H
#define SENSORS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

constexpr uint32_t UWB_ID = 0;
constexpr uint32_t IMU_ID = 1;

namespace Sensors {
    void spin(void* context);
    void registerSensorTimer(TimerHandle_t timer);
    void registerSensorTaskHandle(TaskHandle_t taskHandle);
};

#endif
