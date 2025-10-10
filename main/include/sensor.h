#ifndef SENSOR_H
#define SENSOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include <array>

constexpr uint32_t UWB_ID = 0;
constexpr uint32_t IMU_ID = 1;
constexpr uint32_t SENSOR_COUNT = 2;

class Sensor {
private:
    static TaskHandle_t sensorTaskHandle;
    uint32_t sensor_id;

public:
    Sensor(uint32_t id, const char* name, uint32_t period_ms);
    ~Sensor() = default;

    static void spin(void* context);
    static void registerSensorTimer(TimerHandle_t timer);
    static void registerSensorTaskHandle(TaskHandle_t taskHandle);
};

#endif
