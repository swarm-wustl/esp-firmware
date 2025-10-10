#include "sensor.h"
#include "error.h"

TaskHandle_t Sensor::sensorTaskHandle = nullptr;

Sensor::Sensor(uint32_t id, const char* name, uint32_t period_ms) : sensor_id(id) {
    if (Sensor::sensorTaskHandle != nullptr) {
        TimerHandle_t timer = xTimerCreate(
            name,
            pdMS_TO_TICKS(period_ms),
            pdTRUE,
            (void*)id,
            Sensor::registerSensorTimer
        );

        // TODO: error handle
        xTimerStart(timer, 0);
    }
}

void Sensor::registerSensorTaskHandle(TaskHandle_t taskHandle) {
    Sensor::sensorTaskHandle = taskHandle;
}

void Sensor::registerSensorTimer(TimerHandle_t timer) {
    void* timer_id = pvTimerGetTimerID(timer);
    uint32_t sensor_id = reinterpret_cast<uint32_t>(timer_id);
    xTaskNotifyFromISR(Sensor::sensorTaskHandle, 1 << sensor_id, eSetBits, NULL);
}

void Sensor::spin(void* context) {
    uint32_t notification;

    while (1) {
        if (xTaskNotifyWait(
            0,                  // Clear 0 bits on entry
            ULONG_MAX,          // Clear all bits on exit
            &notification,      // Notification value that stores which bits are set
            portMAX_DELAY       // Wait until any sort of message
        )) {
            if ((notification >> UWB_ID) & 1) {
                log("Received notification for UWB!");
            }
        }
    }
}