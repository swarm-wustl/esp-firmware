#include "sensors.h"

static TaskHandle_t sensorTaskHandle = NULL;

void Sensors::registerSensorTaskHandle(TaskHandle_t taskHandle) {
    sensorTaskHandle = taskHandle;
}

void Sensors::registerSensorTimer(TimerHandle_t timer) {
    uint32_t sensor_id = *reinterpret_cast<uint32_t*>(timer);

    // TODO: wtf is this for?
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(sensorTaskHandle, 1 << sensor_id, eSetBits, &xHigherPriorityTaskWoken);
}

void Sensors::spin(void* context) {
    uint32_t notification;

    while (1) {
        if (xTaskNotifyWait(
            0,                  // Clear 0 bits on entry
            ULONG_MAX,          // Clear all bits on exit
            &notification,      // Notification value that stores which bits are set
            portMAX_DELAY       // Wait until any sort of message
        )) {

        }
    }
}