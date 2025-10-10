#include "sensors.h"
#include "error.h"

static TaskHandle_t sensorTaskHandle = NULL;

StaticTimer_t Sensors::sensor_buffers[SENSOR_COUNT];

void Sensors::registerSensorTaskHandle(TaskHandle_t taskHandle) {
    sensorTaskHandle = taskHandle;
}

void Sensors::registerSensorTimer(TimerHandle_t timer) {
    void* timer_id = pvTimerGetTimerID(timer);
    uint32_t sensor_id = reinterpret_cast<uint32_t>(timer_id);
    xTaskNotifyFromISR(sensorTaskHandle, 1 << sensor_id, eSetBits, NULL);
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
            if ((notification >> UWB_ID) & 1) {
                log("Received notification for UWB!");
            }
        }
    }
}