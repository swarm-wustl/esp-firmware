#include "ros.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void ROS::spin() {
    rcl_get_zero_initialized_init_options();
}