#include <stdio.h>

#include "consumer.h"

extern "C" void app_main(void) {
    printf("Hello from ESP32 C++\n");

    Consumer::spin();
}
