#include "consumer.h"
#include "motor.h"
#include "esp32.h"

extern "C" void app_main(void) {
    // Motor::Instance<int>();

    Consumer::spin();
}
