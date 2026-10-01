#include "session.h"

#include "esp_log.h"

static const char *TAG = "ros";

bool ROS::check(rcl_ret_t status, const char *what) {
  if (status == RCL_RET_OK) {
    return true;
  }

  ESP_LOGE(TAG, "%s failed: %d (%s)", what, static_cast<int>(status),
           rcl_get_error_string().str);
  rcl_reset_error();

  return false;
}
