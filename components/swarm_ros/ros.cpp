#include "session.h"

#include "esp_log.h"

static const char *TAG = "ros";

bool ROS::check(rcl_ret_t status, const char *what) {
  if (status == RCL_RET_OK) {
    return true;
  }

  // libmicroros is built with RCUTILS_AVOID_DYNAMIC_ALLOCATION, which shrinks
  // rcl_error_string_t to an unterminated char[1] and compiles out every
  // RCUTILS_SET_ERROR_MSG, so there is no message to print -- only the code
  ESP_LOGE(TAG, "%s failed: %d", what, static_cast<int>(status));
  rcl_reset_error();

  return false;
}
