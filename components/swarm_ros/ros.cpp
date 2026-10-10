#include "session.h"

#include "esp_log.h"

static const char *TAG = "ros";

void ROS::report_poll(uint32_t polls, uint32_t errors, int last) {
  ESP_LOGI(TAG, "poll #%lu: %lu spin errors, last rc=%d",
           static_cast<unsigned long>(polls),
           static_cast<unsigned long>(errors), last);
}

bool ROS::check(rcl_ret_t status, const char *what) {
  if (status == RCL_RET_OK) {
    return true;
  }

  // rcl_get_error_string() returns the buffer by value; naming it keeps the
  // array alive for the whole log call instead of decaying off a temporary
  const rcl_error_string_t error = rcl_get_error_string();

  ESP_LOGE(TAG, "%s failed: %d (%s)", what, static_cast<int>(status),
           error.str);
  rcl_reset_error();

  return false;
}
