#include "ros.h"

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#include <geometry_msgs/msg/twist.h>

#include <uros_network_interfaces.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "queue.h"

static const char *TAG = "ros";

namespace {
struct CallbackContext {
  Consumer::QueueType &queue;
  ROS::TwistHandler on_twist;
};
} // namespace

#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      ESP_LOGE(TAG, "Failed status on line %d: %d. Aborting.", __LINE__,       \
               (int)temp_rc);                                                  \
      vTaskDelete(NULL);                                                       \
    }                                                                          \
  }

static void callback(const void *msgin, void *context) {
  const geometry_msgs__msg__Twist &twist_msg =
      *reinterpret_cast<const geometry_msgs__msg__Twist *>(msgin);
  CallbackContext &ctx = *reinterpret_cast<CallbackContext *>(context);

  ctx.on_twist(twist_msg, ctx.queue);
}

void ROS::spin(Consumer::QueueType &queue, TwistHandler on_twist) {
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
  rmw_init_options_t *rmw_options =
      rcl_init_options_get_rmw_init_options(&init_options);

  // Static Agent IP and port can be used instead of autodisvery.
  RCCHECK(rmw_uros_options_set_udp_address(
      CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
#endif

  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options,
                                         &allocator));

  rcl_node_t node;
  RCCHECK(rclc_node_init_default(&node, "uros_node", "", &support));

  rcl_subscription_t subscriber;
  RCCHECK(rclc_subscription_init_default(
      &subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "uros_topic"));

  rclc_executor_t executor;
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  geometry_msgs__msg__Twist msgin;
  CallbackContext ctx{queue, on_twist};
  RCCHECK(rclc_executor_add_subscription_with_context(
      &executor, &subscriber, &msgin, &callback, &ctx, ON_NEW_DATA));

  rclc_executor_spin(&executor);

  vTaskDelete(NULL);
}
