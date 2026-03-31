#include "ros.h"
#include "esp_mac.h"
#include <stdio.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <builtin_interfaces/msg/time.h>

#include "error.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "queue.h"
#include "esp_timer.h"

static const uint32_t MSG_SIZE = 512;

#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      printf("Failed status on line %d: %d. Aborting.\n", __LINE__,            \
             (int)temp_rc);                                                    \
      vTaskDelete(NULL);                                                       \
    }                                                                          \
  }

// Context for twist subscriber callback
struct TwistCallbackContext {
  Consumer::QueueType *queue;
};

// Context for IMU timer callback (static since timer callbacks don't support user data)
static struct {
  HW::IMUSensor *imu;
  rcl_publisher_t *publisher;
  sensor_msgs__msg__Imu *msg;
} g_imu_ctx;

static void twist_callback(const void *msgin, void *context) {
  printf("Received message!\n");

  // Cast void pointer parameters
  const geometry_msgs__msg__Twist twist_msg =
      *reinterpret_cast<const geometry_msgs__msg__Twist *>(msgin);
  TwistCallbackContext *ctx = reinterpret_cast<TwistCallbackContext *>(context);

  std::array<Motor::Command, HW::MOTOR_COUNT> motor_commands =
      HW::DriveStyle::convert_twist<HW::MOTOR_COUNT>(twist_msg);

  // Send a message to the queue for each motor command in the array
  for (Motor::Command cmd : motor_commands) {
    ctx->queue->pushToQueue(Consumer::MessageTag::MOTOR_COMMAND,
                            Consumer::MessageBody{.motor_cmd = cmd});
  }
}

static void imu_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  (void)timer;

  if (g_imu_ctx.imu == NULL || g_imu_ctx.publisher == NULL || g_imu_ctx.msg == NULL) {
    return;
  }

  // Read IMU data
  HAL::IMUData imu_data = g_imu_ctx.imu->read();

  // Get current time
  int64_t time_us = esp_timer_get_time();
  g_imu_ctx.msg->header.stamp.sec = time_us / 1000000;
  g_imu_ctx.msg->header.stamp.nanosec = (time_us % 1000000) * 1000;

  // Populate acceleration (m/s²)
  g_imu_ctx.msg->linear_acceleration.x = imu_data.accel_x;
  g_imu_ctx.msg->linear_acceleration.y = imu_data.accel_y;
  g_imu_ctx.msg->linear_acceleration.z = imu_data.accel_z;

  // Populate angular velocity (rad/s)
  g_imu_ctx.msg->angular_velocity.x = imu_data.gyro_x;
  g_imu_ctx.msg->angular_velocity.y = imu_data.gyro_y;
  g_imu_ctx.msg->angular_velocity.z = imu_data.gyro_z;

  // Orientation is not available (no sensor fusion)
  // Set covariance[0] to -1 to indicate unknown
  g_imu_ctx.msg->orientation.x = 0;
  g_imu_ctx.msg->orientation.y = 0;
  g_imu_ctx.msg->orientation.z = 0;
  g_imu_ctx.msg->orientation.w = 1;  // Identity quaternion
  g_imu_ctx.msg->orientation_covariance[0] = -1;

  // Publish
  rcl_ret_t ret __attribute__((unused)) = rcl_publish(g_imu_ctx.publisher, g_imu_ctx.msg, NULL);
}

void ROS::spin(Consumer::QueueType &queue, HW::IMUSensor &imu) {
  // Create ID from MAC address
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);

  // uses the last 2 bytes, can be increased
  char node_namespace[20];
  snprintf(node_namespace, sizeof(node_namespace), "/swarmbot_%02x%02x", mac[4],
           mac[5]);

  log("Namespace generated: %s", node_namespace);

  // Initialize IMU
  if (!imu.initialize()) {
    log("Failed to initialize IMU");
  }

  // Create memory allocator
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;

  // Create default init options
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
  rmw_init_options_t *rmw_options =
      rcl_init_options_get_rmw_init_options(&init_options);

  // Static Agent IP and port can be used instead of autodisvery.
  RCCHECK(rmw_uros_options_set_udp_address(
      CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
#endif

  // Create init_options
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options,
                                         &allocator));

  // Create node with namespace
  rcl_node_t node;
  RCCHECK(rclc_node_init_default(&node, "swarm", node_namespace, &support));

  // Create subscriber
  rcl_subscription_t subscriber;
  RCCHECK(rclc_subscription_init_default(
      &subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

  // Create IMU publisher
  rcl_publisher_t imu_publisher;
  RCCHECK(rclc_publisher_init_default(
      &imu_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data"));

  // Create IMU timer (20ms period = 50Hz)
  rcl_timer_t imu_timer;
  const unsigned int imu_timer_period_ms = 20;
  RCCHECK(rclc_timer_init_default(
      &imu_timer, &support, RCL_MS_TO_NS(imu_timer_period_ms), imu_timer_callback));

  // Initialize IMU message
  sensor_msgs__msg__Imu imu_msg;
  sensor_msgs__msg__Imu__init(&imu_msg);

  // Set frame_id
  static const char frame_id[] = "imu_link";
  imu_msg.header.frame_id.data = const_cast<char *>(frame_id);
  imu_msg.header.frame_id.size = sizeof(frame_id) - 1;
  imu_msg.header.frame_id.capacity = sizeof(frame_id);

  // Set covariances to indicate unknown (-1 in first element)
  imu_msg.orientation_covariance[0] = -1;
  imu_msg.angular_velocity_covariance[0] = 0;
  imu_msg.linear_acceleration_covariance[0] = 0;

  // Create callback contexts
  static TwistCallbackContext twist_ctx;
  twist_ctx.queue = &queue;

  // Set up global IMU context for timer callback
  g_imu_ctx.imu = &imu;
  g_imu_ctx.publisher = &imu_publisher;
  g_imu_ctx.msg = &imu_msg;

  // Create executor with 2 handles (subscriber + timer)
  rclc_executor_t executor;
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

  // Add subscriber to executor
  geometry_msgs__msg__Twist msgin;
  RCCHECK(rclc_executor_add_subscription_with_context(
      &executor, &subscriber, &msgin, &twist_callback, (void *)&twist_ctx, ON_NEW_DATA));

  // Add timer to executor
  RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));

  rclc_executor_spin(&executor);

  vTaskDelete(NULL);
}
