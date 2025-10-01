#include "ros.h"

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#include <geometry_msgs/msg/twist.h>

#include "error.h"
#include "queue.h"
#include "consumer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const uint32_t MSG_SIZE = 512;

#define RCCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { \
        printf("Failed status on line %d: %d. Aborting.\n",__LINE__, (int)temp_rc); \
        vTaskDelete(NULL); \
    } \
}

static void callback(const void* msgin, void* context) {
    printf("Received message!\n");

    // Cast void pointer parameters
    const geometry_msgs__msg__Twist twist_msg = *reinterpret_cast<const geometry_msgs__msg__Twist*>(msgin);
    Queue<Consumer::MessageTag, Consumer::MessageBody, Consumer::CONSUMER_QUEUE_SIZE> queue = *reinterpret_cast<Queue<Consumer::MessageTag, Consumer::MessageBody, Consumer::CONSUMER_QUEUE_SIZE>*>(context);

    std::array<Motor::Command, HW::MOTOR_COUNT> motor_commands = HW::DriveStyle::convert_twist<HW::MOTOR_COUNT>(twist_msg);

    // Send a message to the queue for each motor command in the array
    for (Motor::Command cmd : motor_commands) {
        queue.pushToQueue(Consumer::MessageTag::MOTOR_COMMAND, Consumer::MessageBody{ .motor_cmd = cmd });
    }
}

void ROS::spin(void* context) {
    // Create memory allocator
    rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

    // Create default init options
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
#endif

    // Create init_options
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // Create node
    rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "uros_node", "", &support));

    // Create publisher
    rcl_publisher_t publisher;
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), // TODO
		"uros_topic"
    ));

    // Create subscriber
    rcl_subscription_t subscriber;
    RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"uros_topic"
    ));

    // Create executor with a single handle
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    // Add subscriber to executor
    geometry_msgs__msg__Twist msgin;
    RCCHECK(rclc_executor_add_subscription_with_context(
        &executor, 
        &subscriber, 
        &msgin,
		&callback, 
        context,
        ON_NEW_DATA
    ));

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)); // TODO: time constant
    }

    vTaskDelete(NULL);
}