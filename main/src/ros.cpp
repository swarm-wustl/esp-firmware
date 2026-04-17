#include "ros.h"
#include "i2c.cpp"
#include <cmath>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>

#include "error.h"
#include "queue.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const uint32_t MSG_SIZE = 512;
static const double G = 9.81;

#define LSBSENS_GYRO 131 //LSB sensitivity for the gyroscope
#define LSBSENS_ACCEL 16384 //LSB sensitivity for the accelerometer

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
    Consumer::QueueType& queue = *reinterpret_cast<Consumer::QueueType*>(context);

    std::array<Motor::Command, HW::MOTOR_COUNT> motor_commands = HW::DriveStyle::convert_twist<HW::MOTOR_COUNT>(twist_msg);

    // Send a message to the queue for each motor command in the array
    for (Motor::Command cmd : motor_commands) {
        queue.pushToQueue(Consumer::MessageTag::MOTOR_COMMAND, Consumer::MessageBody{ .motor_cmd = cmd });
    }
}

void ROS::imu_publish(void *arg){
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_node_t node;
    rcl_publisher_t publisher;

    sensor_msgs__msg__Imu imu_msg;

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    RCCHECK(rclc_node_init_default(&node, "imu_node", "", &support));

    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"
    ));

    printf("IMU Publisher Started\n");

    // -----------------------
    // IMU INITIALIZATION
    // -----------------------

    uint8_t data[2];

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_ERROR_CHECK(imu_register_write_byte(IMU_PWR_MGMT_1, 0x00));

    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_ERROR_CHECK(mpu6050_register_read(IMU_WHO_AM_I_ADDR , data, 1));
    printf("WHO_AM_I = %X\n", data[0]);

    while (true){
        int16_t gyroX, gyroY, gyroZ;
        int16_t accelX, accelY, accelZ;

        if (imu_read_gyroscope_data(&gyroX, &gyroY, &gyroZ) == ESP_OK &&
            imu_read_accelerometer_data(&accelX, &accelY, &accelZ) == ESP_OK) {

                float gx = ((float)gyroX) / LSBSENS_GYRO;
                float gy = ((float)gyroY) / LSBSENS_GYRO;
                float gz = ((float)gyroZ) / LSBSENS_GYRO;

                float ax = ((float)accelX) / LSBSENS_ACCEL;
                float ay = ((float)accelY) / LSBSENS_ACCEL;
                float az = ((float)accelZ) / LSBSENS_ACCEL;

                    // ROS expects rad/s
                gx *= (M_PI / 180.0);
                gy *= (M_PI / 180.0);
                gz *= (M_PI / 180.0);

                imu_msg.angular_velocity.x = gx;
                imu_msg.angular_velocity.y = gy;
                imu_msg.angular_velocity.z = gz;

                imu_msg.linear_acceleration.x = ax * G;
                imu_msg.linear_acceleration.y = ay * G;
                imu_msg.linear_acceleration.z = az * G;

                rcl_publish(&publisher, &imu_msg, NULL);
            }

        vTaskDelay(pdMS_TO_TICKS(100));

    }

    ESP_ERROR_CHECK(imu_register_write_byte(IMU_PWR_MGMT_1, 1 << IMU_PWR_MGMT_1_RESET_BIT));
    ESP_LOGI(TAG, "I2C written successfully");

    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C unitialized successfully");
    

}
void ROS::spin(Consumer::QueueType& queue) {
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
        (void*)&queue,
        ON_NEW_DATA
    ));

    rclc_executor_spin(&executor);

    vTaskDelete(NULL);
}