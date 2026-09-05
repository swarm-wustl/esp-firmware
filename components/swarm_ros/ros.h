#ifndef ROS_H
#define ROS_H

#include <geometry_msgs/msg/twist.h>

#include "consumer.h"

namespace ROS {
using TwistHandler = void (*)(const geometry_msgs__msg__Twist &twist,
                              Consumer::QueueType &queue);

void spin(Consumer::QueueType &queue, TwistHandler on_twist);
} // namespace ROS

#endif
