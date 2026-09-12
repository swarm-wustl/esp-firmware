// Written with Claude
#ifndef ROS_H
#define ROS_H

#include <geometry_msgs/msg/twist.h>

namespace ROS {
using TwistHandler = void (*)(const geometry_msgs__msg__Twist &twist,
                              void *context);

void spin(void *context, TwistHandler on_twist);
} // namespace ROS

#endif
