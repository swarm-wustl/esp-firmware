#!/usr/bin/env python3

# stands in for the board: same node name, topics, message types and QoS as
# main/src/main.cpp, so anything that works against this works against the robot

import math
import random
import time

import rclpy
from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
    TransformStamped,
    Twist,
)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from tf2_msgs.msg import TFMessage

# HW:: constants in main/src/main.cpp, which sdkconfig doesn't carry
MAX_WHEEL_SPEED = 43.0 * 8.0 / 6.0 * 2.0 * math.pi / 60.0
CMD_VEL_TIMEOUT = 0.5
CONTROL_PERIOD = 0.02
ODOM_PERIOD = 0.2
RANGE_PERIOD = 0.1
POSE_VARIANCE = [0.01, 0.01, 1e6, 1e6, 1e6, 0.05]


def clamp(value, limit):
    return max(-limit, min(limit, value))


def advance(pose, v, w, dt):
    x, y, theta = pose
    mid = theta + w * dt / 2.0
    return (x + v * math.cos(mid) * dt, y + v * math.sin(mid) * dt, theta + w * dt)


def quaternion(theta):
    return math.sin(theta / 2.0), math.cos(theta / 2.0)


class FakeBase(Node):
    def __init__(self):
        super().__init__("base")

        self.radius = self.declare_parameter("wheel_radius", 0.033).value
        self.half_track = self.declare_parameter("half_track", 0.075).value
        # per-wheel speed error the encoders don't see, so odom drifts from truth
        self.slip = self.declare_parameter("slip", 0.05).value
        self.anchor = self.declare_parameter("anchor", [2.0, 0.0]).value
        self.range_noise = self.declare_parameter("range_noise", 0.05).value

        self.command = Twist()
        self.last_command = -math.inf
        self.truth = (0.0, 0.0, 0.0)
        self.odom = (0.0, 0.0, 0.0)

        self.create_subscription(
            Twist, "cmd_vel", self.on_cmd_vel, qos_profile_sensor_data
        )
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "odom/pose", qos_profile_sensor_data
        )
        self.range_pub = self.create_publisher(
            Range, "uwb/range", qos_profile_sensor_data
        )
        self.truth_pub = self.create_publisher(PoseStamped, "sim/truth", 10)
        # sim-only, like sim/truth: the board publishes no TF, and the EKF will
        # own odom -> base_link, so this frame must not be base_link
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 100)
        self.truth_frame = f"{self.get_namespace().strip('/')}/truth"

        self.create_timer(CONTROL_PERIOD, self.step)
        self.create_timer(ODOM_PERIOD, self.publish_pose)
        self.create_timer(RANGE_PERIOD, self.publish_range)

    def on_cmd_vel(self, msg):
        self.command = msg
        self.last_command = time.monotonic()

    def wheel_targets(self):
        if time.monotonic() - self.last_command > CMD_VEL_TIMEOUT:
            return 0.0, 0.0

        v, w = self.command.linear.x, self.command.angular.z
        left = (v - self.half_track * w) / self.radius
        right = (v + self.half_track * w) / self.radius

        return clamp(left, MAX_WHEEL_SPEED), clamp(right, MAX_WHEEL_SPEED)

    def body_twist(self, left, right):
        return (
            self.radius * (left + right) / 2.0,
            self.radius * (right - left) / (2.0 * self.half_track),
        )

    def step(self):
        left, right = self.wheel_targets()

        actual_left = left * random.gauss(1.0, self.slip)
        actual_right = right * random.gauss(1.0, self.slip)

        self.odom = advance(self.odom, *self.body_twist(left, right), CONTROL_PERIOD)
        self.truth = advance(
            self.truth, *self.body_twist(actual_left, actual_right), CONTROL_PERIOD
        )

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = self.truth_frame
        transform.transform.translation.x = self.truth[0]
        transform.transform.translation.y = self.truth[1]
        transform.transform.rotation.z, transform.transform.rotation.w = quaternion(
            self.truth[2]
        )
        self.tf_pub.publish(TFMessage(transforms=[transform]))

    def publish_pose(self):
        x, y, theta = self.odom
        z, w = quaternion(theta)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = z
        msg.pose.pose.orientation.w = w
        msg.pose.covariance = [
            POSE_VARIANCE[i // 6] if i % 7 == 0 else 0.0 for i in range(36)
        ]
        self.pose_pub.publish(msg)

        truth = PoseStamped()
        truth.header = msg.header
        truth.pose.position.x, truth.pose.position.y = self.truth[0], self.truth[1]
        truth.pose.orientation.z, truth.pose.orientation.w = quaternion(self.truth[2])
        self.truth_pub.publish(truth)

    def publish_range(self):
        distance = math.dist(self.truth[:2], self.anchor)

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "uwb_link"
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 6.2831853
        msg.min_range = 0.0
        msg.max_range = 300.0
        msg.range = max(0.0, distance + random.gauss(0.0, self.range_noise))
        self.range_pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(FakeBase())


if __name__ == "__main__":
    main()
