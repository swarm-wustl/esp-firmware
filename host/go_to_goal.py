#!/usr/bin/env python3

# drives one robot to a point: goal in, cmd_vel out, steering on the EKF's
# estimate. It runs on the host and only speaks topics, so it drives the sim
# and the board alike

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

GAIN_LINEAR = 0.5  # 1/s: full speed until ~0.3 m out, then ease in
GAIN_ANGULAR = 1.5  # 1/s
RATE_HZ = 10.0  # well inside the board's 500 ms cmd_vel watchdog
STALE_ODOMETRY = 0.5  # s


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def command(pose, goal, max_linear, max_angular, tolerance):
    x, y, theta = pose
    dx, dy = goal[0] - x, goal[1] - y
    distance = math.hypot(dx, dy)

    if distance < tolerance:
        return None

    bearing = wrap(math.atan2(dy, dx) - theta)

    # cos(bearing) slows the robot while it is pointed away and stops it
    # outright past 90 degrees, so a goal behind it is turned toward in place
    # rather than reversed onto
    linear = min(max_linear, GAIN_LINEAR * distance) * max(0.0, math.cos(bearing))
    angular = max(-max_angular, min(max_angular, GAIN_ANGULAR * bearing))

    return linear, angular


def in_frame(transform, point):
    t = transform.transform
    yaw = yaw_of(t.rotation)
    x, y = point

    return (t.translation.x + math.cos(yaw) * x - math.sin(yaw) * y,
            t.translation.y + math.sin(yaw) * x + math.cos(yaw) * y)


class GoToGoal(Node):
    def __init__(self):
        super().__init__("go_to_goal")

        # the board tops out near 0.2 m/s, and desaturate scales both wheels
        # down past that, so asking for more only slows the turn
        self.max_linear = self.declare_parameter("max_linear", 0.15).value
        self.max_angular = self.declare_parameter("max_angular", 1.0).value
        self.tolerance = self.declare_parameter("tolerance", 0.05).value

        self.goal = None
        self.odometry = None
        self.heard = -math.inf

        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)

        self.create_subscription(PoseStamped, "goal", self.on_goal, 10)
        self.create_subscription(Odometry, "odometry/filtered", self.on_odometry, 10)
        self.cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_timer(1.0 / RATE_HZ, self.tick)

    def on_goal(self, msg):
        self.goal = msg
        self.get_logger().info(
            f"goal ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}) "
            f"in {msg.header.frame_id or 'odometry frame'}")

    def on_odometry(self, msg):
        self.odometry = msg
        self.heard = time.monotonic()

    def target(self):
        point = (self.goal.pose.position.x, self.goal.pose.position.y)
        source = self.goal.header.frame_id
        frame = self.odometry.header.frame_id

        if not source or source == frame:
            return point

        try:
            return in_frame(self.tf.lookup_transform(frame, source, Time()), point)
        except TransformException as e:
            self.get_logger().warn(f"no {source} -> {frame}: {e}",
                                   throttle_duration_sec=2.0)
            return None

    def stop(self):
        self.cmd_vel.publish(Twist())

    def tick(self):
        if self.goal is None:
            return

        if time.monotonic() - self.heard > STALE_ODOMETRY:
            self.get_logger().warn("odometry is stale; holding",
                                   throttle_duration_sec=2.0)
            self.stop()
            return

        target = self.target()

        if target is None:
            self.stop()
            return

        p = self.odometry.pose.pose
        pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        result = command(pose, target, self.max_linear, self.max_angular,
                         self.tolerance)

        if result is None:
            self.get_logger().info(
                f"arrived at ({pose[0]:.2f}, {pose[1]:.2f})")
            self.stop()
            self.goal = None
            return

        msg = Twist()
        msg.linear.x, msg.angular.z = result
        self.cmd_vel.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(GoToGoal())


if __name__ == "__main__":
    main()
