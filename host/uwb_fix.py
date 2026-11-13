#!/usr/bin/env python3

# turns one robot's UWB ranges into a position fix for the global EKF: ranges
# in, uwb/pose out in map. An anchor is any frame a range names that resolves
# in map, so anchors are declared once, as TF, and nowhere else

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Range
from tf2_ros import Buffer, TransformException, TransformListener

MIN_ANCHORS = 3
ITERATIONS = 10
CONVERGED = 1e-6  # m
UNMEASURED = 1e6  # variance the EKF reads as "ignore", not as certainty


def invert(m):
    (a, b), (c, d) = m
    det = a * d - b * c

    if abs(det) < 1e-9:
        return None

    return ((d / det, -b / det), (-c / det, a / det))


# Gauss-Newton over [((ax, ay), range)]. The covariance is sigma^2 (J^T J)^-1,
# so anchors bunched on one side give a fix that is honestly wide across them
def trilaterate(observations, guess, sigma):
    x, y = guess
    inverse = None

    for _ in range(ITERATIONS):
        jtj = [[0.0, 0.0], [0.0, 0.0]]
        jtr = [0.0, 0.0]

        for (ax, ay), r in observations:
            dx, dy = x - ax, y - ay
            d = math.hypot(dx, dy)

            if d < CONVERGED:
                return None

            u = (dx / d, dy / d)
            residual = r - d

            for i in range(2):
                jtr[i] += u[i] * residual
                for j in range(2):
                    jtj[i][j] += u[i] * u[j]

        inverse = invert(jtj)

        if inverse is None:
            return None

        step = (inverse[0][0] * jtr[0] + inverse[0][1] * jtr[1],
                inverse[1][0] * jtr[0] + inverse[1][1] * jtr[1])
        x, y = x + step[0], y + step[1]

        if math.hypot(*step) < CONVERGED:
            break

    s2 = sigma * sigma
    covariance = ((s2 * inverse[0][0], s2 * inverse[0][1]),
                  (s2 * inverse[1][0], s2 * inverse[1][1]))

    return (x, y), covariance


class UwbFix(Node):
    def __init__(self):
        super().__init__("uwb_fix")

        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.sigma = self.declare_parameter("range_sigma", 0.05).value
        # ranges older than this are dropped from a fix. A robot moving while
        # the anchors are ranged in turn smears the fix by speed * max_age, so
        # keep it just over one round of anchors
        self.max_age = self.declare_parameter("max_age", 0.5).value

        self.anchors = {}
        self.latest = {}
        self.guess = None

        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)

        self.create_subscription(Range, "uwb/range", self.on_range,
                                 qos_profile_sensor_data)
        self.fix = self.create_publisher(PoseWithCovarianceStamped, "uwb/pose", 10)

    def anchor(self, frame):
        if frame not in self.anchors:
            try:
                t = self.tf.lookup_transform(self.map_frame, frame, Time())
            except TransformException as e:
                self.get_logger().warn(f"no {frame} in {self.map_frame}: {e}",
                                       throttle_duration_sec=2.0)
                return None

            self.anchors[frame] = (t.transform.translation.x,
                                   t.transform.translation.y)

        return self.anchors[frame]

    def on_range(self, msg):
        at = self.anchor(msg.header.frame_id)

        if at is None:
            return

        stamp = Time.from_msg(msg.header.stamp)
        self.latest[msg.header.frame_id] = (at, msg.range, stamp)

        observations = [
            (anchor, r) for anchor, r, heard in self.latest.values()
            if (stamp - heard).nanoseconds * 1e-9 <= self.max_age
        ]

        if len(observations) < MIN_ANCHORS:
            return

        guess = self.guess or tuple(
            sum(a[i] for a, _ in observations) / len(observations)
            for i in range(2))
        result = trilaterate(observations, guess, self.sigma)

        if result is None:
            return

        (x, y), cov = result
        self.guess = (x, y)
        self.fix.publish(self.pose(msg.header.stamp, x, y, cov))

    def pose(self, stamp, x, y, cov):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.w = 1.0

        c = [0.0] * 36
        c[0], c[1], c[6], c[7] = cov[0][0], cov[0][1], cov[1][0], cov[1][1]
        for i in (14, 21, 28, 35):
            c[i] = UNMEASURED
        msg.pose.covariance = c

        return msg


def main():
    rclpy.init()
    rclpy.spin(UwbFix())


if __name__ == "__main__":
    main()
