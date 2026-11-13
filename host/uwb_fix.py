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

# 99% points of chi-square by degrees of freedom (anchors - 2). A fix whose
# squared residuals sum past this is not explained by the noise alone
CHI2_99 = {1: 6.63, 2: 9.21, 3: 11.34, 4: 13.28, 5: 15.09, 6: 16.81}


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


def residuals(observations, position):
    x, y = position
    return [r - math.hypot(x - ax, y - ay) for (ax, ay), r in observations]


def misfit(observations, position, sigma):
    return sum(e * e for e in residuals(observations, position)) / (sigma * sigma)


# drops one anchor at a time until the rest agree. Largest residual is the
# wrong pick: least squares spreads one bad range over the others, and in a
# square a long read on one corner fits much like a short read on the
# opposite one. So each anchor is left out in turn and the rest refit, and the
# one dropped is the one whose absence leaves the best fit -- provided it
# reads long against that fit, since a blocked path makes the radio lock onto
# a reflection and only ever adds. Three anchors leave one degree of freedom,
# so a set still holding the bad range can fit as well as the clean one; the
# previous fix, when there is a recent one, breaks the tie, since the robot
# moves centimetres between fixes. With three left there is no redundancy to
# say which is wrong, so a misfit there is no fix
def consistent_fix(observations, guess, sigma, prior=None):
    result = trilaterate(observations, guess, sigma)

    if result is None:
        return None, []

    position, _ = result
    gate = CHI2_99.get(len(observations) - 2, math.inf)

    if misfit(observations, position, sigma) <= gate:
        return result, []

    if len(observations) <= MIN_ANCHORS:
        return None, list(observations)

    candidates = []

    for i, ((ax, ay), r) in enumerate(observations):
        rest = observations[:i] + observations[i + 1:]
        fit = trilaterate(rest, position, sigma)

        if fit is None:
            continue

        (x, y), _ = fit

        if r > math.hypot(x - ax, y - ay):
            score = misfit(rest, (x, y), sigma)
            if prior:
                score += (math.hypot(x - guess[0], y - guess[1]) / prior) ** 2
            candidates.append((score, i))

    if not candidates:
        return None, list(observations)

    _, worst = min(candidates)
    result, dropped = consistent_fix(
        observations[:worst] + observations[worst + 1:], guess, sigma, prior)

    return result, [observations[worst]] + dropped


class UwbFix(Node):
    def __init__(self):
        super().__init__("uwb_fix")

        self.map_frame = self.declare_parameter("map_frame", "map").value
        # the antenna-delay bias goes into the fix's sigma alongside the
        # noise: it is constant per link, so it is not independent from fix to
        # fix, but leaving it out tells the EKF the fix is better than it is
        noise = self.declare_parameter("range_sigma", 0.05).value
        bias = self.declare_parameter("range_bias", 0.0).value
        self.sigma = math.hypot(noise, bias)
        self.reject = self.declare_parameter("reject_outliers", True).value
        # ranges older than this are dropped from a fix. A robot moving while
        # the anchors are ranged in turn smears the fix by speed * max_age, so
        # keep it just over one round of anchors
        self.max_age = self.declare_parameter("max_age", 0.5).value
        # how far the robot plausibly is from the last fix, and how old that
        # fix may be before it says nothing about where the robot is now
        self.prior = self.declare_parameter("prior_sigma", 0.15).value
        self.prior_age = self.declare_parameter("prior_age", 1.0).value

        self.anchors = {}
        self.latest = {}
        self.guess = None
        self.guessed = None

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

        fresh = {
            frame: (anchor, r) for frame, (anchor, r, heard) in self.latest.items()
            if (stamp - heard).nanoseconds * 1e-9 <= self.max_age
        }
        observations = list(fresh.values())

        if len(observations) < MIN_ANCHORS:
            return

        recent = (self.guessed is not None and
                  (stamp - self.guessed).nanoseconds * 1e-9 <= self.prior_age)
        guess = self.guess if recent else tuple(
            sum(a[i] for a, _ in observations) / len(observations)
            for i in range(2))

        if self.reject:
            result, dropped = consistent_fix(observations, guess, self.sigma,
                                             self.prior if recent else None)
            self.report(fresh, dropped)
        else:
            result = trilaterate(observations, guess, self.sigma)

        if result is None:
            return

        (x, y), cov = result
        self.guess = (x, y)
        self.guessed = stamp
        self.fix.publish(self.pose(msg.header.stamp, x, y, cov))

    def report(self, fresh, dropped):
        if not dropped:
            return

        names = [f for f, o in fresh.items() if o in dropped]
        self.get_logger().info(f"rejected {', '.join(names)}",
                               throttle_duration_sec=1.0)

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
