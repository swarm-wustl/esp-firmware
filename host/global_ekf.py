#!/usr/bin/env python3

# the global filter: the board's twist and uwb_fix's position in, the robot's
# pose in map and map -> odom out. It replaces robot_localization's global
# instance because that one's state is fixed at pose and its derivatives, and
# the errors that matter here are biases: the gyro's, and the floor's slip.
# Process noise can only say "you might be off", and only by root t; a bias in
# the state is learned from the fixes and taken off the twist, so the track
# keeps up through a gap in UWB rather than drifting at the full bias

import math
from dataclasses import dataclass

import numpy as np
import rclpy
from geometry_msgs.msg import (PoseWithCovarianceStamped, TransformStamped,
                               TwistWithCovarianceStamped)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

X, Y, THETA, GYRO_BIAS, SLIP = range(5)
N = 5

# 99.99% point of chi-square with 2 degrees of freedom: a fix further than
# this from the track, in its own and the filter's combined spread, is wrong
GATE = 18.42

MAX_STEP = 1.0  # s; a longer silence is not integrated at the last twist


@dataclass(frozen=True)
class GaussMarkov:
    sigma: float
    correlation_time: float  # s

    def keep(self, dt):
        return math.exp(-dt / self.correlation_time)

    # held at sigma whatever dt is, as World::wander does
    def variance(self, dt):
        return self.sigma ** 2 * (1.0 - self.keep(dt) ** 2)


@dataclass(frozen=True)
class Model:
    gyro_bias: GaussMarkov
    # the fraction of forward speed the floor takes or gives. One number,
    # not one per wheel: the board's yaw rate comes from the gyro, which turns
    # with the robot whatever its wheels do, so uneven slip never reaches
    # heading and only the common part is left, as a scale on v
    slip: GaussMarkov
    # m^2/s: what is left that neither bias explains
    wander: float


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


# twist is (vx, vy, wz) as the board measured it over dt, with its 3x3
# covariance. The step is the midpoint one Odometry::integrate takes
def predict(state, cov, twist, twist_cov, dt, model):
    x, y, theta, bias, slip = state
    vx, vy, wz = twist
    scale = 1.0 + slip
    w = wz - bias
    mid = theta + w * dt / 2.0
    c, s = math.cos(mid), math.sin(mid)
    forward = scale * vx * dt
    lateral = vy * dt

    moved = np.array([
        x + forward * c - lateral * s,
        y + forward * s + lateral * c,
        wrap(theta + w * dt),
        bias * model.gyro_bias.keep(dt),
        slip * model.slip.keep(dt),
    ])

    # how far the midpoint heading turns for each unit of the input or state
    # that moves it, and what that does to x and y
    dx_dmid = -forward * s - lateral * c
    dy_dmid = forward * c - lateral * s

    f = np.eye(N)
    f[X, THETA], f[Y, THETA] = dx_dmid, dy_dmid
    f[X, GYRO_BIAS], f[Y, GYRO_BIAS] = -dx_dmid * dt / 2.0, -dy_dmid * dt / 2.0
    f[X, SLIP], f[Y, SLIP] = vx * dt * c, vx * dt * s
    f[THETA, GYRO_BIAS] = -dt
    f[GYRO_BIAS, GYRO_BIAS] = model.gyro_bias.keep(dt)
    f[SLIP, SLIP] = model.slip.keep(dt)

    g = np.zeros((N, 3))
    g[X] = [scale * dt * c, -dt * s, dx_dmid * dt / 2.0]
    g[Y] = [scale * dt * s, dt * c, dy_dmid * dt / 2.0]
    g[THETA, 2] = dt

    q = g @ twist_cov @ g.T
    q[X, X] += model.wander * dt
    q[Y, Y] += model.wander * dt
    q[GYRO_BIAS, GYRO_BIAS] += model.gyro_bias.variance(dt)
    q[SLIP, SLIP] += model.slip.variance(dt)

    return moved, f @ cov @ f.T + q


# a position fix in map. Returns None for a fix the gate turns away, so the
# caller can say so
def update(state, cov, position, position_cov):
    h = np.zeros((2, N))
    h[0, X] = h[1, Y] = 1.0

    innovation = np.asarray(position) - state[[X, Y]]
    spread = h @ cov @ h.T + position_cov
    distance = float(innovation @ np.linalg.solve(spread, innovation))

    if distance > GATE:
        return None

    gain = cov @ h.T @ np.linalg.inv(spread)
    corrected = state + gain @ innovation
    corrected[THETA] = wrap(corrected[THETA])
    # Joseph form: stays symmetric and positive under rounding
    keep = np.eye(N) - gain @ h

    return corrected, keep @ cov @ keep.T + gain @ position_cov @ gain.T


# takes a fix's position whole and forgets what the old one said about the
# rest of the state
def relocate(state, cov, position, position_cov):
    moved = state.copy()
    moved[[X, Y]] = position
    reset = cov.copy()
    reset[[X, Y], :] = 0.0
    reset[:, [X, Y]] = 0.0
    reset[np.ix_([X, Y], [X, Y])] = position_cov

    return moved, reset


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    ax, ay, at = a
    bx, by, bt = b
    return (ax + math.cos(at) * bx - math.sin(at) * by,
            ay + math.sin(at) * bx + math.cos(at) * by, wrap(at + bt))


def invert(a):
    x, y, t = a
    return (-math.cos(t) * x - math.sin(t) * y,
            math.sin(t) * x - math.cos(t) * y, -t)


def seconds(stamp):
    return Time.from_msg(stamp).nanoseconds * 1e-9


class GlobalEkf(Node):
    def __init__(self):
        super().__init__("global_ekf")

        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_link_frame", "base_link").value

        # the sim's own models are tighter and faster; these say what an
        # unknown floor and a calibrated gyro could plausibly do, slowly
        self.model = Model(
            gyro_bias=GaussMarkov(
                self.declare_parameter("gyro_bias_sigma", 0.005).value,
                self.declare_parameter("gyro_bias_time", 120.0).value),
            slip=GaussMarkov(
                self.declare_parameter("slip_sigma", 0.05).value,
                self.declare_parameter("slip_time", 120.0).value),
            wander=self.declare_parameter("wander", 1e-4).value)

        start = self.declare_parameter("initial_state", [0.0, 0.0, 0.0]).value
        start_sigma = self.declare_parameter("initial_sigma", [0.05, 0.05, 0.05]).value
        self.state = np.array([*start, 0.0, 0.0])
        self.cov = np.diag([*(s * s for s in start_sigma),
                            self.model.gyro_bias.sigma ** 2,
                            self.model.slip.sigma ** 2])
        # s of nothing but gated fixes before the filter takes the fixes'
        # word over its own: a track the gate trusts too much would otherwise
        # turn every fix away for good
        self.relock = self.declare_parameter("relock_after", 3.0).value
        self.gated_since = None
        self.at = None
        self.twist = np.zeros(3)
        self.twist_cov = np.zeros((3, 3))

        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.broadcaster = TransformBroadcaster(self)

        # best-effort, as micro-ROS publishes it
        self.create_subscription(TwistWithCovarianceStamped, "odom/twist",
                                 self.on_twist, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, "uwb/pose",
                                 self.on_fix, 10)
        self.odometry = self.create_publisher(Odometry, "odometry/global", 10)
        self.bias_pub = self.create_publisher(Float64, "global_ekf/gyro_bias", 10)
        self.slip_pub = self.create_publisher(Float64, "global_ekf/slip", 10)
        self.create_timer(1.0 / self.declare_parameter("frequency", 30.0).value,
                          self.publish_transform)

    # each twist is the mean over the period it closes, so it moves the state
    # up to its own stamp. A fix between twists is met by carrying the last
    # twist forward to it
    def advance(self, to):
        if self.at is not None:
            dt = min(to - self.at, MAX_STEP)
            if dt > 0.0:
                self.state, self.cov = predict(self.state, self.cov, self.twist,
                                               self.twist_cov, dt, self.model)

        self.at = to if self.at is None else max(self.at, to)

    def on_twist(self, msg):
        t = msg.twist.twist
        c = msg.twist.covariance
        self.twist = np.array([t.linear.x, t.linear.y, t.angular.z])
        self.twist_cov = np.array([[c[0], c[1], c[5]],
                                   [c[6], c[7], c[11]],
                                   [c[30], c[31], c[35]]])
        self.advance(seconds(msg.header.stamp))
        self.publish(msg.header.stamp)

    def on_fix(self, msg):
        if msg.header.frame_id != self.map_frame:
            self.get_logger().warn(f"fix in {msg.header.frame_id}, not {self.map_frame}",
                                   throttle_duration_sec=5.0)
            return

        self.advance(seconds(msg.header.stamp))
        p = msg.pose.pose.position
        c = msg.pose.covariance
        result = update(self.state, self.cov, (p.x, p.y),
                        np.array([[c[0], c[1]], [c[6], c[7]]]))

        if result is not None:
            self.state, self.cov = result
            self.gated_since = None
            return

        self.gated_since = self.gated_since or self.at

        if self.at - self.gated_since < self.relock:
            self.get_logger().info(f"gated fix ({p.x:.2f}, {p.y:.2f})",
                                   throttle_duration_sec=1.0)
            return

        self.get_logger().warn(f"relocked on ({p.x:.2f}, {p.y:.2f}) after "
                               f"{self.relock:.0f} s of gated fixes")
        self.state, self.cov = relocate(self.state, self.cov, (p.x, p.y),
                                        np.array([[c[0], c[1]], [c[6], c[7]]]))
        self.gated_since = None

    def publish(self, stamp):
        x, y, theta, bias, slip = self.state
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        pose_cov = [0.0] * 36
        for i, a in enumerate((X, Y, None, None, None, THETA)):
            for j, b in enumerate((X, Y, None, None, None, THETA)):
                if a is not None and b is not None:
                    pose_cov[i * 6 + j] = self.cov[a, b]
        msg.pose.covariance = pose_cov

        msg.twist.twist.linear.x = (1.0 + slip) * self.twist[0]
        msg.twist.twist.linear.y = self.twist[1]
        msg.twist.twist.angular.z = self.twist[2] - bias
        self.odometry.publish(msg)

        self.bias_pub.publish(Float64(data=float(bias)))
        self.slip_pub.publish(Float64(data=float(slip)))

    # map -> odom is what's left of map -> base once the local filter's
    # odom -> base is taken off, both at the time the state is for
    def publish_transform(self):
        if self.at is None:
            return

        at = Time(nanoseconds=int(self.at * 1e9))
        try:
            local = self.tf.lookup_transform(self.odom_frame, self.base_frame, at)
        except TransformException:
            try:
                local = self.tf.lookup_transform(self.odom_frame, self.base_frame, Time())
            except TransformException as e:
                self.get_logger().warn(f"no {self.odom_frame} -> {self.base_frame}: {e}",
                                       throttle_duration_sec=2.0)
                return

        l = local.transform
        odom_to_base = (l.translation.x, l.translation.y, yaw_of(l.rotation))
        x, y, t = compose(tuple(self.state[[X, Y, THETA]]), invert(odom_to_base))

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.odom_frame
        msg.transform.translation.x = x
        msg.transform.translation.y = y
        msg.transform.rotation.z = math.sin(t / 2.0)
        msg.transform.rotation.w = math.cos(t / 2.0)
        self.broadcaster.sendTransform(msg)


def main():
    rclpy.init()
    rclpy.spin(GlobalEkf())


if __name__ == "__main__":
    main()
