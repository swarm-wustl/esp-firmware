#!/usr/bin/env python3

# drives every robot round a fixed route through the 3D panel's goal topic and
# scores the estimates against sim/truth, so a change to the world, the fix or
# the filters gets a number rather than a look. Run it with
# `scripts/sim.sh check [ns...]` against a sim started with a fixed SIM_SEED.
# Everything the global filter and the fix node were fed, and the truth, is
# kept in RECORDING, so `scripts/sim.sh replay` can rerun them over the same
# inputs in seconds

import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Range
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformException, TransformListener

ROUTE = [(2.0, 2.0), (-1.0, 2.0), (-1.0, -1.0), (2.0, -1.0), (2.0, 2.0),
         (-1.0, 2.0)]
SAMPLE_PERIOD = 0.1  # s
LEG_TIMEOUT = 40.0  # s
SETTLE = 3.0  # s, for TF and the filters before the first goal
RECORDING = "build_sim/check.jsonl"


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def gap(a, b):
    return math.hypot(a.position.x - b.position.x, a.position.y - b.position.y)


def seconds(stamp):
    return Time.from_msg(stamp).nanoseconds * 1e-9


def rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else math.nan


class Robot(Node):
    def __init__(self, ns):
        super().__init__("check", namespace=ns)
        self.ns = ns
        self.truth = self.odom = self.estimate = None
        # go_to_goal ends a goal with one explicit zero twist and then goes
        # quiet, so that zero after motion is the arrival
        self.moving = False
        self.arrived = False
        self.samples = {"global": [], "tf": [], "dead reckoning": [], "yaw": []}
        self.events = []
        self.anchors = set()

        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)

        self.create_subscription(PoseStamped, "sim/truth", self.on_truth, 10)
        self.create_subscription(PoseStamped, "sim/odom",
                                 lambda m: setattr(self, "odom", m.pose), 10)
        self.create_subscription(Odometry, "odometry/global",
                                 lambda m: setattr(self, "estimate", m.pose.pose), 10)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(TwistWithCovarianceStamped, "odom/twist",
                                 self.on_twist, qos_profile_sensor_data)
        self.create_subscription(Range, "uwb/range", self.on_range,
                                 qos_profile_sensor_data)
        for name in ("gyro_bias", "slip"):
            self.create_subscription(
                Float64, f"sim/{name}",
                lambda m, name=name: self.record(name, value=m.data), 10)

    # what uwb_fix was told to expect, so a replay solves with the same sigma
    def record_fix_settings(self, executor):
        client = self.create_client(GetParameters, "uwb_fix/get_parameters")

        if not client.wait_for_service(timeout_sec=2.0):
            return

        names = ["range_sigma", "range_bias"]
        future = client.call_async(GetParameters.Request(names=names))
        executor.spin_until_future_complete(future, timeout_sec=2.0)

        if future.result() is not None:
            values = [v.double_value for v in future.result().values]
            self.record("fixer", **dict(zip(names, values)))

    def record(self, kind, t=None, **fields):
        t = self.get_clock().now().nanoseconds * 1e-9 if t is None else t
        self.events.append({"ns": self.ns, "kind": kind, "t": t, **fields})

    def on_truth(self, msg):
        self.truth = msg.pose
        p = msg.pose
        self.record("truth", seconds(msg.header.stamp), x=p.position.x,
                    y=p.position.y, theta=yaw_of(p.orientation))

    def on_twist(self, msg):
        t = msg.twist.twist
        c = msg.twist.covariance
        self.record("twist", seconds(msg.header.stamp),
                    twist=[t.linear.x, t.linear.y, t.angular.z],
                    cov=[[c[0], c[1], c[5]], [c[6], c[7], c[11]],
                         [c[30], c[31], c[35]]])

    def on_range(self, msg):
        frame = msg.header.frame_id

        if frame not in self.anchors:
            try:
                a = self.tf.lookup_transform("map", frame, Time()).transform.translation
            except TransformException:
                return
            self.anchors.add(frame)
            self.record("anchor", frame=frame, x=a.x, y=a.y)

        self.record("range", seconds(msg.header.stamp), frame=frame, range=msg.range)

    def on_cmd_vel(self, msg):
        still = msg.linear.x == 0.0 and msg.angular.z == 0.0
        self.arrived = self.arrived or (self.moving and still)
        self.moving = not still

    def start_leg(self):
        self.moving = False
        self.arrived = False

    # sim/odom is in <ns>/odom, which starts on the robot's put-down pose with
    # no rotation, so the start offset puts it in map
    def sample(self, start):
        if None in (self.truth, self.odom, self.estimate):
            return

        try:
            t = self.tf.lookup_transform("map", f"{self.ns}/base_link", Time())
        except TransformException:
            return

        tf_pose = PoseStamped().pose
        tf_pose.position.x = t.transform.translation.x
        tf_pose.position.y = t.transform.translation.y
        dead = PoseStamped().pose
        dead.position.x = self.odom.position.x + start[0]
        dead.position.y = self.odom.position.y + start[1]

        self.samples["global"].append(gap(self.estimate, self.truth))
        self.samples["tf"].append(gap(tf_pose, self.truth))
        self.samples["dead reckoning"].append(gap(dead, self.truth))
        self.samples["yaw"].append(math.degrees(abs(wrap(
            yaw_of(self.estimate.orientation) - yaw_of(self.truth.orientation)))))


def spin_for(executor, seconds, each=None):
    end = time.monotonic() + seconds
    next_sample = time.monotonic()

    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.02)
        if each and time.monotonic() >= next_sample:
            each()
            next_sample += SAMPLE_PERIOD


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("namespaces", nargs="*")
    # sim.sh passes sdkconfig's, for when none are named
    parser.add_argument("--default-namespace", required=True)
    parser.add_argument("--legs", type=int, default=len(ROUTE),
                        help="drive only the first N legs, for a quicker run")
    # robots that do something stop between goals; one that never stops never
    # shows its gyro at rest
    parser.add_argument("--pause", type=float, default=0.0,
                        help="s to stand at each goal before the next")
    args = parser.parse_args()
    names = args.namespaces or [args.default_namespace]
    rclpy.init()
    robots = [Robot(ns) for ns in names]
    executor = SingleThreadedExecutor()
    for robot in robots:
        executor.add_node(robot)

    spin_for(executor, SETTLE)
    for r in robots:
        r.record_fix_settings(executor)
    starts = {r.ns: (r.truth.position.x, r.truth.position.y) if r.truth else (0.0, 0.0)
              for r in robots}
    goals = robots[0].create_publisher(PoseStamped, "/move_base_simple/goal", 10)

    def sample_all():
        for r in robots:
            r.sample(starts[r.ns])

    print(f"{'goal':>14}  {'robot':>8}  {'took':>6}  {'off goal':>8}  "
          f"{'global':>7}  {'dead rec':>8}  {'yaw':>6}", flush=True)

    for goal in ROUTE[:args.legs]:
        for r in robots:
            r.start_leg()

        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x, msg.pose.position.y = goal
        goals.publish(msg)
        began = time.monotonic()

        while (not all(r.arrived for r in robots) and
               time.monotonic() - began < LEG_TIMEOUT):
            spin_for(executor, SAMPLE_PERIOD, sample_all)

        took = time.monotonic() - began
        spin_for(executor, args.pause, sample_all)

        for r in robots:
            s = r.samples
            if not s["global"]:
                print(f"{str(goal):>14}  {r.ns:>8}  no estimate yet", flush=True)
                continue

            off = math.hypot(r.truth.position.x - goal[0], r.truth.position.y - goal[1])
            print(f"{str(goal):>14}  {r.ns:>8}  "
                  f"{'timeout' if not r.arrived else f'{took:5.1f}s':>6}  "
                  f"{off * 100:6.1f}cm  {s['global'][-1] * 100:5.1f}cm  "
                  f"{s['dead reckoning'][-1] * 100:6.1f}cm  "
                  f"{s['yaw'][-1]:5.1f}°", flush=True)

    print()
    for r in robots:
        s = r.samples
        if not s["global"]:
            print(f"{r.ns}: no odometry/global, sim/truth or map -> base_link heard")
            continue

        print(f"{r.ns} over {len(s['global'])} samples:")
        for key in ("global", "tf", "dead reckoning"):
            print(f"  {key:>14}  rms {rms(s[key]) * 100:5.1f}cm  "
                  f"max {max(s[key]) * 100:5.1f}cm")
        print(f"  {'yaw':>14}  rms {rms(s['yaw']):5.1f}°   max {max(s['yaw']):5.1f}°")

    with open(RECORDING, "w") as f:
        for r in robots:
            for event in r.events:
                f.write(json.dumps(event) + "\n")
    print(f"\nrecorded to {RECORDING}; scripts/sim.sh replay reruns the filters on it")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
