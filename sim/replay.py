#!/usr/bin/env python3

# reruns uwb_fix and the global filter over what a `scripts/sim.sh check` run
# recorded, through the same Fixer and Filter the nodes wrap, and scores them
# against the recorded truth. It takes seconds, and every variant sees exactly
# the same ranges and twists, so a difference is the change and not the run.
# It is open loop: the trajectory is the recorded one, where live the global
# estimate steers go_to_goal's goals. A change to the plant still needs a
# live run to record
#
#   scripts/sim.sh replay                           the recording as it was
#   scripts/sim.sh replay --set ambiguity=0.25      any Filter or Fixer setting
#   scripts/sim.sh replay --outage 50:80            no fixes for those seconds
#   scripts/sim.sh replay --twist-cov 100           the twist covariance scaled

import argparse
import bisect
import json
import math
import os
import sys
from collections import defaultdict
from dataclasses import fields, replace

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "host"))

from global_ekf import DEFAULTS, Filter, wrap  # noqa: E402
from uwb_fix import Fixer, Settings  # noqa: E402


# s before the bias error counts: the filter starts knowing nothing of it
SETTLED = 30.0


def setting(text):
    name, value = text.split("=", 1)
    return name, json.loads(value)


def window(text):
    start, end = text.split(":")
    return float(start), float(end)


def rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else math.nan


class Series:
    def __init__(self, events):
        self.times = [e["t"] for e in events]
        self.events = events

    def at(self, t):
        i = min(bisect.bisect_left(self.times, t), len(self.times) - 1)
        if i > 0 and abs(self.times[i - 1] - t) < abs(self.times[i] - t):
            i -= 1
        return self.events[i]


def replay(events, overrides, outage, twist_scale):
    by_kind = defaultdict(list)
    for e in events:
        by_kind[e["kind"]].append(e)

    anchors = {e["frame"]: (e["x"], e["y"]) for e in by_kind["anchor"]}
    truth = Series(sorted(by_kind["truth"], key=lambda e: e["t"]))
    true_bias = Series(sorted(by_kind["gyro_bias"], key=lambda e: e["t"])) \
        if by_kind["gyro_bias"] else None
    fix_settings = by_kind["fixer"][0] if by_kind["fixer"] else {}
    sigma = math.hypot(fix_settings.get("range_sigma", 0.05),
                       fix_settings.get("range_bias", 0.0))

    unknown = set(overrides) - set(DEFAULTS) - {f.name for f in fields(Settings)}
    if unknown:
        sys.exit(f"replay.py: no setting {', '.join(sorted(unknown))}")

    fixer = Fixer(sigma, replace(Settings(), **{
        k: v for k, v in overrides.items() if k in {f.name for f in fields(Settings)}}))

    stream = sorted(by_kind["twist"] + by_kind["range"], key=lambda e: e["t"])
    first = stream[0]["t"]
    start = truth.at(first)
    ekf = Filter((start["x"], start["y"], start["theta"]),
                 {**DEFAULTS, **{k: v for k, v in overrides.items() if k in DEFAULTS}})

    errors, yaws, in_outage, bias_errors = [], [], [], []
    outcomes = defaultdict(int)

    for e in stream:
        if e["kind"] == "twist":
            for outcome in ekf.on_twist(e["t"], e["twist"], np.asarray(e["cov"]) * twist_scale):
                outcomes[outcome] += 1
            real = truth.at(e["t"])
            error = math.hypot(ekf.state[0] - real["x"], ekf.state[1] - real["y"])
            errors.append(error)
            yaws.append(math.degrees(abs(wrap(ekf.state[2] - real["theta"]))))
            if true_bias and e["t"] - first >= SETTLED:
                bias_errors.append(ekf.state[3] - true_bias.at(e["t"])["value"])
            if outage and outage[0] <= e["t"] - first <= outage[1]:
                in_outage.append(error)
            continue

        result, dropped, _ = fixer.add(e["frame"], anchors[e["frame"]], e["range"], e["t"])

        if result is None:
            outcomes["no fix" if dropped else "too few anchors"] += 1
            continue

        if dropped:
            outcomes["fix with a drop"] += 1

        if outage and outage[0] <= e["t"] - first <= outage[1]:
            outcomes["fix lost to outage"] += 1
            continue

        position, cov = result
        ekf.on_fix(e["t"], position, np.asarray(cov))

    return {
        "errors": errors, "yaws": yaws, "bias_errors": bias_errors, "outage": in_outage, "outcomes": outcomes,
        "duration": stream[-1]["t"] - first, "state": ekf.state,
        "gyro_bias": [e["value"] for e in by_kind["gyro_bias"]],
        "slip": [e["value"] for e in by_kind["slip"]],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--recording", default="build_sim/check.jsonl")
    parser.add_argument("--set", type=setting, action="append", default=[],
                        metavar="NAME=VALUE", help="a Filter or Fixer setting")
    parser.add_argument("--outage", type=window, metavar="START:END",
                        help="s from the start with no fixes reaching the filter")
    parser.add_argument("--twist-cov", type=float, default=1.0, metavar="SCALE",
                        help="multiply the recorded twist covariance")
    args = parser.parse_args()

    with open(args.recording) as f:
        events = [json.loads(line) for line in f]

    by_ns = defaultdict(list)
    for e in events:
        by_ns[e["ns"]].append(e)

    for ns, ns_events in by_ns.items():
        out = replay(ns_events, dict(args.set), args.outage, args.twist_cov)
        e, y = out["errors"], out["yaws"]
        print(f"{ns}, {out['duration']:.0f} s:")
        print(f"  position  rms {rms(e) * 100:5.1f}cm  max {max(e) * 100:5.1f}cm")
        print(f"  heading   rms {rms(y):5.1f}°   max {max(y):5.1f}°")
        if out["outage"]:
            print(f"  outage    worst {max(out['outage']) * 100:5.1f}cm  "
                  f"at its end {out['outage'][-1] * 100:5.1f}cm")

        _, _, _, bias, slip = out["state"]
        if out["bias_errors"]:
            b = out["bias_errors"]
            print(f"  gyro bias {bias:+.4f} rad/s, true {out['gyro_bias'][-1]:+.4f}; "
                  f"error after {SETTLED:.0f} s rms {rms(b):.4f}, "
                  f"mean {sum(b) / len(b):+.4f}")
        if out["slip"]:
            print(f"  slip      {slip * 100:+5.1f}%, true mean "
                  f"{sum(out['slip']) / len(out['slip']) * 100:+5.1f}%")
        print("  fixes     " + ", ".join(f"{k} {v}" for k, v in sorted(out["outcomes"].items())))


if __name__ == "__main__":
    main()
