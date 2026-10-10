#!/usr/bin/env bash


# Drives /cmd_vel from the keyboard, through the micro-ROS agent the board
# connects to over UDP (scripts/agent.sh).
#
#   scripts/teleop.sh           start the agent if needed, then teleop
#   scripts/teleop.sh build     (re)build the teleop image
#
# The topic comes out of sdkconfig, so the board and the keyboard can't drift
# apart.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
IMAGE="swarm-teleop:latest"
DOCKERFILE="$REPO/docker/teleop/Dockerfile"
AGENT="swarm-agent"

die() { echo "teleop.sh: $*" >&2; exit 1; }
command -v docker >/dev/null 2>&1 || die "docker not found in PATH"
docker info >/dev/null 2>&1 || die "Docker daemon is not running"

NAMESPACE="$(sed -n 's/^CONFIG_SWARM_ROS_NAMESPACE="\{0,1\}\([^"]*\)"\{0,1\}$/\1/p' \
  "$REPO/sdkconfig" | tail -1)"
[ -n "$NAMESPACE" ] || die "no CONFIG_SWARM_ROS_NAMESPACE in sdkconfig"
TOPIC="/$NAMESPACE/cmd_vel"

build_image() {
  docker build "$@" -t "$IMAGE" -f "$DOCKERFILE" "$(dirname "$DOCKERFILE")"
}

if [ "${1:-}" = "build" ]; then
  build_image --no-cache
  exit 0
fi

docker image inspect "$IMAGE" >/dev/null 2>&1 || build_image

"$REPO/scripts/agent.sh" detach

# the agent's network namespace, not a bridge: XRCE-mapped topics are only
# discoverable from inside it
exec docker run --rm -it --network "container:$AGENT" "$IMAGE" bash -lc \
  ". /opt/ros/humble/setup.bash && exec ros2 run teleop_twist_keyboard \
   teleop_twist_keyboard --ros-args -r /cmd_vel:=$TOPIC"
